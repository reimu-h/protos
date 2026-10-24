#ifndef PROTOS_USER_USBKBD_XHCI_SIM_H_
#define PROTOS_USER_USBKBD_XHCI_SIM_H_

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <deque>
#include <iomanip>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "user/usbkbd/usb_hid.h"
#include "user/usbkbd/xhci_context.h"
#include "user/usbkbd/xhci_platform.h"
#include "user/usbkbd/xhci_regs.h"
#include "user/usbkbd/xhci_ring.h"
#include "user/usbkbd/xhci_sim_usb.h"

// A simulated xHCI controller and a fake platform for the host unit tests of
// the USB keyboard driver core (user/usbkbd/xhci_*.h). Host-only,
// single-threaded and deterministic.
//
// - `SimClock` is the time base of a test. Time passes only when the code
//   under test calls `FakePlatform::Relax()`, so bounded waits of seconds run
//   in microseconds and timeouts are exact.
// - `XhciSim` implements the core's MMIO window with a register file that has
//   the side effects of real hardware: Run/Stop -> HCH after a delay, HCRST ->
//   CNR sequence with defaults restored, RW1C/RsvdP/RsvdZ/RO semantics, the
//   USB Legacy Support semaphores with a firmware model that releases
//   ownership after N polls, after T ticks or never, SMI enables and events,
//   and a controller that firmware left running.
// - Every access is logged with a sequence number and tick, and protocol
//   violations (HCRST while running, programming before a completed reset,
//   operational writes while CNR = 1, firmware semaphore cleared too early,
//   RsvdP changes, ...) are recorded. Unexpected violations fail the test
//   when the simulator is destroyed; tests that provoke one call
//   AllowViolation().
// This header models the full xHCI lifecycle: capability/extended-capability
// discovery, BIOS-to-OS Legacy Support handoff, controller halt/reset,
// operational/runtime/doorbell programming, command/transfer/event rings,
// root-hub port status/reset state machines, USB device enumeration, and
// interrupt IN endpoint polling, together with protocol violation checks.

namespace protos::xhci_sim {

// Register layout and bits, transcribed from the xHCI 1.2 specification
// independently of user/usbkbd/xhci_regs.h, so that a wrong constant in the
// driver core is not mirrored here.
namespace spec {

// Capability registers (xHCI 5.3).
constexpr int64_t kCapLengthHciVersion = 0x00;
constexpr int64_t kHcsParams1 = 0x04;
constexpr int64_t kHcsParams2 = 0x08;
constexpr int64_t kHcsParams3 = 0x0C;
constexpr int64_t kHccParams1 = 0x10;
constexpr int64_t kDbOff = 0x14;
constexpr int64_t kRtsOff = 0x18;
constexpr int64_t kHccParams2 = 0x1C;

// Operational registers (xHCI 5.4), from the operational base.
constexpr int64_t kUsbCmd = 0x00;
constexpr int64_t kUsbSts = 0x04;
constexpr int64_t kPageSize = 0x08;
constexpr int64_t kDnCtrl = 0x14;
constexpr int64_t kCrcrLow = 0x18;
constexpr int64_t kCrcrHigh = 0x1C;
constexpr int64_t kDcbaapLow = 0x30;
constexpr int64_t kDcbaapHigh = 0x34;
constexpr int64_t kConfig = 0x38;
constexpr int64_t kPortRegisterSets = 0x400;
constexpr int64_t kPortRegisterSetSize = 0x10;

// USBCMD (xHCI 5.4.1).
constexpr uint32_t kCmdRunStop = 1u << 0;
constexpr uint32_t kCmdHcReset = 1u << 1;
constexpr uint32_t kCmdInterrupterEnable = 1u << 2;
constexpr uint32_t kCmdHostSystemErrorEnable = 1u << 3;
constexpr uint32_t kCmdLightHcReset = 1u << 7;
constexpr uint32_t kCmdSaveState = 1u << 8;
constexpr uint32_t kCmdRestoreState = 1u << 9;
constexpr uint32_t kCmdEnableWrapEvent = 1u << 10;
// RW bits: 0, 2, 3, 10, 11 (EU3S), 13 (CME), 14 (ETE), 15 (TSC_EN), 16.
constexpr uint32_t kCmdReadWriteMask = (1u << 0) | (1u << 2) | (1u << 3) |
                                       (1u << 10) | (1u << 11) | (1u << 13) |
                                       (1u << 14) | (1u << 15) | (1u << 16);
// RsvdP: bits 6:4, 12, 31:17.
constexpr uint32_t kCmdRsvdPMask = (0x7u << 4) | (1u << 12) | (0x7FFFu << 17);

// USBSTS (xHCI 5.4.2).
constexpr uint32_t kStsHalted = 1u << 0;
constexpr uint32_t kStsHostSystemError = 1u << 2;
constexpr uint32_t kStsEventInterrupt = 1u << 3;
constexpr uint32_t kStsPortChangeDetect = 1u << 4;
constexpr uint32_t kStsSaveRestoreError = 1u << 10;
constexpr uint32_t kStsNotReady = 1u << 11;
constexpr uint32_t kStsHcError = 1u << 12;
constexpr uint32_t kStsWriteOneToClearMask =
    (1u << 2) | (1u << 3) | (1u << 4) | (1u << 10);
// RsvdZ: bits 1, 7:5, 31:13.
constexpr uint32_t kStsRsvdZMask = (1u << 1) | (0x7u << 5) | (0x7FFFFu << 13);

// PORTSC (xHCI 5.4.8).
constexpr uint32_t kPortConnected = 1u << 0;
constexpr uint32_t kPortEnabled = 1u << 1;
constexpr uint32_t kPortReset = 1u << 4;
constexpr int kPortLinkStateShift = 5;
constexpr uint32_t kPortLinkStateMask = 0xFu << 5;
constexpr uint32_t kPortPower = 1u << 9;
constexpr int kPortSpeedShift = 10;
constexpr uint32_t kPortIndicatorMask = 0x3u << 14;
constexpr uint32_t kPortLinkWriteStrobe = 1u << 16;
constexpr uint32_t kPortConnectChange = 1u << 17;
constexpr uint32_t kPortWarmResetChange = 1u << 19;
constexpr uint32_t kPortOverCurrentChange = 1u << 20;
constexpr uint32_t kPortResetChange = 1u << 21;
constexpr uint32_t kPortLinkStateChange = 1u << 22;
// CSC, PEC, WRC, OCC, PRC, PLC, CEC: bits 17-23.
constexpr uint32_t kPortChangeMask = 0x7Fu << 17;
// WCE, WDE, WOE: bits 25-27.
constexpr uint32_t kPortWakeMask = 0x7u << 25;
constexpr uint32_t kPortWarmReset = 1u << 31;
// RsvdZ: bits 2, 28, 29.
constexpr uint32_t kPortRsvdZMask = (1u << 2) | (1u << 28) | (1u << 29);
constexpr uint32_t kLinkU0 = 0;
constexpr uint32_t kLinkRxDetect = 5;
constexpr uint32_t kLinkInactive = 6;
constexpr uint32_t kLinkPolling = 7;

// CRCR (xHCI 5.4.5), low dword.
constexpr uint32_t kCrcrCommandStop = 1u << 1;
constexpr uint32_t kCrcrCommandAbort = 1u << 2;
constexpr uint32_t kCrcrRunning = 1u << 3;

// Runtime registers (xHCI 5.5), from the runtime base.
constexpr int64_t kMfIndex = 0x00;
constexpr int64_t kInterrupterSets = 0x20;
constexpr int64_t kInterrupterSetSize = 0x20;
constexpr int64_t kIman = 0x00;
constexpr int64_t kImod = 0x04;
constexpr int64_t kErstSz = 0x08;
constexpr int64_t kErstBaLow = 0x10;
constexpr int64_t kErstBaHigh = 0x14;
constexpr int64_t kErdpLow = 0x18;
constexpr int64_t kErdpHigh = 0x1C;
constexpr uint32_t kImanPending = 1u << 0;
constexpr uint32_t kImanEnable = 1u << 1;
constexpr uint32_t kErdpBusy = 1u << 3;

// Extended capabilities (xHCI 7).
constexpr int kLegacySupportId = 1;
constexpr int kSupportedProtocolId = 2;
// USBLEGSUP (xHCI 7.1.1).
constexpr uint32_t kBiosOwned = 1u << 16;
constexpr uint32_t kOsOwned = 1u << 24;
// RsvdP: bits 23:17, 31:25.
constexpr uint32_t kLegSupRsvdPMask = (0x7Fu << 17) | (0x7Fu << 25);
// USBLEGCTLSTS (xHCI 7.1.2).
constexpr uint32_t kSmiOnOsOwnershipEnable = 1u << 13;
// Enables: bits 0, 4, 13, 14, 15.
constexpr uint32_t kSmiEnableMask =
    (1u << 0) | (1u << 4) | (1u << 13) | (1u << 14) | (1u << 15);
constexpr uint32_t kSmiOnEventInterrupt = 1u << 16;
constexpr uint32_t kSmiOnHostSystemError = 1u << 20;
constexpr uint32_t kSmiOnOsOwnershipChange = 1u << 29;
// Events (RW1C): bits 29, 30, 31.
constexpr uint32_t kSmiEventMask = (1u << 29) | (1u << 30) | (1u << 31);
// RsvdP: bits 3:1, 12:5, 19:17. RsvdZ: bits 28:21.
constexpr uint32_t kLegCtlStsRsvdPMask =
    (0x7u << 1) | (0xFFu << 5) | (0x7u << 17);
constexpr uint32_t kLegCtlStsRsvdZMask = 0xFFu << 21;

}  // namespace spec

// The time base of a test, shared by the fake platform and the simulated
// controllers. Starts at a non-zero tick so that "deadline = timeout" bugs
// show.
class SimClock {
 public:
  static constexpr int64_t kStartTick = 1000;
  // Register accesses and NowTicks() calls allowed without time passing.
  static constexpr int64_t kBusyLoopLimit = 1'000'000;

  int64_t now() const { return now_; }

  void Advance(const int64_t ticks) {
    if (ticks > 0) {
      now_ += ticks;
      activity_since_advance_ = 0;
    }
  }

  // Called on every simulated register access and NowTicks() call. Code that
  // polls without Relax() would hang the test; instead this reports a
  // failure and jumps the clock far ahead, so that every bounded wait ends.
  void NoteActivity() {
    if (++activity_since_advance_ > kBusyLoopLimit) {
      ADD_FAILURE() << "busy loop: " << kBusyLoopLimit
                    << " register accesses or clock reads without Relax()";
      busy_loop_detected_ = true;
      Advance(int64_t{1} << 20);
    }
  }

  bool busy_loop_detected() const { return busy_loop_detected_; }

 private:
  int64_t now_ = kStartTick;
  int64_t activity_since_advance_ = 0;
  bool busy_loop_detected_ = false;
};

// The longest log line the driver may emit, including its '\n'
// (`kMaxLogLineLength` in user/usbkbd/xhci_platform.h).
constexpr int kSimMaxLogLineLength = 128;

// Simulated guest physical DMA memory arena. High allocations start above
// 4 GiB (0x1'4000'0000) so any 32-bit address truncation is caught; low
// allocations (below_4gib = true) start at 0x2000'0000. Allocations are
// separated by a 4 KiB unmapped guard gap and filled with 0xAA poison bytes.
class SimDmaArena {
 public:
  static constexpr uint64_t kHighBase = 0x0000'0001'4000'0000ULL;
  static constexpr uint64_t kLowBase = 0x0000'0000'2000'0000ULL;
  static constexpr uint64_t kPageSize = 4096;

  usb::DmaBuffer Alloc(const int64_t size, const bool below_4gib) {
    if (size <= 0 || size > (1 << 20)) {
      return usb::DmaBuffer{};
    }
    const uint64_t aligned_size =
        (static_cast<uint64_t>(size) + kPageSize - 1) & ~(kPageSize - 1);
    uint64_t& cursor = below_4gib ? next_low_ : next_high_;
    const uint64_t phys = cursor;
    cursor += aligned_size + kPageSize;  // 4 KiB unmapped guard gap

    auto block = std::make_unique<Block>();
    block->phys = phys;
    block->size = aligned_size;
    block->below_4gib = below_4gib;
    block->data.assign(static_cast<size_t>(aligned_size), 0xAAu);
    void* const va = block->data.data();
    blocks_.push_back(std::move(block));
    return usb::DmaBuffer{va, phys};
  }

  uint8_t* Translate(const uint64_t phys,  //
                     const int64_t len,    //
                     const bool require_below_4gib) {
    if (len < 0) {
      return nullptr;
    }
    const uint64_t ulen = static_cast<uint64_t>(len);
    if (phys + ulen < phys) {
      return nullptr;
    }
    if (require_below_4gib &&
        (phys > 0xFFFF'FFFFULL || ulen > (0x1'0000'0000ULL - phys))) {
      return nullptr;
    }
    for (const std::unique_ptr<Block>& b : blocks_) {
      if (phys >= b->phys && (phys - b->phys) <= b->size &&
          ulen <= b->size - (phys - b->phys)) {
        return b->data.data() + static_cast<size_t>(phys - b->phys);
      }
    }
    return nullptr;
  }

 private:
  struct Block {
    uint64_t phys = 0;
    uint64_t size = 0;
    bool below_4gib = false;
    std::vector<uint8_t> data;
  };

  uint64_t next_high_ = kHighBase;
  uint64_t next_low_ = kLowBase;
  std::vector<std::unique_ptr<Block>> blocks_;
};

// A `Platform` for tests: time from a `SimClock`, captured log lines and
// console input. Log lines are checked: one complete line of at most 128
// bytes that starts with "[USBKBD] " and never contains "FAIL" or "[FATAL]".
class FakePlatform final : public usb::Platform {
 public:
  explicit FakePlatform(SimClock* const clock,
                        SimDmaArena* const arena = nullptr)
      : clock_(clock), arena_(arena) {
    usb::g_default_keyboard_locks.caps_lock = false;
  }

  usb::DmaBuffer DmaAlloc(const int64_t size, const bool below_4gib) override {
    ++dma_alloc_calls_;
    if (arena_ == nullptr) {
      return usb::DmaBuffer{};
    }
    if (fail_dma_after_ >= 0 && dma_alloc_calls_ > fail_dma_after_) {
      return usb::DmaBuffer{};
    }
    usb::DmaBuffer buf =
        arena_->Alloc(size, ignore_below_4gib_ ? false : below_4gib);
    if (bad_dma_alignment_ && buf.va != nullptr) {
      buf.phys |= 0x40ULL;
    }
    return buf;
  }

  int64_t NowTicks() override {
    ++now_calls_;
    clock_->NoteActivity();
    return clock_->now();
  }

  int64_t TicksPerSecond() override { return ticks_per_second_; }

  // Advances the clock by one tick every `relaxes_per_tick` calls.
  void Relax() override {
    ++relax_calls_;
    if (++relaxes_since_tick_ >= relaxes_per_tick_) {
      relaxes_since_tick_ = 0;
      clock_->Advance(1);
    }
  }

  void PushInput(const char* const bytes, const int count) override {
    if (bytes != nullptr && count > 0) {
      input_.append(bytes, static_cast<size_t>(count));
    }
  }

  void Log(const char* const line, const int length) override {
    if (line == nullptr || length < 1 || length > kSimMaxLogLineLength) {
      ADD_FAILURE() << "log line with invalid length " << length;
      return;
    }
    const std::string text(line, static_cast<size_t>(length));
    if (text.find('\n') != text.size() - 1) {
      ADD_FAILURE() << "log line is not one line ending with '\\n': " << text;
    }
    if (text.find('\0') != std::string::npos) {
      ADD_FAILURE() << "log line contains NUL: " << text;
    }
    if (text.rfind("[USBKBD] ", 0) != 0) {
      ADD_FAILURE() << "log line without the [USBKBD] prefix: " << text;
    }
    if (text.find("FAIL") != std::string::npos ||
        text.find("[FATAL]") != std::string::npos) {
      ADD_FAILURE() << "log line contains a failure marker: " << text;
    }
    logs_.push_back(text.substr(0, text.size() - 1));
  }

  void enable_dma() { arena_ = &owned_arena_; }
  void set_dma_arena(SimDmaArena* const arena) { arena_ = arena; }
  SimDmaArena* dma_arena() { return arena_; }
  void set_fail_dma_after(const int count) { fail_dma_after_ = count; }
  void set_bad_dma_alignment(const bool bad) { bad_dma_alignment_ = bad; }
  void set_ignore_below_4gib(const bool ignore) { ignore_below_4gib_ = ignore; }
  void ClearInput() { input_.clear(); }

  void set_ticks_per_second(const int64_t ticks_per_second) {
    ticks_per_second_ = ticks_per_second;
  }
  // > 1: polls are cheaper than ticks (time-based timeouts must still hold).
  void set_relaxes_per_tick(const int relaxes_per_tick) {
    relaxes_per_tick_ = relaxes_per_tick < 1 ? 1 : relaxes_per_tick;
  }

  const std::vector<std::string>& logs() const { return logs_; }
  bool LogContains(const std::string& needle) const {
    for (const std::string& line : logs_) {
      if (line.find(needle) != std::string::npos) {
        return true;
      }
    }
    return false;
  }
  const std::string& input() const { return input_; }
  int64_t relax_calls() const { return relax_calls_; }
  int64_t now_calls() const { return now_calls_; }
  int64_t dma_alloc_calls() const { return dma_alloc_calls_; }

 private:
  SimClock* clock_;
  SimDmaArena owned_arena_;
  SimDmaArena* arena_ = nullptr;
  int fail_dma_after_ = -1;
  bool bad_dma_alignment_ = false;
  bool ignore_below_4gib_ = false;
  int64_t ticks_per_second_ = 250;
  int relaxes_per_tick_ = 1;
  int relaxes_since_tick_ = 0;
  int64_t relax_calls_ = 0;
  int64_t now_calls_ = 0;
  int64_t dma_alloc_calls_ = 0;
  std::vector<std::string> logs_;
  std::string input_;
};

enum class Violation : uint8_t {
  kOutOfRangeAccess,            // Access outside the BAR
  kUnalignedAccess,             // Offset not a multiple of 4
  kWriteToReadOnly,             // Capability or other read-only register
  kWriteToReserved,             // Reserved register space
  kRsvdPChanged,                // A RsvdP field written with another value
  kRsvdZWrittenOne,             // A RsvdZ bit written as 1
  kResetWhileRunning,           // HCRST set while HCH = 0 (reset not done)
  kHcrstRetriggered,            // HCRST set again while a reset runs
  kAccessTooSoonAfterReset,     // Access within hcrst_quiet_ticks of HCRST
  kOpWriteWhileNotReady,        // Operational/doorbell write while CNR = 1
  kProgrammingBeforeReset,      // Setup write before a reset was observed done
  kHighDwordBeforeLow,          // 64-bit high dword not right after the low one
  kUnsupportedCommand,          // LHCRST, CSS or CRS set
  kBiosSemaphoreClearedEarly,   // HC BIOS Owned cleared before forcing is due
  kBiosSemaphoreSetBySoftware,  // HC BIOS Owned set by the driver
  kPortDisabledBySoftware,      // PORTSC.PED written as 1
  kPortPowerCleared,            // PORTSC.PP cleared with PPC = 1
  kUnmodeledWrite,              // Behavior not modeled yet (port resets)
  kDmaWhileBusMasterOff,        // DMA performed with Bus Master disabled
  kInvalidDmaAddress,           // Unmapped, misaligned, or >4GiB (AC64=0) DMA
  kRingModifiedWhileRunning,    // DCBAAP/CRCR/CONFIG/ERST modified while HCH=0
  kErstBaBeforeSize,            // ERSTBA written while ERSTSZ == 0
  kImanClearedBeforeEint,       // IMAN.IP cleared while USBSTS.EINT is still 1
  kDefaultAddressCollision,     // >1 USB 2.0 device at address 0 per root port
  kUsbAddressCollision,         // 2 reachable USB 2.0 devices share an address
  kCommandRingPointerCorrupted,  // CRCR high dword changed the stopped pointer
  kDcbaaChangedBeforeDisableSlot,  // DCBAA[slot] changed while slot enabled
  kDeviceContextModified,  // Software wrote an enabled slot's Output Context
  kDeviceContextShared,    // Two enabled slots use one Output Device Context
};

inline const char* ViolationName(const Violation violation) {
  switch (violation) {
    case Violation::kOutOfRangeAccess:
      return "out-of-range access";
    case Violation::kUnalignedAccess:
      return "unaligned access";
    case Violation::kWriteToReadOnly:
      return "write to read-only register";
    case Violation::kWriteToReserved:
      return "write to reserved space";
    case Violation::kRsvdPChanged:
      return "RsvdP bits changed";
    case Violation::kRsvdZWrittenOne:
      return "RsvdZ bit written as 1";
    case Violation::kResetWhileRunning:
      return "HCRST while running";
    case Violation::kHcrstRetriggered:
      return "HCRST while a reset is in progress";
    case Violation::kAccessTooSoonAfterReset:
      return "access too soon after HCRST";
    case Violation::kOpWriteWhileNotReady:
      return "operational write while CNR = 1";
    case Violation::kProgrammingBeforeReset:
      return "programming before a completed reset";
    case Violation::kHighDwordBeforeLow:
      return "64-bit high dword not written right after the low dword";
    case Violation::kUnsupportedCommand:
      return "unsupported USBCMD command";
    case Violation::kBiosSemaphoreClearedEarly:
      return "HC BIOS Owned cleared before the forcing timeout";
    case Violation::kBiosSemaphoreSetBySoftware:
      return "HC BIOS Owned set by software";
    case Violation::kPortDisabledBySoftware:
      return "PORTSC.PED written as 1";
    case Violation::kPortPowerCleared:
      return "PORTSC.PP cleared";
    case Violation::kUnmodeledWrite:
      return "write with behavior that is not modeled";
    case Violation::kDmaWhileBusMasterOff:
      return "DMA attempted while PCI Bus Master is disabled";
    case Violation::kInvalidDmaAddress:
      return "invalid or unmapped DMA physical address";
    case Violation::kRingModifiedWhileRunning:
      return "ring or config register modified while controller is running";
    case Violation::kErstBaBeforeSize:
      return "ERSTBA written while ERSTSZ is zero";
    case Violation::kImanClearedBeforeEint:
      return "IMAN.IP cleared before USBSTS.EINT";
    case Violation::kDefaultAddressCollision:
      return "two USB 2.0 devices at the default address below one root port";
    case Violation::kUsbAddressCollision:
      return "two reachable USB 2.0 devices share a USB address";
    case Violation::kCommandRingPointerCorrupted:
      return "CRCR high dword changed the stopped Command Ring pointer";
    case Violation::kDcbaaChangedBeforeDisableSlot:
      return "DCBAA entry changed before Disable Slot completed";
    case Violation::kDeviceContextModified:
      return "Output Device Context of an enabled slot modified by software";
    case Violation::kDeviceContextShared:
      return "two enabled slots share one Output Device Context";
  }
  return "unknown violation";
}

// How firmware reacts to the OS's ownership request (an HC OS Owned 0 -> 1
// write).
enum class BiosBehavior : uint8_t {
  kReleasesAfterPolls,  // Clears HC BIOS Owned on the Nth USBLEGSUP read
  kReleasesAfterTicks,  // Clears HC BIOS Owned T ticks after the request
  kNeverReleases,
};

// The firmware model of the USB Legacy Support capability.
struct LegacySupportConfig {
  bool bios_owned = true;
  bool os_owned = false;
  BiosBehavior behavior = BiosBehavior::kReleasesAfterPolls;
  int release_after_polls = 3;
  int64_t release_after_ticks = 0;
  // Firmware learns about the request through the "SMI on OS Ownership" SMI,
  // so it releases only if that enable (bit 13) was set at request time.
  bool release_requires_os_ownership_smi = true;
  bool bios_disables_smis_on_release = false;
  // Software may clear HC BIOS Owned itself (forced takeover) only this many
  // ticks after its request; earlier is a violation.
  int64_t force_floor_ticks = 250;
  // The values of USBLEGSUP's RsvdP bits.
  uint32_t legsup_rsvdp = 0;
  // Initial USBLEGCTLSTS: SMI enables 0, 4, 13, 14, 15, a RsvdP pattern
  // (bits 2, 7, 18) and SMI events 29, 30, 31 pending.
  uint32_t ctlsts = 0xE004E095u;
};

// One extended capability: its byte offset and dwords. The simulator fills
// in the ID/next fields ([15:0]) of dword 0 from `id` and the list order,
// unless `next_override` gives a raw "next" value (malformed lists).
struct ExtCap {
  int64_t offset = 0;
  int id = 0;
  std::vector<uint32_t> dwords;
  int next_override = -1;
};

inline ExtCap ProtocolCap(const int64_t offset,  //
                          const int major,       //
                          const int minor,       //
                          const int first_port,  //
                          const int port_count) {
  ExtCap cap;
  cap.offset = offset;
  cap.id = spec::kSupportedProtocolId;
  cap.dwords = {
      (static_cast<uint32_t>(major) << 24) |
          (static_cast<uint32_t>(minor) << 16),
      0x20425355u,  // "USB "
      static_cast<uint32_t>(first_port) |
          (static_cast<uint32_t>(port_count) << 8),
      0,
  };
  return cap;
}

inline ExtCap LegacyCap(const int64_t offset) {
  ExtCap cap;
  cap.offset = offset;
  cap.id = spec::kLegacySupportId;
  cap.dwords = {0, 0};
  return cap;
}

inline ExtCap VendorCap(const int64_t offset,  //
                        const int id,          //
                        const int dword_count) {
  ExtCap cap;
  cap.offset = offset;
  cap.id = id;
  cap.dwords.assign(static_cast<size_t>(dword_count < 1 ? 1 : dword_count),
                    0x5A5A0000u);
  return cap;
}

// The simulated controller's capabilities, the state firmware left, timing
// and misbehavior. The defaults are QEMU's qemu-xhci (11.1): halted, no USB
// Legacy Support capability, USB 3 ports 1-4 and USB 2 ports 5-8.
struct XhciSimConfig {
  int64_t bar_size = 0x4000;
  int cap_length = 0x40;
  int hci_version = 0x0100;
  int max_slots = 64;
  int max_interrupters = 16;
  int max_ports = 8;
  int ist = 0xF;
  int erst_max = 0;
  int max_scratchpads = 0;
  bool scratchpad_restore = false;
  bool ac64 = true;
  bool csz64 = false;
  bool ppc = false;
  // HCCPARAMS1 bits other than AC64, CSZ, PPC and xECP (QEMU: MaxPSASize 7).
  uint32_t hcc_params1_other = 0x7000;
  uint32_t hcc_params2 = 0;
  // DBOFF and RTSOFF as read (low bits may hold junk).
  uint32_t db_off_raw = 0x2000;
  uint32_t rts_off_raw = 0x1000;
  // Raw HCCPARAMS1.xECP (dwords); -1 = the first capability's offset.
  int xecp_override = -1;
  std::vector<ExtCap> ext_caps = {ProtocolCap(0x20, 2, 0, 5, 4),
                                  ProtocolCap(0x30, 3, 0, 1, 4)};
  // The first capability with ID 1 in `ext_caps` gets this firmware model.
  LegacySupportConfig legacy;

  // State left by firmware.
  bool running = false;           // R/S = 1, HCH = 0
  bool halt_in_progress = false;  // R/S = 0 but HCH = 0 until halted
  uint32_t usbcmd_extra = 0;      // More USBCMD bits (e.g. INTE, EWE)
  uint32_t usbcmd_rsvdp = 0;      // Values of USBCMD's RsvdP bits
  uint32_t usbsts_pending = 0;    // HSE, EINT, PCD, SRE set
  bool hce = false;               // USBSTS.HCE set
  uint64_t dcbaap = 0;
  uint64_t crcr = 0;
  uint32_t config = 0;
  uint32_t dnctrl = 0;
  uint32_t iman = 0;  // Interrupter 0
  uint32_t imod = 0;
  uint32_t erst_size = 0;
  uint64_t erst_base = 0;
  uint64_t erdp = 0;
  // PORTSC of ports 1.. at start; missing entries get the reset default.
  std::vector<uint32_t> initial_portsc;
  // Port Speed of the device attached to ports 1.. (0 = none), for the
  // PORTSC values after a reset.
  std::vector<int> attached_speed;

  // Timing in ticks, and misbehavior.
  int64_t power_on_not_ready_ticks = 0;  // CNR = 1 at start for this long
  bool power_on_not_ready_forever = false;
  int64_t halt_delay_ticks = 1;
  bool never_halts = false;
  int64_t reset_delay_ticks = 1;  // HCRST reads 1 for this long
  bool hcrst_never_clears = false;
  int64_t not_ready_after_reset_ticks = 1;  // Then CNR = 1 for this long
  bool not_ready_after_reset_forever = false;
  // > 0: any access within this many ticks after the HCRST write is a
  // violation (Intel controllers need 1 ms).
  int64_t hcrst_quiet_ticks = 0;
  // Not decoding: every read returns 0xFFFFFFFF and writes are dropped,
  // from the start or from `stop_decoding_after_ticks` (-1 = never) on.
  bool all_ones = false;
  int64_t stop_decoding_after_ticks = -1;
  // IMOD after a reset (QEMU 0; real controllers 0x0FA0).
  uint32_t imod_reset_value = 0;
  // Fault injection: HCH clears to 0 when the post-reset CNR wait ends.
  bool runs_after_reset = false;
  // Phase-2 configuration and fault injection.
  uint32_t page_size_raw = 1;
  int64_t start_delay_ticks = 0;
  bool never_starts = false;
  int64_t port_reset_ticks = 3;
  bool port_reset_never_completes = false;
  bool drop_port_status_change_events = false;
  bool no_slots_available = false;
  // The next command fetched never completes, until CRCR.CA aborts it.
  bool command_hang = false;
  // Commands complete this long after they are fetched (0: at once).
  int64_t command_delay_ticks = 0;
  // CRCR.CA has no effect: the ring keeps running (CRR = 1).
  bool command_abort_hangs = false;
  // Enable Slot hands out slot IDs round-robin instead of the lowest free one
  // (both are legal, xHCI 4.6.3).
  bool round_robin_slots = false;
};

// QEMU's qemu-xhci as the driver finds it after SeaBIOS: running (SeaBIOS
// polls its keyboard with the controller running), an event pending, stale
// ring pointers, and the keyboard on port 5 enabled.
inline XhciSimConfig SeaBiosLeftRunningConfig() {
  XhciSimConfig config;
  config.running = true;
  config.usbsts_pending = spec::kStsEventInterrupt;
  config.dcbaap = 0x7FF0'0000;
  config.crcr = 0x7FF0'1001;
  config.config = 4;
  config.erst_size = 1;
  config.erst_base = 0x7FF0'2000;
  config.erdp = 0x7FF0'3000;
  config.iman = spec::kImanEnable;
  config.attached_speed = {0, 0, 0, 0, 3};
  config.initial_portsc = {0, 0, 0, 0, 0};
  for (int port = 0; port < 4; ++port) {
    config.initial_portsc[port] =
        spec::kPortPower | (spec::kLinkRxDetect << spec::kPortLinkStateShift);
  }
  config.initial_portsc[4] = spec::kPortConnected | spec::kPortEnabled |
                             spec::kPortPower | (3u << spec::kPortSpeedShift);
  return config;
}

// QEMU's qemu-xhci as the driver finds it after OVMF: halted, an event
// pending and stale ring pointers still programmed.
inline XhciSimConfig OvmfLeftHaltedConfig() {
  XhciSimConfig config;
  config.usbsts_pending = spec::kStsEventInterrupt;
  config.dcbaap = 0xBF00'0000;
  config.crcr = 0xBF00'1001;
  config.config = 8;
  config.erst_size = 1;
  config.erst_base = 0xBF00'2000;
  config.erdp = 0xBF00'3000;
  return config;
}

// A PC controller: 64 KiB BAR, version 1.10, 64-byte contexts, port power
// control, 35 scratchpads (Hi 1, Lo 3), DBOFF/RTSOFF elsewhere, USB 2 ports
// 1-10 and USB 3 ports 11-16, and a USB Legacy Support capability owned by
// firmware (released after 3 polls) ahead of the protocol and vendor
// capabilities. Needs 1 ms without register access after HCRST.
inline XhciSimConfig IntelLikeConfig() {
  XhciSimConfig config;
  config.bar_size = 0x10000;
  config.cap_length = 0x80;
  config.hci_version = 0x0110;
  config.max_slots = 64;
  config.max_interrupters = 8;
  config.max_ports = 16;
  config.ist = 0x1;
  config.erst_max = 0xF;
  config.max_scratchpads = 35;
  config.scratchpad_restore = true;
  config.csz64 = true;
  config.ppc = true;
  config.hcc_params1_other = 0x0200;
  config.hcc_params2 = 0x3F;
  config.db_off_raw = 0x3000;
  config.rts_off_raw = 0x2000;
  config.ext_caps = {LegacyCap(0x8000), ProtocolCap(0x8020, 2, 0, 1, 10),
                     ProtocolCap(0x8040, 3, 0x10, 11, 6),
                     VendorCap(0x8070, 0xC0, 4)};
  config.hcrst_quiet_ticks = 1;
  config.imod_reset_value = 0x0FA0;
  return config;
}

// One logged register access.
struct Access {
  int64_t seq = 0;
  int64_t tick = 0;
  bool write = false;
  int64_t offset = 0;
  uint32_t value = 0;
};

struct ViolationRecord {
  Violation kind = Violation::kOutOfRangeAccess;
  int64_t seq = 0;
  int64_t tick = 0;
  int64_t offset = 0;
  uint32_t value = 0;
  std::string detail;
};

// `value` in lowercase hexadecimal, at least `width` digits.
inline std::string HexString(const uint64_t value, const int width) {
  std::ostringstream stream;
  stream << std::hex << std::setw(width) << std::setfill('0') << value;
  return stream.str();
}

inline void PrintTo(const ViolationRecord& record, std::ostream* const os) {
  *os << "#" << record.seq << " t=" << record.tick << " @0x"
      << HexString(static_cast<uint64_t>(record.offset), 1) << " value 0x"
      << HexString(record.value, 8) << ": " << ViolationName(record.kind);
  if (!record.detail.empty()) {
    *os << " (" << record.detail << ")";
  }
}

inline void PrintTo(const Violation violation, std::ostream* const os) {
  *os << ViolationName(violation);
}

class XhciSim final : public usb::MmioWindow {
 public:
  XhciSim(SimClock* const clock, const XhciSimConfig& config)
      : clock_(clock), config_(config), start_tick_(clock->now()) {
    BuildExtendedCapabilities();
    ports_.resize(
        static_cast<size_t>(config_.max_ports > 0 ? config_.max_ports : 0));
    for (int port = 1; port <= static_cast<int>(ports_.size()); ++port) {
      Port& state = ports_[static_cast<size_t>(port - 1)];
      state.portsc =
          static_cast<size_t>(port - 1) < config_.initial_portsc.size()
              ? config_.initial_portsc[static_cast<size_t>(port - 1)]
              : DefaultPortsc(port);
    }
    interrupters_.resize(static_cast<size_t>(
        config_.max_interrupters > 0 ? config_.max_interrupters : 0));
    if (!interrupters_.empty()) {
      Interrupter& first = interrupters_[0];
      first.iman = config_.iman;
      first.imod = config_.imod;
      first.erst_size = config_.erst_size;
      first.erst_base = config_.erst_base;
      first.erdp = config_.erdp;
    }
    usbcmd_ = (config_.usbcmd_extra & spec::kCmdReadWriteMask) |
              (config_.running ? spec::kCmdRunStop : 0);
    usbsts_pending_ = config_.usbsts_pending & spec::kStsWriteOneToClearMask;
    hce_ = config_.hce;
    if (config_.running) {
      run_state_ = RunState::kRunning;
      awaiting_halt_observation_ = true;
    } else if (config_.halt_in_progress) {
      run_state_ = RunState::kHalting;
      halt_due_tick_ = start_tick_ + config_.halt_delay_ticks;
      awaiting_halt_observation_ = true;
    }
    dcbaap_ = config_.dcbaap;
    crcr_ = config_.crcr;
    config_register_ = config_.config;
    dnctrl_ = config_.dnctrl;
    bios_owned_ = config_.legacy.bios_owned;
    os_owned_ = config_.legacy.os_owned;
    ctlsts_ = config_.legacy.ctlsts;
    if (legacy_cap_ >= 0 && bios_owned_ && os_owned_) {
      // The OS asked before (e.g. an earlier driver instance).
      RecordOwnershipRequest(0);
    }
  }

  // Fails the test if violations happened that it did not allow.
  ~XhciSim() {
    std::string unexpected;
    for (const ViolationRecord& record : violations_) {
      if (!IsAllowed(record.kind)) {
        unexpected += "\n  " + ViolationText(record);
      }
    }
    if (!unexpected.empty()) {
      ADD_FAILURE() << "unexpected xHCI simulator violations:" << unexpected
                    << "\naccess log:\n"
                    << TraceString();
    }
  }

  XhciSim(const XhciSim&) = delete;
  XhciSim& operator=(const XhciSim&) = delete;

  uint32_t Read32(const int64_t offset) override {
    clock_->NoteActivity();
    Update();
    const int64_t seq = next_seq_++;
    uint32_t value = usb::kMmioAllOnes;
    if (CheckAccess(seq, offset, value)) {
      CheckQuietPeriod(seq, offset, value);
      if (decoding_) {
        value = ReadRegister(offset, /*side_effects=*/true, seq);
      }
    }
    accesses_.push_back(Access{seq, clock_->now(), false, offset, value});
    previous_access_ = accesses_.back();
    return value;
  }

  void Write32(const int64_t offset, const uint32_t value) override {
    clock_->NoteActivity();
    Update();
    const int64_t seq = next_seq_++;
    const Access previous = previous_access_;
    accesses_.push_back(Access{seq, clock_->now(), true, offset, value});
    previous_access_ = accesses_.back();
    if (!CheckAccess(seq, offset, value)) {
      return;
    }
    CheckQuietPeriod(seq, offset, value);
    if (!decoding_) {
      return;
    }
    CheckHighDword(seq, offset, value, previous);
    WriteRegister(offset, value, seq);
  }

  int64_t Size() const override { return config_.bar_size; }

  // The register at `offset` as the next read would return it, without side
  // effects, logging or violations.
  uint32_t Peek(const int64_t offset) {
    Update();
    if (offset < 0 || offset + 4 > config_.bar_size || (offset & 3) != 0 ||
        !decoding_) {
      return usb::kMmioAllOnes;
    }
    return ReadRegister(offset, /*side_effects=*/false, 0);
  }

  // Register offsets.
  int64_t op(const int64_t reg) const { return config_.cap_length + reg; }
  int64_t portsc(const int port) const {
    return op(spec::kPortRegisterSets) +
           spec::kPortRegisterSetSize * (port - 1);
  }
  int64_t rt_base() const { return config_.rts_off_raw & ~0x1Fu; }
  int64_t db_base() const { return config_.db_off_raw & ~0x3u; }
  int64_t interrupter(const int index, const int64_t reg) const {
    return rt_base() + spec::kInterrupterSets +
           spec::kInterrupterSetSize * index + reg;
  }
  int64_t doorbell(const int slot) const { return db_base() + 4 * slot; }
  // Offset of the USB Legacy Support capability, 0 if there is none.
  int64_t legacy_offset() const {
    return legacy_cap_ >= 0
               ? config_.ext_caps[static_cast<size_t>(legacy_cap_)].offset
               : 0;
  }
  int64_t legsup_offset() const { return legacy_offset(); }
  int64_t legctlsts_offset() const { return legacy_offset() + 4; }

  // Controller state.
  bool halted() {
    Update();
    return run_state_ == RunState::kHalted;
  }
  bool resetting() {
    Update();
    return reset_phase_ == ResetPhase::kResetting;
  }
  bool not_ready() {
    Update();
    return NotReady();
  }
  int reset_count() const { return reset_count_; }
  // Sequence numbers (-1 = never): the first USBSTS read that showed the
  // halt of a controller that ran, the latest accepted HCRST write, and the
  // USBSTS read that completed the observation of that reset (CNR = 0 after
  // a USBCMD read showed HCRST = 0).
  int64_t halt_observed_seq() const { return halt_observed_seq_; }
  int64_t reset_write_seq() const { return reset_write_seq_; }
  int64_t reset_write_tick() const { return reset_write_tick_; }
  int64_t reset_observed_seq() const { return reset_observed_seq_; }
  bool reset_observed() const { return reset_observed_; }
  int64_t start_tick() const { return start_tick_; }

  // USB Legacy Support state.
  bool bios_owned() {
    Update();
    return bios_owned_;
  }
  bool os_owned() const { return os_owned_; }
  // The driver cleared HC BIOS Owned itself (after the forcing floor).
  bool forced() const { return forced_; }
  // Firmware cleared HC BIOS Owned in response to the request.
  bool released_by_bios() {
    Update();
    return released_by_bios_;
  }
  int64_t request_tick() const { return request_tick_; }
  int64_t request_seq() const { return request_seq_; }
  int legsup_reads_since_request() const { return polls_since_request_; }
  uint32_t legsup() { return Peek(legsup_offset()); }
  uint32_t legctlsts() { return Peek(legctlsts_offset()); }

  // The access log.
  const std::vector<Access>& accesses() const { return accesses_; }
  std::vector<Access> Writes() const {
    std::vector<Access> writes;
    for (const Access& access : accesses_) {
      if (access.write) {
        writes.push_back(access);
      }
    }
    return writes;
  }
  std::vector<Access> WritesTo(const int64_t offset) const {
    std::vector<Access> writes;
    for (const Access& access : accesses_) {
      if (access.write && access.offset == offset) {
        writes.push_back(access);
      }
    }
    return writes;
  }
  // The sequence number of the first write to `offset` whose value has all
  // bits of `bits` set (-1 = none).
  int64_t FirstWriteSeq(const int64_t offset, const uint32_t bits = 0) const {
    for (const Access& access : accesses_) {
      if (access.write && access.offset == offset &&
          (access.value & bits) == bits) {
        return access.seq;
      }
    }
    return -1;
  }
  int CountReads(const int64_t offset) const {
    int count = 0;
    for (const Access& access : accesses_) {
      if (!access.write && access.offset == offset) {
        ++count;
      }
    }
    return count;
  }
  // The access log, one access per line (the last `max_lines`).
  std::string TraceString(const int max_lines = 400) const {
    std::string trace;
    const int64_t total = static_cast<int64_t>(accesses_.size());
    const int64_t first = total > max_lines ? total - max_lines : 0;
    if (first > 0) {
      trace += "  ... " + std::to_string(first) + " earlier accesses\n";
    }
    for (int64_t i = first; i < total; ++i) {
      const Access& access = accesses_[static_cast<size_t>(i)];
      trace += "  #" + std::to_string(access.seq) +
               " t=" + std::to_string(access.tick) +
               (access.write ? " W 0x" : " R 0x") +
               HexString(static_cast<uint64_t>(access.offset), 5) + " = 0x" +
               HexString(access.value, 8) + " " +
               DescribeOffset(access.offset) + "\n";
    }
    return trace;
  }

  // Protocol violations.
  const std::vector<ViolationRecord>& violations() const { return violations_; }
  bool HasViolation(const Violation kind) const {
    return CountViolations(kind) > 0;
  }
  int CountViolations(const Violation kind) const {
    int count = 0;
    for (const ViolationRecord& record : violations_) {
      if (record.kind == kind) {
        ++count;
      }
    }
    return count;
  }
  // Violations of this kind don't fail the test at destruction.
  void AllowViolation(const Violation kind) { allowed_.push_back(kind); }

  struct SimEndpoint {
    uint8_t state = 0;
    uint8_t type = 0;
    uint8_t interval = 0;
    uint8_t max_burst = 0;
    uint16_t max_packet_size = 0;
    uint16_t avg_trb_len = 0;
    uint32_t max_esit = 0;
    uint64_t dequeue = 0;
    uint32_t dcs = 1;
    bool doorbell_rung = false;
    // The data toggle (SuperSpeed: sequence number parity) the host expects
    // next on an IN endpoint; Reset Endpoint with TSP = 0 clears it.
    uint8_t toggle = 0;
  };

  struct SimSlot {
    bool enabled = false;
    uint8_t slot_state = 0;
    int port = 0;
    int speed = 0;
    int context_entries = 0;
    uint32_t route_string = 0;
    bool hub = false;
    bool mtt = false;
    int num_ports = 0;
    uint8_t parent_hub_slot_id = 0;
    uint8_t parent_port_number = 0;
    uint8_t ttt = 0;
    uint64_t out_ctx_phys = 0;
    SimEndpoint ep[32] = {};
    // The Output Device Context as the xHC last wrote it (the driver only
    // reads it while the slot is enabled), from Address Device on.
    std::vector<uint8_t> out_ctx_shadow;
  };

  void set_dma_arena(SimDmaArena* const arena) { dma_ = arena; }
  void set_bus_master(const bool enabled) { bus_master_ = enabled; }
  bool bus_master() const { return bus_master_; }

  void set_no_slots_available(const bool v) { config_.no_slots_available = v; }
  void set_command_hang(const bool v) { config_.command_hang = v; }
  void set_command_abort_hangs(const bool v) {
    config_.command_abort_hangs = v;
  }
  void set_drop_port_status_change_events(const bool v) {
    config_.drop_port_status_change_events = v;
  }
  void set_port_reset_never_completes(const bool v) {
    config_.port_reset_never_completes = v;
  }
  void StopDecoding() {
    config_.all_ones = true;
    decoding_ = false;
  }

  // The link of USB 3 port `port` drops to SS.Inactive (e.g. after link
  // errors): the port is disabled until a Warm Reset (xHCI 4.19.1.2).
  void SetUsb3LinkInactive(const int port) {
    if (port < 1 || port > static_cast<int>(ports_.size())) {
      return;
    }
    Update();
    Port& state = ports_[static_cast<size_t>(port - 1)];
    if ((state.portsc & spec::kPortConnected) == 0) {
      return;
    }
    state.portsc = (state.portsc & ~(spec::kPortLinkStateMask |
                                     spec::kPortEnabled | spec::kPortReset)) |
                   (spec::kLinkInactive << spec::kPortLinkStateShift) |
                   spec::kPortLinkStateChange;
    state.reset_due_tick = 0;
    if (run_state_ == RunState::kRunning) {
      usbsts_pending_ |= spec::kStsPortChangeDetect;
      if (!config_.drop_port_status_change_events) {
        PushPortStatusChangeEvent(port);
      }
      FlushPendingEvents();
    }
  }

  // A transient over-current on root port `port` (with PPC = 1) powers the
  // port off (PP = 0) and sets OCC = 1 (xHCI 4.19.4).
  void PulsePortOverCurrent(const int port) {
    if (port < 1 || port > static_cast<int>(ports_.size()) || !config_.ppc) {
      return;
    }
    Update();
    Port& state = ports_[static_cast<size_t>(port - 1)];
    if (state.device != nullptr) {
      state.device->OnBusReset();
    }
    state.portsc = (spec::kLinkRxDetect << spec::kPortLinkStateShift) |
                   spec::kPortOverCurrentChange;
    state.power_on_due_tick = 0;
    state.reset_due_tick = 0;
    CheckUsb2Segment(port, -1, 0, 0);
    if (run_state_ == RunState::kRunning) {
      usbsts_pending_ |= spec::kStsPortChangeDetect;
      if (!config_.drop_port_status_change_events) {
        PushPortStatusChangeEvent(port);
      }
      FlushPendingEvents();
    }
  }

  void AttachDevice(const int port, SimUsbDevice* const device) {
    if (port < 1 || port > static_cast<int>(ports_.size())) {
      return;
    }
    Update();
    Port& state = ports_[static_cast<size_t>(port - 1)];
    state.device = device;
    if (device != nullptr) {
      device->OnBusReset();
    }
    if ((state.portsc & spec::kPortPower) != 0) {
      state.portsc = PoweredPortsc(port);
      if (run_state_ == RunState::kRunning &&
          (state.portsc & spec::kPortConnectChange) != 0) {
        usbsts_pending_ |= spec::kStsPortChangeDetect;
        if (!config_.drop_port_status_change_events) {
          PushPortStatusChangeEvent(port);
        }
        FlushPendingEvents();
      }
    }
  }

  void DetachDevice(const int port) {
    if (port < 1 || port > static_cast<int>(ports_.size())) {
      return;
    }
    Update();
    Port& state = ports_[static_cast<size_t>(port - 1)];
    state.device = nullptr;
    state.reset_due_tick = 0;
    if (static_cast<size_t>(port - 1) < config_.attached_speed.size()) {
      config_.attached_speed[static_cast<size_t>(port - 1)] = 0;
    }
    if ((state.portsc & spec::kPortPower) != 0) {
      state.portsc = spec::kPortPower | spec::kPortConnectChange |
                     (spec::kLinkRxDetect << spec::kPortLinkStateShift);
      if (run_state_ == RunState::kRunning) {
        usbsts_pending_ |= spec::kStsPortChangeDetect;
        if (!config_.drop_port_status_change_events) {
          PushPortStatusChangeEvent(port);
        }
        FlushPendingEvents();
      }
    }
  }

  void TriggerConnectBounce(const int port) {
    if (port < 1 || port > static_cast<int>(ports_.size())) {
      return;
    }
    Update();
    Port& state = ports_[static_cast<size_t>(port - 1)];
    if ((state.portsc & spec::kPortPower) != 0) {
      state.portsc |= spec::kPortConnectChange;
      if (run_state_ == RunState::kRunning) {
        usbsts_pending_ |= spec::kStsPortChangeDetect;
        if (!config_.drop_port_status_change_events) {
          PushPortStatusChangeEvent(port);
        }
        FlushPendingEvents();
      }
    }
  }

  void InjectHostSystemError() {
    Update();
    usbsts_pending_ |= spec::kStsHostSystemError;
    run_state_ = RunState::kHalted;
    cmd_ring_running_ = false;
    cmd_executing_ = false;
    if ((usbcmd_ & (spec::kCmdInterrupterEnable |
                    spec::kCmdHostSystemErrorEnable)) != 0) {
      ++irq_count_;
      irq_asserted_ = true;
    }
  }

  void InjectHostControllerError() {
    Update();
    hce_ = true;
    run_state_ = RunState::kHalted;
    cmd_ring_running_ = false;
    cmd_executing_ = false;
    if ((usbcmd_ & spec::kCmdInterrupterEnable) != 0) {
      ++irq_count_;
      irq_asserted_ = true;
    }
  }

  void InjectEventRingFull() {
    Update();
    usb::Trb ev{};
    ev.dw2 = static_cast<uint32_t>(usb::kCcEventRingFull) << 24;
    ev.dw3 = static_cast<uint32_t>(usb::kTrbHostControllerEvent)
             << usb::kTrbTypeShift;
    pending_events_.push_back(ev);
    FlushPendingEvents();
  }

  void PushRawEvent(const usb::Trb& ev) {
    Update();
    pending_events_.push_back(ev);
    FlushPendingEvents();
  }

  void Step() {
    Update();
    if (run_state_ == RunState::kRunning && decoding_) {
      ServiceInterruptEndpoints();
      FlushPendingEvents();
    }
  }

  bool irq_pending() {
    Step();
    return irq_asserted_;
  }

  bool ConsumeIrq() {
    Step();
    if (!irq_asserted_) {
      return false;
    }
    irq_asserted_ = false;
    return true;
  }

  int irq_count() const { return irq_count_; }

  int enabled_slot_count() const {
    int count = 0;
    for (int i = 1; i < static_cast<int>(slots_.size()); ++i) {
      if (slots_[static_cast<size_t>(i)].enabled) {
        ++count;
      }
    }
    return count;
  }

  const SimSlot& slot(const int slot_id) const {
    return slots_[static_cast<size_t>(slot_id)];
  }
  int slot_capacity() const { return static_cast<int>(slots_.size()); }

  // The most USB 2.0 devices seen at the default address 0 at once below one
  // root port.
  int max_default_state_devices() const { return max_default_state_devices_; }
  // USB 2.0 devices at the default address 0 below root port `port` now.
  int default_state_devices(const int port) const {
    return CountAddress(Usb2SegmentAddresses(port), 0);
  }
  // Stop Endpoint commands received.
  int stop_endpoint_count() const { return stop_endpoint_count_; }
  // Slot IDs in the order Disable Slot commands disabled them.
  const std::vector<int>& disable_slot_log() const { return disable_slot_log_; }
  // Interrupt IN data packets the host dropped as retransmissions because
  // their data toggle (sequence number) was not the expected one.
  int dropped_in_packets() const { return dropped_in_packets_; }
  // CRCR.CRR: the Command Ring runs (from doorbell 0 until it is stopped).
  bool command_ring_running() const { return cmd_ring_running_; }
  // A command has been fetched and has not completed (delayed or hung).
  bool command_in_progress() const { return cmd_executing_; }

  // A readable name of the register at `offset`, for traces.
  std::string DescribeOffset(const int64_t offset) const {
    if (offset < 0 || offset + 4 > config_.bar_size) {
      return "(outside the BAR)";
    }
    const Location location = Locate(offset);
    const std::string reg = HexString(static_cast<uint64_t>(location.reg), 1);
    switch (location.region) {
      case Region::kExtCap: {
        const ExtCap& cap =
            config_.ext_caps[static_cast<size_t>(location.index)];
        return "ext cap " + std::to_string(location.index) + " id " +
               std::to_string(cap.id) + " +0x" + reg;
      }
      case Region::kCapability:
        return "cap +0x" + reg;
      case Region::kOperational:
        return std::string("op ") + OpRegisterName(location.reg);
      case Region::kPort:
        return "port " + std::to_string(location.index) + " +0x" + reg;
      case Region::kRuntime:
        return "runtime +0x" + reg;
      case Region::kDoorbell:
        return "doorbell " + std::to_string(location.index);
      case Region::kReserved:
        break;
    }
    return "reserved";
  }

 private:
  enum class RunState : uint8_t { kHalted, kRunning, kHalting };
  enum class ResetPhase : uint8_t { kIdle, kResetting, kNotReady };
  enum class Region : uint8_t {
    kCapability,
    kExtCap,
    kOperational,
    kPort,
    kRuntime,
    kDoorbell,
    kReserved,
  };
  // Where an offset is: region, index (ext cap, port 1.., interrupter or
  // doorbell) and register offset within it.
  struct Location {
    Region region = Region::kReserved;
    int index = 0;
    int64_t reg = 0;
  };
  struct Port {
    uint32_t portsc = 0;
    uint32_t pmsc = 0;
    uint32_t li = 0;
    uint32_t hlpmc = 0;
    SimUsbDevice* device = nullptr;
    int64_t power_on_due_tick = 0;
    int64_t reset_due_tick = 0;
    // The reset in progress is a Warm Reset (WPR) of a USB 3 port.
    bool warm_reset = false;
    // USB 2.0 segment census below this root port (CheckUsb2Segment()).
    int default_state_devices = 0;
    bool address_collision = false;
  };
  struct Interrupter {
    uint32_t iman = 0;
    uint32_t imod = 0;
    uint32_t erst_size = 0;
    uint64_t erst_base = 0;
    uint64_t erdp = 0;
  };

  static std::string ViolationText(const ViolationRecord& record) {
    std::ostringstream stream;
    PrintTo(record, &stream);
    return stream.str();
  }

  static const char* OpRegisterName(const int64_t reg) {
    switch (reg) {
      case spec::kUsbCmd:
        return "USBCMD";
      case spec::kUsbSts:
        return "USBSTS";
      case spec::kPageSize:
        return "PAGESIZE";
      case spec::kDnCtrl:
        return "DNCTRL";
      case spec::kCrcrLow:
        return "CRCR";
      case spec::kCrcrHigh:
        return "CRCR+4";
      case spec::kDcbaapLow:
        return "DCBAAP";
      case spec::kDcbaapHigh:
        return "DCBAAP+4";
      case spec::kConfig:
        return "CONFIG";
      default:
        return "reserved";
    }
  }

  void BuildExtendedCapabilities() {
    const int ext_count = static_cast<int>(config_.ext_caps.size());
    for (int i = 0; i < ext_count; ++i) {
      ExtCap& cap = config_.ext_caps[static_cast<size_t>(i)];
      if (cap.dwords.empty()) {
        cap.dwords.push_back(0);
      }
      int next = 0;
      if (cap.next_override >= 0) {
        next = cap.next_override;
      } else if (i + 1 < ext_count) {
        const int64_t distance =
            config_.ext_caps[static_cast<size_t>(i + 1)].offset - cap.offset;
        if (distance <= 0 || distance % 4 != 0 || distance / 4 > 0xFF) {
          ADD_FAILURE() << "simulator config: extended capability " << i + 1
                        << " can't follow capability " << i;
        }
        next = static_cast<int>(distance / 4);
      }
      cap.dwords[0] = (cap.dwords[0] & 0xFFFF0000u) |
                      (static_cast<uint32_t>(next & 0xFF) << 8) |
                      static_cast<uint32_t>(cap.id & 0xFF);
      if (cap.id == spec::kLegacySupportId && legacy_cap_ < 0) {
        legacy_cap_ = i;
        if (cap.dwords.size() < 2) {
          cap.dwords.resize(2, 0);
        }
      }
    }
  }

  uint32_t XecpField() const {
    if (config_.xecp_override >= 0) {
      return static_cast<uint32_t>(config_.xecp_override) & 0xFFFF;
    }
    if (config_.ext_caps.empty()) {
      return 0;
    }
    return static_cast<uint32_t>(config_.ext_caps[0].offset / 4) & 0xFFFF;
  }

  int PortProtocolMajor(const int port) const {
    for (const ExtCap& cap : config_.ext_caps) {
      if (cap.id == spec::kSupportedProtocolId && cap.dwords.size() >= 3) {
        const int first = static_cast<int>(cap.dwords[2] & 0xFFu);
        const int count = static_cast<int>((cap.dwords[2] >> 8) & 0xFFu);
        if (port >= first && port < first + count) {
          return static_cast<int>((cap.dwords[0] >> 24) & 0xFFu);
        }
      }
    }
    return 0;
  }

  int PortAttachedSpeed(const int port) const {
    if (port < 1) {
      return 0;
    }
    const int index = port - 1;
    if (index < static_cast<int>(ports_.size()) &&
        ports_[static_cast<size_t>(index)].device != nullptr) {
      return ports_[static_cast<size_t>(index)].device->speed();
    }
    if (index < static_cast<int>(config_.attached_speed.size())) {
      return config_.attached_speed[static_cast<size_t>(index)];
    }
    return 0;
  }

  uint32_t PoweredPortsc(const int port) const {
    const int speed = PortAttachedSpeed(port);
    if (speed == 0) {
      return spec::kPortPower |
             (spec::kLinkRxDetect << spec::kPortLinkStateShift);
    }
    uint32_t value = spec::kPortPower | spec::kPortConnected |
                     spec::kPortConnectChange |
                     (static_cast<uint32_t>(speed) << spec::kPortSpeedShift);
    if (speed >= 4 || PortProtocolMajor(port) == 3) {
      value |=
          spec::kPortEnabled | (spec::kLinkU0 << spec::kPortLinkStateShift);
    } else {
      value |= spec::kLinkPolling << spec::kPortLinkStateShift;
    }
    return value;
  }

  // The PORTSC value of `port` after a reset: unpowered with port power
  // control, else powered with the attached device (if any) connected (USB 3
  // devices are enabled at once, USB 2 ones wait for a port reset).
  uint32_t DefaultPortsc(const int port) const {
    if (config_.ppc) {
      return 0;
    }
    return PoweredPortsc(port);
  }

  bool IsAllowed(const Violation kind) const {
    for (const Violation allowed : allowed_) {
      if (allowed == kind) {
        return true;
      }
    }
    return false;
  }

  void Record(const Violation kind,  //
              const int64_t seq,     //
              const int64_t offset,  //
              const uint32_t value,  //
              const std::string& detail = std::string()) {
    violations_.push_back(
        ViolationRecord{kind, seq, clock_->now(), offset, value, detail});
  }

  // The addresses on the USB 2.0 broadcast segment below root port `port`:
  // the root device (when the port is enabled) and every device behind
  // connected and enabled USB 2.0 hub ports (USB 2.0 11.1.2.1). Root ports
  // are separate links, so segments never span them.
  std::vector<uint8_t> Usb2SegmentAddresses(const int port) const {
    std::vector<uint8_t> addresses;
    if (port < 1 || port > static_cast<int>(ports_.size())) {
      return addresses;
    }
    const Port& state = ports_[static_cast<size_t>(port - 1)];
    if (state.device != nullptr && (state.portsc & spec::kPortEnabled) != 0) {
      addresses.push_back(state.device->address());
      state.device->AppendUsb2SegmentAddresses(&addresses);
    }
    return addresses;
  }

  static int CountAddress(const std::vector<uint8_t>& addresses,
                          const uint8_t address) {
    int count = 0;
    for (const uint8_t a : addresses) {
      if (a == address) {
        ++count;
      }
    }
    return count;
  }

  // Records a kDefaultAddressCollision when more than one device answers the
  // default address 0 below root port `port`, and a kUsbAddressCollision when
  // two reachable devices there share a non-zero address.
  void CheckUsb2Segment(const int port,        //
                        const int64_t seq,     //
                        const int64_t offset,  //
                        const uint32_t value) {
    if (port < 1 || port > static_cast<int>(ports_.size())) {
      return;
    }
    Port& state = ports_[static_cast<size_t>(port - 1)];
    const std::vector<uint8_t> addresses = Usb2SegmentAddresses(port);
    const int at_default = CountAddress(addresses, 0);
    if (at_default > max_default_state_devices_) {
      max_default_state_devices_ = at_default;
    }
    if (at_default > 1 && state.default_state_devices <= 1) {
      Record(Violation::kDefaultAddressCollision, seq, offset, value,
             "root port " + std::to_string(port) + ": " +
                 std::to_string(at_default) + " devices at address 0");
    }
    state.default_state_devices = at_default;
    bool shared = false;
    for (const uint8_t a : addresses) {
      if (a != 0 && CountAddress(addresses, a) > 1) {
        shared = true;
      }
    }
    if (shared && !state.address_collision) {
      Record(Violation::kUsbAddressCollision, seq, offset, value,
             "root port " + std::to_string(port));
    }
    state.address_collision = shared;
  }

  // Two devices answering one non-zero address on one USB 2.0 segment: their
  // responses collide and the xHC sees a USB Transaction Error.
  bool AddressCollides(const SimSlot& slot, const SimUsbDevice& dev) const {
    return dev.address() != 0 &&
           CountAddress(Usb2SegmentAddresses(slot.port), dev.address()) > 1;
  }

  bool NotReady() const {
    const int64_t now = clock_->now();
    const bool power_on = config_.power_on_not_ready_forever ||
                          now < start_tick_ + config_.power_on_not_ready_ticks;
    return power_on || reset_phase_ != ResetPhase::kIdle;
  }

  // Applies the transitions that are due at the current tick.
  void Update() {
    const int64_t now = clock_->now();
    decoding_ = !config_.all_ones &&
                (config_.stop_decoding_after_ticks < 0 ||
                 now < start_tick_ + config_.stop_decoding_after_ticks);
    if (run_state_ == RunState::kHalting && !config_.never_halts &&
        now >= halt_due_tick_) {
      run_state_ = RunState::kHalted;
    }
    if (start_due_tick_ > 0 && !config_.never_starts &&
        now >= start_due_tick_) {
      start_due_tick_ = 0;
      run_state_ = RunState::kRunning;
    }
    if (reset_phase_ == ResetPhase::kResetting && !config_.hcrst_never_clears &&
        now >= reset_due_tick_) {
      reset_phase_ = ResetPhase::kNotReady;
      ready_due_tick_ = reset_due_tick_ + config_.not_ready_after_reset_ticks;
    }
    if (reset_phase_ == ResetPhase::kNotReady &&
        !config_.not_ready_after_reset_forever && now >= ready_due_tick_) {
      reset_phase_ = ResetPhase::kIdle;
      if (config_.runs_after_reset) {
        run_state_ = RunState::kRunning;
      }
    }
    if (bios_owned_ && request_tick_ >= 0 && can_release_ &&
        config_.legacy.behavior == BiosBehavior::kReleasesAfterTicks &&
        now >= request_tick_ + config_.legacy.release_after_ticks) {
      ReleaseByBios();
    }
    if (cmd_executing_ && !cmd_exec_hung_ && now >= cmd_exec_due_tick_ &&
        run_state_ == RunState::kRunning && decoding_) {
      // A delayed command (command_delay_ticks) completes.
      cmd_executing_ = false;
      if (ExecuteCommand(cmd_exec_trb_, cmd_exec_phys_, 0, 0, 0)) {
        if (cmd_stop_after_current_) {
          StopCommandRing();
        } else {
          ProcessCommandRing(0, 0, 0);
        }
      }
    }
    for (int port = 1; port <= static_cast<int>(ports_.size()); ++port) {
      Port& state = ports_[static_cast<size_t>(port - 1)];
      if (state.power_on_due_tick > 0 && now >= state.power_on_due_tick) {
        state.power_on_due_tick = 0;
        if ((state.portsc & spec::kPortPower) != 0) {
          if (state.device != nullptr) {
            state.device->OnBusReset();
          }
          state.portsc = PoweredPortsc(port);
          if (run_state_ == RunState::kRunning &&
              (state.portsc & spec::kPortConnectChange) != 0) {
            usbsts_pending_ |= spec::kStsPortChangeDetect;
            if (!config_.drop_port_status_change_events) {
              PushPortStatusChangeEvent(port);
            }
          }
        }
      }
      if (state.reset_due_tick > 0 && now >= state.reset_due_tick) {
        state.reset_due_tick = 0;
        state.portsc &= ~spec::kPortReset;
        const bool warm = state.warm_reset;
        state.warm_reset = false;
        const int speed = PortAttachedSpeed(port);
        const bool inactive =
            ((state.portsc & spec::kPortLinkStateMask) >>
             spec::kPortLinkStateShift) == spec::kLinkInactive;
        if (speed > 0 && (state.portsc & spec::kPortConnected) != 0 &&
            inactive && !warm) {
          // A Hot Reset needs a working link: SS.Inactive stays until a Warm
          // Reset (USB 3.2 7.5.2), so the port stays disabled.
          state.portsc |= spec::kPortResetChange;
          usbsts_pending_ |= spec::kStsPortChangeDetect;
          if (run_state_ == RunState::kRunning &&
              !config_.drop_port_status_change_events) {
            PushPortStatusChangeEvent(port);
          }
        } else if (speed > 0 && (state.portsc & spec::kPortConnected) != 0) {
          if (state.device != nullptr) {
            state.device->OnBusReset();
          }
          state.portsc =
              (state.portsc &
               ~(spec::kPortLinkStateMask | (0xFu << spec::kPortSpeedShift))) |
              spec::kPortEnabled |
              (spec::kLinkU0 << spec::kPortLinkStateShift) |
              (static_cast<uint32_t>(speed) << spec::kPortSpeedShift) |
              spec::kPortResetChange | (warm ? spec::kPortWarmResetChange : 0);
          usbsts_pending_ |= spec::kStsPortChangeDetect;
          if (run_state_ == RunState::kRunning &&
              !config_.drop_port_status_change_events) {
            PushPortStatusChangeEvent(port);
          }
        }
      }
    }
    if (run_state_ == RunState::kRunning && decoding_) {
      FlushPendingEvents();
    }
  }

  // Records out-of-range and unaligned accesses; returns whether the access
  // may proceed.
  bool CheckAccess(const int64_t seq,     //
                   const int64_t offset,  //
                   const uint32_t value) {
    if (offset < 0 || offset + 4 > config_.bar_size) {
      Record(Violation::kOutOfRangeAccess, seq, offset, value);
      return false;
    }
    if ((offset & 3) != 0) {
      Record(Violation::kUnalignedAccess, seq, offset, value);
      return false;
    }
    return true;
  }

  void CheckQuietPeriod(const int64_t seq,     //
                        const int64_t offset,  //
                        const uint32_t value) {
    if (config_.hcrst_quiet_ticks > 0 && reset_write_seq_ >= 0 &&
        seq > reset_write_seq_ &&
        clock_->now() < reset_write_tick_ + config_.hcrst_quiet_ticks) {
      Record(Violation::kAccessTooSoonAfterReset, seq, offset, value);
    }
  }

  // A high dword of a 64-bit register must be written right after its low
  // dword (xHCI 5.1).
  void CheckHighDword(const int64_t seq,     //
                      const int64_t offset,  //
                      const uint32_t value,  //
                      const Access& previous) {
    const Location location = Locate(offset);
    bool high = false;
    if (location.region == Region::kOperational) {
      high =
          location.reg == spec::kCrcrHigh || location.reg == spec::kDcbaapHigh;
    } else if (location.region == Region::kRuntime &&
               location.reg >= spec::kInterrupterSets) {
      const int64_t reg =
          (location.reg - spec::kInterrupterSets) % spec::kInterrupterSetSize;
      high = reg == spec::kErstBaHigh || reg == spec::kErdpHigh;
    }
    if (high && !(previous.write && previous.offset == offset - 4 &&
                  previous.seq == seq - 1)) {
      Record(Violation::kHighDwordBeforeLow, seq, offset, value);
    }
  }

  Location Locate(const int64_t offset) const {
    const int ext_count = static_cast<int>(config_.ext_caps.size());
    for (int i = 0; i < ext_count; ++i) {
      const ExtCap& cap = config_.ext_caps[static_cast<size_t>(i)];
      if (offset >= cap.offset &&
          offset < cap.offset + 4 * static_cast<int64_t>(cap.dwords.size())) {
        return Location{Region::kExtCap,  //
                        i,                //
                        offset - cap.offset};
      }
    }
    if (offset < config_.cap_length) {
      return Location{Region::kCapability, 0, offset};
    }
    const int64_t op_base = config_.cap_length;
    if (offset >= op_base && offset < op_base + spec::kPortRegisterSets) {
      return Location{Region::kOperational, 0, offset - op_base};
    }
    const int64_t ports_base = op_base + spec::kPortRegisterSets;
    const int64_t ports_end =
        ports_base +
        spec::kPortRegisterSetSize * static_cast<int64_t>(ports_.size());
    if (offset >= ports_base && offset < ports_end) {
      const int64_t relative = offset - ports_base;
      return Location{
          Region::kPort,
          static_cast<int>(relative / spec::kPortRegisterSetSize) + 1,
          relative % spec::kPortRegisterSetSize};
    }
    const int64_t runtime_end =
        rt_base() + spec::kInterrupterSets +
        spec::kInterrupterSetSize * static_cast<int64_t>(interrupters_.size());
    if (offset >= rt_base() && offset < runtime_end) {
      return Location{Region::kRuntime, 0, offset - rt_base()};
    }
    const int64_t doorbells_end = db_base() + 4 * (config_.max_slots + 1);
    if (offset >= db_base() && offset < doorbells_end) {
      return Location{Region::kDoorbell,                           //
                      static_cast<int>((offset - db_base()) / 4),  //
                      0};
    }
    return Location{Region::kReserved, 0, offset};
  }

  uint32_t CapabilityRegister(const int64_t reg) const {
    switch (reg) {
      case spec::kCapLengthHciVersion:
        return (static_cast<uint32_t>(config_.hci_version) << 16) |
               (static_cast<uint32_t>(config_.cap_length) & 0xFF);
      case spec::kHcsParams1:
        return (static_cast<uint32_t>(config_.max_ports) << 24) |
               ((static_cast<uint32_t>(config_.max_interrupters) & 0x7FF)
                << 8) |
               (static_cast<uint32_t>(config_.max_slots) & 0xFF);
      case spec::kHcsParams2: {
        const uint32_t scratchpads =
            static_cast<uint32_t>(config_.max_scratchpads);
        return (((scratchpads >> 5) & 0x1F) << 21) |
               ((scratchpads & 0x1F) << 27) |
               (config_.scratchpad_restore ? 1u << 26 : 0) |
               ((static_cast<uint32_t>(config_.erst_max) & 0xF) << 4) |
               (static_cast<uint32_t>(config_.ist) & 0xF);
      }
      case spec::kHcsParams3:
        return 0;
      case spec::kHccParams1:
        return (XecpField() << 16) | (config_.hcc_params1_other & 0xFFF0) |
               (config_.ppc ? 1u << 3 : 0) | (config_.csz64 ? 1u << 2 : 0) |
               (config_.ac64 ? 1u << 0 : 0);
      case spec::kDbOff:
        return config_.db_off_raw;
      case spec::kRtsOff:
        return config_.rts_off_raw;
      case spec::kHccParams2:
        return config_.hcc_params2;
      default:
        return 0;
    }
  }

  uint32_t UsbCmdValue() const {
    return (usbcmd_ & spec::kCmdReadWriteMask) |
           (reset_phase_ == ResetPhase::kResetting ? spec::kCmdHcReset : 0) |
           (config_.usbcmd_rsvdp & spec::kCmdRsvdPMask);
  }

  uint32_t UsbStsValue() const {
    return (run_state_ == RunState::kHalted ? spec::kStsHalted : 0) |
           usbsts_pending_ | (NotReady() ? spec::kStsNotReady : 0) |
           (hce_ ? spec::kStsHcError : 0);
  }

  uint32_t LegSupValue() const {
    const ExtCap& cap = config_.ext_caps[static_cast<size_t>(legacy_cap_)];
    return (cap.dwords[0] & 0xFFFFu) |
           (config_.legacy.legsup_rsvdp & spec::kLegSupRsvdPMask) |
           (bios_owned_ ? spec::kBiosOwned : 0) |
           (os_owned_ ? spec::kOsOwned : 0);
  }

  uint32_t LegCtlStsValue() const {
    return (ctlsts_ &
            ~(spec::kSmiOnEventInterrupt | spec::kSmiOnHostSystemError |
              spec::kLegCtlStsRsvdZMask)) |
           ((usbsts_pending_ & spec::kStsEventInterrupt) != 0
                ? spec::kSmiOnEventInterrupt
                : 0) |
           ((usbsts_pending_ & spec::kStsHostSystemError) != 0
                ? spec::kSmiOnHostSystemError
                : 0);
  }

  uint32_t ReadRegister(const int64_t offset,     //
                        const bool side_effects,  //
                        const int64_t seq) {
    const Location location = Locate(offset);
    switch (location.region) {
      case Region::kExtCap: {
        if (location.index == legacy_cap_) {
          if (location.reg == 0) {
            if (side_effects) {
              PollLegSup();
            }
            return LegSupValue();
          }
          if (location.reg == 4) {
            return LegCtlStsValue();
          }
        }
        const ExtCap& cap =
            config_.ext_caps[static_cast<size_t>(location.index)];
        return cap.dwords[static_cast<size_t>(location.reg / 4)];
      }
      case Region::kCapability:
        return CapabilityRegister(location.reg);
      case Region::kOperational:
        return ReadOperational(location.reg, side_effects, seq);
      case Region::kPort: {
        const Port& port = ports_[static_cast<size_t>(location.index - 1)];
        switch (location.reg) {
          case 0x0:
            return port.portsc;
          case 0x4:
            return port.pmsc;
          case 0x8:
            return port.li;
          default:
            return port.hlpmc;
        }
      }
      case Region::kRuntime:
        return ReadRuntime(location.reg);
      case Region::kDoorbell:
      case Region::kReserved:
        return 0;
    }
    return 0;
  }

  uint32_t ReadOperational(const int64_t reg,        //
                           const bool side_effects,  //
                           const int64_t seq) {
    switch (reg) {
      case spec::kUsbCmd: {
        const uint32_t value = UsbCmdValue();
        if (side_effects && reset_write_seq_ >= 0 &&
            (value & spec::kCmdHcReset) == 0) {
          hcrst_clear_observed_ = true;
        }
        return value;
      }
      case spec::kUsbSts: {
        const uint32_t value = UsbStsValue();
        if (side_effects) {
          if (awaiting_halt_observation_ && (value & spec::kStsHalted) != 0) {
            awaiting_halt_observation_ = false;
            halt_observed_seq_ = seq;
          }
          if (hcrst_clear_observed_ && !reset_observed_ &&
              (value & spec::kStsNotReady) == 0) {
            reset_observed_ = true;
            reset_observed_seq_ = seq;
          }
        }
        return value;
      }
      case spec::kPageSize:
        return config_.page_size_raw;
      case spec::kDnCtrl:
        return dnctrl_;
      case spec::kCrcrLow:
        // The pointer reads 0; CRR is 1 from doorbell 0 until the ring stops.
        return cmd_ring_running_ ? spec::kCrcrRunning : 0u;
      case spec::kCrcrHigh:
        return 0;
      case spec::kDcbaapLow:
        return static_cast<uint32_t>(dcbaap_);
      case spec::kDcbaapHigh:
        return static_cast<uint32_t>(dcbaap_ >> 32);
      case spec::kConfig:
        return config_register_;
      default:
        return 0;
    }
  }

  uint32_t ReadRuntime(const int64_t reg) const {
    if (reg < spec::kInterrupterSets) {
      return 0;  // MFINDEX: the controller is not running
    }
    const int64_t relative = reg - spec::kInterrupterSets;
    const Interrupter& state = interrupters_[static_cast<size_t>(
        relative / spec::kInterrupterSetSize)];
    switch (relative % spec::kInterrupterSetSize) {
      case spec::kIman:
        return state.iman;
      case spec::kImod:
        return state.imod;
      case spec::kErstSz:
        return state.erst_size;
      case spec::kErstBaLow:
        return static_cast<uint32_t>(state.erst_base);
      case spec::kErstBaHigh:
        return static_cast<uint32_t>(state.erst_base >> 32);
      case spec::kErdpLow:
        return static_cast<uint32_t>(state.erdp);
      case spec::kErdpHigh:
        return static_cast<uint32_t>(state.erdp >> 32);
      default:
        return 0;
    }
  }

  // A write that sets the controller up (rings, ports, interrupts, start)
  // before the latest reset was observed complete.
  void CheckProgramming(const int64_t seq,     //
                        const int64_t offset,  //
                        const uint32_t value) {
    if (!reset_observed_) {
      Record(Violation::kProgrammingBeforeReset, seq, offset, value);
    }
  }

  void CheckReady(const int64_t seq,     //
                  const int64_t offset,  //
                  const uint32_t value) {
    if (NotReady()) {
      Record(Violation::kOpWriteWhileNotReady, seq, offset, value);
    }
  }

  void WriteRegister(const int64_t offset,  //
                     const uint32_t value,  //
                     const int64_t seq) {
    const Location location = Locate(offset);
    switch (location.region) {
      case Region::kExtCap:
        if (location.index == legacy_cap_ && location.reg == 0) {
          WriteLegSup(value, seq, offset);
        } else if (location.index == legacy_cap_ && location.reg == 4) {
          WriteLegCtlSts(value, seq, offset);
        } else {
          Record(Violation::kWriteToReadOnly, seq, offset, value);
        }
        return;
      case Region::kCapability:
        Record(Violation::kWriteToReadOnly, seq, offset, value);
        return;
      case Region::kOperational:
        WriteOperational(location.reg, value, seq, offset);
        return;
      case Region::kPort:
        CheckReady(seq, offset, value);
        CheckProgramming(seq, offset, value);
        WritePort(location.index, location.reg, value, seq, offset);
        return;
      case Region::kRuntime:
        WriteRuntime(location.reg, value, seq, offset);
        return;
      case Region::kDoorbell:
        CheckReady(seq, offset, value);
        CheckProgramming(seq, offset, value);
        WriteDoorbell(location.index, value, seq, offset);
        return;
      case Region::kReserved:
        Record(Violation::kWriteToReserved, seq, offset, value);
        return;
    }
  }

  void WriteOperational(const int64_t reg,     //
                        const uint32_t value,  //
                        const int64_t seq,     //
                        const int64_t offset) {
    if (reg != spec::kUsbSts) {
      CheckReady(seq, offset, value);
    }
    switch (reg) {
      case spec::kUsbCmd:
        WriteUsbCmd(value, seq, offset);
        return;
      case spec::kUsbSts:
        if ((value & spec::kStsRsvdZMask) != 0) {
          Record(Violation::kRsvdZWrittenOne, seq, offset, value);
        }
        usbsts_pending_ &= ~(value & spec::kStsWriteOneToClearMask);
        return;
      case spec::kPageSize:
        Record(Violation::kWriteToReadOnly, seq, offset, value);
        return;
      case spec::kDnCtrl:
        CheckProgramming(seq, offset, value);
        dnctrl_ = value & 0xFFFF;
        return;
      case spec::kCrcrLow:
        CheckProgramming(seq, offset, value);
        crcr_abort_low_written_ = false;
        if (run_state_ != RunState::kHalted) {
          if ((value & (spec::kCrcrCommandStop | spec::kCrcrCommandAbort)) ==
              0) {
            Record(Violation::kRingModifiedWhileRunning, seq, offset, value);
            return;
          }
          // CS / CA; the pointer and RCS bits are ignored while the ring runs
          // (xHCI 5.4.5), and CS / CA have no effect on a stopped ring.
          crcr_abort_low_written_ = true;
          if (cmd_ring_running_ && !config_.command_abort_hangs) {
            if ((value & spec::kCrcrCommandAbort) != 0 || !cmd_executing_) {
              StopCommandRing();
            } else {
              cmd_stop_after_current_ = true;
            }
            FlushPendingEvents();
          }
          return;
        }
        crcr_ = (crcr_ & ~uint64_t{0xFFFFFFFF}) | value;
        return;
      case spec::kCrcrHigh: {
        CheckProgramming(seq, offset, value);
        const bool after_stop_request = crcr_abort_low_written_;
        crcr_abort_low_written_ = false;
        if (run_state_ != RunState::kHalted) {
          if (!after_stop_request) {
            Record(Violation::kRingModifiedWhileRunning, seq, offset, value);
          } else if (!cmd_ring_running_ &&
                     value != static_cast<uint32_t>(cmd_dequeue_ >> 32)) {
            // The ring stopped before this half of the 64-bit write landed:
            // the xHC takes it as the upper half of its dequeue pointer (the
            // corruption behind Linux commits ff0e50d3564f, 09f736aa9547).
            Record(Violation::kCommandRingPointerCorrupted, seq, offset, value);
            cmd_dequeue_ =
                (cmd_dequeue_ & uint64_t{0xFFFFFFFF}) | (uint64_t{value} << 32);
          }
          return;
        }
        crcr_ = (crcr_ & uint64_t{0xFFFFFFFF}) | (uint64_t{value} << 32);
        return;
      }
      case spec::kDcbaapLow:
        CheckProgramming(seq, offset, value);
        if (run_state_ != RunState::kHalted) {
          Record(Violation::kRingModifiedWhileRunning, seq, offset, value);
        }
        dcbaap_ = (dcbaap_ & ~uint64_t{0xFFFFFFFF}) | (value & ~0x3Fu);
        return;
      case spec::kDcbaapHigh:
        CheckProgramming(seq, offset, value);
        if (run_state_ != RunState::kHalted) {
          Record(Violation::kRingModifiedWhileRunning, seq, offset, value);
        }
        dcbaap_ = (dcbaap_ & uint64_t{0xFFFFFFFF}) | (uint64_t{value} << 32);
        return;
      case spec::kConfig:
        CheckProgramming(seq, offset, value);
        if (run_state_ != RunState::kHalted) {
          Record(Violation::kRingModifiedWhileRunning, seq, offset, value);
        }
        config_register_ = value & 0x3FF;
        return;
      default:
        Record(Violation::kWriteToReserved, seq, offset, value);
        return;
    }
  }

  void WriteUsbCmd(const uint32_t value,  //
                   const int64_t seq,     //
                   const int64_t offset) {
    if ((value & spec::kCmdRsvdPMask) !=
        (config_.usbcmd_rsvdp & spec::kCmdRsvdPMask)) {
      Record(Violation::kRsvdPChanged, seq, offset, value);
    }
    if ((value & (spec::kCmdLightHcReset | spec::kCmdSaveState |
                  spec::kCmdRestoreState)) != 0) {
      Record(Violation::kUnsupportedCommand, seq, offset, value);
    }
    if ((value &
         (spec::kCmdRunStop | spec::kCmdInterrupterEnable |
          spec::kCmdHostSystemErrorEnable | spec::kCmdEnableWrapEvent)) != 0) {
      CheckProgramming(seq, offset, value);
    }
    const bool was_run = (usbcmd_ & spec::kCmdRunStop) != 0;
    const bool run = (value & spec::kCmdRunStop) != 0;
    usbcmd_ = value & spec::kCmdReadWriteMask;
    if (was_run && !run && run_state_ == RunState::kRunning) {
      run_state_ = RunState::kHalting;
      halt_due_tick_ = clock_->now() + config_.halt_delay_ticks;
      awaiting_halt_observation_ = true;
      // Halting stops the Command Ring (CRR = 0, xHCI 5.4.5).
      cmd_ring_running_ = false;
      cmd_executing_ = false;
      cmd_stop_after_current_ = false;
    } else if (!was_run && run) {
      if (reset_observed_ && dma_ != nullptr) {
        LatchRingsAndValidateOnStart(seq, offset, value);
      }
      if (!config_.never_starts) {
        if (config_.start_delay_ticks > 0) {
          start_due_tick_ = clock_->now() + config_.start_delay_ticks;
        } else {
          run_state_ = RunState::kRunning;
        }
      }
    }
    if ((value & spec::kCmdHcReset) != 0) {
      if (reset_phase_ == ResetPhase::kResetting) {
        Record(Violation::kHcrstRetriggered, seq, offset, value);
      } else if (run_state_ != RunState::kHalted) {
        Record(Violation::kResetWhileRunning, seq, offset, value);
      } else {
        PerformReset(seq);
      }
    }
  }

  // HCRST: the reset starts; operational, runtime and port registers get
  // their defaults. The USB Legacy Support registers keep their values.
  void PerformReset(const int64_t seq) {
    ++reset_count_;
    reset_write_seq_ = seq;
    reset_write_tick_ = clock_->now();
    reset_phase_ = ResetPhase::kResetting;
    reset_due_tick_ = reset_write_tick_ + config_.reset_delay_ticks;
    start_due_tick_ = 0;
    cmd_ring_running_ = false;
    cmd_executing_ = false;
    cmd_stop_after_current_ = false;
    crcr_abort_low_written_ = false;
    hcrst_clear_observed_ = false;
    reset_observed_ = false;
    usbcmd_ = 0;
    usbsts_pending_ = 0;
    hce_ = false;
    dnctrl_ = 0;
    crcr_ = 0;
    dcbaap_ = 0;
    config_register_ = 0;
    evt_seg_base_ = 0;
    evt_seg_size_ = 0;
    evt_enqueue_ = 0;
    evt_pcs_ = 1;
    evt_unconsumed_ = 0;
    evt_full_reported_ = false;
    pending_events_.clear();
    irq_asserted_ = false;
    slots_.assign(static_cast<size_t>(config_.max_slots + 1), SimSlot{});
    last_enabled_slot_ = 0;
    for (Interrupter& state : interrupters_) {
      state = Interrupter{};
      state.imod = config_.imod_reset_value;
    }
    for (int port = 1; port <= static_cast<int>(ports_.size()); ++port) {
      Port& state = ports_[static_cast<size_t>(port - 1)];
      SimUsbDevice* const dev = state.device;
      if (dev != nullptr) {
        dev->OnBusReset();
      }
      state = Port{};
      state.device = dev;
      state.portsc = DefaultPortsc(port);
    }
  }

  void WritePort(const int port,        //
                 const int64_t reg,     //
                 const uint32_t value,  //
                 const int64_t seq,     //
                 const int64_t offset) {
    Port& state = ports_[static_cast<size_t>(port - 1)];
    if (reg != 0) {
      if (reg == 0x4) {
        state.pmsc = value;
      } else if (reg == 0xC) {
        state.hlpmc = value;
      } else {
        Record(Violation::kWriteToReadOnly, seq, offset, value);
      }
      return;
    }
    const uint32_t old = state.portsc;
    uint32_t updated = old & ~(value & spec::kPortChangeMask);
    if ((value & spec::kPortRsvdZMask) != 0) {
      Record(Violation::kRsvdZWrittenOne, seq, offset, value);
    }
    const bool usb3 = PortProtocolMajor(port) == 3;
    const bool reset = (value & (spec::kPortReset | spec::kPortWarmReset)) != 0;
    if ((value & spec::kPortLinkWriteStrobe) != 0 ||
        (reset && run_state_ != RunState::kRunning)) {
      Record(Violation::kUnmodeledWrite, seq, offset, value, "port reset/link");
    } else if ((value & spec::kPortWarmReset) != 0 && !usb3) {
      // WPR is RsvdZ on USB 2 protocol ports (xHCI 5.4.8).
      Record(Violation::kRsvdZWrittenOne, seq, offset, value, "WPR");
    } else if (reset) {
      // PR: a USB 2 bus reset, or a Hot Reset of a USB 3 port; WPR: a Warm
      // Reset of a USB 3 port (xHCI 5.4.8, 4.19.5.1). PR also reads 1 while
      // a Warm Reset runs.
      updated |= spec::kPortReset;
      updated &= ~spec::kPortEnabled;
      state.warm_reset = (value & spec::kPortWarmReset) != 0;
      if (!config_.port_reset_never_completes) {
        state.reset_due_tick =
            clock_->now() +
            (config_.port_reset_ticks > 0 ? config_.port_reset_ticks : 1);
      }
    }
    if ((value & spec::kPortEnabled) != 0) {
      Record(Violation::kPortDisabledBySoftware, seq, offset, value);
      updated &= ~spec::kPortEnabled;
    }
    if (config_.ppc) {
      if ((old & spec::kPortPower) != 0 && (value & spec::kPortPower) == 0) {
        Record(Violation::kPortPowerCleared, seq, offset, value);
      }
      if ((old & spec::kPortPower) == 0 && (value & spec::kPortPower) != 0) {
        state.power_on_due_tick = clock_->now() + 1;
      }
      updated = (updated & ~spec::kPortPower) | (value & spec::kPortPower);
    }
    const uint32_t stored = spec::kPortIndicatorMask | spec::kPortWakeMask;
    updated = (updated & ~stored) | (value & stored);
    state.portsc = updated;
  }

  void WriteRuntime(const int64_t reg,     //
                    const uint32_t value,  //
                    const int64_t seq,     //
                    const int64_t offset) {
    if (reg < spec::kInterrupterSets) {
      Record(reg == spec::kMfIndex ? Violation::kWriteToReadOnly
                                   : Violation::kWriteToReserved,
             seq, offset, value);
      return;
    }
    CheckProgramming(seq, offset, value);
    const int64_t relative = reg - spec::kInterrupterSets;
    const int ir_idx = static_cast<int>(relative / spec::kInterrupterSetSize);
    Interrupter& state = interrupters_[static_cast<size_t>(ir_idx)];
    switch (relative % spec::kInterrupterSetSize) {
      case spec::kIman:
        if (run_state_ == RunState::kRunning &&
            (value & spec::kImanPending) != 0 &&
            (state.iman & spec::kImanPending) != 0 &&
            (usbsts_pending_ & spec::kStsEventInterrupt) != 0) {
          Record(Violation::kImanClearedBeforeEint, seq, offset, value);
        }
        state.iman =
            (value & spec::kImanEnable) |
            (state.iman & spec::kImanPending & ~(value & spec::kImanPending));
        return;
      case spec::kImod:
        state.imod = value;
        return;
      case spec::kErstSz:
        if (run_state_ != RunState::kHalted) {
          Record(Violation::kRingModifiedWhileRunning, seq, offset, value);
        }
        state.erst_size = value & 0xFFFF;
        return;
      case spec::kErstBaLow:
        if (run_state_ != RunState::kHalted) {
          Record(Violation::kRingModifiedWhileRunning, seq, offset, value);
        }
        state.erst_base =
            (state.erst_base & ~uint64_t{0xFFFFFFFF}) | (value & ~0x3Fu);
        return;
      case spec::kErstBaHigh:
        if (run_state_ != RunState::kHalted) {
          Record(Violation::kRingModifiedWhileRunning, seq, offset, value);
        }
        state.erst_base =
            (state.erst_base & uint64_t{0xFFFFFFFF}) | (uint64_t{value} << 32);
        if (reset_observed_ && state.erst_size == 0 && state.erst_base != 0 &&
            accesses_.size() >= 2 && accesses_[accesses_.size() - 2].write &&
            accesses_[accesses_.size() - 2].offset == offset - 4) {
          Record(Violation::kErstBaBeforeSize, seq, offset - 4,
                 static_cast<uint32_t>(state.erst_base));
        }
        return;
      case spec::kErdpLow: {
        erdp_ehb_written_ = (value & spec::kErdpBusy) != 0;
        const uint32_t busy = static_cast<uint32_t>(state.erdp) &
                              spec::kErdpBusy & ~(value & spec::kErdpBusy);
        state.erdp = (state.erdp & ~uint64_t{0xFFFFFFFF}) |
                     (value & ~spec::kErdpBusy) | busy;
        return;
      }
      case spec::kErdpHigh: {
        state.erdp =
            (state.erdp & uint64_t{0xFFFFFFFF}) | (uint64_t{value} << 32);
        if (ir_idx == 0 && evt_seg_size_ > 0) {
          const uint64_t deq_phys = state.erdp & ~0xFull;
          if (deq_phys >= evt_seg_base_ &&
              deq_phys < evt_seg_base_ + evt_seg_size_ * 16) {
            const int deq_idx =
                static_cast<int>((deq_phys - evt_seg_base_) / 16);
            if (deq_idx == evt_enqueue_) {
              evt_unconsumed_ = 0;
            } else if (evt_enqueue_ > deq_idx) {
              evt_unconsumed_ = evt_enqueue_ - deq_idx;
            } else {
              evt_unconsumed_ =
                  static_cast<int>(evt_seg_size_) - deq_idx + evt_enqueue_;
            }
            if (evt_unconsumed_ < static_cast<int>(evt_seg_size_) - 1) {
              evt_full_reported_ = false;
            }
          }
          if (erdp_ehb_written_) {
            erdp_ehb_written_ = false;
            FlushPendingEvents();
            if (evt_unconsumed_ > 0) {
              usbsts_pending_ |= spec::kStsEventInterrupt;
              state.iman |= spec::kImanPending;
              state.erdp |= spec::kErdpBusy;
              if ((usbcmd_ & spec::kCmdInterrupterEnable) != 0 &&
                  (state.iman & spec::kImanEnable) != 0) {
                ++irq_count_;
                irq_asserted_ = true;
              }
            }
          }
        }
        return;
      }
      default:
        Record(Violation::kWriteToReserved, seq, offset, value);
        return;
    }
  }

  void WriteDoorbell(const int slot,        //
                     const uint32_t value,  //
                     const int64_t seq,     //
                     const int64_t offset) {
    if (run_state_ != RunState::kRunning || dma_ == nullptr) {
      return;
    }
    if (slot == 0) {
      if ((value & 0xFFu) == 0) {
        // Doorbell 0 starts (or restarts) the Command Ring (xHCI 4.6.1).
        cmd_ring_running_ = true;
        if (!cmd_executing_) {
          ProcessCommandRing(seq, offset, value);
        }
      }
    } else if (slot >= 1 && slot < static_cast<int>(slots_.size())) {
      const uint8_t dci = static_cast<uint8_t>(value & 0xFFu);
      SimSlot& s = slots_[static_cast<size_t>(slot)];
      if (s.enabled && dci >= 1 && dci < 32) {
        s.ep[dci].doorbell_rung = true;
        if (s.ep[dci].state == usb::kEpStateStopped) {
          // A doorbell restarts a Stopped endpoint (xHCI 4.8.3).
          s.ep[dci].state = usb::kEpStateRunning;
          UpdateOutputEpState(s, dci, config_.csz64 ? 64 : 32, seq, offset,
                              value);
        }
        if (dci == 1) {
          ProcessEp0Ring(static_cast<uint8_t>(slot), seq, offset, value);
        } else {
          ServiceInterruptEndpoint(static_cast<uint8_t>(slot), dci, seq, offset,
                                   value);
        }
      }
    }
    FlushPendingEvents();
  }

  bool ReadDma(const uint64_t phys,       //
               void* const dst,           //
               const int64_t len,         //
               const int64_t seq = 0,     //
               const int64_t offset = 0,  //
               const uint32_t val = 0) {
    if (!bus_master_) {
      Record(Violation::kDmaWhileBusMasterOff, seq, offset, val);
      return false;
    }
    if (dma_ == nullptr) {
      Record(Violation::kInvalidDmaAddress, seq, offset, val, "no DMA arena");
      return false;
    }
    const uint8_t* const ptr = dma_->Translate(phys, len, !config_.ac64);
    if (ptr == nullptr) {
      Record(Violation::kInvalidDmaAddress, seq, offset, val,
             "0x" + HexString(phys, 8));
      return false;
    }
    std::memcpy(dst, ptr, static_cast<size_t>(len));
    return true;
  }

  bool WriteDma(const uint64_t phys,       //
                const void* const src,     //
                const int64_t len,         //
                const int64_t seq = 0,     //
                const int64_t offset = 0,  //
                const uint32_t val = 0) {
    if (!bus_master_) {
      Record(Violation::kDmaWhileBusMasterOff, seq, offset, val);
      return false;
    }
    if (dma_ == nullptr) {
      Record(Violation::kInvalidDmaAddress, seq, offset, val, "no DMA arena");
      return false;
    }
    uint8_t* const ptr = dma_->Translate(phys, len, !config_.ac64);
    if (ptr == nullptr) {
      Record(Violation::kInvalidDmaAddress, seq, offset, val,
             "0x" + HexString(phys, 8));
      return false;
    }
    std::memcpy(ptr, src, static_cast<size_t>(len));
    return true;
  }

  void LatchRingsAndValidateOnStart(const int64_t seq,     //
                                    const int64_t offset,  //
                                    const uint32_t value) {
    slots_.assign(static_cast<size_t>(config_.max_slots + 1), SimSlot{});
    last_enabled_slot_ = 0;
    if (!bus_master_) {
      Record(Violation::kDmaWhileBusMasterOff, seq, offset, value);
      return;
    }
    const uint32_t max_slots_en = config_register_ & 0xFFu;
    const int64_t dcbaa_bytes = static_cast<int64_t>((max_slots_en + 1) * 8);
    uint64_t dcbaa_head = 0;
    if (dcbaap_ == 0 || (dcbaap_ & 0x3Full) != 0 ||
        dma_->Translate(dcbaap_, dcbaa_bytes, !config_.ac64) == nullptr ||
        !ReadDma(dcbaap_, &dcbaa_head, sizeof(dcbaa_head), seq, offset,
                 value)) {
      Record(Violation::kInvalidDmaAddress, seq, offset, value, "bad DCBAAP");
    } else if (config_.max_scratchpads > 0) {
      if (dcbaa_head == 0 || (dcbaa_head & 0x3Full) != 0) {
        Record(Violation::kInvalidDmaAddress, seq, offset, value,
               "missing scratchpad array in DCBAA[0]");
      } else {
        for (int i = 0; i < config_.max_scratchpads; ++i) {
          uint64_t sp_phys = 0;
          if (!ReadDma(dcbaa_head + static_cast<uint64_t>(i) * 8, &sp_phys, 8,
                       seq, offset, value) ||
              sp_phys == 0 || (sp_phys & 0xFFFull) != 0) {
            Record(Violation::kInvalidDmaAddress, seq, offset, value,
                   "bad scratchpad entry");
            break;
          }
          uint8_t probe = 0;
          if (!ReadDma(sp_phys, &probe, 1, seq, offset, value) ||
              !ReadDma(sp_phys + 4095, &probe, 1, seq, offset, value)) {
            break;
          }
        }
      }
    }
    cmd_dequeue_ = crcr_ & ~0x3Full;
    cmd_ccs_ = static_cast<uint32_t>(crcr_ & 1u);
    cmd_ring_running_ = false;
    cmd_executing_ = false;
    cmd_stop_after_current_ = false;
    usb::Trb probe_trb{};
    if (cmd_dequeue_ == 0 || !ReadDma(cmd_dequeue_, &probe_trb,
                                      sizeof(probe_trb), seq, offset, value)) {
      Record(Violation::kInvalidDmaAddress, seq, offset, value, "bad CRCR");
    }

    if (!interrupters_.empty()) {
      const Interrupter& ir0 = interrupters_[0];
      usb::ErstEntry erst0{};
      if (ir0.erst_size == 0 || ir0.erst_base == 0 ||
          !ReadDma(ir0.erst_base, &erst0, sizeof(erst0), seq, offset, value) ||
          erst0.ring_segment_base == 0 ||
          (erst0.ring_segment_base & 0x3Full) != 0 ||
          erst0.ring_segment_size < 16 ||
          dma_->Translate(erst0.ring_segment_base,
                          static_cast<int64_t>(erst0.ring_segment_size) * 16,
                          !config_.ac64) == nullptr) {
        Record(Violation::kInvalidDmaAddress, seq, offset, value, "bad ERST0");
      } else {
        evt_seg_base_ = erst0.ring_segment_base;
        evt_seg_size_ = erst0.ring_segment_size;
        evt_enqueue_ = 0;
        evt_pcs_ = 1;
        evt_unconsumed_ = 0;
        evt_full_reported_ = false;
      }
    }
  }

  void PushPortStatusChangeEvent(const int port) {
    usb::Trb ev{};
    ev.dw0 = static_cast<uint32_t>(port & 0xFF) << 24;
    ev.dw2 = static_cast<uint32_t>(usb::kCcSuccess) << 24;
    ev.dw3 = static_cast<uint32_t>(usb::kTrbPortStatusChange)
             << usb::kTrbTypeShift;
    pending_events_.push_back(ev);
  }

  void PushCommandCompletion(const uint64_t cmd_trb_phys,  //
                             const uint8_t cc,             //
                             const uint8_t slot_id) {
    usb::Trb ev{};
    ev.dw0 = static_cast<uint32_t>(cmd_trb_phys & ~0xFull);
    ev.dw1 = static_cast<uint32_t>(cmd_trb_phys >> 32);
    ev.dw2 = static_cast<uint32_t>(cc) << 24;
    ev.dw3 = (static_cast<uint32_t>(usb::kTrbCommandCompletion)
              << usb::kTrbTypeShift) |
             (static_cast<uint32_t>(slot_id) << usb::kTrbSlotIdShift);
    pending_events_.push_back(ev);
  }

  void PushTransferEvent(const uint64_t trb_phys,  //
                         const uint8_t cc,         //
                         const uint32_t residual,  //
                         const uint8_t slot_id,    //
                         const uint8_t ep_dci) {
    usb::Trb ev{};
    ev.dw0 = static_cast<uint32_t>(trb_phys);
    ev.dw1 = static_cast<uint32_t>(trb_phys >> 32);
    ev.dw2 = (residual & 0x00FF'FFFFu) | (static_cast<uint32_t>(cc) << 24);
    ev.dw3 =
        (static_cast<uint32_t>(usb::kTrbTransferEvent) << usb::kTrbTypeShift) |
        (static_cast<uint32_t>(ep_dci & 0x1Fu) << usb::kTrbEndpointIdShift) |
        (static_cast<uint32_t>(slot_id) << usb::kTrbSlotIdShift);
    pending_events_.push_back(ev);
  }

  void FlushPendingEvents() {
    if (evt_seg_size_ == 0 || interrupters_.empty() || dma_ == nullptr) {
      return;
    }
    Interrupter& ir0 = interrupters_[0];
    while (!pending_events_.empty()) {
      if (evt_unconsumed_ >= static_cast<int>(evt_seg_size_) - 1) {
        if (!evt_full_reported_) {
          evt_full_reported_ = true;
          usb::Trb full_ev{};
          full_ev.dw2 = static_cast<uint32_t>(usb::kCcEventRingFull) << 24;
          full_ev.dw3 = (static_cast<uint32_t>(usb::kTrbHostControllerEvent)
                         << usb::kTrbTypeShift) |
                        (evt_pcs_ & usb::kTrbCycle);
          WriteDma(evt_seg_base_ + static_cast<uint64_t>(evt_enqueue_) * 16,
                   &full_ev, sizeof(full_ev));
          ++evt_enqueue_;
          ++evt_unconsumed_;
          if (evt_enqueue_ >= static_cast<int>(evt_seg_size_)) {
            evt_enqueue_ = 0;
            evt_pcs_ ^= 1u;
          }
          RaiseInterrupter0(ir0);
        }
        // Drop events generated while the event ring is full (xHCI 4.9.4).
        pending_events_.clear();
        break;
      }
      usb::Trb ev = pending_events_.front();
      pending_events_.pop_front();
      ev.dw3 = (ev.dw3 & ~usb::kTrbCycle) | (evt_pcs_ & usb::kTrbCycle);
      if (!WriteDma(evt_seg_base_ + static_cast<uint64_t>(evt_enqueue_) * 16,
                    &ev, sizeof(ev))) {
        break;
      }
      ++evt_enqueue_;
      ++evt_unconsumed_;
      if (evt_enqueue_ >= static_cast<int>(evt_seg_size_)) {
        evt_enqueue_ = 0;
        evt_pcs_ ^= 1u;
      }
      RaiseInterrupter0(ir0);
    }
  }

  void RaiseInterrupter0(Interrupter& ir0) {
    usbsts_pending_ |= spec::kStsEventInterrupt;
    ir0.iman |= spec::kImanPending;
    if ((ir0.erdp & spec::kErdpBusy) == 0) {
      ir0.erdp |= spec::kErdpBusy;
      if ((usbcmd_ & spec::kCmdInterrupterEnable) != 0 &&
          (ir0.iman & spec::kImanEnable) != 0) {
        ++irq_count_;
        irq_asserted_ = true;
      }
    }
  }

  // Fetches and executes commands while the ring runs, until it is empty or
  // a command takes time (command_delay_ticks) or hangs.
  void ProcessCommandRing(const int64_t seq,     //
                          const int64_t offset,  //
                          const uint32_t value) {
    int steps = 0;
    while (cmd_ring_running_ && !cmd_executing_ && steps++ < 512) {
      usb::Trb trb{};
      if (!ReadDma(cmd_dequeue_, &trb, sizeof(trb), seq, offset, value)) {
        return;
      }
      if ((trb.dw3 & usb::kTrbCycle) != cmd_ccs_) {
        return;
      }
      const uint64_t trb_phys = cmd_dequeue_;
      if (usb::TrbType(trb) == usb::kTrbLink) {
        cmd_dequeue_ = usb::TrbPointer(trb) & ~0xFull;
        if ((trb.dw3 & usb::kTrbToggleCycle) != 0) {
          cmd_ccs_ ^= 1u;
        }
        continue;
      }
      cmd_dequeue_ += 16;
      if (config_.command_hang) {
        HangCommand(trb, trb_phys);
        return;
      }
      if (config_.command_delay_ticks > 0) {
        cmd_executing_ = true;
        cmd_exec_hung_ = false;
        cmd_exec_trb_ = trb;
        cmd_exec_phys_ = trb_phys;
        cmd_exec_due_tick_ = clock_->now() + config_.command_delay_ticks;
        return;
      }
      if (ExecuteCommand(trb, trb_phys, seq, offset, value) &&
          cmd_stop_after_current_) {
        StopCommandRing();
      }
    }
  }

  // The command never completes on its own (e.g. a SET_ADDRESS that the
  // device NAKs forever): only CRCR.CA ends it.
  void HangCommand(const usb::Trb& trb, const uint64_t trb_phys) {
    cmd_executing_ = true;
    cmd_exec_hung_ = true;
    cmd_exec_trb_ = trb;
    cmd_exec_phys_ = trb_phys;
  }

  // CRCR.CA (or CS once the command in progress is done): an executing
  // command completes with Command Aborted, then the ring stops with a
  // Command Ring Stopped event at its dequeue pointer, the next command TRB
  // (xHCI 4.6.1.2).
  void StopCommandRing() {
    if (cmd_executing_) {
      cmd_executing_ = false;
      PushCommandCompletion(cmd_exec_phys_, usb::kCcCommandAborted, 0);
    }
    cmd_stop_after_current_ = false;
    cmd_ring_running_ = false;
    PushCommandCompletion(cmd_dequeue_, usb::kCcCommandRingStopped, 0);
  }

  // Executes one command and posts its completion. Returns false if the
  // command hangs instead (HangCommand()).
  bool ExecuteCommand(const usb::Trb& trb,      //
                      const uint64_t trb_phys,  //
                      const int64_t seq,        //
                      const int64_t offset,     //
                      const uint32_t value) {
    const int csz = config_.csz64 ? 64 : 32;
    const int max_slots_en = static_cast<int>(config_register_ & 0xFFu);
    {
      const uint8_t type = usb::TrbType(trb);
      switch (type) {
        case usb::kTrbEnableSlot: {
          uint8_t chosen = 0;
          if (!config_.no_slots_available) {
            const int first =
                config_.round_robin_slots ? last_enabled_slot_ : 0;
            for (int k = 1; k <= max_slots_en; ++k) {
              const int s = (first + k - 1) % max_slots_en + 1;
              if (s < static_cast<int>(slots_.size()) &&
                  !slots_[static_cast<size_t>(s)].enabled) {
                chosen = static_cast<uint8_t>(s);
                break;
              }
            }
          }
          if (chosen == 0) {
            PushCommandCompletion(trb_phys, usb::kCcNoSlotsAvailable, 0);
          } else {
            SimSlot& slot = slots_[chosen];
            slot = SimSlot{};
            slot.enabled = true;
            slot.slot_state = usb::kSlotStateDisabledOrEnabled;
            last_enabled_slot_ = chosen;
            PushCommandCompletion(trb_phys, usb::kCcSuccess, chosen);
          }
          break;
        }
        case usb::kTrbDisableSlot: {
          const uint8_t s = usb::TrbSlotId(trb);
          if (s < 1 || s > max_slots_en ||
              !slots_[static_cast<size_t>(s)].enabled) {
            PushCommandCompletion(trb_phys, usb::kCcSlotNotEnabled, s);
          } else {
            SimSlot& slot = slots_[static_cast<size_t>(s)];
            if (slot.out_ctx_phys != 0) {
              // The xHC finds the slot's contexts through DCBAA, which
              // software clears only after this command (xHCI 4.6.4).
              uint64_t entry = 0;
              ReadDma(dcbaap_ + static_cast<uint64_t>(s) * 8, &entry, 8, seq,
                      offset, value);
              if (entry != slot.out_ctx_phys) {
                Record(Violation::kDcbaaChangedBeforeDisableSlot, seq, offset,
                       value, "slot " + std::to_string(s));
              }
              CheckOutputContext(slot, s, seq, offset, value);
            }
            slot = SimSlot{};
            disable_slot_log_.push_back(s);
            PushCommandCompletion(trb_phys, usb::kCcSuccess, s);
          }
          break;
        }
        case usb::kTrbAddressDevice: {
          const uint8_t s = usb::TrbSlotId(trb);
          if (s < 1 || s > max_slots_en ||
              !slots_[static_cast<size_t>(s)].enabled) {
            PushCommandCompletion(trb_phys, usb::kCcSlotNotEnabled, s);
            break;
          }
          std::vector<uint8_t> in_ctx(static_cast<size_t>(33 * csz), 0);
          uint64_t out_ctx_phys = 0;
          if (!ReadDma(usb::TrbPointer(trb), in_ctx.data(), 33 * csz, seq,
                       offset, value) ||
              !ReadDma(dcbaap_ + static_cast<uint64_t>(s) * 8, &out_ctx_phys, 8,
                       seq, offset, value) ||
              out_ctx_phys == 0) {
            PushCommandCompletion(trb_phys, usb::kCcParameterError, s);
            break;
          }
          const usb::InputControlContext* const icc =
              usb::InputCtrlCtx(in_ctx.data());
          const usb::SlotContext* const in_slot =
              usb::InputSlotCtx(in_ctx.data(), csz);
          const usb::EndpointContext* const in_ep0 =
              usb::InputEpCtx(in_ctx.data(), csz, 1);
          const int port = usb::SlotContextRootHubPort(*in_slot);
          if (icc->drop_flags != 0 || icc->add_flags != 0x3u ||
              usb::SlotContextEntries(*in_slot) != 1 || port < 1 ||
              port > static_cast<int>(ports_.size()) ||
              usb::EpContextType(*in_ep0) != usb::kEpTypeControlBiDir ||
              usb::EpContextDequeuePtr(*in_ep0) == 0) {
            PushCommandCompletion(trb_phys, usb::kCcParameterError, s);
            break;
          }
          SimSlot& slot = slots_[s];
          slot.port = port;
          slot.speed = usb::SlotContextSpeed(*in_slot);
          slot.context_entries = 1;
          slot.route_string = in_slot->dw0 & 0xFFFFFu;
          slot.mtt = ((in_slot->dw0 >> 25) & 1u) != 0;
          slot.hub = ((in_slot->dw0 >> 26) & 1u) != 0;
          slot.num_ports = static_cast<int>((in_slot->dw1 >> 24) & 0xFFu);
          slot.parent_hub_slot_id = static_cast<uint8_t>(in_slot->dw2 & 0xFFu);
          slot.parent_port_number =
              static_cast<uint8_t>((in_slot->dw2 >> 8) & 0xFFu);
          slot.ttt = static_cast<uint8_t>((in_slot->dw2 >> 16) & 0x3u);
          const bool bsr = (trb.dw3 & usb::kTrbBsr) != 0;
          SimUsbDevice* const target_dev = ResolveDeviceForSlot(slot);
          if (!bsr && target_dev != nullptr && target_dev->naks_set_address()) {
            HangCommand(trb, trb_phys);
            return false;
          }
          if (!bsr && (target_dev == nullptr || target_dev->address() != 0)) {
            // Only a device at the default address takes the SET_ADDRESS;
            // an addressed one ignores it, so it gets no handshake (xHCI
            // 4.6.5: USB Transaction Error) and the slot isn't addressed.
            PushCommandCompletion(trb_phys, usb::kCcUsbTransactionError, s);
            break;
          }
          for (int t = 1; t < static_cast<int>(slots_.size()); ++t) {
            if (t != s && slots_[static_cast<size_t>(t)].enabled &&
                slots_[static_cast<size_t>(t)].out_ctx_phys == out_ctx_phys) {
              Record(
                  Violation::kDeviceContextShared, seq, offset, value,
                  "slots " + std::to_string(t) + " and " + std::to_string(s));
            }
          }
          slot.out_ctx_phys = out_ctx_phys;
          slot.slot_state =
              bsr ? usb::kSlotStateDefault : usb::kSlotStateAddressed;
          slot.ep[1].state = usb::kEpStateRunning;
          slot.ep[1].type = usb::kEpTypeControlBiDir;
          slot.ep[1].max_packet_size = usb::EpContextMaxPacketSize(*in_ep0);
          slot.ep[1].avg_trb_len = usb::EpContextAvgTrbLength(*in_ep0);
          slot.ep[1].dequeue = usb::EpContextDequeuePtr(*in_ep0);
          slot.ep[1].dcs = usb::EpContextDcs(*in_ep0);

          usb::SlotContext out_slot = *in_slot;
          out_slot.dw3 =
              (static_cast<uint32_t>(slot.slot_state) << 27) | (bsr ? 0u : s);
          usb::EndpointContext out_ep0 = *in_ep0;
          out_ep0.dw0 = (out_ep0.dw0 & ~0x7u) | usb::kEpStateRunning;
          // From here on the xHC owns the Output Device Context.
          slot.out_ctx_shadow.assign(static_cast<size_t>(32 * csz), 0);
          ReadDma(out_ctx_phys, slot.out_ctx_shadow.data(), 32 * csz, seq,
                  offset, value);
          WriteOutputContext(slot, 0, &out_slot, sizeof(out_slot), seq, offset,
                             value);
          WriteOutputContext(slot, csz, &out_ep0, sizeof(out_ep0), seq, offset,
                             value);
          if (!bsr) {
            target_dev->OnSetAddress(s);
          }
          CheckUsb2Segment(port, seq, offset, value);
          PushCommandCompletion(trb_phys, usb::kCcSuccess, s);
          break;
        }
        case usb::kTrbEvaluateContext: {
          const uint8_t s = usb::TrbSlotId(trb);
          if (s < 1 || s > max_slots_en ||
              !slots_[static_cast<size_t>(s)].enabled) {
            PushCommandCompletion(trb_phys, usb::kCcSlotNotEnabled, s);
            break;
          }
          std::vector<uint8_t> in_ctx(static_cast<size_t>(33 * csz), 0);
          if (!ReadDma(usb::TrbPointer(trb), in_ctx.data(), 33 * csz, seq,
                       offset, value)) {
            PushCommandCompletion(trb_phys, usb::kCcParameterError, s);
            break;
          }
          const usb::InputControlContext* const icc =
              usb::InputCtrlCtx(in_ctx.data());
          SimSlot& slot = slots_[s];
          if ((icc->add_flags & 0x1u) != 0) {
            const usb::SlotContext* const in_slot =
                usb::InputSlotCtx(in_ctx.data(), csz);
            slot.mtt = ((in_slot->dw0 >> 25) & 1u) != 0;
            slot.hub = ((in_slot->dw0 >> 26) & 1u) != 0;
            slot.num_ports = static_cast<int>((in_slot->dw1 >> 24) & 0xFFu);
            slot.ttt = static_cast<uint8_t>((in_slot->dw2 >> 16) & 0x3u);
            usb::SlotContext out_slot{};
            ReadOutputContext(slot, 0, &out_slot, sizeof(out_slot), seq, offset,
                              value);
            out_slot.dw0 = (out_slot.dw0 & ~((1u << 26) | (1u << 25))) |
                           (in_slot->dw0 & ((1u << 26) | (1u << 25)));
            out_slot.dw1 =
                (out_slot.dw1 & 0x00FF'FFFFu) | (in_slot->dw1 & 0xFF00'0000u);
            out_slot.dw2 =
                (out_slot.dw2 & ~(0x3u << 16)) | (in_slot->dw2 & (0x3u << 16));
            WriteOutputContext(slot, 0, &out_slot, sizeof(out_slot), seq,
                               offset, value);
          }
          if ((icc->add_flags & 0x2u) != 0) {
            const usb::EndpointContext* const in_ep0 =
                usb::InputEpCtx(in_ctx.data(), csz, 1);
            slot.ep[1].max_packet_size = usb::EpContextMaxPacketSize(*in_ep0);
            usb::EndpointContext out_ep0{};
            ReadOutputContext(slot, csz, &out_ep0, sizeof(out_ep0), seq, offset,
                              value);
            out_ep0.dw1 =
                (out_ep0.dw1 & 0x0000FFFFu) |
                (static_cast<uint32_t>(slot.ep[1].max_packet_size) << 16);
            WriteOutputContext(slot, csz, &out_ep0, sizeof(out_ep0), seq,
                               offset, value);
          }
          PushCommandCompletion(trb_phys, usb::kCcSuccess, s);
          break;
        }
        case usb::kTrbConfigureEndpoint: {
          const uint8_t s = usb::TrbSlotId(trb);
          if (s < 1 || s > max_slots_en ||
              !slots_[static_cast<size_t>(s)].enabled) {
            PushCommandCompletion(trb_phys, usb::kCcSlotNotEnabled, s);
            break;
          }
          std::vector<uint8_t> in_ctx(static_cast<size_t>(33 * csz), 0);
          if (!ReadDma(usb::TrbPointer(trb), in_ctx.data(), 33 * csz, seq,
                       offset, value)) {
            PushCommandCompletion(trb_phys, usb::kCcParameterError, s);
            break;
          }
          const usb::InputControlContext* const icc =
              usb::InputCtrlCtx(in_ctx.data());
          const usb::SlotContext* const in_slot =
              usb::InputSlotCtx(in_ctx.data(), csz);
          if ((icc->add_flags & 0x1u) == 0 || (icc->add_flags & 0x2u) != 0) {
            PushCommandCompletion(trb_phys, usb::kCcParameterError, s);
            break;
          }
          SimSlot& slot = slots_[s];
          slot.context_entries = usb::SlotContextEntries(*in_slot);
          slot.route_string = in_slot->dw0 & 0xFFFFFu;
          slot.mtt = ((in_slot->dw0 >> 25) & 1u) != 0;
          slot.hub = ((in_slot->dw0 >> 26) & 1u) != 0;
          slot.num_ports = static_cast<int>((in_slot->dw1 >> 24) & 0xFFu);
          slot.parent_hub_slot_id = static_cast<uint8_t>(in_slot->dw2 & 0xFFu);
          slot.parent_port_number =
              static_cast<uint8_t>((in_slot->dw2 >> 8) & 0xFFu);
          slot.ttt = static_cast<uint8_t>((in_slot->dw2 >> 16) & 0x3u);
          slot.slot_state = usb::kSlotStateConfigured;
          for (int dci = 2; dci < 32; ++dci) {
            if ((icc->add_flags & (1u << dci)) != 0) {
              const usb::EndpointContext* const in_ep =
                  usb::InputEpCtx(in_ctx.data(), csz, dci);
              SimEndpoint& ep = slot.ep[dci];
              ep.state = usb::kEpStateRunning;
              ep.type = usb::EpContextType(*in_ep);
              ep.interval = usb::EpContextInterval(*in_ep);
              ep.max_burst = usb::EpContextMaxBurst(*in_ep);
              ep.max_packet_size = usb::EpContextMaxPacketSize(*in_ep);
              ep.avg_trb_len = usb::EpContextAvgTrbLength(*in_ep);
              ep.max_esit = usb::EpContextMaxEsitPayload(*in_ep);
              ep.dequeue = usb::EpContextDequeuePtr(*in_ep);
              ep.dcs = usb::EpContextDcs(*in_ep);
              ep.toggle = 0;
              usb::EndpointContext out_ep = *in_ep;
              out_ep.dw0 = (out_ep.dw0 & ~0x7u) | usb::kEpStateRunning;
              WriteOutputContext(slot, dci * csz, &out_ep, sizeof(out_ep), seq,
                                 offset, value);
            }
          }
          usb::SlotContext out_slot = *in_slot;
          out_slot.dw3 =
              (static_cast<uint32_t>(usb::kSlotStateConfigured) << 27) | s;
          WriteOutputContext(slot, 0, &out_slot, sizeof(out_slot), seq, offset,
                             value);
          PushCommandCompletion(trb_phys, usb::kCcSuccess, s);
          break;
        }
        case usb::kTrbResetEndpoint: {
          const uint8_t s = usb::TrbSlotId(trb);
          const uint8_t dci = usb::TrbEndpointId(trb);
          if (s < 1 || s > max_slots_en ||
              !slots_[static_cast<size_t>(s)].enabled || dci < 1 || dci >= 32) {
            PushCommandCompletion(trb_phys, usb::kCcParameterError, s);
            break;
          }
          SimSlot& slot = slots_[s];
          if (slot.ep[dci].state != usb::kEpStateHalted) {
            PushCommandCompletion(trb_phys, usb::kCcContextStateError, s);
            break;
          }
          // TSP = 1 (Soft Retry) keeps the transfer state, so the TD at the
          // dequeue pointer is retried with the same data toggle; TSP = 0
          // resets the host's toggle / sequence number (xHCI 4.6.8).
          if ((trb.dw3 & usb::kTrbTsp) == 0) {
            slot.ep[dci].toggle = 0;
          }
          slot.ep[dci].state = usb::kEpStateStopped;
          UpdateOutputEpState(slot, dci, csz, seq, offset, value);
          PushCommandCompletion(trb_phys, usb::kCcSuccess, s);
          break;
        }
        case usb::kTrbStopEndpoint: {
          const uint8_t s = usb::TrbSlotId(trb);
          const uint8_t dci = usb::TrbEndpointId(trb);
          ++stop_endpoint_count_;
          if (s < 1 || s > max_slots_en ||
              !slots_[static_cast<size_t>(s)].enabled || dci < 1 || dci >= 32) {
            PushCommandCompletion(trb_phys, usb::kCcParameterError, s);
            break;
          }
          SimSlot& slot = slots_[s];
          if (slot.ep[dci].state != usb::kEpStateRunning) {
            // Only a Running endpoint can be stopped; a Halted, Stopped or
            // Error one is left as it is (xHCI 4.6.9).
            PushCommandCompletion(trb_phys, usb::kCcContextStateError, s);
            break;
          }
          slot.ep[dci].state = usb::kEpStateStopped;
          UpdateOutputEpState(slot, dci, csz, seq, offset, value);
          PushCommandCompletion(trb_phys, usb::kCcSuccess, s);
          break;
        }
        case usb::kTrbSetTrDequeue: {
          const uint8_t s = usb::TrbSlotId(trb);
          const uint8_t dci = usb::TrbEndpointId(trb);
          if (s < 1 || s > max_slots_en ||
              !slots_[static_cast<size_t>(s)].enabled || dci < 1 || dci >= 32) {
            PushCommandCompletion(trb_phys, usb::kCcParameterError, s);
            break;
          }
          SimSlot& slot = slots_[s];
          if (slot.ep[dci].state != usb::kEpStateStopped &&
              slot.ep[dci].state != usb::kEpStateError) {
            PushCommandCompletion(trb_phys, usb::kCcContextStateError, s);
            break;
          }
          const uint64_t raw = usb::TrbPointer(trb);
          slot.ep[dci].dequeue = raw & ~0xFull;
          slot.ep[dci].dcs = static_cast<uint32_t>(raw & 1u);
          slot.ep[dci].state = usb::kEpStateStopped;
          UpdateOutputEpState(slot, dci, csz, seq, offset, value);
          PushCommandCompletion(trb_phys, usb::kCcSuccess, s);
          break;
        }
        case usb::kTrbNoOpCmd:
          PushCommandCompletion(trb_phys, usb::kCcSuccess, 0);
          break;
        default:
          PushCommandCompletion(trb_phys, usb::kCcTrbError, 0);
          break;
      }
    }
    return true;
  }

  // The Output Device Context of `slot` at `ctx_offset`: the xHC's own copy
  // (the shadow) once it owns the context, else memory.
  void ReadOutputContext(const SimSlot& slot,       //
                         const int64_t ctx_offset,  //
                         void* const dst,           //
                         const int64_t len,         //
                         const int64_t seq,         //
                         const int64_t offset,      //
                         const uint32_t value) {
    if (ctx_offset >= 0 && len >= 0 &&
        ctx_offset + len <= static_cast<int64_t>(slot.out_ctx_shadow.size())) {
      std::memcpy(dst, slot.out_ctx_shadow.data() + ctx_offset,
                  static_cast<size_t>(len));
      return;
    }
    ReadDma(slot.out_ctx_phys + static_cast<uint64_t>(ctx_offset), dst, len,
            seq, offset, value);
  }

  void WriteOutputContext(SimSlot& slot,             //
                          const int64_t ctx_offset,  //
                          const void* const src,     //
                          const int64_t len,         //
                          const int64_t seq,         //
                          const int64_t offset,      //
                          const uint32_t value) {
    WriteDma(slot.out_ctx_phys + static_cast<uint64_t>(ctx_offset), src, len,
             seq, offset, value);
    if (ctx_offset >= 0 && len >= 0 &&
        ctx_offset + len <= static_cast<int64_t>(slot.out_ctx_shadow.size())) {
      std::memcpy(slot.out_ctx_shadow.data() + ctx_offset, src,
                  static_cast<size_t>(len));
    }
  }

  // Software must not write the Output Device Context of an enabled slot
  // (xHCI 4.5.3, 6.1), e.g. reuse its memory for another device.
  void CheckOutputContext(const SimSlot& slot,   //
                          const int slot_id,     //
                          const int64_t seq,     //
                          const int64_t offset,  //
                          const uint32_t value) {
    if (slot.out_ctx_shadow.empty()) {
      return;
    }
    std::vector<uint8_t> current(slot.out_ctx_shadow.size(), 0);
    if (ReadDma(slot.out_ctx_phys, current.data(),
                static_cast<int64_t>(current.size()), seq, offset, value) &&
        current != slot.out_ctx_shadow) {
      Record(Violation::kDeviceContextModified, seq, offset, value,
             "slot " + std::to_string(slot_id));
    }
  }

  void UpdateOutputEpState(SimSlot& slot,         //
                           const uint8_t dci,     //
                           const int csz,         //
                           const int64_t seq,     //
                           const int64_t offset,  //
                           const uint32_t value) {
    if (slot.out_ctx_phys == 0) {
      return;
    }
    usb::EndpointContext out_ep{};
    const int64_t ctx_offset = static_cast<int64_t>(dci) * csz;
    ReadOutputContext(slot, ctx_offset, &out_ep, sizeof(out_ep), seq, offset,
                      value);
    out_ep.dw0 = (out_ep.dw0 & ~0x7u) | (slot.ep[dci].state & 0x7u);
    out_ep.tr_dequeue_low =
        static_cast<uint32_t>(slot.ep[dci].dequeue & ~0xFull) |
        (slot.ep[dci].dcs & 1u);
    out_ep.tr_dequeue_high = static_cast<uint32_t>(slot.ep[dci].dequeue >> 32);
    WriteOutputContext(slot, ctx_offset, &out_ep, sizeof(out_ep), seq, offset,
                       value);
  }

  bool PopEndpointTrb(SimEndpoint& ep,         //
                      usb::Trb* const out,     //
                      uint64_t* const out_pa,  //
                      const int64_t seq,       //
                      const int64_t offset,    //
                      const uint32_t value) {
    for (int hop = 0; hop < 4; ++hop) {
      usb::Trb trb{};
      if (!ReadDma(ep.dequeue, &trb, sizeof(trb), seq, offset, value)) {
        return false;
      }
      if ((trb.dw3 & usb::kTrbCycle) != ep.dcs) {
        return false;
      }
      if (usb::TrbType(trb) == usb::kTrbLink) {
        ep.dequeue = usb::TrbPointer(trb) & ~0xFull;
        if ((trb.dw3 & usb::kTrbToggleCycle) != 0) {
          ep.dcs ^= 1u;
        }
        continue;
      }
      *out = trb;
      *out_pa = ep.dequeue;
      ep.dequeue += 16;
      return true;
    }
    return false;
  }

  bool PeekEndpointTrb(SimEndpoint& ep,         //
                       usb::Trb* const out,     //
                       uint64_t* const out_pa,  //
                       const int64_t seq,       //
                       const int64_t offset,    //
                       const uint32_t value) {
    for (int hop = 0; hop < 4; ++hop) {
      usb::Trb trb{};
      if (!ReadDma(ep.dequeue, &trb, sizeof(trb), seq, offset, value)) {
        return false;
      }
      if ((trb.dw3 & usb::kTrbCycle) != ep.dcs) {
        return false;
      }
      if (usb::TrbType(trb) == usb::kTrbLink) {
        ep.dequeue = usb::TrbPointer(trb) & ~0xFull;
        if ((trb.dw3 & usb::kTrbToggleCycle) != 0) {
          ep.dcs ^= 1u;
        }
        continue;
      }
      *out = trb;
      *out_pa = ep.dequeue;
      return true;
    }
    return false;
  }

  SimUsbDevice* ResolveDeviceForSlot(const SimSlot& slot) const {
    if (slot.port < 1 || slot.port > static_cast<int>(ports_.size())) {
      return nullptr;
    }
    SimUsbDevice* dev = ports_[static_cast<size_t>(slot.port - 1)].device;
    uint32_t route = slot.route_string & 0xFFFFFu;
    uint32_t expected_tt_route = 0;
    int expected_tt_port = 0;
    for (int tier = 0; tier < 5 && route != 0 && dev != nullptr; ++tier) {
      const int hub_port = static_cast<int>(route & 0xFu);
      if (hub_port == 0) {
        return nullptr;
      }
      if (dev->speed() == usb::kPortSpeedHigh) {
        expected_tt_route =
            slot.route_string & ((uint32_t{1} << (tier * 4)) - 1u);
        expected_tt_port = hub_port;
      }
      dev = dev->ResolveDownstreamDevice(hub_port);
      route >>= 4;
    }
    if (dev == nullptr) {
      return nullptr;
    }
    // Split transactions to an LS/FS device behind a High-Speed hub use the
    // slot's Parent Hub Slot ID and Parent Port Number to reach the TT (xHCI
    // 6.2.2); other devices must leave both fields 0.
    const bool needs_tt = (slot.speed == usb::kPortSpeedLow ||
                           slot.speed == usb::kPortSpeedFull) &&
                          expected_tt_port != 0;
    if (needs_tt) {
      const int hs = slot.parent_hub_slot_id;
      if (hs < 1 || hs >= static_cast<int>(slots_.size())) {
        return nullptr;
      }
      const SimSlot& tt = slots_[static_cast<size_t>(hs)];
      if (!tt.enabled || !tt.hub || tt.port != slot.port ||
          (tt.route_string & 0xFFFFFu) != expected_tt_route ||
          slot.parent_port_number != expected_tt_port) {
        return nullptr;
      }
    } else if (slot.parent_hub_slot_id != 0 || slot.parent_port_number != 0) {
      return nullptr;
    }
    return dev;
  }

  void ProcessEp0Ring(const uint8_t slot_id,  //
                      const int64_t seq,      //
                      const int64_t offset,   //
                      const uint32_t value) {
    SimSlot& slot = slots_[slot_id];
    SimEndpoint& ep0 = slot.ep[1];
    if (ep0.state != usb::kEpStateRunning) {
      return;
    }
    usb::Trb setup_trb{};
    uint64_t setup_pa = 0;
    if (!PopEndpointTrb(ep0, &setup_trb, &setup_pa, seq, offset, value)) {
      return;
    }
    if (usb::TrbType(setup_trb) != usb::kTrbSetupStage ||
        (setup_trb.dw3 & usb::kTrbIdt) == 0) {
      PushTransferEvent(setup_pa, usb::kCcTrbError, 0, slot_id, 1);
      return;
    }
    if (config_.hci_version < 0x0100 && ((setup_trb.dw3 >> 16) & 0x3u) != 0) {
      // Transfer Type exists since xHCI 1.0; a 0.96 controller may reject
      // its RsvdZ bits.
      ep0.state = usb::kEpStateHalted;
      UpdateOutputEpState(slot, 1, config_.csz64 ? 64 : 32, seq, offset, value);
      PushTransferEvent(setup_pa, usb::kCcTrbError, 0, slot_id, 1);
      return;
    }

    SetupPacket setup{};
    setup.bm_request_type = static_cast<uint8_t>(setup_trb.dw0 & 0xFFu);
    setup.b_request = static_cast<uint8_t>((setup_trb.dw0 >> 8) & 0xFFu);
    setup.w_value = static_cast<uint16_t>((setup_trb.dw0 >> 16) & 0xFFFFu);
    setup.w_index = static_cast<uint16_t>(setup_trb.dw1 & 0xFFFFu);
    setup.w_length = static_cast<uint16_t>((setup_trb.dw1 >> 16) & 0xFFFFu);

    bool has_data = false;
    usb::Trb data_trb{};
    uint64_t data_pa = 0;
    usb::Trb status_trb{};
    uint64_t status_pa = 0;

    usb::Trb next_trb{};
    uint64_t next_pa = 0;
    if (!PopEndpointTrb(ep0, &next_trb, &next_pa, seq, offset, value)) {
      PushTransferEvent(setup_pa, usb::kCcTrbError, 0, slot_id, 1);
      return;
    }
    if (usb::TrbType(next_trb) == usb::kTrbDataStage) {
      has_data = true;
      data_trb = next_trb;
      data_pa = next_pa;
      if (!PopEndpointTrb(ep0, &status_trb, &status_pa, seq, offset, value) ||
          usb::TrbType(status_trb) != usb::kTrbStatusStage) {
        PushTransferEvent(data_pa, usb::kCcTrbError, 0, slot_id, 1);
        return;
      }
    } else if (usb::TrbType(next_trb) == usb::kTrbStatusStage) {
      status_trb = next_trb;
      status_pa = next_pa;
    } else {
      PushTransferEvent(next_pa, usb::kCcTrbError, 0, slot_id, 1);
      return;
    }

    SimUsbDevice* const dev = ResolveDeviceForSlot(slot);
    if (dev == nullptr || AddressCollides(slot, *dev)) {
      ep0.state = usb::kEpStateHalted;
      UpdateOutputEpState(slot, 1, config_.csz64 ? 64 : 32, seq, offset, value);
      PushTransferEvent(has_data ? data_pa : status_pa,
                        usb::kCcUsbTransactionError, setup.w_length, slot_id,
                        1);
      return;
    }

    // For Full-Speed devices, any control transfer > 8 bytes requires EP0's
    // max_packet_size in the Endpoint Context to match the device's
    // bMaxPacketSize0 (verifying that Evaluate Context ran when MPS != 8).
    if (dev->speed() == 1 && setup.w_length > 8 &&
        ep0.max_packet_size != dev->ep0_max_packet_size()) {
      ep0.state = usb::kEpStateHalted;
      UpdateOutputEpState(slot, 1, config_.csz64 ? 64 : 32, seq, offset, value);
      PushTransferEvent(has_data ? data_pa : status_pa,
                        usb::kCcUsbTransactionError, setup.w_length, slot_id,
                        1);
      return;
    }

    std::vector<uint8_t> buf(
        static_cast<size_t>(setup.w_length > 0 ? setup.w_length : 1), 0);
    uint16_t actual_len = 0;
    const ControlOutcome outcome =
        dev->OnControl(setup, buf.data(), setup.w_length, &actual_len);
    // Hub port resets and status reads move devices into the default state.
    CheckUsb2Segment(slot.port, seq, offset, value);
    if (outcome == ControlOutcome::kNoResponse) {
      return;
    }
    if (outcome != ControlOutcome::kAck) {
      ep0.state = usb::kEpStateHalted;
      UpdateOutputEpState(slot, 1, config_.csz64 ? 64 : 32, seq, offset, value);
      const uint8_t cc = outcome == ControlOutcome::kStall
                             ? usb::kCcStallError
                             : (outcome == ControlOutcome::kBabble
                                    ? usb::kCcBabbleDetected
                                    : usb::kCcUsbTransactionError);
      const uint64_t err_pa = dev->stall_on_setup_stage()
                                  ? setup_pa
                                  : (has_data ? data_pa : status_pa);
      PushTransferEvent(err_pa, cc, setup.w_length, slot_id, 1);
      return;
    }

    if (has_data) {
      const uint32_t trb_len = data_trb.dw2 & 0x1FFFFu;
      const uint16_t copy_len =
          actual_len < trb_len ? actual_len : static_cast<uint16_t>(trb_len);
      if (copy_len > 0) {
        WriteDma(usb::TrbPointer(data_trb), buf.data(), copy_len, seq, offset,
                 value);
      }
      if (copy_len < trb_len && (data_trb.dw3 & usb::kTrbIsp) != 0) {
        PushTransferEvent(data_pa, usb::kCcShortPacket, trb_len - copy_len,
                          slot_id, 1);
      }
    }
    PushTransferEvent(status_pa, usb::kCcSuccess, 0, slot_id, 1);
  }

  void ServiceInterruptEndpoints() {
    for (int s = 1; s < static_cast<int>(slots_.size()); ++s) {
      SimSlot& slot = slots_[static_cast<size_t>(s)];
      if (!slot.enabled) {
        continue;
      }
      for (uint8_t dci = 2; dci < 32; ++dci) {
        if (slot.ep[dci].state == usb::kEpStateRunning &&
            slot.ep[dci].doorbell_rung) {
          ServiceInterruptEndpoint(static_cast<uint8_t>(s), dci, 0, 0, 0);
        }
      }
    }
  }

  void ServiceInterruptEndpoint(const uint8_t slot_id,  //
                                const uint8_t dci,      //
                                const int64_t seq,      //
                                const int64_t offset,   //
                                const uint32_t value) {
    SimSlot& slot = slots_[slot_id];
    SimEndpoint& ep = slot.ep[dci];
    if (ep.state != usb::kEpStateRunning || !ep.doorbell_rung) {
      return;
    }
    SimUsbDevice* const dev = ResolveDeviceForSlot(slot);
    if (dev == nullptr) {
      return;
    }
    if (AddressCollides(slot, *dev)) {
      usb::Trb trb{};
      uint64_t trb_pa = 0;
      if (!PeekEndpointTrb(ep, &trb, &trb_pa, seq, offset, value)) {
        return;
      }
      ep.state = usb::kEpStateHalted;
      UpdateOutputEpState(slot, dci, config_.csz64 ? 64 : 32, seq, offset,
                          value);
      PushTransferEvent(trb_pa, usb::kCcUsbTransactionError, trb.dw2 & 0x1FFFFu,
                        slot_id, dci);
      return;
    }

    int served = 0;
    while (served++ < 16 && ep.state == usb::kEpStateRunning) {
      usb::Trb trb{};
      uint64_t trb_pa = 0;
      if (!PeekEndpointTrb(ep, &trb, &trb_pa, seq, offset, value)) {
        return;
      }
      if (usb::TrbType(trb) != usb::kTrbNormal) {
        ep.dequeue += 16;
        PushTransferEvent(trb_pa, usb::kCcTrbError, 0, slot_id, dci);
        return;
      }
      const uint32_t req_len = trb.dw2 & 0x1FFFFu;
      std::vector<uint8_t> buf(static_cast<size_t>(req_len > 0 ? req_len : 1),
                               0);
      uint16_t actual_len = 0;
      const IntrOutcome outcome =
          dev->PollInterruptIn(dci, buf.data(), static_cast<uint16_t>(req_len),
                               &actual_len, clock_->now());
      if (outcome == IntrOutcome::kNak) {
        return;
      }
      if (outcome == IntrOutcome::kData) {
        const int device_toggle = dev->InterruptInToggle(dci);
        if (device_toggle >= 0) {
          // The host ACKs every data packet, also one it drops as a
          // retransmission because its toggle (sequence number) is not the
          // expected one: the device moves on either way (USB 2.0 8.6.4).
          dev->OnInterruptInAck(dci);
          if (device_toggle != ep.toggle) {
            ++dropped_in_packets_;
            continue;  // The data is lost; the TRB stays armed
          }
          ep.toggle ^= 1u;
        }
        ep.dequeue += 16;
        const uint16_t copy_len =
            actual_len < req_len ? actual_len : static_cast<uint16_t>(req_len);
        if (copy_len > 0) {
          WriteDma(usb::TrbPointer(trb), buf.data(), copy_len, seq, offset,
                   value);
        }
        const uint32_t residual = req_len - copy_len;
        const uint8_t cc = (residual > 0 && (trb.dw3 & usb::kTrbIsp) != 0)
                               ? usb::kCcShortPacket
                               : usb::kCcSuccess;
        if ((trb.dw3 & usb::kTrbIoc) != 0 || cc == usb::kCcShortPacket) {
          PushTransferEvent(trb_pa, cc, residual, slot_id, dci);
        }
      } else {
        // A halting error: the dequeue pointer stays on the TD that failed,
        // which a Soft Retry retries (xHCI 4.10.2, 4.6.8.1).
        ep.state = usb::kEpStateHalted;
        UpdateOutputEpState(slot, dci, config_.csz64 ? 64 : 32, seq, offset,
                            value);
        const uint8_t cc = outcome == IntrOutcome::kStall
                               ? usb::kCcStallError
                               : (outcome == IntrOutcome::kBabble
                                      ? usb::kCcBabbleDetected
                                      : usb::kCcUsbTransactionError);
        PushTransferEvent(trb_pa, cc, req_len, slot_id, dci);
        return;
      }
    }
  }

  // A USBLEGSUP read after the OS's request: firmware that releases after N
  // polls does so on the Nth read.
  void PollLegSup() {
    if (!bios_owned_ || request_tick_ < 0) {
      return;
    }
    ++polls_since_request_;
    if (can_release_ &&
        config_.legacy.behavior == BiosBehavior::kReleasesAfterPolls &&
        polls_since_request_ >= config_.legacy.release_after_polls) {
      ReleaseByBios();
    }
  }

  void ReleaseByBios() {
    bios_owned_ = false;
    released_by_bios_ = true;
    if (config_.legacy.bios_disables_smis_on_release) {
      ctlsts_ &= ~spec::kSmiEnableMask;
    }
  }

  void RecordOwnershipRequest(const int64_t seq) {
    request_tick_ = clock_->now();
    request_seq_ = seq;
    polls_since_request_ = 0;
    can_release_ = !config_.legacy.release_requires_os_ownership_smi ||
                   (ctlsts_ & spec::kSmiOnOsOwnershipEnable) != 0;
  }

  void WriteLegSup(const uint32_t value,  //
                   const int64_t seq,     //
                   const int64_t offset) {
    if ((value & spec::kLegSupRsvdPMask) !=
        (config_.legacy.legsup_rsvdp & spec::kLegSupRsvdPMask)) {
      Record(Violation::kRsvdPChanged, seq, offset, value);
    }
    const bool os = (value & spec::kOsOwned) != 0;
    const bool bios = (value & spec::kBiosOwned) != 0;
    if (os != os_owned_) {
      os_owned_ = os;
      ctlsts_ |= spec::kSmiOnOsOwnershipChange;
      if (os) {
        RecordOwnershipRequest(seq);
      }
    }
    if (bios_owned_ && !bios) {
      if (request_tick_ < 0 ||
          clock_->now() < request_tick_ + config_.legacy.force_floor_ticks) {
        Record(Violation::kBiosSemaphoreClearedEarly, seq, offset, value);
      } else {
        forced_ = true;
      }
      bios_owned_ = false;
    } else if (!bios_owned_ && bios) {
      Record(Violation::kBiosSemaphoreSetBySoftware, seq, offset, value);
      bios_owned_ = true;
    }
  }

  void WriteLegCtlSts(const uint32_t value,  //
                      const int64_t seq,     //
                      const int64_t offset) {
    if ((value & spec::kLegCtlStsRsvdPMask) !=
        (ctlsts_ & spec::kLegCtlStsRsvdPMask)) {
      Record(Violation::kRsvdPChanged, seq, offset, value);
    }
    if ((value & spec::kLegCtlStsRsvdZMask) != 0) {
      Record(Violation::kRsvdZWrittenOne, seq, offset, value);
    }
    ctlsts_ =
        (ctlsts_ & ~spec::kSmiEnableMask) | (value & spec::kSmiEnableMask);
    ctlsts_ &= ~(value & spec::kSmiEventMask);
  }

  SimClock* clock_;
  XhciSimConfig config_;
  int64_t start_tick_;
  int legacy_cap_ = -1;
  bool decoding_ = true;

  int64_t next_seq_ = 1;
  std::vector<Access> accesses_;
  Access previous_access_;
  std::vector<ViolationRecord> violations_;
  std::vector<Violation> allowed_;

  // Operational, runtime and port registers.
  uint32_t usbcmd_ = 0;
  uint32_t usbsts_pending_ = 0;
  bool hce_ = false;
  uint32_t dnctrl_ = 0;
  uint64_t crcr_ = 0;
  uint64_t dcbaap_ = 0;
  uint32_t config_register_ = 0;
  std::vector<Port> ports_;
  std::vector<Interrupter> interrupters_;

  // Run/Stop and reset state machines.
  RunState run_state_ = RunState::kHalted;
  int64_t halt_due_tick_ = 0;
  int64_t start_due_tick_ = 0;
  bool awaiting_halt_observation_ = false;
  int64_t halt_observed_seq_ = -1;
  ResetPhase reset_phase_ = ResetPhase::kIdle;
  int64_t reset_due_tick_ = 0;
  int64_t ready_due_tick_ = 0;
  int reset_count_ = 0;
  int64_t reset_write_seq_ = -1;
  int64_t reset_write_tick_ = -1;
  bool hcrst_clear_observed_ = false;
  bool reset_observed_ = false;
  int64_t reset_observed_seq_ = -1;

  // Phase-2 DMA, event ring, command ring, and slots.
  SimDmaArena* dma_ = nullptr;
  bool bus_master_ = true;
  // Command Ring Running (CRR), and the command in progress: delayed
  // (command_delay_ticks, due at cmd_exec_due_tick_) or hung until aborted.
  bool cmd_ring_running_ = false;
  bool cmd_executing_ = false;
  bool cmd_exec_hung_ = false;
  usb::Trb cmd_exec_trb_ = {};
  uint64_t cmd_exec_phys_ = 0;
  int64_t cmd_exec_due_tick_ = 0;
  bool cmd_stop_after_current_ = false;
  // The last write was a CRCR low dword with CS or CA.
  bool crcr_abort_low_written_ = false;
  bool erdp_ehb_written_ = false;
  uint64_t cmd_dequeue_ = 0;
  uint32_t cmd_ccs_ = 1;
  uint64_t evt_seg_base_ = 0;
  uint32_t evt_seg_size_ = 0;
  int evt_enqueue_ = 0;
  uint32_t evt_pcs_ = 1;
  int evt_unconsumed_ = 0;
  bool evt_full_reported_ = false;
  std::deque<usb::Trb> pending_events_;
  int irq_count_ = 0;
  bool irq_asserted_ = false;
  std::vector<SimSlot> slots_{65};
  int last_enabled_slot_ = 0;
  int max_default_state_devices_ = 0;
  int stop_endpoint_count_ = 0;
  int dropped_in_packets_ = 0;
  std::vector<int> disable_slot_log_;

  // USB Legacy Support state.
  bool bios_owned_ = false;
  bool os_owned_ = false;
  uint32_t ctlsts_ = 0;
  int64_t request_tick_ = -1;
  int64_t request_seq_ = -1;
  int polls_since_request_ = 0;
  bool can_release_ = false;
  bool forced_ = false;
  bool released_by_bios_ = false;
};

}  // namespace protos::xhci_sim

#endif  // PROTOS_USER_USBKBD_XHCI_SIM_H_
