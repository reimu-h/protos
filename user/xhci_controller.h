#ifndef PROTOS_USER_XHCI_CONTROLLER_H_
#define PROTOS_USER_XHCI_CONTROLLER_H_

#include <cstdint>
#include <type_traits>

#include "user/usb_descriptors.h"
#include "user/usb_hid.h"
#include "user/xhci_context.h"
#include "user/xhci_platform.h"
#include "user/xhci_regs.h"
#include "user/xhci_ring.h"

// One xHCI controller of the ring-3 USB keyboard driver. Freestanding,
// header-only and without system calls (see user/xhci_platform.h).
//
// Bring-up has two phases, so that the program enables bus mastering only
// after the controller has stopped using whatever state firmware left in it:
// 1. TakeOwnershipAndReset() takes the controller over from firmware (USB
//    Legacy Support handoff), halts it if it runs, and resets it. No DMA.
//    Note: pci_claim leaves PCI Bus Master as firmware set it; if Phase 1
//    returns an error after handoff, the caller clears Bus Master.
// 2. The caller enables PCI Bus Master and allocates the controller's
//    interrupt (MSI-X preferred, then MSI) BEFORE calling Start(IrqMode),
//    which allocates DMA rings/contexts, starts the controller, powers and
//    scans ports, and drives non-blocking enumeration and HID input via
//    HandleInterrupt() and NextDeadline().

namespace protos::usb {

// Phase-1 timeouts in milliseconds.
// Firmware must release the HC BIOS Owned semaphore within this time after
// the OS asks for ownership; otherwise the driver takes ownership by force
// (like Linux).
constexpr int64_t kHandoffTimeoutMs = 1000;
// USBSTS.CNR must clear before the first operational register write. On a
// timeout the driver logs it and continues, like Linux; the reset that follows
// has bounded waits of its own.
constexpr int64_t kControllerReadyTimeoutMs = 5000;
// A running controller halts within 16 ms after Run/Stop is cleared (xHCI
// 5.4.1); Linux allows 32 ms. A controller that does not halt is not reset.
constexpr int64_t kHaltTimeoutMs = 32;
// No register access for at least this long after HCRST is set (Intel
// controllers need 1 ms; EDK2 waits on all controllers).
constexpr int64_t kPostResetDelayMs = 1;
// After HCRST is set, HCRST must clear and then CNR must clear, each within
// this time (Linux XHCI_RESET_LONG_USEC).
constexpr int64_t kResetTimeoutMs = 10000;
constexpr int64_t kResetReadyTimeoutMs = 10000;

// Phase-2 timeouts and limits.
constexpr int64_t kNoDeadline = INT64_MAX;
constexpr int64_t kStartTimeoutMs = 32;
constexpr int64_t kPortPowerSettleMs = 20;
constexpr int64_t kPortDebounceMs = 100;
constexpr int64_t kPortDebounceMaxMs = 2000;
constexpr int64_t kPortResetTimeoutMs = 800;
constexpr int64_t kPortResetRecoveryMs = 50;
constexpr int64_t kAddressSettleMs = 10;
constexpr int64_t kCommandTimeoutMs = 5000;
constexpr int64_t kControlTimeoutMs = 5000;

constexpr int kMaxPorts = 32;
constexpr int kMaxDevices = 8;
constexpr int kRingTrbs = 256;
constexpr int kReportTrbs = 8;
constexpr int kReportBufferSize = 512;
constexpr int kControlBufferSize = 1024;
constexpr int kMaxEnumerationAttempts = 3;
constexpr int kMaxInterruptErrors = 3;
constexpr int kMaxRecoveries = 3;

// The longest controller label (e.g. "bb:dd.f") kept for log lines.
constexpr int kMaxControllerLabelLength = 15;

enum class XhciState : uint8_t {
  kUnbound = 0,  // Init() not called yet
  kBound,        // Bound to its platform and registers
  kReset,        // TakeOwnershipAndReset() succeeded: OS-owned, halted, reset
  kError,        // Unrecoverable error: leave the controller alone
  kRunning,      // Start() succeeded: rings active, interrupts enabled
};

// How TakeOwnershipAndReset() took the controller over from firmware.
enum class HandoffOutcome : uint8_t {
  kNotRun = 0,
  kNoLegacyCapability,  // No USB Legacy Support capability (e.g. QEMU)
  kNotBiosOwned,        // Firmware did not own it: claimed without waiting
  kReleased,            // Firmware released it within kHandoffTimeoutMs
  kForced,              // Firmware did not release it: ownership forced
};

enum class IrqMode : uint8_t {
  kMsix = 0,
  kMsi,
};

inline const char* IrqModeName(const IrqMode mode) {
  return mode == IrqMode::kMsix ? "msix" : "msi";
}

inline const char* SpeedName(const int speed) {
  switch (speed) {
    case kPortSpeedLow:
      return "ls";
    case kPortSpeedFull:
      return "fs";
    case kPortSpeedHigh:
      return "hs";
    case kPortSpeedSuper:
    case kPortSpeedSuperPlus:
      return "ss";
    default:
      return "unknown";
  }
}

class XhciController {
 public:
  // constexpr, so that a program can keep its controllers in a constinit
  // array (crt never runs dynamic initializers).
  constexpr XhciController() = default;
  // A controller object stands for the hardware; never copy it.
  XhciController(const XhciController&) = delete;
  XhciController& operator=(const XhciController&) = delete;

  // Binds the controller to the program's services and to its registers;
  // both must outlive this object. `label` (e.g. "00:04.0") names the
  // controller in log lines; its first kMaxControllerLabelLength characters
  // are kept. `locks` optionally shares Caps Lock state across controllers
  // (defaults to process-wide `g_default_keyboard_locks`).
  void Init(Platform& platform,  //
            MmioWindow& mmio,    //
            const char* label,   //
            KeyboardLocks* locks = nullptr);

  // Phase 1 of the bring-up, in this order:
  // 1. Decodes and checks the capability registers and the extended
  //    capabilities (DecodeCaps(), ParseExtendedCaps()).
  // 2. USB Legacy Support handoff, if the capability exists: sets HC OS
  //    Owned, waits up to kHandoffTimeoutMs for firmware to clear HC BIOS
  //    Owned and on a timeout clears it itself (keeping OS Owned set), then
  //    disables all legacy SMIs and clears the SMI events.
  // 3. Waits up to kControllerReadyTimeoutMs for USBSTS.CNR = 0 (logs a
  //    timeout and continues).
  // 4. Clears USBCMD Run/Stop and the interrupt enables and, if the
  //    controller runs, waits up to kHaltTimeoutMs for USBSTS.HCH = 1. A
  //    controller that does not halt is abandoned without a reset.
  // 5. Sets USBCMD.HCRST, accesses no register for kPostResetDelayMs, then
  //    waits for HCRST = 0 and CNR = 0 and checks that HCH = 1.
  // Allocates no DMA memory and needs no bus mastering. Every wait is
  // bounded and stops early when registers read 0xFFFFFFFF. On an error,
  // logs "[USBKBD] <label>: error: <status name>", sets state kError and
  // writes nothing more. Can be called again in any state (the handoff then
  // finds the controller OS-owned and does not wait).
  [[nodiscard]] XhciStatus TakeOwnershipAndReset();

  // Phase 2 of the bring-up: requires state() == XhciState::kReset, PCI Bus
  // Master enabled, and the controller's interrupt allocated. Allocates and
  // zeroes DMA structures (DCBAA, scratchpads, command/event/transfer rings,
  // device contexts), programs registers, sets USBCMD.INTE | USBCMD.RS,
  // powers ports if PPC=1, emits "[USBKBD] controller <label> ready (<mode>)",
  // and scans all root-hub ports.
  [[nodiscard]] XhciStatus Start(IrqMode mode);

  // Drains Interrupter 0's event ring (acknowledging USBSTS.EINT, IMAN.IP, and
  // ERDP.EHB) and advances all port, command, and device state machines whose
  // deadlines are due at Platform::NowTicks(). Safe to call on an interrupt,
  // a deadline expiry, or a spurious wakeup.
  void HandleInterrupt();

  // Earliest pending absolute tick deadline across all ports, commands, and
  // device state machines, or kNoDeadline (INT64_MAX) when idle.
  int64_t NextDeadline() const;

  XhciState state() const { return state_; }
  const XhciCaps& caps() const { return caps_; }
  const char* label() const { return label_; }
  HandoffOutcome handoff_outcome() const { return handoff_; }
  IrqMode irq_mode() const { return irq_mode_; }
  int attached_keyboard_count() const { return attached_keyboards_; }

 private:
  enum class PollResult : uint8_t { kMatched, kTimedOut, kNotDecoding };

  enum class PortFsm : uint8_t {
    kDisconnected = 0,
    kDebouncing,
    kResetting,
    kResetRecovery,
    kEnumerating,
    kActive,
    kDisabled,
  };

  enum class DevFsm : uint8_t {
    kFree = 0,
    kWantEnableSlot,
    kWaitingEnableSlot,
    kWantAddressDevice,
    kWaitingAddressDevice,
    kAddressSettle,
    kWaitingGetDesc8,
    kWantEvaluateEp0,
    kWaitingEvaluateEp0,
    kWaitingGetDesc18,
    kWaitingGetConfig9,
    kWaitingGetConfigFull,
    kWantConfigureEndpoint,
    kWaitingConfigureEndpoint,
    kWaitingSetConfig,
    kWaitingSetProtocol,
    kWaitingSetIdle,
    kConfigured,
    kWantResetEp0,
    kWaitingResetEp0,
    kWantSetDeqEp0,
    kWaitingSetDeqEp0,
    kWantResetIntrEp,
    kWaitingResetIntrEp,
    kWantSetDeqIntrEp,
    kWaitingSetDeqIntrEp,
  };

  enum class Ep0RecoveryNext : uint8_t {
    kNone = 0,
    kSetIdle,
    kFinishAttach,
  };

  struct PortState {
    PortFsm fsm = PortFsm::kDisconnected;
    int speed = 0;
    int attempts = 0;
    int dev_idx = -1;
    int64_t debounce_start = 0;
    int64_t deadline = kNoDeadline;
  };

  struct DeviceSlot {
    bool in_use = false;
    bool announced = false;
    int port = 0;
    int speed = 0;
    uint8_t slot_id = 0;
    uint16_t ep0_mps = 8;
    DevFsm state = DevFsm::kFree;
    Ep0RecoveryNext after_ep0_recovery = Ep0RecoveryNext::kNone;

    uint8_t* out_ctx_va = nullptr;
    uint64_t out_ctx_phys = 0;
    uint8_t* in_ctx_va = nullptr;
    uint64_t in_ctx_phys = 0;
    uint8_t* ctrl_buf_va = nullptr;
    uint64_t ctrl_buf_phys = 0;
    Trb* ep0_ring_va = nullptr;
    uint64_t ep0_ring_phys = 0;
    Trb* intr_ring_va = nullptr;
    uint64_t intr_ring_phys = 0;
    uint8_t* report_buf_va = nullptr;
    uint64_t report_buf_phys = 0;

    ProducerRing<kRingTrbs> ep0_ring = {};
    ProducerRing<kRingTrbs> intr_ring = {};

    bool ctrl_in_flight = false;
    uint64_t ctrl_setup_phys = 0;
    uint64_t ctrl_data_phys = 0;
    uint64_t ctrl_status_phys = 0;
    uint16_t ctrl_req_len = 0;
    uint16_t ctrl_actual_len = 0;
    int64_t deadline = kNoDeadline;

    BootKeyboardMatch kbd = {};
    uint64_t report_trb_phys[kReportTrbs] = {};
    int next_rearm_idx = 0;
    int intr_errors = 0;
    HidKeyboardState hid_state = {};
  };

  struct CommandState {
    bool busy = false;
    bool is_disable_slot = false;
    bool abandoned = false;
    int dev_idx = -1;
    uint8_t disable_slot_id = 0;
    uint64_t trb_phys = 0;
    int64_t deadline = kNoDeadline;
  };

  XhciStatus RunPhase1();
  XhciStatus TakeOwnershipFromFirmware();
  XhciStatus WaitUntilReady();
  XhciStatus HaltIfRunning();
  XhciStatus ResetHalted();
  PollResult Poll(int64_t offset,     //
                  uint32_t mask,      //
                  uint32_t expected,  //
                  int64_t timeout_ms);
  uint32_t ReadOp(const int64_t reg) { return mmio_->Read32(caps_.Op(reg)); }
  void WriteOp(const int64_t reg, const uint32_t value) {
    mmio_->Write32(caps_.Op(reg), value);
  }
  uint32_t ReadPortSc(const int port) {
    return mmio_->Read32(caps_.PortReg(port, kPortSc));
  }
  void WritePortSc(const int port, const uint32_t value) {
    mmio_->Write32(caps_.PortReg(port, kPortSc), value);
  }
  LogLine StartLogLine() const;

  // Phase-2 helpers.
  XhciStatus RunStart(IrqMode mode);
  XhciStatus EnsureDmaAllocated();
  void FormatCoreDma();
  void ScanPort(int port, int64_t now);
  void HandlePortDisconnect(int port, int64_t now);
  void StartPortResetOrUsb3(int port, int64_t now);
  void RetryOrDisablePort(int port, int64_t now, const char* reason);
  void DisableUnsupportedDevice(DeviceSlot& dev, const char* reason);
  void BeginDeviceEnumeration(int port, int64_t now);
  void ReleaseDevice(int dev_idx);

  void DispatchEvent(const Trb& ev);
  void OnCommandCompletion(const Trb& ev, int64_t now);
  void OnTransferEvent(const Trb& ev, int64_t now);
  void OnEp0TransferEvent(DeviceSlot& dev, const Trb& ev, int64_t now);
  void OnControlTransferComplete(DeviceSlot& dev,  //
                                 bool ok,          //
                                 uint8_t cc,       //
                                 int64_t now);
  void OnInterruptTransferEvent(DeviceSlot& dev, const Trb& ev, int64_t now);

  void SubmitCommand(const Trb& trb,           //
                     int dev_idx,              //
                     bool is_disable_slot,     //
                     uint8_t disable_slot_id,  //
                     int64_t now);
  void PumpCommandRing(int64_t now);
  void SubmitControlTransfer(DeviceSlot& dev,          //
                             uint8_t bm_request_type,  //
                             uint8_t b_request,        //
                             uint16_t w_value,         //
                             uint16_t w_index,         //
                             uint16_t w_length,        //
                             int64_t now);
  void StartSetIdle(DeviceSlot& dev, int64_t now);
  void ArmInterruptTransfers(DeviceSlot& dev);
  void FinishKeyboardAttach(DeviceSlot& dev);
  void AdvanceStateMachines(int64_t now);
  void AbortCommandAndRecover(const char* reason);
  void Recover(const char* reason);
  void DetachAllKeyboards();

  Platform* platform_ = nullptr;
  MmioWindow* mmio_ = nullptr;
  KeyboardLocks* locks_ = &g_default_keyboard_locks;
  char label_[kMaxControllerLabelLength + 1] = {};
  XhciState state_ = XhciState::kUnbound;
  HandoffOutcome handoff_ = HandoffOutcome::kNotRun;
  IrqMode irq_mode_ = IrqMode::kMsix;
  XhciCaps caps_ = {};

  bool dma_ready_ = false;
  uint8_t* core_dma_va_ = nullptr;
  uint64_t core_dma_phys_ = 0;
  int64_t core_dma_bytes_ = 0;
  uint64_t* dcbaa_ = nullptr;
  uint64_t dcbaa_phys_ = 0;
  uint64_t sp_array_phys_ = 0;
  ProducerRing<kRingTrbs> cmd_ring_ = {};
  EventRing<kRingTrbs> event_ring_ = {};

  int ports_ = 0;
  int slots_ = 0;
  int attached_keyboards_ = 0;
  int recoveries_ = 0;
  bool in_recovery_ = false;
  bool event_ring_full_rescan_ = false;
  uint64_t pending_disable_slots_ = 0;
  CommandState cmd_state_ = {};
  PortState port_states_[kMaxPorts] = {};
  DeviceSlot devices_[kMaxDevices] = {};
};

static_assert(std::is_trivially_destructible_v<XhciController>);

inline void XhciController::Init(Platform& platform,       //
                                 MmioWindow& mmio,         //
                                 const char* const label,  //
                                 KeyboardLocks* const locks) {
  platform_ = &platform;
  mmio_ = &mmio;
  locks_ = locks != nullptr ? locks : &g_default_keyboard_locks;
  int length = 0;
  if (label != nullptr) {
    while (length < kMaxControllerLabelLength && label[length] != '\0') {
      label_[length] = label[length];
      ++length;
    }
  }
  label_[length] = '\0';
  state_ = XhciState::kBound;
  handoff_ = HandoffOutcome::kNotRun;
  irq_mode_ = IrqMode::kMsix;
  caps_ = XhciCaps{};
  dma_ready_ = false;
  core_dma_va_ = nullptr;
  core_dma_phys_ = 0;
  core_dma_bytes_ = 0;
  dcbaa_ = nullptr;
  dcbaa_phys_ = 0;
  sp_array_phys_ = 0;
  ports_ = 0;
  slots_ = 0;
  attached_keyboards_ = 0;
  recoveries_ = 0;
  in_recovery_ = false;
  event_ring_full_rescan_ = false;
  pending_disable_slots_ = 0;
  cmd_state_ = CommandState{};
  for (int i = 0; i < kMaxPorts; ++i) {
    port_states_[i] = PortState{};
  }
  for (int i = 0; i < kMaxDevices; ++i) {
    devices_[i] = DeviceSlot{};
  }
}

inline XhciStatus XhciController::TakeOwnershipAndReset() {
  if (platform_ == nullptr || mmio_ == nullptr) {
    return XhciStatus::kNotBound;
  }
  state_ = XhciState::kBound;
  handoff_ = HandoffOutcome::kNotRun;
  const XhciStatus status = RunPhase1();
  if (status != XhciStatus::kOk) {
    state_ = XhciState::kError;
    StartLogLine().Str("error: ").Str(XhciStatusName(status)).Emit(*platform_);
    return status;
  }
  state_ = XhciState::kReset;
  return XhciStatus::kOk;
}

inline XhciStatus XhciController::RunPhase1() {
  if (const XhciStatus status = DecodeCaps(*mmio_, &caps_);
      status != XhciStatus::kOk) {
    return status;
  }
  if (const XhciStatus status = ParseExtendedCaps(*mmio_, &caps_);
      status != XhciStatus::kOk) {
    return status;
  }
  if (ReadOp(kOpUsbSts) == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  if (const XhciStatus status = TakeOwnershipFromFirmware();
      status != XhciStatus::kOk) {
    return status;
  }
  if (const XhciStatus status = WaitUntilReady(); status != XhciStatus::kOk) {
    return status;
  }
  if (const XhciStatus status = HaltIfRunning(); status != XhciStatus::kOk) {
    return status;
  }
  return ResetHalted();
}

inline XhciStatus XhciController::TakeOwnershipFromFirmware() {
  const int64_t legsup = caps_.legacy_offset;
  if (legsup == 0) {
    handoff_ = HandoffOutcome::kNoLegacyCapability;
    return XhciStatus::kOk;
  }
  const uint32_t value = mmio_->Read32(legsup + kLegSup);
  if (value == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  if ((value & kLegSupOsOwned) == 0) {
    mmio_->Write32(legsup + kLegSup, value | kLegSupOsOwned);
  }
  if ((value & kLegSupBiosOwned) == 0) {
    handoff_ = HandoffOutcome::kNotBiosOwned;
  } else {
    const int64_t start = platform_->NowTicks();
    const PollResult released =
        Poll(legsup + kLegSup, kLegSupBiosOwned, 0, kHandoffTimeoutMs);
    if (released == PollResult::kNotDecoding) {
      return XhciStatus::kNotDecoding;
    }
    if (released == PollResult::kMatched) {
      handoff_ = HandoffOutcome::kReleased;
      StartLogLine()
          .Str("firmware released ownership after ")
          .Dec(TicksToMs(*platform_, platform_->NowTicks() - start))
          .Str(" ms")
          .Emit(*platform_);
    } else {
      const uint32_t current = mmio_->Read32(legsup + kLegSup);
      if (current == kMmioAllOnes) {
        return XhciStatus::kNotDecoding;
      }
      mmio_->Write32(legsup + kLegSup,
                     (current | kLegSupOsOwned) & ~kLegSupBiosOwned);
      handoff_ = HandoffOutcome::kForced;
      StartLogLine()
          .Str("firmware handoff timeout after ")
          .Dec(kHandoffTimeoutMs)
          .Str(" ms, forcing OS ownership")
          .Emit(*platform_);
    }
  }
  const uint32_t control = mmio_->Read32(legsup + kLegCtlSts);
  if (control == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  mmio_->Write32(legsup + kLegCtlSts, LegacyControlDisablingSmis(control));
  return XhciStatus::kOk;
}

inline XhciStatus XhciController::WaitUntilReady() {
  const PollResult ready =
      Poll(caps_.Op(kOpUsbSts), kUsbStsCnr, 0, kControllerReadyTimeoutMs);
  if (ready == PollResult::kNotDecoding) {
    return XhciStatus::kNotDecoding;
  }
  if (ready == PollResult::kTimedOut) {
    StartLogLine()
        .Str("controller not ready after ")
        .Dec(kControllerReadyTimeoutMs)
        .Str(" ms, continuing")
        .Emit(*platform_);
  }
  return XhciStatus::kOk;
}

inline XhciStatus XhciController::HaltIfRunning() {
  const uint32_t command = ReadOp(kOpUsbCmd);
  if (command == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  if ((command & kUsbCmdHaltMask) != 0) {
    WriteOp(kOpUsbCmd, command & ~kUsbCmdHaltMask);
  }
  const uint32_t status = ReadOp(kOpUsbSts);
  if (status == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  if ((status & kUsbStsHch) != 0) {
    return XhciStatus::kOk;
  }
  const PollResult halted =
      Poll(caps_.Op(kOpUsbSts), kUsbStsHch, kUsbStsHch, kHaltTimeoutMs);
  if (halted == PollResult::kNotDecoding) {
    return XhciStatus::kNotDecoding;
  }
  if (halted == PollResult::kTimedOut) {
    return XhciStatus::kHaltTimeout;
  }
  StartLogLine().Str("controller was running, halted").Emit(*platform_);
  return XhciStatus::kOk;
}

inline XhciStatus XhciController::ResetHalted() {
  const uint32_t command = ReadOp(kOpUsbCmd);
  if (command == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  WriteOp(kOpUsbCmd, (command & ~(kUsbCmdHaltMask | kUsbCmdLhcrst | kUsbCmdCss |
                                  kUsbCmdCrs)) |
                         kUsbCmdHcrst);
  DelayMs(*platform_, kPostResetDelayMs);
  const PollResult reset =
      Poll(caps_.Op(kOpUsbCmd), kUsbCmdHcrst, 0, kResetTimeoutMs);
  if (reset == PollResult::kNotDecoding) {
    return XhciStatus::kNotDecoding;
  }
  if (reset == PollResult::kTimedOut) {
    return XhciStatus::kResetTimeout;
  }
  const PollResult ready =
      Poll(caps_.Op(kOpUsbSts), kUsbStsCnr, 0, kResetReadyTimeoutMs);
  if (ready == PollResult::kNotDecoding) {
    return XhciStatus::kNotDecoding;
  }
  if (ready == PollResult::kTimedOut) {
    return XhciStatus::kNotReadyAfterReset;
  }
  const uint32_t status = ReadOp(kOpUsbSts);
  if (status == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  if ((status & kUsbStsHch) == 0) {
    return XhciStatus::kRunningAfterReset;
  }
  return XhciStatus::kOk;
}

inline XhciController::PollResult XhciController::Poll(
    const int64_t offset,     //
    const uint32_t mask,      //
    const uint32_t expected,  //
    const int64_t timeout_ms) {
  PollResult result = PollResult::kTimedOut;
  WaitUntil(*platform_, timeout_ms, [&]() {
    const uint32_t value = mmio_->Read32(offset);
    if (value == kMmioAllOnes) {
      result = PollResult::kNotDecoding;
      return true;
    }
    if ((value & mask) == expected) {
      result = PollResult::kMatched;
      return true;
    }
    return false;
  });
  return result;
}

inline LogLine XhciController::StartLogLine() const {
  LogLine line;
  line.Str(kLogPrefix).Str(label_).Str(": ");
  return line;
}

namespace ctrl_internal {

constexpr int64_t kPageBytes = 4096;
constexpr int kFixedCorePages = 3 + 5 * kMaxDevices;  // 43 pages

inline bool ValidDmaBuffer(const DmaBuffer& buf,  //
                           const int64_t bytes,   //
                           const bool below_4gib) {
  if (buf.va == nullptr || bytes <= 0 || (buf.phys & 0xFFFull) != 0) {
    return false;
  }
  const uint64_t ubytes = static_cast<uint64_t>(bytes);
  if (buf.phys + ubytes < buf.phys) {
    return false;
  }
  if (below_4gib && buf.phys > (0x1'0000'0000ULL - ubytes)) {
    return false;
  }
  return true;
}

inline void ZeroBytesVolatile(uint8_t* const base, const int64_t bytes) {
  for (int64_t i = 0; i < bytes; ++i) {
    *reinterpret_cast<volatile uint8_t*>(base + i) = 0;
  }
}

}  // namespace ctrl_internal

inline XhciStatus XhciController::EnsureDmaAllocated() {
  if (dma_ready_) {
    return XhciStatus::kOk;
  }
  const bool below_4gib = !caps_.ac64;
  const int sp_count = caps_.max_scratchpads;
  const int sp_array_pages = sp_count > 0 ? (sp_count * 8 + 4095) / 4096 : 0;
  const int total_core_pages = ctrl_internal::kFixedCorePages + sp_array_pages;
  const int64_t core_bytes =
      static_cast<int64_t>(total_core_pages) * ctrl_internal::kPageBytes;

  const DmaBuffer core = platform_->DmaAlloc(core_bytes, below_4gib);
  if (!ctrl_internal::ValidDmaBuffer(core, core_bytes, below_4gib)) {
    return XhciStatus::kNoMemory;
  }
  ctrl_internal::ZeroBytesVolatile(static_cast<uint8_t*>(core.va), core_bytes);

  uint64_t sp_array_phys = 0;
  if (sp_count > 0) {
    uint64_t* const sp_array = reinterpret_cast<uint64_t*>(
        static_cast<uint8_t*>(core.va) +
        ctrl_internal::kFixedCorePages * ctrl_internal::kPageBytes);
    sp_array_phys =
        core.phys + static_cast<uint64_t>(ctrl_internal::kFixedCorePages) *
                        ctrl_internal::kPageBytes;
    int allocated = 0;
    while (allocated < sp_count) {
      const int batch =
          (sp_count - allocated) > 256 ? 256 : (sp_count - allocated);
      const int64_t batch_bytes =
          static_cast<int64_t>(batch) * ctrl_internal::kPageBytes;
      const DmaBuffer sp_buf = platform_->DmaAlloc(batch_bytes, below_4gib);
      if (!ctrl_internal::ValidDmaBuffer(sp_buf, batch_bytes, below_4gib)) {
        return XhciStatus::kNoMemory;
      }
      ctrl_internal::ZeroBytesVolatile(static_cast<uint8_t*>(sp_buf.va),
                                       batch_bytes);
      for (int j = 0; j < batch; ++j) {
        sp_array[allocated + j] =
            sp_buf.phys + static_cast<uint64_t>(j) * ctrl_internal::kPageBytes;
      }
      allocated += batch;
    }
  }

  core_dma_va_ = static_cast<uint8_t*>(core.va);
  core_dma_phys_ = core.phys;
  core_dma_bytes_ = static_cast<int64_t>(ctrl_internal::kFixedCorePages) *
                    ctrl_internal::kPageBytes;
  dcbaa_ = reinterpret_cast<uint64_t*>(core_dma_va_);
  dcbaa_phys_ = core_dma_phys_;
  sp_array_phys_ = sp_array_phys;
  dma_ready_ = true;
  return XhciStatus::kOk;
}

inline void XhciController::FormatCoreDma() {
  ctrl_internal::ZeroBytesVolatile(core_dma_va_, core_dma_bytes_);
  dcbaa_[0] = sp_array_phys_;

  ErstEntry* const erst = reinterpret_cast<ErstEntry*>(core_dma_va_ + 0x800);
  const uint64_t erst_phys = core_dma_phys_ + 0x800;

  Trb* const cmd_va =
      reinterpret_cast<Trb*>(core_dma_va_ + 1 * ctrl_internal::kPageBytes);
  const uint64_t cmd_phys = core_dma_phys_ + 1 * ctrl_internal::kPageBytes;
  cmd_ring_.Init(cmd_va, cmd_phys);

  Trb* const evt_va =
      reinterpret_cast<Trb*>(core_dma_va_ + 2 * ctrl_internal::kPageBytes);
  const uint64_t evt_phys = core_dma_phys_ + 2 * ctrl_internal::kPageBytes;
  event_ring_.Init(evt_va, evt_phys, erst, erst_phys);

  for (int i = 0; i < kMaxDevices; ++i) {
    DeviceSlot& dev = devices_[i];
    dev = DeviceSlot{};
    const int base_page = 3 + 5 * i;
    uint8_t* const p0_va = core_dma_va_ + base_page * ctrl_internal::kPageBytes;
    const uint64_t p0_phys = core_dma_phys_ + static_cast<uint64_t>(base_page) *
                                                  ctrl_internal::kPageBytes;
    dev.out_ctx_va = p0_va;
    dev.out_ctx_phys = p0_phys;
    dev.ctrl_buf_va = p0_va + 0x800;
    dev.ctrl_buf_phys = p0_phys + 0x800;
    dev.in_ctx_va = core_dma_va_ + (base_page + 1) * ctrl_internal::kPageBytes;
    dev.in_ctx_phys = core_dma_phys_ + static_cast<uint64_t>(base_page + 1) *
                                           ctrl_internal::kPageBytes;
    dev.ep0_ring_va = reinterpret_cast<Trb*>(
        core_dma_va_ + (base_page + 2) * ctrl_internal::kPageBytes);
    dev.ep0_ring_phys = core_dma_phys_ + static_cast<uint64_t>(base_page + 2) *
                                             ctrl_internal::kPageBytes;
    dev.intr_ring_va = reinterpret_cast<Trb*>(
        core_dma_va_ + (base_page + 3) * ctrl_internal::kPageBytes);
    dev.intr_ring_phys = core_dma_phys_ + static_cast<uint64_t>(base_page + 3) *
                                              ctrl_internal::kPageBytes;
    dev.report_buf_va =
        core_dma_va_ + (base_page + 4) * ctrl_internal::kPageBytes;
    dev.report_buf_phys =
        core_dma_phys_ +
        static_cast<uint64_t>(base_page + 4) * ctrl_internal::kPageBytes;
  }
  DmaBarrier();
}

inline XhciStatus XhciController::Start(const IrqMode mode) {
  if (platform_ == nullptr || mmio_ == nullptr) {
    return XhciStatus::kNotBound;
  }
  if (state_ != XhciState::kReset) {
    state_ = XhciState::kError;
    StartLogLine()
        .Str("error: ")
        .Str(XhciStatusName(XhciStatus::kBadState))
        .Emit(*platform_);
    return XhciStatus::kBadState;
  }
  const XhciStatus status = RunStart(mode);
  if (status != XhciStatus::kOk) {
    state_ = XhciState::kError;
    StartLogLine().Str("error: ").Str(XhciStatusName(status)).Emit(*platform_);
    return status;
  }
  return XhciStatus::kOk;
}

inline XhciStatus XhciController::RunStart(const IrqMode mode) {
  const uint32_t page_size = ReadOp(kOpPageSize);
  if (page_size == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  if ((page_size & kPageSize4K) == 0) {
    return XhciStatus::kUnsupportedPageSize;
  }
  const uint32_t sts = ReadOp(kOpUsbSts);
  if (sts == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  if ((sts & (kUsbStsCnr | kUsbStsHch)) != kUsbStsHch) {
    return XhciStatus::kBadState;
  }

  if (const XhciStatus dma_status = EnsureDmaAllocated();
      dma_status != XhciStatus::kOk) {
    return dma_status;
  }
  FormatCoreDma();

  ports_ = caps_.max_ports < kMaxPorts ? caps_.max_ports : kMaxPorts;
  slots_ = caps_.max_slots < 64 ? caps_.max_slots : 64;
  attached_keyboards_ = 0;
  event_ring_full_rescan_ = false;
  pending_disable_slots_ = 0;
  cmd_state_ = CommandState{};
  for (int i = 0; i < kMaxPorts; ++i) {
    port_states_[i] = PortState{};
  }
  if (!in_recovery_) {
    recoveries_ = 0;
  }

  const uint32_t cfg = ReadOp(kOpConfig);
  if (cfg == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  WriteOp(kOpConfig,
          (cfg & ~(kConfigMaxSlotsEnMask | kConfigU3e | kConfigCie)) |
              static_cast<uint32_t>(slots_));

  Write64LoHi(*mmio_, caps_.Op(kOpDcbaap), dcbaa_phys_);
  Write64LoHi(*mmio_, caps_.Op(kOpCrcr),
              cmd_ring_.phys() | static_cast<uint64_t>(kCrcrRcs));

  mmio_->Write32(caps_.InterrupterReg(0, kIrImod), kImodDefault);
  const uint32_t erstsz = mmio_->Read32(caps_.InterrupterReg(0, kIrErstSz));
  if (erstsz == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  mmio_->Write32(caps_.InterrupterReg(0, kIrErstSz),
                 (erstsz & ~kErstSzMask) | 1u);
  Write64LoHi(*mmio_, caps_.InterrupterReg(0, kIrErdp),
              event_ring_.dequeue_phys());
  Write64LoHi(*mmio_, caps_.InterrupterReg(0, kIrErstBa),
              event_ring_.erst_phys());

  const uint32_t iman = mmio_->Read32(caps_.InterrupterReg(0, kIrIman));
  if (iman == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  mmio_->Write32(caps_.InterrupterReg(0, kIrIman),
                 (iman & ~kImanIp) | kImanIe | kImanIp);

  WriteOp(kOpUsbSts, kUsbStsEint | kUsbStsPcd | kUsbStsHse);

  const uint32_t cmd = ReadOp(kOpUsbCmd);
  if (cmd == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  WriteOp(kOpUsbCmd,
          (cmd & ~(kUsbCmdLhcrst | kUsbCmdCss | kUsbCmdCrs | kUsbCmdHcrst)) |
              kUsbCmdInte | kUsbCmdHsee | kUsbCmdRs);

  const PollResult running =
      Poll(caps_.Op(kOpUsbSts), kUsbStsHch, 0, kStartTimeoutMs);
  if (running == PollResult::kNotDecoding) {
    return XhciStatus::kNotDecoding;
  }
  if (running == PollResult::kTimedOut) {
    return XhciStatus::kStartTimeout;
  }

  if (caps_.ppc) {
    for (int port = 1; port <= ports_; ++port) {
      const uint32_t ps = ReadPortSc(port);
      if (ps == kMmioAllOnes) {
        return XhciStatus::kNotDecoding;
      }
      if ((ps & kPortScPp) == 0) {
        WritePortSc(port, (ps & kPortScNeutralMask) | kPortScPp);
      }
    }
    DelayMs(*platform_, kPortPowerSettleMs);
  }

  irq_mode_ = mode;
  state_ = XhciState::kRunning;
  LogLine()
      .Str(kLogPrefix)
      .Str("controller ")
      .Str(label_)
      .Str(" ready (")
      .Str(IrqModeName(mode))
      .Str(")")
      .Emit(*platform_);

  const int64_t now = platform_->NowTicks();
  for (int port = 1; port <= ports_; ++port) {
    ScanPort(port, now);
  }
  AdvanceStateMachines(now);
  return XhciStatus::kOk;
}

inline void XhciController::HandleInterrupt() {
  if (state_ != XhciState::kRunning || platform_ == nullptr ||
      mmio_ == nullptr) {
    return;
  }
  const uint32_t sts = ReadOp(kOpUsbSts);
  if (sts == kMmioAllOnes) {
    DetachAllKeyboards();
    StartLogLine()
        .Str("error: ")
        .Str(XhciStatusName(XhciStatus::kNotDecoding))
        .Emit(*platform_);
    state_ = XhciState::kError;
    return;
  }
  if ((sts & (kUsbStsHse | kUsbStsHce)) != 0) {
    if ((sts & kUsbStsHse) != 0) {
      WriteOp(kOpUsbSts, kUsbStsHse);
    }
    Recover((sts & kUsbStsHce) != 0 ? "host controller error"
                                    : "host system error");
    return;
  }
  if ((sts & kUsbStsEint) != 0) {
    WriteOp(kOpUsbSts, kUsbStsEint);
  }

  const uint32_t iman = mmio_->Read32(caps_.InterrupterReg(0, kIrIman));
  if (iman == kMmioAllOnes) {
    DetachAllKeyboards();
    StartLogLine()
        .Str("error: ")
        .Str(XhciStatusName(XhciStatus::kNotDecoding))
        .Emit(*platform_);
    state_ = XhciState::kError;
    return;
  }
  mmio_->Write32(caps_.InterrupterReg(0, kIrIman),
                 (iman & ~kImanIp) | kImanIe | kImanIp);

  Trb ev;
  int drained = 0;
  while (drained < kRingTrbs && event_ring_.Pop(&ev)) {
    ++drained;
    DispatchEvent(ev);
    if (state_ != XhciState::kRunning) {
      return;
    }
  }

  Write64LoHi(*mmio_, caps_.InterrupterReg(0, kIrErdp),
              event_ring_.dequeue_phys() | static_cast<uint64_t>(kErdpEhb));

  AdvanceStateMachines(platform_->NowTicks());
}

inline int64_t XhciController::NextDeadline() const {
  if (state_ != XhciState::kRunning) {
    return kNoDeadline;
  }
  int64_t earliest = kNoDeadline;
  if (cmd_state_.busy && cmd_state_.deadline < earliest) {
    earliest = cmd_state_.deadline;
  }
  for (int p = 0; p < ports_; ++p) {
    if (port_states_[p].deadline < earliest) {
      earliest = port_states_[p].deadline;
    }
  }
  for (int i = 0; i < kMaxDevices; ++i) {
    if (devices_[i].in_use && devices_[i].deadline < earliest) {
      earliest = devices_[i].deadline;
    }
  }
  return earliest;
}

inline void XhciController::DispatchEvent(const Trb& ev) {
  const int64_t now = platform_->NowTicks();
  switch (TrbType(ev)) {
    case kTrbPortStatusChange: {
      const int port = TrbPortId(ev);
      if (port >= 1 && port <= ports_) {
        ScanPort(port, now);
      } else {
        for (int p = 1; p <= ports_; ++p) {
          ScanPort(p, now);
        }
      }
      break;
    }
    case kTrbCommandCompletion:
      OnCommandCompletion(ev, now);
      break;
    case kTrbTransferEvent:
      OnTransferEvent(ev, now);
      break;
    case kTrbHostControllerEvent:
      if (TrbCompletionCode(ev) == kCcEventRingFull) {
        event_ring_full_rescan_ = true;
      }
      break;
    default:
      break;
  }
}

inline void XhciController::ScanPort(const int port, const int64_t now) {
  if (port < 1 || port > ports_) {
    return;
  }
  const uint32_t ps = ReadPortSc(port);
  if (ps == kMmioAllOnes) {
    return;
  }
  const uint32_t changes = ps & kPortScChangeMask;
  if (changes != 0) {
    WritePortSc(port, (ps & kPortScNeutralMask) | changes);
  }
  PortState& p = port_states_[port - 1];
  const bool connected = (ps & kPortScCcs) != 0;
  const bool csc = (changes & kPortScCsc) != 0;
  const bool prc = (changes & (kPortScPrc | kPortScWrc)) != 0;

  if (!connected) {
    if (p.fsm != PortFsm::kDisconnected || p.dev_idx >= 0) {
      HandlePortDisconnect(port, now);
    }
    return;
  }

  if (csc && p.fsm != PortFsm::kDisconnected && p.fsm != PortFsm::kDebouncing) {
    HandlePortDisconnect(port, now);
  }

  if (p.fsm == PortFsm::kDisconnected) {
    p.fsm = PortFsm::kDebouncing;
    p.debounce_start = now;
    p.deadline = now + TicksForMs(*platform_, kPortDebounceMs);
    p.attempts = 0;
    return;
  }

  if (p.fsm == PortFsm::kDebouncing && csc) {
    if (now - p.debounce_start >= TicksForMs(*platform_, kPortDebounceMaxMs)) {
      StartLogLine()
          .Str("port ")
          .Dec(port)
          .Str(": debounce timeout")
          .Emit(*platform_);
      p.fsm = PortFsm::kDisabled;
      p.deadline = kNoDeadline;
      return;
    }
    p.deadline = now + TicksForMs(*platform_, kPortDebounceMs);
    return;
  }

  if (p.fsm == PortFsm::kResetting &&
      (prc || ((ps & kPortScPr) == 0 && (ps & kPortScPed) != 0))) {
    p.speed = PortSpeedOf(ps);
    if ((ps & kPortScPed) == 0 || p.speed == 0) {
      RetryOrDisablePort(port, now, "port reset did not enable port");
    } else {
      p.fsm = PortFsm::kResetRecovery;
      p.deadline = now + TicksForMs(*platform_, kPortResetRecoveryMs);
    }
  }
}

inline void XhciController::HandlePortDisconnect(const int port,
                                                 const int64_t now) {
  (void)now;
  PortState& p = port_states_[port - 1];
  if (p.dev_idx >= 0 && p.dev_idx < kMaxDevices) {
    DeviceSlot& dev = devices_[p.dev_idx];
    if (dev.in_use) {
      if (dev.announced) {
        dev.announced = false;
        if (attached_keyboards_ > 0) {
          --attached_keyboards_;
        }
        LogLine()
            .Str(kLogPrefix)
            .Str("keyboard detached: ")
            .Str(label_)
            .Str(" port ")
            .Dec(port)
            .Emit(*platform_);
      }
      if (cmd_state_.busy && cmd_state_.dev_idx == p.dev_idx) {
        cmd_state_.abandoned = true;
      }
      if (dev.slot_id != 0 && dev.slot_id < 64) {
        pending_disable_slots_ |= (1ULL << dev.slot_id);
      }
      ReleaseDevice(p.dev_idx);
    }
  }
  p.fsm = PortFsm::kDisconnected;
  p.speed = 0;
  p.attempts = 0;
  p.dev_idx = -1;
  p.debounce_start = 0;
  p.deadline = kNoDeadline;
}

inline void XhciController::ReleaseDevice(const int dev_idx) {
  if (dev_idx < 0 || dev_idx >= kMaxDevices) {
    return;
  }
  DeviceSlot& dev = devices_[dev_idx];
  dev.in_use = false;
  dev.announced = false;
  dev.port = 0;
  dev.speed = 0;
  dev.slot_id = 0;
  dev.state = DevFsm::kFree;
  dev.ctrl_in_flight = false;
  dev.deadline = kNoDeadline;
}

inline void XhciController::StartPortResetOrUsb3(const int port,
                                                 const int64_t now) {
  const uint32_t ps = ReadPortSc(port);
  if (ps == kMmioAllOnes || (ps & kPortScCcs) == 0) {
    HandlePortDisconnect(port, now);
    return;
  }
  if ((ps & kPortScChangeMask) != 0) {
    WritePortSc(port, (ps & kPortScNeutralMask) | (ps & kPortScChangeMask));
  }
  PortState& p = port_states_[port - 1];
  const int major = caps_.PortProtocolMajor(port);
  const int speed = PortSpeedOf(ps);
  if ((major == 3 || speed >= kPortSpeedSuper) && (ps & kPortScPed) != 0 &&
      speed != 0) {
    p.speed = speed;
    p.fsm = PortFsm::kEnumerating;
    p.deadline = kNoDeadline;
    BeginDeviceEnumeration(port, now);
    return;
  }
  WritePortSc(port, (ps & kPortScNeutralMask) | kPortScPr);
  p.fsm = PortFsm::kResetting;
  p.deadline = now + TicksForMs(*platform_, kPortResetTimeoutMs);
}

inline void XhciController::RetryOrDisablePort(const int port,     //
                                               const int64_t now,  //
                                               const char* const reason) {
  PortState& p = port_states_[port - 1];
  if (p.dev_idx >= 0 && p.dev_idx < kMaxDevices) {
    DeviceSlot& dev = devices_[p.dev_idx];
    if (dev.in_use) {
      if (dev.announced) {
        dev.announced = false;
        if (attached_keyboards_ > 0) {
          --attached_keyboards_;
        }
        LogLine()
            .Str(kLogPrefix)
            .Str("keyboard detached: ")
            .Str(label_)
            .Str(" port ")
            .Dec(port)
            .Emit(*platform_);
      }
      if (cmd_state_.busy && cmd_state_.dev_idx == p.dev_idx) {
        cmd_state_.abandoned = true;
      }
      if (dev.slot_id != 0 && dev.slot_id < 64) {
        pending_disable_slots_ |= (1ULL << dev.slot_id);
      }
      ReleaseDevice(p.dev_idx);
    }
    p.dev_idx = -1;
  }

  ++p.attempts;
  StartLogLine()
      .Str("port ")
      .Dec(port)
      .Str(": ")
      .Str(reason)
      .Str(" (attempt ")
      .Dec(p.attempts)
      .Str(")")
      .Emit(*platform_);

  const uint32_t ps = ReadPortSc(port);
  if (ps == kMmioAllOnes || (ps & kPortScCcs) == 0) {
    HandlePortDisconnect(port, now);
    return;
  }
  if (p.attempts < kMaxEnumerationAttempts) {
    p.fsm = PortFsm::kDebouncing;
    p.debounce_start = now;
    p.deadline = now + TicksForMs(*platform_, kPortDebounceMs);
  } else {
    p.fsm = PortFsm::kDisabled;
    p.deadline = kNoDeadline;
  }
}

inline void XhciController::DisableUnsupportedDevice(DeviceSlot& dev,
                                                     const char* const reason) {
  const int port = dev.port;
  StartLogLine().Str("port ").Dec(port).Str(": ").Str(reason).Emit(*platform_);
  if (dev.slot_id != 0 && dev.slot_id < 64) {
    pending_disable_slots_ |= (1ULL << dev.slot_id);
  }
  if (port >= 1 && port <= ports_) {
    PortState& p = port_states_[port - 1];
    if (p.dev_idx >= 0) {
      ReleaseDevice(p.dev_idx);
      p.dev_idx = -1;
    }
    p.fsm = PortFsm::kDisabled;
    p.deadline = kNoDeadline;
  }
}

inline void XhciController::BeginDeviceEnumeration(const int port,
                                                   const int64_t now) {
  int free_idx = -1;
  for (int i = 0; i < kMaxDevices; ++i) {
    if (!devices_[i].in_use) {
      free_idx = i;
      break;
    }
  }
  PortState& p = port_states_[port - 1];
  if (free_idx < 0) {
    StartLogLine()
        .Str("port ")
        .Dec(port)
        .Str(": no free device slots")
        .Emit(*platform_);
    p.fsm = PortFsm::kDisabled;
    p.deadline = kNoDeadline;
    return;
  }

  DeviceSlot& dev = devices_[free_idx];
  dev.in_use = true;
  dev.announced = false;
  dev.port = port;
  dev.speed = p.speed;
  dev.slot_id = 0;
  dev.ep0_mps = InitialEp0MaxPacketSize(p.speed);
  dev.state = DevFsm::kWantEnableSlot;
  dev.after_ep0_recovery = Ep0RecoveryNext::kNone;
  dev.ep0_ring.Init(dev.ep0_ring_va, dev.ep0_ring_phys);
  dev.intr_ring.Init(dev.intr_ring_va, dev.intr_ring_phys);
  ZeroContextBuffer(dev.out_ctx_va, 32 * caps_.context_size);
  ZeroContextBuffer(dev.in_ctx_va, 33 * caps_.context_size);
  dev.ctrl_in_flight = false;
  dev.deadline = kNoDeadline;
  dev.kbd = BootKeyboardMatch{};
  dev.next_rearm_idx = 0;
  dev.intr_errors = 0;
  dev.hid_state = HidKeyboardState{};

  p.dev_idx = free_idx;
  PumpCommandRing(now);
}

inline void XhciController::SubmitCommand(const Trb& trb,                 //
                                          const int dev_idx,              //
                                          const bool is_disable_slot,     //
                                          const uint8_t disable_slot_id,  //
                                          const int64_t now) {
  cmd_state_.busy = true;
  cmd_state_.is_disable_slot = is_disable_slot;
  cmd_state_.abandoned = false;
  cmd_state_.dev_idx = dev_idx;
  cmd_state_.disable_slot_id = disable_slot_id;
  cmd_state_.trb_phys = cmd_ring_.Enqueue(trb);
  cmd_state_.deadline = now + TicksForMs(*platform_, kCommandTimeoutMs);
  mmio_->Write32(caps_.Doorbell(0), 0u);
}

inline void XhciController::PumpCommandRing(const int64_t now) {
  if (cmd_state_.busy || state_ != XhciState::kRunning) {
    return;
  }
  if (pending_disable_slots_ != 0) {
    for (uint8_t s = 1; s < 64; ++s) {
      if ((pending_disable_slots_ & (1ULL << s)) != 0) {
        pending_disable_slots_ &= ~(1ULL << s);
        dcbaa_[s] = 0;
        DmaBarrier();
        SubmitCommand(MakeDisableSlotTrb(s), -1, true, s, now);
        return;
      }
    }
  }

  for (int i = 0; i < kMaxDevices; ++i) {
    DeviceSlot& dev = devices_[i];
    if (!dev.in_use) {
      continue;
    }
    switch (dev.state) {
      case DevFsm::kWantEnableSlot: {
        const uint8_t slot_type =
            static_cast<uint8_t>(caps_.PortSlotType(dev.port));
        dev.state = DevFsm::kWaitingEnableSlot;
        SubmitCommand(MakeEnableSlotTrb(slot_type), i, false, 0, now);
        return;
      }
      case DevFsm::kWantAddressDevice: {
        FormatAddressDeviceInputContext(
            dev.in_ctx_va, caps_.context_size, dev.port, dev.speed,
            dev.ep0_ring.phys(), dev.ep0_ring.cycle());
        dcbaa_[dev.slot_id] = dev.out_ctx_phys;
        DmaBarrier();
        dev.state = DevFsm::kWaitingAddressDevice;
        SubmitCommand(MakeAddressDeviceTrb(dev.in_ctx_phys, dev.slot_id, false),
                      i, false, 0, now);
        return;
      }
      case DevFsm::kWantEvaluateEp0: {
        FormatEvaluateEp0MpsInputContext(dev.in_ctx_va, caps_.context_size,
                                         dev.out_ctx_va, dev.ep0_mps);
        DmaBarrier();
        dev.state = DevFsm::kWaitingEvaluateEp0;
        SubmitCommand(MakeEvaluateContextTrb(dev.in_ctx_phys, dev.slot_id), i,
                      false, 0, now);
        return;
      }
      case DevFsm::kWantConfigureEndpoint: {
        FormatConfigureInterruptEndpointInputContext(
            dev.in_ctx_va, caps_.context_size, dev.out_ctx_va, dev.kbd.ep_dci,
            dev.speed, dev.kbd.max_packet_size, dev.kbd.max_burst,
            dev.kbd.b_interval, dev.intr_ring.phys(), dev.intr_ring.cycle());
        DmaBarrier();
        dev.state = DevFsm::kWaitingConfigureEndpoint;
        SubmitCommand(
            MakeConfigureEndpointTrb(dev.in_ctx_phys, dev.slot_id, false), i,
            false, 0, now);
        return;
      }
      case DevFsm::kWantResetEp0: {
        dev.state = DevFsm::kWaitingResetEp0;
        SubmitCommand(MakeResetEndpointTrb(dev.slot_id, 1, false), i, false, 0,
                      now);
        return;
      }
      case DevFsm::kWantSetDeqEp0: {
        const uint64_t deq = dev.ep0_ring.enqueue_phys() | dev.ep0_ring.cycle();
        dev.state = DevFsm::kWaitingSetDeqEp0;
        SubmitCommand(MakeSetTrDequeueTrb(deq, dev.slot_id, 1), i, false, 0,
                      now);
        return;
      }
      case DevFsm::kWantResetIntrEp: {
        dev.state = DevFsm::kWaitingResetIntrEp;
        SubmitCommand(MakeResetEndpointTrb(dev.slot_id, dev.kbd.ep_dci, false),
                      i, false, 0, now);
        return;
      }
      case DevFsm::kWantSetDeqIntrEp: {
        dev.intr_ring.Init(dev.intr_ring_va, dev.intr_ring_phys);
        const uint64_t deq =
            dev.intr_ring.enqueue_phys() | dev.intr_ring.cycle();
        dev.state = DevFsm::kWaitingSetDeqIntrEp;
        SubmitCommand(MakeSetTrDequeueTrb(deq, dev.slot_id, dev.kbd.ep_dci), i,
                      false, 0, now);
        return;
      }
      default:
        break;
    }
  }
}

inline void XhciController::OnCommandCompletion(const Trb& ev,
                                                const int64_t now) {
  if (!cmd_state_.busy || TrbPointer(ev) != cmd_state_.trb_phys) {
    return;
  }
  const CommandState completed = cmd_state_;
  cmd_state_ = CommandState{};

  const uint8_t cc = TrbCompletionCode(ev);
  const uint8_t slot_id = TrbSlotId(ev);

  if (completed.is_disable_slot) {
    PumpCommandRing(now);
    return;
  }

  if (completed.abandoned || completed.dev_idx < 0 ||
      completed.dev_idx >= kMaxDevices) {
    if (cc == kCcSuccess && slot_id >= 1 && slot_id < 64) {
      pending_disable_slots_ |= (1ULL << slot_id);
    }
    PumpCommandRing(now);
    return;
  }

  DeviceSlot& dev = devices_[completed.dev_idx];
  if (!dev.in_use) {
    if (cc == kCcSuccess && slot_id >= 1 && slot_id < 64) {
      pending_disable_slots_ |= (1ULL << slot_id);
    }
    PumpCommandRing(now);
    return;
  }

  switch (dev.state) {
    case DevFsm::kWaitingEnableSlot:
      if (cc != kCcSuccess || slot_id < 1 || slot_id > slots_) {
        RetryOrDisablePort(dev.port, now, "enable slot command error");
      } else {
        dev.slot_id = slot_id;
        dev.state = DevFsm::kWantAddressDevice;
      }
      break;
    case DevFsm::kWaitingAddressDevice:
      if (cc != kCcSuccess) {
        RetryOrDisablePort(dev.port, now, "address device command error");
      } else {
        dev.state = DevFsm::kAddressSettle;
        dev.deadline = now + TicksForMs(*platform_, kAddressSettleMs);
      }
      break;
    case DevFsm::kWaitingEvaluateEp0:
      if (cc != kCcSuccess) {
        RetryOrDisablePort(dev.port, now, "evaluate context command error");
      } else {
        dev.state = DevFsm::kWaitingGetDesc18;
        SubmitControlTransfer(dev, kUsbReqTypeStdDevIn, kUsbReqGetDescriptor,
                              static_cast<uint16_t>(kUsbDescDevice << 8), 0, 18,
                              now);
      }
      break;
    case DevFsm::kWaitingConfigureEndpoint:
      if (cc != kCcSuccess) {
        RetryOrDisablePort(dev.port, now, "configure endpoint command error");
      } else {
        dev.state = DevFsm::kWaitingSetConfig;
        SubmitControlTransfer(dev, kUsbReqTypeStdDevOut,
                              kUsbReqSetConfiguration, dev.kbd.config_value, 0,
                              0, now);
      }
      break;
    case DevFsm::kWaitingResetEp0:
      dev.state = DevFsm::kWantSetDeqEp0;
      break;
    case DevFsm::kWaitingSetDeqEp0:
      if (cc != kCcSuccess) {
        RetryOrDisablePort(dev.port, now, "ep0 set dequeue command error");
      } else if (dev.after_ep0_recovery == Ep0RecoveryNext::kSetIdle) {
        StartSetIdle(dev, now);
      } else {
        FinishKeyboardAttach(dev);
      }
      break;
    case DevFsm::kWaitingResetIntrEp:
      dev.state = DevFsm::kWantSetDeqIntrEp;
      break;
    case DevFsm::kWaitingSetDeqIntrEp:
      if (cc != kCcSuccess) {
        RetryOrDisablePort(dev.port, now, "interrupt ep recovery error");
      } else {
        dev.state = DevFsm::kConfigured;
        ArmInterruptTransfers(dev);
      }
      break;
    default:
      break;
  }

  PumpCommandRing(now);
}

inline void XhciController::SubmitControlTransfer(
    DeviceSlot& dev,                //
    const uint8_t bm_request_type,  //
    const uint8_t b_request,        //
    const uint16_t w_value,         //
    const uint16_t w_index,         //
    const uint16_t w_length,        //
    const int64_t now) {
  ctrl_internal::ZeroBytesVolatile(dev.ctrl_buf_va, kControlBufferSize);
  if (w_length > 0) {
    const bool dir_in = (bm_request_type & 0x80u) != 0;
    const uint32_t trt = dir_in ? kTrtInData : kTrtOutData;
    const Trb td[3] = {
        MakeSetupStageTrb(bm_request_type, b_request, w_value, w_index,
                          w_length, trt),
        MakeDataStageTrb(dev.ctrl_buf_phys, w_length, dir_in),
        MakeStatusStageTrb(!dir_in),
    };
    uint64_t phys[3] = {};
    dev.ep0_ring.EnqueueTd(td, 3, phys);
    dev.ctrl_setup_phys = phys[0];
    dev.ctrl_data_phys = phys[1];
    dev.ctrl_status_phys = phys[2];
  } else {
    const Trb td[2] = {
        MakeSetupStageTrb(bm_request_type, b_request, w_value, w_index, 0,
                          kTrtNoData),
        MakeStatusStageTrb(true),
    };
    uint64_t phys[2] = {};
    dev.ep0_ring.EnqueueTd(td, 2, phys);
    dev.ctrl_setup_phys = phys[0];
    dev.ctrl_data_phys = 0;
    dev.ctrl_status_phys = phys[1];
  }
  dev.ctrl_req_len = w_length;
  dev.ctrl_actual_len = w_length;
  dev.ctrl_in_flight = true;
  dev.deadline = now + TicksForMs(*platform_, kControlTimeoutMs);
  mmio_->Write32(caps_.Doorbell(dev.slot_id), 1u);
}

inline void XhciController::OnTransferEvent(const Trb& ev, const int64_t now) {
  const uint8_t slot_id = TrbSlotId(ev);
  const uint8_t dci = TrbEndpointId(ev);
  if (slot_id == 0) {
    return;
  }
  for (int i = 0; i < kMaxDevices; ++i) {
    DeviceSlot& dev = devices_[i];
    if (!dev.in_use || dev.slot_id != slot_id) {
      continue;
    }
    if (dci == 1) {
      OnEp0TransferEvent(dev, ev, now);
    } else if (dci == dev.kbd.ep_dci) {
      OnInterruptTransferEvent(dev, ev, now);
    }
    return;
  }
}

inline void XhciController::OnEp0TransferEvent(DeviceSlot& dev,  //
                                               const Trb& ev,    //
                                               const int64_t now) {
  if (!dev.ctrl_in_flight) {
    return;
  }
  const uint64_t ptr = TrbPointer(ev);
  const uint8_t cc = TrbCompletionCode(ev);

  if (dev.ctrl_data_phys != 0 && ptr == dev.ctrl_data_phys &&
      (cc == kCcShortPacket || cc == kCcSuccess)) {
    const uint32_t res = TrbTransferResidual(ev);
    dev.ctrl_actual_len = res < dev.ctrl_req_len
                              ? static_cast<uint16_t>(dev.ctrl_req_len - res)
                              : 0u;
    return;
  }

  if (ptr == dev.ctrl_status_phys &&
      (cc == kCcSuccess || cc == kCcShortPacket)) {
    dev.ctrl_in_flight = false;
    dev.deadline = kNoDeadline;
    OnControlTransferComplete(dev, true, cc, now);
    return;
  }

  if (cc != kCcSuccess) {
    dev.ctrl_in_flight = false;
    dev.deadline = kNoDeadline;
    OnControlTransferComplete(dev, false, cc, now);
  }
}

inline void XhciController::StartSetIdle(DeviceSlot& dev, const int64_t now) {
  dev.state = DevFsm::kWaitingSetIdle;
  SubmitControlTransfer(dev, kUsbReqTypeClassIfaceOut, kHidReqSetIdle, 0,
                        dev.kbd.interface_number, 0, now);
}

inline void XhciController::OnControlTransferComplete(DeviceSlot& dev,   //
                                                      const bool ok,     //
                                                      const uint8_t cc,  //
                                                      const int64_t now) {
  (void)cc;
  switch (dev.state) {
    case DevFsm::kWaitingGetDesc8: {
      if (!ok) {
        RetryOrDisablePort(dev.port, now, "get device descriptor prefix error");
        return;
      }
      uint8_t b_mps0 = 0;
      if (!ParseDeviceDescriptorPrefix(dev.ctrl_buf_va, dev.ctrl_actual_len,
                                       &b_mps0)) {
        RetryOrDisablePort(dev.port, now, "invalid device descriptor prefix");
        return;
      }
      const uint16_t actual_mps = DecodeEp0MaxPacketSize(dev.speed, b_mps0);
      if (actual_mps != dev.ep0_mps) {
        dev.ep0_mps = actual_mps;
        dev.state = DevFsm::kWantEvaluateEp0;
        PumpCommandRing(now);
      } else {
        dev.state = DevFsm::kWaitingGetDesc18;
        SubmitControlTransfer(dev, kUsbReqTypeStdDevIn, kUsbReqGetDescriptor,
                              static_cast<uint16_t>(kUsbDescDevice << 8), 0, 18,
                              now);
      }
      return;
    }
    case DevFsm::kWaitingGetDesc18: {
      if (!ok) {
        RetryOrDisablePort(dev.port, now, "get device descriptor error");
        return;
      }
      uint8_t dev_class = 0;
      uint8_t num_configs = 0;
      if (!ParseDeviceDescriptorFull(dev.ctrl_buf_va, dev.ctrl_actual_len,
                                     &dev_class, &num_configs) ||
          num_configs == 0) {
        RetryOrDisablePort(dev.port, now, "invalid full device descriptor");
        return;
      }
      if (dev_class == kUsbClassHub) {
        DisableUnsupportedDevice(dev, "ignoring usb hub");
        PumpCommandRing(now);
        return;
      }
      dev.state = DevFsm::kWaitingGetConfig9;
      SubmitControlTransfer(dev, kUsbReqTypeStdDevIn, kUsbReqGetDescriptor,
                            static_cast<uint16_t>(kUsbDescConfiguration << 8),
                            0, 9, now);
      return;
    }
    case DevFsm::kWaitingGetConfig9: {
      if (!ok) {
        RetryOrDisablePort(dev.port, now, "get config header error");
        return;
      }
      uint16_t w_total_len = 0;
      uint8_t cfg_val = 0;
      if (!ParseConfigDescriptorHeader(dev.ctrl_buf_va, dev.ctrl_actual_len,
                                       &w_total_len, &cfg_val)) {
        DisableUnsupportedDevice(dev, "invalid config descriptor header");
        PumpCommandRing(now);
        return;
      }
      const uint16_t req_len =
          w_total_len > kControlBufferSize ? kControlBufferSize : w_total_len;
      dev.state = DevFsm::kWaitingGetConfigFull;
      SubmitControlTransfer(dev, kUsbReqTypeStdDevIn, kUsbReqGetDescriptor,
                            static_cast<uint16_t>(kUsbDescConfiguration << 8),
                            0, req_len, now);
      return;
    }
    case DevFsm::kWaitingGetConfigFull: {
      if (!ok) {
        RetryOrDisablePort(dev.port, now, "get full config descriptor error");
        return;
      }
      BootKeyboardMatch match{};
      if (!FindBootKeyboard(dev.ctrl_buf_va, dev.ctrl_actual_len, &match)) {
        DisableUnsupportedDevice(dev, "no boot keyboard interface");
        PumpCommandRing(now);
        return;
      }
      dev.kbd = match;
      dev.state = DevFsm::kWantConfigureEndpoint;
      PumpCommandRing(now);
      return;
    }
    case DevFsm::kWaitingSetConfig: {
      if (!ok) {
        RetryOrDisablePort(dev.port, now, "set configuration error");
        return;
      }
      dev.state = DevFsm::kWaitingSetProtocol;
      SubmitControlTransfer(dev, kUsbReqTypeClassIfaceOut, kHidReqSetProtocol,
                            0, dev.kbd.interface_number, 0, now);
      return;
    }
    case DevFsm::kWaitingSetProtocol: {
      if (ok) {
        StartSetIdle(dev, now);
      } else {
        StartLogLine()
            .Str("port ")
            .Dec(dev.port)
            .Str(": SET_PROTOCOL stalled, recovering ep0")
            .Emit(*platform_);
        dev.after_ep0_recovery = Ep0RecoveryNext::kSetIdle;
        dev.state = DevFsm::kWantResetEp0;
        PumpCommandRing(now);
      }
      return;
    }
    case DevFsm::kWaitingSetIdle: {
      if (ok) {
        FinishKeyboardAttach(dev);
      } else {
        StartLogLine()
            .Str("port ")
            .Dec(dev.port)
            .Str(": SET_IDLE stalled, recovering ep0")
            .Emit(*platform_);
        dev.after_ep0_recovery = Ep0RecoveryNext::kFinishAttach;
        dev.state = DevFsm::kWantResetEp0;
        PumpCommandRing(now);
      }
      return;
    }
    default:
      return;
  }
}

inline void XhciController::ArmInterruptTransfers(DeviceSlot& dev) {
  const uint32_t xfer_len = dev.kbd.max_packet_size < kReportBufferSize
                                ? dev.kbd.max_packet_size
                                : kReportBufferSize;
  dev.next_rearm_idx = 0;
  for (int i = 0; i < kReportTrbs; ++i) {
    const uint64_t buf_phys =
        dev.report_buf_phys + static_cast<uint64_t>(i * kReportBufferSize);
    dev.report_trb_phys[i] =
        dev.intr_ring.Enqueue(MakeNormalTrb(buf_phys, xfer_len, true, true));
  }
  mmio_->Write32(caps_.Doorbell(dev.slot_id), dev.kbd.ep_dci);
}

inline void XhciController::FinishKeyboardAttach(DeviceSlot& dev) {
  dev.hid_state = HidKeyboardState{};
  dev.intr_errors = 0;
  dev.deadline = kNoDeadline;
  dev.state = DevFsm::kConfigured;
  if (dev.port >= 1 && dev.port <= ports_) {
    port_states_[dev.port - 1].fsm = PortFsm::kActive;
    port_states_[dev.port - 1].deadline = kNoDeadline;
  }
  ArmInterruptTransfers(dev);
  if (!dev.announced) {
    dev.announced = true;
    ++attached_keyboards_;
    LogLine()
        .Str(kLogPrefix)
        .Str("keyboard attached: ")
        .Str(label_)
        .Str(" port ")
        .Dec(dev.port)
        .Str(" (")
        .Str(SpeedName(dev.speed))
        .Str(")")
        .Emit(*platform_);
  }
}

inline void XhciController::OnInterruptTransferEvent(DeviceSlot& dev,  //
                                                     const Trb& ev,    //
                                                     const int64_t now) {
  if (dev.state != DevFsm::kConfigured) {
    return;
  }
  const uint64_t ptr = TrbPointer(ev);
  const uint8_t cc = TrbCompletionCode(ev);

  int buf_idx = -1;
  for (int i = 0; i < kReportTrbs; ++i) {
    if (dev.report_trb_phys[i] == ptr) {
      buf_idx = i;
      break;
    }
  }

  if (cc == kCcSuccess || cc == kCcShortPacket) {
    dev.intr_errors = 0;
    const uint32_t req_len = dev.kbd.max_packet_size < kReportBufferSize
                                 ? dev.kbd.max_packet_size
                                 : kReportBufferSize;
    const uint32_t res = TrbTransferResidual(ev);
    const uint32_t actual = res < req_len ? req_len - res : 0u;
    const int idx = buf_idx >= 0 ? buf_idx : dev.next_rearm_idx;
    dev.next_rearm_idx = (idx + 1) % kReportTrbs;
    if (actual >= static_cast<uint32_t>(kBootReportMinSize)) {
      char out[kBootReportKeys] = {};
      const int n = TranslateBootReport(
          &dev.hid_state, locks_, dev.report_buf_va + idx * kReportBufferSize,
          static_cast<int>(actual), out, static_cast<int>(sizeof(out)));
      if (n > 0) {
        platform_->PushInput(out, n);
      }
    }
    const uint64_t buf_phys =
        dev.report_buf_phys + static_cast<uint64_t>(idx * kReportBufferSize);
    dev.report_trb_phys[idx] =
        dev.intr_ring.Enqueue(MakeNormalTrb(buf_phys, req_len, true, true));
    mmio_->Write32(caps_.Doorbell(dev.slot_id), dev.kbd.ep_dci);
    return;
  }

  if (cc == kCcRingUnderrun || cc == kCcRingOverrun || cc == kCcStopped ||
      cc == kCcStoppedLengthInvalid) {
    return;
  }

  // Check if port disconnected before attempting endpoint recovery.
  const uint32_t ps = ReadPortSc(dev.port);
  if (ps == kMmioAllOnes || (ps & kPortScCcs) == 0) {
    HandlePortDisconnect(dev.port, now);
    PumpCommandRing(now);
    return;
  }

  ++dev.intr_errors;
  if (dev.intr_errors <= kMaxInterruptErrors) {
    StartLogLine()
        .Str("port ")
        .Dec(dev.port)
        .Str(": interrupt endpoint error, recovering")
        .Emit(*platform_);
    dev.state = DevFsm::kWantResetIntrEp;
    PumpCommandRing(now);
  } else {
    RetryOrDisablePort(dev.port, now, "interrupt endpoint error limit");
    PumpCommandRing(now);
  }
}

inline void XhciController::AdvanceStateMachines(const int64_t now) {
  if (state_ != XhciState::kRunning) {
    return;
  }

  if (event_ring_full_rescan_) {
    event_ring_full_rescan_ = false;
    for (int port = 1; port <= ports_; ++port) {
      ScanPort(port, now);
    }
  }

  if (cmd_state_.busy && now >= cmd_state_.deadline) {
    AbortCommandAndRecover("command timeout");
    return;
  }

  for (int port = 1; port <= ports_; ++port) {
    PortState& p = port_states_[port - 1];
    if (p.deadline == kNoDeadline || now < p.deadline) {
      continue;
    }
    if (p.fsm == PortFsm::kDebouncing) {
      StartPortResetOrUsb3(port, now);
    } else if (p.fsm == PortFsm::kResetting) {
      const uint32_t ps = ReadPortSc(port);
      if (ps != kMmioAllOnes && (ps & kPortScCcs) != 0 &&
          (ps & kPortScPr) == 0 && (ps & kPortScPed) != 0) {
        if ((ps & kPortScChangeMask) != 0) {
          WritePortSc(port,
                      (ps & kPortScNeutralMask) | (ps & kPortScChangeMask));
        }
        p.speed = PortSpeedOf(ps);
        p.fsm = PortFsm::kResetRecovery;
        p.deadline = now + TicksForMs(*platform_, kPortResetRecoveryMs);
      } else {
        RetryOrDisablePort(port, now, "port reset timeout");
      }
    } else if (p.fsm == PortFsm::kResetRecovery) {
      const uint32_t ps = ReadPortSc(port);
      if (ps == kMmioAllOnes || (ps & kPortScCcs) == 0) {
        HandlePortDisconnect(port, now);
      } else if ((ps & kPortScPed) == 0) {
        RetryOrDisablePort(port, now, "port disabled after reset");
      } else {
        if (PortSpeedOf(ps) != 0) {
          p.speed = PortSpeedOf(ps);
        }
        p.fsm = PortFsm::kEnumerating;
        p.deadline = kNoDeadline;
        BeginDeviceEnumeration(port, now);
      }
    }
  }

  for (int i = 0; i < kMaxDevices; ++i) {
    DeviceSlot& dev = devices_[i];
    if (!dev.in_use || dev.deadline == kNoDeadline || now < dev.deadline) {
      continue;
    }
    if (dev.state == DevFsm::kAddressSettle) {
      dev.deadline = kNoDeadline;
      dev.state = DevFsm::kWaitingGetDesc8;
      SubmitControlTransfer(dev, kUsbReqTypeStdDevIn, kUsbReqGetDescriptor,
                            static_cast<uint16_t>(kUsbDescDevice << 8), 0, 8,
                            now);
    } else if (dev.ctrl_in_flight) {
      dev.ctrl_in_flight = false;
      dev.deadline = kNoDeadline;
      RetryOrDisablePort(dev.port, now, "control transfer timeout");
    }
  }

  PumpCommandRing(now);
}

inline void XhciController::AbortCommandAndRecover(const char* const reason) {
  Write64LoHi(*mmio_, caps_.Op(kOpCrcr), static_cast<uint64_t>(kCrcrCa));
  (void)Poll(caps_.Op(kOpCrcr), kCrcrCrr, 0, kHaltTimeoutMs);
  Recover(reason);
}

inline void XhciController::DetachAllKeyboards() {
  for (int i = 0; i < kMaxDevices; ++i) {
    DeviceSlot& dev = devices_[i];
    if (dev.in_use && dev.announced) {
      dev.announced = false;
      LogLine()
          .Str(kLogPrefix)
          .Str("keyboard detached: ")
          .Str(label_)
          .Str(" port ")
          .Dec(dev.port)
          .Emit(*platform_);
    }
  }
  attached_keyboards_ = 0;
}

inline void XhciController::Recover(const char* const reason) {
  StartLogLine()
      .Str("controller error (")
      .Str(reason)
      .Str("), recovering")
      .Emit(*platform_);
  DetachAllKeyboards();

  ++recoveries_;
  if (recoveries_ > kMaxRecoveries) {
    StartLogLine().Str("error: recovery limit exceeded").Emit(*platform_);
    state_ = XhciState::kError;
    return;
  }

  const IrqMode saved_mode = irq_mode_;
  in_recovery_ = true;
  if (TakeOwnershipAndReset() != XhciStatus::kOk) {
    in_recovery_ = false;
    return;
  }
  (void)Start(saved_mode);
  in_recovery_ = false;
}

}  // namespace protos::usb

#endif  // PROTOS_USER_XHCI_CONTROLLER_H_
