// usbkbd: ring-3 xHCI USB boot-keyboard driver (PCI class 0C/03/30).
//
// Discovers and claims all PCI xHCI host controllers, performs two-phase
// bring-up (Phase 1: MMIO map + BIOS handoff + HCRST with Bus Master off;
// Phase 2: Bus Master on + MSI-X/MSI allocation + ring/port start), and runs
// a unified interrupt- and deadline-driven event loop feeding translated ASCII
// keystrokes into the console input ring via ConsolePush().
//
// Only init starts it as a driver (spawn_driver(), at boot). Started by plain
// spawn() instead (for example by `sh`'s `run usbkbd`), it has no driver
// capability: it reports that and exits with kDriverExitNotDriver.

#include <cstdint>
#include <type_traits>

#include "syscall_abi.h"
#include "user/syscall.h"
#include "user/usbkbd/usb_hid.h"
#include "user/usbkbd/xhci_controller.h"
#include "user/usbkbd/xhci_platform.h"

namespace protos::user {
namespace {

constexpr int kMaxControllers = kSyscallMaxClaims;
constexpr uint8_t kPciClassSerialBus = 0x0C;
constexpr uint8_t kPciSubclassUsb = 0x03;
constexpr uint8_t kPciProgIfXhci = 0x30;
constexpr int64_t kPageSize = 4096;

class Ring3Platform final : public usb::Platform {
 public:
  constexpr Ring3Platform() = default;

  usb::DmaBuffer DmaAlloc(const int64_t size, const bool below_4gib) override {
    if (size <= 0) {
      return {};
    }
    const int64_t aligned_size = (size + (kPageSize - 1)) & ~(kPageSize - 1);
    const uint64_t flags = below_4gib ? kSyscallDmaBelow4GiB : 0;
    const StatusOr<DmaAllocation> dma =
        ::protos::user::DmaAlloc(aligned_size, flags);
    if (!dma.ok() || dma->phys == 0) {
      return {};
    }
    return usb::DmaBuffer{
        .va = reinterpret_cast<void*>(dma->va),
        .phys = dma->phys,
        .size = aligned_size,
    };
  }

  int64_t NowTicks() override { return GetTicks(); }

  int64_t TicksPerSecond() override { return kSyscallTicksPerSecond; }

  void Relax() override { (void)IrqWait(/*mask=*/0, /*timeout_ticks=*/1); }

  void PushInput(const char* const bytes, const int count) override {
    if (bytes == nullptr || count <= 0) {
      return;
    }
    int offset = 0;
    while (offset < count) {
      int chunk = count - offset;
      if (chunk > static_cast<int>(kSyscallConsolePushMaxSize)) {
        chunk = static_cast<int>(kSyscallConsolePushMaxSize);
      }
      (void)ConsolePush(bytes + offset, chunk);
      offset += chunk;
    }
  }

  void Log(const char* const line, const int length) override {
    if (line != nullptr && length > 0) {
      (void)Write(line, length);
    }
  }
};

struct ControllerSlot {
  int pci_index = -1;
  uint64_t irq_bit_mask = 0;
  usb::IrqMode irq_mode = usb::IrqMode::kMsix;
  bool phase1_ok = false;
  bool active = false;
  // Restarts of a controller that stopped running: when the next one is due
  // (0: none scheduled), how many ran since the controller last ran for
  // kHealthyTicks, and since when it runs again.
  int64_t restart_at = 0;
  int restarts = 0;
  int64_t running_since = 0;
  char label[8] = {};
};

// A controller stops running (XhciState::kError) when its recovery budget
// runs out, a controller reset fails, or it stops decoding. Nothing would
// start the driver again if it exited, so it restarts such a controller after
// a back-off that doubles each time (10 s, 20 s, ... 160 s), and gives up on
// it only after kMaxRestarts restarts without kHealthyTicks of running.
constexpr int64_t kRestartDelayTicks = 10 * kSyscallTicksPerSecond;
constexpr int kMaxRestarts = 5;
constexpr int64_t kHealthyTicks = 300 * kSyscallTicksPerSecond;

constinit Ring3Platform g_platform{};
constinit usb::KeyboardLocks g_shared_locks{};
constinit usb::MappedMmioWindow g_mmio[kMaxControllers]{};
constinit usb::XhciController g_controllers[kMaxControllers]{};
constinit ControllerSlot g_slots[kMaxControllers]{};
static_assert(std::is_trivially_destructible_v<decltype(g_platform)>);
static_assert(std::is_trivially_destructible_v<decltype(g_shared_locks)>);
static_assert(std::is_trivially_destructible_v<decltype(g_mmio)>);
static_assert(std::is_trivially_destructible_v<decltype(g_controllers)>);
static_assert(std::is_trivially_destructible_v<decltype(g_slots)>);

static bool IsXhciDevice(const PciDeviceInfo& info) {
  if (info.claimable == 0 || info.class_code != kPciClassSerialBus ||
      info.subclass != kPciSubclassUsb || info.prog_if != kPciProgIfXhci) {
    return false;
  }
  const PciBarInfo& bar0 = info.bars[0];
  const bool is_mem_bar = (bar0.type == PciBarInfoType::kMemory32 ||
                           bar0.type == PciBarInfoType::kMemory64);
  return is_mem_bar && bar0.phys_base != 0 && bar0.size >= kPageSize;
}

// Maps the registers in BAR0 (at most kSyscallMmioMaxSize bytes) into a
// MappedMmioWindow.
//
// A PCIe function with MSI-X (Message Signaled Interrupts - Extended) places
// its MSI-X Table (per-vector target address/data/mask entries) and PBA
// (Pending Bit Array bitmap) inside a memory BAR, often BAR0. The kernel
// programs MSI-X itself via IrqAlloc() and refuses to map any 4 KiB page that
// overlaps the MSI-X table or PBA (returning kSyscallErrorAccess without
// consuming virtual address space, so user space cannot forge MSI writes),
// and does not report their BAR offsets; no other register may share their
// pages (PCIe 7.7.2). So BAR0 is mapped as the longest runs of pages that the
// kernel accepts, one mmio_map() each, leaving holes for the MSI-X pages:
// e.g. pages 0..2 of QEMU's `qemu-xhci`, whose 16 KiB BAR0 has the MSI-X table
// and PBA in page 3, or the operational registers below and the runtime
// registers, doorbells or extended capabilities above an MSI-X page in the
// middle of BAR0.
//
// Returns the mapped window, or the error of the first mmio_map() that failed
// otherwise.
static StatusOr<usb::MappedMmioWindow> MapControllerBar0(
    const PciBarInfo& bar0) {
  int64_t limit = bar0.size;
  // Clamp to the per-syscall mmio_map() limit (16 MiB) instead of failing when
  // BAR0 is larger: standard xHCI registers sit near the start of BAR0, while
  // the tail of a large BAR0 stays unmapped. DecodeCaps() and
  // ParseExtendedCaps() validate every required register offset against
  // window.Size() and fail with kBadCapabilities if any lies past the mapped
  // prefix.
  if (limit > kSyscallMmioMaxSize) {
    limit = kSyscallMmioMaxSize;
  }
  limit &= ~(kPageSize - 1);
  usb::MappedMmioWindow window{};
  window.Init(nullptr, 0);
  int64_t offset = 0;
  while (offset < limit) {
    int64_t size = limit - offset;
    StatusOr<uintptr_t> va = Error(kSyscallErrorAccess);
    while (size >= kPageSize) {
      va = MmioMap(bar0.phys_base + static_cast<uint64_t>(offset), size);
      if (va.status() != kSyscallErrorAccess) {
        break;
      }
      size -= kPageSize;
    }
    if (!va.ok() && va.status() != kSyscallErrorAccess) {
      return va.status();
    }
    if (size < kPageSize) {
      offset += kPageSize;  // The page at `offset` holds MSI-X structures
      continue;
    }
    if (!window.AddSegment(offset,                                    //
                           reinterpret_cast<volatile uint8_t*>(*va),  //
                           size)) {
      break;  // Out of segments: the registers above stay unmapped
    }
    offset += size;
  }
  // Standards-compliant xHCI + PCIe hardware always has at least one mappable
  // page in BAR0 (capability registers at 0x00 and MSI-X isolated on dedicated
  // 4 KiB pages per PCIe 7.7.2.2). However, if a non-compliant device shares
  // every page of BAR0 with MSI-X structures (e.g. a 4 KiB BAR0 packing both
  // registers and MSI-X) or every mmio_map() returns kSyscallErrorAccess, all
  // pages are skipped above and `window` has no segments. (If only page 0 was
  // skipped while a later page mapped, DecodeCaps() reads 0xFFFFFFFF from the
  // unmapped hole at 0x00 and fails with kNotDecoding.)
  if (window.Size() <= 0) {
    return kSyscallErrorAccess;
  }
  return window;
}

struct ControllerIrq {
  uint64_t bit_mask = 0;
  usb::IrqMode mode = usb::IrqMode::kMsix;
};

static StatusOr<ControllerIrq> AllocateControllerIrq(
    const int pci_index, const PciDeviceInfo& info) {
  if (info.msix_vectors > 0) {
    const int64_t bit = IrqAlloc(pci_index, /*count=*/1, kSyscallIrqModeMsix);
    if (!IsError(bit) && bit >= 0 && bit < kSyscallIrqMaxCount) {
      return ControllerIrq{
          .bit_mask = uint64_t{1} << static_cast<uint32_t>(bit),
          .mode = usb::IrqMode::kMsix,
      };
    }
  }
  if (info.msi_vectors > 0) {
    const int64_t bit = IrqAlloc(pci_index, /*count=*/1, kSyscallIrqModeMsi);
    if (!IsError(bit) && bit >= 0 && bit < kSyscallIrqMaxCount) {
      return ControllerIrq{
          .bit_mask = uint64_t{1} << static_cast<uint32_t>(bit),
          .mode = usb::IrqMode::kMsi,
      };
    }
  }
  return kStatusError;
}

static void LogBringUpError(const char* const label, const char* const step) {
  usb::LogLine line;
  line.Str(usb::kLogPrefix)
      .Str(label)
      .Str(": error: ")
      .Str(step)
      .Emit(g_platform);
}

// Restarts controller `i`, which no longer runs, once its back-off is over.
// Returns the tick of its next restart, or usb::kNoDeadline if it runs again
// or was given up on.
static int64_t ServiceStoppedController(const int i, const int64_t now) {
  ControllerSlot& slot = g_slots[i];
  if (slot.restarts >= kMaxRestarts) {
    return usb::kNoDeadline;
  }
  if (slot.restart_at == 0) {
    slot.restart_at = now + (kRestartDelayTicks << slot.restarts);
  }
  if (now < slot.restart_at) {
    return slot.restart_at;
  }
  slot.restart_at = 0;
  ++slot.restarts;
  usb::LogLine line;
  line.Str(usb::kLogPrefix)
      .Str(slot.label)
      .Str(": restarting the controller")
      .Emit(g_platform);
  usb::XhciController& controller = g_controllers[i];
  if (controller.TakeOwnershipAndReset() == usb::XhciStatus::kOk &&
      controller.Start(slot.irq_mode) == usb::XhciStatus::kOk) {
    slot.running_since = GetTicks();
    return usb::kNoDeadline;
  }
  if (slot.restarts >= kMaxRestarts) {
    LogBringUpError(slot.label, "controller given up");
    return usb::kNoDeadline;
  }
  slot.restart_at = GetTicks() + (kRestartDelayTicks << slot.restarts);
  return slot.restart_at;
}

}  // namespace

int64_t Main(const uint64_t arg) {
  const int64_t pci_count = GetPciDeviceCount();
  if (pci_count == kSyscallErrorPermission) {
    usb::LogLine line;
    line.Str(usb::kLogPrefix)
        .Str("not started as a driver (init starts usbkbd at boot)")
        .Emit(g_platform);
    return kDriverExitNotDriver;
  }
  if (pci_count <= 0) {
    if (arg == kDriverArgHandoffOnly) {
      return 0;
    }
    usb::LogLine line;
    line.Str(usb::kLogPrefix).Str("no xHCI controllers found").Emit(g_platform);
    return (arg == kDriverArgNormal) ? 0 : 1;
  }

  // Phase 1: PCI discovery, PciClaim (Memory Space on, Bus Master untouched),
  // BAR0 MMIO mapping, BIOS-to-OS handoff, halt, and (except in handoff-only
  // mode) HCRST.
  int discovered_count = 0;
  int decoding_count = 0;
  for (int64_t idx = 0; idx < pci_count && discovered_count < kMaxControllers;
       ++idx) {
    const StatusOr<PciDeviceInfo> info = GetPciInfo(idx);
    if (!info.ok() || !IsXhciDevice(*info)) {
      continue;
    }
    const int slot_idx = discovered_count++;
    ControllerSlot& slot = g_slots[slot_idx];
    slot = ControllerSlot{};
    slot.pci_index = static_cast<int>(idx);
    usb::FormatBdf(info->bus, info->device, info->function, slot.label);

    if (PciClaim(idx) != 0) {
      LogBringUpError(slot.label, "claim");
      continue;
    }

    const StatusOr<usb::MappedMmioWindow> mmio =
        MapControllerBar0(info->bars[0]);
    if (!mmio.ok()) {
      (void)PciBusMaster(idx, /*enable=*/0);
      LogBringUpError(slot.label, "mmio_map");
      continue;
    }
    g_mmio[slot_idx] = *mmio;

    g_controllers[slot_idx].Init(g_platform,        //
                                 g_mmio[slot_idx],  //
                                 slot.label,        //
                                 &g_shared_locks);
    const usb::XhciStatus phase1_status =
        (arg == kDriverArgHandoffOnly)
            ? g_controllers[slot_idx].TakeOwnershipAndHalt()
            : g_controllers[slot_idx].TakeOwnershipAndReset();
    if (phase1_status != usb::XhciStatus::kNotDecoding) {
      ++decoding_count;
    }
    if (phase1_status != usb::XhciStatus::kOk) {
      (void)PciBusMaster(idx, /*enable=*/0);
      LogBringUpError(slot.label,
                      (arg == kDriverArgHandoffOnly) ? "handoff" : "reset");
      continue;
    }

    slot.phase1_ok = true;
  }

  if (arg == kDriverArgHandoffOnly) {
    return 0;
  }

  if (discovered_count == 0) {
    usb::LogLine line;
    line.Str(usb::kLogPrefix).Str("no xHCI controllers found").Emit(g_platform);
    return (arg == kDriverArgNormal) ? 0 : 1;
  }

  // Phase 2: Enable Bus Master (including upstream bridges), allocate MSI-X or
  // MSI interrupt vector, and start controller rings and port scanning.
  uint64_t active_irq_mask = 0;
  int active_count = 0;
  for (int i = 0; i < discovered_count; ++i) {
    ControllerSlot& slot = g_slots[i];
    if (!slot.phase1_ok) {
      continue;
    }
    const StatusOr<PciDeviceInfo> info = GetPciInfo(slot.pci_index);
    if (!info.ok()) {
      continue;
    }
    if (PciBusMaster(slot.pci_index, /*enable=*/1) != 0) {
      LogBringUpError(slot.label, "bus_master");
      continue;
    }
    const StatusOr<ControllerIrq> irq =
        AllocateControllerIrq(slot.pci_index, *info);
    if (!irq.ok()) {
      (void)PciBusMaster(slot.pci_index, /*enable=*/0);
      LogBringUpError(slot.label, "irq_alloc");
      continue;
    }
    slot.irq_bit_mask = irq->bit_mask;
    slot.irq_mode = irq->mode;
    if (g_controllers[i].Start(slot.irq_mode) != usb::XhciStatus::kOk) {
      (void)PciBusMaster(slot.pci_index, /*enable=*/0);
      LogBringUpError(slot.label, "start");
      continue;
    }
    slot.active = true;
    active_irq_mask |= slot.irq_bit_mask;
    ++active_count;
  }

  if (active_count == 0) {
    return 1;
  }
  if (arg != kDriverArgNormal && active_count != decoding_count) {
    return 2;
  }

  if (arg == kDriverArgExitAfterReady || arg == kDriverArgFaultAfterReady) {
    for (int i = 0; i < discovered_count; ++i) {
      if (g_slots[i].active &&
          g_controllers[i].state() == usb::XhciState::kRunning) {
        g_controllers[i].HandleInterrupt();
      }
    }
    if (arg == kDriverArgFaultAfterReady) {
      asm volatile("ud2");
    }
    return 0;
  }

  // Main event loop: block on IrqWait until either an interrupt fires or the
  // earliest non-blocking state-machine deadline expires. When no deadline is
  // pending (`kNoDeadline`), `timeout_ticks = -1` blocks indefinitely without
  // any periodic timer wakeups. The loop ends only when every controller was
  // given up on (ServiceStoppedController()).
  for (;;) {
    int live_controllers = 0;
    int64_t earliest_deadline = usb::kNoDeadline;
    for (int i = 0; i < discovered_count; ++i) {
      ControllerSlot& slot = g_slots[i];
      if (!slot.active) {
        continue;
      }
      const int64_t now = GetTicks();
      int64_t d = usb::kNoDeadline;
      if (g_controllers[i].state() != usb::XhciState::kRunning) {
        d = ServiceStoppedController(i, now);
      }
      if (g_controllers[i].state() == usb::XhciState::kRunning) {
        if (slot.restarts > 0 && now - slot.running_since >= kHealthyTicks) {
          slot.restarts = 0;
        }
        d = g_controllers[i].NextDeadline();
      } else if (d == usb::kNoDeadline) {
        continue;  // Given up on
      }
      ++live_controllers;
      if (d < earliest_deadline) {
        earliest_deadline = d;
      }
    }
    if (live_controllers == 0) {
      return 1;
    }

    constexpr int64_t kMaxPendingDeadlineWaitTicks = 20;
    int64_t timeout_ticks = -1;
    if (earliest_deadline != usb::kNoDeadline) {
      const int64_t now = GetTicks();
      const int64_t remaining = earliest_deadline - now;
      if (remaining <= 0) {
        timeout_ticks = 0;
      } else if (remaining > kMaxPendingDeadlineWaitTicks) {
        timeout_ticks = kMaxPendingDeadlineWaitTicks;
      } else {
        timeout_ticks = remaining;
      }
    }

    (void)IrqWait(active_irq_mask, timeout_ticks);

    for (int i = 0; i < discovered_count; ++i) {
      if (g_slots[i].active &&
          g_controllers[i].state() == usb::XhciState::kRunning) {
        g_controllers[i].HandleInterrupt();
      }
    }
  }
}

}  // namespace protos::user
