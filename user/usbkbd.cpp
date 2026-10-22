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

#include "syscall_abi.h"
#include "user/syscall.h"
#include "user/usb_hid.h"
#include "user/xhci_controller.h"
#include "user/xhci_platform.h"

namespace protos::user {
namespace {

constexpr int kMaxControllers = 4;
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
    uint64_t phys = 0;
    const int64_t va = ::protos::user::DmaAlloc(aligned_size, &phys, flags);
    if (IsError(va) || phys == 0) {
      return {};
    }
    return usb::DmaBuffer{
        .va = reinterpret_cast<void*>(static_cast<uintptr_t>(va)),
        .phys = phys,
        .size = aligned_size,
    };
  }

  int64_t NowTicks() override { return GetTicks(); }

  int64_t TicksPerSecond() override { return kSyscallTicksPerSecond; }

  void Relax() override { (void)IrqWait(0, 1); }

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
  char label[8] = {};
};

constinit Ring3Platform g_platform{};
constinit usb::KeyboardLocks g_shared_locks{};
constinit usb::MappedMmioWindow g_mmio[kMaxControllers]{};
constinit usb::XhciController g_controllers[kMaxControllers]{};
constinit ControllerSlot g_slots[kMaxControllers]{};

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

// Maps the xHCI register portion of BAR0. If the device places its MSI-X table
// or PBA in the trailing page(s) of BAR0 (such as QEMU's `qemu-xhci`, whose
// 16 KiB BAR0 has xHCI registers in pages 0..2 and MSI-X table/PBA in page 3),
// the kernel withholds the MSI-X page(s) and rejects a full-BAR MmioMap() with
// kSyscallErrorAccess without consuming any virtual address space. Trimming
// one 4 KiB page at a time maps the contiguous register prefix cleanly.
static int64_t MapControllerBar0(const PciBarInfo& bar0,
                                 int64_t* const out_mapped_size) {
  int64_t map_size = bar0.size;
  if (map_size > kSyscallMmioMaxSize) {
    map_size = kSyscallMmioMaxSize;
  }
  map_size &= ~(kPageSize - 1);
  while (map_size >= kPageSize) {
    const int64_t va = MmioMap(bar0.phys_base, map_size);
    if (!IsError(va)) {
      *out_mapped_size = map_size;
      return va;
    }
    if (va != kSyscallErrorAccess) {
      return va;
    }
    map_size -= kPageSize;
  }
  return kSyscallErrorAccess;
}

static bool AllocateControllerIrq(const int pci_index,           //
                                  const PciDeviceInfo& info,     //
                                  uint64_t* const out_bit_mask,  //
                                  usb::IrqMode* const out_mode) {
  if (info.msix_vectors > 0) {
    const int64_t bit = IrqAlloc(pci_index, 1, kSyscallIrqModeMsix);
    if (!IsError(bit) && bit >= 0 && bit < kSyscallIrqMaxCount) {
      *out_bit_mask = uint64_t{1} << static_cast<uint32_t>(bit);
      *out_mode = usb::IrqMode::kMsix;
      return true;
    }
  }
  if (info.msi_vectors > 0) {
    const int64_t bit = IrqAlloc(pci_index, 1, kSyscallIrqModeMsi);
    if (!IsError(bit) && bit >= 0 && bit < kSyscallIrqMaxCount) {
      *out_bit_mask = uint64_t{1} << static_cast<uint32_t>(bit);
      *out_mode = usb::IrqMode::kMsi;
      return true;
    }
  }
  return false;
}

static void LogBringUpError(const char* const label, const char* const step) {
  usb::LogLine line;
  line.Str(usb::kLogPrefix)
      .Str(label)
      .Str(": error: ")
      .Str(step)
      .Emit(g_platform);
}

}  // namespace

int64_t Main(const uint64_t arg) {
  const int64_t pci_count = GetPciInfo(0, nullptr);
  if (pci_count == kSyscallErrorPermission) {
    usb::LogLine line;
    line.Str(usb::kLogPrefix)
        .Str("not started as a driver (init starts usbkbd at boot)")
        .Emit(g_platform);
    return kDriverExitNotDriver;
  }
  if (pci_count <= 0) {
    usb::LogLine line;
    line.Str(usb::kLogPrefix).Str("no xHCI controllers found").Emit(g_platform);
    return (arg == kDriverArgNormal) ? 0 : 1;
  }

  // Phase 1: PCI discovery, PciClaim (Memory Space on, Bus Master untouched),
  // BAR0 MMIO mapping, BIOS-to-OS handoff, halt, and HCRST.
  int discovered_count = 0;
  for (int64_t idx = 0; idx < pci_count && discovered_count < kMaxControllers;
       ++idx) {
    PciDeviceInfo info{};
    if (GetPciInfo(idx, &info) < 0 || !IsXhciDevice(info)) {
      continue;
    }
    const int slot_idx = discovered_count++;
    ControllerSlot& slot = g_slots[slot_idx];
    slot = ControllerSlot{};
    slot.pci_index = static_cast<int>(idx);
    usb::FormatBdf(info.bus, info.device, info.function, slot.label);

    if (PciClaim(idx) != 0) {
      LogBringUpError(slot.label, "claim");
      continue;
    }

    int64_t mapped_size = 0;
    const int64_t mmio_va = MapControllerBar0(info.bars[0], &mapped_size);
    if (IsError(mmio_va)) {
      (void)PciBusMaster(idx, 0);
      LogBringUpError(slot.label, "mmio_map");
      continue;
    }

    g_mmio[slot_idx].Init(
        reinterpret_cast<volatile uint8_t*>(static_cast<uintptr_t>(mmio_va)),
        mapped_size);
    g_controllers[slot_idx].Init(g_platform,        //
                                 g_mmio[slot_idx],  //
                                 slot.label,        //
                                 &g_shared_locks);
    if (g_controllers[slot_idx].TakeOwnershipAndReset() !=
        usb::XhciStatus::kOk) {
      (void)PciBusMaster(idx, 0);
      LogBringUpError(slot.label, "reset");
      continue;
    }

    slot.phase1_ok = true;
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
    PciDeviceInfo info{};
    if (GetPciInfo(slot.pci_index, &info) < 0) {
      continue;
    }
    if (PciBusMaster(slot.pci_index, 1) != 0) {
      LogBringUpError(slot.label, "bus_master");
      continue;
    }
    if (!AllocateControllerIrq(slot.pci_index,      //
                               info,                //
                               &slot.irq_bit_mask,  //
                               &slot.irq_mode)) {
      (void)PciBusMaster(slot.pci_index, 0);
      LogBringUpError(slot.label, "irq_alloc");
      continue;
    }
    if (g_controllers[i].Start(slot.irq_mode) != usb::XhciStatus::kOk) {
      (void)PciBusMaster(slot.pci_index, 0);
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
  if (arg != kDriverArgNormal && active_count != discovered_count) {
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
  // any periodic timer wakeups.
  for (;;) {
    int running_controllers = 0;
    int64_t earliest_deadline = usb::kNoDeadline;
    for (int i = 0; i < discovered_count; ++i) {
      if (!g_slots[i].active ||
          g_controllers[i].state() != usb::XhciState::kRunning) {
        continue;
      }
      ++running_controllers;
      const int64_t d = g_controllers[i].NextDeadline();
      if (d < earliest_deadline) {
        earliest_deadline = d;
      }
    }
    if (running_controllers == 0) {
      return 1;
    }

    int64_t timeout_ticks = -1;
    if (earliest_deadline != usb::kNoDeadline) {
      const int64_t now = GetTicks();
      const int64_t remaining = earliest_deadline - now;
      if (remaining <= 0) {
        timeout_ticks = 0;
      } else if (remaining > kSyscallIrqWaitMaxTicks) {
        timeout_ticks = kSyscallIrqWaitMaxTicks;
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
