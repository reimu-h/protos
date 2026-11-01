#include "ioapic.h"

#include <cstdint>
#include <memory>
#include <utility>

#include "check.h"
#include "idt.h"
#include "pci.h"
#include "spinlock.h"
#if !__STDC_HOSTED__
#include "paging.h"
#include "pmm.h"
#include "uart.h"
#endif

namespace protos {
namespace {

constexpr uint16_t kI8042DataPort = 0x0060;
constexpr uint16_t kI8042CommandStatusPort = 0x0064;
constexpr int kI8042IsaIrq = 1;

struct ActiveIoApic {
  int hw_index = -1;
  uint8_t ioapic_id = 0;
  uintptr_t phys_addr = 0;
  uint32_t gsi_base = 0;
  int max_redir = -1;
  bool valid = false;
};

struct ResolvedPin {
  int controller_slot = -1;
  int pin = -1;
  IsaIrqRoute route = {};
};

IrqSpinLock g_ioapic_lock;
std::shared_ptr<IoApicHardware> g_hardware;
ActiveIoApic g_controllers[kMaxSupportedIoApics] = {};
int g_controller_count = 0;
IsaIrqRoute g_isa_routes[kIsaIrqCount] = {};

IrqSpinLock g_isa_claim_lock;
uintptr_t g_isa_owners[kIsaDeviceCount] = {};
bool g_has_i8042 = true;

static void InitDefaultIsaRoutesLocked() {
  for (int irq = 0; irq < kIsaIrqCount; ++irq) {
    g_isa_routes[irq] = {
        .gsi = static_cast<uint32_t>(irq),
        .active_low = false,
        .level_triggered = false,
    };
  }
}

static ResolvedPin ResolveIsaIrqLocked(const int isa_irq) {
  if (isa_irq < 0 || isa_irq >= kIsaIrqCount || g_hardware == nullptr) {
    return ResolvedPin{};
  }
  const IsaIrqRoute route = g_isa_routes[isa_irq];
  for (int i = 0; i < g_controller_count; ++i) {
    const ActiveIoApic& ctrl = g_controllers[i];
    if (!ctrl.valid || route.gsi < ctrl.gsi_base) {
      continue;
    }
    const uint32_t offset = route.gsi - ctrl.gsi_base;
    if (offset <= static_cast<uint32_t>(ctrl.max_redir)) {
      return ResolvedPin{
          .controller_slot = i,
          .pin = static_cast<int>(offset),
          .route = route,
      };
    }
  }
  return ResolvedPin{};
}

}  // namespace

void IoApicInit(std::shared_ptr<IoApicHardware> hardware,  //
                const IoApicInfo* const ioapics,           //
                const int ioapic_count,                    //
                const IsaIrqRoute* const isa_irq_routes,   //
                const bool has_i8042) {
  CHECK(hardware != nullptr);
  CHECK(ioapic_count >= 0);
  CHECK(ioapic_count == 0 || ioapics != nullptr);

  const int clamped_count = (ioapic_count > kMaxSupportedIoApics)
                                ? kMaxSupportedIoApics
                                : ioapic_count;
  bool mapped[kMaxSupportedIoApics] = {};
  for (int i = 0; i < clamped_count; ++i) {
    mapped[i] = hardware->MapController(i, ioapics[i].phys_addr);
  }

  {
    const IrqSpinLockGuard guard(g_ioapic_lock);
    g_hardware = std::move(hardware);
    g_controller_count = 0;
    for (ActiveIoApic& ctrl : g_controllers) {
      ctrl = ActiveIoApic{};
    }
    if (isa_irq_routes != nullptr) {
      for (int irq = 0; irq < kIsaIrqCount; ++irq) {
        g_isa_routes[irq] = isa_irq_routes[irq];
      }
    } else {
      InitDefaultIsaRoutesLocked();
    }

    for (int i = 0; i < clamped_count; ++i) {
      if (!mapped[i]) {
        continue;
      }
      const uint32_t ver = g_hardware->ReadReg(i, kIoApicRegVer);
      if (ver == 0xFFFFFFFFu) {
        continue;
      }
      const int max_redir = static_cast<int>((ver >> 16) & 0xFFu);
      if (max_redir < 0 || max_redir >= kIoApicMaxRedirectionPins) {
        continue;
      }
      for (int pin = 0; pin <= max_redir; ++pin) {
        const uint8_t low_reg = IoApicRedirLowReg(pin);
        const uint32_t low = g_hardware->ReadReg(i, low_reg);
        g_hardware->WriteReg(i, low_reg, low | kIoApicRedirMaskBit);
      }
      g_controllers[g_controller_count++] = ActiveIoApic{
          .hw_index = i,
          .ioapic_id = ioapics[i].ioapic_id,
          .phys_addr = ioapics[i].phys_addr,
          .gsi_base = ioapics[i].gsi_base,
          .max_redir = max_redir,
          .valid = true,
      };
    }
  }

  {
    const IrqSpinLockGuard guard(g_isa_claim_lock);
    for (uintptr_t& owner : g_isa_owners) {
      owner = kPciNoOwner;
    }
    g_has_i8042 = has_i8042;
  }
}

bool IoApicHasIsaIrq(const int isa_irq) {
  const IrqSpinLockGuard guard(g_ioapic_lock);
  return ResolveIsaIrqLocked(isa_irq).controller_slot >= 0;
}

IoApicStatus IoApicProgramIsaIrq(const int isa_irq,           //
                                 const int vector,            //
                                 const uint8_t dest_apic_id,  //
                                 bool* const out_level_triggered) {
  if (isa_irq < 0 || isa_irq >= kIsaIrqCount || vector < kCpuExceptionCount ||
      vector >= kIdtEntryCount || vector == kVectorSpurious ||
      dest_apic_id == kInvalidXapicId) {
    return IoApicStatus::kInvalid;
  }

  const IrqSpinLockGuard guard(g_ioapic_lock);
  const ResolvedPin resolved = ResolveIsaIrqLocked(isa_irq);
  if (resolved.controller_slot < 0) {
    return IoApicStatus::kNoDevice;
  }
  const int hw_index = g_controllers[resolved.controller_slot].hw_index;
  const uint8_t low_reg = IoApicRedirLowReg(resolved.pin);
  const uint8_t high_reg = IoApicRedirHighReg(resolved.pin);

  // Mask the pin before updating its destination or trigger/polarity bits so an
  // in-flight signal never fires with a half-programmed redirection entry.
  const uint32_t old_low = g_hardware->ReadReg(hw_index, low_reg);
  g_hardware->WriteReg(hw_index, low_reg, old_low | kIoApicRedirMaskBit);

  const uint32_t high = static_cast<uint32_t>(dest_apic_id) << 24;
  g_hardware->WriteReg(hw_index, high_reg, high);

  uint32_t low = static_cast<uint32_t>(vector & 0xFF);
  if (resolved.route.active_low) {
    low |= kIoApicRedirActiveLow;
  }
  if (resolved.route.level_triggered) {
    // Keep level-triggered lines masked until `DeviceIrqWait` arms the slot so
    // an already-asserted line cannot fire before `slot.level_isa_irq` is set.
    low |= kIoApicRedirLevelTriggered | kIoApicRedirMaskBit;
  }
  g_hardware->WriteReg(hw_index, low_reg, low);

  if (out_level_triggered != nullptr) {
    *out_level_triggered = resolved.route.level_triggered;
  }
  return IoApicStatus::kOk;
}

bool IoApicSetIsaIrqMasked(const int isa_irq, const bool masked) {
  const IrqSpinLockGuard guard(g_ioapic_lock);
  const ResolvedPin resolved = ResolveIsaIrqLocked(isa_irq);
  if (resolved.controller_slot < 0) {
    return false;
  }
  const int hw_index = g_controllers[resolved.controller_slot].hw_index;
  const uint8_t low_reg = IoApicRedirLowReg(resolved.pin);
  const uint32_t old_low = g_hardware->ReadReg(hw_index, low_reg);
  const uint32_t new_low = masked ? (old_low | kIoApicRedirMaskBit)
                                  : (old_low & ~kIoApicRedirMaskBit);
  // Always write when masking so `WriteReg`'s read-back flushes any prior
  // posted MMIO writes before the caller frees IDT vectors on teardown.
  if (masked || new_low != old_low) {
    g_hardware->WriteReg(hw_index, low_reg, new_low);
  }
  return true;
}

IsaStatus IsaClaim(const int device, const uintptr_t owner) {
  CHECK(owner != kPciNoOwner);
  if (device < 0 || device >= kIsaDeviceCount) {
    return IsaStatus::kNoDevice;
  }
  const IrqSpinLockGuard guard(g_isa_claim_lock);
  if (device == kIsaDeviceI8042 && !g_has_i8042) {
    return IsaStatus::kNoDevice;
  }
  if (g_isa_owners[device] == owner) {
    return IsaStatus::kOk;
  }
  if (g_isa_owners[device] != kPciNoOwner) {
    return IsaStatus::kBusy;
  }
  g_isa_owners[device] = owner;
  return IsaStatus::kOk;
}

IsaStatus IsaRelease(const int device, const uintptr_t owner) {
  CHECK(owner != kPciNoOwner);
  if (device < 0 || device >= kIsaDeviceCount) {
    return IsaStatus::kNoDevice;
  }
  const IrqSpinLockGuard guard(g_isa_claim_lock);
  if (g_isa_owners[device] != owner) {
    return IsaStatus::kNotOwner;
  }
  g_isa_owners[device] = kPciNoOwner;
  return IsaStatus::kOk;
}

uintptr_t IsaOwner(const int device) {
  if (device < 0 || device >= kIsaDeviceCount) {
    return kPciNoOwner;
  }
  const IrqSpinLockGuard guard(g_isa_claim_lock);
  return g_isa_owners[device];
}

int IsaClaimedCount() {
  const IrqSpinLockGuard guard(g_isa_claim_lock);
  int count = 0;
  for (const uintptr_t owner : g_isa_owners) {
    if (owner != kPciNoOwner) {
      ++count;
    }
  }
  return count;
}

int IsaDeviceIrq(const int device) {
  if (device == kIsaDeviceI8042) {
    return kI8042IsaIrq;
  }
  return -1;
}

bool IsaDeviceOwnsPort(const int device, const uint16_t port) {
  return IsaFindDeviceForPort(port) == device && device >= 0;
}

int IsaFindDeviceForPort(const uint16_t port) {
  if (port == kI8042DataPort || port == kI8042CommandStatusPort) {
    return kIsaDeviceI8042;
  }
  return -1;
}

#if __STDC_HOSTED__
void IoApicResetForTest() {
  {
    const IrqSpinLockGuard guard(g_ioapic_lock);
    g_hardware.reset();
    g_controller_count = 0;
    for (ActiveIoApic& ctrl : g_controllers) {
      ctrl = ActiveIoApic{};
    }
    InitDefaultIsaRoutesLocked();
  }
  {
    const IrqSpinLockGuard guard(g_isa_claim_lock);
    for (uintptr_t& owner : g_isa_owners) {
      owner = kPciNoOwner;
    }
    g_has_i8042 = true;
  }
}
#endif

#if !__STDC_HOSTED__
namespace {

// Reserved kernel virtual window (`0x00007D0000100000`, 8 pages = 32 KiB) used
// to map I/O APIC MMIO pages uncacheable once at boot in `IoApicInit()`,
// sharing the same 2 MiB PT region as `device_irq.cpp`'s MSI-X window so
// runtime ISA IRQ programming and teardown never touch page tables or PMM.
constexpr uintptr_t kIoApicMmioWindowBase = 0x00007D0000100000ULL;

class KernelIoApicHardware final : public IoApicHardware {
 public:
  KernelIoApicHardware() {
    CHECK(PagingReserveKernelRange(
        kIoApicMmioWindowBase,
        static_cast<int64_t>(kMaxSupportedIoApics) * kPageSize));
  }

  bool MapController(const int index, const uintptr_t phys_addr) override {
    if (index < 0 || index >= kMaxSupportedIoApics || phys_addr == 0) {
      return false;
    }
    const uintptr_t page_mask = static_cast<uintptr_t>(kPageSize) - 1;
    const uintptr_t page_base = phys_addr & ~page_mask;
    const uintptr_t page_offset = phys_addr & page_mask;
    if (page_base == 0 || (phys_addr & 0x3u) != 0 ||
        page_offset + 0x20 > static_cast<uintptr_t>(kPageSize)) {
      return false;
    }
    const uintptr_t virt_page =
        kIoApicMmioWindowBase + static_cast<uintptr_t>(index) * kPageSize;
    if (!PagingMapKernelMmioPage(virt_page, page_base)) {
      return false;
    }
    regs_[index] =
        reinterpret_cast<volatile uint32_t*>(virt_page + page_offset);
    return true;
  }

  uint32_t ReadReg(const int index, const uint8_t reg) override {
    DCHECK(index >= 0 && index < kMaxSupportedIoApics);
    volatile uint32_t* const base = regs_[index];
    DCHECK(base != nullptr);
    base[0] = reg;
    return base[4];
  }

  void WriteReg(const int index,    //
                const uint8_t reg,  //
                const uint32_t value) override {
    DCHECK(index >= 0 && index < kMaxSupportedIoApics);
    volatile uint32_t* const base = regs_[index];
    DCHECK(base != nullptr);
    base[0] = reg;
    base[4] = value;
    (void)base[4];
  }

 private:
  volatile uint32_t* regs_[kMaxSupportedIoApics] = {};
};

}  // namespace

std::shared_ptr<IoApicHardware> CreateKernelIoApicHardware() {
  return std::make_shared<KernelIoApicHardware>();
}

void IoApicInit() {
  IoApicInit(CreateKernelIoApicHardware(),  //
             SmpIoApicInfos(),              //
             SmpIoApicCount(),              //
             SmpIsaIrqRoutes(),             //
             SmpHas8042());
  UartWrite("[IOAPIC] Controllers discovered in MADT: ");
  UartWriteDec(static_cast<uint64_t>(SmpIoApicCount()));
  UartWrite(", IRQ1 routed: ");
  UartWrite(IoApicHasIsaIrq(kI8042IsaIrq) ? "yes\n" : "no\n");
}
#endif

}  // namespace protos
