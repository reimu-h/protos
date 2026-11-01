#ifndef PROTOS_IOAPIC_H_
#define PROTOS_IOAPIC_H_

#include <cstdint>
#include <memory>

#include "smp.h"
#include "syscall_abi.h"

namespace protos {

// Intel 82093AA-compatible I/O APIC indirect register indices (`IOREGSEL` at
// MMIO `+0x00`, `IOWIN` at MMIO `+0x10`) and redirection-entry bit masks.
constexpr uint8_t kIoApicRegId = 0x00;
constexpr uint8_t kIoApicRegVer = 0x01;
constexpr uint8_t kIoApicRegRedirBase = 0x10;
constexpr int kIoApicMaxRedirectionPins = 120;

constexpr uint32_t kIoApicRedirMaskBit = 1u << 16;
constexpr uint32_t kIoApicRedirLevelTriggered = 1u << 15;
constexpr uint32_t kIoApicRedirActiveLow = 1u << 13;

constexpr uint8_t IoApicRedirLowReg(const int pin) {
  return static_cast<uint8_t>(kIoApicRegRedirBase + 2 * pin);
}

constexpr uint8_t IoApicRedirHighReg(const int pin) {
  return static_cast<uint8_t>(kIoApicRegRedirBase + 2 * pin + 1);
}

enum class IoApicStatus {
  kOk = 0,
  kNoDevice,  // no active I/O APIC controller covers the ISA IRQ's GSI
  kInvalid,   // out-of-range IRQ/vector or unaddressable destination APIC ID
};

enum class IsaStatus {
  kOk = 0,
  kBusy,      // already owned by another driver process
  kNoDevice,  // device index out of range or reported absent by firmware
  kNotOwner,  // caller does not own the device
};

// Hardware seam for indirect I/O APIC register access. Callers in `ioapic.cpp`
// serialize all `ReadReg` / `WriteReg` calls under an `IrqSpinLock` so
// `IOREGSEL` (`+0x00`) and `IOWIN` (`+0x10`) accesses never interleave across
// CPUs.
class IoApicHardware {
 public:
  virtual ~IoApicHardware() = default;

  // Maps the 4 KiB MMIO page containing `phys_addr` for controller `index`
  // (`0 <= index < kMaxSupportedIoApics`). Called only during `IoApicInit()`.
  virtual bool MapController(int index, uintptr_t phys_addr) = 0;

  virtual uint32_t ReadReg(int index, uint8_t reg) = 0;
  // Writes `value` to indirect register `reg` of controller `index` and reads
  // `IOWIN` back so posted MMIO writes (such as masking a redirection entry)
  // take effect before returning.
  virtual void WriteReg(int index, uint8_t reg, uint32_t value) = 0;
};

#if !__STDC_HOSTED__
std::shared_ptr<IoApicHardware> CreateKernelIoApicHardware();
#endif

// Initializes I/O APIC routing and resets legacy ISA device claims. Maps each
// controller in `ioapics[0..ioapic_count - 1]`, validates its `IOAPICVER`
// register, and masks all of its redirection entries so no legacy line fires
// until a driver claims and allocates it. If `isa_irq_routes` is null, defaults
// to identity active-high edge-triggered routing for ISA IRQs `0..15`. When
// `has_i8042` is false (e.g. ACPI FADT v3+ clears `IAPC_BOOT_ARCH` bit 1),
// `IsaClaim(kIsaDeviceI8042, ...)` returns `IsaStatus::kNoDevice`.
void IoApicInit(std::shared_ptr<IoApicHardware> hardware,  //
                const IoApicInfo* ioapics,                 //
                int ioapic_count,                          //
                const IsaIrqRoute* isa_irq_routes,         //
                bool has_i8042 = true);
#if !__STDC_HOSTED__
void IoApicInit();
#endif

// Returns true if an initialized I/O APIC controller covers the GSI mapped to
// `isa_irq` (`0 <= isa_irq < kIsaIrqCount`).
bool IoApicHasIsaIrq(int isa_irq);

// Programs the I/O APIC redirection entry for `isa_irq` to deliver fixed
// physical-destination `vector` (`32..254`) to `dest_apic_id` (`0..254`) with
// the polarity and trigger mode from the MADT, unmasks edge-triggered pins
// while keeping level-triggered pins masked until `DeviceIrqWait` arms them,
// and writes whether the pin is level-triggered to `*out_level_triggered` (if
// non-null).
IoApicStatus IoApicProgramIsaIrq(int isa_irq,           //
                                 int vector,            //
                                 uint8_t dest_apic_id,  //
                                 bool* out_level_triggered);

// Masks (`masked == true`) or unmasks (`masked == false`) the I/O APIC
// redirection entry for `isa_irq`, flushing the posted MMIO write before
// returning. Returns false if no active I/O APIC covers `isa_irq`.
bool IoApicSetIsaIrqMasked(int isa_irq, bool masked);

// Exclusive legacy ISA device ownership and static resource table (`i8042` at
// ports `0x60`/`0x64` and ISA IRQ `1`).
IsaStatus IsaClaim(int device, uintptr_t owner);
IsaStatus IsaRelease(int device, uintptr_t owner);
uintptr_t IsaOwner(int device);
int IsaClaimedCount();

int IsaDeviceIrq(int device);
bool IsaDeviceOwnsPort(int device, uint16_t port);
int IsaFindDeviceForPort(uint16_t port);

#if __STDC_HOSTED__
void IoApicResetForTest();
#endif

}  // namespace protos

#endif  // PROTOS_IOAPIC_H_
