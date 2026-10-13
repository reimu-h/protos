#ifndef PROTOS_SMP_HARDWARE_H_
#define PROTOS_SMP_HARDWARE_H_

#include <atomic>
#include <cstdint>
#include <memory>

namespace protos {

// Result of the CPUID instruction.
struct CpuidResult {
  uint32_t eax = 0;
  uint32_t ebx = 0;
  uint32_t ecx = 0;
  uint32_t edx = 0;
};

// Bounds of a stack: it grows down from `top` towards `base`.
struct StackBounds {
  uintptr_t base = 0;
  uintptr_t top = 0;
};

// What smp.cpp does to the machine, and nothing more: CPUID and MSRs (x2APIC
// registers are MSRs), xAPIC Local APIC MMIO registers, port I/O (the PIT),
// the time stamp counter and spin-wait loops, physical memory (ACPI tables and
// the guard pages of the AP stacks), the AP startup trampoline, and halting.
// All SMP logic, including AP bring-up with INIT-SIPI-SIPI, IPIs and the AP
// idle loop, runs unchanged on top of either implementation:
// CreateKernelSmpHardware() executes the instructions, and host unit tests use
// FakeSmpHardware (smp_hardware_fake.h), which models the CPUs.
//
// Every method acts on the CPU that calls it, and may be called on several
// CPUs at once.
class SmpHardware {
 public:
  virtual ~SmpHardware() = default;

  virtual CpuidResult Cpuid(uint32_t leaf, uint32_t subleaf) = 0;
  virtual uint64_t ReadMsr(uint32_t msr) = 0;
  virtual void WriteMsr(uint32_t msr, uint64_t value) = 0;

  // Local APIC register `reg_offset` in xAPIC mode, memory-mapped at
  // `lapic_base`.
  virtual uint32_t ReadLapicMmio(uintptr_t lapic_base, int reg_offset) = 0;
  virtual void WriteLapicMmio(uintptr_t lapic_base,  //
                              int reg_offset,        //
                              uint32_t value) = 0;

  virtual uint8_t InByte(uint16_t port) = 0;
  virtual void OutByte(uint16_t port, uint8_t value) = 0;

  // The time stamp counter, which times delays and timeouts. SmpInit()
  // measures its rate against the PIT.
  virtual uint64_t ReadTimestampCounter() = 0;
  // Called in every iteration of a spin-wait loop (`pause`).
  virtual void SpinPause() = 0;

  // Returns a pointer through which the `size` bytes of physical memory at
  // `phys_addr` can be read, or nullptr if they cannot be mapped.
  virtual const void* MapPhysicalMemory(uintptr_t phys_addr, int64_t size) = 0;
  // Maps the 4 KiB Local APIC MMIO page at `phys_addr` uncached. Returns false
  // on failure.
  virtual bool MapLocalApicPage(uintptr_t phys_addr) = 0;
  // Unmaps the 4 KiB page at page-aligned `addr` as the guard page below a
  // stack, and maps it back. Unmapping flushes only the calling CPU's TLB, so
  // smp.cpp does it only before it starts the first AP. Both return false on
  // failure.
  virtual bool UnmapGuardPage(uintptr_t addr) = 0;
  virtual bool RemapGuardPage(uintptr_t addr) = 0;

  // Registers of the calling CPU.
  virtual uintptr_t ReadRsp() = 0;
  virtual uintptr_t ReadCr3() = 0;
  virtual uint16_t ReadCs() = 0;

  // The stack the BSP boots on.
  virtual StackBounds BspStack() = 0;

  // Copies the AP startup trampoline to `kApTrampolinePhysAddr` (smp.h),
  // saving the bytes it overwrites, and puts them back.
  virtual void InstallApTrampoline() = 0;
  virtual void RestoreApTrampoline() = 0;

  // `sti; hlt`: enables interrupts and halts the calling CPU until it has
  // handled an interrupt, then returns true with interrupts enabled. Returns
  // false if the CPU has been powered off instead, which only the host fake
  // does.
  virtual bool HaltUntilInterrupt() = 0;
  // `cli; hlt` for good: does not return.
  virtual void HaltForever() = 0;
};

#if !__STDC_HOSTED__
// Returns the SmpHardware that executes the instructions (smp_hardware.cpp).
std::shared_ptr<SmpHardware> CreateKernelSmpHardware();
#endif

}  // namespace protos

// The AP boot handoff. The BSP fills it in (smp.cpp) before it sends
// INIT-SIPI-SIPI to an AP, one AP at a time; the startup trampoline in boot.S
// (in host tests, FakeSmpHardware) reads it on that AP.
extern "C" {
// Top of the AP's 16 KiB kernel stack.
extern volatile uint64_t g_ap_boot_stack;
// Address of the AP's `CpuLocal`, which the trampoline writes to IA32_GS_BASE
// before the AP runs any C++ code.
extern volatile uint64_t g_ap_boot_cpu_local;
// Logical CPU index (1 .. cpu_count - 1) of the AP, passed to ApKernelEntry().
extern volatile int g_ap_boot_cpu_index;
// Set to 1 by the trampoline as soon as the AP runs in 32-bit protected mode,
// so that the BSP can skip a redundant second SIPI.
extern std::atomic<int> g_ap_boot_started;

// C++ entry point of an AP (smp.cpp), called by the trampoline on the AP's own
// stack with its `CpuLocal` bound. Initializes the AP, reports it online, and
// runs its idle loop. Returns only if the AP is powered off (host fake).
void ApKernelEntry(int cpu_index);
}

#endif  // PROTOS_SMP_HARDWARE_H_
