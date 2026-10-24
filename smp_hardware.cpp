// Kernel implementation of SmpHardware (smp_hardware.h): executes the
// instructions. Linked into the kernel only; host tests use FakeSmpHardware.

#include "smp_hardware.h"

#include <atomic>
#include <cstdint>
#include <memory>

#include "check.h"
#include "paging.h"
#include "pmm.h"
#include "smp.h"

extern "C" {
// boot.S: the AP startup trampoline, and the stack the BSP boots on.
extern const uint8_t ap_trampoline_start[];
extern const uint8_t ap_trampoline_end[];
extern const uint8_t stack_bottom[];
extern const uint8_t stack_top[];
}

namespace protos {
namespace {

// Room for the bytes that the trampoline overwrites at kApTrampolinePhysAddr.
constexpr int64_t kMaxTrampolineBytes = 512;

static int64_t TrampolineBytes() {
  const int64_t bytes = reinterpret_cast<uintptr_t>(ap_trampoline_end) -
                        reinterpret_cast<uintptr_t>(ap_trampoline_start);
  CHECK(bytes > 0 && bytes <= kMaxTrampolineBytes);
  return bytes;
}

class KernelSmpHardware final : public SmpHardware {
 public:
  CpuidResult Cpuid(const uint32_t leaf, const uint32_t subleaf) override {
    uint32_t a = leaf;
    uint32_t b = 0;
    uint32_t c = subleaf;
    uint32_t d = 0;
    asm volatile("cpuid" : "+a"(a), "=b"(b), "+c"(c), "=d"(d) : : "cc");
    return CpuidResult{a, b, c, d};
  }

  uint64_t ReadMsr(const uint32_t msr) override {
    uint32_t low = 0;
    uint32_t high = 0;
    asm volatile("rdmsr" : "=a"(low), "=d"(high) : "c"(msr));
    return (static_cast<uint64_t>(high) << 32) | low;
  }

  void WriteMsr(const uint32_t msr, const uint64_t value) override {
    const uint32_t low = static_cast<uint32_t>(value & 0xFFFFFFFFu);
    const uint32_t high = static_cast<uint32_t>(value >> 32);
    asm volatile("wrmsr" : : "c"(msr), "a"(low), "d"(high) : "memory");
  }

  uint32_t ReadLapicMmio(const uintptr_t lapic_base,
                         const int reg_offset) override {
    return *reinterpret_cast<const volatile uint32_t*>(lapic_base + reg_offset);
  }

  void WriteLapicMmio(const uintptr_t lapic_base,  //
                      const int reg_offset,        //
                      const uint32_t value) override {
    *reinterpret_cast<volatile uint32_t*>(lapic_base + reg_offset) = value;
  }

  uint8_t InByte(const uint16_t port) override {
    uint8_t value = 0;
    asm volatile("in %0, %1" : "=a"(value) : "Nd"(port) : "memory");
    return value;
  }

  void OutByte(const uint16_t port, const uint8_t value) override {
    asm volatile("out %1, %0" : : "a"(value), "Nd"(port) : "memory");
  }

  uint64_t ReadTimestampCounter() override {
    uint32_t low = 0;
    uint32_t high = 0;
    asm volatile("rdtsc" : "=a"(low), "=d"(high));
    return (static_cast<uint64_t>(high) << 32) | low;
  }

  void SpinPause() override { asm volatile("pause" : : : "memory"); }

  const void* MapPhysicalMemory(const uintptr_t phys_addr,
                                const int64_t size) override {
    if (!PagingEnsureIdentityMappedRange(phys_addr, size)) {
      return nullptr;
    }
    return reinterpret_cast<const void*>(phys_addr);
  }

  bool MapLocalApicPage(const uintptr_t phys_addr) override {
    return PagingEnsureIdentityMappedRange(phys_addr, kPageSize) &&
           PagingMarkPageUncacheable(phys_addr);
  }

  bool UnmapGuardPage(const uintptr_t addr) override {
    // Other CPUs could keep stale translations of the page (see paging.h).
    DCHECK(!aps_started_);
    return PagingUnmapGuardPage(addr);
  }

  bool RemapGuardPage(const uintptr_t addr) override {
    return PagingRemapGuardPage(addr);
  }

  void FlushTlb() override { PagingFlushLocalTlb(); }

  uintptr_t ReadRsp() override {
    uint64_t rsp = 0;
    asm volatile("mov %0, rsp" : "=r"(rsp));
    return rsp;
  }

  uintptr_t ReadCr3() override {
    uint64_t cr3 = 0;
    asm volatile("mov %0, cr3" : "=r"(cr3));
    return cr3;
  }

  uint16_t ReadCs() override {
    uint16_t cs = 0;
    asm volatile("mov %0, cs" : "=r"(cs));
    return cs;
  }

  StackBounds BspStack() override {
    return StackBounds{reinterpret_cast<uintptr_t>(stack_bottom),
                       reinterpret_cast<uintptr_t>(stack_top)};
  }

  void InstallApTrampoline() override {
    aps_started_ = true;
    const int64_t bytes = TrampolineBytes();
    uint8_t* const dst = reinterpret_cast<uint8_t*>(kApTrampolinePhysAddr);
    for (int64_t i = 0; i < bytes; ++i) {
      saved_trampoline_bytes_[i] = dst[i];
      dst[i] = ap_trampoline_start[i];
    }
    std::atomic_thread_fence(std::memory_order_seq_cst);
  }

  void RestoreApTrampoline() override {
    const int64_t bytes = TrampolineBytes();
    uint8_t* const dst = reinterpret_cast<uint8_t*>(kApTrampolinePhysAddr);
    for (int64_t i = 0; i < bytes; ++i) {
      dst[i] = saved_trampoline_bytes_[i];
    }
    std::atomic_thread_fence(std::memory_order_seq_cst);
  }

  bool HaltUntilInterrupt() override {
    asm volatile("sti; hlt" : : : "memory", "cc");
    return true;
  }

  void HaltForever() override {
    for (;;) {
      asm volatile("cli; hlt" : : : "memory");
    }
  }

 private:
  uint8_t saved_trampoline_bytes_[kMaxTrampolineBytes] = {};
  // Set when the trampoline is installed for the first time: from then on, APs
  // may run.
  bool aps_started_ = false;
};

}  // namespace

std::shared_ptr<SmpHardware> CreateKernelSmpHardware() {
  return std::make_shared<KernelSmpHardware>();
}

}  // namespace protos
