#include "gdt.h"

#include <atomic>
#include <cstdint>

#include "check.h"
#include "idt.h"
#include "smp.h"

#if !__STDC_HOSTED__
#include "paging.h"
#include "pmm.h"

extern "C" {
// boot.S: the guard page directly below the boot stack.
extern const uint8_t stack_guard[];
}
#endif

namespace protos {
namespace {

static_assert(kGdtKernelCodeSelector == kIdtKernelCodeSelector);

// The code and data descriptors of boot.S's `gdt64`, with their Accessed bits
// preset so that segment loads never write to the GDT:
// 64-bit kernel code: P=1, DPL=0, S=1, execute/read, L=1, D=0, G=1.
constexpr uint64_t kKernelCodeDescriptor = 0x00AF9B000000FFFFULL;
// Kernel data: P=1, DPL=0, S=1, read/write, D/B=1, G=1.
constexpr uint64_t kKernelDataDescriptor = 0x00CF93000000FFFFULL;
// The same for user code and data, with DPL=3.
constexpr uint64_t kUserCodeDescriptor = 0x00AFFB000000FFFFULL;
constexpr uint64_t kUserDataDescriptor = 0x00CFF3000000FFFFULL;

// Type (bits 40..43) of an available 64-bit TSS descriptor. `ltr` changes it
// to 0xB (busy).
constexpr uint64_t kTssTypeAvailable = 0x9;
constexpr uint64_t kDescriptorPresent = 1ULL << 47;
constexpr uint32_t kMaxDescriptorLimit = 0xFFFFF;

// GDT entries: null, kernel code, kernel data, null (see
// `kGdtUserDataSelector`), user data, user code, then two per TSS descriptor.
constexpr int kFirstTssEntry = kGdtFirstTssSelector / 8;
constexpr int kGdtEntryCount = kFirstTssEntry + 2 * kMaxSupportedCpus;
static_assert(kGdtKernelCodeSelector / 8 == 1);
static_assert(kGdtKernelDataSelector / 8 == 2);
static_assert(kGdtUserDataSelector / 8 == 4);
static_assert(kGdtUserCodeSelector / 8 == 5);
static_assert(kFirstTssEntry == 6);

// Writable, because `ltr` marks the descriptor it loads busy.
alignas(16) uint64_t g_gdt[kGdtEntryCount] = {0,                      //
                                              kKernelCodeDescriptor,  //
                                              kKernelDataDescriptor,  //
                                              0,                      //
                                              kUserDataDescriptor,    //
                                              kUserCodeDescriptor};
alignas(16) TaskStateSegment g_tss[kMaxSupportedCpus] = {};

#if __STDC_HOSTED__
std::atomic<int> g_host_load_count{0};
thread_local uint16_t t_host_task_register = 0;
#endif

static bool IsCanonicalAddress(const uint64_t addr) {
  const uint64_t sign_ext =
      static_cast<uint64_t>(static_cast<int64_t>(addr) >> 47);
  return sign_ext == 0 || sign_ext == ~0ULL;
}

}  // namespace

GdtTssDescriptor GdtEncodeTssDescriptor(const uint64_t base,
                                        const uint32_t limit) {
  DCHECK(IsCanonicalAddress(base));
  DCHECK(limit <= kMaxDescriptorLimit);
  GdtTssDescriptor desc = {};
  desc.low = (static_cast<uint64_t>(limit) & 0xFFFFu) |
             ((base & 0xFFFFFFu) << 16) | (kTssTypeAvailable << 40) |
             kDescriptorPresent |
             ((static_cast<uint64_t>(limit >> 16) & 0xFu) << 48) |
             (((base >> 24) & 0xFFu) << 56);
  desc.high = base >> 32;
  return desc;
}

uint16_t GdtTssSelector(const int cpu_index) {
  DCHECK(cpu_index >= 0 && cpu_index < kMaxSupportedCpus);
  return static_cast<uint16_t>(kGdtFirstTssSelector + 16 * cpu_index);
}

void GdtLoadForCpu(const int cpu_index,
                   const uintptr_t (&ist_stack_tops)[kIstStackCount]) {
  DCHECK(cpu_index >= 0 && cpu_index < kMaxSupportedCpus);
  TaskStateSegment& tss = g_tss[cpu_index];
  for (int i = 0; i < kIstStackCount; ++i) {
    DCHECK(ist_stack_tops[i] != 0 && (ist_stack_tops[i] & 0xFu) == 0);
    tss.ist[i] = ist_stack_tops[i];
  }
  tss.io_map_base = static_cast<uint16_t>(sizeof(TaskStateSegment));

  // Written on every load, available: `ltr` faults on a busy descriptor.
  const GdtTssDescriptor desc = GdtEncodeTssDescriptor(
      reinterpret_cast<uintptr_t>(&tss), sizeof(TaskStateSegment) - 1);
  const int entry = kFirstTssEntry + 2 * cpu_index;
  g_gdt[entry] = desc.low;
  g_gdt[entry + 1] = desc.high;

  const uint16_t selector = GdtTssSelector(cpu_index);
#if !__STDC_HOSTED__
  const GdtPointer gdtr = GdtGetPointer();
  asm volatile("lgdt [%0]" : : "r"(&gdtr) : "memory");
  asm volatile("ltr %0" : : "r"(selector) : "memory");
#else
  t_host_task_register = selector;
  g_host_load_count.fetch_add(1, std::memory_order_relaxed);
#endif
}

GdtPointer GdtGetPointer() {
  return GdtPointer{static_cast<uint16_t>(sizeof(g_gdt) - 1),
                    reinterpret_cast<uint64_t>(&g_gdt[0])};
}

const TaskStateSegment* GdtGetTss(const int cpu_index) {
  DCHECK(cpu_index >= 0 && cpu_index < kMaxSupportedCpus);
  return &g_tss[cpu_index];
}

void GdtSetKernelStack(const uintptr_t rsp0) {
  DCHECK(!AreInterruptsEnabled());
  DCHECK(rsp0 != 0 && (rsp0 & 0xFu) == 0);
  g_tss[CurrentCpuIdIrqsOff()].rsp[0] = rsp0;
}

#if !__STDC_HOSTED__
namespace {

// One of CPU 0's IST stacks, directly above its guard page.
struct alignas(kPageSize) GuardedIstStack {
  uint8_t guard_page[kPageSize];
  uint8_t stack[kIstStackSize];
};

GuardedIstStack g_bsp_ist_stacks[kIstStackCount];

}  // namespace

void GdtInit() {
  CHECK(PagingUnmapGuardPage(reinterpret_cast<uintptr_t>(stack_guard)));
  uintptr_t ist_stack_tops[kIstStackCount];
  for (int i = 0; i < kIstStackCount; ++i) {
    GuardedIstStack& ist_stack = g_bsp_ist_stacks[i];
    const uintptr_t guard_page =
        reinterpret_cast<uintptr_t>(ist_stack.guard_page);
    CHECK(PagingUnmapGuardPage(guard_page));
    ist_stack_tops[i] =
        reinterpret_cast<uintptr_t>(ist_stack.stack) + kIstStackSize;
  }
  GdtLoadForCpu(0, ist_stack_tops);
}
#endif

#if __STDC_HOSTED__
void GdtResetForTest() {
  for (int i = kFirstTssEntry; i < kGdtEntryCount; ++i) {
    g_gdt[i] = 0;
  }
  for (TaskStateSegment& tss : g_tss) {
    tss = {};
  }
  g_host_load_count.store(0, std::memory_order_relaxed);
  t_host_task_register = 0;
}

int GdtGetHostLoadCountForTest() {
  return g_host_load_count.load(std::memory_order_relaxed);
}

uint16_t GdtGetHostTaskRegisterForTest() { return t_host_task_register; }
#endif

}  // namespace protos
