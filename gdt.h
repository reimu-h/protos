#ifndef PROTOS_GDT_H_
#define PROTOS_GDT_H_

#include <cstdint>

namespace protos {

// Segment selectors of the kernel GDT. The kernel code and data segments are
// the same as in boot.S's `gdt64`, which a CPU uses until it loads this GDT.
constexpr uint16_t kGdtKernelCodeSelector = 0x08;
constexpr uint16_t kGdtKernelDataSelector = 0x10;
// User (DPL 3) data and 64-bit code segments, with RPL 3, as user code loads
// them. Entry 0x18 stays null: a 32-bit user code segment would go there, so
// that the user segments are where `SYSRET` expects them (STAR[63:48] = 0x18).
constexpr uint16_t kGdtUserDataSelector = 0x20 | 3;
constexpr uint16_t kGdtUserCodeSelector = 0x28 | 3;
// Selector of CPU 0's TSS descriptor. Every CPU up to `kMaxSupportedCpus`
// (smp.h) has a 16-byte TSS descriptor; CPU i's is at
// `kGdtFirstTssSelector + 16 * i`.
constexpr uint16_t kGdtFirstTssSelector = 0x30;

// Interrupt Stack Table (IST) slots (1..7) that the kernel IDT's gates switch
// to (see `IdtInit()`), so that these vectors never run on the interrupted
// stack: a stack overflow surfaces as a double fault, the panic path stops the
// other CPUs with NMIs, and a machine check can arrive on any stack.
constexpr uint8_t kIstDoubleFault = 1;
constexpr uint8_t kIstNmi = 2;
constexpr uint8_t kIstMachineCheck = 3;
// Number of IST stacks per CPU (slots 1 .. kIstStackCount).
constexpr int kIstStackCount = 3;
// Size of each IST stack. Each one sits directly above an unmapped 4 KiB guard
// page.
constexpr int64_t kIstStackFrames = 2;
constexpr int64_t kIstStackSize = kIstStackFrames * 4096;

// 104-byte 64-bit Task State Segment (Intel SDM Vol. 3A, 8.7). In 64-bit mode
// it only holds stack pointers: `rsp[0]`, which the CPU switches to when an
// interrupt or exception arrives in user mode (see `GdtSetKernelStack()`;
// `rsp[1..2]` are unused), and the IST stacks (`ist[i]` for slot i + 1).
struct [[gnu::packed]] TaskStateSegment {
  uint32_t reserved0;
  uint64_t rsp[3];
  uint64_t reserved1;
  uint64_t ist[7];
  uint64_t reserved2;
  uint16_t reserved3;
  // Offset of the I/O permission bitmap; one beyond the TSS limit means there
  // is none.
  uint16_t io_map_base;
};
static_assert(sizeof(TaskStateSegment) == 104);

// 16-byte system-segment descriptor of a 64-bit TSS (Intel SDM Vol. 3A, 8.2.3),
// as the two 8-byte GDT entries it occupies.
struct GdtTssDescriptor {
  uint64_t low;
  uint64_t high;
};

// 10-byte GDTR pseudo-descriptor loaded by `lgdt`.
struct [[gnu::packed]] GdtPointer {
  uint16_t limit;
  uint64_t base;
};
static_assert(sizeof(GdtPointer) == 10);

// Encodes the descriptor of an available (not busy) 64-bit TSS at canonical
// address `base` with byte-granular `limit` (`<= 0xFFFFF`): present, DPL 0.
// Validates the arguments with `DCHECK`.
GdtTssDescriptor GdtEncodeTssDescriptor(uint64_t base, uint32_t limit);

// Returns the selector of CPU `cpu_index`'s TSS descriptor
// (`0 <= cpu_index < kMaxSupportedCpus`, `DCHECK`ed).
uint16_t GdtTssSelector(int cpu_index);

// Sets up CPU `cpu_index`'s TSS, with IST slot i + 1 pointing at
// `ist_stack_tops[i]` and no I/O permission bitmap, writes its descriptor into
// the GDT, and loads the GDT (`lgdt`) and the TSS (`ltr`) on the calling CPU.
// Every CPU calls it for its own index before it loads the kernel IDT. The
// segment registers keep their selectors, which index the same descriptors in
// both GDTs. Validates with `DCHECK` that `0 <= cpu_index < kMaxSupportedCpus`
// and that every IST stack top is non-zero and 16-byte aligned.
void GdtLoadForCpu(int cpu_index,
                   const uintptr_t (&ist_stack_tops)[kIstStackCount]);

// Returns the GDTR value that `GdtLoadForCpu()` loads.
GdtPointer GdtGetPointer();

// Returns CPU `cpu_index`'s TSS (`0 <= cpu_index < kMaxSupportedCpus`,
// `DCHECK`ed).
const TaskStateSegment* GdtGetTss(int cpu_index);

// Sets the calling CPU's `TSS.rsp0` to `rsp0` (non-zero, 16-byte aligned): the
// kernel stack that the CPU switches to when an interrupt or exception arrives
// while it runs user code. A task that runs user code needs it set on every
// CPU that returns it to user mode (user_mode.h). Interrupts must be disabled
// (`DCHECK`ed), so that the calling CPU can't change.
void GdtSetKernelStack(uintptr_t rsp0);

#if !__STDC_HOSTED__
// Sets up CPU 0 (the BSP) before `IdtInit()`, whose gates need its TSS: unmaps
// the guard pages below the boot stack (boot.S) and below CPU 0's IST stacks,
// which are static because the PMM does not exist yet, then calls
// `GdtLoadForCpu(0, ...)`. Panics via `CHECK` if a guard page cannot be
// unmapped.
void GdtInit();
#endif

#if __STDC_HOSTED__
// Host unit-test helpers: clearing the TSSes and their descriptors, and
// counting loads. Host builds have no `lgdt`/`ltr`; a load sets a per-thread
// task register instead.
void GdtResetForTest();
int GdtGetHostLoadCountForTest();
// The selector that the calling thread last loaded into its task register, or
// 0 if it has loaded none.
uint16_t GdtGetHostTaskRegisterForTest();
#endif

}  // namespace protos

#endif  // PROTOS_GDT_H_
