#ifndef PROTOS_TASK_STACK_H_
#define PROTOS_TASK_STACK_H_

#include <cstdint>
#include <memory>

#include "pmm.h"
#include "spinlock.h"
#include "task.h"

namespace protos {

// Task stacks with guard pages
// ============================
//
// Every task stack that the scheduler allocates (`kTaskStackSize` bytes,
// task.h) lives in its own slot of a task stack area: a range of kernel
// virtual addresses (paging.h) that the identity map never reaches. A slot is
// a guard page followed by the `kTaskStackFrames` pages of the stack. The
// guard page is never mapped, so a task that runs off the bottom of its stack
// faults at once instead of overwriting the memory below it: with a page
// fault, or with a double fault, which runs on its own IST stack (gdt.h), if
// the CPU can't push the page fault's frame onto the full stack. Either way
// the panic message shows an address in the guard page as `cr2`. The stack
// pages are mapped to PMM frames while the stack is allocated; the slot above
// starts with its own guard page, which also catches a stack underflow.
//
// SMP safety: mapping a page needs no TLB shootdown, because CPUs never cache
// a translation for a page that is not mapped. For the same reason no CPU
// ever caches a translation for a guard page. Unmapping a page does need a
// shootdown: any CPU may still hold the old translation, even one that never
// ran the task, because CPUs also cache translations speculatively. `Free()`
// therefore works like Linux's lazy vunmap: it unmaps the stack and returns
// its frames to the PMM at once, which is safe because nothing accesses a
// freed stack's addresses again, but it keeps the slot from being reused until
// a TLB shootdown has flushed the stale translations. Reusing the slot
// earlier would let a CPU reach the old frames, which may belong to someone
// else by then, through the new stack. `Alloc()` takes a flushed slot if there
// is one, then a fresh (never used) slot while fewer than `kRecycleBatch`
// slots wait for a shootdown, and otherwise runs one shootdown
// (`SmpTlbShootdown()`, smp.h) that recycles every slot freed before it
// started.
//
// Deadlock freedom: `Free()` runs at the end of a context switch with
// interrupts disabled; it takes only short spinlocks (this allocator's, the
// page tables' and the PMM's) and never waits for other CPUs. `Alloc()` runs
// the shootdown without holding a lock, and `SmpTlbShootdown()` handles the
// requests of other CPUs while it waits, so CPUs that allocate stacks at the
// same time, also with interrupts disabled, never wait for each other forever.

// Size of a slot of a task stack area: a guard page and a task stack.
constexpr int64_t kTaskStackSlotSize = kPageSize + kTaskStackSize;

// What `TaskStackAllocator` does to the machine. The kernel implementation
// (task_stack.cpp) calls the PMM, the kernel page tables and
// `SmpTlbShootdown()`; host tests use a fake.
class TaskStackMemory {
 public:
  virtual ~TaskStackMemory() = default;

  // `PagingReserveKernelRange()`.
  virtual bool ReserveRange(uintptr_t virt_addr, int64_t size) = 0;
  // `PmmAllocFrame()` and `PmmFreeFrame()`.
  virtual uintptr_t AllocFrame() = 0;
  virtual void FreeFrame(uintptr_t phys_addr) = 0;
  // `PagingMapKernelPage()` and `PagingUnmapKernelPage()`.
  virtual bool MapPage(uintptr_t virt_addr, uintptr_t phys_addr) = 0;
  virtual uintptr_t UnmapPage(uintptr_t virt_addr) = 0;
  // `SmpTlbShootdown()`.
  virtual void ShootdownTlb() = 0;
};

// Allocates task stacks in the slots of a task stack area (see above). All
// methods are safe to call on several CPUs at once.
class TaskStackAllocator {
 public:
  // Number of freed slots that wait for a TLB shootdown before `Alloc()`
  // recycles them, as long as fresh slots are left.
  static constexpr int kRecycleBatch = 64;

  // Manages the `slot_count` (> 0) slots of `kTaskStackSlotSize` bytes that
  // start at page-aligned `area_base` (in the kernel virtual ranges, paging.h)
  // through `memory`.
  TaskStackAllocator(std::shared_ptr<TaskStackMemory> memory,  //
                     uintptr_t area_base,                      //
                     int slot_count);

  TaskStackAllocator(const TaskStackAllocator&) = delete;
  TaskStackAllocator& operator=(const TaskStackAllocator&) = delete;

  // Reserves the page tables of the whole area, so that `Alloc()` and `Free()`
  // never allocate page tables. Must succeed before the first `Alloc()`.
  // Returns false if the page tables can't be reserved.
  bool Init();

  // Returns the lowest address of a new task stack: `kTaskStackSize` bytes,
  // mapped to newly allocated frames, directly above an unmapped guard page.
  // Returns 0 if every slot holds a stack or a frame can't be allocated or
  // mapped. May run a TLB shootdown, so the caller must not hold an
  // `IrqSpinLock` (`DCHECK`); interrupts may be disabled.
  uintptr_t Alloc();

  // Frees the stack at `stack_base`, which `Alloc()` returned and nobody has
  // freed since: unmaps it and frees its frames, and keeps its slot until a
  // TLB shootdown. Nothing may access the stack afterwards. Never waits for
  // other CPUs, so it may run with interrupts disabled.
  void Free(uintptr_t stack_base);

 private:
  // Returns a slot without stale translations, recycling freed slots with a
  // TLB shootdown if necessary, or -1 if every slot holds a stack.
  int TakeSlot();
  // Unmaps the first `mapped_pages` pages of the stack in `slot`, frees their
  // frames and returns the slot to the free list, or, if any page was mapped,
  // to the pending list until the next TLB shootdown.
  void ReleaseSlot(int slot, int mapped_pages);
  // Moves the pending slots that a completed TLB shootdown has flushed to the
  // free list.
  void RecycleFlushedSlotsLocked();
  uintptr_t StackBaseOfSlot(int slot) const;
  // Returns the slot of the stack at `stack_base`, or -1 if `stack_base` is
  // not the base of a stack in the area.
  int SlotOfStack(uintptr_t stack_base) const;

  const std::shared_ptr<TaskStackMemory> memory_;
  const uintptr_t area_base_;
  const int slot_count_;

  // Guards everything below. Lock order: the page table and PMM locks are
  // never taken while it is held.
  IrqSpinLock lock_;
  // Links of the free and pending lists (slot indices, -1 ends a list).
  const std::unique_ptr<int[]> next_slot_;
  // For each pending slot: `shootdowns_started_` when it was freed. The slot
  // is flushed once a shootdown with a higher number has completed.
  const std::unique_ptr<int64_t[]> freed_at_shootdown_;
  // Slots without stale translations, last freed first.
  int free_head_ = -1;
  // Slots that wait for a TLB shootdown, in the order in which they were
  // freed.
  int pending_head_ = -1;
  int pending_tail_ = -1;
  int pending_count_ = 0;
  // Slots `next_fresh_slot_ .. slot_count_ - 1` have never been used.
  int next_fresh_slot_ = 0;
  // Number of TLB shootdowns started, and the highest number of a completed
  // one.
  int64_t shootdowns_started_ = 0;
  int64_t shootdowns_completed_ = 0;
};

// The kernel's task stack area: `kTaskStackSlotCount` slots from
// `kTaskStackAreaBase` on (80 MiB of address space in the kernel virtual
// ranges, paging.h), so at most 4096 tasks can have a stack at the same time.
// Its page tables take 42 frames.
constexpr uintptr_t kTaskStackAreaBase = 0x00007F0000000000ULL;
constexpr int kTaskStackSlotCount = 4096;

// The kernel's task stacks, which the core scheduler allocates and frees.
// Host tests of the scheduler replace these three functions with fakes
// (scheduler_test_fakes.cpp).
//
// `TaskStackInit()` creates the allocator for the kernel's task stack area and
// reserves its page tables. `TaskInit()` calls it once, on the BSP, after
// `PmmInit()`, `HeapInit()` and `SmpInit()`. CHECK-fails if it is called
// twice or the page tables can't be reserved. `TaskStackAlloc()` and
// `TaskStackFree()` are `TaskStackAllocator::Alloc()` and `Free()` of that
// allocator.
void TaskStackInit();
uintptr_t TaskStackAlloc();
void TaskStackFree(uintptr_t stack_base);

}  // namespace protos

#endif  // PROTOS_TASK_STACK_H_
