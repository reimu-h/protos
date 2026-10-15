#include "task_stack.h"

#include <cstdint>
#include <memory>
#include <utility>

#include "check.h"
#include "pmm.h"
#include "spinlock.h"
#include "task.h"
#if !__STDC_HOSTED__
#include "paging.h"
#include "smp.h"
#endif

namespace protos {

TaskStackAllocator::TaskStackAllocator(std::shared_ptr<TaskStackMemory> memory,
                                       const uintptr_t area_base,
                                       const int slot_count)
    : memory_(std::move(memory)),
      area_base_(area_base),
      slot_count_(slot_count),
      next_slot_(new int[slot_count]),
      freed_at_shootdown_(new int64_t[slot_count]) {
  CHECK(memory_ != nullptr);
  CHECK(slot_count_ > 0);
  CHECK((area_base_ & (kPageSize - 1)) == 0);
  CHECK(next_slot_ != nullptr && freed_at_shootdown_ != nullptr);
}

bool TaskStackAllocator::Init() {
  return memory_->ReserveRange(area_base_, slot_count_ * kTaskStackSlotSize);
}

uintptr_t TaskStackAllocator::Alloc() {
  DCHECK(!HoldsAnyIrqSpinLock());
  const int slot = TakeSlot();
  if (slot < 0) {
    return 0;
  }
  const uintptr_t stack_base = StackBaseOfSlot(slot);
  for (int i = 0; i < kTaskStackFrames; ++i) {
    const uintptr_t page = stack_base + static_cast<uintptr_t>(i * kPageSize);
    const uintptr_t frame = memory_->AllocFrame();
    if (frame == 0 || !memory_->MapPage(page, frame)) {
      if (frame != 0) {
        memory_->FreeFrame(frame);
      }
      ReleaseSlot(slot, i);
      return 0;
    }
  }
  return stack_base;
}

void TaskStackAllocator::Free(const uintptr_t stack_base) {
  const int slot = SlotOfStack(stack_base);
  CHECK(slot >= 0);
  ReleaseSlot(slot, kTaskStackFrames);
}

int TaskStackAllocator::TakeSlot() {
  for (;;) {
    int64_t shootdown = 0;
    {
      const IrqSpinLockGuard guard(lock_);
      RecycleFlushedSlotsLocked();
      if (free_head_ >= 0) {
        const int slot = free_head_;
        free_head_ = next_slot_[slot];
        return slot;
      }
      if (next_fresh_slot_ < slot_count_ && pending_count_ < kRecycleBatch) {
        return next_fresh_slot_++;
      }
      if (pending_count_ == 0) {
        // No slot is free, pending or fresh: every slot holds a stack.
        return -1;
      }
      shootdown = ++shootdowns_started_;
    }
    // Every slot that is pending now was unmapped before this shootdown
    // started, so the shootdown flushes its stale translations. Another CPU
    // may take the recycled slots first; then this loops.
    memory_->ShootdownTlb();
    const IrqSpinLockGuard guard(lock_);
    if (shootdown > shootdowns_completed_) {
      shootdowns_completed_ = shootdown;
    }
  }
}

void TaskStackAllocator::ReleaseSlot(const int slot, const int mapped_pages) {
  const uintptr_t stack_base = StackBaseOfSlot(slot);
  for (int i = 0; i < mapped_pages; ++i) {
    const uintptr_t page = stack_base + static_cast<uintptr_t>(i * kPageSize);
    const uintptr_t frame = memory_->UnmapPage(page);
    CHECK(frame != 0);
    memory_->FreeFrame(frame);
  }
  const IrqSpinLockGuard guard(lock_);
  if (mapped_pages == 0) {
    // Nothing was mapped since the slot was last flushed.
    next_slot_[slot] = free_head_;
    free_head_ = slot;
    return;
  }
  freed_at_shootdown_[slot] = shootdowns_started_;
  next_slot_[slot] = -1;
  if (pending_tail_ < 0) {
    pending_head_ = slot;
  } else {
    next_slot_[pending_tail_] = slot;
  }
  pending_tail_ = slot;
  ++pending_count_;
}

void TaskStackAllocator::RecycleFlushedSlotsLocked() {
  // Slots are pending in the order in which they were freed, so the ones that
  // a completed shootdown has flushed are at the head.
  while (pending_head_ >= 0 &&
         freed_at_shootdown_[pending_head_] < shootdowns_completed_) {
    const int slot = pending_head_;
    pending_head_ = next_slot_[slot];
    if (pending_head_ < 0) {
      pending_tail_ = -1;
    }
    --pending_count_;
    next_slot_[slot] = free_head_;
    free_head_ = slot;
  }
}

uintptr_t TaskStackAllocator::StackBaseOfSlot(const int slot) const {
  DCHECK(slot >= 0 && slot < slot_count_);
  return area_base_ + static_cast<uintptr_t>(slot * kTaskStackSlotSize) +
         kPageSize;
}

int TaskStackAllocator::SlotOfStack(const uintptr_t stack_base) const {
  if (stack_base < area_base_ + kPageSize) {
    return -1;
  }
  const uintptr_t offset = stack_base - area_base_ - kPageSize;
  const uintptr_t slot_size = static_cast<uintptr_t>(kTaskStackSlotSize);
  if (offset % slot_size != 0 ||
      offset / slot_size >= static_cast<uintptr_t>(slot_count_)) {
    return -1;
  }
  return static_cast<int>(offset / slot_size);
}

#if !__STDC_HOSTED__
namespace {

// The task stack area lies in the kernel virtual ranges (paging.h).
static_assert(kTaskStackAreaBase >= kKernelVirtualBase);
static_assert(kKernelVirtualEnd - kTaskStackAreaBase >=
              static_cast<uintptr_t>(kTaskStackSlotCount * kTaskStackSlotSize));

class KernelTaskStackMemory final : public TaskStackMemory {
 public:
  bool ReserveRange(const uintptr_t virt_addr, const int64_t size) override {
    return PagingReserveKernelRange(virt_addr, size);
  }

  uintptr_t AllocFrame() override { return PmmAllocFrame(); }

  void FreeFrame(const uintptr_t phys_addr) override {
    PmmFreeFrame(phys_addr);
  }

  bool MapPage(const uintptr_t virt_addr, const uintptr_t phys_addr) override {
    return PagingMapKernelPage(virt_addr, phys_addr);
  }

  uintptr_t UnmapPage(const uintptr_t virt_addr) override {
    return PagingUnmapKernelPage(virt_addr);
  }

  void ShootdownTlb() override { SmpTlbShootdown(); }
};

// The allocator that TaskStackInit() creates. The kernel never destroys it.
// Other CPUs use it only after TaskInit() has published the scheduler (with
// release semantics), which happens after TaskStackInit() returns.
TaskStackAllocator* g_task_stacks = nullptr;

}  // namespace

void TaskStackInit() {
  CHECK(g_task_stacks == nullptr);
  TaskStackAllocator* const stacks =
      new TaskStackAllocator(std::make_shared<KernelTaskStackMemory>(),  //
                             kTaskStackAreaBase,                         //
                             kTaskStackSlotCount);
  CHECK(stacks != nullptr);
  CHECK(stacks->Init());
  g_task_stacks = stacks;
}

uintptr_t TaskStackAlloc() {
  DCHECK(g_task_stacks != nullptr);
  return g_task_stacks->Alloc();
}

void TaskStackFree(const uintptr_t stack_base) {
  DCHECK(g_task_stacks != nullptr);
  g_task_stacks->Free(stack_base);
}
#endif

}  // namespace protos
