#include "task_stack.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include "pmm.h"
#include "spinlock.h"
#include "task.h"

namespace protos {
namespace {

namespace t = ::testing;

constexpr uintptr_t kAreaBase = 0x00007F8000000000ULL;
constexpr uintptr_t kFirstFakeFrame = 0x100000;

// Stack base of slot `slot` of the area at kAreaBase.
static uintptr_t StackOfSlot(const int slot) {
  return kAreaBase + static_cast<uintptr_t>(slot * kTaskStackSlotSize) +
         kPageSize;
}

// Page `page` of the stack at `stack_base`.
static uintptr_t StackPage(const uintptr_t stack_base, const int page) {
  return stack_base + static_cast<uintptr_t>(page * kPageSize);
}

// Models the PMM, the page tables of the area and TLB shootdowns, and fails
// the test if the allocator maps a guard page, maps a page outside the
// reserved range or twice, frees a frame twice, runs a shootdown while it
// holds a spinlock, or maps a page again before a shootdown that started after
// the page was unmapped has completed (another CPU could still use the old
// translation). Thread-safe.
class FakeTaskStackMemory final : public TaskStackMemory {
 public:
  bool ReserveRange(const uintptr_t virt_addr, const int64_t size) override {
    const std::lock_guard<std::mutex> lock(mutex_);
    ++reserve_calls_;
    if (fail_reserve_) {
      return false;
    }
    reserved_base_ = virt_addr;
    reserved_size_ = size;
    return true;
  }

  uintptr_t AllocFrame() override {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (frames_until_failure_ == 0) {
      return 0;
    }
    if (frames_until_failure_ > 0) {
      --frames_until_failure_;
    }
    const uintptr_t frame = next_frame_;
    next_frame_ += kPageSize;
    live_frames_.insert(frame);
    return frame;
  }

  void FreeFrame(const uintptr_t phys_addr) override {
    const std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_THAT(live_frames_.erase(phys_addr), t::Eq(1u))
        << "frame " << phys_addr << " freed but not allocated";
  }

  bool MapPage(const uintptr_t virt_addr, const uintptr_t phys_addr) override {
    const std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_TRUE(virt_addr >= reserved_base_ &&
                virt_addr - reserved_base_ <
                    static_cast<uintptr_t>(reserved_size_))
        << "page " << virt_addr << " mapped outside the reserved range";
    EXPECT_THAT((virt_addr - kAreaBase) % kTaskStackSlotSize, t::Ne(0u))
        << "guard page " << virt_addr << " mapped";
    EXPECT_THAT(live_frames_.count(phys_addr), t::Eq(1u));
    EXPECT_THAT(mappings_.count(virt_addr), t::Eq(0u))
        << "page " << virt_addr << " mapped twice";
    const auto unmapped = unmapped_at_.find(virt_addr);
    if (unmapped != unmapped_at_.end()) {
      EXPECT_THAT(last_completed_shootdown_start_, t::Gt(unmapped->second))
          << "page " << virt_addr << " mapped again before a TLB shootdown";
    }
    if (fail_map_) {
      return false;
    }
    mappings_[virt_addr] = phys_addr;
    return true;
  }

  uintptr_t UnmapPage(const uintptr_t virt_addr) override {
    const std::lock_guard<std::mutex> lock(mutex_);
    const auto it = mappings_.find(virt_addr);
    if (it == mappings_.end()) {
      return 0;
    }
    const uintptr_t frame = it->second;
    mappings_.erase(it);
    unmapped_at_[virt_addr] = ++clock_;
    return frame;
  }

  void ShootdownTlb() override {
    EXPECT_FALSE(HoldsAnyIrqSpinLock());
    int64_t start = 0;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      start = ++clock_;
    }
    // Lets other threads run while the "IPIs" are in flight.
    std::this_thread::yield();
    const std::lock_guard<std::mutex> lock(mutex_);
    last_completed_shootdown_start_ =
        std::max(last_completed_shootdown_start_, start);
    ++shootdowns_;
  }

  void set_fail_reserve(const bool fail) {
    const std::lock_guard<std::mutex> lock(mutex_);
    fail_reserve_ = fail;
  }

  // Makes AllocFrame() fail after `count` more frames (-1: never).
  void set_frames_until_failure(const int count) {
    const std::lock_guard<std::mutex> lock(mutex_);
    frames_until_failure_ = count;
  }

  void set_fail_map(const bool fail) {
    const std::lock_guard<std::mutex> lock(mutex_);
    fail_map_ = fail;
  }

  int reserve_calls() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return reserve_calls_;
  }

  uintptr_t reserved_base() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return reserved_base_;
  }

  int64_t reserved_size() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return reserved_size_;
  }

  int shootdowns() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return shootdowns_;
  }

  int live_frame_count() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(live_frames_.size());
  }

  int mapped_page_count() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(mappings_.size());
  }

  // The frame that `virt_addr` maps to, or 0.
  uintptr_t FrameAt(const uintptr_t virt_addr) const {
    const std::lock_guard<std::mutex> lock(mutex_);
    const auto it = mappings_.find(virt_addr);
    return (it != mappings_.end()) ? it->second : 0;
  }

 private:
  mutable std::mutex mutex_;
  bool fail_reserve_ = false;
  int frames_until_failure_ = -1;
  bool fail_map_ = false;
  int reserve_calls_ = 0;
  uintptr_t reserved_base_ = 0;
  int64_t reserved_size_ = 0;
  uintptr_t next_frame_ = kFirstFakeFrame;
  std::set<uintptr_t> live_frames_;
  std::map<uintptr_t, uintptr_t> mappings_;
  // Orders unmaps and shootdown starts.
  int64_t clock_ = 0;
  // When each page was last unmapped.
  std::map<uintptr_t, int64_t> unmapped_at_;
  int64_t last_completed_shootdown_start_ = 0;
  int shootdowns_ = 0;
};

class TaskStackAllocatorTest : public t::Test {
 protected:
  // Creates `allocator_` with `slot_count` slots at kAreaBase and initializes
  // it.
  void CreateAllocator(const int slot_count) {
    allocator_ = std::make_unique<TaskStackAllocator>(memory_,    //
                                                      kAreaBase,  //
                                                      slot_count);
    ASSERT_TRUE(allocator_->Init());
  }

  const std::shared_ptr<FakeTaskStackMemory> memory_ =
      std::make_shared<FakeTaskStackMemory>();
  std::unique_ptr<TaskStackAllocator> allocator_;
};

TEST_F(TaskStackAllocatorTest, InitReservesThePageTablesOfTheWholeArea) {
  CreateAllocator(8);
  EXPECT_THAT(memory_->reserve_calls(), t::Eq(1));
  EXPECT_THAT(memory_->reserved_base(), t::Eq(kAreaBase));
  EXPECT_THAT(memory_->reserved_size(), t::Eq(8 * kTaskStackSlotSize));

  memory_->set_fail_reserve(true);
  TaskStackAllocator allocator(memory_, kAreaBase, 8);
  EXPECT_FALSE(allocator.Init());
}

TEST_F(TaskStackAllocatorTest, AllocMapsAStackAboveAnUnmappedGuardPage) {
  CreateAllocator(8);
  const uintptr_t first = allocator_->Alloc();
  const uintptr_t second = allocator_->Alloc();
  EXPECT_THAT(first, t::Eq(StackOfSlot(0)));
  EXPECT_THAT(second, t::Eq(StackOfSlot(1)));
  EXPECT_THAT(first % kPageSize, t::Eq(0u));

  std::set<uintptr_t> frames;
  for (const uintptr_t stack : {first, second}) {
    for (int page = 0; page < kTaskStackFrames; ++page) {
      const uintptr_t frame = memory_->FrameAt(StackPage(stack, page));
      EXPECT_THAT(frame, t::Ne(0u));
      frames.insert(frame);
    }
    // The guard page below the stack and the one of the next slot, directly
    // above it, stay unmapped.
    EXPECT_THAT(memory_->FrameAt(stack - kPageSize), t::Eq(0u));
    EXPECT_THAT(memory_->FrameAt(stack + kTaskStackSize), t::Eq(0u));
  }
  EXPECT_THAT(frames.size(), t::Eq(static_cast<size_t>(2 * kTaskStackFrames)));
  EXPECT_THAT(memory_->live_frame_count(), t::Eq(2 * kTaskStackFrames));
  EXPECT_THAT(memory_->shootdowns(), t::Eq(0));
}

TEST_F(TaskStackAllocatorTest,
       FreeReturnsFramesAtOnceButTheSlotOnlyAfterAShootdown) {
  CreateAllocator(4);
  const uintptr_t first = allocator_->Alloc();
  allocator_->Free(first);
  EXPECT_THAT(memory_->live_frame_count(), t::Eq(0));
  EXPECT_THAT(memory_->mapped_page_count(), t::Eq(0));
  EXPECT_THAT(memory_->shootdowns(), t::Eq(0));

  // The freed slot waits for a shootdown, so the next stacks get fresh slots.
  EXPECT_THAT(allocator_->Alloc(), t::Eq(StackOfSlot(1)));
  EXPECT_THAT(allocator_->Alloc(), t::Eq(StackOfSlot(2)));
  EXPECT_THAT(allocator_->Alloc(), t::Eq(StackOfSlot(3)));
  EXPECT_THAT(memory_->shootdowns(), t::Eq(0));

  // Without fresh slots, one shootdown recycles the freed slot.
  EXPECT_THAT(allocator_->Alloc(), t::Eq(first));
  EXPECT_THAT(memory_->shootdowns(), t::Eq(1));

  // Every slot holds a stack now: Alloc() fails without a shootdown.
  EXPECT_THAT(allocator_->Alloc(), t::Eq(0u));
  EXPECT_THAT(memory_->shootdowns(), t::Eq(1));
  EXPECT_THAT(memory_->live_frame_count(), t::Eq(4 * kTaskStackFrames));
}

TEST_F(TaskStackAllocatorTest, RecyclesFreedSlotsWithOneShootdownPerBatch) {
  CreateAllocator(kTaskStackSlotCount);
  constexpr int kRounds = 10;
  uintptr_t highest_stack = 0;
  for (int i = 0; i < kRounds * TaskStackAllocator::kRecycleBatch; ++i) {
    const uintptr_t stack = allocator_->Alloc();
    ASSERT_THAT(stack, t::Ne(0u));
    highest_stack = std::max(highest_stack, stack);
    allocator_->Free(stack);
  }
  // The first batch of stacks takes fresh slots, each later batch recycles
  // the slots of the one before with a single shootdown, so the stacks stay
  // within the first batch of slots.
  EXPECT_THAT(memory_->shootdowns(), t::Eq(kRounds - 1));
  EXPECT_THAT(highest_stack,
              t::Eq(StackOfSlot(TaskStackAllocator::kRecycleBatch - 1)));
  EXPECT_THAT(memory_->live_frame_count(), t::Eq(0));
  EXPECT_THAT(memory_->mapped_page_count(), t::Eq(0));
}

TEST_F(TaskStackAllocatorTest, AllocCleansUpWhenAFrameCantBeAllocated) {
  CreateAllocator(8);
  // The third frame of the stack fails: the two mapped pages are unmapped and
  // their frames freed, and the slot waits for a shootdown.
  memory_->set_frames_until_failure(2);
  EXPECT_THAT(allocator_->Alloc(), t::Eq(0u));
  EXPECT_THAT(memory_->live_frame_count(), t::Eq(0));
  EXPECT_THAT(memory_->mapped_page_count(), t::Eq(0));
  memory_->set_frames_until_failure(-1);
  EXPECT_THAT(allocator_->Alloc(), t::Eq(StackOfSlot(1)));

  // If the first frame fails, nothing was mapped, and the slot is reused at
  // once.
  memory_->set_frames_until_failure(0);
  EXPECT_THAT(allocator_->Alloc(), t::Eq(0u));
  memory_->set_frames_until_failure(-1);
  EXPECT_THAT(allocator_->Alloc(), t::Eq(StackOfSlot(2)));
  EXPECT_THAT(memory_->shootdowns(), t::Eq(0));
  EXPECT_THAT(memory_->live_frame_count(), t::Eq(2 * kTaskStackFrames));
}

TEST_F(TaskStackAllocatorTest, AllocCleansUpWhenAPageCantBeMapped) {
  CreateAllocator(8);
  memory_->set_fail_map(true);
  EXPECT_THAT(allocator_->Alloc(), t::Eq(0u));
  EXPECT_THAT(memory_->live_frame_count(), t::Eq(0));
  memory_->set_fail_map(false);
  EXPECT_THAT(allocator_->Alloc(), t::Eq(StackOfSlot(0)));
}

TEST_F(TaskStackAllocatorTest, FreeAndAllocCheckTheirArguments) {
  CreateAllocator(8);
  const uintptr_t stack = allocator_->Alloc();
  EXPECT_DEATH(allocator_->Free(stack - kPageSize), "Check failed: slot >= 0");
  EXPECT_DEATH(allocator_->Free(stack + 16), "Check failed: slot >= 0");
  EXPECT_DEATH(allocator_->Free(StackOfSlot(8)), "Check failed: slot >= 0");
  // Alloc() may wait for other CPUs, which could be spinning on the lock.
  EXPECT_DEATH(
      {
        IrqSpinLock lock;
        const IrqSpinLockGuard guard(lock);
        (void)allocator_->Alloc();
      },
      "Check failed: !HoldsAnyIrqSpinLock\\(\\)");
}

TEST_F(TaskStackAllocatorTest,
       ConcurrentAllocAndFreeReuseSlotsOnlyAfterShootdowns) {
  // Few slots, so that the threads run out of fresh slots and recycle small
  // batches concurrently.
  constexpr int kSlots = 48;
  constexpr int kThreads = 8;
  constexpr int kStacksPerThread = 4;
  constexpr int kIterations = 2000;
  CreateAllocator(kSlots);

  std::mutex live_mutex;
  std::set<uintptr_t> live_stacks;
  std::vector<std::thread> threads;
  for (int i = 0; i < kThreads; ++i) {
    threads.emplace_back([&, i] {
      std::vector<uintptr_t> stacks;
      uint32_t random = 0x9E3779B9u * static_cast<uint32_t>(i + 1);
      for (int iter = 0; iter < kIterations; ++iter) {
        random = random * 1664525u + 1013904223u;
        const bool alloc = stacks.empty() || (static_cast<int>(stacks.size()) <
                                                  kStacksPerThread &&
                                              (random >> 16) % 2 == 0);
        if (alloc) {
          const uintptr_t stack = allocator_->Alloc();
          ASSERT_THAT(stack, t::Ne(0u));
          {
            const std::lock_guard<std::mutex> lock(live_mutex);
            EXPECT_TRUE(live_stacks.insert(stack).second)
                << "stack " << stack << " handed out twice";
          }
          stacks.push_back(stack);
        } else {
          const size_t index = (random >> 8) % stacks.size();
          const uintptr_t stack = stacks[index];
          stacks.erase(stacks.begin() + static_cast<ptrdiff_t>(index));
          {
            const std::lock_guard<std::mutex> lock(live_mutex);
            live_stacks.erase(stack);
          }
          allocator_->Free(stack);
        }
      }
      for (const uintptr_t stack : stacks) {
        {
          const std::lock_guard<std::mutex> lock(live_mutex);
          live_stacks.erase(stack);
        }
        allocator_->Free(stack);
      }
    });
  }
  for (std::thread& thread : threads) {
    thread.join();
  }

  EXPECT_THAT(memory_->shootdowns(), t::Gt(0));
  EXPECT_THAT(memory_->live_frame_count(), t::Eq(0));
  EXPECT_THAT(memory_->mapped_page_count(), t::Eq(0));
}

}  // namespace
}  // namespace protos
