#include "spinlock.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <new>
#include <thread>
#include <vector>

namespace protos {
namespace {

namespace t = ::testing;

// `constinit` only compiles if `IrqSpinLock` is constant-initialized, i.e.
// needs no runtime constructor, which kernel globals in `.bss` rely on.
constinit IrqSpinLock g_static_spinlock;

// Every test starts and ends on an unbound thread with (simulated) interrupts
// enabled and no locks held.
class SpinLockTest : public t::Test {
 protected:
  void SetUp() override {
    ResetCpuLocalForTest();
    SetInterruptsEnabledForTest(true);
  }

  void TearDown() override {
    // A failed assertion can leave the thread bound to a `CpuLocal` on the
    // test's stack, or holding locks.
    ResetCpuLocalForTest();
    EXPECT_FALSE(HoldsAnyIrqSpinLock());
    *internal::TopHeldLockSlot() = nullptr;
    SetInterruptsEnabledForTest(true);
    // Reset the ticket counters so the zero-initialization checks in
    // ConstinitInitAndBasicLockUnlock also hold under --gtest_repeat.
    new (&g_static_spinlock) IrqSpinLock();
  }
};

using SpinLockDeathTest = SpinLockTest;

TEST_F(SpinLockTest, ConstinitInitAndBasicLockUnlock) {
  EXPECT_FALSE(g_static_spinlock.IsLocked());
  EXPECT_FALSE(g_static_spinlock.IsLockedByCurrentCpu());
  EXPECT_THAT(g_static_spinlock.NextTicket(), t::Eq(0));
  EXPECT_THAT(g_static_spinlock.ServingTicket(), t::Eq(0));
  ASSERT_TRUE(AreInterruptsEnabled());

  g_static_spinlock.Lock();
  EXPECT_TRUE(g_static_spinlock.IsLocked());
  EXPECT_TRUE(g_static_spinlock.IsLockedByCurrentCpu());
  EXPECT_FALSE(AreInterruptsEnabled());
  EXPECT_THAT(g_static_spinlock.NextTicket(), t::Eq(1));
  EXPECT_THAT(g_static_spinlock.ServingTicket(), t::Eq(0));

  g_static_spinlock.Unlock();
  EXPECT_FALSE(g_static_spinlock.IsLocked());
  EXPECT_FALSE(g_static_spinlock.IsLockedByCurrentCpu());
  EXPECT_TRUE(AreInterruptsEnabled());
  EXPECT_THAT(g_static_spinlock.NextTicket(), t::Eq(1));
  EXPECT_THAT(g_static_spinlock.ServingTicket(), t::Eq(1));
}

TEST_F(SpinLockTest, ScopedGuardSupportsReferenceAndPointer) {
  IrqSpinLock lock;

  {
    const IrqSpinLockGuard guard(lock);
    EXPECT_TRUE(lock.IsLocked());
    EXPECT_TRUE(lock.IsLockedByCurrentCpu());
    EXPECT_FALSE(AreInterruptsEnabled());
  }
  EXPECT_FALSE(lock.IsLocked());
  EXPECT_TRUE(AreInterruptsEnabled());

  {
    const IrqSpinLockGuard guard(&lock);
    EXPECT_TRUE(lock.IsLocked());
    EXPECT_TRUE(lock.IsLockedByCurrentCpu());
    EXPECT_FALSE(AreInterruptsEnabled());
  }
  EXPECT_FALSE(lock.IsLocked());
  EXPECT_TRUE(AreInterruptsEnabled());
}

TEST_F(SpinLockTest, InterruptFlagSaveAndRestoreAcrossSingleAndNestedLocks) {
  IrqSpinLock lock_a;
  IrqSpinLock lock_b;
  IrqSpinLock lock_c;

  // Case 1: Interrupts initially enabled -> disabled during nested critical
  // sections -> restored only when outermost lock is released.
  ASSERT_TRUE(AreInterruptsEnabled());
  {
    const IrqSpinLockGuard guard_a(lock_a);
    EXPECT_FALSE(AreInterruptsEnabled());
    {
      const IrqSpinLockGuard guard_b(lock_b);
      EXPECT_FALSE(AreInterruptsEnabled());
      {
        const IrqSpinLockGuard guard_c(lock_c);
        EXPECT_FALSE(AreInterruptsEnabled());
      }
      EXPECT_FALSE(AreInterruptsEnabled());
    }
    EXPECT_FALSE(AreInterruptsEnabled());
  }
  EXPECT_TRUE(AreInterruptsEnabled());

  // Case 2: Interrupts initially disabled -> remain disabled throughout and
  // after outermost unlock.
  SetInterruptsEnabledForTest(false);
  ASSERT_FALSE(AreInterruptsEnabled());
  {
    const IrqSpinLockGuard guard_a(lock_a);
    EXPECT_FALSE(AreInterruptsEnabled());
    {
      const IrqSpinLockGuard guard_b(lock_b);
      EXPECT_FALSE(AreInterruptsEnabled());
    }
    EXPECT_FALSE(AreInterruptsEnabled());
  }
  EXPECT_FALSE(AreInterruptsEnabled());
}

TEST_F(SpinLockTest, InterruptHelpersSaveDisableAndRestore) {
  static_assert(kRflagsInterruptEnableBit == 0x200);

  ASSERT_TRUE(AreInterruptsEnabled());
  const bool outer_was_enabled = SaveAndDisableInterrupts();
  EXPECT_TRUE(outer_was_enabled);
  EXPECT_FALSE(AreInterruptsEnabled());

  // Saving while disabled reports "disabled", and restoring that value keeps
  // interrupts disabled until the outer restore.
  const bool inner_was_enabled = SaveAndDisableInterrupts();
  EXPECT_FALSE(inner_was_enabled);
  RestoreInterrupts(inner_was_enabled);
  EXPECT_FALSE(AreInterruptsEnabled());
  RestoreInterrupts(outer_was_enabled);
  EXPECT_TRUE(AreInterruptsEnabled());

  {
    const ScopedIrqDisable outer_guard;
    EXPECT_FALSE(AreInterruptsEnabled());
    {
      const ScopedIrqDisable inner_guard;
      EXPECT_FALSE(AreInterruptsEnabled());
    }
    EXPECT_FALSE(AreInterruptsEnabled());
  }
  EXPECT_TRUE(AreInterruptsEnabled());

  SetInterruptsEnabledForTest(false);
  {
    const ScopedIrqDisable guard;
    EXPECT_FALSE(AreInterruptsEnabled());
  }
  EXPECT_FALSE(AreInterruptsEnabled());
}

TEST_F(SpinLockTest, HoldsAnyIrqSpinLockAndIsLockedAcrossThreads) {
  IrqSpinLock lock_a;
  IrqSpinLock lock_b;

  EXPECT_FALSE(HoldsAnyIrqSpinLock());
  {
    const IrqSpinLockGuard guard_a(lock_a);
    EXPECT_TRUE(HoldsAnyIrqSpinLock());
    {
      const IrqSpinLockGuard guard_b(lock_b);
      EXPECT_TRUE(HoldsAnyIrqSpinLock());
    }
    EXPECT_TRUE(HoldsAnyIrqSpinLock());
  }
  EXPECT_FALSE(HoldsAnyIrqSpinLock());

  // A lock held by another thread is locked, but neither held by nor counted
  // for this thread.
  std::atomic<bool> holding{false};
  std::atomic<bool> release{false};
  std::thread holder([&lock_a, &holding, &release]() {
    const IrqSpinLockGuard guard(lock_a);
    holding.store(true, std::memory_order_release);
    while (!release.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
  });
  while (!holding.load(std::memory_order_acquire)) {
    std::this_thread::yield();
  }
  EXPECT_TRUE(lock_a.IsLocked());
  EXPECT_FALSE(lock_a.IsLockedByCurrentCpu());
  EXPECT_FALSE(HoldsAnyIrqSpinLock());
  release.store(true, std::memory_order_release);
  holder.join();
  EXPECT_FALSE(lock_a.IsLocked());
}

TEST_F(SpinLockTest, EnforcesStrictFifoTicketOrdering) {
  IrqSpinLock lock;
  constexpr int kNumWaiters = 6;
  std::vector<int> acquisition_order;
  acquisition_order.reserve(kNumWaiters);

  // Acquire the lock on the main thread (ticket 0) so all worker threads must
  // queue up on consecutive tickets 1..kNumWaiters.
  lock.Lock();
  ASSERT_THAT(lock.NextTicket(), t::Eq(1));
  ASSERT_THAT(lock.ServingTicket(), t::Eq(0));

  std::vector<std::thread> waiters;
  waiters.reserve(kNumWaiters);
  for (int i = 0; i < kNumWaiters; ++i) {
    const int64_t expected_next_ticket = i + 2;
    waiters.emplace_back([&lock, &acquisition_order, i]() {
      const IrqSpinLockGuard guard(lock);
      acquisition_order.push_back(i);
    });
    // Wait until thread `i` has fetched its ticket before spawning `i + 1` so
    // ticket assignment order is strictly 0, 1, ..., kNumWaiters - 1.
    while (lock.NextTicket() < expected_next_ticket) {
      std::this_thread::yield();
    }
  }

  // Release the initial lock; queued waiters must enter and exit in exact
  // ticket order 0, 1, 2, 3, 4, 5.
  lock.Unlock();

  for (std::thread& waiter : waiters) {
    waiter.join();
  }

  EXPECT_THAT(acquisition_order, t::ElementsAre(0, 1, 2, 3, 4, 5));
  EXPECT_THAT(lock.NextTicket(), t::Eq(kNumWaiters + 1));
  EXPECT_THAT(lock.ServingTicket(), t::Eq(kNumWaiters + 1));
}

TEST_F(SpinLockTest, MutualExclusionAndProgressUnderHighContention) {
  IrqSpinLock lock;
  constexpr int kNumThreads = 8;
  constexpr int kIterationsPerThread = 1500;

  std::atomic<bool> start_flag{false};
  std::atomic<int> ready_threads{0};
  std::atomic<int> active_in_critical_section{0};
  int64_t shared_counter = 0;
  int64_t invariant_a = 0;
  int64_t invariant_b = 0;

  std::vector<std::thread> threads;
  threads.reserve(kNumThreads);
  for (int tid = 0; tid < kNumThreads; ++tid) {
    threads.emplace_back([&, tid]() {
      const bool initial_irq = (tid % 2 == 0);
      SetInterruptsEnabledForTest(initial_irq);
      ready_threads.fetch_add(1, std::memory_order_acq_rel);
      while (!start_flag.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }

      for (int iter = 0; iter < kIterationsPerThread; ++iter) {
        {
          const IrqSpinLockGuard guard(lock);
          const int concurrent = active_in_critical_section.fetch_add(
              1, std::memory_order_acq_rel);
          ASSERT_THAT(concurrent, t::Eq(0));
          ASSERT_FALSE(AreInterruptsEnabled());
          ASSERT_TRUE(lock.IsLockedByCurrentCpu());
          ASSERT_THAT(invariant_a + invariant_b, t::Eq(0));

          const int64_t delta = (tid + 1) * 13 + iter;
          invariant_a += delta;
          ++shared_counter;
          invariant_b -= delta;

          active_in_critical_section.fetch_sub(1, std::memory_order_acq_rel);
        }
        ASSERT_THAT(AreInterruptsEnabled(), t::Eq(initial_irq));
      }
      SetInterruptsEnabledForTest(true);
    });
  }

  while (ready_threads.load(std::memory_order_acquire) < kNumThreads) {
    std::this_thread::yield();
  }
  start_flag.store(true, std::memory_order_release);

  for (std::thread& worker : threads) {
    worker.join();
  }

  constexpr int64_t kExpectedTotal =
      static_cast<int64_t>(kNumThreads) * kIterationsPerThread;
  EXPECT_THAT(shared_counter, t::Eq(kExpectedTotal));
  EXPECT_THAT(invariant_a + invariant_b, t::Eq(0));
  EXPECT_THAT(lock.NextTicket(), t::Eq(kExpectedTotal));
  EXPECT_THAT(lock.ServingTicket(), t::Eq(kExpectedTotal));
  EXPECT_FALSE(lock.IsLocked());
}

TEST_F(SpinLockDeathTest, InvariantViolationsTriggerDcheck) {
#ifdef NDEBUG
  // Without DCHECKs, these cases crash, hang, or silently succeed.
  GTEST_SKIP() << "requires DCHECKs (built with NDEBUG)";
#endif
  // Null pointer passed to IrqSpinLockGuard triggers DCHECK.
  EXPECT_DEATH(
      {
        IrqSpinLock* const null_lock = nullptr;
        const IrqSpinLockGuard guard(null_lock);
      },
      "Check failed");

  // Unlocking an unlocked spinlock triggers DCHECK.
  EXPECT_DEATH(
      {
        IrqSpinLock lock;
        lock.Unlock();
      },
      "Check failed");

  // Double unlock triggers DCHECK.
  EXPECT_DEATH(
      {
        IrqSpinLock lock;
        lock.Lock();
        lock.Unlock();
        lock.Unlock();
      },
      "Check failed");

  // Recursive lock on the same CPU/thread triggers DCHECK immediately without
  // deadlocking.
  EXPECT_DEATH(
      {
        IrqSpinLock lock;
        lock.Lock();
        lock.Lock();
      },
      "Check failed");

  // Out-of-order (non-LIFO) unlock of nested locks triggers DCHECK.
  EXPECT_DEATH(
      {
        IrqSpinLock lock_a;
        IrqSpinLock lock_b;
        lock_a.Lock();
        lock_b.Lock();
        lock_a.Unlock();
      },
      "Check failed");

  // Re-enabling interrupts while holding an IrqSpinLock triggers DCHECK on
  // Unlock().
  EXPECT_DEATH(
      {
        IrqSpinLock lock;
        lock.Lock();
        SetInterruptsEnabledForTest(true);
        lock.Unlock();
      },
      "Check failed");

  // Unlocking a lock held by a different thread triggers DCHECK.
  EXPECT_DEATH(
      {
        IrqSpinLock lock;
        lock.Lock();
        std::thread other([&lock]() { lock.Unlock(); });
        other.join();
      },
      "Check failed");

  // A CpuLocal whose id is out of range would make lock owner ids ambiguous.
  EXPECT_DEATH(
      {
        alignas(64) CpuLocal cpu = {};
        cpu.self = &cpu;
        cpu.cpu_id = kMaxSupportedCpus;
        BindCpuLocal(&cpu);
        IrqSpinLock lock;
        lock.Lock();
      },
      "Check failed");
}

TEST_F(SpinLockTest, OwnerIdsOfBoundCpusAreDistinctFromUnboundAndUnowned) {
  alignas(64) CpuLocal cpu = {};
  cpu.self = &cpu;
  for (const int cpu_id : {0, 1, kMaxSupportedCpus - 1}) {
    cpu.cpu_id = cpu_id;
    const uintptr_t owner = internal::OwnerIdForCpu(cpu);
    EXPECT_THAT(owner, t::Eq(static_cast<uintptr_t>(cpu_id) + 1));
    EXPECT_THAT(owner, t::Ne(uintptr_t{0}));
    EXPECT_THAT(owner, t::Ne(internal::kUnboundCpuOwnerId));
  }
}

TEST_F(SpinLockTest,
       OwnershipAndTopHeldLockTrackingWithCpuLocalBoundAndUnbound) {
  ASSERT_THAT(CurrentCpuOrNull(), t::IsNull());

  IrqSpinLock lock_a;
  IrqSpinLock lock_b;

  // 1. Unbound fallback path (`CurrentCpuOrNull() == nullptr`).
  EXPECT_THAT(*internal::TopHeldLockSlot(), t::IsNull());
  EXPECT_THAT(internal::TopHeldLock(), t::IsNull());
  EXPECT_TRUE(AreInterruptsEnabled());
  {
    const IrqSpinLockGuard guard_a(lock_a);
    EXPECT_TRUE(lock_a.IsLocked());
    EXPECT_TRUE(lock_a.IsLockedByCurrentCpu());
    EXPECT_THAT(*internal::TopHeldLockSlot(), t::Eq(&lock_a));
    EXPECT_THAT(internal::TopHeldLock(), t::Eq(&lock_a));
    EXPECT_FALSE(AreInterruptsEnabled());
    {
      const IrqSpinLockGuard guard_b(lock_b);
      EXPECT_TRUE(lock_b.IsLocked());
      EXPECT_TRUE(lock_b.IsLockedByCurrentCpu());
      EXPECT_THAT(*internal::TopHeldLockSlot(), t::Eq(&lock_b));
      EXPECT_THAT(internal::TopHeldLock(), t::Eq(&lock_b));
      EXPECT_FALSE(AreInterruptsEnabled());
    }
    EXPECT_THAT(*internal::TopHeldLockSlot(), t::Eq(&lock_a));
    EXPECT_THAT(internal::TopHeldLock(), t::Eq(&lock_a));
  }
  EXPECT_THAT(*internal::TopHeldLockSlot(), t::IsNull());
  EXPECT_THAT(internal::TopHeldLock(), t::IsNull());
  EXPECT_TRUE(AreInterruptsEnabled());

  // 2. Bound `CpuLocal` fast path (`CurrentCpuOrNull() != nullptr`).
  alignas(64) CpuLocal cpu0 = {};
  cpu0.self = &cpu0;
  cpu0.cpu_id = 0;
  cpu0.apic_id = 0;
  cpu0.online = true;

  alignas(64) CpuLocal cpu2 = {};
  cpu2.self = &cpu2;
  cpu2.cpu_id = 2;
  cpu2.apic_id = 4;
  cpu2.online = true;

  BindCpuLocal(&cpu0);
  ASSERT_THAT(CurrentCpuOrNull(), t::Eq(&cpu0));
  EXPECT_THAT(internal::CurrentOwnerId(), t::Eq(uintptr_t{1}));
  EXPECT_THAT(internal::TopHeldLockSlot(), t::Eq(&cpu0.top_held_lock));
  EXPECT_THAT(internal::TopHeldLock(), t::IsNull());
  EXPECT_THAT(cpu0.top_held_lock, t::IsNull());

  lock_a.Lock();
  EXPECT_TRUE(lock_a.IsLocked());
  EXPECT_TRUE(lock_a.IsLockedByCurrentCpu());
  EXPECT_THAT(cpu0.top_held_lock, t::Eq(&lock_a));
  EXPECT_THAT(internal::TopHeldLock(), t::Eq(&lock_a));

  lock_b.Lock();
  EXPECT_TRUE(lock_b.IsLockedByCurrentCpu());
  EXPECT_THAT(cpu0.top_held_lock, t::Eq(&lock_b));
  EXPECT_THAT(internal::TopHeldLock(), t::Eq(&lock_b));

  // Switching to a different CpuLocal shows lock_a/lock_b are NOT owned by
  // cpu2, and cpu2 has its own independent top_held_lock slot.
  BindCpuLocal(&cpu2);
  SetInterruptsEnabledForTest(true);
  EXPECT_THAT(internal::CurrentOwnerId(), t::Eq(uintptr_t{3}));
  EXPECT_FALSE(lock_a.IsLockedByCurrentCpu());
  EXPECT_FALSE(lock_b.IsLockedByCurrentCpu());
  EXPECT_THAT(cpu2.top_held_lock, t::IsNull());
  EXPECT_THAT(internal::TopHeldLock(), t::IsNull());
  EXPECT_FALSE(HoldsAnyIrqSpinLock());
  EXPECT_TRUE(AreInterruptsEnabled());

  SetInterruptsEnabledForTest(false);
  BindCpuLocal(&cpu0);
  lock_b.Unlock();
  EXPECT_THAT(cpu0.top_held_lock, t::Eq(&lock_a));
  EXPECT_THAT(internal::TopHeldLock(), t::Eq(&lock_a));
  lock_a.Unlock();
  EXPECT_THAT(cpu0.top_held_lock, t::IsNull());
  EXPECT_THAT(internal::TopHeldLock(), t::IsNull());
  EXPECT_TRUE(AreInterruptsEnabled());

  ResetCpuLocalForTest();
  EXPECT_THAT(CurrentCpuOrNull(), t::IsNull());
}

TEST_F(SpinLockTest, TryLockTakesAFreeLockLikeLock) {
  IrqSpinLock lock;
  ASSERT_TRUE(AreInterruptsEnabled());

  ASSERT_TRUE(lock.TryLock());
  EXPECT_TRUE(lock.IsLocked());
  EXPECT_TRUE(lock.IsLockedByCurrentCpu());
  EXPECT_TRUE(HoldsAnyIrqSpinLock());
  EXPECT_FALSE(AreInterruptsEnabled());
  EXPECT_THAT(lock.NextTicket(), t::Eq(1));
  EXPECT_THAT(lock.ServingTicket(), t::Eq(0));
  lock.Unlock();
  EXPECT_FALSE(lock.IsLocked());
  EXPECT_FALSE(HoldsAnyIrqSpinLock());
  EXPECT_TRUE(AreInterruptsEnabled());
  EXPECT_THAT(lock.ServingTicket(), t::Eq(1));

  // With interrupts disabled, they stay disabled after the release.
  SetInterruptsEnabledForTest(false);
  ASSERT_TRUE(lock.TryLock());
  lock.Unlock();
  EXPECT_FALSE(AreInterruptsEnabled());
}

TEST_F(SpinLockTest, TryLockFailsAtOnceWhileAnotherThreadHoldsTheLock) {
  IrqSpinLock lock;
  std::atomic<bool> holding{false};
  std::atomic<bool> release{false};
  std::thread holder([&lock, &holding, &release]() {
    const IrqSpinLockGuard guard(lock);
    holding.store(true, std::memory_order_release);
    while (!release.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
  });
  while (!holding.load(std::memory_order_acquire)) {
    std::this_thread::yield();
  }

  EXPECT_FALSE(lock.TryLock());
  // Nothing changed: no ticket taken, interrupts as before, nothing held.
  EXPECT_THAT(lock.NextTicket(), t::Eq(1));
  EXPECT_THAT(lock.ServingTicket(), t::Eq(0));
  EXPECT_TRUE(AreInterruptsEnabled());
  EXPECT_FALSE(HoldsAnyIrqSpinLock());

  release.store(true, std::memory_order_release);
  holder.join();
  ASSERT_TRUE(lock.TryLock());
  lock.Unlock();
}

TEST_F(SpinLockTest, TryLockNeverTakesTheLockAheadOfAWaiter) {
  IrqSpinLock lock;
  lock.Lock();
  std::atomic<bool> waiter_entered{false};
  std::thread waiter([&lock, &waiter_entered]() {
    const IrqSpinLockGuard guard(lock);
    waiter_entered.store(true, std::memory_order_release);
  });
  while (lock.NextTicket() < 2) {
    std::this_thread::yield();
  }

  // The lock is held and has a waiter, so another thread's TryLock() fails.
  bool other_took_lock = true;
  std::thread other([&lock, &other_took_lock]() {
    other_took_lock = lock.TryLock();
    if (other_took_lock) {
      lock.Unlock();
    }
  });
  other.join();
  EXPECT_FALSE(other_took_lock);
  EXPECT_FALSE(waiter_entered.load(std::memory_order_acquire));

  lock.Unlock();
  waiter.join();
  EXPECT_TRUE(waiter_entered.load(std::memory_order_acquire));
  EXPECT_THAT(lock.NextTicket(), t::Eq(2));
  EXPECT_THAT(lock.ServingTicket(), t::Eq(2));
}

TEST_F(SpinLockTest, TryLockAndLockKeepMutualExclusionUnderContention) {
  IrqSpinLock lock;
  constexpr int kNumThreads = 8;
  constexpr int kIterationsPerThread = 1000;
  std::atomic<bool> start_flag{false};
  std::atomic<int> active_in_critical_section{0};
  std::atomic<int64_t> failed_try_locks{0};
  int64_t shared_counter = 0;

  std::vector<std::thread> threads;
  threads.reserve(kNumThreads);
  for (int tid = 0; tid < kNumThreads; ++tid) {
    threads.emplace_back([&, tid]() {
      while (!start_flag.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      for (int iter = 0; iter < kIterationsPerThread; ++iter) {
        // Odd threads use TryLock() and count their failures.
        if (tid % 2 == 1) {
          while (!lock.TryLock()) {
            failed_try_locks.fetch_add(1, std::memory_order_relaxed);
            std::this_thread::yield();
          }
        } else {
          lock.Lock();
        }
        const int concurrent =
            active_in_critical_section.fetch_add(1, std::memory_order_acq_rel);
        EXPECT_THAT(concurrent, t::Eq(0));
        EXPECT_FALSE(AreInterruptsEnabled());
        ++shared_counter;
        active_in_critical_section.fetch_sub(1, std::memory_order_acq_rel);
        lock.Unlock();
      }
    });
  }
  start_flag.store(true, std::memory_order_release);
  for (std::thread& worker : threads) {
    worker.join();
  }

  constexpr int64_t kExpectedTotal =
      static_cast<int64_t>(kNumThreads) * kIterationsPerThread;
  EXPECT_THAT(shared_counter, t::Eq(kExpectedTotal));
  // A failed TryLock() takes no ticket.
  EXPECT_THAT(lock.NextTicket(), t::Eq(kExpectedTotal));
  EXPECT_THAT(lock.ServingTicket(), t::Eq(kExpectedTotal));
  EXPECT_FALSE(lock.IsLocked());
}

TEST_F(SpinLockDeathTest, TryLockOnALockThatTheCallerHoldsTriggersDcheck) {
#ifdef NDEBUG
  GTEST_SKIP() << "requires DCHECKs (built with NDEBUG)";
#endif
  EXPECT_DEATH(
      {
        IrqSpinLock lock;
        lock.Lock();
        (void)lock.TryLock();
      },
      "Check failed: owner_.load\\(std::memory_order_relaxed\\) != "
      "current_owner at");
}

}  // namespace
}  // namespace protos
