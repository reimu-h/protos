#include "preemptive_scheduler.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <utility>

#include "check.h"
#include "forwarding_task_scheduler.h"
#include "idt.h"
#include "interrupts.h"
#include "smp.h"
#include "spinlock.h"
#include "task_scheduler_internals.h"

namespace protos {
namespace {

// Returns CPU 0's timer tick count, the clock of BlockUntil() deadlines.
static int64_t Cpu0TimerTicks() {
  const CpuLocal* const cpu0 = SmpGetCpuLocal(0);
  DCHECK(cpu0 != nullptr);
  return cpu0->timer_ticks.load(std::memory_order_relaxed);
}

// Owns the global preemption flag and decides whether a timer interrupt or
// wakeup IPI may reach the inner layers at all. Also implements timed waits
// (BlockUntil()), which CPU 0's timer interrupt ends whether or not it may
// preempt. Every other method is inherited from `ForwardingTaskScheduler`.
class PreemptiveTaskScheduler final : public ForwardingTaskScheduler {
 public:
  using ForwardingTaskScheduler::ForwardingTaskScheduler;

  void BlockUntil(const int64_t deadline_tick) override {
    CHECK(!HoldsAnyIrqSpinLock());
    CHECK(AreInterruptsEnabled());
    // Lives on the waiting task's stack, which stays allocated while the task
    // is blocked, and is linked in `timed_waiters_` until the timer interrupt
    // unlinks it or this function does.
    TimedWaiter waiter;
    waiter.task = inner().CurrentTask();
    waiter.deadline_tick = deadline_tick;
    {
      const IrqSpinLockGuard guard(timed_waiters_lock_);
      // Under the lock: a tick that reaches the deadline after this check
      // finds the waiter linked.
      if (Cpu0TimerTicks() >= deadline_tick) {
        return;
      }
      LinkTimedWaiterLocked(&waiter);
    }
    inner().Block();
    const IrqSpinLockGuard guard(timed_waiters_lock_);
    if (!waiter.timer_woke) {
      UnlinkTimedWaiterLocked(&waiter);
    }
  }

  void SetPreemptEnabled(const bool enabled) override {
    preempt_enabled_.store(enabled, std::memory_order_release);
  }

  bool IsPreemptEnabled() const override {
    return preempt_enabled_.load(std::memory_order_acquire);
  }

 private:
  // A task in BlockUntil(), linked in `timed_waiters_` until its deadline.
  struct TimedWaiter {
    Task* task = nullptr;
    int64_t deadline_tick = 0;
    TimedWaiter* prev = nullptr;
    TimedWaiter* next = nullptr;
    // Set when the timer interrupt unlinked the waiter and woke its task.
    bool timer_woke = false;
  };

  // Scheduler internals:
  void Init() override {
    preempt_enabled_.store(true, std::memory_order_relaxed);
    {
      const IrqSpinLockGuard guard(timed_waiters_lock_);
      timed_waiters_ = nullptr;
    }
    TaskSchedulerInternals::Init(inner());
  }

  void OnTimerInterrupt(InterruptFrame* const frame) override {
    // Before the preemption checks, which would drop the tick while
    // preemption is disabled or the interrupted code has interrupts disabled:
    // a timed wait must end anyway.
    WakeExpiredTimedWaiters();
    if (!CanPreemptFromInterrupt(frame)) {
      return;
    }
    TaskSchedulerInternals::OnTimerInterrupt(inner(), frame);
  }

  void OnWakeupIpi(InterruptFrame* const frame) override {
    if (!CanPreemptFromInterrupt(frame)) {
      return;
    }
    TaskSchedulerInternals::OnWakeupIpi(inner(), frame);
  }

  // Returns true if the context interrupted by `frame` may be switched out.
  // The decision depends only on the saved RFLAGS in `frame`, not on the
  // current interrupt flag, which is always clear inside an interrupt gate.
  // Host tests call the handlers directly and must pass a frame like the one
  // the CPU would push (`rflags = kRflagsInterruptEnableBit | 0x2`).
  bool CanPreemptFromInterrupt(const InterruptFrame* const frame) const {
    if (frame == nullptr || (frame->rflags & kRflagsInterruptEnableBit) == 0) {
      return false;
    }
    const CpuLocal* const cpu = CurrentCpuOrNull();
    if (cpu == nullptr || !cpu->online) {
      return false;
    }
    if (!preempt_enabled_.load(std::memory_order_acquire)) {
      return false;
    }
    return !HoldsAnyIrqSpinLock();
  }

  // On CPU 0 (whose tick count is the deadline clock), wakes the task of each
  // waiter whose deadline has passed, after unlinking the waiter. The Wake()
  // goes through the inner layers, which pick the CPU, and happens under
  // `timed_waiters_lock_`, so the waiter's task can't return from
  // BlockUntil() and exit meanwhile.
  void WakeExpiredTimedWaiters() {
    const CpuLocal* const cpu = CurrentCpuOrNull();
    if (cpu == nullptr || cpu->cpu_id != 0) {
      return;
    }
    const int64_t now = cpu->timer_ticks.load(std::memory_order_relaxed);
    const IrqSpinLockGuard guard(timed_waiters_lock_);
    while (timed_waiters_ != nullptr && timed_waiters_->deadline_tick <= now) {
      TimedWaiter* const waiter = timed_waiters_;
      UnlinkTimedWaiterLocked(waiter);
      waiter->timer_woke = true;
      inner().Wake(waiter->task);
    }
  }

  // Links `waiter` into `timed_waiters_`, after every waiter whose deadline
  // is not later.
  void LinkTimedWaiterLocked(TimedWaiter* const waiter) {
    TimedWaiter* prev = nullptr;
    TimedWaiter* next = timed_waiters_;
    while (next != nullptr && next->deadline_tick <= waiter->deadline_tick) {
      prev = next;
      next = next->next;
    }
    waiter->prev = prev;
    waiter->next = next;
    if (prev != nullptr) {
      prev->next = waiter;
    } else {
      timed_waiters_ = waiter;
    }
    if (next != nullptr) {
      next->prev = waiter;
    }
  }

  void UnlinkTimedWaiterLocked(TimedWaiter* const waiter) {
    if (waiter->prev != nullptr) {
      waiter->prev->next = waiter->next;
    } else {
      DCHECK(timed_waiters_ == waiter);
      timed_waiters_ = waiter->next;
    }
    if (waiter->next != nullptr) {
      waiter->next->prev = waiter->prev;
    }
    waiter->prev = nullptr;
    waiter->next = nullptr;
  }

  std::atomic<bool> preempt_enabled_{true};
  // Taken by BlockUntil() in task context and by CPU 0's timer interrupt.
  // Lock order: this lock, then the inner layers' runqueue locks (Wake()).
  IrqSpinLock timed_waiters_lock_;
  // The tasks in BlockUntil(), by deadline, earliest first. Guarded by
  // `timed_waiters_lock_`.
  TimedWaiter* timed_waiters_ = nullptr;
};

}  // namespace

std::shared_ptr<TaskScheduler> CreatePreemptiveScheduler(
    std::shared_ptr<TaskScheduler> inner) {
  return std::make_shared<PreemptiveTaskScheduler>(std::move(inner));
}

}  // namespace protos
