#include "preemptive_scheduler.h"

#include <atomic>
#include <memory>
#include <utility>

#include "forwarding_task_scheduler.h"
#include "idt.h"
#include "smp.h"
#include "spinlock.h"
#include "task_scheduler_internals.h"

namespace protos {
namespace {

// Owns the global preemption flag and decides whether a timer interrupt or
// wakeup IPI may reach the inner layers at all. Every other method is
// inherited from `ForwardingTaskScheduler`.
class PreemptiveTaskScheduler final : public ForwardingTaskScheduler {
 public:
  using ForwardingTaskScheduler::ForwardingTaskScheduler;

  void SetPreemptEnabled(const bool enabled) override {
    preempt_enabled_.store(enabled, std::memory_order_release);
  }

  bool IsPreemptEnabled() const override {
    return preempt_enabled_.load(std::memory_order_acquire);
  }

 private:
  // Scheduler internals:
  void Init() override {
    preempt_enabled_.store(true, std::memory_order_relaxed);
    TaskSchedulerInternals::Init(inner());
  }

  void OnTimerInterrupt(InterruptFrame* const frame) override {
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

  std::atomic<bool> preempt_enabled_{true};
};

}  // namespace

std::shared_ptr<TaskScheduler> CreatePreemptiveScheduler(
    std::shared_ptr<TaskScheduler> inner) {
  return std::make_shared<PreemptiveTaskScheduler>(std::move(inner));
}

}  // namespace protos
