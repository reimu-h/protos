#include "preemptive_scheduler.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <utility>

#include "check.h"
#include "idt.h"
#include "smp.h"
#include "spinlock.h"

namespace protos {
namespace {

// Outermost layer: owns the global preemption flag and decides whether a timer
// interrupt or wakeup IPI may reach the inner layers at all. Everything else
// is forwarded unchanged.
class PreemptiveTaskScheduler final : public TaskScheduler {
 public:
  explicit PreemptiveTaskScheduler(std::shared_ptr<TaskScheduler> inner)
      : inner_(std::move(inner)) {
    DCHECK(inner_ != nullptr);
  }

  void Init() override {
    preempt_enabled_.store(true, std::memory_order_relaxed);
    inner_->Init();
  }

  Task* CreateTask(const TaskFn entry,  //
                   void* const arg,     //
                   const int64_t weight) override {
    return inner_->CreateTask(entry, arg, weight);
  }

  Task* CreateTaskOnCpu(const TaskFn entry,    //
                        void* const arg,       //
                        const int64_t weight,  //
                        const int target_cpu) override {
    return inner_->CreateTaskOnCpu(entry, arg, weight, target_cpu);
  }

  void Yield() override { inner_->Yield(); }
  void Join(Task* const task) override { inner_->Join(task); }
  void Detach(Task* const task) override { inner_->Detach(task); }
  int ReapZombies() override { return inner_->ReapZombies(); }

  bool PollIdleCpu(const int cpu_index) override {
    return inner_->PollIdleCpu(cpu_index);
  }

  int RunqueueLoad(const int cpu_index) const override {
    return inner_->RunqueueLoad(cpu_index);
  }

  bool StealTask(const int dst_cpu, const int src_cpu) override {
    return inner_->StealTask(dst_cpu, src_cpu);
  }

  void SetPreemptEnabled(const bool enabled) override {
    preempt_enabled_.store(enabled, std::memory_order_release);
  }

  bool IsPreemptEnabled() const override {
    return preempt_enabled_.load(std::memory_order_acquire);
  }

  void OnTimerInterrupt(InterruptFrame* const frame) override {
    if (!CanPreemptFromInterrupt(frame)) {
      return;
    }
    inner_->OnTimerInterrupt(frame);
  }

  void OnWakeupIpi(InterruptFrame* const frame) override {
    if (!CanPreemptFromInterrupt(frame)) {
      return;
    }
    inner_->OnWakeupIpi(frame);
  }

  Task* CurrentTask() const override { return inner_->CurrentTask(); }

 private:
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

  const std::shared_ptr<TaskScheduler> inner_;
  std::atomic<bool> preempt_enabled_{true};
};

}  // namespace

std::shared_ptr<TaskScheduler> CreatePreemptiveScheduler(
    std::shared_ptr<TaskScheduler> inner) {
  return std::make_shared<PreemptiveTaskScheduler>(std::move(inner));
}

}  // namespace protos
