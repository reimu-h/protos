#ifndef PROTOS_FORWARDING_TASK_SCHEDULER_H_
#define PROTOS_FORWARDING_TASK_SCHEDULER_H_

#include <cstdint>
#include <memory>
#include <utility>

#include "check.h"
#include "task.h"
#include "task_scheduler_internals.h"

namespace protos {

// A scheduler layer (see `TaskScheduler` in task.h) that forwards every method,
// unchanged, to the inner layer that it wraps. Each layer that wraps another
// derives from it and overrides only the methods it changes, calling `inner()`
// to reach the wrapped layer (through `TaskSchedulerInternals` for the
// scheduler internals). The overrides of the scheduler internals are protected
// rather than private, so that a derived layer can also call the forwarding
// version, e.g. `ForwardingTaskScheduler::OnTimerInterrupt(frame)`.
class ForwardingTaskScheduler : public TaskScheduler {
 public:
  // CHECK-fails if `inner` is null.
  explicit ForwardingTaskScheduler(std::shared_ptr<TaskScheduler> inner)
      : inner_(std::move(inner)) {
    CHECK(inner_ != nullptr);
  }

  // Task API:
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

  void SetPreemptEnabled(const bool enabled) override {
    inner_->SetPreemptEnabled(enabled);
  }

  bool IsPreemptEnabled() const override { return inner_->IsPreemptEnabled(); }
  Task* CurrentTask() const override { return inner_->CurrentTask(); }

 protected:
  // Scheduler internals:
  void Init() override { TaskSchedulerInternals::Init(*inner_); }

  void OnTimerInterrupt(InterruptFrame* const frame) override {
    TaskSchedulerInternals::OnTimerInterrupt(*inner_, frame);
  }

  void OnWakeupIpi(InterruptFrame* const frame) override {
    TaskSchedulerInternals::OnWakeupIpi(*inner_, frame);
  }

  bool PollIdleCpu(const int cpu_index) override {
    return TaskSchedulerInternals::PollIdleCpu(*inner_, cpu_index);
  }

  int RunqueueLoad(const int cpu_index) const override {
    return TaskSchedulerInternals::RunqueueLoad(*inner_, cpu_index);
  }

  bool IsCpuBusy(const int cpu_index) const override {
    return TaskSchedulerInternals::IsCpuBusy(*inner_, cpu_index);
  }

  bool StealTask(const int dst_cpu, const int src_cpu) override {
    return TaskSchedulerInternals::StealTask(*inner_, dst_cpu, src_cpu);
  }

  // Returns the wrapped layer.
  TaskScheduler& inner() const { return *inner_; }

 private:
  const std::shared_ptr<TaskScheduler> inner_;
};

}  // namespace protos

#endif  // PROTOS_FORWARDING_TASK_SCHEDULER_H_
