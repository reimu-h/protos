#ifndef PROTOS_TASK_SCHEDULER_INTERNALS_H_
#define PROTOS_TASK_SCHEDULER_INTERNALS_H_

#include "task.h"

namespace protos {

// The only way to call the scheduler internals of a `TaskScheduler` (see
// task.h), which are private: each static method calls the method of the same
// name on `scheduler`. task.cpp, the scheduler layers and the scheduler tests
// use it. Task code calls only the task API and must not include this header.
class TaskSchedulerInternals {
 public:
  TaskSchedulerInternals() = delete;

  static void Init(TaskScheduler& scheduler) { scheduler.Init(); }

  static void OnTimerInterrupt(TaskScheduler& scheduler,
                               InterruptFrame* const frame) {
    scheduler.OnTimerInterrupt(frame);
  }

  static void OnWakeupIpi(TaskScheduler& scheduler,
                          InterruptFrame* const frame) {
    scheduler.OnWakeupIpi(frame);
  }

  static bool PollIdleCpu(TaskScheduler& scheduler, const int cpu_index) {
    return scheduler.PollIdleCpu(cpu_index);
  }

  static int RunqueueLoad(const TaskScheduler& scheduler, const int cpu_index) {
    return scheduler.RunqueueLoad(cpu_index);
  }

  static bool IsCpuBusy(const TaskScheduler& scheduler, const int cpu_index) {
    return scheduler.IsCpuBusy(cpu_index);
  }

  static bool StealTask(TaskScheduler& scheduler,  //
                        const int dst_cpu,         //
                        const int src_cpu) {
    return scheduler.StealTask(dst_cpu, src_cpu);
  }

  static bool WakeOnCpu(TaskScheduler& scheduler,  //
                        Task* const task,          //
                        const int target_cpu) {
    return scheduler.WakeOnCpu(task, target_cpu);
  }
};

}  // namespace protos

#endif  // PROTOS_TASK_SCHEDULER_INTERNALS_H_
