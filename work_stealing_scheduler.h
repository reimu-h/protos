#ifndef PROTOS_WORK_STEALING_SCHEDULER_H_
#define PROTOS_WORK_STEALING_SCHEDULER_H_

#include <memory>

#include "task.h"

namespace protos {

// Creates the cross-CPU placement and work-stealing layer wrapping `inner`.
// `inner` must be non-null and must provide the core layer's `RunqueueLoad()`,
// `IsCpuBusy()` and `StealTask()` semantics (see core_runqueue_scheduler.h),
// so in practice this layer wraps `CreateCoreRunqueueScheduler()` directly,
// inside the preemptive and join-lifecycle layers (see `TaskInit()`). It
// overrides `CreateTask()`, `CreateTaskOnCpu()`, `Yield()`, `PollIdleCpu()`,
// `OnTimerInterrupt()` and `OnWakeupIpi()` to add:
//
// - Load: CPUs are compared by their load, the number of their queued tasks
//   that may be stolen (`RunqueueLoad()`) plus 1 if they are running a task
//   other than their idle task (`IsCpuBusy()`). A busy CPU with nothing queued
//   thus has load 1, and an idle one load 0.
// - Placement: `CreateTask()` picks the online CPU with the smallest load
//   (the calling CPU's load includes the caller). Ties go to the calling CPU,
//   which needs no wakeup IPI, and then to the lowest CPU index.
// - Wakeups: `CreateTask()` and `CreateTaskOnCpu()` send `kVectorWakeupIpi` to
//   the target CPU unless it is the calling CPU, so a halted AP notices the new
//   task at once instead of at its next timer tick.
// - Stealing: when the calling CPU has nothing queued, `PollIdleCpu()` and
//   `Yield()` first move one task to it from the CPU with the largest load
//   that has a queued task: an idle CPU, and a CPU whose running task yields
//   it, take any queued task. `OnTimerInterrupt()` and `OnWakeupIpi()` do the
//   same if the interrupted CPU is idle. If it is busy, they take a task only
//   from a CPU whose load exceeds its own by at least 2: a task taken from a
//   CPU only one task busier would just swap the two loads, and with slightly
//   more tasks than CPUs, tasks would hop between CPUs on almost every tick.
//   `StealTask()` moves the donor's task that would wait longest there. Each
//   steal tries at most a few donors, busiest first.
//
// Every other method is inherited from `ForwardingTaskScheduler`, and every
// call after the stealing step is forwarded to `inner` unchanged; in
// particular, `inner` decides whether an interrupt preempts the running task.
//
// Interrupts and threading: the layer reads the calling CPU only with local
// interrupts disabled. `PollIdleCpu(cpu_index)` does nothing and returns false
// unless it is called on CPU `cpu_index`, so it never steals work for another
// CPU. With the core as `inner`, `PollIdleCpu()`, `OnTimerInterrupt()` and
// `OnWakeupIpi()` are safe before `Init()`.
std::shared_ptr<TaskScheduler> CreateWorkStealingScheduler(
    std::shared_ptr<TaskScheduler> inner);

}  // namespace protos

#endif  // PROTOS_WORK_STEALING_SCHEDULER_H_
