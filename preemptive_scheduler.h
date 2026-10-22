#ifndef PROTOS_PREEMPTIVE_SCHEDULER_H_
#define PROTOS_PREEMPTIVE_SCHEDULER_H_

#include <memory>

#include "task.h"

namespace protos {

// Creates the preemptive interrupt task scheduler layer wrapping `inner`, which
// must be non-null. It overrides:
//
// - `SetPreemptEnabled()` and `IsPreemptEnabled()`, which it does not forward:
//   the layer owns the global preemption flag.
// - `OnTimerInterrupt()` and `OnWakeupIpi()`, which it forwards only when the
//   interrupted context may be preempted (see
//   `TaskScheduler::OnTimerInterrupt()`).
// - `BlockUntil()`, which it does not forward: the layer implements timed
//   waits on top of inner `CurrentTask()`, `Block()` and `Wake()`. A waiting
//   task is linked, on its own stack, into a list sorted by deadline (no
//   fixed capacity), and `OnTimerInterrupt()` on CPU 0 wakes, with inner
//   `Wake()`, each task whose deadline (in CPU 0's `CpuLocal::timer_ticks`)
//   has passed, before it decides whether to forward the tick. So a timed
//   wait ends while preemption is disabled too. The list has an `IrqSpinLock`
//   of its own, which the layer holds while it calls inner `Wake()`.
// - `Init()`, to re-enable preemption and forget timed waiters before it
//   forwards.
//
// Every other method is inherited from `ForwardingTaskScheduler` and forwarded
// to `inner` unchanged.
std::shared_ptr<TaskScheduler> CreatePreemptiveScheduler(
    std::shared_ptr<TaskScheduler> inner);

}  // namespace protos

#endif  // PROTOS_PREEMPTIVE_SCHEDULER_H_
