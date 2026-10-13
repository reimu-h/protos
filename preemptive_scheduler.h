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
// - `Init()`, to re-enable preemption before it forwards.
//
// Every other method is inherited from `ForwardingTaskScheduler` and forwarded
// to `inner` unchanged.
std::shared_ptr<TaskScheduler> CreatePreemptiveScheduler(
    std::shared_ptr<TaskScheduler> inner);

}  // namespace protos

#endif  // PROTOS_PREEMPTIVE_SCHEDULER_H_
