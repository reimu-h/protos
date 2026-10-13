#ifndef PROTOS_PREEMPTIVE_SCHEDULER_H_
#define PROTOS_PREEMPTIVE_SCHEDULER_H_

#include <memory>

#include "task.h"

namespace protos {

// Creates the outermost preemptive interrupt task scheduler layer wrapping
// `inner`. It owns the global preemption flag (SetPreemptEnabled() and
// IsPreemptEnabled() are not forwarded) and forwards OnTimerInterrupt() and
// OnWakeupIpi() only when the interrupted context may be preempted (see
// TaskScheduler::OnTimerInterrupt()). All other methods are forwarded to
// `inner`.
std::shared_ptr<TaskScheduler> CreatePreemptiveScheduler(
    std::shared_ptr<TaskScheduler> inner);

}  // namespace protos

#endif  // PROTOS_PREEMPTIVE_SCHEDULER_H_
