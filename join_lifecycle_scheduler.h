#ifndef PROTOS_JOIN_LIFECYCLE_SCHEDULER_H_
#define PROTOS_JOIN_LIFECYCLE_SCHEDULER_H_

#include <memory>

#include "task.h"

namespace protos {

// Creates the outermost scheduler layer, which wraps `inner` (which must be
// non-null) and adds task join synchronization and the zombie lifecycle, which
// the inner layers don't implement:
//
// - `CreateTask()` and `CreateTaskOnCpu()` track each new task in a join
//   record that the task marks when its entry function returns.
// - `Join()`, `Detach()` and `ReapZombies()` are built on inner `Join()`,
//   which must wait for a task to exit and then free it. They never call inner
//   `Detach()` or `ReapZombies()`.
// - `Init()` drops the records of all tracked tasks (without calling inner
//   `Join()`) before it forwards, since `Init()` resets the inner layers too.
//
// Every other method is inherited from `ForwardingTaskScheduler` and forwarded
// to `inner` unchanged.
std::shared_ptr<TaskScheduler> CreateJoinLifecycleScheduler(
    std::shared_ptr<TaskScheduler> inner);

}  // namespace protos

#endif  // PROTOS_JOIN_LIFECYCLE_SCHEDULER_H_
