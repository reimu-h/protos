#ifndef PROTOS_JOIN_LIFECYCLE_SCHEDULER_H_
#define PROTOS_JOIN_LIFECYCLE_SCHEDULER_H_

#include <memory>

#include "task.h"

namespace protos {

// Creates the task join synchronization and zombie lifecycle scheduler layer
// wrapping `inner`. It implements Join(), Detach() and ReapZombies() on top of
// inner Join(), which must wait for a task to exit and then free it; it never
// calls inner Detach() or inner ReapZombies(). All other methods are forwarded
// to `inner`.
std::shared_ptr<TaskScheduler> CreateJoinLifecycleScheduler(
    std::shared_ptr<TaskScheduler> inner);

}  // namespace protos

#endif  // PROTOS_JOIN_LIFECYCLE_SCHEDULER_H_
