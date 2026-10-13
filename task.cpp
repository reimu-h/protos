#include "task.h"

#include <atomic>
#include <memory>
#include <utility>

#include "check.h"
#include "core_runqueue_scheduler.h"
#include "idt.h"
#include "join_lifecycle_scheduler.h"
#include "preemptive_scheduler.h"
#include "work_stealing_scheduler.h"

namespace protos {
namespace {

// Owns the scheduler composed by `TaskInit()`. Only `TaskInit()` and
// `TaskResetForTest()` touch it.
std::shared_ptr<TaskScheduler> g_task_scheduler_owner;

// The scheduler published to all CPUs. `TaskInit()` stores it with release
// semantics after `Init()` has completed; readers (interrupt handlers, AP idle
// loops, tasks) load it with acquire semantics, so they see a fully
// initialized scheduler or `nullptr`.
std::atomic<TaskScheduler*> g_task_scheduler{nullptr};

static void ApicTimerInterruptHandler(InterruptFrame* const frame) {
  TaskScheduler* const scheduler = GetTaskScheduler();
  if (scheduler != nullptr) {
    scheduler->OnTimerInterrupt(frame);
  }
}

static void WakeupIpiInterruptHandler(InterruptFrame* const frame) {
  TaskScheduler* const scheduler = GetTaskScheduler();
  if (scheduler != nullptr) {
    scheduler->OnWakeupIpi(frame);
  }
}

}  // namespace

void TaskInit() {
  CHECK(IdtIsInitialized());
  CHECK(GetTaskScheduler() == nullptr);

  std::shared_ptr<TaskScheduler> scheduler =
      CreatePreemptiveScheduler(CreateJoinLifecycleScheduler(
          CreateWorkStealingScheduler(CreateCoreRunqueueScheduler())));
  scheduler->Init();
  TaskScheduler* const published = scheduler.get();
  g_task_scheduler_owner = std::move(scheduler);
  g_task_scheduler.store(published, std::memory_order_release);

  IdtRegisterHandler(kVectorApicTimer, ApicTimerInterruptHandler);
  IdtRegisterHandler(kVectorWakeupIpi, WakeupIpiInterruptHandler);
}

TaskScheduler* GetTaskScheduler() {
  return g_task_scheduler.load(std::memory_order_acquire);
}

#if __STDC_HOSTED__
void TaskResetForTest() {
  if (IdtIsInitialized()) {
    IdtUnregisterHandler(kVectorApicTimer);
    IdtUnregisterHandler(kVectorWakeupIpi);
  }
  g_task_scheduler.store(nullptr, std::memory_order_release);
  g_task_scheduler_owner.reset();
}
#endif

}  // namespace protos
