#include "work_stealing_scheduler.h"

#include <climits>
#include <cstdint>
#include <memory>
#include <utility>

#include "check.h"
#include "forwarding_task_scheduler.h"
#include "idt.h"
#include "smp.h"
#include "spinlock.h"
#include "task_scheduler_internals.h"

namespace protos {
namespace {

// Smallest `RunqueueLoad()` a donor CPU needs before this layer steals from
// it. A CPU that is idle, or yielding with nothing queued, takes any queued
// task. Timer ticks and wakeup IPIs also arrive on busy CPUs, whose running
// task `RunqueueLoad()` does not count, so they steal only from a donor with at
// least two queued tasks; otherwise, with slightly more tasks than CPUs, tasks
// would hop between busy CPUs on almost every tick. An idle AP still picks up a
// lone queued task in `PollIdleCpu()` right after the interrupt wakes it.
constexpr int kIdleStealMinDonorLoad = 1;
constexpr int kInterruptStealMinDonorLoad = 2;

// Maximum number of donor CPUs, busiest first, that one steal tries.
// `StealTask()` fails only when the donor's queue drained after its load was
// read, so a few attempts suffice; each attempt rescans all CPUs, so this
// bounds a steal to O(cpu_count) load reads.
constexpr int kMaxStealAttempts = 3;

class WorkStealingTaskScheduler final : public ForwardingTaskScheduler {
 public:
  using ForwardingTaskScheduler::ForwardingTaskScheduler;

  Task* CreateTask(const TaskFn entry,  //
                   void* const arg,     //
                   const int64_t weight) override {
    DCHECK(entry != nullptr);
    DCHECK(weight >= kMinTaskWeight && weight <= kMaxTaskWeight);
    const int target_cpu = SelectLeastLoadedCpu();
    return CreateTaskOnCpu(entry, arg, weight, target_cpu);
  }

  Task* CreateTaskOnCpu(const TaskFn entry,    //
                        void* const arg,       //
                        const int64_t weight,  //
                        const int target_cpu) override {
    DCHECK(entry != nullptr);
    DCHECK(weight >= kMinTaskWeight && weight <= kMaxTaskWeight);
    const int cpu_count = SmpCpuCount();
    DCHECK(target_cpu >= 0 && target_cpu < cpu_count);
    DCHECK(IsCpuOnline(target_cpu, cpu_count));

    Task* const task = inner().CreateTaskOnCpu(entry, arg, weight, target_cpu);
    SendWakeupIpiIfRemote(target_cpu);
    return task;
  }

  void Yield() override {
    const bool saved_irq = SaveAndDisableInterrupts();
    StealIfLocalQueueEmpty(kIdleStealMinDonorLoad);
    RestoreInterrupts(saved_irq);
    inner().Yield();
  }

 private:
  // Scheduler internals:
  bool PollIdleCpu(const int cpu_index) override {
    const bool saved_irq = SaveAndDisableInterrupts();
    const bool dispatched = PollIdleCpuIrqsOff(cpu_index);
    RestoreInterrupts(saved_irq);
    return dispatched;
  }

  void OnTimerInterrupt(InterruptFrame* const frame) override {
    const bool saved_irq = SaveAndDisableInterrupts();
    StealIfLocalQueueEmpty(kInterruptStealMinDonorLoad);
    RestoreInterrupts(saved_irq);
    TaskSchedulerInternals::OnTimerInterrupt(inner(), frame);
  }

  void OnWakeupIpi(InterruptFrame* const frame) override {
    const bool saved_irq = SaveAndDisableInterrupts();
    StealIfLocalQueueEmpty(kInterruptStealMinDonorLoad);
    RestoreInterrupts(saved_irq);
    TaskSchedulerInternals::OnWakeupIpi(inner(), frame);
  }

  // Body of `PollIdleCpu()`, called with local interrupts disabled. Steals
  // work only for the calling CPU, so that a stolen task never lands on a
  // remote CPU that nobody wakes up.
  bool PollIdleCpuIrqsOff(const int cpu_index) {
    const CpuLocal* const cpu = CurrentCpuOrNullIrqsOff();
    if (cpu == nullptr || !cpu->online || cpu->cpu_id != cpu_index) {
      return false;
    }
    if (TaskSchedulerInternals::RunqueueLoad(inner(), cpu_index) == 0 &&
        !TryStealForCpu(cpu_index, kIdleStealMinDonorLoad)) {
      return false;
    }
    return TaskSchedulerInternals::PollIdleCpu(inner(), cpu_index);
  }

  // With local interrupts disabled: if the calling CPU is online and has
  // nothing queued, steals one task for it from a donor with at least
  // `min_donor_load` queued tasks.
  void StealIfLocalQueueEmpty(const int min_donor_load) {
    const CpuLocal* const cpu = CurrentCpuOrNullIrqsOff();
    if (cpu != nullptr && cpu->online &&
        TaskSchedulerInternals::RunqueueLoad(inner(), cpu->cpu_id) == 0) {
      (void)TryStealForCpu(cpu->cpu_id, min_donor_load);
    }
  }

  // Returns the online CPU with the smallest `RunqueueLoad()`, counting the
  // calling CPU as one task busier because it is running the caller, which
  // `RunqueueLoad()` does not count. Ties go to the lowest CPU index.
  int SelectLeastLoadedCpu() const {
    const int cpu_count = SmpCpuCount();
    DCHECK(cpu_count >= 1);
    const int local_cpu = CurrentCpuIdOrUnbound();
    int best_cpu = 0;
    int best_load = INT_MAX;
    for (int cpu = 0; cpu < cpu_count; ++cpu) {
      if (!IsCpuOnline(cpu, cpu_count)) {
        continue;
      }
      const int load = TaskSchedulerInternals::RunqueueLoad(inner(), cpu) +
                       ((cpu == local_cpu) ? 1 : 0);
      if (load < best_load) {
        best_load = load;
        best_cpu = cpu;
      }
    }
    return best_cpu;
  }

  static void SendWakeupIpiIfRemote(const int target_cpu) {
    if (target_cpu != CurrentCpuIdOrUnbound()) {
      SmpSendIpi(target_cpu, kVectorWakeupIpi);
    }
  }

  // Tries to move one task to `dst_cpu` from the online CPU with the largest
  // `RunqueueLoad()` that is at least `min_donor_load` (ties go to the lowest
  // index). If `StealTask()` fails, tries the next-busiest donor, for at most
  // `kMaxStealAttempts` donors. Returns true if a steal succeeded.
  bool TryStealForCpu(const int dst_cpu, const int min_donor_load) {
    DCHECK(min_donor_load >= 1);
    const int cpu_count = SmpCpuCount();
    if (dst_cpu < 0 || dst_cpu >= cpu_count || cpu_count <= 1) {
      return false;
    }

    // Donors are tried in descending (load, -cpu) order; a CPU that sorts at
    // or before the last donor tried has already been tried.
    int prev_load = INT_MAX;
    int prev_src = -1;

    for (int attempt = 0; attempt < kMaxStealAttempts; ++attempt) {
      int best_src = -1;
      int best_load = min_donor_load - 1;
      for (int cpu = 0; cpu < cpu_count; ++cpu) {
        if (cpu == dst_cpu || !IsCpuOnline(cpu, cpu_count)) {
          continue;
        }
        const int load = TaskSchedulerInternals::RunqueueLoad(inner(), cpu);
        const bool already_tried =
            (load > prev_load) || (load == prev_load && cpu <= prev_src);
        if (already_tried) {
          continue;
        }
        if (load > best_load) {
          best_load = load;
          best_src = cpu;
        }
      }
      if (best_src < 0) {
        break;
      }
      prev_load = best_load;
      prev_src = best_src;
      if (TaskSchedulerInternals::StealTask(inner(), dst_cpu, best_src)) {
        return true;
      }
    }
    return false;
  }
};

}  // namespace

std::shared_ptr<TaskScheduler> CreateWorkStealingScheduler(
    std::shared_ptr<TaskScheduler> inner) {
  return std::make_shared<WorkStealingTaskScheduler>(std::move(inner));
}

}  // namespace protos
