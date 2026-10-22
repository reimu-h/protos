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

// `min_donor_load` (see `TryStealForCpu()`) that lets a CPU take any queued
// task: every CPU with a queued task has at least this load.
constexpr int kAnyQueuedTask = 1;

// Smallest amount by which the load of a donor CPU must exceed the load of a
// busy CPU with nothing queued (1) before a timer tick or wakeup IPI on the
// busy CPU steals from the donor. Taking a task from a CPU that is only one
// task busier would just swap the two loads, and with slightly more tasks
// than CPUs the tasks would then hop between CPUs on almost every tick.
constexpr int kMinInterruptStealImbalance = 2;

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
    // The yielding task offers its CPU, so the CPU takes any queued task, as
    // an idle CPU does.
    const int cpu = LocalCpuWithEmptyQueueIrqsOff();
    if (cpu >= 0) {
      (void)TryStealForCpu(cpu, kAnyQueuedTask);
    }
    RestoreInterrupts(saved_irq);
    inner().Yield();
  }

  void Wake(Task* const task) override {
    DCHECK(task != nullptr);
    // With interrupts disabled, so that the CPU that picks the target is the
    // one that decides whether the target is remote.
    const bool saved_irq = SaveAndDisableInterrupts();
    (void)WakeOnCpu(task, SelectLeastLoadedCpu());
    RestoreInterrupts(saved_irq);
  }

 private:
  // Scheduler internals:
  bool WakeOnCpu(Task* const task, const int target_cpu) override {
    DCHECK(task != nullptr);
    const int cpu_count = SmpCpuCount();
    DCHECK(target_cpu >= 0 && target_cpu < cpu_count);
    DCHECK(IsCpuOnline(target_cpu, cpu_count));
    const bool queued =
        TaskSchedulerInternals::WakeOnCpu(inner(), task, target_cpu);
    // Only the call that queued the task wakes its CPU. `task` may have run
    // and exited already, so this doesn't touch it.
    if (queued) {
      SendWakeupIpiIfRemote(target_cpu);
    }
    return queued;
  }

  bool PollIdleCpu(const int cpu_index) override {
    const bool saved_irq = SaveAndDisableInterrupts();
    const bool dispatched = PollIdleCpuIrqsOff(cpu_index);
    RestoreInterrupts(saved_irq);
    return dispatched;
  }

  void OnTimerInterrupt(InterruptFrame* const frame) override {
    const bool saved_irq = SaveAndDisableInterrupts();
    StealForInterruptedCpuIrqsOff();
    RestoreInterrupts(saved_irq);
    TaskSchedulerInternals::OnTimerInterrupt(inner(), frame);
  }

  void OnWakeupIpi(InterruptFrame* const frame) override {
    const bool saved_irq = SaveAndDisableInterrupts();
    StealForInterruptedCpuIrqsOff();
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
        !TryStealForCpu(cpu_index, kAnyQueuedTask)) {
      return false;
    }
    return TaskSchedulerInternals::PollIdleCpu(inner(), cpu_index);
  }

  // With local interrupts disabled, on a timer tick or wakeup IPI: if the
  // interrupted (calling) CPU has nothing queued, steals one task for it. An
  // idle CPU takes any queued task. A busy CPU, whose load is 1, takes a task
  // only from a CPU whose load exceeds its own by at least
  // `kMinInterruptStealImbalance`.
  void StealForInterruptedCpuIrqsOff() {
    const int cpu = LocalCpuWithEmptyQueueIrqsOff();
    if (cpu < 0) {
      return;
    }
    const int min_donor_load = TaskSchedulerInternals::IsCpuBusy(inner(), cpu)
                                   ? 1 + kMinInterruptStealImbalance
                                   : kAnyQueuedTask;
    (void)TryStealForCpu(cpu, min_donor_load);
  }

  // With local interrupts disabled: returns the calling CPU if it is online
  // and has nothing queued, or -1 otherwise.
  int LocalCpuWithEmptyQueueIrqsOff() const {
    const CpuLocal* const cpu = CurrentCpuOrNullIrqsOff();
    if (cpu == nullptr || !cpu->online ||
        TaskSchedulerInternals::RunqueueLoad(inner(), cpu->cpu_id) != 0) {
      return -1;
    }
    return cpu->cpu_id;
  }

  // Returns the load of `cpu` that placement and stealing compare: the
  // number of its queued tasks that may be stolen (`RunqueueLoad()`), plus 1
  // if it is running a task other than its idle task (`IsCpuBusy()`). A busy
  // CPU with nothing queued thus has load 1, and an idle one load 0.
  int Load(const int cpu) const {
    return TaskSchedulerInternals::RunqueueLoad(inner(), cpu) +
           (TaskSchedulerInternals::IsCpuBusy(inner(), cpu) ? 1 : 0);
  }

  // Returns the online CPU with the smallest `Load()`. The calling CPU's load
  // includes the caller, which is the task it is running. Ties go to the
  // calling CPU, which needs no wakeup IPI, and then to the lowest CPU index:
  // ties toward CPU 0 alone would pile new work up on CPU 0 whenever all CPUs
  // are equally busy.
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
      const int load = Load(cpu);
      if (load < best_load || (load == best_load && cpu == local_cpu)) {
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
  // `Load()` among those that have a queued task and a load of at least
  // `min_donor_load` (ties go to the lowest index). If `StealTask()` fails,
  // tries the next-busiest such donor, for at most `kMaxStealAttempts`
  // donors. Returns true if a steal succeeded.
  bool TryStealForCpu(const int dst_cpu, const int min_donor_load) {
    DCHECK(min_donor_load >= kAnyQueuedTask);
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
        const int queued = TaskSchedulerInternals::RunqueueLoad(inner(), cpu);
        if (queued == 0) {
          continue;
        }
        // `Load(cpu)`, without reading `RunqueueLoad()` again.
        const int load =
            queued + (TaskSchedulerInternals::IsCpuBusy(inner(), cpu) ? 1 : 0);
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
