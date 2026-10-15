#ifndef PROTOS_CPU_CLOCK_H_
#define PROTOS_CPU_CLOCK_H_

#include <atomic>
#include <cstdint>

#include "check.h"
#include "interrupts.h"
#include "smp.h"

namespace protos {

// What a task can tell about the CPU that it runs on at one point: which CPU
// it is, how many context switches the CPU has made so far, and how many
// timer interrupts have interrupted the task so far (`CpuLocal`).
struct CpuClockSample {
  int cpu = -1;
  // How many timer interrupts have interrupted the calling task so far, on
  // whichever CPUs it ran (`CpuLocal::task_timer_ticks`).
  int64_t task_ticks = 0;
  int64_t context_switches = 0;
};

// Returns the sample of the calling CPU. Reads it with interrupts disabled, so
// that all of it comes from one CPU and the calling task, and then restores
// the interrupt flag. Requires a bound `CpuLocal` (`DCHECK`).
inline CpuClockSample ReadCpuClock() {
  const ScopedIrqDisable irq_disable;
  const CpuLocal* const cpu = CurrentCpuOrNullIrqsOff();
  DCHECK(cpu != nullptr);
  CpuClockSample sample;
  sample.cpu = cpu->cpu_id;
  sample.task_ticks = cpu->task_timer_ticks.load(std::memory_order_relaxed);
  sample.context_switches =
      cpu->context_switches.load(std::memory_order_relaxed);
  return sample;
}

// Measures, in timer interrupts, how long a task ran, from samples that the
// task takes now and then (`ReadCpuClock()`): every timer interrupt that
// interrupted the task between two samples counts, on whichever CPU, also the
// one after which the scheduler switched it out. So the count grows whenever
// the task runs during a timer interrupt, however many tasks share its CPU.
// It also counts the intervals in which the task was switched out: those
// whose samples come from two CPUs, or from one CPU that made a context switch
// in between.
class TickMeter {
 public:
  // Starts measuring at `start`.
  explicit TickMeter(const CpuClockSample& start)
      : start_cpu_(start.cpu), last_(start) {}

  // Adds the interval from the previous sample to `sample`, a later sample of
  // the same task.
  void Add(const CpuClockSample& sample) {
    DCHECK(sample.task_ticks >= last_.task_ticks);
    ticks_ += sample.task_ticks - last_.task_ticks;
    if (sample.cpu != last_.cpu ||
        sample.context_switches != last_.context_switches) {
      ++switch_outs_;
    }
    last_ = sample;
  }

  // The timer interrupts that interrupted the task so far.
  int64_t ticks() const { return ticks_; }
  // The number of intervals in which the task was switched out.
  int64_t switch_outs() const { return switch_outs_; }
  // The CPU of the first sample.
  int start_cpu() const { return start_cpu_; }
  // Whether the latest sample comes from another CPU than the first one.
  bool migrated() const { return last_.cpu != start_cpu_; }

 private:
  const int start_cpu_;
  CpuClockSample last_;
  int64_t ticks_ = 0;
  int64_t switch_outs_ = 0;
};

}  // namespace protos

#endif  // PROTOS_CPU_CLOCK_H_
