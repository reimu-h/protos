#include "cpu_clock.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>

#include "interrupts.h"
#include "smp.h"

namespace protos {
namespace {

namespace t = ::testing;

// Returns a sample of CPU `cpu` taken by a task that `task_ticks` timer
// interrupts have interrupted so far, after `context_switches` context
// switches of the CPU.
static CpuClockSample Sample(const int cpu,             //
                             const int64_t task_ticks,  //
                             const int64_t context_switches) {
  CpuClockSample sample;
  sample.cpu = cpu;
  sample.task_ticks = task_ticks;
  sample.context_switches = context_switches;
  return sample;
}

TEST(TickMeterTest, CountsTheTicksWhileTheTaskKeepsRunningOnOneCpu) {
  TickMeter meter(Sample(1, 100, 7));
  EXPECT_THAT(meter.start_cpu(), t::Eq(1));
  EXPECT_THAT(meter.ticks(), t::Eq(0));
  meter.Add(Sample(1, 103, 7));
  meter.Add(Sample(1, 103, 7));
  meter.Add(Sample(1, 110, 7));
  EXPECT_THAT(meter.ticks(), t::Eq(10));
  EXPECT_THAT(meter.switch_outs(), t::Eq(0));
  EXPECT_FALSE(meter.migrated());
}

TEST(TickMeterTest, CountsTheTicksOfAnIntervalInWhichTheTaskWasSwitchedOut) {
  TickMeter meter(Sample(1, 100, 7));
  // Switched out and back in: the task's own count says how many of the
  // CPU's timer interrupts interrupted it, the last one before the switch
  // included.
  meter.Add(Sample(1, 103, 9));
  EXPECT_THAT(meter.ticks(), t::Eq(3));
  EXPECT_THAT(meter.switch_outs(), t::Eq(1));
  EXPECT_FALSE(meter.migrated());
}

TEST(TickMeterTest, KeepsCountingOnAnotherCpuAndTracksWhereTheTaskEnded) {
  TickMeter meter(Sample(0, 500, 3));
  // Moved to CPU 2, even if its switch count happens to be the same.
  meter.Add(Sample(2, 502, 3));
  EXPECT_TRUE(meter.migrated());
  EXPECT_THAT(meter.ticks(), t::Eq(2));
  EXPECT_THAT(meter.switch_outs(), t::Eq(1));
  meter.Add(Sample(2, 505, 3));
  EXPECT_THAT(meter.ticks(), t::Eq(5));
  // Back on CPU 0.
  meter.Add(Sample(0, 506, 4));
  EXPECT_FALSE(meter.migrated());
  EXPECT_THAT(meter.switch_outs(), t::Eq(2));
  EXPECT_THAT(meter.ticks(), t::Eq(6));
  EXPECT_THAT(meter.start_cpu(), t::Eq(0));
}

TEST(TickMeterTest, MakesProgressWhenEveryTickSwitchesTheTaskOut) {
  // Two tasks whose virtual runtimes are close take turns at every tick: each
  // interval of the task contains a context switch (two, there and back) and
  // the one tick that interrupted the task before it.
  TickMeter meter(Sample(1, 100, 7));
  for (int i = 1; i <= 100; ++i) {
    meter.Add(Sample(1, 100 + i, 7 + 2 * i));
  }
  EXPECT_THAT(meter.ticks(), t::Eq(100));
  EXPECT_THAT(meter.switch_outs(), t::Eq(100));
}

TEST(CpuClockTest, ReadsTheCallingCpuAndRestoresTheInterruptFlag) {
  static CpuLocal cpu;
  cpu.self = &cpu;
  cpu.cpu_id = 3;
  cpu.timer_ticks.store(1000, std::memory_order_relaxed);
  cpu.task_timer_ticks.store(42, std::memory_order_relaxed);
  cpu.context_switches.store(5, std::memory_order_relaxed);
  BindCpuLocal(&cpu);
  for (const bool enabled : {true, false}) {
    SetInterruptsEnabledForTest(enabled);
    const CpuClockSample sample = ReadCpuClock();
    EXPECT_THAT(sample.cpu, t::Eq(3));
    // The running task's count, not the CPU's.
    EXPECT_THAT(sample.task_ticks, t::Eq(42));
    EXPECT_THAT(sample.context_switches, t::Eq(5));
    EXPECT_THAT(AreInterruptsEnabled(), t::Eq(enabled));
  }
  SetInterruptsEnabledForTest(true);
  ResetCpuLocalForTest();
}

TEST(TickMeterDeathTest, TicksThatGoBackTriggerDcheck) {
  TickMeter meter(Sample(1, 100, 7));
  EXPECT_DEATH(meter.Add(Sample(1, 99, 7)), "Check failed");
  // Also on another CPU: the task's count goes with it.
  EXPECT_DEATH(meter.Add(Sample(2, 99, 9)), "Check failed");
}

}  // namespace
}  // namespace protos
