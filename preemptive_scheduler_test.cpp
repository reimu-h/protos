#include "preemptive_scheduler.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "forwarding_task_scheduler.h"
#include "scheduler_test_support.h"
#include "task_scheduler_internals.h"

namespace protos {
namespace {

namespace t = ::testing;

// Returns the frame the CPU pushes when it interrupts code that runs with
// interrupts enabled (RFLAGS.IF set; RFLAGS bit 1 always reads as 1).
static InterruptFrame MakeInterruptFrame(const uint8_t vector) {
  InterruptFrame frame = {};
  frame.vector = vector;
  frame.rflags = kRflagsInterruptEnableBit | 0x2ULL;
  return frame;
}

TEST(PreemptiveSchedulerTest,
     DefaultPreemptStateTogglingAndInitResetsToEnabled) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);
  ASSERT_THAT(sched, t::NotNull());

  // Preemption is enabled by default upon construction.
  EXPECT_TRUE(sched->IsPreemptEnabled());

  // SetPreemptEnabled toggles state locally without calling inner.
  sched->SetPreemptEnabled(false);
  EXPECT_FALSE(sched->IsPreemptEnabled());

  sched->SetPreemptEnabled(true);
  EXPECT_TRUE(sched->IsPreemptEnabled());

  // Init() resets preempt_enabled_ back to true and delegates to inner->Init().
  sched->SetPreemptEnabled(false);
  EXPECT_FALSE(sched->IsPreemptEnabled());

  int init_calls = 0;
  EXPECT_CALL(*mock_inner, Init).WillOnce([&]() { ++init_calls; });

  TaskSchedulerInternals::Init(*sched);
  EXPECT_THAT(init_calls, t::Eq(1));
  EXPECT_TRUE(sched->IsPreemptEnabled());
}

TEST(PreemptiveSchedulerTest,
     ForwardsTimerInterruptAndWakeupIpiWhenAllSafetyConditionsHold) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  InterruptFrame timer_frame = MakeInterruptFrame(kVectorApicTimer);
  InterruptFrame ipi_frame = MakeInterruptFrame(kVectorWakeupIpi);
  InterruptFrame* observed_timer_frame = nullptr;
  InterruptFrame* observed_ipi_frame = nullptr;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce(
          [&](InterruptFrame* const frame) { observed_timer_frame = frame; });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce(
          [&](InterruptFrame* const frame) { observed_ipi_frame = frame; });

  TaskSchedulerInternals::OnTimerInterrupt(*sched, &timer_frame);
  EXPECT_THAT(observed_timer_frame, t::Eq(&timer_frame));

  TaskSchedulerInternals::OnWakeupIpi(*sched, &ipi_frame);
  EXPECT_THAT(observed_ipi_frame, t::Eq(&ipi_frame));
}

TEST(PreemptiveSchedulerTest,
     SuppressesInterruptsUnlessFrameShowsInterruptsWereEnabled) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  // StrictMock verifies that none of these reach the inner layer: without a
  // frame, or with RFLAGS.IF (bit 9) clear in the saved RFLAGS, the
  // interrupted context can't be preempted.
  TaskSchedulerInternals::OnTimerInterrupt(*sched, nullptr);
  TaskSchedulerInternals::OnWakeupIpi(*sched, nullptr);

  InterruptFrame cli_frame = MakeInterruptFrame(kVectorApicTimer);
  constexpr uint64_t kRflagsWithIfClear[] = {
      0x0ULL, 0x2ULL, 0x246ULL & ~kRflagsInterruptEnableBit};
  for (const uint64_t rflags : kRflagsWithIfClear) {
    cli_frame.rflags = rflags;
    TaskSchedulerInternals::OnTimerInterrupt(*sched, &cli_frame);
    TaskSchedulerInternals::OnWakeupIpi(*sched, &cli_frame);
  }

  // Setting IF (bit 9) in the saved RFLAGS allows forwarding.
  cli_frame.rflags |= kRflagsInterruptEnableBit;
  int forwarded = 0;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&cli_frame));
        ++forwarded;
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&cli_frame));
        ++forwarded;
      });

  TaskSchedulerInternals::OnTimerInterrupt(*sched, &cli_frame);
  TaskSchedulerInternals::OnWakeupIpi(*sched, &cli_frame);
  EXPECT_THAT(forwarded, t::Eq(2));
}

TEST(PreemptiveSchedulerTest,
     DecidesFromSavedRflagsAndIgnoresCurrentInterruptFlag) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  // Inside an interrupt gate the CPU has cleared IF, so the current interrupt
  // flag says nothing about the interrupted context. Simulate that: interrupts
  // are disabled now, but the frame shows they were enabled when the interrupt
  // arrived, so the interrupt is forwarded.
  SetInterruptsEnabledForTest(false);
  InterruptFrame frame = MakeInterruptFrame(kVectorApicTimer);
  int forwarded = 0;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++forwarded;
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++forwarded;
      });

  TaskSchedulerInternals::OnTimerInterrupt(*sched, &frame);
  TaskSchedulerInternals::OnWakeupIpi(*sched, &frame);
  EXPECT_THAT(forwarded, t::Eq(2));
  SetInterruptsEnabledForTest(true);
}

TEST(PreemptiveSchedulerTest, SuppressesInterruptsWhenPreemptDisabled) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  InterruptFrame frame = MakeInterruptFrame(kVectorApicTimer);

  sched->SetPreemptEnabled(false);
  EXPECT_FALSE(sched->IsPreemptEnabled());

  // StrictMock verifies zero calls to inner when preemption is disabled: the
  // inner layers don't even see the tick (no vruntime accounting either).
  TaskSchedulerInternals::OnTimerInterrupt(*sched, &frame);
  TaskSchedulerInternals::OnWakeupIpi(*sched, &frame);

  // Re-enabling preemption restores forwarding.
  sched->SetPreemptEnabled(true);
  int timer_calls = 0;
  int ipi_calls = 0;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++timer_calls;
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++ipi_calls;
      });

  TaskSchedulerInternals::OnTimerInterrupt(*sched, &frame);
  TaskSchedulerInternals::OnWakeupIpi(*sched, &frame);
  EXPECT_THAT(timer_calls, t::Eq(1));
  EXPECT_THAT(ipi_calls, t::Eq(1));
}

TEST(PreemptiveSchedulerTest, SuppressesInterruptsWhenCpuLocalIsNull) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  InterruptFrame frame = MakeInterruptFrame(kVectorApicTimer);

  // Unbind CpuLocal on the current host thread.
  ResetCpuLocalForTest();
  EXPECT_THAT(CurrentCpuOrNull(), t::IsNull());

  // StrictMock verifies zero calls to inner when CurrentCpuOrNull() == nullptr.
  TaskSchedulerInternals::OnTimerInterrupt(*sched, &frame);
  TaskSchedulerInternals::OnWakeupIpi(*sched, &frame);

  // Re-binding valid online CpuLocal restores forwarding.
  BindCpuLocal(&g_env.cpu_locals[0]);
  int forwarded = 0;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++forwarded;
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++forwarded;
      });

  TaskSchedulerInternals::OnTimerInterrupt(*sched, &frame);
  TaskSchedulerInternals::OnWakeupIpi(*sched, &frame);
  EXPECT_THAT(forwarded, t::Eq(2));
}

TEST(PreemptiveSchedulerTest, SuppressesInterruptsWhenCurrentCpuIsOffline) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  InterruptFrame frame = MakeInterruptFrame(kVectorApicTimer);

  g_env.cpu_locals[0].online = false;
  ASSERT_THAT(CurrentCpuOrNull(), t::NotNull());
  EXPECT_FALSE(CurrentCpu()->online);

  // StrictMock verifies zero calls to inner when CurrentCpu()->online == false.
  TaskSchedulerInternals::OnTimerInterrupt(*sched, &frame);
  TaskSchedulerInternals::OnWakeupIpi(*sched, &frame);

  // Marking CPU online restores forwarding.
  g_env.cpu_locals[0].online = true;
  int forwarded = 0;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++forwarded;
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++forwarded;
      });

  TaskSchedulerInternals::OnTimerInterrupt(*sched, &frame);
  TaskSchedulerInternals::OnWakeupIpi(*sched, &frame);
  EXPECT_THAT(forwarded, t::Eq(2));
}

TEST(PreemptiveSchedulerTest,
     SuppressesInterruptsWhenIrqSpinLockIsHeldOnCurrentCpu) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  InterruptFrame frame = MakeInterruptFrame(kVectorApicTimer);

  IrqSpinLock outer_lock;
  IrqSpinLock inner_lock;
  {
    const IrqSpinLockGuard outer_guard(outer_lock);
    TaskSchedulerInternals::OnTimerInterrupt(*sched, &frame);
    TaskSchedulerInternals::OnWakeupIpi(*sched, &frame);

    {
      const IrqSpinLockGuard inner_guard(inner_lock);
      TaskSchedulerInternals::OnTimerInterrupt(*sched, &frame);
      TaskSchedulerInternals::OnWakeupIpi(*sched, &frame);
    }

    // Outer lock is still held after inner guard unlocks.
    TaskSchedulerInternals::OnTimerInterrupt(*sched, &frame);
    TaskSchedulerInternals::OnWakeupIpi(*sched, &frame);
  }

  // Once all locks are released, forwarding succeeds.
  int forwarded = 0;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++forwarded;
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++forwarded;
      });

  TaskSchedulerInternals::OnTimerInterrupt(*sched, &frame);
  TaskSchedulerInternals::OnWakeupIpi(*sched, &frame);
  EXPECT_THAT(forwarded, t::Eq(2));
}

TEST(PreemptiveSchedulerTest,
     ConcurrentPreemptToggleAndInterruptDispatchUnderTsan) {
  constexpr int kNumThreads = 4;
  constexpr int kIterationsPerThread = 500;
  SetupEnv(kNumThreads);

  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  std::atomic<int64_t> timer_forwarded{0};
  std::atomic<int64_t> ipi_forwarded{0};
  std::atomic<bool> safety_violation{false};

  // Checks the conditions that the preemptive layer guarantees for every
  // forwarded interrupt.
  const auto check_forwarded = [&](const InterruptFrame* const frame) {
    const CpuLocal* const cpu = CurrentCpuOrNull();
    if (cpu == nullptr || !cpu->online || HoldsAnyIrqSpinLock() ||
        frame == nullptr || (frame->rflags & kRflagsInterruptEnableBit) == 0) {
      safety_violation.store(true, std::memory_order_relaxed);
    }
  };
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillRepeatedly([&](InterruptFrame* const frame) {
        check_forwarded(frame);
        timer_forwarded.fetch_add(1, std::memory_order_relaxed);
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillRepeatedly([&](InterruptFrame* const frame) {
        check_forwarded(frame);
        ipi_forwarded.fetch_add(1, std::memory_order_relaxed);
      });

  std::atomic<int> ready_threads{0};
  std::atomic<bool> start_flag{false};
  std::vector<std::thread> threads;
  threads.reserve(kNumThreads);

  for (int cpu = 0; cpu < kNumThreads; ++cpu) {
    threads.emplace_back([cpu, &sched, &ready_threads, &start_flag]() {
      BindCpuLocal(&g_env.cpu_locals[cpu]);
      SetInterruptsEnabledForTest(true);
      ready_threads.fetch_add(1, std::memory_order_acq_rel);

      while (!start_flag.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }

      InterruptFrame safe_frame = MakeInterruptFrame(kVectorApicTimer);
      InterruptFrame cli_frame = MakeInterruptFrame(kVectorApicTimer);
      cli_frame.rflags = 0x2ULL;

      IrqSpinLock local_lock;

      for (int iter = 0; iter < kIterationsPerThread; ++iter) {
        const bool enable = ((iter + cpu) & 3) != 0;
        sched->SetPreemptEnabled(enable);
        (void)sched->IsPreemptEnabled();

        TaskSchedulerInternals::OnTimerInterrupt(*sched, &safe_frame);
        TaskSchedulerInternals::OnWakeupIpi(*sched, &safe_frame);
        TaskSchedulerInternals::OnTimerInterrupt(*sched, &cli_frame);
        TaskSchedulerInternals::OnWakeupIpi(*sched, &cli_frame);

        {
          const IrqSpinLockGuard guard(local_lock);
          TaskSchedulerInternals::OnTimerInterrupt(*sched, &safe_frame);
          TaskSchedulerInternals::OnWakeupIpi(*sched, &safe_frame);
        }
      }

      ResetCpuLocalForTest();
    });
  }

  while (ready_threads.load(std::memory_order_acquire) < kNumThreads) {
    std::this_thread::yield();
  }
  start_flag.store(true, std::memory_order_release);

  for (std::thread& th : threads) {
    th.join();
  }

  BindCpuLocal(&g_env.cpu_locals[0]);
  SetInterruptsEnabledForTest(true);
  sched->SetPreemptEnabled(true);

  InterruptFrame final_frame = MakeInterruptFrame(kVectorApicTimer);
  TaskSchedulerInternals::OnTimerInterrupt(*sched, &final_frame);
  TaskSchedulerInternals::OnWakeupIpi(*sched, &final_frame);

  EXPECT_FALSE(safety_violation.load(std::memory_order_acquire));
  EXPECT_THAT(timer_forwarded.load(std::memory_order_acquire), t::Gt(0));
  EXPECT_THAT(ipi_forwarded.load(std::memory_order_acquire), t::Gt(0));
}

TEST(PreemptiveSchedulerDeathTest, NullInnerSchedulerTriggersDcheck) {
  SetupEnv(1);
  EXPECT_DEATH(CreatePreemptiveScheduler(nullptr), "Check failed");
}

// ForwardingTaskScheduler is header-only (forwarding_task_scheduler.h). It is
// tested here because the preemptive layer inherits most of its methods from
// it.
TEST(ForwardingTaskSchedulerTest,
     ForwardsEachCallToInnerOnceWithUnchangedArgumentsAndResult) {
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  ForwardingTaskScheduler layer(mock_inner);
  // Every expectation must be met by exactly one call, in the order below, and
  // the StrictMock fails the test on any other call to the inner layer.
  const t::InSequence in_sequence;

  Task* const fake_task_a = reinterpret_cast<Task*>(0x1001);
  Task* const fake_task_b = reinterpret_cast<Task*>(0x1002);
  Task* const fake_current = reinterpret_cast<Task*>(0x1003);
  int dummy_arg_a = 11;
  int dummy_arg_b = 22;
  InterruptFrame timer_frame = MakeInterruptFrame(kVectorApicTimer);
  InterruptFrame ipi_frame = MakeInterruptFrame(kVectorWakeupIpi);

  // Task API.
  EXPECT_CALL(*mock_inner, CreateTask)
      .WillOnce([&](const TaskFn entry, void* const arg, const int64_t weight) {
        EXPECT_THAT(entry, t::Eq(&NoopTask));
        EXPECT_THAT(arg, t::Eq(&dummy_arg_a));
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight * 2));
        return fake_task_a;
      });
  EXPECT_THAT(layer.CreateTask(NoopTask, &dummy_arg_a, kDefaultTaskWeight * 2),
              t::Eq(fake_task_a));

  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,    //
                    void* const arg,       //
                    const int64_t weight,  //
                    const int target_cpu) {
        EXPECT_THAT(entry, t::Eq(&NoopTask));
        EXPECT_THAT(arg, t::Eq(&dummy_arg_b));
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight / 2));
        EXPECT_THAT(target_cpu, t::Eq(3));
        return fake_task_b;
      });
  EXPECT_THAT(
      layer.CreateTaskOnCpu(NoopTask, &dummy_arg_b, kDefaultTaskWeight / 2, 3),
      t::Eq(fake_task_b));

  EXPECT_CALL(*mock_inner, Yield);
  layer.Yield();

  EXPECT_CALL(*mock_inner, Join).WillOnce([&](Task* const task) {
    EXPECT_THAT(task, t::Eq(fake_task_a));
  });
  layer.Join(fake_task_a);

  EXPECT_CALL(*mock_inner, Detach).WillOnce([&](Task* const task) {
    EXPECT_THAT(task, t::Eq(fake_task_b));
  });
  layer.Detach(fake_task_b);

  EXPECT_CALL(*mock_inner, ReapZombies)
      .WillOnce([]() { return 5; })
      .WillOnce([]() { return 0; });
  EXPECT_THAT(layer.ReapZombies(), t::Eq(5));
  EXPECT_THAT(layer.ReapZombies(), t::Eq(0));

  EXPECT_CALL(*mock_inner, SetPreemptEnabled)
      .WillOnce([](const bool enabled) { EXPECT_FALSE(enabled); })
      .WillOnce([](const bool enabled) { EXPECT_TRUE(enabled); });
  layer.SetPreemptEnabled(false);
  layer.SetPreemptEnabled(true);

  EXPECT_CALL(*mock_inner, IsPreemptEnabled)
      .WillOnce([]() { return false; })
      .WillOnce([]() { return true; });
  EXPECT_FALSE(layer.IsPreemptEnabled());
  EXPECT_TRUE(layer.IsPreemptEnabled());

  EXPECT_CALL(*mock_inner, CurrentTask).WillOnce([&]() {
    return fake_current;
  });
  EXPECT_THAT(layer.CurrentTask(), t::Eq(fake_current));

  // Scheduler internals.
  EXPECT_CALL(*mock_inner, Init);
  TaskSchedulerInternals::Init(layer);

  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&timer_frame));
      });
  TaskSchedulerInternals::OnTimerInterrupt(layer, &timer_frame);

  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&ipi_frame));
      });
  TaskSchedulerInternals::OnWakeupIpi(layer, &ipi_frame);

  EXPECT_CALL(*mock_inner, PollIdleCpu)
      .WillOnce([](const int cpu_index) {
        EXPECT_THAT(cpu_index, t::Eq(2));
        return true;
      })
      .WillOnce([](const int cpu_index) {
        EXPECT_THAT(cpu_index, t::Eq(1));
        return false;
      });
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(layer, 2));
  EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(layer, 1));

  EXPECT_CALL(*mock_inner, RunqueueLoad).WillOnce([](const int cpu_index) {
    EXPECT_THAT(cpu_index, t::Eq(3));
    return 7;
  });
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(layer, 3), t::Eq(7));

  EXPECT_CALL(*mock_inner, IsCpuBusy)
      .WillOnce([](const int cpu_index) {
        EXPECT_THAT(cpu_index, t::Eq(1));
        return true;
      })
      .WillOnce([](const int cpu_index) {
        EXPECT_THAT(cpu_index, t::Eq(2));
        return false;
      });
  EXPECT_TRUE(TaskSchedulerInternals::IsCpuBusy(layer, 1));
  EXPECT_FALSE(TaskSchedulerInternals::IsCpuBusy(layer, 2));

  EXPECT_CALL(*mock_inner, StealTask)
      .WillOnce([](const int dst_cpu, const int src_cpu) {
        EXPECT_THAT(dst_cpu, t::Eq(0));
        EXPECT_THAT(src_cpu, t::Eq(2));
        return true;
      })
      .WillOnce([](const int dst_cpu, const int src_cpu) {
        EXPECT_THAT(dst_cpu, t::Eq(1));
        EXPECT_THAT(src_cpu, t::Eq(3));
        return false;
      });
  EXPECT_TRUE(TaskSchedulerInternals::StealTask(layer, 0, 2));
  EXPECT_FALSE(TaskSchedulerInternals::StealTask(layer, 1, 3));
}

// The scheduler internals are private in `TaskScheduler` and protected in
// `ForwardingTaskScheduler`, so only `TaskSchedulerInternals` can call them on
// a scheduler. `MockTaskScheduler` mocks them in its public section, so the
// same calls compile on it: only the access level rejects them. The checks
// use concepts because an invalid expression in a requires-expression outside
// a template is a compile error rather than `false`.
template <typename Scheduler>
concept CanCallInitDirectly =
    requires(Scheduler& scheduler) { scheduler.Init(); };

template <typename Scheduler>
concept CanCallRunqueueLoadDirectly =
    requires(const Scheduler& scheduler) { scheduler.RunqueueLoad(0); };

static_assert(!CanCallInitDirectly<TaskScheduler>);
static_assert(!CanCallRunqueueLoadDirectly<TaskScheduler>);
static_assert(!CanCallInitDirectly<ForwardingTaskScheduler>);
static_assert(!CanCallRunqueueLoadDirectly<ForwardingTaskScheduler>);
static_assert(CanCallInitDirectly<MockTaskScheduler>);
static_assert(CanCallRunqueueLoadDirectly<MockTaskScheduler>);

}  // namespace
}  // namespace protos
