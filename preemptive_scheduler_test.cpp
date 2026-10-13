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

#include "scheduler_test_support.h"

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

  sched->Init();
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

  sched->OnTimerInterrupt(&timer_frame);
  EXPECT_THAT(observed_timer_frame, t::Eq(&timer_frame));

  sched->OnWakeupIpi(&ipi_frame);
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
  sched->OnTimerInterrupt(nullptr);
  sched->OnWakeupIpi(nullptr);

  InterruptFrame cli_frame = MakeInterruptFrame(kVectorApicTimer);
  constexpr uint64_t kRflagsWithIfClear[] = {
      0x0ULL, 0x2ULL, 0x246ULL & ~kRflagsInterruptEnableBit};
  for (const uint64_t rflags : kRflagsWithIfClear) {
    cli_frame.rflags = rflags;
    sched->OnTimerInterrupt(&cli_frame);
    sched->OnWakeupIpi(&cli_frame);
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

  sched->OnTimerInterrupt(&cli_frame);
  sched->OnWakeupIpi(&cli_frame);
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

  sched->OnTimerInterrupt(&frame);
  sched->OnWakeupIpi(&frame);
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
  sched->OnTimerInterrupt(&frame);
  sched->OnWakeupIpi(&frame);

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

  sched->OnTimerInterrupt(&frame);
  sched->OnWakeupIpi(&frame);
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
  sched->OnTimerInterrupt(&frame);
  sched->OnWakeupIpi(&frame);

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

  sched->OnTimerInterrupt(&frame);
  sched->OnWakeupIpi(&frame);
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
  sched->OnTimerInterrupt(&frame);
  sched->OnWakeupIpi(&frame);

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

  sched->OnTimerInterrupt(&frame);
  sched->OnWakeupIpi(&frame);
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
    sched->OnTimerInterrupt(&frame);
    sched->OnWakeupIpi(&frame);

    {
      const IrqSpinLockGuard inner_guard(inner_lock);
      sched->OnTimerInterrupt(&frame);
      sched->OnWakeupIpi(&frame);
    }

    // Outer lock is still held after inner guard unlocks.
    sched->OnTimerInterrupt(&frame);
    sched->OnWakeupIpi(&frame);
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

  sched->OnTimerInterrupt(&frame);
  sched->OnWakeupIpi(&frame);
  EXPECT_THAT(forwarded, t::Eq(2));
}

TEST(PreemptiveSchedulerTest,
     ForwardsAllTaskManagementAndSchedulingOperationsDirectlyToInner) {
  SetupEnv(4);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  Task* const fake_task_a = reinterpret_cast<Task*>(0x1001);
  Task* const fake_task_b = reinterpret_cast<Task*>(0x1002);
  Task* const fake_current = reinterpret_cast<Task*>(0x1003);
  int dummy_arg_a = 11;
  int dummy_arg_b = 22;

  // 1. CreateTask
  EXPECT_CALL(*mock_inner, CreateTask)
      .WillOnce([&](const TaskFn entry, void* const arg, const int64_t weight) {
        EXPECT_THAT(entry, t::Eq(&NoopTask));
        EXPECT_THAT(arg, t::Eq(&dummy_arg_a));
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight * 2));
        return fake_task_a;
      });
  EXPECT_THAT(sched->CreateTask(NoopTask, &dummy_arg_a, kDefaultTaskWeight * 2),
              t::Eq(fake_task_a));

  // 2. CreateTaskOnCpu
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
      sched->CreateTaskOnCpu(NoopTask, &dummy_arg_b, kDefaultTaskWeight / 2, 3),
      t::Eq(fake_task_b));

  // 3. Yield
  int yield_calls = 0;
  EXPECT_CALL(*mock_inner, Yield).WillOnce([&]() { ++yield_calls; });
  sched->Yield();
  EXPECT_THAT(yield_calls, t::Eq(1));

  // 4. Join
  Task* joined_task = nullptr;
  EXPECT_CALL(*mock_inner, Join).WillOnce([&](Task* const task) {
    joined_task = task;
  });
  sched->Join(fake_task_a);
  EXPECT_THAT(joined_task, t::Eq(fake_task_a));

  // 5. Detach
  Task* detached_task = nullptr;
  EXPECT_CALL(*mock_inner, Detach).WillOnce([&](Task* const task) {
    detached_task = task;
  });
  sched->Detach(fake_task_b);
  EXPECT_THAT(detached_task, t::Eq(fake_task_b));

  // 6. ReapZombies
  EXPECT_CALL(*mock_inner, ReapZombies).WillOnce([&]() { return 5; });
  EXPECT_THAT(sched->ReapZombies(), t::Eq(5));

  // 7. PollIdleCpu
  EXPECT_CALL(*mock_inner, PollIdleCpu)
      .WillOnce([&](const int cpu_index) {
        EXPECT_THAT(cpu_index, t::Eq(2));
        return true;
      })
      .WillOnce([&](const int cpu_index) {
        EXPECT_THAT(cpu_index, t::Eq(1));
        return false;
      });
  EXPECT_TRUE(sched->PollIdleCpu(2));
  EXPECT_FALSE(sched->PollIdleCpu(1));

  // 8. RunqueueLoad
  EXPECT_CALL(*mock_inner, RunqueueLoad).WillOnce([&](const int cpu_index) {
    EXPECT_THAT(cpu_index, t::Eq(3));
    return 7;
  });
  EXPECT_THAT(sched->RunqueueLoad(3), t::Eq(7));

  // 9. StealTask
  EXPECT_CALL(*mock_inner, StealTask)
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        EXPECT_THAT(dst_cpu, t::Eq(0));
        EXPECT_THAT(src_cpu, t::Eq(2));
        return true;
      })
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        EXPECT_THAT(dst_cpu, t::Eq(1));
        EXPECT_THAT(src_cpu, t::Eq(3));
        return false;
      });
  EXPECT_TRUE(sched->StealTask(0, 2));
  EXPECT_FALSE(sched->StealTask(1, 3));

  // 10. CurrentTask
  EXPECT_CALL(*mock_inner, CurrentTask).WillOnce([&]() {
    return fake_current;
  });
  EXPECT_THAT(sched->CurrentTask(), t::Eq(fake_current));
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

        sched->OnTimerInterrupt(&safe_frame);
        sched->OnWakeupIpi(&safe_frame);
        sched->OnTimerInterrupt(&cli_frame);
        sched->OnWakeupIpi(&cli_frame);

        {
          const IrqSpinLockGuard guard(local_lock);
          sched->OnTimerInterrupt(&safe_frame);
          sched->OnWakeupIpi(&safe_frame);
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
  sched->OnTimerInterrupt(&final_frame);
  sched->OnWakeupIpi(&final_frame);

  EXPECT_FALSE(safety_violation.load(std::memory_order_acquire));
  EXPECT_THAT(timer_forwarded.load(std::memory_order_acquire), t::Gt(0));
  EXPECT_THAT(ipi_forwarded.load(std::memory_order_acquire), t::Gt(0));
}

TEST(PreemptiveSchedulerDeathTest, NullInnerSchedulerTriggersDcheck) {
  SetupEnv(1);
  EXPECT_DEATH(CreatePreemptiveScheduler(nullptr), "Check failed");
}

}  // namespace
}  // namespace protos
