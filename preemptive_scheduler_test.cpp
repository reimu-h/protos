#include "preemptive_scheduler.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
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
  AddressSpace* const fake_address_space =
      reinterpret_cast<AddressSpace*>(0x2000);
  FpuState* const fake_fpu_state = reinterpret_cast<FpuState*>(0x3000);
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

  EXPECT_CALL(*mock_inner, Block);
  layer.Block();

  EXPECT_CALL(*mock_inner, BlockUntil).WillOnce([](const int64_t deadline) {
    EXPECT_THAT(deadline, t::Eq(77));
  });
  layer.BlockUntil(77);

  EXPECT_CALL(*mock_inner, Wake).WillOnce([&](Task* const task) {
    EXPECT_THAT(task, t::Eq(fake_task_a));
  });
  layer.Wake(fake_task_a);

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

  EXPECT_CALL(*mock_inner, SetAddressSpace)
      .WillOnce([&](AddressSpace* const address_space) {
        EXPECT_THAT(address_space, t::Eq(fake_address_space));
      })
      .WillOnce([](AddressSpace* const address_space) {
        EXPECT_THAT(address_space, t::IsNull());
      });
  layer.SetAddressSpace(fake_address_space);
  layer.SetAddressSpace(nullptr);

  EXPECT_CALL(*mock_inner, SetFpuState)
      .WillOnce([&](FpuState* const state) {
        EXPECT_THAT(state, t::Eq(fake_fpu_state));
      })
      .WillOnce([](FpuState* const state) { EXPECT_THAT(state, t::IsNull()); });
  layer.SetFpuState(fake_fpu_state);
  layer.SetFpuState(nullptr);

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

  EXPECT_CALL(*mock_inner, WakeOnCpu)
      .WillOnce([&](Task* const task, const int target_cpu) {
        EXPECT_THAT(task, t::Eq(fake_task_b));
        EXPECT_THAT(target_cpu, t::Eq(2));
        return true;
      })
      .WillOnce([&](Task* const task, const int target_cpu) {
        EXPECT_THAT(task, t::Eq(fake_task_a));
        EXPECT_THAT(target_cpu, t::Eq(0));
        return false;
      });
  EXPECT_TRUE(TaskSchedulerInternals::WakeOnCpu(layer, fake_task_b, 2));
  EXPECT_FALSE(TaskSchedulerInternals::WakeOnCpu(layer, fake_task_a, 0));
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

template <typename Scheduler>
concept CanCallWakeOnCpuDirectly = requires(Scheduler& scheduler, Task* task) {
  scheduler.WakeOnCpu(task, 0);
};

static_assert(!CanCallInitDirectly<TaskScheduler>);
static_assert(!CanCallRunqueueLoadDirectly<TaskScheduler>);
static_assert(!CanCallWakeOnCpuDirectly<TaskScheduler>);
static_assert(!CanCallInitDirectly<ForwardingTaskScheduler>);
static_assert(!CanCallRunqueueLoadDirectly<ForwardingTaskScheduler>);
static_assert(!CanCallWakeOnCpuDirectly<ForwardingTaskScheduler>);
static_assert(CanCallInitDirectly<MockTaskScheduler>);
static_assert(CanCallRunqueueLoadDirectly<MockTaskScheduler>);
static_assert(CanCallWakeOnCpuDirectly<MockTaskScheduler>);

// ============================================================================
// Blocking: Block() and Wake() pass through, BlockUntil() (timed waits)
// ============================================================================

TEST(PreemptiveSchedulerTest,
     ForwardsBlockAndWakeWhateverThePreemptionAndInterruptState) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);
  Task* const fake_task = reinterpret_cast<Task*>(0x4001);
  EXPECT_CALL(*mock_inner, Wake(fake_task)).Times(3);
  EXPECT_CALL(*mock_inner, Block).Times(2);

  sched->Wake(fake_task);
  sched->Block();
  sched->SetPreemptEnabled(false);
  sched->Wake(fake_task);
  sched->Block();
  sched->SetPreemptEnabled(true);
  // As from an interrupt handler that holds a lock of its own.
  SetInterruptsEnabledForTest(false);
  {
    IrqSpinLock lock;
    const IrqSpinLockGuard guard(lock);
    sched->Wake(fake_task);
  }
  SetInterruptsEnabledForTest(true);
}

// Calls OnTimerInterrupt() as CPU `cpu`'s timer interrupt does, with CPU 0's
// tick count set to `cpu0_ticks` (idt.cpp counts the tick before the handler
// runs). The frame says that the interrupted code had interrupts disabled, so
// the tick may not preempt anything.
static void DeliverUnpreemptibleTick(TaskScheduler& sched,  //
                                     const int cpu,         //
                                     const int64_t cpu0_ticks) {
  SmpGetCpuLocal(0)->timer_ticks.store(cpu0_ticks, std::memory_order_relaxed);
  InterruptFrame frame = MakeInterruptFrame(kVectorApicTimer);
  frame.rflags = 0x2ULL;
  BindCpuLocal(SmpGetCpuLocal(cpu));
  TaskSchedulerInternals::OnTimerInterrupt(sched, &frame);
  BindCpuLocal(SmpGetCpuLocal(0));
}

TEST(PreemptiveSchedulerTest,
     BlockUntilEndsWithAWakeOnTheCpu0TickThatReachesTheDeadline) {
  SetupEnv(2);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);
  Task* const fake_task = reinterpret_cast<Task*>(0x4002);
  // Ticks end timed waits even while they may not preempt.
  sched->SetPreemptEnabled(false);
  SmpGetCpuLocal(0)->timer_ticks.store(100, std::memory_order_relaxed);
  int wakes = 0;
  int64_t woken_at_tick = -1;
  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly(t::Return(fake_task));
  EXPECT_CALL(*mock_inner, Wake).WillRepeatedly([&](Task* const task) {
    EXPECT_THAT(task, t::Eq(fake_task));
    ++wakes;
    woken_at_tick = SmpGetCpuLocal(0)->timer_ticks.load();
  });
  // Stands in for the inner Block(): the ticks arrive while the task is
  // blocked.
  EXPECT_CALL(*mock_inner, Block).WillOnce([&]() {
    for (int64_t tick = 101; tick <= 104; ++tick) {
      DeliverUnpreemptibleTick(*sched, 0, tick);
    }
    EXPECT_THAT(wakes, t::Eq(0));
    // Only CPU 0's ticks end timed waits.
    DeliverUnpreemptibleTick(*sched, 1, 105);
    EXPECT_THAT(wakes, t::Eq(0));
    DeliverUnpreemptibleTick(*sched, 0, 105);
    EXPECT_THAT(wakes, t::Eq(1));
  });

  sched->BlockUntil(105);
  EXPECT_THAT(wakes, t::Eq(1));
  EXPECT_THAT(woken_at_tick, t::Eq(105));

  // The waiter is gone: later ticks wake nobody.
  for (int64_t tick = 106; tick <= 108; ++tick) {
    DeliverUnpreemptibleTick(*sched, 0, tick);
  }
  EXPECT_THAT(wakes, t::Eq(1));
}

TEST(PreemptiveSchedulerTest,
     BlockUntilReturnsAtOnceAndBlocksNotIfTheDeadlineHasPassed) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);
  Task* const fake_task = reinterpret_cast<Task*>(0x4003);
  SmpGetCpuLocal(0)->timer_ticks.store(50, std::memory_order_relaxed);
  // No Block() and no Wake() expected.
  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly(t::Return(fake_task));
  sched->BlockUntil(50);
  sched->BlockUntil(10);
  DeliverUnpreemptibleTick(*sched, 0, 51);
}

TEST(PreemptiveSchedulerTest,
     BlockUntilEndedByAnotherWakeLeavesNoWaiterForTheTimer) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);
  Task* const fake_task = reinterpret_cast<Task*>(0x4004);
  SmpGetCpuLocal(0)->timer_ticks.store(200, std::memory_order_relaxed);
  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly(t::Return(fake_task));
  // Another Wake() ends the Block() before the deadline (the inner Block()
  // returns at once), so the timer must not wake the task later: its waiter,
  // on the stack of the returned BlockUntil() call, is gone. The StrictMock
  // fails on any Wake().
  EXPECT_CALL(*mock_inner, Block).Times(2);
  sched->BlockUntil(210);
  sched->BlockUntil(205);
  for (int64_t tick = 201; tick <= 215; ++tick) {
    DeliverUnpreemptibleTick(*sched, 0, tick);
  }
}

// Host threads bound to CPUs 1-3 stand in for three tasks in BlockUntil() with
// different deadlines; the inner Block() waits until the inner Wake() of the
// thread's task. CPU 0's ticks wake each task once, in deadline order, at its
// deadline.
TEST(PreemptiveSchedulerTest,
     ConcurrentTimedWaitersAreWokenOnceEachInDeadlineOrder) {
  constexpr int kNumCpus = 4;
  SetupEnv(kNumCpus);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);
  SmpGetCpuLocal(0)->timer_ticks.store(100, std::memory_order_relaxed);
  // The task of CPU `cpu`, and its deadline.
  const auto fake_task = [](const int cpu) {
    return reinterpret_cast<Task*>(0x5000 + cpu * 0x100);
  };
  const int64_t deadlines[kNumCpus] = {0, 110, 105, 120};
  std::atomic<int> blocked{0};
  std::atomic<bool> woken[kNumCpus] = {};
  std::atomic<int64_t> woken_at[kNumCpus] = {};
  std::atomic<int> wake_count{0};
  std::mutex order_mutex;
  std::vector<int> wake_order;

  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([&fake_task]() {
    return fake_task(CurrentCpuId());
  });
  EXPECT_CALL(*mock_inner, Wake).WillRepeatedly([&](Task* const task) {
    for (int cpu = 1; cpu < kNumCpus; ++cpu) {
      if (task == fake_task(cpu)) {
        woken_at[cpu].store(SmpGetCpuLocal(0)->timer_ticks.load());
        {
          const std::lock_guard<std::mutex> lock(order_mutex);
          wake_order.push_back(cpu);
        }
        wake_count.fetch_add(1);
        woken[cpu].store(true);
      }
    }
  });
  EXPECT_CALL(*mock_inner, Block).Times(kNumCpus - 1).WillRepeatedly([&]() {
    const int cpu = CurrentCpuId();
    blocked.fetch_add(1);
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (!woken[cpu].load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::yield();
    }
  });

  std::vector<std::thread> waiters;
  for (int cpu = 1; cpu < kNumCpus; ++cpu) {
    waiters.emplace_back([cpu, &sched, &deadlines]() {
      BindCpuLocal(SmpGetCpuLocal(cpu));
      SetInterruptsEnabledForTest(true);
      sched->BlockUntil(deadlines[cpu]);
      ResetCpuLocalForTest();
    });
  }
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (blocked.load() < kNumCpus - 1 &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  EXPECT_THAT(blocked.load(), t::Eq(kNumCpus - 1));
  for (int64_t tick = 101; tick <= 125; ++tick) {
    DeliverUnpreemptibleTick(*sched, 0, tick);
  }
  for (std::thread& waiter : waiters) {
    waiter.join();
  }

  EXPECT_THAT(wake_count.load(), t::Eq(kNumCpus - 1));
  EXPECT_THAT(wake_order, t::ElementsAre(2, 1, 3));
  for (int cpu = 1; cpu < kNumCpus; ++cpu) {
    EXPECT_THAT(woken_at[cpu].load(), t::Eq(deadlines[cpu])) << "cpu " << cpu;
  }
}

TEST(PreemptiveSchedulerDeathTest, BlockUntilMisuseFailsChecks) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::NiceMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);
  EXPECT_DEATH(
      {
        IrqSpinLock lock;
        const IrqSpinLockGuard guard(lock);
        sched->BlockUntil(1);
      },
      "Check failed: !HoldsAnyIrqSpinLock\\(\\)");
  EXPECT_DEATH(
      {
        SetInterruptsEnabledForTest(false);
        sched->BlockUntil(1);
      },
      "Check failed: AreInterruptsEnabled\\(\\)");
}

}  // namespace
}  // namespace protos
