#include "work_stealing_scheduler.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "core_runqueue_scheduler.h"
#include "forwarding_task_scheduler.h"
#include "scheduler_test_support.h"
#include "task_scheduler_internals.h"

namespace protos {
namespace {

namespace t = ::testing;

TEST(WorkStealingSchedulerTest,
     CreateTaskSelectsMinimumLoadOnlineCpuBreaksTiesAndSendsRemoteWakeupIpi) {
  SetupEnv(4);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  int loads[4] = {4, 2, 0, 3};
  Task* const fake_task = reinterpret_cast<Task*>(0x1001);
  int dummy_arg = 42;
  int captured_cpu = -1;
  int64_t captured_weight = 0;

  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    EXPECT_THAT(cpu, t::AllOf(t::Ge(0), t::Lt(4)));
    return loads[cpu];
  });
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,    //
                    void* const arg,       //
                    const int64_t weight,  //
                    const int target_cpu) {
        EXPECT_THAT(entry, t::Eq(&NoopTask));
        EXPECT_THAT(arg, t::Eq(&dummy_arg));
        captured_weight = weight;
        captured_cpu = target_cpu;
        return fake_task;
      });

  // 1. CPU 2 has minimum load (0). Calling from CPU 0 places the task on remote
  //    CPU 2 and sends kVectorWakeupIpi (0x21) to CPU 2 only.
  EXPECT_THAT(sched->CreateTask(NoopTask, &dummy_arg, kDefaultTaskWeight * 2),
              t::Eq(fake_task));
  EXPECT_THAT(captured_cpu, t::Eq(2));
  EXPECT_THAT(captured_weight, t::Eq(kDefaultTaskWeight * 2));
  EXPECT_THAT(g_env.ipi_sent_count[2].load(), t::Eq(1));
  EXPECT_THAT(g_env.last_ipi_vector[2].load(), t::Eq(kVectorWakeupIpi));
  EXPECT_THAT(g_env.ipi_sent_count[0].load(), t::Eq(0));
  EXPECT_THAT(g_env.ipi_sent_count[1].load(), t::Eq(0));
  EXPECT_THAT(g_env.ipi_sent_count[3].load(), t::Eq(0));

  // 2. Tie-breaking between non-zero CPUs (loads {5, 1, 3, 1}): must select
  //    lowest cpu_id (CPU 1) and send wakeup IPI to remote CPU 1.
  loads[0] = 5;
  loads[1] = 1;
  loads[2] = 3;
  loads[3] = 1;
  captured_cpu = -1;
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,    //
                    void* const arg,       //
                    const int64_t weight,  //
                    const int target_cpu) {
        EXPECT_THAT(entry, t::Eq(&NoopTask));
        EXPECT_THAT(arg, t::IsNull());
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight));
        captured_cpu = target_cpu;
        return fake_task;
      });

  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(fake_task));
  EXPECT_THAT(captured_cpu, t::Eq(1));
  EXPECT_THAT(g_env.ipi_sent_count[1].load(), t::Eq(1));
  EXPECT_THAT(g_env.last_ipi_vector[1].load(), t::Eq(kVectorWakeupIpi));

  // 3. The calling CPU (CPU 0) counts as one task busier than its queue,
  //    because it is running the caller: with loads {1, 2, 1, 1} it ties with
  //    CPU 1 at 2, and CPU 2 (the lowest of the two CPUs at 1) wins.
  loads[0] = 1;
  loads[1] = 2;
  loads[2] = 1;
  loads[3] = 1;
  captured_cpu = -1;
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn /*entry*/,    //
                    void* const /*arg*/,       //
                    const int64_t /*weight*/,  //
                    const int target_cpu) {
        captured_cpu = target_cpu;
        return fake_task;
      });

  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(fake_task));
  EXPECT_THAT(captured_cpu, t::Eq(2));
  EXPECT_THAT(g_env.ipi_sent_count[2].load(), t::Eq(2));

  // 4. Only when every other CPU is busier does the task stay on the calling
  //    CPU, without a self-IPI (loads {0, 1, 1, 1}: CPU 0 counts as 1 and wins
  //    the tie).
  loads[0] = 0;
  loads[1] = 1;
  loads[2] = 1;
  loads[3] = 1;
  captured_cpu = -1;
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn /*entry*/,    //
                    void* const /*arg*/,       //
                    const int64_t /*weight*/,  //
                    const int target_cpu) {
        captured_cpu = target_cpu;
        return fake_task;
      });

  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(fake_task));
  EXPECT_THAT(captured_cpu, t::Eq(0));
  EXPECT_THAT(g_env.ipi_sent_count[0].load(), t::Eq(0));
}

TEST(WorkStealingSchedulerTest,
     CreateTaskSkipsOfflineCpusAndAvoidsSelfIpiOnNonZeroLocalCpu) {
  SetupEnv(4);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  // Mark CPU 1 offline and CPU 2 as having a null CpuLocal descriptor.
  g_env.cpu_locals[1].online = false;
  g_env.null_cpu_local[2] = true;
  // Bind current thread to CPU 3.
  BindCpuLocal(&g_env.cpu_locals[3]);

  int loads[4] = {4, 0, 0, 2};
  std::vector<int> queried_cpus;
  Task* const fake_task = reinterpret_cast<Task*>(0x1002);
  int captured_cpu = -1;

  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    queried_cpus.push_back(cpu);
    return loads[cpu];
  });
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn /*entry*/,    //
                    void* const /*arg*/,       //
                    const int64_t /*weight*/,  //
                    const int target_cpu) {
        captured_cpu = target_cpu;
        return fake_task;
      });

  // CPUs 1 and 2 are offline so only CPUs 0 and 3 are queried; CPU 3 is still
  // less loaded (2, plus 1 for the running caller, < 4) and is the current CPU,
  // so no IPI is sent.
  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(fake_task));
  EXPECT_THAT(queried_cpus, t::ElementsAre(0, 3));
  EXPECT_THAT(captured_cpu, t::Eq(3));
  for (int i = 0; i < 4; ++i) {
    EXPECT_THAT(g_env.ipi_sent_count[i].load(), t::Eq(0));
  }

  // Also verify that if CPU 0 itself is offline despite having load 0, CPU 0 is
  // skipped and online CPU 3 is selected.
  g_env.cpu_locals[0].online = false;
  loads[0] = 0;
  loads[3] = 5;
  queried_cpus.clear();
  captured_cpu = -1;
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn /*entry*/,    //
                    void* const /*arg*/,       //
                    const int64_t /*weight*/,  //
                    const int target_cpu) {
        captured_cpu = target_cpu;
        return fake_task;
      });
  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(fake_task));
  EXPECT_THAT(queried_cpus, t::ElementsAre(3));
  EXPECT_THAT(captured_cpu, t::Eq(3));
}

TEST(WorkStealingSchedulerTest,
     CreateTaskOnCpuDelegatesAndSendsWakeupIpiOnlyWhenRemoteOrUnbound) {
  SetupEnv(3);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  Task* const fake_task = reinterpret_cast<Task*>(0x2001);
  int dummy_arg = 7;

  // 1. Target is local CPU (CPU 0 while bound to CPU 0): no wakeup IPI.
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,    //
                    void* const arg,       //
                    const int64_t weight,  //
                    const int target_cpu) {
        EXPECT_THAT(entry, t::Eq(&NoopTask));
        EXPECT_THAT(arg, t::Eq(&dummy_arg));
        EXPECT_THAT(weight, t::Eq(kMinTaskWeight));
        EXPECT_THAT(target_cpu, t::Eq(0));
        return fake_task;
      });
  EXPECT_THAT(sched->CreateTaskOnCpu(NoopTask, &dummy_arg, kMinTaskWeight, 0),
              t::Eq(fake_task));
  EXPECT_THAT(g_env.ipi_sent_count[0].load(), t::Eq(0));

  // 2. Target is remote CPU (CPU 2 while bound to CPU 0): sends wakeup IPI to
  //    CPU 2.
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,    //
                    void* const arg,       //
                    const int64_t weight,  //
                    const int target_cpu) {
        EXPECT_THAT(entry, t::Eq(&NoopTask));
        EXPECT_THAT(arg, t::Eq(&dummy_arg));
        EXPECT_THAT(weight, t::Eq(kMaxTaskWeight));
        EXPECT_THAT(target_cpu, t::Eq(2));
        return fake_task;
      });
  EXPECT_THAT(sched->CreateTaskOnCpu(NoopTask, &dummy_arg, kMaxTaskWeight, 2),
              t::Eq(fake_task));
  EXPECT_THAT(g_env.ipi_sent_count[2].load(), t::Eq(1));
  EXPECT_THAT(g_env.last_ipi_vector[2].load(), t::Eq(kVectorWakeupIpi));

  // 3. When CurrentCpuOrNull() == nullptr (unbound host thread), target_cpu 0
  //    is treated as remote and receives kVectorWakeupIpi (0x21).
  ResetCpuLocalForTest();
  ASSERT_THAT(CurrentCpuOrNull(), t::IsNull());
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,    //
                    void* const arg,       //
                    const int64_t weight,  //
                    const int target_cpu) {
        EXPECT_THAT(entry, t::Eq(&NoopTask));
        EXPECT_THAT(arg, t::IsNull());
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight));
        EXPECT_THAT(target_cpu, t::Eq(0));
        return fake_task;
      });
  EXPECT_THAT(sched->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 0),
              t::Eq(fake_task));
  EXPECT_THAT(g_env.ipi_sent_count[0].load(), t::Eq(1));
  EXPECT_THAT(g_env.last_ipi_vector[0].load(), t::Eq(kVectorWakeupIpi));
}

TEST(WorkStealingSchedulerTest,
     YieldStealsFromBusiestOnlineDonorWhenLocalRunqueueEmptyAndSkipsWhenBusy) {
  SetupEnv(4);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  // Case 1: Local runqueue load > 0 -> delegates directly to inner->Yield()
  // without calling StealTask.
  int yield_calls = 0;
  EXPECT_CALL(*mock_inner, RunqueueLoad).WillOnce([&](const int cpu) {
    EXPECT_THAT(cpu, t::Eq(0));
    return 2;
  });
  EXPECT_CALL(*mock_inner, Yield).WillOnce([&]() { ++yield_calls; });

  sched->Yield();
  EXPECT_THAT(yield_calls, t::Eq(1));

  // Case 2: Local runqueue load == 0 -> attempts to steal from busiest online
  // remote donor before delegating to inner->Yield().
  // Mark CPU 3 offline despite high load so CPU 2 (load 4) is selected.
  g_env.cpu_locals[3].online = false;
  const int loads[4] = {0, 1, 4, 9};
  std::vector<std::string> call_order;

  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    return loads[cpu];
  });
  EXPECT_CALL(*mock_inner, StealTask)
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        EXPECT_FALSE(AreInterruptsEnabled());
        EXPECT_THAT(dst_cpu, t::Eq(0));
        EXPECT_THAT(src_cpu, t::Eq(2));
        call_order.push_back("StealTask(0,2)");
        return true;
      });
  EXPECT_CALL(*mock_inner, Yield).WillOnce([&]() {
    EXPECT_TRUE(AreInterruptsEnabled());
    call_order.push_back("Yield");
  });

  sched->Yield();
  EXPECT_THAT(call_order, t::ElementsAre("StealTask(0,2)", "Yield"));

  // Case 3: Yield() takes even a lone queued task from a donor (CPU 1, load
  // 1): the yielding task is offering its CPU, and the bootstrap task on CPU 0
  // pulls in work this way while it waits in Join().
  const int single_loads[4] = {0, 1, 0, 0};
  call_order.clear();
  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    return single_loads[cpu];
  });
  EXPECT_CALL(*mock_inner, StealTask)
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        EXPECT_THAT(dst_cpu, t::Eq(0));
        EXPECT_THAT(src_cpu, t::Eq(1));
        call_order.push_back("StealTask(0,1)");
        return true;
      });
  EXPECT_CALL(*mock_inner, Yield).WillOnce([&]() {
    call_order.push_back("Yield");
  });

  sched->Yield();
  EXPECT_THAT(call_order, t::ElementsAre("StealTask(0,1)", "Yield"));
}

TEST(WorkStealingSchedulerTest,
     PollIdleCpuIgnoresOtherCpusAndDelegatesDirectlyWhenLocalLoadNonZero) {
  SetupEnv(4);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  // 1. A cpu_index other than the calling CPU (CPU 0 here), including an
  //    out-of-range one, returns false without touching mock_inner: a task
  //    stolen for a remote CPU could sit on that halted CPU until its next
  //    timer tick. So do an unbound and an offline calling CPU.
  EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*sched, -1));
  EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*sched, 2));
  EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*sched, 4));
  EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*sched, 99));
  ResetCpuLocalForTest();
  EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*sched, 0));
  BindCpuLocal(SmpGetCpuLocal(2));
  g_env.cpu_locals[2].online = false;
  EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*sched, 2));
  g_env.cpu_locals[2].online = true;

  // 2. On CPU 2, when inner->RunqueueLoad(2) > 0, delegates directly to
  //    inner->PollIdleCpu(2) without calling StealTask. Local interrupts are
  //    disabled throughout and restored afterwards.
  EXPECT_CALL(*mock_inner, RunqueueLoad).WillOnce([&](const int cpu) {
    EXPECT_THAT(cpu, t::Eq(2));
    EXPECT_FALSE(AreInterruptsEnabled());
    return 3;
  });
  EXPECT_CALL(*mock_inner, PollIdleCpu).WillOnce([&](const int cpu) {
    EXPECT_THAT(cpu, t::Eq(2));
    EXPECT_FALSE(AreInterruptsEnabled());
    return true;
  });
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*sched, 2));
  EXPECT_TRUE(AreInterruptsEnabled());
}

TEST(
    WorkStealingSchedulerTest,
    PollIdleCpuTriesDonorsInDescendingLoadOrderAndPollsOnlyAfterStealSucceeds) {
  SetupEnv(5);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  // Mark CPU 4 offline (even with load 10, it must be ignored).
  g_env.cpu_locals[4].online = false;
  BindCpuLocal(SmpGetCpuLocal(2));

  // Destination is CPU 2 (load 0). Online remote donors:
  // CPU 1 (load 5, busiest), CPU 3 (load 3, 2nd busiest), CPU 0 (load 1, 3rd).
  int loads[5] = {1, 5, 0, 3, 10};
  std::vector<std::pair<int, int>> steal_attempts;
  int polled_cpu = -1;

  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    return loads[cpu];
  });
  EXPECT_CALL(*mock_inner, StealTask)
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        steal_attempts.emplace_back(dst_cpu, src_cpu);
        return false;
      })
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        steal_attempts.emplace_back(dst_cpu, src_cpu);
        loads[src_cpu] -= 1;
        loads[dst_cpu] += 1;
        return true;
      });
  EXPECT_CALL(*mock_inner, PollIdleCpu).WillOnce([&](const int cpu_index) {
    polled_cpu = cpu_index;
    return true;
  });

  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*sched, 2));
  EXPECT_THAT(steal_attempts,
              t::ElementsAre(std::make_pair(2, 1), std::make_pair(2, 3)));
  EXPECT_THAT(polled_cpu, t::Eq(2));

  // When all online remote donors have load > 0 but every StealTask call fails,
  // PollIdleCpu must NOT call inner->PollIdleCpu and must return false.
  loads[0] = 2;
  loads[1] = 4;
  loads[2] = 0;
  loads[3] = 2;
  steal_attempts.clear();

  EXPECT_CALL(*mock_inner, StealTask)
      .Times(3)
      .WillRepeatedly([&](const int dst_cpu, const int src_cpu) {
        steal_attempts.emplace_back(dst_cpu, src_cpu);
        return false;
      });

  EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*sched, 2));
  // Tie between CPU 0 (load 2) and CPU 3 (load 2) breaks toward lower cpu_id 0.
  EXPECT_THAT(steal_attempts,
              t::ElementsAre(std::make_pair(2, 1), std::make_pair(2, 0),
                             std::make_pair(2, 3)));

  // When all online remote donors have load 0, PollIdleCpu returns false
  // without calling StealTask or inner->PollIdleCpu.
  loads[0] = 0;
  loads[1] = 0;
  loads[2] = 0;
  loads[3] = 0;
  EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*sched, 2));
}

TEST(WorkStealingSchedulerTest, StealTriesAtMostThreeDonorsBusiestFirst) {
  SetupEnv(8);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  // Seven donors with work, but every StealTask() fails: only the three
  // busiest (CPUs 2, 4, 6) are tried, so a steal costs O(cpu_count) load
  // reads rather than O(cpu_count^2).
  const int loads[8] = {0, 1, 7, 2, 6, 3, 5, 4};
  std::vector<int> tried_donors;
  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    return loads[cpu];
  });
  EXPECT_CALL(*mock_inner, StealTask)
      .WillRepeatedly([&](const int dst_cpu, const int src_cpu) {
        EXPECT_THAT(dst_cpu, t::Eq(0));
        tried_donors.push_back(src_cpu);
        return false;
      });

  EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*sched, 0));
  EXPECT_THAT(tried_donors, t::ElementsAre(2, 4, 6));
}

TEST(WorkStealingSchedulerTest,
     InterruptsStealOnlyFromDonorsWithTwoQueuedTasksBeforeForwarding) {
  SetupEnv(3);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  InterruptFrame timer_frame = {};
  timer_frame.vector = kVectorApicTimer;
  InterruptFrame ipi_frame = {};
  ipi_frame.vector = kVectorWakeupIpi;

  // 1. CurrentCpuOrNull() is non-null, online, and local RunqueueLoad == 0,
  //    and CPU 1 has 3 queued tasks: steal from it (with local interrupts
  //    disabled) before forwarding the frame to inner.
  const int idle_loads[3] = {0, 3, 1};
  std::vector<std::string> sequence;

  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    return idle_loads[cpu];
  });
  EXPECT_CALL(*mock_inner, StealTask)
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        EXPECT_FALSE(AreInterruptsEnabled());
        EXPECT_THAT(dst_cpu, t::Eq(0));
        EXPECT_THAT(src_cpu, t::Eq(1));
        sequence.push_back("TimerSteal");
        return true;
      });
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&timer_frame));
        sequence.push_back("OnTimerInterrupt");
      });

  TaskSchedulerInternals::OnTimerInterrupt(*sched, &timer_frame);
  EXPECT_THAT(sequence, t::ElementsAre("TimerSteal", "OnTimerInterrupt"));

  sequence.clear();
  EXPECT_CALL(*mock_inner, StealTask)
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        EXPECT_FALSE(AreInterruptsEnabled());
        EXPECT_THAT(dst_cpu, t::Eq(0));
        EXPECT_THAT(src_cpu, t::Eq(1));
        sequence.push_back("IpiSteal");
        return true;
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&ipi_frame));
        sequence.push_back("OnWakeupIpi");
      });

  TaskSchedulerInternals::OnWakeupIpi(*sched, &ipi_frame);
  EXPECT_THAT(sequence, t::ElementsAre("IpiSteal", "OnWakeupIpi"));

  // 2. When no donor has more than one queued task, forwards without
  //    stealing: the interrupted CPU may be busy with a task that
  //    RunqueueLoad() does not count, and moving a lone task to it would only
  //    move the imbalance. An idle CPU takes such a task in PollIdleCpu().
  const int single_loads[3] = {0, 1, 1};
  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    return single_loads[cpu];
  });
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&timer_frame));
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&ipi_frame));
      });

  TaskSchedulerInternals::OnTimerInterrupt(*sched, &timer_frame);
  TaskSchedulerInternals::OnWakeupIpi(*sched, &ipi_frame);

  // 3. When local RunqueueLoad > 0: forwards directly without stealing.
  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    EXPECT_THAT(cpu, t::Eq(0));
    return 1;
  });
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&timer_frame));
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&ipi_frame));
      });

  TaskSchedulerInternals::OnTimerInterrupt(*sched, &timer_frame);
  TaskSchedulerInternals::OnWakeupIpi(*sched, &ipi_frame);

  // 4. When current CPU is offline: forwards directly without querying load or
  //    stealing.
  g_env.cpu_locals[0].online = false;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&timer_frame));
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&ipi_frame));
      });

  TaskSchedulerInternals::OnTimerInterrupt(*sched, &timer_frame);
  TaskSchedulerInternals::OnWakeupIpi(*sched, &ipi_frame);
  g_env.cpu_locals[0].online = true;

  // 5. When CurrentCpuOrNull() == nullptr: forwards directly without querying
  //    load or stealing.
  ResetCpuLocalForTest();
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&timer_frame));
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&ipi_frame));
      });

  TaskSchedulerInternals::OnTimerInterrupt(*sched, &timer_frame);
  TaskSchedulerInternals::OnWakeupIpi(*sched, &ipi_frame);
}

TEST(WorkStealingSchedulerDeathTest, PreconditionViolationsTriggerDcheck) {
  SetupEnv(2);

  // 1. Null inner scheduler passed to CreateWorkStealingScheduler.
  EXPECT_DEATH(CreateWorkStealingScheduler(nullptr),
               "Check failed: inner_ != nullptr");

  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  // 2. Null entry function in CreateTask and CreateTaskOnCpu.
  constexpr const char* kNullEntry = "Check failed: entry != nullptr";
  EXPECT_DEATH(sched->CreateTask(nullptr, nullptr, kDefaultTaskWeight),
               kNullEntry);
  EXPECT_DEATH(sched->CreateTaskOnCpu(nullptr, nullptr, kDefaultTaskWeight, 0),
               kNullEntry);

  // 3. Out-of-bounds weight in CreateTask and CreateTaskOnCpu.
  constexpr const char* kBadWeight =
      "Check failed: weight >= kMinTaskWeight && weight <= kMaxTaskWeight";
  EXPECT_DEATH(sched->CreateTask(NoopTask, nullptr, 0), kBadWeight);
  EXPECT_DEATH(sched->CreateTask(NoopTask, nullptr, -10), kBadWeight);
  EXPECT_DEATH(sched->CreateTask(NoopTask, nullptr, kMaxTaskWeight + 1),
               kBadWeight);
  EXPECT_DEATH(sched->CreateTaskOnCpu(NoopTask, nullptr, 0, 0), kBadWeight);
  EXPECT_DEATH(sched->CreateTaskOnCpu(NoopTask, nullptr, -1, 0), kBadWeight);
  EXPECT_DEATH(sched->CreateTaskOnCpu(NoopTask, nullptr, kMaxTaskWeight + 1, 0),
               kBadWeight);

  // 4. Negative or >= SmpCpuCount() target_cpu in CreateTaskOnCpu.
  constexpr const char* kBadTargetCpu =
      "Check failed: target_cpu >= 0 && target_cpu < cpu_count";
  EXPECT_DEATH(
      sched->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, -1),
      kBadTargetCpu);
  EXPECT_DEATH(sched->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 2),
               kBadTargetCpu);

  // 5. Offline target_cpu (either local->online == false or SmpGetCpuLocal ==
  //    nullptr) in CreateTaskOnCpu.
  constexpr const char* kOfflineTargetCpu =
      "Check failed: IsCpuOnline\\(target_cpu, cpu_count\\)";
  g_env.cpu_locals[1].online = false;
  EXPECT_DEATH(sched->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 1),
               kOfflineTargetCpu);
  g_env.cpu_locals[1].online = true;
  g_env.null_cpu_local[1] = true;
  EXPECT_DEATH(sched->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 1),
               kOfflineTargetCpu);
}

TEST(WorkStealingSchedulerTest,
     InterruptAndIdleEntryPointsAreSafeBeforeInitWhenWrappingTheCore) {
  SetupEnv(4);
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(CreateCoreRunqueueScheduler());

  InterruptFrame timer_frame = {};
  timer_frame.vector = kVectorApicTimer;
  timer_frame.rflags = kRflagsInterruptEnableBit;
  InterruptFrame ipi_frame = {};
  ipi_frame.vector = kVectorWakeupIpi;
  ipi_frame.rflags = kRflagsInterruptEnableBit;

  // The core accepts these before Init(); stealing must not change that.
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*sched, 0), t::Eq(0));
  for (int cpu = 0; cpu < 4; ++cpu) {
    BindCpuLocal(SmpGetCpuLocal(cpu));
    EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*sched, cpu));
    TaskSchedulerInternals::OnTimerInterrupt(*sched, &timer_frame);
    TaskSchedulerInternals::OnWakeupIpi(*sched, &ipi_frame);
  }
  BindCpuLocal(SmpGetCpuLocal(0));

  TaskSchedulerInternals::Init(*sched);
  EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*sched, 0));
}

// Forwards every call to the wrapped layer and counts the StealTask() calls
// that take a task from CPU 0.
class ForwardingSpyTaskScheduler final : public ForwardingTaskScheduler {
 public:
  using ForwardingTaskScheduler::ForwardingTaskScheduler;

  int StealFromCpu0Calls() const {
    return steal_from_cpu0_calls_.load(std::memory_order_acquire);
  }

 private:
  bool StealTask(const int dst_cpu, const int src_cpu) override {
    if (src_cpu == 0) {
      steal_from_cpu0_calls_.fetch_add(1, std::memory_order_acq_rel);
    }
    return TaskSchedulerInternals::StealTask(inner(), dst_cpu, src_cpu);
  }

  std::atomic<int> steal_from_cpu0_calls_{0};
};

struct BootstrapLoadProbeCtx {
  TaskScheduler* sched = nullptr;
  ForwardingSpyTaskScheduler* spy = nullptr;
  int cpu0_load_with_only_bootstrap_queued = -1;
  int ap_polls_dispatched_with_only_bootstrap_queued = 0;
  int steal_calls_with_only_bootstrap_queued = -1;
  int cpu0_load_with_helper_queued = -1;
  bool ap1_poll_with_helper_queued = false;
  int steal_calls_with_helper_queued = -1;
  int helper_first_cpu = -1;
  int helper_resumed_cpu = -1;
};

// Runs on CPU 0 while the bootstrap task waits in Join(), so the bootstrap
// task is the only task queued on CPU 0, and stands in for idle APs 1..3.
static void BootstrapOnlyQueuedProbeWorker(void* const raw_arg) {
  BootstrapLoadProbeCtx* const ctx =
      static_cast<BootstrapLoadProbeCtx*>(raw_arg);
  // The bootstrap task cannot be stolen, so RunqueueLoad(0) excludes it.
  ctx->cpu0_load_with_only_bootstrap_queued =
      TaskSchedulerInternals::RunqueueLoad(*ctx->sched, 0);

  // Idle APs polling for work must see RunqueueLoad(0) == 0 and must not call
  // StealTask(ap, 0).
  for (int ap = 1; ap < 4; ++ap) {
    BindCpuLocal(SmpGetCpuLocal(ap));
    if (TaskSchedulerInternals::PollIdleCpu(*ctx->sched, ap)) {
      ++ctx->ap_polls_dispatched_with_only_bootstrap_queued;
    }
  }
  BindCpuLocal(SmpGetCpuLocal(0));
  ctx->steal_calls_with_only_bootstrap_queued = ctx->spy->StealFromCpu0Calls();
}

// Created with weight 1, so the Yield() in its first slice charges it about
// 1M of vruntime and leaves it queued behind the bootstrap task.
static void QueuedBehindBootstrapHelperWorker(void* const raw_arg) {
  BootstrapLoadProbeCtx* const ctx =
      static_cast<BootstrapLoadProbeCtx*>(raw_arg);
  ctx->helper_first_cpu = CurrentCpuId();
  ctx->sched->Yield();
  ctx->helper_resumed_cpu = CurrentCpuId();
}

// Runs on CPU 0 while the bootstrap task (waiting in Join()) is the leftmost
// task in CPU 0's runqueue with the helper behind it, and stands in for idle
// AP 1.
static void BootstrapAtHeadProbeWorker(void* const raw_arg) {
  BootstrapLoadProbeCtx* const ctx =
      static_cast<BootstrapLoadProbeCtx*>(raw_arg);
  ctx->cpu0_load_with_helper_queued =
      TaskSchedulerInternals::RunqueueLoad(*ctx->sched, 0);
  BindCpuLocal(SmpGetCpuLocal(1));
  ctx->ap1_poll_with_helper_queued =
      TaskSchedulerInternals::PollIdleCpu(*ctx->sched, 1);
  BindCpuLocal(SmpGetCpuLocal(0));
  ctx->steal_calls_with_helper_queued = ctx->spy->StealFromCpu0Calls();
}

static void CountRunsWorker(void* const raw_arg) {
  ++*static_cast<int*>(raw_arg);
}

TEST(WorkStealingSchedulerTest,
     IdleApsStealOnlyStealableTasksFromCpu0AndNeverTheBootstrapTask) {
  SetupEnv(4);
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  const auto spy = std::make_shared<ForwardingSpyTaskScheduler>(core);
  const std::shared_ptr<TaskScheduler> sched = CreateWorkStealingScheduler(spy);
  TaskSchedulerInternals::Init(*sched);
  Task* const bootstrap = sched->CurrentTask();

  BootstrapLoadProbeCtx ctx;
  ctx.sched = sched.get();
  ctx.spy = spy.get();

  // 1. With only the bootstrap task queued on CPU 0, idle APs find nothing to
  //    steal there and do not even try.
  Task* const first_probe =
      sched->CreateTaskOnCpu(BootstrapOnlyQueuedProbeWorker,  //
                             &ctx,                            //
                             kDefaultTaskWeight,              //
                             0);
  ASSERT_THAT(first_probe, t::NotNull());
  sched->Join(first_probe);
  EXPECT_THAT(ctx.cpu0_load_with_only_bootstrap_queued, t::Eq(0));
  EXPECT_THAT(ctx.ap_polls_dispatched_with_only_bootstrap_queued, t::Eq(0));
  EXPECT_THAT(ctx.steal_calls_with_only_bootstrap_queued, t::Eq(0));

  // 2. Run the helper's first slice; its Yield() switches back to the
  //    bootstrap task and leaves the helper queued on CPU 0.
  Task* const helper =
      sched->CreateTaskOnCpu(QueuedBehindBootstrapHelperWorker,  //
                             &ctx,                               //
                             kMinTaskWeight,                     //
                             0);
  ASSERT_THAT(helper, t::NotNull());
  sched->Yield();
  ASSERT_THAT(ctx.helper_first_cpu, t::Eq(0));
  ASSERT_THAT(ctx.helper_resumed_cpu, t::Eq(-1));

  // 3. An idle AP 1 now steals the helper, which sits behind the bootstrap task
  //    at the head of CPU 0's runqueue, and runs it.
  Task* const second_probe =
      sched->CreateTaskOnCpu(BootstrapAtHeadProbeWorker,  //
                             &ctx,                        //
                             kDefaultTaskWeight,          //
                             0);
  ASSERT_THAT(second_probe, t::NotNull());
  sched->Join(second_probe);
  EXPECT_THAT(ctx.cpu0_load_with_helper_queued, t::Eq(1));
  EXPECT_TRUE(ctx.ap1_poll_with_helper_queued);
  EXPECT_THAT(ctx.steal_calls_with_helper_queued, t::Eq(1));
  EXPECT_THAT(ctx.helper_resumed_cpu, t::Eq(1));
  EXPECT_THAT(sched->CurrentTask(), t::Eq(bootstrap));
  sched->Join(helper);

  // 4. CPU 1's idle context was last switched out by the second probe, which
  //    has exited since. When the bootstrap task stands in for it, switching
  //    back must resume the bootstrap task's own context (and, under TSan, its
  //    own fiber), not the probe's.
  int runs = 0;
  Task* const ap1_task =
      sched->CreateTaskOnCpu(CountRunsWorker, &runs, kDefaultTaskWeight, 1);
  ASSERT_THAT(ap1_task, t::NotNull());
  BindCpuLocal(SmpGetCpuLocal(1));
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*sched, 1));
  BindCpuLocal(SmpGetCpuLocal(0));
  EXPECT_THAT(runs, t::Eq(1));
  sched->Join(ap1_task);
  EXPECT_THAT(sched->CurrentTask(), t::Eq(bootstrap));
}

// ============================================================================
// Multi-threaded work stealing on top of the core
// ============================================================================

struct MtStealCtx {
  TaskScheduler* sched = nullptr;
  std::atomic<int> completed_tasks{0};
  std::atomic<int64_t> remote_slices{0};
  std::atomic<uint32_t> cpu_mask{0};
};

static void MtStealWorker(void* const raw_arg) {
  MtStealCtx* const ctx = static_cast<MtStealCtx*>(raw_arg);
  InterruptFrame timer_frame = {};
  timer_frame.vector = kVectorApicTimer;
  timer_frame.rflags = kRflagsInterruptEnableBit;

  for (int round = 0; round < 8; ++round) {
    const int cpu = CurrentCpuId();
    ctx->cpu_mask.fetch_or(1u << static_cast<uint32_t>(cpu),
                           std::memory_order_acq_rel);
    if (cpu != 0) {
      ctx->remote_slices.fetch_add(1, std::memory_order_acq_rel);
    }
    if ((round % 3) == 2) {
      TaskSchedulerInternals::OnTimerInterrupt(*ctx->sched, &timer_frame);
    }
    ctx->sched->Yield();
  }
  ctx->completed_tasks.fetch_add(1, std::memory_order_acq_rel);
}

TEST(WorkStealingSchedulerTest,
     ConcurrentIdleCpusRunPlacedAndStolenTasksThroughWorkStealingLayer) {
  constexpr int kNumCpus = 4;
  constexpr int kNumTasks = 32;
  SetupEnv(kNumCpus);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(CreateCoreRunqueueScheduler());
  TaskSchedulerInternals::Init(*sched);
  Task* const bootstrap = sched->CurrentTask();

  MtStealCtx ctx;
  ctx.sched = sched.get();

  // Each AP thread plays ApKernelEntry(): it delivers the wakeup IPIs sent to
  // its CPU and polls for work with interrupts disabled.
  std::atomic<int> ready_aps{0};
  std::atomic<bool> stop_aps{false};
  std::vector<std::thread> ap_threads;
  ap_threads.reserve(kNumCpus - 1);
  for (int cpu = 1; cpu < kNumCpus; ++cpu) {
    ap_threads.emplace_back([cpu, &sched, &ready_aps, &stop_aps]() {
      BindCpuLocal(SmpGetCpuLocal(cpu));
      SetInterruptsEnabledForTest(true);
      ready_aps.fetch_add(1, std::memory_order_acq_rel);

      int64_t handled_ipis = 0;
      while (!stop_aps.load(std::memory_order_acquire)) {
        const int64_t sent_ipis =
            g_env.ipi_sent_count[cpu].load(std::memory_order_acquire);
        if (sent_ipis > handled_ipis) {
          handled_ipis = sent_ipis;
          InterruptFrame ipi_frame = {};
          ipi_frame.vector = kVectorWakeupIpi;
          ipi_frame.rflags = kRflagsInterruptEnableBit;
          SetInterruptsEnabledForTest(false);
          TaskSchedulerInternals::OnWakeupIpi(*sched, &ipi_frame);
          SetInterruptsEnabledForTest(true);
        }
        SetInterruptsEnabledForTest(false);
        const bool dispatched =
            TaskSchedulerInternals::PollIdleCpu(*sched, cpu);
        SetInterruptsEnabledForTest(true);
        if (!dispatched) {
          std::this_thread::yield();
        }
      }
      ResetCpuLocalForTest();
    });
  }
  while (ready_aps.load(std::memory_order_acquire) < (kNumCpus - 1)) {
    std::this_thread::yield();
  }

  // CreateTask() spreads the tasks over all CPUs (waking the remote ones)
  // while the APs are already running and stealing.
  std::vector<Task*> tasks;
  tasks.reserve(kNumTasks);
  for (int i = 0; i < kNumTasks; ++i) {
    Task* const task = sched->CreateTask(MtStealWorker,  //
                                         &ctx,           //
                                         kDefaultTaskWeight + (i % 4) * 128);
    EXPECT_THAT(task, t::NotNull());
    if (task != nullptr) {
      tasks.push_back(task);
    }
  }
  for (Task* const task : tasks) {
    sched->Join(task);
  }

  stop_aps.store(true, std::memory_order_release);
  for (std::thread& th : ap_threads) {
    th.join();
  }

  EXPECT_THAT(ctx.completed_tasks.load(), t::Eq(kNumTasks));
  EXPECT_THAT(ctx.remote_slices.load(), t::Gt(0));
  EXPECT_THAT(ctx.cpu_mask.load() & ~1u, t::Ne(0u));
  EXPECT_THAT(CurrentCpuId(), t::Eq(0));
  EXPECT_THAT(sched->CurrentTask(), t::Eq(bootstrap));
  for (int c = 0; c < kNumCpus; ++c) {
    EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*sched, c), t::Eq(0));
  }
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

}  // namespace
}  // namespace protos
