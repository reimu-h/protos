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
     CreateTaskSelectsLeastLoadedOnlineCpuBreaksTiesAndSendsRemoteWakeupIpi) {
  SetupEnv(4);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  // A CPU's load is its RunqueueLoad() plus 1 if IsCpuBusy(). The calling CPU
  // 0 is busy running the caller.
  int queued[4] = {4, 2, 0, 3};
  bool busy[4] = {true, false, false, false};
  Task* const fake_task = reinterpret_cast<Task*>(0x1001);
  int dummy_arg = 42;
  int captured_cpu = -1;
  int64_t captured_weight = 0;

  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    EXPECT_THAT(cpu, t::AllOf(t::Ge(0), t::Lt(4)));
    return queued[cpu];
  });
  EXPECT_CALL(*mock_inner, IsCpuBusy).WillRepeatedly([&](const int cpu) {
    EXPECT_THAT(cpu, t::AllOf(t::Ge(0), t::Lt(4)));
    return busy[cpu];
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

  // 1. CPU 2 has the smallest load (0, loads {5, 2, 0, 3}). Calling from CPU 0
  //    places the task on remote CPU 2 and sends kVectorWakeupIpi (0x21) to
  //    CPU 2 only.
  EXPECT_THAT(sched->CreateTask(NoopTask, &dummy_arg, kDefaultTaskWeight * 2),
              t::Eq(fake_task));
  EXPECT_THAT(captured_cpu, t::Eq(2));
  EXPECT_THAT(captured_weight, t::Eq(kDefaultTaskWeight * 2));
  EXPECT_THAT(g_env.ipi_sent_count[2].load(), t::Eq(1));
  EXPECT_THAT(g_env.last_ipi_vector[2].load(), t::Eq(kVectorWakeupIpi));
  EXPECT_THAT(g_env.ipi_sent_count[0].load(), t::Eq(0));
  EXPECT_THAT(g_env.ipi_sent_count[1].load(), t::Eq(0));
  EXPECT_THAT(g_env.ipi_sent_count[3].load(), t::Eq(0));

  // 2. Tie-breaking between non-zero loads ({6, 1, 3, 1}): must select the
  //    lowest cpu_id (CPU 1) and send a wakeup IPI to remote CPU 1.
  queued[0] = 5;
  queued[1] = 1;
  queued[2] = 3;
  queued[3] = 1;
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

  // 3. With nothing queued anywhere, a busy CPU is more loaded than an idle
  //    one: CPUs 0, 1 and 3 run tasks, so the task goes to idle CPU 2 rather
  //    than to CPU 1, the lowest CPU with nothing queued.
  queued[0] = 0;
  queued[1] = 0;
  queued[2] = 0;
  queued[3] = 0;
  busy[1] = true;
  busy[3] = true;
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

  // 4. Only when no other CPU is less loaded does the task stay on the
  //    calling CPU, without a self-IPI (loads {1, 1, 1, 1}: CPU 0 runs the
  //    caller, CPUs 1 and 2 are idle with one queued task each, and CPU 3 is
  //    busy with nothing queued).
  queued[1] = 1;
  queued[2] = 1;
  busy[1] = false;
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

  // 5. Ties go to the calling CPU before the lowest CPU index: called from CPU
  //    2, which runs the caller with nothing queued, with loads {1, 1, 1, 1},
  //    the task stays on CPU 2, without an IPI.
  BindCpuLocal(SmpGetCpuLocal(2));
  queued[2] = 0;
  busy[2] = true;
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
  EXPECT_THAT(g_env.ipi_sent_count[0].load(), t::Eq(0));
  EXPECT_THAT(g_env.ipi_sent_count[2].load(), t::Eq(2));
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

  int queued[4] = {4, 0, 0, 2};
  std::vector<int> queried_cpus;
  Task* const fake_task = reinterpret_cast<Task*>(0x1002);
  int captured_cpu = -1;

  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    queried_cpus.push_back(cpu);
    return queued[cpu];
  });
  EXPECT_CALL(*mock_inner, IsCpuBusy).WillRepeatedly([&](const int cpu) {
    EXPECT_THAT(cpu, t::AnyOf(t::Eq(0), t::Eq(3)));
    return true;
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
  // less loaded (2 queued plus the running caller, 3 < 5) and is the current
  // CPU, so no IPI is sent.
  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(fake_task));
  EXPECT_THAT(queried_cpus, t::ElementsAre(0, 3));
  EXPECT_THAT(captured_cpu, t::Eq(3));
  for (int i = 0; i < 4; ++i) {
    EXPECT_THAT(g_env.ipi_sent_count[i].load(), t::Eq(0));
  }

  // Also verify that if CPU 0 itself is offline despite having load 1, CPU 0
  // is skipped and online CPU 3 is selected.
  g_env.cpu_locals[0].online = false;
  queued[0] = 0;
  queued[3] = 5;
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

  // Case 2: Local runqueue load == 0 -> attempts to steal from the busiest
  // online remote donor before delegating to inner->Yield().
  // Mark CPU 3 offline despite high load so CPU 2 (load 4) is selected over
  // CPU 1 (load 2: one queued task plus a running one).
  g_env.cpu_locals[3].online = false;
  int queued[4] = {0, 1, 4, 9};
  bool busy[4] = {true, true, false, true};
  std::vector<std::string> call_order;

  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    return queued[cpu];
  });
  EXPECT_CALL(*mock_inner, IsCpuBusy).WillRepeatedly([&](const int cpu) {
    return busy[cpu];
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

  // Case 3: Donors are ranked by load, so of two CPUs with two queued tasks
  // each, the one that is also running a task (CPU 2, load 3) goes first.
  queued[1] = 2;
  queued[2] = 2;
  busy[1] = false;
  busy[2] = true;
  call_order.clear();
  EXPECT_CALL(*mock_inner, StealTask)
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        EXPECT_THAT(dst_cpu, t::Eq(0));
        EXPECT_THAT(src_cpu, t::Eq(2));
        call_order.push_back("StealTask(0,2)");
        return true;
      });
  EXPECT_CALL(*mock_inner, Yield).WillOnce([&]() {
    call_order.push_back("Yield");
  });

  sched->Yield();
  EXPECT_THAT(call_order, t::ElementsAre("StealTask(0,2)", "Yield"));

  // Case 4: Yield() takes even a lone queued task from an idle donor (CPU 1,
  // load 1): the yielding task is offering its CPU, and the bootstrap task on
  // CPU 0 pulls in work this way while it waits in Join(). CPUs that are busy
  // with nothing queued (CPU 2) have nothing to steal.
  queued[1] = 1;
  queued[2] = 0;
  call_order.clear();
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

  // Case 5: With nothing queued anywhere, Yield() delegates without stealing.
  queued[1] = 0;
  call_order.clear();
  EXPECT_CALL(*mock_inner, Yield).WillOnce([&]() {
    call_order.push_back("Yield");
  });

  sched->Yield();
  EXPECT_THAT(call_order, t::ElementsAre("Yield"));
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

  // Destination is CPU 2 (load 0). Online remote donors, ranked by queued
  // tasks plus 1 if busy: CPU 1 (load 5, busiest), CPU 3 (3 queued and busy:
  // load 4), CPU 0 (load 2).
  int queued[5] = {1, 5, 0, 3, 10};
  bool busy[5] = {true, false, false, true, true};
  std::vector<std::pair<int, int>> steal_attempts;
  int polled_cpu = -1;

  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    return queued[cpu];
  });
  EXPECT_CALL(*mock_inner, IsCpuBusy).WillRepeatedly([&](const int cpu) {
    EXPECT_THAT(cpu, t::Ne(4));
    return busy[cpu];
  });
  EXPECT_CALL(*mock_inner, StealTask)
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        steal_attempts.emplace_back(dst_cpu, src_cpu);
        return false;
      })
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        steal_attempts.emplace_back(dst_cpu, src_cpu);
        queued[src_cpu] -= 1;
        queued[dst_cpu] += 1;
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

  // When all online remote donors have queued tasks but every StealTask call
  // fails, PollIdleCpu must NOT call inner->PollIdleCpu and must return false.
  // Loads {2, 4, -, 3}: the busy CPU 3 now goes before CPU 0, which is idle
  // with the same number of queued tasks.
  queued[0] = 2;
  queued[1] = 4;
  queued[2] = 0;
  queued[3] = 2;
  busy[0] = false;
  steal_attempts.clear();

  EXPECT_CALL(*mock_inner, StealTask)
      .Times(3)
      .WillRepeatedly([&](const int dst_cpu, const int src_cpu) {
        steal_attempts.emplace_back(dst_cpu, src_cpu);
        return false;
      });

  EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*sched, 2));
  EXPECT_THAT(steal_attempts,
              t::ElementsAre(std::make_pair(2, 1), std::make_pair(2, 3),
                             std::make_pair(2, 0)));

  // When no online remote donor has a queued task, PollIdleCpu returns false
  // without calling StealTask or inner->PollIdleCpu, even if they are busy.
  queued[0] = 0;
  queued[1] = 0;
  queued[2] = 0;
  queued[3] = 0;
  busy[0] = true;
  busy[1] = true;
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
  const int queued[8] = {0, 1, 7, 2, 6, 3, 5, 4};
  std::vector<int> tried_donors;
  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    return queued[cpu];
  });
  EXPECT_CALL(*mock_inner, IsCpuBusy).WillRepeatedly(t::Return(false));
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
     InterruptsOnIdleCpuTakeAnyQueuedTaskAndOnBusyCpuOnlyFromTwoTasksBusier) {
  SetupEnv(3);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  InterruptFrame timer_frame = {};
  timer_frame.vector = kVectorApicTimer;
  InterruptFrame ipi_frame = {};
  ipi_frame.vector = kVectorWakeupIpi;

  // 1. The interrupted CPU 0 is idle with nothing queued (load 0). Like
  //    PollIdleCpu(), it takes even a lone queued task: CPU 1 (one queued task,
  //    idle) is the only CPU with something to steal, while CPU 2 is busy with
  //    nothing queued. The steal happens with local interrupts disabled,
  //    before the frame is forwarded to inner.
  int queued[3] = {0, 1, 0};
  bool busy[3] = {false, false, true};
  std::vector<std::string> sequence;

  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    return queued[cpu];
  });
  EXPECT_CALL(*mock_inner, IsCpuBusy).WillRepeatedly([&](const int cpu) {
    return busy[cpu];
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

  // 2. The interrupted CPU 0 is busy with nothing queued (load 1), and no
  //    other CPU is more than one task busier: CPU 1 runs a task and has one
  //    queued (load 2), and CPU 2 is idle with two queued (load 2). Taking a
  //    task from either would only swap the loads, so both interrupts are
  //    forwarded without stealing.
  queued[1] = 1;
  queued[2] = 2;
  busy[0] = true;
  busy[1] = true;
  busy[2] = false;
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

  // 3. Once CPU 1 has two queued tasks besides its running one (load 3), it is
  //    two tasks busier than CPU 0, and both interrupts steal from it.
  queued[1] = 2;
  sequence.clear();
  EXPECT_CALL(*mock_inner, StealTask)
      .Times(2)
      .WillRepeatedly([&](const int dst_cpu, const int src_cpu) {
        EXPECT_THAT(dst_cpu, t::Eq(0));
        EXPECT_THAT(src_cpu, t::Eq(1));
        sequence.push_back("Steal");
        return true;
      });
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const /*frame*/) {
        sequence.push_back("OnTimerInterrupt");
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const /*frame*/) {
        sequence.push_back("OnWakeupIpi");
      });

  TaskSchedulerInternals::OnTimerInterrupt(*sched, &timer_frame);
  TaskSchedulerInternals::OnWakeupIpi(*sched, &ipi_frame);
  EXPECT_THAT(sequence, t::ElementsAre("Steal", "OnTimerInterrupt", "Steal",
                                       "OnWakeupIpi"));

  // 4. When local RunqueueLoad > 0: forwards directly without stealing.
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

  // 5. When current CPU is offline: forwards directly without querying load or
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

  // 6. When CurrentCpuOrNull() == nullptr: forwards directly without querying
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
  EXPECT_FALSE(TaskSchedulerInternals::IsCpuBusy(*sched, 0));
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

// Returns the RunqueueLoad() of CPUs 0 .. cpu_count - 1.
static std::vector<int> QueuedTasksPerCpu(const TaskScheduler& sched,
                                          const int cpu_count) {
  std::vector<int> queued;
  for (int cpu = 0; cpu < cpu_count; ++cpu) {
    queued.push_back(TaskSchedulerInternals::RunqueueLoad(sched, cpu));
  }
  return queued;
}

struct SpreadCtx {
  TaskScheduler* sched = nullptr;
  Task* children[4] = {};
  int child_runs = 0;
  // QueuedTasksPerCpu() after each CreateTask() of the spawner.
  std::vector<std::vector<int>> queued_after_create;
};

// Runs on CPU 1 while CPU 0 runs the bootstrap task and CPUs 2 and 3 are
// idle, and creates four children with CreateTask().
static void SpawnerWorker(void* const raw_arg) {
  SpreadCtx* const ctx = static_cast<SpreadCtx*>(raw_arg);
  for (Task*& child : ctx->children) {
    child = ctx->sched->CreateTask(CountRunsWorker,   //
                                   &ctx->child_runs,  //
                                   kDefaultTaskWeight);
    ctx->queued_after_create.push_back(QueuedTasksPerCpu(*ctx->sched, 4));
  }
}

TEST(WorkStealingSchedulerTest,
     CreateTaskCountsRunningTasksSoNewWorkGoesToIdleCpusBeforeBusyOnes) {
  SetupEnv(4);
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(CreateCoreRunqueueScheduler());
  TaskSchedulerInternals::Init(*sched);

  SpreadCtx ctx;
  ctx.sched = sched.get();
  Task* const spawner =
      sched->CreateTaskOnCpu(SpawnerWorker, &ctx, kDefaultTaskWeight, 1);
  ASSERT_THAT(spawner, t::NotNull());

  // The calling thread stands in for CPU 1's idle loop, which runs the
  // spawner. Before returning, the idle loop also runs the child that the
  // spawner left queued on CPU 1.
  BindCpuLocal(SmpGetCpuLocal(1));
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*sched, 1));
  BindCpuLocal(SmpGetCpuLocal(0));

  // CPU 0 (running the bootstrap task) and CPU 1 (running the spawner) have
  // nothing queued but are busy, so the first two children go to the idle
  // CPUs 2 and 3. Then every CPU has load 1: the tie goes to the spawner's own
  // CPU 1 and, once CPU 1 is busier, to the lowest CPU, 0. Neither the busy
  // CPU 0 nor the calling CPU gets more than its share.
  EXPECT_THAT(ctx.queued_after_create,
              t::ElementsAre(t::ElementsAre(0, 0, 1, 0),  //
                             t::ElementsAre(0, 0, 1, 1),  //
                             t::ElementsAre(0, 1, 1, 1),  //
                             t::ElementsAre(1, 1, 1, 1)));
  // Each CPU got one wakeup IPI: CPU 1 for the spawner, the others for the
  // children placed there by the spawner.
  for (int cpu = 0; cpu < 4; ++cpu) {
    EXPECT_THAT(g_env.ipi_sent_count[cpu].load(), t::Eq(1));
  }
  EXPECT_THAT(ctx.child_runs, t::Eq(1));

  // The calling thread stands in for the idle loops of CPUs 2 and 3 to run
  // the children placed there; joining runs the one queued on CPU 0.
  BindCpuLocal(SmpGetCpuLocal(2));
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*sched, 2));
  BindCpuLocal(SmpGetCpuLocal(3));
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*sched, 3));
  BindCpuLocal(SmpGetCpuLocal(0));
  sched->Join(spawner);
  for (Task* const child : ctx.children) {
    ASSERT_THAT(child, t::NotNull());
    sched->Join(child);
  }
  EXPECT_THAT(ctx.child_runs, t::Eq(4));
}

struct BusyTickerCtx {
  TaskScheduler* sched = nullptr;
  std::vector<int> queued_after_balanced_interrupts;
  int cpu_after_balanced_interrupts = -1;
  Task* extra = nullptr;
  int extra_cpu = -1;
  std::vector<int> queued_after_imbalanced_tick;
};

static void RecordCpuWorker(void* const raw_arg) {
  *static_cast<int*>(raw_arg) = CurrentCpuId();
}

// Runs on CPU 1 with nothing queued there, so CPU 1 is busy with load 1, and
// takes timer ticks and wakeup IPIs.
static void BusyTickerWorker(void* const raw_arg) {
  BusyTickerCtx* const ctx = static_cast<BusyTickerCtx*>(raw_arg);
  InterruptFrame timer_frame = {};
  timer_frame.vector = kVectorApicTimer;
  timer_frame.rflags = kRflagsInterruptEnableBit;
  InterruptFrame ipi_frame = {};
  ipi_frame.vector = kVectorWakeupIpi;
  ipi_frame.rflags = kRflagsInterruptEnableBit;

  for (int i = 0; i < 8; ++i) {
    TaskSchedulerInternals::OnTimerInterrupt(*ctx->sched, &timer_frame);
    TaskSchedulerInternals::OnWakeupIpi(*ctx->sched, &ipi_frame);
  }
  ctx->queued_after_balanced_interrupts = QueuedTasksPerCpu(*ctx->sched, 4);
  ctx->cpu_after_balanced_interrupts = CurrentCpuId();

  // A second queued task makes CPU 0 two tasks busier than CPU 1.
  ctx->extra = ctx->sched->CreateTaskOnCpu(RecordCpuWorker,     //
                                           &ctx->extra_cpu,     //
                                           kDefaultTaskWeight,  //
                                           0);
  TaskSchedulerInternals::OnTimerInterrupt(*ctx->sched, &timer_frame);
  ctx->queued_after_imbalanced_tick = QueuedTasksPerCpu(*ctx->sched, 4);
  // The stolen task runs on CPU 1 once it has the smallest vruntime there.
  for (int i = 0; i < 4 && ctx->extra_cpu < 0; ++i) {
    ctx->sched->Yield();
  }
}

TEST(WorkStealingSchedulerTest,
     InterruptsOnBusyCpuStealOnlyFromCpuTwoTasksBusierSoTasksDoNotPingPong) {
  SetupEnv(4);
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(CreateCoreRunqueueScheduler());
  TaskSchedulerInternals::Init(*sched);

  // CPU 0 runs the bootstrap task and has one task queued (load 2). CPU 2 is
  // idle with two tasks queued (load 2). CPU 3 is idle with nothing queued.
  int runs = 0;
  std::vector<Task*> queued_tasks;
  queued_tasks.push_back(
      sched->CreateTaskOnCpu(CountRunsWorker, &runs, kDefaultTaskWeight, 0));
  queued_tasks.push_back(
      sched->CreateTaskOnCpu(CountRunsWorker, &runs, kDefaultTaskWeight, 2));
  queued_tasks.push_back(
      sched->CreateTaskOnCpu(CountRunsWorker, &runs, kDefaultTaskWeight, 2));
  for (Task* const task : queued_tasks) {
    ASSERT_THAT(task, t::NotNull());
  }

  BusyTickerCtx ctx;
  ctx.sched = sched.get();
  Task* const ticker =
      sched->CreateTaskOnCpu(BusyTickerWorker, &ctx, kDefaultTaskWeight, 1);
  ASSERT_THAT(ticker, t::NotNull());
  BindCpuLocal(SmpGetCpuLocal(1));
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*sched, 1));
  BindCpuLocal(SmpGetCpuLocal(0));

  // 1. No CPU is more than one task busier than the ticker's CPU 1, so its
  //    ticks and IPIs moved nothing: moving a task from CPU 0 or CPU 2 would
  //    only swap the two loads.
  EXPECT_THAT(ctx.queued_after_balanced_interrupts, t::ElementsAre(1, 0, 2, 0));
  EXPECT_THAT(ctx.cpu_after_balanced_interrupts, t::Eq(1));

  // 2. With a second queued task, CPU 0 (load 3) is two tasks busier, and the
  //    next tick moved its newest task, the tail of its runqueue, to CPU 1.
  ASSERT_THAT(ctx.extra, t::NotNull());
  ASSERT_THAT(ctx.queued_after_imbalanced_tick, t::SizeIs(4));
  EXPECT_THAT(ctx.queued_after_imbalanced_tick[0], t::Eq(1));
  EXPECT_THAT(ctx.queued_after_imbalanced_tick[2], t::Eq(2));
  EXPECT_THAT(ctx.extra_cpu, t::Eq(1));

  // The calling thread stands in for CPU 2's idle loop to run the two tasks
  // queued there; joining runs the one left on CPU 0.
  BindCpuLocal(SmpGetCpuLocal(2));
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*sched, 2));
  BindCpuLocal(SmpGetCpuLocal(0));
  sched->Join(ticker);
  sched->Join(ctx.extra);
  for (Task* const task : queued_tasks) {
    sched->Join(task);
  }
  EXPECT_THAT(runs, t::Eq(3));
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
