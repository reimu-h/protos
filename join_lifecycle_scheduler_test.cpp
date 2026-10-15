#include "join_lifecycle_scheduler.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "scheduler_test_support.h"
#include "task_scheduler_internals.h"

namespace protos {
namespace {

namespace t = ::testing;

static Task* FakeTaskPtr(const int id) {
  return reinterpret_cast<Task*>(static_cast<uintptr_t>(0x10000 + id * 0x10));
}

// The entry function and argument that the join layer passed to the inner
// layer's CreateTask*(). Calling it runs the wrapped task to completion, as
// the inner layer would.
struct CapturedTaskLaunch {
  TaskFn entry = nullptr;
  void* arg = nullptr;

  void RunToCompletion() const {
    ASSERT_THAT(entry, t::NotNull());
    entry(arg);
  }
};

// Creates a task through `sched` while `mock_inner` returns `task`, and stores
// the wrapped launch in `*launch`.
static void CreateFakeTask(MockTaskScheduler& mock_inner,  //
                           TaskScheduler& sched,           //
                           Task* const task,               //
                           CapturedTaskLaunch* const launch) {
  EXPECT_CALL(mock_inner, CreateTask)
      .WillOnce([task, launch](const TaskFn entry,  //
                               void* const arg,     //
                               const int64_t /*weight*/) {
        *launch = {entry, arg};
        return task;
      });
  EXPECT_THAT(sched.CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(task));
}

// 1. Init() frees all join records and delegates to inner->Init().
TEST(JoinLifecycleSchedulerTest, InitClearsAllJoinRecordsAndDelegatesToInner) {
  ResetTestEnv();
  const int64_t heap_before = HeapTotalFreeBytes();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  Task* const stale_zombie = FakeTaskPtr(1);
  Task* const stale_running = FakeTaskPtr(2);
  CapturedTaskLaunch zombie_launch;
  CapturedTaskLaunch running_launch;
  CreateFakeTask(*mock_inner, *sched, stale_zombie, &zombie_launch);
  CreateFakeTask(*mock_inner, *sched, stale_running, &running_launch);

  // `stale_zombie` exits and is detached, so it waits for ReapZombies().
  zombie_launch.RunToCompletion();
  sched->Detach(stale_zombie);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Lt(heap_before));

  // Init() must free every join record and forward to inner->Init().
  int init_calls = 0;
  EXPECT_CALL(*mock_inner, Init).WillOnce([&]() { ++init_calls; });
  TaskSchedulerInternals::Init(*sched);
  EXPECT_THAT(init_calls, t::Eq(1));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));

  // The zombie's record is gone, so ReapZombies() finds nothing and must not
  // call inner->Join() (StrictMock).
  EXPECT_THAT(sched->ReapZombies(), t::Eq(0));

  // Tasks created after Init() are tracked as usual.
  Task* const self_task = FakeTaskPtr(3);
  Task* const fresh_task = FakeTaskPtr(4);
  CapturedTaskLaunch fresh_launch;
  CreateFakeTask(*mock_inner, *sched, fresh_task, &fresh_launch);
  fresh_launch.RunToCompletion();
  EXPECT_CALL(*mock_inner, CurrentTask).WillOnce([&]() { return self_task; });
  EXPECT_CALL(*mock_inner, Join).WillOnce([&](Task* const task) {
    EXPECT_THAT(task, t::Eq(fresh_task));
  });
  sched->Join(fresh_task);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// 2. CreateTask and CreateTaskOnCpu delegate to inner, track a join record for
//    each non-null Task*, and free the record when inner returns nullptr.
TEST(JoinLifecycleSchedulerTest,
     CreateTaskAndCreateTaskOnCpuTrackNonNullAndHandleNullptrCleanly) {
  ResetTestEnv();
  const int64_t heap_before = HeapTotalFreeBytes();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  int arg_a = 11;
  int arg_b = 22;
  Task* const self_task = FakeTaskPtr(1);
  Task* const task_a = FakeTaskPtr(2);
  Task* const task_b = FakeTaskPtr(3);
  CapturedTaskLaunch launch_a;
  CapturedTaskLaunch launch_b;

  // Non-null CreateTask return.
  EXPECT_CALL(*mock_inner, CreateTask)
      .WillOnce([&](const TaskFn entry, void* const arg, const int64_t weight) {
        EXPECT_THAT(entry, t::NotNull());
        EXPECT_THAT(arg, t::NotNull());
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight * 2));
        launch_a = {entry, arg};
        return task_a;
      });
  EXPECT_THAT(sched->CreateTask(
                  [](void* const raw) { *static_cast<int*>(raw) += 100; },  //
                  &arg_a,                                                   //
                  kDefaultTaskWeight * 2),
              t::Eq(task_a));
  const int64_t heap_with_one_record = HeapTotalFreeBytes();
  EXPECT_THAT(heap_with_one_record, t::Lt(heap_before));

  // A nullptr CreateTask return frees the join record allocated for it.
  EXPECT_CALL(*mock_inner, CreateTask)
      .WillOnce([&](const TaskFn entry, void* const arg, const int64_t weight) {
        EXPECT_THAT(entry, t::NotNull());
        EXPECT_THAT(arg, t::NotNull());
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight));
        return nullptr;
      });
  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::IsNull());
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_with_one_record));

  // Non-null CreateTaskOnCpu return.
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,    //
                    void* const arg,       //
                    const int64_t weight,  //
                    const int target_cpu) {
        EXPECT_THAT(entry, t::NotNull());
        EXPECT_THAT(arg, t::NotNull());
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight / 2));
        EXPECT_THAT(target_cpu, t::Eq(3));
        launch_b = {entry, arg};
        return task_b;
      });
  EXPECT_THAT(sched->CreateTaskOnCpu(
                  [](void* const raw) { *static_cast<int*>(raw) += 200; },  //
                  &arg_b,                                                   //
                  kDefaultTaskWeight / 2,                                   //
                  3),
              t::Eq(task_b));
  const int64_t heap_with_two_records = HeapTotalFreeBytes();

  // A nullptr CreateTaskOnCpu return frees the join record allocated for it.
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,    //
                    void* const arg,       //
                    const int64_t weight,  //
                    const int target_cpu) {
        EXPECT_THAT(entry, t::NotNull());
        EXPECT_THAT(arg, t::NotNull());
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight));
        EXPECT_THAT(target_cpu, t::Eq(1));
        return nullptr;
      });
  EXPECT_THAT(sched->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 1),
              t::IsNull());
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_with_two_records));

  // Both wrapped entries run the user callbacks.
  launch_a.RunToCompletion();
  launch_b.RunToCompletion();
  EXPECT_THAT(arg_a, t::Eq(111));
  EXPECT_THAT(arg_b, t::Eq(222));

  std::vector<Task*> joined_tasks;
  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([&]() {
    return self_task;
  });
  EXPECT_CALL(*mock_inner, Join).WillRepeatedly([&](Task* const task) {
    joined_tasks.push_back(task);
  });

  sched->Join(task_a);
  sched->Join(task_b);
  EXPECT_THAT(joined_tasks, t::ElementsAre(task_a, task_b));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// 3. Join(task) when task has already finished before Join(task) is called:
//    immediately delegates to inner->Join(task) without calling inner->Yield().
TEST(JoinLifecycleSchedulerTest,
     JoinWhenTaskAlreadyExitedDelegatesImmediatelyWithoutYielding) {
  ResetTestEnv();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  Task* const self_task = FakeTaskPtr(10);
  Task* const target_task = FakeTaskPtr(11);
  Task* joined_task = nullptr;
  CapturedTaskLaunch target_launch;
  CreateFakeTask(*mock_inner, *sched, target_task, &target_launch);

  // Target finishes before Join(target_task) is invoked.
  target_launch.RunToCompletion();

  // StrictMock ensures inner->Yield() is NEVER called during Join().
  EXPECT_CALL(*mock_inner, CurrentTask).WillOnce([&]() { return self_task; });
  EXPECT_CALL(*mock_inner, Join).WillOnce([&](Task* const task) {
    joined_task = task;
  });

  sched->Join(target_task);
  EXPECT_THAT(joined_task, t::Eq(target_task));

  // Join() freed the join record, so ReapZombies() finds nothing.
  EXPECT_THAT(sched->ReapZombies(), t::Eq(0));
}

// 4. Join(task) when task has not yet exited: calls inner->Yield() repeatedly
//    until task finishes its entry callback, then frees the join record and
//    delegates to inner->Join(task).
TEST(JoinLifecycleSchedulerTest,
     JoinWhenTaskRunningYieldsRepeatedlyUntilExitAndFreesRecord) {
  ResetTestEnv();
  const int64_t heap_before = HeapTotalFreeBytes();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  Task* const self_task = FakeTaskPtr(20);
  Task* const target_task = FakeTaskPtr(21);
  int yield_count = 0;
  constexpr int kYieldsBeforeExit = 4;
  Task* joined_task = nullptr;
  CapturedTaskLaunch target_launch;
  CreateFakeTask(*mock_inner, *sched, target_task, &target_launch);

  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([&]() {
    return self_task;
  });
  EXPECT_CALL(*mock_inner, Yield).WillRepeatedly([&]() {
    ++yield_count;
    if (yield_count == kYieldsBeforeExit) {
      target_launch.RunToCompletion();
    }
  });
  EXPECT_CALL(*mock_inner, Join).WillOnce([&](Task* const task) {
    EXPECT_THAT(yield_count, t::Eq(kYieldsBeforeExit));
    joined_task = task;
  });

  sched->Join(target_task);
  EXPECT_THAT(yield_count, t::Eq(kYieldsBeforeExit));
  EXPECT_THAT(joined_task, t::Eq(target_task));
  EXPECT_THAT(sched->ReapZombies(), t::Eq(0));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// 5. ReapZombies() reclaims exactly the detached tasks that have exited, in
//    the order in which they became zombies, by calling inner->Join() on them.
//    It leaves running detached tasks and every joinable task alone, including
//    joinable tasks that have exited but have not been joined yet.
TEST(JoinLifecycleSchedulerTest,
     ReapZombiesReclaimsOnlyDetachedTasksThatHaveExited) {
  ResetTestEnv();
  const int64_t heap_before = HeapTotalFreeBytes();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  Task* const joiner_task = FakeTaskPtr(30);
  Task* const detached_then_exited = FakeTaskPtr(31);
  Task* const exited_joinable = FakeTaskPtr(32);
  Task* const running_detached = FakeTaskPtr(33);
  Task* const exited_then_detached = FakeTaskPtr(34);
  Task* const actively_joined = FakeTaskPtr(35);

  CapturedTaskLaunch launch_detached_then_exited;
  CapturedTaskLaunch launch_exited_joinable;
  CapturedTaskLaunch launch_running_detached;
  CapturedTaskLaunch launch_exited_then_detached;
  CapturedTaskLaunch launch_active;
  CreateFakeTask(*mock_inner,           //
                 *sched,                //
                 detached_then_exited,  //
                 &launch_detached_then_exited);
  CreateFakeTask(*mock_inner, *sched, exited_joinable, &launch_exited_joinable);
  CreateFakeTask(*mock_inner,       //
                 *sched,            //
                 running_detached,  //
                 &launch_running_detached);
  CreateFakeTask(*mock_inner,           //
                 *sched,                //
                 exited_then_detached,  //
                 &launch_exited_then_detached);
  CreateFakeTask(*mock_inner, *sched, actively_joined, &launch_active);

  // Detach() never reaches the inner layer (StrictMock).
  sched->Detach(detached_then_exited);
  launch_detached_then_exited.RunToCompletion();
  launch_exited_joinable.RunToCompletion();
  sched->Detach(running_detached);
  launch_exited_then_detached.RunToCompletion();
  sched->Detach(exited_then_detached);

  // Start `Join(actively_joined)` on `joiner_task`. Inside `Yield()`,
  // `actively_joined` finishes AND `ReapZombies()` runs before `Join()`
  // finishes.
  std::vector<Task*> reaped_during_yield;
  int reap_count_during_yield = -1;
  int reap_count_second_call = -1;
  Task* joined_by_joiner = nullptr;

  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([&]() {
    return joiner_task;
  });
  EXPECT_CALL(*mock_inner, Yield).WillOnce([&]() {
    launch_active.RunToCompletion();
    reap_count_during_yield = sched->ReapZombies();
    reap_count_second_call = sched->ReapZombies();
  });
  EXPECT_CALL(*mock_inner, Join).WillRepeatedly([&](Task* const task) {
    if (reap_count_during_yield < 0) {
      reaped_during_yield.push_back(task);
    } else {
      joined_by_joiner = task;
    }
  });

  sched->Join(actively_joined);

  EXPECT_THAT(reap_count_during_yield, t::Eq(2));
  EXPECT_THAT(reaped_during_yield,
              t::ElementsAre(detached_then_exited, exited_then_detached));
  EXPECT_THAT(reap_count_second_call, t::Eq(0));
  EXPECT_THAT(joined_by_joiner, t::Eq(actively_joined));

  // `exited_joinable` survived both ReapZombies() calls and can still be
  // joined.
  Task* joined_later = nullptr;
  EXPECT_CALL(*mock_inner, Join).WillOnce([&](Task* const task) {
    joined_later = task;
  });
  sched->Join(exited_joinable);
  EXPECT_THAT(joined_later, t::Eq(exited_joinable));

  // `running_detached` is reaped once it exits.
  EXPECT_THAT(sched->ReapZombies(), t::Eq(0));
  launch_running_detached.RunToCompletion();
  Task* reaped_running = nullptr;
  EXPECT_CALL(*mock_inner, Join).WillOnce([&](Task* const task) {
    reaped_running = task;
  });
  EXPECT_THAT(sched->ReapZombies(), t::Eq(1));
  EXPECT_THAT(reaped_running, t::Eq(running_detached));
  EXPECT_THAT(sched->ReapZombies(), t::Eq(0));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// 6. Join records are allocated dynamically: more than a thousand can be live
//    at once, and every record is freed again, whether its task is joined or
//    detached and reaped.
TEST(JoinLifecycleSchedulerTest, ManyLiveRecordsAreAllReclaimedByJoinOrReap) {
  ResetTestEnv();
  const int64_t heap_before = HeapTotalFreeBytes();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  constexpr int kWaves = 2;
  constexpr int kTasksPerWave = 1500;
  Task* const self_task = FakeTaskPtr(1);
  int next_task_id = 1000;
  int inner_join_calls = 0;
  std::vector<CapturedTaskLaunch> launches;

  EXPECT_CALL(*mock_inner, CreateTask)
      .WillRepeatedly(
          [&](const TaskFn entry, void* const arg, const int64_t /*weight*/) {
            launches.push_back({entry, arg});
            return FakeTaskPtr(next_task_id++);
          });
  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([&]() {
    return self_task;
  });
  EXPECT_CALL(*mock_inner, Join).WillRepeatedly([&](Task* const task) {
    EXPECT_THAT(task, t::NotNull());
    ++inner_join_calls;
  });

  for (int wave = 0; wave < kWaves; ++wave) {
    launches.clear();
    std::vector<Task*> tasks;
    tasks.reserve(kTasksPerWave);
    for (int i = 0; i < kTasksPerWave; ++i) {
      Task* const task =
          sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight);
      ASSERT_THAT(task, t::NotNull());
      tasks.push_back(task);
    }
    for (const CapturedTaskLaunch& launch : launches) {
      launch.RunToCompletion();
    }

    // Join the first half; detach the second half and reap it.
    const int half = kTasksPerWave / 2;
    for (int i = 0; i < half; ++i) {
      sched->Join(tasks[i]);
    }
    for (int i = half; i < kTasksPerWave; ++i) {
      sched->Detach(tasks[i]);
    }
    EXPECT_THAT(sched->ReapZombies(), t::Eq(kTasksPerWave - half));
    EXPECT_THAT(sched->ReapZombies(), t::Eq(0));
    EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
  }
  EXPECT_THAT(inner_join_calls, t::Eq(kWaves * kTasksPerWave));
}

// 7. Multi-threaded concurrency test: one thread runs Join(task)
//    spinning/yielding while another thread concurrently finishes task's
//    wrapped entry, verifying thread-safe synchronization under TSAN.
TEST(JoinLifecycleSchedulerTest,
     ConcurrentJoinAndTaskCompletionAcrossThreadsSynchronizesCleanlyUnderTsan) {
  ResetTestEnv();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  constexpr int kNumPairs = 32;
  Task* const joiner_task = FakeTaskPtr(1);
  thread_local Task* tls_current_task = nullptr;

  std::atomic<int> total_yields{0};
  std::atomic<int> total_completions{0};
  std::atomic<int> total_joins{0};

  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([&]() {
    return tls_current_task;
  });
  EXPECT_CALL(*mock_inner, Yield).WillRepeatedly([&]() {
    total_yields.fetch_add(1, std::memory_order_relaxed);
  });
  EXPECT_CALL(*mock_inner, Join).WillRepeatedly([&](Task* const task) {
    EXPECT_THAT(task, t::NotNull());
    total_joins.fetch_add(1, std::memory_order_acq_rel);
  });

  for (int round = 0; round < kNumPairs; ++round) {
    Task* const worker_task = FakeTaskPtr(100 + round);
    CapturedTaskLaunch worker_launch;
    EXPECT_CALL(*mock_inner, CreateTask)
        .WillOnce([worker_task, &worker_launch](const TaskFn entry,
                                                void* const arg,
                                                const int64_t /*weight*/) {
          worker_launch = {entry, arg};
          return worker_task;
        });
    ASSERT_THAT(sched->CreateTask(
                    [](void* const raw) {
                      static_cast<std::atomic<int>*>(raw)->fetch_add(
                          1, std::memory_order_acq_rel);
                    },                   //
                    &total_completions,  //
                    kDefaultTaskWeight),
                t::Eq(worker_task));

    std::atomic<bool> join_entered_yield{false};
    const int yields_before = total_yields.load(std::memory_order_relaxed);

    std::thread worker_thread([&]() {
      SetInterruptsEnabledForTest(true);
      tls_current_task = worker_task;
      while (total_yields.load(std::memory_order_acquire) <= yields_before &&
             !join_entered_yield.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      worker_launch.RunToCompletion();
    });

    tls_current_task = joiner_task;
    join_entered_yield.store(true, std::memory_order_release);
    sched->Join(worker_task);
    worker_thread.join();
  }

  EXPECT_THAT(total_completions.load(std::memory_order_acquire),
              t::Eq(kNumPairs));
  EXPECT_THAT(total_joins.load(std::memory_order_acquire), t::Eq(kNumPairs));
  EXPECT_THAT(sched->ReapZombies(), t::Eq(0));
}

// 8. A reaper thread that calls ReapZombies() in a loop never reclaims a
//    joinable task, even one that has exited long before its owner joins it,
//    while it still reclaims every detached task (TSAN covers the locking).
TEST(JoinLifecycleSchedulerTest,
     ConcurrentReapZombiesNeverReclaimsTasksThatAwaitJoin) {
  constexpr int kTasksPerThread = 200;
  SetupEnv(3);
  const int64_t heap_before = HeapTotalFreeBytes();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  thread_local CapturedTaskLaunch tls_last_launch;
  std::atomic<int> next_task_id{1};
  std::atomic<int> inner_joins{0};
  Task* const joiner_task = FakeTaskPtr(1 << 20);

  EXPECT_CALL(*mock_inner, CreateTask)
      .WillRepeatedly(
          [&](const TaskFn entry, void* const arg, const int64_t /*weight*/) {
            tls_last_launch = {entry, arg};
            return FakeTaskPtr(
                next_task_id.fetch_add(1, std::memory_order_relaxed));
          });
  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([&]() {
    return joiner_task;
  });
  EXPECT_CALL(*mock_inner, Join).WillRepeatedly([&](Task* const task) {
    EXPECT_THAT(task, t::NotNull());
    inner_joins.fetch_add(1, std::memory_order_acq_rel);
  });

  std::atomic<int> reaped{0};
  std::atomic<bool> stop_reaper{false};
  std::thread reaper([&]() {
    BindCpuLocal(&g_env.cpu_locals[1]);
    SetInterruptsEnabledForTest(true);
    while (!stop_reaper.load(std::memory_order_acquire)) {
      reaped.fetch_add(sched->ReapZombies(), std::memory_order_acq_rel);
      std::this_thread::yield();
    }
    ResetCpuLocalForTest();
  });

  // Detached tasks exit either before or after they are detached.
  std::thread detacher([&]() {
    BindCpuLocal(&g_env.cpu_locals[2]);
    SetInterruptsEnabledForTest(true);
    for (int i = 0; i < kTasksPerThread; ++i) {
      Task* const task =
          sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight);
      if ((i & 1) == 0) {
        tls_last_launch.RunToCompletion();
        sched->Detach(task);
      } else {
        sched->Detach(task);
        tls_last_launch.RunToCompletion();
      }
    }
    ResetCpuLocalForTest();
  });

  // Joinable tasks exit, then give the reaper time to run before the join.
  for (int i = 0; i < kTasksPerThread; ++i) {
    Task* const task = sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight);
    tls_last_launch.RunToCompletion();
    std::this_thread::yield();
    sched->Join(task);
  }

  detacher.join();
  while (reaped.load(std::memory_order_acquire) < kTasksPerThread) {
    std::this_thread::yield();
  }
  stop_reaper.store(true, std::memory_order_release);
  reaper.join();

  EXPECT_THAT(reaped.load(std::memory_order_acquire), t::Eq(kTasksPerThread));
  EXPECT_THAT(inner_joins.load(std::memory_order_acquire),
              t::Eq(2 * kTasksPerThread));
  EXPECT_THAT(sched->ReapZombies(), t::Eq(0));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// 9. DCHECK death tests (JoinLifecycleSchedulerDeathTest) verifying
//    preconditions: null inner in CreateJoinLifecycleScheduler, null entry in
//    CreateTask/CreateTaskOnCpu, null task in Join, CurrentTask() == nullptr
//    in Join, self-join (Join(self)), joining or detaching an unknown Task*,
//    calling Join while holding an IrqSpinLock, joining a detached task,
//    detaching twice, detaching a task that is being joined, returning from a
//    task while holding an IrqSpinLock, and double-joining.
TEST(JoinLifecycleSchedulerDeathTest, PreconditionViolationsTriggerDcheck) {
  ResetTestEnv();

  // 1. Null inner in CreateJoinLifecycleScheduler.
  EXPECT_DEATH(CreateJoinLifecycleScheduler(nullptr), "Check failed");

  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  EXPECT_DEATH(sched->CreateTask(nullptr, nullptr, kDefaultTaskWeight),
               "Check failed");
  EXPECT_DEATH(sched->CreateTaskOnCpu(nullptr, nullptr, kDefaultTaskWeight, 0),
               "Check failed");

  Task* const self_task = FakeTaskPtr(1);
  Task* const target_task = FakeTaskPtr(2);
  Task* const unknown_task = FakeTaskPtr(3);
  Task* const running_task = FakeTaskPtr(4);
  Task* const lock_leaking_task = FakeTaskPtr(5);
  CapturedTaskLaunch target_launch;
  CapturedTaskLaunch running_launch;
  CreateFakeTask(*mock_inner, *sched, target_task, &target_launch);
  CreateFakeTask(*mock_inner, *sched, running_task, &running_launch);

  // Complete `target_task` so valid `Join(target_task)` can complete.
  target_launch.RunToCompletion();

  // 2. Null task in Join(nullptr) and Detach(nullptr).
  EXPECT_DEATH(sched->Join(nullptr), "Check failed");
  EXPECT_DEATH(sched->Detach(nullptr), "Check failed");

  // 3. CurrentTask() == nullptr in Join.
  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([]() {
    return nullptr;
  });
  EXPECT_DEATH(sched->Join(target_task), "Check failed");

  // Restore CurrentTask() == self_task for remaining checks.
  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([&]() {
    return self_task;
  });

  // 4. Self-join: Join(self).
  EXPECT_DEATH(sched->Join(self_task), "Check failed");

  // 5. Joining or detaching an unknown/unregistered Task*.
  EXPECT_DEATH(sched->Join(unknown_task), "Check failed");
  EXPECT_DEATH(sched->Detach(unknown_task), "Check failed");

  // 6. Calling Join while holding an IrqSpinLock.
  EXPECT_DEATH(
      {
        IrqSpinLock held_lock;
        const IrqSpinLockGuard guard(held_lock);
        sched->Join(target_task);
      },
      "Check failed");

  // 7. Joining a detached task, and detaching a task twice.
  EXPECT_DEATH(
      {
        sched->Detach(target_task);
        sched->Join(target_task);
      },
      "Check failed");
  EXPECT_DEATH(
      {
        sched->Detach(running_task);
        sched->Detach(running_task);
      },
      "Check failed");

  // 8. Detaching a task while another task is joining it.
  EXPECT_DEATH(
      {
        EXPECT_CALL(*mock_inner, Yield).WillOnce([&]() {
          sched->Detach(running_task);
        });
        sched->Join(running_task);
      },
      "Check failed");

  // 9. Returning from a task while holding an IrqSpinLock.
  static IrqSpinLock leaked_lock;
  CapturedTaskLaunch lock_leaking_launch;
  EXPECT_CALL(*mock_inner, CreateTask)
      .WillOnce(
          [&](const TaskFn entry, void* const arg, const int64_t /*weight*/) {
            lock_leaking_launch = {entry, arg};
            return lock_leaking_task;
          });
  ASSERT_THAT(
      sched->CreateTask([](void* const /*arg*/) { leaked_lock.Lock(); },  //
                        nullptr,                                          //
                        kDefaultTaskWeight),
      t::Eq(lock_leaking_task));
  EXPECT_DEATH(lock_leaking_launch.RunToCompletion(), "Check failed");

  // 10. Double-joining an already-joined Task*.
  EXPECT_CALL(*mock_inner, Join).WillOnce([&](Task* const task) {
    EXPECT_THAT(task, t::Eq(target_task));
  });
  sched->Join(target_task);
  EXPECT_DEATH(sched->Join(target_task), "Check failed");
}

// 10. A task that runs to completion on the creating CPU before the inner
//     CreateTask*() call returns (as a synchronous inner layer would do) must
//     not wait for its record to be finalized, which would deadlock: the
//     creator finalizes the record only after the inner call returns.
TEST(JoinLifecycleSchedulerTest,
     SynchronousTaskCompletionInsideCreateTaskDoesNotDeadlock) {
  ResetTestEnv();
  const int64_t heap_before = HeapTotalFreeBytes();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  Task* const self_task = FakeTaskPtr(1);
  Task* const sync_task_1 = FakeTaskPtr(2);
  Task* const sync_task_2 = FakeTaskPtr(3);
  int callback_runs = 0;

  // The newly created task runs to completion inside inner().CreateTask*()
  // BEFORE it returns.
  EXPECT_CALL(*mock_inner, CreateTask)
      .WillOnce(
          [&](const TaskFn entry, void* const arg, const int64_t /*weight*/) {
            entry(arg);
            return sync_task_1;
          });
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,        //
                    void* const arg,           //
                    const int64_t /*weight*/,  //
                    const int /*target_cpu*/) {
        entry(arg);
        return sync_task_2;
      });

  EXPECT_THAT(sched->CreateTask(
                  [](void* const raw) { ++(*static_cast<int*>(raw)); },  //
                  &callback_runs,                                        //
                  kDefaultTaskWeight),
              t::Eq(sync_task_1));
  EXPECT_THAT(sched->CreateTaskOnCpu(
                  [](void* const raw) { ++(*static_cast<int*>(raw)); },  //
                  &callback_runs,                                        //
                  kDefaultTaskWeight,                                    //
                  0),
              t::Eq(sync_task_2));
  EXPECT_THAT(callback_runs, t::Eq(2));

  // Join sync_task_1 immediately (must not yield), and detach sync_task_2 and
  // reap it via ReapZombies().
  std::vector<Task*> joined_tasks;
  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([&]() {
    return self_task;
  });
  EXPECT_CALL(*mock_inner, Join).WillRepeatedly([&](Task* const task) {
    joined_tasks.push_back(task);
  });

  sched->Join(sync_task_1);
  sched->Detach(sync_task_2);
  EXPECT_THAT(sched->ReapZombies(), t::Eq(1));
  EXPECT_THAT(joined_tasks, t::ElementsAre(sync_task_1, sync_task_2));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// 11. OOM during AllocateJoinRecord returns nullptr cleanly without invoking
//     inner->CreateTask or inner->CreateTaskOnCpu, and a task that starts on
//     another CPU waits for FinalizeJoinRecord before running its user
//     function.
TEST(JoinLifecycleSchedulerTest,
     AllocateJoinRecordOomReturnsNullptrAndRemoteCpuEntryWaitsForFinalization) {
  SetupEnv(2);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  // Force Kmalloc(sizeof(TaskJoinRecord)) to return nullptr. StrictMock
  // verifies inner->CreateTask* is not called.
  g_env.fail_kmalloc = true;
  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::IsNull());
  EXPECT_THAT(sched->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 0),
              t::IsNull());
  g_env.fail_kmalloc = false;
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));

  // A task that starts on CPU 1 while its creator on CPU 0 is still inside
  // CreateTaskOnCpu() must not run its user function before the record is
  // finalized, so that Join(remote_task) from anywhere finds the record.
  BindCpuLocal(SmpGetCpuLocal(0));
  Task* const self_task = FakeTaskPtr(70);
  Task* const remote_task = FakeTaskPtr(71);
  std::atomic<bool> child_entered_inner{false};
  std::atomic<bool> child_ran_user_fn{false};
  std::thread remote_thread;

  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,        //
                    void* const arg,           //
                    const int64_t /*weight*/,  //
                    const int /*target_cpu*/) {
        // Interrupts must be disabled across inner().CreateTaskOnCpu +
        // FinalizeJoinRecord.
        EXPECT_FALSE(AreInterruptsEnabled());
        remote_thread = std::thread([entry, arg, &child_entered_inner]() {
          SetInterruptsEnabledForTest(true);
          BindCpuLocal(SmpGetCpuLocal(1));
          child_entered_inner.store(true, std::memory_order_release);
          entry(arg);
        });
        while (!child_entered_inner.load(std::memory_order_acquire)) {
          std::this_thread::yield();
        }
        // The record is finalized only after this returns. Hold the return
        // long enough that a task that didn't wait would have run its user
        // function by now.
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        EXPECT_FALSE(child_ran_user_fn.load(std::memory_order_acquire));
        return remote_task;
      });

  EXPECT_THAT(sched->CreateTaskOnCpu(
                  [](void* const raw) {
                    static_cast<std::atomic<bool>*>(raw)->store(
                        true, std::memory_order_release);
                  },                   //
                  &child_ran_user_fn,  //
                  kDefaultTaskWeight,  //
                  1),
              t::Eq(remote_task));
  remote_thread.join();
  EXPECT_TRUE(child_ran_user_fn.load(std::memory_order_acquire));

  EXPECT_CALL(*mock_inner, CurrentTask).WillOnce([&]() { return self_task; });
  EXPECT_CALL(*mock_inner, Join).WillOnce([&](Task* const task) {
    EXPECT_THAT(task, t::Eq(remote_task));
  });
  sched->Join(remote_task);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// 12. Yield(), SetPreemptEnabled(), IsPreemptEnabled(), CurrentTask(),
//     SetAddressSpace() and the scheduler internals other than Init()
//     (OnTimerInterrupt(), OnWakeupIpi(), PollIdleCpu(), RunqueueLoad(),
//     IsCpuBusy() and StealTask()) are forwarded to inner once each, with
//     unchanged arguments and result.
TEST(JoinLifecycleSchedulerTest,
     ForwardsOtherMethodsToInnerOnceWithUnchangedArgumentsAndResult) {
  ResetTestEnv();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);
  // Every expectation must be met by exactly one call, in the order below, and
  // the StrictMock fails the test on any other call to the inner layer.
  const t::InSequence in_sequence;

  Task* const fake_current = FakeTaskPtr(80);
  InterruptFrame timer_frame = {};
  InterruptFrame ipi_frame = {};

  EXPECT_CALL(*mock_inner, Yield);
  sched->Yield();

  EXPECT_CALL(*mock_inner, SetPreemptEnabled)
      .WillOnce([](const bool enabled) { EXPECT_FALSE(enabled); })
      .WillOnce([](const bool enabled) { EXPECT_TRUE(enabled); });
  sched->SetPreemptEnabled(false);
  sched->SetPreemptEnabled(true);

  EXPECT_CALL(*mock_inner, IsPreemptEnabled)
      .WillOnce([]() { return false; })
      .WillOnce([]() { return true; });
  EXPECT_FALSE(sched->IsPreemptEnabled());
  EXPECT_TRUE(sched->IsPreemptEnabled());

  EXPECT_CALL(*mock_inner, CurrentTask).WillOnce([&]() {
    return fake_current;
  });
  EXPECT_THAT(sched->CurrentTask(), t::Eq(fake_current));

  AddressSpace* const fake_address_space =
      reinterpret_cast<AddressSpace*>(0x2000);
  EXPECT_CALL(*mock_inner, SetAddressSpace)
      .WillOnce([&](AddressSpace* const address_space) {
        EXPECT_THAT(address_space, t::Eq(fake_address_space));
      })
      .WillOnce([](AddressSpace* const address_space) {
        EXPECT_THAT(address_space, t::IsNull());
      });
  sched->SetAddressSpace(fake_address_space);
  sched->SetAddressSpace(nullptr);

  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&timer_frame));
      });
  TaskSchedulerInternals::OnTimerInterrupt(*sched, &timer_frame);

  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&ipi_frame));
      });
  TaskSchedulerInternals::OnWakeupIpi(*sched, &ipi_frame);

  EXPECT_CALL(*mock_inner, PollIdleCpu)
      .WillOnce([](const int cpu_index) {
        EXPECT_THAT(cpu_index, t::Eq(2));
        return true;
      })
      .WillOnce([](const int cpu_index) {
        EXPECT_THAT(cpu_index, t::Eq(1));
        return false;
      });
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*sched, 2));
  EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*sched, 1));

  EXPECT_CALL(*mock_inner, RunqueueLoad).WillOnce([](const int cpu_index) {
    EXPECT_THAT(cpu_index, t::Eq(3));
    return 7;
  });
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*sched, 3), t::Eq(7));

  EXPECT_CALL(*mock_inner, IsCpuBusy)
      .WillOnce([](const int cpu_index) {
        EXPECT_THAT(cpu_index, t::Eq(1));
        return true;
      })
      .WillOnce([](const int cpu_index) {
        EXPECT_THAT(cpu_index, t::Eq(2));
        return false;
      });
  EXPECT_TRUE(TaskSchedulerInternals::IsCpuBusy(*sched, 1));
  EXPECT_FALSE(TaskSchedulerInternals::IsCpuBusy(*sched, 2));

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
  EXPECT_TRUE(TaskSchedulerInternals::StealTask(*sched, 0, 2));
  EXPECT_FALSE(TaskSchedulerInternals::StealTask(*sched, 1, 3));
}

}  // namespace
}  // namespace protos
