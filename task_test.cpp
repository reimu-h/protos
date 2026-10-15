#include "task.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gdt.h"
#include "idt.h"
#include "interrupts.h"
#include "scheduler_test_support.h"
#include "smp.h"
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

static void SetFlagWorker(void* const arg) { *static_cast<bool*>(arg) = true; }

// ============================================================================
// Composed 4-Layer `TaskScheduler` Unit Tests (`TaskTest`)
// ============================================================================

TEST(TaskTest, InitRegistersHandlersAndSetsUpBootstrapAndPerCpuIdleTasks) {
  constexpr int kNumCpus = 4;
  SetupEnv(kNumCpus);
  EXPECT_THAT(GetTaskScheduler(), t::IsNull());
  EXPECT_FALSE(TaskPollIdleCpu(0));

  TaskInit();
  TaskScheduler* const sched = GetTaskScheduler();
  ASSERT_THAT(sched, t::NotNull());
  EXPECT_TRUE(IdtIsInitialized());
  EXPECT_THAT(IdtGetHandler(kVectorApicTimer), t::NotNull());
  EXPECT_THAT(IdtGetHandler(kVectorWakeupIpi), t::NotNull());
  EXPECT_TRUE(sched->IsPreemptEnabled());

  // The published scheduler disables and re-enables preemption.
  sched->SetPreemptEnabled(false);
  EXPECT_FALSE(sched->IsPreemptEnabled());
  sched->SetPreemptEnabled(true);
  EXPECT_TRUE(sched->IsPreemptEnabled());

  // CPU 0 runs the bootstrap task (the calling thread); every AP runs its own
  // idle task.
  Task* const bootstrap = sched->CurrentTask();
  ASSERT_THAT(bootstrap, t::NotNull());
  std::vector<Task*> current_tasks = {bootstrap};
  for (int c = 1; c < kNumCpus; ++c) {
    BindCpuLocal(SmpGetCpuLocal(c));
    Task* const idle = sched->CurrentTask();
    BindCpuLocal(SmpGetCpuLocal(0));
    ASSERT_THAT(idle, t::NotNull());
    EXPECT_THAT(current_tasks, t::Not(t::Contains(idle)));
    current_tasks.push_back(idle);
  }

  for (int c = 0; c < kNumCpus; ++c) {
    EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*sched, c), t::Eq(0));
    EXPECT_FALSE(TaskPollIdleCpu(c));
  }

  // Both vectors dispatch (each with one EOI) and, with nothing runnable,
  // leave the bootstrap task running.
  InterruptFrame timer_frame = MakeInterruptFrame(kVectorApicTimer);
  InterruptFrame ipi_frame = MakeInterruptFrame(kVectorWakeupIpi);
  const int64_t eoi_before = g_env.eoi_count.load();
  IdtDispatch(&timer_frame);
  IdtDispatch(&ipi_frame);
  EXPECT_THAT(g_env.eoi_count.load(), t::Eq(eoi_before + 2));
  EXPECT_THAT(sched->CurrentTask(), t::Eq(bootstrap));

  // The wakeup IPI handler reaches the scheduler, and idle CPU 1's idle loop
  // then polls it and runs the task queued there. (The calling thread briefly
  // acts as CPU 1's idle loop.)
  bool remote_ran = false;
  Task* const remote =
      sched->CreateTaskOnCpu(SetFlagWorker, &remote_ran, kDefaultTaskWeight, 1);
  ASSERT_THAT(remote, t::NotNull());
  EXPECT_THAT(g_env.ipi_sent_count[1].load(), t::Eq(1));
  BindCpuLocal(SmpGetCpuLocal(1));
  const int64_t eoi_before_ipi = g_env.eoi_count.load();
  IdtDispatch(&ipi_frame);
  EXPECT_THAT(g_env.eoi_count.load(), t::Eq(eoi_before_ipi + 1));
  EXPECT_TRUE(TaskPollIdleCpu(1));
  BindCpuLocal(SmpGetCpuLocal(0));
  EXPECT_TRUE(remote_ran);
  sched->Join(remote);

  TaskResetForTest();
  EXPECT_THAT(GetTaskScheduler(), t::IsNull());
  EXPECT_THAT(IdtGetHandler(kVectorApicTimer), t::IsNull());
  EXPECT_THAT(IdtGetHandler(kVectorWakeupIpi), t::IsNull());
}

TEST(TaskTest, PublishesTheSchedulerOnlyAfterInitToConcurrentReaders) {
  constexpr int kNumCpus = 3;
  SetupEnv(kNumCpus);

  // AP threads call GetTaskScheduler() and TaskPollIdleCpu() while TaskInit()
  // runs, the way the kernel's tasks and AP idle loops do. TSAN checks that
  // publishing the scheduler is race-free and that readers see it fully
  // initialized.
  std::atomic<bool> stop_aps{false};
  std::atomic<int> aps_saw_scheduler{0};
  std::vector<std::thread> ap_threads;
  for (int cpu = 1; cpu < kNumCpus; ++cpu) {
    ap_threads.emplace_back([cpu, &stop_aps, &aps_saw_scheduler]() {
      BindCpuLocal(SmpGetCpuLocal(cpu));
      SetInterruptsEnabledForTest(true);
      bool saw_scheduler = false;
      while (!stop_aps.load(std::memory_order_acquire)) {
        TaskScheduler* const ap_sched = GetTaskScheduler();
        if (ap_sched != nullptr && !saw_scheduler) {
          saw_scheduler = true;
          aps_saw_scheduler.fetch_add(1, std::memory_order_acq_rel);
        }
        if (!TaskPollIdleCpu(cpu)) {
          std::this_thread::yield();
        }
      }
      ResetCpuLocalForTest();
    });
  }

  TaskInit();
  TaskScheduler* const sched = GetTaskScheduler();
  ASSERT_THAT(sched, t::NotNull());
  while (aps_saw_scheduler.load(std::memory_order_acquire) < kNumCpus - 1) {
    std::this_thread::yield();
  }

  // The scheduler that the APs picked up runs tasks (on any CPU).
  bool task_ran = false;
  Task* const task = sched->CreateTaskOnCpu(SetFlagWorker,       //
                                            &task_ran,           //
                                            kDefaultTaskWeight,  //
                                            kNumCpus - 1);
  ASSERT_THAT(task, t::NotNull());
  sched->Join(task);
  EXPECT_TRUE(task_ran);

  stop_aps.store(true, std::memory_order_release);
  for (std::thread& th : ap_threads) {
    th.join();
  }
  TaskResetForTest();
}

struct LifecycleTaskCtx {
  std::vector<int> trace;
  int task_index = 0;
  bool stack_aligned = false;
  Task* observed_task = nullptr;
};

static void LifecycleWorker(void* const raw_arg) {
  LifecycleTaskCtx* const ctx = static_cast<LifecycleTaskCtx*>(raw_arg);
  ASSERT_THAT(ctx, t::NotNull());

  TaskScheduler* const sched = GetTaskScheduler();
  alignas(16) uint64_t probe = 0;
  ctx->stack_aligned = ((reinterpret_cast<uintptr_t>(&probe) & 0xFu) == 0);
  ctx->observed_task = sched->CurrentTask();

  for (int step = 0; step < 3; ++step) {
    ctx->trace.push_back(ctx->task_index * 10 + step);
    sched->Yield();
  }
}

TEST(TaskTest, CreateYieldJoinAndImplicitExitLifecycle) {
  SetupEnv(1);
  TaskInit();
  TaskScheduler* const sched = GetTaskScheduler();

  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  LifecycleTaskCtx ctx_a;
  ctx_a.task_index = 1;
  LifecycleTaskCtx ctx_b;
  ctx_b.task_index = 2;

  Task* const task_a =
      sched->CreateTask(LifecycleWorker, &ctx_a, kDefaultTaskWeight);
  Task* const task_b =
      sched->CreateTask(LifecycleWorker, &ctx_b, kDefaultTaskWeight);
  ASSERT_THAT(task_a, t::NotNull());
  ASSERT_THAT(task_b, t::NotNull());
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*sched, 0), t::Eq(2));

  // Both tasks exit by returning from their entry function.
  sched->Join(task_a);
  sched->Join(task_b);

  EXPECT_TRUE(ctx_a.stack_aligned);
  EXPECT_TRUE(ctx_b.stack_aligned);
  EXPECT_THAT(ctx_a.observed_task, t::Eq(task_a));
  EXPECT_THAT(ctx_b.observed_task, t::Eq(task_b));
  EXPECT_THAT(task_a, t::Ne(task_b));
  EXPECT_THAT(ctx_a.trace, t::ElementsAre(10, 11, 12));
  EXPECT_THAT(ctx_b.trace, t::ElementsAre(20, 21, 22));

  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*sched, 0), t::Eq(0));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));

  TaskResetForTest();
}

struct WeightedShareCtx {
  std::vector<int> dispatch_order;
  int slices_for_task[3] = {};
  int slices_when_high_finished[3] = {};
};

struct WeightedShareTaskArg {
  WeightedShareCtx* shared = nullptr;
  int index = 0;
};

static void WeightedShareWorker(void* const raw_arg) {
  WeightedShareTaskArg* const arg = static_cast<WeightedShareTaskArg*>(raw_arg);
  WeightedShareCtx* const shared = arg->shared;
  const int idx = arg->index;
  TaskScheduler* const sched = GetTaskScheduler();

  for (int i = 0; i < 8; ++i) {
    shared->dispatch_order.push_back(idx);
    ++shared->slices_for_task[idx];
    if (idx == 0 && i == 7) {
      for (int k = 0; k < 3; ++k) {
        shared->slices_when_high_finished[k] = shared->slices_for_task[k];
      }
    }
    sched->Yield();
  }
}

TEST(TaskTest, WeightedRbTreeVruntimeOrderingAndInverseWeightScaling) {
  SetupEnv(1);
  TaskInit();
  TaskScheduler* const sched = GetTaskScheduler();

  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  WeightedShareCtx shared;
  WeightedShareTaskArg args[3] = {
      {&shared, 0},
      {&shared, 1},
      {&shared, 2},
  };

  // Create three tasks on CPU 0 with weights 2048 (2x), 1024 (1x), and 512
  // (0.5x). All three start at min_vruntime == 0 and are ordered initially by
  // FIFO enqueue_seq (0, 1, 2).
  Task* const t_high =
      sched->CreateTask(WeightedShareWorker, &args[0], kDefaultTaskWeight * 2);
  Task* const t_med =
      sched->CreateTask(WeightedShareWorker, &args[1], kDefaultTaskWeight);
  Task* const t_low =
      sched->CreateTask(WeightedShareWorker, &args[2], kDefaultTaskWeight / 2);

  sched->Join(t_high);
  sched->Join(t_med);
  sched->Join(t_low);

  // First 3 dispatches must follow initial FIFO enqueue_seq tie-breaking
  // (0, 1, 2) because all 3 started at vruntime == 0.
  ASSERT_THAT(static_cast<int>(shared.dispatch_order.size()), t::Eq(24));
  EXPECT_THAT(shared.dispatch_order[0], t::Eq(0));
  EXPECT_THAT(shared.dispatch_order[1], t::Eq(1));
  EXPECT_THAT(shared.dispatch_order[2], t::Eq(2));

  // When t_high (weight 2048, +512/slice) completes its 8th slice (at
  // vruntime == 4096), t_med (weight 1024, +1024/slice) has completed 4 slices
  // and t_low (weight 512, +2048/slice) has completed 2 slices (4:2:1 ratio).
  EXPECT_THAT(shared.slices_when_high_finished[0], t::Eq(8));
  EXPECT_THAT(shared.slices_when_high_finished[1], t::Eq(4));
  EXPECT_THAT(shared.slices_when_high_finished[2], t::Eq(2));

  int high_finish_pos = -1;
  int med_finish_pos = -1;
  int low_finish_pos = -1;
  for (int i = 0; i < static_cast<int>(shared.dispatch_order.size()); ++i) {
    if (shared.dispatch_order[i] == 0) {
      high_finish_pos = i;
    } else if (shared.dispatch_order[i] == 1) {
      med_finish_pos = i;
    } else if (shared.dispatch_order[i] == 2) {
      low_finish_pos = i;
    }
  }
  EXPECT_THAT(high_finish_pos, t::Lt(med_finish_pos));
  EXPECT_THAT(med_finish_pos, t::Lt(low_finish_pos));

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));

  TaskResetForTest();
}

struct MinVruntimeFloorCtx {
  std::vector<int> create_trace;
  std::vector<int> migrate_trace;
  Task* newborn = nullptr;
};

static void NewbornFloorWorker(void* const raw_arg) {
  MinVruntimeFloorCtx* const ctx = static_cast<MinVruntimeFloorCtx*>(raw_arg);
  TaskScheduler* const sched = GetTaskScheduler();
  for (int i = 0; i < 4; ++i) {
    ctx->create_trace.push_back(2);
    sched->Yield();
  }
}

static void VeteranFloorWorker(void* const raw_arg) {
  MinVruntimeFloorCtx* const ctx = static_cast<MinVruntimeFloorCtx*>(raw_arg);
  TaskScheduler* const sched = GetTaskScheduler();

  // Advance CPU 0's min_vruntime by 10 slices before creating `newborn`.
  for (int i = 0; i < 10; ++i) {
    sched->Yield();
  }

  ctx->newborn =
      sched->CreateTaskOnCpu(NewbornFloorWorker, ctx, kDefaultTaskWeight, 0);
  for (int i = 0; i < 4; ++i) {
    ctx->create_trace.push_back(1);
    sched->Yield();
  }
}

static void MigratedFloorWorker(void* const raw_arg) {
  MinVruntimeFloorCtx* const ctx = static_cast<MinVruntimeFloorCtx*>(raw_arg);
  TaskScheduler* const sched = GetTaskScheduler();
  for (int i = 0; i < 4; ++i) {
    ctx->migrate_trace.push_back(20);
    sched->Yield();
  }
}

static void LocalPeerFloorWorker(void* const raw_arg) {
  MinVruntimeFloorCtx* const ctx = static_cast<MinVruntimeFloorCtx*>(raw_arg);
  TaskScheduler* const sched = GetTaskScheduler();
  for (int i = 0; i < 4; ++i) {
    ctx->migrate_trace.push_back(10);
    sched->Yield();
  }
}

TEST(TaskTest, MonotonicMinVruntimeFloorPreventsStarvationOnCreateAndMigrate) {
  SetupEnv(2);
  TaskInit();
  TaskScheduler* const sched = GetTaskScheduler();

  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  MinVruntimeFloorCtx ctx;

  // (a) Creation floor: `veteran` advances CPU 0's min_vruntime by 10 yields,
  //     then spawns `newborn` on CPU 0. Because `newborn`'s vruntime is
  //     clamped to CPU 0's min_vruntime (instead of 0), `newborn` does not
  //     starve `veteran` for 4 consecutive slices — they interleave.
  Task* const veteran =
      sched->CreateTaskOnCpu(VeteranFloorWorker, &ctx, kDefaultTaskWeight, 0);
  sched->Join(veteran);
  ASSERT_THAT(ctx.newborn, t::NotNull());
  sched->Join(ctx.newborn);

  ASSERT_THAT(static_cast<int>(ctx.create_trace.size()), t::Eq(8));
  // Verify `newborn` (2) never runs 3+ times consecutively while `veteran` (1)
  // is runnable.
  int max_consecutive_newborn = 0;
  int current_run = 0;
  for (const int id : ctx.create_trace) {
    if (id == 2) {
      ++current_run;
      if (current_run > max_consecutive_newborn) {
        max_consecutive_newborn = current_run;
      }
    } else {
      current_run = 0;
    }
  }
  EXPECT_THAT(max_consecutive_newborn, t::Le(2));

  // (b) Work-stealing migration floor: create `remote_task` on CPU 1 (where
  //     min_vruntime == 0), then steal it onto CPU 0 (where min_vruntime > 0)
  //     and run it alongside `local_peer` on CPU 0. Because `StealTask(0, 1)`
  //     normalizes `remote_task`'s vruntime relative to CPU 0's min_vruntime,
  //     `remote_task` (20) interleaves with `local_peer` (10) instead of
  //     monopolizing CPU 0 for all 4 slices.
  Task* const remote_task =
      sched->CreateTaskOnCpu(MigratedFloorWorker, &ctx, kDefaultTaskWeight, 1);
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*sched, 1), t::Eq(1));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*sched, 0), t::Eq(0));
  EXPECT_TRUE(TaskSchedulerInternals::StealTask(*sched, 0, 1));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*sched, 1), t::Eq(0));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*sched, 0), t::Eq(1));

  Task* const local_peer =
      sched->CreateTaskOnCpu(LocalPeerFloorWorker, &ctx, kDefaultTaskWeight, 0);
  sched->Join(remote_task);
  sched->Join(local_peer);

  ASSERT_THAT(static_cast<int>(ctx.migrate_trace.size()), t::Eq(8));
  int max_consecutive_migrated = 0;
  current_run = 0;
  for (const int id : ctx.migrate_trace) {
    if (id == 20) {
      ++current_run;
      if (current_run > max_consecutive_migrated) {
        max_consecutive_migrated = current_run;
      }
    } else {
      current_run = 0;
    }
  }
  EXPECT_THAT(max_consecutive_migrated, t::Le(2));

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));

  TaskResetForTest();
}

TEST(TaskTest, LeastLoadedPlacementAndCrossCpuWakeupIpi) {
  SetupEnv(4);
  TaskInit();
  TaskScheduler* const sched = GetTaskScheduler();

  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  // Create 8 tasks via CreateTask. Deterministic lowest-cpu_id tie-breaking
  // must place 2 tasks on each of CPUs 0, 1, 2, 3.
  std::vector<Task*> tasks;
  tasks.reserve(8);
  for (int i = 0; i < 8; ++i) {
    Task* const t = sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight);
    ASSERT_THAT(t, t::NotNull());
    tasks.push_back(t);
  }

  for (int c = 0; c < 4; ++c) {
    EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*sched, c), t::Eq(2));
  }

  // Enqueuing on local CPU 0 must not send a self-IPI; enqueuing on remote
  // CPUs 1, 2, 3 must send kVectorWakeupIpi (0x21) once per placement.
  EXPECT_THAT(g_env.ipi_sent_count[0].load(), t::Eq(0));
  for (int c = 1; c < 4; ++c) {
    EXPECT_THAT(g_env.ipi_sent_count[c].load(), t::Eq(2));
    EXPECT_THAT(g_env.last_ipi_vector[c].load(), t::Eq(kVectorWakeupIpi));
  }

  // Join all 8 tasks (CPU 0 steals the tasks from CPUs 1..3 as its local
  // runqueue drains).
  for (Task* const t : tasks) {
    sched->Join(t);
  }

  for (int c = 0; c < 4; ++c) {
    EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*sched, c), t::Eq(0));
  }
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));

  TaskResetForTest();
}

static void YieldFourTimesWorker(void* const raw_arg) {
  int* const counter = static_cast<int*>(raw_arg);
  TaskScheduler* const sched = GetTaskScheduler();
  for (int i = 0; i < 4; ++i) {
    ++(*counter);
    sched->Yield();
  }
}

TEST(TaskTest, JoinYieldsUntilTargetExitsAndDoesNotYieldForAnExitedTarget) {
  SetupEnv(1);
  TaskInit();
  TaskScheduler* const sched = GetTaskScheduler();

  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  // Case 1: Target has not yet exited when Join is called. Join keeps
  // yielding (the bootstrap task stays runnable) until it has.
  int slices = 0;
  Task* const running_target =
      sched->CreateTask(YieldFourTimesWorker, &slices, kDefaultTaskWeight);
  sched->Join(running_target);
  EXPECT_THAT(slices, t::Eq(4));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));

  // Case 2: Target already exited (and its CPU switched away from it) before
  // Join is called.
  bool finished = false;
  Task* const exited_target =
      sched->CreateTask(SetFlagWorker, &finished, kDefaultTaskWeight);
  while (!finished) {
    sched->Yield();
  }

  // Enqueue a runnable probe task on CPU 0; joining the already-exited
  // `exited_target` must reap it immediately without yielding to `probe_task`.
  int probe_runs = 0;
  Task* const probe_task = sched->CreateTask(
      [](void* const arg) { ++(*static_cast<int*>(arg)); },  //
      &probe_runs,                                           //
      kDefaultTaskWeight);
  sched->Join(exited_target);
  EXPECT_THAT(probe_runs, t::Eq(0));

  sched->Join(probe_task);
  EXPECT_THAT(probe_runs, t::Eq(1));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));

  TaskResetForTest();
}

TEST(TaskTest, ExitFreesStackOnSwitchAndDetachedTaskIsReapedLater) {
  SetupEnv(1);
  TaskInit();
  TaskScheduler* const sched = GetTaskScheduler();

  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  bool task_ran = false;
  Task* const zombie =
      sched->CreateTask(SetFlagWorker, &task_ran, kDefaultTaskWeight);
  ASSERT_THAT(zombie, t::NotNull());
  // Stack frames and TCB are currently allocated.
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before - kTaskStackFrames));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Lt(heap_before));

  while (!task_ran) {
    sched->Yield();
  }

  // Immediately upon switching back to the bootstrap task, FinishContextSwitch
  // on the bootstrap stack MUST have already freed `zombie`'s kernel stack back
  // to the PMM, while `zombie`'s TCB remains allocated until reclaimed.
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Lt(heap_before));

  // ReapZombies() leaves an exited task alone until it is detached: its owner
  // may still join it.
  EXPECT_THAT(sched->ReapZombies(), t::Eq(0));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Lt(heap_before));

  // Once detached, ReapZombies() frees its TCB and join record.
  sched->Detach(zombie);
  EXPECT_THAT(sched->ReapZombies(), t::Eq(1));
  EXPECT_THAT(sched->ReapZombies(), t::Eq(0));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));

  // A task detached before it exits is reaped after it exits.
  bool detached_ran = false;
  Task* const detached =
      sched->CreateTask(SetFlagWorker, &detached_ran, kDefaultTaskWeight);
  ASSERT_THAT(detached, t::NotNull());
  sched->Detach(detached);
  EXPECT_THAT(sched->ReapZombies(), t::Eq(0));
  while (!detached_ran) {
    sched->Yield();
  }
  EXPECT_THAT(sched->ReapZombies(), t::Eq(1));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));

  TaskResetForTest();
}

struct ReaperCtx {
  std::atomic<bool> stop{false};
  std::atomic<int> passes{0};
  std::atomic<int> reaped{0};
};

static void ReaperWorker(void* const raw_arg) {
  ReaperCtx* const ctx = static_cast<ReaperCtx*>(raw_arg);
  TaskScheduler* const sched = GetTaskScheduler();
  while (!ctx->stop.load(std::memory_order_acquire)) {
    ctx->reaped.fetch_add(sched->ReapZombies(), std::memory_order_acq_rel);
    ctx->passes.fetch_add(1, std::memory_order_acq_rel);
    sched->Yield();
  }
}

TEST(TaskTest, ReaperTaskNeverReclaimsAnExitedTaskThatAwaitsJoin) {
  SetupEnv(1);
  TaskInit();
  TaskScheduler* const sched = GetTaskScheduler();

  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  // A periodic reaper task runs next to a joinable and a detached child.
  ReaperCtx reaper_ctx;
  Task* const reaper =
      sched->CreateTask(ReaperWorker, &reaper_ctx, kDefaultTaskWeight);
  bool joinable_ran = false;
  bool detached_ran = false;
  Task* const joinable =
      sched->CreateTask(SetFlagWorker, &joinable_ran, kDefaultTaskWeight);
  Task* const detached =
      sched->CreateTask(SetFlagWorker, &detached_ran, kDefaultTaskWeight);
  ASSERT_THAT(reaper, t::NotNull());
  ASSERT_THAT(joinable, t::NotNull());
  ASSERT_THAT(detached, t::NotNull());
  sched->Detach(detached);

  // Both children exit, then the reaper makes several more passes before the
  // creator gets around to joining `joinable`.
  while (!joinable_ran || !detached_ran) {
    sched->Yield();
  }
  const int passes_after_exit = reaper_ctx.passes.load();
  while (reaper_ctx.passes.load() < passes_after_exit + 3) {
    sched->Yield();
  }

  // Only the detached child was reaped; `joinable` is still a valid handle.
  EXPECT_THAT(reaper_ctx.reaped.load(), t::Eq(1));
  sched->Join(joinable);

  reaper_ctx.stop.store(true, std::memory_order_release);
  sched->Join(reaper);
  EXPECT_THAT(reaper_ctx.reaped.load(), t::Eq(1));
  EXPECT_THAT(sched->ReapZombies(), t::Eq(0));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));

  TaskResetForTest();
}

struct PreemptTimerCtx {
  int64_t eoi_observed_in_worker = 0;
  bool worker_ran = false;
};

static void PreemptTargetWorker(void* const raw_arg) {
  PreemptTimerCtx* const ctx = static_cast<PreemptTimerCtx*>(raw_arg);
  // Record EOI count while inside the switched-in task (before the original
  // interrupted frame's IdtDispatch has returned!).
  ctx->eoi_observed_in_worker = g_env.eoi_count.load(std::memory_order_acquire);
  ctx->worker_ran = true;
}

TEST(TaskTest, PreemptiveLocalApicTimerChargesVruntimeAndPreemptsWhenSafe) {
  SetupEnv(1);
  TaskInit();
  TaskScheduler* const sched = GetTaskScheduler();

  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  // `worker` starts at the bootstrap task's vruntime, so it can only preempt
  // the bootstrap task after a tick has charged the bootstrap task.
  PreemptTimerCtx ctx;
  Task* const worker =
      sched->CreateTask(PreemptTargetWorker, &ctx, kDefaultTaskWeight);
  ASSERT_THAT(worker, t::NotNull());

  InterruptFrame timer_frame = MakeInterruptFrame(kVectorApicTimer);

  // 1. When a spinlock is held, timer interrupt MUST NOT preempt
  //    mid-critical-section.
  {
    IrqSpinLock critical_lock;
    const IrqSpinLockGuard guard(critical_lock);
    IdtDispatch(&timer_frame);
    EXPECT_FALSE(ctx.worker_ran);
  }

  // 2. When interrupts are disabled in the interrupted frame (IF == 0) or
  //    preemption is disabled, timer interrupt MUST NOT preempt.
  InterruptFrame cli_timer_frame = MakeInterruptFrame(kVectorApicTimer);
  cli_timer_frame.rflags = 0x2ULL;  // RFLAGS.IF == 0
  IdtDispatch(&cli_timer_frame);
  EXPECT_FALSE(ctx.worker_ran);

  sched->SetPreemptEnabled(false);
  IdtDispatch(&timer_frame);
  EXPECT_FALSE(ctx.worker_ran);
  sched->SetPreemptEnabled(true);

  // 3. With preemption enabled and no spinlocks held, `IdtDispatch` sends EOI
  //    *before* context switching, charges a tick of vruntime to the bootstrap
  //    task (without that charge `worker` would not be ahead of it), and
  //    preempts into `worker`.
  const int64_t eoi_before = g_env.eoi_count.load();
  IdtDispatch(&timer_frame);

  EXPECT_TRUE(ctx.worker_ran);
  EXPECT_THAT(ctx.eoi_observed_in_worker, t::Ge(eoi_before + 1));

  sched->Join(worker);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));

  TaskResetForTest();
}

struct MtStealSharedCtx {
  std::atomic<uint32_t> cpu_mask{0};
  std::atomic<int> completed_tasks{0};
  std::atomic<int64_t> total_iterations{0};
  std::atomic<int64_t> remote_executions{0};
};

struct MtStealTaskArg {
  MtStealSharedCtx* shared = nullptr;
  int task_index = 0;
  std::atomic<int> iterations{0};
};

static void MtStealWorker(void* const raw_arg) {
  MtStealTaskArg* const arg = static_cast<MtStealTaskArg*>(raw_arg);
  MtStealSharedCtx* const shared = arg->shared;
  TaskScheduler* const sched = GetTaskScheduler();

  for (int round = 0; round < 6; ++round) {
    const int cpu = CurrentCpuId();
    shared->cpu_mask.fetch_or(1u << static_cast<uint32_t>(cpu),
                              std::memory_order_acq_rel);
    if (cpu != 0) {
      shared->remote_executions.fetch_add(1, std::memory_order_acq_rel);
    }
    arg->iterations.fetch_add(1, std::memory_order_acq_rel);
    shared->total_iterations.fetch_add(1, std::memory_order_acq_rel);
    sched->Yield();
  }

  const int cpu = CurrentCpuId();
  shared->cpu_mask.fetch_or(1u << static_cast<uint32_t>(cpu),
                            std::memory_order_acq_rel);
  if (cpu != 0) {
    shared->remote_executions.fetch_add(1, std::memory_order_acq_rel);
  }
  shared->completed_tasks.fetch_add(1, std::memory_order_acq_rel);
}

TEST(TaskTest, ConcurrentMultiThreadedWorkStealingAndContentionUnderTsan) {
  constexpr int kNumCpus = 4;
  constexpr int kNumTasks = 24;
  SetupEnv(kNumCpus);
  TaskInit();
  TaskScheduler* const sched = GetTaskScheduler();
  Task* const bootstrap = sched->CurrentTask();

  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  MtStealSharedCtx shared;
  std::unique_ptr<MtStealTaskArg[]> args(new MtStealTaskArg[kNumTasks]);
  std::vector<Task*> tasks;
  tasks.reserve(kNumTasks);

  // Enqueue all 24 tasks onto CPU 0 so CPUs 1..3 must steal work using the
  // ascending cpu_id dual-runqueue lock protocol.
  for (int i = 0; i < kNumTasks; ++i) {
    args[i].shared = &shared;
    args[i].task_index = i;
    Task* const t = sched->CreateTaskOnCpu(MtStealWorker,               //
                                           &args[i],                    //
                                           kDefaultTaskWeight + i * 8,  //
                                           0);
    ASSERT_THAT(t, t::NotNull());
    tasks.push_back(t);
  }
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*sched, 0),
              t::Eq(kNumTasks));

  std::atomic<int> ready_workers{0};
  std::atomic<bool> stop_workers{false};
  std::vector<std::thread> ap_threads;
  ap_threads.reserve(kNumCpus - 1);

  for (int cpu = 1; cpu < kNumCpus; ++cpu) {
    ap_threads.emplace_back([cpu, &ready_workers, &stop_workers]() {
      BindCpuLocal(SmpGetCpuLocal(cpu));
      SetInterruptsEnabledForTest(true);
      ready_workers.fetch_add(1, std::memory_order_acq_rel);

      while (!stop_workers.load(std::memory_order_acquire)) {
        if (!TaskPollIdleCpu(cpu)) {
          std::this_thread::yield();
        }
      }
      ResetCpuLocalForTest();
    });
  }

  while (ready_workers.load(std::memory_order_acquire) < (kNumCpus - 1)) {
    std::this_thread::yield();
  }

  // Wait until an AP has stolen and run a slice before CPU 0 starts running
  // the tasks itself. While the bootstrap task waits here, outside the
  // scheduler, nothing runs on CPU 0, so the cross-CPU checks below don't
  // depend on how the host schedules the AP threads.
  while (shared.remote_executions.load(std::memory_order_acquire) == 0) {
    std::this_thread::yield();
  }

  for (Task* const t : tasks) {
    sched->Join(t);
  }

  stop_workers.store(true, std::memory_order_release);
  for (std::thread& th : ap_threads) {
    th.join();
  }

  EXPECT_THAT(CurrentCpuId(), t::Eq(0));
  EXPECT_THAT(sched->CurrentTask(), t::Eq(bootstrap));
  EXPECT_THAT(shared.completed_tasks.load(), t::Eq(kNumTasks));
  EXPECT_THAT(shared.total_iterations.load(), t::Eq(kNumTasks * 6));
  for (int i = 0; i < kNumTasks; ++i) {
    EXPECT_THAT(args[i].iterations.load(), t::Eq(6));
  }

  // Verify that cross-CPU work stealing occurred and multiple CPUs executed
  // stolen tasks.
  EXPECT_THAT(shared.remote_executions.load(), t::Gt(0));
  EXPECT_THAT(shared.cpu_mask.load() & ~1u, t::Ne(0u));

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));

  TaskResetForTest();
}

struct PreemptedSyscallCtx {
  std::atomic<bool> in_syscall{false};
  std::atomic<bool> done{false};
  int entry_cpu = -1;
  int exit_cpu = -1;
  int timer_interrupts = 0;
  bool interrupts_enabled_after_return = true;
  uint64_t rsp0_after_return = 0;
  uintptr_t frame_top = 0;
  uint64_t rax = 0;
};

// The context of `PreemptibleSyscallHandler()`, which can't take an argument.
PreemptedSyscallCtx* g_preempted_syscall = nullptr;

// Stands in for UserProcess::OnSyscall(): serves the call with interrupts
// enabled, taking timer interrupts until one of them has switched the task out
// and the task has resumed on another CPU, and returns `rdi + 1`.
static void PreemptibleSyscallHandler(InterruptFrame* const frame) {
  PreemptedSyscallCtx* const ctx = g_preempted_syscall;
  ctx->entry_cpu = CurrentCpuIdIrqsOff();
  RestoreInterrupts((frame->rflags & kRflagsInterruptEnableBit) != 0);
  ctx->in_syscall.store(true, std::memory_order_release);
  for (int i = 0; i < 100 && CurrentCpuId() == ctx->entry_cpu; ++i) {
    InterruptFrame timer_frame = MakeInterruptFrame(kVectorApicTimer);
    // As the interrupt gate and `iretq` do.
    SetInterruptsEnabledForTest(false);
    IdtDispatch(&timer_frame);
    SetInterruptsEnabledForTest(true);
    ++ctx->timer_interrupts;
  }
  ctx->exit_cpu = CurrentCpuId();
  frame->rax = frame->rdi + 1;
}

// Makes a system call from "user mode" (a frame with a user code segment).
static void SyscallTaskWorker(void* const raw_ctx) {
  PreemptedSyscallCtx* const ctx = static_cast<PreemptedSyscallCtx*>(raw_ctx);
  alignas(16) InterruptFrame frame = MakeInterruptFrame(kVectorSyscall);
  frame.cs = kGdtUserCodeSelector;
  frame.rdi = 41;
  // As the interrupt gate leaves them.
  SetInterruptsEnabledForTest(false);
  IdtDispatch(&frame);
  ctx->interrupts_enabled_after_return = AreInterruptsEnabled();
  ctx->rsp0_after_return = GdtGetTss(CurrentCpuIdIrqsOff())->rsp[0];
  ctx->frame_top = reinterpret_cast<uintptr_t>(&frame + 1);
  ctx->rax = frame.rax;
  // `iretq` restores the interrupt flag of user mode.
  SetInterruptsEnabledForTest(true);
  ctx->done.store(true, std::memory_order_release);
}

TEST(TaskTest,
     SystemCallWithInterruptsEnabledIsPreemptedAndFinishesOnAnotherCpu) {
  SetupEnv(2);
  GdtResetForTest();
  TaskInit();
  TaskScheduler* const sched = GetTaskScheduler();
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();
  PreemptedSyscallCtx ctx;
  g_preempted_syscall = &ctx;
  IdtRegisterHandler(kVectorSyscall, PreemptibleSyscallHandler);

  Task* const task =
      sched->CreateTaskOnCpu(SyscallTaskWorker, &ctx, kDefaultTaskWeight, 0);
  ASSERT_THAT(task, t::NotNull());
  // The task enters the system call on CPU 0, where a timer interrupt
  // switches it out in the middle of it, back to this task.
  sched->Yield();
  EXPECT_TRUE(ctx.in_syscall.load(std::memory_order_acquire));
  EXPECT_FALSE(ctx.done.load(std::memory_order_acquire));
  EXPECT_THAT(TaskRunningCpu(task), t::Eq(-1));

  // CPU 1, idle, takes the task and resumes it, while this task stays out of
  // the scheduler.
  std::thread cpu1([&ctx]() {
    BindCpuLocal(SmpGetCpuLocal(1));
    SetInterruptsEnabledForTest(true);
    while (!ctx.done.load(std::memory_order_acquire)) {
      if (!TaskPollIdleCpu(1)) {
        std::this_thread::yield();
      }
    }
    ResetCpuLocalForTest();
  });
  cpu1.join();

  EXPECT_THAT(ctx.entry_cpu, t::Eq(0));
  EXPECT_THAT(ctx.exit_cpu, t::Eq(1));
  EXPECT_THAT(ctx.timer_interrupts, t::Ge(1));
  // CPU 1 returned to user mode with interrupts disabled, its TSS.rsp0 right
  // above the frame; CPU 0 never returned to user mode.
  EXPECT_FALSE(ctx.interrupts_enabled_after_return);
  EXPECT_THAT(ctx.rsp0_after_return, t::Eq(ctx.frame_top));
  const uint64_t cpu0_rsp0 = GdtGetTss(0)->rsp[0];
  EXPECT_THAT(cpu0_rsp0, t::Eq(0u));
  EXPECT_THAT(ctx.rax, t::Eq(42u));
  // CPU 0 switched to the task and away from it; CPU 1 to it and back to its
  // idle loop.
  EXPECT_THAT(SmpGetCpuLocal(0)->context_switches.load(), t::Ge(2));
  EXPECT_THAT(SmpGetCpuLocal(1)->context_switches.load(), t::Ge(2));

  sched->Join(task);
  IdtUnregisterHandler(kVectorSyscall);
  g_preempted_syscall = nullptr;
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
  TaskResetForTest();
}

TEST(TaskDeathTest, PreconditionAndInvariantViolationsTriggerDcheck) {
  SetupEnv(2);
  TaskInit();
  TaskScheduler* const sched = GetTaskScheduler();

  // Invalid CreateTask / CreateTaskOnCpu arguments must trigger DCHECK.
  EXPECT_DEATH(sched->CreateTask(nullptr, nullptr, kDefaultTaskWeight),
               "Check failed");
  EXPECT_DEATH(sched->CreateTask(NoopTask, nullptr, 0), "Check failed");
  EXPECT_DEATH(sched->CreateTask(NoopTask, nullptr, -5), "Check failed");
  EXPECT_DEATH(sched->CreateTask(NoopTask, nullptr, kMaxTaskWeight + 1),
               "Check failed");
  EXPECT_DEATH(
      sched->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, -1),
      "Check failed");
  EXPECT_DEATH(sched->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 2),
               "Check failed");

  // Invalid Join calls must trigger DCHECK.
  EXPECT_DEATH(sched->Join(nullptr), "Check failed");
  EXPECT_DEATH(sched->Join(sched->CurrentTask()), "Check failed");

  // Using a handle after Detach() must trigger DCHECK.
  Task* const task = sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight);
  ASSERT_THAT(task, t::NotNull());
  EXPECT_DEATH(
      {
        sched->Detach(task);
        sched->Join(task);
      },
      "Check failed");
  EXPECT_DEATH(
      {
        sched->Detach(task);
        sched->Detach(task);
      },
      "Check failed");
  sched->Join(task);

  // Yielding while holding an IrqSpinLock must trigger DCHECK.
  EXPECT_DEATH(
      {
        IrqSpinLock lock;
        const IrqSpinLockGuard guard(lock);
        sched->Yield();
      },
      "Check failed");

  TaskResetForTest();
}

TEST(TaskDeathTest, TaskInitRequiresTheIdtAndRunsOnlyOnce) {
  SetupEnv(1);
  TaskInit();
  EXPECT_DEATH(TaskInit(), "Check failed");

  TaskResetForTest();
  IdtResetForTest();
  EXPECT_DEATH(TaskInit(), "Check failed");
  IdtInit();
}

}  // namespace
}  // namespace protos
