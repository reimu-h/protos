#include "core_runqueue_scheduler.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <new>
#include <string>
#include <thread>
#include <vector>

#include "scheduler_test_support.h"
#include "task_scheduler_internals.h"

namespace protos {
namespace {

namespace t = ::testing;

// Returns the length of the longest run of consecutive `id` entries in `trace`.
static int MaxConsecutiveRun(const std::vector<int>& trace, const int id) {
  int longest = 0;
  int current = 0;
  for (const int entry : trace) {
    current = (entry == id) ? current + 1 : 0;
    if (current > longest) {
      longest = current;
    }
  }
  return longest;
}

// ============================================================================
// 1. `Init()` and Topology Initialization (1..4 and 128 CPUs)
// ============================================================================

TEST(CoreRunqueueSchedulerTest,
     InitAdoptsBootstrapTaskAndInitializesPerCpuRunqueuesAcrossTopologies) {
  for (const int num_cpus : {1, 2, 3, 4, 128}) {
    SetupEnv(num_cpus);
    EXPECT_TRUE(IdtIsInitialized());

    const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
    ASSERT_THAT(core, t::NotNull());
    // Before Init(), PollIdleCpu must safely return false and RunqueueLoad
    // must report an empty runqueue.
    EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*core, 0));
    EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(0));

    TaskSchedulerInternals::Init(*core);

    EXPECT_THAT(CurrentCpuId(), t::Eq(0));

    // Preemption state on the core layer is always enabled and
    // SetPreemptEnabled is a safe no-op.
    EXPECT_TRUE(core->IsPreemptEnabled());
    core->SetPreemptEnabled(false);
    EXPECT_TRUE(core->IsPreemptEnabled());
    core->SetPreemptEnabled(true);
    EXPECT_TRUE(core->IsPreemptEnabled());

    Task* const bootstrap = core->CurrentTask();
    ASSERT_THAT(bootstrap, t::NotNull());

    for (int c = 0; c < num_cpus; ++c) {
      EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, c), t::Eq(0));
      EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*core, c));
    }
    // Out-of-range PollIdleCpu returns false safely.
    EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*core, -1));
    EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*core, num_cpus));

    // Verify each AP (1 .. num_cpus - 1) has its own distinct idle task bound
    // as current_task.
    std::vector<Task*> initial_tasks;
    initial_tasks.push_back(bootstrap);
    for (int c = 1; c < num_cpus; ++c) {
      BindCpuLocal(SmpGetCpuLocal(c));
      Task* const ap_idle = core->CurrentTask();
      ASSERT_THAT(ap_idle, t::NotNull());
      for (Task* const prev_task : initial_tasks) {
        EXPECT_THAT(ap_idle, t::Ne(prev_task));
      }
      initial_tasks.push_back(ap_idle);
    }
    BindCpuLocal(SmpGetCpuLocal(0));
  }
}

// ============================================================================
// 2. `CreateTask`, `CreateTaskOnCpu`, Stack Alignment, `Yield`, Task Return
// ============================================================================

struct LifecycleTaskCtx {
  TaskScheduler* sched = nullptr;
  std::vector<int> trace;
  int task_index = 0;
  bool stack_aligned = false;
  Task* observed_task = nullptr;
  int observed_cpu = -1;
};

static void LifecycleWorker(void* const raw_arg) {
  LifecycleTaskCtx* const ctx = static_cast<LifecycleTaskCtx*>(raw_arg);
  ASSERT_THAT(ctx, t::NotNull());
  ASSERT_THAT(ctx->sched, t::NotNull());

  alignas(16) uint64_t probe = 0;
  ctx->stack_aligned = ((reinterpret_cast<uintptr_t>(&probe) & 0xFu) == 0);
  ctx->observed_task = ctx->sched->CurrentTask();
  ctx->observed_cpu = CurrentCpuId();

  for (int step = 0; step < 3; ++step) {
    ctx->trace.push_back(ctx->task_index * 10 + step);
    ctx->sched->Yield();
  }
}

TEST(CoreRunqueueSchedulerTest,
     CreateTaskAndCreateTaskOnCpuVerifyPlacementAlignmentIdentityAndYield) {
  SetupEnv(3);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  LifecycleTaskCtx ctx_a;
  ctx_a.sched = core.get();
  ctx_a.task_index = 1;

  LifecycleTaskCtx ctx_b;
  ctx_b.sched = core.get();
  ctx_b.task_index = 2;

  LifecycleTaskCtx ctx_remote;
  ctx_remote.sched = core.get();
  ctx_remote.task_index = 3;

  // CreateTask always places onto CPU 0's runqueue.
  Task* const task_a =
      core->CreateTask(LifecycleWorker, &ctx_a, kDefaultTaskWeight);
  Task* const task_b = core->CreateTaskOnCpu(LifecycleWorker,     //
                                             &ctx_b,              //
                                             kDefaultTaskWeight,  //
                                             0);
  // CreateTaskOnCpu places onto the explicit target_cpu (CPU 2) without sending
  // any IPIs (IPIs belong to WorkStealingScheduler).
  Task* const task_c = core->CreateTaskOnCpu(LifecycleWorker,     //
                                             &ctx_remote,         //
                                             kDefaultTaskWeight,  //
                                             2);

  ASSERT_THAT(task_a, t::NotNull());
  ASSERT_THAT(task_b, t::NotNull());
  ASSERT_THAT(task_c, t::NotNull());
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(2));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 1), t::Eq(0));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 2), t::Eq(1));
  for (int c = 0; c < 3; ++c) {
    EXPECT_THAT(g_env.ipi_sent_count[c].load(), t::Eq(0));
  }

  core->Join(task_a);
  core->Join(task_b);
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(0));

  EXPECT_TRUE(ctx_a.stack_aligned);
  EXPECT_THAT(ctx_a.observed_task, t::Eq(task_a));
  EXPECT_THAT(ctx_a.observed_cpu, t::Eq(0));
  EXPECT_THAT(ctx_a.trace, t::ElementsAre(10, 11, 12));

  EXPECT_TRUE(ctx_b.stack_aligned);
  EXPECT_THAT(ctx_b.observed_task, t::Eq(task_b));
  EXPECT_THAT(ctx_b.observed_cpu, t::Eq(0));
  EXPECT_THAT(ctx_b.trace, t::ElementsAre(20, 21, 22));

  // Steal `task_c` from CPU 2 onto CPU 0 and run it to completion.
  EXPECT_TRUE(TaskSchedulerInternals::StealTask(*core, 0, 2));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 2), t::Eq(0));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(1));
  core->Join(task_c);

  EXPECT_TRUE(ctx_remote.stack_aligned);
  EXPECT_THAT(ctx_remote.observed_task, t::Eq(task_c));
  EXPECT_THAT(ctx_remote.observed_cpu, t::Eq(0));
  EXPECT_THAT(ctx_remote.trace, t::ElementsAre(30, 31, 32));

  // Verify `CurrentTask()` and `Yield()` disable interrupts internally before
  // querying `CurrentCpuIdIrqsOff()` and restore the caller's prior interrupt
  // state (both when called with interrupts enabled and when disabled).
  SetInterruptsEnabledForTest(true);
  Task* const curr_irq_on = core->CurrentTask();
  EXPECT_TRUE(AreInterruptsEnabled());
  core->Yield();
  EXPECT_TRUE(AreInterruptsEnabled());

  SetInterruptsEnabledForTest(false);
  Task* const curr_irq_off = core->CurrentTask();
  EXPECT_FALSE(AreInterruptsEnabled());
  core->Yield();
  EXPECT_FALSE(AreInterruptsEnabled());
  SetInterruptsEnabledForTest(true);
  EXPECT_THAT(curr_irq_on, t::Eq(curr_irq_off));

  // Verify PollIdleCpu returns false if called for a different CPU than
  // CurrentCpuId() or when the target CPU is offline, even when tasks are
  // queued on that remote CPU.
  Task* const queued_on_cpu2 =
      core->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 2);
  ASSERT_THAT(queued_on_cpu2, t::NotNull());
  EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*core, 2));
  g_env.cpu_locals[2].online = false;
  BindCpuLocal(SmpGetCpuLocal(2));
  EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*core, 2));
  g_env.cpu_locals[2].online = true;
  BindCpuLocal(SmpGetCpuLocal(0));
  EXPECT_TRUE(TaskSchedulerInternals::StealTask(*core, 0, 2));
  core->Join(queued_on_cpu2);

  // Verify CreateTaskOnCpu returns nullptr without leaking heap or PMM memory
  // when either Kmalloc(sizeof(Task)) or PmmAllocFrames(kTaskStackFrames) fails
  // due to OOM.
  const int64_t free_pmm_baseline = PmmFreeFrameCount();
  const int64_t free_heap_baseline = HeapTotalFreeBytes();

  // Case 1: PMM stack allocation fails while heap still has room for Task TCB.
  g_env.fail_pmm_alloc = true;
  EXPECT_THAT(core->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::IsNull());
  g_env.fail_pmm_alloc = false;
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(free_pmm_baseline));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_heap_baseline));

  // Case 2: Heap TCB allocation fails.
  g_env.fail_kmalloc = true;
  EXPECT_THAT(core->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::IsNull());
  g_env.fail_kmalloc = false;

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// ============================================================================
// 3. Weighted `RbTree` `vruntime` Ordering and Inverse-Weight Scaling
// ============================================================================

struct WeightedShareCtx {
  TaskScheduler* sched = nullptr;
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

  for (int i = 0; i < 8; ++i) {
    shared->dispatch_order.push_back(idx);
    ++shared->slices_for_task[idx];
    if (idx == 0 && i == 7) {
      for (int k = 0; k < 3; ++k) {
        shared->slices_when_high_finished[k] = shared->slices_for_task[k];
      }
    }
    shared->sched->Yield();
  }
}

TEST(CoreRunqueueSchedulerTest,
     WeightedRbTreeVruntimeOrderingProportionalShareAndFifoTieBreaking) {
  SetupEnv(1);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  WeightedShareCtx shared;
  shared.sched = core.get();
  WeightedShareTaskArg args[3] = {
      {&shared, 0},
      {&shared, 1},
      {&shared, 2},
  };

  // Create three tasks on CPU 0 with weights 2048 (2x), 1024 (1x), and 512
  // (0.5x). All three start at min_vruntime == 0 and are ordered initially by
  // FIFO enqueue_seq (1, 2, 3).
  Task* const t_high =
      core->CreateTask(WeightedShareWorker, &args[0], kDefaultTaskWeight * 2);
  Task* const t_med =
      core->CreateTask(WeightedShareWorker, &args[1], kDefaultTaskWeight);
  Task* const t_low =
      core->CreateTask(WeightedShareWorker, &args[2], kDefaultTaskWeight / 2);

  core->Join(t_high);
  core->Join(t_med);
  core->Join(t_low);

  // The first 3 dispatches must follow creation order (FIFO enqueue_seq
  // tie-breaking) because all 3 started at vruntime == 0.
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
}

// ============================================================================
// 4. `min_vruntime` Floor on Create and `vruntime` Rebasing on `StealTask`
// ============================================================================

struct MinVruntimeFloorCtx {
  TaskScheduler* sched = nullptr;
  std::vector<int> create_trace;
  std::vector<int> migrate_trace;
  Task* newborn = nullptr;
  bool steal_from_high_cpu_result = false;
};

static void NewbornFloorWorker(void* const raw_arg) {
  MinVruntimeFloorCtx* const ctx = static_cast<MinVruntimeFloorCtx*>(raw_arg);
  for (int i = 0; i < 4; ++i) {
    ctx->create_trace.push_back(2);
    ctx->sched->Yield();
  }
}

static void VeteranFloorWorker(void* const raw_arg) {
  MinVruntimeFloorCtx* const ctx = static_cast<MinVruntimeFloorCtx*>(raw_arg);

  // Advance CPU 0's min_vruntime by 10 slices before creating `newborn`.
  for (int i = 0; i < 10; ++i) {
    ctx->sched->Yield();
  }

  ctx->newborn = ctx->sched->CreateTaskOnCpu(NewbornFloorWorker,  //
                                             ctx,                 //
                                             kDefaultTaskWeight,  //
                                             0);
  for (int i = 0; i < 4; ++i) {
    ctx->create_trace.push_back(1);
    ctx->sched->Yield();
  }
}

static void MigratedFloorWorker(void* const raw_arg) {
  MinVruntimeFloorCtx* const ctx = static_cast<MinVruntimeFloorCtx*>(raw_arg);
  for (int i = 0; i < 4; ++i) {
    ctx->migrate_trace.push_back(20);
    ctx->sched->Yield();
  }
}

static void LocalPeerFloorWorker(void* const raw_arg) {
  MinVruntimeFloorCtx* const ctx = static_cast<MinVruntimeFloorCtx*>(raw_arg);
  for (int i = 0; i < 4; ++i) {
    ctx->migrate_trace.push_back(10);
    ctx->sched->Yield();
  }
}

// Runs on CPU 1 and, while running there, steals a task from CPU 0 (it has to
// be running rather than queued: StealTask does nothing while the destination
// has queued work), then interleaves with it.
static void StealingPeerFloorWorker(void* const raw_arg) {
  MinVruntimeFloorCtx* const ctx = static_cast<MinVruntimeFloorCtx*>(raw_arg);
  ctx->steal_from_high_cpu_result =
      TaskSchedulerInternals::StealTask(*ctx->sched, 1, 0);
  for (int i = 0; i < 4; ++i) {
    ctx->migrate_trace.push_back(10);
    ctx->sched->Yield();
  }
}

TEST(CoreRunqueueSchedulerTest,
     MinVruntimeFloorOnCreationAndVruntimeRebasingOnCrossCpuSteal) {
  SetupEnv(2);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  MinVruntimeFloorCtx ctx;
  ctx.sched = core.get();

  // (a) Creation floor: `veteran` advances CPU 0's min_vruntime by 10 yields,
  //     then spawns `newborn` on CPU 0. Because `newborn`'s vruntime is
  //     clamped to CPU 0's min_vruntime (instead of 0), `newborn` does not
  //     starve `veteran` for 4 consecutive slices — they interleave.
  Task* const veteran =
      core->CreateTaskOnCpu(VeteranFloorWorker, &ctx, kDefaultTaskWeight, 0);
  core->Join(veteran);
  ASSERT_THAT(ctx.newborn, t::NotNull());
  core->Join(ctx.newborn);

  ASSERT_THAT(static_cast<int>(ctx.create_trace.size()), t::Eq(8));
  EXPECT_THAT(MaxConsecutiveRun(ctx.create_trace, 2), t::Le(2));

  // (b) Low-to-high migration: create `remote_task` on CPU 1 (where
  //     min_vruntime == 0), then steal it onto CPU 0 (where min_vruntime > 0)
  //     and run it alongside `local_peer` on CPU 0. Because `StealTask(0, 1)`
  //     rebases `remote_task`'s vruntime onto CPU 0's min_vruntime,
  //     `remote_task` (20) interleaves with `local_peer` (10) instead of
  //     starving it.
  Task* const remote_task =
      core->CreateTaskOnCpu(MigratedFloorWorker, &ctx, kDefaultTaskWeight, 1);
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 1), t::Eq(1));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(0));
  EXPECT_TRUE(TaskSchedulerInternals::StealTask(*core, 0, 1));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 1), t::Eq(0));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(1));

  Task* const local_peer =
      core->CreateTaskOnCpu(LocalPeerFloorWorker, &ctx, kDefaultTaskWeight, 0);
  core->Join(remote_task);
  core->Join(local_peer);

  ASSERT_THAT(static_cast<int>(ctx.migrate_trace.size()), t::Eq(8));
  EXPECT_THAT(MaxConsecutiveRun(ctx.migrate_trace, 20), t::Le(2));

  // (c) High-to-low migration: `cpu1_peer` is created on CPU 1 (min_vruntime
  //     still 0) and `from_high_cpu` on CPU 0, whose min_vruntime is now more
  //     than 10 slices. Impersonating CPU 1, the bootstrap task dispatches
  //     `cpu1_peer`, which steals `from_high_cpu` while running on CPU 1.
  //     `StealTask(1, 0)` rebases `from_high_cpu`'s vruntime onto CPU 1's
  //     min_vruntime, so the two interleave. Keeping (or clamping to) its CPU 0
  //     vruntime would instead let `cpu1_peer` (10) run all 4 of its slices
  //     before `from_high_cpu` (20) ran once.
  ctx.migrate_trace.clear();
  Task* const cpu1_peer = core->CreateTaskOnCpu(StealingPeerFloorWorker,  //
                                                &ctx,                     //
                                                kDefaultTaskWeight,       //
                                                1);
  Task* const from_high_cpu =
      core->CreateTaskOnCpu(MigratedFloorWorker, &ctx, kDefaultTaskWeight, 0);
  ASSERT_THAT(cpu1_peer, t::NotNull());
  ASSERT_THAT(from_high_cpu, t::NotNull());

  BindCpuLocal(SmpGetCpuLocal(1));
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*core, 1));
  BindCpuLocal(SmpGetCpuLocal(0));
  core->Join(cpu1_peer);
  core->Join(from_high_cpu);

  EXPECT_TRUE(ctx.steal_from_high_cpu_result);
  ASSERT_THAT(static_cast<int>(ctx.migrate_trace.size()), t::Eq(8));
  EXPECT_THAT(ctx.migrate_trace[0], t::Eq(10));
  EXPECT_THAT(ctx.migrate_trace[1], t::Eq(20));
  EXPECT_THAT(MaxConsecutiveRun(ctx.migrate_trace, 10), t::Le(2));
  EXPECT_THAT(MaxConsecutiveRun(ctx.migrate_trace, 20), t::Le(2));

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// ============================================================================
// 5. `StealTask(dst_cpu, src_cpu)` in Isolation
// ============================================================================

struct StealOrderCtx {
  TaskScheduler* sched = nullptr;
  std::vector<int> execution_order;
};

static void RecordIdOneWorker(void* const raw_arg) {
  StealOrderCtx* const ctx = static_cast<StealOrderCtx*>(raw_arg);
  ctx->execution_order.push_back(1);
}

static void RecordIdTwoWorker(void* const raw_arg) {
  StealOrderCtx* const ctx = static_cast<StealOrderCtx*>(raw_arg);
  ctx->execution_order.push_back(2);
}

TEST(CoreRunqueueSchedulerTest,
     StealTaskMovesLeftmostTaskAndRespectsDestinationAndSourceState) {
  SetupEnv(4);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  // 1. Edge cases: same CPU (`dst_cpu == src_cpu`) or empty `src_cpu` returns
  //    false.
  EXPECT_FALSE(TaskSchedulerInternals::StealTask(*core, 0, 0));
  EXPECT_FALSE(TaskSchedulerInternals::StealTask(*core, 0, 1));
  EXPECT_FALSE(TaskSchedulerInternals::StealTask(*core, 2, 1));

  StealOrderCtx ctx;
  ctx.sched = core.get();

  // 2. Enqueue two tasks on CPU 2 (`t1` first, `t2` second).
  Task* const t1 =
      core->CreateTaskOnCpu(RecordIdOneWorker, &ctx, kDefaultTaskWeight, 2);
  Task* const t2 =
      core->CreateTaskOnCpu(RecordIdTwoWorker, &ctx, kDefaultTaskWeight, 2);
  ASSERT_THAT(t1, t::NotNull());
  ASSERT_THAT(t2, t::NotNull());
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 2), t::Eq(2));

  // 3. Test `dst_cpu > src_cpu`: steal from CPU 2 to CPU 3 (`3 > 2`).
  //    Must steal the lowest-key task (`t1`, earlier `enqueue_seq`).
  EXPECT_TRUE(TaskSchedulerInternals::StealTask(*core, 3, 2));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 2), t::Eq(1));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 3), t::Eq(1));

  // 4. Calling `StealTask(3, 2)` again while CPU 3 is already non-empty
  //    (`RunqueueLoad(3) == 1`) must return `true` immediately WITHOUT stealing
  //    `t2` from CPU 2.
  EXPECT_TRUE(TaskSchedulerInternals::StealTask(*core, 3, 2));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 2), t::Eq(1));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 3), t::Eq(1));

  // 5. Offline CPU check: if either `dst_cpu` or `src_cpu` is offline,
  //    `StealTask` returns `false` without stealing.
  g_env.cpu_locals[1].online = false;
  EXPECT_FALSE(TaskSchedulerInternals::StealTask(*core, 1, 2));
  EXPECT_FALSE(TaskSchedulerInternals::StealTask(*core, 0, 1));
  g_env.cpu_locals[1].online = true;

  // 6. Test `dst_cpu < src_cpu`: steal `t1` from CPU 3 to CPU 0 (`0 < 3`) and
  //    run it, then steal `t2` from CPU 2 to CPU 0 (`0 < 2`) and run it.
  EXPECT_TRUE(TaskSchedulerInternals::StealTask(*core, 0, 3));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 3), t::Eq(0));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(1));
  core->Join(t1);
  EXPECT_THAT(ctx.execution_order, t::ElementsAre(1));

  EXPECT_TRUE(TaskSchedulerInternals::StealTask(*core, 0, 2));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 2), t::Eq(0));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(1));
  core->Join(t2);
  EXPECT_THAT(ctx.execution_order, t::ElementsAre(1, 2));

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

struct StealProbeResult {
  int src_load_before = -1;
  bool stolen = false;
  int src_load_after = -1;
  int dst_load_after = -1;
};

struct SkipBootstrapCtx {
  TaskScheduler* sched = nullptr;
  StealProbeResult probe;
  int helper_first_cpu = -1;
  int helper_resumed_cpu = -1;
};

// Runs on CPU 0 while the bootstrap task waits in Join(), that is, while the
// bootstrap task is queued in CPU 0's runqueue, and steals from CPU 0 for
// CPU 2.
static void StealFromCpu0ProbeWorker(void* const raw_arg) {
  SkipBootstrapCtx* const ctx = static_cast<SkipBootstrapCtx*>(raw_arg);
  ctx->probe.src_load_before =
      TaskSchedulerInternals::RunqueueLoad(*ctx->sched, 0);
  ctx->probe.stolen = TaskSchedulerInternals::StealTask(*ctx->sched, 2, 0);
  ctx->probe.src_load_after =
      TaskSchedulerInternals::RunqueueLoad(*ctx->sched, 0);
  ctx->probe.dst_load_after =
      TaskSchedulerInternals::RunqueueLoad(*ctx->sched, 2);
}

// Created with weight 1, so the Yield() in its first slice charges it about
// 1M of vruntime and leaves it queued behind the bootstrap task.
static void SkipBootstrapHelperWorker(void* const raw_arg) {
  SkipBootstrapCtx* const ctx = static_cast<SkipBootstrapCtx*>(raw_arg);
  ctx->helper_first_cpu = CurrentCpuId();
  ctx->sched->Yield();
  ctx->helper_resumed_cpu = CurrentCpuId();
}

TEST(CoreRunqueueSchedulerTest,
     StealTaskSkipsBootstrapTaskAtHeadOfSourceRunqueue) {
  SetupEnv(4);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);
  Task* const bootstrap = core->CurrentTask();

  SkipBootstrapCtx ctx;
  ctx.sched = core.get();

  // 1. With only the bootstrap task queued on CPU 0 there is nothing to steal:
  //    RunqueueLoad(0) excludes it and StealTask(2, 0) returns false.
  Task* const first_probe = core->CreateTaskOnCpu(StealFromCpu0ProbeWorker,  //
                                                  &ctx,                      //
                                                  kDefaultTaskWeight,        //
                                                  0);
  ASSERT_THAT(first_probe, t::NotNull());
  core->Join(first_probe);
  EXPECT_THAT(ctx.probe.src_load_before, t::Eq(0));
  EXPECT_FALSE(ctx.probe.stolen);
  EXPECT_THAT(ctx.probe.dst_load_after, t::Eq(0));

  // 2. Run the helper's first slice; its Yield() switches back to the
  //    bootstrap task and leaves the helper queued on CPU 0.
  Task* const helper = core->CreateTaskOnCpu(SkipBootstrapHelperWorker,  //
                                             &ctx,                       //
                                             kMinTaskWeight,             //
                                             0);
  ASSERT_THAT(helper, t::NotNull());
  core->Yield();
  ASSERT_THAT(ctx.helper_first_cpu, t::Eq(0));
  ASSERT_THAT(ctx.helper_resumed_cpu, t::Eq(-1));

  // 3. While the second probe runs, the bootstrap task is the leftmost task in
  //    CPU 0's runqueue and the helper is behind it. StealTask(2, 0) must skip
  //    the bootstrap task and move the helper.
  ctx.probe = StealProbeResult{};
  Task* const second_probe = core->CreateTaskOnCpu(StealFromCpu0ProbeWorker,  //
                                                   &ctx,                      //
                                                   kDefaultTaskWeight,        //
                                                   0);
  ASSERT_THAT(second_probe, t::NotNull());
  core->Join(second_probe);
  EXPECT_THAT(ctx.probe.src_load_before, t::Eq(1));
  EXPECT_TRUE(ctx.probe.stolen);
  EXPECT_THAT(ctx.probe.src_load_after, t::Eq(0));
  EXPECT_THAT(ctx.probe.dst_load_after, t::Eq(1));
  EXPECT_THAT(core->CurrentTask(), t::Eq(bootstrap));

  // 4. Standing in for CPU 2's idle loop resumes the helper on CPU 2.
  BindCpuLocal(SmpGetCpuLocal(2));
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*core, 2));
  BindCpuLocal(SmpGetCpuLocal(0));
  EXPECT_THAT(ctx.helper_resumed_cpu, t::Eq(2));
  core->Join(helper);

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// ============================================================================
// 6. `OnTimerInterrupt` and `OnWakeupIpi` Preemption & Weighted Tick Charging
// ============================================================================

struct TimerPreemptCtx {
  TaskScheduler* sched = nullptr;
  int task_b_slices = 0;
  int task_b_slices_after_wakeup_ipi = -1;
  int task_b_slices_after_tick_1 = -1;
  int task_b_slices_after_tick_2 = -1;
};

static void TimerPreemptWorkerB(void* const raw_arg) {
  TimerPreemptCtx* const ctx = static_cast<TimerPreemptCtx*>(raw_arg);
  // Slice 1: advance `task_b->vruntime` from 0 to +512 (weight = 2048) and
  // yield to `task_a` (which is still at `vruntime == 0`).
  ++ctx->task_b_slices;
  ctx->sched->Yield();

  // Slice 2: reached when `task_a` is preempted by `OnTimerInterrupt` once
  // `task_a->vruntime` (1024) exceeds `task_b->vruntime` (512).
  ++ctx->task_b_slices;
}

static void TimerPreemptWorkerA(void* const raw_arg) {
  TimerPreemptCtx* const ctx = static_cast<TimerPreemptCtx*>(raw_arg);
  InterruptFrame frame = {};
  frame.vector = kVectorApicTimer;
  frame.rflags = kRflagsInterruptEnableBit;

  // At entry, `task_a->vruntime == 0` and `task_b` is in CPU 0's runqueue at
  // `vruntime == 512`.
  // 1. `OnWakeupIpi` does not charge vruntime and sees `first->vruntime` (512)
  //    >= `curr->vruntime` (0), so it MUST NOT preempt.
  TaskSchedulerInternals::OnWakeupIpi(*ctx->sched, &frame);
  ctx->task_b_slices_after_wakeup_ipi = ctx->task_b_slices;

  // 2. First `OnTimerInterrupt` charges +512 (`weight = 2048`) to `task_a`,
  //    bringing `task_a->vruntime` to 512. Because `first->vruntime <
  //    curr->vruntime` (`512 < 512`) is false, it MUST NOT preempt yet.
  TaskSchedulerInternals::OnTimerInterrupt(*ctx->sched, &frame);
  ctx->task_b_slices_after_tick_1 = ctx->task_b_slices;

  // 3. Second `OnTimerInterrupt` charges another +512 to `task_a`, bringing
  //    `task_a->vruntime` to 1024. Now `first->vruntime < curr->vruntime`
  //    (`512 < 1024`) is true, so `task_a` IS preempted and `task_b` runs its
  //    second slice!
  TaskSchedulerInternals::OnTimerInterrupt(*ctx->sched, &frame);
  ctx->task_b_slices_after_tick_2 = ctx->task_b_slices;
}

static void ApSimpleRunWorker(void* const raw_arg) {
  int* const counter = static_cast<int*>(raw_arg);
  ++(*counter);
}

TEST(CoreRunqueueSchedulerTest,
     InterruptsChargeWeightedVruntimeAndPreemptLaggingTasksButNotIdleTasks) {
  SetupEnv(2);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();

  InterruptFrame frame = {};
  frame.vector = kVectorApicTimer;
  frame.rflags = kRflagsInterruptEnableBit;

  // Calling OnTimerInterrupt / OnWakeupIpi before Init() is a safe no-op.
  TaskSchedulerInternals::OnTimerInterrupt(*core, &frame);
  TaskSchedulerInternals::OnWakeupIpi(*core, &frame);

  TaskSchedulerInternals::Init(*core);

  // Calling OnTimerInterrupt / OnWakeupIpi with unbound CpuLocal or offline CPU
  // is a safe no-op.
  ResetCpuLocalForTest();
  TaskSchedulerInternals::OnTimerInterrupt(*core, &frame);
  TaskSchedulerInternals::OnWakeupIpi(*core, &frame);
  BindCpuLocal(SmpGetCpuLocal(0));
  g_env.cpu_locals[0].online = false;
  TaskSchedulerInternals::OnTimerInterrupt(*core, &frame);
  TaskSchedulerInternals::OnWakeupIpi(*core, &frame);
  g_env.cpu_locals[0].online = true;

  // Part A: Verify weighted vruntime charging and strict `<` preemption
  // threshold on CPU 0.
  TimerPreemptCtx ctx;
  ctx.sched = core.get();
  Task* const task_b = core->CreateTaskOnCpu(TimerPreemptWorkerB,     //
                                             &ctx,                    //
                                             kDefaultTaskWeight * 2,  //
                                             0);
  Task* const task_a = core->CreateTaskOnCpu(TimerPreemptWorkerA,     //
                                             &ctx,                    //
                                             kDefaultTaskWeight * 2,  //
                                             0);
  core->Join(task_b);
  core->Join(task_a);

  EXPECT_THAT(ctx.task_b_slices_after_wakeup_ipi, t::Eq(1));
  EXPECT_THAT(ctx.task_b_slices_after_tick_1, t::Eq(1));
  EXPECT_THAT(ctx.task_b_slices_after_tick_2, t::Eq(2));

  // Part B: Interrupts never preempt an AP's idle task, even with work queued
  // on that AP. The idle loop runs queued work itself by calling PollIdleCpu()
  // after every wakeup, so it is never suspended in the middle of its own work.
  int ap_runs = 0;
  Task* const ap_task =
      core->CreateTaskOnCpu(ApSimpleRunWorker, &ap_runs, kDefaultTaskWeight, 1);
  ASSERT_THAT(ap_task, t::NotNull());
  BindCpuLocal(SmpGetCpuLocal(1));
  InterruptFrame ipi_frame = {};
  ipi_frame.vector = kVectorWakeupIpi;
  ipi_frame.rflags = kRflagsInterruptEnableBit;
  TaskSchedulerInternals::OnWakeupIpi(*core, &ipi_frame);
  TaskSchedulerInternals::OnTimerInterrupt(*core, &frame);
  EXPECT_THAT(ap_runs, t::Eq(0));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 1), t::Eq(1));

  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*core, 1));
  EXPECT_THAT(ap_runs, t::Eq(1));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 1), t::Eq(0));
  BindCpuLocal(SmpGetCpuLocal(0));
  core->Join(ap_task);

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// ============================================================================
// 7. Two-Stage Zombie Stack/TCB Reclamation (Stack on Exit, TCB on `Join`)
// ============================================================================

struct ZombieStageCtx {
  TaskScheduler* sched = nullptr;
  bool zombie_ran = false;
};

static void ReturnImmediatelyZombieWorker(void* const raw_arg) {
  ZombieStageCtx* const ctx = static_cast<ZombieStageCtx*>(raw_arg);
  ctx->zombie_ran = true;
}

TEST(CoreRunqueueSchedulerTest,
     TwoStageZombieReclamationFreesStackOnContextSwitchAndTcbOnJoin) {
  SetupEnv(1);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  ZombieStageCtx ctx;
  ctx.sched = core.get();
  Task* const zombie =
      core->CreateTask(ReturnImmediatelyZombieWorker, &ctx, kDefaultTaskWeight);
  ASSERT_THAT(zombie, t::NotNull());

  // Immediately after creation, both the 4-frame kernel stack and TCB are
  // allocated.
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before - kTaskStackFrames));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Lt(heap_before));

  while (!ctx.zombie_ran) {
    core->Yield();
  }

  // Stage 1: As soon as `zombie` exited and switched back to the bootstrap
  // task, `FinishContextSwitch` on the bootstrap stack MUST have freed
  // `zombie`'s 16 KiB stack back to the PMM, while `zombie`'s TCB remains
  // allocated on the heap until `Join(zombie)`.
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Lt(heap_before));

  // Enqueue a runnable probe task on CPU 0; calling `Join(zombie)` when
  // `zombie` has already exited must reap its TCB immediately WITHOUT yielding
  // to `probe_task`.
  int probe_runs = 0;
  Task* const probe_task =
      core->CreateTask(ApSimpleRunWorker, &probe_runs, kDefaultTaskWeight);
  ASSERT_THAT(probe_task, t::NotNull());

  core->Join(zombie);
  EXPECT_THAT(probe_runs, t::Eq(0));

  core->Join(probe_task);
  EXPECT_THAT(probe_runs, t::Eq(1));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// ============================================================================
// 8. Multi-Threaded SMP Concurrency Test (`Yield`, `StealTask`,
// `OnTimerInterrupt`)
// ============================================================================

struct MtCoreSharedCtx {
  TaskScheduler* sched = nullptr;
  std::atomic<uint32_t> cpu_mask{0};
  std::atomic<int> completed_tasks{0};
  std::atomic<int64_t> total_iterations{0};
  std::atomic<int64_t> remote_executions{0};
};

struct MtCoreTaskArg {
  MtCoreSharedCtx* shared = nullptr;
  int task_index = 0;
  std::atomic<int> iterations{0};
};

static void MtCoreConcurrencyWorker(void* const raw_arg) {
  MtCoreTaskArg* const arg = static_cast<MtCoreTaskArg*>(raw_arg);
  MtCoreSharedCtx* const shared = arg->shared;
  TaskScheduler* const sched = shared->sched;

  InterruptFrame timer_frame = {};
  timer_frame.vector = kVectorApicTimer;
  timer_frame.rflags = kRflagsInterruptEnableBit;

  for (int round = 0; round < 6; ++round) {
    const int cpu = CurrentCpuId();
    shared->cpu_mask.fetch_or(1u << static_cast<uint32_t>(cpu),
                              std::memory_order_acq_rel);
    if (cpu != 0) {
      shared->remote_executions.fetch_add(1, std::memory_order_acq_rel);
    }
    arg->iterations.fetch_add(1, std::memory_order_acq_rel);
    shared->total_iterations.fetch_add(1, std::memory_order_acq_rel);

    if ((round & 1) == 0) {
      sched->Yield();
    } else {
      TaskSchedulerInternals::OnTimerInterrupt(*sched, &timer_frame);
      sched->Yield();
    }
  }

  const int cpu = CurrentCpuId();
  shared->cpu_mask.fetch_or(1u << static_cast<uint32_t>(cpu),
                            std::memory_order_acq_rel);
  if (cpu != 0) {
    shared->remote_executions.fetch_add(1, std::memory_order_acq_rel);
  }
  shared->completed_tasks.fetch_add(1, std::memory_order_acq_rel);
}

TEST(CoreRunqueueSchedulerTest,
     ConcurrentMultiThreadedYieldStealTaskAndTimerPreemption) {
  constexpr int kNumCpus = 4;
  constexpr int kNumTasks = 24;
  SetupEnv(kNumCpus);

  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);
  Task* const bootstrap = core->CurrentTask();

  MtCoreSharedCtx shared;
  shared.sched = core.get();
  std::unique_ptr<MtCoreTaskArg[]> args(new MtCoreTaskArg[kNumTasks]);
  std::vector<Task*> tasks;
  tasks.reserve(kNumTasks);

  // Enqueue tasks across CPU 0 (majority) and CPUs 1..3 so StealTask exercises
  // both `dst_cpu < src_cpu` and `dst_cpu > src_cpu` lock acquisition orders.
  for (int i = 0; i < kNumTasks; ++i) {
    args[i].shared = &shared;
    args[i].task_index = i;
    const int initial_cpu = (i < 18) ? 0 : (i % kNumCpus);
    Task* const task = core->CreateTaskOnCpu(MtCoreConcurrencyWorker,     //
                                             &args[i],                    //
                                             kDefaultTaskWeight + i * 8,  //
                                             initial_cpu);
    ASSERT_THAT(task, t::NotNull());
    tasks.push_back(task);
  }

  std::atomic<int> ready_workers{0};
  std::atomic<bool> stop_workers{false};
  std::vector<std::thread> ap_threads;
  ap_threads.reserve(kNumCpus - 1);

  for (int cpu = 1; cpu < kNumCpus; ++cpu) {
    ap_threads.emplace_back([cpu, &core, &ready_workers, &stop_workers]() {
      BindCpuLocal(SmpGetCpuLocal(cpu));
      SetInterruptsEnabledForTest(true);
      ready_workers.fetch_add(1, std::memory_order_acq_rel);

      int donor_cursor = 0;
      while (!stop_workers.load(std::memory_order_acquire)) {
        if (TaskSchedulerInternals::RunqueueLoad(*core, cpu) == 0) {
          for (int step = 0; step < kNumCpus; ++step) {
            const int donor = (donor_cursor + step) % kNumCpus;
            if (donor != cpu &&
                TaskSchedulerInternals::RunqueueLoad(*core, donor) > 0) {
              if (TaskSchedulerInternals::StealTask(*core, cpu, donor)) {
                donor_cursor = (donor + 1) % kNumCpus;
                break;
              }
            }
          }
        }
        if (!TaskSchedulerInternals::PollIdleCpu(*core, cpu)) {
          std::this_thread::yield();
        }
      }
      ResetCpuLocalForTest();
    });
  }

  while (ready_workers.load(std::memory_order_acquire) < (kNumCpus - 1)) {
    std::this_thread::yield();
  }

  // While waiting for all tasks to finish on CPU 0, also allow CPU 0 to steal
  // back remaining runnable tasks if CPU 0 drains its local queue first.
  while (shared.completed_tasks.load(std::memory_order_acquire) < kNumTasks) {
    if (TaskSchedulerInternals::RunqueueLoad(*core, 0) == 0) {
      for (int src = 1; src < kNumCpus; ++src) {
        if (TaskSchedulerInternals::RunqueueLoad(*core, src) > 0 &&
            TaskSchedulerInternals::StealTask(*core, 0, src)) {
          break;
        }
      }
    }
    core->Yield();
    std::this_thread::yield();
  }

  for (Task* const task : tasks) {
    core->Join(task);
  }

  stop_workers.store(true, std::memory_order_release);
  for (std::thread& th : ap_threads) {
    th.join();
  }

  EXPECT_THAT(CurrentCpuId(), t::Eq(0));
  EXPECT_THAT(core->CurrentTask(), t::Eq(bootstrap));
  EXPECT_THAT(shared.completed_tasks.load(), t::Eq(kNumTasks));
  EXPECT_THAT(shared.total_iterations.load(), t::Eq(kNumTasks * 6));
  for (int i = 0; i < kNumTasks; ++i) {
    EXPECT_THAT(args[i].iterations.load(), t::Eq(6));
  }

  EXPECT_THAT(shared.remote_executions.load(), t::Gt(0));
  EXPECT_THAT(shared.cpu_mask.load() & ~1u, t::Ne(0u));

  for (int c = 0; c < kNumCpus; ++c) {
    EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, c), t::Eq(0));
  }
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// ============================================================================
// 9. `DCHECK` Precondition and Invariant Death Tests
// ============================================================================

struct DeathLockWorkerCtx {
  TaskScheduler* sched = nullptr;
  Task* bootstrap = nullptr;
  IrqSpinLock leaked_lock;
};

static void ExitWhileHoldingSpinLockWorker(void* const raw_arg) {
  DeathLockWorkerCtx* const ctx = static_cast<DeathLockWorkerCtx*>(raw_arg);
  ctx->leaked_lock.Lock();
}

static void JoinBootstrapWorker(void* const raw_arg) {
  DeathLockWorkerCtx* const ctx = static_cast<DeathLockWorkerCtx*>(raw_arg);
  ctx->sched->Join(ctx->bootstrap);
}

TEST(CoreRunqueueSchedulerDeathTest,
     PreconditionAndInvariantViolationsTriggerDcheck) {
  SetupEnv(2);

  // 1. Calling methods before `Init()` must trigger `DCHECK`.
  const std::shared_ptr<TaskScheduler> uninit = CreateCoreRunqueueScheduler();
  constexpr const char* kNotInitialized =
      "Check failed: initialized_\\.load\\(std::memory_order_acquire\\)";
  EXPECT_DEATH(uninit->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
               kNotInitialized);
  EXPECT_DEATH(
      uninit->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 0),
      kNotInitialized);
  EXPECT_DEATH(uninit->Yield(), kNotInitialized);
  EXPECT_DEATH(uninit->Join(nullptr), kNotInitialized);
  EXPECT_DEATH(TaskSchedulerInternals::StealTask(*uninit, 0, 1),
               kNotInitialized);
  EXPECT_DEATH(uninit->CurrentTask(), kNotInitialized);

  // `Init()` must run after `IdtInit()`, exactly once, on the BSP (CPU 0) with
  // its `CpuLocal` bound; it no longer binds one itself.
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  g_env.idt_initialized = false;
  EXPECT_DEATH(TaskSchedulerInternals::Init(*core),
               "Check failed: IdtIsInitialized\\(\\)");
  g_env.idt_initialized = true;
  ResetCpuLocalForTest();
  EXPECT_DEATH(TaskSchedulerInternals::Init(*core),
               "Check failed: CurrentCpuOrNullIrqsOff\\(\\) != nullptr");
  BindCpuLocal(SmpGetCpuLocal(1));
  EXPECT_DEATH(TaskSchedulerInternals::Init(*core),
               "Check failed: CurrentCpuIdIrqsOff\\(\\) == 0");
  BindCpuLocal(SmpGetCpuLocal(0));
  TaskSchedulerInternals::Init(*core);
  EXPECT_DEATH(
      TaskSchedulerInternals::Init(*core),
      "Check failed: !initialized_\\.load\\(std::memory_order_acquire\\)");

  // 2. Invalid `CreateTask` / `CreateTaskOnCpu` arguments (`entry == nullptr`,
  //    invalid `weight`, out-of-range or offline `target_cpu`).
  constexpr const char* kBadWeight =
      "Check failed: weight >= kMinTaskWeight && weight <= kMaxTaskWeight";
  constexpr const char* kBadTargetCpu =
      "Check failed: target_cpu >= 0 && target_cpu < cpu_count_";
  EXPECT_DEATH(core->CreateTask(nullptr, nullptr, kDefaultTaskWeight),
               "Check failed: entry != nullptr");
  EXPECT_DEATH(core->CreateTask(NoopTask, nullptr, 0), kBadWeight);
  EXPECT_DEATH(core->CreateTask(NoopTask, nullptr, -5), kBadWeight);
  EXPECT_DEATH(core->CreateTask(NoopTask, nullptr, kMaxTaskWeight + 1),
               kBadWeight);
  EXPECT_DEATH(core->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, -1),
               kBadTargetCpu);
  EXPECT_DEATH(core->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 2),
               kBadTargetCpu);

  g_env.cpu_locals[1].online = false;
  EXPECT_DEATH(core->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 1),
               "Check failed: IsCpuOnline\\(target_cpu, cpu_count_\\)");
  g_env.cpu_locals[1].online = true;

  // 3. Out-of-range CPU indices on `RunqueueLoad` and `StealTask`.
  constexpr const char* kBadCpuIndex =
      "Check failed: cpu_index >= 0 && cpu_index < cpu_count_";
  constexpr const char* kBadDstCpu =
      "Check failed: dst_cpu >= 0 && dst_cpu < cpu_count_";
  constexpr const char* kBadSrcCpu =
      "Check failed: src_cpu >= 0 && src_cpu < cpu_count_";
  EXPECT_DEATH(TaskSchedulerInternals::RunqueueLoad(*core, -1), kBadCpuIndex);
  EXPECT_DEATH(TaskSchedulerInternals::RunqueueLoad(*core, 2), kBadCpuIndex);
  EXPECT_DEATH(TaskSchedulerInternals::StealTask(*core, -1, 0), kBadDstCpu);
  EXPECT_DEATH(TaskSchedulerInternals::StealTask(*core, 2, 0), kBadDstCpu);
  EXPECT_DEATH(TaskSchedulerInternals::StealTask(*core, 0, -1), kBadSrcCpu);
  EXPECT_DEATH(TaskSchedulerInternals::StealTask(*core, 0, 2), kBadSrcCpu);

  // 4. Invalid `Join` calls: a null task, a self-join (here by the bootstrap
  //    task), joining an idle task, and a task joining the bootstrap task.
  EXPECT_DEATH(core->Join(nullptr), "Check failed: task != nullptr");
  EXPECT_DEATH(core->Join(core->CurrentTask()), "Check failed: task != self");

  BindCpuLocal(SmpGetCpuLocal(1));
  Task* const ap1_idle = core->CurrentTask();
  BindCpuLocal(SmpGetCpuLocal(0));
  EXPECT_DEATH(core->Join(ap1_idle), "Check failed: !task->is_idle");

  DeathLockWorkerCtx worker_ctx;
  worker_ctx.sched = core.get();
  worker_ctx.bootstrap = core->CurrentTask();

  EXPECT_DEATH(
      {
        Task* const t = core->CreateTask(JoinBootstrapWorker,  //
                                         &worker_ctx,          //
                                         kDefaultTaskWeight);
        core->Join(t);
      },
      "Check failed: !task->is_bootstrap");

  // 5. Yielding, Joining, or returning from a task while holding an
  //    `IrqSpinLock` must trigger `DCHECK`.
  constexpr const char* kLockHeld = "Check failed: !HoldsAnyIrqSpinLock\\(\\)";
  EXPECT_DEATH(
      {
        IrqSpinLock lock;
        const IrqSpinLockGuard guard(lock);
        core->Yield();
      },
      kLockHeld);

  EXPECT_DEATH(
      {
        Task* const dummy =
            core->CreateTask(NoopTask, nullptr, kDefaultTaskWeight);
        IrqSpinLock lock;
        const IrqSpinLockGuard guard(lock);
        core->Join(dummy);
      },
      kLockHeld);

  EXPECT_DEATH(
      {
        Task* const t = core->CreateTask(ExitWhileHoldingSpinLockWorker,  //
                                         &worker_ctx,                     //
                                         kDefaultTaskWeight);
        core->Join(t);
      },
      kLockHeld);
}

// ============================================================================
// 10. Task Stack Overflow Canary
// ============================================================================

struct StackUseCtx {
  TaskScheduler* sched = nullptr;
  uintptr_t stack_base = 0;
  uintptr_t frame_address = 0;
  int64_t checksum = 0;
};

// Fills 8 KiB of locals (half the task stack), yields while they are live,
// and checks them afterwards.
static void DeepStackUseWorker(void* const raw_arg) {
  StackUseCtx* const ctx = static_cast<StackUseCtx*>(raw_arg);
  ctx->stack_base = CoreTaskStackBaseForTest(ctx->sched->CurrentTask());
  ctx->frame_address = reinterpret_cast<uintptr_t>(__builtin_frame_address(0));
  volatile uint8_t buffer[8192];
  for (int i = 0; i < 8192; ++i) {
    buffer[i] = static_cast<uint8_t>(i);
  }
  ctx->sched->Yield();
  int64_t sum = 0;
  for (int i = 0; i < 8192; ++i) {
    sum += buffer[i];
  }
  ctx->checksum = sum;
}

TEST(CoreRunqueueSchedulerTest, StackCanaryToleratesDeepButInBoundsStackUse) {
  SetupEnv(1);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  StackUseCtx ctx;
  ctx.sched = core.get();
  // Half the default weight: the worker's Yield() charges it more than the
  // bootstrap task, so it is switched out (and its canary checked) mid-run.
  Task* const task = core->CreateTask(DeepStackUseWorker,  //
                                      &ctx,                //
                                      kDefaultTaskWeight / 2);
  ASSERT_THAT(task, t::NotNull());
  core->Join(task);

  // The canary lives at the lowest address of the worker's own stack.
  EXPECT_THAT(ctx.stack_base, t::Ne(0u));
  EXPECT_THAT(ctx.frame_address, t::Gt(ctx.stack_base));
  EXPECT_THAT(ctx.frame_address,
              t::Lt(ctx.stack_base + static_cast<uintptr_t>(kTaskStackSize)));
  EXPECT_THAT(ctx.checksum, t::Eq(int64_t{32} * (255 * 256 / 2)));

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// Simulates a stack overflow that reached the bottom of the calling task's
// stack, then exits.
static void SmashStackCanaryWorker(void* const raw_arg) {
  TaskScheduler* const sched = static_cast<TaskScheduler*>(raw_arg);
  *reinterpret_cast<volatile uint64_t*>(
      CoreTaskStackBaseForTest(sched->CurrentTask())) = 0;
}

// Smashes the canary only while switched out: it restores the original value
// after its Yield() returns, so only the switch-out check can catch it.
static void SmashStackCanaryAcrossYieldWorker(void* const raw_arg) {
  TaskScheduler* const sched = static_cast<TaskScheduler*>(raw_arg);
  volatile uint64_t* const canary = reinterpret_cast<volatile uint64_t*>(
      CoreTaskStackBaseForTest(sched->CurrentTask()));
  const uint64_t saved = *canary;
  *canary = ~saved;
  sched->Yield();
  *canary = saved;
}

TEST(CoreRunqueueSchedulerDeathTest,
     StackCanaryIsCheckedWhenTaskIsSwitchedOutAndWhenItExits) {
  SetupEnv(1);
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  constexpr const char* kStackOverflow =
      "Check failed: task_stack_canary_intact";
  // Weight 1: the worker's Yield() charges it about 1M of vruntime, so it is
  // switched out to the bootstrap task.
  EXPECT_DEATH(
      {
        Task* const t = core->CreateTask(SmashStackCanaryAcrossYieldWorker,  //
                                         core.get(),                         //
                                         kMinTaskWeight);
        core->Join(t);
      },
      kStackOverflow);
  EXPECT_DEATH(
      {
        Task* const t = core->CreateTask(SmashStackCanaryWorker,  //
                                         core.get(),              //
                                         kDefaultTaskWeight);
        core->Join(t);
      },
      kStackOverflow);
}

// The core keeps no join records, so the task lifecycle methods that the join
// lifecycle layer implements crash with a message that names that layer.
TEST(CoreRunqueueSchedulerDeathTest, DetachAndReapZombiesPanic) {
  SetupEnv(1);
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);
  Task* const task = core->CreateTask(NoopTask, nullptr, kDefaultTaskWeight);
  ASSERT_THAT(task, t::NotNull());

  constexpr const char* kDetachMessage =
      "^\\[FATAL\\] CoreRunqueueTaskScheduler::Detach\\(\\) is not supported: "
      "detaching is handled by the join lifecycle layer at "
      ".*core_runqueue_scheduler\\.cpp:[0-9]+\n$";
  constexpr const char* kReapZombiesMessage =
      "^\\[FATAL\\] CoreRunqueueTaskScheduler::ReapZombies\\(\\) is not "
      "supported: reaping zombies is handled by the join lifecycle layer at "
      ".*core_runqueue_scheduler\\.cpp:[0-9]+\n$";
  EXPECT_EXIT(core->Detach(task), t::ExitedWithCode(134), kDetachMessage);
  EXPECT_EXIT(core->ReapZombies(), t::ExitedWithCode(134), kReapZombiesMessage);

  // The death tests ran in child processes; the task is still queued here.
  core->Join(task);
}

}  // namespace
}  // namespace protos
