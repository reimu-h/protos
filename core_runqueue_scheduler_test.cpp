#include "core_runqueue_scheduler.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <vector>

#include "cpu_clock.h"
#include "scheduler_test_support.h"
#include "smp.h"
#include "task.h"
#include "task_scheduler_internals.h"
#include "user_access.h"

namespace protos {
namespace {

namespace t = ::testing;

// The comments give vruntime values relative to the `min_vruntime` that every
// CPU starts with, which the core puts just below the 2^64 wrap of vruntime
// (see VruntimeOrderHoldsAcrossTheWrapAround).

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

// Returns a distinct fake address space for each `index`. The core only stores,
// compares and passes on address space pointers, so they need not point to
// real address spaces.
static AddressSpace* FakeAddressSpace(const int index) {
  return reinterpret_cast<AddressSpace*>(0x10000 + index * 0x1000);
}

// Returns the address space that the fake ActivateAddressSpace() last switched
// CPU `cpu` to (nullptr: the kernel's page tables).
static const AddressSpace* ActiveAddressSpace(const int cpu) {
  return g_env.active_address_space[cpu].load(std::memory_order_acquire);
}

// Returns a distinct fake FPU state for each `index`. The core only stores and
// passes on FpuState pointers, which the fake FpuSave() and FpuRestore() only
// record, so they need not point to real states.
static FpuState* FakeFpuState(const int index) {
  return reinterpret_cast<FpuState*>(0x100000 + index * 0x1000);
}

// Returns the FPU state whose values the registers of CPU `cpu` hold: the one
// that the fake FpuRestore() last loaded there.
static const FpuState* LoadedFpuState(const int cpu) {
  return g_env.loaded_fpu_state[cpu].load(std::memory_order_acquire);
}

// Returns the calls of the fake FpuSave() and FpuRestore() so far, in order.
static std::vector<FakeFpuCall> FpuCalls() {
  const std::lock_guard<std::mutex> lock(g_env.log_mutex);
  return g_env.fpu_calls;
}

static void ClearFpuCalls() {
  const std::lock_guard<std::mutex> lock(g_env.log_mutex);
  g_env.fpu_calls.clear();
}

// Returns the number of calls in `calls` that did `op` with `state`.
static int CountFpuCalls(const std::vector<FakeFpuCall>& calls,  //
                         const FakeFpuOp op,                     //
                         const FpuState* const state) {
  int count = 0;
  for (const FakeFpuCall& call : calls) {
    count += (call.op == op && call.state == state) ? 1 : 0;
  }
  return count;
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
    // Before Init(), PollIdleCpu must safely return false, RunqueueLoad must
    // report an empty runqueue and IsCpuBusy an idle CPU.
    EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*core, 0));
    EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(0));
    EXPECT_FALSE(TaskSchedulerInternals::IsCpuBusy(*core, 0));

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

    // Nothing is queued anywhere. CPU 0 is busy running the bootstrap task;
    // every AP runs its idle task.
    for (int c = 0; c < num_cpus; ++c) {
      EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, c), t::Eq(0));
      EXPECT_THAT(TaskSchedulerInternals::IsCpuBusy(*core, c), t::Eq(c == 0));
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
  // when either Kmalloc(sizeof(Task)) or TaskStackAlloc() (which the fake
  // serves from the PMM) fails due to OOM.
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

TEST(CoreRunqueueSchedulerTest, CountsEveryContextSwitchOnTheCpuThatMakesIt) {
  SetupEnv(2);
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);
  const CpuLocal& cpu0 = *SmpGetCpuLocal(0);
  const CpuLocal& cpu1 = *SmpGetCpuLocal(1);

  // With nothing else to run, `Yield()` keeps running the caller.
  core->Yield();
  EXPECT_THAT(cpu0.context_switches.load(), t::Eq(0));

  // To the task, and back once it has returned.
  Task* const local = core->CreateTask(NoopTask, nullptr, kDefaultTaskWeight);
  ASSERT_THAT(local, t::NotNull());
  core->Join(local);
  EXPECT_THAT(cpu0.context_switches.load(), t::Eq(2));

  // CPU 1 counts its own switches: from its idle loop to the task and back.
  Task* const remote =
      core->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 1);
  ASSERT_THAT(remote, t::NotNull());
  BindCpuLocal(SmpGetCpuLocal(1));
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*core, 1));
  BindCpuLocal(SmpGetCpuLocal(0));
  core->Join(remote);
  EXPECT_THAT(cpu1.context_switches.load(), t::Eq(2));
  EXPECT_THAT(cpu0.context_switches.load(), t::Eq(2));
}

struct RunningCpuCtx {
  TaskScheduler* sched = nullptr;
  int running_cpu = -2;
  bool started = false;
  bool release = false;
  bool finished = false;
};

static void RunningCpuWorker(void* const raw_ctx) {
  RunningCpuCtx* const ctx = static_cast<RunningCpuCtx*>(raw_ctx);
  ctx->running_cpu = TaskRunningCpu(ctx->sched->CurrentTask());
  ctx->started = true;
  while (!ctx->release) {
    ctx->sched->Yield();
  }
  ctx->finished = true;
}

TEST(CoreRunqueueSchedulerTest,
     TaskRunningCpuReportsTheCpuOnlyWhileTheTaskRuns) {
  SetupEnv(3);
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);
  EXPECT_THAT(TaskRunningCpu(core->CurrentTask()), t::Eq(0));

  RunningCpuCtx ctx;
  ctx.sched = core.get();
  Task* const task =
      core->CreateTaskOnCpu(RunningCpuWorker, &ctx, kDefaultTaskWeight, 2);
  ASSERT_THAT(task, t::NotNull());
  // Queued and not started yet, also after another CPU took it.
  EXPECT_THAT(TaskRunningCpu(task), t::Eq(-1));
  EXPECT_TRUE(TaskSchedulerInternals::StealTask(*core, 0, 2));
  EXPECT_THAT(TaskRunningCpu(task), t::Eq(-1));

  // The task saw itself running on CPU 0, and it is switched out while this
  // task runs.
  while (!ctx.started) {
    core->Yield();
  }
  EXPECT_THAT(ctx.running_cpu, t::Eq(0));
  EXPECT_THAT(TaskRunningCpu(task), t::Eq(-1));

  // Ended, but not reclaimed yet.
  ctx.release = true;
  while (!ctx.finished) {
    core->Yield();
  }
  EXPECT_THAT(TaskRunningCpu(task), t::Eq(-1));
  core->Join(task);
}

struct TaskTimerTicksCtx {
  TaskScheduler* sched = nullptr;
  CpuClockSample at_start;
  CpuClockSample at_end;
  bool started = false;
  bool release = false;
};

// Samples its clock, takes three timer interrupts (counted as idt.cpp counts
// them), and samples its clock again once released, maybe on another CPU.
static void TaskTimerTicksWorker(void* const raw_ctx) {
  TaskTimerTicksCtx* const ctx = static_cast<TaskTimerTicksCtx*>(raw_ctx);
  ctx->at_start = ReadCpuClock();
  for (int i = 0; i < 3; ++i) {
    CurrentCpu()->task_timer_ticks.fetch_add(1, std::memory_order_relaxed);
  }
  ctx->started = true;
  while (!ctx->release) {
    ctx->sched->Yield();
  }
  ctx->at_end = ReadCpuClock();
}

TEST(CoreRunqueueSchedulerTest, KeepsEachTasksTimerTicksAcrossContextSwitches) {
  SetupEnv(2);
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);
  CpuLocal& cpu0 = *SmpGetCpuLocal(0);
  CpuLocal& cpu1 = *SmpGetCpuLocal(1);
  // So far, 7 timer interrupts interrupted this task and 11 CPU 1's idle task.
  cpu0.task_timer_ticks.store(7, std::memory_order_relaxed);
  cpu1.task_timer_ticks.store(11, std::memory_order_relaxed);

  TaskTimerTicksCtx ctx;
  ctx.sched = core.get();
  Task* const task =
      core->CreateTaskOnCpu(TaskTimerTicksWorker, &ctx, kDefaultTaskWeight, 0);
  ASSERT_THAT(task, t::NotNull());
  while (!ctx.started) {
    core->Yield();
  }
  // The new task started from 0, and this task got its own count back.
  EXPECT_THAT(ctx.at_start.cpu, t::Eq(0));
  EXPECT_THAT(ctx.at_start.task_ticks, t::Eq(0));
  EXPECT_THAT(cpu0.task_timer_ticks.load(), t::Eq(7));

  // The task moves to CPU 1, whose idle loop runs it to its end.
  EXPECT_TRUE(TaskSchedulerInternals::StealTask(*core, 1, 0));
  ctx.release = true;
  BindCpuLocal(&cpu1);
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*core, 1));
  BindCpuLocal(&cpu0);
  core->Join(task);
  // Its three ticks went with it, and CPU 1's idle task got its count back.
  EXPECT_THAT(ctx.at_end.cpu, t::Eq(1));
  EXPECT_THAT(ctx.at_end.task_ticks, t::Eq(3));
  EXPECT_THAT(cpu1.task_timer_ticks.load(), t::Eq(11));
  EXPECT_THAT(cpu0.task_timer_ticks.load(), t::Eq(7));
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

struct WrapAroundCtx {
  TaskScheduler* sched = nullptr;
  std::vector<int> dispatch_order;
};

struct WrapAroundTaskArg {
  WrapAroundCtx* ctx = nullptr;
  int id = 0;
  int slices = 0;
};

static void WrapAroundWorker(void* const raw_arg) {
  const WrapAroundTaskArg* const arg =
      static_cast<const WrapAroundTaskArg*>(raw_arg);
  for (int i = 0; i < arg->slices; ++i) {
    arg->ctx->dispatch_order.push_back(arg->id);
    arg->ctx->sched->Yield();
  }
}

// The core keeps vruntime modulo 2^64 and starts every CPU a few
// default-weight charges below the wrap, so a weight-1 task's first slice
// (2^20) takes it past the wrap at once, and a default-weight one (1024 per
// slice) follows a few slices later. Tasks must still run in the order of how
// far they got, not of the wrapped values: after one slice each, the
// default-weight task is 2^20 - 1024 behind, so it runs all its 8 slices
// before the weight-1 task runs again.
TEST(CoreRunqueueSchedulerTest, VruntimeOrderHoldsAcrossTheWrapAround) {
  SetupEnv(1);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  WrapAroundCtx ctx;
  ctx.sched = core.get();
  WrapAroundTaskArg light_arg{&ctx, /*id=*/1, /*slices=*/2};
  WrapAroundTaskArg normal_arg{&ctx, /*id=*/2, /*slices=*/8};
  Task* const light =
      core->CreateTask(WrapAroundWorker, &light_arg, kMinTaskWeight);
  Task* const normal =
      core->CreateTask(WrapAroundWorker, &normal_arg, kDefaultTaskWeight);
  ASSERT_THAT(light, t::NotNull());
  ASSERT_THAT(normal, t::NotNull());
  core->Join(light);
  core->Join(normal);

  EXPECT_THAT(ctx.dispatch_order, t::ElementsAre(1, 2, 2, 2, 2, 2, 2, 2, 2, 1));

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

// Called by the bootstrap task: yields until the tasks queued on CPU 0 have
// run, for at most a few yields, so that a test whose task landed on another
// CPU fails instead of hanging in Join().
static void YieldUntilCpu0QueueIsEmpty(TaskScheduler& core) {
  for (int i = 0; i < 8 && TaskSchedulerInternals::RunqueueLoad(core, 0) > 0;
       ++i) {
    core.Yield();
  }
}

TEST(CoreRunqueueSchedulerTest,
     StealTaskMovesTheTailTaskAndRespectsDestinationAndSourceState) {
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

  // 2. Enqueue two tasks on CPU 2 (`t1` first, `t2` second). Both start at CPU
  //    2's `min_vruntime`, so the later `enqueue_seq` puts `t2` at the tail.
  Task* const t1 =
      core->CreateTaskOnCpu(RecordIdOneWorker, &ctx, kDefaultTaskWeight, 2);
  Task* const t2 =
      core->CreateTaskOnCpu(RecordIdTwoWorker, &ctx, kDefaultTaskWeight, 2);
  ASSERT_THAT(t1, t::NotNull());
  ASSERT_THAT(t2, t::NotNull());
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 2), t::Eq(2));

  // 3. Test `dst_cpu > src_cpu`: steal from CPU 2 to CPU 3 (`3 > 2`).
  //    Must steal the highest-key task (`t2`, later `enqueue_seq`).
  EXPECT_TRUE(TaskSchedulerInternals::StealTask(*core, 3, 2));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 2), t::Eq(1));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 3), t::Eq(1));

  // 4. Calling `StealTask(3, 2)` again while CPU 3 is already non-empty
  //    (`RunqueueLoad(3) == 1`) must return `true` immediately WITHOUT stealing
  //    `t1` from CPU 2.
  EXPECT_TRUE(TaskSchedulerInternals::StealTask(*core, 3, 2));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 2), t::Eq(1));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 3), t::Eq(1));

  // 5. Offline CPU check: if either `dst_cpu` or `src_cpu` is offline,
  //    `StealTask` returns `false` without stealing.
  g_env.cpu_locals[1].online = false;
  EXPECT_FALSE(TaskSchedulerInternals::StealTask(*core, 1, 2));
  EXPECT_FALSE(TaskSchedulerInternals::StealTask(*core, 0, 1));
  g_env.cpu_locals[1].online = true;

  // 6. Test `dst_cpu < src_cpu`: steal `t2` from CPU 3 to CPU 0 (`0 < 3`) and
  //    run it, then steal `t1` from CPU 2 to CPU 0 (`0 < 2`) and run it.
  EXPECT_TRUE(TaskSchedulerInternals::StealTask(*core, 0, 3));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 3), t::Eq(0));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(1));
  YieldUntilCpu0QueueIsEmpty(*core);
  EXPECT_THAT(ctx.execution_order, t::ElementsAre(2));

  EXPECT_TRUE(TaskSchedulerInternals::StealTask(*core, 0, 2));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 2), t::Eq(0));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(1));
  YieldUntilCpu0QueueIsEmpty(*core);
  EXPECT_THAT(ctx.execution_order, t::ElementsAre(2, 1));
  core->Join(t1);
  core->Join(t2);

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

struct LargestVruntimeCtx {
  TaskScheduler* sched = nullptr;
  int ahead_first_cpu = -1;
  int ahead_resumed_cpu = -1;
  int newcomer_cpu = -1;
};

// Created with weight 1, so the Yield() in its first slice charges it about
// 1M of vruntime and leaves it queued far ahead of any task created later.
static void AheadWorker(void* const raw_arg) {
  LargestVruntimeCtx* const ctx = static_cast<LargestVruntimeCtx*>(raw_arg);
  ctx->ahead_first_cpu = CurrentCpuId();
  ctx->sched->Yield();
  ctx->ahead_resumed_cpu = CurrentCpuId();
}

static void NewcomerWorker(void* const raw_arg) {
  LargestVruntimeCtx* const ctx = static_cast<LargestVruntimeCtx*>(raw_arg);
  ctx->newcomer_cpu = CurrentCpuId();
}

TEST(CoreRunqueueSchedulerTest,
     StealTaskMovesTheTaskWithTheLargestVruntimeNotTheNewestOrLeftmostOne) {
  SetupEnv(3);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  LargestVruntimeCtx ctx;
  ctx.sched = core.get();

  // 1. Run the first slice of `ahead`; its Yield() switches back to the
  //    bootstrap task and leaves `ahead` queued on CPU 0 with about 1M of
  //    vruntime.
  Task* const ahead =
      core->CreateTaskOnCpu(AheadWorker, &ctx, kMinTaskWeight, 0);
  ASSERT_THAT(ahead, t::NotNull());
  core->Yield();
  ASSERT_THAT(ctx.ahead_first_cpu, t::Eq(0));
  ASSERT_THAT(ctx.ahead_resumed_cpu, t::Eq(-1));

  // 2. `newcomer` is enqueued after `ahead` but starts at CPU 0's
  //    min_vruntime, so it is the leftmost task and `ahead` the rightmost.
  Task* const newcomer =
      core->CreateTaskOnCpu(NewcomerWorker, &ctx, kDefaultTaskWeight, 0);
  ASSERT_THAT(newcomer, t::NotNull());
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(2));

  // 3. StealTask() takes `ahead`, the task that would wait longest on CPU 0,
  //    rather than the newest or the leftmost task (`newcomer` in both cases).
  EXPECT_TRUE(TaskSchedulerInternals::StealTask(*core, 2, 0));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(1));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 2), t::Eq(1));

  // 4. `ahead` resumes on CPU 2 when the calling thread stands in for CPU 2's
  //    idle loop, and `newcomer` runs on CPU 0.
  BindCpuLocal(SmpGetCpuLocal(2));
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*core, 2));
  BindCpuLocal(SmpGetCpuLocal(0));
  core->Join(newcomer);
  core->Join(ahead);
  EXPECT_THAT(ctx.ahead_resumed_cpu, t::Eq(2));
  EXPECT_THAT(ctx.newcomer_cpu, t::Eq(0));

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
  int first_target_cpu = -1;
  int second_target_cpu = -1;
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
     StealTaskNeverMovesBootstrapTaskAtHeadOfSourceRunqueue) {
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
  //    CPU 0's runqueue and the helper, with about 1M of vruntime, is behind
  //    it at the tail. StealTask(2, 0) must move the helper.
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

static void FirstTargetWorker(void* const raw_arg) {
  static_cast<SkipBootstrapCtx*>(raw_arg)->first_target_cpu = CurrentCpuId();
}

static void SecondTargetWorker(void* const raw_arg) {
  static_cast<SkipBootstrapCtx*>(raw_arg)->second_target_cpu = CurrentCpuId();
}

TEST(CoreRunqueueSchedulerTest,
     StealTaskSkipsBootstrapTaskAtTailOfSourceRunqueue) {
  SetupEnv(3);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);
  Task* const bootstrap = core->CurrentTask();

  SkipBootstrapCtx ctx;
  ctx.sched = core.get();

  // The probe and both targets start at CPU 0's min_vruntime, which is the
  // bootstrap task's vruntime. Join(probe) charges the bootstrap task for a
  // Yield() before it switches to the probe, so while the probe runs, CPU 0's
  // runqueue holds the first target, the second target and, at the tail, the
  // bootstrap task. StealTask(2, 0) must skip the bootstrap task and move the
  // second target.
  Task* const probe = core->CreateTaskOnCpu(StealFromCpu0ProbeWorker,  //
                                            &ctx,                      //
                                            kDefaultTaskWeight,        //
                                            0);
  Task* const first_target =
      core->CreateTaskOnCpu(FirstTargetWorker, &ctx, kDefaultTaskWeight, 0);
  Task* const second_target =
      core->CreateTaskOnCpu(SecondTargetWorker, &ctx, kDefaultTaskWeight, 0);
  ASSERT_THAT(probe, t::NotNull());
  ASSERT_THAT(first_target, t::NotNull());
  ASSERT_THAT(second_target, t::NotNull());
  core->Join(probe);
  EXPECT_THAT(ctx.probe.src_load_before, t::Eq(2));
  EXPECT_TRUE(ctx.probe.stolen);
  EXPECT_THAT(ctx.probe.src_load_after, t::Eq(1));
  EXPECT_THAT(ctx.probe.dst_load_after, t::Eq(1));
  EXPECT_THAT(core->CurrentTask(), t::Eq(bootstrap));

  // The first target ran on CPU 0 before the bootstrap task, and the second
  // target runs on CPU 2 when the calling thread stands in for its idle loop.
  BindCpuLocal(SmpGetCpuLocal(2));
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*core, 2));
  BindCpuLocal(SmpGetCpuLocal(0));
  core->Join(first_target);
  core->Join(second_target);
  EXPECT_THAT(ctx.first_target_cpu, t::Eq(0));
  EXPECT_THAT(ctx.second_target_cpu, t::Eq(2));

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

struct BusyProbeCtx {
  TaskScheduler* sched = nullptr;
  int cpu = -1;
  bool own_cpu_busy = false;
  int own_cpu_load = -1;
  bool idle_cpu_busy = true;
};

// Records what the core reports, while this task runs with nothing queued
// behind it, for its own CPU `ctx->cpu` and for the idle CPU 2.
static void BusyProbeWorker(void* const raw_arg) {
  BusyProbeCtx* const ctx = static_cast<BusyProbeCtx*>(raw_arg);
  ctx->own_cpu_busy = TaskSchedulerInternals::IsCpuBusy(*ctx->sched, ctx->cpu);
  ctx->own_cpu_load =
      TaskSchedulerInternals::RunqueueLoad(*ctx->sched, ctx->cpu);
  ctx->idle_cpu_busy = TaskSchedulerInternals::IsCpuBusy(*ctx->sched, 2);
}

TEST(CoreRunqueueSchedulerTest,
     IsCpuBusyTellsBusyCpuWithNothingQueuedFromIdle) {
  SetupEnv(3);
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  // 1. A task queued on an idle AP does not make it busy.
  BusyProbeCtx ap_ctx;
  ap_ctx.sched = core.get();
  ap_ctx.cpu = 1;
  Task* const ap_probe =
      core->CreateTaskOnCpu(BusyProbeWorker, &ap_ctx, kDefaultTaskWeight, 1);
  ASSERT_THAT(ap_probe, t::NotNull());
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 1), t::Eq(1));
  EXPECT_FALSE(TaskSchedulerInternals::IsCpuBusy(*core, 1));

  // 2. While the probe runs on CPU 1 with nothing queued, RunqueueLoad(1) is 0
  //    as for the idle CPU 2, but only CPU 1 is busy. Once the probe has
  //    exited, CPU 1 runs its idle task again.
  BindCpuLocal(SmpGetCpuLocal(1));
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*core, 1));
  BindCpuLocal(SmpGetCpuLocal(0));
  EXPECT_TRUE(ap_ctx.own_cpu_busy);
  EXPECT_THAT(ap_ctx.own_cpu_load, t::Eq(0));
  EXPECT_FALSE(ap_ctx.idle_cpu_busy);
  EXPECT_FALSE(TaskSchedulerInternals::IsCpuBusy(*core, 1));
  core->Join(ap_probe);

  // 3. CPU 0 has no idle task: it is busy whether it runs the bootstrap task
  //    or, while the bootstrap task waits in Join(), another task.
  EXPECT_TRUE(TaskSchedulerInternals::IsCpuBusy(*core, 0));
  BusyProbeCtx cpu0_ctx;
  cpu0_ctx.sched = core.get();
  cpu0_ctx.cpu = 0;
  Task* const cpu0_probe =
      core->CreateTaskOnCpu(BusyProbeWorker, &cpu0_ctx, kDefaultTaskWeight, 0);
  ASSERT_THAT(cpu0_probe, t::NotNull());
  core->Join(cpu0_probe);
  EXPECT_TRUE(cpu0_ctx.own_cpu_busy);
  EXPECT_THAT(cpu0_ctx.own_cpu_load, t::Eq(0));
  EXPECT_FALSE(cpu0_ctx.idle_cpu_busy);
  EXPECT_TRUE(TaskSchedulerInternals::IsCpuBusy(*core, 0));
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
// `OnTimerInterrupt`, `SetAddressSpace`, `SetFpuState`)
// ============================================================================

struct MtCoreSharedCtx {
  TaskScheduler* sched = nullptr;
  std::atomic<uint32_t> cpu_mask{0};
  std::atomic<int> completed_tasks{0};
  std::atomic<int64_t> total_iterations{0};
  std::atomic<int64_t> remote_executions{0};
  // Times a task saw its own CPU reported idle, or an idle loop saw its CPU
  // reported busy.
  std::atomic<int64_t> busy_state_violations{0};
  // Times a task found its CPU in another address space than its own.
  std::atomic<int64_t> address_space_violations{0};
  // Times a task found another FPU state than its own loaded on its CPU.
  std::atomic<int64_t> fpu_state_violations{0};
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

  // Each task runs in an address space of its own, which every CPU that runs
  // the task must switch to, and keeps an FPU state of its own, which every
  // CPU that runs the task must load. The task exits with the state attached.
  AddressSpace* const address_space = FakeAddressSpace(arg->task_index);
  sched->SetAddressSpace(address_space);
  FpuState* const fpu_state = FakeFpuState(arg->task_index);
  sched->SetFpuState(fpu_state);

  for (int round = 0; round < 6; ++round) {
    const int cpu = CurrentCpuId();
    shared->cpu_mask.fetch_or(1u << static_cast<uint32_t>(cpu),
                              std::memory_order_acq_rel);
    if (cpu != 0) {
      shared->remote_executions.fetch_add(1, std::memory_order_acq_rel);
    }
    if (!TaskSchedulerInternals::IsCpuBusy(*sched, cpu)) {
      shared->busy_state_violations.fetch_add(1, std::memory_order_acq_rel);
    }
    if (ActiveAddressSpace(cpu) != address_space) {
      shared->address_space_violations.fetch_add(1, std::memory_order_acq_rel);
    }
    if (LoadedFpuState(cpu) != fpu_state) {
      shared->fpu_state_violations.fetch_add(1, std::memory_order_acq_rel);
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
    ap_threads.emplace_back([cpu, &core, &shared, &ready_workers,
                             &stop_workers]() {
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
        const bool dispatched = TaskSchedulerInternals::PollIdleCpu(*core, cpu);
        // Only this thread switches CPU `cpu`'s tasks, and its idle task
        // runs again once PollIdleCpu() returns.
        if (TaskSchedulerInternals::IsCpuBusy(*core, cpu)) {
          shared.busy_state_violations.fetch_add(1, std::memory_order_acq_rel);
        }
        if (!dispatched) {
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
  EXPECT_THAT(shared.busy_state_violations.load(), t::Eq(0));
  EXPECT_THAT(shared.address_space_violations.load(), t::Eq(0));
  // The bootstrap task never left the kernel's page tables.
  EXPECT_THAT(ActiveAddressSpace(0), t::IsNull());
  // Every CPU that ran a task had loaded the task's FPU state and saved it
  // into nothing else, and each task's state was loaded once more than it was
  // saved: when the task attached it, and whenever a CPU resumed the task,
  // while the task was saved whenever it was switched out, except when it
  // exited.
  EXPECT_THAT(shared.fpu_state_violations.load(), t::Eq(0));
  EXPECT_THAT(g_env.fpu_save_mismatches.load(), t::Eq(0));
  const std::vector<FakeFpuCall> fpu_calls = FpuCalls();
  for (int i = 0; i < kNumTasks; ++i) {
    const int restores =
        CountFpuCalls(fpu_calls, FakeFpuOp::kRestore, FakeFpuState(i));
    const int saves =
        CountFpuCalls(fpu_calls, FakeFpuOp::kSave, FakeFpuState(i));
    EXPECT_THAT(restores, t::Eq(saves + 1)) << "task " << i;
  }

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

  // 6. Switching tasks while the user-access window is open (user_access.h)
  //    must trigger `DCHECK`: RFLAGS doesn't switch with the task, so the next
  //    task would run with SMAP lifted.
  EXPECT_DEATH(
      {
        Task* const task =
            core->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 0);
        ASSERT_THAT(task, t::NotNull());
        SetUserAccessWindowOpenForTest(true);
        core->Yield();
      },
      "Check failed: !UserAccessWindowOpen\\(\\)");
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

// ============================================================================
// 11. Address Spaces (`SetAddressSpace`)
// ============================================================================

struct AddressSpaceTaskCtx {
  TaskScheduler* sched = nullptr;
  AddressSpace* address_space = nullptr;
  // The address space that CPU 0 used when the task started.
  const AddressSpace* active_at_start = FakeAddressSpace(0);
  // Times the task found CPU 0 in another address space than its own after
  // its SetAddressSpace() call.
  int mismatches = 0;
};

static void AddressSpaceWorker(void* const raw_arg) {
  AddressSpaceTaskCtx* const ctx = static_cast<AddressSpaceTaskCtx*>(raw_arg);
  ctx->active_at_start = ActiveAddressSpace(0);
  ctx->sched->SetAddressSpace(ctx->address_space);
  for (int i = 0; i < 4; ++i) {
    if (ActiveAddressSpace(0) != ctx->address_space) {
      ++ctx->mismatches;
    }
    ctx->sched->Yield();
  }
  if (ActiveAddressSpace(0) != ctx->address_space) {
    ++ctx->mismatches;
  }
}

TEST(CoreRunqueueSchedulerTest,
     SetAddressSpaceSwitchesTheCpuWithTheTaskThatRunsOnIt) {
  SetupEnv(1);
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);
  AddressSpace* const bootstrap_space = FakeAddressSpace(1);

  // The calling task's CPU switches at once, but only if the address space
  // changes.
  EXPECT_THAT(ActiveAddressSpace(0), t::IsNull());
  core->SetAddressSpace(bootstrap_space);
  EXPECT_THAT(ActiveAddressSpace(0), t::Eq(bootstrap_space));
  EXPECT_THAT(g_env.address_space_switches.load(), t::Eq(1));
  core->SetAddressSpace(bootstrap_space);
  EXPECT_THAT(g_env.address_space_switches.load(), t::Eq(1));

  // Tasks start in the kernel's page tables, and CPU 0 always uses the address
  // space of the task that it runs: of each task once it has called
  // SetAddressSpace(), and the bootstrap task's again once both tasks have
  // exited, one of them in its address space.
  AddressSpaceTaskCtx ctx_a;
  ctx_a.sched = core.get();
  ctx_a.address_space = FakeAddressSpace(2);
  AddressSpaceTaskCtx ctx_b;
  ctx_b.sched = core.get();
  ctx_b.address_space = FakeAddressSpace(3);
  Task* const task_a =
      core->CreateTask(AddressSpaceWorker, &ctx_a, kDefaultTaskWeight);
  Task* const task_b =
      core->CreateTask(AddressSpaceWorker, &ctx_b, kDefaultTaskWeight);
  ASSERT_THAT(task_a, t::NotNull());
  ASSERT_THAT(task_b, t::NotNull());
  core->Join(task_a);
  core->Join(task_b);

  EXPECT_THAT(ctx_a.active_at_start, t::IsNull());
  EXPECT_THAT(ctx_b.active_at_start, t::IsNull());
  EXPECT_THAT(ctx_a.mismatches, t::Eq(0));
  EXPECT_THAT(ctx_b.mismatches, t::Eq(0));
  EXPECT_THAT(ActiveAddressSpace(0), t::Eq(bootstrap_space));

  core->SetAddressSpace(nullptr);
  EXPECT_THAT(ActiveAddressSpace(0), t::IsNull());
}

// ============================================================================
// 12. x87/MMX/SSE/AVX Register State (`SetFpuState`)
// ============================================================================

struct FpuStateTaskCtx {
  TaskScheduler* sched = nullptr;
  FpuState* state = nullptr;
  // Times the task found another state than its own loaded on CPU 0 after
  // its SetFpuState() call.
  int mismatches = 0;
};

static void FpuStateWorker(void* const raw_arg) {
  FpuStateTaskCtx* const ctx = static_cast<FpuStateTaskCtx*>(raw_arg);
  ctx->sched->SetFpuState(ctx->state);
  for (int i = 0; i < 4; ++i) {
    if (LoadedFpuState(0) != ctx->state) {
      ++ctx->mismatches;
    }
    ctx->sched->Yield();
  }
  if (LoadedFpuState(0) != ctx->state) {
    ++ctx->mismatches;
  }
  ctx->sched->SetFpuState(nullptr);
}

TEST(CoreRunqueueSchedulerTest,
     SetFpuStateLoadsTheStateAtOnceAndOnEveryResume) {
  SetupEnv(1);
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);
  FpuState* const bootstrap_state = FakeFpuState(1);

  // The calling task's state is loaded at once, and the registers' previous
  // values are not saved anywhere.
  core->SetFpuState(bootstrap_state);
  EXPECT_THAT(FpuCalls(), t::ElementsAre(FakeFpuCall{FakeFpuOp::kRestore,  //
                                                     0,                    //
                                                     bootstrap_state}));
  EXPECT_THAT(LoadedFpuState(0), t::Eq(bootstrap_state));

  // CPU 0 always holds the state of the task with a state that it runs: of
  // each worker after its SetFpuState() call, on every resume, and the
  // bootstrap task's again once both workers have exited.
  FpuStateTaskCtx ctx_a;
  ctx_a.sched = core.get();
  ctx_a.state = FakeFpuState(2);
  FpuStateTaskCtx ctx_b;
  ctx_b.sched = core.get();
  ctx_b.state = FakeFpuState(3);
  Task* const task_a =
      core->CreateTask(FpuStateWorker, &ctx_a, kDefaultTaskWeight);
  Task* const task_b =
      core->CreateTask(FpuStateWorker, &ctx_b, kDefaultTaskWeight);
  ASSERT_THAT(task_a, t::NotNull());
  ASSERT_THAT(task_b, t::NotNull());
  core->Join(task_a);
  core->Join(task_b);

  EXPECT_THAT(ctx_a.mismatches, t::Eq(0));
  EXPECT_THAT(ctx_b.mismatches, t::Eq(0));
  EXPECT_THAT(g_env.fpu_save_mismatches.load(), t::Eq(0));
  EXPECT_THAT(LoadedFpuState(0), t::Eq(bootstrap_state));
  // Each state was loaded when its task attached it and on every resume, and
  // saved whenever its task was switched out with it attached. A worker was
  // switched out at least once while the other one was queued.
  const std::vector<FakeFpuCall> calls = FpuCalls();
  for (const FpuState* const state :
       {bootstrap_state, ctx_a.state, ctx_b.state}) {
    EXPECT_THAT(CountFpuCalls(calls, FakeFpuOp::kRestore, state),
                t::Eq(CountFpuCalls(calls, FakeFpuOp::kSave, state) + 1));
  }
  EXPECT_THAT(CountFpuCalls(calls, FakeFpuOp::kSave, ctx_a.state) +
                  CountFpuCalls(calls, FakeFpuOp::kSave, ctx_b.state),
              t::Ge(1));

  // Detaching drops the registers without saving them.
  core->SetFpuState(nullptr);
  EXPECT_THAT(FpuCalls(), t::SizeIs(calls.size()));
}

struct FpuOrderTaskCtx {
  TaskScheduler* sched = nullptr;
  FpuState* state = nullptr;
  std::atomic<bool> attached{false};
  std::atomic<bool> release{false};
};

static void FpuOrderWorker(void* const raw_arg) {
  FpuOrderTaskCtx* const ctx = static_cast<FpuOrderTaskCtx*>(raw_arg);
  ctx->sched->SetFpuState(ctx->state);
  ctx->attached.store(true, std::memory_order_release);
  while (!ctx->release.load(std::memory_order_acquire)) {
    ctx->sched->Yield();
  }
  ctx->sched->SetFpuState(nullptr);
}

TEST(CoreRunqueueSchedulerTest,
     SwitchSavesThePreviousStateBeforeLoadingTheNext) {
  SetupEnv(1);
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);
  FpuState* const bootstrap_state = FakeFpuState(1);
  FpuState* const task_state = FakeFpuState(2);
  core->SetFpuState(bootstrap_state);

  FpuOrderTaskCtx ctx;
  ctx.sched = core.get();
  ctx.state = task_state;
  Task* const task = core->CreateTask(FpuOrderWorker, &ctx, kDefaultTaskWeight);
  ASSERT_THAT(task, t::NotNull());
  while (!ctx.attached.load(std::memory_order_acquire)) {
    core->Yield();
  }

  // A switch to the task and one back: each saves the registers of the task
  // that leaves the CPU before it loads those of the task that gets it.
  ClearFpuCalls();
  while (FpuCalls().size() < 4) {
    core->Yield();
  }
  EXPECT_THAT(
      FpuCalls(),
      t::ElementsAre(FakeFpuCall{FakeFpuOp::kSave, 0, bootstrap_state},
                     FakeFpuCall{FakeFpuOp::kRestore, 0, task_state},
                     FakeFpuCall{FakeFpuOp::kSave, 0, task_state},
                     FakeFpuCall{FakeFpuOp::kRestore, 0, bootstrap_state}));

  // The task detaches its state and exits: neither saves its registers.
  ClearFpuCalls();
  ctx.release.store(true, std::memory_order_release);
  core->Join(task);
  EXPECT_THAT(
      FpuCalls(),
      t::ElementsAre(FakeFpuCall{FakeFpuOp::kSave, 0, bootstrap_state},
                     FakeFpuCall{FakeFpuOp::kRestore, 0, task_state},
                     FakeFpuCall{FakeFpuOp::kRestore, 0, bootstrap_state}));
  EXPECT_THAT(g_env.fpu_save_mismatches.load(), t::Eq(0));
  core->SetFpuState(nullptr);
}

static void YieldFourTimes(void* const raw_sched) {
  TaskScheduler* const sched = static_cast<TaskScheduler*>(raw_sched);
  for (int i = 0; i < 4; ++i) {
    sched->Yield();
  }
}

static void AttachFpuStateAndExit(void* const raw_ctx) {
  FpuStateTaskCtx* const ctx = static_cast<FpuStateTaskCtx*>(raw_ctx);
  ctx->sched->SetFpuState(ctx->state);
}

TEST(CoreRunqueueSchedulerTest,
     ExitingTasksAreNotSavedAndTasksWithoutAStateSkipBothSteps) {
  SetupEnv(1);
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  // Switches between tasks without a state neither save nor load registers.
  Task* kernel_tasks[3] = {};
  for (Task*& task : kernel_tasks) {
    task = core->CreateTask(YieldFourTimes, core.get(), kDefaultTaskWeight);
    ASSERT_THAT(task, t::NotNull());
  }
  for (Task* const task : kernel_tasks) {
    core->Join(task);
  }
  EXPECT_THAT(FpuCalls(), t::IsEmpty());

  // A task that exits with its state attached: loaded once, never saved.
  FpuStateTaskCtx exiting_ctx;
  exiting_ctx.sched = core.get();
  exiting_ctx.state = FakeFpuState(4);
  Task* const exiting_task =
      core->CreateTask(AttachFpuStateAndExit, &exiting_ctx, kDefaultTaskWeight);
  ASSERT_THAT(exiting_task, t::NotNull());
  core->Join(exiting_task);
  EXPECT_THAT(FpuCalls(), t::ElementsAre(FakeFpuCall{FakeFpuOp::kRestore,  //
                                                     0,                    //
                                                     exiting_ctx.state}));

  // Switches between a task with a state and one without: only the state of
  // the former is saved and loaded, once per switch.
  ClearFpuCalls();
  FpuState* const bootstrap_state = FakeFpuState(1);
  core->SetFpuState(bootstrap_state);
  Task* const kernel_task =
      core->CreateTask(YieldFourTimes, core.get(), kDefaultTaskWeight);
  ASSERT_THAT(kernel_task, t::NotNull());
  core->Join(kernel_task);
  const std::vector<FakeFpuCall> calls = FpuCalls();
  EXPECT_THAT(calls,
              t::Each(t::Field(&FakeFpuCall::state, t::Eq(bootstrap_state))));
  EXPECT_THAT(CountFpuCalls(calls, FakeFpuOp::kSave, bootstrap_state),
              t::Ge(1));
  EXPECT_THAT(
      CountFpuCalls(calls, FakeFpuOp::kRestore, bootstrap_state),
      t::Eq(CountFpuCalls(calls, FakeFpuOp::kSave, bootstrap_state) + 1));
  EXPECT_THAT(g_env.fpu_save_mismatches.load(), t::Eq(0));
  core->SetFpuState(nullptr);
}

struct FpuMigrationTaskCtx {
  TaskScheduler* sched = nullptr;
  FpuState* state = nullptr;
  std::atomic<bool> attached{false};
  // The CPU on which the task attached its state, the CPU that resumed it
  // after it left that CPU, and the state that the latter held then.
  int attach_cpu = -1;
  int resume_cpu = -1;
  const FpuState* loaded_on_resume = nullptr;
};

static void FpuMigrationWorker(void* const raw_arg) {
  FpuMigrationTaskCtx* const ctx = static_cast<FpuMigrationTaskCtx*>(raw_arg);
  ctx->sched->SetFpuState(ctx->state);
  ctx->attach_cpu = CurrentCpuId();
  ctx->attached.store(true, std::memory_order_release);
  // Bounded, so that the test fails rather than hangs if no other CPU takes
  // the task.
  for (int i = 0; i < 1000 && CurrentCpuId() == ctx->attach_cpu; ++i) {
    ctx->sched->Yield();
  }
  ctx->resume_cpu = CurrentCpuId();
  ctx->loaded_on_resume = LoadedFpuState(ctx->resume_cpu);
  ctx->sched->SetFpuState(nullptr);
}

TEST(CoreRunqueueSchedulerTest, FpuStateFollowsATaskThatMovesToAnotherCpu) {
  SetupEnv(2);
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);
  FpuState* const bootstrap_state = FakeFpuState(1);
  FpuState* const task_state = FakeFpuState(2);
  core->SetFpuState(bootstrap_state);

  // The task attaches its state on CPU 0 and yields until it runs elsewhere.
  FpuMigrationTaskCtx ctx;
  ctx.sched = core.get();
  ctx.state = task_state;
  Task* const task =
      core->CreateTaskOnCpu(FpuMigrationWorker, &ctx, kDefaultTaskWeight, 0);
  ASSERT_THAT(task, t::NotNull());
  while (!ctx.attached.load(std::memory_order_acquire)) {
    core->Yield();
  }
  ASSERT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(1));

  // CPU 1 takes the queued task from CPU 0 and runs it until it exits.
  std::thread cpu1([&core]() {
    BindCpuLocal(SmpGetCpuLocal(1));
    SetInterruptsEnabledForTest(true);
    EXPECT_TRUE(TaskSchedulerInternals::StealTask(*core, 1, 0));
    EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*core, 1));
    ResetCpuLocalForTest();
  });
  cpu1.join();
  core->Join(task);

  // CPU 0 saved the task's registers when the task left it, and CPU 1 loaded
  // them before it resumed the task.
  EXPECT_THAT(ctx.attach_cpu, t::Eq(0));
  EXPECT_THAT(ctx.resume_cpu, t::Eq(1));
  EXPECT_THAT(ctx.loaded_on_resume, t::Eq(task_state));
  EXPECT_THAT(
      FpuCalls(),
      t::ElementsAre(FakeFpuCall{FakeFpuOp::kRestore, 0, bootstrap_state},
                     FakeFpuCall{FakeFpuOp::kSave, 0, bootstrap_state},
                     FakeFpuCall{FakeFpuOp::kRestore, 0, task_state},
                     FakeFpuCall{FakeFpuOp::kSave, 0, task_state},
                     FakeFpuCall{FakeFpuOp::kRestore, 0, bootstrap_state},
                     FakeFpuCall{FakeFpuOp::kRestore, 1, task_state}));
  EXPECT_THAT(g_env.fpu_save_mismatches.load(), t::Eq(0));
  EXPECT_THAT(LoadedFpuState(0), t::Eq(bootstrap_state));
  core->SetFpuState(nullptr);
}

// ============================================================================
// 13. Blocking (`Block`, `Wake`, `WakeOnCpu`) and `TaskTimerTicks`
// ============================================================================

// Bound on each wait of the blocking tests. Neither gtest nor make has a
// timeout, so a lost wakeup must fail a test instead of hanging it.
constexpr std::chrono::seconds kBlockWaitTimeout(30);

// Calls `step()` until `done()` holds or kBlockWaitTimeout has passed, and
// returns whether `done()` holds.
template <typename DoneFn, typename StepFn>
static bool WaitUntil(const DoneFn& done, const StepFn& step) {
  const auto deadline = std::chrono::steady_clock::now() + kBlockWaitTimeout;
  while (!done()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return done();
    }
    step();
  }
  return true;
}

// WaitUntil() that yields the calling task, and the host thread, in between.
template <typename DoneFn>
static bool YieldUntil(TaskScheduler& sched, const DoneFn& done) {
  return WaitUntil(done, [&sched]() {
    sched.Yield();
    std::this_thread::yield();
  });
}

static void SetAtomicFlagWorker(void* const raw_flag) {
  static_cast<std::atomic<bool>*>(raw_flag)->store(true,
                                                   std::memory_order_release);
}

struct BlockOnceCtx {
  TaskScheduler* sched = nullptr;
  std::atomic<int> phase{0};
  std::atomic<int> resumed_cpu{-1};
};

// Blocks once, then records the CPU that resumed it.
static void BlockOnceWorker(void* const raw_ctx) {
  BlockOnceCtx* const ctx = static_cast<BlockOnceCtx*>(raw_ctx);
  ctx->phase.store(1, std::memory_order_release);
  ctx->sched->Block();
  ctx->resumed_cpu.store(CurrentCpuId(), std::memory_order_release);
  ctx->phase.store(2, std::memory_order_release);
}

TEST(CoreRunqueueSchedulerTest,
     BlockSuspendsTheTaskUntilWakeAndWakeNeverSwitches) {
  SetupEnv(1);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);
  Task* const bootstrap = core->CurrentTask();
  const CpuLocal& cpu0 = *SmpGetCpuLocal(0);

  BlockOnceCtx ctx;
  ctx.sched = core.get();
  Task* const task =
      core->CreateTask(BlockOnceWorker, &ctx, kDefaultTaskWeight);
  ASSERT_THAT(task, t::NotNull());

  // The task runs and blocks, which switches back to this task.
  core->Yield();
  EXPECT_THAT(ctx.phase.load(), t::Eq(1));
  EXPECT_THAT(TaskRunningCpu(task), t::Eq(-1));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(0));

  // It is not runnable: yielding switches to nothing.
  const int64_t switches_while_blocked = cpu0.context_switches.load();
  for (int i = 0; i < 3; ++i) {
    core->Yield();
  }
  EXPECT_THAT(ctx.phase.load(), t::Eq(1));
  EXPECT_THAT(cpu0.context_switches.load(), t::Eq(switches_while_blocked));

  // Wake() queues it without switching to it.
  core->Wake(task);
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(1));
  EXPECT_THAT(core->CurrentTask(), t::Eq(bootstrap));
  EXPECT_THAT(cpu0.context_switches.load(), t::Eq(switches_while_blocked));
  EXPECT_THAT(ctx.phase.load(), t::Eq(1));

  EXPECT_TRUE(YieldUntil(*core, [&ctx]() { return ctx.phase.load() == 2; }));
  EXPECT_THAT(ctx.resumed_cpu.load(), t::Eq(0));
  core->Join(task);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

struct PermitCtx {
  TaskScheduler* sched = nullptr;
  int blocks_to_make = 0;
  std::atomic<bool> started{false};
  std::atomic<bool> go{false};
  std::atomic<int> returns{0};
  // Whether the CPU switched tasks during each Block() call: whether the call
  // blocked, rather than consumed a pending wakeup and returned at once.
  std::atomic<bool> switched[4] = {};
};

// Yields until released, then calls Block() `blocks_to_make` times and
// records which calls blocked. Used with a single CPU.
static void PermitWorker(void* const raw_ctx) {
  PermitCtx* const ctx = static_cast<PermitCtx*>(raw_ctx);
  ctx->started.store(true, std::memory_order_release);
  while (!ctx->go.load(std::memory_order_acquire)) {
    ctx->sched->Yield();
  }
  for (int i = 0; i < ctx->blocks_to_make; ++i) {
    const int64_t before =
        CurrentCpu()->context_switches.load(std::memory_order_relaxed);
    ctx->sched->Block();
    const int64_t after =
        CurrentCpu()->context_switches.load(std::memory_order_relaxed);
    ctx->switched[i].store(after != before, std::memory_order_release);
    ctx->returns.fetch_add(1, std::memory_order_acq_rel);
  }
}

TEST(CoreRunqueueSchedulerTest,
     WakeBeforeBlockLeavesAPendingWakeupThatTheNextBlockConsumes) {
  SetupEnv(1);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  PermitCtx ctx;
  ctx.sched = core.get();
  ctx.blocks_to_make = 2;
  Task* const task = core->CreateTask(PermitWorker, &ctx, kDefaultTaskWeight);
  ASSERT_THAT(task, t::NotNull());
  ASSERT_TRUE(YieldUntil(*core, [&ctx]() { return ctx.started.load(); }));

  // A Wake() of the queued task leaves a pending wakeup and queues nothing.
  core->Wake(task);
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(1));

  // The first Block() consumes it and returns at once, the second one blocks.
  ctx.go.store(true, std::memory_order_release);
  EXPECT_TRUE(YieldUntil(*core, [&ctx]() { return ctx.returns.load() >= 1; }));
  for (int i = 0; i < 3; ++i) {
    core->Yield();
  }
  EXPECT_FALSE(ctx.switched[0].load());
  EXPECT_THAT(ctx.returns.load(), t::Eq(1));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(0));
  EXPECT_THAT(TaskRunningCpu(task), t::Eq(-1));

  core->Wake(task);
  EXPECT_TRUE(YieldUntil(*core, [&ctx]() { return ctx.returns.load() == 2; }));
  EXPECT_TRUE(ctx.switched[1].load());
  core->Join(task);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

TEST(CoreRunqueueSchedulerTest,
     DoubleWakeLeavesOnePendingWakeupAndNeverQueuesTheTaskTwice) {
  SetupEnv(1);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  PermitCtx ctx;
  ctx.sched = core.get();
  ctx.blocks_to_make = 4;
  Task* const task = core->CreateTask(PermitWorker, &ctx, kDefaultTaskWeight);
  ASSERT_THAT(task, t::NotNull());
  ASSERT_TRUE(YieldUntil(*core, [&ctx]() { return ctx.started.load(); }));

  // Two Wake()s of the queued task: one pending wakeup, which is a flag, not a
  // count. The first Block() returns at once, the second one blocks.
  core->Wake(task);
  core->Wake(task);
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(1));
  ctx.go.store(true, std::memory_order_release);
  EXPECT_TRUE(YieldUntil(*core, [&ctx]() { return ctx.returns.load() >= 1; }));
  for (int i = 0; i < 3; ++i) {
    core->Yield();
  }
  EXPECT_FALSE(ctx.switched[0].load());
  EXPECT_THAT(ctx.returns.load(), t::Eq(1));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(0));
  EXPECT_THAT(TaskRunningCpu(task), t::Eq(-1));

  // Two Wake()s of the blocked task: the first queues it (a second runqueue
  // entry would fail a DCHECK), the second leaves a pending wakeup. So the
  // second Block() returns, the third returns at once without a Wake() of its
  // own (which the contract allows), and the fourth blocks.
  core->Wake(task);
  core->Wake(task);
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(1));
  EXPECT_TRUE(YieldUntil(*core, [&ctx]() { return ctx.returns.load() >= 3; }));
  for (int i = 0; i < 3; ++i) {
    core->Yield();
  }
  EXPECT_TRUE(ctx.switched[1].load());
  EXPECT_FALSE(ctx.switched[2].load());
  EXPECT_THAT(ctx.returns.load(), t::Eq(3));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(0));
  EXPECT_THAT(TaskRunningCpu(task), t::Eq(-1));

  core->Wake(task);
  EXPECT_TRUE(YieldUntil(*core, [&ctx]() { return ctx.returns.load() == 4; }));
  EXPECT_TRUE(ctx.switched[3].load());
  core->Join(task);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

TEST(CoreRunqueueSchedulerTest,
     BlockedTaskIsInvisibleToLoadStealingAndIdlePolling) {
  SetupEnv(3);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  BlockOnceCtx ctx;
  ctx.sched = core.get();
  Task* const task =
      core->CreateTaskOnCpu(BlockOnceWorker, &ctx, kDefaultTaskWeight, 1);
  ASSERT_THAT(task, t::NotNull());

  // CPU 1's idle loop (the calling thread stands in for it) runs the task
  // until it blocks, which switches back to the idle loop.
  BindCpuLocal(SmpGetCpuLocal(1));
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*core, 1));
  BindCpuLocal(SmpGetCpuLocal(0));
  EXPECT_THAT(ctx.phase.load(), t::Eq(1));
  EXPECT_THAT(SmpGetCpuLocal(1)->context_switches.load(), t::Eq(2));

  // Blocked: not running, not queued, not stealable, and CPU 1 is idle.
  EXPECT_THAT(TaskRunningCpu(task), t::Eq(-1));
  for (int c = 0; c < 3; ++c) {
    EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, c), t::Eq(0));
  }
  EXPECT_FALSE(TaskSchedulerInternals::IsCpuBusy(*core, 1));
  EXPECT_FALSE(TaskSchedulerInternals::StealTask(*core, 0, 1));
  EXPECT_FALSE(TaskSchedulerInternals::StealTask(*core, 2, 1));
  BindCpuLocal(SmpGetCpuLocal(2));
  EXPECT_FALSE(TaskSchedulerInternals::PollIdleCpu(*core, 2));
  BindCpuLocal(SmpGetCpuLocal(0));
  EXPECT_THAT(ctx.phase.load(), t::Eq(1));

  // The core's Wake() queues the task on the calling CPU, where it can be
  // stolen like any queued task.
  core->Wake(task);
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(1));
  EXPECT_TRUE(TaskSchedulerInternals::StealTask(*core, 2, 0));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(0));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 2), t::Eq(1));
  BindCpuLocal(SmpGetCpuLocal(2));
  EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*core, 2));
  BindCpuLocal(SmpGetCpuLocal(0));
  EXPECT_THAT(ctx.phase.load(), t::Eq(2));
  EXPECT_THAT(ctx.resumed_cpu.load(), t::Eq(2));

  core->Join(task);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

TEST(CoreRunqueueSchedulerTest,
     WakeFromAnotherCpuQueuesTheTaskThereAndItResumesThere) {
  SetupEnv(2);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  BlockOnceCtx ctx;
  ctx.sched = core.get();
  Task* const task =
      core->CreateTaskOnCpu(BlockOnceWorker, &ctx, kDefaultTaskWeight, 0);
  ASSERT_THAT(task, t::NotNull());
  core->Yield();
  ASSERT_THAT(ctx.phase.load(), t::Eq(1));

  // A thread standing in for CPU 1 wakes the task, which is queued on CPU 1,
  // and then runs it there.
  std::atomic<int> cpu1_load_after_wake{-1};
  std::atomic<bool> cpu1_dispatched{false};
  std::thread cpu1([&core, task, &cpu1_load_after_wake, &cpu1_dispatched]() {
    BindCpuLocal(SmpGetCpuLocal(1));
    SetInterruptsEnabledForTest(true);
    core->Wake(task);
    cpu1_load_after_wake.store(TaskSchedulerInternals::RunqueueLoad(*core, 1));
    cpu1_dispatched.store(TaskSchedulerInternals::PollIdleCpu(*core, 1));
    ResetCpuLocalForTest();
  });
  cpu1.join();

  EXPECT_THAT(cpu1_load_after_wake.load(), t::Eq(1));
  EXPECT_TRUE(cpu1_dispatched.load());
  EXPECT_THAT(ctx.phase.load(), t::Eq(2));
  EXPECT_THAT(ctx.resumed_cpu.load(), t::Eq(1));
  core->Join(task);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

struct HandoffWakeCtx {
  TaskScheduler* sched = nullptr;
  std::atomic<bool> armed{false};
  std::atomic<int> returns{0};
};

// Blocks twice with an FPU state attached, so that the fake FpuSave(), and the
// test's hook in it, run in the handoff window of each Block().
static void HandoffBlockWorker(void* const raw_ctx) {
  HandoffWakeCtx* const ctx = static_cast<HandoffWakeCtx*>(raw_ctx);
  ctx->sched->SetFpuState(FakeFpuState(7));
  ctx->armed.store(true, std::memory_order_release);
  for (int i = 0; i < 2; ++i) {
    ctx->sched->Block();
    ctx->returns.fetch_add(1, std::memory_order_acq_rel);
  }
  ctx->sched->SetFpuState(nullptr);
}

// A Wake() that comes while the CPU is still switching away from a blocking
// task (Schedule() has released the runqueue lock, but the CPU still runs on
// the task's stack) must neither queue the task there, where another CPU
// could resume it on a stack that is still in use, nor get lost. The fake
// FpuSave() runs in exactly that window, so its hook injects the Wake()s
// deterministically.
TEST(CoreRunqueueSchedulerTest,
     WakeInTheHandoffWindowIsNeitherLostNorQueuedTooEarly) {
  SetupEnv(2);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  HandoffWakeCtx ctx;
  ctx.sched = core.get();
  std::atomic<Task*> task_handle{nullptr};
  std::atomic<bool> hook_ran{false};
  std::atomic<bool> remote_wake_done{false};
  std::atomic<int> running_cpu_in_window{-2};
  std::atomic<int> queued_in_window{-1};
  g_env.fpu_save_hook = [&](const FpuState* const state) {
    if (state != FakeFpuState(7) || !ctx.armed.exchange(false)) {
      return;
    }
    Task* const task = task_handle.load();
    running_cpu_in_window.store(TaskRunningCpu(task));
    // A Wake() from another CPU, then one from this CPU (as an interrupt
    // handler would): both find the task not parked yet.
    std::thread waker([&core, task, &remote_wake_done]() {
      BindCpuLocal(SmpGetCpuLocal(1));
      SetInterruptsEnabledForTest(true);
      core->Wake(task);
      remote_wake_done.store(true);
      ResetCpuLocalForTest();
    });
    waker.join();
    core->Wake(task);
    queued_in_window.store(TaskSchedulerInternals::RunqueueLoad(*core, 0) +
                           TaskSchedulerInternals::RunqueueLoad(*core, 1));
    hook_ran.store(true);
  };

  Task* const task =
      core->CreateTaskOnCpu(HandoffBlockWorker, &ctx, kDefaultTaskWeight, 0);
  ASSERT_THAT(task, t::NotNull());
  task_handle.store(task);

  // The task runs and blocks; both Wake()s hit the handoff window.
  core->Yield();
  EXPECT_TRUE(hook_ran.load());
  EXPECT_TRUE(remote_wake_done.load());
  EXPECT_THAT(running_cpu_in_window.load(), t::Eq(0));
  EXPECT_THAT(queued_in_window.load(), t::Eq(0));
  // Once off the task's stack, CPU 0 found the wakeup and queued the task
  // there, once.
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(1));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 1), t::Eq(0));
  EXPECT_THAT(TaskRunningCpu(task), t::Eq(-1));
  EXPECT_THAT(ctx.returns.load(), t::Eq(0));

  // That wakeup ended the first Block(), so the second one blocks.
  EXPECT_TRUE(YieldUntil(*core, [&ctx]() { return ctx.returns.load() >= 1; }));
  for (int i = 0; i < 3; ++i) {
    core->Yield();
  }
  EXPECT_THAT(ctx.returns.load(), t::Eq(1));
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(0));
  EXPECT_THAT(TaskRunningCpu(task), t::Eq(-1));

  core->Wake(task);
  EXPECT_TRUE(YieldUntil(*core, [&ctx]() { return ctx.returns.load() == 2; }));
  core->Join(task);
  g_env.fpu_save_hook = nullptr;
  EXPECT_THAT(g_env.fpu_save_mismatches.load(), t::Eq(0));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

constexpr int kPingPongPairs = 2;

struct PingPongCtx {
  TaskScheduler* sched = nullptr;
  int rounds = 0;
  std::atomic<bool> abort{false};
  std::atomic<int> done{0};
  // Per pair: the side (0 or 1) whose turn it is.
  std::atomic<int> turn[kPingPongPairs] = {};
  std::atomic<Task*> tasks[kPingPongPairs][2] = {};
  std::atomic<int64_t> rounds_done[kPingPongPairs][2] = {};
  // Block() returns after which it still wasn't the task's turn.
  std::atomic<int64_t> spurious[kPingPongPairs][2] = {};
};

struct PingPongArg {
  PingPongCtx* ctx = nullptr;
  int pair = 0;
  int side = 0;
};

// Waits (blocked) for its turn, hands the turn to its partner and wakes it,
// `rounds` times.
static void PingPongWorker(void* const raw_arg) {
  const PingPongArg* const arg = static_cast<const PingPongArg*>(raw_arg);
  PingPongCtx* const ctx = arg->ctx;
  std::atomic<int>& turn = ctx->turn[arg->pair];
  for (int round = 0; round < ctx->rounds; ++round) {
    while (turn.load(std::memory_order_acquire) != arg->side) {
      if (ctx->abort.load(std::memory_order_acquire)) {
        ctx->done.fetch_add(1, std::memory_order_acq_rel);
        return;
      }
      ctx->sched->Block();
      if (turn.load(std::memory_order_acquire) != arg->side) {
        ctx->spurious[arg->pair][arg->side].fetch_add(1);
      }
    }
    ctx->rounds_done[arg->pair][arg->side].fetch_add(1);
    turn.store(1 - arg->side, std::memory_order_release);
    ctx->sched->Wake(ctx->tasks[arg->pair][1 - arg->side].load());
  }
  ctx->done.fetch_add(1, std::memory_order_acq_rel);
}

// Pairs of tasks hand a turn back and forth with Block() and Wake() on four
// simulated CPUs, which steal from each other, so wakeups race with blocking
// on other CPUs all the time. A lost wakeup stops a pair (the test then fails
// at its deadline), and a double enqueue fails a DCHECK.
TEST(CoreRunqueueSchedulerTest, ConcurrentPingPongAcrossCpusLosesNoWakeups) {
  constexpr int kNumCpus = 4;
  constexpr int kTasks = 2 * kPingPongPairs;
  SetupEnv(kNumCpus);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  PingPongCtx ctx;
  ctx.sched = core.get();
  ctx.rounds = 1000;
  PingPongArg args[kPingPongPairs][2];
  std::vector<Task*> tasks;
  // The tasks start only once this task yields, after all handles are
  // published.
  for (int pair = 0; pair < kPingPongPairs; ++pair) {
    for (int side = 0; side < 2; ++side) {
      args[pair][side] = PingPongArg{&ctx, pair, side};
      Task* const task = core->CreateTaskOnCpu(PingPongWorker,      //
                                               &args[pair][side],   //
                                               kDefaultTaskWeight,  //
                                               0);
      ASSERT_THAT(task, t::NotNull());
      ctx.tasks[pair][side].store(task);
      tasks.push_back(task);
    }
  }

  std::atomic<bool> stop_aps{false};
  std::vector<std::thread> ap_threads;
  for (int cpu = 1; cpu < kNumCpus; ++cpu) {
    ap_threads.emplace_back([cpu, &core, &stop_aps]() {
      BindCpuLocal(SmpGetCpuLocal(cpu));
      SetInterruptsEnabledForTest(true);
      while (!stop_aps.load(std::memory_order_acquire)) {
        if (TaskSchedulerInternals::RunqueueLoad(*core, cpu) == 0) {
          for (int donor = 0; donor < kNumCpus; ++donor) {
            if (donor != cpu &&
                TaskSchedulerInternals::RunqueueLoad(*core, donor) > 0 &&
                TaskSchedulerInternals::StealTask(*core, cpu, donor)) {
              break;
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

  const bool finished = WaitUntil(
      [&ctx]() { return ctx.done.load() == kTasks; },
      [&core]() {
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
      });
  EXPECT_TRUE(finished);
  if (!finished) {
    // Release every task, so that all of them exit and can be joined.
    ctx.abort.store(true, std::memory_order_release);
    EXPECT_TRUE(WaitUntil([&ctx]() { return ctx.done.load() == kTasks; },
                          [&core, &tasks]() {
                            for (Task* const task : tasks) {
                              core->Wake(task);
                            }
                            core->Yield();
                            std::this_thread::yield();
                          }));
  }
  for (Task* const task : tasks) {
    core->Join(task);
  }
  stop_aps.store(true, std::memory_order_release);
  for (std::thread& th : ap_threads) {
    th.join();
  }

  for (int pair = 0; pair < kPingPongPairs; ++pair) {
    for (int side = 0; side < 2; ++side) {
      EXPECT_THAT(ctx.rounds_done[pair][side].load(), t::Eq(ctx.rounds))
          << "pair " << pair << " side " << side;
      // A Block() that busy-waited instead would return far more often.
      EXPECT_THAT(ctx.spurious[pair][side].load(), t::Le(ctx.rounds));
    }
  }
  for (int c = 0; c < kNumCpus; ++c) {
    EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, c), t::Eq(0));
  }
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

struct TimerTicksProbeCtx {
  TaskScheduler* sched = nullptr;
  std::atomic<int64_t> own_ticks{-1};
  std::atomic<int64_t> clock_ticks{-1};
  std::atomic<bool> running_on_cpu1{false};
  std::atomic<bool> sampled{false};
};

// Takes three timer interrupts (counted as idt.cpp counts them), checks its
// own count, blocks, and after it is resumed (on CPU 1) takes four more and
// waits until the test has sampled its count.
static void TimerTicksProbeWorker(void* const raw_ctx) {
  TimerTicksProbeCtx* const ctx = static_cast<TimerTicksProbeCtx*>(raw_ctx);
  for (int i = 0; i < 3; ++i) {
    CurrentCpu()->task_timer_ticks.fetch_add(1, std::memory_order_relaxed);
  }
  ctx->own_ticks.store(TaskTimerTicks(ctx->sched->CurrentTask()));
  ctx->clock_ticks.store(ReadCpuClock().task_ticks);
  ctx->sched->Block();
  for (int i = 0; i < 4; ++i) {
    CurrentCpu()->task_timer_ticks.fetch_add(1, std::memory_order_relaxed);
  }
  ctx->running_on_cpu1.store(true, std::memory_order_release);
  (void)WaitUntil([ctx]() { return ctx->sampled.load(); },
                  []() { std::this_thread::yield(); });
}

TEST(CoreRunqueueSchedulerTest,
     TaskTimerTicksIsLiveForRunningTasksAndFrozenForOthers) {
  SetupEnv(2);
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);
  CpuLocal& cpu0 = *SmpGetCpuLocal(0);
  CpuLocal& cpu1 = *SmpGetCpuLocal(1);
  // 7 timer interrupts so far for this task, a million for CPU 1's idle task,
  // so that a count read from the wrong CPU or task stands out.
  cpu0.task_timer_ticks.store(7, std::memory_order_relaxed);
  cpu1.task_timer_ticks.store(1'000'000, std::memory_order_relaxed);
  Task* const bootstrap = core->CurrentTask();

  // A running task's count is live.
  EXPECT_THAT(TaskTimerTicks(bootstrap), t::Eq(7));
  cpu0.task_timer_ticks.fetch_add(2, std::memory_order_relaxed);
  EXPECT_THAT(TaskTimerTicks(bootstrap), t::Eq(9));

  TimerTicksProbeCtx ctx;
  ctx.sched = core.get();
  Task* const task =
      core->CreateTaskOnCpu(TimerTicksProbeWorker, &ctx, kDefaultTaskWeight, 0);
  ASSERT_THAT(task, t::NotNull());
  EXPECT_THAT(TaskTimerTicks(task), t::Eq(0));

  // The task takes 3 ticks and blocks: it saw them itself, as ReadCpuClock()
  // does, and they are saved with it.
  core->Yield();
  EXPECT_THAT(ctx.own_ticks.load(), t::Eq(3));
  EXPECT_THAT(ctx.clock_ticks.load(), t::Eq(3));
  EXPECT_THAT(TaskTimerTicks(task), t::Eq(3));
  EXPECT_THAT(TaskTimerTicks(bootstrap), t::Eq(9));

  // A blocked task accrues nothing; the running one does.
  cpu0.task_timer_ticks.fetch_add(5, std::memory_order_relaxed);
  EXPECT_THAT(TaskTimerTicks(task), t::Eq(3));
  EXPECT_THAT(TaskTimerTicks(bootstrap), t::Eq(14));

  // Nor does a queued one. Woken, the task moves to CPU 1, which runs it.
  core->Wake(task);
  EXPECT_THAT(TaskTimerTicks(task), t::Eq(3));
  ASSERT_TRUE(TaskSchedulerInternals::StealTask(*core, 1, 0));
  std::thread cpu1_thread([&core]() {
    BindCpuLocal(SmpGetCpuLocal(1));
    SetInterruptsEnabledForTest(true);
    EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*core, 1));
    ResetCpuLocalForTest();
  });
  EXPECT_TRUE(WaitUntil([&ctx]() { return ctx.running_on_cpu1.load(); },
                        []() { std::this_thread::yield(); }));
  // Read from CPU 0 while CPU 1 runs the task: CPU 1's live count, which
  // includes the 3 saved ticks, the 4 new ones and any that come in now.
  EXPECT_THAT(TaskTimerTicks(task), t::Eq(7));
  cpu1.task_timer_ticks.fetch_add(2, std::memory_order_relaxed);
  EXPECT_THAT(TaskTimerTicks(task), t::Eq(9));
  ctx.sampled.store(true, std::memory_order_release);
  cpu1_thread.join();

  // Exited: its count is saved, and CPU 1's idle task has its own back.
  EXPECT_THAT(TaskTimerTicks(task), t::Eq(9));
  EXPECT_THAT(cpu1.task_timer_ticks.load(), t::Eq(1'000'000));
  EXPECT_THAT(TaskTimerTicks(bootstrap), t::Eq(14));
  core->Join(task);
}

constexpr int kTicksMigrationRounds = 300;

struct TicksMigrationCtx {
  TaskScheduler* sched = nullptr;
  // How many rounds the task has done; the next wakeup comes from CPU
  // `rounds % kNumCpus`.
  std::atomic<int> rounds{0};
  std::atomic<uint32_t> cpu_mask{0};
  std::atomic<bool> done{false};
};

// Takes one timer interrupt on whichever CPU runs it, then blocks, until it
// has taken kTicksMigrationRounds of them.
static void TicksMigrationWorker(void* const raw_ctx) {
  TicksMigrationCtx* const ctx = static_cast<TicksMigrationCtx*>(raw_ctx);
  for (int i = 0; i < kTicksMigrationRounds; ++i) {
    CpuLocal* const cpu = CurrentCpu();
    cpu->task_timer_ticks.fetch_add(1, std::memory_order_relaxed);
    ctx->cpu_mask.fetch_or(1u << static_cast<uint32_t>(cpu->cpu_id));
    ctx->rounds.store(i + 1, std::memory_order_release);
    ctx->sched->Block();
  }
  ctx->done.store(true, std::memory_order_release);
}

// The CPUs take turns waking the task (the core queues a woken task on the
// calling CPU), so that it runs, and takes its ticks, on one CPU after
// another, while CPU 0 samples TaskTimerTicks(). The samples never decrease
// and never show another task's count. (A read of the saved count without
// synchronization would also be a data race that TSan reports.)
TEST(CoreRunqueueSchedulerTest, TaskTimerTicksIsMonotonicWhileTheTaskMigrates) {
  constexpr int kNumCpus = 4;
  SetupEnv(kNumCpus);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);
  Task* const bootstrap = core->CurrentTask();
  SmpGetCpuLocal(0)->task_timer_ticks.store(2'000'000);
  for (int cpu = 1; cpu < kNumCpus; ++cpu) {
    SmpGetCpuLocal(cpu)->task_timer_ticks.store(1'000'000);
  }

  TicksMigrationCtx ctx;
  ctx.sched = core.get();
  Task* const task =
      core->CreateTaskOnCpu(TicksMigrationWorker, &ctx, kDefaultTaskWeight, 0);
  ASSERT_THAT(task, t::NotNull());

  std::atomic<bool> stop_aps{false};
  std::vector<std::thread> ap_threads;
  for (int cpu = 1; cpu < kNumCpus; ++cpu) {
    ap_threads.emplace_back([cpu, &core, task, &ctx, &stop_aps]() {
      BindCpuLocal(SmpGetCpuLocal(cpu));
      SetInterruptsEnabledForTest(true);
      while (!stop_aps.load(std::memory_order_acquire)) {
        if (ctx.rounds.load(std::memory_order_acquire) % kNumCpus == cpu &&
            TaskRunningCpu(task) == -1) {
          core->Wake(task);
        }
        if (!TaskSchedulerInternals::PollIdleCpu(*core, cpu)) {
          std::this_thread::yield();
        }
      }
      ResetCpuLocalForTest();
    });
  }

  int64_t last = 0;
  int64_t samples = 0;
  int64_t violations = 0;
  const bool finished = WaitUntil(
      [&ctx]() { return ctx.done.load(std::memory_order_acquire); },
      [&]() {
        const int64_t ticks = TaskTimerTicks(task);
        if (ticks < last || ticks > kTicksMigrationRounds) {
          ++violations;
        }
        last = (ticks > last) ? ticks : last;
        ++samples;
        if (ctx.rounds.load(std::memory_order_acquire) % kNumCpus == 0 &&
            TaskRunningCpu(task) == -1) {
          core->Wake(task);
        }
        core->Yield();
        std::this_thread::yield();
      });
  EXPECT_TRUE(finished);
  EXPECT_THAT(TaskTimerTicks(task), t::Eq(kTicksMigrationRounds));
  // Having set `done`, the task never blocks again: it finishes exiting on
  // the CPU that runs it before that CPU's thread can stop. Join() reclaims it
  // once no AP can call Wake() on it anymore.
  stop_aps.store(true, std::memory_order_release);
  for (std::thread& th : ap_threads) {
    th.join();
  }
  core->Join(task);

  EXPECT_THAT(samples, t::Gt(0));
  EXPECT_THAT(violations, t::Eq(0));
  EXPECT_THAT(__builtin_popcount(ctx.cpu_mask.load()), t::Ge(2));
  // Every other task got its own count back.
  EXPECT_THAT(TaskTimerTicks(bootstrap), t::Eq(2'000'000));
  for (int cpu = 1; cpu < kNumCpus; ++cpu) {
    EXPECT_THAT(SmpGetCpuLocal(cpu)->task_timer_ticks.load(), t::Eq(1'000'000));
  }
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

struct RebaseCtx {
  TaskScheduler* sched = nullptr;
  int busy_yields = 0;
  std::atomic<bool> about_to_block{false};
  std::atomic<bool> peer_started{false};
  std::atomic<bool> go{false};
  std::atomic<int64_t> peer_yields{0};
  std::atomic<int64_t> peer_yields_at_resume{-1};
  std::atomic<bool> sleeper_done{false};
  std::atomic<bool> stop_peer{false};
  std::atomic<bool> peer_done{false};
};

// Takes turns with the bootstrap task on CPU 0 for `busy_yields` yields, each
// of which raises CPU 0's min_vruntime, then blocks, and records how often
// the peer had yielded when the sleeper resumed (on the peer's CPU).
static void RebaseSleeperWorker(void* const raw_ctx) {
  RebaseCtx* const ctx = static_cast<RebaseCtx*>(raw_ctx);
  for (int i = 0; i < ctx->busy_yields; ++i) {
    ctx->sched->Yield();
  }
  ctx->about_to_block.store(true, std::memory_order_release);
  ctx->sched->Block();
  ctx->peer_yields_at_resume.store(ctx->peer_yields.load());
  ctx->sleeper_done.store(true, std::memory_order_release);
}

// Runs on CPU 1 without using CPU time (in vruntime) until released, then
// yields until told to stop.
static void RebasePeerWorker(void* const raw_ctx) {
  RebaseCtx* const ctx = static_cast<RebaseCtx*>(raw_ctx);
  ctx->peer_started.store(true, std::memory_order_release);
  (void)WaitUntil([ctx]() { return ctx->go.load(); },
                  []() { std::this_thread::yield(); });
  (void)WaitUntil([ctx]() { return ctx->stop_peer.load(); },
                  [ctx]() {
                    ctx->peer_yields.fetch_add(1);
                    ctx->sched->Yield();
                  });
  ctx->peer_done.store(true, std::memory_order_release);
}

// A task that blocked on a CPU whose min_vruntime is far ahead (here CPU 0,
// after many yields) and is woken on another CPU keeps only its lead over the
// old CPU's min_vruntime: it runs at once, instead of waiting behind the
// tasks there until they have caught up (about `busy_yields` yields each).
TEST(CoreRunqueueSchedulerTest,
     WokenTaskIsRebasedOnTheMinVruntimeOfTheCpuThatQueuesIt) {
  SetupEnv(2);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  RebaseCtx ctx;
  ctx.sched = core.get();
  ctx.busy_yields = 200;
  Task* const sleeper =
      core->CreateTaskOnCpu(RebaseSleeperWorker, &ctx, kDefaultTaskWeight, 0);
  ASSERT_THAT(sleeper, t::NotNull());
  ASSERT_TRUE(YieldUntil(*core, [&ctx, sleeper]() {
    return ctx.about_to_block.load() && TaskRunningCpu(sleeper) == -1;
  }));

  Task* const peer =
      core->CreateTaskOnCpu(RebasePeerWorker, &ctx, kDefaultTaskWeight, 1);
  ASSERT_THAT(peer, t::NotNull());
  std::thread cpu1([&core, &ctx]() {
    BindCpuLocal(SmpGetCpuLocal(1));
    SetInterruptsEnabledForTest(true);
    (void)WaitUntil([&ctx]() { return ctx.peer_done.load(); },
                    [&core]() {
                      if (!TaskSchedulerInternals::PollIdleCpu(*core, 1)) {
                        std::this_thread::yield();
                      }
                    });
    ResetCpuLocalForTest();
  });
  EXPECT_TRUE(WaitUntil([&ctx]() { return ctx.peer_started.load(); },
                        []() { std::this_thread::yield(); }));

  EXPECT_TRUE(TaskSchedulerInternals::WakeOnCpu(*core, sleeper, 1));
  ctx.go.store(true, std::memory_order_release);
  EXPECT_TRUE(WaitUntil([&ctx]() { return ctx.sleeper_done.load(); },
                        []() { std::this_thread::yield(); }));
  ctx.stop_peer.store(true, std::memory_order_release);
  cpu1.join();
  core->Join(sleeper);
  core->Join(peer);

  EXPECT_THAT(ctx.peer_yields_at_resume.load(), t::AllOf(t::Ge(0), t::Le(2)));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

struct BlockKeepsStateCtx {
  TaskScheduler* sched = nullptr;
  std::atomic<bool> about_to_block{false};
  std::atomic<int> resume_cpu{-1};
  std::atomic<const AddressSpace*> active_on_resume{nullptr};
  std::atomic<const FpuState*> loaded_on_resume{nullptr};
  std::atomic<int64_t> checksum{0};
};

// Fills a stack buffer, attaches an address space and an FPU state, blocks,
// and after it resumes (on CPU 1) checks all three.
static void BlockKeepsStateWorker(void* const raw_ctx) {
  BlockKeepsStateCtx* const ctx = static_cast<BlockKeepsStateCtx*>(raw_ctx);
  volatile uint8_t buffer[4096];
  for (int i = 0; i < 4096; ++i) {
    buffer[i] = static_cast<uint8_t>(i * 7);
  }
  ctx->sched->SetAddressSpace(FakeAddressSpace(3));
  ctx->sched->SetFpuState(FakeFpuState(3));
  ctx->about_to_block.store(true, std::memory_order_release);
  ctx->sched->Block();
  const int cpu = CurrentCpuId();
  ctx->resume_cpu.store(cpu);
  ctx->active_on_resume.store(ActiveAddressSpace(cpu));
  ctx->loaded_on_resume.store(LoadedFpuState(cpu));
  int64_t sum = 0;
  for (int i = 0; i < 4096; ++i) {
    sum += buffer[i];
  }
  ctx->checksum.store(sum);
  ctx->sched->SetFpuState(nullptr);
  ctx->sched->SetAddressSpace(nullptr);
}

TEST(CoreRunqueueSchedulerTest,
     BlockedTaskKeepsItsStackAddressSpaceAndFpuStateOnAnotherCpu) {
  SetupEnv(2);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  BlockKeepsStateCtx ctx;
  ctx.sched = core.get();
  Task* const task =
      core->CreateTaskOnCpu(BlockKeepsStateWorker, &ctx, kDefaultTaskWeight, 0);
  ASSERT_THAT(task, t::NotNull());
  ASSERT_TRUE(YieldUntil(*core, [&ctx, task]() {
    return ctx.about_to_block.load() && TaskRunningCpu(task) == -1;
  }));
  EXPECT_THAT(ActiveAddressSpace(0), t::IsNull());

  std::thread cpu1([&core, task]() {
    BindCpuLocal(SmpGetCpuLocal(1));
    SetInterruptsEnabledForTest(true);
    core->Wake(task);
    EXPECT_TRUE(TaskSchedulerInternals::PollIdleCpu(*core, 1));
    ResetCpuLocalForTest();
  });
  cpu1.join();
  core->Join(task);

  int64_t expected_sum = 0;
  for (int i = 0; i < 4096; ++i) {
    expected_sum += static_cast<uint8_t>(i * 7);
  }
  EXPECT_THAT(ctx.resume_cpu.load(), t::Eq(1));
  EXPECT_THAT(ctx.active_on_resume.load(), t::Eq(FakeAddressSpace(3)));
  EXPECT_THAT(ctx.loaded_on_resume.load(), t::Eq(FakeFpuState(3)));
  EXPECT_THAT(ctx.checksum.load(), t::Eq(expected_sum));
  // CPU 0 saved the registers when the task blocked there, and CPU 1 loaded
  // them before it resumed the task.
  const std::vector<FakeFpuCall> calls = FpuCalls();
  EXPECT_THAT(calls, t::Contains(FakeFpuCall{FakeFpuOp::kSave,  //
                                             0,                 //
                                             FakeFpuState(3)}));
  EXPECT_THAT(calls, t::Contains(FakeFpuCall{FakeFpuOp::kRestore,  //
                                             1,                    //
                                             FakeFpuState(3)}));
  EXPECT_THAT(g_env.fpu_save_mismatches.load(), t::Eq(0));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

TEST(CoreRunqueueSchedulerTest,
     WakeOfAQueuedOrExitedTaskOnlyLeavesAPendingWakeup) {
  SetupEnv(1);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  // Exited (its CPU has switched away from it), but not reclaimed yet.
  std::atomic<bool> exited_ran{false};
  Task* const exited =
      core->CreateTask(SetAtomicFlagWorker, &exited_ran, kDefaultTaskWeight);
  ASSERT_THAT(exited, t::NotNull());
  ASSERT_TRUE(YieldUntil(*core, [&exited_ran]() { return exited_ran.load(); }));
  core->Wake(exited);
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(0));
  EXPECT_THAT(TaskRunningCpu(exited), t::Eq(-1));
  core->Join(exited);

  // Queued and not started yet: still queued once, and it runs normally.
  std::atomic<bool> queued_ran{false};
  Task* const queued =
      core->CreateTask(SetAtomicFlagWorker, &queued_ran, kDefaultTaskWeight);
  ASSERT_THAT(queued, t::NotNull());
  core->Wake(queued);
  core->Wake(queued);
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(1));
  core->Join(queued);
  EXPECT_TRUE(queued_ran.load());
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

TEST(CoreRunqueueSchedulerTest,
     WakeWorksWithInterruptsDisabledAndUnderACallerSpinLock) {
  SetupEnv(1);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);

  BlockOnceCtx ctx;
  ctx.sched = core.get();
  Task* const task =
      core->CreateTask(BlockOnceWorker, &ctx, kDefaultTaskWeight);
  ASSERT_THAT(task, t::NotNull());
  core->Yield();
  ASSERT_THAT(ctx.phase.load(), t::Eq(1));

  // As in an interrupt handler that holds a lock of its own (lock order: the
  // caller's lock, then the runqueue lock).
  SetInterruptsEnabledForTest(false);
  {
    IrqSpinLock caller_lock;
    const IrqSpinLockGuard guard(caller_lock);
    core->Wake(task);
  }
  EXPECT_FALSE(AreInterruptsEnabled());
  SetInterruptsEnabledForTest(true);
  EXPECT_THAT(TaskSchedulerInternals::RunqueueLoad(*core, 0), t::Eq(1));

  EXPECT_TRUE(YieldUntil(*core, [&ctx]() { return ctx.phase.load() == 2; }));
  core->Join(task);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

static void BlockWithUserAccessWindowOpenWorker(void* const raw_sched) {
  SetUserAccessWindowOpenForTest(true);
  static_cast<TaskScheduler*>(raw_sched)->Block();
}

TEST(CoreRunqueueSchedulerDeathTest, BlockAndWakeMisuseFailsChecks) {
  SetupEnv(2);
  const std::shared_ptr<TaskScheduler> uninit = CreateCoreRunqueueScheduler();
  constexpr const char* kNotInitialized =
      "Check failed: initialized_\\.load\\(std::memory_order_acquire\\)";
  EXPECT_DEATH(uninit->Block(), kNotInitialized);
  EXPECT_DEATH(uninit->Wake(nullptr), kNotInitialized);

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  TaskSchedulerInternals::Init(*core);
  EXPECT_DEATH(core->Wake(nullptr), "Check failed: task != nullptr");

  // The bootstrap task and the idle tasks never block (and so are never woken
  // either).
  Task* const bootstrap = core->CurrentTask();
  EXPECT_DEATH(core->Block(), "Check failed: !self->is_bootstrap");
  EXPECT_DEATH(core->Wake(bootstrap), "Check failed: !task->is_bootstrap");
  BindCpuLocal(SmpGetCpuLocal(1));
  Task* const idle = core->CurrentTask();
  EXPECT_DEATH(core->Block(), "Check failed: !self->is_idle");
  BindCpuLocal(SmpGetCpuLocal(0));
  EXPECT_DEATH(core->Wake(idle), "Check failed: !task->is_idle");

  // Not while holding a lock, nor with interrupts disabled (as in an
  // interrupt handler), nor with the user-access window open.
  EXPECT_DEATH(
      {
        IrqSpinLock lock;
        const IrqSpinLockGuard guard(lock);
        core->Block();
      },
      "Check failed: !HoldsAnyIrqSpinLock\\(\\)");
  EXPECT_DEATH(
      {
        SetInterruptsEnabledForTest(false);
        core->Block();
      },
      "Check failed: AreInterruptsEnabled\\(\\)");
  EXPECT_DEATH(
      {
        Task* const task =
            core->CreateTask(BlockWithUserAccessWindowOpenWorker,  //
                             core.get(),                           //
                             kDefaultTaskWeight);
        core->Join(task);
      },
      "Check failed: !UserAccessWindowOpen\\(\\)");

  // WakeOnCpu() takes an online CPU.
  Task* const task = core->CreateTask(NoopTask, nullptr, kDefaultTaskWeight);
  ASSERT_THAT(task, t::NotNull());
  constexpr const char* kBadTargetCpu =
      "Check failed: target_cpu >= 0 && target_cpu < cpu_count_";
  EXPECT_DEATH(TaskSchedulerInternals::WakeOnCpu(*core, task, -1),
               kBadTargetCpu);
  EXPECT_DEATH(TaskSchedulerInternals::WakeOnCpu(*core, task, 2),
               kBadTargetCpu);
  g_env.cpu_locals[1].online = false;
  EXPECT_DEATH(TaskSchedulerInternals::WakeOnCpu(*core, task, 1),
               "Check failed: IsCpuOnline\\(target_cpu, cpu_count_\\)");
  g_env.cpu_locals[1].online = true;

  // Timed waits belong to the preemptive layer.
  EXPECT_EXIT(core->BlockUntil(1), t::ExitedWithCode(134),
              "^\\[FATAL\\] CoreRunqueueTaskScheduler::BlockUntil\\(\\) is not "
              "supported: timed waits are handled by the preemptive layer at "
              ".*core_runqueue_scheduler\\.cpp:[0-9]+\n$");

  // The death tests ran in child processes; the task is still queued here.
  core->Join(task);
}

}  // namespace
}  // namespace protos
