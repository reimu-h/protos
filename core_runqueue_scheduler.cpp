#include "core_runqueue_scheduler.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#if __STDC_HOSTED__
#include <pthread.h>

#include <thread>
#if defined(__SANITIZE_ADDRESS__)
#include <sanitizer/asan_interface.h>
#include <sanitizer/common_interface_defs.h>
#endif
#if defined(__SANITIZE_THREAD__)
#include <sanitizer/tsan_interface.h>
#endif
#endif

#include "check.h"
#include "heap.h"
#include "idt.h"
#include "pmm.h"
#include "rbtree.h"
#include "smp.h"
#include "spinlock.h"

namespace protos {
struct Task;
extern "C" void TaskStartupEntry(Task* curr);
}  // namespace protos

// The context switch (context_switch.S, which the host tests link as well). In
// sanitizer builds, Schedule() wraps each switch in fiber-switch annotations.
extern "C" {
void TaskContextSwitch(uintptr_t* prev_rsp, uintptr_t next_rsp);
void TaskEntryTrampoline();
}

namespace protos {

namespace {

enum class TaskState : uint8_t {
  kReady = 0,
  kRunning = 1,
  kExited = 2,
};

struct TaskRunqueueKey {
  int64_t vruntime = 0;
  int64_t enqueue_seq = 0;
  int64_t task_id = 0;

  constexpr bool operator<(const TaskRunqueueKey& other) const {
    if (vruntime != other.vruntime) {
      return vruntime < other.vruntime;
    }
    if (enqueue_seq != other.enqueue_seq) {
      return enqueue_seq < other.enqueue_seq;
    }
    return task_id < other.task_id;
  }
};

class CoreRunqueueTaskScheduler;

}  // namespace

// Task Control Block (TCB), hidden from `task.h` as an opaque type and defined
// privately inside `core_runqueue_scheduler.cpp`.
struct Task {
  CoreRunqueueTaskScheduler* owner = nullptr;
  int64_t id = 0;
  uintptr_t rsp = 0;
  uintptr_t stack_base = 0;
  uintptr_t stack_top = 0;
  int64_t stack_frames = 0;

  TaskFn entry = nullptr;
  void* arg = nullptr;

  int64_t weight = kDefaultTaskWeight;
  int64_t vruntime = 0;
  int64_t enqueue_seq = 0;
  RbNode runqueue_node;

  std::atomic<TaskState> state{TaskState::kReady};
  std::atomic<int> running_cpu{-1};
  bool is_idle = false;
  bool is_bootstrap = false;
  bool owns_stack = false;
  bool has_custom_stack = false;
  std::atomic<bool> stack_reaped{false};

#if __STDC_HOSTED__
#if defined(__SANITIZE_ADDRESS__)
  void* asan_fake_stack = nullptr;
  const void* asan_stack_bottom = nullptr;
  size_t asan_stack_size = 0;
#endif
#if defined(__SANITIZE_THREAD__)
  void* tsan_fiber = nullptr;
  bool owns_tsan_fiber = false;
#endif
#endif
};

namespace {

struct TaskRunqueueRbTraits {
  static TaskRunqueueKey GetKey(const Task& task) {
    return TaskRunqueueKey{task.vruntime, task.enqueue_seq, task.id};
  }

  static bool Less(const TaskRunqueueKey& a, const TaskRunqueueKey& b) {
    return a < b;
  }
};

using TaskRunqueueTree =
    RbTree<Task, &Task::runqueue_node, TaskRunqueueRbTraits>;

// Initial stack frame layout popped by `TaskContextSwitch` when dispatching a
// newly created task into `TaskEntryTrampoline`. `r12` holds `Task*` so
// `TaskEntryTrampoline` passes it to `TaskStartupEntry(Task* curr)` in `rdi`.
// The fields up to `rip` follow the pop order in context_switch.S, which pops
// six callee-saved registers and then returns.
struct InitialSwitchFrame {
  uint64_t r15;
  uint64_t r14;
  uint64_t r13;
  uint64_t r12;
  uint64_t rbx;
  uint64_t rbp;
  uint64_t rip;
  uint64_t pad_zero;
};
static_assert(sizeof(InitialSwitchFrame) == 64);
static_assert(offsetof(InitialSwitchFrame, r12) == 3 * sizeof(uint64_t));
static_assert(offsetof(InitialSwitchFrame, rip) == 6 * sizeof(uint64_t));

// Written at the lowest address (`stack_base`) of every stack the core
// allocates and CHECKed by `FinishContextSwitch()` each time the task is
// switched out and before the stack is freed, so that a task that ran off the
// end of its stack is caught at its next switch rather than wherever the
// corrupted memory is used next. This is best effort: there is no guard page,
// so an overflow that skips over this word goes unnoticed.
constexpr uint64_t kTaskStackCanary = 0x57ACCA9A57ACCA9AULL;

struct alignas(64) CpuRunqueue {
  IrqSpinLock lock;
  TaskRunqueueTree tree;
  int cpu_id = 0;
  int64_t min_vruntime = 0;
  int64_t next_enqueue_seq = 0;
  std::atomic<int> runnable_count{0};

  Task* current_task = nullptr;
  // The AP's adopted boot context. `nullptr` on CPU 0, whose boot context is
  // the bootstrap task (see `Init()`).
  Task* idle_task = nullptr;

  Task* pending_handoff_task = nullptr;
  TaskState pending_handoff_state = TaskState::kReady;
  std::atomic<bool> in_schedule{false};
};

// A task that is switched out can resume on another CPU, so per-CPU state is
// re-read after every `TaskContextSwitch()` through the out-of-line accessors
// `CurrentCpuIdIrqsOff()` and `CurrentCpuOrNullIrqsOff()` (smp.h). The
// scheduler functions that can switch tasks stay out of line too, so that the
// compiler cannot carry a CPU id, `CpuLocal` pointer, or TLS address computed
// before a switch across it.

static int64_t ComputeWeightedVruntimeDelta(const int64_t delta_exec,
                                            const int64_t weight) {
  DCHECK(delta_exec > 0);
  DCHECK(weight >= kMinTaskWeight && weight <= kMaxTaskWeight);
  const int64_t scaled = delta_exec * kBaseVruntimeDelta * kDefaultTaskWeight;
  const int64_t delta = scaled / weight;
  return (delta > 0) ? delta : 1;
}

// Records the sanitizer state of a task that runs on a stack the core did not
// allocate (the bootstrap task or an AP idle task) before it is switched out.
// Such a context is whatever was running when it was switched out, and in host
// tests any thread or task can stand in for a CPU's boot context, so the TSan
// fiber is re-read on every switch rather than cached once. ASan stack bounds
// are refreshed in `FinishContextSwitch()` from the stack that was just left.
static void RefreshNativeTaskSanitizerState(Task* const task) {
  DCHECK(task != nullptr);
#if __STDC_HOSTED__
  if (task->has_custom_stack) {
    return;
  }
#if defined(__SANITIZE_ADDRESS__)
  if (task->asan_stack_bottom == nullptr) {
    pthread_attr_t attr;
    if (pthread_getattr_np(pthread_self(), &attr) == 0) {
      void* saddr = nullptr;
      size_t ssize = 0;
      if (pthread_attr_getstack(&attr, &saddr, &ssize) == 0) {
        task->asan_stack_bottom = saddr;
        task->asan_stack_size = ssize;
      }
      pthread_attr_destroy(&attr);
    }
  }
#endif
#if defined(__SANITIZE_THREAD__)
  task->tsan_fiber = __tsan_get_current_fiber();
  task->owns_tsan_fiber = false;
#endif
#else
  (void)task;
#endif
}

static void DestroyTaskSanitizerState(Task* const task) {
  DCHECK(task != nullptr);
#if __STDC_HOSTED__ && defined(__SANITIZE_THREAD__)
  if (task->owns_tsan_fiber && task->tsan_fiber != nullptr) {
    __tsan_destroy_fiber(task->tsan_fiber);
    task->tsan_fiber = nullptr;
    task->owns_tsan_fiber = false;
  }
#else
  (void)task;
#endif
}

static void InitializeTaskStackFrame(Task* const task) {
  DCHECK(task != nullptr);
  DCHECK(task->stack_base != 0);
  DCHECK(task->stack_top > task->stack_base);
  DCHECK((task->stack_top & 0xFu) == 0);

  const uintptr_t frame_addr = task->stack_top - sizeof(InitialSwitchFrame);
  DCHECK((frame_addr & 0xFu) == 0);
  InitialSwitchFrame* const frame =
      reinterpret_cast<InitialSwitchFrame*>(frame_addr);
  frame->r15 = 0;
  frame->r14 = 0;
  frame->r13 = 0;
  frame->r12 = reinterpret_cast<uint64_t>(task);
  frame->rbx = 0;
  frame->rbp = 0;
  frame->rip = reinterpret_cast<uintptr_t>(&TaskEntryTrampoline);
  frame->pad_zero = 0;
  task->rsp = frame_addr;

#if __STDC_HOSTED__
#if defined(__SANITIZE_ADDRESS__)
  task->asan_fake_stack = nullptr;
  task->asan_stack_bottom = reinterpret_cast<const void*>(task->stack_base);
  task->asan_stack_size =
      static_cast<size_t>(task->stack_top - task->stack_base);
  __asan_unpoison_memory_region(task->asan_stack_bottom, task->asan_stack_size);
#endif
#if defined(__SANITIZE_THREAD__)
  task->tsan_fiber = __tsan_create_fiber(0);
  task->owns_tsan_fiber = true;
#endif
#endif
}

static void WriteTaskStackCanary(const Task* const task) {
  DCHECK(task->owns_stack && task->stack_base != 0);
  *reinterpret_cast<uint64_t*>(task->stack_base) = kTaskStackCanary;
}

// CHECK-fails if `task` overflowed the stack that the core allocated for it.
// Tasks on other stacks (bootstrap and idle tasks) are not checked.
static void CheckTaskStackCanary(const Task* const task) {
  if (!task->owns_stack) {
    return;
  }
  DCHECK(task->stack_base != 0);
  const bool task_stack_canary_intact =
      *reinterpret_cast<const uint64_t*>(task->stack_base) == kTaskStackCanary;
  CHECK(task_stack_canary_intact);
}

static void UpdateRunqueueMinVruntime(CpuRunqueue* const rq) {
  DCHECK(rq != nullptr);
  int64_t min_candidate = INT64_MAX;
  const Task* const curr = rq->current_task;
  if (curr != nullptr && !curr->is_idle &&
      curr->state.load(std::memory_order_relaxed) == TaskState::kRunning) {
    min_candidate = curr->vruntime;
  }
  const Task* const first_ready = rq->tree.First();
  if (first_ready != nullptr && first_ready->vruntime < min_candidate) {
    min_candidate = first_ready->vruntime;
  }
  if (min_candidate != INT64_MAX && min_candidate > rq->min_vruntime) {
    rq->min_vruntime = min_candidate;
  }
}

static void EnqueueTaskLocked(CpuRunqueue* const rq,  //
                              Task* const task,       //
                              const bool clamp_to_min_vruntime) {
  DCHECK(rq != nullptr);
  DCHECK(task != nullptr);
  DCHECK(!task->is_idle);

  if (clamp_to_min_vruntime && task->vruntime < rq->min_vruntime) {
    task->vruntime = rq->min_vruntime;
  }
  task->enqueue_seq = ++rq->next_enqueue_seq;
  task->running_cpu.store(-1, std::memory_order_release);
  task->state.store(TaskState::kReady, std::memory_order_release);

  const bool inserted = rq->tree.Insert(task);
  DCHECK(inserted);
  if (!task->is_bootstrap) {
    rq->runnable_count.fetch_add(1, std::memory_order_release);
  }
  UpdateRunqueueMinVruntime(rq);
}

class CoreRunqueueTaskScheduler final : public TaskScheduler {
 public:
  CoreRunqueueTaskScheduler() = default;

  ~CoreRunqueueTaskScheduler() override {
    if (idle_tasks_ != nullptr) {
      for (int i = 0; i < cpu_count_; ++i) {
        DestroyTaskSanitizerState(&idle_tasks_[i]);
      }
    }
    DestroyTaskSanitizerState(&bootstrap_task_);
  }

  Task* CreateTask(const TaskFn entry,  //
                   void* const arg,     //
                   const int64_t weight) override {
    return CreateTaskOnCpu(entry, arg, weight, 0);
  }

  Task* CreateTaskOnCpu(const TaskFn entry,    //
                        void* const arg,       //
                        const int64_t weight,  //
                        const int target_cpu) override {
    DCHECK(initialized_.load(std::memory_order_acquire));
    DCHECK(entry != nullptr);
    DCHECK(weight >= kMinTaskWeight && weight <= kMaxTaskWeight);
    DCHECK(target_cpu >= 0 && target_cpu < cpu_count_);
    DCHECK(IsCpuOnline(target_cpu, cpu_count_));

    void* const raw_tcb = Kmalloc(sizeof(Task));
    if (raw_tcb == nullptr) {
      return nullptr;
    }
    const uintptr_t stack_phys = PmmAllocFrames(kTaskStackFrames);
    if (stack_phys == 0) {
      Kfree(raw_tcb);
      return nullptr;
    }
    DCHECK((stack_phys & 0xFu) == 0);

    Task* const task = new (raw_tcb) Task();
    task->owner = this;
    task->id = next_task_id_.fetch_add(1, std::memory_order_relaxed);
    task->stack_base = stack_phys;
    task->stack_top = stack_phys + kTaskStackSize;
    task->stack_frames = kTaskStackFrames;
    task->entry = entry;
    task->arg = arg;
    task->weight = weight;
    task->vruntime = 0;
    task->enqueue_seq = 0;
    task->state.store(TaskState::kReady, std::memory_order_relaxed);
    task->running_cpu.store(-1, std::memory_order_relaxed);
    task->is_idle = false;
    task->is_bootstrap = false;
    task->owns_stack = true;
    task->has_custom_stack = true;
    task->stack_reaped.store(false, std::memory_order_relaxed);

    InitializeTaskStackFrame(task);
    WriteTaskStackCanary(task);

    CpuRunqueue& rq = runqueues_[target_cpu];
    {
      const IrqSpinLockGuard guard(rq.lock);
      task->vruntime = rq.min_vruntime;
      EnqueueTaskLocked(&rq, task, true);
    }
    return task;
  }

  [[gnu::noinline]] void Yield() override {
    DCHECK(initialized_.load(std::memory_order_acquire));
    DCHECK(!HoldsAnyIrqSpinLock());

    {
      const ScopedIrqDisable irq_guard;
      const int cpu_id = CurrentCpuIdIrqsOff();
      DCHECK(cpu_id >= 0 && cpu_id < cpu_count_);
      CpuRunqueue& rq = runqueues_[cpu_id];
      if (rq.in_schedule.load(std::memory_order_acquire)) {
        return;
      }
      const IrqSpinLockGuard guard(rq.lock);
      Task* const curr = rq.current_task;
      DCHECK(curr != nullptr);
      if (!curr->is_idle) {
        curr->vruntime += ComputeWeightedVruntimeDelta(1, curr->weight);
        UpdateRunqueueMinVruntime(&rq);
      }
    }
    (void)Schedule(TaskState::kReady);
  }

  void Join(Task* const task) override {
    DCHECK(initialized_.load(std::memory_order_acquire));
    DCHECK(task != nullptr);
    Task* const self = CurrentTask();
    DCHECK(self != nullptr);
    DCHECK(task != self);
    DCHECK(!task->is_idle);
    DCHECK(!task->is_bootstrap);
    DCHECK(!self->is_idle);
    DCHECK(!HoldsAnyIrqSpinLock());

    // Yield on this layer, not through the outer layers (which this layer
    // doesn't know). In the composed scheduler, the join lifecycle layer has
    // already waited for the task's entry function to return, so only the
    // short window until the task's CPU switches away from it is left.
    while (task->state.load(std::memory_order_acquire) != TaskState::kExited) {
      Yield();
#if __STDC_HOSTED__
      std::this_thread::yield();
#else
      asm volatile("pause" : : : "memory");
#endif
    }

    DCHECK(task->stack_reaped.load(std::memory_order_acquire));
    DestroyTaskSanitizerState(task);
    task->~Task();
    Kfree(task);
  }

  // The core keeps no join records, so it can't detach tasks or reap them.
  void Detach(Task* const /*task*/) override {
    PANIC(
        "CoreRunqueueTaskScheduler::Detach() is not supported: detaching is "
        "handled by the join lifecycle layer");
  }

  int ReapZombies() override {
    PANIC(
        "CoreRunqueueTaskScheduler::ReapZombies() is not supported: reaping "
        "zombies is handled by the join lifecycle layer");
  }

  void SetPreemptEnabled(const bool /*enabled*/) override {}
  bool IsPreemptEnabled() const override { return true; }

  [[gnu::noinline]] Task* CurrentTask() const override {
    DCHECK(initialized_.load(std::memory_order_acquire));
    const ScopedIrqDisable irq_guard;
    const int cpu_id = CurrentCpuIdIrqsOff();
    DCHECK(cpu_id >= 0 && cpu_id < cpu_count_);
    CpuRunqueue& rq = runqueues_[cpu_id];
    const IrqSpinLockGuard lock_guard(rq.lock);
    Task* const curr = rq.current_task;
    DCHECK(curr != nullptr);
    return curr;
  }

 private:
  friend void protos::TaskStartupEntry(Task* curr);

  // Scheduler internals:
  void Init() override {
    DCHECK(!initialized_.load(std::memory_order_acquire));
    DCHECK(IdtIsInitialized());
    const ScopedIrqDisable irq_guard;

    const int cpu_count = SmpCpuCount();
    CHECK(cpu_count >= 1);
    CHECK(SmpOnlineCpuCount() == cpu_count);
    cpu_count_ = cpu_count;

    runqueues_.reset(new CpuRunqueue[cpu_count]);
    CHECK(runqueues_ != nullptr);
    idle_tasks_.reset(new Task[cpu_count]);
    CHECK(idle_tasks_ != nullptr);

    // `SmpInit()` binds the BSP's `CpuLocal`, and `Init()` must run on the BSP.
    CHECK(CurrentCpuOrNullIrqsOff() != nullptr);
    CHECK(CurrentCpuIdIrqsOff() == 0);

    for (int i = 0; i < cpu_count; ++i) {
      runqueues_[i].cpu_id = i;
    }

    // Adopt the calling context (`kernel_main()` on the BSP's boot stack) as
    // the bootstrap task. `StealTask()` never migrates it, so it is always
    // either running on CPU 0 or queued there. CPU 0 therefore needs no idle
    // task: its runqueue is never empty when another task exits on it.
    // `idle_tasks_[0]` is unused.
    CpuLocal* const bsp_local = SmpGetCpuLocal(0);
    CHECK(bsp_local != nullptr);
    bootstrap_task_.owner = this;
    bootstrap_task_.id = 0;
    bootstrap_task_.stack_base = bsp_local->stack_base;
    bootstrap_task_.stack_top = bsp_local->stack_top;
    bootstrap_task_.stack_frames = kTaskStackFrames;
    bootstrap_task_.weight = kDefaultTaskWeight;
    bootstrap_task_.vruntime = 0;
    bootstrap_task_.enqueue_seq = 0;
    bootstrap_task_.state.store(TaskState::kRunning, std::memory_order_relaxed);
    bootstrap_task_.running_cpu.store(0, std::memory_order_relaxed);
    bootstrap_task_.is_idle = false;
    bootstrap_task_.is_bootstrap = true;
    bootstrap_task_.owns_stack = false;
    bootstrap_task_.has_custom_stack = false;
    RefreshNativeTaskSanitizerState(&bootstrap_task_);
    runqueues_[0].current_task = &bootstrap_task_;

    // Adopt each AP's boot context, the idle loop in `ApKernelEntry()` that is
    // already running on the AP's stack, as that AP's idle task.
    for (int i = 1; i < cpu_count; ++i) {
      CpuLocal* const cpu_local = SmpGetCpuLocal(i);
      CHECK(cpu_local != nullptr);
      CHECK(cpu_local->online);

      Task& idle = idle_tasks_[i];
      idle.owner = this;
      idle.id = i + 1;
      idle.stack_base = cpu_local->stack_base;
      idle.stack_top = cpu_local->stack_top;
      idle.stack_frames = kApStackFrames;
      idle.weight = kDefaultTaskWeight;
      idle.vruntime = 0;
      idle.enqueue_seq = 0;
      idle.state.store(TaskState::kRunning, std::memory_order_relaxed);
      idle.running_cpu.store(i, std::memory_order_relaxed);
      idle.is_idle = true;
      idle.is_bootstrap = false;
      idle.owns_stack = false;
      idle.has_custom_stack = false;

      runqueues_[i].idle_task = &idle;
      runqueues_[i].current_task = &idle;
    }

    next_task_id_.store(cpu_count + 1, std::memory_order_relaxed);
    initialized_.store(true, std::memory_order_release);
  }

  bool PollIdleCpu(const int cpu_index) override {
    if (!initialized_.load(std::memory_order_acquire) ||
        !IsCpuOnline(cpu_index, cpu_count_)) {
      return false;
    }
    {
      const ScopedIrqDisable irq_guard;
      const CpuLocal* const cpu = CurrentCpuOrNullIrqsOff();
      if (cpu == nullptr || !cpu->online || cpu->cpu_id != cpu_index) {
        return false;
      }
    }
    if (runqueues_[cpu_index].runnable_count.load(std::memory_order_acquire) ==
        0) {
      return false;
    }
    return Schedule(TaskState::kReady);
  }

  int RunqueueLoad(const int cpu_index) const override {
    // Before `Init()` nothing is queued. Returning 0 instead of failing keeps
    // outer layers that consult the load on interrupt and idle paths as safe to
    // call before `Init()` as this layer's `PollIdleCpu()`,
    // `OnTimerInterrupt()` and `OnWakeupIpi()` are.
    if (!initialized_.load(std::memory_order_acquire)) {
      return 0;
    }
    DCHECK(cpu_index >= 0 && cpu_index < cpu_count_);
    return runqueues_[cpu_index].runnable_count.load(std::memory_order_acquire);
  }

  bool StealTask(const int dst_cpu, const int src_cpu) override {
    DCHECK(initialized_.load(std::memory_order_acquire));
    DCHECK(dst_cpu >= 0 && dst_cpu < cpu_count_);
    DCHECK(src_cpu >= 0 && src_cpu < cpu_count_);
    if (dst_cpu == src_cpu || !IsCpuOnline(dst_cpu, cpu_count_) ||
        !IsCpuOnline(src_cpu, cpu_count_)) {
      return false;
    }

    const int first_cpu = (dst_cpu < src_cpu) ? dst_cpu : src_cpu;
    const int second_cpu = (dst_cpu < src_cpu) ? src_cpu : dst_cpu;
    const IrqSpinLockGuard first_guard(runqueues_[first_cpu].lock);
    const IrqSpinLockGuard second_guard(runqueues_[second_cpu].lock);

    CpuRunqueue& dst_rq = runqueues_[dst_cpu];
    CpuRunqueue& src_rq = runqueues_[src_cpu];

    if (dst_rq.runnable_count.load(std::memory_order_relaxed) > 0) {
      return true;
    }
    if (src_rq.runnable_count.load(std::memory_order_relaxed) == 0) {
      return false;
    }

    // The bootstrap task runs on the BSP's boot stack and must stay on CPU 0,
    // so skip it even when it is the leftmost task.
    Task* stolen = src_rq.tree.First();
    while (stolen != nullptr && (stolen->is_bootstrap || stolen->is_idle)) {
      stolen = TaskRunqueueTree::Next(stolen);
    }
    if (stolen == nullptr) {
      return false;
    }

    // Keep the task's lag behind the source CPU's `min_vruntime`, rebased on
    // the destination's, so that a task moving to a CPU with a much smaller
    // or larger `min_vruntime` neither starves nor is starved by the tasks
    // already there.
    const int64_t src_min_vruntime = src_rq.min_vruntime;
    const int64_t rel_vruntime = (stolen->vruntime > src_min_vruntime)
                                     ? (stolen->vruntime - src_min_vruntime)
                                     : 0;

    src_rq.tree.Erase(stolen);
    src_rq.runnable_count.fetch_sub(1, std::memory_order_release);
    UpdateRunqueueMinVruntime(&src_rq);

    stolen->vruntime = dst_rq.min_vruntime + rel_vruntime;
    EnqueueTaskLocked(&dst_rq, stolen, false);
    return true;
  }

  void OnTimerInterrupt(InterruptFrame* const /*frame*/) override {
    PreemptFromInterrupt(/*charge_tick=*/true);
  }

  void OnWakeupIpi(InterruptFrame* const /*frame*/) override {
    PreemptFromInterrupt(/*charge_tick=*/false);
  }

  // Shared body of `OnTimerInterrupt()` (`charge_tick`) and `OnWakeupIpi()`.
  // Optionally charges one tick of virtual runtime to the running task, then
  // preempts it if the leftmost queued task has a strictly smaller `vruntime`.
  // The idle task is never preempted here: the idle loop switches to queued
  // work itself by calling `PollIdleCpu()` after every wakeup, so work that the
  // idle loop runs with interrupts enabled (for example `SmpRunOnAllCpus()`
  // callbacks) is not suspended halfway. Does nothing before `Init()` or on an
  // unbound or offline CPU.
  void PreemptFromInterrupt(const bool charge_tick) {
    if (!initialized_.load(std::memory_order_acquire)) {
      return;
    }
    const ScopedIrqDisable irq_guard;
    const CpuLocal* const cpu = CurrentCpuOrNullIrqsOff();
    if (cpu == nullptr || !cpu->online) {
      return;
    }
    const int cpu_id = cpu->cpu_id;
    if (cpu_id < 0 || cpu_id >= cpu_count_) {
      return;
    }
    CpuRunqueue& rq = runqueues_[cpu_id];
    if (rq.in_schedule.load(std::memory_order_acquire) ||
        rq.lock.IsLockedByCurrentCpu()) {
      return;
    }

    bool should_preempt = false;
    {
      const IrqSpinLockGuard guard(rq.lock);
      Task* const curr = rq.current_task;
      if (!rq.in_schedule.load(std::memory_order_relaxed) && curr != nullptr &&
          !curr->is_idle &&
          curr->state.load(std::memory_order_relaxed) == TaskState::kRunning) {
        if (charge_tick) {
          curr->vruntime += ComputeWeightedVruntimeDelta(1, curr->weight);
          UpdateRunqueueMinVruntime(&rq);
        }
        const Task* const first = rq.tree.First();
        should_preempt = first != nullptr && first->vruntime < curr->vruntime;
      }
    }
    if (should_preempt) {
      (void)Schedule(TaskState::kReady);
    }
  }

  [[gnu::noinline]] void ExitCurrentTask() {
    DCHECK(initialized_.load(std::memory_order_acquire));
    Task* const self = CurrentTask();
    DCHECK(self != nullptr);
    DCHECK(!self->is_idle);
    DCHECK(!self->is_bootstrap);
    DCHECK(!HoldsAnyIrqSpinLock());

    (void)Schedule(TaskState::kExited);
    CHECK(false);
  }

  // Completes the second half of a context switch after `TaskContextSwitch`
  // has switched `%rsp` onto the incoming task's stack (`next`). Only once the
  // CPU is no longer executing on the outgoing task's stack (`prev`) is it
  // safe on SMP to either re-enqueue `prev` into `rq.tree` (where a remote CPU
  // could immediately steal and resume it) or free `prev`'s stack frames back
  // to the PMM on task exit. Both paths first check `prev`'s stack canary.
  [[gnu::noinline]] void FinishContextSwitch() {
    DCHECK(!AreInterruptsEnabled());
    DCHECK(!HoldsAnyIrqSpinLock());

    const int cpu_id = CurrentCpuIdIrqsOff();
    DCHECK(cpu_id >= 0 && cpu_id < cpu_count_);
    CpuRunqueue& rq = runqueues_[cpu_id];
    DCHECK(rq.in_schedule.load(std::memory_order_acquire));

#if __STDC_HOSTED__ && defined(__SANITIZE_ADDRESS__)
    Task* const curr = rq.current_task;
    const void* old_bottom = nullptr;
    size_t old_size = 0;
    __sanitizer_finish_switch_fiber(
        (curr != nullptr) ? curr->asan_fake_stack : nullptr,  //
        &old_bottom,                                          //
        &old_size);
    if (rq.pending_handoff_task != nullptr &&
        !rq.pending_handoff_task->has_custom_stack && old_bottom != nullptr &&
        old_size > 0) {
      rq.pending_handoff_task->asan_stack_bottom = old_bottom;
      rq.pending_handoff_task->asan_stack_size = old_size;
    }
#endif

    Task* prev = nullptr;
    TaskState target_state = TaskState::kReady;
    {
      const IrqSpinLockGuard guard(rq.lock);
      prev = rq.pending_handoff_task;
      target_state = rq.pending_handoff_state;
      rq.pending_handoff_task = nullptr;
      if (prev != nullptr && target_state == TaskState::kReady) {
        if (prev->is_idle) {
          prev->running_cpu.store(-1, std::memory_order_release);
          prev->state.store(TaskState::kReady, std::memory_order_release);
        } else {
          CheckTaskStackCanary(prev);
          EnqueueTaskLocked(&rq, prev, false);
        }
      }
      rq.in_schedule.store(false, std::memory_order_release);
    }

    if (prev == nullptr || target_state == TaskState::kReady) {
      return;
    }

    DCHECK(target_state == TaskState::kExited);
    prev->running_cpu.store(-1, std::memory_order_release);
    if (prev->owns_stack && prev->stack_base != 0) {
      CheckTaskStackCanary(prev);
      const uintptr_t base = prev->stack_base;
      const int64_t frames = prev->stack_frames;
      prev->stack_base = 0;
      prev->stack_top = 0;
      prev->stack_frames = 0;
#if __STDC_HOSTED__ && defined(__SANITIZE_ADDRESS__)
      __asan_unpoison_memory_region(reinterpret_cast<const void*>(base),
                                    static_cast<size_t>(frames * kPageSize));
#endif
      PmmFreeFrames(base, frames);
      prev->stack_reaped.store(true, std::memory_order_release);
    } else {
      prev->stack_reaped.store(true, std::memory_order_release);
    }
    prev->state.store(TaskState::kExited, std::memory_order_release);
  }

  // Selects the next runnable task on the calling CPU and context-switches to
  // it if a switch is warranted.
  //
  // Parameters:
  // - `outgoing_state`: Target post-switch state for the currently running task
  //   (`prev`):
  //   * `TaskState::kReady` when `prev` remains runnable (cooperative `Yield`,
  //     `PollIdleCpu`, or interrupt preemption). If `prev` is not the idle
  //     task, it switches only if `rq.tree.First()` has strictly smaller
  //     `vruntime` (`candidate->vruntime < prev->vruntime`).
  //   * `TaskState::kExited` when `prev` is terminating (`ExitCurrentTask()`),
  //     in which case `prev` must be switched out to `rq.tree.First()` (or
  //     `rq.idle_task` when `rq.tree` is empty, which never happens on CPU 0
  //     because the bootstrap task is queued there whenever another task
  //     runs).
  //
  // Returns `true` if a context switch occurred (after `prev` is eventually
  // scheduled back onto a CPU), or `false` if `prev` remained the active task
  // (or if `rq.in_schedule` guarded against nested re-entry).
  [[gnu::noinline]] bool Schedule(const TaskState outgoing_state) {
    // Keep local interrupts disabled across the entire `Schedule()` call—not
    // just while `rq.lock` is held—for three reasons:
    // 1. Before `rq.lock` is acquired: `CurrentCpuIdIrqsOff()` is read before
    //    `lock_guard` locks `rq.lock`. If an interrupt fired in between, the
    //    ISR could preempt this task and another CPU could steal it, leaving
    //    `cpu_id` and `rq` pointing to the old CPU's runqueue.
    // 2. Across the unlocked handoff window (`TaskContextSwitch` through
    //    `FinishContextSwitch()`): once `lock_guard` releases `rq.lock`,
    //    `rq.current_task` is already `next` while still on `prev`'s stack, and
    //    `FinishContextSwitch()` clears `rq.in_schedule = false` under
    //    `rq.lock` after re-enqueuing `prev`. An interrupt preempting and
    //    migrating `next` there would leave `FinishContextSwitch()` holding a
    //    stale `rq` reference mid-handoff.
    // 3. Preserving per-task `RFLAGS.IF` across cooperative (`IF=1`) <->
    //    ISR-preempted (`IF=0`) switches: if `lock_guard` restored `IF=1`
    //    before `TaskContextSwitch`, a task resuming inside `OnTimerInterrupt`
    //    would run with interrupts enabled before `iretq`; conversely, a task
    //    resuming into `Yield()` from an ISR-preempted task does not execute
    //    `iretq` and relies on `irq_guard` to restore its saved `RFLAGS.IF`.
    const ScopedIrqDisable irq_guard;

    Task* prev = nullptr;
    Task* next = nullptr;
    uintptr_t* prev_rsp_ptr = nullptr;
    uintptr_t next_rsp_val = 0;
    {
      const int cpu_id = CurrentCpuIdIrqsOff();
      DCHECK(cpu_id >= 0 && cpu_id < cpu_count_);
      CpuRunqueue& rq = runqueues_[cpu_id];
      const IrqSpinLockGuard lock_guard(rq.lock);
      if (rq.in_schedule.load(std::memory_order_relaxed)) {
        return false;
      }
      rq.in_schedule.store(true, std::memory_order_release);

      prev = rq.current_task;
      DCHECK(prev != nullptr);

      if (outgoing_state == TaskState::kReady) {
        if (prev->is_idle) {
          next = rq.tree.Empty() ? prev : rq.tree.First();
        } else {
          Task* const candidate = rq.tree.First();
          next = (candidate != nullptr && candidate->vruntime < prev->vruntime)
                     ? candidate
                     : prev;
        }
      } else {
        DCHECK(!prev->is_idle);
        next = rq.tree.Empty() ? rq.idle_task : rq.tree.First();
      }

      DCHECK(next != nullptr);
      if (next == prev) {
        // We are on the same task. No need to switch.
        DCHECK(outgoing_state == TaskState::kReady);
        UpdateRunqueueMinVruntime(&rq);
        rq.in_schedule.store(false, std::memory_order_release);
        return false;
      }
      // We need to do a context switch.

      if (!next->is_idle) {
        rq.tree.Erase(next);
        if (!next->is_bootstrap) {
          rq.runnable_count.fetch_sub(1, std::memory_order_release);
        }
      }
      next->state.store(TaskState::kRunning, std::memory_order_release);
      next->running_cpu.store(cpu_id, std::memory_order_release);
      rq.current_task = next;
      UpdateRunqueueMinVruntime(&rq);

      // SMP stack-safety caveat: Do NOT re-insert `prev` into `rq.tree` (or
      // free its stack on `kExited`) before `TaskContextSwitch` switches `%rsp`
      // off `prev`'s stack. Because `rq.lock` must be released before calling
      // `TaskContextSwitch`, re-enqueuing `prev` here would allow another CPU
      // to steal `prev` and begin executing on `prev`'s stack while this CPU is
      // still saving registers and switching away from it. Instead, stash
      // `prev` in `rq.pending_handoff_task` and finalize its state transition
      // in `FinishContextSwitch()` after landing on `next`'s stack.
      rq.pending_handoff_task = prev;
      rq.pending_handoff_state = outgoing_state;
      RefreshNativeTaskSanitizerState(prev);

      prev_rsp_ptr = &prev->rsp;
      next_rsp_val = next->rsp;
    }

#if __STDC_HOSTED__
#if defined(__SANITIZE_ADDRESS__)
    void** const fake_stack_save = (outgoing_state == TaskState::kExited)
                                       ? nullptr
                                       : &prev->asan_fake_stack;
    __sanitizer_start_switch_fiber(fake_stack_save,          //
                                   next->asan_stack_bottom,  //
                                   next->asan_stack_size);
#endif
#if defined(__SANITIZE_THREAD__)
    __tsan_switch_to_fiber(next->tsan_fiber, 0);
#endif
#endif

    // Note:
    // 1. No locks (including `rq.lock`) may be held across `TaskContextSwitch`,
    //    because it is uncertain when (or if, on `kExited`) `prev` will be
    //    scheduled again, and holding a lock while suspended would block or
    //    deadlock other CPUs and tasks.
    // 2. When `TaskContextSwitch` returns, `prev` may have been stolen and
    //    resumed on a different CPU (or, in host tests, a different thread).
    //    Any pre-switch per-CPU state (such as `cpu_id` or `rq`) is invalid
    //    after this point; the code below and `FinishContextSwitch()` re-query
    //    it through the out-of-line `CurrentCpuIdIrqsOff()` on the CPU that
    //    resumed `prev`. Do not cache per-CPU or `thread_local` addresses
    //    across the switch, and keep this function out of line.

    // Make sure no locks are held and interrupts are disabled.
    DCHECK(!AreInterruptsEnabled());
    DCHECK(!HoldsAnyIrqSpinLock());

    // Do the context switch.
    TaskContextSwitch(prev_rsp_ptr, next_rsp_val);

    // If the task keeps running here, it should not be exiting.
    DCHECK(outgoing_state != TaskState::kExited);

    // Check the preconditions again.
    DCHECK(!AreInterruptsEnabled());
    {
      DCHECK(!HoldsAnyIrqSpinLock());
      const int resumed_cpu_id = CurrentCpuIdIrqsOff();
      DCHECK(resumed_cpu_id >= 0 && resumed_cpu_id < cpu_count_);
      DCHECK(prev->running_cpu.load(std::memory_order_relaxed) ==
             resumed_cpu_id);
      DCHECK(runqueues_[resumed_cpu_id].current_task == prev);
    }

    FinishContextSwitch();
    return true;
  }

  std::unique_ptr<CpuRunqueue[]> runqueues_;
  Task bootstrap_task_;
  std::unique_ptr<Task[]> idle_tasks_;
  int cpu_count_ = 0;
  std::atomic<int64_t> next_task_id_{1};
  std::atomic<bool> initialized_{false};
};

}  // namespace

extern "C" void TaskStartupEntry(Task* const curr) {
  DCHECK(curr != nullptr);
  DCHECK(curr->owner != nullptr);
  curr->owner->FinishContextSwitch();
  RestoreInterrupts(true);

  DCHECK(curr->entry != nullptr);
  curr->entry(curr->arg);

  curr->owner->ExitCurrentTask();
  CHECK(false);
}

std::shared_ptr<TaskScheduler> CreateCoreRunqueueScheduler() {
  return std::make_shared<CoreRunqueueTaskScheduler>();
}

#if __STDC_HOSTED__
uintptr_t CoreTaskStackBaseForTest(const Task* const task) {
  DCHECK(task != nullptr);
  return task->stack_base;
}
#endif

}  // namespace protos
