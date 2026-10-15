#ifndef PROTOS_TASK_H_
#define PROTOS_TASK_H_

#include <cstdint>

namespace protos {

class AddressSpace;
struct InterruptFrame;
struct Task;

// Default, minimum, and maximum task weights for proportional-share vruntime
// scheduling (matching Linux CFS NICE_0_LOAD = 1024).
constexpr int64_t kDefaultTaskWeight = 1024;
constexpr int64_t kMinTaskWeight = 1;
constexpr int64_t kMaxTaskWeight = 1024 * 1024;

// Base virtual-runtime quantum charged per timer tick or cooperative yield to
// a task with weight == kDefaultTaskWeight.
constexpr int64_t kBaseVruntimeDelta = 1024;

// Number of 4 KiB pages of each task's kernel stack (16 KiB). Each task stack
// sits directly above an unmapped guard page (task_stack.h).
constexpr int64_t kTaskStackFrames = 4;
constexpr int64_t kTaskStackSize = kTaskStackFrames * 4096;

// Entry function signature for kernel tasks.
using TaskFn = void (*)(void* arg);

// Task lifecycle
// ==============
//
//   CreateTask*() -> entry(arg) runs -> entry returns -> task exits
//                 -> Join(), or Detach() + ReapZombies() -> task reclaimed
//
// - CreateTask() and CreateTaskOnCpu() return an opaque `Task*` handle, or
//   nullptr if the join record, the task control block (TCB) or the stack
//   can't be allocated. At most `kTaskStackSlotCount` (task_stack.h) tasks
//   can have a stack at the same time.
// - A task exits when its entry function returns (there is no explicit exit
//   call). The next task that runs on that CPU frees the exited task's stack.
//   The TCB and the join record stay allocated until the task is reclaimed.
// - The creator owns the handle and must pass it to exactly one of:
//   * Join(), which waits for the task to exit and then reclaims it, or
//   * Detach(), which gives up the handle without waiting. A detached task is
//     reclaimed by the first ReapZombies() call after it has exited.
//   A task that is never joined or detached leaks its TCB and join record.
// - The handle is invalid once Join() returns or Detach() is called. Its TCB
//   may be reused for a new task with the same `Task*` value.

// Kernel task scheduler. `TaskInit()` creates the scheduler and
// `GetTaskScheduler()` returns it.
//
// Each online CPU runs its queued tasks in order of weighted virtual runtime
// (proportional-share scheduling; see `kDefaultTaskWeight`). A CPU switches
// tasks when the running task yields or exits and, while preemption is
// enabled, on timer interrupts and wakeup IPIs. CPUs that run out of queued
// tasks take tasks queued on busier CPUs.
//
// The methods form two sections: the public task API, which task code calls,
// and the private scheduler internals, which only task.cpp, the scheduler
// layers and their tests call, through `TaskSchedulerInternals`.
//
// `TaskInit()` composes the scheduler from layers, each of which implements
// this interface and has a single scheduling responsibility. Each layer but
// the innermost one wraps the next layer inward (see `ForwardingTaskScheduler`
// in forwarding_task_scheduler.h). The innermost layer
// (core_runqueue_scheduler.h) manages the per-CPU runqueues and switches
// contexts. The outermost layer, the join lifecycle layer
// (join_lifecycle_scheduler.h), implements the task lifecycle methods
// `Detach()` and `ReapZombies()` on top of `Join()`. The layers below it
// forward these two methods inward to the core, which PANICs in them, so below
// the join lifecycle layer a task is reclaimed only when it is joined, and
// each task created through an inner layer must be joined exactly once.
//
// The method comments describe the composed scheduler; a layer used on its
// own may implement only part of the contract (see each layer's header). A
// method that a layer can't implement because another layer is responsible
// for it calls PANIC() (check.h) with a message that names the method, the
// layer and the layer that handles it.
class TaskScheduler {
 public:
  virtual ~TaskScheduler() = default;

  // Task API
  // --------
  //
  // Task code calls these methods on the scheduler that `GetTaskScheduler()`
  // returns.

  // Creates a task that runs `entry(arg)` on its own 16-byte-aligned 16 KiB
  // kernel stack, directly above an unmapped guard page, with
  // proportional-share `weight` (`kMinTaskWeight <= weight <= kMaxTaskWeight`),
  // queues it on a CPU chosen by the scheduler (the least-loaded online CPU,
  // counting both its queued tasks and the task it is running) and returns its
  // handle, or nullptr if memory runs out. The caller must eventually Join() or
  // Detach() the handle (see "Task lifecycle" above). Must not be called while
  // holding an `IrqSpinLock`: allocating the stack may wait for the other CPUs
  // to flush their TLBs (task_stack.h).
  virtual Task* CreateTask(TaskFn entry, void* arg, int64_t weight) = 0;

  // Like CreateTask(), but queues the task on online CPU `target_cpu`
  // (`0 <= target_cpu < SmpCpuCount()`), waking `target_cpu` with an IPI if it
  // is not the calling CPU.
  virtual Task* CreateTaskOnCpu(TaskFn entry,    //
                                void* arg,       //
                                int64_t weight,  //
                                int target_cpu) = 0;

  // Voluntarily yields the calling CPU to the next eligible runnable task,
  // charging virtual runtime to the calling task inversely proportional to its
  // weight. Must not be called while holding an `IrqSpinLock`.
  virtual void Yield() = 0;

  // Waits until `task` has exited, then frees its TCB and join record.
  //
  // The wait is a busy-wait: the caller stays runnable and loops on Yield(),
  // so it keeps getting CPU time while it waits. Exit happens in two steps:
  // the entry function returns, then the task's CPU switches away from it and
  // frees its stack. Join() returns without yielding only if both steps are
  // done; a task whose entry function has just returned may still need a few
  // yields to finish switching away.
  //
  // `task` must be a handle from CreateTask*() that has been neither joined
  // nor detached, and only one task may join it. Must not be called on
  // `CurrentTask()`, from an idle task or an interrupt handler, or while
  // holding an `IrqSpinLock`.
  virtual void Join(Task* task) = 0;

  // Gives up the handle `task` (from CreateTask*(), neither joined nor
  // detached yet) without waiting for the task to exit. The first
  // ReapZombies() call after the task has exited reclaims it. The handle must
  // not be used again by the caller.
  virtual void Detach(Task* task) = 0;

  // Reclaims every detached task that has exited (frees its TCB and join
  // record; its stack was already freed when it exited) and returns the number
  // of tasks reclaimed. Tasks that were not detached are left alone: only
  // Join() reclaims them. ReapZombies() may wait, like Join(), for a detached
  // task whose entry function has just returned to finish switching away, so
  // it has the same calling restrictions as Join().
  virtual int ReapZombies() = 0;

  // Enables or disables scheduling on timer interrupts and wakeup IPIs.
  //
  // The flag is global (it applies to every CPU at once) and does not nest:
  // code that disables it temporarily must save and restore the previous
  // value. While it is disabled, the scheduler ignores timer interrupts and
  // wakeup IPIs entirely: besides not preempting, ticks charge no virtual
  // runtime and idle CPUs don't steal work on ticks or IPIs, so a CPU-bound
  // task that runs in that window is favored afterwards. This differs from
  // Linux's per-CPU, nesting `preempt_disable()`.
  virtual void SetPreemptEnabled(bool enabled) = 0;

  // Returns `true` if interrupt-driven scheduling is currently enabled.
  virtual bool IsPreemptEnabled() const = 0;

  // Returns the opaque `Task*` currently executing on the calling CPU in O(1).
  virtual Task* CurrentTask() const = 0;

  // Runs the calling task in `address_space` (address_space.h) from now on:
  // switches the calling CPU to its page tables at once, and whichever CPU the
  // task runs on later switches to them whenever it resumes the task. nullptr
  // returns the task to the kernel's page tables, in which every task starts.
  // The kernel is mapped the same in every address space, so the task's kernel
  // code, data and stack stay where they are. The address space must outlive
  // its use by the task, which ends when the task switches to another address
  // space or exits. Must not be called from an idle task or an interrupt
  // handler, or while holding an `IrqSpinLock`.
  virtual void SetAddressSpace(AddressSpace* address_space) = 0;

 private:
  // Scheduler internals
  // -------------------
  //
  // These methods are private: only the friend class `TaskSchedulerInternals`
  // (task_scheduler_internals.h) can call them. task.cpp (`TaskInit()`, the
  // timer and wakeup IPI handlers and `TaskPollIdleCpu()`), the scheduler
  // layers and the scheduler tests call them through it; task code must not
  // call them. The layers still implement them by overriding them: access
  // levels don't restrict overriding.
  friend class TaskSchedulerInternals;

  // Initializes the scheduler layer and any wrapped inner layers. Must be
  // called once, on the BSP (CPU 0), after SMP bring-up and before any other
  // method.
  virtual void Init() = 0;

  // Handles a Local APIC timer interrupt (`kVectorApicTimer = 0x20`) on the
  // calling CPU. `frame` is the interrupted context. Acts only if preemption
  // is enabled, the interrupted context had interrupts enabled (`frame` is
  // non-null and has RFLAGS.IF set) and holds no `IrqSpinLock`: then charges a
  // tick of virtual runtime to the interrupted task and switches to a runnable
  // task with lower `vruntime`, if any.
  virtual void OnTimerInterrupt(InterruptFrame* frame) = 0;

  // Handles a cross-CPU wakeup/reschedule IPI (`kVectorWakeupIpi = 0x21`) on
  // the calling CPU. Under the same conditions as OnTimerInterrupt(), switches
  // to an eligible runnable task (or wakes an idle CPU), without charging
  // virtual runtime.
  virtual void OnWakeupIpi(InterruptFrame* frame) = 0;

  // Polls and dispatches runnable (or stolen) work on idle CPU `cpu_index`.
  // Returns `true` if a runnable task was switched in, or `false` if no work
  // was available.
  virtual bool PollIdleCpu(int cpu_index) = 0;

  // Returns the number of tasks queued on CPU `cpu_index`
  // (`0 <= cpu_index < SmpCpuCount()`) and waiting to run, not counting the
  // running task, the idle task, or the bootstrap task. Returns 0 before
  // `Init()`.
  virtual int RunqueueLoad(int cpu_index) const = 0;

  // Returns `true` if CPU `cpu_index` (`0 <= cpu_index < SmpCpuCount()`) is
  // running a task other than its idle task, which tells a busy CPU with
  // nothing queued from an idle one. CPU 0 has no idle task (it always runs
  // the bootstrap task or another task), so it is always busy. Returns `false`
  // before `Init()`.
  virtual bool IsCpuBusy(int cpu_index) const = 0;

  // Tries to move one runnable task (never an idle or bootstrap task) from
  // `src_cpu`'s runqueue to `dst_cpu`'s: the one with the largest virtual
  // runtime, which would wait longest on `src_cpu`. The stolen task's virtual
  // runtime is normalized relative to `dst_cpu`'s min_vruntime: it keeps the
  // lead it had over `src_cpu`'s min_vruntime, and a task that was behind
  // starts at `dst_cpu`'s min_vruntime. Returns `true` if `dst_cpu` already
  // had runnable work or a task was moved.
  virtual bool StealTask(int dst_cpu, int src_cpu) = 0;
};

// Composes the scheduler layers (join lifecycle -> preemptive -> work stealing
// -> core runqueues), sets up the task stack area (`TaskStackInit()`,
// task_stack.h), initializes the layers and registers the timer and wakeup IPI
// handlers. Must be called exactly once, on the BSP, after IdtInit(),
// PmmInit(), HeapInit() and SmpInit(). CHECK-fails if the IDT is not
// initialized or a scheduler already exists.
void TaskInit();

// Polls and dispatches runnable (or stolen) work on idle CPU `cpu_index`. The
// idle loop of each AP (`ApKernelEntry()`) calls it on that CPU each time it
// wakes up. Returns `true` if a runnable task was switched in, or `false` if
// no work was available or `TaskInit()` has not run yet.
bool TaskPollIdleCpu(int cpu_index);

// Returns the scheduler created by `TaskInit()`, or `nullptr` before
// `TaskInit()`. Safe to call from any CPU, including from interrupt handlers
// and idle loops. The kernel never destroys the scheduler.
TaskScheduler* GetTaskScheduler();

#if __STDC_HOSTED__
// Host unit-test hook: unregisters the interrupt handlers and destroys the
// scheduler, so that the next test can call `TaskInit()` again. The caller
// must make sure that no other thread uses the scheduler anymore. Tasks that
// have not been reclaimed are leaked.
void TaskResetForTest();
#endif

}  // namespace protos

#endif  // PROTOS_TASK_H_
