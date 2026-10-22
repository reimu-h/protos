#ifndef PROTOS_TASK_H_
#define PROTOS_TASK_H_

#include <cstdint>

namespace protos {

class AddressSpace;
struct FpuState;
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
// - While it runs, a task may block (Block(), BlockUntil()): it is then
//   neither runnable nor exited, and uses no CPU time, until a Wake() (or its
//   BlockUntil() deadline) makes it runnable again.

// Kernel task scheduler. `TaskInit()` creates the scheduler and
// `GetTaskScheduler()` returns it.
//
// Each online CPU runs its queued tasks in order of weighted virtual runtime
// (proportional-share scheduling; see `kDefaultTaskWeight`). A CPU switches
// tasks when the running task yields, blocks or exits and, while preemption
// is enabled, on timer interrupts and wakeup IPIs. CPUs that run out of queued
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

  // Blocks the calling task until Wake() is called on it: the task leaves its
  // CPU and is not runnable, so it uses no CPU time, until then. Each task has
  // one pending-wakeup flag. A Wake() that finds the task not blocked sets it,
  // and Block() returns at once, clearing it, if it is set. So a Wake() that
  // arrives before Block(), however early, is not lost, several Wake() calls
  // before one Block() count as one, and a Wake() that ends a Block() leaves
  // no flag behind. Block() may also return without a Wake() (rarely), so
  // callers wait for their condition in a loop:
  //
  //   while (!condition()) {  // The waker makes it true, then calls Wake().
  //     scheduler->Block();
  //   }
  //
  // What the waker wrote before it called Wake() is visible to the task once
  // Block() returns. Like Yield(), the task may resume on another CPU.
  // CHECK-fails if called from the bootstrap task (CPU 0's boot context,
  // which must stay runnable) or an idle task, with interrupts disabled (so
  // also from an interrupt handler), or while holding an `IrqSpinLock`.
  virtual void Block() = 0;

  // Like Block(), but also returns once CPU 0's timer tick count (the
  // `CpuLocal::timer_ticks` of CPU 0, `kApicTimerTargetHz` ticks per second)
  // has reached `deadline_tick`, at once if it already has. CPU 0's timer
  // interrupt ends the wait with a Wake(), also while preemption is disabled
  // (SetPreemptEnabled()). If that Wake() comes after another Wake() has ended
  // the wait but before BlockUntil() returns, it sets the pending-wakeup flag.
  // BlockUntil() may also return before either happened, so callers loop:
  //
  //   while (!condition() && cpu0_ticks() < deadline_tick) {
  //     scheduler->BlockUntil(deadline_tick);
  //   }
  //
  // Same calling rules as Block().
  virtual void BlockUntil(int64_t deadline_tick) = 0;

  // Ends the Block() or BlockUntil() of `task` if the task is blocked: queues
  // it on a CPU chosen by the scheduler (the least-loaded online CPU, as for
  // CreateTask()) and wakes that CPU with an IPI if it is not the calling CPU.
  // Otherwise sets the task's pending-wakeup flag (see Block()). Never
  // switches tasks and never waits for another CPU, so it may be called on any
  // CPU, also from interrupt handlers (but not NMI handlers), with interrupts
  // disabled, and while preemption is disabled. A caller may hold
  // `IrqSpinLock`s that the scheduler never takes: Wake() takes exactly one
  // runqueue lock, so the lock order is the caller's locks, then the runqueue
  // lock. `task` is a handle from CreateTask*() that must not be reclaimed
  // (joined or reaped) before Wake() returns. Waking a task that has exited
  // does nothing.
  virtual void Wake(Task* task) = 0;

  // Waits until `task` has exited, then frees its TCB and join record.
  //
  // The wait is a busy-wait: the caller stays runnable and loops on Yield(),
  // so it keeps getting CPU time while it waits. Exit happens in two steps:
  // the entry function returns, then the task's CPU switches away from it and
  // frees its stack. Join() returns without yielding only if both steps are
  // done; a task whose entry function has just returned may still need a few
  // yields to finish switching away. A blocked task exits only after it has
  // been woken, so Join() waits for that as well.
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
  // Linux's per-CPU, nesting `preempt_disable()`. Block(), BlockUntil() (with
  // its deadline) and Wake() work regardless.
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

  // Keeps the calling task's x87/MMX/SSE/AVX registers in `state` (fpu.h)
  // from now on: loads the registers from `state` at once, saves them into it
  // whenever the task is switched out, and loads them from it whenever the
  // task resumes, on whichever CPU. nullptr stops this, dropping the current
  // register values. Tasks start without a state, and the scheduler never
  // saves or loads the registers for a task without one: kernel code doesn't
  // use them. `state` must stay allocated until the task stops using it, by
  // calling SetFpuState() again or by exiting (the registers of an exiting
  // task are not saved). Must not be called from an idle task or an interrupt
  // handler, or while holding an `IrqSpinLock`.
  virtual void SetFpuState(FpuState* state) = 0;

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
  // calling CPU. `frame` is the interrupted context. On CPU 0 it first ends
  // every BlockUntil() whose deadline has passed, whatever the conditions
  // below. Otherwise acts only if preemption is enabled, the interrupted
  // context had interrupts enabled (`frame` is non-null and has RFLAGS.IF
  // set) and holds no `IrqSpinLock`: then charges a tick of virtual runtime to
  // the interrupted task and switches to a runnable task with lower
  // `vruntime`, if any.
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

  // Wake() with the CPU chosen by the caller: if this call ends the Block() of
  // `task`, queues the task on online CPU `target_cpu`
  // (`0 <= target_cpu < SmpCpuCount()`), waking that CPU with an IPI if it is
  // not the calling CPU, and returns true. Otherwise sets the task's
  // pending-wakeup flag and returns false. The woken task keeps the lead over
  // `min_vruntime` that it had on the CPU where it blocked, rebased on
  // `target_cpu`'s, as StealTask() rebases a stolen task. Same calling rules
  // as Wake(); once it returns true, `task` may already have run and exited.
  virtual bool WakeOnCpu(Task* task, int target_cpu) = 0;
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

// Returns the CPU that is running `task` (a handle from CreateTask*() that has
// not been reclaimed yet), or -1 if no CPU is: the task is queued or blocked,
// has not started or has exited. A task counts as running from the moment a
// CPU picks it until that CPU has completely switched away from it, so -1
// means that the task executes on no CPU at that moment. The answer may be out
// of date as soon as it is returned. Safe on any CPU, with interrupts enabled
// or disabled. The core layer (core_runqueue_scheduler.h), which owns the task
// control blocks, implements it.
int TaskRunningCpu(const Task* task);

// Returns how many timer interrupts (`kVectorApicTimer`, `kApicTimerTargetHz`
// per CPU) have interrupted `task` so far, on whichever CPUs it ran: the CPU
// time it has used, in ticks, including its current run if a CPU is running
// it now (the count that ReadCpuClock() in cpu_clock.h reports to the task
// itself). The count doesn't change while the task is queued, blocked or
// exited, and it never decreases. `task` is a handle from CreateTask*() that
// has not been reclaimed yet, or `CurrentTask()` (also the bootstrap or an
// idle task). The answer may be out of date as soon as it is returned. Safe on
// any CPU, also from interrupt handlers (but not NMI handlers). It takes the
// runqueue lock of the CPU that runs the task, so a caller that holds an
// `IrqSpinLock` follows Wake()'s lock order. The core layer
// (core_runqueue_scheduler.h) implements it.
int64_t TaskTimerTicks(const Task* task);

#if __STDC_HOSTED__
// Host unit-test hook: unregisters the interrupt handlers and destroys the
// scheduler, so that the next test can call `TaskInit()` again. The caller
// must make sure that no other thread uses the scheduler anymore. Tasks that
// have not been reclaimed are leaked.
void TaskResetForTest();
#endif

}  // namespace protos

#endif  // PROTOS_TASK_H_
