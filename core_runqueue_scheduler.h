#ifndef PROTOS_CORE_RUNQUEUE_SCHEDULER_H_
#define PROTOS_CORE_RUNQUEUE_SCHEDULER_H_

#include <cstdint>
#include <memory>

#include "task.h"

namespace protos {

// Creates the innermost scheduler layer: one red-black tree runqueue per CPU,
// ordered by weighted virtual runtime (`vruntime`), plus the low-level context
// switch. It must be the innermost layer, because outer layers forward to it
// and it never calls into an inner scheduler: it implements every
// `TaskScheduler` method itself. On its own it implements only part of the
// contract described in task.h and departs from it as follows; outer layers
// (see `TaskInit()`) supply the rest:
//
// - `CreateTask()` always places the task on CPU 0, and `CreateTaskOnCpu()`
//   never sends a wakeup IPI. Placement and wakeups belong to the
//   work-stealing layer.
// - `SetPreemptEnabled()` does nothing and `IsPreemptEnabled()` always returns
//   true. The preemptive layer decides whether an interrupt may preempt.
// - `Join()` busy-waits, yielding until the task has exited, and then frees
//   its TCB; there are no join records. The core frees a task's stack when the
//   task exits, but its TCB only in `Join()`, and does not keep track of tasks
//   that nobody joins (the join-lifecycle layer does).
// - `Detach()` and `ReapZombies()` are not supported: without join records
//   the core can't reclaim a task that nobody joins, so both crash with a
//   PANIC() message that names the join lifecycle layer, which implements
//   them.
// - `RunqueueLoad()` counts only the queued tasks that `StealTask()` may
//   migrate: the running task, the idle tasks and the bootstrap task are
//   excluded, so a CPU running one task with nothing queued reports 0. It
//   returns 0 before `Init()`.
// - `IsCpuBusy()` reports whether a CPU's current task is not its idle task.
//   CPU 0, whose current task is always the bootstrap task or another task,
//   is therefore always busy. It returns false before `Init()`.
// - `StealTask()` moves the stealable task of `src_cpu` with the largest
//   `vruntime` (the rightmost one in its runqueue) and keeps its `vruntime`
//   offset from `src_cpu`'s `min_vruntime`, rebased on `dst_cpu`'s.
//
// Boot contexts: `Init()` adopts the calling context (`kernel_main()` on the
// BSP) as the bootstrap task and each AP's boot context (the idle loop in
// `ApKernelEntry()`) as that AP's idle task. The bootstrap task is never
// stolen, so it is always running on CPU 0 or queued there, and CPU 0 has no
// idle task. An idle task runs only while its CPU has nothing queued. It is
// not preempted by interrupts: the idle loop switches to queued work by calling
// `PollIdleCpu()` each time it wakes up, and is resumed only when a task exits
// while nothing else is queued on that CPU. Work that the idle loop performs
// itself, such as `SmpRunOnAllCpus()` callbacks, therefore waits while tasks
// run on that AP.
//
// Interrupts and threading: methods may be called on any CPU and disable local
// interrupts internally where they touch per-CPU state. `OnTimerInterrupt()`,
// `OnWakeupIpi()` and `PollIdleCpu()` do nothing before `Init()` or on an
// unbound or offline CPU, and `PollIdleCpu(cpu_index)` does nothing unless
// called on CPU `cpu_index`. `CreateTask*()`, `Yield()`, `Join()` and
// returning from a task must not happen while an `IrqSpinLock` is held.
// `Init()` must run on the BSP after `SmpInit()` (which binds the BSP's
// `CpuLocal`) and `IdtInit()`.
//
// The core allocates task stacks with `TaskStackAlloc()` (task_stack.h), which
// puts an unmapped guard page below each stack, and frees a stack with
// `TaskStackFree()` when its task exits. Each stack also has a canary at its
// lowest address. The core CHECK-fails if a task overwrote it (overflowed its
// stack) when the task is switched out or exits.
//
// Address spaces: the core records the address space of each task
// (`SetAddressSpace()`) and calls `ActivateAddressSpace()` (address_space.h)
// when a CPU switches to a task whose address space differs from the one the
// CPU uses. Every task starts in the kernel's page tables, and idle tasks
// always run in them.
//
// x87/MMX/SSE/AVX registers: the core records the register state of each task
// (`SetFpuState()`), saves the registers into it with `FpuSave()` (fpu.h) when
// a CPU switches the task out, unless it exits, and loads them from it with
// `FpuRestore()` when a CPU switches to the task. Tasks without a state, idle
// tasks included, leave the registers alone.
std::shared_ptr<TaskScheduler> CreateCoreRunqueueScheduler();

#if __STDC_HOSTED__
// Returns the lowest address of the stack that the core allocated for `task`,
// where the stack canary lives. For host unit tests only.
uintptr_t CoreTaskStackBaseForTest(const Task* task);
#endif

}  // namespace protos

#endif  // PROTOS_CORE_RUNQUEUE_SCHEDULER_H_
