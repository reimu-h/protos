#ifndef PROTOS_USER_MODE_H_
#define PROTOS_USER_MODE_H_

#include <atomic>
#include <cstdint>
#include <memory>

#include "spinlock.h"
#include "syscall_abi.h"

namespace protos {

class AddressSpace;
class TickMeter;
struct ElfExecutable;
struct InterruptFrame;
struct Task;

// User mode
// =========
//
// A user process runs a user program: an ELF executable (elf.h) in kernel
// memory, such as the programs in user/ that the kernel image embeds
// (user_programs.h). Each process is a kernel task that runs the program in
// an address space of its own (address_space.h): its code and data at the
// addresses the executable gives, and its stack below `kUserStackTop`.
//
// The task enters user mode (CPL 3) at the program's entry point with
// interrupts enabled. From then on the CPU returns to the kernel on every
// interrupt, exception and system call, on the task's kernel stack:
// `TSS.rsp0` (`GdtSetKernelStack()`) points right below the kernel context
// that the task saved when it entered user mode. The program ends when it
// calls exit() (syscall_abi.h), when it raises an exception, which kills it,
// or when another task kills it (`UserProcess::Kill()`), and the task then
// continues in the kernel after the point where it entered user mode.
//
// System calls run with interrupts enabled, like other kernel code: the
// scheduler may switch the task out in the middle of one and resume it later,
// possibly on another CPU, where `IdtDispatch()` (idt.h) points `TSS.rsp0` at
// the task's stack again before the program continues.
//
// User programs may use the x87, MMX, SSE and AVX registers (AVX and the
// later state components that the kernel enables in XCR0, cpu.h, if the CPU
// has them). Each process has register state of its own (fpu.h): it starts in
// the architectural initial state (x87 control word 0x037F, MXCSR 0x1F80, all
// registers empty or zero), and the scheduler saves it whenever it switches
// the process's task out and loads it whenever a CPU resumes the task, so the
// registers, MXCSR and the x87 control word keep their values across
// preemption, system calls and moves to other CPUs, and no process sees
// another one's values. The state is freed when the program ends, however it
// ends.

// The top of the user stack: the initial stack pointer of a program is 8 bytes
// below it, where a call would have pushed a return address. The stack takes
// `kUserStackPages` pages below it. The page above is unmapped, and so is the
// page below the stack, which catches stack overflows.
constexpr uintptr_t kUserStackTop = 0xFFFFFFFF7FFFF000ULL;
constexpr int64_t kUserStackPages = 4;

// The most pages that a program's segments may take.
constexpr int64_t kUserProgramMaxPages = 4096;

// A user program: an ELF executable at `[image, image + size)`.
struct UserProgram {
  const uint8_t* image = nullptr;
  int64_t size = 0;
};

// `UserProgramOutcome` and `UserProgramResult` are defined in syscall_abi.h.

// Registers the system call handler (`kVectorSyscall`), the handler that
// kills a user program on a CPU exception (`IdtSetUserFaultHandler()`), and
// the return-to-user handler that ends killed programs
// (`IdtSetUserReturnHandler()`). Must be called once, after `IdtInit()`,
// before the first `UserProcess::Start()`.
void UserModeInit();

// Writes how a program ended with `write` (for example `ConsoleWrite()` from
// console.h), such as "exited with status 0" or "killed by CPU exception 14
// (#PF) at rip=0x... error=0x6 address=0x...", without a newline.
void WriteUserProgramResult(const UserProgramResult& result,
                            void (*write)(const char* text));

// What the kernel has observed of a user process so far, for tests and
// diagnostics (`UserProcess::stats()`). The `*_tick` fields count timer
// interrupts of the bootstrap processor (CPU 0's `CpuLocal::timer_ticks`,
// 250 per second): the global clock of user mode. The other tick counts are
// timer interrupts of the CPUs that ran the program.
struct UserProcessStats {
  // The system calls that the program has made (`syscall_count()`).
  int64_t syscalls = 0;
  // The system calls in the middle of which the scheduler switched the task
  // out at least once.
  int64_t syscalls_switched_out = 0;
  // The system calls that ended on another CPU than the one they started on.
  int64_t syscalls_migrated = 0;
  // The timer interrupts that the CPUs running the program's system calls
  // took while they ran them, summed over the calls. Every timer interrupt
  // that interrupted a call counts, on whichever CPU, also the one after which
  // the scheduler switched the task out (cpu_clock.h `TickMeter`).
  int64_t syscall_timer_ticks = 0;
  // The most timer interrupts that one CPU took in the middle of a single
  // system call that it ran from start to end without switching the task out.
  int64_t max_syscall_timer_ticks = 0;
  // Whether the program is in the middle of a system call (other than exit())
  // right now.
  bool in_syscall = false;
  // The timer interrupts that interrupted the program's user code.
  int64_t user_timer_ticks = 0;
  // The global tick of the first `Kill()` before `Wait()`, or -1 if there was
  // none. The `kill_*` fields describe the program right before that request,
  // which it could not have reacted to yet.
  int64_t kill_tick = -1;
  // The CPU that was running the task, or -1 if none was (the task was
  // queued, had not started yet or had ended).
  int kill_running_cpu = -1;
  // Whether the program was in the middle of a system call.
  bool kill_in_syscall = false;
  // The vector (idt.h) of the interrupt or system call from user mode on whose
  // way back the killed program ended, or -1 if it ended otherwise (also if it
  // was killed before it entered user mode).
  int end_vector = -1;
  // The global tick at which the process ended, once the program's memory was
  // freed, or -1 while it runs. Valid before `Wait()`.
  int64_t end_tick = -1;
  // The address of the program's x87/MMX/SSE/AVX state (`FpuState`, fpu.h),
  // or 0 before the process has created it. It keeps the address after the
  // state is freed, so that a test can tell whether a later process got the
  // same memory.
  uintptr_t fpu_state = 0;
  // Excluded steal-time ticks (`floor(max(0, E1 - B0) / P_conv)`) and
  // diagnostics for a killed process once `end_tick >= 0`.
  int64_t kill_steal_ticks = 0;
  int64_t kill_notice_tick = -1;
  int kill_end_cpu = -1;
  bool kill_same_cpu_no_switch = false;
};

// A user process: a task that loads a user program into a new address space,
// runs it, and keeps the result. The object must outlive the task, so if
// `Wait()` has not run, the destructor kills the program (see `Kill()`) and
// waits for the task, which takes a few timer ticks at most.
class UserProcess {
 public:
  // Starts a process that runs `program` (which must stay in memory until the
  // process has ended) with `arg` in the program's rdi, on a CPU that the
  // scheduler (task.h) chooses. Returns nullptr if the task can't be
  // created; a program that can't be loaded ends with `kLoadFailed`. Must
  // not be called while holding an `IrqSpinLock` (see `CreateTask()`).
  static std::unique_ptr<UserProcess> Start(const UserProgram& program,
                                            uint64_t arg);

  ~UserProcess();

  UserProcess(const UserProcess&) = delete;
  UserProcess& operator=(const UserProcess&) = delete;

  // Waits until the program has ended, reclaims the task and returns how the
  // program ended. Must be called at most once, by one task.
  UserProgramResult Wait();

  // Asks for the program to be killed (`kKilled`) unless it has ended by
  // then. It ends within a few timer ticks: if its task is running, `Kill()`
  // interrupts that CPU (`kVectorKickIpi`), a system call in progress returns
  // early, and the program ends on its way back to user mode; a task that is
  // not running ends the program the same way once it runs again. A program
  // killed before it entered user mode never does. May be called by any task,
  // any number of times, also by several tasks at once and after the program
  // has ended or `Wait()` has returned, which leaves the result alone. Once
  // `Wait()` has started, it only sets the request. Must not be called by an
  // interrupt handler: it takes a lock that the interrupted code may hold.
  void Kill();

  // Arms a synthetic post-kill delay of `ticks` non-stolen timer ticks for the
  // delay-injection proofs (`INJECT_KILL_DELAY=<variant>`).
  void ArmInjectedKillDelay(int64_t ticks) {
    injected_kill_delay_ticks_.store(ticks, std::memory_order_release);
  }

  // The number of system calls that the program has made so far.
  int64_t syscall_count() const {
    return syscall_count_.load(std::memory_order_acquire);
  }

  // The CPU on which the program's latest system call started, or -1 before
  // the first one.
  int last_syscall_cpu() const {
    return last_syscall_cpu_.load(std::memory_order_acquire);
  }

  // Returns what the kernel has observed of the process so far. Each field is
  // read atomically, but not all of them at the same instant. May be called by
  // any task while the object exists.
  UserProcessStats stats() const;

 private:
  friend void UserModeInit();

  UserProcess(const UserProgram& program, uint64_t arg);

  static void TaskEntry(void* raw_process);
  // Loads the program and runs it until it ends, leaving the result in
  // `result_`.
  void Run();
  // Maps and fills the segments of `executable` and the stack in `space`.
  bool Load(const ElfExecutable& executable, AddressSpace& space) const;

  // Interrupt handlers (see `UserModeInit()`). The process whose program
  // raised the interrupt is in the slot above `frame`, which the task pushed
  // when it entered user mode (user_mode_entry.S).
  static void OnSyscall(InterruptFrame* frame);
  static void OnUserFault(InterruptFrame* frame);
  static void OnReturnToUser(InterruptFrame* frame);
  static UserProcess* FromFrame(const InterruptFrame* frame);
  // Returns true once `Kill()` has been called. Long system calls poll it and
  // return early; the program then ends on its way back to user mode.
  bool IsKillRequested() const {
    return kill_requested_.load(std::memory_order_acquire);
  }
  // Returns true if user code may read/write the `size` (> 0) bytes at
  // `buffer`.
  bool MayRead(uintptr_t buffer, int64_t size) const;
  bool MayWrite(uintptr_t buffer, int64_t size) const;
  // Serve write(), spin(), read(), spawn() and wait() (syscall_abi.h): return
  // the result for rax.
  // `meter` measures the system call from its start; spin() lasts until
  // `meter` has counted `ticks` timer interrupts that interrupted the task,
  // on whichever CPUs: other runnable tasks on its CPU make it last longer in
  // wall-clock ticks, but it always ends.
  int64_t Write(uintptr_t buffer, int64_t size) const;
  int64_t Spin(uintptr_t buffer,  //
               int64_t size,      //
               int64_t ticks,     //
               TickMeter* meter) const;
  int64_t Read(uintptr_t buffer, int64_t size);
  int64_t Spawn(uintptr_t name_ptr, int64_t name_len, uint64_t arg);
  int64_t WaitChild(int64_t pid, uintptr_t result_ptr);
  void CleanupChildren();
  // Adds a finished system call, which `meter` measured, to the counters.
  void RecordSyscall(const TickMeter& meter);
  void RecordKillNoticeIrqsOff();
  void RecordEnd();
  // Ends the killed program on the way back from the interrupt or system call
  // of `frame`.
  [[noreturn]] void EndKilled(const InterruptFrame* frame);
  // Ends the program with `result` and returns to the kernel after the point
  // where the task entered user mode.
  [[noreturn]] void End(const InterruptFrame* frame,
                        const UserProgramResult& result);

  const UserProgram program_;
  const uint64_t arg_;
  Task* task_ = nullptr;
  bool waited_ = false;
  // The address space while the program runs, else nullptr.
  AddressSpace* space_ = nullptr;
  // Written by the task (and its interrupt handlers); read after `Join()`.
  UserProgramResult result_;
  std::atomic<bool> kill_requested_{false};
  std::atomic<int64_t> syscall_count_{0};
  std::atomic<int> last_syscall_cpu_{-1};
  // Kill() reads `task_` only under this lock and while `task_reclaimed_` is
  // false. Wait() sets `task_reclaimed_` under it before `Join()` frees the
  // task. A leaf lock: nothing else is taken while holding it.
  IrqSpinLock kill_lock_;
  bool task_reclaimed_ = false;
  // Written by the first Kill() before Wait() under `kill_lock_`, before it
  // publishes the request (`kill_requested_`); `kill_tick_` (stored last, with
  // release) publishes the other two.
  std::atomic<int64_t> kill_tick_{-1};
  std::atomic<int> kill_running_cpu_{-1};
  std::atomic<bool> kill_in_syscall_{false};
  std::atomic<int64_t> killer_steal_ns_at_kill_{-1};
  // Written only by the task (and its interrupt handlers); read by stats().
  std::atomic<int64_t> syscalls_switched_out_{0};
  std::atomic<int64_t> syscalls_migrated_{0};
  std::atomic<int64_t> syscall_timer_ticks_{0};
  std::atomic<int64_t> max_syscall_timer_ticks_{0};
  std::atomic<bool> in_syscall_{false};
  std::atomic<int64_t> user_timer_ticks_{0};
  std::atomic<int> end_vector_{-1};
  std::atomic<int64_t> end_tick_{-1};
  std::atomic<uintptr_t> fpu_state_{0};
  std::atomic<int64_t> kill_notice_tick_{-1};
  std::atomic<int64_t> killed_steal_ns_at_notice_{-1};
  std::atomic<int> kill_notice_cpu_{-1};
  std::atomic<int64_t> kill_notice_switches_{-1};
  std::atomic<int64_t> killed_steal_ns_at_end_{-1};
  std::atomic<int> kill_end_cpu_{-1};
  std::atomic<int64_t> kill_end_switches_{-1};
  std::atomic<int64_t> injected_kill_delay_ticks_{0};
  std::unique_ptr<UserProcess> children_[kMaxUserChildProcesses];
};

}  // namespace protos

#endif  // PROTOS_USER_MODE_H_
