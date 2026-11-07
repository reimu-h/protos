#ifndef PROTOS_USER_MODE_H_
#define PROTOS_USER_MODE_H_

#include <atomic>
#include <cstdint>
#include <memory>

#include "driver.h"
#include "fs_protocol.h"
#include "ipc.h"
#include "paging.h"
#include "pmm.h"
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
// (user_programs.h), or one that `SpawnFile()` reads from the filesystem into
// the new process's address space before the process starts. Each process is
// a kernel task that runs the program in an address space of its own
// (address_space.h): its code and data at the addresses the executable gives,
// and its stack below `kUserStackTop`.
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

// The top of the user stack. The initial stack pointer of an embedded program
// is 8 bytes below it, where a call would have pushed a return address; that
// of a `SpawnFile()` child is below its argument strings and `argv` array,
// which the top stack page holds. Either way it is 8 more than a multiple of
// 16, as right after a call. The stack takes `kUserStackPages` pages below it.
// The page above is unmapped, and so is the page below the stack, which
// catches stack overflows.
constexpr uintptr_t kUserStackTop = 0xFFFFFFFF7FFFF000ULL;
constexpr int64_t kUserStackPages = 4;

// Per-process user virtual address window for anonymous memory mappings
// (`MemMap()`, `kSyscallMemMap`), placed above `kDriverMmioWindowBase` and
// `kDriverDmaWindowBase` (driver.h) and below the user stack, and so below the
// programs that user/user.ld links above the stack. Other executables may put
// their segments anywhere outside these windows and the stack
// (`UserSegmentRangeAllowed()`). Each mapping is followed by one unmapped
// 4 KiB guard page.
constexpr uintptr_t kUserAnonWindowBase = 0xFFFFB00000000000ULL;
constexpr uintptr_t kUserAnonWindowSize = 1ULL << 30;  // 1 GiB
static_assert(kDriverMmioWindowBase + kDriverMmioWindowSize <=
              kDriverDmaWindowBase);
static_assert(kDriverDmaWindowBase + kDriverDmaWindowSize <=
              kUserAnonWindowBase);
// The user stack and its unmapped bottom and top guard pages.
constexpr uintptr_t kUserStackRegionBase =
    kUserStackTop - static_cast<uintptr_t>((kUserStackPages + 1) * kPageSize);
constexpr uintptr_t kUserStackRegionEnd =
    kUserStackTop + static_cast<uintptr_t>(kPageSize);
static_assert(kUserAnonWindowBase + kUserAnonWindowSize <=
              kUserStackRegionBase);

// Returns true if a loadable ELF segment of `mem_size` bytes at `vaddr` lies in
// user space without its pages overlapping the per-process MMIO, DMA, anonymous
// `MemMap()`, or stack/guard-page regions.
constexpr bool UserSegmentRangeAllowed(const uintptr_t vaddr,
                                       const int64_t mem_size) {
  constexpr uintptr_t page_mask = static_cast<uintptr_t>(kPageSize) - 1;
  if (mem_size <= 0 || !IsUserRange(vaddr, mem_size)) {
    return false;
  }
  const uintptr_t page_base = vaddr & ~page_mask;
  const uintptr_t end =
      ((vaddr + static_cast<uintptr_t>(mem_size)) + page_mask) & ~page_mask;
  auto overlaps = [page_base, end](const uintptr_t win_base,
                                   const uintptr_t win_end) {
    return page_base < win_end && end > win_base;
  };
  return !overlaps(kDriverMmioWindowBase,
                   kDriverMmioWindowBase + kDriverMmioWindowSize) &&
         !overlaps(kDriverDmaWindowBase,
                   kDriverDmaWindowBase + kDriverDmaWindowSize) &&
         !overlaps(kUserAnonWindowBase,
                   kUserAnonWindowBase + kUserAnonWindowSize) &&
         !overlaps(kUserStackRegionBase, kUserStackRegionEnd);
}

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
  // Whether the program is currently in the middle of kSyscallIrqWait /
  // kSyscallIpcRecv / an IPC call (kSyscallIpcCall or a filesystem system
  // call), whether its task is parked (not running on any CPU) inside them,
  // and the total timer interrupts its task has taken so far
  // (`TaskTimerTicks()`).
  bool in_irq_wait = false;
  bool blocked_in_irq_wait = false;
  bool in_ipc_recv = false;
  bool blocked_in_ipc_recv = false;
  bool in_ipc_call = false;
  bool blocked_in_ipc_call = false;
  int64_t task_timer_ticks = 0;
};

// A user process: a task that loads a user program into a new address space,
// runs it, and keeps the result. The object must outlive the task, so if
// `Wait()` has not run, the destructor kills the program (see `Kill()`) and
// waits for the task, which takes a few timer ticks at most.
class UserProcess {
 public:
  // Starts a process that runs `program` (which must stay in memory until the
  // process has ended) with `arg` in the program's rdi and `capabilities`
  // (`kUserCap*`, syscall_abi.h), on a CPU that the scheduler (task.h)
  // chooses. Returns nullptr if the task can't be created; a program that
  // can't be loaded ends with `kLoadFailed`. Must not be called while holding
  // an `IrqSpinLock` (see `CreateTask()`).
  static std::unique_ptr<UserProcess> Start(
      const UserProgram& program,  //
      uint64_t arg,                //
      uint64_t capabilities = kUserCapNone);

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
  // not running (or is blocked in `irq_wait`) is woken and ends the program
  // the same way once it runs again. A program killed before it entered user
  // mode never does. May be called by any task, any number of times, also by
  // several tasks at once and after the program has ended or `Wait()` has
  // returned, which leaves the result alone. Once `Wait()` has started, it
  // only sets the request. Must not be called by an interrupt handler: it
  // takes a lock that the interrupted code may hold.
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

  // Returns the process's capability mask (`kUserCap*`).
  uint64_t capabilities() const { return capabilities_; }

  // Returns the non-zero owner token used by this process when claiming PCI
  // devices and device IRQs.
  uintptr_t OwnerToken() const { return driver_.OwnerToken(); }
  // Returns the token that names this process as the owner of IPC ports
  // (`IpcPortOwner()`, ipc.h).
  uintptr_t IpcOwnerToken() const { return ipc_.OwnerToken(); }

  // Looks up or queries `user_addr` in the process's live address space.
  // Intended for boot self-tests that inspect mapped pages while the process is
  // alive (`space_ != nullptr`).
  uintptr_t LookupUserPage(uintptr_t user_addr) const;
  StatusOr<UserPageInfo> QueryUserPage(uintptr_t user_addr) const;
  bool BindVectorsForSelftest(int device_index, int count);

  // Returns what the kernel has observed of the process so far. Each field is
  // read atomically, but not all of them at the same instant. May be called by
  // any task while the object exists.
  UserProcessStats stats() const;

 private:
  friend void UserModeInit();

  UserProcess(const UserProgram& program,  //
              uint64_t arg,                //
              uint64_t capabilities);
  UserProcess(std::unique_ptr<AddressSpace> preloaded_space,  //
              uintptr_t preloaded_entry,                      //
              uintptr_t initial_rsp,                          //
              uint64_t arg,                                   //
              uint64_t initial_rsi,                           //
              uint64_t initial_rdx,                           //
              uint64_t capabilities);

  // Starts a process whose segments and stack are already mapped in `space`
  // with entry point `entry` and initial stack/arguments.
  static std::unique_ptr<UserProcess> StartPrepared(
      std::unique_ptr<AddressSpace> space,  //
      uintptr_t entry,                      //
      uintptr_t initial_rsp,                //
      uint64_t arg,                         //
      uint64_t initial_rsi,                 //
      uint64_t initial_rdx,                 //
      uint64_t capabilities);

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
  // The stop condition of the blocking waits of process `ctx` (IrqWait() and
  // the IPC and filesystem system calls): true once it is being killed.
  static bool ShouldStopIrqWait(void* ctx);
  // Returns true once `Kill()` has been called. Long system calls poll it and
  // return early; the program then ends on its way back to user mode.
  bool IsKillRequested() const {
    return kill_requested_.load(std::memory_order_acquire);
  }
  // Returns true if user code may read/write the `size` (> 0) bytes at
  // `buffer`.
  bool MayRead(uintptr_t buffer, int64_t size) const;
  bool MayWrite(uintptr_t buffer, int64_t size) const;
  // Serve write(), spin(), read(), spawn(), wait(), driver, IPC, boot module
  // and filesystem system calls (syscall_abi.h): return the result for rax.
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
  int64_t Spawn(uintptr_t name_ptr,  //
                int64_t name_len,    //
                uint64_t arg,        //
                uint64_t child_caps);
  int64_t SpawnFile(uintptr_t path_ptr,  //
                    int64_t path_len,    //
                    uintptr_t args_ptr,  //
                    int64_t argc,        //
                    uint64_t arg);
  int64_t WaitChild(int64_t pid, uintptr_t result_ptr);
  int64_t PciInfo(int64_t index, uintptr_t info_ptr) const;
  int64_t DmaAlloc(int64_t size, uintptr_t phys_out_ptr, uint64_t flags);
  int64_t IrqWait(uint64_t mask, int64_t timeout_ticks);
  int64_t ConsolePush(uintptr_t buffer, int64_t size) const;
  int64_t IpcListen(int64_t port);
  int64_t IpcRecv(int64_t port,       //
                  uintptr_t buf_ptr,  //
                  int64_t buf_cap,    //
                  uintptr_t msg_ptr,  //
                  int64_t timeout_ticks);
  int64_t IpcReply(int64_t port,         //
                   uint64_t token,       //
                   uintptr_t reply_ptr,  //
                   int64_t reply_size,   //
                   int64_t status);
  int64_t IpcCallSyscall(int64_t port,         //
                         uintptr_t req_ptr,    //
                         int64_t req_size,     //
                         uintptr_t reply_ptr,  //
                         int64_t reply_cap,    //
                         uintptr_t reply_size_ptr);
  int64_t MemMap(int64_t size);
  // The boot module system calls, BootModuleInfo() and BootModuleRead().
  int64_t GetBootModuleInfo(int64_t index, uintptr_t info_ptr) const;
  int64_t ReadBootModule(int64_t index,     //
                         int64_t offset,    //
                         uintptr_t buffer,  //
                         int64_t size) const;
  // The filesystem system calls (syscall_abi.h) and their helpers.
  struct FsFdSlot;
  // Returns the slot of open file descriptor `fd`, or nullptr.
  FsFdSlot* OpenFdSlot(int64_t fd);
  // Checks the path of `path_len` bytes at `path_ptr` and copies it to
  // `out_path` (room for `kFsMaxPathLength` bytes).
  int64_t ValidateAndReadFsPath(uintptr_t path_ptr,  //
                                int64_t path_len,    //
                                char* out_path) const;
  // Sends the `req_size`-byte request at `req_buf` to the filesystem server,
  // with room for `reply_cap` reply bytes at `reply_buf`, and returns the
  // server's status. Fails with `kSyscallErrorNoDevice` if no process owns
  // `kIpcPortFs`, or with `kSyscallErrorStale` if another server than `server`
  // (`!= kIpcNoOwner`) owns it. Writes the reply's size, whether the server
  // received the request, and the owner token of the server that the request
  // was sent to (`kIpcNoOwner` if it wasn't sent) to the `out_*` pointers that
  // aren't null.
  int64_t CallFsServer(uintptr_t server,         //
                       const void* req_buf,      //
                       int64_t req_size,         //
                       void* reply_buf,          //
                       int64_t reply_cap,        //
                       int64_t* out_reply_size,  //
                       bool* out_was_popped,     //
                       uintptr_t* out_server);
  // Sends the `FsPathRequest` `op` for the path of `path_len` bytes at
  // `path_ptr`, like `CallFsServer()`.
  int64_t CallFsServerWithPath(FsRpcOp op,          //
                               uintptr_t path_ptr,  //
                               int64_t path_len,    //
                               void* reply_buf,     //
                               int64_t reply_cap,   //
                               int64_t* out_reply_size);
  int64_t CloseFsHandle(uintptr_t server, uint64_t handle);
  int64_t SeekFsHandle(uintptr_t server,  //
                       uint64_t handle,   //
                       int64_t offset,    //
                       int64_t whence);
  // Reads `size` (> 0) bytes from open file `handle` on `server` at its
  // current offset into `dst`, returning 0 on success, 1 if the file ended
  // early, or a negative `kSyscallError*` value.
  int64_t ReadExactFsSequential(uintptr_t server,  //
                                uint64_t handle,   //
                                void* dst,         //
                                int64_t size);
  // Seeks `handle` on `server` to `offset` (`kFsSeekSet`) and reads `size`
  // (> 0) bytes into `dst`, with the same return values as
  // `ReadExactFsSequential()`.
  int64_t ReadExactFsRange(uintptr_t server,  //
                           uint64_t handle,   //
                           int64_t offset,    //
                           void* dst,         //
                           int64_t size);
  // Maps and fills the segments of `executable` from `handle` on `server` and
  // maps the stack in a new address space, storing it in `*out_space`. Returns
  // 0 on success or a negative `kSyscallError*` value.
  int64_t LoadFromFs(uintptr_t server,                 //
                     uint64_t handle,                  //
                     const ElfExecutable& executable,  //
                     std::unique_ptr<AddressSpace>* out_space);
  int64_t FsOpen(uintptr_t path_ptr, int64_t path_len, uint64_t flags);
  int64_t FsClose(int64_t fd);
  int64_t FsRead(int64_t fd, uintptr_t buf_ptr, int64_t size);
  int64_t FsWrite(int64_t fd, uintptr_t buf_ptr, int64_t size);
  int64_t FsSeek(int64_t fd, int64_t offset, int64_t whence);
  int64_t FsStatPath(uintptr_t path_ptr, int64_t path_len, uintptr_t stat_ptr);
  int64_t FsReadDir(int64_t fd, uintptr_t entries_ptr, int64_t max_entries);
  int64_t FsMkDir(uintptr_t path_ptr, int64_t path_len);
  int64_t FsUnlink(uintptr_t path_ptr, int64_t path_len);
  int64_t FsRename(uintptr_t old_ptr,  //
                   int64_t old_len,    //
                   uintptr_t new_ptr,  //
                   int64_t new_len);
  void RecordKillInterruptInSyscall();
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
  std::unique_ptr<AddressSpace> preloaded_space_;
  const uintptr_t preloaded_entry_ = 0;
  const uintptr_t initial_rsp_ = kUserStackTop - 8;
  const uint64_t arg_;
  const uint64_t initial_rsi_ = 0;
  const uint64_t initial_rdx_ = 0;
  const uint64_t capabilities_;
  // The process's task. Start() stores it once CreateTask() has returned, but
  // the task may start running, on another CPU, before that: only the creator,
  // and tasks to which it hands the process after Start(), may read it. Code
  // that runs on the task itself (`Run()` and the system call and interrupt
  // handlers) must not; it gets its task from `TaskScheduler::CurrentTask()`.
  Task* task_ = nullptr;
  bool waited_ = false;
  // The address space while the program runs, else nullptr. `space_lock_`
  // synchronizes publication and teardown of `space_` in `Run()` with external
  // callers of `LookupUserPage()` and `QueryUserPage()`.
  mutable IrqSpinLock space_lock_;
  AddressSpace* space_ = nullptr;
  DriverContext driver_;
  // The process's IPC ports, and `ipc_buf_`, the kernel buffer for the
  // requests and replies of the ports that it serves (kIpcMaxMessageSize
  // bytes, allocated by its first successful IpcListen(); one suffices since
  // the process has a single task).
  IpcContext ipc_;
  std::unique_ptr<uint8_t[]> ipc_buf_;
  // Identifies the process as an IPC client (`IpcReceivedMessage::client_id`);
  // never reused.
  const uint64_t ipc_client_id_;
  // For each port in `kIpcFirstUserPort .. kIpcPortCount - 1`, the owner token
  // of the latest server that received a call from this process (`kIpcNoOwner`
  // if none): if that server still owns the port when the process ends, it gets
  // a disconnect notification.
  uintptr_t ipc_called_port_owners_[kIpcPortCount] = {};
  // The anonymous memory window's next free address and the bytes mapped.
  uintptr_t next_anon_addr_ = kUserAnonWindowBase;
  int64_t anon_total_bytes_ = 0;
  // An open file descriptor: the filesystem server's handle and the owner
  // token of the server that issued it.
  struct FsFdSlot {
    bool open = false;
    // Whether the server opened a directory (`FsNodeType::kDirectory`) rather
    // than a regular file, which decides which Seek() replies are well-formed.
    bool directory = false;
    uint64_t handle = 0;
    uintptr_t server = kIpcNoOwner;
  };
  FsFdSlot fds_[kMaxOpenFiles] = {};
  // Owner token of a filesystem server that may hold an open file of this
  // process that no descriptor names (an open or close was interrupted or
  // failed after or before reaching the server), so that if that server still
  // owns `kIpcPortFs` when the process ends, it gets a disconnect notification
  // even with no descriptor open.
  uintptr_t fs_cleanup_server_ = kIpcNoOwner;
  // Written by the task (and its interrupt handlers); read after `Join()`.
  UserProgramResult result_;
  std::atomic<bool> kill_requested_{false};
  std::atomic<int64_t> syscall_count_{0};
  std::atomic<int> last_syscall_cpu_{-1};
  // Kill() reads `task_` and wakes it only under this lock and while
  // `task_reclaimed_` is false. Wait() sets `task_reclaimed_` under it before
  // `Join()` frees the task. Only `TaskScheduler::Wake()` (which takes a
  // single innermost runqueue lock) may be called while holding it.
  mutable IrqSpinLock kill_lock_;
  bool task_reclaimed_ = false;
  Task* exit_waiter_task_ = nullptr;
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
  std::atomic<bool> in_irq_wait_{false};
  std::atomic<bool> in_ipc_recv_{false};
  std::atomic<bool> in_ipc_call_{false};
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
