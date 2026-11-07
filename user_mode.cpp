#include "user_mode.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>

#include "address_space.h"
#include "check.h"
#include "console.h"
#include "cpu_clock.h"
#include "elf.h"
#include "format.h"
#include "fpu.h"
#include "fs_protocol.h"
#include "gdt.h"
#include "idt.h"
#include "interrupts.h"
#include "ipc.h"
#include "kill_delay_injection.h"
#include "multiboot.h"
#include "paging.h"
#include "pmm.h"
#include "smp.h"
#include "spinlock.h"
#include "steal_time.h"
#include "syscall_abi.h"
#include "task.h"
#include "user_access.h"
#include "user_programs.h"

// user_mode_entry.S.
extern "C" void UserModeEnter(uint64_t rip,      //
                              uint64_t rsp,      //
                              uint64_t rdi_arg,  //
                              uint64_t rsi_arg,  //
                              uint64_t rdx_arg,  //
                              protos::UserProcess* process);
extern "C" [[noreturn]] void UserModeExit(uintptr_t kernel_rsp);

namespace protos {

// user_mode_entry.S hard-codes the user selectors.
static_assert(kGdtUserDataSelector == 0x23);
static_assert(kGdtUserCodeSelector == 0x2B);

namespace {

constexpr uintptr_t kPageMask = static_cast<uintptr_t>(kPageSize) - 1;
constexpr int64_t kUserStackSize = kUserStackPages * kPageSize;
static_assert((kUserStackTop & kPageMask) == 0);
static_assert(IsUserRange(kUserStackTop - kUserStackSize, kUserStackSize));
// write() copies the user buffer to the console in chunks of this size.
constexpr int kWriteChunkSize = 128;
// read() copies console input to the user buffer in chunks of this size.
constexpr int kReadChunkSize = 128;
// spin() reads the user buffer in chunks of this size.
constexpr int kSpinChunkSize = 256;

// Returns the timer ticks of the bootstrap processor (CPU 0): the global clock
// of `UserProcessStats`.
static int64_t GlobalTimerTicks() {
  return SmpGetCpuLocal(0)->timer_ticks.load(std::memory_order_relaxed);
}

struct ConsoleReadWaiter {
  Task* task = nullptr;
  ConsoleReadWaiter* next = nullptr;
};

IrqSpinLock g_console_read_wait_lock;
ConsoleReadWaiter* g_console_read_waiters = nullptr;
std::atomic<uint64_t> g_next_ipc_client_id{1};

static uint64_t AllocateIpcClientId() {
  uint64_t id = g_next_ipc_client_id.fetch_add(1, std::memory_order_relaxed);
  if (id == 0) {
    id = g_next_ipc_client_id.fetch_add(1, std::memory_order_relaxed);
  }
  return id;
}

static void WakeConsoleReaders() {
  TaskScheduler* const scheduler = GetTaskScheduler();
  if (scheduler == nullptr) {
    return;
  }
  const IrqSpinLockGuard guard(g_console_read_wait_lock);
  for (ConsoleReadWaiter* w = g_console_read_waiters; w != nullptr;
       w = w->next) {
    scheduler->Wake(w->task);
  }
}

// Copies the `size` bytes at `user_address` in the address space of the
// running program to `dst`. Every kernel read of user memory goes through
// here, and from here through CopyFromUser(), the only code that reads user
// pages while SMAP is enabled (user_access.h). The caller has checked that
// the program may read them (`MayRead()`). They stay readable while the
// system call runs, also when the task is switched out in the middle of it or
// moves to another CPU: only the program's own task changes its address
// space, and every CPU that resumes the task switches to it
// (`TaskScheduler::SetAddressSpace()` in task.h).
static void ReadUserMemory(void* const dst,               //
                           const uintptr_t user_address,  //
                           const int64_t size) {
  DCHECK(size >= 0);
  // Part of the user range that MayRead() checked.
  const bool copied = CopyFromUser(dst, user_address, size);
  DCHECK(copied);
}

// Copies the `size` bytes at `src` to `user_address` in the address space of
// the running program. Every kernel write to user memory goes through here,
// and from here through CopyToUser(), the only code that writes user pages
// while SMAP is enabled (user_access.h). The caller has checked that the
// program may write them (`MayWrite()`).
static void WriteUserMemory(const uintptr_t user_address,  //
                            const void* const src,         //
                            const int64_t size) {
  DCHECK(size >= 0);
  // Part of the user range that MayWrite() checked.
  const bool copied = CopyToUser(user_address, src, size);
  DCHECK(copied);
}

// Returns `SyscallSpinChecksum()` of the `size` bytes at `buffer`, which the
// running program may read.
static int64_t ChecksumUserMemory(const uintptr_t buffer, const int64_t size) {
  int64_t checksum = kSyscallSpinChecksumInit;
  uint8_t chunk[kSpinChunkSize];
  for (int64_t done = 0; done < size;) {
    const int64_t length =
        (size - done < kSpinChunkSize) ? size - done : kSpinChunkSize;
    ReadUserMemory(chunk, buffer + static_cast<uintptr_t>(done), length);
    checksum = SyscallSpinChecksum(chunk, length, checksum);
    done += length;
  }
  return checksum;
}

}  // namespace

// Points the calling CPU's TSS.rsp0 at the saved kernel context of the program
// that UserModeEnter() is about to enter, with interrupts disabled.
extern "C" void UserModeSetKernelStack(const uintptr_t kernel_rsp) {
  GdtSetKernelStack(kernel_rsp);
}

void UserModeInit() {
  CHECK(IdtIsInitialized());
  IdtRegisterHandler(kVectorSyscall, UserProcess::OnSyscall);
  IdtSetUserFaultHandler(UserProcess::OnUserFault);
  IdtSetUserReturnHandler(UserProcess::OnReturnToUser);
  ConsoleSetInputWakeCallback(WakeConsoleReaders);
}

void WriteUserProgramResult(const UserProgramResult& result,
                            void (*const write)(const char* text)) {
  char dec[kFormatDecBufferSize];
  char hex[kFormatHexBufferSize];
  switch (result.outcome) {
    case UserProgramOutcome::kExited:
      write("exited with status ");
      if (result.exit_status < 0) {
        write("-");
        write(FormatDec(0 - static_cast<uint64_t>(result.exit_status), dec));
      } else {
        write(FormatDec(static_cast<uint64_t>(result.exit_status), dec));
      }
      return;
    case UserProgramOutcome::kFaulted:
      write("killed by CPU exception ");
      write(FormatDec(static_cast<uint64_t>(result.fault_vector), dec));
      if (result.fault_vector >= 0 &&
          result.fault_vector < kCpuExceptionCount) {
        write(" (");
        write(IdtExceptionMnemonic(result.fault_vector));
        write(")");
      }
      write(" at rip=");
      write(FormatHex(result.fault_rip, hex));
      write(" error=");
      write(FormatHex(result.fault_error_code, hex));
      if (result.fault_vector == kVectorPageFault) {
        write(" address=");
        write(FormatHex(result.fault_address, hex));
      }
      return;
    case UserProgramOutcome::kKilled:
      write("killed");
      return;
    case UserProgramOutcome::kLoadFailed:
      write("could not be loaded");
      return;
  }
}

std::unique_ptr<UserProcess> UserProcess::Start(const UserProgram& program,  //
                                                const uint64_t arg,          //
                                                const uint64_t capabilities) {
  TaskScheduler* const scheduler = GetTaskScheduler();
  DCHECK(scheduler != nullptr);
  std::unique_ptr<UserProcess> process(
      new (std::nothrow) UserProcess(program, arg, capabilities));
  if (process == nullptr) {
    return nullptr;
  }
  process->task_ =
      scheduler->CreateTask(TaskEntry, process.get(), kDefaultTaskWeight);
  if (process->task_ == nullptr) {
    process->waited_ = true;
    return nullptr;
  }
  return process;
}

std::unique_ptr<UserProcess> UserProcess::StartPrepared(
    std::unique_ptr<AddressSpace> space,  //
    const uintptr_t entry,                //
    const uintptr_t initial_rsp,          //
    const uint64_t arg,                   //
    const uint64_t initial_rsi,           //
    const uint64_t initial_rdx,           //
    const uint64_t capabilities) {
  DCHECK(space != nullptr);
  TaskScheduler* const scheduler = GetTaskScheduler();
  DCHECK(scheduler != nullptr);
  std::unique_ptr<UserProcess> process(
      new (std::nothrow) UserProcess(std::move(space), entry, initial_rsp, arg,
                                     initial_rsi, initial_rdx, capabilities));
  if (process == nullptr) {
    return nullptr;
  }
  process->task_ =
      scheduler->CreateTask(TaskEntry, process.get(), kDefaultTaskWeight);
  if (process->task_ == nullptr) {
    process->waited_ = true;
    return nullptr;
  }
  return process;
}

UserProcess::UserProcess(const UserProgram& program,  //
                         const uint64_t arg,          //
                         const uint64_t capabilities)
    : program_(program),
      arg_(arg),
      capabilities_(capabilities),
      ipc_client_id_(AllocateIpcClientId()) {}

UserProcess::UserProcess(std::unique_ptr<AddressSpace> preloaded_space,  //
                         const uintptr_t preloaded_entry,                //
                         const uintptr_t initial_rsp,                    //
                         const uint64_t arg,                             //
                         const uint64_t initial_rsi,                     //
                         const uint64_t initial_rdx,                     //
                         const uint64_t capabilities)
    : preloaded_space_(std::move(preloaded_space)),
      preloaded_entry_(preloaded_entry),
      initial_rsp_(initial_rsp),
      arg_(arg),
      initial_rsi_(initial_rsi),
      initial_rdx_(initial_rdx),
      capabilities_(capabilities),
      ipc_client_id_(AllocateIpcClientId()) {}

UserProcess::~UserProcess() {
  if (!waited_) {
    Kill();
    Wait();
  }
}

UserProgramResult UserProcess::Wait() {
  DCHECK(!waited_);
  {
    // From here on Kill() leaves the task alone, which Join() frees.
    const IrqSpinLockGuard guard(kill_lock_);
    task_reclaimed_ = true;
  }
  GetTaskScheduler()->Join(task_);
  waited_ = true;
  return result_;
}

void UserProcess::Kill() {
  // With interrupts disabled, the request, the look at where the task runs and
  // the kick happen on this CPU, back to back.
  const ScopedIrqDisable irq_disable;
  int running_cpu = -1;
  {
    const IrqSpinLockGuard guard(kill_lock_);
    // The first request before Wait() records what the program is doing
    // before the request is published: until then the program can't react
    // to it (leave its system call, stop running or end), so the record
    // doesn't depend on how long this CPU takes from one step to the next
    // (for example while the host deschedules its vCPU).
    if (!task_reclaimed_ && kill_tick_.load(std::memory_order_relaxed) < 0) {
      DCHECK(task_ != nullptr);
      const int64_t cpu0_steal_ns = SmpStealNsOfCpu(0);
      const int64_t now = GlobalTimerTicks();
      kill_running_cpu_.store(TaskRunningCpu(task_), std::memory_order_relaxed);
      kill_in_syscall_.store(in_syscall_.load(std::memory_order_acquire),
                             std::memory_order_relaxed);
      killer_steal_ns_at_kill_.store(cpu0_steal_ns, std::memory_order_relaxed);
      kill_tick_.store(now, std::memory_order_release);
    }
    // Sequentially consistent, like the load in TaskRunningCpu(): if the task
    // starts running after that load, it sees the request when it returns to
    // user mode (TaskRunningCpu() explains the task's side).
    kill_requested_.store(true, std::memory_order_seq_cst);
    if (!task_reclaimed_) {
      running_cpu = TaskRunningCpu(task_);
      TaskScheduler* const scheduler = GetTaskScheduler();
      if (scheduler != nullptr) {
        scheduler->Wake(task_);
      }
    }
  }
  // The program ends on its way back to user mode from whichever interrupt or
  // system call comes next. The kick makes that happen right away rather than
  // at the next timer interrupt of the CPU that runs the task. (That can't be
  // this CPU, which runs the caller.)
  if (running_cpu >= 0 && running_cpu != CurrentCpuIdIrqsOff()) {
    SmpSendIpi(running_cpu, kVectorKickIpi);
  }
}

uintptr_t UserProcess::LookupUserPage(const uintptr_t user_addr) const {
  const IrqSpinLockGuard guard(space_lock_);
  return (space_ != nullptr) ? space_->PageFrame(user_addr) : 0;
}

StatusOr<UserPageInfo> UserProcess::QueryUserPage(
    const uintptr_t user_addr) const {
  const IrqSpinLockGuard guard(space_lock_);
  if (space_ == nullptr) {
    return kStatusError;
  }
  return space_->QueryPage(user_addr);
}

bool UserProcess::BindVectorsForSelftest(const int device_index,
                                         const int count) {
  // The creator calls it after Start() (see `task_`): a vector bound to no
  // task would never wake the process in irq_wait. Nor would one bound to a
  // task that Wait() reclaimed, or that ends because it was killed.
  Task* owner_task = nullptr;
  {
    const IrqSpinLockGuard guard(kill_lock_);
    DCHECK(task_ != nullptr);
    if (task_reclaimed_ || IsKillRequested()) {
      return false;
    }
    owner_task = task_;
  }
  return driver_.BindVectorsForSelftest(device_index, count, owner_task);
}

UserProcessStats UserProcess::stats() const {
  UserProcessStats snapshot;
  snapshot.syscalls = syscall_count_.load(std::memory_order_acquire);
  snapshot.syscalls_switched_out =
      syscalls_switched_out_.load(std::memory_order_acquire);
  snapshot.syscalls_migrated =
      syscalls_migrated_.load(std::memory_order_acquire);
  snapshot.syscall_timer_ticks =
      syscall_timer_ticks_.load(std::memory_order_acquire);
  snapshot.max_syscall_timer_ticks =
      max_syscall_timer_ticks_.load(std::memory_order_acquire);
  snapshot.in_syscall = in_syscall_.load(std::memory_order_acquire);
  snapshot.user_timer_ticks = user_timer_ticks_.load(std::memory_order_acquire);
  snapshot.kill_tick = kill_tick_.load(std::memory_order_acquire);
  if (snapshot.kill_tick >= 0) {
    snapshot.kill_running_cpu =
        kill_running_cpu_.load(std::memory_order_relaxed);
    snapshot.kill_in_syscall = kill_in_syscall_.load(std::memory_order_relaxed);
  }
  snapshot.end_vector = end_vector_.load(std::memory_order_acquire);
  snapshot.end_tick = end_tick_.load(std::memory_order_acquire);
  snapshot.fpu_state = fpu_state_.load(std::memory_order_acquire);
  if (snapshot.kill_tick >= 0 && snapshot.end_tick >= 0) {
    snapshot.kill_notice_tick =
        kill_notice_tick_.load(std::memory_order_acquire);
    snapshot.kill_end_cpu = kill_end_cpu_.load(std::memory_order_relaxed);
    KillStealRecord rec;
    rec.killer_steal_ns_at_kill =
        killer_steal_ns_at_kill_.load(std::memory_order_relaxed);
    rec.killer_steal_ns_at_observe = SmpStealNsOfCpu(0);
    rec.killed_steal_ns_at_notice =
        killed_steal_ns_at_notice_.load(std::memory_order_relaxed);
    rec.killed_steal_ns_at_end =
        killed_steal_ns_at_end_.load(std::memory_order_relaxed);
    rec.notice_cpu = kill_notice_cpu_.load(std::memory_order_relaxed);
    rec.end_cpu = snapshot.kill_end_cpu;
    rec.notice_switches = kill_notice_switches_.load(std::memory_order_relaxed);
    rec.end_switches = kill_end_switches_.load(std::memory_order_relaxed);
    snapshot.kill_same_cpu_no_switch =
        rec.notice_cpu >= 0 && rec.notice_cpu == rec.end_cpu &&
        rec.notice_switches >= 0 && rec.notice_switches == rec.end_switches;
    const int64_t excluded_ns = KillExcludedStealNs(rec);
    snapshot.kill_steal_ticks =
        StealNsToTicks(excluded_ns, SmpTimerTickNsUpperBound());
  }
  snapshot.in_irq_wait = in_irq_wait_.load(std::memory_order_acquire);
  snapshot.in_ipc_recv = in_ipc_recv_.load(std::memory_order_acquire);
  snapshot.in_ipc_call = in_ipc_call_.load(std::memory_order_acquire);
  {
    const IrqSpinLockGuard guard(kill_lock_);
    if (!task_reclaimed_ && task_ != nullptr) {
      snapshot.task_timer_ticks = TaskTimerTicks(task_);
      const bool parked = TaskRunningCpu(task_) < 0;
      snapshot.blocked_in_irq_wait = snapshot.in_irq_wait && parked;
      snapshot.blocked_in_ipc_recv = snapshot.in_ipc_recv && parked;
      snapshot.blocked_in_ipc_call = snapshot.in_ipc_call && parked;
    }
  }
  return snapshot;
}

void UserProcess::TaskEntry(void* const raw_process) {
  UserProcess* const process = static_cast<UserProcess*>(raw_process);
  process->Run();
  // Nothing of the program is left but this task, which exits next.
  process->RecordEnd();
}

void UserProcess::Run() {
  // `result_` says kLoadFailed until the program ends.
  std::unique_ptr<AddressSpace> space = std::move(preloaded_space_);
  uintptr_t entry = preloaded_entry_;
  if (space == nullptr) {
    const StatusOr<ElfExecutable, ElfError> executable =
        ElfParseExecutable(program_.image, program_.size);
    if (!executable.ok()) {
      return;
    }
    space = AddressSpace::Create();
    if (space == nullptr || !Load(*executable, *space)) {
      return;
    }
    entry = executable->entry;
  }
  // The program's x87/MMX/SSE/AVX registers, which start in the architectural
  // initial state. Freed after the scheduler has stopped using them, like
  // `space`, on every path by which the program ends.
  const FpuStatePtr fpu_state = FpuStateCreate();
  if (fpu_state == nullptr) {
    return;
  }
  fpu_state_.store(reinterpret_cast<uintptr_t>(fpu_state.get()),
                   std::memory_order_release);
  TaskScheduler* const scheduler = GetTaskScheduler();
  {
    const IrqSpinLockGuard guard(space_lock_);
    space_ = space.get();
  }
  scheduler->SetAddressSpace(space_);
  scheduler->SetFpuState(fpu_state.get());
  // A program killed before this point never enters user mode. A later kill
  // ends it at its first interrupt from user mode at the latest
  // (OnReturnToUser()): Kill() kicks the CPU that is about to enter it.
  if (IsKillRequested()) {
    {
      const ScopedIrqDisable irq_disable;
      RecordKillNoticeIrqsOff();
    }
    result_.outcome = UserProgramOutcome::kKilled;
  } else {
    // Returns once an interrupt handler has stored the result and called
    // End().
    UserModeEnter(entry,         //
                  initial_rsp_,  //
                  arg_,          //
                  initial_rsi_,  //
                  initial_rdx_,  //
                  this);
  }
  // Quiesce any claimed PCI devices, disable MSI/MSI-X, and release vectors
  // and PCI claims while `space_` is still alive, before DMA frames are
  // unmapped and returned to the PMM, and release the process's IPC ports so
  // any callers (including children) wake up before `CleanupChildren()`.
  driver_.Release(/*address_space_alive=*/space_ != nullptr);
  ipc_.Release();
  ipc_buf_.reset();
#if PROTOS_INJECT_KILL_DELAY != 0
  if (result_.outcome == UserProgramOutcome::kKilled) {
    if (kActiveKillDelayInjection == KillDelayInjection::kTeardown) {
      const int64_t delay =
          injected_kill_delay_ticks_.exchange(0, std::memory_order_acq_rel);
      if (delay > 0) {
        space->SetInjectTeardownDelayTicks(delay);
      }
    } else if (kActiveKillDelayInjection == KillDelayInjection::kSpinner ||
               kActiveKillDelayInjection ==
                   KillDelayInjection::kConcurrentKillers ||
               kActiveKillDelayInjection == KillDelayInjection::kBusyKiller ||
               kActiveKillDelayInjection == KillDelayInjection::kRunaway ||
               kActiveKillDelayInjection == KillDelayInjection::kShutdown) {
      const int64_t delay =
          injected_kill_delay_ticks_.exchange(0, std::memory_order_acq_rel);
      if (delay > 0) {
        BusyWaitKillMeasureTicks(delay);
      }
    }
  }
#endif
  // The registers keep the program's values until a task with a state of its
  // own runs here; nothing saves them anymore.
  scheduler->SetFpuState(nullptr);
  scheduler->SetAddressSpace(nullptr);
  {
    const IrqSpinLockGuard guard(space_lock_);
    space_ = nullptr;
  }
  CleanupChildren();
  // Tell the servers that still own the ports this process used (after any
  // child servers have ended) that it has ended, each once per port: the
  // filesystem servers that may hold open files of the process
  // (`fs_cleanup_server_` and those of its open descriptors), and on each user
  // port the latest owner that received a call from it.
  uintptr_t fs_servers[kMaxOpenFiles + 1];
  int fs_server_count = 0;
  const auto add_fs_server = [&](const uintptr_t server) {
    if (server == kIpcNoOwner) {
      return;
    }
    for (int i = 0; i < fs_server_count; ++i) {
      if (fs_servers[i] == server) {
        return;
      }
    }
    fs_servers[fs_server_count++] = server;
  };
  add_fs_server(fs_cleanup_server_);
  for (const FsFdSlot& slot : fds_) {
    if (slot.open) {
      add_fs_server(slot.server);
    }
  }
  for (int i = 0; i < fs_server_count; ++i) {
    IpcNotifyClientDisconnect(kIpcPortFs, ipc_client_id_, fs_servers[i]);
  }
  for (int port = kIpcFirstUserPort; port < kIpcPortCount; ++port) {
    if (ipc_called_port_owners_[port] != kIpcNoOwner) {
      IpcNotifyClientDisconnect(port,            //
                                ipc_client_id_,  //
                                ipc_called_port_owners_[port]);
    }
  }
}

static bool ExecutableSegmentsAllowed(const ElfExecutable& executable) {
  int64_t pages = 0;
  for (int i = 0; i < executable.segment_count; ++i) {
    const ElfSegment& segment = executable.segments[i];
    if (!UserSegmentRangeAllowed(segment.vaddr, segment.mem_size)) {
      return false;
    }
    const uintptr_t page_start = segment.vaddr & ~kPageMask;
    const uintptr_t page_end =
        (segment.vaddr + static_cast<uintptr_t>(segment.mem_size) + kPageMask) &
        ~kPageMask;
    pages += static_cast<int64_t>((page_end - page_start) / kPageSize);
  }
  return pages <= kUserProgramMaxPages;
}

bool UserProcess::Load(const ElfExecutable& executable,
                       AddressSpace& space) const {
  if (!ExecutableSegmentsAllowed(executable)) {
    return false;
  }
  for (int i = 0; i < executable.segment_count; ++i) {
    const ElfSegment& segment = executable.segments[i];
    const uintptr_t page_start = segment.vaddr & ~kPageMask;
    const uintptr_t end =
        segment.vaddr + static_cast<uintptr_t>(segment.mem_size);
    const uintptr_t file_end =
        segment.vaddr + static_cast<uintptr_t>(segment.file_size);
    for (uintptr_t page = page_start; page < end; page += kPageMask + 1) {
      const uintptr_t frame = space.MapNewPage(page, segment.access);
      if (frame == 0) {
        return false;
      }
      // The new page is zeroed; copy the part of `[segment.vaddr, file_end)`
      // that falls inside `[page, page + kPageSize)`.
      const uintptr_t copy_start =
          (page > segment.vaddr) ? page : segment.vaddr;
      const uintptr_t page_limit = page + kPageSize;
      const uintptr_t copy_end =
          (file_end < page_limit) ? file_end : page_limit;
      if (copy_start < copy_end) {
        __builtin_memcpy(
            reinterpret_cast<uint8_t*>(frame) + (copy_start - page),
            program_.image + segment.file_offset +
                static_cast<int64_t>(copy_start - segment.vaddr),
            copy_end - copy_start);
      }
    }
  }
  for (int64_t i = 1; i <= kUserStackPages; ++i) {
    const uintptr_t page =
        kUserStackTop - static_cast<uintptr_t>(i * kPageSize);
    if (space.MapNewPage(page, UserPageAccess::kReadWrite) == 0) {
      return false;
    }
  }
  return true;
}

UserProcess* UserProcess::FromFrame(const InterruptFrame* const frame) {
  DCHECK(IsUserFrame(*frame));
  const uintptr_t kernel_rsp = reinterpret_cast<uintptr_t>(frame + 1);
  DCHECK(kernel_rsp == GdtGetTss(CurrentCpuIdIrqsOff())->rsp[0]);
  UserProcess* const process =
      *reinterpret_cast<UserProcess* const*>(kernel_rsp);
  DCHECK(process != nullptr && process->space_ != nullptr);
  return process;
}

bool UserProcess::ShouldStopIrqWait(void* const ctx) {
  return static_cast<const UserProcess*>(ctx)->IsKillRequested();
}

void UserProcess::OnSyscall(InterruptFrame* const frame) {
  // The interrupt gate disabled interrupts: the task still runs on the CPU
  // that took the call, whose TSS.rsp0 is right above `frame` (FromFrame()).
  UserProcess* const process = FromFrame(frame);
  TickMeter meter(ReadCpuClock());
  process->syscall_count_.fetch_add(1, std::memory_order_acq_rel);
  process->last_syscall_cpu_.store(meter.start_cpu(),
                                   std::memory_order_release);
  if (process->IsKillRequested()) {
    process->EndKilled(frame);
  }
  const int64_t number = static_cast<int64_t>(frame->rax);
  if (number == kSyscallExit) {
    UserProgramResult result;
    result.outcome = UserProgramOutcome::kExited;
    result.exit_status = static_cast<int64_t>(frame->rdi);
    process->End(frame, result);
  }

  // Serves the call with interrupts enabled, as they were in user mode: the
  // scheduler may switch the task out meanwhile and resume it on another CPU.
  process->in_syscall_.store(true, std::memory_order_release);
  RestoreInterrupts((frame->rflags & kRflagsInterruptEnableBit) != 0);
  int64_t ret = kSyscallErrorNoSys;
  if (!MayMakeSyscall(process->capabilities_, number)) {
    ret = kSyscallErrorPermission;
  } else {
    switch (number) {
      case kSyscallWrite:
        ret = process->Write(frame->rdi, static_cast<int64_t>(frame->rsi));
        break;
      case kSyscallGetCpu:
        ret = meter.start_cpu();
        break;
      case kSyscallSpin:
        ret = process->Spin(frame->rdi,                        //
                            static_cast<int64_t>(frame->rsi),  //
                            static_cast<int64_t>(frame->rdx),  //
                            &meter);
        break;
      case kSyscallRead:
        ret = process->Read(frame->rdi, static_cast<int64_t>(frame->rsi));
        break;
      case kSyscallSpawn:
        ret = process->Spawn(frame->rdi,                        //
                             static_cast<int64_t>(frame->rsi),  //
                             frame->rdx,                        //
                             kUserCapNone);
        break;
      case kSyscallWait:
        ret = process->WaitChild(static_cast<int64_t>(frame->rdi), frame->rsi);
        break;
      case kSyscallSpawnDriver:
        ret = process->Spawn(frame->rdi,                        //
                             static_cast<int64_t>(frame->rsi),  //
                             frame->rdx,                        //
                             kUserCapDriver);
        break;
      case kSyscallPciInfo:
        ret = process->PciInfo(static_cast<int64_t>(frame->rdi), frame->rsi);
        break;
      case kSyscallPciClaim:
        ret = process->driver_.Claim(static_cast<int64_t>(frame->rdi));
        break;
      case kSyscallPciBusMaster:
        ret = process->driver_.SetBusMaster(static_cast<int64_t>(frame->rdi),
                                            static_cast<int64_t>(frame->rsi));
        break;
      case kSyscallMmioMap:
        ret = process->driver_.MmioMap(process->space_,  //
                                       frame->rdi,       //
                                       static_cast<int64_t>(frame->rsi));
        break;
      case kSyscallDmaAlloc:
        ret = process->DmaAlloc(static_cast<int64_t>(frame->rdi),  //
                                frame->rsi,                        //
                                frame->rdx);
        break;
      case kSyscallIrqAlloc:
        // The vectors wake the task that runs this call, the process's own.
        // Not `task_`: Start() stores it only after CreateTask() has
        // returned, and the task may already be running by then.
        ret = process->driver_.IrqAlloc(static_cast<int64_t>(frame->rdi),  //
                                        static_cast<int64_t>(frame->rsi),  //
                                        static_cast<int64_t>(frame->rdx),  //
                                        GetTaskScheduler()->CurrentTask());
        break;
      case kSyscallIrqWait:
        ret = process->IrqWait(frame->rdi, static_cast<int64_t>(frame->rsi));
        break;
      case kSyscallConsolePush:
        ret =
            process->ConsolePush(frame->rdi, static_cast<int64_t>(frame->rsi));
        break;
      case kSyscallGetTicks:
        ret = GlobalTimerTicks();
        break;
      case kSyscallIsaClaim:
        ret = process->driver_.IsaClaim(static_cast<int64_t>(frame->rdi));
        break;
      case kSyscallIsaPortIo:
        ret = process->driver_.IsaPortIo(static_cast<int64_t>(frame->rdi),  //
                                         frame->rsi,                        //
                                         frame->rdx);
        break;
      case kSyscallIsaIrqAlloc:
        ret = process->driver_.IsaIrqAlloc(static_cast<int64_t>(frame->rdi),
                                           GetTaskScheduler()->CurrentTask());
        break;
      case kSyscallIpcListen:
        ret = process->IpcListen(static_cast<int64_t>(frame->rdi));
        break;
      case kSyscallIpcRecv:
        ret = process->IpcRecv(static_cast<int64_t>(frame->rdi),  //
                               frame->rsi,                        //
                               static_cast<int64_t>(frame->rdx),  //
                               frame->r10,                        //
                               static_cast<int64_t>(frame->r8));
        break;
      case kSyscallIpcReply:
        ret = process->IpcReply(static_cast<int64_t>(frame->rdi),  //
                                frame->rsi,                        //
                                frame->rdx,                        //
                                static_cast<int64_t>(frame->r10),  //
                                static_cast<int64_t>(frame->r8));
        break;
      case kSyscallIpcCall:
        ret = process->IpcCallSyscall(static_cast<int64_t>(frame->rdi),  //
                                      frame->rsi,                        //
                                      static_cast<int64_t>(frame->rdx),  //
                                      frame->r10,                        //
                                      static_cast<int64_t>(frame->r8),   //
                                      frame->r9);
        break;
      case kSyscallMemMap:
        ret = process->MemMap(static_cast<int64_t>(frame->rdi));
        break;
      case kSyscallOpen:
        ret = process->FsOpen(frame->rdi,                        //
                              static_cast<int64_t>(frame->rsi),  //
                              frame->rdx);
        break;
      case kSyscallClose:
        ret = process->FsClose(static_cast<int64_t>(frame->rdi));
        break;
      case kSyscallFileRead:
        ret = process->FsRead(static_cast<int64_t>(frame->rdi),  //
                              frame->rsi,                        //
                              static_cast<int64_t>(frame->rdx));
        break;
      case kSyscallFileWrite:
        ret = process->FsWrite(static_cast<int64_t>(frame->rdi),  //
                               frame->rsi,                        //
                               static_cast<int64_t>(frame->rdx));
        break;
      case kSyscallSeek:
        ret = process->FsSeek(static_cast<int64_t>(frame->rdi),  //
                              static_cast<int64_t>(frame->rsi),  //
                              static_cast<int64_t>(frame->rdx));
        break;
      case kSyscallStat:
        ret = process->FsStatPath(frame->rdi,                        //
                                  static_cast<int64_t>(frame->rsi),  //
                                  frame->rdx);
        break;
      case kSyscallReadDir:
        ret = process->FsReadDir(static_cast<int64_t>(frame->rdi),  //
                                 frame->rsi,                        //
                                 static_cast<int64_t>(frame->rdx));
        break;
      case kSyscallMkDir:
        ret = process->FsMkDir(frame->rdi, static_cast<int64_t>(frame->rsi));
        break;
      case kSyscallUnlink:
        ret = process->FsUnlink(frame->rdi, static_cast<int64_t>(frame->rsi));
        break;
      case kSyscallRename:
        ret = process->FsRename(frame->rdi,                        //
                                static_cast<int64_t>(frame->rsi),  //
                                frame->rdx,                        //
                                static_cast<int64_t>(frame->r10));
        break;
      case kSyscallBootModuleInfo:
        ret = process->GetBootModuleInfo(static_cast<int64_t>(frame->rdi),
                                         frame->rsi);
        break;
      case kSyscallBootModuleRead:
        ret = process->ReadBootModule(static_cast<int64_t>(frame->rdi),  //
                                      static_cast<int64_t>(frame->rsi),  //
                                      frame->rdx,                        //
                                      static_cast<int64_t>(frame->r10));
        break;
      case kSyscallSpawnFile:
        ret = process->SpawnFile(frame->rdi,                        //
                                 static_cast<int64_t>(frame->rsi),  //
                                 frame->rdx,                        //
                                 static_cast<int64_t>(frame->r10),  //
                                 frame->r8);
        break;
      default:
        break;
    }
  }
  // Interrupts stay disabled until the program runs again (IdtDispatch()),
  // which ends it instead if it was killed meanwhile (OnReturnToUser()).
  (void)SaveAndDisableInterrupts();
  meter.Add(ReadCpuClock());
  process->RecordSyscall(meter);
  process->in_syscall_.store(false, std::memory_order_release);
  frame->rax = static_cast<uint64_t>(ret);
}

bool UserProcess::MayRead(const uintptr_t buffer, const int64_t size) const {
  DCHECK(size > 0);
  return IsUserRange(buffer, size) &&
         PagingUserRangeAccessible(space_->root(),  //
                                   buffer,          //
                                   size,            //
                                   /*write=*/false);
}

bool UserProcess::MayWrite(const uintptr_t buffer, const int64_t size) const {
  DCHECK(size > 0);
  return IsUserRange(buffer, size) &&
         PagingUserRangeAccessible(space_->root(),  //
                                   buffer,          //
                                   size,            //
                                   /*write=*/true);
}

int64_t UserProcess::Write(const uintptr_t buffer, const int64_t size) const {
  if (size < 0 || size > kSyscallWriteMaxSize) {
    return kSyscallErrorInvalid;
  }
  if (size == 0) {
    return 0;
  }
  if (!MayRead(buffer, size)) {
    return kSyscallErrorFault;
  }
  int64_t done = 0;
  while (done < size && !IsKillRequested()) {
    const int length = (size - done < kWriteChunkSize)
                           ? static_cast<int>(size - done)
                           : kWriteChunkSize;
    char bytes[kWriteChunkSize];
    ReadUserMemory(bytes, buffer + static_cast<uintptr_t>(done), length);
    done += length;
    char text[kWriteChunkSize + 1];
    int text_length = 0;
    for (int i = 0; i < length; ++i) {
      if (bytes[i] != '\0') {
        text[text_length++] = bytes[i];
      }
    }
    text[text_length] = '\0';
    ConsoleWrite(text);
  }
  // Less than `size` only for a killed program, which ends before it can see
  // the result.
  return done;
}

int64_t UserProcess::Spin(const uintptr_t buffer,  //
                          const int64_t size,      //
                          const int64_t ticks,     //
                          TickMeter* const meter) const {
  if (size < 0 || size > kSyscallSpinMaxSize || ticks < 0 ||
      ticks > kSyscallSpinMaxTicks) {
    return kSyscallErrorInvalid;
  }
  if (size > 0 && !MayRead(buffer, size)) {
    return kSyscallErrorFault;
  }
  const int64_t checksum = ChecksumUserMemory(buffer, size);
  bool changed = false;
  meter->Add(ReadCpuClock());
  while (meter->ticks() < ticks && !IsKillRequested()) {
    // The program waits for this call, so every pass reads the same bytes,
    // also after the task moved to another CPU, unless a device writes them
    // (a driver's DMA buffer passes MayRead() like other memory of the
    // program). The kernel must not assert on what user space or a device
    // controls: the call fails instead.
    if (ChecksumUserMemory(buffer, size) != checksum) {
      changed = true;
      break;
    }
    meter->Add(ReadCpuClock());
  }
  if (IsKillRequested()) {
    {
      const ScopedIrqDisable irq_disable;
      const_cast<UserProcess*>(this)->RecordKillNoticeIrqsOff();
    }
#if PROTOS_INJECT_KILL_DELAY != 0
    if (kActiveKillDelayInjection == KillDelayInjection::kMidSyscall) {
      const int64_t delay =
          const_cast<UserProcess*>(this)->injected_kill_delay_ticks_.exchange(
              0, std::memory_order_acq_rel);
      if (delay > 0) {
        BusyWaitKillMeasureTicks(delay);
      }
    }
#endif
  }
  return changed ? kSyscallErrorBusy : checksum;
}

int64_t UserProcess::Read(const uintptr_t buffer, const int64_t size) {
  if (size < 0 || size > kSyscallReadMaxSize) {
    return kSyscallErrorInvalid;
  }
  if (size == 0) {
    return 0;
  }
  if (!MayWrite(buffer, size)) {
    return kSyscallErrorFault;
  }
  StatusOr<char> first = ConsoleTryReadc();
  while (!first.ok()) {
    if (IsKillRequested()) {
      {
        const ScopedIrqDisable irq_disable;
        RecordKillNoticeIrqsOff();
      }
#if PROTOS_INJECT_KILL_DELAY != 0
      if (kActiveKillDelayInjection == KillDelayInjection::kMidSyscall) {
        const int64_t delay =
            injected_kill_delay_ticks_.exchange(0, std::memory_order_acq_rel);
        if (delay > 0) {
          BusyWaitKillMeasureTicks(delay);
        }
      }
#endif
      return 0;
    }
    TaskScheduler* const scheduler = GetTaskScheduler();
    Task* const self =
        (scheduler != nullptr) ? scheduler->CurrentTask() : nullptr;
    if (self != nullptr) {
      ConsoleReadWaiter waiter{.task = self, .next = nullptr};
      {
        const IrqSpinLockGuard guard(g_console_read_wait_lock);
        waiter.next = g_console_read_waiters;
        g_console_read_waiters = &waiter;
      }
      first = ConsoleTryReadc();
      if (!first.ok() && !IsKillRequested()) {
        // Wake immediately on `ConsolePushInput()` (`usbkbd`/`ps2kbd`) or
        // `Kill()`, and poll at most once per 250 Hz tick for hardware COM1 RX.
        // The kernel takes no COM1 interrupt, so this poll has to keep up with
        // the 16-byte receive FIFO: a tick (4 ms) brings about 15 bytes at the
        // 38400 baud of UartInit(), and while the shell echoes, the transmit
        // path drains the receiver too (uart.cpp). A faster baud rate would
        // need the receive interrupt.
        scheduler->BlockUntil(GlobalTimerTicks() + 1);
      }
      {
        const IrqSpinLockGuard guard(g_console_read_wait_lock);
        ConsoleReadWaiter** link = &g_console_read_waiters;
        while (*link != nullptr) {
          if (*link == &waiter) {
            *link = waiter.next;
            break;
          }
          link = &(*link)->next;
        }
      }
      if (first.ok()) {
        break;
      }
    } else {
      asm volatile("pause" : : : "memory");
    }
    first = ConsoleTryReadc();
  }

  char chunk[kReadChunkSize];
  chunk[0] = *first;
  int chunk_len = 1;
  int64_t done = 0;
  while (done + chunk_len < size && !IsKillRequested()) {
    const StatusOr<char> next = ConsoleTryReadc();
    if (!next.ok()) {
      break;
    }
    chunk[chunk_len++] = *next;
    if (chunk_len == kReadChunkSize) {
      WriteUserMemory(buffer + static_cast<uintptr_t>(done), chunk, chunk_len);
      done += chunk_len;
      chunk_len = 0;
    }
  }
  if (chunk_len > 0) {
    WriteUserMemory(buffer + static_cast<uintptr_t>(done), chunk, chunk_len);
    done += chunk_len;
  }
  if (IsKillRequested()) {
    const ScopedIrqDisable irq_disable;
    RecordKillNoticeIrqsOff();
  }
  return done;
}

int64_t UserProcess::Spawn(const uintptr_t name_ptr,  //
                           const int64_t name_len,    //
                           const uint64_t arg,        //
                           const uint64_t child_caps) {
  if (name_len <= 0 || name_len > kSyscallSpawnMaxNameLength) {
    return kSyscallErrorInvalid;
  }
  if (!MayRead(name_ptr, name_len)) {
    return kSyscallErrorFault;
  }
  char name[kSyscallSpawnMaxNameLength];
  ReadUserMemory(name, name_ptr, name_len);
  const StatusOr<UserProgram> program = FindUserProgram(name, name_len);
  if (!program.ok()) {
    return kSyscallErrorInvalid;
  }
  int64_t slot = -1;
  for (int64_t i = 0; i < kMaxUserChildProcesses; ++i) {
    if (children_[i] == nullptr) {
      slot = i;
      break;
    }
  }
  if (slot < 0) {
    return kSyscallErrorInvalid;
  }
  if (IsKillRequested()) {
    const ScopedIrqDisable irq_disable;
    RecordKillNoticeIrqsOff();
    return kSyscallErrorInvalid;
  }
  std::unique_ptr<UserProcess> child =
      UserProcess::Start(*program, arg, child_caps);
  if (child == nullptr) {
    return kSyscallErrorInvalid;
  }
  children_[slot] = std::move(child);
  return slot + 1;
}

int64_t UserProcess::WaitChild(const int64_t pid, const uintptr_t result_ptr) {
  if (pid < 1 || pid > kMaxUserChildProcesses) {
    return kSyscallErrorInvalid;
  }
  const int64_t slot = pid - 1;
  UserProcess* const child = children_[slot].get();
  if (child == nullptr) {
    return kSyscallErrorInvalid;
  }
  if (!MayWrite(result_ptr, sizeof(UserProgramResult))) {
    return kSyscallErrorFault;
  }
  while (child->stats().end_tick < 0) {
    if (IsKillRequested()) {
      {
        const ScopedIrqDisable irq_disable;
        RecordKillNoticeIrqsOff();
      }
#if PROTOS_INJECT_KILL_DELAY != 0
      if (kActiveKillDelayInjection == KillDelayInjection::kMidSyscall) {
        const int64_t delay =
            injected_kill_delay_ticks_.exchange(0, std::memory_order_acq_rel);
        if (delay > 0) {
          BusyWaitKillMeasureTicks(delay);
        }
      }
#endif
      return 0;
    }
    TaskScheduler* const scheduler = GetTaskScheduler();
    Task* const self =
        (scheduler != nullptr) ? scheduler->CurrentTask() : nullptr;
    if (self != nullptr) {
      {
        const IrqSpinLockGuard guard(child->kill_lock_);
        child->exit_waiter_task_ = self;
      }
      if (child->stats().end_tick < 0 && !IsKillRequested()) {
        scheduler->Block();
      }
      {
        const IrqSpinLockGuard guard(child->kill_lock_);
        child->exit_waiter_task_ = nullptr;
      }
    } else {
      asm volatile("pause" : : : "memory");
    }
  }
  const UserProgramResult child_result = child->Wait();
  children_[slot].reset();
  // User space gets a copy built field by field in zeroed memory, as Linux
  // clears a structure before it fills it for copy_to_user(): every byte that
  // reaches user memory is one that the kernel chose, and the reserved fields
  // are zero. (The struct has no padding either, syscall_abi.h.)
  UserProgramResult user_result;
  __builtin_memset(static_cast<void*>(&user_result), 0, sizeof(user_result));
  user_result.outcome = child_result.outcome;
  user_result.exit_status = child_result.exit_status;
  user_result.fault_vector = child_result.fault_vector;
  user_result.fault_error_code = child_result.fault_error_code;
  user_result.fault_rip = child_result.fault_rip;
  user_result.fault_address = child_result.fault_address;
  WriteUserMemory(result_ptr, &user_result, sizeof(user_result));
  if (IsKillRequested()) {
    const ScopedIrqDisable irq_disable;
    RecordKillNoticeIrqsOff();
  }
  return 0;
}

int64_t UserProcess::PciInfo(const int64_t index,
                             const uintptr_t info_ptr) const {
  if (info_ptr == 0) {
    const int count = PciDeviceCount();
    if (index == 0) {
      return count;
    }
    return (index < 0 || index >= count) ? kSyscallErrorNoDevice
                                         : kSyscallErrorFault;
  }
  if (!MayWrite(info_ptr, sizeof(PciDeviceInfo))) {
    return kSyscallErrorFault;
  }
  const StatusOr<DriverPciInfoResult> res = driver_.PciInfo(index);
  if (!res.ok()) {
    return res.status();
  }
  WriteUserMemory(info_ptr, &res->info, sizeof(res->info));
  return res->count;
}

int64_t UserProcess::DmaAlloc(const int64_t size,            //
                              const uintptr_t phys_out_ptr,  //
                              const uint64_t flags) {
  if (size <= 0 || (static_cast<uint64_t>(size) & kPageMask) != 0 ||
      size > kSyscallDmaMaxSize || (flags & ~kSyscallDmaFlagsMask) != 0) {
    return kSyscallErrorInvalid;
  }
  if (phys_out_ptr == 0 || !MayWrite(phys_out_ptr, sizeof(uint64_t))) {
    return kSyscallErrorFault;
  }
  const StatusOr<DriverDmaAllocation> alloc =
      driver_.DmaAlloc(space_, size, flags);
  if (!alloc.ok()) {
    return alloc.status();
  }
  WriteUserMemory(phys_out_ptr, &alloc->phys_addr, sizeof(alloc->phys_addr));
  return static_cast<int64_t>(alloc->user_addr);
}

int64_t UserProcess::IrqWait(const uint64_t mask, const int64_t timeout_ticks) {
  in_irq_wait_.store(true, std::memory_order_release);
  const int64_t ret = driver_.IrqWait(mask, timeout_ticks,
                                      &UserProcess::ShouldStopIrqWait, this);
  in_irq_wait_.store(false, std::memory_order_release);
  if (IsKillRequested()) {
    const ScopedIrqDisable irq_disable;
    RecordKillNoticeIrqsOff();
  }
  return ret;
}

int64_t UserProcess::ConsolePush(const uintptr_t buffer,
                                 const int64_t size) const {
  if (size < 0 || size > kSyscallConsolePushMaxSize) {
    return kSyscallErrorInvalid;
  }
  if (size == 0) {
    return 0;
  }
  if (!MayRead(buffer, size)) {
    return kSyscallErrorFault;
  }
  char local[kSyscallConsolePushMaxSize];
  ReadUserMemory(local, buffer, size);
  ConsolePushInput(local, size);
  return size;
}

void UserProcess::RecordKillInterruptInSyscall() {
  if (!IsKillRequested()) {
    return;
  }
  {
    const ScopedIrqDisable irq_disable;
    RecordKillNoticeIrqsOff();
  }
#if PROTOS_INJECT_KILL_DELAY != 0
  if (kActiveKillDelayInjection == KillDelayInjection::kMidSyscall) {
    const int64_t delay =
        injected_kill_delay_ticks_.exchange(0, std::memory_order_acq_rel);
    if (delay > 0) {
      BusyWaitKillMeasureTicks(delay);
    }
  }
#endif
}

int64_t UserProcess::IpcListen(const int64_t port) {
  if (port < 0 || port >= kIpcPortCount) {
    return kSyscallErrorInvalid;
  }
  if (port == kIpcPortFs && (capabilities_ & kUserCapDriver) == 0) {
    return kSyscallErrorPermission;
  }
  const uintptr_t owner = IpcPortOwner(port);
  if (owner == ipc_.OwnerToken()) {
    return 0;
  }
  if (owner != kIpcNoOwner) {
    return kSyscallErrorBusy;
  }
  // The ports that the process serves share `ipc_buf_`, so that IpcRecv() and
  // IpcReply() never run out of memory.
  const bool had_buffer = ipc_buf_ != nullptr;
  if (!had_buffer) {
    ipc_buf_.reset(new (std::nothrow) uint8_t[kIpcMaxMessageSize]);
    if (ipc_buf_ == nullptr) {
      return kSyscallErrorNoMemory;
    }
  }
  const int64_t rc = ipc_.Listen(port, capabilities_);
  if (rc != 0 && !had_buffer) {
    ipc_buf_.reset();
  }
  return rc;
}

int64_t UserProcess::IpcRecv(const int64_t port,       //
                             const uintptr_t buf_ptr,  //
                             const int64_t buf_cap,    //
                             const uintptr_t msg_ptr,  //
                             const int64_t timeout_ticks) {
  if (port < 0 || port >= kIpcPortCount || buf_cap < 0 ||
      buf_cap > kIpcMaxMessageSize ||
      timeout_ticks > kSyscallIpcTimeoutMaxTicks) {
    return kSyscallErrorInvalid;
  }
  if (msg_ptr == 0 || !MayWrite(msg_ptr, sizeof(IpcReceivedMessage)) ||
      (buf_cap > 0 && (buf_ptr == 0 || !MayWrite(buf_ptr, buf_cap)))) {
    return kSyscallErrorFault;
  }
  if (ipc_buf_ == nullptr) {
    // The process never claimed a port.
    return kSyscallErrorAccess;
  }
  // Recv() fills in every byte (the struct has no padding).
  IpcReceivedMessage msg{};
  in_ipc_recv_.store(true, std::memory_order_release);
  const int64_t rc = ipc_.Recv(port,                             //
                               ipc_buf_.get(),                   //
                               buf_cap,                          //
                               &msg,                             //
                               timeout_ticks,                    //
                               &UserProcess::ShouldStopIrqWait,  //
                               this);
  in_ipc_recv_.store(false, std::memory_order_release);
  RecordKillInterruptInSyscall();
  if (rc == 1 && !IsKillRequested()) {
    if (msg.kind == IpcMessageKind::kCall && msg.size > 0) {
      WriteUserMemory(buf_ptr, ipc_buf_.get(), msg.size);
    }
    WriteUserMemory(msg_ptr, &msg, sizeof(msg));
  }
  return rc;
}

int64_t UserProcess::IpcReply(const int64_t port,         //
                              const uint64_t token,       //
                              const uintptr_t reply_ptr,  //
                              const int64_t reply_size,   //
                              const int64_t status) {
  if (port < 0 || port >= kIpcPortCount || token == 0 || reply_size < 0 ||
      reply_size > kIpcMaxMessageSize) {
    return kSyscallErrorInvalid;
  }
  if (status < 0 && (!SyscallIsError(status) || reply_size != 0)) {
    return kSyscallErrorInvalid;
  }
  if (reply_size > 0 && (reply_ptr == 0 || !MayRead(reply_ptr, reply_size))) {
    return kSyscallErrorFault;
  }
  if (ipc_buf_ == nullptr) {
    // The process never claimed a port.
    return kSyscallErrorAccess;
  }
  if (reply_size > 0) {
    ReadUserMemory(ipc_buf_.get(), reply_ptr, reply_size);
  }
  return ipc_.Reply(port, token, ipc_buf_.get(), reply_size, status);
}

int64_t UserProcess::IpcCallSyscall(const int64_t port,         //
                                    const uintptr_t req_ptr,    //
                                    const int64_t req_size,     //
                                    const uintptr_t reply_ptr,  //
                                    const int64_t reply_cap,    //
                                    const uintptr_t reply_size_ptr) {
  if (port < 0 || port >= kIpcPortCount || req_size < 0 ||
      req_size > kIpcMaxMessageSize || reply_cap < 0 ||
      reply_cap > kIpcMaxMessageSize) {
    return kSyscallErrorInvalid;
  }
  if (port == kIpcPortFs) {
    // Only the filesystem system calls call the filesystem server: its
    // per-process limits (`kMaxOpenFiles`) rely on that.
    return kSyscallErrorPermission;
  }
  if ((req_size > 0 && (req_ptr == 0 || !MayRead(req_ptr, req_size))) ||
      (reply_cap > 0 && (reply_ptr == 0 || !MayWrite(reply_ptr, reply_cap))) ||
      (reply_size_ptr != 0 && !MayWrite(reply_size_ptr, sizeof(int64_t)))) {
    return kSyscallErrorFault;
  }

  std::unique_ptr<uint8_t[]> kreq;
  if (req_size > 0) {
    kreq.reset(new (std::nothrow) uint8_t[static_cast<size_t>(req_size)]);
    if (kreq == nullptr) {
      return kSyscallErrorNoMemory;
    }
    ReadUserMemory(kreq.get(), req_ptr, req_size);
  }
  std::unique_ptr<uint8_t[]> kreply;
  if (reply_cap > 0) {
    kreply.reset(new (std::nothrow) uint8_t[static_cast<size_t>(reply_cap)]);
    if (kreply == nullptr) {
      return kSyscallErrorNoMemory;
    }
  }

  int64_t reply_size = 0;
  bool was_popped = false;
  uintptr_t server = kIpcNoOwner;
  in_ipc_call_.store(true, std::memory_order_release);
  const int64_t status = protos::IpcCall(port,                             //
                                         ipc_client_id_,                   //
                                         ipc_.OwnerToken(),                //
                                         kIpcNoOwner,                      //
                                         kreq.get(),                       //
                                         req_size,                         //
                                         kreply.get(),                     //
                                         reply_cap,                        //
                                         &reply_size,                      //
                                         &was_popped,                      //
                                         &server,                          //
                                         &UserProcess::ShouldStopIrqWait,  //
                                         this);
  in_ipc_call_.store(false, std::memory_order_release);
  if (was_popped) {
    ipc_called_port_owners_[port] = server;
  }
  RecordKillInterruptInSyscall();
  if (!IsKillRequested()) {
    if (reply_size > 0) {
      WriteUserMemory(reply_ptr, kreply.get(), reply_size);
    }
    if (reply_size_ptr != 0) {
      WriteUserMemory(reply_size_ptr, &reply_size, sizeof(reply_size));
    }
  }
  return status;
}

// Frees the physical frames of a list that is linked through the first word
// of each frame.
static void FreeFrameChain(uintptr_t frame) {
  while (frame != 0) {
    const uintptr_t next = *reinterpret_cast<const uintptr_t*>(frame);
    PmmFreeFrame(frame);
    frame = next;
  }
}

int64_t UserProcess::MemMap(const int64_t size) {
  if (size <= 0 || (static_cast<uint64_t>(size) & kPageMask) != 0 ||
      size > kSyscallMemMapMaxSize) {
    return kSyscallErrorInvalid;
  }
  if (anon_total_bytes_ + size > kSyscallMemMapMaxTotalSize) {
    return kSyscallErrorNoMemory;
  }
  const int64_t pages = size / kPageSize;
  // The pages and the guard page after them.
  const uintptr_t span_bytes = static_cast<uintptr_t>(size + kPageSize);
  const uintptr_t window_end = kUserAnonWindowBase + kUserAnonWindowSize;
  if (span_bytes > window_end - next_anon_addr_) {
    return kSyscallErrorNoMemory;
  }

  // Take every frame and page table first, so that running out of memory maps
  // nothing. The frames are linked through their first word until they are
  // mapped.
  uintptr_t frames = 0;
  for (int64_t i = 0; i < pages; ++i) {
    const uintptr_t frame = PmmAllocFrame();
    if (frame == 0) {
      FreeFrameChain(frames);
      return kSyscallErrorNoMemory;
    }
    *reinterpret_cast<uintptr_t*>(frame) = frames;
    frames = frame;
  }
  const uintptr_t user_base = next_anon_addr_;
  if (!space_->ReserveRange(user_base, size)) {
    FreeFrameChain(frames);
    return kSyscallErrorNoMemory;
  }
  for (int64_t mapped = 0; mapped < pages; ++mapped) {
    DCHECK(frames != 0);
    const uintptr_t frame = frames;
    frames = *reinterpret_cast<const uintptr_t*>(frame);
    __builtin_memset(reinterpret_cast<void*>(frame), 0,
                     static_cast<size_t>(kPageSize));
    const uintptr_t page =
        user_base + static_cast<uintptr_t>(mapped * kPageSize);
    const bool mapped_ok =
        space_->MapOwnedFrame(page, frame, UserPageAccess::kReadWrite);
    CHECK(mapped_ok);
  }
  DCHECK(frames == 0);
  next_anon_addr_ = user_base + span_bytes;
  anon_total_bytes_ += size;
  return static_cast<int64_t>(user_base);
}

int64_t UserProcess::GetBootModuleInfo(const int64_t index,
                                       const uintptr_t info_ptr) const {
  const int count = PmmBootModuleCount();
  if (index == 0 && info_ptr == 0) {
    return count;
  }
  if (index < 0 || index >= count) {
    return kSyscallErrorInvalid;
  }
  if (!MayWrite(info_ptr, sizeof(BootModuleInfo))) {
    return kSyscallErrorFault;
  }
  const BootModule& module = PmmBootModule(static_cast<int>(index));
  // Built in zeroed memory, like Wait()'s result (WaitChild()).
  BootModuleInfo info;
  __builtin_memset(static_cast<void*>(&info), 0, sizeof(info));
  info.size = static_cast<int64_t>(module.end - module.start);
  static_assert(sizeof(info.name) == sizeof(module.name));
  for (int64_t i = 0; i < kBootModuleMaxNameLength && module.name[i] != '\0';
       ++i) {
    info.name[i] = module.name[i];
  }
  WriteUserMemory(info_ptr, &info, sizeof(info));
  return count;
}

int64_t UserProcess::ReadBootModule(const int64_t index,     //
                                    const int64_t offset,    //
                                    const uintptr_t buffer,  //
                                    const int64_t size) const {
  if (index < 0 || index >= PmmBootModuleCount() || offset < 0 || size < 0 ||
      size > kSyscallBootModuleReadMaxSize) {
    return kSyscallErrorInvalid;
  }
  if (size == 0) {
    return 0;
  }
  if (!MayWrite(buffer, size)) {
    return kSyscallErrorFault;
  }
  const BootModule& module = PmmBootModule(static_cast<int>(index));
  const int64_t module_size = static_cast<int64_t>(module.end - module.start);
  if (offset >= module_size) {
    return 0;
  }
  const int64_t length =
      (size < module_size - offset) ? size : module_size - offset;
  // The PMM keeps the module's memory reserved and identity-mapped.
  const void* const bytes = reinterpret_cast<const void*>(
      module.start + static_cast<uintptr_t>(offset));
  WriteUserMemory(buffer, bytes, length);
  return length;
}

int64_t UserProcess::ValidateAndReadFsPath(const uintptr_t path_ptr,  //
                                           const int64_t path_len,    //
                                           char* const out_path) const {
  if (path_len < 0) {
    return kSyscallErrorInvalid;
  }
  if (path_len == 0) {
    return kSyscallErrorNoEntry;
  }
  if (path_len > kFsMaxPathLength) {
    if (!MayRead(path_ptr, 1)) {
      return kSyscallErrorFault;
    }
    return kSyscallErrorNameTooLong;
  }
  if (!MayRead(path_ptr, path_len)) {
    return kSyscallErrorFault;
  }
  ReadUserMemory(out_path, path_ptr, path_len);
  for (int64_t i = 0; i < path_len; ++i) {
    if (out_path[i] == '\0') {
      return kSyscallErrorInvalid;
    }
  }
  return 0;
}

UserProcess::FsFdSlot* UserProcess::OpenFdSlot(const int64_t fd) {
  if (fd < 0 || fd >= kMaxOpenFiles || !fds_[fd].open) {
    return nullptr;
  }
  return &fds_[fd];
}

int64_t UserProcess::CallFsServer(const uintptr_t server,         //
                                  const void* const req_buf,      //
                                  const int64_t req_size,         //
                                  void* const reply_buf,          //
                                  const int64_t reply_cap,        //
                                  int64_t* const out_reply_size,  //
                                  bool* const out_was_popped,     //
                                  uintptr_t* const out_server) {
  in_ipc_call_.store(true, std::memory_order_release);
  const int64_t rc = protos::IpcCall(kIpcPortFs,                       //
                                     ipc_client_id_,                   //
                                     ipc_.OwnerToken(),                //
                                     server,                           //
                                     req_buf,                          //
                                     req_size,                         //
                                     reply_buf,                        //
                                     reply_cap,                        //
                                     out_reply_size,                   //
                                     out_was_popped,                   //
                                     out_server,                       //
                                     &UserProcess::ShouldStopIrqWait,  //
                                     this);
  in_ipc_call_.store(false, std::memory_order_release);
  RecordKillInterruptInSyscall();
  return rc;
}

int64_t UserProcess::CallFsServerWithPath(const FsRpcOp op,          //
                                          const uintptr_t path_ptr,  //
                                          const int64_t path_len,    //
                                          void* const reply_buf,     //
                                          const int64_t reply_cap,   //
                                          int64_t* const out_reply_size) {
  // The path goes right after the header.
  uint8_t req_buf[sizeof(FsPathRequest) + kFsMaxPathLength];
  const int64_t path_rc = ValidateAndReadFsPath(
      path_ptr, path_len,
      reinterpret_cast<char*>(req_buf + sizeof(FsPathRequest)));
  if (path_rc < 0) {
    return path_rc;
  }
  FsPathRequest hdr{};
  hdr.op = op;
  hdr.path_len = static_cast<int32_t>(path_len);
  __builtin_memcpy(req_buf, &hdr, sizeof(hdr));
  return CallFsServer(kIpcNoOwner,                                   //
                      req_buf,                                       //
                      static_cast<int64_t>(sizeof(hdr)) + path_len,  //
                      reply_buf,                                     //
                      reply_cap,                                     //
                      out_reply_size,                                //
                      nullptr,                                       //
                      nullptr);
}

static bool IsValidFsNodeType(const FsNodeType type) {
  return type == FsNodeType::kRegular || type == FsNodeType::kDirectory;
}

static bool IsValidFsEntryName(const char* const name, const int64_t len) {
  if (len <= 0 || len > kFsMaxNameLength) {
    return false;
  }
  if ((len == 1 && name[0] == '.') ||
      (len == 2 && name[0] == '.' && name[1] == '.')) {
    return false;
  }
  for (int64_t i = 0; i < len; ++i) {
    if (name[i] == '\0' || name[i] == '/') {
      return false;
    }
  }
  return true;
}

int64_t UserProcess::FsOpen(const uintptr_t path_ptr,  //
                            const int64_t path_len,    //
                            const uint64_t flags) {
  if (!FsOpenFlagsValid(flags)) {
    return kSyscallErrorInvalid;
  }
  // The path goes right after the header.
  uint8_t req_buf[sizeof(FsOpenRequest) + kFsMaxPathLength];
  const int64_t path_rc = ValidateAndReadFsPath(
      path_ptr, path_len,
      reinterpret_cast<char*>(req_buf + sizeof(FsOpenRequest)));
  if (path_rc < 0) {
    return path_rc;
  }
  int fd = -1;
  for (int i = 0; i < kMaxOpenFiles; ++i) {
    if (!fds_[i].open) {
      fd = i;
      break;
    }
  }
  if (fd < 0) {
    return kSyscallErrorTooManyFiles;
  }
  FsOpenRequest hdr{};
  hdr.op = FsRpcOp::kOpen;
  hdr.path_len = static_cast<int32_t>(path_len);
  hdr.flags = flags;
  __builtin_memcpy(req_buf, &hdr, sizeof(hdr));

  FsOpenReply reply{};
  int64_t reply_size = 0;
  bool was_popped = false;
  uintptr_t server = kIpcNoOwner;
  const int64_t rc =
      CallFsServer(kIpcNoOwner,                                   //
                   req_buf,                                       //
                   static_cast<int64_t>(sizeof(hdr)) + path_len,  //
                   &reply,                                        //
                   sizeof(reply),                                 //
                   &reply_size,                                   //
                   &was_popped,                                   //
                   &server);
  if (rc != 0 || reply_size != static_cast<int64_t>(sizeof(FsOpenReply)) ||
      reply.handle == 0 || !IsValidFsNodeType(reply.type)) {
    if (was_popped && (rc >= 0 || IsKillRequested())) {
      // The server got the request, so it may have opened the file anyway
      // (the program was killed meanwhile, or the reply is malformed).
      fs_cleanup_server_ = server;
    }
    return rc < 0 ? rc : kSyscallErrorIo;
  }
  // Also if the program was killed meanwhile: it ends on its way back to user
  // mode, and `Run()` then has the server close the file.
  fds_[fd] = FsFdSlot{.open = true,                                       //
                      .directory = reply.type == FsNodeType::kDirectory,  //
                      .handle = reply.handle,                             //
                      .server = server};
  return fd;
}

int64_t UserProcess::CloseFsHandle(const uintptr_t server,
                                   const uint64_t handle) {
  FsCloseRequest req{};
  req.op = FsRpcOp::kClose;
  req.handle = handle;
  bool was_popped = false;
  const int64_t rc = CallFsServer(server,       //
                                  &req,         //
                                  sizeof(req),  //
                                  nullptr,      //
                                  0,            //
                                  nullptr,      //
                                  &was_popped,  //
                                  nullptr);
  // `IpcCall()` may also fail for another reason before it finds that
  // `server` has ended (`kSyscallErrorNoMemory`), and `server` may end after
  // the call: either way, a server that no longer owns `kIpcPortFs` holds no
  // files anymore.
  if (rc == kSyscallErrorNoDevice || rc == kSyscallErrorStale ||
      IpcPortOwner(kIpcPortFs) != server) {
    // The server that opened the file has ended, and the open file with it.
    return 0;
  }
  if (!was_popped || rc != 0) {
    // The server didn't complete the close (the kernel ran out of memory, the
    // program was killed, or the server returned an error): it closes the file
    // when the process ends. Only the server that owns `kIpcPortFs` gets here,
    // so this never replaces a live server with one that has ended.
    fs_cleanup_server_ = server;
  }
  return rc <= 0 ? rc : kSyscallErrorIo;
}

int64_t UserProcess::FsClose(const int64_t fd) {
  FsFdSlot* const slot = OpenFdSlot(fd);
  if (slot == nullptr) {
    return kSyscallErrorBadFd;
  }
  const FsFdSlot closed = *slot;
  *slot = FsFdSlot{};
  return CloseFsHandle(closed.server, closed.handle);
}

int64_t UserProcess::FsRead(const int64_t fd,         //
                            const uintptr_t buf_ptr,  //
                            const int64_t size) {
  const FsFdSlot* const slot = OpenFdSlot(fd);
  if (slot == nullptr) {
    return kSyscallErrorBadFd;
  }
  if (size < 0 || size > kSyscallFsMaxIoSize) {
    return kSyscallErrorInvalid;
  }
  if (size > 0 && (buf_ptr == 0 || !MayWrite(buf_ptr, size))) {
    return kSyscallErrorFault;
  }

  std::unique_ptr<uint8_t[]> kbuf;
  if (size > 0) {
    kbuf.reset(new (std::nothrow) uint8_t[static_cast<size_t>(size)]);
    if (kbuf == nullptr) {
      return kSyscallErrorNoMemory;
    }
  }
  FsReadRequest req{};
  req.op = FsRpcOp::kRead;
  req.handle = slot->handle;
  req.size = size;
  int64_t reply_size = 0;
  const int64_t rc = CallFsServer(slot->server,  //
                                  &req,          //
                                  sizeof(req),   //
                                  kbuf.get(),    //
                                  size,          //
                                  &reply_size,   //
                                  nullptr,       //
                                  nullptr);
  if (rc < 0) {
    return rc;
  }
  if (reply_size != rc) {
    return kSyscallErrorIo;
  }
  if (rc > 0 && !IsKillRequested()) {
    WriteUserMemory(buf_ptr, kbuf.get(), rc);
  }
  return rc;
}

int64_t UserProcess::FsWrite(const int64_t fd,         //
                             const uintptr_t buf_ptr,  //
                             const int64_t size) {
  const FsFdSlot* const slot = OpenFdSlot(fd);
  if (slot == nullptr) {
    return kSyscallErrorBadFd;
  }
  if (size < 0 || size > kSyscallFsMaxIoSize) {
    return kSyscallErrorInvalid;
  }
  if (size > 0 && (buf_ptr == 0 || !MayRead(buf_ptr, size))) {
    return kSyscallErrorFault;
  }

  const int64_t req_size = static_cast<int64_t>(sizeof(FsWriteRequest)) + size;
  std::unique_ptr<uint8_t[]> req_buf(
      new (std::nothrow) uint8_t[static_cast<size_t>(req_size)]);
  if (req_buf == nullptr) {
    return kSyscallErrorNoMemory;
  }
  FsWriteRequest hdr{};
  hdr.op = FsRpcOp::kWrite;
  hdr.handle = slot->handle;
  hdr.size = size;
  __builtin_memcpy(req_buf.get(), &hdr, sizeof(hdr));
  if (size > 0) {
    ReadUserMemory(req_buf.get() + sizeof(hdr), buf_ptr, size);
  }

  const int64_t rc = CallFsServer(slot->server,   //
                                  req_buf.get(),  //
                                  req_size,       //
                                  nullptr,        //
                                  0,              //
                                  nullptr,        //
                                  nullptr,        //
                                  nullptr);
  // A server that can't write any of `size > 0` bytes fails instead
  // (`kSyscallErrorFileTooBig` or `kSyscallErrorNoSpace`).
  return (rc <= size && (rc != 0 || size == 0)) ? rc : kSyscallErrorIo;
}

int64_t UserProcess::SeekFsHandle(const uintptr_t server,  //
                                  const uint64_t handle,   //
                                  const int64_t offset,    //
                                  const int64_t whence) {
  FsSeekRequest req{};
  req.op = FsRpcOp::kSeek;
  req.whence = static_cast<int32_t>(whence);
  req.handle = handle;
  req.offset = offset;
  return CallFsServer(server,       //
                      &req,         //
                      sizeof(req),  //
                      nullptr,      //
                      0,            //
                      nullptr,      //
                      nullptr,      //
                      nullptr);
}

// Whether `new_offset` (>= 0), the server's reply to a Seek() by `(offset,
// whence)` of a directory (`directory`) or regular file, is well-formed
// (syscall_abi.h): the new offset, which is `offset` for `kFsSeekSet` and at
// most `kFsMaxFileSize` for a regular file. A directory takes only
// `(0, kFsSeekSet)`, which returns 0, and `(0, kFsSeekCur)`, which returns an
// opaque position.
static bool IsValidFsSeekReply(const bool directory,  //
                               const int64_t offset,  //
                               const int64_t whence,  //
                               const int64_t new_offset) {
  if (directory) {
    return offset == 0 &&
           (whence == kFsSeekCur || (whence == kFsSeekSet && new_offset == 0));
  }
  return new_offset <= kFsMaxFileSize &&
         (whence != kFsSeekSet || new_offset == offset);
}

int64_t UserProcess::FsSeek(const int64_t fd,      //
                            const int64_t offset,  //
                            const int64_t whence) {
  const FsFdSlot* const slot = OpenFdSlot(fd);
  if (slot == nullptr) {
    return kSyscallErrorBadFd;
  }
  if (whence != kFsSeekSet && whence != kFsSeekCur && whence != kFsSeekEnd) {
    return kSyscallErrorInvalid;
  }
  const int64_t rc = SeekFsHandle(slot->server, slot->handle, offset, whence);
  if (rc >= 0 && !IsValidFsSeekReply(slot->directory, offset, whence, rc)) {
    return kSyscallErrorIo;
  }
  return rc;
}

int64_t UserProcess::ReadExactFsSequential(const uintptr_t server,  //
                                           const uint64_t handle,   //
                                           void* const dst,         //
                                           const int64_t size) {
  DCHECK(size > 0);
  uint8_t* const out = static_cast<uint8_t*>(dst);
  int64_t done = 0;
  while (done < size) {
    if (IsKillRequested()) {
      return kSyscallErrorInvalid;
    }
    const int64_t left = size - done;
    const int64_t chunk =
        (left < kSyscallFsMaxIoSize) ? left : kSyscallFsMaxIoSize;
    FsReadRequest req{};
    req.op = FsRpcOp::kRead;
    req.handle = handle;
    req.size = chunk;
    int64_t reply_size = 0;
    const int64_t rc = CallFsServer(server,       //
                                    &req,         //
                                    sizeof(req),  //
                                    out + done,   //
                                    chunk,        //
                                    &reply_size,  //
                                    nullptr,      //
                                    nullptr);
    if (rc < 0) {
      return rc;
    }
    if (reply_size != rc) {
      return kSyscallErrorIo;
    }
    if (rc == 0) {
      return 1;
    }
    done += rc;
  }
  return 0;
}

int64_t UserProcess::ReadExactFsRange(const uintptr_t server,  //
                                      const uint64_t handle,   //
                                      const int64_t offset,    //
                                      void* const dst,         //
                                      const int64_t size) {
  const int64_t seek_rc = SeekFsHandle(server, handle, offset, kFsSeekSet);
  if (seek_rc < 0) {
    return seek_rc;
  }
  if (seek_rc != offset) {
    return kSyscallErrorIo;
  }
  return ReadExactFsSequential(server, handle, dst, size);
}

int64_t UserProcess::LoadFromFs(
    const uintptr_t server,           //
    const uint64_t handle,            //
    const ElfExecutable& executable,  //
    std::unique_ptr<AddressSpace>* const out_space) {
  if (!ExecutableSegmentsAllowed(executable)) {
    return kSyscallErrorExecFormat;
  }
  std::unique_ptr<AddressSpace> space = AddressSpace::Create();
  if (space == nullptr) {
    return kSyscallErrorNoMemory;
  }
  for (int i = 0; i < executable.segment_count; ++i) {
    const ElfSegment& segment = executable.segments[i];
    if (segment.file_size > 0) {
      const int64_t seek_rc =
          SeekFsHandle(server, handle, segment.file_offset, kFsSeekSet);
      if (seek_rc < 0) {
        return seek_rc;
      }
      if (seek_rc != segment.file_offset) {
        return kSyscallErrorIo;
      }
    }
    const uintptr_t page_start = segment.vaddr & ~kPageMask;
    const uintptr_t end =
        segment.vaddr + static_cast<uintptr_t>(segment.mem_size);
    const uintptr_t file_end =
        segment.vaddr + static_cast<uintptr_t>(segment.file_size);
    for (uintptr_t page = page_start; page < end; page += kPageMask + 1) {
      const uintptr_t frame = space->MapNewPage(page, segment.access);
      if (frame == 0) {
        return kSyscallErrorNoMemory;
      }
      const uintptr_t copy_start =
          (page > segment.vaddr) ? page : segment.vaddr;
      const uintptr_t page_limit = page + kPageSize;
      const uintptr_t copy_end =
          (file_end < page_limit) ? file_end : page_limit;
      if (copy_start < copy_end) {
        const int64_t want = static_cast<int64_t>(copy_end - copy_start);
        const int64_t read_rc = ReadExactFsSequential(
            server, handle,
            reinterpret_cast<uint8_t*>(frame) + (copy_start - page), want);
        if (read_rc < 0) {
          return read_rc;
        }
        if (read_rc > 0) {
          return kSyscallErrorExecFormat;
        }
      }
    }
  }
  for (int64_t i = 1; i <= kUserStackPages; ++i) {
    const uintptr_t page =
        kUserStackTop - static_cast<uintptr_t>(i * kPageSize);
    if (space->MapNewPage(page, UserPageAccess::kReadWrite) == 0) {
      return kSyscallErrorNoMemory;
    }
  }
  *out_space = std::move(space);
  return 0;
}

// Places the `argc` (`0 <= argc <= kSyscallSpawnMaxArgs`) NUL-terminated
// argument strings from `arg_bytes` (described by `slices`) and the
// `argv[0 .. argc]` pointer array (`argv[argc] == nullptr`) in the already
// zeroed top page of `space`'s user stack, returning the child's `rsp` (8 mod
// 16, pointing at a zero return address) and `argv` user address.
static void PopulateUserStackArgs(AddressSpace& space,                  //
                                  const SyscallArgSlice* const slices,  //
                                  const char* const arg_bytes,          //
                                  const int64_t argc,                   //
                                  uintptr_t* const out_rsp,             //
                                  uintptr_t* const out_argv) {
  const uintptr_t top_page_va = kUserStackTop - kPageSize;
  const uintptr_t top_frame = space.PageFrame(top_page_va);
  DCHECK(top_frame != 0);
  uint8_t* const page = reinterpret_cast<uint8_t*>(top_frame);
  if (argc == 0) {
    // `[kUserStackTop - 16]` is 0 (`argv[0] == nullptr`);
    // `[kUserStackTop - 24]` (`8 mod 16`) is the zero return address.
    *out_argv = kUserStackTop - 16;
    *out_rsp = kUserStackTop - 24;
    return;
  }

  int64_t total_string_bytes = 0;
  for (int64_t i = 0; i < argc; ++i) {
    total_string_bytes += slices[i].len + 1;
  }
  const uintptr_t strings_base =
      (kUserStackTop - static_cast<uintptr_t>(total_string_bytes)) &
      ~uintptr_t{0x7};
  uint64_t argv_ptrs[kSyscallSpawnMaxArgs]{};
  int64_t src_off = 0;
  uintptr_t dst_va = strings_base;
  for (int64_t i = 0; i < argc; ++i) {
    argv_ptrs[i] = dst_va;
    const int64_t len = slices[i].len;
    if (len > 0) {
      __builtin_memcpy(page + (dst_va - top_page_va), arg_bytes + src_off, len);
      src_off += len;
    }
    dst_va += static_cast<uintptr_t>(len + 1);
  }

  uintptr_t argv_end = strings_base & ~uintptr_t{0xF};
  if ((argc & 1) == 0) {
    argv_end -= 8;
  }
  const uintptr_t argv_va =
      argv_end - static_cast<uintptr_t>((argc + 1) * sizeof(uint64_t));
  const uintptr_t rsp_va = argv_va - sizeof(uint64_t);
  DCHECK((rsp_va & 0xF) == 8);
  DCHECK(rsp_va >= top_page_va);
  __builtin_memcpy(page + (argv_va - top_page_va), argv_ptrs,
                   argc * static_cast<int64_t>(sizeof(uint64_t)));
  *out_argv = argv_va;
  *out_rsp = rsp_va;
}

int64_t UserProcess::SpawnFile(const uintptr_t path_ptr,  //
                               const int64_t path_len,    //
                               const uintptr_t args_ptr,  //
                               const int64_t argc,        //
                               const uint64_t arg) {
  // The path goes right after the open request header.
  uint8_t req_buf[sizeof(FsOpenRequest) + kFsMaxPathLength];
  const int64_t path_rc = ValidateAndReadFsPath(
      path_ptr, path_len,
      reinterpret_cast<char*>(req_buf + sizeof(FsOpenRequest)));
  if (path_rc < 0) {
    return path_rc;
  }
  if (argc < 0 || argc > kSyscallSpawnMaxArgs) {
    return kSyscallErrorInvalid;
  }
  SyscallArgSlice slices[kSyscallSpawnMaxArgs]{};
  char arg_bytes[kSyscallSpawnMaxArgBytes];
  if (argc > 0) {
    const int64_t slices_size =
        argc * static_cast<int64_t>(sizeof(SyscallArgSlice));
    if (!MayRead(args_ptr, slices_size)) {
      return kSyscallErrorFault;
    }
    ReadUserMemory(slices, args_ptr, slices_size);
    int64_t total_bytes = 0;
    for (int64_t i = 0; i < argc; ++i) {
      const int64_t len = slices[i].len;
      if (len < 0 || len > kSyscallSpawnMaxArgBytes - total_bytes) {
        return kSyscallErrorInvalid;
      }
      if (len > 0) {
        if (!MayRead(slices[i].ptr, len)) {
          return kSyscallErrorFault;
        }
        ReadUserMemory(arg_bytes + total_bytes, slices[i].ptr, len);
        for (int64_t j = 0; j < len; ++j) {
          if (arg_bytes[total_bytes + j] == '\0') {
            return kSyscallErrorInvalid;
          }
        }
        total_bytes += len;
      }
    }
  }

  int64_t slot = -1;
  for (int64_t i = 0; i < kMaxUserChildProcesses; ++i) {
    if (children_[i] == nullptr) {
      slot = i;
      break;
    }
  }
  if (slot < 0) {
    return kSyscallErrorBusy;
  }

  FsOpenRequest hdr{};
  hdr.op = FsRpcOp::kOpen;
  hdr.path_len = static_cast<int32_t>(path_len);
  hdr.flags = kFsOpenRead;
  __builtin_memcpy(req_buf, &hdr, sizeof(hdr));

  FsOpenReply open_reply{};
  int64_t reply_size = 0;
  bool was_popped = false;
  uintptr_t server = kIpcNoOwner;
  const int64_t open_rc =
      CallFsServer(kIpcNoOwner,                                   //
                   req_buf,                                       //
                   static_cast<int64_t>(sizeof(hdr)) + path_len,  //
                   &open_reply,                                   //
                   sizeof(open_reply),                            //
                   &reply_size,                                   //
                   &was_popped,                                   //
                   &server);
  if (open_rc != 0 || reply_size != static_cast<int64_t>(sizeof(FsOpenReply)) ||
      open_reply.handle == 0 || !IsValidFsNodeType(open_reply.type)) {
    if (was_popped && (open_rc >= 0 || IsKillRequested())) {
      fs_cleanup_server_ = server;
    }
    return open_rc < 0 ? open_rc : kSyscallErrorIo;
  }

  std::unique_ptr<AddressSpace> space;
  uintptr_t entry = 0;
  int64_t load_rc = 0;
  if (open_reply.type == FsNodeType::kDirectory) {
    load_rc = kSyscallErrorIsDir;
  } else {
    const int64_t file_size =
        SeekFsHandle(server, open_reply.handle, 0, kFsSeekEnd);
    if (file_size < 0) {
      load_rc = file_size;
    } else if (file_size > kFsMaxFileSize) {
      load_rc = kSyscallErrorIo;
    } else {
      struct FsElfReadCtx {
        UserProcess* process;
        uintptr_t server;
        uint64_t handle;
      };
      FsElfReadCtx read_ctx{
          .process = this,
          .server = server,
          .handle = open_reply.handle,
      };
      const auto read_fn = [](void* const raw_ctx,   //
                              const int64_t offset,  //
                              void* const buffer,    //
                              const int64_t size) -> int64_t {
        const FsElfReadCtx* const ctx =
            static_cast<const FsElfReadCtx*>(raw_ctx);
        return ctx->process->ReadExactFsRange(ctx->server, ctx->handle, offset,
                                              buffer, size);
      };
      int64_t read_error = 0;
      const StatusOr<ElfExecutable, ElfError> executable =
          ElfParseExecutable(file_size, read_fn, &read_ctx, &read_error);
      if (!executable.ok()) {
        load_rc = (executable.status() == ElfError::kReadError)
                      ? read_error
                      : kSyscallErrorExecFormat;
      } else {
        load_rc = LoadFromFs(server, open_reply.handle, *executable, &space);
        entry = executable->entry;
      }
    }
  }
  (void)CloseFsHandle(server, open_reply.handle);
  if (load_rc < 0) {
    return load_rc;
  }
  if (IsKillRequested()) {
    RecordKillInterruptInSyscall();
    return kSyscallErrorInvalid;
  }
  uintptr_t initial_rsp = 0;
  uintptr_t argv_va = 0;
  PopulateUserStackArgs(*space, slices, arg_bytes, argc, &initial_rsp,
                        &argv_va);
  std::unique_ptr<UserProcess> child = UserProcess::StartPrepared(
      std::move(space), entry, initial_rsp, arg, static_cast<uint64_t>(argc),
      argv_va, kUserCapNone);
  if (child == nullptr) {
    return kSyscallErrorNoMemory;
  }
  children_[slot] = std::move(child);
  return slot + 1;
}

int64_t UserProcess::FsStatPath(const uintptr_t path_ptr,  //
                                const int64_t path_len,    //
                                const uintptr_t stat_ptr) {
  if (stat_ptr == 0 || !MayWrite(stat_ptr, sizeof(FsStat))) {
    return kSyscallErrorFault;
  }
  FsStat server_stat{};
  int64_t reply_size = 0;
  const int64_t rc = CallFsServerWithPath(FsRpcOp::kStat,       //
                                          path_ptr,             //
                                          path_len,             //
                                          &server_stat,         //
                                          sizeof(server_stat),  //
                                          &reply_size);
  if (rc < 0) {
    return rc;
  }
  if (rc != 0 || reply_size != static_cast<int64_t>(sizeof(FsStat)) ||
      !IsValidFsNodeType(server_stat.type) || server_stat.fnode_id == 0 ||
      server_stat.size < 0 || server_stat.nlink <= 0) {
    return kSyscallErrorIo;
  }
  if (!IsKillRequested()) {
    // Only the fields reach the program; the reserved bytes are zero.
    FsStat user_stat;
    __builtin_memset(static_cast<void*>(&user_stat), 0, sizeof(user_stat));
    user_stat.type = server_stat.type;
    user_stat.fnode_id = server_stat.fnode_id;
    user_stat.size = server_stat.size;
    user_stat.nlink = server_stat.nlink;
    WriteUserMemory(stat_ptr, &user_stat, sizeof(user_stat));
  }
  return 0;
}

int64_t UserProcess::FsReadDir(const int64_t fd,             //
                               const uintptr_t entries_ptr,  //
                               const int64_t max_entries) {
  const FsFdSlot* const slot = OpenFdSlot(fd);
  if (slot == nullptr) {
    return kSyscallErrorBadFd;
  }
  if (max_entries < 0 || max_entries > kSyscallFsMaxDirEntries) {
    return kSyscallErrorInvalid;
  }
  const int64_t byte_cap =
      max_entries * static_cast<int64_t>(sizeof(FsDirEntry));
  if (byte_cap > 0 && (entries_ptr == 0 || !MayWrite(entries_ptr, byte_cap))) {
    return kSyscallErrorFault;
  }

  std::unique_ptr<FsDirEntry[]> entries;
  if (max_entries > 0) {
    entries.reset(new (std::nothrow)
                      FsDirEntry[static_cast<size_t>(max_entries)]);
    if (entries == nullptr) {
      return kSyscallErrorNoMemory;
    }
  }
  FsReadDirRequest req{};
  req.op = FsRpcOp::kReadDir;
  req.max_entries = static_cast<int32_t>(max_entries);
  req.handle = slot->handle;
  int64_t reply_size = 0;
  const int64_t rc = CallFsServer(slot->server,   //
                                  &req,           //
                                  sizeof(req),    //
                                  entries.get(),  //
                                  byte_cap,       //
                                  &reply_size,    //
                                  nullptr,        //
                                  nullptr);
  if (rc < 0) {
    return rc;
  }
  if (rc > max_entries ||
      reply_size != rc * static_cast<int64_t>(sizeof(FsDirEntry))) {
    return kSyscallErrorIo;
  }
  for (int64_t i = 0; i < rc; ++i) {
    // Only the fields and the name's `name_len` bytes reach the program;
    // the rest is zero.
    FsDirEntry& entry = entries[static_cast<size_t>(i)];
    if (entry.fnode_id == 0 || !IsValidFsNodeType(entry.type) ||
        !IsValidFsEntryName(entry.name, entry.name_len)) {
      return kSyscallErrorIo;
    }
    FsDirEntry clean;
    __builtin_memset(static_cast<void*>(&clean), 0, sizeof(clean));
    clean.fnode_id = entry.fnode_id;
    clean.type = entry.type;
    clean.name_len = entry.name_len;
    __builtin_memcpy(clean.name, entry.name,
                     static_cast<size_t>(entry.name_len));
    entry = clean;
  }
  if (rc > 0 && !IsKillRequested()) {
    WriteUserMemory(entries_ptr,    //
                    entries.get(),  //
                    rc * static_cast<int64_t>(sizeof(FsDirEntry)));
  }
  return rc;
}

int64_t UserProcess::FsMkDir(const uintptr_t path_ptr, const int64_t path_len) {
  const int64_t rc = CallFsServerWithPath(FsRpcOp::kMkDir,  //
                                          path_ptr,         //
                                          path_len,         //
                                          nullptr,          //
                                          0,                //
                                          nullptr);
  return rc <= 0 ? rc : kSyscallErrorIo;
}

int64_t UserProcess::FsUnlink(const uintptr_t path_ptr,
                              const int64_t path_len) {
  const int64_t rc = CallFsServerWithPath(FsRpcOp::kUnlink,  //
                                          path_ptr,          //
                                          path_len,          //
                                          nullptr,           //
                                          0,                 //
                                          nullptr);
  return rc <= 0 ? rc : kSyscallErrorIo;
}

int64_t UserProcess::FsRename(const uintptr_t old_ptr,  //
                              const int64_t old_len,    //
                              const uintptr_t new_ptr,  //
                              const int64_t new_len) {
  // The old path goes right after the header, and the new path right after
  // the old one.
  uint8_t req_buf[sizeof(FsRenameRequest) + 2 * kFsMaxPathLength];
  char* const old_path =
      reinterpret_cast<char*>(req_buf + sizeof(FsRenameRequest));
  const int64_t old_rc = ValidateAndReadFsPath(old_ptr, old_len, old_path);
  if (old_rc < 0) {
    return old_rc;
  }
  const int64_t new_rc =
      ValidateAndReadFsPath(new_ptr, new_len, old_path + old_len);
  if (new_rc < 0) {
    return new_rc;
  }
  FsRenameRequest hdr{};
  hdr.op = FsRpcOp::kRename;
  hdr.old_len = static_cast<int32_t>(old_len);
  hdr.new_len = static_cast<int32_t>(new_len);
  __builtin_memcpy(req_buf, &hdr, sizeof(hdr));
  const int64_t rc =
      CallFsServer(kIpcNoOwner,                                            //
                   req_buf,                                                //
                   static_cast<int64_t>(sizeof(hdr)) + old_len + new_len,  //
                   nullptr,                                                //
                   0,                                                      //
                   nullptr,                                                //
                   nullptr,                                                //
                   nullptr);
  return rc <= 0 ? rc : kSyscallErrorIo;
}

void UserProcess::CleanupChildren() {
  for (int64_t i = 0; i < kMaxUserChildProcesses; ++i) {
    if (children_[i] != nullptr) {
      children_[i]->Kill();
    }
  }
  for (int64_t i = 0; i < kMaxUserChildProcesses; ++i) {
    if (children_[i] != nullptr) {
      (void)children_[i]->Wait();
      children_[i].reset();
    }
  }
}

void UserProcess::RecordSyscall(const TickMeter& meter) {
  if (meter.switch_outs() > 0) {
    syscalls_switched_out_.fetch_add(1, std::memory_order_release);
  } else if (meter.ticks() >
             max_syscall_timer_ticks_.load(std::memory_order_relaxed)) {
    max_syscall_timer_ticks_.store(meter.ticks(), std::memory_order_release);
  }
  if (meter.migrated()) {
    syscalls_migrated_.fetch_add(1, std::memory_order_release);
  }
  syscall_timer_ticks_.fetch_add(meter.ticks(), std::memory_order_release);
}

void UserProcess::RecordKillNoticeIrqsOff() {
  DCHECK(!AreInterruptsEnabled());
  if (kill_notice_tick_.load(std::memory_order_relaxed) >= 0) {
    return;
  }
  const CpuLocal* const cpu = CurrentCpuOrNullIrqsOff();
  const int cpu_id = (cpu != nullptr) ? cpu->cpu_id : -1;
  const int64_t switches =
      (cpu != nullptr) ? cpu->context_switches.load(std::memory_order_relaxed)
                       : -1;
  const int64_t steal_ns = SmpOwnStealNsIrqsOff();
  const int64_t now = GlobalTimerTicks();
  killed_steal_ns_at_notice_.store(steal_ns, std::memory_order_relaxed);
  kill_notice_cpu_.store(cpu_id, std::memory_order_relaxed);
  kill_notice_switches_.store(switches, std::memory_order_relaxed);
  kill_notice_tick_.store(now, std::memory_order_release);
}

void UserProcess::RecordEnd() {
  const ScopedIrqDisable irq_disable;
  const CpuLocal* const cpu = CurrentCpuOrNullIrqsOff();
  const int cpu_id = (cpu != nullptr) ? cpu->cpu_id : -1;
  const int64_t switches =
      (cpu != nullptr) ? cpu->context_switches.load(std::memory_order_relaxed)
                       : -1;
  const int64_t steal_ns = SmpOwnStealNsIrqsOff();
  const int64_t now = GlobalTimerTicks();
  killed_steal_ns_at_end_.store(steal_ns, std::memory_order_relaxed);
  kill_end_cpu_.store(cpu_id, std::memory_order_relaxed);
  kill_end_switches_.store(switches, std::memory_order_relaxed);
  end_tick_.store(now, std::memory_order_release);
  TaskScheduler* const scheduler = GetTaskScheduler();
  if (scheduler != nullptr) {
    const IrqSpinLockGuard guard(kill_lock_);
    if (exit_waiter_task_ != nullptr) {
      scheduler->Wake(exit_waiter_task_);
    }
  }
}

void UserProcess::OnUserFault(InterruptFrame* const frame) {
  UserProgramResult result;
  result.outcome = UserProgramOutcome::kFaulted;
  result.fault_vector = static_cast<int>(frame->vector);
  result.fault_error_code = frame->error_code;
  result.fault_rip = frame->rip;
  result.fault_address =
      (result.fault_vector == kVectorPageFault) ? ReadCr2() : 0;
  FromFrame(frame)->End(frame, result);
}

void UserProcess::OnReturnToUser(InterruptFrame* const frame) {
  // IdtDispatch() has disabled interrupts and pointed this CPU's TSS.rsp0
  // right above `frame` (FromFrame()).
  UserProcess* const process = FromFrame(frame);
  if (frame->vector == static_cast<uint64_t>(kVectorApicTimer)) {
    process->user_timer_ticks_.fetch_add(1, std::memory_order_release);
  }
  if (process->IsKillRequested()) {
    process->EndKilled(frame);
  }
}

void UserProcess::EndKilled(const InterruptFrame* const frame) {
  RecordKillNoticeIrqsOff();
  end_vector_.store(static_cast<int>(frame->vector), std::memory_order_release);
  UserProgramResult result;
  result.outcome = UserProgramOutcome::kKilled;
  End(frame, result);
}

void UserProcess::End(const InterruptFrame* const frame,
                      const UserProgramResult& result) {
  result_ = result;
  UserModeExit(reinterpret_cast<uintptr_t>(frame + 1));
}

}  // namespace protos
