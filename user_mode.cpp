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
#include "gdt.h"
#include "idt.h"
#include "interrupts.h"
#include "kill_delay_injection.h"
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
extern "C" void UserModeEnter(uint64_t rip,  //
                              uint64_t rsp,  //
                              uint64_t arg,  //
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

UserProcess::UserProcess(const UserProgram& program,  //
                         const uint64_t arg,          //
                         const uint64_t capabilities)
    : program_(program), arg_(arg), capabilities_(capabilities) {}

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
  {
    const IrqSpinLockGuard guard(kill_lock_);
    if (!task_reclaimed_ && task_ != nullptr) {
      snapshot.task_timer_ticks = TaskTimerTicks(task_);
      snapshot.blocked_in_irq_wait =
          snapshot.in_irq_wait && TaskRunningCpu(task_) < 0;
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
  const StatusOr<ElfExecutable, ElfError> executable =
      ElfParseExecutable(program_.image, program_.size);
  if (!executable.ok()) {
    return;
  }
  const std::unique_ptr<AddressSpace> space = AddressSpace::Create();
  if (space == nullptr || !Load(*executable, *space)) {
    return;
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
    UserModeEnter(executable->entry, kUserStackTop - 8, arg_, this);
  }
  // Quiesce any claimed PCI devices, disable MSI/MSI-X, and release vectors
  // and PCI claims while `space_` is still alive, before DMA frames are
  // unmapped and returned to the PMM.
  driver_.Release(/*address_space_alive=*/space_ != nullptr);
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
}

bool UserProcess::Load(const ElfExecutable& executable,
                       AddressSpace& space) const {
  int64_t pages = 0;
  for (int i = 0; i < executable.segment_count; ++i) {
    const ElfSegment& segment = executable.segments[i];
    pages += (segment.mem_size + kPageSize - 1) / kPageSize;
  }
  if (pages > kUserProgramMaxPages) {
    return false;
  }
  for (int i = 0; i < executable.segment_count; ++i) {
    const ElfSegment& segment = executable.segments[i];
    const uintptr_t end =
        segment.vaddr + static_cast<uintptr_t>(segment.mem_size);
    const uintptr_t file_end =
        segment.vaddr + static_cast<uintptr_t>(segment.file_size);
    for (uintptr_t page = segment.vaddr; page < end; page += kPageMask + 1) {
      const uintptr_t frame = space.MapNewPage(page, segment.access);
      if (frame == 0) {
        return false;
      }
      // The new page is zeroed; copy the part of the file contents that it
      // holds. `segment.vaddr` is page-aligned, so the page starts in the
      // segment.
      if (page < file_end) {
        const uintptr_t copy_size =
            (file_end - page < kPageMask + 1) ? file_end - page : kPageMask + 1;
        __builtin_memcpy(reinterpret_cast<void*>(frame),
                         program_.image + segment.file_offset +
                             static_cast<int64_t>(page - segment.vaddr),
                         copy_size);
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
    if (scheduler != nullptr) {
      scheduler->Yield();
    }
    asm volatile("pause" : : : "memory");
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
    if (scheduler != nullptr) {
      scheduler->Yield();
    }
    asm volatile("pause" : : : "memory");
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
