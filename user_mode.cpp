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

std::unique_ptr<UserProcess> UserProcess::Start(const UserProgram& program,
                                                const uint64_t arg) {
  TaskScheduler* const scheduler = GetTaskScheduler();
  DCHECK(scheduler != nullptr);
  std::unique_ptr<UserProcess> process(new (std::nothrow)
                                           UserProcess(program, arg));
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

UserProcess::UserProcess(const UserProgram& program, const uint64_t arg)
    : program_(program), arg_(arg) {}

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
  ElfExecutable executable;
  if (ElfParseExecutable(program_.image, program_.size, &executable) !=
      ElfError::kNone) {
    return;
  }
  const std::unique_ptr<AddressSpace> space = AddressSpace::Create();
  if (space == nullptr || !Load(executable, *space)) {
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
  space_ = space.get();
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
    UserModeEnter(executable.entry, kUserStackTop - 8, arg_, this);
  }
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
  space_ = nullptr;
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
    default:
      break;
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
  meter->Add(ReadCpuClock());
  while (meter->ticks() < ticks && !IsKillRequested()) {
    // Nothing but the program could change the buffer, and it waits for this
    // call, so every pass reads the same bytes, also after the task moved to
    // another CPU.
    const int64_t again = ChecksumUserMemory(buffer, size);
    DCHECK(again == checksum);
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
  return checksum;
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
