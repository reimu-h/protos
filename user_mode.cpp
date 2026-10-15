#include "user_mode.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <new>

#include "address_space.h"
#include "check.h"
#include "console.h"
#include "elf.h"
#include "format.h"
#include "gdt.h"
#include "idt.h"
#include "paging.h"
#include "pmm.h"
#include "smp.h"
#include "syscall_abi.h"
#include "task.h"

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
  GetTaskScheduler()->Join(task_);
  waited_ = true;
  return result_;
}

void UserProcess::Kill() {
  kill_requested_.store(true, std::memory_order_release);
}

void UserProcess::TaskEntry(void* const raw_process) {
  static_cast<UserProcess*>(raw_process)->Run();
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
  TaskScheduler* const scheduler = GetTaskScheduler();
  space_ = space.get();
  scheduler->SetAddressSpace(space_);
  // Returns once an interrupt handler has stored the result and called End().
  UserModeEnter(executable.entry, kUserStackTop - 8, arg_, this);
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
  UserProcess* const process = FromFrame(frame);
  process->syscall_count_.fetch_add(1, std::memory_order_acq_rel);
  process->last_syscall_cpu_.store(CurrentCpuIdIrqsOff(),
                                   std::memory_order_release);
  if (process->kill_requested_.load(std::memory_order_acquire)) {
    UserProgramResult result;
    result.outcome = UserProgramOutcome::kKilled;
    process->End(frame, result);
  }

  int64_t ret = kSyscallErrorNoSys;
  switch (static_cast<int64_t>(frame->rax)) {
    case kSyscallExit: {
      UserProgramResult result;
      result.outcome = UserProgramOutcome::kExited;
      result.exit_status = static_cast<int64_t>(frame->rdi);
      process->End(frame, result);
    }
    case kSyscallWrite:
      ret = process->Write(frame->rdi, static_cast<int64_t>(frame->rsi));
      break;
    case kSyscallGetCpu:
      ret = CurrentCpuIdIrqsOff();
      break;
    default:
      break;
  }
  frame->rax = static_cast<uint64_t>(ret);
}

int64_t UserProcess::Write(const uintptr_t buffer, const int64_t size) const {
  if (size < 0 || size > kSyscallWriteMaxSize) {
    return kSyscallErrorInvalid;
  }
  if (size == 0) {
    return 0;
  }
  if (!IsUserRange(buffer, size) ||
      !PagingUserRangeAccessible(space_->root(),  //
                                 buffer,          //
                                 size,            //
                                 /*write=*/false)) {
    return kSyscallErrorFault;
  }
  // The buffer stays mapped: only the program's own system calls could change
  // its address space, and interrupts stay disabled.
  const char* const bytes = reinterpret_cast<const char*>(buffer);
  char chunk[kWriteChunkSize + 1];
  int64_t done = 0;
  while (done < size) {
    int length = 0;
    while (length < kWriteChunkSize && done < size) {
      const char c = bytes[done++];
      if (c != '\0') {
        chunk[length++] = c;
      }
    }
    chunk[length] = '\0';
    ConsoleWrite(chunk);
  }
  return size;
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

void UserProcess::End(const InterruptFrame* const frame,
                      const UserProgramResult& result) {
  result_ = result;
  UserModeExit(reinterpret_cast<uintptr_t>(frame + 1));
}

}  // namespace protos
