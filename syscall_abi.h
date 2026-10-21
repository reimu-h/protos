#ifndef PROTOS_SYSCALL_ABI_H_
#define PROTOS_SYSCALL_ABI_H_

#include <cstdint>

namespace protos {

// The system call interface between user programs (user/) and the kernel
// (user_mode.h).
//
// User code makes a system call with `int 0x80` (`kVectorSyscall` in idt.h):
// the system call number goes in rax and the arguments in rdi, rsi and rdx,
// and the result comes back in rax. All other registers, including the
// x87/MMX/SSE/AVX state with MXCSR and the x87 control word, and the flags are
// preserved. Results from -4095 to -1 are errors (`kSyscallError*`).
//
// The kernel serves system calls with interrupts enabled, like other kernel
// code: the program may be switched out in the middle of one and continue on
// another CPU.

// How a user program ended.
enum class UserProgramOutcome : uint8_t {
  // It called exit(); `UserProgramResult::exit_status` holds the status.
  kExited = 0,
  // It raised a CPU exception, which killed it; the `fault_*` fields of
  // `UserProgramResult` describe the exception.
  kFaulted = 1,
  // `UserProcess::Kill()` killed it.
  kKilled = 2,
  // It never ran: its executable is invalid (elf.h) or memory ran out.
  kLoadFailed = 3,
};

struct UserProgramResult {
  UserProgramOutcome outcome = UserProgramOutcome::kLoadFailed;
  int64_t exit_status = 0;
  // The exception vector (idt.h), error code and the address of the faulting
  // instruction.
  int fault_vector = -1;
  uint64_t fault_error_code = 0;
  uint64_t fault_rip = 0;
  // For page faults, the address whose access faulted (CR2), else 0.
  uint64_t fault_address = 0;
};

// exit(status): ends the program with exit status `status` (rdi, int64_t).
// Does not return.
constexpr int64_t kSyscallExit = 0;
// write(buffer, size): writes the `size` (rsi) bytes at `buffer` (rdi) to the
// console, leaving out NUL bytes, and returns `size`. Fails with
// `kSyscallErrorInvalid` unless `0 <= size <= kSyscallWriteMaxSize`, and with
// `kSyscallErrorFault` if user code may not read all of the buffer.
constexpr int64_t kSyscallWrite = 1;
// get_cpu(): returns the index of the CPU on which the call started. The
// program may run on another CPU by the time it gets the result.
constexpr int64_t kSyscallGetCpu = 2;
// spin(buffer, size, ticks): reads the `size` (rsi) bytes at `buffer` (rdi)
// over and over until the CPUs that run the call have taken `ticks` (rdx)
// timer interrupts while running it, and returns
// `SyscallSpinChecksum(buffer, size)`, which is never negative. Every timer
// interrupt that interrupts the call counts, also the one after which the
// program is switched out, so a call that shares its CPU with k - 1 other
// runnable tasks lasts about k * `ticks` timer ticks. A long system call for
// tests: it keeps the kernel busy reading user memory with interrupts
// enabled, and may be switched out and continue on another CPU meanwhile.
// `ticks == 0` reads the buffer once. If the program is killed in the middle
// of the call, the call stops early and the program ends instead of getting a
// result (user_mode.h). Fails with `kSyscallErrorInvalid` unless
// `0 <= size <= kSyscallSpinMaxSize` and `0 <= ticks <= kSyscallSpinMaxTicks`,
// and with `kSyscallErrorFault` if user code may not read all of the buffer.
constexpr int64_t kSyscallSpin = 3;
// read(buffer, size): reads between 1 and `size` (rsi) bytes of console input
// into `buffer` (rdi), waiting if none is available yet, and returns the
// number of bytes read (`0` when `size == 0`). If the program is killed while
// waiting, the call stops early and the program ends instead of getting a
// result. Fails with `kSyscallErrorInvalid` unless
// `0 <= size <= kSyscallReadMaxSize`, and with `kSyscallErrorFault` if
// `size > 0` and user code may not write all of the buffer.
constexpr int64_t kSyscallRead = 4;
// spawn(name, name_len, arg): starts the embedded user program whose name is
// the `name_len` (rsi) bytes at `name` (rdi) in a new child user process with
// `arg` (rdx) in its rdi, and returns the child's process ID
// (`1 <= pid <= kMaxUserChildProcesses`). Fails with `kSyscallErrorInvalid`
// unless `1 <= name_len <= kSyscallSpawnMaxNameLength`, the name matches an
// embedded user program, a child slot is free, and the child process can be
// started, and with `kSyscallErrorFault` if user code may not read all of
// `[name, name + name_len)`.
constexpr int64_t kSyscallSpawn = 5;
// wait(pid, result): waits until child process `pid` (rdi) of the calling
// process has ended, writes its `UserProgramResult` to `result` (rsi),
// reclaims the child slot, and returns 0. If the calling process is killed
// while waiting, the call stops early and the calling process ends as killed.
// Fails with `kSyscallErrorInvalid` unless `1 <= pid <= kMaxUserChildProcesses`
// and `pid` names an active child of the calling process, and with
// `kSyscallErrorFault` if user code may not write `sizeof(UserProgramResult)`
// bytes at `result`.
constexpr int64_t kSyscallWait = 6;

// The most bytes that one write() writes.
constexpr int64_t kSyscallWriteMaxSize = 4096;
// The most bytes that one spin() reads, and the most timer ticks that it lasts
// (1000000 ticks are more than an hour at 250 Hz).
constexpr int64_t kSyscallSpinMaxSize = 4096;
constexpr int64_t kSyscallSpinMaxTicks = 1000000;
// The most bytes that one read() reads.
constexpr int64_t kSyscallReadMaxSize = 4096;
// The longest program name that spawn() accepts, and the most unwaited child
// processes that one process may have at a time.
constexpr int64_t kSyscallSpawnMaxNameLength = 64;
constexpr int64_t kMaxUserChildProcesses = 8;

// Errors, with the numbers of the corresponding Linux errno values.
constexpr int64_t kSyscallErrorFault = -14;    // EFAULT: bad address
constexpr int64_t kSyscallErrorInvalid = -22;  // EINVAL: invalid argument
constexpr int64_t kSyscallErrorNoSys = -38;    // ENOSYS: unknown system call

// The checksum of no bytes; see `SyscallSpinChecksum()`.
constexpr int64_t kSyscallSpinChecksumInit = 0x4BF29CE484222325;

// Returns spin()'s checksum of the bytes whose checksum is `checksum`,
// followed by the `size` (>= 0) bytes at `data`: the 64-bit FNV-1a hash of all
// of them with bit 63 cleared, so that it is never negative and never an
// error. Clearing bit 63 after every byte gives the same result, so a buffer
// can be checksummed piece by piece.
constexpr int64_t SyscallSpinChecksum(
    const uint8_t* const data,  //
    const int64_t size,         //
    const int64_t checksum = kSyscallSpinChecksumInit) {
  uint64_t hash = static_cast<uint64_t>(checksum);
  for (int64_t i = 0; i < size; ++i) {
    hash = (hash ^ data[i]) * 0x100000001B3ULL;
  }
  return static_cast<int64_t>(hash & 0x7FFFFFFFFFFFFFFFULL);
}

}  // namespace protos

#endif  // PROTOS_SYSCALL_ABI_H_
