#ifndef PROTOS_SYSCALL_ABI_H_
#define PROTOS_SYSCALL_ABI_H_

#include <cstdint>

namespace protos {

// The system call interface between user programs (user/) and the kernel
// (user_mode.h).
//
// User code makes a system call with `int 0x80` (`kVectorSyscall` in idt.h):
// the system call number goes in rax and the arguments in rdi, rsi and rdx,
// and the result comes back in rax. All other registers and the flags are
// preserved. Results from -4095 to -1 are errors (`kSyscallError*`).

// exit(status): ends the program with exit status `status` (rdi, int64_t).
// Does not return.
constexpr int64_t kSyscallExit = 0;
// write(buffer, size): writes the `size` (rsi) bytes at `buffer` (rdi) to the
// console, leaving out NUL bytes, and returns `size`. Fails with
// `kSyscallErrorInvalid` unless `0 <= size <= kSyscallWriteMaxSize`, and with
// `kSyscallErrorFault` if user code may not read all of the buffer.
constexpr int64_t kSyscallWrite = 1;
// get_cpu(): returns the index of the CPU that serves the call.
constexpr int64_t kSyscallGetCpu = 2;

// The most bytes that one write() writes.
constexpr int64_t kSyscallWriteMaxSize = 4096;

// Errors, with the numbers of the corresponding Linux errno values.
constexpr int64_t kSyscallErrorFault = -14;    // EFAULT: bad address
constexpr int64_t kSyscallErrorInvalid = -22;  // EINVAL: invalid argument
constexpr int64_t kSyscallErrorNoSys = -38;    // ENOSYS: unknown system call

}  // namespace protos

#endif  // PROTOS_SYSCALL_ABI_H_

