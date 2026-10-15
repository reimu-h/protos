#ifndef PROTOS_USER_SYSCALL_H_
#define PROTOS_USER_SYSCALL_H_

#include <cstdint>

#include "syscall_abi.h"

// System call wrappers for the user programs (see syscall_abi.h).

namespace protos::user {

// Makes system call `number` with up to three arguments and returns its
// result. The kernel preserves every register but rax.
inline int64_t Syscall(const int64_t number,     //
                       const uint64_t arg0 = 0,  //
                       const uint64_t arg1 = 0,  //
                       const uint64_t arg2 = 0) {
  int64_t result = 0;
  asm volatile("int 0x80"
               : "=a"(result)
               : "a"(number), "D"(arg0), "S"(arg1), "d"(arg2)
               : "memory");
  return result;
}

[[noreturn]] inline void Exit(const int64_t status) {
  Syscall(kSyscallExit, static_cast<uint64_t>(status));
  // exit() does not return; trap (#UD) if it ever did.
  __builtin_trap();
}

inline int64_t Write(const void* const buffer, const int64_t size) {
  return Syscall(kSyscallWrite,                        //
                 reinterpret_cast<uintptr_t>(buffer),  //
                 static_cast<uint64_t>(size));
}

// Writes the null-terminated string `text`.
inline int64_t Print(const char* const text) {
  int64_t length = 0;
  while (text[length] != '\0') {
    ++length;
  }
  return Write(text, length);
}

inline int GetCpu() { return static_cast<int>(Syscall(kSyscallGetCpu)); }

// Each user program defines its main function, which the entry point
// (user/crt.cpp) calls with the argument that the kernel passed in rdi. The
// program exits with the status that it returns.
int64_t Main(uint64_t arg);

}  // namespace protos::user

#endif  // PROTOS_USER_SYSCALL_H_
