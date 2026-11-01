#ifndef PROTOS_USER_SYSCALL_H_
#define PROTOS_USER_SYSCALL_H_

#include <cstdint>

#include "status_or.h"
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
  // Exit() does not return; trap (#UD) if it ever did.
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

// Makes a Spin() system call: the kernel reads the `size` bytes at `buffer`
// over and over until the CPUs running the call have taken `ticks` timer
// interrupts while running it. Returns their checksum
// (`SyscallSpinChecksum()`), or an error (syscall_abi.h).
inline int64_t SpinOnBuffer(const void* const buffer,  //
                            const int64_t size,        //
                            const int64_t ticks) {
  return Syscall(kSyscallSpin,                         //
                 reinterpret_cast<uintptr_t>(buffer),  //
                 static_cast<uint64_t>(size),          //
                 static_cast<uint64_t>(ticks));
}

// Reads between 1 and `size` bytes of console input into `buffer`, waiting if
// none is available yet, and returns the number of bytes read (`0` when
// `size == 0`), or an error (syscall_abi.h).
inline int64_t Read(void* const buffer, const int64_t size) {
  return Syscall(kSyscallRead,                         //
                 reinterpret_cast<uintptr_t>(buffer),  //
                 static_cast<uint64_t>(size));
}

// Starts the embedded user program whose name is the `name_len` bytes at
// `name` in a new child user process with `arg` in its rdi, and returns the
// child's process ID (`1 <= pid <= kMaxUserChildProcesses`), or an error.
inline int64_t Spawn(const char* const name,  //
                     const int64_t name_len,  //
                     const uint64_t arg) {
  return Syscall(kSyscallSpawn,                      //
                 reinterpret_cast<uintptr_t>(name),  //
                 static_cast<uint64_t>(name_len),    //
                 arg);
}

// Convenience overload for null-terminated `name`.
inline int64_t Spawn(const char* const name, const uint64_t arg = 0) {
  if (name == nullptr) {
    return Spawn(nullptr, 1, arg);
  }
  int64_t length = 0;
  while (name[length] != '\0') {
    ++length;
  }
  return Spawn(name, length, arg);
}

// Waits until child process `pid` has ended and returns its
// `UserProgramResult`, or an error.
inline StatusOr<UserProgramResult> Wait(const int64_t pid) {
  UserProgramResult result{};
  const int64_t rc = Syscall(kSyscallWait,                //
                             static_cast<uint64_t>(pid),  //
                             reinterpret_cast<uintptr_t>(&result));
  if (SyscallIsError(rc)) {
    return rc;
  }
  return result;
}

constexpr bool IsError(const int64_t result) { return SyscallIsError(result); }

// Starts the embedded user program whose name is the `name_len` bytes at
// `name` in a new child driver process (`kUserCapDriver`) with `arg` in its
// rdi, and returns the child's process ID, or an error
// (`kSyscallErrorPermission` unless the caller has `kUserCapSpawnDrivers`).
inline int64_t SpawnDriver(const char* const name,  //
                           const int64_t name_len,  //
                           const uint64_t arg) {
  return Syscall(kSyscallSpawnDriver,                //
                 reinterpret_cast<uintptr_t>(name),  //
                 static_cast<uint64_t>(name_len),    //
                 arg);
}

inline int64_t SpawnDriver(const char* const name, const uint64_t arg = 0) {
  if (name == nullptr) {
    return SpawnDriver(nullptr, 1, arg);
  }
  int64_t length = 0;
  while (name[length] != '\0') {
    ++length;
  }
  return SpawnDriver(name, length, arg);
}

inline int64_t GetPciDeviceCount() { return Syscall(kSyscallPciInfo, 0, 0); }

inline StatusOr<PciDeviceInfo> GetPciInfo(const int64_t index) {
  PciDeviceInfo info{};
  const int64_t rc = Syscall(kSyscallPciInfo,               //
                             static_cast<uint64_t>(index),  //
                             reinterpret_cast<uintptr_t>(&info));
  if (SyscallIsError(rc)) {
    return rc;
  }
  return info;
}

inline int64_t PciClaim(const int64_t index) {
  return Syscall(kSyscallPciClaim, static_cast<uint64_t>(index));
}

inline int64_t PciBusMaster(const int64_t index, const int64_t enable) {
  return Syscall(kSyscallPciBusMaster,          //
                 static_cast<uint64_t>(index),  //
                 static_cast<uint64_t>(enable));
}

inline StatusOr<uintptr_t> MmioMap(const uint64_t phys_addr,
                                   const int64_t size) {
  const int64_t rc =
      Syscall(kSyscallMmioMap, phys_addr, static_cast<uint64_t>(size));
  if (SyscallIsError(rc)) {
    return Error(rc);
  }
  return static_cast<uintptr_t>(rc);
}

struct DmaAllocation {
  uintptr_t va = 0;
  uint64_t phys = 0;
};

inline StatusOr<DmaAllocation> DmaAlloc(const int64_t size,
                                        const uint64_t flags = 0) {
  uint64_t phys = 0;
  const int64_t rc = Syscall(kSyscallDmaAlloc,                    //
                             static_cast<uint64_t>(size),         //
                             reinterpret_cast<uintptr_t>(&phys),  //
                             flags);
  if (SyscallIsError(rc)) {
    return rc;
  }
  return DmaAllocation{
      .va = static_cast<uintptr_t>(rc),
      .phys = phys,
  };
}

inline int64_t IrqAlloc(const int64_t index,  //
                        const int64_t count,  //
                        const int64_t mode = kSyscallIrqModeAuto) {
  return Syscall(kSyscallIrqAlloc,              //
                 static_cast<uint64_t>(index),  //
                 static_cast<uint64_t>(count),  //
                 static_cast<uint64_t>(mode));
}

inline int64_t IrqWait(const uint64_t mask, const int64_t timeout_ticks) {
  return Syscall(kSyscallIrqWait, mask, static_cast<uint64_t>(timeout_ticks));
}

inline int64_t ConsolePush(const void* const buffer, const int64_t size) {
  return Syscall(kSyscallConsolePush,                  //
                 reinterpret_cast<uintptr_t>(buffer),  //
                 static_cast<uint64_t>(size));
}

inline int64_t GetTicks() { return Syscall(kSyscallGetTicks); }

inline int64_t IsaClaim(const int64_t device) {
  return Syscall(kSyscallIsaClaim, static_cast<uint64_t>(device));
}

inline int64_t IsaPortIo(const int64_t op,     //
                         const uint64_t port,  //
                         const uint64_t value = 0) {
  return Syscall(kSyscallIsaPortIo, static_cast<uint64_t>(op), port, value);
}

inline StatusOr<uint8_t> IsaPortIn8(const uint16_t port) {
  const int64_t rc = IsaPortIo(kSyscallPortIoRead8, port, 0);
  if (SyscallIsError(rc)) {
    return Error(rc);
  }
  return static_cast<uint8_t>(rc);
}

inline int64_t IsaPortOut8(const uint16_t port, const uint8_t value) {
  return IsaPortIo(kSyscallPortIoWrite8, port, value);
}

inline int64_t IsaIrqAlloc(const int64_t device) {
  return Syscall(kSyscallIsaIrqAlloc, static_cast<uint64_t>(device));
}

// Each user program defines its main function, which the entry point
// (user/crt.cpp) calls with the argument that the kernel passed in rdi. The
// program exits with the status that it returns.
int64_t Main(uint64_t arg);

// The size of `g_initial_fpu_state`: the legacy region and the header of an
// XSAVE area and the AVX state, which follows them in the standard format.
constexpr int kInitialFpuStateSize = 1024;

}  // namespace protos::user

// The x87, SSE and AVX state that the program started with, which the entry
// point (user/crt.cpp) saves before any other code runs: in the XSAVE format
// with the components that both XCR0 and 0x7 (x87, SSE, AVX) name if
// CPUID.1:ECX.OSXSAVE is set, else in the FXSAVE format.
extern "C" uint8_t g_initial_fpu_state[protos::user::kInitialFpuStateSize];

#endif  // PROTOS_USER_SYSCALL_H_
