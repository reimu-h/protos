#ifndef PROTOS_USER_SYSCALL_H_
#define PROTOS_USER_SYSCALL_H_

#include <cstdint>
#include <type_traits>

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

// Make system call `number` with four, five or six arguments, which go in rdi,
// rsi, rdx, r10, r8 and r9 (syscall_abi.h), and return its result.
inline int64_t Syscall4(const int64_t number,  //
                        const uint64_t arg0,   //
                        const uint64_t arg1,   //
                        const uint64_t arg2,   //
                        const uint64_t arg3) {
  int64_t result = 0;
  register uint64_t r10 asm("r10") = arg3;
  asm volatile("int 0x80"
               : "=a"(result)
               : "a"(number), "D"(arg0), "S"(arg1), "d"(arg2), "r"(r10)
               : "memory");
  return result;
}

inline int64_t Syscall5(const int64_t number,  //
                        const uint64_t arg0,   //
                        const uint64_t arg1,   //
                        const uint64_t arg2,   //
                        const uint64_t arg3,   //
                        const uint64_t arg4) {
  int64_t result = 0;
  register uint64_t r10 asm("r10") = arg3;
  register uint64_t r8 asm("r8") = arg4;
  asm volatile("int 0x80"
               : "=a"(result)
               : "a"(number), "D"(arg0), "S"(arg1), "d"(arg2), "r"(r10), "r"(r8)
               : "memory");
  return result;
}

inline int64_t Syscall6(const int64_t number,  //
                        const uint64_t arg0,   //
                        const uint64_t arg1,   //
                        const uint64_t arg2,   //
                        const uint64_t arg3,   //
                        const uint64_t arg4,   //
                        const uint64_t arg5) {
  int64_t result = 0;
  register uint64_t r10 asm("r10") = arg3;
  register uint64_t r8 asm("r8") = arg4;
  register uint64_t r9 asm("r9") = arg5;
  asm volatile("int 0x80"
               : "=a"(result)
               : "a"(number), "D"(arg0), "S"(arg1), "d"(arg2), "r"(r10),
                 "r"(r8), "r"(r9)
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

// Starts the ELF executable at the path of `path_len` bytes at `path` in a new
// unprivileged child user process with `args[0 .. argc - 1]` as its `argv` and
// `arg` in its rdi, and returns the child's process ID
// (`1 <= pid <= kMaxUserChildProcesses`), or an error.
inline int64_t SpawnFile(const char* const path,             //
                         const int64_t path_len,             //
                         const SyscallArgSlice* const args,  //
                         const int64_t argc,                 //
                         const uint64_t arg = 0) {
  return Syscall5(kSyscallSpawnFile,                  //
                  reinterpret_cast<uintptr_t>(path),  //
                  static_cast<uint64_t>(path_len),    //
                  reinterpret_cast<uintptr_t>(args),  //
                  static_cast<uint64_t>(argc),        //
                  arg);
}

inline int64_t SpawnFile(const char* const path,  //
                         const int64_t path_len,  //
                         const uint64_t arg) {
  return SpawnFile(path, path_len, nullptr, 0, arg);
}

// Convenience overload for null-terminated `path` with optional `arg` (and
// `argc == 0`).
inline int64_t SpawnFile(const char* const path, const uint64_t arg = 0) {
  if (path == nullptr) {
    return SpawnFile(nullptr, 1, nullptr, 0, arg);
  }
  int64_t length = 0;
  while (path[length] != '\0') {
    ++length;
  }
  return SpawnFile(path, length, nullptr, 0, arg);
}

// Convenience overload for null-terminated `path` and `argc` null-terminated
// `argv` strings (`0 <= argc <= kSyscallSpawnMaxArgs`). Constrained to actual
// string-array pointers so integer `arg` literals (`0`) resolve to the
// `(path, path_len, arg)` overload above.
template <typename ArgvPtr>
  requires(std::is_pointer_v<ArgvPtr> &&
           std::is_convertible_v<ArgvPtr, const char* const*>)
inline int64_t SpawnFile(const char* const path,  //
                         const int64_t argc,      //
                         const ArgvPtr argv,      //
                         const uint64_t arg = 0) {
  int64_t path_len = 1;
  if (path != nullptr) {
    path_len = 0;
    while (path[path_len] != '\0') {
      ++path_len;
    }
  }
  const char* const* const strings = argv;
  if (argc <= 0 || argc > kSyscallSpawnMaxArgs || strings == nullptr) {
    return SpawnFile(path,                                               //
                     path_len,                                           //
                     reinterpret_cast<const SyscallArgSlice*>(strings),  //
                     argc,                                               //
                     arg);
  }
  SyscallArgSlice slices[kSyscallSpawnMaxArgs]{};
  for (int64_t i = 0; i < argc; ++i) {
    if (strings[i] == nullptr) {
      return kSyscallErrorFault;
    }
    int64_t len = 0;
    while (strings[i][len] != '\0') {
      ++len;
    }
    slices[i] = SyscallArgSlice{
        .ptr = reinterpret_cast<uintptr_t>(strings[i]),
        .len = len,
    };
  }
  return SpawnFile(path, path_len, slices, argc, arg);
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

// Polls whether child process `pid` has ended without blocking. Returns 1 and
// writes the child's result to `*out_result` (and reclaims the child slot) if
// `pid` has ended, 0 (leaving `out_result` untouched and the child active) if
// it is still running, or a negative `kSyscallError*` error (`< 0`, such as
// `kSyscallErrorInvalid` or `kSyscallErrorFault`).
inline int64_t TryWait(const int64_t pid, UserProgramResult* const out_result) {
  return Syscall(kSyscallTryWait,             //
                 static_cast<uint64_t>(pid),  //
                 reinterpret_cast<uintptr_t>(out_result));
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

// IPC system calls (syscall_abi.h): IpcListen() claims a port, IpcRecv() and
// IpcReply() serve its calls, and IpcCall() calls another process's port.
inline int64_t IpcListen(const int64_t port) {
  return Syscall(kSyscallIpcListen, static_cast<uint64_t>(port));
}

inline int64_t IpcRecv(const int64_t port,                 //
                       void* const buffer,                 //
                       const int64_t buffer_cap,           //
                       IpcReceivedMessage* const out_msg,  //
                       const int64_t timeout_ticks) {
  return Syscall5(kSyscallIpcRecv,                       //
                  static_cast<uint64_t>(port),           //
                  reinterpret_cast<uintptr_t>(buffer),   //
                  static_cast<uint64_t>(buffer_cap),     //
                  reinterpret_cast<uintptr_t>(out_msg),  //
                  static_cast<uint64_t>(timeout_ticks));
}

inline int64_t IpcReply(const int64_t port,           //
                        const uint64_t token,         //
                        const void* const reply_buf,  //
                        const int64_t reply_size,     //
                        const int64_t status) {
  return Syscall5(kSyscallIpcReply,                        //
                  static_cast<uint64_t>(port),             //
                  token,                                   //
                  reinterpret_cast<uintptr_t>(reply_buf),  //
                  static_cast<uint64_t>(reply_size),       //
                  static_cast<uint64_t>(status));
}

// Returns the owner's status; writes the reply's size to `*out_reply_size`
// unless it is null.
inline int64_t IpcCall(const int64_t port,         //
                       const void* const req_buf,  //
                       const int64_t req_size,     //
                       void* const reply_buf,      //
                       const int64_t reply_cap,    //
                       int64_t* const out_reply_size = nullptr) {
  return Syscall6(kSyscallIpcCall,                         //
                  static_cast<uint64_t>(port),             //
                  reinterpret_cast<uintptr_t>(req_buf),    //
                  static_cast<uint64_t>(req_size),         //
                  reinterpret_cast<uintptr_t>(reply_buf),  //
                  static_cast<uint64_t>(reply_cap),        //
                  reinterpret_cast<uintptr_t>(out_reply_size));
}

// Maps `size` bytes of zeroed anonymous memory and returns their address.
inline StatusOr<uintptr_t> MemMap(const int64_t size) {
  const int64_t rc = Syscall(kSyscallMemMap, static_cast<uint64_t>(size));
  if (SyscallIsError(rc)) {
    return Error(rc);
  }
  return static_cast<uintptr_t>(rc);
}

inline int64_t StringLength(const char* const text) {
  int64_t length = 0;
  while (text[length] != '\0') {
    ++length;
  }
  return length;
}

// Filesystem system calls (syscall_abi.h). Each call that takes a path has an
// overload for a null-terminated `path` (a null `path` makes the call fail
// with `kSyscallErrorFault`). The deleted overloads catch calls that leave out
// an argument and would otherwise convert a path length to flags or a null
// pointer.
inline int64_t Open(const char* const path,  //
                    const int64_t path_len,  //
                    const uint64_t flags) {
  return Syscall(kSyscallOpen,                       //
                 reinterpret_cast<uintptr_t>(path),  //
                 static_cast<uint64_t>(path_len),    //
                 flags);
}

inline int64_t Open(const char* const path, const uint64_t flags) {
  if (path == nullptr) {
    return Open(nullptr, 1, flags);
  }
  return Open(path, StringLength(path), flags);
}

int64_t Open(const char* path, int flags) = delete;
int64_t Open(const char* path, int64_t path_len) = delete;

inline int64_t Close(const int64_t fd) {
  return Syscall(kSyscallClose, static_cast<uint64_t>(fd));
}

inline int64_t FileRead(const int64_t fd,    //
                        void* const buffer,  //
                        const int64_t size) {
  return Syscall(kSyscallFileRead,                     //
                 static_cast<uint64_t>(fd),            //
                 reinterpret_cast<uintptr_t>(buffer),  //
                 static_cast<uint64_t>(size));
}

inline int64_t FileWrite(const int64_t fd,          //
                         const void* const buffer,  //
                         const int64_t size) {
  return Syscall(kSyscallFileWrite,                    //
                 static_cast<uint64_t>(fd),            //
                 reinterpret_cast<uintptr_t>(buffer),  //
                 static_cast<uint64_t>(size));
}

inline int64_t Seek(const int64_t fd,      //
                    const int64_t offset,  //
                    const int64_t whence) {
  return Syscall(kSyscallSeek,                   //
                 static_cast<uint64_t>(fd),      //
                 static_cast<uint64_t>(offset),  //
                 static_cast<uint64_t>(whence));
}

inline int64_t Stat(const char* const path,  //
                    const int64_t path_len,  //
                    FsStat* const out_stat) {
  return Syscall(kSyscallStat,                       //
                 reinterpret_cast<uintptr_t>(path),  //
                 static_cast<uint64_t>(path_len),    //
                 reinterpret_cast<uintptr_t>(out_stat));
}

inline int64_t Stat(const char* const path, FsStat* const out_stat) {
  if (path == nullptr) {
    return Stat(nullptr, 1, out_stat);
  }
  return Stat(path, StringLength(path), out_stat);
}

int64_t Stat(const char* path, int64_t path_len) = delete;
int64_t Stat(const char* path, int path_len) = delete;

inline int64_t ReadDir(const int64_t fd,               //
                       FsDirEntry* const out_entries,  //
                       const int64_t max_entries) {
  return Syscall(kSyscallReadDir,                           //
                 static_cast<uint64_t>(fd),                 //
                 reinterpret_cast<uintptr_t>(out_entries),  //
                 static_cast<uint64_t>(max_entries));
}

inline int64_t MkDir(const char* const path, const int64_t path_len) {
  return Syscall(kSyscallMkDir,                      //
                 reinterpret_cast<uintptr_t>(path),  //
                 static_cast<uint64_t>(path_len));
}

inline int64_t MkDir(const char* const path) {
  if (path == nullptr) {
    return MkDir(nullptr, 1);
  }
  return MkDir(path, StringLength(path));
}

inline int64_t Unlink(const char* const path, const int64_t path_len) {
  return Syscall(kSyscallUnlink,                     //
                 reinterpret_cast<uintptr_t>(path),  //
                 static_cast<uint64_t>(path_len));
}

inline int64_t Unlink(const char* const path) {
  if (path == nullptr) {
    return Unlink(nullptr, 1);
  }
  return Unlink(path, StringLength(path));
}

inline int64_t Rename(const char* const old_path,  //
                      const int64_t old_len,       //
                      const char* const new_path,  //
                      const int64_t new_len) {
  return Syscall4(kSyscallRename,                         //
                  reinterpret_cast<uintptr_t>(old_path),  //
                  static_cast<uint64_t>(old_len),         //
                  reinterpret_cast<uintptr_t>(new_path),  //
                  static_cast<uint64_t>(new_len));
}

inline int64_t Rename(const char* const old_path, const char* const new_path) {
  return Rename(old_path,                                          //
                old_path == nullptr ? 1 : StringLength(old_path),  //
                new_path,                                          //
                new_path == nullptr ? 1 : StringLength(new_path));
}

// Boot module system calls (syscall_abi.h), which only drivers may make.
inline int64_t GetBootModuleCount() {
  return Syscall(kSyscallBootModuleInfo, 0, 0);
}

inline StatusOr<BootModuleInfo> GetBootModuleInfo(const int64_t index) {
  BootModuleInfo info{};
  const int64_t rc = Syscall(kSyscallBootModuleInfo,        //
                             static_cast<uint64_t>(index),  //
                             reinterpret_cast<uintptr_t>(&info));
  if (SyscallIsError(rc)) {
    return rc;
  }
  return info;
}

// Copies up to `size` bytes of boot module `index`, from byte `offset` on, to
// `buffer`, and returns their number (0 at the end of the module), or an
// error.
inline int64_t ReadBootModule(const int64_t index,   //
                              const int64_t offset,  //
                              void* const buffer,    //
                              const int64_t size) {
  return Syscall4(kSyscallBootModuleRead,               //
                  static_cast<uint64_t>(index),         //
                  static_cast<uint64_t>(offset),        //
                  reinterpret_cast<uintptr_t>(buffer),  //
                  static_cast<uint64_t>(size));
}

// Returns the index of the first boot module whose name is the
// null-terminated `name`, `kSyscallErrorNoEntry` if no module has that name,
// or the error of a boot module system call.
inline int64_t FindBootModule(const char* const name) {
  const int64_t count = GetBootModuleCount();
  if (SyscallIsError(count)) {
    return count;
  }
  for (int64_t index = 0; index < count; ++index) {
    const StatusOr<BootModuleInfo> info = GetBootModuleInfo(index);
    if (!info.ok()) {
      return info.status();
    }
    int64_t i = 0;
    while (i < kBootModuleMaxNameLength && name[i] != '\0' &&
           info->name[i] == name[i]) {
      ++i;
    }
    if (info->name[i] == name[i]) {
      return index;
    }
  }
  return kSyscallErrorNoEntry;
}

// Each user program defines either `Main(int argc, char** argv)` (command-line
// programs in `/bin/`) or `Main(uint64_t arg)` (drivers, `init`, `sh` and
// `usertest`). The entry point (user/crt.cpp) calls whichever one the program
// defines and exits with the status that it returns.
int64_t Main(uint64_t arg);
int64_t Main(int argc, char** argv);

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
