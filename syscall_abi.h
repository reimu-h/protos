#ifndef PROTOS_SYSCALL_ABI_H_
#define PROTOS_SYSCALL_ABI_H_

#include <cstddef>
#include <cstdint>
#include <type_traits>

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
  // It called Exit(); `UserProgramResult::exit_status` holds the status.
  kExited = 0,
  // It raised a CPU exception, which killed it; the `fault_*` fields of
  // `UserProgramResult` describe the exception.
  kFaulted = 1,
  // `UserProcess::Kill()` killed it.
  kKilled = 2,
  // It never ran: its executable is invalid (elf.h) or memory ran out.
  kLoadFailed = 3,
};

// How a user program ended, as Wait() writes it to user memory. It has no
// implicit padding, which would carry stale kernel bytes to user space: the
// gaps are reserved fields, which are always zero (see the static_asserts
// below).
struct UserProgramResult {
  UserProgramOutcome outcome = UserProgramOutcome::kLoadFailed;
  uint8_t reserved0[7] = {};
  int64_t exit_status = 0;
  // The exception vector (idt.h), error code and the address of the faulting
  // instruction.
  int fault_vector = -1;
  uint32_t reserved1 = 0;
  uint64_t fault_error_code = 0;
  uint64_t fault_rip = 0;
  // For page faults, the address whose access faulted (CR2), else 0.
  uint64_t fault_address = 0;
};
static_assert(sizeof(UserProgramResult) == 48);
static_assert(alignof(UserProgramResult) == 8);
static_assert(offsetof(UserProgramResult, reserved0) == 1);
static_assert(offsetof(UserProgramResult, exit_status) == 8);
static_assert(offsetof(UserProgramResult, fault_vector) == 16);
static_assert(offsetof(UserProgramResult, reserved1) == 20);
static_assert(offsetof(UserProgramResult, fault_error_code) == 24);
static_assert(offsetof(UserProgramResult, fault_rip) == 32);
static_assert(offsetof(UserProgramResult, fault_address) == 40);
// Every byte belongs to a field.
static_assert(std::has_unique_object_representations_v<UserProgramResult>);

// Type of a PCI Base Address Register (`PciBarInfo::type`).
enum class PciBarInfoType : uint8_t {
  // Not implemented (or not decodable), or the upper 32-bit slot of the 64-bit
  // memory BAR at the preceding index.
  kNone = 0,
  // 32-bit memory-space BAR (MMIO, mapped below 4 GiB).
  kMemory32 = 1,
  // 64-bit memory-space BAR (MMIO, consumes two consecutive BAR slots;
  // recorded at the lower index).
  kMemory64 = 2,
  // x86 I/O-port BAR (`in`/`out` port space).
  kIo = 3,
};

// One Base Address Register of a PCI function (`PciDeviceInfo::bars`).
struct PciBarInfo {
  PciBarInfoType type = PciBarInfoType::kNone;
  // 1 if prefetchable (memory BARs), else 0.
  uint8_t prefetchable = 0;
  uint8_t reserved[6] = {};
  // Physical (or I/O port) base address assigned by firmware without flag bits,
  // or 0 if unassigned.
  uint64_t phys_base = 0;
  // Size in bytes (a power of two), or 0 if unassigned or not sized (only
  // claimable functions have their BARs sized).
  int64_t size = 0;
};
static_assert(sizeof(PciBarInfo) == 24);
static_assert(alignof(PciBarInfo) == 8);
static_assert(std::has_unique_object_representations_v<PciBarInfo>);

// Description of one PCI function, as PciInfo() writes it to user memory. Like
// `UserProgramResult`, it has no implicit padding: `reserved` is always zero.
struct PciDeviceInfo {
  // PCI location: bus 0..255, device 0..31, function 0..7.
  uint8_t bus = 0;
  uint8_t device = 0;
  uint8_t function = 0;
  // Header Type register: layout in bits 6:0 (0 = endpoint, 1 = PCI-to-PCI
  // bridge, 2 = CardBus) and multi-function flag (of function 0) in bit 7
  // (0x80).
  uint8_t header_type = 0;
  uint16_t vendor_id = 0;
  uint16_t device_id = 0;
  uint8_t class_code = 0;
  uint8_t subclass = 0;
  uint8_t prog_if = 0;
  uint8_t revision = 0;
  // 1 if a driver may claim the function with PciClaim() (header type 0 and
  // not a bridge of class 0x06), else 0.
  uint8_t claimable = 0;
  // Number of MSI vectors the function can request (1..32, a power of two), or
  // 0 if it has no MSI capability.
  uint8_t msi_vectors = 0;
  // MSI-X table size (1..2048) if the function has a valid MSI-X capability
  // whose table and PBA lie inside sized memory BARs, else 0.
  uint16_t msix_vectors = 0;
  // BAR0..5 for header type 0, BAR0..1 for header type 1. A 64-bit BAR is at
  // its lower index; the upper slot has `PciBarInfoType::kNone`.
  PciBarInfo bars[6] = {};
  // Index of the PCI-to-PCI bridge whose secondary bus the function is on, or
  // -1 on a root bus.
  int32_t parent_index = -1;
  // 1 if the MSI capability supports per-vector masking, else 0.
  uint8_t msi_masking = 0;
  uint8_t reserved[11] = {};
};
static_assert(sizeof(PciDeviceInfo) == 176);
static_assert(alignof(PciDeviceInfo) == 8);
static_assert(offsetof(PciDeviceInfo, vendor_id) == 4);
static_assert(offsetof(PciDeviceInfo, class_code) == 8);
static_assert(offsetof(PciDeviceInfo, claimable) == 12);
static_assert(offsetof(PciDeviceInfo, msix_vectors) == 14);
static_assert(offsetof(PciDeviceInfo, bars) == 16);
static_assert(offsetof(PciDeviceInfo, parent_index) == 160);
static_assert(offsetof(PciDeviceInfo, msi_masking) == 164);
static_assert(std::has_unique_object_representations_v<PciDeviceInfo>);

// Exit(status): ends the program with exit status `status` (rdi, int64_t).
// Does not return.
constexpr int64_t kSyscallExit = 0;
// Write(buffer, size): writes the `size` (rsi) bytes at `buffer` (rdi) to the
// console, leaving out NUL bytes, and returns `size`. Fails with
// `kSyscallErrorInvalid` unless `0 <= size <= kSyscallWriteMaxSize`, and with
// `kSyscallErrorFault` if user code may not read all of the buffer.
constexpr int64_t kSyscallWrite = 1;
// GetCpu(): returns the index of the CPU on which the call started. The
// program may run on another CPU by the time it gets the result.
constexpr int64_t kSyscallGetCpu = 2;
// Spin(buffer, size, ticks): reads the `size` (rsi) bytes at `buffer` (rdi)
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
// with `kSyscallErrorFault` if user code may not read all of the buffer, and
// with `kSyscallErrorBusy`, once it notices, if the bytes change while the
// call reads them: the program, which waits for the call, can't change them,
// but a device can (one that writes a driver's DMA buffer, for example).
constexpr int64_t kSyscallSpin = 3;
// Read(buffer, size): reads between 1 and `size` (rsi) bytes of console input
// into `buffer` (rdi), waiting if none is available yet, and returns the
// number of bytes read (`0` when `size == 0`). If the program is killed while
// waiting, the call stops early and the program ends instead of getting a
// result. Fails with `kSyscallErrorInvalid` unless
// `0 <= size <= kSyscallReadMaxSize`, and with `kSyscallErrorFault` if
// `size > 0` and user code may not write all of the buffer.
constexpr int64_t kSyscallRead = 4;
// Spawn(name, name_len, arg): starts the embedded user program whose name is
// the `name_len` (rsi) bytes at `name` (rdi) in a new child user process with
// `arg` (rdx) in its rdi, and returns the child's process ID
// (`1 <= pid <= kMaxUserChildProcesses`). Fails with `kSyscallErrorInvalid`
// unless `1 <= name_len <= kSyscallSpawnMaxNameLength`, the name matches an
// embedded user program, a child slot is free, and the child process can be
// started, and with `kSyscallErrorFault` if user code may not read all of
// `[name, name + name_len)`.
constexpr int64_t kSyscallSpawn = 5;
// Wait(pid, result): waits until child process `pid` (rdi) of the calling
// process has ended, writes its `UserProgramResult` to `result` (rsi),
// reclaims the child slot, and returns 0. If the calling process is killed
// while waiting, the call stops early and the calling process ends as killed.
// Fails with `kSyscallErrorInvalid` unless `1 <= pid <= kMaxUserChildProcesses`
// and `pid` names an active child of the calling process, and with
// `kSyscallErrorFault` if user code may not write `sizeof(UserProgramResult)`
// bytes at `result`.
constexpr int64_t kSyscallWait = 6;
// SpawnDriver(name, name_len, arg): like Spawn(), but grants the child
// `kUserCapDriver` (device-driver capability). Requires `kUserCapSpawnDrivers`
// on the caller; otherwise fails with `kSyscallErrorPermission`.
constexpr int64_t kSyscallSpawnDriver = 7;
// PciInfo(index, out_info): returns the number of PCI functions when
// `out_info == 0 && index == 0`; otherwise writes `PciDeviceInfo` for `index`
// to `out_info` and returns the number of PCI functions. Driver-only.
constexpr int64_t kSyscallPciInfo = 8;
// PciClaim(index): claims PCI function `index` for the calling driver process
// and returns 0 on success. Fails with `kSyscallErrorNoDevice` if `index` is
// out of range, `kSyscallErrorInvalid` if the function is not claimable,
// `kSyscallErrorBusy` if another process owns it, or `kSyscallErrorNoMemory` if
// the caller already holds `kSyscallMaxClaims` claims. Driver-only.
constexpr int64_t kSyscallPciClaim = 9;
// PciBusMaster(index, enable): enables (`enable == 1`) or disables
// (`enable == 0`) Bus Master on claimed PCI function `index` and returns 0 on
// success. Fails with `kSyscallErrorInvalid` unless `enable` is 0 or 1,
// `kSyscallErrorNoDevice` if `index` is out of range, or `kSyscallErrorAccess`
// if the calling process does not own `index`. Driver-only.
constexpr int64_t kSyscallPciBusMaster = 10;
// MmioMap(phys_addr, size): maps `[phys_addr, phys_addr + size)` from a memory
// BAR of a claimed PCI function into the driver's user MMIO window (UC+NX) and
// returns the user virtual address. Driver-only.
constexpr int64_t kSyscallMmioMap = 11;
// DmaAlloc(size, out_phys, flags): allocates `size` bytes of contiguous zeroed
// physical RAM, writes the physical base to `*out_phys`, maps the pages into
// the driver's user DMA window (WB+NX), and returns the user virtual address.
// Driver-only.
constexpr int64_t kSyscallDmaAlloc = 12;
// IrqAlloc(index, count, mode): allocates `count` interrupt vectors for
// claimed PCI function `index` using `mode` (`kSyscallIrqMode*`) and returns
// the starting bit index `b` (`0..31`) in the process's `IrqWait` mask.
// Driver-only.
constexpr int64_t kSyscallIrqAlloc = 13;
// IrqWait(mask, timeout_ticks): waits for any interrupt bit in `mask` or
// `timeout_ticks` timer ticks, atomically clearing and returning the fired
// bits (`0` on timeout). Driver-only.
constexpr int64_t kSyscallIrqWait = 14;
// ConsolePush(buf, size): pushes up to `size` bytes of console input into the
// kernel console ring and returns the number of bytes pushed. Driver-only.
constexpr int64_t kSyscallConsolePush = 15;
// GetTicks(): returns the monotonic 250 Hz timer tick counter. Available to
// all processes.
constexpr int64_t kSyscallGetTicks = 16;

// The most bytes that one Write() writes.
constexpr int64_t kSyscallWriteMaxSize = 4096;
// The most bytes that one Spin() reads, and the most timer ticks that it lasts
// (1000000 ticks are more than an hour at 250 Hz).
constexpr int64_t kSyscallSpinMaxSize = 4096;
constexpr int64_t kSyscallSpinMaxTicks = 1000000;
// The most bytes that one Read() reads.
constexpr int64_t kSyscallReadMaxSize = 4096;
// The longest program name that Spawn() accepts, and the most unwaited child
// processes that one process may have at a time.
constexpr int64_t kSyscallSpawnMaxNameLength = 64;
constexpr int64_t kMaxUserChildProcesses = 8;

// Driver syscall limits, flags and modes.
constexpr int64_t kSyscallConsolePushMaxSize = 256;
constexpr int64_t kSyscallMmioMaxSize = 16LL << 20;
constexpr int64_t kSyscallDmaMaxSize = 1LL << 20;
constexpr int64_t kSyscallDmaMaxTotalSize = 16LL << 20;
constexpr int kSyscallMaxClaims = 16;

constexpr uint64_t kSyscallDmaBelow4GiB = 1ULL << 0;
constexpr uint64_t kSyscallDmaFlagsMask = kSyscallDmaBelow4GiB;

constexpr int64_t kSyscallIrqModeAuto = 0;
constexpr int64_t kSyscallIrqModeMsi = 1;
constexpr int64_t kSyscallIrqModeMsix = 2;
constexpr int64_t kSyscallIrqMaxCount = 32;
constexpr int64_t kSyscallIrqWaitMaxTicks = 1000000;
constexpr int64_t kSyscallTicksPerSecond = 250;

// Errors, with the numbers of the corresponding Linux errno values.
constexpr int64_t kSyscallErrorPermission = -1;  // EPERM: operation not allowed
constexpr int64_t kSyscallErrorNoMemory = -12;   // ENOMEM: out of memory
constexpr int64_t kSyscallErrorAccess = -13;     // EACCES: permission denied
constexpr int64_t kSyscallErrorFault = -14;      // EFAULT: bad address
constexpr int64_t kSyscallErrorBusy = -16;       // EBUSY: resource busy
constexpr int64_t kSyscallErrorNoDevice = -19;   // ENODEV: no such device
constexpr int64_t kSyscallErrorInvalid = -22;    // EINVAL: invalid argument
constexpr int64_t kSyscallErrorNoSys = -38;      // ENOSYS: unknown system call

// Process capability bits (user_mode.h).
constexpr uint64_t kUserCapNone = 0;
constexpr uint64_t kUserCapDriver = 1ULL << 0;
constexpr uint64_t kUserCapSpawnDrivers = 1ULL << 1;

// Bit flag in `init`'s `arg` (`rdi` at entry) instructing `init` to spawn
// user-space hardware drivers via `SpawnDriver` before starting the shell.
constexpr uint64_t kInitArgStartDrivers = 1ULL << 0;

// Standard argument modes for driver processes (`Main(uint64_t arg)`).
// - `kDriverArgNormal` (0): normal interrupt-driven daemon loop (exits with 0
//   if no matching hardware devices are present).
// - `kDriverArgExitAfterReady` (1): claims all matching devices, completes
//   two-phase hardware bring-up and one initial interrupt poll, then exits
//   with status 0 (or non-zero if any discovered device failed).
// - `kDriverArgFaultAfterReady` (2): same as `kDriverArgExitAfterReady`, but
//   executes `ud2` (`#UD`) after all controllers are running with Bus Master,
//   MMIO, DMA, and MSI/MSI-X active.
constexpr uint64_t kDriverArgNormal = 0;
constexpr uint64_t kDriverArgExitAfterReady = 1;
constexpr uint64_t kDriverArgFaultAfterReady = 2;

// Exit status of a driver program started without `kUserCapDriver`, that is by
// `Spawn()` (as `sh`'s `run <program>` does) rather than by `SpawnDriver()`:
// its driver system calls fail with `kSyscallErrorPermission`.
constexpr int64_t kDriverExitNotDriver = 3;

// True if `result` is in the system call error range `[-4095, -1]`. User
// virtual addresses in the higher half (`>= 0xFFFF800000000000`) cast to
// negative `int64_t` values `< -4095`, so only `SyscallIsError()` distinguishes
// errors from valid pointer results.
constexpr bool SyscallIsError(const int64_t result) {
  return result >= -4095 && result < 0;
}

// True if system call `number` requires `kUserCapDriver` (`8..15`).
constexpr bool SyscallIsDriverOnly(const int64_t number) {
  return number >= kSyscallPciInfo && number <= kSyscallConsolePush;
}

// True if a process with `capabilities` is allowed to invoke system call
// `number` (before any per-syscall argument validation).
constexpr bool MayMakeSyscall(const uint64_t capabilities,
                              const int64_t number) {
  if (number == kSyscallSpawnDriver) {
    return (capabilities & kUserCapSpawnDrivers) != 0;
  }
  if (SyscallIsDriverOnly(number)) {
    return (capabilities & kUserCapDriver) != 0;
  }
  return true;
}

// The checksum of no bytes; see `SyscallSpinChecksum()`.
constexpr int64_t kSyscallSpinChecksumInit = 0x4BF29CE484222325;

// Returns Spin()'s checksum of the bytes whose checksum is `checksum`,
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
