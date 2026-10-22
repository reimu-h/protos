#ifndef PROTOS_SYSCALL_ABI_H_
#define PROTOS_SYSCALL_ABI_H_

#include <cstddef>
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

enum class PciBarInfoType : uint8_t {
  kNone = 0,
  kMemory32 = 1,
  kMemory64 = 2,
  kIo = 3,
};

struct PciBarInfo {
  PciBarInfoType type = PciBarInfoType::kNone;
  uint8_t prefetchable = 0;
  uint8_t reserved[6] = {};
  uint64_t phys_base = 0;
  int64_t size = 0;
};
static_assert(sizeof(PciBarInfo) == 24);
static_assert(alignof(PciBarInfo) == 8);

struct PciDeviceInfo {
  uint8_t bus = 0;
  uint8_t device = 0;
  uint8_t function = 0;
  uint8_t header_type = 0;
  uint16_t vendor_id = 0;
  uint16_t device_id = 0;
  uint8_t class_code = 0;
  uint8_t subclass = 0;
  uint8_t prog_if = 0;
  uint8_t revision = 0;
  uint8_t claimable = 0;
  uint8_t msi_vectors = 0;
  uint16_t msix_vectors = 0;
  PciBarInfo bars[6] = {};
  int32_t parent_index = -1;
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
// spawn_driver(name, name_len, arg): like spawn(), but grants the child
// `kUserCapDriver` (device-driver capability). Requires `kUserCapSpawnDrivers`
// on the caller; otherwise fails with `kSyscallErrorPermission`.
constexpr int64_t kSyscallSpawnDriver = 7;
// pci_info(index, out_info): returns the number of PCI functions when
// `out_info == 0 && index == 0`; otherwise writes `PciDeviceInfo` for `index`
// to `out_info` and returns the number of PCI functions. Driver-only.
constexpr int64_t kSyscallPciInfo = 8;
// pci_claim(index): claims PCI function `index` for the calling driver process.
// Driver-only.
constexpr int64_t kSyscallPciClaim = 9;
// pci_bus_master(index, enable): enables (`enable == 1`) or disables
// (`enable == 0`) Bus Master on claimed PCI function `index`. Driver-only.
constexpr int64_t kSyscallPciBusMaster = 10;
// mmio_map(phys_addr, size): maps `[phys_addr, phys_addr + size)` from a memory
// BAR of a claimed PCI function into the driver's user MMIO window (UC+NX) and
// returns the user virtual address. Driver-only.
constexpr int64_t kSyscallMmioMap = 11;
// dma_alloc(size, out_phys, flags): allocates `size` bytes of contiguous zeroed
// physical RAM, writes the physical base to `*out_phys`, maps the pages into
// the driver's user DMA window (WB+NX), and returns the user virtual address.
// Driver-only.
constexpr int64_t kSyscallDmaAlloc = 12;
// irq_alloc(index, count, mode): allocates `count` interrupt vectors for
// claimed PCI function `index` using `mode` (`kSyscallIrqMode*`) and returns
// the starting bit index `b` (`0..31`) in the process's `irq_wait` mask.
// Driver-only.
constexpr int64_t kSyscallIrqAlloc = 13;
// irq_wait(mask, timeout_ticks): waits for any interrupt bit in `mask` or
// `timeout_ticks` timer ticks, atomically clearing and returning the fired
// bits (`0` on timeout). Driver-only.
constexpr int64_t kSyscallIrqWait = 14;
// console_push(buf, size): pushes up to `size` bytes of console input into the
// kernel console ring and returns the number of bytes pushed. Driver-only.
constexpr int64_t kSyscallConsolePush = 15;
// get_ticks(): returns the monotonic 250 Hz timer tick counter. Available to
// all processes.
constexpr int64_t kSyscallGetTicks = 16;

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
// user-space hardware drivers via `spawn_driver` before starting the shell.
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
// `spawn()` (as `sh`'s `run <program>` does) rather than by `spawn_driver()`:
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
