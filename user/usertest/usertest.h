#ifndef PROTOS_USER_USERTEST_USERTEST_H_
#define PROTOS_USER_USERTEST_USERTEST_H_

#include <cstdint>

// The interface of user/usertest/usertest.cpp, a user program that the boot
// self-tests (selftest/boot_selftest.cpp) run in many ways to check user mode:
// system calls, faults, preemption, kills and the x87/SSE/AVX state of
// processes.

namespace protos::usertest {

// What the program does, selected by the low 8 bits of its argument (`Arg()`).
// In every mode but `kFpuInitialState`, the program first checks that it
// started with the architectural initial x87/SSE/AVX state, and exits with
// `kFpuInitialStateMismatchStatusBase` plus the number of the failed check if
// not.
enum class Mode : uint8_t {
  // Exits with `kExitStatus`.
  kExit,
  // Checks that it runs in user mode with interrupts enabled, and the
  // results of system calls with good and bad arguments. Exits with 0 if all
  // checks pass, else with the number of the first failed check.
  kSyscalls,
  // Each of the following raises a CPU exception, which kills the program:
  // `cli` (#GP).
  kPrivilegedInstruction,
  // Reads kernel memory at `kKernelAddress` (#PF).
  kReadKernelMemory,
  // Writes to its read-only data (#PF).
  kWriteReadOnlyData,
  // Calls code in its data (#PF if the CPUs support no-execute pages, else
  // the code returns and the program exits with 0).
  kExecuteData,
  // `ud2` (#UD).
  kInvalidOpcode,
  // `int kKernelOnlyVector`, a vector that user code may not raise (#GP).
  kKernelInterrupt,
  // Recurses until its stack overflows into the guard page below (#PF).
  kStackOverflow,
  // Computes forever, without yielding, and checks that its registers, stack
  // and data stay intact, making a get_cpu() system call every
  // `kSpinRoundIterations` iterations, until the kernel kills it (the first
  // system call comes before the first round). Exits with
  // `kSpinCorruptedStatus` if a check fails.
  kSpin,
  // Makes the spin() system calls (syscall_abi.h) that its `KernelSpinArg()`
  // argument asks for. Each call reads a buffer with new contents, size and
  // offset, with values that depend on the seed and differ from register to
  // register in every other general-purpose register. After each call the
  // program checks that the kernel preserved every register but rax and
  // returned the buffer's checksum, and that the buffer, the stack and the
  // data are intact. Exits with 0 after the last call (never if the argument
  // asks for 0 calls: then it runs until the kernel kills it), or with the
  // `kKernelSpin*Status` of the first failed check.
  kKernelSpin,
  // Computes forever without ever making a system call, until the kernel
  // kills it, with values that depend on the seed and differ from register to
  // register in its general-purpose registers, and checks that its registers,
  // stack and data stay intact. Exits with `kSpinRegistersChangedStatus` or
  // `kSpinCorruptedStatus` if a check fails.
  kSpinWithoutSyscalls,
  // Holds values that depend on the seed and change every round in all
  // x87/MMX registers (as 80-bit x87 values and, in every other round, as
  // 64-bit MMX values), in XMM0-XMM15, in the upper halves of YMM0-YMM15 if
  // the kernel enabled the AVX state (XCR0), and in the x87 control word and
  // MXCSR, which never hold their initial values, and checks them
  // continuously, until the kernel kills it. Each round loads its own values,
  // saves the registers, makes `kFpuHoldSyscalls` get_cpu() system calls with
  // integer-only computation before each, saves the registers again and
  // checks that both saves hold the loaded values. Exits with the
  // `kFpuCorrupted*Status` of the first difference, or with
  // `kFpuLayoutUnsupportedStatus`.
  kFpuHold,
  // Loads values that depend on the seed and differ from the initial state
  // into the same registers as `kFpuHold`, then makes get_cpu() system calls
  // with integer-only computation in between, without touching them again,
  // until the kernel kills it. Exits with `kFpuLayoutUnsupportedStatus` if
  // it can't load the AVX state.
  kFpuDirty,
  // Checks that the program started with the architectural initial x87, SSE
  // and AVX state, as the entry point (user/crt.cpp) saved it before any
  // compiled code ran. Exits with 0 if so, else with the number of the first
  // failed check: 1 x87 control word not 0x37F, 2 x87 status word not 0,
  // 3 x87 registers not all empty, 4 last x87 instruction or operand pointer
  // or opcode not 0, 5 ST0-ST7 not 0, 6 MXCSR not 0x1F80, 7 XMM0-XMM15 not 0,
  // 8 upper halves of YMM0-YMM15 not 0.
  kFpuInitialState,
  // Exits with the state components that user code may save with XSAVE (the
  // low 32 bits of XCR0), or with 0 if the kernel did not enable XSAVE
  // (CPUID.1:ECX.OSXSAVE clear).
  kFpuFeatures,
  // Makes system calls with buffers that user code may not read, or with
  // unusual buffers that it may read, and invalid arguments, and checks each
  // result (`kBufferCheckCount` checks): a null, kernel, non-canonical,
  // unmapped, wrapping or guard-page buffer, or one that runs past the top of
  // the stack, gets `kSyscallErrorFault`; invalid sizes and ticks get
  // `kSyscallErrorInvalid`; read-only data, code, unaligned buffers, buffers
  // that cross a page boundary and the lowest stack page get their checksum;
  // the kernel preserves the registers around failing calls; and RFLAGS.AC,
  // which user code may set (CR0.AM is clear, so it has no effect there),
  // stays set across system calls and timer interrupts. Exits with 0 if all
  // checks pass, else with the number of the first failed check.
  kBufferChecks,
  // Runs rounds that each load values that depend on the seed and the round
  // into the same registers as `kFpuHold` (in every other round MMX values
  // into MM0-MM7), make one spin() system call of the argument's ticks on a
  // buffer with new contents, and check that the call preserved every
  // register, returned the buffer's checksum and left the buffer alone. The
  // argument is a `FpuLongSyscallArg()`. Exits with 0 after the last round
  // (never if it asks for 0 rounds: then it runs until the kernel kills it),
  // or with the `kFpuCorrupted*Status` or `kFpuLongSyscall*Status` of the
  // first failed check, or with `kFpuLayoutUnsupportedStatus`.
  kFpuLongSyscall,
  // Checks that the x87 status word is 0, unmasks the x87 zero-divide
  // exception and divides by zero, which leaves the exception pending, then
  // computes with integer instructions only, making `kX87PendingRounds`
  // get_cpu() system calls in between, each after a round of at least the TSC
  // cycles that its argument asks for (`X87PendingArg()`), and finally
  // executes FWAIT, which must raise the pending exception (#MF, which kills
  // the program). Exits with a `kX87Pending*Status` if a check fails or FWAIT
  // returns.
  kX87PendingException,
  // Validates read() error cases (invalid sizes -> kSyscallErrorInvalid,
  // null/kernel/non-canonical/unmapped/wrapping/guard-page/read-only user
  // buffers -> kSyscallErrorFault), then reads `arg >> 8` bytes from the
  // console (or blocks in read() until killed if `arg >> 8` is 0) and exits
  // with `SyscallSpinChecksum()` of the bytes read.
  kConsoleRead,
  // Exercises spawn() and wait() according to the sub-mode in `arg >> 8`
  // (`SpawnTestSubmode`):
  //   0 (`kBasicAndErrors`): validates bad lengths/names/PIDs
  //     (kSyscallErrorInvalid) and bad/read-only pointers
  //     (kSyscallErrorFault), spawns child usertest processes that exit,
  //     fault with #UD, and fault with #PF, tests child slot exhaustion and
  //     double-wait rejection, and exits with 0.
  //   1 (`kBlockInWait`): spawns a spinning child and blocks in wait() until
  //     the parent is killed.
  //   2 (`kOrphanOnExit`): spawns two spinning children and exits with 0
  //     without waiting for them.
  //   3 (`kOrphanOnFault`): spawns a spinning child and faults (#UD) without
  //     waiting for it.
  kSpawnAndWait,
  // Calls every privileged system call (`7..16`) from an unprivileged process
  // (`kUserCapNone`), checks that each returns `kSyscallErrorPermission`, that
  // out-of-range system calls (`17`, `-1`) return `kSyscallErrorNoSys`, and
  // that a child spawned via `spawn()` also gets `kSyscallErrorPermission`.
  kDriverDenied,
  // Probes capability separation (`DriverSubmode(arg)`):
  //   0 (`kProbeDriverRole`): run as `kUserCapDriver`; checks `pci_info` works,
  //     `spawn_driver` returns `kSyscallErrorPermission`, reserved syscall `16`
  //     returns `kSyscallErrorNoSys`, and a child spawned via `spawn()` runs
  //     `kDriverDenied` with `kUserCapNone` and exits with 0.
  //   1 (`kProbeSpawnerRole`): run as `kUserCapSpawnDrivers`; checks driver
  //     syscalls `8..16` return `kSyscallErrorPermission`, then calls
  //     `spawn_driver("usertest", ...)` with `kProbeDriverRole` and waits for
  //     the driver child to exit with 0.
  kDriverCapProbe,
  // Claims PCI device `DriverDeviceIndex(arg)`, enables Bus Master, maps BAR
  // `DriverBarIndex(arg)`, and according to `DriverSubmode(arg)`
  // (`DriverHoldSubmode`):
  //   0 (`kHoldSpin`): also allocates 4 pages of DMA memory, allocates device
  //     IRQs (if supported), spawns a spinning child, and blocks in
  //     `irq_wait(0, kSyscallIrqWaitMaxTicks)` until killed.
  //   1 (`kHoldExit`): sets up the same 6 resources and exits with 0.
  //   2 (`kHoldFault`): sets up the same 6 resources and raises #UD.
  //   3 (`kHoldClaimOnly`): claims the device, enables Bus Master, maps the BAR
  //     (if valid), and blocks in `irq_wait(0, kSyscallIrqWaitMaxTicks)` until
  //     killed.
  kDriverHold,
  // Exercises PCI info, claim, and foreign/unowned device rejection on
  // `DriverDeviceIndex(arg)` / `DriverBarIndex(arg)` according to
  // `DriverSubmode(arg)` (`DriverForeignSubmode`).
  kDriverForeign,
  // Claims `DriverDeviceIndex(arg)`, maps BAR `DriverBarIndex(arg)`, verifies
  // virtual address range, page offset, idempotency, 32-bit MMIO read, and
  // invalid BAR arguments, and (when `DriverSubmode(arg) == 1`) blocks in
  // `irq_wait` so the boot self-test can inspect live PTEs before killing it.
  kDriverMmioChecks,
  // Claims `DriverDeviceIndex(arg)`, maps BAR `DriverBarIndex(arg)`, and faults
  // on purpose (`DriverMmioFaultSubmode`):
  //   0 (`kFaultExecuteMmio`): jumps to the mapped MMIO address (#PF NX fetch).
  //   1 (`kFaultReadWithheldMsix`): reads from the withheld MSI-X table page
  //     offset inside the BAR (#PF non-present).
  kDriverMmioFault,
  // Validates `dma_alloc` error cases, allocates 4-page and 1-page DMA buffers,
  // verifies zero-initialization, virtual/physical alignment, and distinct
  // addresses, writes test patterns, and either blocks in `irq_wait` (`submode
  // == 0`) so the boot self-test can inspect PTEs and physical contiguity or
  // jumps to the DMA buffer (`submode == 1`) to trigger an NX instruction-fetch
  // #PF.
  kDriverDmaChecks,
  // Validates `console_push` error cases and pushes `"drv_in!\n"` into the
  // console input ring.
  kDriverConsolePush,
  // Exercises `irq_alloc` and `irq_wait` on `DriverDeviceIndex(arg)` according
  // to `DriverSubmode(arg)` (`DriverIrqSubmode`).
  kDriverIrqWait,
  // Claims the xHCI controller `DriverDeviceIndex(arg)` and does what usbkbd's
  // bring-up does before it enables Bus Master: takes the controller over
  // from firmware (USB Legacy Support handoff, firmware SMIs disabled), halts
  // it if it runs and resets it (HCRST), all without touching Bus Master.
  // Until a driver starts it again, the controller then neither runs nor
  // accesses memory, whatever firmware or a killed driver left in it. Exits
  // with 0 if the controller was halted already, with
  // `kXhciResetWasRunningStatus` if it ran and was halted, or with another
  // `kXhciReset*Status` if a step failed.
  kDriverXhciReset,
};

enum class SpawnTestSubmode : uint8_t {
  kBasicAndErrors = 0,
  kBlockInWait = 1,
  kOrphanOnExit = 2,
  kOrphanOnFault = 3,
};

enum class DriverCapProbeSubmode : uint8_t {
  kProbeDriverRole = 0,
  kProbeSpawnerRole = 1,
};

enum class DriverHoldSubmode : uint8_t {
  kHoldSpin = 0,
  kHoldExit = 1,
  kHoldFault = 2,
  kHoldClaimOnly = 3,
};

enum class DriverForeignSubmode : uint8_t {
  kUnclaimedDevice = 0,
  kOwnedByOtherProcess = 1,
  kValidationAndClaimToggle = 2,
};

enum class DriverMmioFaultSubmode : uint8_t {
  kFaultExecuteMmio = 0,
  kFaultReadWithheldMsix = 1,
};

enum class DriverIrqSubmode : uint8_t {
  kIrqValidationAndTimeout = 0,
  kIrqPrePendingAndSelectiveMask = 1,
  kIrqBlockingWakeup = 2,
  kIrqKillWhileBlocked = 3,
};

// The exit statuses of `kDriverXhciReset` besides 0: the controller ran and
// was halted before its reset; the device can't be claimed or is not an xHCI
// with a memory BAR 0; its registers read all-ones (or don't map); its
// capability registers are invalid; it did not halt; HCRST or then CNR did
// not clear, or it ran after the reset.
constexpr int64_t kXhciResetWasRunningStatus = 1;
constexpr int64_t kXhciResetNotXhciStatus = 2;
constexpr int64_t kXhciResetNotDecodingStatus = 3;
constexpr int64_t kXhciResetBadCapabilitiesStatus = 4;
constexpr int64_t kXhciResetHaltTimeoutStatus = 5;
constexpr int64_t kXhciResetTimeoutStatus = 6;

constexpr int64_t kExitStatus = -0x123456789AB;
// The vector that `kKernelInterrupt` raises: the wakeup IPI's (idt.h), whose
// gate admits only the kernel.
constexpr uint8_t kKernelOnlyVector = 0x21;
constexpr int64_t kSpinCorruptedStatus = 99;
constexpr int kSpinRoundIterations = 4096;
// The kernel image starts at 1 MiB (linker.ld).
constexpr uint64_t kKernelAddress = 0x100000;

// The exit statuses of `kKernelSpin` when a spin() call changed a register
// other than rax, returned an error or a wrong checksum, or changed the
// buffer, or when the program's stack or data changed.
constexpr int64_t kKernelSpinRegistersChangedStatus = 110;
constexpr int64_t kKernelSpinErrorStatus = 111;
constexpr int64_t kKernelSpinChecksumStatus = 112;
constexpr int64_t kKernelSpinBufferChangedStatus = 113;
constexpr int64_t kKernelSpinCorruptedStatus = 114;
// The exit status of `kSpinWithoutSyscalls` when a register changed.
constexpr int64_t kSpinRegistersChangedStatus = 115;

// The exit statuses of `kFpuHold` when a register did not hold the value that
// the program loaded into it: the x87 control, status or tag word or the last
// x87 instruction and operand pointers and opcode; ST0-ST7 (MM0-MM7); MXCSR;
// XMM0-XMM15; the upper halves of YMM0-YMM15.
constexpr int64_t kFpuCorruptedX87ControlStatus = 101;
constexpr int64_t kFpuCorruptedX87RegistersStatus = 102;
constexpr int64_t kFpuCorruptedMxcsrStatus = 103;
constexpr int64_t kFpuCorruptedXmmStatus = 104;
constexpr int64_t kFpuCorruptedYmmStatus = 105;
// The exit status of `kFpuHold` and `kFpuDirty` when the kernel enabled the
// AVX state but CPUID puts it where the program's save areas don't hold it.
constexpr int64_t kFpuLayoutUnsupportedStatus = 106;
// The exit status of a program in any mode but `kFpuInitialState` that did not
// start with the architectural initial x87/SSE/AVX state: this plus the number
// of the first failed check of `kFpuInitialState` (131-138). Every program
// checks the state that the entry point saved (user/crt.cpp) before it runs
// its mode.
constexpr int64_t kFpuInitialStateMismatchStatusBase = 130;
// The get_cpu() system calls of each round of `kFpuHold`, and the iterations
// of integer-only computation before each of them.
constexpr int kFpuHoldSyscalls = 32;
constexpr int kFpuHoldIterations = 4096;

// Returns the argument that makes the program run `mode`, with `seed` (below
// 2^56) for `kSpin`, `kSpinWithoutSyscalls`, `kFpuHold` and `kFpuDirty`.
constexpr uint64_t Arg(const Mode mode, const uint64_t seed = 0) {
  return static_cast<uint64_t>(mode) | (seed << 8);
}

// The fields of a `kKernelSpin` argument above the mode byte: the timer ticks
// of each spin() call, the number of calls and a seed for the buffer
// contents.
constexpr int kKernelSpinTicksShift = 8;
constexpr int kKernelSpinCallsShift = 28;
constexpr int kKernelSpinSeedShift = 44;
constexpr int64_t kKernelSpinMaxTicks = (int64_t{1} << 20) - 1;
constexpr int64_t kKernelSpinMaxCalls = (int64_t{1} << 16) - 1;
constexpr uint64_t kKernelSpinMaxSeed = (uint64_t{1} << 20) - 1;

// Returns the argument that makes the program run `kKernelSpin` with `calls`
// spin() calls (0: until it is killed) of `ticks` timer ticks each, for
// `0 <= ticks <= kKernelSpinMaxTicks`, `0 <= calls <= kKernelSpinMaxCalls` and
// `seed <= kKernelSpinMaxSeed`.
constexpr uint64_t KernelSpinArg(const int64_t ticks,  //
                                 const int64_t calls,  //
                                 const uint64_t seed) {
  return Arg(Mode::kKernelSpin) |
         (static_cast<uint64_t>(ticks) << kKernelSpinTicksShift) |
         (static_cast<uint64_t>(calls) << kKernelSpinCallsShift) |
         (seed << kKernelSpinSeedShift);
}

// Return the fields of a `KernelSpinArg()` value.
constexpr int64_t KernelSpinTicks(const uint64_t arg) {
  return static_cast<int64_t>(arg >> kKernelSpinTicksShift) &
         kKernelSpinMaxTicks;
}
constexpr int64_t KernelSpinCalls(const uint64_t arg) {
  return static_cast<int64_t>(arg >> kKernelSpinCallsShift) &
         kKernelSpinMaxCalls;
}
constexpr uint64_t KernelSpinSeed(const uint64_t arg) {
  return arg >> kKernelSpinSeedShift;
}

static_assert(KernelSpinTicks(KernelSpinArg(25, 2, 7)) == 25);
static_assert(KernelSpinCalls(KernelSpinArg(25, 2, 7)) == 2);
static_assert(KernelSpinSeed(KernelSpinArg(25, 2, 7)) == 7);
static_assert(KernelSpinSeed(KernelSpinArg(kKernelSpinMaxTicks,
                                           kKernelSpinMaxCalls,
                                           kKernelSpinMaxSeed)) ==
              kKernelSpinMaxSeed);

// The checks of `kBufferChecks`: it exits with the number of the first one
// that fails (1 to `kBufferCheckCount`).
constexpr int kBufferCheckCount = 13;

// The user stack (`kUserStackTop` and `kUserStackPages` in user_mode.h, which
// the boot self-tests check against these): its top, above which nothing is
// mapped, and its pages, below which the guard page is unmapped.
constexpr uint64_t kStackTop = 0xFFFFFFFF7FFFF000;
constexpr int64_t kStackPages = 4;

// The exit statuses of `kFpuLongSyscall` besides the `kFpuCorrupted*Status`
// values (and `kFpuLayoutUnsupportedStatus`): MM0-MM7 did not hold their
// values in an MMX round; the spin() call returned an error or a wrong
// checksum, changed a general-purpose register other than rax, or changed
// the buffer. 120-129 are the end-to-end tests' range.
constexpr int64_t kFpuLongSyscallMmxStatus = 120;
constexpr int64_t kFpuLongSyscallErrorStatus = 121;
constexpr int64_t kFpuLongSyscallChecksumStatus = 122;
constexpr int64_t kFpuLongSyscallRegistersChangedStatus = 123;
constexpr int64_t kFpuLongSyscallBufferChangedStatus = 124;
// The exit statuses of `kX87PendingException`: the x87 status word was not 0
// at the start; the division by zero did not leave the exception pending
// (the status word lacks ZE or ES); FWAIT returned, so the exception that was
// pending went away.
constexpr int64_t kX87PendingInitialStatusWordStatus = 125;
constexpr int64_t kX87PendingNotRaisedStatus = 126;
constexpr int64_t kX87PendingContinuedStatus = 127;
// The get_cpu() system calls that `kX87PendingException` makes while the
// exception is pending, and the iterations of integer-only computation
// before each of them: at least kX87PendingIterations per round (2^24 in
// all, several timer ticks under QEMU's TCG, but about one on a fast CPU)
// and, where that takes less time, until the TSC has advanced by the cycles
// per round of its argument (`X87PendingArg()`), so that timer interrupts and
// task switches come while the exception is pending.
constexpr int kX87PendingRounds = 64;
constexpr int kX87PendingIterations = 1 << 18;
// The most TSC cycles per round that an `X87PendingArg()` value holds.
constexpr int64_t kX87PendingMaxRoundCycles = (int64_t{1} << 48) - 1;

// Returns the argument that makes the program run `kX87PendingException`
// with rounds of at least `round_cycles` TSC cycles each, for
// `0 <= round_cycles <= kX87PendingMaxRoundCycles`.
constexpr uint64_t X87PendingArg(const int64_t round_cycles) {
  return Arg(Mode::kX87PendingException, static_cast<uint64_t>(round_cycles));
}

// Returns the TSC cycles per round of an `X87PendingArg()` value.
constexpr int64_t X87PendingRoundCycles(const uint64_t arg) {
  return static_cast<int64_t>(arg >> 8);
}

static_assert((X87PendingArg(12345) & 0xFF) ==
              static_cast<uint64_t>(Mode::kX87PendingException));
static_assert(X87PendingRoundCycles(X87PendingArg(12345)) == 12345);
static_assert(X87PendingRoundCycles(X87PendingArg(kX87PendingMaxRoundCycles)) ==
              kX87PendingMaxRoundCycles);

// Returns the argument that makes the program run `kFpuLongSyscall` with
// `rounds` rounds (0: until it is killed) of a spin() call of `ticks` timer
// ticks each, and FPU values from `seed`, with the limits of
// `KernelSpinArg()`, whose fields `KernelSpinTicks()`, `KernelSpinCalls()`
// (the rounds) and `KernelSpinSeed()` return.
constexpr uint64_t FpuLongSyscallArg(const int64_t ticks,   //
                                     const int64_t rounds,  //
                                     const uint64_t seed) {
  return Arg(Mode::kFpuLongSyscall) |
         (static_cast<uint64_t>(ticks) << kKernelSpinTicksShift) |
         (static_cast<uint64_t>(rounds) << kKernelSpinCallsShift) |
         (seed << kKernelSpinSeedShift);
}

static_assert((FpuLongSyscallArg(8, 4, 9) & 0xFF) ==
              static_cast<uint64_t>(Mode::kFpuLongSyscall));
static_assert(KernelSpinTicks(FpuLongSyscallArg(8, 4, 9)) == 8);
static_assert(KernelSpinCalls(FpuLongSyscallArg(8, 4, 9)) == 4);
static_assert(KernelSpinSeed(FpuLongSyscallArg(8, 4, 9)) == 9);

// Encodes a driver mode argument:
//   bits  0.. 7: `mode`
//   bits  8..15: `submode`
//   bits 16..31: `device_index`
//   bits 32..39: `bar_index`
constexpr uint64_t DriverArg(const Mode mode,             //
                             const uint8_t submode = 0,   //
                             const int device_index = 0,  //
                             const int bar_index = 0) {
  return Arg(mode) | (static_cast<uint64_t>(submode) << 8) |
         (static_cast<uint64_t>(device_index & 0xFFFF) << 16) |
         (static_cast<uint64_t>(bar_index & 0xFF) << 32);
}

constexpr uint8_t DriverSubmode(const uint64_t arg) {
  return static_cast<uint8_t>((arg >> 8) & 0xFF);
}

constexpr int DriverDeviceIndex(const uint64_t arg) {
  return static_cast<int>(static_cast<int16_t>((arg >> 16) & 0xFFFF));
}

constexpr int DriverBarIndex(const uint64_t arg) {
  return static_cast<int>((arg >> 32) & 0xFF);
}

static_assert(DriverSubmode(DriverArg(Mode::kDriverHold, 2, 5, 3)) == 2);
static_assert(DriverDeviceIndex(DriverArg(Mode::kDriverHold, 2, 5, 3)) == 5);
static_assert(DriverBarIndex(DriverArg(Mode::kDriverHold, 2, 5, 3)) == 3);

}  // namespace protos::usertest

#endif  // PROTOS_USER_USERTEST_USERTEST_H_
