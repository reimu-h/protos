#ifndef PROTOS_USER_USERTEST_H_
#define PROTOS_USER_USERTEST_H_

#include <cstdint>

// The interface of user/usertest.cpp, a user program that the boot self-tests
// (selftest/boot_selftest.cpp) run in many ways to check user mode: system
// calls, faults, preemption, kills and the x87/SSE/AVX state of processes.

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
};

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

}  // namespace protos::usertest

#endif  // PROTOS_USER_USERTEST_H_
