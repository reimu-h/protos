#ifndef PROTOS_USER_USERTEST_H_
#define PROTOS_USER_USERTEST_H_

#include <cstdint>

// The interface of user/usertest.cpp, a user program that the boot self-tests
// (selftest/boot_selftest.cpp) run in many ways to check user mode: system
// calls, faults, and preemption.

namespace protos::usertest {

// What the program does, selected by the low 8 bits of its argument (`Arg()`).
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
};

constexpr int64_t kExitStatus = -0x123456789AB;
// The vector that `kKernelInterrupt` raises: the wakeup IPI's (idt.h), whose
// gate admits only the kernel.
constexpr uint8_t kKernelOnlyVector = 0x21;
constexpr int64_t kSpinCorruptedStatus = 99;
constexpr int kSpinRoundIterations = 4096;
// The kernel image starts at 1 MiB (linker.ld).
constexpr uint64_t kKernelAddress = 0x100000;

// Returns the argument that makes the program run `mode`, with `seed` (below
// 2^56) for `kSpin`.
constexpr uint64_t Arg(const Mode mode, const uint64_t seed = 0) {
  return static_cast<uint64_t>(mode) | (seed << 8);
}

}  // namespace protos::usertest

#endif  // PROTOS_USER_USERTEST_H_
