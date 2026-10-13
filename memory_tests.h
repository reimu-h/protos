#ifndef PROTOS_MEMORY_TESTS_H_
#define PROTOS_MEMORY_TESTS_H_

#include <cstdint>

namespace protos {

// Runs the boot verification suite and returns true if every check passed.
//
// Covers the physical memory manager, kernel heap, C++ runtime support,
// Multiboot parsing, paging, the RB-tree, SMP bring-up, per-CPU IDT/LAPIC timer
// state, and the task scheduler. Logs a `[TEST] <name>: PASS|FAIL` line per
// test and a final `[TEST] ALL MEMORY TESTS PASSED` or
// `[TEST] MEMORY VERIFICATION FAILED` line over UART COM1 (test_boot.sh depends
// on these markers), and a one-line summary on the display console.
//
// Preconditions: called once from `kernel_main` on CPU 0 by the bootstrap task,
// after `TaskInit()`, with interrupts and preemption enabled and no other tasks
// running.
//
// Side effects: the heap may grow permanently, LAPIC timers are temporarily
// reprogrammed, and live subsystems are probed with invalid arguments.
bool RunBootVerificationSuite();

}  // namespace protos

#endif  // PROTOS_MEMORY_TESTS_H_
