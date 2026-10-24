#ifndef PROTOS_SELFTEST_BOOT_SELFTEST_H_
#define PROTOS_SELFTEST_BOOT_SELFTEST_H_

#include <cstdint>

#include "selftest/fpu_lifecycle_layout.h"

namespace protos {

// Runs the boot self-test suite and returns true if every check passed.
//
// Covers the physical memory manager, kernel heap, C++ runtime support,
// Multiboot parsing, paging, the RB-tree, SMP bring-up, per-CPU IDT/LAPIC timer
// state, and the task scheduler. Logs a `[TEST] <name>: PASS|FAIL` line per
// test and a final `[TEST] ALL BOOT SELF-TESTS PASSED` or
// `[TEST] BOOT SELF-TESTS FAILED` line over UART COM1 (test_boot.sh depends on
// these markers), and a one-line summary on the display console.
//
// Preconditions: called once by `RunSelfTests()` (selftest.h) from
// `kernel_main` on CPU 0 by the bootstrap task, after `TaskInit()`, with
// interrupts and preemption enabled and no other tasks running.
//
// Side effects: the heap may grow permanently, LAPIC timers are temporarily
// reprogrammed, and live subsystems are probed with invalid arguments.
bool RunBootSelfTests();

// Whether the boot self-tests may run their driver tests, which claim a PCI
// function and change its interrupt settings and Bus Master, on a function of
// class `class_code`, `subclass` and programming interface `prog_if`. Not on
// functions that firmware may still be using:
// - Display controllers (class 03): firmware left one scanning out the
//   framebuffer that the console draws on, which an integrated GPU reads by
//   bus mastering, so clearing its Bus Master blanks the screen.
// - USB host controllers other than xHCI (class 0C, subclass 03: UHCI, OHCI,
//   EHCI, USB4): firmware may still run its USB legacy keyboard emulation on
//   them (in SMM), and nothing in this kernel takes them over. The tests take
//   an xHCI over from firmware and reset it first, as usbkbd does.
// - Base system peripherals (class 08), such as an IOMMU, which bus-masters to
//   read the tables that firmware may have set up for it.
// Exposed for host tests.
constexpr bool BootSelfTestMayUsePciFunction(const uint8_t class_code,
                                             const uint8_t subclass,
                                             const uint8_t prog_if) {
  constexpr uint8_t kDisplay = 0x03;
  constexpr uint8_t kBaseSystemPeripheral = 0x08;
  constexpr uint8_t kSerialBus = 0x0C;
  constexpr uint8_t kUsb = 0x03;
  constexpr uint8_t kXhci = 0x30;
  if (class_code == kDisplay || class_code == kBaseSystemPeripheral) {
    return false;
  }
  return class_code != kSerialBus || subclass != kUsb || prog_if == kXhci;
}

}  // namespace protos

#endif  // PROTOS_SELFTEST_BOOT_SELFTEST_H_
