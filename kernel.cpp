#include <cstdint>

#include "gdt.h"
#include "heap.h"
#include "idt.h"
#include "pmm.h"
#include "smp.h"
#include "smp_hardware.h"
#include "task.h"
#include "uart.h"
#include "vga.h"

#if PROTOS_SELFTEST
#include "selftest/selftest.h"
#endif

namespace protos {

extern "C" [[noreturn]] void kernel_main(const uint32_t multiboot_magic,
                                         const uint64_t multiboot_info_addr) {
  constexpr const char* kGreeting = "Hello, x86-64 Kernel World!\n";

  VgaClear();
  VgaWrite(kGreeting);

  UartInit();
  UartWrite(kGreeting);

  // CPU 0's TSS must be loaded before the IDT, whose #DF, NMI and #MC gates
  // switch to its IST stacks.
  GdtInit();
  IdtInit();
  PmmInit(multiboot_magic, multiboot_info_addr);
  HeapInit();
  SmpSetHardware(CreateKernelSmpHardware());
  SmpInit(multiboot_magic, multiboot_info_addr);
  TaskInit();

#if PROTOS_SELFTEST
  // Self-test build (the default; see SELFTEST in the Makefile): run the boot
  // self-tests once and then the randomized multi-CPU stress test forever,
  // keeping every CPU busy. RunSelfTests() returns only if the boot self-tests
  // failed; the idle loop below then keeps their failure report on the console.
  RunSelfTests();
#endif

  // Nothing (more) to run: idle with interrupts enabled so that timer ticks and
  // IPIs are still serviced.
  for (;;) {
    asm volatile("sti; hlt" : : : "memory");
  }
}

}  // namespace protos
