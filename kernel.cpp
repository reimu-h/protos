#include <cstdint>

#include "heap.h"
#include "idt.h"
#include "pmm.h"
#include "smp.h"
#include "task.h"
#include "uart.h"
#include "vga.h"

#if PROTOS_SELFTEST
#include "memory_tests.h"
#include "stress_test.h"
#endif

namespace protos {

extern "C" [[noreturn]] void kernel_main(const uint32_t multiboot_magic,
                                         const uint64_t multiboot_info_addr) {
  constexpr const char* kGreeting = "Hello, x86-64 Kernel World!\n";

  VgaClear();
  VgaWrite(kGreeting);

  UartInit();
  UartWrite(kGreeting);

  IdtInit();
  PmmInit(multiboot_magic, multiboot_info_addr);
  HeapInit();
  SmpInit(multiboot_magic, multiboot_info_addr);
  TaskInit();

#if PROTOS_SELFTEST
  // Self-test build (the default; see SELFTEST in the Makefile): run the boot
  // verification suite once and then the randomized multi-CPU stress test
  // forever, keeping every CPU busy. If the suite fails, stop here so that its
  // failure report stays on the console.
  if (RunBootVerificationSuite()) {
    for (;;) {
      RunRandomMultiCpuStressTest();
    }
  }
#endif

  // Nothing (more) to run: idle with interrupts enabled so that timer ticks and
  // IPIs are still serviced.
  for (;;) {
    asm volatile("sti; hlt" : : : "memory");
  }
}

}  // namespace protos
