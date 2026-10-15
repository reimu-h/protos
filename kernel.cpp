#include <cstdint>
#include <memory>

#include "check.h"
#include "console.h"
#include "cpu.h"
#include "gdt.h"
#include "heap.h"
#include "idt.h"
#include "paging.h"
#include "pmm.h"
#include "smp.h"
#include "smp_hardware.h"
#include "task.h"
#include "uart.h"
#include "user_mode.h"
#include "user_programs.h"
#include "vga.h"

#if PROTOS_SELFTEST
#include "selftest/selftest.h"
#endif

namespace protos {

// Runs the first user process, init (user/init.cpp), in an address space of
// its own until it ends, and reports how it ended.
static void RunInitProcess() {
  const std::unique_ptr<UserProcess> init =
      UserProcess::Start(InitProgram(), /*arg=*/0);
  CHECK(init != nullptr);
  const UserProgramResult result = init->Wait();
  ConsoleWrite("[INIT] ");
  WriteUserProgramResult(result, ConsoleWrite);
  ConsoleWrite("\n");
}

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
  // Before SmpInit() starts the APs, which apply the configuration it chooses,
  // and before any user page exists: with SMAP, the kernel reaches user
  // memory only through the user-copy helpers (user_access.h).
  CpuInitBsp();
  PmmInit(multiboot_magic, multiboot_info_addr);
  HeapInit();
  // Before any user address space exists and before the APs start: from here
  // on, the kernel's top-level page-table entries never change (paging.h).
  PagingInitKernelHalf();
  SmpSetHardware(CreateKernelSmpHardware());
  SmpInit(multiboot_magic, multiboot_info_addr);
  TaskInit();
  UserModeInit();

  // The kernel is up: start the first user process.
  RunInitProcess();

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
