#include <cstdint>
#include <memory>

#include "check.h"
#include "console.h"
#include "cpu.h"
#include "device_irq.h"
#include "driver.h"
#include "gdt.h"
#include "heap.h"
#include "idt.h"
#include "ioapic.h"
#include "multiboot.h"
#include "paging.h"
#include "pci.h"
#include "pmm.h"
#include "port_io.h"
#include "smp.h"
#include "smp_hardware.h"
#include "syscall_abi.h"
#include "task.h"
#include "uart.h"
#include "user_mode.h"
#include "user_programs.h"
#include "vga.h"

#if PROTOS_SELFTEST
#include "selftest/boot_selftest.h"
#include "selftest/selftest.h"
#include "selftest/stress.h"
#endif

namespace protos {

#if PROTOS_SELFTEST
constexpr uint16_t kFwCfgPortSelector = 0x0510;
constexpr uint16_t kFwCfgPortData = 0x0511;
constexpr uint16_t kFwCfgSelectorSignature = 0x0000;
constexpr uint16_t kFwCfgSelectorFileDir = 0x0019;
constexpr int kFwCfgFileNameBytes = 56;
constexpr uint32_t kFwCfgMaxFileDirEntries = 256;

static uint16_t FwCfgReadBe16() {
  const uint16_t b0 = PortIn8(kFwCfgPortData);
  const uint16_t b1 = PortIn8(kFwCfgPortData);
  return static_cast<uint16_t>((b0 << 8) | b1);
}

static uint32_t FwCfgReadBe32() {
  const uint32_t b0 = PortIn8(kFwCfgPortData);
  const uint32_t b1 = PortIn8(kFwCfgPortData);
  const uint32_t b2 = PortIn8(kFwCfgPortData);
  const uint32_t b3 = PortIn8(kFwCfgPortData);
  return (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
}

static bool StringsEqual(const char* const a, const char* const b) {
  int i = 0;
  while (a[i] != '\0' && b[i] != '\0') {
    if (a[i] != b[i]) {
      return false;
    }
    ++i;
  }
  return a[i] == b[i];
}

// Reads the named QEMU fw_cfg file into `buffer` (up to `capacity - 1` bytes),
// strips trailing NUL/CR/LF bytes, NUL-terminates `buffer`, and returns true if
// the file exists and fits in `buffer`.
static bool FwCfgReadString(const char* const target_name,  //
                            char* const buffer,             //
                            const int capacity) {
  if (capacity <= 0) {
    return false;
  }
  buffer[0] = '\0';
  PortOut16(kFwCfgPortSelector, kFwCfgSelectorSignature);
  if (PortIn8(kFwCfgPortData) != 'Q' || PortIn8(kFwCfgPortData) != 'E' ||
      PortIn8(kFwCfgPortData) != 'M' || PortIn8(kFwCfgPortData) != 'U') {
    return false;
  }
  PortOut16(kFwCfgPortSelector, kFwCfgSelectorFileDir);
  const uint32_t count = FwCfgReadBe32();
  if (count == 0 || count > kFwCfgMaxFileDirEntries) {
    return false;
  }
  for (uint32_t i = 0; i < count; ++i) {
    const uint32_t size = FwCfgReadBe32();
    const uint16_t select = FwCfgReadBe16();
    (void)FwCfgReadBe16();  // reserved
    char entry_name[kFwCfgFileNameBytes] = {};
    for (int j = 0; j < kFwCfgFileNameBytes; ++j) {
      entry_name[j] = static_cast<char>(PortIn8(kFwCfgPortData));
    }
    entry_name[kFwCfgFileNameBytes - 1] = '\0';
    if (!StringsEqual(entry_name, target_name)) {
      continue;
    }
    if (size >= static_cast<uint32_t>(capacity)) {
      return false;
    }
    PortOut16(kFwCfgPortSelector, select);
    int length = static_cast<int>(size);
    for (int j = 0; j < length; ++j) {
      buffer[j] = static_cast<char>(PortIn8(kFwCfgPortData));
    }
    while (length > 0 &&
           (buffer[length - 1] == '\0' || buffer[length - 1] == '\n' ||
            buffer[length - 1] == '\r')) {
      --length;
    }
    buffer[length] = '\0';
    return true;
  }
  return false;
}

// Returns true in a virtual machine: hypervisors set CPUID.01H:ECX bit 31
// (QEMU does for TCG and KVM, also with -cpu host), and physical processors
// return 0 there (Intel SDM Vol. 2A, CPUID: "Not Used. Always returns 0"; AMD
// APM Vol. 3: reserved for use by hypervisors to indicate guest status).
static bool RunsUnderHypervisor() {
  constexpr uint32_t kCpuidLeaf1EcxHypervisor = 1u << 31;
  uint32_t eax = 1;
  uint32_t ebx = 0;
  uint32_t ecx = 0;
  uint32_t edx = 0;
  asm volatile("cpuid" : "+a"(eax), "=b"(ebx), "+c"(ecx), "=d"(edx));
  return (ecx & kCpuidLeaf1EcxHypervisor) != 0;
}

static bool IsInteractiveBootSession() {
  // QEMU's fw_cfg device exists only in virtual machines. On a physical PC, I/O
  // port 0x510 can be a chipset register (PROC_CNT of an ICH whose PMBASE is
  // 0x500, or a GPIO or vendor register), which the selector write would
  // clobber.
  if (!RunsUnderHypervisor()) {
    return false;
  }
  char mode[32] = {};
  return FwCfgReadString("opt/protos/boot_session", mode, sizeof(mode)) &&
         StringsEqual(mode, "interactive");
}
#endif

// Runs the first user process, init (user/init/init.cpp), in an address space
// of its own until it ends, and reports how it ended.
static bool RunInitProcess(const bool interactive) {
  ConsoleClearInput();
#if PROTOS_SELFTEST
  constexpr char kBootShellScript[] =
      "help\necho boot_sh_ok\nusertest 0\nexit 0\n";
  constexpr char kInteractiveBootShellScript[] =
      "help\necho boot_sh_ok\nusertest 0\n";
  if (interactive) {
    ConsolePushInput(
        kInteractiveBootShellScript,
        static_cast<int64_t>(sizeof(kInteractiveBootShellScript) - 1));
  } else {
    ConsolePushInput(kBootShellScript,
                     static_cast<int64_t>(sizeof(kBootShellScript) - 1));
  }
  ConsoleSetVgaEcho(false);
#else
  (void)interactive;
#endif
  const std::unique_ptr<UserProcess> init = UserProcess::Start(
      InitProgram(), kInitArgStartDrivers, kUserCapSpawnDrivers);
  CHECK(init != nullptr);
  const UserProgramResult result = init->Wait();
#if PROTOS_SELFTEST
  ConsoleSetVgaEcho(true);
#endif
  ConsoleClearInput();
  ConsoleWrite("[INIT] ");
  WriteUserProgramResult(result, ConsoleWrite);
  ConsoleWrite("\n");
  return result.outcome == UserProgramOutcome::kExited &&
         result.exit_status == 0;
}

#if PROTOS_SELFTEST
static void RunInteractiveBootSession() {
  if (!RunBootSelfTests()) {
    return;
  }
  const int64_t pmm_free_before = PmmFreeFrameCount();
  const int64_t heap_free_before = HeapTotalFreeBytes();
  const bool init_ok = RunInitProcess(/*interactive=*/true);
  const bool cleanup_ok = init_ok &&                                   //
                          PagingActiveRoot() == PagingKernelRoot() &&  //
                          PagingUserSpaceIsEmpty() &&                  //
                          PciClaimedCount() == 0 &&                    //
                          IsaClaimedCount() == 0 &&                    //
                          DeviceIrqUsedVectorCount() == 0 &&           //
                          PmmFreeFrameCount() == pmm_free_before &&    //
                          HeapTotalFreeBytes() == heap_free_before;
  if (!cleanup_ok) {
    UartWrite("[TEST] driver_session_cleanup: FAIL\n");
    return;
  }
  UartWrite("[TEST] driver_session_cleanup: PASS\n");
  for (;;) {
    RunRandomMultiCpuStressTest();
  }
}
#endif

extern "C" [[noreturn]] void kernel_main(const uint32_t multiboot_magic,
                                         const uint64_t multiboot_info_addr) {
  constexpr const char* kGreeting = "Hello, x86-64 Kernel World!\n";

  UartInit();
  // Before anything is written to the screen: the legacy VGA text buffer and
  // the CRT controller exist only if the boot loader left the screen in VGA
  // text mode. Otherwise the console keeps its text in memory until PmmInit()
  // attaches the framebuffer that the boot loader describes, if there is one.
  if (MultibootHasVgaTextMode(multiboot_magic,      //
                              multiboot_info_addr,  //
                              kIdentityMapLimit)) {
    VgaEnableLegacyText();
  }
  VgaClear();
  VgaWrite(kGreeting);
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
  // Enumerate the PCI functions that driver processes may claim. PciInit()
  // briefly turns off each function's decoding to size its BARs, so it must
  // run before any user process exists. The device list goes to the serial
  // port only: the screen keeps its rows for the boot summary.
  PciInit(CreateKernelPciConfigSpace());
  PciWriteBootReport(UartWrite);
  IoApicInit();
  DriverInit();

#if PROTOS_SELFTEST
  if (IsInteractiveBootSession()) {
    RunInteractiveBootSession();
  } else {
    (void)RunInitProcess(/*interactive=*/false);
    // Self-test build (the default; see SELFTEST in the Makefile): run the boot
    // self-tests once and then the randomized multi-CPU stress test forever,
    // keeping every CPU busy. RunSelfTests() returns only if the boot
    // self-tests failed; the idle loop below then keeps their failure report on
    // the console.
    RunSelfTests();
  }
#else
  (void)RunInitProcess(/*interactive=*/false);
#endif

  // Nothing (more) to run: idle with interrupts enabled so that timer ticks and
  // IPIs are still serviced.
  for (;;) {
    asm volatile("sti; hlt" : : : "memory");
  }
}

}  // namespace protos
