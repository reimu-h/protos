#ifndef PROTOS_IDT_H_
#define PROTOS_IDT_H_

#include <cstddef>
#include <cstdint>

namespace protos {

// Total number of entries in the x86-64 Interrupt Descriptor Table.
constexpr int kIdtEntryCount = 256;
// Number of architectural CPU exception vectors (0..31).
constexpr int kCpuExceptionCount = 32;

// 64-bit Kernel Code Segment selector, the same in boot.S's `gdt64` and in the
// kernel GDT (`kGdtKernelCodeSelector` in gdt.h).
constexpr uint16_t kIdtKernelCodeSelector = 0x08;

// Gate type/attribute bytes (`P=1`, `S=0`):
// 0x8E: Present, DPL=0, 64-bit Interrupt Gate (clears RFLAGS.IF on entry).
constexpr uint8_t kIdtInterruptGateAttr = 0x8E;
// 0x8F: Present, DPL=0, 64-bit Trap Gate (leaves RFLAGS.IF unchanged).
constexpr uint8_t kIdtTrapGateAttr = 0x8F;
// 0xEE: Present, DPL=3, 64-bit Interrupt Gate.
constexpr uint8_t kIdtUserInterruptGateAttr = 0xEE;
// 0xEF: Present, DPL=3, 64-bit Trap Gate.
constexpr uint8_t kIdtUserTrapGateAttr = 0xEF;

// Architectural x86-64 CPU exception vectors (0..31).
constexpr int kVectorDivideError = 0;
constexpr int kVectorDebug = 1;
constexpr int kVectorNmi = 2;
constexpr int kVectorBreakpoint = 3;
constexpr int kVectorOverflow = 4;
constexpr int kVectorBoundRange = 5;
constexpr int kVectorInvalidOpcode = 6;
constexpr int kVectorDeviceNotAvailable = 7;
constexpr int kVectorDoubleFault = 8;
constexpr int kVectorInvalidTss = 10;
constexpr int kVectorSegmentNotPresent = 11;
constexpr int kVectorStackSegmentFault = 12;
constexpr int kVectorGeneralProtection = 13;
constexpr int kVectorPageFault = 14;
constexpr int kVectorX87FloatingPoint = 16;
constexpr int kVectorAlignmentCheck = 17;
constexpr int kVectorMachineCheck = 18;
constexpr int kVectorSimdFloatingPoint = 19;
constexpr int kVectorVirtualization = 20;
constexpr int kVectorControlProtection = 21;
constexpr int kVectorVmmCommunication = 29;
constexpr int kVectorSecurityException = 30;

// Vectors of the legacy 8259 PICs. boot.S remaps them away from the CPU
// exception vectors (IRQ0-7 to 0xE0-0xE7, IRQ8-15 to 0xE8-0xEF) and masks every
// line, so the only interrupts expected here are spurious IRQ7/IRQ15
// deliveries. `IdtDispatch` sends no Local APIC EOI for these vectors: the PIC
// delivers them as ExtINT, which never sets a Local APIC in-service bit.
constexpr int kVectorLegacyPicBase = 0xE0;
constexpr int kLegacyPicVectorCount = 16;

// Local APIC interrupt vectors (32..255).
constexpr uint8_t kVectorApicTimer = 0x20;
constexpr uint8_t kVectorWakeupIpi = 0x21;
// Kick IPI (`UserProcess::Kill()` in user_mode.h): makes a CPU that runs user
// code enter the kernel, so that it returns to user mode through the
// return-to-user handler (`IdtSetUserReturnHandler()`). Nothing handles it:
// `IdtDispatch` only sends the Local APIC EOI. (Same priority class as the
// timer.)
constexpr uint8_t kVectorKickIpi = 0x22;
// System calls (user_mode.h): the only vector whose gate user code may invoke
// (DPL 3), with `int 0x80`. A software interrupt, so `IdtDispatch` sends no
// Local APIC EOI for it. Its gate is an interrupt gate like every other one:
// `isr_common_stub` (boot.S) must swap GS before an interrupt can arrive. The
// system call handler enables interrupts itself once it has done its per-CPU
// bookkeeping (user_mode.h).
constexpr uint8_t kVectorSyscall = 0x80;
// TLB shootdown IPI (`SmpTlbShootdown()` in smp.h), in the highest Local APIC
// priority class so that it is delivered ahead of pending timer and wakeup
// interrupts.
constexpr uint8_t kVectorTlbShootdown = 0xF0;
constexpr uint8_t kVectorSpurious = 0xFF;

// Device MSI / MSI-X interrupt vectors (device_irq.h): 48 vectors in priority
// classes 5, 6 and 7, leaving out 0x20-0x22, 0x2F (boot_selftest), 0x40
// (idt_test), 0x80 (system calls), 0xE0-0xEF (PIC) and 0xF0/0xFF (APIC).
constexpr int kDeviceIrqVectorFirst = 0x50;
constexpr int kDeviceIrqVectorLast = 0x7F;
constexpr int kDeviceIrqVectorCount =
    kDeviceIrqVectorLast - kDeviceIrqVectorFirst + 1;

// 16-byte 64-bit IDT Gate Descriptor per Intel SDM Vol. 3A §6.14.1.
struct [[gnu::packed]] IdtGateDescriptor {
  uint16_t offset_low;
  uint16_t selector;
  uint8_t ist;
  uint8_t type_attr;
  uint16_t offset_mid;
  uint32_t offset_high;
  uint32_t reserved;
};
static_assert(sizeof(IdtGateDescriptor) == 16);

// 10-byte IDTR pseudo-descriptor loaded by `lidt`.
struct [[gnu::packed]] IdtPointer {
  uint16_t limit;
  uint64_t base;
};
static_assert(sizeof(IdtPointer) == 10);

// Saved CPU register state and hardware interrupt frame passed in `rdi` from
// `isr_common_stub` (`boot.S`) to `IdtDispatch`.
struct InterruptFrame {
  uint64_t r15;
  uint64_t r14;
  uint64_t r13;
  uint64_t r12;
  uint64_t r11;
  uint64_t r10;
  uint64_t r9;
  uint64_t r8;
  uint64_t rbp;
  uint64_t rdi;
  uint64_t rsi;
  uint64_t rdx;
  uint64_t rcx;
  uint64_t rbx;
  uint64_t rax;
  uint64_t vector;
  uint64_t error_code;
  uint64_t rip;
  uint64_t cs;
  uint64_t rflags;
  uint64_t rsp;
  uint64_t ss;
};
static_assert(sizeof(InterruptFrame) == 22 * sizeof(uint64_t));

// Callback signature for custom/recoverable exception and interrupt handlers.
using InterruptHandlerFn = void (*)(InterruptFrame* frame);

// Encodes a 16-byte 64-bit IDT gate descriptor. Validates `handler_addr != 0`,
// `selector != 0`, `ist <= 7`, and valid 64-bit interrupt/trap gate
// `type_attr` with `DCHECK`.
IdtGateDescriptor IdtEncodeGate(uint64_t handler_addr,  //
                                uint16_t selector,      //
                                uint8_t ist,            //
                                uint8_t type_attr);

// Extracts the 64-bit handler virtual address from a 16-byte IDT gate
// descriptor.
uint64_t IdtDecodeGateOffset(const IdtGateDescriptor& desc);

// Returns true if the x86-64 CPU automatically pushes a hardware error code
// onto the stack when raising exception `vector` (`0 <= vector < 256`).
// Validates bounds with `DCHECK`.
bool IdtVectorPushesErrorCode(int vector);

// Returns the architectural mnemonic string (e.g., "#DE", "#BP", "#PF") for
// CPU exception `vector` (`0 <= vector < 32`). Validates bounds with `DCHECK`.
const char* IdtExceptionMnemonic(int vector);

// Populates all 256 entries of the 64-bit IDT with 64-bit interrupt gate
// descriptors pointing to `g_isr_stub_table[0..255]` and loads `IDTR` on the
// calling CPU (BSP) via `IdtLoad()`. The gates of #DF, NMI and #MC switch to
// the IST stacks `kIstDoubleFault`, `kIstNmi` and `kIstMachineCheck` (gdt.h),
// so every CPU must load its TSS (`GdtLoadForCpu()`) before it loads the IDT;
// all other gates run on the interrupted stack (IST 0), or on the task's
// kernel stack (`GdtSetKernelStack()`) if the CPU was running user code. Only
// the gate of `kVectorSyscall` admits `int` from user code (DPL 3).
void IdtInit();

// Loads the 256-entry 64-bit IDT into the calling CPU's `IDTR` via `lidt`.
// Validates with `DCHECK` that `IdtInit()` has completed.
void IdtLoad();

// Returns true once `IdtInit()` has populated the 256-entry IDT.
bool IdtIsInitialized();

// Returns a non-null pointer to the IDT gate descriptor for `vector`
// (`0 <= vector < 256`). Validates with `DCHECK` that `IdtInit()` has
// completed and `vector` is in bounds.
const IdtGateDescriptor* IdtGetGate(int vector);

// Returns the 10-byte `IdtPointer` (`limit = 4095`, `base = &g_idt[0]`).
// Validates with `DCHECK` that `IdtInit()` has completed.
IdtPointer IdtGetPointer();

// Registers a non-null custom interrupt/exception handler for `vector`
// (`0 <= vector < 256`). Validates preconditions with `DCHECK`.
void IdtRegisterHandler(int vector, InterruptHandlerFn handler);

// Clears any custom handler registered for `vector` (`0 <= vector < 256`).
// Validates preconditions with `DCHECK`.
void IdtUnregisterHandler(int vector);

// Returns the custom handler currently registered for `vector`
// (`0 <= vector < 256`), or `nullptr` if none is registered. Validates
// preconditions with `DCHECK`.
InterruptHandlerFn IdtGetHandler(int vector);

// Returns true if `frame` interrupted user code (CPL 3).
inline bool IsUserFrame(const InterruptFrame& frame) {
  return (frame.cs & 3) == 3;
}

// Registers `handler` for the CPU exceptions that user code causes and that
// have no handler of their own (`IdtRegisterHandler()`), which would otherwise
// panic. `handler` must end the user program and not return (user_mode.h).
// Double faults and machine checks still panic: they mean that the kernel's
// state is broken. `nullptr` unregisters the handler.
void IdtSetUserFaultHandler(InterruptHandlerFn handler);

// Registers `handler` to run each time `IdtDispatch()` is about to return to
// user code (`IsUserFrame()`) from a vector without an IST stack: after the
// vector's own handler, with interrupts disabled, once `TSS.rsp0` of the CPU
// that returns (which may be another CPU than the one that took the interrupt)
// points at the top of `frame`. `handler` may end the user program instead of
// returning, for example when another task has killed it (user_mode.h).
// `nullptr` unregisters the handler.
void IdtSetUserReturnHandler(InterruptHandlerFn handler);

// Returns CR2, the linear address whose access caused the most recent page
// fault on the calling CPU. Host tests link a fake instead
// (idt_host_fakes.h).
uint64_t ReadCr2();

// Kernel fault probes
// ===================
//
// RunFaultProbe() lets kernel code make an access that page-faults on purpose
// and carry on: the boot self-tests use it to show that SMAP and SMEP stop the
// kernel from touching and executing user pages. It calls a function with
// interrupts disabled and a probe armed on the calling CPU
// (`CpuLocal::fault_probe`, smp.h). If the function page-faults, IdtDispatch()
// records the fault in the probe, disarms it and resumes RunFaultProbe() as if
// the function had returned. Every other fault still panics: a page fault
// with no probe armed on the CPU, of user code, with interrupts enabled, or of
// code that does not run on the probe's stack within kFaultProbeStackLimit
// below its resume point (such as an NMI or machine-check handler on its IST
// stack), and every other exception.

// The resume point of an armed fault probe and the page fault that ended it.
// FaultProbeCall() (fault_probe.S) writes the first two fields.
struct FaultProbe {
  uint64_t resume_rip = 0;
  uint64_t resume_rsp = 0;
  // Set by IdtDispatch() when a page fault ends the probe: its error code,
  // address (CR2) and RIP.
  bool faulted = false;
  uint64_t error_code = 0;
  uint64_t fault_address = 0;
  uint64_t fault_rip = 0;
};
// fault_probe.S hard-codes these offsets.
static_assert(offsetof(FaultProbe, resume_rip) == 0);
static_assert(offsetof(FaultProbe, resume_rsp) == 8);

// How far below its resume point the stack of a probed function may reach, in
// bytes.
constexpr int64_t kFaultProbeStackLimit = 4096;

// What RunFaultProbe() observed.
struct FaultProbeResult {
  // The CPU that ran the function.
  int cpu = -1;
  // Whether the function page-faulted instead of returning, and if so the
  // page fault's error code, address (CR2) and RIP.
  bool faulted = false;
  uint64_t error_code = 0;
  uint64_t address = 0;
  uint64_t rip = 0;
};

#if !__STDC_HOSTED__
// Calls `fn(arg)` on the calling CPU with interrupts disabled and a fault probe
// armed, and returns whether and how it page-faulted. `fn` may be the address
// of code in a user page. `fn` must neither enable interrupts nor switch
// tasks, and probes don't nest (DCHECK). Not for interrupt handlers.
FaultProbeResult RunFaultProbe(void (*fn)(uintptr_t arg), uintptr_t arg);
#endif

// C++ interrupt and exception dispatch entry point invoked by `isr_common_stub`
// in `boot.S` with `rdi = rsp` (`InterruptFrame*`). It starts with interrupts
// disabled (every gate is an interrupt gate). The handlers of vectors that
// interrupt user code may enable interrupts and switch tasks; the system call
// handler does (user_mode.h). Before such a frame returns to user code, which
// may then happen on another CPU than the one that took the interrupt,
// `IdtDispatch` disables interrupts again, points that CPU's `TSS.rsp0` back
// at the top of the task's interrupt frame (`GdtSetKernelStack()`), and runs
// the return-to-user handler (`IdtSetUserReturnHandler()`). Interrupts stay
// disabled from there until `iretq`. Frames of interrupted kernel code, and of
// the IST vectors (NMI, #DF, #MC), return with the interrupt flag as their
// handlers left it. Unhandled CPU exceptions of kernel code panic, except a
// page fault that ends the fault probe armed on the CPU (RunFaultProbe()).
extern "C" void IdtDispatch(InterruptFrame* frame);

#if __STDC_HOSTED__
// Host unit-test helpers for resetting IDT state and counting `lidt` calls.
// (idt_host_fakes.h has the helper for the faked `CR2` register.)
void IdtResetForTest();
int IdtGetHostLoadCountForTest();
#endif

}  // namespace protos

// Addresses of the 256 ISR entry stubs, indexed by vector. boot.S defines the
// table; host tests link a stand-in from idt_host_fakes.cpp.
extern "C" const uint64_t g_isr_stub_table[protos::kIdtEntryCount];

#endif  // PROTOS_IDT_H_
