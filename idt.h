#ifndef PROTOS_IDT_H_
#define PROTOS_IDT_H_

#include <cstdint>

namespace protos {

// Total number of entries in the x86-64 Interrupt Descriptor Table.
constexpr int kIdtEntryCount = 256;
// Number of architectural CPU exception vectors (0..31).
constexpr int kCpuExceptionCount = 32;

// 64-bit Kernel Code Segment selector in `gdt64` (`boot.S`).
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
constexpr uint8_t kVectorSpurious = 0xFF;

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
// calling CPU (BSP) via `IdtLoad()`.
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

// C++ interrupt and exception dispatch entry point invoked by `isr_common_stub`
// in `boot.S` with `rdi = rsp` (`InterruptFrame*`).
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
