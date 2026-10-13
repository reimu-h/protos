#include "idt.h"

#include <atomic>
#include <cstdint>
#if __STDC_HOSTED__
#include <cstdio>
#include <cstdlib>
#endif

#include "check.h"
#include "format.h"
#include "gdt.h"
#include "smp.h"
#include "uart.h"
#include "vga.h"

namespace protos {

// Returns CR2, the linear address of the most recent page fault. Host tests
// link a fake instead (idt_host_fakes.cpp).
uint64_t ReadCr2();

#if !__STDC_HOSTED__
uint64_t ReadCr2() {
  uint64_t cr2 = 0;
  asm volatile("mov %0, cr2" : "=r"(cr2));
  return cr2;
}
#endif

namespace {

constexpr uint8_t kMaxIstIndex = 7;

alignas(16) IdtGateDescriptor g_idt[kIdtEntryCount] = {};
IdtPointer g_idt_pointer = {};
std::atomic<InterruptHandlerFn> g_handlers[kIdtEntryCount] = {};
std::atomic<bool> g_idt_initialized{false};

#if __STDC_HOSTED__
std::atomic<int> g_host_idt_load_count{0};
#endif

static bool IsValidGateTypeAttr(const uint8_t type_attr) {
  // Must have Present=1 (bit 7), S=0 (bit 4), and 64-bit Interrupt Gate (0xE)
  // or 64-bit Trap Gate (0xF) in bits 0..3.
  const uint8_t gate_type = static_cast<uint8_t>(type_attr & 0x0Fu);
  return (type_attr & 0x90u) == 0x80u &&
         (gate_type == 0x0Eu || gate_type == 0x0Fu);
}

// Returns the IST slot (gdt.h) whose stack `vector` runs on, or 0 for the
// interrupted stack.
static uint8_t IstSlotForVector(const int vector) {
  switch (vector) {
    case kVectorDoubleFault:
      return kIstDoubleFault;
    case kVectorNmi:
      return kIstNmi;
    case kVectorMachineCheck:
      return kIstMachineCheck;
    default:
      return 0;
  }
}

// Kernel panic path, shared by unhandled CPU exceptions, failed CHECKs
// (internal::KernelCheckFailure()) and PANIC() (internal::KernelPanic()). The
// first CPU to panic owns the panic: it stops the other CPUs
// (SmpStopOtherCpusForPanic()) and then prints without taking the console
// locks, which a stopped CPU may hold. A nested panic on that CPU prints to the
// UART only, and every other panicking CPU halts without printing (see
// internal::EnterPanic()).

// Consoles the panic path prints to.
enum class PanicOutput {
  kUartAndVga,
  kUartOnly,
};

#if !__STDC_HOSTED__
// Panic arbitration state for internal::EnterPanic().
std::atomic<uint64_t> g_panic_owner{0};
int g_panic_depth = 0;
// Panic token of a CPU that has not bound a CpuLocal yet (CpuLocal addresses
// are 64-byte aligned, so none equals it). Only the BSP runs unbound, before
// SmpInit(): the startup trampoline binds an AP's CpuLocal before the AP runs
// any C++ code.
constexpr uint64_t kUnboundCpuPanicToken = 1;
#endif

static void PanicWriteStr(const PanicOutput output, const char* const str) {
  UartPanicWrite(str);
  if (output == PanicOutput::kUartAndVga) {
    VgaPanicWrite(str);
  }
}

static void PanicWriteDec(const PanicOutput output, const uint64_t value) {
  char buffer[kFormatDecBufferSize];
  PanicWriteStr(output, FormatDec(value, buffer));
}

static void PanicWriteHex(const PanicOutput output, const uint64_t value) {
  char buffer[kFormatHexBufferSize];
  PanicWriteStr(output, FormatHex(value, buffer));
}

[[noreturn]] static void PanicHalt() {
#if !__STDC_HOSTED__
  for (;;) {
    asm volatile("cli; hlt");
  }
#else
  std::fflush(stderr);
  std::_Exit(134);
#endif
}

// Enters the panic path. On bare metal, disables interrupts and arbitrates
// between CPUs: returns where the calling CPU prints the diagnostic, or halts
// here if it must stay silent. Host builds always print to both consoles.
static PanicOutput BeginPanic() {
#if !__STDC_HOSTED__
  asm volatile("cli" : : : "memory", "cc");
  const CpuLocal* const cpu = CurrentCpuOrNull();
  const uint64_t token = (cpu != nullptr) ? reinterpret_cast<uintptr_t>(cpu)
                                          : kUnboundCpuPanicToken;
  switch (internal::EnterPanic(&g_panic_owner, &g_panic_depth, token)) {
    case internal::PanicRole::kOwner:
      SmpStopOtherCpusForPanic();
      return PanicOutput::kUartAndVga;
    case internal::PanicRole::kNestedOwner:
      return PanicOutput::kUartOnly;
    case internal::PanicRole::kSilent:
      break;
  }
  // The owner's stop NMI parks this CPU if it arrives later.
  PanicHalt();
#else
  return PanicOutput::kUartAndVga;
#endif
}

#if !__STDC_HOSTED__
// Prints "[FATAL] <prefix><message> at <file>:<line>" on the panic path and
// halts. Shared by internal::KernelCheckFailure() and internal::KernelPanic().
[[noreturn]] static void PanicWithMessage(const char* const prefix,
                                          const char* const message,
                                          const char* const file,
                                          const int line) {
  const PanicOutput output = BeginPanic();
  PanicWriteStr(output, "[FATAL] ");
  PanicWriteStr(output, prefix);
  PanicWriteStr(output, message);
  PanicWriteStr(output, " at ");
  PanicWriteStr(output, file);
  PanicWriteStr(output, ":");
  PanicWriteDec(output, (line > 0) ? static_cast<uint64_t>(line) : 0);
  PanicWriteStr(output, "\n");
  PanicHalt();
}
#endif

}  // namespace

#if !__STDC_HOSTED__
[[noreturn]] void internal::KernelCheckFailure(const char* const expr,
                                               const char* const file,
                                               const int line) {
  PanicWithMessage("Check failed: ", expr, file, line);
}

[[noreturn]] void internal::KernelPanic(const char* const message,
                                        const char* const file,
                                        const int line) {
  PanicWithMessage("", message, file, line);
}
#endif

namespace {

[[noreturn]] static void PanicUnhandledException(
    const InterruptFrame* const frame) {
  // Read CR2 first: a fault in the code below would overwrite it.
  const uint64_t cr2 = ReadCr2();
  const PanicOutput output = BeginPanic();
  const int vector = static_cast<int>(frame->vector);
  const char* const mnemonic = IdtExceptionMnemonic(vector);
  const CpuLocal* const cpu = CurrentCpuOrNull();

  PanicWriteStr(output, "[FATAL] Unhandled CPU exception ");
  PanicWriteDec(output, frame->vector);
  PanicWriteStr(output, " (");
  PanicWriteStr(output, mnemonic);
  PanicWriteStr(output, ") err=");
  PanicWriteHex(output, frame->error_code);
  PanicWriteStr(output, " rip=");
  PanicWriteHex(output, frame->rip);
  PanicWriteStr(output, " rsp=");
  PanicWriteHex(output, frame->rsp);
  PanicWriteStr(output, " rflags=");
  PanicWriteHex(output, frame->rflags);
  PanicWriteStr(output, " cr2=");
  PanicWriteHex(output, cr2);
  PanicWriteStr(output, " cpu=");
  if (cpu != nullptr) {
    PanicWriteDec(output, static_cast<uint64_t>(cpu->cpu_id));
  } else {
    PanicWriteStr(output, "unbound");
  }
  PanicWriteStr(output, "\n");
  PanicHalt();
}

}  // namespace

IdtGateDescriptor IdtEncodeGate(const uint64_t handler_addr,  //
                                const uint16_t selector,      //
                                const uint8_t ist,            //
                                const uint8_t type_attr) {
  DCHECK(handler_addr != 0);
  const uint64_t sign_ext =
      static_cast<uint64_t>(static_cast<int64_t>(handler_addr) >> 47);
  DCHECK(sign_ext == 0ULL || sign_ext == ~0ULL);
  DCHECK(selector != 0);
  DCHECK(ist <= kMaxIstIndex);
  DCHECK(IsValidGateTypeAttr(type_attr));

  IdtGateDescriptor desc = {};
  desc.offset_low = static_cast<uint16_t>(handler_addr & 0xFFFFu);
  desc.selector = selector;
  desc.ist = static_cast<uint8_t>(ist & 0x07u);
  desc.type_attr = type_attr;
  desc.offset_mid = static_cast<uint16_t>((handler_addr >> 16) & 0xFFFFu);
  desc.offset_high = static_cast<uint32_t>((handler_addr >> 32) & 0xFFFFFFFFu);
  desc.reserved = 0;
  return desc;
}

uint64_t IdtDecodeGateOffset(const IdtGateDescriptor& desc) {
  return static_cast<uint64_t>(desc.offset_low) |
         (static_cast<uint64_t>(desc.offset_mid) << 16) |
         (static_cast<uint64_t>(desc.offset_high) << 32);
}

bool IdtVectorPushesErrorCode(const int vector) {
  DCHECK(vector >= 0 && vector < kIdtEntryCount);
  switch (vector) {
    case 8:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 17:
    case 21:
    case 29:
    case 30:
      return true;
    default:
      return false;
  }
}

const char* IdtExceptionMnemonic(const int vector) {
  DCHECK(vector >= 0 && vector < kCpuExceptionCount);
  static constexpr const char* kMnemonics[kCpuExceptionCount] = {
      "#DE",  "#DB",  "#NMI", "#BP",  "#OF", "#BR", "#UD",  "#NM",
      "#DF",  "#CSO", "#TS",  "#NP",  "#SS", "#GP", "#PF",  "#RES",
      "#MF",  "#AC",  "#MC",  "#XM",  "#VE", "#CP", "#RES", "#RES",
      "#RES", "#RES", "#RES", "#RES", "#HV", "#VC", "#SX",  "#RES",
  };
  return kMnemonics[vector];
}

void IdtInit() {
  for (int v = 0; v < kIdtEntryCount; ++v) {
    g_handlers[v].store(nullptr, std::memory_order_relaxed);
    g_idt[v] = IdtEncodeGate(g_isr_stub_table[v],     //
                             kIdtKernelCodeSelector,  //
                             IstSlotForVector(v),     //
                             kIdtInterruptGateAttr);
  }
  g_idt_pointer.limit = static_cast<uint16_t>(sizeof(g_idt) - 1);
  g_idt_pointer.base = reinterpret_cast<uint64_t>(&g_idt[0]);
  g_idt_initialized.store(true, std::memory_order_release);
  IdtLoad();
}

void IdtLoad() {
  DCHECK(g_idt_initialized.load(std::memory_order_acquire));
#if !__STDC_HOSTED__
  asm volatile("lidt [%0]" : : "r"(&g_idt_pointer) : "memory");
#else
  g_host_idt_load_count.fetch_add(1, std::memory_order_relaxed);
#endif
}

bool IdtIsInitialized() {
  return g_idt_initialized.load(std::memory_order_acquire);
}

const IdtGateDescriptor* IdtGetGate(const int vector) {
  DCHECK(g_idt_initialized.load(std::memory_order_acquire));
  DCHECK(vector >= 0 && vector < kIdtEntryCount);
  return &g_idt[vector];
}

IdtPointer IdtGetPointer() {
  DCHECK(g_idt_initialized.load(std::memory_order_acquire));
  return g_idt_pointer;
}

void IdtRegisterHandler(const int vector, const InterruptHandlerFn handler) {
  DCHECK(g_idt_initialized.load(std::memory_order_acquire));
  DCHECK(vector >= 0 && vector < kIdtEntryCount);
  DCHECK(handler != nullptr);
  g_handlers[vector].store(handler, std::memory_order_release);
}

void IdtUnregisterHandler(const int vector) {
  DCHECK(g_idt_initialized.load(std::memory_order_acquire));
  DCHECK(vector >= 0 && vector < kIdtEntryCount);
  g_handlers[vector].store(nullptr, std::memory_order_release);
}

InterruptHandlerFn IdtGetHandler(const int vector) {
  DCHECK(g_idt_initialized.load(std::memory_order_acquire));
  DCHECK(vector >= 0 && vector < kIdtEntryCount);
  return g_handlers[vector].load(std::memory_order_acquire);
}

extern "C" void IdtDispatch(InterruptFrame* const frame) {
  DCHECK(g_idt_initialized.load(std::memory_order_acquire));
  DCHECK(frame != nullptr);
  DCHECK(frame->vector < static_cast<uint64_t>(kIdtEntryCount));

  const int vector = static_cast<int>(frame->vector);
  const InterruptHandlerFn handler =
      g_handlers[vector].load(std::memory_order_acquire);

  if (vector == kVectorNmi) {
    if (handler != nullptr) {
      handler(frame);
    }
    return;
  }

  if (vector < kCpuExceptionCount) {
    if (handler != nullptr) {
      handler(frame);
      return;
    }
    PanicUnhandledException(frame);
  }

  if (vector == static_cast<int>(kVectorApicTimer)) {
    CpuLocal* const cpu = CurrentCpuOrNull();
    if (cpu != nullptr) {
      cpu->timer_ticks.fetch_add(1, std::memory_order_relaxed);
    }
    SmpSendLocalApicEoi();
    if (handler != nullptr) {
      handler(frame);
    }
    return;
  }

  if (vector == static_cast<int>(kVectorWakeupIpi)) {
    CpuLocal* const cpu = CurrentCpuOrNull();
    if (cpu != nullptr) {
      cpu->ipi_count.fetch_add(1, std::memory_order_relaxed);
    }
    SmpSendLocalApicEoi();
    if (handler != nullptr) {
      handler(frame);
    }
    return;
  }

  if (vector == static_cast<int>(kVectorSpurious)) {
    if (handler != nullptr) {
      handler(frame);
    }
    return;
  }

  if (vector >= kVectorLegacyPicBase &&
      vector < kVectorLegacyPicBase + kLegacyPicVectorCount) {
    // Legacy 8259 PIC vector (see kVectorLegacyPicBase): no Local APIC EOI.
    // With every PIC line masked this is a spurious IRQ7/IRQ15, which must not
    // be acknowledged at the PIC either; a handler for an unmasked line has to
    // send the PIC EOI itself.
    if (handler != nullptr) {
      handler(frame);
    }
    return;
  }

  SmpSendLocalApicEoi();
  if (handler != nullptr) {
    handler(frame);
  }
}

#if __STDC_HOSTED__
void IdtResetForTest() {
  for (int v = 0; v < kIdtEntryCount; ++v) {
    g_idt[v] = {};
    g_handlers[v].store(nullptr, std::memory_order_relaxed);
  }
  g_idt_pointer = {};
  g_idt_initialized.store(false, std::memory_order_release);
  g_host_idt_load_count.store(0, std::memory_order_relaxed);
}

int IdtGetHostLoadCountForTest() {
  return g_host_idt_load_count.load(std::memory_order_relaxed);
}
#endif

}  // namespace protos
