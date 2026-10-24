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
#include "interrupts.h"
#include "smp.h"
#include "uart.h"
#include "user_access.h"
#include "vga.h"

namespace protos {

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
// See `IdtSetUserFaultHandler()`.
std::atomic<InterruptHandlerFn> g_user_fault_handler{nullptr};
// See `IdtSetUserReturnHandler()`.
std::atomic<InterruptHandlerFn> g_user_return_handler{nullptr};

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
    const uint8_t type_attr = (v == kVectorSyscall) ? kIdtUserInterruptGateAttr
                                                    : kIdtInterruptGateAttr;
    g_idt[v] = IdtEncodeGate(g_isr_stub_table[v],     //
                             kIdtKernelCodeSelector,  //
                             IstSlotForVector(v),     //
                             type_attr);
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

void IdtSetUserFaultHandler(const InterruptHandlerFn handler) {
  g_user_fault_handler.store(handler, std::memory_order_release);
}

void IdtSetUserReturnHandler(const InterruptHandlerFn handler) {
  g_user_return_handler.store(handler, std::memory_order_release);
}

namespace {

// Ends the fault probe armed on the calling CPU at the page fault of kernel
// code in `frame`, if the fault is the probed function's (RunFaultProbe() in
// idt.h): records it in the probe, disarms the probe and makes `frame` return
// to the probe's resume point with RFLAGS.AC and RFLAGS.DF clear (the probed
// function may fault in the middle of a backward string copy). Returns false,
// changing nothing, otherwise.
static bool RecoverFaultProbe(InterruptFrame* const frame) {
  // Read CR2 first, like PanicUnhandledException().
  const uint64_t cr2 = ReadCr2();
  CpuLocal* const cpu = CurrentCpuOrNull();
  if (cpu == nullptr || cpu->fault_probe == nullptr) {
    return false;
  }
  FaultProbe* const probe = cpu->fault_probe;
  // The probed function runs with interrupts disabled, on the probe's stack
  // right below the resume point.
  if ((frame->rflags & kRflagsInterruptEnableBit) != 0 ||
      frame->rsp > probe->resume_rsp ||
      probe->resume_rsp - frame->rsp >
          static_cast<uint64_t>(kFaultProbeStackLimit)) {
    return false;
  }
  cpu->fault_probe = nullptr;
  probe->faulted = true;
  probe->error_code = frame->error_code;
  probe->fault_address = cr2;
  probe->fault_rip = frame->rip;
  frame->rip = probe->resume_rip;
  frame->rsp = probe->resume_rsp;
  frame->rflags &= ~(kRflagsAlignmentCheckBit | kRflagsDirectionBit);
  return true;
}

// Calls the handler of `frame->vector`, sends the Local APIC EOI if the vector
// needs one, and panics on CPU exceptions that nothing handles.
static void DispatchToHandler(InterruptFrame* const frame) {
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
    // A page fault that kernel code causes on purpose ends the fault probe
    // that expects it.
    if (vector == kVectorPageFault && !IsUserFrame(*frame) &&
        RecoverFaultProbe(frame)) {
      return;
    }
    // A fault of user code ends the user program instead, unless it is a
    // double fault or machine check, which mean that the kernel is broken.
    const InterruptHandlerFn user_fault_handler =
        g_user_fault_handler.load(std::memory_order_acquire);
    if (IsUserFrame(*frame) && user_fault_handler != nullptr &&
        vector != kVectorDoubleFault && vector != kVectorMachineCheck) {
      user_fault_handler(frame);
      PANIC("the user fault handler returned");
    }
    PanicUnhandledException(frame);
  }

  if (vector == static_cast<int>(kVectorApicTimer)) {
    CpuLocal* const cpu = CurrentCpuOrNull();
    if (cpu != nullptr) {
      cpu->timer_ticks.fetch_add(1, std::memory_order_relaxed);
      // Before the handler, which may switch the task out: the interrupt
      // counts for the task that it interrupted (cpu_clock.h).
      cpu->task_timer_ticks.fetch_add(1, std::memory_order_relaxed);
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

  if (vector == static_cast<int>(kVectorSyscall)) {
    // `int 0x80` sets no Local APIC in-service bit: an EOI would acknowledge
    // whichever interrupt is in service instead.
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

}  // namespace

extern "C" void IdtDispatch(InterruptFrame* const frame) {
  DCHECK(g_idt_initialized.load(std::memory_order_acquire));
  DCHECK(frame != nullptr);
  DCHECK(frame->vector < static_cast<uint64_t>(kIdtEntryCount));
  // The entry stub has cleared RFLAGS.AC if SMAP is enabled (boot.S), so that
  // handlers reach user memory only through the user-copy helpers
  // (user_access.h). The interrupted code's AC is in `frame->rflags`, and
  // iretq restores it.
  DCHECK(!UserAccessWindowOpen());

  DispatchToHandler(frame);

  // The interrupted user code resumes when the stub returns, maybe on another
  // CPU than the one that took the interrupt: the handler may have enabled
  // interrupts (the system call handler does, user_mode.h), and the task may
  // have been switched out and resumed elsewhere meanwhile. From here to
  // `iretq` interrupts stay disabled: the task must stay on this CPU, whose
  // next interrupt from user mode must push its frame where this one is, at
  // the top of the task's interrupt stack (user_mode.h), and nothing may
  // interrupt the stub's swapgs and iretq (boot.S). Frames of the IST vectors
  // are not on that stack, and their handlers neither enable interrupts nor
  // switch tasks.
  if (IsUserFrame(*frame) &&
      IstSlotForVector(static_cast<int>(frame->vector)) == 0) {
    (void)SaveAndDisableInterrupts();
    GdtSetKernelStack(reinterpret_cast<uintptr_t>(frame + 1));
    // Last: the handler may end the user program instead of returning, so
    // nothing above may need cleaning up after it.
    const InterruptHandlerFn on_return =
        g_user_return_handler.load(std::memory_order_acquire);
    if (on_return != nullptr) {
      on_return(frame);
    }
  }
}

#if !__STDC_HOSTED__
// fault_probe.S: calls `fn(arg)` and returns false, or returns true once
// IdtDispatch() resumed it at the resume point that it stored in `probe`
// after a page fault in `fn`.
extern "C" bool FaultProbeCall(void (*fn)(uintptr_t),  //
                               uintptr_t arg,          //
                               FaultProbe* probe);

FaultProbeResult RunFaultProbe(void (*const fn)(uintptr_t),
                               const uintptr_t arg) {
  DCHECK(fn != nullptr);
  const bool interrupts_were_enabled = SaveAndDisableInterrupts();
  CpuLocal* const cpu = CurrentCpu();
  DCHECK(cpu->fault_probe == nullptr);
  FaultProbe probe;
  cpu->fault_probe = &probe;
  const bool faulted = FaultProbeCall(fn, arg, &probe);
  // A page fault has disarmed it already.
  cpu->fault_probe = nullptr;
  DCHECK(faulted == probe.faulted);
  FaultProbeResult result;
  result.cpu = cpu->cpu_id;
  result.faulted = faulted;
  result.error_code = probe.error_code;
  result.address = probe.fault_address;
  result.rip = probe.fault_rip;
  RestoreInterrupts(interrupts_were_enabled);
  return result;
}
#endif

#if __STDC_HOSTED__
void IdtResetForTest() {
  for (int v = 0; v < kIdtEntryCount; ++v) {
    g_idt[v] = {};
    g_handlers[v].store(nullptr, std::memory_order_relaxed);
  }
  g_user_fault_handler.store(nullptr, std::memory_order_relaxed);
  g_user_return_handler.store(nullptr, std::memory_order_relaxed);
  g_idt_pointer = {};
  g_idt_initialized.store(false, std::memory_order_release);
  g_host_idt_load_count.store(0, std::memory_order_relaxed);
}

int IdtGetHostLoadCountForTest() {
  return g_host_idt_load_count.load(std::memory_order_relaxed);
}
#endif

}  // namespace protos
