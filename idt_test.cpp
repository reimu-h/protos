#include "idt.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "check.h"
#include "gdt.h"
#include "idt_host_fakes.h"
#include "smp.h"
#include "uart.h"
#include "vga.h"

namespace protos {
namespace {

namespace t = ::testing;

// Console whose panic output the fakes below copy to stderr, where death tests
// can match it (copying both would interleave them).
enum class PanicConsole { kNone, kUart, kVga };

struct FakeIdtEnv {
  std::atomic<int64_t> eoi_count{0};
  PanicConsole stderr_console = PanicConsole::kNone;
};

FakeIdtEnv g_env;

static void ResetEnv() {
  IdtResetForTest();
  IdtSetHostCr2ForTest(0);
  ResetCpuLocalForTest();
  g_env.eoi_count.store(0, std::memory_order_relaxed);
  g_env.stderr_console = PanicConsole::kNone;
}

static void WritePanicOutput(const PanicConsole console,
                             const char* const str) {
  if (str != nullptr && g_env.stderr_console == console) {
    std::fputs(str, stderr);
  }
}

}  // namespace

void SmpSendLocalApicEoi() {
  g_env.eoi_count.fetch_add(1, std::memory_order_relaxed);
}

void UartPanicWrite(const char* const str) {
  WritePanicOutput(PanicConsole::kUart, str);
}

void VgaPanicWrite(const char* const str) {
  WritePanicOutput(PanicConsole::kVga, str);
}

namespace {

TEST(IdtTest, EncodeGateAndDecodeOffsetEncodeAllFieldsCorrectly) {
  ResetEnv();
  constexpr uint64_t kHandlerAddr = 0xFFFFFFFF80123456ULL;

  const IdtGateDescriptor irq_gate = IdtEncodeGate(
      kHandlerAddr, kIdtKernelCodeSelector, 0, kIdtInterruptGateAttr);
  EXPECT_THAT(irq_gate.offset_low, t::Eq(0x3456u));
  EXPECT_THAT(irq_gate.selector, t::Eq(kIdtKernelCodeSelector));
  EXPECT_THAT(irq_gate.ist, t::Eq(0u));
  EXPECT_THAT(irq_gate.type_attr, t::Eq(kIdtInterruptGateAttr));
  EXPECT_THAT(irq_gate.offset_mid, t::Eq(0x8012u));
  EXPECT_THAT(irq_gate.offset_high, t::Eq(0xFFFFFFFFu));
  EXPECT_THAT(irq_gate.reserved, t::Eq(0u));
  EXPECT_THAT(IdtDecodeGateOffset(irq_gate), t::Eq(kHandlerAddr));

  const IdtGateDescriptor trap_gate = IdtEncodeGate(
      0x0000123456789ABCULL, kIdtKernelCodeSelector, 7, kIdtUserTrapGateAttr);
  EXPECT_THAT(trap_gate.offset_low, t::Eq(0x9ABCu));
  EXPECT_THAT(trap_gate.selector, t::Eq(kIdtKernelCodeSelector));
  EXPECT_THAT(trap_gate.ist, t::Eq(7u));
  EXPECT_THAT(trap_gate.type_attr, t::Eq(kIdtUserTrapGateAttr));
  EXPECT_THAT(trap_gate.offset_mid, t::Eq(0x5678u));
  EXPECT_THAT(trap_gate.offset_high, t::Eq(0x00001234u));
  EXPECT_THAT(trap_gate.reserved, t::Eq(0u));
  EXPECT_THAT(IdtDecodeGateOffset(trap_gate), t::Eq(0x0000123456789ABCULL));
}

TEST(IdtTest, VectorErrorCodeAndExceptionMnemonicsMatchX8664Spec) {
  ResetEnv();
  for (int v = 0; v < kIdtEntryCount; ++v) {
    const bool expected_err =
        (v == 8 || v == 10 || v == 11 || v == 12 || v == 13 || v == 14 ||
         v == 17 || v == 21 || v == 29 || v == 30);
    EXPECT_THAT(IdtVectorPushesErrorCode(v), t::Eq(expected_err));
  }

  EXPECT_THAT(std::string(IdtExceptionMnemonic(kVectorDivideError)),
              t::Eq("#DE"));
  EXPECT_THAT(std::string(IdtExceptionMnemonic(kVectorNmi)), t::Eq("#NMI"));
  EXPECT_THAT(std::string(IdtExceptionMnemonic(kVectorBreakpoint)),
              t::Eq("#BP"));
  EXPECT_THAT(std::string(IdtExceptionMnemonic(kVectorDoubleFault)),
              t::Eq("#DF"));
  EXPECT_THAT(std::string(IdtExceptionMnemonic(kVectorGeneralProtection)),
              t::Eq("#GP"));
  EXPECT_THAT(std::string(IdtExceptionMnemonic(kVectorPageFault)),
              t::Eq("#PF"));
}

TEST(IdtTest, IdtInitPopulatesAll256EntriesAndLoadsIdtr) {
  ResetEnv();
  EXPECT_FALSE(IdtIsInitialized());
  EXPECT_THAT(IdtGetHostLoadCountForTest(), t::Eq(0));

  IdtInit();
  EXPECT_TRUE(IdtIsInitialized());
  EXPECT_THAT(IdtGetHostLoadCountForTest(), t::Eq(1));

  const IdtPointer idtr = IdtGetPointer();
  EXPECT_THAT(idtr.limit,
              t::Eq(static_cast<uint16_t>(kIdtEntryCount * 16 - 1)));
  EXPECT_THAT(idtr.base, t::Eq(reinterpret_cast<uint64_t>(IdtGetGate(0))));

  for (int v = 0; v < kIdtEntryCount; ++v) {
    const IdtGateDescriptor* const gate = IdtGetGate(v);
    ASSERT_THAT(gate, t::NotNull());
    EXPECT_THAT(gate->selector, t::Eq(kIdtKernelCodeSelector));
    if (v != kVectorDoubleFault && v != kVectorNmi &&
        v != kVectorMachineCheck) {
      EXPECT_THAT(gate->ist, t::Eq(0u)) << "vector " << v;
    }
    EXPECT_THAT(gate->type_attr, t::Eq(kIdtInterruptGateAttr));
    EXPECT_THAT(gate->reserved, t::Eq(0u));
    EXPECT_THAT(IdtDecodeGateOffset(*gate), t::Eq(g_isr_stub_table[v]));
    EXPECT_THAT(IdtGetHandler(v), t::IsNull());
  }

  IdtLoad();
  EXPECT_THAT(IdtGetHostLoadCountForTest(), t::Eq(2));
}

TEST(IdtTest, DoubleFaultNmiAndMachineCheckRunOnIstStacksOfTheirOwn) {
  ResetEnv();
  IdtInit();
  EXPECT_THAT(IdtGetGate(kVectorDoubleFault)->ist, t::Eq(kIstDoubleFault));
  EXPECT_THAT(IdtGetGate(kVectorNmi)->ist, t::Eq(kIstNmi));
  EXPECT_THAT(IdtGetGate(kVectorMachineCheck)->ist, t::Eq(kIstMachineCheck));
  // Distinct slots among the ones that every CPU's TSS fills in.
  EXPECT_THAT((std::vector<int>{kIstDoubleFault, kIstNmi, kIstMachineCheck}),
              t::UnorderedElementsAre(1, 2, 3));
  EXPECT_THAT(kIstStackCount, t::Eq(3));
}

TEST(IdtTest, DispatchHandlesNmiCustomExceptionHandlersAndFrameMutations) {
  ResetEnv();
  IdtInit();

  // Vector 2 (NMI) without a custom handler returns cleanly without EOI or
  // panic.
  InterruptFrame nmi_frame = {};
  nmi_frame.vector = kVectorNmi;
  IdtDispatch(&nmi_frame);
  EXPECT_THAT(g_env.eoi_count.load(), t::Eq(0));

  // Vector 2 (NMI) with a custom handler invokes the handler and does not send
  // EOI.
  static int nmi_hits = 0;
  nmi_hits = 0;
  IdtRegisterHandler(kVectorNmi, [](InterruptFrame* const frame) {
    EXPECT_THAT(frame->vector, t::Eq(static_cast<uint64_t>(kVectorNmi)));
    ++nmi_hits;
  });
  IdtDispatch(&nmi_frame);
  EXPECT_THAT(nmi_hits, t::Eq(1));
  EXPECT_THAT(g_env.eoi_count.load(), t::Eq(0));
  IdtUnregisterHandler(kVectorNmi);
  EXPECT_THAT(IdtGetHandler(kVectorNmi), t::IsNull());

  // Custom handler on Vector 3 (#BP) can inspect and mutate InterruptFrame
  // registers (simulating end-to-end register preservation / modification).
  IdtRegisterHandler(kVectorBreakpoint, [](InterruptFrame* const frame) {
    EXPECT_THAT(frame->vector, t::Eq(static_cast<uint64_t>(kVectorBreakpoint)));
    EXPECT_THAT(frame->rax, t::Eq(0xCAFEBABEULL));
    frame->rax = 0xDEADC0DEULL;
    frame->r15 = 0x123456789ABCDEF0ULL;
    frame->rip += 2;
  });

  InterruptFrame bp_frame = {};
  bp_frame.vector = kVectorBreakpoint;
  bp_frame.rax = 0xCAFEBABEULL;
  bp_frame.rip = 0x100000ULL;
  IdtDispatch(&bp_frame);
  EXPECT_THAT(bp_frame.rax, t::Eq(0xDEADC0DEULL));
  EXPECT_THAT(bp_frame.r15, t::Eq(0x123456789ABCDEF0ULL));
  EXPECT_THAT(bp_frame.rip, t::Eq(0x100002ULL));
  EXPECT_THAT(g_env.eoi_count.load(), t::Eq(0));
  IdtUnregisterHandler(kVectorBreakpoint);
}

TEST(IdtTest, DispatchHandlesLocalApicTimerWakeupIpiSpuriousAndCustomVectors) {
  ResetEnv();
  IdtInit();

  CpuLocal cpu = {};
  cpu.self = &cpu;
  cpu.cpu_id = 2;
  cpu.apic_id = 5;
  cpu.online = true;
  BindCpuLocal(&cpu);
  EXPECT_THAT(CurrentCpu(), t::Eq(&cpu));
  EXPECT_THAT(CurrentCpuId(), t::Eq(2));

  // Vector 0x20 (Local APIC Timer): increments timer_ticks and sends EOI.
  static int timer_handler_calls = 0;
  timer_handler_calls = 0;
  IdtRegisterHandler(kVectorApicTimer, [](InterruptFrame* const frame) {
    EXPECT_THAT(frame->vector, t::Eq(static_cast<uint64_t>(kVectorApicTimer)));
    ++timer_handler_calls;
  });
  InterruptFrame timer_frame = {};
  timer_frame.vector = kVectorApicTimer;
  IdtDispatch(&timer_frame);
  IdtDispatch(&timer_frame);
  EXPECT_THAT(cpu.timer_ticks.load(), t::Eq(2));
  EXPECT_THAT(cpu.ipi_count.load(), t::Eq(0));
  EXPECT_THAT(timer_handler_calls, t::Eq(2));
  EXPECT_THAT(g_env.eoi_count.load(), t::Eq(2));

  // Vector 0x21 (Wakeup / Reschedule IPI): increments ipi_count and sends EOI.
  static int ipi_handler_calls = 0;
  ipi_handler_calls = 0;
  IdtRegisterHandler(kVectorWakeupIpi, [](InterruptFrame* const frame) {
    EXPECT_THAT(frame->vector, t::Eq(static_cast<uint64_t>(kVectorWakeupIpi)));
    ++ipi_handler_calls;
  });
  InterruptFrame ipi_frame = {};
  ipi_frame.vector = kVectorWakeupIpi;
  IdtDispatch(&ipi_frame);
  EXPECT_THAT(cpu.timer_ticks.load(), t::Eq(2));
  EXPECT_THAT(cpu.ipi_count.load(), t::Eq(1));
  EXPECT_THAT(ipi_handler_calls, t::Eq(1));
  EXPECT_THAT(g_env.eoi_count.load(), t::Eq(3));

  // Vector 0xFF (Local APIC Spurious Interrupt): when unhandled (no handler
  // registered), returns cleanly WITHOUT panicking and WITHOUT sending EOI.
  InterruptFrame spurious_frame = {};
  spurious_frame.vector = kVectorSpurious;
  EXPECT_THAT(IdtGetHandler(kVectorSpurious), t::IsNull());
  IdtDispatch(&spurious_frame);
  EXPECT_THAT(g_env.eoi_count.load(), t::Eq(3));

  // Vector 0xFF with an optional handler registered invokes the handler and
  // still returns WITHOUT sending EOI.
  static int spurious_calls = 0;
  spurious_calls = 0;
  IdtRegisterHandler(kVectorSpurious, [](InterruptFrame* const frame) {
    EXPECT_THAT(frame->vector, t::Eq(static_cast<uint64_t>(kVectorSpurious)));
    ++spurious_calls;
  });
  IdtDispatch(&spurious_frame);
  EXPECT_THAT(spurious_calls, t::Eq(1));
  EXPECT_THAT(g_env.eoi_count.load(), t::Eq(3));
  IdtUnregisterHandler(kVectorSpurious);

  // Unhandled external hardware vector (e.g. 0x50, no handler registered):
  // sends EOI and returns cleanly without panicking.
  InterruptFrame custom_frame = {};
  custom_frame.vector = 0x50;
  EXPECT_THAT(IdtGetHandler(0x50), t::IsNull());
  IdtDispatch(&custom_frame);
  EXPECT_THAT(g_env.eoi_count.load(), t::Eq(4));

  // General external vector (0x50) with handler registered: invokes handler and
  // sends EOI.
  static int custom_irq_calls = 0;
  custom_irq_calls = 0;
  IdtRegisterHandler(0x50, [](InterruptFrame* const frame) {
    EXPECT_THAT(frame->vector, t::Eq(0x50u));
    ++custom_irq_calls;
  });
  IdtDispatch(&custom_frame);
  EXPECT_THAT(custom_irq_calls, t::Eq(1));
  EXPECT_THAT(g_env.eoi_count.load(), t::Eq(5));

  ResetCpuLocalForTest();
  EXPECT_THAT(CurrentCpuOrNull(), t::IsNull());

  // When CurrentCpuOrNull() is nullptr, timer (0x20) and wakeup IPI (0x21)
  // dispatches must still invoke registered handlers and send EOI without
  // dereferencing a null CpuLocal pointer.
  IdtDispatch(&timer_frame);
  IdtDispatch(&ipi_frame);
  EXPECT_THAT(cpu.timer_ticks.load(), t::Eq(2));
  EXPECT_THAT(cpu.ipi_count.load(), t::Eq(1));
  EXPECT_THAT(timer_handler_calls, t::Eq(3));
  EXPECT_THAT(ipi_handler_calls, t::Eq(2));
  EXPECT_THAT(g_env.eoi_count.load(), t::Eq(7));
}

TEST(IdtTest, LegacyPicVectorsSkipLocalApicEoiAndStillCallHandlers) {
  static_assert(kVectorLegacyPicBase >= kCpuExceptionCount);
  static_assert(kVectorLegacyPicBase + kLegacyPicVectorCount <=
                kVectorSpurious);
  static_assert(kVectorApicTimer < kVectorLegacyPicBase &&
                kVectorWakeupIpi < kVectorLegacyPicBase);
  ResetEnv();
  IdtInit();

  // Spurious IRQ7/IRQ15 and any other PIC vector: no panic and no EOI.
  for (int v = kVectorLegacyPicBase;
       v < kVectorLegacyPicBase + kLegacyPicVectorCount; ++v) {
    InterruptFrame frame = {};
    frame.vector = static_cast<uint64_t>(v);
    IdtDispatch(&frame);
  }
  EXPECT_THAT(g_env.eoi_count.load(), t::Eq(0));

  // A registered handler (e.g. a future driver that unmasks a PIC line) runs
  // and still gets no Local APIC EOI.
  static int irq7_calls = 0;
  irq7_calls = 0;
  constexpr int kSpuriousIrq7Vector = kVectorLegacyPicBase + 7;
  IdtRegisterHandler(kSpuriousIrq7Vector, [](InterruptFrame* const frame) {
    EXPECT_THAT(frame->vector,
                t::Eq(static_cast<uint64_t>(kVectorLegacyPicBase + 7)));
    ++irq7_calls;
  });
  InterruptFrame irq7_frame = {};
  irq7_frame.vector = kSpuriousIrq7Vector;
  IdtDispatch(&irq7_frame);
  EXPECT_THAT(irq7_calls, t::Eq(1));
  EXPECT_THAT(g_env.eoi_count.load(), t::Eq(0));
  IdtUnregisterHandler(kSpuriousIrq7Vector);

  // The vectors just outside the PIC range are ordinary external vectors.
  InterruptFrame below_frame = {};
  below_frame.vector = kVectorLegacyPicBase - 1;
  IdtDispatch(&below_frame);
  InterruptFrame above_frame = {};
  above_frame.vector = kVectorLegacyPicBase + kLegacyPicVectorCount;
  IdtDispatch(&above_frame);
  EXPECT_THAT(g_env.eoi_count.load(), t::Eq(2));
}

TEST(IdtTest, ConcurrentMultiThreadedPerCpuDispatchUnderTsan) {
  ResetEnv();
  IdtInit();

  constexpr int kThreadCount = 4;
  constexpr int kTicksPerThread = 400;
  constexpr int kIpisPerThread = 250;
  constexpr int kSpuriousPerThread = 50;

  alignas(64) CpuLocal locals[kThreadCount] = {};
  for (int i = 0; i < kThreadCount; ++i) {
    locals[i].self = &locals[i];
    locals[i].cpu_id = i;
    locals[i].apic_id = static_cast<uint8_t>(i + 10);
    locals[i].online = true;
  }

  std::atomic<int> ready{0};
  std::vector<std::thread> workers;
  workers.reserve(kThreadCount);
  for (int i = 0; i < kThreadCount; ++i) {
    workers.emplace_back([i, &locals, &ready]() {
      BindCpuLocal(&locals[i]);
      ready.fetch_add(1, std::memory_order_acq_rel);
      while (ready.load(std::memory_order_acquire) < kThreadCount) {
        asm volatile("pause" : : : "memory");
      }

      for (int tick = 0; tick < kTicksPerThread; ++tick) {
        EXPECT_THAT(CurrentCpu(), t::Eq(&locals[i]));
        EXPECT_THAT(CurrentCpuId(), t::Eq(i));
        InterruptFrame frame = {};
        frame.vector = kVectorApicTimer;
        IdtDispatch(&frame);
      }
      for (int p = 0; p < kIpisPerThread; ++p) {
        InterruptFrame frame = {};
        frame.vector = kVectorWakeupIpi;
        IdtDispatch(&frame);
      }
      for (int s = 0; s < kSpuriousPerThread; ++s) {
        InterruptFrame frame = {};
        frame.vector = kVectorSpurious;
        IdtDispatch(&frame);
      }
      ResetCpuLocalForTest();
    });
  }

  for (std::thread& worker : workers) {
    worker.join();
  }

  for (int i = 0; i < kThreadCount; ++i) {
    EXPECT_THAT(locals[i].timer_ticks.load(), t::Eq(kTicksPerThread));
    EXPECT_THAT(locals[i].ipi_count.load(), t::Eq(kIpisPerThread));
  }
  EXPECT_THAT(g_env.eoi_count.load(),
              t::Eq(kThreadCount * (kTicksPerThread + kIpisPerThread)));
}

TEST(IdtDeathTest, UnhandledCpuExceptionsLogDiagnosticsAndPanic) {
  ResetEnv();
  IdtInit();

  // The same message goes to both consoles.
  for (const PanicConsole console : {PanicConsole::kUart, PanicConsole::kVga}) {
    g_env.stderr_console = console;
    EXPECT_DEATH(
        {
          CpuLocal cpu = {};
          cpu.self = &cpu;
          cpu.cpu_id = 3;
          BindCpuLocal(&cpu);
          IdtSetHostCr2ForTest(0xDEADBEEF12340000ULL);
          InterruptFrame pf_frame = {};
          pf_frame.vector = kVectorPageFault;
          pf_frame.error_code = 0x2;
          pf_frame.rip = 0xFFFFFFFF80104567ULL;
          pf_frame.rsp = 0x207FF0ULL;
          pf_frame.rflags = 0x202ULL;
          IdtDispatch(&pf_frame);
        },
        "^\\[FATAL\\] Unhandled CPU exception 14 \\(#PF\\) err=0x2 "
        "rip=0xFFFFFFFF80104567 rsp=0x207FF0 rflags=0x202 "
        "cr2=0xDEADBEEF12340000 cpu=3\n$");
  }

  g_env.stderr_console = PanicConsole::kUart;
  EXPECT_DEATH(
      {
        ResetCpuLocalForTest();
        IdtSetHostCr2ForTest(0);
        InterruptFrame de_frame = {};
        de_frame.vector = kVectorDivideError;
        de_frame.error_code = 0;
        de_frame.rip = 0;
        de_frame.rsp = 0;
        de_frame.rflags = 0;
        IdtDispatch(&de_frame);
      },
      "^\\[FATAL\\] Unhandled CPU exception 0 \\(#DE\\) err=0x0 rip=0x0 "
      "rsp=0x0 rflags=0x0 cr2=0x0 cpu=unbound\n$");
}

TEST(IdtTest, EnterPanicLetsOnlyTheFirstCpuPrint) {
  std::atomic<uint64_t> owner{0};
  int depth = 0;

  // Many CPUs panic at the same time: exactly one of them owns the panic.
  constexpr int kCpuCount = 8;
  std::atomic<int> owner_count{0};
  std::atomic<int> silent_count{0};
  std::vector<std::thread> cpus;
  cpus.reserve(kCpuCount);
  for (int i = 0; i < kCpuCount; ++i) {
    cpus.emplace_back([i, &owner, &depth, &owner_count, &silent_count]() {
      const uint64_t token = 0x1000 + 64 * static_cast<uint64_t>(i);
      const internal::PanicRole role =
          internal::EnterPanic(&owner, &depth, token);
      if (role == internal::PanicRole::kOwner) {
        owner_count.fetch_add(1);
      } else if (role == internal::PanicRole::kSilent) {
        silent_count.fetch_add(1);
      }
    });
  }
  for (std::thread& cpu : cpus) {
    cpu.join();
  }
  EXPECT_THAT(owner_count.load(), t::Eq(1));
  EXPECT_THAT(silent_count.load(), t::Eq(kCpuCount - 1));
  EXPECT_THAT(depth, t::Eq(1));

  // The owner panicking again prints to the UART only, and a third time not
  // at all; other CPUs stay silent.
  const uint64_t owner_token = owner.load();
  EXPECT_THAT(internal::EnterPanic(&owner, &depth, owner_token),
              t::Eq(internal::PanicRole::kNestedOwner));
  EXPECT_THAT(internal::EnterPanic(&owner, &depth, owner_token + 64),
              t::Eq(internal::PanicRole::kSilent));
  EXPECT_THAT(internal::EnterPanic(&owner, &depth, owner_token),
              t::Eq(internal::PanicRole::kSilent));
  EXPECT_THAT(owner.load(), t::Eq(owner_token));
}

// In host builds, PANIC() and failed CHECKs (check.h) print one line to stderr
// and exit with status 134 instead of taking the kernel panic path.
TEST(IdtDeathTest, PanicAndFailedCheckPrintFatalMessageAndExit) {
  constexpr const char* kPanicMessage =
      "^\\[FATAL\\] Layer::Method\\(\\) is not supported: handled elsewhere at "
      ".*idt_test\\.cpp:[0-9]+\n$";
  EXPECT_EXIT(PANIC("Layer::Method() is not supported: handled elsewhere"),  //
              t::ExitedWithCode(134),                                        //
              kPanicMessage);

  const bool invariant_holds = false;
  constexpr const char* kCheckMessage =
      "^\\[FATAL\\] Check failed: invariant_holds at "
      ".*idt_test\\.cpp:[0-9]+\n$";
  EXPECT_EXIT(CHECK(invariant_holds), t::ExitedWithCode(134), kCheckMessage);
}

TEST(IdtDeathTest, PreconditionViolationsTriggerDcheck) {
  ResetEnv();
  InterruptFrame frame = {};

  // Calling IDT accessors or IdtDispatch before IdtInit() must trigger DCHECK.
  EXPECT_DEATH(IdtLoad(), "Check failed");
  EXPECT_DEATH(IdtGetGate(0), "Check failed");
  EXPECT_DEATH(IdtGetPointer(), "Check failed");
  EXPECT_DEATH(IdtRegisterHandler(3, [](InterruptFrame* const) {}),
               "Check failed");
  EXPECT_DEATH(IdtUnregisterHandler(3), "Check failed");
  EXPECT_DEATH(IdtGetHandler(3), "Check failed");
  EXPECT_DEATH(IdtDispatch(&frame), "Check failed");

  // Invalid IdtEncodeGate parameters must trigger DCHECK.
  EXPECT_DEATH(
      IdtEncodeGate(0, kIdtKernelCodeSelector, 0, kIdtInterruptGateAttr),
      "Check failed");
  EXPECT_DEATH(IdtEncodeGate(0x0001000000001000ULL,   //
                             kIdtKernelCodeSelector,  //
                             0,                       //
                             kIdtInterruptGateAttr),
               "Check failed");
  EXPECT_DEATH(IdtEncodeGate(0xFFFEFFFFFFFF0000ULL,   //
                             kIdtKernelCodeSelector,  //
                             0,                       //
                             kIdtInterruptGateAttr),
               "Check failed");
  EXPECT_DEATH(IdtEncodeGate(0x1000, 0, 0, kIdtInterruptGateAttr),
               "Check failed");
  EXPECT_DEATH(
      IdtEncodeGate(0x1000, kIdtKernelCodeSelector, 8, kIdtInterruptGateAttr),
      "Check failed");
  EXPECT_DEATH(IdtEncodeGate(0x1000, kIdtKernelCodeSelector, 0, 0x0E),
               "Check failed");
  EXPECT_DEATH(IdtEncodeGate(0x1000, kIdtKernelCodeSelector, 0, 0x9E),
               "Check failed");
  EXPECT_DEATH(IdtEncodeGate(0x1000, kIdtKernelCodeSelector, 0, 0x89),
               "Check failed");

  // Out-of-bounds vector queries must trigger DCHECK.
  EXPECT_DEATH(IdtVectorPushesErrorCode(-1), "Check failed");
  EXPECT_DEATH(IdtVectorPushesErrorCode(256), "Check failed");
  EXPECT_DEATH(IdtExceptionMnemonic(-1), "Check failed");
  EXPECT_DEATH(IdtExceptionMnemonic(32), "Check failed");

  IdtInit();
  EXPECT_DEATH(IdtGetGate(-1), "Check failed");
  EXPECT_DEATH(IdtGetGate(256), "Check failed");
  EXPECT_DEATH(IdtRegisterHandler(-1, [](InterruptFrame* const) {}),
               "Check failed");
  EXPECT_DEATH(IdtRegisterHandler(256, [](InterruptFrame* const) {}),
               "Check failed");
  EXPECT_DEATH(IdtRegisterHandler(3, nullptr), "Check failed");
  EXPECT_DEATH(IdtUnregisterHandler(-1), "Check failed");
  EXPECT_DEATH(IdtUnregisterHandler(256), "Check failed");
  EXPECT_DEATH(IdtGetHandler(-1), "Check failed");
  EXPECT_DEATH(IdtGetHandler(256), "Check failed");
  EXPECT_DEATH(IdtDispatch(nullptr), "Check failed");
  frame.vector = 256;
  EXPECT_DEATH(IdtDispatch(&frame), "Check failed");
}

}  // namespace
}  // namespace protos
