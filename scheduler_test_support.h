#ifndef PROTOS_SCHEDULER_TEST_SUPPORT_H_
#define PROTOS_SCHEDULER_TEST_SUPPORT_H_

// Shared fixtures for the scheduler unit tests (core_runqueue_scheduler_test,
// work_stealing_scheduler_test, join_lifecycle_scheduler_test,
// preemptive_scheduler_test, task_test and stress_test).
//
// The scheduler code calls into other kernel modules. These tests replace the
// modules with host fakes whose state lives in `g_env`. The fakes are plain
// (non-inline, non-weak) definitions in two source files, and the Makefile
// links each test with the ones it needs (`TEST_EXTRA_SRCS_<test>`):
// - scheduler_test_fakes.cpp fakes the PMM, heap, SMP, task stack
//   (task_stack.h), address space switching (`ActivateAddressSpace()`,
//   address_space.h), FPU register switching (`FpuSave()` and `FpuRestore()`,
//   fpu.h), UART and VGA functions that the scheduler code and these tests
//   call. Every test that includes this header links it; none of them links
//   pmm.cpp, heap.cpp, smp.cpp, task_stack.cpp, address_space.cpp, fpu.cpp,
//   uart.cpp or vga.cpp. Its task stacks come from the fake PMM and have no
//   guard pages.
// - scheduler_test_idt_task_fakes.cpp fakes the parts of idt.h and task.h
//   that SetupEnv() and the scheduler layers use. Only tests that link
//   neither idt.cpp nor task.cpp use it; task_test and stress_test link
//   the real modules instead.
// A test that links both a fake and the real module fails to link with a
// duplicate symbol error rather than silently using one of them.
//
// These tests cannot preempt a task asynchronously. Host builds emulate
// RFLAGS.IF with a thread-local flag (interrupts.h), so an asynchronous host
// "interrupt" would be a POSIX signal whose handler switches tasks. But the
// emulated save-and-disable is not atomic with respect to signals, and a task
// can resume on another host thread, which has its own flag. Switching stacks
// in a signal handler is also not async-signal-safe, and ASan and TSan don't
// support it. Instead, the tests inject interrupts synchronously at chosen
// points, through IdtDispatch() or, for a single layer, through its
// OnTimerInterrupt() and OnWakeupIpi() (TaskSchedulerInternals). Real
// asynchronous timer interrupts are covered by the boot self-tests and the
// multi-CPU stress test (selftest/), which test_boot.sh runs in QEMU, or with
// KVM=1 under KVM.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <ostream>
#include <string>
#include <vector>

#include "check.h"
#include "fpu.h"
#include "heap.h"
#include "idt.h"
#include "pmm.h"
#include "smp.h"
#include "spinlock.h"
#include "task.h"
#include "uart.h"
#include "vga.h"

namespace protos {

constexpr int64_t kInitialFakeFreeFrames = 16384;
constexpr int64_t kInitialFakeHeapBytes = 4 * 1024 * 1024;
constexpr uintptr_t kFakeStackBase = 0x200000;

// A call of the fake FpuSave() or FpuRestore() (FakeSchedulerEnv::fpu_calls):
// the operation, the CPU that made it, and the state.
enum class FakeFpuOp : uint8_t { kSave, kRestore };

struct FakeFpuCall {
  FakeFpuOp op = FakeFpuOp::kSave;
  int cpu = -1;
  const FpuState* state = nullptr;

  bool operator==(const FakeFpuCall& other) const = default;
};

// Prints `call` in gtest messages.
inline void PrintTo(const FakeFpuCall& call, std::ostream* const os) {
  *os << (call.op == FakeFpuOp::kSave ? "Save" : "Restore") << "(cpu "
      << call.cpu << ", " << static_cast<const void*>(call.state) << ")";
}

// State of the fakes. SetupEnv() resets it.
struct FakeSchedulerEnv {
  int cpu_count = 1;
  // Returned by the fake IdtIsInitialized(), so it only matters in tests that
  // link scheduler_test_idt_task_fakes.cpp.
  bool idt_initialized = true;
  // Make the fake PmmAllocFrames() and KmallocAligned() fail.
  bool fail_pmm_alloc = false;
  bool fail_kmalloc = false;
  std::unique_ptr<CpuLocal[]> cpu_locals;
  std::unique_ptr<CpuInfo[]> cpu_infos;
  // Makes the fake SmpGetCpuLocal(i) return nullptr while set.
  std::unique_ptr<bool[]> null_cpu_local;

  std::atomic<int64_t> allocated_frames{0};
  std::atomic<int64_t> allocated_heap_bytes{0};
  std::atomic<int64_t> allocated_heap_blocks{0};

  std::atomic<int64_t> eoi_count{0};
  std::unique_ptr<std::atomic<int64_t>[]> ipi_sent_count;
  std::unique_ptr<std::atomic<uint8_t>[]> last_ipi_vector;

  // The address space that the fake ActivateAddressSpace() last switched each
  // CPU to (nullptr: the kernel's page tables), and how many switches it made.
  std::unique_ptr<std::atomic<const AddressSpace*>[]> active_address_space;
  std::atomic<int64_t> address_space_switches{0};

  // The FpuState whose values the registers of each CPU hold: the one that
  // the fake FpuRestore() last loaded there (nullptr: none yet). The fake
  // FpuSave() counts each save into another state than the CPU's in
  // `fpu_save_mismatches`: the scheduler saved the registers of another task
  // into it, because it missed a load or loaded before it saved.
  std::unique_ptr<std::atomic<const FpuState*>[]> loaded_fpu_state;
  std::atomic<int64_t> fpu_save_mismatches{0};
  // Every call of the fake FpuSave() and FpuRestore(), in order, guarded by
  // `log_mutex`.
  std::vector<FakeFpuCall> fpu_calls;

  // Output of the fake UART and VGA functions, guarded by `log_mutex`.
  std::mutex log_mutex;
  std::string uart_log;
  std::string vga_log;
  std::string uart_panic_log;
  std::string vga_panic_log;
};

inline FakeSchedulerEnv g_env;

// Destroys the scheduler of a previous TaskInit() (if any), resets the fakes
// and the IDT, and simulates `num_cpus` online CPUs. On return CPU 0 is bound
// to the calling thread, its simulated interrupts are enabled and the IDT is
// initialized, so the test may call TaskInit() or create scheduler layers.
inline void SetupEnv(const int num_cpus = 1) {
  CHECK(num_cpus >= 1);
  TaskResetForTest();
  IdtResetForTest();
  ResetCpuLocalForTest();
  SetInterruptsEnabledForTest(true);

  g_env.fail_pmm_alloc = false;
  g_env.fail_kmalloc = false;
  g_env.cpu_count = num_cpus;
  g_env.allocated_frames.store(0, std::memory_order_relaxed);
  g_env.allocated_heap_bytes.store(0, std::memory_order_relaxed);
  g_env.allocated_heap_blocks.store(0, std::memory_order_relaxed);
  g_env.eoi_count.store(0, std::memory_order_relaxed);
  g_env.address_space_switches.store(0, std::memory_order_relaxed);
  g_env.fpu_save_mismatches.store(0, std::memory_order_relaxed);
  {
    const std::lock_guard<std::mutex> lock(g_env.log_mutex);
    g_env.fpu_calls.clear();
    g_env.uart_log.clear();
    g_env.vga_log.clear();
    g_env.uart_panic_log.clear();
    g_env.vga_panic_log.clear();
  }

  g_env.cpu_locals.reset(new CpuLocal[num_cpus]());
  g_env.cpu_infos.reset(new CpuInfo[num_cpus]());
  g_env.null_cpu_local.reset(new bool[num_cpus]());
  g_env.ipi_sent_count.reset(new std::atomic<int64_t>[num_cpus]());
  g_env.last_ipi_vector.reset(new std::atomic<uint8_t>[num_cpus]());
  g_env.active_address_space.reset(
      new std::atomic<const AddressSpace*>[num_cpus]());
  g_env.loaded_fpu_state.reset(new std::atomic<const FpuState*>[num_cpus]());

  for (int i = 0; i < num_cpus; ++i) {
    const uintptr_t stack_base =
        kFakeStackBase + static_cast<uintptr_t>(i) * kTaskStackSize;
    const uintptr_t stack_top = stack_base + kTaskStackSize;

    CpuLocal& local = g_env.cpu_locals[i];
    local.self = &local;
    local.cpu_id = i;
    local.apic_id = static_cast<uint32_t>(i);
    local.online = true;
    local.stack_base = stack_base;
    local.stack_top = stack_top;

    CpuInfo& info = g_env.cpu_infos[i];
    info.apic_id = static_cast<uint32_t>(i);
    info.acpi_processor_id = static_cast<uint32_t>(i);
    info.observed_apic_id = static_cast<uint32_t>(i);
    info.is_bsp = (i == 0);
    info.online = true;
    info.long_mode_active = true;
    info.stack_base = stack_base;
    info.stack_top = stack_top;
  }

  IdtInit();
  BindCpuLocal(&g_env.cpu_locals[0]);
}

inline void ResetTestEnv() { SetupEnv(1); }

inline void NoopTask(void* const /*arg*/) {}

// Mocks every `TaskScheduler` method. It mocks the private scheduler internals
// in its public section too, so that tests can set expectations on them.
class MockTaskScheduler : public TaskScheduler {
 public:
  MOCK_METHOD(void, Init, (), (override));
  MOCK_METHOD(Task*, CreateTask, (TaskFn, void*, int64_t), (override));
  MOCK_METHOD(Task*,                          //
              CreateTaskOnCpu,                //
              (TaskFn, void*, int64_t, int),  //
              (override));
  MOCK_METHOD(void, Yield, (), (override));
  MOCK_METHOD(void, Join, (Task*), (override));
  MOCK_METHOD(void, Detach, (Task*), (override));
  MOCK_METHOD(int, ReapZombies, (), (override));
  MOCK_METHOD(bool, PollIdleCpu, (int), (override));
  MOCK_METHOD(int, RunqueueLoad, (int), (const, override));
  MOCK_METHOD(bool, IsCpuBusy, (int), (const, override));
  MOCK_METHOD(bool, StealTask, (int, int), (override));
  MOCK_METHOD(void, SetPreemptEnabled, (bool), (override));
  MOCK_METHOD(bool, IsPreemptEnabled, (), (const, override));
  MOCK_METHOD(void, OnTimerInterrupt, (InterruptFrame*), (override));
  MOCK_METHOD(void, OnWakeupIpi, (InterruptFrame*), (override));
  MOCK_METHOD(Task*, CurrentTask, (), (const, override));
  MOCK_METHOD(void, SetAddressSpace, (AddressSpace*), (override));
  MOCK_METHOD(void, SetFpuState, (FpuState*), (override));
};

}  // namespace protos

#endif  // PROTOS_SCHEDULER_TEST_SUPPORT_H_
