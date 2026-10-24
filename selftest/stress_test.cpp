#include "selftest/stress.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "scheduler_test_support.h"

namespace protos {
namespace {

// State of the fake LAPIC timers below.
struct FakeLapicTimers {
  std::mutex mutex;
  uint32_t calibrated_count = 0;
  // Initial counts programmed on each CPU, in order.
  std::vector<std::vector<uint32_t>> programmed_counts;
};

FakeLapicTimers g_lapic_timers;

// CPU whose work the fake SmpRunOnAllCpus() is running on this thread, or -1.
thread_local int g_run_on_all_cpus_cpu = -1;

static void ResetFakeLapicTimers(const uint32_t calibrated_count) {
  const std::lock_guard<std::mutex> lock(g_lapic_timers.mutex);
  g_lapic_timers.calibrated_count = calibrated_count;
  g_lapic_timers.programmed_counts.clear();
}

static std::vector<uint32_t> ProgrammedTimerCounts(const int cpu) {
  const std::lock_guard<std::mutex> lock(g_lapic_timers.mutex);
  if (cpu >= static_cast<int>(g_lapic_timers.programmed_counts.size())) {
    return {};
  }
  return g_lapic_timers.programmed_counts[cpu];
}

}  // namespace

// Fakes of the smp.h LAPIC timer functions that only the stress test uses, so
// scheduler_test_fakes.cpp doesn't provide them. SmpRunOnAllCpus() runs the
// work of every CPU on the calling thread.
uint32_t SmpCalibratedTimerInitialCount() {
  const std::lock_guard<std::mutex> lock(g_lapic_timers.mutex);
  return g_lapic_timers.calibrated_count;
}

void SmpSetLocalApicTimerInitialCount(const uint32_t initial_count) {
  const int cpu =
      (g_run_on_all_cpus_cpu >= 0) ? g_run_on_all_cpus_cpu : CurrentCpuId();
  const std::lock_guard<std::mutex> lock(g_lapic_timers.mutex);
  std::vector<std::vector<uint32_t>>& counts = g_lapic_timers.programmed_counts;
  if (cpu >= static_cast<int>(counts.size())) {
    counts.resize(cpu + 1);
  }
  counts[cpu].push_back(initial_count);
}

void SmpRunOnAllCpus(const SmpWorkFn work_fn, void* const context) {
  CHECK(g_run_on_all_cpus_cpu < 0);
  for (int cpu = 0; cpu < SmpCpuCount(); ++cpu) {
    g_run_on_all_cpus_cpu = cpu;
    work_fn(cpu, context);
  }
  g_run_on_all_cpus_cpu = -1;
}

namespace {

namespace t = ::testing;

constexpr uint32_t kFakeCalibratedTimerCount = 160000;
constexpr uint32_t kFakeBoostedTimerCount = kFakeCalibratedTimerCount / 16;

// Bound on each wait of the host runs. Larger than the default because
// sanitizers and parallel test binaries slow the simulated CPUs down.
constexpr uint64_t kHostWaitTimeoutTscCycles =
    20 * kSelfTestWaitTimeoutTscCycles;

// Host stand-in for `pause` in busy waits: lets the threads of the other
// simulated CPUs run.
static void YieldHostThread() { std::this_thread::yield(); }

// Simulates the LAPIC timer while a task spins without yielding: every 32nd
// call delivers a timer interrupt to the calling CPU, if its interrupts are
// enabled.
static void DeliverSimulatedTimerInterrupt() {
  thread_local int calls = 0;
  if (++calls % 32 != 0 || !IdtIsInitialized() || !AreInterruptsEnabled()) {
    return;
  }
  InterruptFrame timer_frame = {};
  timer_frame.vector = kVectorApicTimer;
  timer_frame.rflags = kRflagsInterruptEnableBit | 0x2;
  IdtDispatch(&timer_frame);
}

// Runs the idle loops of the simulated APs 1 .. cpu_count - 1, one host thread
// each: delivers the wakeup IPIs sent to the AP and runs its tasks through
// TaskPollIdleCpu(). Create it after TaskInit().
class SimulatedAps {
 public:
  // Returns once every AP thread is bound to its CPU. AP `delayed_cpu` waits
  // for `start_delay` before its idle loop starts.
  explicit SimulatedAps(const int cpu_count, const int delayed_cpu = 0,
                        const std::chrono::milliseconds start_delay =
                            std::chrono::milliseconds(0)) {
    threads_.reserve(cpu_count - 1);
    for (int cpu = 1; cpu < cpu_count; ++cpu) {
      const std::chrono::milliseconds delay =
          (cpu == delayed_cpu) ? start_delay : std::chrono::milliseconds(0);
      threads_.emplace_back([this, cpu, delay]() { IdleLoop(cpu, delay); });
    }
    while (ready_.load(std::memory_order_acquire) < cpu_count - 1) {
      std::this_thread::yield();
    }
  }

  ~SimulatedAps() { Stop(); }

  SimulatedAps(const SimulatedAps&) = delete;
  SimulatedAps& operator=(const SimulatedAps&) = delete;

  // Stops and joins the AP threads.
  void Stop() {
    stop_.store(true, std::memory_order_release);
    for (std::thread& thread : threads_) {
      thread.join();
    }
    threads_.clear();
  }

 private:
  void IdleLoop(const int cpu, const std::chrono::milliseconds start_delay) {
    BindCpuLocal(SmpGetCpuLocal(cpu));
    SetInterruptsEnabledForTest(true);
    ready_.fetch_add(1, std::memory_order_acq_rel);
    std::this_thread::sleep_for(start_delay);

    int64_t handled_ipis = 0;
    while (!stop_.load(std::memory_order_acquire)) {
      const int64_t sent_ipis =
          g_env.ipi_sent_count[cpu].load(std::memory_order_acquire);
      if (sent_ipis > handled_ipis && IdtIsInitialized() &&
          AreInterruptsEnabled()) {
        handled_ipis = sent_ipis;
        InterruptFrame ipi_frame = {};
        ipi_frame.vector = kVectorWakeupIpi;
        ipi_frame.rflags = kRflagsInterruptEnableBit | 0x2;
        IdtDispatch(&ipi_frame);
      }
      if (!TaskPollIdleCpu(cpu)) {
        std::this_thread::yield();
      }
    }
    ResetCpuLocalForTest();
  }

  std::atomic<int> ready_{0};
  std::atomic<bool> stop_{false};
  std::vector<std::thread> threads_;
};

// Installs the host stress-test hooks and fake LAPIC timers. Every run uses a
// fixed seed, so that a failing run can be reproduced.
class StressTest : public t::Test {
 protected:
  void SetUp() override {
    StressTestHooks hooks;
    hooks.cpu_relax = YieldHostThread;
    hooks.non_yielding_spin = DeliverSimulatedTimerInterrupt;
    SetStressTestHooksForTest(hooks);
    ResetFakeLapicTimers(kFakeCalibratedTimerCount);
  }

  void TearDown() override {
    SetStressTestHooksForTest({});
    TaskResetForTest();
  }
};

using StressDeathTest = StressTest;

TEST(CpuMask128Test, FirstCpusContainsAndContainsAll) {
  EXPECT_THAT(CpuMask128::FirstCpus(0), t::Eq(CpuMask128{}));
  EXPECT_THAT(CpuMask128::FirstCpus(1), t::Eq(CpuMask128{0x1, 0}));
  EXPECT_THAT(CpuMask128::FirstCpus(63), t::Eq(CpuMask128{~0ULL >> 1, 0}));
  EXPECT_THAT(CpuMask128::FirstCpus(64), t::Eq(CpuMask128{~0ULL, 0}));
  EXPECT_THAT(CpuMask128::FirstCpus(68), t::Eq(CpuMask128{~0ULL, 0xF}));
  EXPECT_THAT(CpuMask128::FirstCpus(128), t::Eq(CpuMask128{~0ULL, ~0ULL}));

  const CpuMask128 mask{0x5, 0x1};  // CPUs 0, 2 and 64.
  EXPECT_TRUE(mask.Contains(0));
  EXPECT_FALSE(mask.Contains(1));
  EXPECT_TRUE(mask.Contains(2));
  EXPECT_TRUE(mask.Contains(64));
  EXPECT_FALSE(mask.Contains(65));
  EXPECT_FALSE(mask.Contains(-1));
  EXPECT_FALSE(mask.Contains(128));
  EXPECT_TRUE(mask.ContainsAll(CpuMask128{0x4, 0x1}));
  EXPECT_TRUE(mask.ContainsAll(CpuMask128{}));
  EXPECT_FALSE(mask.ContainsAll(CpuMask128{0x2, 0}));
  EXPECT_FALSE(mask.ContainsAll(CpuMask128{0x1, 0x2}));
}

TEST(CpuMask128Test, AtomicAddReportsNewCpusAndIgnoresOutOfRange) {
  AtomicCpuMask128 mask;
  EXPECT_TRUE(mask.Add(3));
  EXPECT_FALSE(mask.Add(3));
  EXPECT_TRUE(mask.Add(kMaxSupportedCpus - 1));
  EXPECT_FALSE(mask.Add(-1));
  EXPECT_FALSE(mask.Add(kMaxSupportedCpus));
  const CpuMask128 loaded = mask.Load();
  EXPECT_TRUE(loaded.Contains(3));
  EXPECT_TRUE(loaded.Contains(kMaxSupportedCpus - 1));
  EXPECT_THAT(
      __builtin_popcountll(loaded.low) + __builtin_popcountll(loaded.high),
      t::Eq(2));
  mask.Clear();
  EXPECT_THAT(mask.Load(), t::Eq(CpuMask128{}));
}

TEST(TscDeadlineTest, ExpiresOnceTheTimeoutHasElapsed) {
  EXPECT_TRUE(TscDeadline(0).Expired());
  // Timeouts above INT64_MAX never expire.
  EXPECT_FALSE(TscDeadline(~0ULL).Expired());

  const TscDeadline short_deadline(1000);
  const TscDeadline long_deadline(kSelfTestWaitTimeoutTscCycles);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_TRUE(short_deadline.Expired());
  EXPECT_FALSE(long_deadline.Expired());
}

TEST(TickWaitTimeoutTscCyclesTest, CoversTheTicksAtTheMeasuredRate) {
  // The usbkbd idle window: 550 ticks at 250 Hz, 2.2 s, on a 5 GHz TSC
  // (20M cycles per tick) is longer than the fixed bound.
  constexpr int64_t kWindowTicks = 550;
  constexpr int64_t kCyclesPerTick = 5'000'000'000LL / kApicTimerTargetHz;
  constexpr uint64_t kWindowCycles =
      static_cast<uint64_t>(kWindowTicks * kCyclesPerTick);
  ASSERT_THAT(kWindowCycles, t::Gt(kSelfTestWaitTimeoutTscCycles));
  EXPECT_THAT(TickWaitTimeoutTscCycles(kWindowTicks, kCyclesPerTick),
              t::Eq(2 * kWindowCycles + kSelfTestWaitTimeoutTscCycles));
  // Other waits keep the fixed bound.
  EXPECT_THAT(TickWaitTimeoutTscCycles(0, kCyclesPerTick),
              t::Eq(kSelfTestWaitTimeoutTscCycles));
  EXPECT_THAT(TickWaitTimeoutTscCycles(kWindowTicks, 0),
              t::Eq(kSelfTestWaitTimeoutTscCycles));
  EXPECT_THAT(TickWaitTimeoutTscCycles(-1, kCyclesPerTick),
              t::Eq(kSelfTestWaitTimeoutTscCycles));
  EXPECT_THAT(TickWaitTimeoutTscCycles(kWindowTicks, -1),
              t::Eq(kSelfTestWaitTimeoutTscCycles));
}

TEST(TickWaitTimeoutTscCyclesTest, SaturatesInsteadOfOverflowing) {
  constexpr uint64_t kMaxWindowCycles =
      (UINT64_MAX - kSelfTestWaitTimeoutTscCycles) / 2;
  constexpr int64_t kTicks = 1000;
  constexpr int64_t kLargestCyclesPerTick =
      static_cast<int64_t>(kMaxWindowCycles / kTicks);
  EXPECT_THAT(TickWaitTimeoutTscCycles(kTicks, kLargestCyclesPerTick),
              t::Eq(2 * static_cast<uint64_t>(kTicks * kLargestCyclesPerTick) +
                    kSelfTestWaitTimeoutTscCycles));
  EXPECT_THAT(TickWaitTimeoutTscCycles(kTicks, kLargestCyclesPerTick + 1),
              t::Eq(UINT64_MAX));
  EXPECT_THAT(TickWaitTimeoutTscCycles(INT64_MAX, INT64_MAX),
              t::Eq(UINT64_MAX));
  // Such a bound never expires.
  EXPECT_FALSE(TscDeadline(TickWaitTimeoutTscCycles(INT64_MAX, 1)).Expired());
}

TEST_F(StressTest, ResolveStressTestSeedKeepsNonZeroSeedsAndHarvestsForZero) {
  SetupEnv(1);
  EXPECT_THAT(ResolveStressTestSeed(1), t::Eq(1u));
  EXPECT_THAT(ResolveStressTestSeed(0xFEEDFACECAFEBEEFULL),
              t::Eq(0xFEEDFACECAFEBEEFULL));

  const uint64_t harvested_a = ResolveStressTestSeed(0);
  const uint64_t harvested_b = ResolveStressTestSeed(0);
  EXPECT_THAT(harvested_a, t::Ne(0u));
  EXPECT_THAT(harvested_b, t::Ne(0u));
  EXPECT_THAT(harvested_a, t::Ne(harvested_b));
}

TEST_F(StressTest, ScopedLapicTimerBoostSpeedsUpAndRestoresEveryCpu) {
  constexpr int kNumCpus = 3;
  SetupEnv(kNumCpus);

  {
    ScopedLapicTimerBoost boost(true);
    for (int cpu = 0; cpu < kNumCpus; ++cpu) {
      EXPECT_THAT(ProgrammedTimerCounts(cpu),
                  t::ElementsAre(kFakeBoostedTimerCount));
    }
    boost.Restore();
  }
  // The destructor doesn't restore the counts a second time.
  for (int cpu = 0; cpu < kNumCpus; ++cpu) {
    EXPECT_THAT(
        ProgrammedTimerCounts(cpu),
        t::ElementsAre(kFakeBoostedTimerCount, kFakeCalibratedTimerCount));
  }

  ResetFakeLapicTimers(kFakeCalibratedTimerCount);
  {
    const ScopedLapicTimerBoost boost(true);
  }
  EXPECT_THAT(
      ProgrammedTimerCounts(kNumCpus - 1),
      t::ElementsAre(kFakeBoostedTimerCount, kFakeCalibratedTimerCount));

  // Neither a disabled boost nor a calibrated count too small to divide by 16
  // reprograms any timer.
  ResetFakeLapicTimers(kFakeCalibratedTimerCount);
  {
    const ScopedLapicTimerBoost boost(false);
  }
  ResetFakeLapicTimers(32);
  {
    const ScopedLapicTimerBoost boost(true);
  }
  for (int cpu = 0; cpu < kNumCpus; ++cpu) {
    EXPECT_THAT(ProgrammedTimerCounts(cpu), t::IsEmpty());
  }
}

TEST_F(StressTest, SingleCpuRandomizedStressRestoresExactBaselines) {
  SetupEnv(1);
  TaskInit();

  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  StressTestConfig config;
  config.seed = 0xABCDEF0123456789ULL;
  config.num_workers = 4;
  config.rounds_per_worker = 12;
  config.max_live_allocs_per_worker = 4;
  config.max_heap_alloc_bytes = 1024;
  config.max_pmm_alloc_frames = 3;
  config.boost_lapic_timer = false;
  config.wait_timeout_tsc_cycles = kHostWaitTimeoutTscCycles;

  const StressTestStats stats = ExecuteRandomMultiCpuStress(config);

  EXPECT_THAT(stats.seed, t::Eq(config.seed));
  EXPECT_THAT(stats.cpu_count, t::Eq(1));
  EXPECT_THAT(stats.cpu_participation_mask, t::Eq(0x1u));
  EXPECT_THAT(stats.cpu_participation_mask_high, t::Eq(0u));
  EXPECT_THAT(stats.cooperative_yields, t::Gt(0));
  EXPECT_THAT(stats.preemptive_spin_bursts, t::Gt(0));
  EXPECT_THAT(stats.timer_preemptions_observed, t::Gt(0));
  EXPECT_THAT(stats.wakeup_ipis_sent, t::Eq(0));
  EXPECT_THAT(stats.task_migrations_observed, t::Eq(0));
  EXPECT_THAT(stats.cross_cpu_steals_verified, t::Eq(0));
  EXPECT_THAT(stats.child_tasks_joined, t::Gt(0));
  EXPECT_THAT(stats.child_tasks_detached, t::Gt(0));
  EXPECT_THAT(stats.zombies_reaped, t::Eq(stats.child_tasks_detached));
  // The workers requested child tasks with both extreme weights.
  EXPECT_THAT(stats.min_child_task_weight, t::Eq(kMinTaskWeight));
  EXPECT_THAT(stats.max_child_task_weight, t::Eq(kMaxTaskWeight));
  EXPECT_THAT(stats.kmalloc_ops, t::Gt(0));
  EXPECT_THAT(stats.kmalloc_aligned_ops, t::Gt(0));
  EXPECT_THAT(stats.cpp_new_delete_ops, t::Gt(0));
  EXPECT_THAT(stats.pmm_alloc_ops, t::Gt(0));
  EXPECT_THAT(stats.payload_verifications, t::Gt(0));
  EXPECT_THAT(stats.payload_corruptions, t::Eq(0));
  EXPECT_THAT(stats.stack_canary_corruptions, t::Eq(0));
  EXPECT_THAT(stats.pmm_free_before, t::Eq(pmm_before));
  EXPECT_THAT(stats.pmm_free_after, t::Eq(pmm_before));
  EXPECT_THAT(stats.heap_free_before, t::Eq(heap_before));
  EXPECT_THAT(stats.heap_free_after, t::Eq(heap_before));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
  EXPECT_THAT(ProgrammedTimerCounts(0), t::IsEmpty());
}

TEST_F(StressTest, FourCpuRandomizedStressAndConsoleEntryPoint) {
  constexpr int kNumCpus = 4;
  SetupEnv(kNumCpus);
  TaskInit();

  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();
  SimulatedAps aps(kNumCpus);

  StressTestConfig config;
  config.seed = 0x55AA55AA12349876ULL;
  config.num_workers = 8;
  config.rounds_per_worker = 16;
  config.max_live_allocs_per_worker = 4;
  config.max_heap_alloc_bytes = 2048;
  config.max_pmm_alloc_frames = 3;
  config.boost_lapic_timer = true;
  config.wait_timeout_tsc_cycles = kHostWaitTimeoutTscCycles;

  const StressTestStats stats = ExecuteRandomMultiCpuStress(config);

  EXPECT_THAT(stats.seed, t::Eq(config.seed));
  EXPECT_THAT(stats.cpu_count, t::Eq(kNumCpus));
  EXPECT_THAT(stats.cpu_participation_mask, t::Eq(0xFu));
  EXPECT_THAT(stats.cooperative_yields, t::Gt(0));
  EXPECT_THAT(stats.preemptive_spin_bursts, t::Gt(0));
  EXPECT_THAT(stats.timer_preemptions_observed, t::Gt(0));
  EXPECT_THAT(stats.wakeup_ipis_sent, t::Gt(0));
  EXPECT_THAT(stats.cross_cpu_steals_verified, t::Eq(kNumCpus - 1));
  EXPECT_THAT(stats.child_tasks_joined, t::Gt(0));
  EXPECT_THAT(stats.child_tasks_detached, t::Gt(0));
  EXPECT_THAT(stats.zombies_reaped, t::Eq(stats.child_tasks_detached));
  EXPECT_THAT(stats.min_child_task_weight, t::Eq(kMinTaskWeight));
  EXPECT_THAT(stats.max_child_task_weight, t::Eq(kMaxTaskWeight));
  EXPECT_THAT(stats.kmalloc_ops, t::Gt(0));
  EXPECT_THAT(stats.kmalloc_aligned_ops, t::Gt(0));
  EXPECT_THAT(stats.cpp_new_delete_ops, t::Gt(0));
  EXPECT_THAT(stats.pmm_alloc_ops, t::Gt(0));
  EXPECT_THAT(stats.payload_verifications, t::Gt(0));
  EXPECT_THAT(stats.payload_corruptions, t::Eq(0));
  EXPECT_THAT(stats.stack_canary_corruptions, t::Eq(0));
  EXPECT_THAT(stats.pmm_free_before, t::Eq(pmm_before));
  EXPECT_THAT(stats.pmm_free_after, t::Eq(pmm_before));
  EXPECT_THAT(stats.heap_free_before, t::Eq(heap_before));
  EXPECT_THAT(stats.heap_free_after, t::Eq(heap_before));
  // The run sped up the LAPIC timer of every CPU and restored it afterwards.
  for (int cpu = 0; cpu < kNumCpus; ++cpu) {
    EXPECT_THAT(
        ProgrammedTimerCounts(cpu),
        t::ElementsAre(kFakeBoostedTimerCount, kFakeCalibratedTimerCount))
        << "CPU " << cpu;
  }

  // The entry point that RunSelfTests() runs: the default config, with console
  // output.
  RunRandomMultiCpuStressTest(0x5EED5EED12345678ULL);
  aps.Stop();

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
  const std::lock_guard<std::mutex> lock(g_env.log_mutex);
  EXPECT_THAT(g_env.uart_log,
              t::HasSubstr("[STRESS] seed=0x5EED5EED12345678\n"));
  EXPECT_THAT(g_env.uart_log,
              t::HasSubstr("[STRESS] random_multicpu_stress: PASS\n"));
  EXPECT_THAT(g_env.uart_log, t::Not(t::HasSubstr("FAIL")));
  EXPECT_THAT(g_env.vga_log,
              t::HasSubstr("\r[STRESS] random_multicpu_stress: PASS #"));
  EXPECT_THAT(g_env.vga_log, t::HasSubstr("seed=0x5EED5EED12345678"));
  EXPECT_THAT(g_env.vga_log, t::Not(t::HasSubstr("FAIL")));
}

TEST_F(StressTest, MinimalConfigOnOneCpuAndOnFourCpusWithALateAp) {
  StressTestConfig config;
  config.num_workers = 1;
  config.rounds_per_worker = 1;
  config.max_live_allocs_per_worker = 1;
  config.max_heap_alloc_bytes = 64;
  config.max_pmm_alloc_frames = 1;
  config.boost_lapic_timer = false;
  config.wait_timeout_tsc_cycles = kHostWaitTimeoutTscCycles;

  // 1. One CPU: the single worker still uses all four allocation kinds.
  {
    SetupEnv(1);
    TaskInit();
    const int64_t pmm_before = PmmFreeFrameCount();
    const int64_t heap_before = HeapTotalFreeBytes();

    config.seed = 0x1122334455667788ULL;
    const StressTestStats stats = ExecuteRandomMultiCpuStress(config);

    EXPECT_THAT(stats.cpu_participation_mask, t::Eq(0x1u));
    EXPECT_THAT(stats.timer_preemptions_observed, t::Gt(0));
    EXPECT_THAT(stats.kmalloc_ops, t::Ge(1));
    EXPECT_THAT(stats.kmalloc_aligned_ops, t::Ge(1));
    EXPECT_THAT(stats.cpp_new_delete_ops, t::Ge(1));
    EXPECT_THAT(stats.pmm_alloc_ops, t::Ge(1));
    EXPECT_THAT(stats.payload_verifications, t::Ge(4));
    EXPECT_THAT(stats.payload_corruptions, t::Eq(0));
    EXPECT_THAT(stats.stack_canary_corruptions, t::Eq(0));
    EXPECT_THAT(stats.pmm_free_after, t::Eq(pmm_before));
    EXPECT_THAT(stats.heap_free_after, t::Eq(heap_before));
  }

  // 2. Four CPUs, one worker, and AP 3 starting its idle loop 15 ms late: the
  //    final steal check keeps waking AP 3 until it has stolen its task, so
  //    all four CPUs still take part.
  {
    constexpr int kNumCpus = 4;
    SetupEnv(kNumCpus);
    TaskInit();
    const int64_t pmm_before = PmmFreeFrameCount();
    const int64_t heap_before = HeapTotalFreeBytes();
    SimulatedAps aps(kNumCpus, 3, std::chrono::milliseconds(15));

    config.seed = 0xDEADC0DEBEEF1234ULL;
    config.max_heap_alloc_bytes = 128;
    const StressTestStats stats = ExecuteRandomMultiCpuStress(config);
    aps.Stop();

    EXPECT_THAT(stats.cpu_count, t::Eq(kNumCpus));
    EXPECT_THAT(stats.cpu_participation_mask, t::Eq(0xFu));
    EXPECT_THAT(stats.cross_cpu_steals_verified, t::Eq(kNumCpus - 1));
    EXPECT_THAT(stats.timer_preemptions_observed, t::Gt(0));
    EXPECT_THAT(stats.kmalloc_ops, t::Ge(1));
    EXPECT_THAT(stats.kmalloc_aligned_ops, t::Ge(1));
    EXPECT_THAT(stats.cpp_new_delete_ops, t::Ge(1));
    EXPECT_THAT(stats.pmm_alloc_ops, t::Ge(1));
    EXPECT_THAT(stats.payload_verifications, t::Ge(4));
    EXPECT_THAT(stats.payload_corruptions, t::Eq(0));
    EXPECT_THAT(stats.stack_canary_corruptions, t::Eq(0));
    EXPECT_THAT(stats.pmm_free_after, t::Eq(pmm_before));
    EXPECT_THAT(stats.heap_free_after, t::Eq(heap_before));
    EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
    EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
  }
}

TEST_F(StressTest, SupportsMoreThan64CpusUpTo128) {
  for (const int num_cpus : {68, kMaxSupportedCpus}) {
    SCOPED_TRACE(num_cpus);
    SetupEnv(num_cpus);
    TaskInit();
    ASSERT_THAT(GetTaskScheduler(), t::NotNull());

    const int64_t pmm_before = PmmFreeFrameCount();
    const int64_t heap_before = HeapTotalFreeBytes();
    SimulatedAps aps(num_cpus);

    StressTestConfig config;
    config.seed = 0x123456789ABCDEF0ULL ^ static_cast<uint64_t>(num_cpus);
    config.num_workers = 8;
    config.rounds_per_worker = 2;
    config.max_live_allocs_per_worker = 2;
    config.max_heap_alloc_bytes = 256;
    config.max_pmm_alloc_frames = 1;
    config.boost_lapic_timer = false;
    config.wait_timeout_tsc_cycles = kHostWaitTimeoutTscCycles;

    const StressTestStats stats = ExecuteRandomMultiCpuStress(config);
    aps.Stop();

    const CpuMask128 all_cpus = CpuMask128::FirstCpus(num_cpus);
    EXPECT_THAT(stats.cpu_count, t::Eq(num_cpus));
    EXPECT_THAT(stats.cpu_participation_mask, t::Eq(all_cpus.low));
    EXPECT_THAT(stats.cpu_participation_mask_high, t::Eq(all_cpus.high));
    EXPECT_THAT(stats.cross_cpu_steals_verified, t::Eq(num_cpus - 1));
    EXPECT_THAT(stats.payload_corruptions, t::Eq(0));
    EXPECT_THAT(stats.stack_canary_corruptions, t::Eq(0));
    EXPECT_THAT(stats.pmm_free_after, t::Eq(pmm_before));
    EXPECT_THAT(stats.heap_free_after, t::Eq(heap_before));
  }
}

TEST_F(StressDeathTest, InvalidConfigsCheckFail) {
  SetupEnv(1);
  TaskInit();

  // Each config field just outside its valid range.
  StressTestConfig config;
  config.num_workers = -1;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(config), "Check failed");
  config = {};
  config.num_workers = kMaxStressWorkers + 1;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(config), "Check failed");
  config = {};
  config.rounds_per_worker = 0;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(config), "Check failed");
  config = {};
  config.rounds_per_worker = kMaxStressRoundsPerWorker + 1;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(config), "Check failed");
  config = {};
  config.max_live_allocs_per_worker = 0;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(config), "Check failed");
  config = {};
  config.max_live_allocs_per_worker = kMaxStressLiveAllocsPerWorker + 1;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(config), "Check failed");
  config = {};
  config.max_heap_alloc_bytes = kMinStressHeapAllocBytes - 1;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(config), "Check failed");
  config = {};
  config.max_heap_alloc_bytes = kMaxStressHeapAllocBytes + 1;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(config), "Check failed");
  config = {};
  config.max_pmm_alloc_frames = 0;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(config), "Check failed");
  config = {};
  config.max_pmm_alloc_frames = kMaxStressPmmAllocFrames + 1;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(config), "Check failed");
  config = {};
  config.wait_timeout_tsc_cycles = 0;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(config), "Check failed");

  SetupEnv(kMaxSupportedCpus + 1);
  TaskInit();
  config = {};
  config.num_workers = 1;
  config.rounds_per_worker = 1;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(config), "Check failed");
}

}  // namespace
}  // namespace protos
