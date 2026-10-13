#include "stress_test.h"

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
// PollIdleCpu(). Create it after TaskInit().
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

    TaskScheduler* const scheduler = GetTaskScheduler();
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
      if (scheduler == nullptr || !scheduler->PollIdleCpu(cpu)) {
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

static_assert(Prng64(0).state() != 0);
static_assert(Prng64(1).state() != 0);
static_assert(Prng64(1).state() != Prng64(2).state());

TEST(Prng64Test, DeterminismDistinctSeedsAndSeedZeroNonZeroState) {
  Prng64 rng_zero_a(0);
  Prng64 rng_zero_b(0);
  EXPECT_THAT(rng_zero_a.state(), t::Ne(0u));
  EXPECT_THAT(rng_zero_a.state(), t::Eq(rng_zero_b.state()));

  Prng64 rng_one(1);
  Prng64 rng_two(2);
  EXPECT_THAT(rng_one.state(), t::Ne(rng_two.state()));

  for (int i = 0; i < 256; ++i) {
    const uint64_t val_a = rng_zero_a.NextU64();
    const uint64_t val_b = rng_zero_b.NextU64();
    EXPECT_THAT(val_a, t::Eq(val_b));
    EXPECT_THAT(rng_zero_a.state(), t::Ne(0u));
    EXPECT_THAT(rng_one.NextU64(), t::Ne(rng_two.NextU64()));
  }
}

TEST(Prng64Test, ForkProducesIndependentDeterministicStreams) {
  Prng64 base_a(0xCAFEBABE12345678ULL);
  Prng64 base_b(0xCAFEBABE12345678ULL);

  Prng64 stream_a1 = base_a.Fork(1);
  Prng64 stream_a2 = base_a.Fork(2);
  Prng64 stream_b1 = base_b.Fork(1);
  Prng64 stream_b2 = base_b.Fork(2);

  for (int i = 0; i < 128; ++i) {
    const uint64_t v_a1 = stream_a1.NextU64();
    const uint64_t v_a2 = stream_a2.NextU64();
    EXPECT_THAT(v_a1, t::Eq(stream_b1.NextU64()));
    EXPECT_THAT(v_a2, t::Eq(stream_b2.NextU64()));
    EXPECT_THAT(v_a1, t::Ne(v_a2));
  }

  // Forking with different stream IDs from identical starting states must also
  // produce distinct child streams.
  Prng64 copy_1(42);
  Prng64 copy_2(42);
  Prng64 forked_1 = copy_1.Fork(100);
  Prng64 forked_2 = copy_2.Fork(101);
  EXPECT_THAT(forked_1.NextU64(), t::Ne(forked_2.NextU64()));
}

TEST(Prng64Test, BoundedAndRangeHelpersRespectBoundsAndDistribution) {
  Prng64 rng(0x13579BDF2468ACE0ULL);

  for (int i = 0; i < 64; ++i) {
    EXPECT_THAT(rng.NextBounded(1), t::Eq(0u));
    EXPECT_THAT(rng.NextRange(42, 42), t::Eq(42));
    EXPECT_THAT(rng.NextIntRange(-17, -17), t::Eq(-17));
    EXPECT_FALSE(rng.NextChance(0, 10));
    EXPECT_TRUE(rng.NextChance(10, 10));
  }

  constexpr int kBuckets = 8;
  constexpr int kSamples = 4000;
  int bucket_counts[kBuckets] = {};
  int true_bools = 0;
  int chance_hits = 0;

  for (int i = 0; i < kSamples; ++i) {
    const uint64_t b = rng.NextBounded(kBuckets);
    ASSERT_THAT(b, t::Lt(static_cast<uint64_t>(kBuckets)));
    ++bucket_counts[static_cast<int>(b)];

    const int64_t r64 = rng.NextRange(-500, 500);
    EXPECT_THAT(r64, t::AllOf(t::Ge(-500), t::Le(500)));

    const int r_int = rng.NextIntRange(kMinTaskWeight, kMaxTaskWeight);
    EXPECT_THAT(r_int, t::AllOf(t::Ge(static_cast<int>(kMinTaskWeight)),
                                t::Le(static_cast<int>(kMaxTaskWeight))));

    if (rng.NextBool()) {
      ++true_bools;
    }
    if (rng.NextChance(3, 10)) {
      ++chance_hits;
    }
  }

  for (int b = 0; b < kBuckets; ++b) {
    EXPECT_THAT(bucket_counts[b], t::AllOf(t::Gt(300), t::Lt(700)));
  }
  EXPECT_THAT(true_bools, t::AllOf(t::Gt(1700), t::Lt(2300)));
  EXPECT_THAT(chance_hits, t::AllOf(t::Gt(900), t::Lt(1500)));

  // Full 64-bit signed range [INT64_MIN, INT64_MAX] must not overflow.
  const int64_t full_range_val = rng.NextRange(INT64_MIN, INT64_MAX);
  (void)full_range_val;
}

TEST(Prng64Test, NextPowerOfTwoAlignmentCoversAllValidPowersOfTwo) {
  Prng64 rng(0x9E3779B97F4A7C15ULL);

  for (int i = 0; i < 32; ++i) {
    EXPECT_THAT(rng.NextPowerOfTwoAlignment(64, 64), t::Eq(64));
  }

  int exp_hits[13] = {};
  for (int i = 0; i < 2600; ++i) {
    const int64_t align = rng.NextPowerOfTwoAlignment(1, kPageSize);
    EXPECT_THAT(align, t::AllOf(t::Ge(1), t::Le(kPageSize)));
    EXPECT_THAT(align & (align - 1), t::Eq(0));

    int exp = 0;
    while ((1LL << exp) < align) {
      ++exp;
    }
    ASSERT_THAT(exp, t::AllOf(t::Ge(0), t::Le(12)));
    ++exp_hits[exp];
  }

  for (int exp = 0; exp <= 12; ++exp) {
    EXPECT_THAT(exp_hits[exp], t::Gt(80));
  }
}

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

std::atomic<int> g_rdrand_attempts{0};

static bool FailFourTimesThenSucceed(uint64_t* const out) {
  if (g_rdrand_attempts.fetch_add(1, std::memory_order_relaxed) < 4) {
    return false;
  }
  *out = 0xA5A55A5AF00DCAFEULL;
  return true;
}

static bool AlwaysFail(uint64_t* const /*out*/) {
  g_rdrand_attempts.fetch_add(1, std::memory_order_relaxed);
  return false;
}

TEST_F(StressTest, HardwareEntropyHelpersAndRdrandRetryFallback) {
  SetupEnv(2);

  const uint64_t tsc1 = ReadTsc64();
  const uint64_t tsc2 = ReadTsc64();
  EXPECT_THAT(tsc2, t::Ge(tsc1));

  // Changing one input bit flips about half of the output bits.
  const uint64_t m0 = MixEntropy64(0x123456789ABCDEF0ULL, 1ULL);
  const uint64_t m1 = MixEntropy64(0x123456789ABCDEF0ULL, 2ULL);
  EXPECT_THAT(m0, t::Ne(m1));
  EXPECT_THAT(__builtin_popcountll(m0 ^ m1), t::AllOf(t::Ge(16), t::Le(48)));

  // The real RDRAND (if the host CPU has it) and RDTSC.
  uint64_t hw_val = 0;
  if (CpuSupportsRdrand()) {
    EXPECT_TRUE(TryReadRdrand64(&hw_val));
  }
  const uint64_t seed_a = HarvestHardwareEntropySeed(0x1111ULL);
  const uint64_t seed_b = HarvestHardwareEntropySeed(0x2222ULL);
  EXPECT_THAT(seed_a, t::Ne(0u));
  EXPECT_THAT(seed_b, t::Ne(0u));
  EXPECT_THAT(seed_a, t::Ne(seed_b));

  // 1. Without RDRAND, TryReadRdrand64 fails cleanly and seeds are still
  //    non-zero.
  StressTestHooks no_rdrand;
  no_rdrand.rdrand_supported = [] { return false; };
  SetStressTestHooksForTest(no_rdrand);
  EXPECT_FALSE(CpuSupportsRdrand());
  uint64_t unsupported_out = 0xDEADBEEFULL;
  EXPECT_FALSE(TryReadRdrand64(&unsupported_out));
  EXPECT_THAT(unsupported_out, t::Eq(0u));
  const uint64_t fallback_seed_1 = HarvestHardwareEntropySeed(10);
  const uint64_t fallback_seed_2 = HarvestHardwareEntropySeed(20);
  EXPECT_THAT(fallback_seed_1, t::Ne(0u));
  EXPECT_THAT(fallback_seed_2, t::Ne(0u));
  EXPECT_THAT(fallback_seed_1, t::Ne(fallback_seed_2));

  // 2. A transiently empty RDRAND FIFO is retried; attempt 5 succeeds.
  StressTestHooks flaky_rdrand;
  flaky_rdrand.rdrand_supported = [] { return true; };
  flaky_rdrand.rdrand_attempt = FailFourTimesThenSucceed;
  SetStressTestHooksForTest(flaky_rdrand);
  g_rdrand_attempts.store(0, std::memory_order_relaxed);
  EXPECT_TRUE(CpuSupportsRdrand());
  uint64_t retry_out = 0;
  EXPECT_TRUE(TryReadRdrand64(&retry_out));
  EXPECT_THAT(retry_out, t::Eq(0xA5A55A5AF00DCAFEULL));
  EXPECT_THAT(g_rdrand_attempts.load(), t::Eq(5));

  // 3. RDRAND that never delivers gives up after 10 attempts.
  StressTestHooks broken_rdrand;
  broken_rdrand.rdrand_supported = [] { return true; };
  broken_rdrand.rdrand_attempt = AlwaysFail;
  SetStressTestHooksForTest(broken_rdrand);
  g_rdrand_attempts.store(0, std::memory_order_relaxed);
  uint64_t exhausted_out = 0x1234ULL;
  EXPECT_FALSE(TryReadRdrand64(&exhausted_out));
  EXPECT_THAT(exhausted_out, t::Eq(0u));
  EXPECT_THAT(g_rdrand_attempts.load(), t::Eq(10));

  // 4. HarvestHardwareEntropySeed also works without a bound CpuLocal.
  ResetCpuLocalForTest();
  EXPECT_THAT(CurrentCpuOrNull(), t::IsNull());
  EXPECT_THAT(HarvestHardwareEntropySeed(0x3333ULL), t::Ne(0u));
  BindCpuLocal(&g_env.cpu_locals[0]);
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

  // The entry point that kernel_main runs: the default config, with console
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

TEST_F(StressDeathTest, InvalidArgumentsAndConfigsCheckFail) {
  SetupEnv(1);
  TaskInit();

  EXPECT_DEATH(TryReadRdrand64(nullptr), "Check failed");
  EXPECT_DEATH(Prng64(1).NextBounded(0), "Check failed");
  EXPECT_DEATH(Prng64(1).NextRange(10, 5), "Check failed");
  EXPECT_DEATH(Prng64(1).NextIntRange(10, -1), "Check failed");
  EXPECT_DEATH(Prng64(1).NextChance(-1, 5), "Check failed");
  EXPECT_DEATH(Prng64(1).NextChance(6, 5), "Check failed");
  EXPECT_DEATH(Prng64(1).NextChance(1, 0), "Check failed");
  EXPECT_DEATH(Prng64(1).NextPowerOfTwoAlignment(0, 16), "Check failed");
  EXPECT_DEATH(Prng64(1).NextPowerOfTwoAlignment(3, 16), "Check failed");
  EXPECT_DEATH(Prng64(1).NextPowerOfTwoAlignment(16, 15), "Check failed");
  EXPECT_DEATH(Prng64(1).NextPowerOfTwoAlignment(32, 16), "Check failed");

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
