#include "random.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>

#include "smp.h"

namespace protos {
namespace {

namespace t = ::testing;

// Restores the hardware entropy sources and unbinds the test's `CpuLocal`
// after each test.
class RandomTest : public t::Test {
 protected:
  void TearDown() override {
    SetRandomHooksForTest({});
    ResetCpuLocalForTest();
  }
};

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
  constexpr int kWideRangeMin = 1;
  constexpr int kWideRangeMax = 1024 * 1024;
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

    const int r_int = rng.NextIntRange(kWideRangeMin, kWideRangeMax);
    EXPECT_THAT(r_int, t::AllOf(t::Ge(kWideRangeMin), t::Le(kWideRangeMax)));

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
  constexpr int64_t kMaxAlign = 4096;
  Prng64 rng(0x9E3779B97F4A7C15ULL);

  for (int i = 0; i < 32; ++i) {
    EXPECT_THAT(rng.NextPowerOfTwoAlignment(64, 64), t::Eq(64));
  }

  int exp_hits[13] = {};
  for (int i = 0; i < 2600; ++i) {
    const int64_t align = rng.NextPowerOfTwoAlignment(1, kMaxAlign);
    EXPECT_THAT(align, t::AllOf(t::Ge(1), t::Le(kMaxAlign)));
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

TEST(ReadTsc64Test, DoesNotGoBackwards) {
  const uint64_t tsc1 = ReadTsc64();
  const uint64_t tsc2 = ReadTsc64();
  EXPECT_THAT(tsc2, t::Ge(tsc1));
}

TEST(MixEntropy64Test, ChangingAnInputFlipsAboutHalfOfTheOutputBits) {
  const uint64_t m0 = MixEntropy64(0x123456789ABCDEF0ULL, 1ULL);
  const uint64_t m1 = MixEntropy64(0x123456789ABCDEF0ULL, 2ULL);
  EXPECT_THAT(m0, t::Ne(m1));
  EXPECT_THAT(__builtin_popcountll(m0 ^ m1), t::AllOf(t::Ge(16), t::Le(48)));
}

TEST_F(RandomTest, HarvestsDistinctNonZeroSeedsWithAndWithoutABoundCpuLocal) {
  CpuLocal cpu;
  cpu.self = &cpu;
  cpu.cpu_id = 1;
  cpu.apic_id = 2;
  cpu.timer_ticks.store(1000, std::memory_order_relaxed);
  cpu.ipi_count.store(10, std::memory_order_relaxed);
  BindCpuLocal(&cpu);
  ASSERT_THAT(CurrentCpuOrNull(), t::Eq(&cpu));

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

  ResetCpuLocalForTest();
  ASSERT_THAT(CurrentCpuOrNull(), t::IsNull());
  EXPECT_THAT(HarvestHardwareEntropySeed(0x3333ULL), t::Ne(0u));
}

TEST_F(RandomTest, WithoutRdrandReadsFailCleanlyAndSeedsAreStillNonZero) {
  RandomHooks no_rdrand;
  no_rdrand.rdrand_supported = [] { return false; };
  SetRandomHooksForTest(no_rdrand);
  EXPECT_FALSE(CpuSupportsRdrand());
  uint64_t unsupported_out = 0xDEADBEEFULL;
  EXPECT_FALSE(TryReadRdrand64(&unsupported_out));
  EXPECT_THAT(unsupported_out, t::Eq(0u));
  const uint64_t fallback_seed_1 = HarvestHardwareEntropySeed(10);
  const uint64_t fallback_seed_2 = HarvestHardwareEntropySeed(20);
  EXPECT_THAT(fallback_seed_1, t::Ne(0u));
  EXPECT_THAT(fallback_seed_2, t::Ne(0u));
  EXPECT_THAT(fallback_seed_1, t::Ne(fallback_seed_2));
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

TEST_F(RandomTest, RetriesATransientlyEmptyRdrandFifo) {
  RandomHooks flaky_rdrand;
  flaky_rdrand.rdrand_supported = [] { return true; };
  flaky_rdrand.rdrand_attempt = FailFourTimesThenSucceed;
  SetRandomHooksForTest(flaky_rdrand);
  g_rdrand_attempts.store(0, std::memory_order_relaxed);
  EXPECT_TRUE(CpuSupportsRdrand());
  uint64_t retry_out = 0;
  EXPECT_TRUE(TryReadRdrand64(&retry_out));
  EXPECT_THAT(retry_out, t::Eq(0xA5A55A5AF00DCAFEULL));
  EXPECT_THAT(g_rdrand_attempts.load(), t::Eq(5));
}

TEST_F(RandomTest, GivesUpOnRdrandAfterTenAttempts) {
  RandomHooks broken_rdrand;
  broken_rdrand.rdrand_supported = [] { return true; };
  broken_rdrand.rdrand_attempt = AlwaysFail;
  SetRandomHooksForTest(broken_rdrand);
  g_rdrand_attempts.store(0, std::memory_order_relaxed);
  uint64_t exhausted_out = 0x1234ULL;
  EXPECT_FALSE(TryReadRdrand64(&exhausted_out));
  EXPECT_THAT(exhausted_out, t::Eq(0u));
  EXPECT_THAT(g_rdrand_attempts.load(), t::Eq(10));
}

TEST_F(RandomTest, EmptyHooksRestoreTheHardwareRdrand) {
  RandomHooks no_rdrand;
  no_rdrand.rdrand_supported = [] { return false; };
  SetRandomHooksForTest(no_rdrand);
  ASSERT_FALSE(CpuSupportsRdrand());

  SetRandomHooksForTest({});
  const bool host_has_rdrand = __builtin_cpu_supports("rdrnd") != 0;
  EXPECT_THAT(CpuSupportsRdrand(), t::Eq(host_has_rdrand));
  uint64_t hw_val = 0;
  EXPECT_THAT(TryReadRdrand64(&hw_val), t::Eq(host_has_rdrand));
}

TEST(RandomDeathTest, InvalidArgumentsCheckFail) {
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
}

}  // namespace
}  // namespace protos
