#include "steal_time.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <thread>

namespace protos {
namespace {

namespace t = ::testing;

struct FakeCpuidRegs {
  uint32_t eax = 0;
  uint32_t ebx = 0;
  uint32_t ecx = 0;
  uint32_t edx = 0;
};

TEST(StealTimeTest, HasKvmStealTimeScansHypervisorLeaves) {
  // 1. Standard KVM signature at 0x40000000 with kKvmFeatureStealTime set.
  const auto standard_kvm = [](const uint32_t leaf, const uint32_t) {
    FakeCpuidRegs r;
    if (leaf == kCpuidLeafProcessorInfo) {
      r.ecx = kCpuidEcxHypervisor;
    } else if (leaf == 0x40000000u) {
      r.eax = 0x40000001u;
      r.ebx = kKvmSignatureEbx;
      r.ecx = kKvmSignatureEcx;
      r.edx = kKvmSignatureEdx;
    } else if (leaf == 0x40000001u) {
      r.eax = kKvmFeatureStealTime;
    }
    return r;
  };
  EXPECT_TRUE(HasKvmStealTime(standard_kvm));

  // 2. Hyper-V signature at 0x40000000 and KVM signature at 0x40000100.
  const auto hyperv_plus_kvm = [](const uint32_t leaf, const uint32_t) {
    FakeCpuidRegs r;
    if (leaf == kCpuidLeafProcessorInfo) {
      r.ecx = kCpuidEcxHypervisor;
    } else if (leaf == 0x40000000u) {
      r.eax = 0x40000005u;
      r.ebx = 0x7263694Du;  // "Micr"
    } else if (leaf == 0x40000100u) {
      r.eax = 0;  // Legacy KVM max_leaf == 0 means 0x40000101 is still valid.
      r.ebx = kKvmSignatureEbx;
      r.ecx = kKvmSignatureEcx;
      r.edx = kKvmSignatureEdx;
    } else if (leaf == 0x40000101u) {
      r.eax = kKvmFeatureStealTime;
    }
    return r;
  };
  EXPECT_TRUE(HasKvmStealTime(hyperv_plus_kvm));

  // 3. KVM signature present, but kKvmFeatureStealTime bit not set, or
  // HYPERVISOR bit missing in leaf 1.
  const auto kvm_without_steal = [](const uint32_t leaf, const uint32_t) {
    FakeCpuidRegs r;
    if (leaf == kCpuidLeafProcessorInfo) {
      r.ecx = kCpuidEcxHypervisor;
    } else if (leaf == 0x40000000u) {
      r.eax = 0x40000001u;
      r.ebx = kKvmSignatureEbx;
      r.ecx = kKvmSignatureEcx;
      r.edx = kKvmSignatureEdx;
    }
    return r;
  };
  EXPECT_FALSE(HasKvmStealTime(kvm_without_steal));

  const auto no_hypervisor_bit = [](const uint32_t, const uint32_t) {
    return FakeCpuidRegs{};
  };
  EXPECT_FALSE(HasKvmStealTime(no_hypervisor_bit));
}

TEST(StealTimeTest, StealNsFromSnapshotAndReadKvmStealNsSeqlock) {
  // Matching even version succeeds.
  EXPECT_THAT(StealNsFromSnapshot(4, 12345678ULL, 4), t::Eq(12345678));

  // Odd version (hypervisor write in progress) fails.
  EXPECT_THAT(StealNsFromSnapshot(3, 12345678ULL, 3), t::Eq(-1));

  // Mismatched even versions fail.
  EXPECT_THAT(StealNsFromSnapshot(2, 12345678ULL, 4), t::Eq(-1));

  // Overflow above INT64_MAX fails.
  EXPECT_THAT(
      StealNsFromSnapshot(2, static_cast<uint64_t>(INT64_MAX) + 1ULL, 2),
      t::Eq(-1));

  // ReadKvmStealNs returns -1 on permanently odd version.
  KvmStealTime slot = {};
  slot.version = 1;
  slot.steal = 5000000;
  EXPECT_THAT(ReadKvmStealNs(slot), t::Eq(-1));

  // Even version reads cleanly.
  slot.version = 2;
  EXPECT_THAT(ReadKvmStealNs(slot), t::Eq(5000000));

  // Concurrent writer performing seqlock updates (odd -> write steal -> even)
  // never causes a torn read, and the final state reads cleanly.
  std::atomic<bool> writer_started{false};
  std::atomic<bool> writer_done{false};
  std::thread writer([&slot, &writer_started, &writer_done]() {
    const std::atomic_ref<uint32_t> version_ref(slot.version);
    const std::atomic_ref<uint64_t> steal_ref(slot.steal);
    writer_started.store(true, std::memory_order_release);
    for (uint32_t i = 2; i <= 500; ++i) {
      version_ref.store(2 * i - 1, std::memory_order_relaxed);
      steal_ref.store(static_cast<uint64_t>(i) * 10000ULL,
                      std::memory_order_release);
      version_ref.store(2 * i, std::memory_order_release);
    }
    writer_done.store(true, std::memory_order_release);
  });
  while (!writer_started.load(std::memory_order_acquire)) {
    asm volatile("pause" : : : "memory");
  }
  while (!writer_done.load(std::memory_order_acquire)) {
    const int64_t sample = ReadKvmStealNs(slot);
    if (sample >= 0) {
      EXPECT_THAT(sample % 10000, t::Eq(0));
    }
  }
  writer.join();
  EXPECT_THAT(ReadKvmStealNs(slot), t::Eq(500LL * 10000LL));
}

TEST(StealTimeTest, TimerTickBoundsAndStealNsToTicksConversion) {
  // Invalid inputs return 0.
  EXPECT_THAT(TickNsUpperBound(0), t::Eq(0));
  EXPECT_THAT(TickNsLowerBound(0), t::Eq(0));
  EXPECT_THAT(TickNsUpperBound(-4000000), t::Eq(0));
  EXPECT_THAT(TickNsLowerBound(-4000000), t::Eq(0));

  // A nominal 250 Hz tick lasts 4,000,000 ns. The upper bound on a tick of at
  // most that is strictly larger (by 1/1024 and 1 ns), and the lower bound on
  // a tick of at least that strictly smaller (by 1/1024).
  const int64_t cal_upper = TickNsUpperBound(4000000);
  const int64_t cal_lower = TickNsLowerBound(4000000);
  EXPECT_THAT(cal_upper, t::Eq(4000000 + 3906 + 1));
  EXPECT_THAT(cal_lower, t::Eq(4000000 - 3906));

  // Longer ticks yield proportionally larger bounds.
  EXPECT_GT(TickNsUpperBound(6000000), 6000000);
  EXPECT_LT(TickNsLowerBound(6000000), 6000000);
  EXPECT_GT(TickNsLowerBound(6000000), cal_upper);
  // The upper bound saturates instead of overflowing.
  EXPECT_THAT(TickNsUpperBound(INT64_MAX), t::Eq(INT64_MAX));
  EXPECT_THAT(TickNsUpperBound(INT64_MAX - 1), t::Eq(INT64_MAX));

  // StealNsToTicks uses floor division so steal ticks are never over-counted.
  EXPECT_THAT(StealNsToTicks(0, cal_upper), t::Eq(0));
  EXPECT_THAT(StealNsToTicks(-100, cal_upper), t::Eq(0));
  EXPECT_THAT(StealNsToTicks(5000000, 0), t::Eq(0));
  EXPECT_THAT(StealNsToTicks(cal_upper - 1, cal_upper), t::Eq(0));
  EXPECT_THAT(StealNsToTicks(cal_upper, cal_upper), t::Eq(1));
  EXPECT_THAT(StealNsToTicks(5 * cal_upper + 1234, cal_upper), t::Eq(5));
}

TEST(StealTimeTest, KillExcludedStealNsEnforcesStage1OnlyInvariants) {
  KillStealRecord base = {};
  base.killer_steal_ns_at_kill = 10000000;
  base.killer_steal_ns_at_observe = 15000000;
  base.killed_steal_ns_at_notice = 20000000;
  base.killed_steal_ns_at_end = 50000000;
  base.notice_cpu = 3;
  base.end_cpu = 3;
  base.notice_switches = 7;
  base.end_switches = 7;

  // The steal exclusion returns max(0, E1 - B0) = (50M - 20M) - (15M - 10M)
  // = 30M - 5M = 25,000,000 ns.
  EXPECT_THAT(KillExcludedStealNs(base), t::Eq(25000000));

  // When B0 >= E1 (e.g., target ran on CPU 0 or CPU 0 was also descheduled as
  // long as target), excluded steal is clamped to 0.
  KillStealRecord r = base;
  r.killer_steal_ns_at_observe = 45000000;
  EXPECT_THAT(KillExcludedStealNs(r), t::Eq(0));

  // Cross-CPU migration between notice and end -> 0 excluded steal.
  r = base;
  r.end_cpu = 4;
  EXPECT_THAT(KillExcludedStealNs(r), t::Eq(0));
  r = base;
  r.notice_cpu = -1;
  r.end_cpu = -1;
  EXPECT_THAT(KillExcludedStealNs(r), t::Eq(0));

  // Context switch occurred on the target CPU -> 0 excluded steal.
  r = base;
  r.end_switches = 8;
  EXPECT_THAT(KillExcludedStealNs(r), t::Eq(0));

  // Invalid or non-monotonic steal snapshots on either killed CPU or CPU 0 -> 0
  // excluded steal.
  r = base;
  r.killed_steal_ns_at_notice = -1;
  EXPECT_THAT(KillExcludedStealNs(r), t::Eq(0));
  r = base;
  r.killed_steal_ns_at_end = 19000000;
  EXPECT_THAT(KillExcludedStealNs(r), t::Eq(0));
  r = base;
  r.killer_steal_ns_at_kill = -1;
  EXPECT_THAT(KillExcludedStealNs(r), t::Eq(0));
  r = base;
  r.killer_steal_ns_at_observe = 9000000;
  EXPECT_THAT(KillExcludedStealNs(r), t::Eq(0));
}

}  // namespace
}  // namespace protos
