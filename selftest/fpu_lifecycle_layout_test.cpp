#include "selftest/fpu_lifecycle_layout.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

#include "smp.h"

namespace protos {
namespace {

namespace t = ::testing;

constexpr int kTestMaxBatches = 5;
constexpr int64_t kTestSyscallTicks = 6;

TEST(FpuLifecycleLayoutTest,
     LifecycleRoundsStaysBetweenTwoAndFourCallsAcrossAllConfigs) {
  for (int cpu_count = 1; cpu_count <= kMaxSupportedCpus; ++cpu_count) {
    const int process_count = 2 * cpu_count;
    for (int batch = 0; batch < kTestMaxBatches; ++batch) {
      for (int i = 0; i < process_count; ++i) {
        const int64_t rounds = FpuLifecycleRounds(i, cpu_count, batch);
        EXPECT_THAT(rounds, t::AllOf(t::Ge(2), t::Le(4)));
      }
    }
  }
}

TEST(FpuLifecycleLayoutTest,
     LeastLoadedPlacementPairsProcessesWithMatchingRoundsOnEachCpu) {
  for (int cpu_count = 1; cpu_count <= kMaxSupportedCpus; ++cpu_count) {
    const int process_count = 2 * cpu_count;
    std::vector<int> load(cpu_count, 0);
    load[0] = 1;  // Bootstrap task on CPU 0.
    std::vector<std::vector<int>> per_cpu(cpu_count);
    for (int i = 0; i < process_count; ++i) {
      int best_cpu = 0;
      for (int c = 1; c < cpu_count; ++c) {
        if (load[c] < load[best_cpu]) {
          best_cpu = c;
        }
      }
      EXPECT_THAT(best_cpu, t::Eq(FpuLifecyclePlacedCpu(i, cpu_count)));
      per_cpu[best_cpu].push_back(i);
      ++load[best_cpu];
    }
    for (int c = 0; c < cpu_count; ++c) {
      ASSERT_THAT(per_cpu[c], t::SizeIs(2));
      for (int batch = 0; batch < kTestMaxBatches; ++batch) {
        EXPECT_THAT(FpuLifecycleRounds(per_cpu[c][0], cpu_count, batch),
                    t::Eq(FpuLifecycleRounds(per_cpu[c][1], cpu_count, batch)));
      }
    }
  }
}

TEST(FpuLifecycleLayoutTest,
     EveryMultiCpuCountAndBatchHasStrictWorkImbalanceForStealing) {
  for (int cpu_count = 2; cpu_count <= kMaxSupportedCpus; ++cpu_count) {
    for (int batch = 0; batch < kTestMaxBatches; ++batch) {
      int64_t min_finish_rounds = 100;
      int64_t max_both_alive_rounds = 0;
      for (int k = 0; k < cpu_count; ++k) {
        const int64_t r0 = FpuLifecycleRounds(k, cpu_count, batch);
        const int64_t r1 = FpuLifecycleRounds(k + cpu_count, cpu_count, batch);
        min_finish_rounds = std::min(min_finish_rounds, r0 + r1);
        max_both_alive_rounds =
            std::max(max_both_alive_rounds, 2 * std::min(r0, r1));
      }
      // At least one thief CPU finishes both of its processes at least 2 calls
      // (12 timer ticks) before a donor CPU finishes its two-process phase.
      EXPECT_THAT(min_finish_rounds + 2, t::Le(max_both_alive_rounds))
          << "cpu_count=" << cpu_count << " batch=" << batch;
    }
  }
}

TEST(FpuLifecycleLayoutTest,
     TickModelPredictsMidSyscallMigrationInEveryBatchForAllMultiCpuCounts) {
  for (int batch = 0; batch < kTestMaxBatches; ++batch) {
    EXPECT_THAT(
        SimulateFpuLifecycleBatchMigrations(1, batch, kTestSyscallTicks),
        t::Eq(0));
  }
  for (int cpu_count = 2; cpu_count <= kMaxSupportedCpus; ++cpu_count) {
    for (int batch = 0; batch < kTestMaxBatches; ++batch) {
      EXPECT_THAT(SimulateFpuLifecycleBatchMigrations(cpu_count,  //
                                                      batch,      //
                                                      kTestSyscallTicks),
                  t::Ge(1))
          << "cpu_count=" << cpu_count << " batch=" << batch;
    }
  }
}

}  // namespace
}  // namespace protos
