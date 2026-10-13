#include "selftest/selftest.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "selftest/boot_selftest.h"
#include "selftest/stress.h"

namespace protos {
namespace {

// Thrown by the fake stress test to end the endless loop of RunSelfTests().
struct StopSelfTests {};

// Stress-test runs after which the fake stress test throws `StopSelfTests`.
constexpr int kStressRunsBeforeStop = 3;

bool g_boot_selftests_pass = false;
int g_boot_selftest_runs = 0;
// The `seed_override` of each stress-test run, in order.
std::vector<uint64_t> g_stress_seed_overrides;

}  // namespace

// Fakes of the two suites that RunSelfTests() runs.
bool RunBootSelfTests() {
  ++g_boot_selftest_runs;
  return g_boot_selftests_pass;
}

void RunRandomMultiCpuStressTest(const uint64_t seed_override) {
  g_stress_seed_overrides.push_back(seed_override);
  if (static_cast<int>(g_stress_seed_overrides.size()) ==
      kStressRunsBeforeStop) {
    throw StopSelfTests();
  }
}

namespace {

namespace t = ::testing;

class SelfTestTest : public t::Test {
 protected:
  void SetUp() override {
    g_boot_selftests_pass = false;
    g_boot_selftest_runs = 0;
    g_stress_seed_overrides.clear();
  }
};

TEST_F(SelfTestTest, ReturnsWithoutStressTestingIfTheBootSelfTestsFail) {
  g_boot_selftests_pass = false;
  RunSelfTests();
  EXPECT_THAT(g_boot_selftest_runs, t::Eq(1));
  EXPECT_THAT(g_stress_seed_overrides, t::IsEmpty());
}

TEST_F(SelfTestTest, StressTestsWithFreshSeedsForeverIfTheBootSelfTestsPass) {
  g_boot_selftests_pass = true;
  EXPECT_THROW(RunSelfTests(), StopSelfTests);
  EXPECT_THAT(g_boot_selftest_runs, t::Eq(1));
  // A zero `seed_override` makes every run harvest a new seed.
  EXPECT_THAT(g_stress_seed_overrides, t::ElementsAre(0u, 0u, 0u));
}

}  // namespace
}  // namespace protos
