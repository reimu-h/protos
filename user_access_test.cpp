#include "user_access.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "cpu.h"
#include "paging.h"
#include "pmm.h"

namespace protos {
namespace {

namespace t = ::testing;

// Where the tests install their stand-in for user memory, and its size.
constexpr uintptr_t kStandInBase = kUserSpaceBase + 0x10000;
constexpr int64_t kStandInSize = 2 * kPageSize;

// A user range and why the helpers must reject it.
struct BadRange {
  uintptr_t address;
  int64_t size;
  const char* what;
};

// Ranges that don't lie in user space (paging.h).
const BadRange kBadRanges[] = {
    {0, 8, "null"},
    {0x100000, 8, "kernel image"},
    {kKernelVirtualBase, 8, "kernel task stacks"},
    {0x0000800000000000ULL, 8, "non-canonical"},
    {kUserSpaceBase - 8, 16, "straddles into user space from below"},
    {kUserSpaceEnd, 1, "top page"},
    {kUserSpaceEnd - 4, 8, "straddles into the top page"},
    {0xFFFFFFFFFFFFFFF8ULL, 16, "wraps around"},
    {kStandInBase, -1, "negative size"},
    {kStandInBase, INT64_MAX, "larger than user space"},
};

class UserAccessTest : public t::Test {
 protected:
  void SetUp() override {
    CpuResetForTest();
    memory_.resize(kStandInSize);
    for (int64_t i = 0; i < kStandInSize; ++i) {
      memory_[i] = Pattern(i);
    }
    UserAccessSetHostMemoryForTest(kStandInBase, memory_.data(), kStandInSize);
  }

  void TearDown() override {
    UserAccessSetHostMemoryForTest(0, nullptr, 0);
    SetUserAccessWindowOpenForTest(false);
    CpuResetForTest();
  }

  // The byte at `offset` in the stand-in before a test changes it.
  static uint8_t Pattern(const int64_t offset) {
    return static_cast<uint8_t>(offset * 7 + 3);
  }

  std::vector<uint8_t> memory_;
};

TEST_F(UserAccessTest, CopyFromUserCopiesUserBytes) {
  // Unaligned, and across the page boundary.
  constexpr int64_t kOffset = kPageSize - 6;
  uint8_t bytes[12] = {};
  EXPECT_TRUE(CopyFromUser(bytes, kStandInBase + kOffset, sizeof(bytes)));
  for (int64_t i = 0; i < static_cast<int64_t>(sizeof(bytes)); ++i) {
    EXPECT_THAT(bytes[i], t::Eq(Pattern(kOffset + i))) << "byte " << i;
  }
  EXPECT_THAT(UserAccessGetHostCopyCountForTest(), t::Eq(1));
}

TEST_F(UserAccessTest, CopyFromUserRejectsRangesOutsideUserSpace) {
  for (const BadRange& range : kBadRanges) {
    uint8_t bytes[16] = {};
    bytes[0] = 0xEE;
    EXPECT_FALSE(CopyFromUser(bytes, range.address, range.size)) << range.what;
    EXPECT_THAT(bytes[0], t::Eq(0xEE)) << range.what;
  }
  EXPECT_THAT(UserAccessGetHostCopyCountForTest(), t::Eq(0));
}

TEST_F(UserAccessTest, CopyFromUserAcceptsTheEdgesOfUserSpace) {
  // The last byte below the top page.
  UserAccessSetHostMemoryForTest(kUserSpaceEnd - kPageSize,  //
                                 memory_.data(),             //
                                 kPageSize);
  uint8_t byte = 0;
  EXPECT_TRUE(CopyFromUser(&byte, kUserSpaceEnd - 1, 1));
  EXPECT_THAT(byte, t::Eq(Pattern(kPageSize - 1)));
  // The first byte of user space.
  UserAccessSetHostMemoryForTest(kUserSpaceBase, memory_.data(), kPageSize);
  EXPECT_TRUE(CopyFromUser(&byte, kUserSpaceBase, 1));
  EXPECT_THAT(byte, t::Eq(Pattern(0)));
  EXPECT_THAT(UserAccessGetHostCopyCountForTest(), t::Eq(1));
  // Nothing to copy, but the address must still be a user address.
  EXPECT_TRUE(CopyFromUser(nullptr, kUserSpaceBase, 0));
  EXPECT_TRUE(CopyFromUser(nullptr, kUserSpaceEnd, 0));
  EXPECT_FALSE(CopyFromUser(nullptr, 0, 0));
  EXPECT_THAT(UserAccessGetHostCopyCountForTest(), t::Eq(1));
}

TEST_F(UserAccessTest, CopyToUserWritesUserBytes) {
  constexpr int64_t kOffset = kPageSize - 3;
  const uint8_t bytes[] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5};
  EXPECT_TRUE(CopyToUser(kStandInBase + kOffset, bytes, sizeof(bytes)));
  for (int64_t i = 0; i < kStandInSize; ++i) {
    const bool written =
        i >= kOffset && i < kOffset + static_cast<int64_t>(sizeof(bytes));
    EXPECT_THAT(memory_[i], t::Eq(written ? bytes[i - kOffset] : Pattern(i)))
        << "byte " << i;
  }
  EXPECT_THAT(UserAccessGetHostCopyCountForTest(), t::Eq(1));
}

TEST_F(UserAccessTest, CopyToUserRejectsRangesOutsideUserSpace) {
  const uint8_t bytes[16] = {0xEE};
  for (const BadRange& range : kBadRanges) {
    EXPECT_FALSE(CopyToUser(range.address, bytes, range.size)) << range.what;
  }
  EXPECT_FALSE(CopyToUser(0, nullptr, 0));
  EXPECT_TRUE(CopyToUser(kStandInBase, nullptr, 0));
  EXPECT_THAT(UserAccessGetHostCopyCountForTest(), t::Eq(0));
  for (int64_t i = 0; i < kStandInSize; ++i) {
    EXPECT_THAT(memory_[i], t::Eq(Pattern(i))) << "byte " << i;
  }
}

TEST_F(UserAccessTest, CopiesLiftSmapOnlyWhileItIsEnabled) {
  uint8_t byte = 0;
  EXPECT_TRUE(CopyFromUser(&byte, kStandInBase, 1));
  EXPECT_FALSE(UserAccessGetHostLastCopyLiftedSmapForTest());
  CpuSetSmapEnabledForTest(true);
  EXPECT_TRUE(CopyFromUser(&byte, kStandInBase, 1));
  EXPECT_TRUE(UserAccessGetHostLastCopyLiftedSmapForTest());
  CpuSetSmapEnabledForTest(false);
  EXPECT_TRUE(CopyToUser(kStandInBase, &byte, 1));
  EXPECT_FALSE(UserAccessGetHostLastCopyLiftedSmapForTest());
  // Neither copy changes the window that the DCHECKs check.
  EXPECT_FALSE(UserAccessWindowOpen());
}

// Pins the host seam that the AC death tests of idt_test.cpp and
// core_runqueue_scheduler_test.cpp use to open the window.
TEST_F(UserAccessTest, WindowFollowsTheSimulatedStateOfTheThread) {
  EXPECT_FALSE(UserAccessWindowOpen());
  SetUserAccessWindowOpenForTest(true);
  EXPECT_TRUE(UserAccessWindowOpen());
  SetUserAccessWindowOpenForTest(false);
  EXPECT_FALSE(UserAccessWindowOpen());
}

using UserAccessDeathTest = UserAccessTest;

TEST_F(UserAccessDeathTest, HostCopyOutsideTheInstalledMemoryFails) {
  // A user range, but not the stand-in: host builds have no user space.
  uint8_t byte = 0;
  EXPECT_DEATH(CopyFromUser(&byte, kStandInBase + kStandInSize, 1),
               "Check failed: in_host_memory");
}

}  // namespace
}  // namespace protos
