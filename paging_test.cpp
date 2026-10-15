#include "paging.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>

namespace protos {
namespace {

// paging.cpp itself programs the MMU and runs only in the kernel (the boot
// self-tests cover it); these tests cover the parts of paging.h that need no
// hardware.

TEST(PagingTest, UserSpaceIsTheUpperHalfWithoutItsTopPage) {
  EXPECT_EQ(kUserSpaceBase, 0xFFFF800000000000ULL);
  EXPECT_EQ(kUserSpaceEnd, 0ULL - 0x1000);
  EXPECT_EQ(kKernelVirtualEnd, 0x0000800000000000ULL);
  EXPECT_LE(kIdentityMapLimit, kKernelVirtualEnd);
}

TEST(PagingTest, IsUserRangeAcceptsRangesInUserSpace) {
  EXPECT_TRUE(IsUserRange(kUserSpaceBase, 0));
  EXPECT_TRUE(IsUserRange(kUserSpaceBase, 1));
  EXPECT_TRUE(IsUserRange(kUserSpaceEnd, 0));
  EXPECT_TRUE(IsUserRange(kUserSpaceEnd - 1, 1));
  EXPECT_TRUE(IsUserRange(kUserSpaceBase, kUserSpaceEnd - kUserSpaceBase));
  EXPECT_TRUE(IsUserRange(0xFFFFFFFF80000000ULL, 0x1000));
}

TEST(PagingTest, IsUserRangeRejectsRangesOutsideUserSpace) {
  EXPECT_FALSE(IsUserRange(0, 0));
  EXPECT_FALSE(IsUserRange(0, 1));
  EXPECT_FALSE(IsUserRange(kKernelVirtualBase, 0x1000));
  EXPECT_FALSE(IsUserRange(kUserSpaceBase - 1, 1));
  EXPECT_FALSE(IsUserRange(kUserSpaceBase - 0x1000, 0x2000));
  EXPECT_FALSE(IsUserRange(kUserSpaceEnd, 1));
  EXPECT_FALSE(IsUserRange(kUserSpaceEnd - 1, 2));
  EXPECT_FALSE(IsUserRange(kUserSpaceBase, kUserSpaceEnd - kUserSpaceBase + 1));
  EXPECT_FALSE(IsUserRange(kUserSpaceBase, -1));
}

TEST(PagingTest, IsUserRangeDoesNotOverflow) {
  // Ranges whose end address would wrap around past 0.
  EXPECT_FALSE(IsUserRange(kUserSpaceEnd - 8, INT64_MAX));
  EXPECT_FALSE(IsUserRange(0xFFFFFFFFFFFFFFF0ULL, 0x20));
  EXPECT_FALSE(IsUserRange(UINT64_MAX, 1));
  EXPECT_FALSE(IsUserRange(UINT64_MAX, 0));
}

}  // namespace
}  // namespace protos
