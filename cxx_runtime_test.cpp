#include "cxx_runtime.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

namespace protos {
namespace {

namespace t = ::testing;

// Returns `count` bytes with a position-dependent pattern.
static std::vector<uint8_t> Pattern(const int count, const int seed) {
  std::vector<uint8_t> bytes(count);
  for (int i = 0; i < count; ++i) {
    bytes[i] = static_cast<uint8_t>(i * 7 + seed);
  }
  return bytes;
}

// Returns true if the direction flag is clear, as the ABI requires.
static bool DirectionFlagClear() {
  uint64_t rflags = 0;
  asm volatile("pushfq\n\tpop %0" : "=r"(rflags));
  return (rflags & (1u << 10)) == 0;
}

TEST(CxxRuntimeTest, MemcpyCopiesBytesAndReturnsDest) {
  for (const int count : {0, 1, 7, 64, 4096 + 3}) {
    const std::vector<uint8_t> src = Pattern(count, 3);
    std::vector<uint8_t> dest(count + 2, 0xEE);
    EXPECT_THAT(RuntimeMemcpy(dest.data() + 1, src.data(), count),
                t::Eq(dest.data() + 1));
    EXPECT_THAT(dest.front(), t::Eq(0xEE));
    EXPECT_THAT(dest.back(), t::Eq(0xEE));
    EXPECT_TRUE(std::equal(src.begin(), src.end(), dest.begin() + 1));
  }
  EXPECT_TRUE(DirectionFlagClear());
}

TEST(CxxRuntimeTest, MemmoveHandlesForwardBackwardAndExactOverlap) {
  constexpr int kSize = 256;
  const std::vector<uint8_t> original = Pattern(kSize, 11);
  for (const int count : {0, 1, 5, 64, 200}) {
    for (const int shift : {-64, -1, 0, 1, 3, 64}) {
      std::vector<uint8_t> buffer = original;
      std::vector<uint8_t> expected = original;
      const int src_offset = 28;
      const int dest_offset = src_offset + shift;
      if (dest_offset < 0 || dest_offset + count > kSize) {
        continue;
      }
      std::memmove(expected.data() + dest_offset,  //
                   expected.data() + src_offset,   //
                   count);
      void* const result = RuntimeMemmove(buffer.data() + dest_offset,  //
                                          buffer.data() + src_offset,   //
                                          count);
      EXPECT_THAT(result, t::Eq(buffer.data() + dest_offset));
      EXPECT_THAT(buffer, t::ContainerEq(expected))
          << "count=" << count << " shift=" << shift;
      EXPECT_TRUE(DirectionFlagClear());
    }
  }
}

TEST(CxxRuntimeTest, MemsetFillsLowByteOfValue) {
  std::vector<uint8_t> buffer(100, 0x11);
  EXPECT_THAT(RuntimeMemset(buffer.data() + 10, 0x1A5, 80),
              t::Eq(buffer.data() + 10));
  for (int i = 0; i < 100; ++i) {
    const uint8_t expected = (i >= 10 && i < 90) ? 0xA5 : 0x11;
    ASSERT_THAT(buffer[i], t::Eq(expected)) << "i=" << i;
  }
  EXPECT_THAT(RuntimeMemset(buffer.data(), 0, 0), t::Eq(buffer.data()));
  EXPECT_THAT(buffer[0], t::Eq(0x11));
}

TEST(CxxRuntimeTest, MemcmpComparesAsUnsignedBytes) {
  const uint8_t a[] = {1, 2, 3, 0x80};
  const uint8_t b[] = {1, 2, 3, 0x7F};
  const uint8_t c[] = {1, 2, 4, 0x00};
  EXPECT_THAT(RuntimeMemcmp(a, a, sizeof(a)), t::Eq(0));
  EXPECT_THAT(RuntimeMemcmp(a, b, 3), t::Eq(0));
  EXPECT_THAT(RuntimeMemcmp(a, b, sizeof(a)), t::Gt(0));
  EXPECT_THAT(RuntimeMemcmp(b, a, sizeof(a)), t::Lt(0));
  EXPECT_THAT(RuntimeMemcmp(a, c, sizeof(a)), t::Lt(0));
  EXPECT_THAT(RuntimeMemcmp(c, a, sizeof(a)), t::Gt(0));
  EXPECT_THAT(RuntimeMemcmp(a, c, 0), t::Eq(0));
}

TEST(CxxRuntimeTest, GuardAcquireReleaseAndAbort) {
  int64_t guard = 0;
  ASSERT_THAT(RuntimeGuardAcquire(&guard), t::Eq(1));
  // An abandoned initialization lets the next caller retry.
  RuntimeGuardAbort(&guard);
  ASSERT_THAT(RuntimeGuardAcquire(&guard), t::Eq(1));
  RuntimeGuardRelease(&guard);
  // Compiler-generated code tests the first byte before calling acquire.
  EXPECT_THAT(*reinterpret_cast<const uint8_t*>(&guard), t::Ne(0));
  EXPECT_THAT(RuntimeGuardAcquire(&guard), t::Eq(0));
}

TEST(CxxRuntimeTest, GuardRunsInitializerExactlyOnceAcrossThreads) {
  constexpr int kThreads = 8;
  constexpr int kRounds = 50;
  for (int round = 0; round < kRounds; ++round) {
    int64_t guard = 0;
    int initialized_value = 0;
    std::atomic<int> initializer_runs{0};
    std::atomic<bool> start{false};
    std::vector<int> observed(kThreads, -1);
    std::vector<std::thread> threads;
    threads.reserve(kThreads);
    for (int i = 0; i < kThreads; ++i) {
      threads.emplace_back([&, i]() {
        while (!start.load(std::memory_order_acquire)) {
        }
        if (RuntimeGuardAcquire(&guard) != 0) {
          initializer_runs.fetch_add(1, std::memory_order_relaxed);
          initialized_value = 1000 + round;
          RuntimeGuardRelease(&guard);
        }
        observed[i] = initialized_value;
      });
    }
    start.store(true, std::memory_order_release);
    for (std::thread& thread : threads) {
      thread.join();
    }
    ASSERT_THAT(initializer_runs.load(), t::Eq(1));
    EXPECT_THAT(observed, t::Each(t::Eq(1000 + round)));
  }
}

}  // namespace
}  // namespace protos
