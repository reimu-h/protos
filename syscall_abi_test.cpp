#include "syscall_abi.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>

namespace protos {
namespace {

namespace t = ::testing;

// User programs verify spin()'s result with the same function at compile time
// or at run time.
constexpr uint8_t kAbc[] = {'a', 'b', 'c'};
static_assert(SyscallSpinChecksum(kAbc, 0) == kSyscallSpinChecksumInit);
static_assert(SyscallSpinChecksum(kAbc + 1, 2, SyscallSpinChecksum(kAbc, 1)) ==
              SyscallSpinChecksum(kAbc, 3));

// Returns the bytes of `text`.
static const uint8_t* Bytes(const char* const text) {
  return reinterpret_cast<const uint8_t*>(text);
}

// The 64-bit FNV-1a hash of the `size` bytes at `data`, computed in one go,
// as a reference.
static uint64_t Fnv1a64(const uint8_t* const data, const int64_t size) {
  uint64_t hash = 0xCBF29CE484222325ULL;
  for (int64_t i = 0; i < size; ++i) {
    hash ^= data[i];
    hash *= 0x100000001B3ULL;
  }
  return hash;
}

constexpr uint64_t kBit63 = uint64_t{1} << 63;

TEST(SyscallAbiTest, SpinChecksumIsTheFnv1aHashWithBit63Cleared) {
  EXPECT_THAT(SyscallSpinChecksum(nullptr, 0), t::Eq(0x4BF29CE484222325));
  EXPECT_THAT(SyscallSpinChecksum(Bytes("a"), 1), t::Eq(0x2F63DC4C8601EC8C));
  EXPECT_THAT(SyscallSpinChecksum(Bytes("foobar"), 6),
              t::Eq(0x05944171F73967E8));
  uint8_t pattern[300];
  for (int i = 0; i < 300; ++i) {
    pattern[i] = static_cast<uint8_t>(i * 37 + 11);
  }
  EXPECT_THAT(static_cast<uint64_t>(SyscallSpinChecksum(pattern, 300)),
              t::Eq(Fnv1a64(pattern, 300) & ~kBit63));
}

TEST(SyscallAbiTest, SpinChecksumCanBeComputedPieceByPiece) {
  uint8_t pattern[64];
  for (int i = 0; i < 64; ++i) {
    pattern[i] = static_cast<uint8_t>(0xF0 ^ (i * 13));
  }
  const int64_t whole = SyscallSpinChecksum(pattern, 64);
  for (int split = 0; split <= 64; ++split) {
    const int64_t head = SyscallSpinChecksum(pattern, split);
    EXPECT_THAT(SyscallSpinChecksum(pattern + split, 64 - split, head),
                t::Eq(whole))
        << "split at " << split;
  }
}

TEST(SyscallAbiTest, SpinChecksumIsNeverNegativeSoNeverAnError) {
  static uint8_t buffer[kSyscallSpinMaxSize];
  for (const uint8_t fill : {uint8_t{0x00}, uint8_t{0xFF}}) {
    for (int64_t i = 0; i < kSyscallSpinMaxSize; ++i) {
      buffer[i] = fill;
    }
    EXPECT_THAT(SyscallSpinChecksum(buffer, kSyscallSpinMaxSize), t::Ge(0));
  }
  // Pseudo-random contents and sizes; about half of their FNV-1a hashes have
  // bit 63 set.
  uint64_t state = 0x9E3779B97F4A7C15ULL;
  int high_hashes = 0;
  for (int round = 0; round < 256; ++round) {
    for (int64_t i = 0; i < kSyscallSpinMaxSize; ++i) {
      state = state * 6364136223846793005ULL + 1442695040888963407ULL;
      buffer[i] = static_cast<uint8_t>(state >> 56);
    }
    const int64_t size =
        static_cast<int64_t>(state >> 40) % kSyscallSpinMaxSize;
    high_hashes += ((Fnv1a64(buffer, size) & kBit63) != 0) ? 1 : 0;
    EXPECT_THAT(SyscallSpinChecksum(buffer, size), t::Ge(0));
  }
  EXPECT_THAT(high_hashes, t::Gt(0));
}

TEST(SyscallAbiTest, NumbersAndLimitsStayTheSame) {
  EXPECT_THAT(kSyscallExit, t::Eq(0));
  EXPECT_THAT(kSyscallWrite, t::Eq(1));
  EXPECT_THAT(kSyscallGetCpu, t::Eq(2));
  EXPECT_THAT(kSyscallSpin, t::Eq(3));
  EXPECT_THAT(kSyscallWriteMaxSize, t::Eq(4096));
  EXPECT_THAT(kSyscallSpinMaxSize, t::Eq(4096));
  EXPECT_THAT(kSyscallSpinMaxTicks, t::Eq(1000000));
  EXPECT_THAT(kSyscallErrorFault, t::Eq(-14));
  EXPECT_THAT(kSyscallErrorInvalid, t::Eq(-22));
  EXPECT_THAT(kSyscallErrorNoSys, t::Eq(-38));
}

}  // namespace
}  // namespace protos
