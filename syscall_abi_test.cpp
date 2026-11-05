#include "syscall_abi.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#include <string>
#include <type_traits>
#include <vector>

#include "fs_protocol.h"
#include "user/ramfs/ramfs.h"
#include "user/sh/sh_lib.h"
#include "user_programs.h"

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
  EXPECT_THAT(kSyscallRead, t::Eq(4));
  EXPECT_THAT(kSyscallSpawn, t::Eq(5));
  EXPECT_THAT(kSyscallWait, t::Eq(6));
  EXPECT_THAT(kSyscallSpawnDriver, t::Eq(7));
  EXPECT_THAT(kSyscallPciInfo, t::Eq(8));
  EXPECT_THAT(kSyscallPciClaim, t::Eq(9));
  EXPECT_THAT(kSyscallPciBusMaster, t::Eq(10));
  EXPECT_THAT(kSyscallMmioMap, t::Eq(11));
  EXPECT_THAT(kSyscallDmaAlloc, t::Eq(12));
  EXPECT_THAT(kSyscallIrqAlloc, t::Eq(13));
  EXPECT_THAT(kSyscallIrqWait, t::Eq(14));
  EXPECT_THAT(kSyscallConsolePush, t::Eq(15));
  EXPECT_THAT(kSyscallGetTicks, t::Eq(16));
  EXPECT_THAT(kSyscallIsaClaim, t::Eq(17));
  EXPECT_THAT(kSyscallIsaPortIo, t::Eq(18));
  EXPECT_THAT(kSyscallIsaIrqAlloc, t::Eq(19));
  EXPECT_THAT(kSyscallIpcListen, t::Eq(20));
  EXPECT_THAT(kSyscallIpcRecv, t::Eq(21));
  EXPECT_THAT(kSyscallIpcReply, t::Eq(22));
  EXPECT_THAT(kSyscallIpcCall, t::Eq(23));
  EXPECT_THAT(kSyscallMemMap, t::Eq(24));
  EXPECT_THAT(kSyscallOpen, t::Eq(25));
  EXPECT_THAT(kSyscallClose, t::Eq(26));
  EXPECT_THAT(kSyscallFileRead, t::Eq(27));
  EXPECT_THAT(kSyscallFileWrite, t::Eq(28));
  EXPECT_THAT(kSyscallSeek, t::Eq(29));
  EXPECT_THAT(kSyscallStat, t::Eq(30));
  EXPECT_THAT(kSyscallReadDir, t::Eq(31));
  EXPECT_THAT(kSyscallMkDir, t::Eq(32));
  EXPECT_THAT(kSyscallUnlink, t::Eq(33));
  EXPECT_THAT(kSyscallRename, t::Eq(34));
  EXPECT_THAT(kSyscallCount, t::Eq(35));
  EXPECT_THAT(kIsaDeviceI8042, t::Eq(0));
  EXPECT_THAT(kIsaDeviceCount, t::Eq(1));
  EXPECT_THAT(kSyscallPortIoRead8, t::Eq(0));
  EXPECT_THAT(kSyscallPortIoWrite8, t::Eq(1));
  EXPECT_THAT(kSyscallWriteMaxSize, t::Eq(4096));
  EXPECT_THAT(kSyscallSpinMaxSize, t::Eq(4096));
  EXPECT_THAT(kSyscallSpinMaxTicks, t::Eq(1000000));
  EXPECT_THAT(kSyscallReadMaxSize, t::Eq(4096));
  EXPECT_THAT(kSyscallSpawnMaxNameLength, t::Eq(64));
  EXPECT_THAT(kMaxUserChildProcesses, t::Eq(8));
  EXPECT_THAT(kSyscallConsolePushMaxSize, t::Eq(256));
  EXPECT_THAT(kSyscallMmioMaxSize, t::Eq(16LL << 20));
  EXPECT_THAT(kSyscallDmaMaxSize, t::Eq(1LL << 20));
  EXPECT_THAT(kSyscallDmaMaxTotalSize, t::Eq(16LL << 20));
  EXPECT_THAT(kSyscallMaxClaims, t::Eq(16));
  EXPECT_THAT(kSyscallDmaBelow4GiB, t::Eq(1ULL));
  EXPECT_THAT(kSyscallDmaFlagsMask, t::Eq(kSyscallDmaBelow4GiB));
  EXPECT_THAT(kSyscallIrqModeAuto, t::Eq(0));
  EXPECT_THAT(kSyscallIrqModeMsi, t::Eq(1));
  EXPECT_THAT(kSyscallIrqModeMsix, t::Eq(2));
  EXPECT_THAT(kSyscallIrqMaxCount, t::Eq(32));
  EXPECT_THAT(kSyscallIrqWaitMaxTicks, t::Eq(1000000));
  EXPECT_THAT(kSyscallTicksPerSecond, t::Eq(250));
  EXPECT_THAT(kSyscallErrorPermission, t::Eq(-1));
  EXPECT_THAT(kSyscallErrorNoMemory, t::Eq(-12));
  EXPECT_THAT(kSyscallErrorAccess, t::Eq(-13));
  EXPECT_THAT(kSyscallErrorFault, t::Eq(-14));
  EXPECT_THAT(kSyscallErrorBusy, t::Eq(-16));
  EXPECT_THAT(kSyscallErrorNoDevice, t::Eq(-19));
  EXPECT_THAT(kSyscallErrorInvalid, t::Eq(-22));
  EXPECT_THAT(kSyscallErrorNoSys, t::Eq(-38));
  EXPECT_THAT(kSyscallErrorNoEntry, t::Eq(-2));
  EXPECT_THAT(kSyscallErrorIo, t::Eq(-5));
  EXPECT_THAT(kSyscallErrorBadFd, t::Eq(-9));
  EXPECT_THAT(kSyscallErrorExists, t::Eq(-17));
  EXPECT_THAT(kSyscallErrorNotDir, t::Eq(-20));
  EXPECT_THAT(kSyscallErrorIsDir, t::Eq(-21));
  EXPECT_THAT(kSyscallErrorFileTableFull, t::Eq(-23));
  EXPECT_THAT(kSyscallErrorTooManyFiles, t::Eq(-24));
  EXPECT_THAT(kSyscallErrorFileTooBig, t::Eq(-27));
  EXPECT_THAT(kSyscallErrorNoSpace, t::Eq(-28));
  EXPECT_THAT(kSyscallErrorDeadlock, t::Eq(-35));
  EXPECT_THAT(kSyscallErrorNameTooLong, t::Eq(-36));
  EXPECT_THAT(kSyscallErrorNotEmpty, t::Eq(-39));
  EXPECT_THAT(kSyscallErrorStale, t::Eq(-116));
  EXPECT_THAT(kIpcPortFs, t::Eq(0));
  EXPECT_THAT(kIpcFirstUserPort, t::Eq(1));
  EXPECT_THAT(kIpcPortCount, t::Eq(16));
  EXPECT_THAT(kIpcMaxMessageSize, t::Eq(4352));
  EXPECT_THAT(kSyscallIpcTimeoutMaxTicks, t::Eq(1000000));
  EXPECT_THAT(kSyscallMemMapMaxSize, t::Eq(1LL << 20));
  EXPECT_THAT(kSyscallMemMapMaxTotalSize, t::Eq(64LL << 20));
  EXPECT_THAT(kMaxOpenFiles, t::Eq(32));
  EXPECT_THAT(kSyscallFsMaxIoSize, t::Eq(4096));
  EXPECT_THAT(kSyscallFsMaxDirEntries, t::Eq(32));
  EXPECT_THAT(kFsMaxFileSize, t::Eq(32LL << 20));
  EXPECT_THAT(kFsMaxNameLength, t::Eq(63));
  EXPECT_THAT(kFsMaxPathLength, t::Eq(255));
  EXPECT_THAT(kFsOpenAccessMask, t::Eq(0x3ULL));
  EXPECT_THAT(kFsOpenRead, t::Eq(0x0ULL));
  EXPECT_THAT(kFsOpenWrite, t::Eq(0x1ULL));
  EXPECT_THAT(kFsOpenReadWrite, t::Eq(0x2ULL));
  EXPECT_THAT(kFsOpenCreate, t::Eq(0x4ULL));
  EXPECT_THAT(kFsOpenTruncate, t::Eq(0x8ULL));
  EXPECT_THAT(kFsOpenAppend, t::Eq(0x10ULL));
  EXPECT_THAT(kFsOpenExclusive, t::Eq(0x20ULL));
  EXPECT_THAT(kFsOpenDirectory, t::Eq(0x40ULL));
  EXPECT_THAT(kFsOpenFlagsMask, t::Eq(0x7FULL));
  EXPECT_THAT(kFsSeekSet, t::Eq(0));
  EXPECT_THAT(kFsSeekCur, t::Eq(1));
  EXPECT_THAT(kFsSeekEnd, t::Eq(2));
  EXPECT_THAT(kUserCapNone, t::Eq(0ULL));
  EXPECT_THAT(kUserCapDriver, t::Eq(1ULL << 0));
  EXPECT_THAT(kUserCapSpawnDrivers, t::Eq(1ULL << 1));
  EXPECT_THAT(kInitArgStartDrivers, t::Eq(1ULL << 0));
  EXPECT_THAT(kInitArgStartFs, t::Eq(1ULL << 1));
  EXPECT_THAT(kDriverArgNormal, t::Eq(0ULL));
  EXPECT_THAT(kDriverArgExitAfterReady, t::Eq(1ULL));
  EXPECT_THAT(kDriverArgFaultAfterReady, t::Eq(2ULL));
}

TEST(SyscallAbiTest, DriverAbiLayoutsAndHelpers) {
  EXPECT_TRUE(SyscallIsError(-1));
  EXPECT_TRUE(SyscallIsError(-4095));
  EXPECT_TRUE(SyscallIsError(kSyscallErrorPermission));
  EXPECT_TRUE(SyscallIsError(kSyscallErrorNoSys));
  EXPECT_FALSE(SyscallIsError(0));
  EXPECT_FALSE(SyscallIsError(1));
  EXPECT_FALSE(SyscallIsError(-4096));
  // Higher-half user virtual addresses (MMIO / DMA windows) cast to negative
  // int64_t values outside [-4095, -1] and are not errors.
  EXPECT_FALSE(SyscallIsError(static_cast<int64_t>(0xFFFF900000000000ULL)));
  EXPECT_FALSE(SyscallIsError(static_cast<int64_t>(0xFFFFA00000000000ULL)));
  EXPECT_FALSE(SyscallIsError(static_cast<int64_t>(0xFFFFB00000000000ULL)));

  for (int64_t num = -1; num <= kSyscallCount + 2; ++num) {
    const bool expected = (num >= 8 && num <= 15) || (num >= 17 && num <= 19);
    EXPECT_THAT(SyscallIsDriverOnly(num), t::Eq(expected)) << "syscall " << num;
  }

  for (int64_t num = -1; num <= kSyscallCount + 2; ++num) {
    const bool is_spawn_drv = (num == kSyscallSpawnDriver);
    const bool is_drv_only =
        (num >= kSyscallPciInfo && num <= kSyscallConsolePush) ||
        (num >= kSyscallIsaClaim && num <= kSyscallIsaIrqAlloc);
    EXPECT_THAT(MayMakeSyscall(kUserCapNone, num),
                t::Eq(!is_spawn_drv && !is_drv_only))
        << "syscall " << num;
    EXPECT_THAT(MayMakeSyscall(kUserCapDriver, num), t::Eq(!is_spawn_drv))
        << "syscall " << num;
    EXPECT_THAT(MayMakeSyscall(kUserCapSpawnDrivers, num), t::Eq(!is_drv_only))
        << "syscall " << num;
    EXPECT_TRUE(MayMakeSyscall(kUserCapDriver | kUserCapSpawnDrivers, num))
        << "syscall " << num;
  }

  const PciDeviceInfo info{};
  EXPECT_THAT(sizeof(info), t::Eq(176U));
  EXPECT_THAT(alignof(PciDeviceInfo), t::Eq(8U));
  EXPECT_THAT(sizeof(PciBarInfo), t::Eq(24U));
  EXPECT_THAT(alignof(PciBarInfo), t::Eq(8U));
  EXPECT_THAT(info.parent_index, t::Eq(-1));
  EXPECT_THAT(info.bars[0].type, t::Eq(PciBarInfoType::kNone));
}

TEST(SyscallAbiTest, IpcAndFsEnumsStayTheSame) {
  EXPECT_TRUE(
      (std::is_same_v<std::underlying_type_t<IpcMessageKind>, uint32_t>));
  EXPECT_THAT(static_cast<uint32_t>(IpcMessageKind::kNone), t::Eq(0U));
  EXPECT_THAT(static_cast<uint32_t>(IpcMessageKind::kCall), t::Eq(1U));
  EXPECT_THAT(static_cast<uint32_t>(IpcMessageKind::kClientDisconnect),
              t::Eq(2U));
  EXPECT_TRUE((std::is_same_v<std::underlying_type_t<FsNodeType>, uint8_t>));
  EXPECT_THAT(static_cast<uint8_t>(FsNodeType::kNone), t::Eq(0));
  EXPECT_THAT(static_cast<uint8_t>(FsNodeType::kRegular), t::Eq(1));
  EXPECT_THAT(static_cast<uint8_t>(FsNodeType::kDirectory), t::Eq(2));
  EXPECT_TRUE((std::is_same_v<std::underlying_type_t<FsRpcOp>, uint32_t>));
  EXPECT_THAT(static_cast<uint32_t>(FsRpcOp::kNone), t::Eq(0U));
  EXPECT_THAT(static_cast<uint32_t>(FsRpcOp::kOpen), t::Eq(1U));
  EXPECT_THAT(static_cast<uint32_t>(FsRpcOp::kClose), t::Eq(2U));
  EXPECT_THAT(static_cast<uint32_t>(FsRpcOp::kRead), t::Eq(3U));
  EXPECT_THAT(static_cast<uint32_t>(FsRpcOp::kWrite), t::Eq(4U));
  EXPECT_THAT(static_cast<uint32_t>(FsRpcOp::kSeek), t::Eq(5U));
  EXPECT_THAT(static_cast<uint32_t>(FsRpcOp::kStat), t::Eq(6U));
  EXPECT_THAT(static_cast<uint32_t>(FsRpcOp::kReadDir), t::Eq(7U));
  EXPECT_THAT(static_cast<uint32_t>(FsRpcOp::kMkDir), t::Eq(8U));
  EXPECT_THAT(static_cast<uint32_t>(FsRpcOp::kUnlink), t::Eq(9U));
  EXPECT_THAT(static_cast<uint32_t>(FsRpcOp::kRename), t::Eq(10U));
}

TEST(SyscallAbiTest, IpcReceivedMessageLayoutStaysTheSame) {
  EXPECT_THAT(sizeof(IpcReceivedMessage), t::Eq(32U));
  EXPECT_THAT(offsetof(IpcReceivedMessage, token), t::Eq(0U));
  EXPECT_THAT(offsetof(IpcReceivedMessage, client_id), t::Eq(8U));
  EXPECT_THAT(offsetof(IpcReceivedMessage, size), t::Eq(16U));
  EXPECT_THAT(offsetof(IpcReceivedMessage, kind), t::Eq(24U));
  EXPECT_THAT(offsetof(IpcReceivedMessage, reply_cap), t::Eq(28U));
  EXPECT_TRUE(
      (std::is_same_v<decltype(IpcReceivedMessage::reply_cap), int32_t>));
}

// Returns the bytes of a value-initialized `T` (`T{}`) constructed in memory
// that held other bytes.
template <typename T>
static std::vector<uint8_t> ValueInitializedBytes() {
  alignas(T) uint8_t storage[sizeof(T)];
  volatile uint8_t* const dirty = storage;
  for (int i = 0; i < static_cast<int>(sizeof(T)); ++i) {
    dirty[i] = 0xA5;
  }
  const T* const obj = new (storage) T{};
  const uint8_t* const bytes = reinterpret_cast<const uint8_t*>(obj);
  return std::vector<uint8_t>(bytes, bytes + sizeof(T));
}

// Returns the bytes of a `T` that holds `op` in its first 4 bytes, like every
// filesystem request header, and zeros in all others.
template <typename T>
static std::vector<uint8_t> OpcodeAndZeros(const FsRpcOp op) {
  std::vector<uint8_t> bytes(sizeof(T), 0);
  const uint32_t value = static_cast<uint32_t>(op);
  __builtin_memcpy(bytes.data(), &value, sizeof(value));
  return bytes;
}

// The kernel value-initializes each request header that it sends to the
// filesystem server (`FsCloseRequest req{}` and so on, user_mode.cpp) and then
// sets the opcode and the fields that the request uses, so the defaults are
// what the server gets in every other byte: they must be zero, as the protocol
// requires of reserved fields (fs_protocol.h). Likewise for the
// `IpcReceivedMessage msg{}` that the kernel copies to an IpcRecv() caller.
TEST(SyscallAbiTest, ValueInitializedHeadersHoldNothingButTheirOpcode) {
  EXPECT_THAT(
      ValueInitializedBytes<FsOpenRequest>(),
      t::ElementsAreArray(OpcodeAndZeros<FsOpenRequest>(FsRpcOp::kOpen)));
  EXPECT_THAT(
      ValueInitializedBytes<FsCloseRequest>(),
      t::ElementsAreArray(OpcodeAndZeros<FsCloseRequest>(FsRpcOp::kClose)));
  EXPECT_THAT(
      ValueInitializedBytes<FsReadRequest>(),
      t::ElementsAreArray(OpcodeAndZeros<FsReadRequest>(FsRpcOp::kRead)));
  EXPECT_THAT(
      ValueInitializedBytes<FsWriteRequest>(),
      t::ElementsAreArray(OpcodeAndZeros<FsWriteRequest>(FsRpcOp::kWrite)));
  EXPECT_THAT(
      ValueInitializedBytes<FsSeekRequest>(),
      t::ElementsAreArray(OpcodeAndZeros<FsSeekRequest>(FsRpcOp::kSeek)));
  EXPECT_THAT(
      ValueInitializedBytes<FsPathRequest>(),
      t::ElementsAreArray(OpcodeAndZeros<FsPathRequest>(FsRpcOp::kNone)));
  EXPECT_THAT(
      ValueInitializedBytes<FsReadDirRequest>(),
      t::ElementsAreArray(OpcodeAndZeros<FsReadDirRequest>(FsRpcOp::kReadDir)));
  EXPECT_THAT(
      ValueInitializedBytes<FsRenameRequest>(),
      t::ElementsAreArray(OpcodeAndZeros<FsRenameRequest>(FsRpcOp::kRename)));
  EXPECT_THAT(ValueInitializedBytes<IpcReceivedMessage>(), t::Each(t::Eq(0)));
}

TEST(SyscallAbiTest, FsOpenFlagsValidAcceptsTheMeaningfulCombinations) {
  EXPECT_TRUE(FsOpenFlagsValid(kFsOpenRead));
  EXPECT_TRUE(FsOpenFlagsValid(kFsOpenWrite));
  EXPECT_TRUE(FsOpenFlagsValid(kFsOpenReadWrite));
  EXPECT_TRUE(FsOpenFlagsValid(kFsOpenRead | kFsOpenDirectory));
  EXPECT_TRUE(FsOpenFlagsValid(kFsOpenRead | kFsOpenAppend));
  EXPECT_TRUE(FsOpenFlagsValid(kFsOpenRead | kFsOpenCreate | kFsOpenExclusive));
  EXPECT_TRUE(FsOpenFlagsValid(kFsOpenWrite | kFsOpenCreate | kFsOpenTruncate));
  EXPECT_TRUE(FsOpenFlagsValid(kFsOpenReadWrite | kFsOpenCreate |
                               kFsOpenTruncate | kFsOpenAppend |
                               kFsOpenExclusive | kFsOpenDirectory));

  // Access mode 3 is none of the three.
  EXPECT_FALSE(FsOpenFlagsValid(kFsOpenAccessMask));
  EXPECT_FALSE(FsOpenFlagsValid(kFsOpenAccessMask | kFsOpenCreate));
  // kFsOpenExclusive without kFsOpenCreate.
  EXPECT_FALSE(FsOpenFlagsValid(kFsOpenRead | kFsOpenExclusive));
  EXPECT_FALSE(FsOpenFlagsValid(kFsOpenReadWrite | kFsOpenExclusive));
  // kFsOpenTruncate with read-only access.
  EXPECT_FALSE(FsOpenFlagsValid(kFsOpenRead | kFsOpenTruncate));
  EXPECT_FALSE(FsOpenFlagsValid(kFsOpenRead | kFsOpenCreate | kFsOpenTruncate));
  // Unknown bits, alone and with valid flags.
  for (int bit = 0; bit < 64; ++bit) {
    const uint64_t flag = uint64_t{1} << bit;
    if ((flag & kFsOpenFlagsMask) != 0) {
      continue;
    }
    EXPECT_FALSE(FsOpenFlagsValid(flag)) << "bit " << bit;
    EXPECT_FALSE(FsOpenFlagsValid(kFsOpenWrite | kFsOpenCreate | flag))
        << "bit " << bit;
  }

  // Those three rules are the only ones: they decide every combination of the
  // known bits.
  for (uint64_t flags = 0; flags <= kFsOpenFlagsMask; ++flags) {
    const uint64_t access = flags & kFsOpenAccessMask;
    const bool bad_access = access == kFsOpenAccessMask;
    const bool exclusive_without_create =
        (flags & kFsOpenExclusive) != 0 && (flags & kFsOpenCreate) == 0;
    const bool read_only_truncate =
        (flags & kFsOpenTruncate) != 0 && access == kFsOpenRead;
    EXPECT_THAT(
        FsOpenFlagsValid(flags),
        t::Eq(!bad_access && !exclusive_without_create && !read_only_truncate))
        << "flags " << flags;
  }
}

TEST(SyscallAbiTest, UserProgramResultDefaultsAndOutcomes) {
  EXPECT_THAT(static_cast<uint8_t>(UserProgramOutcome::kExited), t::Eq(0));
  EXPECT_THAT(static_cast<uint8_t>(UserProgramOutcome::kFaulted), t::Eq(1));
  EXPECT_THAT(static_cast<uint8_t>(UserProgramOutcome::kKilled), t::Eq(2));
  EXPECT_THAT(static_cast<uint8_t>(UserProgramOutcome::kLoadFailed), t::Eq(3));

  const UserProgramResult result{};
  EXPECT_THAT(result.outcome, t::Eq(UserProgramOutcome::kLoadFailed));
  EXPECT_THAT(result.reserved0, t::Each(t::Eq(0)));
  EXPECT_THAT(result.exit_status, t::Eq(0));
  EXPECT_THAT(result.fault_vector, t::Eq(-1));
  EXPECT_THAT(result.reserved1, t::Eq(0U));
  EXPECT_THAT(result.fault_error_code, t::Eq(0U));
  EXPECT_THAT(result.fault_rip, t::Eq(0U));
  EXPECT_THAT(result.fault_address, t::Eq(0U));
}

// wait() copies the whole struct to user memory, so each of its bytes must
// belong to a field that the kernel sets: implicit padding would carry stale
// kernel memory to user space.
TEST(SyscallAbiTest, UserProgramResultIsAllFieldsWithZeroReservedBytes) {
  EXPECT_TRUE(std::has_unique_object_representations_v<UserProgramResult>);
  const UserProgramResult result;
  EXPECT_THAT(sizeof(result.outcome) + sizeof(result.reserved0) +
                  sizeof(result.exit_status) + sizeof(result.fault_vector) +
                  sizeof(result.reserved1) + sizeof(result.fault_error_code) +
                  sizeof(result.fault_rip) + sizeof(result.fault_address),
              t::Eq(sizeof(UserProgramResult)));

  // Constructed in dirty memory, as `UserProcess::result_` is in a heap
  // object that the kernel doesn't clear, it holds its default value in every
  // byte, and so do its copies.
  constexpr int kSize = static_cast<int>(sizeof(UserProgramResult));
  alignas(UserProgramResult) uint8_t storage[kSize];
  volatile uint8_t* const dirty = storage;
  for (int i = 0; i < kSize; ++i) {
    dirty[i] = 0xA5;
  }
  const UserProgramResult* const constructed = new (storage) UserProgramResult;
  std::vector<uint8_t> expected(kSize, 0);
  expected[0] = static_cast<uint8_t>(UserProgramOutcome::kLoadFailed);
  // `fault_vector` (-1) takes bytes 16 to 19.
  for (int i = 16; i < 20; ++i) {
    expected[i] = 0xFF;
  }
  EXPECT_THAT(std::vector<uint8_t>(storage, storage + kSize),
              t::ElementsAreArray(expected));
  const UserProgramResult copy = *constructed;
  const uint8_t* const copy_bytes = reinterpret_cast<const uint8_t*>(&copy);
  EXPECT_THAT(std::vector<uint8_t>(copy_bytes, copy_bytes + kSize),
              t::ElementsAreArray(expected));
}

TEST(SyscallAbiTest, UserProgramNameEqualsMatchesExactLengthStrings) {
  EXPECT_TRUE(UserProgramNameEquals("sh", 2, "sh"));
  EXPECT_TRUE(UserProgramNameEquals("init", 4, "init"));
  EXPECT_TRUE(UserProgramNameEquals("usertest", 8, "usertest"));
  EXPECT_TRUE(UserProgramNameEquals("ramfs", 5, "ramfs"));
  // Substring of longer buffer with exact length matches.
  EXPECT_TRUE(UserProgramNameEquals("sh_extra", 2, "sh"));
  // Mismatched lengths, embedded NULs, or null pointers do not match.
  EXPECT_FALSE(UserProgramNameEquals("sh", 1, "sh"));
  EXPECT_FALSE(UserProgramNameEquals("sh\0x", 4, "sh"));
  EXPECT_FALSE(UserProgramNameEquals("sh", 0, "sh"));
  EXPECT_FALSE(UserProgramNameEquals(nullptr, 2, "sh"));
  EXPECT_FALSE(UserProgramNameEquals("unknown", 7, "sh"));
}

// The console and process system calls of the shell: Read() returns `input`
// one byte per call, Write() appends to `output`, Spawn() starts `init`, `sh`
// and `usertest`, and each program ends, as Wait() reports, with exit status
// `arg`, except that it faults with vector 6 if `arg == 6` and is killed if
// `arg == 99`. `env` has these callbacks but no filesystem callbacks, and its
// `ctx` points to this object.
struct FakeShellState {
  FakeShellState() = default;
  FakeShellState(const FakeShellState&) = delete;
  FakeShellState& operator=(const FakeShellState&) = delete;

  std::string input;
  int64_t read_pos = 0;
  std::string output;
  int cpu_id = 3;
  struct SpawnCall {
    std::string name;
    uint64_t arg = 0;
  };
  std::vector<SpawnCall> spawns;
  sh::ShellEnv env{
      .ctx = this,
      .read = Read,
      .write = Write,
      .get_cpu = GetCpu,
      .spawn = Spawn,
      .wait = Wait,
  };

  // Executes command line `line`, which must not exit the shell, and returns
  // what it writes.
  std::string Run(const std::string& line) {
    output.clear();
    const StatusOr<int64_t> exit_status =
        sh::ExecuteLine(env, line.data(), static_cast<int>(line.size()));
    EXPECT_FALSE(exit_status.ok()) << line;
    return output;
  }

  static int64_t Read(void* const ctx, char* const buf, const int64_t size) {
    FakeShellState* const s = static_cast<FakeShellState*>(ctx);
    if (size <= 0 || s->read_pos >= static_cast<int64_t>(s->input.size())) {
      return 0;
    }
    buf[0] = s->input[static_cast<size_t>(s->read_pos)];
    ++s->read_pos;
    return 1;
  }

  static int64_t Write(void* const ctx,        //
                       const char* const buf,  //
                       const int64_t size) {
    FakeShellState* const s = static_cast<FakeShellState*>(ctx);
    if (buf != nullptr && size > 0) {
      s->output.append(buf, static_cast<size_t>(size));
    }
    return size;
  }

  static int GetCpu(void* const ctx) {
    return static_cast<FakeShellState*>(ctx)->cpu_id;
  }

  static int64_t Spawn(void* const ctx,         //
                       const char* const name,  //
                       const int64_t name_len,  //
                       const uint64_t arg) {
    FakeShellState* const s = static_cast<FakeShellState*>(ctx);
    const std::string prog(name, static_cast<size_t>(name_len));
    if (prog != "init" && prog != "sh" && prog != "usertest") {
      return kSyscallErrorInvalid;
    }
    s->spawns.push_back(SpawnCall{prog, arg});
    return static_cast<int64_t>(s->spawns.size());
  }

  static StatusOr<UserProgramResult> Wait(void* const ctx, const int64_t pid) {
    FakeShellState* const s = static_cast<FakeShellState*>(ctx);
    if (pid < 1 || pid > static_cast<int64_t>(s->spawns.size())) {
      return kSyscallErrorInvalid;
    }
    const SpawnCall& call = s->spawns[static_cast<size_t>(pid - 1)];
    UserProgramResult result{};
    if (call.arg == 6) {
      result.outcome = UserProgramOutcome::kFaulted;
      result.fault_vector = 6;
      result.fault_error_code = 0;
    } else if (call.arg == 99) {
      result.outcome = UserProgramOutcome::kKilled;
    } else {
      result.outcome = UserProgramOutcome::kExited;
      result.exit_status = static_cast<int64_t>(call.arg);
    }
    return result;
  }
};

TEST(ShellLibTest, LineEditorHandlesBackspaceCrLfAndClamping) {
  sh::ShellLineEditor editor;
  char ch = '\b';
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kNone));
  ch = static_cast<char>(0x7F);
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kNone));
  EXPECT_THAT(editor.length, t::Eq(0));

  ch = 'a';
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kEchoChar));
  ch = '\t';
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kEchoChar));
  EXPECT_THAT(ch, t::Eq(' '));
  ch = 'b';
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kEchoChar));
  ch = '\b';
  EXPECT_THAT(editor.FeedChar(&ch),
              t::Eq(sh::ShellCharAction::kBackspaceErase));
  ch = 'c';
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kEchoChar));
  ch = '\r';
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kSubmitLine));
  EXPECT_THAT(std::string(editor.buffer), t::Eq("a c"));
  editor.ResetLine();

  // '\n' right after '\r' is ignored rather than submitting an empty line.
  ch = '\n';
  EXPECT_THAT(editor.FeedChar(&ch),
              t::Eq(sh::ShellCharAction::kIgnoreLfAfterCr));
  // A subsequent '\n' submits an empty line.
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kSubmitLine));

  // Clamping at kShellMaxLineLength.
  editor.ResetLine();
  for (int i = 0; i < sh::kShellMaxLineLength + 10; ++i) {
    char c = 'x';
    const sh::ShellCharAction action = editor.FeedChar(&c);
    if (i < sh::kShellMaxLineLength) {
      EXPECT_THAT(action, t::Eq(sh::ShellCharAction::kEchoChar));
    } else {
      EXPECT_THAT(action, t::Eq(sh::ShellCharAction::kNone));
    }
  }
  EXPECT_THAT(editor.length, t::Eq(sh::kShellMaxLineLength));
}

TEST(ShellLibTest, TokenizeAndParseInt64HandleEdgeCases) {
  constexpr const char kLine[] = "  echo   hello\tworld  ";
  const sh::ShellCommand cmd = sh::TokenizeLine(kLine, sizeof(kLine) - 1);
  EXPECT_THAT(cmd.token_count, t::Eq(3));
  EXPECT_FALSE(cmd.truncated);
  EXPECT_TRUE(sh::TokenEquals(cmd.tokens[0], "echo"));
  EXPECT_TRUE(sh::TokenEquals(cmd.tokens[1], "hello"));
  EXPECT_TRUE(sh::TokenEquals(cmd.tokens[2], "world"));

  // A command holds `kShellMaxTokens` tokens; it drops more and is marked
  // truncated.
  std::string line;
  for (int i = 0; i < sh::kShellMaxTokens; ++i) {
    line += " t" + std::to_string(i);
  }
  const sh::ShellCommand full =
      sh::TokenizeLine(line.data(), static_cast<int>(line.size()));
  EXPECT_THAT(full.token_count, t::Eq(sh::kShellMaxTokens));
  EXPECT_FALSE(full.truncated);
  line += " dropped";
  const sh::ShellCommand over =
      sh::TokenizeLine(line.data(), static_cast<int>(line.size()));
  EXPECT_THAT(over.token_count, t::Eq(sh::kShellMaxTokens));
  EXPECT_TRUE(over.truncated);
  const std::string last = "t" + std::to_string(sh::kShellMaxTokens - 1);
  EXPECT_TRUE(
      sh::TokenEquals(over.tokens[sh::kShellMaxTokens - 1], last.c_str()));

  EXPECT_THAT(*sh::ParseInt64(sh::ShellToken{"0", 1}), t::Eq(0));
  EXPECT_THAT(*sh::ParseInt64(sh::ShellToken{"-42", 3}), t::Eq(-42));
  EXPECT_THAT(*sh::ParseInt64(sh::ShellToken{"0x10", 4}), t::Eq(16));
  EXPECT_THAT(*sh::ParseInt64(sh::ShellToken{"-9223372036854775808", 20}),
              t::Eq(INT64_MIN));
  EXPECT_FALSE(sh::ParseInt64(sh::ShellToken{"9223372036854775808", 19}).ok());
  EXPECT_FALSE(sh::ParseInt64(sh::ShellToken{"0x", 2}).ok());
  EXPECT_FALSE(sh::ParseInt64(sh::ShellToken{"-", 1}).ok());
  EXPECT_FALSE(sh::ParseInt64(sh::ShellToken{"12a", 3}).ok());

  EXPECT_THAT(*sh::ParseProgramArg(sh::ShellToken{"syscalls", 8}), t::Eq(1U));
  EXPECT_THAT(*sh::ParseProgramArg(sh::ShellToken{"fault", 5}), t::Eq(6U));
  EXPECT_THAT(*sh::ParseProgramArg(sh::ShellToken{"buffers", 7}), t::Eq(16U));
  EXPECT_FALSE(sh::ParseProgramArg(sh::ShellToken{"bad_alias", 9}).ok());
}

TEST(ShellLibTest, RunShellExecutesBuiltinsSpawnsProgramsAndExitsCleanly) {
  FakeShellState state;
  state.input =
      "\n"
      "   \n"
      "\b\x7f"
      "ecxx\b\bho hello shell\r\n"
      "help\n"
      "cpu\n"
      "programs\n"
      "usertest 0\n"
      "run usertest 6\n"
      "run usertest 99\n"
      "run\n"
      "usertest badarg\n"
      "unknown_cmd\n"
      "exit 7\n";
  const int64_t status = sh::RunShell(state.env);
  EXPECT_THAT(status, t::Eq(7));
  // The prompts, the echo of what was typed ("\b \b" erases a character), and
  // the output of each command.
  EXPECT_THAT(state.output,
              t::Eq("sh> \n"
                    "sh>    \n"
                    "sh> ecxx\b \b\b \bho hello shell\n"
                    "hello shell\n"
                    "sh> help\n"
                    "built-in commands: help echo cpu ls cat mkdir rm touch "
                    "programs run exit\n"
                    "user programs: init sh usertest\n"
                    "sh> cpu\n"
                    "cpu: 3\n"
                    "sh> programs\n"
                    "programs: init sh usertest\n"
                    "sh> usertest 0\n"
                    "sh: usertest exited with status 0\n"
                    "sh> run usertest 6\n"
                    "sh: usertest faulted vector=6 error=0x0\n"
                    "sh> run usertest 99\n"
                    "sh: usertest killed\n"
                    "sh> run\n"
                    "sh: usage: run <program> [arg]\n"
                    "sh> usertest badarg\n"
                    "sh: invalid argument: badarg\n"
                    "sh> unknown_cmd\n"
                    "sh: unknown command: unknown_cmd\n"
                    "sh> exit 7\n"));
}

// FakeShellState whose filesystem callbacks serve `fs` the way the kernel does,
// for client `kClient`: `fds` maps each file descriptor to the server's handle,
// or to 0 if the descriptor isn't open. FileRead() and ReadDir() fail with
// `file_read_error` and `read_dir_error` instead if those aren't 0, and
// `bad_fd_calls` counts the calls with a descriptor that isn't open.
struct RamfsShellHarness : FakeShellState {
  class SimpleBlockAlloc final : public ramfs::BlockAllocator {
   public:
    uint8_t* AllocBlock() override {
      // make_unique<uint8_t[]>() value-initializes: the block is zeroed.
      blocks_.push_back(std::make_unique<uint8_t[]>(ramfs::kBlockSize));
      return blocks_.back().get();
    }
    void FreeBlock(uint8_t* const /*block*/) override {}

   private:
    std::vector<std::unique_ptr<uint8_t[]>> blocks_;
  };

  static constexpr uint64_t kClient = 1;

  SimpleBlockAlloc alloc;
  std::unique_ptr<ramfs::FileSystem> fs = std::make_unique<ramfs::FileSystem>();
  uint64_t fds[kMaxOpenFiles] = {};
  int64_t file_read_error = 0;
  int64_t read_dir_error = 0;
  int bad_fd_calls = 0;

  RamfsShellHarness() {
    fs->Initialize(&alloc);
    env.open = Open;
    env.close = Close;
    env.file_read = FileRead;
    env.stat = Stat;
    env.read_dir = ReadDir;
    env.mkdir = MkDir;
    env.unlink = Unlink;
  }

  // Creates directory `path`.
  void MakeDir(const std::string& path) {
    ASSERT_THAT(fs->MkDir(path.data(), static_cast<int64_t>(path.size())),
                t::Eq(0))
        << path;
  }

  // Creates file `path`, or truncates it, and writes `contents` to it.
  void WriteFile(const std::string& path, const std::string& contents) {
    FsOpenReply rep{};
    ASSERT_THAT(fs->Open(kClient,                                         //
                         path.data(),                                     //
                         static_cast<int64_t>(path.size()),               //
                         kFsOpenWrite | kFsOpenCreate | kFsOpenTruncate,  //
                         &rep),
                t::Eq(0))
        << path;
    const int64_t total = static_cast<int64_t>(contents.size());
    for (int64_t done = 0; done < total;) {
      const int64_t size = (total - done < kSyscallFsMaxIoSize)
                               ? total - done
                               : kSyscallFsMaxIoSize;
      ASSERT_THAT(fs->Write(kClient, rep.handle, contents.data() + done, size),
                  t::Eq(size))
          << path;
      done += size;
    }
    ASSERT_THAT(fs->Close(kClient, rep.handle), t::Eq(0)) << path;
  }

  // Returns the type of the file or directory at `path`, or
  // `FsNodeType::kNone` if there is none.
  FsNodeType TypeOf(const std::string& path) const {
    FsStat st{};
    const int64_t rc =
        fs->Stat(path.data(), static_cast<int64_t>(path.size()), &st);
    return (rc == 0) ? st.type : FsNodeType::kNone;
  }

  static RamfsShellHarness* Self(void* const ctx) {
    return static_cast<RamfsShellHarness*>(static_cast<FakeShellState*>(ctx));
  }

  // Returns the server's handle of file descriptor `fd`, or 0 if `fd` isn't
  // open (a bad call).
  uint64_t Handle(const int64_t fd) {
    if (fd < 0 || fd >= kMaxOpenFiles || fds[fd] == 0) {
      ++bad_fd_calls;
      return 0;
    }
    return fds[fd];
  }

  static int64_t Open(void* const ctx,         //
                      const char* const path,  //
                      const int64_t path_len,  //
                      const uint64_t flags) {
    RamfsShellHarness* const h = Self(ctx);
    int fd = 0;
    while (fd < kMaxOpenFiles && h->fds[fd] != 0) {
      ++fd;
    }
    if (fd == kMaxOpenFiles) {
      return kSyscallErrorTooManyFiles;
    }
    FsOpenReply rep{};
    const int64_t rc = h->fs->Open(kClient, path, path_len, flags, &rep);
    if (rc < 0) {
      return rc;
    }
    h->fds[fd] = rep.handle;
    return fd;
  }

  static int64_t Close(void* const ctx, const int64_t fd) {
    RamfsShellHarness* const h = Self(ctx);
    const uint64_t handle = h->Handle(fd);
    if (handle == 0) {
      return kSyscallErrorBadFd;
    }
    h->fds[fd] = 0;
    return h->fs->Close(kClient, handle);
  }

  static int64_t FileRead(void* const ctx,   //
                          const int64_t fd,  //
                          void* const buf,   //
                          const int64_t size) {
    RamfsShellHarness* const h = Self(ctx);
    const uint64_t handle = h->Handle(fd);
    if (handle == 0) {
      return kSyscallErrorBadFd;
    }
    if (h->file_read_error != 0) {
      return h->file_read_error;
    }
    return h->fs->Read(kClient, handle, buf, size);
  }

  static int64_t Stat(void* const ctx,         //
                      const char* const path,  //
                      const int64_t path_len,  //
                      FsStat* const out_stat) {
    return Self(ctx)->fs->Stat(path, path_len, out_stat);
  }

  static int64_t ReadDir(void* const ctx,            //
                         const int64_t fd,           //
                         FsDirEntry* const entries,  //
                         const int64_t max_entries) {
    RamfsShellHarness* const h = Self(ctx);
    const uint64_t handle = h->Handle(fd);
    if (handle == 0) {
      return kSyscallErrorBadFd;
    }
    if (h->read_dir_error != 0) {
      return h->read_dir_error;
    }
    return h->fs->ReadDir(kClient, handle, entries, max_entries);
  }

  static int64_t MkDir(void* const ctx,         //
                       const char* const path,  //
                       const int64_t path_len) {
    return Self(ctx)->fs->MkDir(path, path_len);
  }

  static int64_t Unlink(void* const ctx,         //
                        const char* const path,  //
                        const int64_t path_len) {
    return Self(ctx)->fs->Unlink(path, path_len);
  }
};

TEST(ShellLibTest, FsErrorMessagesDescribeTheErrorOrGiveItsNumber) {
  const auto message = [](const int64_t rc) {
    sh::ShellOutBuffer out;
    sh::AppendFsErrorMessage(out, rc);
    return std::string(out.data);
  };
  EXPECT_THAT(message(kSyscallErrorNoEntry),
              t::Eq("no such file or directory"));
  EXPECT_THAT(message(kSyscallErrorIo), t::Eq("io error"));
  EXPECT_THAT(message(kSyscallErrorBadFd), t::Eq("bad file descriptor"));
  EXPECT_THAT(message(kSyscallErrorNoMemory), t::Eq("out of memory"));
  EXPECT_THAT(message(kSyscallErrorFault), t::Eq("bad address"));
  EXPECT_THAT(message(kSyscallErrorBusy), t::Eq("device or resource busy"));
  EXPECT_THAT(message(kSyscallErrorExists), t::Eq("file exists"));
  EXPECT_THAT(message(kSyscallErrorNoDevice), t::Eq("filesystem unavailable"));
  EXPECT_THAT(message(kSyscallErrorNotDir), t::Eq("not a directory"));
  EXPECT_THAT(message(kSyscallErrorIsDir), t::Eq("is a directory"));
  EXPECT_THAT(message(kSyscallErrorInvalid), t::Eq("invalid argument"));
  EXPECT_THAT(message(kSyscallErrorFileTableFull),
              t::Eq("too many open files in system"));
  EXPECT_THAT(message(kSyscallErrorTooManyFiles), t::Eq("too many open files"));
  EXPECT_THAT(message(kSyscallErrorFileTooBig), t::Eq("file too large"));
  EXPECT_THAT(message(kSyscallErrorNoSpace), t::Eq("no space left"));
  EXPECT_THAT(message(kSyscallErrorNameTooLong), t::Eq("name too long"));
  EXPECT_THAT(message(kSyscallErrorNotEmpty), t::Eq("directory not empty"));
  EXPECT_THAT(message(kSyscallErrorStale), t::Eq("stale file descriptor"));
  // Errors without a description.
  EXPECT_THAT(message(kSyscallErrorAccess), t::Eq("error -13"));
  EXPECT_THAT(message(kSyscallErrorDeadlock), t::Eq("error -35"));
  EXPECT_THAT(message(-4095), t::Eq("error -4095"));
}

TEST(ShellLibTest, FilesystemBuiltinsWithoutAFilesystemReportItUnavailable) {
  FakeShellState no_fs;
  EXPECT_THAT(no_fs.Run("ls"), t::Eq("sh: ls: /: filesystem unavailable\n"));
  EXPECT_THAT(no_fs.Run("ls /a /b"),
              t::Eq("sh: ls: /a: filesystem unavailable\n"
                    "sh: ls: /b: filesystem unavailable\n"));
  EXPECT_THAT(no_fs.Run("cat /a"),
              t::Eq("sh: cat: /a: filesystem unavailable\n"));
  EXPECT_THAT(no_fs.Run("mkdir /a"),
              t::Eq("sh: mkdir: /a: filesystem unavailable\n"));
  EXPECT_THAT(no_fs.Run("rm /a"),
              t::Eq("sh: rm: /a: filesystem unavailable\n"));
  EXPECT_THAT(no_fs.Run("touch /a"),
              t::Eq("sh: touch: /a: filesystem unavailable\n"));
}

// A built-in that lacks one of the callbacks that it needs calls none of the
// others: it changes nothing and opens no file that it then couldn't read,
// list or close.
TEST(ShellLibTest, FilesystemBuiltinsNeedEachOfTheirCallbacks) {
  const struct {
    const char* cmd;
    const char* path;
    void (*remove)(sh::ShellEnv& env);
  } kCases[] = {
      {"ls", "/d", [](sh::ShellEnv& env) { env.stat = nullptr; }},
      {"ls", "/d", [](sh::ShellEnv& env) { env.open = nullptr; }},
      {"ls", "/d", [](sh::ShellEnv& env) { env.read_dir = nullptr; }},
      {"ls", "/d", [](sh::ShellEnv& env) { env.close = nullptr; }},
      {"cat", "/d/f", [](sh::ShellEnv& env) { env.open = nullptr; }},
      {"cat", "/d/f", [](sh::ShellEnv& env) { env.file_read = nullptr; }},
      {"cat", "/d/f", [](sh::ShellEnv& env) { env.close = nullptr; }},
      {"touch", "/d/new", [](sh::ShellEnv& env) { env.open = nullptr; }},
      {"touch", "/d/new", [](sh::ShellEnv& env) { env.close = nullptr; }},
      {"mkdir", "/d/new", [](sh::ShellEnv& env) { env.mkdir = nullptr; }},
      {"rm", "/d/f", [](sh::ShellEnv& env) { env.unlink = nullptr; }},
  };
  for (const auto& c : kCases) {
    RamfsShellHarness h;
    h.MakeDir("/d");
    h.WriteFile("/d/f", "data");
    const int inodes = h.fs->ActiveInodeCount();
    c.remove(h.env);
    const std::string line = std::string(c.cmd) + " " + c.path;
    const std::string error = "sh: " + std::string(c.cmd) + ": " + c.path +
                              ": filesystem unavailable\n";
    EXPECT_THAT(h.Run(line), t::Eq(error)) << line;
    EXPECT_THAT(h.fs->ActiveHandleCount(), t::Eq(0)) << line;
    EXPECT_THAT(h.fs->ActiveInodeCount(), t::Eq(inodes)) << line;
  }
}

TEST(ShellLibTest, FilesystemBuiltinsOperateOnRamfsAndReportErrors) {
  RamfsShellHarness h;
  // Longer than two of cat's reads.
  const std::string readme =
      std::string(2 * sh::kShellCatChunkSize + 100, 'A') + "END_OF_README\n";
  h.MakeDir("/docs");
  h.WriteFile("/docs/readme.txt", readme);
  const int inodes = h.fs->ActiveInodeCount();

  EXPECT_THAT(h.Run("mkdir"), t::Eq("sh: usage: mkdir <path> ...\n"));
  EXPECT_THAT(h.Run("touch"), t::Eq("sh: usage: touch <path> ...\n"));
  EXPECT_THAT(h.Run("cat"), t::Eq("sh: usage: cat <path> ...\n"));
  EXPECT_THAT(h.Run("rm"), t::Eq("sh: usage: rm <path> ...\n"));

  EXPECT_THAT(h.Run("mkdir /work /work/sub"), t::Eq(""));
  EXPECT_THAT(h.TypeOf("/work/sub"), t::Eq(FsNodeType::kDirectory));
  EXPECT_THAT(h.Run("mkdir /work /nodir/x"),
              t::Eq("sh: mkdir: /work: file exists\n"
                    "sh: mkdir: /nodir/x: no such file or directory\n"));
  // touch creates missing files and leaves existing files and directories
  // alone.
  EXPECT_THAT(h.Run("touch /work/empty /docs/readme.txt /work"), t::Eq(""));
  EXPECT_THAT(h.TypeOf("/work/empty"), t::Eq(FsNodeType::kRegular));
  EXPECT_THAT(h.TypeOf("/work"), t::Eq(FsNodeType::kDirectory));
  EXPECT_THAT(h.Run("touch /nodir/x"),
              t::Eq("sh: touch: /nodir/x: no such file or directory\n"));

  // ls lists the entries in the order in which they were created.
  EXPECT_THAT(h.Run("ls"), t::Eq("docs/\nwork/\n"));
  EXPECT_THAT(h.Run("ls /work"), t::Eq("sub/\nempty\n"));
  EXPECT_THAT(h.Run("ls /work/sub"), t::Eq(""));
  EXPECT_THAT(h.Run("ls /docs/readme.txt"), t::Eq("/docs/readme.txt\n"));
  EXPECT_THAT(h.Run("ls /missing"),
              t::Eq("sh: ls: /missing: no such file or directory\n"));

  EXPECT_THAT(h.Run("cat /docs/readme.txt"), t::Eq(readme));
  EXPECT_THAT(h.Run("cat /work/empty"), t::Eq(""));
  EXPECT_THAT(h.Run("cat /work /missing /docs/readme.txt"),
              t::Eq("sh: cat: /work: is a directory\n"
                    "sh: cat: /missing: no such file or directory\n" +
                    readme));

  // Errors of ReadDir() and FileRead(), which come after the directory or file
  // was opened.
  h.read_dir_error = kSyscallErrorNoMemory;
  EXPECT_THAT(h.Run("ls /docs"), t::Eq("sh: ls: /docs: out of memory\n"));
  h.read_dir_error = 0;
  h.file_read_error = kSyscallErrorAccess;
  EXPECT_THAT(h.Run("cat /docs/readme.txt"),
              t::Eq("sh: cat: /docs/readme.txt: error -13\n"));
  h.file_read_error = 0;

  // rm removes files and empty directories.
  EXPECT_THAT(h.Run("rm /work"), t::Eq("sh: rm: /work: directory not empty\n"));
  EXPECT_THAT(h.TypeOf("/work"), t::Eq(FsNodeType::kDirectory));
  EXPECT_THAT(h.Run("rm /work/empty /work/sub /work /missing"),
              t::Eq("sh: rm: /missing: no such file or directory\n"));
  EXPECT_THAT(h.TypeOf("/work"), t::Eq(FsNodeType::kNone));
  EXPECT_THAT(h.fs->ActiveInodeCount(), t::Eq(inodes));
  EXPECT_THAT(h.Run("ls"), t::Eq("docs/\n"));

  // The shell closed every file that it opened, also after errors.
  EXPECT_THAT(h.fs->ActiveHandleCount(), t::Eq(0));
  EXPECT_THAT(h.fds, t::Each(t::Eq(0U)));
  EXPECT_THAT(h.bad_fd_calls, t::Eq(0));
}

TEST(ShellLibTest, LsListsDirectoriesOfMoreEntriesThanOneReadDirBatch) {
  RamfsShellHarness h;
  h.MakeDir("/many");
  std::string listing;
  for (int i = 0; i < 2 * sh::kShellReadDirBatch + 3; ++i) {
    const std::string name = "entry" + std::to_string(i);
    if (i % 3 == 0) {
      h.MakeDir("/many/" + name);
      listing += name + "/\n";
    } else {
      h.WriteFile("/many/" + name, "");
      listing += name + "\n";
    }
  }
  EXPECT_THAT(h.Run("ls /many"), t::Eq(listing));
  EXPECT_THAT(h.fs->ActiveHandleCount(), t::Eq(0));
  EXPECT_THAT(h.bad_fd_calls, t::Eq(0));
}

TEST(ShellLibTest, LsWithSeveralOperandsGroupsThemLikePosixLs) {
  RamfsShellHarness h;
  h.MakeDir("/docs");
  h.WriteFile("/docs/readme.txt", "hello\n");
  h.MakeDir("/work");
  h.MakeDir("/work/sub");
  h.WriteFile("/work/empty", "");
  h.MakeDir("/void");

  // Directories: each one's entries follow a "<path>:" line, and blank lines
  // separate them.
  EXPECT_THAT(h.Run("ls /docs /void /work"), t::Eq("/docs:\n"
                                                   "readme.txt\n"
                                                   "\n"
                                                   "/void:\n"
                                                   "\n"
                                                   "/work:\n"
                                                   "sub/\n"
                                                   "empty\n"));
  // The other operands come first, in their order, with the errors.
  EXPECT_THAT(h.Run("ls /work /docs/readme.txt /missing /work/empty"),
              t::Eq("/docs/readme.txt\n"
                    "sh: ls: /missing: no such file or directory\n"
                    "/work/empty\n"
                    "\n"
                    "/work:\n"
                    "sub/\n"
                    "empty\n"));
  EXPECT_THAT(h.Run("ls /docs/readme.txt /work/empty"),
              t::Eq("/docs/readme.txt\n/work/empty\n"));
  // No blank line follows an error.
  EXPECT_THAT(h.Run("ls /missing /docs"),
              t::Eq("sh: ls: /missing: no such file or directory\n"
                    "/docs:\n"
                    "readme.txt\n"));
  EXPECT_THAT(h.fs->ActiveHandleCount(), t::Eq(0));
  EXPECT_THAT(h.bad_fd_calls, t::Eq(0));
}

// A line holds at most `kShellMaxTokens` tokens. Rather than skip operands, the
// filesystem built-ins refuse to run with more.
TEST(ShellLibTest, FilesystemBuiltinsRefuseLinesWithTooManyTokens) {
  RamfsShellHarness h;
  h.MakeDir("/t");
  const int inodes = h.fs->ActiveInodeCount();
  // " /t/0 /t/1 ...": as many operands as a command holds, and one more.
  std::string most;
  for (int i = 0; i < sh::kShellMaxTokens - 1; ++i) {
    most += " /t/" + std::to_string(i);
  }
  const std::string too_many = most + " /t/extra";

  for (const char* const cmd : {"ls", "cat", "mkdir", "rm", "touch"}) {
    EXPECT_THAT(h.Run(cmd + too_many), t::Eq("sh: too many arguments\n"))
        << cmd;
  }
  EXPECT_THAT(h.fs->ActiveInodeCount(), t::Eq(inodes));

  EXPECT_THAT(h.Run("touch" + most), t::Eq(""));
  EXPECT_THAT(h.fs->ActiveInodeCount(),
              t::Eq(inodes + sh::kShellMaxTokens - 1));
  EXPECT_THAT(h.Run("touch /t/extra"), t::Eq(""));
  EXPECT_THAT(h.Run("rm" + too_many), t::Eq("sh: too many arguments\n"));
  EXPECT_THAT(h.fs->ActiveInodeCount(), t::Eq(inodes + sh::kShellMaxTokens));
  EXPECT_THAT(h.Run("rm" + most), t::Eq(""));
  EXPECT_THAT(h.Run("ls /t"), t::Eq("extra\n"));

  // echo writes the arguments that the command holds instead.
  EXPECT_THAT(h.Run("echo" + too_many), t::Eq(most.substr(1) + "\n"));
}

// FakeShellState whose filesystem callbacks claim more than they may: the
// first ReadDir() claims more entries than asked for, with names longer than
// `kFsMaxNameLength` or of negative length, and the first FileRead() claims
// more bytes than asked for. `/` is a directory and every other path a file.
struct OverclaimingFs : FakeShellState {
  int read_dir_calls = 0;
  int file_read_calls = 0;

  OverclaimingFs() {
    env.open = Open;
    env.close = Close;
    env.stat = Stat;
    env.read_dir = ReadDir;
    env.file_read = FileRead;
  }

  static OverclaimingFs* Self(void* const ctx) {
    return static_cast<OverclaimingFs*>(static_cast<FakeShellState*>(ctx));
  }

  static int64_t Open(void* const /*ctx*/,         //
                      const char* const /*path*/,  //
                      const int64_t /*path_len*/,  //
                      const uint64_t /*flags*/) {
    return 3;
  }

  static int64_t Close(void* const /*ctx*/, const int64_t /*fd*/) { return 0; }

  static int64_t Stat(void* const /*ctx*/,     //
                      const char* const path,  //
                      const int64_t path_len,  //
                      FsStat* const out_stat) {
    *out_stat = FsStat{};
    out_stat->type = (path_len == 1 && path[0] == '/') ? FsNodeType::kDirectory
                                                       : FsNodeType::kRegular;
    return 0;
  }

  static int64_t ReadDir(void* const ctx,            //
                         const int64_t /*fd*/,       //
                         FsDirEntry* const entries,  //
                         const int64_t max_entries) {
    if (Self(ctx)->read_dir_calls++ > 0) {
      return 0;
    }
    for (int64_t i = 0; i < max_entries; ++i) {
      entries[i] = FsDirEntry{};
      __builtin_memset(entries[i].name, 'n', sizeof(entries[i].name));
      entries[i].name_len = (i % 2 == 0) ? 1000 : -1;
    }
    return max_entries + 100;
  }

  static int64_t FileRead(void* const ctx,       //
                          const int64_t /*fd*/,  //
                          void* const buf,       //
                          const int64_t size) {
    if (Self(ctx)->file_read_calls++ > 0) {
      return 0;
    }
    __builtin_memset(buf, 'x', static_cast<size_t>(size));
    return size + 100;
  }
};

TEST(ShellLibTest, LsAndCatClampWhatCallbacksClaimToTheirBuffers) {
  OverclaimingFs fs;
  std::string listing;
  for (int i = 0; i < sh::kShellReadDirBatch; ++i) {
    listing +=
        (i % 2 == 0) ? std::string(kFsMaxNameLength, 'n') : std::string();
    listing += "\n";
  }
  EXPECT_THAT(fs.Run("ls"), t::Eq(listing));
  EXPECT_THAT(fs.Run("cat /f"),
              t::Eq(std::string(sh::kShellCatChunkSize, 'x')));
}

}  // namespace
}  // namespace protos
