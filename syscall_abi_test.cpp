#include "syscall_abi.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include "block_protocol.h"
#include "fs_protocol.h"
#include "user/block.h"
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
  EXPECT_THAT(kSyscallBootModuleInfo, t::Eq(35));
  EXPECT_THAT(kSyscallBootModuleRead, t::Eq(36));
  EXPECT_THAT(kSyscallSpawnFile, t::Eq(37));
  EXPECT_THAT(kSyscallTryWait, t::Eq(38));
  EXPECT_THAT(kSyscallCount, t::Eq(39));
  EXPECT_THAT(kIsaDeviceI8042, t::Eq(0));
  EXPECT_THAT(kIsaDeviceCount, t::Eq(1));
  EXPECT_THAT(kSyscallPortIoRead8, t::Eq(0));
  EXPECT_THAT(kSyscallPortIoWrite8, t::Eq(1));
  EXPECT_THAT(kSyscallWriteMaxSize, t::Eq(4096));
  EXPECT_THAT(kSyscallSpinMaxSize, t::Eq(4096));
  EXPECT_THAT(kSyscallSpinMaxTicks, t::Eq(1000000));
  EXPECT_THAT(kSyscallReadMaxSize, t::Eq(4096));
  EXPECT_THAT(kSyscallSpawnMaxNameLength, t::Eq(64));
  EXPECT_THAT(kSyscallSpawnMaxArgs, t::Eq(16));
  EXPECT_THAT(kSyscallSpawnMaxArgBytes, t::Eq(1024));
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
  EXPECT_THAT(kSyscallErrorExecFormat, t::Eq(-8));
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
  EXPECT_THAT(kIpcPortBlock, t::Eq(1));
  EXPECT_THAT(kIpcFirstUserPort, t::Eq(2));
  EXPECT_THAT(kIpcPortCount, t::Eq(16));
  EXPECT_THAT(kBlockMinBlockSize, t::Eq(512));
  EXPECT_THAT(kBlockMaxIoBytes, t::Eq(4096));
  EXPECT_THAT(kBlockModelCapacity, t::Eq(40));
  EXPECT_THAT(kBlockSerialCapacity, t::Eq(24));
  EXPECT_THAT(kIpcMaxMessageSize, t::Eq(4352));
  EXPECT_THAT(kSyscallIpcTimeoutMaxTicks, t::Eq(1000000));
  EXPECT_THAT(kSyscallMemMapMaxSize, t::Eq(1LL << 20));
  EXPECT_THAT(kSyscallMemMapMaxTotalSize, t::Eq(64LL << 20));
  EXPECT_THAT(kSyscallBootModuleReadMaxSize, t::Eq(4096));
  EXPECT_THAT(kBootModuleMaxNameLength, t::Eq(63));
  EXPECT_THAT(std::string(kRootfsBootModuleName), t::Eq("rootfs"));
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
  EXPECT_THAT(kInitArgEmptyFs, t::Eq(1ULL << 2));
  EXPECT_THAT(kDriverArgNormal, t::Eq(0ULL));
  EXPECT_THAT(kDriverArgExitAfterReady, t::Eq(1ULL));
  EXPECT_THAT(kDriverArgFaultAfterReady, t::Eq(2ULL));
  EXPECT_THAT(kDriverArgHandoffOnly, t::Eq(3ULL));
  EXPECT_THAT(kRamfsArgEmpty, t::Eq(4ULL));
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
  // 64-bit values whose lower 32 bits lie in [-4095, -1] must not be treated
  // as syscall errors via 32-bit truncation.
  EXPECT_FALSE(SyscallIsError(-(int64_t{1} << 32) - 1));
  EXPECT_FALSE(SyscallIsError(-(int64_t{1} << 32) - 22));
  EXPECT_FALSE(SyscallIsError(-(int64_t{1} << 32) - 4095));
  EXPECT_FALSE(SyscallIsError((int64_t{1} << 32) - 1));
  EXPECT_FALSE(SyscallIsError((int64_t{1} << 32) - 22));
  EXPECT_FALSE(SyscallIsError((int64_t{1} << 32) - 4095));
  EXPECT_FALSE(SyscallIsError(static_cast<int64_t>(0xFFFF9000FFFFFFFFULL)));

  for (int64_t num = -1; num <= kSyscallCount + 2; ++num) {
    const bool expected = (num >= 8 && num <= 15) || (num >= 17 && num <= 19) ||
                          (num >= 35 && num <= 36);
    EXPECT_THAT(SyscallIsDriverOnly(num), t::Eq(expected)) << "syscall " << num;
  }

  for (int64_t num = -1; num <= kSyscallCount + 2; ++num) {
    const bool is_spawn_drv = (num == kSyscallSpawnDriver);
    const bool is_drv_only =
        (num >= kSyscallPciInfo && num <= kSyscallConsolePush) ||
        (num >= kSyscallIsaClaim && num <= kSyscallIsaIrqAlloc) ||
        num == kSyscallBootModuleInfo || num == kSyscallBootModuleRead;
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
  EXPECT_FALSE(SyscallIsDriverOnly(kSyscallTryWait));
  EXPECT_TRUE(MayMakeSyscall(kUserCapNone, kSyscallTryWait));
  EXPECT_FALSE(SyscallIsDriverOnly((int64_t{1} << 32) | kSyscallPciInfo));
  EXPECT_FALSE(SyscallIsDriverOnly(-(int64_t{1} << 32) + kSyscallPciInfo));
  EXPECT_FALSE(
      SyscallIsDriverOnly((int64_t{1} << 32) | kSyscallBootModuleRead));
  EXPECT_TRUE(
      MayMakeSyscall(kUserCapNone, (int64_t{1} << 32) | kSyscallPciInfo));
  EXPECT_TRUE(
      MayMakeSyscall(kUserCapNone, -(int64_t{1} << 32) + kSyscallPciInfo));
  EXPECT_TRUE(
      MayMakeSyscall(kUserCapNone, (int64_t{1} << 32) | kSyscallSpawnDriver));
  EXPECT_TRUE(
      MayMakeSyscall(kUserCapNone, -(int64_t{1} << 32) + kSyscallSpawnDriver));

  for (int64_t port = -1; port <= kIpcPortCount; ++port) {
    const bool driver_port = (port == kIpcPortFs || port == kIpcPortBlock);
    EXPECT_THAT(IpcPortRequiresDriverCap(port), t::Eq(driver_port))
        << "port " << port;
    EXPECT_THAT(MayListenOnIpcPort(kUserCapNone, port), t::Eq(!driver_port))
        << "port " << port;
    EXPECT_TRUE(MayListenOnIpcPort(kUserCapDriver, port)) << "port " << port;
    EXPECT_THAT(MayListenOnIpcPort(kUserCapSpawnDrivers, port),
                t::Eq(!driver_port))
        << "port " << port;
    EXPECT_TRUE(MayListenOnIpcPort(kUserCapDriver | kUserCapSpawnDrivers, port))
        << "port " << port;
  }
  EXPECT_FALSE(IpcPortRequiresDriverCap((int64_t{1} << 32) | kIpcPortFs));
  EXPECT_FALSE(IpcPortRequiresDriverCap((int64_t{1} << 32) | kIpcPortBlock));
  EXPECT_FALSE(IpcPortRequiresDriverCap(-(int64_t{1} << 32) + kIpcPortFs));
  EXPECT_FALSE(IpcPortRequiresDriverCap(-(int64_t{1} << 32) + kIpcPortBlock));
  EXPECT_TRUE(
      MayListenOnIpcPort(kUserCapNone, (int64_t{1} << 32) | kIpcPortFs));
  EXPECT_TRUE(
      MayListenOnIpcPort(kUserCapNone, (int64_t{1} << 32) | kIpcPortBlock));
  EXPECT_TRUE(
      MayListenOnIpcPort(kUserCapNone, -(int64_t{1} << 32) + kIpcPortFs));
  EXPECT_TRUE(
      MayListenOnIpcPort(kUserCapNone, -(int64_t{1} << 32) + kIpcPortBlock));

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

TEST(SyscallAbiTest, BlockProtocolEnumsAndLayoutsStayTheSame) {
  EXPECT_TRUE((std::is_same_v<std::underlying_type_t<BlockRpcOp>, uint32_t>));
  EXPECT_THAT(static_cast<uint32_t>(BlockRpcOp::kNone), t::Eq(0U));
  EXPECT_THAT(static_cast<uint32_t>(BlockRpcOp::kGetInfo), t::Eq(1U));
  EXPECT_THAT(static_cast<uint32_t>(BlockRpcOp::kRead), t::Eq(2U));
  EXPECT_THAT(static_cast<uint32_t>(BlockRpcOp::kWrite), t::Eq(3U));
  EXPECT_THAT(static_cast<uint32_t>(BlockRpcOp::kFlush), t::Eq(4U));

  EXPECT_THAT(sizeof(BlockGetInfoRequest), t::Eq(8U));
  EXPECT_THAT(alignof(BlockGetInfoRequest), t::Eq(4U));
  EXPECT_THAT(offsetof(BlockGetInfoRequest, op), t::Eq(0U));
  EXPECT_THAT(offsetof(BlockGetInfoRequest, disk_index), t::Eq(4U));
  EXPECT_TRUE(std::has_unique_object_representations_v<BlockGetInfoRequest>);

  EXPECT_THAT(sizeof(BlockDeviceInfo), t::Eq(88U));
  EXPECT_THAT(alignof(BlockDeviceInfo), t::Eq(8U));
  EXPECT_THAT(offsetof(BlockDeviceInfo, block_count), t::Eq(0U));
  EXPECT_THAT(offsetof(BlockDeviceInfo, block_size), t::Eq(8U));
  EXPECT_THAT(offsetof(BlockDeviceInfo, max_transfer_blocks), t::Eq(12U));
  EXPECT_THAT(offsetof(BlockDeviceInfo, bus), t::Eq(16U));
  EXPECT_THAT(offsetof(BlockDeviceInfo, device), t::Eq(17U));
  EXPECT_THAT(offsetof(BlockDeviceInfo, function), t::Eq(18U));
  EXPECT_THAT(offsetof(BlockDeviceInfo, read_only), t::Eq(19U));
  EXPECT_THAT(offsetof(BlockDeviceInfo, nsid), t::Eq(20U));
  EXPECT_THAT(offsetof(BlockDeviceInfo, model), t::Eq(24U));
  EXPECT_THAT(offsetof(BlockDeviceInfo, serial), t::Eq(64U));
  EXPECT_THAT(sizeof(BlockDeviceInfo::model),
              t::Eq(static_cast<size_t>(kBlockModelCapacity)));
  EXPECT_THAT(sizeof(BlockDeviceInfo::serial),
              t::Eq(static_cast<size_t>(kBlockSerialCapacity)));
  EXPECT_TRUE(std::has_unique_object_representations_v<BlockDeviceInfo>);

  EXPECT_THAT(sizeof(BlockReadRequest), t::Eq(24U));
  EXPECT_THAT(alignof(BlockReadRequest), t::Eq(8U));
  EXPECT_THAT(offsetof(BlockReadRequest, op), t::Eq(0U));
  EXPECT_THAT(offsetof(BlockReadRequest, disk_index), t::Eq(4U));
  EXPECT_THAT(offsetof(BlockReadRequest, lba), t::Eq(8U));
  EXPECT_THAT(offsetof(BlockReadRequest, block_count), t::Eq(16U));
  EXPECT_THAT(offsetof(BlockReadRequest, reserved), t::Eq(20U));
  EXPECT_TRUE(std::has_unique_object_representations_v<BlockReadRequest>);

  EXPECT_THAT(sizeof(BlockWriteRequest), t::Eq(24U));
  EXPECT_THAT(alignof(BlockWriteRequest), t::Eq(8U));
  EXPECT_THAT(offsetof(BlockWriteRequest, op), t::Eq(0U));
  EXPECT_THAT(offsetof(BlockWriteRequest, disk_index), t::Eq(4U));
  EXPECT_THAT(offsetof(BlockWriteRequest, lba), t::Eq(8U));
  EXPECT_THAT(offsetof(BlockWriteRequest, block_count), t::Eq(16U));
  EXPECT_THAT(offsetof(BlockWriteRequest, reserved), t::Eq(20U));
  EXPECT_TRUE(std::has_unique_object_representations_v<BlockWriteRequest>);
  EXPECT_THAT(
      static_cast<int64_t>(sizeof(BlockWriteRequest)) + kBlockMaxIoBytes,
      t::Le(kIpcMaxMessageSize));

  EXPECT_THAT(sizeof(BlockFlushRequest), t::Eq(8U));
  EXPECT_THAT(alignof(BlockFlushRequest), t::Eq(4U));
  EXPECT_THAT(offsetof(BlockFlushRequest, op), t::Eq(0U));
  EXPECT_THAT(offsetof(BlockFlushRequest, disk_index), t::Eq(4U));
  EXPECT_TRUE(std::has_unique_object_representations_v<BlockFlushRequest>);
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

TEST(SyscallAbiTest, BootModuleInfoLayoutStaysTheSame) {
  EXPECT_THAT(sizeof(BootModuleInfo), t::Eq(72U));
  EXPECT_THAT(alignof(BootModuleInfo), t::Eq(8U));
  EXPECT_THAT(offsetof(BootModuleInfo, size), t::Eq(0U));
  EXPECT_THAT(offsetof(BootModuleInfo, name), t::Eq(8U));
  EXPECT_THAT(sizeof(BootModuleInfo::name),
              t::Eq(static_cast<size_t>(kBootModuleMaxNameLength + 1)));
  EXPECT_TRUE((std::is_same_v<decltype(BootModuleInfo::size), int64_t>));
  // The defaults are zeros, which the kernel doesn't rely on: it builds the
  // struct that it writes in zeroed memory and copies at most
  // `kBootModuleMaxNameLength` bytes of the name into it
  // (`UserProcess::GetBootModuleInfo()`, user_mode.cpp), so a name always ends
  // with NUL bytes.
  const BootModuleInfo info{};
  EXPECT_THAT(info.size, t::Eq(0));
  EXPECT_THAT(info.name, t::Each(t::Eq('\0')));
  // The boot filesystem's name fits.
  EXPECT_THAT(sizeof(kRootfsBootModuleName),
              t::Le(static_cast<size_t>(kBootModuleMaxNameLength + 1)));

  EXPECT_THAT(sizeof(SyscallArgSlice), t::Eq(16U));
  EXPECT_THAT(alignof(SyscallArgSlice), t::Eq(8U));
  EXPECT_THAT(offsetof(SyscallArgSlice, ptr), t::Eq(0U));
  EXPECT_THAT(offsetof(SyscallArgSlice, len), t::Eq(8U));
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
// filesystem or block request header, and zeros in all others.
template <typename T, typename Op>
static std::vector<uint8_t> OpcodeAndZeros(const Op op) {
  static_assert(sizeof(Op) == sizeof(uint32_t));
  std::vector<uint8_t> bytes(sizeof(T), 0);
  const uint32_t value = static_cast<uint32_t>(op);
  __builtin_memcpy(bytes.data(), &value, sizeof(value));
  return bytes;
}

// The kernel value-initializes each request header that it sends to the
// filesystem server (`FsCloseRequest req{}` and so on, user_mode.cpp) and then
// sets the opcode and the fields that the request uses, so the defaults are
// what the server gets in every other byte: they must be zero, as the protocol
// requires of reserved fields (fs_protocol.h, block_protocol.h). Likewise for
// `IpcReceivedMessage msg{}` and `BlockDeviceInfo info{}`.
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
  EXPECT_THAT(ValueInitializedBytes<BlockGetInfoRequest>(),
              t::ElementsAreArray(
                  OpcodeAndZeros<BlockGetInfoRequest>(BlockRpcOp::kGetInfo)));
  EXPECT_THAT(
      ValueInitializedBytes<BlockReadRequest>(),
      t::ElementsAreArray(OpcodeAndZeros<BlockReadRequest>(BlockRpcOp::kRead)));
  EXPECT_THAT(ValueInitializedBytes<BlockWriteRequest>(),
              t::ElementsAreArray(
                  OpcodeAndZeros<BlockWriteRequest>(BlockRpcOp::kWrite)));
  EXPECT_THAT(ValueInitializedBytes<BlockFlushRequest>(),
              t::ElementsAreArray(
                  OpcodeAndZeros<BlockFlushRequest>(BlockRpcOp::kFlush)));
  EXPECT_THAT(ValueInitializedBytes<BlockDeviceInfo>(), t::Each(t::Eq(0)));
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
  EXPECT_TRUE(UserProgramNameEquals("usbkbd", 6, "usbkbd"));
  EXPECT_TRUE(UserProgramNameEquals("ps2kbd", 6, "ps2kbd"));
  EXPECT_TRUE(UserProgramNameEquals("nvme", 4, "nvme"));
  // Substring of longer buffer with exact length matches.
  EXPECT_TRUE(UserProgramNameEquals("sh_extra", 2, "sh"));
  EXPECT_TRUE(UserProgramNameEquals("nvme_extra", 4, "nvme"));
  // Mismatched lengths, embedded NULs, or null pointers do not match.
  EXPECT_FALSE(UserProgramNameEquals("sh", 1, "sh"));
  EXPECT_FALSE(UserProgramNameEquals("nvme", 3, "nvme"));
  EXPECT_FALSE(UserProgramNameEquals("sh\0x", 4, "sh"));
  EXPECT_FALSE(UserProgramNameEquals("nvme\0x", 6, "nvme"));
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
    std::vector<std::string> argv;
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
    s->spawns.push_back(SpawnCall{prog, arg, {}});
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

  // Clamping at kShellMaxLineLength (1024, as rootfs/doc/shell.txt says).
  EXPECT_THAT(sh::kShellMaxLineLength, t::Eq(1024));
  editor.ResetLine();
  for (int i = 0; i < sh::kShellMaxLineLength; ++i) {
    ch = 'x';
    EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kEchoChar));
  }
  EXPECT_THAT(editor.length, t::Eq(sh::kShellMaxLineLength));
  EXPECT_FALSE(editor.overflowed);
  // A control character, which the editor ignores anyway, loses nothing.
  ch = '\x1b';
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kNone));
  EXPECT_FALSE(editor.overflowed);
  // A printable character that doesn't fit, also a tab, is dropped without an
  // echo and marks the line overflowed.
  ch = 'y';
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kNone));
  EXPECT_TRUE(editor.overflowed);
  ch = '\t';
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kNone));
  EXPECT_THAT(editor.length, t::Eq(sh::kShellMaxLineLength));
  EXPECT_THAT(std::string(editor.buffer),
              t::Eq(std::string(sh::kShellMaxLineLength, 'x')));
  // Backspace makes room but doesn't bring back what was dropped: the line
  // stays overflowed until ResetLine().
  ch = '\b';
  EXPECT_THAT(editor.FeedChar(&ch),
              t::Eq(sh::ShellCharAction::kBackspaceErase));
  ch = 'z';
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kEchoChar));
  ch = '\n';
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kSubmitLine));
  EXPECT_TRUE(editor.overflowed);
  editor.ResetLine();
  EXPECT_FALSE(editor.overflowed);
  EXPECT_THAT(editor.length, t::Eq(0));
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
                    "built-in commands: help echo cpu programs run exit\n"
                    "built-in file commands: ls cat stat mkdir rm touch write "
                    "append cp mv\n"
                    "user programs: init sh usertest, or files in /bin or by "
                    "path\n"
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
                    "sh: usage: run <program> [arg ...]\n"
                    "sh> usertest badarg\n"
                    "sh: invalid argument: badarg\n"
                    "sh> unknown_cmd\n"
                    "sh: unknown command: unknown_cmd\n"
                    "sh> exit 7\n"));
}

// FakeShellState whose Unlink() records its path in `unlinked` and succeeds.
struct UnlinkRecordingShellState : FakeShellState {
  std::vector<std::string> unlinked;

  UnlinkRecordingShellState() { env.unlink = Unlink; }

  static int64_t Unlink(void* const ctx,         //
                        const char* const path,  //
                        const int64_t path_len) {
    static_cast<UnlinkRecordingShellState*>(static_cast<FakeShellState*>(ctx))
        ->unlinked.emplace_back(path, static_cast<size_t>(path_len));
    return 0;
  }
};

// A line holds at most `kShellMaxLineLength` characters. The shell drops the
// characters that don't fit and refuses the line rather than run what it
// kept, then reads the next line as usual.
TEST(ShellLibTest, RunShellRefusesLinesLongerThanItsLimit) {
  UnlinkRecordingShellState state;
  // `rm /f` padded with spaces to the limit: with more characters, it names
  // another path, such as /fx.
  const std::string rm_f =
      "rm" + std::string(sh::kShellMaxLineLength - 5, ' ') + " /f";
  ASSERT_THAT(rm_f.size(), t::Eq(static_cast<size_t>(sh::kShellMaxLineLength)));
  state.input = rm_f + "x\n" + rm_f + "xy\n" + "echo next\n" + rm_f + "\n";
  EXPECT_THAT(sh::RunShell(state.env), t::Eq(0));
  EXPECT_THAT(state.unlinked, t::ElementsAre("/f"));
  // The shell echoes the characters that it keeps, and no others.
  const std::string refused = "sh> " + rm_f + "\nsh: line too long\n";
  EXPECT_THAT(state.output, t::Eq(refused + refused +
                                  "sh> echo next\n"
                                  "next\n"
                                  "sh> " +
                                  rm_f + "\nsh> "));
}

// Backspace makes room on an overflowed line but doesn't bring back what was
// dropped, so the shell still refuses the line, also at the end of the input.
TEST(ShellLibTest, RunShellRefusesOverflowedLinesAfterBackspaceAndAtEof) {
  // `exit 7` padded with spaces to the limit.
  const std::string exit_7 =
      "exit 7" + std::string(sh::kShellMaxLineLength - 6, ' ');
  ASSERT_THAT(exit_7.size(),
              t::Eq(static_cast<size_t>(sh::kShellMaxLineLength)));
  FakeShellState state;
  state.input = exit_7 + "x\b\n" + "echo next\n" + exit_7 + "x";
  EXPECT_THAT(sh::RunShell(state.env), t::Eq(0));
  // Backspace erases the last character that the shell kept.
  EXPECT_THAT(state.output, t::Eq("sh> " + exit_7 +
                                  "\b \b\n"
                                  "sh: line too long\n"
                                  "sh> echo next\n"
                                  "next\n"
                                  "sh> " +
                                  exit_7 + "\nsh: line too long\n"));
  // Also when Backspace erased all of the line before the input ended.
  FakeShellState erased;
  erased.input = exit_7 + "x" + std::string(sh::kShellMaxLineLength, '\b');
  EXPECT_THAT(sh::RunShell(erased.env), t::Eq(0));
  EXPECT_THAT(erased.output, t::EndsWith("\b \b\nsh: line too long\n"));

  // A line of exactly the limit runs, also at the end of the input.
  FakeShellState at_limit;
  at_limit.input = exit_7;
  EXPECT_THAT(sh::RunShell(at_limit.env), t::Eq(7));
  EXPECT_THAT(at_limit.output, t::Eq("sh> " + exit_7 + "\n"));
}

// FakeShellState whose filesystem callbacks serve `fs` the way the kernel does,
// for client `kClient`: `fds` maps each file descriptor to the server's handle,
// or to 0 if the descriptor isn't open. FileRead(), FileWrite() and ReadDir()
// fail with `file_read_error`, `file_write_error` and `read_dir_error` instead
// if those aren't 0, FileWrite() writes at most `write_limit` bytes per call if
// it isn't 0 (a short write), and `bad_fd_calls` counts the calls with a
// descriptor that isn't open.
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
  int64_t file_write_error = 0;
  int64_t read_dir_error = 0;
  int64_t write_limit = 0;
  int bad_fd_calls = 0;

  RamfsShellHarness() {
    fs->Initialize(&alloc);
    env.spawn_file = SpawnFile;
    env.open = Open;
    env.close = Close;
    env.file_read = FileRead;
    env.file_write = FileWrite;
    env.stat = Stat;
    env.read_dir = ReadDir;
    env.mkdir = MkDir;
    env.unlink = Unlink;
    env.rename = Rename;
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

  // Returns the contents of regular file `path`, or "<error N>" if it can't be
  // read.
  std::string Contents(const std::string& path) {
    FsOpenReply rep{};
    const int64_t open_rc = fs->Open(kClient,                            //
                                     path.data(),                        //
                                     static_cast<int64_t>(path.size()),  //
                                     kFsOpenRead,                        //
                                     &rep);
    if (open_rc < 0) {
      return "<error " + std::to_string(open_rc) + ">";
    }
    std::string contents;
    for (;;) {
      char buf[kSyscallFsMaxIoSize];
      const int64_t n = fs->Read(kClient, rep.handle, buf, sizeof(buf));
      if (n <= 0) {
        if (n < 0) {
          contents = "<error " + std::to_string(n) + ">";
        }
        break;
      }
      contents.append(buf, static_cast<size_t>(n));
    }
    EXPECT_THAT(fs->Close(kClient, rep.handle), t::Eq(0)) << path;
    return contents;
  }

  // Expects that the shell closed every file that it opened, also after
  // errors.
  void ExpectNoOpenFiles() const {
    EXPECT_THAT(fs->ActiveHandleCount(), t::Eq(0));
    EXPECT_THAT(fds, t::Each(t::Eq(0U)));
    EXPECT_THAT(bad_fd_calls, t::Eq(0));
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

  static int64_t FileWrite(void* const ctx,        //
                           const int64_t fd,       //
                           const void* const buf,  //
                           const int64_t size) {
    RamfsShellHarness* const h = Self(ctx);
    const uint64_t handle = h->Handle(fd);
    if (handle == 0) {
      return kSyscallErrorBadFd;
    }
    if (h->file_write_error != 0) {
      return h->file_write_error;
    }
    const int64_t limited =
        (h->write_limit > 0 && size > h->write_limit) ? h->write_limit : size;
    return h->fs->Write(kClient, handle, buf, limited);
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

  static int64_t Rename(void* const ctx,             //
                        const char* const old_path,  //
                        const int64_t old_len,       //
                        const char* const new_path,  //
                        const int64_t new_len) {
    return Self(ctx)->fs->Rename(old_path, old_len, new_path, new_len);
  }

  static int64_t SpawnFile(void* const ctx,                    //
                           const char* const path,             //
                           const int64_t path_len,             //
                           const SyscallArgSlice* const args,  //
                           const int64_t argc) {
    RamfsShellHarness* const h = Self(ctx);
    FsOpenReply rep{};
    const int64_t open_rc =
        h->fs->Open(kClient, path, path_len, kFsOpenRead, &rep);
    if (open_rc < 0) {
      return open_rc;
    }
    if (rep.type == FsNodeType::kDirectory) {
      (void)h->fs->Close(kClient, rep.handle);
      return kSyscallErrorIsDir;
    }
    char magic[4] = {};
    const int64_t n = h->fs->Read(kClient, rep.handle, magic, sizeof(magic));
    (void)h->fs->Close(kClient, rep.handle);
    if (n < 0) {
      return n;
    }
    if (n != 4 || magic[0] != '\x7F' || magic[1] != 'E' || magic[2] != 'L' ||
        magic[3] != 'F') {
      return kSyscallErrorExecFormat;
    }
    std::vector<std::string> argv;
    for (int64_t i = 0; i < argc; ++i) {
      argv.emplace_back(reinterpret_cast<const char*>(args[i].ptr),
                        args[i].len);
    }
    uint64_t sim_arg = static_cast<uint64_t>(argc - 1);
    if (argc >= 2 && argv[1] == "fault") {
      sim_arg = 6;
    } else if (argc >= 2 && argv[1] == "kill") {
      sim_arg = 99;
    }
    h->spawns.push_back(
        SpawnCall{std::string(path, path_len), sim_arg, std::move(argv)});
    return static_cast<int64_t>(h->spawns.size());
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
  EXPECT_THAT(message(kSyscallErrorExecFormat),
              t::Eq("executable format error"));
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
  EXPECT_THAT(no_fs.Run("stat /a"),
              t::Eq("sh: stat: /a: filesystem unavailable\n"));
  EXPECT_THAT(no_fs.Run("mkdir /a"),
              t::Eq("sh: mkdir: /a: filesystem unavailable\n"));
  EXPECT_THAT(no_fs.Run("rm /a"),
              t::Eq("sh: rm: /a: filesystem unavailable\n"));
  EXPECT_THAT(no_fs.Run("touch /a"),
              t::Eq("sh: touch: /a: filesystem unavailable\n"));
  EXPECT_THAT(no_fs.Run("write /a text"),
              t::Eq("sh: write: /a: filesystem unavailable\n"));
  EXPECT_THAT(no_fs.Run("append /a text"),
              t::Eq("sh: append: /a: filesystem unavailable\n"));
  // cp and mv report each source.
  EXPECT_THAT(no_fs.Run("cp /a /b /c"),
              t::Eq("sh: cp: /a: filesystem unavailable\n"
                    "sh: cp: /b: filesystem unavailable\n"));
  EXPECT_THAT(no_fs.Run("mv /a /b"),
              t::Eq("sh: mv: /a: filesystem unavailable\n"));
  EXPECT_THAT(no_fs.Run("/bin/hello"),
              t::Eq("sh: /bin/hello: filesystem unavailable\n"));
  EXPECT_THAT(no_fs.Run("run /bin/hello"),
              t::Eq("sh: run: /bin/hello: filesystem unavailable\n"));
}

// A built-in that lacks one of the callbacks that it needs calls none of the
// others: it changes nothing and opens no file that it then couldn't read,
// write, list or close.
TEST(ShellLibTest, FilesystemBuiltinsNeedEachOfTheirCallbacks) {
  using Remove = void (*)(sh::ShellEnv& env);
  const Remove no_open = [](sh::ShellEnv& env) { env.open = nullptr; };
  const Remove no_close = [](sh::ShellEnv& env) { env.close = nullptr; };
  const Remove no_read = [](sh::ShellEnv& env) { env.file_read = nullptr; };
  const Remove no_write = [](sh::ShellEnv& env) { env.file_write = nullptr; };
  const Remove no_stat = [](sh::ShellEnv& env) { env.stat = nullptr; };
  const Remove no_read_dir = [](sh::ShellEnv& env) { env.read_dir = nullptr; };
  const Remove no_mkdir = [](sh::ShellEnv& env) { env.mkdir = nullptr; };
  const Remove no_unlink = [](sh::ShellEnv& env) { env.unlink = nullptr; };
  const Remove no_rename = [](sh::ShellEnv& env) { env.rename = nullptr; };
  const struct {
    const char* line;
    // The operand that the error names.
    const char* path;
    Remove remove;
  } kCases[] = {
      {"ls /d", "/d", no_stat},
      {"ls /d", "/d", no_open},
      {"ls /d", "/d", no_read_dir},
      {"ls /d", "/d", no_close},
      {"cat /d/f", "/d/f", no_open},
      {"cat /d/f", "/d/f", no_read},
      {"cat /d/f", "/d/f", no_close},
      {"stat /d/f", "/d/f", no_stat},
      {"touch /d/new", "/d/new", no_open},
      {"touch /d/new", "/d/new", no_close},
      {"mkdir /d/new", "/d/new", no_mkdir},
      {"rm /d/f", "/d/f", no_unlink},
      {"write /d/f text", "/d/f", no_open},
      {"write /d/f text", "/d/f", no_write},
      {"write /d/f text", "/d/f", no_close},
      {"append /d/new text", "/d/new", no_open},
      {"append /d/new text", "/d/new", no_write},
      {"append /d/new text", "/d/new", no_close},
      {"cp /d/f /d/new", "/d/f", no_stat},
      {"cp /d/f /d/new", "/d/f", no_open},
      {"cp /d/f /d/new", "/d/f", no_read},
      {"cp /d/f /d/new", "/d/f", no_write},
      {"cp /d/f /d/new", "/d/f", no_close},
      {"mv /d/f /d/new", "/d/f", no_stat},
      {"mv /d/f /d/new", "/d/f", no_rename},
  };
  for (const auto& c : kCases) {
    RamfsShellHarness h;
    h.MakeDir("/d");
    h.WriteFile("/d/f", "data");
    const int fnodes = h.fs->ActiveFnodeCount();
    c.remove(h.env);
    const std::string line = c.line;
    const std::string cmd = line.substr(0, line.find(' '));
    const std::string error =
        "sh: " + cmd + ": " + c.path + ": filesystem unavailable\n";
    EXPECT_THAT(h.Run(line), t::Eq(error)) << line;
    EXPECT_THAT(h.fs->ActiveHandleCount(), t::Eq(0)) << line;
    EXPECT_THAT(h.fs->ActiveFnodeCount(), t::Eq(fnodes)) << line;
    EXPECT_THAT(h.Contents("/d/f"), t::Eq("data")) << line;
  }
}

TEST(ShellLibTest, FilesystemBuiltinsOperateOnRamfsAndReportErrors) {
  RamfsShellHarness h;
  // Longer than two of cat's reads.
  const std::string readme =
      std::string(2 * sh::kShellFileChunkSize + 100, 'A') + "END_OF_README\n";
  h.MakeDir("/docs");
  h.WriteFile("/docs/readme.txt", readme);
  const int fnodes = h.fs->ActiveFnodeCount();

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
  EXPECT_THAT(h.fs->ActiveFnodeCount(), t::Eq(fnodes));
  EXPECT_THAT(h.Run("ls"), t::Eq("docs/\n"));

  h.ExpectNoOpenFiles();
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
  // An error in listing a directory goes in its group.
  h.read_dir_error = kSyscallErrorNoMemory;
  EXPECT_THAT(h.Run("ls /docs/readme.txt /void"),
              t::Eq("/docs/readme.txt\n"
                    "\n"
                    "/void:\n"
                    "sh: ls: /void: out of memory\n"));
  h.read_dir_error = 0;
  EXPECT_THAT(h.fs->ActiveHandleCount(), t::Eq(0));
  EXPECT_THAT(h.bad_fd_calls, t::Eq(0));
}

// A line holds at most `kShellMaxTokens` tokens. Rather than skip operands, the
// filesystem built-ins refuse to run with more.
TEST(ShellLibTest, FilesystemBuiltinsRefuseLinesWithTooManyTokens) {
  RamfsShellHarness h;
  h.MakeDir("/t");
  const int fnodes = h.fs->ActiveFnodeCount();
  // " /t/0 /t/1 ...": as many operands as a command holds, and one more.
  std::string most;
  for (int i = 0; i < sh::kShellMaxTokens - 1; ++i) {
    most += " /t/" + std::to_string(i);
  }
  const std::string too_many = most + " /t/extra";

  for (const char* const cmd : {"ls", "cat", "stat", "mkdir", "rm", "touch",
                                "write", "append", "cp", "mv"}) {
    EXPECT_THAT(h.Run(cmd + too_many), t::Eq("sh: too many arguments\n"))
        << cmd;
  }
  EXPECT_THAT(h.fs->ActiveFnodeCount(), t::Eq(fnodes));

  EXPECT_THAT(h.Run("touch" + most), t::Eq(""));
  EXPECT_THAT(h.fs->ActiveFnodeCount(),
              t::Eq(fnodes + sh::kShellMaxTokens - 1));
  EXPECT_THAT(h.Run("touch /t/extra"), t::Eq(""));
  EXPECT_THAT(h.Run("rm" + too_many), t::Eq("sh: too many arguments\n"));
  EXPECT_THAT(h.fs->ActiveFnodeCount(), t::Eq(fnodes + sh::kShellMaxTokens));
  EXPECT_THAT(h.Run("rm" + most), t::Eq(""));
  EXPECT_THAT(h.Run("ls /t"), t::Eq("extra\n"));

  // echo writes the arguments that the command holds instead.
  EXPECT_THAT(h.Run("echo" + too_many), t::Eq(most.substr(1) + "\n"));
}

// A line has room for paths of `kFsMaxPathLength` bytes, two of them for `cp`,
// and the shell passes them whole to the filesystem.
TEST(ShellLibTest, RunShellTakesLinesWithTheLongestPaths) {
  RamfsShellHarness h;
  const std::string dir = "/" + std::string(kFsMaxNameLength, 'a') + "/" +
                          std::string(kFsMaxNameLength, 'b') + "/" +
                          std::string(kFsMaxNameLength, 'c');
  const std::string src = dir + "/" + std::string(62, 'd');
  const std::string dst = dir + "/" + std::string(62, 'e');
  ASSERT_THAT(static_cast<int64_t>(src.size()), t::Eq(kFsMaxPathLength));
  ASSERT_THAT(static_cast<int64_t>(dst.size()), t::Eq(kFsMaxPathLength));
  h.MakeDir(dir.substr(0, 64));
  h.MakeDir(dir.substr(0, 128));
  h.MakeDir(dir);
  h.WriteFile(src, "long\n");
  h.input = "cat " + src + "\n" + "cp " + src + " " + dst + "\n";
  EXPECT_THAT(sh::RunShell(h.env), t::Eq(0));
  EXPECT_THAT(h.output, t::Eq("sh> cat " + src +
                              "\n"
                              "long\n"
                              "sh> cp " +
                              src + " " + dst + "\nsh> "));
  EXPECT_THAT(h.Contents(dst), t::Eq("long\n"));
  h.ExpectNoOpenFiles();
}

// FakeShellState whose filesystem callbacks claim more than they may: the
// first ReadDir() claims more entries than asked for, with names longer than
// `kFsMaxNameLength` or of negative length, the first FileRead() claims more
// bytes than asked for, and FileWrite(), which appends to `written`, claims
// more bytes than it got (or none if `zero_writes`). `/` is a directory and
// every other path a file, whose fnode ID is its last character.
struct OverclaimingFs : FakeShellState {
  int read_dir_calls = 0;
  int file_read_calls = 0;
  bool zero_writes = false;
  std::string written;

  OverclaimingFs() {
    env.open = Open;
    env.close = Close;
    env.stat = Stat;
    env.read_dir = ReadDir;
    env.file_read = FileRead;
    env.file_write = FileWrite;
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
    out_stat->fnode_id = static_cast<uint8_t>(path[path_len - 1]);
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

  static int64_t FileWrite(void* const ctx,        //
                           const int64_t /*fd*/,   //
                           const void* const buf,  //
                           const int64_t size) {
    OverclaimingFs* const fs = Self(ctx);
    if (fs->zero_writes) {
      return 0;
    }
    fs->written.append(static_cast<const char*>(buf),
                       static_cast<size_t>(size));
    return size + 100;
  }
};

TEST(ShellLibTest, LsCatAndCpClampWhatCallbacksClaimToTheirBuffers) {
  OverclaimingFs fs;
  std::string listing;
  for (int i = 0; i < sh::kShellReadDirBatch; ++i) {
    listing +=
        (i % 2 == 0) ? std::string(kFsMaxNameLength, 'n') : std::string();
    listing += "\n";
  }
  EXPECT_THAT(fs.Run("ls"), t::Eq(listing));
  EXPECT_THAT(fs.Run("cat /f"),
              t::Eq(std::string(sh::kShellFileChunkSize, 'x')));

  OverclaimingFs cp_fs;
  EXPECT_THAT(cp_fs.Run("cp /f /g"), t::Eq(""));
  EXPECT_THAT(cp_fs.written, t::Eq(std::string(sh::kShellFileChunkSize, 'x')));
}

// A FileWrite() that writes nothing fails the command instead of being
// retried forever.
TEST(ShellLibTest, WritesThatWriteNothingFailWithAnIoError) {
  OverclaimingFs fs;
  fs.zero_writes = true;
  EXPECT_THAT(fs.Run("write /f text"), t::Eq("sh: write: /f: io error\n"));
  EXPECT_THAT(fs.Run("append /f text"), t::Eq("sh: append: /f: io error\n"));
  EXPECT_THAT(fs.Run("cp /f /g"), t::Eq("sh: cp: /g: io error\n"));
}

TEST(ShellLibTest, WriteAndAppendReplaceOrExtendAFile) {
  RamfsShellHarness h;
  h.MakeDir("/d");

  EXPECT_THAT(h.Run("write"), t::Eq("sh: usage: write <path> [word ...]\n"));
  EXPECT_THAT(h.Run("append"), t::Eq("sh: usage: append <path> [word ...]\n"));

  // write creates a missing file and writes the words as echo writes them.
  EXPECT_THAT(h.Run("write /d/f  hello   world "), t::Eq(""));
  EXPECT_THAT(h.Contents("/d/f"), t::Eq("hello world\n"));
  EXPECT_THAT(h.Run("append /d/f second line"), t::Eq(""));
  EXPECT_THAT(h.Contents("/d/f"), t::Eq("hello world\nsecond line\n"));
  // write replaces the contents, also with shorter ones.
  EXPECT_THAT(h.Run("write /d/f new"), t::Eq(""));
  EXPECT_THAT(h.Contents("/d/f"), t::Eq("new\n"));
  // Without words, the newline that echo writes.
  EXPECT_THAT(h.Run("write /d/f"), t::Eq(""));
  EXPECT_THAT(h.Contents("/d/f"), t::Eq("\n"));
  // append creates a missing file. '<', '>' and '&' are only characters.
  EXPECT_THAT(h.Run("append /d/new <a> & b>c"), t::Eq(""));
  EXPECT_THAT(h.Contents("/d/new"), t::Eq("<a> & b>c\n"));

  // After a short write, the shell writes the rest.
  h.write_limit = 3;
  EXPECT_THAT(h.Run("write /d/f one two three"), t::Eq(""));
  EXPECT_THAT(h.Contents("/d/f"), t::Eq("one two three\n"));
  h.write_limit = 0;

  // Errors of Open() and FileWrite().
  EXPECT_THAT(h.Run("write /d text"), t::Eq("sh: write: /d: is a directory\n"));
  EXPECT_THAT(h.Run("append /nodir/f text"),
              t::Eq("sh: append: /nodir/f: no such file or directory\n"));
  h.file_write_error = kSyscallErrorNoSpace;
  EXPECT_THAT(h.Run("append /d/f more"),
              t::Eq("sh: append: /d/f: no space left\n"));
  h.file_write_error = 0;
  EXPECT_THAT(h.Contents("/d/f"), t::Eq("one two three\n"));
  EXPECT_THAT(h.TypeOf("/d"), t::Eq(FsNodeType::kDirectory));
  EXPECT_THAT(h.TypeOf("/nodir"), t::Eq(FsNodeType::kNone));
  h.ExpectNoOpenFiles();
}

TEST(ShellLibTest, CpCopiesFilesToPathsOrIntoDirectories) {
  RamfsShellHarness h;
  // Longer than two of cp's reads.
  const std::string big =
      std::string(2 * sh::kShellFileChunkSize + 100, 'B') + "END\n";
  h.MakeDir("/d");
  h.MakeDir("/e");
  h.WriteFile("/f", big);
  h.WriteFile("/g", "gee\n");

  EXPECT_THAT(h.Run("cp"), t::Eq("sh: usage: cp <source> ... <target>\n"));
  EXPECT_THAT(h.Run("cp /f"), t::Eq("sh: usage: cp <source> ... <target>\n"));

  // To a new path, and over a file, whose contents the copy replaces.
  EXPECT_THAT(h.Run("cp /f /copy"), t::Eq(""));
  EXPECT_THAT(h.Contents("/copy"), t::Eq(big));
  EXPECT_THAT(h.Run("cp /g /copy"), t::Eq(""));
  EXPECT_THAT(h.Contents("/copy"), t::Eq("gee\n"));
  // Into a directory, under the source's name. Several sources need one.
  EXPECT_THAT(h.Run("cp /f /d"), t::Eq(""));
  EXPECT_THAT(h.Contents("/d/f"), t::Eq(big));
  EXPECT_THAT(h.Run("cp /f /g /e/"), t::Eq(""));
  EXPECT_THAT(h.Run("ls /e"), t::Eq("f\ng\n"));
  EXPECT_THAT(h.Contents("/e/g"), t::Eq("gee\n"));
  EXPECT_THAT(h.Run("cp /f /g /copy"),
              t::Eq("sh: cp: /copy: not a directory\n"));
  EXPECT_THAT(h.Run("cp /f /g /missing"),
              t::Eq("sh: cp: /missing: no such file or directory\n"));
  EXPECT_THAT(h.TypeOf("/missing"), t::Eq(FsNodeType::kNone));
  EXPECT_THAT(h.Contents("/f"), t::Eq(big));

  // After a short write, the shell writes the rest.
  h.write_limit = 1000;
  EXPECT_THAT(h.Run("cp /f /short"), t::Eq(""));
  EXPECT_THAT(h.Contents("/short"), t::Eq(big));
  h.write_limit = 0;

  // Sources that cp doesn't copy: missing ones, directories, and a file that
  // is the destination too (opening the destination would empty it).
  EXPECT_THAT(h.Run("cp /missing /x /d"),
              t::Eq("sh: cp: /missing: no such file or directory\n"
                    "sh: cp: /x: no such file or directory\n"));
  EXPECT_THAT(h.Run("cp /d /x"), t::Eq("sh: cp: /d: is a directory\n"));
  EXPECT_THAT(h.Run("cp /g /g"),
              t::Eq("sh: cp: /g and /g are the same file\n"));
  EXPECT_THAT(h.Run("cp /g /./g"),
              t::Eq("sh: cp: /g and /./g are the same file\n"));
  EXPECT_THAT(h.Run("cp /g /"), t::Eq("sh: cp: /g and /g are the same file\n"));
  EXPECT_THAT(h.Contents("/g"), t::Eq("gee\n"));
  // Destinations that cp can't write: a directory, a missing directory.
  h.WriteFile("/x", "ex\n");
  h.MakeDir("/e/x");
  EXPECT_THAT(h.Run("cp /x /e"), t::Eq("sh: cp: /e/x: is a directory\n"));
  EXPECT_THAT(h.Run("cp /x /nodir/x"),
              t::Eq("sh: cp: /nodir/x: no such file or directory\n"));
  // Errors of FileRead() and FileWrite().
  h.file_read_error = kSyscallErrorAccess;
  EXPECT_THAT(h.Run("cp /x /y"), t::Eq("sh: cp: /x: error -13\n"));
  h.file_read_error = 0;
  h.file_write_error = kSyscallErrorNoSpace;
  EXPECT_THAT(h.Run("cp /x /z"), t::Eq("sh: cp: /z: no space left\n"));
  h.file_write_error = 0;

  // A destination in a directory whose path is too long for it: 252 bytes
  // and a '/' leave room for names of up to 2 bytes.
  const std::string dir = "/" + std::string(kFsMaxNameLength, 'a') + "/" +
                          std::string(kFsMaxNameLength, 'b') + "/" +
                          std::string(kFsMaxNameLength, 'c') + "/" +
                          std::string(59, 'd');
  ASSERT_THAT(dir.size(), t::Eq(252U));
  h.MakeDir(dir.substr(0, 64));
  h.MakeDir(dir.substr(0, 128));
  h.MakeDir(dir.substr(0, 192));
  h.MakeDir(dir);
  EXPECT_THAT(h.Run("cp /x " + dir), t::Eq(""));
  EXPECT_THAT(h.Contents(dir + "/x"), t::Eq("ex\n"));
  EXPECT_THAT(h.Run("cp /copy " + dir),
              t::Eq("sh: cp: /copy: name too long\n"));
  h.ExpectNoOpenFiles();
}

TEST(ShellLibTest, MvRenamesAndMovesFilesAndDirectories) {
  RamfsShellHarness h;
  h.MakeDir("/d");
  h.MakeDir("/e");
  h.WriteFile("/f", "eff\n");
  h.WriteFile("/g", "gee\n");
  h.WriteFile("/h", "aitch\n");
  const int fnodes = h.fs->ActiveFnodeCount();

  EXPECT_THAT(h.Run("mv"), t::Eq("sh: usage: mv <source> ... <target>\n"));
  EXPECT_THAT(h.Run("mv /f"), t::Eq("sh: usage: mv <source> ... <target>\n"));

  // Renames a file, and replaces a file.
  EXPECT_THAT(h.Run("mv /f /f2"), t::Eq(""));
  EXPECT_THAT(h.TypeOf("/f"), t::Eq(FsNodeType::kNone));
  EXPECT_THAT(h.Contents("/f2"), t::Eq("eff\n"));
  EXPECT_THAT(h.Run("mv /f2 /g"), t::Eq(""));
  EXPECT_THAT(h.Contents("/g"), t::Eq("eff\n"));
  EXPECT_THAT(h.fs->ActiveFnodeCount(), t::Eq(fnodes - 1));
  // Moves files and directories into a directory, under their names.
  EXPECT_THAT(h.Run("mv /g /d"), t::Eq(""));
  EXPECT_THAT(h.Run("mv /d/g /h /e"), t::Eq(""));
  EXPECT_THAT(h.Run("ls /d /e"), t::Eq("/d:\n\n/e:\ng\nh\n"));
  EXPECT_THAT(h.Run("mv /e /d/"), t::Eq(""));
  EXPECT_THAT(h.Run("ls /d/e"), t::Eq("g\nh\n"));
  // Renames a directory. A source that is its own destination stays.
  EXPECT_THAT(h.Run("mv /d/e /moved"), t::Eq(""));
  EXPECT_THAT(h.Contents("/moved/h"), t::Eq("aitch\n"));
  EXPECT_THAT(h.Run("mv /moved/h /moved/h"), t::Eq(""));
  EXPECT_THAT(h.Contents("/moved/h"), t::Eq("aitch\n"));
  EXPECT_THAT(h.fs->ActiveFnodeCount(), t::Eq(fnodes - 1));

  // Errors name the destination, except those about the source: it doesn't
  // exist, it is the root, its last component is `.` or `..` (as is then that
  // of its destination in a directory), or it is a directory that would move
  // into itself or a subdirectory.
  EXPECT_THAT(h.Run("mv /missing /x"),
              t::Eq("sh: mv: /missing: no such file or directory\n"));
  EXPECT_THAT(h.Run("mv / /d"), t::Eq("sh: mv: /: device or resource busy\n"));
  EXPECT_THAT(h.Run("mv /. /moved/.. /d"),
              t::Eq("sh: mv: /.: invalid argument\n"
                    "sh: mv: /moved/..: invalid argument\n"));
  EXPECT_THAT(h.Run("mv /moved /moved"),
              t::Eq("sh: mv: /moved: invalid argument\n"));
  EXPECT_THAT(h.Run("mv /moved/h /nodir/h"),
              t::Eq("sh: mv: /nodir/h: no such file or directory\n"));
  EXPECT_THAT(h.Run("mv /moved/h /h/"),
              t::Eq("sh: mv: /h/: not a directory\n"));
  EXPECT_THAT(h.Run("mv /moved/g /moved/h /moved/g"),
              t::Eq("sh: mv: /moved/g: not a directory\n"));
  h.MakeDir("/y");
  h.MakeDir("/y/sub");
  EXPECT_THAT(h.Run("mv /y /y/sub"), t::Eq("sh: mv: /y: invalid argument\n"));
  h.WriteFile("/y/sub/file", "");
  h.MakeDir("/sub");
  EXPECT_THAT(h.Run("mv /sub /y"),
              t::Eq("sh: mv: /y/sub: directory not empty\n"));
  h.MakeDir("/y/g");
  EXPECT_THAT(h.Run("mv /moved/g /y"), t::Eq("sh: mv: /y/g: is a directory\n"));
  EXPECT_THAT(h.Contents("/moved/g"), t::Eq("eff\n"));
  EXPECT_THAT(h.TypeOf("/sub"), t::Eq(FsNodeType::kDirectory));
  EXPECT_THAT(h.TypeOf("/y"), t::Eq(FsNodeType::kDirectory));
  h.ExpectNoOpenFiles();

  // A filesystem that checks the last components of both paths before it
  // looks them up fails because of a destination whose last component is `.`
  // or `..`.
  FakeShellState dots_first;
  dots_first.env.stat = [](void* const /*ctx*/,     //
                           const char* const path,  //
                           const int64_t path_len,  //
                           FsStat* const out_stat) -> int64_t {
    *out_stat = FsStat{};
    out_stat->type = FsNodeType::kRegular;
    return (std::string(path, static_cast<size_t>(path_len)) == "/f")
               ? 0
               : kSyscallErrorNoEntry;
  };
  dots_first.env.rename = [](void* const /*ctx*/,             //
                             const char* const /*old_path*/,  //
                             const int64_t /*old_len*/,       //
                             const char* const /*new_path*/,  //
                             const int64_t /*new_len*/) -> int64_t {
    return kSyscallErrorInvalid;
  };
  EXPECT_THAT(dots_first.Run("mv /f /nodir/.."),
              t::Eq("sh: mv: /nodir/..: invalid argument\n"));
}

TEST(ShellLibTest, StatDescribesFilesAndDirectories) {
  RamfsShellHarness h;
  // The root has fnode ID 1, and the others follow in creation order.
  h.MakeDir("/d");
  h.WriteFile("/d/f", "hello\n");

  EXPECT_THAT(h.Run("stat"), t::Eq("sh: usage: stat <path> ...\n"));
  // A directory's size is that of its entries (`sizeof(FsDirEntry)` each).
  EXPECT_THAT(h.Run("stat /d/f /d / /missing"),
              t::Eq("  File: /d/f\n"
                    "  Type: regular file\n"
                    "  Size: 6\n"
                    " Fnode: 3\n"
                    " Links: 1\n"
                    "  File: /d\n"
                    "  Type: directory\n"
                    "  Size: 80\n"
                    " Fnode: 2\n"
                    " Links: 2\n"
                    "  File: /\n"
                    "  Type: directory\n"
                    "  Size: 80\n"
                    " Fnode: 1\n"
                    " Links: 3\n"
                    "sh: stat: /missing: no such file or directory\n"));

  // A `kFsMaxPathLength` (255-byte) path does not truncate the trailing fields.
  const std::string deep_dir = "/" + std::string(kFsMaxNameLength, 'a') + "/" +
                               std::string(kFsMaxNameLength, 'b') + "/" +
                               std::string(kFsMaxNameLength, 'c');
  const std::string max_path = deep_dir + "/" + std::string(62, 'd');
  ASSERT_THAT(static_cast<int64_t>(max_path.size()), t::Eq(kFsMaxPathLength));
  h.MakeDir(deep_dir.substr(0, 64));
  h.MakeDir(deep_dir.substr(0, 128));
  h.MakeDir(deep_dir);
  h.WriteFile(max_path, "0123456789");
  EXPECT_THAT(h.Run("stat " + max_path), t::Eq("  File: " + max_path +
                                               "\n"
                                               "  Type: regular file\n"
                                               "  Size: 10\n"
                                               " Fnode: 7\n"
                                               " Links: 1\n"));

  // A type that the shell doesn't know.
  FakeShellState odd;
  odd.env.stat = [](void* const /*ctx*/,         //
                    const char* const /*path*/,  //
                    const int64_t /*path_len*/,  //
                    FsStat* const out_stat) -> int64_t {
    *out_stat = FsStat{};
    out_stat->type = static_cast<FsNodeType>(7);
    return 0;
  };
  EXPECT_THAT(odd.Run("stat /x"), t::Eq("  File: /x\n"
                                        "  Type: unknown\n"
                                        "  Size: 0\n"
                                        " Fnode: 0\n"
                                        " Links: 0\n"));
}

TEST(ShellLibTest, RunsProgramsFromPathsAndBinAndReportsErrors) {
  RamfsShellHarness h;
  h.MakeDir("/bin");
  h.MakeDir("/bin/subdir");
  h.WriteFile("/bin/hello", std::string("\x7F"
                                        "ELFbinary",
                                        10));
  h.WriteFile("/bin/not_elf", "plain text\n");
  h.WriteFile("/README.txt", "readme\n");

  // `programs` lists embedded programs and regular files in /bin (ignoring
  // subdirectories).
  EXPECT_THAT(h.Run("programs"), t::Eq("programs: init sh usertest\n"
                                       "/bin: hello not_elf\n"));

  // `programs` flushes `/bin:` in pieces so arbitrarily many long names across
  // multiple ReadDir() batches all appear with a closing newline.
  {
    RamfsShellHarness many;
    many.MakeDir("/bin");
    std::string expected = "programs: init sh usertest\n/bin:";
    // 25 names of 50 bytes each = 1275 bytes in `/bin:`, exceeding both
    // `kShellReadDirBatch` (8) and `kShellOutputBufferSize` (1088).
    for (int i = 0; i < 25; ++i) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "prog_%02d_", i);
      std::string name(buf);
      name.append(50 - name.size(), 'a');
      many.WriteFile("/bin/" + name,
                     "\x7F"
                     "ELF");
      expected += " ";
      expected += name;
    }
    expected += "\n";
    EXPECT_THAT(many.Run("programs"), t::Eq(expected));
    many.ExpectNoOpenFiles();
  }

  // Bare name from /bin, direct path, relative path, and `run` (both bare and
  // path), forwarding arbitrary `argv` words, faults and kills.
  EXPECT_THAT(h.Run("hello"), t::Eq("sh: hello exited with status 0\n"));
  EXPECT_THAT(h.spawns.back().argv, t::ElementsAre("hello"));

  EXPECT_THAT(h.Run("hello world from protos"),
              t::Eq("sh: hello exited with status 3\n"));
  EXPECT_THAT(h.spawns.back().argv,
              t::ElementsAre("hello", "world", "from", "protos"));

  EXPECT_THAT(h.Run("/bin/hello alpha beta"),
              t::Eq("sh: /bin/hello exited with status 2\n"));
  EXPECT_THAT(h.spawns.back().argv,
              t::ElementsAre("/bin/hello", "alpha", "beta"));

  EXPECT_THAT(h.Run("bin/hello fault"),
              t::Eq("sh: bin/hello faulted vector=6 error=0x0\n"));
  EXPECT_THAT(h.Run("run hello kill"), t::Eq("sh: hello killed\n"));
  EXPECT_THAT(h.Run("run /bin/hello one two"),
              t::Eq("sh: /bin/hello exited with status 2\n"));
  EXPECT_THAT(h.spawns.back().argv, t::ElementsAre("/bin/hello", "one", "two"));

  // Copying a binary to another path and running the copy with string words.
  EXPECT_THAT(h.Run("cp /bin/hello /copy"), t::Eq(""));
  EXPECT_THAT(h.Run("/copy arg1"), t::Eq("sh: /copy exited with status 1\n"));
  EXPECT_THAT(h.spawns.back().argv, t::ElementsAre("/copy", "arg1"));

  // Embedded program validation vs truncated lines (> 16 tokens) vs unknown
  // commands.
  EXPECT_THAT(h.Run("run usertest bad"), t::Eq("sh: invalid argument: bad\n"));
  std::string sixteen_args;
  for (int i = 0; i < sh::kShellMaxTokens; ++i) {
    sixteen_args += " a";
  }
  EXPECT_THAT(h.Run("hello" + sixteen_args), t::Eq("sh: too many arguments\n"));
  EXPECT_THAT(h.Run("/bin/hello" + sixteen_args),
              t::Eq("sh: too many arguments\n"));
  EXPECT_THAT(h.Run("run hello" + sixteen_args),
              t::Eq("sh: too many arguments\n"));
  EXPECT_THAT(h.Run("run /bin/hello" + sixteen_args),
              t::Eq("sh: too many arguments\n"));
  EXPECT_THAT(h.Run("unknown_cmd"),
              t::Eq("sh: unknown command: unknown_cmd\n"));
  EXPECT_THAT(h.Run("unknown_cmd a b c"),
              t::Eq("sh: unknown command: unknown_cmd\n"));
  EXPECT_THAT(h.Run("unknown_cmd" + sixteen_args),
              t::Eq("sh: unknown command: unknown_cmd\n"));
  EXPECT_THAT(h.Run("run nope"), t::Eq("sh: unknown program: nope\n"));

  // Non-ELF files, directories, and missing paths.
  EXPECT_THAT(h.Run("/README.txt"),
              t::Eq("sh: /README.txt: executable format error\n"));
  EXPECT_THAT(h.Run("run /README.txt"),
              t::Eq("sh: run: /README.txt: executable format error\n"));
  EXPECT_THAT(h.Run("not_elf"),
              t::Eq("sh: /bin/not_elf: executable format error\n"));
  EXPECT_THAT(h.Run("run not_elf"),
              t::Eq("sh: run: /bin/not_elf: executable format error\n"));
  EXPECT_THAT(h.Run("/bin"), t::Eq("sh: /bin: is a directory\n"));
  EXPECT_THAT(h.Run("subdir"), t::Eq("sh: /bin/subdir: is a directory\n"));
  EXPECT_THAT(h.Run("run subdir"),
              t::Eq("sh: run: /bin/subdir: is a directory\n"));
  EXPECT_THAT(h.Run("/missing"),
              t::Eq("sh: /missing: no such file or directory\n"));
  EXPECT_THAT(h.Run("run /missing"),
              t::Eq("sh: run: /missing: no such file or directory\n"));

  h.ExpectNoOpenFiles();
}

TEST(SyscallAbiTest, UserSegmentRangeAllowedRejectsReservedWindows) {
  // Valid placements: bottom of user space, right below each reserved window,
  // right above each reserved window, and the linked top-2-GiB ELF region
  // (including unaligned segment virtual addresses whose pages stay outside
  // reserved windows).
  EXPECT_TRUE(UserSegmentRangeAllowed(kUserSpaceBase, 1));
  EXPECT_TRUE(UserSegmentRangeAllowed(kUserSpaceBase + 0x1A0, 0x200));
  EXPECT_TRUE(UserSegmentRangeAllowed(kDriverMmioWindowBase - kPageSize, 1));
  EXPECT_TRUE(UserSegmentRangeAllowed(kDriverMmioWindowBase - 1, 1));
  EXPECT_TRUE(UserSegmentRangeAllowed(
      kDriverMmioWindowBase + kDriverMmioWindowSize, kPageSize));
  EXPECT_TRUE(UserSegmentRangeAllowed(
      kDriverMmioWindowBase + kDriverMmioWindowSize + 0x100, 0x200));
  EXPECT_TRUE(UserSegmentRangeAllowed(kDriverDmaWindowBase - kPageSize, 1));
  EXPECT_TRUE(UserSegmentRangeAllowed(
      kDriverDmaWindowBase + kDriverDmaWindowSize, kPageSize));
  EXPECT_TRUE(UserSegmentRangeAllowed(kUserAnonWindowBase - kPageSize, 1));
  EXPECT_TRUE(
      UserSegmentRangeAllowed(kUserAnonWindowBase + kUserAnonWindowSize, 1));
  EXPECT_TRUE(
      UserSegmentRangeAllowed(kUserStackRegionBase - kPageSize, kPageSize));
  EXPECT_TRUE(UserSegmentRangeAllowed(0xFFFFFFFF80000000ULL, kPageSize));
  EXPECT_TRUE(UserSegmentRangeAllowed(0xFFFFFFFF80000001ULL, kPageSize));
  EXPECT_TRUE(UserSegmentRangeAllowed(kUserSpaceEnd - kPageSize, 1));

  // Empty, negative, or out-of-user-space segments.
  EXPECT_FALSE(UserSegmentRangeAllowed(0xFFFFFFFF80000000ULL, 0));
  EXPECT_FALSE(UserSegmentRangeAllowed(0xFFFFFFFF80000000ULL, -kPageSize));
  EXPECT_FALSE(UserSegmentRangeAllowed(kUserSpaceEnd, kPageSize));
  EXPECT_FALSE(
      UserSegmentRangeAllowed(kUserSpaceEnd - kPageSize, kPageSize + 1));

  // Overlapping MMIO, DMA, anonymous `MemMap()`, or stack + guard pages
  // (including unaligned segments whose rounded pages touch a reserved window).
  EXPECT_FALSE(UserSegmentRangeAllowed(kDriverMmioWindowBase - kPageSize,
                                       kPageSize + 1));
  EXPECT_FALSE(UserSegmentRangeAllowed(kDriverMmioWindowBase - 1, 2));
  EXPECT_FALSE(UserSegmentRangeAllowed(kDriverMmioWindowBase, 1));
  EXPECT_FALSE(UserSegmentRangeAllowed(
      kDriverMmioWindowBase + kDriverMmioWindowSize - 1, 0x200));
  EXPECT_FALSE(UserSegmentRangeAllowed(
      kDriverMmioWindowBase + kDriverMmioWindowSize - kPageSize, 1));
  EXPECT_FALSE(UserSegmentRangeAllowed(kDriverDmaWindowBase, 1));
  EXPECT_FALSE(UserSegmentRangeAllowed(kUserAnonWindowBase, 1));
  EXPECT_FALSE(UserSegmentRangeAllowed(
      kUserAnonWindowBase + kUserAnonWindowSize - kPageSize, 1));
  EXPECT_FALSE(
      UserSegmentRangeAllowed(kUserStackRegionBase - kPageSize, kPageSize + 1));
  EXPECT_FALSE(UserSegmentRangeAllowed(kUserStackRegionBase, 1));
  EXPECT_FALSE(UserSegmentRangeAllowed(kUserStackTop - kPageSize, 1));
  EXPECT_FALSE(UserSegmentRangeAllowed(kUserStackTop, 1));
}

TEST(ShellLibTest, RunDispatchesNvmeThroughEmbeddedProgramBranch) {
  struct NvmeSpawnShellHarness : RamfsShellHarness {
    NvmeSpawnShellHarness() { env.spawn = SpawnWithNvme; }

    static int64_t SpawnWithNvme(void* const ctx,         //
                                 const char* const name,  //
                                 const int64_t name_len,  //
                                 const uint64_t arg) {
      FakeShellState* const s = static_cast<FakeShellState*>(ctx);
      const std::string prog(name, static_cast<size_t>(name_len));
      if (prog == "nvme") {
        s->spawns.push_back(SpawnCall{prog, arg, {}});
        return static_cast<int64_t>(s->spawns.size());
      }
      return FakeShellState::Spawn(ctx, name, name_len, arg);
    }
  };

  NvmeSpawnShellHarness h;
  h.MakeDir("/bin");
  h.WriteFile("/bin/nvme", std::string("\x7F"
                                       "ELFbinary",
                                       10));

  // `run nvme` uses the embedded-program branch (`env.spawn`) rather than
  // `/bin/nvme`, and validates its optional integer argument.
  EXPECT_THAT(h.Run("run nvme"), t::Eq("sh: nvme exited with status 0\n"));
  ASSERT_FALSE(h.spawns.empty());
  EXPECT_THAT(h.spawns.back().name, t::Eq("nvme"));
  EXPECT_THAT(h.spawns.back().arg, t::Eq(0ULL));
  EXPECT_TRUE(h.spawns.back().argv.empty());

  EXPECT_THAT(h.Run("run nvme 1"), t::Eq("sh: nvme exited with status 1\n"));
  EXPECT_THAT(h.spawns.back().name, t::Eq("nvme"));
  EXPECT_THAT(h.spawns.back().arg, t::Eq(1ULL));

  const size_t spawns_before_bad = h.spawns.size();
  EXPECT_THAT(h.Run("run nvme badarg"),
              t::Eq("sh: invalid argument: badarg\n"));
  EXPECT_THAT(h.Run("run nvme 1 2"), t::Eq("sh: too many arguments\n"));
  EXPECT_THAT(h.spawns.size(), t::Eq(spawns_before_bad));

  // Bare `nvme` is not listed in `IsKnownUserProgram`, so when `/bin/nvme`
  // exists it runs `/bin/nvme` via `env.spawn_file`.
  EXPECT_THAT(h.Run("nvme"), t::Eq("sh: nvme exited with status 0\n"));
  EXPECT_THAT(h.spawns.back().name, t::Eq("/bin/nvme"));

  // Without embedded `"nvme"` spawn support (or without `/bin/nvme`), standard
  // `FakeShellState` reports `unknown program` / `unknown command`.
  FakeShellState plain;
  EXPECT_THAT(plain.Run("run nvme"), t::Eq("sh: unknown program: nvme\n"));
  EXPECT_THAT(plain.Run("nvme"), t::Eq("sh: unknown command: nvme\n"));
  h.ExpectNoOpenFiles();
}

struct FakeBlockClient {
  int calls = 0;
  int64_t last_port = -1;
  std::vector<uint8_t> last_req;
  void* last_reply_buf = nullptr;
  int64_t last_reply_cap = -1;
  int64_t next_rc = 0;
  std::vector<uint8_t> next_reply;
  std::optional<int64_t> override_reply_size;

  user::BlockClientEnv Env() {
    return user::BlockClientEnv{
        .ctx = this,
        .ipc_call = &IpcCall,
    };
  }

  static int64_t IpcCall(void* const ctx,            //
                         const int64_t port,         //
                         const void* const req_buf,  //
                         const int64_t req_size,     //
                         void* const reply_buf,      //
                         const int64_t reply_cap,    //
                         int64_t* const out_reply_size) {
    FakeBlockClient* const self = static_cast<FakeBlockClient*>(ctx);
    ++self->calls;
    self->last_port = port;
    self->last_reply_buf = reply_buf;
    self->last_reply_cap = reply_cap;
    self->last_req.resize(static_cast<size_t>(req_size));
    if (req_size > 0 && req_buf != nullptr) {
      __builtin_memcpy(self->last_req.data(),  //
                       req_buf,                //
                       static_cast<size_t>(req_size));
    }
    const int64_t reported_size =
        self->override_reply_size.has_value()
            ? *self->override_reply_size
            : static_cast<int64_t>(self->next_reply.size());
    if (self->next_rc >= 0 && reply_buf != nullptr && reply_cap > 0 &&
        !self->next_reply.empty()) {
      const int64_t to_copy =
          (static_cast<int64_t>(self->next_reply.size()) < reply_cap)
              ? static_cast<int64_t>(self->next_reply.size())
              : reply_cap;
      __builtin_memcpy(reply_buf,                //
                       self->next_reply.data(),  //
                       static_cast<size_t>(to_copy));
    }
    if (out_reply_size != nullptr) {
      *out_reply_size = reported_size;
    }
    return self->next_rc;
  }
};

static BlockDeviceInfo MakeValidBlockInfo(const int32_t block_size = 512,
                                          const int32_t max_transfer_blocks = 8,
                                          const uint64_t block_count = 1024,
                                          const uint8_t read_only = 0) {
  BlockDeviceInfo info{};
  info.block_count = block_count;
  info.block_size = block_size;
  info.max_transfer_blocks = max_transfer_blocks;
  info.bus = 0;
  info.device = 3;
  info.function = 1;
  info.read_only = read_only;
  info.nsid = 1;
  __builtin_memcpy(info.model, "PROTOS NVMe", 11);
  __builtin_memcpy(info.serial, "PROTOS-DISK-0", 13);
  return info;
}

template <typename T>
static std::vector<uint8_t> ToBytes(const T& value) {
  const uint8_t* const p = reinterpret_cast<const uint8_t*>(&value);
  return std::vector<uint8_t>(p, p + sizeof(value));
}

TEST(SyscallAbiTest, BlockProtocolHelpersValidateGeometryAndStrings) {
  EXPECT_THAT(user::kBlockWriteScratchBytes, t::Eq(4120));

  EXPECT_TRUE(user::IsValidBlockSize(512));
  EXPECT_TRUE(user::IsValidBlockSize(1024));
  EXPECT_TRUE(user::IsValidBlockSize(2048));
  EXPECT_TRUE(user::IsValidBlockSize(4096));
  EXPECT_FALSE(user::IsValidBlockSize(INT32_MIN));
  EXPECT_FALSE(user::IsValidBlockSize(-512));
  EXPECT_FALSE(user::IsValidBlockSize(0));
  EXPECT_FALSE(user::IsValidBlockSize(256));
  EXPECT_FALSE(user::IsValidBlockSize(511));
  EXPECT_FALSE(user::IsValidBlockSize(513));
  EXPECT_FALSE(user::IsValidBlockSize(1536));
  EXPECT_FALSE(user::IsValidBlockSize(4097));
  EXPECT_FALSE(user::IsValidBlockSize(8192));
  EXPECT_FALSE(user::IsValidBlockSize(INT32_MAX));

  EXPECT_FALSE(user::IsNulTerminated(nullptr, 4));
  EXPECT_FALSE(user::IsNulTerminated(nullptr, 1));
  EXPECT_FALSE(user::IsNulTerminated("", 0));
  EXPECT_FALSE(user::IsNulTerminated("", -1));
  const char empty[4] = {};
  EXPECT_TRUE(user::IsNulTerminated(empty, 4));
  EXPECT_TRUE(user::IsNulTerminated("", 1));
  EXPECT_FALSE(user::IsNulTerminated("A", 1));
  const char early[4] = {'\0', 'A', 'B', 'C'};
  EXPECT_TRUE(user::IsNulTerminated(early, 4));
  // Inspect exact 7 bytes so `buf[cap - 1] == 'Z' != '\0'`.
  const char early_tail[7] = {'a', 'b', 'c', '\0', 'X', 'Y', 'Z'};
  EXPECT_TRUE(user::IsNulTerminated(early_tail, 7));
  char full_model[kBlockModelCapacity];
  __builtin_memset(full_model, 'M', sizeof(full_model));
  EXPECT_FALSE(user::IsNulTerminated(full_model, kBlockModelCapacity));
  full_model[kBlockModelCapacity - 1] = '\0';
  EXPECT_TRUE(user::IsNulTerminated(full_model, kBlockModelCapacity));
  char full_serial[kBlockSerialCapacity];
  __builtin_memset(full_serial, 'S', sizeof(full_serial));
  EXPECT_FALSE(user::IsNulTerminated(full_serial, kBlockSerialCapacity));
  full_serial[kBlockSerialCapacity - 1] = '\0';
  EXPECT_TRUE(user::IsNulTerminated(full_serial, kBlockSerialCapacity));
  char high_bytes[4];
  __builtin_memset(high_bytes, 0x80, sizeof(high_bytes));
  high_bytes[1] = static_cast<char>(0x81);
  high_bytes[2] = static_cast<char>(0xFE);
  high_bytes[3] = static_cast<char>(0xFF);
  EXPECT_FALSE(user::IsNulTerminated(high_bytes, 4));
  high_bytes[2] = '\0';
  EXPECT_TRUE(user::IsNulTerminated(high_bytes, 4));

  EXPECT_TRUE(user::IsValidBlockDeviceInfo(MakeValidBlockInfo(512, 8, 1, 0)));
  EXPECT_TRUE(user::IsValidBlockDeviceInfo(MakeValidBlockInfo(1024, 4, 1, 0)));
  EXPECT_TRUE(user::IsValidBlockDeviceInfo(MakeValidBlockInfo(2048, 2, 1, 1)));
  EXPECT_TRUE(
      user::IsValidBlockDeviceInfo(MakeValidBlockInfo(4096, 1, UINT64_MAX, 1)));
  EXPECT_TRUE(user::IsValidBlockDeviceInfo(
      MakeValidBlockInfo(512, 8, uint64_t{1} << 32, 0)));
  {
    BlockDeviceInfo early_nul = MakeValidBlockInfo();
    __builtin_memset(early_nul.model, 'X', sizeof(early_nul.model));
    early_nul.model[0] = '\0';
    __builtin_memset(early_nul.serial, 'Y', sizeof(early_nul.serial));
    early_nul.serial[3] = '\0';
    EXPECT_TRUE(user::IsValidBlockDeviceInfo(early_nul));

    __builtin_memset(early_nul.model, 'X', sizeof(early_nul.model));
    early_nul.model[3] = '\0';
    __builtin_memset(early_nul.serial, 'Y', sizeof(early_nul.serial));
    early_nul.serial[0] = '\0';
    EXPECT_TRUE(user::IsValidBlockDeviceInfo(early_nul));
  }

  EXPECT_FALSE(user::IsValidBlockDeviceInfo(BlockDeviceInfo{}));
  {
    BlockDeviceInfo bad = MakeValidBlockInfo();
    bad.block_count = 0;
    EXPECT_FALSE(user::IsValidBlockDeviceInfo(bad));
  }
  {
    BlockDeviceInfo bad = MakeValidBlockInfo();
    bad.block_size = 0;
    EXPECT_FALSE(user::IsValidBlockDeviceInfo(bad));
    bad.block_size = 256;
    EXPECT_FALSE(user::IsValidBlockDeviceInfo(bad));
  }
  {
    BlockDeviceInfo bad = MakeValidBlockInfo();
    bad.max_transfer_blocks = 0;
    EXPECT_FALSE(user::IsValidBlockDeviceInfo(bad));
    bad.max_transfer_blocks = -1;
    EXPECT_FALSE(user::IsValidBlockDeviceInfo(bad));
    bad.max_transfer_blocks = 9;
    EXPECT_FALSE(user::IsValidBlockDeviceInfo(bad));
    bad.max_transfer_blocks = INT32_MAX;
    EXPECT_FALSE(user::IsValidBlockDeviceInfo(bad));
    bad = MakeValidBlockInfo(1024, 5);
    EXPECT_FALSE(user::IsValidBlockDeviceInfo(bad));
    bad = MakeValidBlockInfo(2048, 3);
    EXPECT_FALSE(user::IsValidBlockDeviceInfo(bad));
    bad = MakeValidBlockInfo(4096, 2);
    EXPECT_FALSE(user::IsValidBlockDeviceInfo(bad));
    bad = MakeValidBlockInfo();
    bad.block_size = -512;
    bad.max_transfer_blocks = -1;
    EXPECT_FALSE(user::IsValidBlockDeviceInfo(bad));
  }
  {
    BlockDeviceInfo bad = MakeValidBlockInfo();
    bad.read_only = 2;
    EXPECT_FALSE(user::IsValidBlockDeviceInfo(bad));
    bad.read_only = 255;
    EXPECT_FALSE(user::IsValidBlockDeviceInfo(bad));
  }
  {
    BlockDeviceInfo bad = MakeValidBlockInfo();
    __builtin_memset(bad.model, 'A', sizeof(bad.model));
    EXPECT_FALSE(user::IsValidBlockDeviceInfo(bad));
    __builtin_memset(bad.model, 0x80, sizeof(bad.model));
    EXPECT_FALSE(user::IsValidBlockDeviceInfo(bad));
  }
  {
    BlockDeviceInfo bad = MakeValidBlockInfo();
    __builtin_memset(bad.serial, 'B', sizeof(bad.serial));
    EXPECT_FALSE(user::IsValidBlockDeviceInfo(bad));
    __builtin_memset(bad.serial, 0xFF, sizeof(bad.serial));
    EXPECT_FALSE(user::IsValidBlockDeviceInfo(bad));
  }
}

TEST(SyscallAbiTest, BlockClientGetDeviceInfoValidatesServerReplies) {
  {
    const user::BlockClientEnv null_env{};
    int64_t disk_count = 123;
    const StatusOr<BlockDeviceInfo> res =
        user::GetBlockDeviceInfo(null_env, 0, &disk_count);
    EXPECT_EQ(res.status(), kSyscallErrorInvalid);
    EXPECT_EQ(disk_count, 123);
  }

  // Valid query returns metadata and populates `*out_disk_count`.
  {
    FakeBlockClient mock;
    BlockDeviceInfo expected = MakeValidBlockInfo(4096, 1, 2048, 1);
    __builtin_memset(expected.model, 'X', kBlockModelCapacity - 1);
    expected.model[kBlockModelCapacity - 1] = '\0';
    __builtin_memset(expected.serial, 'Y', kBlockSerialCapacity - 1);
    expected.serial[kBlockSerialCapacity - 1] = '\0';
    mock.next_rc = 2;
    mock.next_reply = ToBytes(expected);

    int64_t disk_count = -1;
    const StatusOr<BlockDeviceInfo> res =
        user::GetBlockDeviceInfo(mock.Env(), 1, &disk_count);
    ASSERT_TRUE(res.ok());
    EXPECT_EQ(disk_count, 2);
    EXPECT_EQ(mock.calls, 1);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    EXPECT_EQ(mock.last_reply_cap,
              static_cast<int64_t>(sizeof(BlockDeviceInfo)));
    const BlockGetInfoRequest expected_req{.op = BlockRpcOp::kGetInfo,
                                           .disk_index = 1};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));
    EXPECT_THAT(ToBytes(*res), t::ElementsAreArray(ToBytes(expected)));
  }

  // Boundary `max_transfer_blocks` values across all valid sector sizes pass
  // through `GetBlockDeviceInfo()`.
  {
    struct ValidBound {
      int32_t bs;
      int32_t max_blocks;
    };
    static constexpr ValidBound kValidBounds[] = {{1024, 4}, {2048, 2}};
    for (const ValidBound& bound : kValidBounds) {
      FakeBlockClient mock;
      const BlockDeviceInfo info =
          MakeValidBlockInfo(bound.bs, bound.max_blocks);
      mock.next_rc = 1;
      mock.next_reply = ToBytes(info);
      EXPECT_TRUE(user::GetBlockDeviceInfo(mock.Env(), 0).ok());
    }
  }

  // Early NUL with non-NUL trailing bytes in `model` and `serial` succeeds.
  {
    FakeBlockClient mock;
    BlockDeviceInfo expected = MakeValidBlockInfo();
    __builtin_memset(expected.model, 'X', sizeof(expected.model));
    expected.model[0] = '\0';
    __builtin_memset(expected.serial, 'Y', sizeof(expected.serial));
    expected.serial[3] = '\0';
    mock.next_rc = 1;
    mock.next_reply = ToBytes(expected);

    const StatusOr<BlockDeviceInfo> res =
        user::GetBlockDeviceInfo(mock.Env(), 0);
    ASSERT_TRUE(res.ok());
    EXPECT_THAT(ToBytes(*res), t::ElementsAreArray(ToBytes(expected)));

    __builtin_memset(expected.model, 'X', sizeof(expected.model));
    expected.model[3] = '\0';
    __builtin_memset(expected.serial, 'Y', sizeof(expected.serial));
    expected.serial[0] = '\0';
    mock.next_reply = ToBytes(expected);
    const StatusOr<BlockDeviceInfo> swapped =
        user::GetBlockDeviceInfo(mock.Env(), 0);
    ASSERT_TRUE(swapped.ok());
    EXPECT_THAT(ToBytes(*swapped), t::ElementsAreArray(ToBytes(expected)));
  }

  // Negative server errors propagate without touching `*out_disk_count`.
  {
    FakeBlockClient mock;
    mock.next_rc = kSyscallErrorNoDevice;
    int64_t disk_count = 77;
    const StatusOr<BlockDeviceInfo> res =
        user::GetBlockDeviceInfo(mock.Env(), 0, &disk_count);
    EXPECT_EQ(res.status(), kSyscallErrorNoDevice);
    EXPECT_EQ(disk_count, 77);

    mock.next_rc = kSyscallErrorPermission;
    mock.next_reply = ToBytes(MakeValidBlockInfo());
    const StatusOr<BlockDeviceInfo> perm_res =
        user::GetBlockDeviceInfo(mock.Env(), 0, &disk_count);
    EXPECT_EQ(perm_res.status(), kSyscallErrorPermission);
    EXPECT_EQ(disk_count, 77);
  }

  // Malformed status / index / reply_size / payload all fail with Io.
  const BlockDeviceInfo valid = MakeValidBlockInfo();
  {
    FakeBlockClient mock;
    mock.next_rc = 0;
    mock.next_reply = ToBytes(valid);
    int64_t disk_count = 55;
    EXPECT_EQ(user::GetBlockDeviceInfo(mock.Env(), 0, &disk_count).status(),
              kSyscallErrorIo);
    EXPECT_EQ(disk_count, 55);
  }
  {
    FakeBlockClient mock;
    mock.next_rc = 1;
    mock.next_reply = ToBytes(valid);
    int64_t disk_count = 44;
    EXPECT_EQ(user::GetBlockDeviceInfo(mock.Env(), 1, &disk_count).status(),
              kSyscallErrorIo);
    EXPECT_EQ(disk_count, 44);
    EXPECT_EQ(user::GetBlockDeviceInfo(mock.Env(), UINT32_MAX).status(),
              kSyscallErrorIo);

    mock.next_rc = 2;
    EXPECT_EQ(user::GetBlockDeviceInfo(mock.Env(), 2, &disk_count).status(),
              kSyscallErrorIo);
    EXPECT_EQ(user::GetBlockDeviceInfo(mock.Env(), 256u, &disk_count).status(),
              kSyscallErrorIo);
    EXPECT_EQ(
        user::GetBlockDeviceInfo(mock.Env(), 65536u, &disk_count).status(),
        kSyscallErrorIo);
    EXPECT_EQ(disk_count, 44);
  }
  {
    // 0x100000000LL must not truncate to 0 in uint32_t comparison or
    // `*out_disk_count`, and 64-bit `block_count` / 32-bit `disk_index`
    // must not truncate.
    FakeBlockClient mock;
    const BlockDeviceInfo large_disk =
        MakeValidBlockInfo(512, 8, uint64_t{1} << 32, 0);
    mock.next_rc = int64_t{1} << 32;
    mock.next_reply = ToBytes(large_disk);
    int64_t disk_count = 0;
    const StatusOr<BlockDeviceInfo> first =
        user::GetBlockDeviceInfo(mock.Env(), 0, &disk_count);
    ASSERT_TRUE(first.ok());
    EXPECT_EQ(first->block_count, uint64_t{1} << 32);
    EXPECT_EQ(disk_count, int64_t{1} << 32);

    const uint32_t wide_disk_index = 0x12345678u;
    EXPECT_TRUE(user::GetBlockDeviceInfo(mock.Env(), wide_disk_index).ok());
    const BlockGetInfoRequest expected_req{.op = BlockRpcOp::kGetInfo,
                                           .disk_index = wide_disk_index};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));
    EXPECT_TRUE(user::GetBlockDeviceInfo(mock.Env(), UINT32_MAX).ok());

    mock.next_rc = int64_t{1} << 31;
    disk_count = 0;
    const StatusOr<BlockDeviceInfo> bit31 =
        user::GetBlockDeviceInfo(mock.Env(), 0, &disk_count);
    ASSERT_TRUE(bit31.ok());
    EXPECT_EQ(bit31->block_count, uint64_t{1} << 32);
    EXPECT_EQ(disk_count, int64_t{1} << 31);
  }
  {
    FakeBlockClient mock;
    mock.next_rc = 1;
    mock.next_reply = ToBytes(valid);
    mock.override_reply_size = 0;
    EXPECT_EQ(user::GetBlockDeviceInfo(mock.Env(), 0).status(),
              kSyscallErrorIo);
    mock.override_reply_size =
        static_cast<int64_t>(sizeof(BlockDeviceInfo)) - 1;
    EXPECT_EQ(user::GetBlockDeviceInfo(mock.Env(), 0).status(),
              kSyscallErrorIo);
    mock.override_reply_size =
        static_cast<int64_t>(sizeof(BlockDeviceInfo)) + 1;
    EXPECT_EQ(user::GetBlockDeviceInfo(mock.Env(), 0).status(),
              kSyscallErrorIo);
    mock.override_reply_size = -1;
    int64_t disk_count = 33;
    EXPECT_EQ(user::GetBlockDeviceInfo(mock.Env(), 0, &disk_count).status(),
              kSyscallErrorIo);
    EXPECT_EQ(disk_count, 33);
    mock.override_reply_size =
        (int64_t{1} << 32) + static_cast<int64_t>(sizeof(BlockDeviceInfo));
    EXPECT_EQ(user::GetBlockDeviceInfo(mock.Env(), 0, &disk_count).status(),
              kSyscallErrorIo);
    EXPECT_EQ(disk_count, 33);
    mock.override_reply_size =
        -(int64_t{1} << 32) + static_cast<int64_t>(sizeof(BlockDeviceInfo));
    EXPECT_EQ(user::GetBlockDeviceInfo(mock.Env(), 0, &disk_count).status(),
              kSyscallErrorIo);
    EXPECT_EQ(disk_count, 33);
    mock.override_reply_size.reset();
  }

  const auto expect_invalid_payload = [](const BlockDeviceInfo& bad) {
    FakeBlockClient mock;
    mock.next_rc = 1;
    mock.next_reply = ToBytes(bad);
    int64_t disk_count = 99;
    EXPECT_EQ(user::GetBlockDeviceInfo(mock.Env(), 0, &disk_count).status(),
              kSyscallErrorIo);
    EXPECT_EQ(disk_count, 99);
  };
  expect_invalid_payload(BlockDeviceInfo{});
  {
    BlockDeviceInfo bad = valid;
    bad.block_count = 0;
    expect_invalid_payload(bad);
  }
  {
    BlockDeviceInfo bad = valid;
    bad.block_size = 0;
    expect_invalid_payload(bad);
    bad.block_size = 513;
    expect_invalid_payload(bad);
  }
  {
    BlockDeviceInfo bad = valid;
    bad.max_transfer_blocks = 9;
    expect_invalid_payload(bad);
  }
  expect_invalid_payload(MakeValidBlockInfo(1024, 5));
  expect_invalid_payload(MakeValidBlockInfo(2048, 3));
  expect_invalid_payload(MakeValidBlockInfo(4096, 2));
  {
    BlockDeviceInfo bad = valid;
    bad.block_size = -512;
    bad.max_transfer_blocks = -1;
    expect_invalid_payload(bad);
  }
  {
    BlockDeviceInfo bad = valid;
    bad.read_only = 2;
    expect_invalid_payload(bad);
    bad.read_only = 255;
    expect_invalid_payload(bad);
  }
  {
    BlockDeviceInfo bad = valid;
    __builtin_memset(bad.model, 'Z', sizeof(bad.model));
    expect_invalid_payload(bad);
    __builtin_memset(bad.model, 0x80, sizeof(bad.model));
    expect_invalid_payload(bad);
  }
  {
    BlockDeviceInfo bad = valid;
    __builtin_memset(bad.serial, 'W', sizeof(bad.serial));
    expect_invalid_payload(bad);
    __builtin_memset(bad.serial, 0xFF, sizeof(bad.serial));
    expect_invalid_payload(bad);
  }
}

TEST(SyscallAbiTest, BlockClientReadBlocksValidatesArgsZeroBlockAndReplies) {
  uint8_t buf[kBlockMaxIoBytes] = {};
  {
    const user::BlockClientEnv null_env{};
    EXPECT_EQ(user::ReadBlocks(null_env, 0, 0, 1, buf, 512),
              kSyscallErrorInvalid);
  }
  {
    FakeBlockClient mock;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, -1, buf, 512),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, -1),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, kBlockMaxIoBytes + 1),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, 0),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, nullptr, 512),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 0, nullptr, 512),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, nullptr, 1),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 0, nullptr, 1),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 0, nullptr, -1),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 0, buf, kBlockMaxIoBytes + 1),
              kSyscallErrorInvalid);
    EXPECT_EQ(
        user::ReadBlocks(mock.Env(), 0, 0, 1, buf, (int64_t{1} << 32) + 512),
        kSyscallErrorInvalid);
    EXPECT_EQ(
        user::ReadBlocks(mock.Env(), 0, 0, 0, buf, (int64_t{1} << 32) + 512),
        kSyscallErrorInvalid);
    EXPECT_EQ(
        user::ReadBlocks(mock.Env(), 0, 0, 1, buf, -(int64_t{1} << 32) + 512),
        kSyscallErrorInvalid);
    EXPECT_EQ(
        user::ReadBlocks(mock.Env(), 0, 0, 0, buf, -(int64_t{1} << 32) + 512),
        kSyscallErrorInvalid);
    EXPECT_EQ(mock.calls, 0);
  }

  // Zero-block read passes reply_buf=nullptr, reply_cap=0 even with a valid
  // buffer so a server cannot overwrite caller memory.
  {
    FakeBlockClient mock;
    uint8_t sentinel[512];
    __builtin_memset(sentinel, 0xA5, sizeof(sentinel));
    mock.next_rc = 0;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 4, 0, nullptr, 0), 0);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    EXPECT_EQ(mock.last_reply_buf, nullptr);
    EXPECT_EQ(mock.last_reply_cap, 0);
    const BlockReadRequest expected_req{.op = BlockRpcOp::kRead,
                                        .disk_index = 0,
                                        .lba = 4,
                                        .block_count = 0,
                                        .reserved = 0};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));

    mock.next_reply.assign(64, 0x11);
    mock.override_reply_size = 0;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 4, 0, sentinel, 0), 0);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    EXPECT_EQ(mock.last_reply_buf, nullptr);
    EXPECT_EQ(mock.last_reply_cap, 0);
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));
    EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));
    mock.override_reply_size.reset();

    mock.next_reply.assign(64, 0x11);
    mock.override_reply_size = 0;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 4, 0, sentinel, sizeof(sentinel)),
              0);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    EXPECT_EQ(mock.last_reply_buf, nullptr);
    EXPECT_EQ(mock.last_reply_cap, 0);
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));
    EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));

    const uint32_t wide_disk_index = 0x12345678u;
    const uint64_t wide_lba = 0x123456789ABCDEF0ULL;
    EXPECT_EQ(user::ReadBlocks(mock.Env(),       //
                               wide_disk_index,  //
                               wide_lba,         //
                               0,                //
                               sentinel,         //
                               sizeof(sentinel)),
              0);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    EXPECT_EQ(mock.last_reply_buf, nullptr);
    EXPECT_EQ(mock.last_reply_cap, 0);
    const BlockReadRequest wide_req{.op = BlockRpcOp::kRead,
                                    .disk_index = wide_disk_index,
                                    .lba = wide_lba,
                                    .block_count = 0,
                                    .reserved = 0};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(wide_req)));
    EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));

    const uint64_t high_lba = uint64_t{1} << 63;
    EXPECT_EQ(user::ReadBlocks(mock.Env(),       //
                               wide_disk_index,  //
                               high_lba,         //
                               0,                //
                               sentinel,         //
                               sizeof(sentinel)),
              0);
    const BlockReadRequest high_req{.op = BlockRpcOp::kRead,
                                    .disk_index = wide_disk_index,
                                    .lba = high_lba,
                                    .block_count = 0,
                                    .reserved = 0};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(high_req)));
    EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));

    mock.override_reply_size = 16;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 4, 0, sentinel, sizeof(sentinel)),
              kSyscallErrorIo);
    mock.override_reply_size = 1;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 4, 0, sentinel, sizeof(sentinel)),
              kSyscallErrorIo);
    mock.override_reply_size = -1;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 4, 0, sentinel, sizeof(sentinel)),
              kSyscallErrorIo);
    mock.override_reply_size = int64_t{1} << 32;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 4, 0, sentinel, sizeof(sentinel)),
              kSyscallErrorIo);
    mock.override_reply_size = -(int64_t{1} << 32);
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 4, 0, sentinel, sizeof(sentinel)),
              kSyscallErrorIo);
    mock.override_reply_size = 0;
    mock.next_rc = 1;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 4, 0, sentinel, sizeof(sentinel)),
              kSyscallErrorIo);
    mock.next_rc = int64_t{1} << 32;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 4, 0, sentinel, sizeof(sentinel)),
              kSyscallErrorIo);
    mock.override_reply_size.reset();
    mock.next_rc = kSyscallErrorPermission;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 4, 0, sentinel, sizeof(sentinel)),
              kSyscallErrorPermission);
    mock.next_rc = kSyscallErrorIo;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 4, 0, sentinel, sizeof(sentinel)),
              kSyscallErrorIo);
    EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));
  }

  // Valid reads across valid sector sizes (even when byte_cap is larger).
  for (const int32_t bs : {512, 1024, 2048, 4096}) {
    FakeBlockClient mock;
    mock.next_rc = 1;
    mock.next_reply.assign(static_cast<size_t>(bs), 0x3C);
    __builtin_memset(buf, 0, sizeof(buf));
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 7, 1, buf, sizeof(buf)), 1);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    EXPECT_EQ(mock.last_reply_buf, buf);
    EXPECT_EQ(mock.last_reply_cap, static_cast<int64_t>(sizeof(buf)));
    const BlockReadRequest expected_req{.op = BlockRpcOp::kRead,
                                        .disk_index = 0,
                                        .lba = 7,
                                        .block_count = 1,
                                        .reserved = 0};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));
    EXPECT_EQ(buf[0], 0x3C);
    EXPECT_EQ(buf[bs - 1], 0x3C);
  }

  // `reply_cap` passed to IPC must match caller `byte_cap` (`512`, `1024`),
  // never `kBlockMaxIoBytes`.
  for (const int64_t cap : {int64_t{512}, int64_t{1024}}) {
    FakeBlockClient mock;
    mock.next_rc = 1;
    mock.next_reply.assign(512, 0x3C);
    __builtin_memset(buf, 0xA5, sizeof(buf));
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 7, 1, buf, cap), 1);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    EXPECT_EQ(mock.last_reply_buf, buf);
    EXPECT_EQ(mock.last_reply_cap, cap);
    EXPECT_EQ(buf[0], 0x3C);
    EXPECT_EQ(buf[511], 0x3C);
    EXPECT_EQ(buf[512], 0xA5);
  }

  // Multi-block read: 3 x 512 B = 1536 B exercises `reply_size / block_count`
  // (1536 itself is not a valid block size), and 4 x 1024 B = 4096 B checks
  // full-buffer transfer.
  {
    FakeBlockClient mock;
    mock.next_rc = 3;
    mock.next_reply.resize(1536);
    for (int i = 0; i < 1536; ++i) {
      mock.next_reply[static_cast<size_t>(i)] =
          static_cast<uint8_t>((i * 19 + 3) & 0xFF);
    }
    __builtin_memset(buf, 0, sizeof(buf));
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 1, 12, 3, buf, sizeof(buf)), 3);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    const BlockReadRequest expected_req{.op = BlockRpcOp::kRead,
                                        .disk_index = 1,
                                        .lba = 12,
                                        .block_count = 3,
                                        .reserved = 0};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));
    EXPECT_EQ(__builtin_memcmp(buf, mock.next_reply.data(), 1536), 0);
  }
  {
    FakeBlockClient mock;
    mock.next_rc = 4;
    mock.next_reply.assign(4096, 0x6B);
    __builtin_memset(buf, 0, sizeof(buf));
    const uint32_t disk_index = 0x12345678u;
    const uint64_t lba = 0x123456789ABCDEF0ULL;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), disk_index, lba, 4, buf, 4096), 4);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    EXPECT_EQ(mock.last_reply_buf, buf);
    EXPECT_EQ(mock.last_reply_cap, 4096);
    const BlockReadRequest expected_req{.op = BlockRpcOp::kRead,
                                        .disk_index = disk_index,
                                        .lba = lba,
                                        .block_count = 4,
                                        .reserved = 0};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));
    EXPECT_EQ(__builtin_memcmp(buf, mock.next_reply.data(), 4096), 0);
  }
  {
    FakeBlockClient mock;
    mock.next_rc = 1;
    mock.next_reply.assign(512, 0x7E);
    __builtin_memset(buf, 0, sizeof(buf));
    const uint32_t msb_disk = 0x80000001u;
    const uint64_t high_lba = 0xFEDCBA9876543210ULL;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), msb_disk, high_lba, 1, buf, 512), 1);
    const BlockReadRequest expected_req{.op = BlockRpcOp::kRead,
                                        .disk_index = msb_disk,
                                        .lba = high_lba,
                                        .block_count = 1,
                                        .reserved = 0};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));
    EXPECT_EQ(buf[0], 0x7E);
    EXPECT_EQ(buf[511], 0x7E);
  }

  // Sub-sector byte_cap and >8 block_count still reach the server so
  // server-side rejection tests keep working, while malformed positive replies
  // are caught.
  {
    FakeBlockClient mock;
    uint8_t sentinel[512];
    __builtin_memset(sentinel, 0xA5, sizeof(sentinel));
    mock.next_rc = kSyscallErrorInvalid;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, sentinel, 511),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 9, buf, kBlockMaxIoBytes),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocks(mock.Env(),  //
                               0,           //
                               0,           //
                               0x1020304,   //
                               sentinel,    //
                               sizeof(sentinel)),
              kSyscallErrorInvalid);
    const BlockReadRequest expected_wide_cnt_req{.op = BlockRpcOp::kRead,
                                                 .disk_index = 0,
                                                 .lba = 0,
                                                 .block_count = 0x1020304,
                                                 .reserved = 0};
    EXPECT_THAT(mock.last_req,
                t::ElementsAreArray(ToBytes(expected_wide_cnt_req)));
    mock.next_rc = kSyscallErrorNoDevice;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, sentinel, sizeof(sentinel)),
              kSyscallErrorNoDevice);
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 0, nullptr, 0),
              kSyscallErrorNoDevice);
    mock.next_rc = kSyscallErrorIo;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, sentinel, sizeof(sentinel)),
              kSyscallErrorIo);
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 0, nullptr, 0),
              kSyscallErrorIo);
    mock.next_reply.assign(sizeof(sentinel), 0x11);
    mock.next_rc = kSyscallErrorPermission;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, sentinel, sizeof(sentinel)),
              kSyscallErrorPermission);
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 0, nullptr, 0),
              kSyscallErrorPermission);
    EXPECT_EQ(mock.calls, 9);
    EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));
  }
  {
    FakeBlockClient mock;
    mock.next_rc = 2;  // mismatch with block_count=1
    mock.next_reply.assign(512, 0x11);
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, 4096),
              kSyscallErrorIo);
    mock.next_rc = 0;  // mismatch (< block_count)
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, 4096),
              kSyscallErrorIo);
    mock.next_rc = (int64_t{1} << 32) + 1;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, 4096),
              kSyscallErrorIo);
    mock.next_rc = int64_t{1} << 31;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, 512), kSyscallErrorIo);

    mock.next_rc = 1;
    mock.next_reply.clear();
    __builtin_memset(buf, 0xA5, sizeof(buf));
    mock.override_reply_size = 0;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, 4096),
              kSyscallErrorIo);
    EXPECT_THAT(buf, t::Each(t::Eq(0xA5)));
    mock.override_reply_size = 513;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, 512), kSyscallErrorIo);
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, 4096),
              kSyscallErrorIo);
    mock.override_reply_size = 511;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, 4096),
              kSyscallErrorIo);
    {
      uint8_t cap_sentinel[511];
      __builtin_memset(cap_sentinel, 0xA5, sizeof(cap_sentinel));
      mock.next_rc = 1;
      mock.next_reply.clear();
      mock.override_reply_size = 512;
      EXPECT_EQ(user::ReadBlocks(mock.Env(),    //
                                 0,             //
                                 0,             //
                                 1,             //
                                 cap_sentinel,  //
                                 sizeof(cap_sentinel)),
                kSyscallErrorIo);
      EXPECT_THAT(cap_sentinel, t::Each(t::Eq(0xA5)));
    }
    mock.override_reply_size = 256;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, 4096),
              kSyscallErrorIo);
    mock.override_reply_size = 1536;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, 4096),
              kSyscallErrorIo);
    EXPECT_THAT(buf, t::Each(t::Eq(0xA5)));

    // Valid sector size (1024) exceeding caller `byte_cap` (512) must fail
    // solely on `reply_size > byte_cap` (`COV-02`).
    mock.override_reply_size = 1024;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, 512), kSyscallErrorIo);
    mock.next_rc = 2;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 2, buf, 512), kSyscallErrorIo);

    // Negative `reply_size` must fail with `kSyscallErrorIo`, including values
    // whose lower 32 bits match a valid sector size.
    mock.next_rc = 1;
    mock.override_reply_size = -1;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, 512), kSyscallErrorIo);
    mock.override_reply_size = -512;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, 512), kSyscallErrorIo);
    mock.override_reply_size = -(int64_t{1} << 32) + 512;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, 512), kSyscallErrorIo);
    mock.override_reply_size = (int64_t{1} << 32) + 512;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1, buf, 512), kSyscallErrorIo);
    mock.next_rc = 1 << 20;
    mock.override_reply_size = int64_t{1} << 32;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 1 << 20, buf, 512),
              kSyscallErrorIo);
    EXPECT_THAT(buf, t::Each(t::Eq(0xA5)));

    // Non-divisible total and block_count=9 with 4096 reply fail post-reply,
    // including totals whose floored quotient equals a valid sector size.
    mock.next_rc = 3;
    mock.override_reply_size = 1024;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 3, buf, 4096),
              kSyscallErrorIo);
    mock.override_reply_size = 1537;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 3, buf, 4096),
              kSyscallErrorIo);
    mock.override_reply_size = 3073;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 3, buf, 4096),
              kSyscallErrorIo);
    mock.next_rc = 2;
    mock.override_reply_size = 1025;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 2, buf, 4096),
              kSyscallErrorIo);
    mock.next_rc = 9;
    mock.override_reply_size = 4096;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 9, buf, 4096),
              kSyscallErrorIo);
    EXPECT_THAT(buf, t::Each(t::Eq(0xA5)));

    // 2 blocks returning 3072 total bytes yields invalid non-pow2 1536 B/block,
    // and 2 blocks returning 512 total bytes yields invalid 256 B/block.
    mock.next_rc = 2;
    mock.override_reply_size = 3072;
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 2, buf, 4096),
              kSyscallErrorIo);
    EXPECT_THAT(buf, t::Each(t::Eq(0xA5)));
    mock.override_reply_size.reset();
    mock.next_reply.assign(512, 0x22);
    EXPECT_EQ(user::ReadBlocks(mock.Env(), 0, 0, 2, buf, 4096),
              kSyscallErrorIo);
  }
}

TEST(SyscallAbiTest, BlockClientReadBlocksCheckedEnforcesExpectedBlockSize) {
  uint8_t buf[kBlockMaxIoBytes] = {};
  {
    const user::BlockClientEnv null_env{};
    EXPECT_EQ(user::ReadBlocksChecked(null_env, 0, 0, 1, 512, buf, 512),
              kSyscallErrorInvalid);
  }
  {
    FakeBlockClient mock;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 0, 256, nullptr, 0),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 0, 513, nullptr, 0),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 1, 1536, buf, 4096),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 0, 1536, nullptr, 0),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 0, 8192, nullptr, 0),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, -1, 512, buf, 512),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 0, 512, nullptr, -1),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 1, 512, buf, -1),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 0, 512, nullptr, 512),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 1, 512, buf, 0),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 1, 4096, buf, 512),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 2, 4096, buf, 4096),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 1, 512, buf, 511),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 0, 512, nullptr, 1),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      0,           //
                                      1 << 20,     //
                                      4096,        //
                                      buf,         //
                                      kBlockMaxIoBytes),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      0,           //
                                      INT32_MAX,   //
                                      4096,        //
                                      buf,         //
                                      kBlockMaxIoBytes),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 1, 512, nullptr, 512),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      0,           //
                                      1,           //
                                      512,         //
                                      buf,         //
                                      kBlockMaxIoBytes + 1),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      0,           //
                                      0,           //
                                      512,         //
                                      buf,         //
                                      kBlockMaxIoBytes + 1),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      0,           //
                                      1,           //
                                      512,         //
                                      buf,         //
                                      (int64_t{1} << 32) + 512),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      0,           //
                                      0,           //
                                      512,         //
                                      buf,         //
                                      (int64_t{1} << 32) + 512),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      0,           //
                                      1,           //
                                      512,         //
                                      buf,         //
                                      -(int64_t{1} << 32) + 512),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      0,           //
                                      0,           //
                                      512,         //
                                      buf,         //
                                      -(int64_t{1} << 32) + 512),
              kSyscallErrorInvalid);
    EXPECT_EQ(mock.calls, 0);
  }
  {
    FakeBlockClient mock;
    uint8_t sentinel[512];
    __builtin_memset(sentinel, 0xA5, sizeof(sentinel));
    mock.next_rc = 0;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 1, 0, 4096, nullptr, 0),
              0);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    EXPECT_EQ(mock.last_reply_buf, nullptr);
    EXPECT_EQ(mock.last_reply_cap, 0);
    const BlockReadRequest expected_req{.op = BlockRpcOp::kRead,
                                        .disk_index = 0,
                                        .lba = 1,
                                        .block_count = 0,
                                        .reserved = 0};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));

    mock.next_reply.assign(sizeof(sentinel), 0x11);
    mock.override_reply_size = 0;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 1, 0, 4096, sentinel, 0),
              0);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    EXPECT_EQ(mock.last_reply_buf, nullptr);
    EXPECT_EQ(mock.last_reply_cap, 0);
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));
    EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));
    mock.override_reply_size.reset();

    // Non-null buffer on a 0-block read still masks `reply_buf`/`reply_cap`
    // so server replies cannot touch caller memory.
    mock.next_reply.assign(sizeof(sentinel), 0x11);
    mock.override_reply_size = 0;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      1,           //
                                      0,           //
                                      4096,        //
                                      sentinel,    //
                                      sizeof(sentinel)),
              0);
    EXPECT_EQ(mock.last_reply_buf, nullptr);
    EXPECT_EQ(mock.last_reply_cap, 0);
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));
    EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));

    const uint32_t wide_disk_index = 0x12345678u;
    const uint64_t wide_lba = 0x123456789ABCDEF0ULL;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),       //
                                      wide_disk_index,  //
                                      wide_lba,         //
                                      0,                //
                                      4096,             //
                                      sentinel,         //
                                      sizeof(sentinel)),
              0);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    EXPECT_EQ(mock.last_reply_buf, nullptr);
    EXPECT_EQ(mock.last_reply_cap, 0);
    const BlockReadRequest wide_req{.op = BlockRpcOp::kRead,
                                    .disk_index = wide_disk_index,
                                    .lba = wide_lba,
                                    .block_count = 0,
                                    .reserved = 0};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(wide_req)));
    EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));

    const uint64_t high_lba = uint64_t{1} << 63;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),       //
                                      wide_disk_index,  //
                                      high_lba,         //
                                      0,                //
                                      4096,             //
                                      sentinel,         //
                                      sizeof(sentinel)),
              0);
    const BlockReadRequest high_req{.op = BlockRpcOp::kRead,
                                    .disk_index = wide_disk_index,
                                    .lba = high_lba,
                                    .block_count = 0,
                                    .reserved = 0};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(high_req)));
    EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));

    mock.override_reply_size = 512;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      1,           //
                                      0,           //
                                      4096,        //
                                      sentinel,    //
                                      sizeof(sentinel)),
              kSyscallErrorIo);
    mock.override_reply_size = 1;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      1,           //
                                      0,           //
                                      4096,        //
                                      sentinel,    //
                                      sizeof(sentinel)),
              kSyscallErrorIo);
    mock.override_reply_size = -1;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      1,           //
                                      0,           //
                                      4096,        //
                                      sentinel,    //
                                      sizeof(sentinel)),
              kSyscallErrorIo);
    mock.override_reply_size = int64_t{1} << 32;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      1,           //
                                      0,           //
                                      4096,        //
                                      sentinel,    //
                                      sizeof(sentinel)),
              kSyscallErrorIo);
    mock.override_reply_size = -(int64_t{1} << 32);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      1,           //
                                      0,           //
                                      4096,        //
                                      sentinel,    //
                                      sizeof(sentinel)),
              kSyscallErrorIo);
    mock.override_reply_size = 0;
    mock.next_rc = 1;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      1,           //
                                      0,           //
                                      4096,        //
                                      sentinel,    //
                                      sizeof(sentinel)),
              kSyscallErrorIo);
    mock.next_rc = int64_t{1} << 32;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      1,           //
                                      0,           //
                                      4096,        //
                                      sentinel,    //
                                      sizeof(sentinel)),
              kSyscallErrorIo);
    mock.override_reply_size.reset();
    mock.next_rc = kSyscallErrorPermission;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      1,           //
                                      0,           //
                                      4096,        //
                                      sentinel,    //
                                      sizeof(sentinel)),
              kSyscallErrorPermission);
    mock.next_rc = kSyscallErrorIo;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      1,           //
                                      0,           //
                                      4096,        //
                                      sentinel,    //
                                      sizeof(sentinel)),
              kSyscallErrorIo);
    EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));
  }
  {
    FakeBlockClient mock;
    mock.next_rc = 1;
    mock.next_reply.assign(4096, 0x5A);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 2, 1, 4096, buf, 4096), 1);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    EXPECT_EQ(mock.last_reply_buf, buf);
    EXPECT_EQ(mock.last_reply_cap, 4096);

    mock.next_reply.assign(2048, 0x5A);
    __builtin_memset(buf, 0xA5, sizeof(buf));
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 2, 1, 2048, buf, 4096), 1);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    EXPECT_EQ(mock.last_reply_buf, buf);
    EXPECT_EQ(mock.last_reply_cap, 4096);
    EXPECT_EQ(buf[0], 0x5A);
    EXPECT_EQ(buf[2047], 0x5A);
    EXPECT_EQ(buf[2048], 0xA5);

    // `reply_cap` must match caller `byte_cap` (`512`, `1024`, `4096`), never
    // hardcoded `kBlockMaxIoBytes` or `expected_bytes` (`512`).
    mock.next_reply.assign(512, 0x5A);
    __builtin_memset(buf, 0xA5, sizeof(buf));
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 2, 1, 512, buf, 512), 1);
    EXPECT_EQ(mock.last_reply_buf, buf);
    EXPECT_EQ(mock.last_reply_cap, 512);
    EXPECT_EQ(buf[0], 0x5A);
    EXPECT_EQ(buf[511], 0x5A);
    EXPECT_EQ(buf[512], 0xA5);

    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 2, 1, 512, buf, 1024), 1);
    EXPECT_EQ(mock.last_reply_buf, buf);
    EXPECT_EQ(mock.last_reply_cap, 1024);

    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 2, 1, 512, buf, 4096), 1);
    EXPECT_EQ(mock.last_reply_buf, buf);
    EXPECT_EQ(mock.last_reply_cap, 4096);

    // Multi-block > 512 B happy path (4 x 1024 B = 4096 B) with framing & byte
    // verification.
    mock.next_rc = 4;
    mock.next_reply.resize(4096);
    for (int i = 0; i < 4096; ++i) {
      mock.next_reply[static_cast<size_t>(i)] =
          static_cast<uint8_t>((i * 17 + 5) & 0xFF);
    }
    __builtin_memset(buf, 0, sizeof(buf));
    const uint32_t disk_index = 0x12345678u;
    const uint64_t lba = 0x123456789ABCDEF0ULL;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      disk_index,  //
                                      lba,         //
                                      4,           //
                                      1024,        //
                                      buf,         //
                                      4096),
              4);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    EXPECT_EQ(mock.last_reply_buf, buf);
    EXPECT_EQ(mock.last_reply_cap, 4096);
    const BlockReadRequest expected_req{.op = BlockRpcOp::kRead,
                                        .disk_index = disk_index,
                                        .lba = lba,
                                        .block_count = 4,
                                        .reserved = 0};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));
    EXPECT_EQ(__builtin_memcmp(buf, mock.next_reply.data(), 4096), 0);

    mock.next_rc = 1;
    mock.next_reply.assign(512, 0x7E);
    __builtin_memset(buf, 0, sizeof(buf));
    const uint32_t msb_disk = 0x80000001u;
    const uint64_t high_lba = 0xFEDCBA9876543210ULL;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      msb_disk,    //
                                      high_lba,    //
                                      1,           //
                                      512,         //
                                      buf,         //
                                      512),
              1);
    const BlockReadRequest high_req{.op = BlockRpcOp::kRead,
                                    .disk_index = msb_disk,
                                    .lba = high_lba,
                                    .block_count = 1,
                                    .reserved = 0};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(high_req)));
    EXPECT_EQ(buf[0], 0x7E);
    EXPECT_EQ(buf[511], 0x7E);

    // Catches 512 B short-read when caller expected 4096 B, and 4096 B
    // over-read when caller expected 512 B.
    mock.next_rc = 1;
    mock.next_reply.assign(512, 0x5A);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 2, 1, 4096, buf, 4096),
              kSyscallErrorIo);
    mock.next_reply.assign(4096, 0x5A);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 2, 1, 512, buf, 4096),
              kSyscallErrorIo);

    // `rc != block_count` must fail even when `reply_size == expected_bytes`.
    mock.next_reply.assign(512, 0x5A);
    mock.next_rc = 2;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 2, 1, 512, buf, 512),
              kSyscallErrorIo);
    mock.next_rc = 0;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 2, 1, 512, buf, 512),
              kSyscallErrorIo);
    mock.next_reply.assign(2048, 0x5A);
    mock.next_rc = 1;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 2, 2, 1024, buf, 2048),
              kSyscallErrorIo);
    mock.next_reply.assign(512, 0x5A);
    mock.next_rc = (int64_t{1} << 32) + 1;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 2, 1, 512, buf, 512),
              kSyscallErrorIo);
    mock.next_rc = int64_t{1} << 31;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 2, 1, 512, buf, 512),
              kSyscallErrorIo);

    // Negative, 64-bit aliased, and empty (`reply_size == 0`) replies must be
    // rejected with `kSyscallErrorIo` without touching caller memory.
    mock.next_rc = 1;
    mock.override_reply_size = -1;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 2, 1, 512, buf, 512),
              kSyscallErrorIo);
    mock.override_reply_size = -512;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 2, 1, 512, buf, 512),
              kSyscallErrorIo);
    {
      uint8_t sentinel[512];
      __builtin_memset(sentinel, 0xA5, sizeof(sentinel));
      mock.next_reply.clear();
      mock.override_reply_size = (int64_t{1} << 32) + 512;
      EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                        0,           //
                                        2,           //
                                        1,           //
                                        512,         //
                                        sentinel,    //
                                        sizeof(sentinel)),
                kSyscallErrorIo);
      mock.override_reply_size = -(int64_t{1} << 32) + 512;
      EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                        0,           //
                                        2,           //
                                        1,           //
                                        512,         //
                                        sentinel,    //
                                        sizeof(sentinel)),
                kSyscallErrorIo);
      EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));
      mock.override_reply_size = 0;
      EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                        0,           //
                                        2,           //
                                        1,           //
                                        512,         //
                                        sentinel,    //
                                        sizeof(sentinel)),
                kSyscallErrorIo);
      mock.override_reply_size.reset();
      EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));
    }
    // Multi-block short reply matching `expected_block_size` (512 != 1024)
    // must fail with `kSyscallErrorIo` without touching caller memory.
    {
      uint8_t sentinel[1024];
      __builtin_memset(sentinel, 0xA5, sizeof(sentinel));
      mock.next_rc = 2;
      mock.next_reply.clear();
      mock.override_reply_size = 512;
      EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                        0,           //
                                        2,           //
                                        2,           //
                                        512,         //
                                        sentinel,    //
                                        sizeof(sentinel)),
                kSyscallErrorIo);
      mock.override_reply_size.reset();
      EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));
    }
    // Exact +/-1 reply-size boundary around `expected_bytes` (511/513 and
    // 1023/1025) must fail with `kSyscallErrorIo` without touching memory.
    {
      uint8_t sentinel[1024];
      __builtin_memset(sentinel, 0xA5, sizeof(sentinel));
      mock.next_reply.clear();
      mock.next_rc = 1;
      for (const int64_t bad_sz : {int64_t{511}, int64_t{513}}) {
        mock.override_reply_size = bad_sz;
        EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                          0,           //
                                          2,           //
                                          1,           //
                                          512,         //
                                          sentinel,    //
                                          sizeof(sentinel)),
                  kSyscallErrorIo);
      }
      mock.next_rc = 2;
      for (const int64_t bad_sz : {int64_t{1023}, int64_t{1025}}) {
        mock.override_reply_size = bad_sz;
        EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                          0,           //
                                          2,           //
                                          2,           //
                                          512,         //
                                          sentinel,    //
                                          sizeof(sentinel)),
                  kSyscallErrorIo);
      }
      mock.override_reply_size.reset();
      EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));
    }
  }
  {
    FakeBlockClient mock;
    uint8_t sentinel[512];
    __builtin_memset(sentinel, 0xA5, sizeof(sentinel));
    mock.next_rc = kSyscallErrorIo;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      0,           //
                                      1,           //
                                      512,         //
                                      sentinel,    //
                                      sizeof(sentinel)),
              kSyscallErrorIo);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 0, 512, nullptr, 0),
              kSyscallErrorIo);
    EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));
    mock.next_rc = kSyscallErrorNoDevice;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      0,           //
                                      1,           //
                                      512,         //
                                      sentinel,    //
                                      sizeof(sentinel)),
              kSyscallErrorNoDevice);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 0, 512, nullptr, 0),
              kSyscallErrorNoDevice);
    EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));
    mock.next_rc = kSyscallErrorInvalid;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      0,           //
                                      1,           //
                                      512,         //
                                      sentinel,    //
                                      sizeof(sentinel)),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 0, 512, nullptr, 0),
              kSyscallErrorInvalid);
    EXPECT_EQ(mock.calls, 6);
    EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));
    mock.next_reply.assign(sizeof(sentinel), 0x11);
    mock.next_rc = kSyscallErrorPermission;
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(),  //
                                      0,           //
                                      0,           //
                                      1,           //
                                      512,         //
                                      sentinel,    //
                                      sizeof(sentinel)),
              kSyscallErrorPermission);
    EXPECT_EQ(user::ReadBlocksChecked(mock.Env(), 0, 0, 0, 512, nullptr, 0),
              kSyscallErrorPermission);
    EXPECT_EQ(mock.calls, 8);
    EXPECT_THAT(sentinel, t::Each(t::Eq(0xA5)));
  }
}

TEST(SyscallAbiTest,
     BlockClientWriteBlocksAndWithScratchValidateArgsPayloadAndReplies) {
  uint8_t payload[kBlockMaxIoBytes] = {};
  uint8_t scratch[user::kBlockWriteScratchBytes] = {};
  {
    const user::BlockClientEnv null_env{};
    EXPECT_EQ(user::WriteBlocks(null_env, 0, 0, 1, payload, 512),
              kSyscallErrorInvalid);
    EXPECT_EQ(user::WriteBlocksWithScratch(null_env,  //
                                           0,         //
                                           0,         //
                                           1,         //
                                           payload,   //
                                           512,       //
                                           scratch,   //
                                           sizeof(scratch)),
              kSyscallErrorInvalid);
  }
  {
    FakeBlockClient mock;
    uint8_t big_scratch[user::kBlockWriteScratchBytes + 1] = {};
    const auto expect_no_ipc = [&](const int64_t rc) {
      EXPECT_EQ(rc, kSyscallErrorInvalid);
      EXPECT_EQ(mock.calls, 0);
    };

    // Direct `WriteBlocks` invalid-argument checks.
    expect_no_ipc(user::WriteBlocks(mock.Env(), 0, 0, -1, payload, 0));
    expect_no_ipc(user::WriteBlocks(mock.Env(), 0, 0, 0, payload, -1));
    expect_no_ipc(user::WriteBlocks(mock.Env(), 0, 0, 1, payload, -1));
    expect_no_ipc(user::WriteBlocks(mock.Env(),  //
                                    0,           //
                                    0,           //
                                    1,           //
                                    payload,     //
                                    -(int64_t{1} << 32) + 512));
    expect_no_ipc(
        user::WriteBlocks(mock.Env(), 0, 0, 1, payload, kBlockMaxIoBytes + 1));
    expect_no_ipc(user::WriteBlocks(mock.Env(),  //
                                    0,           //
                                    0,           //
                                    1,           //
                                    payload,     //
                                    (int64_t{1} << 32) + 512));
    expect_no_ipc(user::WriteBlocks(mock.Env(), 0, 0, 0, payload, 512));
    expect_no_ipc(user::WriteBlocks(mock.Env(), 0, 0, 0, payload, 1));
    expect_no_ipc(user::WriteBlocks(mock.Env(), 0, 0, 1, payload, 0));
    expect_no_ipc(user::WriteBlocks(mock.Env(), 0, 0, 1, nullptr, 512));
    expect_no_ipc(user::WriteBlocks(mock.Env(), 0, 0, 1, nullptr, 1));

    // Direct `WriteBlocksWithScratch` invalid-argument checks (`COV-01`),
    // including `(1, -1)` so `byte_count < 0` is isolated from
    // `block_count == 0` and `4121`-byte `big_scratch` so
    // `byte_count > kBlockMaxIoBytes` is isolated from `scratch_cap <
    // msg_size`.
    expect_no_ipc(user::WriteBlocksWithScratch(mock.Env(),  //
                                               0,           //
                                               0,           //
                                               -1,          //
                                               payload,     //
                                               0,           //
                                               scratch,     //
                                               sizeof(scratch)));
    expect_no_ipc(user::WriteBlocksWithScratch(mock.Env(),  //
                                               0,           //
                                               0,           //
                                               0,           //
                                               payload,     //
                                               -1,          //
                                               scratch,     //
                                               sizeof(scratch)));
    expect_no_ipc(user::WriteBlocksWithScratch(mock.Env(),  //
                                               0,           //
                                               0,           //
                                               1,           //
                                               payload,     //
                                               -1,          //
                                               scratch,     //
                                               sizeof(scratch)));
    expect_no_ipc(user::WriteBlocksWithScratch(mock.Env(),                 //
                                               0,                          //
                                               0,                          //
                                               1,                          //
                                               payload,                    //
                                               -(int64_t{1} << 32) + 512,  //
                                               scratch,                    //
                                               sizeof(scratch)));
    expect_no_ipc(user::WriteBlocksWithScratch(mock.Env(),            //
                                               0,                     //
                                               0,                     //
                                               1,                     //
                                               payload,               //
                                               kBlockMaxIoBytes + 1,  //
                                               big_scratch,           //
                                               sizeof(big_scratch)));
    expect_no_ipc(user::WriteBlocksWithScratch(
        mock.Env(),                //
        0,                         //
        0,                         //
        1,                         //
        payload,                   //
        (int64_t{1} << 32) + 512,  //
        scratch,                   //
        static_cast<int64_t>(sizeof(BlockWriteRequest)) + (int64_t{1} << 32) +
            512));
    expect_no_ipc(user::WriteBlocksWithScratch(mock.Env(),  //
                                               0,           //
                                               0,           //
                                               0,           //
                                               payload,     //
                                               512,         //
                                               scratch,     //
                                               sizeof(scratch)));
    expect_no_ipc(user::WriteBlocksWithScratch(mock.Env(),  //
                                               0,           //
                                               0,           //
                                               0,           //
                                               payload,     //
                                               1,           //
                                               scratch,     //
                                               sizeof(scratch)));
    expect_no_ipc(user::WriteBlocksWithScratch(mock.Env(),  //
                                               0,           //
                                               0,           //
                                               1,           //
                                               payload,     //
                                               0,           //
                                               scratch,     //
                                               sizeof(scratch)));
    expect_no_ipc(user::WriteBlocksWithScratch(mock.Env(),  //
                                               0,           //
                                               0,           //
                                               1,           //
                                               nullptr,     //
                                               512,         //
                                               scratch,     //
                                               sizeof(scratch)));
    expect_no_ipc(user::WriteBlocksWithScratch(mock.Env(),  //
                                               0,           //
                                               0,           //
                                               1,           //
                                               nullptr,     //
                                               1,           //
                                               scratch,     //
                                               sizeof(scratch)));
    expect_no_ipc(user::WriteBlocksWithScratch(mock.Env(),  //
                                               0,           //
                                               0,           //
                                               0,           //
                                               nullptr,     //
                                               0,           //
                                               nullptr,     //
                                               sizeof(scratch)));
    expect_no_ipc(user::WriteBlocksWithScratch(mock.Env(),  //
                                               0,           //
                                               0,           //
                                               1,           //
                                               payload,     //
                                               512,         //
                                               nullptr,     //
                                               sizeof(scratch)));
    expect_no_ipc(user::WriteBlocksWithScratch(mock.Env(),  //
                                               0,           //
                                               0,           //
                                               0,           //
                                               nullptr,     //
                                               0,           //
                                               scratch,     //
                                               -1));
    expect_no_ipc(user::WriteBlocksWithScratch(mock.Env(),  //
                                               0,           //
                                               0,           //
                                               0,           //
                                               nullptr,     //
                                               0,           //
                                               scratch,     //
                                               0));
    expect_no_ipc(user::WriteBlocksWithScratch(mock.Env(),  //
                                               0,           //
                                               0,           //
                                               0,           //
                                               nullptr,     //
                                               0,           //
                                               scratch,     //
                                               sizeof(BlockWriteRequest) - 1));
    expect_no_ipc(user::WriteBlocksWithScratch(mock.Env(),  //
                                               0,           //
                                               0,           //
                                               8,           //
                                               payload,     //
                                               4096,        //
                                               scratch,     //
                                               4096));
    expect_no_ipc(
        user::WriteBlocksWithScratch(mock.Env(),  //
                                     0,           //
                                     0,           //
                                     8,           //
                                     payload,     //
                                     4096,        //
                                     scratch,     //
                                     user::kBlockWriteScratchBytes - 1));
    expect_no_ipc(user::WriteBlocksWithScratch(
        mock.Env(),  //
        0,           //
        0,           //
        1,           //
        payload,     //
        512,         //
        scratch,     //
        static_cast<int64_t>(sizeof(BlockWriteRequest)) + 512 - 1));
    expect_no_ipc(user::WriteBlocksWithScratch(
        mock.Env(),  //
        0,           //
        0,           //
        1,           //
        payload,     //
        512,         //
        scratch,     //
        -(int64_t{1} << 32) + static_cast<int64_t>(sizeof(BlockWriteRequest)) +
            512));
  }

  // Valid 0-block writes send 24-byte header only (with full 64-bit LBA and
  // 32-bit disk_index), including when `data` is non-null with `byte_count=0`.
  {
    FakeBlockClient mock;
    mock.next_rc = 0;
    const uint32_t disk_index = 0x12345678u;
    const uint64_t lba = 0x123456789ABCDEF0ULL;
    EXPECT_EQ(user::WriteBlocks(mock.Env(), disk_index, lba, 0, nullptr, 0), 0);
    const BlockWriteRequest expected_req{.op = BlockRpcOp::kWrite,
                                         .disk_index = disk_index,
                                         .lba = lba,
                                         .block_count = 0,
                                         .reserved = 0};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));
    EXPECT_EQ(mock.last_reply_buf, nullptr);
    EXPECT_EQ(mock.last_reply_cap, 0);

    EXPECT_EQ(user::WriteBlocks(mock.Env(), disk_index, lba, 0, payload, 0), 0);
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));
    EXPECT_EQ(mock.last_reply_buf, nullptr);
    EXPECT_EQ(mock.last_reply_cap, 0);

    const uint64_t high_lba = uint64_t{1} << 63;
    EXPECT_EQ(
        user::WriteBlocks(mock.Env(), disk_index, high_lba, 0, nullptr, 0), 0);
    const BlockWriteRequest high_req{.op = BlockRpcOp::kWrite,
                                     .disk_index = disk_index,
                                     .lba = high_lba,
                                     .block_count = 0,
                                     .reserved = 0};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(high_req)));
  }
  {
    FakeBlockClient mock;
    uint8_t min_scratch[sizeof(BlockWriteRequest)] = {};
    mock.next_rc = 0;
    const uint32_t disk_index = 0x12345678u;
    const uint64_t lba = 0x123456789ABCDEF0ULL;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),   //
                                           disk_index,   //
                                           lba,          //
                                           0,            //
                                           nullptr,      //
                                           0,            //
                                           min_scratch,  //
                                           sizeof(min_scratch)),
              0);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    const BlockWriteRequest expected_req{.op = BlockRpcOp::kWrite,
                                         .disk_index = disk_index,
                                         .lba = lba,
                                         .block_count = 0,
                                         .reserved = 0};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));
    EXPECT_EQ(mock.last_reply_buf, nullptr);
    EXPECT_EQ(mock.last_reply_cap, 0);

    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),   //
                                           disk_index,   //
                                           lba,          //
                                           0,            //
                                           payload,      //
                                           0,            //
                                           min_scratch,  //
                                           sizeof(min_scratch)),
              0);
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));
    EXPECT_EQ(mock.last_reply_buf, nullptr);
    EXPECT_EQ(mock.last_reply_cap, 0);

    const uint64_t high_lba = uint64_t{1} << 63;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),   //
                                           disk_index,   //
                                           high_lba,     //
                                           0,            //
                                           nullptr,      //
                                           0,            //
                                           min_scratch,  //
                                           sizeof(min_scratch)),
              0);
    const BlockWriteRequest high_req{.op = BlockRpcOp::kWrite,
                                     .disk_index = disk_index,
                                     .lba = high_lba,
                                     .block_count = 0,
                                     .reserved = 0};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(high_req)));
  }

  for (int i = 0; i < kBlockMaxIoBytes; ++i) {
    payload[i] = static_cast<uint8_t>((i * 29 + 7) & 0xFF);
  }

  // `WriteBlocks(block_count > 0)` 1-block and max 4096 B stack-buffer paths.
  {
    FakeBlockClient mock;
    mock.next_rc = 1;
    EXPECT_EQ(user::WriteBlocks(mock.Env(), 2, 17, 1, payload, 512), 1);
    EXPECT_EQ(mock.calls, 1);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    EXPECT_EQ(mock.last_reply_buf, nullptr);
    EXPECT_EQ(mock.last_reply_cap, 0);
    ASSERT_EQ(mock.last_req.size(), sizeof(BlockWriteRequest) + 512);
    const BlockWriteRequest expected_req{.op = BlockRpcOp::kWrite,
                                         .disk_index = 2,
                                         .lba = 17,
                                         .block_count = 1,
                                         .reserved = 0};
    EXPECT_EQ(__builtin_memcmp(mock.last_req.data(),  //
                               &expected_req,         //
                               sizeof(expected_req)),
              0);
    EXPECT_EQ(
        __builtin_memcmp(mock.last_req.data() + sizeof(BlockWriteRequest),  //
                         payload,                                           //
                         512),
        0);
  }
  {
    FakeBlockClient mock;
    mock.next_rc = 8;
    const uint32_t disk_index = 0x12345678u;
    const uint64_t lba = 0x123456789ABCDEF0ULL;
    EXPECT_EQ(user::WriteBlocks(mock.Env(),  //
                                disk_index,  //
                                lba,         //
                                8,           //
                                payload,     //
                                kBlockMaxIoBytes),
              8);
    EXPECT_EQ(mock.calls, 1);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    EXPECT_EQ(mock.last_reply_buf, nullptr);
    EXPECT_EQ(mock.last_reply_cap, 0);
    ASSERT_EQ(mock.last_req.size(),
              static_cast<size_t>(user::kBlockWriteScratchBytes));
    const BlockWriteRequest expected_req{.op = BlockRpcOp::kWrite,
                                         .disk_index = disk_index,
                                         .lba = lba,
                                         .block_count = 8,
                                         .reserved = 0};
    EXPECT_EQ(__builtin_memcmp(mock.last_req.data(),  //
                               &expected_req,         //
                               sizeof(expected_req)),
              0);
    EXPECT_EQ(
        __builtin_memcmp(mock.last_req.data() + sizeof(BlockWriteRequest),  //
                         payload,                                           //
                         kBlockMaxIoBytes),
        0);

    mock.next_rc = 1;
    const uint32_t msb_disk = 0x80000001u;
    const uint64_t high_lba = 0xFEDCBA9876543210ULL;
    EXPECT_EQ(
        user::WriteBlocks(mock.Env(), msb_disk, high_lba, 1, payload, 512), 1);
    const BlockWriteRequest high_req{.op = BlockRpcOp::kWrite,
                                     .disk_index = msb_disk,
                                     .lba = high_lba,
                                     .block_count = 1,
                                     .reserved = 0};
    std::vector<uint8_t> expected_msg = ToBytes(high_req);
    expected_msg.insert(expected_msg.end(), payload, payload + 512);
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(expected_msg));
  }

  // Disjoint, zero-copy (scratch + 24), and overlapping staging.
  {
    FakeBlockClient mock;
    mock.next_rc = 1;
    const uint32_t disk_index = 0x12345678u;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),        //
                                           disk_index,        //
                                           5,                 //
                                           1,                 //
                                           payload,           //
                                           kBlockMaxIoBytes,  //
                                           scratch,           //
                                           sizeof(scratch)),
              1);
    ASSERT_EQ(mock.last_req.size(),
              static_cast<size_t>(user::kBlockWriteScratchBytes));
    const BlockWriteRequest expected_req{.op = BlockRpcOp::kWrite,
                                         .disk_index = disk_index,
                                         .lba = 5,
                                         .block_count = 1,
                                         .reserved = 0};
    EXPECT_EQ(__builtin_memcmp(mock.last_req.data(),  //
                               &expected_req,         //
                               sizeof(expected_req)),
              0);
    EXPECT_EQ(
        __builtin_memcmp(mock.last_req.data() + sizeof(BlockWriteRequest),  //
                         payload,                                           //
                         kBlockMaxIoBytes),
        0);

    const uint32_t msb_disk = 0x80000001u;
    const uint64_t high_lba = 0xFEDCBA9876543210ULL;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           msb_disk,    //
                                           high_lba,    //
                                           1,           //
                                           payload,     //
                                           512,         //
                                           scratch,     //
                                           sizeof(scratch)),
              1);
    const BlockWriteRequest high_req{.op = BlockRpcOp::kWrite,
                                     .disk_index = msb_disk,
                                     .lba = high_lba,
                                     .block_count = 1,
                                     .reserved = 0};
    std::vector<uint8_t> expected_msg = ToBytes(high_req);
    expected_msg.insert(expected_msg.end(), payload, payload + 512);
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(expected_msg));
  }
  {
    FakeBlockClient mock;
    mock.next_rc = 2;
    __builtin_memcpy(scratch + sizeof(BlockWriteRequest), payload, 1024);
    EXPECT_EQ(
        user::WriteBlocksWithScratch(mock.Env(),                           //
                                     0,                                    //
                                     10,                                   //
                                     2,                                    //
                                     scratch + sizeof(BlockWriteRequest),  //
                                     1024,                                 //
                                     scratch,                              //
                                     sizeof(scratch)),
        2);
    ASSERT_EQ(mock.last_req.size(), sizeof(BlockWriteRequest) + 1024);
    const BlockWriteRequest expected_req{.op = BlockRpcOp::kWrite,
                                         .disk_index = 0,
                                         .lba = 10,
                                         .block_count = 2,
                                         .reserved = 0};
    EXPECT_EQ(__builtin_memcmp(mock.last_req.data(),  //
                               &expected_req,         //
                               sizeof(expected_req)),
              0);
    EXPECT_EQ(
        __builtin_memcmp(mock.last_req.data() + sizeof(BlockWriteRequest),  //
                         payload,                                           //
                         1024),
        0);
  }
  {
    FakeBlockClient mock;
    mock.next_rc = 1;
    __builtin_memcpy(scratch, payload, 512);
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           11,          //
                                           1,           //
                                           scratch,     //
                                           512,         //
                                           scratch,     //
                                           sizeof(scratch)),
              1);
    ASSERT_EQ(mock.last_req.size(), sizeof(BlockWriteRequest) + 512);
    const BlockWriteRequest expected_req{.op = BlockRpcOp::kWrite,
                                         .disk_index = 0,
                                         .lba = 11,
                                         .block_count = 1,
                                         .reserved = 0};
    EXPECT_EQ(__builtin_memcmp(mock.last_req.data(),  //
                               &expected_req,         //
                               sizeof(expected_req)),
              0);
    EXPECT_EQ(
        __builtin_memcmp(mock.last_req.data() + sizeof(BlockWriteRequest),  //
                         payload,                                           //
                         512),
        0);
  }

  // Rightward overlaps (`d < s`: `scratch + 40`, `scratch + 25`) and partial
  // header-straddling overlap (`scratch + 12`).
  for (const int offset : {40, 25, 12}) {
    FakeBlockClient mock;
    mock.next_rc = 1;
    __builtin_memcpy(scratch + offset, payload, 512);
    const uint64_t lba = static_cast<uint64_t>(offset);
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),        //
                                           0,                 //
                                           lba,               //
                                           1,                 //
                                           scratch + offset,  //
                                           512,               //
                                           scratch,           //
                                           sizeof(scratch)),
              1)
        << "offset=" << offset;
    ASSERT_EQ(mock.last_req.size(), sizeof(BlockWriteRequest) + 512);
    const BlockWriteRequest expected_req{.op = BlockRpcOp::kWrite,
                                         .disk_index = 0,
                                         .lba = lba,
                                         .block_count = 1,
                                         .reserved = 0};
    EXPECT_EQ(__builtin_memcmp(mock.last_req.data(),  //
                               &expected_req,         //
                               sizeof(expected_req)),
              0);
    EXPECT_EQ(
        __builtin_memcmp(mock.last_req.data() + sizeof(BlockWriteRequest),  //
                         payload,                                           //
                         512),
        0);
  }

  // Exact-fit small scratch buffer (`sizeof(BlockWriteRequest) + 512 == 536`
  // bytes < `kBlockWriteScratchBytes`) and 64-bit `scratch_cap` succeed, while
  // `535` fails before IPC.
  {
    FakeBlockClient mock;
    uint8_t small_scratch[sizeof(BlockWriteRequest) + 512] = {};
    const int64_t exact_cap = static_cast<int64_t>(sizeof(small_scratch));
    mock.next_rc = 1;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),     //
                                           0,              //
                                           13,             //
                                           1,              //
                                           payload,        //
                                           512,            //
                                           small_scratch,  //
                                           exact_cap - 1),
              kSyscallErrorInvalid);
    EXPECT_EQ(mock.calls, 0);
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),     //
                                           0,              //
                                           13,             //
                                           1,              //
                                           payload,        //
                                           512,            //
                                           small_scratch,  //
                                           exact_cap),
              1);
    EXPECT_EQ(mock.calls, 1);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    const BlockWriteRequest expected_req{.op = BlockRpcOp::kWrite,
                                         .disk_index = 0,
                                         .lba = 13,
                                         .block_count = 1,
                                         .reserved = 0};
    std::vector<uint8_t> expected_msg = ToBytes(expected_req);
    expected_msg.insert(expected_msg.end(), payload, payload + 512);
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(expected_msg));

    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           13,          //
                                           1,           //
                                           payload,     //
                                           512,         //
                                           scratch,     //
                                           int64_t{1} << 32),
              1);
    EXPECT_EQ(mock.calls, 2);
  }

  // Non-sector-multiple `byte_count` (511 B) passes through to the server so
  // server-side rejection tests keep working.
  {
    FakeBlockClient mock;
    mock.next_rc = kSyscallErrorInvalid;
    EXPECT_EQ(user::WriteBlocks(mock.Env(), 0, 0, 1, payload, 511),
              kSyscallErrorInvalid);
    EXPECT_EQ(mock.calls, 1);
    EXPECT_EQ(mock.last_req.size(), sizeof(BlockWriteRequest) + 511);
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           1,           //
                                           payload,     //
                                           511,         //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorInvalid);
    EXPECT_EQ(mock.calls, 2);
    EXPECT_EQ(mock.last_req.size(), sizeof(BlockWriteRequest) + 511);

    const BlockWriteRequest expected_wide_cnt_req{.op = BlockRpcOp::kWrite,
                                                  .disk_index = 0,
                                                  .lba = 0,
                                                  .block_count = 0x1020304,
                                                  .reserved = 0};
    std::vector<uint8_t> expected_msg = ToBytes(expected_wide_cnt_req);
    expected_msg.insert(expected_msg.end(), payload, payload + 512);
    EXPECT_EQ(user::WriteBlocks(mock.Env(), 0, 0, 0x1020304, payload, 512),
              kSyscallErrorInvalid);
    EXPECT_EQ(mock.calls, 3);
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(expected_msg));
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           0x1020304,   //
                                           payload,     //
                                           512,         //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorInvalid);
    EXPECT_EQ(mock.calls, 4);
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(expected_msg));
  }

  // Server mismatch/non-empty reply fail with Io, while negative errors pass.
  {
    FakeBlockClient mock;
    mock.next_rc = 2;
    EXPECT_EQ(user::WriteBlocks(mock.Env(), 0, 0, 1, payload, 512),
              kSyscallErrorIo);
    mock.next_rc = (int64_t{1} << 32) + 1;
    EXPECT_EQ(user::WriteBlocks(mock.Env(), 0, 0, 1, payload, 512),
              kSyscallErrorIo);
    mock.next_rc = int64_t{1} << 31;
    EXPECT_EQ(user::WriteBlocks(mock.Env(), 0, 0, 1, payload, 512),
              kSyscallErrorIo);
    mock.next_rc = 1;
    mock.override_reply_size = 4;
    EXPECT_EQ(user::WriteBlocks(mock.Env(), 0, 0, 1, payload, 512),
              kSyscallErrorIo);
    mock.override_reply_size = 1;
    EXPECT_EQ(user::WriteBlocks(mock.Env(), 0, 0, 1, payload, 512),
              kSyscallErrorIo);
    mock.override_reply_size = -1;
    EXPECT_EQ(user::WriteBlocks(mock.Env(), 0, 0, 1, payload, 512),
              kSyscallErrorIo);
    mock.override_reply_size = int64_t{1} << 32;
    EXPECT_EQ(user::WriteBlocks(mock.Env(), 0, 0, 1, payload, 512),
              kSyscallErrorIo);
    mock.override_reply_size = -(int64_t{1} << 32);
    EXPECT_EQ(user::WriteBlocks(mock.Env(), 0, 0, 1, payload, 512),
              kSyscallErrorIo);
    mock.override_reply_size.reset();
    mock.next_rc = kSyscallErrorPermission;
    EXPECT_EQ(user::WriteBlocks(mock.Env(), 0, 0, 1, payload, 512),
              kSyscallErrorPermission);
    mock.next_rc = kSyscallErrorNoDevice;
    EXPECT_EQ(user::WriteBlocks(mock.Env(), 0, 0, 0, nullptr, 0),
              kSyscallErrorNoDevice);

    // Direct `WriteBlocksWithScratch` reply checks (status mismatch, non-zero
    // and negative `reply_size`, 0-block errors, negative error propagation).
    mock.next_rc = 2;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           1,           //
                                           payload,     //
                                           512,         //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorIo);
    mock.next_rc = 0;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           1,           //
                                           payload,     //
                                           512,         //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorIo);
    mock.next_rc = (int64_t{1} << 32) + 1;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           1,           //
                                           payload,     //
                                           512,         //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorIo);
    mock.next_rc = int64_t{1} << 31;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           1,           //
                                           payload,     //
                                           512,         //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorIo);
    mock.next_rc = 1;
    mock.override_reply_size = 4;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           1,           //
                                           payload,     //
                                           512,         //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorIo);
    mock.override_reply_size = 1;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           1,           //
                                           payload,     //
                                           512,         //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorIo);
    mock.override_reply_size = -1;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           1,           //
                                           payload,     //
                                           512,         //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorIo);
    mock.override_reply_size = int64_t{1} << 32;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           1,           //
                                           payload,     //
                                           512,         //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorIo);
    mock.override_reply_size = -(int64_t{1} << 32);
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           1,           //
                                           payload,     //
                                           512,         //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorIo);
    mock.override_reply_size.reset();
    mock.next_rc = 1;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           0,           //
                                           nullptr,     //
                                           0,           //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorIo);
    mock.next_rc = int64_t{1} << 32;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           0,           //
                                           nullptr,     //
                                           0,           //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorIo);
    mock.next_rc = 0;
    mock.override_reply_size = 4;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           0,           //
                                           nullptr,     //
                                           0,           //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorIo);
    mock.override_reply_size = 1;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           0,           //
                                           nullptr,     //
                                           0,           //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorIo);
    mock.override_reply_size = -1;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           0,           //
                                           nullptr,     //
                                           0,           //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorIo);
    mock.override_reply_size = int64_t{1} << 32;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           0,           //
                                           nullptr,     //
                                           0,           //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorIo);
    mock.override_reply_size = -(int64_t{1} << 32);
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           0,           //
                                           nullptr,     //
                                           0,           //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorIo);
    mock.override_reply_size.reset();
    mock.next_rc = kSyscallErrorPermission;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           1,           //
                                           payload,     //
                                           512,         //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorPermission);
    mock.next_rc = kSyscallErrorNoDevice;
    EXPECT_EQ(user::WriteBlocksWithScratch(mock.Env(),  //
                                           0,           //
                                           0,           //
                                           0,           //
                                           nullptr,     //
                                           0,           //
                                           scratch,     //
                                           sizeof(scratch)),
              kSyscallErrorNoDevice);
  }
}

TEST(SyscallAbiTest, BlockClientFlushDiskNormalizesPositiveAndNonEmptyReplies) {
  {
    const user::BlockClientEnv null_env{};
    EXPECT_EQ(user::FlushDisk(null_env, 0), kSyscallErrorInvalid);
  }
  {
    FakeBlockClient mock;
    mock.next_rc = 0;
    EXPECT_EQ(user::FlushDisk(mock.Env(), 2), 0);
    EXPECT_EQ(mock.last_port, kIpcPortBlock);
    const BlockFlushRequest expected_req{.op = BlockRpcOp::kFlush,
                                         .disk_index = 2};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(expected_req)));
    EXPECT_EQ(mock.last_reply_buf, nullptr);
    EXPECT_EQ(mock.last_reply_cap, 0);

    const uint32_t wide_disk_index = 0x12345678u;
    EXPECT_EQ(user::FlushDisk(mock.Env(), wide_disk_index), 0);
    const BlockFlushRequest wide_req{.op = BlockRpcOp::kFlush,
                                     .disk_index = wide_disk_index};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(wide_req)));
    const uint32_t msb_disk = 0x80000001u;
    EXPECT_EQ(user::FlushDisk(mock.Env(), msb_disk), 0);
    const BlockFlushRequest msb_req{.op = BlockRpcOp::kFlush,
                                    .disk_index = msb_disk};
    EXPECT_THAT(mock.last_req, t::ElementsAreArray(ToBytes(msb_req)));

    mock.next_rc = kSyscallErrorNoDevice;
    EXPECT_EQ(user::FlushDisk(mock.Env(), 2), kSyscallErrorNoDevice);
    mock.next_rc = kSyscallErrorPermission;
    EXPECT_EQ(user::FlushDisk(mock.Env(), 2), kSyscallErrorPermission);

    mock.next_rc = 1;
    EXPECT_EQ(user::FlushDisk(mock.Env(), 2), kSyscallErrorIo);
    mock.next_rc = 42;
    EXPECT_EQ(user::FlushDisk(mock.Env(), 2), kSyscallErrorIo);
    mock.next_rc = int64_t{1} << 32;
    EXPECT_EQ(user::FlushDisk(mock.Env(), 2), kSyscallErrorIo);
    mock.next_rc = int64_t{1} << 31;
    EXPECT_EQ(user::FlushDisk(mock.Env(), 2), kSyscallErrorIo);

    mock.next_rc = 0;
    mock.override_reply_size = 8;
    EXPECT_EQ(user::FlushDisk(mock.Env(), 2), kSyscallErrorIo);
    mock.override_reply_size = 1;
    EXPECT_EQ(user::FlushDisk(mock.Env(), 2), kSyscallErrorIo);
    mock.override_reply_size = -1;
    EXPECT_EQ(user::FlushDisk(mock.Env(), 2), kSyscallErrorIo);
    mock.override_reply_size = int64_t{1} << 32;
    EXPECT_EQ(user::FlushDisk(mock.Env(), 2), kSyscallErrorIo);
    mock.override_reply_size = -(int64_t{1} << 32);
    EXPECT_EQ(user::FlushDisk(mock.Env(), 2), kSyscallErrorIo);
    mock.override_reply_size.reset();
  }
}

}  // namespace
}  // namespace protos
