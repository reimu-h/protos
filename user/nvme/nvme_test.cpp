#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "block_protocol.h"
#include "syscall_abi.h"
#include "user/nvme/nvme_controller.h"
#include "user/nvme/nvme_platform.h"
#include "user/nvme/nvme_regs.h"
#include "user/nvme/nvme_server.h"
#include "user/nvme/nvme_sim.h"

namespace protos::nvme {
namespace {

namespace t = ::testing;

// --- NvmeRegsTest ------------------------------------------------------------

TEST(NvmeRegsTest, CapAndDoorbellDecoders) {
  const uint64_t cap = (uint64_t{0} << 0) | (uint64_t{0} << 24) |
                       (uint64_t{2} << 32) | kCapCssNvm | (uint64_t{1} << 48) |
                       (uint64_t{4} << 52);
  EXPECT_EQ(CapMqes(cap), 1);
  EXPECT_EQ(CapToMs(cap), 500);
  EXPECT_EQ(CapDstrd(cap), 2);
  EXPECT_EQ(CapDstrd(uint64_t{1} << 36), 0);
  EXPECT_EQ(CapDstrd((uint64_t{1} << 36) | (uint64_t{8} << 32)), 8);
  EXPECT_EQ(CapDstrd(uint64_t{15} << 32), 15);
  EXPECT_EQ(CapDstrd(0xFFFFFFFFULL), 0);
  EXPECT_EQ(CapMpsMin(cap), 1);
  EXPECT_EQ(CapMpsMin((uint64_t{8} << 48) | (uint64_t{1} << 52)), 8);
  EXPECT_EQ(CapMpsMin(uint64_t{15} << 48), 15);
  EXPECT_EQ(CapMpsMin(uint64_t{1} << 52), 0);
  EXPECT_EQ(CapMpsMin(0x0000FFFFFFFFFFFFULL), 0);
  EXPECT_EQ(CapMpsMax(cap), 4);
  EXPECT_EQ(CapMpsMax((uint64_t{1} << 56) | (uint64_t{8} << 52)), 8);
  EXPECT_EQ(CapMpsMax(uint64_t{15} << 52), 15);
  EXPECT_EQ(CapMpsMax(uint64_t{1} << 56), 0);
  EXPECT_EQ(CapMpsMax(0x000FFFFFFFFFFFFFULL), 0);

  const uint64_t cap_max_to = (uint64_t{63} << 0) | (uint64_t{255} << 24);
  EXPECT_EQ(CapMqes(cap_max_to), 64);
  EXPECT_EQ(CapMqes(0x0100ULL), 257);
  EXPECT_EQ(CapMqes(0x8000ULL), 32769);
  EXPECT_EQ(CapMqes(0xFFFFULL), 65536);
  EXPECT_EQ(CapMqes(0x00010000ULL), 1);
  EXPECT_EQ(CapToMs(cap_max_to), 127500);
  EXPECT_EQ(CapToMs(uint64_t{1} << 32), 500);
  EXPECT_EQ(CapToMs((uint64_t{1} << 32) | (uint64_t{2} << 24)), 1000);
  EXPECT_EQ(CapToMs((uint64_t{1} << 32) | (uint64_t{255} << 24)), 127500);
  EXPECT_EQ(CapToMs(0x00FFFFFFULL), 500);

  EXPECT_EQ(SqTailDoorbellOffset(0, 0), 0x1000);
  EXPECT_EQ(CqHeadDoorbellOffset(0, 0), 0x1004);
  EXPECT_EQ(SqTailDoorbellOffset(1, 0), 0x1008);
  EXPECT_EQ(CqHeadDoorbellOffset(1, 0), 0x100C);

  EXPECT_EQ(SqTailDoorbellOffset(1, 2), 0x1020);
  EXPECT_EQ(CqHeadDoorbellOffset(1, 2), 0x1030);
  EXPECT_EQ(SqTailDoorbellOffset(1, 10), 0x3000);
  EXPECT_EQ(CqHeadDoorbellOffset(1, 10), 0x4000);
  EXPECT_EQ(CqHeadDoorbellOffset(1, 15), 0x61000);

  EXPECT_EQ(SqTailDoorbellOffset(-1, 0), -1);
  EXPECT_EQ(CqHeadDoorbellOffset(-1, 0), -1);
  EXPECT_EQ(SqTailDoorbellOffset(0, -1), -1);
  EXPECT_EQ(SqTailDoorbellOffset(0, 16), -1);
  EXPECT_EQ(CqHeadDoorbellOffset(0, -1), -1);
  EXPECT_EQ(CqHeadDoorbellOffset(0, 16), -1);
  EXPECT_EQ(SqTailDoorbellOffset(1, 15), 0x41000);
  EXPECT_EQ(SqTailDoorbellOffset(2, 0), 0x1010);
  EXPECT_EQ(CqHeadDoorbellOffset(2, 0), 0x1014);
  EXPECT_EQ(SqTailDoorbellOffset(0x40000000, 0), 0x200001000LL);
  EXPECT_EQ(CqHeadDoorbellOffset(0x40000000, 0), 0x200001004LL);

  EXPECT_EQ(MakeSqCdw0(kAdminIdentify, 0x1234u), 0x12340006u);
  EXPECT_EQ(MakeSqCdw0(0xABu, 0x1234u), 0x123400ABu);
}

TEST(NvmeRegsTest, CqeStatusDecodersAndStructAbis) {
  EXPECT_EQ(static_cast<int>(sizeof(SubmissionEntry)), 64);
  EXPECT_EQ(static_cast<int>(sizeof(CompletionEntry)), 16);

  EXPECT_TRUE(CqeOk(0x0000u));
  EXPECT_TRUE(CqeOk(kCqePhaseMask));
  EXPECT_TRUE(CqeOk(kCqeDnrMask));
  EXPECT_TRUE(CqeOk(static_cast<uint16_t>(kCqeDnrMask | kCqePhaseMask)));
  EXPECT_FALSE(CqeDoNotRetry(0x0001u));
  EXPECT_FALSE(CqeDoNotRetry(static_cast<uint16_t>(1u << 14)));
  EXPECT_TRUE(CqeDoNotRetry(static_cast<uint16_t>(1u << 15)));
  EXPECT_TRUE(CqeDoNotRetry(kCqeDnrMask));
  EXPECT_EQ(kCqeDnrMask, static_cast<uint16_t>(1u << 15));

  // SCT != 0 or SC != 0 must both fail CqeOk().
  const uint16_t sct1_sc0 = static_cast<uint16_t>(1u << 9);
  const uint16_t sct0_sc5 = static_cast<uint16_t>(5u << 1);
  const uint16_t ro_err = static_cast<uint16_t>(
      (kStatusTypeCommandSpecific << 9) |
      (kStatusCodeWriteToReadOnlyRange << 1) | kCqeDnrMask);
  EXPECT_EQ(CqeStatusType(sct1_sc0), 1);
  EXPECT_EQ(CqeStatusCode(sct1_sc0), 0);
  EXPECT_FALSE(CqeOk(sct1_sc0));
  EXPECT_EQ(CqeStatusType(sct0_sc5), 0);
  EXPECT_EQ(CqeStatusCode(sct0_sc5), 5);
  EXPECT_FALSE(CqeOk(sct0_sc5));
  EXPECT_FALSE(CqeOk(ro_err));
  EXPECT_TRUE(CqeDoNotRetry(ro_err));
  const uint16_t sct4_sc0 = static_cast<uint16_t>(4u << 9);
  EXPECT_EQ(CqeStatusType(sct4_sc0), 4);
  EXPECT_EQ(CqeStatusCode(sct4_sc0), 0);
  EXPECT_FALSE(CqeOk(sct4_sc0));
  const uint16_t sct7_sc_max = static_cast<uint16_t>((7u << 9) | (0xFFu << 1));
  EXPECT_EQ(CqeStatusType(sct7_sc_max), 7);
  EXPECT_EQ(CqeStatusCode(sct7_sc_max), 0xFF);
  EXPECT_FALSE(CqeOk(sct7_sc_max));
  const uint16_t crd_more_ok = static_cast<uint16_t>((1u << 14) | (3u << 12));
  EXPECT_EQ(CqeStatusType(crd_more_ok), 0);
  EXPECT_TRUE(CqeOk(crd_more_ok));
  EXPECT_TRUE(CqeOk(static_cast<uint16_t>(crd_more_ok | kCqePhaseMask)));
  EXPECT_EQ(CqeStatusType(static_cast<uint16_t>((7u << 12) | (2u << 9))), 2);
  EXPECT_EQ(CqeStatusType(0xFFFFu), 7);
}

// --- NvmePlatformTest --------------------------------------------------------

TEST(NvmePlatformTest, MappedMmioWindowRespectsSegmentsAndHoles) {
  alignas(4) uint32_t page0[1024] = {};
  alignas(4) uint32_t page1[1024] = {};
  page0[0] = 0x11223344u;
  page0[1] = 0x55667788u;
  page1[0] = 0xAABBCCDDu;

  MappedMmioWindow win;
  win.Init(nullptr, 4096);
  EXPECT_EQ(win.Size(), 0);
  EXPECT_FALSE(win.Covers(0, 4));
  EXPECT_EQ(win.Read32(0), kMmioAllOnes);

  win.Init(reinterpret_cast<volatile uint8_t*>(page0), 4096);
  EXPECT_FALSE(
      win.AddSegment(-4, reinterpret_cast<volatile uint8_t*>(page1), 4096));
  EXPECT_FALSE(
      win.AddSegment(4092, reinterpret_cast<volatile uint8_t*>(page1), 4096));
  EXPECT_FALSE(
      win.AddSegment(4096, reinterpret_cast<volatile uint8_t*>(page1), 0));
  EXPECT_FALSE(
      win.AddSegment(4096, reinterpret_cast<volatile uint8_t*>(page1), -4));
  EXPECT_FALSE(
      win.AddSegment(4096, reinterpret_cast<volatile uint8_t*>(page1), 1));
  EXPECT_FALSE(
      win.AddSegment(4096, reinterpret_cast<volatile uint8_t*>(page1), 2));
  EXPECT_FALSE(
      win.AddSegment(4096, reinterpret_cast<volatile uint8_t*>(page1), 6));
  EXPECT_FALSE(
      win.AddSegment(4098, reinterpret_cast<volatile uint8_t*>(page1), 4096));
  EXPECT_FALSE(
      win.AddSegment(4097, reinterpret_cast<volatile uint8_t*>(page1), 4096));
  EXPECT_FALSE(win.AddSegment(INT64_MAX - 8,                               //
                              reinterpret_cast<volatile uint8_t*>(page1),  //
                              16));
  EXPECT_FALSE(win.AddSegment(INT64_MAX - 11,                              //
                              reinterpret_cast<volatile uint8_t*>(page1),  //
                              16));
  EXPECT_FALSE(win.AddSegment(4096, nullptr, 4096));
  EXPECT_FALSE(win.AddSegment(INT64_MAX - 7,                               //
                              reinterpret_cast<volatile uint8_t*>(page1),  //
                              8));
  EXPECT_TRUE(
      win.AddSegment(8192, reinterpret_cast<volatile uint8_t*>(page1), 4096));
  EXPECT_EQ(win.Size(), 12288);
  EXPECT_TRUE(win.Covers(0, 8));
  EXPECT_TRUE(win.Covers(0, 4096));
  EXPECT_TRUE(win.Covers(4092, 4));
  EXPECT_TRUE(win.Covers(8192, 4096));
  EXPECT_TRUE(win.Covers(12284, 4));
  EXPECT_FALSE(win.Covers(4096, 4));   // Inside unmapped MSI-X hole.
  EXPECT_FALSE(win.Covers(4092, 8));   // Crosses out of segment 0.
  EXPECT_FALSE(win.Covers(12284, 8));  // Crosses out of segment 1.
  EXPECT_FALSE(win.Covers(INT64_MAX - 2, 8));
  EXPECT_FALSE(win.Covers(INT64_MAX - 3, 4));
  EXPECT_FALSE(win.Covers(0, 0));
  EXPECT_FALSE(win.Covers(0, -4));
  EXPECT_FALSE(win.Covers(-4, 4));
  EXPECT_EQ(win.Read64(0), 0x5566778811223344ULL);
  EXPECT_EQ(win.Read64(-4), 0xFFFFFFFFFFFFFFFFULL);
  EXPECT_EQ(win.Read64(INT64_MAX - 2), 0xFFFFFFFFFFFFFFFFULL);
  EXPECT_EQ(win.Read64(INT64_MAX - 3), 0xFFFFFFFFFFFFFFFFULL);
  win.Write64(-4, 0);
  win.Write64(INT64_MAX - 2, 0);
  win.Write64(INT64_MAX - 3, 0);
  EXPECT_EQ(page0[0], 0x11223344u);
  EXPECT_EQ(page0[1], 0x55667788u);
  EXPECT_EQ(win.Read32(4096), kMmioAllOnes);
  EXPECT_EQ(win.Read32(8192), 0xAABBCCDDu);
  win.Write32(4096, 0x12345678u);  // Out of mapped segments, dropped.
  win.Write32(8196, 0x01020304u);
  EXPECT_EQ(page1[1], 0x01020304u);
  win.Write32(4092, 0xCAFEBABEu);
  EXPECT_EQ(page0[1023], 0xCAFEBABEu);
  EXPECT_EQ(win.Read32(4092), 0xCAFEBABEu);
  win.Write32(12284, 0x13572468u);
  EXPECT_EQ(page1[1023], 0x13572468u);
  EXPECT_EQ(win.Read32(12284), 0x13572468u);
  EXPECT_EQ(win.Read32(-4), kMmioAllOnes);
  EXPECT_EQ(win.Read32(2), kMmioAllOnes);
  EXPECT_EQ(win.Read32(INT64_MAX - 3), kMmioAllOnes);
  const uint32_t saved_page0_0 = page0[0];
  const uint32_t saved_page0_1 = page0[1];
  win.Write32(2, 0xDEADBEEFu);
  win.Write32(-4, 0xDEADBEEFu);
  EXPECT_EQ(win.Read32(1), kMmioAllOnes);
  EXPECT_EQ(win.Read32(3), kMmioAllOnes);
  win.Write32(1, 0xDEADBEEFu);
  win.Write32(3, 0xDEADBEEFu);
  EXPECT_EQ(page0[0], saved_page0_0);
  EXPECT_EQ(page0[1], saved_page0_1);

  win.Init(reinterpret_cast<volatile uint8_t*>(page1), 4096);
  EXPECT_EQ(win.Size(), 4096);
  EXPECT_EQ(win.Read32(0), 0xAABBCCDDu);
  EXPECT_FALSE(win.Covers(8192, 4));
  win.Init(nullptr, 0);
  EXPECT_EQ(win.Size(), 0);
  EXPECT_FALSE(win.Covers(0, 4));

  {
    alignas(4) uint32_t hi_page[2] = {};
    volatile uint8_t* const hi_ptr =
        reinterpret_cast<volatile uint8_t*>(hi_page);
    MappedMmioWindow near_max;
    near_max.Init(hi_ptr, 8);
    EXPECT_TRUE(near_max.AddSegment(INT64_MAX - 11, hi_ptr, 8));
    EXPECT_EQ(near_max.Size(), INT64_MAX - 3);
    EXPECT_TRUE(near_max.Covers(INT64_MAX - 11, 8));
    EXPECT_FALSE(near_max.Covers(INT64_MAX - 7, 8));
    EXPECT_FALSE(near_max.AddSegment(INT64_MAX - 7, hi_ptr, 8));
    near_max.Write64(INT64_MAX - 11, 0x0123456789ABCDEFULL);
    EXPECT_EQ(near_max.Read64(INT64_MAX - 11), 0x0123456789ABCDEFULL);
  }
  {
    alignas(4) uint32_t hi4[1] = {};
    volatile uint8_t* const hi4_ptr = reinterpret_cast<volatile uint8_t*>(hi4);
    MappedMmioWindow split_max;
    split_max.Init(hi4_ptr, 4);
    ASSERT_TRUE(split_max.AddSegment(INT64_MAX - 7, hi4_ptr, 4));
    EXPECT_EQ(split_max.Size(), INT64_MAX - 3);
    EXPECT_TRUE(split_max.Covers(INT64_MAX - 7, 4));
    EXPECT_FALSE(split_max.Covers(INT64_MAX - 12, 16));
    split_max.Write64(INT64_MAX - 7, 0xAABBCCDDEEFF0011ULL);
    EXPECT_EQ(hi4[0], 0xEEFF0011u);
    EXPECT_EQ(split_max.Read64(INT64_MAX - 7), 0xFFFFFFFFEEFF0011ULL);
  }

  MappedMmioWindow many;
  many.Init(reinterpret_cast<volatile uint8_t*>(page0), 4096);
  for (int i = 1; i < MappedMmioWindow::kMaxSegments; ++i) {
    EXPECT_TRUE(many.AddSegment(static_cast<int64_t>(i) * 4096,
                                reinterpret_cast<volatile uint8_t*>(page0),
                                4096));
  }
  EXPECT_FALSE(many.AddSegment(
      static_cast<int64_t>(MappedMmioWindow::kMaxSegments) * 4096,  //
      reinterpret_cast<volatile uint8_t*>(page0),                   //
      4096));
}

TEST(NvmePlatformTest, TicksFromMsRoundsUpAddsSlackAndClampsOverflow) {
  EXPECT_EQ(TicksFromMs(0, 250), 0);
  EXPECT_EQ(TicksFromMs(-10, 250), 0);
  EXPECT_EQ(TicksFromMs(100, 0), 0);
  EXPECT_EQ(TicksFromMs(100, -1), 0);

  // At 250 Hz (4 ms/tick), every positive duration includes +1 tick slack.
  EXPECT_EQ(TicksFromMs(1, 250), 2);
  EXPECT_EQ(TicksFromMs(4, 250), 2);
  EXPECT_EQ(TicksFromMs(5, 250), 3);
  EXPECT_EQ(TicksFromMs(5000, 250), 1251);
  EXPECT_EQ(TicksFromMs(127500, 250), 31876);

  EXPECT_EQ(TicksFromMs(1, 1000), 2);
  EXPECT_EQ(TicksFromMs(1, 1250), 3);
  EXPECT_EQ(TicksFromMs(1, 1), 2);
  EXPECT_EQ(TicksFromMs(1, 1001), 3);

  constexpr int64_t kMaxTicks = INT64_MAX / 2;
  EXPECT_EQ(TicksFromMs(INT64_MAX, 250), 2305843009213693953LL);
  EXPECT_EQ(TicksFromMs(INT64_MAX, 1000), kMaxTicks);
  EXPECT_EQ(TicksFromMs(5000, INT64_MAX), kMaxTicks);
  EXPECT_EQ(TicksFromMs(INT64_MAX, INT64_MAX), kMaxTicks);
  EXPECT_GT(TicksFromMs(1, kMaxTicks), 0);
  EXPECT_LT(TicksFromMs(1, kMaxTicks), kMaxTicks);
  EXPECT_EQ(TicksFromMs(1, kMaxTicks), kMaxTicks / 1000 + 2);
  EXPECT_EQ(TicksFromMs(1, kMaxTicks + 1), kMaxTicks);
  EXPECT_EQ(TicksFromMs(1, INT64_MAX), kMaxTicks);
  EXPECT_EQ(TicksFromMs(999, INT64_MAX), kMaxTicks);
  EXPECT_EQ(TicksFromMs((kMaxTicks / 1000) * 1000 + 999, 1000), kMaxTicks);
  EXPECT_EQ(TicksFromMs(4616302320748136LL * 1000 + 500, 999), kMaxTicks);
  EXPECT_EQ(TicksFromMs(4616302320748136LL * 1000 + 39, 999), kMaxTicks);
  EXPECT_EQ(TicksFromMs(4616302320748136LL * 1000, 999),
            4616302320748136LL * 999 + 1);
  EXPECT_EQ(TicksFromMs((kMaxTicks / 4000) * 1000 + 975, 4000), kMaxTicks - 2);
}

class CaptureLogPlatform final : public Platform {
 public:
  std::string captured;
  int captured_length = -1;
  int log_calls = 0;

  DmaBuffer DmaAlloc(const int64_t /*size*/,
                     const bool /*below_4gib*/) override {
    return {};
  }

  int64_t NowTicks() override { return 0; }
  int64_t TicksPerSecond() override { return 1; }
  void Relax() override {}

  void WaitForIrq(const uint64_t /*irq_mask*/,
                  const int64_t /*timeout_ticks*/) override {}

  void Log(const char* const line, const int length) override {
    ++log_calls;
    captured_length = length;
    if (line != nullptr && length > 0) {
      captured.assign(line, line + length);
    } else {
      captured.clear();
    }
  }
};

TEST(NvmePlatformTest, LogLineFormatsTruncatesAndClampsEmit) {
  sim::SimNvmeDevice dev;
  {
    LogLine line;
    line.Str(nullptr).Str("val=").Dec(0).Str(",max=").Dec(UINT64_MAX);
    line.Emit(dev);
    ASSERT_EQ(static_cast<int>(dev.logs.size()), 1);
    EXPECT_EQ(dev.logs[0], "val=0,max=18446744073709551615\n");
  }
  {
    LogLine line;
    const std::string over(200, 'X');
    line.Str(over.c_str());
    line.Emit(dev);
    ASSERT_EQ(static_cast<int>(dev.logs.size()), 2);
    EXPECT_EQ(static_cast<int>(dev.logs[1].size()), kMaxLogLineLength);
    EXPECT_EQ(dev.logs[1].back(), '\n');
    EXPECT_EQ(dev.logs[1].substr(0, kMaxLogLineLength - 1),
              std::string(kMaxLogLineLength - 1, 'X'));

    // Repeated Emit() at kMaxLogLineLength must clamp rather than OOB write.
    line.Emit(dev);
    ASSERT_EQ(static_cast<int>(dev.logs.size()), 3);
    EXPECT_EQ(static_cast<int>(dev.logs[2].size()), kMaxLogLineLength);
    EXPECT_EQ(dev.logs[2].back(), '\n');
  }
  {
    LogLine line;
    const std::string prefix(kMaxLogLineLength - 2, 'A');
    line.Str(prefix.c_str()).Dec(UINT64_MAX);
    line.Emit(dev);
    ASSERT_EQ(static_cast<int>(dev.logs.size()), 4);
    EXPECT_EQ(static_cast<int>(dev.logs[3].size()), kMaxLogLineLength);
    EXPECT_EQ(dev.logs[3], prefix + "1\n");
  }
  {
    CaptureLogPlatform cap;
    LogLine line;
    const std::string part_a(64, 'A');
    const std::string part_b(64, 'B');
    line.Str(part_a.c_str()).Str(part_b.c_str()).Str("CCCCCCCCCC");
    line.Emit(cap);
    ASSERT_EQ(cap.log_calls, 1);
    EXPECT_EQ(cap.captured_length, kMaxLogLineLength);
    EXPECT_EQ(static_cast<int>(cap.captured.size()), kMaxLogLineLength);
    EXPECT_EQ(cap.captured, part_a + std::string(63, 'B') + "\n");
  }
}

TEST(NvmePlatformTest, FormatBdfMasksFieldsAndLowercasesHex) {
  char bdf[8] = {};
  FormatBdf(0x00, 0x04, 0x00, bdf);
  EXPECT_STREQ(bdf, "00:04.0");
  FormatBdf(0xFF, 0x1F, 0x07, bdf);
  EXPECT_STREQ(bdf, "ff:1f.7");
  // Bits outside 5-bit device and 3-bit function are masked cleanly.
  FormatBdf(0x00, 0xFF, 0xFF, bdf);
  EXPECT_STREQ(bdf, "00:1f.7");
  FormatBdf(0x12, 0x03, 0x05, bdf);
  EXPECT_STREQ(bdf, "12:03.5");
  FormatBdf(0xA4, 0x1B, 0x06, bdf);
  EXPECT_STREQ(bdf, "a4:1b.6");
  std::memset(bdf, 'X', sizeof(bdf));
  FormatBdf(0x12, 0x03, 0x05, bdf);
  ASSERT_EQ(bdf[7], '\0');
  EXPECT_STREQ(bdf, "12:03.5");
}

TEST(NvmePlatformTest, ClampAndAlignDmaSizeAndIsNvmePciDevice) {
  EXPECT_EQ(ClampAndAlignDmaSize(0, kSyscallDmaMaxSize), 0);
  EXPECT_EQ(ClampAndAlignDmaSize(-4096, kSyscallDmaMaxSize), 0);
  EXPECT_EQ(ClampAndAlignDmaSize(1, kSyscallDmaMaxSize), 4096);
  EXPECT_EQ(ClampAndAlignDmaSize(4096, kSyscallDmaMaxSize), 4096);
  EXPECT_EQ(ClampAndAlignDmaSize(4097, kSyscallDmaMaxSize), 8192);
  EXPECT_EQ(ClampAndAlignDmaSize(kSyscallDmaMaxSize, kSyscallDmaMaxSize),
            kSyscallDmaMaxSize);
  EXPECT_EQ(ClampAndAlignDmaSize(kSyscallDmaMaxSize + 1, kSyscallDmaMaxSize),
            0);
  EXPECT_EQ(ClampAndAlignDmaSize(1, 4095), 0);
  EXPECT_EQ(ClampAndAlignDmaSize(1, 4096), 4096);
  EXPECT_EQ(ClampAndAlignDmaSize(4096, 4096), 4096);
  EXPECT_EQ(ClampAndAlignDmaSize(4097, 4096), 0);
  EXPECT_EQ(ClampAndAlignDmaSize(4097, 5000), 0);
  EXPECT_EQ(ClampAndAlignDmaSize(INT64_MAX, kSyscallDmaMaxSize), 0);
  EXPECT_EQ(ClampAndAlignDmaSize(INT64_MAX, INT64_MAX), 0);

  PciDeviceInfo valid{};
  valid.claimable = 1;
  valid.class_code = 0x01;
  valid.subclass = 0x08;
  valid.prog_if = 0x02;
  valid.bars[0] = PciBarInfo{
      .type = PciBarInfoType::kMemory64,
      .phys_base = 0xFE000000ULL,
      .size = 0x4000,
  };
  EXPECT_TRUE(IsNvmePciDevice(valid));

  {
    PciDeviceInfo bad = valid;
    bad.bars[0].type = PciBarInfoType::kMemory32;
    EXPECT_TRUE(IsNvmePciDevice(bad));
    bad.bars[0].type = PciBarInfoType::kIo;
    EXPECT_FALSE(IsNvmePciDevice(bad));
    bad.bars[0].type = PciBarInfoType::kNone;
    EXPECT_FALSE(IsNvmePciDevice(bad));
  }
  {
    PciDeviceInfo bad = valid;
    bad.claimable = 0;
    EXPECT_FALSE(IsNvmePciDevice(bad));
  }
  {
    PciDeviceInfo bad = valid;
    bad.class_code = 0x02;
    EXPECT_FALSE(IsNvmePciDevice(bad));
  }
  {
    PciDeviceInfo bad = valid;
    bad.subclass = 0x06;
    EXPECT_FALSE(IsNvmePciDevice(bad));
  }
  {
    PciDeviceInfo bad = valid;
    bad.prog_if = 0x01;
    EXPECT_FALSE(IsNvmePciDevice(bad));
  }
  {
    PciDeviceInfo bad = valid;
    bad.bars[0].phys_base = 0;
    EXPECT_FALSE(IsNvmePciDevice(bad));
  }
  {
    PciDeviceInfo bad = valid;
    bad.bars[0].phys_base = 0xFE001004ULL;
    EXPECT_FALSE(IsNvmePciDevice(bad));
  }
  {
    PciDeviceInfo bad = valid;
    bad.bars[0].phys_base = 0xFE000800ULL;
    EXPECT_FALSE(IsNvmePciDevice(bad));
  }
  for (const uint64_t bad_phys : {0xFE000001ULL, 0xFE000002ULL}) {
    PciDeviceInfo bad = valid;
    bad.bars[0].phys_base = bad_phys;
    EXPECT_FALSE(IsNvmePciDevice(bad));
  }
  {
    PciDeviceInfo bad = valid;
    bad.bars[0].size = 4096;
    EXPECT_FALSE(IsNvmePciDevice(bad));
  }
  {
    PciDeviceInfo bad = valid;
    bad.bars[0].size = 8191;
    EXPECT_FALSE(IsNvmePciDevice(bad));
  }
  {
    PciDeviceInfo min_ok = valid;
    min_ok.bars[0].size = 0x2000;
    EXPECT_TRUE(IsNvmePciDevice(min_ok));
  }
}

TEST(NvmePlatformTest, MapNvmeBar0UsesTwoStepDstrdAndBinarySearch) {
  struct FakeMmioMapper {
    uint64_t phys_base = 0xFE000000ULL;
    int64_t bar_size = 16LL << 20;
    uint64_t cap = (63ULL << 0) | kCapCssNvm;
    std::vector<std::pair<int64_t, int64_t>> holes;
    std::vector<std::vector<uint8_t>> buffers;
    std::vector<int64_t> map_sizes;
    int map_calls = 0;
    int64_t force_error = 0;
    int zero_va_on_call = 0;
    int fail_on_call = 0;
    int64_t fail_on_call_error = 0;

    static int64_t Map(void* const ctx,      //
                       const uint64_t phys,  //
                       const int64_t size,   //
                       uintptr_t* const out_va) {
      FakeMmioMapper* const m = static_cast<FakeMmioMapper*>(ctx);
      ++m->map_calls;
      m->map_sizes.push_back(size);
      if (m->force_error != 0) {
        return m->force_error;
      }
      if (m->fail_on_call > 0 && m->map_calls == m->fail_on_call) {
        return m->fail_on_call_error;
      }
      if (m->zero_va_on_call > 0 && m->map_calls == m->zero_va_on_call) {
        *out_va = 0;
        return 0;
      }
      if (phys < m->phys_base || size <= 0) {
        return kSyscallErrorInvalid;
      }
      const int64_t off = static_cast<int64_t>(phys - m->phys_base);
      if (off + size > m->bar_size) {
        return kSyscallErrorAccess;
      }
      for (const auto& [hs, he] : m->holes) {
        if (off < he && off + size > hs) {
          return kSyscallErrorAccess;
        }
      }
      m->buffers.emplace_back(size, uint8_t{0});
      uint8_t* const ptr = m->buffers.back().data();
      if (off == 0 && size >= 8) {
        const uint32_t lo = static_cast<uint32_t>(m->cap);
        const uint32_t hi = static_cast<uint32_t>(m->cap >> 32);
        std::memcpy(ptr + kRegCap, &lo, 4);
        std::memcpy(ptr + kRegCap + 4, &hi, 4);
      }
      *out_va = reinterpret_cast<uintptr_t>(ptr);
      return 0;
    }
  };

  // DSTRD=0 on a 16 MiB BAR0 should issue only 2 MmioMap calls (0x0000,
  // 0x1000).
  {
    FakeMmioMapper mapper;
    const PciBarInfo bar0{
        .type = PciBarInfoType::kMemory64,
        .phys_base = mapper.phys_base,
        .size = mapper.bar_size,
    };
    MappedMmioWindow win;
    const MmioMapEnv env{.ctx = &mapper, .map = FakeMmioMapper::Map};
    EXPECT_EQ(MapNvmeBar0(bar0, env, &win), 0);
    EXPECT_EQ(mapper.map_calls, 2);
    EXPECT_EQ(win.Size(), 0x2000);
    EXPECT_TRUE(win.Covers(CqHeadDoorbellOffset(1, 0), 4));
  }

  // Page 0 overlapping an MSI-X hole fails immediately in 1 call.
  {
    FakeMmioMapper mapper;
    mapper.holes.push_back({0, 4096});
    const PciBarInfo bar0{
        .type = PciBarInfoType::kMemory64,
        .phys_base = mapper.phys_base,
        .size = mapper.bar_size,
    };
    MappedMmioWindow win;
    const MmioMapEnv env{.ctx = &mapper, .map = FakeMmioMapper::Map};
    EXPECT_EQ(MapNvmeBar0(bar0, env, &win), kSyscallErrorAccess);
    EXPECT_EQ(mapper.map_calls, 1);
  }

  // DSTRD=15 with 2 MSI-X holes maps up to 0x62000 in O(log N) calls (<= 32
  // segments, NOT linear 4096 calls across 16 MiB).
  {
    FakeMmioMapper mapper;
    mapper.cap = (63ULL << 0) | (15ULL << 32) | kCapCssNvm;
    mapper.holes.push_back({0x2000, 0x4000});
    mapper.holes.push_back({0x30000, 0x31000});
    const PciBarInfo bar0{
        .type = PciBarInfoType::kMemory64,
        .phys_base = mapper.phys_base,
        .size = mapper.bar_size,
    };
    MappedMmioWindow win;
    const MmioMapEnv env{.ctx = &mapper, .map = FakeMmioMapper::Map};
    EXPECT_EQ(MapNvmeBar0(bar0, env, &win), 0);
    EXPECT_LE(mapper.map_calls, 48);
    EXPECT_EQ(win.Size(), 0x62000);
    EXPECT_TRUE(win.Covers(SqTailDoorbellOffset(0, 15), 4));
    EXPECT_TRUE(win.Covers(CqHeadDoorbellOffset(0, 15), 4));
    EXPECT_TRUE(win.Covers(SqTailDoorbellOffset(1, 15), 4));
    EXPECT_TRUE(win.Covers(CqHeadDoorbellOffset(1, 15), 4));
    EXPECT_FALSE(win.Covers(0x2000, 4));
    EXPECT_FALSE(win.Covers(0x30000, 4));
  }

  // Half-all-ones CAP (either high or low 32 bits) returns page 0 only without
  // failing MapNvmeBar0.
  for (const uint64_t half_ones_cap :
       {0xFFFFFFFF00000000ULL, 0x00000000FFFFFFFFULL}) {
    FakeMmioMapper mapper;
    mapper.cap = half_ones_cap;
    const PciBarInfo bar0{
        .type = PciBarInfoType::kMemory32,
        .phys_base = mapper.phys_base,
        .size = mapper.bar_size,
    };
    MappedMmioWindow win;
    const MmioMapEnv env{.ctx = &mapper, .map = FakeMmioMapper::Map};
    EXPECT_EQ(MapNvmeBar0(bar0, env, &win), 0);
    EXPECT_EQ(mapper.map_calls, 1);
    EXPECT_EQ(win.Size(), 4096);
  }

  // Null args, invalid BAR geometry/max_mmio_size, overflow, and errors.
  {
    FakeMmioMapper mapper;
    const MmioMapEnv env{.ctx = &mapper, .map = FakeMmioMapper::Map};
    MappedMmioWindow win;
    const PciBarInfo ok_bar{
        .type = PciBarInfoType::kMemory32,
        .phys_base = mapper.phys_base,
        .size = 0x4000,
    };
    const MmioMapEnv null_env{};
    EXPECT_EQ(MapNvmeBar0(ok_bar, null_env, &win), kSyscallErrorInvalid);
    EXPECT_EQ(MapNvmeBar0(ok_bar, env, nullptr), kSyscallErrorInvalid);
    EXPECT_EQ(MapNvmeBar0(ok_bar, env, &win, 4096), kSyscallErrorInvalid);
    EXPECT_EQ(MapNvmeBar0(ok_bar, env, &win, 0), kSyscallErrorInvalid);
    EXPECT_EQ(MapNvmeBar0(ok_bar, env, &win, -1), kSyscallErrorInvalid);
    const PciBarInfo high_phys_bar{
        .type = PciBarInfoType::kMemory64,
        .phys_base = 0x9000000000000000ULL,
        .size = 0x4000,
    };
    EXPECT_EQ(MapNvmeBar0(high_phys_bar, env, &win, -4096),
              kSyscallErrorInvalid);
    EXPECT_EQ(mapper.map_calls, 0);

    const PciBarInfo zero_phys{
        .type = PciBarInfoType::kMemory64,
        .phys_base = 0,
        .size = 0x4000,
    };
    EXPECT_EQ(MapNvmeBar0(zero_phys, env, &win), kSyscallErrorInvalid);
    const PciBarInfo unaligned_phys{
        .type = PciBarInfoType::kMemory64,
        .phys_base = 0xFE001004ULL,
        .size = 0x4000,
    };
    EXPECT_EQ(MapNvmeBar0(unaligned_phys, env, &win), kSyscallErrorInvalid);
    const PciBarInfo half_page_phys{
        .type = PciBarInfoType::kMemory64,
        .phys_base = 0xFE000800ULL,
        .size = 0x4000,
    };
    EXPECT_EQ(MapNvmeBar0(half_page_phys, env, &win), kSyscallErrorInvalid);
    for (const uint64_t bad_phys : {0xFE000001ULL, 0xFE000002ULL}) {
      const PciBarInfo sub_dword_phys{
          .type = PciBarInfoType::kMemory64,
          .phys_base = bad_phys,
          .size = 0x4000,
      };
      EXPECT_EQ(MapNvmeBar0(sub_dword_phys, env, &win), kSyscallErrorInvalid);
    }
    EXPECT_EQ(mapper.map_calls, 0);
    const PciBarInfo tiny_bar{
        .type = PciBarInfoType::kMemory64,
        .phys_base = mapper.phys_base,
        .size = 4096,
    };
    EXPECT_EQ(MapNvmeBar0(tiny_bar, env, &win), kSyscallErrorInvalid);

    const PciBarInfo wrap_bar{
        .type = PciBarInfoType::kMemory64,
        .phys_base = 0xFFFFFFFFFFFFF000ULL,
        .size = 0x4000,
    };
    EXPECT_EQ(MapNvmeBar0(wrap_bar, env, &win), kSyscallErrorInvalid);
    EXPECT_EQ(mapper.map_calls, 0);

    const PciBarInfo top_one_page_bar{
        .type = PciBarInfoType::kMemory64,
        .phys_base = 0xFFFFFFFFFFFFE000ULL,
        .size = 0x2000,
    };
    EXPECT_EQ(MapNvmeBar0(top_one_page_bar, env, &win), kSyscallErrorInvalid);
    EXPECT_EQ(mapper.map_calls, 0);

    const PciBarInfo bad_type{
        .type = PciBarInfoType::kIo,
        .phys_base = 0xFE000000ULL,
        .size = 0x4000,
    };
    EXPECT_EQ(MapNvmeBar0(bad_type, env, &win), kSyscallErrorInvalid);
    const PciBarInfo none_type{
        .type = PciBarInfoType::kNone,
        .phys_base = 0xFE000000ULL,
        .size = 0x4000,
    };
    EXPECT_EQ(MapNvmeBar0(none_type, env, &win), kSyscallErrorInvalid);
    const PciBarInfo size_8191{
        .type = PciBarInfoType::kMemory64,
        .phys_base = mapper.phys_base,
        .size = 8191,
    };
    EXPECT_EQ(MapNvmeBar0(size_8191, env, &win), kSyscallErrorInvalid);
    const PciBarInfo neg_high_phys{
        .type = PciBarInfoType::kMemory64,
        .phys_base = 0x9000000000000000ULL,
        .size = -4096,
    };
    EXPECT_EQ(MapNvmeBar0(neg_high_phys, env, &win), kSyscallErrorInvalid);
    EXPECT_EQ(mapper.map_calls, 0);

    mapper.force_error = kSyscallErrorNoMemory;
    EXPECT_EQ(MapNvmeBar0(ok_bar, env, &win), kSyscallErrorNoMemory);

    alignas(4) uint32_t stale_page[1024] = {};
    win.Init(reinterpret_cast<volatile uint8_t*>(stale_page), 4096);
    ASSERT_EQ(win.Size(), 4096);
    EXPECT_EQ(MapNvmeBar0(zero_phys, env, &win), kSyscallErrorInvalid);
    EXPECT_EQ(win.Size(), 0);
    EXPECT_FALSE(win.Covers(0, 4));

    win.Init(reinterpret_cast<volatile uint8_t*>(stale_page), 4096);
    ASSERT_EQ(win.Size(), 4096);
    EXPECT_EQ(MapNvmeBar0(ok_bar, env, &win), kSyscallErrorNoMemory);
    EXPECT_EQ(win.Size(), 0);
    EXPECT_FALSE(win.Covers(0, 4));
  }

  // Exact 2-page (0x2000) BAR0 boundary succeeds in a fresh mapper scope.
  {
    FakeMmioMapper mapper;
    mapper.bar_size = 0x2000;
    const PciBarInfo min_ok_bar{
        .type = PciBarInfoType::kMemory64,
        .phys_base = mapper.phys_base,
        .size = 0x2000,
    };
    MappedMmioWindow win;
    const MmioMapEnv env{.ctx = &mapper, .map = FakeMmioMapper::Map};
    EXPECT_EQ(MapNvmeBar0(min_ok_bar, env, &win), 0);
    EXPECT_EQ(mapper.map_calls, 2);
    EXPECT_EQ(win.Size(), 0x2000);
    EXPECT_TRUE(win.Covers(CqHeadDoorbellOffset(1, 0), 4));
  }

  // Clamp bar0_limit to default max_mmio_size (32 MiB > 16 MiB) and W39
  // (DSTRD=15 with explicit max_mmio_size = 0x2000 < required_end = 0x62000).
  {
    FakeMmioMapper mapper;
    mapper.bar_size = 32LL << 20;
    const PciBarInfo big_bar{
        .type = PciBarInfoType::kMemory64,
        .phys_base = mapper.phys_base,
        .size = mapper.bar_size,
    };
    MappedMmioWindow win;
    const MmioMapEnv env{.ctx = &mapper, .map = FakeMmioMapper::Map};
    EXPECT_EQ(MapNvmeBar0(big_bar, env, &win), 0);
    EXPECT_EQ(mapper.map_calls, 2);
    EXPECT_EQ(win.Size(), 0x2000);
  }
  {
    FakeMmioMapper mapper;
    mapper.bar_size = 0x4000;
    mapper.cap = (63ULL << 0) | (15ULL << 32) | kCapCssNvm;
    const PciBarInfo bar0{
        .type = PciBarInfoType::kMemory64,
        .phys_base = mapper.phys_base,
        .size = mapper.bar_size,
    };
    MappedMmioWindow win;
    const MmioMapEnv env{.ctx = &mapper, .map = FakeMmioMapper::Map};
    EXPECT_EQ(MapNvmeBar0(bar0, env, &win, 0x2000), 0);
    EXPECT_EQ(mapper.map_calls, 2);
    EXPECT_EQ(win.Size(), 0x2000);
    EXPECT_TRUE(win.Covers(0x1000, 4));
    EXPECT_FALSE(win.Covers(0x2000, 4));
  }

  // W40 (va0 == 0 fails AddSegment), W54 (Step-2 non-access error), and Step-2
  // null VA AddSegment failure.
  {
    FakeMmioMapper mapper;
    mapper.zero_va_on_call = 1;
    const PciBarInfo bar0{
        .type = PciBarInfoType::kMemory32,
        .phys_base = mapper.phys_base,
        .size = 0x4000,
    };
    MappedMmioWindow win;
    const MmioMapEnv env{.ctx = &mapper, .map = FakeMmioMapper::Map};
    EXPECT_EQ(MapNvmeBar0(bar0, env, &win), kSyscallErrorInvalid);
    EXPECT_EQ(mapper.map_calls, 1);
    EXPECT_EQ(win.Size(), 0);
  }
  {
    FakeMmioMapper mapper;
    mapper.fail_on_call = 2;
    mapper.fail_on_call_error = kSyscallErrorNoMemory;
    const PciBarInfo bar0{
        .type = PciBarInfoType::kMemory32,
        .phys_base = mapper.phys_base,
        .size = 0x4000,
    };
    MappedMmioWindow win;
    const MmioMapEnv env{.ctx = &mapper, .map = FakeMmioMapper::Map};
    EXPECT_EQ(MapNvmeBar0(bar0, env, &win), kSyscallErrorNoMemory);
    EXPECT_EQ(mapper.map_calls, 2);
  }
  // Page-align bar0_limit down even when DSTRD=15 requests > 0x2000.
  {
    FakeMmioMapper mapper;
    mapper.phys_base = 0xFFFFFFFFFFFFD000ULL;
    mapper.bar_size = 0x4000;
    mapper.cap = (63ULL << 0) | (15ULL << 32) | kCapCssNvm;
    const PciBarInfo bar0{
        .type = PciBarInfoType::kMemory64,
        .phys_base = mapper.phys_base,
        .size = mapper.bar_size,
    };
    MappedMmioWindow win;
    const MmioMapEnv env{.ctx = &mapper, .map = FakeMmioMapper::Map};
    EXPECT_EQ(MapNvmeBar0(bar0, env, &win), 0);
    EXPECT_EQ(win.Size(), 0x2000);
    EXPECT_THAT(mapper.map_sizes, t::ElementsAre(0x1000, 0x1000));
  }
  {
    FakeMmioMapper mapper;
    mapper.bar_size = 0x2800;
    mapper.cap = (63ULL << 0) | (15ULL << 32) | kCapCssNvm;
    const PciBarInfo bar0{
        .type = PciBarInfoType::kMemory64,
        .phys_base = mapper.phys_base,
        .size = mapper.bar_size,
    };
    MappedMmioWindow win;
    const MmioMapEnv env{.ctx = &mapper, .map = FakeMmioMapper::Map};
    EXPECT_EQ(MapNvmeBar0(bar0, env, &win), 0);
    EXPECT_EQ(win.Size(), 0x2000);
    EXPECT_THAT(mapper.map_sizes, t::ElementsAre(0x1000, 0x1000));
  }
  {
    FakeMmioMapper mapper;
    mapper.bar_size = 0x4000;
    mapper.cap = (63ULL << 0) | (15ULL << 32) | kCapCssNvm;
    const PciBarInfo bar0{
        .type = PciBarInfoType::kMemory64,
        .phys_base = mapper.phys_base,
        .size = mapper.bar_size,
    };
    MappedMmioWindow win;
    const MmioMapEnv env{.ctx = &mapper, .map = FakeMmioMapper::Map};
    EXPECT_EQ(MapNvmeBar0(bar0, env, &win, 0x2FFF), 0);
    EXPECT_EQ(win.Size(), 0x2000);
    EXPECT_THAT(mapper.map_sizes, t::ElementsAre(0x1000, 0x1000));
  }
  {
    FakeMmioMapper mapper;
    mapper.zero_va_on_call = 2;
    const PciBarInfo bar0{
        .type = PciBarInfoType::kMemory32,
        .phys_base = mapper.phys_base,
        .size = 0x4000,
    };
    MappedMmioWindow win;
    const MmioMapEnv env{.ctx = &mapper, .map = FakeMmioMapper::Map};
    EXPECT_EQ(MapNvmeBar0(bar0, env, &win), 0);
    EXPECT_EQ(mapper.map_calls, 2);
    EXPECT_EQ(win.Size(), 4096);
    EXPECT_FALSE(win.Covers(0x1000, 4));
  }
  // Binary halving must reach 1-page (4 KiB) granularity on a 2-page tail.
  {
    FakeMmioMapper mapper;
    mapper.bar_size = 0x3000;
    mapper.cap = (63ULL << 0) | (15ULL << 32) | kCapCssNvm;
    mapper.holes.push_back({0x2000, 0x3000});
    const PciBarInfo bar0{
        .type = PciBarInfoType::kMemory64,
        .phys_base = mapper.phys_base,
        .size = mapper.bar_size,
    };
    MappedMmioWindow win;
    const MmioMapEnv env{.ctx = &mapper, .map = FakeMmioMapper::Map};
    EXPECT_EQ(MapNvmeBar0(bar0, env, &win), 0);
    EXPECT_EQ(win.Size(), 0x2000);
    EXPECT_TRUE(win.Covers(0x1000, 4));
    EXPECT_FALSE(win.Covers(0x2000, 4));
    EXPECT_THAT(mapper.map_sizes,
                t::ElementsAre(0x1000, 0x2000, 0x1000, 0x1000));
  }
}

// --- SimNvmeDeviceTest -------------------------------------------------------

TEST(SimNvmeDeviceTest, PhysToPtrAndPrp1RejectNegativeOversizedAndCrossPage) {
  sim::SimNvmeDevice dev;
  const DmaBuffer buf = dev.DmaAlloc(4096, /*below_4gib=*/false);
  ASSERT_NE(buf.va, nullptr);
  EXPECT_EQ(dev.PhysToPtr(buf.phys, -512), nullptr);
  EXPECT_EQ(dev.PhysToPtr(buf.phys, 0), nullptr);
  EXPECT_EQ(dev.PhysToPtr(buf.phys, 8192), nullptr);
  EXPECT_EQ(dev.PhysToPtr(0, 512), nullptr);
  EXPECT_EQ(dev.PhysToPtr(buf.phys, 4096), buf.va);

  const DmaBuffer buf8k = dev.DmaAlloc(8192, /*below_4gib=*/false);
  ASSERT_NE(buf8k.va, nullptr);
  EXPECT_NE(dev.PhysToPtr(buf8k.phys + 2048, 4096), nullptr);
  EXPECT_EQ(
      dev.Prp1ToPtr(buf8k.phys + 2048, 4096, /*require_page_aligned=*/false),
      nullptr);
  EXPECT_EQ(
      dev.Prp1ToPtr(buf.phys + 2048, 4096, /*require_page_aligned=*/false),
      nullptr);
  EXPECT_EQ(dev.Prp1ToPtr(buf.phys + 4, 512, /*require_page_aligned=*/true),
            nullptr);
  EXPECT_NE(dev.Prp1ToPtr(buf.phys + 2048, 2048, false), nullptr);
}

TEST(SimNvmeDeviceTest, DmaAllocValidatesPageAlignmentAndBounds) {
  sim::SimNvmeDevice dev;
  EXPECT_EQ(dev.DmaAlloc(0, false).va, nullptr);
  EXPECT_EQ(dev.DmaAlloc(-4096, false).va, nullptr);
  EXPECT_EQ(dev.DmaAlloc(100, false).va, nullptr);
  EXPECT_EQ(dev.DmaAlloc(2LL << 20, false).va, nullptr);

  const DmaBuffer buf = dev.DmaAlloc(4096, true);
  ASSERT_NE(buf.va, nullptr);
  EXPECT_EQ(buf.phys & 0xFFFULL, 0u);
  EXPECT_EQ(reinterpret_cast<uintptr_t>(buf.va) & 0xFFFULL, 0u);
  EXPECT_LT(buf.phys, 0x100000000ULL);
}

TEST(SimNvmeDeviceTest, DoorbellOobAndCstsCfsDoNotHangAndStopProcessing) {
  sim::SimNvmeDevice dev;
  NvmeController ctrl;
  ctrl.Init(dev, dev);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  ASSERT_TRUE(dev.io_cq_created);
  ASSERT_TRUE(dev.io_sq_created);

  // Out-of-bounds Admin and I/O doorbells must set CSTS.CFS without hanging.
  dev.Write32(SqTailDoorbellOffset(0, 0), 999u);
  EXPECT_NE(dev.csts & kCstsCfs, 0u);

  const int admin_before = dev.admin_commands_seen;
  const uint32_t next_admin_tail =
      static_cast<uint32_t>((dev.asq_head + 1) % 32);
  dev.Write32(SqTailDoorbellOffset(0, 0), next_admin_tail);
  EXPECT_EQ(dev.admin_commands_seen, admin_before);

  EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kControllerFatal);
  EXPECT_FALSE(dev.saw_shn_normal);
  EXPECT_FALSE(dev.io_cq_created);
  EXPECT_FALSE(dev.io_sq_created);

  // Also test OOB I/O doorbell directly on a healthy running controller.
  dev.ResetRegisters(/*initially_enabled=*/false);
  ctrl.Init(dev, dev);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  dev.Write32(SqTailDoorbellOffset(1, 0), 999u);
  EXPECT_NE(dev.csts & kCstsCfs, 0u);
  const int io_before = dev.io_commands_seen;
  const uint32_t next_io_tail =
      static_cast<uint32_t>((dev.io_sq_head + 1) % 32);
  dev.Write32(SqTailDoorbellOffset(1, 0), next_io_tail);
  EXPECT_EQ(dev.io_commands_seen, io_before);
}

TEST(SimNvmeDeviceTest, Cns2FiltersByNsidAndSortsAscendingAndCoversOverflow) {
  sim::SimNvmeDevice dev;
  dev.namespaces.clear();
  for (const uint32_t id : {20u, 5u, 1u, 0xFFFFFFFFu}) {
    sim::SimNvmeDevice::NamespaceConfig ns;
    ns.nsid = id;
    ns.block_count = 32;
    ns.block_size = 512;
    ns.InitStorage();
    dev.namespaces.push_back(std::move(ns));
  }

  EXPECT_FALSE(dev.Covers(INT64_MAX - 2, 8));

  NvmeController ctrl;
  ctrl.Init(dev, dev);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.namespace_count(), 3);
  EXPECT_EQ(ctrl.ns(0).nsid, 1u);
  EXPECT_EQ(ctrl.ns(1).nsid, 5u);
  EXPECT_EQ(ctrl.ns(2).nsid, 20u);
}

// Lightweight, fast 4 KiB-aligned NVMe MMIO/DMA host fake for testing
// `UINT64_MAX` namespaces without `InitStorage()`, custom `CNS=2` / `NSZE` /
// `NCAP` layouts, `read_only` RO protection, fast 65536-CID wraparound, and
// Admin queue creation failures.
class FakeNvmeHost final : public Platform, public MmioWindow {
 public:
  uint64_t cap = (63ULL << 0) | (2ULL << 24) | kCapCssNvm;
  uint64_t nsze = UINT64_MAX;
  uint64_t ncap = UINT64_MAX;
  uint32_t nn_override = 0;
  uint8_t lbads = 9;
  uint8_t vwc = 1;
  bool read_only = false;
  bool advertise_nsattr_ro = true;
  bool nul_pad_strings = false;
  bool support_cns2 = true;
  bool fail_cns2_after_page1 = false;
  bool fail_cns0_with_cfs = false;
  uint8_t fail_admin_opcode = 0;
  std::function<void(uint32_t cursor, uint8_t* out)> custom_cns2;
  bool force_zero_dma_phys = false;
  uint64_t dma_phys_offset = 0;
  int64_t override_dma_size = 0;
  bool force_nn_zero = false;
  bool timeout_cns2_once = false;
  uint16_t cns2_page1_status = 0;
  uint16_t inactive_ns_status =
      static_cast<uint16_t>(kStatusCodeInvalidField << 1);
  std::vector<uint32_t> present_nsids;

  int io_db_writes = 0;
  bool saw_zero_cid = false;
  bool saw_cid_65535 = false;
  bool saw_wrap_to_1 = false;
  uint16_t last_io_cid = 0;
  uint8_t last_io_opc = 0;
  uint32_t last_io_nsid = 0;
  uint64_t last_io_lba = 0;
  uint32_t last_create_iocq_cdw11 = 0;
  uint32_t last_create_iosq_cdw11 = 0;
  uint32_t cc_reg = 0;
  std::vector<uint32_t> cns2_cursors;
  uint32_t max_probed_nsid = 0;
  int set_features_count = 0;
  std::vector<uint32_t> set_features_cdw11;

  static void WriteLe32(uint8_t* const p, const uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
  }

  static void WriteLe64(uint8_t* const p, const uint64_t v) {
    WriteLe32(p, static_cast<uint32_t>(v));
    WriteLe32(p + 4, static_cast<uint32_t>(v >> 32));
  }

  DmaBuffer DmaAlloc(const int64_t size, const bool /*below_4gib*/) override {
    if (size != 4096) {
      return {};
    }
    const int idx = static_cast<int>(pages_.size());
    pages_.push_back(std::make_unique<AlignedPage>());
    const uint64_t phys =
        force_zero_dma_phys
            ? 0ULL
            : (kBasePhys + static_cast<uint64_t>(idx) * 4096ULL +
               dma_phys_offset);
    const int64_t ret_size = (override_dma_size > 0) ? override_dma_size : size;
    return DmaBuffer{pages_.back()->bytes, phys, ret_size};
  }

  int64_t NowTicks() override { return now_ticks_; }
  int64_t TicksPerSecond() override { return 250; }

  void Relax() override {
    ++now_ticks_;
    if ((cc_reg & kCcEn) != 0) {
      csts_reg_ |= kCstsRdy;
    } else {
      csts_reg_ &= ~kCstsRdy;
    }
    if ((cc_reg & kCcEn) != 0 && (cc_reg & kCcShnMask) != 0) {
      csts_reg_ = (csts_reg_ & ~kCstsShstMask) | kCstsShstComplete;
    }
  }

  void WaitForIrq(const uint64_t /*irq_mask*/,
                  const int64_t timeout_ticks) override {
    now_ticks_ += (timeout_ticks > 0) ? timeout_ticks : 1;
  }

  void Log(const char* const /*line*/, const int /*length*/) override {}

  int64_t Size() const override { return 0x2000; }

  bool Covers(const int64_t offset, const int64_t size) const override {
    return offset >= 0 && size > 0 && size <= 0x2000 && offset <= 0x2000 - size;
  }

  uint32_t Read32(const int64_t offset) override {
    switch (offset) {
      case kRegCap:
        return static_cast<uint32_t>(cap);
      case kRegCap + 4:
        return static_cast<uint32_t>(cap >> 32);
      case kRegCc:
        return cc_reg;
      case kRegCsts:
        return csts_reg_;
      case kRegAqa:
        return aqa_reg_;
      case kRegAsq:
        return static_cast<uint32_t>(asq_reg_);
      case kRegAsq + 4:
        return static_cast<uint32_t>(asq_reg_ >> 32);
      case kRegAcq:
        return static_cast<uint32_t>(acq_reg_);
      case kRegAcq + 4:
        return static_cast<uint32_t>(acq_reg_ >> 32);
      default:
        return 0;
    }
  }

  void Write32(const int64_t offset, const uint32_t value) override {
    switch (offset) {
      case kRegCc: {
        const bool was_en = (cc_reg & kCcEn) != 0;
        const bool now_en = (value & kCcEn) != 0;
        cc_reg = value;
        if (was_en && !now_en) {
          csts_reg_ &= ~(kCstsRdy | kCstsCfs | kCstsShstMask);
          admin_sq_head_ = 0;
          admin_cq_.tail = 0;
          admin_cq_.phase = 1;
          io_sq_head_ = 0;
          io_cq_.tail = 0;
          io_cq_.phase = 1;
        } else if (!was_en && now_en) {
          admin_sq_va_ = PhysToPtr(asq_reg_);
          admin_cq_.va = PhysToPtr(acq_reg_);
          csts_reg_ |= kCstsRdy;
        }
        if (now_en && (value & kCcShnMask) != 0) {
          csts_reg_ = (csts_reg_ & ~kCstsShstMask) | kCstsShstComplete;
        }
        return;
      }
      case kRegAqa:
        aqa_reg_ = value;
        admin_sq_depth_ = static_cast<int>(value & 0xFFFu) + 1;
        admin_cq_.depth = static_cast<int>((value >> 16) & 0xFFFu) + 1;
        return;
      case kRegAsq:
        asq_reg_ = (asq_reg_ & ~0xFFFFFFFFULL) | value;
        return;
      case kRegAsq + 4:
        asq_reg_ =
            (asq_reg_ & 0xFFFFFFFFULL) | (static_cast<uint64_t>(value) << 32);
        return;
      case kRegAcq:
        acq_reg_ = (acq_reg_ & ~0xFFFFFFFFULL) | value;
        return;
      case kRegAcq + 4:
        acq_reg_ =
            (acq_reg_ & 0xFFFFFFFFULL) | (static_cast<uint64_t>(value) << 32);
        return;
      default:
        break;
    }
    if (offset == SqTailDoorbellOffset(0, 0)) {
      ProcessAdmin(static_cast<int>(value));
    } else if (offset == SqTailDoorbellOffset(1, 0)) {
      ProcessIo(static_cast<int>(value));
    }
  }

 private:
  static constexpr uint64_t kBasePhys = 0x100000000ULL;

  struct alignas(4096) AlignedPage {
    uint8_t bytes[4096] = {};
  };

  struct CqState {
    uint8_t* va = nullptr;
    int depth = 0;
    int tail = 0;
    uint16_t phase = 1;
  };

  uint8_t* PhysToPtr(const uint64_t phys) const {
    if (phys < kBasePhys || (phys & 0xFFFULL) != 0) {
      return nullptr;
    }
    const uint64_t page_idx = (phys - kBasePhys) >> 12;
    if (page_idx > static_cast<uint64_t>(INT32_MAX)) {
      return nullptr;
    }
    const int idx = static_cast<int>(page_idx);
    if (idx < 0 || idx >= static_cast<int>(pages_.size())) {
      return nullptr;
    }
    return pages_[idx]->bytes;
  }

  static void PostCqe(CqState& cq, const CompletionEntry& cqe) {
    if (cq.va == nullptr || cq.depth <= 0 || cq.depth > 256) {
      return;
    }
    CompletionEntry stamped = cqe;
    stamped.status = static_cast<uint16_t>(cqe.status | cq.phase);
    std::memcpy(cq.va + cq.tail * 16, &stamped, 16);
    cq.tail = (cq.tail + 1) % cq.depth;
    if (cq.tail == 0) {
      cq.phase ^= 1u;
    }
  }

  void ProcessAdmin(const int new_tail) {
    if (admin_sq_va_ == nullptr || admin_sq_depth_ <= 0 ||
        admin_sq_depth_ > 64 || new_tail < 0 || new_tail >= admin_sq_depth_) {
      return;
    }
    while (admin_sq_head_ != new_tail) {
      SubmissionEntry cmd{};
      std::memcpy(&cmd, admin_sq_va_ + admin_sq_head_ * 64, 64);
      admin_sq_head_ = (admin_sq_head_ + 1) % admin_sq_depth_;
      const uint8_t opc = static_cast<uint8_t>(cmd.cdw0 & 0xFFu);
      const uint16_t cid = static_cast<uint16_t>(cmd.cdw0 >> 16);
      uint16_t status_field = 0;
      if (fail_admin_opcode != 0 && opc == fail_admin_opcode) {
        status_field = static_cast<uint16_t>(1u << 9);
      } else if (opc == kAdminIdentify) {
        uint8_t* const out = PhysToPtr(cmd.prp1);
        if (out == nullptr) {
          status_field = static_cast<uint16_t>(kStatusCodeInvalidField << 1);
        } else {
          std::memset(out, 0, kIdentifyDataBytes);
          const uint32_t cns = cmd.cdw10 & 0xFFu;
          if (cns == kIdentifyCnsController) {
            if (nul_pad_strings) {
              static constexpr char kNulSerial[kIdCtrlSerialLength] = "NULSN";
              static constexpr char kNulModel[kIdCtrlModelLength] = "NulModel";
              std::memcpy(out + kIdCtrlSerialOffset,  //
                          kNulSerial,                 //
                          kIdCtrlSerialLength);
              std::memcpy(out + kIdCtrlModelOffset,  //
                          kNulModel,                 //
                          kIdCtrlModelLength);
            } else {
              static constexpr char kSerial[] = "HUGE001             ";
              static constexpr char kModel[] =
                  "HugeFake                                ";
              std::memcpy(out + kIdCtrlSerialOffset,  //
                          kSerial,                    //
                          kIdCtrlSerialLength);
              std::memcpy(out + kIdCtrlModelOffset, kModel, kIdCtrlModelLength);
            }
            out[kIdCtrlMdtsOffset] = 5;
            uint32_t default_nn = 1u;
            if (!present_nsids.empty()) {
              default_nn = 0u;
              for (const uint32_t id : present_nsids) {
                if (id != 0xFFFFFFFFu && id > default_nn) {
                  default_nn = id;
                }
              }
            }
            const uint32_t reported_nn =
                force_nn_zero ? 0u
                              : ((nn_override > 0) ? nn_override : default_nn);
            WriteLe32(out + kIdCtrlNnOffset, reported_nn);
            out[kIdCtrlVwcOffset] = vwc;
          } else if (cns == kIdentifyCnsActiveNsList) {
            cns2_cursors.push_back(cmd.nsid);
            if (cmd.nsid == 0 && timeout_cns2_once) {
              timeout_cns2_once = false;
              return;
            }
            if (cmd.nsid == 0 && cns2_page1_status != 0) {
              status_field = cns2_page1_status;
            } else if (!support_cns2 ||
                       (fail_cns2_after_page1 && cmd.nsid > 0)) {
              status_field =
                  static_cast<uint16_t>(kStatusCodeInvalidField << 1);
            } else if (custom_cns2) {
              custom_cns2(cmd.nsid, out);
            } else if (cmd.nsid == 0) {
              if (present_nsids.empty()) {
                WriteLe32(out, 1u);
              } else {
                int out_idx = 0;
                for (const uint32_t id : present_nsids) {
                  if (out_idx < 1024 && id != 0 && id != 0xFFFFFFFFu) {
                    WriteLe32(out + 4 * out_idx, id);
                    ++out_idx;
                  }
                }
              }
            }
          } else if (cns == kIdentifyCnsNamespace) {
            if (cmd.nsid > max_probed_nsid) {
              max_probed_nsid = cmd.nsid;
            }
            if (fail_cns0_with_cfs) {
              csts_reg_ |= kCstsCfs;
              return;
            }
            bool present = false;
            if (present_nsids.empty()) {
              present = (cmd.nsid == 1u);
            } else {
              for (const uint32_t id : present_nsids) {
                if (id == cmd.nsid) {
                  present = true;
                  break;
                }
              }
            }
            if (!present) {
              status_field = inactive_ns_status;
            } else {
              WriteLe64(out + kIdNsSizeOffset, nsze);
              WriteLe64(out + kIdNsCapOffset, ncap);
              out[kIdNsAttrOffset] = (read_only && advertise_nsattr_ro)
                                         ? kIdNsAttrWriteProtected
                                         : 0;
              const uint32_t lbaf0 = static_cast<uint32_t>(lbads) << 16;
              WriteLe32(out + kIdNsLbaf0Offset, lbaf0);
            }
          }
        }
      } else if (opc == kAdminSetFeatures) {
        ++set_features_count;
        if (cmd.cdw10 == kFeatureNumberOfQueues) {
          set_features_cdw11.push_back(cmd.cdw11);
        }
      } else if (opc == kAdminCreateIoCq) {
        last_create_iocq_cdw11 = cmd.cdw11;
        io_cq_.va = PhysToPtr(cmd.prp1);
        io_cq_.depth = static_cast<int>((cmd.cdw10 >> 16) & 0xFFFFu) + 1;
        io_cq_.tail = 0;
        io_cq_.phase = 1;
      } else if (opc == kAdminCreateIoSq) {
        last_create_iosq_cdw11 = cmd.cdw11;
        io_sq_va_ = PhysToPtr(cmd.prp1);
        io_sq_depth_ = static_cast<int>((cmd.cdw10 >> 16) & 0xFFFFu) + 1;
        io_sq_head_ = 0;
      }
      const CompletionEntry cqe{
          .dw0 = 0,
          .dw1 = 0,
          .sq_head = static_cast<uint16_t>(admin_sq_head_),
          .sq_id = 0,
          .cid = cid,
          .status = status_field,
      };
      PostCqe(admin_cq_, cqe);
    }
  }

  void ProcessIo(const int new_tail) {
    ++io_db_writes;
    if ((cc_reg & kCcEn) == 0 || (csts_reg_ & kCstsRdy) == 0 ||
        (csts_reg_ & kCstsCfs) != 0 || io_sq_va_ == nullptr ||
        io_sq_depth_ <= 0 || io_sq_depth_ > 64 || new_tail < 0 ||
        new_tail >= io_sq_depth_) {
      return;
    }
    while (io_sq_head_ != new_tail) {
      SubmissionEntry cmd{};
      std::memcpy(&cmd, io_sq_va_ + io_sq_head_ * 64, 64);
      io_sq_head_ = (io_sq_head_ + 1) % io_sq_depth_;
      const uint8_t opc = static_cast<uint8_t>(cmd.cdw0 & 0xFFu);
      const uint16_t cid = static_cast<uint16_t>(cmd.cdw0 >> 16);
      if (cid == 0) {
        saw_zero_cid = true;
      }
      if (last_io_cid == 65535 && cid == 1) {
        saw_wrap_to_1 = true;
      }
      if (cid == 65535) {
        saw_cid_65535 = true;
      }
      last_io_cid = cid;
      last_io_opc = opc;
      last_io_nsid = cmd.nsid;
      last_io_lba = static_cast<uint64_t>(cmd.cdw10) |
                    (static_cast<uint64_t>(cmd.cdw11) << 32);
      uint16_t status_field = 0;
      if (read_only && opc == kNvmWrite) {
        status_field =
            static_cast<uint16_t>((kStatusTypeCommandSpecific << 9) |
                                  (kStatusCodeWriteToReadOnlyRange << 1));
      }
      const CompletionEntry cqe{
          .dw0 = 0,
          .dw1 = 0,
          .sq_head = static_cast<uint16_t>(io_sq_head_),
          .sq_id = 1,
          .cid = cid,
          .status = status_field,
      };
      PostCqe(io_cq_, cqe);
    }
  }

  std::vector<std::unique_ptr<AlignedPage>> pages_;
  int64_t now_ticks_ = 0;
  uint32_t csts_reg_ = 0;
  uint32_t aqa_reg_ = 0;
  uint64_t asq_reg_ = 0;
  uint64_t acq_reg_ = 0;
  uint8_t* admin_sq_va_ = nullptr;
  int admin_sq_depth_ = 0;
  int admin_sq_head_ = 0;
  CqState admin_cq_{};
  uint8_t* io_sq_va_ = nullptr;
  int io_sq_depth_ = 0;
  int io_sq_head_ = 0;
  CqState io_cq_{};
};

class HalfAllOnesMmio final : public MmioWindow {
 public:
  explicit HalfAllOnesMmio(MmioWindow& inner) : inner_(inner) {}

  int64_t spoof_offset = -1;
  int spoof_reads_left = 0;

  uint32_t Read32(const int64_t offset) override {
    if (offset == spoof_offset && spoof_reads_left != 0) {
      if (spoof_reads_left > 0) {
        --spoof_reads_left;
      }
      return kMmioAllOnes;
    }
    return inner_.Read32(offset);
  }

  void Write32(const int64_t offset, const uint32_t value) override {
    inner_.Write32(offset, value);
  }

  int64_t Size() const override { return inner_.Size(); }

  bool Covers(const int64_t offset, const int64_t size) const override {
    return inner_.Covers(offset, size);
  }

 private:
  MmioWindow& inner_;
};

class CcWriteSpyMmio final : public MmioWindow {
 public:
  explicit CcWriteSpyMmio(MmioWindow& inner) : inner_(inner) {}

  std::vector<uint32_t> cc_writes;

  uint32_t Read32(const int64_t offset) override {
    return inner_.Read32(offset);
  }

  void Write32(const int64_t offset, const uint32_t value) override {
    if (offset == kRegCc) {
      cc_writes.push_back(value);
    }
    inner_.Write32(offset, value);
  }

  int64_t Size() const override { return inner_.Size(); }

  bool Covers(const int64_t offset, const int64_t size) const override {
    return inner_.Covers(offset, size);
  }

 private:
  MmioWindow& inner_;
};

// --- NvmeControllerTest ------------------------------------------------------

TEST(NvmeControllerTest, TwoPhaseBringUpAndMultiBlockReadWriteFlush) {
  sim::SimNvmeDevice dev;
  dev.ResetRegisters(/*initially_enabled=*/true);
  dev.model = "QEMU\x01\x1FNVMe~\x7F\x80 Ctrl";
  dev.serial = "protos0";

  NvmeController ctrl;
  ctrl.Init(dev, dev, /*bus=*/0, /*device=*/4, /*function=*/0);
  EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  EXPECT_EQ(ctrl.state(), NvmeState::kHalted);
  EXPECT_EQ(ctrl.Start(/*irq_mask=*/1), NvmeStatus::kOk);
  EXPECT_EQ(ctrl.state(), NvmeState::kRunning);
  EXPECT_TRUE(ctrl.has_volatile_write_cache());
  EXPECT_EQ(ctrl.namespace_count(), 1);
  EXPECT_STREQ(ctrl.model(), "QEMU??NVMe~?? Ctrl");
  EXPECT_STREQ(ctrl.serial(), "protos0");
  EXPECT_EQ(ctrl.ns(0).nsid, 1u);
  EXPECT_EQ(ctrl.ns(0).block_count, uint64_t{128});
  EXPECT_EQ(ctrl.ns(0).block_size, 512);
  EXPECT_EQ(ctrl.ns(0).max_transfer_blocks, 8);
  EXPECT_FALSE(ctrl.ns(0).read_only);
  EXPECT_EQ(dev.set_features_count, 1);
  EXPECT_TRUE(dev.io_cq_created);
  EXPECT_TRUE(dev.io_sq_created);
  // Queue addresses live above 4 GiB to exercise full 64-bit DMA registers.
  EXPECT_GE(dev.asq, 0x100000000ULL);
  EXPECT_GE(dev.acq, 0x100000000ULL);

  // Round-trip 8-sector (4 KiB) write, flush, and read.
  uint8_t tx[4096];
  for (int i = 0; i < 4096; ++i) {
    tx[i] = static_cast<uint8_t>((i * 7 + 13) & 0xFF);
  }
  const int io_cq_db_before = dev.io_cq_head_doorbell_writes;
  EXPECT_EQ(ctrl.WriteBlocks(0, 10, 8, tx), NvmeStatus::kOk);
  EXPECT_GT(dev.acq_head_doorbell_writes, 0);
  EXPECT_EQ(dev.io_cq_head_doorbell_writes, io_cq_db_before + 1);
  EXPECT_EQ(dev.io_cq_head_seen, 1);
  EXPECT_EQ(ctrl.Flush(0), NvmeStatus::kOk);
  EXPECT_EQ(dev.flush_count, 1);

  uint8_t rx[4096] = {};
  EXPECT_EQ(ctrl.ReadBlocks(0, 10, 8, rx), NvmeStatus::kOk);
  EXPECT_EQ(std::memcmp(tx, rx, sizeof(tx)), 0);

  // M2R8-01 (HUNT_68 / HUNT_71): Pre-populate LBA 20 directly in storage
  // without WriteBlocks on LBA 20, taint io_buf_ via WriteBlocks(0), then
  // ReadBlocks(20) across {2, 4, 8} blocks and 512/1024/2048-B geometries.
  {
    uint8_t poison_0xA5[4096] = {};
    std::memset(poison_0xA5, 0xA5, sizeof(poison_0xA5));
    for (const int32_t blocks : {2, 4, 8}) {
      const int byte_len = blocks * 512;
      const int storage_off = 20 * 512;
      for (int i = 0; i < byte_len; ++i) {
        const uint8_t raw =
            static_cast<uint8_t>((i * 13 + blocks * 17 + 0x31) & 0xFF);
        dev.namespaces[0].storage[storage_off + i] =
            (raw == 0 || raw == 0xA5) ? uint8_t{0x5A} : raw;
      }
      const NvmeStatus taint_rc = ctrl.WriteBlocks(0, 0, blocks, poison_0xA5);
      ASSERT_EQ(taint_rc, NvmeStatus::kOk) << blocks;
      uint8_t rx_fresh[4096] = {};
      std::memset(rx_fresh, 0xCC, sizeof(rx_fresh));
      const NvmeStatus read_rc = ctrl.ReadBlocks(0, 20, blocks, rx_fresh);
      ASSERT_EQ(read_rc, NvmeStatus::kOk) << blocks;
      const uint8_t* const expected_fresh =
          dev.namespaces[0].storage.data() + storage_off;
      EXPECT_EQ(std::memcmp(rx_fresh, expected_fresh, byte_len), 0) << blocks;
      for (int i = byte_len; i < 4096; ++i) {
        EXPECT_EQ(rx_fresh[i], 0xCC) << blocks << "/" << i;
      }
    }
    for (const int32_t bs : {1024, 2048}) {
      sim::SimNvmeDevice geom_dev;
      geom_dev.namespaces[0].block_size = bs;
      geom_dev.namespaces[0].InitStorage();
      NvmeController geom_ctrl;
      geom_ctrl.Init(geom_dev, geom_dev);
      ASSERT_EQ(geom_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk) << bs;
      ASSERT_EQ(geom_ctrl.Start(1), NvmeStatus::kOk) << bs;
      ASSERT_EQ(geom_ctrl.ns(0).block_size, bs) << bs;
      const int32_t max_blk = geom_ctrl.ns(0).max_transfer_blocks;
      ASSERT_EQ(max_blk, 4096 / bs) << bs;
      for (const int32_t blocks : {2, 4, 8}) {
        if (blocks > max_blk) {
          continue;
        }
        const int byte_len = blocks * bs;
        const int storage_off = 20 * bs;
        for (int i = 0; i < byte_len; ++i) {
          const uint8_t raw =
              static_cast<uint8_t>((i * 13 + blocks * 17 + bs + 0x31) & 0xFF);
          geom_dev.namespaces[0].storage[storage_off + i] =
              (raw == 0 || raw == 0xA5) ? uint8_t{0x5A} : raw;
        }
        const NvmeStatus taint_rc =
            geom_ctrl.WriteBlocks(0, 0, blocks, poison_0xA5);
        ASSERT_EQ(taint_rc, NvmeStatus::kOk) << bs << "/" << blocks;
        uint8_t rx_fresh[4096] = {};
        std::memset(rx_fresh, 0xCC, sizeof(rx_fresh));
        const NvmeStatus read_rc =
            geom_ctrl.ReadBlocks(0, 20, blocks, rx_fresh);
        ASSERT_EQ(read_rc, NvmeStatus::kOk) << bs << "/" << blocks;
        const uint8_t* const expected_fresh =
            geom_dev.namespaces[0].storage.data() + storage_off;
        EXPECT_EQ(std::memcmp(rx_fresh, expected_fresh, byte_len), 0)
            << bs << "/" << blocks;
        for (int i = byte_len; i < 4096; ++i) {
          EXPECT_EQ(rx_fresh[i], 0xCC) << bs << "/" << blocks << "/" << i;
        }
      }
    }
  }

  // Zero-block I/O is a no-op without failing.
  EXPECT_EQ(ctrl.ReadBlocks(0, 128, 0, nullptr), NvmeStatus::kOk);
  EXPECT_EQ(ctrl.WriteBlocks(0, 128, 0, nullptr), NvmeStatus::kOk);

  // Out-of-bounds and invalid-state checks.
  const int io_before = dev.io_commands_seen;
  EXPECT_EQ(ctrl.ReadBlocks(0, 125, 4, rx), NvmeStatus::kInvalidArgument);
  EXPECT_EQ(ctrl.ReadBlocks(0, UINT64_MAX, 1, rx),
            NvmeStatus::kInvalidArgument);
  EXPECT_EQ(ctrl.ReadBlocks(0, 0, 9, rx), NvmeStatus::kInvalidArgument);
  EXPECT_EQ(ctrl.WriteBlocks(0, 0, 9, tx), NvmeStatus::kInvalidArgument);
  EXPECT_EQ(ctrl.ReadBlocks(-1, 0, 1, rx), NvmeStatus::kInvalidArgument);
  EXPECT_EQ(ctrl.ReadBlocks(-1, 0, 0, nullptr), NvmeStatus::kInvalidArgument);
  EXPECT_EQ(ctrl.WriteBlocks(-1, 0, 0, nullptr), NvmeStatus::kInvalidArgument);
  EXPECT_EQ(ctrl.ReadBlocks(0, 129, 0, nullptr), NvmeStatus::kInvalidArgument);
  EXPECT_EQ(ctrl.WriteBlocks(0, 129, 0, nullptr), NvmeStatus::kInvalidArgument);
  EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, nullptr), NvmeStatus::kInvalidArgument);
  EXPECT_EQ(ctrl.Flush(-1), NvmeStatus::kInvalidArgument);
  EXPECT_EQ(ctrl.Flush(1), NvmeStatus::kInvalidArgument);
  EXPECT_EQ(ctrl.ReadBlocks(1, 0, 1, rx), NvmeStatus::kInvalidArgument);
  EXPECT_EQ(ctrl.WriteBlocks(1, 0, 1, tx), NvmeStatus::kInvalidArgument);
  const volatile int max_ns_runtime = NvmeController::kMaxNamespaces;
  const int oob_ns = max_ns_runtime;
  EXPECT_EQ(ctrl.ReadBlocks(oob_ns, 0, 1, rx), NvmeStatus::kInvalidArgument);
  EXPECT_EQ(ctrl.WriteBlocks(oob_ns, 0, 1, tx), NvmeStatus::kInvalidArgument);
  EXPECT_EQ(ctrl.Flush(oob_ns), NvmeStatus::kInvalidArgument);
  EXPECT_EQ(dev.io_commands_seen, io_before);

  dev.fail_flush = true;
  EXPECT_EQ(ctrl.Flush(0), NvmeStatus::kCommandFailed);
  EXPECT_EQ(ctrl.state(), NvmeState::kRunning);
  dev.fail_flush = false;

  // MDTS=0 still clamps max_transfer_bytes_ to kIoMaxBytes (4096 B).
  dev.mdts = 0;
  ASSERT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
  EXPECT_EQ(ctrl.ReadBlocks(0, 0, 0, nullptr), NvmeStatus::kNotRunning);
  EXPECT_EQ(ctrl.WriteBlocks(0, 0, 0, nullptr), NvmeStatus::kNotRunning);
  EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, rx), NvmeStatus::kNotRunning);
  EXPECT_EQ(ctrl.WriteBlocks(0, 0, 1, tx), NvmeStatus::kNotRunning);
  EXPECT_EQ(ctrl.Flush(0), NvmeStatus::kNotRunning);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  EXPECT_EQ(ctrl.ns(0).max_transfer_blocks, 8);
}

TEST(NvmeControllerTest,
     TakeOwnershipAndResetClearsStaleShnAndValidatesCapabilities) {
  // Stale CC (CC.EN=0, CC.SHN=01b) must be cleared to 0.
  {
    sim::SimNvmeDevice dev;
    dev.cc = kCcShnNormal | (2u << 7);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(dev.cc, 0u);
  }
  // NEW_RESET_ALWAYS_WRITE_CC0: clean CC == 0 skips Write32(kRegCc, 0).
  {
    sim::SimNvmeDevice dev;
    CcWriteSpyMmio spy(dev);
    NvmeController ctrl;
    ctrl.Init(dev, spy);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_TRUE(spy.cc_writes.empty());
  }
  // Pre-faulted enabled controller (CSTS.CFS=1) resets cleanly to kHalted
  // because WaitReadyBit(/*want_ready=*/false) ignores CSTS.CFS on disable.
  {
    sim::SimNvmeDevice dev;
    dev.ResetRegisters(/*initially_enabled=*/true);
    dev.csts |= kCstsCfs;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.state(), NvmeState::kHalted);
    EXPECT_EQ(dev.cc, 0u);
  }
  // Half-all-ones CAP detection (|| vs &&).
  {
    sim::SimNvmeDevice dev;
    dev.cap = 0x00000000FFFFFFFFULL;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kNotDecoding);
  }
  {
    sim::SimNvmeDevice dev;
    dev.cap = 0xFFFFFFFF00000000ULL;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kNotDecoding);
  }
  // N34: HalfAllOnes on CC during TakeOwnershipAndReset returns kNotDecoding.
  {
    sim::SimNvmeDevice dev;
    HalfAllOnesMmio mmio(dev);
    mmio.spoof_offset = kRegCc;
    mmio.spoof_reads_left = 1;
    NvmeController ctrl;
    ctrl.Init(dev, mmio);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kNotDecoding);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  // MPSMIN != 0 rejected; MPSMIN == 0 && MPSMAX == 4 accepted.
  {
    sim::SimNvmeDevice dev;
    dev.cap = (63ULL << 0) | kCapCssNvm | (1ULL << 48) | (4ULL << 52);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kBadCapabilities);
  }
  {
    sim::SimNvmeDevice dev;
    dev.cap = (63ULL << 0) | kCapCssNvm | (4ULL << 52);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  }
  {
    sim::SimNvmeDevice dev;
    dev.cap = (63ULL << 0) | kCapCssNvm | (8ULL << 48) | (8ULL << 52);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kBadCapabilities);
  }
  {
    sim::SimNvmeDevice dev;
    dev.cap = (63ULL << 0) | kCapCssNvm | (1ULL << 52);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  }
  // MQES < 2 (CAP[15:0] == 0 => MQES == 1) rejected.
  {
    sim::SimNvmeDevice dev;
    dev.cap = kCapCssNvm;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kBadCapabilities);
  }
  // DSTRD=2 works when doorbells are covered; fails kBadCapabilities on hole.
  {
    sim::SimNvmeDevice dev;
    dev.cap = (63ULL << 0) | (2ULL << 24) | (2ULL << 32) | kCapCssNvm;
    dev.SetSizeAndHole(0x2000, 0, 0);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk);
  }
  {
    sim::SimNvmeDevice dev;
    dev.cap = (63ULL << 0) | (2ULL << 24) | (10ULL << 32) | kCapCssNvm;
    dev.SetSizeAndHole(0x4000, 0, 0);  // Covers SQ0/CQ0/SQ1, misses CQ1 0x4000.
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kBadCapabilities);
  }
  // Separate DSTRD=10 holes over SQ0 (0x1000), CQ0 (0x2000), and SQ1 (0x3000)
  // so each doorbell Covers() check is tested independently.
  for (const int64_t hole_start : {0x1000, 0x2000, 0x3000}) {
    sim::SimNvmeDevice dev;
    dev.cap = (63ULL << 0) | (2ULL << 24) | (10ULL << 32) | kCapCssNvm;
    dev.SetSizeAndHole(0x5000, hole_start, hole_start + 0x1000);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kBadCapabilities)
        << hole_start;
    EXPECT_EQ(ctrl.state(), NvmeState::kError) << hole_start;
  }
  // Tight 4-byte doorbell Covers() span check across SQ0, CQ0, SQ1, CQ1.
  for (const int64_t db_off : {0x1000, 0x2000, 0x3000, 0x4000}) {
    {
      sim::SimNvmeDevice dev;
      dev.cap = (63ULL << 0) | (2ULL << 24) | (10ULL << 32) | kCapCssNvm;
      dev.SetSizeAndHole(0x5000, db_off + 4, db_off + 8);
      NvmeController ctrl;
      ctrl.Init(dev, dev);
      EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk) << db_off;
    }
    {
      sim::SimNvmeDevice dev;
      dev.cap = (63ULL << 0) | (2ULL << 24) | (10ULL << 32) | kCapCssNvm;
      dev.SetSizeAndHole(0x5000, db_off + 3, db_off + 4);
      NvmeController ctrl;
      ctrl.Init(dev, dev);
      EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kBadCapabilities)
          << db_off;
      EXPECT_EQ(ctrl.state(), NvmeState::kError) << db_off;
    }
  }

  // CAP.NSSRS (bit 36) must not bleed into CAP.DSTRD.
  {
    sim::SimNvmeDevice dev;
    dev.cap = (63ULL << 0) | (2ULL << 24) | (1ULL << 36) | kCapCssNvm;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk);
  }
  // ADV_M2_R3_05: MQES=256 (> kQueueDepth=32) clamps Admin and I/O depths
  // tightly to 32.
  {
    sim::SimNvmeDevice dev;
    dev.cap = (255ULL << 0) | (2ULL << 24) | kCapCssNvm;
    ASSERT_EQ(CapMqes(dev.cap), 256);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_EQ(dev.asqs, NvmeController::kQueueDepth);
    EXPECT_EQ(dev.acqs, NvmeController::kQueueDepth);
    EXPECT_EQ(dev.io_sq_size, NvmeController::kQueueDepth);
    EXPECT_EQ(dev.io_cq_size, NvmeController::kQueueDepth);
    EXPECT_EQ(dev.asqs, 32);
    EXPECT_EQ(dev.io_sq_size, 32);
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk);
  }
  // N32a / N32b: CAP.MQES values in [16..31] size queues strictly to CapMqes.
  for (const uint32_t raw_mqes : {15u, 16u, 29u, 30u}) {
    sim::SimNvmeDevice dev;
    dev.cap = static_cast<uint64_t>(raw_mqes) | (2ULL << 24) | kCapCssNvm;
    const int expected_depth = static_cast<int>(raw_mqes) + 1;
    ASSERT_EQ(CapMqes(dev.cap), expected_depth);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk) << expected_depth;
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk) << expected_depth;
    EXPECT_EQ(dev.asqs, expected_depth);
    EXPECT_EQ(dev.acqs, expected_depth);
    EXPECT_EQ(dev.io_sq_size, expected_depth);
    EXPECT_EQ(dev.io_cq_size, expected_depth);
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk) << expected_depth;
  }
  // Zero TicksPerSecond clamps WaitReadyBit wait_ticks to at least 1 tick.
  {
    sim::SimNvmeDevice dev;
    dev.ticks_per_second = 0;
    dev.enable_delay_ticks = 1;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  }
}

TEST(NvmeControllerTest, HonorsLongCapTimeoutBeyond30Seconds) {
  {
    sim::SimNvmeDevice dev;
    dev.enable_delay_ticks = 9000;  // 36 s at 250 Hz
    dev.ResetRegisters(/*initially_enabled=*/false);
    dev.cap = (63ULL << 0) | (80ULL << 24) | kCapCssNvm;  // 40,000 ms
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  }
  {
    sim::SimNvmeDevice dev;
    dev.cap = (63ULL << 0) | (80ULL << 24) | kCapCssNvm;
    dev.hang_enable = true;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kTimeout);
    EXPECT_GE(dev.now_ticks, 10001);
    EXPECT_EQ(dev.cc, 0u);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  // Default CAP.TO=2 (1000 ms) must still honor the 2000 ms enable floor.
  {
    sim::SimNvmeDevice dev;
    ASSERT_EQ(CapToMs(dev.cap), 1000);
    dev.enable_delay_ticks = 300;  // 1200 ms at 250 Hz (> 1000 ms, < 2000 ms)
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  }
  // C_C152_MIN_TO_1500 & C_C249_WAIT_READY_GE: CAP.TO=1 (500 ms) enforces
  // the 2000 ms floor (2001 ticks @ 1000 Hz passes, 2002 times out), and
  // TicksPerSecond()==0 clamps wait_ticks=1 with >= deadline (2 times out).
  {
    sim::SimNvmeDevice dev;
    dev.ticks_per_second = 1000;
    dev.cap = (63ULL << 0) | (1ULL << 24) | kCapCssNvm;  // 500 ms
    ASSERT_EQ(CapToMs(dev.cap), 500);
    dev.enable_delay_ticks = 2001;  // TicksFromMs(2000, 1000) == 2001
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  }
  {
    sim::SimNvmeDevice dev;
    dev.ticks_per_second = 1000;
    dev.cap = (63ULL << 0) | (1ULL << 24) | kCapCssNvm;
    dev.enable_delay_ticks = 2002;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kTimeout);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  {
    sim::SimNvmeDevice dev;
    dev.ticks_per_second = 0;
    dev.enable_delay_ticks = 2;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kTimeout);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  // N33: CAP.TO = 5 (2500 ms) and CAP.TO = 6 (3000 ms) honor exact CAP.TO
  // values between 2000 ms and 10000 ms.
  {
    sim::SimNvmeDevice dev;
    dev.ticks_per_second = 1000;
    dev.enable_delay_ticks = 2250;
    dev.ResetRegisters(/*initially_enabled=*/false);
    dev.cap = (63ULL << 0) | (5ULL << 24) | kCapCssNvm;
    ASSERT_EQ(CapToMs(dev.cap), 2500);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  }
  {
    sim::SimNvmeDevice dev;
    dev.ticks_per_second = 1000;
    dev.enable_delay_ticks = 2500;
    dev.ResetRegisters(/*initially_enabled=*/false);
    dev.cap = (63ULL << 0) | (6ULL << 24) | kCapCssNvm;
    ASSERT_EQ(CapToMs(dev.cap), 3000);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  }
  {
    sim::SimNvmeDevice dev;
    dev.ticks_per_second = 1000;
    dev.enable_delay_ticks = 3002;
    dev.ResetRegisters(/*initially_enabled=*/false);
    dev.cap = (63ULL << 0) | (6ULL << 24) | kCapCssNvm;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kTimeout);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  // N35 + TO_500: TakeOwnershipAndReset WaitReadyBit(false) honors
  // enable_timeout_ms_ (both CAP.TO=10 -> 5000 ms and CAP.TO=0 -> 2000 ms
  // floor).
  for (const int cap_to : {10, 0}) {
    sim::SimNvmeDevice dev;
    dev.ticks_per_second = 1000;
    dev.disable_delay_ticks = (cap_to == 10) ? 3000 : 1000;
    dev.ResetRegisters(/*initially_enabled=*/true);
    dev.cap = (63ULL << 0) | (static_cast<uint64_t>(cap_to) << 24) | kCapCssNvm;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk) << cap_to;
    EXPECT_EQ(ctrl.state(), NvmeState::kHalted) << cap_to;
  }
  // N45 + TO_500: WaitShutdownComplete honors enable_timeout_ms_.
  for (const int cap_to : {10, 0}) {
    sim::SimNvmeDevice dev;
    dev.ticks_per_second = 1000;
    dev.shutdown_delay_ticks = (cap_to == 10) ? 3000 : 1000;
    dev.ResetRegisters(/*initially_enabled=*/false);
    dev.cap = (63ULL << 0) | (static_cast<uint64_t>(cap_to) << 24) | kCapCssNvm;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk) << cap_to;
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk) << cap_to;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kOk) << cap_to;
    EXPECT_EQ(ctrl.state(), NvmeState::kHalted) << cap_to;
  }
  // N46 + TO_500: DisableControllerBestEffort honors enable_timeout_ms_.
  for (const int cap_to : {10, 0}) {
    sim::SimNvmeDevice dev;
    dev.ticks_per_second = 1000;
    dev.disable_delay_ticks = (cap_to == 10) ? 3000 : 1000;
    dev.ResetRegisters(/*initially_enabled=*/false);
    dev.cap = (63ULL << 0) | (static_cast<uint64_t>(cap_to) << 24) | kCapCssNvm;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk) << cap_to;
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk) << cap_to;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kOk) << cap_to;
    EXPECT_EQ(ctrl.state(), NvmeState::kHalted) << cap_to;
  }
}

TEST(NvmeControllerTest, EnsureDmaBuffersChecksAlignmentAndPartialFailure) {
  for (int limit = 1; limit <= 5; ++limit) {
    sim::SimNvmeDevice dev;
    dev.dma_alloc_limit = limit;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kDmaAllocFailed)
        << limit;
    dev.dma_alloc_limit = 0;
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk) << limit;
  }

  sim::SimNvmeDevice dev;
  dev.force_unaligned_dma_phys = true;
  NvmeController ctrl;
  ctrl.Init(dev, dev);
  EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kDmaAllocFailed);
  // Failed unaligned allocation must not stick on subsequent retries.
  dev.force_unaligned_dma_phys = false;
  EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  EXPECT_EQ(ctrl.Start(1), NvmeStatus::kOk);

  // HUNT_09: non-null va with phys == 0 must fail kDmaAllocFailed.
  {
    FakeNvmeHost fake;
    fake.force_zero_dma_phys = true;
    NvmeController fake_ctrl;
    fake_ctrl.Init(fake, fake);
    EXPECT_EQ(fake_ctrl.TakeOwnershipAndReset(), NvmeStatus::kDmaAllocFailed);
    EXPECT_EQ(fake_ctrl.state(), NvmeState::kError);
    fake.force_zero_dma_phys = false;
    EXPECT_EQ(fake_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(fake_ctrl.Start(1), NvmeStatus::kOk);
  }
  // HUNT_10: non-null va and aligned phys with size < kPageSize must fail.
  {
    FakeNvmeHost fake;
    fake.override_dma_size = 4095;
    NvmeController fake_ctrl;
    fake_ctrl.Init(fake, fake);
    EXPECT_EQ(fake_ctrl.TakeOwnershipAndReset(), NvmeStatus::kDmaAllocFailed);
    EXPECT_EQ(fake_ctrl.state(), NvmeState::kError);
    fake.override_dma_size = 0;
    EXPECT_EQ(fake_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(fake_ctrl.Start(1), NvmeStatus::kOk);
  }
  // C_C468_PHYS_2K: 2 KiB-aligned (non-4 KiB) DMA physical address fails.
  {
    FakeNvmeHost fake;
    fake.dma_phys_offset = 2048ULL;
    NvmeController fake_ctrl;
    fake_ctrl.Init(fake, fake);
    EXPECT_EQ(fake_ctrl.TakeOwnershipAndReset(), NvmeStatus::kDmaAllocFailed);
    EXPECT_EQ(fake_ctrl.state(), NvmeState::kError);
    fake.dma_phys_offset = 0;
    EXPECT_EQ(fake_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(fake_ctrl.Start(1), NvmeStatus::kOk);
  }
  // M2R7-07 (SHUNT_STEP2_DMA_ALIGN_MASK_0XFFC): reject sub-4B DMA physical
  // address offsets (+1ULL and +2ULL) in EnsureDmaBuffers.
  for (const uint64_t bad_offset : {1ULL, 2ULL}) {
    FakeNvmeHost fake;
    fake.dma_phys_offset = bad_offset;
    NvmeController fake_ctrl;
    fake_ctrl.Init(fake, fake);
    EXPECT_EQ(fake_ctrl.TakeOwnershipAndReset(), NvmeStatus::kDmaAllocFailed)
        << bad_offset;
    EXPECT_EQ(fake_ctrl.state(), NvmeState::kError) << bad_offset;
    fake.dma_phys_offset = 0;
    EXPECT_EQ(fake_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk) << bad_offset;
    EXPECT_EQ(fake_ctrl.Start(1), NvmeStatus::kOk) << bad_offset;
  }
}

TEST(NvmeControllerTest, MsiModeUnmasksIntmcWhileMsixLeavesIntmsUntouched) {
  {
    sim::SimNvmeDevice dev;
    dev.msi_mode = true;
    dev.intms = 0xFFFFFFFFu;
    dev.async_io_delay_ticks = 3;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1, IrqMode::kMsi), NvmeStatus::kOk);
    EXPECT_EQ(dev.intms, 0u);
    EXPECT_EQ(dev.intmc_writes_seen, 1);
    EXPECT_EQ(dev.intms_writes_seen, 0);

    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk);
    EXPECT_EQ(dev.wait_for_irq_calls, 1);
    EXPECT_LT(dev.wait_for_irq_ticks_total, 20);
  }
  {
    sim::SimNvmeDevice dev;
    dev.msi_mode = false;
    dev.intms = 0xFFFFFFFFu;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1, IrqMode::kMsix), NvmeStatus::kOk);
    EXPECT_EQ(dev.intmc_writes_seen, 0);
    EXPECT_EQ(dev.intms_writes_seen, 0);
    EXPECT_EQ(dev.intms, 0xFFFFFFFFu);
  }
}

TEST(NvmeControllerTest, AsyncCompletionsExerciseWaitForIrqAndExactDeadline) {
  {
    sim::SimNvmeDevice dev;
    dev.async_admin_delay_ticks = 3;
    dev.async_io_delay_ticks = 25;  // > kMaxWaitChunkTicks (20)
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    const int calls_after_start = dev.wait_for_irq_calls;
    EXPECT_GT(calls_after_start, 0);

    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk);
    EXPECT_EQ(dev.wait_for_irq_calls - calls_after_start, 2);
    EXPECT_LE(dev.last_wait_timeout_ticks, 20);
  }
  // Exact 5000 ms deadline at 250 Hz == 1251 ticks (+1 slack): 1251 passes,
  // 1252 times out.
  {
    sim::SimNvmeDevice dev;
    dev.async_io_delay_ticks = 1251;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk);
  }
  {
    sim::SimNvmeDevice dev;
    dev.async_io_delay_ticks = 1252;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kTimeout);
    EXPECT_EQ(dev.cc, 0u);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }

  // ADV_M2_R3_06: 64-bit irq_mask >= 32 (and multi-bit) propagates intact.
  for (const uint64_t high_mask :
       {uint64_t{1} << 35, (uint64_t{1} << 35) | (uint64_t{1} << 63) | 0x20u}) {
    sim::SimNvmeDevice dev;
    dev.async_admin_delay_ticks = 2;
    dev.async_io_delay_ticks = 2;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(high_mask), NvmeStatus::kOk);
    EXPECT_EQ(dev.last_wait_irq_mask, high_mask);
    dev.last_wait_irq_mask = 0;
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk);
    EXPECT_EQ(dev.last_wait_irq_mask, high_mask);
    EXPECT_LT(dev.wait_for_irq_ticks_total, 20);
  }
}

TEST(NvmeControllerTest, PreemptionPastDeadlineChecksStatusBeforeTimingOut) {
  // 1. WaitReadyBit preempted past deadline simultaneous with CSTS.RDY = 1.
  {
    sim::SimNvmeDevice dev;
    dev.enable_delay_ticks = 100;
    dev.ResetRegisters(/*initially_enabled=*/false);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);

    int now_calls = 0;
    dev.on_now_ticks = [&]() {
      if ((dev.cc & kCcEn) != 0) {
        ++now_calls;
        if (now_calls == 2) {
          dev.now_ticks += 5000;
          dev.csts |= kCstsRdy;
        }
      }
    };
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  }

  // 2. SubmitAndWait preempted past deadline simultaneous with CQE completion.
  {
    sim::SimNvmeDevice dev;
    dev.async_io_delay_ticks = 5;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);

    const int before_irq_calls = dev.wait_for_irq_calls;
    int io_now_calls = 0;
    dev.on_now_ticks = [&]() {
      if (dev.io_commands_seen > 0) {
        ++io_now_calls;
        if (io_now_calls == 2) {
          dev.now_ticks += 5000;
          if (dev.delayed_action) {
            const auto fn = std::move(dev.delayed_action);
            dev.delayed_action = {};
            fn();
          }
        }
      }
    };
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk);
    EXPECT_EQ(dev.wait_for_irq_calls, before_irq_calls);
  }

  // 3. Preemption between top-of-loop NowTicks() and WaitForIrq skips
  // WaitForIrq and completes on the re-poll iteration.
  {
    sim::SimNvmeDevice dev;
    dev.async_io_delay_ticks = 5;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);

    const int before_irq_calls = dev.wait_for_irq_calls;
    int io_now_calls = 0;
    dev.on_now_ticks = [&]() {
      if (dev.io_commands_seen > 0) {
        ++io_now_calls;
        if (io_now_calls == 3) {
          dev.now_ticks += 5000;
          if (dev.delayed_action) {
            const auto fn = std::move(dev.delayed_action);
            dev.delayed_action = {};
            fn();
          }
        }
      }
    };
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk);
    EXPECT_EQ(dev.wait_for_irq_calls, before_irq_calls);
  }

  // 4. Wraparound across INT64_MAX -> INT64_MIN succeeds cleanly.
  {
    sim::SimNvmeDevice dev;
    dev.now_ticks = INT64_MAX - 2;
    dev.async_io_delay_ticks = 10;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk);
    // P5_W26_SHN_WRAP: set now_ticks right before Shutdown().
    dev.now_ticks = INT64_MAX - 1;
    dev.shutdown_delay_ticks = 5;
    dev.disable_delay_ticks = 5;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.state(), NvmeState::kHalted);
    EXPECT_TRUE(dev.saw_shn_normal);
    EXPECT_TRUE(dev.saw_cc_zero_after_shn);
    EXPECT_LT(dev.now_ticks, 0);
  }
  // M2R8-03 (CTRL_SHUNT_04_UNSIGNED): WaitShutdownComplete timeout across
  // INT64_MAX and negative->0 wraparound, plus WaitReadyBit(false) wraparound.
  for (const int64_t wrap_start :
       {static_cast<int64_t>(INT64_MAX - 1), static_cast<int64_t>(-5)}) {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk) << wrap_start;
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk) << wrap_start;
    dev.now_ticks = wrap_start;
    dev.hang_shutdown = true;
    dev.disable_delay_ticks = 5;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kTimeout) << wrap_start;
    EXPECT_EQ(ctrl.state(), NvmeState::kError) << wrap_start;
    EXPECT_EQ(dev.cc, 0u) << wrap_start;
    if (wrap_start > 0) {
      EXPECT_LT(dev.now_ticks, 0);
    } else {
      EXPECT_GT(dev.now_ticks, 0);
    }
  }
  {
    sim::SimNvmeDevice dev;
    dev.ResetRegisters(/*initially_enabled=*/true);
    dev.now_ticks = INT64_MAX - 1;
    dev.disable_delay_ticks = 5;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.state(), NvmeState::kHalted);
    EXPECT_LT(dev.now_ticks, 0);
  }
  // WILD_M5_SUBMIT_WRAP_SUB_UBSAN & WILD_M5_SUBMIT_WRAP_NOW_LT_START: set
  // now_ticks after Start(1) so SubmitAndWait pre-wait logic crosses
  // INT64_MAX -> INT64_MIN and negative -> positive across > 20-tick wait.
  for (const int64_t wrap_start :
       {static_cast<int64_t>(INT64_MAX - 5), static_cast<int64_t>(-5)}) {
    sim::SimNvmeDevice dev;
    dev.async_io_delay_ticks = 25;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    dev.now_ticks = wrap_start;
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk) << wrap_start;
  }
  // M2R7-01 (SHUNT_01): SubmitAndWait timeout across INT64_MAX and
  // negative->0 wraparound must still return kTimeout without hanging.
  for (const int64_t wrap_start :
       {static_cast<int64_t>(INT64_MAX - 5), static_cast<int64_t>(-5)}) {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk) << wrap_start;
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk) << wrap_start;
    dev.now_ticks = wrap_start;
    dev.hang_io_cmd = true;
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kTimeout)
        << wrap_start;
    EXPECT_EQ(ctrl.state(), NvmeState::kError) << wrap_start;
    EXPECT_EQ(dev.cc, 0u) << wrap_start;
    if (wrap_start > 0) {
      EXPECT_LT(dev.now_ticks, 0);
    } else {
      EXPECT_GT(dev.now_ticks, 0);
    }
  }
  // M2R7-02 (SHUNT_03): WaitReadyBit enable and disable timeouts across
  // INT64_MAX and negative->0 wraparound must still return kTimeout.
  for (const int64_t wrap_start :
       {static_cast<int64_t>(INT64_MAX - 5), static_cast<int64_t>(-5)}) {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk) << wrap_start;
    dev.now_ticks = wrap_start;
    dev.hang_enable = true;
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kTimeout) << wrap_start;
    EXPECT_EQ(ctrl.state(), NvmeState::kError) << wrap_start;
    EXPECT_EQ(dev.cc, 0u) << wrap_start;
    if (wrap_start > 0) {
      EXPECT_LT(dev.now_ticks, 0);
    } else {
      EXPECT_GT(dev.now_ticks, 0);
    }
  }
  for (const int64_t wrap_start :
       {static_cast<int64_t>(INT64_MAX - 5), static_cast<int64_t>(-5)}) {
    sim::SimNvmeDevice dev;
    dev.ResetRegisters(/*initially_enabled=*/true);
    dev.now_ticks = wrap_start;
    dev.hang_disable = true;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kTimeout) << wrap_start;
    EXPECT_EQ(ctrl.state(), NvmeState::kError) << wrap_start;
    EXPECT_EQ(dev.cc, 0u) << wrap_start;
    if (wrap_start > 0) {
      EXPECT_LT(dev.now_ticks, 0);
    } else {
      EXPECT_GT(dev.now_ticks, 0);
    }
  }
  // M2R7-09 (SHUNT_02): SubmitAndWait pre-wait preemption crossing INT64_MAX
  // or negative->0 past deadline must skip WaitForIrq and return kTimeout.
  struct WrapJumpCase {
    int64_t start_ticks;
    int64_t jump_ticks;
  };
  const WrapJumpCase kWrapJumps[] = {
      {INT64_MAX - 5, INT64_MIN + 6000},
      {-5, 6000},
  };
  for (const WrapJumpCase& wc : kWrapJumps) {
    sim::SimNvmeDevice dev;
    dev.ticks_per_second = 1000;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk) << wc.start_ticks;
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk) << wc.start_ticks;
    dev.now_ticks = wc.start_ticks;
    dev.hang_io_cmd = true;
    int io_now_calls = 0;
    dev.on_now_ticks = [&]() {
      if (dev.io_commands_seen > 0) {
        ++io_now_calls;
        if (io_now_calls == 3) {
          dev.now_ticks = wc.jump_ticks;
        }
      }
    };
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kTimeout)
        << wc.start_ticks;
    EXPECT_EQ(ctrl.state(), NvmeState::kError) << wc.start_ticks;
    EXPECT_EQ(dev.wait_for_irq_calls, 0) << wc.start_ticks;
    EXPECT_EQ(dev.cc, 0u) << wc.start_ticks;
  }
  // N51a & N51b: SubmitAndWait clamps wait_ticks == 1 when TicksPerSecond ==
  // 0 (1 tick succeeds, 2 ticks times out with 1 WaitForIrq tick total).
  {
    sim::SimNvmeDevice dev;
    dev.ticks_per_second = 0;
    dev.enable_delay_ticks = 1;
    dev.disable_delay_ticks = 1;
    dev.async_io_delay_ticks = 1;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    const int64_t ticks_before = dev.wait_for_irq_ticks_total;
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk);
    EXPECT_EQ(dev.wait_for_irq_ticks_total - ticks_before, 1);
  }
  {
    sim::SimNvmeDevice dev;
    dev.ticks_per_second = 0;
    dev.enable_delay_ticks = 1;
    dev.disable_delay_ticks = 1;
    dev.async_io_delay_ticks = 2;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    const int64_t ticks_before = dev.wait_for_irq_ticks_total;
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kTimeout);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.wait_for_irq_ticks_total - ticks_before, 1);
  }
}

TEST(NvmeControllerTest, HandlesQueueWrapAroundAndPhaseFlip) {
  // Exercise small MQES=4 queue wrap and > 32 Admin SQ/CQ wraps.
  sim::SimNvmeDevice dev;
  dev.cap = (3ULL << 0) | (2ULL << 24) | kCapCssNvm;  // MQES = 4
  dev.namespaces.clear();
  for (uint32_t id = 1; id <= 36; ++id) {
    sim::SimNvmeDevice::NamespaceConfig ns;
    ns.nsid = id;
    ns.block_count = (id <= 30) ? 0 : 64;  // 30 skipped, 6 valid => >36 cmds
    ns.block_size = 512;
    if (ns.block_count > 0) {
      ns.InitStorage();
    }
    dev.namespaces.push_back(std::move(ns));
  }

  NvmeController ctrl;
  ctrl.Init(dev, dev);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  EXPECT_EQ(ctrl.namespace_count(), 6);
  EXPECT_GT(dev.admin_commands_seen, 36);
  EXPECT_EQ(dev.asqs, 4);
  EXPECT_EQ(dev.acqs, 4);
  EXPECT_EQ(dev.io_sq_size, 4);
  EXPECT_EQ(dev.io_cq_size, 4);

  // Submit > MQES and > kQueueDepth I/O commands so SQ/CQ wrap repeatedly.
  for (int round = 0; round < 70; ++round) {
    uint8_t sector[512] = {};
    sector[0] = static_cast<uint8_t>(round);
    sector[511] = static_cast<uint8_t>(round ^ 0x5A);
    const uint64_t lba = static_cast<uint64_t>(round % 64);
    ASSERT_EQ(ctrl.WriteBlocks(0, lba, 1, sector), NvmeStatus::kOk) << round;
    uint8_t back[512] = {};
    ASSERT_EQ(ctrl.ReadBlocks(0, lba, 1, back), NvmeStatus::kOk) << round;
    ASSERT_EQ(std::memcmp(sector, back, sizeof(sector)), 0) << round;
  }
  // H03 / C_C137_MQES_2: Minimal valid queue depth MQES == 2 (CAP.MQES=1).
  {
    sim::SimNvmeDevice min_dev;
    min_dev.cap = (1ULL << 0) | (2ULL << 24) | kCapCssNvm;
    ASSERT_EQ(CapMqes(min_dev.cap), 2);
    NvmeController min_ctrl;
    min_ctrl.Init(min_dev, min_dev);
    ASSERT_EQ(min_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(min_ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_EQ(min_dev.asqs, 2);
    EXPECT_EQ(min_dev.acqs, 2);
    EXPECT_EQ(min_dev.io_sq_size, 2);
    EXPECT_EQ(min_dev.io_cq_size, 2);
    for (int round = 0; round < 6; ++round) {
      uint8_t sector[512] = {};
      sector[0] = static_cast<uint8_t>(round + 1);
      sector[511] = static_cast<uint8_t>(round ^ 0xA5);
      const uint64_t lba = static_cast<uint64_t>(round);
      ASSERT_EQ(min_ctrl.WriteBlocks(0, lba, 1, sector), NvmeStatus::kOk)
          << round;
      uint8_t back[512] = {};
      ASSERT_EQ(min_ctrl.ReadBlocks(0, lba, 1, back), NvmeStatus::kOk) << round;
      ASSERT_EQ(std::memcmp(sector, back, sizeof(sector)), 0) << round;
    }
    EXPECT_EQ(min_ctrl.Shutdown(), NvmeStatus::kOk);
    EXPECT_EQ(min_ctrl.state(), NvmeState::kHalted);
  }
}

TEST(NvmeControllerTest, SetFeaturesNumQueuesValidatesDw0) {
  {
    sim::SimNvmeDevice dev;
    dev.set_features_num_queues_dw0 = 0x00070007u;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    const int64_t asq_bytes = static_cast<int64_t>(dev.asqs) *
                              static_cast<int64_t>(sizeof(SubmissionEntry));
    const uint8_t* const asq_ptr = dev.PhysToPtr(dev.asq, asq_bytes);
    ASSERT_NE(asq_ptr, nullptr);
    int sf_found = 0;
    for (int i = 0; i < dev.asq_head; ++i) {
      SubmissionEntry sqe{};
      const int64_t off = static_cast<int64_t>(i) *
                          static_cast<int64_t>(sizeof(SubmissionEntry));
      std::memcpy(&sqe, asq_ptr + off, sizeof(sqe));
      if (static_cast<uint8_t>(sqe.cdw0 & 0xFFu) == kAdminSetFeatures) {
        ++sf_found;
        EXPECT_EQ(sqe.cdw10, kFeatureNumberOfQueues);
        EXPECT_EQ(sqe.cdw11, 0u);
      }
    }
    EXPECT_EQ(sf_found, 1);
  }
  {
    sim::SimNvmeDevice dev;
    dev.set_features_num_queues_dw0 = 0xFFFF0000u;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kCommandFailed);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.cc, 0u);
  }
  {
    sim::SimNvmeDevice dev;
    dev.set_features_num_queues_dw0 = 0x0000FFFFu;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kCommandFailed);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.cc, 0u);
  }
  // N37: 0xFFFEFFFEu (max legal zero-based queue count) must succeed.
  {
    sim::SimNvmeDevice dev;
    dev.set_features_num_queues_dw0 = 0xFFFEFFFEu;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.state(), NvmeState::kRunning);
  }
  {
    FakeNvmeHost fake;
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_EQ(fake.set_features_count, 1);
    EXPECT_THAT(fake.set_features_cdw11, t::ElementsAre(0u));
  }
}

TEST(NvmeControllerTest, Cns2CursorPaginationSkipsInvalidAndHandlesOver1024Ns) {
  // 12 invalid + 10 valid + broadcast 0xFFFFFFFF -> exact first 8 valid.
  {
    sim::SimNvmeDevice dev;
    dev.namespaces.clear();
    {
      sim::SimNvmeDevice::NamespaceConfig bcast;
      bcast.nsid = 0xFFFFFFFFu;
      dev.namespaces.push_back(std::move(bcast));
    }
    for (uint32_t id = 1; id <= 12; ++id) {
      sim::SimNvmeDevice::NamespaceConfig inv;
      inv.nsid = id;
      inv.block_count = (id % 3 == 0) ? 0 : 64;
      inv.metadata_size = (id % 3 == 1) ? 8 : 0;
      inv.lbads_override = (id % 3 == 2) ? 13u : 0u;
      dev.namespaces.push_back(std::move(inv));
    }
    for (uint32_t id = 13; id <= 22; ++id) {
      sim::SimNvmeDevice::NamespaceConfig ok;
      ok.nsid = id;
      ok.block_count = 16;
      ok.block_size = 512;
      dev.namespaces.push_back(std::move(ok));
    }
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.namespace_count(), 8);
    EXPECT_EQ(ctrl.ns(0).nsid, 13u);
    EXPECT_EQ(ctrl.ns(7).nsid, 20u);
  }

  // > 1024 active namespaces: 1028 invalid followed by valid 1029 and 1030.
  {
    sim::SimNvmeDevice dev;
    dev.namespaces.clear();
    dev.namespaces.reserve(1030);
    for (uint32_t id = 1; id <= 1028; ++id) {
      sim::SimNvmeDevice::NamespaceConfig inv;
      inv.nsid = id;
      inv.block_count = 64;
      inv.metadata_size = 8;
      dev.namespaces.push_back(std::move(inv));
    }
    for (uint32_t id = 1029; id <= 1030; ++id) {
      sim::SimNvmeDevice::NamespaceConfig ok;
      ok.nsid = id;
      ok.block_count = 32;
      ok.block_size = 512;
      ok.InitStorage();
      dev.namespaces.push_back(std::move(ok));
    }
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.namespace_count(), 2);
    EXPECT_EQ(ctrl.ns(0).nsid, 1029u);
    EXPECT_EQ(ctrl.ns(1).nsid, 1030u);
    EXPECT_LT(dev.admin_commands_seen, 1100);
  }

  // Corrupt non-monotonic CNS=2 response must not hang.
  {
    sim::SimNvmeDevice dev;
    dev.namespaces.clear();
    for (const uint32_t id : {5u, 10u}) {
      sim::SimNvmeDevice::NamespaceConfig ns;
      ns.nsid = id;
      ns.block_count = 16;
      ns.block_size = 512;
      dev.namespaces.push_back(std::move(ns));
    }
    dev.corrupt_cns2_nonmonotonic = true;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.namespace_count(), 1);
    EXPECT_EQ(ctrl.ns(0).nsid, 10u);
  }
}

TEST(NvmeControllerTest, LegacyFallbackProbesBeyond16Namespaces) {
  sim::SimNvmeDevice dev;
  dev.namespaces.clear();
  sim::SimNvmeDevice::NamespaceConfig ns20;
  ns20.nsid = 20;
  ns20.block_count = 32;
  ns20.block_size = 4096;
  ns20.InitStorage();
  dev.namespaces.push_back(std::move(ns20));
  dev.support_active_ns_list = false;
  dev.nn_override = 20;

  NvmeController ctrl;
  ctrl.Init(dev, dev);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.namespace_count(), 1);
  EXPECT_EQ(ctrl.ns(0).nsid, 20u);
  EXPECT_EQ(ctrl.ns(0).block_size, 4096);
  EXPECT_EQ(ctrl.ns(0).max_transfer_blocks, 1);

  uint8_t block[4096];
  std::memset(block, 0xCD, sizeof(block));
  EXPECT_EQ(ctrl.WriteBlocks(0, 3, 1, block), NvmeStatus::kOk);
  uint8_t back[4096] = {};
  EXPECT_EQ(ctrl.ReadBlocks(0, 3, 1, back), NvmeStatus::kOk);
  EXPECT_EQ(std::memcmp(block, back, sizeof(block)), 0);

  // ADV_M2_R3_18 / HUNT_16: legacy 1..NN probe must stop strictly at 1024u.
  {
    FakeNvmeHost fake;
    fake.support_cns2 = false;
    fake.nn_override = 1026;
    fake.present_nsids = {1025u};
    NvmeController cap_ctrl;
    cap_ctrl.Init(fake, fake);
    ASSERT_EQ(cap_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(cap_ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_EQ(cap_ctrl.namespace_count(), 0);
    EXPECT_EQ(fake.max_probed_nsid, 1024u);
  }

  // gcov nvme_controller.h:863 br 3: legacy 1..NN stops at kMaxNamespaces = 8.
  {
    sim::SimNvmeDevice leg_dev;
    leg_dev.namespaces.clear();
    for (uint32_t id = 1; id <= 10; ++id) {
      sim::SimNvmeDevice::NamespaceConfig ns;
      ns.nsid = id;
      ns.block_count = 16;
      ns.block_size = 512;
      leg_dev.namespaces.push_back(std::move(ns));
    }
    leg_dev.support_active_ns_list = false;
    leg_dev.nn_override = 10;
    NvmeController leg_ctrl;
    leg_ctrl.Init(leg_dev, leg_dev);
    ASSERT_EQ(leg_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(leg_ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_EQ(leg_ctrl.namespace_count(), NvmeController::kMaxNamespaces);
    EXPECT_EQ(leg_ctrl.ns(0).nsid, 1u);
    EXPECT_EQ(leg_ctrl.ns(7).nsid, 8u);
  }
  // C_C861_NN_MIN: legacy 1..NN probe honors NN == 0 and NN < 1024 bounds.
  {
    FakeNvmeHost fake;
    fake.support_cns2 = false;
    fake.force_nn_zero = true;
    fake.present_nsids = {1u};
    NvmeController zero_nn_ctrl;
    zero_nn_ctrl.Init(fake, fake);
    ASSERT_EQ(zero_nn_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(zero_nn_ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_EQ(zero_nn_ctrl.namespace_count(), 0);
    EXPECT_EQ(fake.max_probed_nsid, 0u);
  }
  {
    FakeNvmeHost fake;
    fake.support_cns2 = false;
    fake.nn_override = 2;
    fake.present_nsids = {1u, 2u, 3u};
    NvmeController small_nn_ctrl;
    small_nn_ctrl.Init(fake, fake);
    ASSERT_EQ(small_nn_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(small_nn_ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(small_nn_ctrl.namespace_count(), 2);
    EXPECT_EQ(small_nn_ctrl.ns(0).nsid, 1u);
    EXPECT_EQ(small_nn_ctrl.ns(1).nsid, 2u);
    EXPECT_EQ(fake.max_probed_nsid, 2u);
  }
  // C_C726_NN_16: legacy 1..NN probe reads full 32-bit NN (bits [23:0] == 0).
  {
    FakeNvmeHost fake;
    fake.support_cns2 = false;
    fake.nn_override = 0x01000000u;
    fake.present_nsids = {1u};
    NvmeController high_nn_ctrl;
    high_nn_ctrl.Init(fake, fake);
    ASSERT_EQ(high_nn_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(high_nn_ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(high_nn_ctrl.namespace_count(), 1);
    EXPECT_EQ(high_nn_ctrl.ns(0).nsid, 1u);
    EXPECT_EQ(fake.max_probed_nsid, 1024u);
  }
}

TEST(NvmeControllerTest, FlbasBits65AndNlbafBounds) {
  sim::SimNvmeDevice dev;
  dev.namespaces.clear();

  // NS1: nlbaf = 20, flbas_index = 18 (uses FLBAS[6:5]).
  {
    sim::SimNvmeDevice::NamespaceConfig ns;
    ns.nsid = 1;
    ns.block_count = 16;
    ns.block_size = 4096;
    ns.nlbaf = 20;
    ns.flbas_index = 18;
    dev.namespaces.push_back(std::move(ns));
  }
  // NS2: nlbaf = 5 (< 16) with stray reserved FLBAS[6:5] bits -> ignored.
  {
    sim::SimNvmeDevice::NamespaceConfig ns;
    ns.nsid = 2;
    ns.block_count = 16;
    ns.block_size = 512;
    ns.nlbaf = 5;
    ns.flbas_index = 2;
    ns.raw_flbas_reserved = 0x20u;
    dev.namespaces.push_back(std::move(ns));
  }
  // NS3: nlbaf = 64 (>= 64) -> skipped.
  {
    sim::SimNvmeDevice::NamespaceConfig ns;
    ns.nsid = 3;
    ns.block_count = 16;
    ns.block_size = 512;
    ns.nlbaf = 64;
    ns.flbas_index = 0;
    dev.namespaces.push_back(std::move(ns));
  }
  // NS4: flbas_index > nlbaf -> skipped.
  {
    sim::SimNvmeDevice::NamespaceConfig ns;
    ns.nsid = 4;
    ns.block_count = 16;
    ns.block_size = 512;
    ns.nlbaf = 3;
    ns.flbas_index = 4;
    dev.namespaces.push_back(std::move(ns));
  }
  // NS5: exact boundary nlbaf = 16, flbas_index = 16 (kills WILD_04 > 16).
  {
    sim::SimNvmeDevice::NamespaceConfig ns;
    ns.nsid = 5;
    ns.block_count = 16;
    ns.block_size = 4096;
    ns.nlbaf = 16;
    ns.flbas_index = 16;
    dev.namespaces.push_back(std::move(ns));
  }
  // NS6: exact max valid nlbaf = 63, flbas_index = 63 (FLBAS[6:5] = 3).
  {
    sim::SimNvmeDevice::NamespaceConfig ns;
    ns.nsid = 6;
    ns.block_count = 16;
    ns.block_size = 512;
    ns.nlbaf = 63;
    ns.flbas_index = 63;
    dev.namespaces.push_back(std::move(ns));
  }
  // NS7: active namespace with lbads = 8 (256 B < 512 B) -> skipped.
  {
    sim::SimNvmeDevice::NamespaceConfig ns;
    ns.nsid = 7;
    ns.block_count = 16;
    ns.metadata_size = 0;
    ns.lbads_override = 8u;
    dev.namespaces.push_back(std::move(ns));
  }
  // NS8: active namespace with real lbads = 0 (1 B < 512 B) -> skipped.
  {
    sim::SimNvmeDevice::NamespaceConfig ns;
    ns.nsid = 8;
    ns.block_count = 16;
    ns.block_size = 1;
    ns.metadata_size = 0;
    ns.lbads_override = 0u;
    dev.namespaces.push_back(std::move(ns));
  }
  // NS9-NS10 (STEP2_EXT_01): lbads = 25u and 137u catch 4-bit/7-bit masks.
  for (const uint8_t bad_lbads : {uint8_t{25}, uint8_t{137}}) {
    sim::SimNvmeDevice::NamespaceConfig ns;
    ns.nsid = static_cast<uint32_t>(bad_lbads);
    ns.block_count = 16;
    ns.metadata_size = 0;
    ns.lbads_override = bad_lbads;
    dev.namespaces.push_back(std::move(ns));
  }
  // NS11-NS12 (ADV_M2_R3_13): metadata_size = 256 and 0x8000 catch 8-bit MS.
  for (const uint16_t bad_ms : {uint16_t{256}, uint16_t{0x8000}}) {
    sim::SimNvmeDevice::NamespaceConfig ns;
    ns.nsid = (bad_ms == 256u) ? 9u : 10u;
    ns.block_count = 16;
    ns.block_size = 512;
    ns.metadata_size = bad_ms;
    dev.namespaces.push_back(std::move(ns));
  }

  NvmeController ctrl;
  ctrl.Init(dev, dev);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.namespace_count(), 4);
  EXPECT_EQ(ctrl.ns(0).nsid, 1u);
  EXPECT_EQ(ctrl.ns(0).block_size, 4096);
  EXPECT_EQ(ctrl.ns(1).nsid, 2u);
  EXPECT_EQ(ctrl.ns(1).block_size, 512);
  EXPECT_EQ(ctrl.ns(2).nsid, 5u);
  EXPECT_EQ(ctrl.ns(2).block_size, 4096);
  EXPECT_EQ(ctrl.ns(3).nsid, 6u);
  EXPECT_EQ(ctrl.ns(3).block_size, 512);

  // H08: FLBAS[7] reserved bit must be masked out when nlbaf >= 16.
  {
    sim::SimNvmeDevice flbas7_dev;
    flbas7_dev.namespaces.clear();
    sim::SimNvmeDevice::NamespaceConfig ns;
    ns.nsid = 1;
    ns.block_count = 16;
    ns.block_size = 512;
    ns.nlbaf = 20;
    ns.flbas_index = 18;
    ns.raw_flbas_reserved = 0x80u;
    flbas7_dev.namespaces.push_back(std::move(ns));

    NvmeController flbas7_ctrl;
    flbas7_ctrl.Init(flbas7_dev, flbas7_dev);
    ASSERT_EQ(flbas7_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(flbas7_ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(flbas7_ctrl.namespace_count(), 1);
    EXPECT_EQ(flbas7_ctrl.ns(0).nsid, 1u);
    EXPECT_EQ(flbas7_ctrl.ns(0).block_size, 512);
  }
  // N56 (nlbaf=65), N57b (nlbaf=20, flbas_index=36), and WILD_M5_MS1 (ms=1).
  {
    sim::SimNvmeDevice edge_dev;
    edge_dev.namespaces.clear();
    {
      sim::SimNvmeDevice::NamespaceConfig ns;
      ns.nsid = 1;
      ns.block_count = 16;
      ns.block_size = 512;
      ns.nlbaf = 65;
      ns.flbas_index = 18;
      edge_dev.namespaces.push_back(std::move(ns));
    }
    {
      sim::SimNvmeDevice::NamespaceConfig ns;
      ns.nsid = 2;
      ns.block_count = 16;
      ns.block_size = 512;
      ns.nlbaf = 20;
      ns.flbas_index = 36;
      edge_dev.namespaces.push_back(std::move(ns));
    }
    {
      sim::SimNvmeDevice::NamespaceConfig ns;
      ns.nsid = 3;
      ns.block_count = 16;
      ns.block_size = 512;
      ns.metadata_size = 1;
      edge_dev.namespaces.push_back(std::move(ns));
    }
    {
      sim::SimNvmeDevice::NamespaceConfig ns;
      ns.nsid = 4;
      ns.block_count = 16;
      ns.block_size = 512;
      edge_dev.namespaces.push_back(std::move(ns));
    }
    NvmeController edge_ctrl;
    edge_ctrl.Init(edge_dev, edge_dev);
    ASSERT_EQ(edge_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(edge_ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(edge_ctrl.namespace_count(), 1);
    EXPECT_EQ(edge_ctrl.ns(0).nsid, 4u);
  }
  // SHUNT_30: FLBAS[4] (0x10u extended-LBA metadata bit) is ignored for both
  // nlbaf < 16 and nlbaf >= 16.
  for (const uint8_t nlbaf_val : {uint8_t{5}, uint8_t{20}}) {
    sim::SimNvmeDevice flbas4_dev;
    flbas4_dev.namespaces.clear();
    sim::SimNvmeDevice::NamespaceConfig ns;
    ns.nsid = 1;
    ns.block_count = 16;
    ns.block_size = 512;
    ns.nlbaf = nlbaf_val;
    ns.flbas_index = 2;
    ns.raw_flbas_reserved = 0x10u;
    flbas4_dev.namespaces.push_back(std::move(ns));

    NvmeController flbas4_ctrl;
    flbas4_ctrl.Init(flbas4_dev, flbas4_dev);
    ASSERT_EQ(flbas4_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(flbas4_ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(flbas4_ctrl.namespace_count(), 1);
    EXPECT_EQ(flbas4_ctrl.ns(0).nsid, 1u);
    EXPECT_EQ(flbas4_ctrl.ns(0).block_size, 512);
  }
  // WILD_R6_NLBAF_15: exact 16-format threshold (nlbaf = 15 < 16) must not
  // decode FLBAS[6:5].
  {
    sim::SimNvmeDevice nlbaf15_dev;
    nlbaf15_dev.namespaces.clear();
    sim::SimNvmeDevice::NamespaceConfig ns;
    ns.nsid = 1;
    ns.block_count = 16;
    ns.block_size = 512;
    ns.nlbaf = 15;
    ns.flbas_index = 2;
    ns.raw_flbas_reserved = 0x20u;
    nlbaf15_dev.namespaces.push_back(std::move(ns));

    NvmeController nlbaf15_ctrl;
    nlbaf15_ctrl.Init(nlbaf15_dev, nlbaf15_dev);
    ASSERT_EQ(nlbaf15_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(nlbaf15_ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(nlbaf15_ctrl.namespace_count(), 1);
    EXPECT_EQ(nlbaf15_ctrl.ns(0).nsid, 1u);
    EXPECT_EQ(nlbaf15_ctrl.ns(0).block_size, 512);
  }
  // M2R8-02 (HUNT_01): LBAF[31:24] Relative Performance (RP=1..3) bits must be
  // masked via `(lbaf >> 16) & 0xFFu` in ProbeNamespace().
  {
    class LbafRpProxy final : public Platform, public MmioWindow {
     public:
      LbafRpProxy(sim::SimNvmeDevice& inner, const uint8_t rp_byte)
          : inner_(inner), rp_byte_(rp_byte) {}

      int patched_cns0_count = 0;

      DmaBuffer DmaAlloc(const int64_t size, const bool below_4gib) override {
        const DmaBuffer buf = inner_.DmaAlloc(size, below_4gib);
        if (buf.va != nullptr) {
          allocs_.push_back(buf);
        }
        return buf;
      }

      int64_t NowTicks() override { return inner_.NowTicks(); }
      int64_t TicksPerSecond() override { return inner_.TicksPerSecond(); }
      void Relax() override { inner_.Relax(); }
      void WaitForIrq(const uint64_t irq_mask,
                      const int64_t timeout_ticks) override {
        inner_.WaitForIrq(irq_mask, timeout_ticks);
      }
      void Log(const char* const line, const int length) override {
        inner_.Log(line, length);
      }
      uint32_t Read32(const int64_t offset) override {
        return inner_.Read32(offset);
      }
      void Write32(const int64_t offset, const uint32_t value) override {
        bool patch_cns0 = false;
        uint64_t identify_phys = 0;
        const int64_t admin_db = SqTailDoorbellOffset(0, CapDstrd(inner_.cap));
        if (offset == admin_db && inner_.asqs > 0 && inner_.asq_head >= 0 &&
            inner_.asq_head < inner_.asqs) {
          const uint8_t* const sq_va =
              FindDmaVa(inner_.asq, NvmeController::kPageSize);
          if (sq_va != nullptr) {
            SubmissionEntry cmd{};
            const int64_t sq_off =
                static_cast<int64_t>(inner_.asq_head) *
                static_cast<int64_t>(sizeof(SubmissionEntry));
            std::memcpy(&cmd, sq_va + sq_off, sizeof(cmd));
            const uint8_t opc = static_cast<uint8_t>(cmd.cdw0 & 0xFFu);
            if (opc == kAdminIdentify && cmd.cdw10 == kIdentifyCnsNamespace) {
              patch_cns0 = true;
              identify_phys = cmd.prp1;
            }
          }
        }
        inner_.Write32(offset, value);
        if (patch_cns0) {
          uint8_t* const id_va = FindDmaVa(identify_phys, kIdentifyDataBytes);
          if (id_va != nullptr) {
            id_va[kIdNsLbaf0Offset + 3] = rp_byte_;
            ++patched_cns0_count;
          }
        }
      }
      int64_t Size() const override { return inner_.Size(); }
      bool Covers(const int64_t offset, const int64_t size) const override {
        return inner_.Covers(offset, size);
      }

     private:
      uint8_t* FindDmaVa(const uint64_t phys, const int64_t min_bytes) const {
        for (const DmaBuffer& buf : allocs_) {
          if (buf.va != nullptr && buf.phys == phys && buf.size >= min_bytes) {
            return static_cast<uint8_t*>(buf.va);
          }
        }
        return nullptr;
      }

      sim::SimNvmeDevice& inner_;
      const uint8_t rp_byte_;
      std::vector<DmaBuffer> allocs_;
    };

    for (const uint8_t rp_byte :
         {uint8_t{0x01}, uint8_t{0x02}, uint8_t{0x03}}) {
      sim::SimNvmeDevice rp_dev;
      LbafRpProxy proxy(rp_dev, rp_byte);
      NvmeController rp_ctrl;
      rp_ctrl.Init(proxy, proxy);
      ASSERT_EQ(rp_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk)
          << static_cast<int>(rp_byte);
      ASSERT_EQ(rp_ctrl.Start(1), NvmeStatus::kOk) << static_cast<int>(rp_byte);
      EXPECT_EQ(proxy.patched_cns0_count, 1) << static_cast<int>(rp_byte);
      ASSERT_EQ(rp_ctrl.namespace_count(), 1) << static_cast<int>(rp_byte);
      EXPECT_EQ(rp_ctrl.ns(0).nsid, 1u) << static_cast<int>(rp_byte);
      EXPECT_EQ(rp_ctrl.ns(0).block_size, 512) << static_cast<int>(rp_byte);
    }
  }
}

TEST(NvmeControllerTest, NsattrWriteProtectAndHardwareRoCqe) {
  // (a) NSATTR bit 0 advertises RO upfront: WriteBlocks rejects without I/O.
  {
    sim::SimNvmeDevice dev;
    dev.namespaces[0].read_only = true;
    dev.namespaces[0].advertise_nsattr_ro = true;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_TRUE(ctrl.ns(0).read_only);
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.WriteBlocks(0, 0, 1, sec), NvmeStatus::kReadOnly);
    EXPECT_EQ(dev.io_commands_seen, 0);
    EXPECT_EQ(ctrl.WriteBlocks(0, 0, 0, nullptr), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.state(), NvmeState::kRunning);
    // M2R8-08 (M04 / M04b): ValidateIoArgs bounds, nullptr, and !kRunning
    // precede read_only checks in WriteBlocks().
    const int io_before = dev.io_commands_seen;
    EXPECT_EQ(ctrl.WriteBlocks(0, 128, 1, sec), NvmeStatus::kInvalidArgument);
    EXPECT_EQ(ctrl.WriteBlocks(0, 0, 9, sec), NvmeStatus::kInvalidArgument);
    EXPECT_EQ(ctrl.WriteBlocks(0, 0, 1, nullptr), NvmeStatus::kInvalidArgument);
    EXPECT_EQ(ctrl.WriteBlocks(0, 0, -1, sec), NvmeStatus::kInvalidArgument);
    EXPECT_EQ(dev.io_commands_seen, io_before);
    ASSERT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.state(), NvmeState::kHalted);
    EXPECT_EQ(ctrl.WriteBlocks(0, 0, 1, sec), NvmeStatus::kNotRunning);
    EXPECT_EQ(ctrl.WriteBlocks(0, 0, 0, nullptr), NvmeStatus::kNotRunning);
    EXPECT_EQ(dev.io_commands_seen, io_before);
  }
  // (b) Hardware omits NSATTR RO bit, returns SCT=1/SC=0x82 CQE.
  {
    sim::SimNvmeDevice dev;
    dev.namespaces[0].read_only = true;
    dev.namespaces[0].advertise_nsattr_ro = false;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_FALSE(ctrl.ns(0).read_only);
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.WriteBlocks(0, 0, 1, sec), NvmeStatus::kReadOnly);
    EXPECT_EQ(dev.io_commands_seen, 1);
    EXPECT_EQ(ctrl.state(), NvmeState::kRunning);
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk);
  }
  // N58 / WILD_M5_NSATTR_MASK: reserved NSATTR bits [7:1] (0xFE) must not be
  // treated as read-only.
  {
    class NsAttrProxy final : public Platform, public MmioWindow {
     public:
      explicit NsAttrProxy(sim::SimNvmeDevice& inner) : inner_(inner) {}

      DmaBuffer DmaAlloc(const int64_t size, const bool below_4gib) override {
        const DmaBuffer buf = inner_.DmaAlloc(size, below_4gib);
        dma_vas_.push_back(buf.va);
        return buf;
      }

      int64_t NowTicks() override { return inner_.NowTicks(); }
      int64_t TicksPerSecond() override { return inner_.TicksPerSecond(); }
      void Relax() override { inner_.Relax(); }
      void WaitForIrq(const uint64_t irq_mask,
                      const int64_t timeout_ticks) override {
        inner_.WaitForIrq(irq_mask, timeout_ticks);
      }
      void Log(const char* const line, const int length) override {
        inner_.Log(line, length);
      }
      uint32_t Read32(const int64_t offset) override {
        return inner_.Read32(offset);
      }
      void Write32(const int64_t offset, const uint32_t value) override {
        const int before = inner_.admin_commands_seen;
        inner_.Write32(offset, value);
        if (before < 3 && inner_.admin_commands_seen >= 3 &&
            static_cast<int>(dma_vas_.size()) >= 5 && dma_vas_[4] != nullptr) {
          static_cast<uint8_t*>(dma_vas_[4])[kIdNsAttrOffset] = 0xFEu;
        }
      }
      int64_t Size() const override { return inner_.Size(); }
      bool Covers(const int64_t offset, const int64_t size) const override {
        return inner_.Covers(offset, size);
      }

     private:
      sim::SimNvmeDevice& inner_;
      std::vector<void*> dma_vas_;
    };

    sim::SimNvmeDevice dev;
    NsAttrProxy proxy(dev);
    NvmeController ctrl;
    ctrl.Init(proxy, proxy);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.namespace_count(), 1);
    EXPECT_FALSE(ctrl.ns(0).read_only);
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.WriteBlocks(0, 0, 1, sec), NvmeStatus::kOk);
  }
}

TEST(NvmeControllerTest, ZeroNamespacesSupportedCleanly) {
  sim::SimNvmeDevice dev;
  dev.namespaces.clear();
  NvmeController ctrl;
  ctrl.Init(dev, dev);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  EXPECT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  EXPECT_EQ(ctrl.state(), NvmeState::kRunning);
  EXPECT_EQ(ctrl.namespace_count(), 0);
}

TEST(NvmeControllerTest, ShutdownFollowsShnThenCcZeroAndAllowsReinit) {
  sim::SimNvmeDevice dev;
  NvmeController ctrl;
  ctrl.Init(dev, dev);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);

  EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
  EXPECT_EQ(ctrl.state(), NvmeState::kHalted);
  EXPECT_TRUE(dev.saw_shn_normal);
  EXPECT_FALSE(dev.saw_shn_abrupt);
  EXPECT_EQ(dev.shutdown_count, 1);
  EXPECT_TRUE(dev.saw_cc_zero_after_shn);
  EXPECT_EQ(dev.cc & kCcEn, 0u);
  EXPECT_EQ(dev.csts & kCstsRdy, 0u);
  EXPECT_STREQ(ctrl.model(), "SimNVMe 1.4");
  EXPECT_EQ(ctrl.namespace_count(), 1);

  // Clean kHalted Shutdown() (cc == 0 && (csts & kCstsRdy) == 0).
  ASSERT_EQ(dev.cc, 0u);
  ASSERT_EQ(dev.csts & kCstsRdy, 0u);
  EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
  EXPECT_EQ(ctrl.state(), NvmeState::kHalted);

  // Idempotent on kHalted (clearing any stale CC) and clean re-bring-up.
  dev.cc = kCcEn;
  EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
  EXPECT_EQ(dev.cc, 0u);
  // NEW_HALTED_CC_EN_ONLY / NEW_DISABLE_CC_EN_ONLY: stale non-EN CC bits in
  // kHalted still trigger DisableControllerBestEffort and write CC = 0.
  dev.cc = kCcShnNormal;
  dev.csts = 0;
  EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
  EXPECT_EQ(ctrl.state(), NvmeState::kHalted);
  EXPECT_EQ(dev.cc, 0u);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  EXPECT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  EXPECT_EQ(ctrl.state(), NvmeState::kRunning);

  // Shutdown preserves kError while resetting CC=0.
  dev.hang_io_cmd = true;
  uint8_t sec[512] = {};
  EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kTimeout);
  EXPECT_EQ(ctrl.state(), NvmeState::kError);
  EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
  EXPECT_EQ(ctrl.state(), NvmeState::kError);
  EXPECT_EQ(dev.cc, 0u);
  // N41, N42, & NEW_DISABLE_CC_EN_ONLY: kError Shutdown() invokes
  // DisableControllerBestEffort (even for non-EN CC), clears CC, and
  // propagates disable timeout errors.
  dev.hang_io_cmd = false;
  dev.hang_disable = true;
  dev.cc = kCcEn;
  dev.csts = kCstsRdy;
  EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kTimeout);
  EXPECT_EQ(ctrl.state(), NvmeState::kError);

  dev.hang_disable = false;
  dev.cc = kCcEn;
  dev.csts = kCstsRdy;
  EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
  EXPECT_EQ(ctrl.state(), NvmeState::kError);
  EXPECT_EQ(dev.cc, 0u);
  EXPECT_EQ(dev.csts & kCstsRdy, 0u);

  dev.cc = kCcShnNormal;
  dev.csts = 0;
  EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
  EXPECT_EQ(ctrl.state(), NvmeState::kError);
  EXPECT_EQ(dev.cc, 0u);

  // Re-Start() after prior I/O must reset io_sq_tail_, io_cq_head_, and
  // io_cq_phase_ across queue wrap (35 > kQueueDepth 32).
  {
    sim::SimNvmeDevice reinit_dev;
    reinit_dev.namespaces[0].storage[0] = 0x5A;
    NvmeController reinit_ctrl;
    reinit_ctrl.Init(reinit_dev, reinit_dev);
    ASSERT_EQ(reinit_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(reinit_ctrl.Start(1), NvmeStatus::kOk);
    uint8_t sec[512] = {};
    for (int i = 0; i < 35; ++i) {
      ASSERT_EQ(reinit_ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk) << i;
    }
    EXPECT_EQ(sec[0], 0x5A);
    ASSERT_EQ(reinit_ctrl.Shutdown(), NvmeStatus::kOk);
    ASSERT_EQ(reinit_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(reinit_ctrl.Start(1), NvmeStatus::kOk);
    sec[0] = 0;
    const int io_before = reinit_dev.io_commands_seen;
    EXPECT_EQ(reinit_ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk);
    EXPECT_EQ(reinit_dev.io_commands_seen, io_before + 1);
    EXPECT_EQ(sec[0], 0x5A);
  }
  // H05: Start() resets admin_cq_phase_ = 1 after an odd Admin CQ wrap
  // (CAP.MQES=3 => depth 4; 6 Start() commands flip phase to 0).
  {
    sim::SimNvmeDevice admin_dev;
    admin_dev.cap = (3ULL << 0) | (2ULL << 24) | kCapCssNvm;
    NvmeController admin_ctrl;
    admin_ctrl.Init(admin_dev, admin_dev);
    ASSERT_EQ(admin_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(admin_ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(admin_ctrl.Shutdown(), NvmeStatus::kOk);
    ASSERT_EQ(admin_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(admin_ctrl.Start(1), NvmeStatus::kOk);
    uint8_t admin_sec[512] = {};
    EXPECT_EQ(admin_ctrl.ReadBlocks(0, 0, 1, admin_sec), NvmeStatus::kOk);
  }
  // H13d: Start() clears has_volatile_write_cache() and cached identify
  // fields even when re-Start() fails before IdentifyController().
  {
    sim::SimNvmeDevice vwc_dev;
    vwc_dev.model = "OldCtrl";
    vwc_dev.serial = "OLD001";
    NvmeController vwc_ctrl;
    vwc_ctrl.Init(vwc_dev, vwc_dev);
    ASSERT_EQ(vwc_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(vwc_ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_TRUE(vwc_ctrl.has_volatile_write_cache());
    ASSERT_EQ(vwc_ctrl.namespace_count(), 1);
    ASSERT_STREQ(vwc_ctrl.model(), "OldCtrl");
    ASSERT_STREQ(vwc_ctrl.serial(), "OLD001");
    ASSERT_EQ(vwc_ctrl.Shutdown(), NvmeStatus::kOk);
    ASSERT_EQ(vwc_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    vwc_dev.hang_enable = true;
    EXPECT_EQ(vwc_ctrl.Start(1), NvmeStatus::kTimeout);
    EXPECT_EQ(vwc_ctrl.state(), NvmeState::kError);
    EXPECT_FALSE(vwc_ctrl.has_volatile_write_cache());
    EXPECT_EQ(vwc_ctrl.namespace_count(), 0);
    EXPECT_FALSE(vwc_ctrl.ns(0).valid);
    EXPECT_STREQ(vwc_ctrl.model(), "");
    EXPECT_STREQ(vwc_ctrl.serial(), "");
  }
  // C_C189_ZERO_ACQ: Re-Start() must zero stale P==1 Admin CQEs.
  {
    sim::SimNvmeDevice acq_dev;
    NvmeController acq_ctrl;
    acq_ctrl.Init(acq_dev, acq_dev);
    ASSERT_EQ(acq_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(acq_ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(acq_ctrl.Shutdown(), NvmeStatus::kOk);
    ASSERT_EQ(acq_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    acq_dev.hang_admin_cmd = true;
    EXPECT_EQ(acq_ctrl.Start(1), NvmeStatus::kTimeout);
    EXPECT_EQ(acq_ctrl.state(), NvmeState::kError);
  }
  // C_C191_ZERO_ICQ: Re-Start() must zero stale P==1 I/O CQEs.
  {
    sim::SimNvmeDevice icq_dev;
    NvmeController icq_ctrl;
    icq_ctrl.Init(icq_dev, icq_dev);
    ASSERT_EQ(icq_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(icq_ctrl.Start(1), NvmeStatus::kOk);
    uint8_t icq_sec[512] = {};
    ASSERT_EQ(icq_ctrl.ReadBlocks(0, 0, 1, icq_sec), NvmeStatus::kOk);
    ASSERT_EQ(icq_ctrl.Shutdown(), NvmeStatus::kOk);
    ASSERT_EQ(icq_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(icq_ctrl.Start(1), NvmeStatus::kOk);
    icq_dev.hang_io_cmd = true;
    EXPECT_EQ(icq_ctrl.ReadBlocks(0, 0, 1, icq_sec), NvmeStatus::kTimeout);
    EXPECT_EQ(icq_ctrl.state(), NvmeState::kError);
  }
  // M2R7-08 (SHUNT_69 / WILD_R6_REALLOC_DMA_ON_RESET): Re-TakeOwnershipAndReset
  // after Shutdown reuses cached DMA buffers without new allocations.
  {
    sim::SimNvmeDevice dma_dev;
    dma_dev.dma_alloc_limit = 6;
    NvmeController dma_ctrl;
    dma_ctrl.Init(dma_dev, dma_dev);
    ASSERT_EQ(dma_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(dma_dev.dma_allocs_done, 6);
    ASSERT_EQ(dma_ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(dma_ctrl.Shutdown(), NvmeStatus::kOk);
    EXPECT_EQ(dma_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(dma_dev.dma_allocs_done, 6);
    EXPECT_EQ(dma_ctrl.Start(1), NvmeStatus::kOk);
    uint8_t sec[512] = {};
    EXPECT_EQ(dma_ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk);
  }
  // M2R7-16 (SHUNT_72, SHUNT_73, SHUNT_74): Re-Start() must scrub cached
  // admin_sq_, io_sq_, and io_buf_ DMA pages (SHUNT_75 identify_buf_ stays
  // omitted because Identify* memset-zeros all 4096 bytes before any use).
  {
    class DmaTrackerPlatform final : public Platform {
     public:
      explicit DmaTrackerPlatform(Platform& inner) : inner_(inner) {}

      std::vector<DmaBuffer> allocs;

      DmaBuffer DmaAlloc(const int64_t size, const bool below_4gib) override {
        const DmaBuffer buf = inner_.DmaAlloc(size, below_4gib);
        if (buf.va != nullptr) {
          allocs.push_back(buf);
        }
        return buf;
      }

      int64_t NowTicks() override { return inner_.NowTicks(); }
      int64_t TicksPerSecond() override { return inner_.TicksPerSecond(); }
      void Relax() override { inner_.Relax(); }
      void WaitForIrq(const uint64_t irq_mask,
                      const int64_t timeout_ticks) override {
        inner_.WaitForIrq(irq_mask, timeout_ticks);
      }
      void Log(const char* const line, const int length) override {
        inner_.Log(line, length);
      }

     private:
      Platform& inner_;
    };

    sim::SimNvmeDevice scrub_dev;
    DmaTrackerPlatform tracker(scrub_dev);
    NvmeController scrub_ctrl;
    scrub_ctrl.Init(tracker, scrub_dev);
    ASSERT_EQ(scrub_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(static_cast<int>(tracker.allocs.size()), 6);
    ASSERT_EQ(scrub_ctrl.Start(1), NvmeStatus::kOk);

    uint8_t dirty_sec[512] = {};
    std::memset(dirty_sec, 0xA5, sizeof(dirty_sec));
    ASSERT_EQ(scrub_ctrl.WriteBlocks(0, 0, 1, dirty_sec), NvmeStatus::kOk);

    const auto has_nonzero = [](const DmaBuffer& buf) -> bool {
      const uint8_t* const bytes = static_cast<const uint8_t*>(buf.va);
      for (int64_t i = 0; i < buf.size; ++i) {
        if (bytes[i] != 0) {
          return true;
        }
      }
      return false;
    };
    // EnsureDmaBuffers allocates: 0=admin_sq_, 1=admin_cq_, 2=io_sq_,
    // 3=io_cq_, 4=identify_buf_, 5=io_buf_.
    ASSERT_TRUE(has_nonzero(tracker.allocs[0]));
    ASSERT_TRUE(has_nonzero(tracker.allocs[2]));
    ASSERT_TRUE(has_nonzero(tracker.allocs[5]));

    ASSERT_EQ(scrub_ctrl.Shutdown(), NvmeStatus::kOk);
    ASSERT_EQ(scrub_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    bool in_session2 = false;
    bool checked_at_enable = false;
    scrub_dev.on_read32 = [&](const int64_t offset, const uint32_t /*val*/) {
      if (in_session2 && !checked_at_enable && offset == kRegCsts &&
          (scrub_dev.cc & kCcEn) != 0) {
        checked_at_enable = true;
        EXPECT_FALSE(has_nonzero(tracker.allocs[0]));
        EXPECT_FALSE(has_nonzero(tracker.allocs[2]));
        EXPECT_FALSE(has_nonzero(tracker.allocs[5]));
      }
    };
    scrub_dev.namespaces.clear();
    in_session2 = true;
    ASSERT_EQ(scrub_ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_TRUE(checked_at_enable);
    EXPECT_FALSE(has_nonzero(tracker.allocs[2]));
    EXPECT_FALSE(has_nonzero(tracker.allocs[5]));
    const uint8_t* const admin_sq_bytes =
        static_cast<const uint8_t*>(tracker.allocs[0].va);
    for (int64_t i = 5 * static_cast<int64_t>(sizeof(SubmissionEntry));
         i < tracker.allocs[0].size; ++i) {
      EXPECT_EQ(admin_sq_bytes[i], 0) << i;
    }
  }
}

TEST(NvmeControllerTest, ShutdownHandlesHangAndDetachedDevice) {
  {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kInvalidArgument);
    EXPECT_EQ(ctrl.state(), NvmeState::kUninitialized);
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kInvalidArgument);
    EXPECT_EQ(ctrl.state(), NvmeState::kUninitialized);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    dev.hang_shutdown = true;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kTimeout);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.cc, 0u);
  }
  // WaitShutdownComplete clamps wait_ticks == 1 when TicksPerSecond() == 0 and
  // checks >= deadline before Relax().
  {
    sim::SimNvmeDevice dev;
    dev.ticks_per_second = 0;
    dev.enable_delay_ticks = 1;
    dev.disable_delay_ticks = 1;
    dev.shutdown_delay_ticks = 2;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kTimeout);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  // NEW_SHN_WAIT_ZERO: TicksPerSecond() == 0 clamps wait_ticks to 1 so a
  // 1-tick shutdown succeeds.
  {
    sim::SimNvmeDevice dev;
    dev.ticks_per_second = 0;
    dev.enable_delay_ticks = 1;
    dev.disable_delay_ticks = 1;
    dev.shutdown_delay_ticks = 1;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.state(), NvmeState::kHalted);
  }
  // Normal SHN completes, then CC=0 disable times out -> kTimeout, kError.
  {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    dev.hang_disable = true;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kTimeout);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_TRUE(dev.saw_shn_normal);
    EXPECT_EQ(dev.cc, 0u);
  }
  // N52: illegal CSTS.SHST == 11b must not be accepted as Complete (10b).
  {
    class ShstIllegalMmio final : public MmioWindow {
     public:
      ShstIllegalMmio(MmioWindow& inner, const uint32_t* const cc_ptr)
          : inner_(inner), cc_ptr_(cc_ptr) {}

      uint32_t Read32(const int64_t offset) override {
        const uint32_t v = inner_.Read32(offset);
        if (offset == kRegCsts && (*cc_ptr_ & kCcShnMask) != 0) {
          return (v & ~kCstsShstMask) | kCstsShstMask;
        }
        return v;
      }

      void Write32(const int64_t offset, const uint32_t value) override {
        inner_.Write32(offset, value);
      }

      int64_t Size() const override { return inner_.Size(); }

      bool Covers(const int64_t offset, const int64_t size) const override {
        return inner_.Covers(offset, size);
      }

     private:
      MmioWindow& inner_;
      const uint32_t* const cc_ptr_;
    };

    sim::SimNvmeDevice dev;
    dev.ticks_per_second = 10;
    dev.shutdown_delay_ticks = 50;
    ShstIllegalMmio mmio(dev, &dev.cc);
    NvmeController ctrl;
    ctrl.Init(dev, mmio);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kTimeout);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    dev.not_decoding = true;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kNotDecoding);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }

  // ADV_M2_R3_07: kRunning Shutdown detects half-all-ones MMIO (CC alone or
  // CSTS alone).
  {
    sim::SimNvmeDevice dev;
    HalfAllOnesMmio mmio(dev);
    NvmeController ctrl;
    ctrl.Init(dev, mmio);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    mmio.spoof_offset = kRegCc;
    mmio.spoof_reads_left = 1;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kNotDecoding);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_FALSE(dev.saw_shn_normal);
    EXPECT_EQ(dev.shutdown_count, 0);
  }
  {
    sim::SimNvmeDevice dev;
    HalfAllOnesMmio mmio(dev);
    NvmeController ctrl;
    ctrl.Init(dev, mmio);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    mmio.spoof_offset = kRegCsts;
    mmio.spoof_reads_left = -1;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kNotDecoding);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_FALSE(dev.saw_shn_normal);
  }
  // HUNT_05: Shutdown CC.SHN write preserves IOSQES/IOCQES/CSS/MPS/AMS.
  {
    sim::SimNvmeDevice dev;
    CcWriteSpyMmio spy(dev);
    NvmeController ctrl;
    ctrl.Init(dev, spy);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    uint32_t cc_during_shn = 0;
    dev.on_read32 = [&](const int64_t offset, const uint32_t /*val*/) {
      if (offset == kRegCsts && (dev.cc & kCcShnMask) != 0 &&
          cc_during_shn == 0) {
        cc_during_shn = dev.cc;
      }
    };
    spy.cc_writes.clear();
    ASSERT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
    const uint32_t expected_shn_cc = kCcEn | kCcCssNvm | kCcMps4K |
                                     kCcAmsRoundRobin | kCcIoSqes64 |
                                     kCcIoCqes16 | kCcShnNormal;
    EXPECT_EQ(cc_during_shn, expected_shn_cc);
    EXPECT_NE(cc_during_shn & kCcIoSqes64, 0u);
    EXPECT_NE(cc_during_shn & kCcIoCqes16, 0u);
    EXPECT_THAT(spy.cc_writes, t::ElementsAre(expected_shn_cc, 0u));
  }
  // N44: Shutdown clears pre-existing CC.SHN bits (10b abrupt) before setting
  // kCcShnNormal (01b) instead of OR-ing into invalid 11b.
  {
    class ShnAbruptProxyMmio final : public MmioWindow {
     public:
      explicit ShnAbruptProxyMmio(MmioWindow& inner) : inner_(inner) {}

      uint32_t Read32(const int64_t offset) override {
        const uint32_t v = inner_.Read32(offset);
        return (offset == kRegCc) ? (v | kCcShnAbrupt) : v;
      }

      void Write32(const int64_t offset, const uint32_t value) override {
        inner_.Write32(offset, value);
      }

      int64_t Size() const override { return inner_.Size(); }

      bool Covers(const int64_t offset, const int64_t size) const override {
        return inner_.Covers(offset, size);
      }

     private:
      MmioWindow& inner_;
    };

    sim::SimNvmeDevice dev;
    CcWriteSpyMmio spy(dev);
    ShnAbruptProxyMmio proxy(spy);
    NvmeController ctrl;
    ctrl.Init(dev, proxy);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    spy.cc_writes.clear();
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.state(), NvmeState::kHalted);
    EXPECT_TRUE(dev.saw_shn_normal);
    ASSERT_FALSE(spy.cc_writes.empty());
    EXPECT_EQ(spy.cc_writes[0] & kCcShnMask, kCcShnNormal);
  }
  // ADV_M2_R3_08 / HUNT_06..08: kRunning Shutdown skips CC.SHN if either
  // CC.EN == 0 or CSTS.RDY == 0.
  {
    sim::SimNvmeDevice dev;
    CcWriteSpyMmio spy(dev);
    NvmeController ctrl;
    ctrl.Init(dev, spy);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    spy.cc_writes.clear();
    dev.cc = 0;
    dev.csts = kCstsRdy;
    dev.hang_shutdown = true;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.state(), NvmeState::kHalted);
    EXPECT_FALSE(dev.saw_shn_normal);
    EXPECT_TRUE(spy.cc_writes.empty());
    EXPECT_EQ(dev.csts & kCstsRdy, 0u);
  }
  {
    sim::SimNvmeDevice dev;
    CcWriteSpyMmio spy(dev);
    NvmeController ctrl;
    ctrl.Init(dev, spy);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    spy.cc_writes.clear();
    dev.cc = kCcEn | kCcIoSqes64 | kCcIoCqes16;
    dev.csts = 0;
    dev.hang_shutdown = true;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.state(), NvmeState::kHalted);
    EXPECT_FALSE(dev.saw_shn_normal);
    EXPECT_THAT(spy.cc_writes, t::ElementsAre(0u));
    EXPECT_EQ(dev.cc, 0u);
  }
  // Dynamic MMIO holes over CC/CSTS in Shutdown() &
  // DisableControllerBestEffort (tight 4-byte Covers() spans).
  for (const int64_t reg_off : {kRegCc, kRegCsts}) {
    {
      sim::SimNvmeDevice dev;
      NvmeController ctrl;
      ctrl.Init(dev, dev);
      ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
      ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
      dev.SetSizeAndHole(0x2000, reg_off + 4, reg_off + 8);
      EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kOk) << reg_off;
      EXPECT_EQ(ctrl.state(), NvmeState::kHalted) << reg_off;
    }
    for (const int64_t hole_start : {reg_off, reg_off + 3}) {
      sim::SimNvmeDevice dev;
      NvmeController ctrl;
      ctrl.Init(dev, dev);
      ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
      ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
      dev.SetSizeAndHole(0x2000, hole_start, reg_off + 4);
      int mmio_reads = 0;
      dev.on_read32 = [&](const int64_t /*offset*/, const uint32_t /*val*/) {
        ++mmio_reads;
      };
      EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kNotDecoding) << reg_off;
      EXPECT_EQ(ctrl.state(), NvmeState::kError) << reg_off;
      EXPECT_EQ(mmio_reads, 0) << reg_off;
    }
    {
      sim::SimNvmeDevice dev;
      NvmeController ctrl;
      ctrl.Init(dev, dev);
      ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
      ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
      dev.hang_io_cmd = true;
      dev.SetSizeAndHole(0x2000, reg_off + 4, reg_off + 8);
      uint8_t sec[512] = {};
      EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kTimeout) << reg_off;
      EXPECT_EQ(ctrl.state(), NvmeState::kError) << reg_off;
      EXPECT_EQ(dev.cc, 0u) << reg_off;
    }
    for (const int64_t hole_start : {reg_off, reg_off + 3}) {
      sim::SimNvmeDevice dev;
      NvmeController ctrl;
      ctrl.Init(dev, dev);
      ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
      ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
      dev.hang_io_cmd = true;
      const int64_t io_start = dev.now_ticks;
      const int64_t io_wait =
          TicksFromMs(NvmeController::kCommandTimeoutMs, dev.ticks_per_second);
      bool hole_armed = false;
      int reads_after_hole = 0;
      dev.on_read32 = [&](const int64_t offset, const uint32_t /*val*/) {
        if (hole_armed) {
          ++reads_after_hole;
          return;
        }
        if (offset == kRegCsts && dev.io_commands_seen > 0 &&
            (dev.now_ticks - io_start) >= io_wait) {
          dev.SetSizeAndHole(0x2000, hole_start, reg_off + 4);
          hole_armed = true;
        }
      };
      uint8_t sec[512] = {};
      EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kTimeout) << reg_off;
      EXPECT_EQ(ctrl.state(), NvmeState::kError) << reg_off;
      EXPECT_TRUE(hole_armed) << reg_off;
      EXPECT_EQ(reads_after_hole, 0) << reg_off;
      EXPECT_NE(dev.cc, 0u) << reg_off;
    }
  }
  // ADV_M2_R3_04: Re-Init from kRunning and kError resets state_ and metadata.
  for (const bool from_error : {false, true}) {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    if (from_error) {
      dev.hang_io_cmd = true;
      uint8_t sec[512] = {};
      EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kTimeout);
      ASSERT_EQ(ctrl.state(), NvmeState::kError);
    } else {
      ASSERT_EQ(ctrl.state(), NvmeState::kRunning);
    }
    ASSERT_EQ(ctrl.namespace_count(), 1);
    ASSERT_TRUE(ctrl.has_volatile_write_cache());

    sim::SimNvmeDevice fresh_dev;
    ctrl.Init(fresh_dev, fresh_dev);
    EXPECT_EQ(ctrl.state(), NvmeState::kUninitialized);
    EXPECT_EQ(ctrl.namespace_count(), 0);
    EXPECT_STREQ(ctrl.model(), "");
    EXPECT_STREQ(ctrl.serial(), "");
    EXPECT_FALSE(ctrl.has_volatile_write_cache());
    EXPECT_FALSE(ctrl.ns(0).valid);
    EXPECT_EQ(ctrl.ns(0).nsid, 0u);
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kInvalidArgument);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kInvalidArgument);
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kNotRunning);
    EXPECT_EQ(ctrl.WriteBlocks(0, 0, 1, sec), NvmeStatus::kNotRunning);
    EXPECT_EQ(ctrl.Flush(0), NvmeStatus::kNotRunning);
  }
  // WB12b: Re-Init() resets enable_timeout_ms_ = 2000 before failing
  // TakeOwnershipAndReset() so error-state Shutdown() avoids stale CAP.TO.
  {
    sim::SimNvmeDevice slow_dev;
    slow_dev.cap = (63ULL << 0) | (255ULL << 24) | kCapCssNvm;
    NvmeController ctrl;
    ctrl.Init(slow_dev, slow_dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);

    sim::SimNvmeDevice bad_dev;
    bad_dev.ticks_per_second = 1000;
    bad_dev.ResetRegisters(/*initially_enabled=*/true);
    bad_dev.cap = 0;
    bad_dev.hang_disable = true;
    ctrl.Init(bad_dev, bad_dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kBadCapabilities);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    const int64_t ticks_before = bad_dev.now_ticks;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kTimeout);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(bad_dev.cc, 0u);
    EXPECT_EQ(bad_dev.now_ticks - ticks_before, 2001);

    sim::SimNvmeDevice ok_dev;
    ok_dev.ticks_per_second = 10;
    ok_dev.disable_delay_ticks = 15;
    ok_dev.ResetRegisters(/*initially_enabled=*/true);
    ok_dev.cap = 0;
    ctrl.Init(ok_dev, ok_dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kBadCapabilities);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(ok_dev.cc, 0u);
    EXPECT_EQ(ok_dev.csts & kCstsRdy, 0u);
  }
}

TEST(NvmeControllerTest, RejectsBadCqeCidAndSqIdBeforeDoorbellAndCqeStatus) {
  // Bad CID must fail before CQHDBL advance and clear CC=0.
  {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    const int cqh_before = dev.io_cq_head_doorbell_writes;
    dev.corrupt_cqe_cid = true;
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kCommandFailed);
    EXPECT_EQ(dev.io_cq_head_doorbell_writes, cqh_before);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.cc, 0u);
  }
  // Bad SQ ID must fail before CQHDBL advance and clear CC=0.
  {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    const int cqh_before = dev.io_cq_head_doorbell_writes;
    dev.corrupt_cqe_sq_id = true;
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kCommandFailed);
    EXPECT_EQ(dev.io_cq_head_doorbell_writes, cqh_before);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.cc, 0u);
  }
  // CQE error status codes and Start() guard on already-running controller.
  {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);

    // Calling Start() when already kRunning returns kInvalidArgument without
    // poisoning state_ into kError.
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kInvalidArgument);
    EXPECT_EQ(ctrl.state(), NvmeState::kRunning);

    uint8_t sec[512] = {};
    dev.inject_next_io_status = static_cast<uint16_t>(1u << 9);  // SCT=1,SC=0
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kCommandFailed);
    EXPECT_EQ(ctrl.state(), NvmeState::kRunning);

    dev.inject_next_io_status = static_cast<uint16_t>(5u << 1);  // SCT=0,SC=5
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kCommandFailed);

    dev.inject_next_io_status =
        static_cast<uint16_t>((2u << 9) | (0x80u << 1));  // Media 0x80
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kCommandFailed);

    dev.inject_next_io_status =
        static_cast<uint16_t>(kStatusCodeInvalidField << 1);
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kInvalidArgument);

    dev.inject_next_io_status =
        static_cast<uint16_t>(kStatusCodeLbaOutOfRange << 1);
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kInvalidArgument);
    EXPECT_EQ(ctrl.state(), NvmeState::kRunning);

    // H07, H11a (0x02-only), H11b: cross-SCT confusion must stay
    // kCommandFailed.
    dev.inject_next_io_status = static_cast<uint16_t>(
        (kStatusTypeGeneric << 9) | (kStatusCodeWriteToReadOnlyRange << 1));
    EXPECT_EQ(ctrl.WriteBlocks(0, 0, 1, sec), NvmeStatus::kCommandFailed);
    EXPECT_EQ(ctrl.state(), NvmeState::kRunning);

    dev.inject_next_io_status = static_cast<uint16_t>(
        (kStatusTypeCommandSpecific << 9) | (kStatusCodeInvalidField << 1));
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kCommandFailed);
    EXPECT_EQ(ctrl.state(), NvmeState::kRunning);

    dev.inject_next_io_status = static_cast<uint16_t>(
        (kStatusTypeCommandSpecific << 9) | (kStatusCodeLbaOutOfRange << 1));
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kCommandFailed);
    EXPECT_EQ(ctrl.state(), NvmeState::kRunning);

    // N53: SCT=0, SC=0 with DNR (bit 15) and reserved bit 14 still succeeds.
    dev.inject_next_io_status = static_cast<uint16_t>(kCqeDnrMask | (1u << 14));
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.state(), NvmeState::kRunning);
  }
}

TEST(NvmeControllerTest, StartFailuresAndRuntimeFaultsCleanUpHardware) {
  {
    sim::SimNvmeDevice dev;
    dev.not_decoding = true;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kNotDecoding);
  }
  {
    sim::SimNvmeDevice dev;
    dev.SetSizeAndHole(0x20, 0, 0);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kNotDecoding);
  }
  {
    sim::SimNvmeDevice dev;
    dev.SetSizeAndHole(0x37, 0, 0);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kNotDecoding);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  {
    sim::SimNvmeDevice dev;
    dev.SetSizeAndHole(0x2000, 0x30, 0x37);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kNotDecoding);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  // M2R8-06 (HUNT_04 / HUNT_04b / CHAL_COVERS_SKIP_0X18): sweep all 14
  // 4-byte aligned offsets in [0x00, 0x38) for Covers(kRegCap, 0x38).
  for (int64_t hole_off = 0x00; hole_off < 0x38; hole_off += 4) {
    sim::SimNvmeDevice dev;
    dev.SetSizeAndHole(0x4000, hole_off, hole_off + 4);
    if (hole_off >= kRegVs) {
      dev.cap = 0;
    }
    int mmio_reads = 0;
    dev.on_read32 = [&](const int64_t /*offset*/, const uint32_t /*val*/) {
      ++mmio_reads;
    };
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kNotDecoding)
        << hole_off;
    EXPECT_EQ(ctrl.state(), NvmeState::kError) << hole_off;
    EXPECT_EQ(mmio_reads, 0) << hole_off;
  }
  {
    sim::SimNvmeDevice dev;
    dev.SetSizeAndHole(0x38, 0, 0);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kBadCapabilities);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  {
    sim::SimNvmeDevice dev;
    dev.SetSizeAndHole(0x1000, 0, 0);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kBadCapabilities);
  }
  {
    sim::SimNvmeDevice dev;
    dev.cap &= ~kCapCssNvm;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kBadCapabilities);
  }
  {
    sim::SimNvmeDevice dev;
    dev.cap = (dev.cap & ~kCapCssNvm) | (0xFEULL << 37);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kBadCapabilities);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.cc, 0u);
  }
  {
    sim::SimNvmeDevice dev;
    dev.fail_dma_alloc = true;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kDmaAllocFailed);
  }
  {
    sim::SimNvmeDevice dev;
    dev.ResetRegisters(/*initially_enabled=*/true);
    dev.hang_disable = true;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kTimeout);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  {
    sim::SimNvmeDevice dev;
    dev.fatal_on_enable = true;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kControllerFatal);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.cc, 0u);
  }
  {
    sim::SimNvmeDevice dev;
    dev.hang_admin_cmd = true;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kTimeout);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.cc, 0u);
  }
  {
    sim::SimNvmeDevice dev;
    dev.fatal_on_io = true;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kControllerFatal);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.cc, 0u);
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 0, nullptr), NvmeStatus::kNotRunning);
    EXPECT_EQ(ctrl.WriteBlocks(0, 0, 0, nullptr), NvmeStatus::kNotRunning);
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kNotRunning);
    EXPECT_EQ(ctrl.WriteBlocks(0, 0, 1, sec), NvmeStatus::kNotRunning);
    EXPECT_EQ(ctrl.Flush(0), NvmeStatus::kNotRunning);
  }
  {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    dev.not_decoding = true;
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kNotDecoding);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  // H04: Init()-ed controller in kError rejects Start(1) with
  // kInvalidArgument and stays in kError (pre-DMA and post-DMA faults).
  {
    sim::SimNvmeDevice dev;
    dev.ResetRegisters(/*initially_enabled=*/true);
    dev.hang_disable = true;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kTimeout);
    ASSERT_EQ(ctrl.state(), NvmeState::kError);
    dev.hang_disable = false;
    dev.ResetRegisters(/*initially_enabled=*/false);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kInvalidArgument);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    dev.hang_admin_cmd = true;
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kTimeout);
    ASSERT_EQ(ctrl.state(), NvmeState::kError);
    dev.hang_admin_cmd = false;
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kInvalidArgument);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  // M2R7-15 / SHUNT_48 (sticky): CSTS.CFS | CSTS.RDY during WaitReadyBit()
  // fails Start() with kControllerFatal before unmasking INTMC or Admin I/O.
  {
    sim::SimNvmeDevice dev;
    dev.enable_delay_ticks = 5;
    dev.ResetRegisters(/*initially_enabled=*/false);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    bool injected = false;
    dev.on_read32 = [&](const int64_t offset, const uint32_t /*val*/) {
      if (!injected && offset == kRegCsts && (dev.cc & kCcEn) != 0) {
        injected = true;
        dev.csts |= kCstsCfs | kCstsRdy;
      }
    };
    EXPECT_EQ(ctrl.Start(1, IrqMode::kMsi), NvmeStatus::kControllerFatal);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.admin_commands_seen, 0);
    EXPECT_EQ(dev.intmc_writes_seen, 0);
    EXPECT_EQ(dev.cc, 0u);
  }
  // M2R7-15 / SHUNT_48 (transient 1-poll): single CSTS.CFS | CSTS.RDY read
  // fails Start() directly instead of returning kOk and issuing AdminIdentify.
  {
    class OneShotCfsRdyMmio final : public MmioWindow {
     public:
      explicit OneShotCfsRdyMmio(MmioWindow& inner) : inner_(inner) {}

      bool armed = false;

      uint32_t Read32(const int64_t offset) override {
        const uint32_t v = inner_.Read32(offset);
        if (armed && offset == kRegCsts) {
          armed = false;
          return v | kCstsCfs | kCstsRdy;
        }
        return v;
      }

      void Write32(const int64_t offset, const uint32_t value) override {
        inner_.Write32(offset, value);
      }

      int64_t Size() const override { return inner_.Size(); }

      bool Covers(const int64_t offset, const int64_t size) const override {
        return inner_.Covers(offset, size);
      }

     private:
      MmioWindow& inner_;
    };

    sim::SimNvmeDevice dev;
    OneShotCfsRdyMmio mmio(dev);
    NvmeController ctrl;
    ctrl.Init(dev, mmio);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    mmio.armed = true;
    EXPECT_EQ(ctrl.Start(1, IrqMode::kMsi), NvmeStatus::kControllerFatal);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.admin_commands_seen, 0);
    EXPECT_EQ(dev.intmc_writes_seen, 0);
    EXPECT_EQ(dev.cc, 0u);
  }
}

TEST(NvmeControllerTest,
     Truncates40CharModelAndHandlesUninitializedAndBadIrqMode) {
  {
    sim::SimNvmeDevice dev;
    const std::string full_model = "0123456789ABCDEFGHIJ0123456789abcdefghij";
    ASSERT_EQ(static_cast<int>(full_model.size()), kIdCtrlModelLength);
    dev.model = full_model;
    dev.serial = "SERIAL12345678901234";
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_EQ(static_cast<int>(std::strlen(ctrl.model())), 39);
    EXPECT_EQ(ctrl.model()[39], '\0');
    EXPECT_STREQ(ctrl.model(), full_model.substr(0, 39).c_str());
    EXPECT_STREQ(ctrl.serial(), "SERIAL12345678901234");
  }
  // WILD_R6_COPY_TRIM_NO_NUL_PAD: re-Start() with shorter model/serial
  // NUL-pads every byte past the trimmed string length.
  {
    sim::SimNvmeDevice dev;
    dev.model = "0123456789ABCDEFGHIJ0123456789abcdefghij";
    dev.serial = "SERIAL12345678901234";
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
    dev.model = "Short";
    dev.serial = "SN123";
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_STREQ(ctrl.model(), "Short");
    EXPECT_STREQ(ctrl.serial(), "SN123");
    for (int i = 5; i < kBlockModelCapacity; ++i) {
      EXPECT_EQ(ctrl.model()[i], '\0') << i;
    }
    for (int i = 5; i < kBlockSerialCapacity; ++i) {
      EXPECT_EQ(ctrl.serial()[i], '\0') << i;
    }
  }
  // ADV_M2_R3_11 / HUNT_11 & HUNT_12: all-space/all-NUL trim and mdts = 55.
  {
    sim::SimNvmeDevice dev;
    dev.model = "   ";
    dev.serial = "";
    dev.mdts = 55;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_STREQ(ctrl.model(), "");
    EXPECT_STREQ(ctrl.serial(), "");
    ASSERT_EQ(ctrl.namespace_count(), 1);
    EXPECT_EQ(ctrl.ns(0).max_transfer_blocks, 8);
  }
  {
    sim::SimNvmeDevice dev;
    dev.model = std::string(kIdCtrlModelLength, '\0');
    dev.serial = std::string(kIdCtrlSerialLength, '\0');
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_STREQ(ctrl.model(), "");
    EXPECT_STREQ(ctrl.serial(), "");
  }
  {
    FakeNvmeHost fake;
    fake.nul_pad_strings = true;
    fake.vwc = 0xFE;
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_STREQ(ctrl.serial(), "NULSN");
    EXPECT_STREQ(ctrl.model(), "NulModel");
    EXPECT_FALSE(ctrl.has_volatile_write_cache());
  }
  // N54a: Interior NUL bytes in model/serial are sanitized to '?' rather than
  // truncating early.
  {
    sim::SimNvmeDevice dev;
    dev.model = std::string("AB\0CD", 5);
    dev.serial = std::string("XY\0ZW", 5);
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_STREQ(ctrl.model(), "AB?CD");
    EXPECT_STREQ(ctrl.serial(), "XY?ZW");
  }
  {
    NvmeController uninit_ctrl;
    EXPECT_EQ(uninit_ctrl.TakeOwnershipAndReset(),
              NvmeStatus::kInvalidArgument);
    EXPECT_EQ(uninit_ctrl.state(), NvmeState::kError);
    EXPECT_EQ(uninit_ctrl.Start(1), NvmeStatus::kInvalidArgument);
    uint8_t sec[512] = {};
    EXPECT_EQ(uninit_ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kNotRunning);
    EXPECT_EQ(uninit_ctrl.WriteBlocks(0, 0, 1, sec), NvmeStatus::kNotRunning);
    EXPECT_EQ(uninit_ctrl.Flush(0), NvmeStatus::kNotRunning);
  }
  {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1, static_cast<IrqMode>(2)),
              NvmeStatus::kInvalidArgument);
    EXPECT_EQ(ctrl.state(), NvmeState::kHalted);
    EXPECT_EQ(dev.cc, 0u);
  }
  {
    sim::SimNvmeDevice dev;
    dev.on_read32 = [&](const int64_t offset, const uint32_t /*val*/) {
      if (offset == kRegCap + 4) {
        dev.not_decoding = true;
      }
    };
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    EXPECT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kNotDecoding);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
}

TEST(NvmeControllerTest, CreateIoQueueAndResetFailuresAndMidPollDetach) {
  // WB01: IdentifyController() CQE failure disables controller (CC=0).
  {
    FakeNvmeHost fake;
    fake.fail_admin_opcode = kAdminIdentify;
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kCommandFailed);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(fake.cc_reg, 0u);
    EXPECT_EQ(fake.Read32(kRegCsts) & kCstsRdy, 0u);
  }
  {
    FakeNvmeHost fake;
    fake.fail_admin_opcode = kAdminSetFeatures;
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kCommandFailed);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(fake.cc_reg, 0u);
  }
  {
    FakeNvmeHost fake;
    fake.fail_admin_opcode = kAdminCreateIoCq;
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kCommandFailed);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(fake.cc_reg, 0u);
  }
  {
    FakeNvmeHost fake;
    fake.fail_admin_opcode = kAdminCreateIoSq;
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kCommandFailed);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(fake.cc_reg, 0u);
  }
  {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.state(), NvmeState::kHalted);
    dev.cc = kCcEn;
    dev.csts = kCstsRdy;
    dev.hang_disable = true;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kTimeout);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.state(), NvmeState::kHalted);
    dev.cc = 0;
    dev.csts = kCstsRdy;
    dev.hang_disable = true;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kTimeout);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  {
    sim::SimNvmeDevice dev;
    dev.enable_delay_ticks = 5;
    dev.ResetRegisters(/*initially_enabled=*/false);
    int csts_reads = 0;
    dev.on_read32 = [&](const int64_t offset, const uint32_t /*val*/) {
      if (offset == kRegCsts && (dev.cc & kCcEn) != 0) {
        ++csts_reads;
        if (csts_reads == 1) {
          dev.not_decoding = true;
        }
      }
    };
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kNotDecoding);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  // M2R7-11 / WILD_R6_FAIL_DISABLE_ON_NOT_DEC: transient CSTS == kMmioAllOnes
  // in Start() triggers FailAndDisable(kNotDecoding) without a redundant CC=0
  // write or kRegCc read.
  {
    sim::SimNvmeDevice dev;
    dev.enable_delay_ticks = 5;
    dev.ResetRegisters(/*initially_enabled=*/false);
    CcWriteSpyMmio spy(dev);
    HalfAllOnesMmio mmio(spy);
    NvmeController ctrl;
    ctrl.Init(dev, mmio);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    int start_cc_reads = 0;
    dev.on_read32 = [&](const int64_t offset, const uint32_t /*val*/) {
      if (offset == kRegCc) {
        ++start_cc_reads;
      }
    };
    spy.cc_writes.clear();
    mmio.spoof_offset = kRegCsts;
    mmio.spoof_reads_left = 1;
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kNotDecoding);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(static_cast<int>(spy.cc_writes.size()), 1);
    EXPECT_NE(dev.cc, 0u);
    EXPECT_EQ(start_cc_reads, 0);
  }
  {
    sim::SimNvmeDevice dev;
    dev.shutdown_delay_ticks = 5;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    int shn_csts_reads = 0;
    dev.on_read32 = [&](const int64_t offset, const uint32_t /*val*/) {
      if (offset == kRegCsts && (dev.cc & kCcShnMask) != 0) {
        ++shn_csts_reads;
        if (shn_csts_reads == 1) {
          dev.not_decoding = true;
        }
      }
    };
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kNotDecoding);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  // M2R7-12 / WILD_R6_SHN_DISABLE_ON_NOT_DEC: transient CSTS == kMmioAllOnes
  // in WaitShutdownComplete() skips DisableControllerBestEffort() so CC stays
  // non-zero with only the single CC.SHN write.
  {
    sim::SimNvmeDevice dev;
    dev.shutdown_delay_ticks = 5;
    CcWriteSpyMmio spy(dev);
    HalfAllOnesMmio mmio(spy);
    NvmeController ctrl;
    ctrl.Init(dev, mmio);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    spy.cc_writes.clear();
    bool armed = false;
    dev.on_read32 = [&](const int64_t offset, const uint32_t /*val*/) {
      if (!armed && offset == kRegCsts) {
        armed = true;
        mmio.spoof_offset = kRegCsts;
        mmio.spoof_reads_left = 1;
      }
    };
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kNotDecoding);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(static_cast<int>(spy.cc_writes.size()), 1);
    EXPECT_NE(dev.cc, 0u);
  }
}

TEST(NvmeControllerTest, ShutdownHandlesInitialAndRuntimeCfsAndPreemption) {
  // Initial CSTS.CFS skips CC.SHN and returns kControllerFatal.
  {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    dev.csts |= kCstsCfs;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kControllerFatal);
    EXPECT_FALSE(dev.saw_shn_normal);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.cc, 0u);
  }
  // CSTS.CFS raised mid-WaitShutdownComplete returns kControllerFatal.
  {
    sim::SimNvmeDevice dev;
    dev.shutdown_delay_ticks = 10;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    int shn_reads = 0;
    dev.on_read32 = [&](const int64_t offset, const uint32_t /*val*/) {
      if (offset == kRegCsts && (dev.cc & kCcShnMask) != 0) {
        ++shn_reads;
        if (shn_reads == 1) {
          dev.csts |= kCstsCfs;
        }
      }
    };
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kControllerFatal);
    EXPECT_TRUE(dev.saw_shn_normal);
    EXPECT_EQ(dev.shutdown_count, 0);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.cc, 0u);
  }
  // M2R7-10 / SHUNT_49: CSTS.CFS takes precedence over CSTS.SHST == Complete
  // when both are asserted simultaneously in WaitShutdownComplete().
  {
    sim::SimNvmeDevice dev;
    dev.shutdown_delay_ticks = 10;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    int shn_reads = 0;
    dev.on_read32 = [&](const int64_t offset, const uint32_t /*val*/) {
      if (offset == kRegCsts && (dev.cc & kCcShnMask) != 0) {
        ++shn_reads;
        if (shn_reads == 1) {
          dev.csts = (dev.csts & ~kCstsShstMask) | kCstsCfs | kCstsShstComplete;
        }
      }
    };
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kControllerFatal);
    EXPECT_TRUE(dev.saw_shn_normal);
    EXPECT_EQ(dev.shutdown_count, 0);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.cc, 0u);
  }
  // WaitShutdownComplete preempted past deadline still sees SHST == Complete.
  {
    sim::SimNvmeDevice dev;
    dev.shutdown_delay_ticks = 100;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    int shn_now_calls = 0;
    dev.on_now_ticks = [&]() {
      if ((dev.cc & kCcShnMask) != 0) {
        ++shn_now_calls;
        if (shn_now_calls == 2) {
          dev.now_ticks += 5000;
          dev.csts = (dev.csts & ~kCstsShstMask) | kCstsShstComplete;
        }
      }
    };
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.state(), NvmeState::kHalted);
  }

  // ADV_M2_R3_09: Non-kOk first_err (CSTS.CFS or WaitShutdownComplete timeout)
  // is preserved both when DisableControllerBestEffort succeeds (reset_rc ==
  // kOk) and when it fails (reset_rc == kTimeout / kNotDecoding).
  {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    dev.csts |= kCstsCfs;
    dev.hang_disable = false;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kControllerFatal);
    EXPECT_FALSE(dev.saw_shn_normal);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.cc, 0u);
    EXPECT_EQ(dev.csts & kCstsRdy, 0u);
  }
  {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    dev.csts |= kCstsCfs;
    dev.hang_disable = true;
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kControllerFatal);
    EXPECT_FALSE(dev.saw_shn_normal);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.cc, 0u);
  }
  {
    sim::SimNvmeDevice dev;
    CcWriteSpyMmio spy(dev);
    NvmeController ctrl;
    ctrl.Init(dev, spy);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    spy.cc_writes.clear();
    dev.hang_shutdown = true;
    const int64_t shn_start = dev.now_ticks;
    const int64_t shn_wait = TicksFromMs(2000, dev.ticks_per_second);
    dev.on_read32 = [&](const int64_t offset, const uint32_t /*val*/) {
      if (offset == kRegCsts && (dev.cc & kCcShnMask) != 0 &&
          (dev.now_ticks - shn_start) >= shn_wait) {
        dev.not_decoding = true;
      }
    };
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kTimeout);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(static_cast<int>(spy.cc_writes.size()), 1);
  }
  // M2R7-13 / SHUNT_22: asymmetric kRegCc == kMmioAllOnes inside
  // DisableControllerBestEffort() from kError Shutdown() returns
  // kNotDecoding and issues zero CC=0 writes.
  {
    sim::SimNvmeDevice dev;
    CcWriteSpyMmio spy(dev);
    HalfAllOnesMmio mmio(spy);
    NvmeController ctrl;
    ctrl.Init(dev, mmio);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    dev.hang_io_cmd = true;
    uint8_t sec[512] = {};
    ASSERT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kTimeout);
    ASSERT_EQ(ctrl.state(), NvmeState::kError);
    spy.cc_writes.clear();
    bool armed = false;
    dev.on_read32 = [&](const int64_t offset, const uint32_t /*val*/) {
      if (!armed && offset == kRegCsts) {
        armed = true;
        mmio.spoof_offset = kRegCc;
        mmio.spoof_reads_left = 1;
      }
    };
    EXPECT_EQ(ctrl.Shutdown(), NvmeStatus::kNotDecoding);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(static_cast<int>(spy.cc_writes.size()), 0);
  }
  // M2R8-04 (M02 / HUNT_05): transient 1-poll and persistent CSTS-only
  // kMmioAllOnes during SubmitAndWait() return kNotDecoding without issuing
  // DisableControllerBestEffort() CC=0 writes.
  for (const int spoof_reads : {1, -1}) {
    sim::SimNvmeDevice dev;
    CcWriteSpyMmio spy(dev);
    HalfAllOnesMmio mmio(spy);
    NvmeController ctrl;
    ctrl.Init(dev, mmio);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk) << spoof_reads;
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk) << spoof_reads;
    spy.cc_writes.clear();
    mmio.spoof_offset = kRegCsts;
    mmio.spoof_reads_left = spoof_reads;
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kNotDecoding)
        << spoof_reads;
    EXPECT_EQ(ctrl.state(), NvmeState::kError) << spoof_reads;
    EXPECT_EQ(static_cast<int>(spy.cc_writes.size()), 0) << spoof_reads;
    EXPECT_NE(dev.cc, 0u) << spoof_reads;
  }
  // M2R8-05 (M03): simultaneous CSTS.CFS == 1 and ready synchronous CQE in
  // SubmitAndWait() fails kControllerFatal without advancing CQHDBL.
  {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    const int cqh_before = dev.io_cq_head_doorbell_writes;
    const int io_before = dev.io_commands_seen;
    bool armed = true;
    dev.on_now_ticks = [&]() {
      if (armed && dev.io_commands_seen > io_before) {
        armed = false;
        dev.csts |= kCstsCfs;
      }
    };
    uint8_t sec[512] = {};
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, 1, sec), NvmeStatus::kControllerFatal);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(dev.io_commands_seen, io_before + 1);
    EXPECT_EQ(dev.io_cq_head_doorbell_writes, cqh_before);
    EXPECT_EQ(dev.cc, 0u);
  }
}

TEST(NvmeControllerTest, Cns2EdgeCasesFullBatchStopAndProbeErrorPropagation) {
  // 33 active namespaces fill kMaxNamespaces=8 on a full batch without
  // reaching zero sentinel (exits while-loop to `!first_page`).
  {
    sim::SimNvmeDevice dev;
    dev.namespaces.clear();
    for (uint32_t id = 1; id <= 33; ++id) {
      sim::SimNvmeDevice::NamespaceConfig ns;
      ns.nsid = id;
      ns.block_count = 16;
      ns.block_size = 512;
      dev.namespaces.push_back(std::move(ns));
    }
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.namespace_count(), 8);
    EXPECT_EQ(ctrl.ns(0).nsid, 1u);
    EXPECT_EQ(ctrl.ns(7).nsid, 8u);
    EXPECT_EQ(dev.admin_commands_seen, 13);
  }
  // W31: 1024th slot (i == 1023) in CNS=2 response must be processed.
  {
    FakeNvmeHost fake;
    fake.nsze = 64;
    fake.ncap = 64;
    fake.custom_cns2 = [](const uint32_t cursor, uint8_t* const out) {
      if (cursor == 0) {
        for (int i = 0; i < 1023; ++i) {
          FakeNvmeHost::WriteLe32(out + 4 * i, 0xFFFFFFFFu);
        }
        FakeNvmeHost::WriteLe32(out + 4 * 1023, 1u);
      }
    };
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.namespace_count(), 1);
    EXPECT_EQ(ctrl.ns(0).nsid, 1u);
  }
  // W32: CNS=2 page 2 error must fail Start rather than falling back to 1..NN.
  {
    FakeNvmeHost fake;
    fake.nsze = 64;
    fake.ncap = 64;
    fake.nn_override = 1;
    fake.fail_cns2_after_page1 = true;
    fake.custom_cns2 = [](const uint32_t cursor, uint8_t* const out) {
      if (cursor == 0) {
        for (int i = 0; i < 32; ++i) {
          FakeNvmeHost::WriteLe32(out + 4 * i, static_cast<uint32_t>(2 + i));
        }
      }
    };
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kInvalidArgument);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(fake.cc_reg, 0u);
    EXPECT_EQ(fake.Read32(kRegCsts) & kCstsRdy, 0u);
  }
  // W33: CNS=2 stops immediately on zero sentinel (ignores trailing entries).
  {
    FakeNvmeHost fake;
    fake.nsze = 64;
    fake.ncap = 64;
    fake.custom_cns2 = [](const uint32_t cursor, uint8_t* const out) {
      if (cursor == 0) {
        FakeNvmeHost::WriteLe32(out, 0u);
        FakeNvmeHost::WriteLe32(out + 4, 1u);
      }
    };
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.namespace_count(), 0);
  }
  // ProbeNamespace fatal error propagates through both CNS=2 and 1..NN.
  {
    FakeNvmeHost fake;
    fake.nsze = 64;
    fake.ncap = 64;
    fake.fail_cns0_with_cfs = true;
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kControllerFatal);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  {
    FakeNvmeHost fake;
    fake.nsze = 64;
    fake.ncap = 64;
    fake.support_cns2 = false;
    fake.nn_override = 1;
    fake.fail_cns0_with_cfs = true;
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kControllerFatal);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
  }
  // NW17: Full 32-entry CNS=2 page ending at 0xFFFFFFFEu terminates in 1 query.
  {
    FakeNvmeHost fake;
    fake.nsze = 64;
    fake.ncap = 64;
    fake.custom_cns2 = [](const uint32_t cursor, uint8_t* const out) {
      if (cursor == 0) {
        for (int i = 0; i < 31; ++i) {
          FakeNvmeHost::WriteLe32(out + 4 * i, static_cast<uint32_t>(1 + i));
        }
        FakeNvmeHost::WriteLe32(out + 4 * 31, 0xFFFFFFFEu);
      }
    };
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.namespace_count(), 1);
    EXPECT_EQ(ctrl.ns(0).nsid, 1u);
    EXPECT_THAT(fake.cns2_cursors, t::ElementsAre(0u));
  }
  // N59c: Full CNS=2 Page 1 ending at 0xFFFFFFFDu paginates to Page 2 and
  // discovers 0xFFFFFFFEu (Page 1 entries are inactive so count stays < 8).
  {
    FakeNvmeHost fake;
    fake.nsze = 64;
    fake.ncap = 64;
    fake.present_nsids = {0xFFFFFFFEu};
    fake.custom_cns2 = [](const uint32_t cursor, uint8_t* const out) {
      if (cursor == 0) {
        for (int i = 0; i < 31; ++i) {
          FakeNvmeHost::WriteLe32(out + 4 * i, static_cast<uint32_t>(1 + i));
        }
        FakeNvmeHost::WriteLe32(out + 4 * 31, 0xFFFFFFFDu);
      } else if (cursor == 0xFFFFFFFDu) {
        FakeNvmeHost::WriteLe32(out, 0xFFFFFFFEu);
      }
    };
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.namespace_count(), 1);
    EXPECT_EQ(ctrl.ns(0).nsid, 0xFFFFFFFEu);
    EXPECT_THAT(fake.cns2_cursors, t::ElementsAre(0u, 0xFFFFFFFDu));
  }

  // ADV_M2_R3_12a / HUNT_13: ProbeNamespace treats kCommandFailed as inactive.
  {
    FakeNvmeHost fake;
    fake.nsze = 64;
    fake.ncap = 64;
    fake.present_nsids = {2u};
    fake.inactive_ns_status = static_cast<uint16_t>(0x0Bu << 1);
    fake.custom_cns2 = [](const uint32_t cursor, uint8_t* const out) {
      if (cursor == 0) {
        FakeNvmeHost::WriteLe32(out, 1u);
        FakeNvmeHost::WriteLe32(out + 4, 2u);
      }
    };
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.namespace_count(), 1);
    EXPECT_EQ(ctrl.ns(0).nsid, 2u);
  }
  // ADV_M2_R3_12b / HUNT_14: CNS=2 page-1 kCommandFailed falls back to 1..NN.
  {
    FakeNvmeHost fake;
    fake.nsze = 64;
    fake.ncap = 64;
    fake.nn_override = 1;
    fake.cns2_page1_status = static_cast<uint16_t>(0x01u << 1);
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.namespace_count(), 1);
    EXPECT_EQ(ctrl.ns(0).nsid, 1u);
  }
  // HUNT_15: transient page-1 CNS=2 kTimeout with NN=0 must fail Start(),
  // without latched CSTS.CFS masking the check in SetFeatures.
  {
    FakeNvmeHost fake;
    fake.force_nn_zero = true;
    fake.timeout_cns2_once = true;
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.Start(1), NvmeStatus::kTimeout);
    EXPECT_EQ(ctrl.state(), NvmeState::kError);
    EXPECT_EQ(fake.set_features_count, 0);
    EXPECT_THAT(fake.cns2_cursors, t::ElementsAre(0u));
  }
  // ADV_M2_R3_17 (ADV_17_SPLIT): non-monotonic nsid <= max_seen continues.
  {
    FakeNvmeHost fake;
    fake.nsze = 64;
    fake.ncap = 64;
    fake.present_nsids = {2u, 3u};
    fake.fail_cns2_after_page1 = true;
    fake.custom_cns2 = [](const uint32_t cursor, uint8_t* const out) {
      if (cursor == 0) {
        FakeNvmeHost::WriteLe32(out, 2u);
        FakeNvmeHost::WriteLe32(out + 4, 1u);
        FakeNvmeHost::WriteLe32(out + 8, 3u);
      }
    };
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.namespace_count(), 2);
    EXPECT_EQ(ctrl.ns(0).nsid, 2u);
    EXPECT_EQ(ctrl.ns(1).nsid, 3u);
    EXPECT_THAT(fake.cns2_cursors, t::ElementsAre(0u));
  }
}

TEST(NvmeControllerTest, RejectsHugeNamespaceLbaWrapAndZeroNcapAndWrapsCid) {
  // WILD_07: UINT64_MAX namespace catches LBA addition wrap before I/O SQE.
  {
    FakeNvmeHost fake;
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_EQ(fake.last_create_iocq_cdw11,
              kCqInterruptEnabled | kQueuePhysicallyContiguous);
    EXPECT_EQ(fake.last_create_iosq_cdw11,
              (1u << 16) | kQueuePhysicallyContiguous);
    ASSERT_EQ(ctrl.namespace_count(), 1);
    EXPECT_EQ(ctrl.ns(0).block_count, UINT64_MAX);

    uint8_t buf[1024] = {};
    const int db_before = fake.io_db_writes;
    EXPECT_EQ(ctrl.ReadBlocks(0, UINT64_MAX, 1, buf),
              NvmeStatus::kInvalidArgument);
    EXPECT_EQ(fake.io_db_writes, db_before);
    EXPECT_EQ(ctrl.WriteBlocks(0, UINT64_MAX - 1, 2, buf),
              NvmeStatus::kInvalidArgument);
    EXPECT_EQ(fake.io_db_writes, db_before);
    EXPECT_EQ(ctrl.ReadBlocks(0, UINT64_MAX - 1, 2, buf),
              NvmeStatus::kInvalidArgument);
    EXPECT_EQ(fake.io_db_writes, db_before);
    EXPECT_EQ(ctrl.WriteBlocks(0, UINT64_MAX, 1, buf),
              NvmeStatus::kInvalidArgument);
    EXPECT_EQ(fake.io_db_writes, db_before);
    EXPECT_EQ(ctrl.ReadBlocks(0, 0, -1, buf), NvmeStatus::kInvalidArgument);
    EXPECT_EQ(fake.io_db_writes, db_before);
    EXPECT_EQ(ctrl.WriteBlocks(0, 0, -1, buf), NvmeStatus::kInvalidArgument);
    EXPECT_EQ(fake.io_db_writes, db_before);
    EXPECT_EQ(ctrl.ReadBlocks(0, UINT64_MAX, 0, nullptr), NvmeStatus::kOk);
    EXPECT_EQ(fake.io_db_writes, db_before);

    // NOVEL_01 / NW12+13: Live 64-bit LBA WriteBlocks/ReadBlocks checks.
    const uint64_t lba_w = 0xFEDCBA9876543210ULL;
    const uint64_t lba_r = 0x123456789ABCDEF0ULL;
    EXPECT_EQ(ctrl.WriteBlocks(0, lba_w, 1, buf), NvmeStatus::kOk);
    EXPECT_EQ(fake.last_io_opc, kNvmWrite);
    EXPECT_EQ(fake.last_io_lba, lba_w);
    EXPECT_EQ(ctrl.ReadBlocks(0, lba_r, 1, buf), NvmeStatus::kOk);
    EXPECT_EQ(fake.last_io_opc, kNvmRead);
    EXPECT_EQ(fake.last_io_lba, lba_r);
  }
  // W35: ncap == 0 (even with nsze > 0) and nsze == 0 (with ncap > 0) skipped.
  {
    FakeNvmeHost fake;
    fake.nsze = 64;
    fake.ncap = 0;
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.namespace_count(), 0);
  }
  {
    FakeNvmeHost fake;
    fake.nsze = 0;
    fake.ncap = 64;
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.namespace_count(), 0);
  }
  // N55b: Full 64-bit NCAP (low 32 bits == 0) is not truncated by ReadLe32.
  {
    FakeNvmeHost fake;
    fake.nsze = 0x100000000ULL;
    fake.ncap = 0x100000000ULL;
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.namespace_count(), 1);
    EXPECT_EQ(ctrl.ns(0).block_count, 0x100000000ULL);
  }
  // U10 (raw): LBAF0.LBADS = 0 (1 B < 512 B) via FakeNvmeHost is skipped.
  {
    FakeNvmeHost fake;
    fake.nsze = 64;
    fake.ncap = 64;
    fake.lbads = 0;
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.namespace_count(), 0);
  }
  // W30: NextCid wraps 65535 -> 1 and never issues CID 0.
  {
    FakeNvmeHost fake;
    fake.nsze = 64;
    fake.ncap = 64;
    NvmeController ctrl;
    ctrl.Init(fake, fake);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
    for (int i = 0; i < 65536; ++i) {
      ASSERT_EQ(ctrl.Flush(0), NvmeStatus::kOk) << i;
    }
    EXPECT_TRUE(fake.saw_cid_65535);
    EXPECT_TRUE(fake.saw_wrap_to_1);
    EXPECT_FALSE(fake.saw_zero_cid);
  }
}

TEST(NvmeControllerTest, MultiNamespaceReadWriteFlushIsolateNamespaces) {
  sim::SimNvmeDevice dev;
  dev.namespaces.clear();

  sim::SimNvmeDevice::NamespaceConfig ns0;
  ns0.nsid = 10u;
  ns0.block_count = 8;
  ns0.block_size = 512;
  ns0.read_only = true;
  ns0.InitStorage();
  std::memset(ns0.storage.data(), 0x11, ns0.storage.size());
  dev.namespaces.push_back(std::move(ns0));

  sim::SimNvmeDevice::NamespaceConfig ns1;
  ns1.nsid = 20u;
  ns1.block_count = 16;
  ns1.block_size = 4096;
  ns1.read_only = false;
  ns1.InitStorage();
  std::memset(ns1.storage.data(), 0x22, ns1.storage.size());
  dev.namespaces.push_back(std::move(ns1));

  NvmeController ctrl;
  ctrl.Init(dev, dev);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.namespace_count(), 2);
  EXPECT_EQ(ctrl.ns(0).nsid, 10u);
  EXPECT_EQ(ctrl.ns(1).nsid, 20u);

  const auto read_last_io_sqe = [&]() -> SubmissionEntry {
    if (dev.io_sq_size <= 0) {
      return SubmissionEntry{};
    }
    const int slot = (dev.io_sq_head + dev.io_sq_size - 1) % dev.io_sq_size;
    const int64_t sq_bytes = static_cast<int64_t>(dev.io_sq_size) *
                             static_cast<int64_t>(sizeof(SubmissionEntry));
    const uint8_t* const sq_ptr = dev.PhysToPtr(dev.io_sq_phys, sq_bytes);
    SubmissionEntry sqe{};
    if (sq_ptr != nullptr) {
      const int64_t off = static_cast<int64_t>(slot) *
                          static_cast<int64_t>(sizeof(SubmissionEntry));
      std::memcpy(&sqe, sq_ptr + off, sizeof(sqe));
    }
    return sqe;
  };

  uint8_t tx[4096];
  for (int i = 0; i < 4096; ++i) {
    tx[i] = static_cast<uint8_t>(0x30 + (i & 0x3F));
  }
  ASSERT_EQ(ctrl.WriteBlocks(1, 10, 1, tx), NvmeStatus::kOk);
  const SubmissionEntry write_sqe = read_last_io_sqe();
  EXPECT_EQ(static_cast<uint8_t>(write_sqe.cdw0 & 0xFFu), kNvmWrite);
  EXPECT_EQ(write_sqe.nsid, 20u);
  EXPECT_THAT(dev.namespaces[0].storage, t::Each(t::Eq(0x11)));

  uint8_t rx[4096] = {};
  ASSERT_EQ(ctrl.ReadBlocks(1, 10, 1, rx), NvmeStatus::kOk);
  const SubmissionEntry read_sqe = read_last_io_sqe();
  EXPECT_EQ(static_cast<uint8_t>(read_sqe.cdw0 & 0xFFu), kNvmRead);
  EXPECT_EQ(read_sqe.nsid, 20u);
  EXPECT_EQ(std::memcmp(tx, rx, sizeof(tx)), 0);

  ASSERT_EQ(ctrl.Flush(0), NvmeStatus::kOk);
  const SubmissionEntry flush0_sqe = read_last_io_sqe();
  EXPECT_EQ(static_cast<uint8_t>(flush0_sqe.cdw0 & 0xFFu), kNvmFlush);
  EXPECT_EQ(flush0_sqe.nsid, 10u);

  ASSERT_EQ(ctrl.Flush(1), NvmeStatus::kOk);
  const SubmissionEntry flush1_sqe = read_last_io_sqe();
  EXPECT_EQ(static_cast<uint8_t>(flush1_sqe.cdw0 & 0xFFu), kNvmFlush);
  EXPECT_EQ(flush1_sqe.nsid, 20u);

  FakeNvmeHost fake;
  fake.nsze = 64;
  fake.ncap = 64;
  fake.present_nsids = {10u, 20u};
  NvmeController fake_ctrl;
  fake_ctrl.Init(fake, fake);
  ASSERT_EQ(fake_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(fake_ctrl.Start(1), NvmeStatus::kOk);
  ASSERT_EQ(fake_ctrl.namespace_count(), 2);
  uint8_t sec[512] = {};
  EXPECT_EQ(fake_ctrl.WriteBlocks(1, 2, 1, sec), NvmeStatus::kOk);
  EXPECT_EQ(fake.last_io_opc, kNvmWrite);
  EXPECT_EQ(fake.last_io_nsid, 20u);
  EXPECT_EQ(fake_ctrl.ReadBlocks(1, 2, 1, sec), NvmeStatus::kOk);
  EXPECT_EQ(fake.last_io_opc, kNvmRead);
  EXPECT_EQ(fake.last_io_nsid, 20u);
  EXPECT_EQ(fake_ctrl.Flush(1), NvmeStatus::kOk);
  EXPECT_EQ(fake.last_io_opc, kNvmFlush);
  EXPECT_EQ(fake.last_io_nsid, 20u);
  // Full-capacity (8 NS) OOB guard with 0x01-tainted tail (zero UB on REF).
  {
    sim::SimNvmeDevice dev8;
    dev8.namespaces.clear();
    for (int i = 1; i <= NvmeController::kMaxNamespaces; ++i) {
      sim::SimNvmeDevice::NamespaceConfig ns;
      ns.nsid = static_cast<uint32_t>(i);
      ns.block_count = 16;
      ns.block_size = 512;
      dev8.namespaces.push_back(std::move(ns));
    }
    struct ControllerWithTail {
      NvmeController ctrl;
      uint8_t tail[64];
    } tainted;
    tainted.ctrl.Init(dev8, dev8);
    ASSERT_EQ(tainted.ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(tainted.ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(tainted.ctrl.namespace_count(), 8);
    std::memset(tainted.tail, 0x01, sizeof(tainted.tail));
    asm volatile("" : : "m"(tainted.tail) : "memory");
    const volatile int full_oob_ns = NvmeController::kMaxNamespaces;
    const int oob_ns = full_oob_ns;
    const int tainted_io_before = dev8.io_commands_seen;
    uint8_t buf8[512] = {};
    EXPECT_EQ(tainted.ctrl.ReadBlocks(oob_ns, 0, 1, buf8),
              NvmeStatus::kInvalidArgument);
    EXPECT_EQ(tainted.ctrl.WriteBlocks(oob_ns, 0, 1, buf8),
              NvmeStatus::kInvalidArgument);
    EXPECT_EQ(tainted.ctrl.Flush(oob_ns), NvmeStatus::kInvalidArgument);
    EXPECT_EQ(dev8.io_commands_seen, tainted_io_before);
  }
}

// --- BlockServerTest ---------------------------------------------------------

TEST(BlockServerTest, DispatchesRpcRequestsAndValidatesInputs) {
  sim::SimNvmeDevice dev;
  NvmeController ctrl;
  ctrl.Init(dev, dev, /*bus=*/0, /*device=*/4, /*function=*/0);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);

  BlockServer server;
  ASSERT_TRUE(server.AddController(&ctrl));
  EXPECT_EQ(server.disk_count(), 1);
  EXPECT_EQ(server.RunningDiskCount(), 1);

  uint8_t reply[kIpcMaxMessageSize] = {};
  int64_t reply_size = 0;

  // kGetInfo
  {
    const BlockGetInfoRequest req{.op = BlockRpcOp::kGetInfo, .disk_index = 0};
    const int64_t rc =
        server.Dispatch(reinterpret_cast<const uint8_t*>(&req),  //
                        sizeof(req),                             //
                        sizeof(reply),                           //
                        reply,                                   //
                        &reply_size);
    EXPECT_EQ(rc, 1);
    EXPECT_EQ(reply_size, static_cast<int64_t>(sizeof(BlockDeviceInfo)));
    BlockDeviceInfo info{};
    std::memcpy(&info, reply, sizeof(info));
    EXPECT_EQ(info.block_count, uint64_t{128});
    EXPECT_EQ(info.block_size, 512);
    EXPECT_EQ(info.max_transfer_blocks, 8);
    EXPECT_EQ(info.bus, 0);
    EXPECT_EQ(info.device, 4);
    EXPECT_EQ(info.function, 0);
    EXPECT_EQ(info.read_only, 0);
    EXPECT_EQ(info.nsid, 1u);
    EXPECT_STREQ(info.model, "SimNVMe 1.4");
    EXPECT_STREQ(info.serial, "SIM0001");
  }

  // kWrite + kFlush + kRead + 0-block no-ops.
  {
    uint8_t req_buf[sizeof(BlockWriteRequest) + 1024] = {};
    const BlockWriteRequest wreq{
        .op = BlockRpcOp::kWrite,
        .disk_index = 0,
        .lba = 7,
        .block_count = 2,
        .reserved = 0,
    };
    std::memcpy(req_buf, &wreq, sizeof(wreq));
    for (int i = 0; i < 1024; ++i) {
      req_buf[sizeof(wreq) + i] = static_cast<uint8_t>(0xA0 ^ (i & 0x3F));
    }
    EXPECT_EQ(
        server.Dispatch(req_buf, sizeof(req_buf), 0, nullptr, &reply_size), 2);
    EXPECT_EQ(reply_size, 0);

    const BlockFlushRequest freq{.op = BlockRpcOp::kFlush, .disk_index = 0};
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&freq),  //
                              sizeof(freq),                             //
                              0,                                        //
                              nullptr,                                  //
                              &reply_size),
              0);

    const BlockReadRequest rreq{
        .op = BlockRpcOp::kRead,
        .disk_index = 0,
        .lba = 7,
        .block_count = 2,
        .reserved = 0,
    };
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&rreq),  //
                              sizeof(rreq),                             //
                              1024,                                     //
                              reply,                                    //
                              &reply_size),
              2);
    EXPECT_EQ(reply_size, 1024);
    EXPECT_EQ(std::memcmp(reply, req_buf + sizeof(wreq), 1024), 0);

    const BlockReadRequest zero_rreq{
        .op = BlockRpcOp::kRead,
        .disk_index = 0,
        .lba = 128,
        .block_count = 0,
        .reserved = 0,
    };
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&zero_rreq),  //
                              sizeof(zero_rreq),                             //
                              0,                                             //
                              nullptr,                                       //
                              &reply_size),
              0);
    EXPECT_EQ(reply_size, 0);
  }
  // Multi-block BlockServer::kRead anti-aliasing: taint ctrl.io_buf_ via
  // kWrite at LBA 0 (0xA5), then read pre-populated distinct LBA 20..27.
  {
    for (int i = 0; i < 4096; ++i) {
      dev.namespaces[0].storage[20 * 512 + i] =
          static_cast<uint8_t>(0x5B ^ ((i * 17 + (i >> 9)) & 0xFF));
    }
    uint8_t taint_wbuf[sizeof(BlockWriteRequest) + 4096] = {};
    const BlockWriteRequest taint_wreq{
        .op = BlockRpcOp::kWrite,
        .disk_index = 0,
        .lba = 0,
        .block_count = 8,
        .reserved = 0,
    };
    std::memcpy(taint_wbuf, &taint_wreq, sizeof(taint_wreq));
    std::memset(taint_wbuf + sizeof(taint_wreq), 0xA5, 4096);
    ASSERT_EQ(server.Dispatch(taint_wbuf,          //
                              sizeof(taint_wbuf),  //
                              0,                   //
                              nullptr,             //
                              &reply_size),
              8);
    const BlockReadRequest alias_rreq{
        .op = BlockRpcOp::kRead,
        .disk_index = 0,
        .lba = 20,
        .block_count = 8,
        .reserved = 0,
    };
    std::memset(reply, 0, 4096);
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&alias_rreq),  //
                              sizeof(alias_rreq),                             //
                              4096,                                           //
                              reply,                                          //
                              &reply_size),
              8);
    EXPECT_EQ(reply_size, 4096);
    const uint8_t* const expected_lba20 =
        dev.namespaces[0].storage.data() + 20 * 512;
    EXPECT_EQ(std::memcmp(reply, expected_lba20, 4096), 0);
  }

  // Null out_reply_size/req_buf, negative reply_cap, kNone/unknown op,
  // wrong req_size, OOB disk_index (+ Reset() stale slot), reply_cap OOB, and
  // RW Read/Write bounds/reserved validation.
  {
    const auto dispatch = [&](const void* const req,
                              const int64_t sz) -> int64_t {
      return server.Dispatch(static_cast<const uint8_t*>(req),  //
                             sz,                                //
                             sizeof(reply),                     //
                             reply,                             //
                             &reply_size);
    };
    const int io_before = dev.io_commands_seen;
    const int flush_before = dev.flush_count;

    const uint8_t short_req[2] = {1, 0};
    EXPECT_EQ(server.Dispatch(short_req,          //
                              sizeof(short_req),  //
                              sizeof(reply),      //
                              reply,              //
                              nullptr),
              kSyscallErrorInvalid);
    const BlockGetInfoRequest valid_info{.op = BlockRpcOp::kGetInfo,
                                         .disk_index = 0};
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&valid_info),  //
                              sizeof(valid_info),                             //
                              sizeof(reply),                                  //
                              reply,                                          //
                              nullptr),
              kSyscallErrorInvalid);
    reply_size = 77;
    EXPECT_EQ(
        server.Dispatch(short_req, sizeof(short_req), -1, reply, &reply_size),
        kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);
    reply_size = 77;
    EXPECT_EQ(dispatch(short_req, sizeof(short_req)), kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);
    const uint8_t short_req3[3] = {1, 0, 0};
    reply_size = 77;
    EXPECT_EQ(dispatch(short_req3, sizeof(short_req3)), kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);
    reply_size = 77;
    EXPECT_EQ(dispatch(nullptr, 8), kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);
    reply_size = 77;
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&valid_info),  //
                              sizeof(valid_info),                             //
                              -1,                                             //
                              reply,                                          //
                              &reply_size),
              kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);

    // W41 / WILD_02: kNone and unknown opcode in 4-byte and 8-byte requests.
    const uint32_t op_none4 = static_cast<uint32_t>(BlockRpcOp::kNone);
    const BlockGetInfoRequest op_none8{.op = BlockRpcOp::kNone,
                                       .disk_index = 0};
    const uint32_t op_bad4 = 99u;
    const BlockGetInfoRequest op_bad8{.op = static_cast<BlockRpcOp>(99),
                                      .disk_index = 0};
    reply_size = 77;
    EXPECT_EQ(dispatch(&op_none4, sizeof(op_none4)), kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);
    reply_size = 77;
    EXPECT_EQ(dispatch(&op_none8, sizeof(op_none8)), kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);
    reply_size = 77;
    EXPECT_EQ(dispatch(&op_bad4, sizeof(op_bad4)), kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);
    reply_size = 77;
    EXPECT_EQ(dispatch(&op_bad8, sizeof(op_bad8)), kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);

    // W43, W49, L390, L426, W47: truncated & oversized req_size with zeroed
    // ASan-safe staging buffers.
    uint8_t get_pad[sizeof(BlockGetInfoRequest) + 4] = {};
    std::memcpy(get_pad, &valid_info, sizeof(valid_info));
    EXPECT_EQ(dispatch(get_pad, 4), kSyscallErrorInvalid);
    EXPECT_EQ(dispatch(get_pad, sizeof(BlockGetInfoRequest) - 1),
              kSyscallErrorInvalid);
    EXPECT_EQ(dispatch(get_pad, sizeof(BlockGetInfoRequest) + 1),
              kSyscallErrorInvalid);

    const BlockReadRequest valid_read{
        .op = BlockRpcOp::kRead,
        .disk_index = 0,
        .lba = 0,
        .block_count = 1,
        .reserved = 0,
    };
    uint8_t read_pad[sizeof(BlockReadRequest) + 4] = {};
    std::memcpy(read_pad, &valid_read, sizeof(valid_read));
    EXPECT_EQ(dispatch(read_pad, 4), kSyscallErrorInvalid);
    EXPECT_EQ(dispatch(read_pad, sizeof(BlockReadRequest) - 1),
              kSyscallErrorInvalid);
    EXPECT_EQ(dispatch(read_pad, sizeof(BlockReadRequest) + 1),
              kSyscallErrorInvalid);

    uint8_t wpad[sizeof(BlockWriteRequest) + 516] = {};
    BlockWriteRequest w_hdr{
        .op = BlockRpcOp::kWrite,
        .disk_index = 0,
        .lba = 0,
        .block_count = 0,
        .reserved = 0,
    };
    std::memcpy(wpad, &w_hdr, sizeof(w_hdr));
    EXPECT_EQ(dispatch(wpad, 4), kSyscallErrorInvalid);
    EXPECT_EQ(dispatch(wpad, sizeof(BlockWriteRequest) - 1),
              kSyscallErrorInvalid);
    EXPECT_EQ(dispatch(wpad, sizeof(BlockWriteRequest) + 4),
              kSyscallErrorInvalid);
    w_hdr.block_count = 1;
    std::memcpy(wpad, &w_hdr, sizeof(w_hdr));
    EXPECT_EQ(dispatch(wpad, sizeof(BlockWriteRequest)), kSyscallErrorInvalid);
    EXPECT_EQ(dispatch(wpad, sizeof(BlockWriteRequest) + 511),
              kSyscallErrorInvalid);
    EXPECT_EQ(dispatch(wpad, sizeof(BlockWriteRequest) + 513),
              kSyscallErrorInvalid);

    const BlockFlushRequest valid_flush{.op = BlockRpcOp::kFlush,
                                        .disk_index = 0};
    uint8_t fpad[sizeof(BlockFlushRequest) + 4] = {};
    std::memcpy(fpad, &valid_flush, sizeof(valid_flush));
    EXPECT_EQ(dispatch(fpad, 4), kSyscallErrorInvalid);
    EXPECT_EQ(dispatch(fpad, sizeof(BlockFlushRequest) - 1),
              kSyscallErrorInvalid);
    EXPECT_EQ(dispatch(fpad, sizeof(BlockFlushRequest) + 1),
              kSyscallErrorInvalid);

    // M2R8-09 (HUNT_60, HUNT_READ_REQ_SZ_VS_DISK, HUNT_70): oversized
    // kGetInfo, kRead (reserved == 0), and kFlush with out-of-range disk_index
    // must fail kSyscallErrorInvalid before kSyscallErrorNoDevice.
    const BlockGetInfoRequest oob_info_sz{.op = BlockRpcOp::kGetInfo,
                                          .disk_index = 99};
    uint8_t oob_get_pad[sizeof(BlockGetInfoRequest) + 8] = {};
    std::memcpy(oob_get_pad, &oob_info_sz, sizeof(oob_info_sz));
    for (const int64_t extra : {4, 8}) {
      reply_size = 77;
      EXPECT_EQ(dispatch(oob_get_pad, sizeof(BlockGetInfoRequest) + extra),
                kSyscallErrorInvalid);
      EXPECT_EQ(reply_size, 0);
    }

    const BlockReadRequest oob_read_sz{
        .op = BlockRpcOp::kRead,
        .disk_index = 99,
        .lba = 0,
        .block_count = 1,
        .reserved = 0,
    };
    uint8_t oob_read_pad[sizeof(BlockReadRequest) + 8] = {};
    std::memcpy(oob_read_pad, &oob_read_sz, sizeof(oob_read_sz));
    for (const int64_t extra : {4, 8}) {
      reply_size = 77;
      EXPECT_EQ(dispatch(oob_read_pad, sizeof(BlockReadRequest) + extra),
                kSyscallErrorInvalid);
      EXPECT_EQ(reply_size, 0);
    }

    const BlockFlushRequest oob_flush_sz{.op = BlockRpcOp::kFlush,
                                         .disk_index = 99};
    uint8_t oob_flush_pad[sizeof(BlockFlushRequest) + 8] = {};
    std::memcpy(oob_flush_pad, &oob_flush_sz, sizeof(oob_flush_sz));
    for (const int64_t extra : {4, 8}) {
      reply_size = 77;
      EXPECT_EQ(dispatch(oob_flush_pad, sizeof(BlockFlushRequest) + extra),
                kSyscallErrorInvalid);
      EXPECT_EQ(reply_size, 0);
    }

    // H30: Dispatch rejects reply_cap < 0 even on 0-block kWrite and kFlush.
    const BlockWriteRequest zero_write{
        .op = BlockRpcOp::kWrite,
        .disk_index = 0,
        .lba = 0,
        .block_count = 0,
        .reserved = 0,
    };
    reply_size = 77;
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&zero_write),  //
                              sizeof(zero_write),                             //
                              -1,                                             //
                              nullptr,                                        //
                              &reply_size),
              kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);

    reply_size = 77;
    EXPECT_EQ(
        server.Dispatch(reinterpret_cast<const uint8_t*>(&valid_flush),  //
                        sizeof(valid_flush),                             //
                        -1,                                              //
                        nullptr,                                         //
                        &reply_size),
        kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);

    // W50-W52 & HUNT_25a..d: out-of-range disk_index across all 4 RPCs.
    for (const uint32_t bad_idx : {1u, 16u, 99u, 0x80000000u, 0xFFFFFFFFu}) {
      const BlockGetInfoRequest gi{.op = BlockRpcOp::kGetInfo,
                                   .disk_index = bad_idx};
      reply_size = 77;
      EXPECT_EQ(dispatch(&gi, sizeof(gi)), kSyscallErrorNoDevice);
      EXPECT_EQ(reply_size, 0);

      const BlockReadRequest rd{
          .op = BlockRpcOp::kRead,
          .disk_index = bad_idx,
          .lba = 0,
          .block_count = 1,
          .reserved = 0,
      };
      reply_size = 77;
      EXPECT_EQ(dispatch(&rd, sizeof(rd)), kSyscallErrorNoDevice);
      EXPECT_EQ(reply_size, 0);

      const BlockWriteRequest wr0{
          .op = BlockRpcOp::kWrite,
          .disk_index = bad_idx,
          .lba = 0,
          .block_count = 0,
          .reserved = 0,
      };
      EXPECT_EQ(dispatch(&wr0, sizeof(wr0)), kSyscallErrorNoDevice);

      uint8_t wr1_buf[sizeof(BlockWriteRequest) + 512] = {};
      const BlockWriteRequest wr1{
          .op = BlockRpcOp::kWrite,
          .disk_index = bad_idx,
          .lba = 0,
          .block_count = 1,
          .reserved = 0,
      };
      std::memcpy(wr1_buf, &wr1, sizeof(wr1));
      EXPECT_EQ(dispatch(wr1_buf, sizeof(wr1_buf)), kSyscallErrorNoDevice);

      const BlockFlushRequest fl{.op = BlockRpcOp::kFlush,
                                 .disk_index = bad_idx};
      EXPECT_EQ(dispatch(&fl, sizeof(fl)), kSyscallErrorNoDevice);
    }
    const BlockGetInfoRequest oob_gi{.op = BlockRpcOp::kGetInfo,
                                     .disk_index = 99};
    reply_size = 77;
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&oob_gi),  //
                              sizeof(oob_gi),                             //
                              0,                                          //
                              nullptr,                                    //
                              &reply_size),
              kSyscallErrorNoDevice);
    EXPECT_EQ(reply_size, 0);
    // WB11, WB26_SHORT_ONLY, WB06_SHORT_ONLY: insufficient non-null reply_cap
    // on kGetInfo and short kRead/kWrite sizes in [8, sizeof(req) - 1] with
    // out-of-range disk_index must check disk_index (kGetInfo) and req_size
    // (kRead/kWrite) in documented precedence order without writing tiny_reply.
    {
      uint8_t tiny_reply[sizeof(BlockDeviceInfo) - 1];
      std::memset(tiny_reply, 0xA5, sizeof(tiny_reply));
      const int32_t tiny_cap =
          static_cast<int32_t>(sizeof(BlockDeviceInfo)) - 1;
      for (const int32_t cap : {int32_t{0}, tiny_cap}) {
        reply_size = 77;
        EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&oob_gi),  //
                                  sizeof(oob_gi),                             //
                                  cap,                                        //
                                  tiny_reply,                                 //
                                  &reply_size),
                  kSyscallErrorNoDevice);
        EXPECT_EQ(reply_size, 0);
      }

      const int64_t min_short_sz =
          static_cast<int64_t>(sizeof(BlockGetInfoRequest));

      const BlockReadRequest oob_short_rd_hdr{
          .op = BlockRpcOp::kRead,
          .disk_index = 99,
          .lba = 0,
          .block_count = 0,
          .reserved = 0,
      };
      uint8_t oob_short_rd[sizeof(BlockReadRequest)] = {};
      std::memcpy(oob_short_rd, &oob_short_rd_hdr, sizeof(oob_short_rd_hdr));
      const int64_t rd_short_max =
          static_cast<int64_t>(sizeof(BlockReadRequest)) - 1;
      for (int64_t short_sz = min_short_sz; short_sz <= rd_short_max;
           ++short_sz) {
        reply_size = 77;
        EXPECT_EQ(server.Dispatch(oob_short_rd,  //
                                  short_sz,      //
                                  tiny_cap,      //
                                  tiny_reply,    //
                                  &reply_size),
                  kSyscallErrorInvalid);
        EXPECT_EQ(reply_size, 0);
      }

      const BlockWriteRequest oob_short_wr_hdr{
          .op = BlockRpcOp::kWrite,
          .disk_index = 99,
          .lba = 0,
          .block_count = 0,
          .reserved = 0,
      };
      uint8_t oob_short_wr[sizeof(BlockWriteRequest)] = {};
      std::memcpy(oob_short_wr, &oob_short_wr_hdr, sizeof(oob_short_wr_hdr));
      const int64_t wr_short_max =
          static_cast<int64_t>(sizeof(BlockWriteRequest)) - 1;
      for (int64_t short_sz = min_short_sz; short_sz <= wr_short_max;
           ++short_sz) {
        reply_size = 77;
        EXPECT_EQ(server.Dispatch(oob_short_wr,  //
                                  short_sz,      //
                                  tiny_cap,      //
                                  tiny_reply,    //
                                  &reply_size),
                  kSyscallErrorInvalid);
        EXPECT_EQ(reply_size, 0);
      }
      EXPECT_THAT(tiny_reply, t::Each(t::Eq(0xA5)));
    }

    const BlockGetInfoRequest small_cap{.op = BlockRpcOp::kGetInfo,
                                        .disk_index = 0};
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&small_cap),  //
                              sizeof(small_cap),                             //
                              16,                                            //
                              reply,                                         //
                              &reply_size),
              kSyscallErrorInvalid);
    reply_size = 77;
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&small_cap),  //
                              sizeof(small_cap),                             //
                              sizeof(BlockDeviceInfo),                       //
                              nullptr,                                       //
                              &reply_size),
              kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);

    uint8_t info_reply[sizeof(BlockDeviceInfo)];
    std::memset(info_reply, 0xA5, sizeof(info_reply));
    reply_size = 77;
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&small_cap),  //
                              sizeof(small_cap),                             //
                              sizeof(BlockDeviceInfo) - 1,                   //
                              info_reply,                                    //
                              &reply_size),
              kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);
    EXPECT_THAT(info_reply, t::Each(t::Eq(0xA5)));

    // W44: HandleRead reply_cap < bytes (256, 511) and reply_buf == nullptr.
    uint8_t small_reply[512];
    std::memset(small_reply, 0xA5, sizeof(small_reply));
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&valid_read),  //
                              sizeof(valid_read),                             //
                              256,                                            //
                              small_reply,                                    //
                              &reply_size),
              kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);
    EXPECT_THAT(small_reply, t::Each(t::Eq(0xA5)));

    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&valid_read),  //
                              sizeof(valid_read),                             //
                              511,                                            //
                              small_reply,                                    //
                              &reply_size),
              kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);
    EXPECT_THAT(small_reply, t::Each(t::Eq(0xA5)));

    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&valid_read),  //
                              sizeof(valid_read),                             //
                              512,                                            //
                              nullptr,                                        //
                              &reply_size),
              kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);

    // W45 & W46: running RW HandleWrite & HandleRead bounds, reserved, LBA.
    const BlockReadRequest bad_res{
        .op = BlockRpcOp::kRead,
        .disk_index = 0,
        .lba = 0,
        .block_count = 1,
        .reserved = 1,
    };
    EXPECT_EQ(dispatch(&bad_res, sizeof(bad_res)), kSyscallErrorInvalid);

    const BlockReadRequest bad_read_cases[] = {
        {.op = BlockRpcOp::kRead,
         .disk_index = 0,
         .lba = 0,
         .block_count = 0,
         .reserved = 1},
        {.op = BlockRpcOp::kRead,
         .disk_index = 0,
         .lba = 0,
         .block_count = 1,
         .reserved = 2u},
        {.op = BlockRpcOp::kRead,
         .disk_index = 0,
         .lba = 0,
         .block_count = 1,
         .reserved = 0x80000000u},
        {.op = BlockRpcOp::kRead,
         .disk_index = 0,
         .lba = 0,
         .block_count = 1,
         .reserved = 4u},
        {.op = BlockRpcOp::kRead,
         .disk_index = 0,
         .lba = 0,
         .block_count = 1,
         .reserved = 0x00010000u},
        {.op = BlockRpcOp::kRead,
         .disk_index = 0,
         .lba = 0,
         .block_count = 1,
         .reserved = 0x7FFFFFFCu},
        {.op = BlockRpcOp::kRead,
         .disk_index = 0,
         .lba = 0,
         .block_count = -1,
         .reserved = 0},
        {.op = BlockRpcOp::kRead,
         .disk_index = 0,
         .lba = 0,
         .block_count = 9,
         .reserved = 0},
        {.op = BlockRpcOp::kRead,
         .disk_index = 0,
         .lba = 129,
         .block_count = 0,
         .reserved = 0},
        {.op = BlockRpcOp::kRead,
         .disk_index = 0,
         .lba = UINT64_MAX,
         .block_count = 0,
         .reserved = 0},
        {.op = BlockRpcOp::kRead,
         .disk_index = 0,
         .lba = 128,
         .block_count = 1,
         .reserved = 0},
        {.op = BlockRpcOp::kRead,
         .disk_index = 0,
         .lba = 127,
         .block_count = 2,
         .reserved = 0},
        {.op = BlockRpcOp::kRead,
         .disk_index = 0,
         .lba = UINT64_MAX,
         .block_count = 1,
         .reserved = 0},
        {.op = BlockRpcOp::kRead,
         .disk_index = 0,
         .lba = UINT64_MAX - 1,
         .block_count = 2,
         .reserved = 0},
    };
    for (const BlockReadRequest& bad_r : bad_read_cases) {
      EXPECT_EQ(dispatch(&bad_r, sizeof(bad_r)), kSyscallErrorInvalid);
    }

    uint8_t big_wbuf[sizeof(BlockWriteRequest) + 9 * 512] = {};
    const BlockWriteRequest bad_write_hdrs[] = {
        {.op = BlockRpcOp::kWrite,
         .disk_index = 0,
         .lba = 0,
         .block_count = 0,
         .reserved = 1},
        {.op = BlockRpcOp::kWrite,
         .disk_index = 0,
         .lba = 0,
         .block_count = 1,
         .reserved = 1},
        {.op = BlockRpcOp::kWrite,
         .disk_index = 0,
         .lba = 0,
         .block_count = 1,
         .reserved = 2u},
        {.op = BlockRpcOp::kWrite,
         .disk_index = 0,
         .lba = 0,
         .block_count = 1,
         .reserved = 0x80000000u},
        {.op = BlockRpcOp::kWrite,
         .disk_index = 0,
         .lba = 0,
         .block_count = 1,
         .reserved = 4u},
        {.op = BlockRpcOp::kWrite,
         .disk_index = 0,
         .lba = 0,
         .block_count = 1,
         .reserved = 0x00010000u},
        {.op = BlockRpcOp::kWrite,
         .disk_index = 0,
         .lba = 0,
         .block_count = 1,
         .reserved = 0x7FFFFFFCu},
        {.op = BlockRpcOp::kWrite,
         .disk_index = 0,
         .lba = 0,
         .block_count = -1,
         .reserved = 0},
        {.op = BlockRpcOp::kWrite,
         .disk_index = 0,
         .lba = 0,
         .block_count = 9,
         .reserved = 0},
        {.op = BlockRpcOp::kWrite,
         .disk_index = 0,
         .lba = 129,
         .block_count = 0,
         .reserved = 0},
        {.op = BlockRpcOp::kWrite,
         .disk_index = 0,
         .lba = UINT64_MAX,
         .block_count = 0,
         .reserved = 0},
        {.op = BlockRpcOp::kWrite,
         .disk_index = 0,
         .lba = 128,
         .block_count = 1,
         .reserved = 0},
        {.op = BlockRpcOp::kWrite,
         .disk_index = 0,
         .lba = 125,
         .block_count = 4,
         .reserved = 0},
        {.op = BlockRpcOp::kWrite,
         .disk_index = 0,
         .lba = UINT64_MAX,
         .block_count = 1,
         .reserved = 0},
        {.op = BlockRpcOp::kWrite,
         .disk_index = 0,
         .lba = UINT64_MAX - 1,
         .block_count = 2,
         .reserved = 0},
    };
    for (const BlockWriteRequest& hdr : bad_write_hdrs) {
      std::memcpy(big_wbuf, &hdr, sizeof(hdr));
      const int64_t payload = (hdr.block_count > 0)
                                  ? static_cast<int64_t>(hdr.block_count) * 512
                                  : 0;
      EXPECT_EQ(dispatch(big_wbuf, static_cast<int64_t>(sizeof(hdr)) + payload),
                kSyscallErrorInvalid);
    }

    // NEW_READ_RES_AFTER_DISK & NEW_WRITE_RES_AFTER_DISK: req.reserved != 0
    // takes precedence over out-of-range disk_index.
    const BlockReadRequest oob_read_res{
        .op = BlockRpcOp::kRead,
        .disk_index = 99,
        .lba = 0,
        .block_count = 0,
        .reserved = 1,
    };
    EXPECT_EQ(dispatch(&oob_read_res, sizeof(oob_read_res)),
              kSyscallErrorInvalid);
    const BlockWriteRequest oob_write_res{
        .op = BlockRpcOp::kWrite,
        .disk_index = 99,
        .lba = 0,
        .block_count = 0,
        .reserved = 1,
    };
    EXPECT_EQ(dispatch(&oob_write_res, sizeof(oob_write_res)),
              kSyscallErrorInvalid);

    EXPECT_EQ(dev.io_commands_seen, io_before);
    EXPECT_EQ(dev.flush_count, flush_before);

    // Valid exact boundaries: 0-block write at lba == 128 and 1-block read at
    // exact reply_cap == 512.
    const BlockWriteRequest zero_at_end{
        .op = BlockRpcOp::kWrite,
        .disk_index = 0,
        .lba = 128,
        .block_count = 0,
        .reserved = 0,
    };
    EXPECT_EQ(dispatch(&zero_at_end, sizeof(zero_at_end)), 0);
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&valid_read),  //
                              sizeof(valid_read),                             //
                              512,                                            //
                              small_reply,                                    //
                              &reply_size),
              1);
    EXPECT_EQ(reply_size, 512);
  }

  // BlockServer::Reset() drops disk_count_ without clearing disks_[1], so
  // disk_index == 1 must fail disk_index >= disk_count_ even without ASan.
  {
    sim::SimNvmeDevice rdev;
    sim::SimNvmeDevice::NamespaceConfig ns2;
    ns2.nsid = 2;
    ns2.block_count = 32;
    ns2.block_size = 512;
    rdev.namespaces.push_back(std::move(ns2));
    NvmeController rctrl;
    rctrl.Init(rdev, rdev);
    ASSERT_EQ(rctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(rctrl.Start(1), NvmeStatus::kOk);

    BlockServer reset_srv;
    ASSERT_TRUE(reset_srv.AddController(&rctrl));
    ASSERT_EQ(reset_srv.disk_count(), 2);
    reset_srv.Reset();
    ASSERT_EQ(reset_srv.disk_count(), 0);
    ASSERT_TRUE(reset_srv.AddController(&ctrl));
    ASSERT_EQ(reset_srv.disk_count(), 1);

    const BlockGetInfoRequest gi1{.op = BlockRpcOp::kGetInfo, .disk_index = 1};
    EXPECT_EQ(reset_srv.Dispatch(reinterpret_cast<const uint8_t*>(&gi1),  //
                                 sizeof(gi1),                             //
                                 sizeof(reply),                           //
                                 reply,                                   //
                                 &reply_size),
              kSyscallErrorNoDevice);
    const BlockReadRequest rd1{
        .op = BlockRpcOp::kRead,
        .disk_index = 1,
        .lba = 0,
        .block_count = 1,
        .reserved = 0,
    };
    EXPECT_EQ(reset_srv.Dispatch(reinterpret_cast<const uint8_t*>(&rd1),  //
                                 sizeof(rd1),                             //
                                 512,                                     //
                                 reply,                                   //
                                 &reply_size),
              kSyscallErrorNoDevice);
    const BlockWriteRequest wr1_0{
        .op = BlockRpcOp::kWrite,
        .disk_index = 1,
        .lba = 0,
        .block_count = 0,
        .reserved = 0,
    };
    EXPECT_EQ(reset_srv.Dispatch(reinterpret_cast<const uint8_t*>(&wr1_0),  //
                                 sizeof(wr1_0),                             //
                                 0,                                         //
                                 nullptr,                                   //
                                 &reply_size),
              kSyscallErrorNoDevice);
    uint8_t wr1_buf[sizeof(BlockWriteRequest) + 512] = {};
    const BlockWriteRequest wr1_1{
        .op = BlockRpcOp::kWrite,
        .disk_index = 1,
        .lba = 0,
        .block_count = 1,
        .reserved = 0,
    };
    std::memcpy(wr1_buf, &wr1_1, sizeof(wr1_1));
    EXPECT_EQ(
        reset_srv.Dispatch(wr1_buf, sizeof(wr1_buf), 0, nullptr, &reply_size),
        kSyscallErrorNoDevice);
    const BlockFlushRequest fl1{.op = BlockRpcOp::kFlush, .disk_index = 1};
    EXPECT_EQ(reset_srv.Dispatch(reinterpret_cast<const uint8_t*>(&fl1),  //
                                 sizeof(fl1),                             //
                                 0,                                       //
                                 nullptr,                                 //
                                 &reply_size),
              kSyscallErrorNoDevice);
  }
}

TEST(BlockServerTest,
     GetInfoPreservesStaticMetadataOnDeadControllerWhileIoChecksStateFirst) {
  sim::SimNvmeDevice dev0;
  dev0.model = "DeadCtrl0";
  dev0.namespaces[0].read_only = true;
  sim::SimNvmeDevice dev1;
  dev1.model = "LiveCtrl1";

  NvmeController ctrl0;
  ctrl0.Init(dev0, dev0, /*bus=*/0, /*device=*/4, /*function=*/0);
  ASSERT_EQ(ctrl0.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl0.Start(1), NvmeStatus::kOk);

  NvmeController ctrl1;
  ctrl1.Init(dev1, dev1, /*bus=*/0, /*device=*/5, /*function=*/0);
  ASSERT_EQ(ctrl1.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl1.Start(1), NvmeStatus::kOk);

  BlockServer server;
  ASSERT_TRUE(server.AddController(&ctrl0));
  ASSERT_TRUE(server.AddController(&ctrl1));
  EXPECT_EQ(server.disk_count(), 2);
  EXPECT_EQ(server.RunningDiskCount(), 2);

  // Fault ctrl0 into kError.
  dev0.hang_io_cmd = true;
  uint8_t sec[512] = {};
  EXPECT_EQ(ctrl0.ReadBlocks(0, 0, 1, sec), NvmeStatus::kTimeout);
  ASSERT_EQ(ctrl0.state(), NvmeState::kError);
  EXPECT_EQ(server.RunningDiskCount(), 1);

  uint8_t reply[kIpcMaxMessageSize] = {};
  int64_t reply_size = 0;

  // kGetInfo(0) must keep serving static metadata (including read_only) so
  // disk 1 stays visible.
  {
    const BlockGetInfoRequest req0{.op = BlockRpcOp::kGetInfo, .disk_index = 0};
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&req0),  //
                              sizeof(req0),                             //
                              sizeof(reply),                            //
                              reply,                                    //
                              &reply_size),
              2);
    EXPECT_EQ(reply_size, static_cast<int64_t>(sizeof(BlockDeviceInfo)));
    BlockDeviceInfo info0{};
    std::memcpy(&info0, reply, sizeof(info0));
    EXPECT_EQ(info0.read_only, 1);
    EXPECT_STREQ(info0.model, "DeadCtrl0");

    const BlockGetInfoRequest req1{.op = BlockRpcOp::kGetInfo, .disk_index = 1};
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&req1),  //
                              sizeof(req1),                             //
                              sizeof(reply),                            //
                              reply,                                    //
                              &reply_size),
              2);
    BlockDeviceInfo info1{};
    std::memcpy(&info1, reply, sizeof(info1));
    EXPECT_EQ(info1.read_only, 0);
    EXPECT_STREQ(info1.model, "LiveCtrl1");
  }

  // HandleRead/HandleWrite/HandleFlush on disk 0 (kError) must return
  // kSyscallErrorIo BEFORE checking OOB LBA, negative block_count, reply_cap,
  // or write payload length.
  {
    const BlockReadRequest oob_read{
        .op = BlockRpcOp::kRead,
        .disk_index = 0,
        .lba = UINT64_MAX,
        .block_count = -5,
        .reserved = 0,
    };
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&oob_read),  //
                              sizeof(oob_read),                             //
                              0,                                            //
                              nullptr,                                      //
                              &reply_size),
              kSyscallErrorIo);

    const BlockWriteRequest oob_write{
        .op = BlockRpcOp::kWrite,
        .disk_index = 0,
        .lba = UINT64_MAX,
        .block_count = 1,
        .reserved = 0,
    };
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&oob_write),  //
                              sizeof(oob_write),                             //
                              0,                                             //
                              nullptr,                                       //
                              &reply_size),
              kSyscallErrorIo);

    const BlockFlushRequest flush0{.op = BlockRpcOp::kFlush, .disk_index = 0};
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&flush0),  //
                              sizeof(flush0),                             //
                              0,                                          //
                              nullptr,                                    //
                              &reply_size),
              kSyscallErrorIo);
  }

  // Disk 1 remains fully operational.
  {
    uint8_t wbuf[sizeof(BlockWriteRequest) + 512] = {};
    const BlockWriteRequest wreq{
        .op = BlockRpcOp::kWrite,
        .disk_index = 1,
        .lba = 2,
        .block_count = 1,
        .reserved = 0,
    };
    std::memcpy(wbuf, &wreq, sizeof(wreq));
    wbuf[sizeof(wreq)] = 0x77;
    EXPECT_EQ(server.Dispatch(wbuf, sizeof(wbuf), 0, nullptr, &reply_size), 1);

    const BlockReadRequest rreq{
        .op = BlockRpcOp::kRead,
        .disk_index = 1,
        .lba = 2,
        .block_count = 1,
        .reserved = 0,
    };
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&rreq),  //
                              sizeof(rreq),                             //
                              512,                                      //
                              reply,                                    //
                              &reply_size),
              1);
    EXPECT_EQ(reply[0], 0x77);
  }

  // Halted controller (via Shutdown) returns kSyscallErrorNoDevice and drops
  // RunningDiskCount() to 0.
  EXPECT_EQ(ctrl1.Shutdown(), NvmeStatus::kOk);
  EXPECT_EQ(server.RunningDiskCount(), 0);
  const BlockReadRequest r1{
      .op = BlockRpcOp::kRead,
      .disk_index = 1,
      .lba = 0,
      .block_count = 1,
      .reserved = 0,
  };
  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&r1),  //
                            sizeof(r1),                             //
                            512,                                    //
                            reply,                                  //
                            &reply_size),
            kSyscallErrorNoDevice);
  const BlockWriteRequest w1{
      .op = BlockRpcOp::kWrite,
      .disk_index = 1,
      .lba = 0,
      .block_count = 0,
      .reserved = 0,
  };
  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&w1),  //
                            sizeof(w1),                             //
                            0,                                      //
                            nullptr,                                //
                            &reply_size),
            kSyscallErrorNoDevice);
  const BlockFlushRequest f1{.op = BlockRpcOp::kFlush, .disk_index = 1};
  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&f1),  //
                            sizeof(f1),                             //
                            0,                                      //
                            nullptr,                                //
                            &reply_size),
            kSyscallErrorNoDevice);
  const BlockReadRequest r1_oob{
      .op = BlockRpcOp::kRead,
      .disk_index = 1,
      .lba = UINT64_MAX,
      .block_count = -5,
      .reserved = 0,
  };
  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&r1_oob),  //
                            sizeof(r1_oob),                             //
                            0,                                          //
                            nullptr,                                    //
                            &reply_size),
            kSyscallErrorNoDevice);
  const BlockReadRequest r1_nocap{
      .op = BlockRpcOp::kRead,
      .disk_index = 1,
      .lba = 0,
      .block_count = 1,
      .reserved = 0,
  };
  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&r1_nocap),  //
                            sizeof(r1_nocap),                             //
                            0,                                            //
                            nullptr,                                      //
                            &reply_size),
            kSyscallErrorNoDevice);
  const BlockWriteRequest w1_oob{
      .op = BlockRpcOp::kWrite,
      .disk_index = 1,
      .lba = UINT64_MAX,
      .block_count = 1,
      .reserved = 0,
  };
  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&w1_oob),  //
                            sizeof(w1_oob),                             //
                            0,                                          //
                            nullptr,                                    //
                            &reply_size),
            kSyscallErrorNoDevice);
  const BlockWriteRequest w1_short{
      .op = BlockRpcOp::kWrite,
      .disk_index = 1,
      .lba = 0,
      .block_count = 1,
      .reserved = 0,
  };
  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&w1_short),  //
                            sizeof(w1_short),                             //
                            0,                                            //
                            nullptr,                                      //
                            &reply_size),
            kSyscallErrorNoDevice);
}

TEST(BlockServerTest, WriteToReadOnlyNamespaceReturnsPermissionError) {
  sim::SimNvmeDevice dev;
  dev.namespaces[0].read_only = true;
  dev.namespaces[0].advertise_nsattr_ro = true;
  NvmeController ctrl;
  ctrl.Init(dev, dev);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);

  BlockServer server;
  ASSERT_TRUE(server.AddController(&ctrl));

  uint8_t wbuf[sizeof(BlockWriteRequest) + 512] = {};
  const BlockWriteRequest wreq{
      .op = BlockRpcOp::kWrite,
      .disk_index = 0,
      .lba = 0,
      .block_count = 1,
      .reserved = 0,
  };
  std::memcpy(wbuf, &wreq, sizeof(wreq));
  int64_t reply_size = 0;
  EXPECT_EQ(server.Dispatch(wbuf, sizeof(wbuf), 0, nullptr, &reply_size),
            kSyscallErrorPermission);
  EXPECT_EQ(dev.io_commands_seen, 0);

  const BlockWriteRequest zero_wreq{
      .op = BlockRpcOp::kWrite,
      .disk_index = 0,
      .lba = 0,
      .block_count = 0,
      .reserved = 0,
  };
  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&zero_wreq),  //
                            sizeof(zero_wreq),                             //
                            0,                                             //
                            nullptr,                                       //
                            &reply_size),
            0);

  // W45/W46 on RO disk: reserved != 0, block_count > max, and OOB LBA must
  // return kSyscallErrorInvalid BEFORE hitting ns.read_only permission error.
  uint8_t big_wbuf[sizeof(BlockWriteRequest) + 9 * 512] = {};
  const BlockWriteRequest ro_invalid[] = {
      {.op = BlockRpcOp::kWrite,
       .disk_index = 0,
       .lba = 0,
       .block_count = 1,
       .reserved = 1},
      {.op = BlockRpcOp::kWrite,
       .disk_index = 0,
       .lba = 0,
       .block_count = 9,
       .reserved = 0},
      {.op = BlockRpcOp::kWrite,
       .disk_index = 0,
       .lba = 128,
       .block_count = 1,
       .reserved = 0},
      {.op = BlockRpcOp::kWrite,
       .disk_index = 0,
       .lba = UINT64_MAX,
       .block_count = 1,
       .reserved = 0},
      {.op = BlockRpcOp::kWrite,
       .disk_index = 0,
       .lba = 129,
       .block_count = 0,
       .reserved = 0},
  };
  for (const BlockWriteRequest& hdr : ro_invalid) {
    std::memcpy(big_wbuf, &hdr, sizeof(hdr));
    const int64_t payload =
        (hdr.block_count > 0) ? static_cast<int64_t>(hdr.block_count) * 512 : 0;
    EXPECT_EQ(server.Dispatch(big_wbuf,                                     //
                              static_cast<int64_t>(sizeof(hdr)) + payload,  //
                              0,                                            //
                              nullptr,                                      //
                              &reply_size),
              kSyscallErrorInvalid);
  }
  const BlockWriteRequest ro_bad_len{
      .op = BlockRpcOp::kWrite,
      .disk_index = 0,
      .lba = 0,
      .block_count = 1,
      .reserved = 0,
  };
  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&ro_bad_len),  //
                            sizeof(ro_bad_len),                             //
                            0,                                              //
                            nullptr,                                        //
                            &reply_size),
            kSyscallErrorInvalid);
  std::memcpy(big_wbuf, &ro_bad_len, sizeof(ro_bad_len));
  EXPECT_EQ(server.Dispatch(big_wbuf,                         //
                            sizeof(BlockWriteRequest) + 513,  //
                            0,                                //
                            nullptr,                          //
                            &reply_size),
            kSyscallErrorInvalid);
  const BlockWriteRequest zero_at_end{
      .op = BlockRpcOp::kWrite,
      .disk_index = 0,
      .lba = 128,
      .block_count = 0,
      .reserved = 0,
  };
  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&zero_at_end),  //
                            sizeof(zero_at_end),                             //
                            0,                                               //
                            nullptr,                                         //
                            &reply_size),
            0);
  EXPECT_EQ(dev.io_commands_seen, 0);
}

TEST(BlockServerTest, MapNvmeErrorMapsAllControllerFailuresThroughDispatch) {
  // Part 1: Non-fatal kRunning errors (kCommandFailed, kInvalidArgument).
  {
    sim::SimNvmeDevice dev;
    NvmeController ctrl;
    ctrl.Init(dev, dev);
    ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);

    BlockServer server;
    ASSERT_TRUE(server.AddController(&ctrl));
    int64_t reply_size = 0;

    const BlockFlushRequest freq{.op = BlockRpcOp::kFlush, .disk_index = 0};
    dev.fail_flush = true;
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&freq),  //
                              sizeof(freq),                             //
                              0,                                        //
                              nullptr,                                  //
                              &reply_size),
              kSyscallErrorIo);
    EXPECT_EQ(reply_size, 0);
    EXPECT_EQ(ctrl.state(), NvmeState::kRunning);
    dev.fail_flush = false;
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&freq),  //
                              sizeof(freq),                             //
                              0,                                        //
                              nullptr,                                  //
                              &reply_size),
              0);

    uint8_t read_buf[512];
    std::memset(read_buf, 0xA5, sizeof(read_buf));
    const BlockReadRequest rreq{
        .op = BlockRpcOp::kRead,
        .disk_index = 0,
        .lba = 0,
        .block_count = 1,
        .reserved = 0,
    };
    dev.inject_next_io_status =
        static_cast<uint16_t>(kStatusCodeInvalidField << 1);
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&rreq),  //
                              sizeof(rreq),                             //
                              sizeof(read_buf),                         //
                              read_buf,                                 //
                              &reply_size),
              kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);
    EXPECT_THAT(read_buf, t::Each(t::Eq(0xA5)));
    EXPECT_EQ(ctrl.state(), NvmeState::kRunning);

    uint8_t wbuf[sizeof(BlockWriteRequest) + 512] = {};
    const BlockWriteRequest wreq{
        .op = BlockRpcOp::kWrite,
        .disk_index = 0,
        .lba = 0,
        .block_count = 1,
        .reserved = 0,
    };
    std::memcpy(wbuf, &wreq, sizeof(wreq));
    dev.inject_next_io_status =
        static_cast<uint16_t>(kStatusCodeLbaOutOfRange << 1);
    EXPECT_EQ(server.Dispatch(wbuf, sizeof(wbuf), 0, nullptr, &reply_size),
              kSyscallErrorInvalid);
    EXPECT_EQ(ctrl.state(), NvmeState::kRunning);

    dev.inject_next_io_status = static_cast<uint16_t>(1u << 9);
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&rreq),  //
                              sizeof(rreq),                             //
                              sizeof(read_buf),                         //
                              read_buf,                                 //
                              &reply_size),
              kSyscallErrorIo);
    EXPECT_EQ(reply_size, 0);
    EXPECT_THAT(read_buf, t::Each(t::Eq(0xA5)));
    EXPECT_EQ(ctrl.state(), NvmeState::kRunning);

    dev.inject_next_io_status = static_cast<uint16_t>(1u << 9);
    EXPECT_EQ(server.Dispatch(wbuf, sizeof(wbuf), 0, nullptr, &reply_size),
              kSyscallErrorIo);
    EXPECT_EQ(ctrl.state(), NvmeState::kRunning);
  }

  // Part 2: Hardware-only RO (kReadOnly -> kSyscallErrorPermission via CQE).
  {
    sim::SimNvmeDevice ro_dev;
    ro_dev.namespaces[0].read_only = true;
    ro_dev.namespaces[0].advertise_nsattr_ro = false;
    NvmeController ro_ctrl;
    ro_ctrl.Init(ro_dev, ro_dev);
    ASSERT_EQ(ro_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ro_ctrl.Start(1), NvmeStatus::kOk);
    ASSERT_FALSE(ro_ctrl.ns(0).read_only);

    BlockServer ro_srv;
    ASSERT_TRUE(ro_srv.AddController(&ro_ctrl));
    uint8_t reply[512] = {};
    int64_t reply_size = 0;

    const BlockGetInfoRequest greq{.op = BlockRpcOp::kGetInfo, .disk_index = 0};
    EXPECT_EQ(ro_srv.Dispatch(reinterpret_cast<const uint8_t*>(&greq),  //
                              sizeof(greq),                             //
                              sizeof(reply),                            //
                              reply,                                    //
                              &reply_size),
              1);
    BlockDeviceInfo info{};
    std::memcpy(&info, reply, sizeof(info));
    EXPECT_EQ(info.read_only, 0);

    uint8_t wbuf[sizeof(BlockWriteRequest) + 512] = {};
    const BlockWriteRequest wreq{
        .op = BlockRpcOp::kWrite,
        .disk_index = 0,
        .lba = 0,
        .block_count = 1,
        .reserved = 0,
    };
    std::memcpy(wbuf, &wreq, sizeof(wreq));
    EXPECT_EQ(ro_srv.Dispatch(wbuf, sizeof(wbuf), 0, nullptr, &reply_size),
              kSyscallErrorPermission);
    EXPECT_EQ(ro_dev.io_commands_seen, 1);
    EXPECT_EQ(ro_ctrl.state(), NvmeState::kRunning);

    const BlockReadRequest rreq{
        .op = BlockRpcOp::kRead,
        .disk_index = 0,
        .lba = 0,
        .block_count = 1,
        .reserved = 0,
    };
    EXPECT_EQ(ro_srv.Dispatch(reinterpret_cast<const uint8_t*>(&rreq),  //
                              sizeof(rreq),                             //
                              sizeof(reply),                            //
                              reply,                                    //
                              &reply_size),
              1);
    EXPECT_EQ(reply_size, 512);
  }

  // Part 3: Separate fresh kRunning controllers for state-poisoning faults
  // (kTimeout, kControllerFatal, kNotDecoding).
  {
    sim::SimNvmeDevice dev_to;
    NvmeController ctrl_to;
    ctrl_to.Init(dev_to, dev_to);
    ASSERT_EQ(ctrl_to.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl_to.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(ctrl_to.state(), NvmeState::kRunning);
    BlockServer srv_to;
    ASSERT_TRUE(srv_to.AddController(&ctrl_to));
    dev_to.hang_io_cmd = true;
    uint8_t reply[512] = {};
    int64_t reply_size = 0;
    const BlockReadRequest rreq{
        .op = BlockRpcOp::kRead,
        .disk_index = 0,
        .lba = 0,
        .block_count = 1,
        .reserved = 0,
    };
    EXPECT_EQ(srv_to.Dispatch(reinterpret_cast<const uint8_t*>(&rreq),  //
                              sizeof(rreq),                             //
                              sizeof(reply),                            //
                              reply,                                    //
                              &reply_size),
              kSyscallErrorIo);
    EXPECT_EQ(reply_size, 0);
    EXPECT_EQ(ctrl_to.state(), NvmeState::kError);
    EXPECT_EQ(srv_to.RunningDiskCount(), 0);
  }
  {
    sim::SimNvmeDevice dev_fat;
    NvmeController ctrl_fat;
    ctrl_fat.Init(dev_fat, dev_fat);
    ASSERT_EQ(ctrl_fat.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl_fat.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(ctrl_fat.state(), NvmeState::kRunning);
    BlockServer srv_fat;
    ASSERT_TRUE(srv_fat.AddController(&ctrl_fat));
    dev_fat.fatal_on_io = true;
    uint8_t wbuf[sizeof(BlockWriteRequest) + 512] = {};
    const BlockWriteRequest wreq{
        .op = BlockRpcOp::kWrite,
        .disk_index = 0,
        .lba = 0,
        .block_count = 1,
        .reserved = 0,
    };
    std::memcpy(wbuf, &wreq, sizeof(wreq));
    int64_t reply_size = 0;
    EXPECT_EQ(srv_fat.Dispatch(wbuf, sizeof(wbuf), 0, nullptr, &reply_size),
              kSyscallErrorIo);
    EXPECT_EQ(reply_size, 0);
    EXPECT_EQ(ctrl_fat.state(), NvmeState::kError);
    EXPECT_EQ(srv_fat.RunningDiskCount(), 0);
  }
  {
    sim::SimNvmeDevice dev_det;
    NvmeController ctrl_det;
    ctrl_det.Init(dev_det, dev_det);
    ASSERT_EQ(ctrl_det.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrl_det.Start(1), NvmeStatus::kOk);
    ASSERT_EQ(ctrl_det.state(), NvmeState::kRunning);
    BlockServer srv_det;
    ASSERT_TRUE(srv_det.AddController(&ctrl_det));
    dev_det.not_decoding = true;
    const BlockFlushRequest freq{.op = BlockRpcOp::kFlush, .disk_index = 0};
    int64_t reply_size = 0;
    EXPECT_EQ(srv_det.Dispatch(reinterpret_cast<const uint8_t*>(&freq),  //
                               sizeof(freq),                             //
                               0,                                        //
                               nullptr,                                  //
                               &reply_size),
              kSyscallErrorIo);
    EXPECT_EQ(reply_size, 0);
    EXPECT_EQ(ctrl_det.state(), NvmeState::kError);
    EXPECT_EQ(srv_det.RunningDiskCount(), 0);
  }
}

TEST(BlockServerTest,
     WriteRejectsLbaWrapBeforePermissionOnHugeReadOnlyNamespace) {
  FakeNvmeHost fake;
  fake.nsze = UINT64_MAX;
  fake.ncap = UINT64_MAX;
  fake.read_only = true;
  fake.advertise_nsattr_ro = true;

  NvmeController ctrl;
  ctrl.Init(fake, fake);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);

  BlockServer server;
  ASSERT_TRUE(server.AddController(&ctrl));

  uint8_t reply[1024] = {};
  int64_t reply_size = 0;
  const BlockGetInfoRequest greq{.op = BlockRpcOp::kGetInfo, .disk_index = 0};
  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&greq),  //
                            sizeof(greq),                             //
                            sizeof(reply),                            //
                            reply,                                    //
                            &reply_size),
            1);
  BlockDeviceInfo info{};
  std::memcpy(&info, reply, sizeof(info));
  EXPECT_EQ(info.block_count, UINT64_MAX);
  EXPECT_EQ(info.read_only, 1);

  uint8_t wbuf[sizeof(BlockWriteRequest) + 1024] = {};
  const auto write_blocks = [&](const uint64_t lba,
                                const int32_t blocks) -> int64_t {
    const BlockWriteRequest wreq{
        .op = BlockRpcOp::kWrite,
        .disk_index = 0,
        .lba = lba,
        .block_count = blocks,
        .reserved = 0,
    };
    std::memcpy(wbuf, &wreq, sizeof(wreq));
    const int64_t payload =
        (blocks > 0) ? static_cast<int64_t>(blocks) * 512 : 0;
    return server.Dispatch(wbuf,                                          //
                           static_cast<int64_t>(sizeof(wreq)) + payload,  //
                           0,                                             //
                           nullptr,                                       //
                           &reply_size);
  };

  EXPECT_EQ(write_blocks(0, 1), kSyscallErrorPermission);
  // WILD_09: wrapped LBA writes on RO UINT64_MAX must fail Invalid, not
  // Permission.
  EXPECT_EQ(write_blocks(UINT64_MAX, 1), kSyscallErrorInvalid);
  EXPECT_EQ(write_blocks(UINT64_MAX, 2), kSyscallErrorInvalid);
  EXPECT_EQ(write_blocks(UINT64_MAX - 1, 2), kSyscallErrorInvalid);
  EXPECT_EQ(write_blocks(UINT64_MAX, 0), 0);

  const BlockReadRequest r1{
      .op = BlockRpcOp::kRead,
      .disk_index = 0,
      .lba = UINT64_MAX,
      .block_count = 1,
      .reserved = 0,
  };
  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&r1),  //
                            sizeof(r1),                             //
                            sizeof(reply),                          //
                            reply,                                  //
                            &reply_size),
            kSyscallErrorInvalid);
  const BlockReadRequest r2{
      .op = BlockRpcOp::kRead,
      .disk_index = 0,
      .lba = UINT64_MAX - 1,
      .block_count = 2,
      .reserved = 0,
  };
  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&r2),  //
                            sizeof(r2),                             //
                            sizeof(reply),                          //
                            reply,                                  //
                            &reply_size),
            kSyscallErrorInvalid);
  EXPECT_EQ(fake.io_db_writes, 0);
}

TEST(BlockServerTest, Dispatches64BitLbaReadAndWriteWithoutTruncation) {
  FakeNvmeHost fake;
  fake.read_only = false;
  NvmeController ctrl;
  ctrl.Init(fake, fake);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);

  BlockServer server;
  ASSERT_TRUE(server.AddController(&ctrl));

  const uint64_t lba_w = 0xFEDCBA9876543210ULL;
  uint8_t wbuf[sizeof(BlockWriteRequest) + 512] = {};
  const BlockWriteRequest wreq{
      .op = BlockRpcOp::kWrite,
      .disk_index = 0,
      .lba = lba_w,
      .block_count = 1,
      .reserved = 0,
  };
  std::memcpy(wbuf, &wreq, sizeof(wreq));
  int64_t reply_size = 77;
  EXPECT_EQ(server.Dispatch(wbuf, sizeof(wbuf), 0, nullptr, &reply_size), 1);
  EXPECT_EQ(reply_size, 0);
  EXPECT_EQ(fake.last_io_opc, kNvmWrite);
  EXPECT_EQ(fake.last_io_lba, lba_w);

  const uint64_t lba_r = 0x123456789ABCDEF0ULL;
  const BlockReadRequest rreq{
      .op = BlockRpcOp::kRead,
      .disk_index = 0,
      .lba = lba_r,
      .block_count = 1,
      .reserved = 0,
  };
  uint8_t reply[512] = {};
  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&rreq),  //
                            sizeof(rreq),                             //
                            sizeof(reply),                            //
                            reply,                                    //
                            &reply_size),
            1);
  EXPECT_EQ(reply_size, 512);
  EXPECT_EQ(fake.last_io_opc, kNvmRead);
  EXPECT_EQ(fake.last_io_lba, lba_r);
}

TEST(BlockServerTest,
     MultiControllerRegistrationEnforcesMaxDisks16AndRejectsNonRunning) {
  sim::SimNvmeDevice halted_dev;
  NvmeController halted_ctrl;
  halted_ctrl.Init(halted_dev, halted_dev);
  ASSERT_EQ(halted_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);

  BlockServer server;
  EXPECT_FALSE(server.AddController(nullptr));
  EXPECT_FALSE(server.AddController(&halted_ctrl));

  NvmeController uninit_ctrl;
  uninit_ctrl.Init(halted_dev, halted_dev);
  EXPECT_FALSE(server.AddController(&uninit_ctrl));

  sim::SimNvmeDevice err_dev;
  NvmeController err_ctrl;
  err_ctrl.Init(err_dev, err_dev);
  ASSERT_EQ(err_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(err_ctrl.Start(1), NvmeStatus::kOk);
  err_dev.hang_io_cmd = true;
  uint8_t err_sec[512] = {};
  EXPECT_EQ(err_ctrl.ReadBlocks(0, 0, 1, err_sec), NvmeStatus::kTimeout);
  ASSERT_EQ(err_ctrl.state(), NvmeState::kError);
  ASSERT_EQ(err_ctrl.namespace_count(), 1);
  EXPECT_FALSE(server.AddController(&err_ctrl));
  EXPECT_EQ(server.disk_count(), 0);

  sim::SimNvmeDevice devs[3];
  NvmeController ctrls[3];
  for (int c = 0; c < 3; ++c) {
    devs[c].namespaces.clear();
    const int n_cnt = (c < 2) ? 8 : 1;
    for (int n = 0; n < n_cnt; ++n) {
      sim::SimNvmeDevice::NamespaceConfig ns;
      ns.nsid = static_cast<uint32_t>(n + 1);
      ns.block_count = 16;
      ns.block_size = 512;
      devs[c].namespaces.push_back(std::move(ns));
    }
    ctrls[c].Init(devs[c], devs[c]);
    ASSERT_EQ(ctrls[c].TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(ctrls[c].Start(1), NvmeStatus::kOk);
  }
  EXPECT_TRUE(server.AddController(&ctrls[0]));
  EXPECT_EQ(server.disk_count(), 8);
  EXPECT_TRUE(server.AddController(&ctrls[1]));
  EXPECT_EQ(server.disk_count(), 16);
  EXPECT_FALSE(server.AddController(&ctrls[2]));
  EXPECT_EQ(server.disk_count(), 16);
  {
    sim::SimNvmeDevice pdevs[3];
    NvmeController pctrls[3];
    const int pcounts[3] = {8, 6, 3};
    for (int c = 0; c < 3; ++c) {
      pdevs[c].namespaces.clear();
      for (int n = 0; n < pcounts[c]; ++n) {
        sim::SimNvmeDevice::NamespaceConfig ns;
        ns.nsid = static_cast<uint32_t>(n + 1);
        ns.block_count = 16;
        ns.block_size = 512;
        pdevs[c].namespaces.push_back(std::move(ns));
      }
      pctrls[c].Init(pdevs[c], pdevs[c]);
      ASSERT_EQ(pctrls[c].TakeOwnershipAndReset(), NvmeStatus::kOk);
      ASSERT_EQ(pctrls[c].Start(1), NvmeStatus::kOk);
    }
    BlockServer partial_srv;
    EXPECT_TRUE(partial_srv.AddController(&pctrls[0]));
    EXPECT_EQ(partial_srv.disk_count(), 8);
    EXPECT_TRUE(partial_srv.AddController(&pctrls[1]));
    EXPECT_EQ(partial_srv.disk_count(), 14);
    EXPECT_FALSE(partial_srv.AddController(&pctrls[2]));
    EXPECT_EQ(partial_srv.disk_count(), 16);
    EXPECT_EQ(partial_srv.RunningDiskCount(), 16);
  }
}

TEST(BlockServerTest, ZeroNamespaceControllerAddsZeroDisks) {
  sim::SimNvmeDevice dev;
  dev.namespaces.clear();
  NvmeController ctrl;
  ctrl.Init(dev, dev);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);

  BlockServer server;
  EXPECT_TRUE(server.AddController(&ctrl));
  EXPECT_EQ(server.disk_count(), 0);
  EXPECT_EQ(server.RunningDiskCount(), 0);
}

TEST(BlockServerTest, MultiNamespaceDispatchIsolatesDisksAndFlushNsid) {
  sim::SimNvmeDevice dev;
  dev.namespaces.clear();

  sim::SimNvmeDevice::NamespaceConfig ns0;
  ns0.nsid = 10u;
  ns0.block_count = 16;
  ns0.block_size = 512;
  ns0.InitStorage();
  std::memset(ns0.storage.data(), 0xA0, ns0.storage.size());
  dev.namespaces.push_back(std::move(ns0));

  sim::SimNvmeDevice::NamespaceConfig ns1;
  ns1.nsid = 20u;
  ns1.block_count = 32;
  ns1.block_size = 512;
  ns1.InitStorage();
  std::memset(ns1.storage.data(), 0xB1, ns1.storage.size());
  dev.namespaces.push_back(std::move(ns1));

  NvmeController ctrl;
  ctrl.Init(dev, dev);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);

  BlockServer server;
  ASSERT_TRUE(server.AddController(&ctrl));
  ASSERT_EQ(server.disk_count(), 2);

  uint8_t reply[kIpcMaxMessageSize] = {};
  int64_t reply_size = 0;

  // ADV_M2_R3_14a: disk 0 -> nsid 10 (16 blk), disk 1 -> nsid 20 (32 blk).
  {
    const BlockGetInfoRequest g0{.op = BlockRpcOp::kGetInfo, .disk_index = 0};
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&g0),  //
                              sizeof(g0),                             //
                              sizeof(reply),                          //
                              reply,                                  //
                              &reply_size),
              2);
    BlockDeviceInfo info0{};
    std::memcpy(&info0, reply, sizeof(info0));
    EXPECT_EQ(info0.nsid, 10u);
    EXPECT_EQ(info0.block_count, uint64_t{16});

    const BlockGetInfoRequest g1{.op = BlockRpcOp::kGetInfo, .disk_index = 1};
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&g1),  //
                              sizeof(g1),                             //
                              sizeof(reply),                          //
                              reply,                                  //
                              &reply_size),
              2);
    BlockDeviceInfo info1{};
    std::memcpy(&info1, reply, sizeof(info1));
    EXPECT_EQ(info1.nsid, 20u);
    EXPECT_EQ(info1.block_count, uint64_t{32});
  }

  // ADV_M2_R3_14c & 14b: Write/Read on disk 1 leaves disk 0 untouched.
  {
    uint8_t wbuf[sizeof(BlockWriteRequest) + 512] = {};
    const BlockWriteRequest w1{
        .op = BlockRpcOp::kWrite,
        .disk_index = 1,
        .lba = 3,
        .block_count = 1,
        .reserved = 0,
    };
    std::memcpy(wbuf, &w1, sizeof(w1));
    std::memset(wbuf + sizeof(w1), 0x5C, 512);
    EXPECT_EQ(server.Dispatch(wbuf, sizeof(wbuf), 0, nullptr, &reply_size), 1);
    EXPECT_THAT(dev.namespaces[0].storage, t::Each(t::Eq(0xA0)));

    const BlockReadRequest r1_written{
        .op = BlockRpcOp::kRead,
        .disk_index = 1,
        .lba = 3,
        .block_count = 1,
        .reserved = 0,
    };
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&r1_written),  //
                              sizeof(r1_written),                             //
                              512,                                            //
                              reply,                                          //
                              &reply_size),
              1);
    EXPECT_EQ(reply_size, 512);
    EXPECT_EQ(reply[0], 0x5C);
    EXPECT_EQ(reply[511], 0x5C);

    const BlockReadRequest r1_untouched{
        .op = BlockRpcOp::kRead,
        .disk_index = 1,
        .lba = 4,
        .block_count = 1,
        .reserved = 0,
    };
    EXPECT_EQ(
        server.Dispatch(reinterpret_cast<const uint8_t*>(&r1_untouched),  //
                        sizeof(r1_untouched),                             //
                        512,                                              //
                        reply,                                            //
                        &reply_size),
        1);
    EXPECT_EQ(reply[0], 0xB1);

    const BlockReadRequest r0_check{
        .op = BlockRpcOp::kRead,
        .disk_index = 0,
        .lba = 3,
        .block_count = 1,
        .reserved = 0,
    };
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&r0_check),  //
                              sizeof(r0_check),                             //
                              512,                                          //
                              reply,                                        //
                              &reply_size),
              1);
    EXPECT_EQ(reply[0], 0xA0);
  }

  // ADV_M2_R3_14d: Flush(1) writes SQE with exact nsid == 20u.
  const auto read_last_io_sqe = [&]() -> SubmissionEntry {
    if (dev.io_sq_size <= 0) {
      return SubmissionEntry{};
    }
    const int slot = (dev.io_sq_head + dev.io_sq_size - 1) % dev.io_sq_size;
    const int64_t sq_bytes = static_cast<int64_t>(dev.io_sq_size) *
                             static_cast<int64_t>(sizeof(SubmissionEntry));
    const uint8_t* const sq_ptr = dev.PhysToPtr(dev.io_sq_phys, sq_bytes);
    SubmissionEntry sqe{};
    if (sq_ptr != nullptr) {
      const int64_t off = static_cast<int64_t>(slot) *
                          static_cast<int64_t>(sizeof(SubmissionEntry));
      std::memcpy(&sqe, sq_ptr + off, sizeof(sqe));
    }
    return sqe;
  };
  {
    const BlockFlushRequest f1{.op = BlockRpcOp::kFlush, .disk_index = 1};
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&f1),  //
                              sizeof(f1),                             //
                              0,                                      //
                              nullptr,                                //
                              &reply_size),
              0);
    const SubmissionEntry sqe1 = read_last_io_sqe();
    EXPECT_EQ(static_cast<uint8_t>(sqe1.cdw0 & 0xFFu), kNvmFlush);
    EXPECT_EQ(sqe1.nsid, 20u);

    const BlockFlushRequest f0{.op = BlockRpcOp::kFlush, .disk_index = 0};
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&f0),  //
                              sizeof(f0),                             //
                              0,                                      //
                              nullptr,                                //
                              &reply_size),
              0);
    const SubmissionEntry sqe0 = read_last_io_sqe();
    EXPECT_EQ(static_cast<uint8_t>(sqe0.cdw0 & 0xFFu), kNvmFlush);
    EXPECT_EQ(sqe0.nsid, 10u);
  }
  {
    FakeNvmeHost fake;
    fake.nsze = 32;
    fake.ncap = 32;
    fake.present_nsids = {10u, 20u};
    NvmeController fake_ctrl;
    fake_ctrl.Init(fake, fake);
    ASSERT_EQ(fake_ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
    ASSERT_EQ(fake_ctrl.Start(1), NvmeStatus::kOk);
    BlockServer fake_srv;
    ASSERT_TRUE(fake_srv.AddController(&fake_ctrl));
    const BlockFlushRequest f1{.op = BlockRpcOp::kFlush, .disk_index = 1};
    EXPECT_EQ(fake_srv.Dispatch(reinterpret_cast<const uint8_t*>(&f1),  //
                                sizeof(f1),                             //
                                0,                                      //
                                nullptr,                                //
                                &reply_size),
              0);
    EXPECT_EQ(fake.last_io_opc, kNvmFlush);
    EXPECT_EQ(fake.last_io_nsid, 20u);
  }
}

TEST(BlockServerTest, Supports4KnGeometryBdfMetadataAnd39CharModel) {
  sim::SimNvmeDevice dev;
  const std::string full_model = "0123456789ABCDEFGHIJ0123456789abcdefghij";
  ASSERT_EQ(static_cast<int>(full_model.size()), kIdCtrlModelLength);
  dev.model = full_model;
  dev.serial = "SERIAL12345678901234";
  dev.namespaces[0].nsid = 7u;
  dev.namespaces[0].block_count = 16;
  dev.namespaces[0].block_size = 4096;
  dev.namespaces[0].InitStorage();

  NvmeController ctrl;
  ctrl.Init(dev, dev, /*bus=*/3, /*device=*/4, /*function=*/2);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);

  BlockServer server;
  ASSERT_TRUE(server.AddController(&ctrl));

  uint8_t reply[kIpcMaxMessageSize] = {};
  int64_t reply_size = 0;

  // ADV_M2_R3_15c & HUNT_20: 4Kn geometry, 3:4.2 BDF, nsid=7, 39-char model.
  const BlockGetInfoRequest greq{.op = BlockRpcOp::kGetInfo, .disk_index = 0};
  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&greq),  //
                            sizeof(greq),                             //
                            sizeof(reply),                            //
                            reply,                                    //
                            &reply_size),
            1);
  ASSERT_EQ(reply_size, static_cast<int64_t>(sizeof(BlockDeviceInfo)));
  BlockDeviceInfo info{};
  std::memcpy(&info, reply, sizeof(info));
  EXPECT_EQ(info.block_count, uint64_t{16});
  EXPECT_EQ(info.block_size, 4096);
  EXPECT_EQ(info.max_transfer_blocks, 1);
  EXPECT_EQ(info.bus, 3);
  EXPECT_EQ(info.device, 4);
  EXPECT_EQ(info.function, 2);
  EXPECT_EQ(info.nsid, 7u);
  EXPECT_EQ(info.read_only, 0);
  EXPECT_STREQ(info.model, full_model.substr(0, 39).c_str());
  EXPECT_EQ(static_cast<int>(std::strlen(info.model)), 39);
  EXPECT_STREQ(info.serial, "SERIAL12345678901234");

  // ADV_M2_R3_15b: 1-block write requires 4096 B, rejects 512 B & 2 blocks.
  uint8_t wbuf_bad512[sizeof(BlockWriteRequest) + 512] = {};
  const BlockWriteRequest wreq1{
      .op = BlockRpcOp::kWrite,
      .disk_index = 0,
      .lba = 5,
      .block_count = 1,
      .reserved = 0,
  };
  std::memcpy(wbuf_bad512, &wreq1, sizeof(wreq1));
  EXPECT_EQ(server.Dispatch(wbuf_bad512,          //
                            sizeof(wbuf_bad512),  //
                            0,                    //
                            nullptr,              //
                            &reply_size),
            kSyscallErrorInvalid);

  uint8_t wbuf_4k[sizeof(BlockWriteRequest) + 4096] = {};
  std::memcpy(wbuf_4k, &wreq1, sizeof(wreq1));
  for (int i = 0; i < 4096; ++i) {
    wbuf_4k[sizeof(wreq1) + i] = static_cast<uint8_t>((i * 3 + 7) & 0xFF);
  }
  EXPECT_EQ(server.Dispatch(wbuf_4k, sizeof(wbuf_4k), 0, nullptr, &reply_size),
            1);

  std::vector<uint8_t> wbuf_2blk(sizeof(BlockWriteRequest) + 8192, uint8_t{0});
  const BlockWriteRequest wreq2{
      .op = BlockRpcOp::kWrite,
      .disk_index = 0,
      .lba = 0,
      .block_count = 2,
      .reserved = 0,
  };
  std::memcpy(wbuf_2blk.data(), &wreq2, sizeof(wreq2));
  EXPECT_EQ(server.Dispatch(wbuf_2blk.data(),                        //
                            static_cast<int64_t>(wbuf_2blk.size()),  //
                            0,                                       //
                            nullptr,                                 //
                            &reply_size),
            kSyscallErrorInvalid);

  // ADV_M2_R3_15a: 1-block read requires 4096-B reply_cap, rejects 512/4095/2b.
  const BlockReadRequest rreq1{
      .op = BlockRpcOp::kRead,
      .disk_index = 0,
      .lba = 5,
      .block_count = 1,
      .reserved = 0,
  };
  uint8_t small_512[512];
  std::memset(small_512, 0xA5, sizeof(small_512));
  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&rreq1),  //
                            sizeof(rreq1),                             //
                            sizeof(small_512),                         //
                            small_512,                                 //
                            &reply_size),
            kSyscallErrorInvalid);
  EXPECT_EQ(reply_size, 0);
  EXPECT_THAT(small_512, t::Each(t::Eq(0xA5)));

  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&rreq1),  //
                            sizeof(rreq1),                             //
                            4095,                                      //
                            reply,                                     //
                            &reply_size),
            kSyscallErrorInvalid);
  EXPECT_EQ(reply_size, 0);

  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&rreq1),  //
                            sizeof(rreq1),                             //
                            4096,                                      //
                            reply,                                     //
                            &reply_size),
            1);
  EXPECT_EQ(reply_size, 4096);
  EXPECT_EQ(std::memcmp(reply, wbuf_4k + sizeof(wreq1), 4096), 0);

  const BlockReadRequest rreq2{
      .op = BlockRpcOp::kRead,
      .disk_index = 0,
      .lba = 0,
      .block_count = 2,
      .reserved = 0,
  };
  std::vector<uint8_t> big_8k(8192, uint8_t{0});
  EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&rreq2),  //
                            sizeof(rreq2),                             //
                            static_cast<int32_t>(big_8k.size()),       //
                            big_8k.data(),                             //
                            &reply_size),
            kSyscallErrorInvalid);
}

TEST(BlockServerTest,
     Supports1024And2048ByteSectorsAcrossControllerAndDispatch) {
  sim::SimNvmeDevice dev;
  dev.namespaces.clear();

  struct SectorSpec {
    uint32_t nsid;
    uint64_t block_count;
    int32_t block_size;
    int32_t max_blocks;
  };
  const SectorSpec specs[2] = {
      {.nsid = 1u, .block_count = 16, .block_size = 1024, .max_blocks = 4},
      {.nsid = 2u, .block_count = 12, .block_size = 2048, .max_blocks = 2},
  };
  for (int i = 0; i < 2; ++i) {
    sim::SimNvmeDevice::NamespaceConfig ns;
    ns.nsid = specs[i].nsid;
    ns.block_count = specs[i].block_count;
    ns.block_size = specs[i].block_size;
    ns.InitStorage();
    dev.namespaces.push_back(std::move(ns));
  }

  NvmeController ctrl;
  ctrl.Init(dev, dev);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.namespace_count(), 2);

  uint8_t tx_4k[4096];
  for (int i = 0; i < 4096; ++i) {
    tx_4k[i] = static_cast<uint8_t>((i * 5 + 19) & 0xFF);
  }
  for (int d = 0; d < 2; ++d) {
    const SectorSpec& sp = specs[d];
    EXPECT_EQ(ctrl.ns(d).nsid, sp.nsid);
    EXPECT_EQ(ctrl.ns(d).block_count, sp.block_count);
    EXPECT_EQ(ctrl.ns(d).block_size, sp.block_size);
    EXPECT_EQ(ctrl.ns(d).max_transfer_blocks, sp.max_blocks);

    uint8_t rx_4k[4096] = {};
    EXPECT_EQ(ctrl.WriteBlocks(d, 2, sp.max_blocks, tx_4k), NvmeStatus::kOk);
    EXPECT_EQ(ctrl.ReadBlocks(d, 2, sp.max_blocks, rx_4k), NvmeStatus::kOk);
    EXPECT_EQ(std::memcmp(tx_4k, rx_4k, 4096), 0);

    EXPECT_EQ(ctrl.WriteBlocks(d, 0, sp.max_blocks + 1, tx_4k),
              NvmeStatus::kInvalidArgument);
    EXPECT_EQ(ctrl.ReadBlocks(d, 0, sp.max_blocks + 1, rx_4k),
              NvmeStatus::kInvalidArgument);
  }

  BlockServer server;
  ASSERT_TRUE(server.AddController(&ctrl));
  ASSERT_EQ(server.disk_count(), 2);

  uint8_t reply[kIpcMaxMessageSize] = {};
  int64_t reply_size = 0;

  for (int d = 0; d < 2; ++d) {
    const SectorSpec& sp = specs[d];
    const uint32_t disk_idx = static_cast<uint32_t>(d);
    const int bs = sp.block_size;
    const int max_b = sp.max_blocks;
    const int hdr_bytes = static_cast<int>(sizeof(BlockWriteRequest));

    const BlockGetInfoRequest greq{.op = BlockRpcOp::kGetInfo,
                                   .disk_index = disk_idx};
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&greq),  //
                              sizeof(greq),                             //
                              sizeof(reply),                            //
                              reply,                                    //
                              &reply_size),
              2);
    ASSERT_EQ(reply_size, static_cast<int64_t>(sizeof(BlockDeviceInfo)));
    BlockDeviceInfo info{};
    std::memcpy(&info, reply, sizeof(info));
    EXPECT_EQ(info.block_count, sp.block_count);
    EXPECT_EQ(info.block_size, bs);
    EXPECT_EQ(info.max_transfer_blocks, max_b);
    EXPECT_EQ(info.nsid, sp.nsid);

    uint8_t wbuf_full[sizeof(BlockWriteRequest) + 4096] = {};
    const BlockWriteRequest wreq_full{
        .op = BlockRpcOp::kWrite,
        .disk_index = disk_idx,
        .lba = 4,
        .block_count = max_b,
        .reserved = 0,
    };
    std::memcpy(wbuf_full, &wreq_full, sizeof(wreq_full));
    for (int i = 0; i < 4096; ++i) {
      wbuf_full[hdr_bytes + i] =
          static_cast<uint8_t>(0x30 + d * 0x20 + (i & 0x1F));
    }
    const int64_t full_req_bytes = static_cast<int64_t>(hdr_bytes) + 4096;
    EXPECT_EQ(
        server.Dispatch(wbuf_full, full_req_bytes, 0, nullptr, &reply_size),
        max_b);
    EXPECT_EQ(reply_size, 0);

    const BlockReadRequest rreq_full{
        .op = BlockRpcOp::kRead,
        .disk_index = disk_idx,
        .lba = 4,
        .block_count = max_b,
        .reserved = 0,
    };
    std::memset(reply, 0, sizeof(reply));
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&rreq_full),  //
                              sizeof(rreq_full),                             //
                              4096,                                          //
                              reply,                                         //
                              &reply_size),
              max_b);
    EXPECT_EQ(reply_size, 4096);
    EXPECT_EQ(std::memcmp(reply, wbuf_full + hdr_bytes, 4096), 0);

    std::vector<uint8_t> wbuf_one(hdr_bytes + bs, uint8_t{0});
    const BlockWriteRequest wreq_one{
        .op = BlockRpcOp::kWrite,
        .disk_index = disk_idx,
        .lba = 1,
        .block_count = 1,
        .reserved = 0,
    };
    std::memcpy(wbuf_one.data(), &wreq_one, sizeof(wreq_one));
    for (int i = 0; i < bs; ++i) {
      wbuf_one[hdr_bytes + i] = static_cast<uint8_t>(0x80 ^ ((i + d) & 0x3F));
    }
    const int64_t req_bytes = static_cast<int64_t>(hdr_bytes) + bs;
    EXPECT_EQ(
        server.Dispatch(wbuf_one.data(), req_bytes, 0, nullptr, &reply_size),
        1);
    EXPECT_EQ(reply_size, 0);

    const BlockReadRequest rreq_one{
        .op = BlockRpcOp::kRead,
        .disk_index = disk_idx,
        .lba = 1,
        .block_count = 1,
        .reserved = 0,
    };
    std::memset(reply, 0, sizeof(reply));
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&rreq_one),  //
                              sizeof(rreq_one),                             //
                              bs,                                           //
                              reply,                                        //
                              &reply_size),
              1);
    EXPECT_EQ(reply_size, bs);
    EXPECT_EQ(std::memcmp(reply, wbuf_one.data() + hdr_bytes, bs), 0);

    uint8_t wbuf_bad512[sizeof(BlockWriteRequest) + 512] = {};
    std::memcpy(wbuf_bad512, &wreq_one, sizeof(wreq_one));
    EXPECT_EQ(server.Dispatch(wbuf_bad512,          //
                              sizeof(wbuf_bad512),  //
                              0,                    //
                              nullptr,              //
                              &reply_size),
              kSyscallErrorInvalid);

    const int oob_blocks = max_b + 1;
    std::vector<uint8_t> wbuf_oob(hdr_bytes + oob_blocks * bs, uint8_t{0});
    const BlockWriteRequest wreq_oob{
        .op = BlockRpcOp::kWrite,
        .disk_index = disk_idx,
        .lba = 0,
        .block_count = oob_blocks,
        .reserved = 0,
    };
    std::memcpy(wbuf_oob.data(), &wreq_oob, sizeof(wreq_oob));
    const int64_t oob_wbytes =
        static_cast<int64_t>(hdr_bytes) + static_cast<int64_t>(oob_blocks) * bs;
    EXPECT_EQ(
        server.Dispatch(wbuf_oob.data(), oob_wbytes, 0, nullptr, &reply_size),
        kSyscallErrorInvalid);

    const BlockReadRequest rreq_oob{
        .op = BlockRpcOp::kRead,
        .disk_index = disk_idx,
        .lba = 0,
        .block_count = oob_blocks,
        .reserved = 0,
    };
    std::vector<uint8_t> big_reply(oob_blocks * bs, uint8_t{0});
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&rreq_oob),  //
                              sizeof(rreq_oob),                             //
                              oob_blocks * bs,                              //
                              big_reply.data(),                             //
                              &reply_size),
              kSyscallErrorInvalid);

    std::vector<uint8_t> short_reply(bs, uint8_t{0xA5});
    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&rreq_one),  //
                              sizeof(rreq_one),                             //
                              bs - 1,                                       //
                              short_reply.data(),                           //
                              &reply_size),
              kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);
    EXPECT_THAT(short_reply, t::Each(t::Eq(0xA5)));

    EXPECT_EQ(server.Dispatch(reinterpret_cast<const uint8_t*>(&rreq_full),  //
                              sizeof(rreq_full),                             //
                              4095,                                          //
                              reply,                                         //
                              &reply_size),
              kSyscallErrorInvalid);
    EXPECT_EQ(reply_size, 0);
  }
}

TEST(BlockServerTest, ServeOneHandlesDisconnectAndCalls) {
  sim::SimNvmeDevice dev;
  NvmeController ctrl;
  ctrl.Init(dev, dev);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);

  BlockServer server;
  ASSERT_TRUE(server.AddController(&ctrl));

  struct FakeIpc {
    int stage = 0;
    int recv_calls = 0;
    int reply_calls = 0;
    int64_t last_recv_port = -1;
    int64_t last_timeout_ticks = 0;
    int64_t last_reply_port = -1;
    uint64_t last_reply_token = 0;
    int64_t last_status = -999;
    int64_t last_size = -1;
    const void* last_reply_buf = nullptr;
    BlockDeviceInfo last_info{};
  } ipc;

  const ServeEnv env{
      .ctx = &ipc,
      .recv = [](void* const ctx,                //
                 const int64_t port,             //
                 void* const buf,                //
                 const int64_t /*cap*/,          //
                 IpcReceivedMessage* const msg,  //
                 const int64_t timeout_ticks) -> int64_t {
        FakeIpc* const f = static_cast<FakeIpc*>(ctx);
        ++f->recv_calls;
        f->last_recv_port = port;
        f->last_timeout_ticks = timeout_ticks;
        if (f->stage == 0) {
          ++f->stage;
          msg->kind = IpcMessageKind::kClientDisconnect;
          msg->client_id = 42;
          return 1;
        }
        if (f->stage == 1) {
          ++f->stage;
          const BlockGetInfoRequest req{.op = BlockRpcOp::kGetInfo,
                                        .disk_index = 0};
          std::memcpy(buf, &req, sizeof(req));
          msg->kind = IpcMessageKind::kCall;
          msg->token = 99;
          msg->size = sizeof(req);
          msg->reply_cap = sizeof(BlockDeviceInfo);
          return 1;
        }
        return kSyscallErrorInvalid;
      },
      .reply = [](void* const ctx,        //
                  const int64_t port,     //
                  const uint64_t token,   //
                  const void* const buf,  //
                  const int64_t size,     //
                  const int64_t status) -> int64_t {
        FakeIpc* const f = static_cast<FakeIpc*>(ctx);
        ++f->reply_calls;
        f->last_reply_port = port;
        f->last_reply_token = token;
        f->last_reply_buf = buf;
        f->last_size = size;
        f->last_status = status;
        if (buf != nullptr &&
            size == static_cast<int64_t>(sizeof(BlockDeviceInfo))) {
          std::memcpy(&f->last_info, buf, sizeof(f->last_info));
        }
        return 0;
      },
  };

  uint8_t req_buf[kIpcMaxMessageSize] = {};
  uint8_t reply_buf[kIpcMaxMessageSize] = {};
  EXPECT_TRUE(ServeOne(server,           //
                       env,              //
                       req_buf,          //
                       sizeof(req_buf),  //
                       reply_buf,        //
                       sizeof(reply_buf)));
  EXPECT_EQ(ipc.recv_calls, 1);
  EXPECT_EQ(ipc.last_recv_port, kIpcPortBlock);
  EXPECT_EQ(ipc.last_timeout_ticks, -1);
  EXPECT_EQ(ipc.reply_calls, 0);

  EXPECT_TRUE(ServeOne(server,           //
                       env,              //
                       req_buf,          //
                       sizeof(req_buf),  //
                       reply_buf,        //
                       sizeof(reply_buf)));
  EXPECT_EQ(ipc.recv_calls, 2);
  EXPECT_EQ(ipc.last_recv_port, kIpcPortBlock);
  EXPECT_EQ(ipc.last_timeout_ticks, -1);
  EXPECT_EQ(ipc.reply_calls, 1);
  EXPECT_EQ(ipc.last_reply_port, kIpcPortBlock);
  EXPECT_EQ(ipc.last_reply_token, 99u);
  EXPECT_EQ(ipc.last_status, 1);
  EXPECT_EQ(ipc.last_size, static_cast<int64_t>(sizeof(BlockDeviceInfo)));
  EXPECT_EQ(ipc.last_reply_buf, reply_buf);
  EXPECT_EQ(ipc.last_info.block_count, uint64_t{128});
  EXPECT_EQ(ipc.last_info.block_size, 512);
  EXPECT_EQ(ipc.last_info.nsid, 1u);

  EXPECT_FALSE(ServeOne(server,           //
                        env,              //
                        req_buf,          //
                        sizeof(req_buf),  //
                        reply_buf,        //
                        sizeof(reply_buf)));
  EXPECT_EQ(ipc.recv_calls, 3);
  EXPECT_EQ(ipc.last_recv_port, kIpcPortBlock);
  EXPECT_EQ(ipc.last_timeout_ticks, -1);
}

TEST(BlockServerTest, ServeOneClampsNegativeAndOversizedReplyCapAndBadSizes) {
  sim::SimNvmeDevice dev;
  NvmeController ctrl;
  ctrl.Init(dev, dev);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);

  BlockServer server;
  ASSERT_TRUE(server.AddController(&ctrl));

  struct FakeIpc {
    int step = 0;
    int64_t last_recv_port = -1;
    int64_t last_recv_cap = -1;
    int64_t last_timeout_ticks = 0;
    int64_t last_reply_port = -1;
    uint64_t last_reply_token = 0;
    int64_t last_status = -999;
    int64_t last_size = -1;
    const void* last_reply_buf = nullptr;
  } ipc;

  const ServeEnv env{
      .ctx = &ipc,
      .recv = [](void* const ctx,                //
                 const int64_t port,             //
                 void* const buf,                //
                 const int64_t cap,              //
                 IpcReceivedMessage* const msg,  //
                 const int64_t timeout_ticks) -> int64_t {
        FakeIpc* const f = static_cast<FakeIpc*>(ctx);
        f->last_recv_port = port;
        f->last_recv_cap = cap;
        f->last_timeout_ticks = timeout_ticks;
        msg->kind = IpcMessageKind::kCall;
        msg->token = 100 + static_cast<uint64_t>(f->step);
        if (f->step == 0) {
          const BlockFlushRequest freq{.op = BlockRpcOp::kFlush,
                                       .disk_index = 0};
          std::memcpy(buf, &freq, sizeof(freq));
          msg->size = sizeof(freq);
          msg->reply_cap = -5;
        } else if (f->step == 1) {
          const BlockGetInfoRequest greq{.op = BlockRpcOp::kGetInfo,
                                         .disk_index = 0};
          std::memcpy(buf, &greq, sizeof(greq));
          msg->size = sizeof(greq);
          msg->reply_cap = -5;
        } else if (f->step == 2) {
          const BlockGetInfoRequest greq{.op = BlockRpcOp::kGetInfo,
                                         .disk_index = 0};
          std::memcpy(buf, &greq, sizeof(greq));
          msg->size = sizeof(greq);
          msg->reply_cap = 4096;
        } else if (f->step == 3) {
          msg->size = kIpcMaxMessageSize + 100;
          msg->reply_cap = 64;
        } else if (f->step == 4) {
          const BlockGetInfoRequest greq{.op = BlockRpcOp::kGetInfo,
                                         .disk_index = 0};
          std::memcpy(buf, &greq, sizeof(greq));
          msg->size = -1;
          msg->reply_cap = sizeof(BlockDeviceInfo);
        } else if (f->step == 5) {
          const BlockGetInfoRequest greq{.op = BlockRpcOp::kGetInfo,
                                         .disk_index = 0};
          std::memcpy(buf, &greq, sizeof(greq));
          msg->size = 0;
          msg->reply_cap = sizeof(BlockDeviceInfo);
        } else if (f->step == 6) {
          const BlockReadRequest rreq{
              .op = BlockRpcOp::kRead,
              .disk_index = 0,
              .lba = 0,
              .block_count = 8,
              .reserved = 0,
          };
          std::memcpy(buf, &rreq, sizeof(rreq));
          msg->size = sizeof(rreq);
          msg->reply_cap = kIpcMaxMessageSize + 100;
        } else if (f->step == 7) {
          const BlockGetInfoRequest greq{.op = BlockRpcOp::kGetInfo,
                                         .disk_index = 0};
          std::memcpy(buf, &greq, sizeof(greq));
          msg->size = sizeof(greq);
          msg->reply_cap = sizeof(BlockDeviceInfo);
        } else if (f->step == 8) {
          const BlockGetInfoRequest greq{.op = BlockRpcOp::kGetInfo,
                                         .disk_index = 0};
          std::memcpy(buf, &greq, sizeof(greq));
          msg->size = sizeof(greq);
          msg->reply_cap = INT32_MAX;
        } else if (f->step == 9) {
          const BlockGetInfoRequest greq{.op = BlockRpcOp::kGetInfo,
                                         .disk_index = 0};
          std::memcpy(buf, &greq, sizeof(greq));
          msg->size = sizeof(greq);
          msg->reply_cap = 32;
        } else if (f->step == 10) {
          const BlockWriteRequest wreq{
              .op = BlockRpcOp::kWrite,
              .disk_index = 0,
              .lba = 0,
              .block_count = 1,
              .reserved = 0,
          };
          std::memcpy(buf, &wreq, sizeof(wreq));
          msg->size = static_cast<int64_t>(sizeof(wreq)) + 512;
          msg->reply_cap = 0;
        } else if (f->step == 11) {
          const BlockGetInfoRequest greq{.op = BlockRpcOp::kGetInfo,
                                         .disk_index = 0};
          std::memcpy(buf, &greq, sizeof(greq));
          msg->size = sizeof(greq);
          msg->reply_cap = static_cast<int32_t>(sizeof(BlockDeviceInfo)) - 1;
        } else if (f->step == 12) {
          const BlockReadRequest rreq{
              .op = BlockRpcOp::kRead,
              .disk_index = 0,
              .lba = 0,
              .block_count = 1,
              .reserved = 0,
          };
          std::memcpy(buf, &rreq, sizeof(rreq));
          msg->size = sizeof(rreq);
          msg->reply_cap = 511;
        } else if (f->step == 13 || f->step == 14) {
          const BlockReadRequest rreq{
              .op = BlockRpcOp::kRead,
              .disk_index = 0,
              .lba = 0,
              .block_count = 1,
              .reserved = 0,
          };
          const int64_t rreq_bytes = static_cast<int64_t>(sizeof(rreq));
          const int64_t copy_bytes = (cap < rreq_bytes) ? cap : rreq_bytes;
          std::memcpy(buf, &rreq, copy_bytes);
          msg->size = rreq_bytes;
          msg->reply_cap = 512;
        } else {
          const BlockWriteRequest wreq{
              .op = BlockRpcOp::kWrite,
              .disk_index = 0,
              .lba = 0,
              .block_count = 1,
              .reserved = 0,
          };
          const int64_t hdr_bytes = static_cast<int64_t>(sizeof(wreq));
          const int64_t copy_bytes = (cap < hdr_bytes) ? cap : hdr_bytes;
          std::memcpy(buf, &wreq, copy_bytes);
          msg->size = hdr_bytes + 512;
          msg->reply_cap = 0;
        }
        ++f->step;
        return 1;
      },
      .reply = [](void* const ctx,        //
                  const int64_t port,     //
                  const uint64_t token,   //
                  const void* const buf,  //
                  const int64_t size,     //
                  const int64_t status) -> int64_t {
        FakeIpc* const f = static_cast<FakeIpc*>(ctx);
        f->last_reply_port = port;
        f->last_reply_token = token;
        f->last_reply_buf = buf;
        f->last_size = size;
        f->last_status = status;
        return 0;
      },
  };

  uint8_t req_buf[kIpcMaxMessageSize] = {};
  uint8_t reply_buf[kIpcMaxMessageSize] = {};
  // Step 0: negative msg.reply_cap clamped to 0 lets kFlush succeed.
  EXPECT_TRUE(ServeOne(server,           //
                       env,              //
                       req_buf,          //
                       sizeof(req_buf),  //
                       reply_buf,        //
                       sizeof(reply_buf)));
  EXPECT_EQ(ipc.last_status, 0);
  EXPECT_EQ(ipc.last_size, 0);

  // Step 1: negative msg.reply_cap clamped to 0 fails kGetInfo cleanly.
  EXPECT_TRUE(ServeOne(server,           //
                       env,              //
                       req_buf,          //
                       sizeof(req_buf),  //
                       reply_buf,        //
                       sizeof(reply_buf)));
  EXPECT_EQ(ipc.last_status, kSyscallErrorInvalid);
  EXPECT_EQ(ipc.last_size, 0);

  // Step 2: msg.reply_cap (4096) > server reply_cap (16) clamped to 16.
  EXPECT_TRUE(ServeOne(server, env, req_buf, sizeof(req_buf), reply_buf, 16));
  EXPECT_EQ(ipc.last_status, kSyscallErrorInvalid);
  EXPECT_EQ(ipc.last_size, 0);

  // Step 3: oversized msg.size (> req_cap) rejected without Dispatch OOB read.
  EXPECT_TRUE(ServeOne(server,           //
                       env,              //
                       req_buf,          //
                       sizeof(req_buf),  //
                       reply_buf,        //
                       sizeof(reply_buf)));
  EXPECT_EQ(ipc.last_status, kSyscallErrorInvalid);
  EXPECT_EQ(ipc.last_size, 0);

  // Step 4: negative msg.size rejected cleanly before Dispatch.
  EXPECT_TRUE(ServeOne(server,           //
                       env,              //
                       req_buf,          //
                       sizeof(req_buf),  //
                       reply_buf,        //
                       sizeof(reply_buf)));
  EXPECT_EQ(ipc.last_status, kSyscallErrorInvalid);
  EXPECT_EQ(ipc.last_size, 0);

  // Step 5: zero msg.size rejected cleanly before Dispatch.
  EXPECT_TRUE(ServeOne(server,           //
                       env,              //
                       req_buf,          //
                       sizeof(req_buf),  //
                       reply_buf,        //
                       sizeof(reply_buf)));
  EXPECT_EQ(ipc.last_status, kSyscallErrorInvalid);
  EXPECT_EQ(ipc.last_size, 0);

  // Step 6: oversized msg.reply_cap & caller reply_cap clamped to
  // kIpcMaxMessageSize (nvme_server.h:506).
  uint8_t big_reply[kIpcMaxMessageSize + 100] = {};
  EXPECT_TRUE(ServeOne(server,           //
                       env,              //
                       req_buf,          //
                       sizeof(req_buf),  //
                       big_reply,        //
                       sizeof(big_reply)));
  EXPECT_EQ(ipc.last_status, 8);
  EXPECT_EQ(ipc.last_size, 4096);
  EXPECT_EQ(ipc.last_reply_buf, big_reply);

  // Step 7: exact msg.size == req_cap boundary passes ServeOne cleanly.
  EXPECT_TRUE(ServeOne(server,                       //
                       env,                          //
                       req_buf,                      //
                       sizeof(BlockGetInfoRequest),  //
                       reply_buf,                    //
                       sizeof(reply_buf)));
  EXPECT_EQ(ipc.last_recv_port, kIpcPortBlock);
  EXPECT_EQ(ipc.last_recv_cap,
            static_cast<int64_t>(sizeof(BlockGetInfoRequest)));
  EXPECT_EQ(ipc.last_timeout_ticks, -1);
  EXPECT_EQ(ipc.last_reply_port, kIpcPortBlock);
  EXPECT_EQ(ipc.last_reply_token, 107u);
  EXPECT_EQ(ipc.last_status, 1);
  EXPECT_EQ(ipc.last_size, static_cast<int64_t>(sizeof(BlockDeviceInfo)));
  EXPECT_EQ(ipc.last_reply_buf, reply_buf);

  // Step 8: msg.reply_cap == INT32_MAX paired with 64-bit reply_cap >= 1LL<<32
  // avoids truncating reply_cap to int32_t before min().
  const int64_t huge_cap = static_cast<int64_t>(uint64_t{1} << 32);
  const int64_t req_cap = sizeof(req_buf);
  EXPECT_TRUE(ServeOne(server, env, req_buf, req_cap, reply_buf, huge_cap));
  EXPECT_EQ(ipc.last_status, 1);
  EXPECT_EQ(ipc.last_size, static_cast<int64_t>(sizeof(BlockDeviceInfo)));
  EXPECT_EQ(ipc.last_reply_buf, reply_buf);

  // Step 9 (C_S501_POS_SMALL): positive msg.reply_cap (32) below
  // sizeof(BlockDeviceInfo) (88) and below caller reply_cap (4352) is honored.
  EXPECT_TRUE(ServeOne(server,           //
                       env,              //
                       req_buf,          //
                       sizeof(req_buf),  //
                       reply_buf,        //
                       sizeof(reply_buf)));
  EXPECT_EQ(ipc.last_status, kSyscallErrorInvalid);
  EXPECT_EQ(ipc.last_size, 0);

  // Step 10 (C_S510_MSG_SIZE / C_S510_MAX_MSG): msg.size > req_cap with
  // req_cap < kIpcMaxMessageSize is rejected without Dispatch OOB read.
  uint8_t small_req[sizeof(BlockWriteRequest)] = {};
  const int io_before = dev.io_commands_seen;
  EXPECT_TRUE(ServeOne(server,             //
                       env,                //
                       small_req,          //
                       sizeof(small_req),  //
                       reply_buf,          //
                       sizeof(reply_buf)));
  EXPECT_EQ(ipc.last_status, kSyscallErrorInvalid);
  EXPECT_EQ(ipc.last_size, 0);
  EXPECT_EQ(dev.io_commands_seen, io_before);

  // Step 11 (N73): msg.reply_cap = 87 (< sizeof(BlockDeviceInfo)) clamped.
  EXPECT_TRUE(ServeOne(server,           //
                       env,              //
                       req_buf,          //
                       sizeof(req_buf),  //
                       reply_buf,        //
                       sizeof(reply_buf)));
  EXPECT_EQ(ipc.last_status, kSyscallErrorInvalid);
  EXPECT_EQ(ipc.last_size, 0);

  // Step 12 (N73): msg.reply_cap = 511 (< 1-block kRead) clamped.
  EXPECT_TRUE(ServeOne(server,           //
                       env,              //
                       req_buf,          //
                       sizeof(req_buf),  //
                       reply_buf,        //
                       sizeof(reply_buf)));
  EXPECT_EQ(ipc.last_status, kSyscallErrorInvalid);
  EXPECT_EQ(ipc.last_size, 0);
  EXPECT_EQ(dev.io_commands_seen, io_before);

  // Step 13 (N72 ASan): 23-byte heap buffer with msg.size = 24 rejected.
  std::vector<uint8_t> req_23(sizeof(BlockReadRequest) - 1, uint8_t{0});
  EXPECT_TRUE(ServeOne(server,                               //
                       env,                                  //
                       req_23.data(),                        //
                       static_cast<int64_t>(req_23.size()),  //
                       reply_buf,                            //
                       sizeof(reply_buf)));
  EXPECT_EQ(ipc.last_status, kSyscallErrorInvalid);
  EXPECT_EQ(ipc.last_size, 0);
  EXPECT_EQ(dev.io_commands_seen, io_before);

  // Step 14 (N72 -O2 nosan): zeroed 24-B kRead passed with req_cap = 23.
  const int64_t reply_cap = sizeof(reply_buf);
  uint8_t req_24[sizeof(BlockReadRequest)] = {};
  const int64_t rcap_23 = static_cast<int64_t>(sizeof(req_24)) - 1;
  EXPECT_TRUE(ServeOne(server, env, req_24, rcap_23, reply_buf, reply_cap));
  EXPECT_EQ(ipc.last_status, kSyscallErrorInvalid);
  EXPECT_EQ(ipc.last_size, 0);
  EXPECT_EQ(dev.io_commands_seen, io_before);

  // Step 15 (N72 -O2 nosan): zeroed 536-B kWrite passed with req_cap = 535.
  uint8_t req_536[sizeof(BlockWriteRequest) + 512] = {};
  const int64_t wcap_535 = static_cast<int64_t>(sizeof(req_536)) - 1;
  EXPECT_TRUE(ServeOne(server, env, req_536, wcap_535, reply_buf, reply_cap));
  EXPECT_EQ(ipc.last_status, kSyscallErrorInvalid);
  EXPECT_EQ(ipc.last_size, 0);
  EXPECT_EQ(dev.io_commands_seen, io_before);
}

TEST(BlockServerTest, ServeOneHandlesInvalidEnvAndReplyFailureFallback) {
  sim::SimNvmeDevice dev;
  NvmeController ctrl;
  ctrl.Init(dev, dev);
  ASSERT_EQ(ctrl.TakeOwnershipAndReset(), NvmeStatus::kOk);
  ASSERT_EQ(ctrl.Start(1), NvmeStatus::kOk);

  BlockServer server;
  ASSERT_TRUE(server.AddController(&ctrl));

  uint8_t req_buf[kIpcMaxMessageSize] = {};
  uint8_t reply_buf[kIpcMaxMessageSize] = {};
  const ServeEnv empty_env{};
  EXPECT_FALSE(ServeOne(server,           //
                        empty_env,        //
                        req_buf,          //
                        sizeof(req_buf),  //
                        reply_buf,        //
                        sizeof(reply_buf)));

  struct FakeIpc {
    int stage = 0;
    int reply_calls = 0;
    int64_t recv_port = -1;
    int64_t recv_timeout = 0;
    int64_t first_port = -1;
    uint64_t first_token = 0;
    int64_t second_port = -1;
    uint64_t second_token = 0;
    int64_t second_status = 0;
    int64_t second_size = -1;
    const void* second_buf = nullptr;
  } ipc;
  ipc.second_buf = req_buf;

  const ServeEnv env{
      .ctx = &ipc,
      .recv = [](void* const ctx,                //
                 const int64_t port,             //
                 void* const buf,                //
                 const int64_t /*cap*/,          //
                 IpcReceivedMessage* const msg,  //
                 const int64_t timeout_ticks) -> int64_t {
        FakeIpc* const f = static_cast<FakeIpc*>(ctx);
        f->recv_port = port;
        f->recv_timeout = timeout_ticks;
        if (f->stage == 0) {
          ++f->stage;
          const BlockGetInfoRequest req{.op = BlockRpcOp::kGetInfo,
                                        .disk_index = 0};
          std::memcpy(buf, &req, sizeof(req));
          msg->kind = IpcMessageKind::kCall;
          msg->token = 77;
          msg->size = sizeof(req);
          msg->reply_cap = sizeof(BlockDeviceInfo);
          return 1;
        }
        msg->kind = IpcMessageKind::kNone;
        return 1;
      },
      .reply = [](void* const ctx,        //
                  const int64_t port,     //
                  const uint64_t token,   //
                  const void* const buf,  //
                  const int64_t size,     //
                  const int64_t status) -> int64_t {
        FakeIpc* const f = static_cast<FakeIpc*>(ctx);
        ++f->reply_calls;
        if (f->reply_calls == 1) {
          f->first_port = port;
          f->first_token = token;
          return kSyscallErrorInvalid;
        }
        f->second_port = port;
        f->second_token = token;
        f->second_buf = buf;
        f->second_size = size;
        f->second_status = status;
        return 0;
      },
  };

  ServeEnv no_recv = env;
  no_recv.recv = nullptr;
  EXPECT_FALSE(ServeOne(server,           //
                        no_recv,          //
                        req_buf,          //
                        sizeof(req_buf),  //
                        reply_buf,        //
                        sizeof(reply_buf)));

  ServeEnv no_reply = env;
  no_reply.reply = nullptr;
  EXPECT_FALSE(ServeOne(server,           //
                        no_reply,         //
                        req_buf,          //
                        sizeof(req_buf),  //
                        reply_buf,        //
                        sizeof(reply_buf)));
  EXPECT_FALSE(ServeOne(server,           //
                        env,              //
                        nullptr,          //
                        sizeof(req_buf),  //
                        reply_buf,        //
                        sizeof(reply_buf)));
  EXPECT_FALSE(ServeOne(server, env, req_buf, 0, reply_buf, sizeof(reply_buf)));
  EXPECT_FALSE(
      ServeOne(server, env, req_buf, -1, reply_buf, sizeof(reply_buf)));
  EXPECT_FALSE(ServeOne(server,           //
                        env,              //
                        req_buf,          //
                        sizeof(req_buf),  //
                        nullptr,          //
                        sizeof(reply_buf)));
  EXPECT_FALSE(ServeOne(server, env, req_buf, sizeof(req_buf), reply_buf, 0));
  EXPECT_FALSE(ServeOne(server, env, req_buf, sizeof(req_buf), reply_buf, -1));

  EXPECT_TRUE(ServeOne(server,           //
                       env,              //
                       req_buf,          //
                       sizeof(req_buf),  //
                       reply_buf,        //
                       sizeof(reply_buf)));
  EXPECT_EQ(ipc.recv_port, kIpcPortBlock);
  EXPECT_EQ(ipc.recv_timeout, -1);
  EXPECT_EQ(ipc.reply_calls, 2);
  EXPECT_EQ(ipc.first_port, kIpcPortBlock);
  EXPECT_EQ(ipc.first_token, 77u);
  EXPECT_EQ(ipc.second_port, kIpcPortBlock);
  EXPECT_EQ(ipc.second_token, 77u);
  EXPECT_EQ(ipc.second_buf, nullptr);
  EXPECT_EQ(ipc.second_size, 0);
  EXPECT_EQ(ipc.second_status, kSyscallErrorIo);

  EXPECT_FALSE(ServeOne(server,           //
                        env,              //
                        req_buf,          //
                        sizeof(req_buf),  //
                        reply_buf,        //
                        sizeof(reply_buf)));

  // N74: env.reply returning > 0 (+1) is != 0 and triggers fallback reply.
  {
    struct PosReplyState {
      int reply_calls = 0;
      int64_t first_port = -1;
      uint64_t first_token = 0;
      int64_t second_port = -1;
      uint64_t second_token = 0;
      int64_t second_status = 0;
      int64_t second_size = -1;
      const void* second_buf = nullptr;
    } pos_ipc;
    pos_ipc.second_buf = req_buf;

    const ServeEnv pos_env{
        .ctx = &pos_ipc,
        .recv = [](void* const /*ctx*/,            //
                   const int64_t /*port*/,         //
                   void* const buf,                //
                   const int64_t /*cap*/,          //
                   IpcReceivedMessage* const msg,  //
                   const int64_t /*timeout*/) -> int64_t {
          const BlockGetInfoRequest req{.op = BlockRpcOp::kGetInfo,
                                        .disk_index = 0};
          std::memcpy(buf, &req, sizeof(req));
          msg->kind = IpcMessageKind::kCall;
          msg->token = 78;
          msg->size = sizeof(req);
          msg->reply_cap = sizeof(BlockDeviceInfo);
          return 1;
        },
        .reply = [](void* const ctx,        //
                    const int64_t port,     //
                    const uint64_t token,   //
                    const void* const buf,  //
                    const int64_t size,     //
                    const int64_t status) -> int64_t {
          PosReplyState* const s = static_cast<PosReplyState*>(ctx);
          ++s->reply_calls;
          if (s->reply_calls == 1) {
            s->first_port = port;
            s->first_token = token;
            return 1;
          }
          s->second_port = port;
          s->second_token = token;
          s->second_buf = buf;
          s->second_size = size;
          s->second_status = status;
          return 0;
        },
    };

    EXPECT_TRUE(ServeOne(server,           //
                         pos_env,          //
                         req_buf,          //
                         sizeof(req_buf),  //
                         reply_buf,        //
                         sizeof(reply_buf)));
    EXPECT_EQ(pos_ipc.reply_calls, 2);
    EXPECT_EQ(pos_ipc.first_port, kIpcPortBlock);
    EXPECT_EQ(pos_ipc.first_token, 78u);
    EXPECT_EQ(pos_ipc.second_port, kIpcPortBlock);
    EXPECT_EQ(pos_ipc.second_token, 78u);
    EXPECT_EQ(pos_ipc.second_buf, nullptr);
    EXPECT_EQ(pos_ipc.second_size, 0);
    EXPECT_EQ(pos_ipc.second_status, kSyscallErrorIo);
  }

  // ADV_M2_R3_16b: env.recv returning 0 or 2 must return false without
  // invoking Dispatch() or env.reply(), even when msg->kind == kCall.
  for (const int64_t non_one_rc : {int64_t{0}, int64_t{2}}) {
    struct NonOneRecvState {
      int64_t ret_rc = 0;
      int reply_calls = 0;
    } state{non_one_rc, 0};
    const ServeEnv non_one_env{
        .ctx = &state,
        .recv = [](void* const ctx,                //
                   const int64_t /*port*/,         //
                   void* const buf,                //
                   const int64_t /*cap*/,          //
                   IpcReceivedMessage* const msg,  //
                   const int64_t /*timeout*/) -> int64_t {
          const NonOneRecvState* const s =
              static_cast<const NonOneRecvState*>(ctx);
          const BlockGetInfoRequest req{.op = BlockRpcOp::kGetInfo,
                                        .disk_index = 0};
          std::memcpy(buf, &req, sizeof(req));
          msg->kind = IpcMessageKind::kCall;
          msg->token = 88;
          msg->size = sizeof(req);
          msg->reply_cap = sizeof(BlockDeviceInfo);
          return s->ret_rc;
        },
        .reply = [](void* const ctx,            //
                    const int64_t /*port*/,     //
                    const uint64_t /*token*/,   //
                    const void* const /*buf*/,  //
                    const int64_t /*size*/,     //
                    const int64_t /*status*/) -> int64_t {
          NonOneRecvState* const s = static_cast<NonOneRecvState*>(ctx);
          ++s->reply_calls;
          return 0;
        },
    };
    EXPECT_FALSE(ServeOne(server,           //
                          non_one_env,      //
                          req_buf,          //
                          sizeof(req_buf),  //
                          reply_buf,        //
                          sizeof(reply_buf)));
    EXPECT_EQ(state.reply_calls, 0);
  }
}

}  // namespace
}  // namespace protos::nvme
