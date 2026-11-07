#include "multiboot.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sys/mman.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "paging.h"
#include "uart.h"
#include "vga.h"

namespace protos {
namespace {

namespace t = ::testing;

// Memory types that multiboot.h does not name.
constexpr uint32_t kMemoryTypeReserved = 2;
constexpr uint32_t kMemoryTypeAcpiReclaimable = 3;
constexpr uint32_t kMemoryTypeAcpiNvs = 4;
constexpr uint32_t kMemoryTypeDefective = 5;
constexpr int64_t kPageBytes = 4096;

struct MapCall {
  uintptr_t phys_addr;
  int64_t size;

  bool operator==(const MapCall&) const = default;
};

struct FakeEnvState {
  bool fail_bootstrap_map = false;
  // PagingMapBootstrapRange() also fails for ranges that start here, if not 0.
  uintptr_t fail_bootstrap_map_addr = 0;
  // 0-based index of the RecordingMapper() call that fails, or -1.
  int fail_map_call = -1;
  std::vector<MapCall> map_calls;
  std::string uart_log;
  std::string vga_log;
};

FakeEnvState g_env;

static void ResetFakeEnv() { g_env = FakeEnvState{}; }

// MultibootPhysicalMapper that treats host addresses as physical addresses,
// records its calls in `g_env.map_calls` and fails call `g_env.fail_map_call`.
static const void* RecordingMapper(const uintptr_t phys_addr,
                                   const int64_t size) {
  const int call = static_cast<int>(g_env.map_calls.size());
  g_env.map_calls.push_back({phys_addr, size});
  if (call == g_env.fail_map_call) {
    return nullptr;
  }
  return reinterpret_cast<const void*>(phys_addr);
}

}  // namespace

bool PagingMapBootstrapRange(const uintptr_t phys_addr, const int64_t size) {
  g_env.map_calls.push_back({phys_addr, size});
  return !g_env.fail_bootstrap_map &&
         (g_env.fail_bootstrap_map_addr == 0 ||
          phys_addr != g_env.fail_bootstrap_map_addr);
}

// console.cpp prints through these.
void UartWrite(const char* const str) { g_env.uart_log.append(str); }

void VgaWrite(const char* const str) { g_env.vga_log.append(str); }

namespace {

// Multiboot2 information structure with one memory map tag of `kEntries`
// entries followed by the end tag.
template <int kEntries>
struct [[gnu::packed]] Mb2MmapInfo {
  Multiboot2InfoHeader header;
  Multiboot2TagMmap mmap;
  Multiboot2MmapEntry entries[kEntries];
  Multiboot2Tag end;
};

template <int kEntries>
static void InitMb2MmapInfo(Mb2MmapInfo<kEntries>* const info) {
  std::memset(info, 0, sizeof(*info));
  info->header.total_size = sizeof(*info);
  info->mmap.type = kMultiboot2TagMmap;
  info->mmap.size =
      sizeof(Multiboot2TagMmap) + static_cast<uint32_t>(sizeof(info->entries));
  info->mmap.entry_size = sizeof(Multiboot2MmapEntry);
  info->end.type = kMultiboot2TagEnd;
  info->end.size = sizeof(Multiboot2Tag);
}

static uintptr_t AddrOf(const void* const ptr) {
  return reinterpret_cast<uintptr_t>(ptr);
}

// Anonymous mapping in the low 2 GiB (MAP_32BIT), for Multiboot1 structures
// whose 32-bit address fields must be able to point at host memory. The last
// page is left inaccessible to catch reads past the end of the usable pages.
class Low32BitBuffer {
 public:
  explicit Low32BitBuffer(const int64_t usable_pages)
      : bytes_((usable_pages + 1) * kPageBytes) {
    void* const mapped = mmap(nullptr,                                  //
                              bytes_,                                   //
                              PROT_READ | PROT_WRITE,                   //
                              MAP_PRIVATE | MAP_ANONYMOUS | MAP_32BIT,  //
                              -1,                                       //
                              0);
    if (mapped == MAP_FAILED) {
      return;
    }
    base_ = static_cast<uint8_t*>(mapped);
    uint8_t* const guard_page = base_ + usable_pages * kPageBytes;
    if (mprotect(guard_page, kPageBytes, PROT_NONE) != 0 ||
        AddrOf(base_) + bytes_ > UINT32_MAX) {
      munmap(base_, bytes_);
      base_ = nullptr;
    }
  }

  ~Low32BitBuffer() {
    if (base_ != nullptr) {
      munmap(base_, bytes_);
    }
  }

  Low32BitBuffer(const Low32BitBuffer&) = delete;
  Low32BitBuffer& operator=(const Low32BitBuffer&) = delete;

  bool ok() const { return base_ != nullptr; }

  // Returns the address `offset_from_end` bytes before the inaccessible page.
  uint8_t* EndMinus(const int64_t offset_from_end) const {
    return base_ + bytes_ - kPageBytes - offset_from_end;
  }

 private:
  int64_t bytes_;
  uint8_t* base_ = nullptr;
};

TEST(MultibootTest, RecordMmapEntryCountsOnlyRamTypesInByteTotals) {
  ResetFakeEnv();
  MultibootMemoryMap map = {};

  struct Entry {
    uint64_t base;
    uint64_t length;
    uint32_t type;
  };
  constexpr Entry kEntries[] = {
      {0x100000, 0x7F00000, kMemoryTypeAvailable},
      {0x8000000, 0x10000, kMemoryTypeAcpiReclaimable},
      {0x8010000, 0x1000, kMemoryTypeAcpiNvs},
      {0x8020000, 0x2000, kMemoryTypeDefective},
      // Reserved address space (MMIO windows, firmware ROM) and unknown types
      // are recorded as regions to carve out but are not RAM.
      {0xFD00000000, 3ULL << 32, kMemoryTypeReserved},
      {0xFEC00000, 0x1000, 12},
  };
  for (const Entry& entry : kEntries) {
    ASSERT_TRUE(
        MultibootRecordMmapEntry(&map, entry.base, entry.length, entry.type));
  }

  EXPECT_THAT(map.usable_ram_bytes, t::Eq(0x7F00000));
  EXPECT_THAT(map.reserved_ram_bytes, t::Eq(0x10000 + 0x1000 + 0x2000));
  EXPECT_THAT(map.total_ram_bytes,
              t::Eq(map.usable_ram_bytes + map.reserved_ram_bytes));
  EXPECT_THAT(map.region_count, t::Eq(5));
  EXPECT_THAT(g_env.uart_log, t::IsEmpty());
}

TEST(MultibootTest, RecordMmapEntrySaturatesByteTotalsWithoutSignedOverflow) {
  ResetFakeEnv();
  MultibootMemoryMap map = {};
  constexpr uint64_t kHugeLen = static_cast<uint64_t>(INT64_MAX) + 4096ULL;

  ASSERT_TRUE(
      MultibootRecordMmapEntry(&map, 0x100000, kHugeLen, kMemoryTypeAvailable));
  EXPECT_THAT(map.usable_ram_bytes, t::Eq(INT64_MAX));
  EXPECT_THAT(map.total_ram_bytes, t::Eq(INT64_MAX));

  ASSERT_TRUE(MultibootRecordMmapEntry(&map,      //
                                       0x200000,  //
                                       kHugeLen,  //
                                       kMemoryTypeAcpiReclaimable));
  EXPECT_THAT(map.reserved_ram_bytes, t::Eq(INT64_MAX));
  EXPECT_THAT(map.total_ram_bytes, t::Eq(INT64_MAX));

  MultibootMemoryMap mmio_map = {};
  ASSERT_TRUE(MultibootRecordMmapEntry(&mmio_map,  //
                                       0x200000,   //
                                       kHugeLen,   //
                                       kMemoryTypeReserved));
  EXPECT_THAT(mmio_map.total_ram_bytes, t::Eq(0));
  EXPECT_THAT(mmio_map.reserved_ram_bytes, t::Eq(0));
  EXPECT_THAT(mmio_map.region_count, t::Eq(1));
}

TEST(MultibootTest, RecordMmapEntryRejectsNullMapAndIgnoresEmptyEntries) {
  ResetFakeEnv();
  EXPECT_FALSE(MultibootRecordMmapEntry(nullptr,   //
                                        0x100000,  //
                                        0x1000,    //
                                        kMemoryTypeAvailable));

  MultibootMemoryMap map = {};
  EXPECT_TRUE(
      MultibootRecordMmapEntry(&map, 0x100000, 0, kMemoryTypeAvailable));
  EXPECT_THAT(map.region_count, t::Eq(0));
  EXPECT_THAT(map.total_ram_bytes, t::Eq(0));
}

TEST(MultibootTest, RecordMmapEntryBridgeEntriesMergeTransitively) {
  ResetFakeEnv();
  MultibootMemoryMap map = {};
  constexpr uintptr_t kBase = 0x200000;

  constexpr int kPairs = 260;
  for (int i = 0; i < kPairs; ++i) {
    const uintptr_t even_base =
        kBase + static_cast<uintptr_t>(2 * i) * kPageBytes;
    ASSERT_TRUE(MultibootRecordMmapEntry(&map,        //
                                         even_base,   //
                                         kPageBytes,  //
                                         kMemoryTypeAvailable));
  }
  EXPECT_THAT(map.region_count, t::Eq(kPairs));

  for (int i = 0; i < kPairs; ++i) {
    const uintptr_t odd_base =
        kBase + static_cast<uintptr_t>(2 * i + 1) * kPageBytes;
    ASSERT_TRUE(MultibootRecordMmapEntry(&map,        //
                                         odd_base,    //
                                         kPageBytes,  //
                                         kMemoryTypeAvailable));
  }
  EXPECT_THAT(map.region_count, t::Eq(1));
  EXPECT_THAT(map.regions[0].base, t::Eq(kBase));
  EXPECT_THAT(map.regions[0].length, t::Eq(2 * kPairs * kPageBytes));

  // Regions whose combined span would exceed INT64_MAX stay separate instead
  // of being merged into a truncated region.
  MultibootMemoryMap span_map = {};
  constexpr uint64_t kMaxSpan = static_cast<uint64_t>(INT64_MAX);
  ASSERT_TRUE(
      MultibootRecordMmapEntry(&span_map, 0, kMaxSpan, kMemoryTypeAvailable));
  ASSERT_TRUE(MultibootRecordMmapEntry(&span_map,   //
                                       kMaxSpan,    //
                                       kPageBytes,  //
                                       kMemoryTypeAvailable));
  EXPECT_THAT(span_map.region_count, t::Eq(2));
  EXPECT_THAT(span_map.regions[0].length, t::Eq(INT64_MAX));
  EXPECT_THAT(span_map.regions[1].length, t::Eq(kPageBytes));
}

TEST(MultibootTest, Multiboot2ParseLogsEntriesAndReservesInfoBlock) {
  ResetFakeEnv();
  Mb2MmapInfo<3> info;
  InitMb2MmapInfo(&info);
  info.entries[0] = {0x0, 0x9FC00, kMemoryTypeAvailable, 0};
  info.entries[1] = {0x100000, 0x7EE0000, kMemoryTypeAvailable, 0};
  info.entries[2] = {0x7FE0000, 0x20000, kMemoryTypeAcpiNvs, 0};

  MultibootMemoryMap parsed = {};
  ASSERT_TRUE(MultibootParseMemoryMap(kMultiboot2Magic,   //
                                      AddrOf(&info),      //
                                      kIdentityMapLimit,  //
                                      &parsed));
  ASSERT_THAT(parsed.region_count, t::Eq(3));
  EXPECT_THAT(parsed.regions[1].base, t::Eq(0x100000u));
  EXPECT_THAT(parsed.regions[1].length, t::Eq(0x7EE0000));
  EXPECT_THAT(parsed.regions[2].type, t::Eq(kMemoryTypeAcpiNvs));
  EXPECT_THAT(parsed.usable_ram_bytes, t::Eq(0x9FC00 + 0x7EE0000));
  EXPECT_THAT(parsed.reserved_ram_bytes, t::Eq(0x20000));
  EXPECT_THAT(parsed.mb_reserved_start, t::Eq(AddrOf(&info)));
  EXPECT_THAT(parsed.mb_reserved_end, t::Eq(AddrOf(&info) + sizeof(info)));
  EXPECT_THAT(parsed.mb1_mmap_reserved_end, t::Eq(0u));
  EXPECT_THAT(parsed.fb_addr, t::Eq(0u));

  // The parser maps the fixed header and then the whole structure before
  // reading tags.
  ASSERT_THAT(g_env.map_calls.size(), t::Ge(2u));
  EXPECT_THAT(g_env.map_calls.back().phys_addr, t::Eq(AddrOf(&info)));
  EXPECT_THAT(g_env.map_calls.back().size,
              t::Eq(static_cast<int64_t>(sizeof(info))));

  EXPECT_THAT(g_env.uart_log, t::HasSubstr("[PMM] Multiboot magic=0x36D76289"));
  EXPECT_THAT(g_env.uart_log,
              t::HasSubstr("[PMM] mmap entry: base=0x100000 len=0x7EE0000 "
                           "type=1\n"));
  EXPECT_THAT(g_env.uart_log,
              t::HasSubstr("[PMM] mmap entry: base=0x7FE0000 len=0x20000 "
                           "type=4\n"));
  EXPECT_THAT(g_env.vga_log, t::Eq(g_env.uart_log));
}

TEST(MultibootTest, ParseRejectsUnknownMagicBadInfoAddressAndMappingFailure) {
  ResetFakeEnv();
  Mb2MmapInfo<1> info;
  InitMb2MmapInfo(&info);
  info.entries[0] = {0x100000, 0x100000, kMemoryTypeAvailable, 0};
  const uintptr_t info_addr = AddrOf(&info);
  MultibootMemoryMap parsed = {};
  const auto parse = [&parsed](const uint32_t magic,
                               const uint64_t multiboot_addr) {
    return MultibootParseMemoryMap(magic,              //
                                   multiboot_addr,     //
                                   kIdentityMapLimit,  //
                                   &parsed);
  };

  EXPECT_FALSE(parse(0x12345678, info_addr));
  EXPECT_FALSE(parse(kMultiboot2Magic, 0));
  // The information structure must lie below `max_physical_addr`.
  EXPECT_FALSE(MultibootParseMemoryMap(kMultiboot2Magic,  //
                                       info_addr,         //
                                       info_addr,         //
                                       &parsed));
  EXPECT_FALSE(MultibootParseMemoryMap(kMultiboot2Magic,   //
                                       info_addr,          //
                                       kIdentityMapLimit,  //
                                       nullptr));

  g_env.fail_bootstrap_map = true;
  EXPECT_FALSE(parse(kMultiboot2Magic, info_addr));

  g_env.fail_bootstrap_map = false;
  EXPECT_TRUE(parse(kMultiboot2Magic, info_addr));
}

TEST(MultibootTest, ParseCoalescesEntriesAndRejectsUncoalesceableOverflow) {
  ResetFakeEnv();
  constexpr int kTotalEntries = kMaxMemoryRegions + 16;
  const auto info = std::make_unique<Mb2MmapInfo<kTotalEntries>>();
  InitMb2MmapInfo(info.get());
  constexpr uintptr_t kBase = 0x200000;

  // More than `kMaxMemoryRegions` page-sized available entries that all fall
  // into `[kBase, kBase + 64 pages)`, followed by a reserved entry inside that
  // range. Coalescing keeps the table small enough to record the reserved
  // entry, which the PMM then carves out.
  for (int i = 0; i < kTotalEntries - 1; ++i) {
    const uintptr_t base = kBase + (i % 64) * kPageBytes;
    info->entries[i] = {base, kPageBytes, kMemoryTypeAvailable, 0};
  }
  info->entries[kTotalEntries - 1] = {kBase + 10 * kPageBytes,  //
                                      4 * kPageBytes,           //
                                      kMemoryTypeReserved,      //
                                      0};

  MultibootMemoryMap parsed = {};
  ASSERT_TRUE(MultibootParseMemoryMap(kMultiboot2Magic,    //
                                      AddrOf(info.get()),  //
                                      kIdentityMapLimit,   //
                                      &parsed));
  ASSERT_THAT(parsed.region_count, t::Eq(2));
  EXPECT_THAT(parsed.regions[0].base, t::Eq(kBase));
  EXPECT_THAT(parsed.regions[0].length, t::Eq(64 * kPageBytes));
  EXPECT_THAT(parsed.regions[0].type, t::Eq(kMemoryTypeAvailable));
  EXPECT_THAT(parsed.regions[1].base, t::Eq(kBase + 10 * kPageBytes));
  EXPECT_THAT(parsed.regions[1].length, t::Eq(4 * kPageBytes));
  EXPECT_THAT(parsed.regions[1].type, t::Eq(kMemoryTypeReserved));

  // More than `kMaxMemoryRegions` disjoint entries cannot be coalesced, so the
  // parser fails instead of dropping regions.
  for (int i = 0; i < kTotalEntries; ++i) {
    const uintptr_t base = kBase + static_cast<uintptr_t>(i * 2) * kPageBytes;
    info->entries[i] = {base, kPageBytes, kMemoryTypeAvailable, 0};
  }
  EXPECT_FALSE(MultibootParseMemoryMap(kMultiboot2Magic,    //
                                       AddrOf(info.get()),  //
                                       kIdentityMapLimit,   //
                                       &parsed));
}

TEST(MultibootTest, Multiboot2MmapTagSizeExceedingTotalSizeIsRejected) {
  ResetFakeEnv();
  struct [[gnu::packed]] TruncatedMb2MmapInfo {
    Multiboot2InfoHeader header;
    Multiboot2TagMmap mmap;
    Multiboot2MmapEntry entry;
  };

  constexpr int64_t kExactSize = sizeof(TruncatedMb2MmapInfo);
  const auto raw_mb2 = std::make_unique<uint8_t[]>(kExactSize);
  std::memset(raw_mb2.get(), 0, kExactSize);
  TruncatedMb2MmapInfo* const mb2 =
      reinterpret_cast<TruncatedMb2MmapInfo*>(raw_mb2.get());
  mb2->header.total_size = static_cast<uint32_t>(kExactSize);
  mb2->mmap.type = kMultiboot2TagMmap;
  mb2->mmap.size = 4096;  // Exceeds total_size - offset (40 bytes).
  mb2->mmap.entry_size = sizeof(Multiboot2MmapEntry);
  mb2->entry = {0x200000, 8 * kPageBytes, kMemoryTypeAvailable, 0};

  MultibootMemoryMap parsed = {};
  EXPECT_FALSE(MultibootParseMemoryMap(kMultiboot2Magic,       //
                                       AddrOf(raw_mb2.get()),  //
                                       kIdentityMapLimit,      //
                                       &parsed));
  EXPECT_THAT(parsed.region_count, t::Eq(0));
}

TEST(MultibootTest, Multiboot1MmapParsesEntriesAndReservesInfoAndMmapBuffer) {
  ResetFakeEnv();
  const Low32BitBuffer buffer(1);
  if (!buffer.ok()) {
    GTEST_SKIP() << "cannot map host memory below 4 GiB (MAP_32BIT)";
  }
  constexpr int kEntries = 4;
  Multiboot1MmapEntry* const entries =
      reinterpret_cast<Multiboot1MmapEntry*>(buffer.EndMinus(kEntries * 24));
  entries[0] = {20, 0x0, 0x9FC00, kMemoryTypeAvailable};
  entries[1] = {20, 0x100000, 0x7EE0000, kMemoryTypeAvailable};
  entries[2] = {20, 0x7FE0000, 0x20000, kMemoryTypeAcpiReclaimable};
  entries[3] = {20, 0xFD00000000, 3ULL << 32, kMemoryTypeReserved};

  Multiboot1Info info = {};
  info.flags = kMultiboot1FlagMmap | kMultiboot1FlagMemBasic;
  info.mem_lower = 639;
  info.mem_upper = 130048;
  info.mmap_length = kEntries * sizeof(Multiboot1MmapEntry);
  info.mmap_addr = static_cast<uint32_t>(AddrOf(entries));

  MultibootMemoryMap parsed = {};
  ASSERT_TRUE(MultibootParseMemoryMap(kMultiboot1Magic,   //
                                      AddrOf(&info),      //
                                      kIdentityMapLimit,  //
                                      &parsed));
  ASSERT_THAT(parsed.region_count, t::Eq(kEntries));
  EXPECT_THAT(parsed.regions[3].base, t::Eq(0xFD00000000u));
  EXPECT_THAT(parsed.usable_ram_bytes, t::Eq(0x9FC00 + 0x7EE0000));
  EXPECT_THAT(parsed.reserved_ram_bytes, t::Eq(0x20000));
  EXPECT_THAT(parsed.total_ram_bytes, t::Eq(0x9FC00 + 0x7EE0000 + 0x20000));
  EXPECT_THAT(parsed.mb_reserved_start, t::Eq(AddrOf(&info)));
  EXPECT_THAT(parsed.mb_reserved_end, t::Gt(parsed.mb_reserved_start));
  EXPECT_THAT(parsed.mb1_mmap_reserved_start, t::Eq(AddrOf(entries)));
  EXPECT_THAT(parsed.mb1_mmap_reserved_end,
              t::Eq(AddrOf(entries) + info.mmap_length));
  EXPECT_THAT(g_env.map_calls.back().phys_addr, t::Eq(AddrOf(entries)));
  EXPECT_THAT(g_env.map_calls.back().size,
              t::Eq(static_cast<int64_t>(info.mmap_length)));
}

TEST(MultibootTest, Multiboot1MmapEntrySizeExceedingMmapLengthIsRejected) {
  ResetFakeEnv();
  const Low32BitBuffer buffer(1);
  if (!buffer.ok()) {
    GTEST_SKIP() << "cannot map host memory below 4 GiB (MAP_32BIT)";
  }
  // The only entry ends right before an inaccessible page, so reading past
  // `mmap_length` would fault.
  Multiboot1MmapEntry* const entry = reinterpret_cast<Multiboot1MmapEntry*>(
      buffer.EndMinus(sizeof(Multiboot1MmapEntry)));
  *entry = {1024, 0x200000, 8 * kPageBytes, kMemoryTypeAvailable};

  Multiboot1Info info = {};
  info.flags = kMultiboot1FlagMmap;
  info.mmap_length = sizeof(Multiboot1MmapEntry);
  info.mmap_addr = static_cast<uint32_t>(AddrOf(entry));

  MultibootMemoryMap parsed = {};
  EXPECT_FALSE(MultibootParseMemoryMap(kMultiboot1Magic,   //
                                       AddrOf(&info),      //
                                       kIdentityMapLimit,  //
                                       &parsed));
  EXPECT_THAT(parsed.region_count, t::Eq(0));
}

TEST(MultibootTest, Multiboot1MemBasicFallbackUses1MiBBaseOffsetForMemUpper) {
  ResetFakeEnv();
  // Without the mmap flag, `mem_lower` describes `[0, mem_lower KiB)` and
  // `mem_upper` describes `[1 MiB, 1 MiB + mem_upper KiB)`.
  Multiboot1Info mb1 = {};
  mb1.flags = kMultiboot1FlagMemBasic;
  mb1.mem_lower = 640;
  mb1.mem_upper = 64;

  MultibootMemoryMap parsed = {};
  ASSERT_TRUE(MultibootParseMemoryMap(kMultiboot1Magic,   //
                                      AddrOf(&mb1),       //
                                      kIdentityMapLimit,  //
                                      &parsed));
  ASSERT_THAT(parsed.region_count, t::Eq(2));
  EXPECT_THAT(parsed.regions[0].base, t::Eq(0u));
  EXPECT_THAT(parsed.regions[0].length, t::Eq(640 * 1024));
  EXPECT_THAT(parsed.regions[0].type, t::Eq(kMemoryTypeAvailable));
  EXPECT_THAT(parsed.regions[1].base, t::Eq(0x100000u));
  EXPECT_THAT(parsed.regions[1].length, t::Eq(64 * 1024));
  EXPECT_THAT(parsed.regions[1].type, t::Eq(kMemoryTypeAvailable));
  EXPECT_THAT(parsed.mb1_mmap_reserved_end, t::Eq(0u));

  // Neither the mmap nor the mem_basic flag: no memory information.
  Multiboot1Info mb1_empty = {};
  mb1_empty.mem_upper = 64;
  MultibootMemoryMap empty_parsed = {};
  EXPECT_FALSE(MultibootParseMemoryMap(kMultiboot1Magic,    //
                                       AddrOf(&mb1_empty),  //
                                       kIdentityMapLimit,   //
                                       &empty_parsed));
}

TEST(MultibootTest, Multiboot2FramebufferTagValidatesPitchAndDimensions) {
  ResetFakeEnv();
  struct [[gnu::packed]] Mb2FramebufferInfo {
    Multiboot2InfoHeader header;
    Multiboot2TagMmap mmap;
    Multiboot2MmapEntry entry;
    Multiboot2TagFramebuffer fb;
    MultibootRgbColorInfo rgb;
    uint8_t fb_padding[2];
    Multiboot2Tag end;
  };
  constexpr uintptr_t kFbAddr = 0xFD000000;

  Mb2FramebufferInfo buf = {};
  buf.header.total_size = sizeof(Mb2FramebufferInfo);
  buf.mmap.type = kMultiboot2TagMmap;
  buf.mmap.size = sizeof(Multiboot2TagMmap) + sizeof(Multiboot2MmapEntry);
  buf.mmap.entry_size = sizeof(Multiboot2MmapEntry);
  buf.entry = {0x200000, 16 * kPageBytes, kMemoryTypeAvailable, 0};
  buf.fb.type = kMultiboot2TagFramebuffer;
  buf.fb.size = sizeof(Multiboot2TagFramebufferRgb);
  buf.fb.framebuffer_addr = kFbAddr;
  buf.fb.framebuffer_pitch = 640 * 4;
  buf.fb.framebuffer_width = 640;
  buf.fb.framebuffer_height = 480;
  buf.fb.framebuffer_bpp = 32;
  buf.fb.framebuffer_type = kMultiboot2FramebufferTypeRgb;
  // A layout with the reserved byte at the bottom, as GOP bitmask modes can
  // have.
  buf.rgb = {24, 8, 16, 8, 8, 8};
  buf.end.type = kMultiboot2TagEnd;
  buf.end.size = sizeof(Multiboot2Tag);

  const uintptr_t buf_addr = AddrOf(&buf);
  MultibootMemoryMap parsed = {};
  const auto parse = [&]() {
    return MultibootParseMemoryMap(kMultiboot2Magic,   //
                                   buf_addr,           //
                                   kIdentityMapLimit,  //
                                   &parsed);
  };

  // Valid 32-bpp and 24-bpp framebuffer tags are accepted, with their color
  // channels.
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.fb_addr, t::Eq(kFbAddr));
  EXPECT_THAT(parsed.fb_pitch, t::Eq(640 * 4));
  EXPECT_THAT(parsed.fb_width, t::Eq(640));
  EXPECT_THAT(parsed.fb_height, t::Eq(480));
  EXPECT_THAT(parsed.fb_bpp, t::Eq(32));
  EXPECT_THAT(parsed.fb_red_position, t::Eq(24));
  EXPECT_THAT(parsed.fb_red_size, t::Eq(8));
  EXPECT_THAT(parsed.fb_green_position, t::Eq(16));
  EXPECT_THAT(parsed.fb_green_size, t::Eq(8));
  EXPECT_THAT(parsed.fb_blue_position, t::Eq(8));
  EXPECT_THAT(parsed.fb_blue_size, t::Eq(8));

  buf.fb.framebuffer_bpp = 24;
  buf.fb.framebuffer_pitch = 640 * 3;
  buf.rgb = {0, 8, 8, 8, 16, 8};
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.fb_addr, t::Eq(kFbAddr));
  EXPECT_THAT(parsed.fb_pitch, t::Eq(640 * 3));
  EXPECT_THAT(parsed.fb_bpp, t::Eq(24));
  EXPECT_THAT(parsed.fb_red_position, t::Eq(0));
  EXPECT_THAT(parsed.fb_blue_position, t::Eq(16));

  // pitch < width * (bpp / 8) is rejected for both 24-bpp and 32-bpp.
  buf.fb.framebuffer_pitch = 640 * 3 - 1;
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.fb_addr, t::Eq(0u));
  EXPECT_THAT(parsed.fb_pitch, t::Eq(0));

  buf.fb.framebuffer_bpp = 32;
  buf.fb.framebuffer_pitch = 640 * 4 - 1;
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.fb_addr, t::Eq(0u));

  // Zero width or height is rejected.
  buf.fb.framebuffer_pitch = 640 * 4;
  buf.fb.framebuffer_width = 0;
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.fb_addr, t::Eq(0u));

  buf.fb.framebuffer_width = 640;
  buf.fb.framebuffer_height = 0;
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.fb_addr, t::Eq(0u));

  // Dimensions or pitch exceeding INT32_MAX are rejected.
  constexpr uint32_t kOverInt32Max = static_cast<uint32_t>(INT32_MAX) + 1u;
  buf.fb.framebuffer_height = 480;
  buf.fb.framebuffer_pitch = kOverInt32Max;
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.fb_addr, t::Eq(0u));

  buf.fb.framebuffer_pitch = 640 * 4;
  buf.fb.framebuffer_width = kOverInt32Max;
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.fb_addr, t::Eq(0u));

  buf.fb.framebuffer_width = 640;
  buf.fb.framebuffer_height = kOverInt32Max;
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.fb_addr, t::Eq(0u));

  // Non-RGB framebuffers (e.g. EGA text mode) are ignored.
  buf.fb.framebuffer_height = 480;
  buf.fb.framebuffer_type = 2;
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.fb_addr, t::Eq(0u));

  // So is an RGB framebuffer tag that is too small for the color information.
  buf.fb.framebuffer_type = kMultiboot2FramebufferTypeRgb;
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.fb_addr, t::Eq(kFbAddr));
  buf.fb.size = sizeof(Multiboot2TagFramebuffer);
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.fb_addr, t::Eq(0u));
  EXPECT_THAT(parsed.fb_red_size, t::Eq(0));
}

// Writes a Multiboot2 tag header at `offset` in `info`.
static void PutTag(uint8_t* const info, const int64_t offset,
                   const uint32_t type, const uint32_t size) {
  const Multiboot2Tag tag = {type, size};
  std::memcpy(info + offset, &tag, sizeof(tag));
}

// Returns the (type, offset) of every tag that Multiboot2TagIterator visits.
static std::vector<std::pair<uint32_t, int64_t>> VisitTags(
    const uint8_t* const info, const int64_t total_size) {
  std::vector<std::pair<uint32_t, int64_t>> visited;
  for (Multiboot2TagIterator it(info, total_size); !it.Done(); it.Next()) {
    EXPECT_THAT(
        &it.tag(),
        t::Eq(reinterpret_cast<const Multiboot2Tag*>(info + it.offset())));
    visited.emplace_back(it.tag().type, it.offset());
  }
  return visited;
}

TEST(MultibootTest, Multiboot2TagIteratorVisitsTagsAtEightByteAlignedOffsets) {
  alignas(8) uint8_t info[80] = {};
  PutTag(info, 8, kMultiboot2TagMmap, 12);
  PutTag(info, 24, kMultiboot2TagAcpiOld, 8);
  PutTag(info, 32, kMultiboot2TagFramebuffer, 17);
  PutTag(info, 56, kMultiboot2TagEnd, 8);
  // Not visited: it follows the end tag.
  PutTag(info, 64, kMultiboot2TagAcpiNew, 16);

  EXPECT_THAT(VisitTags(info, sizeof(info)),
              t::ElementsAre(t::Pair(kMultiboot2TagMmap, 8),
                             t::Pair(kMultiboot2TagAcpiOld, 24),
                             t::Pair(kMultiboot2TagFramebuffer, 32)));
}

TEST(MultibootTest, Multiboot2TagIteratorStopsAtMalformedTagsAndTotalSize) {
  alignas(8) uint8_t info[40] = {};
  PutTag(info, 8, kMultiboot2TagMmap, 16);

  // A tag smaller than its header ends the walk.
  PutTag(info, 24, kMultiboot2TagAcpiOld, 7);
  EXPECT_THAT(VisitTags(info, 40),
              t::ElementsAre(t::Pair(kMultiboot2TagMmap, 8)));
  // So does a tag that extends past `total_size`.
  PutTag(info, 24, kMultiboot2TagAcpiOld, 17);
  EXPECT_THAT(VisitTags(info, 40),
              t::ElementsAre(t::Pair(kMultiboot2TagMmap, 8)));

  // Without an end tag, the walk ends at `total_size`, also when fewer bytes
  // than a tag header are left.
  PutTag(info, 24, kMultiboot2TagAcpiOld, 16);
  EXPECT_THAT(VisitTags(info, 40),
              t::ElementsAre(t::Pair(kMultiboot2TagMmap, 8),
                             t::Pair(kMultiboot2TagAcpiOld, 24)));
  EXPECT_THAT(VisitTags(info, 30),
              t::ElementsAre(t::Pair(kMultiboot2TagMmap, 8)));

  // A structure that ends after its fixed header has no tags, and an iterator
  // that is done stays done.
  EXPECT_THAT(VisitTags(info, sizeof(Multiboot2InfoHeader)), t::IsEmpty());
  Multiboot2TagIterator it(info, 24);
  ASSERT_FALSE(it.Done());
  it.Next();
  EXPECT_TRUE(it.Done());
  it.Next();
  EXPECT_TRUE(it.Done());
}

TEST(MultibootTest, Multiboot2MapInfoMapsTheHeaderAndThenTheWholeStructure) {
  ResetFakeEnv();
  alignas(8) uint8_t info[48] = {};
  const Multiboot2InfoHeader header = {sizeof(info), 0};
  std::memcpy(info, &header, sizeof(header));
  const uintptr_t info_addr = AddrOf(info);

  const StatusOr<Multiboot2MappedInfo> mapped =
      Multiboot2MapInfo(info_addr, kIdentityMapLimit, &RecordingMapper);
  ASSERT_TRUE(mapped.ok());
  EXPECT_THAT(mapped->data, t::Eq(info));
  EXPECT_THAT(mapped->total_size, t::Eq(48));
  EXPECT_THAT(g_env.map_calls,
              t::ElementsAre(MapCall{info_addr, sizeof(Multiboot2InfoHeader)},
                             MapCall{info_addr, 48}));

  // The structure may end exactly at `max_physical_addr`.
  const StatusOr<Multiboot2MappedInfo> mapped_exact =
      Multiboot2MapInfo(info_addr,       //
                        info_addr + 48,  //
                        &RecordingMapper);
  ASSERT_TRUE(mapped_exact.ok());
  EXPECT_THAT(mapped_exact->data, t::Eq(info));
}

TEST(MultibootTest, Multiboot2MapInfoRejectsBadAddressesSizesAndMapFailures) {
  ResetFakeEnv();
  alignas(8) uint8_t info[48] = {};
  const uintptr_t info_addr = AddrOf(info);
  const auto set_total_size = [&info](const uint32_t total_size) {
    const Multiboot2InfoHeader header = {total_size, 0};
    std::memcpy(info, &header, sizeof(header));
  };
  const auto map_info = [](const uintptr_t addr,
                           const uintptr_t max_physical_addr) {
    const StatusOr<Multiboot2MappedInfo> mapped =
        Multiboot2MapInfo(addr, max_physical_addr, &RecordingMapper);
    if (mapped.ok()) {
      EXPECT_THAT(mapped->total_size, t::Eq(48));
      return mapped->data;
    }
    return static_cast<const uint8_t*>(nullptr);
  };
  set_total_size(sizeof(info));

  // Bad addresses are rejected before anything is mapped: 0, at or above
  // `max_physical_addr`, or with a header that does not end below it.
  EXPECT_THAT(map_info(0, kIdentityMapLimit), t::IsNull());
  EXPECT_THAT(map_info(info_addr, info_addr), t::IsNull());
  EXPECT_THAT(map_info(info_addr, info_addr + 7), t::IsNull());
  EXPECT_THAT(g_env.map_calls, t::IsEmpty());

  // So are sizes smaller than the header, larger than
  // `kMaxMultibootStructureBytes`, or reaching past `max_physical_addr`.
  set_total_size(7);
  EXPECT_THAT(map_info(info_addr, kIdentityMapLimit), t::IsNull());
  set_total_size(static_cast<uint32_t>(kMaxMultibootStructureBytes) + 1);
  EXPECT_THAT(map_info(info_addr, kIdentityMapLimit), t::IsNull());
  set_total_size(sizeof(info));
  EXPECT_THAT(map_info(info_addr, info_addr + 47), t::IsNull());

  // Either mapping can fail.
  g_env.map_calls.clear();
  g_env.fail_map_call = 0;
  EXPECT_THAT(map_info(info_addr, kIdentityMapLimit), t::IsNull());
  g_env.map_calls.clear();
  g_env.fail_map_call = 1;
  EXPECT_THAT(map_info(info_addr, kIdentityMapLimit), t::IsNull());
  EXPECT_THAT(g_env.map_calls, t::SizeIs(2));
  g_env.map_calls.clear();
  g_env.fail_map_call = -1;
  EXPECT_THAT(map_info(info_addr, kIdentityMapLimit), t::Eq(info));
}

// Multiboot1 boot information up to the end of its framebuffer fields.
struct [[gnu::packed]] Mb1InfoWithFramebuffer {
  Multiboot1Info info;
  uint8_t other_fields[36];
  Multiboot1FramebufferInfo fb;
};
static_assert(offsetof(Mb1InfoWithFramebuffer, fb) ==
              kMultiboot1FramebufferInfoOffset);

// Fills `fb` with GRUB's description of VGA text mode, which it also passes
// with Multiboot2 (except for the color information).
static void SetVgaTextMode(Multiboot1FramebufferInfo* const fb) {
  *fb = {};
  fb->framebuffer_addr = 0xB8000;
  fb->framebuffer_pitch = 160;
  fb->framebuffer_width = 80;
  fb->framebuffer_height = 25;
  fb->framebuffer_bpp = 16;
  fb->framebuffer_type = kMultiboot2FramebufferTypeEgaText;
}

TEST(MultibootTest, Multiboot1FramebufferFieldsAreParsedWhenFlagged) {
  ResetFakeEnv();
  Mb1InfoWithFramebuffer mb1 = {};
  mb1.info.flags = kMultiboot1FlagMemBasic | kMultiboot1FlagFramebuffer;
  mb1.info.mem_lower = 639;
  mb1.info.mem_upper = 130048;
  // What GRUB passes for the GOP framebuffer of a UEFI QEMU VM.
  mb1.fb.framebuffer_addr = 0xC0000000;
  mb1.fb.framebuffer_pitch = 800 * 3;
  mb1.fb.framebuffer_width = 800;
  mb1.fb.framebuffer_height = 600;
  mb1.fb.framebuffer_bpp = 24;
  mb1.fb.framebuffer_type = kMultiboot2FramebufferTypeRgb;
  mb1.fb.rgb = {16, 8, 8, 8, 0, 8};
  const uintptr_t mb1_addr = AddrOf(&mb1);
  MultibootMemoryMap parsed = {};
  const auto parse = [&](const uintptr_t max_physical_addr) {
    return MultibootParseMemoryMap(kMultiboot1Magic,   //
                                   mb1_addr,           //
                                   max_physical_addr,  //
                                   &parsed);
  };

  ASSERT_TRUE(parse(kIdentityMapLimit));
  EXPECT_THAT(parsed.fb_addr, t::Eq(0xC0000000u));
  EXPECT_THAT(parsed.fb_pitch, t::Eq(800 * 3));
  EXPECT_THAT(parsed.fb_width, t::Eq(800));
  EXPECT_THAT(parsed.fb_height, t::Eq(600));
  EXPECT_THAT(parsed.fb_bpp, t::Eq(24));
  EXPECT_THAT(parsed.fb_red_position, t::Eq(16));
  EXPECT_THAT(parsed.fb_red_size, t::Eq(8));
  EXPECT_THAT(parsed.fb_green_position, t::Eq(8));
  EXPECT_THAT(parsed.fb_green_size, t::Eq(8));
  EXPECT_THAT(parsed.fb_blue_position, t::Eq(0));
  EXPECT_THAT(parsed.fb_blue_size, t::Eq(8));
  EXPECT_THAT(parsed.mb_reserved_start, t::Eq(mb1_addr));
  EXPECT_THAT(parsed.mb_reserved_end, t::Eq(mb1_addr + sizeof(mb1)));

  // Framebuffer fields that would reach past `max_physical_addr` are ignored.
  ASSERT_TRUE(parse(mb1_addr + sizeof(mb1) - 1));
  EXPECT_THAT(parsed.fb_addr, t::Eq(0u));
  EXPECT_THAT(parsed.mb_reserved_end, t::Eq(mb1_addr + sizeof(Multiboot1Info)));

  // VGA text mode is no framebuffer to draw on.
  SetVgaTextMode(&mb1.fb);
  ASSERT_TRUE(parse(kIdentityMapLimit));
  EXPECT_THAT(parsed.fb_addr, t::Eq(0u));

  // Without the flag, the parser reads nothing past the fixed fields, which
  // here end right before an inaccessible page.
  const Low32BitBuffer buffer(1);
  if (!buffer.ok()) {
    GTEST_SKIP() << "cannot map host memory below 4 GiB (MAP_32BIT)";
  }
  Multiboot1Info* const info = reinterpret_cast<Multiboot1Info*>(
      buffer.EndMinus(sizeof(Multiboot1Info)));
  *info = mb1.info;
  info->flags = kMultiboot1FlagMemBasic;
  ASSERT_TRUE(MultibootParseMemoryMap(kMultiboot1Magic,   //
                                      AddrOf(info),       //
                                      kIdentityMapLimit,  //
                                      &parsed));
  EXPECT_THAT(parsed.fb_addr, t::Eq(0u));
  EXPECT_THAT(parsed.mb_reserved_end,
              t::Eq(AddrOf(info) + sizeof(Multiboot1Info)));
}

// Builds a Multiboot2 boot information structure from tags.
class Mb2InfoBuilder {
 public:
  // Appends the `size` bytes at `tag`, a tag whose header holds `size`, at the
  // next 8-byte aligned offset.
  void Add(const void* const tag, const int64_t size) {
    std::memcpy(bytes_ + size_, tag, size);
    size_ = (size_ + size + 7) & ~int64_t{7};
  }

  // Appends a tag of type `type` without contents.
  void AddEmpty(const uint32_t type) {
    const Multiboot2Tag tag = {type, sizeof(Multiboot2Tag)};
    Add(&tag, sizeof(tag));
  }

  // Appends the end tag and returns the address of the structure.
  uintptr_t Finish() {
    AddEmpty(kMultiboot2TagEnd);
    const Multiboot2InfoHeader header = {static_cast<uint32_t>(size_), 0};
    std::memcpy(bytes_, &header, sizeof(header));
    return AddrOf(bytes_);
  }

 private:
  alignas(8) uint8_t bytes_[1024] = {};
  int64_t size_ = sizeof(Multiboot2InfoHeader);
};

// Returns a Multiboot2 framebuffer tag of type `type` at `addr`, of the size
// that GRUB gives it.
static Multiboot2TagFramebufferRgb MakeFramebufferTag(const uint8_t type,
                                                      const uint64_t addr) {
  Multiboot2TagFramebufferRgb tag = {};
  tag.common.type = kMultiboot2TagFramebuffer;
  tag.common.framebuffer_addr = addr;
  tag.common.framebuffer_type = type;
  if (type == kMultiboot2FramebufferTypeRgb) {
    tag.common.size = sizeof(Multiboot2TagFramebufferRgb);
    tag.common.framebuffer_pitch = 800 * 3;
    tag.common.framebuffer_width = 800;
    tag.common.framebuffer_height = 600;
    tag.common.framebuffer_bpp = 24;
    tag.rgb = {16, 8, 8, 8, 0, 8};
  } else {
    tag.common.size = sizeof(Multiboot2TagFramebuffer);
    tag.common.framebuffer_pitch = 160;
    tag.common.framebuffer_width = 80;
    tag.common.framebuffer_height = 25;
    tag.common.framebuffer_bpp = 16;
  }
  return tag;
}

TEST(MultibootTest, HasVgaTextModeFollowsTheMultiboot2FramebufferAndEfiTags) {
  ResetFakeEnv();
  const auto has_vga_text = [](Mb2InfoBuilder* const builder) {
    return MultibootHasVgaTextMode(kMultiboot2Magic,   //
                                   builder->Finish(),  //
                                   kIdentityMapLimit);
  };
  const auto add_framebuffer = [](Mb2InfoBuilder* const builder,
                                  const uint8_t type, const uint64_t addr) {
    const Multiboot2TagFramebufferRgb tag = MakeFramebufferTag(type, addr);
    builder->Add(&tag, tag.common.size);
  };

  {
    // What GRUB passes on a BIOS machine in text mode.
    Mb2InfoBuilder builder;
    builder.AddEmpty(kMultiboot2TagAcpiOld);
    add_framebuffer(&builder, kMultiboot2FramebufferTypeEgaText, 0xB8000);
    EXPECT_TRUE(has_vga_text(&builder));
  }
  {
    // The monochrome text buffer, whose CRT controller is elsewhere.
    Mb2InfoBuilder builder;
    add_framebuffer(&builder, kMultiboot2FramebufferTypeEgaText, 0xB0000);
    EXPECT_FALSE(has_vga_text(&builder));
  }
  {
    // What GRUB passes on a UEFI machine with a GOP framebuffer.
    Mb2InfoBuilder builder;
    add_framebuffer(&builder, kMultiboot2FramebufferTypeRgb, 0xC0000000);
    builder.AddEmpty(kMultiboot2TagEfi64);
    builder.AddEmpty(kMultiboot2TagEfiMmap);
    EXPECT_FALSE(has_vga_text(&builder));
  }
  {
    // A graphics mode on a BIOS machine.
    Mb2InfoBuilder builder;
    add_framebuffer(&builder, kMultiboot2FramebufferTypeRgb, 0xC0000000);
    EXPECT_FALSE(has_vga_text(&builder));
  }
  {
    Mb2InfoBuilder builder;
    add_framebuffer(&builder, kMultiboot2FramebufferTypeIndexed, 0xA0000);
    EXPECT_FALSE(has_vga_text(&builder));
  }
  {
    // No framebuffer tag and no EFI tag: a BIOS boot.
    Mb2InfoBuilder builder;
    builder.AddEmpty(kMultiboot2TagAcpiOld);
    EXPECT_TRUE(has_vga_text(&builder));
  }
  // No framebuffer tag but an EFI tag: a UEFI boot without a framebuffer.
  for (const uint32_t efi_tag :
       {kMultiboot2TagEfi32, kMultiboot2TagEfi64, kMultiboot2TagEfiMmap,
        kMultiboot2TagEfiBootServices, kMultiboot2TagEfi32ImageHandle,
        kMultiboot2TagEfi64ImageHandle}) {
    Mb2InfoBuilder builder;
    builder.AddEmpty(kMultiboot2TagAcpiNew);
    builder.AddEmpty(efi_tag);
    EXPECT_FALSE(has_vga_text(&builder)) << "EFI tag " << efi_tag;
  }
  {
    // The framebuffer tag decides when there is one.
    Mb2InfoBuilder builder;
    builder.AddEmpty(kMultiboot2TagEfi64);
    add_framebuffer(&builder, kMultiboot2FramebufferTypeEgaText, 0xB8000);
    EXPECT_TRUE(has_vga_text(&builder));
  }
  {
    // Boot information that cannot be read tells nothing.
    Mb2InfoBuilder builder;
    const uintptr_t info_addr = builder.Finish();
    g_env.fail_bootstrap_map = true;
    EXPECT_FALSE(MultibootHasVgaTextMode(kMultiboot2Magic,  //
                                         info_addr,         //
                                         kIdentityMapLimit));
    g_env.fail_bootstrap_map = false;
    EXPECT_TRUE(MultibootHasVgaTextMode(kMultiboot2Magic,  //
                                        info_addr,         //
                                        kIdentityMapLimit));
    EXPECT_FALSE(
        MultibootHasVgaTextMode(0x12345678, info_addr, kIdentityMapLimit));
    EXPECT_FALSE(
        MultibootHasVgaTextMode(kMultiboot2Magic, 0, kIdentityMapLimit));
    EXPECT_FALSE(
        MultibootHasVgaTextMode(kMultiboot2Magic, info_addr, info_addr));
  }
  EXPECT_THAT(g_env.uart_log, t::IsEmpty());
  EXPECT_THAT(g_env.vga_log, t::IsEmpty());
}

TEST(MultibootTest, HasVgaTextModeFollowsTheMultiboot1Framebuffer) {
  ResetFakeEnv();
  Mb1InfoWithFramebuffer mb1 = {};
  mb1.info.flags = kMultiboot1FlagMemBasic | kMultiboot1FlagFramebuffer;
  const uintptr_t mb1_addr = AddrOf(&mb1);
  const auto has_vga_text = [mb1_addr](const uintptr_t max_physical_addr) {
    return MultibootHasVgaTextMode(kMultiboot1Magic,  //
                                   mb1_addr,          //
                                   max_physical_addr);
  };

  // What GRUB passes on a BIOS machine in text mode.
  SetVgaTextMode(&mb1.fb);
  EXPECT_TRUE(has_vga_text(kIdentityMapLimit));
  // Unless the framebuffer fields are out of reach.
  EXPECT_FALSE(has_vga_text(mb1_addr + sizeof(mb1) - 1));
  g_env.fail_bootstrap_map = true;
  EXPECT_FALSE(has_vga_text(kIdentityMapLimit));
  g_env.fail_bootstrap_map = false;

  mb1.fb.framebuffer_addr = 0xB0000;
  EXPECT_FALSE(has_vga_text(kIdentityMapLimit));

  // What GRUB passes on a UEFI machine with a GOP framebuffer.
  mb1.fb.framebuffer_addr = 0xC0000000;
  mb1.fb.framebuffer_type = kMultiboot2FramebufferTypeRgb;
  EXPECT_FALSE(has_vga_text(kIdentityMapLimit));

  // Without framebuffer fields: a BIOS boot. They are not read then, and here
  // they would lie in an inaccessible page.
  const Low32BitBuffer buffer(1);
  if (!buffer.ok()) {
    GTEST_SKIP() << "cannot map host memory below 4 GiB (MAP_32BIT)";
  }
  Multiboot1Info* const info = reinterpret_cast<Multiboot1Info*>(
      buffer.EndMinus(sizeof(Multiboot1Info)));
  *info = {};
  info->flags = kMultiboot1FlagMemBasic;
  EXPECT_TRUE(MultibootHasVgaTextMode(kMultiboot1Magic,  //
                                      AddrOf(info),      //
                                      kIdentityMapLimit));
  EXPECT_THAT(g_env.uart_log, t::IsEmpty());
}

// Appends a memory map tag with one available region to `builder`.
static void AddAvailableMemoryTag(Mb2InfoBuilder* const builder) {
  struct [[gnu::packed]] MmapTagWithOneEntry {
    Multiboot2TagMmap mmap;
    Multiboot2MmapEntry entry;
  };
  MmapTagWithOneEntry tag = {};
  tag.mmap.type = kMultiboot2TagMmap;
  tag.mmap.size = sizeof(tag);
  tag.mmap.entry_size = sizeof(Multiboot2MmapEntry);
  tag.entry = {0x100000, 0x7F00000, kMemoryTypeAvailable, 0};
  builder->Add(&tag, sizeof(tag));
}

// Appends a module tag for `[start, end)` to `builder`, whose command line
// `cmdline` fills the rest of the tag, followed by a NUL byte if
// `nul_terminated`.
static void AddModuleTag(Mb2InfoBuilder* const builder,  //
                         const uintptr_t start,          //
                         const uintptr_t end,            //
                         const std::string& cmdline,     //
                         const bool nul_terminated = true) {
  const int64_t size =
      static_cast<int64_t>(sizeof(Multiboot2TagModule) + cmdline.size()) +
      (nul_terminated ? 1 : 0);
  std::vector<uint8_t> tag(size, 0);
  const Multiboot2TagModule header = {kMultiboot2TagModule,          //
                                      static_cast<uint32_t>(size),   //
                                      static_cast<uint32_t>(start),  //
                                      static_cast<uint32_t>(end)};
  std::memcpy(tag.data(), &header, sizeof(header));
  std::memcpy(tag.data() + sizeof(header), cmdline.data(), cmdline.size());
  builder->Add(tag.data(), size);
}

// Returns the name of `module` if NUL bytes follow it through the end of
// `BootModule::name`, else a description of the bad name.
static std::string NameOf(const BootModule& module) {
  const int64_t length = static_cast<int64_t>(
      strnlen(module.name, static_cast<size_t>(kBootModuleNameSize)));
  if (length == kBootModuleNameSize) {
    return "<no NUL terminator>";
  }
  for (int64_t i = length; i < kBootModuleNameSize; ++i) {
    if (module.name[i] != '\0') {
      return "<garbage after the NUL terminator>";
    }
  }
  return std::string(module.name, static_cast<size_t>(length));
}

TEST(MultibootTest, Multiboot2ModuleTagsAreNamedByTheLastWordOfTheirCmdline) {
  ResetFakeEnv();
  const std::string long_word(80, 'x');
  Mb2InfoBuilder builder;
  // What GRUB 2 passes for `module2 /boot/rootfs.tar rootfs`: only the
  // arguments after the file name.
  AddModuleTag(&builder, 0x200000, 0x202800, "rootfs");
  // An empty module with an empty command line.
  AddModuleTag(&builder, 0x300000, 0x300000, "");
  AddModuleTag(&builder, 0x400000, 0x401000, "  first\tlast \t ");
  AddModuleTag(&builder, 0x500000, 0x501000, "a " + long_word);
  AddModuleTag(&builder, 0x600000, 0x601000, "word unterminated",
               /*nul_terminated=*/false);
  AddAvailableMemoryTag(&builder);
  const uintptr_t info_addr = builder.Finish();

  MultibootMemoryMap parsed = {};
  ASSERT_TRUE(MultibootParseMemoryMap(kMultiboot2Magic,   //
                                      info_addr,          //
                                      kIdentityMapLimit,  //
                                      &parsed));
  ASSERT_THAT(parsed.module_count, t::Eq(5));
  EXPECT_THAT(parsed.modules[0].start, t::Eq(0x200000u));
  EXPECT_THAT(parsed.modules[0].end, t::Eq(0x202800u));
  EXPECT_THAT(NameOf(parsed.modules[0]), t::Eq("rootfs"));
  EXPECT_THAT(parsed.modules[1].start, t::Eq(0x300000u));
  EXPECT_THAT(parsed.modules[1].end, t::Eq(0x300000u));
  EXPECT_THAT(NameOf(parsed.modules[1]), t::Eq(""));
  EXPECT_THAT(NameOf(parsed.modules[2]), t::Eq("last"));
  // A name is cut to `kBootModuleNameSize - 1` bytes.
  EXPECT_THAT(NameOf(parsed.modules[3]),
              t::Eq(std::string(kBootModuleNameSize - 1, 'x')));
  // A command line ends at the end of its tag.
  EXPECT_THAT(NameOf(parsed.modules[4]), t::Eq("unterminated"));
  // The memory map still counts.
  EXPECT_THAT(parsed.region_count, t::Eq(1));

  EXPECT_THAT(g_env.uart_log,
              t::HasSubstr("[PMM] boot module: base=0x200000 len=0x2800 "
                           "name=rootfs\n"));
  EXPECT_THAT(g_env.uart_log,
              t::HasSubstr("[PMM] boot module: base=0x400000 len=0x1000 "
                           "name=last\n"));
  EXPECT_THAT(g_env.vga_log, t::Eq(g_env.uart_log));

  // The quiet parser records the same modules without logging them.
  ResetFakeEnv();
  MultibootMemoryMap quiet = {};
  ASSERT_TRUE(MultibootParseMemoryMapQuiet(kMultiboot2Magic,   //
                                           info_addr,          //
                                           kIdentityMapLimit,  //
                                           &quiet));
  EXPECT_THAT(quiet.module_count, t::Eq(5));
  EXPECT_THAT(g_env.uart_log, t::IsEmpty());
}

TEST(MultibootTest, Multiboot2ParserSkipsInvalidModulesAndKeepsTheFirstEight) {
  ResetFakeEnv();
  Mb2InfoBuilder builder;
  AddModuleTag(&builder, 0, 0x1000, "at_zero");
  AddModuleTag(&builder, 0x3000, 0x2000, "backwards");
  // A tag too small for the module fields.
  const Multiboot2Tag short_tag = {kMultiboot2TagModule, sizeof(Multiboot2Tag)};
  builder.Add(&short_tag, sizeof(short_tag));
  for (int i = 0; i <= kMaxBootModules; ++i) {
    const uintptr_t start = 0x100000 * static_cast<uintptr_t>(i + 1);
    AddModuleTag(&builder, start, start + 0x1000, "m" + std::to_string(i));
  }
  AddAvailableMemoryTag(&builder);

  MultibootMemoryMap parsed = {};
  ASSERT_TRUE(MultibootParseMemoryMap(kMultiboot2Magic,   //
                                      builder.Finish(),   //
                                      kIdentityMapLimit,  //
                                      &parsed));
  ASSERT_THAT(parsed.module_count, t::Eq(kMaxBootModules));
  for (int i = 0; i < kMaxBootModules; ++i) {
    EXPECT_THAT(NameOf(parsed.modules[i]), t::Eq("m" + std::to_string(i)));
  }
  EXPECT_THAT(g_env.uart_log, t::Not(t::HasSubstr("at_zero")));
  EXPECT_THAT(g_env.uart_log, t::Not(t::HasSubstr("backwards")));

  // A new parse starts without modules.
  Mb2InfoBuilder no_modules;
  AddAvailableMemoryTag(&no_modules);
  ASSERT_TRUE(MultibootParseMemoryMap(kMultiboot2Magic,     //
                                      no_modules.Finish(),  //
                                      kIdentityMapLimit,    //
                                      &parsed));
  EXPECT_THAT(parsed.module_count, t::Eq(0));
}

TEST(MultibootTest, MultibootModulesMustEndAtOrBelowTheMaxPhysicalAddress) {
  ResetFakeEnv();
  const Low32BitBuffer buffer(1);
  if (!buffer.ok()) {
    GTEST_SKIP() << "cannot map host memory below 4 GiB (MAP_32BIT)";
  }
  // The boot information ends where the accessible memory ends, which is
  // also the `max_physical_addr` of the parse.
  const uintptr_t max_physical_addr = AddrOf(buffer.EndMinus(0));
  Mb2InfoBuilder builder;
  AddModuleTag(&builder,                    //
               max_physical_addr - 0x2000,  //
               max_physical_addr,           //
               "at_limit");
  AddModuleTag(&builder,                    //
               max_physical_addr - 0x1000,  //
               max_physical_addr + 1,       //
               "past_limit");
  AddAvailableMemoryTag(&builder);
  const uintptr_t built = builder.Finish();
  const int64_t total_size =
      reinterpret_cast<const Multiboot2InfoHeader*>(built)->total_size;
  uint8_t* const info = buffer.EndMinus(total_size);
  std::memcpy(info, reinterpret_cast<const void*>(built), total_size);

  MultibootMemoryMap parsed = {};
  ASSERT_TRUE(MultibootParseMemoryMap(kMultiboot2Magic,   //
                                      AddrOf(info),       //
                                      max_physical_addr,  //
                                      &parsed));
  ASSERT_THAT(parsed.module_count, t::Eq(1));
  EXPECT_THAT(NameOf(parsed.modules[0]), t::Eq("at_limit"));
  EXPECT_THAT(parsed.modules[0].end, t::Eq(max_physical_addr));
}

TEST(MultibootTest, Multiboot1ModuleListIsNamedByTheLastWordOfEachString) {
  ResetFakeEnv();
  const Low32BitBuffer buffer(1);
  if (!buffer.ok()) {
    GTEST_SKIP() << "cannot map host memory below 4 GiB (MAP_32BIT)";
  }
  // The boot information, the module list and the module strings in low
  // memory, which their 32-bit addresses can point at, up to the end of the
  // accessible memory.
  struct [[gnu::packed]] Mb1WithModules {
    Multiboot1Info info;
    Multiboot1Module mods[4];
    char rootfs_cmdline[32];
    char initrd_cmdline[32];
    // At the end, without a NUL terminator.
    char last_cmdline[7];
  };
  Mb1WithModules* const mb1 = reinterpret_cast<Mb1WithModules*>(
      buffer.EndMinus(sizeof(Mb1WithModules)));
  std::memset(static_cast<void*>(mb1), 0, sizeof(*mb1));
  const auto low_addr = [](const void* const ptr) {
    return static_cast<uint32_t>(AddrOf(ptr));
  };
  std::strcpy(mb1->rootfs_cmdline, "/boot/rootfs.tar rootfs");
  std::strcpy(mb1->initrd_cmdline, "initrd");
  std::memcpy(mb1->last_cmdline, "abc def", sizeof(mb1->last_cmdline));
  mb1->info.flags = kMultiboot1FlagMemBasic | kMultiboot1FlagMods;
  mb1->info.mem_lower = 639;
  mb1->info.mem_upper = 130048;
  mb1->info.mods_count = 4;
  mb1->info.mods_addr = low_addr(mb1->mods);
  // A command line as QEMU's `-initrd` and GRUB Legacy pass it: the module's
  // file name, then its arguments (GRUB 2 passes only the arguments).
  mb1->mods[0] = {0x200000, 0x202800, low_addr(mb1->rootfs_cmdline), 0};
  // Without a command line.
  mb1->mods[1] = {0x300000, 0x301000, 0, 0};
  // Ends before it starts.
  mb1->mods[2] = {0x400000, 0x3FF000, low_addr(mb1->initrd_cmdline), 0};
  mb1->mods[3] = {0x500000, 0x501000, low_addr(mb1->initrd_cmdline), 0};
  const uintptr_t max_physical_addr = AddrOf(buffer.EndMinus(0));
  MultibootMemoryMap parsed = {};
  const auto parse = [&]() {
    return MultibootParseMemoryMap(kMultiboot1Magic,    //
                                   AddrOf(&mb1->info),  //
                                   max_physical_addr,   //
                                   &parsed);
  };

  ASSERT_TRUE(parse());
  ASSERT_THAT(parsed.module_count, t::Eq(3));
  EXPECT_THAT(parsed.modules[0].start, t::Eq(0x200000u));
  EXPECT_THAT(parsed.modules[0].end, t::Eq(0x202800u));
  EXPECT_THAT(NameOf(parsed.modules[0]), t::Eq("rootfs"));
  EXPECT_THAT(NameOf(parsed.modules[1]), t::Eq(""));
  EXPECT_THAT(parsed.modules[2].start, t::Eq(0x500000u));
  EXPECT_THAT(NameOf(parsed.modules[2]), t::Eq("initrd"));
  EXPECT_THAT(g_env.uart_log,
              t::HasSubstr("[PMM] boot module: base=0x200000 len=0x2800 "
                           "name=rootfs\n"));
  // The module list is not reserved: nothing reads it after the parser.
  EXPECT_THAT(parsed.mb_reserved_end,
              t::Eq(AddrOf(&mb1->info) + sizeof(Multiboot1Info)));

  // A string is read no further than `max_physical_addr`.
  mb1->mods[3].string = low_addr(mb1->last_cmdline);
  ASSERT_TRUE(parse());
  ASSERT_THAT(parsed.module_count, t::Eq(3));
  EXPECT_THAT(NameOf(parsed.modules[2]), t::Eq("def"));

  // A string that can't be mapped leaves its module without a name.
  g_env.fail_bootstrap_map_addr = AddrOf(mb1->rootfs_cmdline);
  ASSERT_TRUE(parse());
  ASSERT_THAT(parsed.module_count, t::Eq(3));
  EXPECT_THAT(NameOf(parsed.modules[0]), t::Eq(""));

  // A module list that can't be mapped, or that is larger than the largest
  // Multiboot structure, or that reaches past `max_physical_addr`, is left
  // out, but the rest of the boot information still counts.
  g_env.fail_bootstrap_map_addr = AddrOf(mb1->mods);
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.module_count, t::Eq(0));
  g_env.fail_bootstrap_map_addr = 0;
  mb1->info.mods_count = 0xFFFFFFFF;
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.module_count, t::Eq(0));
  mb1->info.mods_count = 4;
  mb1->info.mods_addr = low_addr(mb1->last_cmdline);
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.module_count, t::Eq(0));
  EXPECT_THAT(parsed.region_count, t::Eq(2));

  // Without the flag, the list is not read.
  mb1->info.mods_addr = low_addr(mb1->mods);
  mb1->info.flags = kMultiboot1FlagMemBasic;
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.module_count, t::Eq(0));
}

}  // namespace
}  // namespace protos
