#include "multiboot.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sys/mman.h>

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
  return !g_env.fail_bootstrap_map;
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
  buf.fb.size = sizeof(Multiboot2TagFramebuffer);
  buf.fb.framebuffer_addr = kFbAddr;
  buf.fb.framebuffer_pitch = 640 * 4;
  buf.fb.framebuffer_width = 640;
  buf.fb.framebuffer_height = 480;
  buf.fb.framebuffer_bpp = 32;
  buf.fb.framebuffer_type = kMultiboot2FramebufferTypeRgb;
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

  // Valid 32-bpp and 24-bpp framebuffer tags are accepted.
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.fb_addr, t::Eq(kFbAddr));
  EXPECT_THAT(parsed.fb_pitch, t::Eq(640 * 4));
  EXPECT_THAT(parsed.fb_width, t::Eq(640));
  EXPECT_THAT(parsed.fb_height, t::Eq(480));
  EXPECT_THAT(parsed.fb_bpp, t::Eq(32));

  buf.fb.framebuffer_bpp = 24;
  buf.fb.framebuffer_pitch = 640 * 3;
  ASSERT_TRUE(parse());
  EXPECT_THAT(parsed.fb_addr, t::Eq(kFbAddr));
  EXPECT_THAT(parsed.fb_pitch, t::Eq(640 * 3));
  EXPECT_THAT(parsed.fb_bpp, t::Eq(24));

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

  int64_t total_size = -1;
  EXPECT_THAT(Multiboot2MapInfo(info_addr, kIdentityMapLimit, &RecordingMapper,
                                &total_size),
              t::Eq(info));
  EXPECT_THAT(total_size, t::Eq(48));
  EXPECT_THAT(g_env.map_calls,
              t::ElementsAre(MapCall{info_addr, sizeof(Multiboot2InfoHeader)},
                             MapCall{info_addr, 48}));

  // The structure may end exactly at `max_physical_addr`.
  EXPECT_THAT(Multiboot2MapInfo(info_addr,         //
                                info_addr + 48,    //
                                &RecordingMapper,  //
                                &total_size),
              t::Eq(info));
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
    int64_t total_size = -1;
    const uint8_t* const mapped = Multiboot2MapInfo(
        addr, max_physical_addr, &RecordingMapper, &total_size);
    EXPECT_THAT(total_size, t::Eq(mapped == nullptr ? 0 : 48));
    return mapped;
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

}  // namespace
}  // namespace protos
