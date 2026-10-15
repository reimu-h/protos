#include "multiboot.h"

#include <cstdint>

#include "console.h"
#include "paging.h"

namespace protos {

namespace {

static constexpr int64_t AlignUp(const int64_t value, const int64_t alignment) {
  return (value + alignment - 1) & ~(alignment - 1);
}

// MultibootPhysicalMapper that identity-maps the memory with
// PagingMapBootstrapRange().
static const void* MapBootstrapIdentity(const uintptr_t phys_addr,
                                        const int64_t size) {
  if (!PagingMapBootstrapRange(phys_addr, size)) {
    return nullptr;
  }
  return reinterpret_cast<const void*>(phys_addr);
}

static void LogMmapEntry(const uintptr_t base,  //
                         const int64_t length,  //
                         const uint32_t type) {
  ConsoleWrite("[PMM] mmap entry: base=");
  ConsoleWriteHex(base);
  ConsoleWrite(" len=");
  ConsoleWriteHex(length);
  ConsoleWrite(" type=");
  ConsoleWriteDec(type);
  ConsoleWrite("\n");
}

static constexpr int64_t kMaxInt64 = INT64_MAX;

static constexpr int64_t SaturatingAddInt64(const int64_t a, const int64_t b) {
  if (b <= 0) {
    return a;
  }
  if (a > kMaxInt64 - b) {
    return kMaxInt64;
  }
  return a + b;
}

static constexpr uintptr_t RegionEnd(const MemoryRegion& region) {
  const uint64_t len = static_cast<uint64_t>(region.length);
  if (region.base > UINTPTR_MAX - len) {
    return UINTPTR_MAX;
  }
  return region.base + static_cast<uintptr_t>(len);
}

static bool TryMergeRegions(MemoryRegion* const dst, const MemoryRegion& src) {
  const bool dst_avail = (dst->type == kMemoryTypeAvailable);
  const bool src_avail = (src.type == kMemoryTypeAvailable);
  if (dst_avail != src_avail) {
    return false;
  }
  const uintptr_t dst_start = dst->base;
  const uintptr_t dst_end = RegionEnd(*dst);
  const uintptr_t src_start = src.base;
  const uintptr_t src_end = RegionEnd(src);
  if (src_start > dst_end || dst_start > src_end) {
    return false;
  }
  const uintptr_t merged_start =
      (dst_start < src_start) ? dst_start : src_start;
  const uintptr_t merged_end = (dst_end > src_end) ? dst_end : src_end;
  const uint64_t merged_span = static_cast<uint64_t>(merged_end - merged_start);
  if (merged_span > static_cast<uint64_t>(kMaxInt64)) {
    return false;
  }
  dst->base = merged_start;
  dst->length = static_cast<int64_t>(merged_span);
  return true;
}

}  // namespace

static bool RecordMmapEntryImpl(MultibootMemoryMap* const out_map,  //
                                const uint64_t raw_base,            //
                                const uint64_t raw_length,          //
                                const uint32_t type,                //
                                const bool log_entry) {
  if (out_map == nullptr) {
    return false;
  }
  if (raw_length == 0) {
    return true;
  }

  const uint64_t clamped_total = (raw_length > static_cast<uint64_t>(kMaxInt64))
                                     ? static_cast<uint64_t>(kMaxInt64)
                                     : raw_length;
  const int64_t total_delta = static_cast<int64_t>(clamped_total);

  // The byte totals count RAM only: memory types 1 (available), 3 (ACPI
  // reclaimable), 4 (ACPI NVS) and 5 (defective). Every other type is reserved
  // address space that may be MMIO (QEMU, for example, reports a 12 GiB
  // reserved window at 0xFD00000000), so total = usable + reserved RAM.
  constexpr uint32_t kMemoryTypeAcpiReclaimable = 3;
  constexpr uint32_t kMemoryTypeAcpiNvs = 4;
  constexpr uint32_t kMemoryTypeDefective = 5;
  const bool is_ram =
      type == kMemoryTypeAvailable || type == kMemoryTypeAcpiReclaimable ||
      type == kMemoryTypeAcpiNvs || type == kMemoryTypeDefective;
  if (is_ram) {
    out_map->total_ram_bytes =
        SaturatingAddInt64(out_map->total_ram_bytes, total_delta);
    if (type == kMemoryTypeAvailable) {
      out_map->usable_ram_bytes =
          SaturatingAddInt64(out_map->usable_ram_bytes, total_delta);
    } else {
      out_map->reserved_ram_bytes =
          SaturatingAddInt64(out_map->reserved_ram_bytes, total_delta);
    }
  }

  if (raw_base >= UINTPTR_MAX) {
    return true;
  }
  const uintptr_t base = static_cast<uintptr_t>(raw_base);
  const uint64_t max_span = static_cast<uint64_t>(UINTPTR_MAX - base);
  uint64_t span = clamped_total;
  if (span > max_span) {
    span = max_span;
  }
  const int64_t length = static_cast<int64_t>(span);
  if (length <= 0) {
    return true;
  }

  const MemoryRegion incoming = {base, length, type};
  const bool incoming_avail = (type == kMemoryTypeAvailable);
  for (int i = out_map->region_count - 1; i >= 0; --i) {
    if (TryMergeRegions(&out_map->regions[i], incoming)) {
      for (int j = i - 1; j >= 0; --j) {
        if (TryMergeRegions(&out_map->regions[j], out_map->regions[i])) {
          for (int k = i + 1; k < out_map->region_count; ++k) {
            out_map->regions[k - 1] = out_map->regions[k];
          }
          --out_map->region_count;
          i = j;
          continue;
        }
        const bool prev_avail =
            (out_map->regions[j].type == kMemoryTypeAvailable);
        if (prev_avail != incoming_avail) {
          const uintptr_t prev_start = out_map->regions[j].base;
          const uintptr_t prev_end = RegionEnd(out_map->regions[j]);
          const uintptr_t merged_start = out_map->regions[i].base;
          const uintptr_t merged_end = RegionEnd(out_map->regions[i]);
          if (merged_start < prev_end && prev_start < merged_end) {
            break;
          }
        }
      }
      if (log_entry) {
        LogMmapEntry(base, length, type);
      }
      return true;
    }
    const bool cur_avail = (out_map->regions[i].type == kMemoryTypeAvailable);
    if (cur_avail != incoming_avail) {
      const uintptr_t cur_start = out_map->regions[i].base;
      const uintptr_t cur_end = RegionEnd(out_map->regions[i]);
      const uintptr_t inc_end = RegionEnd(incoming);
      if (base < cur_end && cur_start < inc_end) {
        break;
      }
    }
  }

  if (out_map->region_count >= kMaxMemoryRegions) {
    return false;
  }

  out_map->regions[out_map->region_count] = incoming;
  ++out_map->region_count;

  if (log_entry) {
    LogMmapEntry(base, length, type);
  }
  return true;
}

static bool ParseMemoryMapImpl(const uint32_t multiboot_magic,
                               const uint64_t multiboot_info_addr,
                               const uintptr_t max_physical_addr,
                               MultibootMemoryMap* const out_map,
                               const bool log_boot) {
  if (out_map == nullptr) {
    return false;
  }

  out_map->region_count = 0;
  out_map->total_ram_bytes = 0;
  out_map->usable_ram_bytes = 0;
  out_map->reserved_ram_bytes = 0;
  out_map->mb_reserved_start = 0;
  out_map->mb_reserved_end = 0;
  out_map->mb1_mmap_reserved_start = 0;
  out_map->mb1_mmap_reserved_end = 0;
  out_map->fb_addr = 0;
  out_map->fb_pitch = 0;
  out_map->fb_width = 0;
  out_map->fb_height = 0;
  out_map->fb_bpp = 0;
  out_map->fb_type = 0;

  if (log_boot) {
    ConsoleWrite("[PMM] Multiboot magic=");
    ConsoleWriteHex(multiboot_magic);
    ConsoleWrite(" info_addr=");
    ConsoleWriteHex(multiboot_info_addr);
    ConsoleWrite("\n");
  }

  if (multiboot_info_addr == 0 || multiboot_info_addr >= max_physical_addr) {
    return false;
  }
  const uintptr_t info_phys = multiboot_info_addr;

  if (multiboot_magic == kMultiboot2Magic) {
    int64_t total_size = 0;
    const uint8_t* const info = Multiboot2MapInfo(
        info_phys, max_physical_addr, &MapBootstrapIdentity, &total_size);
    if (info == nullptr) {
      return false;
    }
    out_map->mb_reserved_start = info_phys;
    out_map->mb_reserved_end = info_phys + total_size;

    constexpr int64_t kMmapTagSize = sizeof(Multiboot2TagMmap);
    constexpr int64_t kMmapEntrySize = sizeof(Multiboot2MmapEntry);
    constexpr int64_t kFbTagSize = sizeof(Multiboot2TagFramebuffer);
    for (Multiboot2TagIterator it(info, total_size); !it.Done(); it.Next()) {
      const Multiboot2Tag& tag = it.tag();
      const int64_t tag_size = tag.size;
      if (tag.type == kMultiboot2TagMmap && tag_size >= kMmapTagSize) {
        const Multiboot2TagMmap* const mmap_tag =
            reinterpret_cast<const Multiboot2TagMmap*>(&tag);
        const int64_t entry_size = mmap_tag->entry_size;
        if (entry_size >= kMmapEntrySize) {
          int64_t entry_offset = kMmapTagSize;
          while (entry_offset <= tag_size - entry_size) {
            const Multiboot2MmapEntry* const entry =
                reinterpret_cast<const Multiboot2MmapEntry*>(
                    reinterpret_cast<uintptr_t>(mmap_tag) + entry_offset);
            if (!RecordMmapEntryImpl(out_map,      //
                                     entry->addr,  //
                                     entry->len,   //
                                     entry->type,  //
                                     log_boot)) {
              return false;
            }
            entry_offset += entry_size;
          }
        }
      } else if (tag.type == kMultiboot2TagFramebuffer &&
                 tag_size >= kFbTagSize) {
        const Multiboot2TagFramebuffer* const fb_tag =
            reinterpret_cast<const Multiboot2TagFramebuffer*>(&tag);
        if (fb_tag->framebuffer_type == kMultiboot2FramebufferTypeRgb &&
            fb_tag->framebuffer_addr != 0 &&
            fb_tag->framebuffer_addr < kIdentityMapLimit &&
            fb_tag->framebuffer_width > 0 && fb_tag->framebuffer_height > 0 &&
            fb_tag->framebuffer_pitch <= static_cast<uint32_t>(INT32_MAX) &&
            fb_tag->framebuffer_width <= static_cast<uint32_t>(INT32_MAX) &&
            fb_tag->framebuffer_height <= static_cast<uint32_t>(INT32_MAX) &&
            (fb_tag->framebuffer_bpp == 24 || fb_tag->framebuffer_bpp == 32) &&
            static_cast<int64_t>(fb_tag->framebuffer_pitch) >=
                static_cast<int64_t>(fb_tag->framebuffer_width) *
                    (fb_tag->framebuffer_bpp / 8)) {
          out_map->fb_addr = fb_tag->framebuffer_addr;
          out_map->fb_pitch = fb_tag->framebuffer_pitch;
          out_map->fb_width = fb_tag->framebuffer_width;
          out_map->fb_height = fb_tag->framebuffer_height;
          out_map->fb_bpp = fb_tag->framebuffer_bpp;
          out_map->fb_type = fb_tag->framebuffer_type;
        }
      }
    }
  } else if (multiboot_magic == kMultiboot1Magic) {
    constexpr int64_t kInfoSize = sizeof(Multiboot1Info);
    if ((max_physical_addr - info_phys) < kInfoSize ||
        !PagingMapBootstrapRange(info_phys, kInfoSize)) {
      return false;
    }
    const Multiboot1Info* const info =
        reinterpret_cast<const Multiboot1Info*>(info_phys);
    out_map->mb_reserved_start = info_phys;
    out_map->mb_reserved_end = info_phys + kInfoSize;
    if ((info->flags & kMultiboot1FlagMmap) != 0) {
      const int64_t mmap_length = info->mmap_length;
      if (info->mmap_addr == 0 || mmap_length <= 0 ||
          mmap_length > kMaxMultibootStructureBytes) {
        return false;
      }
      const uintptr_t mmap_phys = info->mmap_addr;
      if (mmap_phys >= max_physical_addr ||
          (max_physical_addr - mmap_phys) <
              static_cast<uintptr_t>(mmap_length) ||
          !PagingMapBootstrapRange(mmap_phys, mmap_length)) {
        return false;
      }
      out_map->mb1_mmap_reserved_start = mmap_phys;
      out_map->mb1_mmap_reserved_end = mmap_phys + mmap_length;

      int64_t entry_offset = 0;
      constexpr int64_t kMinEntryStructSize = sizeof(Multiboot1MmapEntry);
      constexpr int64_t kSizeFieldBytes = sizeof(uint32_t);
      while (entry_offset <= mmap_length - kMinEntryStructSize) {
        const Multiboot1MmapEntry* const entry =
            reinterpret_cast<const Multiboot1MmapEntry*>(mmap_phys +
                                                         entry_offset);
        const int64_t entry_step =
            static_cast<int64_t>(entry->size) + kSizeFieldBytes;
        if (entry->size < 20 || entry_step > mmap_length - entry_offset) {
          break;
        }
        if (!RecordMmapEntryImpl(out_map,      //
                                 entry->addr,  //
                                 entry->len,   //
                                 entry->type,  //
                                 log_boot)) {
          return false;
        }
        entry_offset += entry_step;
      }
    } else if ((info->flags & kMultiboot1FlagMemBasic) != 0) {
      if (info->mem_lower > 0) {
        const uint64_t lower_bytes =
            static_cast<uint64_t>(info->mem_lower) * 1024ULL;
        if (!RecordMmapEntryImpl(out_map,               //
                                 0ULL,                  //
                                 lower_bytes,           //
                                 kMemoryTypeAvailable,  //
                                 log_boot)) {
          return false;
        }
      }
      if (info->mem_upper > 0) {
        constexpr uint64_t kUpperMemoryBase = 0x100000ULL;
        const uint64_t upper_end =
            kUpperMemoryBase + static_cast<uint64_t>(info->mem_upper) * 1024ULL;
        if (!RecordMmapEntryImpl(out_map,                       //
                                 kUpperMemoryBase,              //
                                 upper_end - kUpperMemoryBase,  //
                                 kMemoryTypeAvailable,          //
                                 log_boot)) {
          return false;
        }
      }
    } else {
      return false;
    }
  } else {
    return false;
  }

  return out_map->region_count > 0 && out_map->usable_ram_bytes > 0;
}

const uint8_t* Multiboot2MapInfo(const uintptr_t info_addr,
                                 const uintptr_t max_physical_addr,
                                 const MultibootPhysicalMapper map,
                                 int64_t* const out_total_size) {
  *out_total_size = 0;
  constexpr int64_t kHeaderSize = sizeof(Multiboot2InfoHeader);
  if (info_addr == 0 || info_addr >= max_physical_addr ||
      (max_physical_addr - info_addr) < static_cast<uintptr_t>(kHeaderSize)) {
    return nullptr;
  }
  const Multiboot2InfoHeader* const header =
      static_cast<const Multiboot2InfoHeader*>(map(info_addr, kHeaderSize));
  if (header == nullptr) {
    return nullptr;
  }
  const int64_t total_size = header->total_size;
  if (total_size < kHeaderSize || total_size > kMaxMultibootStructureBytes ||
      (max_physical_addr - info_addr) < static_cast<uintptr_t>(total_size)) {
    return nullptr;
  }
  const uint8_t* const info =
      static_cast<const uint8_t*>(map(info_addr, total_size));
  if (info == nullptr) {
    return nullptr;
  }
  *out_total_size = total_size;
  return info;
}

Multiboot2TagIterator::Multiboot2TagIterator(const uint8_t* const info,
                                             const int64_t total_size)
    : info_(info),
      total_size_(total_size),
      offset_(sizeof(Multiboot2InfoHeader)) {
  CheckCurrentTag();
}

const Multiboot2Tag& Multiboot2TagIterator::tag() const {
  return *reinterpret_cast<const Multiboot2Tag*>(info_ + offset_);
}

void Multiboot2TagIterator::Next() {
  if (Done()) {
    return;
  }
  offset_ = AlignUp(offset_ + tag().size, 8);
  CheckCurrentTag();
}

void Multiboot2TagIterator::CheckCurrentTag() {
  constexpr int64_t kTagHeaderSize = sizeof(Multiboot2Tag);
  if (offset_ > total_size_ - kTagHeaderSize) {
    offset_ = -1;
    return;
  }
  const Multiboot2Tag& current = tag();
  const int64_t tag_size = current.size;
  if (current.type == kMultiboot2TagEnd || tag_size < kTagHeaderSize ||
      tag_size > total_size_ - offset_) {
    offset_ = -1;
  }
}

bool MultibootRecordMmapEntry(MultibootMemoryMap* const out_map,  //
                              const uint64_t raw_base,            //
                              const uint64_t raw_length,          //
                              const uint32_t type) {
  return RecordMmapEntryImpl(out_map, raw_base, raw_length, type, false);
}

bool MultibootParseMemoryMapQuiet(const uint32_t multiboot_magic,
                                  const uint64_t multiboot_info_addr,
                                  const uintptr_t max_physical_addr,
                                  MultibootMemoryMap* const out_map) {
  return ParseMemoryMapImpl(multiboot_magic,      //
                            multiboot_info_addr,  //
                            max_physical_addr,    //
                            out_map,              //
                            false);
}

// pmm_test substitutes a fake for this function, so host builds define it as a
// weak symbol. Kernel builds define it normally, where a second definition is a
// link error.
#if __STDC_HOSTED__
[[gnu::weak]]
#endif
bool MultibootParseMemoryMap(const uint32_t multiboot_magic,
                             const uint64_t multiboot_info_addr,
                             const uintptr_t max_physical_addr,
                             MultibootMemoryMap* const out_map) {
  return ParseMemoryMapImpl(multiboot_magic,      //
                            multiboot_info_addr,  //
                            max_physical_addr,    //
                            out_map,              //
                            true);
}

}  // namespace protos
