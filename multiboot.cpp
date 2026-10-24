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

// Maps the fixed part of the Multiboot1 boot information at `info_phys`
// (below `max_physical_addr`) and returns it, or nullptr if it reaches past
// `max_physical_addr` or cannot be mapped.
static const Multiboot1Info* MapMultiboot1Info(
    const uintptr_t info_phys, const uintptr_t max_physical_addr) {
  constexpr int64_t kInfoSize = sizeof(Multiboot1Info);
  if ((max_physical_addr - info_phys) < kInfoSize ||
      !PagingMapBootstrapRange(info_phys, kInfoSize)) {
    return nullptr;
  }
  return reinterpret_cast<const Multiboot1Info*>(info_phys);
}

// Size of the Multiboot1 boot information up to the end of its framebuffer
// fields, its last fields.
constexpr int64_t kMultiboot1InfoWithFramebufferSize =
    kMultiboot1FramebufferInfoOffset + sizeof(Multiboot1FramebufferInfo);

// Maps the Multiboot1 boot information at `info_phys` (below
// `max_physical_addr`) up to the end of its framebuffer fields and returns
// those, or nullptr if they reach past `max_physical_addr` or cannot be mapped.
static const Multiboot1FramebufferInfo* MapMultiboot1FramebufferInfo(
    const uintptr_t info_phys, const uintptr_t max_physical_addr) {
  if ((max_physical_addr - info_phys) <
          static_cast<uintptr_t>(kMultiboot1InfoWithFramebufferSize) ||
      !PagingMapBootstrapRange(info_phys, kMultiboot1InfoWithFramebufferSize)) {
    return nullptr;
  }
  return reinterpret_cast<const Multiboot1FramebufferInfo*>(
      info_phys + kMultiboot1FramebufferInfoOffset);
}

// A framebuffer as a Multiboot2 framebuffer tag or the Multiboot1 framebuffer
// fields describe it.
struct FramebufferFields {
  uint64_t addr;
  uint32_t pitch;
  uint32_t width;
  uint32_t height;
  uint8_t bpp;
  uint8_t type;
  // Whether the description is long enough to hold `rgb`, the color
  // information of an RGB framebuffer.
  bool has_color_info;
  MultibootRgbColorInfo rgb;
};

// Returns the fields of a Multiboot2 framebuffer tag of at least
// `sizeof(Multiboot2TagFramebuffer)` bytes.
static FramebufferFields Multiboot2FramebufferFields(const Multiboot2Tag& tag) {
  const Multiboot2TagFramebuffer* const fb =
      reinterpret_cast<const Multiboot2TagFramebuffer*>(&tag);
  FramebufferFields fields = {};
  fields.addr = fb->framebuffer_addr;
  fields.pitch = fb->framebuffer_pitch;
  fields.width = fb->framebuffer_width;
  fields.height = fb->framebuffer_height;
  fields.bpp = fb->framebuffer_bpp;
  fields.type = fb->framebuffer_type;
  fields.has_color_info = tag.size >= sizeof(Multiboot2TagFramebufferRgb);
  if (fields.has_color_info) {
    fields.rgb =
        reinterpret_cast<const Multiboot2TagFramebufferRgb*>(&tag)->rgb;
  }
  return fields;
}

static FramebufferFields Multiboot1FramebufferFields(
    const Multiboot1FramebufferInfo& fb) {
  FramebufferFields fields = {};
  fields.addr = fb.framebuffer_addr;
  fields.pitch = fb.framebuffer_pitch;
  fields.width = fb.framebuffer_width;
  fields.height = fb.framebuffer_height;
  fields.bpp = fb.framebuffer_bpp;
  fields.type = fb.framebuffer_type;
  fields.has_color_info = true;
  fields.rgb = fb.rgb;
  return fields;
}

// Records `fb` in `*out_map` if it is an RGB framebuffer that the console
// could draw on: with color information, 24 or 32 bits per pixel, and an
// address, dimensions and pitch that are consistent and fit the identity map
// and `int`. Whether the console supports its color channels is left to
// `VgaInitFramebuffer`.
static void RecordFramebuffer(const FramebufferFields& fb,
                              MultibootMemoryMap* const out_map) {
  if (fb.type != kMultiboot2FramebufferTypeRgb || !fb.has_color_info ||
      fb.addr == 0 || fb.addr >= kIdentityMapLimit || fb.width == 0 ||
      fb.height == 0 || fb.pitch > static_cast<uint32_t>(INT32_MAX) ||
      fb.width > static_cast<uint32_t>(INT32_MAX) ||
      fb.height > static_cast<uint32_t>(INT32_MAX) ||
      (fb.bpp != 24 && fb.bpp != 32) ||
      static_cast<int64_t>(fb.pitch) <
          static_cast<int64_t>(fb.width) * (fb.bpp / 8)) {
    return;
  }
  out_map->fb_addr = fb.addr;
  out_map->fb_pitch = static_cast<int>(fb.pitch);
  out_map->fb_width = static_cast<int>(fb.width);
  out_map->fb_height = static_cast<int>(fb.height);
  out_map->fb_bpp = fb.bpp;
  out_map->fb_type = fb.type;
  out_map->fb_red_position = fb.rgb.red_field_position;
  out_map->fb_red_size = fb.rgb.red_mask_size;
  out_map->fb_green_position = fb.rgb.green_field_position;
  out_map->fb_green_size = fb.rgb.green_mask_size;
  out_map->fb_blue_position = fb.rgb.blue_field_position;
  out_map->fb_blue_size = fb.rgb.blue_mask_size;
}

// Returns true if `fb` is the VGA text buffer in text mode: the EGA text
// framebuffer at 0xB8000 that GRUB reports for it (and not, for example, the
// monochrome text buffer at 0xB0000, whose CRT controller is at 0x3B4).
static bool IsVgaTextFramebuffer(const FramebufferFields& fb) {
  constexpr uint64_t kVgaTextBufferAddr = 0xB8000;
  return fb.type == kMultiboot2FramebufferTypeEgaText &&
         fb.addr == kVgaTextBufferAddr;
}

// Returns true for the Multiboot2 tag types that only UEFI boot loaders pass.
static bool IsEfiTag(const uint32_t type) {
  return type == kMultiboot2TagEfi32 || type == kMultiboot2TagEfi64 ||
         type == kMultiboot2TagEfiMmap ||
         type == kMultiboot2TagEfiBootServices ||
         type == kMultiboot2TagEfi32ImageHandle ||
         type == kMultiboot2TagEfi64ImageHandle;
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
  out_map->fb_red_position = 0;
  out_map->fb_red_size = 0;
  out_map->fb_green_position = 0;
  out_map->fb_green_size = 0;
  out_map->fb_blue_position = 0;
  out_map->fb_blue_size = 0;

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
    const StatusOr<Multiboot2MappedInfo> mapped =
        Multiboot2MapInfo(info_phys, max_physical_addr, &MapBootstrapIdentity);
    if (!mapped.ok()) {
      return false;
    }
    out_map->mb_reserved_start = info_phys;
    out_map->mb_reserved_end = info_phys + mapped->total_size;

    constexpr int64_t kMmapTagSize = sizeof(Multiboot2TagMmap);
    constexpr int64_t kMmapEntrySize = sizeof(Multiboot2MmapEntry);
    constexpr int64_t kFbTagSize = sizeof(Multiboot2TagFramebuffer);
    for (Multiboot2TagIterator it(mapped->data, mapped->total_size); !it.Done();
         it.Next()) {
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
        RecordFramebuffer(Multiboot2FramebufferFields(tag), out_map);
      }
    }
  } else if (multiboot_magic == kMultiboot1Magic) {
    const Multiboot1Info* const info =
        MapMultiboot1Info(info_phys, max_physical_addr);
    if (info == nullptr) {
      return false;
    }
    out_map->mb_reserved_start = info_phys;
    out_map->mb_reserved_end = info_phys + sizeof(Multiboot1Info);
    if ((info->flags & kMultiboot1FlagFramebuffer) != 0) {
      const Multiboot1FramebufferInfo* const fb_info =
          MapMultiboot1FramebufferInfo(info_phys, max_physical_addr);
      if (fb_info != nullptr) {
        out_map->mb_reserved_end =
            info_phys + kMultiboot1InfoWithFramebufferSize;
        RecordFramebuffer(Multiboot1FramebufferFields(*fb_info), out_map);
      }
    }
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

StatusOr<Multiboot2MappedInfo> Multiboot2MapInfo(
    const uintptr_t info_addr, const uintptr_t max_physical_addr,
    const MultibootPhysicalMapper map) {
  constexpr int64_t kHeaderSize = sizeof(Multiboot2InfoHeader);
  if (info_addr == 0 || info_addr >= max_physical_addr ||
      (max_physical_addr - info_addr) < static_cast<uintptr_t>(kHeaderSize)) {
    return kStatusError;
  }
  const Multiboot2InfoHeader* const header =
      static_cast<const Multiboot2InfoHeader*>(map(info_addr, kHeaderSize));
  if (header == nullptr) {
    return kStatusError;
  }
  const int64_t total_size = header->total_size;
  if (total_size < kHeaderSize || total_size > kMaxMultibootStructureBytes ||
      (max_physical_addr - info_addr) < static_cast<uintptr_t>(total_size)) {
    return kStatusError;
  }
  const uint8_t* const info =
      static_cast<const uint8_t*>(map(info_addr, total_size));
  if (info == nullptr) {
    return kStatusError;
  }
  return Multiboot2MappedInfo{info, total_size};
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

bool MultibootHasVgaTextMode(const uint32_t multiboot_magic,
                             const uint64_t multiboot_info_addr,
                             const uintptr_t max_physical_addr) {
  if (multiboot_info_addr == 0 || multiboot_info_addr >= max_physical_addr) {
    return false;
  }
  const uintptr_t info_phys = multiboot_info_addr;
  if (multiboot_magic == kMultiboot2Magic) {
    const StatusOr<Multiboot2MappedInfo> mapped =
        Multiboot2MapInfo(info_phys, max_physical_addr, &MapBootstrapIdentity);
    if (!mapped.ok()) {
      return false;
    }
    constexpr int64_t kFbTagSize = sizeof(Multiboot2TagFramebuffer);
    bool efi_boot = false;
    for (Multiboot2TagIterator it(mapped->data, mapped->total_size); !it.Done();
         it.Next()) {
      const Multiboot2Tag& tag = it.tag();
      if (tag.type == kMultiboot2TagFramebuffer && tag.size >= kFbTagSize) {
        return IsVgaTextFramebuffer(Multiboot2FramebufferFields(tag));
      }
      efi_boot = efi_boot || IsEfiTag(tag.type);
    }
    return !efi_boot;
  }
  if (multiboot_magic == kMultiboot1Magic) {
    // Multiboot1 has no EFI tags, so its boot information without framebuffer
    // fields is taken to come from a BIOS boot. (Only a UEFI boot loader that
    // finds no framebuffer, such as GRUB on a headless UEFI machine, passes
    // that too.)
    const Multiboot1Info* const info =
        MapMultiboot1Info(info_phys, max_physical_addr);
    if (info == nullptr) {
      return false;
    }
    if ((info->flags & kMultiboot1FlagFramebuffer) == 0) {
      return true;
    }
    const Multiboot1FramebufferInfo* const fb_info =
        MapMultiboot1FramebufferInfo(info_phys, max_physical_addr);
    return fb_info != nullptr &&
           IsVgaTextFramebuffer(Multiboot1FramebufferFields(*fb_info));
  }
  return false;
}

}  // namespace protos
