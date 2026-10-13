#ifndef PROTOS_MULTIBOOT_H_
#define PROTOS_MULTIBOOT_H_

#include <cstdint>

namespace protos {

// Multiboot1 and Multiboot2 bootloader magic numbers passed in EAX.
constexpr uint32_t kMultiboot1Magic = 0x2BADB002;
constexpr uint32_t kMultiboot2Magic = 0x36D76289;
// Multiboot1 info `flags` bit 0: `mem_lower` and `mem_upper` fields are valid.
constexpr uint32_t kMultiboot1FlagMemBasic = 1u << 0;
// Multiboot1 info `flags` bit 6: `mmap_length` and `mmap_addr` fields are
// valid.
constexpr uint32_t kMultiboot1FlagMmap = 1u << 6;
// Multiboot2 tag types: the end tag that terminates the tag list, the memory
// map, the framebuffer, and copies of the ACPI 1.0 RSDP (old) and the ACPI 2.0+
// RSDP (new).
constexpr uint32_t kMultiboot2TagEnd = 0;
constexpr uint32_t kMultiboot2TagMmap = 6;
constexpr uint32_t kMultiboot2TagFramebuffer = 8;
constexpr uint32_t kMultiboot2TagAcpiOld = 14;
constexpr uint32_t kMultiboot2TagAcpiNew = 15;
// Multiboot2 framebuffer tag `framebuffer_type` of a direct-color (RGB)
// framebuffer.
constexpr uint8_t kMultiboot2FramebufferTypeRgb = 1;
// Multiboot memory map entry type indicating usable RAM.
constexpr uint32_t kMemoryTypeAvailable = 1;
// Largest Multiboot2 boot information structure, and largest Multiboot1 memory
// map, that the parsers accept.
constexpr int64_t kMaxMultibootStructureBytes = 4 * 1024 * 1024;
// Maximum number of memory map regions stored in `MultibootMemoryMap`
// (sized for large UEFI memory maps which often exceed 100+ descriptors).
constexpr int kMaxMemoryRegions = 512;

// Multiboot1 boot information, up to the memory map fields.
struct [[gnu::packed]] Multiboot1Info {
  uint32_t flags;
  uint32_t mem_lower;
  uint32_t mem_upper;
  uint32_t boot_device;
  uint32_t cmdline;
  uint32_t mods_count;
  uint32_t mods_addr;
  uint32_t syms[4];
  uint32_t mmap_length;
  uint32_t mmap_addr;
};
static_assert(sizeof(Multiboot1Info) == 52);

// Multiboot1 memory map entry. `size` does not count the `size` field itself,
// so consecutive entries are `size + 4` bytes apart.
struct [[gnu::packed]] Multiboot1MmapEntry {
  uint32_t size;
  uint64_t addr;
  uint64_t len;
  uint32_t type;
};
static_assert(sizeof(Multiboot1MmapEntry) == 24);

// Fixed header of the Multiboot2 boot information structure. The tags follow
// it, each starting at an 8-byte aligned offset.
struct [[gnu::packed]] Multiboot2InfoHeader {
  uint32_t total_size;
  uint32_t reserved;
};
static_assert(sizeof(Multiboot2InfoHeader) == 8);

// Header that every Multiboot2 tag starts with. `size` includes the header but
// not the padding up to the next tag.
struct [[gnu::packed]] Multiboot2Tag {
  uint32_t type;
  uint32_t size;
};
static_assert(sizeof(Multiboot2Tag) == 8);

// Multiboot2 memory map tag (`kMultiboot2TagMmap`), followed by entries that
// are `entry_size` bytes apart.
struct [[gnu::packed]] Multiboot2TagMmap {
  uint32_t type;
  uint32_t size;
  uint32_t entry_size;
  uint32_t entry_version;
};
static_assert(sizeof(Multiboot2TagMmap) == 16);

// Multiboot2 memory map entry.
struct [[gnu::packed]] Multiboot2MmapEntry {
  uint64_t addr;
  uint64_t len;
  uint32_t type;
  uint32_t reserved;
};
static_assert(sizeof(Multiboot2MmapEntry) == 24);

// Multiboot2 framebuffer tag (`kMultiboot2TagFramebuffer`), up to the color
// information that follows.
struct [[gnu::packed]] Multiboot2TagFramebuffer {
  uint32_t type;
  uint32_t size;
  uint64_t framebuffer_addr;
  uint32_t framebuffer_pitch;
  uint32_t framebuffer_width;
  uint32_t framebuffer_height;
  uint8_t framebuffer_bpp;
  uint8_t framebuffer_type;
  uint16_t reserved;
};
static_assert(sizeof(Multiboot2TagFramebuffer) == 32);

// Returns a pointer through which the `size` bytes of physical memory at
// `phys_addr` can be read, or nullptr if they cannot be mapped.
using MultibootPhysicalMapper = const void* (*)(uintptr_t phys_addr,
                                                int64_t size);

// Maps the Multiboot2 boot information structure at physical address
// `info_addr` with `map`: first its fixed header, then the `total_size` bytes
// that the header declares. Returns the mapped structure and stores its
// `total_size` in `*out_total_size`. Returns nullptr if `info_addr` is 0, if
// `map` fails, or if `total_size` is smaller than the header, larger than
// `kMaxMultibootStructureBytes`, or reaches past `max_physical_addr`.
const uint8_t* Multiboot2MapInfo(uintptr_t info_addr,
                                 uintptr_t max_physical_addr,
                                 MultibootPhysicalMapper map,
                                 int64_t* out_total_size);

// Walks the tags of a mapped Multiboot2 boot information structure of
// `total_size` bytes (see `Multiboot2MapInfo`):
//
//   for (Multiboot2TagIterator it(info, total_size); !it.Done(); it.Next()) {
//     const Multiboot2Tag& tag = it.tag();
//     ...
//   }
//
// The walk ends at the end tag or at the first malformed tag, one that is
// smaller than its header or extends past `total_size`, so every tag it yields
// lies entirely inside the structure.
class Multiboot2TagIterator {
 public:
  // Starts at the first tag of the structure at `info`, which must be at least
  // `sizeof(Multiboot2InfoHeader)` bytes long.
  Multiboot2TagIterator(const uint8_t* info, int64_t total_size);

  // Returns true once the walk has ended.
  bool Done() const { return offset_ < 0; }
  // The current tag and its byte offset from the start of the structure.
  // Only valid while `!Done()`.
  const Multiboot2Tag& tag() const;
  int64_t offset() const { return offset_; }
  // Moves to the next tag, at the next 8-byte aligned offset after the current
  // one.
  void Next();

 private:
  // Ends the walk unless a well-formed tag other than the end tag starts at
  // `offset_`.
  void CheckCurrentTag();

  const uint8_t* info_;
  int64_t total_size_;
  // Offset of the current tag, or -1 once the walk has ended.
  int64_t offset_;
};

// Normalized physical memory region from a Multiboot1 or Multiboot2 memory map.
struct MemoryRegion {
  uintptr_t base;
  int64_t length;
  uint32_t type;
};

// Parsed summary of the bootloader memory map, optional linear framebuffer, and
// the physical address ranges occupied by the Multiboot info structures.
struct MultibootMemoryMap {
  MemoryRegion regions[kMaxMemoryRegions];
  int region_count;
  int64_t total_ram_bytes;
  int64_t usable_ram_bytes;
  int64_t reserved_ram_bytes;
  uintptr_t mb_reserved_start;
  uintptr_t mb_reserved_end;
  uintptr_t mb1_mmap_reserved_start;
  uintptr_t mb1_mmap_reserved_end;
  uintptr_t fb_addr;
  int fb_pitch;
  int fb_width;
  int fb_height;
  int fb_bpp;
  int fb_type;
};

// Records a single memory map entry into `*out_map`, coalescing adjacent or
// overlapping regions of the same availability class and saturating byte
// totals at `INT64_MAX`. Returns false if `out_map` is null or if adding an
// uncoalesceable region would exceed `kMaxMemoryRegions`.
bool MultibootRecordMmapEntry(MultibootMemoryMap* out_map,  //
                              uint64_t raw_base,            //
                              uint64_t raw_length,          //
                              uint32_t type);

// Parses the Multiboot1 (`0x2BADB002`) or Multiboot2 (`0x36D76289`) boot
// information at physical address `multiboot_info_addr` (which must lie below
// `max_physical_addr`) into `*out_map`: the memory map, the Multiboot2
// framebuffer tag, and the physical ranges occupied by the Multiboot
// structures. Structures above the 64 MiB bootstrap identity map are mapped on
// demand with `PagingMapBootstrapRange`. Also logs the magic number and every
// memory map entry to the console ("[PMM] mmap entry: ..."); `PmmInit` calls
// this at boot. Returns true if a valid memory map with usable RAM was found
// without overflowing the region table.
bool MultibootParseMemoryMap(uint32_t multiboot_magic,
                             uint64_t multiboot_info_addr,
                             uintptr_t max_physical_addr,
                             MultibootMemoryMap* out_map);

// Same as `MultibootParseMemoryMap`, but without any console output.
bool MultibootParseMemoryMapQuiet(uint32_t multiboot_magic,
                                  uint64_t multiboot_info_addr,
                                  uintptr_t max_physical_addr,
                                  MultibootMemoryMap* out_map);

}  // namespace protos

#endif  // PROTOS_MULTIBOOT_H_
