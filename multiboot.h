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
// Multiboot memory map entry type indicating usable RAM.
constexpr uint32_t kMemoryTypeAvailable = 1;
// Maximum number of memory map regions stored in `MultibootMemoryMap`
// (sized for large UEFI memory maps which often exceed 100+ descriptors).
constexpr int kMaxMemoryRegions = 512;

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
