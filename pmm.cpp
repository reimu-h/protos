#include "pmm.h"

#include <cstdint>

#include "check.h"
#include "multiboot.h"
#include "paging.h"
#include "rbtree.h"
#include "spinlock.h"
#include "uart.h"
#include "vga.h"

extern "C" {
extern const uint8_t g_kernel_start[];
extern const uint8_t g_kernel_end[];
}

namespace protos {

namespace {

// Intrusive node stored at the start of the first 4 KiB frame of each
// contiguous free physical frame run:
//
//   +-----------------------+--------------------------------------------+
//   | FreeRunNode           | Remaining bytes of free run                |
//   | (sizeof(FreeRunNode)) | (frame_count * kPageSize -                 |
//   |                       |  sizeof(FreeRunNode))                      |
//   +-----------------------+--------------------------------------------+
//   ^
//   run base physical address (4 KiB page-aligned)
//
// Free runs are stored in an address-ordered Red-Black Tree (`g_free_run_tree`)
// augmented with `max_subtree_frames` (the largest `frame_count` in the node's
// subtree) to support O(log R) lowest-address first-fit allocation and O(log R)
// neighbor lookup for immediate physical coalescing.
struct FreeRunNode {
  int64_t frame_count;
  int64_t max_subtree_frames;
  RbNode rb_node;
};

static_assert(sizeof(FreeRunNode) <= kPageSize);

struct FreeRunRbTraits {
  static uintptr_t GetKey(const FreeRunNode& node) {
    return reinterpret_cast<uintptr_t>(&node);
  }

  static bool Less(const uintptr_t a, const uintptr_t b) { return a < b; }

  static void UpdateAugment(FreeRunNode* const node,
                            const FreeRunNode* const left,
                            const FreeRunNode* const right) {
    int64_t max_frames = node->frame_count;
    if (left != nullptr && left->max_subtree_frames > max_frames) {
      max_frames = left->max_subtree_frames;
    }
    if (right != nullptr && right->max_subtree_frames > max_frames) {
      max_frames = right->max_subtree_frames;
    }
    node->max_subtree_frames = max_frames;
  }
};

using FreeRunTree = RbTree<FreeRunNode, &FreeRunNode::rb_node, FreeRunRbTraits>;

struct PhysInterval {
  uintptr_t start;
  uintptr_t end;
};

// Maximum number of disjoint usable physical intervals after carving out
// non-available regions, lower memory, the kernel image, Multiboot structures,
// and the framebuffer from at most `kMaxMemoryRegions` entries.
constexpr int kMaxUsableIntervals = kMaxMemoryRegions * 2 + 16;

// Spinlock protecting `g_free_run_tree` and `g_pmm_free_frames` across
// concurrent multi-CPU calls. The remaining state is written only by
// `PmmInit` (before other CPUs start) and `PmmResetForTest`.
IrqSpinLock g_pmm_lock;

// Intrusive Red-Black Tree containing all currently FREE contiguous physical
// frame runs across usable RAM, keyed by each run's base physical address
// (`uintptr_t`, where the `FreeRunNode` is stored in the run's first 4 KiB
// frame) and augmented with `max_subtree_frames` (the maximum `frame_count`
// among all free runs in the node's subtree). Allocated frame ranges are
// removed (or split off) from this tree and re-inserted (and coalesced with
// adjacent free runs) when freed.
FreeRunTree g_free_run_tree;
PhysInterval g_usable_intervals[kMaxUsableIntervals];
int g_usable_interval_count = 0;
uintptr_t g_max_managed_phys_addr = 0;
int64_t g_max_frames = 0;
int64_t g_pmm_free_frames = 0;
int64_t g_pmm_total_usable_frames = 0;
bool g_pmm_initialized = false;
MultibootMemoryMap g_memory_map = {};

// Rounds `value` up to the nearest multiple of `alignment` (power of two).
static constexpr uintptr_t AlignUp(const uintptr_t value,
                                   const int64_t alignment) {
  const uintptr_t align = alignment;
  return (value + align - 1) & ~(align - 1);
}

// Rounds `value` down to the nearest multiple of `alignment` (power of two).
static constexpr uintptr_t AlignDown(const uintptr_t value,
                                     const int64_t alignment) {
  const uintptr_t align = alignment;
  return value & ~(align - 1);
}

static uintptr_t KernelStartAddr() {
  return reinterpret_cast<uintptr_t>(g_kernel_start);
}

static uintptr_t KernelEndAddr() {
  return reinterpret_cast<uintptr_t>(g_kernel_end);
}

static void ResetPmmState() {
  g_free_run_tree.Clear();
  g_usable_interval_count = 0;
  g_max_managed_phys_addr = 0;
  g_max_frames = 0;
  g_pmm_free_frames = 0;
  g_pmm_total_usable_frames = 0;
  g_pmm_initialized = false;
  g_memory_map = {};
}

static uintptr_t RunBaseAddress(const FreeRunNode* const node) {
  DCHECK(node != nullptr);
  const uintptr_t addr = reinterpret_cast<uintptr_t>(node);
  DCHECK((addr & (kPageSize - 1)) == 0);
  return addr;
}

static uintptr_t RunEndAddress(const FreeRunNode* const node) {
  DCHECK(node != nullptr);
  DCHECK(node->frame_count > 0);
  const uintptr_t base_addr = RunBaseAddress(node);
  DCHECK(static_cast<uintptr_t>(node->frame_count) <=
         (UINTPTR_MAX - base_addr) / kPageSize);
  return base_addr + node->frame_count * kPageSize;
}

static bool FreeRunsAreAdjacent(const FreeRunNode* const first,
                                const FreeRunNode* const second) {
  DCHECK(first != nullptr);
  DCHECK(second != nullptr);
  return RunEndAddress(first) == RunBaseAddress(second);
}

static FreeRunNode* FreeRunTreeInsert(const uintptr_t base_addr,
                                      const int64_t frame_count) {
  DCHECK(base_addr >= kLowerMemoryLimit);
  DCHECK((base_addr & (kPageSize - 1)) == 0);
  DCHECK(frame_count > 0);
  DCHECK(base_addr < g_max_managed_phys_addr);
  DCHECK(static_cast<uintptr_t>(frame_count) <=
         (g_max_managed_phys_addr - base_addr) / kPageSize);

  FreeRunNode* const node = reinterpret_cast<FreeRunNode*>(base_addr);
  node->frame_count = frame_count;
  node->max_subtree_frames = frame_count;
  node->rb_node = {};

  const bool inserted = g_free_run_tree.Insert(node);
  DCHECK(inserted);
  g_pmm_free_frames += frame_count;
  return node;
}

static void FreeRunTreeRemove(FreeRunNode* const node) {
  DCHECK(node != nullptr);
  DCHECK(node->frame_count > 0);
  DCHECK(g_pmm_free_frames >= node->frame_count);

  const int64_t count = node->frame_count;
  g_free_run_tree.Erase(node);
  node->frame_count = 0;
  node->max_subtree_frames = 0;
  node->rb_node = {};
  g_pmm_free_frames -= count;
}

// Inserts a validated free run `[base_addr, base_addr + frame_count *
// kPageSize)` into `g_free_run_tree` and immediately merges it in O(log R) time
// with its in-order predecessor (`Prev`) and/or successor (`Next`) if they are
// physically contiguous.
static FreeRunNode* PmmCoalesceFreeRun(const uintptr_t base_addr,
                                       const int64_t frame_count) {
  DCHECK(base_addr >= kLowerMemoryLimit);
  DCHECK((base_addr & (kPageSize - 1)) == 0);
  DCHECK(frame_count > 0);
  DCHECK(base_addr < g_max_managed_phys_addr);
  DCHECK(static_cast<uintptr_t>(frame_count) <=
         (g_max_managed_phys_addr - base_addr) / kPageSize);

  // Freeing frames that are already free (a double free or an overlapping
  // range) would corrupt the tree and double-count frames, so these checks stay
  // on in every build.
  const uintptr_t end_addr = base_addr + frame_count * kPageSize;
  const FreeRunNode* const next_check = g_free_run_tree.LowerBound(base_addr);
  const FreeRunNode* const prev_check = (next_check != nullptr)
                                            ? FreeRunTree::Prev(next_check)
                                            : g_free_run_tree.Last();
  CHECK(prev_check == nullptr || RunEndAddress(prev_check) <= base_addr);
  CHECK(next_check == nullptr || RunBaseAddress(next_check) >= end_addr);

  FreeRunNode* run = FreeRunTreeInsert(base_addr, frame_count);

  FreeRunNode* const next_run = FreeRunTree::Next(run);
  FreeRunNode* const prev_run = FreeRunTree::Prev(run);

  if (next_run != nullptr && FreeRunsAreAdjacent(run, next_run)) {
    const int64_t next_frames = next_run->frame_count;
    FreeRunTreeRemove(next_run);
    run->frame_count += next_frames;
    g_pmm_free_frames += next_frames;
    g_free_run_tree.PropagateAugment(run);
  }

  if (prev_run != nullptr && FreeRunsAreAdjacent(prev_run, run)) {
    const int64_t run_frames = run->frame_count;
    FreeRunTreeRemove(run);
    prev_run->frame_count += run_frames;
    g_pmm_free_frames += run_frames;
    g_free_run_tree.PropagateAugment(prev_run);
    run = prev_run;
  }

  return run;
}

// Adds the interior page-aligned range of `[base, base + length)` above
// `kLowerMemoryLimit` into `g_usable_intervals`, keeping the table sorted and
// merging overlapping or contiguous intervals.
static void AddAvailableRegion(const uintptr_t base, const int64_t length) {
  if (length < kPageSize || base >= kMaxCanonicalIdentityAddress) {
    return;
  }
  const uintptr_t max_len = kMaxCanonicalIdentityAddress - base;
  const uintptr_t raw_len = length;
  const uintptr_t clamped_len = (raw_len > max_len) ? max_len : raw_len;
  uintptr_t start = AlignUp(base, kPageSize);
  const uintptr_t end = AlignDown(base + clamped_len, kPageSize);
  if (start < kLowerMemoryLimit) {
    start = kLowerMemoryLimit;
  }
  if (start >= end) {
    return;
  }

  int insert_pos = 0;
  while (insert_pos < g_usable_interval_count &&
         g_usable_intervals[insert_pos].end < start) {
    ++insert_pos;
  }

  uintptr_t merged_start = start;
  uintptr_t merged_end = end;
  int merge_end = insert_pos;
  while (merge_end < g_usable_interval_count &&
         g_usable_intervals[merge_end].start <= merged_end) {
    if (g_usable_intervals[merge_end].start < merged_start) {
      merged_start = g_usable_intervals[merge_end].start;
    }
    if (g_usable_intervals[merge_end].end > merged_end) {
      merged_end = g_usable_intervals[merge_end].end;
    }
    ++merge_end;
  }

  const int merged_count = merge_end - insert_pos;
  if (merged_count == 0) {
    CHECK(g_usable_interval_count < kMaxUsableIntervals);
    for (int i = g_usable_interval_count; i > insert_pos; --i) {
      g_usable_intervals[i] = g_usable_intervals[i - 1];
    }
    g_usable_intervals[insert_pos] = {merged_start, merged_end};
    ++g_usable_interval_count;
  } else {
    g_usable_intervals[insert_pos] = {merged_start, merged_end};
    const int remove_extra = merged_count - 1;
    if (remove_extra > 0) {
      for (int i = insert_pos + 1; i < g_usable_interval_count - remove_extra;
           ++i) {
        g_usable_intervals[i] = g_usable_intervals[i + remove_extra];
      }
      g_usable_interval_count -= remove_extra;
    }
  }
}

// Carves all 4 KiB pages overlapping `[base, base + length)` out of
// `g_usable_intervals` (rounding start DOWN and end UP to page boundaries so
// any partially touched page is conservatively excluded).
static void CarveExcludedRange(const uintptr_t base, const int64_t length) {
  if (length <= 0 || base >= kMaxCanonicalIdentityAddress) {
    return;
  }
  const uintptr_t max_len = kMaxCanonicalIdentityAddress - base;
  const uintptr_t raw_len = length;
  const uintptr_t clamped_len = (raw_len > max_len) ? max_len : raw_len;
  const uintptr_t res_start = AlignDown(base, kPageSize);
  const uintptr_t res_end = AlignUp(base + clamped_len, kPageSize);
  if (res_start >= res_end) {
    return;
  }

  int i = 0;
  while (i < g_usable_interval_count) {
    const uintptr_t u_start = g_usable_intervals[i].start;
    const uintptr_t u_end = g_usable_intervals[i].end;

    if (res_end <= u_start || res_start >= u_end) {
      ++i;
      continue;
    }

    if (res_start <= u_start && res_end >= u_end) {
      for (int j = i; j < g_usable_interval_count - 1; ++j) {
        g_usable_intervals[j] = g_usable_intervals[j + 1];
      }
      --g_usable_interval_count;
      continue;
    }

    if (res_start <= u_start) {
      g_usable_intervals[i].start = res_end;
      ++i;
      continue;
    }

    if (res_end >= u_end) {
      g_usable_intervals[i].end = res_start;
      ++i;
      continue;
    }

    CHECK(g_usable_interval_count < kMaxUsableIntervals);
    for (int j = g_usable_interval_count; j > i + 1; --j) {
      g_usable_intervals[j] = g_usable_intervals[j - 1];
    }
    g_usable_intervals[i].end = res_start;
    g_usable_intervals[i + 1] = {res_end, u_end};
    ++g_usable_interval_count;
    i += 2;
  }
}

static void ConsoleWrite(const char* const str) {
  UartWrite(str);
  VgaWrite(str);
}

static void ConsoleWriteHex(const uint64_t value) {
  UartWriteHex(value);
  VgaWriteHex(value);
}

static void ConsoleWriteDec(const uint64_t value) {
  UartWriteDec(value);
  VgaWriteDec(value);
}

static bool PmmRangeIsValidUsableRamLocked(const uintptr_t addr,
                                           const int64_t size) {
  if (!g_pmm_initialized || size <= 0 || addr < kLowerMemoryLimit ||
      addr >= g_max_managed_phys_addr ||
      (g_max_managed_phys_addr - addr) < static_cast<uintptr_t>(size)) {
    return false;
  }

  const uintptr_t end_addr = addr + size;
  int low = 0;
  int high = g_usable_interval_count - 1;
  while (low <= high) {
    const int mid = low + (high - low) / 2;
    const PhysInterval& interval = g_usable_intervals[mid];
    if (addr < interval.start) {
      high = mid - 1;
    } else if (addr >= interval.end) {
      low = mid + 1;
    } else {
      return end_addr <= interval.end;
    }
  }
  return false;
}

static uintptr_t PmmAllocFramesLocked(const int64_t count) {
  DCHECK(g_pmm_initialized);
  DCHECK(count > 0);
  if (!g_pmm_initialized || count <= 0 || count > g_pmm_free_frames) {
    return 0;
  }

  FreeRunNode* const candidate = g_free_run_tree.FindFirstAugmented(
      [count](const FreeRunNode& node) {
        return node.max_subtree_frames >= count;
      },
      [count](const FreeRunNode& node) { return node.frame_count >= count; });
  if (candidate == nullptr) {
    return 0;
  }

  const uintptr_t alloc_addr = RunBaseAddress(candidate);
  const int64_t total_frames = candidate->frame_count;
  DCHECK(total_frames >= count);
  FreeRunTreeRemove(candidate);

  if (total_frames > count) {
    const uintptr_t remainder_addr = alloc_addr + count * kPageSize;
    const int64_t remainder_frames = total_frames - count;
    FreeRunTreeInsert(remainder_addr, remainder_frames);
  }

  return alloc_addr;
}

static void PmmFreeFramesLocked(const uintptr_t base_addr,
                                const int64_t count) {
  DCHECK(g_pmm_initialized);
  DCHECK(count > 0);
  DCHECK(count <= INT64_MAX / kPageSize);
  DCHECK(base_addr >= kLowerMemoryLimit);
  DCHECK((base_addr & (kPageSize - 1)) == 0);
  DCHECK(base_addr < g_max_managed_phys_addr);
  DCHECK(static_cast<uintptr_t>(count) <=
         (g_max_managed_phys_addr - base_addr) / kPageSize);
  const int64_t byte_size = count * kPageSize;
  DCHECK(PmmRangeIsValidUsableRamLocked(base_addr, byte_size));

  PmmCoalesceFreeRun(base_addr, count);
}

// Fills `g_usable_intervals` with the available regions of `memory_map` minus
// its non-available regions, lower memory, the kernel image, the Multiboot
// information structures, and the framebuffer. Multiboot modules and the
// Multiboot1 command line and boot loader name strings are not carved out,
// because `MultibootMemoryMap` does not record them (no modules are loaded).
static void CollectUsableRegions(const MultibootMemoryMap& memory_map) {
  for (int i = 0; i < memory_map.region_count; ++i) {
    if (memory_map.regions[i].type == kMemoryTypeAvailable) {
      AddAvailableRegion(memory_map.regions[i].base,
                         memory_map.regions[i].length);
    }
  }

  for (int i = 0; i < memory_map.region_count; ++i) {
    if (memory_map.regions[i].type != kMemoryTypeAvailable) {
      CarveExcludedRange(memory_map.regions[i].base,
                         memory_map.regions[i].length);
    }
  }

  CarveExcludedRange(0, kLowerMemoryLimit);

  const uintptr_t kernel_start = KernelStartAddr();
  const uintptr_t kernel_end = KernelEndAddr();
  if (kernel_end > kernel_start) {
    CarveExcludedRange(kernel_start, kernel_end - kernel_start);
  }

  if (memory_map.mb_reserved_end > memory_map.mb_reserved_start) {
    CarveExcludedRange(
        memory_map.mb_reserved_start,
        memory_map.mb_reserved_end - memory_map.mb_reserved_start);
  }
  if (memory_map.mb1_mmap_reserved_end > memory_map.mb1_mmap_reserved_start) {
    CarveExcludedRange(
        memory_map.mb1_mmap_reserved_start,
        memory_map.mb1_mmap_reserved_end - memory_map.mb1_mmap_reserved_start);
  }
  if (memory_map.fb_addr != 0 &&
      memory_map.fb_addr < kMaxCanonicalIdentityAddress &&
      memory_map.fb_pitch > 0 && memory_map.fb_height > 0 &&
      static_cast<int64_t>(memory_map.fb_pitch) <=
          INT64_MAX / static_cast<int64_t>(memory_map.fb_height)) {
    const int64_t fb_bytes =
        static_cast<int64_t>(memory_map.fb_pitch) * memory_map.fb_height;
    CarveExcludedRange(memory_map.fb_addr, fb_bytes);
  }
}

}  // namespace

void PmmInit(const uint32_t multiboot_magic,
             const uint64_t multiboot_info_addr) {
  ResetPmmState();

  CHECK(MultibootParseMemoryMap(multiboot_magic,               //
                                multiboot_info_addr,           //
                                kMaxCanonicalIdentityAddress,  //
                                &g_memory_map));

  if (g_memory_map.fb_addr != 0) {
    VgaInitFramebuffer(g_memory_map.fb_addr,    //
                       g_memory_map.fb_pitch,   //
                       g_memory_map.fb_width,   //
                       g_memory_map.fb_height,  //
                       g_memory_map.fb_bpp);
    ConsoleWrite("[PMM] Framebuffer: addr=");
    ConsoleWriteHex(g_memory_map.fb_addr);
    ConsoleWrite(" width=");
    ConsoleWriteDec(g_memory_map.fb_width);
    ConsoleWrite(" height=");
    ConsoleWriteDec(g_memory_map.fb_height);
    ConsoleWrite(" bpp=");
    ConsoleWriteDec(g_memory_map.fb_bpp);
    ConsoleWrite("\n");
  }

  const uintptr_t kernel_start = KernelStartAddr();
  const uintptr_t kernel_end = KernelEndAddr();
  CollectUsableRegions(g_memory_map);

  CHECK(g_usable_interval_count > 0);

  // Intervals are non-empty and start at or above `kLowerMemoryLimit`.
  const uintptr_t highest_usable_addr =
      g_usable_intervals[g_usable_interval_count - 1].end;
  DCHECK(highest_usable_addr > kLowerMemoryLimit);

  g_max_managed_phys_addr = highest_usable_addr;
  g_max_frames = g_max_managed_phys_addr / kPageSize;
  g_pmm_initialized = true;

  int64_t total_populated_frames = 0;

  // Populate free runs that lie inside the initial 64 MiB bootstrap window
  // first so `PagingExtendIdentityMap` can allocate page-table frames from
  // already-mapped low memory if needed without touching unmapped high RAM.
  for (int i = 0; i < g_usable_interval_count; ++i) {
    const uintptr_t u_start = g_usable_intervals[i].start;
    const uintptr_t u_end = g_usable_intervals[i].end;
    if (u_start >= kBootstrapIdentityMapSize) {
      break;
    }
    const uintptr_t low_end =
        (u_end < kBootstrapIdentityMapSize) ? u_end : kBootstrapIdentityMapSize;
    if (low_end > u_start) {
      const int64_t frames = (low_end - u_start) / kPageSize;
      PmmCoalesceFreeRun(u_start, frames);
      total_populated_frames += frames;
    }
  }

  CHECK(PagingExtendIdentityMap(g_max_managed_phys_addr));

  // Now that the identity mapping covers all usable physical RAM up to
  // `g_max_managed_phys_addr`, insert intervals at or above 64 MiB and
  // coalesce any interval that straddled `kBootstrapIdentityMapSize`.
  for (int i = 0; i < g_usable_interval_count; ++i) {
    const uintptr_t u_start = g_usable_intervals[i].start;
    const uintptr_t u_end = g_usable_intervals[i].end;
    if (u_end <= kBootstrapIdentityMapSize) {
      continue;
    }
    const uintptr_t high_start = (u_start > kBootstrapIdentityMapSize)
                                     ? u_start
                                     : kBootstrapIdentityMapSize;
    if (u_end > high_start) {
      const int64_t frames = (u_end - high_start) / kPageSize;
      PmmCoalesceFreeRun(high_start, frames);
      total_populated_frames += frames;
    }
  }

  CHECK(!g_free_run_tree.Empty() && g_pmm_free_frames > 0);
  g_pmm_total_usable_frames = total_populated_frames;

  ConsoleWrite("[PMM] Kernel range: ");
  ConsoleWriteHex(kernel_start);
  ConsoleWrite(" .. ");
  ConsoleWriteHex(kernel_end);
  ConsoleWrite(", Identity-mapped: 0x0 .. ");
  ConsoleWriteHex(PagingIdentityMappedLimit());
  ConsoleWrite("\n");

  ConsoleWrite("[PMM] Total RAM: ");
  ConsoleWriteDec(g_memory_map.total_ram_bytes / 1024);
  ConsoleWrite(" KiB, Usable RAM: ");
  ConsoleWriteDec(g_memory_map.usable_ram_bytes / 1024);
  ConsoleWrite(" KiB, Reserved RAM: ");
  ConsoleWriteDec(g_memory_map.reserved_ram_bytes / 1024);
  ConsoleWrite(" KiB, Free 4KiB frames: ");
  ConsoleWriteDec(g_pmm_free_frames);
  ConsoleWrite("\n");
}

uintptr_t PmmAllocFrames(const int64_t count) {
  const IrqSpinLockGuard lock_guard(g_pmm_lock);
  return PmmAllocFramesLocked(count);
}

uintptr_t PmmAllocFrame() {
  const IrqSpinLockGuard lock_guard(g_pmm_lock);
  return PmmAllocFramesLocked(1);
}

bool PmmRangeIsValidUsableRam(const uintptr_t addr, const int64_t size) {
  const IrqSpinLockGuard lock_guard(g_pmm_lock);
  return PmmRangeIsValidUsableRamLocked(addr, size);
}

void PmmFreeFrames(const uintptr_t base_addr, const int64_t count) {
  const IrqSpinLockGuard lock_guard(g_pmm_lock);
  PmmFreeFramesLocked(base_addr, count);
}

void PmmFreeFrame(const uintptr_t frame_addr) {
  const IrqSpinLockGuard lock_guard(g_pmm_lock);
  PmmFreeFramesLocked(frame_addr, 1);
}

uintptr_t PmmMaxPhysicalAddress() { return g_max_managed_phys_addr; }

int64_t PmmMaxFrameCount() { return g_max_frames; }

int64_t PmmFreeFrameCount() {
  const IrqSpinLockGuard lock_guard(g_pmm_lock);
  return g_pmm_free_frames;
}

int64_t PmmTotalUsableFrameCount() { return g_pmm_total_usable_frames; }

#if __STDC_HOSTED__
void PmmResetForTest() {
  const IrqSpinLockGuard lock_guard(g_pmm_lock);
  ResetPmmState();
}

std::vector<PmmIntervalForTest> PmmCollectUsableIntervalsForTest(
    const MultibootMemoryMap& memory_map) {
  const IrqSpinLockGuard lock_guard(g_pmm_lock);
  ResetPmmState();
  CollectUsableRegions(memory_map);
  std::vector<PmmIntervalForTest> intervals;
  for (int i = 0; i < g_usable_interval_count; ++i) {
    intervals.push_back(
        {g_usable_intervals[i].start, g_usable_intervals[i].end});
  }
  return intervals;
}
#endif

}  // namespace protos
