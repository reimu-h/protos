#include "heap.h"

#include <cstddef>
#include <cstdint>
#include <new>

#include "check.h"
#include "pmm.h"
#include "rbtree.h"
#include "spinlock.h"

namespace protos {

namespace {

// ASCII "HEAP", stored in every valid HeapBlockHeader to detect invalid frees
// or header corruption.
constexpr uint32_t kHeapBlockMagic = 0x48454150;

// Intrusive header placed immediately before each heap block's usable payload:
//
//   +-------------------+------------------------------------+
//   | HeapBlockHeader   | Usable Payload (`size` bytes)      |
//   | (64 bytes)        | (16-byte aligned, returned to user)|
//   +-------------------+------------------------------------+
//   ^                   ^
//   block               reinterpret_cast<void*>(block + 1)
//
// Free blocks are stored in an address-ordered Red-Black Tree (`g_free_tree`)
// augmented with `max_free_size` (the largest free block payload `size` in the
// node's subtree) to support O(log n) address-ordered first-fit allocation of
// `kHeapAlignment`-aligned blocks and O(log n) neighbor lookup for immediate
// physical coalescing.
struct alignas(kHeapAlignment) HeapBlockHeader {
  uint32_t magic;
  uint32_t is_free;
  int64_t size;
  int64_t max_free_size;
  RbNode rb_node;
  int64_t align_offset;
};

constexpr int64_t kBlockHeaderSize = sizeof(HeapBlockHeader);
static_assert(kBlockHeaderSize == 64);
static_assert(alignof(HeapBlockHeader) == kHeapAlignment);

// Smallest alignment padding that `KmallocAlignedLocked` turns into a separate
// free block (a header plus a minimal payload) via `HeapSplitFront`. Smaller
// paddings are absorbed by shifting the header forward (`HeapShiftHeader`).
constexpr int64_t kMinFrontSplitOffset = kBlockHeaderSize + kHeapAlignment;

struct FreeBlockRbTraits {
  static uintptr_t GetKey(const HeapBlockHeader& block) {
    return reinterpret_cast<uintptr_t>(&block);
  }

  static bool Less(const uintptr_t a, const uintptr_t b) { return a < b; }

  static void UpdateAugment(HeapBlockHeader* const block,
                            const HeapBlockHeader* const left,
                            const HeapBlockHeader* const right) {
    int64_t max_size = block->size;
    if (left != nullptr && left->max_free_size > max_size) {
      max_size = left->max_free_size;
    }
    if (right != nullptr && right->max_free_size > max_size) {
      max_size = right->max_free_size;
    }
    block->max_free_size = max_size;
  }
};

using FreeBlockTree =
    RbTree<HeapBlockHeader, &HeapBlockHeader::rb_node, FreeBlockRbTraits>;

// Spinlock protecting `g_free_tree`, `g_total_free_bytes`,
// `g_heap_initialized`, and `g_max_phys_addr` across concurrent multi-CPU heap
// operations. When `Kmalloc` triggers `HeapExpand -> PmmAllocFrames`, lock
// acquisition follows the strict order `g_heap_lock -> g_pmm_lock`.
IrqSpinLock g_heap_lock;

// Intrusive Red-Black Tree containing all currently FREE heap blocks across all
// PMM arenas, keyed by each block header's physical address (`uintptr_t`) and
// augmented with `max_free_size` (the maximum payload `size` among all free
// blocks in the node's subtree). Allocated blocks are removed from this tree
// and re-inserted when freed.
FreeBlockTree g_free_tree;
// Sum of usable payload bytes (`block->size`) across all free blocks in
// `g_free_tree`.
int64_t g_total_free_bytes = 0;
// True once `HeapInit()` has successfully allocated and inserted the initial
// heap arena.
bool g_heap_initialized = false;
// `PmmMaxPhysicalAddress()`, cached by `HeapInit()` because it does not change
// after `PmmInit()`, so that allocations and frees never query the PMM.
uintptr_t g_max_phys_addr = 0;

// Rounds `value` up to the nearest multiple of `alignment` (must be a power
// of two).
static constexpr int64_t AlignUp(const int64_t value, const int64_t alignment) {
  return (value + alignment - 1) & ~(alignment - 1);
}

static constexpr bool IsValidAlignment(const int64_t alignment) {
  return alignment > 0 && alignment <= kPageSize &&
         (alignment & (alignment - 1)) == 0;
}

static int64_t HeapAlignmentOffset(const HeapBlockHeader* const block,
                                   const int64_t alignment) {
  DCHECK(block != nullptr);
  const uintptr_t payload_addr = reinterpret_cast<uintptr_t>(block + 1);
  const uintptr_t mask = static_cast<uintptr_t>(alignment - 1);
  const uintptr_t aligned_addr = (payload_addr + mask) & ~mask;
  return static_cast<int64_t>(aligned_addr - payload_addr);
}

// Returns true if `second` starts at the exact byte immediately following the
// end of `first`'s payload in physical memory (i.e. both blocks are physically
// contiguous with no allocated block or unmapped arena gap between them).
static bool HeapBlocksAreAdjacent(const HeapBlockHeader* const first,
                                  const HeapBlockHeader* const second) {
  DCHECK(first != nullptr);
  DCHECK(second != nullptr);
  const uintptr_t first_end =
      reinterpret_cast<uintptr_t>(first) + kBlockHeaderSize + first->size;
  const uintptr_t second_start = reinterpret_cast<uintptr_t>(second);
  return first_end == second_start;
}

static void FreeTreeInsert(HeapBlockHeader* const block) {
  DCHECK(block != nullptr);
  DCHECK(block->magic == kHeapBlockMagic);
  DCHECK(block->is_free == 0);
  block->is_free = 1;
  block->max_free_size = block->size;
  block->rb_node = {};
  block->align_offset = 0;
  const bool inserted = g_free_tree.Insert(block);
  DCHECK(inserted);
  g_total_free_bytes += block->size;
}

static void FreeTreeRemove(HeapBlockHeader* const block) {
  DCHECK(block != nullptr);
  DCHECK(block->magic == kHeapBlockMagic);
  DCHECK(block->is_free == 1);
  DCHECK(g_total_free_bytes >= block->size);
  g_free_tree.Erase(block);
  block->is_free = 0;
  block->max_free_size = 0;
  g_total_free_bytes -= block->size;
}

// Inserts the in-use (`is_free == 0`) `initial_block` into `g_free_tree` and
// merges it in O(log n) time with its in-order predecessor (`Prev`) and/or
// successor (`Next`) if they are physically adjacent in memory. Clears absorbed
// headers' magic numbers and returns a pointer to the resulting merged free
// block.
static HeapBlockHeader* HeapCoalesceBlock(
    HeapBlockHeader* const initial_block) {
  DCHECK(initial_block != nullptr);
  HeapBlockHeader* block = initial_block;
  FreeTreeInsert(block);

  HeapBlockHeader* const prev_free = FreeBlockTree::Prev(block);
  HeapBlockHeader* const next_free = FreeBlockTree::Next(block);

  if (next_free != nullptr && HeapBlocksAreAdjacent(block, next_free)) {
    const int64_t next_size = next_free->size;
    FreeTreeRemove(next_free);
    next_free->magic = 0;
    const int64_t gained_bytes = kBlockHeaderSize + next_size;
    block->size += gained_bytes;
    g_total_free_bytes += gained_bytes;
    g_free_tree.PropagateAugment(block);
  }

  if (prev_free != nullptr && HeapBlocksAreAdjacent(prev_free, block)) {
    const int64_t block_size = block->size;
    FreeTreeRemove(block);
    block->magic = 0;
    const int64_t gained_bytes = kBlockHeaderSize + block_size;
    prev_free->size += gained_bytes;
    g_total_free_bytes += gained_bytes;
    g_free_tree.PropagateAugment(prev_free);
    block = prev_free;
  }

  return block;
}

// Returns true if `block` has enough trailing bytes beyond `aligned_size` to
// form a separate free block with its own `HeapBlockHeader` (64 bytes) and at
// least `kHeapAlignment` (16 bytes) of payload.
static bool HeapCanSplitBlock(const HeapBlockHeader* const block,
                              const int64_t aligned_size) {
  DCHECK(block != nullptr);
  const int64_t min_split_threshold =
      aligned_size + kBlockHeaderSize + kHeapAlignment;
  return block->size >= min_split_threshold;
}

// Splits the in-use (`is_free == 0`, not in `g_free_tree`) `block` at offset
// `sizeof(HeapBlockHeader) + aligned_size` into two contiguous blocks and
// inserts the trailing remainder into `g_free_tree` in O(log n) time. Requires
// `HeapCanSplitBlock(block, aligned_size)`.
static void HeapSplitBlock(HeapBlockHeader* const block,
                           const int64_t aligned_size) {
  DCHECK(block != nullptr);
  DCHECK(block->is_free == 0);
  DCHECK(HeapCanSplitBlock(block, aligned_size));

  const uintptr_t new_block_addr =
      reinterpret_cast<uintptr_t>(block) + kBlockHeaderSize + aligned_size;
  HeapBlockHeader* const new_block =
      reinterpret_cast<HeapBlockHeader*>(new_block_addr);
  new_block->magic = kHeapBlockMagic;
  new_block->is_free = 0;
  new_block->size = block->size - aligned_size - kBlockHeaderSize;
  new_block->max_free_size = 0;
  new_block->rb_node = {};
  new_block->align_offset = 0;
  block->size = aligned_size;

  FreeTreeInsert(new_block);
}

// Splits the front of an already-removed `block` when `offset >=
// kMinFrontSplitOffset`, returning the leading `[block, block + offset)` slice
// to `g_free_tree` and returning the new aligned block header at
// `block + offset`.
static HeapBlockHeader* HeapSplitFront(HeapBlockHeader* const block,
                                       const int64_t offset) {
  DCHECK(block != nullptr);
  DCHECK(block->is_free == 0);
  DCHECK(offset >= kMinFrontSplitOffset);
  DCHECK((offset & (kHeapAlignment - 1)) == 0);
  DCHECK(block->size > offset);

  const int64_t orig_size = block->size;
  const uintptr_t block_addr = reinterpret_cast<uintptr_t>(block);
  const uintptr_t aligned_block_addr =
      block_addr + static_cast<uintptr_t>(offset);

  block->size = offset - kBlockHeaderSize;
  block->align_offset = 0;
  FreeTreeInsert(block);

  HeapBlockHeader* const aligned_block =
      reinterpret_cast<HeapBlockHeader*>(aligned_block_addr);
  aligned_block->magic = kHeapBlockMagic;
  aligned_block->is_free = 0;
  aligned_block->size = orig_size - offset;
  aligned_block->max_free_size = 0;
  aligned_block->rb_node = {};
  aligned_block->align_offset = 0;
  return aligned_block;
}

// Shifts the header of an already-removed `block` forward by `offset` bytes
// when `0 < offset < kMinFrontSplitOffset` (where the prefix gap is too small
// to form a standalone free block), recording `align_offset = offset` so
// `Kfree` can restore the original block boundary for coalescing.
static HeapBlockHeader* HeapShiftHeader(HeapBlockHeader* const block,
                                        const int64_t offset) {
  DCHECK(block != nullptr);
  DCHECK(block->is_free == 0);
  DCHECK(offset > 0);
  DCHECK(offset < kMinFrontSplitOffset);
  DCHECK((offset & (kHeapAlignment - 1)) == 0);
  DCHECK(block->size > offset);

  const int64_t remaining_size = block->size - offset;
  const uintptr_t aligned_block_addr =
      reinterpret_cast<uintptr_t>(block) + static_cast<uintptr_t>(offset);
  block->magic = 0;

  HeapBlockHeader* const aligned_block =
      reinterpret_cast<HeapBlockHeader*>(aligned_block_addr);
  aligned_block->magic = kHeapBlockMagic;
  aligned_block->is_free = 0;
  aligned_block->size = remaining_size;
  aligned_block->max_free_size = 0;
  aligned_block->rb_node = {};
  aligned_block->align_offset = offset;
  return aligned_block;
}

// Initializes a newly allocated PMM frame range `[arena_addr, arena_addr +
// arena_bytes)` as a free heap block, inserts it into `g_free_tree` in O(log n)
// time, and coalesces it with physically adjacent free neighbors if contiguous.
static HeapBlockHeader* HeapInsertArena(const uintptr_t arena_addr,
                                        const int64_t arena_bytes) {
  DCHECK(arena_addr != 0);
  DCHECK(arena_bytes >= kBlockHeaderSize + kHeapAlignment);
  HeapBlockHeader* const new_block =
      reinterpret_cast<HeapBlockHeader*>(arena_addr);
  new_block->magic = kHeapBlockMagic;
  new_block->is_free = 0;
  new_block->size = arena_bytes - kBlockHeaderSize;
  new_block->max_free_size = 0;
  new_block->rb_node = {};
  new_block->align_offset = 0;

  return HeapCoalesceBlock(new_block);
}

// Requests additional contiguous 4 KiB frames from the PMM for a free block
// with at least `min_payload_size` payload bytes (requesting at least
// `kMinExpandFrames` when possible, falling back to the exact frame count
// needed), and inserts the new arena into `g_free_tree`.
static HeapBlockHeader* HeapExpand(const int64_t min_payload_size) {
  if (min_payload_size <= 0 || g_max_phys_addr <= kBlockHeaderSize ||
      static_cast<uintptr_t>(min_payload_size) >
          g_max_phys_addr - kBlockHeaderSize) {
    return nullptr;
  }
  const int64_t needed_bytes = kBlockHeaderSize + min_payload_size;
  const int64_t exact_frames = (needed_bytes + kPageSize - 1) / kPageSize;
  const int64_t preferred_frames =
      (exact_frames < kMinExpandFrames) ? kMinExpandFrames : exact_frames;

  int64_t alloc_frames = preferred_frames;
  uintptr_t arena_addr = PmmAllocFrames(alloc_frames);
  if (arena_addr == 0 && alloc_frames > exact_frames) {
    alloc_frames = exact_frames;
    arena_addr = PmmAllocFrames(alloc_frames);
  }
  if (arena_addr == 0) {
    return nullptr;
  }

  return HeapInsertArena(arena_addr, alloc_frames * kPageSize);
}

// Finds the lowest-address free block in `g_free_tree` whose payload can
// accommodate `aligned_size` bytes after aligning the payload start to
// `eff_align`. The subtree predicate prunes by payload size only, so the search
// is O(log n) for `eff_align == kHeapAlignment` (every payload is already
// aligned) but may visit every free block of at least `aligned_size` bytes for
// larger alignments, whose padding depends on each block's address.
static HeapBlockHeader* HeapFindFirstFit(const int64_t aligned_size,
                                         const int64_t eff_align) {
  return g_free_tree.FindFirstAugmented(
      [aligned_size](const HeapBlockHeader& block) {
        return block.max_free_size >= aligned_size;
      },
      [aligned_size, eff_align](const HeapBlockHeader& block) {
        if (block.size < aligned_size) {
          return false;
        }
        const int64_t offset = HeapAlignmentOffset(&block, eff_align);
        return (block.size - aligned_size) >= offset;
      });
}

static void* KmallocAlignedLocked(const int64_t size, const int64_t alignment) {
  DCHECK(g_heap_initialized);
  DCHECK(size >= 0);
  if (size < 0 || !IsValidAlignment(alignment)) {
    return nullptr;
  }
  const int64_t eff_align =
      (alignment < kHeapAlignment) ? kHeapAlignment : alignment;
  const int64_t max_pad = eff_align - kHeapAlignment;
  const uintptr_t min_overhead =
      static_cast<uintptr_t>(kBlockHeaderSize + kHeapAlignment + max_pad);
  if (g_max_phys_addr <= min_overhead ||
      static_cast<uintptr_t>(size) > g_max_phys_addr - min_overhead) {
    return nullptr;
  }

  const int64_t raw_size = (size == 0) ? kHeapAlignment : size;
  const int64_t aligned_size = AlignUp(raw_size, kHeapAlignment);

  HeapBlockHeader* candidate = HeapFindFirstFit(aligned_size, eff_align);
  if (candidate == nullptr) {
    candidate = HeapExpand(aligned_size + max_pad);
    if (candidate == nullptr) {
      return nullptr;
    }
  }

  const int64_t offset = HeapAlignmentOffset(candidate, eff_align);
  DCHECK(candidate->size >= aligned_size + offset);

  FreeTreeRemove(candidate);
  if (offset >= kMinFrontSplitOffset) {
    candidate = HeapSplitFront(candidate, offset);
  } else if (offset > 0) {
    candidate = HeapShiftHeader(candidate, offset);
  } else {
    candidate->align_offset = 0;
  }

  if (HeapCanSplitBlock(candidate, aligned_size)) {
    HeapSplitBlock(candidate, aligned_size);
  }
  return reinterpret_cast<void*>(candidate + 1);
}

}  // namespace

void HeapInit() {
  const IrqSpinLockGuard lock_guard(g_heap_lock);
  g_free_tree.Clear();
  g_total_free_bytes = 0;
  g_heap_initialized = false;
  g_max_phys_addr = PmmMaxPhysicalAddress();

  const uintptr_t arena_addr = PmmAllocFrames(kInitialHeapFrames);
  CHECK(arena_addr != 0);
  HeapInsertArena(arena_addr, kInitialHeapFrames * kPageSize);
  g_heap_initialized = true;
}

int64_t HeapTotalFreeBytes() {
  const IrqSpinLockGuard lock_guard(g_heap_lock);
  return g_total_free_bytes;
}

void* KmallocAligned(const int64_t size, const int64_t alignment) {
  const IrqSpinLockGuard lock_guard(g_heap_lock);
  return KmallocAlignedLocked(size, alignment);
}

// Address-ordered first-fit allocator over `g_free_tree`:
// 1. Rounds `size` up to a non-zero multiple of `kHeapAlignment` (16 bytes).
// 2. Queries `g_free_tree.FindFirstAugmented` in O(log n) time for the lowest-
//    address free block with `size >= aligned_size`, removes it from the free
//    tree, splits off any excess tail via `HeapSplitBlock`, and returns the
//    payload pointer (`candidate + 1`).
// 3. If no existing free block fits, expands the heap via `HeapExpand`.
void* Kmalloc(const int64_t size) {
  const IrqSpinLockGuard lock_guard(g_heap_lock);
  DCHECK(g_heap_initialized);
  DCHECK(size >= 0);
  return KmallocAlignedLocked(size, kHeapAlignment);
}

// Validates that `ptr` points to a live, 16-byte-aligned payload preceded by a
// valid in-use `HeapBlockHeader`, restores any shifted alignment header,
// inserts the block into `g_free_tree`, and coalesces it with physically
// adjacent free neighbors in O(log n) time. Double frees fail the header's
// `magic`/`is_free` checks, which is best effort: once a freed block has been
// coalesced or reused, its old header bytes may have been overwritten.
void Kfree(void* const ptr) {
  if (ptr == nullptr) {
    return;
  }
  const IrqSpinLockGuard lock_guard(g_heap_lock);
  DCHECK(g_heap_initialized);
  const uintptr_t ptr_addr = reinterpret_cast<uintptr_t>(ptr);
  DCHECK(ptr_addr >= kLowerMemoryLimit + kBlockHeaderSize);
  DCHECK(ptr_addr < g_max_phys_addr);
  DCHECK((ptr_addr & (kHeapAlignment - 1)) == 0);

  HeapBlockHeader* block = reinterpret_cast<HeapBlockHeader*>(ptr) - 1;
  CHECK(block->magic == kHeapBlockMagic);
  CHECK(block->is_free == 0);
  DCHECK(block->size >= kHeapAlignment);
  DCHECK((block->size & (kHeapAlignment - 1)) == 0);
  DCHECK(block->align_offset >= 0);
  DCHECK(block->align_offset < kMinFrontSplitOffset);
  DCHECK((block->align_offset & (kHeapAlignment - 1)) == 0);
  DCHECK(ptr_addr >= kLowerMemoryLimit + kBlockHeaderSize +
                         static_cast<uintptr_t>(block->align_offset));

  if (block->align_offset > 0) {
    const int64_t shift = block->align_offset;
    const int64_t restored_size = block->size + shift;
    const uintptr_t orig_addr =
        reinterpret_cast<uintptr_t>(block) - static_cast<uintptr_t>(shift);
    block->magic = 0;
    block->is_free = 1;
    block->align_offset = 0;

    HeapBlockHeader* const orig_block =
        reinterpret_cast<HeapBlockHeader*>(orig_addr);
    orig_block->magic = kHeapBlockMagic;
    orig_block->is_free = 0;
    orig_block->size = restored_size;
    orig_block->max_free_size = 0;
    orig_block->rb_node = {};
    orig_block->align_offset = 0;
    block = orig_block;
  }

  HeapCoalesceBlock(block);
}

void* KmallocForNewNothrow(const size_t size, const size_t alignment) noexcept {
  if (size > static_cast<size_t>(INT64_MAX) ||
      alignment > static_cast<size_t>(kPageSize)) {
    return nullptr;
  }
  return KmallocAligned(static_cast<int64_t>(size),
                        static_cast<int64_t>(alignment));
}

void* KmallocForNew(const size_t size, const size_t alignment) {
  void* const ptr = KmallocForNewNothrow(size, alignment);
  CHECK(ptr != nullptr &&
        "operator new: heap exhausted or alignment above kPageSize");
  return ptr;
}

}  // namespace protos

static_assert(static_cast<size_t>(protos::kHeapAlignment) ==
              __STDCPP_DEFAULT_NEW_ALIGNMENT__);

#if !__STDC_HOSTED__
// Global allocation and deallocation functions. The throwing forms never
// return nullptr; code that must survive an allocation failure uses the
// `std::nothrow` forms (`new (std::nothrow) T`) or `Kmalloc`.
void* operator new(const size_t size) {
  return protos::KmallocForNew(size, protos::kHeapAlignment);
}

void* operator new[](const size_t size) {
  return protos::KmallocForNew(size, protos::kHeapAlignment);
}

void* operator new(const size_t size, const std::align_val_t alignment) {
  return protos::KmallocForNew(size, static_cast<size_t>(alignment));
}

void* operator new[](const size_t size, const std::align_val_t alignment) {
  return protos::KmallocForNew(size, static_cast<size_t>(alignment));
}

void* operator new(const size_t size, const std::nothrow_t&) noexcept {
  return protos::KmallocForNewNothrow(size, protos::kHeapAlignment);
}

void* operator new[](const size_t size, const std::nothrow_t&) noexcept {
  return protos::KmallocForNewNothrow(size, protos::kHeapAlignment);
}

void* operator new(const size_t size,                 //
                   const std::align_val_t alignment,  //
                   const std::nothrow_t&) noexcept {
  return protos::KmallocForNewNothrow(size, static_cast<size_t>(alignment));
}

void* operator new[](const size_t size,                 //
                     const std::align_val_t alignment,  //
                     const std::nothrow_t&) noexcept {
  return protos::KmallocForNewNothrow(size, static_cast<size_t>(alignment));
}

void operator delete(void* const ptr) noexcept { protos::Kfree(ptr); }

void operator delete[](void* const ptr) noexcept { protos::Kfree(ptr); }

void operator delete(void* const ptr, const size_t size) noexcept {
  (void)size;
  protos::Kfree(ptr);
}

void operator delete[](void* const ptr, const size_t size) noexcept {
  (void)size;
  protos::Kfree(ptr);
}

void operator delete(void* const ptr,
                     const std::align_val_t alignment) noexcept {
  (void)alignment;
  protos::Kfree(ptr);
}

void operator delete[](void* const ptr,
                       const std::align_val_t alignment) noexcept {
  (void)alignment;
  protos::Kfree(ptr);
}

void operator delete(void* const ptr,    //
                     const size_t size,  //
                     const std::align_val_t alignment) noexcept {
  (void)size;
  (void)alignment;
  protos::Kfree(ptr);
}

void operator delete[](void* const ptr,    //
                       const size_t size,  //
                       const std::align_val_t alignment) noexcept {
  (void)size;
  (void)alignment;
  protos::Kfree(ptr);
}

void operator delete(void* const ptr, const std::nothrow_t&) noexcept {
  protos::Kfree(ptr);
}

void operator delete[](void* const ptr, const std::nothrow_t&) noexcept {
  protos::Kfree(ptr);
}

void operator delete(void* const ptr,                   //
                     const std::align_val_t alignment,  //
                     const std::nothrow_t&) noexcept {
  (void)alignment;
  protos::Kfree(ptr);
}

void operator delete[](void* const ptr,                   //
                       const std::align_val_t alignment,  //
                       const std::nothrow_t&) noexcept {
  (void)alignment;
  protos::Kfree(ptr);
}
#endif  // !__STDC_HOSTED__
