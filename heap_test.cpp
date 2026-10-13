#include "heap.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

#include "pmm.h"
#include "spinlock.h"

namespace protos {
namespace {

namespace t = ::testing;

constexpr int64_t kFakeRamFrames = 4096;  // 16 MiB
constexpr int64_t kFakeRamBytes = kFakeRamFrames * kPageSize;

alignas(kPageSize) uint8_t g_fake_ram[kFakeRamBytes];

struct FakePmmState {
  int64_t next_frame = 0;
  int64_t limit_frame = kFakeRamFrames;
  int64_t gap_frames = 0;
  bool force_oom = false;
  std::vector<int64_t> scripted_start_frames;
  int alloc_call_count = 0;
  std::vector<int64_t> requested_counts;
  int max_phys_call_count = 0;
};

IrqSpinLock g_fake_pmm_lock;
FakePmmState g_pmm;

static uintptr_t FakeRamBase() {
  return reinterpret_cast<uintptr_t>(g_fake_ram);
}

static void ResetFakePmm(const int64_t zero_frames = 512) {
  const IrqSpinLockGuard lock_guard(g_fake_pmm_lock);
  g_pmm = FakePmmState{};
  const int64_t bytes_to_zero = (zero_frames <= kFakeRamFrames)
                                    ? (zero_frames * kPageSize)
                                    : kFakeRamBytes;
  std::memset(g_fake_ram, 0, bytes_to_zero);
}

}  // namespace

uintptr_t PmmMaxPhysicalAddress() {
  const IrqSpinLockGuard lock_guard(g_fake_pmm_lock);
  ++g_pmm.max_phys_call_count;
  return FakeRamBase() + kFakeRamBytes;
}

uintptr_t PmmAllocFrames(const int64_t count) {
  const IrqSpinLockGuard lock_guard(g_fake_pmm_lock);
  ++g_pmm.alloc_call_count;
  g_pmm.requested_counts.push_back(count);
  if (g_pmm.force_oom || count <= 0) {
    return 0;
  }
  if (!g_pmm.scripted_start_frames.empty()) {
    const int64_t start_frame = g_pmm.scripted_start_frames.front();
    if (start_frame < 0 || start_frame > g_pmm.limit_frame ||
        count > g_pmm.limit_frame - start_frame) {
      return 0;
    }
    g_pmm.scripted_start_frames.erase(g_pmm.scripted_start_frames.begin());
    return FakeRamBase() + start_frame * kPageSize;
  }
  const int64_t start_frame =
      (g_pmm.next_frame == 0) ? 0 : (g_pmm.next_frame + g_pmm.gap_frames);
  if (start_frame < 0 || start_frame > g_pmm.limit_frame ||
      count > g_pmm.limit_frame - start_frame) {
    return 0;
  }
  g_pmm.next_frame = start_frame + count;
  return FakeRamBase() + start_frame * kPageSize;
}

namespace {

TEST(HeapTest, InitZeroSizeAndAlignment) {
  // Verify explicit HeapInit() and Kmalloc(0) minimum 16-byte allocation.
  ResetFakePmm();
  HeapInit();
  EXPECT_THAT(g_pmm.alloc_call_count, t::Eq(1));
  ASSERT_THAT(g_pmm.requested_counts, t::ElementsAre(kInitialHeapFrames));

  const int64_t initial_free = HeapTotalFreeBytes();
  const int64_t header_size = kInitialHeapFrames * kPageSize - initial_free;
  EXPECT_THAT(header_size, t::Eq(64));
  EXPECT_THAT(header_size % kHeapAlignment, t::Eq(0));

  void* const zero_ptr = Kmalloc(0);
  ASSERT_THAT(zero_ptr, t::NotNull());
  EXPECT_THAT(reinterpret_cast<uintptr_t>(zero_ptr) & (kHeapAlignment - 1),
              t::Eq(0));
  EXPECT_THAT(HeapTotalFreeBytes(),
              t::Eq(initial_free - kHeapAlignment - header_size));
  Kfree(zero_ptr);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));

  // Verify varied allocation sizes, 16-byte alignment, payload isolation, and
  // full free-byte restoration.
  constexpr int64_t kSizes[] = {1,  7,  15, 16,  17,  31,   32,
                                33, 48, 64, 127, 256, 1024, 4096};
  constexpr int kNumSizes = sizeof(kSizes) / sizeof(kSizes[0]);
  uint8_t* ptrs[kNumSizes];

  for (int i = 0; i < kNumSizes; ++i) {
    const int64_t req_size = kSizes[i];
    ptrs[i] = static_cast<uint8_t*>(Kmalloc(req_size));
    ASSERT_THAT(ptrs[i], t::NotNull());
    const uintptr_t addr = reinterpret_cast<uintptr_t>(ptrs[i]);
    EXPECT_THAT(addr & (kHeapAlignment - 1), t::Eq(0));
    std::memset(ptrs[i], (i + 1) * 19, req_size);
  }

  for (int i = 0; i < kNumSizes; ++i) {
    const uintptr_t start_i = reinterpret_cast<uintptr_t>(ptrs[i]);
    const uintptr_t end_i = start_i + kSizes[i];
    for (int j = i + 1; j < kNumSizes; ++j) {
      const uintptr_t start_j = reinterpret_cast<uintptr_t>(ptrs[j]);
      const uintptr_t end_j = start_j + kSizes[j];
      EXPECT_TRUE(end_i <= start_j || end_j <= start_i);
    }
    const uint8_t expected_byte = (i + 1) * 19;
    for (int64_t b = 0; b < kSizes[i]; ++b) {
      ASSERT_THAT(ptrs[i][b], t::Eq(expected_byte));
    }
  }

  for (int i = 0; i < kNumSizes; ++i) {
    Kfree(ptrs[i]);
  }
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));
}

TEST(HeapTest, BlockSplittingThresholds) {
  ResetFakePmm();
  HeapInit();
  const int64_t initial_free = HeapTotalFreeBytes();
  const int64_t header_size = kInitialHeapFrames * kPageSize - initial_free;

  constexpr int64_t kTargetBlockSize = 256;
  void* const front_block = Kmalloc(kTargetBlockSize);
  void* const barrier = Kmalloc(64);
  ASSERT_THAT(front_block, t::NotNull());
  ASSERT_THAT(barrier, t::NotNull());

  Kfree(front_block);
  const int64_t free_with_hole = HeapTotalFreeBytes();

  // Exact fit (256 bytes) does not split.
  void* const exact_fit = Kmalloc(kTargetBlockSize);
  EXPECT_THAT(exact_fit, t::Eq(front_block));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_with_hole - kTargetBlockSize));
  Kfree(exact_fit);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_with_hole));

  // Remainder is `header_size` (64 bytes), which is 16 bytes below the minimum
  // split threshold (`header_size + kHeapAlignment = 80` bytes). Must NOT
  // split; the full 256-byte block is consumed and later restored.
  const int64_t no_split_req = kTargetBlockSize - header_size;
  void* const unsplit = Kmalloc(no_split_req);
  EXPECT_THAT(unsplit, t::Eq(front_block));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_with_hole - kTargetBlockSize));
  Kfree(unsplit);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_with_hole));

  // Remainder is exactly `header_size + kHeapAlignment` (80 bytes). Must split
  // into `exact_split_req` (176 bytes) + a new 16-byte free block.
  const int64_t exact_split_req =
      kTargetBlockSize - header_size - kHeapAlignment;
  void* const split_head = Kmalloc(exact_split_req);
  EXPECT_THAT(split_head, t::Eq(front_block));
  EXPECT_THAT(HeapTotalFreeBytes(),
              t::Eq(free_with_hole - exact_split_req - header_size));

  // The 16-byte split remainder has a lower address than the main arena tail
  // after `barrier`, so Kmalloc(16) must return it via FindFirstAugmented.
  void* const split_tail = Kmalloc(kHeapAlignment);
  const uintptr_t expected_tail_addr =
      reinterpret_cast<uintptr_t>(split_head) + exact_split_req + header_size;
  EXPECT_THAT(reinterpret_cast<uintptr_t>(split_tail),
              t::Eq(expected_tail_addr));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_with_hole - kTargetBlockSize));

  Kfree(split_head);
  Kfree(split_tail);
  Kfree(barrier);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));
}

TEST(HeapTest, AddressOrderedFirstFitSelectionViaFindFirstAugmented) {
  ResetFakePmm();
  HeapInit();
  const int64_t initial_free = HeapTotalFreeBytes();
  const int64_t header_size = kInitialHeapFrames * kPageSize - initial_free;

  // Create 4 free holes at ascending addresses:
  //   h0 (32B) < h1 (128B) < h2 (512B) < h3 (128B)
  // separated by allocated barrier blocks b0..b3.
  void* const h0 = Kmalloc(32);
  void* const b0 = Kmalloc(32);
  void* const h1 = Kmalloc(128);
  void* const b1 = Kmalloc(32);
  void* const h2 = Kmalloc(512);
  void* const b2 = Kmalloc(32);
  void* const h3 = Kmalloc(128);
  void* const b3 = Kmalloc(32);
  ASSERT_THAT(h0, t::NotNull());
  ASSERT_THAT(b0, t::NotNull());
  ASSERT_THAT(h1, t::NotNull());
  ASSERT_THAT(b1, t::NotNull());
  ASSERT_THAT(h2, t::NotNull());
  ASSERT_THAT(b2, t::NotNull());
  ASSERT_THAT(h3, t::NotNull());
  ASSERT_THAT(b3, t::NotNull());

  Kfree(h0);
  Kfree(h1);
  Kfree(h2);
  Kfree(h3);

  // Request 64B: h0 (32B) is too small; h1 (128B) is the lowest-address
  // qualifying free block and must be chosen over larger h2 (512B) and equal
  // higher-address h3 (128B).
  void* const pick1 = Kmalloc(64);
  EXPECT_THAT(pick1, t::Eq(h1));

  // Request 128B: h1 is now in use; h2 (512B) and h3 (128B) both qualify.
  // Address-ordered first-fit must choose h2 (lower address) over exact-fit h3.
  void* const pick2 = Kmalloc(128);
  EXPECT_THAT(pick2, t::Eq(h2));

  // h2 (512B) was split into 128B + (512 - 128 - header_size = 320B).
  // Requesting 256B must choose the 320B split remainder of h2.
  void* const pick3 = Kmalloc(256);
  const uintptr_t expected_h2_rem =
      reinterpret_cast<uintptr_t>(h2) + 128 + header_size;
  EXPECT_THAT(reinterpret_cast<uintptr_t>(pick3), t::Eq(expected_h2_rem));

  // Request 128B: now h3 (128B) is the lowest-address block with size >= 128.
  void* const pick4 = Kmalloc(128);
  EXPECT_THAT(pick4, t::Eq(h3));

  // Request 32B: h0 (32B) at the very lowest address is still free and fits.
  void* const pick5 = Kmalloc(32);
  EXPECT_THAT(pick5, t::Eq(h0));

  Kfree(pick1);
  Kfree(pick2);
  Kfree(pick3);
  Kfree(pick4);
  Kfree(pick5);
  Kfree(b0);
  Kfree(b1);
  Kfree(b2);
  Kfree(b3);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));
}

TEST(HeapTest, CoalescingForwardBackwardAndThreeWay) {
  ResetFakePmm();
  HeapInit();
  const int64_t initial_free = HeapTotalFreeBytes();
  const int64_t header_size = kInitialHeapFrames * kPageSize - initial_free;

  void* a = Kmalloc(64);
  void* b = Kmalloc(128);
  void* const c = Kmalloc(256);
  void* const d = Kmalloc(64);
  ASSERT_THAT(a, t::NotNull());
  ASSERT_THAT(b, t::NotNull());
  ASSERT_THAT(c, t::NotNull());
  ASSERT_THAT(d, t::NotNull());
  const int64_t free_after_abcd = HeapTotalFreeBytes();

  // Backward coalescing: free `a` first, then free `b` -> `b` merges into
  // predecessor `a`, reclaiming `b`'s header.
  Kfree(a);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd + 64));
  Kfree(b);
  const int64_t merged_ab_size = 64 + 128 + header_size;
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd + merged_ab_size));

  // Absorbed header magic at `b` must be cleared to 0.
  const uint32_t* const b_magic_ptr = reinterpret_cast<const uint32_t*>(
      reinterpret_cast<uintptr_t>(b) - header_size);
  EXPECT_THAT(*b_magic_ptr, t::Eq(0));

  void* const ab = Kmalloc(merged_ab_size);
  EXPECT_THAT(ab, t::Eq(a));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd));

  // Restore `a` (64B) and `b` (128B).
  Kfree(ab);
  a = Kmalloc(64);
  b = Kmalloc(128);
  ASSERT_THAT(a, t::NotNull());
  ASSERT_THAT(b, t::NotNull());
  ASSERT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd));

  // Forward coalescing: free `b` first, then free `a` -> `a` merges forward
  // with successor `b`.
  Kfree(b);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd + 128));
  Kfree(a);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd + merged_ab_size));
  EXPECT_THAT(*b_magic_ptr, t::Eq(0));

  void* const ab2 = Kmalloc(merged_ab_size);
  EXPECT_THAT(ab2, t::Eq(a));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd));

  // Restore `a` (64B) and `b` (128B).
  Kfree(ab2);
  a = Kmalloc(64);
  b = Kmalloc(128);
  ASSERT_THAT(a, t::NotNull());
  ASSERT_THAT(b, t::NotNull());

  // Three-way coalescing: free `a` and `c` first, then free middle block `b`
  // -> `a + b + c` merge into a single free block at `a`.
  Kfree(a);
  Kfree(c);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd + 64 + 256));

  Kfree(b);
  const int64_t merged_abc_size = 64 + 128 + 256 + 2 * header_size;
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd + merged_abc_size));

  const uint32_t* const c_magic_ptr = reinterpret_cast<const uint32_t*>(
      reinterpret_cast<uintptr_t>(c) - header_size);
  EXPECT_THAT(*b_magic_ptr, t::Eq(0));
  EXPECT_THAT(*c_magic_ptr, t::Eq(0));

  void* const abc = Kmalloc(merged_abc_size);
  EXPECT_THAT(abc, t::Eq(a));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd));

  Kfree(abc);
  Kfree(d);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));
}

TEST(HeapTest, MultiArenaExpansionContiguousAndNonContiguousIsolation) {
  // Non-contiguous arena expansion with a 1-frame gap. Also verifies that
  // exhausting Arena 1 (`g_free_tree.Empty() == true`) triggers HeapExpand
  // rather than re-running HeapInit().
  ResetFakePmm(1024);
  g_pmm.gap_frames = 1;
  HeapInit();

  const int64_t arena1_payload = HeapTotalFreeBytes();
  const int64_t header_size = kInitialHeapFrames * kPageSize - arena1_payload;

  void* const full_arena1 = Kmalloc(arena1_payload);
  ASSERT_THAT(full_arena1, t::NotNull());
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(0));

  void* const arena2_alloc = Kmalloc(1024);
  ASSERT_THAT(arena2_alloc, t::NotNull());
  ASSERT_THAT(g_pmm.requested_counts,
              t::ElementsAre(kInitialHeapFrames, kMinExpandFrames));

  const uintptr_t expected_arena2_addr =
      FakeRamBase() + (kInitialHeapFrames + 1) * kPageSize + header_size;
  EXPECT_THAT(reinterpret_cast<uintptr_t>(arena2_alloc),
              t::Eq(expected_arena2_addr));

  Kfree(full_arena1);
  Kfree(arena2_alloc);

  const int64_t arena2_payload = kMinExpandFrames * kPageSize - header_size;
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(arena1_payload + arena2_payload));

  // Because of the 1-frame gap, Arena 1 and Arena 2 must NOT have coalesced.
  g_pmm.force_oom = true;
  EXPECT_THAT(Kmalloc(arena1_payload + 16), t::IsNull());
  g_pmm.force_oom = false;

  // Non-contiguous arenas inserted in descending & interleaved address order
  // (frame 512, then frame 100, then frame 300).
  ResetFakePmm(1024);
  g_pmm.scripted_start_frames = {512, 100, 300};
  HeapInit();

  void* const high_arena = Kmalloc(arena1_payload);
  ASSERT_THAT(high_arena, t::NotNull());
  void* const low_arena = Kmalloc(arena2_payload);
  ASSERT_THAT(low_arena, t::NotNull());
  void* const mid_arena = Kmalloc(arena2_payload);
  ASSERT_THAT(mid_arena, t::NotNull());

  EXPECT_LT(reinterpret_cast<uintptr_t>(low_arena),
            reinterpret_cast<uintptr_t>(mid_arena));
  EXPECT_LT(reinterpret_cast<uintptr_t>(mid_arena),
            reinterpret_cast<uintptr_t>(high_arena));

  Kfree(high_arena);
  Kfree(low_arena);
  Kfree(mid_arena);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(arena1_payload + 2 * arena2_payload));

  g_pmm.force_oom = true;
  // Small request must select `low_arena` (lowest physical address).
  void* const first_fit_low = Kmalloc(64);
  EXPECT_THAT(first_fit_low, t::Eq(low_arena));
  // Request larger than `arena2_payload` must skip `low_arena` and `mid_arena`
  // and select `high_arena`.
  void* const first_fit_high = Kmalloc(arena2_payload + 64);
  EXPECT_THAT(first_fit_high, t::Eq(high_arena));
  Kfree(first_fit_low);
  Kfree(first_fit_high);

  // Contiguous arena expansion + fallback from `kMinExpandFrames` (16) to
  // `exact_frames` (1) when PMM has only 4 frames remaining.
  ResetFakePmm(512);
  g_pmm.limit_frame = kInitialHeapFrames + 4;
  HeapInit();

  void* const p1 = Kmalloc(arena1_payload);
  ASSERT_THAT(p1, t::NotNull());
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(0));

  void* const p2 = Kmalloc(2048);
  ASSERT_THAT(p2, t::NotNull());
  // HeapExpand tried kMinExpandFrames (16), failed, and fell back to
  // exact_frames (1).
  ASSERT_THAT(g_pmm.requested_counts,
              t::ElementsAre(kInitialHeapFrames, kMinExpandFrames, 1));

  Kfree(p1);
  Kfree(p2);
  // Because Arena 2 at frame 256 is physically contiguous with Arena 1
  // [0, 256), the two arenas coalesce into a single free block of size
  // `arena1_payload + kPageSize`.
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(arena1_payload + kPageSize));

  g_pmm.force_oom = true;
  void* const cross_arena = Kmalloc(arena1_payload + 2048);
  EXPECT_THAT(cross_arena, t::Eq(p1));
  Kfree(cross_arena);
}

TEST(HeapDeathTest, InitPanicsIfInitialArenaCannotBeAllocated) {
  ResetFakePmm();
  g_pmm.force_oom = true;
  EXPECT_DEATH(HeapInit(), "Check failed: arena_addr != 0 at");
}

TEST(HeapTest, OomAndNullFreeResilience) {
  ResetFakePmm();
  HeapInit();
  void* const recovered = Kmalloc(64);
  ASSERT_THAT(recovered, t::NotNull());
  Kfree(recovered);

  const int64_t initial_free = HeapTotalFreeBytes();
  const int64_t header_size = kInitialHeapFrames * kPageSize - initial_free;

  // Overflow and out-of-bounds Kmalloc sizes and PMM exhaustion.
  EXPECT_THAT(Kmalloc(INT64_MAX), t::IsNull());
  EXPECT_THAT(Kmalloc(PmmMaxPhysicalAddress()), t::IsNull());
  EXPECT_THAT(Kmalloc(PmmMaxPhysicalAddress() - header_size), t::IsNull());

  g_pmm.force_oom = true;
  EXPECT_THAT(Kmalloc(initial_free + 64), t::IsNull());
  g_pmm.force_oom = false;
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));

  // Kfree(nullptr) is a safe no-op.
  uint8_t* const a = static_cast<uint8_t*>(Kmalloc(128));
  ASSERT_THAT(a, t::NotNull());
  const int64_t free_baseline = HeapTotalFreeBytes();
  Kfree(nullptr);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_baseline));
  Kfree(a);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));
}

TEST(HeapDeathTest, InvalidOrDoubleFreePanics) {
  ResetFakePmm();
  HeapInit();

  uint8_t* const a = static_cast<uint8_t*>(Kmalloc(128));
  uint8_t* const b = static_cast<uint8_t*>(Kmalloc(128));
  ASSERT_THAT(a, t::NotNull());
  ASSERT_THAT(b, t::NotNull());
  std::memset(a, 0, 128);

  // A negative Kmalloc size violates a DCHECKed precondition.
  EXPECT_DEATH(Kmalloc(-1), "Check failed: size >= 0 at");

  // Out-of-bounds and misaligned pointers fail Kfree's DCHECKs.
  EXPECT_DEATH(
      Kfree(reinterpret_cast<void*>(kLowerMemoryLimit)),
      "Check failed: ptr_addr >= kLowerMemoryLimit \\+ kBlockHeaderSize at");
  EXPECT_DEATH(Kfree(reinterpret_cast<void*>(PmmMaxPhysicalAddress())),
               "Check failed: ptr_addr < g_max_phys_addr at");
  EXPECT_DEATH(
      Kfree(a + 1),
      "Check failed: \\(ptr_addr & \\(kHeapAlignment - 1\\)\\) == 0 at");

  // A pointer into the middle of a payload has no block header in front of it.
  EXPECT_DEATH(Kfree(a + 64),
               "Check failed: block->magic == kHeapBlockMagic at");

  // Double free of a standalone free block fails the `is_free` CHECK.
  Kfree(b);
  EXPECT_DEATH(Kfree(b), "Check failed: block->is_free == 0 at");

  // Double free of a block whose header was absorbed by coalescing fails the
  // `magic` CHECK.
  Kfree(a);
  EXPECT_DEATH(Kfree(b), "Check failed: block->magic == kHeapBlockMagic at");
}

TEST(HeapDeathTest, DoubleFreeOfHeaderShiftedBlockPanics) {
  ResetFakePmm();
  HeapInit();

  // The arena is page aligned, so the first payload starts at arena + 64 and
  // the free block after a 32-byte allocation has its payload at arena + 160.
  // Aligning that payload to 64 bytes needs 32 bytes of padding, which is too
  // small for a separate free block, so KmallocAligned shifts the header.
  uint8_t* const first = static_cast<uint8_t*>(Kmalloc(32));
  uint8_t* const shifted = static_cast<uint8_t*>(KmallocAligned(64, 64));
  ASSERT_THAT(first, t::NotNull());
  ASSERT_THAT(shifted, t::Eq(first + 128));

  Kfree(shifted);
  EXPECT_DEATH(Kfree(shifted),
               "Check failed: block->magic == kHeapBlockMagic at");

  // Still detected after the restored block coalesced with its predecessor.
  Kfree(first);
  EXPECT_DEATH(Kfree(shifted),
               "Check failed: block->magic == kHeapBlockMagic at");
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(kInitialHeapFrames * kPageSize - 64));
}

TEST(HeapTest, OverAlignedExpansionReservesWorstCaseAlignmentPadding) {
  struct Case {
    int64_t alignment;
    int64_t size;
    int64_t expected_frames;
  };
  // Each size exactly fills whole frames together with one 64-byte header, so
  // the new arena must also reserve `alignment - kHeapAlignment` bytes for
  // aligning the payload, or it is a frame short.
  constexpr Case kCases[] = {
      {128, 17 * kPageSize - 64, 18},
      {kPageSize, 16 * kPageSize - 64, 17},
  };
  for (const Case& c : kCases) {
    SCOPED_TRACE(c.alignment);
    ResetFakePmm(1024);
    g_pmm.gap_frames = 1;
    HeapInit();
    // Exhaust the initial arena so that the request needs a new, separate one.
    void* const arena1 = Kmalloc(HeapTotalFreeBytes());
    ASSERT_THAT(arena1, t::NotNull());
    ASSERT_THAT(HeapTotalFreeBytes(), t::Eq(0));

    uint8_t* const aligned =
        static_cast<uint8_t*>(KmallocAligned(c.size, c.alignment));
    ASSERT_THAT(aligned, t::NotNull());
    EXPECT_THAT(reinterpret_cast<uintptr_t>(aligned) & (c.alignment - 1),
                t::Eq(0u));
    EXPECT_THAT(g_pmm.requested_counts,
                t::ElementsAre(kInitialHeapFrames, c.expected_frames));
    std::memset(aligned, 0x5A, c.size);
    Kfree(aligned);
    Kfree(arena1);
  }
}

TEST(HeapTest, OnlyHeapInitQueriesPmmMaxPhysicalAddress) {
  ResetFakePmm();
  HeapInit();
  EXPECT_THAT(g_pmm.max_phys_call_count, t::Eq(1));

  void* const small = Kmalloc(64);
  void* const aligned = KmallocAligned(100, 512);
  void* const expanding = Kmalloc(2 * kInitialHeapFrames * kPageSize);
  ASSERT_THAT(small, t::NotNull());
  ASSERT_THAT(aligned, t::NotNull());
  ASSERT_THAT(expanding, t::NotNull());
  EXPECT_THAT(Kmalloc(INT64_MAX), t::IsNull());
  Kfree(small);
  Kfree(aligned);
  Kfree(expanding);

  EXPECT_THAT(g_pmm.alloc_call_count, t::Eq(2));
  EXPECT_THAT(g_pmm.max_phys_call_count, t::Eq(1));
}

TEST(HeapTest, KmallocForNewNothrowReturnsNullForUnsatisfiableRequests) {
  ResetFakePmm();
  HeapInit();
  const int64_t initial_free = HeapTotalFreeBytes();

  EXPECT_THAT(KmallocForNewNothrow(SIZE_MAX, kHeapAlignment), t::IsNull());
  EXPECT_THAT(
      KmallocForNewNothrow(static_cast<size_t>(INT64_MAX) + 1, kHeapAlignment),
      t::IsNull());
  EXPECT_THAT(KmallocForNewNothrow(64, 2 * kPageSize), t::IsNull());
  EXPECT_THAT(KmallocForNewNothrow(64, SIZE_MAX), t::IsNull());
  EXPECT_THAT(KmallocForNewNothrow(64, 24), t::IsNull());
  g_pmm.force_oom = true;
  EXPECT_THAT(KmallocForNewNothrow(initial_free + 64, kHeapAlignment),
              t::IsNull());
  g_pmm.force_oom = false;
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));

  void* const ptr = KmallocForNewNothrow(100, 256);
  ASSERT_THAT(ptr, t::NotNull());
  EXPECT_THAT(reinterpret_cast<uintptr_t>(ptr) & 255u, t::Eq(0u));
  Kfree(ptr);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));
}

TEST(HeapDeathTest, KmallocForNewPanicsInsteadOfReturningNull) {
  ResetFakePmm();
  HeapInit();
  void* const ptr = KmallocForNew(100, kHeapAlignment);
  ASSERT_THAT(ptr, t::NotNull());
  EXPECT_THAT(reinterpret_cast<uintptr_t>(ptr) & (kHeapAlignment - 1),
              t::Eq(0u));
  Kfree(ptr);

  EXPECT_DEATH(KmallocForNew(SIZE_MAX, kHeapAlignment),
               "operator new: heap exhausted");
  EXPECT_DEATH(KmallocForNew(64, 2 * kPageSize),
               "operator new: heap exhausted");
  g_pmm.force_oom = true;
  EXPECT_DEATH(
      KmallocForNew(2 * kInitialHeapFrames * kPageSize, kHeapAlignment),
      "operator new: heap exhausted");
}

TEST(HeapTest, HighFragmentationStressAndFullCoalescence) {
  ResetFakePmm(kFakeRamFrames);
  HeapInit();
  const int64_t initial_free = HeapTotalFreeBytes();

  constexpr int kNumBlocks = 512;
  uint8_t* targets[kNumBlocks];
  uint8_t* barriers[kNumBlocks];
  int64_t target_sizes[kNumBlocks];

  for (int i = 0; i < kNumBlocks; ++i) {
    target_sizes[i] = ((i % 16) + 1) * kHeapAlignment;
    targets[i] = static_cast<uint8_t*>(Kmalloc(target_sizes[i]));
    barriers[i] = static_cast<uint8_t*>(Kmalloc(kHeapAlignment));
    ASSERT_THAT(targets[i], t::NotNull());
    ASSERT_THAT(barriers[i], t::NotNull());

    std::memset(targets[i], (i + 3) & 0xFF, target_sizes[i]);
    std::memset(barriers[i], (i ^ 0x5A) & 0xFF, kHeapAlignment);
  }

  // Free all 512 target blocks in pseudo-random order (gcd(73, 512) == 1)
  // while keeping all 512 barriers allocated so no two free blocks coalesce.
  for (int step = 0; step < kNumBlocks; ++step) {
    const int idx = (step * 73 + 19) & (kNumBlocks - 1);
    Kfree(targets[idx]);
  }

  // Re-allocate and free across the 512 separated free blocks in multiple
  // rounds. Because we request `target_sizes[i]` in ascending index order,
  // address-ordered first-fit via FindFirstAugmented must return the exact
  // original hole address `targets[i]` for every block.
  for (int round = 0; round < 3; ++round) {
    for (int i = 0; i < kNumBlocks; ++i) {
      void* const realloc_ptr = Kmalloc(target_sizes[i]);
      ASSERT_THAT(realloc_ptr, t::Eq(targets[i]));
    }
    for (int step = 0; step < kNumBlocks; ++step) {
      const int idx = (step * 109 + 43) & (kNumBlocks - 1);
      Kfree(targets[idx]);
    }
  }

  // Verify barrier payloads were never corrupted across all fragmentation
  // rounds, then free all barriers in pseudo-random order.
  for (int step = 0; step < kNumBlocks; ++step) {
    const int idx = (step * 83 + 11) & (kNumBlocks - 1);
    const uint8_t expected = (idx ^ 0x5A) & 0xFF;
    for (int64_t b = 0; b < kHeapAlignment; ++b) {
      ASSERT_THAT(barriers[idx][b], t::Eq(expected));
    }
    Kfree(barriers[idx]);
  }

  // All 1024 blocks + the arena tail must have coalesced back into a single
  // contiguous free block equal to `initial_free`.
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));
  void* const whole_arena = Kmalloc(initial_free);
  ASSERT_THAT(whole_arena, t::NotNull());
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(0));
  Kfree(whole_arena);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));
}

TEST(HeapTest, ConcurrentMultiThreadedAllocFreeAndArenaExpansionStress) {
  ResetFakePmm(kFakeRamFrames);
  HeapInit();
  const int64_t initial_free = HeapTotalFreeBytes();
  const int64_t header_size = kInitialHeapFrames * kPageSize - initial_free;

  constexpr int kNumThreads = 8;
  constexpr int kIterationsPerThread = 120;
  constexpr int kBatchSize = 6;
  std::atomic<int> ready_threads{0};
  std::atomic<bool> start_gate{false};
  std::atomic<int> expanded_threads{0};

  std::vector<std::thread> workers;
  workers.reserve(kNumThreads);
  for (int tid = 0; tid < kNumThreads; ++tid) {
    workers.emplace_back([&, tid]() {
      ready_threads.fetch_add(1, std::memory_order_acq_rel);
      while (!start_gate.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }

      // Phase 1: Each thread simultaneously holds a 192 KiB allocation
      // (8 * 192 KiB = 1.5 MiB > 1 MiB initial arena) so concurrent threads
      // deterministically trigger HeapExpand -> PmmAllocFrames under nested
      // g_heap_lock -> g_fake_pmm_lock ordering.
      constexpr int64_t kExpandAllocSize = 192 * 1024;
      uint8_t* const expand_ptr =
          static_cast<uint8_t*>(Kmalloc(kExpandAllocSize));
      ASSERT_THAT(expand_ptr, t::NotNull());
      const uint8_t expand_tag = static_cast<uint8_t>(0xA0 + tid);
      std::memset(expand_ptr, expand_tag, kExpandAllocSize);
      expanded_threads.fetch_add(1, std::memory_order_acq_rel);
      while (expanded_threads.load(std::memory_order_acquire) < kNumThreads) {
        std::this_thread::yield();
      }
      ASSERT_THAT(expand_ptr[0], t::Eq(expand_tag));
      ASSERT_THAT(expand_ptr[kExpandAllocSize / 2], t::Eq(expand_tag));
      ASSERT_THAT(expand_ptr[kExpandAllocSize - 1], t::Eq(expand_tag));
      Kfree(expand_ptr);

      // Phase 2: High-contention mixed-size allocations and frees.
      for (int iter = 0; iter < kIterationsPerThread; ++iter) {
        uint8_t* ptrs[kBatchSize] = {};
        int64_t sizes[kBatchSize] = {};

        for (int b = 0; b < kBatchSize; ++b) {
          const int64_t req_size =
              (((iter + tid + b) % 16) + 1) * kHeapAlignment;
          sizes[b] = req_size;
          ptrs[b] = static_cast<uint8_t*>(Kmalloc(req_size));
          ASSERT_THAT(ptrs[b], t::NotNull());
          ASSERT_THAT(
              reinterpret_cast<uintptr_t>(ptrs[b]) & (kHeapAlignment - 1),
              t::Eq(0u));
          const uint8_t pattern =
              static_cast<uint8_t>(((tid + 1) * 37 + iter * 11 + b * 5) & 0xFF);
          std::memset(ptrs[b], pattern, req_size);
        }

        if ((iter & 15) == 0) {
          EXPECT_GE(HeapTotalFreeBytes(), 0);
        }

        for (int b = 0; b < kBatchSize; ++b) {
          const uint8_t expected =
              static_cast<uint8_t>(((tid + 1) * 37 + iter * 11 + b * 5) & 0xFF);
          ASSERT_THAT(ptrs[b][0], t::Eq(expected));
          ASSERT_THAT(ptrs[b][sizes[b] / 2], t::Eq(expected));
          ASSERT_THAT(ptrs[b][sizes[b] - 1], t::Eq(expected));
        }

        if ((iter & 1) == 0) {
          for (int b = 0; b < kBatchSize; ++b) {
            Kfree(ptrs[b]);
          }
        } else {
          for (int b = kBatchSize; b > 0; --b) {
            Kfree(ptrs[b - 1]);
          }
        }
      }
    });
  }

  while (ready_threads.load(std::memory_order_acquire) < kNumThreads) {
    std::this_thread::yield();
  }
  start_gate.store(true, std::memory_order_release);

  for (std::thread& worker : workers) {
    worker.join();
  }

  // Because contiguous FakePmm arenas coalesce into the initial arena, the
  // final free bytes must equal `g_pmm.next_frame * kPageSize - header_size`.
  EXPECT_GT(g_pmm.alloc_call_count, 1);
  const int64_t expected_total_free =
      g_pmm.next_frame * kPageSize - header_size;
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(expected_total_free));

  void* const whole_heap = Kmalloc(expected_total_free);
  ASSERT_THAT(whole_heap, t::NotNull());
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(0));
  Kfree(whole_heap);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(expected_total_free));
}

TEST(
    HeapTest,
    KmallocAlignedSupportsPowerOfTwoAlignmentsUpToPageSizeAndRestoresFreeBytes) {
  ResetFakePmm(kFakeRamFrames);
  HeapInit();
  const int64_t initial_free = HeapTotalFreeBytes();

  // Invalid alignments (<= 0, non-power-of-two, or > kPageSize) return nullptr
  // without modifying heap state.
  EXPECT_THAT(KmallocAligned(64, 0), t::IsNull());
  EXPECT_THAT(KmallocAligned(64, -16), t::IsNull());
  EXPECT_THAT(KmallocAligned(64, 3), t::IsNull());
  EXPECT_THAT(KmallocAligned(64, 24), t::IsNull());
  EXPECT_THAT(KmallocAligned(64, 96), t::IsNull());
  EXPECT_THAT(KmallocAligned(64, kPageSize * 2), t::IsNull());
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));

  constexpr int64_t kAlignments[] = {1,   2,   4,   8,    16,   32,  64,
                                     128, 256, 512, 1024, 2048, 4096};
  constexpr int kNumAlignments = sizeof(kAlignments) / sizeof(kAlignments[0]);
  constexpr int64_t kSizes[] = {0, 1, 17, 64, 192, 512, 4096};
  constexpr int kNumSizes = sizeof(kSizes) / sizeof(kSizes[0]);

  struct AllocationRecord {
    uint8_t* ptr;
    int64_t size;
    int64_t alignment;
    uint8_t pattern;
  };
  std::vector<AllocationRecord> records;
  records.reserve(kNumAlignments * kNumSizes);

  for (int a_idx = 0; a_idx < kNumAlignments; ++a_idx) {
    const int64_t alignment = kAlignments[a_idx];
    const int64_t effective_align =
        (alignment < kHeapAlignment) ? kHeapAlignment : alignment;
    for (int s_idx = 0; s_idx < kNumSizes; ++s_idx) {
      const int64_t req_size = kSizes[s_idx];
      const int64_t usable_size = (req_size == 0) ? kHeapAlignment : req_size;
      uint8_t* const ptr =
          static_cast<uint8_t*>(KmallocAligned(req_size, alignment));
      ASSERT_THAT(ptr, t::NotNull());
      const uintptr_t addr = reinterpret_cast<uintptr_t>(ptr);
      EXPECT_THAT(addr & static_cast<uintptr_t>(effective_align - 1),
                  t::Eq(0u));
      const uint8_t pattern =
          static_cast<uint8_t>(((a_idx + 1) * 29 + (s_idx + 1) * 7) & 0xFF);
      std::memset(ptr, pattern, usable_size);
      records.push_back(
          AllocationRecord{ptr, usable_size, effective_align, pattern});
    }
  }

  // Verify payload isolation and integrity across all aligned allocations.
  const int num_records = static_cast<int>(records.size());
  for (int i = 0; i < num_records; ++i) {
    const uintptr_t start_i = reinterpret_cast<uintptr_t>(records[i].ptr);
    const uintptr_t end_i = start_i + static_cast<uintptr_t>(records[i].size);
    for (int j = i + 1; j < num_records; ++j) {
      const uintptr_t start_j = reinterpret_cast<uintptr_t>(records[j].ptr);
      const uintptr_t end_j = start_j + static_cast<uintptr_t>(records[j].size);
      EXPECT_TRUE(end_i <= start_j || end_j <= start_i);
    }
    for (int64_t b = 0; b < records[i].size; ++b) {
      ASSERT_THAT(records[i].ptr[b], t::Eq(records[i].pattern));
    }
  }

  // Free in interleaved order and verify 100% of free bytes are restored and
  // coalesced back into a single arena block.
  for (int step = 0; step < num_records; ++step) {
    const int idx = (step * 37 + 11) % num_records;
    Kfree(records[idx].ptr);
  }
  const int64_t expected_total_free =
      g_pmm.next_frame * kPageSize -
      (kInitialHeapFrames * kPageSize - initial_free);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(expected_total_free));

  void* const whole_heap = Kmalloc(expected_total_free);
  ASSERT_THAT(whole_heap, t::NotNull());
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(0));
  Kfree(whole_heap);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(expected_total_free));
}

TEST(HeapTest, KmallocAlignedFrontSplitHeaderShiftAndCoalescing) {
  ResetFakePmm();
  HeapInit();
  const int64_t initial_free = HeapTotalFreeBytes();

  // Initial payload starts at FakeRamBase() + 64 (offset 64 mod 4096).
  // Case 1: Request 64-byte alignment when current free payload is at +64
  // (offset == 0): no shift or front split needed.
  void* const aligned64_zero_offset = KmallocAligned(32, 64);
  ASSERT_THAT(aligned64_zero_offset, t::NotNull());
  EXPECT_THAT(reinterpret_cast<uintptr_t>(aligned64_zero_offset) & 63u,
              t::Eq(0u));

  // Next free block payload is at +64 + 32 + 64 = +160 (32 mod 64).
  // Requesting 64-byte alignment requires offset = 32 bytes, exercising the
  // header-shift path (`0 < offset < kMinFrontSplitOffset`, 80 bytes).
  void* const aligned64_shifted = KmallocAligned(64, 64);
  ASSERT_THAT(aligned64_shifted, t::NotNull());
  EXPECT_THAT(reinterpret_cast<uintptr_t>(aligned64_shifted) & 63u, t::Eq(0u));
  std::memset(aligned64_shifted, 0x5A, 64);

  // Requesting 4096-byte alignment requires a large offset, exercising the
  // front-split path (`offset >= kMinFrontSplitOffset`), which returns the
  // front padding block to `g_free_tree`.
  void* const aligned4096_front_split = KmallocAligned(128, kPageSize);
  ASSERT_THAT(aligned4096_front_split, t::NotNull());
  EXPECT_THAT(
      reinterpret_cast<uintptr_t>(aligned4096_front_split) & (kPageSize - 1),
      t::Eq(0u));
  std::memset(aligned4096_front_split, 0xA5, 128);

  // Allocate a small 16-byte block: it must fit inside the front-split free
  // block created before `aligned4096_front_split`.
  void* const from_front_split = Kmalloc(16);
  ASSERT_THAT(from_front_split, t::NotNull());
  EXPECT_LT(reinterpret_cast<uintptr_t>(from_front_split),
            reinterpret_cast<uintptr_t>(aligned4096_front_split));

  // Free in various orders to exercise forward, backward, and 3-way coalescing
  // of shifted and front-split blocks.
  Kfree(aligned64_shifted);
  Kfree(from_front_split);
  Kfree(aligned4096_front_split);
  Kfree(aligned64_zero_offset);

  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));
}

}  // namespace
}  // namespace protos
