#ifndef PROTOS_PMM_H_
#define PROTOS_PMM_H_

// Physical memory manager (PMM): hands out contiguous runs of 4 KiB physical
// frames from the usable RAM described by the Multiboot memory map.
//
// Contract:
// - `PmmInit` must run exactly once, on the BSP, before any other CPU is
//   started and before any other PMM function (and before `HeapInit`). It
//   mutates the PMM state without locking.
// - After `PmmInit`, all functions are safe to call concurrently from any CPU
//   and from interrupt context. Allocation and free serialize on an
//   IRQ-disabling spinlock; the getters for values that `PmmInit` fixes read
//   them without locking. The PMM never calls into the heap or paging code, so
//   it is always last in the lock order (heap -> PMM, paging -> PMM).
// - Frame addresses are physical addresses inside the identity map
//   (virtual == physical) that `PmmInit` extends over all usable RAM, so they
//   can be dereferenced directly. Frames are not zeroed.
// - Invalid arguments are caught by `DCHECK`, which every build in this project
//   enables. Freeing frames that overlap frames that are already free (for
//   example a double free) panics via `CHECK` even if `DCHECK`s are disabled.

#include <cstdint>

#if __STDC_HOSTED__
#include <vector>
#endif

namespace protos {

struct MultibootMemoryMap;

// Physical frame size (4 KiB).
constexpr int64_t kPageSize = 4096;
// Physical addresses below 1 MiB (BIOS, real-mode IVT/BDA, VGA buffer at
// 0xB8000) are always reserved and never handed out by the frame allocator.
constexpr uintptr_t kLowerMemoryLimit = 0x100000;

// Parses the Multiboot memory map, carves out lower memory, the kernel image,
// the framebuffer, and the Multiboot information structures (the info block
// and, for Multiboot1, the memory map buffer), extends the identity mapping
// across usable physical RAM, and populates the intrusive Red-Black Tree of
// free frame runs. Logs the memory map and RAM totals to the console and, if
// the bootloader provided a linear framebuffer, switches the VGA console to it
// (`VgaInitFramebuffer`). Panics via `CHECK` if the memory map cannot be
// parsed, no usable physical frames are available, or identity mapping fails.
//
// Multiboot modules and the Multiboot1 command line and boot loader name
// strings are not reserved: their memory is handed out like any other free RAM
// and gets overwritten. Booting with modules requires reserving them first.
void PmmInit(uint32_t multiboot_magic, uint64_t multiboot_info_addr);

// Returns true once `PmmInit` has set up the frame allocator.
bool PmmIsInitialized();

// Allocates `count` contiguous 4 KiB physical frames (`count > 0`, `DCHECK`)
// from the lowest-addressed free run that is large enough and returns the base
// physical address, or 0 if no contiguous run of `count` frames is available.
uintptr_t PmmAllocFrames(int64_t count);

// Allocates a single 4 KiB physical frame and returns its physical address,
// or 0 if out of memory.
uintptr_t PmmAllocFrame();

// Frees `count` contiguous 4 KiB physical frames starting at page-aligned
// `base_addr` and coalesces the range with physically adjacent free runs in
// O(log n) time. Validates with `DCHECK` that the range is non-empty,
// page-aligned, and within usable RAM, and panics via `CHECK` if it overlaps
// frames that are already free.
void PmmFreeFrames(uintptr_t base_addr, int64_t count);

// Frees a single 4 KiB physical frame at page-aligned `frame_addr`.
void PmmFreeFrame(uintptr_t frame_addr);

// Accumulates single 4 KiB physical frames to free in one batch without
// taking the PMM lock on each frame. `Add` coalesces adjacent frames into
// runs and links the runs through the last 16 bytes of each run's first
// frame, so the caller must own every frame added and must not touch its last
// 16 bytes afterwards, and any `FreeRunNode` at offset 0 of an accidentally
// double-freed frame is left intact for `PmmFreeFrameList` to detect.
struct PmmFrameList {
  uintptr_t head_addr = 0;
  uintptr_t tail_addr = 0;
  uintptr_t cur_base = 0;
  int64_t cur_count = 0;

  void Add(uintptr_t frame_addr);
};

// Frees all frames accumulated in `*list` and resets `*list` to empty. If the
// PMM lock is uncontended, coalesces the runs into the free-run tree
// immediately; if another CPU holds the lock, enqueues the runs on an atomic
// lock-free list that the next lock holder drains before allocating, freeing,
// or counting frames (and before releasing the PMM lock).
void PmmFreeFrameList(PmmFrameList* list);

// Returns true if `[addr, addr + size)` lies strictly within a usable RAM
// region above 1 MiB and does not overlap the kernel image, framebuffer, or
// Multiboot data.
bool PmmRangeIsValidUsableRam(uintptr_t addr, int64_t size);

// Returns the exclusive upper bound (end address) of the highest usable
// physical RAM interval found by `PmmInit`; the highest managed frame starts
// at `PmmMaxPhysicalAddress() - kPageSize`.
uintptr_t PmmMaxPhysicalAddress();

// Returns the total number of 4 KiB frame slots up to the highest usable
// physical address (`PmmMaxPhysicalAddress() / kPageSize`).
int64_t PmmMaxFrameCount();

// Returns the number of currently free 4 KiB frames.
int64_t PmmFreeFrameCount();

// Returns the total number of usable 4 KiB frames that `PmmInit` placed under
// PMM management, including frames that were allocated during `PmmInit` (for
// example page tables for the identity map extension).
int64_t PmmTotalUsableFrameCount();

#if __STDC_HOSTED__
// Resets PMM internal state (`g_pmm_initialized`, free-run tree, and frame
// counters) for hermetic host unit tests.
void PmmResetForTest();

// Acquires and releases the PMM spinlock directly for host tests that verify
// lock-free queueing when `PmmFreeFrameList` observes a contended lock.
void PmmLockForTest();
void PmmUnlockForTest();
void PmmDrainAndUnlockForTest();

// Returns the number of queued frames that no lock holder has added to the
// free runs yet. Requires `PmmLockForTest()`.
int64_t PmmQueuedFrameCountForTest();

// Half-open physical address interval `[start, end)`.
struct PmmIntervalForTest {
  uintptr_t start;
  uintptr_t end;

  bool operator==(const PmmIntervalForTest&) const = default;
};

// Resets the PMM state and returns the sorted, disjoint, page-aligned usable
// intervals that `PmmInit` would populate for `memory_map`, without touching
// the memory they describe (for host unit tests of the interval arithmetic).
std::vector<PmmIntervalForTest> PmmCollectUsableIntervalsForTest(
    const MultibootMemoryMap& memory_map);
#endif

}  // namespace protos

#endif  // PROTOS_PMM_H_
