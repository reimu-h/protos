#ifndef PROTOS_HEAP_H_
#define PROTOS_HEAP_H_

// Kernel heap: a first-fit allocator for variable-sized blocks, carved out of
// physical frames that it requests from the PMM (see pmm.h).
//
// Contract:
// - `HeapInit` must be called exactly once, after `PmmInit` and before any
//   other heap function (including the global `operator new`). Calling it
//   again drops every existing arena without returning its frames to the PMM.
// - After `HeapInit`, all functions are safe to call concurrently from any CPU
//   and from interrupt context: they serialize on an IRQ-disabling spinlock.
//   The heap acquires the PMM lock while holding its own lock when it grows,
//   so the lock order is heap -> PMM; PMM code never calls into the heap.
// - Returned memory is not zeroed. It is identity-mapped physical RAM, 16-byte
//   aligned unless a larger alignment is requested.
// - Precondition violations (such as a negative size) are caught by `DCHECK`,
//   which every build in this project enables. `Kfree` panics via `CHECK` if
//   the header in front of the pointer is not a live heap block, which catches
//   double frees on a best-effort basis (the header of a freed block can be
//   overwritten once the block is coalesced or reused).
// - The heap never returns arenas to the PMM.

#include <cstddef>
#include <cstdint>

namespace protos {

// Minimum and default alignment (in bytes) guaranteed for all heap allocations.
// Equals `__STDCPP_DEFAULT_NEW_ALIGNMENT__` on x86-64.
constexpr int64_t kHeapAlignment = 16;
// Size of the arena that `HeapInit` allocates from the PMM, in 4 KiB frames
// (1 MiB).
constexpr int64_t kInitialHeapFrames = 256;
// Minimum number of 4 KiB frames that the heap requests from the PMM when it
// grows (64 KiB). Larger requests get exactly as many frames as they need.
constexpr int64_t kMinExpandFrames = 16;

// Initializes the kernel heap with an initial arena of `kInitialHeapFrames`
// physical frames from the PMM and caches `PmmMaxPhysicalAddress()`. Panics
// via `CHECK` if the initial arena cannot be allocated.
void HeapInit();

// Returns the sum of payload bytes across all free blocks currently in the
// heap's free-block tree.
int64_t HeapTotalFreeBytes();

// Allocates at least `size` bytes of 16-byte-aligned memory from the kernel
// heap and returns nullptr if out of memory. If `size == 0`, allocates a
// minimum 16-byte payload block. `size` must be >= 0 (`DCHECK`).
void* Kmalloc(int64_t size);

// Allocates at least `size` bytes of memory aligned to `alignment` and returns
// nullptr if `alignment` is not a power of two in `[1, kPageSize]` or the heap
// is out of memory. `size` must be >= 0 (`DCHECK`). Alignments above
// `kHeapAlignment` search the free blocks in O(n) time in the worst case,
// because a block's alignment padding depends on its address.
void* KmallocAligned(int64_t size, int64_t alignment);

// Frees a pointer previously returned by `Kmalloc` or `KmallocAligned` and
// coalesces it with any physically adjacent free blocks. Safe no-op if
// `ptr == nullptr`. Panics via `CHECK` if the block header in front of `ptr`
// is not a live heap block (for example on a double free); range and
// alignment of `ptr` are validated with `DCHECK`.
void Kfree(void* ptr);

// Allocation functions behind the kernel's global `operator new` forms
// (defined in heap.cpp for kernel builds only, so host tests call these
// directly). They take the unsigned `operator new` arguments; `alignment` is
// `kHeapAlignment` for the forms without a `std::align_val_t` argument.
//
// `KmallocForNewNothrow` backs the `std::nothrow_t` forms: it returns
// `KmallocAligned(size, alignment)`, or nullptr if `size` exceeds `INT64_MAX`,
// `alignment` is not a power of two in `[1, kPageSize]`, or the heap is out of
// memory.
void* KmallocForNewNothrow(size_t size, size_t alignment) noexcept;

// `KmallocForNew` backs the throwing forms. It never returns nullptr and
// panics via `CHECK` instead, because callers of a throwing `operator new`
// (including libstdc++'s `std::allocator` behind `std::make_shared`) use the
// result without checking it.
void* KmallocForNew(size_t size, size_t alignment);

#if __STDC_HOSTED__
// Acquires and releases the heap spinlock directly for host tests that verify
// lock-free queueing when `Kfree` observes a contended lock.
void HeapLockForTest();
void HeapUnlockForTest();
void HeapDrainAndUnlockForTest();

// Returns the number of queued blocks that no lock holder has returned to the
// free tree yet. Requires `HeapLockForTest()`.
int64_t HeapQueuedFreeCountForTest();
#endif

}  // namespace protos

#endif  // PROTOS_HEAP_H_
