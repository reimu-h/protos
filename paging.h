#ifndef PROTOS_PAGING_H_
#define PROTOS_PAGING_H_

#include <cstdint>

namespace protos {

// Size of a 2 MiB huge page mapped by a Page Directory (PD) entry with PS=1.
constexpr uintptr_t kHugePageSize = 0x200000;
// Page Directory (PD) entry Page Size (PS) bit indicating a 2 MiB huge page.
constexpr uint64_t kPageEntryHugePage = 1ULL << 7;
// Initial physical address range identity-mapped by `boot.S` before `PmmInit`
// runs (64 MiB = 32 * 2 MiB huge pages).
constexpr uintptr_t kBootstrapIdentityMapSize = 0x04000000;

// Virtual address layout
// ======================
//
// With 4-level paging, the canonical virtual addresses form two 128 TiB halves.
// The kernel uses the lower half and user space the upper half, the mirror
// image of the usual x86-64 layout. That works because the processor decides
// privilege only with the page-table user bit, not by half.
//
//   0x0000000000000000  identity map (virtual == physical) of physical memory,
//                       through which the kernel reaches its image, the heap
//                       and every PMM frame
//   0x0000400000000000  kernel virtual ranges, for example the task stacks
//                       (task_stack.h)
//   0x0000800000000000  non-canonical
//   0xFFFF800000000000  user space
//   0xFFFFFFFFFFFFF000  top page, never mapped
//
// The kernel maps nothing in the upper half, so PML4 entries 0..255 are the
// kernel's and 256..511 user space's. The top page of the address space stays
// unmapped, so that the end address of every user range is representable
// (`kUserSpaceEnd` at most), a user buffer that is accessed upward faults
// before its address can wrap around to 0 (into the kernel), and no value from
// -4095 to -1 is a user address.

// Exclusive upper bound of the identity map, 64 TiB (the size of Linux's
// direct map with 4-level paging): physical memory at or above it can't be
// identity-mapped. `PagingIdentityMappedLimit()` is how far the identity map
// reaches now. Host unit tests use host memory as physical memory, and host
// memory can lie anywhere in the host's lower half, so host builds accept all
// of it.
#if __STDC_HOSTED__
constexpr uintptr_t kIdentityMapLimit = 0x0000800000000000ULL;
#else
constexpr uintptr_t kIdentityMapLimit = 0x0000400000000000ULL;
#endif
// The kernel virtual ranges (see `PagingReserveKernelRange`): from the end of
// the identity map to the end of the lower half.
constexpr uintptr_t kKernelVirtualBase = 0x0000400000000000ULL;
constexpr uintptr_t kKernelVirtualEnd = 0x0000800000000000ULL;
// User space: the upper half, without its top page.
constexpr uintptr_t kUserSpaceBase = 0xFFFF800000000000ULL;
constexpr uintptr_t kUserSpaceEnd = 0xFFFFFFFFFFFFF000ULL;

// Returns true if `[addr, addr + size)` lies in user space, for `size >= 0`.
// Never overflows: the end address is not computed.
constexpr bool IsUserRange(const uintptr_t addr, const int64_t size) {
  return size >= 0 && addr >= kUserSpaceBase && addr <= kUserSpaceEnd &&
         static_cast<uint64_t>(size) <= kUserSpaceEnd - addr;
}

// TLB maintenance: all functions below flush only the calling CPU's TLB (by
// reloading CR3 or with INVLPG); `SmpTlbShootdown()` (smp.h) flushes the TLBs
// of all CPUs. The mapping functions only add translations, which other CPUs
// cannot have cached, so they are safe on any CPU. (They also set missing
// write-enable bits, but every entry these functions and boot.S create is
// already writable.) `PagingMarkPageUncacheable` and `PagingUnmapGuardPage`
// change existing translations and are restricted to early boot.
// `PagingUnmapKernelPage` leaves the shootdown to its caller.
//
// Guard pages: `PagingUnmapGuardPage` unmaps the 4 KiB page below a stack, so
// that a stack overflow faults instead of overwriting the memory below it. The
// mapping functions leave guard pages unmapped, even inside the ranges they are
// asked to map; only `PagingRemapGuardPage` maps one again.

// Identity-maps all 2 MiB huge pages overlapping `[phys_addr, phys_addr +
// size)` using the pre-reserved bootstrap page-table pool in `.bss` (safe to
// call before `PmmInit` completes). Does not advance
// `PagingIdentityMappedLimit()`.
bool PagingMapBootstrapRange(uintptr_t phys_addr, int64_t size);

// Ensures that all pages overlapping `[phys_addr, phys_addr + size)` are
// present and writable in the active identity map, handling 2 MiB PD entries
// that are already mapped as 2 MiB huge pages (`kPageEntryHugePage`) as well
// as 2 MiB PD entries that have been split into 4 KiB page tables.
bool PagingEnsureIdentityMappedRange(uintptr_t phys_addr, int64_t size);

// Extends the active 4-level identity mapping (`virtual_addr == physical_addr`)
// from `0` up to `max_physical_addr` (rounded up to `kHugePageSize`),
// allocating zeroed 4 KiB page-table frames (`PDPT` and `PD`) from the PMM
// whenever a `1 GiB` or `512 GiB` boundary is crossed.
// Returns true if the entire `[0, max_physical_addr)` range is mapped.
bool PagingExtendIdentityMap(uintptr_t max_physical_addr);

// Marks the single 4 KiB identity-mapped physical page containing `phys_addr`
// as uncacheable (`PWT | PCD`), splitting the enclosing 2 MiB huge-page PD
// entry into 512 4 KiB PT entries if necessary. Returns false if `phys_addr`
// lies in physical page 0, at or above `kIdentityMapLimit`, or in an unmapped
// page, or if no page table is available for the split.
//
// Only the calling CPU's TLB is flushed, so other CPUs could keep using stale
// cacheable translations (memory-type aliasing, Intel SDM Vol. 3A, 11.12.4).
// It must therefore run on the BSP before `SmpInit` binds per-CPU state and
// starts the APs; this is `DCHECK`ed with `CurrentCpuOrNull() == nullptr`.
bool PagingMarkPageUncacheable(uintptr_t phys_addr);

// Unmaps the identity-mapped 4 KiB page at page-aligned `addr` as a guard page,
// splitting the enclosing 2 MiB huge page into 512 4 KiB pages if necessary.
// The page table for the split comes from the PMM once `PmmInit` has run, and
// from the bootstrap pool before. Returns false if `addr` is not page-aligned,
// lies in physical page 0 or at or above `kIdentityMapLimit`, is not
// identity-mapped, or if no page table is available for the split.
//
// Only the calling CPU's TLB is flushed, so other CPUs could still reach the
// page through stale translations. It must therefore run before `SmpInit`
// starts the APs.
bool PagingUnmapGuardPage(uintptr_t addr);

// Maps the guard page at `addr` again (identity, writable), for example before
// its frame goes back to the PMM. Returns false if `addr` is not the start of a
// guard page. This only adds a translation, so it is safe on any CPU.
bool PagingRemapGuardPage(uintptr_t addr);

// Returns true if `addr` lies in a guard page.
bool PagingIsGuardPage(uintptr_t addr);

// Walks the active CR3 page tables and returns true if `addr` is mapped as an
// identity-mapped (`virtual == physical`) writable page.
bool PagingIsIdentityMapped(uintptr_t addr);

// Returns the upper bound of the contiguous identity-mapped physical address
// range starting at 0.
uintptr_t PagingIdentityMappedLimit();

// Kernel virtual ranges: besides the identity map, the kernel maps single 4 KiB
// pages at virtual addresses in `[kKernelVirtualBase, kKernelVirtualEnd)`, for
// example the task stacks (task_stack.h). The pages are writable, kernel-only
// and not global, so a CR3 reload flushes them from the TLB.

// Allocates the page tables (PDPT, PD and PT levels) for the 4 KiB pages of
// `[virt_addr, virt_addr + size)` and leaves every page unmapped, so that
// `PagingMapKernelPage` and `PagingUnmapKernelPage` never allocate in that
// range and work in any context, also with interrupts disabled. The tables
// come from the PMM once `PmmInit` has run (from the bootstrap pool if it is
// out of frames) and are never freed; tables that exist already are kept.
// `virt_addr` must be page-aligned, `size` a positive multiple of the page
// size, and the range must lie in the kernel virtual ranges. Returns false if
// the arguments are invalid or a page table can't be allocated (the tables
// allocated so far stay in place).
bool PagingReserveKernelRange(uintptr_t virt_addr, int64_t size);

// Maps the 4 KiB page at page-aligned `virt_addr`, in a range reserved with
// `PagingReserveKernelRange`, to the frame at non-zero page-aligned
// `phys_addr`. Returns false if the arguments are invalid, the page tables for
// `virt_addr` are missing, or the page is already mapped. This only adds a
// translation, so it needs no TLB flush and is safe on any CPU.
bool PagingMapKernelPage(uintptr_t virt_addr, uintptr_t phys_addr);

// Unmaps the 4 KiB page at page-aligned `virt_addr` that `PagingMapKernelPage`
// mapped, invalidates it in the calling CPU's TLB, and returns the physical
// address of its frame, or 0 if the page was not mapped. Other CPUs may keep
// using the old translation until the next `SmpTlbShootdown()` that starts
// after this call, so the caller must not map `virt_addr` again before such a
// shootdown has returned. If nothing accesses `virt_addr` until then, the
// frame can be reused at once: a stale translation that is never used does no
// harm.
uintptr_t PagingUnmapKernelPage(uintptr_t virt_addr);

// Returns the physical address of the frame that the page at page-aligned
// `virt_addr` (in the kernel virtual ranges) maps to, or 0 if it is not mapped.
uintptr_t PagingKernelPageFrame(uintptr_t virt_addr);

// Returns true if the active page tables map nothing in user space, that is,
// if no PML4 entry of the upper half is present.
bool PagingUserSpaceIsEmpty();

// Flushes the calling CPU's TLB, paging-structure caches included, by
// reloading CR3. The kernel maps no global pages, so this drops every cached
// translation. Takes no lock, so interrupt handlers may call it.
void PagingFlushLocalTlb();

// Page-table roots and user pages
// ===============================
//
// A root is the physical address of a PML4, which CR3 points to. The kernel
// root is the PML4 that boot.S builds and that every CPU starts on; it maps
// nothing in user space. Every user address space (address_space.h) has a root
// of its own: its lower half is a copy of the kernel root's, and its upper half
// maps the address space's user pages.
//
// The copies stay valid because `PagingInitKernelHalf()` points every
// lower-half PML4 entry of the kernel root to a page directory pointer table,
// after which these entries never change: later kernel mappings change only
// the tables below the PML4, which every root shares. The kernel mapping
// functions above therefore work the same whichever root is active.

// Points every lower-half PML4 entry of the kernel root to a page directory
// pointer table, allocating the missing ones from the PMM (at most 256 frames).
// Must run once, on the BSP while the kernel root is active, after `PmmInit()`
// and before the first `PagingCreateUserRoot()`. Panics via `CHECK` if a table
// can't be allocated.
void PagingInitKernelHalf();

// Returns the kernel root. Requires `PagingInitKernelHalf()` (`DCHECK`).
uintptr_t PagingKernelRoot();

// Returns the root that the calling CPU uses.
uintptr_t PagingActiveRoot();

// Switches the calling CPU to `root`, the kernel root or one from
// `PagingCreateUserRoot()`, by loading CR3. That also flushes the CPU's TLB,
// since the kernel maps no global pages.
void PagingActivateRoot(uintptr_t root);

// Creates a root that maps the kernel half and nothing in user space. Returns
// 0 if out of memory. Requires `PagingInitKernelHalf()` (`DCHECK`).
uintptr_t PagingCreateUserRoot();

// How user code may access a user page. No user page is both writable and
// executable.
enum class UserPageAccess : uint8_t {
  kRead,
  kReadWrite,
  kReadExecute,
};

// Maps the user page at page-aligned `user_addr`, in
// `[kUserSpaceBase, kUserSpaceEnd)`, in `root` (from `PagingCreateUserRoot()`)
// to the frame at non-zero page-aligned `phys_addr`, allocating missing page
// tables from the PMM. User code may access the page as `access` allows,
// except that every page is executable on CPUs without no-execute support. The
// root takes over the frame: `PagingDestroyUserRoot()` frees it. Returns false,
// taking nothing over, if an argument is invalid, the page is mapped already,
// or a page table can't be allocated. This only adds a translation, so it
// needs no TLB flush, even while CPUs use `root`.
bool PagingMapUserPage(uintptr_t root,       //
                       uintptr_t user_addr,  //
                       uintptr_t phys_addr,  //
                       UserPageAccess access);

// Returns the frame that the user page at page-aligned `user_addr` maps to in
// `root`, or 0 if it is not mapped.
uintptr_t PagingUserPageFrame(uintptr_t root, uintptr_t user_addr);

// Returns true if user code may access every byte of `[addr, addr + size)`
// (`IsUserRange(addr, size)`, `DCHECK`ed) in `root` (from
// `PagingCreateUserRoot()`): every page that the range overlaps is a mapped
// user page, and writable too if `write` is true. An empty range is
// accessible.
bool PagingUserRangeAccessible(uintptr_t root,  //
                               uintptr_t addr,  //
                               int64_t size,    //
                               bool write);

// Returns true if the CPUs support no-execute pages, so that user pages that
// `PagingMapUserPage()` maps without `kReadExecute` access are not executable.
// Requires `PagingInitKernelHalf()` (`DCHECK`).
bool PagingNoExecuteEnabled();

// Frees `root` (from `PagingCreateUserRoot()`) together with every frame
// mapped in its user space and the page tables of its user space. No CPU may
// use `root` anymore (`DCHECK`ed for the calling CPU), and no other call may
// use it at the same time. CPUs that used `root` before keep no translation
// from it: loading another root flushed their TLBs.
void PagingDestroyUserRoot(uintptr_t root);

}  // namespace protos

#endif  // PROTOS_PAGING_H_
