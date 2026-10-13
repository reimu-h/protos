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
// Maximum canonical lower-half 48-bit physical/virtual address limit (128 TiB).
constexpr uintptr_t kMaxCanonicalIdentityAddress = 0x0000800000000000ULL;

// TLB maintenance: all functions below flush only the calling CPU's TLB (by
// reloading CR3); there is no cross-CPU shootdown. The mapping functions only
// add translations, which other CPUs cannot have cached, so they are safe on
// any CPU. (They also set missing write-enable bits, but every entry these
// functions and boot.S create is already writable.) `PagingMarkPageUncacheable`
// changes existing translations and is restricted to early boot.

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
// lies in physical page 0, beyond the canonical identity range, or in an
// unmapped page, or if no page table is available for the split.
//
// Only the calling CPU's TLB is flushed, so other CPUs could keep using stale
// cacheable translations (memory-type aliasing, Intel SDM Vol. 3A, 11.12.4).
// It must therefore run on the BSP before `SmpInit` binds per-CPU state and
// starts the APs; this is `DCHECK`ed with `CurrentCpuOrNull() == nullptr`.
bool PagingMarkPageUncacheable(uintptr_t phys_addr);

// Walks the active CR3 page tables and returns true if `addr` is mapped as an
// identity-mapped (`virtual == physical`) writable page.
bool PagingIsIdentityMapped(uintptr_t addr);

// Returns the upper bound of the contiguous identity-mapped physical address
// range starting at 0.
uintptr_t PagingIdentityMappedLimit();

}  // namespace protos

#endif  // PROTOS_PAGING_H_
