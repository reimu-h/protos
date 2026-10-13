#include "paging.h"

#include <cstdint>

#include "check.h"
#include "pmm.h"
#include "smp.h"
#include "spinlock.h"

namespace protos {

namespace {

constexpr int kEntriesPerPageTable = 512;
constexpr int kMaxBootstrapPageTables = 16;
constexpr uint64_t kPtePresent = 1ULL << 0;
constexpr uint64_t kPteWritable = 1ULL << 1;
constexpr uint64_t kPteUser = 1ULL << 2;
constexpr uint64_t kPteWriteThrough = 1ULL << 3;
constexpr uint64_t kPteCacheDisable = 1ULL << 4;
constexpr uint64_t kPteAccessed = 1ULL << 5;
constexpr uint64_t kPteDirty = 1ULL << 6;
constexpr uint64_t kPteHugePage = kPageEntryHugePage;
// The PAT bit is bit 7 in 4 KiB PTEs (where 2 MiB PDEs have PS) and bit 12 in
// 2 MiB PDEs.
constexpr uint64_t kPtePat = 1ULL << 7;
constexpr uint64_t kPdeHugePagePat = 1ULL << 12;
constexpr uint64_t kPteGlobal = 1ULL << 8;
constexpr uint64_t kPteNoExecute = 1ULL << 63;
constexpr uint64_t kPteAddressMask = 0x000FFFFFFFFFF000ULL;
constexpr uint64_t kHugePageAddressMask = 0x000FFFFFFFE00000ULL;
// Marks a guard page (see `PagingUnmapGuardPage`): a not-present 4 KiB PTE
// with this bit set, which keeps the page's physical address. The processor
// ignores every bit of a not-present entry except Present.
constexpr uint64_t kPteGuardPage = 1ULL << 9;

IrqSpinLock g_paging_lock;

// Static fallback pool of 4 KiB-aligned page tables in `.bss`, used for
// mappings made before `PmmInit` (the Multiboot info, ACPI tables, the GOP
// framebuffer, Local APIC MMIO) and whenever the PMM has no free frame.
alignas(4096) uint64_t
    g_bootstrap_page_tables[kMaxBootstrapPageTables][kEntriesPerPageTable];
int g_bootstrap_page_tables_used = 0;

uintptr_t g_identity_mapped_limit = kBootstrapIdentityMapSize;

// Rounds `value` up to the nearest multiple of `alignment` (power of two).
static constexpr uintptr_t AlignUp(const uintptr_t value,
                                   const uintptr_t alignment) {
  return (value + alignment - 1) & ~(alignment - 1);
}

// Reads the physical base address of the active PML4 table from CR3.
static uintptr_t ReadCr3() {
  uint64_t cr3 = 0;
  asm volatile("mov %0, cr3" : "=r"(cr3));
  return cr3 & kPteAddressMask;
}

// Writes `cr3_phys` to CR3, flushing non-global TLB entries.
static void WriteCr3(const uintptr_t cr3_phys) {
  const uint64_t value = cr3_phys;
  asm volatile("mov cr3, %0" : : "r"(value) : "memory");
}

// Allocates a single 4 KiB page-table frame and zeroes its 512 entries. The
// frame comes from the PMM when `use_pmm` is true, and from the static
// bootstrap pool when `use_pmm` is false or the PMM is out of frames. Zeroing
// needs the frame to be identity-mapped already, which holds because `PmmInit`
// adds only frames below the 64 MiB bootstrap mapping to its free tree until it
// has extended the identity map over all usable RAM. Returns the physical
// address of the table, or 0 if both sources are exhausted.
static uintptr_t AllocZeroedPageTable(const bool use_pmm) {
  uintptr_t frame_phys = use_pmm ? PmmAllocFrame() : 0;
  if (frame_phys == 0) {
    if (g_bootstrap_page_tables_used >= kMaxBootstrapPageTables) {
      return 0;
    }
    frame_phys = reinterpret_cast<uintptr_t>(
        g_bootstrap_page_tables[g_bootstrap_page_tables_used]);
    ++g_bootstrap_page_tables_used;
  }
  uint64_t* const entries = reinterpret_cast<uint64_t*>(frame_phys);
  for (int i = 0; i < kEntriesPerPageTable; ++i) {
    entries[i] = 0;
  }
  return frame_phys;
}

static bool MapHugePageRangeLocked(const uintptr_t phys_addr,  //
                                   const int64_t size,         //
                                   const bool use_pmm) {
  if (size <= 0) {
    return size == 0;
  }
  if (phys_addr >= kMaxCanonicalIdentityAddress ||
      (kMaxCanonicalIdentityAddress - phys_addr) <
          static_cast<uintptr_t>(size)) {
    return false;
  }

  const uintptr_t end_addr = phys_addr + size;
  if (end_addr > kMaxCanonicalIdentityAddress - kHugePageSize) {
    return false;
  }
  const uintptr_t start_page = phys_addr & ~(kHugePageSize - 1);
  const uintptr_t end_page = AlignUp(end_addr, kHugePageSize);

  const uintptr_t pml4_phys = ReadCr3();
  if (pml4_phys == 0) {
    return false;
  }
  uint64_t* const pml4 = reinterpret_cast<uint64_t*>(pml4_phys);
  bool tlb_flush_needed = false;

  for (uintptr_t addr = start_page; addr < end_page; addr += kHugePageSize) {
    const int pml4_idx = (addr >> 39) & 0x1FF;
    const int pdpt_idx = (addr >> 30) & 0x1FF;
    const int pd_idx = (addr >> 21) & 0x1FF;

    if ((pml4[pml4_idx] & kPtePresent) == 0) {
      const uintptr_t new_pdpt_phys = AllocZeroedPageTable(use_pmm);
      if (new_pdpt_phys == 0) {
        return false;
      }
      pml4[pml4_idx] = new_pdpt_phys | kPtePresent | kPteWritable;
      tlb_flush_needed = true;
    } else if ((pml4[pml4_idx] & kPteWritable) == 0) {
      pml4[pml4_idx] |= kPteWritable;
      tlb_flush_needed = true;
    }

    uint64_t* const pdpt =
        reinterpret_cast<uint64_t*>(pml4[pml4_idx] & kPteAddressMask);
    if ((pdpt[pdpt_idx] & kPtePresent) == 0) {
      const uintptr_t new_pd_phys = AllocZeroedPageTable(use_pmm);
      if (new_pd_phys == 0) {
        return false;
      }
      pdpt[pdpt_idx] = new_pd_phys | kPtePresent | kPteWritable;
      tlb_flush_needed = true;
    } else if ((pdpt[pdpt_idx] & kPteWritable) == 0) {
      pdpt[pdpt_idx] |= kPteWritable;
      tlb_flush_needed = true;
    }

    uint64_t* const pd =
        reinterpret_cast<uint64_t*>(pdpt[pdpt_idx] & kPteAddressMask);
    const uint64_t pde = pd[pd_idx];
    if ((pde & kPtePresent) == 0) {
      pd[pd_idx] = addr | kPtePresent | kPteWritable | kPageEntryHugePage;
      tlb_flush_needed = true;
      continue;
    }

    if ((pde & kPageEntryHugePage) != 0) {
      if ((pde & kHugePageAddressMask) != addr) {
        return false;
      }
      if ((pde & kPteWritable) == 0) {
        pd[pd_idx] = pde | kPteWritable;
        tlb_flush_needed = true;
      }
      continue;
    }

    if ((pde & kPteWritable) == 0) {
      pd[pd_idx] = pde | kPteWritable;
      tlb_flush_needed = true;
    }
    uint64_t* const pt = reinterpret_cast<uint64_t*>(pde & kPteAddressMask);
    for (int i = 0; i < kEntriesPerPageTable; ++i) {
      const uintptr_t page_phys = addr + static_cast<uintptr_t>(i) * kPageSize;
      const uint64_t pte = pt[i];
      if ((pte & kPtePresent) == 0) {
        if ((pte & kPteGuardPage) != 0) {
          continue;
        }
        pt[i] = page_phys | kPtePresent | kPteWritable;
        tlb_flush_needed = true;
      } else {
        if ((pte & kPteAddressMask) != page_phys) {
          return false;
        }
        if ((pte & kPteWritable) == 0) {
          pt[i] = pte | kPteWritable;
          tlb_flush_needed = true;
        }
      }
    }
  }

  if (tlb_flush_needed) {
    WriteCr3(pml4_phys);
  }
  return true;
}

// Returns the PD entry for `addr` in the active page tables, or nullptr if the
// PML4 or PDPT entry on the way to it is not present.
static uint64_t* FindPdEntryLocked(const uintptr_t addr) {
  const uintptr_t pml4_phys = ReadCr3();
  if (pml4_phys == 0) {
    return nullptr;
  }
  const int pml4_idx = (addr >> 39) & 0x1FF;
  const int pdpt_idx = (addr >> 30) & 0x1FF;
  const int pd_idx = (addr >> 21) & 0x1FF;

  uint64_t* const pml4 = reinterpret_cast<uint64_t*>(pml4_phys);
  if ((pml4[pml4_idx] & kPtePresent) == 0) {
    return nullptr;
  }
  uint64_t* const pdpt =
      reinterpret_cast<uint64_t*>(pml4[pml4_idx] & kPteAddressMask);
  if ((pdpt[pdpt_idx] & kPtePresent) == 0) {
    return nullptr;
  }
  uint64_t* const pd =
      reinterpret_cast<uint64_t*>(pdpt[pdpt_idx] & kPteAddressMask);
  return &pd[pd_idx];
}

// Returns the 4 KiB PT entry for `addr` if the PD entry `pde` points to a page
// table, or nullptr if it maps a 2 MiB page or nothing.
static uint64_t* FindPtEntry(const uint64_t pde, const uintptr_t addr) {
  if ((pde & kPtePresent) == 0 || (pde & kPteHugePage) != 0) {
    return nullptr;
  }
  uint64_t* const pt = reinterpret_cast<uint64_t*>(pde & kPteAddressMask);
  return &pt[(addr >> 12) & 0x1FF];
}

// Returns the 4 KiB PT entry that maps `addr`, first splitting the 2 MiB page
// that maps it into 512 4 KiB pages with identical attributes if necessary,
// with a page table from `AllocZeroedPageTable(use_pmm)`. The split flushes the
// TLB before the caller changes any of the 4 KiB pages, so that the TLB never
// holds a 2 MiB and a 4 KiB translation with different attributes for the same
// address (Intel SDM Vol. 3A, 4.10.2.3). Returns nullptr if `addr` is not
// mapped or no page table is available for the split.
static uint64_t* FindPtEntrySplittingLocked(const uintptr_t addr,
                                            const bool use_pmm) {
  uint64_t* const pde = FindPdEntryLocked(addr);
  if (pde == nullptr || (*pde & kPtePresent) == 0) {
    return nullptr;
  }
  if ((*pde & kPteHugePage) != 0) {
    const uintptr_t pt_phys = AllocZeroedPageTable(use_pmm);
    if (pt_phys == 0) {
      return nullptr;
    }
    uint64_t* const pt = reinterpret_cast<uint64_t*>(pt_phys);
    const uintptr_t huge_base = *pde & kHugePageAddressMask;
    constexpr uint64_t kSharedFlags = kPtePresent | kPteWritable | kPteUser |
                                      kPteWriteThrough | kPteCacheDisable |
                                      kPteAccessed | kPteDirty | kPteGlobal |
                                      kPteNoExecute;
    const uint64_t page_flags =
        (*pde & kSharedFlags) | (((*pde & kPdeHugePagePat) != 0) ? kPtePat : 0);
    for (int i = 0; i < kEntriesPerPageTable; ++i) {
      pt[i] = (huge_base + static_cast<uintptr_t>(i) * kPageSize) | page_flags;
    }
    *pde = pt_phys | kPtePresent | kPteWritable;
    WriteCr3(ReadCr3());
  }
  return FindPtEntry(*pde, addr);
}

// Returns the PT entry of the guard page that starts at `addr`, or nullptr if
// there is none.
static uint64_t* FindGuardPtEntryLocked(const uintptr_t addr) {
  if (addr >= kMaxCanonicalIdentityAddress) {
    return nullptr;
  }
  const uint64_t* const pde = FindPdEntryLocked(addr);
  uint64_t* const pte = (pde != nullptr) ? FindPtEntry(*pde, addr) : nullptr;
  if (pte == nullptr ||
      (*pte & (kPtePresent | kPteGuardPage)) != kPteGuardPage ||
      (*pte & kPteAddressMask) != addr) {
    return nullptr;
  }
  return pte;
}

}  // namespace

bool PagingMapBootstrapRange(const uintptr_t phys_addr, const int64_t size) {
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  return MapHugePageRangeLocked(phys_addr, size, false);
}

bool PagingEnsureIdentityMappedRange(const uintptr_t phys_addr,
                                     const int64_t size) {
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  return MapHugePageRangeLocked(phys_addr, size, true);
}

bool PagingExtendIdentityMap(const uintptr_t max_physical_addr) {
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  if (max_physical_addr == 0 ||
      max_physical_addr > kMaxCanonicalIdentityAddress - kHugePageSize) {
    return false;
  }

  const uintptr_t target_end = AlignUp(max_physical_addr, kHugePageSize);
  if (!MapHugePageRangeLocked(0, target_end, true)) {
    return false;
  }
  if (target_end > g_identity_mapped_limit) {
    g_identity_mapped_limit = target_end;
  }
  return true;
}

bool PagingMarkPageUncacheable(const uintptr_t phys_addr) {
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  if (phys_addr < static_cast<uintptr_t>(kPageSize) ||
      phys_addr >= kMaxCanonicalIdentityAddress) {
    return false;
  }
  // The TLB flushes below reach only the calling CPU (see paging.h).
  DCHECK(CurrentCpuOrNull() == nullptr);
  uint64_t* const pte = FindPtEntrySplittingLocked(phys_addr, true);
  if (pte == nullptr || (*pte & kPtePresent) == 0) {
    return false;
  }
  constexpr uint64_t kUcFlags = kPteWriteThrough | kPteCacheDisable;
  if ((*pte & kUcFlags) != kUcFlags) {
    *pte |= kUcFlags;
    WriteCr3(ReadCr3());
  }
  return true;
}

bool PagingUnmapGuardPage(const uintptr_t addr) {
  const bool use_pmm = PmmIsInitialized();
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  if ((addr & (static_cast<uintptr_t>(kPageSize) - 1)) != 0 || addr == 0 ||
      addr >= kMaxCanonicalIdentityAddress) {
    return false;
  }
  uint64_t* const pte = FindPtEntrySplittingLocked(addr, use_pmm);
  if (pte == nullptr || (*pte & kPtePresent) == 0 ||
      (*pte & kPteAddressMask) != addr) {
    return false;
  }
  *pte = addr | kPteGuardPage;
  WriteCr3(ReadCr3());
  return true;
}

bool PagingRemapGuardPage(const uintptr_t addr) {
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  uint64_t* const pte = FindGuardPtEntryLocked(addr);
  if (pte == nullptr) {
    return false;
  }
  *pte = addr | kPtePresent | kPteWritable;
  return true;
}

bool PagingIsGuardPage(const uintptr_t addr) {
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  return FindGuardPtEntryLocked(addr & kPteAddressMask) != nullptr;
}

bool PagingIsIdentityMapped(const uintptr_t addr) {
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  if (addr >= kMaxCanonicalIdentityAddress) {
    return false;
  }
  const uintptr_t pml4_phys = ReadCr3();
  if (pml4_phys == 0) {
    return false;
  }

  const int pml4_idx = (addr >> 39) & 0x1FF;
  const int pdpt_idx = (addr >> 30) & 0x1FF;
  const int pd_idx = (addr >> 21) & 0x1FF;

  // Every level must be present and writable.
  const uint64_t* const pml4 = reinterpret_cast<const uint64_t*>(pml4_phys);
  const uint64_t pml4e = pml4[pml4_idx];
  if ((pml4e & (kPtePresent | kPteWritable)) != (kPtePresent | kPteWritable)) {
    return false;
  }

  const uint64_t* const pdpt =
      reinterpret_cast<const uint64_t*>(pml4e & kPteAddressMask);
  const uint64_t pdpte = pdpt[pdpt_idx];
  if ((pdpte & (kPtePresent | kPteWritable)) != (kPtePresent | kPteWritable)) {
    return false;
  }

  const uint64_t* const pd =
      reinterpret_cast<const uint64_t*>(pdpte & kPteAddressMask);
  const uint64_t pde = pd[pd_idx];
  if ((pde & (kPtePresent | kPteWritable)) != (kPtePresent | kPteWritable)) {
    return false;
  }

  if ((pde & kPteHugePage) != 0) {
    const uintptr_t mapped_base = pde & kHugePageAddressMask;
    const uintptr_t expected_base = addr & ~(kHugePageSize - 1);
    return mapped_base == expected_base;
  }

  const int pt_idx = (addr >> 12) & 0x1FF;
  const uint64_t* const pt =
      reinterpret_cast<const uint64_t*>(pde & kPteAddressMask);
  const uint64_t pte = pt[pt_idx];
  if ((pte & (kPtePresent | kPteWritable)) != (kPtePresent | kPteWritable)) {
    return false;
  }
  const uintptr_t mapped_page = pte & kPteAddressMask;
  const uintptr_t expected_page =
      addr & ~(static_cast<uintptr_t>(kPageSize) - 1);
  return mapped_page == expected_page;
}

uintptr_t PagingIdentityMappedLimit() {
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  return g_identity_mapped_limit;
}

}  // namespace protos
