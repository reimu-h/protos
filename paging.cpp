#include "paging.h"

#include <cstdint>

#include "check.h"
#include "pmm.h"
#include "smp.h"
#include "spinlock.h"

namespace protos {

namespace {

constexpr int kEntriesPerPageTable = 512;
// The first PML4 entry of the upper half, where user space is (see paging.h).
constexpr int kFirstUserPml4Entry =
    static_cast<int>((kUserSpaceBase >> 39) & 0x1FF);
static_assert(kFirstUserPml4Entry == kEntriesPerPageTable / 2);
// The kernel virtual ranges start where the identity map ends.
static_assert(kKernelVirtualBase == kIdentityMapLimit);
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
// Software-available PTE bit (bit 10) on present user PTEs: the physical page
// is a device MMIO page not owned by the PMM, so `PagingDestroyUserRoot` must
// not free it and `PagingUserRangeAccessible` must reject it for kernel user
// copies.
constexpr uint64_t kPteNotOwned = 1ULL << 10;

constexpr uint32_t kMsrIa32Pat = 0x00000277;
constexpr uint32_t kMsrIa32Efer = 0xC0000080;
constexpr uint64_t kEferNoExecuteEnable = 1ULL << 11;

IrqSpinLock g_paging_lock;

// The kernel root, set by `PagingInitKernelHalf()`, which runs before the APs
// start; 0 before. From then on the lower-half PML4 entries of every root are
// fixed.
uintptr_t g_kernel_root = 0;
// Whether EFER.NXE is set, which allows `kPteNoExecute` in page-table entries.
// Set by `PagingInitKernelHalf()` (boot.S sets EFER.NXE if the CPU supports
// it).
bool g_no_execute_enabled = false;

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

// Invalidates the calling CPU's TLB entries for the page that contains `addr`.
static void InvalidatePage(const uintptr_t addr) {
  asm volatile("invlpg [%0]" : : "r"(addr) : "memory");
}

static uint64_t ReadMsr(const uint32_t msr) {
  uint32_t low = 0;
  uint32_t high = 0;
  asm volatile("rdmsr" : "=a"(low), "=d"(high) : "c"(msr));
  return (static_cast<uint64_t>(high) << 32) | low;
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
  if (phys_addr >= kIdentityMapLimit ||
      (kIdentityMapLimit - phys_addr) < static_cast<uintptr_t>(size)) {
    return false;
  }

  const uintptr_t end_addr = phys_addr + size;
  if (end_addr > kIdentityMapLimit - kHugePageSize) {
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
      // Every lower-half PML4 entry is present once `PagingInitKernelHalf()`
      // has run, and the copies in user roots must stay valid.
      DCHECK(g_kernel_root == 0);
      const uintptr_t new_pdpt_phys = AllocZeroedPageTable(use_pmm);
      if (new_pdpt_phys == 0) {
        return false;
      }
      pml4[pml4_idx] = new_pdpt_phys | kPtePresent | kPteWritable;
      tlb_flush_needed = true;
    } else if ((pml4[pml4_idx] & kPteWritable) == 0) {
      DCHECK(g_kernel_root == 0);
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
  if (addr >= kIdentityMapLimit) {
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

// Returns true if `virt_addr` is a page-aligned address in the kernel virtual
// ranges.
static bool IsKernelVirtualPage(const uintptr_t virt_addr) {
  return virt_addr >= kKernelVirtualBase && virt_addr < kKernelVirtualEnd &&
         (virt_addr & (static_cast<uintptr_t>(kPageSize) - 1)) == 0;
}

// Returns the page table that the paging-structure entry `*entry` points to,
// first pointing it to a new zeroed table from `AllocZeroedPageTable(use_pmm)`
// if it is not present. Returns nullptr if `*entry` maps a large page or no
// table is available.
static uint64_t* EnsureNextTableLocked(uint64_t* const entry,
                                       const bool use_pmm) {
  if ((*entry & kPtePresent) == 0) {
    const uintptr_t table_phys = AllocZeroedPageTable(use_pmm);
    if (table_phys == 0) {
      return nullptr;
    }
    *entry = table_phys | kPtePresent | kPteWritable;
  } else if ((*entry & kPteHugePage) != 0) {
    return nullptr;
  }
  return reinterpret_cast<uint64_t*>(*entry & kPteAddressMask);
}

// Returns the page table that the present paging-structure entry `entry`
// points to, or nullptr if it is not present or maps a large page.
static uint64_t* NextTable(const uint64_t entry) {
  if ((entry & kPtePresent) == 0 || (entry & kPteHugePage) != 0) {
    return nullptr;
  }
  return reinterpret_cast<uint64_t*>(entry & kPteAddressMask);
}

// Returns the 4 KiB PT entry for `virt_addr` in the page tables of `root`, or
// nullptr if a table on the way to it is missing.
static uint64_t* FindPtEntryInRootLocked(const uintptr_t root,
                                         const uintptr_t virt_addr) {
  uint64_t* const pml4 = reinterpret_cast<uint64_t*>(root);
  uint64_t* const pdpt = NextTable(pml4[(virt_addr >> 39) & 0x1FF]);
  if (pdpt == nullptr) {
    return nullptr;
  }
  uint64_t* const pd = NextTable(pdpt[(virt_addr >> 30) & 0x1FF]);
  if (pd == nullptr) {
    return nullptr;
  }
  uint64_t* const pt = NextTable(pd[(virt_addr >> 21) & 0x1FF]);
  if (pt == nullptr) {
    return nullptr;
  }
  return &pt[(virt_addr >> 12) & 0x1FF];
}

// Returns the 4 KiB PT entry for the kernel virtual page `virt_addr`, or
// nullptr if the page tables on the way to it are missing. Every root shares
// the kernel half, so the active root is as good as any.
static uint64_t* FindKernelPtEntryLocked(const uintptr_t virt_addr) {
  const uintptr_t pml4_phys = ReadCr3();
  if (pml4_phys == 0) {
    return nullptr;
  }
  return FindPtEntryInRootLocked(pml4_phys, virt_addr);
}

// Returns true if `user_addr` is a page-aligned address in user space.
static bool IsUserPage(const uintptr_t user_addr) {
  return user_addr >= kUserSpaceBase && user_addr < kUserSpaceEnd &&
         (user_addr & (static_cast<uintptr_t>(kPageSize) - 1)) == 0;
}

// Returns true if `root` could be a user root: non-zero, page-aligned and not
// the kernel root.
static bool IsUserRoot(const uintptr_t root) {
  return root != 0 && (root & ~kPteAddressMask) == 0 && root != g_kernel_root;
}

// Allocates a zeroed page table for the user space of a root from the PMM.
// Unlike `AllocZeroedPageTable()`, it never falls back to the bootstrap pool,
// whose tables can't go back to the PMM when the root is destroyed. Returns 0
// if the PMM is out of frames.
static uintptr_t AllocZeroedUserPageTable() {
  const uintptr_t frame_phys = PmmAllocFrame();
  if (frame_phys == 0) {
    return 0;
  }
  uint64_t* const entries = reinterpret_cast<uint64_t*>(frame_phys);
  for (int i = 0; i < kEntriesPerPageTable; ++i) {
    entries[i] = 0;
  }
  return frame_phys;
}

// Returns the page table that the entry `*entry` in the user space of a root
// points to, first pointing it to a new zeroed table if it is not present. The
// entry allows user access and writes, so that the PT entry of each page alone
// decides how user code may access it. Returns nullptr if no table is
// available.
static uint64_t* EnsureUserTableLocked(uint64_t* const entry) {
  if ((*entry & kPtePresent) == 0) {
    const uintptr_t table_phys = AllocZeroedUserPageTable();
    if (table_phys == 0) {
      return nullptr;
    }
    *entry = table_phys | kPtePresent | kPteWritable | kPteUser;
  }
  return reinterpret_cast<uint64_t*>(*entry & kPteAddressMask);
}

// Appends the user page table at `table_phys` and everything below it to
// `*frames`: the tables it points to and, if it is a PT (`level` 1; 2 for a
// PD, 3 for a PDPT), the PMM-owned frames that it maps (`kPteNotOwned` clear).
static void CollectUserTable(const uintptr_t table_phys,  //
                             const int level,             //
                             PmmFrameList* const frames) {
  DCHECK(level >= 1 && level <= 3);
  DCHECK(frames != nullptr);
  const uint64_t* const entries = reinterpret_cast<const uint64_t*>(table_phys);
  for (int i = 0; i < kEntriesPerPageTable; ++i) {
    const uint64_t entry = entries[i];
    if ((entry & kPtePresent) == 0) {
      continue;
    }
    if (level == 1) {
      if ((entry & kPteNotOwned) == 0) {
        frames->Add(entry & kPteAddressMask);
      }
    } else {
      DCHECK((entry & kPteHugePage) == 0);
      CollectUserTable(entry & kPteAddressMask, level - 1, frames);
    }
  }
  frames->Add(table_phys);
}

// Returns true if user code may access the page at page-aligned `user_page` in
// `root` as normal RAM, and write to it if `write` is true: the entries on
// every level are present and allow user access (and writes), and the leaf PT
// entry is an owned RAM page (`kPteNotOwned` clear). User space has no large
// pages.
static bool IsUserPageAccessibleLocked(const uintptr_t root,
                                       const uintptr_t user_page,
                                       const bool write) {
  const uint64_t required =
      kPtePresent | kPteUser | (write ? kPteWritable : uint64_t{0});
  const uint64_t* table = reinterpret_cast<const uint64_t*>(root);
  for (int shift = 39;; shift -= 9) {
    const uint64_t entry = table[(user_page >> shift) & 0x1FF];
    if ((entry & required) != required) {
      return false;
    }
    if (shift == 12) {
      return (entry & kPteNotOwned) == 0;
    }
    if ((entry & kPteHugePage) != 0) {
      return false;
    }
    table = reinterpret_cast<const uint64_t*>(entry & kPteAddressMask);
  }
}

static bool MapUserPageWithFlags(const uintptr_t root,       //
                                 const uintptr_t user_addr,  //
                                 const uintptr_t phys_addr,  //
                                 const uint64_t pte_flags) {
  if (!IsUserRoot(root) || !IsUserPage(user_addr) || phys_addr == 0 ||
      (phys_addr & ~kPteAddressMask) != 0) {
    return false;
  }
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  uint64_t* const pml4 = reinterpret_cast<uint64_t*>(root);
  uint64_t* const pdpt =
      EnsureUserTableLocked(&pml4[(user_addr >> 39) & 0x1FF]);
  if (pdpt == nullptr) {
    return false;
  }
  uint64_t* const pd = EnsureUserTableLocked(&pdpt[(user_addr >> 30) & 0x1FF]);
  if (pd == nullptr) {
    return false;
  }
  uint64_t* const pt = EnsureUserTableLocked(&pd[(user_addr >> 21) & 0x1FF]);
  if (pt == nullptr) {
    return false;
  }
  uint64_t* const pte = &pt[(user_addr >> 12) & 0x1FF];
  if ((*pte & kPtePresent) != 0) {
    return false;
  }
  *pte = phys_addr | pte_flags;
  return true;
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
      max_physical_addr > kIdentityMapLimit - kHugePageSize) {
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
      phys_addr >= kIdentityMapLimit) {
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
      addr >= kIdentityMapLimit) {
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
  if (addr >= kIdentityMapLimit) {
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

bool PagingReserveKernelRange(const uintptr_t virt_addr, const int64_t size) {
  const bool use_pmm = PmmIsInitialized();
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  if (!IsKernelVirtualPage(virt_addr) || size <= 0 ||
      (size & (kPageSize - 1)) != 0 ||
      static_cast<uintptr_t>(size) > kKernelVirtualEnd - virt_addr) {
    return false;
  }
  const uintptr_t pml4_phys = ReadCr3();
  if (pml4_phys == 0) {
    return false;
  }
  uint64_t* const pml4 = reinterpret_cast<uint64_t*>(pml4_phys);
  const uintptr_t last_addr = virt_addr + static_cast<uintptr_t>(size - 1);
  // One PT maps a 2 MiB region; stepping by regions from the one that
  // contains `virt_addr` visits every PT of the range.
  for (uintptr_t addr = virt_addr & ~(kHugePageSize - 1);;
       addr += kHugePageSize) {
    uint64_t* const pml4_entry = &pml4[(addr >> 39) & 0x1FF];
    // See `MapHugePageRangeLocked()`.
    DCHECK(g_kernel_root == 0 || (*pml4_entry & kPtePresent) != 0);
    uint64_t* const pdpt = EnsureNextTableLocked(pml4_entry, use_pmm);
    if (pdpt == nullptr) {
      return false;
    }
    uint64_t* const pd =
        EnsureNextTableLocked(&pdpt[(addr >> 30) & 0x1FF], use_pmm);
    if (pd == nullptr) {
      return false;
    }
    if (EnsureNextTableLocked(&pd[(addr >> 21) & 0x1FF], use_pmm) == nullptr) {
      return false;
    }
    if (last_addr - addr < kHugePageSize) {
      return true;
    }
  }
}

bool PagingMapKernelPage(const uintptr_t virt_addr, const uintptr_t phys_addr) {
  if (!IsKernelVirtualPage(virt_addr) || phys_addr == 0 ||
      (phys_addr & ~kPteAddressMask) != 0) {
    return false;
  }
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  uint64_t* const pte = FindKernelPtEntryLocked(virt_addr);
  if (pte == nullptr || (*pte & kPtePresent) != 0) {
    return false;
  }
  *pte = phys_addr | kPtePresent | kPteWritable;
  return true;
}

bool PagingMapKernelMmioPage(const uintptr_t virt_addr,
                             const uintptr_t phys_addr) {
  if (!IsKernelVirtualPage(virt_addr) || phys_addr == 0 ||
      (phys_addr & ~kPteAddressMask) != 0) {
    return false;
  }
  uint64_t pte_flags =
      kPtePresent | kPteWritable | kPteWriteThrough | kPteCacheDisable;
  if (g_no_execute_enabled) {
    pte_flags |= kPteNoExecute;
  }
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  uint64_t* const pte = FindKernelPtEntryLocked(virt_addr);
  if (pte == nullptr || (*pte & kPtePresent) != 0) {
    return false;
  }
  *pte = phys_addr | pte_flags;
  return true;
}

uintptr_t PagingUnmapKernelPage(const uintptr_t virt_addr) {
  if (!IsKernelVirtualPage(virt_addr)) {
    return 0;
  }
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  uint64_t* const pte = FindKernelPtEntryLocked(virt_addr);
  if (pte == nullptr || (*pte & kPtePresent) == 0) {
    return 0;
  }
  const uintptr_t phys_addr = *pte & kPteAddressMask;
  *pte = 0;
  InvalidatePage(virt_addr);
  return phys_addr;
}

uintptr_t PagingKernelPageFrame(const uintptr_t virt_addr) {
  if (!IsKernelVirtualPage(virt_addr)) {
    return 0;
  }
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  const uint64_t* const pte = FindKernelPtEntryLocked(virt_addr);
  if (pte == nullptr || (*pte & kPtePresent) == 0) {
    return 0;
  }
  return *pte & kPteAddressMask;
}

bool PagingUserSpaceIsEmpty() {
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  const uintptr_t pml4_phys = ReadCr3();
  if (pml4_phys == 0) {
    return true;
  }
  const uint64_t* const pml4 = reinterpret_cast<const uint64_t*>(pml4_phys);
  for (int i = kFirstUserPml4Entry; i < kEntriesPerPageTable; ++i) {
    if ((pml4[i] & kPtePresent) != 0) {
      return false;
    }
  }
  return true;
}

void PagingFlushLocalTlb() { WriteCr3(ReadCr3()); }

void PagingInitKernelHalf() {
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  CHECK(g_kernel_root == 0);
  const uintptr_t root = ReadCr3();
  CHECK(root != 0);
  uint64_t* const pml4 = reinterpret_cast<uint64_t*>(root);
  for (int i = 0; i < kFirstUserPml4Entry; ++i) {
    if ((pml4[i] & kPtePresent) == 0) {
      const uintptr_t pdpt_phys = AllocZeroedPageTable(true);
      CHECK(pdpt_phys != 0);
      pml4[i] = pdpt_phys | kPtePresent | kPteWritable;
    } else {
      pml4[i] |= kPteWritable;
    }
  }
  g_no_execute_enabled = (ReadMsr(kMsrIa32Efer) & kEferNoExecuteEnable) != 0;
  g_kernel_root = root;
}

uintptr_t PagingKernelRoot() {
  DCHECK(g_kernel_root != 0);
  return g_kernel_root;
}

uintptr_t PagingActiveRoot() { return ReadCr3(); }

void PagingActivateRoot(const uintptr_t root) {
  DCHECK(root != 0 && (root & ~kPteAddressMask) == 0);
  WriteCr3(root);
}

uintptr_t PagingCreateUserRoot() {
  DCHECK(g_kernel_root != 0);
  const uintptr_t root = AllocZeroedUserPageTable();
  if (root == 0) {
    return 0;
  }
  // The kernel root's lower-half entries never change anymore (see
  // `PagingInitKernelHalf()`), so they can be copied without the lock.
  const uint64_t* const kernel_pml4 =
      reinterpret_cast<const uint64_t*>(g_kernel_root);
  uint64_t* const pml4 = reinterpret_cast<uint64_t*>(root);
  for (int i = 0; i < kFirstUserPml4Entry; ++i) {
    pml4[i] = kernel_pml4[i];
  }
  return root;
}

bool PagingMapUserPage(const uintptr_t root,       //
                       const uintptr_t user_addr,  //
                       const uintptr_t phys_addr,  //
                       const UserPageAccess access) {
  uint64_t pte_flags = kPtePresent | kPteUser;
  if (access == UserPageAccess::kReadWrite) {
    pte_flags |= kPteWritable;
  }
  if (access != UserPageAccess::kReadExecute && g_no_execute_enabled) {
    pte_flags |= kPteNoExecute;
  }
  return MapUserPageWithFlags(root, user_addr, phys_addr, pte_flags);
}

bool PagingMapUserDevicePage(const uintptr_t root,       //
                             const uintptr_t user_addr,  //
                             const uintptr_t phys_addr) {
  uint64_t pte_flags = kPtePresent | kPteUser | kPteWritable |
                       kPteWriteThrough | kPteCacheDisable | kPteNotOwned;
  if (g_no_execute_enabled) {
    pte_flags |= kPteNoExecute;
  }
  return MapUserPageWithFlags(root, user_addr, phys_addr, pte_flags);
}

uintptr_t PagingUserPageFrame(const uintptr_t root, const uintptr_t user_addr) {
  if (!IsUserRoot(root) || !IsUserPage(user_addr)) {
    return 0;
  }
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  const uint64_t* const pte = FindPtEntryInRootLocked(root, user_addr);
  if (pte == nullptr || (*pte & kPtePresent) == 0) {
    return 0;
  }
  return *pte & kPteAddressMask;
}

bool PagingQueryUserPage(const uintptr_t root,       //
                         const uintptr_t user_addr,  //
                         UserPageInfo* const info) {
  if (!IsUserRoot(root) || !IsUserPage(user_addr) || info == nullptr) {
    return false;
  }
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  const uint64_t* const pte = FindPtEntryInRootLocked(root, user_addr);
  if (pte == nullptr || (*pte & kPtePresent) == 0) {
    return false;
  }
  const uint64_t entry = *pte;
  info->phys_addr = entry & kPteAddressMask;
  info->writable = (entry & kPteWritable) != 0;
  info->executable = !g_no_execute_enabled || (entry & kPteNoExecute) == 0;
  info->write_through = (entry & kPteWriteThrough) != 0;
  info->cache_disable = (entry & kPteCacheDisable) != 0;
  info->pat = (entry & kPtePat) != 0;
  info->owned = (entry & kPteNotOwned) == 0;
  return true;
}

uint64_t PagingReadPatMsr() { return ReadMsr(kMsrIa32Pat); }

bool PagingUserRangeAccessible(const uintptr_t root,  //
                               const uintptr_t addr,  //
                               const int64_t size,    //
                               const bool write) {
  DCHECK(IsUserRange(addr, size));
  DCHECK(IsUserRoot(root));
  if (size == 0) {
    return true;
  }
  constexpr uintptr_t kPageMask = static_cast<uintptr_t>(kPageSize) - 1;
  const uintptr_t first_page = addr & ~kPageMask;
  // The range ends at `kUserSpaceEnd` at most, so its last byte is no overflow.
  const uintptr_t last_page =
      (addr + static_cast<uintptr_t>(size) - 1) & ~kPageMask;
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  for (uintptr_t page = first_page;; page += kPageMask + 1) {
    if (!IsUserPageAccessibleLocked(root, page, write)) {
      return false;
    }
    if (page == last_page) {
      return true;
    }
  }
}

bool PagingNoExecuteEnabled() {
  DCHECK(g_kernel_root != 0);
  return g_no_execute_enabled;
}

void PagingDestroyUserRoot(const uintptr_t root) {
  DCHECK(IsUserRoot(root));
  DCHECK(ReadCr3() != root);
  // Nothing else uses `root`, and its user page tables belong to it alone, so
  // the lock is not needed (and not held while many frames are freed).
  PmmFrameList frames;
  const uint64_t* const pml4 = reinterpret_cast<const uint64_t*>(root);
  for (int i = kFirstUserPml4Entry; i < kEntriesPerPageTable; ++i) {
    if ((pml4[i] & kPtePresent) != 0) {
      CollectUserTable(pml4[i] & kPteAddressMask, 3, &frames);
    }
  }
  frames.Add(root);
  PmmFreeFrameList(&frames);
}

}  // namespace protos
