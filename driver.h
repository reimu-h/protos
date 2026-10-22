#ifndef PROTOS_DRIVER_H_
#define PROTOS_DRIVER_H_

#include <atomic>
#include <cstdint>
#include <memory>

#include "device_irq.h"
#include "pci.h"
#include "syscall_abi.h"

namespace protos {

class AddressSpace;
struct Task;

// Per-process driver state (`DriverContext`), user MMIO/DMA virtual windows,
// pure BAR/MSI-X validation helpers, and ordered device teardown. Nothing here
// knows any device class or device-specific register layout.

// Per-process user virtual address windows for `mmio_map` and `dma_alloc`,
// placed in the upper half well above the ELF image (`0xFFFF800000400000`) and
// user stack (`0xFFFF80007FFF0000`). Each mapping is followed by one unmapped
// 4 KiB guard page.
constexpr uintptr_t kDriverMmioWindowBase = 0xFFFF900000000000ULL;
constexpr uintptr_t kDriverMmioWindowSize = 1ULL << 30;  // 1 GiB
constexpr uintptr_t kDriverDmaWindowBase = 0xFFFFA00000000000ULL;
constexpr uintptr_t kDriverDmaWindowSize = 1ULL << 30;  // 1 GiB

// Environment seam for host unit tests and the kernel implementation.
class DriverEnvironment {
 public:
  virtual ~DriverEnvironment() = default;

  // Returns true if any 4 KiB page of `[phys_addr, phys_addr + size)` is usable
  // physical RAM (`PmmRangeIsValidUsableRam`).
  virtual bool RangeOverlapsUsableRam(uint64_t phys_addr, int64_t size) = 0;

  // Maps a 4 KiB device MMIO page at `user_addr` in `space` (`UC + NX +
  // kPteNotOwned`).
  virtual bool MapUserDevicePage(AddressSpace* space,  //
                                 uintptr_t user_addr,  //
                                 uintptr_t phys_addr) = 0;

  // Allocates `frame_count` physically contiguous 4 KiB frames strictly below
  // `max_phys_addr`, or returns 0 on failure.
  virtual uintptr_t AllocContiguousFrames(int64_t frame_count,
                                          uintptr_t max_phys_addr) = 0;

  // Frees `frame_count` contiguous frames starting at `base_phys`.
  virtual void FreeContiguousFrames(uintptr_t base_phys,
                                    int64_t frame_count) = 0;

  // Zeroes `[base_phys, base_phys + size)` before exposing it to user space.
  virtual void ZeroPhysicalRange(uintptr_t base_phys, int64_t size) = 0;

  // Maps a PMM-owned 4 KiB frame at `user_addr` in `space` as writable,
  // non-executable RAM (`UserPageAccess::kReadWrite`). On success, `space`
  // takes ownership of `frame_phys` and frees it when `space` is destroyed.
  virtual bool MapUserOwnedFrame(AddressSpace* space,  //
                                 uintptr_t user_addr,  //
                                 uintptr_t frame_phys) = 0;
};

#if !__STDC_HOSTED__
std::shared_ptr<DriverEnvironment> CreateKernelDriverEnvironment();
#endif

// Initializes the driver subsystem (`DeviceIrqInit` + `DriverEnvironment`).
// In the kernel, `kernel.cpp` calls `DriverInit()` once at boot after
// `PciInit()`.
void DriverInit(std::shared_ptr<DriverEnvironment> driver_env,
                std::shared_ptr<DeviceIrqEnvironment> irq_env);
#if !__STDC_HOSTED__
void DriverInit();
#endif

// Converts a kernel `PciDevice` record (`pci.h`) into the user-ABI
// `PciDeviceInfo` structure (`syscall_abi.h`).
void DriverFillPciDeviceInfo(const PciDevice& src, PciDeviceInfo* dst);

// Returns true if the 4 KiB physical page at page-aligned `page_phys` overlaps
// the MSI-X table or Pending Bit Array of `msix` (when `msix.valid` is true).
// User space must never map pages that hold MSI-X structures, which would let
// a user-mode driver forge arbitrary MSI writes.
bool DriverPageHoldsMsixStructure(const PciMsix& msix, uint64_t page_phys);

// Validates `[phys_addr, phys_addr + size)` for `mmio_map`:
// - `phys_addr` non-zero and 4 KiB-aligned, `size > 0`, 4 KiB-aligned, `<=
//   kSyscallMmioMaxSize`, no 64-bit overflow -> else `kSyscallErrorInvalid`.
// - Fully contained inside a sized memory BAR (`kMem32` or `kMem64`) of one of
//   `claimed_indices[0..claimed_count - 1]` -> else `kSyscallErrorAccess`.
// - Overlaps no MSI-X table or PBA page of that device -> else
//   `kSyscallErrorAccess`.
// Returns 0 on success and sets `*out_device_index` and `*out_bar_index` (if
// non-null).
int64_t DriverFindClaimedBarForRange(const int* claimed_indices,  //
                                     int claimed_count,           //
                                     uint64_t phys_addr,          //
                                     int64_t size,                //
                                     int* out_device_index,       //
                                     int* out_bar_index);

// Observability record populated by `DriverContext::Release()` so unit tests
// and boot self-tests can verify teardown ordering without fragile timing.
struct DriverReleaseRecord {
  int quiesced_count = 0;
  int freed_vectors = 0;
  int released_claims = 0;
  bool quiesced_before_vectors_freed = false;
  bool quiesced_before_address_space_destroyed = false;
};

// Returns the most recent `DriverReleaseRecord` from a `DriverContext` that had
// at least one claimed device when released.
DriverReleaseRecord DriverLastReleaseRecord();

class DriverContext {
 public:
  DriverContext() = default;
  ~DriverContext();

  DriverContext(const DriverContext&) = delete;
  DriverContext& operator=(const DriverContext&) = delete;

  // Non-zero owner token passed to `PciClaim`, `PciSetBusMaster`, `PciRelease`,
  // and `DeviceIrqAllocateVectors`.
  uintptr_t OwnerToken() const { return reinterpret_cast<uintptr_t>(this); }

  int64_t PciInfo(int64_t index, PciDeviceInfo* out_info) const;
  int64_t Claim(int64_t index);
  int64_t SetBusMaster(int64_t index, int64_t enable);
  int64_t MmioMap(AddressSpace* space, uint64_t phys_addr, int64_t size);
  int64_t DmaAlloc(AddressSpace* space,  //
                   int64_t size,         //
                   uint64_t flags,       //
                   uint64_t* out_phys);
  int64_t IrqAlloc(int64_t index,  //
                   int64_t count,  //
                   int64_t mode,   //
                   Task* owner_task);
  int64_t IrqWait(uint64_t mask,                   //
                  int64_t timeout_ticks,           //
                  bool (*should_stop)(void* ctx),  //
                  void* stop_ctx);
  bool BindVectorsForSelftest(int device_index, int count, Task* owner_task);

  // Quiesces all claimed devices, masks and unmaps any MSI-X tables, frees all
  // allocated IDT vectors, and releases all PCI claims in strict hardware-safe
  // order. Idempotent; `UserProcess::Run()` calls this while `address_space_`
  // is still alive (before DMA frames are freed).
  void Release(bool address_space_alive = true,
               DriverReleaseRecord* out_record = nullptr);

  bool IsClaimed(int index) const;
  int ClaimedCount() const { return claimed_count_; }
  int NextIrqBit() const { return next_irq_bit_; }
  int64_t DmaTotalBytes() const { return dma_total_bytes_; }
  uint64_t PendingIrqMask() const {
    return irq_pending_.load(std::memory_order_acquire);
  }

 private:
  int claimed_[kSyscallMaxClaims] = {};
  int claimed_count_ = 0;
  uintptr_t next_mmio_addr_ = kDriverMmioWindowBase;
  uintptr_t next_dma_addr_ = kDriverDmaWindowBase;
  int64_t dma_total_bytes_ = 0;
  int next_irq_bit_ = 0;
  std::atomic<uint64_t> irq_pending_{0};
  bool released_ = false;
};

#if __STDC_HOSTED__
void DriverResetForTest();
#endif

}  // namespace protos

#endif  // PROTOS_DRIVER_H_
