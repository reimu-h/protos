#include "driver.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <utility>

#include "address_space.h"
#include "check.h"
#include "device_irq.h"
#include "paging.h"
#include "pci.h"
#include "pmm.h"
#include "spinlock.h"
#include "syscall_abi.h"

namespace protos {
namespace {

constexpr uint64_t kPageMask = static_cast<uint64_t>(kPageSize) - 1;
constexpr uintptr_t kBelow4GiBLimit = 0x100000000ULL;

std::shared_ptr<DriverEnvironment> g_driver_env;
IrqSpinLock g_release_record_lock;
DriverReleaseRecord g_last_release_record{};

static PciBarInfoType ToAbiBarType(const PciBarType type) {
  switch (type) {
    case PciBarType::kNone:
      return PciBarInfoType::kNone;
    case PciBarType::kMem32:
      return PciBarInfoType::kMemory32;
    case PciBarType::kMem64:
      return PciBarInfoType::kMemory64;
    case PciBarType::kIo:
      return PciBarInfoType::kIo;
  }
  return PciBarInfoType::kNone;
}

static bool PageOverlapsRange(const uint64_t page_phys,   //
                              const uint64_t range_phys,  //
                              const int64_t range_bytes) {
  if (range_phys == 0 || range_bytes <= 0) {
    return false;
  }
  const uint64_t page_end = page_phys + static_cast<uint64_t>(kPageSize);
  const uint64_t range_end = range_phys + static_cast<uint64_t>(range_bytes);
  return page_phys < range_end && range_phys < page_end;
}

}  // namespace

void DriverInit(std::shared_ptr<DriverEnvironment> driver_env,
                std::shared_ptr<DeviceIrqEnvironment> irq_env) {
  CHECK(driver_env != nullptr);
  CHECK(irq_env != nullptr);
  g_driver_env = std::move(driver_env);
  DeviceIrqInit(std::move(irq_env));
}

#if !__STDC_HOSTED__
void DriverInit() {
  DriverInit(CreateKernelDriverEnvironment(),
             CreateKernelDeviceIrqEnvironment());
}
#endif

void DriverFillPciDeviceInfo(const PciDevice& src, PciDeviceInfo* const dst) {
  CHECK(dst != nullptr);
  *dst = PciDeviceInfo{};
  dst->bus = src.address.bus;
  dst->device = src.address.device;
  dst->function = src.address.function;
  dst->header_type =
      src.header_type | (src.multi_function ? kPciHeaderTypeMultiFunction : 0);
  dst->vendor_id = src.vendor_id;
  dst->device_id = src.device_id;
  dst->class_code = src.class_code;
  dst->subclass = src.subclass;
  dst->prog_if = src.prog_if;
  dst->revision = src.revision;
  dst->claimable = src.claimable ? 1 : 0;
  dst->msi_vectors = src.msi.offset != 0
                         ? static_cast<uint8_t>(1U << src.msi.vectors_log2)
                         : 0;
  dst->msix_vectors = (src.msix.offset != 0 && src.msix.valid)
                          ? static_cast<uint16_t>(src.msix.table_size)
                          : 0;
  for (int i = 0; i < kPciMaxBars; ++i) {
    dst->bars[i].type = ToAbiBarType(src.bars[i].type);
    dst->bars[i].prefetchable = src.bars[i].prefetchable ? 1 : 0;
    dst->bars[i].phys_base = src.bars[i].base;
    dst->bars[i].size = src.bars[i].size;
  }
  dst->parent_index = static_cast<int32_t>(src.parent_index);
  dst->msi_masking =
      (src.msi.offset != 0 && src.msi.per_vector_masking) ? 1 : 0;
}

bool DriverPageHoldsMsixStructure(const PciMsix& msix,
                                  const uint64_t page_phys) {
  if (msix.offset == 0 || !msix.valid) {
    return false;
  }
  const uint64_t aligned_page = page_phys & ~kPageMask;
  return PageOverlapsRange(aligned_page, msix.table_phys, msix.table_bytes) ||
         PageOverlapsRange(aligned_page, msix.pba_phys, msix.pba_bytes);
}

int64_t DriverFindClaimedBarForRange(const int* const claimed_indices,  //
                                     const int claimed_count,           //
                                     const uint64_t phys_addr,          //
                                     const int64_t size,                //
                                     int* const out_device_index,       //
                                     int* const out_bar_index) {
  if (phys_addr == 0 || (phys_addr & kPageMask) != 0 || size <= 0 ||
      (static_cast<uint64_t>(size) & kPageMask) != 0 ||
      size > kSyscallMmioMaxSize ||
      phys_addr > UINT64_MAX - static_cast<uint64_t>(size)) {
    return kSyscallErrorInvalid;
  }
  if (claimed_indices == nullptr || claimed_count <= 0) {
    return kSyscallErrorAccess;
  }

  for (int c = 0; c < claimed_count; ++c) {
    const int dev_idx = claimed_indices[c];
    const PciDevice* const dev = PciGetDevice(dev_idx);
    if (dev == nullptr) {
      continue;
    }
    for (int b = 0; b < kPciMaxBars; ++b) {
      if (!PciBarContainsRange(dev->bars[b], phys_addr, size)) {
        continue;
      }
      const int64_t pages = size / kPageSize;
      for (int64_t p = 0; p < pages; ++p) {
        const uint64_t page_phys =
            phys_addr + static_cast<uint64_t>(p) * kPageSize;
        if (DriverPageHoldsMsixStructure(dev->msix, page_phys)) {
          return kSyscallErrorAccess;
        }
      }
      if (out_device_index != nullptr) {
        *out_device_index = dev_idx;
      }
      if (out_bar_index != nullptr) {
        *out_bar_index = b;
      }
      return 0;
    }
  }
  return kSyscallErrorAccess;
}

DriverReleaseRecord DriverLastReleaseRecord() {
  const IrqSpinLockGuard guard(g_release_record_lock);
  return g_last_release_record;
}

DriverContext::~DriverContext() { Release(/*address_space_alive=*/false); }

bool DriverContext::IsClaimed(const int index) const {
  for (int i = 0; i < claimed_count_; ++i) {
    if (claimed_[i] == index) {
      return true;
    }
  }
  return false;
}

int64_t DriverContext::PciInfo(const int64_t index,
                               PciDeviceInfo* const out_info) const {
  const int count = PciDeviceCount();
  if (out_info == nullptr && index == 0) {
    return count;
  }
  if (index < 0 || index >= count) {
    return kSyscallErrorNoDevice;
  }
  if (out_info == nullptr) {
    return kSyscallErrorFault;
  }
  const PciDevice* const dev = PciGetDevice(static_cast<int>(index));
  if (dev == nullptr) {
    return kSyscallErrorNoDevice;
  }
  DriverFillPciDeviceInfo(*dev, out_info);
  return count;
}

int64_t DriverContext::Claim(const int64_t index) {
  if (index < 0 || index >= PciDeviceCount()) {
    return kSyscallErrorNoDevice;
  }
  const int dev_idx = static_cast<int>(index);
  if (IsClaimed(dev_idx)) {
    const PciStatus status = PciClaim(dev_idx, OwnerToken());
    return status == PciStatus::kOk ? 0 : kSyscallErrorBusy;
  }
  const PciDevice* const dev = PciGetDevice(dev_idx);
  if (dev == nullptr) {
    return kSyscallErrorNoDevice;
  }
  if (!dev->claimable) {
    return kSyscallErrorInvalid;
  }
  if (claimed_count_ >= kSyscallMaxClaims) {
    const uintptr_t cur_owner = PciOwner(dev_idx);
    if (cur_owner != kPciNoOwner && cur_owner != OwnerToken()) {
      return kSyscallErrorBusy;
    }
    return kSyscallErrorNoMemory;
  }

  const PciStatus status = PciClaim(dev_idx, OwnerToken());
  switch (status) {
    case PciStatus::kOk:
      DeviceIrqResetDevice(dev_idx);
      claimed_[claimed_count_++] = dev_idx;
      return 0;
    case PciStatus::kBusy:
      return kSyscallErrorBusy;
    case PciStatus::kNoDevice:
      return kSyscallErrorNoDevice;
    case PciStatus::kNotClaimable:
    case PciStatus::kNotOwner:
      return kSyscallErrorInvalid;
  }
  return kSyscallErrorInvalid;
}

int64_t DriverContext::SetBusMaster(const int64_t index, const int64_t enable) {
  if (enable != 0 && enable != 1) {
    return kSyscallErrorInvalid;
  }
  if (index < 0 || index >= PciDeviceCount()) {
    return kSyscallErrorNoDevice;
  }
  const PciStatus status =
      PciSetBusMaster(static_cast<int>(index), OwnerToken(), enable != 0);
  switch (status) {
    case PciStatus::kOk:
      return 0;
    case PciStatus::kNotOwner:
      return kSyscallErrorAccess;
    case PciStatus::kNoDevice:
      return kSyscallErrorNoDevice;
    case PciStatus::kNotClaimable:
    case PciStatus::kBusy:
      return kSyscallErrorInvalid;
  }
  return kSyscallErrorInvalid;
}

int64_t DriverContext::MmioMap(AddressSpace* const space,  //
                               const uint64_t phys_addr,   //
                               const int64_t size) {
  CHECK(g_driver_env != nullptr);
  int dev_idx = -1;
  int bar_idx = -1;
  const int64_t check_rc = DriverFindClaimedBarForRange(
      claimed_, claimed_count_, phys_addr, size, &dev_idx, &bar_idx);
  if (check_rc != 0) {
    return check_rc;
  }
  if (g_driver_env->RangeOverlapsUsableRam(phys_addr, size)) {
    return kSyscallErrorAccess;
  }

  const int64_t pages = size / kPageSize;
  const uintptr_t span_bytes =
      static_cast<uintptr_t>(pages + 1) * static_cast<uintptr_t>(kPageSize);
  const uintptr_t window_end = kDriverMmioWindowBase + kDriverMmioWindowSize;
  if (span_bytes > window_end - next_mmio_addr_) {
    return kSyscallErrorNoMemory;
  }

  const uintptr_t user_base = next_mmio_addr_;
  next_mmio_addr_ += span_bytes;
  for (int64_t p = 0; p < pages; ++p) {
    const uintptr_t va = user_base + static_cast<uintptr_t>(p) *
                                         static_cast<uintptr_t>(kPageSize);
    const uintptr_t pa =
        static_cast<uintptr_t>(phys_addr) +
        static_cast<uintptr_t>(p) * static_cast<uintptr_t>(kPageSize);
    if (!g_driver_env->MapUserDevicePage(space, va, pa)) {
      return kSyscallErrorNoMemory;
    }
  }
  return static_cast<int64_t>(user_base);
}

int64_t DriverContext::DmaAlloc(AddressSpace* const space,  //
                                const int64_t size,         //
                                const uint64_t flags,       //
                                uint64_t* const out_phys) {
  CHECK(g_driver_env != nullptr);
  if (size <= 0 || (static_cast<uint64_t>(size) & kPageMask) != 0 ||
      size > kSyscallDmaMaxSize || (flags & ~kSyscallDmaFlagsMask) != 0 ||
      out_phys == nullptr) {
    return kSyscallErrorInvalid;
  }
  if (dma_total_bytes_ + size > kSyscallDmaMaxTotalSize) {
    return kSyscallErrorNoMemory;
  }

  const int64_t pages = size / kPageSize;
  const uintptr_t span_bytes =
      static_cast<uintptr_t>(pages + 1) * static_cast<uintptr_t>(kPageSize);
  const uintptr_t window_end = kDriverDmaWindowBase + kDriverDmaWindowSize;
  if (span_bytes > window_end - next_dma_addr_) {
    return kSyscallErrorNoMemory;
  }

  const uintptr_t max_phys =
      (flags & kSyscallDmaBelow4GiB) != 0 ? kBelow4GiBLimit : kIdentityMapLimit;
  const uintptr_t phys_base =
      g_driver_env->AllocContiguousFrames(pages, max_phys);
  if (phys_base == 0) {
    return kSyscallErrorNoMemory;
  }

  g_driver_env->ZeroPhysicalRange(phys_base, size);

  const uintptr_t user_base = next_dma_addr_;
  next_dma_addr_ += span_bytes;
  for (int64_t p = 0; p < pages; ++p) {
    const uintptr_t va = user_base + static_cast<uintptr_t>(p) *
                                         static_cast<uintptr_t>(kPageSize);
    const uintptr_t pa = phys_base + static_cast<uintptr_t>(p) *
                                         static_cast<uintptr_t>(kPageSize);
    if (!g_driver_env->MapUserOwnedFrame(space, va, pa)) {
      g_driver_env->FreeContiguousFrames(pa, pages - p);
      return kSyscallErrorNoMemory;
    }
  }

  dma_total_bytes_ += size;
  *out_phys = static_cast<uint64_t>(phys_base);
  return static_cast<int64_t>(user_base);
}

int64_t DriverContext::IrqAlloc(const int64_t index,  //
                                const int64_t count,  //
                                const int64_t mode,   //
                                Task* const owner_task) {
  if (index < 0 || index >= PciDeviceCount()) {
    return kSyscallErrorNoDevice;
  }
  const int dev_idx = static_cast<int>(index);
  if (!IsClaimed(dev_idx)) {
    return kSyscallErrorAccess;
  }
  if (count < 1 || count > kSyscallIrqMaxCount ||
      (mode != kSyscallIrqModeAuto && mode != kSyscallIrqModeMsi &&
       mode != kSyscallIrqModeMsix)) {
    return kSyscallErrorInvalid;
  }
  const int int_count = static_cast<int>(count);
  if (next_irq_bit_ + int_count > kSyscallIrqMaxCount) {
    return kSyscallErrorNoMemory;
  }

  int vectors[kSyscallIrqMaxCount] = {};
  const DeviceIrqStatus alloc_status =
      DeviceIrqAllocateVectors(OwnerToken(), dev_idx, int_count, next_irq_bit_,
                               &irq_pending_, owner_task, vectors);
  if (alloc_status == DeviceIrqStatus::kBusy) {
    return kSyscallErrorBusy;
  }
  if (alloc_status == DeviceIrqStatus::kNoMemory) {
    return kSyscallErrorNoMemory;
  }
  if (alloc_status != DeviceIrqStatus::kOk) {
    return kSyscallErrorInvalid;
  }

  const DeviceIrqStatus prog_status =
      DeviceIrqProgramDevice(dev_idx, int_count, mode, vectors);
  if (prog_status != DeviceIrqStatus::kOk) {
    DeviceIrqFreeDeviceVectors(OwnerToken(), dev_idx);
    switch (prog_status) {
      case DeviceIrqStatus::kBusy:
        return kSyscallErrorBusy;
      case DeviceIrqStatus::kNoMemory:
        return kSyscallErrorNoMemory;
      case DeviceIrqStatus::kInvalid:
      case DeviceIrqStatus::kOk:
        return kSyscallErrorInvalid;
    }
  }

  const int first_bit = next_irq_bit_;
  next_irq_bit_ += int_count;
  return first_bit;
}

int64_t DriverContext::IrqWait(const uint64_t mask,                        //
                               const int64_t timeout_ticks,                //
                               bool (*const should_stop)(void* stop_ctx),  //
                               void* const stop_ctx) {
  return DeviceIrqWait(&irq_pending_, mask, timeout_ticks, should_stop,
                       stop_ctx);
}

bool DriverContext::BindVectorsForSelftest(const int device_index,  //
                                           const int count,         //
                                           Task* const owner_task) {
  if (!IsClaimed(device_index) || count < 1 || count > kSyscallIrqMaxCount ||
      next_irq_bit_ + count > kSyscallIrqMaxCount) {
    return false;
  }
  int vectors[kSyscallIrqMaxCount] = {};
  if (DeviceIrqAllocateVectors(OwnerToken(), device_index, count, next_irq_bit_,
                               &irq_pending_, owner_task,
                               vectors) != DeviceIrqStatus::kOk) {
    return false;
  }
  next_irq_bit_ += count;
  return true;
}

void DriverContext::Release(const bool address_space_alive,
                            DriverReleaseRecord* const out_record) {
  if (released_) {
    return;
  }
  released_ = true;

  DriverReleaseRecord record{};
  // Step 1: Quiesce every claimed device in PCI config space (stops DMA and
  // MSI/MSI-X generation and flushes posted writes with a non-posted config
  // read-back), then mask MSI-X table entries in MMIO while Memory Space is
  // still enabled and unmap the kernel MSI-X mapping.
  for (int i = 0; i < claimed_count_; ++i) {
    const int dev_idx = claimed_[i];
    PciQuiesce(dev_idx);
    DeviceIrqReleaseDevice(dev_idx);
    ++record.quiesced_count;
  }

  // Step 2: Unregister IDT handlers and free vectors in [0x50, 0x7F].
  const bool all_quiesced_before_free =
      (record.quiesced_count == claimed_count_);
  record.freed_vectors = DeviceIrqFreeVectors(OwnerToken());
  record.quiesced_before_vectors_freed =
      all_quiesced_before_free && (DeviceIrqFreeVectors(OwnerToken()) == 0) &&
      (DeviceIrqVectorForBit(OwnerToken(), 0) < 0);
  record.quiesced_before_address_space_destroyed = address_space_alive;

  // Step 3: Release PCI ownership so another driver can claim the devices.
  for (int i = 0; i < claimed_count_; ++i) {
    if (PciRelease(claimed_[i], OwnerToken()) == PciStatus::kOk) {
      ++record.released_claims;
    }
  }
  const int had_claims = claimed_count_;
  claimed_count_ = 0;

  if (had_claims > 0) {
    const IrqSpinLockGuard guard(g_release_record_lock);
    g_last_release_record = record;
  }
  if (out_record != nullptr) {
    *out_record = record;
  }
}

#if __STDC_HOSTED__
void DriverResetForTest() {
  {
    const IrqSpinLockGuard guard(g_release_record_lock);
    g_last_release_record = DriverReleaseRecord{};
  }
  DeviceIrqResetForTest();
  g_driver_env.reset();
}
#endif

#if !__STDC_HOSTED__
namespace {

class KernelDriverEnvironment final : public DriverEnvironment {
 public:
  bool RangeOverlapsUsableRam(const uint64_t phys_addr,
                              const int64_t size) override {
    const int64_t pages = size / kPageSize;
    for (int64_t p = 0; p < pages; ++p) {
      const uint64_t page_phys =
          phys_addr + static_cast<uint64_t>(p) * kPageSize;
      if (page_phys < kIdentityMapLimit &&
          PmmRangeIsValidUsableRam(static_cast<uintptr_t>(page_phys),
                                   kPageSize)) {
        return true;
      }
    }
    return false;
  }

  bool MapUserDevicePage(AddressSpace* const space,  //
                         const uintptr_t user_addr,  //
                         const uintptr_t phys_addr) override {
    return space != nullptr && space->MapDevicePage(user_addr, phys_addr);
  }

  uintptr_t AllocContiguousFrames(const int64_t frame_count,
                                  const uintptr_t max_phys_addr) override {
    const uintptr_t base = PmmAllocFrames(frame_count);
    if (base == 0) {
      return 0;
    }
    const uintptr_t bytes =
        static_cast<uintptr_t>(frame_count) * static_cast<uintptr_t>(kPageSize);
    if (base > max_phys_addr || bytes > max_phys_addr - base) {
      PmmFreeFrames(base, frame_count);
      return 0;
    }
    return base;
  }

  void FreeContiguousFrames(const uintptr_t base_phys,
                            const int64_t frame_count) override {
    PmmFreeFrames(base_phys, frame_count);
  }

  void ZeroPhysicalRange(const uintptr_t base_phys,
                         const int64_t size) override {
    uint64_t* const words = reinterpret_cast<uint64_t*>(base_phys);
    const int64_t word_count = size / static_cast<int64_t>(sizeof(uint64_t));
    for (int64_t i = 0; i < word_count; ++i) {
      words[i] = 0;
    }
  }

  bool MapUserOwnedFrame(AddressSpace* const space,  //
                         const uintptr_t user_addr,  //
                         const uintptr_t frame_phys) override {
    return space != nullptr && space->MapOwnedFrame(user_addr, frame_phys,
                                                    UserPageAccess::kReadWrite);
  }
};

}  // namespace

std::shared_ptr<DriverEnvironment> CreateKernelDriverEnvironment() {
  return std::make_shared<KernelDriverEnvironment>();
}
#endif

}  // namespace protos
