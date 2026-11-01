#include "driver.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <utility>

#include "address_space.h"
#include "check.h"
#include "device_irq.h"
#include "ioapic.h"
#include "paging.h"
#include "pci.h"
#include "pmm.h"
#include "spinlock.h"
#include "syscall_abi.h"
#if !__STDC_HOSTED__
#include "console.h"
#include "port_io.h"
#include "smp.h"
#include "task.h"
#endif

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

static const char* PowerStateName(const PciPowerState state) {
  switch (state) {
    case PciPowerState::kD0:
      return "D0";
    case PciPowerState::kD1:
      return "D1";
    case PciPowerState::kD2:
      return "D2";
    case PciPowerState::kD3hot:
      return "D3hot";
  }
  return "?";
}

// A console line about a PCI function, "[PCI] bb:dd.f: <text>\n", built in
// place; text beyond its capacity is cut off.
class DeviceLogLine {
 public:
  explicit DeviceLogLine(const PciDevice& dev) {
    Append("[PCI] ");
    AppendHex(dev.address.bus, 2);
    Append(":");
    AppendHex(dev.address.device, 2);
    Append(".");
    AppendHex(dev.address.function, 1);
    Append(": ");
  }

  DeviceLogLine& Append(const char* text) {
    while (*text != '\0' && length_ < kCapacity - 2) {
      buffer_[length_++] = *text++;
    }
    return *this;
  }

  // Lowercase hexadecimal, without a prefix, `digits` digits.
  DeviceLogLine& AppendHex(const uint32_t value, const int digits) {
    for (int i = digits - 1; i >= 0 && length_ < kCapacity - 2; --i) {
      buffer_[length_++] = "0123456789abcdef"[(value >> (4 * i)) & 0xF];
    }
    return *this;
  }

  // Ends the line and writes it to the kernel console.
  void Emit() {
    buffer_[length_++] = '\n';
    buffer_[length_] = '\0';
    g_driver_env->Log(buffer_);
  }

 private:
  static constexpr int kCapacity = 112;
  char buffer_[kCapacity] = {};
  int length_ = 0;
};

// How long, at most, PowerUpFunction() waits after the recovery time for a
// function to answer configuration reads again: the 1 s within which PCI
// Express requires a function to complete them after a Conventional Reset
// [PCIe Base 6.0 §6.6.1]. It polls like Linux's pci_dev_wait(), sleeping 1, 2,
// 4, ... ms in between.
constexpr int64_t kPowerUpReadyTimeoutMicroseconds = 1000000;
constexpr int64_t kPowerUpFirstPollMicroseconds = 1000;

// Waits, without a lock held, until function `index` answers configuration
// reads (PciFunctionResponds()), for at most about
// kPowerUpReadyTimeoutMicroseconds. Returns whether it answers.
static bool WaitUntilFunctionResponds(const int index) {
  int64_t waited_us = 0;
  for (int64_t poll_us = kPowerUpFirstPollMicroseconds;; poll_us *= 2) {
    if (PciFunctionResponds(index)) {
      return true;
    }
    if (waited_us >= kPowerUpReadyTimeoutMicroseconds) {
      return false;
    }
    g_driver_env->SleepMicroseconds(poll_us);
    waited_us += poll_us;
  }
}

// Puts function `index` (a device that `owner` has just claimed, or a bridge
// above it) into D0 (pci.h), in case firmware left it in a low-power state
// where it decodes no memory accesses and forwards nothing to the buses behind
// it. Waits out its recovery time and until it answers again, without a lock
// held, and then writes back the configuration that the transition reset (or,
// when `force_restore` is set because an ancestor bridge just woke, applies the
// claim-time configuration that PciClaim() deferred while the bridge was
// asleep). Logs "[PCI] bb:dd.f: woke from <state>". Returns whether `index`
// left a low-power state.
static bool PowerUpFunction(const int index,        //
                            const uintptr_t owner,  //
                            const bool force_restore = false) {
  const StatusOr<PciPowerTransition, PciStatus> transition =
      PciSetPowerStateD0(index, owner);
  if (!transition.ok()) {
    return false;
  }
  const PciPowerState from = transition->from;
  if (from == PciPowerState::kD0) {
    if (force_restore) {
      const StatusOr<bool, PciStatus> resized =
          PciRestoreAfterPowerUp(index, owner);
      if (resized.ok() && *resized) {
        CHECK(g_driver_env != nullptr);
        DeviceLogLine(*PciGetDevice(index))
            .Append("corrected BAR sizes in D0")
            .Emit();
      }
    }
    return false;
  }
  CHECK(g_driver_env != nullptr);
  const PciDevice* const dev = PciGetDevice(index);
  const int64_t delay_us = PciPowerUpDelayMicroseconds(from);
  if (delay_us > 0) {
    g_driver_env->SleepMicroseconds(delay_us);
  }
  // A function that is still not ready drops the writes below, as if it had
  // never been woken; its driver then finds it not decoding.
  if (!WaitUntilFunctionResponds(index)) {
    DeviceLogLine(*dev)
        .Append("not ready after leaving ")
        .Append(PowerStateName(from))
        .Emit();
  }
  const StatusOr<bool, PciStatus> resized =
      PciRestoreAfterPowerUp(index, owner, &transition->context);
  DeviceLogLine(*dev).Append("woke from ").Append(PowerStateName(from)).Emit();
  if (resized.ok() && *resized) {
    DeviceLogLine(*dev).Append("corrected BAR sizes in D0").Emit();
  }
  return true;
}

// Puts device `dev_idx`, which `owner` has just claimed, into D0, and first
// every bridge between it and its root bus, from the root bus down: a bridge
// in D1, D2 or D3hot forwards no requests to its secondary bus, so nothing
// behind it can be woken or reached [PCIe Base 6.0 §5.3.1.4.1] (as Linux's
// pci_enable_bridge() wakes the upstream bridges first).
static void PowerUpClaimedDevice(const int dev_idx, const uintptr_t owner) {
  int depth = 0;
  for (int bridge = PciGetDevice(dev_idx)->parent_index; bridge != kPciNoParent;
       bridge = PciGetDevice(bridge)->parent_index) {
    ++depth;
  }
  bool ancestor_woke = false;
  for (int level = depth; level >= 0; --level) {
    int index = dev_idx;
    for (int i = 0; i < level; ++i) {
      index = PciGetDevice(index)->parent_index;
    }
    if (PowerUpFunction(index, owner, level == 0 && ancestor_woke)) {
      ancestor_woke = true;
    }
  }
}

// Applies the claim-time fixups of device `dev_idx`, which `owner` has just
// claimed and woken (PciApplyClaimQuirks()), and logs what they changed.
static void ApplyClaimQuirks(const int dev_idx, const uintptr_t owner) {
  const StatusOr<PciClaimQuirks, PciStatus> quirks =
      PciApplyClaimQuirks(dev_idx, owner);
  if (!quirks.ok() || !quirks->intel_usb_ports_routed) {
    return;
  }
  CHECK(g_driver_env != nullptr);
  DeviceLogLine(*PciGetDevice(dev_idx))
      .Append("routed USB ports from EHCI: SuperSpeed 0x")
      .AppendHex(quirks->usb3_ports, 8)
      .Append(", USB 2.0 0x")
      .AppendHex(quirks->usb2_ports, 8)
      .Emit();
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

PciDeviceInfo DriverFillPciDeviceInfo(const PciDevice& src) {
  PciDeviceInfo dst{};
  dst.bus = src.address.bus;
  dst.device = src.address.device;
  dst.function = src.address.function;
  dst.header_type =
      src.header_type | (src.multi_function ? kPciHeaderTypeMultiFunction : 0);
  dst.vendor_id = src.vendor_id;
  dst.device_id = src.device_id;
  dst.class_code = src.class_code;
  dst.subclass = src.subclass;
  dst.prog_if = src.prog_if;
  dst.revision = src.revision;
  dst.claimable = src.claimable ? 1 : 0;
  dst.msi_vectors = src.msi.offset != 0
                        ? static_cast<uint8_t>(1U << src.msi.vectors_log2)
                        : 0;
  dst.msix_vectors = (src.msix.offset != 0 && src.msix.valid)
                         ? static_cast<uint16_t>(src.msix.table_size)
                         : 0;
  for (int i = 0; i < kPciMaxBars; ++i) {
    dst.bars[i].type = ToAbiBarType(src.bars[i].type);
    dst.bars[i].prefetchable = src.bars[i].prefetchable ? 1 : 0;
    dst.bars[i].phys_base = src.bars[i].base;
    dst.bars[i].size = src.bars[i].size;
  }
  dst.parent_index = static_cast<int32_t>(src.parent_index);
  dst.msi_masking = (src.msi.offset != 0 && src.msi.per_vector_masking) ? 1 : 0;
  return dst;
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

StatusOr<ClaimedBarMatch> DriverFindClaimedBarForRange(
    const int* const claimed_indices,  //
    const int claimed_count,           //
    const uint64_t phys_addr,          //
    const int64_t size) {
  if (phys_addr == 0 || (phys_addr & kPageMask) != 0 || size <= 0 ||
      (static_cast<uint64_t>(size) & kPageMask) != 0 ||
      size > kSyscallMmioMaxSize ||
      phys_addr > UINT64_MAX - static_cast<uint64_t>(size)) {
    return Error(kSyscallErrorInvalid);
  }
  if (claimed_indices == nullptr || claimed_count <= 0) {
    return Error(kSyscallErrorAccess);
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
          return Error(kSyscallErrorAccess);
        }
      }
      return ClaimedBarMatch{.device_index = dev_idx, .bar_index = b};
    }
  }
  return Error(kSyscallErrorAccess);
}

DriverReleaseRecord DriverLastReleaseRecord() {
  const IrqSpinLockGuard guard(g_release_record_lock);
  return g_last_release_record;
}

DriverContext::~DriverContext() {
  (void)Release(/*address_space_alive=*/false);
}

int DriverContext::FindClaimedSlotLocked(const int index) const {
  for (int i = 0; i < claimed_count_; ++i) {
    if (claimed_[i] == index) {
      return i;
    }
  }
  return -1;
}

bool DriverContext::IsClaimed(const int index) const {
  const IrqSpinLockGuard guard(lock_);
  return FindClaimedSlotLocked(index) >= 0;
}

bool DriverContext::IsIsaClaimed(const int device) const {
  if (device < 0 || device >= kIsaDeviceCount) {
    return false;
  }
  const IrqSpinLockGuard guard(lock_);
  return isa_claimed_[device];
}

StatusOr<DriverPciInfoResult> DriverContext::PciInfo(
    const int64_t index) const {
  const int count = PciDeviceCount();
  if (index < 0 || index >= count) {
    return Error(kSyscallErrorNoDevice);
  }
  // Under the PCI lock: another process's claim may be correcting the record
  // (PciRestoreAfterPowerUp()).
  const StatusOr<PciDevice> dev = PciCopyDevice(static_cast<int>(index));
  if (!dev.ok()) {
    return Error(kSyscallErrorNoDevice);
  }
  return DriverPciInfoResult{.count = count,
                             .info = DriverFillPciDeviceInfo(*dev)};
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
  if (ClaimedCount() >= kSyscallMaxClaims) {
    const uintptr_t cur_owner = PciOwner(dev_idx);
    if (cur_owner != kPciNoOwner && cur_owner != OwnerToken()) {
      return kSyscallErrorBusy;
    }
    return kSyscallErrorNoMemory;
  }

  const PciStatus status = PciClaim(dev_idx, OwnerToken());
  switch (status) {
    case PciStatus::kOk: {
      PowerUpClaimedDevice(dev_idx, OwnerToken());
      ApplyClaimQuirks(dev_idx, OwnerToken());
      DeviceIrqResetDevice(dev_idx);
      // Only the process's own task claims devices and releases them.
      const IrqSpinLockGuard guard(lock_);
      DCHECK(!released_ &&
             claimed_count_ + isa_claimed_count_ < kSyscallMaxClaims);
      claimed_[claimed_count_] = dev_idx;
      irq_allocated_[claimed_count_] = false;
      ++claimed_count_;
      return 0;
    }
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
  int claimed_snapshot[kSyscallMaxClaims] = {};
  int claimed_count_snapshot = 0;
  {
    const IrqSpinLockGuard guard(lock_);
    claimed_count_snapshot = claimed_count_;
    for (int i = 0; i < claimed_count_snapshot; ++i) {
      claimed_snapshot[i] = claimed_[i];
    }
  }
  const StatusOr<ClaimedBarMatch> bar_match = DriverFindClaimedBarForRange(
      claimed_snapshot, claimed_count_snapshot, phys_addr, size);
  if (!bar_match.ok()) {
    return bar_match.status();
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
  for (int64_t p = 0; p < pages; ++p) {
    const uintptr_t va = user_base + static_cast<uintptr_t>(p) *
                                         static_cast<uintptr_t>(kPageSize);
    const uintptr_t pa =
        static_cast<uintptr_t>(phys_addr) +
        static_cast<uintptr_t>(p) * static_cast<uintptr_t>(kPageSize);
    if (!g_driver_env->MapUserDevicePage(space, va, pa)) {
      if (p > 0) {
        next_mmio_addr_ = user_base + static_cast<uintptr_t>(p + 1) *
                                          static_cast<uintptr_t>(kPageSize);
      }
      return kSyscallErrorNoMemory;
    }
  }
  next_mmio_addr_ = user_base + span_bytes;
  return static_cast<int64_t>(user_base);
}

StatusOr<DriverDmaAllocation> DriverContext::DmaAlloc(
    AddressSpace* const space,  //
    const int64_t size,         //
    const uint64_t flags) {
  CHECK(g_driver_env != nullptr);
  if (size <= 0 || (static_cast<uint64_t>(size) & kPageMask) != 0 ||
      size > kSyscallDmaMaxSize || (flags & ~kSyscallDmaFlagsMask) != 0) {
    return Error(kSyscallErrorInvalid);
  }
  if (dma_total_bytes_ + size > kSyscallDmaMaxTotalSize) {
    return Error(kSyscallErrorNoMemory);
  }

  const int64_t pages = size / kPageSize;
  const uintptr_t span_bytes =
      static_cast<uintptr_t>(pages + 1) * static_cast<uintptr_t>(kPageSize);
  const uintptr_t window_end = kDriverDmaWindowBase + kDriverDmaWindowSize;
  if (span_bytes > window_end - next_dma_addr_) {
    return Error(kSyscallErrorNoMemory);
  }

  const uintptr_t max_phys =
      (flags & kSyscallDmaBelow4GiB) != 0 ? kBelow4GiBLimit : kIdentityMapLimit;
  const uintptr_t phys_base =
      g_driver_env->AllocContiguousFrames(pages, max_phys);
  if (phys_base == 0) {
    return Error(kSyscallErrorNoMemory);
  }

  g_driver_env->ZeroPhysicalRange(phys_base, size);

  const uintptr_t user_base = next_dma_addr_;
  for (int64_t p = 0; p < pages; ++p) {
    const uintptr_t va = user_base + static_cast<uintptr_t>(p) *
                                         static_cast<uintptr_t>(kPageSize);
    const uintptr_t pa = phys_base + static_cast<uintptr_t>(p) *
                                         static_cast<uintptr_t>(kPageSize);
    if (!g_driver_env->MapUserOwnedFrame(space, va, pa)) {
      g_driver_env->FreeContiguousFrames(pa, pages - p);
      if (p > 0) {
        next_dma_addr_ = user_base + static_cast<uintptr_t>(p + 1) *
                                         static_cast<uintptr_t>(kPageSize);
        dma_total_bytes_ += p * kPageSize;
      }
      return Error(kSyscallErrorNoMemory);
    }
  }

  next_dma_addr_ = user_base + span_bytes;
  dma_total_bytes_ += size;
  return DriverDmaAllocation{.user_addr = user_base,
                             .phys_addr = static_cast<uint64_t>(phys_base)};
}

int64_t DriverContext::ReserveIrqBitsLocked(const int index, const int count) {
  const int slot = FindClaimedSlotLocked(index);
  if (released_ || slot < 0) {
    return kSyscallErrorAccess;
  }
  if (irq_allocated_[slot]) {
    return kSyscallErrorBusy;
  }
  if (next_irq_bit_ + count > kSyscallIrqMaxCount) {
    return kSyscallErrorNoMemory;
  }
  irq_allocated_[slot] = true;
  const int first_bit = next_irq_bit_;
  next_irq_bit_ += count;
  return first_bit;
}

void DriverContext::UnreserveIrqBitsLocked(const int index, const int first_bit,
                                           const int count) {
  const int slot = FindClaimedSlotLocked(index);
  if (slot >= 0) {
    irq_allocated_[slot] = false;
  }
  // Bits reserved since then stay reserved, and these stay unused.
  if (next_irq_bit_ == first_bit + count) {
    next_irq_bit_ = first_bit;
  }
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
  int64_t reserved = 0;
  {
    const IrqSpinLockGuard guard(lock_);
    reserved = ReserveIrqBitsLocked(dev_idx, int_count);
  }
  if (reserved < 0) {
    return reserved;
  }
  const int first_bit = static_cast<int>(reserved);

  // Without `lock_`: mapping the MSI-X table may need a TLB shootdown, which
  // must not run under an IrqSpinLock.
  int vectors[kSyscallIrqMaxCount] = {};
  DeviceIrqStatus status = DeviceIrqAllocateVectors(OwnerToken(),   //
                                                    dev_idx,        //
                                                    int_count,      //
                                                    first_bit,      //
                                                    &irq_pending_,  //
                                                    owner_task,     //
                                                    vectors);
  if (status == DeviceIrqStatus::kOk) {
    status = DeviceIrqProgramDevice(dev_idx, int_count, mode, vectors);
    if (status != DeviceIrqStatus::kOk) {
      DeviceIrqFreeDeviceVectors(OwnerToken(), dev_idx);
    }
  }
  if (status != DeviceIrqStatus::kOk) {
    const IrqSpinLockGuard guard(lock_);
    UnreserveIrqBitsLocked(dev_idx, first_bit, int_count);
  }
  switch (status) {
    case DeviceIrqStatus::kOk:
      return first_bit;
    case DeviceIrqStatus::kBusy:
      return kSyscallErrorBusy;
    case DeviceIrqStatus::kNoDevice:
      return kSyscallErrorNoDevice;
    case DeviceIrqStatus::kNoMemory:
      return kSyscallErrorNoMemory;
    case DeviceIrqStatus::kInvalid:
      return kSyscallErrorInvalid;
  }
  return kSyscallErrorInvalid;
}

int64_t DriverContext::IrqWait(const uint64_t mask,                        //
                               const int64_t timeout_ticks,                //
                               bool (*const should_stop)(void* stop_ctx),  //
                               void* const stop_ctx) {
  return DeviceIrqWait(&irq_pending_, mask, timeout_ticks, should_stop,
                       stop_ctx);
}

int64_t DriverContext::IsaClaim(const int64_t device) {
  if (device < 0 || device >= kIsaDeviceCount) {
    return kSyscallErrorNoDevice;
  }
  const int isa_dev = static_cast<int>(device);
  if (IsIsaClaimed(isa_dev)) {
    const IsaStatus status = protos::IsaClaim(isa_dev, OwnerToken());
    return status == IsaStatus::kOk ? 0 : kSyscallErrorBusy;
  }
  if (ClaimedCount() >= kSyscallMaxClaims) {
    const uintptr_t cur_owner = IsaOwner(isa_dev);
    if (cur_owner != kPciNoOwner && cur_owner != OwnerToken()) {
      return kSyscallErrorBusy;
    }
    return kSyscallErrorNoMemory;
  }

  const IsaStatus status = protos::IsaClaim(isa_dev, OwnerToken());
  switch (status) {
    case IsaStatus::kOk: {
      DeviceIrqResetIsaDevice(isa_dev);
      const IrqSpinLockGuard guard(lock_);
      DCHECK(!released_ &&
             claimed_count_ + isa_claimed_count_ < kSyscallMaxClaims);
      isa_claimed_[isa_dev] = true;
      isa_irq_allocated_[isa_dev] = false;
      ++isa_claimed_count_;
      return 0;
    }
    case IsaStatus::kBusy:
      return kSyscallErrorBusy;
    case IsaStatus::kNoDevice:
      return kSyscallErrorNoDevice;
    case IsaStatus::kNotOwner:
      return kSyscallErrorInvalid;
  }
  return kSyscallErrorInvalid;
}

int64_t DriverContext::IsaPortIo(const int64_t op,     //
                                 const uint64_t port,  //
                                 const uint64_t value) {
  CHECK(g_driver_env != nullptr);
  if ((op != kSyscallPortIoRead8 && op != kSyscallPortIoWrite8) ||
      port > 0xFFFFu) {
    return kSyscallErrorInvalid;
  }
  if (op == kSyscallPortIoRead8 && value != 0) {
    return kSyscallErrorInvalid;
  }
  if (op == kSyscallPortIoWrite8 && value > 0xFFu) {
    return kSyscallErrorInvalid;
  }
  const uint16_t port16 = static_cast<uint16_t>(port);
  const int isa_dev = IsaFindDeviceForPort(port16);
  if (isa_dev < 0 || !IsIsaClaimed(isa_dev)) {
    return kSyscallErrorAccess;
  }
  if (op == kSyscallPortIoRead8) {
    return static_cast<int64_t>(g_driver_env->PortRead8(port16));
  }
  g_driver_env->PortWrite8(port16, static_cast<uint8_t>(value));
  return 0;
}

int64_t DriverContext::IsaIrqAlloc(const int64_t device,
                                   Task* const owner_task) {
  if (device < 0 || device >= kIsaDeviceCount) {
    return kSyscallErrorNoDevice;
  }
  const int isa_dev = static_cast<int>(device);
  if (!IsIsaClaimed(isa_dev)) {
    return kSyscallErrorAccess;
  }
  int bit = -1;
  {
    const IrqSpinLockGuard guard(lock_);
    if (released_ || !isa_claimed_[isa_dev]) {
      return kSyscallErrorAccess;
    }
    if (isa_irq_allocated_[isa_dev]) {
      return kSyscallErrorBusy;
    }
    if (next_irq_bit_ + 1 > kSyscallIrqMaxCount) {
      return kSyscallErrorNoMemory;
    }
    isa_irq_allocated_[isa_dev] = true;
    bit = next_irq_bit_++;
  }

  const int slot_id = DeviceIrqIsaSlotId(isa_dev);
  int vector = -1;
  DeviceIrqStatus status = DeviceIrqAllocateVectors(OwnerToken(),   //
                                                    slot_id,        //
                                                    1,              //
                                                    bit,            //
                                                    &irq_pending_,  //
                                                    owner_task,     //
                                                    &vector);
  if (status == DeviceIrqStatus::kOk) {
    status = DeviceIrqProgramIsaDevice(isa_dev, vector);
    if (status != DeviceIrqStatus::kOk) {
      DeviceIrqFreeDeviceVectors(OwnerToken(), slot_id);
    }
  }
  if (status != DeviceIrqStatus::kOk) {
    const IrqSpinLockGuard guard(lock_);
    isa_irq_allocated_[isa_dev] = false;
    if (next_irq_bit_ == bit + 1) {
      next_irq_bit_ = bit;
    }
  }
  switch (status) {
    case DeviceIrqStatus::kOk:
      return bit;
    case DeviceIrqStatus::kBusy:
      return kSyscallErrorBusy;
    case DeviceIrqStatus::kNoDevice:
      return kSyscallErrorNoDevice;
    case DeviceIrqStatus::kNoMemory:
      return kSyscallErrorNoMemory;
    case DeviceIrqStatus::kInvalid:
      return kSyscallErrorInvalid;
  }
  return kSyscallErrorInvalid;
}

bool DriverContext::BindVectorsForSelftest(const int device_index,  //
                                           const int count,         //
                                           Task* const owner_task) {
  if (count < 1 || count > kSyscallIrqMaxCount) {
    return false;
  }
  // Under `lock_` throughout (allocating vectors maps nothing), so that a
  // concurrent Release() either comes first or frees these vectors too.
  const IrqSpinLockGuard guard(lock_);
  const int64_t reserved = ReserveIrqBitsLocked(device_index, count);
  if (reserved < 0) {
    return false;
  }
  const int first_bit = static_cast<int>(reserved);
  int vectors[kSyscallIrqMaxCount] = {};
  if (DeviceIrqAllocateVectors(OwnerToken(),   //
                               device_index,   //
                               count,          //
                               first_bit,      //
                               &irq_pending_,  //
                               owner_task,     //
                               vectors) != DeviceIrqStatus::kOk) {
    UnreserveIrqBitsLocked(device_index, first_bit, count);
    return false;
  }
  return true;
}

DriverReleaseRecord DriverContext::Release(const bool address_space_alive) {
  int claimed_snapshot[kSyscallMaxClaims] = {};
  int claimed_count_snapshot = 0;
  bool isa_claimed_snapshot[kIsaDeviceCount] = {};
  int isa_claimed_count_snapshot = 0;
  {
    const IrqSpinLockGuard guard(lock_);
    if (released_) {
      return DriverReleaseRecord{};
    }
    released_ = true;
    claimed_count_snapshot = claimed_count_;
    for (int i = 0; i < claimed_count_snapshot; ++i) {
      claimed_snapshot[i] = claimed_[i];
      irq_allocated_[i] = false;
    }
    claimed_count_ = 0;
    isa_claimed_count_snapshot = isa_claimed_count_;
    for (int i = 0; i < kIsaDeviceCount; ++i) {
      isa_claimed_snapshot[i] = isa_claimed_[i];
      isa_claimed_[i] = false;
      isa_irq_allocated_[i] = false;
    }
    isa_claimed_count_ = 0;
  }

  DriverReleaseRecord record{};
  // Step 1: Quiesce every claimed device in PCI config space (stops DMA and
  // MSI/MSI-X generation and flushes posted writes with a non-posted config
  // read-back), mask MSI-X table entries in MMIO while Memory Space is still
  // enabled and unmap the kernel MSI-X mapping, and mask any claimed ISA
  // I/O APIC redirection entries (with a non-posted IOWIN read-back flush).
  for (int i = 0; i < claimed_count_snapshot; ++i) {
    const int dev_idx = claimed_snapshot[i];
    PciQuiesce(dev_idx);
    DeviceIrqReleaseDevice(dev_idx);
    ++record.quiesced_count;
  }
  for (int i = 0; i < kIsaDeviceCount; ++i) {
    if (isa_claimed_snapshot[i]) {
      DeviceIrqReleaseIsaDevice(i);
      ++record.quiesced_count;
    }
  }

  // Step 2: Unregister IDT handlers and free vectors in [0x50, 0x7F].
  const int total_claims = claimed_count_snapshot + isa_claimed_count_snapshot;
  const bool all_quiesced_before_free = (record.quiesced_count == total_claims);
  record.freed_vectors = DeviceIrqFreeVectors(OwnerToken());
  record.quiesced_before_vectors_freed =
      all_quiesced_before_free && (DeviceIrqFreeVectors(OwnerToken()) == 0) &&
      (DeviceIrqVectorForBit(OwnerToken(), 0) < 0);
  record.quiesced_before_address_space_destroyed = address_space_alive;

  // Step 3: Release PCI and ISA ownership so another driver can claim them.
  for (int i = 0; i < claimed_count_snapshot; ++i) {
    if (PciRelease(claimed_snapshot[i], OwnerToken()) == PciStatus::kOk) {
      ++record.released_claims;
    }
  }
  for (int i = 0; i < kIsaDeviceCount; ++i) {
    if (isa_claimed_snapshot[i] &&
        IsaRelease(i, OwnerToken()) == IsaStatus::kOk) {
      ++record.released_claims;
    }
  }

  if (total_claims > 0) {
    const IrqSpinLockGuard guard(g_release_record_lock);
    g_last_release_record = record;
  }
  return record;
}

#if __STDC_HOSTED__
void DriverResetForTest() {
  {
    const IrqSpinLockGuard guard(g_release_record_lock);
    g_last_release_record = DriverReleaseRecord{};
  }
  DeviceIrqResetForTest();
  IoApicResetForTest();
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

  void SleepMicroseconds(const int64_t microseconds) override {
    TaskScheduler* const scheduler = GetTaskScheduler();
    CHECK(scheduler != nullptr);
    const CpuLocal* const cpu0 = SmpGetCpuLocal(0);
    CHECK(cpu0 != nullptr);
    // Whole ticks of CPU 0's timer, which BlockUntil() counts, plus one: the
    // current tick may be nearly over.
    constexpr int64_t kTickMicroseconds = 1000000 / kApicTimerTargetHz;
    const int64_t deadline =
        cpu0->timer_ticks.load(std::memory_order_acquire) +
        (microseconds + kTickMicroseconds - 1) / kTickMicroseconds + 1;
    while (cpu0->timer_ticks.load(std::memory_order_acquire) < deadline) {
      scheduler->BlockUntil(deadline);
    }
  }

  void Log(const char* const text) override { ConsoleWrite(text); }

  uint8_t PortRead8(const uint16_t port) override { return PortIn8(port); }

  void PortWrite8(const uint16_t port, const uint8_t value) override {
    PortOut8(port, value);
  }
};

}  // namespace

std::shared_ptr<DriverEnvironment> CreateKernelDriverEnvironment() {
  return std::make_shared<KernelDriverEnvironment>();
}
#endif

}  // namespace protos
