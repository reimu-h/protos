#include "pci_config_fake.h"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <ios>
#include <mutex>
#include <ostream>
#include <thread>
#include <vector>

#include "pci.h"

namespace protos {
namespace {

// Command bits that QEMU's devices implement as writable: I/O Space, Memory
// Space, Bus Master, Parity Error Response, SERR# Enable, Interrupt Disable.
constexpr uint16_t kWritableCommandBits = 0x0547;
// Status bits that are write-1-to-clear: Master Data Parity Error, Signaled
// and Received Target Abort, Received Master Abort, Signaled System Error,
// Detected Parity Error.
constexpr uint16_t kStatusErrorBits = 0xF900;
constexpr int kInterruptLineOffset = 0x3C;
constexpr int kPrimaryBusOffset = 0x18;
constexpr uint16_t kMsiWritableControlBits = 0x0071;
constexpr uint16_t kMsixWritableControlBits =
    kPciMsixControlFunctionMask | kPciMsixControlEnable;
constexpr uint16_t kPmcsrPmeEnable = 1u << 8;
constexpr uint16_t kPmcsrStateD3hot = 3;
// PCI Express capability sizes (versions 1 and 2), Slot Implemented, the
// writable bits of Device Control (not Initiate Function Level Reset) and Link
// Control (not Retrain Link), and Device Control's reset value: Relaxed
// Ordering and No Snoop enabled, a 512-byte Max_Read_Request_Size.
constexpr int kPcieBytesV1 = 0x24;
constexpr int kPcieBytesV2 = 0x3C;
constexpr uint32_t kPcieSlotImplemented = 1u << 8;
constexpr uint32_t kPcieDeviceControlWritable = 0x7FFF;
constexpr uint32_t kPcieDeviceControlReset = 0x2810;
constexpr uint32_t kPcieLinkControlWritable = 0x0FDB;

static bool IsValidAccess(const int offset, const int width) {
  return offset >= 0 && offset + width <= kPciConfigSpaceSize &&
         offset % width == 0;
}

static uint32_t AllOnes(const int width) {
  return (width == 4) ? 0xFFFFFFFFu : ((1u << (8 * width)) - 1);
}

// The number of BAR registers of a header layout.
static int BarCount(const uint8_t header_type) {
  switch (header_type & kPciHeaderTypeLayoutMask) {
    case kPciHeaderTypeEndpoint:
      return 6;
    case kPciHeaderTypeBridge:
      return 2;
    default:
      return 0;
  }
}

// Counts an access as in progress for its lifetime, and records the largest
// count seen. The yield gives a concurrent access the chance to overlap, so an
// unserialized caller shows up in the maximum.
class AccessInProgress {
 public:
  AccessInProgress(std::atomic<int>* const in_progress,
                   std::atomic<int>* const max_in_progress)
      : in_progress_(in_progress) {
    const int now = in_progress_->fetch_add(1) + 1;
    int seen = max_in_progress->load();
    while (now > seen && !max_in_progress->compare_exchange_weak(seen, now)) {
    }
    std::this_thread::yield();
  }

  ~AccessInProgress() { in_progress_->fetch_sub(1); }

  AccessInProgress(const AccessInProgress&) = delete;
  AccessInProgress& operator=(const AccessInProgress&) = delete;

 private:
  std::atomic<int>* const in_progress_;
};

}  // namespace

uint32_t FakePciConfigSpace::Key(const PciAddress address) {
  return (static_cast<uint32_t>(address.bus) << 16) |
         (static_cast<uint32_t>(address.device) << 8) | address.function;
}

FakePciConfigSpace::Function& FakePciConfigSpace::FunctionLocked(
    const PciAddress address) {
  const auto it = functions_.find(Key(address));
  if (it == functions_.end()) {
    ADD_FAILURE() << "no function " << int{address.bus} << ":"
                  << int{address.device} << "." << int{address.function};
    return functions_[Key(address)];
  }
  return it->second;
}

const FakePciConfigSpace::Function* FakePciConfigSpace::FindLocked(
    const PciAddress address) const {
  const auto it = functions_.find(Key(address));
  return (it == functions_.end()) ? nullptr : &it->second;
}

uint32_t FakePciConfigSpace::Load(const Function& function,  //
                                  const int offset,          //
                                  const int width) {
  uint32_t value = 0;
  for (int i = width - 1; i >= 0; --i) {
    value = (value << 8) | function.bytes[offset + i];
  }
  return value;
}

void FakePciConfigSpace::Store(std::array<uint8_t, kPciConfigSpaceSize>* bytes,
                               const int offset,  //
                               const int width,   //
                               const uint32_t value) {
  for (int i = 0; i < width; ++i) {
    (*bytes)[offset + i] = static_cast<uint8_t>(value >> (8 * i));
  }
}

uint32_t FakePciConfigSpace::Read(const PciAddress address,  //
                                  const int offset,          //
                                  const int width) {
  if (!IsValidAccess(offset, width)) {
    ADD_FAILURE() << "invalid " << width << "-byte read at offset " << offset;
    return AllOnes(width);
  }
  const AccessInProgress in_progress(&accesses_in_progress_,
                                     &max_concurrent_accesses_);
  const std::lock_guard<std::mutex> lock(mutex_);
  const auto it = functions_.find(Key(address));
  uint32_t value = AllOnes(width);
  if (it != functions_.end() && !BehindSleepingBridgeLocked(address.bus)) {
    Function& function = it->second;
    if (function.reads_until_ready_left > 0) {
      --function.reads_until_ready_left;
    } else {
      value = Load(function, offset, width);
    }
  }
  accesses_.push_back(Access{/*write=*/false, width, address, offset, value});
  return value;
}

void FakePciConfigSpace::Write(const PciAddress address,  //
                               const int offset,          //
                               const int width,           //
                               const uint32_t value) {
  if (!IsValidAccess(offset, width)) {
    ADD_FAILURE() << "invalid " << width << "-byte write at offset " << offset;
    return;
  }
  const AccessInProgress in_progress(&accesses_in_progress_,
                                     &max_concurrent_accesses_);
  const std::lock_guard<std::mutex> lock(mutex_);
  accesses_.push_back(Access{/*write=*/true, width, address, offset, value});
  const auto it = functions_.find(Key(address));
  if (it == functions_.end() || BehindSleepingBridgeLocked(address.bus)) {
    ++writes_to_absent_functions_;
    return;
  }
  Function& function = it->second;
  if (function.reads_until_ready_left > 0) {
    return;
  }
  const int bars_end =
      kPciBar0Offset + 4 * BarCount(function.bytes[kPciHeaderTypeOffset]);
  const uint32_t command = Load(function, kPciCommandOffset, 2);
  if (offset < bars_end && offset + width > kPciBar0Offset &&
      (command & (kPciCommandIoSpace | kPciCommandMemorySpace)) != 0) {
    ++bar_writes_while_decoding_;
  }
  const int pmcsr = (function.pm_offset != 0)
                        ? function.pm_offset + kPciPmControlStatusOffset
                        : -1;
  const bool writes_pmcsr =
      pmcsr >= 0 && offset < pmcsr + 2 && pmcsr < offset + width;
  const uint32_t old_state =
      writes_pmcsr ? (Load(function, pmcsr, 2) & kPciPmcsrPowerStateMask) : 0;
  const bool bars_frozen = function.ignores_bar_writes_in_d3hot &&
                           PowerState(function) == kPmcsrStateD3hot;
  for (int i = 0; i < width; ++i) {
    const int b = offset + i;
    if (bars_frozen && b >= kPciBar0Offset && b < bars_end) {
      continue;
    }
    const uint8_t written = static_cast<uint8_t>(value >> (8 * i));
    const uint8_t writable = function.writable[b];
    uint8_t byte = static_cast<uint8_t>((function.bytes[b] & ~writable) |
                                        (written & writable));
    byte = static_cast<uint8_t>(byte &
                                ~(written & function.write_one_to_clear[b]));
    function.bytes[b] = byte;
  }
  if (writes_pmcsr) {
    const uint32_t pmcsr_value = Load(function, pmcsr, 2);
    if (old_state == kPmcsrStateD3hot &&
        (pmcsr_value & kPciPmcsrPowerStateMask) == 0) {
      // From D3hot to D0 without No_Soft_Reset: the function resets itself.
      if ((pmcsr_value & kPciPmcsrNoSoftReset) == 0) {
        for (int b = 0; b < kPciConfigSpaceSize; ++b) {
          if (b != pmcsr && b != pmcsr + 1) {
            function.bytes[b] = static_cast<uint8_t>(
                (function.bytes[b] & ~function.writable[b]) |
                (function.reset_values[b] & function.writable[b]));
          }
        }
      }
      function.reads_until_ready_left = function.reads_until_ready;
    }
  }
}

int FakePciConfigSpace::PowerState(const Function& function) {
  return (function.pm_offset != 0)
             ? static_cast<int>(
                   Load(function,
                        function.pm_offset + kPciPmControlStatusOffset, 2) &
                   kPciPmcsrPowerStateMask)
             : 0;
}

bool FakePciConfigSpace::BehindSleepingBridgeLocked(const int bus) const {
  for (const auto& entry : functions_) {
    const Function& bridge = entry.second;
    if ((bridge.bytes[kPciHeaderTypeOffset] & kPciHeaderTypeLayoutMask) !=
        kPciHeaderTypeBridge) {
      continue;
    }
    const int secondary = bridge.bytes[kPciSecondaryBusOffset];
    const int subordinate = bridge.bytes[kPciSubordinateBusOffset];
    if (secondary != 0 && secondary <= bus && bus <= subordinate &&
        PowerState(bridge) != 0) {
      return true;
    }
  }
  return false;
}

void FakePciConfigSpace::AddFunction(const PciAddress address,
                                     const FunctionSpec& spec) {
  const std::lock_guard<std::mutex> lock(mutex_);
  Function& function = functions_[Key(address)];
  function = Function{};
  Store(&function.bytes, kPciVendorIdOffset, 2, spec.vendor_id);
  Store(&function.bytes, kPciDeviceIdOffset, 2, spec.device_id);
  Store(&function.bytes, kPciCommandOffset, 2, spec.command);
  Store(&function.bytes, kPciStatusOffset, 2, spec.status);
  function.bytes[kPciRevisionOffset] = spec.revision;
  function.bytes[kPciRevisionOffset + 1] = spec.prog_if;
  function.bytes[kPciRevisionOffset + 2] = spec.subclass;
  function.bytes[kPciRevisionOffset + 3] = spec.class_code;
  function.bytes[kPciHeaderTypeOffset] = spec.header_type;
  Store(&function.writable, kPciCommandOffset, 2, kWritableCommandBits);
  Store(&function.write_one_to_clear, kPciStatusOffset, 2, kStatusErrorBits);
  function.writable[kPciCacheLineSizeOffset] = 0xFF;
  function.writable[kPciLatencyTimerOffset] = 0xFF;
  function.writable[kInterruptLineOffset] = 0xFF;
}

void FakePciConfigSpace::SetBar(const PciAddress address,  //
                                const int index,           //
                                const BarSpec& bar) {
  const std::lock_guard<std::mutex> lock(mutex_);
  Function& function = FunctionLocked(address);
  const int count = BarCount(function.bytes[kPciHeaderTypeOffset]);
  const int span = (bar.kind == BarKind::kMem64) ? 2 : 1;
  if (index < 0 || index + span > count) {
    ADD_FAILURE() << "BAR " << index << " does not fit in the header";
    return;
  }
  const uint64_t min_size = (bar.kind == BarKind::kIo) ? 4 : 16;
  if (bar.kind != BarKind::kNone &&
      (bar.size < min_size || (bar.size & (bar.size - 1)) != 0 ||
       (bar.base & (bar.size - 1)) != 0 ||
       (bar.kind != BarKind::kMem64 && bar.base > 0xFFFFFFFFu))) {
    ADD_FAILURE() << "invalid BAR " << index << ": base 0x" << std::hex
                  << bar.base << " size 0x" << bar.size;
    return;
  }
  // Address bits below the size are read-only zeros.
  const uint64_t address_mask = ~(bar.size - 1);
  uint32_t low = 0;
  uint32_t low_writable = 0;
  uint32_t high = 0;
  uint32_t high_writable = 0;
  switch (bar.kind) {
    case BarKind::kNone:
      break;
    case BarKind::kIo:
      low = static_cast<uint32_t>(bar.base) | 0x1u;
      low_writable = static_cast<uint32_t>(address_mask) & ~0x3u;
      if (bar.io_16bit_decode) {
        low &= 0xFFFFu;
        low_writable &= 0xFFFFu;
      }
      break;
    case BarKind::kMem32:
      low = static_cast<uint32_t>(bar.base) | (bar.prefetchable ? 0x8u : 0u);
      low_writable = static_cast<uint32_t>(address_mask) & ~0xFu;
      break;
    case BarKind::kMem64:
      low = static_cast<uint32_t>(bar.base) | 0x4u |
            (bar.prefetchable ? 0x8u : 0u);
      low_writable = static_cast<uint32_t>(address_mask) & ~0xFu;
      high = static_cast<uint32_t>(bar.base >> 32);
      high_writable = static_cast<uint32_t>(address_mask >> 32);
      break;
  }
  const int offset = kPciBar0Offset + 4 * index;
  Store(&function.bytes, offset, 4, low);
  Store(&function.writable, offset, 4, low_writable);
  if (span == 2) {
    Store(&function.bytes, offset + 4, 4, high);
    Store(&function.writable, offset + 4, 4, high_writable);
  }
}

void FakePciConfigSpace::SetBridgeBuses(const PciAddress address,
                                        const uint8_t primary,
                                        const uint8_t secondary,
                                        const uint8_t subordinate) {
  const std::lock_guard<std::mutex> lock(mutex_);
  Function& function = FunctionLocked(address);
  function.bytes[kPrimaryBusOffset] = primary;
  function.bytes[kPciSecondaryBusOffset] = secondary;
  function.bytes[kPciSubordinateBusOffset] = subordinate;
  for (int offset = kPrimaryBusOffset; offset <= kPciSubordinateBusOffset;
       ++offset) {
    function.writable[offset] = 0xFF;
  }
}

void FakePciConfigSpace::LinkCapabilityLocked(Function* const function,
                                              const int offset,
                                              const uint8_t id) {
  if (offset < 0x40 || offset > kPciConfigSpaceSize - 4 || offset % 4 != 0) {
    ADD_FAILURE() << "invalid capability offset " << offset;
    return;
  }
  function->bytes[offset] = id;
  function->bytes[offset + 1] = 0;
  if (function->capability_offsets.empty()) {
    function->bytes[kPciCapabilitiesPointerOffset] =
        static_cast<uint8_t>(offset);
    function->bytes[kPciStatusOffset] |= kPciStatusCapabilitiesList;
  } else {
    function->bytes[function->capability_offsets.back() + 1] =
        static_cast<uint8_t>(offset);
  }
  function->capability_offsets.push_back(offset);
}

void FakePciConfigSpace::AddCapability(const PciAddress address,  //
                                       const int offset,          //
                                       const uint8_t id) {
  const std::lock_guard<std::mutex> lock(mutex_);
  LinkCapabilityLocked(&FunctionLocked(address), offset, id);
}

void FakePciConfigSpace::AddMsi(const PciAddress address,  //
                                const int offset,          //
                                const MsiSpec& msi) {
  const std::lock_guard<std::mutex> lock(mutex_);
  Function& function = FunctionLocked(address);
  const int bytes =
      (msi.is_64bit ? 0x0E : 0x0A) + (msi.per_vector_masking ? 0x0A : 0);
  if (offset + bytes > kPciConfigSpaceSize) {
    ADD_FAILURE() << "MSI capability at " << offset << " does not fit";
    return;
  }
  LinkCapabilityLocked(&function, offset, kPciCapabilityMsi);
  const uint32_t control =
      (msi.is_64bit ? kPciMsiControl64Bit : 0u) |
      (msi.per_vector_masking ? kPciMsiControlPerVectorMasking : 0u) |
      ((static_cast<uint32_t>(msi.vectors_log2) & 7u) << 1);
  Store(&function.bytes, offset + kPciMsiControlOffset, 2, control);
  Store(&function.writable,             //
        offset + kPciMsiControlOffset,  //
        2,                              //
        kMsiWritableControlBits);
  // Message Address (dword-aligned), its upper half, Message Data, Mask Bits.
  Store(&function.writable, offset + 4, 4, 0xFFFFFFFCu);
  int next = offset + 8;
  if (msi.is_64bit) {
    Store(&function.writable, next, 4, 0xFFFFFFFFu);
    next += 4;
  }
  Store(&function.writable, next, 2, 0xFFFFu);
  if (msi.per_vector_masking) {
    Store(&function.writable, next + 4, 4, 0xFFFFFFFFu);
  }
}

void FakePciConfigSpace::AddMsix(const PciAddress address,  //
                                 const int offset,          //
                                 const MsixSpec& msix) {
  const std::lock_guard<std::mutex> lock(mutex_);
  Function& function = FunctionLocked(address);
  if (offset + 12 > kPciConfigSpaceSize || msix.table_size < 1 ||
      msix.table_size > 2048 || (msix.table_offset & 7u) != 0 ||
      (msix.pba_offset & 7u) != 0) {
    ADD_FAILURE() << "invalid MSI-X capability at " << offset;
    return;
  }
  LinkCapabilityLocked(&function, offset, kPciCapabilityMsix);
  Store(&function.bytes,                //
        offset + kPciMsiControlOffset,  //
        2,                              //
        static_cast<uint32_t>(msix.table_size - 1));
  Store(&function.writable,             //
        offset + kPciMsiControlOffset,  //
        2,                              //
        kMsixWritableControlBits);
  Store(&function.bytes,  //
        offset + 4,       //
        4,                //
        msix.table_offset | (static_cast<uint32_t>(msix.table_bar) & 7u));
  Store(&function.bytes,  //
        offset + 8,       //
        4,                //
        msix.pba_offset | (static_cast<uint32_t>(msix.pba_bar) & 7u));
}

void FakePciConfigSpace::AddPowerManagement(const PciAddress address,  //
                                            const int offset,          //
                                            const PmSpec& pm) {
  const std::lock_guard<std::mutex> lock(mutex_);
  Function& function = FunctionLocked(address);
  if (offset + kPciPmCapabilityBytes > kPciConfigSpaceSize ||
      pm.power_state > kPmcsrStateD3hot) {
    ADD_FAILURE() << "invalid Power Management capability at " << offset;
    return;
  }
  LinkCapabilityLocked(&function, offset, kPciCapabilityPowerManagement);
  const int pmcsr = offset + kPciPmControlStatusOffset;
  Store(&function.bytes,  //
        pmcsr,            //
        2,                //
        pm.power_state | (pm.no_soft_reset ? kPciPmcsrNoSoftReset : 0u));
  Store(&function.writable,  //
        pmcsr,               //
        2,                   //
        kPciPmcsrPowerStateMask | kPmcsrPmeEnable);
  Store(&function.write_one_to_clear, pmcsr, 2, kPciPmcsrPmeStatus);
  function.pm_offset = offset;
  function.reads_until_ready = pm.reads_until_ready;
  function.ignores_bar_writes_in_d3hot = pm.ignores_bar_writes_in_d3hot;
}

void FakePciConfigSpace::AddPcie(const PciAddress address,  //
                                 const int offset,          //
                                 const PcieSpec& pcie) {
  const std::lock_guard<std::mutex> lock(mutex_);
  Function& function = FunctionLocked(address);
  const int bytes = (pcie.version >= 2) ? kPcieBytesV2 : kPcieBytesV1;
  if (offset + bytes > kPciConfigSpaceSize) {
    ADD_FAILURE() << "PCI Express capability at " << offset << " does not fit";
    return;
  }
  LinkCapabilityLocked(&function, offset, kPciCapabilityExpress);
  Store(&function.bytes,                         //
        offset + kPciExpressCapabilitiesOffset,  //
        2,                                       //
        (static_cast<uint32_t>(pcie.version) & 0xFu) |
            ((static_cast<uint32_t>(pcie.port_type) & 0xFu) << 4) |
            (pcie.slot_implemented ? kPcieSlotImplemented : 0u));
  // A register that starts at its reset value, with `mask` writable.
  const auto add_register = [&](const int reg,        //
                                const uint32_t mask,  //
                                const uint32_t reset) {
    Store(&function.bytes, offset + reg, 2, reset);
    Store(&function.writable, offset + reg, 2, mask);
    Store(&function.reset_values, offset + reg, 2, reset);
  };
  const int type = pcie.port_type;
  add_register(kPciExpressDeviceControlOffset,  //
               kPcieDeviceControlWritable,      //
               kPcieDeviceControlReset);
  if (type == 0x0 || type == 0x1 || (type >= 0x4 && type <= 0x8)) {
    add_register(kPciExpressLinkControlOffset, kPcieLinkControlWritable, 0);
  }
  if ((type == 0x4 || type == 0x6 || type == 0x7) && pcie.slot_implemented) {
    add_register(kPciExpressSlotControlOffset, 0x1FFFu, 0);
  }
  if (type == 0x4 || type == 0xA) {
    add_register(kPciExpressRootControlOffset, 0x001Fu, 0);
  }
  if (pcie.version >= 2) {
    add_register(kPciExpressDeviceControl2Offset, 0xFFFFu, 0);
  }
}

void FakePciConfigSpace::SetWritable(const PciAddress address,  //
                                     const int offset,          //
                                     const int width,           //
                                     const uint32_t mask) {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!IsValidAccess(offset, width)) {
    ADD_FAILURE() << "invalid " << width << "-byte register at " << offset;
    return;
  }
  Store(&FunctionLocked(address).writable, offset, width, mask);
}

void FakePciConfigSpace::SetRaw8(const PciAddress address,  //
                                 const int offset,          //
                                 const uint8_t value) {
  const std::lock_guard<std::mutex> lock(mutex_);
  Store(&FunctionLocked(address).bytes, offset, 1, value);
}

void FakePciConfigSpace::SetRaw16(const PciAddress address,  //
                                  const int offset,          //
                                  const uint16_t value) {
  const std::lock_guard<std::mutex> lock(mutex_);
  Store(&FunctionLocked(address).bytes, offset, 2, value);
}

void FakePciConfigSpace::SetRaw32(const PciAddress address,  //
                                  const int offset,          //
                                  const uint32_t value) {
  const std::lock_guard<std::mutex> lock(mutex_);
  Store(&FunctionLocked(address).bytes, offset, 4, value);
}

uint8_t FakePciConfigSpace::Peek8(const PciAddress address,
                                  const int offset) const {
  const std::lock_guard<std::mutex> lock(mutex_);
  const Function* const function = FindLocked(address);
  return static_cast<uint8_t>(function != nullptr ? Load(*function, offset, 1)
                                                  : AllOnes(1));
}

uint16_t FakePciConfigSpace::Peek16(const PciAddress address,
                                    const int offset) const {
  const std::lock_guard<std::mutex> lock(mutex_);
  const Function* const function = FindLocked(address);
  return static_cast<uint16_t>(function != nullptr ? Load(*function, offset, 2)
                                                   : AllOnes(2));
}

uint32_t FakePciConfigSpace::Peek32(const PciAddress address,
                                    const int offset) const {
  const std::lock_guard<std::mutex> lock(mutex_);
  const Function* const function = FindLocked(address);
  return (function != nullptr) ? Load(*function, offset, 4) : AllOnes(4);
}

std::vector<FakePciConfigSpace::Access> FakePciConfigSpace::Accesses() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return accesses_;
}

void FakePciConfigSpace::ClearAccesses() {
  const std::lock_guard<std::mutex> lock(mutex_);
  accesses_.clear();
}

int FakePciConfigSpace::BarWritesWhileDecoding() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return bar_writes_while_decoding_;
}

int FakePciConfigSpace::WritesToAbsentFunctions() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return writes_to_absent_functions_;
}

int FakePciConfigSpace::MaxConcurrentAccesses() const {
  return max_concurrent_accesses_.load();
}

uint8_t FakePciConfigSpace::Read8(const PciAddress address, const int offset) {
  return static_cast<uint8_t>(Read(address, offset, 1));
}

uint16_t FakePciConfigSpace::Read16(const PciAddress address,
                                    const int offset) {
  return static_cast<uint16_t>(Read(address, offset, 2));
}

uint32_t FakePciConfigSpace::Read32(const PciAddress address,
                                    const int offset) {
  return Read(address, offset, 4);
}

void FakePciConfigSpace::Write8(const PciAddress address,  //
                                const int offset,          //
                                const uint8_t value) {
  Write(address, offset, 1, value);
}

void FakePciConfigSpace::Write16(const PciAddress address,  //
                                 const int offset,          //
                                 const uint16_t value) {
  Write(address, offset, 2, value);
}

void FakePciConfigSpace::Write32(const PciAddress address,  //
                                 const int offset,          //
                                 const uint32_t value) {
  Write(address, offset, 4, value);
}

// Values as read through 0xCF8/0xCFC from QEMU 11.1 at kernel entry (the
// sizes from QEMU's query-pci).
void AddQemuPcMachine(FakePciConfigSpace* const fake,
                      const QemuFirmware firmware,
                      const QemuTopology topology) {
  using BarKind = FakePciConfigSpace::BarKind;
  const bool ovmf = firmware == QemuFirmware::kOvmf;
  const bool single = topology == QemuTopology::kSingle;
  const bool multi = topology == QemuTopology::kMulti;
  // SeaBIOS enables I/O and memory decoding and SERR#, and Bus Master only on
  // the devices it drives (the xHCIs); OVMF enables I/O, memory and Bus Master
  // everywhere.
  const uint16_t command = ovmf ? 0x0007 : 0x0103;
  const uint16_t bus_master_command = ovmf ? 0x0007 : 0x0107;

  const PciAddress host_bridge = {0, 0, 0};
  fake->AddFunction(host_bridge, {.vendor_id = 0x8086,
                                  .device_id = 0x1237,
                                  .class_code = 0x06,
                                  .subclass = 0x00,
                                  .prog_if = 0x00,
                                  .revision = 0x02,
                                  .header_type = 0x00,
                                  .command = command,
                                  .status = 0x0000});

  const PciAddress isa_bridge = {0, 1, 0};
  fake->AddFunction(isa_bridge, {.vendor_id = 0x8086,
                                 .device_id = 0x7000,
                                 .class_code = 0x06,
                                 .subclass = 0x01,
                                 .prog_if = 0x00,
                                 .revision = 0x00,
                                 .header_type = 0x80,
                                 .command = command,
                                 .status = 0x0200});

  const PciAddress ide = {0, 1, 1};
  fake->AddFunction(ide, {.vendor_id = 0x8086,
                          .device_id = 0x7010,
                          .class_code = 0x01,
                          .subclass = 0x01,
                          .prog_if = 0x80,
                          .revision = 0x00,
                          .header_type = 0x00,
                          .command = command,
                          .status = 0x0280});
  fake->SetBar(ide,  //
               4,    //
               {.kind = BarKind::kIo,
                .base = (!ovmf && multi) ? 0xD040u : 0xC040u,
                .size = 0x10});

  const PciAddress acpi = {0, 1, 3};
  fake->AddFunction(acpi, {.vendor_id = 0x8086,
                           .device_id = 0x7113,
                           .class_code = 0x06,
                           .subclass = 0x80,
                           .prog_if = 0x00,
                           .revision = 0x03,
                           .header_type = 0x00,
                           .command = command,
                           .status = 0x0280});

  const PciAddress vga = {0, 2, 0};
  fake->AddFunction(vga, {.vendor_id = 0x1234,
                          .device_id = 0x1111,
                          .class_code = 0x03,
                          .subclass = 0x00,
                          .prog_if = 0x00,
                          .revision = 0x02,
                          .header_type = 0x00,
                          .command = command,
                          .status = 0x0000});
  fake->SetBar(vga,  //
               0,    //
               {.kind = BarKind::kMem32,
                .base = ovmf ? 0xC0000000u : 0xFD000000u,
                .size = 0x1000000,
                .prefetchable = true});
  uint64_t vga_bar2 = 0;
  if (ovmf) {
    vga_bar2 = multi ? 0xC1260000u : 0xC1060000u;
  } else {
    vga_bar2 = multi ? 0xFEA74000u : (single ? 0xFEBF4000u : 0xFEBF0000u);
  }
  fake->SetBar(vga,  //
               2,    //
               {.kind = BarKind::kMem32, .base = vga_bar2, .size = 0x1000});

  const PciAddress nic = {0, 3, 0};
  fake->AddFunction(nic, {.vendor_id = 0x8086,
                          .device_id = 0x100E,
                          .class_code = 0x02,
                          .subclass = 0x00,
                          .prog_if = 0x00,
                          .revision = 0x03,
                          .header_type = 0x00,
                          .command = command,
                          .status = 0x0000});
  uint64_t nic_bar0 = 0;
  if (ovmf) {
    nic_bar0 = multi ? 0xC1240000u : 0xC1040000u;
  } else {
    nic_bar0 = multi ? 0xFEA40000u : 0xFEBC0000u;
  }
  fake->SetBar(nic,  //
               0,    //
               {.kind = BarKind::kMem32, .base = nic_bar0, .size = 0x20000});
  fake->SetBar(nic,  //
               1,    //
               {.kind = BarKind::kIo,
                .base = (!ovmf && multi) ? 0xD000u : 0xC000u,
                .size = 0x40});

  if (topology == QemuTopology::kNone) {
    return;
  }
  // `-device qemu-xhci`: MSI-X with 16 vectors, table and PBA in BAR0.
  const PciAddress root_xhci = {0, 4, 0};
  fake->AddFunction(root_xhci, {.vendor_id = 0x1B36,
                                .device_id = 0x000D,
                                .class_code = 0x0C,
                                .subclass = 0x03,
                                .prog_if = 0x30,
                                .revision = 0x01,
                                .header_type = 0x00,
                                .command = bus_master_command,
                                .status = 0x0010});
  uint64_t root_xhci_bar0 = 0;
  if (ovmf) {
    root_xhci_bar0 = multi ? 0xC020000000u : 0xC000000000u;
  } else {
    root_xhci_bar0 = multi ? 0xFEA70000u : 0xFEBF0000u;
  }
  fake->SetBar(
      root_xhci,  //
      0,          //
      {.kind = BarKind::kMem64, .base = root_xhci_bar0, .size = 0x4000});
  fake->AddMsix(root_xhci,  //
                0x90,       //
                {.table_size = 16,
                 .table_bar = 0,
                 .table_offset = 0x3000,
                 .pba_bar = 0,
                 .pba_offset = 0x3800});

  if (topology != QemuTopology::kMulti) {
    return;
  }
  // `-device pci-bridge,chassis_nr=1`: bus 1, a 256-byte 64-bit BAR (SHPC),
  // and the capabilities MSI (64-bit, per-vector masking) -> slot ID -> SHPC.
  const PciAddress bridge = {0, 5, 0};
  fake->AddFunction(bridge, {.vendor_id = 0x1B36,
                             .device_id = 0x0001,
                             .class_code = 0x06,
                             .subclass = 0x04,
                             .prog_if = 0x00,
                             .revision = 0x00,
                             .header_type = 0x01,
                             .command = command,
                             .status = 0x00B0});
  fake->SetBar(bridge,  //
               0,       //
               {.kind = BarKind::kMem64,
                .base = ovmf ? 0xC020004000u : 0xFEA75000u,
                .size = 0x100});
  fake->SetBridgeBuses(bridge, 0, 1, 1);
  fake->AddMsi(
      bridge,  //
      0x4C,    //
      {.is_64bit = true, .per_vector_masking = true, .vectors_log2 = 0});
  fake->AddCapability(bridge, 0x48, 0x04);
  fake->SetRaw16(bridge, 0x4A, 0x0120);
  fake->AddCapability(bridge, 0x40, 0x0C);

  // `-device nec-usb-xhci,bus=br1,addr=1,msi=on,msix=off`: MSI only (64-bit,
  // 16 vectors).
  const PciAddress bridged_xhci = {1, 1, 0};
  fake->AddFunction(bridged_xhci, {.vendor_id = 0x1033,
                                   .device_id = 0x0194,
                                   .class_code = 0x0C,
                                   .subclass = 0x03,
                                   .prog_if = 0x30,
                                   .revision = 0x03,
                                   .header_type = 0x00,
                                   .command = bus_master_command,
                                   .status = 0x0010});
  fake->SetBar(bridged_xhci,  //
               0,             //
               {.kind = BarKind::kMem64,
                .base = ovmf ? 0xC1000000u : 0xFE800000u,
                .size = 0x4000});
  fake->AddMsi(
      bridged_xhci,  //
      0x70,          //
      {.is_64bit = true, .per_vector_masking = false, .vectors_log2 = 4});
}

void PrintTo(const PciAddress& address, std::ostream* const os) {
  const std::ios_base::fmtflags flags = os->flags();
  *os << std::hex << int{address.bus} << ":" << int{address.device} << "."
      << int{address.function};
  os->flags(flags);
}

void PrintTo(const PciBar& bar, std::ostream* const os) {
  static constexpr const char* kTypeNames[] = {"none", "io", "mem32", "mem64"};
  const std::ios_base::fmtflags flags = os->flags();
  *os << "{" << kTypeNames[static_cast<int>(bar.type)]
      << (bar.prefetchable ? " prefetchable" : "") << " base 0x" << std::hex
      << bar.base << " size 0x" << bar.size << "}";
  os->flags(flags);
}

void PrintTo(const PciStatus status, std::ostream* const os) {
  static constexpr const char* kNames[] = {"kOk", "kNoDevice", "kNotClaimable",
                                           "kBusy", "kNotOwner"};
  *os << kNames[static_cast<int>(status)];
}

void PrintTo(const FakePciConfigSpace::Access& access, std::ostream* const os) {
  const std::ios_base::fmtflags flags = os->flags();
  *os << (access.write ? "write" : "read") << access.width << " ";
  PrintTo(access.address, os);
  *os << std::hex << " +0x" << access.offset << " = 0x" << access.value;
  os->flags(flags);
}

}  // namespace protos
