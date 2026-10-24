// PCI enumeration, the device table and device ownership (pci.h), on top of a
// PciConfigSpace: configuration mechanism #1 in the kernel (at the end of this
// file), FakePciConfigSpace (pci_config_fake.h) in host tests.

#include "pci.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <utility>

#include "check.h"
#include "spinlock.h"
#if !__STDC_HOSTED__
#include "port_io.h"
#endif

namespace protos {
namespace {

constexpr int kPciBusCount = 256;
constexpr int kPciDevicesPerBus = 32;
constexpr int kPciFunctionsPerDevice = 8;

// Vendor IDs that no present function has: what an absent function reads
// (all ones, or zeros on some chipsets), and the Configuration Request Retry
// Status value of a function that is not ready.
constexpr uint16_t kPciVendorIdAbsent = 0xFFFF;
constexpr uint16_t kPciVendorIdZero = 0x0000;
constexpr uint16_t kPciVendorIdRetry = 0x0001;

// BAR flag bits [PCI Local Bus 3.0 §6.2.5.1].
constexpr uint32_t kPciBarIoSpace = 1u << 0;
constexpr uint32_t kPciBarMemoryTypeMask = 3u << 1;
constexpr uint32_t kPciBarMemoryType64 = 2u << 1;
constexpr uint32_t kPciBarMemoryTypeReserved = 3u << 1;
constexpr uint32_t kPciBarPrefetchable = 1u << 3;
constexpr uint32_t kPciBarIoAddressMask = ~0x3u;
constexpr uint32_t kPciBarMemoryAddressMask = ~0xFu;
constexpr int kPciEndpointBarCount = 6;
constexpr int kPciBridgeBarCount = 2;
constexpr uint16_t kPciCommandDecode =
    kPciCommandIoSpace | kPciCommandMemorySpace;

// Capabilities lie above the standard header, each in its own dwords, so a
// list with more entries than that loops.
constexpr int kPciCapabilitiesStart = 0x40;
constexpr int kPciMaxCapabilities =
    (kPciConfigSpaceSize - kPciCapabilitiesStart) / 4;
constexpr int kPciCapabilityPointerMask = 0xFC;

// MSI capability layout: Message Data follows the 32- or 64-bit Message
// Address; with per-vector masking, Mask Bits and Pending Bits follow it.
constexpr int kPciMsiDataOffset32 = 0x08;
constexpr int kPciMsiDataOffset64 = 0x0C;
constexpr int kPciMsiMaskOffset32 = 0x0C;
constexpr int kPciMsiMaskOffset64 = 0x10;
constexpr int kPciMsiBytes32 = 0x0A;
constexpr int kPciMsiBytes64 = 0x0E;
constexpr int kPciMsiMaskedBytes32 = 0x14;
constexpr int kPciMsiMaskedBytes64 = 0x18;
constexpr int kPciMsiVectorsShift = 1;
constexpr int kPciMsiVectorsMask = 0x7;
constexpr int kPciMsiMaxVectorsLog2 = 5;

// MSI-X capability layout.
constexpr int kPciMsixTableRegisterOffset = 4;
constexpr int kPciMsixPbaRegisterOffset = 8;
constexpr int kPciMsixBytes = 12;
constexpr uint32_t kPciMsixBirMask = 0x7;
constexpr int64_t kPciMsixPbaEntryBytes = 8;

// The PCI Express Capabilities register (pci.h), and the Device/Port Types
// that decide which other registers exist [PCIe Base 6.0 §7.5.3.2].
constexpr uint16_t kPciExpressVersionMask = 0x000F;
constexpr int kPciExpressTypeShift = 4;
constexpr uint16_t kPciExpressTypeMask = 0x000F;
constexpr uint16_t kPciExpressSlotImplemented = 1u << 8;
constexpr int kPcieTypeEndpoint = 0x0;
constexpr int kPcieTypeLegacyEndpoint = 0x1;
constexpr int kPcieTypeRootPort = 0x4;
constexpr int kPcieTypeUpstreamPort = 0x5;
constexpr int kPcieTypeDownstreamPort = 0x6;
constexpr int kPcieTypePcieToPciBridge = 0x7;
constexpr int kPcieTypePciToPcieBridge = 0x8;
constexpr int kPcieTypeRootComplexEventCollector = 0xA;

// The registers of a PCI-to-PCI bridge that tell it which bus numbers and
// address ranges lie behind it [PCI-to-PCI Bridge 1.2 §3.2], in
// configuration-space order: the primary, secondary and subordinate bus
// numbers with the secondary latency timer; I/O base and limit (16 bits: the
// Secondary Status register above them has write-1-to-clear bits); memory
// base and limit; prefetchable base and limit; the upper halves of the
// prefetchable base and limit; the upper halves of I/O base and limit. Then
// Bridge Control, whose Secondary Bus Reset bit is never written back.
struct RegisterSpec {
  int offset = 0;
  int width = 0;
};
constexpr RegisterSpec kPciBridgeRoutingRegisters[] = {
    {0x18, 4}, {0x1C, 2}, {0x20, 4}, {0x24, 4}, {0x28, 4}, {0x2C, 4}, {0x30, 4},
};
constexpr int kPciBridgeRoutingStart = 0x18;
constexpr int kPciBridgeRoutingEnd = 0x34;
constexpr int kPciBridgeControlOffset = 0x3E;
constexpr uint16_t kPciBridgeControlSecondaryBusReset = 1u << 6;

// USB host controllers (class 0Ch, subclass 03h) by programming interface, and
// the port routing registers of the xHCI of Intel 7/8/9-series PCHs, as
// Linux's drivers/usb/host/pci-quirks.c names them.
constexpr uint8_t kPciClassSerialBus = 0x0C;
constexpr uint8_t kPciSubclassUsb = 0x03;
constexpr uint8_t kPciProgIfEhci = 0x20;
constexpr uint8_t kPciProgIfXhci = 0x30;
constexpr uint16_t kPciVendorIntel = 0x8086;
constexpr uint16_t kPciVendorSony = 0x104D;
constexpr uint16_t kSonyVaioTSeriesSubsystemId = 0x90A8;
constexpr int kIntelXusb2prOffset = 0xD0;    // USB 2.0 port routing
constexpr int kIntelXusb2prmOffset = 0xD4;   // ... which ports may switch
constexpr int kIntelUsb3PssenOffset = 0xD8;  // SuperSpeed enable
constexpr int kIntelUsb3prmOffset = 0xDC;    // ... which ports may switch

// Room for the longest line of the boot report (about 380 characters).
constexpr int kPciReportLineCapacity = 512;

// Guards every call into g_config (each address/data port pair, and the
// multi-access sequences of PciInit(), the update helpers and the claims), and
// the claim table. A leaf lock: while holding it, pci.cpp takes no other lock,
// and logs, allocates and waits for nothing.
IrqSpinLock g_pci_lock;
// The configuration space that PciInit() installed.
std::shared_ptr<PciConfigSpace> g_config;
// The device table. PciInit() fills it, publishing each record with a release
// store of the count, before any other function runs; it never changes after.
PciDevice g_devices[kPciMaxDevices];
std::atomic<int> g_device_count{0};
// Functions that PciInit() found but could not record: the table was full.
int g_unrecorded_count = 0;
// The owner of each device (kPciNoOwner if unclaimed), and how many devices
// have one. Guarded by g_pci_lock.
uintptr_t g_owners[kPciMaxDevices] = {};
int g_claimed_count = 0;
// Whether PciInit() sized the BARs of each device while the device was in a
// low-power state, until PciRestoreAfterPowerUp() has sized them again in D0.
// Written by PciInit() before it publishes the device, then guarded by
// g_pci_lock.
bool g_sized_in_low_power[kPciMaxDevices] = {};

// A bus waiting to be scanned, and the index of the bridge that leads to it.
struct PendingBus {
  uint8_t bus = 0;
  int16_t parent_index = kPciNoParent;
};

// The state of PciInit()'s walk over the buses. Each bus is visited (scanned,
// or reserved for a CardBus bridge) at most once, which bounds the walk even
// if the bridges' bus numbers loop; `queue` holds the buses to scan, in order.
struct BusWalk {
  bool visited[kPciBusCount] = {};
  PendingBus queue[kPciBusCount];
  int head = 0;
  int tail = 0;
};

static void DcheckAccessLocked(const int offset, const int width) {
  DCHECK(g_pci_lock.IsLockedByCurrentCpu());
  DCHECK(offset >= 0 && offset + width <= kPciConfigSpaceSize &&
         offset % width == 0);
}

static uint8_t Read8Locked(const PciAddress address, const int offset) {
  DcheckAccessLocked(offset, 1);
  return g_config->Read8(address, offset);
}

static uint16_t Read16Locked(const PciAddress address, const int offset) {
  DcheckAccessLocked(offset, 2);
  return g_config->Read16(address, offset);
}

static uint32_t Read32Locked(const PciAddress address, const int offset) {
  DcheckAccessLocked(offset, 4);
  return g_config->Read32(address, offset);
}

static void Write8Locked(const PciAddress address,  //
                         const int offset,          //
                         const uint8_t value) {
  DcheckAccessLocked(offset, 1);
  g_config->Write8(address, offset, value);
}

static void Write16Locked(const PciAddress address,  //
                          const int offset,          //
                          const uint16_t value) {
  DcheckAccessLocked(offset, 2);
  g_config->Write16(address, offset, value);
}

static void Write32Locked(const PciAddress address,  //
                          const int offset,          //
                          const uint32_t value) {
  DcheckAccessLocked(offset, 4);
  g_config->Write32(address, offset, value);
}

// A read or write of a register of `width` bytes (1, 2 or 4).
static uint32_t ReadWidthLocked(const PciAddress address,  //
                                const int offset,          //
                                const int width) {
  switch (width) {
    case 1:
      return Read8Locked(address, offset);
    case 2:
      return Read16Locked(address, offset);
    default:
      DCHECK(width == 4);
      return Read32Locked(address, offset);
  }
}

static void WriteWidthLocked(const PciAddress address,  //
                             const int offset,          //
                             const int width,           //
                             const uint32_t value) {
  switch (width) {
    case 1:
      Write8Locked(address, offset, static_cast<uint8_t>(value));
      break;
    case 2:
      Write16Locked(address, offset, static_cast<uint16_t>(value));
      break;
    default:
      DCHECK(width == 4);
      Write32Locked(address, offset, value);
      break;
  }
}

// Sets `set` and clears `clear` in the 16-bit register at `offset` with one
// read-modify-write (no write if nothing changes or if the register reads
// 0xFFFF, e.g. when a device is gone), then reads it back.
static uint16_t Update16Locked(const PciAddress address,  //
                               const int offset,          //
                               const uint16_t set,        //
                               const uint16_t clear) {
  const uint16_t old_value = Read16Locked(address, offset);
  if (old_value == 0xFFFF) {
    return 0xFFFF;
  }
  const uint16_t new_value = static_cast<uint16_t>((old_value | set) & ~clear);
  if (new_value != old_value) {
    Write16Locked(address, offset, new_value);
  }
  return Read16Locked(address, offset);
}

static bool IsValidIndex(const int index) {
  return index >= 0 && index < g_device_count.load(std::memory_order_acquire);
}

// The address of device `index`, which must be valid.
static PciAddress CheckedAddress(const int index) {
  CHECK(IsValidIndex(index));
  return g_devices[index].address;
}

static void CheckOffset(const int offset, const int width) {
  CHECK(offset >= 0 && offset + width <= kPciConfigSpaceSize &&
        offset % width == 0);
}

// Sets `bits` in the Command register of every bridge between `device` and its
// root bus, from the closest one up. A bridge is always recorded before the
// functions behind it, so the walk ends.
static void SetUpstreamBridgeCommandLocked(const PciDevice& device,
                                           const uint16_t bits) {
  int child = static_cast<int>(&device - g_devices);
  for (int bridge = device.parent_index; bridge != kPciNoParent;
       bridge = g_devices[bridge].parent_index) {
    DCHECK(bridge >= 0 && bridge < child);
    Update16Locked(g_devices[bridge].address, kPciCommandOffset, bits, 0);
    child = bridge;
  }
}

static PciQuiesceResult QuiesceLocked(const PciDevice& device) {
  PciQuiesceResult result;
  // First, in one write: no new DMA or MSI (both need Bus Master), and no
  // legacy interrupt once MSI and MSI-X are off.
  result.command = Update16Locked(device.address,               //
                                  kPciCommandOffset,            //
                                  kPciCommandInterruptDisable,  //
                                  kPciCommandBusMaster);
  if (device.msix.offset != 0) {
    result.msix_control =
        Update16Locked(device.address,                             //
                       device.msix.offset + kPciMsiControlOffset,  //
                       kPciMsixControlFunctionMask,                //
                       kPciMsixControlEnable);
  }
  if (device.msi.offset != 0) {
    result.msi_control =
        Update16Locked(device.address,                            //
                       device.msi.offset + kPciMsiControlOffset,  //
                       0,                                         //
                       kPciMsiControlEnable);
  }
  return result;
}

static int BarOffset(const int index) { return kPciBar0Offset + 4 * index; }

// The lowest set bit of `mask`, the address bits that a BAR's size probe read
// back: the BAR's size, or 0 if it has no address bits (not implemented).
// Unlike ~mask + 1, also right for decoders narrower than the register, such as
// I/O BARs whose upper 16 bits read 0.
static uint64_t LowestSetBit(const uint64_t mask) { return mask & (~mask + 1); }

// Writes all ones to the BAR register at `offset`, reads back what sticks and
// restores `original`.
static uint32_t ProbeBarRegisterLocked(const PciAddress address,
                                       const int offset,
                                       const uint32_t original) {
  Write32Locked(address, offset, 0xFFFFFFFFu);
  const uint32_t probe = Read32Locked(address, offset);
  Write32Locked(address, offset, original);
  return probe;
}

// Decodes the `count` BAR registers of the function at `address` into `bars`:
// type, prefetchable and base. A BAR register of zero (32-bit memory, no
// address) is not implemented or unusable, and a reserved memory type or a
// 64-bit BAR without a register for its upper half is not decodable: these
// stay kNone. With `size_assigned`, also sizes each BAR that has a base: with
// the function's I/O and memory decoding off (so that no probe value decodes as
// an address), writes all ones to the BAR, reads back its size mask and
// restores it, then restores the Command register. A BAR whose probe reads
// back no address bit, or whose size does not fit in int64_t, becomes kNone.
// Called with the PCI lock held, which keeps every other configuration access,
// and interrupt handlers on this CPU, away while decoding is off.
static void DecodeBarsLocked(const PciAddress address,  //
                             const int count,           //
                             const bool size_assigned,  //
                             PciBar* const bars) {
  uint32_t raw[kPciMaxBars] = {};
  for (int i = 0; i < count; ++i) {
    raw[i] = Read32Locked(address, BarOffset(i));
  }
  bool any_assigned = false;
  for (int i = 0; i < count; ++i) {
    const uint32_t low = raw[i];
    const uint32_t memory_type = low & kPciBarMemoryTypeMask;
    PciBar& bar = bars[i];
    if ((low & kPciBarIoSpace) != 0) {
      bar.type = PciBarType::kIo;
      bar.base = low & kPciBarIoAddressMask;
    } else if (memory_type == kPciBarMemoryType64) {
      if (i + 1 == count) {
        continue;
      }
      bar.type = PciBarType::kMem64;
      bar.prefetchable = (low & kPciBarPrefetchable) != 0;
      bar.base = (static_cast<uint64_t>(raw[i + 1]) << 32) |
                 (low & kPciBarMemoryAddressMask);
      ++i;  // The upper half stays kNone.
    } else if (memory_type != kPciBarMemoryTypeReserved && low != 0) {
      bar.type = PciBarType::kMem32;
      bar.prefetchable = (low & kPciBarPrefetchable) != 0;
      bar.base = low & kPciBarMemoryAddressMask;
    }
    any_assigned = any_assigned || bar.base != 0;
  }
  if (!size_assigned || !any_assigned) {
    return;
  }

  const uint16_t command = Read16Locked(address, kPciCommandOffset);
  const bool decoding = (command & kPciCommandDecode) != 0;
  if (decoding) {
    Write16Locked(address,            //
                  kPciCommandOffset,  //
                  static_cast<uint16_t>(command & ~kPciCommandDecode));
  }
  for (int i = 0; i < count; ++i) {
    PciBar& bar = bars[i];
    if (bar.type == PciBarType::kNone || bar.base == 0) {
      continue;
    }
    const int offset = BarOffset(i);
    const uint32_t address_mask = (bar.type == PciBarType::kIo)
                                      ? kPciBarIoAddressMask
                                      : kPciBarMemoryAddressMask;
    uint64_t mask =
        ProbeBarRegisterLocked(address, offset, raw[i]) & address_mask;
    if (bar.type == PciBarType::kMem64) {
      mask |= static_cast<uint64_t>(
                  ProbeBarRegisterLocked(address, offset + 4, raw[i + 1]))
              << 32;
    }
    const uint64_t size = LowestSetBit(mask);
    if (size == 0 || size > static_cast<uint64_t>(INT64_MAX)) {
      bar = PciBar{};
    } else {
      bar.size = static_cast<int64_t>(size);
    }
  }
  if (decoding) {
    Write16Locked(address, kPciCommandOffset, command);
  }
}

// Whether every BAR of `device` that has a recorded base still holds it.
// Called with the PCI lock held.
static bool BarsIntactLocked(const PciDevice& device) {
  for (int i = 0; i < kPciMaxBars; ++i) {
    const PciBar& bar = device.bars[i];
    if (bar.type == PciBarType::kNone || bar.base == 0) {
      continue;
    }
    const uint32_t low = Read32Locked(device.address, BarOffset(i));
    uint64_t base = 0;
    if (bar.type == PciBarType::kIo) {
      base = low & kPciBarIoAddressMask;
    } else {
      base = low & kPciBarMemoryAddressMask;
      if (bar.type == PciBarType::kMem64) {
        base |= static_cast<uint64_t>(
                    Read32Locked(device.address, BarOffset(i + 1)))
                << 32;
      }
    }
    if (base != bar.base) {
      return false;
    }
  }
  return true;
}

// Writes the recorded base of every BAR of `device` that has one back to its
// register(s); the flag bits are read-only. Called with the PCI lock held and
// the device's decoding off.
static void RestoreBarsLocked(const PciDevice& device) {
  for (int i = 0; i < kPciMaxBars; ++i) {
    const PciBar& bar = device.bars[i];
    if (bar.type == PciBarType::kNone || bar.base == 0) {
      continue;
    }
    Write32Locked(device.address,  //
                  BarOffset(i),    //
                  static_cast<uint32_t>(bar.base));
    if (bar.type == PciBarType::kMem64) {
      Write32Locked(device.address,    //
                    BarOffset(i + 1),  //
                    static_cast<uint32_t>(bar.base >> 32));
    }
  }
}

// Decodes the MSI capability at `offset`, unless it does not fit in the
// configuration space.
static void DecodeMsiLocked(const PciAddress address,  //
                            const int offset,          //
                            PciMsi* const msi) {
  const uint16_t control = Read16Locked(address, offset + kPciMsiControlOffset);
  const bool is_64bit = (control & kPciMsiControl64Bit) != 0;
  const bool masking = (control & kPciMsiControlPerVectorMasking) != 0;
  const int bytes = is_64bit
                        ? (masking ? kPciMsiMaskedBytes64 : kPciMsiBytes64)
                        : (masking ? kPciMsiMaskedBytes32 : kPciMsiBytes32);
  if (offset + bytes > kPciConfigSpaceSize) {
    return;
  }
  const int vectors_log2 =
      (control >> kPciMsiVectorsShift) & kPciMsiVectorsMask;
  msi->offset = offset;
  msi->is_64bit = is_64bit;
  msi->per_vector_masking = masking;
  msi->vectors_log2 =
      (vectors_log2 <= kPciMsiMaxVectorsLog2) ? vectors_log2 : 0;
  msi->data_offset =
      offset + (is_64bit ? kPciMsiDataOffset64 : kPciMsiDataOffset32);
  msi->mask_offset =
      masking ? offset + (is_64bit ? kPciMsiMaskOffset64 : kPciMsiMaskOffset32)
              : 0;
}

// Decodes the MSI-X capability at `offset`, unless it does not fit in the
// configuration space. ValidateMsix() checks its table and PBA later.
static void DecodeMsixLocked(const PciAddress address,  //
                             const int offset,          //
                             PciMsix* const msix) {
  if (offset + kPciMsixBytes > kPciConfigSpaceSize) {
    return;
  }
  const uint16_t control = Read16Locked(address, offset + kPciMsiControlOffset);
  const uint32_t table =
      Read32Locked(address, offset + kPciMsixTableRegisterOffset);
  const uint32_t pba =
      Read32Locked(address, offset + kPciMsixPbaRegisterOffset);
  msix->offset = offset;
  msix->table_size = (control & kPciMsixControlTableSizeMask) + 1;
  msix->table_bar = static_cast<int>(table & kPciMsixBirMask);
  msix->table_offset = table & ~kPciMsixBirMask;
  msix->pba_bar = static_cast<int>(pba & kPciMsixBirMask);
  msix->pba_offset = pba & ~kPciMsixBirMask;
}

// Walks the capability list of the function at `address` (header types 0 and
// 1) for its first MSI, MSI-X, Power Management and PCI Express capabilities.
// The list comes from the device, so the walk stops at a pointer into the
// header and after kPciMaxCapabilities entries.
static void ParseCapabilitiesLocked(const PciAddress address,
                                    PciDevice* const device) {
  if ((Read16Locked(address, kPciStatusOffset) & kPciStatusCapabilitiesList) ==
      0) {
    return;
  }
  int pointer = Read8Locked(address, kPciCapabilitiesPointerOffset) &
                kPciCapabilityPointerMask;
  for (int n = 0; n < kPciMaxCapabilities && pointer >= kPciCapabilitiesStart;
       ++n) {
    const uint16_t header = Read16Locked(address, pointer);
    const uint8_t id = static_cast<uint8_t>(header & 0xFF);
    if (id == kPciCapabilityMsi && device->msi.offset == 0) {
      DecodeMsiLocked(address, pointer, &device->msi);
    } else if (id == kPciCapabilityMsix && device->msix.offset == 0) {
      DecodeMsixLocked(address, pointer, &device->msix);
    } else if (id == kPciCapabilityPowerManagement && device->pm_offset == 0 &&
               pointer + kPciPmCapabilityBytes <= kPciConfigSpaceSize) {
      device->pm_offset = pointer;
    } else if (id == kPciCapabilityExpress && device->pcie_offset == 0 &&
               pointer + kPciExpressDeviceControlOffset + 2 <=
                   kPciConfigSpaceSize) {
      device->pcie_offset = pointer;
    }
    pointer = (header >> 8) & kPciCapabilityPointerMask;
  }
}

// BAR `bar_index` of `device` if it is a memory BAR with a base and a size,
// else nullptr.
static const PciBar* UsableMemoryBar(const PciDevice& device,
                                     const int bar_index) {
  if (bar_index < 0 || bar_index >= kPciMaxBars) {
    return nullptr;
  }
  const PciBar& bar = device.bars[bar_index];
  const bool memory =
      bar.type == PciBarType::kMem32 || bar.type == PciBarType::kMem64;
  return (memory && bar.base != 0 && bar.size > 0) ? &bar : nullptr;
}

// Whether the `bytes` bytes at `offset` in `bar` (sized) lie inside it.
static bool OffsetRangeInBar(const PciBar& bar,      //
                             const uint64_t offset,  //
                             const int64_t bytes) {
  const uint64_t size = static_cast<uint64_t>(bar.size);
  return offset <= size && static_cast<uint64_t>(bytes) <= size - offset;
}

// Marks the MSI-X capability of `device` valid, with the physical ranges of its
// table and PBA, if both lie inside usable memory BARs of the function and do
// not overlap; otherwise invalid, without them.
static void ValidateMsix(PciDevice* const device) {
  PciMsix& msix = device->msix;
  msix.valid = false;
  msix.table_phys = 0;
  msix.table_bytes = 0;
  msix.pba_phys = 0;
  msix.pba_bytes = 0;
  if (msix.offset == 0) {
    return;
  }
  const PciBar* const table_bar = UsableMemoryBar(*device, msix.table_bar);
  const PciBar* const pba_bar = UsableMemoryBar(*device, msix.pba_bar);
  const int64_t table_bytes = int64_t{kPciMsixEntryBytes} * msix.table_size;
  const int64_t pba_bytes =
      kPciMsixPbaEntryBytes *
      ((msix.table_size + kPciMsixVectorsPerPbaEntry - 1) /
       kPciMsixVectorsPerPbaEntry);
  if (table_bar == nullptr || pba_bar == nullptr ||
      !OffsetRangeInBar(*table_bar, msix.table_offset, table_bytes) ||
      !OffsetRangeInBar(*pba_bar, msix.pba_offset, pba_bytes)) {
    return;
  }
  const uint64_t table_end = uint64_t{msix.table_offset} + table_bytes;
  const uint64_t pba_end = uint64_t{msix.pba_offset} + pba_bytes;
  if (table_bar == pba_bar && msix.table_offset < pba_end &&
      msix.pba_offset < table_end) {
    return;
  }
  msix.valid = true;
  msix.table_phys = table_bar->base + msix.table_offset;
  msix.table_bytes = table_bytes;
  msix.pba_phys = pba_bar->base + msix.pba_offset;
  msix.pba_bytes = pba_bytes;
}

// Whether `owner` may change the power state of function `index`: it owns it,
// or `index` is a bridge between a function that it owns and that function's
// root bus. Called with the PCI lock held.
static bool MayManagePowerLocked(const int index, const uintptr_t owner) {
  if (g_owners[index] == owner) {
    return true;
  }
  if (g_devices[index].header_type != kPciHeaderTypeBridge) {
    return false;
  }
  // A function behind a bridge always has a larger index than the bridge.
  const int count = g_device_count.load(std::memory_order_relaxed);
  for (int i = index + 1; i < count; ++i) {
    if (g_owners[i] != owner) {
      continue;
    }
    for (int bridge = g_devices[i].parent_index; bridge != kPciNoParent;
         bridge = g_devices[bridge].parent_index) {
      if (bridge == index) {
        return true;
      }
    }
  }
  return false;
}

// Which PCI Express registers a function of Device/Port Type `type` has
// [PCIe Base 6.0 §7.5.3], as Linux's pcie_capability_reg_implemented() decides:
// the link registers belong to the functions at either end of a link (not to
// Root Complex Integrated Endpoints and Event Collectors), the slot registers
// to downstream ports with a slot, the root registers to Root Ports and Root
// Complex Event Collectors.
static bool PcieHasLinkRegisters(const int type) {
  return type == kPcieTypeEndpoint || type == kPcieTypeLegacyEndpoint ||
         type == kPcieTypeRootPort || type == kPcieTypeUpstreamPort ||
         type == kPcieTypeDownstreamPort || type == kPcieTypePcieToPciBridge ||
         type == kPcieTypePciToPcieBridge;
}

static bool PcieHasSlotRegisters(const int type, const uint16_t capabilities) {
  return (type == kPcieTypeRootPort || type == kPcieTypeDownstreamPort ||
          type == kPcieTypePcieToPciBridge) &&
         (capabilities & kPciExpressSlotImplemented) != 0;
}

static bool PcieHasRootRegisters(const int type) {
  return type == kPcieTypeRootPort ||
         type == kPcieTypeRootComplexEventCollector;
}

// Whether the register at `offset` of `device` is one of a bridge's routing
// registers (kPciBridgeRoutingRegisters).
static bool IsBridgeRoutingRegister(const PciDevice& device, const int offset) {
  return device.header_type == kPciHeaderTypeBridge &&
         offset >= kPciBridgeRoutingStart && offset < kPciBridgeRoutingEnd;
}

// Saves the configuration of `device` that PciRestoreAfterPowerUp() writes
// back after a reset (PciPowerUpContext), in ascending order of offset. Called
// with the PCI lock held.
static PciPowerUpContext SaveContextLocked(const PciDevice& device) {
  const PciAddress address = device.address;
  PciPowerUpContext context{};
  context.saved = true;
  context.command = Read16Locked(address, kPciCommandOffset);
  const auto save = [&](const int offset, const int width) {
    DCHECK(context.register_count < PciPowerUpContext::kMaxRegisters);
    context.registers[context.register_count++] = {
        .offset = offset,
        .width = width,
        .value = ReadWidthLocked(address, offset, width),
    };
  };
  save(kPciCacheLineSizeOffset, 1);
  save(kPciLatencyTimerOffset, 1);
  if (device.header_type == kPciHeaderTypeBridge) {
    for (const RegisterSpec& reg : kPciBridgeRoutingRegisters) {
      save(reg.offset, reg.width);
    }
    save(kPciBridgeControlOffset, 2);
    context.registers[context.register_count - 1].value &=
        ~uint32_t{kPciBridgeControlSecondaryBusReset};
  }
  if (device.pcie_offset != 0) {
    const int pcie = device.pcie_offset;
    const uint16_t capabilities =
        Read16Locked(address, pcie + kPciExpressCapabilitiesOffset);
    const int version = capabilities & kPciExpressVersionMask;
    const int type =
        (capabilities >> kPciExpressTypeShift) & kPciExpressTypeMask;
    const auto save_if_fits = [&](const int offset) {
      if (pcie + offset + 2 <= kPciConfigSpaceSize) {
        save(pcie + offset, 2);
      }
    };
    save_if_fits(kPciExpressDeviceControlOffset);
    if (PcieHasLinkRegisters(type)) {
      save_if_fits(kPciExpressLinkControlOffset);
    }
    if (PcieHasSlotRegisters(type, capabilities)) {
      save_if_fits(kPciExpressSlotControlOffset);
    }
    if (PcieHasRootRegisters(type)) {
      save_if_fits(kPciExpressRootControlOffset);
    }
    if (version >= 2) {
      save_if_fits(kPciExpressDeviceControl2Offset);
    }
  }
  return context;
}

// Writes `reg` back to the function at `address` unless it holds that value.
static void RestoreRegisterLocked(const PciAddress address,
                                  const PciPowerUpContext::Register& reg) {
  if (ReadWidthLocked(address, reg.offset, reg.width) != reg.value) {
    WriteWidthLocked(address, reg.offset, reg.width, reg.value);
  }
}

static bool SameBar(const PciBar& a, const PciBar& b) {
  return a.type == b.type && a.prefetchable == b.prefetchable &&
         a.base == b.base && a.size == b.size;
}

// Sizes the BARs of `device`, an endpoint in D0, again, and if they differ
// from its record, corrects the record's BARs and MSI-X geometry. Returns
// whether it did. Called with the PCI lock held.
static bool ResizeBarsLocked(PciDevice* const device) {
  PciBar bars[kPciMaxBars];
  DecodeBarsLocked(device->address,         //
                   kPciEndpointBarCount,    //
                   /*size_assigned=*/true,  //
                   bars);
  bool same = true;
  for (int i = 0; i < kPciMaxBars; ++i) {
    same = same && SameBar(bars[i], device->bars[i]);
  }
  if (same) {
    return false;
  }
  PciDevice corrected = *device;
  for (int i = 0; i < kPciMaxBars; ++i) {
    corrected.bars[i] = bars[i];
  }
  ValidateMsix(&corrected);
  *device = corrected;
  return true;
}

static bool IsUsbHostController(const PciDevice& device,
                                const uint8_t prog_if) {
  return device.class_code == kPciClassSerialBus &&
         device.subclass == kPciSubclassUsb && device.prog_if == prog_if;
}

// Whether the device table holds an Intel EHCI controller.
static bool IntelEhciExists() {
  const int count = g_device_count.load(std::memory_order_acquire);
  for (int i = 0; i < count; ++i) {
    if (g_devices[i].vendor_id == kPciVendorIntel &&
        IsUsbHostController(g_devices[i], kPciProgIfEhci)) {
      return true;
    }
  }
  return false;
}

// usb_enable_intel_xhci_ports() of Linux for the Intel xHCI `device`: routes
// every switchable port to it, the SuperSpeed terminations first so that USB
// 3.0 devices connect at SuperSpeed rather than at USB 2.0 speed, and returns
// the routing as read back. Called with the PCI lock held.
static PciClaimQuirks RouteIntelUsbPortsToXhciLocked(const PciDevice& device) {
  const PciAddress address = device.address;
  PciClaimQuirks result{};
  Write32Locked(address,                //
                kIntelUsb3PssenOffset,  //
                Read32Locked(address, kIntelUsb3prmOffset));
  result.usb3_ports = Read32Locked(address, kIntelUsb3PssenOffset);
  Write32Locked(address,              //
                kIntelXusb2prOffset,  //
                Read32Locked(address, kIntelXusb2prmOffset));
  result.usb2_ports = Read32Locked(address, kIntelXusb2prOffset);
  result.intel_usb_ports_routed = true;
  return result;
}

static bool IsPresent(const uint16_t vendor_id) {
  return vendor_id != kPciVendorIdAbsent && vendor_id != kPciVendorIdZero &&
         vendor_id != kPciVendorIdRetry;
}

static uint8_t ReadConfig8(const PciAddress address, const int offset) {
  const IrqSpinLockGuard guard(g_pci_lock);
  return Read8Locked(address, offset);
}

static uint16_t ReadVendorId(const PciAddress address) {
  const IrqSpinLockGuard guard(g_pci_lock);
  return Read16Locked(address, kPciVendorIdOffset);
}

// Whether `device` has a Power Management capability whose power state is not
// D0 (and the function answers). Called with the PCI lock held.
static bool InLowPowerStateLocked(const PciDevice& device) {
  if (device.pm_offset == 0) {
    return false;
  }
  const uint16_t pmcsr = Read16Locked(
      device.address, device.pm_offset + kPciPmControlStatusOffset);
  return pmcsr != 0xFFFF && (pmcsr & kPciPmcsrPowerStateMask) != 0;
}

// Whether `device` or any bridge between it and its root bus is in D1, D2 or
// D3hot. The highest sleeping bridge on the path is on a reachable bus, so
// walking up `parent_index` finds it even when functions below it read 0xFFFF.
// Called with the PCI lock held.
static bool PathInLowPowerLocked(const PciDevice& device) {
  if (InLowPowerStateLocked(device)) {
    return true;
  }
  for (int bridge = device.parent_index; bridge != kPciNoParent;
       bridge = g_devices[bridge].parent_index) {
    if (InLowPowerStateLocked(g_devices[bridge])) {
      return true;
    }
  }
  return false;
}

// Turns off MSI and MSI-X on `device` if either is on: the new owner's
// irq_alloc enables MSI or MSI-X, never both, because with both on the
// function's behavior is undefined [PCI Local Bus 3.0 §6.8]. Called with the
// PCI lock held.
static void DisableMsiAndMsixLocked(const PciDevice& device) {
  if (device.msix.offset != 0) {
    Update16Locked(device.address,                             //
                   device.msix.offset + kPciMsiControlOffset,  //
                   0,                                          //
                   kPciMsixControlEnable);
  }
  if (device.msi.offset != 0) {
    Update16Locked(device.address,                            //
                   device.msi.offset + kPciMsiControlOffset,  //
                   0,                                         //
                   kPciMsiControlEnable);
  }
}

// Records the present function at `address`, on a bus that the bridge with
// index `parent_index` leads to, and returns its index; or counts it as
// unrecorded and returns kPciNoParent if the table is full. Reads (and sizes)
// everything under one hold of the PCI lock.
static int RecordFunction(const PciAddress address,  //
                          const int parent_index,    //
                          const bool multi_function) {
  const int index = g_device_count.load(std::memory_order_relaxed);
  if (index == kPciMaxDevices) {
    ++g_unrecorded_count;
    return kPciNoParent;
  }
  PciDevice device;
  device.address = address;
  device.parent_index = parent_index;
  device.multi_function = multi_function;
  bool sized_in_low_power = false;
  {
    const IrqSpinLockGuard guard(g_pci_lock);
    const uint32_t ids = Read32Locked(address, kPciVendorIdOffset);
    const uint32_t class_revision = Read32Locked(address, kPciRevisionOffset);
    device.vendor_id = static_cast<uint16_t>(ids & 0xFFFF);
    device.device_id = static_cast<uint16_t>(ids >> 16);
    device.revision = static_cast<uint8_t>(class_revision & 0xFF);
    device.prog_if = static_cast<uint8_t>((class_revision >> 8) & 0xFF);
    device.subclass = static_cast<uint8_t>((class_revision >> 16) & 0xFF);
    device.class_code = static_cast<uint8_t>(class_revision >> 24);
    device.header_type =
        Read8Locked(address, kPciHeaderTypeOffset) & kPciHeaderTypeLayoutMask;
    device.claimable = device.header_type == kPciHeaderTypeEndpoint &&
                       device.class_code != kPciClassBridge;
    if (device.header_type == kPciHeaderTypeEndpoint) {
      // Only drivers use BARs, so only claimable functions get sized: host
      // bridges may stop decoding RAM, and other bridges their whole bus.
      DecodeBarsLocked(address,               //
                       kPciEndpointBarCount,  //
                       device.claimable,      //
                       device.bars);
      ParseCapabilitiesLocked(address, &device);
      sized_in_low_power = device.claimable && InLowPowerStateLocked(device);
    } else if (device.header_type == kPciHeaderTypeBridge) {
      DecodeBarsLocked(address,                  //
                       kPciBridgeBarCount,       //
                       /*size_assigned=*/false,  //
                       device.bars);
      ParseCapabilitiesLocked(address, &device);
      device.secondary_bus = Read8Locked(address, kPciSecondaryBusOffset);
      device.subordinate_bus = Read8Locked(address, kPciSubordinateBusOffset);
    }
  }
  ValidateMsix(&device);
  g_devices[index] = device;
  g_sized_in_low_power[index] = sized_in_low_power;
  g_device_count.store(index + 1, std::memory_order_release);
  return index;
}

// Scans the devices of `bus`, which the bridge with index `parent_index` leads
// to (kPciNoParent for a root bus): records every present function and queues
// the secondary bus of every PCI-to-PCI bridge, unless that bus is visited or
// not numbered above `bus` (a loop).
static void ScanBus(const int bus,           //
                    const int parent_index,  //
                    BusWalk* const walk) {
  for (int device = 0; device < kPciDevicesPerBus; ++device) {
    const PciAddress function0 = {static_cast<uint8_t>(bus),
                                  static_cast<uint8_t>(device), 0};
    if (!IsPresent(ReadVendorId(function0))) {
      continue;
    }
    // Only function 0 tells whether the device has other functions: a
    // single-function device may answer at every function number.
    const bool multi_function = (ReadConfig8(function0, kPciHeaderTypeOffset) &
                                 kPciHeaderTypeMultiFunction) != 0;
    const int functions = multi_function ? kPciFunctionsPerDevice : 1;
    for (int function = 0; function < functions; ++function) {
      const PciAddress address = {static_cast<uint8_t>(bus),
                                  static_cast<uint8_t>(device),
                                  static_cast<uint8_t>(function)};
      if (function != 0 && !IsPresent(ReadVendorId(address))) {
        continue;
      }
      const int index = RecordFunction(address, parent_index, multi_function);
      const int layout =
          ReadConfig8(address, kPciHeaderTypeOffset) & kPciHeaderTypeLayoutMask;
      if (layout != kPciHeaderTypeBridge && layout != kPciHeaderTypeCardBus) {
        continue;
      }
      const int secondary = ReadConfig8(address, kPciSecondaryBusOffset);
      if (secondary <= bus || walk->visited[secondary]) {
        continue;
      }
      walk->visited[secondary] = true;
      // A CardBus bridge's bus is only reserved: PC Card devices are not
      // supported, and must not be taken for a root bus's devices either.
      if (layout == kPciHeaderTypeBridge) {
        DCHECK(walk->tail < kPciBusCount);
        walk->queue[walk->tail++] = {static_cast<uint8_t>(secondary),
                                     static_cast<int16_t>(index)};
      }
    }
  }
}

// Scans `root_bus` and, breadth first, every bus that bridges below it lead
// to.
static void ScanFromRootBus(const int root_bus, BusWalk* const walk) {
  DCHECK(!walk->visited[root_bus] && walk->tail < kPciBusCount);
  walk->visited[root_bus] = true;
  walk->queue[walk->tail++] = {static_cast<uint8_t>(root_bus),
                               static_cast<int16_t>(kPciNoParent)};
  while (walk->head < walk->tail) {
    const PendingBus pending = walk->queue[walk->head++];
    ScanBus(pending.bus, pending.parent_index, walk);
  }
}

// A line of the boot report, built in place. Text beyond the capacity is cut
// off, so the line always stays NUL-terminated.
class ReportLine {
 public:
  void Append(const char* text) {
    while (*text != '\0' && length_ < kPciReportLineCapacity - 1) {
      buffer_[length_++] = *text++;
    }
    buffer_[length_] = '\0';
  }

  // Lowercase hexadecimal, without a prefix, at least `min_digits` digits.
  void AppendHex(const uint64_t value, const int min_digits) {
    char digits[17] = {};
    int pos = 16;
    uint64_t remaining = value;
    do {
      digits[--pos] = "0123456789abcdef"[remaining & 0xF];
      remaining >>= 4;
    } while (remaining != 0 || 16 - pos < min_digits);
    Append(&digits[pos]);
  }

  void AppendDec(const int64_t value) {
    char digits[21] = {};
    int pos = 20;
    uint64_t remaining = static_cast<uint64_t>(value < 0 ? 0 : value);
    do {
      digits[--pos] = static_cast<char>('0' + remaining % 10);
      remaining /= 10;
    } while (remaining != 0);
    Append(&digits[pos]);
  }

  const char* text() const { return buffer_; }

 private:
  char buffer_[kPciReportLineCapacity] = {};
  int length_ = 0;
};

static const char* BarTypeName(const PciBarType type) {
  switch (type) {
    case PciBarType::kIo:
      return "io";
    case PciBarType::kMem32:
      return "mem32";
    case PciBarType::kMem64:
      return "mem64";
    case PciBarType::kNone:
      break;
  }
  return "none";
}

static void AppendDeviceLine(const PciDevice& device, ReportLine* const line) {
  line->Append("[PCI] ");
  line->AppendHex(device.address.bus, 2);
  line->Append(":");
  line->AppendHex(device.address.device, 2);
  line->Append(".");
  line->AppendHex(device.address.function, 1);
  line->Append(" ");
  line->AppendHex(device.vendor_id, 4);
  line->Append(":");
  line->AppendHex(device.device_id, 4);
  line->Append(" class ");
  line->AppendHex(device.class_code, 2);
  line->Append(".");
  line->AppendHex(device.subclass, 2);
  line->Append(".");
  line->AppendHex(device.prog_if, 2);
  if (device.header_type == kPciHeaderTypeBridge) {
    line->Append(" bridge=");
    line->AppendHex(device.secondary_bus, 2);
    line->Append("-");
    line->AppendHex(device.subordinate_bus, 2);
  } else if (device.header_type != kPciHeaderTypeEndpoint) {
    line->Append(" hdr=");
    line->AppendHex(device.header_type, 2);
  }
  for (int i = 0; i < kPciMaxBars; ++i) {
    const PciBar& bar = device.bars[i];
    if (bar.type == PciBarType::kNone) {
      continue;
    }
    line->Append(" bar");
    line->AppendDec(i);
    line->Append("=");
    line->Append(BarTypeName(bar.type));
    line->Append(bar.prefetchable ? "p:0x" : ":0x");
    line->AppendHex(bar.base, 1);
    if (bar.size > 0) {
      line->Append("+0x");
      line->AppendHex(static_cast<uint64_t>(bar.size), 1);
    }
  }
  if (device.msi.offset != 0) {
    line->Append(device.msi.is_64bit ? " msi64" : " msi32");
    if (device.msi.per_vector_masking) {
      line->Append(" pvm");
    }
  }
  if (device.msix.offset != 0) {
    if (device.msix.valid) {
      line->Append(" msix=");
      line->AppendDec(device.msix.table_size);
    } else {
      line->Append(" msix-ignored");
    }
  }
  line->Append("\n");
}

}  // namespace

uint32_t PciMechanism1Address(const PciAddress address, const int offset) {
  return 0x80000000u | (static_cast<uint32_t>(address.bus) << 16) |
         ((static_cast<uint32_t>(address.device) & 0x1Fu) << 11) |
         ((static_cast<uint32_t>(address.function) & 0x7u) << 8) |
         (static_cast<uint32_t>(offset) & 0xFCu);
}

uint16_t PciMechanism1DataPort(const int offset) {
  return static_cast<uint16_t>(0xCFC + (offset & 3));
}

void PciInit(std::shared_ptr<PciConfigSpace> config) {
  CHECK(config != nullptr);
  int claimed_count = 0;
  {
    const IrqSpinLockGuard guard(g_pci_lock);
    claimed_count = g_claimed_count;
  }
  CHECK(claimed_count == 0);
  g_config = std::move(config);
  g_device_count.store(0, std::memory_order_release);
  g_unrecorded_count = 0;

  BusWalk walk;
  ScanFromRootBus(0, &walk);
  // Peer root buses (e.g. of a second host bridge): every other bus that no
  // bridge leads to, if some device answers on it. On the other buses, this
  // costs one vendor ID read per device number.
  for (int bus = 1; bus < kPciBusCount; ++bus) {
    if (!walk.visited[bus]) {
      ScanFromRootBus(bus, &walk);
    }
  }
}

int PciDeviceCount() { return g_device_count.load(std::memory_order_acquire); }

const PciDevice* PciGetDevice(const int index) {
  return IsValidIndex(index) ? &g_devices[index] : nullptr;
}

StatusOr<PciDevice> PciCopyDevice(const int index) {
  if (!IsValidIndex(index)) {
    return kStatusError;
  }
  const IrqSpinLockGuard guard(g_pci_lock);
  return g_devices[index];
}

int PciFindDevice(const PciAddress address) {
  const int count = PciDeviceCount();
  for (int index = 0; index < count; ++index) {
    if (g_devices[index].address == address) {
      return index;
    }
  }
  return -1;
}

void PciWriteBootReport(void (*const write)(const char* text)) {
  DCHECK(write != nullptr);
  const int count = PciDeviceCount();
  for (int index = 0; index < count; ++index) {
    ReportLine line;
    AppendDeviceLine(g_devices[index], &line);
    write(line.text());
  }
  ReportLine summary;
  summary.Append("[PCI] ");
  summary.AppendDec(count);
  summary.Append(count == 1 ? " function" : " functions");
  if (g_unrecorded_count > 0) {
    summary.Append(", ");
    summary.AppendDec(g_unrecorded_count);
    summary.Append(" not recorded (table full)");
  }
  summary.Append("\n");
  write(summary.text());
}

uint8_t PciConfigRead8(const int index, const int offset) {
  const PciAddress address = CheckedAddress(index);
  CheckOffset(offset, 1);
  const IrqSpinLockGuard guard(g_pci_lock);
  return Read8Locked(address, offset);
}

uint16_t PciConfigRead16(const int index, const int offset) {
  const PciAddress address = CheckedAddress(index);
  CheckOffset(offset, 2);
  const IrqSpinLockGuard guard(g_pci_lock);
  return Read16Locked(address, offset);
}

uint32_t PciConfigRead32(const int index, const int offset) {
  const PciAddress address = CheckedAddress(index);
  CheckOffset(offset, 4);
  const IrqSpinLockGuard guard(g_pci_lock);
  return Read32Locked(address, offset);
}

void PciConfigWrite8(const int index, const int offset, const uint8_t value) {
  const PciAddress address = CheckedAddress(index);
  CheckOffset(offset, 1);
  const IrqSpinLockGuard guard(g_pci_lock);
  Write8Locked(address, offset, value);
}

void PciConfigWrite16(const int index, const int offset, const uint16_t value) {
  const PciAddress address = CheckedAddress(index);
  CheckOffset(offset, 2);
  const IrqSpinLockGuard guard(g_pci_lock);
  Write16Locked(address, offset, value);
}

void PciConfigWrite32(const int index, const int offset, const uint32_t value) {
  const PciAddress address = CheckedAddress(index);
  CheckOffset(offset, 4);
  CHECK(offset != kPciCommandOffset);
  const IrqSpinLockGuard guard(g_pci_lock);
  Write32Locked(address, offset, value);
}

uint16_t PciConfigUpdate16(const int index,     //
                           const int offset,    //
                           const uint16_t set,  //
                           const uint16_t clear) {
  const PciAddress address = CheckedAddress(index);
  CheckOffset(offset, 2);
  CHECK((set & clear) == 0);
  const IrqSpinLockGuard guard(g_pci_lock);
  return Update16Locked(address, offset, set, clear);
}

uint16_t PciUpdateCommand(const int index,     //
                          const uint16_t set,  //
                          const uint16_t clear) {
  return PciConfigUpdate16(index, kPciCommandOffset, set, clear);
}

PciStatus PciClaim(const int index, const uintptr_t owner) {
  CHECK(owner != kPciNoOwner);
  if (!IsValidIndex(index)) {
    return PciStatus::kNoDevice;
  }
  const PciDevice& device = g_devices[index];
  if (!device.claimable) {
    return PciStatus::kNotClaimable;
  }
  const IrqSpinLockGuard guard(g_pci_lock);
  if (g_owners[index] == owner) {
    return PciStatus::kOk;
  }
  if (g_owners[index] != kPciNoOwner) {
    return PciStatus::kBusy;
  }
  g_owners[index] = owner;
  ++g_claimed_count;
  // If the function or a bridge above it is in D1, D2 or D3hot, leave the
  // configuration writes to PciRestoreAfterPowerUp() once the path is in D0.
  if (!PathInLowPowerLocked(device)) {
    // Enable Bus Master as well as Memory Space on upstream bridges (as Linux's
    // pci_enable_bridge() does) so USB4/Thunderbolt tunnels forward BAR read
    // completions before the driver enables Bus Master on the endpoint itself.
    SetUpstreamBridgeCommandLocked(
        device, kPciCommandMemorySpace | kPciCommandBusMaster);
    Update16Locked(device.address,                                        //
                   kPciCommandOffset,                                     //
                   kPciCommandMemorySpace | kPciCommandInterruptDisable,  //
                   0);
    DisableMsiAndMsixLocked(device);
  }
  return PciStatus::kOk;
}

PciStatus PciRelease(const int index, const uintptr_t owner) {
  CHECK(owner != kPciNoOwner);
  if (!IsValidIndex(index)) {
    return PciStatus::kNoDevice;
  }
  const IrqSpinLockGuard guard(g_pci_lock);
  if (g_owners[index] != owner) {
    return PciStatus::kNotOwner;
  }
  QuiesceLocked(g_devices[index]);
  g_owners[index] = kPciNoOwner;
  --g_claimed_count;
  return PciStatus::kOk;
}

int PciReleaseAll(const uintptr_t owner) {
  CHECK(owner != kPciNoOwner);
  int released = 0;
  const int count = PciDeviceCount();
  for (int index = 0; index < count; ++index) {
    if (PciRelease(index, owner) == PciStatus::kOk) {
      ++released;
    }
  }
  return released;
}

uintptr_t PciOwner(const int index) {
  if (!IsValidIndex(index)) {
    return kPciNoOwner;
  }
  const IrqSpinLockGuard guard(g_pci_lock);
  return g_owners[index];
}

int PciClaimedCount() {
  const IrqSpinLockGuard guard(g_pci_lock);
  return g_claimed_count;
}

PciStatus PciSetBusMaster(const int index,        //
                          const uintptr_t owner,  //
                          const bool enable) {
  CHECK(owner != kPciNoOwner);
  if (!IsValidIndex(index)) {
    return PciStatus::kNoDevice;
  }
  const PciDevice& device = g_devices[index];
  const IrqSpinLockGuard guard(g_pci_lock);
  if (g_owners[index] != owner) {
    return PciStatus::kNotOwner;
  }
  if (enable) {
    SetUpstreamBridgeCommandLocked(
        device, kPciCommandMemorySpace | kPciCommandBusMaster);
    Update16Locked(device.address, kPciCommandOffset, kPciCommandBusMaster, 0);
  } else {
    Update16Locked(device.address, kPciCommandOffset, 0, kPciCommandBusMaster);
  }
  return PciStatus::kOk;
}

StatusOr<PciPowerTransition, PciStatus> PciSetPowerStateD0(
    const int index, const uintptr_t owner, const bool save_context) {
  CHECK(owner != kPciNoOwner);
  if (!IsValidIndex(index)) {
    return PciStatus::kNoDevice;
  }
  const PciDevice& device = g_devices[index];
  const IrqSpinLockGuard guard(g_pci_lock);
  if (!MayManagePowerLocked(index, owner)) {
    return PciStatus::kNotOwner;
  }
  PciPowerTransition transition{};
  if (device.pm_offset == 0) {
    return transition;
  }
  const int offset = device.pm_offset + kPciPmControlStatusOffset;
  const uint16_t pmcsr = Read16Locked(device.address, offset);
  const uint16_t state = pmcsr & kPciPmcsrPowerStateMask;
  // All ones: the function doesn't answer at all.
  if (state == 0 || pmcsr == 0xFFFF) {
    return transition;
  }
  // A function keeps its configuration in D1, D2 and D3hot; the transition to
  // D0 may reset it.
  if (save_context) {
    transition.context = SaveContextLocked(device);
  }
  // PME_Status is write-1-to-clear: writing it as 0 leaves it as it is.
  Write16Locked(device.address,  //
                offset,          //
                static_cast<uint16_t>(
                    pmcsr & ~(kPciPmcsrPowerStateMask | kPciPmcsrPmeStatus)));
  transition.from = static_cast<PciPowerState>(state);
  return transition;
}

int64_t PciPowerUpDelayMicroseconds(const PciPowerState from) {
  switch (from) {
    case PciPowerState::kD3hot:
      return 10000;
    case PciPowerState::kD2:
      return 200;
    case PciPowerState::kD0:
    case PciPowerState::kD1:
      break;
  }
  return 0;
}

bool PciFunctionResponds(const int index) {
  const PciAddress address = CheckedAddress(index);
  const IrqSpinLockGuard guard(g_pci_lock);
  return Read32Locked(address, kPciCommandOffset) != 0xFFFFFFFFu;
}

StatusOr<bool, PciStatus> PciRestoreAfterPowerUp(
    const int index, const uintptr_t owner,
    const PciPowerUpContext* const context) {
  CHECK(owner != kPciNoOwner);
  if (!IsValidIndex(index)) {
    return PciStatus::kNoDevice;
  }
  PciDevice& device = g_devices[index];
  const PciAddress address = device.address;
  const bool saved = context != nullptr && context->saved;
  const int saved_count = saved ? context->register_count : 0;
  DCHECK(saved_count >= 0 && saved_count <= PciPowerUpContext::kMaxRegisters);
  const IrqSpinLockGuard guard(g_pci_lock);
  if (!MayManagePowerLocked(index, owner)) {
    return PciStatus::kNotOwner;
  }
  // From the top of the configuration space down, as Linux's
  // pci_restore_state() does, and Command last: first the PCI Express
  // registers and Bridge Control.
  for (int i = saved_count - 1; i >= 0; --i) {
    if (context->registers[i].offset >= kPciBridgeRoutingEnd) {
      RestoreRegisterLocked(address, context->registers[i]);
    }
  }
  // Then the BARs and a bridge's routing registers, with the function's
  // decoding (and Bus Master) off while any of them is written, so that no
  // half-written address range is ever decoded.
  bool routing_intact = true;
  for (int i = 0; i < saved_count; ++i) {
    const PciPowerUpContext::Register& reg = context->registers[i];
    if (IsBridgeRoutingRegister(device, reg.offset) &&
        ReadWidthLocked(address, reg.offset, reg.width) != reg.value) {
      routing_intact = false;
    }
  }
  if (!routing_intact || !BarsIntactLocked(device)) {
    Update16Locked(address,            //
                   kPciCommandOffset,  //
                   0,                  //
                   kPciCommandDecode | kPciCommandBusMaster);
    for (int i = saved_count - 1; i >= 0; --i) {
      if (IsBridgeRoutingRegister(device, context->registers[i].offset)) {
        RestoreRegisterLocked(address, context->registers[i]);
      }
    }
    RestoreBarsLocked(device);
  }
  bool resized = false;
  const bool owned = g_owners[index] == owner;
  if (owned && g_sized_in_low_power[index] &&
      Read32Locked(address, kPciCommandOffset) != 0xFFFFFFFFu) {
    g_sized_in_low_power[index] = false;
    resized = ResizeBarsLocked(&device);
  }
  // Cache Line Size and Latency Timer.
  for (int i = saved_count - 1; i >= 0; --i) {
    if (context->registers[i].offset < kPciBar0Offset) {
      RestoreRegisterLocked(address, context->registers[i]);
    }
  }
  if (owned) {
    SetUpstreamBridgeCommandLocked(
        device, kPciCommandMemorySpace | kPciCommandBusMaster);
    DisableMsiAndMsixLocked(device);
    const uint16_t saved_command =
        saved ? static_cast<uint16_t>(context->command & ~kPciCommandBusMaster)
              : 0;
    Update16Locked(
        address,            //
        kPciCommandOffset,  //
        static_cast<uint16_t>(saved_command | kPciCommandMemorySpace |
                              kPciCommandInterruptDisable),
        0);
  } else if (saved) {
    Update16Locked(address, kPciCommandOffset, context->command, 0);
  }
  return resized;
}

StatusOr<PciClaimQuirks, PciStatus> PciApplyClaimQuirks(const int index,
                                                        const uintptr_t owner) {
  CHECK(owner != kPciNoOwner);
  if (!IsValidIndex(index)) {
    return PciStatus::kNoDevice;
  }
  const PciDevice& device = g_devices[index];
  const IrqSpinLockGuard guard(g_pci_lock);
  if (g_owners[index] != owner) {
    return PciStatus::kNotOwner;
  }
  PciClaimQuirks result{};
  if (device.vendor_id == kPciVendorIntel &&
      IsUsbHostController(device, kPciProgIfXhci) && IntelEhciExists()) {
    const bool sony_vaio_t_series =
        Read16Locked(device.address, kPciSubsystemVendorIdOffset) ==
            kPciVendorSony &&
        Read16Locked(device.address, kPciSubsystemIdOffset) ==
            kSonyVaioTSeriesSubsystemId;
    if (!sony_vaio_t_series) {
      result = RouteIntelUsbPortsToXhciLocked(device);
    }
  }
  return result;
}

PciQuiesceResult PciQuiesce(const int index) {
  CHECK(IsValidIndex(index));
  const IrqSpinLockGuard guard(g_pci_lock);
  return QuiesceLocked(g_devices[index]);
}

bool PciBarContainsRange(const PciBar& bar,    //
                         const uint64_t phys,  //
                         const int64_t size) {
  const bool memory =
      bar.type == PciBarType::kMem32 || bar.type == PciBarType::kMem64;
  if (!memory || bar.base == 0 || bar.size <= 0 || size <= 0 ||
      size > bar.size || phys < bar.base) {
    return false;
  }
  return phys - bar.base <= static_cast<uint64_t>(bar.size - size);
}

#if __STDC_HOSTED__
void PciResetForTest() {
  {
    const IrqSpinLockGuard guard(g_pci_lock);
    for (uintptr_t& owner : g_owners) {
      owner = kPciNoOwner;
    }
    for (bool& sized_in_low_power : g_sized_in_low_power) {
      sized_in_low_power = false;
    }
    g_claimed_count = 0;
    g_device_count.store(0, std::memory_order_release);
    g_unrecorded_count = 0;
  }
  g_config.reset();
}
#endif

#if !__STDC_HOSTED__
namespace {

constexpr uint16_t kPciConfigAddressPort = 0xCF8;

// Configuration mechanism #1. CONFIG_ADDRESS is written only with 32-bit
// accesses (port_io.h), and each data access has the access width.
class KernelPciConfigSpace final : public PciConfigSpace {
 public:
  uint8_t Read8(const PciAddress address, const int offset) override {
    PortOut32(kPciConfigAddressPort, PciMechanism1Address(address, offset));
    return PortIn8(PciMechanism1DataPort(offset));
  }

  uint16_t Read16(const PciAddress address, const int offset) override {
    PortOut32(kPciConfigAddressPort, PciMechanism1Address(address, offset));
    return PortIn16(PciMechanism1DataPort(offset));
  }

  uint32_t Read32(const PciAddress address, const int offset) override {
    PortOut32(kPciConfigAddressPort, PciMechanism1Address(address, offset));
    return PortIn32(PciMechanism1DataPort(offset));
  }

  void Write8(const PciAddress address,  //
              const int offset,          //
              const uint8_t value) override {
    PortOut32(kPciConfigAddressPort, PciMechanism1Address(address, offset));
    PortOut8(PciMechanism1DataPort(offset), value);
  }

  void Write16(const PciAddress address,  //
               const int offset,          //
               const uint16_t value) override {
    PortOut32(kPciConfigAddressPort, PciMechanism1Address(address, offset));
    PortOut16(PciMechanism1DataPort(offset), value);
  }

  void Write32(const PciAddress address,  //
               const int offset,          //
               const uint32_t value) override {
    PortOut32(kPciConfigAddressPort, PciMechanism1Address(address, offset));
    PortOut32(PciMechanism1DataPort(offset), value);
  }
};

}  // namespace

std::shared_ptr<PciConfigSpace> CreateKernelPciConfigSpace() {
  return std::make_shared<KernelPciConfigSpace>();
}
#endif

}  // namespace protos
