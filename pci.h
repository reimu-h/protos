#ifndef PROTOS_PCI_H_
#define PROTOS_PCI_H_

#include <cstdint>
#include <memory>

#include "status_or.h"

namespace protos {

// PCI functions, their configuration space and their owners, for the generic
// user-mode driver primitives. Nothing here knows a device class except the
// PCI-to-PCI bridge rules of the PCI specifications: bridges forward their
// buses, and they and the other bridge functions (class 06h) are shared
// infrastructure, which drivers can't own.
//
// PciInit() enumerates every function once at boot, through a PciConfigSpace,
// into a fixed device table that never changes afterwards: firmware assigns
// bus numbers and BARs, and there is no PCI hot-plug. Devices are named by
// their index in that table, 0 .. PciDeviceCount() - 1, and a function behind
// bridges always has a larger index than each of its bridges.
//
// Locking: every configuration-space access below takes the PCI lock, an
// IrqSpinLock in pci.cpp. It is a leaf lock: callers may hold other locks
// (taken before it), and pci.cpp logs, allocates and waits for nothing while it
// holds it, so all functions may be called on any CPU, also with interrupts
// disabled. Multi-access sequences (a read-modify-write of a register, the
// sizing of a function's BARs, a claim with its command updates) are atomic
// with respect to each other. The device records need no lock: they never
// change after PciInit(), except for the BARs and the MSI-X geometry of a
// function that PciInit() had to size in a low-power state, which the first
// PciRestoreAfterPowerUp() of its owner may correct (see there).

// The location of a function: bus 0..255, device 0..31, function 0..7.
struct PciAddress {
  uint8_t bus = 0;
  uint8_t device = 0;
  uint8_t function = 0;

  friend constexpr bool operator==(const PciAddress&,
                                   const PciAddress&) = default;
};

// The conventional 256-byte configuration space of every function: what
// pci.cpp does to PCI hardware, and nothing more. Offsets are naturally aligned
// (0 <= offset < 256, offset % width == 0), and each access has exactly its
// width: a 16-bit register is never written as part of a 32-bit one (the
// Status register next to Command has write-1-to-clear bits). A function that
// does not exist reads as all ones and ignores writes.
// CreateKernelPciConfigSpace() executes the port I/O; host unit tests use
// FakePciConfigSpace (pci_config_fake.h), which models the registers.
// Implementations need not be thread-safe: pci.cpp makes every call with the
// PCI lock held, which also keeps each address/data port pair together.
class PciConfigSpace {
 public:
  virtual ~PciConfigSpace() = default;

  virtual uint8_t Read8(PciAddress address, int offset) = 0;
  virtual uint16_t Read16(PciAddress address, int offset) = 0;
  virtual uint32_t Read32(PciAddress address, int offset) = 0;
  virtual void Write8(PciAddress address, int offset, uint8_t value) = 0;
  virtual void Write16(PciAddress address, int offset, uint16_t value) = 0;
  virtual void Write32(PciAddress address, int offset, uint32_t value) = 0;
};

#if !__STDC_HOSTED__
// Returns the PciConfigSpace of configuration mechanism #1 (pci.cpp): a 32-bit
// write of PciMechanism1Address() to port 0xCF8, then a data access of the
// access width at port PciMechanism1DataPort().
std::shared_ptr<PciConfigSpace> CreateKernelPciConfigSpace();
#endif

// Configuration mechanism #1 [PCI Local Bus 3.0 §3.2.2.3.2], also for host
// tests: the CONFIG_ADDRESS value (enable bit, bus, device, function, and the
// dword-aligned register number) of `offset` in `address`'s configuration
// space, and the data port, 0xCFC + (offset & 3), at which an access of an
// aligned `offset` of any width transfers the data.
uint32_t PciMechanism1Address(PciAddress address, int offset);
uint16_t PciMechanism1DataPort(int offset);

// Configuration-space registers and bits [PCI Local Bus 3.0 §6.1-6.2, §6.7-6.8,
// PCI-to-PCI Bridge 1.2 §3.2], with their widths in bits. The Revision ID byte
// is followed by the Programming Interface, Subclass and Class Code bytes.
inline constexpr int kPciConfigSpaceSize = 256;
inline constexpr int kPciVendorIdOffset = 0x00;             // 16
inline constexpr int kPciDeviceIdOffset = 0x02;             // 16
inline constexpr int kPciCommandOffset = 0x04;              // 16
inline constexpr int kPciStatusOffset = 0x06;               // 16
inline constexpr int kPciRevisionOffset = 0x08;             // 8
inline constexpr int kPciCacheLineSizeOffset = 0x0C;        // 8
inline constexpr int kPciLatencyTimerOffset = 0x0D;         // 8
inline constexpr int kPciHeaderTypeOffset = 0x0E;           // 8
inline constexpr int kPciBar0Offset = 0x10;                 // 32 each
inline constexpr int kPciSecondaryBusOffset = 0x19;         // 8, header type 1
inline constexpr int kPciSubordinateBusOffset = 0x1A;       // 8, header type 1
inline constexpr int kPciSubsystemVendorIdOffset = 0x2C;    // 16, header type 0
inline constexpr int kPciSubsystemIdOffset = 0x2E;          // 16, header type 0
inline constexpr int kPciCapabilitiesPointerOffset = 0x34;  // 8, types 0, 1

inline constexpr uint16_t kPciCommandIoSpace = 1u << 0;
inline constexpr uint16_t kPciCommandMemorySpace = 1u << 1;
inline constexpr uint16_t kPciCommandBusMaster = 1u << 2;
inline constexpr uint16_t kPciCommandInterruptDisable = 1u << 10;
inline constexpr uint16_t kPciStatusCapabilitiesList = 1u << 4;

// Header Type bits 6:0 (the register layout) and bit 7.
inline constexpr uint8_t kPciHeaderTypeEndpoint = 0x00;
inline constexpr uint8_t kPciHeaderTypeBridge = 0x01;  // also PCIe ports
inline constexpr uint8_t kPciHeaderTypeCardBus = 0x02;
inline constexpr uint8_t kPciHeaderTypeLayoutMask = 0x7F;
inline constexpr uint8_t kPciHeaderTypeMultiFunction = 0x80;
// Base class of host, ISA, PCI-to-PCI and other bridges.
inline constexpr uint8_t kPciClassBridge = 0x06;

// Capability IDs, and the Message Control register (capability + 2, 16 bits)
// of the MSI and MSI-X capabilities [PCI Local Bus 3.0 §6.8.1-6.8.2].
inline constexpr uint8_t kPciCapabilityMsi = 0x05;
inline constexpr uint8_t kPciCapabilityMsix = 0x11;
inline constexpr int kPciMsiControlOffset = 2;
inline constexpr uint16_t kPciMsiControlEnable = 1u << 0;
inline constexpr uint16_t kPciMsiControl64Bit = 1u << 7;
inline constexpr uint16_t kPciMsiControlPerVectorMasking = 1u << 8;
inline constexpr uint16_t kPciMsixControlTableSizeMask = 0x07FF;  // size - 1
inline constexpr uint16_t kPciMsixControlFunctionMask = 1u << 14;
inline constexpr uint16_t kPciMsixControlEnable = 1u << 15;
// Bytes per MSI-X table entry, and table entries per 64-bit PBA entry.
inline constexpr int kPciMsixEntryBytes = 16;
inline constexpr int kPciMsixVectorsPerPbaEntry = 64;

// The Power Management capability [PCI Bus Power Management 1.2 §3.2]: 8
// bytes, whose Control/Status register (PMCSR, capability + 4, 16 bits) holds
// the power state in bits 1:0.
inline constexpr uint8_t kPciCapabilityPowerManagement = 0x01;
inline constexpr int kPciPmCapabilityBytes = 8;
inline constexpr int kPciPmControlStatusOffset = 4;
inline constexpr uint16_t kPciPmcsrPowerStateMask = 0x0003;
inline constexpr uint16_t kPciPmcsrNoSoftReset = 1u << 3;
inline constexpr uint16_t kPciPmcsrPmeStatus = 1u << 15;  // write 1 to clear

// The PCI Express capability [PCIe Base 6.0 §7.5.3] and its 16-bit registers
// that hold configuration (all at the capability's offset plus these). The
// PCI Express Capabilities register holds the capability version in bits 3:0,
// the Device/Port Type in bits 7:4 and Slot Implemented in bit 8; which of
// the other registers exist depends on them.
inline constexpr uint8_t kPciCapabilityExpress = 0x10;
inline constexpr int kPciExpressCapabilitiesOffset = 0x02;
inline constexpr int kPciExpressDeviceControlOffset = 0x08;
inline constexpr int kPciExpressLinkControlOffset = 0x10;
inline constexpr int kPciExpressSlotControlOffset = 0x18;
inline constexpr int kPciExpressRootControlOffset = 0x1C;
inline constexpr int kPciExpressDeviceControl2Offset = 0x28;  // version >= 2

inline constexpr int kPciMaxBars = 6;
// Capacity of the device table. Functions beyond it are counted (the boot
// report names the number) but not recorded.
inline constexpr int kPciMaxDevices = 256;
inline constexpr int kPciNoParent = -1;

enum class PciBarType : uint8_t { kNone, kIo, kMem32, kMem64 };

struct PciBar {
  // kNone: not implemented (or not decodable), or the upper half of the
  // 64-bit BAR below it.
  PciBarType type = PciBarType::kNone;
  bool prefetchable = false;
  // The address that firmware assigned (on x86, bus and physical addresses are
  // equal), without the flag bits; 0 if unassigned.
  uint64_t base = 0;
  // Bytes, a power of two (at least 16 for memory BARs and 4 for I/O BARs), or
  // 0 if unknown: PciInit() sizes only the assigned BARs of claimable
  // functions. Only BARs with a base and a size can be used.
  int64_t size = 0;
};

// The MSI capability of a function.
struct PciMsi {
  // Configuration-space offset of the capability; 0 if the function has none
  // (or one that does not fit in the configuration space).
  int offset = 0;
  bool is_64bit = false;
  bool per_vector_masking = false;
  // log2 of the number of vectors the function can request, 0..5 (the
  // reserved encodings 6 and 7 are recorded as 0).
  int vectors_log2 = 0;
  // Offsets of the Message Data register (16 bits) and of the Mask Bits
  // register (32 bits; 0 without per-vector masking). Message Address is at
  // offset + 4, its upper half (64-bit capabilities only) at offset + 8.
  int data_offset = 0;
  int mask_offset = 0;
};

// The MSI-X capability of a function.
struct PciMsix {
  // Configuration-space offset of the capability; 0 if the function has none.
  int offset = 0;
  // Entries of the MSI-X table, 1..2048.
  int table_size = 0;
  // BAR indicator (BIR) and offset in that BAR of the table and of the
  // Pending Bit Array, as the capability states them.
  int table_bar = 0;
  uint32_t table_offset = 0;
  int pba_bar = 0;
  uint32_t pba_offset = 0;
  // Whether the table and the PBA lie inside assigned, sized memory BARs of
  // the function and do not overlap. Only then are the physical ranges below
  // set, and only then may MSI-X be used (Message Control can always be
  // written, e.g. to disable MSI-X).
  bool valid = false;
  uint64_t table_phys = 0;
  int64_t table_bytes = 0;  // kPciMsixEntryBytes * table_size
  uint64_t pba_phys = 0;
  int64_t pba_bytes = 0;  // 8 bytes per kPciMsixVectorsPerPbaEntry entries
};

struct PciDevice {
  PciAddress address;
  uint16_t vendor_id = 0;
  uint16_t device_id = 0;
  uint8_t class_code = 0;
  uint8_t subclass = 0;
  uint8_t prog_if = 0;
  uint8_t revision = 0;
  // Header Type bits 6:0: kPciHeaderTypeEndpoint, kPciHeaderTypeBridge,
  // kPciHeaderTypeCardBus, or another (unknown) layout.
  uint8_t header_type = 0;
  // Header Type bit 7 of the device's function 0.
  bool multi_function = false;
  // Whether a driver may claim the function: header type 0 and not a bridge
  // (class kPciClassBridge, e.g. the host bridge).
  bool claimable = false;
  // Header type 0: BAR0-5; header type 1: BAR0-1; other layouts: none. A
  // 64-bit BAR is recorded at its lower index.
  PciBar bars[kPciMaxBars];
  // Header type 1: the bus numbers that firmware assigned; 0 otherwise.
  uint8_t secondary_bus = 0;
  uint8_t subordinate_bus = 0;
  // Index of the bridge whose secondary bus the function is on, or
  // kPciNoParent on a root bus (bus 0, and any other bus that no bridge leads
  // to, e.g. of a second host bridge).
  int parent_index = kPciNoParent;
  PciMsi msi;
  PciMsix msix;
  // Configuration-space offset of the Power Management capability; 0 if the
  // function has none (or one that does not fit in the configuration space).
  int pm_offset = 0;
  // Configuration-space offset of the PCI Express capability; 0 if the
  // function has none (or one whose Device Control register does not fit in
  // the configuration space).
  int pcie_offset = 0;
};

// Enumerates every function through `config` (non-null), which pci.cpp then
// uses for good: bus 0 and, through the PCI-to-PCI bridges, the buses behind
// them, then every other bus on which function 0 of a device answers (peer
// root buses). Decodes the IDs, the BARs (sizing the assigned BARs of
// claimable functions, each function with its I/O and memory decoding briefly
// disabled), and the MSI, MSI-X, Power Management and PCI Express
// capabilities. Writes nothing to the consoles, and to configuration space
// only while it sizes BARs; it restores every register it changes. A function
// in a low-power state answers configuration accesses, so its BARs are sized
// there too; the first PciRestoreAfterPowerUp() after its owner took it to D0
// checks the sizes.
// Call it once at boot, before any user process exists and before anything
// accesses device BARs (while a function's decoding is off, its BARs do not
// respond); host tests call it again after PciResetForTest(). CHECK-fails if
// a device is claimed. A function behind a bridge that firmware left in D1,
// D2 or D3hot is not found: such a bridge forwards no configuration requests
// to its secondary bus [PCIe Base 6.0 §5.3.1.4.1].
void PciInit(std::shared_ptr<PciConfigSpace> config);

int PciDeviceCount();
// The record of device `index`, or nullptr unless 0 <= index <
// PciDeviceCount(). The record stays valid for good, and unchanged but for
// the correction that PciRestoreAfterPowerUp() describes, which only the
// device's owner can cause; read the record of a device that the caller does
// not own with PciCopyDevice().
const PciDevice* PciGetDevice(int index);
// Copies the record of device `index` under the PCI lock and returns it, or
// returns an error unless 0 <= index < PciDeviceCount().
StatusOr<PciDevice> PciCopyDevice(int index);
// The index of the function at `address`, or -1 if there is none.
int PciFindDevice(PciAddress address);

// Writes one line per function with `write` (for example UartWrite()), in
// index order, then a summary line. For QEMU with a root MSI-X endpoint and an
// MSI-only endpoint behind a PCI-to-PCI bridge (SeaBIOS):
// [PCI] 00:04.0 1b36:000d class 0c.03.30 bar0=mem64:0xfea70000+0x4000 msix=16
// [PCI] 00:05.0 1b36:0001 class 06.04.00 bridge=01-01 bar0=mem64:0xfea75000
//       msi64 pvm  (one line)
// [PCI] 01:01.0 1033:0194 class 0c.03.30 bar0=mem64:0xfe800000+0x4000 msi64
// [PCI] 9 functions
// The line of a function is "[PCI] bb:dd.f vvvv:dddd class cc.ss.pp" (all
// lowercase hexadecimal) followed by these items, where they apply:
// "bridge=<secondary>-<subordinate>" (header type 1), "hdr=<layout>" (other
// unknown or CardBus layouts), "bar<i>=<io|mem32|mem64>[p]:0x<base>[+0x<size>]"
// for each BAR that is not kNone ("p": prefetchable; no size if unknown),
// "msi32" or "msi64" (then "pvm" with per-vector masking), and "msix=<table
// size>" or, if PciMsix::valid is false, "msix-ignored". The summary adds
// ", <n> not recorded (table full)" if functions did not fit in the table.
void PciWriteBootReport(void (*write)(const char* text));

// Configuration-space access to device `index`, each under the PCI lock.
// CHECK-fail unless `index` is valid and `offset` is aligned and inside the
// 256 bytes. A 32-bit write must not cover the Command register (it would also
// write the write-1-to-clear Status bits).
uint8_t PciConfigRead8(int index, int offset);
uint16_t PciConfigRead16(int index, int offset);
uint32_t PciConfigRead32(int index, int offset);
void PciConfigWrite8(int index, int offset, uint8_t value);
void PciConfigWrite16(int index, int offset, uint16_t value);
void PciConfigWrite32(int index, int offset, uint32_t value);

// Sets the bits `set` and clears the bits `clear` (disjoint, CHECKed) of the
// 16-bit register at `offset` of device `index` with one read-modify-write
// under the PCI lock (writing nothing if no bit changes), then reads the
// register back and returns it. The read is non-posted: it completes only after
// the write, and after the writes (DMA, MSIs) that the device posted before it.
// Only for registers without write-1-to-clear bits (such as Command, or MSI and
// MSI-X Message Control): the write-back would clear those that are set.
uint16_t PciConfigUpdate16(int index, int offset, uint16_t set, uint16_t clear);
// PciConfigUpdate16() of the Command register.
uint16_t PciUpdateCommand(int index, uint16_t set, uint16_t clear);

// Device ownership. An owner is an opaque non-zero token (e.g. the address of a
// driver's state) that pci.cpp never dereferences; each device has at most one.
inline constexpr uintptr_t kPciNoOwner = 0;

enum class PciStatus {
  kOk,
  // `index` is out of range.
  kNoDevice,
  // The function is not claimable (PciDevice::claimable).
  kNotClaimable,
  // Another owner owns the device.
  kBusy,
  // The caller does not own the device.
  kNotOwner,
};

// Makes `owner` (not kPciNoOwner, CHECKed) the owner of device `index`: kOk
// also if it already was (then nothing changes), kBusy if another owner is,
// kNoDevice or kNotClaimable. A new claim sets Memory Space and Bus Master on
// every bridge between the device and its root bus (as Linux's
// pci_enable_bridge() does: USB4/Thunderbolt bridges block even BAR read
// completions while firmware's pre-boot DMA protection leaves Bus Master off,
// and a bridge initiates no DMA of its own), then Memory Space and Interrupt
// Disable (no legacy INTx) on the device, clears MSI Enable and MSI-X Enable
// (as Linux's pci_msi_init() does: firmware may have left one on, and the
// owner's irq_alloc must find both off), and leaves the device's Bus Master as
// it was.
PciStatus PciClaim(int index, uintptr_t owner);
// If `owner` owns device `index`: PciQuiesce()s it, frees the claim and returns
// kOk. Otherwise changes nothing and returns kNotOwner or kNoDevice.
PciStatus PciRelease(int index, uintptr_t owner);
// PciRelease() of every device that `owner` owns; returns how many.
int PciReleaseAll(uintptr_t owner);
// The owner of device `index`, kPciNoOwner if it is unclaimed or out of range.
uintptr_t PciOwner(int index);
int PciClaimedCount();

// If `owner` owns device `index`: with `enable`, first sets Memory Space and
// Bus Master on every bridge between the device and its root bus, so that its
// DMA and MSIs reach memory, then sets Bus Master on the device; without, only
// clears the device's Bus Master. Bridges are shared, so nothing ever clears
// their bits. Returns kOk, kNotOwner or kNoDevice.
PciStatus PciSetBusMaster(int index, uintptr_t owner, bool enable);

// Device power states [PCI Bus Power Management 1.2 §5], as PMCSR encodes
// them. Firmware may leave a function that it doesn't use in D3hot, where the
// function still answers configuration accesses but decodes no memory or I/O
// accesses (its BARs read all ones). A bridge in D1, D2 or D3hot forwards no
// requests to its secondary bus at all [PCIe Base 6.0 §5.3.1.4.1].
enum class PciPowerState : uint8_t { kD0 = 0, kD1 = 1, kD2 = 2, kD3hot = 3 };

// The configuration of a function that PciSetPowerStateD0() saves before it
// takes the function out of a low-power state, for PciRestoreAfterPowerUp():
// leaving D3hot without No_Soft_Reset resets the function [PCI Bus Power
// Management 1.2 §5.4.1], and firmware's settings of these registers matter
// (e.g. a Max_Payload_Size that matches the upstream port). The BARs are in
// the device record.
struct PciPowerUpContext {
  struct Register {
    int offset = 0;
    int width = 0;  // bytes: 1, 2 or 4
    uint32_t value = 0;
  };
  static constexpr int kMaxRegisters = 16;

  // Whether PciSetPowerStateD0() saved anything.
  bool saved = false;
  uint16_t command = 0;
  // Cache Line Size and Latency Timer; the PCI Express Device Control, Link
  // Control, Slot Control, Root Control and Device Control 2 registers that
  // the function has; and a bridge's bus numbers, I/O, memory and
  // prefetchable windows, and Bridge Control.
  int register_count = 0;
  Register registers[kMaxRegisters];
};

struct PciPowerTransition {
  PciPowerState from = PciPowerState::kD0;
  PciPowerUpContext context{};
};

// If `owner` owns device `index`, or `index` is a bridge between such a device
// and its root bus (bridges must be in D0 before anything behind them can be
// reached; wake them first, from the root bus down): if the function has a
// Power Management capability whose power state is not D0, saves its
// configuration in `context` (when `save_context` is true), writes D0 (leaving
// PME_Status as it is) and sets `from` to the state that it left; otherwise
// writes nothing and sets `from` to kD0. A function that left a lower state
// may only be accessed after PciPowerUpDelayMicroseconds(from), and once
// PciFunctionResponds(); then it needs PciRestoreAfterPowerUp(). Returns the
// transition on success, or kNotOwner / kNoDevice.
StatusOr<PciPowerTransition, PciStatus> PciSetPowerStateD0(
    int index, uintptr_t owner, bool save_context = true);

// The recovery time that a function needs after it left `from` for D0 before
// it may be accessed [PCI Bus Power Management 1.2 §5.6.1]: 10 ms from D3hot,
// 200 us from D2, none from D1 or D0.
int64_t PciPowerUpDelayMicroseconds(PciPowerState from);

// Whether function `index` (CHECKed) completes configuration reads: whether its
// Command and Status registers read as something other than all ones. After
// its recovery time a function may still be initializing, and answers
// configuration requests with Configuration Request Retry Status meanwhile,
// which the Root Complex retries and finally completes as all ones [PCIe Base
// 6.0 §2.3.1].
bool PciFunctionResponds(int index);

// After PciSetPowerStateD0() and the recovery time, if `owner` may manage
// function `index` as for PciSetPowerStateD0(): writes back what a reset
// cleared, each register only if it differs: the BARs (from the record), a
// bridge's bus numbers and windows (these with the function's decoding off),
// and the other registers saved in `context` (if not null). Then, for the
// owned device, sets Memory Space and Bus Master on its upstream bridges and
// Memory Space and Interrupt Disable on the device, as PciClaim() did, and the
// other Command bits that `context` saved except Bus Master (which the driver
// sets when it is ready); for a bridge, sets the Command bits that `context`
// saved (nothing clears a bridge's bits).
// If PciInit() sized the owned device's BARs while it was in a low-power
// state, also sizes them again, once, now that it is in D0: a function that
// ignored the size probes there would have reported its base alignment as its
// size. If the BARs differ, corrects them and the MSI-X geometry in the record
// under the PCI lock and returns true; otherwise returns false. Returns
// kNotOwner or kNoDevice on error.
StatusOr<bool, PciStatus> PciRestoreAfterPowerUp(
    int index, uintptr_t owner, const PciPowerUpContext* context = nullptr);

// What PciApplyClaimQuirks() changed.
struct PciClaimQuirks {
  // The Intel xHCI port routing, and the masks of the ports that it routed to
  // the xHCI as read back: SuperSpeed (USB3_PSSEN) and USB 2.0 (XUSB2PR).
  bool intel_usb_ports_routed = false;
  uint32_t usb3_ports = 0;
  uint32_t usb2_ports = 0;
};

// If `owner` owns device `index`: applies the fixups that some devices need
// before their driver touches them and returns them; otherwise changes nothing
// and returns kNotOwner or kNoDevice. Call it after PciRestoreAfterPowerUp():
// a reset would undo them. The one fixup: the USB ports that the EHCI and xHCI
// controllers of Intel 7/8/9-series PCHs share stay routed to EHCI, for which
// there is no driver, until the OS switches them. So for an Intel xHCI (class
// 0C0330) while an Intel EHCI (class 0C0320) exists, as Linux's
// usb_enable_intel_xhci_ports() does, writes USB3PRM (0xDC) to USB3_PSSEN
// (0xD8), which turns on the SuperSpeed terminations of the switchable ports,
// and then XUSB2PRM (0xD4) to XUSB2PR (0xD0), which switches their USB 2.0
// lines to the xHCI; except on Sony VAIO T-series laptops (subsystem
// 104D:90A8), which can't switch them.
StatusOr<PciClaimQuirks, PciStatus> PciApplyClaimQuirks(int index,
                                                        uintptr_t owner);

// The registers that PciQuiesce() read back at its end.
struct PciQuiesceResult {
  uint16_t command = 0;
  // 0 if the function has no such capability.
  uint16_t msi_control = 0;
  uint16_t msix_control = 0;
};

// Stops device `index` (CHECKed) from doing DMA and from signaling interrupts,
// through its configuration space, under the PCI lock: sets Interrupt Disable
// and clears Bus Master (no DMA and no MSI can start after this write), sets
// the MSI-X Function Mask and clears MSI-X Enable, clears MSI Enable, and
// then reads these registers back. The reads complete only after the device's
// earlier posted writes. Leaves Memory Space as it is, so that the caller can
// still reach the device's BARs (e.g. to mask MSI-X table entries).
PciQuiesceResult PciQuiesce(int index);

// Whether the `size` bytes at `phys` (size > 0) lie inside `bar`, a memory BAR
// with a base and a size. Overflow-safe for any inputs.
bool PciBarContainsRange(const PciBar& bar, uint64_t phys, int64_t size);

#if __STDC_HOSTED__
// Host unit-test hook: forgets the devices, the claims and the PciConfigSpace.
void PciResetForTest();
#endif

}  // namespace protos

#endif  // PROTOS_PCI_H_
