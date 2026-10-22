#ifndef PROTOS_USER_XHCI_REGS_H_
#define PROTOS_USER_XHCI_REGS_H_

#include <cstdint>

#include "user/xhci_platform.h"

// xHCI register definitions (xHCI 1.2 chapter 5 and section 7) and the
// decoding of a controller's capability registers and extended capabilities.
// Freestanding and header-only; see user/xhci_platform.h for the rules.
//
// Access rules (xHCI 5.1): only aligned 32-bit accesses; 64-bit registers are
// written as two dwords, low dword first (Write64LoHi()); RsvdP bits are
// written back as read, RsvdZ bits as 0, and RW1C bits only as 1 to clear
// them.

namespace protos::usb {

// Status of the core's operations. Names never contain "FAIL" (the boot test
// harness treats that word as a failure marker).
enum class [[nodiscard]] XhciStatus : uint8_t {
  kOk = 0,
  // The controller object was not bound with Init().
  kNotBound,
  // Registers read 0xFFFFFFFF: memory decoding off, D3hot, or device gone.
  kNotDecoding,
  // The capability registers or the extended-capability list are invalid
  // (layout outside the BAR, zero ports, malformed list).
  kBadCapabilities,
  // The controller did not halt after Run/Stop was cleared, so it was not
  // reset (resetting a running controller is undefined).
  kHaltTimeout,
  // USBCMD.HCRST did not clear after a reset.
  kResetTimeout,
  // USBSTS.CNR did not clear after a reset.
  kNotReadyAfterReset,
  // USBSTS.HCH was 0 after a completed reset.
  kRunningAfterReset,
  // Start() was called before a successful TakeOwnershipAndReset().
  kBadState,
  // PAGESIZE bit 0 (4 KiB pages) is not supported by the controller.
  kUnsupportedPageSize,
  // Platform::DmaAlloc() returned nullptr or an invalid physical address.
  kNoMemory,
  // USBSTS.HCH did not clear after USBCMD.R/S was set.
  kStartTimeout,
};

inline const char* XhciStatusName(const XhciStatus status) {
  switch (status) {
    case XhciStatus::kOk:
      return "ok";
    case XhciStatus::kNotBound:
      return "controller not initialized";
    case XhciStatus::kNotDecoding:
      return "controller not responding";
    case XhciStatus::kBadCapabilities:
      return "invalid capability registers";
    case XhciStatus::kHaltTimeout:
      return "halt timeout";
    case XhciStatus::kResetTimeout:
      return "reset timeout";
    case XhciStatus::kNotReadyAfterReset:
      return "controller not ready after reset";
    case XhciStatus::kRunningAfterReset:
      return "controller running after reset";
    case XhciStatus::kBadState:
      return "invalid controller state";
    case XhciStatus::kUnsupportedPageSize:
      return "unsupported page size";
    case XhciStatus::kNoMemory:
      return "out of dma memory";
    case XhciStatus::kStartTimeout:
      return "start timeout";
  }
  return "unknown status";
}

// Capability registers (xHCI 5.3), byte offsets from the start of BAR 0.
// CAPLENGTH and HCIVERSION are read as one dword (some controllers only
// accept dword reads).
constexpr int64_t kCapLengthHciVersion =
    0x00;  // [7:0] CAPLENGTH, [31:16] HCIVERSION
constexpr int64_t kHcsParams1 = 0x04;
constexpr int64_t kHcsParams2 = 0x08;
constexpr int64_t kHcsParams3 = 0x0C;
constexpr int64_t kHccParams1 = 0x10;
constexpr int64_t kDbOff = 0x14;
constexpr int64_t kRtsOff = 0x18;
constexpr int64_t kHccParams2 = 0x1C;
// The capability registers above; CAPLENGTH can't be smaller.
constexpr int kMinCapLength = 0x20;

constexpr uint32_t kHcsParams2Spr = 1u << 26;  // Scratchpad Restore
constexpr uint32_t kHccParams1Ac64 = 1u << 0;  // 64-bit addressing
constexpr uint32_t kHccParams1Bnc = 1u << 1;   // BW Negotiation
constexpr uint32_t kHccParams1Csz = 1u << 2;   // 64-byte contexts
constexpr uint32_t kHccParams1Ppc = 1u << 3;   // Port Power Control
constexpr uint32_t kDbOffMask = ~0x3u;
constexpr uint32_t kRtsOffMask = ~0x1Fu;

constexpr int CapLengthOf(const uint32_t dword0) {
  return static_cast<int>(dword0 & 0xFF);
}
constexpr int HciVersionOf(const uint32_t dword0) {
  return static_cast<int>(dword0 >> 16);
}
constexpr int MaxSlotsOf(const uint32_t hcs_params1) {
  return static_cast<int>(hcs_params1 & 0xFF);
}
constexpr int MaxInterruptersOf(const uint32_t hcs_params1) {
  return static_cast<int>((hcs_params1 >> 8) & 0x7FF);
}
constexpr int MaxPortsOf(const uint32_t hcs_params1) {
  return static_cast<int>(hcs_params1 >> 24);
}
// Max Scratchpad Buffers: Hi in [25:21], Lo in [31:27].
constexpr int MaxScratchpadsOf(const uint32_t hcs_params2) {
  return static_cast<int>((((hcs_params2 >> 21) & 0x1F) << 5) |
                          ((hcs_params2 >> 27) & 0x1F));
}
// ERST Max: log2 of the maximum number of Event Ring Segment Table entries.
constexpr int ErstMaxOf(const uint32_t hcs_params2) {
  return static_cast<int>((hcs_params2 >> 4) & 0xF);
}
// xECP: dword offset of the first extended capability from BAR 0, 0 = none.
constexpr int XecpDwordsOf(const uint32_t hcc_params1) {
  return static_cast<int>(hcc_params1 >> 16);
}

// Operational registers (xHCI 5.4), byte offsets from the operational base
// (BAR 0 + CAPLENGTH).
constexpr int64_t kOpUsbCmd = 0x00;
constexpr int64_t kOpUsbSts = 0x04;
constexpr int64_t kOpPageSize = 0x08;
constexpr int64_t kOpDnCtrl = 0x14;
constexpr int64_t kOpCrcr = 0x18;    // 64-bit
constexpr int64_t kOpDcbaap = 0x30;  // 64-bit
constexpr int64_t kOpConfig = 0x38;
// Port register set n (1-based) is at kOpPortRegs + kPortRegStride * (n - 1).
constexpr int64_t kOpPortRegs = 0x400;
constexpr int64_t kPortRegStride = 0x10;
constexpr int64_t kPortSc = 0x0;
constexpr int64_t kPortPmsc = 0x4;
constexpr int64_t kPortLi = 0x8;
constexpr int64_t kPortHlpmc = 0xC;

// USBCMD. RsvdP: bits 4-6, 12 and 17-31.
constexpr uint32_t kUsbCmdRs = 1u << 0;      // Run/Stop
constexpr uint32_t kUsbCmdHcrst = 1u << 1;   // Host Controller Reset
constexpr uint32_t kUsbCmdInte = 1u << 2;    // Interrupter Enable
constexpr uint32_t kUsbCmdHsee = 1u << 3;    // Host System Error Enable
constexpr uint32_t kUsbCmdLhcrst = 1u << 7;  // Light Host Controller Reset
constexpr uint32_t kUsbCmdCss = 1u << 8;     // Controller Save State
constexpr uint32_t kUsbCmdCrs = 1u << 9;     // Controller Restore State
constexpr uint32_t kUsbCmdEwe = 1u << 10;    // Enable Wrap Event
// Cleared to halt a controller: Run/Stop and every interrupt/event enable
// (Linux: XHCI_CMD_RUN | XHCI_IRQS).
constexpr uint32_t kUsbCmdHaltMask =
    kUsbCmdRs | kUsbCmdInte | kUsbCmdHsee | kUsbCmdEwe;

// USBSTS. RsvdZ: bits 1, 5-7 and 13-31.
constexpr uint32_t kUsbStsHch = 1u << 0;   // HC Halted (RO)
constexpr uint32_t kUsbStsHse = 1u << 2;   // Host System Error (RW1C)
constexpr uint32_t kUsbStsEint = 1u << 3;  // Event Interrupt (RW1C)
constexpr uint32_t kUsbStsPcd = 1u << 4;   // Port Change Detect (RW1C)
constexpr uint32_t kUsbStsSss = 1u << 8;   // Save State Status (RO)
constexpr uint32_t kUsbStsRss = 1u << 9;   // Restore State Status (RO)
constexpr uint32_t kUsbStsSre = 1u << 10;  // Save/Restore Error (RW1C)
constexpr uint32_t kUsbStsCnr = 1u << 11;  // Controller Not Ready (RO)
constexpr uint32_t kUsbStsHce = 1u << 12;  // Host Controller Error (RO)
constexpr uint32_t kUsbStsRw1cMask =
    kUsbStsHse | kUsbStsEint | kUsbStsPcd | kUsbStsSre;

// PAGESIZE.
constexpr uint32_t kPageSize4K = 1u << 0;

// CRCR (64-bit).
constexpr uint32_t kCrcrRcs = 1u << 0;  // Ring Cycle State
constexpr uint32_t kCrcrCs = 1u << 1;   // Command Stop (RW1S)
constexpr uint32_t kCrcrCa = 1u << 2;   // Command Abort (RW1S)
constexpr uint32_t kCrcrCrr = 1u << 3;  // Command Ring Running (RO)

// CONFIG.
constexpr uint32_t kConfigMaxSlotsEnMask = 0xFF;
constexpr uint32_t kConfigU3e = 1u << 8;
constexpr uint32_t kConfigCie = 1u << 9;

// PORTSC (xHCI 5.4.8).
constexpr uint32_t kPortScCcs = 1u << 0;  // Current Connect Status (RO)
constexpr uint32_t kPortScPed = 1u << 1;  // Port Enabled (RW1C: 1 disables!)
constexpr uint32_t kPortScOca = 1u << 3;  // Over-current Active (RO)
constexpr uint32_t kPortScPr = 1u << 4;   // Port Reset (RW1S)
constexpr int kPortScPlsShift = 5;        // Port Link State [8:5] (RWS)
constexpr uint32_t kPortScPlsMask = 0xFu << kPortScPlsShift;
constexpr uint32_t kPortScPp = 1u << 9;  // Port Power (RWS)
constexpr int kPortScSpeedShift = 10;    // Port Speed [13:10] (RO)
constexpr uint32_t kPortScSpeedMask = 0xFu << kPortScSpeedShift;
constexpr uint32_t kPortScPicMask = 0x3u << 14;  // Port Indicator (RWS)
constexpr uint32_t kPortScLws = 1u << 16;        // Link Write Strobe (RW)
constexpr uint32_t kPortScCsc = 1u << 17;        // Connect Status Change
constexpr uint32_t kPortScPec = 1u << 18;        // Port Enabled Change
constexpr uint32_t kPortScWrc = 1u << 19;        // Warm Port Reset Change
constexpr uint32_t kPortScOcc = 1u << 20;        // Over-current Change
constexpr uint32_t kPortScPrc = 1u << 21;        // Port Reset Change
constexpr uint32_t kPortScPlc = 1u << 22;        // Port Link State Change
constexpr uint32_t kPortScCec = 1u << 23;        // Config Error Change
constexpr uint32_t kPortScCas = 1u << 24;        // Cold Attach Status (RO)
constexpr uint32_t kPortScWce = 1u << 25;        // Wake on Connect Enable
constexpr uint32_t kPortScWde = 1u << 26;        // Wake on Disconnect Enable
constexpr uint32_t kPortScWoe = 1u << 27;        // Wake on Over-current En.
constexpr uint32_t kPortScDr = 1u << 30;         // Device Removable (RO)
constexpr uint32_t kPortScWpr = 1u << 31;        // Warm Port Reset (RW1S)
// The change bits (RW1C).
constexpr uint32_t kPortScChangeMask = kPortScCsc | kPortScPec | kPortScWrc |
                                       kPortScOcc | kPortScPrc | kPortScPlc |
                                       kPortScCec;
// The RO and RWS bits that a PORTSC write carries over from the value read so
// that it changes nothing else (Linux xhci_port_state_to_neutral()): the
// write never contains PED, PR, LWS, WPR or a change bit unless intended.
constexpr uint32_t kPortScReadOnlyMask =
    kPortScCcs | kPortScOca | kPortScSpeedMask | kPortScDr;
constexpr uint32_t kPortScPreserveMask = kPortScPlsMask | kPortScPp |
                                         kPortScPicMask | kPortScWce |
                                         kPortScWde | kPortScWoe;
constexpr uint32_t kPortScNeutralMask =
    kPortScReadOnlyMask | kPortScPreserveMask;
// Port Speed values with the default Protocol Speed IDs (xHCI 7.2.1).
constexpr int kPortSpeedFull = 1;
constexpr int kPortSpeedLow = 2;
constexpr int kPortSpeedHigh = 3;
constexpr int kPortSpeedSuper = 4;
constexpr int kPortSpeedSuperPlus = 5;

constexpr int PortSpeedOf(const uint32_t portsc) {
  return static_cast<int>((portsc & kPortScSpeedMask) >> kPortScSpeedShift);
}

// Runtime registers (xHCI 5.5), byte offsets from the runtime base
// (BAR 0 + RTSOFF). Interrupter i's registers are at
// kRtInterrupters + kRtInterrupterStride * i.
constexpr int64_t kRtMfIndex = 0x00;
constexpr int64_t kRtInterrupters = 0x20;
constexpr int64_t kRtInterrupterStride = 0x20;
constexpr int64_t kIrIman = 0x00;
constexpr int64_t kIrImod = 0x04;
constexpr int64_t kIrErstSz = 0x08;
constexpr int64_t kIrErstBa = 0x10;        // 64-bit
constexpr int64_t kIrErdp = 0x18;          // 64-bit
constexpr uint32_t kImanIp = 1u << 0;      // Interrupt Pending (RW1C)
constexpr uint32_t kImanIe = 1u << 1;      // Interrupt Enable
constexpr uint32_t kImodDefault = 0x0FA0;  // 1 ms, the reset value
constexpr uint32_t kErstSzMask = 0xFFFFu;
constexpr uint32_t kErstBaRsvdPMask = 0x3Fu;
constexpr uint32_t kErdpDesiMask = 0x7;
constexpr uint32_t kErdpEhb = 1u << 3;  // Event Handler Busy (RW1C)

// Doorbell registers (xHCI 5.6): doorbell s at BAR 0 + DBOFF + 4 * s;
// doorbell 0 is the command ring's.
constexpr int64_t kDoorbellStride = 4;

// Extended capabilities (xHCI 7). The header dword holds the ID in [7:0] and
// the dword offset of the next capability in [15:8] (0 = last).
constexpr int kExtCapIdLegacySupport = 1;
constexpr int kExtCapIdSupportedProtocol = 2;
constexpr int kExtCapIdExtendedPowerManagement = 3;
constexpr int kExtCapIdIoVirtualization = 4;
constexpr int kExtCapIdMessageInterrupt = 5;
constexpr int kExtCapIdLocalMemory = 6;
constexpr int kExtCapIdUsbDebug = 10;
constexpr int kExtCapIdExtendedMessageInterrupt = 17;

constexpr int ExtCapIdOf(const uint32_t header) {
  return static_cast<int>(header & 0xFF);
}
constexpr int ExtCapNextOf(const uint32_t header) {
  return static_cast<int>((header >> 8) & 0xFF);
}

// USB Legacy Support capability (xHCI 7.1), byte offsets from the capability.
constexpr int64_t kLegSup = 0x0;     // USBLEGSUP
constexpr int64_t kLegCtlSts = 0x4;  // USBLEGCTLSTS
// USBLEGSUP. RsvdP: bits 17-23 and 25-31.
constexpr uint32_t kLegSupBiosOwned = 1u << 16;  // HC BIOS Owned Semaphore
constexpr uint32_t kLegSupOsOwned = 1u << 24;    // HC OS Owned Semaphore
// USBLEGCTLSTS. The SMI enables (RW):
constexpr uint32_t kLegCtlStsUsbSmiEnable = 1u << 0;
constexpr uint32_t kLegCtlStsSmiOnHostErrorEnable = 1u << 4;
constexpr uint32_t kLegCtlStsSmiOnOsOwnershipEnable = 1u << 13;
constexpr uint32_t kLegCtlStsSmiOnPciCommandEnable = 1u << 14;
constexpr uint32_t kLegCtlStsSmiOnBarEnable = 1u << 15;
constexpr uint32_t kLegCtlStsSmiEnableMask =
    kLegCtlStsUsbSmiEnable | kLegCtlStsSmiOnHostErrorEnable |
    kLegCtlStsSmiOnOsOwnershipEnable | kLegCtlStsSmiOnPciCommandEnable |
    kLegCtlStsSmiOnBarEnable;
// Shadows of USBSTS.EINT and USBSTS.HSE (RO).
constexpr uint32_t kLegCtlStsSmiOnEventInterrupt = 1u << 16;
constexpr uint32_t kLegCtlStsSmiOnHostError = 1u << 20;
// The SMI events (RW1C).
constexpr uint32_t kLegCtlStsSmiOnOsOwnershipChange = 1u << 29;
constexpr uint32_t kLegCtlStsSmiOnPciCommand = 1u << 30;
constexpr uint32_t kLegCtlStsSmiOnBar = 1u << 31;
constexpr uint32_t kLegCtlStsSmiEventMask = kLegCtlStsSmiOnOsOwnershipChange |
                                            kLegCtlStsSmiOnPciCommand |
                                            kLegCtlStsSmiOnBar;
// RsvdP: bits 1-3, 5-12 and 17-19 (Linux XHCI_LEGACY_DISABLE_SMI).
constexpr uint32_t kLegCtlStsRsvdPMask =
    (0x7u << 1) | (0xFFu << 5) | (0x7u << 17);

// The USBLEGCTLSTS value that disables every SMI and clears every SMI event,
// given the value read: RsvdP bits as read, enables 0, RW1C events 1, RO and
// RsvdZ bits 0.
constexpr uint32_t LegacyControlDisablingSmis(const uint32_t value) {
  return (value & kLegCtlStsRsvdPMask) | kLegCtlStsSmiEventMask;
}

// Supported Protocol capability (xHCI 7.2): dword 0 [23:16] minor and [31:24]
// major revision; dword 1 the name ("USB "); dword 2 [7:0] the first port
// (1-based), [15:8] the port count and [31:28] the PSI count; dword 3 [4:0]
// the Protocol Slot Type.
constexpr int64_t kProtocolPorts = 0x8;
constexpr int64_t kProtocolSlotType = 0xC;
constexpr int kProtocolCapabilitySize = 16;
constexpr int kLegacyCapabilitySize = 8;

// The bounds of the extended-capability walk (Linux XHCI_MAX_EXT_CAPS) and
// the number of Supported Protocol ranges recorded.
constexpr int kMaxExtendedCaps = 50;
constexpr int kMaxProtocolRanges = 8;

// One Supported Protocol capability: ports first_port .. first_port +
// port_count - 1 speak USB `major`.`minor` (major 2 or 3).
struct ProtocolRange {
  int major = 0;
  int minor = 0;  // BCD, e.g. 0x10 for USB 3.1
  int first_port = 0;
  int port_count = 0;
  int psi_count = 0;
  int slot_type = 0;
};

// The decoded capabilities of a controller. All zero = not decoded.
struct XhciCaps {
  int cap_length = 0;
  int hci_version = 0;  // BCD, e.g. 0x0110
  int max_slots = 0;
  int max_interrupters = 0;
  int max_ports = 0;
  int max_scratchpads = 0;
  int erst_max = 0;
  bool scratchpad_restore = false;
  bool ac64 = false;
  bool ppc = false;
  int context_size = 0;  // Bytes per device context entry: 32, or 64 (CSZ)
  uint32_t hcc_params1 = 0;
  uint32_t hcc_params2 = 0;
  // Byte offsets from BAR 0.
  int64_t op_base = 0;
  int64_t rt_base = 0;
  int64_t db_base = 0;
  int64_t xecp_offset = 0;    // First extended capability, 0 = none
  int64_t legacy_offset = 0;  // USB Legacy Support capability, 0 = none
  int protocol_count = 0;
  ProtocolRange protocols[kMaxProtocolRanges] = {};

  // Byte offset of operational register `reg`.
  int64_t Op(const int64_t reg) const { return op_base + reg; }
  // Byte offset of register `reg` of port `port` (1-based).
  int64_t PortReg(const int port, const int64_t reg) const {
    return op_base + kOpPortRegs + kPortRegStride * (port - 1) + reg;
  }
  // Byte offset of register `reg` of interrupter `interrupter`.
  int64_t InterrupterReg(const int interrupter, const int64_t reg) const {
    return rt_base + kRtInterrupters + kRtInterrupterStride * interrupter + reg;
  }
  // Byte offset of doorbell `slot` (0 = command ring).
  int64_t Doorbell(const int slot) const {
    return db_base + kDoorbellStride * slot;
  }
  // The USB major revision (2 or 3) of port `port` (1-based) per the
  // Supported Protocol capabilities, or 0 if no capability covers it.
  int PortProtocolMajor(const int port) const {
    if (port < 1 || port > max_ports) {
      return 0;
    }
    for (int i = 0; i < protocol_count; ++i) {
      const ProtocolRange& range = protocols[i];
      if (port >= range.first_port &&
          port < range.first_port + range.port_count) {
        return range.major;
      }
    }
    return 0;
  }
  // The Protocol Slot Type (0..31) of port `port` (1-based) per the
  // Supported Protocol capabilities, or 0 if no capability covers it.
  int PortSlotType(const int port) const {
    if (port < 1 || port > max_ports) {
      return 0;
    }
    for (int i = 0; i < protocol_count; ++i) {
      const ProtocolRange& range = protocols[i];
      if (port >= range.first_port &&
          port < range.first_port + range.port_count) {
        return range.slot_type;
      }
    }
    return 0;
  }
};

// Reads and checks the capability registers of the controller behind `mmio`
// into `*caps`. Every register region that the driver uses must lie inside
// the window and the operational registers (with all port register sets) must
// not overlap the runtime registers or the doorbells. Reads nothing beyond
// the capability registers and writes nothing. Returns kNotDecoding if a
// register reads 0xFFFFFFFF, kBadCapabilities if a check fails.
inline XhciStatus DecodeCaps(MmioWindow& mmio, XhciCaps* const caps) {
  *caps = XhciCaps{};
  const int64_t size = mmio.Size();
  if (size < kMinCapLength) {
    return XhciStatus::kBadCapabilities;
  }
  const uint32_t dword0 = mmio.Read32(kCapLengthHciVersion);
  const uint32_t hcs_params1 = mmio.Read32(kHcsParams1);
  const uint32_t hcs_params2 = mmio.Read32(kHcsParams2);
  const uint32_t hcc_params1 = mmio.Read32(kHccParams1);
  const uint32_t db_off = mmio.Read32(kDbOff);
  const uint32_t rts_off = mmio.Read32(kRtsOff);
  const uint32_t hcc_params2 = mmio.Read32(kHccParams2);
  if (dword0 == kMmioAllOnes || hcs_params1 == kMmioAllOnes ||
      hcs_params2 == kMmioAllOnes || hcc_params1 == kMmioAllOnes ||
      db_off == kMmioAllOnes || rts_off == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  XhciCaps decoded;
  decoded.cap_length = CapLengthOf(dword0);
  decoded.hci_version = HciVersionOf(dword0);
  decoded.max_slots = MaxSlotsOf(hcs_params1);
  decoded.max_interrupters = MaxInterruptersOf(hcs_params1);
  decoded.max_ports = MaxPortsOf(hcs_params1);
  decoded.max_scratchpads = MaxScratchpadsOf(hcs_params2);
  decoded.erst_max = ErstMaxOf(hcs_params2);
  decoded.scratchpad_restore = (hcs_params2 & kHcsParams2Spr) != 0;
  decoded.ac64 = (hcc_params1 & kHccParams1Ac64) != 0;
  decoded.ppc = (hcc_params1 & kHccParams1Ppc) != 0;
  decoded.context_size = (hcc_params1 & kHccParams1Csz) != 0 ? 64 : 32;
  decoded.hcc_params1 = hcc_params1;
  decoded.hcc_params2 = hcc_params2;
  decoded.op_base = decoded.cap_length;
  decoded.rt_base = rts_off & kRtsOffMask;
  decoded.db_base = db_off & kDbOffMask;
  decoded.xecp_offset = int64_t{XecpDwordsOf(hcc_params1)} * 4;
  if (decoded.cap_length < kMinCapLength || decoded.cap_length % 4 != 0 ||
      decoded.max_slots < 1 || decoded.max_interrupters < 1 ||
      decoded.max_ports < 1) {
    return XhciStatus::kBadCapabilities;
  }
  // The operational registers end with the last port register set.
  const int64_t op_end =
      decoded.op_base + kOpPortRegs + kPortRegStride * decoded.max_ports;
  // Only interrupter 0 is used.
  const int64_t rt_end =
      decoded.rt_base + kRtInterrupters + kRtInterrupterStride;
  const int64_t db_end =
      decoded.db_base + kDoorbellStride * (int64_t{decoded.max_slots} + 1);
  if (op_end > size || rt_end > size || db_end > size ||
      decoded.rt_base < op_end || decoded.db_base < op_end ||
      (decoded.rt_base < db_end && decoded.db_base < rt_end)) {
    return XhciStatus::kBadCapabilities;
  }
  *caps = decoded;
  return XhciStatus::kOk;
}

namespace regs_internal {

// Whether `[offset, offset + length)` is a valid 4-byte-aligned extended
// capability range: inside `[kMinCapLength, window_size]` and disjoint from
// the operational, runtime and doorbell register regions.
inline bool ValidExtCapRange(const XhciCaps& caps,       //
                             const int64_t window_size,  //
                             const int64_t offset,       //
                             const int64_t length) {
  if (offset < kMinCapLength || (offset & 3) != 0 || length < 4 ||
      offset > window_size - length) {
    return false;
  }
  const int64_t end = offset + length;
  const int64_t op_end =
      caps.op_base + kOpPortRegs + kPortRegStride * caps.max_ports;
  const int64_t rt_end = caps.rt_base + kRtInterrupters + kRtInterrupterStride;
  const int64_t db_end =
      caps.db_base + kDoorbellStride * (int64_t{caps.max_slots} + 1);
  if (offset < op_end && caps.op_base < end) {
    return false;
  }
  if (offset < rt_end && caps.rt_base < end) {
    return false;
  }
  if (offset < db_end && caps.db_base < end) {
    return false;
  }
  return true;
}

}  // namespace regs_internal

// Walks the extended-capability list of a controller whose capabilities
// DecodeCaps() decoded into `*caps`, and records the USB Legacy Support
// capability (the first one) and the Supported Protocol capabilities of USB 2
// and USB 3 (ranges clamped to the controller's ports; unusable ones are
// skipped). Returns kNotDecoding if a header reads 0xFFFFFFFF, and
// kBadCapabilities if the list leaves the window, overlaps the capability /
// operational / runtime / doorbell registers, or has more than
// kMaxExtendedCaps entries. Writes nothing.
inline XhciStatus ParseExtendedCaps(MmioWindow& mmio, XhciCaps* const caps) {
  caps->legacy_offset = 0;
  caps->protocol_count = 0;
  for (ProtocolRange& range : caps->protocols) {
    range = ProtocolRange{};
  }
  int64_t offset = caps->xecp_offset;
  if (offset == 0) {
    return XhciStatus::kOk;
  }
  const int64_t size = mmio.Size();
  for (int count = 0; count < kMaxExtendedCaps; ++count) {
    if (!regs_internal::ValidExtCapRange(*caps, size, offset, 4)) {
      return XhciStatus::kBadCapabilities;
    }
    const uint32_t header = mmio.Read32(offset);
    if (header == kMmioAllOnes) {
      return XhciStatus::kNotDecoding;
    }
    const int id = ExtCapIdOf(header);
    if (id == kExtCapIdLegacySupport && caps->legacy_offset == 0) {
      if (!regs_internal::ValidExtCapRange(*caps, size, offset,
                                           kLegacyCapabilitySize)) {
        return XhciStatus::kBadCapabilities;
      }
      caps->legacy_offset = offset;
    } else if (id == kExtCapIdSupportedProtocol) {
      if (!regs_internal::ValidExtCapRange(*caps, size, offset,
                                           kProtocolCapabilitySize)) {
        return XhciStatus::kBadCapabilities;
      }
      const uint32_t ports = mmio.Read32(offset + kProtocolPorts);
      const uint32_t slot_type = mmio.Read32(offset + kProtocolSlotType);
      if (ports == kMmioAllOnes || slot_type == kMmioAllOnes) {
        return XhciStatus::kNotDecoding;
      }
      ProtocolRange range;
      range.major = static_cast<int>(header >> 24);
      range.minor = static_cast<int>((header >> 16) & 0xFF);
      range.first_port = static_cast<int>(ports & 0xFF);
      range.port_count = static_cast<int>((ports >> 8) & 0xFF);
      range.psi_count = static_cast<int>(ports >> 28);
      range.slot_type = static_cast<int>(slot_type & 0x1F);
      if (range.first_port + range.port_count - 1 > caps->max_ports) {
        range.port_count = caps->max_ports - range.first_port + 1;
      }
      if ((range.major == 2 || range.major == 3) && range.first_port >= 1 &&
          range.port_count >= 1 && caps->protocol_count < kMaxProtocolRanges) {
        caps->protocols[caps->protocol_count++] = range;
      }
    }
    const int next = ExtCapNextOf(header);
    if (next == 0) {
      return XhciStatus::kOk;
    }
    offset += int64_t{next} * 4;
  }
  return XhciStatus::kBadCapabilities;
}

// Writes a 64-bit register as two dwords, the low one first. Both halves are
// always written (some controllers latch the value on the high write).
inline void Write64LoHi(MmioWindow& mmio,      //
                        const int64_t offset,  //
                        const uint64_t value) {
  mmio.Write32(offset, static_cast<uint32_t>(value));
  mmio.Write32(offset + 4, static_cast<uint32_t>(value >> 32));
}

}  // namespace protos::usb

#endif  // PROTOS_USER_XHCI_REGS_H_
