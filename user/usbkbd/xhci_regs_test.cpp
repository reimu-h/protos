#include "user/usbkbd/xhci_regs.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <ostream>
#include <random>
#include <vector>

#include "user/usbkbd/xhci_sim.h"

namespace protos::usb {

// Prints statuses by name in test failure messages.
static void PrintTo(const XhciStatus status, std::ostream* const os) {
  *os << XhciStatusName(status);
}

namespace {

namespace t = ::testing;

using xhci_sim::Access;
using xhci_sim::ExtCap;
using xhci_sim::IntelLikeConfig;
using xhci_sim::LegacyCap;
using xhci_sim::ProtocolCap;
using xhci_sim::SimClock;
using xhci_sim::VendorCap;
using xhci_sim::Violation;
using xhci_sim::XhciSim;
using xhci_sim::XhciSimConfig;

// The register bits and masks, checked against the values transcribed from
// the xHCI 1.2 specification (and Linux) by hand.
TEST(XhciRegsTest, ConstantsMatchTheSpecification) {
  EXPECT_THAT(kUsbCmdRs, t::Eq(0x1u));
  EXPECT_THAT(kUsbCmdHcrst, t::Eq(0x2u));
  EXPECT_THAT(kUsbCmdInte, t::Eq(0x4u));
  EXPECT_THAT(kUsbCmdHsee, t::Eq(0x8u));
  EXPECT_THAT(kUsbCmdEwe, t::Eq(0x400u));
  // Linux xhci_halt(): ~(CMD_RUN | XHCI_IRQS) with XHCI_IRQS = EIE | HSEIE.
  EXPECT_THAT(kUsbCmdHaltMask, t::Eq(0x40Du));
  EXPECT_THAT(kUsbStsHch, t::Eq(0x1u));
  EXPECT_THAT(kUsbStsCnr, t::Eq(0x800u));
  EXPECT_THAT(kUsbStsHce, t::Eq(0x1000u));
  EXPECT_THAT(kUsbStsRw1cMask, t::Eq(0x41Cu));
  EXPECT_THAT(kOpPortRegs, t::Eq(0x400));
  EXPECT_THAT(kOpCrcr, t::Eq(0x18));
  EXPECT_THAT(kOpDcbaap, t::Eq(0x30));
  EXPECT_THAT(kOpConfig, t::Eq(0x38));
  EXPECT_THAT(kLegSupBiosOwned, t::Eq(0x00010000u));
  EXPECT_THAT(kLegSupOsOwned, t::Eq(0x01000000u));
  EXPECT_THAT(kLegCtlStsSmiEnableMask, t::Eq(0x0000E011u));
  // Linux XHCI_LEGACY_DISABLE_SMI and XHCI_LEGACY_SMI_EVENTS.
  EXPECT_THAT(kLegCtlStsRsvdPMask, t::Eq(0x000E1FEEu));
  EXPECT_THAT(kLegCtlStsSmiEventMask, t::Eq(0xE0000000u));
  // Linux xhci_port_state_to_neutral(): RO | RWS bits.
  EXPECT_THAT(kPortScNeutralMask, t::Eq(0x4E00FFE9u));
  EXPECT_THAT(kPortScChangeMask, t::Eq(0x00FE0000u));
  EXPECT_THAT(kPortScNeutralMask & (kPortScPed | kPortScPr | kPortScLws |
                                    kPortScWpr | kPortScChangeMask),
              t::Eq(0u));
  EXPECT_THAT(kImanIp | kImanIe, t::Eq(0x3u));
  EXPECT_THAT(kErdpEhb, t::Eq(0x8u));
}

TEST(XhciRegsTest, FieldExtraction) {
  EXPECT_THAT(CapLengthOf(0x01100080u), t::Eq(0x80));
  EXPECT_THAT(HciVersionOf(0x01100080u), t::Eq(0x0110));
  EXPECT_THAT(MaxSlotsOf(0x08001040u), t::Eq(64));
  EXPECT_THAT(MaxInterruptersOf(0x08001040u), t::Eq(16));
  EXPECT_THAT(MaxPortsOf(0x08001040u), t::Eq(8));
  EXPECT_THAT(MaxInterruptersOf(0x0007FF00u), t::Eq(0x7FF));
  // Max Scratchpad Buffers Hi = 1 (bits 25:21), Lo = 3 (bits 31:27).
  EXPECT_THAT(MaxScratchpadsOf((3u << 27) | (1u << 21)), t::Eq(35));
  EXPECT_THAT(MaxScratchpadsOf(0xFFFFFFFFu), t::Eq(1023));
  EXPECT_THAT(ErstMaxOf(0x000000F0u), t::Eq(15));
  EXPECT_THAT(XecpDwordsOf(0x00087001u), t::Eq(8));
  EXPECT_THAT(ExtCapIdOf(0x03100C02u), t::Eq(2));
  EXPECT_THAT(ExtCapNextOf(0x03100C02u), t::Eq(12));
}

TEST(XhciRegsTest, DisablingSmisClearsEnablesAcksEventsAndKeepsRsvdP) {
  EXPECT_THAT(LegacyControlDisablingSmis(0xE004E095u), t::Eq(0xE0040084u));
  EXPECT_THAT(LegacyControlDisablingSmis(0xFFFFFFFFu), t::Eq(0xE00E1FEEu));
  EXPECT_THAT(LegacyControlDisablingSmis(0u), t::Eq(0xE0000000u));
  // RO bits 16 and 20 and RsvdZ bits 21-28 are written as 0.
  EXPECT_THAT(LegacyControlDisablingSmis(0x1FF10000u), t::Eq(0xE0000000u));
}

class XhciRegsSimTest : public t::Test {
 protected:
  XhciSim& Create(const XhciSimConfig& config) {
    sim_ = std::make_unique<XhciSim>(&clock_, config);
    return *sim_;
  }

  // The driver may only read; checks that nothing was written and nothing
  // outside the window was touched.
  void ExpectReadOnlyAccesses() {
    EXPECT_THAT(sim_->Writes(), t::IsEmpty());
    for (const Access& access : sim_->accesses()) {
      EXPECT_THAT(access.offset % 4, t::Eq(0));
      EXPECT_THAT(access.offset, t::Lt(sim_->Size()));
    }
  }

  SimClock clock_;
  std::unique_ptr<XhciSim> sim_;
};

TEST_F(XhciRegsSimTest, DecodesQemuCapabilities) {
  XhciSim& sim = Create(XhciSimConfig{});
  const StatusOr<XhciCaps, XhciStatus> decoded = DecodeCaps(sim);
  ASSERT_THAT(decoded.status(), t::Eq(XhciStatus::kOk));
  const XhciCaps& caps = *decoded;
  EXPECT_THAT(caps.cap_length, t::Eq(0x40));
  EXPECT_THAT(caps.hci_version, t::Eq(0x0100));
  EXPECT_THAT(caps.max_slots, t::Eq(64));
  EXPECT_THAT(caps.max_interrupters, t::Eq(16));
  EXPECT_THAT(caps.max_ports, t::Eq(8));
  EXPECT_THAT(caps.max_scratchpads, t::Eq(0));
  EXPECT_THAT(caps.erst_max, t::Eq(0));
  EXPECT_FALSE(caps.scratchpad_restore);
  EXPECT_TRUE(caps.ac64);
  EXPECT_FALSE(caps.ppc);
  EXPECT_THAT(caps.context_size, t::Eq(32));
  EXPECT_THAT(caps.op_base, t::Eq(0x40));
  EXPECT_THAT(caps.rt_base, t::Eq(0x1000));
  EXPECT_THAT(caps.db_base, t::Eq(0x2000));
  EXPECT_THAT(caps.xecp_offset, t::Eq(0x20));
  EXPECT_THAT(caps.Op(kOpUsbSts), t::Eq(0x44));
  EXPECT_THAT(caps.PortReg(1, kPortSc), t::Eq(0x440));
  EXPECT_THAT(caps.PortReg(5, kPortSc), t::Eq(0x480));
  EXPECT_THAT(caps.InterrupterReg(0, kIrIman), t::Eq(0x1020));
  EXPECT_THAT(caps.InterrupterReg(1, kIrErdp), t::Eq(0x1058));
  EXPECT_THAT(caps.Doorbell(0), t::Eq(0x2000));
  EXPECT_THAT(caps.Doorbell(3), t::Eq(0x200C));
  // Only the capability registers were read.
  for (const Access& access : sim.accesses()) {
    EXPECT_THAT(access.offset, t::Lt(0x20));
  }
  ExpectReadOnlyAccesses();
}

TEST_F(XhciRegsSimTest, DecodesARealHardwareProfileAndMasksOffsetJunk) {
  XhciSimConfig config = IntelLikeConfig();
  config.db_off_raw = 0x3003;   // bits 1:0 reserved
  config.rts_off_raw = 0x201F;  // bits 4:0 reserved
  XhciSim& sim = Create(config);
  const StatusOr<XhciCaps, XhciStatus> decoded = DecodeCaps(sim);
  ASSERT_THAT(decoded.status(), t::Eq(XhciStatus::kOk));
  const XhciCaps& caps = *decoded;
  EXPECT_THAT(caps.cap_length, t::Eq(0x80));
  EXPECT_THAT(caps.hci_version, t::Eq(0x0110));
  EXPECT_THAT(caps.max_ports, t::Eq(16));
  EXPECT_THAT(caps.max_interrupters, t::Eq(8));
  EXPECT_THAT(caps.max_scratchpads, t::Eq(35));
  EXPECT_THAT(caps.erst_max, t::Eq(15));
  EXPECT_TRUE(caps.scratchpad_restore);
  EXPECT_TRUE(caps.ppc);
  EXPECT_THAT(caps.context_size, t::Eq(64));
  EXPECT_THAT(caps.db_base, t::Eq(0x3000));
  EXPECT_THAT(caps.rt_base, t::Eq(0x2000));
  EXPECT_THAT(caps.xecp_offset, t::Eq(0x8000));
  EXPECT_THAT(caps.hcc_params2, t::Eq(0x3Fu));
  ExpectReadOnlyAccesses();
}

TEST_F(XhciRegsSimTest, RejectsInvalidCapabilityLength) {
  for (const int cap_length : {0x10, 0x1C, 0x42}) {
    XhciSimConfig config;
    config.cap_length = cap_length;
    config.ext_caps.clear();
    XhciSim& sim = Create(config);
    EXPECT_THAT(DecodeCaps(sim).status(), t::Eq(XhciStatus::kBadCapabilities))
        << "CAPLENGTH 0x" << std::hex << cap_length;
    ExpectReadOnlyAccesses();
  }
}

TEST_F(XhciRegsSimTest, RejectsZeroSlotsPortsOrInterrupters) {
  for (int which = 0; which < 3; ++which) {
    XhciSimConfig config;
    config.max_slots = which == 0 ? 0 : 64;
    config.max_ports = which == 1 ? 0 : 8;
    config.max_interrupters = which == 2 ? 0 : 16;
    XhciSim& sim = Create(config);
    EXPECT_THAT(DecodeCaps(sim).status(), t::Eq(XhciStatus::kBadCapabilities))
        << which;
    ExpectReadOnlyAccesses();
  }
}

TEST_F(XhciRegsSimTest, RejectsRegisterRegionsOutsideTheBarOrOverlapping) {
  struct Layout {
    const char* name;
    int64_t bar_size;
    int max_ports;
    uint32_t db_off;
    uint32_t rts_off;
  };
  const Layout layouts[] = {
      // Ports end at 0x40 + 0x400 + 255 * 0x10 = 0x1430.
      {"ports past the BAR", 0x1400, 255, 0x0800, 0x0C00},
      // 65 doorbells end at 0x4004.
      {"doorbells past the BAR", 0x4000, 8, 0x3F00, 0x1000},
      {"doorbell offset beyond the BAR", 0x4000, 8, 0x10000, 0x1000},
      // Interrupter 0 ends at 0x4020.
      {"runtime past the BAR", 0x4000, 8, 0x2000, 0x3FE0},
      {"runtime inside the port registers", 0x4000, 8, 0x2000, 0x460},
      {"doorbells inside the operational registers", 0x4000, 8, 0x100, 0x1000},
      {"runtime overlaps the doorbells", 0x4000, 8, 0x2000, 0x2000},
  };
  for (const Layout& layout : layouts) {
    XhciSimConfig config;
    config.bar_size = layout.bar_size;
    config.max_ports = layout.max_ports;
    config.db_off_raw = layout.db_off;
    config.rts_off_raw = layout.rts_off;
    XhciSim& sim = Create(config);
    EXPECT_THAT(DecodeCaps(sim).status(), t::Eq(XhciStatus::kBadCapabilities))
        << layout.name;
    ExpectReadOnlyAccesses();
  }
}

TEST_F(XhciRegsSimTest, WindowTooSmallForTheCapabilitiesIsNotRead) {
  XhciSimConfig config;
  config.bar_size = 0x1C;
  config.ext_caps.clear();
  XhciSim& sim = Create(config);
  EXPECT_THAT(DecodeCaps(sim).status(), t::Eq(XhciStatus::kBadCapabilities));
  EXPECT_THAT(sim.accesses(), t::IsEmpty());
}

TEST_F(XhciRegsSimTest, ControllerThatDoesNotDecodeIsReportedAsSuch) {
  XhciSimConfig config;
  config.all_ones = true;
  XhciSim& sim = Create(config);
  EXPECT_THAT(DecodeCaps(sim).status(), t::Eq(XhciStatus::kNotDecoding));
  ExpectReadOnlyAccesses();
}

TEST_F(XhciRegsSimTest, ExtendedCapabilitiesOfQemu) {
  XhciSim& sim = Create(XhciSimConfig{});
  const StatusOr<XhciCaps, XhciStatus> decoded = DecodeCaps(sim);
  ASSERT_THAT(decoded.status(), t::Eq(XhciStatus::kOk));
  XhciCaps caps = *decoded;
  ASSERT_THAT(ParseExtendedCaps(sim, &caps), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(caps.legacy_offset, t::Eq(0));
  ASSERT_THAT(caps.protocol_count, t::Eq(2));
  EXPECT_THAT(caps.protocols[0].major, t::Eq(2));
  EXPECT_THAT(caps.protocols[0].first_port, t::Eq(5));
  EXPECT_THAT(caps.protocols[0].port_count, t::Eq(4));
  EXPECT_THAT(caps.protocols[1].major, t::Eq(3));
  EXPECT_THAT(caps.protocols[1].first_port, t::Eq(1));
  EXPECT_THAT(caps.protocols[1].port_count, t::Eq(4));
  EXPECT_THAT(caps.PortProtocolMajor(1), t::Eq(3));
  EXPECT_THAT(caps.PortProtocolMajor(4), t::Eq(3));
  EXPECT_THAT(caps.PortProtocolMajor(5), t::Eq(2));
  EXPECT_THAT(caps.PortProtocolMajor(8), t::Eq(2));
  EXPECT_THAT(caps.PortProtocolMajor(0), t::Eq(0));
  EXPECT_THAT(caps.PortProtocolMajor(9), t::Eq(0));
  ExpectReadOnlyAccesses();
}

TEST_F(XhciRegsSimTest, FindsTheLegacyCapabilityBetweenOtherCapabilities) {
  XhciSimConfig config = IntelLikeConfig();
  config.ext_caps = {
      VendorCap(0x8000, 0xC0, 2),          VendorCap(0x8010, 10, 4),
      ProtocolCap(0x8040, 2, 0, 1, 10),    LegacyCap(0x8100),
      ProtocolCap(0x8200, 3, 0x20, 11, 6), VendorCap(0x8300, 0xFF, 1)};
  XhciSim& sim = Create(config);
  const StatusOr<XhciCaps, XhciStatus> decoded = DecodeCaps(sim);
  ASSERT_THAT(decoded.status(), t::Eq(XhciStatus::kOk));
  XhciCaps caps = *decoded;
  ASSERT_THAT(ParseExtendedCaps(sim, &caps), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(caps.legacy_offset, t::Eq(0x8100));
  ASSERT_THAT(caps.protocol_count, t::Eq(2));
  EXPECT_THAT(caps.protocols[1].major, t::Eq(3));
  EXPECT_THAT(caps.protocols[1].minor, t::Eq(0x20));
  EXPECT_THAT(caps.PortProtocolMajor(10), t::Eq(2));
  EXPECT_THAT(caps.PortProtocolMajor(11), t::Eq(3));
  EXPECT_THAT(caps.PortProtocolMajor(16), t::Eq(3));
  ExpectReadOnlyAccesses();
}

TEST_F(XhciRegsSimTest, NoExtendedCapabilities) {
  XhciSimConfig config;
  config.ext_caps.clear();
  XhciSim& sim = Create(config);
  const StatusOr<XhciCaps, XhciStatus> decoded = DecodeCaps(sim);
  ASSERT_THAT(decoded.status(), t::Eq(XhciStatus::kOk));
  XhciCaps caps = *decoded;
  EXPECT_THAT(caps.xecp_offset, t::Eq(0));
  const int reads = static_cast<int>(sim.accesses().size());
  ASSERT_THAT(ParseExtendedCaps(sim, &caps), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(static_cast<int>(sim.accesses().size()), t::Eq(reads));
  EXPECT_THAT(caps.legacy_offset, t::Eq(0));
  EXPECT_THAT(caps.protocol_count, t::Eq(0));
  EXPECT_THAT(caps.PortProtocolMajor(1), t::Eq(0));
}

TEST_F(XhciRegsSimTest, MalformedListsAreRejectedWithoutLeavingTheWindow) {
  struct Case {
    const char* name;
    std::vector<ExtCap> ext_caps;
    int xecp_override;
  };
  std::vector<Case> cases;
  // xECP = 0x1000 dwords: byte 0x4000, the end of the BAR.
  cases.push_back({"xECP beyond the BAR", {}, 0x1000});
  {
    // The next pointer of a capability near the end leads beyond the BAR.
    std::vector<ExtCap> ext_caps = {VendorCap(0x3E00, 0xC0, 1)};
    ext_caps[0].next_override = 0xFF;  // 0x3E00 + 0x3FC = 0x41FC
    cases.push_back({"next pointer leaves the BAR", ext_caps, -1});
  }
  // The legacy capability has 2 dwords, a protocol capability 4.
  cases.push_back(
      {"legacy capability straddles the BAR end", {LegacyCap(0x3FFC)}, -1});
  cases.push_back({"protocol capability straddles the BAR end",
                   {ProtocolCap(0x3FF8, 3, 0, 1, 4)},
                   -1});
  // Capabilities pointing into the capability, operational, runtime or
  // doorbell registers.
  cases.push_back({"xECP inside capability registers", {}, 1});  // offset 0x04
  cases.push_back(
      {"xECP at operational base (USBCMD)", {}, 0x10});  // offset 0x40
  cases.push_back(
      {"xECP inside port registers", {}, 0x480 / 4});  // offset 0x480
  cases.push_back(
      {"xECP inside runtime registers", {}, 0x1020 / 4});  // offset 0x1020
  cases.push_back(
      {"xECP inside doorbell registers", {}, 0x2000 / 4});  // offset 0x2000
  cases.push_back(
      {"legacy capability straddles operational base", {LegacyCap(0x3C)}, -1});
  for (const Case& test_case : cases) {
    XhciSimConfig config;
    config.ext_caps = test_case.ext_caps;
    config.xecp_override = test_case.xecp_override;
    XhciSim& sim = Create(config);
    const StatusOr<XhciCaps, XhciStatus> decoded = DecodeCaps(sim);
    ASSERT_THAT(decoded.status(), t::Eq(XhciStatus::kOk)) << test_case.name;
    XhciCaps caps = *decoded;
    EXPECT_THAT(ParseExtendedCaps(sim, &caps),
                t::Eq(XhciStatus::kBadCapabilities))
        << test_case.name;
    EXPECT_THAT(caps.legacy_offset, t::Eq(0)) << test_case.name;
    // Out-of-window accesses would also fail the test when `sim` is
    // destroyed.
    EXPECT_FALSE(sim.HasViolation(Violation::kOutOfRangeAccess))
        << test_case.name;
    ExpectReadOnlyAccesses();
  }
}

TEST_F(XhciRegsSimTest, ExtendedCapabilityWalkIsBounded) {
  // 60 capabilities one dword apart, the legacy capability after them.
  XhciSimConfig config = IntelLikeConfig();
  config.ext_caps.clear();
  for (int i = 0; i < 60; ++i) {
    config.ext_caps.push_back(VendorCap(0x8000 + 4 * i, 0xC0, 1));
  }
  config.ext_caps.push_back(LegacyCap(0x8000 + 4 * 60));
  XhciSim& sim = Create(config);
  const StatusOr<XhciCaps, XhciStatus> decoded = DecodeCaps(sim);
  ASSERT_THAT(decoded.status(), t::Eq(XhciStatus::kOk));
  XhciCaps caps = *decoded;
  const int before = static_cast<int>(sim.accesses().size());
  EXPECT_THAT(ParseExtendedCaps(sim, &caps),
              t::Eq(XhciStatus::kBadCapabilities));
  EXPECT_THAT(static_cast<int>(sim.accesses().size()) - before,
              t::Eq(kMaxExtendedCaps));
  EXPECT_THAT(caps.legacy_offset, t::Eq(0));
  ExpectReadOnlyAccesses();
}

TEST_F(XhciRegsSimTest, AllOnesCapabilityHeaderMeansNotDecoding) {
  XhciSimConfig config;
  config.ext_caps = {ProtocolCap(0x20, 2, 0, 5, 4), VendorCap(0x30, 0xFF, 1)};
  config.ext_caps[1].dwords[0] = 0xFFFF0000u;  // id/next filled in: 0xFFFF00FF
  config.ext_caps[1].next_override = 0xFF;     // header 0xFFFFFFFF
  XhciSim& sim = Create(config);
  const StatusOr<XhciCaps, XhciStatus> decoded = DecodeCaps(sim);
  ASSERT_THAT(decoded.status(), t::Eq(XhciStatus::kOk));
  XhciCaps caps = *decoded;
  EXPECT_THAT(sim.Peek(0x30), t::Eq(0xFFFFFFFFu));
  EXPECT_THAT(ParseExtendedCaps(sim, &caps), t::Eq(XhciStatus::kNotDecoding));
  ExpectReadOnlyAccesses();
}

TEST_F(XhciRegsSimTest, SupportedProtocolRangesAreClampedToThePorts) {
  XhciSimConfig config;
  config.ext_caps = {
      ProtocolCap(0x20, 2, 0, 7, 10),  // ports 7-16 of 8: clamped to 7-8
      ProtocolCap(0x30, 3, 0, 0, 4),   // first port 0: invalid
      ProtocolCap(0x40, 3, 0, 9, 2),   // beyond the last port
      ProtocolCap(0x50, 1, 0, 1, 2),   // USB 1: not a USB 2/3 range
      ProtocolCap(0x60, 3, 0, 1, 0),   // no ports
      ProtocolCap(0x70, 3, 0, 1, 6),
  };
  config.cap_length = 0x80;
  XhciSim& sim = Create(config);
  const StatusOr<XhciCaps, XhciStatus> decoded = DecodeCaps(sim);
  ASSERT_THAT(decoded.status(), t::Eq(XhciStatus::kOk));
  XhciCaps caps = *decoded;
  ASSERT_THAT(ParseExtendedCaps(sim, &caps), t::Eq(XhciStatus::kOk));
  ASSERT_THAT(caps.protocol_count, t::Eq(2));
  EXPECT_THAT(caps.protocols[0].first_port, t::Eq(7));
  EXPECT_THAT(caps.protocols[0].port_count, t::Eq(2));
  EXPECT_THAT(caps.protocols[1].major, t::Eq(3));
  EXPECT_THAT(caps.protocols[1].port_count, t::Eq(6));
  // Overlapping ranges: the first capability wins.
  EXPECT_THAT(caps.PortProtocolMajor(7), t::Eq(2));
  EXPECT_THAT(caps.PortProtocolMajor(6), t::Eq(3));
  EXPECT_THAT(caps.PortProtocolMajor(9), t::Eq(0));
  ExpectReadOnlyAccesses();
}

TEST_F(XhciRegsSimTest, AtMostEightProtocolRangesAreRecorded) {
  XhciSimConfig config;
  config.cap_length = 0xC0;
  config.ext_caps.clear();
  for (int i = 0; i < 10; ++i) {
    config.ext_caps.push_back(ProtocolCap(0x20 + 0x10 * i, 2, 0, i + 1, 1));
  }
  config.max_ports = 10;
  XhciSim& sim = Create(config);
  const StatusOr<XhciCaps, XhciStatus> decoded = DecodeCaps(sim);
  ASSERT_THAT(decoded.status(), t::Eq(XhciStatus::kOk));
  XhciCaps caps = *decoded;
  ASSERT_THAT(ParseExtendedCaps(sim, &caps), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(caps.protocol_count, t::Eq(kMaxProtocolRanges));
  EXPECT_THAT(caps.PortProtocolMajor(8), t::Eq(2));
  EXPECT_THAT(caps.PortProtocolMajor(9), t::Eq(0));
}

TEST_F(XhciRegsSimTest, RandomListsAlwaysTerminateInsideTheWindow) {
  std::mt19937 random(12345);
  for (int round = 0; round < 300; ++round) {
    XhciSimConfig config;
    config.bar_size = 0x4000;
    config.ext_caps.clear();
    const int count = 1 + static_cast<int>(random() % 8);
    for (int i = 0; i < count; ++i) {
      ExtCap cap;
      cap.offset = 0x20 + 0x10 * i;
      cap.id = static_cast<int>(random() % 4);
      cap.dwords = {
          static_cast<uint32_t>(random()), static_cast<uint32_t>(random()),
          static_cast<uint32_t>(random()), static_cast<uint32_t>(random())};
      cap.next_override = static_cast<int>(random() % 256);
      config.ext_caps.push_back(cap);
    }
    config.cap_length = 0x20 + 0x10 * count;  // The capabilities fit in
    config.xecp_override = static_cast<int>(random() % 0x1100);
    XhciSim& sim = Create(config);
    const StatusOr<XhciCaps, XhciStatus> decoded = DecodeCaps(sim);
    if (!decoded.ok()) {
      continue;
    }
    XhciCaps caps = *decoded;
    const XhciStatus status = ParseExtendedCaps(sim, &caps);
    EXPECT_THAT(status, t::AnyOf(t::Eq(XhciStatus::kOk),
                                 t::Eq(XhciStatus::kBadCapabilities),
                                 t::Eq(XhciStatus::kNotDecoding)));
    EXPECT_THAT(caps.protocol_count, t::Le(kMaxProtocolRanges));
    for (int i = 0; i < caps.protocol_count; ++i) {
      EXPECT_THAT(caps.protocols[i].first_port, t::Ge(1));
      EXPECT_THAT(
          caps.protocols[i].first_port + caps.protocols[i].port_count - 1,
          t::Le(caps.max_ports));
    }
    EXPECT_FALSE(sim.HasViolation(Violation::kOutOfRangeAccess))
        << "round " << round;
    EXPECT_THAT(sim.Writes(), t::IsEmpty());
  }
}

TEST_F(XhciRegsSimTest, Write64LoHiWritesTheLowDwordFirst) {
  XhciSim& sim = Create(XhciSimConfig{});
  // Reset the controller so that writing DCBAAP is allowed.
  sim.Write32(0x40, kUsbCmdHcrst);
  clock_.Advance(10);
  ASSERT_THAT(sim.Read32(0x40) & kUsbCmdHcrst, t::Eq(0u));
  ASSERT_THAT(sim.Read32(0x44) & kUsbStsCnr, t::Eq(0u));
  const int before = static_cast<int>(sim.accesses().size());
  Write64LoHi(sim, 0x40 + kOpDcbaap, 0x0000000123456780ull);
  ASSERT_THAT(static_cast<int>(sim.accesses().size()), t::Eq(before + 2));
  const Access& low = sim.accesses()[static_cast<size_t>(before)];
  const Access& high = sim.accesses()[static_cast<size_t>(before + 1)];
  EXPECT_TRUE(low.write);
  EXPECT_THAT(low.offset, t::Eq(0x70));
  EXPECT_THAT(low.value, t::Eq(0x23456780u));
  EXPECT_TRUE(high.write);
  EXPECT_THAT(high.offset, t::Eq(0x74));
  EXPECT_THAT(high.value, t::Eq(0x1u));
  // Both halves are written even if the high one is 0.
  Write64LoHi(sim, 0x40 + kOpDcbaap, 0x2000);
  EXPECT_THAT(static_cast<int>(sim.WritesTo(0x74).size()), t::Eq(2));
  EXPECT_THAT(sim.Read32(0x70), t::Eq(0x2000u));
  EXPECT_THAT(sim.Read32(0x74), t::Eq(0u));
  EXPECT_THAT(sim.violations(), t::IsEmpty());
}

}  // namespace
}  // namespace protos::usb
