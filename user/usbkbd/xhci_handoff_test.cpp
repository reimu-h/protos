// Tests of the USB Legacy Support handoff (BIOS -> OS) of
// XhciController::TakeOwnershipAndReset() against the simulated controller.
// QEMU has no USB Legacy Support capability, so these tests are the only
// check of this path before real PCs.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <ostream>
#include <vector>

#include "user/usbkbd/xhci_controller.h"
#include "user/usbkbd/xhci_platform.h"
#include "user/usbkbd/xhci_regs.h"
#include "user/usbkbd/xhci_sim.h"

namespace protos::usb {

// Print statuses and outcomes by name in failure messages.
static void PrintTo(const XhciStatus status, std::ostream* const os) {
  *os << XhciStatusName(status);
}

static void PrintTo(const HandoffOutcome outcome, std::ostream* const os) {
  switch (outcome) {
    case HandoffOutcome::kNotRun:
      *os << "not run";
      return;
    case HandoffOutcome::kNoLegacyCapability:
      *os << "no legacy capability";
      return;
    case HandoffOutcome::kNotBiosOwned:
      *os << "not BIOS owned";
      return;
    case HandoffOutcome::kReleased:
      *os << "released";
      return;
    case HandoffOutcome::kForced:
      *os << "forced";
      return;
  }
  *os << "unknown outcome";
}

namespace {

namespace t = ::testing;

using xhci_sim::Access;
using xhci_sim::BiosBehavior;
using xhci_sim::FakePlatform;
using xhci_sim::IntelLikeConfig;
using xhci_sim::LegacyCap;
using xhci_sim::ProtocolCap;
using xhci_sim::SimClock;
using xhci_sim::Violation;
using xhci_sim::XhciSim;
using xhci_sim::XhciSimConfig;

// The legacy capability of IntelLikeConfig() and its registers' bits, written
// out by hand from the xHCI specification.
constexpr int64_t kLegSup = 0x8000;
constexpr int64_t kLegCtlSts = 0x8004;
constexpr uint32_t kBiosOwned = 1u << 16;
constexpr uint32_t kOsOwned = 1u << 24;
constexpr uint32_t kSmiEnables = 0x0000E011u;   // Bits 0, 4, 13, 14, 15
constexpr uint32_t kSmiEvents = 0xE0000000u;    // Bits 29, 30, 31
constexpr uint32_t kCtlStsRsvdP = 0x000E1FEEu;  // Bits 1-3, 5-12, 17-19
// The default USBLEGCTLSTS of the firmware model: all SMI enables, RsvdP
// pattern 0x00040084 and all SMI events.
constexpr uint32_t kFirmwareCtlSts = 0xE004E095u;
// 1000 ms at 250 Hz, rounded up, plus one tick.
constexpr int64_t kHandoffTicks = 251;

class XhciHandoffTest : public t::Test {
 protected:
  // A halted PC-like controller whose firmware owns it.
  static XhciSimConfig PcConfig() { return IntelLikeConfig(); }

  XhciSim& Start(const XhciSimConfig& config) {
    sim_ = std::make_unique<XhciSim>(&clock_, config);
    controller_.Init(platform_, *sim_, "00:04.0");
    return *sim_;
  }

  // The only write to `offset`.
  Access OnlyWrite(const int64_t offset) {
    const std::vector<Access> writes = sim_->WritesTo(offset);
    EXPECT_THAT(writes.size(), t::Eq(1u)) << sim_->TraceString();
    return writes.empty() ? Access{} : writes[0];
  }

  // Checks the end state of every successful handoff: OS owned, SMIs off,
  // SMI events clear, RsvdP bits as firmware set them.
  void ExpectOwnedWithSmisDisabled(const uint32_t rsvdp) {
    const uint32_t legsup = sim_->legsup();
    EXPECT_THAT(legsup & (kBiosOwned | kOsOwned), t::Eq(kOsOwned));
    const uint32_t control = sim_->legctlsts();
    EXPECT_THAT(control & kSmiEnables, t::Eq(0u));
    EXPECT_THAT(control & kSmiEvents, t::Eq(0u));
    EXPECT_THAT(control & kCtlStsRsvdP, t::Eq(rsvdp));
  }

  SimClock clock_;
  FakePlatform platform_{&clock_};
  std::unique_ptr<XhciSim> sim_;
  XhciController controller_;
};

TEST_F(XhciHandoffTest, TimeoutConstantsAreTheSpecifiedOnes) {
  EXPECT_THAT(kHandoffTimeoutMs, t::Eq(1000));
  EXPECT_THAT(TicksForMs(platform_, kHandoffTimeoutMs), t::Eq(kHandoffTicks));
}

TEST_F(XhciHandoffTest, FirmwareReleasesAfterAFewPolls) {
  XhciSim& sim = Start(PcConfig());
  const uint32_t initial = sim.Peek(kLegSup);
  ASSERT_THAT(initial & (kBiosOwned | kOsOwned), t::Eq(kBiosOwned));

  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(controller_.handoff_outcome(), t::Eq(HandoffOutcome::kReleased));
  EXPECT_TRUE(sim.released_by_bios());
  EXPECT_FALSE(sim.forced());
  // The request sets OS Owned and leaves BIOS Owned for firmware to clear.
  const Access request = OnlyWrite(kLegSup);
  EXPECT_THAT(request.value, t::Eq(initial | kOsOwned));
  EXPECT_THAT(sim.legsup_reads_since_request(), t::Ge(3));
  const Access control = OnlyWrite(kLegCtlSts);
  EXPECT_THAT(control.value,
              t::Eq((kFirmwareCtlSts & kCtlStsRsvdP) | kSmiEvents));
  EXPECT_THAT(control.tick - request.tick, t::Lt(kHandoffTicks));
  ExpectOwnedWithSmisDisabled(kFirmwareCtlSts & kCtlStsRsvdP);
  EXPECT_TRUE(platform_.LogContains(
      "[USBKBD] 00:04.0: firmware released ownership after 8 ms"));
  EXPECT_FALSE(platform_.LogContains("timeout"));
  EXPECT_THAT(controller_.state(), t::Eq(XhciState::kReset));
}

TEST_F(XhciHandoffTest, FirmwareReleasesAfterTwoHundredMilliseconds) {
  XhciSimConfig config = PcConfig();
  config.legacy.behavior = BiosBehavior::kReleasesAfterTicks;
  config.legacy.release_after_ticks = 50;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(controller_.handoff_outcome(), t::Eq(HandoffOutcome::kReleased));
  EXPECT_TRUE(sim.released_by_bios());
  const Access request = OnlyWrite(kLegSup);
  const Access control = OnlyWrite(kLegCtlSts);
  EXPECT_THAT(control.tick - request.tick, t::Eq(50));
  EXPECT_TRUE(platform_.LogContains("released ownership after 200 ms"));
  ExpectOwnedWithSmisDisabled(kFirmwareCtlSts & kCtlStsRsvdP);
}

TEST_F(XhciHandoffTest, FirmwareReleasingUpToTheDeadlineIsNotForced) {
  // 996 ms, and the tick of the deadline itself.
  for (const int64_t release_ticks : {int64_t{249}, kHandoffTicks}) {
    XhciSimConfig config = PcConfig();
    config.legacy.behavior = BiosBehavior::kReleasesAfterTicks;
    config.legacy.release_after_ticks = release_ticks;
    XhciSim& sim = Start(config);
    EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
    EXPECT_THAT(controller_.handoff_outcome(), t::Eq(HandoffOutcome::kReleased))
        << release_ticks;
    EXPECT_FALSE(sim.forced());
    EXPECT_THAT(sim.WritesTo(kLegSup).size(), t::Eq(1u));
  }
}

TEST_F(XhciHandoffTest, FirmwareReleasingAfterTheDeadlineIsForced) {
  XhciSimConfig config = PcConfig();
  config.legacy.behavior = BiosBehavior::kReleasesAfterTicks;
  config.legacy.release_after_ticks = kHandoffTicks + 1;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(controller_.handoff_outcome(), t::Eq(HandoffOutcome::kForced));
  EXPECT_TRUE(sim.forced());
  EXPECT_FALSE(sim.released_by_bios());
}

TEST_F(XhciHandoffTest, FirmwareThatNeverReleasesIsForcedAfterOneSecond) {
  XhciSimConfig config = PcConfig();
  config.legacy.behavior = BiosBehavior::kNeverReleases;
  config.legacy.legsup_rsvdp = 0x82040000u;  // RsvdP bits 31, 25, 18
  XhciSim& sim = Start(config);
  const uint32_t initial = sim.Peek(kLegSup);

  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(controller_.handoff_outcome(), t::Eq(HandoffOutcome::kForced));
  // The simulator saw the driver clear BIOS Owned itself, and not too early.
  EXPECT_TRUE(sim.forced());
  EXPECT_FALSE(sim.HasViolation(Violation::kBiosSemaphoreClearedEarly));
  const std::vector<Access> writes = sim.WritesTo(kLegSup);
  ASSERT_THAT(writes.size(), t::Eq(2u));
  EXPECT_THAT(writes[0].value, t::Eq(initial | kOsOwned));
  // The forced write: OS Owned kept, BIOS Owned cleared, the rest as read.
  EXPECT_THAT(writes[1].value, t::Eq((initial | kOsOwned) & ~kBiosOwned));
  // After at least 1000 ms (250 ticks) and at most one tick more than the
  // rounded-up timeout.
  const int64_t waited = writes[1].tick - writes[0].tick;
  EXPECT_THAT(waited, t::Ge(250));
  EXPECT_THAT(waited, t::Le(kHandoffTicks + 1));
  // SMIs are disabled after the forced takeover too.
  const Access control = OnlyWrite(kLegCtlSts);
  EXPECT_THAT(control.seq, t::Gt(writes[1].seq));
  ExpectOwnedWithSmisDisabled(kFirmwareCtlSts & kCtlStsRsvdP);
  EXPECT_TRUE(platform_.LogContains(
      "[USBKBD] 00:04.0: firmware handoff timeout after 1000 ms, forcing OS "
      "ownership"));
  // The halt and reset still follow.
  EXPECT_THAT(sim.reset_count(), t::Eq(1));
  EXPECT_THAT(controller_.state(), t::Eq(XhciState::kReset));
  // The whole bring-up stays bounded: the handoff wait, the 1 ms after the
  // reset and a few polls.
  EXPECT_THAT(platform_.relax_calls(), t::Le(kHandoffTicks + 10));
}

TEST_F(XhciHandoffTest, HandoffTimeoutIsMeasuredInTicksNotPolls) {
  XhciSimConfig config = PcConfig();
  config.legacy.behavior = BiosBehavior::kNeverReleases;
  XhciSim& sim = Start(config);
  platform_.set_relaxes_per_tick(10);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(controller_.handoff_outcome(), t::Eq(HandoffOutcome::kForced));
  const std::vector<Access> writes = sim.WritesTo(kLegSup);
  ASSERT_THAT(writes.size(), t::Eq(2u));
  EXPECT_THAT(writes[1].tick - writes[0].tick, t::Eq(kHandoffTicks));
  EXPECT_THAT(sim.legsup_reads_since_request(), t::Gt(2000));
}

TEST_F(XhciHandoffTest, HandoffTimeoutFollowsTheTickRate) {
  platform_.set_ticks_per_second(1000);
  XhciSimConfig config = PcConfig();
  config.legacy.behavior = BiosBehavior::kNeverReleases;
  config.legacy.force_floor_ticks = 1000;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  const std::vector<Access> writes = sim.WritesTo(kLegSup);
  ASSERT_THAT(writes.size(), t::Eq(2u));
  EXPECT_THAT(writes[1].tick - writes[0].tick, t::Eq(1001));
  EXPECT_TRUE(sim.forced());
}

TEST_F(XhciHandoffTest, ControllerThatFirmwareDoesNotOwnIsClaimedAtOnce) {
  XhciSimConfig config = PcConfig();
  config.legacy.bios_owned = false;
  XhciSim& sim = Start(config);
  const uint32_t initial = sim.Peek(kLegSup);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(controller_.handoff_outcome(),
              t::Eq(HandoffOutcome::kNotBiosOwned));
  const Access request = OnlyWrite(kLegSup);
  EXPECT_THAT(request.value, t::Eq(initial | kOsOwned));
  // No polling after the request.
  for (const Access& access : sim.accesses()) {
    if (access.offset == kLegSup && access.seq > request.seq) {
      ADD_FAILURE() << "USBLEGSUP accessed after the request: #" << access.seq;
    }
  }
  EXPECT_THAT(OnlyWrite(kLegCtlSts).tick, t::Eq(request.tick));
  ExpectOwnedWithSmisDisabled(kFirmwareCtlSts & kCtlStsRsvdP);
  EXPECT_THAT(platform_.logs(), t::IsEmpty());
}

TEST_F(XhciHandoffTest, ControllerAlreadyOwnedByTheOsIsLeftAsIs) {
  // OVMF hands the controller over at ExitBootServices (OS Owned set).
  XhciSimConfig config = PcConfig();
  config.legacy.bios_owned = false;
  config.legacy.os_owned = true;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(controller_.handoff_outcome(),
              t::Eq(HandoffOutcome::kNotBiosOwned));
  EXPECT_THAT(sim.WritesTo(kLegSup), t::IsEmpty());
  // Read once as the capability header by the capability walk, once by the
  // handoff; no polling.
  EXPECT_THAT(sim.CountReads(kLegSup), t::Eq(2));
  // SMIs are disabled all the same.
  OnlyWrite(kLegCtlSts);
  ExpectOwnedWithSmisDisabled(kFirmwareCtlSts & kCtlStsRsvdP);
}

TEST_F(XhciHandoffTest, PendingRequestIsWaitedForWithoutAskingAgain) {
  // An earlier driver instance asked; firmware has not released yet.
  XhciSimConfig config = PcConfig();
  config.legacy.os_owned = true;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(controller_.handoff_outcome(), t::Eq(HandoffOutcome::kReleased));
  EXPECT_THAT(sim.WritesTo(kLegSup), t::IsEmpty());
  ExpectOwnedWithSmisDisabled(kFirmwareCtlSts & kCtlStsRsvdP);
}

TEST_F(XhciHandoffTest, ControllerWithoutTheCapabilityIsNotHandedOver) {
  XhciSim& sim = Start(XhciSimConfig{});  // QEMU
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(controller_.handoff_outcome(),
              t::Eq(HandoffOutcome::kNoLegacyCapability));
  EXPECT_THAT(controller_.caps().legacy_offset, t::Eq(0));
  // Only USBCMD was written (the reset).
  for (const Access& write : sim.Writes()) {
    EXPECT_THAT(write.offset, t::Eq(sim.op(0x00)));
  }
  EXPECT_THAT(clock_.now() - SimClock::kStartTick, t::Le(5));
}

TEST_F(XhciHandoffTest, SmiDisableKeepsRsvdPAndClearsEnablesAndEvents) {
  XhciSimConfig config = PcConfig();
  config.legacy.ctlsts = 0xE00EFFFFu;  // Every enable, RsvdP bit and event
  config.usbsts_pending = 0x4 | 0x8;   // Shadowed in bits 16 and 20
  XhciSim& sim = Start(config);
  ASSERT_THAT(sim.Peek(kLegCtlSts), t::Eq(0xE01FFFFFu));
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  // RsvdP as read, enables 0, events written 1 (cleared), RO and RsvdZ 0.
  EXPECT_THAT(OnlyWrite(kLegCtlSts).value, t::Eq(0xE00E1FEEu));
  EXPECT_THAT(sim.legctlsts(), t::Eq(0x000E1FEEu));
  EXPECT_FALSE(sim.HasViolation(Violation::kRsvdPChanged));
  EXPECT_FALSE(sim.HasViolation(Violation::kRsvdZWrittenOne));
}

TEST_F(XhciHandoffTest, OwnershipIsRequestedBeforeSmisAreDisabled) {
  // Firmware notices the request through the "SMI on OS Ownership" SMI, so
  // disabling the SMIs first would make every handoff time out.
  XhciSimConfig config = PcConfig();
  config.legacy.release_requires_os_ownership_smi = true;
  config.legacy.release_after_polls = 2;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(controller_.handoff_outcome(), t::Eq(HandoffOutcome::kReleased));
  EXPECT_THAT(sim.FirstWriteSeq(kLegSup, kOsOwned),
              t::Lt(sim.FirstWriteSeq(kLegCtlSts)));
}

TEST_F(XhciHandoffTest, OwnershipChangeEventOfTheRequestIsCleared) {
  XhciSimConfig config = PcConfig();
  config.legacy.ctlsts = 0x0000E011u;  // Enables only, no events pending
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  // The request set "SMI on OS Ownership Change" (bit 29); the driver
  // cleared it afterwards.
  EXPECT_THAT(OnlyWrite(kLegCtlSts).value & (1u << 29), t::Ne(0u));
  EXPECT_THAT(sim.legctlsts() & (1u << 29), t::Eq(0u));
}

TEST_F(XhciHandoffTest, LegacyCapabilityFarIntoTheBar) {
  XhciSimConfig config = PcConfig();
  config.ext_caps = {ProtocolCap(0xEF00, 2, 0, 1, 10),
                     ProtocolCap(0xEF10, 3, 0, 11, 6), LegacyCap(0xF000)};
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(controller_.caps().legacy_offset, t::Eq(0xF000));
  EXPECT_THAT(controller_.handoff_outcome(), t::Eq(HandoffOutcome::kReleased));
  EXPECT_THAT(sim.WritesTo(0xF000).size(), t::Eq(1u));
  EXPECT_THAT(sim.WritesTo(0xF004).size(), t::Eq(1u));
}

TEST_F(XhciHandoffTest, HandoffEndsBeforeTheFirstOperationalRegisterWrite) {
  XhciSimConfig config = PcConfig();
  config.running = true;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  const int64_t first_command = sim.FirstWriteSeq(sim.op(0x00));
  ASSERT_THAT(first_command, t::Gt(0));
  for (const Access& write : sim.Writes()) {
    if (write.offset == kLegSup || write.offset == kLegCtlSts) {
      EXPECT_THAT(write.seq, t::Lt(first_command));
    }
  }
}

TEST_F(XhciHandoffTest, ControllerThatStopsRespondingDuringTheHandoff) {
  XhciSimConfig config = PcConfig();
  config.legacy.behavior = BiosBehavior::kNeverReleases;
  config.stop_decoding_after_ticks = 10;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(),
              t::Eq(XhciStatus::kNotDecoding));
  EXPECT_THAT(controller_.state(), t::Eq(XhciState::kError));
  // No forced write, no SMI write, no reset: the controller is left alone.
  EXPECT_THAT(sim.Writes().size(), t::Eq(1u));
  EXPECT_THAT(clock_.now() - SimClock::kStartTick, t::Le(11));
  EXPECT_TRUE(platform_.LogContains(
      "[USBKBD] 00:04.0: error: controller not responding"));
}

TEST_F(XhciHandoffTest, SecondTakeoverFindsTheControllerOwnedByTheOs) {
  XhciSim& sim = Start(PcConfig());
  ASSERT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  ASSERT_THAT(controller_.handoff_outcome(), t::Eq(HandoffOutcome::kReleased));
  const int64_t before = clock_.now();
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(controller_.handoff_outcome(),
              t::Eq(HandoffOutcome::kNotBiosOwned));
  EXPECT_THAT(sim.WritesTo(kLegSup).size(), t::Eq(1u));
  EXPECT_THAT(sim.WritesTo(kLegCtlSts).size(), t::Eq(2u));
  EXPECT_THAT(sim.reset_count(), t::Eq(2));
  EXPECT_THAT(clock_.now() - before, t::Le(5));
}

TEST_F(XhciHandoffTest, BiosDisablingSmisOnReleaseStillLeavesEventsClear) {
  XhciSimConfig config = PcConfig();
  config.legacy.bios_disables_smis_on_release = true;
  Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(controller_.handoff_outcome(), t::Eq(HandoffOutcome::kReleased));
  ExpectOwnedWithSmisDisabled(kFirmwareCtlSts & kCtlStsRsvdP);
}

TEST_F(XhciHandoffTest, MalformedXecpIntoLiveRegistersIsRejectedWithoutWrites) {
  for (const int xecp : {1, 0x10}) {  // HCSPARAMS1 (0x04) and USBCMD (0x40)
    XhciSimConfig config;
    config.running = true;
    config.max_slots = 1;  // Low byte of HCSPARAMS1 == 1 (Legacy Support ID)
    config.xecp_override = xecp;
    XhciSim& sim = Start(config);
    EXPECT_THAT(controller_.TakeOwnershipAndReset(),
                t::Eq(XhciStatus::kBadCapabilities));
    EXPECT_THAT(sim.Writes(), t::IsEmpty());
  }
}

}  // namespace
}  // namespace protos::usb
