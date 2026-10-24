// Tests of the halt and reset of XhciController::TakeOwnershipAndReset()
// against the simulated controller: whatever state firmware left the
// controller in, it is halted (if it runs) and reset before anything is
// programmed, every wait is bounded, and a controller that does not halt is
// never reset.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "user/usbkbd/xhci_controller.h"
#include "user/usbkbd/xhci_platform.h"
#include "user/usbkbd/xhci_regs.h"
#include "user/usbkbd/xhci_sim.h"

namespace protos::usb {

// Prints statuses by name in failure messages.
static void PrintTo(const XhciStatus status, std::ostream* const os) {
  *os << XhciStatusName(status);
}

namespace {

namespace t = ::testing;

using xhci_sim::Access;
using xhci_sim::BiosBehavior;
using xhci_sim::FakePlatform;
using xhci_sim::IntelLikeConfig;
using xhci_sim::OvmfLeftHaltedConfig;
using xhci_sim::SeaBiosLeftRunningConfig;
using xhci_sim::SimClock;
using xhci_sim::Violation;
using xhci_sim::XhciSim;
using xhci_sim::XhciSimConfig;

// Registers of the QEMU layout and their bits, written out by hand from the
// xHCI specification.
constexpr int64_t kUsbCmd = 0x40;
constexpr int64_t kUsbSts = 0x44;
constexpr int64_t kCrcr = 0x58;
constexpr int64_t kDcbaap = 0x70;
constexpr int64_t kConfig = 0x78;
constexpr int64_t kPortsc5 = 0x480;
constexpr int64_t kIman0 = 0x1020;
constexpr int64_t kErstSz0 = 0x1028;
constexpr int64_t kErstBa0 = 0x1030;
constexpr int64_t kErdp0 = 0x1038;
constexpr uint32_t kRunStop = 1u << 0;
constexpr uint32_t kHcReset = 1u << 1;
constexpr uint32_t kInterrupterEnable = 1u << 2;
constexpr uint32_t kHostSystemErrorEnable = 1u << 3;
constexpr uint32_t kEnableWrapEvent = 1u << 10;
constexpr uint32_t kHalted = 1u << 0;
constexpr uint32_t kHcError = 1u << 12;
// Timeouts in ticks at 250 Hz: rounded up, plus one tick.
constexpr int64_t kHandoffTicks = 251;  // 1000 ms
constexpr int64_t kReadyTicks = 1251;   // 5000 ms
constexpr int64_t kHaltTicks = 9;       // 32 ms
constexpr int64_t kQuietTicks = 2;      // 1 ms
constexpr int64_t kResetTicks = 2501;   // 10000 ms

class XhciResetTest : public t::Test {
 protected:
  XhciSim& Start(const XhciSimConfig& config) {
    sim_ = std::make_unique<XhciSim>(&clock_, config);
    controller_.Init(platform_, *sim_, "00:04.0");
    return *sim_;
  }

  // Every phase-1 run: no DMA memory, no console input.
  void TearDown() override {
    EXPECT_THAT(platform_.dma_alloc_calls(), t::Eq(0));
    EXPECT_THAT(platform_.input(), t::IsEmpty());
  }

  SimClock clock_;
  FakePlatform platform_{&clock_};
  std::unique_ptr<XhciSim> sim_;
  XhciController controller_;
};

TEST_F(XhciResetTest, TimeoutConstantsAreTheSpecifiedOnes) {
  EXPECT_THAT(kControllerReadyTimeoutMs, t::Eq(5000));
  EXPECT_THAT(kHaltTimeoutMs, t::Eq(32));
  EXPECT_THAT(kPostResetDelayMs, t::Eq(1));
  EXPECT_THAT(kResetTimeoutMs, t::Eq(10000));
  EXPECT_THAT(kResetReadyTimeoutMs, t::Eq(10000));
  EXPECT_THAT(TicksForMs(platform_, kControllerReadyTimeoutMs),
              t::Eq(kReadyTicks));
  EXPECT_THAT(TicksForMs(platform_, kHaltTimeoutMs), t::Eq(kHaltTicks));
  EXPECT_THAT(TicksForMs(platform_, kPostResetDelayMs), t::Eq(kQuietTicks));
  EXPECT_THAT(TicksForMs(platform_, kResetTimeoutMs), t::Eq(kResetTicks));
}

TEST_F(XhciResetTest, HaltedQemuControllerIsResetWithoutAHaltWrite) {
  XhciSim& sim = Start(XhciSimConfig{});
  EXPECT_THAT(controller_.state(), t::Eq(XhciState::kBound));
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(controller_.state(), t::Eq(XhciState::kReset));
  EXPECT_THAT(controller_.caps().max_ports, t::Eq(8));
  EXPECT_THAT(controller_.caps().protocol_count, t::Eq(2));
  const std::vector<Access> writes = sim.Writes();
  ASSERT_THAT(writes.size(), t::Eq(1u));
  EXPECT_THAT(writes[0].offset, t::Eq(kUsbCmd));
  EXPECT_THAT(writes[0].value, t::Eq(kHcReset));
  EXPECT_THAT(sim.reset_count(), t::Eq(1));
  EXPECT_TRUE(sim.reset_observed());
  EXPECT_THAT(platform_.logs(), t::IsEmpty());
}

TEST_F(XhciResetTest, RunningControllerIsHaltedThenResetBeforeReprogramming) {
  XhciSimConfig config = SeaBiosLeftRunningConfig();
  config.usbcmd_extra =
      kInterrupterEnable | kHostSystemErrorEnable | kEnableWrapEvent;
  config.usbcmd_rsvdp = 0x00A00010u;  // RsvdP bits 23, 21, 4
  config.halt_delay_ticks = 2;
  XhciSim& sim = Start(config);
  const uint32_t initial_command = sim.Peek(kUsbCmd);
  ASSERT_THAT(sim.Peek(kUsbSts) & kHalted, t::Eq(0u));

  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  // Two USBCMD writes and nothing else: the halt, then the reset.
  const std::vector<Access> writes = sim.Writes();
  ASSERT_THAT(writes.size(), t::Eq(2u)) << sim.TraceString();
  EXPECT_THAT(writes[0].offset, t::Eq(kUsbCmd));
  EXPECT_THAT(writes[1].offset, t::Eq(kUsbCmd));
  // The halt clears Run/Stop and the interrupt enables and keeps RsvdP.
  EXPECT_THAT(
      writes[0].value,
      t::Eq(initial_command & ~(kRunStop | kInterrupterEnable |
                                kHostSystemErrorEnable | kEnableWrapEvent)));
  EXPECT_THAT(writes[0].value, t::Eq(0x00A00010u));
  // HCRST is set only after HCH = 1 was seen, and the reset completes.
  EXPECT_THAT(writes[1].value, t::Eq(0x00A00010u | kHcReset));
  EXPECT_THAT(sim.halt_observed_seq(), t::Gt(writes[0].seq));
  EXPECT_THAT(sim.reset_write_seq(), t::Eq(writes[1].seq));
  EXPECT_THAT(sim.halt_observed_seq(), t::Lt(sim.reset_write_seq()));
  EXPECT_THAT(sim.reset_observed_seq(), t::Gt(sim.reset_write_seq()));
  EXPECT_THAT(sim.reset_count(), t::Eq(1));
  // Everything firmware left is gone.
  EXPECT_THAT(sim.Peek(kUsbCmd), t::Eq(0x00A00010u));
  EXPECT_THAT(sim.Peek(kUsbSts), t::Eq(kHalted));
  EXPECT_THAT(sim.Peek(kDcbaap), t::Eq(0u));
  EXPECT_THAT(sim.Peek(kCrcr), t::Eq(0u));
  EXPECT_THAT(sim.Peek(kConfig), t::Eq(0u));
  EXPECT_THAT(sim.Peek(kIman0), t::Eq(0u));
  EXPECT_THAT(sim.Peek(kErstSz0), t::Eq(0u));
  EXPECT_THAT(sim.Peek(kErstBa0), t::Eq(0u));
  EXPECT_THAT(sim.Peek(kErdp0), t::Eq(0u));
  // The keyboard port is connected again but not enabled.
  EXPECT_THAT(sim.Peek(kPortsc5), t::Eq(0x00020EE1u));
  EXPECT_TRUE(platform_.LogContains(
      "[USBKBD] 00:04.0: controller was running, halted"));
}

TEST_F(XhciResetTest, HaltedControllerWithStaleStateIsResetToo) {
  XhciSim& sim = Start(OvmfLeftHaltedConfig());
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(sim.WritesTo(kUsbCmd).size(), t::Eq(1u));
  EXPECT_THAT(sim.FirstWriteSeq(kUsbCmd, kRunStop), t::Eq(-1));
  EXPECT_THAT(sim.reset_count(), t::Eq(1));
  EXPECT_THAT(sim.Peek(kDcbaap), t::Eq(0u));
  EXPECT_THAT(sim.Peek(kCrcr), t::Eq(0u));
  EXPECT_THAT(sim.Peek(kConfig), t::Eq(0u));
  EXPECT_THAT(sim.Peek(kErstBa0), t::Eq(0u));
  EXPECT_THAT(sim.Peek(kUsbSts), t::Eq(kHalted));
}

TEST_F(XhciResetTest, ControllerStillHaltingIsWaitedForBeforeTheReset) {
  XhciSimConfig config;
  config.halt_in_progress = true;
  config.halt_delay_ticks = 3;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  // Run/Stop was already 0: no halt write, but HCRST waited for HCH = 1.
  EXPECT_THAT(sim.WritesTo(kUsbCmd).size(), t::Eq(1u));
  EXPECT_THAT(sim.halt_observed_seq(), t::Gt(0));
  EXPECT_THAT(sim.halt_observed_seq(), t::Lt(sim.reset_write_seq()));
  EXPECT_THAT(sim.reset_write_tick() - sim.start_tick(), t::Eq(3));
}

TEST_F(XhciResetTest, HaltWithinThirtyTwoMillisecondsSucceeds) {
  for (const int64_t delay : {kHaltTicks - 1, kHaltTicks}) {
    XhciSimConfig config;
    config.running = true;
    config.halt_delay_ticks = delay;
    XhciSim& sim = Start(config);
    EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk))
        << delay;
    EXPECT_THAT(sim.reset_count(), t::Eq(1));
  }
}

TEST_F(XhciResetTest, ControllerThatDoesNotHaltIsAbandonedWithoutAReset) {
  XhciSimConfig config;
  config.running = true;
  config.never_halts = true;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(),
              t::Eq(XhciStatus::kHaltTimeout));
  EXPECT_THAT(controller_.state(), t::Eq(XhciState::kError));
  // Only the halt write: HCRST was never written, nothing was programmed.
  const std::vector<Access> writes = sim.Writes();
  ASSERT_THAT(writes.size(), t::Eq(1u));
  EXPECT_THAT(writes[0].value & kHcReset, t::Eq(0u));
  EXPECT_THAT(sim.FirstWriteSeq(kUsbCmd, kHcReset), t::Eq(-1));
  EXPECT_THAT(sim.reset_count(), t::Eq(0));
  // Given up after at least 32 ms (8 ticks), at the rounded-up deadline.
  const int64_t waited = clock_.now() - writes[0].tick;
  EXPECT_THAT(waited, t::Ge(8));
  EXPECT_THAT(waited, t::Le(kHaltTicks + 1));
  EXPECT_TRUE(platform_.LogContains("[USBKBD] 00:04.0: error: halt timeout"));
}

TEST_F(XhciResetTest, HaltTakingLongerThanThirtyTwoMillisecondsIsAbandoned) {
  XhciSimConfig config;
  config.running = true;
  config.halt_delay_ticks = kHaltTicks + 1;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(),
              t::Eq(XhciStatus::kHaltTimeout));
  EXPECT_THAT(sim.reset_count(), t::Eq(0));
}

TEST_F(XhciResetTest, NotReadyAtStartIsWaitedOutBeforeTheFirstWrite) {
  XhciSimConfig config;
  config.running = true;
  config.power_on_not_ready_ticks = 50;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  const std::vector<Access> writes = sim.Writes();
  ASSERT_THAT(writes.size(), t::Eq(2u));
  EXPECT_THAT(writes[0].tick - sim.start_tick(), t::Eq(50));
  EXPECT_FALSE(sim.HasViolation(Violation::kOpWriteWhileNotReady));
  EXPECT_THAT(platform_.logs(),
              t::ElementsAre("[USBKBD] 00:04.0: controller was running, "
                             "halted"));
}

TEST_F(XhciResetTest, NotReadyForFiveSecondsIsLoggedAndTheResetGoesOn) {
  // Like Linux: after 5 s the driver says so and resets anyway; here the
  // controller becomes ready during the reset.
  XhciSimConfig config;
  config.power_on_not_ready_ticks = 1300;
  XhciSim& sim = Start(config);
  sim.AllowViolation(Violation::kOpWriteWhileNotReady);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(sim.reset_write_tick() - sim.start_tick(), t::Eq(kReadyTicks));
  EXPECT_THAT(sim.CountViolations(Violation::kOpWriteWhileNotReady), t::Eq(1));
  EXPECT_THAT(clock_.now() - sim.start_tick(), t::Eq(1300));
  EXPECT_THAT(platform_.logs(),
              t::ElementsAre("[USBKBD] 00:04.0: controller not ready after "
                             "5000 ms, continuing"));
}

TEST_F(XhciResetTest, ControllerThatIsNeverReadyEndsWithAnError) {
  XhciSimConfig config;
  config.power_on_not_ready_forever = true;
  XhciSim& sim = Start(config);
  sim.AllowViolation(Violation::kOpWriteWhileNotReady);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(),
              t::Eq(XhciStatus::kNotReadyAfterReset));
  EXPECT_THAT(controller_.state(), t::Eq(XhciState::kError));
  // 5 s before the reset, the 1 ms quiet period, 10 s after it.
  EXPECT_THAT(clock_.now() - sim.start_tick(),
              t::Eq(kReadyTicks + kQuietTicks + kResetTicks));
  EXPECT_TRUE(platform_.LogContains("not ready after 5000 ms, continuing"));
  EXPECT_TRUE(platform_.LogContains(
      "[USBKBD] 00:04.0: error: controller not ready after reset"));
}

TEST_F(XhciResetTest, SlowResetIsWaitedForWithReadsOnly) {
  XhciSimConfig config;
  config.reset_delay_ticks = 20;
  config.not_ready_after_reset_ticks = 10;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(sim.reset_observed_seq(), t::Gt(sim.reset_write_seq()));
  for (const Access& access : sim.accesses()) {
    if (access.seq > sim.reset_write_seq() &&
        access.seq <= sim.reset_observed_seq()) {
      EXPECT_FALSE(access.write);
      EXPECT_THAT(access.offset, t::AnyOf(t::Eq(kUsbCmd), t::Eq(kUsbSts)));
    }
  }
  EXPECT_THAT(clock_.now() - sim.reset_write_tick(), t::Eq(30));
}

TEST_F(XhciResetTest, HcrstThatNeverClearsEndsWithAnError) {
  XhciSimConfig config;
  config.hcrst_never_clears = true;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(),
              t::Eq(XhciStatus::kResetTimeout));
  EXPECT_THAT(controller_.state(), t::Eq(XhciState::kError));
  // Nothing after the HCRST write.
  EXPECT_THAT(sim.Writes().back().seq, t::Eq(sim.reset_write_seq()));
  EXPECT_THAT(clock_.now() - sim.reset_write_tick(),
              t::Eq(kQuietTicks + kResetTicks));
  EXPECT_TRUE(platform_.LogContains("[USBKBD] 00:04.0: error: reset timeout"));
}

TEST_F(XhciResetTest, NotReadyAfterTheResetEndsWithAnError) {
  XhciSimConfig config;
  config.not_ready_after_reset_forever = true;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(),
              t::Eq(XhciStatus::kNotReadyAfterReset));
  EXPECT_THAT(sim.Writes().back().seq, t::Eq(sim.reset_write_seq()));
  // HCRST clears after 1 tick (seen right after the quiet period), then CNR
  // gets its 10 s.
  EXPECT_THAT(clock_.now() - sim.reset_write_tick(),
              t::Eq(kQuietTicks + kResetTicks));
}

TEST_F(XhciResetTest, RunningAfterResetEndsWithAnError) {
  XhciSimConfig config;
  config.runs_after_reset = true;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(),
              t::Eq(XhciStatus::kRunningAfterReset));
  EXPECT_THAT(controller_.state(), t::Eq(XhciState::kError));
  EXPECT_THAT(sim.Writes().back().seq, t::Eq(sim.reset_write_seq()));
  EXPECT_TRUE(platform_.LogContains(
      "[USBKBD] 00:04.0: error: controller running after reset"));
}

TEST_F(XhciResetTest, NoRegisterAccessWithinOneMillisecondOfHcrst) {
  for (const int64_t ticks_per_second : {int64_t{250}, int64_t{1000}}) {
    platform_.set_ticks_per_second(ticks_per_second);
    XhciSimConfig config = IntelLikeConfig();
    // 1 ms rounded up to ticks.
    config.hcrst_quiet_ticks = (ticks_per_second + 999) / 1000;
    XhciSim& sim = Start(config);
    EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
    EXPECT_FALSE(sim.HasViolation(Violation::kAccessTooSoonAfterReset));
    for (const Access& access : sim.accesses()) {
      if (access.seq > sim.reset_write_seq()) {
        EXPECT_THAT(access.tick - sim.reset_write_tick(), t::Ge(2));
      }
    }
  }
}

TEST_F(XhciResetTest, ControllerInErrorWithRunStopSetIsResetWithoutWaiting) {
  XhciSimConfig config;
  config.hce = true;
  config.usbcmd_extra = kRunStop;  // Halted by the error, R/S still 1
  XhciSim& sim = Start(config);
  ASSERT_THAT(sim.Peek(kUsbSts), t::Eq(kHalted | kHcError));
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  const std::vector<Access> writes = sim.WritesTo(kUsbCmd);
  ASSERT_THAT(writes.size(), t::Eq(2u));
  EXPECT_THAT(writes[0].value & kRunStop, t::Eq(0u));
  EXPECT_THAT(writes[1].tick, t::Eq(writes[0].tick));  // No halt wait
  EXPECT_THAT(sim.Peek(kUsbSts), t::Eq(kHalted));
  EXPECT_FALSE(platform_.LogContains("was running"));
}

TEST_F(XhciResetTest, PendingStatusIsClearedByTheResetNotByWrites) {
  XhciSimConfig config;
  config.running = true;
  config.usbsts_pending = 0x4 | 0x8 | 0x10;  // HSE, EINT, PCD
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(sim.WritesTo(kUsbSts), t::IsEmpty());
  EXPECT_THAT(sim.Peek(kUsbSts), t::Eq(kHalted));
}

TEST_F(XhciResetTest, ControllerThatDoesNotRespondIsNotTouched) {
  XhciSimConfig config;
  config.all_ones = true;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(),
              t::Eq(XhciStatus::kNotDecoding));
  EXPECT_THAT(controller_.state(), t::Eq(XhciState::kError));
  EXPECT_THAT(sim.Writes(), t::IsEmpty());
  EXPECT_THAT(clock_.now(), t::Eq(sim.start_tick()));
  EXPECT_THAT(platform_.logs(),
              t::ElementsAre("[USBKBD] 00:04.0: error: controller not "
                             "responding"));
}

TEST_F(XhciResetTest, ControllerThatStopsRespondingWhileHaltingIsNotReset) {
  XhciSimConfig config;
  config.running = true;
  config.never_halts = true;
  config.stop_decoding_after_ticks = 5;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(),
              t::Eq(XhciStatus::kNotDecoding));
  EXPECT_THAT(sim.FirstWriteSeq(kUsbCmd, kHcReset), t::Eq(-1));
  EXPECT_THAT(clock_.now() - sim.start_tick(), t::Eq(5));
}

TEST_F(XhciResetTest, ControllerThatStopsRespondingDuringTheReset) {
  XhciSimConfig config;
  config.hcrst_never_clears = true;
  config.stop_decoding_after_ticks = 20;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(),
              t::Eq(XhciStatus::kNotDecoding));
  EXPECT_THAT(clock_.now() - sim.start_tick(), t::Eq(20));
}

TEST_F(XhciResetTest, InvalidCapabilitiesAreRejectedBeforeAnyWrite) {
  XhciSimConfig short_caps;
  short_caps.cap_length = 0x10;
  short_caps.ext_caps.clear();
  XhciSimConfig ports_past_bar;
  ports_past_bar.bar_size = 0x1400;
  ports_past_bar.max_ports = 255;
  ports_past_bar.db_off_raw = 0x800;
  ports_past_bar.rts_off_raw = 0xC00;
  ports_past_bar.ext_caps.clear();
  for (const XhciSimConfig& config : {short_caps, ports_past_bar}) {
    XhciSim& sim = Start(config);
    EXPECT_THAT(controller_.TakeOwnershipAndReset(),
                t::Eq(XhciStatus::kBadCapabilities));
    EXPECT_THAT(sim.Writes(), t::IsEmpty());
  }
  EXPECT_TRUE(platform_.LogContains(
      "[USBKBD] 00:04.0: error: invalid capability registers"));
}

TEST_F(XhciResetTest, UnboundControllerDoesNothing) {
  XhciController controller;
  EXPECT_THAT(controller.TakeOwnershipAndReset(), t::Eq(XhciStatus::kNotBound));
  EXPECT_THAT(controller.state(), t::Eq(XhciState::kUnbound));
  EXPECT_THAT(std::string(controller.label()), t::IsEmpty());
}

TEST_F(XhciResetTest, PhaseOneCanRunAgain) {
  XhciSimConfig config = SeaBiosLeftRunningConfig();
  XhciSim& sim = Start(config);
  ASSERT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(sim.reset_count(), t::Eq(2));
  EXPECT_THAT(controller_.state(), t::Eq(XhciState::kReset));
}

TEST_F(XhciResetTest, PcControllerFoundRunningIsHandedOverHaltedAndReset) {
  XhciSimConfig config = IntelLikeConfig();
  config.running = true;
  config.usbcmd_extra = kInterrupterEnable;
  config.halt_delay_ticks = 4;
  config.legacy.behavior = BiosBehavior::kReleasesAfterTicks;
  config.legacy.release_after_ticks = 25;  // 100 ms
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_TRUE(sim.released_by_bios());
  EXPECT_THAT(sim.halt_observed_seq(), t::Lt(sim.reset_write_seq()));
  EXPECT_TRUE(sim.reset_observed());
  // Port power control: the ports are unpowered after the reset.
  EXPECT_THAT(sim.Peek(sim.portsc(1)), t::Eq(0u));
  EXPECT_THAT(controller_.caps().context_size, t::Eq(64));
  EXPECT_THAT(controller_.caps().max_scratchpads, t::Eq(35));
  EXPECT_THAT(platform_.logs(),
              t::ElementsAre(
                  "[USBKBD] 00:04.0: firmware released ownership after 100 ms",
                  "[USBKBD] 00:04.0: controller was running, halted"));
}

TEST_F(XhciResetTest, SlowestSuccessfulBringUpIsBounded) {
  XhciSimConfig config = IntelLikeConfig();
  config.legacy.behavior = BiosBehavior::kNeverReleases;
  config.running = true;
  config.halt_delay_ticks = kHaltTicks - 1;
  config.reset_delay_ticks = 2000;
  config.not_ready_after_reset_ticks = 2000;
  XhciSim& sim = Start(config);
  EXPECT_THAT(controller_.TakeOwnershipAndReset(), t::Eq(XhciStatus::kOk));
  EXPECT_THAT(sim.reset_count(), t::Eq(1));
  // The forced handoff, the halt, the quiet period and both reset waits.
  EXPECT_THAT(platform_.relax_calls(), t::Le(kHandoffTicks + kHaltTicks +
                                             kQuietTicks + 2 * kResetTicks));
}

TEST_F(XhciResetTest, LongLabelsAreShortenedInLogLines) {
  XhciSimConfig config;
  config.all_ones = true;
  Start(config);
  controller_.Init(platform_, *sim_, "0000:00:14.0 and more");
  EXPECT_THAT(std::string(controller_.label()), t::Eq("0000:00:14.0 an"));
  EXPECT_THAT(controller_.TakeOwnershipAndReset(),
              t::Eq(XhciStatus::kNotDecoding));
  EXPECT_THAT(platform_.logs(),
              t::ElementsAre("[USBKBD] 0000:00:14.0 an: error: controller "
                             "not responding"));
}

}  // namespace
}  // namespace protos::usb
