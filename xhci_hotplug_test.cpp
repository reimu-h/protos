#include <cstdint>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "user/xhci_controller.h"
#include "user/xhci_regs.h"
#include "xhci_sim.h"
#include "xhci_sim_usb.h"
#include "xhci_sim_world.h"

namespace protos::xhci_sim {
namespace {

namespace t = ::testing;

TEST(XhciHotplugTest, ColdPlugHotPlugDetachAndReattachReuseSlot) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));
  EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));

  // Hot-plug on port 5.
  SimUsbKeyboard& kbd1 =
      world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  EXPECT_THAT(world.platform().logs(),
              t::Contains("[USBKBD] keyboard attached: 00:04.0 port 5 (hs)"));
  EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));
  world.TypeAscii(kbd1, "a");
  EXPECT_THAT(world.platform().input(), t::Eq("a"));

  // Hot-unplug port 5: logs detached, frees slot 1.
  world.Detach(idx, 5);
  world.ServiceReady();
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(0));
  EXPECT_THAT(world.platform().logs(),
              t::Contains("[USBKBD] keyboard detached: 00:04.0 port 5"));
  EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));

  // Re-attach on port 6: reuses slot 1.
  SimUsbKeyboard& kbd2 =
      world.AttachKeyboard(idx, 6, HighSpeedKeyboardConfig());
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  EXPECT_THAT(world.platform().logs(),
              t::Contains("[USBKBD] keyboard attached: 00:04.0 port 6 (hs)"));
  EXPECT_TRUE(world.sim(idx).slot(1).enabled);
  world.TypeAscii(kbd2, "b");
  EXPECT_THAT(world.platform().input(), t::Eq("ab"));

  // Repeat attach/detach 4 more times on port 6 without leaking slots.
  for (int i = 0; i < 4; ++i) {
    world.Detach(idx, 6);
    world.ServiceReady();
    EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(0));
    SimUsbKeyboard& kbd =
        world.AttachKeyboard(idx, 6, HighSpeedKeyboardConfig());
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
    EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(1));
    world.TypeAscii(kbd, "x");
  }
  EXPECT_THAT(world.platform().input(), t::Eq("abxxxx"));
}

TEST(XhciHotplugTest, ContactBounceRestartsDebounceAndContinuousBounceCapsOut) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));

  SimUsbKeyboard& kbd = world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  world.RunForMs(60);
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));

  // Contact bounce at t = +60 ms restarts the 100 ms debounce window.
  world.sim(idx).TriggerConnectBounce(5);
  world.RunForMs(60);
  // 120 ms since first attach, but only 60 ms since bounce: still debouncing.
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(0));

  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  world.TypeAscii(kbd, "ok");
  EXPECT_THAT(world.platform().input(), t::Eq("ok"));

  // Continuous bounce on port 6 for > 2000 ms times out and parks the port.
  world.AttachKeyboard(idx, 6, HighSpeedKeyboardConfig());
  for (int i = 0; i < 40; ++i) {
    world.RunForMs(60);
    world.sim(idx).TriggerConnectBounce(6);
    world.ServiceReady();
  }
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(1));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(1));
}

TEST(XhciHotplugTest, UnplugDuringDebounceOrMidEnumerationAbortsSilently) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  const int initial_logs = static_cast<int>(world.platform().logs().size());

  // 1. Unplug during the 100 ms debounce window.
  world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  world.RunForMs(40);
  world.Detach(idx, 5);
  world.RunForMs(200);
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(0));
  EXPECT_THAT(static_cast<int>(world.platform().logs().size()),
              t::Eq(initial_logs));

  // 2. Unplug mid-enumeration (after Enable Slot + Address Device while waiting
  // for the post-address settle timer, before SET_CONFIGURATION completes).
  world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  // 100 ms debounce + 20 ms port reset + 10 ms reset recovery = ~130 ms; at
  // ~132 ms the slot is enabled and in kAddressSettle.
  ASSERT_TRUE(
      world.RunUntil([&] { return world.sim(idx).enabled_slot_count() == 1; }));
  ASSERT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));
  world.Detach(idx, 5);
  world.RunForMs(100);
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(0));
  // Neither "attached" nor "detached" was emitted for the aborted enumeration.
  EXPECT_THAT(static_cast<int>(world.platform().logs().size()),
              t::Eq(initial_logs));
}

TEST(XhciHotplugTest, EventRingFullTriggersPortRescanAndRecoversMissedAttach) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));

  // Drop the Port Status Change event when the keyboard is plugged in.
  world.sim(idx).set_drop_port_status_change_events(true);
  SimUsbKeyboard& kbd = world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  world.RunForMs(200);
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));

  // Re-enable PSC events and inject Host Controller Event (kCcEventRingFull):
  // HandleInterrupt() rescans all ports and enumerates port 5.
  world.sim(idx).set_drop_port_status_change_events(false);
  world.sim(idx).InjectEventRingFull();
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  world.TypeAscii(kbd, "z");
  EXPECT_THAT(world.platform().input(), t::Eq("z"));
}

TEST(XhciHotplugTest, SteadyStateHasNoDeadlineAndSpuriousWakesAreSafeNoOps) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbKeyboard& kbd = world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));

  EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));
  const int logs_before = static_cast<int>(world.platform().logs().size());
  for (int i = 0; i < 50; ++i) {
    world.SpuriousWakeAll();
  }
  EXPECT_THAT(static_cast<int>(world.platform().logs().size()),
              t::Eq(logs_before));
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(1));
  world.TypeAscii(kbd, "s");
  EXPECT_THAT(world.platform().input(), t::Eq("s"));
}

TEST(XhciHotplugTest,
     RecoversFromHostSystemErrorHostControllerErrorAndCommandTimeout) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbKeyboard& kbd = world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  world.TypeAscii(kbd, "1");
  EXPECT_THAT(world.platform().input(), t::Eq("1"));

  // Recovery #1: Host System Error (USBSTS.HSE).
  world.sim(idx).InjectHostSystemError();
  world.ServiceReady();
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  world.TypeAscii(kbd, "2");
  EXPECT_THAT(world.platform().input(), t::Eq("12"));

  // Recovery #2: Host Controller Error (USBSTS.HCE).
  world.sim(idx).InjectHostControllerError();
  world.ServiceReady();
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  world.TypeAscii(kbd, "3");
  EXPECT_THAT(world.platform().input(), t::Eq("123"));

  // Recovery #3: Command timeout (500 ms) -> CRCR.CA abort + controller reset.
  world.sim(idx).set_command_hang(true);
  world.AttachKeyboard(idx, 6, HighSpeedKeyboardConfig());
  // Advance past debounce + port reset so Enable Slot is posted and hangs.
  world.RunForMs(200);
  world.sim(idx).set_command_hang(false);
  // Advance past the 500 ms command timeout and wait for both keyboards to
  // re-enumerate.
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 2; }));
  world.TypeAscii(kbd, "4");
  EXPECT_THAT(world.platform().input(), t::Eq("1234"));

  // A 4th fatal error exceeds the 3-recovery cap and transitions to kError.
  world.sim(idx).InjectHostSystemError();
  world.ServiceReady();
  EXPECT_THAT(world.controller(idx).state(), t::Eq(usb::XhciState::kError));
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));
}

TEST(XhciHotplugTest,
     SixtyFiveHotPlugCyclesWrapCommandRingWithoutLeakingSlots) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));

  // Each attach+detach issues 4 commands (Enable Slot, Address Device,
  // Configure Endpoint, Disable Slot). 65 cycles = 260 commands, wrapping the
  // 256-TRB Command Ring across its Link TRB.
  for (int i = 0; i < 65; ++i) {
    SimUsbKeyboard& kbd =
        world.AttachKeyboard(idx, 1, SuperSpeedKeyboardConfig());
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
    EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(1));
    if (i == 64) {
      world.TypeAscii(kbd, "w");
    }
    world.Detach(idx, 1);
    world.ServiceReady();
    EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(0));
  }
  EXPECT_THAT(world.platform().input(), t::Eq("w"));
}

TEST(XhciHotplugTest,
     PortResetTimeoutAndSurpriseControllerRemovalHandledCleanly) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));

  // Port 5 reset never completes: times out after 3 retries without hanging.
  world.sim(idx).set_port_reset_never_completes(true);
  world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  world.RunForMs(3500);
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));

  // Attach on port 6 after restoring normal port reset behavior.
  world.sim(idx).set_port_reset_never_completes(false);
  world.AttachKeyboard(idx, 6, HighSpeedKeyboardConfig());
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));

  // Surprise PCI removal (0xFFFFFFFF on MMIO reads) during HandleInterrupt
  // detaches active keyboards and transitions to kError.
  world.sim(idx).StopDecoding();
  world.controller(idx).HandleInterrupt();
  EXPECT_THAT(world.controller(idx).state(), t::Eq(usb::XhciState::kError));
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));
  EXPECT_THAT(world.platform().logs(),
              t::Contains("[USBKBD] keyboard detached: 00:04.0 port 6"));
}

TEST(XhciHotplugTest, MultiControllerNextDeadlineAndMidEnumerationHseRecovery) {
  SimWorld world;
  const int c0 =
      world.AddController("00:04.0", XhciSimConfig{}, usb::IrqMode::kMsix);
  const int c1 =
      world.AddController("01:01.0", IntelLikeConfig(), usb::IrqMode::kMsi);
  ASSERT_THAT(world.BringUpAll(), t::Eq(usb::XhciStatus::kOk));

  // When both controllers have no attached devices, both return kNoDeadline
  // so user/usbkbd.cpp passes timeout_ticks = -1 to sys_irq_wait.
  EXPECT_THAT(world.controller(c0).NextDeadline(), t::Eq(usb::kNoDeadline));
  EXPECT_THAT(world.controller(c1).NextDeadline(), t::Eq(usb::kNoDeadline));

  // Hot-plug a keyboard on c1 while c0 remains idle: c0 still reports
  // kNoDeadline while c1 reports finite deadlines through debounce, reset,
  // reset recovery, and address settle.
  SimUsbKeyboard& kbd1 =
      world.AttachKeyboard(c1, 1, FullSpeedKeyboardConfig(64));
  world.ServiceReady();
  EXPECT_THAT(world.controller(c0).NextDeadline(), t::Eq(usb::kNoDeadline));
  EXPECT_THAT(world.controller(c1).NextDeadline(), t::Gt(world.clock().now()));
  EXPECT_THAT(world.controller(c1).NextDeadline(), t::Lt(usb::kNoDeadline));

  // Advance into mid-enumeration (after Enable Slot + Address Device, while
  // waiting for the post-address settle timer) and verify NextDeadline() is
  // still finite.
  ASSERT_TRUE(
      world.RunUntil([&] { return world.sim(c1).enabled_slot_count() == 1; }));
  EXPECT_THAT(world.controller(c1).attached_keyboard_count(), t::Eq(0));
  EXPECT_THAT(world.controller(c1).NextDeadline(), t::Lt(usb::kNoDeadline));

  // Inject Host System Error (HSE) mid-enumeration on c1: Recover() resets and
  // restarts c1 cleanly and finishes enumerating kbd1.
  world.sim(c1).InjectHostSystemError();
  world.ServiceReady();
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(c1).attached_keyboard_count() == 1; }));

  // Once enumeration completes, both controllers return kNoDeadline again.
  EXPECT_THAT(world.controller(c0).NextDeadline(), t::Eq(usb::kNoDeadline));
  EXPECT_THAT(world.controller(c1).NextDeadline(), t::Eq(usb::kNoDeadline));

  world.TypeAscii(kbd1, "m");
  EXPECT_THAT(world.platform().input(), t::Eq("m"));
}

}  // namespace
}  // namespace protos::xhci_sim
