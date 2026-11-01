#include <cstdint>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "user/usbkbd/xhci_controller.h"
#include "user/usbkbd/xhci_regs.h"
#include "user/usbkbd/xhci_sim.h"
#include "user/usbkbd/xhci_sim_usb.h"
#include "user/usbkbd/xhci_sim_world.h"

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

  // A normal command timeout (5 s) aborts only that command via CRCR.CA
  // in-place without resetting the controller or detaching port 5.
  const int kbd_resets = kbd.bus_reset_count();
  world.sim(idx).set_command_hang(true);
  world.AttachKeyboard(idx, 6, HighSpeedKeyboardConfig());
  // Advance past debounce + port reset so Enable Slot is posted and hangs,
  // then past the 5 s command timeout so CRCR.CA aborts it.
  world.RunForMs(5500);
  world.sim(idx).set_command_hang(false);
  // Port 6 retries after the aborted Enable Slot and attaches, while port 5
  // was never reset.
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 2; },
      2000));
  EXPECT_THAT(kbd.bus_reset_count(), t::Eq(kbd_resets));
  world.TypeAscii(kbd, "4");
  EXPECT_THAT(world.platform().input(), t::Eq("1234"));

  // Recovery #3: Command abort timeout (command hangs for 5 s AND CRCR.CA
  // hangs for 5 s) triggers a full controller Recover().
  world.Detach(idx, 6);
  world.ServiceReady();
  world.sim(idx).set_command_hang(true);
  world.sim(idx).set_command_abort_hangs(true);
  world.AttachKeyboard(idx, 6, HighSpeedKeyboardConfig());
  world.RunForMs(5500);  // Past 5 s command timeout into CRCR.CA wait
  world.sim(idx).set_command_hang(false);
  world.RunForMs(5500);  // Past 5 s command abort timeout -> Recover()
  world.sim(idx).set_command_abort_hangs(false);
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 2; },
      3000));
  EXPECT_THAT(
      world.platform().logs(),
      t::Contains("[USBKBD] 00:04.0: controller error (command abort timeout), "
                  "recovering"));
  world.TypeAscii(kbd, "5");
  EXPECT_THAT(world.platform().input(), t::Eq("12345"));

  // After 60 s of healthy operation, the 3-recovery window refills so a later
  // HSE still recovers cleanly.
  world.RunForMs(61000);
  world.sim(idx).InjectHostSystemError();
  world.ServiceReady();
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 2; }));

  // 3 more rapid fatal errors within the same 60 s window (4 total in window)
  // exceed the 3-recovery cap and transition to kError.
  for (int r = 0; r < 2; ++r) {
    world.sim(idx).InjectHostSystemError();
    world.ServiceReady();
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 2; }));
  }
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
  // so user/usbkbd/usbkbd.cpp passes timeout_ticks = -1 to sys_irq_wait.
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

TEST(XhciHotplugTest,
     DownstreamHubPortHotPlugUnplugAndReattachReuseSlotAndStayIdle) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbHub& hub =
      world.AttachHub(idx, 5, Usb2HighSpeedSingleTtHubConfig(/*num_ports=*/4));

  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil([&] {
    return world.sim(idx).enabled_slot_count() == 1 &&
           world.controller(idx).NextDeadline() == usb::kNoDeadline;
  }));
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));

  // Hot-plug a Full-Speed keyboard onto hub port 2 after the hub is already
  // configured and idle.
  SimUsbKeyboard& kbd1 =
      world.AttachKeyboardToHub(hub, 2, FullSpeedKeyboardConfig(64));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(2));
  EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));
  world.TypeAscii(kbd1, "a");
  EXPECT_THAT(world.platform().input(), t::Eq("a"));

  // Detach kbd1 from hub port 2: keyboard slot is disabled while the hub slot
  // stays enabled and idle.
  hub.DetachDevice(2);
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 0; }));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(1));
  EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));
  EXPECT_THAT(world.platform().logs(),
              t::Contains("[USBKBD] keyboard detached: 00:04.0 port 5.2"));

  // Run 5 hot-plug / detach cycles across hub ports 1..4 and verify zero slot
  // leaks.
  for (int cycle = 0; cycle < 5; ++cycle) {
    const int hp = (cycle % 4) + 1;
    SimUsbKeyboard& kbd =
        world.AttachKeyboardToHub(hub, hp, LowSpeedKeyboardConfig());
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
    EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(2));
    world.TypeAscii(kbd, "x");
    hub.DetachDevice(hp);
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 0; }));
    EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(1));
  }
  EXPECT_THAT(world.platform().input(), t::Eq("axxxxx"));
}

TEST(XhciHotplugTest,
     DetachingRootOrIntermediateHubRecursivelyTearsDownAllDescendantSlots) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbHub& hub_a =
      world.AttachHub(idx, 5, Usb2HighSpeedMultiTtHubConfig(/*num_ports=*/4));
  SimUsbKeyboard& kbd1 =
      world.AttachKeyboardToHub(hub_a, 1, FullSpeedKeyboardConfig(64));
  SimUsbHub& hub_b =
      world.AttachHubToHub(hub_a, 2, Usb2FullSpeedHubConfig(/*num_ports=*/4));
  SimUsbKeyboard& kbd2 =
      world.AttachKeyboardToHub(hub_b, 1, LowSpeedKeyboardConfig());
  SimUsbKeyboard& kbd3 =
      world.AttachKeyboardToHub(hub_b, 4, FullSpeedKeyboardConfig(8));

  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 3; }));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(5));

  world.TypeAscii(kbd1, "1");
  world.TypeAscii(kbd2, "2");
  world.TypeAscii(kbd3, "3");
  EXPECT_THAT(world.platform().input(), t::Eq("123"));

  // Detach intermediate hub_b from hub_a port 2: hub_b, kbd2, and kbd3 must all
  // be torn down, leaving hub_a and kbd1 active.
  hub_a.DetachDevice(2);
  ASSERT_TRUE(world.RunUntil([&] {
    return world.controller(idx).attached_keyboard_count() == 1 &&
           world.sim(idx).enabled_slot_count() == 2;
  }));
  EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));
  world.TypeAscii(kbd1, "!");
  EXPECT_THAT(world.platform().input(), t::Eq("123!"));

  // Re-attach hub_b (which still has kbd2 and kbd3 attached to its ports 1 and
  // 4) to hub_a port 2 and wait for all 3 keyboards to be active again.
  hub_a.AttachDevice(2, &hub_b);
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 3; }));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(5));

  // Detach root port 5: hub_a, hub_b, kbd1, kbd2, and kbd3 must all be torn
  // down cleanly with zero enabled slots remaining.
  world.Detach(idx, 5);
  world.ServiceReady();
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(0));
  EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));
}

TEST(XhciHotplugTest,
     HubAndDownstreamKeyboardsRecoverCleanlyAcrossHostSystemError) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbHub& hub =
      world.AttachHub(idx, 5, Usb2HighSpeedSingleTtHubConfig(/*num_ports=*/4));
  SimUsbKeyboard& kbd1 =
      world.AttachKeyboardToHub(hub, 1, FullSpeedKeyboardConfig(64));
  SimUsbKeyboard& kbd2 =
      world.AttachKeyboardToHub(hub, 3, LowSpeedKeyboardConfig());

  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 2; }));

  world.TypeAscii(kbd1, "a");
  world.TypeAscii(kbd2, "b");
  EXPECT_THAT(world.platform().input(), t::Eq("ab"));

  world.sim(idx).InjectHostSystemError();
  world.ServiceReady();
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 2; }));
  EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));

  world.TypeAscii(kbd1, "c");
  world.TypeAscii(kbd2, "d");
  EXPECT_THAT(world.platform().input(), t::Eq("abcd"));
}

static int CountLogLines(const SimWorld& world, const std::string& line) {
  int count = 0;
  for (const std::string& logged : world.platform().logs()) {
    if (logged == line) {
      ++count;
    }
  }
  return count;
}

// Waits until controller `idx` of `world` enumerated the hub(s) and is idle.
static bool RunUntilSlotsIdle(SimWorld& world, const int idx, const int slots) {
  return world.RunUntil([&] {
    return world.sim(idx).enabled_slot_count() == slots &&
           world.controller(idx).NextDeadline() == usb::kNoDeadline;
  });
}

TEST(XhciHotplugTest,
     LevelTriggeredHubTransitionsCostOneReportPerChangeAndQuiesce) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbHub& hub = world.AttachHub(idx, 5, Usb2HighSpeedSingleTtHubConfig(4));
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(RunUntilSlotsIdle(world, idx, 1));

  // A plug latches C_PORT_CONNECTION and later C_PORT_RESET: each change is
  // reported once, because the driver clears it before re-arming.
  const int reports0 = hub.intr_data_count();
  const int port_reads0 = hub.get_port_status_count();
  SimUsbKeyboard& kbd =
      world.AttachKeyboardToHub(hub, 2, FullSpeedKeyboardConfig(64));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  ASSERT_TRUE(world.RunUntilIdle(idx));
  EXPECT_THAT(hub.intr_data_count() - reports0, t::Le(2));
  EXPECT_THAT(hub.get_port_status_count() - port_reads0, t::Le(4));

  // Idle: no report and no driver wakeup.
  const int reports1 = hub.intr_data_count();
  const int64_t wakeups1 = world.wakeups();
  world.RunForMs(2000);
  EXPECT_THAT(hub.intr_data_count(), t::Eq(reports1));
  EXPECT_THAT(world.wakeups(), t::Eq(wakeups1));
  world.TypeAscii(kbd, "q");

  // An unplug latches C_PORT_CONNECTION once.
  const int reports2 = hub.intr_data_count();
  hub.DetachDevice(2);
  ASSERT_TRUE(world.RunUntil([&] {
    return world.controller(idx).attached_keyboard_count() == 0 &&
           world.sim(idx).enabled_slot_count() == 1;
  }));
  ASSERT_TRUE(world.RunUntilIdle(idx));
  EXPECT_THAT(hub.intr_data_count() - reports2, t::Le(1));
  EXPECT_THAT(hub.redundant_status_reports(), t::Eq(0));
  EXPECT_THAT(world.platform().input(), t::Eq("q"));
}

TEST(XhciHotplugTest,
     SimultaneousAttachOnThreeChainedHubsKeepsOneDeviceAtDefaultAddress) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbHub& hub_a = world.AttachHub(idx, 5, Usb2HighSpeedSingleTtHubConfig(4));
  SimUsbHub& hub_b =
      world.AttachHubToHub(hub_a, 1, Usb2HighSpeedSingleTtHubConfig(4));
  SimUsbHub& hub_c = world.AttachHubToHub(hub_b, 1, Usb2FullSpeedHubConfig(4));
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(RunUntilSlotsIdle(world, idx, 3));

  // Three keyboards plugged at the same instant into three hubs of one USB
  // 2.0 segment debounce together; only one may be at address 0 at a time.
  SimUsbKeyboard& k1 =
      world.AttachKeyboardToHub(hub_a, 2, HighSpeedKeyboardConfig());
  SimUsbKeyboard& k2 =
      world.AttachKeyboardToHub(hub_b, 3, FullSpeedKeyboardConfig(64));
  SimUsbKeyboard& k3 =
      world.AttachKeyboardToHub(hub_c, 4, LowSpeedKeyboardConfig());
  ASSERT_TRUE(world.RunUntil([&] {
    return world.controller(idx).attached_keyboard_count() == 3 &&
           world.controller(idx).NextDeadline() == usb::kNoDeadline;
  }));
  EXPECT_THAT(world.sim(idx).max_default_state_devices(), t::Eq(1));
  EXPECT_THAT(
      world.sim(idx).CountViolations(Violation::kDefaultAddressCollision),
      t::Eq(0));

  const int hub_b_slot = world.FindSlot(idx, 5, 0x1u);
  ASSERT_THAT(hub_b_slot, t::Gt(0));
  struct Expected {
    uint32_t route;
    int tt_hub_slot;
    int tt_port;
  };
  const Expected expected[] = {
      {0x2u, 0, 0},             // High-Speed keyboard: no TT
      {0x31u, hub_b_slot, 3},   // Full-Speed keyboard behind hub_b
      {0x411u, hub_b_slot, 1},  // Low-Speed keyboard: TT through hub_c
  };
  for (const Expected& e : expected) {
    SCOPED_TRACE("route " + std::to_string(e.route));
    const int s = world.FindSlot(idx, 5, e.route);
    ASSERT_THAT(s, t::Gt(0));
    EXPECT_THAT(world.sim(idx).slot(s).parent_hub_slot_id,
                t::Eq(e.tt_hub_slot));
    EXPECT_THAT(world.sim(idx).slot(s).parent_port_number, t::Eq(e.tt_port));
  }
  world.TypeAscii(k1, "1");
  world.TypeAscii(k2, "2");
  world.TypeAscii(k3, "3");
  EXPECT_THAT(world.platform().input(), t::Eq("123"));
}

// GET_PORT_STATUS requests to every port of `hub` except `a` and `b`.
static int PortStatusReadsExcept(const SimUsbHub& hub,  //
                                 const int a,           //
                                 const int b) {
  int reads = 0;
  for (int p = 1; p <= hub.num_ports(); ++p) {
    if (p != a && p != b) {
      reads += hub.get_port_status_count(p);
    }
  }
  return reads;
}

TEST(XhciHotplugTest, FifteenPortHubReportsChangesAcrossBitmapBytes) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbHub& hub = world.AttachHub(idx, 5, Usb2HighSpeedSingleTtHubConfig(15));
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(RunUntilSlotsIdle(world, idx, 1));
  EXPECT_THAT(world.sim(idx).slot(1).num_ports, t::Eq(15));

  // Ports 3 and 12 change in one report (bitmap bytes 0 and 1); only they
  // are read.
  const int other_reads0 = PortStatusReadsExcept(hub, 3, 12);
  SimUsbKeyboard& k3 =
      world.AttachKeyboardToHub(hub, 3, FullSpeedKeyboardConfig(64));
  SimUsbKeyboard& k12 =
      world.AttachKeyboardToHub(hub, 12, LowSpeedKeyboardConfig());
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 2; }));
  ASSERT_TRUE(world.RunUntilIdle(idx));
  EXPECT_THAT(PortStatusReadsExcept(hub, 3, 12), t::Eq(other_reads0));

  const int other_reads1 = PortStatusReadsExcept(hub, 8, 15);
  SimUsbKeyboard& k8 =
      world.AttachKeyboardToHub(hub, 8, HighSpeedKeyboardConfig());
  SimUsbKeyboard& k15 =
      world.AttachKeyboardToHub(hub, 15, FullSpeedKeyboardConfig(8));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 4; }));
  ASSERT_TRUE(world.RunUntilIdle(idx));
  EXPECT_THAT(PortStatusReadsExcept(hub, 8, 15), t::Eq(other_reads1));

  EXPECT_THAT(world.FindSlot(idx, 5, 0x3u), t::Gt(0));
  EXPECT_THAT(world.FindSlot(idx, 5, 0xCu), t::Gt(0));
  EXPECT_THAT(world.FindSlot(idx, 5, 0x8u), t::Gt(0));
  EXPECT_THAT(world.FindSlot(idx, 5, 0xFu), t::Gt(0));
  world.TypeAscii(k3, "a");
  world.TypeAscii(k12, "b");
  world.TypeAscii(k8, "c");
  world.TypeAscii(k15, "d");
  EXPECT_THAT(world.platform().input(), t::Eq("abcd"));

  hub.DetachDevice(12);
  hub.DetachDevice(8);
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 2; }));
  ASSERT_TRUE(world.RunUntilIdle(idx));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(3));
}

TEST(XhciHotplugTest,
     HubStatusChangeEndpointErrorsRecoverInPlaceThenReenumerateHub) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbHub& hub = world.AttachHub(idx, 5, Usb2HighSpeedSingleTtHubConfig(4));
  SimUsbKeyboard& k1 =
      world.AttachKeyboardToHub(hub, 1, FullSpeedKeyboardConfig(64));
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  ASSERT_TRUE(world.RunUntilIdle(idx));

  // Transient errors, one at a time, recover the endpoint in place: the hub
  // is not reset, so neither is the keyboard behind it.
  const int k1_resets = k1.bus_reset_count();
  const std::string recovering =
      "[USBKBD] 00:04.0: port 5: interrupt endpoint error, recovering";
  hub.InjectInterruptError(IntrOutcome::kTransactionError, 1);
  world.RunForMs(200);
  hub.InjectInterruptError(IntrOutcome::kStall, 1);
  world.RunForMs(200);
  hub.InjectInterruptError(IntrOutcome::kBabble, 1);
  world.RunForMs(200);
  ASSERT_TRUE(world.RunUntilIdle(idx));
  EXPECT_THAT(CountLogLines(world, recovering), t::Eq(3));
  EXPECT_THAT(k1.bus_reset_count(), t::Eq(k1_resets));
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(1));

  // The status-change pipe still reports hot-plugs.
  SimUsbKeyboard& k3 =
      world.AttachKeyboardToHub(hub, 3, LowSpeedKeyboardConfig());
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 2; }));
  world.TypeAscii(k1, "1");
  world.TypeAscii(k3, "3");
  EXPECT_THAT(world.platform().input(), t::Eq("13"));

  // Persistent errors: the hub is re-enumerated and both keyboards return.
  hub.InjectInterruptError(IntrOutcome::kTransactionError, 4);
  world.RunForMs(200);
  ASSERT_TRUE(world.RunUntil(
      [&] {
        return world.controller(idx).attached_keyboard_count() == 2 &&
               world.sim(idx).enabled_slot_count() == 3 &&
               world.controller(idx).NextDeadline() == usb::kNoDeadline;
      },
      5000));
  EXPECT_THAT(CountLogLines(world,
                            "[USBKBD] 00:04.0: port 5: interrupt endpoint "
                            "error limit (attempt 1)"),
              t::Eq(1));
  world.TypeAscii(k1, "x");
  world.TypeAscii(k3, "y");
  EXPECT_THAT(world.platform().input(), t::Eq("13xy"));
}

TEST(XhciHotplugTest, HubPortResetThatNeverCompletesGivesUpAfterThreeAttempts) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimHubConfig cfg = Usb2HighSpeedSingleTtHubConfig(4);
  cfg.port_reset_never_completes = true;
  SimUsbHub& hub = world.AttachHub(idx, 5, cfg);
  world.AttachKeyboardToHub(hub, 2, HighSpeedKeyboardConfig());
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  world.RunForMs(8000);
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(1));
  EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));
  EXPECT_THAT(hub.port_reset_count(2), t::Eq(3));
  EXPECT_THAT(
      CountLogLines(
          world,
          "[USBKBD] 00:04.0: port 5.2: hub port reset timeout (attempt 3)"),
      t::Eq(1));
}

TEST(XhciHotplugTest, FastKeyboardSwapOnHubPortReenumeratesTheNewDevice) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbHub& hub = world.AttachHub(idx, 5, Usb2HighSpeedSingleTtHubConfig(4));
  world.AttachKeyboardToHub(hub, 2, FullSpeedKeyboardConfig(64));
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  ASSERT_TRUE(world.RunUntilIdle(idx));
  const int fs_slot = world.FindSlot(idx, 5, 0x2u);
  ASSERT_THAT(fs_slot, t::Gt(0));
  EXPECT_THAT(world.sim(idx).slot(fs_slot).speed, t::Eq(1));

  // Swapped faster than the driver polls: the port stays connected, but
  // C_PORT_CONNECTION is set.
  hub.DetachDevice(2);
  SimUsbKeyboard& ls =
      world.AttachKeyboardToHub(hub, 2, LowSpeedKeyboardConfig());
  ASSERT_TRUE(world.RunUntil([&] {
    const int s = world.FindSlot(idx, 5, 0x2u);
    return s > 0 && world.sim(idx).slot(s).speed == 2 &&
           world.controller(idx).attached_keyboard_count() == 1 &&
           world.controller(idx).NextDeadline() == usb::kNoDeadline;
  }));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(2));
  EXPECT_THAT(world.platform().logs(),
              t::Contains("[USBKBD] keyboard attached: 00:04.0 port 5.2 (ls)"));
  world.TypeAscii(ls, "n");
  EXPECT_THAT(world.platform().input(), t::Eq("n"));
}

TEST(XhciHotplugTest, TransientHubPortResetStallRetriesOnlyThatPort) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbHub& hub = world.AttachHub(idx, 5, Usb2HighSpeedSingleTtHubConfig(4));
  SimUsbKeyboard& k1 =
      world.AttachKeyboardToHub(hub, 1, FullSpeedKeyboardConfig(64));
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  ASSERT_TRUE(world.RunUntilIdle(idx));
  world.TypeAscii(k1, "a");

  // One SET_FEATURE(PORT_RESET) STALLs while ports 2 and 3 both wait for
  // their reset: only port 2 is retried, quickly, and port 1 is untouched.
  const int logs0 = static_cast<int>(world.platform().logs().size());
  hub.ArmPortResetFailures(1);
  SimUsbKeyboard& k2 =
      world.AttachKeyboardToHub(hub, 2, HighSpeedKeyboardConfig());
  SimUsbKeyboard& k3 =
      world.AttachKeyboardToHub(hub, 3, LowSpeedKeyboardConfig());
  const int64_t t0 = world.clock().now();
  ASSERT_TRUE(world.RunUntil(
      [&] {
        return world.controller(idx).attached_keyboard_count() == 3 &&
               world.controller(idx).NextDeadline() == usb::kNoDeadline;
      },
      5000));
  const int64_t took_ms = (world.clock().now() - t0) * 4;  // 250 Hz ticks
  bool k1_detached = false;
  const std::vector<std::string>& logs = world.platform().logs();
  for (int i = logs0; i < static_cast<int>(logs.size()); ++i) {
    if (logs[static_cast<size_t>(i)] ==
        "[USBKBD] keyboard detached: 00:04.0 port 5.1") {
      k1_detached = true;
    }
  }
  EXPECT_FALSE(k1_detached);
  EXPECT_THAT(took_ms, t::Lt(2000));
  EXPECT_THAT(CountLogLines(world,
                            "[USBKBD] 00:04.0: port 5.2: hub port control "
                            "transfer error (attempt 1)"),
              t::Eq(1));
  EXPECT_THAT(
      logs, t::Not(t::Contains(t::HasSubstr("hub control transfer timeout"))));
  world.TypeAscii(k1, "b");
  world.TypeAscii(k2, "c");
  world.TypeAscii(k3, "d");
  EXPECT_THAT(world.platform().input(), t::Eq("abcd"));
}

TEST(XhciHotplugTest,
     HubControlTransferTimeoutRecoversWithoutHubReenumeration) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  // The hub never answers its first GET_PORT_STATUS (5 s control timeout).
  SimHubConfig cfg = Usb2HighSpeedSingleTtHubConfig(4);
  cfg.fail_get_port_status_attempts = 1;
  cfg.injected_ctrl_error = ControlOutcome::kNoResponse;
  SimUsbHub& hub = world.AttachHub(idx, 5, cfg);
  SimUsbKeyboard& kbd =
      world.AttachKeyboardToHub(hub, 2, HighSpeedKeyboardConfig());
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil([&] { return hub.configured(); }));
  const int hub_resets = hub.bus_reset_count();
  const int stops0 = world.sim(idx).stop_endpoint_count();

  // EP0 is still Running with the stuck TD: Stop Endpoint and Set TR Dequeue
  // recover it, and the GET_PORT_STATUS is retried; the hub is kept.
  ASSERT_TRUE(world.RunUntil(
      [&] {
        return world.controller(idx).attached_keyboard_count() == 1 &&
               world.controller(idx).NextDeadline() == usb::kNoDeadline;
      },
      6000));
  EXPECT_THAT(hub.bus_reset_count(), t::Eq(hub_resets));
  EXPECT_THAT(world.sim(idx).stop_endpoint_count() - stops0, t::Eq(1));
  EXPECT_THAT(
      world.platform().logs(),
      t::Not(t::Contains(t::HasSubstr("ep0 set dequeue command error"))));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(2));
  world.TypeAscii(kbd, "t");
  EXPECT_THAT(world.platform().input(), t::Eq("t"));
}

TEST(XhciHotplugTest, HubStatusChangePipeSurvivesEventRingFull) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbHub& hub = world.AttachHub(idx, 5, Usb2HighSpeedSingleTtHubConfig(4));
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(RunUntilSlotsIdle(world, idx, 1));

  // Fill the event ring with Port Status Change events of an empty root port
  // while the driver does not run.
  for (int i = 0; i < 300; ++i) {
    world.sim(idx).TriggerConnectBounce(1);
  }
  // The hub reports a keyboard on port 2 while the ring is full, so its
  // Transfer Event is dropped (xHCI 4.9.4).
  SimUsbKeyboard& k2 =
      world.AttachKeyboardToHub(hub, 2, FullSpeedKeyboardConfig(64));
  world.sim(idx).Step();
  // The driver drains the ring, sees Event Ring Full and rescans.
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  ASSERT_TRUE(world.RunUntilIdle(idx));
  world.TypeAscii(k2, "e");

  // A later hot-plug and a later unplug on the hub are still noticed.
  SimUsbKeyboard& k3 =
      world.AttachKeyboardToHub(hub, 3, LowSpeedKeyboardConfig());
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 2; }));
  world.TypeAscii(k3, "f");
  hub.DetachDevice(2);
  EXPECT_TRUE(world.RunUntil([&] {
    return world.controller(idx).attached_keyboard_count() == 1 &&
           world.sim(idx).enabled_slot_count() == 2;
  }));
  EXPECT_THAT(world.platform().input(), t::Eq("ef"));
}

TEST(XhciHotplugTest, KeyboardReportPipeSurvivesEventRingFull) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbKeyboard& kbd = world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  ASSERT_TRUE(world.RunUntilIdle(idx));

  // Fill the event ring with Port Status Change events of an empty root port
  // while the driver does not run. The keyboard then completes every report
  // TRB that the driver armed, and their Transfer Events are dropped (xHCI
  // 4.9.4).
  for (int i = 0; i < 300; ++i) {
    world.sim(idx).TriggerConnectBounce(1);
  }
  for (int i = 0; i < 5; ++i) {
    kbd.TapKey(usb::kHidUsageA);
    world.sim(idx).Step();
  }
  // The driver drains the ring, sees Event Ring Full and re-arms the
  // keyboard's interrupt endpoint, which would stay deaf otherwise.
  world.ServiceReady();
  ASSERT_TRUE(world.RunUntilIdle(idx));
  world.platform().ClearInput();
  world.TypeAscii(kbd, "g");
  EXPECT_THAT(world.platform().input(), t::Eq("g"));
}

TEST(XhciHotplugTest, PersistentHubPortStatusErrorsEndQuiescent) {
  SimWorld world;
  // The hub's C_PORT_CONNECTION can never be cleared, so it re-reports it.
  world.AllowRedundantHubReports();
  const int idx = world.AddController("00:04.0");
  SimHubConfig cfg = Usb2HighSpeedSingleTtHubConfig(4);
  cfg.fail_get_port_status_attempts = 1000000;
  SimUsbHub& hub = world.AttachHub(idx, 5, cfg);
  world.AttachKeyboardToHub(hub, 2, HighSpeedKeyboardConfig());
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  world.RunForMs(20000);

  // The driver has given up: no deadline, no traffic, no wakeups.
  const int port_reads0 = hub.get_port_status_count();
  const int64_t wakeups0 = world.wakeups();
  world.RunForMs(5000);
  EXPECT_THAT(world.controller(idx).state(), t::Eq(usb::XhciState::kRunning));
  EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));
  EXPECT_THAT(hub.get_port_status_count(), t::Eq(port_reads0));
  EXPECT_THAT(world.wakeups(), t::Eq(wakeups0));
  // The hub's failure budget re-enumerated it through root port 5 until that
  // port's attempts ran out: the hub ends disabled.
  EXPECT_THAT(
      CountLogLines(
          world,
          "[USBKBD] 00:04.0: port 5: hub control error limit (attempt 3)"),
      t::Eq(1));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(0));
}

TEST(XhciHotplugTest, HubPortDeadlinesWaitWhileTheHubRecoversEndpointZero) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  // The hub STALLs its first CLEAR_FEATURE(C_PORT_*): EP0 needs recovery.
  SimHubConfig cfg = Usb2HighSpeedSingleTtHubConfig(4);
  cfg.fail_clear_port_feature_attempts = 1;
  cfg.injected_ctrl_error = ControlOutcome::kStall;
  SimUsbHub& hub = world.AttachHub(idx, 5, cfg);
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(RunUntilSlotsIdle(world, idx, 1));

  // The plug starts port 2's debounce; the failed CLEAR_FEATURE then puts
  // the hub into EP0 recovery, whose Reset Endpoint command hangs.
  world.sim(idx).set_command_hang(true);
  SimUsbKeyboard& kbd =
      world.AttachKeyboardToHub(hub, 2, FullSpeedKeyboardConfig(64));
  world.ServiceReady();
  // The debounce expires while the hub cannot be serviced: the next deadline
  // stays in the future (the command timeout) and the driver sleeps.
  const int64_t wakeups0 = world.wakeups();
  world.RunForMs(300);
  EXPECT_THAT(world.controller(idx).NextDeadline(), t::Gt(world.clock().now()));
  EXPECT_THAT(world.wakeups() - wakeups0, t::Le(2));

  // The command timeout recovers the controller and the keyboard attaches.
  world.sim(idx).set_command_hang(false);
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  world.TypeAscii(kbd, "r");
  EXPECT_THAT(world.platform().input(), t::Eq("r"));
}

TEST(XhciHotplugTest,
     DefaultAddressWindowIsReleasedWhenTheResetDeviceIsUnplugged) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbHub& hub_a = world.AttachHub(idx, 5, Usb2HighSpeedSingleTtHubConfig(4));
  SimUsbHub& hub_b =
      world.AttachHubToHub(hub_a, 1, Usb2HighSpeedSingleTtHubConfig(4));
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(RunUntilSlotsIdle(world, idx, 2));

  SimUsbKeyboard& k1 =
      world.AttachKeyboardToHub(hub_a, 2, HighSpeedKeyboardConfig());
  SimUsbKeyboard& k2 =
      world.AttachKeyboardToHub(hub_b, 3, HighSpeedKeyboardConfig());
  // Unplug whichever keyboard owns the default address 0.
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.sim(idx).default_state_devices(5) == 1; }));
  const bool k1_first =
      k1.address() == 0 && (hub_a.port(2).status & 0x0002u) != 0;
  if (k1_first) {
    hub_a.DetachDevice(2);
  } else {
    hub_b.DetachDevice(3);
  }
  ASSERT_TRUE(world.RunUntil(
      [&] {
        return world.controller(idx).attached_keyboard_count() == 1 &&
               world.controller(idx).NextDeadline() == usb::kNoDeadline;
      },
      1000));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(3));
  world.TypeAscii(k1_first ? k2 : k1, "s");
  EXPECT_THAT(world.platform().input(), t::Eq("s"));
  EXPECT_THAT(
      world.sim(idx).CountViolations(Violation::kDefaultAddressCollision),
      t::Eq(0));
}

TEST(XhciHotplugTest, SpuriousStoppedEventOnHubStatusPipeIsHarmless) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbHub& hub = world.AttachHub(idx, 5, Usb2HighSpeedSingleTtHubConfig(4));
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(RunUntilSlotsIdle(world, idx, 1));
  usb::Trb ev{};
  ev.dw2 = static_cast<uint32_t>(usb::kCcStopped) << 24;
  ev.dw3 =
      (static_cast<uint32_t>(usb::kTrbTransferEvent) << usb::kTrbTypeShift) |
      (3u << usb::kTrbEndpointIdShift) | (1u << usb::kTrbSlotIdShift);
  world.sim(idx).PushRawEvent(ev);
  world.ServiceReady();
  SimUsbKeyboard& kbd =
      world.AttachKeyboardToHub(hub, 3, HighSpeedKeyboardConfig());
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  world.TypeAscii(kbd, "p");
  EXPECT_THAT(world.platform().input(), t::Eq("p"));
}

TEST(XhciHotplugTest, HubPortUnplugAtEveryEnumerationStageIsClean) {
  for (int stop = 0; stop <= 70; ++stop) {
    SCOPED_TRACE("stop_tick=" + std::to_string(stop));
    SimWorld world;
    const int idx = world.AddController("00:04.0");
    SimUsbHub& hub = world.AttachHub(idx, 5, Usb2HighSpeedSingleTtHubConfig(4));
    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    ASSERT_TRUE(RunUntilSlotsIdle(world, idx, 1));
    world.AttachKeyboardToHub(hub, 2, FullSpeedKeyboardConfig(64));
    world.RunForTicks(stop);
    hub.DetachDevice(2);
    world.RunForMs(3000);
    EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));
    EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(1));
    EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));
    SimUsbKeyboard& kbd =
        world.AttachKeyboardToHub(hub, 2, LowSpeedKeyboardConfig());
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
    world.TypeAscii(kbd, "z");
    EXPECT_THAT(world.platform().input(), t::EndsWith("z"));
  }
}

TEST(XhciHotplugTest, HubUnplugDuringChildEnumerationThenReplugIsClean) {
  for (int stop = 0; stop <= 70; stop += 2) {
    SCOPED_TRACE("stop_tick=" + std::to_string(stop));
    SimWorld world;
    const int idx = world.AddController("00:04.0");
    SimUsbHub& hub = world.AttachHub(idx, 5, Usb2HighSpeedMultiTtHubConfig(4));
    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    ASSERT_TRUE(RunUntilSlotsIdle(world, idx, 1));
    SimUsbKeyboard& kbd =
        world.AttachKeyboardToHub(hub, 2, LowSpeedKeyboardConfig());
    world.RunForTicks(stop);
    world.Detach(idx, 5);
    world.RunForMs(1000);
    EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));
    EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(0));
    EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));
    world.sim(idx).AttachDevice(5, &hub);
    ASSERT_TRUE(world.RunUntil([&] {
      return world.controller(idx).attached_keyboard_count() == 1 &&
             world.controller(idx).NextDeadline() == usb::kNoDeadline;
    }));
    EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(2));
    world.TypeAscii(kbd, "y");
    EXPECT_THAT(world.platform().input(), t::EndsWith("y"));
  }
}

TEST(XhciHotplugTest, SuperSpeedHubDownstreamHotPlugUnplugReusesSlots) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbHub& hub = world.AttachHub(idx, 1, Usb3SuperSpeedHubConfig(4));
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(RunUntilSlotsIdle(world, idx, 1));
  for (int cycle = 0; cycle < 4; ++cycle) {
    const int p = 4 - cycle;
    SimUsbKeyboard& kbd =
        world.AttachKeyboardToHub(hub, p, SuperSpeedKeyboardConfig());
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
    EXPECT_THAT(world.FindSlot(idx, 1, static_cast<uint32_t>(p)), t::Gt(0));
    world.TypeAscii(kbd, "w");
    hub.DetachDevice(p);
    ASSERT_TRUE(world.RunUntil([&] {
      return world.controller(idx).attached_keyboard_count() == 0 &&
             world.sim(idx).enabled_slot_count() == 1 &&
             world.controller(idx).NextDeadline() == usb::kNoDeadline;
    }));
  }
  EXPECT_THAT(world.platform().input(), t::Eq("wwww"));
}

TEST(XhciHotplugTest, HubDetachDisablesChildSlotsBeforeTheHubSlot) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbHub& hub = world.AttachHub(idx, 5, Usb2HighSpeedSingleTtHubConfig(4));
  world.AttachKeyboardToHub(hub, 1, LowSpeedKeyboardConfig());
  world.AttachKeyboardToHub(hub, 2, FullSpeedKeyboardConfig(64));
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil([&] {
    return world.controller(idx).attached_keyboard_count() == 2 &&
           world.controller(idx).NextDeadline() == usb::kNoDeadline;
  }));
  const int hub_slot = world.FindSlot(idx, 5, 0x0u);
  ASSERT_THAT(hub_slot, t::Gt(0));
  const int disabled0 =
      static_cast<int>(world.sim(idx).disable_slot_log().size());

  // The keyboards' slots (whose TT is the hub's) go before the hub's slot.
  world.Detach(idx, 5);
  world.ServiceReady();
  const std::vector<int>& disabled = world.sim(idx).disable_slot_log();
  ASSERT_THAT(static_cast<int>(disabled.size()) - disabled0, t::Eq(3));
  EXPECT_THAT(disabled.back(), t::Eq(hub_slot));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(0));
}

TEST(XhciHotplugTest,
     HubStatusChangeThatCannotBeClearedIsIgnoredAfterThreeTries) {
  SimWorld world;
  // The hub keeps reporting the change it cannot clear.
  world.AllowRedundantHubReports();
  const int idx = world.AddController("00:04.0");
  SimHubConfig cfg = Usb2HighSpeedSingleTtHubConfig(4);
  cfg.fail_clear_hub_feature_attempts = 1000000;
  cfg.injected_ctrl_error = ControlOutcome::kStall;
  SimUsbHub& hub = world.AttachHub(idx, 5, cfg);
  SimUsbKeyboard& k1 =
      world.AttachKeyboardToHub(hub, 1, FullSpeedKeyboardConfig(64));
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil([&] {
    return world.controller(idx).attached_keyboard_count() == 1 &&
           world.controller(idx).NextDeadline() == usb::kNoDeadline;
  }));

  // C_HUB_LOCAL_POWER cannot be cleared: after three CLEAR_HUB_FEATURE
  // attempts the driver ignores the hub change instead of dropping the hub.
  hub.InjectHubChange(0x0001);
  world.RunForMs(500);
  ASSERT_TRUE(world.RunUntilIdle(idx));
  EXPECT_THAT(hub.clear_hub_feature_count(), t::Eq(3));
  EXPECT_THAT(CountLogLines(world,
                            "[USBKBD] 00:04.0: port 5: hub status change not "
                            "acknowledged, ignoring it"),
              t::Eq(1));
  const int port_reads = hub.get_port_status_count();
  world.RunForMs(1000);
  EXPECT_THAT(hub.clear_hub_feature_count(), t::Eq(3));
  EXPECT_THAT(hub.get_port_status_count(), t::Eq(port_reads));

  // The keyboards behind the hub keep working, including a hot-plugged one.
  SimUsbKeyboard& k2 =
      world.AttachKeyboardToHub(hub, 2, LowSpeedKeyboardConfig());
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 2; }));
  world.TypeAscii(k1, "g");
  world.TypeAscii(k2, "h");
  EXPECT_THAT(world.platform().input(), t::Eq("gh"));
}

TEST(XhciHotplugTest,
     ColdBootSilentConnectAfterPortPowerOnZeroPwrGoodHubEnumeratesKeyboard) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimHubConfig cfg = Usb2HighSpeedSingleTtHubConfig(4);
  cfg.vendor_id = 0x0bda;
  cfg.product_id = 0x5411;
  cfg.pwr_on_2_pwr_good = 0;  // Realtek RTS5411 USB 2.0 hub reports 0
  SimUsbHub& hub = world.AttachHub(idx, 5, cfg);
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));

  // Wait until the hub is configured and finishes its initial 100 ms
  // post-power port scan with all ports still disconnected.
  ASSERT_TRUE(world.RunUntil(
      [&] { return hub.configured() && hub.get_port_status_count() == 4; }));
  EXPECT_THAT(
      CountLogLines(
          world,
          "[USBKBD] hub attached: 00:04.0 port 5 (hs, 4 ports) (0bda:5411)"),
      t::Eq(1));

  // A cold-booting keyboard finishes powering up ~50 ms after the initial
  // scan and sets PORT_CONNECTION without latching C_PORT_CONNECTION on the
  // hub's interrupt pipe.
  world.RunForMs(50);
  SimUsbKeyboard kbd(FullSpeedKeyboardConfig(64));
  hub.AttachDeviceWithoutChange(2, &kbd);

  ASSERT_TRUE(world.RunUntil([&] {
    return world.controller(idx).attached_keyboard_count() == 1 &&
           world.controller(idx).NextDeadline() == usb::kNoDeadline;
  }));
  world.TypeAscii(kbd, "r");
  EXPECT_THAT(world.platform().input(), t::Eq("r"));
}

TEST(XhciHotplugTest,
     HubPortResetCompletesWhenConnectChangeOrSe0DropOccursDuringReset) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimHubConfig cfg = Usb2HighSpeedMultiTtHubConfig(4);
  cfg.port_reset_delay_polls = 2;
  cfg.port_reset_drops_connection_while_active = true;
  SimUsbHub& hub = world.AttachHub(idx, 5, cfg);
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(RunUntilSlotsIdle(world, idx, 1));

  SimUsbKeyboard& kbd =
      world.AttachKeyboardToHub(hub, 3, LowSpeedKeyboardConfig());
  ASSERT_TRUE(world.RunUntil([&] {
    return world.controller(idx).attached_keyboard_count() == 1 &&
           world.controller(idx).NextDeadline() == usb::kNoDeadline;
  }));
  EXPECT_THAT(hub.port_reset_count(3), t::Eq(1));
  world.TypeAscii(kbd, "m");
  EXPECT_THAT(world.platform().input(), t::Eq("m"));
}

TEST(XhciHotplugTest,
     AsyncDisableSlotParksDmaUntilCompletionAndSupportsSlot64) {
  XhciSimConfig cfg;
  cfg.max_slots = 64;
  cfg.round_robin_slots = true;
  cfg.command_delay_ticks = 2;
  SimWorld world;
  const int idx = world.AddController("00:04.0", cfg);
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));

  // Cycle 65 attaches/detaches across ports 5 and 6 while Disable Slot
  // completes asynchronously (2 ticks after doorbell). While a detached port's
  // DeviceSlot is still waiting for Disable Slot completion, an immediate
  // attach on the other port must allocate a distinct DeviceSlot and leave
  // DCBAA[disabling_slot_id] intact until OnDisableSlotCompletion runs.
  for (int i = 0; i < 65; ++i) {
    const int port = (i % 2 == 0) ? 5 : 6;
    SimUsbKeyboard& kbd =
        world.AttachKeyboard(idx, port, HighSpeedKeyboardConfig());
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; },
        2000));
    if (i == 63) {
      // 64th cycle uses Slot ID 64.
      EXPECT_TRUE(world.sim(idx).slot(64).enabled);
      world.TypeAscii(kbd, "6");
    }
    world.Detach(idx, port);
    world.ServiceReady();
  }
  ASSERT_TRUE(world.RunUntilIdle(idx));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(0));
  EXPECT_THAT(world.sim(idx).disable_slot_log(), t::Contains(64));
  EXPECT_THAT(world.sim(idx).violations(), t::IsEmpty());
  EXPECT_THAT(world.platform().input(), t::Eq("6"));
}

TEST(XhciHotplugTest,
     PortAttemptsResetAfterHealthyOperationAndOverCurrentRepowersPorts) {
  SimWorld world;
  const int idx = world.AddController("00:04.0", IntelLikeConfig());
  SimUsbHub& hub = world.AttachHub(idx, 5, Usb2HighSpeedSingleTtHubConfig(4));
  SimUsbKeyboard& hub_kbd =
      world.AttachKeyboardToHub(hub, 2, FullSpeedKeyboardConfig(64));
  SimKeyboardConfig root_cfg = HighSpeedKeyboardConfig();
  root_cfg.fail_get_device_desc_attempts = 2;  // Attaches on attempt 3
  SimUsbKeyboard& root_kbd = world.AttachKeyboard(idx, 6, root_cfg);

  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] {
        return world.controller(idx).attached_keyboard_count() == 2 &&
               world.controller(idx).NextDeadline() == usb::kNoDeadline;
      },
      5000));

  // Port 6 used 2 of its 3 attempts during initial bring-up. After > 60 s of
  // healthy operation, its attempt counter resets on the next failure so a
  // forced re-enumeration succeeds instead of disabling the port.
  world.RunForMs(61000);
  root_kbd.InjectInterruptError(IntrOutcome::kTransactionError, 3);
  world.ServiceReady();
  ASSERT_TRUE(world.RunUntil(
      [&] {
        return world.controller(idx).attached_keyboard_count() == 2 &&
               world.controller(idx).NextDeadline() == usb::kNoDeadline;
      },
      3000));
  world.TypeAscii(root_kbd, "r");
  EXPECT_THAT(world.platform().input(), t::Eq("r"));

  // Transient over-current on PPC=1 root port 6 drops PORTSC.PP, detaches
  // root_kbd, then re-powers PORTSC.PP and re-enumerates root_kbd.
  world.sim(idx).PulsePortOverCurrent(6);
  ASSERT_TRUE(world.RunUntil(
      [&] {
        return world.controller(idx).attached_keyboard_count() == 2 &&
               world.controller(idx).NextDeadline() == usb::kNoDeadline;
      },
      3000));
  world.TypeAscii(root_kbd, "p");
  EXPECT_THAT(world.platform().input(), t::Eq("rp"));

  // Transient over-current on hub port 2 drops PORT_POWER on the hub port,
  // detaches hub_kbd, then sends SET_FEATURE(PORT_POWER) and re-enumerates
  // hub_kbd.
  hub.PulsePortOverCurrent(2);
  ASSERT_TRUE(world.RunUntil(
      [&] {
        return world.controller(idx).attached_keyboard_count() == 2 &&
               world.controller(idx).NextDeadline() == usb::kNoDeadline;
      },
      3000));
  world.TypeAscii(hub_kbd, "h");
  EXPECT_THAT(world.platform().input(), t::Eq("rph"));
}

TEST(XhciHotplugTest, RecoveryReZeroesScratchpadBuffersBeforeRestart) {
  SimWorld world;
  const int idx = world.AddController("00:04.0", IntelLikeConfig());
  SimUsbKeyboard& kbd = world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));

  // IntelLikeConfig() has cap_length = 0x80 and max_scratchpads = 35.
  constexpr int kSpCount = 35;
  const uint64_t dcbaap =
      static_cast<uint64_t>(world.sim(idx).Peek(0x80 + spec::kDcbaapLow)) |
      (static_cast<uint64_t>(world.sim(idx).Peek(0x80 + spec::kDcbaapHigh))
       << 32);
  uint8_t* const dcbaa_bytes =
      world.arena().Translate(dcbaap, 8, /*require_below_4gib=*/false);
  ASSERT_NE(dcbaa_bytes, nullptr);
  uint64_t sp_array_phys = 0;
  std::memcpy(&sp_array_phys, dcbaa_bytes, sizeof(sp_array_phys));
  ASSERT_NE(sp_array_phys, 0u);

  const uint8_t* const sp_array_bytes = world.arena().Translate(
      sp_array_phys, kSpCount * 8, /*require_below_4gib=*/false);
  ASSERT_NE(sp_array_bytes, nullptr);

  // Simulate controller scratchpad writes while running by dirtying every
  // scratchpad buffer page.
  for (int i = 0; i < kSpCount; ++i) {
    uint64_t sp_phys = 0;
    std::memcpy(&sp_phys, sp_array_bytes + static_cast<size_t>(i) * 8,
                sizeof(sp_phys));
    uint8_t* const page =
        world.arena().Translate(sp_phys, 4096, /*require_below_4gib=*/false);
    ASSERT_NE(page, nullptr);
    std::memset(page, 0x5A, 4096);
  }

  // Trigger controller recovery (HCRST + Start): xHCI 1.2 §4.20 requires all
  // scratchpad buffers to be cleared to 0 again before USBCMD.RS = 1.
  world.sim(idx).InjectHostSystemError();
  world.ServiceReady();
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));

  for (int i = 0; i < kSpCount; ++i) {
    uint64_t sp_phys = 0;
    std::memcpy(&sp_phys, sp_array_bytes + static_cast<size_t>(i) * 8,
                sizeof(sp_phys));
    const uint8_t* const page =
        world.arena().Translate(sp_phys, 4096, /*require_below_4gib=*/false);
    ASSERT_NE(page, nullptr);
    for (int b = 0; b < 4096; ++b) {
      ASSERT_EQ(page[b], 0u) << "scratchpad " << i << " byte " << b;
    }
  }

  world.TypeAscii(kbd, "k");
  EXPECT_THAT(world.platform().input(), t::Eq("k"));
}

}  // namespace
}  // namespace protos::xhci_sim
