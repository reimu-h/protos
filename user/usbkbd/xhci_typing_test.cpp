#include <array>
#include <cstdint>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "user/usbkbd/usb_hid.h"
#include "user/usbkbd/xhci_controller.h"
#include "user/usbkbd/xhci_regs.h"
#include "user/usbkbd/xhci_sim.h"
#include "user/usbkbd/xhci_sim_usb.h"
#include "user/usbkbd/xhci_sim_world.h"

namespace protos::xhci_sim {
namespace {

namespace t = ::testing;

TEST(XhciTypingTest, EndToEndTypingHelloWorldAndShiftedSymbols) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbKeyboard& kbd = world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));

  world.TypeAscii(kbd, "hello world\n");
  EXPECT_THAT(world.platform().input(), t::Eq("hello world\r"));

  world.platform().ClearInput();
  world.TypeAscii(kbd, "AbC!@#$%^&*()_+{}:<>?123\t\b");
  EXPECT_THAT(world.platform().input(), t::Eq("AbC!@#$%^&*()_+{}:<>?123\t\b"));
}

TEST(XhciTypingTest, RolloverAutoRepeatSuppressionAndPhantomErrorRollOver) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbKeyboard& kbd = world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));

  // Press 'a', then press 'b' while 'a' is still held (2-key rollover).
  kbd.PressKey(usb::kHidUsageA);
  world.ServiceReady();
  EXPECT_THAT(world.platform().input(), t::Eq("a"));

  kbd.PressKey(usb::kHidUsageA + 1);  // 'b'
  world.ServiceReady();
  EXPECT_THAT(world.platform().input(), t::Eq("ab"));

  // Duplicate report with 'a' and 'b' still held: auto-repeat suppressed.
  kbd.PushReport({0, 0, usb::kHidUsageA, usb::kHidUsageA + 1, 0, 0, 0, 0});
  world.ServiceReady();
  EXPECT_THAT(world.platform().input(), t::Eq("ab"));

  // Phantom / ErrorRollOver report preserves prior held keys.
  kbd.PushReport({0, 0, 1, 1, 1, 1, 1, 1});
  world.ServiceReady();
  kbd.PushReport({0, 0, usb::kHidUsageA, usb::kHidUsageA + 1, 0, 0, 0, 0});
  world.ServiceReady();
  EXPECT_THAT(world.platform().input(), t::Eq("ab"));

  // Release 'a', then press 'a' again: emits 'a'.
  kbd.ReleaseKey(usb::kHidUsageA);
  world.ServiceReady();
  kbd.PressKey(usb::kHidUsageA);
  world.ServiceReady();
  EXPECT_THAT(world.platform().input(), t::Eq("aba"));

  kbd.ReleaseAllKeys();
  world.ServiceReady();
  world.platform().ClearInput();

  // Simultaneous 6-key chord in one report emits all 6 characters in order.
  kbd.PushReport({0, 0, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09});
  world.ServiceReady();
  EXPECT_THAT(world.platform().input(), t::Eq("abcdef"));
}

TEST(XhciTypingTest,
     ThreeHundredPlusKeystrokesWrapTransferAndEventRingsMultipleTimes) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbKeyboard& kbd = world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));

  // 720 keystrokes = 1440 interrupt IN transfers (> 5.6 wraps of the 256-TRB
  // transfer ring and 256-TRB event ring, flipping cycle state 1 -> 0 -> 1).
  std::string expected;
  expected.reserve(720);
  for (int i = 0; i < 720; ++i) {
    const char ch = static_cast<char>('a' + (i % 26));
    expected.push_back(ch);
    const char s[2] = {ch, '\0'};
    world.TypeAscii(kbd, s);
  }
  EXPECT_THAT(world.platform().input().size(), t::Eq(720u));
  EXPECT_THAT(world.platform().input(), t::Eq(expected));
}

TEST(XhciTypingTest, MultiKeyboardAndMultiControllerShareConsoleCapsLock) {
  SimWorld world;
  const int c0 =
      world.AddController("00:04.0", XhciSimConfig{}, usb::IrqMode::kMsix);
  const int c1 =
      world.AddController("00:14.0", IntelLikeConfig(), usb::IrqMode::kMsi);

  SimUsbKeyboard& kbd_a =
      world.AttachKeyboard(c0, 5, HighSpeedKeyboardConfig());
  SimUsbKeyboard& kbd_b =
      world.AttachKeyboard(c0, 6, FullSpeedKeyboardConfig(8));
  SimUsbKeyboard& kbd_c =
      world.AttachKeyboard(c1, 13, SuperSpeedKeyboardConfig(1));

  ASSERT_THAT(world.BringUpAll(), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(
      world.RunUntil([&] { return world.TotalAttachedKeyboards() == 3; }));

  world.TypeAscii(kbd_a, "a");
  world.TypeAscii(kbd_b, "b");
  world.TypeAscii(kbd_c, "c");
  EXPECT_THAT(world.platform().input(), t::Eq("abc"));

  // Toggle Caps Lock ON from kbd_a (Controller 0): affects kbd_b (Controller 0)
  // and kbd_c (Controller 1).
  kbd_a.TapKey(usb::kHidUsageCapsLock);
  world.ServiceReady();
  EXPECT_TRUE(world.locks().caps_lock);

  world.TypeAscii(kbd_b, "d");
  world.TypeAscii(kbd_c, "e");
  // Shift + 'f' while Caps Lock is ON inverts back to lowercase 'f'.
  kbd_c.TapKey(usb::kHidUsageA + 5, usb::kHidModLeftShift);
  world.ServiceReady();
  EXPECT_THAT(world.platform().input(), t::Eq("abcDEf"));

  // Toggle Caps Lock OFF from kbd_c (Controller 1): affects kbd_a.
  kbd_c.TapKey(usb::kHidUsageCapsLock);
  world.ServiceReady();
  EXPECT_FALSE(world.locks().caps_lock);

  world.TypeAscii(kbd_a, "g");
  EXPECT_THAT(world.platform().input(), t::Eq("abcDEfg"));
}

TEST(XhciTypingTest,
     InterruptEndpointRecoversFromTransientErrorsAndReenumeratesAfterThree) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbKeyboard& kbd = world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));

  world.TypeAscii(kbd, "a");
  EXPECT_THAT(world.platform().input(), t::Eq("a"));

  // Press 'b' (1 report -> device & host data toggle become DATA1), then inject
  // 1 transient transaction error before releasing 'b': recovered via Soft
  // Retry (Reset Endpoint TSP=1 + Set TR Dequeue Pointer) without clearing
  // DATA1 or dropping the release report.
  kbd.PressKey(usb::kHidUsageA + 1);
  world.ServiceReady();
  EXPECT_THAT(kbd.interrupt_toggle(), t::Eq(1u));
  kbd.InjectInterruptError(IntrOutcome::kTransactionError, 1);
  world.ServiceReady();
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(1));
  EXPECT_THAT(kbd.interrupt_toggle(), t::Eq(1u));
  kbd.ReleaseKey(usb::kHidUsageA + 1);
  world.ServiceReady();
  EXPECT_THAT(world.sim(idx).dropped_in_packets(), t::Eq(0));
  EXPECT_THAT(world.platform().input(), t::Eq("ab"));

  // After > 1 s of healthy operation, the interrupt error counter decays so
  // 2 consecutive STALL errors still recover in-place via Reset Endpoint
  // (TSP=0) + Set TR Dequeue Pointer + CLEAR_FEATURE(ENDPOINT_HALT).
  world.RunForMs(1100);
  kbd.InjectInterruptError(IntrOutcome::kStall, 2);
  world.ServiceReady();
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(1));
  EXPECT_THAT(kbd.clear_halt_count(), t::Eq(2));
  EXPECT_FALSE(kbd.interrupt_halted());
  EXPECT_THAT(world.sim(idx).dropped_in_packets(), t::Eq(0));
  world.TypeAscii(kbd, "c");
  EXPECT_THAT(world.platform().input(), t::Eq("abc"));

  // 3 consecutive transaction errors hit the 3-strike threshold: detaches,
  // disables the slot, re-resets the port, and re-enumerates the keyboard.
  kbd.InjectInterruptError(IntrOutcome::kTransactionError, 3);
  world.ServiceReady();
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  world.TypeAscii(kbd, "d");
  EXPECT_THAT(world.platform().input(), t::Eq("abcd"));

  // Transient Babble error on interrupt IN endpoint also recovers in-place.
  kbd.InjectInterruptError(IntrOutcome::kBabble, 1);
  world.ServiceReady();
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(1));
  world.TypeAscii(kbd, "e");
  EXPECT_THAT(world.platform().input(), t::Eq("abcde"));
}

TEST(XhciTypingTest,
     ConcurrentMultiControllerHsFsLsKeyboardsIsolateRolloverAndShareLocks) {
  SimWorld world;
  const int c0 =
      world.AddController("00:04.0", XhciSimConfig{}, usb::IrqMode::kMsix);
  const int c1 =
      world.AddController("01:01.0", IntelLikeConfig(), usb::IrqMode::kMsi);

  SimUsbKeyboard& kbd_hs =
      world.AttachKeyboard(c0, 5, HighSpeedKeyboardConfig());
  SimUsbKeyboard& kbd_fs =
      world.AttachKeyboard(c1, 1, FullSpeedKeyboardConfig(64));
  SimUsbKeyboard& kbd_ls =
      world.AttachKeyboard(c1, 2, LowSpeedKeyboardConfig());

  ASSERT_THAT(world.BringUpAll(), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(
      world.RunUntil([&] { return world.TotalAttachedKeyboards() == 3; }));

  // Hold Left Shift + 'a' on kbd_hs (Controller 0): emits 'A'.
  kbd_hs.PressKey(usb::kHidUsageA, usb::kHidModLeftShift);
  world.ServiceReady();
  EXPECT_THAT(world.platform().input(), t::Eq("A"));

  // While Left Shift + 'a' are still held on kbd_hs, press 'a' and 'b' on
  // kbd_fs (Controller 1) and 'c' on kbd_ls (Controller 1): per-port modifier
  // and rollover state are isolated, so kbd_fs emits lowercase "ab" and kbd_ls
  // emits lowercase "c".
  kbd_fs.PressKey(usb::kHidUsageA);
  kbd_fs.PressKey(usb::kHidUsageA + 1);
  kbd_ls.PressKey(usb::kHidUsageA + 2);
  world.ServiceReady();
  EXPECT_THAT(world.platform().input(), t::Eq("Aabc"));

  // Toggle Caps Lock ON from kbd_ls (Controller 1) while keys are still held:
  // shared KeyboardLocks flips caps_lock to true across both controllers.
  kbd_ls.TapKey(usb::kHidUsageCapsLock);
  world.ServiceReady();
  EXPECT_TRUE(world.locks().caps_lock);

  // Pressing 'd' on kbd_hs while Left Shift is still held on kbd_hs + Caps Lock
  // is ON produces lowercase 'd' (Shift XOR Caps Lock), whereas pressing 'e' on
  // kbd_fs (no Shift held) produces uppercase 'E'.
  kbd_hs.PressKey(usb::kHidUsageA + 3, usb::kHidModLeftShift);
  kbd_fs.PressKey(usb::kHidUsageA + 4);
  world.ServiceReady();
  EXPECT_THAT(world.platform().input(), t::Eq("AabcdE"));

  kbd_hs.ReleaseAllKeys();
  kbd_fs.ReleaseAllKeys();
  kbd_ls.ReleaseAllKeys();
  world.ServiceReady();
}

TEST(XhciTypingTest,
     SimultaneousKeyboardsAcrossRootPortUsb2HubUsb3HubAndNestedHub) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  // 1. Direct root-port High-Speed keyboard on port 6.
  SimUsbKeyboard& kbd_root =
      world.AttachKeyboard(idx, 6, HighSpeedKeyboardConfig());
  // 2. USB 2.0 High-Speed Multi-TT hub on port 5 with:
  //    - Full-Speed keyboard on hub port 1
  //    - Nested Full-Speed hub on hub port 2 -> Low-Speed keyboard on port 3
  SimUsbHub& usb2_hub =
      world.AttachHub(idx, 5, Usb2HighSpeedMultiTtHubConfig(4));
  SimUsbKeyboard& kbd_usb2_hub =
      world.AttachKeyboardToHub(usb2_hub, 1, FullSpeedKeyboardConfig(64));
  SimUsbHub& nested_hub =
      world.AttachHubToHub(usb2_hub, 2, Usb2FullSpeedHubConfig(4));
  SimUsbKeyboard& kbd_nested =
      world.AttachKeyboardToHub(nested_hub, 3, LowSpeedKeyboardConfig());
  // 3. USB 3.0 SuperSpeed hub on port 1 with SuperSpeed keyboard on hub port 4.
  SimUsbHub& usb3_hub = world.AttachHub(idx, 1, Usb3SuperSpeedHubConfig(4));
  SimUsbKeyboard& kbd_usb3_hub =
      world.AttachKeyboardToHub(usb3_hub, 4, SuperSpeedKeyboardConfig());

  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 4; }));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(7));

  // Hold Left Shift + 'a' on kbd_usb2_hub (Slot 5): emits 'A'.
  kbd_usb2_hub.PressKey(usb::kHidUsageA, usb::kHidModLeftShift);
  world.ServiceReady();
  EXPECT_THAT(world.platform().input(), t::Eq("A"));

  // While Left Shift + 'a' remain held on kbd_usb2_hub, press 'b' on kbd_root
  // (Slot 3), 'c' on kbd_usb3_hub (Slot 4), and 'd' on kbd_nested (Slot 7)
  // concurrently: per-device modifier and rollover state are isolated.
  kbd_root.PressKey(usb::kHidUsageA + 1);
  kbd_usb3_hub.PressKey(usb::kHidUsageA + 2);
  kbd_nested.PressKey(usb::kHidUsageA + 3);
  world.ServiceReady();
  EXPECT_THAT(world.platform().input(), t::Eq("Abcd"));

  // Toggle Caps Lock from kbd_nested (shared KeyboardLocks across all slots),
  // then concurrently press 'e' on kbd_usb3_hub (no Shift + Caps Lock ON ->
  // 'E') and 'f' on kbd_usb2_hub (Shift held + Caps Lock ON -> 'f').
  kbd_nested.TapKey(usb::kHidUsageCapsLock);
  world.ServiceReady();
  EXPECT_TRUE(world.locks().caps_lock);

  kbd_usb3_hub.PressKey(usb::kHidUsageA + 4);
  kbd_usb2_hub.PressKey(usb::kHidUsageA + 5, usb::kHidModLeftShift);
  world.ServiceReady();
  EXPECT_THAT(world.platform().input(), t::Eq("AbcdEf"));

  kbd_root.ReleaseAllKeys();
  kbd_usb2_hub.ReleaseAllKeys();
  kbd_nested.ReleaseAllKeys();
  kbd_usb3_hub.ReleaseAllKeys();
  world.ServiceReady();
}

}  // namespace
}  // namespace protos::xhci_sim
