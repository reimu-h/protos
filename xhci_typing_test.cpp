#include <array>
#include <cstdint>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "user/usb_hid.h"
#include "user/xhci_controller.h"
#include "user/xhci_regs.h"
#include "xhci_sim.h"
#include "xhci_sim_usb.h"
#include "xhci_sim_world.h"

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

  // 1 transient transaction error on the interrupt IN endpoint: recovered via
  // Reset Endpoint + Set TR Dequeue Pointer without detaching the keyboard.
  kbd.InjectInterruptError(IntrOutcome::kTransactionError, 1);
  world.ServiceReady();
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(1));
  world.TypeAscii(kbd, "b");
  EXPECT_THAT(world.platform().input(), t::Eq("ab"));

  // 2 consecutive STALL errors on the interrupt IN endpoint: still below the
  // 3-strike threshold, recovers in place and resumes typing.
  kbd.InjectInterruptError(IntrOutcome::kStall, 2);
  world.ServiceReady();
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(1));
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

}  // namespace
}  // namespace protos::xhci_sim
