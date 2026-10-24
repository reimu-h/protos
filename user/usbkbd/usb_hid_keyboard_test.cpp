// Tests of the HID boot-keyboard translation (user/usbkbd/usb_hid.h). The
// expected characters are written out by hand here (US layout, HID Usage Tables
// page 0x07), not taken from the code under test.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <initializer_list>
#include <random>
#include <string>
#include <vector>

#include "user/sh/sh_lib.h"
#include "user/usbkbd/usb_hid.h"

namespace protos::usb {
namespace {

namespace t = ::testing;

constexpr uint8_t kLeftCtrl = 0x01;
constexpr uint8_t kLeftShift = 0x02;
constexpr uint8_t kLeftAlt = 0x04;
constexpr uint8_t kLeftGui = 0x08;
constexpr uint8_t kRightCtrl = 0x10;
constexpr uint8_t kRightShift = 0x20;
constexpr uint8_t kRightAlt = 0x40;
constexpr uint8_t kRightGui = 0x80;
constexpr uint8_t kCapsLock = 0x39;

struct Key {
  uint8_t usage;
  char unshifted;
  char shifted;
};

// Every key with a character, except the letters.
constexpr Key kKeys[] = {
    {0x1E, '1', '!'},
    {0x1F, '2', '@'},
    {0x20, '3', '#'},
    {0x21, '4', '$'},
    {0x22, '5', '%'},
    {0x23, '6', '^'},
    {0x24, '7', '&'},
    {0x25, '8', '*'},
    {0x26, '9', '('},
    {0x27, '0', ')'},
    {0x28, '\r', '\r'},
    {0x2A, '\b', '\b'},
    {0x2B, '\t', '\t'},
    {0x2C, ' ', ' '},
    {0x2D, '-', '_'},
    {0x2E, '=', '+'},
    {0x2F, '[', '{'},
    {0x30, ']', '}'},
    {0x31, '\\', '|'},
    {0x32, '\\', '|'},
    {0x33, ';', ':'},
    {0x34, '\'', '"'},
    {0x35, '`', '~'},
    {0x36, ',', '<'},
    {0x37, '.', '>'},
    {0x38, '/', '?'},
    // Keypad (no Num Lock state: digits always).
    {0x54, '/', '/'},
    {0x55, '*', '*'},
    {0x56, '-', '-'},
    {0x57, '+', '+'},
    {0x58, '\r', '\r'},
    {0x59, '1', '1'},
    {0x5A, '2', '2'},
    {0x5B, '3', '3'},
    {0x5C, '4', '4'},
    {0x5D, '5', '5'},
    {0x5E, '6', '6'},
    {0x5F, '7', '7'},
    {0x60, '8', '8'},
    {0x61, '9', '9'},
    {0x62, '0', '0'},
    {0x63, '.', '.'},
    {0x64, '\\', '|'},
    {0x67, '=', '='},
};

// The expected character of a key: letters a-z at 0x04-0x1D, uppercase if
// exactly one of Shift and Caps Lock; the table above; else none.
static char ExpectedChar(const int usage,   //
                         const bool shift,  //
                         const bool caps_lock) {
  if (usage >= 0x04 && usage <= 0x1D) {
    return static_cast<char>((shift != caps_lock ? 'A' : 'a') + usage - 0x04);
  }
  for (const Key& key : kKeys) {
    if (key.usage == usage) {
      return shift ? key.shifted : key.unshifted;
    }
  }
  return 0;
}

// An 8-byte boot report.
static std::vector<uint8_t> Report(const uint8_t modifiers,
                                   const std::initializer_list<uint8_t> keys) {
  std::vector<uint8_t> report(kBootReportSize, 0);
  report[0] = modifiers;
  int slot = 2;
  for (const uint8_t key : keys) {
    report[static_cast<size_t>(slot++)] = key;
  }
  return report;
}

// Translates one report and returns the bytes.
static std::string Feed(HidKeyboardState* const state,
                        const std::vector<uint8_t>& report) {
  char out[kBootReportKeys] = {};
  const int count = TranslateBootReport(state,                            //
                                        report.data(),                    //
                                        static_cast<int>(report.size()),  //
                                        out,                              //
                                        kBootReportKeys);
  EXPECT_THAT(count, t::AllOf(t::Ge(0), t::Le(kBootReportKeys)));
  return std::string(out, static_cast<size_t>(count));
}

static std::string FeedAll(HidKeyboardState* const state,
                           const std::vector<std::vector<uint8_t>>& reports) {
  std::string bytes;
  for (const std::vector<uint8_t>& report : reports) {
    bytes += Feed(state, report);
  }
  return bytes;
}

// A keyboard that sends reports the way QEMU's usb-kbd does (hw/input/hid.c):
// one report per key event; modifiers in byte 0; keys in press order; a
// released key's slot gets the last key; more than six keys down -> all six
// slots ErrorRollOver.
class QemuKeyboard {
 public:
  std::vector<uint8_t> Press(const uint8_t usage) {
    if (usage >= 0xE0 && usage <= 0xE7) {
      modifiers_ |= static_cast<uint8_t>(1u << (usage - 0xE0));
    } else if (Find(usage) < 0 && keys_.size() < 16) {
      keys_.push_back(usage);
    }
    return Poll();
  }

  std::vector<uint8_t> Release(const uint8_t usage) {
    if (usage >= 0xE0 && usage <= 0xE7) {
      modifiers_ &= static_cast<uint8_t>(~(1u << (usage - 0xE0)));
    } else if (const int index = Find(usage); index >= 0) {
      keys_[static_cast<size_t>(index)] = keys_.back();
      keys_.pop_back();
    }
    return Poll();
  }

  // The reports for typing `text` (US layout), Shift pressed and released
  // around shifted characters.
  std::vector<std::vector<uint8_t>> Type(const std::string& text) {
    std::vector<std::vector<uint8_t>> reports;
    for (const char ch : text) {
      bool shift = false;
      const uint8_t usage = UsageOf(ch, &shift);
      if (shift) {
        reports.push_back(Press(0xE1));
      }
      reports.push_back(Press(usage));
      reports.push_back(Release(usage));
      if (shift) {
        reports.push_back(Release(0xE1));
      }
    }
    return reports;
  }

  // The key that types `ch` (US layout), from the hand-written tables.
  static uint8_t UsageOf(const char ch, bool* const shift) {
    *shift = false;
    if (ch >= 'a' && ch <= 'z') {
      return static_cast<uint8_t>(0x04 + (ch - 'a'));
    }
    if (ch >= 'A' && ch <= 'Z') {
      *shift = true;
      return static_cast<uint8_t>(0x04 + (ch - 'A'));
    }
    for (const Key& key : kKeys) {
      if (key.usage >= 0x54) {
        break;  // Prefer the main keys over the keypad
      }
      if (key.unshifted == ch) {
        return key.usage;
      }
      if (key.shifted == ch) {
        *shift = true;
        return key.usage;
      }
    }
    ADD_FAILURE() << "no key types 0x" << std::hex << int{ch};
    return 0;
  }

 private:
  int Find(const uint8_t usage) const {
    for (int i = static_cast<int>(keys_.size()) - 1; i >= 0; --i) {
      if (keys_[static_cast<size_t>(i)] == usage) {
        return i;
      }
    }
    return -1;
  }

  std::vector<uint8_t> Poll() const {
    std::vector<uint8_t> report(kBootReportSize, 0);
    report[0] = modifiers_;
    for (int i = 0; i < 6; ++i) {
      if (keys_.size() > 6) {
        report[static_cast<size_t>(2 + i)] = 0x01;
      } else if (static_cast<size_t>(i) < keys_.size()) {
        report[static_cast<size_t>(2 + i)] = keys_[static_cast<size_t>(i)];
      }
    }
    return report;
  }

  uint8_t modifiers_ = 0;
  std::vector<uint8_t> keys_;
};

TEST(UsbHidKeyboardTest, EveryUsageTranslatesLikeTheUsLayout) {
  for (int usage = 0; usage <= 0xFF; ++usage) {
    for (const bool shift : {false, true}) {
      for (const bool caps_lock : {false, true}) {
        EXPECT_THAT(
            TranslateUsage(static_cast<uint8_t>(usage), shift, caps_lock),
            t::Eq(ExpectedChar(usage, shift, caps_lock)))
            << "usage 0x" << std::hex << usage << " shift " << shift << " caps "
            << caps_lock;
      }
    }
  }
}

TEST(UsbHidKeyboardTest, AllLettersWithAndWithoutEitherShift) {
  for (const uint8_t modifiers : {uint8_t{0}, kLeftShift, kRightShift}) {
    HidKeyboardState state;
    std::string typed;
    for (uint8_t usage = 0x04; usage <= 0x1D; ++usage) {
      typed += Feed(&state, Report(modifiers, {usage}));
      EXPECT_THAT(Feed(&state, Report(modifiers, {})), t::IsEmpty());
    }
    EXPECT_THAT(typed, t::Eq(modifiers == 0 ? "abcdefghijklmnopqrstuvwxyz"
                                            : "ABCDEFGHIJKLMNOPQRSTUVWXYZ"));
  }
}

TEST(UsbHidKeyboardTest, DigitsAndSymbolsWithAndWithoutShift) {
  HidKeyboardState state;
  std::string plain;
  std::string shifted;
  for (uint8_t usage = 0x1E; usage <= 0x38; ++usage) {
    if (usage >= 0x28 && usage <= 0x2C) {
      continue;  // Enter, Escape, Backspace, Tab, Space
    }
    plain += Feed(&state, Report(0, {usage}));
    Feed(&state, Report(0, {}));
    shifted += Feed(&state, Report(kLeftShift, {usage}));
    Feed(&state, Report(0, {}));
  }
  EXPECT_THAT(plain, t::Eq("1234567890-=[]\\\\;'`,./"));
  EXPECT_THAT(shifted, t::Eq("!@#$%^&*()_+{}||:\"~<>?"));
}

TEST(UsbHidKeyboardTest, EnterIsACarriageReturnOnly) {
  HidKeyboardState state;
  EXPECT_THAT(Feed(&state, Report(0, {0x28})), t::Eq("\r"));
  Feed(&state, Report(0, {}));
  EXPECT_THAT(Feed(&state, Report(kRightShift, {0x28})), t::Eq("\r"));
  Feed(&state, Report(0, {}));
  EXPECT_THAT(Feed(&state, Report(0, {0x58})), t::Eq("\r"));  // Keypad Enter
}

TEST(UsbHidKeyboardTest, BackspaceIs0x08AndDeleteTypesNothing) {
  HidKeyboardState state;
  EXPECT_THAT(Feed(&state, Report(0, {0x2A})), t::Eq("\x08"));
  Feed(&state, Report(0, {}));
  // 0x7F would erase like Backspace in the shell.
  EXPECT_THAT(Feed(&state, Report(0, {0x4C})), t::IsEmpty());
  EXPECT_THAT(Feed(&state, Report(kLeftShift, {0x4C})), t::IsEmpty());
}

TEST(UsbHidKeyboardTest, TabAndSpaceIgnoreShift) {
  HidKeyboardState state;
  EXPECT_THAT(FeedAll(&state, {Report(0, {0x2B}), Report(0, {}),
                               Report(kLeftShift, {0x2B}), Report(0, {}),
                               Report(0, {0x2C}), Report(0, {}),
                               Report(kLeftShift, {0x2C})}),
              t::Eq("\t\t  "));
}

TEST(UsbHidKeyboardTest, KeysWithoutCharactersTypeNothing) {
  // Escape, F1-F12, Print Screen .. Up, Num Lock, Application, Power,
  // F13 and beyond, and everything above the keyboard page's keys.
  std::vector<uint8_t> usages = {0x29, 0x53, 0x65, 0x66};
  for (int usage = 0x3A; usage <= 0x52; ++usage) {
    usages.push_back(static_cast<uint8_t>(usage));
  }
  for (int usage = 0x68; usage <= 0xDF; ++usage) {
    usages.push_back(static_cast<uint8_t>(usage));
  }
  for (int usage = 0xE8; usage <= 0xFF; ++usage) {
    usages.push_back(static_cast<uint8_t>(usage));
  }
  HidKeyboardState state;
  for (const uint8_t usage : usages) {
    EXPECT_THAT(Feed(&state, Report(0, {usage})), t::IsEmpty())
        << "usage 0x" << std::hex << int{usage};
    EXPECT_THAT(Feed(&state, Report(kLeftShift, {})), t::IsEmpty());
  }
  // The state still works.
  EXPECT_THAT(Feed(&state, Report(0, {0x04})), t::Eq("a"));
}

TEST(UsbHidKeyboardTest, CapsLockTogglesLettersOnly) {
  HidKeyboardState state;
  EXPECT_THAT(Feed(&state, Report(0, {kCapsLock})), t::IsEmpty());
  EXPECT_TRUE(state.caps_lock);
  EXPECT_THAT(FeedAll(&state, {Report(0, {}), Report(0, {0x04}), Report(0, {}),
                               Report(0, {0x1E}), Report(0, {}),
                               Report(0, {0x2D}), Report(0, {})}),
              t::Eq("A1-"));
  EXPECT_THAT(Feed(&state, Report(0, {kCapsLock})), t::IsEmpty());
  EXPECT_FALSE(state.caps_lock);
  EXPECT_THAT(FeedAll(&state, {Report(0, {}), Report(0, {0x04})}), t::Eq("a"));
}

TEST(UsbHidKeyboardTest, ShiftWithCapsLockTypesLowercaseLetters) {
  HidKeyboardState state;
  Feed(&state, Report(0, {kCapsLock}));
  Feed(&state, Report(0, {}));
  EXPECT_THAT(FeedAll(&state, {Report(kLeftShift, {0x04}), Report(0, {}),
                               Report(kRightShift, {0x1E}), Report(0, {}),
                               Report(kLeftShift, {0x2D})}),
              t::Eq("a!_"));
}

TEST(UsbHidKeyboardTest, CapsLockTogglesOncePerPress) {
  HidKeyboardState state;
  Feed(&state, Report(0, {kCapsLock}));
  EXPECT_TRUE(state.caps_lock);
  Feed(&state, Report(0, {kCapsLock}));  // Held: must not toggle off
  EXPECT_TRUE(state.caps_lock);
  EXPECT_THAT(Feed(&state, Report(0, {kCapsLock, 0x04})), t::Eq("A"));
  EXPECT_TRUE(state.caps_lock);
  Feed(&state, Report(0, {}));  // Released: no toggle
  EXPECT_TRUE(state.caps_lock);
  EXPECT_THAT(Feed(&state, Report(0, {0x05})), t::Eq("B"));
}

TEST(UsbHidKeyboardTest, CapsLockAppliesToKeysOfTheSameReport) {
  // Whatever slot order the keyboard uses.
  HidKeyboardState first;
  EXPECT_THAT(Feed(&first, Report(0, {kCapsLock, 0x04})), t::Eq("A"));
  HidKeyboardState second;
  EXPECT_THAT(Feed(&second, Report(0, {0x04, kCapsLock})), t::Eq("A"));
}

TEST(UsbHidKeyboardTest, ErrorReportsAreIgnoredAndTheStateIsKept) {
  HidKeyboardState state;
  EXPECT_THAT(Feed(&state, Report(0, {0x04, 0x05, 0x06, 0x07, 0x08, 0x09})),
              t::Eq("abcdef"));
  // A seventh key: ErrorRollOver in every slot.
  EXPECT_THAT(Feed(&state, Report(0, {1, 1, 1, 1, 1, 1})), t::IsEmpty());
  // One key released: only the new key is new.
  EXPECT_THAT(Feed(&state, Report(0, {0x04, 0x05, 0x06, 0x07, 0x08, 0x0A})),
              t::Eq("g"));
  // ErrorRollOver, POSTFail and ErrorUndefined in any slot.
  for (const uint8_t error : {uint8_t{1}, uint8_t{2}, uint8_t{3}}) {
    EXPECT_THAT(Feed(&state, Report(0, {0x0B, error})), t::IsEmpty());
  }
  // A Caps Lock press in an error report does not toggle.
  EXPECT_THAT(Feed(&state, Report(0, {kCapsLock, 1})), t::IsEmpty());
  EXPECT_FALSE(state.caps_lock);
  EXPECT_THAT(Feed(&state, Report(0, {0x0B})), t::Eq("h"));
}

TEST(UsbHidKeyboardTest, HeldKeysTypeOnce) {
  HidKeyboardState state;
  EXPECT_THAT(FeedAll(&state, {Report(0, {0x04}), Report(0, {0x04}),
                               Report(0, {0x04})}),
              t::Eq("a"));
}

TEST(UsbHidKeyboardTest, ReleaseThenPressTypesAgain) {
  HidKeyboardState state;
  EXPECT_THAT(
      FeedAll(&state, {Report(0, {0x04}), Report(0, {}), Report(0, {0x04})}),
      t::Eq("aa"));
}

TEST(UsbHidKeyboardTest, SeveralNewKeysInOneReportTypeInSlotOrder) {
  HidKeyboardState state;
  EXPECT_THAT(Feed(&state, Report(0, {0x04, 0x05, 0x06})), t::Eq("abc"));
  EXPECT_THAT(Feed(&state, Report(0, {0x04, 0x05, 0x06})), t::IsEmpty());
}

TEST(UsbHidKeyboardTest, KeysMovingToOtherSlotsAreNotNewPresses) {
  // QEMU moves the last key into the slot of a released key.
  HidKeyboardState state;
  EXPECT_THAT(FeedAll(&state, {Report(0, {0x04}), Report(0, {0x04, 0x05}),
                               Report(0, {0x05}), Report(0, {})}),
              t::Eq("ab"));
  EXPECT_THAT(
      FeedAll(&state, {Report(0, {0x04, 0x05, 0x06}), Report(0, {0x06, 0x05}),
                       Report(0, {0x05, 0x06})}),
      t::Eq("abc"));
}

TEST(UsbHidKeyboardTest, ModifierOnlyReportsTypeNothing) {
  HidKeyboardState state;
  EXPECT_THAT(
      FeedAll(&state, {Report(kLeftShift, {}), Report(kLeftShift, {0x0B}),
                       Report(kLeftShift, {}), Report(0, {})}),
      t::Eq("H"));
  // Shift pressed while a key is held does not type it again.
  EXPECT_THAT(FeedAll(&state, {Report(0, {0x04}), Report(kLeftShift, {0x04}),
                               Report(0, {0x04})}),
              t::Eq("a"));
}

TEST(UsbHidKeyboardTest, ShiftReleasedTogetherWithTheNextPress) {
  HidKeyboardState state;
  EXPECT_THAT(
      FeedAll(&state, {Report(kLeftShift, {0x04}), Report(0, {0x04, 0x05})}),
      t::Eq("Ab"));
}

TEST(UsbHidKeyboardTest, ModifierUsagesInTheKeyArrayAreIgnored) {
  HidKeyboardState state;
  EXPECT_THAT(Feed(&state, Report(0, {0xE1, 0x04, 0xE7})), t::Eq("a"));
}

TEST(UsbHidKeyboardTest, DuplicateKeyInOneReportTypesOnce) {
  HidKeyboardState state;
  EXPECT_THAT(Feed(&state, Report(0, {0x04, 0x04})), t::Eq("a"));
}

TEST(UsbHidKeyboardTest, ReservedByteIsIgnored) {
  HidKeyboardState state;
  std::vector<uint8_t> report = Report(0, {0x04});
  report[1] = 0xFF;
  EXPECT_THAT(Feed(&state, report), t::Eq("a"));
}

TEST(UsbHidKeyboardTest, ShortAndLongReports) {
  HidKeyboardState state;
  const uint8_t key_a[] = {0x00, 0x00, 0x04, 0x05, 0x06};
  char out[kBootReportKeys] = {};
  // Too short: ignored, state unchanged.
  for (const int length : {0, 1, 2}) {
    EXPECT_THAT(TranslateBootReport(&state, key_a, length, out, 6), t::Eq(0));
  }
  // Three bytes: one key slot; the others count as empty.
  EXPECT_THAT(TranslateBootReport(&state, key_a, 3, out, 6), t::Eq(1));
  EXPECT_THAT(out[0], t::Eq('a'));
  EXPECT_THAT(TranslateBootReport(&state, key_a, 5, out, 6), t::Eq(2));
  EXPECT_THAT(std::string(out, 2), t::Eq("bc"));
  // Longer than 8 bytes: the rest is ignored.
  std::vector<uint8_t> long_report(64, 0x07);  // 'd' everywhere
  long_report[0] = 0;
  long_report[1] = 0;
  for (int i = 2; i < 8; ++i) {
    long_report[static_cast<size_t>(i)] = 0;
  }
  long_report[2] = 0x08;
  EXPECT_THAT(Feed(&state, long_report), t::Eq("e"));
}

TEST(UsbHidKeyboardTest, KeysPressedWithCtrlAltOrGuiTypeNothing) {
  for (const uint8_t modifier :
       {kLeftCtrl, kLeftAlt, kLeftGui, kRightCtrl, kRightAlt, kRightGui}) {
    HidKeyboardState state;
    // Ctrl+C, and Ctrl+H/I/J/M, which would be Backspace, Tab and Enter.
    for (const uint8_t usage : {uint8_t{0x06}, uint8_t{0x0B}, uint8_t{0x0C},
                                uint8_t{0x0D}, uint8_t{0x10}, uint8_t{0x28}}) {
      EXPECT_THAT(Feed(&state, Report(modifier, {usage})), t::IsEmpty())
          << "modifiers 0x" << std::hex << int{modifier};
      Feed(&state, Report(modifier, {}));
    }
    // Releasing the modifier while the key is held does not type it.
    Feed(&state, Report(modifier | kLeftShift, {0x04}));
    EXPECT_THAT(Feed(&state, Report(0, {0x04})), t::IsEmpty());
    EXPECT_THAT(FeedAll(&state, {Report(0, {}), Report(0, {0x04})}),
                t::Eq("a"));
  }
  // Caps Lock still toggles.
  HidKeyboardState state;
  Feed(&state, Report(kLeftCtrl, {kCapsLock}));
  EXPECT_TRUE(state.caps_lock);
}

TEST(UsbHidKeyboardTest, OutputCapacityLimitsTheBytesButNotTheState) {
  HidKeyboardState state;
  const std::vector<uint8_t> report =
      Report(0, {0x04, 0x05, 0x06, 0x07, 0x08, 0x09});
  char out[3] = {};
  EXPECT_THAT(TranslateBootReport(&state, report.data(), 8, out, 3), t::Eq(3));
  EXPECT_THAT(std::string(out, 3), t::Eq("abc"));
  EXPECT_THAT(Feed(&state, report), t::IsEmpty());
  EXPECT_THAT(TranslateBootReport(&state, report.data(), 8, nullptr, 6),
              t::Eq(0));
}

TEST(UsbHidKeyboardTest, NullArgumentsAreHarmless) {
  HidKeyboardState state;
  const std::vector<uint8_t> report = Report(0, {0x04});
  char out[kBootReportKeys] = {};
  EXPECT_THAT(TranslateBootReport(nullptr, report.data(), 8, out, 6), t::Eq(0));
  EXPECT_THAT(TranslateBootReport(&state, nullptr, 8, out, 6), t::Eq(0));
  EXPECT_THAT(Feed(&state, report), t::Eq("a"));
}

TEST(UsbHidKeyboardTest, QemuReportsTypeTheHarnessAlphabet) {
  const std::string text =
      "echo AbC-_+{}[]|\\:;\"'<>,.?/~`!@#$%^&*()= 0123456789 xyzXYZ\r";
  QemuKeyboard keyboard;
  HidKeyboardState state;
  EXPECT_THAT(FeedAll(&state, keyboard.Type(text)), t::Eq(text));
}

TEST(UsbHidKeyboardTest, QemuRolloverAndReleaseOrder) {
  QemuKeyboard keyboard;
  HidKeyboardState state;
  std::string typed;
  // Seven keys down: the seventh report is all ErrorRollOver.
  for (uint8_t usage = 0x04; usage <= 0x0A; ++usage) {
    typed += Feed(&state, keyboard.Press(usage));
  }
  EXPECT_THAT(typed, t::Eq("abcdef"));
  // Releasing 'a' makes room: QEMU moves 'g' into its slot.
  const std::vector<uint8_t> report = keyboard.Release(0x04);
  EXPECT_THAT(report[2], t::Eq(0x0A));
  typed += Feed(&state, report);
  EXPECT_THAT(typed, t::Eq("abcdefg"));
  for (uint8_t usage = 0x05; usage <= 0x0A; ++usage) {
    typed += Feed(&state, keyboard.Release(usage));
  }
  EXPECT_THAT(typed, t::Eq("abcdefg"));
  EXPECT_THAT(Feed(&state, keyboard.Press(0x04)), t::Eq("a"));
}

TEST(UsbHidKeyboardTest, TypedCommandWithCorrectionsIsWhatTheShellRuns) {
  QemuKeyboard keyboard;
  HidKeyboardState state;
  std::vector<std::vector<uint8_t>> reports = keyboard.Type("echo Hx");
  reports.push_back(keyboard.Press(0x2A));  // Backspace
  reports.push_back(keyboard.Release(0x2A));
  for (const std::vector<uint8_t>& report : keyboard.Type("eLLo_1!\r")) {
    reports.push_back(report);
  }
  const std::string bytes = FeedAll(&state, reports);

  sh::ShellLineEditor editor;
  int erases = 0;
  int submits = 0;
  std::string submitted;
  for (char ch : bytes) {
    switch (editor.FeedChar(&ch)) {
      case sh::ShellCharAction::kBackspaceErase:
        ++erases;
        break;
      case sh::ShellCharAction::kSubmitLine:
        ++submits;
        submitted = editor.buffer;
        break;
      default:
        break;
    }
  }
  EXPECT_THAT(submitted, t::Eq("echo HeLLo_1!"));
  EXPECT_THAT(erases, t::Eq(1));
  EXPECT_THAT(submits, t::Eq(1));
}

TEST(UsbHidKeyboardTest, RandomReportsOnlyProduceShellBytes) {
  std::mt19937 random(2026);
  HidKeyboardState state;
  std::vector<uint8_t> previous(kBootReportKeys, 0);
  for (int round = 0; round < 20000; ++round) {
    std::vector<uint8_t> report(kBootReportSize, 0);
    report[0] = static_cast<uint8_t>(random());
    report[1] = static_cast<uint8_t>(random());
    const int keys = static_cast<int>(random() % 7);
    for (int i = 0; i < keys; ++i) {
      // Mostly real keys, sometimes anything (including error codes).
      report[static_cast<size_t>(2 + i)] =
          static_cast<uint8_t>(random() % 8 == 0 ? random() : random() % 0x68);
    }
    bool error = false;
    bool only_held_keys = true;
    for (int i = 2; i < kBootReportSize; ++i) {
      const uint8_t usage = report[static_cast<size_t>(i)];
      error = error || (usage >= 1 && usage <= 3);
      if (usage >= 4 && !(usage >= 0xE0 && usage <= 0xE7)) {
        bool held = false;
        for (const uint8_t old : previous) {
          held = held || old == usage;
        }
        only_held_keys = only_held_keys && held;
      }
    }
    const std::string bytes = Feed(&state, report);
    for (const char ch : bytes) {
      const auto byte = static_cast<uint8_t>(ch);
      EXPECT_TRUE(byte == 0x08 || byte == 0x09 || byte == 0x0D ||
                  (byte >= 0x20 && byte <= 0x7E))
          << "byte 0x" << std::hex << int{byte};
    }
    if (error || only_held_keys) {
      EXPECT_THAT(bytes, t::IsEmpty());
    }
    if (!error) {
      previous.assign(report.begin() + 2, report.end());
    }
  }
}

TEST(UsbHidKeyboardTest, CapsLockSharedAcrossKeyboardsViaKeyboardLocks) {
  KeyboardLocks locks;
  HidKeyboardState kbd_a;
  HidKeyboardState kbd_b;
  char out[kBootReportKeys] = {};
  const std::vector<uint8_t> caps = Report(0, {kCapsLock});
  const std::vector<uint8_t> rel = Report(0, {});
  const std::vector<uint8_t> key_a = Report(0, {0x04});

  EXPECT_THAT(TranslateBootReport(&kbd_a, &locks, caps.data(), 8, out, 6),
              t::Eq(0));
  EXPECT_TRUE(locks.caps_lock);
  EXPECT_THAT(TranslateBootReport(&kbd_b, &locks, key_a.data(), 8, out, 6),
              t::Eq(1));
  EXPECT_THAT(out[0], t::Eq('A'));
  // Held 'a' on kbd_b is still tracked per-keyboard: pressing 'a' on kbd_a
  // is a new press on kbd_a.
  EXPECT_THAT(TranslateBootReport(&kbd_a, &locks, key_a.data(), 8, out, 6),
              t::Eq(1));
  EXPECT_THAT(out[0], t::Eq('A'));
  // Toggling Caps Lock on kbd_b turns it off for both.
  TranslateBootReport(&kbd_b, &locks, rel.data(), 8, out, 6);
  TranslateBootReport(&kbd_b, &locks, caps.data(), 8, out, 6);
  EXPECT_FALSE(locks.caps_lock);
  TranslateBootReport(&kbd_a, &locks, rel.data(), 8, out, 6);
  EXPECT_THAT(TranslateBootReport(&kbd_a, &locks, key_a.data(), 8, out, 6),
              t::Eq(1));
  EXPECT_THAT(out[0], t::Eq('a'));
}

}  // namespace
}  // namespace protos::usb
