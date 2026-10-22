#ifndef PROTOS_USER_USB_HID_H_
#define PROTOS_USER_USB_HID_H_

#include <cstdint>
#include <type_traits>

// Translation of USB HID boot-protocol keyboard reports (HID 1.11 appendix B,
// HID Usage Tables page 0x07) into console bytes for the shell, with a US
// layout. Freestanding and header-only (see user/xhci_platform.h); pure
// functions plus a small per-keyboard state.
//
// The bytes suit the shell's line editor (user/sh_lib.h): Enter is '\r',
// Backspace is 0x08 and Tab is '\t'. Delete produces nothing (0x7F would
// erase like Backspace), and no other control bytes are produced: a key
// pressed while Ctrl, Alt or GUI is held produces nothing, because Ctrl+H,
// Ctrl+I, Ctrl+J and Ctrl+M would be editing keys for the shell.

namespace protos::usb {

// A boot report: modifier bits, a reserved byte and up to six key usages.
constexpr int kBootReportSize = 8;
constexpr int kBootReportKeys = 6;
// The shortest report that is translated (modifiers, reserved, one key).
constexpr int kBootReportMinSize = 3;

// Modifier bits (report byte 0).
constexpr uint8_t kHidModLeftCtrl = 1u << 0;
constexpr uint8_t kHidModLeftShift = 1u << 1;
constexpr uint8_t kHidModLeftAlt = 1u << 2;
constexpr uint8_t kHidModLeftGui = 1u << 3;
constexpr uint8_t kHidModRightCtrl = 1u << 4;
constexpr uint8_t kHidModRightShift = 1u << 5;
constexpr uint8_t kHidModRightAlt = 1u << 6;
constexpr uint8_t kHidModRightGui = 1u << 7;
constexpr uint8_t kHidModShiftMask = kHidModLeftShift | kHidModRightShift;
// Keys pressed while one of these is held produce no byte.
constexpr uint8_t kHidModCommandMask = kHidModLeftCtrl | kHidModLeftAlt |
                                       kHidModLeftGui | kHidModRightCtrl |
                                       kHidModRightAlt | kHidModRightGui;

// Keyboard usages (HID Usage Tables page 0x07).
constexpr uint8_t kHidUsageErrorRollOver = 0x01;
constexpr uint8_t kHidUsagePostFail = 0x02;
constexpr uint8_t kHidUsageErrorUndefined = 0x03;
constexpr uint8_t kHidUsageA = 0x04;
constexpr uint8_t kHidUsageZ = 0x1D;
constexpr uint8_t kHidUsage1 = 0x1E;
constexpr uint8_t kHidUsage0 = 0x27;
constexpr uint8_t kHidUsageEnter = 0x28;
constexpr uint8_t kHidUsageEscape = 0x29;
constexpr uint8_t kHidUsageBackspace = 0x2A;
constexpr uint8_t kHidUsageTab = 0x2B;
constexpr uint8_t kHidUsageSpace = 0x2C;
constexpr uint8_t kHidUsageCapsLock = 0x39;
constexpr uint8_t kHidUsageDelete = 0x4C;
constexpr uint8_t kHidUsageKeypad1 = 0x59;
constexpr uint8_t kHidUsageKeypad9 = 0x61;
constexpr uint8_t kHidUsageLeftCtrl = 0xE0;
constexpr uint8_t kHidUsageRightGui = 0xE7;

// Console-wide lock state shared across all keyboards and controllers (off at
// driver start).
struct KeyboardLocks {
  bool caps_lock = false;
};

static_assert(std::is_trivially_destructible_v<KeyboardLocks>);

// Default console-wide lock state used when a controller is not given an
// explicit `KeyboardLocks*`.
inline constinit KeyboardLocks g_default_keyboard_locks{};

// The state of one keyboard: the keys of its previous report and Caps Lock.
// Zero-initialized = no key down, Caps Lock off.
struct HidKeyboardState {
  uint8_t previous_keys[kBootReportKeys] = {};
  bool caps_lock = false;
};

static_assert(std::is_trivially_destructible_v<HidKeyboardState>);

// The byte that the key with `usage` types with a US layout, or 0 if it types
// none. Letters are uppercase if exactly one of `shift` and `caps_lock` is
// set; digits and symbols depend on `shift` only. Keypad keys type their
// digit or symbol regardless of Num Lock.
inline char TranslateUsage(const uint8_t usage,  //
                           const bool shift,     //
                           const bool caps_lock) {
  if (usage >= kHidUsageA && usage <= kHidUsageZ) {
    const char first = (shift != caps_lock) ? 'A' : 'a';
    return static_cast<char>(first + (usage - kHidUsageA));
  }
  if (usage >= kHidUsage1 && usage <= kHidUsage0) {
    const char* const row = shift ? "!@#$%^&*()" : "1234567890";
    return row[usage - kHidUsage1];
  }
  if (usage >= kHidUsageKeypad1 && usage <= kHidUsageKeypad9) {
    return static_cast<char>('1' + (usage - kHidUsageKeypad1));
  }
  switch (usage) {
    case kHidUsageEnter:
    case 0x58:  // Keypad Enter
      return '\r';
    case kHidUsageBackspace:
      return '\b';
    case kHidUsageTab:
      return '\t';
    case kHidUsageSpace:
      return ' ';
    case 0x2D:
      return shift ? '_' : '-';
    case 0x2E:
      return shift ? '+' : '=';
    case 0x2F:
      return shift ? '{' : '[';
    case 0x30:
      return shift ? '}' : ']';
    case 0x31:  // Backslash
    case 0x32:  // Non-US # and ~ (the backslash key of a US layout)
    case 0x64:  // Non-US \ and | (the 102nd key)
      return shift ? '|' : '\\';
    case 0x33:
      return shift ? ':' : ';';
    case 0x34:
      return shift ? '"' : '\'';
    case 0x35:
      return shift ? '~' : '`';
    case 0x36:
      return shift ? '<' : ',';
    case 0x37:
      return shift ? '>' : '.';
    case 0x38:
      return shift ? '?' : '/';
    case 0x54:  // Keypad /
      return '/';
    case 0x55:  // Keypad *
      return '*';
    case 0x56:  // Keypad -
      return '-';
    case 0x57:  // Keypad +
      return '+';
    case 0x62:  // Keypad 0
      return '0';
    case 0x63:  // Keypad .
      return '.';
    case 0x67:  // Keypad =
      return '=';
    default:
      return 0;
  }
}

namespace hid_internal {

// Whether key slot `index` of `keys` holds a key that was pressed since the
// previous report: a key usage (not 0, an error code or a modifier), not
// already in an earlier slot, and not in the previous report (in any slot:
// keyboards may reorder the array, e.g. QEMU moves the last key into the slot
// of a released one).
inline bool IsNewKeyPress(const HidKeyboardState& state,
                          const uint8_t (&keys)[kBootReportKeys],
                          const int index) {
  const uint8_t usage = keys[index];
  if (usage < kHidUsageA ||
      (usage >= kHidUsageLeftCtrl && usage <= kHidUsageRightGui)) {
    return false;
  }
  for (int i = 0; i < index; ++i) {
    if (keys[i] == usage) {
      return false;
    }
  }
  for (const uint8_t previous : state.previous_keys) {
    if (previous == usage) {
      return false;
    }
  }
  return true;
}

}  // namespace hid_internal

// Translates one boot report of `length` bytes (only the first
// kBootReportSize are used; missing key slots count as empty) into at most
// `out_capacity` bytes at `out`, and returns their number. Keys produce bytes
// when pressed, in key-slot order; held keys don't repeat. Caps Lock toggles
// when pressed, before the other keys of the same report are translated.
// Reports shorter than kBootReportMinSize and reports with an error code in a
// key slot (ErrorRollOver: too many keys down) are ignored entirely: no
// bytes, state unchanged. An `out_capacity` of kBootReportKeys always
// suffices.
inline int TranslateBootReport(HidKeyboardState* const state,  //
                               const uint8_t* const report,    //
                               const int length,               //
                               char* const out,                //
                               const int out_capacity) {
  if (state == nullptr || report == nullptr || length < kBootReportMinSize) {
    return 0;
  }
  uint8_t keys[kBootReportKeys] = {};
  const int key_bytes =
      (length < kBootReportSize ? length : kBootReportSize) - 2;
  for (int i = 0; i < key_bytes; ++i) {
    keys[i] = report[2 + i];
  }
  for (const uint8_t usage : keys) {
    if (usage == kHidUsageErrorRollOver || usage == kHidUsagePostFail ||
        usage == kHidUsageErrorUndefined) {
      return 0;
    }
  }
  const uint8_t modifiers = report[0];
  const bool shift = (modifiers & kHidModShiftMask) != 0;
  const bool command = (modifiers & kHidModCommandMask) != 0;
  for (int i = 0; i < kBootReportKeys; ++i) {
    if (keys[i] == kHidUsageCapsLock &&
        hid_internal::IsNewKeyPress(*state, keys, i)) {
      state->caps_lock = !state->caps_lock;
    }
  }
  int count = 0;
  for (int i = 0; i < kBootReportKeys; ++i) {
    if (command || keys[i] == kHidUsageCapsLock ||
        !hid_internal::IsNewKeyPress(*state, keys, i)) {
      continue;
    }
    const char ch = TranslateUsage(keys[i], shift, state->caps_lock);
    if (ch != 0 && out != nullptr && count < out_capacity) {
      out[count++] = ch;
    }
  }
  for (int i = 0; i < kBootReportKeys; ++i) {
    state->previous_keys[i] = keys[i];
  }
  return count;
}

// Translates one boot report using a shared `KeyboardLocks` state so that
// Caps Lock is global across keyboards while `state->previous_keys` remains
// per-keyboard.
inline int TranslateBootReport(HidKeyboardState* const state,  //
                               KeyboardLocks* const locks,     //
                               const uint8_t* const report,    //
                               const int length,               //
                               char* const out,                //
                               const int out_capacity) {
  if (state != nullptr && locks != nullptr) {
    state->caps_lock = locks->caps_lock;
  }
  const int count =
      TranslateBootReport(state, report, length, out, out_capacity);
  if (state != nullptr && locks != nullptr) {
    locks->caps_lock = state->caps_lock;
  }
  return count;
}

}  // namespace protos::usb

#endif  // PROTOS_USER_USB_HID_H_
