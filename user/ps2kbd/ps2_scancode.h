#ifndef PROTOS_USER_PS2KBD_PS2_SCANCODE_H_
#define PROTOS_USER_PS2KBD_PS2_SCANCODE_H_

#include <cstdint>
#include <type_traits>

#include "user/usbkbd/usb_hid.h"

// Translation of PS/2 keyboard Scancode Set 1 (XT / i8042-translated) and
// Scancode Set 2 (AT / raw) byte streams into US-layout console bytes via
// `protos::usb::TranslateUsage()`. Freestanding and header-only.
//
// Protocol subtleties handled:
// - Set 1 Left Shift make is `0x2A`, so Left Shift break is `0x2A | 0x80 =
//   0xAA`. `0xAA` must therefore be decoded as Left Shift break in Set 1 (if
//   Left Shift was not held, clearing its bit in `down_bitmap_` is a harmless
//   no-op, which also tolerates an unsolicited `0xAA` BAT byte).
// - Protocol bytes (`0x00`, `0xFF`, `0xFA` ACK, `0xFE` Resend, `0xEE` Echo, and
//   in Set 2 `0xAA`/`0xFC`/`0xFD` BAT responses) reset multi-byte prefix state
//   without emitting characters or corrupting held modifiers.
// - Fake Shifts emitted around Print Screen and grey navigation keys (`E0 2A` /
//   `E0 AA` / `E0 36` / `E0 B6` in Set 1; `E0 12` / `E0 F0 12` / `E0 59` /
//   `E0 F0 59` in Set 2) are ignored so they never press or release real Left
//   or Right Shift.
// - Pause/Break (`E1 1D 45 E1 9D C5` in Set 1; `E1 14 77 E1 F0 14 F0 77` in
//   Set 2) consumes two non-`0xF0` payload bytes after each `0xE1` prefix so
//   embedded `1D`/`14` bytes never press Left Ctrl.
// - A 256-bit `down_bitmap_` tracks held HID usages (`0x00..0xFF`), suppressing
//   typematic auto-repeat on held keys (matching `usbkbd`), tracking Left and
//   Right Shift/Ctrl/Alt/GUI independently, and toggling Caps Lock only on an
//   initial make transition.

namespace protos::ps2 {

enum class Ps2ScancodeSet : uint8_t {
  kSet1 = 1,
  kSet2 = 2,
};

namespace scancode_internal {

constexpr uint8_t kHidLeftCtrl = usb::kHidUsageLeftCtrl;  // 0xE0
constexpr uint8_t kHidLeftShift = 0xE1;
constexpr uint8_t kHidLeftAlt = 0xE2;
constexpr uint8_t kHidLeftGui = 0xE3;
constexpr uint8_t kHidRightCtrl = 0xE4;
constexpr uint8_t kHidRightShift = 0xE5;
constexpr uint8_t kHidRightAlt = 0xE6;
constexpr uint8_t kHidRightGui = usb::kHidUsageRightGui;  // 0xE7

constexpr uint8_t MapSet1MakeToUsage(const uint8_t make, const bool e0) {
  if (e0) {
    switch (make) {
      case 0x1C:
        return 0x58;  // Keypad Enter
      case 0x1D:
        return kHidRightCtrl;
      case 0x2A:
      case 0x36:
        return 0;  // Fake Left/Right Shift
      case 0x35:
        return 0x54;  // Keypad /
      case 0x37:
        return 0x46;  // Print Screen
      case 0x38:
        return kHidRightAlt;
      case 0x46:
        return 0x48;  // Ctrl+Break / Pause
      case 0x47:
        return 0x4A;  // Home
      case 0x48:
        return 0x52;  // Up Arrow
      case 0x49:
        return 0x4B;  // Page Up
      case 0x4B:
        return 0x50;  // Left Arrow
      case 0x4D:
        return 0x4F;  // Right Arrow
      case 0x4F:
        return 0x4D;  // End
      case 0x50:
        return 0x51;  // Down Arrow
      case 0x51:
        return 0x4E;  // Page Down
      case 0x52:
        return 0x49;  // Insert
      case 0x53:
        return usb::kHidUsageDelete;
      case 0x5B:
        return kHidLeftGui;
      case 0x5C:
        return kHidRightGui;
      case 0x5D:
        return 0x65;  // Application / Menu
      default:
        return 0;
    }
  }

  switch (make) {
    case 0x01:
      return usb::kHidUsageEscape;
    case 0x02:
      return 0x1E;  // 1
    case 0x03:
      return 0x1F;  // 2
    case 0x04:
      return 0x20;  // 3
    case 0x05:
      return 0x21;  // 4
    case 0x06:
      return 0x22;  // 5
    case 0x07:
      return 0x23;  // 6
    case 0x08:
      return 0x24;  // 7
    case 0x09:
      return 0x25;  // 8
    case 0x0A:
      return 0x26;  // 9
    case 0x0B:
      return 0x27;  // 0
    case 0x0C:
      return 0x2D;  // -
    case 0x0D:
      return 0x2E;  // =
    case 0x0E:
      return usb::kHidUsageBackspace;
    case 0x0F:
      return usb::kHidUsageTab;
    case 0x10:
      return 0x14;  // Q
    case 0x11:
      return 0x1A;  // W
    case 0x12:
      return 0x08;  // E
    case 0x13:
      return 0x15;  // R
    case 0x14:
      return 0x17;  // T
    case 0x15:
      return 0x1C;  // Y
    case 0x16:
      return 0x18;  // U
    case 0x17:
      return 0x0C;  // I
    case 0x18:
      return 0x12;  // O
    case 0x19:
      return 0x13;  // P
    case 0x1A:
      return 0x2F;  // [
    case 0x1B:
      return 0x30;  // ]
    case 0x1C:
      return usb::kHidUsageEnter;
    case 0x1D:
      return kHidLeftCtrl;
    case 0x1E:
      return 0x04;  // A
    case 0x1F:
      return 0x16;  // S
    case 0x20:
      return 0x07;  // D
    case 0x21:
      return 0x09;  // F
    case 0x22:
      return 0x0A;  // G
    case 0x23:
      return 0x0B;  // H
    case 0x24:
      return 0x0D;  // J
    case 0x25:
      return 0x0E;  // K
    case 0x26:
      return 0x0F;  // L
    case 0x27:
      return 0x33;  // ;
    case 0x28:
      return 0x34;  // '
    case 0x29:
      return 0x35;  // `
    case 0x2A:
      return kHidLeftShift;
    case 0x2B:
      return 0x31;  // Backslash
    case 0x2C:
      return 0x1D;  // Z
    case 0x2D:
      return 0x1B;  // X
    case 0x2E:
      return 0x06;  // C
    case 0x2F:
      return 0x19;  // V
    case 0x30:
      return 0x05;  // B
    case 0x31:
      return 0x11;  // N
    case 0x32:
      return 0x10;  // M
    case 0x33:
      return 0x36;  // ,
    case 0x34:
      return 0x37;  // .
    case 0x35:
      return 0x38;  // /
    case 0x36:
      return kHidRightShift;
    case 0x37:
      return 0x55;  // Keypad *
    case 0x38:
      return kHidLeftAlt;
    case 0x39:
      return usb::kHidUsageSpace;
    case 0x3A:
      return usb::kHidUsageCapsLock;
    case 0x3B:
    case 0x3C:
    case 0x3D:
    case 0x3E:
    case 0x3F:
    case 0x40:
    case 0x41:
    case 0x42:
    case 0x43:
    case 0x44:
      return static_cast<uint8_t>(0x3A + (make - 0x3B));  // F1..F10
    case 0x45:
      return 0x53;  // Num Lock
    case 0x46:
      return 0x47;  // Scroll Lock
    case 0x47:
      return 0x5F;  // Keypad 7
    case 0x48:
      return 0x60;  // Keypad 8
    case 0x49:
      return 0x61;  // Keypad 9
    case 0x4A:
      return 0x56;  // Keypad -
    case 0x4B:
      return 0x5C;  // Keypad 4
    case 0x4C:
      return 0x5D;  // Keypad 5
    case 0x4D:
      return 0x5E;  // Keypad 6
    case 0x4E:
      return 0x57;  // Keypad +
    case 0x4F:
      return 0x59;  // Keypad 1
    case 0x50:
      return 0x5A;  // Keypad 2
    case 0x51:
      return 0x5B;  // Keypad 3
    case 0x52:
      return 0x62;  // Keypad 0
    case 0x53:
      return 0x63;  // Keypad .
    case 0x56:
      return 0x64;  // Non-US \ and |
    case 0x57:
      return 0x44;  // F11
    case 0x58:
      return 0x45;  // F12
    case 0x59:
      return 0x67;  // Keypad =
    default:
      return 0;
  }
}

constexpr uint8_t MapSet2MakeToUsage(const uint8_t make, const bool e0) {
  if (e0) {
    switch (make) {
      case 0x11:
        return kHidRightAlt;
      case 0x12:
      case 0x59:
        return 0;  // Fake Left/Right Shift
      case 0x14:
        return kHidRightCtrl;
      case 0x1F:
        return kHidLeftGui;
      case 0x27:
        return kHidRightGui;
      case 0x2F:
        return 0x65;  // Application / Menu
      case 0x4A:
        return 0x54;  // Keypad /
      case 0x5A:
        return 0x58;  // Keypad Enter
      case 0x69:
        return 0x4D;  // End
      case 0x6B:
        return 0x50;  // Left Arrow
      case 0x6C:
        return 0x4A;  // Home
      case 0x70:
        return 0x49;  // Insert
      case 0x71:
        return usb::kHidUsageDelete;
      case 0x72:
        return 0x51;  // Down Arrow
      case 0x74:
        return 0x4F;  // Right Arrow
      case 0x75:
        return 0x52;  // Up Arrow
      case 0x7A:
        return 0x4E;  // Page Down
      case 0x7C:
        return 0x46;  // Print Screen
      case 0x7D:
        return 0x4B;  // Page Up
      case 0x7E:
        return 0x48;  // Ctrl+Break / Pause
      default:
        return 0;
    }
  }

  switch (make) {
    case 0x01:
      return 0x42;  // F9
    case 0x03:
      return 0x3E;  // F5
    case 0x04:
      return 0x3C;  // F3
    case 0x05:
      return 0x3A;  // F1
    case 0x06:
      return 0x3B;  // F2
    case 0x07:
      return 0x45;  // F12
    case 0x09:
      return 0x43;  // F10
    case 0x0A:
      return 0x41;  // F8
    case 0x0B:
      return 0x3F;  // F6
    case 0x0C:
      return 0x3D;  // F4
    case 0x0D:
      return usb::kHidUsageTab;
    case 0x0E:
      return 0x35;  // `
    case 0x0F:
      return 0x67;  // Keypad =
    case 0x11:
      return kHidLeftAlt;
    case 0x12:
      return kHidLeftShift;
    case 0x14:
      return kHidLeftCtrl;
    case 0x15:
      return 0x14;  // Q
    case 0x16:
      return 0x1E;  // 1
    case 0x1A:
      return 0x1D;  // Z
    case 0x1B:
      return 0x16;  // S
    case 0x1C:
      return 0x04;  // A
    case 0x1D:
      return 0x1A;  // W
    case 0x1E:
      return 0x1F;  // 2
    case 0x21:
      return 0x06;  // C
    case 0x22:
      return 0x1B;  // X
    case 0x23:
      return 0x07;  // D
    case 0x24:
      return 0x08;  // E
    case 0x25:
      return 0x21;  // 4
    case 0x26:
      return 0x20;  // 3
    case 0x29:
      return usb::kHidUsageSpace;
    case 0x2A:
      return 0x19;  // V
    case 0x2B:
      return 0x09;  // F
    case 0x2C:
      return 0x17;  // T
    case 0x2D:
      return 0x15;  // R
    case 0x2E:
      return 0x22;  // 5
    case 0x31:
      return 0x11;  // N
    case 0x32:
      return 0x05;  // B
    case 0x33:
      return 0x0B;  // H
    case 0x34:
      return 0x0A;  // G
    case 0x35:
      return 0x1C;  // Y
    case 0x36:
      return 0x23;  // 6
    case 0x3A:
      return 0x10;  // M
    case 0x3B:
      return 0x0D;  // J
    case 0x3C:
      return 0x18;  // U
    case 0x3D:
      return 0x24;  // 7
    case 0x3E:
      return 0x25;  // 8
    case 0x41:
      return 0x36;  // ,
    case 0x42:
      return 0x0E;  // K
    case 0x43:
      return 0x0C;  // I
    case 0x44:
      return 0x12;  // O
    case 0x45:
      return 0x27;  // 0
    case 0x46:
      return 0x26;  // 9
    case 0x49:
      return 0x37;  // .
    case 0x4A:
      return 0x38;  // /
    case 0x4B:
      return 0x0F;  // L
    case 0x4C:
      return 0x33;  // ;
    case 0x4D:
      return 0x13;  // P
    case 0x4E:
      return 0x2D;  // -
    case 0x52:
      return 0x34;  // '
    case 0x54:
      return 0x2F;  // [
    case 0x55:
      return 0x2E;  // =
    case 0x58:
      return usb::kHidUsageCapsLock;
    case 0x59:
      return kHidRightShift;
    case 0x5A:
      return usb::kHidUsageEnter;
    case 0x5B:
      return 0x30;  // ]
    case 0x5D:
      return 0x31;  // Backslash
    case 0x61:
      return 0x64;  // Non-US \ and |
    case 0x66:
      return usb::kHidUsageBackspace;
    case 0x69:
      return 0x59;  // Keypad 1
    case 0x6B:
      return 0x5C;  // Keypad 4
    case 0x6C:
      return 0x5F;  // Keypad 7
    case 0x70:
      return 0x62;  // Keypad 0
    case 0x71:
      return 0x63;  // Keypad .
    case 0x72:
      return 0x5A;  // Keypad 2
    case 0x73:
      return 0x5D;  // Keypad 5
    case 0x74:
      return 0x5E;  // Keypad 6
    case 0x75:
      return 0x60;  // Keypad 8
    case 0x76:
      return usb::kHidUsageEscape;
    case 0x77:
      return 0x53;  // Num Lock
    case 0x78:
      return 0x44;  // F11
    case 0x79:
      return 0x57;  // Keypad +
    case 0x7A:
      return 0x5B;  // Keypad 3
    case 0x7B:
      return 0x56;  // Keypad -
    case 0x7C:
      return 0x55;  // Keypad *
    case 0x7D:
      return 0x61;  // Keypad 9
    case 0x7E:
      return 0x47;  // Scroll Lock
    case 0x83:
      return 0x40;  // F7
    default:
      return 0;
  }
}

}  // namespace scancode_internal

class Ps2ScancodeDecoder {
 public:
  constexpr Ps2ScancodeDecoder() = default;

  constexpr void Reset(const Ps2ScancodeSet set = Ps2ScancodeSet::kSet1) {
    set_ = set;
    e0_prefix_ = false;
    f0_break_ = false;
    e1_remaining_ = 0;
    caps_lock_ = false;
    for (uint64_t& word : down_bitmap_) {
      word = 0;
    }
  }

  constexpr Ps2ScancodeSet scancode_set() const { return set_; }
  constexpr bool caps_lock() const { return caps_lock_; }

  constexpr bool IsUsageDown(const uint8_t usage) const {
    return (down_bitmap_[usage >> 6] & (uint64_t{1} << (usage & 63u))) != 0;
  }

  // Feeds one keyboard byte read from I/O port `0x60`. Returns the translated
  // US-layout console byte, or `0` if the byte completes no printable/editing
  // key press.
  constexpr char FeedByte(const uint8_t byte) {
    if (byte == 0x00 || byte == 0xFF || byte == 0xFA || byte == 0xFE ||
        byte == 0xEE) {
      ClearPrefixes();
      return 0;
    }
    if (set_ == Ps2ScancodeSet::kSet2 &&
        (byte == 0xAA || byte == 0xFC || byte == 0xFD)) {
      ClearPrefixes();
      return 0;
    }

    if (e1_remaining_ > 0) {
      if (set_ == Ps2ScancodeSet::kSet2 && byte == 0xF0) {
        return 0;
      }
      --e1_remaining_;
      return 0;
    }

    if (byte == 0xE1) {
      ClearPrefixes();
      e1_remaining_ = 2;
      return 0;
    }

    if (byte == 0xE0) {
      e0_prefix_ = true;
      f0_break_ = false;
      return 0;
    }

    if (set_ == Ps2ScancodeSet::kSet2 && byte == 0xF0) {
      f0_break_ = true;
      return 0;
    }

    const bool e0 = e0_prefix_;
    bool is_break = false;
    uint8_t usage = 0;
    if (set_ == Ps2ScancodeSet::kSet1) {
      is_break = (byte & 0x80u) != 0;
      usage = scancode_internal::MapSet1MakeToUsage(
          static_cast<uint8_t>(byte & 0x7Fu), e0);
    } else {
      is_break = f0_break_;
      usage = scancode_internal::MapSet2MakeToUsage(byte, e0);
    }
    ClearPrefixes();

    if (usage == 0) {
      return 0;
    }

    if (is_break) {
      SetUsageDown(usage, false);
      return 0;
    }

    const bool already_down = IsUsageDown(usage);
    SetUsageDown(usage, true);
    if (already_down) {
      return 0;
    }

    if (usage == usb::kHidUsageCapsLock) {
      caps_lock_ = !caps_lock_;
      return 0;
    }

    if (usage >= scancode_internal::kHidLeftCtrl &&
        usage <= scancode_internal::kHidRightGui) {
      return 0;
    }

    if (IsCommandModifierDown()) {
      return 0;
    }

    const bool shift = IsUsageDown(scancode_internal::kHidLeftShift) ||
                       IsUsageDown(scancode_internal::kHidRightShift);
    return usb::TranslateUsage(usage, shift, caps_lock_);
  }

 private:
  constexpr void ClearPrefixes() {
    e1_remaining_ = 0;
    e0_prefix_ = false;
    f0_break_ = false;
  }

  constexpr void SetUsageDown(const uint8_t usage, const bool down) {
    const uint64_t bit = uint64_t{1} << (usage & 63u);
    if (down) {
      down_bitmap_[usage >> 6] |= bit;
    } else {
      down_bitmap_[usage >> 6] &= ~bit;
    }
  }

  constexpr bool IsCommandModifierDown() const {
    return IsUsageDown(scancode_internal::kHidLeftCtrl) ||
           IsUsageDown(scancode_internal::kHidLeftAlt) ||
           IsUsageDown(scancode_internal::kHidLeftGui) ||
           IsUsageDown(scancode_internal::kHidRightCtrl) ||
           IsUsageDown(scancode_internal::kHidRightAlt) ||
           IsUsageDown(scancode_internal::kHidRightGui);
  }

  uint64_t down_bitmap_[4] = {};
  Ps2ScancodeSet set_ = Ps2ScancodeSet::kSet1;
  uint8_t e1_remaining_ = 0;
  bool e0_prefix_ = false;
  bool f0_break_ = false;
  bool caps_lock_ = false;
};

static_assert(std::is_trivially_destructible_v<Ps2ScancodeDecoder>);

}  // namespace protos::ps2

#endif  // PROTOS_USER_PS2KBD_PS2_SCANCODE_H_
