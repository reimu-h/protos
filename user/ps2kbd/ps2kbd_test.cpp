#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <deque>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

#include "user/ps2kbd/i8042.h"
#include "user/ps2kbd/ps2_scancode.h"

namespace protos::ps2 {
namespace {

namespace t = ::testing;

std::string FeedStream(Ps2ScancodeDecoder* const decoder,
                       const std::initializer_list<uint8_t> bytes) {
  std::string out;
  for (const uint8_t b : bytes) {
    const char ch = decoder->FeedByte(b);
    if (ch != 0) {
      out.push_back(ch);
    }
  }
  return out;
}

TEST(Ps2ScancodeTest, Set1TranslatesAlphabetDigitsSymbolsAndEditingKeys) {
  Ps2ScancodeDecoder dec;
  dec.Reset(Ps2ScancodeSet::kSet1);

  // Unshifted qwerty row + digits + space + enter + backspace + tab.
  EXPECT_EQ(FeedStream(&dec,
                       {
                           0x10, 0x90,  // q
                           0x11, 0x91,  // w
                           0x12, 0x92,  // e
                           0x1E, 0x9E,  // a
                           0x2C, 0xAC,  // z
                           0x39, 0xB9,  // space
                           0x02, 0x82,  // 1
                           0x0B, 0x8B,  // 0
                           0x0C, 0x8C,  // -
                           0x0D, 0x8D,  // =
                           0x0F, 0x8F,  // tab
                           0x0E, 0x8E,  // backspace
                           0x1C, 0x9C,  // enter
                       }),
            "qweaz 10-=\t\b\r");

  // Keypad keys and E0 Keypad Enter / Keypad Slash, plus ignored
  // navigation/Esc.
  EXPECT_EQ(FeedStream(&dec,
                       {
                           0x01, 0x81,              // Esc (ignored)
                           0xE0, 0x53, 0xE0, 0xD3,  // Delete (ignored)
                           0xE0, 0x48, 0xE0, 0xC8,  // Up (ignored)
                           0x4F, 0xCF,              // KP 1
                           0x52, 0xD2,              // KP 0
                           0x37, 0xB7,              // KP *
                           0xE0, 0x35, 0xE0, 0xB5,  // KP /
                           0xE0, 0x1C, 0xE0, 0x9C,  // KP Enter
                       }),
            "10*/\r");
}

TEST(Ps2ScancodeTest, Set1LeftShiftBreak0xAADoesNotStickAndBat0xAAIsHarmless) {
  Ps2ScancodeDecoder dec;
  dec.Reset(Ps2ScancodeSet::kSet1);

  // Unsolicited 0xAA when Left Shift is up is a harmless no-op.
  EXPECT_EQ(FeedStream(&dec, {0xAA, 0x1E, 0x9E}), "a");

  // Left Shift make (0x2A), 'a' -> 'A', '2' -> '@', Left Shift break (0xAA),
  // then 'a' -> 'a' and '2' -> '2'.
  EXPECT_EQ(FeedStream(&dec,
                       {
                           0x2A,  // LShift down
                           0x1E,
                           0x9E,  // A
                           0x03,
                           0x83,  // @
                           0xAA,  // LShift up (0x2A | 0x80 == 0xAA)
                           0x1E,
                           0x9E,  // a
                           0x03,
                           0x83,  // 2
                       }),
            "A@a2");
}

TEST(Ps2ScancodeTest, Set2TranslatesShiftedSymbolsCapsLockAndOverlappingShift) {
  Ps2ScancodeDecoder dec;
  dec.Reset(Ps2ScancodeSet::kSet2);

  // LShift down (0x12), RShift down (0x59), type 'h' (0x33) -> 'H', release
  // LShift (F0 12) -> RShift still held so 'i' (0x43) -> 'I' and '1' (0x16) ->
  // '!', release RShift (F0 59) -> 'i' -> 'i'.
  EXPECT_EQ(FeedStream(&dec,
                       {
                           0x12,  // LShift down
                           0x59,  // RShift down
                           0x33,
                           0xF0,
                           0x33,  // H
                           0xF0,
                           0x12,  // LShift up
                           0x43,
                           0xF0,
                           0x43,  // I
                           0x16,
                           0xF0,
                           0x16,  // !
                           0xF0,
                           0x59,  // RShift up
                           0x43,
                           0xF0,
                           0x43,  // i
                       }),
            "HI!i");

  // Caps Lock (0x58) toggles only on initial make, inverts letters with Shift,
  // and leaves digits unshifted. In Set 2, 0xAA is ignored as BAT completion.
  EXPECT_EQ(FeedStream(&dec,
                       {
                           0xAA,        // BAT pass ignored in Set 2
                           0x58, 0x58,  // Caps Lock make + repeat (1 toggle)
                           0xF0, 0x58,  // Caps Lock break
                           0x1C, 0xF0, 0x1C,  // A
                           0x16, 0xF0, 0x16,  // 1 (unaffected by Caps Lock)
                           0x12,              // LShift down
                           0x1C, 0xF0, 0x1C,  // a (Shift + Caps Lock)
                           0xF0, 0x12,        // LShift up
                           0x58, 0xF0, 0x58,  // Caps Lock off
                           0x1C, 0xF0, 0x1C,  // a
                       }),
            "A1aa");
}

TEST(Ps2ScancodeTest, SuppressesFakeShiftsPauseSequenceTypematicAndCtrlAltGui) {
  Ps2ScancodeDecoder set1;
  set1.Reset(Ps2ScancodeSet::kSet1);

  // Fake LShift (E0 2A / E0 AA) must neither press Shift when up nor release
  // real LShift when held. Pause (E1 1D 45 E1 9D C5) must not press LCtrl (1D).
  EXPECT_EQ(FeedStream(&set1,
                       {
                           0xE0, 0x2A, 0x1E, 0x9E, 0xE0, 0xAA,  // 'a'
                           0x2A,                                // Real LShift
                           0xE0, 0xAA,  // Fake LShift up
                           0x1E, 0x9E,  // Still 'A'!
                           0xAA,        // Real LShift up
                           0xE1, 0x1D, 0x45, 0xE1, 0x9D, 0xC5,  // Pause
                           0x1E, 0x1E, 0x1E, 0x9E,  // Typematic 'a' -> once
                       }),
            "aAa");

  // Command modifiers (LCtrl, RCtrl, LAlt, RAlt, LGui, RGui) suppress keys.
  EXPECT_EQ(FeedStream(&set1,
                       {
                           0x1D, 0x1E, 0x9E, 0x9D,              // LCtrl+a
                           0xE0, 0x1D, 0x1E, 0x9E, 0xE0, 0x9D,  // RCtrl+a
                           0x38, 0x1E, 0x9E, 0xB8,              // LAlt+a
                           0xE0, 0x38, 0x1E, 0x9E, 0xE0, 0xB8,  // RAlt+a
                           0xE0, 0x5B, 0x1E, 0x9E, 0xE0, 0xDB,  // LGui+a
                           0x1E, 0x9E,                          // plain 'a'
                       }),
            "a");

  Ps2ScancodeDecoder set2;
  set2.Reset(Ps2ScancodeSet::kSet2);
  // Set 2 8-byte Pause (E1 14 77 E1 F0 14 F0 77) and Fake RShift (E0 59 / E0 F0
  // 59).
  EXPECT_EQ(FeedStream(&set2,
                       {
                           0xE1, 0x14, 0x77, 0xE1, 0xF0, 0x14, 0xF0, 0x77,
                           0x59,              // Real RShift down
                           0xE0, 0xF0, 0x59,  // Fake RShift break
                           0x1C, 0xF0, 0x1C,  // Still 'A'!
                           0xF0, 0x59,        // Real RShift up
                           0x1C, 0xF0, 0x1C,  // 'a'
                       }),
            "Aa");
}

TEST(Ps2ScancodeTest,
     TruncatedE1OrAbortedF0PrefixDoesNotCorruptSubsequentKeys) {
  Ps2ScancodeDecoder set1;
  set1.Reset(Ps2ScancodeSet::kSet1);
  // Truncated 0xE1 interrupted by protocol ACK (0xFA) or overrun (0x00) must
  // reset `e1_remaining_` so the following keystroke ('a' = 0x1E, 0x9E) is not
  // swallowed.
  EXPECT_EQ(FeedStream(&set1, {0xE1, 0xFA, 0x1E, 0x9E}), "a");
  EXPECT_EQ(FeedStream(&set1, {0xE1, 0x1D, 0x00, 0x1E, 0x9E}), "a");

  Ps2ScancodeDecoder set2;
  set2.Reset(Ps2ScancodeSet::kSet2);
  // Truncated Set 2 0xE1 interrupted by 0xFA ACK or 0xAA BAT must not swallow
  // a Shift break prefix (`0xF0 0x12`) or subsequent keystroke (`0x1C` = 'a').
  EXPECT_EQ(FeedStream(&set2, {0x12, 0xE1, 0xFA, 0xF0, 0x12, 0x1C, 0xF0, 0x1C}),
            "a");
  EXPECT_EQ(FeedStream(&set2, {0xE1, 0xAA, 0x1C, 0xF0, 0x1C}), "a");

  // Stale `0xF0` before an `0xE0`-prefixed make code (`0xE0 0x4A` = KP '/',
  // `0xE0 0x14` = RCtrl) must clear `f0_break_` when `0xE0` arrives so the
  // extended make code is treated as a press rather than a break.
  EXPECT_EQ(FeedStream(&set2, {0xF0, 0xE0, 0x4A, 0xE0, 0xF0, 0x4A}), "/");
  EXPECT_EQ(FeedStream(&set2, {0xF0, 0xE0, 0x14, 0x1C, 0xF0, 0x1C, 0xE0, 0xF0,
                               0x14, 0x1C, 0xF0, 0x1C}),
            "a");
}

class FakeI8042Platform final : public Platform {
 public:
  enum class Mode {
    kPresent,
    kFloating0xFF,
    kSilent0x00,
  };

  explicit FakeI8042Platform(const Mode mode = Mode::kPresent) : mode_(mode) {}

  void set_supports_translate(const bool supported) {
    supports_translate_ = supported;
  }
  void set_f4_replies(std::deque<uint8_t> replies) {
    f4_replies_ = std::move(replies);
  }
  void set_ccb(const uint8_t ccb) { ccb_ = ccb; }

  void EnqueueByte(const uint8_t data, const uint8_t extra_status = 0) {
    fifo_.push_back({data, extra_status});
  }

  // Adds `data` to the bytes that the keyboard buffers while its clock is
  // inhibited (`CCB` bit 4 set), which it sends once the clock runs again.
  void HoldKeyboardByte(const uint8_t data) { held_.push_back(data); }

  // Makes the status register read `status` from now on, like a wedged
  // controller (and the data port its last byte).
  void StickStatus(const uint8_t status) { stuck_status_ = status; }
  // Calls StickStatus(`status`) once the keyboard is sent `0xF4`.
  void StickStatusOnEnableScan(const uint8_t status) {
    stuck_status_on_enable_scan_ = status;
  }

  uint8_t In8(const uint16_t port) override {
    // A polling loop that would never end stops at a floating bus, failing
    // the test instead of hanging it.
    if (++reads_ > kMaxReads) {
      if (reads_ == kMaxReads + 1) {
        ADD_FAILURE() << "more than " << kMaxReads << " port reads";
      }
      return kStatusFloatingBus;
    }
    if (mode_ == Mode::kFloating0xFF) {
      return 0xFF;
    }
    if (mode_ == Mode::kSilent0x00) {
      return 0x00;
    }
    if (stuck_status_ != 0) {
      return port == kStatusCmdPort ? stuck_status_ : last_data_;
    }
    if (port == kStatusCmdPort) {
      if (fifo_.empty()) {
        return 0;
      }
      return static_cast<uint8_t>(kStatusOutputFull | fifo_.front().second);
    }
    if (port == kDataPort) {
      if (fifo_.empty()) {
        return last_data_;
      }
      last_data_ = fifo_.front().first;
      fifo_.pop_front();
      return last_data_;
    }
    return 0xFF;
  }

  void Out8(const uint16_t port, const uint8_t value) override {
    if (mode_ != Mode::kPresent) {
      return;
    }
    if (port == kStatusCmdPort) {
      commands_sent_.push_back(value);
      expecting_ccb_write_ = false;
      switch (value) {
        case kCmdDisableKbd:
          ccb_ |= kConfigKbdClockDisable;
          break;
        case kCmdEnableKbd:
          SetCcb(static_cast<uint8_t>(ccb_ & ~kConfigKbdClockDisable));
          break;
        case kCmdDisableAux:
          ccb_ |= kConfigAuxClockDisable;
          break;
        case kCmdReadConfig:
          fifo_.push_back({ccb_, 0});
          break;
        case kCmdWriteConfig:
          expecting_ccb_write_ = true;
          break;
        default:
          break;
      }
      return;
    }
    if (port == kDataPort) {
      data_sent_.push_back(value);
      if (expecting_ccb_write_) {
        expecting_ccb_write_ = false;
        SetCcb(supports_translate_
                   ? value
                   : static_cast<uint8_t>(value & ~kConfigKbdTranslate));
        return;
      }
      if (value == kDevCmdEnableScan) {
        scanning_enabled_ = true;
        if (stuck_status_on_enable_scan_ != 0) {
          stuck_status_ = stuck_status_on_enable_scan_;
        }
        if (!f4_replies_.empty()) {
          const uint8_t reply = f4_replies_.front();
          f4_replies_.pop_front();
          fifo_.push_back({reply, 0});
        }
      }
    }
  }

  int64_t NowTicks() override { return ticks_; }
  int64_t TicksPerSecond() override { return 250; }
  void Relax() override { ++ticks_; }

  void PushInput(const char* const bytes, const int count) override {
    pushed_.append(bytes, static_cast<size_t>(count));
  }

  const std::vector<uint8_t>& commands_sent() const { return commands_sent_; }
  const std::vector<uint8_t>& data_sent() const { return data_sent_; }
  const std::string& pushed() const { return pushed_; }
  uint8_t ccb() const { return ccb_; }
  bool scanning_enabled() const { return scanning_enabled_; }
  bool fifo_empty() const { return fifo_.empty(); }

 private:
  static constexpr int64_t kMaxReads = 100000;

  // Sets the `CCB`. A keyboard whose clock starts running sends its held bytes.
  void SetCcb(const uint8_t ccb) {
    const bool was_inhibited = (ccb_ & kConfigKbdClockDisable) != 0;
    ccb_ = ccb;
    if (was_inhibited && (ccb_ & kConfigKbdClockDisable) == 0) {
      for (const uint8_t data : held_) {
        fifo_.push_back({data, 0});
      }
      held_.clear();
    }
  }

  Mode mode_ = Mode::kPresent;
  bool supports_translate_ = true;
  bool expecting_ccb_write_ = false;
  bool scanning_enabled_ = false;
  uint8_t ccb_ = kConfigKbdTranslate;
  uint8_t last_data_ = 0;
  uint8_t stuck_status_ = 0;
  uint8_t stuck_status_on_enable_scan_ = 0;
  int64_t reads_ = 0;
  int64_t ticks_ = 100;
  std::deque<uint8_t> f4_replies_{kDevReplyAck};
  std::deque<std::pair<uint8_t, uint8_t>> fifo_;
  std::deque<uint8_t> held_;
  std::vector<uint8_t> commands_sent_;
  std::vector<uint8_t> data_sent_;
  std::string pushed_;
};

TEST(I8042ControllerTest, InitializesWithoutSelfTestAndDrainsKeystrokes) {
  FakeI8042Platform platform;
  platform.set_f4_replies({kDevReplyResend, kDevReplyAck});
  // Stale byte present at boot before Initialize() runs.
  platform.EnqueueByte(0x1E);

  I8042Controller ctrl;
  ASSERT_EQ(ctrl.Initialize(platform), I8042Status::kOk);

  // Never sends 0xAA controller self-test to 0x64.
  EXPECT_THAT(platform.commands_sent(), t::Not(t::Contains(uint8_t{0xAA})));
  EXPECT_EQ(ctrl.scancode_set(), Ps2ScancodeSet::kSet1);
  EXPECT_TRUE(platform.scanning_enabled());
  // IRQ 1 stays disabled in CCB until EnableIrq1() is called after IsaIrqAlloc.
  EXPECT_EQ(platform.ccb() & kConfigKbdIrqEnable, 0u);
  EXPECT_EQ(platform.ccb() & kConfigAuxIrqEnable, 0u);
  EXPECT_NE(platform.ccb() & kConfigAuxClockDisable, 0u);

  // Byte arriving just before EnableIrq1() must be drained immediately inside
  // EnableIrq1() so edge-triggered IRQ 1 cannot stall with OBF stuck high.
  platform.EnqueueByte(0x23);  // 'h' make
  platform.EnqueueByte(0xA3);  // 'h' break
  ASSERT_EQ(ctrl.EnableIrq1(), I8042Status::kOk);
  EXPECT_NE(platform.ccb() & kConfigKbdIrqEnable, 0u);
  EXPECT_TRUE(platform.fifo_empty());
  EXPECT_EQ(platform.pushed(), "h");

  // Subsequent IRQ wakeup ignores AUX mouse bytes and parity/timeout errors
  // while translating valid keyboard bytes.
  platform.EnqueueByte(0x17, kStatusAuxData);      // AUX mouse byte -> ignored
  platform.EnqueueByte(0x17, kStatusParityError);  // Parity error -> ignored
  platform.EnqueueByte(0x17, 0);                   // 'i' make
  platform.EnqueueByte(0x97, 0);                   // 'i' break
  platform.EnqueueByte(0x1C, 0);                   // Enter make
  platform.EnqueueByte(0x9C, 0);                   // Enter break
  ctrl.HandleInterrupt();
  EXPECT_EQ(platform.pushed(), "hi\r");
}

TEST(I8042ControllerTest, KeepsPort1DisabledUntilTheCcbIsReadBack) {
  FakeI8042Platform platform;
  // Port 1 is disabled at boot, and the keyboard holds a byte (Enter break)
  // that it sends as soon as its clock runs.
  platform.set_ccb(kConfigKbdClockDisable | kConfigKbdTranslate);
  platform.HoldKeyboardByte(0x9C);

  I8042Controller ctrl;
  ASSERT_EQ(ctrl.Initialize(platform), I8042Status::kOk);
  // The CCB read-back returned the CCB, not the keyboard byte (whose bit 6,
  // translation, is clear), ...
  EXPECT_EQ(ctrl.scancode_set(), Ps2ScancodeSet::kSet1);
  // ... so EnableIrq1() writes back the controller's own configuration.
  ASSERT_EQ(ctrl.EnableIrq1(), I8042Status::kOk);
  EXPECT_EQ(platform.ccb(),
            kConfigKbdIrqEnable | kConfigAuxClockDisable | kConfigKbdTranslate);
  EXPECT_TRUE(platform.fifo_empty());
  EXPECT_EQ(platform.pushed(), "");
}

TEST(I8042ControllerTest, FallsBackToSet2AndToleratesMissingF4Ack) {
  FakeI8042Platform platform;
  platform.set_supports_translate(false);
  platform.set_f4_replies({});  // No 0xFA ACK returned for 0xF4.

  I8042Controller ctrl;
  ASSERT_EQ(ctrl.Initialize(platform), I8042Status::kOk);
  EXPECT_EQ(ctrl.scancode_set(), Ps2ScancodeSet::kSet2);
  ASSERT_EQ(ctrl.EnableIrq1(), I8042Status::kOk);

  platform.EnqueueByte(0x1C);  // Set 2 'a' make
  platform.EnqueueByte(0xF0);
  platform.EnqueueByte(0x1C);  // Set 2 'a' break
  ctrl.HandleInterrupt();
  EXPECT_EQ(platform.pushed(), "a");
}

TEST(I8042ControllerTest, DetectsAbsentControllerOnFloatingOrSilentPort) {
  {
    FakeI8042Platform floating(FakeI8042Platform::Mode::kFloating0xFF);
    I8042Controller ctrl;
    EXPECT_EQ(ctrl.Initialize(floating), I8042Status::kNotPresent);
  }
  {
    FakeI8042Platform silent(FakeI8042Platform::Mode::kSilent0x00);
    I8042Controller ctrl;
    EXPECT_EQ(ctrl.Initialize(silent), I8042Status::kNotPresent);
  }
}

TEST(I8042ControllerTest, GivesUpOnAStatusStuckWithADiscardedByte) {
  // The status sticks at `OBF | AUXB` while the `CCB` is read: the read gives
  // up at its deadline, so the controller counts as absent.
  {
    FakeI8042Platform platform;
    platform.StickStatus(
        static_cast<uint8_t>(kStatusOutputFull | kStatusAuxData));
    I8042Controller ctrl;
    EXPECT_EQ(ctrl.Initialize(platform), I8042Status::kNotPresent);
  }
  // The status sticks with an AUX or error byte while the `0xF4` ACK is
  // awaited: the wait times out, which Initialize() tolerates.
  for (const uint8_t discarded :
       {kStatusAuxData, kStatusTimeoutError, kStatusParityError}) {
    FakeI8042Platform platform;
    platform.StickStatusOnEnableScan(
        static_cast<uint8_t>(kStatusOutputFull | discarded));
    I8042Controller ctrl;
    EXPECT_EQ(ctrl.Initialize(platform), I8042Status::kOk)
        << "status bits " << static_cast<int>(discarded);
    EXPECT_TRUE(platform.scanning_enabled());
  }
}

}  // namespace
}  // namespace protos::ps2
