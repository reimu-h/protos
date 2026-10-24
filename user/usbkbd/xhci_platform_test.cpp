#include "user/usbkbd/xhci_platform.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <type_traits>

#include "user/usbkbd/usb_hid.h"
#include "user/usbkbd/xhci_controller.h"
#include "user/usbkbd/xhci_sim.h"

namespace protos::usb {
namespace {

namespace t = ::testing;

using xhci_sim::FakePlatform;
using xhci_sim::SimClock;

// The core's types can live in constinit globals of a ring-3 program (crt
// runs no dynamic initializers), and need no destructors to run.
constinit MappedMmioWindow g_window;
constinit XhciController g_controllers[2];
constinit HidKeyboardState g_keyboard;
static_assert(std::is_trivially_destructible_v<MappedMmioWindow>);
static_assert(std::is_trivially_destructible_v<XhciController>);
static_assert(std::is_trivially_destructible_v<XhciCaps>);
static_assert(std::is_trivially_destructible_v<LogLine>);
static_assert(std::is_trivially_destructible_v<HidKeyboardState>);
static_assert(!std::is_copy_constructible_v<XhciController>);
static_assert(!std::is_polymorphic_v<XhciController>);

TEST(XhciPlatformTest, ConstinitObjectsStartEmpty) {
  EXPECT_THAT(g_window.Size(), t::Eq(0));
  EXPECT_THAT(g_window.Read32(0), t::Eq(kMmioAllOnes));
  EXPECT_THAT(g_controllers[1].state(), t::Eq(XhciState::kUnbound));
  EXPECT_THAT(g_controllers[1].handoff_outcome(),
              t::Eq(HandoffOutcome::kNotRun));
  EXPECT_FALSE(g_keyboard.caps_lock);
}

TEST(XhciPlatformTest, TicksForMsRoundsUpAndAddsOneTick) {
  SimClock clock;
  FakePlatform platform(&clock);
  // 250 Hz: 4 ms per tick.
  EXPECT_THAT(TicksForMs(platform, 0), t::Eq(1));
  EXPECT_THAT(TicksForMs(platform, 1), t::Eq(2));
  EXPECT_THAT(TicksForMs(platform, 4), t::Eq(2));
  EXPECT_THAT(TicksForMs(platform, 5), t::Eq(3));
  EXPECT_THAT(TicksForMs(platform, 32), t::Eq(9));
  EXPECT_THAT(TicksForMs(platform, 1000), t::Eq(251));
  EXPECT_THAT(TicksForMs(platform, 5000), t::Eq(1251));
  EXPECT_THAT(TicksForMs(platform, 10000), t::Eq(2501));
  EXPECT_THAT(TicksForMs(platform, -5), t::Eq(1));
  platform.set_ticks_per_second(1000);
  EXPECT_THAT(TicksForMs(platform, 1), t::Eq(2));
  EXPECT_THAT(TicksForMs(platform, 32), t::Eq(33));
  // A broken rate does not divide by zero or overflow.
  platform.set_ticks_per_second(0);
  EXPECT_THAT(TicksForMs(platform, 1000), t::Eq(2));
  platform.set_ticks_per_second(INT64_MAX);
  EXPECT_THAT(TicksForMs(platform, INT64_MAX), t::Gt(0));
}

TEST(XhciPlatformTest, TicksToMsConvertsForLogLines) {
  SimClock clock;
  FakePlatform platform(&clock);
  EXPECT_THAT(TicksToMs(platform, 0), t::Eq(0));
  EXPECT_THAT(TicksToMs(platform, 3), t::Eq(12));
  EXPECT_THAT(TicksToMs(platform, 250), t::Eq(1000));
  EXPECT_THAT(TicksToMs(platform, -1), t::Eq(0));
  platform.set_ticks_per_second(0);
  EXPECT_THAT(TicksToMs(platform, 10), t::Eq(0));
}

TEST(XhciPlatformTest, WaitUntilRelaxesOnceBetweenPollsAndStopsWhenDone) {
  SimClock clock;
  FakePlatform platform(&clock);
  int polls = 0;
  EXPECT_TRUE(WaitUntil(platform, 1000, [&] { return ++polls == 5; }));
  EXPECT_THAT(polls, t::Eq(5));
  EXPECT_THAT(platform.relax_calls(), t::Eq(4));
  EXPECT_THAT(clock.now(), t::Eq(SimClock::kStartTick + 4));
}

TEST(XhciPlatformTest, WaitUntilTimesOutAtTheDeadline) {
  SimClock clock;
  FakePlatform platform(&clock);
  int polls = 0;
  EXPECT_FALSE(WaitUntil(platform, 1000, [&] {
    ++polls;
    return false;
  }));
  // 251 ticks: one poll per tick, plus the poll at the deadline and the final
  // check after it.
  EXPECT_THAT(clock.now() - SimClock::kStartTick, t::Eq(251));
  EXPECT_THAT(platform.relax_calls(), t::Eq(251));
  EXPECT_THAT(polls, t::Eq(253));
}

TEST(XhciPlatformTest, WaitUntilChecksOnceMoreAfterTheDeadline) {
  // A caller preempted past the deadline must not report a false timeout:
  // the condition became true while it was not running.
  SimClock clock;
  FakePlatform platform(&clock);
  int polls = 0;
  EXPECT_TRUE(WaitUntil(platform, 100, [&] {
    ++polls;
    if (polls == 1) {
      clock.Advance(1000);  // Preempted for 4 s
      return false;
    }
    return true;
  }));
  EXPECT_THAT(polls, t::Eq(2));
  EXPECT_THAT(platform.relax_calls(), t::Eq(0));
}

TEST(XhciPlatformTest, WaitUntilMeasuresTimeoutsInTicksNotPolls) {
  SimClock clock;
  FakePlatform platform(&clock);
  platform.set_relaxes_per_tick(10);
  EXPECT_FALSE(WaitUntil(platform, 32, [] { return false; }));
  EXPECT_THAT(clock.now() - SimClock::kStartTick, t::Eq(9));
  EXPECT_THAT(platform.relax_calls(), t::Eq(90));
}

TEST(XhciPlatformTest, DelayMsWaitsTheRoundedTicks) {
  SimClock clock;
  FakePlatform platform(&clock);
  DelayMs(platform, 1);
  EXPECT_THAT(clock.now() - SimClock::kStartTick, t::Eq(2));
  DelayMs(platform, 10);
  EXPECT_THAT(clock.now() - SimClock::kStartTick, t::Eq(2 + 4));
}

TEST(XhciPlatformTest, LogLineFormatsNumbersAndEndsWithNewline) {
  SimClock clock;
  FakePlatform platform(&clock);
  LogLine line;
  line.Str(kLogPrefix)
      .Str("dec ")
      .Dec(0)
      .Str(" ")
      .Dec(-42)
      .Str(" ")
      .Dec(INT64_MIN)
      .Str(" hex ")
      .Hex(0xABCu, 1)
      .Str(" ")
      .Hex(0x5u, 4)
      .Str(" ")
      .Hex(UINT64_MAX, 1)
      .Str(nullptr);
  EXPECT_THAT(std::string(line.data(), static_cast<size_t>(line.length())),
              t::Eq("[USBKBD] dec 0 -42 -9223372036854775808 hex abc 0005 "
                    "ffffffffffffffff"));
  line.Emit(platform);
  EXPECT_THAT(platform.logs(),
              t::ElementsAre("[USBKBD] dec 0 -42 -9223372036854775808 hex abc "
                             "0005 ffffffffffffffff"));
}

TEST(XhciPlatformTest, LogLineTruncatesToOneLineOfAtMost128Bytes) {
  SimClock clock;
  FakePlatform platform(&clock);
  LogLine line;
  line.Str(kLogPrefix);
  for (int i = 0; i < 30; ++i) {
    line.Str("abcdefgh");
  }
  EXPECT_THAT(line.length(), t::Eq(kMaxLogLineLength - 1));
  line.Emit(platform);
  ASSERT_THAT(platform.logs().size(), t::Eq(1u));
  EXPECT_THAT(platform.logs()[0].size(),
              t::Eq(static_cast<size_t>(kMaxLogLineLength - 1)));
}

TEST(XhciPlatformTest, FormatBdfIsLowercaseHex) {
  char text[8];
  FormatBdf(0, 4, 0, text);
  EXPECT_THAT(std::string(text), t::Eq("00:04.0"));
  FormatBdf(0xAB, 0x1F, 7, text);
  EXPECT_THAT(std::string(text), t::Eq("ab:1f.7"));
  FormatBdf(0x1FF, 0x3F, 0xF, text);  // Out-of-range parts are masked
  EXPECT_THAT(std::string(text), t::Eq("ff:1f.7"));
}

TEST(XhciPlatformTest, MappedMmioWindowAccessesDwordsInsideTheWindow) {
  alignas(4) uint32_t registers[4] = {0x11111111u, 0x22222222u, 0x33333333u,
                                      0x44444444u};
  MappedMmioWindow window;
  window.Init(reinterpret_cast<volatile uint8_t*>(registers), 16);
  EXPECT_THAT(window.Size(), t::Eq(16));
  EXPECT_THAT(window.Read32(0), t::Eq(0x11111111u));
  EXPECT_THAT(window.Read32(12), t::Eq(0x44444444u));
  window.Write32(4, 0xCAFEF00Du);
  EXPECT_THAT(registers[1], t::Eq(0xCAFEF00Du));
  EXPECT_THAT(window.Read32(4), t::Eq(0xCAFEF00Du));
  // Outside the window or unaligned: not performed.
  EXPECT_THAT(window.Read32(16), t::Eq(kMmioAllOnes));
  EXPECT_THAT(window.Read32(-4), t::Eq(kMmioAllOnes));
  EXPECT_THAT(window.Read32(2), t::Eq(kMmioAllOnes));
  window.Write32(16, 0);
  window.Write32(6, 0);
  window.Write32(-4, 0);
  EXPECT_THAT(registers[0], t::Eq(0x11111111u));
  EXPECT_THAT(registers[1], t::Eq(0xCAFEF00Du));
  EXPECT_THAT(registers[2], t::Eq(0x33333333u));
  EXPECT_THAT(registers[3], t::Eq(0x44444444u));
  // No mapping: nothing is accessible.
  window.Init(nullptr, 16);
  EXPECT_THAT(window.Size(), t::Eq(0));
  EXPECT_THAT(window.Read32(0), t::Eq(kMmioAllOnes));
}

TEST(XhciPlatformTest,
     MappedMmioWindowSupportsNonContiguousSegmentsAroundMsixHole) {
  alignas(4) uint32_t low_regs[4] = {0x10u, 0x20u, 0x30u, 0x40u};
  alignas(4) uint32_t high_regs[4] = {0x50u, 0x60u, 0x70u, 0x80u};
  MappedMmioWindow window;
  ASSERT_TRUE(
      window.AddSegment(0, reinterpret_cast<volatile uint8_t*>(low_regs), 16));
  ASSERT_TRUE(window.AddSegment(
      32, reinterpret_cast<volatile uint8_t*>(high_regs), 16));
  EXPECT_THAT(window.Size(), t::Eq(48));
  EXPECT_THAT(window.Read32(0), t::Eq(0x10u));
  EXPECT_THAT(window.Read32(12), t::Eq(0x40u));
  // Unmapped MSI-X hole [16, 32) returns all-ones and ignores writes.
  EXPECT_THAT(window.Read32(16), t::Eq(kMmioAllOnes));
  EXPECT_THAT(window.Read32(28), t::Eq(kMmioAllOnes));
  window.Write32(20, 0xDEADu);
  // Segment [32, 48) maps to high_regs[0..3].
  EXPECT_THAT(window.Read32(32), t::Eq(0x50u));
  EXPECT_THAT(window.Read32(44), t::Eq(0x80u));
  window.Write32(36, 0xBEEFu);
  EXPECT_THAT(high_regs[1], t::Eq(0xBEEFu));
}

}  // namespace
}  // namespace protos::usb
