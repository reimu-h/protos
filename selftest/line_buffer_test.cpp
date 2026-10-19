#include "selftest/line_buffer.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

namespace protos {
namespace {

namespace t = ::testing;

// The longest line that LineBuffer holds, its '\n' included.
constexpr int kMaxLineLength = LineBuffer::kCapacity - 1;

static int Length(const LineBuffer& line) {
  return static_cast<int>(std::strlen(line.c_str()));
}

static std::string Repeat(const char c, const int count) {
  return std::string(static_cast<size_t>(count), c);
}

TEST(LineBufferTest, NewLineBufferIsEmpty) {
  const LineBuffer line;
  EXPECT_THAT(line.c_str(), t::StrEq(""));
}

TEST(LineBufferTest, AppendingAnEmptyStringKeepsItEmpty) {
  LineBuffer line;
  line.Append("").Append("");
  EXPECT_THAT(line.c_str(), t::StrEq(""));
}

TEST(LineBufferTest, ShortLinePassesThroughUnchanged) {
  LineBuffer line;
  line.Append("[TEST] rbtree_edge_cases: PASS\n");
  EXPECT_THAT(line.c_str(), t::StrEq("[TEST] rbtree_edge_cases: PASS\n"));
}

TEST(LineBufferTest, MultipleAppendsBuildOneLine) {
  LineBuffer line;
  line.Append("[USER] user_mode_kill: missed deadline phase=")
      .Append("spinner")
      .Append(" kill_ticks=")
      .AppendDec(26)
      .Append("\n");
  EXPECT_THAT(line.c_str(),
              t::StrEq("[USER] user_mode_kill: missed deadline phase=spinner "
                       "kill_ticks=26\n"));
}

TEST(LineBufferTest, LineWithoutNewlineThatFitsKeepsItsLastCharacter) {
  // The VGA lines end with '\r' instead of '\n'.
  LineBuffer line;
  line.Append("[SMP] CPU ").AppendDec(23).Append(" ok\r");
  EXPECT_THAT(line.c_str(), t::StrEq("[SMP] CPU 23 ok\r"));
}

TEST(LineBufferTest, LongestLineThatFitsIsUnchanged) {
  const std::string text = Repeat('x', kMaxLineLength - 1) + "\n";
  LineBuffer line;
  line.Append(text.c_str());
  EXPECT_THAT(line.c_str(), t::StrEq(text));
  EXPECT_EQ(Length(line), kMaxLineLength);
}

TEST(LineBufferTest, LineOneCharacterTooLongIsCutAndKeepsItsNewline) {
  LineBuffer line;
  line.Append(Repeat('y', kMaxLineLength).c_str()).Append("\n");
  EXPECT_EQ(Length(line), kMaxLineLength);
  EXPECT_THAT(line.c_str(), t::StrEq(Repeat('y', kMaxLineLength - 1) + "\n"));
}

TEST(LineBufferTest, MuchTooLongLineIsCutKeepsItsNewlineAndIgnoresMore) {
  LineBuffer line;
  line.Append(Repeat('z', 3 * LineBuffer::kCapacity).c_str())
      .Append(" more\n")
      .AppendDec(42)
      .Append("\n");
  EXPECT_EQ(Length(line), kMaxLineLength);
  EXPECT_THAT(line.c_str(), t::StrEq(Repeat('z', kMaxLineLength - 1) + "\n"));
}

TEST(LineBufferTest, LineAfterACutLineStartsOnItsOwnLine) {
  // What the serial console shows when two lines are written one after the
  // other: the first one too long for the buffer.
  LineBuffer late_kill;
  late_kill.Append("[USER] user_mode_kill: missed deadline phase=")
      .Append("mid_syscall")
      .Append(" kill_ticks=")
      .AppendDec(26)
      .Append(" raw_ticks=")
      .AppendDec(26)
      .Append(" steal_ticks=")
      .AppendDec(0)
      .Append(" notice_ticks=")
      .AppendDec(0)
      .Append("\n");
  LineBuffer expectation;
  expectation.Append("[TEST] expectation failed: ")
      .Append("KillTicks(stats) <= kKillDeadlineTicks\n");
  const std::string console =
      std::string(late_kill.c_str()) + std::string(expectation.c_str());
  const size_t first_newline = console.find('\n');
  ASSERT_NE(first_newline, std::string::npos);
  EXPECT_EQ(static_cast<int>(first_newline), kMaxLineLength - 1);
  EXPECT_THAT(console.substr(first_newline + 1),
              t::StartsWith("[TEST] expectation failed: "));
  EXPECT_THAT(console, t::EndsWith(" <= kKillDeadlineTicks\n"));
}

TEST(LineBufferTest, AppendDecFormatsUnsignedDecimals) {
  LineBuffer line;
  line.AppendDec(0)
      .Append(" ")
      .AppendDec(7)
      .Append(" ")
      .AppendDec(static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
      .Append(" ")
      .AppendDec(static_cast<uint64_t>(std::numeric_limits<int64_t>::min()))
      .Append(" ")
      .AppendDec(std::numeric_limits<uint64_t>::max());
  EXPECT_THAT(line.c_str(), t::StrEq("0 7 9223372036854775807 "
                                     "9223372036854775808 "
                                     "18446744073709551615"));
}

TEST(LineBufferTest, AppendDecIsCutLikeText) {
  LineBuffer line;
  line.Append(Repeat('d', kMaxLineLength - 3).c_str())
      .AppendDec(std::numeric_limits<uint64_t>::max());
  EXPECT_EQ(Length(line), kMaxLineLength);
  EXPECT_THAT(line.c_str(), t::StrEq(Repeat('d', kMaxLineLength - 3) + "18\n"));
}

}  // namespace
}  // namespace protos
