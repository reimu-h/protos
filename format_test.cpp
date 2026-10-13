#include "format.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace protos {
namespace {

namespace t = ::testing;

// Values around every digit-count boundary in both bases.
static std::vector<uint64_t> InterestingValues() {
  std::vector<uint64_t> values = {0, UINT64_MAX};
  for (int shift = 0; shift < 64; ++shift) {
    const uint64_t power_of_two = uint64_t{1} << shift;
    values.push_back(power_of_two - 1);
    values.push_back(power_of_two);
  }
  for (uint64_t power_of_ten = 1; power_of_ten <= UINT64_MAX / 10;
       power_of_ten *= 10) {
    values.push_back(power_of_ten - 1);
    values.push_back(power_of_ten);
    values.push_back(power_of_ten * 10 - 1);
  }
  return values;
}

static std::string Printf(const char* const format, const uint64_t value) {
  char text[32];
  std::snprintf(text,          //
                sizeof(text),  //
                format,        //
                static_cast<unsigned long long>(value));
  return text;
}

TEST(FormatTest, FormatHexPrintsUppercaseDigitsWithoutLeadingZeros) {
  char buffer[kFormatHexBufferSize];
  EXPECT_THAT(FormatHex(0, buffer), t::StrEq("0x0"));
  EXPECT_THAT(FormatHex(0x36D76289, buffer), t::StrEq("0x36D76289"));
  for (const uint64_t value : InterestingValues()) {
    EXPECT_THAT(FormatHex(value, buffer), t::StrEq(Printf("0x%llX", value)));
  }
}

TEST(FormatTest, FormatDecPrintsDigitsWithoutLeadingZeros) {
  char buffer[kFormatDecBufferSize];
  EXPECT_THAT(FormatDec(0, buffer), t::StrEq("0"));
  EXPECT_THAT(FormatDec(24000000, buffer), t::StrEq("24000000"));
  for (const uint64_t value : InterestingValues()) {
    EXPECT_THAT(FormatDec(value, buffer), t::StrEq(Printf("%llu", value)));
  }
}

TEST(FormatTest, TextIsRightAlignedAndTheLargestValueFillsTheBuffer) {
  char hex_buffer[kFormatHexBufferSize];
  EXPECT_THAT(FormatHex(0xAB, hex_buffer),
              t::Eq(&hex_buffer[kFormatHexBufferSize - 5]));
  EXPECT_THAT(FormatHex(UINT64_MAX, hex_buffer), t::Eq(&hex_buffer[0]));

  char dec_buffer[kFormatDecBufferSize];
  EXPECT_THAT(FormatDec(42, dec_buffer),
              t::Eq(&dec_buffer[kFormatDecBufferSize - 3]));
  EXPECT_THAT(FormatDec(UINT64_MAX, dec_buffer), t::Eq(&dec_buffer[0]));
}

}  // namespace
}  // namespace protos
