#include "console.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "uart.h"
#include "vga.h"

namespace protos {
namespace {

namespace t = ::testing;

struct FakeConsoles {
  std::string uart_log;
  std::string vga_log;
  // One letter per write, in call order: 'U' for the UART, 'V' for the screen.
  std::string write_order;
};

FakeConsoles g_consoles;

}  // namespace

void UartWrite(const char* const str) {
  g_consoles.uart_log.append(str);
  g_consoles.write_order.push_back('U');
}

void VgaWrite(const char* const str) {
  g_consoles.vga_log.append(str);
  g_consoles.write_order.push_back('V');
}

namespace {

class ConsoleTest : public t::Test {
 protected:
  void SetUp() override { g_consoles = FakeConsoles{}; }
};

TEST_F(ConsoleTest, WriteGoesToTheUartAndThenToTheScreen) {
  ConsoleWrite("[PMM] Kernel range: ");
  EXPECT_THAT(g_consoles.uart_log, t::Eq("[PMM] Kernel range: "));
  EXPECT_THAT(g_consoles.vga_log, t::Eq("[PMM] Kernel range: "));
  EXPECT_THAT(g_consoles.write_order, t::Eq("UV"));
}

TEST_F(ConsoleTest, NumbersAreFormattedOnceAndWrittenToBothConsoles) {
  ConsoleWriteHex(0x7EE0000);
  ConsoleWriteHex(0);
  ConsoleWriteDec(UINT64_MAX);
  ConsoleWriteDec(0);
  EXPECT_THAT(g_consoles.uart_log, t::Eq("0x7EE00000x0184467440737095516150"));
  EXPECT_THAT(g_consoles.vga_log, t::Eq(g_consoles.uart_log));
  // Each number is a single write to each console.
  EXPECT_THAT(g_consoles.write_order, t::Eq("UVUVUVUV"));
}

}  // namespace
}  // namespace protos
