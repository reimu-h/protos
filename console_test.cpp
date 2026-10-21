#include "console.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
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
  std::string rx_buffer;
  int read_pos = 0;
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

bool UartTryGetc(char* const out) {
  if (out == nullptr ||
      g_consoles.read_pos >= static_cast<int>(g_consoles.rx_buffer.size())) {
    return false;
  }
  *out = g_consoles.rx_buffer[g_consoles.read_pos++];
  return true;
}

void UartPushRx(const char* const data, const int64_t size) {
  if (data == nullptr || size <= 0) {
    return;
  }
  g_consoles.rx_buffer.append(data, static_cast<size_t>(size));
}

void UartClearRx() {
  g_consoles.rx_buffer.clear();
  g_consoles.read_pos = 0;
}

namespace {

class ConsoleTest : public t::Test {
 protected:
  void SetUp() override {
    g_consoles = FakeConsoles{};
    ConsoleSetVgaEcho(true);
    ConsoleResetOutputCapture();
  }

  void TearDown() override {
    ConsoleSetVgaEcho(true);
    ConsoleResetOutputCapture();
  }
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

TEST_F(ConsoleTest, PushInputAndTryReadcPopBytesInFifoOrder) {
  char ch = '\0';
  EXPECT_FALSE(ConsoleTryReadc(&ch));
  EXPECT_FALSE(ConsoleTryReadc(nullptr));

  ConsolePushInput("sh\n", 3);
  ConsolePushInput("exit\n", 5);
  ConsolePushInput(nullptr, 4);
  ConsolePushInput("x", 0);
  ConsolePushInput("x", -1);

  std::string out;
  while (ConsoleTryReadc(&ch)) {
    out.push_back(ch);
  }
  EXPECT_THAT(out, t::Eq("sh\nexit\n"));
  EXPECT_FALSE(ConsoleTryReadc(&ch));
}

TEST_F(ConsoleTest, ClearInputDiscardsUnreadBytes) {
  ConsolePushInput("abc", 3);
  char ch = '\0';
  ASSERT_TRUE(ConsoleTryReadc(&ch));
  EXPECT_THAT(ch, t::Eq('a'));

  ConsoleClearInput();
  EXPECT_FALSE(ConsoleTryReadc(&ch));

  ConsolePushInput("z", 1);
  ASSERT_TRUE(ConsoleTryReadc(&ch));
  EXPECT_THAT(ch, t::Eq('z'));
}

TEST_F(ConsoleTest, DisablingVgaEchoSuppressesScreenWritesOnly) {
  EXPECT_TRUE(ConsoleVgaEchoEnabled());
  ConsoleSetVgaEcho(false);
  EXPECT_FALSE(ConsoleVgaEchoEnabled());

  ConsoleWrite("serial only\n");
  ConsoleWriteHex(0x2A);
  ConsoleWriteDec(7);
  EXPECT_THAT(g_consoles.uart_log, t::Eq("serial only\n0x2A7"));
  EXPECT_THAT(g_consoles.vga_log, t::IsEmpty());
  EXPECT_THAT(g_consoles.write_order, t::Eq("UUU"));

  ConsoleSetVgaEcho(true);
  EXPECT_TRUE(ConsoleVgaEchoEnabled());
  ConsoleWrite("both");
  EXPECT_THAT(g_consoles.uart_log, t::Eq("serial only\n0x2A7both"));
  EXPECT_THAT(g_consoles.vga_log, t::Eq("both"));
  EXPECT_THAT(g_consoles.write_order, t::Eq("UUUUV"));
}

TEST_F(ConsoleTest, OutputCaptureRecordsWritesAndSupportsTruncationAndReset) {
  ConsoleWrite("alpha ");
  ConsoleWriteDec(42);
  ConsoleWrite(" beta");

  char full[64] = {};
  EXPECT_THAT(ConsoleCopyOutputCapture(full, sizeof(full)), t::Eq(13));
  EXPECT_THAT(std::string(full), t::Eq("alpha 42 beta"));

  char truncated[6] = {};
  EXPECT_THAT(ConsoleCopyOutputCapture(truncated, sizeof(truncated)), t::Eq(5));
  EXPECT_THAT(std::string(truncated), t::Eq("alpha"));

  char one_byte[1] = {'X'};
  EXPECT_THAT(ConsoleCopyOutputCapture(one_byte, sizeof(one_byte)), t::Eq(0));
  EXPECT_THAT(one_byte[0], t::Eq('\0'));
  EXPECT_THAT(ConsoleCopyOutputCapture(nullptr, 16), t::Eq(0));
  EXPECT_THAT(ConsoleCopyOutputCapture(full, 0), t::Eq(0));

  ConsoleResetOutputCapture();
  EXPECT_THAT(ConsoleCopyOutputCapture(full, sizeof(full)), t::Eq(0));
  EXPECT_THAT(std::string(full), t::IsEmpty());
}

}  // namespace
}  // namespace protos
