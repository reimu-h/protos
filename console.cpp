#include "console.h"

#include <cstdint>

#include "format.h"
#include "uart.h"
#include "vga.h"

namespace protos {

void ConsoleWrite(const char* const str) {
  UartWrite(str);
  VgaWrite(str);
}

void ConsoleWriteHex(const uint64_t value) {
  char buffer[kFormatHexBufferSize];
  ConsoleWrite(FormatHex(value, buffer));
}

void ConsoleWriteDec(const uint64_t value) {
  char buffer[kFormatDecBufferSize];
  ConsoleWrite(FormatDec(value, buffer));
}

}  // namespace protos
