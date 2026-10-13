#ifndef PROTOS_CONSOLE_H_
#define PROTOS_CONSOLE_H_

#include <cstdint>

namespace protos {

// Writers for boot and status messages that go to both consoles: the serial
// port (uart.h) and the screen (vga.h). Each call writes its text to the serial
// port and then to the screen, taking each console's lock in turn, so the
// output of several CPUs can interleave differently on the two consoles.

// Writes a null-terminated string.
void ConsoleWrite(const char* str);
// Writes `value` as "0x" followed by uppercase hexadecimal digits.
void ConsoleWriteHex(uint64_t value);
// Writes `value` in decimal.
void ConsoleWriteDec(uint64_t value);

}  // namespace protos

#endif  // PROTOS_CONSOLE_H_
