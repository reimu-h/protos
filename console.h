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

// Attempts to read one byte from the console input buffer into `*out`.
// Returns false if no input byte is currently available.
bool ConsoleTryReadc(char* out);
// Pushes `size` bytes of programmatic input into the console input buffer.
void ConsolePushInput(const char* data, int64_t size);
// Clears any pending bytes in the console input buffer.
void ConsoleClearInput();

// Enables or disables echoing ConsoleWrite() output to the VGA screen (default
// true). Serial output and output capture remain active when VGA echo is off.
void ConsoleSetVgaEcho(bool enabled);
bool ConsoleVgaEchoEnabled();

// Clears the in-memory console output capture ring buffer.
void ConsoleResetOutputCapture();
// Copies up to `capacity - 1` bytes of captured console output into `dst`,
// null-terminates `dst` (when `dst != nullptr && capacity > 0`), and returns
// the number of bytes copied (excluding the trailing '\0').
int64_t ConsoleCopyOutputCapture(char* dst, int64_t capacity);

}  // namespace protos

#endif  // PROTOS_CONSOLE_H_
