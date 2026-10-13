#ifndef PROTOS_FORMAT_H_
#define PROTOS_FORMAT_H_

#include <cstdint>

namespace protos {

// Number formatting for console output. FormatHex() and FormatDec() take no
// locks and do not allocate, so the panic path can use them too.

// Sizes of the buffers that FormatHex() and FormatDec() fill, including the
// terminating NUL: "0x" and up to 16 hexadecimal digits, and up to 20 decimal
// digits.
constexpr int kFormatHexBufferSize = 19;
constexpr int kFormatDecBufferSize = 21;

// Formats `value` as "0x" followed by its uppercase hexadecimal digits, without
// leading zeros ("0x0" for zero). The text is right-aligned in `buffer`: the
// returned pointer to its first character may lie past the start of `buffer`.
inline const char* FormatHex(const uint64_t value,
                             char (&buffer)[kFormatHexBufferSize]) {
  int pos = kFormatHexBufferSize - 1;
  buffer[pos] = '\0';
  uint64_t remaining = value;
  do {
    --pos;
    buffer[pos] = "0123456789ABCDEF"[remaining & 0xF];
    remaining >>= 4;
  } while (remaining != 0);
  buffer[--pos] = 'x';
  buffer[--pos] = '0';
  return &buffer[pos];
}

// Formats `value` in decimal, right-aligned in `buffer` like FormatHex().
inline const char* FormatDec(const uint64_t value,
                             char (&buffer)[kFormatDecBufferSize]) {
  int pos = kFormatDecBufferSize - 1;
  buffer[pos] = '\0';
  uint64_t remaining = value;
  do {
    --pos;
    buffer[pos] = static_cast<char>('0' + remaining % 10);
    remaining /= 10;
  } while (remaining != 0);
  return &buffer[pos];
}

}  // namespace protos

#endif  // PROTOS_FORMAT_H_
