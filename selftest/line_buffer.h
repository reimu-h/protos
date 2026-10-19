#ifndef PROTOS_SELFTEST_LINE_BUFFER_H_
#define PROTOS_SELFTEST_LINE_BUFFER_H_

#include <cstdint>

#include "format.h"

namespace protos {

// Fixed-capacity text line, so that a line printed by one CPU goes out in a
// single write instead of interleaving with the output of other CPUs. A line
// too long for it is cut short but still ends with '\n', so that the next line
// never continues it.
class LineBuffer {
 public:
  // Bytes of storage, including the terminating '\0': a line holds at most
  // kCapacity - 1 characters, its '\n' included.
  static constexpr int kCapacity = 96;

  LineBuffer() { buf_[0] = '\0'; }

  LineBuffer& Append(const char* str) {
    for (; *str != '\0'; ++str) {
      if (len_ + 1 >= kCapacity) {
        buf_[kCapacity - 2] = '\n';
        break;
      }
      buf_[len_++] = *str;
    }
    buf_[len_] = '\0';
    return *this;
  }

  LineBuffer& AppendDec(const uint64_t value) {
    char digits[kFormatDecBufferSize];
    return Append(FormatDec(value, digits));
  }

  const char* c_str() const { return buf_; }

 private:
  char buf_[kCapacity];
  int len_ = 0;
};

}  // namespace protos

#endif  // PROTOS_SELFTEST_LINE_BUFFER_H_
