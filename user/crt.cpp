// The runtime of the user programs: their entry point, and the memory
// functions that GCC may call even in freestanding code (see cxx_runtime.cpp,
// the kernel's counterpart). User programs are built with
// -fno-tree-loop-distribute-patterns, so the byte loops here stay loops.

#include <cstddef>
#include <cstdint>

#include "user/syscall.h"

// The kernel enters a program here, with the program's argument in rdi and a
// stack pointer that is 8 bytes below a 16-byte boundary, as if a call had
// pushed a return address (user_mode.h).
extern "C" [[noreturn]] void _start(const uint64_t arg) {
  protos::user::Exit(protos::user::Main(arg));
}

extern "C" void* memcpy(void* const dest,       //
                        const void* const src,  //
                        const size_t count) {
  void* dest_cursor = dest;
  const void* src_cursor = src;
  size_t remaining = count;
  asm volatile("rep movsb"
               : "+D"(dest_cursor), "+S"(src_cursor), "+c"(remaining)
               :
               : "memory");
  return dest;
}

extern "C" void* memmove(void* const dest,       //
                         const void* const src,  //
                         const size_t count) {
  unsigned char* const d = static_cast<unsigned char*>(dest);
  const unsigned char* const s = static_cast<const unsigned char*>(src);
  if (d <= s || d >= s + count) {
    return memcpy(dest, src, count);
  }
  for (size_t i = count; i > 0; --i) {
    d[i - 1] = s[i - 1];
  }
  return dest;
}

extern "C" void* memset(void* const dest, const int value, const size_t count) {
  void* dest_cursor = dest;
  size_t remaining = count;
  asm volatile("rep stosb"
               : "+D"(dest_cursor), "+c"(remaining)
               : "a"(value)
               : "memory");
  return dest;
}

extern "C" int memcmp(const void* const lhs,  //
                      const void* const rhs,  //
                      const size_t count) {
  const unsigned char* const a = static_cast<const unsigned char*>(lhs);
  const unsigned char* const b = static_cast<const unsigned char*>(rhs);
  for (size_t i = 0; i < count; ++i) {
    if (a[i] != b[i]) {
      return a[i] < b[i] ? -1 : 1;
    }
  }
  return 0;
}

