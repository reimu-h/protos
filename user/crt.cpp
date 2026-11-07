// The runtime of the user programs: their entry point, which saves the
// initial x87/SSE/AVX state for the programs to check, and the memory
// functions that GCC may call even in freestanding code (see cxx_runtime.cpp,
// the kernel's counterpart). User programs are built with
// -fno-tree-loop-distribute-patterns, so the byte loops here stay loops.

#include <cstddef>
#include <cstdint>

#include "user/syscall.h"

// The x87/SSE/AVX state that the program started with (user/syscall.h).
extern "C" {
alignas(64) uint8_t g_initial_fpu_state[protos::user::kInitialFpuStateSize];
}

namespace protos::user {
namespace {
uint64_t g_entry_arg = 0;
}  // namespace

[[gnu::weak]] int64_t Main(const uint64_t /*arg*/) { return 0; }

[[gnu::weak]] int64_t Main(const int /*argc*/, char** const /*argv*/) {
  return Main(g_entry_arg);
}

}  // namespace protos::user

// Calls the program's main function with `(argc, argv)` (or `arg` if the
// program defines `Main(uint64_t)`) and exits with its result.
extern "C" [[noreturn]] void StartProgram(const uint64_t arg,  //
                                          const int64_t argc,  //
                                          char** const argv) {
  protos::user::g_entry_arg = arg;
  protos::user::Exit(protos::user::Main(static_cast<int>(argc), argv));
}

// The kernel enters a program here, with `arg` in rdi, `argc` in rsi, `argv`
// in rdx, and a stack pointer that is 8 bytes below a 16-byte boundary, as if
// a call had pushed a return address (user_mode.h). Before any compiled code
// can change the x87/SSE/AVX registers, this saves them into
// `g_initial_fpu_state`: with XSAVE (the x87, SSE and AVX components that XCR0
// enables) if the kernel enabled it (CPUID.1:ECX.OSXSAVE), else with FXSAVE.
// Then it jumps to `StartProgram()`, which sees the stack as the entry point
// did.
asm(R"(
    .pushsection .text
    .globl _start
    .type _start, @function
_start:
    mov r12, rdi
    mov r13, rsi
    mov r14, rdx
    mov eax, 1
    cpuid
    lea rdi, [rip + g_initial_fpu_state]
    bt ecx, 27
    jnc 1f
    xor ecx, ecx
    xgetbv
    and eax, 7
    xor edx, edx
    xsave64 [rdi]
    jmp 2f
1:
    fxsave64 [rdi]
2:
    mov rdi, r12
    mov rsi, r13
    mov rdx, r14
    jmp StartProgram
    .size _start, . - _start
    .popsection
)");

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
