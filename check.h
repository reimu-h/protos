#ifndef PROTOS_CHECK_H_
#define PROTOS_CHECK_H_

#include <atomic>
#include <cstdint>
#if __STDC_HOSTED__
#include <cstdio>
#include <cstdlib>
#endif

namespace protos {
namespace internal {

// Role of a CPU that enters the kernel panic path, as decided by EnterPanic().
enum class PanicRole {
  // First panic in the system: stop the other CPUs, then print the diagnostic
  // to every console.
  kOwner,
  // The owner panicked again while printing (for example, a console writer
  // faulted): print to the UART only.
  kNestedOwner,
  // Another CPU owns the panic, or the owner panicked a third time: halt
  // without printing.
  kSilent,
};

// Decides which CPU prints a kernel panic. `*owner` holds the token of the CPU
// that owns the panic (0 while there is none), and `*depth` counts how many
// times the owner has entered the panic path; only the owner writes `*depth`.
// `cpu_token` identifies the calling CPU and must not be 0.
inline PanicRole EnterPanic(std::atomic<uint64_t>* const owner,  //
                            int* const depth,                    //
                            const uint64_t cpu_token) {
  uint64_t current_owner = 0;
  if (owner->compare_exchange_strong(current_owner, cpu_token,
                                     std::memory_order_acq_rel)) {
    *depth = 1;
    return PanicRole::kOwner;
  }
  if (current_owner != cpu_token) {
    return PanicRole::kSilent;
  }
  ++*depth;
  return (*depth == 2) ? PanicRole::kNestedOwner : PanicRole::kSilent;
}

#if !__STDC_HOSTED__
// Kernel implementation of CheckFailure(), part of the panic path in idt.cpp.
[[noreturn]] void KernelCheckFailure(const char* expr,  //
                                     const char* file,  //
                                     int line);

// Kernel implementation of Panic(), part of the panic path in idt.cpp.
[[noreturn]] void KernelPanic(const char* message, const char* file, int line);
#endif

// Reports a failed CHECK as "[FATAL] Check failed: <expr> at <file>:<line>".
// Host builds print to stderr and exit with status 134. The kernel prints to
// the serial port and the console and halts every CPU; when several CPUs fail
// at once, only the first one prints.
[[noreturn]] inline void CheckFailure(const char* const expr,  //
                                      const char* const file,  //
                                      const int line) {
#if __STDC_HOSTED__
  std::fprintf(stderr, "[FATAL] Check failed: %s at %s:%d\n", expr, file, line);
  std::fflush(stderr);
  std::_Exit(134);
#else
  KernelCheckFailure(expr, file, line);
#endif
}

// Reports a PANIC() as "[FATAL] <message> at <file>:<line>", in the same way
// as CheckFailure().
[[noreturn]] inline void Panic(const char* const message,  //
                               const char* const file,     //
                               const int line) {
#if __STDC_HOSTED__
  std::fprintf(stderr, "[FATAL] %s at %s:%d\n", message, file, line);
  std::fflush(stderr);
  std::_Exit(134);
#else
  KernelPanic(message, file, line);
#endif
}

}  // namespace internal
}  // namespace protos

#define CHECK(condition)                                                \
  do {                                                                  \
    if (__builtin_expect(!static_cast<bool>(condition), 0)) {           \
      ::protos::internal::CheckFailure(#condition, __FILE__, __LINE__); \
    }                                                                   \
  } while (false)

// Unconditionally crashes with "[FATAL] <message> at <file>:<line>", like a
// failed CHECK. `message` must be a string literal.
#define PANIC(message) ::protos::internal::Panic("" message, __FILE__, __LINE__)

#if defined(NDEBUG)
#define DCHECK(condition)                        \
  do {                                           \
    if (false && static_cast<bool>(condition)) { \
    }                                            \
  } while (false)
#else
#define DCHECK(condition) CHECK(condition)
#endif

#endif  // PROTOS_CHECK_H_
