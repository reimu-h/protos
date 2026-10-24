#ifndef PROTOS_INTERRUPTS_H_
#define PROTOS_INTERRUPTS_H_

#include <cstdint>

namespace protos {

// RFLAGS.IF (bit 9): maskable interrupts are enabled while it is set.
inline constexpr uint64_t kRflagsInterruptEnableBit = 1ULL << 9;
// RFLAGS.DF (bit 10): string instructions step downwards while it is set. The
// System V AMD64 ABI requires it clear on function entry and return.
inline constexpr uint64_t kRflagsDirectionBit = 1ULL << 10;
// RFLAGS.AC (bit 18). User code may set it freely (CR0.AM is clear, so it
// checks no alignment). While SMAP is enabled (cpu.h), kernel code reaches
// user pages only while it is set, which only the user-copy helpers do
// (user_access.h).
inline constexpr uint64_t kRflagsAlignmentCheckBit = 1ULL << 18;

// Implementation details of the interrupt helpers below. Other code should use
// the public functions instead.
namespace internal {

// Older call sites spell it `internal::kRflagsInterruptEnableBit`.
using ::protos::kRflagsInterruptEnableBit;

#if !__STDC_HOSTED__
inline bool LocalAreInterruptsEnabled() {
  uint64_t rflags = 0;
  asm volatile(
      "pushfq\n\t"
      "pop %0"
      : "=r"(rflags)
      :
      : "memory");
  return (rflags & kRflagsInterruptEnableBit) != 0;
}

inline bool LocalSaveAndDisableInterrupts() {
  uint64_t rflags = 0;
  asm volatile(
      "pushfq\n\t"
      "pop %0\n\t"
      "cli"
      : "=r"(rflags)
      :
      : "memory", "cc");
  return (rflags & kRflagsInterruptEnableBit) != 0;
}

inline void LocalRestoreInterrupts(const bool previously_enabled) {
  if (previously_enabled) {
    asm volatile("sti" : : : "memory", "cc");
  }
}
#else
// Host builds simulate the per-CPU state with `thread_local` variables, and
// host scheduler tests run tasks as fibers that can resume on a different OS
// thread after a context switch. Accessors of such state must therefore
// compute the address of their `thread_local` anew on every call, never
// reusing one computed before a switch (GCC treats the thread pointer as
// constant within a function). `[[gnu::noinline]]` keeps the address
// computation out of the callers, and the empty `asm volatile` with a memory
// operand stops GCC from inferring that the function is pure and merging calls
// across a switch. (`[[gnu::noipa]]` would state both, but Clang doesn't
// support it.)
[[gnu::noinline]] inline bool* HostInterruptEnabledSlot() {
  thread_local bool interrupts_enabled = true;
  asm volatile("" : "+m"(interrupts_enabled));
  return &interrupts_enabled;
}

inline bool LocalAreInterruptsEnabled() { return *HostInterruptEnabledSlot(); }

inline void LocalSetInterruptsEnabledForTest(const bool enabled) {
  *HostInterruptEnabledSlot() = enabled;
}

inline bool LocalSaveAndDisableInterrupts() {
  bool* const slot = HostInterruptEnabledSlot();
  const bool previously_enabled = *slot;
  *slot = false;
  return previously_enabled;
}

inline void LocalRestoreInterrupts(const bool previously_enabled) {
  if (previously_enabled) {
    *HostInterruptEnabledSlot() = true;
  }
}
#endif

}  // namespace internal

// Returns true if maskable interrupts are enabled on the calling CPU (in host
// builds: simulated as enabled on the calling thread).
inline bool AreInterruptsEnabled() {
  return internal::LocalAreInterruptsEnabled();
}

// Disables maskable interrupts on the calling CPU and returns whether they were
// enabled, to be passed to the matching `RestoreInterrupts()`.
inline bool SaveAndDisableInterrupts() {
  return internal::LocalSaveAndDisableInterrupts();
}

// Re-enables maskable interrupts if `previously_enabled`, the result of the
// matching `SaveAndDisableInterrupts()`; otherwise leaves them disabled.
inline void RestoreInterrupts(const bool previously_enabled) {
  internal::LocalRestoreInterrupts(previously_enabled);
}

// Disables maskable interrupts for the lifetime of the guard and then restores
// the previous state.
class ScopedIrqDisable {
 public:
  ScopedIrqDisable() : previously_enabled_(SaveAndDisableInterrupts()) {}
  ~ScopedIrqDisable() { RestoreInterrupts(previously_enabled_); }

  ScopedIrqDisable(const ScopedIrqDisable&) = delete;
  ScopedIrqDisable& operator=(const ScopedIrqDisable&) = delete;

 private:
  const bool previously_enabled_;
};

#if __STDC_HOSTED__
// Sets the simulated interrupt-enable flag of the calling host thread (tests
// only).
inline void SetInterruptsEnabledForTest(const bool enabled) {
  internal::LocalSetInterruptsEnabledForTest(enabled);
}
#endif

}  // namespace protos

#endif  // PROTOS_INTERRUPTS_H_
