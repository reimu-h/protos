#ifndef PROTOS_SPINLOCK_H_
#define PROTOS_SPINLOCK_H_

#include <atomic>
#include <cstdint>
#if __STDC_HOSTED__
#include <thread>
#endif

#include "check.h"
#include "smp.h"

namespace protos {

class IrqSpinLock;

// RFLAGS.IF (bit 9): maskable interrupts are enabled while it is set.
inline constexpr uint64_t kRflagsInterruptEnableBit = 1ULL << 9;

// Implementation details of the interrupt helpers and `IrqSpinLock` below.
// Other code should use the public functions instead.
namespace internal {

// Older call sites spell it `internal::kRflagsInterruptEnableBit`.
using ::protos::kRflagsInterruptEnableBit;

// In the kernel, the `IrqSpinLock` owner id of a CPU without a bound
// `CpuLocal` (host threads use a per-thread token instead). Bound CPUs use
// `OwnerIdForCpu()`, in 1 .. kMaxSupportedCpus, and 0 means "not held", so this
// id collides with neither.
inline constexpr uintptr_t kUnboundCpuOwnerId =
    static_cast<uintptr_t>(kMaxSupportedCpus) + 1;

inline uintptr_t OwnerIdForCpu(const CpuLocal& cpu) {
  DCHECK(cpu.cpu_id >= 0 && cpu.cpu_id < kMaxSupportedCpus);
  return static_cast<uintptr_t>(cpu.cpu_id) + 1;
}

#if !__STDC_HOSTED__
// Top of the held-lock stack of the CPU that runs without a bound `CpuLocal`.
inline IrqSpinLock* g_bootstrap_top_lock = nullptr;

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

// Lock ownership and the held-lock stack are per CPU and live in the CPU's
// bound `CpuLocal`. Only the BSP runs unbound: from boot until `SmpInit()`
// binds its `CpuLocal`, it uses `kUnboundCpuOwnerId` and
// `g_bootstrap_top_lock`, and it must release every lock taken in that phase
// before binding (checked in `TopHeldLockSlot()`).
//
// APs must not take an `IrqSpinLock`, directly or through helpers such as
// `UartWrite()`, before `ApKernelEntry()` binds their `CpuLocal`: by then the
// BSP has bound, so `CurrentCpuOrNull()` on such an AP reads `gs:[0]` through a
// GS base of 0, which faults or yields garbage. When it yields no `CpuLocal`,
// the DCHECKs on the unbound paths below catch the violation.
inline uintptr_t CurrentOwnerId() {
  const CpuLocal* const cpu = CurrentCpuOrNull();
  if (cpu != nullptr) {
    return OwnerIdForCpu(*cpu);
  }
  DCHECK(!g_cpu_local_bound.load(std::memory_order_relaxed));
  return kUnboundCpuOwnerId;
}

inline IrqSpinLock** TopHeldLockSlot() {
  CpuLocal* const cpu = CurrentCpuOrNull();
  if (cpu != nullptr) {
    DCHECK(g_bootstrap_top_lock == nullptr);
    return &cpu->top_held_lock;
  }
  DCHECK(!g_cpu_local_bound.load(std::memory_order_relaxed));
  return &g_bootstrap_top_lock;
}
#else
// Host builds simulate the per-CPU state with `thread_local` variables, and
// host scheduler tests run tasks as fibers that can resume on a different OS
// thread after a context switch. The accessors below must therefore compute
// the address of their `thread_local` anew on every call, never reusing one
// computed before a switch (GCC treats the thread pointer as constant within a
// function). `[[gnu::noinline]]` keeps the address computation out of the
// callers, and the empty `asm volatile` with a memory operand stops GCC from
// inferring that the function is pure and merging calls across a switch.
// (`[[gnu::noipa]]` would state both, but Clang doesn't support it.)
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

[[gnu::noinline]] inline uintptr_t CurrentOwnerId() {
  const CpuLocal* const cpu = CurrentCpuOrNull();
  if (cpu != nullptr) {
    return OwnerIdForCpu(*cpu);
  }
  thread_local uint8_t thread_token = 0;
  asm volatile("" : "+m"(thread_token));
  return reinterpret_cast<uintptr_t>(&thread_token);
}

[[gnu::noinline]] inline IrqSpinLock** TopHeldLockSlot() {
  CpuLocal* const cpu = CurrentCpuOrNull();
  if (cpu != nullptr) {
    return &cpu->top_held_lock;
  }
  thread_local IrqSpinLock* top_lock = nullptr;
  asm volatile("" : "+m"(top_lock));
  return &top_lock;
}
#endif

// Reads the calling CPU's top held `IrqSpinLock*` with local interrupts
// disabled so task preemption and cross-CPU work-stealing migration cannot
// occur between resolving the current CPU's `CpuLocal` and reading its
// `top_held_lock` field.
inline IrqSpinLock* TopHeldLock() {
  const bool saved_irq = LocalSaveAndDisableInterrupts();
  IrqSpinLock** const top_slot = TopHeldLockSlot();
  IrqSpinLock* const top_lock = (top_slot != nullptr) ? *top_slot : nullptr;
  LocalRestoreInterrupts(saved_irq);
  return top_lock;
}

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

// Returns true if the calling CPU (in host builds: thread or bound `CpuLocal`)
// holds at least one `IrqSpinLock`, e.g. to assert that no spinlock is held
// across a context switch.
inline bool HoldsAnyIrqSpinLock() { return internal::TopHeldLock() != nullptr; }

#if __STDC_HOSTED__
// Sets the simulated interrupt-enable flag of the calling host thread (tests
// only).
inline void SetInterruptsEnabledForTest(const bool enabled) {
  internal::LocalSetInterruptsEnabledForTest(enabled);
}
#endif

// Interrupt-safe FIFO ticket spinlock.
//
// `Lock()` saves the caller's interrupt state (`RFLAGS.IF`), disables local
// interrupts, takes a ticket, and spins until the ticket is served, so CPUs
// acquire the lock in the order in which they called `Lock()`. `Unlock()`
// serves the next ticket and then restores the saved interrupt state. All
// fields are zero-initialized, so a global `IrqSpinLock` is
// constant-initialized (in `.bss`) and needs no runtime constructor.
//
// Rules for callers. Several are checked only by DCHECKs; without them, a
// violation deadlocks or corrupts state instead of failing a check:
// - Release locks in the reverse order of acquisition. Releasing an outer lock
//   first re-enables interrupts while the inner lock is still held, and an
//   interrupt handler that then takes the inner lock on the same CPU spins
//   forever.
// - Locks are not recursive: taking a lock that the calling CPU already holds
//   deadlocks.
// - Waiters spin, and holders run, with interrupts disabled. While holding a
//   lock, never wait for another CPU to handle an interrupt or IPI (such as an
//   `SmpRunOnAllCpus()` round), and never block, `Yield()`, or `Join()`.
// - The lock is not NMI-safe: NMI handlers must not take an `IrqSpinLock`.
// - Each CPU's stack of held locks (see `HoldsAnyIrqSpinLock()`) is only used
//   to check the release order. There is no lock-order (ABBA deadlock)
//   validation, so document the order of nested locks next to the locks.
class IrqSpinLock {
 public:
  constexpr IrqSpinLock() = default;

  IrqSpinLock(const IrqSpinLock&) = delete;
  IrqSpinLock& operator=(const IrqSpinLock&) = delete;
  IrqSpinLock(IrqSpinLock&&) = delete;
  IrqSpinLock& operator=(IrqSpinLock&&) = delete;

  void Lock() {
    const bool irq_was_enabled = internal::LocalSaveAndDisableInterrupts();
    const uintptr_t current_owner = internal::CurrentOwnerId();
    DCHECK(current_owner != 0);
    DCHECK(owner_.load(std::memory_order_relaxed) != current_owner);

    const int64_t ticket = next_ticket_.fetch_add(1, std::memory_order_relaxed);
#if !__STDC_HOSTED__
    while (serving_ticket_.load(std::memory_order_acquire) != ticket) {
      asm volatile("pause" : : : "memory");
    }
#else
    int spin_iters = 0;
    while (serving_ticket_.load(std::memory_order_acquire) != ticket) {
      asm volatile("pause" : : : "memory");
      if ((++spin_iters & 63) == 0) {
        std::this_thread::yield();
      }
    }
#endif

    DCHECK(!locked_);
    DCHECK(owner_.load(std::memory_order_relaxed) == 0);
    DCHECK(!internal::LocalAreInterruptsEnabled());

    IrqSpinLock** const top_slot = internal::TopHeldLockSlot();
    prev_held_lock_ = *top_slot;
    *top_slot = this;
    saved_irq_enabled_ = irq_was_enabled;
    locked_ = true;
    owner_.store(current_owner, std::memory_order_relaxed);
  }

  void Unlock() {
    const uintptr_t current_owner = internal::CurrentOwnerId();
    DCHECK(current_owner != 0);
    DCHECK(owner_.load(std::memory_order_relaxed) == current_owner);
    DCHECK(locked_);
    DCHECK(!internal::LocalAreInterruptsEnabled());

    IrqSpinLock** const top_slot = internal::TopHeldLockSlot();
    DCHECK(*top_slot == this);
    *top_slot = prev_held_lock_;
    prev_held_lock_ = nullptr;

    const bool irq_was_enabled = saved_irq_enabled_;
    saved_irq_enabled_ = false;
    locked_ = false;
    owner_.store(0, std::memory_order_relaxed);

    const int64_t next_serving =
        serving_ticket_.load(std::memory_order_relaxed) + 1;
    serving_ticket_.store(next_serving, std::memory_order_release);

    internal::LocalRestoreInterrupts(irq_was_enabled);
  }

  // Returns true if some CPU holds the lock or is acquiring it. This is a racy
  // snapshot for assertions and diagnostics only: it provides no
  // synchronization, so it must never decide whether data that the lock
  // protects may be accessed.
  bool IsLocked() const {
    return next_ticket_.load(std::memory_order_relaxed) !=
           serving_ticket_.load(std::memory_order_relaxed);
  }

  // Returns true if the calling CPU holds the lock.
  bool IsLockedByCurrentCpu() const {
    const bool saved_irq = internal::LocalSaveAndDisableInterrupts();
    const bool owned =
        owner_.load(std::memory_order_relaxed) == internal::CurrentOwnerId();
    internal::LocalRestoreInterrupts(saved_irq);
    return owned;
  }

  // The next ticket to hand out and the ticket being served, for tests and
  // diagnostics only: racy snapshots that provide no synchronization.
  int64_t NextTicket() const {
    return next_ticket_.load(std::memory_order_relaxed);
  }

  int64_t ServingTicket() const {
    return serving_ticket_.load(std::memory_order_relaxed);
  }

 private:
  std::atomic<int64_t> next_ticket_{0};
  std::atomic<int64_t> serving_ticket_{0};
  std::atomic<uintptr_t> owner_{0};
  IrqSpinLock* prev_held_lock_ = nullptr;
  bool saved_irq_enabled_ = false;
  bool locked_ = false;
};

// RAII scoped guard for `IrqSpinLock`.
class IrqSpinLockGuard {
 public:
  explicit IrqSpinLockGuard(IrqSpinLock& lock) : lock_(&lock) { lock_->Lock(); }

  explicit IrqSpinLockGuard(IrqSpinLock* const lock) : lock_(lock) {
    DCHECK(lock_ != nullptr);
    lock_->Lock();
  }

  ~IrqSpinLockGuard() { lock_->Unlock(); }

  IrqSpinLockGuard(const IrqSpinLockGuard&) = delete;
  IrqSpinLockGuard& operator=(const IrqSpinLockGuard&) = delete;
  IrqSpinLockGuard(IrqSpinLockGuard&&) = delete;
  IrqSpinLockGuard& operator=(IrqSpinLockGuard&&) = delete;

 private:
  IrqSpinLock* const lock_;
};

}  // namespace protos

#endif  // PROTOS_SPINLOCK_H_
