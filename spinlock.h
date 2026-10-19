#ifndef PROTOS_SPINLOCK_H_
#define PROTOS_SPINLOCK_H_

#include <atomic>
#include <cstdint>
#if __STDC_HOSTED__
#include <thread>
#endif

#include "check.h"
#include "interrupts.h"
#include "smp.h"

namespace protos {

class IrqSpinLock;

// Implementation details of `IrqSpinLock` below. Other code should use the
// public functions instead.
namespace internal {

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

// Lock ownership and the held-lock stack are per CPU and live in the CPU's
// bound `CpuLocal`. APs bind theirs in the startup trampoline (boot.S) before
// they run any C++ code, so only the BSP runs unbound: from boot until
// `SmpInit()` binds its `CpuLocal`, it uses `kUnboundCpuOwnerId` and
// `g_bootstrap_top_lock`, and it must release every lock taken in that phase
// before binding (checked in `TopHeldLockSlot()`).
inline uintptr_t CurrentOwnerId() {
  const CpuLocal* const cpu = CurrentCpuOrNull();
  return (cpu != nullptr) ? OwnerIdForCpu(*cpu) : kUnboundCpuOwnerId;
}

inline IrqSpinLock** TopHeldLockSlot() {
  CpuLocal* const cpu = CurrentCpuOrNull();
  if (cpu == nullptr) {
    return &g_bootstrap_top_lock;
  }
  DCHECK(g_bootstrap_top_lock == nullptr);
  return &cpu->top_held_lock;
}
#else
// Host threads without a bound `CpuLocal` keep their owner token and held-lock
// stack in `thread_local` variables. Like `HostInterruptEnabledSlot()`
// (interrupts.h), these accessors stay out of line and recompute the
// `thread_local` address on every call.
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

// Returns true if the calling CPU (in host builds: thread or bound `CpuLocal`)
// holds at least one `IrqSpinLock`, e.g. to assert that no spinlock is held
// across a context switch.
inline bool HoldsAnyIrqSpinLock() { return internal::TopHeldLock() != nullptr; }

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

    FinishAcquire(current_owner, irq_was_enabled);
  }

  // Attempts to acquire the lock without spinning. If the lock is uncontended
  // (`next_ticket_ == serving_ticket_`), disables local interrupts, claims the
  // serving ticket, and returns true; otherwise restores the caller's
  // interrupt state and returns false without advancing `next_ticket_`, so
  // FIFO ticket ordering of `Lock()` waiters is preserved.
  bool TryLock() {
    const bool irq_was_enabled = internal::LocalSaveAndDisableInterrupts();
    const uintptr_t current_owner = internal::CurrentOwnerId();
    DCHECK(current_owner != 0);
    DCHECK(owner_.load(std::memory_order_relaxed) != current_owner);

    int64_t expected = serving_ticket_.load(std::memory_order_acquire);
    if (next_ticket_.load(std::memory_order_relaxed) != expected ||
        !next_ticket_.compare_exchange_strong(expected, expected + 1,
                                              std::memory_order_acquire,
                                              std::memory_order_relaxed)) {
      internal::LocalRestoreInterrupts(irq_was_enabled);
      return false;
    }

    FinishAcquire(current_owner, irq_was_enabled);
    return true;
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
  void FinishAcquire(const uintptr_t current_owner,
                     const bool irq_was_enabled) {
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
