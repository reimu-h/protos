#ifndef PROTOS_KILL_DELAY_INJECTION_H_
#define PROTOS_KILL_DELAY_INJECTION_H_

// Compile-time opt-in delay injection (`INJECT_KILL_DELAY=<variant>`) for
// proving that each of the 7 kill-latency checks in
// `selftest/boot_selftest.cpp` still catches genuine kernel kill-path
// regressions although they exclude hypervisor steal time (steal_time.h). When
// `PROTOS_INJECT_KILL_DELAY == 0` (the default), all helpers compile to no-ops.

#include <cstdint>

#include "check.h"
#include "cpu_clock.h"
#include "interrupts.h"
#include "smp.h"
#include "spinlock.h"

#ifndef PROTOS_INJECT_KILL_DELAY
#if defined(PROTOS_INJECT_KILL_DELAY_spinner)
#define PROTOS_INJECT_KILL_DELAY 1
#elif defined(PROTOS_INJECT_KILL_DELAY_mid_syscall)
#define PROTOS_INJECT_KILL_DELAY 2
#elif defined(PROTOS_INJECT_KILL_DELAY_teardown)
#define PROTOS_INJECT_KILL_DELAY 3
#elif defined(PROTOS_INJECT_KILL_DELAY_concurrent_killers)
#define PROTOS_INJECT_KILL_DELAY 4
#elif defined(PROTOS_INJECT_KILL_DELAY_busy_killer)
#define PROTOS_INJECT_KILL_DELAY 5
#elif defined(PROTOS_INJECT_KILL_DELAY_runaway)
#define PROTOS_INJECT_KILL_DELAY 6
#elif defined(PROTOS_INJECT_KILL_DELAY_shutdown)
#define PROTOS_INJECT_KILL_DELAY 7
#else
#define PROTOS_INJECT_KILL_DELAY 0
#endif
#endif

static_assert(PROTOS_INJECT_KILL_DELAY >= 0 && PROTOS_INJECT_KILL_DELAY <= 7,
              "Invalid PROTOS_INJECT_KILL_DELAY value");

namespace protos {

enum class KillDelayInjection : int {
  kNone = 0,
  kSpinner = 1,
  kMidSyscall = 2,
  kTeardown = 3,
  kConcurrentKillers = 4,
  kBusyKiller = 5,
  kRunaway = 6,
  kShutdown = 7,
};

inline constexpr KillDelayInjection kActiveKillDelayInjection =
    static_cast<KillDelayInjection>(PROTOS_INJECT_KILL_DELAY);
inline constexpr KillDelayInjection kInjectedKillDelay =
    kActiveKillDelayInjection;

// Busy-waits with interrupts enabled and no spinlock held until at least
// `ticks` of non-stolen CPU-0 timer ticks have elapsed (`raw_ticks -
// ceil(own_steal_ns / P_low) >= ticks`), summing own-CPU steal deltas across
// same-CPU sample pairs so cross-CPU migrations never compare distinct CPUs'
// cumulative boot steal counters, and guaranteeing that `KillTicks(stats)`
// exceeds the targeted check's bound (`2 * bound + 5` ticks).
inline void BusyWaitKillMeasureTicks(const int64_t ticks) {
#if PROTOS_INJECT_KILL_DELAY != 0
  if (ticks <= 0) {
    return;
  }
  DCHECK(!HoldsAnyIrqSpinLock());
  const int64_t start_ticks =
      SmpGetCpuLocal(0)->timer_ticks.load(std::memory_order_relaxed);
  int prev_cpu = -1;
  int64_t prev_steal_ns = -1;
  {
    const ScopedIrqDisable irq_disable;
    const CpuLocal* const cpu = CurrentCpuOrNullIrqsOff();
    prev_cpu = (cpu != nullptr) ? cpu->cpu_id : -1;
    prev_steal_ns = SmpOwnStealNsIrqsOff();
  }
  int64_t accumulated_steal_ns = 0;
  const int64_t tick_lower_ns = SmpTimerTickNsLowerBound();
  for (;;) {
    const int64_t now_ticks =
        SmpGetCpuLocal(0)->timer_ticks.load(std::memory_order_relaxed);
    int cur_cpu = -1;
    int64_t cur_steal_ns = -1;
    {
      const ScopedIrqDisable irq_disable;
      const CpuLocal* const cpu = CurrentCpuOrNullIrqsOff();
      cur_cpu = (cpu != nullptr) ? cpu->cpu_id : -1;
      cur_steal_ns = SmpOwnStealNsIrqsOff();
    }
    if (cur_cpu >= 0 && cur_cpu == prev_cpu && prev_steal_ns >= 0 &&
        cur_steal_ns > prev_steal_ns) {
      accumulated_steal_ns += (cur_steal_ns - prev_steal_ns);
    }
    prev_cpu = cur_cpu;
    prev_steal_ns = cur_steal_ns;
    const int64_t steal_ticks =
        (tick_lower_ns > 0 && accumulated_steal_ns > 0)
            ? ((accumulated_steal_ns + tick_lower_ns - 1) / tick_lower_ns)
            : 0;
    const int64_t effective_ticks = (now_ticks - start_ticks) - steal_ticks;
    if (effective_ticks >= ticks) {
      break;
    }
    asm volatile("pause" : : : "memory");
  }
#else
  (void)ticks;
#endif
}

}  // namespace protos

#endif  // PROTOS_KILL_DELAY_INJECTION_H_
