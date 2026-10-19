#ifndef PROTOS_STEAL_TIME_H_
#define PROTOS_STEAL_TIME_H_

// KVM paravirtualized steal-time structure (`MSR_KVM_STEAL_TIME`,
// `0x4B564D03`), feature discovery via CPUID, lock-free seqlock reader, and
// steal-time conversion helpers for kill-latency accounting.

#include <atomic>
#include <cstdint>
#include <type_traits>

namespace protos {

inline constexpr uint32_t kCpuidLeafProcessorInfo = 1u;
inline constexpr uint32_t kCpuidEcxHypervisor = 1u << 31;

inline constexpr uint32_t kCpuidHypervisorMinLeaf = 0x40000000u;
inline constexpr uint32_t kCpuidHypervisorMaxLeaf = 0x40010000u;
inline constexpr uint32_t kCpuidHypervisorLeafStride = 0x100u;

// ASCII "KVMKVMKVM\0\0\0" in EBX:ECX:EDX of the KVM hypervisor signature leaf.
inline constexpr uint32_t kKvmSignatureEbx = 0x4B4D564Bu;
inline constexpr uint32_t kKvmSignatureEcx = 0x564B4D56u;
inline constexpr uint32_t kKvmSignatureEdx = 0x0000004Du;

// `KVM_FEATURE_STEAL_TIME` in EAX of `base + 1` (`CPUID_KVM_FEATURES`).
inline constexpr uint32_t kKvmFeatureStealTime = 1u << 5;

inline constexpr uint32_t kMsrKvmStealTime = 0x4B564D03u;
inline constexpr uint64_t kKvmStealTimeSystemEnabled = 1u;

// Architectural layout written by KVM at the 64-byte-aligned physical address
// programmed into `kMsrKvmStealTime` (with bit 0 set).
struct alignas(64) KvmStealTime {
  uint64_t steal = 0;
  uint32_t version = 0;
  uint32_t flags = 0;
  uint8_t preempted = 0;
  uint8_t u8_pad[3] = {};
  uint32_t pad[11] = {};
};

static_assert(sizeof(KvmStealTime) == 64);
static_assert(alignof(KvmStealTime) == 64);
static_assert(std::is_standard_layout_v<KvmStealTime>);

// Probes CPUID via `cpuid_fn(leaf, subleaf)` (returning a struct with `.eax`,
// `.ebx`, `.ecx`, `.edx`) for `KVM_FEATURE_STEAL_TIME`:
// 1. Leaf 1 ECX bit 31 (`HYPERVISOR`) must be set.
// 2. Scans `0x40000000 .. 0x40010000` at stride `0x100` for the `"KVMKVMKVM"`
//    hypervisor signature, because KVM may reside at a higher base leaf when
//    another hypervisor Enlightenment (e.g. Hyper-V) occupies `0x40000000`.
// 3. Checks `base + 1` EAX bit 5 (`KVM_FEATURE_STEAL_TIME`).
template <typename CpuidFn>
inline bool HasKvmStealTime(CpuidFn cpuid_fn) {
  const auto leaf1 = cpuid_fn(kCpuidLeafProcessorInfo, 0);
  if ((leaf1.ecx & kCpuidEcxHypervisor) == 0) {
    return false;
  }
  for (uint32_t base = kCpuidHypervisorMinLeaf; base <= kCpuidHypervisorMaxLeaf;
       base += kCpuidHypervisorLeafStride) {
    const auto sig = cpuid_fn(base, 0);
    if (sig.ebx == kKvmSignatureEbx && sig.ecx == kKvmSignatureEcx &&
        sig.edx == kKvmSignatureEdx) {
      const uint32_t max_leaf = (sig.eax >= base) ? sig.eax : (base + 1u);
      if (max_leaf < base + 1u) {
        return false;
      }
      const auto feat = cpuid_fn(base + 1u, 0);
      return (feat.eax & kKvmFeatureStealTime) != 0;
    }
  }
  return false;
}

// Evaluates a single seqlock snapshot `(v1, raw_steal, v2)`: returns the
// non-negative nanoseconds value if `v1` is even, equals `v2`, and `raw_steal`
// fits in `int64_t`, or `-1` if an update was in progress (`v1` odd or
// `v1 != v2`) or `raw_steal` overflows `int64_t`.
inline constexpr int64_t StealNsFromSnapshot(const uint32_t v1,         //
                                             const uint64_t raw_steal,  //
                                             const uint32_t v2) {
  if ((v1 & 1u) != 0 || v1 != v2 ||
      raw_steal > static_cast<uint64_t>(INT64_MAX)) {
    return -1;
  }
  return static_cast<int64_t>(raw_steal);
}

// Reads `st.steal` (in nanoseconds) using the KVM seqlock protocol on
// `st.version`. Retries up to a bounded iteration count if an update is in
// flight on a remote vCPU; returns -1 if no consistent snapshot is obtained.
inline int64_t ReadKvmStealNs(const KvmStealTime& st) {
  constexpr int kMaxAttempts = 64;
  const std::atomic_ref<const uint32_t> version_ref(st.version);
  const std::atomic_ref<const uint64_t> steal_ref(st.steal);
  for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
    const uint32_t v1 = version_ref.load(std::memory_order_acquire);
    if ((v1 & 1u) != 0) {
      asm volatile("pause" : : : "memory");
      continue;
    }
    const uint64_t steal = steal_ref.load(std::memory_order_acquire);
    std::atomic_signal_fence(std::memory_order_acquire);
    const uint32_t v2 = version_ref.load(std::memory_order_relaxed);
    const int64_t result = StealNsFromSnapshot(v1, steal, v2);
    if (result >= 0) {
      return result;
    }
    asm volatile("pause" : : : "memory");
  }
  return -1;
}

// Returns a conservative upper bound (in nanoseconds) on the duration of one
// `GlobalTimerTicks()` tick (`P_conv = 16 * count + floor(16 * count / 1024) +
// 1` ns) so `StealNsToTicks` never over-subtracts steal time. Returns 0 if
// `initial_count == 0`.
inline constexpr int64_t TimerTickNsUpperBound(const uint32_t initial_count) {
  if (initial_count == 0) {
    return 0;
  }
  const int64_t base_ns = 16LL * static_cast<int64_t>(initial_count);
  return base_ns + (base_ns / 1024LL) + 1LL;
}

// Returns a conservative lower bound (in nanoseconds) on the duration of one
// `GlobalTimerTicks()` tick (`P_low = 16 * count - floor(16 * count / 1024)`
// ns), used by `BusyWaitKillMeasureTicks` so injected delays are never cut
// short by steal-time subtraction. Returns 0 if `initial_count == 0`.
inline constexpr int64_t TimerTickNsLowerBound(const uint32_t initial_count) {
  if (initial_count == 0) {
    return 0;
  }
  const int64_t base_ns = 16LL * static_cast<int64_t>(initial_count);
  return base_ns - (base_ns / 1024LL);
}

// Converts `steal_ns` nanoseconds of excluded steal time to whole timer ticks
// via floor division (`floor(steal_ns / tick_ns_upper_bound)`).
inline constexpr int64_t StealNsToTicks(const int64_t steal_ns,
                                        const int64_t tick_ns_upper_bound) {
  if (steal_ns <= 0 || tick_ns_upper_bound <= 0) {
    return 0;
  }
  return steal_ns / tick_ns_upper_bound;
}

// Steal-time snapshots recorded along a `UserProcess` kill lifecycle for the
// steal exclusion formula:
//   `E1`: killed task's own CPU steal from first kill notice (`EndKilled()` or
//         pre-entry kill branch in `Run()`) to `RecordEnd()` right before
//         `end_tick_.store(...)`, valid only when the task stayed on the same
//         CPU with zero context switches (`notice_cpu == end_cpu &&
//         notice_switches == end_switches`).
//   `B0`: CPU 0's own steal between `Kill()` (right before reading
//         `GlobalTimerTicks()`) and `stats()` on CPU 0.
//   `ExcludedNs = max(0, E1 - B0)`.
struct KillStealRecord {
  int64_t killer_steal_ns_at_kill = -1;
  int64_t killer_steal_ns_at_observe = -1;
  int64_t killed_steal_ns_at_notice = -1;
  int64_t killed_steal_ns_at_end = -1;
  int notice_cpu = -1;
  int end_cpu = -1;
  int64_t notice_switches = -1;
  int64_t end_switches = -1;
};

inline constexpr int64_t KillExcludedStealNs(const KillStealRecord& r) {
  if (r.notice_cpu < 0 || r.notice_cpu != r.end_cpu || r.notice_switches < 0 ||
      r.notice_switches != r.end_switches || r.killed_steal_ns_at_notice < 0 ||
      r.killed_steal_ns_at_end < r.killed_steal_ns_at_notice ||
      r.killer_steal_ns_at_kill < 0 ||
      r.killer_steal_ns_at_observe < r.killer_steal_ns_at_kill) {
    return 0;
  }
  const int64_t e1 = r.killed_steal_ns_at_end - r.killed_steal_ns_at_notice;
  const int64_t b0 = r.killer_steal_ns_at_observe - r.killer_steal_ns_at_kill;
  return (e1 > b0) ? (e1 - b0) : 0;
}

}  // namespace protos

#endif  // PROTOS_STEAL_TIME_H_
