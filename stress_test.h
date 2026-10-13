#ifndef PROTOS_STRESS_TEST_H_
#define PROTOS_STRESS_TEST_H_

#include <atomic>
#include <cstdint>

#include "smp.h"

namespace protos {

// Returns true if the current x86-64 processor advertises `RDRAND` support via
// `CPUID.01H:ECX[30]` (or what the installed `StressTestHooks` report).
bool CpuSupportsRdrand();

// Attempts to read a 64-bit hardware random word via `rdrand` (retrying up to
// 10 times if the hardware RNG FIFO is transiently empty). Writes the result to
// non-null `*out` and returns true on success, or zeroes `*out` and returns
// false cleanly if `RDRAND` is unsupported or retries are exhausted. Validates
// `out != nullptr` with `DCHECK`.
bool TryReadRdrand64(uint64_t* out);

// Reads the 64-bit x86-64 Time-Stamp Counter (`rdtsc`).
uint64_t ReadTsc64();

// Mixes two 64-bit words into one: combines them in the style of
// `boost::hash_combine` and applies the SplitMix64 finalizer. For a fixed `a`
// the result is a bijection of `b`; as a function of both words it is not
// invertible.
uint64_t MixEntropy64(uint64_t a, uint64_t b);

// Harvests a non-zero 64-bit master seed by combining `RDRAND` (when supported
// by the CPU), multiple `RDTSC` readings, per-CPU runtime state (`cpu_id`,
// `timer_ticks`, `ipi_count` when `CurrentCpuOrNull()` is bound), and caller
// `extra_entropy`. Guaranteed to return a non-zero value.
uint64_t HarvestHardwareEntropySeed(uint64_t extra_entropy = 0);

// Fast, deterministic 64-bit pseudorandom number generator (XorShift64*). Not
// thread-safe: use one instance per task.
class Prng64 {
 public:
  explicit constexpr Prng64(const uint64_t seed = 1)
      : state_(MixInitialSeed(seed)) {}

  // Generates the next 64-bit pseudorandom word using XorShift64*.
  uint64_t NextU64();

  // Returns an unbiased pseudorandom integer in `[0, bound - 1]`. Validates
  // `bound > 0` with `DCHECK`.
  uint64_t NextBounded(uint64_t bound);

  // Returns an unbiased pseudorandom signed 64-bit integer in
  // `[min_inclusive, max_inclusive]`. Validates `min_inclusive <=
  // max_inclusive` with `DCHECK`.
  int64_t NextRange(int64_t min_inclusive, int64_t max_inclusive);

  // Returns an unbiased pseudorandom signed `int` in
  // `[min_inclusive, max_inclusive]`. Validates `min_inclusive <=
  // max_inclusive` with `DCHECK`.
  int NextIntRange(int min_inclusive, int max_inclusive);

  // Returns a pseudorandom boolean with probability 1/2.
  bool NextBool();

  // Returns true with probability `numerator / denominator`. Validates
  // `denominator > 0` and `0 <= numerator <= denominator` with `DCHECK`.
  bool NextChance(int numerator, int denominator);

  // Returns a pseudorandom power-of-two alignment in `[min_align, max_align]`.
  // Validates with `DCHECK` that `min_align` and `max_align` are positive
  // powers of two with `min_align <= max_align`.
  int64_t NextPowerOfTwoAlignment(int64_t min_align, int64_t max_align);

  // Derives an independent child `Prng64` stream mixed with `stream_id` while
  // advancing this generator's state.
  Prng64 Fork(uint64_t stream_id);

  // Returns the current non-zero internal 64-bit state.
  constexpr uint64_t state() const { return state_; }

 private:
  static constexpr uint64_t MixInitialSeed(const uint64_t seed) {
    uint64_t z = seed + 0x9E3779B97F4A7C15ULL;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    z = z ^ (z >> 31);
    return (z != 0) ? z : 0x9E3779B97F4A7C15ULL;
  }

  uint64_t state_;
};

// Bound on the self-tests' busy waits, in TSC cycles: roughly 2.5-10 s on the
// 1-4 GHz TSCs of real and emulated x86-64 CPUs, well below the boot test's
// per-boot timeout. This is a heuristic; the TSC frequency is not calibrated.
constexpr uint64_t kSelfTestWaitTimeoutTscCycles = 10'000'000'000ULL;

// Valid ranges of the `StressTestConfig` fields. `ExecuteRandomMultiCpuStress`
// CHECK-fails on values outside them.
constexpr int kMaxStressWorkers = 64;
constexpr int kMaxStressRoundsPerWorker = 4096;
constexpr int kMaxStressLiveAllocsPerWorker = 8;
constexpr int kMinStressHeapAllocBytes = 16;  // kHeapAlignment
constexpr int kMaxStressHeapAllocBytes = 16384;
constexpr int kMaxStressPmmAllocFrames = 8;

// Configuration parameters for `ExecuteRandomMultiCpuStress`.
struct StressTestConfig {
  // Master seed of the run; 0 harvests one (see `ResolveStressTestSeed`).
  uint64_t seed = 0;
  // Number of worker tasks in `[0, kMaxStressWorkers]`; 0 selects
  // `2 * SmpCpuCount()`, capped at `kMaxStressWorkers`.
  int num_workers = 0;
  // Rounds each worker runs, in `[1, kMaxStressRoundsPerWorker]`.
  int rounds_per_worker = 24;
  // Allocations each worker holds at a time, in
  // `[1, kMaxStressLiveAllocsPerWorker]`.
  int max_live_allocs_per_worker = 4;
  // Largest random heap allocation in bytes, in
  // `[kMinStressHeapAllocBytes, kMaxStressHeapAllocBytes]`. Lowered further so
  // that all workers' live heap allocations fit a 128 KiB payload budget.
  int max_heap_alloc_bytes = 2048;
  // Largest random PMM allocation in frames, `[1, kMaxStressPmmAllocFrames]`.
  int max_pmm_alloc_frames = 3;
  // Speeds up every CPU's LAPIC timer 16x for the duration of the run.
  bool boost_lapic_timer = true;
  // Prints the seed, a summary line, and the PASS marker to UART and VGA.
  bool log_to_console = false;
  // Bound on each of the run's internal waits, in TSC cycles. A wait that
  // exceeds it fails the run.
  uint64_t wait_timeout_tsc_cycles = kSelfTestWaitTimeoutTscCycles;
};

// Execution and verification telemetry returned by
// `ExecuteRandomMultiCpuStress`.
struct StressTestStats {
  uint64_t seed = 0;
  int cpu_count = 0;
  // CPUs that ran at least one task of the run (bits 0-63 and 64-127).
  uint64_t cpu_participation_mask = 0;
  uint64_t cpu_participation_mask_high = 0;
  int64_t cooperative_yields = 0;
  // Non-yielding spin bursts run by the workers.
  int64_t preemptive_spin_bursts = 0;
  // LAPIC timer ticks that arrived on a CPU while a spin burst ran there.
  int64_t timer_ticks_during_spin = 0;
  // Proven preemptions of tasks that never yield (normally by the LAPIC
  // timer): spin bursts that ended on another CPU than they started on, plus
  // the preemption probe (see `ExecuteRandomMultiCpuStress`).
  int64_t timer_preemptions_observed = 0;
  int64_t wakeup_ipis_sent = 0;
  // Times a task found itself on another CPU than at its previous checkpoint
  // (task start, `Yield`, `Join`, or spin burst).
  int64_t task_migrations_observed = 0;
  // Tasks enqueued on CPU 0 that ran on another CPU because that CPU stole
  // them (`cpu_count - 1` per run on SMP).
  int64_t cross_cpu_steals_verified = 0;
  int64_t child_tasks_joined = 0;
  int64_t child_tasks_detached = 0;
  int64_t zombies_reaped = 0;
  // Smallest and largest weight of the child tasks the workers created.
  int64_t min_child_task_weight = 0;
  int64_t max_child_task_weight = 0;
  int64_t kmalloc_ops = 0;
  int64_t kmalloc_aligned_ops = 0;
  int64_t cpp_new_delete_ops = 0;
  int64_t pmm_alloc_ops = 0;
  int64_t payload_verifications = 0;
  int64_t payload_corruptions = 0;
  int64_t stack_canary_corruptions = 0;
  int64_t pmm_free_before = 0;
  int64_t pmm_free_after = 0;
  int64_t heap_free_before = 0;
  int64_t heap_free_after = 0;
};

// Returns `requested_seed` if it is non-zero, else a fresh non-zero seed from
// `HarvestHardwareEntropySeed()`.
uint64_t ResolveStressTestSeed(uint64_t requested_seed);

// Runs the randomized multi-CPU kernel stress workload across all online CPUs
// (`0 .. SmpCpuCount() - 1`): concurrent cooperative and preemptive context
// switches, cross-CPU work stealing and wakeup IPIs, dynamic task creation,
// joining, and zombie reaping with randomized weights in
// `[kMinTaskWeight, kMaxTaskWeight]` (both bounds included), and
// pattern-verified PMM and heap allocations. Then, from the bootstrap task, it
// verifies work stealing (on SMP: `cpu_count - 1` tasks enqueued on CPU 0
// while CPU 0 can't run them must be stolen by the other CPUs, one each) and
// timer preemption (a non-yielding task that ran on CPU 0 must be preempted
// so that the unstealable bootstrap task runs on CPU 0 again).
//
// Preconditions (CHECKed where possible): called on CPU 0 by the bootstrap
// task after `TaskInit()`, with interrupts and preemption enabled, no other
// live tasks, and idle APs (the LAPIC timer boost and restore use
// `SmpRunOnAllCpus`). Restores the LAPIC timers and all PMM/heap memory.
//
// Any invariant violation prints
// `[STRESS] random_multicpu_stress: FAIL (<check>) seed=<seed>` and panics via
// `CHECK` instead of being reported in the returned stats.
StressTestStats ExecuteRandomMultiCpuStress(const StressTestConfig& config);

// Entry point that `kernel_main` runs in a loop once
// `RunBootVerificationSuite()` passed. Runs `ExecuteRandomMultiCpuStress` with
// the default `StressTestConfig` and console logging (using `seed_override`
// if non-zero, else a harvested seed) and prints the
// `[STRESS] random_multicpu_stress: PASS` marker to UART and VGA.
void RunRandomMultiCpuStressTest(uint64_t seed_override = 0);

// Platform hooks of the stress test. A null member selects the default, which
// uses the real hardware; host unit tests install fakes.
struct StressTestHooks {
  // Called on each iteration of a busy-wait loop. Default: `pause`.
  void (*cpu_relax)() = nullptr;
  // Called on each iteration of a loop that deliberately never yields, so that
  // a host test can deliver simulated timer interrupts there. Default: no-op.
  void (*non_yielding_spin)() = nullptr;
  // Reports whether `RDRAND` is available. Default: `CPUID.01H:ECX[30]`.
  bool (*rdrand_supported)() = nullptr;
  // Executes one `RDRAND`; returns false if no random value was available.
  // Default: the `rdrand` instruction.
  bool (*rdrand_attempt)(uint64_t* out) = nullptr;
};

// Installs `hooks` (pass `{}` to restore the defaults). Not thread-safe: call
// it only while no stress-test code runs.
void SetStressTestHooksForTest(const StressTestHooks& hooks);

// Helpers shared by the boot self-tests (memory_tests.cpp) and the stress test.

// Set of CPU indices `0 .. kMaxSupportedCpus - 1`.
struct CpuMask128 {
  uint64_t low = 0;   // CPUs 0-63.
  uint64_t high = 0;  // CPUs 64-127.

  // Returns `{0, ..., cpu_count - 1}` for `0 <= cpu_count <= 128`.
  static constexpr CpuMask128 FirstCpus(const int cpu_count) {
    CpuMask128 mask;
    if (cpu_count >= 64) {
      mask.low = ~0ULL;
    } else if (cpu_count > 0) {
      mask.low = (1ULL << static_cast<uint32_t>(cpu_count)) - 1ULL;
    }
    if (cpu_count >= 128) {
      mask.high = ~0ULL;
    } else if (cpu_count > 64) {
      mask.high = (1ULL << static_cast<uint32_t>(cpu_count - 64)) - 1ULL;
    }
    return mask;
  }

  // Returns true if every CPU of `other` is also in this set.
  constexpr bool ContainsAll(const CpuMask128& other) const {
    return (low & other.low) == other.low && (high & other.high) == other.high;
  }

  // Returns true if `cpu` is in this set.
  constexpr bool Contains(const int cpu) const {
    if (cpu < 0 || cpu >= 128) {
      return false;
    }
    const uint64_t word = (cpu < 64) ? low : high;
    return ((word >> static_cast<uint32_t>(cpu & 63)) & 1ULL) != 0;
  }

  constexpr bool operator==(const CpuMask128& other) const = default;
};
static_assert(kMaxSupportedCpus <= 128);

// `CpuMask128` that code on different CPUs can update concurrently.
class AtomicCpuMask128 {
 public:
  // Adds `cpu` and returns true if it was not in the set yet. CPUs outside
  // `[0, kMaxSupportedCpus)` are ignored.
  bool Add(const int cpu) {
    if (cpu < 0 || cpu >= kMaxSupportedCpus) {
      return false;
    }
    std::atomic<uint64_t>& word = (cpu < 64) ? low_ : high_;
    const uint64_t bit = 1ULL << static_cast<uint32_t>(cpu & 63);
    return (word.fetch_or(bit, std::memory_order_acq_rel) & bit) == 0;
  }

  CpuMask128 Load() const {
    CpuMask128 mask;
    mask.low = low_.load(std::memory_order_acquire);
    mask.high = high_.load(std::memory_order_acquire);
    return mask;
  }

  void Clear() {
    low_.store(0, std::memory_order_release);
    high_.store(0, std::memory_order_release);
  }

 private:
  std::atomic<uint64_t> low_{0};
  std::atomic<uint64_t> high_{0};
};

// Deadline of a bounded busy wait, measured with the TSC of the calling CPU.
// A TSC that reads lower after a migration to another CPU does not expire it,
// and neither does any time span if `timeout_tsc_cycles > INT64_MAX`.
class TscDeadline {
 public:
  explicit TscDeadline(const uint64_t timeout_tsc_cycles)
      : start_(ReadTsc64()), timeout_(timeout_tsc_cycles) {}

  bool Expired() const {
    const int64_t elapsed = static_cast<int64_t>(ReadTsc64() - start_);
    return elapsed >= 0 && static_cast<uint64_t>(elapsed) >= timeout_;
  }

 private:
  const uint64_t start_;
  const uint64_t timeout_;
};

// Programs the LAPIC timer initial count of every CPU via `SmpRunOnAllCpus`.
// Call it only on CPU 0 while no tasks run on the APs.
void SetLapicTimerInitialCountOnAllCpus(uint32_t initial_count);

// Speeds up the LAPIC timer of every CPU 16x if `enable` is true (and the
// calibrated initial count is large enough) until `Restore()` or destruction
// restores the calibrated count on every CPU. Construct, restore, and destroy
// it only on CPU 0 while no tasks run on the APs.
class ScopedLapicTimerBoost {
 public:
  explicit ScopedLapicTimerBoost(bool enable);
  ~ScopedLapicTimerBoost() { Restore(); }

  ScopedLapicTimerBoost(const ScopedLapicTimerBoost&) = delete;
  ScopedLapicTimerBoost& operator=(const ScopedLapicTimerBoost&) = delete;

  void Restore();

 private:
  const uint32_t calibrated_count_;
  bool boosted_ = false;
};

}  // namespace protos

#endif  // PROTOS_STRESS_TEST_H_
