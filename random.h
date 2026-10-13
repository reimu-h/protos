#ifndef PROTOS_RANDOM_H_
#define PROTOS_RANDOM_H_

#include <cstdint>

namespace protos {

// Entropy sources and a pseudorandom number generator for kernel code. None of
// this is suitable for cryptographic use.

// Returns true if the current x86-64 processor advertises `RDRAND` support via
// `CPUID.01H:ECX[30]` (or what the installed `RandomHooks` report).
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

// Harvests a non-zero 64-bit seed by combining `RDRAND` (when supported by the
// CPU), multiple `RDTSC` readings, a global call counter, per-CPU runtime state
// (`cpu_id`, `apic_id`, `timer_ticks`, `ipi_count` when `CurrentCpuOrNull()` is
// bound), and caller `extra_entropy`. Guaranteed to return a non-zero value.
// Safe to call concurrently from any CPU.
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

// Platform hooks of the hardware entropy sources. A null member selects the
// default, which uses the real hardware; host unit tests install fakes.
struct RandomHooks {
  // Reports whether `RDRAND` is available. Default: `CPUID.01H:ECX[30]`.
  bool (*rdrand_supported)() = nullptr;
  // Executes one `RDRAND`; returns false if no random value was available.
  // Default: the `rdrand` instruction.
  bool (*rdrand_attempt)(uint64_t* out) = nullptr;
};

// Installs `hooks` (pass `{}` to restore the defaults). Not thread-safe: call
// it only while no other code uses this module.
void SetRandomHooksForTest(const RandomHooks& hooks);

}  // namespace protos

#endif  // PROTOS_RANDOM_H_
