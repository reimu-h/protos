#include "random.h"

#include <atomic>
#include <cstdint>

#include "check.h"
#include "smp.h"

namespace protos {

namespace {

constexpr uint32_t kCpuidLeafFeatures = 1u;
constexpr uint32_t kCpuidEcxRdrandBit = 1u << 30;
constexpr int kRdrandMaxRetries = 10;

std::atomic<uint64_t> g_entropy_call_seq{1};

RandomHooks g_hooks;

static bool HardwareSupportsRdrand() {
  uint32_t eax = kCpuidLeafFeatures;
  uint32_t ebx = 0;
  uint32_t ecx = 0;
  uint32_t edx = 0;
  asm volatile("cpuid" : "+a"(eax), "=b"(ebx), "+c"(ecx), "=d"(edx) : : "cc");
  return (ecx & kCpuidEcxRdrandBit) != 0;
}

static bool HardwareRdrandAttempt(uint64_t* const out) {
  uint64_t val = 0;
  uint8_t ok = 0;
  asm volatile("rdrand %0" : "=r"(val), "=@ccc"(ok) : : "cc");
  *out = val;
  return ok != 0;
}

}  // namespace

void SetRandomHooksForTest(const RandomHooks& hooks) { g_hooks = hooks; }

bool CpuSupportsRdrand() {
  return (g_hooks.rdrand_supported != nullptr) ? g_hooks.rdrand_supported()
                                               : HardwareSupportsRdrand();
}

bool TryReadRdrand64(uint64_t* const out) {
  DCHECK(out != nullptr);
  if (!CpuSupportsRdrand()) {
    *out = 0;
    return false;
  }
  bool (*const attempt_fn)(uint64_t*) = (g_hooks.rdrand_attempt != nullptr)
                                            ? g_hooks.rdrand_attempt
                                            : HardwareRdrandAttempt;
  for (int attempt = 0; attempt < kRdrandMaxRetries; ++attempt) {
    uint64_t val = 0;
    if (attempt_fn(&val)) {
      *out = val;
      return true;
    }
    asm volatile("pause" : : : "memory");
  }
  *out = 0;
  return false;
}

uint64_t ReadTsc64() {
  uint32_t eax = 0;
  uint32_t edx = 0;
  asm volatile("rdtsc" : "=a"(eax), "=d"(edx));
  return (static_cast<uint64_t>(edx) << 32) | static_cast<uint64_t>(eax);
}

uint64_t MixEntropy64(const uint64_t a, const uint64_t b) {
  uint64_t z = a ^ (b + 0x9E3779B97F4A7C15ULL + (a << 6) + (a >> 2));
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

uint64_t HarvestHardwareEntropySeed(const uint64_t extra_entropy) {
  const uint64_t tsc0 = ReadTsc64();
  uint64_t rdrand_val = 0;
  const bool has_rdrand = TryReadRdrand64(&rdrand_val);
  const uint64_t tsc1 = ReadTsc64();

  uint64_t cpu_entropy = 0xD1B54A32D192ED03ULL;
  const CpuLocal* const cpu = CurrentCpuOrNull();
  if (cpu != nullptr) {
    const uint64_t ticks =
        static_cast<uint64_t>(cpu->timer_ticks.load(std::memory_order_relaxed));
    const uint64_t ipis =
        static_cast<uint64_t>(cpu->ipi_count.load(std::memory_order_relaxed));
    cpu_entropy = MixEntropy64((static_cast<uint64_t>(cpu->cpu_id) << 32) |
                                   static_cast<uint64_t>(cpu->apic_id),
                               MixEntropy64(ticks, ipis));
  }

  const uint64_t seq =
      g_entropy_call_seq.fetch_add(1, std::memory_order_relaxed);
  uint64_t mixed = MixEntropy64(tsc0, tsc1 ^ (seq * 0x9E3779B97F4A7C15ULL));
  mixed =
      MixEntropy64(mixed, rdrand_val ^ (has_rdrand ? 0xA5A55A5AA5A55A5AULL
                                                   : 0x5A5AA5A55A5AA5A5ULL));
  mixed = MixEntropy64(mixed, cpu_entropy);
  mixed = MixEntropy64(mixed, extra_entropy);
  return (mixed != 0) ? mixed : 0x9E3779B97F4A7C15ULL;
}

uint64_t Prng64::NextU64() {
  uint64_t x = state_;
  if (x == 0) {
    x = 0x9E3779B97F4A7C15ULL;
  }
  x ^= x >> 12;
  x ^= x << 25;
  x ^= x >> 27;
  state_ = x;
  return x * 0x2545F4914F6CDD1DULL;
}

uint64_t Prng64::NextBounded(const uint64_t bound) {
  DCHECK(bound > 0);
  const uint64_t threshold = (0ULL - bound) % bound;
  for (;;) {
    const uint64_t r = NextU64();
    if (r >= threshold) {
      return r % bound;
    }
  }
}

int64_t Prng64::NextRange(const int64_t min_inclusive,
                          const int64_t max_inclusive) {
  DCHECK(min_inclusive <= max_inclusive);
  const uint64_t span = static_cast<uint64_t>(max_inclusive) -
                        static_cast<uint64_t>(min_inclusive);
  if (span == ~0ULL) {
    return static_cast<int64_t>(NextU64());
  }
  const uint64_t offset = NextBounded(span + 1ULL);
  return static_cast<int64_t>(static_cast<uint64_t>(min_inclusive) + offset);
}

int Prng64::NextIntRange(const int min_inclusive, const int max_inclusive) {
  DCHECK(min_inclusive <= max_inclusive);
  return static_cast<int>(NextRange(static_cast<int64_t>(min_inclusive),
                                    static_cast<int64_t>(max_inclusive)));
}

bool Prng64::NextBool() { return (NextU64() & 1ULL) != 0; }

bool Prng64::NextChance(const int numerator, const int denominator) {
  DCHECK(denominator > 0);
  DCHECK(numerator >= 0);
  DCHECK(numerator <= denominator);
  if (numerator == 0) {
    return false;
  }
  if (numerator == denominator) {
    return true;
  }
  return NextBounded(static_cast<uint64_t>(denominator)) <
         static_cast<uint64_t>(numerator);
}

int64_t Prng64::NextPowerOfTwoAlignment(const int64_t min_align,
                                        const int64_t max_align) {
  DCHECK(min_align > 0);
  DCHECK((min_align & (min_align - 1)) == 0);
  DCHECK(max_align >= min_align);
  DCHECK((max_align & (max_align - 1)) == 0);

  int min_exp = 0;
  while ((1LL << min_exp) < min_align) {
    ++min_exp;
  }
  int max_exp = min_exp;
  while ((1LL << max_exp) < max_align) {
    ++max_exp;
  }
  const int chosen_exp = NextIntRange(min_exp, max_exp);
  return 1LL << chosen_exp;
}

Prng64 Prng64::Fork(const uint64_t stream_id) {
  const uint64_t next_word = NextU64();
  return Prng64(MixEntropy64(next_word, stream_id ^ 0xD1B54A32D192ED03ULL));
}

}  // namespace protos
