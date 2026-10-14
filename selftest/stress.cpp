#include "selftest/stress.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>

#include "check.h"
#include "heap.h"
#include "idt.h"
#include "pmm.h"
#include "random.h"
#include "smp.h"
#include "spinlock.h"
#include "task.h"
#include "uart.h"
#include "vga.h"

namespace protos {

namespace {

// Total number of detached children that the workers of one run may create.
constexpr int kMaxDetachedChildrenPerRun = 64;
// Payload budget, in bytes, of all live worker heap allocations together.
constexpr int kLiveHeapBudgetBytes = 128 * 1024;
// Preemption probes to try before giving up. A probe proves nothing only if
// another CPU stole its spinner before the spinner ran on CPU 0.
constexpr int kMaxPreemptProbeAttempts = 16;

static_assert(kMinStressHeapAllocBytes == kHeapAlignment);

constexpr uint64_t kHeapObjectMagic = 0x5354524553534F42ULL;  // "STRESSOB"
constexpr uint64_t kHeapObjectPoison = 0xDEAD535452455353ULL;

std::atomic<uint64_t> g_stress_pass_count{0};

StressTestHooks g_hooks;

static void CpuRelax() {
  if (g_hooks.cpu_relax != nullptr) {
    g_hooks.cpu_relax();
    return;
  }
  asm volatile("pause" : : : "memory");
}

static void NonYieldingSpinPoint() {
  if (g_hooks.non_yielding_spin != nullptr) {
    g_hooks.non_yielding_spin();
  }
}

static void SetLapicTimerInitialCountOnCpu(const int /*cpu_index*/,
                                           void* const context) {
  DCHECK(context != nullptr);
  SmpSetLocalApicTimerInitialCount(*static_cast<const uint32_t*>(context));
}

// Prints the FAIL marker with the failed check and the seed of the run over
// UART and VGA, then panics.
[[noreturn]] static void FailStressRun(const char* const check,  //
                                       const uint64_t seed,      //
                                       const char* const file,   //
                                       const int line) {
  UartWrite("[STRESS] random_multicpu_stress: FAIL (");
  UartWrite(check);
  UartWrite(") seed=");
  UartWriteHex(seed);
  UartWrite("\n");
  VgaWrite("[STRESS] random_multicpu_stress: FAIL (");
  VgaWrite(check);
  VgaWrite(")\n");
  internal::CheckFailure(check, file, line);
}

// Like `CHECK(cond)`, but prints the stress test's FAIL marker with the seed
// of the run first.
#define STRESS_CHECK(seed, cond)                         \
  do {                                                   \
    if (__builtin_expect(!static_cast<bool>(cond), 0)) { \
      FailStressRun(#cond, (seed), __FILE__, __LINE__);  \
    }                                                    \
  } while (false)

enum class StressAllocKind : uint8_t {
  kNone = 0,
  kKmalloc = 1,
  kKmallocAligned = 2,
  kCppNewDelete = 3,
  kPmmFrames = 4,
};

struct StressHeapObject {
  uint64_t magic = 0;
  uint64_t seed_tag = 0;
  int worker_id = 0;
  int round_created = 0;
  uint64_t payload[16] = {};
  uint64_t checksum = 0;

  // Returns nullptr when the heap is exhausted. Being non-throwing makes every
  // `new StressHeapObject()` check for that before constructing the object.
  static void* operator new(const size_t size) noexcept {
    return Kmalloc(static_cast<int64_t>(size));
  }

  static void operator delete(void* const ptr) noexcept { Kfree(ptr); }

  static void operator delete(void* const ptr, const size_t /*size*/) noexcept {
    Kfree(ptr);
  }
};
static_assert(noexcept(StressHeapObject::operator new(0)));

struct LiveAllocation {
  StressAllocKind kind = StressAllocKind::kNone;
  void* heap_ptr = nullptr;
  StressHeapObject* obj_ptr = nullptr;
  uintptr_t pmm_phys_addr = 0;
  int64_t byte_size = 0;
  int64_t alignment = 0;
  int64_t frame_count = 0;
  bool single_frame_api = false;
  uint64_t pattern_seed = 0;
};

struct StressSharedState {
  Task* bootstrap_task = nullptr;
  int cpu_count = 1;
  int num_workers = 1;
  int rounds_per_worker = 1;
  int max_live_allocs_per_worker = 1;
  int effective_max_heap_bytes = 256;
  int max_pmm_alloc_frames = 1;
  uint64_t master_seed = 1;
  uint64_t wait_timeout_tsc_cycles = kSelfTestWaitTimeoutTscCycles;

  AtomicCpuMask128 cpu_participation;
  std::atomic<int64_t> cooperative_yields{0};
  std::atomic<int64_t> preemptive_spin_bursts{0};
  std::atomic<int64_t> timer_ticks_during_spin{0};
  std::atomic<int64_t> timer_preemptions_observed{0};
  std::atomic<int64_t> wakeup_ipis_sent{0};
  std::atomic<int64_t> task_migrations_observed{0};
  std::atomic<int64_t> cross_cpu_steals_verified{0};
  std::atomic<int64_t> child_tasks_joined{0};
  std::atomic<int64_t> child_tasks_detached{0};
  std::atomic<int64_t> detached_children_finished{0};
  std::atomic<int64_t> zombies_reaped{0};
  std::atomic<int64_t> min_child_task_weight{INT64_MAX};
  std::atomic<int64_t> max_child_task_weight{0};
  std::atomic<int64_t> kmalloc_ops{0};
  std::atomic<int64_t> kmalloc_aligned_ops{0};
  std::atomic<int64_t> cpp_new_delete_ops{0};
  std::atomic<int64_t> pmm_alloc_ops{0};
  std::atomic<int64_t> payload_verifications{0};
  std::atomic<int64_t> payload_corruptions{0};
  std::atomic<int64_t> stack_canary_corruptions{0};
  std::atomic<int> workers_completed{0};
  std::atomic<bool> invariant_failed{false};
};

struct DetachedChildContext {
  StressSharedState* shared = nullptr;
  uint64_t seed = 0;
  bool allow_yield = false;
};

struct JoinedChildContext {
  StressSharedState* shared = nullptr;
  Task* parent_task = nullptr;
  uint64_t expected_token = 0;
  std::atomic<uint64_t> observed_token{0};
  bool allow_yield = false;
};

// Shared by the steal-barrier tasks of VerifyStealingAndPreemption().
struct StealBarrierContext {
  StressSharedState* shared = nullptr;
  AtomicCpuMask128 running_cpus;
  std::atomic<int> ran_on_cpu0{0};
  std::atomic<bool> release{false};
};

// State of one attempt of RunPreemptionProbe().
struct PreemptProbeContext {
  uint64_t timeout_tsc_cycles = 0;
  std::atomic<int> start_cpu{-1};
  std::atomic<bool> finished{false};
  std::atomic<bool> release{false};
};

struct StressWorkerContext {
  StressSharedState* shared = nullptr;
  Task* task = nullptr;
  int worker_index = 0;
  uint64_t worker_seed = 0;
  LiveAllocation live_allocs[kMaxStressLiveAllocsPerWorker] = {};
  std::atomic<bool> done{false};
};

static uint8_t ComputePatternByte(const uint64_t pattern_seed,
                                  const int64_t index) {
  const uint64_t mixed =
      MixEntropy64(pattern_seed, static_cast<uint64_t>(index));
  return static_cast<uint8_t>((mixed ^ (mixed >> 24)) & 0xFFu);
}

static uint64_t ComputePatternWord(const uint64_t pattern_seed,
                                   const int64_t word_index) {
  return MixEntropy64(pattern_seed ^ 0xA5A5A5A55A5A5A5AULL,
                      static_cast<uint64_t>(word_index));
}

static void FillByteBufferPattern(uint8_t* const buf,       //
                                  const int64_t byte_size,  //
                                  const uint64_t pattern_seed) {
  DCHECK(buf != nullptr);
  DCHECK(byte_size >= 0);
  const int64_t word_count = byte_size / 8;
  uint64_t* const words = reinterpret_cast<uint64_t*>(buf);
  for (int64_t w = 0; w < word_count; ++w) {
    words[w] = ComputePatternWord(pattern_seed, w);
  }
  for (int64_t b = word_count * 8; b < byte_size; ++b) {
    buf[b] = ComputePatternByte(pattern_seed, b);
  }
}

static bool VerifyAndPoisonByteBuffer(uint8_t* const buf,
                                      const int64_t byte_size,
                                      const uint64_t pattern_seed) {
  DCHECK(buf != nullptr);
  DCHECK(byte_size >= 0);
  bool ok = true;
  const int64_t word_count = byte_size / 8;
  uint64_t* const words = reinterpret_cast<uint64_t*>(buf);
  for (int64_t w = 0; w < word_count; ++w) {
    if (words[w] != ComputePatternWord(pattern_seed, w)) {
      ok = false;
    }
    words[w] = 0xDEADBEEFCAFEBABEULL ^ static_cast<uint64_t>(w);
  }
  for (int64_t b = word_count * 8; b < byte_size; ++b) {
    if (buf[b] != ComputePatternByte(pattern_seed, b)) {
      ok = false;
    }
    buf[b] = 0x5Au;
  }
  return ok;
}

static void FillFrameRunPattern(const uintptr_t phys_addr,
                                const int64_t frame_count,
                                const uint64_t pattern_seed) {
  DCHECK(phys_addr != 0);
  DCHECK(frame_count > 0);
  uint64_t* const words = reinterpret_cast<uint64_t*>(phys_addr);
  const int64_t total_words =
      (frame_count * kPageSize) / static_cast<int64_t>(sizeof(uint64_t));
  // Fill every word of the allocated physical frame run.
  for (int64_t w = 0; w < total_words; ++w) {
    words[w] = ComputePatternWord(pattern_seed, w);
  }
}

static bool VerifyAndPoisonFrameRun(const uintptr_t phys_addr,
                                    const int64_t frame_count,
                                    const uint64_t pattern_seed) {
  DCHECK(phys_addr != 0);
  DCHECK(frame_count > 0);
  uint64_t* const words = reinterpret_cast<uint64_t*>(phys_addr);
  const int64_t total_words =
      (frame_count * kPageSize) / static_cast<int64_t>(sizeof(uint64_t));
  bool ok = true;
  for (int64_t w = 0; w < total_words; ++w) {
    if (words[w] != ComputePatternWord(pattern_seed, w)) {
      ok = false;
    }
    words[w] = 0xBADF00D0DEADCAFEULL ^ static_cast<uint64_t>(w);
  }
  return ok;
}

static void VerifyAndReleaseSlot(StressSharedState* const shared,
                                 LiveAllocation* const slot) {
  DCHECK(shared != nullptr);
  DCHECK(slot != nullptr);
  if (slot->kind == StressAllocKind::kNone) {
    return;
  }

  bool ok = true;
  switch (slot->kind) {
    case StressAllocKind::kKmalloc: {
      DCHECK(slot->heap_ptr != nullptr);
      const int64_t effective_bytes =
          (slot->byte_size > 0) ? slot->byte_size : kHeapAlignment;
      ok = VerifyAndPoisonByteBuffer(static_cast<uint8_t*>(slot->heap_ptr),  //
                                     effective_bytes,                        //
                                     slot->pattern_seed);
      Kfree(slot->heap_ptr);
      break;
    }
    case StressAllocKind::kKmallocAligned: {
      DCHECK(slot->heap_ptr != nullptr);
      const uintptr_t eff_align = static_cast<uintptr_t>(
          (slot->alignment > kHeapAlignment) ? slot->alignment
                                             : kHeapAlignment);
      if ((reinterpret_cast<uintptr_t>(slot->heap_ptr) & (eff_align - 1u)) !=
          0) {
        ok = false;
      }
      const int64_t effective_bytes =
          (slot->byte_size > 0) ? slot->byte_size : kHeapAlignment;
      if (!VerifyAndPoisonByteBuffer(static_cast<uint8_t*>(slot->heap_ptr),  //
                                     effective_bytes,                        //
                                     slot->pattern_seed)) {
        ok = false;
      }
      Kfree(slot->heap_ptr);
      break;
    }
    case StressAllocKind::kCppNewDelete: {
      StressHeapObject* const obj = slot->obj_ptr;
      DCHECK(obj != nullptr);
      uint64_t expected_checksum =
          obj->seed_tag ^ kHeapObjectMagic ^
          static_cast<uint64_t>(obj->worker_id) ^
          (static_cast<uint64_t>(obj->round_created) << 32);
      for (int i = 0; i < 16; ++i) {
        const uint64_t expected_word =
            ComputePatternWord(slot->pattern_seed, i);
        if (obj->payload[i] != expected_word) {
          ok = false;
        }
        expected_checksum = MixEntropy64(expected_checksum, expected_word);
        obj->payload[i] = kHeapObjectPoison;
      }
      if (obj->magic != kHeapObjectMagic ||
          obj->seed_tag != slot->pattern_seed ||
          obj->checksum != expected_checksum) {
        ok = false;
      }
      obj->magic = kHeapObjectPoison;
      obj->checksum = 0;
      delete obj;
      break;
    }
    case StressAllocKind::kPmmFrames: {
      DCHECK(slot->pmm_phys_addr != 0);
      DCHECK(slot->frame_count > 0);
      if ((slot->pmm_phys_addr & static_cast<uintptr_t>(kPageSize - 1)) != 0) {
        ok = false;
      }
      if (!VerifyAndPoisonFrameRun(slot->pmm_phys_addr,  //
                                   slot->frame_count,    //
                                   slot->pattern_seed)) {
        ok = false;
      }
      if (slot->single_frame_api) {
        DCHECK(slot->frame_count == 1);
        PmmFreeFrame(slot->pmm_phys_addr);
      } else {
        PmmFreeFrames(slot->pmm_phys_addr, slot->frame_count);
      }
      break;
    }
    case StressAllocKind::kNone:
      break;
  }

  shared->payload_verifications.fetch_add(1, std::memory_order_acq_rel);
  if (!ok) {
    shared->payload_corruptions.fetch_add(1, std::memory_order_acq_rel);
    shared->invariant_failed.store(true, std::memory_order_release);
  }
  *slot = LiveAllocation{};
}

static void AllocateIntoSlot(StressSharedState* const shared,  //
                             LiveAllocation* const slot,       //
                             Prng64* const prng,               //
                             const int worker_id,              //
                             const int round,                  //
                             const int alloc_kind_index) {
  DCHECK(shared != nullptr);
  DCHECK(slot != nullptr);
  DCHECK(prng != nullptr);
  DCHECK(slot->kind == StressAllocKind::kNone);

  const uint64_t pattern_seed = MixEntropy64(
      prng->NextU64(), (static_cast<uint64_t>(worker_id + 1) << 32) ^
                           static_cast<uint64_t>(round + 1));

  switch (alloc_kind_index & 3) {
    case 0: {
      const int64_t byte_size =
          prng->NextChance(1, 16)
              ? 0
              : prng->NextRange(16, shared->effective_max_heap_bytes);
      void* const ptr = Kmalloc(byte_size);
      if (ptr == nullptr || (reinterpret_cast<uintptr_t>(ptr) &
                             static_cast<uintptr_t>(kHeapAlignment - 1)) != 0) {
        shared->payload_corruptions.fetch_add(1, std::memory_order_acq_rel);
        shared->invariant_failed.store(true, std::memory_order_release);
        if (ptr != nullptr) {
          Kfree(ptr);
        }
        return;
      }
      const int64_t effective_bytes =
          (byte_size > 0) ? byte_size : kHeapAlignment;
      FillByteBufferPattern(static_cast<uint8_t*>(ptr),  //
                            effective_bytes,             //
                            pattern_seed);
      slot->kind = StressAllocKind::kKmalloc;
      slot->heap_ptr = ptr;
      slot->byte_size = byte_size;
      slot->alignment = kHeapAlignment;
      slot->pattern_seed = pattern_seed;
      shared->kmalloc_ops.fetch_add(1, std::memory_order_acq_rel);
      break;
    }
    case 1: {
      const int64_t byte_size =
          prng->NextRange(16, shared->effective_max_heap_bytes);
      const int64_t alignment = prng->NextPowerOfTwoAlignment(1, kPageSize);
      void* const ptr = KmallocAligned(byte_size, alignment);
      const uintptr_t eff_align = static_cast<uintptr_t>(
          (alignment > kHeapAlignment) ? alignment : kHeapAlignment);
      if (ptr == nullptr ||
          (reinterpret_cast<uintptr_t>(ptr) & (eff_align - 1u)) != 0) {
        shared->payload_corruptions.fetch_add(1, std::memory_order_acq_rel);
        shared->invariant_failed.store(true, std::memory_order_release);
        if (ptr != nullptr) {
          Kfree(ptr);
        }
        return;
      }
      FillByteBufferPattern(static_cast<uint8_t*>(ptr),  //
                            byte_size,                   //
                            pattern_seed);
      slot->kind = StressAllocKind::kKmallocAligned;
      slot->heap_ptr = ptr;
      slot->byte_size = byte_size;
      slot->alignment = alignment;
      slot->pattern_seed = pattern_seed;
      shared->kmalloc_aligned_ops.fetch_add(1, std::memory_order_acq_rel);
      break;
    }
    case 2: {
      StressHeapObject* const obj = new StressHeapObject();
      if (obj == nullptr) {
        shared->payload_corruptions.fetch_add(1, std::memory_order_acq_rel);
        shared->invariant_failed.store(true, std::memory_order_release);
        return;
      }
      obj->magic = kHeapObjectMagic;
      obj->seed_tag = pattern_seed;
      obj->worker_id = worker_id;
      obj->round_created = round;
      uint64_t checksum = pattern_seed ^ kHeapObjectMagic ^
                          static_cast<uint64_t>(worker_id) ^
                          (static_cast<uint64_t>(round) << 32);
      for (int i = 0; i < 16; ++i) {
        const uint64_t word = ComputePatternWord(pattern_seed, i);
        obj->payload[i] = word;
        checksum = MixEntropy64(checksum, word);
      }
      obj->checksum = checksum;
      slot->kind = StressAllocKind::kCppNewDelete;
      slot->obj_ptr = obj;
      slot->byte_size = static_cast<int64_t>(sizeof(StressHeapObject));
      slot->pattern_seed = pattern_seed;
      shared->cpp_new_delete_ops.fetch_add(1, std::memory_order_acq_rel);
      break;
    }
    case 3: {
      const bool single_frame = prng->NextBool();
      const int64_t frame_count =
          single_frame ? 1 : prng->NextRange(1, shared->max_pmm_alloc_frames);
      const uintptr_t phys_addr =
          single_frame ? PmmAllocFrame() : PmmAllocFrames(frame_count);
      if (phys_addr == 0 ||
          (phys_addr & static_cast<uintptr_t>(kPageSize - 1)) != 0) {
        shared->payload_corruptions.fetch_add(1, std::memory_order_acq_rel);
        shared->invariant_failed.store(true, std::memory_order_release);
        if (phys_addr != 0) {
          PmmFreeFrames(phys_addr, frame_count);
        }
        return;
      }
      FillFrameRunPattern(phys_addr, frame_count, pattern_seed);
      slot->kind = StressAllocKind::kPmmFrames;
      slot->pmm_phys_addr = phys_addr;
      slot->frame_count = frame_count;
      slot->single_frame_api = single_frame;
      slot->pattern_seed = pattern_seed;
      shared->pmm_alloc_ops.fetch_add(1, std::memory_order_acq_rel);
      break;
    }
  }
}

// Records the calling task's CPU in the participation mask and counts a
// migration if it differs from the CPU that this task saw at its previous
// checkpoint, `*last_cpu` (-1 before the task's first checkpoint).
static void RecordCpuAndMigration(StressSharedState* const shared,
                                  int* const last_cpu) {
  DCHECK(shared != nullptr);
  DCHECK(last_cpu != nullptr);
  const int cpu = CurrentCpuId();
  shared->cpu_participation.Add(cpu);
  if (*last_cpu >= 0 && cpu != *last_cpu) {
    shared->task_migrations_observed.fetch_add(1, std::memory_order_acq_rel);
  }
  *last_cpu = cpu;
}

static void RecordChildWeight(StressSharedState* const shared,
                              const int64_t weight) {
  DCHECK(shared != nullptr);
  int64_t min_weight =
      shared->min_child_task_weight.load(std::memory_order_relaxed);
  while (weight < min_weight &&
         !shared->min_child_task_weight.compare_exchange_weak(
             min_weight, weight, std::memory_order_relaxed)) {
  }
  int64_t max_weight =
      shared->max_child_task_weight.load(std::memory_order_relaxed);
  while (weight > max_weight &&
         !shared->max_child_task_weight.compare_exchange_weak(
             max_weight, weight, std::memory_order_relaxed)) {
  }
}

// Steal-barrier task of VerifyStealingAndPreemption(): records the CPU it
// runs on and spins, never yielding, until it is released.
static void StealBarrierEntry(void* const raw_arg) {
  StealBarrierContext* const ctx = static_cast<StealBarrierContext*>(raw_arg);
  DCHECK(ctx != nullptr);
  DCHECK(ctx->shared != nullptr);
  int cpu = -1;
  RecordCpuAndMigration(ctx->shared, &cpu);
  if (cpu == 0) {
    ctx->ran_on_cpu0.fetch_add(1, std::memory_order_acq_rel);
  } else {
    // Created on CPU 0, so another CPU stole this task.
    ctx->shared->cross_cpu_steals_verified.fetch_add(1,
                                                     std::memory_order_acq_rel);
  }
  ctx->running_cpus.Add(cpu);
  while (!ctx->release.load(std::memory_order_acquire)) {
    CpuRelax();
  }
}

// Spinner of RunPreemptionProbe(): records the CPU it starts on and spins,
// never yielding, until it is released or times out.
static void PreemptProbeSpinnerEntry(void* const raw_arg) {
  PreemptProbeContext* const probe = static_cast<PreemptProbeContext*>(raw_arg);
  DCHECK(probe != nullptr);
  probe->start_cpu.store(CurrentCpuId(), std::memory_order_release);
  const TscDeadline deadline(probe->timeout_tsc_cycles);
  while (!probe->release.load(std::memory_order_acquire) &&
         !deadline.Expired()) {
    NonYieldingSpinPoint();
    asm volatile("pause" : : : "memory");
  }
  probe->finished.store(true, std::memory_order_release);
}

static void JoinedChildEntry(void* const raw_arg) {
  JoinedChildContext* const arg = static_cast<JoinedChildContext*>(raw_arg);
  if (arg == nullptr || arg->shared == nullptr) {
    return;
  }
  StressSharedState* const shared = arg->shared;
  TaskScheduler* const scheduler = GetTaskScheduler();
  Task* const self = scheduler->CurrentTask();

  alignas(16) volatile uint64_t child_canaries[4] = {
      arg->expected_token ^ 0x1111111111111111ULL,
      arg->expected_token ^ 0x2222222222222222ULL,
      arg->expected_token ^ 0x4444444444444444ULL,
      arg->expected_token ^ 0x8888888888888888ULL,
  };
  const uintptr_t canary_addr =
      reinterpret_cast<uintptr_t>(const_cast<uint64_t*>(&child_canaries[0]));
  if (self == nullptr || self == shared->bootstrap_task ||
      self == arg->parent_task || (canary_addr & 0xFu) != 0 ||
      !AreInterruptsEnabled()) {
    shared->stack_canary_corruptions.fetch_add(1, std::memory_order_acq_rel);
    shared->invariant_failed.store(true, std::memory_order_release);
    return;
  }

  int last_cpu = -1;
  RecordCpuAndMigration(shared, &last_cpu);

  if (arg->allow_yield) {
    scheduler->Yield();
    shared->cooperative_yields.fetch_add(1, std::memory_order_acq_rel);
    RecordCpuAndMigration(shared, &last_cpu);
    if (scheduler->CurrentTask() != self || !AreInterruptsEnabled()) {
      shared->invariant_failed.store(true, std::memory_order_release);
      return;
    }
  }

  if (child_canaries[0] != (arg->expected_token ^ 0x1111111111111111ULL) ||
      child_canaries[1] != (arg->expected_token ^ 0x2222222222222222ULL) ||
      child_canaries[2] != (arg->expected_token ^ 0x4444444444444444ULL) ||
      child_canaries[3] != (arg->expected_token ^ 0x8888888888888888ULL)) {
    shared->stack_canary_corruptions.fetch_add(1, std::memory_order_acq_rel);
    shared->invariant_failed.store(true, std::memory_order_release);
    return;
  }

  arg->observed_token.store(arg->expected_token, std::memory_order_release);
}

static void DetachedChildEntry(void* const raw_arg) {
  std::unique_ptr<DetachedChildContext> arg(
      static_cast<DetachedChildContext*>(raw_arg));
  if (arg == nullptr || arg->shared == nullptr) {
    return;
  }
  StressSharedState* const shared = arg->shared;
  TaskScheduler* const scheduler = GetTaskScheduler();
  Task* const self = scheduler->CurrentTask();

  const uint64_t seed = arg->seed;
  const bool allow_yield = arg->allow_yield;
  arg.reset();

  alignas(16) volatile uint64_t child_canaries[4] = {
      seed ^ 0x13579BDF2468ACE0ULL,
      seed ^ 0x2468ACE013579BDFULL,
      seed ^ 0xFEDCBA9876543210ULL,
      seed ^ 0x0123456789ABCDEFULL,
  };
  const uintptr_t canary_addr =
      reinterpret_cast<uintptr_t>(const_cast<uint64_t*>(&child_canaries[0]));
  if (self == nullptr || self == shared->bootstrap_task ||
      (canary_addr & 0xFu) != 0 || !AreInterruptsEnabled()) {
    shared->stack_canary_corruptions.fetch_add(1, std::memory_order_acq_rel);
    shared->invariant_failed.store(true, std::memory_order_release);
    shared->detached_children_finished.fetch_add(1, std::memory_order_acq_rel);
    return;
  }

  int last_cpu = -1;
  RecordCpuAndMigration(shared, &last_cpu);

  if (allow_yield) {
    scheduler->Yield();
    shared->cooperative_yields.fetch_add(1, std::memory_order_acq_rel);
    RecordCpuAndMigration(shared, &last_cpu);
    if (scheduler->CurrentTask() != self || !AreInterruptsEnabled()) {
      shared->invariant_failed.store(true, std::memory_order_release);
    }
  }

  if (child_canaries[0] != (seed ^ 0x13579BDF2468ACE0ULL) ||
      child_canaries[1] != (seed ^ 0x2468ACE013579BDFULL) ||
      child_canaries[2] != (seed ^ 0xFEDCBA9876543210ULL) ||
      child_canaries[3] != (seed ^ 0x0123456789ABCDEFULL)) {
    shared->stack_canary_corruptions.fetch_add(1, std::memory_order_acq_rel);
    shared->invariant_failed.store(true, std::memory_order_release);
  }

  shared->detached_children_finished.fetch_add(1, std::memory_order_acq_rel);
}

static int64_t SelectRandomChildWeight(Prng64* const prng,
                                       const int child_seq) {
  DCHECK(prng != nullptr);
  // Explicitly exercise exact boundary weights kMinTaskWeight and
  // kMaxTaskWeight as well as random interior weights across
  // [kMinTaskWeight, kMaxTaskWeight].
  if (child_seq == 0) {
    return kMinTaskWeight;
  }
  if (child_seq == 1) {
    return kMaxTaskWeight;
  }
  if (prng->NextBool()) {
    return prng->NextRange(kDefaultTaskWeight / 4, kDefaultTaskWeight * 4);
  }
  return prng->NextRange(kMinTaskWeight, kMaxTaskWeight);
}

static void RunNonYieldingSpinBurst(StressSharedState* const shared,  //
                                    Prng64* const prng,               //
                                    int* const last_cpu) {
  DCHECK(shared != nullptr);
  DCHECK(prng != nullptr);
  DCHECK(last_cpu != nullptr);

  shared->preemptive_spin_bursts.fetch_add(1, std::memory_order_acq_rel);
  const CpuLocal* const cpu_before_local = CurrentCpu();
  const int cpu_before = cpu_before_local->cpu_id;
  const int64_t ticks_before =
      cpu_before_local->timer_ticks.load(std::memory_order_relaxed);
  const int spin_iters = prng->NextIntRange(128, 640);

  uint64_t accumulator = prng->NextU64();
  for (int s = 0; s < spin_iters; ++s) {
    accumulator = MixEntropy64(accumulator, static_cast<uint64_t>(s));
    asm volatile("pause" : "+r"(accumulator) : : "memory");
    NonYieldingSpinPoint();
  }

  RecordCpuAndMigration(shared, last_cpu);
  const CpuLocal* const cpu_after_local = CurrentCpu();
  const int cpu_after = cpu_after_local->cpu_id;
  if (cpu_after != cpu_before) {
    // The burst never yields, so only an interrupt can have moved it.
    shared->timer_preemptions_observed.fetch_add(1, std::memory_order_acq_rel);
    return;
  }
  const int64_t ticks_after =
      cpu_after_local->timer_ticks.load(std::memory_order_relaxed);
  if (ticks_after > ticks_before) {
    shared->timer_ticks_during_spin.fetch_add(ticks_after - ticks_before,
                                              std::memory_order_acq_rel);
  }
}

static void StressWorkerEntry(void* const raw_arg) {
  StressWorkerContext* const worker =
      static_cast<StressWorkerContext*>(raw_arg);
  if (worker == nullptr || worker->shared == nullptr) {
    return;
  }
  StressSharedState* const shared = worker->shared;
  TaskScheduler* const scheduler = GetTaskScheduler();
  Task* const self = scheduler->CurrentTask();

  alignas(16) volatile uint64_t stack_canaries[8] = {};
  const uintptr_t canary_addr =
      reinterpret_cast<uintptr_t>(const_cast<uint64_t*>(&stack_canaries[0]));
  if (self == nullptr || self == shared->bootstrap_task ||
      (canary_addr & 0xFu) != 0 || !AreInterruptsEnabled()) {
    shared->stack_canary_corruptions.fetch_add(1, std::memory_order_acq_rel);
    shared->invariant_failed.store(true, std::memory_order_release);
    return;
  }

  Prng64 prng(worker->worker_seed);
  int last_cpu = -1;
  RecordCpuAndMigration(shared, &last_cpu);

  int alloc_seq = 0;
  int child_seq = 0;
  bool spawned_joined_child = false;
  bool spawned_detached_child = false;

  for (int round = 0; round < shared->rounds_per_worker; ++round) {
    const uint64_t round_tag =
        MixEntropy64(worker->worker_seed, static_cast<uint64_t>(round + 1));
    for (int k = 0; k < 8; ++k) {
      stack_canaries[k] =
          round_tag ^ (static_cast<uint64_t>(k + 1) * 0x1111111111111111ULL);
    }

    RecordCpuAndMigration(shared, &last_cpu);
    if (scheduler->CurrentTask() != self || !AreInterruptsEnabled()) {
      shared->invariant_failed.store(true, std::memory_order_release);
      break;
    }

    // 1. Randomized PMM & Heap allocation / verification across slots.
    const int initial_allocs =
        (worker->worker_index == 0 && round == 0 && shared->num_workers < 4 &&
         shared->rounds_per_worker < 4)
            ? 4
            : 1;
    for (int a = 0; a < initial_allocs; ++a) {
      const int slot_idx =
          (alloc_seq < 4)
              ? (alloc_seq % shared->max_live_allocs_per_worker)
              : prng.NextIntRange(0, shared->max_live_allocs_per_worker - 1);
      LiveAllocation* const slot = &worker->live_allocs[slot_idx];
      if (slot->kind != StressAllocKind::kNone) {
        VerifyAndReleaseSlot(shared, slot);
      }
      const int kind_idx = (alloc_seq < 4)
                               ? ((worker->worker_index + alloc_seq) & 3)
                               : prng.NextIntRange(0, 3);
      ++alloc_seq;
      AllocateIntoSlot(shared,                //
                       slot,                  //
                       &prng,                 //
                       worker->worker_index,  //
                       round,                 //
                       kind_idx);
    }

    // 2. Cross-CPU wakeup/reschedule IPI (0x21).
    if (shared->cpu_count > 1 && (round == 0 || prng.NextChance(1, 2))) {
      const int cur_cpu = CurrentCpuId();
      const int offset = prng.NextIntRange(1, shared->cpu_count - 1);
      const int target_cpu = (cur_cpu + offset) % shared->cpu_count;
      SmpSendIpi(target_cpu, kVectorWakeupIpi);
      shared->wakeup_ipis_sent.fetch_add(1, std::memory_order_acq_rel);
    }

    // 3. Randomly alternate cooperative Yield() and non-yielding CPU-bound
    //    spin bursts while holding live PMM/Heap allocations across switches.
    if (round == 0 || prng.NextBool()) {
      scheduler->Yield();
      shared->cooperative_yields.fetch_add(1, std::memory_order_acq_rel);
      RecordCpuAndMigration(shared, &last_cpu);
    }
    if (round == 0 || prng.NextChance(1, 2)) {
      RunNonYieldingSpinBurst(shared, &prng, &last_cpu);
    }

    if (scheduler->CurrentTask() != self || !AreInterruptsEnabled()) {
      shared->invariant_failed.store(true, std::memory_order_release);
      break;
    }

    // 4. Dynamic child task spawning (both joined and detached, using both
    //    CreateTask and CreateTaskOnCpu with randomized weights in
    //    [kMinTaskWeight, kMaxTaskWeight]).
    const bool do_joined = !spawned_joined_child || prng.NextChance(1, 4);
    if (do_joined) {
      JoinedChildContext child_ctx = {};
      child_ctx.shared = shared;
      child_ctx.parent_task = self;
      child_ctx.expected_token = round_tag ^ 0xA5A55A5AA5A55A5AULL;
      const int64_t child_weight = SelectRandomChildWeight(&prng, child_seq++);
      RecordChildWeight(shared, child_weight);
      child_ctx.allow_yield = (child_weight >= 128);

      Task* child = nullptr;
      if (prng.NextBool()) {
        const int target_cpu = prng.NextIntRange(0, shared->cpu_count - 1);
        child = scheduler->CreateTaskOnCpu(JoinedChildEntry,  //
                                           &child_ctx,        //
                                           child_weight,      //
                                           target_cpu);
      } else {
        child =
            scheduler->CreateTask(JoinedChildEntry, &child_ctx, child_weight);
      }
      if (child == nullptr) {
        shared->invariant_failed.store(true, std::memory_order_release);
        break;
      }
      scheduler->Join(child);
      spawned_joined_child = true;
      shared->child_tasks_joined.fetch_add(1, std::memory_order_acq_rel);
      RecordCpuAndMigration(shared, &last_cpu);
      if (child_ctx.observed_token.load(std::memory_order_acquire) !=
              child_ctx.expected_token ||
          scheduler->CurrentTask() != self || !AreInterruptsEnabled()) {
        shared->invariant_failed.store(true, std::memory_order_release);
        break;
      }
    }

    const bool do_detached =
        (!spawned_detached_child || prng.NextChance(1, 4)) &&
        (shared->child_tasks_detached.load(std::memory_order_acquire) <
         kMaxDetachedChildrenPerRun);
    if (do_detached) {
      const int64_t detached_idx =
          shared->child_tasks_detached.fetch_add(1, std::memory_order_acq_rel);
      if (detached_idx < kMaxDetachedChildrenPerRun) {
        std::unique_ptr<DetachedChildContext> det_ctx(
            new DetachedChildContext());
        if (det_ctx != nullptr) {
          det_ctx->shared = shared;
          det_ctx->seed = round_tag ^ prng.NextU64();
          const int64_t det_weight =
              SelectRandomChildWeight(&prng, child_seq++);
          RecordChildWeight(shared, det_weight);
          det_ctx->allow_yield = (det_weight >= 128);

          Task* det_task = nullptr;
          if (prng.NextBool()) {
            const int target_cpu = prng.NextIntRange(0, shared->cpu_count - 1);
            det_task = scheduler->CreateTaskOnCpu(
                DetachedChildEntry, det_ctx.get(), det_weight, target_cpu);
          } else {
            det_task = scheduler->CreateTask(DetachedChildEntry,  //
                                             det_ctx.get(),       //
                                             det_weight);
          }
          if (det_task == nullptr) {
            shared->child_tasks_detached.fetch_sub(1,
                                                   std::memory_order_acq_rel);
            shared->invariant_failed.store(true, std::memory_order_release);
            break;
          }
          scheduler->Detach(det_task);
          (void)det_ctx.release();
          spawned_detached_child = true;
        } else {
          shared->child_tasks_detached.fetch_sub(1, std::memory_order_acq_rel);
        }
      } else {
        shared->child_tasks_detached.fetch_sub(1, std::memory_order_acq_rel);
      }
    }

    // 5. Verify per-task stack canaries survived all switches and migrations.
    for (int k = 0; k < 8; ++k) {
      const uint64_t expected =
          round_tag ^ (static_cast<uint64_t>(k + 1) * 0x1111111111111111ULL);
      if (stack_canaries[k] != expected) {
        shared->stack_canary_corruptions.fetch_add(1,
                                                   std::memory_order_acq_rel);
        shared->invariant_failed.store(true, std::memory_order_release);
        break;
      }
    }
  }

  // Verify and release all remaining live allocations held by this worker.
  for (int i = 0; i < shared->max_live_allocs_per_worker; ++i) {
    if (worker->live_allocs[i].kind != StressAllocKind::kNone) {
      VerifyAndReleaseSlot(shared, &worker->live_allocs[i]);
    }
  }

  worker->done.store(true, std::memory_order_release);
  shared->workers_completed.fetch_add(1, std::memory_order_acq_rel);
}

// Proves that CPU 0 preempts a task that never yields. The bootstrap task
// creates a non-yielding spinner on CPU 0 and yields to it. The bootstrap task
// can't be stolen by other CPUs, so once it runs again, CPU 0 must have
// switched away from the spinner, which only an interrupt can do. That proves
// a preemption if the spinner started on CPU 0 and is still spinning. If
// another CPU stole the spinner before it ran on CPU 0, the attempt proves
// nothing and is repeated.
static void RunPreemptionProbe(TaskScheduler& scheduler,
                               StressSharedState* const shared) {
  DCHECK(shared != nullptr);
  const uint64_t seed = shared->master_seed;
  for (int attempt = 0; attempt < kMaxPreemptProbeAttempts; ++attempt) {
    PreemptProbeContext probe;
    probe.timeout_tsc_cycles = shared->wait_timeout_tsc_cycles;
    Task* const spinner =
        scheduler.CreateTaskOnCpu(PreemptProbeSpinnerEntry,  //
                                  &probe,                    //
                                  kDefaultTaskWeight,        //
                                  0);
    STRESS_CHECK(seed, spinner != nullptr);
    scheduler.Yield();
    const bool spinner_ran_on_cpu0 =
        probe.start_cpu.load(std::memory_order_acquire) == 0;
    const bool spinner_finished =
        probe.finished.load(std::memory_order_acquire);
    probe.release.store(true, std::memory_order_release);
    scheduler.Join(spinner);
    if (spinner_ran_on_cpu0) {
      // A spinner that ran on CPU 0 until it timed out was never preempted.
      STRESS_CHECK(seed, !spinner_finished);
      shared->timer_preemptions_observed.fetch_add(1,
                                                   std::memory_order_acq_rel);
      return;
    }
  }
  FailStressRun("other CPUs stole every preemption probe spinner", seed,
                __FILE__, __LINE__);
}

// Verifies work stealing and timer preemption with tasks created by the
// bootstrap task on CPU 0 once the randomized phase is over (it exercises both
// but can't guarantee either).
//
// On SMP, `cpu_count - 1` non-yielding barrier tasks are enqueued on CPU 0
// while preemption is disabled. CPU 0 keeps running the bootstrap task, which
// doesn't yield until all of them run, and with preemption disabled no CPU
// preempts or steals from an interrupt handler. So the barrier tasks can only
// run if idle CPUs steal them, and a CPU that runs one keeps it until it is
// released: every other CPU must steal exactly one. While they hold the other
// CPUs, preemption is enabled again for RunPreemptionProbe(). A timer tick or
// wakeup IPI on a CPU busy with a barrier task does not steal the probe's
// spinner: CPU 0, with the bootstrap task and the spinner, is at most one task
// busier (see work_stealing_scheduler.h).
static void VerifyStealingAndPreemption(TaskScheduler& scheduler,
                                        StressSharedState* const shared) {
  DCHECK(shared != nullptr);
  const uint64_t seed = shared->master_seed;
  const int cpu_count = shared->cpu_count;
  if (cpu_count == 1) {
    RunPreemptionProbe(scheduler, shared);
    return;
  }

  StealBarrierContext barrier;
  barrier.shared = shared;
  const std::unique_ptr<Task*[]> barrier_tasks =
      std::make_unique<Task*[]>(cpu_count);
  scheduler.SetPreemptEnabled(false);
  for (int i = 1; i < cpu_count; ++i) {
    barrier_tasks[i] = scheduler.CreateTaskOnCpu(StealBarrierEntry,   //
                                                 &barrier,            //
                                                 kDefaultTaskWeight,  //
                                                 0);
    STRESS_CHECK(seed, barrier_tasks[i] != nullptr);
  }

  CpuMask128 other_cpus = CpuMask128::FirstCpus(cpu_count);
  other_cpus.low &= ~1ULL;
  const TscDeadline deadline(shared->wait_timeout_tsc_cycles);
  for (int spin = 0; !barrier.running_cpus.Load().ContainsAll(other_cpus);
       ++spin) {
    if ((spin & 255) == 0) {
      // Wake the CPUs that haven't stolen a barrier task yet.
      const CpuMask128 running = barrier.running_cpus.Load();
      for (int c = 1; c < cpu_count; ++c) {
        const uint64_t word = (c < 64) ? running.low : running.high;
        if ((word & (1ULL << static_cast<uint32_t>(c & 63))) == 0) {
          SmpSendIpi(c, kVectorWakeupIpi);
          shared->wakeup_ipis_sent.fetch_add(1, std::memory_order_acq_rel);
        }
      }
    }
    if (deadline.Expired()) {
      FailStressRun("timed out waiting for every CPU to steal a barrier task",
                    seed, __FILE__, __LINE__);
    }
    CpuRelax();
  }
  STRESS_CHECK(seed, barrier.ran_on_cpu0.load(std::memory_order_acquire) == 0);
  scheduler.SetPreemptEnabled(true);

  RunPreemptionProbe(scheduler, shared);

  barrier.release.store(true, std::memory_order_release);
  for (int i = 1; i < cpu_count; ++i) {
    scheduler.Join(barrier_tasks[i]);
  }
}

}  // namespace

void SetStressTestHooksForTest(const StressTestHooks& hooks) {
  g_hooks = hooks;
}

void SetLapicTimerInitialCountOnAllCpus(const uint32_t initial_count) {
  uint32_t count = initial_count;
  SmpRunOnAllCpus(SetLapicTimerInitialCountOnCpu, &count);
}

ScopedLapicTimerBoost::ScopedLapicTimerBoost(const bool enable)
    : calibrated_count_(SmpCalibratedTimerInitialCount()) {
  if (enable && calibrated_count_ > 32u) {
    SetLapicTimerInitialCountOnAllCpus(calibrated_count_ / 16u);
    boosted_ = true;
  }
}

void ScopedLapicTimerBoost::Restore() {
  if (boosted_) {
    SetLapicTimerInitialCountOnAllCpus(calibrated_count_);
    boosted_ = false;
  }
}

uint64_t ResolveStressTestSeed(const uint64_t requested_seed) {
  if (requested_seed != 0) {
    return requested_seed;
  }
  return HarvestHardwareEntropySeed(
      static_cast<uint64_t>(PmmFreeFrameCount()) ^
      (static_cast<uint64_t>(HeapTotalFreeBytes()) << 16));
}

StressTestStats ExecuteRandomMultiCpuStress(const StressTestConfig& config) {
  CHECK(config.num_workers >= 0 && config.num_workers <= kMaxStressWorkers);
  CHECK(config.rounds_per_worker >= 1 &&
        config.rounds_per_worker <= kMaxStressRoundsPerWorker);
  CHECK(config.max_live_allocs_per_worker >= 1 &&
        config.max_live_allocs_per_worker <= kMaxStressLiveAllocsPerWorker);
  CHECK(config.max_heap_alloc_bytes >= kMinStressHeapAllocBytes &&
        config.max_heap_alloc_bytes <= kMaxStressHeapAllocBytes);
  CHECK(config.max_pmm_alloc_frames >= 1 &&
        config.max_pmm_alloc_frames <= kMaxStressPmmAllocFrames);
  CHECK(config.wait_timeout_tsc_cycles > 0);

  TaskScheduler* const scheduler = GetTaskScheduler();
  CHECK(scheduler != nullptr);
  const int cpu_count = SmpCpuCount();
  CHECK(cpu_count >= 1 && cpu_count <= kMaxSupportedCpus);
  CHECK(SmpOnlineCpuCount() == cpu_count);

  Task* const bootstrap_task = scheduler->CurrentTask();
  CHECK(bootstrap_task != nullptr);
  CHECK(CurrentCpuId() == 0);
  CHECK(scheduler->IsPreemptEnabled());
  CHECK(AreInterruptsEnabled());

  while (scheduler->ReapZombies() > 0) {
  }

  const int64_t pmm_free_before = PmmFreeFrameCount();
  const int64_t heap_free_before = HeapTotalFreeBytes();
  CHECK(pmm_free_before > 0);
  CHECK(heap_free_before > 0);

  const uint64_t master_seed = ResolveStressTestSeed(config.seed);
  CHECK(master_seed != 0);

  if (config.log_to_console) {
    UartWrite("[STRESS] seed=");
    UartWriteHex(master_seed);
    UartWrite("\n");
  }

  ScopedLapicTimerBoost timer_boost(config.boost_lapic_timer);

  const int num_workers =
      (config.num_workers > 0)
          ? config.num_workers
          : ((2 * cpu_count <= kMaxStressWorkers) ? (2 * cpu_count)
                                                  : kMaxStressWorkers);
  CHECK(num_workers >= 1 && num_workers <= kMaxStressWorkers);

  // Cap the size of random heap allocations so that the payload of all live
  // worker allocations together stays within kLiveHeapBudgetBytes, far below
  // the free heap, and the run normally never has to expand the heap (which
  // would fail the heap_free_after == heap_free_before check). This is a
  // heuristic: it ignores task control blocks, join records, child contexts,
  // and KmallocAligned padding.
  const int total_live_slots = num_workers * config.max_live_allocs_per_worker;
  const int slot_budget = kLiveHeapBudgetBytes / total_live_slots;
  int effective_max_heap_bytes = config.max_heap_alloc_bytes;
  if (effective_max_heap_bytes > slot_budget) {
    effective_max_heap_bytes = slot_budget;
  }
  if (effective_max_heap_bytes < kMinStressHeapAllocBytes) {
    effective_max_heap_bytes = kMinStressHeapAllocBytes;
  }

  StressSharedState shared = {};
  shared.bootstrap_task = bootstrap_task;
  shared.cpu_count = cpu_count;
  shared.num_workers = num_workers;
  shared.rounds_per_worker = config.rounds_per_worker;
  shared.max_live_allocs_per_worker = config.max_live_allocs_per_worker;
  shared.effective_max_heap_bytes = effective_max_heap_bytes;
  shared.max_pmm_alloc_frames = config.max_pmm_alloc_frames;
  shared.master_seed = master_seed;
  shared.wait_timeout_tsc_cycles = config.wait_timeout_tsc_cycles;
  shared.cpu_participation.Add(0);

  std::unique_ptr<StressWorkerContext[]> workers =
      std::make_unique<StressWorkerContext[]>(num_workers);

  Prng64 master_prng(master_seed);

  // Enqueue the initial half of the workers on CPU 0 while preemption is
  // briefly disabled so remote CPUs (1 .. cpu_count - 1) immediately steal work
  // across CPUs, and place the remaining workers via both CreateTask and
  // CreateTaskOnCpu across all online CPUs.
  scheduler->SetPreemptEnabled(false);
  for (int i = 0; i < num_workers; ++i) {
    workers[i].shared = &shared;
    workers[i].worker_index = i;
    workers[i].worker_seed =
        master_prng.Fork(static_cast<uint64_t>(i + 1)).NextU64();
    const int64_t worker_weight =
        master_prng.NextRange(kDefaultTaskWeight / 2, kDefaultTaskWeight * 2);

    if (cpu_count > 1 && i < (num_workers / 2)) {
      workers[i].task = scheduler->CreateTaskOnCpu(
          StressWorkerEntry, &workers[i], worker_weight, 0);
    } else if ((i & 1) == 0) {
      workers[i].task = scheduler->CreateTaskOnCpu(
          StressWorkerEntry, &workers[i], worker_weight, i % cpu_count);
    } else {
      workers[i].task =
          scheduler->CreateTask(StressWorkerEntry, &workers[i], worker_weight);
    }
    STRESS_CHECK(master_seed, workers[i].task != nullptr);
  }
  scheduler->SetPreemptEnabled(true);

  for (int c = 1; c < cpu_count; ++c) {
    SmpSendIpi(c, kVectorWakeupIpi);
    shared.wakeup_ipis_sent.fetch_add(1, std::memory_order_acq_rel);
  }

  for (int i = 0; i < num_workers; ++i) {
    scheduler->Join(workers[i].task);
  }

  // Wait until every detached child has finished and has been reaped.
  {
    const TscDeadline deadline(config.wait_timeout_tsc_cycles);
    while (shared.zombies_reaped.load(std::memory_order_acquire) <
           shared.child_tasks_detached.load(std::memory_order_acquire)) {
      const int reaped = scheduler->ReapZombies();
      if (reaped > 0) {
        shared.zombies_reaped.fetch_add(reaped, std::memory_order_acq_rel);
        continue;
      }
      if (deadline.Expired()) {
        FailStressRun("timed out waiting to reap the detached children",
                      master_seed, __FILE__, __LINE__);
      }
      scheduler->Yield();
      CpuRelax();
    }
  }

  VerifyStealingAndPreemption(*scheduler, &shared);

  for (int i = 0; i < num_workers; ++i) {
    STRESS_CHECK(master_seed, workers[i].done.load(std::memory_order_acquire));
  }
  workers.reset();
  timer_boost.Restore();

  const int64_t pmm_free_after = PmmFreeFrameCount();
  const int64_t heap_free_after = HeapTotalFreeBytes();

  StressTestStats stats = {};
  stats.seed = master_seed;
  stats.cpu_count = cpu_count;
  const CpuMask128 participation = shared.cpu_participation.Load();
  stats.cpu_participation_mask = participation.low;
  stats.cpu_participation_mask_high = participation.high;
  stats.cooperative_yields =
      shared.cooperative_yields.load(std::memory_order_acquire);
  stats.preemptive_spin_bursts =
      shared.preemptive_spin_bursts.load(std::memory_order_acquire);
  stats.timer_ticks_during_spin =
      shared.timer_ticks_during_spin.load(std::memory_order_acquire);
  stats.timer_preemptions_observed =
      shared.timer_preemptions_observed.load(std::memory_order_acquire);
  stats.wakeup_ipis_sent =
      shared.wakeup_ipis_sent.load(std::memory_order_acquire);
  stats.task_migrations_observed =
      shared.task_migrations_observed.load(std::memory_order_acquire);
  stats.cross_cpu_steals_verified =
      shared.cross_cpu_steals_verified.load(std::memory_order_acquire);
  stats.child_tasks_joined =
      shared.child_tasks_joined.load(std::memory_order_acquire);
  stats.child_tasks_detached =
      shared.child_tasks_detached.load(std::memory_order_acquire);
  stats.zombies_reaped = shared.zombies_reaped.load(std::memory_order_acquire);
  stats.min_child_task_weight =
      shared.min_child_task_weight.load(std::memory_order_acquire);
  stats.max_child_task_weight =
      shared.max_child_task_weight.load(std::memory_order_acquire);
  stats.kmalloc_ops = shared.kmalloc_ops.load(std::memory_order_acquire);
  stats.kmalloc_aligned_ops =
      shared.kmalloc_aligned_ops.load(std::memory_order_acquire);
  stats.cpp_new_delete_ops =
      shared.cpp_new_delete_ops.load(std::memory_order_acquire);
  stats.pmm_alloc_ops = shared.pmm_alloc_ops.load(std::memory_order_acquire);
  stats.payload_verifications =
      shared.payload_verifications.load(std::memory_order_acquire);
  stats.payload_corruptions =
      shared.payload_corruptions.load(std::memory_order_acquire);
  stats.stack_canary_corruptions =
      shared.stack_canary_corruptions.load(std::memory_order_acquire);
  stats.pmm_free_before = pmm_free_before;
  stats.pmm_free_after = pmm_free_after;
  stats.heap_free_before = heap_free_before;
  stats.heap_free_after = heap_free_after;

  const bool invariant_failed =
      shared.invariant_failed.load(std::memory_order_acquire);
  const int workers_completed =
      shared.workers_completed.load(std::memory_order_acquire);
  const int64_t detached_children_finished =
      shared.detached_children_finished.load(std::memory_order_acquire);
  const uint64_t seed = master_seed;
  STRESS_CHECK(seed, !invariant_failed);
  STRESS_CHECK(seed, workers_completed == num_workers);
  STRESS_CHECK(seed,
               participation.ContainsAll(CpuMask128::FirstCpus(cpu_count)));
  STRESS_CHECK(seed, stats.cooperative_yields > 0);
  STRESS_CHECK(seed, stats.preemptive_spin_bursts > 0);
  STRESS_CHECK(seed, stats.timer_preemptions_observed > 0);
  if (cpu_count > 1) {
    STRESS_CHECK(seed, stats.wakeup_ipis_sent > 0);
    STRESS_CHECK(seed, stats.cross_cpu_steals_verified == cpu_count - 1);
  } else {
    STRESS_CHECK(seed, stats.task_migrations_observed == 0);
    STRESS_CHECK(seed, stats.cross_cpu_steals_verified == 0);
  }
  STRESS_CHECK(seed, stats.child_tasks_joined > 0);
  STRESS_CHECK(seed, stats.child_tasks_detached > 0);
  STRESS_CHECK(seed, detached_children_finished == stats.child_tasks_detached);
  STRESS_CHECK(seed, stats.zombies_reaped == stats.child_tasks_detached);
  STRESS_CHECK(seed, stats.min_child_task_weight == kMinTaskWeight);
  STRESS_CHECK(seed, stats.max_child_task_weight == kMaxTaskWeight);
  STRESS_CHECK(seed, stats.kmalloc_ops > 0);
  STRESS_CHECK(seed, stats.kmalloc_aligned_ops > 0);
  STRESS_CHECK(seed, stats.cpp_new_delete_ops > 0);
  STRESS_CHECK(seed, stats.pmm_alloc_ops > 0);
  STRESS_CHECK(seed, stats.payload_verifications > 0);
  STRESS_CHECK(seed, stats.payload_corruptions == 0);
  STRESS_CHECK(seed, stats.stack_canary_corruptions == 0);
  STRESS_CHECK(seed, stats.pmm_free_after == stats.pmm_free_before);
  STRESS_CHECK(seed, stats.heap_free_after == stats.heap_free_before);
  STRESS_CHECK(seed, CurrentCpuId() == 0);
  STRESS_CHECK(seed, scheduler->CurrentTask() == bootstrap_task);
  STRESS_CHECK(seed, scheduler->IsPreemptEnabled());
  STRESS_CHECK(seed, AreInterruptsEnabled());

  if (config.log_to_console) {
    UartWrite("[STRESS] cpus=");
    UartWriteDec(static_cast<uint64_t>(stats.cpu_count));
    UartWrite(", yields=");
    UartWriteDec(static_cast<uint64_t>(stats.cooperative_yields));
    UartWrite(", preempts=");
    UartWriteDec(static_cast<uint64_t>(stats.timer_preemptions_observed));
    UartWrite(", migrations=");
    UartWriteDec(static_cast<uint64_t>(stats.task_migrations_observed));
    UartWrite(", steals=");
    UartWriteDec(static_cast<uint64_t>(stats.cross_cpu_steals_verified));
    UartWrite(", joined=");
    UartWriteDec(static_cast<uint64_t>(stats.child_tasks_joined));
    UartWrite(", reaped=");
    UartWriteDec(static_cast<uint64_t>(stats.zombies_reaped));
    UartWrite(", verifications=");
    UartWriteDec(static_cast<uint64_t>(stats.payload_verifications));
    UartWrite("\n");
    UartWrite("[STRESS] random_multicpu_stress: PASS\n");
    constexpr char kSpinnerChars[4] = {'|', '/', '-', '\\'};
    const uint64_t pass_num =
        g_stress_pass_count.fetch_add(1, std::memory_order_relaxed) + 1;
    const char spinner_str[4] = {' ',                             //
                                 kSpinnerChars[pass_num & 3ULL],  //
                                 ' ',                             //
                                 '\0'};
    VgaWrite("\r[STRESS] random_multicpu_stress: PASS #");
    VgaWriteDec(pass_num);
    VgaWrite(spinner_str);
    VgaWrite("seed=");
    VgaWriteHex(stats.seed);
    VgaWrite("   \r");
  }

  return stats;
}

void RunRandomMultiCpuStressTest(const uint64_t seed_override) {
  StressTestConfig config;
  config.seed = seed_override;
  config.log_to_console = true;
  (void)ExecuteRandomMultiCpuStress(config);
}

}  // namespace protos
