#include "selftest/boot_selftest.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <new>

#include "address_space.h"
#include "check.h"
#include "console.h"
#include "cpu.h"
#include "format.h"
#include "fpu.h"
#include "gdt.h"
#include "heap.h"
#include "idt.h"
#include "kill_delay_injection.h"
#include "multiboot.h"
#include "paging.h"
#include "pmm.h"
#include "rbtree.h"
#include "selftest/line_buffer.h"
#include "selftest/stress.h"
#include "smp.h"
#include "spinlock.h"
#include "syscall_abi.h"
#include "task.h"
#include "task_stack.h"
#include "uart.h"
#include "user/usertest.h"
#include "user_access.h"
#include "user_mode.h"
#include "user_programs.h"
#include "vga.h"

namespace protos {

namespace {

int g_widget_ctor_count = 0;
int g_widget_dtor_count = 0;
int g_poly_base_dtor_count = 0;
int g_poly_derived_dtor_count = 0;

class TestWidget {
 public:
  TestWidget()
      : id_(g_widget_ctor_count + 1),
        checksum_((g_widget_ctor_count + 1) ^ 0x5A5A5A5A5A5A5A5A) {
    ++g_widget_ctor_count;
  }

  explicit TestWidget(const uint64_t seed)
      : id_(seed), checksum_(seed ^ 0x5A5A5A5A5A5A5A5A) {
    ++g_widget_ctor_count;
  }

  ~TestWidget() {
    if (IsValid()) {
      ++g_widget_dtor_count;
    }
    id_ = 0;
    checksum_ = 0;
  }

  bool IsValid() const { return (id_ ^ 0x5A5A5A5A5A5A5A5A) == checksum_; }

  uint64_t id() const { return id_; }

 private:
  uint64_t id_;
  uint64_t checksum_;
};

class PolymorphicBase {
 public:
  [[gnu::noinline]] explicit PolymorphicBase(const uint64_t base_tag)
      : base_tag_(base_tag) {}

  [[gnu::noinline]] virtual ~PolymorphicBase() {
    if (base_tag_ != 0) {
      ++g_poly_base_dtor_count;
    }
    base_tag_ = 0;
  }

  virtual uint64_t Compute(uint64_t input) const = 0;

  virtual uint64_t Tag() const { return base_tag_; }

 protected:
  uint64_t base_tag_;
};

class AddDevice final : public PolymorphicBase {
 public:
  AddDevice(const uint64_t base_tag, const uint64_t delta)
      : PolymorphicBase(base_tag), delta_(delta) {}

  ~AddDevice() override {
    if (delta_ != 0) {
      ++g_poly_derived_dtor_count;
    }
    delta_ = 0;
  }

  uint64_t Compute(const uint64_t input) const override {
    return input + delta_ + base_tag_;
  }

 private:
  uint64_t delta_;
};

class XorDevice final : public PolymorphicBase {
 public:
  XorDevice(const uint64_t base_tag, const uint64_t mask)
      : PolymorphicBase(base_tag), mask_(mask), extra_padding_{mask, ~mask} {}

  ~XorDevice() override {
    if (extra_padding_[0] == mask_ && extra_padding_[1] == ~mask_) {
      ++g_poly_derived_dtor_count;
    }
    mask_ = 0;
  }

  uint64_t Compute(const uint64_t input) const override {
    return (input ^ mask_) + base_tag_;
  }

  uint64_t Tag() const override { return base_tag_ ^ mask_; }

 private:
  uint64_t mask_;
  uint64_t extra_padding_[2];
};

// Prints the "[TEST] <name>: PASS|FAIL" line of a boot test over UART only.
// The suite runs after the boot log has filled about half of the 25-row VGA
// text screen, and each VGA line it added would bring the greeting on row 0
// (which test_boot.sh checks) closer to scrolling off. The VGA console only
// gets the one-line summary of RunBootSelfTests().
static void LogTestResult(const char* const test_name, const bool passed) {
  UartWrite("[TEST] ");
  UartWrite(test_name);
  UartWrite(passed ? ": PASS\n" : ": FAIL\n");
}

// Logs `expr` with its location over UART if `ok` is false; returns `ok`.
static bool CheckExpectation(const bool ok,           //
                             const char* const expr,  //
                             const char* const file,  //
                             const int line) {
  if (!ok) {
    UartWrite("[TEST] expectation failed: ");
    UartWrite(expr);
    UartWrite(" at ");
    UartWrite(file);
    UartWrite(":");
    UartWriteDec(static_cast<uint64_t>(line));
    UartWrite("\n");
  }
  return ok;
}

// Evaluates to `cond`, after logging it with its location if it is false.
#define SELFTEST_OK(cond) \
  CheckExpectation(static_cast<bool>(cond), #cond, __FILE__, __LINE__)

// Makes the enclosing boot test return false, after logging `cond` with its
// location, if `cond` is false.
#define SELFTEST_EXPECT(cond) \
  do {                        \
    if (!SELFTEST_OK(cond)) { \
      return false;           \
    }                         \
  } while (false)

// Frames that TestPmmAllocAndBounds() allocates and TestPmmFreeAndReuse()
// frees again.
struct PmmTestFrames {
  int64_t free_before_allocs = 0;
  uintptr_t first = 0;
  uintptr_t second = 0;
  uintptr_t run = 0;  // kPmmTestRunFrames contiguous frames.
};
constexpr int64_t kPmmTestRunFrames = 4;
PmmTestFrames g_pmm_test_frames;

static bool TestPmmMemoryMapInit() {
  SELFTEST_EXPECT(PmmTotalUsableFrameCount() > 0);
  SELFTEST_EXPECT(PmmFreeFrameCount() > 0);
  SELFTEST_EXPECT(PmmFreeFrameCount() <= PmmTotalUsableFrameCount());
  SELFTEST_EXPECT(PmmMaxPhysicalAddress() > kLowerMemoryLimit);
  return true;
}

// Checks that the `count` frames at `frame` are page-aligned, identity-mapped
// (also through PagingEnsureIdentityMappedRange, which must accept ranges that
// 2 MiB huge pages already cover), usable RAM.
static bool IsMappedUsableFrameRun(const uintptr_t frame, const int64_t count) {
  const uintptr_t last_page =
      frame + static_cast<uintptr_t>((count - 1) * kPageSize);
  SELFTEST_EXPECT((frame & (kPageSize - 1)) == 0);
  SELFTEST_EXPECT(PagingIsIdentityMapped(frame));
  SELFTEST_EXPECT(PagingIsIdentityMapped(last_page));
  SELFTEST_EXPECT(PagingEnsureIdentityMappedRange(frame, count * kPageSize));
  SELFTEST_EXPECT(PmmRangeIsValidUsableRam(frame, count * kPageSize));
  return true;
}

// Verifies that the identity map covers all usable RAM and that PMM
// allocation returns distinct, page-aligned, identity-mapped usable frames
// that hold data. Leaves the frames in g_pmm_test_frames.
static bool TestPmmAllocAndBounds() {
  // PmmInit extends the 64 MiB identity map of boot.S up to the highest usable
  // page. That page may be in use, so it is only read.
  const uintptr_t max_phys = PmmMaxPhysicalAddress();
  const uintptr_t top_page = max_phys - kPageSize;
  if (max_phys <= kBootstrapIdentityMapSize) {
    UartWrite(
        "[TEST] pmm_alloc_and_bounds: identity map extension check skipped "
        "(all usable RAM is below 64 MiB)\n");
  }
  SELFTEST_EXPECT(PagingIdentityMappedLimit() >= max_phys);
  SELFTEST_EXPECT(PagingIsIdentityMapped(top_page));
  SELFTEST_EXPECT(PmmRangeIsValidUsableRam(top_page, kPageSize));
  (void)*reinterpret_cast<const volatile uint64_t*>(top_page);

  PmmTestFrames& frames = g_pmm_test_frames;
  frames.free_before_allocs = PmmFreeFrameCount();
  frames.first = PmmAllocFrame();
  frames.second = PmmAllocFrame();
  frames.run = PmmAllocFrames(kPmmTestRunFrames);
  SELFTEST_EXPECT(frames.first != 0);
  SELFTEST_EXPECT(frames.second != 0);
  SELFTEST_EXPECT(frames.run != 0);
  SELFTEST_EXPECT(frames.first != frames.second);
  SELFTEST_EXPECT(frames.first != frames.run);
  SELFTEST_EXPECT(frames.second != frames.run);
  SELFTEST_EXPECT(IsMappedUsableFrameRun(frames.first, 1));
  SELFTEST_EXPECT(IsMappedUsableFrameRun(frames.second, 1));
  SELFTEST_EXPECT(IsMappedUsableFrameRun(frames.run, kPmmTestRunFrames));

  volatile uint64_t* const first_word =
      reinterpret_cast<volatile uint64_t*>(frames.first);
  volatile uint64_t* const second_word =
      reinterpret_cast<volatile uint64_t*>(frames.second);
  volatile uint64_t* const run_last_page_word =
      reinterpret_cast<volatile uint64_t*>(frames.run +
                                           (kPmmTestRunFrames - 1) * kPageSize);
  *first_word = 0xCAFEBABE11112222;
  *second_word = 0xDEADBEEF33334444;
  *run_last_page_word = 0x0123456789ABCDEF;
  SELFTEST_EXPECT(*first_word == 0xCAFEBABE11112222);
  SELFTEST_EXPECT(*second_word == 0xDEADBEEF33334444);
  SELFTEST_EXPECT(*run_last_page_word == 0x0123456789ABCDEF);
  return true;
}

// Frees the frames of TestPmmAllocAndBounds() (whatever it got) and checks the
// free-frame accounting, that the freed frames are handed out again first, and
// that a request larger than all of RAM fails.
static bool TestPmmFreeAndReuse() {
  const PmmTestFrames frames = g_pmm_test_frames;
  g_pmm_test_frames = PmmTestFrames{};
  const int64_t free_with_allocs = PmmFreeFrameCount();
  if (frames.first != 0) {
    PmmFreeFrame(frames.first);
  }
  if (frames.second != 0) {
    PmmFreeFrame(frames.second);
  }
  if (frames.run != 0) {
    PmmFreeFrames(frames.run, kPmmTestRunFrames);
  }
  SELFTEST_EXPECT(frames.first != 0);
  SELFTEST_EXPECT(frames.second != 0);
  SELFTEST_EXPECT(frames.run != 0);
  SELFTEST_EXPECT(frames.free_before_allocs <= PmmTotalUsableFrameCount());
  SELFTEST_EXPECT(free_with_allocs + 2 + kPmmTestRunFrames ==
                  frames.free_before_allocs);
  SELFTEST_EXPECT(PmmFreeFrameCount() == frames.free_before_allocs);

  const uintptr_t reused_first = PmmAllocFrame();
  const uintptr_t reused_second = PmmAllocFrame();
  if (reused_first != 0) {
    PmmFreeFrame(reused_first);
  }
  if (reused_second != 0) {
    PmmFreeFrame(reused_second);
  }
  SELFTEST_EXPECT(reused_first == frames.first);
  SELFTEST_EXPECT(reused_second == frames.second);

  SELFTEST_EXPECT(PmmAllocFrames(PmmMaxFrameCount() + 1) == 0);
  SELFTEST_EXPECT(PmmFreeFrameCount() == frames.free_before_allocs);
  return true;
}

struct alignas(64) CacheLineAlignedWidget {
  uint64_t id;
  uint64_t checksum;

  CacheLineAlignedWidget()
      : id(g_widget_ctor_count + 1),
        checksum((g_widget_ctor_count + 1) ^ 0xA5A5A5A5A5A5A5A5ULL) {
    ++g_widget_ctor_count;
  }

  explicit CacheLineAlignedWidget(const uint64_t seed)
      : id(seed), checksum(seed ^ 0xA5A5A5A5A5A5A5A5ULL) {
    ++g_widget_ctor_count;
  }

  ~CacheLineAlignedWidget() {
    if ((id ^ 0xA5A5A5A5A5A5A5A5ULL) == checksum) {
      ++g_widget_dtor_count;
    }
    id = 0;
    checksum = 0;
  }

  bool IsValid() const { return (id ^ 0xA5A5A5A5A5A5A5A5ULL) == checksum; }
};

struct alignas(256) PageSubblockAlignedWidget {
  uint64_t tag;
  uint64_t mirror;

  explicit PageSubblockAlignedWidget(const uint64_t seed = 0xCAFE0000ULL)
      : tag(seed), mirror(~seed) {
    ++g_widget_ctor_count;
  }

  ~PageSubblockAlignedWidget() {
    if (tag == ~mirror) {
      ++g_widget_dtor_count;
    }
    tag = 0;
    mirror = 0;
  }

  bool IsValid() const { return tag == ~mirror; }
};

static bool TestHeapVariedSizesAndAlignment() {
  const int64_t free_before = HeapTotalFreeBytes();
  SELFTEST_EXPECT(free_before > 0);
  constexpr int64_t kSizes[] = {1,   7,    15,   16,    31,   64,
                                256, 1024, 4096, 16384, 65536};
  constexpr int kNumSizes = sizeof(kSizes) / sizeof(kSizes[0]);

  void* kmalloc_ptrs[kNumSizes];
  uint8_t* new_ptrs[kNumSizes];

  for (int i = 0; i < kNumSizes; ++i) {
    const int64_t req_size = kSizes[i];
    kmalloc_ptrs[i] = Kmalloc(req_size);
    new_ptrs[i] = new uint8_t[req_size];

    const uintptr_t k_addr = reinterpret_cast<uintptr_t>(kmalloc_ptrs[i]);
    const uintptr_t n_addr = reinterpret_cast<uintptr_t>(new_ptrs[i]);
    SELFTEST_EXPECT(kmalloc_ptrs[i] != nullptr);
    SELFTEST_EXPECT(new_ptrs[i] != nullptr);
    SELFTEST_EXPECT((k_addr & (kHeapAlignment - 1)) == 0);
    SELFTEST_EXPECT((n_addr & (kHeapAlignment - 1)) == 0);
    SELFTEST_EXPECT(PmmRangeIsValidUsableRam(k_addr, req_size));
    SELFTEST_EXPECT(PmmRangeIsValidUsableRam(n_addr, req_size));
  }

  for (int i = 0; i < kNumSizes; ++i) {
    Kfree(kmalloc_ptrs[i]);
    delete[] new_ptrs[i];
  }

  // Verify KmallocAligned across all power-of-two alignments in [1, kPageSize],
  // exercising both header-shifting (0 < offset < 80) and front-splitting
  // (offset >= 80).
  constexpr int64_t kAlignments[] = {1,   2,   4,   8,    16,   32,  64,
                                     128, 256, 512, 1024, 2048, 4096};
  constexpr int kNumAlignments = sizeof(kAlignments) / sizeof(kAlignments[0]);
  uint8_t* aligned_ptrs[kNumAlignments];
  for (int i = 0; i < kNumAlignments; ++i) {
    const int64_t align = kAlignments[i];
    const int64_t eff_align = (align < kHeapAlignment) ? kHeapAlignment : align;
    const int64_t req_size = 48 + i * 16;
    aligned_ptrs[i] = static_cast<uint8_t*>(KmallocAligned(req_size, align));
    const uintptr_t addr = reinterpret_cast<uintptr_t>(aligned_ptrs[i]);
    SELFTEST_EXPECT(aligned_ptrs[i] != nullptr);
    SELFTEST_EXPECT((addr & static_cast<uintptr_t>(eff_align - 1)) == 0);
    SELFTEST_EXPECT(PmmRangeIsValidUsableRam(addr, req_size));
    for (int64_t b = 0; b < req_size; ++b) {
      aligned_ptrs[i][b] = static_cast<uint8_t>((i * 31 + b) & 0xFF);
    }
  }
  for (int i = 0; i < kNumAlignments; ++i) {
    const int64_t req_size = 48 + i * 16;
    for (int64_t b = 0; b < req_size; ++b) {
      SELFTEST_EXPECT(aligned_ptrs[i][b] ==
                      static_cast<uint8_t>((i * 31 + b) & 0xFF));
    }
    Kfree(aligned_ptrs[i]);
  }

  SELFTEST_EXPECT(HeapTotalFreeBytes() == free_before);
  return true;
}

static bool TestHeapPatternIsolation() {
  constexpr int64_t kSizes[] = {24, 128, 512, 2048, 8192, 16384};
  constexpr int kCount = sizeof(kSizes) / sizeof(kSizes[0]);

  uint8_t* buffers[kCount];
  for (int i = 0; i < kCount; ++i) {
    buffers[i] = static_cast<uint8_t*>(Kmalloc(kSizes[i]));
    SELFTEST_EXPECT(buffers[i] != nullptr);
    for (int64_t b = 0; b < kSizes[i]; ++b) {
      const uint8_t pattern = ((i + 1) * 41 + b * 17) ^ 0xA5;
      buffers[i][b] = pattern;
    }
  }

  for (int i = 0; i < kCount; ++i) {
    const uintptr_t start_i = reinterpret_cast<uintptr_t>(buffers[i]);
    const uintptr_t end_i = start_i + kSizes[i];
    for (int j = i + 1; j < kCount; ++j) {
      const uintptr_t start_j = reinterpret_cast<uintptr_t>(buffers[j]);
      const uintptr_t end_j = start_j + kSizes[j];
      SELFTEST_EXPECT(start_i >= end_j || end_i <= start_j);
    }
    for (int64_t b = 0; b < kSizes[i]; ++b) {
      const uint8_t expected = ((i + 1) * 41 + b * 17) ^ 0xA5;
      SELFTEST_EXPECT(buffers[i][b] == expected);
    }
  }

  for (int i = 0; i < kCount; ++i) {
    Kfree(buffers[i]);
  }
  return true;
}

static bool TestCppNewDeleteLifecycle() {
  g_widget_ctor_count = 0;
  g_widget_dtor_count = 0;
  g_poly_base_dtor_count = 0;
  g_poly_derived_dtor_count = 0;
  const int64_t free_before = HeapTotalFreeBytes();

  {
    const std::unique_ptr<TestWidget> single =
        std::make_unique<TestWidget>(0x13572468);
    SELFTEST_EXPECT(single != nullptr);
    SELFTEST_EXPECT(single->IsValid());
    SELFTEST_EXPECT(single->id() == 0x13572468);
    SELFTEST_EXPECT(g_widget_ctor_count == 1);
    SELFTEST_EXPECT(g_widget_dtor_count == 0);
  }
  SELFTEST_EXPECT(g_widget_dtor_count == 1);

  constexpr int kArrayLen = 12;
  {
    const std::unique_ptr<TestWidget[]> array =
        std::make_unique<TestWidget[]>(kArrayLen);
    SELFTEST_EXPECT(array != nullptr);
    SELFTEST_EXPECT(g_widget_ctor_count == 1 + kArrayLen);
    for (int i = 0; i < kArrayLen; ++i) {
      SELFTEST_EXPECT(array[i].IsValid());
    }
  }
  SELFTEST_EXPECT(g_widget_dtor_count == 1 + kArrayLen);

  // Verify over-aligned C++17 `operator new(size_t, std::align_val_t)` and
  // `operator new[](size_t, std::align_val_t)` with `alignas(64)` and
  // `alignas(256)` types.
  const int base_ctor_count = g_widget_ctor_count;
  {
    const std::unique_ptr<CacheLineAlignedWidget> aligned64 =
        std::make_unique<CacheLineAlignedWidget>(0x2468ACE0ULL);
    const std::unique_ptr<PageSubblockAlignedWidget> aligned256 =
        std::make_unique<PageSubblockAlignedWidget>(0xF00DBABEULL);
    constexpr int kAlignedArrayLen = 4;
    const std::unique_ptr<CacheLineAlignedWidget[]> aligned64_arr =
        std::make_unique<CacheLineAlignedWidget[]>(kAlignedArrayLen);
    SELFTEST_EXPECT(aligned64 != nullptr);
    SELFTEST_EXPECT(aligned256 != nullptr);
    SELFTEST_EXPECT(aligned64_arr != nullptr);
    SELFTEST_EXPECT((reinterpret_cast<uintptr_t>(aligned64.get()) & 63u) == 0);
    SELFTEST_EXPECT((reinterpret_cast<uintptr_t>(aligned256.get()) & 255u) ==
                    0);
    SELFTEST_EXPECT((reinterpret_cast<uintptr_t>(aligned64_arr.get()) & 63u) ==
                    0);
    SELFTEST_EXPECT(aligned64->IsValid());
    SELFTEST_EXPECT(aligned256->IsValid());
    SELFTEST_EXPECT(g_widget_ctor_count ==
                    base_ctor_count + 2 + kAlignedArrayLen);
    for (int i = 0; i < kAlignedArrayLen; ++i) {
      SELFTEST_EXPECT(aligned64_arr[i].IsValid());
      SELFTEST_EXPECT((reinterpret_cast<uintptr_t>(&aligned64_arr[i]) & 63u) ==
                      0);
    }
  }
  SELFTEST_EXPECT(g_widget_dtor_count == g_widget_ctor_count);

  // Verify dynamic allocation, pure-virtual and overridden vtable dispatch, and
  // polymorphic deletion through smart pointers (`std::unique_ptr` and
  // `std::shared_ptr` / `std::make_unique` / `std::make_shared`).
  {
    const std::unique_ptr<PolymorphicBase> dev0 =
        std::make_unique<AddDevice>(10, 25);
    const std::shared_ptr<PolymorphicBase> dev1 =
        std::make_shared<XorDevice>(7, 0x0F000F00);
    SELFTEST_EXPECT(dev0 != nullptr);
    SELFTEST_EXPECT(dev1 != nullptr);
    SELFTEST_EXPECT(HeapTotalFreeBytes() < free_before);
    std::shared_ptr<PolymorphicBase> dev1_alias = dev1;
    SELFTEST_EXPECT(dev1.use_count() == 2);
    SELFTEST_EXPECT(dev0->Compute(100) == 135);
    SELFTEST_EXPECT(dev0->Tag() == 10);
    SELFTEST_EXPECT(dev1_alias->Compute(0x00FF00FF) == (0x0FFF0FFF + 7));
    SELFTEST_EXPECT(dev1_alias->Tag() == (7 ^ 0x0F000F00));
    dev1_alias.reset();
    SELFTEST_EXPECT(dev1.use_count() == 1);
    SELFTEST_EXPECT(g_poly_derived_dtor_count == 0);
  }
  SELFTEST_EXPECT(g_poly_derived_dtor_count == 2);
  SELFTEST_EXPECT(g_poly_base_dtor_count == 2);

  SELFTEST_EXPECT(HeapTotalFreeBytes() == free_before);
  return true;
}

static bool TestHeapStressReuse() {
  const int64_t free_before = HeapTotalFreeBytes();
  const int64_t pmm_before = PmmFreeFrameCount();

  constexpr int kIterations = 256;
  constexpr int64_t kChunkSizes[] = {4096, 8192, 16384, 32768};
  constexpr int kNumChunks = sizeof(kChunkSizes) / sizeof(kChunkSizes[0]);

  for (int iter = 0; iter < kIterations; ++iter) {
    uint8_t* chunks[kNumChunks];
    for (int c = 0; c < kNumChunks; ++c) {
      const int64_t sz = kChunkSizes[c];
      chunks[c] = static_cast<uint8_t*>(Kmalloc(sz));
      SELFTEST_EXPECT(chunks[c] != nullptr);
      const uint8_t tag = (iter + c) & 0xFF;
      chunks[c][0] = tag;
      chunks[c][sz / 2] = tag ^ 0x55;
      chunks[c][sz - 1] = tag ^ 0xAA;
    }

    for (int c = 0; c < kNumChunks; ++c) {
      const int64_t sz = kChunkSizes[c];
      const uint8_t tag = (iter + c) & 0xFF;
      SELFTEST_EXPECT(chunks[c][0] == tag);
      SELFTEST_EXPECT(chunks[c][sz / 2] == (tag ^ 0x55));
      SELFTEST_EXPECT(chunks[c][sz - 1] == (tag ^ 0xAA));
    }

    if ((iter & 1) == 0) {
      for (int c = 0; c < kNumChunks; ++c) {
        Kfree(chunks[c]);
      }
    } else {
      for (int c = kNumChunks; c > 0; --c) {
        Kfree(chunks[c - 1]);
      }
    }
  }

  SELFTEST_EXPECT(HeapTotalFreeBytes() == free_before);
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_before);

  // Two 768 KiB chunks don't fit the initial 1 MiB arena, so the first
  // iteration expands the heap (permanently); later iterations must reuse
  // that memory without growing the heap any further.
  constexpr int64_t kLargeChunkSize = 768 * 1024;
  constexpr int kLargeIterations = 16;
  int64_t free_after_first_expand = 0;
  int64_t pmm_after_first_expand = 0;

  for (int iter = 0; iter < kLargeIterations; ++iter) {
    uint8_t* const big1 = static_cast<uint8_t*>(Kmalloc(kLargeChunkSize));
    uint8_t* const big2 = static_cast<uint8_t*>(Kmalloc(kLargeChunkSize));
    SELFTEST_EXPECT(big1 != nullptr);
    SELFTEST_EXPECT(big2 != nullptr);
    const uint8_t s1 = (iter + 0x11) & 0xFF;
    const uint8_t s2 = (iter + 0x77) & 0xFF;
    big1[0] = s1;
    big1[kLargeChunkSize - 1] = s1 ^ 0xFF;
    big2[0] = s2;
    big2[kLargeChunkSize - 1] = s2 ^ 0xFF;

    SELFTEST_EXPECT(big1[0] == s1);
    SELFTEST_EXPECT(big1[kLargeChunkSize - 1] == (s1 ^ 0xFF));
    SELFTEST_EXPECT(big2[0] == s2);
    SELFTEST_EXPECT(big2[kLargeChunkSize - 1] == (s2 ^ 0xFF));

    Kfree(big1);
    Kfree(big2);

    if (iter == 0) {
      free_after_first_expand = HeapTotalFreeBytes();
      pmm_after_first_expand = PmmFreeFrameCount();
    } else {
      SELFTEST_EXPECT(HeapTotalFreeBytes() == free_after_first_expand);
      SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_after_first_expand);
    }
  }

  SELFTEST_EXPECT(free_after_first_expand >= free_before);
  return true;
}

struct KernelRbAugNode {
  RbNode hook;
  int64_t span = 0;
  int64_t max_subtree_span = 0;

  explicit KernelRbAugNode(const int64_t s) : span(s), max_subtree_span(s) {}
  KernelRbAugNode(const KernelRbAugNode&) = delete;
  KernelRbAugNode& operator=(const KernelRbAugNode&) = delete;
};

struct KernelRbAugTraits {
  static uintptr_t GetKey(const KernelRbAugNode& item) {
    return reinterpret_cast<uintptr_t>(&item);
  }
  static void UpdateAugment(KernelRbAugNode* const item,
                            const KernelRbAugNode* const left,
                            const KernelRbAugNode* const right) {
    int64_t max_val = item->span;
    if (left != nullptr && left->max_subtree_span > max_val) {
      max_val = left->max_subtree_span;
    }
    if (right != nullptr && right->max_subtree_span > max_val) {
      max_val = right->max_subtree_span;
    }
    item->max_subtree_span = max_val;
  }
};

static bool TestRbTreeEdgeCases() {
  struct NodeGroup {
    KernelRbAugNode n0{4};
    KernelRbAugNode n1{8};
    KernelRbAugNode n2{16};
    KernelRbAugNode uninserted{32};
  } group;

  // Keys are node addresses, so n0 < n1 < n2: the tree is n1 with children n0
  // (left) and n2 (right).
  RbTree<KernelRbAugNode, &KernelRbAugNode::hook, KernelRbAugTraits> tree;
  tree.Insert(group.n0);
  tree.Insert(group.n1);
  tree.Insert(group.n2);

  // Erasing an uninserted node must not corrupt root() or existing nodes.
  tree.Erase(group.uninserted);
  SELFTEST_EXPECT(tree.root() != nullptr);

  // The first node with span >= 12 is n2: the search has to skip n0's subtree
  // (max_subtree_span 4) and the root n1 (span 8) and descend to the right.
  const auto& const_tree = tree;
  bool saw_foreign_address = false;
  const KernelRbAugNode* const match16 = const_tree.FindFirstAugmented(
      [&](const KernelRbAugNode& node) {
        if (&node != &group.n0 && &node != &group.n1 && &node != &group.n2) {
          saw_foreign_address = true;
        }
        return node.max_subtree_span >= 12;
      },
      [&](const KernelRbAugNode& node) {
        if (&node != &group.n0 && &node != &group.n1 && &node != &group.n2) {
          saw_foreign_address = true;
        }
        return node.span >= 12;
      });
  SELFTEST_EXPECT(!saw_foreign_address);
  SELFTEST_EXPECT(match16 == &group.n2);

  // Erasing n1 twice must leave n0 and n2 intact. n2 replaces n1 as the root
  // (with n0 as its left child), and the search must still find it there with
  // the augmented values updated by the erase.
  tree.Erase(group.n1);
  tree.Erase(group.n1);
  const KernelRbAugNode* const match_after_erase =
      const_tree.FindFirstAugmented(
          [&](const KernelRbAugNode& node) {
            if (&node != &group.n0 && &node != &group.n2) {
              saw_foreign_address = true;
            }
            return node.max_subtree_span >= 12;
          },
          [&](const KernelRbAugNode& node) {
            if (&node != &group.n0 && &node != &group.n2) {
              saw_foreign_address = true;
            }
            return node.span >= 12;
          });
  SELFTEST_EXPECT(!saw_foreign_address);
  SELFTEST_EXPECT(match_after_erase == &group.n2);

  tree.Erase(group.n0);
  tree.Erase(group.n2);
  SELFTEST_EXPECT(tree.root() == nullptr);

#if defined(PROTOS_INJECT_FLAKY_TEST) && PROTOS_INJECT_FLAKY_TEST > 0
  // Flake injection for checking the deflaking tool, never in default builds
  // (see INJECT_FLAKY_TEST in the Makefile): after the checks above, fails the
  // test in about one boot in kInjectedFlakeOneIn, decided by a fresh hardware
  // entropy draw that every boot logs.
  constexpr int kInjectedFlakeOneIn = PROTOS_INJECT_FLAKY_TEST;
  static_assert(kInjectedFlakeOneIn >= 1, "INJECT_FLAKY_TEST must be >= 1");
  const uint64_t injected_flake_seed = HarvestHardwareEntropySeed();
  char seed_hex[kFormatHexBufferSize];
  LineBuffer line;
  line.Append("[TEST] rbtree_edge_cases: injected flake one_in=")
      .AppendDec(static_cast<uint64_t>(kInjectedFlakeOneIn))
      .Append(" seed=")
      .Append(FormatHex(injected_flake_seed, seed_hex))
      .Append("\n");
  UartWrite(line.c_str());
  const bool injected_failure =
      injected_flake_seed % static_cast<uint64_t>(kInjectedFlakeOneIn) == 0;
  SELFTEST_EXPECT(!injected_failure);
#endif
  return true;
}

static void ResetTestMemoryMap(MultibootMemoryMap* const map) {
  map->region_count = 0;
  map->total_ram_bytes = 0;
  map->usable_ram_bytes = 0;
  map->reserved_ram_bytes = 0;
  map->mb_reserved_start = 0;
  map->mb_reserved_end = 0;
  map->mb1_mmap_reserved_start = 0;
  map->mb1_mmap_reserved_end = 0;
  map->fb_addr = 0;
  map->fb_pitch = 0;
  map->fb_width = 0;
  map->fb_height = 0;
  map->fb_bpp = 0;
  map->fb_type = 0;
}

static bool TestMultibootMmapEdgeCases() {
  const std::unique_ptr<MultibootMemoryMap> map(
      static_cast<MultibootMemoryMap*>(operator new(sizeof(MultibootMemoryMap),
                                                    std::nothrow)));
  SELFTEST_EXPECT(map != nullptr);
  ResetTestMemoryMap(map.get());

  constexpr uint64_t kHugeLen = static_cast<uint64_t>(INT64_MAX) + 4096ULL;
  SELFTEST_EXPECT(MultibootRecordMmapEntry(map.get(),  //
                                           0x100000,   //
                                           kHugeLen,   //
                                           kMemoryTypeAvailable));
  SELFTEST_EXPECT(map->usable_ram_bytes == INT64_MAX);
  SELFTEST_EXPECT(map->total_ram_bytes == INT64_MAX);
  // Type 3 (ACPI reclaimable) is reserved RAM; type 2 is reserved address
  // space that does not count as RAM at all.
  SELFTEST_EXPECT(MultibootRecordMmapEntry(map.get(), 0x200000, kHugeLen, 3));
  SELFTEST_EXPECT(map->reserved_ram_bytes == INT64_MAX);
  SELFTEST_EXPECT(map->total_ram_bytes == INT64_MAX);
  ResetTestMemoryMap(map.get());
  SELFTEST_EXPECT(
      MultibootRecordMmapEntry(map.get(), 0xFD00000000, 3ULL << 32, 2));
  SELFTEST_EXPECT(map->total_ram_bytes == 0);
  SELFTEST_EXPECT(map->reserved_ram_bytes == 0);
  SELFTEST_EXPECT(map->usable_ram_bytes == 0);

  ResetTestMemoryMap(map.get());
  SELFTEST_EXPECT(MultibootRecordMmapEntry(map.get(),  //
                                           0x200000,   //
                                           kPageSize,  //
                                           kMemoryTypeAvailable));
  SELFTEST_EXPECT(MultibootRecordMmapEntry(map.get(),  //
                                           0x202000,   //
                                           kPageSize,  //
                                           kMemoryTypeAvailable));
  SELFTEST_EXPECT(map->region_count == 2);
  SELFTEST_EXPECT(MultibootRecordMmapEntry(map.get(),  //
                                           0x201000,   //
                                           kPageSize,  //
                                           kMemoryTypeAvailable));
  SELFTEST_EXPECT(map->region_count == 1);
  SELFTEST_EXPECT(map->regions[0].base == 0x200000);
  SELFTEST_EXPECT(map->regions[0].length == 3 * kPageSize);

  struct [[gnu::packed]] SyntheticMb2FbBuffer {
    Multiboot2InfoHeader header;
    Multiboot2TagMmap mmap_tag;
    Multiboot2MmapEntry mmap_entry;
    Multiboot2TagFramebuffer fb_tag;
    Multiboot2Tag end_tag;
  };

  SyntheticMb2FbBuffer fb_buf = {};
  fb_buf.header.total_size = sizeof(SyntheticMb2FbBuffer);
  fb_buf.mmap_tag.type = kMultiboot2TagMmap;
  fb_buf.mmap_tag.size =
      sizeof(Multiboot2TagMmap) + sizeof(Multiboot2MmapEntry);
  fb_buf.mmap_tag.entry_size = sizeof(Multiboot2MmapEntry);
  fb_buf.mmap_entry.addr = 0x200000;
  fb_buf.mmap_entry.len = 4 * kPageSize;
  fb_buf.mmap_entry.type = kMemoryTypeAvailable;
  fb_buf.fb_tag.type = kMultiboot2TagFramebuffer;
  fb_buf.fb_tag.size = sizeof(Multiboot2TagFramebuffer);
  fb_buf.fb_tag.framebuffer_addr = 0x300000;
  fb_buf.fb_tag.framebuffer_pitch = 640 * 4 - 1;
  fb_buf.fb_tag.framebuffer_width = 640;
  fb_buf.fb_tag.framebuffer_height = 480;
  fb_buf.fb_tag.framebuffer_bpp = 32;
  fb_buf.fb_tag.framebuffer_type = kMultiboot2FramebufferTypeRgb;
  fb_buf.end_tag.type = kMultiboot2TagEnd;
  fb_buf.end_tag.size = sizeof(Multiboot2Tag);

  const uintptr_t fb_buf_addr = reinterpret_cast<uintptr_t>(&fb_buf);
  constexpr uint32_t kOverInt32Max = static_cast<uint32_t>(INT32_MAX) + 1u;
  const auto parse_fb_buf = [&]() {
    return MultibootParseMemoryMapQuiet(kMultiboot2Magic,   //
                                        fb_buf_addr,        //
                                        kIdentityMapLimit,  //
                                        map.get());
  };
  SELFTEST_EXPECT(parse_fb_buf());
  SELFTEST_EXPECT(map->fb_addr == 0);
  fb_buf.fb_tag.framebuffer_pitch = kOverInt32Max;
  SELFTEST_EXPECT(parse_fb_buf());
  SELFTEST_EXPECT(map->fb_addr == 0);
  fb_buf.fb_tag.framebuffer_pitch = 640 * 4;
  fb_buf.fb_tag.framebuffer_width = 0;
  SELFTEST_EXPECT(parse_fb_buf());
  SELFTEST_EXPECT(map->fb_addr == 0);

  // VgaInitFramebuffer must reject a pitch below width * (bpp / 8) and a zero
  // width without writing to the buffer. The buffer is as large as the
  // framebuffer it describes, and it is leaked instead of freed if
  // VgaInitFramebuffer accepts it, because the console then draws into it.
  constexpr int kFbWidth = 640;
  constexpr int kFbHeight = 400;
  constexpr int kFbPitch = kFbWidth * 4;
  constexpr int64_t kFbFrames =
      (static_cast<int64_t>(kFbPitch) * kFbHeight + kPageSize - 1) / kPageSize;
  constexpr uint64_t kCanary = 0xDEADBEEFCAFEBABEULL;
  const uintptr_t scratch_fb = PmmAllocFrames(kFbFrames);
  SELFTEST_EXPECT(scratch_fb != 0);
  volatile uint64_t* const scratch_words =
      reinterpret_cast<volatile uint64_t*>(scratch_fb);
  const int64_t last_word = kFbFrames * kPageSize / 8 - 1;
  scratch_words[0] = kCanary;
  scratch_words[last_word] = kCanary;
  SELFTEST_EXPECT(
      !VgaInitFramebuffer(scratch_fb, kFbPitch - 4, kFbWidth, kFbHeight, 32));
  SELFTEST_EXPECT(!VgaInitFramebuffer(scratch_fb, kFbPitch, 0, kFbHeight, 32));
  const bool scratch_untouched =
      scratch_words[0] == kCanary && scratch_words[last_word] == kCanary;
  PmmFreeFrames(scratch_fb, kFbFrames);
  SELFTEST_EXPECT(scratch_untouched);
  return true;
}

static bool TestEdgeCasesAndOom() {
  const int64_t free_before = HeapTotalFreeBytes();

  Kfree(nullptr);
  int* const null_single = nullptr;
  delete null_single;
  int* const null_array = nullptr;
  delete[] null_array;

  void* const zero_alloc = Kmalloc(0);
  SELFTEST_EXPECT(zero_alloc != nullptr);
  SELFTEST_EXPECT(
      (reinterpret_cast<uintptr_t>(zero_alloc) & (kHeapAlignment - 1)) == 0);
  Kfree(zero_alloc);

  // Requests the heap cannot satisfy make `Kmalloc`, `KmallocAligned` and the
  // `std::nothrow` forms of `operator new` return nullptr. (The throwing forms
  // would panic instead.)
  const uintptr_t max_phys = PmmMaxPhysicalAddress();
  void* const oom1 = Kmalloc(INT64_MAX);
  void* const oom2 = Kmalloc(max_phys);
  void* oom3 = operator new(max_phys, std::nothrow);
  asm volatile("" : "+r"(oom3));
  void* oom4 = operator new[](max_phys, std::nothrow);
  asm volatile("" : "+r"(oom4));
  void* const bad_align1 = KmallocAligned(64, 0);
  void* const bad_align2 = KmallocAligned(64, 24);
  void* const bad_align3 = KmallocAligned(64, kPageSize * 2);
  void* bad_align4 = operator new(
      64, static_cast<std::align_val_t>(kPageSize * 2), std::nothrow);
  asm volatile("" : "+r"(bad_align4));
  SELFTEST_EXPECT(oom1 == nullptr);
  SELFTEST_EXPECT(oom2 == nullptr);
  SELFTEST_EXPECT(oom3 == nullptr);
  SELFTEST_EXPECT(oom4 == nullptr);
  SELFTEST_EXPECT(bad_align1 == nullptr);
  SELFTEST_EXPECT(bad_align2 == nullptr);
  SELFTEST_EXPECT(bad_align3 == nullptr);
  SELFTEST_EXPECT(bad_align4 == nullptr);

  // Satisfiable `std::nothrow` requests behave like the throwing forms.
  int* const nothrow_int = new (std::nothrow) int(42);
  CacheLineAlignedWidget* const nothrow_aligned =
      new (std::nothrow) CacheLineAlignedWidget(0x5EED);
  SELFTEST_EXPECT(nothrow_int != nullptr && *nothrow_int == 42);
  SELFTEST_EXPECT(nothrow_aligned != nullptr && nothrow_aligned->IsValid());
  SELFTEST_EXPECT((reinterpret_cast<uintptr_t>(nothrow_aligned) & 63u) == 0);
  delete nothrow_int;
  delete nothrow_aligned;

  void* const recovery = Kmalloc(128);
  SELFTEST_EXPECT(recovery != nullptr);
  Kfree(recovery);

  SELFTEST_EXPECT(HeapTotalFreeBytes() == free_before);
  return true;
}

// Verifies that PagingMarkPageUncacheable rejects physical page 0 (at offsets
// 0, 0x400, and kPageSize - 1) and addresses beyond the identity map, that the
// kernel virtual page functions reject addresses outside the kernel virtual
// ranges, unaligned addresses and pages without page tables, all without
// allocating page tables, and that nothing is mapped in user space.
static bool TestPagingEdgeCases() {
  const int64_t pmm_free_before = PmmFreeFrameCount();
  SELFTEST_EXPECT(!PagingMarkPageUncacheable(0));
  SELFTEST_EXPECT(!PagingMarkPageUncacheable(0x400));
  SELFTEST_EXPECT(!PagingMarkPageUncacheable(kPageSize - 1));
  SELFTEST_EXPECT(!PagingMarkPageUncacheable(kIdentityMapLimit));

  // The first kernel virtual page has no page tables: nothing maps it.
  const uintptr_t frame = kBootstrapIdentityMapSize - kPageSize;
  SELFTEST_EXPECT(!PagingReserveKernelRange(kPageSize, kPageSize));
  SELFTEST_EXPECT(
      !PagingReserveKernelRange(kKernelVirtualBase - kPageSize, kPageSize));
  SELFTEST_EXPECT(!PagingReserveKernelRange(kKernelVirtualBase + 8, kPageSize));
  SELFTEST_EXPECT(!PagingReserveKernelRange(kKernelVirtualBase, 0));
  SELFTEST_EXPECT(!PagingReserveKernelRange(kKernelVirtualBase, 100));
  SELFTEST_EXPECT(
      !PagingReserveKernelRange(kKernelVirtualEnd - kPageSize, 2 * kPageSize));
  SELFTEST_EXPECT(!PagingReserveKernelRange(kUserSpaceBase, kPageSize));
  SELFTEST_EXPECT(!PagingReserveKernelRange(kUserSpaceEnd, kPageSize));
  SELFTEST_EXPECT(!PagingMapKernelPage(frame, frame));
  SELFTEST_EXPECT(!PagingMapKernelPage(kKernelVirtualBase, frame));
  SELFTEST_EXPECT(!PagingMapKernelPage(kKernelVirtualBase, frame + 8));
  SELFTEST_EXPECT(!PagingMapKernelPage(kUserSpaceBase, frame));
  SELFTEST_EXPECT(PagingKernelPageFrame(kKernelVirtualBase) == 0);
  SELFTEST_EXPECT(PagingUnmapKernelPage(kKernelVirtualBase) == 0);
  SELFTEST_EXPECT(PagingUnmapKernelPage(frame) == 0);
  SELFTEST_EXPECT(PagingUserSpaceIsEmpty());
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  return true;
}

// Exhausts physical memory for real, unlike edge_cases_and_oom, whose oversized
// requests the PMM and heap reject up front: allocates runs of 4096, 512, 64,
// 8, and finally single frames until the PMM runs out of each size, chaining
// the runs through their first two words (next run, frame count). While memory
// is exhausted, PMM requests and a heap allocation that needs HeapExpand()
// must fail cleanly and small heap allocations must still be served from the
// existing arena; afterwards every frame must be back and allocatable. Runs
// before the SMP tests, while nothing else allocates memory.
static bool TestPmmAndHeapExhaustion() {
  const int64_t pmm_free_before = PmmFreeFrameCount();
  const int64_t heap_free_before = HeapTotalFreeBytes();

  uintptr_t chain = 0;
  int64_t held_frames = 0;
  for (int64_t run_frames = 4096; run_frames >= 1; run_frames /= 8) {
    for (uintptr_t run = PmmAllocFrames(run_frames); run != 0;
         run = PmmAllocFrames(run_frames)) {
      volatile uintptr_t* const header =
          reinterpret_cast<volatile uintptr_t*>(run);
      header[0] = chain;
      header[1] = static_cast<uintptr_t>(run_frames);
      chain = run;
      held_frames += run_frames;
    }
  }
  const int64_t free_when_exhausted = PmmFreeFrameCount();
  const uintptr_t extra_frame = PmmAllocFrame();
  const uintptr_t extra_run = PmmAllocFrames(2);
  void* const needs_expansion = Kmalloc(heap_free_before + kPageSize);
  void* const fits_arena = Kmalloc(64);

  Kfree(fits_arena);
  Kfree(needs_expansion);
  if (extra_frame != 0) {
    PmmFreeFrame(extra_frame);
  }
  if (extra_run != 0) {
    PmmFreeFrames(extra_run, 2);
  }
  while (chain != 0) {
    const volatile uintptr_t* const header =
        reinterpret_cast<const volatile uintptr_t*>(chain);
    const uintptr_t next = header[0];
    PmmFreeFrames(chain, static_cast<int64_t>(header[1]));
    chain = next;
  }

  SELFTEST_EXPECT(held_frames == pmm_free_before);
  SELFTEST_EXPECT(free_when_exhausted == 0);
  SELFTEST_EXPECT(extra_frame == 0);
  SELFTEST_EXPECT(extra_run == 0);
  SELFTEST_EXPECT(needs_expansion == nullptr);
  SELFTEST_EXPECT(fits_arena != nullptr);
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  SELFTEST_EXPECT(HeapTotalFreeBytes() == heap_free_before);

  const uintptr_t recovered = PmmAllocFrame();
  SELFTEST_EXPECT(recovered != 0);
  PmmFreeFrame(recovered);
  return true;
}

struct SmpAllCpuStartupVerifyContext {
  int cpu_count = 0;
  std::atomic<uint64_t> mask_low{0};
  std::atomic<uint64_t> mask_high{0};
  std::atomic<bool> failed{false};
};

static uint64_t ReadGsBaseMsrForSmpVerify() {
  uint32_t low = 0;
  uint32_t high = 0;
  asm volatile("rdmsr" : "=a"(low), "=d"(high) : "c"(kMsrIa32GsBase));
  return (static_cast<uint64_t>(high) << 32) | low;
}

static uint64_t ReadCr0ForSmpVerify() {
  uint64_t cr0 = 0;
  asm volatile("mov %0, cr0" : "=r"(cr0));
  return cr0;
}

static uint16_t ReadTaskRegisterForSmpVerify() {
  uint16_t selector = 0;
  asm volatile("str %0" : "=r"(selector));
  return selector;
}

static GdtPointer ReadGdtrForSmpVerify() {
  GdtPointer gdtr = {};
  asm volatile("sgdt [%0]" : : "r"(&gdtr) : "memory");
  return gdtr;
}

static uint64_t ReadEferForSmpVerify() {
  constexpr uint32_t kMsrIa32Efer = 0xC0000080u;
  uint32_t low = 0;
  uint32_t high = 0;
  asm volatile("rdmsr" : "=a"(low), "=d"(high) : "c"(kMsrIa32Efer));
  return (static_cast<uint64_t>(high) << 32) | low;
}

// Returns true if CPUID reports no-execute page support (leaf 0x80000001, EDX
// bit 20). boot.S sets EFER.NXE exactly on such CPUs.
static bool CpuSupportsNxForSmpVerify() {
  uint32_t eax = 0x80000001u;
  uint32_t ebx = 0;
  uint32_t ecx = 0;
  uint32_t edx = 0;
  asm volatile("cpuid" : "+a"(eax), "=b"(ebx), "+c"(ecx), "=d"(edx));
  return (edx & (1u << 20)) != 0;
}

static void SmpAllCpuStartupVerifyWorker(const int cpu_index,
                                         void* const raw_ctx) {
  SmpAllCpuStartupVerifyContext* const ctx =
      static_cast<SmpAllCpuStartupVerifyContext*>(raw_ctx);
  if (ctx == nullptr || cpu_index < 0 || cpu_index >= ctx->cpu_count ||
      cpu_index >= kMaxSupportedCpus) {
    if (ctx != nullptr) {
      ctx->failed.store(true, std::memory_order_release);
    }
    return;
  }

  constexpr uint64_t kCr0WriteProtect = 1ULL << 16;
  constexpr uint64_t kEferNoExecuteEnable = 1ULL << 11;
  const CpuLocal* const local = CurrentCpu();
  const CpuInfo* const info = SmpGetCpuInfo(cpu_index);
  const GdtPointer gdtr = ReadGdtrForSmpVerify();
  if (local == nullptr || info == nullptr ||
      local != SmpGetCpuLocal(cpu_index) || local->self != local ||
      ReadGsBaseMsrForSmpVerify() != reinterpret_cast<uintptr_t>(local) ||
      (ReadCr0ForSmpVerify() & kCr0WriteProtect) == 0 ||
      ((ReadEferForSmpVerify() & kEferNoExecuteEnable) != 0) !=
          CpuSupportsNxForSmpVerify() ||
      CurrentCpuId() != cpu_index || local->cpu_id != cpu_index ||
      local->apic_id != info->apic_id || info->apic_id == kInvalidX2ApicId ||
      gdtr.base != GdtGetPointer().base ||
      gdtr.limit != GdtGetPointer().limit ||
      ReadTaskRegisterForSmpVerify() != GdtTssSelector(cpu_index)) {
    ctx->failed.store(true, std::memory_order_release);
    return;
  }

  if (cpu_index < 64) {
    ctx->mask_low.fetch_or(1ULL << static_cast<uint32_t>(cpu_index),
                           std::memory_order_acq_rel);
  } else {
    ctx->mask_high.fetch_or(1ULL << static_cast<uint32_t>(cpu_index - 64),
                            std::memory_order_acq_rel);
  }
}

static bool VerifySmpAllCpuStartupAndGsBase() {
  const int cpu_count = SmpCpuCount();
  SELFTEST_EXPECT(cpu_count >= 1 && cpu_count <= kMaxSupportedCpus);
  SELFTEST_EXPECT(SmpOnlineCpuCount() == cpu_count);

  SmpAllCpuStartupVerifyContext ctx = {};
  ctx.cpu_count = cpu_count;
  SmpRunOnAllCpus(SmpAllCpuStartupVerifyWorker, &ctx);
  SELFTEST_EXPECT(!ctx.failed.load(std::memory_order_acquire));

  const CpuMask128 expected = CpuMask128::FirstCpus(cpu_count);
  SELFTEST_EXPECT(ctx.mask_low.load(std::memory_order_acquire) == expected.low);
  SELFTEST_EXPECT(ctx.mask_high.load(std::memory_order_acquire) ==
                  expected.high);
  return true;
}

// Returns true if the stack that starts at `stack_base` is mapped and sits
// directly above a guard page, which stays unmapped when its whole 2 MiB
// region is identity-mapped again.
static bool VerifyStackGuardPage(const uintptr_t stack_base) {
  const uintptr_t guard_page = stack_base - kPageSize;
  const uintptr_t region = guard_page & ~(kHugePageSize - 1);
  return SELFTEST_OK(PagingIsIdentityMapped(stack_base)) &&
         SELFTEST_OK(PagingIsGuardPage(guard_page)) &&
         SELFTEST_OK(!PagingIsIdentityMapped(guard_page)) &&
         SELFTEST_OK(PagingEnsureIdentityMappedRange(region, kHugePageSize)) &&
         SELFTEST_OK(PagingIsGuardPage(guard_page));
}

// Verifies that CPU `cpu_index`'s kernel stack and the IST stacks in its TSS
// each sit above a guard page.
static bool VerifyCpuStackGuardPages(const int cpu_index) {
  const CpuInfo* const info = SmpGetCpuInfo(cpu_index);
  const TaskStateSegment* const tss = GdtGetTss(cpu_index);
  SELFTEST_EXPECT(VerifyStackGuardPage(info->stack_base));
  for (int i = 0; i < kIstStackCount; ++i) {
    const uintptr_t ist_stack_top = tss->ist[i];
    SELFTEST_EXPECT(ist_stack_top != 0 && (ist_stack_top & 0xF) == 0);
    SELFTEST_EXPECT(VerifyStackGuardPage(ist_stack_top - kIstStackSize));
  }
  for (int i = kIstStackCount; i < 7; ++i) {
    SELFTEST_EXPECT(tss->ist[i] == 0);
  }
  SELFTEST_EXPECT(tss->io_map_base ==
                  static_cast<uint16_t>(sizeof(TaskStateSegment)));
  return true;
}

static bool TestSmpDiscoveryAndApBringup() {
  const int cpu_count = SmpCpuCount();
  const int online_count = SmpOnlineCpuCount();
  const uintptr_t lapic_phys = SmpLocalApicPhysAddr();
  SELFTEST_EXPECT(cpu_count >= 1 && cpu_count <= kMaxSupportedCpus);
  SELFTEST_EXPECT(online_count == cpu_count);
  if (!SmpIsX2ApicEnabled()) {
    SELFTEST_EXPECT(lapic_phys != 0);
    const uintptr_t lapic_neighbor_phys =
        (lapic_phys & ~(static_cast<uintptr_t>(kHugePageSize) - 1)) + kPageSize;
    SELFTEST_EXPECT(PagingIsIdentityMapped(lapic_phys));
    SELFTEST_EXPECT(PagingIsIdentityMapped(lapic_neighbor_phys));
    SELFTEST_EXPECT(PagingEnsureIdentityMappedRange(lapic_phys, kPageSize));
    SELFTEST_EXPECT(
        PagingEnsureIdentityMappedRange(lapic_neighbor_phys, kPageSize));
    SELFTEST_EXPECT(PagingIsIdentityMapped(lapic_phys));
    SELFTEST_EXPECT(PagingIsIdentityMapped(lapic_neighbor_phys));
  }

  const CpuInfo* const bsp = SmpGetCpuInfo(0);
  SELFTEST_EXPECT(bsp != nullptr);
  SELFTEST_EXPECT(bsp->is_bsp);
  SELFTEST_EXPECT(bsp->online);
  SELFTEST_EXPECT(bsp->long_mode_active);
  SELFTEST_EXPECT(bsp->apic_id != kInvalidX2ApicId);
  SELFTEST_EXPECT(bsp->observed_apic_id == bsp->apic_id);
  SELFTEST_EXPECT(bsp->stack_base != 0);
  SELFTEST_EXPECT(bsp->stack_top > bsp->stack_base);
  SELFTEST_EXPECT((bsp->stack_top & 0xF) == 0);
  SELFTEST_EXPECT(bsp->observed_rsp > bsp->stack_base);
  SELFTEST_EXPECT(bsp->observed_rsp <= bsp->stack_top);
  SELFTEST_EXPECT(bsp->observed_cr3 != 0);

  for (int i = 1; i < cpu_count; ++i) {
    const CpuInfo* const ap = SmpGetCpuInfo(i);
    SELFTEST_EXPECT(ap != nullptr);
    SELFTEST_EXPECT(!ap->is_bsp);
    SELFTEST_EXPECT(ap->online);
    SELFTEST_EXPECT(ap->long_mode_active);
    SELFTEST_EXPECT(ap->apic_id != kInvalidX2ApicId);
    SELFTEST_EXPECT(ap->observed_apic_id == ap->apic_id);
    SELFTEST_EXPECT(ap->stack_base != 0);
    SELFTEST_EXPECT(ap->stack_top == ap->stack_base + kApStackSize);
    SELFTEST_EXPECT((ap->stack_top & 0xF) == 0);
    SELFTEST_EXPECT(ap->observed_rsp > ap->stack_base);
    SELFTEST_EXPECT(ap->observed_rsp <= ap->stack_top);
    SELFTEST_EXPECT(ap->observed_cr3 == bsp->observed_cr3);
    SELFTEST_EXPECT(PmmRangeIsValidUsableRam(ap->stack_base, kApStackSize));

    for (int j = 0; j < i; ++j) {
      const CpuInfo* const other = SmpGetCpuInfo(j);
      SELFTEST_EXPECT(other != nullptr);
      SELFTEST_EXPECT(ap->apic_id != other->apic_id);
      SELFTEST_EXPECT(ap->stack_base >= other->stack_top ||
                      ap->stack_top <= other->stack_base);
    }
  }

  for (int i = 0; i < cpu_count; ++i) {
    SELFTEST_EXPECT(VerifyCpuStackGuardPages(i));
  }
  SELFTEST_EXPECT(VerifySmpAllCpuStartupAndGsBase());
  return true;
}

struct ConcurrentHeapItem {
  uint64_t tag;
  uint64_t inverted_tag;

  explicit ConcurrentHeapItem(const uint64_t seed)
      : tag(seed), inverted_tag(~seed) {}

  bool IsValid(const uint64_t expected_seed) const {
    return tag == expected_seed && inverted_tag == ~expected_seed;
  }
};

struct KfreeDeleter {
  void operator()(void* const ptr) const { Kfree(ptr); }
};

class ScopedPmmFrames {
 public:
  ScopedPmmFrames(const uintptr_t addr, const int64_t count)
      : addr_(addr), count_(count) {}

  ~ScopedPmmFrames() {
    if (addr_ != 0) {
      PmmFreeFrames(addr_, count_);
    }
  }

  ScopedPmmFrames(const ScopedPmmFrames&) = delete;
  ScopedPmmFrames& operator=(const ScopedPmmFrames&) = delete;

  uintptr_t get() const { return addr_; }

 private:
  uintptr_t addr_;
  int64_t count_;
};

struct MultiCpuStressContext {
  IrqSpinLock counter_lock;
  int64_t shared_counter = 0;
  int64_t invariant_a = 0;
  int64_t invariant_b = 0;
  int expected_cpus = 0;
  std::atomic<int> start_barrier_count{0};
  int second_pass_count = 0;
  std::unique_ptr<bool[]> per_cpu_ok;
};

constexpr int kMultiCpuStressIterations = 64;

static void MultiCpuStressWorker(const int cpu_index, void* const raw_context) {
  MultiCpuStressContext* const ctx =
      static_cast<MultiCpuStressContext*>(raw_context);
  if (ctx == nullptr || cpu_index < 0 || cpu_index >= ctx->expected_cpus) {
    return;
  }

  ctx->start_barrier_count.fetch_add(1, std::memory_order_acq_rel);
  while (ctx->start_barrier_count.load(std::memory_order_acquire) <
         ctx->expected_cpus) {
    asm volatile("pause" : : : "memory");
  }

  bool ok = true;
  for (int iter = 0; iter < kMultiCpuStressIterations; ++iter) {
    // 1. Increment shared counter and update two-part invariant under
    //    IrqSpinLock.
    {
      const IrqSpinLockGuard guard(ctx->counter_lock);
      ok = SELFTEST_OK(!AreInterruptsEnabled()) && ok;
      ok = SELFTEST_OK(ctx->counter_lock.IsLockedByCurrentCpu()) && ok;
      ok = SELFTEST_OK(ctx->invariant_a + ctx->invariant_b == 0) && ok;
      const int64_t delta = (cpu_index + 1) * 17 + iter;
      ctx->invariant_a += delta;
      ++ctx->shared_counter;
      ctx->invariant_b -= delta;
    }

    // 2. Concurrent PMM single-frame and multi-frame allocation + pattern
    //    verification.
    const ScopedPmmFrames frame_single(PmmAllocFrame(), 1);
    const ScopedPmmFrames frame_pair(PmmAllocFrames(2), 2);
    const bool frames_ok =
        SELFTEST_OK(frame_single.get() != 0) &&
        SELFTEST_OK(frame_pair.get() != 0) &&
        SELFTEST_OK(frame_single.get() != frame_pair.get()) &&
        SELFTEST_OK((frame_single.get() & (kPageSize - 1)) == 0) &&
        SELFTEST_OK((frame_pair.get() & (kPageSize - 1)) == 0) &&
        SELFTEST_OK(PmmRangeIsValidUsableRam(frame_single.get(), kPageSize)) &&
        SELFTEST_OK(PmmRangeIsValidUsableRam(frame_pair.get(), 2 * kPageSize));
    if (!frames_ok) {
      ok = false;
      break;
    }

    const uint64_t frame_pat1 = 0xC0DEC0DE00000000ULL |
                                (static_cast<uint64_t>(cpu_index) << 16) |
                                static_cast<uint64_t>(iter);
    const uint64_t frame_pat2 = frame_pat1 ^ 0x55AA55AA55AA55AAULL;
    volatile uint64_t* const f1_words =
        reinterpret_cast<volatile uint64_t*>(frame_single.get());
    volatile uint64_t* const f2_first =
        reinterpret_cast<volatile uint64_t*>(frame_pair.get());
    volatile uint64_t* const f2_second =
        reinterpret_cast<volatile uint64_t*>(frame_pair.get() + kPageSize);
    f1_words[0] = frame_pat1;
    f1_words[kPageSize / sizeof(uint64_t) - 1] = ~frame_pat1;
    f2_first[0] = frame_pat2;
    f2_second[kPageSize / sizeof(uint64_t) - 1] = ~frame_pat2;

    // 3. Concurrent Kmalloc/Kfree and C++ new/delete allocations with per-CPU
    //    payload verification.
    const int64_t size_a = 32 + ((iter + cpu_index) % 8) * 32;
    const int64_t size_b = 128 + ((iter * 3 + cpu_index) % 8) * 64;
    const std::unique_ptr<uint8_t[], KfreeDeleter> buf_a(
        static_cast<uint8_t*>(Kmalloc(size_a)));
    const std::unique_ptr<uint8_t[], KfreeDeleter> buf_b(
        static_cast<uint8_t*>(Kmalloc(size_b)));
    const uint64_t item_seed =
        0xA5A5000000000000ULL | (static_cast<uint64_t>(cpu_index) << 24) | iter;
    const std::unique_ptr<ConcurrentHeapItem> heap_item =
        std::make_unique<ConcurrentHeapItem>(item_seed);
    constexpr int kArrayWords = 8;
    const std::unique_ptr<uint64_t[]> heap_array =
        std::make_unique<uint64_t[]>(kArrayWords);

    const bool heap_ok = SELFTEST_OK(buf_a != nullptr) &&
                         SELFTEST_OK(buf_b != nullptr) &&
                         SELFTEST_OK(heap_item != nullptr) &&
                         SELFTEST_OK(heap_array != nullptr) &&
                         SELFTEST_OK((reinterpret_cast<uintptr_t>(buf_a.get()) &
                                      (kHeapAlignment - 1)) == 0) &&
                         SELFTEST_OK((reinterpret_cast<uintptr_t>(buf_b.get()) &
                                      (kHeapAlignment - 1)) == 0);
    if (!heap_ok) {
      ok = false;
      break;
    }

    const uint8_t byte_a =
        static_cast<uint8_t>(((cpu_index + 1) * 29 + iter * 7) & 0xFF);
    const uint8_t byte_b = static_cast<uint8_t>(byte_a ^ 0x3Cu);
    for (int64_t i = 0; i < size_a; ++i) {
      buf_a[i] = static_cast<uint8_t>(byte_a + (i & 0xF));
    }
    for (int64_t i = 0; i < size_b; ++i) {
      buf_b[i] = static_cast<uint8_t>(byte_b ^ (i & 0xF));
    }
    for (int i = 0; i < kArrayWords; ++i) {
      heap_array[i] = item_seed + i;
    }

    // Acquire the shared spinlock a second time while holding live PMM frames
    // and Heap allocations so allocations across CPUs overlap in time.
    {
      const IrqSpinLockGuard guard(ctx->counter_lock);
      ok = SELFTEST_OK(ctx->invariant_a + ctx->invariant_b == 0) && ok;
      const int64_t delta = (cpu_index + 3) * 31 + iter;
      ctx->invariant_a -= delta;
      ++ctx->shared_counter;
      ctx->invariant_b += delta;
    }

    ok = SELFTEST_OK(f1_words[0] == frame_pat1) && ok;
    ok = SELFTEST_OK(f1_words[kPageSize / sizeof(uint64_t) - 1] ==
                     ~frame_pat1) &&
         ok;
    ok = SELFTEST_OK(f2_first[0] == frame_pat2) && ok;
    ok = SELFTEST_OK(f2_second[kPageSize / sizeof(uint64_t) - 1] ==
                     ~frame_pat2) &&
         ok;
    ok = SELFTEST_OK(heap_item->IsValid(item_seed)) && ok;
    for (int64_t i = 0; i < size_a; ++i) {
      if (!SELFTEST_OK(buf_a[i] == static_cast<uint8_t>(byte_a + (i & 0xF)))) {
        ok = false;
        break;
      }
    }
    for (int64_t i = 0; i < size_b; ++i) {
      if (!SELFTEST_OK(buf_b[i] == static_cast<uint8_t>(byte_b ^ (i & 0xF)))) {
        ok = false;
        break;
      }
    }
    for (int i = 0; i < kArrayWords; ++i) {
      if (!SELFTEST_OK(heap_array[i] == item_seed + i)) {
        ok = false;
        break;
      }
    }

    // 4. Concurrent UART and VGA console logging on the first and last
    //    iterations. Exercises VgaWrite, VgaPutc, VgaWriteDec, and
    //    VgaWriteHex on all online CPUs, ending every VGA write with '\r' so
    //    the cursor stays on the current row and the greeting on row 0 never
    //    scrolls off.
    if (iter == 0 || iter + 1 == kMultiCpuStressIterations) {
      LineBuffer uart_line;
      uart_line.Append("[SMP] CPU ")
          .AppendDec(static_cast<uint64_t>(cpu_index))
          .Append(" sync stress iteration ")
          .AppendDec(static_cast<uint64_t>(iter))
          .Append(ok ? ": ok\n" : ": FAIL\n");
      UartWrite(uart_line.c_str());
      LineBuffer vga_line;
      vga_line.Append("[SMP] CPU ")
          .AppendDec(static_cast<uint64_t>(cpu_index))
          .Append(ok ? " ok\r" : " FAIL\r");
      VgaWrite(vga_line.c_str());
      VgaPutc('\r');
      {
        const IrqSpinLockGuard guard(ctx->counter_lock);
        VgaWriteDec(static_cast<uint64_t>(cpu_index));
        VgaWriteHex(static_cast<uint64_t>(iter));
        VgaPutc('\r');
      }
    }
  }

  ctx->per_cpu_ok[cpu_index] = ok;
}

static void MultiCpuSecondPassWorker(const int cpu_index,
                                     void* const raw_context) {
  MultiCpuStressContext* const ctx =
      static_cast<MultiCpuStressContext*>(raw_context);
  if (ctx == nullptr || cpu_index < 0 || cpu_index >= ctx->expected_cpus) {
    return;
  }
  const IrqSpinLockGuard guard(ctx->counter_lock);
  ctx->second_pass_count += (cpu_index + 1);
}

static bool TestSmpMultiCpuSyncStress() {
  const int cpu_count = SmpCpuCount();
  SELFTEST_EXPECT(cpu_count >= 1 && cpu_count <= kMaxSupportedCpus);
  SELFTEST_EXPECT(SmpOnlineCpuCount() == cpu_count);

  MultiCpuStressContext ctx = {};
  ctx.expected_cpus = cpu_count;
  ctx.per_cpu_ok = std::make_unique<bool[]>(cpu_count);

  const int64_t pmm_free_before = PmmFreeFrameCount();
  const int64_t heap_free_before = HeapTotalFreeBytes();
  SELFTEST_EXPECT(pmm_free_before > 0);
  SELFTEST_EXPECT(heap_free_before > 0);

  SmpRunOnAllCpus(MultiCpuStressWorker, &ctx);
  // Blank out the scratch characters that the workers left on the current VGA
  // row; the cursor is back at column 0.
  VgaWrite("                      \r");

  // Dispatch a second round of work across all CPUs to verify APs returned
  // cleanly to their halted state after the first round and can be re-woken.
  SmpRunOnAllCpus(MultiCpuSecondPassWorker, &ctx);

  const int64_t expected_counter =
      static_cast<int64_t>(cpu_count) * kMultiCpuStressIterations * 2;
  const int expected_second_pass = (cpu_count * (cpu_count + 1)) / 2;
  SELFTEST_EXPECT(ctx.shared_counter == expected_counter);
  SELFTEST_EXPECT(ctx.invariant_a + ctx.invariant_b == 0);
  SELFTEST_EXPECT(ctx.second_pass_count == expected_second_pass);
  for (int i = 0; i < cpu_count; ++i) {
    SELFTEST_EXPECT(ctx.per_cpu_ok[i]);
  }

  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  SELFTEST_EXPECT(HeapTotalFreeBytes() == heap_free_before);
  SELFTEST_EXPECT(SmpOnlineCpuCount() == cpu_count);
  return true;
}

constexpr uint64_t kInt3CanaryBase = 0x1111222233334440ULL;
constexpr uint64_t kRflagsDirectionFlag = 1ULL << 10;

std::atomic<int> g_int3_handler_hits{0};
std::atomic<bool> g_int3_frame_ok{false};

static void Int3VerificationHandler(InterruptFrame* const frame) {
  if (frame == nullptr) {
    return;
  }
  uint64_t live_rflags = 0;
  asm volatile("pushfq\n\tpop %0" : "=r"(live_rflags) : : "memory");
  const bool ok = frame->vector == static_cast<uint64_t>(kVectorBreakpoint) &&
                  frame->error_code == 0 &&
                  frame->cs == kIdtKernelCodeSelector &&
                  (live_rflags & kRflagsDirectionFlag) == 0 &&
                  (frame->rflags & kRflagsDirectionFlag) != 0 &&
                  frame->rax == (kInt3CanaryBase + 0) &&
                  frame->rbx == (kInt3CanaryBase + 1) &&
                  frame->rcx == (kInt3CanaryBase + 2) &&
                  frame->rdx == (kInt3CanaryBase + 3) &&
                  frame->rsi == (kInt3CanaryBase + 4) &&
                  frame->rdi == (kInt3CanaryBase + 5) &&
                  frame->rbp == (kInt3CanaryBase + 6) &&
                  frame->r8 == (kInt3CanaryBase + 7) &&
                  frame->r9 == (kInt3CanaryBase + 8) &&
                  frame->r10 == (kInt3CanaryBase + 9) &&
                  frame->r11 == (kInt3CanaryBase + 10) &&
                  frame->r12 == (kInt3CanaryBase + 11) &&
                  frame->r13 == (kInt3CanaryBase + 12) &&
                  frame->r14 == (kInt3CanaryBase + 13) &&
                  frame->r15 == (kInt3CanaryBase + 14);
  if (ok) {
    g_int3_frame_ok.store(true, std::memory_order_release);
  }
  g_int3_handler_hits.fetch_add(1, std::memory_order_acq_rel);
}

static bool TriggerInt3AndVerifyAllRegisters() {
  g_int3_handler_hits.store(0, std::memory_order_release);
  g_int3_frame_ok.store(false, std::memory_order_release);
  const InterruptHandlerFn previous_handler = IdtGetHandler(kVectorBreakpoint);
  IdtRegisterHandler(kVectorBreakpoint, Int3VerificationHandler);

  uint64_t post_regs[16] = {};
  uint64_t* const out_ptr = post_regs;
  // End-to-end hardware trap test for all 15 general-purpose registers and
  // System V AMD64 ABI direction-flag (`RFLAGS.DF`) hygiene:
  // 1. Save callee-saved registers (`rbp`, `rbx`, `r12`..`r15`) that GCC
  //    forbids listing in the clobber list (especially `rbp`/`rbx`), and push
  //    `out_ptr` (`&post_regs[0]`) onto the stack before overwriting all 15
  //    GPRs (`rax`..`r15`) so we still have `out_ptr` afterwards.
  // 2. Reserve a 128-byte scratch area on the stack (`16 * 8` bytes, at
  //    `[rsp + 0 .. 120]`) to dump the 15 post-`iretq` GPRs plus the restored
  //    post-`iretq` `RFLAGS`, while `out_ptr` sits at `[rsp + 128]`.
  // 3. Load distinct 64-bit canary values (`kInt3CanaryBase + 0..14`) into all
  //    15 GPRs, set `RFLAGS.DF = 1` via `std`, and execute `int3` (Vector 3
  //    `#BP`), trapping into `isr_stub_3` -> `isr_common_stub` (`cld`) ->
  //    `IdtDispatch` -> `Int3VerificationHandler`.
  // 4. Immediately after `iretq` returns from the interrupt, store all 15 GPRs
  //    into `[rsp + 0 .. 112]` without touching `RFLAGS`, capture the restored
  //    `RFLAGS` into `[rsp + 120]` via `pushfq; pop rax`, clear `DF` via `cld`,
  //    and copy all 16 qwords (`rep movsq`) into `out_ptr` (`post_regs`).
  // 5. Reclaim the 136 bytes of stack space (`128` scratch + `8` for `out_ptr`)
  //    and restore the caller's callee-saved registers.
  asm volatile(
      "push rbp\n\t"
      "push rbx\n\t"
      "push r12\n\t"
      "push r13\n\t"
      "push r14\n\t"
      "push r15\n\t"
      "push %[out]\n\t"
      "sub rsp, 128\n\t"
      "mov rax, 0x1111222233334440\n\t"
      "mov rbx, 0x1111222233334441\n\t"
      "mov rcx, 0x1111222233334442\n\t"
      "mov rdx, 0x1111222233334443\n\t"
      "mov rsi, 0x1111222233334444\n\t"
      "mov rdi, 0x1111222233334445\n\t"
      "mov rbp, 0x1111222233334446\n\t"
      "mov r8,  0x1111222233334447\n\t"
      "mov r9,  0x1111222233334448\n\t"
      "mov r10, 0x1111222233334449\n\t"
      "mov r11, 0x111122223333444A\n\t"
      "mov r12, 0x111122223333444B\n\t"
      "mov r13, 0x111122223333444C\n\t"
      "mov r14, 0x111122223333444D\n\t"
      "mov r15, 0x111122223333444E\n\t"
      "std\n\t"
      "int3\n\t"
      "mov qword ptr [rsp + 0],   rax\n\t"
      "mov qword ptr [rsp + 8],   rbx\n\t"
      "mov qword ptr [rsp + 16],  rcx\n\t"
      "mov qword ptr [rsp + 24],  rdx\n\t"
      "mov qword ptr [rsp + 32],  rsi\n\t"
      "mov qword ptr [rsp + 40],  rdi\n\t"
      "mov qword ptr [rsp + 48],  rbp\n\t"
      "mov qword ptr [rsp + 56],  r8\n\t"
      "mov qword ptr [rsp + 64],  r9\n\t"
      "mov qword ptr [rsp + 72],  r10\n\t"
      "mov qword ptr [rsp + 80],  r11\n\t"
      "mov qword ptr [rsp + 88],  r12\n\t"
      "mov qword ptr [rsp + 96],  r13\n\t"
      "mov qword ptr [rsp + 104], r14\n\t"
      "mov qword ptr [rsp + 112], r15\n\t"
      "pushfq\n\t"
      "pop rax\n\t"
      "mov qword ptr [rsp + 120], rax\n\t"
      "cld\n\t"
      "mov rdi, qword ptr [rsp + 128]\n\t"
      "mov rsi, rsp\n\t"
      "mov rcx, 16\n\t"
      "rep movsq\n\t"
      "add rsp, 136\n\t"
      "pop r15\n\t"
      "pop r14\n\t"
      "pop r13\n\t"
      "pop r12\n\t"
      "pop rbx\n\t"
      "pop rbp"
      :
      : [out] "r"(out_ptr)
      : "rax", "rcx", "rdx", "rsi", "rdi", "r8", "r9", "r10", "r11", "memory",
        "cc");

  if (previous_handler != nullptr) {
    IdtRegisterHandler(kVectorBreakpoint, previous_handler);
  } else {
    IdtUnregisterHandler(kVectorBreakpoint);
  }
  SELFTEST_EXPECT(IdtGetHandler(kVectorBreakpoint) == previous_handler);
  SELFTEST_EXPECT(g_int3_handler_hits.load(std::memory_order_acquire) == 1);
  SELFTEST_EXPECT(g_int3_frame_ok.load(std::memory_order_acquire));
  SELFTEST_EXPECT((post_regs[15] & kRflagsDirectionFlag) != 0);
  for (int i = 0; i < 15; ++i) {
    SELFTEST_EXPECT(post_regs[i] == kInt3CanaryBase + static_cast<uint64_t>(i));
  }
  return true;
}

// Vector of the ring IPIs of TestIdtPerCpuAndLapicTimer(). No kernel subsystem
// uses it, so every delivery is one of the test's IPIs. (Vector 0x21 would not
// do: SmpRunOnAllCpus also sends it to wake the APs.)
constexpr uint8_t kVectorSelfTestRingIpi = 0x40;

// CPUs that received a ring IPI.
AtomicCpuMask128 g_ring_ipi_receivers;

static void SelfTestRingIpiHandler(InterruptFrame* const /*frame*/) {
  g_ring_ipi_receivers.Add(CurrentCpuId());
}

struct IdtPerCpuAndTimerVerifyContext {
  int cpu_count = 0;
  std::unique_ptr<int64_t[]> initial_ticks;
  std::unique_ptr<bool[]> per_cpu_ok;
};

static void IdtPerCpuAndTimerVerifyWorker(const int cpu_index,
                                          void* const raw_ctx) {
  IdtPerCpuAndTimerVerifyContext* const ctx =
      static_cast<IdtPerCpuAndTimerVerifyContext*>(raw_ctx);
  if (ctx == nullptr || cpu_index < 0 || cpu_index >= ctx->cpu_count) {
    return;
  }

  CpuLocal* const local = CurrentCpu();
  const CpuInfo* const info = SmpGetCpuInfo(cpu_index);
  const bool state_ok =
      SELFTEST_OK(local != nullptr) && SELFTEST_OK(info != nullptr) &&
      SELFTEST_OK(local == SmpGetCpuLocal(cpu_index)) &&
      SELFTEST_OK(local->self == local) &&
      SELFTEST_OK((reinterpret_cast<uintptr_t>(local) & 63u) == 0) &&
      SELFTEST_OK(ReadGsBaseMsrForSmpVerify() ==
                  reinterpret_cast<uintptr_t>(local)) &&
      SELFTEST_OK(CurrentCpuId() == cpu_index) &&
      SELFTEST_OK(local->cpu_id == cpu_index) &&
      SELFTEST_OK(local->apic_id == info->apic_id) &&
      SELFTEST_OK(local->online) &&
      SELFTEST_OK(local->stack_base == info->stack_base) &&
      SELFTEST_OK(local->stack_top == info->stack_top);
  if (!state_ok) {
    ctx->per_cpu_ok[cpu_index] = false;
    return;
  }

  // Send a ring IPI to the next CPU, so that every CPU (CPU 0 included)
  // receives exactly one.
  SmpSendIpi((cpu_index + 1) % ctx->cpu_count, kVectorSelfTestRingIpi);

  // With interrupts enabled, wait for two more timer ticks and the ring IPI of
  // the previous CPU.
  const int64_t target_ticks = ctx->initial_ticks[cpu_index] + 2;
  const TscDeadline deadline(kSelfTestWaitTimeoutTscCycles);
  while (!deadline.Expired() &&
         (local->timer_ticks.load(std::memory_order_acquire) < target_ticks ||
          !g_ring_ipi_receivers.Load().Contains(cpu_index))) {
    asm volatile("sti; pause" : : : "memory", "cc");
  }
  ctx->per_cpu_ok[cpu_index] =
      SELFTEST_OK(local->timer_ticks.load(std::memory_order_acquire) >=
                  target_ticks) &&
      SELFTEST_OK(g_ring_ipi_receivers.Load().Contains(cpu_index));
}

// IST slot of the IDT gate for `vector`: #DF, NMI and #MC have stacks of their
// own (gdt.h).
static uint8_t ExpectedIstSlot(const int vector) {
  if (vector == kVectorDoubleFault) {
    return kIstDoubleFault;
  }
  if (vector == kVectorNmi) {
    return kIstNmi;
  }
  return (vector == kVectorMachineCheck) ? kIstMachineCheck : 0;
}

static bool TestIdtPerCpuAndLapicTimer() {
  SELFTEST_EXPECT(IdtIsInitialized());
  SELFTEST_EXPECT(SmpCalibratedTimerInitialCount() != 0);

  const IdtPointer idtr = IdtGetPointer();
  SELFTEST_EXPECT(idtr.limit == sizeof(IdtGateDescriptor) * kIdtEntryCount - 1);
  SELFTEST_EXPECT(idtr.base == reinterpret_cast<uint64_t>(IdtGetGate(0)));
  for (int v = 0; v < kIdtEntryCount; ++v) {
    const IdtGateDescriptor* const gate = IdtGetGate(v);
    SELFTEST_EXPECT(gate != nullptr);
    SELFTEST_EXPECT(gate->selector == kIdtKernelCodeSelector);
    SELFTEST_EXPECT(gate->ist == ExpectedIstSlot(v));
    // Only system calls may come from user code.
    SELFTEST_EXPECT(gate->type_attr == ((v == kVectorSyscall)
                                            ? kIdtUserInterruptGateAttr
                                            : kIdtInterruptGateAttr));
    SELFTEST_EXPECT(gate->reserved == 0);
    SELFTEST_EXPECT(IdtDecodeGateOffset(*gate) == g_isr_stub_table[v]);
  }

  SELFTEST_EXPECT(TriggerInt3AndVerifyAllRegisters());

  const int cpu_count = SmpCpuCount();
  SELFTEST_EXPECT(cpu_count >= 1);
  SELFTEST_EXPECT(SmpOnlineCpuCount() == cpu_count);
  SELFTEST_EXPECT(IdtGetHandler(kVectorSelfTestRingIpi) == nullptr);

  IdtPerCpuAndTimerVerifyContext ctx = {};
  ctx.cpu_count = cpu_count;
  ctx.initial_ticks = std::make_unique<int64_t[]>(cpu_count);
  ctx.per_cpu_ok = std::make_unique<bool[]>(cpu_count);
  for (int i = 0; i < cpu_count; ++i) {
    const CpuLocal* const local = SmpGetCpuLocal(i);
    SELFTEST_EXPECT(local != nullptr);
    ctx.initial_ticks[i] = local->timer_ticks.load(std::memory_order_acquire);
  }

  g_ring_ipi_receivers.Clear();
  IdtRegisterHandler(kVectorSelfTestRingIpi, SelfTestRingIpiHandler);
  SmpRunOnAllCpus(IdtPerCpuAndTimerVerifyWorker, &ctx);
  IdtUnregisterHandler(kVectorSelfTestRingIpi);

  for (int i = 0; i < cpu_count; ++i) {
    const CpuLocal* const local = SmpGetCpuLocal(i);
    const int64_t final_ticks =
        local->timer_ticks.load(std::memory_order_acquire);
    SELFTEST_EXPECT(ctx.per_cpu_ok[i]);
    SELFTEST_EXPECT(final_ticks > ctx.initial_ticks[i]);
    UartWrite("[TIMER] CPU ");
    UartWriteDec(static_cast<uint64_t>(i));
    UartWrite(" (APIC ID ");
    UartWriteDec(local->apic_id);
    UartWrite("): timer_ticks=");
    UartWriteDec(static_cast<uint64_t>(final_ticks));
    UartWrite(", ipi_count=");
    UartWriteDec(static_cast<uint64_t>(
        local->ipi_count.load(std::memory_order_acquire)));
    UartWrite("\n");
  }
  return true;
}

constexpr int kWeightedTaskCount = 8;
constexpr int kHighWeightTaskCount = 4;
constexpr int kWeightedYieldIters = 16;
constexpr int kStealTaskCount = 12;
constexpr int kStealWorkerSlices = 8;
constexpr int kPreemptProgressTasks = 2;
constexpr int kStressTaskCount = 12;
constexpr int kStressRounds = 32;
constexpr int kStressChildSpawnInterval = 8;

// Most tasks one scheduler test phase creates (TestTimerPreemptionPhase()).
constexpr int kMaxPhaseTasks = kMaxSupportedCpus + kPreemptProgressTasks;

// Owns the tasks of one scheduler test phase and Join()s them all when it goes
// out of scope, so that an early `return false` never leaves a task running
// that still points into the phase's stack frame. On that path it first sets
// `*stop_flag` (if given), which tells tasks that wait for each other to stop.
class PhaseTaskGroup {
 public:
  PhaseTaskGroup(TaskScheduler& scheduler, std::atomic<bool>* const stop_flag)
      : scheduler_(scheduler), stop_flag_(stop_flag) {}

  ~PhaseTaskGroup() {
    if (count_ > 0 && stop_flag_ != nullptr) {
      stop_flag_->store(true, std::memory_order_release);
    }
    JoinAll();
  }

  PhaseTaskGroup(const PhaseTaskGroup&) = delete;
  PhaseTaskGroup& operator=(const PhaseTaskGroup&) = delete;

  // Takes ownership of `task`. Returns false if `task` is null, i.e. if
  // creating it failed.
  bool Add(Task* const task) {
    if (task == nullptr) {
      return false;
    }
    CHECK(count_ < kMaxPhaseTasks);
    tasks_[count_++] = task;
    return true;
  }

  // Joins every task added so far.
  void JoinAll() {
    for (int i = 0; i < count_; ++i) {
      scheduler_.Join(tasks_[i]);
    }
    count_ = 0;
  }

 private:
  TaskScheduler& scheduler_;
  std::atomic<bool>* const stop_flag_;
  Task* tasks_[kMaxPhaseTasks];
  int count_ = 0;
};

struct SchedulerTestContext {
  Task* bootstrap_task = nullptr;
  AtomicCpuMask128 observed_cpu_mask;
  AtomicCpuMask128 stolen_cpu_mask;
  std::atomic<int64_t> remote_steal_slices{0};
  std::atomic<int> weighted_completed{0};
  std::atomic<int> steal_completed{0};
  std::atomic<int> stress_completed{0};
  std::atomic<int64_t> stress_child_completed{0};
  std::atomic<bool> stress_failed{false};
  // Indices of the spinners that have started (not CPUs): counted their first
  // iteration (SpinnerWorker()).
  AtomicCpuMask128 spinner_started_mask;
  std::unique_ptr<std::atomic<int64_t>[]> spinner_iters;
  std::atomic<int> progress_done{0};
  std::atomic<bool> preempt_verified{false};
  std::atomic<bool> stop_spinners{false};
  CpuMask128 expected_cpu_mask = {};
};

static bool AllSpinnersStarted(const SchedulerTestContext* const ctx) {
  return ctx->spinner_started_mask.Load().ContainsAll(ctx->expected_cpu_mask);
}

struct WeightedWorkerArg {
  SchedulerTestContext* ctx = nullptr;
  int index = 0;
  std::atomic<int> slices_done{0};
  std::atomic<bool> aligned_ok{false};
  std::atomic<bool> done{false};
};

struct StealWorkerArg {
  SchedulerTestContext* ctx = nullptr;
  int index = 0;
  std::atomic<bool> done{false};
};

struct SpinnerWorkerArg {
  SchedulerTestContext* ctx = nullptr;
  int spinner_index = 0;
  std::atomic<bool> done{false};
};

struct StressWorkerArg {
  SchedulerTestContext* ctx = nullptr;
  int worker_index = 0;
  std::atomic<bool> done{false};
};

struct StressChildArg {
  SchedulerTestContext* ctx = nullptr;
  Task* parent_task = nullptr;
  uint64_t expected_token = 0;
  std::atomic<uint64_t> observed_token{0};
};

static void StressChildWorker(void* const raw_arg) {
  StressChildArg* const arg = static_cast<StressChildArg*>(raw_arg);
  if (arg == nullptr || arg->ctx == nullptr) {
    return;
  }
  SchedulerTestContext* const ctx = arg->ctx;
  TaskScheduler* const scheduler = GetTaskScheduler();
  Task* const self = scheduler->CurrentTask();
  alignas(16) volatile uint64_t child_canary[4] = {
      arg->expected_token ^ 0x1111111111111111ULL,
      arg->expected_token ^ 0x2222222222222222ULL,
      arg->expected_token ^ 0x4444444444444444ULL,
      arg->expected_token ^ 0x8888888888888888ULL,
  };
  const uintptr_t canary_addr =
      reinterpret_cast<uintptr_t>(const_cast<uint64_t*>(&child_canary[0]));
  if (self == nullptr || self == ctx->bootstrap_task ||
      self == arg->parent_task || (canary_addr & 0xFu) != 0 ||
      !AreInterruptsEnabled()) {
    ctx->stress_failed.store(true, std::memory_order_release);
    return;
  }

  for (int step = 0; step < 2; ++step) {
    const int cpu = CurrentCpuId();
    ctx->observed_cpu_mask.Add(cpu);
    scheduler->Yield();
    if (scheduler->CurrentTask() != self || !AreInterruptsEnabled()) {
      ctx->stress_failed.store(true, std::memory_order_release);
      return;
    }
  }

  if (child_canary[0] != (arg->expected_token ^ 0x1111111111111111ULL) ||
      child_canary[1] != (arg->expected_token ^ 0x2222222222222222ULL) ||
      child_canary[2] != (arg->expected_token ^ 0x4444444444444444ULL) ||
      child_canary[3] != (arg->expected_token ^ 0x8888888888888888ULL)) {
    ctx->stress_failed.store(true, std::memory_order_release);
    return;
  }

  ctx->stress_child_completed.fetch_add(1, std::memory_order_acq_rel);
  arg->observed_token.store(arg->expected_token, std::memory_order_release);
}

static void StressWorker(void* const raw_arg) {
  StressWorkerArg* const arg = static_cast<StressWorkerArg*>(raw_arg);
  if (arg == nullptr || arg->ctx == nullptr) {
    return;
  }
  SchedulerTestContext* const ctx = arg->ctx;
  TaskScheduler* const scheduler = GetTaskScheduler();
  const int cpu_count = SmpCpuCount();
  Task* const self = scheduler->CurrentTask();

  alignas(16) volatile uint64_t canaries[8] = {};
  const uintptr_t canary_addr =
      reinterpret_cast<uintptr_t>(const_cast<uint64_t*>(&canaries[0]));
  if (self == nullptr || self == ctx->bootstrap_task ||
      (canary_addr & 0xFu) != 0 || !AreInterruptsEnabled()) {
    ctx->stress_failed.store(true, std::memory_order_release);
    return;
  }

  const uint64_t seed =
      (reinterpret_cast<uintptr_t>(self) ^
       (static_cast<uint64_t>(arg->worker_index + 1) * 0x9E3779B97F4A7C15ULL));

  for (int round = 0; round < kStressRounds; ++round) {
    const uint64_t round_tag =
        seed ^ (static_cast<uint64_t>(round + 1) * 0xBF58476D1CE4E5B9ULL);
    for (int k = 0; k < 8; ++k) {
      canaries[k] = round_tag ^ (static_cast<uint64_t>(k) << 48);
    }

    const int cpu = CurrentCpuId();
    ctx->observed_cpu_mask.Add(cpu);

    // Bombard a peer CPU with a wakeup/reschedule IPI (0x21) so asynchronous
    // interrupt handlers collide frequently with active scheduler operations.
    if (cpu_count > 1) {
      const int target_cpu = (cpu + 1 + (round % (cpu_count - 1))) % cpu_count;
      SmpSendIpi(target_cpu, kVectorWakeupIpi);
    }

    // Tight back-to-back scheduler API calls without `pause` delays.
    if (scheduler->CurrentTask() != self || !AreInterruptsEnabled()) {
      ctx->stress_failed.store(true, std::memory_order_release);
      return;
    }
    scheduler->Yield();

    if (scheduler->CurrentTask() != self || !AreInterruptsEnabled()) {
      ctx->stress_failed.store(true, std::memory_order_release);
      return;
    }

    // Periodically spawn and `Join` a child task from whichever CPU (BSP or AP)
    // this worker is currently running on.
    if ((round % kStressChildSpawnInterval) == 0) {
      StressChildArg child_arg = {};
      child_arg.ctx = ctx;
      child_arg.parent_task = self;
      child_arg.expected_token = round_tag ^ 0xA5A5A5A55A5A5A5AULL;
      Task* const child = scheduler->CreateTask(StressChildWorker,  //
                                                &child_arg,         //
                                                kDefaultTaskWeight);
      if (child == nullptr) {
        ctx->stress_failed.store(true, std::memory_order_release);
        return;
      }
      scheduler->Join(child);
      if (child_arg.observed_token.load(std::memory_order_acquire) !=
              child_arg.expected_token ||
          scheduler->CurrentTask() != self || !AreInterruptsEnabled()) {
        ctx->stress_failed.store(true, std::memory_order_release);
        return;
      }
    }

    // Verify stack canaries survived all context switches, interrupts, and
    // cross-CPU migrations during this round.
    for (int k = 0; k < 8; ++k) {
      const uint64_t expected = round_tag ^ (static_cast<uint64_t>(k) << 48);
      if (canaries[k] != expected) {
        ctx->stress_failed.store(true, std::memory_order_release);
        return;
      }
    }
  }

  arg->done.store(true, std::memory_order_release);
  ctx->stress_completed.fetch_add(1, std::memory_order_acq_rel);
}

static void WeightedYieldWorker(void* const raw_arg) {
  WeightedWorkerArg* const arg = static_cast<WeightedWorkerArg*>(raw_arg);
  if (arg == nullptr || arg->ctx == nullptr) {
    return;
  }

  TaskScheduler* const scheduler = GetTaskScheduler();
  alignas(16) volatile uint64_t stack_probe = 0xA5A55A5AULL;
  const uintptr_t probe_addr =
      reinterpret_cast<uintptr_t>(const_cast<uint64_t*>(&stack_probe));
  Task* const self = scheduler->CurrentTask();
  if (self == nullptr || self == arg->ctx->bootstrap_task ||
      (probe_addr & 0xFu) != 0) {
    return;
  }
  arg->aligned_ok.store(true, std::memory_order_release);

  for (int i = 0; i < kWeightedYieldIters; ++i) {
    const int cpu = CurrentCpuId();
    arg->ctx->observed_cpu_mask.Add(cpu);
    for (int spin = 0; spin < 400; ++spin) {
      asm volatile("pause" : : : "memory");
    }
    scheduler->Yield();
    if (scheduler->CurrentTask() != self) {
      return;
    }
    arg->slices_done.fetch_add(1, std::memory_order_acq_rel);
  }

  const int cpu = CurrentCpuId();
  arg->ctx->observed_cpu_mask.Add(cpu);
  arg->done.store(true, std::memory_order_release);
  arg->ctx->weighted_completed.fetch_add(1, std::memory_order_acq_rel);
}

static void RecordStealWorkerSlice(SchedulerTestContext* const ctx) {
  const int cpu = CurrentCpuId();
  ctx->stolen_cpu_mask.Add(cpu);
  ctx->observed_cpu_mask.Add(cpu);
  if (cpu != 0) {
    ctx->remote_steal_slices.fetch_add(1, std::memory_order_acq_rel);
  }
}

static bool WorkStealingStarted(const SchedulerTestContext& ctx,
                                const TscDeadline& start_deadline) {
  if ((ctx.expected_cpu_mask.low & ~1ULL) == 0 &&
      ctx.expected_cpu_mask.high == 0) {
    return true;
  }
  const CpuMask128 stolen = ctx.stolen_cpu_mask.Load();
  return (stolen.low & ~1ULL) != 0 || stolen.high != 0 ||
         start_deadline.Expired();
}

static void StealWorker(void* const raw_arg) {
  StealWorkerArg* const arg = static_cast<StealWorkerArg*>(raw_arg);
  if (arg == nullptr || arg->ctx == nullptr) {
    return;
  }

  TaskScheduler* const scheduler = GetTaskScheduler();
  const TscDeadline start_deadline(kSelfTestWaitTimeoutTscCycles);
  for (int s = 0; s < kStealWorkerSlices; ++s) {
    RecordStealWorkerSlice(arg->ctx);
    for (int spin = 0; spin < 2000; ++spin) {
      asm volatile("pause" : : : "memory");
    }
    // Start barrier: keep tasks in CPU 0's runqueue until at least one remote
    // CPU wakes and steals a slice, or `start_deadline` expires.
    while (s == 0 && !WorkStealingStarted(*arg->ctx, start_deadline)) {
      scheduler->Yield();
      RecordStealWorkerSlice(arg->ctx);
    }
    scheduler->Yield();
  }

  RecordStealWorkerSlice(arg->ctx);
  arg->done.store(true, std::memory_order_release);
  arg->ctx->steal_completed.fetch_add(1, std::memory_order_acq_rel);
}

static void SpinnerWorker(void* const raw_arg) {
  SpinnerWorkerArg* const arg = static_cast<SpinnerWorkerArg*>(raw_arg);
  if (arg == nullptr || arg->ctx == nullptr) {
    return;
  }

  SchedulerTestContext* const ctx = arg->ctx;
  const int idx = arg->spinner_index;
  ctx->observed_cpu_mask.Add(CurrentCpuId());

  // CPU-bound spinner that NEVER calls Yield(); relies strictly on Local APIC
  // timer (0x20) preemption so PreemptProgressWorker tasks can run.
  const TscDeadline deadline(kSelfTestWaitTimeoutTscCycles);
  while (!ctx->stop_spinners.load(std::memory_order_acquire) &&
         !deadline.Expired()) {
    ctx->observed_cpu_mask.Add(CurrentCpuId());
    const int64_t iters =
        ctx->spinner_iters[idx].fetch_add(1, std::memory_order_relaxed) + 1;
    // Start barrier: a spinner counts as started only once it has counted an
    // iteration, and no spinner sets `stop_spinners` (below) before all have
    // started. A spinner held off its CPU before its first iteration (by
    // another task, or by the host descheduling its virtual CPU) thus delays
    // the stop instead of ending without an iteration; if it never spins, the
    // deadline fails the phase.
    if (iters == 1) {
      ctx->spinner_started_mask.Add(idx);
    }
    if (iters >= 64 && AllSpinnersStarted(ctx) &&
        ctx->observed_cpu_mask.Load().ContainsAll(ctx->expected_cpu_mask) &&
        ctx->progress_done.load(std::memory_order_acquire) ==
            kPreemptProgressTasks) {
      ctx->preempt_verified.store(true, std::memory_order_release);
      break;
    }
    asm volatile("pause" : : : "memory");
  }
  ctx->stop_spinners.store(true, std::memory_order_release);
  arg->done.store(true, std::memory_order_release);
}

static void PreemptProgressWorker(void* const raw_arg) {
  SchedulerTestContext* const ctx = static_cast<SchedulerTestContext*>(raw_arg);
  if (ctx == nullptr) {
    return;
  }

  TaskScheduler* const scheduler = GetTaskScheduler();
  while (!AllSpinnersStarted(ctx) &&
         !ctx->stop_spinners.load(std::memory_order_acquire)) {
    scheduler->Yield();
  }

  for (int step = 0; step < 4; ++step) {
    const int cpu = CurrentCpuId();
    ctx->observed_cpu_mask.Add(cpu);
    for (int spin = 0; spin < 4000; ++spin) {
      asm volatile("pause" : : : "memory");
    }
  }
  ctx->progress_done.fetch_add(1, std::memory_order_acq_rel);
}

// Phase 1: weighted tasks that Yield() and are Join()ed, on all CPUs.
static bool TestWeightedYieldPhase(TaskScheduler& scheduler,
                                   SchedulerTestContext* const ctx) {
  WeightedWorkerArg args[kWeightedTaskCount] = {};
  PhaseTaskGroup tasks(scheduler, nullptr);
  for (int i = 0; i < kWeightedTaskCount; ++i) {
    args[i].ctx = ctx;
    args[i].index = i;
    const int64_t weight = (i < kHighWeightTaskCount)
                               ? (kDefaultTaskWeight * 2)
                               : (kDefaultTaskWeight / 2);
    SELFTEST_EXPECT(
        tasks.Add(scheduler.CreateTask(WeightedYieldWorker, &args[i], weight)));
  }
  tasks.JoinAll();

  SELFTEST_EXPECT(ctx->weighted_completed.load(std::memory_order_acquire) ==
                  kWeightedTaskCount);
  for (int i = 0; i < kWeightedTaskCount; ++i) {
    SELFTEST_EXPECT(args[i].done.load(std::memory_order_acquire));
    SELFTEST_EXPECT(args[i].aligned_ok.load(std::memory_order_acquire));
    SELFTEST_EXPECT(args[i].slices_done.load(std::memory_order_acquire) ==
                    kWeightedYieldIters);
  }
  return true;
}

// Phase 2: cross-CPU work stealing. All tasks are enqueued on CPU 0 while
// preemption is disabled; then preemption is enabled again and the other CPUs
// are woken to steal some of them.
static bool TestWorkStealingPhase(TaskScheduler& scheduler,
                                  SchedulerTestContext* const ctx,
                                  const int cpu_count) {
  StealWorkerArg args[kStealTaskCount] = {};
  PhaseTaskGroup tasks(scheduler, nullptr);
  bool created_all = true;
  scheduler.SetPreemptEnabled(false);
  for (int i = 0; i < kStealTaskCount && created_all; ++i) {
    args[i].ctx = ctx;
    args[i].index = i;
    created_all =
        SELFTEST_OK(tasks.Add(scheduler.CreateTaskOnCpu(StealWorker,         //
                                                        &args[i],            //
                                                        kDefaultTaskWeight,  //
                                                        0)));
  }
  scheduler.SetPreemptEnabled(true);
  if (!created_all) {
    return false;
  }
  for (int c = 1; c < cpu_count; ++c) {
    SmpSendIpi(c, kVectorWakeupIpi);
  }
  tasks.JoinAll();

  SELFTEST_EXPECT(ctx->steal_completed.load(std::memory_order_acquire) ==
                  kStealTaskCount);
  for (int i = 0; i < kStealTaskCount; ++i) {
    SELFTEST_EXPECT(args[i].done.load(std::memory_order_acquire));
  }
  if (cpu_count > 1) {
    const CpuMask128 stolen = ctx->stolen_cpu_mask.Load();
    SELFTEST_EXPECT((stolen.low & ~1ULL) != 0 || stolen.high != 0);
    SELFTEST_EXPECT(ctx->remote_steal_slices.load(std::memory_order_acquire) >
                    0);
  }
  return true;
}

// Phase 3: high-frequency interrupt and cross-CPU API stress (with the LAPIC
// timers boosted by the caller). kStressTaskCount workers on all CPUs hammer
// CurrentTask() and Yield(), bombard other CPUs with wakeup IPIs (0x21), check
// stack canaries, and CreateTask() and Join() children.
static bool TestInterruptStressPhase(TaskScheduler& scheduler,
                                     SchedulerTestContext* const ctx) {
  StressWorkerArg args[kStressTaskCount] = {};
  PhaseTaskGroup tasks(scheduler, nullptr);
  for (int i = 0; i < kStressTaskCount; ++i) {
    args[i].ctx = ctx;
    args[i].worker_index = i;
    const int64_t weight =
        kDefaultTaskWeight + static_cast<int64_t>((i % 4) - 1) * 256;
    SELFTEST_EXPECT(
        tasks.Add(scheduler.CreateTask(StressWorker, &args[i], weight)));
  }
  tasks.JoinAll();

  const int64_t expected_children =
      static_cast<int64_t>(kStressTaskCount) *
      static_cast<int64_t>((kStressRounds + kStressChildSpawnInterval - 1) /
                           kStressChildSpawnInterval);
  SELFTEST_EXPECT(!ctx->stress_failed.load(std::memory_order_acquire));
  SELFTEST_EXPECT(ctx->stress_completed.load(std::memory_order_acquire) ==
                  kStressTaskCount);
  SELFTEST_EXPECT(ctx->stress_child_completed.load(std::memory_order_acquire) ==
                  expected_children);
  for (int i = 0; i < kStressTaskCount; ++i) {
    SELFTEST_EXPECT(args[i].done.load(std::memory_order_acquire));
  }
  return true;
}

// Phase 4: preemption by the LAPIC timer (0x20, boosted by the caller).
// `cpu_count` spinners that never yield occupy the CPUs; the progress tasks
// can only run, and the spinners only finish, if the timer preempts spinners.
static bool TestTimerPreemptionPhase(TaskScheduler& scheduler,
                                     SchedulerTestContext* const ctx,
                                     const int cpu_count) {
  const std::unique_ptr<SpinnerWorkerArg[]> spinner_args =
      std::make_unique<SpinnerWorkerArg[]>(cpu_count);
  // Destroyed in reverse order: on an early return, the spinners are stopped
  // and joined first, which also releases the waiting progress tasks.
  PhaseTaskGroup progress_tasks(scheduler, nullptr);
  PhaseTaskGroup spinner_tasks(scheduler, &ctx->stop_spinners);
  for (int i = 0; i < cpu_count; ++i) {
    spinner_args[i].ctx = ctx;
    spinner_args[i].spinner_index = i;
    SELFTEST_EXPECT(
        spinner_tasks.Add(scheduler.CreateTaskOnCpu(SpinnerWorker,       //
                                                    &spinner_args[i],    //
                                                    kDefaultTaskWeight,  //
                                                    i)));
  }
  for (int i = 0; i < kPreemptProgressTasks; ++i) {
    SELFTEST_EXPECT(progress_tasks.Add(
        scheduler.CreateTask(PreemptProgressWorker, ctx, kDefaultTaskWeight)));
  }
  progress_tasks.JoinAll();
  spinner_tasks.JoinAll();

  SELFTEST_EXPECT(ctx->preempt_verified.load(std::memory_order_acquire));
  SELFTEST_EXPECT(CurrentCpuId() == 0);
  SELFTEST_EXPECT(scheduler.CurrentTask() == ctx->bootstrap_task);
  for (int i = 0; i < cpu_count; ++i) {
    SELFTEST_EXPECT(spinner_args[i].done.load(std::memory_order_acquire));
    SELFTEST_EXPECT(ctx->spinner_iters[i].load(std::memory_order_acquire) > 0);
  }
  SELFTEST_EXPECT(
      ctx->observed_cpu_mask.Load().ContainsAll(ctx->expected_cpu_mask));
  return true;
}

// Set by the task of TestDetachedTaskPhase(). A global, so that the task still
// writes valid memory if it outlives a failed phase.
std::atomic<bool> g_detached_task_ran{false};

static void DetachedTaskEntry(void* const /*arg*/) {
  g_detached_task_ran.store(true, std::memory_order_release);
}

// Phase 5: a detached task that exits without being Join()ed is reclaimed by
// ReapZombies().
static bool TestDetachedTaskPhase(TaskScheduler& scheduler) {
  g_detached_task_ran.store(false, std::memory_order_release);
  Task* const detached =
      scheduler.CreateTask(DetachedTaskEntry, nullptr, kDefaultTaskWeight);
  SELFTEST_EXPECT(detached != nullptr);
  scheduler.Detach(detached);
  const TscDeadline deadline(kSelfTestWaitTimeoutTscCycles);
  while (!g_detached_task_ran.load(std::memory_order_acquire)) {
    SELFTEST_EXPECT(!deadline.Expired());
    scheduler.Yield();
  }
  int reaped = scheduler.ReapZombies();
  while (reaped == 0) {
    SELFTEST_EXPECT(!deadline.Expired());
    scheduler.Yield();
    reaped = scheduler.ReapZombies();
  }
  SELFTEST_EXPECT(reaped == 1);
  return true;
}

static bool TestTaskSchedulerSmp() {
  const int cpu_count = SmpCpuCount();
  TaskScheduler* const scheduler = GetTaskScheduler();
  SELFTEST_EXPECT(scheduler != nullptr);
  SELFTEST_EXPECT(cpu_count >= 1 && cpu_count <= kMaxSupportedCpus);
  SELFTEST_EXPECT(SmpOnlineCpuCount() == cpu_count);

  Task* const bootstrap = scheduler->CurrentTask();
  SELFTEST_EXPECT(bootstrap != nullptr);
  SELFTEST_EXPECT(CurrentCpuId() == 0);
  SELFTEST_EXPECT(scheduler->IsPreemptEnabled());

  SchedulerTestContext ctx = {};
  ctx.bootstrap_task = bootstrap;
  ctx.observed_cpu_mask.Add(0);
  ctx.expected_cpu_mask = CpuMask128::FirstCpus(cpu_count);
  ctx.spinner_iters = std::make_unique<std::atomic<int64_t>[]>(cpu_count);

  const int64_t pmm_free_before = PmmFreeFrameCount();
  const int64_t heap_free_before = HeapTotalFreeBytes();
  SELFTEST_EXPECT(pmm_free_before > 0);
  SELFTEST_EXPECT(heap_free_before > 0);

  // Every phase joins all of its tasks before it returns, also when it fails.
  TaskScheduler& sched = *scheduler;
  SELFTEST_EXPECT(TestWeightedYieldPhase(sched, &ctx));
  SELFTEST_EXPECT(TestWorkStealingPhase(sched, &ctx, cpu_count));
  {
    // Boosts the LAPIC timer of every CPU 16x for phases 3 and 4 and restores
    // all of them on every exit, when the phases have joined their tasks.
    const ScopedLapicTimerBoost timer_boost(true);
    SELFTEST_EXPECT(TestInterruptStressPhase(sched, &ctx));
    SELFTEST_EXPECT(TestTimerPreemptionPhase(sched, &ctx, cpu_count));
  }
  SELFTEST_EXPECT(TestDetachedTaskPhase(sched));

  UartWrite("[SCHED] remote_steal_slices=");
  UartWriteDec(static_cast<uint64_t>(
      ctx.remote_steal_slices.load(std::memory_order_acquire)));
  UartWrite(", progress_done=");
  UartWriteDec(
      static_cast<uint64_t>(ctx.progress_done.load(std::memory_order_acquire)));
  UartWrite(", cpu_mask=");
  UartWriteHex(ctx.observed_cpu_mask.Load().low);
  UartWrite("\n");

  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  SELFTEST_EXPECT(HeapTotalFreeBytes() == heap_free_before);
  SELFTEST_EXPECT(SmpOnlineCpuCount() == cpu_count);
  return true;
}

// A kernel virtual page that only TestTlbShootdown() maps.
constexpr uintptr_t kTlbTestPage = 0x00007E8000000000ULL;
constexpr uint64_t kTlbTestOldValue = 0x01D0F7A3E0000001ULL;
constexpr uint64_t kTlbTestNewValue = 0x2E3FB00C10000002ULL;
constexpr int kShootdownsPerTask = 200;

// Unmaps kTlbTestPage, if it is mapped, when it goes out of scope.
class ScopedTlbTestPageUnmap {
 public:
  ScopedTlbTestPageUnmap() = default;
  ~ScopedTlbTestPageUnmap() {
    if (PagingUnmapKernelPage(kTlbTestPage) != 0) {
      SmpTlbShootdown();
    }
  }

  ScopedTlbTestPageUnmap(const ScopedTlbTestPageUnmap&) = delete;
  ScopedTlbTestPageUnmap& operator=(const ScopedTlbTestPageUnmap&) = delete;
};

struct TlbTestReadContext {
  uint64_t expected = 0;
  std::atomic<int> reads{0};
  std::atomic<int> mismatches{0};
};

static void ReadTlbTestPageOnCpu(const int /*cpu_index*/, void* const raw_ctx) {
  TlbTestReadContext* const ctx = static_cast<TlbTestReadContext*>(raw_ctx);
  const uint64_t value =
      *reinterpret_cast<const volatile uint64_t*>(kTlbTestPage);
  if (value != ctx->expected) {
    ctx->mismatches.fetch_add(1, std::memory_order_relaxed);
  }
  ctx->reads.fetch_add(1, std::memory_order_release);
}

struct ShootdownStormContext {
  int cpu_count = 0;
  std::atomic<int> started{0};
  std::atomic<int> finished{0};
  // Set if not every task could be created.
  std::atomic<bool> stop{false};
};

// Runs kShootdownsPerTask TLB shootdowns, every other one with interrupts
// disabled, once the tasks on all CPUs have started.
static void ShootdownStormWorker(void* const raw_ctx) {
  ShootdownStormContext* const ctx =
      static_cast<ShootdownStormContext*>(raw_ctx);
  ctx->started.fetch_add(1, std::memory_order_acq_rel);
  while (ctx->started.load(std::memory_order_acquire) < ctx->cpu_count) {
    if (ctx->stop.load(std::memory_order_acquire)) {
      return;
    }
    asm volatile("pause" : : : "memory");
  }
  for (int i = 0; i < kShootdownsPerTask; ++i) {
    if ((i & 1) == 0) {
      SmpTlbShootdown();
    } else {
      const ScopedIrqDisable irq_disable;
      SmpTlbShootdown();
    }
  }
  ctx->finished.fetch_add(1, std::memory_order_acq_rel);
}

// Checks that SmpTlbShootdown() reaches every CPU: every CPU reads a kernel
// virtual page, which caches its translation, then the page is mapped to
// another frame, and after a shootdown every CPU must read the new frame.
// Then a task on each CPU runs shootdowns at the same time as the others,
// half of them with interrupts disabled, and all of them must complete.
static bool TestTlbShootdown() {
  const int cpu_count = SmpCpuCount();
  TaskScheduler* const scheduler = GetTaskScheduler();
  SELFTEST_EXPECT(scheduler != nullptr);
  // The page tables of the test page stay allocated for good.
  SELFTEST_EXPECT(PagingReserveKernelRange(kTlbTestPage, kPageSize));
  const int64_t pmm_free_before = PmmFreeFrameCount();
  {
    const ScopedPmmFrames old_frame(PmmAllocFrame(), 1);
    const ScopedPmmFrames new_frame(PmmAllocFrame(), 1);
    SELFTEST_EXPECT(old_frame.get() != 0 && new_frame.get() != 0);
    *reinterpret_cast<volatile uint64_t*>(old_frame.get()) = kTlbTestOldValue;
    *reinterpret_cast<volatile uint64_t*>(new_frame.get()) = kTlbTestNewValue;

    // Destroyed first, so the page is unmapped before its frames are freed.
    const ScopedTlbTestPageUnmap unmap_test_page;
    SELFTEST_EXPECT(PagingMapKernelPage(kTlbTestPage, old_frame.get()));
    SELFTEST_EXPECT(!PagingMapKernelPage(kTlbTestPage, new_frame.get()));
    SELFTEST_EXPECT(PagingKernelPageFrame(kTlbTestPage) == old_frame.get());
    TlbTestReadContext old_reads;
    old_reads.expected = kTlbTestOldValue;
    SmpRunOnAllCpus(ReadTlbTestPageOnCpu, &old_reads);
    SELFTEST_EXPECT(old_reads.reads.load(std::memory_order_acquire) ==
                    cpu_count);
    SELFTEST_EXPECT(old_reads.mismatches.load(std::memory_order_relaxed) == 0);

    SELFTEST_EXPECT(PagingUnmapKernelPage(kTlbTestPage) == old_frame.get());
    SELFTEST_EXPECT(PagingMapKernelPage(kTlbTestPage, new_frame.get()));
    SmpTlbShootdown();
    TlbTestReadContext new_reads;
    new_reads.expected = kTlbTestNewValue;
    SmpRunOnAllCpus(ReadTlbTestPageOnCpu, &new_reads);
    SELFTEST_EXPECT(new_reads.reads.load(std::memory_order_acquire) ==
                    cpu_count);
    SELFTEST_EXPECT(new_reads.mismatches.load(std::memory_order_relaxed) == 0);
  }
  SELFTEST_EXPECT(PagingKernelPageFrame(kTlbTestPage) == 0);
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);

  ShootdownStormContext storm;
  storm.cpu_count = cpu_count;
  {
    PhaseTaskGroup tasks(*scheduler, &storm.stop);
    for (int cpu = 0; cpu < cpu_count; ++cpu) {
      Task* const task = scheduler->CreateTaskOnCpu(ShootdownStormWorker,  //
                                                    &storm,                //
                                                    kDefaultTaskWeight,    //
                                                    cpu);
      SELFTEST_EXPECT(tasks.Add(task));
    }
    tasks.JoinAll();
  }
  SELFTEST_EXPECT(storm.finished.load(std::memory_order_acquire) == cpu_count);
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  return true;
}

struct GuardedStackProbeContext {
  std::atomic<int> probes{0};
  std::atomic<int> failures{0};
};

// Returns true if `addr` lies in the stack of a task stack slot whose stack
// pages are mapped and whose guard page is not.
static bool IsOnGuardedTaskStack(const uintptr_t addr) {
  const uintptr_t area_size =
      static_cast<uintptr_t>(kTaskStackSlotCount * kTaskStackSlotSize);
  if (addr < kTaskStackAreaBase || addr - kTaskStackAreaBase >= area_size) {
    return false;
  }
  const uintptr_t slot_size = static_cast<uintptr_t>(kTaskStackSlotSize);
  const uintptr_t guard_page =
      kTaskStackAreaBase + (addr - kTaskStackAreaBase) / slot_size * slot_size;
  const uintptr_t stack_base = guard_page + kPageSize;
  if (addr < stack_base || PagingKernelPageFrame(guard_page) != 0) {
    return false;
  }
  for (int page = 0; page < kTaskStackFrames; ++page) {
    const uintptr_t page_addr =
        stack_base + static_cast<uintptr_t>(page * kPageSize);
    if (PagingKernelPageFrame(page_addr) == 0) {
      return false;
    }
  }
  return true;
}

static void GuardedStackProbeWorker(void* const raw_ctx) {
  GuardedStackProbeContext* const ctx =
      static_cast<GuardedStackProbeContext*>(raw_ctx);
  volatile uint64_t local = 0;
  if (!IsOnGuardedTaskStack(reinterpret_cast<uintptr_t>(&local))) {
    ctx->failures.fetch_add(1, std::memory_order_relaxed);
  }
  ctx->probes.fetch_add(1, std::memory_order_release);
}

// Checks the task stacks: TaskStackAlloc() maps a stack in the task stack
// area directly above an unmapped guard page, TaskStackFree() unmaps it and
// returns its frames at once, and a task on each CPU runs on such a stack.
static bool TestTaskStackGuardPages() {
  const int cpu_count = SmpCpuCount();
  TaskScheduler* const scheduler = GetTaskScheduler();
  SELFTEST_EXPECT(scheduler != nullptr);
  const int64_t pmm_free_before = PmmFreeFrameCount();

  const uintptr_t stack = TaskStackAlloc();
  SELFTEST_EXPECT(stack != 0);
  bool stack_ok =
      SELFTEST_OK(IsOnGuardedTaskStack(stack)) &&
      SELFTEST_OK(IsOnGuardedTaskStack(stack + kTaskStackSize - 1)) &&
      SELFTEST_OK(PagingKernelPageFrame(stack - kPageSize) == 0) &&
      SELFTEST_OK(PagingKernelPageFrame(stack + kTaskStackSize) == 0) &&
      SELFTEST_OK(PmmFreeFrameCount() == pmm_free_before - kTaskStackFrames);
  for (int page = 0; stack_ok && page < kTaskStackFrames; ++page) {
    // Each stack page is a PMM frame that the identity map also reaches.
    const uintptr_t page_addr =
        stack + static_cast<uintptr_t>(page * kPageSize);
    const uintptr_t frame = PagingKernelPageFrame(page_addr);
    const uint64_t pattern = 0x57AC000000000000ULL | page_addr;
    *reinterpret_cast<volatile uint64_t*>(page_addr + kPageSize - 8) = pattern;
    stack_ok = SELFTEST_OK(PmmRangeIsValidUsableRam(frame, kPageSize)) &&
               SELFTEST_OK(*reinterpret_cast<const volatile uint64_t*>(
                               frame + kPageSize - 8) == pattern);
  }
  TaskStackFree(stack);
  SELFTEST_EXPECT(stack_ok);
  for (int page = 0; page < kTaskStackFrames; ++page) {
    SELFTEST_EXPECT(PagingKernelPageFrame(
                        stack + static_cast<uintptr_t>(page * kPageSize)) == 0);
  }
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);

  GuardedStackProbeContext probe;
  {
    PhaseTaskGroup tasks(*scheduler, nullptr);
    for (int cpu = 0; cpu < cpu_count; ++cpu) {
      Task* const task = scheduler->CreateTaskOnCpu(GuardedStackProbeWorker,  //
                                                    &probe,                   //
                                                    kDefaultTaskWeight,       //
                                                    cpu);
      SELFTEST_EXPECT(tasks.Add(task));
    }
    tasks.JoinAll();
  }
  SELFTEST_EXPECT(probe.probes.load(std::memory_order_acquire) == cpu_count);
  SELFTEST_EXPECT(probe.failures.load(std::memory_order_relaxed) == 0);
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  return true;
}

// Contents of the user pages in the address space tests.
constexpr uint64_t kAddressSpaceTestPattern = 0x05E5A11ED0000000ULL;

// Checks AddressSpace on the calling CPU: it maps new zeroed pages at
// page-aligned user addresses only, in page tables of its own, which map the
// kernel like the kernel's page tables do. While the calling task runs in it,
// its pages appear at their user addresses and kernel memory stays reachable.
// The kernel's page tables never map a user page, and destroying the address
// space returns every frame.
static bool TestAddressSpaces() {
  TaskScheduler* const scheduler = GetTaskScheduler();
  SELFTEST_EXPECT(scheduler != nullptr);
  SELFTEST_EXPECT(PagingActiveRoot() == PagingKernelRoot());
  const int64_t pmm_free_before = PmmFreeFrameCount();
  {
    const std::unique_ptr<AddressSpace> space = AddressSpace::Create();
    SELFTEST_EXPECT(space != nullptr);
    SELFTEST_EXPECT(space->root() != PagingKernelRoot());

    constexpr uintptr_t kFirstPage = kUserSpaceBase;
    constexpr uintptr_t kLastPage = kUserSpaceEnd - kPageSize;
    // In another page table than `kFirstPage`.
    constexpr uintptr_t kCodePage = kUserSpaceBase + kHugePageSize;
    SELFTEST_EXPECT(space->MapNewPage(kFirstPage + 8, UserPageAccess::kRead) ==
                    0);
    SELFTEST_EXPECT(space->MapNewPage(kUserSpaceEnd, UserPageAccess::kRead) ==
                    0);
    SELFTEST_EXPECT(
        space->MapNewPage(kKernelVirtualBase, UserPageAccess::kRead) == 0);
    SELFTEST_EXPECT(space->MapNewPage(0, UserPageAccess::kRead) == 0);
    SELFTEST_EXPECT(space->PageFrame(kFirstPage) == 0);

    const uintptr_t first =
        space->MapNewPage(kFirstPage, UserPageAccess::kReadWrite);
    const uintptr_t last = space->MapNewPage(kLastPage, UserPageAccess::kRead);
    const uintptr_t code =
        space->MapNewPage(kCodePage, UserPageAccess::kReadExecute);
    SELFTEST_EXPECT(first != 0 && last != 0 && code != 0);
    SELFTEST_EXPECT(space->MapNewPage(kFirstPage, UserPageAccess::kRead) == 0);
    SELFTEST_EXPECT(space->PageFrame(kFirstPage) == first);
    SELFTEST_EXPECT(space->PageFrame(kLastPage) == last);
    SELFTEST_EXPECT(space->PageFrame(kCodePage) == code);
    SELFTEST_EXPECT(space->PageFrame(kFirstPage + kPageSize) == 0);

    // New pages are zeroed. The kernel fills them through the identity map.
    const volatile uint64_t* const first_words =
        reinterpret_cast<const volatile uint64_t*>(first);
    bool zeroed = true;
    for (int64_t i = 0; i < kPageSize / 8; ++i) {
      zeroed = zeroed && first_words[i] == 0;
    }
    SELFTEST_EXPECT(zeroed);
    *reinterpret_cast<volatile uint64_t*>(first) = kAddressSpaceTestPattern;
    *reinterpret_cast<volatile uint64_t*>(last + kPageSize - 8) =
        kAddressSpaceTestPattern + 1;
    SELFTEST_EXPECT(PagingUserSpaceIsEmpty());

    // No check may return between the two SetAddressSpace() calls: the
    // address space must not be destroyed while this task runs in it.
    scheduler->SetAddressSpace(space.get());
    const uintptr_t active_root = PagingActiveRoot();
    const bool user_space_empty = PagingUserSpaceIsEmpty();
    // Through the user addresses, the way the kernel reaches system call
    // buffers: with SMAP, a plain access to a user page faults
    // (user_access.h).
    constexpr int64_t kWordSize = sizeof(uint64_t);
    uint64_t first_word = 0;
    uint64_t last_word = 0;
    const bool read_user_words =
        CopyFromUser(&first_word, kFirstPage, kWordSize) &&
        CopyFromUser(&last_word, kLastPage + kPageSize - kWordSize, kWordSize);
    const uint64_t written_word = kAddressSpaceTestPattern + 2;
    const bool wrote_user_word =
        CopyToUser(kFirstPage + kWordSize, &written_word, kWordSize);
    void* const heap_block = Kmalloc(64);
    if (heap_block != nullptr) {
      *static_cast<volatile uint64_t*>(heap_block) = kAddressSpaceTestPattern;
      Kfree(heap_block);
    }
    scheduler->SetAddressSpace(nullptr);

    SELFTEST_EXPECT(active_root == space->root());
    SELFTEST_EXPECT(!user_space_empty);
    SELFTEST_EXPECT(read_user_words);
    SELFTEST_EXPECT(first_word == kAddressSpaceTestPattern);
    SELFTEST_EXPECT(last_word == kAddressSpaceTestPattern + 1);
    // The write reached the first page's frame.
    SELFTEST_EXPECT(wrote_user_word);
    SELFTEST_EXPECT(first_words[1] == written_word);
    SELFTEST_EXPECT(heap_block != nullptr);
    SELFTEST_EXPECT(PagingActiveRoot() == PagingKernelRoot());
    SELFTEST_EXPECT(PagingUserSpaceIsEmpty());
  }
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  return true;
}

// The number of slices that each task of TestAddressSpaceSwitching() runs.
constexpr int kAddressSpaceSwitchingSlices = 64;

struct AddressSpaceSwitchingArg {
  AddressSpace* space = nullptr;
  // What the task expects at `kUserSpaceBase` in `space`.
  uint64_t expected = 0;
  // Whether the task leaves `space` before it returns, rather than exiting in
  // it.
  bool leave_before_exit = false;
  AtomicCpuMask128* cpus = nullptr;
  std::atomic<int>* mismatches = nullptr;
  // Every CPU: each task starts its second slice only once each of them has
  // run one of the tasks, or once `start_deadline` has expired.
  CpuMask128 all_cpus;
  const TscDeadline* start_deadline = nullptr;
};

// Records the calling CPU, and counts a mismatch unless the calling task runs
// in its own page tables, which map its page at `kUserSpaceBase`: whichever
// CPU runs the task, it must use the task's page tables.
static void CheckAddressSpaceSwitchingSlice(
    const AddressSpaceSwitchingArg& arg) {
  arg.cpus->Add(CurrentCpuId());
  uint64_t word = 0;
  const bool copied = CopyFromUser(&word, kUserSpaceBase, sizeof(word));
  if (PagingActiveRoot() != arg.space->root() || !copied ||
      word != arg.expected) {
    arg.mismatches->fetch_add(1, std::memory_order_relaxed);
  }
}

// Returns true once every CPU has run one of the tasks, or once the tasks
// have waited for that for kSelfTestWaitTimeoutTscCycles.
static bool AddressSpaceSwitchingStarted(const AddressSpaceSwitchingArg& arg) {
  return arg.cpus->Load().ContainsAll(arg.all_cpus) ||
         arg.start_deadline->Expired();
}

static void AddressSpaceSwitchingWorker(void* const raw_arg) {
  const AddressSpaceSwitchingArg* const arg =
      static_cast<const AddressSpaceSwitchingArg*>(raw_arg);
  TaskScheduler* const scheduler = GetTaskScheduler();
  scheduler->SetAddressSpace(arg->space);
  for (int slice = 0; slice < kAddressSpaceSwitchingSlices; ++slice) {
    CheckAddressSpaceSwitchingSlice(*arg);
    // Start barrier. The whole run can take less time than a halted CPU needs
    // to wake up (a virtual CPU that the host doesn't run at once), and a CPU
    // whose queue is empty takes the tasks that still wait on other CPUs
    // (work_stealing_scheduler.cpp), so without it a CPU could find its tasks
    // gone, or all of them done, by the time it wakes up. Waiting, yielding,
    // until every CPU has run a task keeps the tasks there for the CPUs that
    // come late; each time the task resumes, it checks its page tables again.
    while (slice == 0 && !AddressSpaceSwitchingStarted(*arg)) {
      scheduler->Yield();
      CheckAddressSpaceSwitchingSlice(*arg);
    }
    scheduler->Yield();
  }
  if (arg->leave_before_exit) {
    scheduler->SetAddressSpace(nullptr);
    if (PagingActiveRoot() != PagingKernelRoot()) {
      arg->mismatches->fetch_add(1, std::memory_order_relaxed);
    }
  }
}

// Checks that the page tables of each task follow it: tasks in address spaces
// of their own, which all map a page at `kUserSpaceBase` to a frame of their
// own, share the CPUs and move between them, and each must always find its own
// page there. Half of the tasks leave their address space before they exit,
// the others exit in it. Afterwards the bootstrap task still uses the kernel's
// page tables, and destroying the address spaces returns every frame.
static bool TestAddressSpaceSwitching() {
  const int cpu_count = SmpCpuCount();
  TaskScheduler* const scheduler = GetTaskScheduler();
  SELFTEST_EXPECT(scheduler != nullptr);
  const int64_t pmm_free_before = PmmFreeFrameCount();
  const int task_count =
      (2 * cpu_count <= kMaxPhaseTasks) ? 2 * cpu_count : kMaxPhaseTasks;
  std::atomic<int> mismatches{0};
  AtomicCpuMask128 cpus;
  {
    // Declared before `tasks`, so that the tasks are joined before their
    // address spaces are destroyed, also when a check below fails.
    const std::unique_ptr<std::unique_ptr<AddressSpace>[]> spaces(
        new std::unique_ptr<AddressSpace>[task_count]);
    const std::unique_ptr<AddressSpaceSwitchingArg[]> args(
        new AddressSpaceSwitchingArg[task_count]);
    const TscDeadline start_deadline(kSelfTestWaitTimeoutTscCycles);
    for (int i = 0; i < task_count; ++i) {
      spaces[i] = AddressSpace::Create();
      SELFTEST_EXPECT(spaces[i] != nullptr);
      const uintptr_t frame =
          spaces[i]->MapNewPage(kUserSpaceBase, UserPageAccess::kReadWrite);
      SELFTEST_EXPECT(frame != 0);
      AddressSpaceSwitchingArg& arg = args[i];
      arg.space = spaces[i].get();
      arg.expected = kAddressSpaceTestPattern + static_cast<uint64_t>(i);
      arg.leave_before_exit = (i % 2) == 0;
      arg.cpus = &cpus;
      arg.mismatches = &mismatches;
      arg.all_cpus = CpuMask128::FirstCpus(cpu_count);
      arg.start_deadline = &start_deadline;
      *reinterpret_cast<volatile uint64_t*>(frame) = arg.expected;
    }
    PhaseTaskGroup tasks(*scheduler, nullptr);
    for (int i = 0; i < task_count; ++i) {
      Task* const task =
          scheduler->CreateTaskOnCpu(AddressSpaceSwitchingWorker,  //
                                     &args[i],                     //
                                     kDefaultTaskWeight,           //
                                     i % cpu_count);
      SELFTEST_EXPECT(tasks.Add(task));
    }
    tasks.JoinAll();
  }
  const CpuMask128 used = cpus.Load();
  int cpus_used = 0;
  for (int cpu = 0; cpu < cpu_count; ++cpu) {
    cpus_used += used.Contains(cpu) ? 1 : 0;
  }
  SELFTEST_EXPECT(mismatches.load(std::memory_order_relaxed) == 0);
  SELFTEST_EXPECT(cpus_used >= ((cpu_count > 1) ? 2 : 1));
  SELFTEST_EXPECT(PagingActiveRoot() == PagingKernelRoot());
  SELFTEST_EXPECT(PagingUserSpaceIsEmpty());
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  return true;
}

// Page-fault error code bits (Intel SDM Vol. 3A, 4.7).
constexpr uint64_t kPageFaultPresent = 1 << 0;
constexpr uint64_t kPageFaultWrite = 1 << 1;
constexpr uint64_t kPageFaultUser = 1 << 2;
constexpr uint64_t kPageFaultInstructionFetch = 1 << 4;
// The bits that tell how the faulting access went: present page, write, user
// mode.
constexpr uint64_t kPageFaultAccessBits =
    kPageFaultPresent | kPageFaultWrite | kPageFaultUser;
constexpr uint64_t kPageFaultUserFetch =
    kPageFaultInstructionFetch | kPageFaultUser;

// `usertest::Mode::kKernelInterrupt` raises a vector whose gate has DPL 0.
static_assert(usertest::kKernelOnlyVector == kVectorWakeupIpi);

// How WaitForProcesses() lets the processes run.
enum class WaitMode {
  // Yields, so that processes that wait for CPU 0 run too.
  kYield,
  // Spins without entering the scheduler. CPU 0, whose queue is empty, would
  // otherwise take a process that has just been queued on another CPU, which
  // has not picked it up yet (work_stealing_scheduler.cpp), so the process
  // would share CPU 0 with this task instead of having that CPU to itself.
  kSpin,
};

// Waits until `done(stats)` holds for the `stats()` of each of the first
// `count` `processes`. Fails if that takes longer than
// kSelfTestWaitTimeoutTscCycles, with a FAIL line right away: the caller's
// destructors then kill the processes and wait for them, which never ends if
// a kill gets lost.
template <typename Done>
static bool WaitForProcesses(
    const std::unique_ptr<UserProcess>* const processes,  //
    const int count,                                      //
    const Done done,                                      //
    const WaitMode mode) {
  TaskScheduler* const scheduler = GetTaskScheduler();
  const TscDeadline deadline(kSelfTestWaitTimeoutTscCycles);
  for (;;) {
    bool all_done = true;
    for (int i = 0; i < count && all_done; ++i) {
      all_done = done(processes[i]->stats());
    }
    if (all_done) {
      return true;
    }
    if (!SELFTEST_OK(!deadline.Expired())) {
      UartWrite("[TEST] wait for processes timed out: FAIL\n");
      return false;
    }
    if (mode == WaitMode::kYield) {
      scheduler->Yield();
    } else {
      asm volatile("pause" : : : "memory");
    }
  }
}

// Returns true once the process has ended and freed the program's memory.
static bool HasEnded(const UserProcessStats& stats) {
  return stats.end_tick >= 0;
}

// Runs user/usertest.cpp in `mode` in a process of its own until it ends, and
// stores how it ended in `*result`. Returns false if the process can't be
// started, or doesn't end in time (instead of hanging in Wait()).
static bool RunUserTest(const usertest::Mode mode,
                        UserProgramResult* const result) {
  const std::unique_ptr<UserProcess> process =
      UserProcess::Start(UserTestProgram(), usertest::Arg(mode));
  if (process == nullptr) {
    return false;
  }
  SELFTEST_EXPECT(WaitForProcesses(&process, 1, HasEnded, WaitMode::kYield));
  *result = process->Wait();
  return true;
}

// Logs `result` over UART as the result of a user program that ended
// otherwise than expected.
static void LogUnexpectedResult(const UserProgramResult& result) {
  UartWrite("[TEST] unexpected user program result: ");
  WriteUserProgramResult(result, UartWrite);
  UartWrite("\n");
}

// Returns true if `result` says that the program exited with `status`, and
// logs the result otherwise.
static bool ExitedWith(const UserProgramResult& result, const int64_t status) {
  if (result.outcome == UserProgramOutcome::kExited &&
      result.exit_status == status) {
    return true;
  }
  LogUnexpectedResult(result);
  return false;
}

// Returns true if `result` says that CPU exception `vector` killed the
// program with an error code whose `error_code_mask` bits are
// `error_code_bits`, and logs the result otherwise.
static bool FaultedWith(const UserProgramResult& result,  //
                        const int vector,                 //
                        const uint64_t error_code_mask,   //
                        const uint64_t error_code_bits) {
  if (result.outcome == UserProgramOutcome::kFaulted &&
      result.fault_vector == vector &&
      (result.fault_error_code & error_code_mask) == error_code_bits &&
      IsUserRange(result.fault_rip, 1)) {
    return true;
  }
  LogUnexpectedResult(result);
  return false;
}

static bool TestUserModeConsoleRead();
static bool TestUserModeSpawnAndWait();
static bool TestUserModeInteractiveShell();

// Checks user processes with user/usertest.cpp, user/init.cpp, and
// user/sh.cpp: the exit status reaches the kernel, system calls work and
// reject bad arguments, console read() and process spawn()/wait() work and
// clean up children across exit/fault/kill, the interactive shell runs
// built-in commands and child programs, and each kind of fault kills the
// program that raised it, reported with the right vector, error code and
// address, while the kernel carries on. An invalid executable does not load.
// Afterwards the kernel's page tables are active again and every frame and
// heap byte is back.
static bool TestUserModePrograms() {
  using usertest::Mode;
  SELFTEST_EXPECT(GetTaskScheduler() != nullptr);
  const int64_t pmm_free_at_start = PmmFreeFrameCount();
  const int64_t heap_free_at_start = HeapTotalFreeBytes();
  // Warm up the kernel heap for multi-CPU console read and multi-process child
  // spawning before recording the exact frame and heap baseline.
  SELFTEST_EXPECT(TestUserModeConsoleRead());
  SELFTEST_EXPECT(TestUserModeSpawnAndWait());
  SELFTEST_EXPECT(TestUserModeInteractiveShell());
  const int64_t pmm_free_before = PmmFreeFrameCount();
  const int64_t heap_free_before = HeapTotalFreeBytes();
  const bool heap_grew = pmm_free_before < pmm_free_at_start &&
                         heap_free_before > heap_free_at_start;
  if (!heap_grew) {
    SELFTEST_EXPECT(pmm_free_before == pmm_free_at_start);
    SELFTEST_EXPECT(heap_free_before == heap_free_at_start);
  }
  UserProgramResult result;

  SELFTEST_EXPECT(RunUserTest(Mode::kExit, &result));
  SELFTEST_EXPECT(ExitedWith(result, usertest::kExitStatus));
  SELFTEST_EXPECT(RunUserTest(Mode::kSyscalls, &result));
  SELFTEST_EXPECT(ExitedWith(result, 0));

  SELFTEST_EXPECT(RunUserTest(Mode::kPrivilegedInstruction, &result));
  SELFTEST_EXPECT(FaultedWith(result, kVectorGeneralProtection, ~0ULL, 0));
  // Kernel memory is mapped, but not for user code.
  SELFTEST_EXPECT(RunUserTest(Mode::kReadKernelMemory, &result));
  SELFTEST_EXPECT(FaultedWith(result,                //
                              kVectorPageFault,      //
                              kPageFaultAccessBits,  //
                              kPageFaultPresent | kPageFaultUser));
  SELFTEST_EXPECT(result.fault_address == usertest::kKernelAddress);
  SELFTEST_EXPECT(RunUserTest(Mode::kWriteReadOnlyData, &result));
  SELFTEST_EXPECT(FaultedWith(result,                //
                              kVectorPageFault,      //
                              kPageFaultAccessBits,  //
                              kPageFaultAccessBits));
  SELFTEST_EXPECT(IsUserRange(result.fault_address, 8));
  SELFTEST_EXPECT(RunUserTest(Mode::kExecuteData, &result));
  if (PagingNoExecuteEnabled()) {
    SELFTEST_EXPECT(FaultedWith(result,               //
                                kVectorPageFault,     //
                                kPageFaultUserFetch,  //
                                kPageFaultUserFetch));
    SELFTEST_EXPECT(result.fault_address == result.fault_rip);
  } else {
    SELFTEST_EXPECT(ExitedWith(result, 0));
  }
  SELFTEST_EXPECT(RunUserTest(Mode::kInvalidOpcode, &result));
  SELFTEST_EXPECT(FaultedWith(result, kVectorInvalidOpcode, 0, 0));
  // The error code names the gate that user code may not use: its vector and
  // the IDT bit. (QEMU 7.2, whose TCG the Docker image runs, reports the
  // gate's offset in the IDT instead of its vector.)
  SELFTEST_EXPECT(RunUserTest(Mode::kKernelInterrupt, &result));
  SELFTEST_EXPECT(FaultedWith(result, kVectorGeneralProtection, 0, 0));
  constexpr uint64_t kKernelOnlyVector = usertest::kKernelOnlyVector;
  SELFTEST_EXPECT(result.fault_error_code == ((kKernelOnlyVector << 3) | 2) ||
                  result.fault_error_code == ((kKernelOnlyVector << 4) | 2));
  // A stack overflow faults in the unmapped page below the user stack.
  SELFTEST_EXPECT(RunUserTest(Mode::kStackOverflow, &result));
  SELFTEST_EXPECT(FaultedWith(result,                              //
                              kVectorPageFault,                    //
                              kPageFaultPresent | kPageFaultUser,  //
                              kPageFaultUser));
  constexpr uintptr_t kStackGuardPage =
      kUserStackTop - (kUserStackPages + 1) * kPageSize;
  SELFTEST_EXPECT(result.fault_address >= kStackGuardPage &&
                  result.fault_address < kStackGuardPage + kPageSize);

  // Not an ELF file.
  static const uint8_t kNotAnExecutable[64] = {0x7F, 'E', 'L', 'F'};
  {
    const std::unique_ptr<UserProcess> process = UserProcess::Start(
        UserProgram{kNotAnExecutable, sizeof(kNotAnExecutable)}, 0);
    SELFTEST_EXPECT(process != nullptr);
    result = process->Wait();
    SELFTEST_EXPECT(result.outcome == UserProgramOutcome::kLoadFailed);
    SELFTEST_EXPECT(process->syscall_count() == 0);
  }

  SELFTEST_EXPECT(TestUserModeConsoleRead());
  SELFTEST_EXPECT(TestUserModeSpawnAndWait());
  SELFTEST_EXPECT(TestUserModeInteractiveShell());

  SELFTEST_EXPECT(PagingActiveRoot() == PagingKernelRoot());
  SELFTEST_EXPECT(PagingUserSpaceIsEmpty());
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  // Including what the programs that faulted and spawned children had: their
  // FpuState, address space and task.
  SELFTEST_EXPECT(HeapTotalFreeBytes() == heap_free_before);
  return true;
}

// Heap-allocated arrays of processes, which may hold up to 3 × CPUs processes
// (too many for the 16 KiB boot stack on machines with many CPUs).
using ProcessArray = std::unique_ptr<std::unique_ptr<UserProcess>[]>;

static ProcessArray MakeProcessArray(const int count) {
  return std::make_unique<std::unique_ptr<UserProcess>[]>(count);
}

// System calls that each spinner of TestUserModePreemption() makes before the
// spinners are killed: one per round of computation.
constexpr int64_t kUserSpinnerSyscalls = 8;

// Checks that the timer preempts user code: twice as many processes as CPUs
// compute without ever yielding or ending, so all of them can make progress
// only if the CPUs switch between them. Each checks its registers, stack and
// data in every round, and they run on several CPUs. Then they are killed;
// none of them must have ended otherwise.
static bool TestUserModePreemption() {
  TaskScheduler* const scheduler = GetTaskScheduler();
  SELFTEST_EXPECT(scheduler != nullptr);
  const int cpu_count = SmpCpuCount();
  const int process_count = 2 * cpu_count;
  const int64_t pmm_free_before = PmmFreeFrameCount();
  AtomicCpuMask128 cpus;
  {
    // On an early return, the destructors kill the processes and wait for
    // them.
    const ProcessArray processes = MakeProcessArray(process_count);
    for (int i = 0; i < process_count; ++i) {
      processes[i] = UserProcess::Start(
          UserTestProgram(),
          usertest::Arg(usertest::Mode::kSpin,
                        0x5EED0000 + static_cast<uint64_t>(i)));
      SELFTEST_EXPECT(processes[i] != nullptr);
    }
    // Without preemption, this loop would never end: the CPUs would never
    // switch away from the first spinners, or back to this task.
    const TscDeadline deadline(kSelfTestWaitTimeoutTscCycles);
    for (bool done = false; !done;) {
      done = true;
      for (int i = 0; i < process_count; ++i) {
        const int cpu = processes[i]->last_syscall_cpu();
        if (cpu >= 0) {
          cpus.Add(cpu);
        }
        done = done && processes[i]->syscall_count() >= kUserSpinnerSyscalls;
      }
      SELFTEST_EXPECT(done || !deadline.Expired());
      scheduler->Yield();
    }
    for (int i = 0; i < process_count; ++i) {
      processes[i]->Kill();
    }
    // Not in Wait() right away, which would never return if a kill got lost.
    SELFTEST_EXPECT(WaitForProcesses(processes.get(),  //
                                     process_count,    //
                                     HasEnded,         //
                                     WaitMode::kYield));
    for (int i = 0; i < process_count; ++i) {
      const UserProgramResult result = processes[i]->Wait();
      if (result.outcome != UserProgramOutcome::kKilled) {
        LogUnexpectedResult(result);
      }
      SELFTEST_EXPECT(result.outcome == UserProgramOutcome::kKilled);
    }
  }
  const CpuMask128 used = cpus.Load();
  int cpus_used = 0;
  for (int cpu = 0; cpu < cpu_count; ++cpu) {
    cpus_used += used.Contains(cpu) ? 1 : 0;
  }
  SELFTEST_EXPECT(cpus_used >= ((cpu_count > 1) ? 2 : 1));
  SELFTEST_EXPECT(PagingActiveRoot() == PagingKernelRoot());
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  return true;
}

// Returns true once the program has run user code: it has made a system call,
// or a timer interrupt has interrupted its user code.
static bool HasRunUserCode(const UserProcessStats& stats) {
  return stats.syscalls >= 1 || stats.user_timer_ticks >= 1;
}

// Returns true once timer interrupts have interrupted the program's user code
// twice.
static bool HasRunInUserMode(const UserProcessStats& stats) {
  return stats.user_timer_ticks >= 2;
}

// Returns true while the program is in the middle of a system call.
static bool IsInSyscall(const UserProcessStats& stats) {
  return stats.in_syscall;
}

// Starts `count` processes that run user/usertest.cpp with the arguments
// `arg_of(0)` to `arg_of(count - 1)` as `processes[0..count-1]`, each on a CPU
// of its own if there are no more of them than CPUs other than CPU 0, which
// runs this task. Each one starts only once the one before has run user code
// (WaitMode::kSpin). Started back to back, two of them could share a CPU: the
// scheduler picks the least loaded CPU for a new task before it queues the
// task there (work_stealing_scheduler.cpp), and in between an idle CPU, woken
// by its timer or by the TLB shootdown that allocating the task's stack may
// start (task_stack.cpp), may take a process that still waits on the CPU that
// it was queued on, even if the idle CPU is the one picked for the new task.
// One by one, every earlier process is running, so none can be taken, and
// every CPU without a process is idle with nothing queued: the new process
// goes to such a CPU, and only such a CPU may take it from there (a busy CPU
// takes a task only from a much busier one). Precondition: no other tasks are
// runnable, so that every CPU but CPU 0 starts idle (as every boot self-test
// leaves them).
template <typename ArgOf>
static bool StartProcessesOnOwnCpus(
    std::unique_ptr<UserProcess>* const processes,  //
    const int count,                                //
    const ArgOf arg_of) {
  for (int i = 0; i < count; ++i) {
    processes[i] = UserProcess::Start(UserTestProgram(), arg_of(i));
    SELFTEST_EXPECT(processes[i] != nullptr);
    SELFTEST_EXPECT(
        WaitForProcesses(&processes[i], 1, HasRunUserCode, WaitMode::kSpin));
  }
  return true;
}

// Returns the larger of `a` and `b`.
static int64_t Max(const int64_t a, const int64_t b) { return (a > b) ? a : b; }

// The spin() calls of each process of TestUserModePreemptibleSyscalls() that
// has a CPU to itself, and the timer ticks of each call.
constexpr int64_t kLongSyscalls = 2;
constexpr int64_t kLongSyscallTicks = 25;
// The spin() calls of each process of a round of
// TestUserModePreemptibleSyscalls() that shares a CPU, and the timer ticks of
// each call.
constexpr int64_t kRoundSyscalls = 6;
constexpr int64_t kRoundSyscallTicks = 4;
// The rounds after which TestUserModePreemptibleSyscalls() stops waiting for a
// system call that ends on another CPU than it started on.
constexpr int kMaxSyscallSpinRounds = 3;

// The counters of the processes of the rounds of
// TestUserModePreemptibleSyscalls(), summed up.
struct SyscallSpinTotals {
  int64_t switched_out = 0;
  int64_t migrated = 0;
};

// Runs `count` processes that each make kRoundSyscalls spin() calls of
// kRoundSyscallTicks ticks, with buffers from `seed` on, checks that each one
// exits with 0 after all of them, and adds their counters to `*totals`.
static bool RunSyscallSpinRound(const int count,      //
                                const uint64_t seed,  //
                                SyscallSpinTotals* const totals) {
  // On an early return, the destructors kill the processes and wait for them.
  const ProcessArray processes = MakeProcessArray(count);
  for (int i = 0; i < count; ++i) {
    processes[i] = UserProcess::Start(
        UserTestProgram(),
        usertest::KernelSpinArg(kRoundSyscallTicks,  //
                                kRoundSyscalls,      //
                                seed + static_cast<uint64_t>(i)));
    SELFTEST_EXPECT(processes[i] != nullptr);
  }
  // A CPU that has run all of its own processes takes one that waits on
  // another CPU, which switched it out in the middle of a call: CPU 0 does
  // when this task yields, the others when they go idle
  // (work_stealing_scheduler.cpp).
  SELFTEST_EXPECT(
      WaitForProcesses(processes.get(), count, HasEnded, WaitMode::kYield));
  for (int i = 0; i < count; ++i) {
    SELFTEST_EXPECT(ExitedWith(processes[i]->Wait(), 0));
    const UserProcessStats stats = processes[i]->stats();
    // The spin() calls and exit().
    SELFTEST_EXPECT(stats.syscalls == kRoundSyscalls + 1);
    totals->switched_out += stats.syscalls_switched_out;
    totals->migrated += stats.syscalls_migrated;
  }
  return true;
}

// Checks that system calls run with interrupts enabled and that the scheduler
// may switch a task out in the middle of one. First one process per CPU but
// CPU 0 (which runs this task) makes spin() calls of kLongSyscallTicks timer
// ticks each: alone on its CPU, it is never switched out, and the CPU keeps
// taking timer interrupts while the program is in the middle of each call.
// Then 2 × CPUs - 1 processes make short spin() calls, so that the CPUs switch
// between them in the middle of calls; a CPU that has run all of its own
// processes takes a waiting one from another CPU, where that one finishes its
// call. Every process checks every result (and its registers, buffer, stack
// and data) and exits with 0, so the calls that were switched out, including
// at least one that ended on another CPU than it started on, completed
// correctly. Afterwards every frame and heap byte is back.
static bool TestUserModePreemptibleSyscalls() {
  SELFTEST_EXPECT(GetTaskScheduler() != nullptr);
  const int cpu_count = SmpCpuCount();
  const int64_t pmm_free_before = PmmFreeFrameCount();
  const int64_t heap_free_before = HeapTotalFreeBytes();

  const int long_count = (cpu_count > 1) ? cpu_count - 1 : 1;
  int64_t min_long_call_ticks = -1;
  {
    const ProcessArray processes = MakeProcessArray(long_count);
    const auto arg_of = [](const int i) {
      return usertest::KernelSpinArg(kLongSyscallTicks,  //
                                     kLongSyscalls,      //
                                     static_cast<uint64_t>(i + 1));
    };
    // Once a process runs, it keeps its CPU: nothing else is queued there.
    SELFTEST_EXPECT(
        StartProcessesOnOwnCpus(processes.get(), long_count, arg_of));
    // A process whose calls never end fails the test here instead of hanging
    // in Wait().
    SELFTEST_EXPECT(WaitForProcesses(processes.get(),  //
                                     long_count,       //
                                     HasEnded,         //
                                     WaitMode::kSpin));
    for (int i = 0; i < long_count; ++i) {
      SELFTEST_EXPECT(ExitedWith(processes[i]->Wait(), 0));
      const UserProcessStats stats = processes[i]->stats();
      SELFTEST_EXPECT(stats.syscalls == kLongSyscalls + 1);
      SELFTEST_EXPECT(stats.syscall_timer_ticks >=
                      kLongSyscalls * kLongSyscallTicks);
      if (cpu_count > 1) {
        // Alone on its CPU, it ran each call from its start to its end there.
        SELFTEST_EXPECT(stats.syscalls_switched_out == 0);
        SELFTEST_EXPECT(stats.syscalls_migrated == 0);
        SELFTEST_EXPECT(stats.max_syscall_timer_ticks >= kLongSyscallTicks);
      }
      if (min_long_call_ticks < 0 ||
          stats.max_syscall_timer_ticks < min_long_call_ticks) {
        min_long_call_ticks = stats.max_syscall_timer_ticks;
      }
    }
  }

  const int round_count = 2 * cpu_count - 1;
  // Some CPU runs fewer of the processes than another one and takes a waiting
  // one from it once it has run its own.
  const bool expect_migration = cpu_count > 1 && round_count > cpu_count;
  SyscallSpinTotals totals;
  int rounds = 0;
  do {
    const uint64_t seed = static_cast<uint64_t>(round_count * (rounds + 1) + 1);
    SELFTEST_EXPECT(RunSyscallSpinRound(round_count, seed, &totals));
    ++rounds;
  } while (expect_migration && totals.migrated == 0 &&
           rounds < kMaxSyscallSpinRounds);

  UartWrite("[TEST] user_mode_preemptible_syscalls: ticks_in_syscall>=");
  UartWriteDec(static_cast<uint64_t>(min_long_call_ticks));
  UartWrite(" switched=");
  UartWriteDec(static_cast<uint64_t>(totals.switched_out));
  UartWrite(" migrated=");
  UartWriteDec(static_cast<uint64_t>(totals.migrated));
  UartWrite(" processes=");
  UartWriteDec(static_cast<uint64_t>(round_count));
  UartWrite(" rounds=");
  UartWriteDec(static_cast<uint64_t>(rounds));
  UartWrite("\n");
  if (round_count > cpu_count) {
    SELFTEST_EXPECT(totals.switched_out >= 1);
  }
  if (expect_migration) {
    SELFTEST_EXPECT(totals.migrated >= 1);
  }
  SELFTEST_EXPECT(PagingActiveRoot() == PagingKernelRoot());
  SELFTEST_EXPECT(PagingUserSpaceIsEmpty());
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  SELFTEST_EXPECT(HeapTotalFreeBytes() == heap_free_before);
  return true;
}

// The most timer ticks of CPU 0 (`UserProcessStats`) from a request to kill a
// program that runs, in user mode or in the middle of a system call, to the
// end of its process.
constexpr int64_t kKillDeadlineTicks = 10;
// The processes that TestUserModeKill() kills right after it started them.
constexpr int kKillsBeforeEntry = 3;

// What the phases of TestUserModeKill() observed. The `*_ticks` fields are the
// most timer ticks from a kill request to the end of a process.
struct KillTestReport {
  // Pure spinners killed while each ran on a CPU of its own.
  int spinners = 0;
  int64_t spinner_ticks = 0;
  int64_t spinner_raw_ticks = 0;
  int64_t spinner_steal_ticks = 0;
  int64_t spinner_notice_ticks = 0;
  // Processes killed in the middle of a long spin() call.
  int mid_syscall = 0;
  int64_t mid_syscall_ticks = 0;
  int64_t mid_syscall_raw_ticks = 0;
  int64_t mid_syscall_steal_ticks = 0;
  int64_t mid_syscall_notice_ticks = 0;
  // Processes killed while more of them than CPUs were runnable, and those
  // that no CPU was running at their kill.
  int waiting = 0;
  int not_running = 0;
  int64_t waiting_ticks = 0;
  // Processes killed right after they were started that never entered user
  // mode.
  int before_entry = 0;
};

// Waits for `process`, which the test has asked to kill, checks that it ended
// as killed after the request, and stores its counters in `*stats`. A process
// that doesn't end in time fails the test here instead of hanging in Wait().
static bool EndedKilled(const std::unique_ptr<UserProcess>& process,
                        UserProcessStats* const stats) {
  SELFTEST_EXPECT(WaitForProcesses(&process, 1, HasEnded, WaitMode::kYield));
  const UserProgramResult result = process->Wait();
  if (result.outcome != UserProgramOutcome::kKilled) {
    LogUnexpectedResult(result);
  }
  SELFTEST_EXPECT(result.outcome == UserProgramOutcome::kKilled);
  *stats = process->stats();
  SELFTEST_EXPECT(stats->kill_tick >= 0);
  SELFTEST_EXPECT(stats->end_tick >= stats->kill_tick);
  return true;
}

// Returns the timer ticks from the kill request to the end of the process,
// excluding any hypervisor steal ticks proven to have occurred on the
// target CPU while the process remained continuously resident without noticing
// the kill.
static int64_t KillTicks(const UserProcessStats& stats) {
  const int64_t raw_ticks = stats.end_tick - stats.kill_tick;
  const int64_t adjusted = raw_ticks - stats.kill_steal_ticks;
  return Max(0, adjusted);
}

// Logs the timer ticks of a kill of `phase` that missed kKillDeadlineTicks,
// before the expectation fails, so that a failing pass says by how much.
// `stats` is that of a process that EndedKilled() accepted. Prints three
// lines, each of which fits in a LineBuffer whatever the values.
static void LogLateKill(const char* const phase,
                        const UserProcessStats& stats) {
  if (KillTicks(stats) <= kKillDeadlineTicks) {
    return;
  }
  const int64_t raw_ticks = stats.end_tick - stats.kill_tick;
  const int64_t notice_ticks = (stats.kill_notice_tick >= stats.kill_tick)
                                   ? (stats.kill_notice_tick - stats.kill_tick)
                                   : -1;
  LineBuffer line;
  line.Append("[USER] user_mode_kill: missed deadline phase=")
      .Append(phase)
      .Append(" kill_ticks=")
      .AppendDec(static_cast<uint64_t>(KillTicks(stats)))
      .Append("\n");
  UartWrite(line.c_str());
  LineBuffer ticks_line;
  ticks_line.Append("[USER] user_mode_kill: raw_ticks=")
      .AppendDec(static_cast<uint64_t>(Max(0, raw_ticks)))
      .Append(" steal_ticks=")
      .AppendDec(static_cast<uint64_t>(Max(0, stats.kill_steal_ticks)))
      .Append("\n");
  UartWrite(ticks_line.c_str());
  LineBuffer cpu_line;
  cpu_line.Append("[USER] user_mode_kill: notice_ticks=")
      .AppendDec(static_cast<uint64_t>(Max(0, notice_ticks)))
      .Append(" cpu=")
      .AppendDec(static_cast<uint64_t>(Max(0, stats.kill_end_cpu)))
      .Append(" same_cpu=")
      .AppendDec(stats.kill_same_cpu_no_switch ? 1 : 0)
      .Append("\n");
  UartWrite(cpu_line.c_str());
}

// Kills pure spinners, which never make a system call, while each runs on a
// CPU of its own and CPU 0 runs this task, so that no more tasks than CPUs
// are runnable: each ends as killed within kKillDeadlineTicks. Meanwhile it
// kills processes right after it started them, which wait behind this task
// for CPU 0 because every other CPU is busy: they end before they enter user
// mode. And it destroys a running spinner without Wait(): the destructor
// kills it and waits for it.
static bool TestKillRunningPhase(const int cpu_count,
                                 KillTestReport* const report) {
  using usertest::Mode;
  TaskScheduler* const scheduler = GetTaskScheduler();
  const int count = (cpu_count > 1) ? cpu_count - 1 : 1;
  const ProcessArray spinners = MakeProcessArray(count);
  const auto arg_of = [](const int i) {
    return usertest::Arg(Mode::kSpinWithoutSyscalls,
                         0xA11CE + static_cast<uint64_t>(i));
  };
  SELFTEST_EXPECT(StartProcessesOnOwnCpus(spinners.get(), count, arg_of));
  SELFTEST_EXPECT(WaitForProcesses(spinners.get(),    //
                                   count,             //
                                   HasRunInUserMode,  //
                                   WaitMode::kSpin));

  for (int i = 0; i < kKillsBeforeEntry; ++i) {
    scheduler->SetPreemptEnabled(false);
    const std::unique_ptr<UserProcess> process = UserProcess::Start(
        UserTestProgram(), usertest::Arg(Mode::kSpinWithoutSyscalls));
    if (process != nullptr) {
      process->Kill();
    }
    scheduler->SetPreemptEnabled(true);
    SELFTEST_EXPECT(process != nullptr);
    UserProcessStats stats;
    SELFTEST_EXPECT(EndedKilled(process, &stats));
    SELFTEST_EXPECT(stats.syscalls == 0);
    // Only a program that entered user mode ends with a vector.
    if (stats.end_vector < 0 && stats.user_timer_ticks == 0) {
      ++report->before_entry;
    }
  }
  {
    const std::unique_ptr<UserProcess> process = UserProcess::Start(
        UserTestProgram(), usertest::Arg(Mode::kSpinWithoutSyscalls));
    SELFTEST_EXPECT(process != nullptr);
    SELFTEST_EXPECT(
        WaitForProcesses(&process, 1, HasRunUserCode, WaitMode::kYield));
  }

  if constexpr (kInjectedKillDelay == KillDelayInjection::kSpinner ||
                kInjectedKillDelay == KillDelayInjection::kTeardown) {
    spinners[0]->ArmInjectedKillDelay(2 * kKillDeadlineTicks + 5);
  }
  for (int i = 0; i < count; ++i) {
    spinners[i]->Kill();
  }
  for (int i = 0; i < count; ++i) {
    UserProcessStats stats;
    SELFTEST_EXPECT(EndedKilled(spinners[i], &stats));
    SELFTEST_EXPECT(stats.syscalls == 0);
    if (cpu_count > 1) {
      // It was running on its CPU, not on CPU 0.
      SELFTEST_EXPECT(stats.kill_running_cpu > 0);
    }
    LogLateKill("spinner", stats);
    SELFTEST_EXPECT(KillTicks(stats) <= kKillDeadlineTicks);
    report->spinner_ticks = Max(report->spinner_ticks, KillTicks(stats));
    report->spinner_raw_ticks =
        Max(report->spinner_raw_ticks, stats.end_tick - stats.kill_tick);
    report->spinner_steal_ticks =
        Max(report->spinner_steal_ticks, stats.kill_steal_ticks);
    if (stats.kill_notice_tick >= stats.kill_tick) {
      report->spinner_notice_ticks =
          Max(report->spinner_notice_ticks,
              stats.kill_notice_tick - stats.kill_tick);
    }
  }
  report->spinners = count;
  SELFTEST_EXPECT(report->before_entry >= 1);
  return true;
}

// Kills processes in the middle of a spin() call that would last more than an
// hour, each running on a CPU of its own: each ends as killed within
// kKillDeadlineTicks, on its way back from the call, without having returned
// to user mode.
static bool TestKillInSyscallPhase(const int cpu_count,
                                   KillTestReport* const report) {
  const int count = (cpu_count > 1) ? cpu_count - 1 : 1;
  const ProcessArray processes = MakeProcessArray(count);
  const auto arg_of = [](const int i) {
    return usertest::KernelSpinArg(kSyscallSpinMaxTicks,  //
                                   1,                     //
                                   0xB0B + static_cast<uint64_t>(i));
  };
  SELFTEST_EXPECT(StartProcessesOnOwnCpus(processes.get(), count, arg_of));
  SELFTEST_EXPECT(
      WaitForProcesses(processes.get(), count, IsInSyscall, WaitMode::kSpin));
  if constexpr (kInjectedKillDelay == KillDelayInjection::kMidSyscall) {
    processes[0]->ArmInjectedKillDelay(2 * kKillDeadlineTicks + 5);
  }
  for (int i = 0; i < count; ++i) {
    processes[i]->Kill();
  }
  for (int i = 0; i < count; ++i) {
    UserProcessStats stats;
    SELFTEST_EXPECT(EndedKilled(processes[i], &stats));
    SELFTEST_EXPECT(stats.kill_in_syscall);
    SELFTEST_EXPECT(stats.syscalls == 1);
    SELFTEST_EXPECT(stats.end_vector == kVectorSyscall);
    if (cpu_count > 1) {
      SELFTEST_EXPECT(stats.kill_running_cpu > 0);
    }
    LogLateKill("mid_syscall", stats);
    SELFTEST_EXPECT(KillTicks(stats) <= kKillDeadlineTicks);
    report->mid_syscall_ticks =
        Max(report->mid_syscall_ticks, KillTicks(stats));
    report->mid_syscall_raw_ticks =
        Max(report->mid_syscall_raw_ticks, stats.end_tick - stats.kill_tick);
    report->mid_syscall_steal_ticks =
        Max(report->mid_syscall_steal_ticks, stats.kill_steal_ticks);
    if (stats.kill_notice_tick >= stats.kill_tick) {
      report->mid_syscall_notice_ticks =
          Max(report->mid_syscall_notice_ticks,
              stats.kill_notice_tick - stats.kill_tick);
    }
  }
  report->mid_syscall = count;
  return true;
}

// Returns the argument of process `i` of TestKillWaitingPhase(): a program
// that makes no system call, one that makes short spin() calls until it is
// killed, or one that makes a single spin() call of more than an hour.
static uint64_t WaitingKillTestArg(const int i) {
  const uint64_t seed = 0xC0DE + static_cast<uint64_t>(i);
  switch (i % 3) {
    case 0:
      return usertest::Arg(usertest::Mode::kSpinWithoutSyscalls, seed);
    case 1:
      return usertest::KernelSpinArg(2, 0, seed);
    default:
      return usertest::KernelSpinArg(kSyscallSpinMaxTicks, 1, seed);
  }
}

// Kills 3 × CPUs processes, which make system calls or not, while most of them
// wait for a CPU: all end as killed, including those that no CPU was running
// at their kill. Asking again to kill a process, also after Wait(), changes
// nothing.
static bool TestKillWaitingPhase(const int cpu_count,
                                 KillTestReport* const report) {
  const int count = 3 * cpu_count;
  const ProcessArray processes = MakeProcessArray(count);
  for (int i = 0; i < count; ++i) {
    processes[i] = UserProcess::Start(UserTestProgram(), WaitingKillTestArg(i));
    SELFTEST_EXPECT(processes[i] != nullptr);
  }
  SELFTEST_EXPECT(WaitForProcesses(processes.get(),  //
                                   count,            //
                                   HasRunUserCode,   //
                                   WaitMode::kYield));
  {
    // Kill from back to front with interrupts disabled on CPU 0 so that CPU 0
    // is not preempted mid-loop and queued processes at the tail of each
    // runqueue are marked killed before the running ones at the front vacate
    // their CPUs.
    const ScopedIrqDisable irq_disable;
    for (int i = count - 1; i >= 0; --i) {
      processes[i]->Kill();
    }
  }
  processes[0]->Kill();
  int not_running = 0;
  for (int i = 0; i < count; ++i) {
    UserProcessStats stats;
    SELFTEST_EXPECT(EndedKilled(processes[i], &stats));
    not_running += (stats.kill_running_cpu < 0) ? 1 : 0;
    report->waiting_ticks = Max(report->waiting_ticks, KillTicks(stats));
  }
  const UserProcessStats before = processes[0]->stats();
  processes[0]->Kill();
  const UserProcessStats after = processes[0]->stats();
  SELFTEST_EXPECT(after.kill_tick == before.kill_tick);
  SELFTEST_EXPECT(after.syscalls == before.syscalls);
  // With more processes than CPUs, most of them wait while this task kills
  // them one by one.
  if (count > cpu_count) {
    SELFTEST_EXPECT(not_running >= 1);
  }
  report->waiting = count;
  report->not_running = not_running;
  return true;
}

// Checks Kill(): programs that never make a system call and run on a CPU of
// their own, while no more processes than CPUs are runnable, end as killed
// within kKillDeadlineTicks timer ticks, and so do programs in the middle of a
// system call. Programs that are not running, and programs killed before they
// enter user mode, end as killed too. Afterwards every frame and heap byte is
// back.
static bool TestUserModeKill() {
  SELFTEST_EXPECT(GetTaskScheduler() != nullptr);
  const int cpu_count = SmpCpuCount();
  // The values before the test. The heap never returns the frames that it
  // takes from the PMM (heap.h): if the test needs more heap at once than
  // earlier tests left free, the counters can't come back to them. So the
  // phase that runs the most processes at once runs first, and the end of the
  // test must match the values after it exactly (every kill path runs after
  // them). Unless the heap grew meanwhile, those are the values from before.
  const int64_t pmm_free_at_start = PmmFreeFrameCount();
  const int64_t heap_free_at_start = HeapTotalFreeBytes();
  KillTestReport warm_up;
  SELFTEST_EXPECT(TestKillWaitingPhase(cpu_count, &warm_up));
  const int64_t pmm_free_before = PmmFreeFrameCount();
  const int64_t heap_free_before = HeapTotalFreeBytes();
  // Growth moves frames from the PMM into the heap, where they are free.
  const bool heap_grew = pmm_free_before < pmm_free_at_start &&
                         heap_free_before > heap_free_at_start;
  if (!heap_grew) {
    SELFTEST_EXPECT(pmm_free_before == pmm_free_at_start);
    SELFTEST_EXPECT(heap_free_before == heap_free_at_start);
  }

  KillTestReport report;
  SELFTEST_EXPECT(TestKillRunningPhase(cpu_count, &report));
  SELFTEST_EXPECT(TestKillInSyscallPhase(cpu_count, &report));
  SELFTEST_EXPECT(TestKillWaitingPhase(cpu_count, &report));

  UartWrite("[TEST] user_mode_kill: spinners=");
  UartWriteDec(static_cast<uint64_t>(report.spinners));
  UartWrite(" max_kill_ticks=");
  UartWriteDec(static_cast<uint64_t>(
      Max(report.spinner_ticks, report.mid_syscall_ticks)));
  UartWrite(" mid_syscall=");
  UartWriteDec(static_cast<uint64_t>(report.mid_syscall));
  UartWrite(" not_running=");
  UartWriteDec(static_cast<uint64_t>(report.not_running));
  UartWrite("\n[USER] user_mode_kill: spinner_ticks=");
  UartWriteDec(static_cast<uint64_t>(report.spinner_ticks));
  UartWrite(" mid_syscall_ticks=");
  UartWriteDec(static_cast<uint64_t>(report.mid_syscall_ticks));
  UartWrite(" waiting=");
  UartWriteDec(static_cast<uint64_t>(report.waiting));
  UartWrite(" waiting_ticks=");
  UartWriteDec(static_cast<uint64_t>(report.waiting_ticks));
  UartWrite(" before_entry=");
  UartWriteDec(static_cast<uint64_t>(report.before_entry));
  UartWrite("/");
  UartWriteDec(static_cast<uint64_t>(kKillsBeforeEntry));
  UartWrite(" heap_grew=");
  UartWriteDec(heap_grew ? 1 : 0);
  UartWrite(" spinner_raw_ticks=");
  UartWriteDec(static_cast<uint64_t>(report.spinner_raw_ticks));
  UartWrite(" spinner_steal_ticks=");
  UartWriteDec(static_cast<uint64_t>(report.spinner_steal_ticks));
  UartWrite(" spinner_notice_ticks=");
  UartWriteDec(static_cast<uint64_t>(report.spinner_notice_ticks));
  UartWrite(" mid_syscall_raw_ticks=");
  UartWriteDec(static_cast<uint64_t>(report.mid_syscall_raw_ticks));
  UartWrite(" mid_syscall_steal_ticks=");
  UartWriteDec(static_cast<uint64_t>(report.mid_syscall_steal_ticks));
  UartWrite(" mid_syscall_notice_ticks=");
  UartWriteDec(static_cast<uint64_t>(report.mid_syscall_notice_ticks));
  UartWrite("\n");

  SELFTEST_EXPECT(PagingActiveRoot() == PagingKernelRoot());
  SELFTEST_EXPECT(PagingUserSpaceIsEmpty());
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  SELFTEST_EXPECT(HeapTotalFreeBytes() == heap_free_before);
  return true;
}

static bool ContainsSubstring(const char* const haystack,
                              const char* const needle) {
  if (haystack == nullptr || needle == nullptr) {
    return false;
  }
  if (needle[0] == '\0') {
    return true;
  }
  for (int i = 0; haystack[i] != '\0'; ++i) {
    int j = 0;
    while (needle[j] != '\0' && haystack[i + j] == needle[j]) {
      ++j;
    }
    if (needle[j] == '\0') {
      return true;
    }
  }
  return false;
}

// Verifies kSyscallRead: pre-buffered reads, error validation (invalid sizes
// and unmapped/kernel/read-only user buffers), streaming multi-chunk reads
// while blocked in Read(), and Kill() termination within kKillDeadlineTicks
// while blocked waiting for console input.
static bool TestUserModeConsoleRead() {
  using usertest::Mode;
  ConsoleClearInput();

  // (a) Pre-seeded console input + Read() validation checks.
  constexpr const char kInputMessage[] = "hello, ring-3 console!\n";
  constexpr int64_t kInputLen = static_cast<int64_t>(sizeof(kInputMessage)) - 1;
  ConsolePushInput(kInputMessage, kInputLen);
  {
    const std::unique_ptr<UserProcess> reader = UserProcess::Start(
        UserTestProgram(),
        usertest::Arg(Mode::kConsoleRead, static_cast<uint64_t>(kInputLen)));
    SELFTEST_EXPECT(reader != nullptr);
    SELFTEST_EXPECT(WaitForProcesses(&reader, 1, HasEnded, WaitMode::kYield));
    const int64_t expected_checksum = SyscallSpinChecksum(
        reinterpret_cast<const uint8_t*>(kInputMessage), kInputLen);
    SELFTEST_EXPECT(ExitedWith(reader->Wait(), expected_checksum));
  }

  // (b) Streaming input fed in two chunks after the process blocks in Read().
  ConsoleClearInput();
  {
    constexpr const char kStreamInput[] = "stream8!";
    constexpr int64_t kStreamLen = 8;
    const std::unique_ptr<UserProcess> reader = UserProcess::Start(
        UserTestProgram(),
        usertest::Arg(Mode::kConsoleRead, static_cast<uint64_t>(kStreamLen)));
    SELFTEST_EXPECT(reader != nullptr);
    SELFTEST_EXPECT(WaitForProcesses(
        &reader, 1,
        [](const UserProcessStats& s) { return s.in_syscall || HasEnded(s); },
        WaitMode::kYield));
    SELFTEST_EXPECT(!HasEnded(reader->stats()));
    ConsolePushInput(kStreamInput, 4);
    GetTaskScheduler()->Yield();
    ConsolePushInput(kStreamInput + 4, 4);
    SELFTEST_EXPECT(WaitForProcesses(&reader, 1, HasEnded, WaitMode::kYield));
    const int64_t expected_checksum = SyscallSpinChecksum(
        reinterpret_cast<const uint8_t*>(kStreamInput), kStreamLen);
    SELFTEST_EXPECT(ExitedWith(reader->Wait(), expected_checksum));
  }

  // (c) Kill() while blocked in Read() terminates within kKillDeadlineTicks.
  ConsoleClearInput();
  {
    const int cpu_count = SmpCpuCount();
    const int count = (cpu_count > 1) ? cpu_count - 1 : 1;
    const ProcessArray readers = MakeProcessArray(count);
    const auto arg_of = [](const int /*i*/) {
      return usertest::Arg(Mode::kConsoleRead, 0);
    };
    SELFTEST_EXPECT(StartProcessesOnOwnCpus(readers.get(), count, arg_of));
    SELFTEST_EXPECT(
        WaitForProcesses(readers.get(), count, IsInSyscall, WaitMode::kSpin));
    for (int i = 0; i < count; ++i) {
      readers[i]->Kill();
    }
    for (int i = 0; i < count; ++i) {
      UserProcessStats stats;
      SELFTEST_EXPECT(EndedKilled(readers[i], &stats));
      SELFTEST_EXPECT(stats.kill_in_syscall);
      SELFTEST_EXPECT(stats.syscalls == 1);
      SELFTEST_EXPECT(stats.end_vector == kVectorSyscall);
      LogLateKill("console_read", stats);
      SELFTEST_EXPECT(KillTicks(stats) <= kKillDeadlineTicks);
    }
  }
  ConsoleClearInput();
  return true;
}

// Verifies kSyscallSpawn and kSyscallWait: error validation, child exit/fault
// outcomes, child slot exhaustion, Kill() while blocked in Wait(), and
// automatic orphan child cleanup when a parent exits, faults, or is killed.
static bool TestUserModeSpawnAndWait() {
  using usertest::Mode;
  using usertest::SpawnTestSubmode;

  // (a) Full Spawn/Wait validation, child exit/fault outcomes, slot limits.
  {
    const std::unique_ptr<UserProcess> proc = UserProcess::Start(
        UserTestProgram(),
        usertest::Arg(
            Mode::kSpawnAndWait,
            static_cast<uint64_t>(SpawnTestSubmode::kBasicAndErrors)));
    SELFTEST_EXPECT(proc != nullptr);
    SELFTEST_EXPECT(WaitForProcesses(&proc, 1, HasEnded, WaitMode::kYield));
    SELFTEST_EXPECT(ExitedWith(proc->Wait(), 0));
  }

  // (b) Kill() while blocked in Wait() kills the parent within deadline and
  // automatically cleans up the spinning child.
  {
    const std::unique_ptr<UserProcess> parent = UserProcess::Start(
        UserTestProgram(),
        usertest::Arg(Mode::kSpawnAndWait,
                      static_cast<uint64_t>(SpawnTestSubmode::kBlockInWait)));
    SELFTEST_EXPECT(parent != nullptr);
    SELFTEST_EXPECT(WaitForProcesses(
        &parent, 1,
        [](const UserProcessStats& s) {
          return (s.syscalls >= 2 && s.in_syscall) || HasEnded(s);
        },
        WaitMode::kYield));
    SELFTEST_EXPECT(!HasEnded(parent->stats()));
    parent->Kill();
    UserProcessStats stats;
    SELFTEST_EXPECT(EndedKilled(parent, &stats));
    SELFTEST_EXPECT(stats.kill_in_syscall);
    SELFTEST_EXPECT(stats.end_vector == kVectorSyscall);
    SELFTEST_EXPECT(KillTicks(stats) <= 5 * kKillDeadlineTicks);
  }

  // (c) Parent exits normally without calling Wait() on two spinning
  // children: CleanupChildren() kills and reaps both children.
  {
    const std::unique_ptr<UserProcess> parent = UserProcess::Start(
        UserTestProgram(),
        usertest::Arg(Mode::kSpawnAndWait,
                      static_cast<uint64_t>(SpawnTestSubmode::kOrphanOnExit)));
    SELFTEST_EXPECT(parent != nullptr);
    SELFTEST_EXPECT(WaitForProcesses(&parent, 1, HasEnded, WaitMode::kYield));
    SELFTEST_EXPECT(ExitedWith(parent->Wait(), 0));
  }

  // (d) Parent faults (#UD) without calling Wait() on a spinning child:
  // CleanupChildren() kills and reaps the child.
  {
    const std::unique_ptr<UserProcess> parent = UserProcess::Start(
        UserTestProgram(),
        usertest::Arg(Mode::kSpawnAndWait,
                      static_cast<uint64_t>(SpawnTestSubmode::kOrphanOnFault)));
    SELFTEST_EXPECT(parent != nullptr);
    SELFTEST_EXPECT(WaitForProcesses(&parent, 1, HasEnded, WaitMode::kYield));
    SELFTEST_EXPECT(FaultedWith(parent->Wait(), kVectorInvalidOpcode, 0, 0));
  }

  return true;
}

// Verifies the end-to-end init -> sh workflow and interactive shell features
// (prompt, character echoing, '\b' and 0x7F backspace editing with empty-line
// underflow guard, '\r\n' handling, empty/whitespace lines, built-in commands
// help/echo/cpu/programs/exit, spawning usertest passing and faulting modes,
// and unknown command recovery).
static bool TestUserModeInteractiveShell() {
  const bool prev_vga_echo = ConsoleVgaEchoEnabled();
  ConsoleSetVgaEcho(false);
  ConsoleClearInput();
  ConsoleResetOutputCapture();

  constexpr const char kShellScript[] =
      "\n"
      "   \t  \n"
      "\b\x7f"
      "ecoh\b\bho edited_line_ok\r\n"
      "help\n"
      "cpu\n"
      "programs\n"
      "usertest 1\n"
      "run usertest 0\n"
      "usertest 6\n"
      "nosuchcommand arg1\n"
      "exit 0\n";
  ConsolePushInput(kShellScript,
                   static_cast<int64_t>(sizeof(kShellScript)) - 1);

  const std::unique_ptr<UserProcess> init_proc =
      UserProcess::Start(InitProgram(), 0);
  if (init_proc == nullptr) {
    ConsoleSetVgaEcho(prev_vga_echo);
    return false;
  }
  const bool waited =
      WaitForProcesses(&init_proc, 1, HasEnded, WaitMode::kYield);
  const UserProgramResult init_result = init_proc->Wait();

  static char captured[4096];
  const int64_t captured_len =
      ConsoleCopyOutputCapture(captured, sizeof(captured));

  // Also verify custom non-zero exit status directly on ShProgram().
  ConsoleClearInput();
  constexpr const char kExit42Script[] = "exit 42\n";
  ConsolePushInput(kExit42Script,
                   static_cast<int64_t>(sizeof(kExit42Script)) - 1);
  const std::unique_ptr<UserProcess> sh_proc =
      UserProcess::Start(ShUserProgram(), 0);
  bool sh_waited = false;
  UserProgramResult sh_result;
  if (sh_proc != nullptr) {
    sh_waited = WaitForProcesses(&sh_proc, 1, HasEnded, WaitMode::kYield);
    sh_result = sh_proc->Wait();
  }

  ConsoleClearInput();
  ConsoleSetVgaEcho(prev_vga_echo);

  SELFTEST_EXPECT(waited);
  SELFTEST_EXPECT(ExitedWith(init_result, 0));
  SELFTEST_EXPECT(captured_len > 0);
  SELFTEST_EXPECT(ContainsSubstring(captured, "[INIT] hello from ring 3\n"));
  SELFTEST_EXPECT(ContainsSubstring(captured, "sh> "));
  SELFTEST_EXPECT(ContainsSubstring(captured, "\b \b"));
  SELFTEST_EXPECT(ContainsSubstring(captured, "edited_line_ok\n"));
  SELFTEST_EXPECT(ContainsSubstring(
      captured, "built-in commands: help echo cpu programs run exit\n"));
  SELFTEST_EXPECT(ContainsSubstring(captured, "cpu: "));
  SELFTEST_EXPECT(ContainsSubstring(captured, "programs: init sh usertest\n"));
  SELFTEST_EXPECT(
      ContainsSubstring(captured, "sh: usertest exited with status 0\n"));
  SELFTEST_EXPECT(ContainsSubstring(
      captured, "sh: usertest exited with status -1250999896491\n"));
  SELFTEST_EXPECT(ContainsSubstring(captured, "sh: usertest faulted vector=6"));
  SELFTEST_EXPECT(
      ContainsSubstring(captured, "sh: unknown command: nosuchcommand\n"));
  SELFTEST_EXPECT(sh_proc != nullptr);
  SELFTEST_EXPECT(sh_waited);
  SELFTEST_EXPECT(ExitedWith(sh_result, 42));
  return true;
}

// The most processes that TestUserModeFpuIsolation() runs at once: twice the
// most CPUs that the kernel supports, so that both FPU tests always run twice
// as many processes as CPUs. Memory is no limit: a process takes about
// 110 KiB (about 23 frames for usertest's pages, its stack, its page tables
// and its kernel stack, and a few KiB of heap), so even the 4 * 128 processes
// that TestUserModeFpuInitialState() runs at once on 128 CPUs fit in about
// 55 MiB. The process group lives on the heap, because its arrays would not
// fit on the 16 KiB boot stack.
constexpr int kMaxFpuTestProcesses = 2 * kMaxSupportedCpus;
// Timer ticks that every CPU must take while all `kFpuHold` processes of
// TestUserModeFpuIsolation() share the CPUs (4 ms each at 250 Hz), and timer
// interrupts that each of the processes must take in user mode meanwhile,
// while it holds its values in the registers.
constexpr int64_t kFpuIsolationTicksPerCpu = 100;
constexpr int64_t kFpuIsolationUserTicks = 10;
// System calls that each `kFpuHold` process must make in each step of
// TestUserModeFpuIsolation(): at least 3 complete checked rounds.
constexpr int64_t kFpuHoldStepSyscalls = 4 * usertest::kFpuHoldSyscalls;
// TestUserModeFpuIsolation() drains each CPU once, and more CPUs (at most
// kFpuDrainRoundsPerCpu rounds per CPU in all) until processes have moved
// between CPUs kFpuDrainMovesPerCpu times per CPU, twice the once per CPU
// that it requires, so that its evidence stays clear of the requirement, and
// until a process that moved has checked 3 more rounds since.
constexpr int kFpuDrainRoundsPerCpu = 4;
constexpr int kFpuDrainMovesPerCpu = 2;
// Timer ticks of CPU 0 that a drain round waits at most for a process to move
// to another CPU.
constexpr int64_t kFpuDrainRoundTicks = 50;
// The first seeds of the `kFpuHold` and `kFpuDirty` processes; each process
// gets a seed of its own.
constexpr uint64_t kFpuHoldSeedBase = 0xF0F0'0000;
constexpr uint64_t kFpuDirtySeedBase = 0xD1D1'0000;
// What TestUserModeFpuIsolation() covered, which test_boot.sh checks: always
// the x87/MMX registers, the SSE registers, MXCSR and the x87 control word,
// and the upper halves of the YMM registers only if the kernel enabled the AVX
// state.
constexpr const char* kFpuCoveredLine =
    "[TEST] user_mode_fpu_isolation: covered x87/MMX SSE MXCSR FCW\n";
constexpr const char* kFpuAvxCoveredLine =
    "[TEST] user_mode_fpu_isolation: covered AVX YMM upper halves\n";
constexpr const char* kFpuAvxNotCoveredLine =
    "[TEST] user_mode_fpu_isolation: AVX not covered (CPU has no AVX state)\n";

// The `kFpuHold` processes of TestUserModeFpuIsolation(), and where each has
// made system calls. On an early return, the destructors kill the processes
// and wait for them.
struct FpuHoldGroup {
  std::unique_ptr<UserProcess> processes[kMaxFpuTestProcesses];
  // The CPUs on which each process has made system calls, and the CPU of its
  // latest system call that the test saw (-1: none yet).
  AtomicCpuMask128 cpus[kMaxFpuTestProcesses];
  int last_cpu[kMaxFpuTestProcesses] = {};
  // The system calls that each process had made when the current step of
  // the test began: 0 for a process started since.
  int64_t step_syscalls[kMaxFpuTestProcesses] = {};
  int count = 0;
  uint64_t next_seed = kFpuHoldSeedBase;
  // The times the test saw a process make a system call on another CPU than
  // the one before.
  int moves = 0;
  // The system calls that each process had made when the test first saw it
  // on a second CPU (-1: not yet), and whether it has made
  // kFpuHoldStepSyscalls more since: then the round in which it moved and
  // the rounds after it passed their checks (a failed check ends `kFpuHold`
  // at once).
  int64_t move_syscalls[kMaxFpuTestProcesses] = {};
  bool migration_checked[kMaxFpuTestProcesses] = {};
  // The processes so checked, each counted once, also after a drain killed
  // it.
  int checked_migrations = 0;
};

// Starts a `kFpuHold` process with a seed of its own as process `i` of
// `*group`.
static bool StartFpuHold(FpuHoldGroup* const group, const int i) {
  group->processes[i] = UserProcess::Start(
      UserTestProgram(),
      usertest::Arg(usertest::Mode::kFpuHold, group->next_seed++));
  SELFTEST_EXPECT(group->processes[i] != nullptr);
  group->cpus[i].Clear();
  group->last_cpu[i] = -1;
  group->step_syscalls[i] = 0;
  group->move_syscalls[i] = -1;
  group->migration_checked[i] = false;
  return true;
}

// Returns the number of CPUs in `mask`.
static int CountCpus(const CpuMask128& mask) {
  int count = 0;
  for (int cpu = 0; cpu < kMaxSupportedCpus; ++cpu) {
    count += mask.Contains(cpu) ? 1 : 0;
  }
  return count;
}

// Records the CPU of the latest system call of each running process of
// `*group`, and credits the processes whose migration has been checked
// (FpuHoldGroup::checked_migrations). Fails, after logging how it ended, if a
// process has ended: `kFpuHold` runs until it is killed, so a process that
// ended found a register changed.
static bool SampleFpuHoldGroup(FpuHoldGroup* const group) {
  for (int i = 0; i < group->count; ++i) {
    if (group->processes[i] == nullptr) {
      continue;
    }
    UserProcess& process = *group->processes[i];
    if (HasEnded(process.stats())) {
      LogUnexpectedResult(process.Wait());
      return false;
    }
    const int cpu = process.last_syscall_cpu();
    if (cpu < 0) {
      continue;
    }
    group->cpus[i].Add(cpu);
    if (group->last_cpu[i] >= 0 && group->last_cpu[i] != cpu) {
      ++group->moves;
    }
    group->last_cpu[i] = cpu;
    const int64_t syscalls = process.syscall_count();
    if (group->move_syscalls[i] < 0 && CountCpus(group->cpus[i].Load()) > 1) {
      group->move_syscalls[i] = syscalls;
    }
    if (group->move_syscalls[i] >= 0 && !group->migration_checked[i] &&
        syscalls >= group->move_syscalls[i] + kFpuHoldStepSyscalls) {
      group->migration_checked[i] = true;
      ++group->checked_migrations;
    }
  }
  return true;
}

// Stores the system calls that each running process of `group` has made in
// `syscalls`.
static void RecordFpuHoldSyscalls(const FpuHoldGroup& group,
                                  int64_t* const syscalls) {
  for (int i = 0; i < group.count; ++i) {
    syscalls[i] = (group.processes[i] != nullptr)
                      ? group.processes[i]->syscall_count()
                      : 0;
  }
}

// Returns true once each running process of `group` has made
// kFpuHoldStepSyscalls system calls more than `syscalls` records.
static bool FpuHoldsMadeStep(const FpuHoldGroup& group,
                             const int64_t* const syscalls) {
  for (int i = 0; i < group.count; ++i) {
    if (group.processes[i] != nullptr &&
        group.processes[i]->syscall_count() <
            syscalls[i] + kFpuHoldStepSyscalls) {
      return false;
    }
  }
  return true;
}

// Returns the sum of the context switches of all CPUs so far.
static int64_t TotalContextSwitches() {
  int64_t total = 0;
  for (int cpu = 0; cpu < SmpCpuCount(); ++cpu) {
    total +=
        SmpGetCpuLocal(cpu)->context_switches.load(std::memory_order_relaxed);
  }
  return total;
}

// Returns the timer ticks that CPU `cpu` has taken so far.
static int64_t CpuTimerTicks(const int cpu) {
  return SmpGetCpuLocal(cpu)->timer_ticks.load(std::memory_order_relaxed);
}

// Drains CPU `drained_cpu`: kills every process of `*group` whose latest
// system call ran there, waits until they have all ended as killed, waits
// (at most kFpuDrainRoundTicks) until a process moves to another CPU and
// every other process has checked a few more rounds, and only then starts
// replacements. A replacement goes to the least loaded CPU, so starting one
// before the drained CPU took a process from another CPU would refill it.
// Finally waits until the test has seen each replacement make a system call:
// a drain spares a process that has made none, and a CPU that took one from
// the CPU where it started would show no move.
static bool DrainFpuHoldCpu(FpuHoldGroup* const group, const int drained_cpu) {
  TaskScheduler* const scheduler = GetTaskScheduler();
  bool victims[kMaxFpuTestProcesses] = {};
  int victim_count = 0;
  for (int i = 0; i < group->count; ++i) {
    if (group->processes[i]->last_syscall_cpu() == drained_cpu) {
      victims[i] = true;
      ++victim_count;
      group->processes[i]->Kill();
    }
  }
  if (victim_count == 0) {
    return true;
  }
  for (int i = 0; i < group->count; ++i) {
    if (victims[i]) {
      UserProcessStats stats;
      SELFTEST_EXPECT(EndedKilled(group->processes[i], &stats));
      group->processes[i].reset();
    }
  }
  // The drained CPU, which has nothing else to run, takes a process that
  // another CPU preempted (work_stealing_scheduler.cpp) and loads its
  // registers.
  RecordFpuHoldSyscalls(*group, group->step_syscalls);
  const int moves_before = group->moves;
  const int64_t deadline_ticks = CpuTimerTicks(0) + kFpuDrainRoundTicks;
  for (;;) {
    SELFTEST_EXPECT(SampleFpuHoldGroup(group));
    if ((group->moves > moves_before &&
         FpuHoldsMadeStep(*group, group->step_syscalls)) ||
        CpuTimerTicks(0) >= deadline_ticks) {
      break;
    }
    scheduler->Yield();
  }
  for (int i = 0; i < group->count; ++i) {
    if (victims[i]) {
      SELFTEST_EXPECT(StartFpuHold(group, i));
    }
  }
  const TscDeadline seen_deadline(kSelfTestWaitTimeoutTscCycles);
  for (;;) {
    SELFTEST_EXPECT(SampleFpuHoldGroup(group));
    bool seen = true;
    for (int i = 0; i < group->count && seen; ++i) {
      seen = !victims[i] || group->last_cpu[i] >= 0;
    }
    if (seen) {
      return true;
    }
    SELFTEST_EXPECT(!seen_deadline.Expired());
    scheduler->Yield();
  }
}

// Checks that each process keeps its own x87/MMX/SSE/AVX state: twice as many
// `kFpuHold` processes as CPUs hold values that depend on their seeds in all
// x87/MMX registers, XMM0-XMM15, MXCSR and the x87 control word, and in the
// upper halves of YMM0-YMM15 if the kernel enabled the AVX state (the data
// registers differ between processes; the control word and MXCSR, which have
// few valid values, may coincide between two of them, but change every
// round), and check them continuously
// while the CPUs switch between them for kFpuIsolationTicksPerCpu timer ticks
// each and until each process took kFpuIsolationUserTicks timer interrupts in
// user mode. Two runnable processes per CPU keep every CPU busy, so no CPU
// takes work from another; the test then drains one CPU after another
// (DrainFpuHoldCpu()), so that processes that other CPUs preempted move to
// it, until it saw kFpuDrainMovesPerCpu moves per CPU (it requires one per
// CPU) and a process that checked 3 more rounds after it moved (it requires
// one), and finally lets every process check a few more rounds. No process
// may end before the test kills it, and afterwards every frame and heap byte
// is back.
static bool TestUserModeFpuIsolation() {
  TaskScheduler* const scheduler = GetTaskScheduler();
  SELFTEST_EXPECT(scheduler != nullptr);
  const int cpu_count = SmpCpuCount();
  SELFTEST_EXPECT(cpu_count >= 1 && cpu_count <= kMaxSupportedCpus);
  // User code sees the state components that the kernel enabled (XCR0), or
  // none if it keeps the state with FXSAVE.
  const uint64_t xcr0 = CpuGetConfig().xcr0;
  UserProgramResult features;
  SELFTEST_EXPECT(RunUserTest(usertest::Mode::kFpuFeatures, &features));
  SELFTEST_EXPECT(
      ExitedWith(features, static_cast<int64_t>(xcr0 & 0xFFFFFFFF)));
  const bool avx = (xcr0 & kXcr0Avx) != 0;

  // On the heap (kMaxFpuTestProcesses), before the heap baseline below.
  const std::unique_ptr<FpuHoldGroup> group_storage =
      std::make_unique<FpuHoldGroup>();
  FpuHoldGroup& group = *group_storage;
  group.count = 2 * cpu_count;
  // First as many processes as the test runs at once, so that the heap has
  // grown as far as the test needs before it records what is free.
  for (int i = 0; i < group.count; ++i) {
    SELFTEST_EXPECT(StartFpuHold(&group, i));
  }
  SELFTEST_EXPECT(WaitForProcesses(group.processes,  //
                                   group.count,      //
                                   HasRunUserCode,   //
                                   WaitMode::kYield));
  for (int i = 0; i < group.count; ++i) {
    group.processes[i]->Kill();
  }
  for (int i = 0; i < group.count; ++i) {
    UserProcessStats stats;
    SELFTEST_EXPECT(EndedKilled(group.processes[i], &stats));
    group.processes[i].reset();
  }
  const int64_t pmm_free_before = PmmFreeFrameCount();
  const int64_t heap_free_before = HeapTotalFreeBytes();
  const int64_t switches_before = TotalContextSwitches();
  int64_t ticks_before[kMaxSupportedCpus];
  for (int cpu = 0; cpu < cpu_count; ++cpu) {
    ticks_before[cpu] = CpuTimerTicks(cpu);
  }

  // Preemption: the CPUs switch between the processes.
  for (int i = 0; i < group.count; ++i) {
    SELFTEST_EXPECT(StartFpuHold(&group, i));
  }
  int64_t min_cpu_ticks = 0;
  int64_t min_user_ticks = 0;
  const TscDeadline deadline(kSelfTestWaitTimeoutTscCycles);
  for (;;) {
    SELFTEST_EXPECT(SampleFpuHoldGroup(&group));
    min_cpu_ticks = INT64_MAX;
    for (int cpu = 0; cpu < cpu_count; ++cpu) {
      const int64_t ticks = CpuTimerTicks(cpu) - ticks_before[cpu];
      min_cpu_ticks = (ticks < min_cpu_ticks) ? ticks : min_cpu_ticks;
    }
    min_user_ticks = INT64_MAX;
    for (int i = 0; i < group.count; ++i) {
      const int64_t ticks = group.processes[i]->stats().user_timer_ticks;
      min_user_ticks = (ticks < min_user_ticks) ? ticks : min_user_ticks;
    }
    // Every process was just started, so its step began at 0.
    if (min_cpu_ticks >= kFpuIsolationTicksPerCpu &&
        min_user_ticks >= kFpuIsolationUserTicks &&
        FpuHoldsMadeStep(group, group.step_syscalls)) {
      break;
    }
    SELFTEST_EXPECT(!deadline.Expired());
    scheduler->Yield();
  }

  // Moves between CPUs.
  int drain_rounds = 0;
  while (cpu_count > 1 && drain_rounds < kFpuDrainRoundsPerCpu * cpu_count &&
         (drain_rounds < cpu_count ||
          group.moves < kFpuDrainMovesPerCpu * cpu_count ||
          group.checked_migrations == 0)) {
    SELFTEST_EXPECT(DrainFpuHoldCpu(&group, (drain_rounds + 1) % cpu_count));
    ++drain_rounds;
  }
  RecordFpuHoldSyscalls(group, group.step_syscalls);
  const TscDeadline final_deadline(kSelfTestWaitTimeoutTscCycles);
  for (;;) {
    SELFTEST_EXPECT(SampleFpuHoldGroup(&group));
    if (FpuHoldsMadeStep(group, group.step_syscalls)) {
      break;
    }
    SELFTEST_EXPECT(!final_deadline.Expired());
    scheduler->Yield();
  }
  for (int i = 0; i < group.count; ++i) {
    group.processes[i]->Kill();
  }
  for (int i = 0; i < group.count; ++i) {
    UserProcessStats stats;
    SELFTEST_EXPECT(EndedKilled(group.processes[i], &stats));
    group.processes[i].reset();
  }
  const int64_t switches = TotalContextSwitches() - switches_before;

  UartWrite("[TEST] user_mode_fpu_isolation: processes=");
  UartWriteDec(static_cast<uint64_t>(group.count));
  UartWrite(" ticks_per_cpu>=");
  UartWriteDec(static_cast<uint64_t>(min_cpu_ticks));
  UartWrite(" user_ticks_per_process>=");
  UartWriteDec(static_cast<uint64_t>(min_user_ticks));
  UartWrite(" context_switches=");
  UartWriteDec(static_cast<uint64_t>(switches));
  UartWrite(" checked_migrations=");
  UartWriteDec(static_cast<uint64_t>(group.checked_migrations));
  UartWrite(" moves=");
  UartWriteDec(static_cast<uint64_t>(group.moves));
  UartWrite(" drain_rounds=");
  UartWriteDec(static_cast<uint64_t>(drain_rounds));
  UartWrite("\n");
  // Each process counted in `checked_migrations` checked at least 3 more
  // rounds after the test saw it on a second CPU.
  SELFTEST_EXPECT(cpu_count == 1 || group.checked_migrations >= 1);
  SELFTEST_EXPECT(cpu_count == 1 || group.moves >= cpu_count);
  UartWrite(kFpuCoveredLine);
  UartWrite(avx ? kFpuAvxCoveredLine : kFpuAvxNotCoveredLine);

  SELFTEST_EXPECT(PagingActiveRoot() == PagingKernelRoot());
  SELFTEST_EXPECT(PagingUserSpaceIsEmpty());
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  SELFTEST_EXPECT(HeapTotalFreeBytes() == heap_free_before);
  return true;
}

// Runs `count` `kFpuDirty` processes (seeds from `seed_base`), which keep
// values other than the initial ones in the x87/SSE/AVX registers, until each
// has made system calls and every CPU has run one, then `count`
// `kFpuInitialState` processes, which must all find that they started with
// the initial state, and then kills the `kFpuDirty` processes. Adds the CPUs
// on which the checkers ended to `*checker_cpus`.
static bool RunFpuInitialStateRound(const int count,           //
                                    const uint64_t seed_base,  //
                                    AtomicCpuMask128* const checker_cpus) {
  TaskScheduler* const scheduler = GetTaskScheduler();
  const int cpu_count = SmpCpuCount();
  // On an early return, the destructors kill the processes and wait for them.
  const std::unique_ptr<std::unique_ptr<UserProcess>[]> dirty =
      std::make_unique<std::unique_ptr<UserProcess>[]>(count);
  for (int i = 0; i < count; ++i) {
    dirty[i] = UserProcess::Start(
        UserTestProgram(), usertest::Arg(usertest::Mode::kFpuDirty,
                                         seed_base + static_cast<uint64_t>(i)));
    SELFTEST_EXPECT(dirty[i] != nullptr);
  }
  // Every CPU's registers hold values of a `kFpuDirty` process.
  AtomicCpuMask128 dirty_cpus;
  const TscDeadline deadline(kSelfTestWaitTimeoutTscCycles);
  for (;;) {
    bool done = true;
    for (int i = 0; i < count; ++i) {
      const UserProcessStats stats = dirty[i]->stats();
      if (HasEnded(stats)) {
        LogUnexpectedResult(dirty[i]->Wait());
        return false;
      }
      dirty_cpus.Add(dirty[i]->last_syscall_cpu());
      done = done && stats.syscalls >= 2;
    }
    if (done &&
        dirty_cpus.Load().ContainsAll(CpuMask128::FirstCpus(cpu_count))) {
      break;
    }
    SELFTEST_EXPECT(!deadline.Expired());
    scheduler->Yield();
  }
  // Live processes never share state memory, and the `kFpuDirty` processes
  // stay alive until they are killed below, so their states
  // (UserProcessStats::fpu_state) are pairwise distinct, and each checker's
  // differs from all of them: RunFpuStateReuseRound() relies on the value.
  const std::unique_ptr<uintptr_t[]> dirty_states =
      std::make_unique<uintptr_t[]>(count);
  for (int i = 0; i < count; ++i) {
    dirty_states[i] = dirty[i]->stats().fpu_state;
    SELFTEST_EXPECT(dirty_states[i] != 0);
    for (int j = 0; j < i; ++j) {
      SELFTEST_EXPECT(dirty_states[i] != dirty_states[j]);
    }
  }
  const std::unique_ptr<std::unique_ptr<UserProcess>[]> checkers =
      std::make_unique<std::unique_ptr<UserProcess>[]>(count);
  for (int i = 0; i < count; ++i) {
    checkers[i] = UserProcess::Start(
        UserTestProgram(), usertest::Arg(usertest::Mode::kFpuInitialState));
    SELFTEST_EXPECT(checkers[i] != nullptr);
  }
  for (int i = 0; i < count; ++i) {
    SELFTEST_EXPECT(ExitedWith(checkers[i]->Wait(), 0));
    // The CPU of its exit() call.
    checker_cpus->Add(checkers[i]->last_syscall_cpu());
    const uintptr_t checker_state = checkers[i]->stats().fpu_state;
    SELFTEST_EXPECT(checker_state != 0);
    for (int j = 0; j < count; ++j) {
      SELFTEST_EXPECT(checker_state != dirty_states[j]);
    }
  }
  for (int i = 0; i < count; ++i) {
    dirty[i]->Kill();
  }
  for (int i = 0; i < count; ++i) {
    UserProcessStats stats;
    SELFTEST_EXPECT(EndedKilled(dirty[i], &stats));
  }
  return true;
}

// RunFpuStateReuseRounds() runs kFpuReuseRoundsPerCpu rounds per CPU, and
// more, up to kMaxFpuReuseRoundsPerCpu rounds per CPU in all, until as many
// rounds as CPUs gave the checker both the CPU and the state memory of the
// process that ended before it: first rounds in which that process is killed,
// then as many in which it exits by itself.
constexpr int kFpuReuseRoundsPerCpu = 2;
constexpr int kMaxFpuReuseRoundsPerCpu = 8;
// The first seeds of its `kFpuDirty`, `kSpin` and `kFpuLongSyscall` processes.
constexpr uint64_t kFpuReuseDirtySeedBase = 0xD2D2'0000;
constexpr uint64_t kFpuReuseBusySeedBase = 0xB5B5'0000;
constexpr uint64_t kFpuReuseExitSeedBase = 0xE7E00;
static_assert(kFpuReuseExitSeedBase +
                  kMaxFpuReuseRoundsPerCpu * kMaxSupportedCpus <=
              usertest::kKernelSpinMaxSeed);

// How the process before the checker of RunFpuStateReuseRound() ends.
enum class FpuReuseEnd {
  // The kernel kills it (`kFpuDirty`).
  kKilled,
  // It exits by itself, with status 0, with values of its own still in the
  // registers (`kFpuLongSyscall`, one round).
  kExited,
};

// What RunFpuStateReuseRounds() observed for one FpuReuseEnd.
struct FpuReuseReport {
  int rounds = 0;
  // The rounds whose checker exited on the CPU of the latest system call of
  // the process that ended before it, whose registers still held that
  // process's values (nothing saves them when a process ends).
  int same_cpu = 0;
  // Those of them whose checker also got the memory of the ended process's
  // state (UserProcessStats::fpu_state), as the address-ordered first-fit
  // heap gives it once all of that process's memory is back (heap.h).
  int same_cpu_and_state = 0;
};

// Returns true once the program has made two system calls, or has ended.
static bool HasMadeTwoSyscallsOrEnded(const UserProcessStats& stats) {
  return stats.syscalls >= 2 || HasEnded(stats);
}

// Runs a process with `seed` that loads values other than the initial ones
// into the x87/SSE/AVX registers, and then ends as `end` says: a `kFpuDirty`
// process, killed once it has loaded its values, or a `kFpuLongSyscall`
// process, which checks its values in its one round and exits with them
// still in its x87 data registers (FNINIT, which ends the round with
// VZEROUPPER and the initial MXCSR, leaves those) and XMM registers. Then
// runs a `kFpuInitialState` process, which must find that it started with the
// initial state. Waits for the processes in `mode`, and counts the round in
// `*report`.
static bool RunFpuStateReuseRound(const FpuReuseEnd end,  //
                                  const uint64_t seed,    //
                                  const WaitMode mode,    //
                                  FpuReuseReport* const report) {
  int dirty_cpu = -1;
  uintptr_t dirty_state = 0;
  if (end == FpuReuseEnd::kKilled) {
    const std::unique_ptr<UserProcess> dirty = UserProcess::Start(
        UserTestProgram(), usertest::Arg(usertest::Mode::kFpuDirty, seed));
    SELFTEST_EXPECT(dirty != nullptr);
    // It makes system calls only after loading its values.
    SELFTEST_EXPECT(
        WaitForProcesses(&dirty, 1, HasMadeTwoSyscallsOrEnded, mode));
    if (HasEnded(dirty->stats())) {
      LogUnexpectedResult(dirty->Wait());
      return false;
    }
    dirty->Kill();
    UserProcessStats stats;
    SELFTEST_EXPECT(EndedKilled(dirty, &stats));
    dirty_cpu = dirty->last_syscall_cpu();
    dirty_state = stats.fpu_state;
  } else {
    // One spin() call of a tick between loading and checking the values.
    const std::unique_ptr<UserProcess> dirty = UserProcess::Start(
        UserTestProgram(), usertest::FpuLongSyscallArg(1, 1, seed));
    SELFTEST_EXPECT(dirty != nullptr);
    SELFTEST_EXPECT(WaitForProcesses(&dirty, 1, HasEnded, mode));
    SELFTEST_EXPECT(ExitedWith(dirty->Wait(), 0));
    // The CPU of its exit() call.
    dirty_cpu = dirty->last_syscall_cpu();
    dirty_state = dirty->stats().fpu_state;
  }
  const std::unique_ptr<UserProcess> checker = UserProcess::Start(
      UserTestProgram(), usertest::Arg(usertest::Mode::kFpuInitialState));
  SELFTEST_EXPECT(checker != nullptr);
  SELFTEST_EXPECT(WaitForProcesses(&checker, 1, HasEnded, mode));
  SELFTEST_EXPECT(ExitedWith(checker->Wait(), 0));
  ++report->rounds;
  // last_syscall_cpu(): the CPU of its exit() call, its only system call.
  if (dirty_cpu >= 0 && checker->last_syscall_cpu() == dirty_cpu) {
    ++report->same_cpu;
    if (dirty_state != 0 && checker->stats().fpu_state == dirty_state) {
      ++report->same_cpu_and_state;
    }
  }
  return true;
}

// Runs the rounds of RunFpuStateReuseRound() (see kFpuReuseRoundsPerCpu),
// those of FpuReuseEnd::kKilled into `*kill_report` and then those of
// FpuReuseEnd::kExited into `*exit_report`, while `kSpin` processes keep
// every CPU busy but CPU 0, which runs this task and waits without yielding,
// and one more, so that both processes of every round run on that one: a new
// task goes to the least loaded CPU, and a busy CPU takes a queued task only
// from a CPU with a load of 3 or more (work_stealing_scheduler.cpp).
// Precondition: no other tasks are runnable.
static bool RunFpuStateReuseRounds(const int cpu_count,
                                   FpuReuseReport* const kill_report,
                                   FpuReuseReport* const exit_report) {
  const int busy_count = (cpu_count > 2) ? cpu_count - 2 : 0;
  const WaitMode mode = (cpu_count > 1) ? WaitMode::kSpin : WaitMode::kYield;
  // On an early return, the destructors kill them and wait for them.
  const std::unique_ptr<std::unique_ptr<UserProcess>[]> busy =
      std::make_unique<std::unique_ptr<UserProcess>[]>(busy_count);
  SELFTEST_EXPECT(
      StartProcessesOnOwnCpus(busy.get(), busy_count, [](const int i) {
        return usertest::Arg(usertest::Mode::kSpin,
                             kFpuReuseBusySeedBase + static_cast<uint64_t>(i));
      }));
  // Runs the rounds of `end`, with seeds from `seed_base`.
  const auto run_rounds = [cpu_count, mode](const FpuReuseEnd end,
                                            const uint64_t seed_base,
                                            FpuReuseReport* const report) {
    for (int round = 0; round < kMaxFpuReuseRoundsPerCpu * cpu_count &&
                        (round < kFpuReuseRoundsPerCpu * cpu_count ||
                         report->same_cpu_and_state < cpu_count);
         ++round) {
      SELFTEST_EXPECT(RunFpuStateReuseRound(
          end, seed_base + static_cast<uint64_t>(round), mode, report));
    }
    return true;
  };
  SELFTEST_EXPECT(
      run_rounds(FpuReuseEnd::kKilled, kFpuReuseDirtySeedBase, kill_report));
  SELFTEST_EXPECT(
      run_rounds(FpuReuseEnd::kExited, kFpuReuseExitSeedBase, exit_report));
  for (int i = 0; i < busy_count; ++i) {
    busy[i]->Kill();
  }
  for (int i = 0; i < busy_count; ++i) {
    UserProcessStats stats;
    SELFTEST_EXPECT(EndedKilled(busy[i], &stats));
  }
  return true;
}

// Checks that a new process starts with the architectural initial x87, SSE
// and AVX state (user/crt.cpp saves it before any compiled code runs), even
// though the registers of every CPU hold the values of other processes:
// twice as many checkers as CPUs start while as many `kFpuDirty` processes
// keep every CPU's registers dirty, twice (the first round lets the heap
// grow), and all of them must exit with 0, each in state memory of its own;
// then checkers start right after a process that was killed, or that exited
// by itself, on its CPU and in its freed state memory
// (RunFpuStateReuseRounds()), and must exit with 0 too. Afterwards every
// frame and heap byte is back.
static bool TestUserModeFpuInitialState() {
  SELFTEST_EXPECT(GetTaskScheduler() != nullptr);
  const int count = 2 * SmpCpuCount();
  UserProgramResult result;
  SELFTEST_EXPECT(RunUserTest(usertest::Mode::kFpuInitialState, &result));
  SELFTEST_EXPECT(ExitedWith(result, 0));
  AtomicCpuMask128 checker_cpus;
  SELFTEST_EXPECT(
      RunFpuInitialStateRound(count, kFpuDirtySeedBase, &checker_cpus));
  const int64_t pmm_free_before = PmmFreeFrameCount();
  const int64_t heap_free_before = HeapTotalFreeBytes();
  SELFTEST_EXPECT(RunFpuInitialStateRound(
      count, kFpuDirtySeedBase + static_cast<uint64_t>(count), &checker_cpus));
  FpuReuseReport reuse;
  FpuReuseReport exit_reuse;
  SELFTEST_EXPECT(RunFpuStateReuseRounds(SmpCpuCount(), &reuse, &exit_reuse));

  UartWrite("[TEST] user_mode_fpu_initial_state: checked=");
  UartWriteDec(static_cast<uint64_t>(1 + 2 * count));
  UartWrite(" cpus=");
  UartWriteDec(static_cast<uint64_t>(CountCpus(checker_cpus.Load())));
  UartWrite("\n[TEST] user_mode_fpu_initial_state: reuse_rounds=");
  UartWriteDec(static_cast<uint64_t>(reuse.rounds));
  UartWrite(" same_cpu=");
  UartWriteDec(static_cast<uint64_t>(reuse.same_cpu));
  UartWrite(" same_cpu_and_state=");
  UartWriteDec(static_cast<uint64_t>(reuse.same_cpu_and_state));
  UartWrite("\n[TEST] user_mode_fpu_initial_state: exit_reuse_rounds=");
  UartWriteDec(static_cast<uint64_t>(exit_reuse.rounds));
  UartWrite(" same_cpu=");
  UartWriteDec(static_cast<uint64_t>(exit_reuse.same_cpu));
  UartWrite(" same_cpu_and_state=");
  UartWriteDec(static_cast<uint64_t>(exit_reuse.same_cpu_and_state));
  UartWrite("\n");

  // Each such round fails on a kernel that leaves a new process the registers
  // of the process that ended before it on its CPU, for example by skipping
  // the load of a state at the address of the state that it loaded last there;
  // the second kind also where only a kill makes it forget that state.
  SELFTEST_EXPECT(reuse.same_cpu_and_state >= SmpCpuCount());
  SELFTEST_EXPECT(exit_reuse.same_cpu_and_state >= SmpCpuCount());
  SELFTEST_EXPECT(PagingActiveRoot() == PagingKernelRoot());
  SELFTEST_EXPECT(PagingUserSpaceIsEmpty());
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  SELFTEST_EXPECT(HeapTotalFreeBytes() == heap_free_before);
  return true;
}

// What CpuProtectionFeaturesWorker() checks against, and what it found.
struct CpuProtectionFeaturesContext {
  CpuConfig config;
  AtomicCpuMask128 matching_cpus;
  AtomicCpuMask128 mismatching_cpus;
};

// Executes CPUID with `leaf` in EAX and 0 in ECX on the calling CPU, apart
// from cpu.cpp's reader.
static CpuidResult CpuidForProtectionFeatures(const uint32_t leaf) {
  CpuidResult result;
  asm volatile("cpuid"
               : "=a"(result.eax), "=b"(result.ebx), "=c"(result.ecx),
                 "=d"(result.edx)
               : "a"(leaf), "c"(0));
  return result;
}

static uint64_t ReadCr4ForProtectionFeatures() {
  uint64_t cr4 = 0;
  asm volatile("mov %0, cr4" : "=r"(cr4));
  return cr4;
}

// Returns XCR0. Raises #UD while CR4.OSXSAVE is clear.
static uint64_t ReadXcr0ForProtectionFeatures() {
  uint32_t low = 0;
  uint32_t high = 0;
  asm volatile("xgetbv" : "=a"(low), "=d"(high) : "c"(0));
  return (static_cast<uint64_t>(high) << 32) | low;
}

// Checks on the calling CPU that CR4.SMEP and CR4.SMAP are set exactly if the
// CPU's own CPUID reports SMEP and SMAP, as the configuration says, that the
// SMAP flag of the entry stubs and the user-copy helpers matches CR4.SMAP,
// that CR4.OSXSAVE and XCR0 are the configuration's, and that OSFXSR and
// OSXMMEXCPT are set and FSGSBASE clear.
static void CpuProtectionFeaturesWorker(const int cpu_index,
                                        void* const raw_context) {
  CpuProtectionFeaturesContext* const context =
      static_cast<CpuProtectionFeaturesContext*>(raw_context);
  const CpuConfig& config = context->config;
  // Leaf 7 only if CPUID has it: Intel CPUs answer a higher leaf with the
  // highest basic leaf's values.
  const uint32_t leaf7_ebx = (CpuidForProtectionFeatures(0).eax >= 7)
                                 ? CpuidForProtectionFeatures(7).ebx
                                 : 0;
  const bool has_smep = (leaf7_ebx & kCpuid7EbxSmep) != 0;
  const bool has_smap = (leaf7_ebx & kCpuid7EbxSmap) != 0;
  const uint64_t cr4 = ReadCr4ForProtectionFeatures();
  const bool smep_on = (cr4 & kCr4Smep) != 0;
  const bool smap_on = (cr4 & kCr4Smap) != 0;
  const bool osxsave = (cr4 & kCr4Osxsave) != 0;
  constexpr uint64_t kSse = kCr4Osfxsr | kCr4Osxmmexcpt;
  const bool matches =
      smep_on == has_smep && has_smep == config.smep && smap_on == has_smap &&
      has_smap == config.smap && CpuSmapEnabled() == smap_on &&
      osxsave == config.xsave &&
      (!osxsave || ReadXcr0ForProtectionFeatures() == config.xcr0) &&
      (cr4 & kSse) == kSse && (cr4 & kCr4Fsgsbase) == 0;
  if (matches) {
    context->matching_cpus.Add(cpu_index);
  } else {
    context->mismatching_cpus.Add(cpu_index);
  }
}

// Every CPU that has SMEP and/or SMAP runs with them enabled, and the CR4
// and XCR0 of every CPU hold the configuration that CpuInitBsp() chose from
// the BSP's CPUID.
static bool TestCpuProtectionFeatures() {
  const int cpu_count = SmpCpuCount();
  CpuProtectionFeaturesContext context;
  context.config = CpuGetConfig();
  // CpuInitBsp() chose it.
  SELFTEST_EXPECT(context.config.fxsr);
  SmpRunOnAllCpus(CpuProtectionFeaturesWorker, &context);
  const CpuMask128 matching = context.matching_cpus.Load();

  char hex[kFormatHexBufferSize];
  LineBuffer line;
  line.Append("[TEST] cpu_protection_features: cpus=")
      .AppendDec(static_cast<uint64_t>(CountCpus(matching)))
      .Append(context.config.smep ? " smep=on" : " smep=off")
      .Append(context.config.smap ? " smap=on" : " smap=off")
      .Append(" xcr0=")
      .Append(context.config.xsave ? FormatHex(context.config.xcr0, hex)
                                   : "none")
      .Append("\n");
  UartWrite(line.c_str());

  SELFTEST_EXPECT(CountCpus(context.mismatching_cpus.Load()) == 0);
  SELFTEST_EXPECT(matching.ContainsAll(CpuMask128::FirstCpus(cpu_count)));
  return true;
}

// The kernel accesses to user memory outside the user-copy helpers that the
// probe tasks of TestSmapSmepProtection() make on purpose. Each must
// page-fault.
enum class UserPageProbe : int {
  // Reads an unmapped user address: shows that the probe works, also on CPUs
  // without SMAP and SMEP.
  kNotPresentRead,
  // Reads the mapped user data page (SMAP).
  kSmapRead,
  // Writes the mapped user data page (SMAP).
  kSmapWrite,
  // Calls the `ret` at the start of the user code page (SMEP).
  kSmepExecute,
};
constexpr int kUserPageProbeKinds = 4;
constexpr const char* kUserPageProbeNames[kUserPageProbeKinds] = {
    "not-present", "smap read", "smap write", "smep exec"};
// The rounds of probes that each probe task runs, yielding after each.
constexpr int kUserPageProbeRounds = 8;
// The most batches of probe tasks: one task per CPU, then one per CPU that no
// task probed on yet.
constexpr int kMaxUserPageProbeBatches = 4;
// The probe address space: a user data page, the unmapped page above it, and
// a user code page in another page table.
constexpr uintptr_t kProbeDataPage = kUserSpaceBase;
constexpr uintptr_t kProbeUnmappedPage = kProbeDataPage + kPageSize;
constexpr uintptr_t kProbeCodePage = kUserSpaceBase + kHugePageSize;
// Each probe task has a slot of this many bytes in the data page: a word that
// it reads with CopyFromUser(), a word that it writes with CopyToUser(), and
// the byte that its write probe aims at.
constexpr int kProbeSlotSize = 32;
static_assert(kPageSize / kProbeSlotSize >= kMaxSupportedCpus);
constexpr int kProbeWriteOffset = 16;
constexpr uint64_t kProbeDataPattern = 0x5AFE5AFE00000000ULL;
constexpr uint8_t kProbeByteBefore = 0x5A;
constexpr uint8_t kProbeWriteValue = 0xA5;
constexpr uint8_t kRetInstruction = 0xC3;
// The page-fault error code bits that the probes check: present page, write,
// user mode, reserved bit and instruction fetch (not protection keys, shadow
// stacks or SGX).
constexpr uint64_t kPageFaultReservedBit = 1 << 3;
constexpr uint64_t kPageFaultCheckedBits =
    kPageFaultAccessBits | kPageFaultReservedBit | kPageFaultInstructionFetch;

// Probed functions (RunFaultProbe()): a kernel read and a kernel write of the
// byte at `address`.
static void ReadUserByteProbe(const uintptr_t address) {
  static_cast<void>(*reinterpret_cast<const volatile uint8_t*>(address));
}

static void WriteUserByteProbe(const uintptr_t address) {
  *reinterpret_cast<volatile uint8_t*>(address) = kProbeWriteValue;
}

// What a probe accesses, with which function, and the error code of the page
// fault that it must get.
struct UserPageProbeTarget {
  uintptr_t address = 0;
  void (*fn)(uintptr_t) = nullptr;
  uint64_t error_code = 0;
};

// Returns the target of `probe` for the probe task with slot `slot`.
static UserPageProbeTarget UserPageProbeTargetOf(const UserPageProbe probe,
                                                 const int slot) {
  const uintptr_t slot_offset = static_cast<uintptr_t>(slot * kProbeSlotSize);
  UserPageProbeTarget target;
  switch (probe) {
    case UserPageProbe::kNotPresentRead:
      target.address = kProbeUnmappedPage + slot_offset;
      target.fn = ReadUserByteProbe;
      target.error_code = 0;
      break;
    case UserPageProbe::kSmapRead:
      target.address = kProbeDataPage + slot_offset;
      target.fn = ReadUserByteProbe;
      target.error_code = kPageFaultPresent;
      break;
    case UserPageProbe::kSmapWrite:
      target.address = kProbeDataPage + slot_offset + kProbeWriteOffset;
      target.fn = WriteUserByteProbe;
      target.error_code = kPageFaultPresent | kPageFaultWrite;
      break;
    case UserPageProbe::kSmepExecute:
      target.address = kProbeCodePage;
      target.fn = reinterpret_cast<void (*)(uintptr_t)>(kProbeCodePage);
      target.error_code = kPageFaultPresent | kPageFaultInstructionFetch;
      break;
  }
  return target;
}

// What the probe tasks observed, all of them together.
struct UserPageProbeCounters {
  // The CPUs where each kind of probe page-faulted as expected, and the error
  // code of the last such fault.
  AtomicCpuMask128 expected[kUserPageProbeKinds];
  std::atomic<uint64_t> error_codes[kUserPageProbeKinds];
  std::atomic<int> probes{0};
  std::atomic<int> unexpected{0};
  std::atomic<int> rounds{0};
  // Rounds that started while a watched program was in a system call.
  std::atomic<int> rounds_during_syscalls{0};
  // Rounds whose copies through the user-copy helpers worked.
  std::atomic<int> copies{0};
  // Probe tasks that finished.
  std::atomic<int> done{0};
};

// What the probe tasks share.
struct UserPageProbeSetup {
  // The probe address space, and the frame of its data page.
  AddressSpace* space = nullptr;
  uintptr_t data_frame = 0;
  // Which probes run, besides the not-present one.
  bool smap = false;
  bool smep = false;
  // Programs whose system calls the probes run alongside (maybe none).
  const std::unique_ptr<UserProcess>* watched = nullptr;
  int watched_count = 0;
};

struct UserPageProbeTaskArg {
  const UserPageProbeSetup* setup = nullptr;
  int slot = 0;
  UserPageProbeCounters* counters = nullptr;
};

// Runs `probe` with a fault probe and counts whether it page-faulted as
// expected: at its address, with its error code, and with the RIP of the
// access (in the kernel, or the user code page's for the SMEP probe). Logs
// any other result.
static void RunCountedUserPageProbe(const UserPageProbe probe,
                                    const UserPageProbeTaskArg& arg) {
  const UserPageProbeTarget target = UserPageProbeTargetOf(probe, arg.slot);
  const FaultProbeResult result = RunFaultProbe(target.fn, target.address);
  UserPageProbeCounters* const counters = arg.counters;
  counters->probes.fetch_add(1, std::memory_order_relaxed);
  const int kind = static_cast<int>(probe);
  const bool rip_ok = (probe == UserPageProbe::kSmepExecute)
                          ? result.rip == target.address
                          : !IsUserRange(result.rip, 1);
  if (result.faulted && result.address == target.address &&
      (result.error_code & kPageFaultCheckedBits) == target.error_code &&
      rip_ok) {
    counters->expected[kind].Add(result.cpu);
    counters->error_codes[kind].store(result.error_code,
                                      std::memory_order_relaxed);
    return;
  }
  counters->unexpected.fetch_add(1, std::memory_order_relaxed);
  // Whole lines, which probes failing on other CPUs at the same time can't
  // split.
  LineBuffer line;
  line.Append("[TEST] smap_smep_protection: unexpected ")
      .Append(kUserPageProbeNames[kind])
      .Append(" probe result: cpu=")
      .AppendDec(static_cast<uint64_t>(result.cpu))
      .Append(result.faulted ? " faulted=1\n" : " faulted=0\n");
  UartWrite(line.c_str());
  char error_code_hex[kFormatHexBufferSize];
  char address_hex[kFormatHexBufferSize];
  char rip_hex[kFormatHexBufferSize];
  LineBuffer details;
  details.Append("[TEST] smap_smep_protection: err=")
      .Append(FormatHex(result.error_code, error_code_hex))
      .Append(" cr2=")
      .Append(FormatHex(result.address, address_hex))
      .Append(" rip=")
      .Append(FormatHex(result.rip, rip_hex))
      .Append("\n");
  UartWrite(details.c_str());
}

// One round of probes on whichever CPU runs the calling task, which runs in
// the probe address space: first the intended accesses to its slot through
// the user-copy helpers, which must work, then the not-present probe and the
// probes of the enabled protections, each of which must page-fault. The
// per-CPU probe helper of TestSmapSmepProtection(), for other tests too.
static void RunUserPageProbeRound(const UserPageProbeTaskArg& arg,
                                  const int round) {
  const UserPageProbeSetup& setup = *arg.setup;
  UserPageProbeCounters* const counters = arg.counters;
  bool in_syscall = false;
  for (int i = 0; i < setup.watched_count; ++i) {
    in_syscall = in_syscall || setup.watched[i]->stats().in_syscall;
  }
  counters->rounds.fetch_add(1, std::memory_order_relaxed);
  if (in_syscall) {
    counters->rounds_during_syscalls.fetch_add(1, std::memory_order_relaxed);
  }

  // The slot through its user address and, to check, through the identity
  // map.
  const uintptr_t offset = static_cast<uintptr_t>(arg.slot * kProbeSlotSize);
  const uintptr_t read_address = kProbeDataPage + offset;
  const uintptr_t write_address = read_address + sizeof(uint64_t);
  const volatile uint64_t* const words =
      reinterpret_cast<const volatile uint64_t*>(setup.data_frame + offset);
  uint64_t read_word = 0;
  const bool copied_in =
      CopyFromUser(&read_word, read_address, sizeof(read_word));
  const uint64_t written_word = read_word ^ static_cast<uint64_t>(round + 1);
  const bool copied_out =
      CopyToUser(write_address, &written_word, sizeof(written_word));
  if (copied_in && copied_out &&
      read_word == (kProbeDataPattern | static_cast<uint64_t>(arg.slot)) &&
      words[1] == written_word && !UserAccessWindowOpen()) {
    counters->copies.fetch_add(1, std::memory_order_relaxed);
  }

  RunCountedUserPageProbe(UserPageProbe::kNotPresentRead, arg);
  if (setup.smap) {
    RunCountedUserPageProbe(UserPageProbe::kSmapRead, arg);
    RunCountedUserPageProbe(UserPageProbe::kSmapWrite, arg);
    // The write that faulted must not have happened.
    const volatile uint8_t* const write_target =
        reinterpret_cast<const volatile uint8_t*>(setup.data_frame + offset +
                                                  kProbeWriteOffset);
    if (*write_target != kProbeByteBefore) {
      counters->unexpected.fetch_add(1, std::memory_order_relaxed);
    }
  }
  if (setup.smep) {
    RunCountedUserPageProbe(UserPageProbe::kSmepExecute, arg);
  }
}

// Returns true if every probe kind enabled in `arg.setup` has faulted as
// expected on every CPU in `mask`.
static bool UserPageProbesCoveredOn(const UserPageProbeTaskArg& arg,
                                    const CpuMask128& mask) {
  const UserPageProbeSetup& setup = *arg.setup;
  const UserPageProbeCounters& counters = *arg.counters;
  if (!counters.expected[static_cast<int>(UserPageProbe::kNotPresentRead)]
           .Load()
           .ContainsAll(mask)) {
    return false;
  }
  if (setup.smap &&
      (!counters.expected[static_cast<int>(UserPageProbe::kSmapRead)]
            .Load()
            .ContainsAll(mask) ||
       !counters.expected[static_cast<int>(UserPageProbe::kSmapWrite)]
            .Load()
            .ContainsAll(mask))) {
    return false;
  }
  if (setup.smep &&
      !counters.expected[static_cast<int>(UserPageProbe::kSmepExecute)]
           .Load()
           .ContainsAll(mask)) {
    return false;
  }
  return true;
}

static bool CurrentCpuUserPageProbesCovered(const UserPageProbeTaskArg& arg) {
  const int cpu = CurrentCpuId();
  CpuMask128 current;
  if (cpu >= 0 && cpu < 64) {
    current.low = 1ULL << static_cast<uint32_t>(cpu);
  } else if (cpu >= 64 && cpu < 128) {
    current.high = 1ULL << static_cast<uint32_t>(cpu - 64);
  }
  return UserPageProbesCoveredOn(arg, current);
}

static bool UserPageProbesStarted(const UserPageProbeTaskArg& arg,
                                  const CpuMask128& all_cpus,
                                  const TscDeadline& start_deadline) {
  return UserPageProbesCoveredOn(arg, all_cpus) || start_deadline.Expired();
}

// Runs kUserPageProbeRounds rounds in the probe address space.
static void UserPageProbeTask(void* const raw_arg) {
  const UserPageProbeTaskArg* const arg =
      static_cast<const UserPageProbeTaskArg*>(raw_arg);
  TaskScheduler* const scheduler = GetTaskScheduler();
  const CpuMask128 all_cpus = CpuMask128::FirstCpus(SmpCpuCount());
  const TscDeadline start_deadline(kSelfTestWaitTimeoutTscCycles /
                                   kMaxUserPageProbeBatches);
  scheduler->SetAddressSpace(arg->setup->space);
  for (int round = 0; round < kUserPageProbeRounds; ++round) {
    RunUserPageProbeRound(*arg, round);
    // Start barrier: keep probe tasks alive (so covered CPUs keep a queued task
    // alongside their watched user process and do not steal queued probe tasks
    // from host-descheduled APs) until every CPU has recorded its probes or the
    // per-batch start deadline expires.
    while (round == 0 &&
           !UserPageProbesStarted(*arg, all_cpus, start_deadline)) {
      scheduler->Yield();
      if (!CurrentCpuUserPageProbesCovered(*arg)) {
        RunUserPageProbeRound(*arg, round);
      }
    }
    scheduler->Yield();
  }
  scheduler->SetAddressSpace(nullptr);
  arg->counters->done.fetch_add(1, std::memory_order_release);
}

// The CPU of the probe task that RunUserPageProbesOnEveryCpu() places at
// countdown index `i` (cpu_count - 1 down to 0) of a batch: CPUs
// cpu_count - 1 down to 2, then CPU 0, then CPU 1.
static int UserPageProbeTaskCpu(const int cpu_count, const int i) {
  return (cpu_count > 1 && i <= 1) ? (1 - i) : i;
}

// Runs probe tasks until each CPU ran the enabled probes as expected, in at
// most kMaxUserPageProbeBatches batches; stops early once every enabled probe
// kind has covered every CPU. One task per CPU (CreateTaskOnCpu()) per batch,
// in descending CPU order (with CPU 0 before CPU 1) so single-process CPUs
// have a queued probe task before CPU 0 and CPU 1 reach a load that
// timer-interrupt work stealing would take from. Yields on CPU 0 until CPU 0
// is covered, then waits by spinning so CPU 0 doesn't take the tasks queued on
// other CPUs. Fails if a task can't be created or a wait times out.
static bool RunUserPageProbesOnEveryCpu(const UserPageProbeSetup& setup,
                                        UserPageProbeCounters* const counters) {
  TaskScheduler* const scheduler = GetTaskScheduler();
  const int cpu_count = SmpCpuCount();
  const CpuMask128 all = CpuMask128::FirstCpus(cpu_count);
  const std::unique_ptr<UserPageProbeTaskArg[]> args(
      new UserPageProbeTaskArg[cpu_count]);
  SELFTEST_EXPECT(args != nullptr);
  for (int batch = 0; batch < kMaxUserPageProbeBatches; ++batch) {
    if (UserPageProbesCoveredOn(UserPageProbeTaskArg{&setup, 0, counters},
                                all)) {
      break;
    }
    const int done_before = counters->done.load(std::memory_order_acquire);
    int started = 0;
    PhaseTaskGroup tasks(*scheduler, nullptr);
    for (int i = cpu_count - 1; i >= 0; --i) {
      const int cpu = UserPageProbeTaskCpu(cpu_count, i);
      args[cpu] = UserPageProbeTaskArg{&setup, cpu, counters};
      SELFTEST_EXPECT(tasks.Add(scheduler->CreateTaskOnCpu(
          UserPageProbeTask, &args[cpu], kDefaultTaskWeight, cpu)));
      ++started;
    }
    // The caller is the bootstrap task, which never leaves CPU 0, so the
    // coverage check of the current CPU below checks CPU 0.
    DCHECK(CurrentCpuId() == 0);
    const TscDeadline deadline(kSelfTestWaitTimeoutTscCycles);
    while (counters->done.load(std::memory_order_acquire) <
           done_before + started) {
      SELFTEST_EXPECT(!deadline.Expired());
      if (!CurrentCpuUserPageProbesCovered(args[0])) {
        scheduler->Yield();
      } else {
        asm volatile("pause" : : : "memory");
      }
    }
    tasks.JoinAll();
  }
  return true;
}

// Set by Int3UserAccessWindowHandler().
std::atomic<int> g_user_access_window_int3_hits{0};
std::atomic<bool> g_user_access_window_int3_ok{false};

// Handler of the int3 of ProbeInterruptInUserAccessWindow(): the entry stub
// must have closed the user-access window, and the frame must hold the
// interrupted code's RFLAGS.AC.
static void Int3UserAccessWindowHandler(InterruptFrame* const frame) {
  g_user_access_window_int3_ok.store(
      !UserAccessWindowOpen() &&
          (frame->rflags & kRflagsAlignmentCheckBit) != 0,
      std::memory_order_release);
  g_user_access_window_int3_hits.fetch_add(1, std::memory_order_acq_rel);
}

// With SMAP: takes an interrupt (int3) with the user-access window open, as
// an interrupt in the middle of a user copy does, and checks that the
// handler ran with the window closed and that iretq opened it again. Touches
// no user memory. Sets `*checked` to whether it ran (STAC raises #UD without
// SMAP).
static bool ProbeInterruptInUserAccessWindow(bool* const checked) {
  *checked = CpuSmapEnabled();
  if (!*checked) {
    return true;
  }
  g_user_access_window_int3_hits.store(0, std::memory_order_release);
  g_user_access_window_int3_ok.store(false, std::memory_order_release);
  const InterruptHandlerFn previous_handler = IdtGetHandler(kVectorBreakpoint);
  IdtRegisterHandler(kVectorBreakpoint, Int3UserAccessWindowHandler);
  uint64_t rflags_after = 0;
  asm volatile(
      "stac\n\t"
      "int3\n\t"
      "pushfq\n\t"
      "pop %0\n\t"
      "clac"
      : "=r"(rflags_after)
      :
      : "memory", "cc");
  if (previous_handler != nullptr) {
    IdtRegisterHandler(kVectorBreakpoint, previous_handler);
  } else {
    IdtUnregisterHandler(kVectorBreakpoint);
  }
  SELFTEST_EXPECT(
      g_user_access_window_int3_hits.load(std::memory_order_acquire) == 1);
  SELFTEST_EXPECT(g_user_access_window_int3_ok.load(std::memory_order_acquire));
  SELFTEST_EXPECT((rflags_after & kRflagsAlignmentCheckBit) != 0);
  SELFTEST_EXPECT(!UserAccessWindowOpen());
  return true;
}

// Set by NmiUserAccessWindowHandler(): the NMIs that interrupted code with the
// user-access window open (RFLAGS.AC set in their frame), and whether the
// handler ever ran with the window open. It then calls the handler that it
// replaced (SmpInit()'s, which parks the CPU on a panic stop).
std::atomic<int> g_user_access_window_nmi_hits{0};
std::atomic<bool> g_user_access_window_nmi_open{false};
std::atomic<InterruptHandlerFn> g_replaced_nmi_handler{nullptr};

// NMI handler of ProbeNmiInUserAccessWindow(): the entry stub of the NMI
// (isr_paranoid_stub, boot.S) must have closed the user-access window, and
// the frame holds the interrupted code's RFLAGS.AC.
static void NmiUserAccessWindowHandler(InterruptFrame* const frame) {
  if (UserAccessWindowOpen()) {
    g_user_access_window_nmi_open.store(true, std::memory_order_release);
  }
  if ((frame->rflags & kRflagsAlignmentCheckBit) != 0) {
    g_user_access_window_nmi_hits.fetch_add(1, std::memory_order_acq_rel);
  }
  const InterruptHandlerFn replaced =
      g_replaced_nmi_handler.load(std::memory_order_acquire);
  if (replaced != nullptr) {
    replaced(frame);
  }
}

// What NmiWindowTask() and ProbeNmiInUserAccessWindow() share.
struct NmiWindowTaskArg {
  // The CPU of the task once it has opened the window, else -1.
  std::atomic<int> cpu{-1};
  // Set by the probe once it stops waiting, and by the task once it has
  // closed the window again.
  std::atomic<bool> abandon{false};
  std::atomic<bool> done{false};
  // RFLAGS right before the task closed the window.
  std::atomic<uint64_t> rflags_after{0};
};

// Opens the user-access window with interrupts disabled, as a user copy keeps
// it open (with interrupts enabled), until an NMI has interrupted it there,
// the probe gives up or kSelfTestWaitTimeoutTscCycles pass, and closes it
// again. Touches no user memory.
static void NmiWindowTask(void* const raw_arg) {
  NmiWindowTaskArg* const arg = static_cast<NmiWindowTaskArg*>(raw_arg);
  const bool interrupts_enabled = SaveAndDisableInterrupts();
  const TscDeadline deadline(kSelfTestWaitTimeoutTscCycles);
  asm volatile("stac" : : : "memory", "cc");
  arg->cpu.store(CurrentCpuIdIrqsOff(), std::memory_order_release);
  while (g_user_access_window_nmi_hits.load(std::memory_order_acquire) == 0 &&
         !arg->abandon.load(std::memory_order_acquire) && !deadline.Expired()) {
    asm volatile("pause" : : : "memory");
  }
  uint64_t rflags = 0;
  asm volatile(
      "pushfq\n\t"
      "pop %0\n\t"
      "clac"
      : "=r"(rflags)
      :
      : "memory", "cc");
  arg->rflags_after.store(rflags, std::memory_order_relaxed);
  RestoreInterrupts(interrupts_enabled);
  arg->done.store(true, std::memory_order_release);
}

// With SMAP, on more than one CPU: takes an NMI with the user-access window
// open, as an NMI in the middle of a user copy does, and checks that its
// handler ran with the window closed and that iretq opened it again: a task
// on another CPU keeps the window open with interrupts disabled until the NMI
// that this task sends it arrives. Every wait is bounded. Sets `*checked` to
// whether it ran.
static bool ProbeNmiInUserAccessWindow(bool* const checked) {
  *checked = CpuSmapEnabled() && SmpCpuCount() > 1;
  if (!*checked) {
    return true;
  }
  TaskScheduler* const scheduler = GetTaskScheduler();
  SELFTEST_EXPECT(scheduler != nullptr);
  g_user_access_window_nmi_hits.store(0, std::memory_order_release);
  g_user_access_window_nmi_open.store(false, std::memory_order_release);
  const InterruptHandlerFn replaced = IdtGetHandler(kVectorNmi);
  g_replaced_nmi_handler.store(replaced, std::memory_order_release);
  IdtRegisterHandler(kVectorNmi, NmiUserAccessWindowHandler);
  NmiWindowTaskArg arg;
  bool sent = false;
  {
    // Joins the task when it goes.
    PhaseTaskGroup tasks(*scheduler, nullptr);
    // Any CPU but this one, which sends the NMI.
    const int window_cpu = (CurrentCpuId() + 1) % SmpCpuCount();
    Task* const task = scheduler->CreateTaskOnCpu(
        NmiWindowTask, &arg, kDefaultTaskWeight, window_cpu);
    if (tasks.Add(task)) {
      const TscDeadline deadline(kSelfTestWaitTimeoutTscCycles);
      while (arg.cpu.load(std::memory_order_acquire) < 0 &&
             !deadline.Expired()) {
        asm volatile("pause" : : : "memory");
      }
      const int cpu = arg.cpu.load(std::memory_order_acquire);
      if (cpu >= 0 && cpu != CurrentCpuId()) {
        SmpSendNmi(cpu);
        sent = true;
      }
      while (!arg.done.load(std::memory_order_acquire) && !deadline.Expired()) {
        asm volatile("pause" : : : "memory");
      }
      arg.abandon.store(true, std::memory_order_release);
    }
  }
  if (replaced != nullptr) {
    IdtRegisterHandler(kVectorNmi, replaced);
  } else {
    IdtUnregisterHandler(kVectorNmi);
  }
  SELFTEST_EXPECT(sent);
  SELFTEST_EXPECT(arg.done.load(std::memory_order_acquire));
  SELFTEST_EXPECT(
      g_user_access_window_nmi_hits.load(std::memory_order_acquire) >= 1);
  SELFTEST_EXPECT(
      !g_user_access_window_nmi_open.load(std::memory_order_acquire));
  SELFTEST_EXPECT((arg.rflags_after.load(std::memory_order_relaxed) &
                   kRflagsAlignmentCheckBit) != 0);
  SELFTEST_EXPECT(!UserAccessWindowOpen());
  return true;
}

// The spin() programs of TestSmapSmepProtection(): timer ticks per call, and
// the seed of the first.
constexpr int64_t kProbeSyscallTicks = 2;
constexpr uint64_t kProbeTestSeed = 0x5A1;

// Creates the address space of the probe tasks (UserPageProbeSetup): the user
// data page, whose slots for the first `slots` probe tasks hold their pattern
// and the byte that their write probe aims at, and the user code page, which
// starts with `ret`. Stores the frame of the data page in `*data_frame`.
// Returns nullptr if memory ran out.
static std::unique_ptr<AddressSpace> CreateUserPageProbeSpace(
    const int slots, uintptr_t* const data_frame) {
  std::unique_ptr<AddressSpace> space = AddressSpace::Create();
  if (space == nullptr) {
    return nullptr;
  }
  const uintptr_t data =
      space->MapNewPage(kProbeDataPage, UserPageAccess::kReadWrite);
  // No NX: an NX fault would have the error code of a SMEP fault.
  const uintptr_t code =
      space->MapNewPage(kProbeCodePage, UserPageAccess::kReadExecute);
  if (data == 0 || code == 0) {
    return nullptr;
  }
  // Through the identity map: the kernel may not write the pages through
  // their user addresses (SMAP).
  for (int slot = 0; slot < slots; ++slot) {
    const uintptr_t slot_frame =
        data + static_cast<uintptr_t>(slot * kProbeSlotSize);
    *reinterpret_cast<volatile uint64_t*>(slot_frame) =
        kProbeDataPattern | static_cast<uint64_t>(slot);
    *reinterpret_cast<volatile uint8_t*>(slot_frame + kProbeWriteOffset) =
        kProbeByteBefore;
  }
  *reinterpret_cast<volatile uint8_t*>(code) = kRetInstruction;
  *data_frame = data;
  return space;
}

// Prints the line of `probe` (UserPageProbe) of TestSmapSmepProtection():
// on how many of the `cpu_count` CPUs it page-faulted as expected, and with
// which error code.
static void LogUserPageProbe(const UserPageProbeCounters& counters,  //
                             const UserPageProbe probe,              //
                             const int cpu_count) {
  const int kind = static_cast<int>(probe);
  UartWrite("[TEST] smap_smep_protection: ");
  UartWrite(kUserPageProbeNames[kind]);
  UartWrite(probe == UserPageProbe::kNotPresentRead
                ? " page fault recovered on "
                : " probe recovered on ");
  UartWriteDec(
      static_cast<uint64_t>(CountCpus(counters.expected[kind].Load())));
  UartWrite("/");
  UartWriteDec(static_cast<uint64_t>(cpu_count));
  UartWrite(" CPUs (error code ");
  UartWriteHex(counters.error_codes[kind].load(std::memory_order_relaxed));
  UartWrite(")\n");
}

// With SMAP enabled, the kernel touches user memory only through the
// user-copy helpers, and with SMEP enabled it never executes user pages. One
// spin() program per CPU loops on system calls that read user buffers
// (syscall 3), and a kSyscalls program makes write() and spin() calls, while
// a probe task on every CPU runs rounds of probes in an address space with a
// user data page and a user code page: kernel reads and writes of the data
// page outside the helpers page-fault (SMAP), a kernel call into the code
// page page-faults (SMEP), each with the architectural error code, address
// and RIP, and the probe recovers without a panic; the helpers reach the
// same page; and a read of an unmapped user address page-faults on every
// pass, which checks the probe itself. The programs are in the middle of
// their calls during the probes and switched out in the middle of some of
// them, keep making calls afterwards and never see a wrong result; the
// kSyscalls program exits with 0. An interrupt (int3), and an NMI that
// another CPU sends, with the user-access window open run their handlers with
// the window closed and give it back. Probes of a protection that the CPU
// lacks are skipped (FAIL-free lines). Afterwards everything is back.
static bool TestSmapSmepProtection() {
  TaskScheduler* const scheduler = GetTaskScheduler();
  SELFTEST_EXPECT(scheduler != nullptr);
  const CpuConfig& config = CpuGetConfig();
  const int cpu_count = SmpCpuCount();
  const int process_count = cpu_count;
  const int64_t pmm_free_before = PmmFreeFrameCount();
  const int64_t heap_free_before = HeapTotalFreeBytes();
  bool window_checked = false;
  SELFTEST_EXPECT(ProbeInterruptInUserAccessWindow(&window_checked));
  bool nmi_window_checked = false;
  SELFTEST_EXPECT(ProbeNmiInUserAccessWindow(&nmi_window_checked));
  UserPageProbeCounters counters;
  int64_t switched_mid_syscall = 0;
  int64_t syscalls = 0;
  {
    uintptr_t data_frame = 0;
    const std::unique_ptr<AddressSpace> space =
        CreateUserPageProbeSpace(cpu_count, &data_frame);
    SELFTEST_EXPECT(space != nullptr);

    // On an early return, the destructors kill the programs and wait for
    // them, before `space` goes.
    const ProcessArray processes = MakeProcessArray(process_count);
    for (int i = 0; i < process_count; ++i) {
      processes[i] = UserProcess::Start(
          UserTestProgram(),
          usertest::KernelSpinArg(kProbeSyscallTicks,  //
                                  0,                   //
                                  kProbeTestSeed + static_cast<uint64_t>(i)));
      SELFTEST_EXPECT(processes[i] != nullptr);
    }
    // Each program finished a call and started the next.
    SELFTEST_EXPECT(WaitForProcesses(
        processes.get(),                                                    //
        process_count,                                                      //
        [](const UserProcessStats& stats) { return stats.syscalls >= 2; },  //
        WaitMode::kSpin));
    const std::unique_ptr<int64_t[]> switched_before =
        std::make_unique<int64_t[]>(process_count);
    for (int i = 0; i < process_count; ++i) {
      switched_before[i] = processes[i]->stats().syscalls_switched_out;
    }
    const std::unique_ptr<UserProcess> syscalls_program = UserProcess::Start(
        UserTestProgram(), usertest::Arg(usertest::Mode::kSyscalls));
    SELFTEST_EXPECT(syscalls_program != nullptr);

    UserPageProbeSetup setup;
    setup.space = space.get();
    setup.data_frame = data_frame;
    setup.smap = config.smap;
    setup.smep = config.smep;
    setup.watched = processes.get();
    setup.watched_count = process_count;
    SELFTEST_EXPECT(RunUserPageProbesOnEveryCpu(setup, &counters));

    // System calls keep working after the faults: each program starts two
    // more calls, and the kSyscalls program passes its checks.
    const std::unique_ptr<int64_t[]> syscalls_after_probes =
        std::make_unique<int64_t[]>(process_count);
    for (int i = 0; i < process_count; ++i) {
      syscalls_after_probes[i] = processes[i]->stats().syscalls;
    }
    const TscDeadline deadline(kSelfTestWaitTimeoutTscCycles);
    for (int i = 0; i < process_count; ++i) {
      while (processes[i]->stats().syscalls < syscalls_after_probes[i] + 2) {
        SELFTEST_EXPECT(!deadline.Expired());
        asm volatile("pause" : : : "memory");
      }
    }
    SELFTEST_EXPECT(
        WaitForProcesses(&syscalls_program, 1, HasEnded, WaitMode::kSpin));
    SELFTEST_EXPECT(ExitedWith(syscalls_program->Wait(), 0));
    for (int i = 0; i < process_count; ++i) {
      processes[i]->Kill();
    }
    for (int i = 0; i < process_count; ++i) {
      UserProcessStats stats;
      // An exit instead would mean a failed check (usertest.h).
      SELFTEST_EXPECT(EndedKilled(processes[i], &stats));
      switched_mid_syscall += stats.syscalls_switched_out - switched_before[i];
      syscalls += stats.syscalls;
    }
  }

  LogUserPageProbe(counters, UserPageProbe::kNotPresentRead, cpu_count);
  if (config.smap) {
    LogUserPageProbe(counters, UserPageProbe::kSmapRead, cpu_count);
    LogUserPageProbe(counters, UserPageProbe::kSmapWrite, cpu_count);
  } else {
    UartWrite(
        "[TEST] smap_smep_protection: smap probes skipped (SMAP disabled)\n");
  }
  if (config.smep) {
    LogUserPageProbe(counters, UserPageProbe::kSmepExecute, cpu_count);
  } else {
    UartWrite(
        "[TEST] smap_smep_protection: smep probe skipped (SMEP disabled)\n");
  }
  UartWrite(window_checked
                ? "[TEST] smap_smep_protection: interrupt in user-access "
                  "window checked\n"
                : "[TEST] smap_smep_protection: interrupt in user-access "
                  "window skipped (SMAP disabled)\n");
  if (nmi_window_checked) {
    UartWrite(
        "[TEST] smap_smep_protection: nmi in user-access window checked\n");
  } else {
    UartWrite(config.smap
                  ? "[TEST] smap_smep_protection: nmi in user-access window "
                    "skipped (one CPU)\n"
                  : "[TEST] smap_smep_protection: nmi in user-access window "
                    "skipped (SMAP disabled)\n");
  }
  const int rounds = counters.rounds.load();
  UartWrite("[TEST] smap_smep_protection: probes=");
  UartWriteDec(static_cast<uint64_t>(counters.probes.load()));
  UartWrite(" rounds_during_syscalls=");
  UartWriteDec(static_cast<uint64_t>(counters.rounds_during_syscalls.load()));
  UartWrite("/");
  UartWriteDec(static_cast<uint64_t>(rounds));
  UartWrite(" copies=");
  UartWriteDec(static_cast<uint64_t>(counters.copies.load()));
  UartWrite(" switched_mid_syscall=");
  UartWriteDec(static_cast<uint64_t>(switched_mid_syscall));
  UartWrite(" processes=");
  UartWriteDec(static_cast<uint64_t>(process_count));
  UartWrite(" syscalls=");
  UartWriteDec(static_cast<uint64_t>(syscalls));
  UartWrite("\n");

  const CpuMask128 all = CpuMask128::FirstCpus(cpu_count);
  const auto on_every_cpu = [&counters, &all](const UserPageProbe probe) {
    return counters.expected[static_cast<int>(probe)].Load().ContainsAll(all);
  };
  SELFTEST_EXPECT(counters.unexpected.load() == 0);
  SELFTEST_EXPECT(on_every_cpu(UserPageProbe::kNotPresentRead));
  SELFTEST_EXPECT(!config.smap || on_every_cpu(UserPageProbe::kSmapRead));
  SELFTEST_EXPECT(!config.smap || on_every_cpu(UserPageProbe::kSmapWrite));
  SELFTEST_EXPECT(!config.smep || on_every_cpu(UserPageProbe::kSmepExecute));
  SELFTEST_EXPECT(window_checked == config.smap);
  SELFTEST_EXPECT(nmi_window_checked == (config.smap && cpu_count > 1));
  SELFTEST_EXPECT(rounds >= kUserPageProbeRounds * cpu_count);
  SELFTEST_EXPECT(counters.copies.load() == rounds);
  SELFTEST_EXPECT(counters.rounds_during_syscalls.load() >= 1);
  SELFTEST_EXPECT(switched_mid_syscall >= 1);
  SELFTEST_EXPECT(PagingActiveRoot() == PagingKernelRoot());
  SELFTEST_EXPECT(PagingUserSpaceIsEmpty());
  for (int cpu = 0; cpu < cpu_count; ++cpu) {
    SELFTEST_EXPECT(SmpGetCpuLocal(cpu)->fault_probe == nullptr);
  }
  // The heap keeps the frames that it takes from the PMM (heap.h): growth
  // moves frames from the PMM into the heap, where they are free.
  const bool heap_grew = PmmFreeFrameCount() < pmm_free_before &&
                         HeapTotalFreeBytes() > heap_free_before;
  if (!heap_grew) {
    SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
    SELFTEST_EXPECT(HeapTotalFreeBytes() == heap_free_before);
  }
  return true;
}

// End-to-end tests
// ================
//
// Tests across the features of user mode: Kill() at its boundaries, system
// calls with bad and unusual buffers with SMAP on or off, the life cycle of
// the x87/SSE/AVX state of processes that are killed, in the middle of long
// system calls or with a pending x87 exception, and a mixed workload that
// combines all of them.

// The user stack layout that usertest.h repeats for kBufferChecks.
static_assert(usertest::kStackTop == kUserStackTop);
static_assert(usertest::kStackPages == kUserStackPages);

// The most timer ticks of CPU 0 from a request to kill a program to the end
// of its process that the end-to-end tests accept: 5 times the bound that
// user_mode_kill checks (`kKillDeadlineTicks`), because they kill in
// situations that user_mode_kill leaves out (several killers, a killer that
// keeps its CPU busy, more runnable tasks than CPUs). A shutdown, which kills
// many processes back to back, may take twice as long.
constexpr int64_t kE2eKillTicks = 5 * kKillDeadlineTicks;
constexpr int64_t kE2eShutdownTicks = 10 * kKillDeadlineTicks;

// Returns true once timer interrupts have interrupted the program's user code
// twice, or once it has ended (otherwise than the test expects, which the
// caller then reports instead of waiting for the deadline).
static bool HasRunInUserModeOrEnded(const UserProcessStats& stats) {
  return HasRunInUserMode(stats) || HasEnded(stats);
}

// Returns true while the program is in the middle of a system call, or once
// it has ended.
static bool IsInSyscallOrEnded(const UserProcessStats& stats) {
  return stats.in_syscall || HasEnded(stats);
}

// Returns true once the program has made a system call, or has ended.
static bool HasMadeSyscallOrEnded(const UserProcessStats& stats) {
  return stats.syscalls >= 1 || HasEnded(stats);
}

// Waits without yielding until CPU 0, which runs the boot self-tests, has
// taken `ticks` more timer interrupts. Fails if that takes longer than
// kSelfTestWaitTimeoutTscCycles.
static bool BusyWaitTicks(const int64_t ticks) {
  const int64_t until = CpuTimerTicks(0) + ticks;
  const TscDeadline deadline(kSelfTestWaitTimeoutTscCycles);
  while (CpuTimerTicks(0) < until) {
    SELFTEST_EXPECT(!deadline.Expired());
    asm volatile("pause" : : : "memory");
  }
  return true;
}

// Measures how many TSC cycles a timer tick of CPU 0 takes, over a few ticks
// from right after one, and stores the result in `*cycles`.
static bool MeasureTscCyclesPerTick(int64_t* const cycles) {
  constexpr int64_t kTicks = 4;
  SELFTEST_EXPECT(BusyWaitTicks(1));
  const uint64_t start = ReadTsc64();
  SELFTEST_EXPECT(BusyWaitTicks(kTicks));
  *cycles = static_cast<int64_t>(ReadTsc64() - start) / kTicks;
  return true;
}

// Checks that none of the first `count` `processes` has ended: each runs
// until the test kills it, so one that ended found a corrupted register.
// Logs how it ended otherwise.
static bool NoneEnded(const std::unique_ptr<UserProcess>* const processes,
                      const int count) {
  for (int i = 0; i < count; ++i) {
    if (HasEnded(processes[i]->stats())) {
      LogUnexpectedResult(processes[i]->Wait());
      return false;
    }
  }
  return true;
}

// Starts `count` processes that run user/usertest.cpp with the arguments
// `arg_of(0)` to `arg_of(count - 1)` as `processes[0..count-1]`.
template <typename ArgOf>
static bool StartProcesses(std::unique_ptr<UserProcess>* const processes,  //
                           const int count,                                //
                           const ArgOf arg_of) {
  for (int i = 0; i < count; ++i) {
    processes[i] = UserProcess::Start(UserTestProgram(), arg_of(i));
    SELFTEST_EXPECT(processes[i] != nullptr);
  }
  return true;
}

// Kills the first `count` `processes` back to back and waits for them: each
// must end as killed (EndedKilled()), and its counters go to `stats[i]` if
// `stats` isn't nullptr.
static bool KillAll(const std::unique_ptr<UserProcess>* const processes,  //
                    const int count,                                      //
                    UserProcessStats* const stats) {
  for (int i = 0; i < count; ++i) {
    processes[i]->Kill();
  }
  for (int i = 0; i < count; ++i) {
    UserProcessStats process_stats;
    SELFTEST_EXPECT(EndedKilled(processes[i], &process_stats));
    if (stats != nullptr) {
      stats[i] = process_stats;
    }
  }
  return true;
}

// The most times that RunUserPageProbesUntilCovered() starts probe tasks.
constexpr int kMaxProbeCoverageAttempts = 4;

// Returns true if `setup` runs probes of kind `probe`.
static bool ProbeKindEnabled(const UserPageProbeSetup& setup,
                             const UserPageProbe probe) {
  switch (probe) {
    case UserPageProbe::kNotPresentRead:
      return true;
    case UserPageProbe::kSmapRead:
    case UserPageProbe::kSmapWrite:
      return setup.smap;
    case UserPageProbe::kSmepExecute:
      return setup.smep;
  }
  return false;
}

// Runs the probes of `setup` on every CPU (RunUserPageProbesOnEveryCpu())
// until each kind of probe that `setup` enables has page-faulted as expected
// on every CPU, with new probe tasks at most kMaxProbeCoverageAttempts times
// (a probe task may move to another CPU between two probes of a round). Each
// probe must page-fault as expected and recover, and each copy through the
// user-copy helpers must work. Stores the probes that ran in `*probes`, and
// the CPUs on which every enabled kind faulted as expected in `*cpus`.
static bool RunUserPageProbesUntilCovered(const UserPageProbeSetup& setup,  //
                                          const int cpu_count,              //
                                          int* const probes,                //
                                          int* const cpus) {
  CpuMask128 covered[kUserPageProbeKinds];
  *probes = 0;
  for (int attempt = 0; attempt < kMaxProbeCoverageAttempts; ++attempt) {
    UserPageProbeCounters counters;
    SELFTEST_EXPECT(RunUserPageProbesOnEveryCpu(setup, &counters));
    SELFTEST_EXPECT(counters.unexpected.load() == 0);
    SELFTEST_EXPECT(counters.copies.load() == counters.rounds.load());
    *probes += counters.probes.load();
    for (int kind = 0; kind < kUserPageProbeKinds; ++kind) {
      const CpuMask128 faulted = counters.expected[kind].Load();
      covered[kind].low |= faulted.low;
      covered[kind].high |= faulted.high;
    }
    *cpus = 0;
    for (int cpu = 0; cpu < cpu_count; ++cpu) {
      bool all_kinds = true;
      for (int kind = 0; kind < kUserPageProbeKinds; ++kind) {
        all_kinds =
            all_kinds &&
            (!ProbeKindEnabled(setup, static_cast<UserPageProbe>(kind)) ||
             covered[kind].Contains(cpu));
      }
      *cpus += all_kinds ? 1 : 0;
    }
    if (*cpus == cpu_count) {
      return true;
    }
  }
  SELFTEST_EXPECT(*cpus == cpu_count);
  return true;
}

// The timer ticks for which KillBehindBusyKiller() keeps CPU 0 busy.
constexpr int64_t kBusyKillerTicks = 3;
// Not an ELF file.
constexpr uint8_t kBrokenExecutable[64] = {0x7F, 'E', 'L', 'F'};
// The programs whose natural end KillRacingNaturalEnds() races, and the
// points after their start at which it kills them: right away, after a
// Yield(), and after a timer tick.
constexpr usertest::Mode kRacingModes[] = {usertest::Mode::kExit,
                                           usertest::Mode::kInvalidOpcode,
                                           usertest::Mode::kStackOverflow};
constexpr int kKillStaggers = 3;

// What TestUserModeKillEdgeCases() observed.
struct KillEdgeReport {
  // Sub-cases that passed.
  int cases = 0;
  // Programs killed while racing their natural end, and those of them that
  // ended as killed (the others ended as they would have without the kill).
  int races = 0;
  int races_killed = 0;
  // The most timer ticks from a kill request to the end of a process, and
  // the most that the destructor of a running process took.
  int64_t max_kill_ticks = 0;
  int64_t destructor_ticks = 0;
};

// Returns true if `result` is how a program in `mode` (kRacingModes) ends
// without being killed: exit() with its status, #UD, or a page fault in the
// guard page below its stack.
static bool EndedNaturally(const usertest::Mode mode,
                           const UserProgramResult& result) {
  constexpr uintptr_t kStackGuardPage =
      kUserStackTop - (kUserStackPages + 1) * kPageSize;
  switch (mode) {
    case usertest::Mode::kExit:
      return result.outcome == UserProgramOutcome::kExited &&
             result.exit_status == usertest::kExitStatus;
    case usertest::Mode::kInvalidOpcode:
      return result.outcome == UserProgramOutcome::kFaulted &&
             result.fault_vector == kVectorInvalidOpcode &&
             IsUserRange(result.fault_rip, 1);
    case usertest::Mode::kStackOverflow:
      return result.outcome == UserProgramOutcome::kFaulted &&
             result.fault_vector == kVectorPageFault &&
             (result.fault_error_code & (kPageFaultPresent | kPageFaultUser)) ==
                 kPageFaultUser &&
             IsUserRange(result.fault_rip, 1) &&
             result.fault_address >= kStackGuardPage &&
             result.fault_address < kStackGuardPage + kPageSize;
    default:
      return false;
  }
}

// (a) Kills programs that end on their own soon after they start, right
// after the start, after a Yield() and after a timer tick: each ends either
// as killed or exactly as it would have without the kill, never otherwise.
static bool KillRacingNaturalEnds(KillEdgeReport* const report) {
  TaskScheduler* const scheduler = GetTaskScheduler();
  for (const usertest::Mode mode : kRacingModes) {
    for (int stagger = 0; stagger < kKillStaggers; ++stagger) {
      const std::unique_ptr<UserProcess> process =
          UserProcess::Start(UserTestProgram(), usertest::Arg(mode));
      SELFTEST_EXPECT(process != nullptr);
      if (stagger == 1) {
        scheduler->Yield();
      } else if (stagger == 2) {
        SELFTEST_EXPECT(BusyWaitTicks(1));
      }
      process->Kill();
      SELFTEST_EXPECT(
          WaitForProcesses(&process, 1, HasEnded, WaitMode::kYield));
      const UserProgramResult result = process->Wait();
      ++report->races;
      if (result.outcome == UserProgramOutcome::kKilled) {
        ++report->races_killed;
      } else if (!EndedNaturally(mode, result)) {
        LogUnexpectedResult(result);
        return false;
      }
    }
  }
  ++report->cases;
  return true;
}

// (b) Kills programs that have ended but that nobody waited for yet, twice:
// the result stays their exit. (c) Kills them twice more after Wait():
// nothing that the kernel reports about them changes.
static bool KillAfterEndAndWait(KillEdgeReport* const report) {
  for (int i = 0; i < 2; ++i) {
    const std::unique_ptr<UserProcess> process = UserProcess::Start(
        UserTestProgram(), usertest::Arg(usertest::Mode::kExit));
    SELFTEST_EXPECT(process != nullptr);
    SELFTEST_EXPECT(WaitForProcesses(&process, 1, HasEnded, WaitMode::kYield));
    const UserProcessStats ended = process->stats();
    process->Kill();
    process->Kill();
    SELFTEST_EXPECT(ExitedWith(process->Wait(), usertest::kExitStatus));
    const UserProcessStats waited = process->stats();
    SELFTEST_EXPECT(waited.end_tick == ended.end_tick);
    SELFTEST_EXPECT(waited.end_vector == -1);
    SELFTEST_EXPECT(waited.syscalls == 1);
    process->Kill();
    process->Kill();
    const UserProcessStats after = process->stats();
    SELFTEST_EXPECT(after.kill_tick == waited.kill_tick);
    SELFTEST_EXPECT(after.end_tick == waited.end_tick);
    SELFTEST_EXPECT(after.syscalls == waited.syscalls);
    SELFTEST_EXPECT(process->syscall_count() == 1);
  }
  report->cases += 2;
  return true;
}

// What the killers of KillWithConcurrentKillers() share.
struct ConcurrentKillContext {
  const std::unique_ptr<UserProcess>* processes = nullptr;
  int count = 0;
  // All killers (the tasks and the test's task), those that have arrived,
  // and the tasks that are done.
  int killers = 0;
  std::atomic<int> arrived{0};
  std::atomic<int> done{0};
  std::atomic<bool> timed_out{false};
};

struct ConcurrentKillTaskArg {
  ConcurrentKillContext* context = nullptr;
  int index = 0;
};

// Waits until every killer has arrived, then kills every process, starting
// at another one for each killer. Returns false (and kills nothing) if the
// others don't arrive within kSelfTestWaitTimeoutTscCycles.
static bool ArriveAndKillAll(ConcurrentKillContext* const context,
                             const int index) {
  context->arrived.fetch_add(1, std::memory_order_acq_rel);
  const TscDeadline deadline(kSelfTestWaitTimeoutTscCycles);
  while (context->arrived.load(std::memory_order_acquire) < context->killers) {
    if (deadline.Expired()) {
      context->timed_out.store(true, std::memory_order_release);
      return false;
    }
    asm volatile("pause" : : : "memory");
  }
  for (int i = 0; i < context->count; ++i) {
    context->processes[(i + index) % context->count]->Kill();
  }
  return true;
}

static void ConcurrentKillTask(void* const raw_arg) {
  const ConcurrentKillTaskArg* const arg =
      static_cast<const ConcurrentKillTaskArg*>(raw_arg);
  ArriveAndKillAll(arg->context, arg->index);
  arg->context->done.fetch_add(1, std::memory_order_release);
}

// (d) Kills pure spinners, as many as CPUs, from a task on every CPU but
// CPU 0 and from this task at the same time: each ends as killed within
// kE2eKillTicks of the first request, and the killers all return.
static bool KillWithConcurrentKillers(const int cpu_count,
                                      KillEdgeReport* const report) {
  TaskScheduler* const scheduler = GetTaskScheduler();
  const int count = cpu_count;
  const ProcessArray spinners = MakeProcessArray(count);
  for (int i = 0; i < count; ++i) {
    spinners[i] = UserProcess::Start(
        UserTestProgram(), usertest::Arg(usertest::Mode::kSpinWithoutSyscalls,
                                         0xD1CE + static_cast<uint64_t>(i)));
    SELFTEST_EXPECT(spinners[i] != nullptr);
  }
  SELFTEST_EXPECT(WaitForProcesses(spinners.get(),           //
                                   count,                    //
                                   HasRunInUserModeOrEnded,  //
                                   WaitMode::kYield));
  if constexpr (kInjectedKillDelay == KillDelayInjection::kConcurrentKillers) {
    spinners[0]->ArmInjectedKillDelay(2 * kE2eKillTicks + 5);
  }
  const int helpers = cpu_count - 1;
  ConcurrentKillContext context;
  context.processes = spinners.get();
  context.count = count;
  context.killers = helpers + 1;
  ConcurrentKillTaskArg args[kMaxSupportedCpus];
  {
    // On an early return, the destructor joins the tasks, which give up
    // waiting for the others by their deadline.
    PhaseTaskGroup tasks(*scheduler, nullptr);
    for (int i = 0; i < helpers; ++i) {
      args[i].context = &context;
      args[i].index = i + 1;
      SELFTEST_EXPECT(tasks.Add(scheduler->CreateTaskOnCpu(
          ConcurrentKillTask, &args[i], kDefaultTaskWeight, i + 1)));
    }
    SELFTEST_EXPECT(ArriveAndKillAll(&context, 0));
    const TscDeadline deadline(kSelfTestWaitTimeoutTscCycles);
    while (context.done.load(std::memory_order_acquire) < helpers) {
      SELFTEST_EXPECT(!deadline.Expired());
      asm volatile("pause" : : : "memory");
    }
    tasks.JoinAll();
  }
  SELFTEST_EXPECT(!context.timed_out.load(std::memory_order_acquire));
  for (int i = 0; i < count; ++i) {
    UserProcessStats stats;
    SELFTEST_EXPECT(EndedKilled(spinners[i], &stats));
    SELFTEST_EXPECT(stats.syscalls == 0);
    SELFTEST_EXPECT(KillTicks(stats) <= kE2eKillTicks);
    report->max_kill_ticks = Max(report->max_kill_ticks, KillTicks(stats));
  }
  ++report->cases;
  return true;
}

// (e) Destroys a running pure spinner without Wait(): the destructor kills
// it and waits for it, within kE2eKillTicks.
static bool DestroyRunningSpinner(KillEdgeReport* const report) {
  std::unique_ptr<UserProcess> process = UserProcess::Start(
      UserTestProgram(),
      usertest::Arg(usertest::Mode::kSpinWithoutSyscalls, 0xDE57));
  SELFTEST_EXPECT(process != nullptr);
  SELFTEST_EXPECT(
      WaitForProcesses(&process, 1, HasRunInUserModeOrEnded, WaitMode::kYield));
  SELFTEST_EXPECT(!HasEnded(process->stats()));
  const int64_t start = CpuTimerTicks(0);
  process.reset();
  const int64_t ticks = CpuTimerTicks(0) - start;
  SELFTEST_EXPECT(ticks <= kE2eKillTicks);
  report->destructor_ticks = Max(report->destructor_ticks, ticks);
  ++report->cases;
  return true;
}

// (f) Kills pure spinners, as many as CPUs (so one may wait behind this task
// for CPU 0), and then keeps CPU 0 busy for kBusyKillerTicks timer ticks
// without yielding: each still ends as killed within kE2eKillTicks.
static bool KillBehindBusyKiller(const int cpu_count,
                                 KillEdgeReport* const report) {
  const int count = cpu_count;
  const ProcessArray spinners = MakeProcessArray(count);
  for (int i = 0; i < count; ++i) {
    spinners[i] = UserProcess::Start(
        UserTestProgram(), usertest::Arg(usertest::Mode::kSpinWithoutSyscalls,
                                         0xB05E + static_cast<uint64_t>(i)));
    SELFTEST_EXPECT(spinners[i] != nullptr);
  }
  SELFTEST_EXPECT(WaitForProcesses(spinners.get(),           //
                                   count,                    //
                                   HasRunInUserModeOrEnded,  //
                                   WaitMode::kYield));
  if constexpr (kInjectedKillDelay == KillDelayInjection::kBusyKiller) {
    spinners[0]->ArmInjectedKillDelay(2 * kE2eKillTicks + 5);
  }
  for (int i = 0; i < count; ++i) {
    spinners[i]->Kill();
  }
  SELFTEST_EXPECT(BusyWaitTicks(kBusyKillerTicks));
  for (int i = 0; i < count; ++i) {
    UserProcessStats stats;
    SELFTEST_EXPECT(EndedKilled(spinners[i], &stats));
    SELFTEST_EXPECT(stats.syscalls == 0);
    SELFTEST_EXPECT(KillTicks(stats) <= kE2eKillTicks);
    report->max_kill_ticks = Max(report->max_kill_ticks, KillTicks(stats));
  }
  ++report->cases;
  return true;
}

// (g) Kills a process right after starting it with an invalid executable: it
// ends as not loaded or as killed, without having made a system call.
static bool KillInvalidExecutable(KillEdgeReport* const report) {
  const std::unique_ptr<UserProcess> process = UserProcess::Start(
      UserProgram{kBrokenExecutable, sizeof(kBrokenExecutable)}, 0);
  SELFTEST_EXPECT(process != nullptr);
  process->Kill();
  SELFTEST_EXPECT(WaitForProcesses(&process, 1, HasEnded, WaitMode::kYield));
  const UserProgramResult result = process->Wait();
  if (result.outcome != UserProgramOutcome::kLoadFailed &&
      result.outcome != UserProgramOutcome::kKilled) {
    LogUnexpectedResult(result);
    return false;
  }
  SELFTEST_EXPECT(process->syscall_count() == 0);
  ++report->cases;
  return true;
}

// Checks Kill() at its boundaries (the contract in user_mode.h): a
// kill that races the natural end of a program leaves either the kill or the
// natural result; a kill after the end, before or after Wait(), changes
// nothing; several tasks may kill the same programs at once; the destructor
// of a running process kills it; a killer that keeps its CPU busy doesn't
// delay the end; and a program that can't load ends as such. Kills end the
// programs within kE2eKillTicks. A warm-up with the most processes and tasks
// at once comes first, and afterwards every frame and heap byte is back.
static bool TestUserModeKillEdgeCases() {
  SELFTEST_EXPECT(GetTaskScheduler() != nullptr);
  const int cpu_count = SmpCpuCount();
  KillEdgeReport warm_up;
  SELFTEST_EXPECT(KillWithConcurrentKillers(cpu_count, &warm_up));
  SELFTEST_EXPECT(DestroyRunningSpinner(&warm_up));
  const int64_t pmm_free_before = PmmFreeFrameCount();
  const int64_t heap_free_before = HeapTotalFreeBytes();

  KillEdgeReport report;
  SELFTEST_EXPECT(KillRacingNaturalEnds(&report));
  SELFTEST_EXPECT(KillAfterEndAndWait(&report));
  SELFTEST_EXPECT(KillWithConcurrentKillers(cpu_count, &report));
  SELFTEST_EXPECT(DestroyRunningSpinner(&report));
  SELFTEST_EXPECT(KillBehindBusyKiller(cpu_count, &report));
  SELFTEST_EXPECT(KillInvalidExecutable(&report));

  // (h) Everything is back.
  SELFTEST_EXPECT(PagingActiveRoot() == PagingKernelRoot());
  SELFTEST_EXPECT(PagingUserSpaceIsEmpty());
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  SELFTEST_EXPECT(HeapTotalFreeBytes() == heap_free_before);
  ++report.cases;

  UartWrite("[TEST] user_mode_kill_edge_cases: cases=");
  UartWriteDec(static_cast<uint64_t>(report.cases));
  UartWrite(" max_kill_ticks=");
  UartWriteDec(static_cast<uint64_t>(
      Max(report.max_kill_ticks, report.destructor_ticks)));
  UartWrite(" destructor_ticks=");
  UartWriteDec(static_cast<uint64_t>(report.destructor_ticks));
  UartWrite(" races_killed=");
  UartWriteDec(static_cast<uint64_t>(report.races_killed));
  UartWrite("/");
  UartWriteDec(static_cast<uint64_t>(report.races));
  UartWrite("\n");
  return true;
}

// Runs `count` kBufferChecks processes at once, which must all exit with 0
// and must each have taken a timer interrupt in user mode: almost all of
// their time in user mode is the computation with RFLAGS.AC set. Lowers
// `*min_user_ticks` to the fewest timer interrupts of the user code of one.
static bool RunBufferCheckProcesses(const int count,
                                    int64_t* const min_user_ticks) {
  const ProcessArray processes = MakeProcessArray(count);
  for (int i = 0; i < count; ++i) {
    processes[i] = UserProcess::Start(
        UserTestProgram(), usertest::Arg(usertest::Mode::kBufferChecks));
    SELFTEST_EXPECT(processes[i] != nullptr);
  }
  SELFTEST_EXPECT(
      WaitForProcesses(processes.get(), count, HasEnded, WaitMode::kYield));
  for (int i = 0; i < count; ++i) {
    SELFTEST_EXPECT(ExitedWith(processes[i]->Wait(), 0));
    const int64_t user_ticks = processes[i]->stats().user_timer_ticks;
    SELFTEST_EXPECT(user_ticks >= 1);
    if (*min_user_ticks < 0 || user_ticks < *min_user_ticks) {
      *min_user_ticks = user_ticks;
    }
  }
  return true;
}

// The seed of the first spin() program of CheckSmapStillEnforced().
constexpr uint64_t kBufferCheckProbeSeed = 0xB0F;

// With SMAP: checks with probes on every CPU that the kernel's own accesses
// to user pages outside the user-copy helpers still page-fault (and the
// helpers still work), so that no system call, also no failing one, left
// RFLAGS.AC set behind, while as many spin() programs as CPUs are in the
// middle of system calls that read user buffers. (They also keep every CPU
// busy, so that no idle CPU takes a probe task away from the CPU that it was
// queued on.) Each must end as killed afterwards. Stores the number of CPUs
// on which every probe faulted as expected in `*cpus`.
static bool CheckSmapStillEnforced(const int cpu_count, int* const cpus) {
  uintptr_t data_frame = 0;
  const std::unique_ptr<AddressSpace> space =
      CreateUserPageProbeSpace(cpu_count, &data_frame);
  SELFTEST_EXPECT(space != nullptr);
  const ProcessArray programs = MakeProcessArray(cpu_count);
  SELFTEST_EXPECT(StartProcesses(programs.get(), cpu_count, [](const int i) {
    return usertest::KernelSpinArg(
        kProbeSyscallTicks,  //
        0,                   //
        kBufferCheckProbeSeed + static_cast<uint64_t>(i));
  }));
  SELFTEST_EXPECT(WaitForProcesses(programs.get(),         //
                                   cpu_count,              //
                                   HasMadeSyscallOrEnded,  //
                                   WaitMode::kYield));
  SELFTEST_EXPECT(NoneEnded(programs.get(), cpu_count));
  UserPageProbeSetup setup;
  setup.space = space.get();
  setup.data_frame = data_frame;
  setup.smap = true;
  setup.smep = CpuGetConfig().smep;
  setup.watched = programs.get();
  setup.watched_count = cpu_count;
  int probes = 0;
  SELFTEST_EXPECT(
      RunUserPageProbesUntilCovered(setup, cpu_count, &probes, cpus));
  return KillAll(programs.get(), cpu_count, nullptr);
}

// Checks system calls with bad and unusual buffers and arguments
// (kBufferChecks in user/usertest.h), in one process alone and then in one
// process per CPU at once: user code gets kSyscallErrorFault for every buffer
// that it may not read, also kernel, non-canonical and wrapping ones, without
// a panic; kSyscallErrorInvalid for invalid sizes and ticks; the checksum of
// unusual buffers that it may read; and its registers and RFLAGS.AC back,
// also after timer interrupts of its user code with RFLAGS.AC set. The passes
// cover both SMAP on and off. With SMAP, a SMAP read probe on every CPU
// afterwards shows that the kernel still faults on its own accesses to user
// pages. Runs everything twice; the first round lets the heap grow, and after
// the second every frame and heap byte is back.
static bool TestUserModeSyscallBufferChecks() {
  SELFTEST_EXPECT(GetTaskScheduler() != nullptr);
  const int cpu_count = SmpCpuCount();
  const bool smap = CpuGetConfig().smap;
  const int count = cpu_count;
  int64_t pmm_free_before = 0;
  int64_t heap_free_before = 0;
  int64_t min_user_ticks = -1;
  int smap_cpus = 0;
  for (int round = 0; round < 2; ++round) {
    if (round == 1) {
      pmm_free_before = PmmFreeFrameCount();
      heap_free_before = HeapTotalFreeBytes();
    }
    SELFTEST_EXPECT(RunBufferCheckProcesses(1, &min_user_ticks));
    SELFTEST_EXPECT(RunBufferCheckProcesses(count, &min_user_ticks));
    if (smap) {
      SELFTEST_EXPECT(CheckSmapStillEnforced(cpu_count, &smap_cpus));
    }
  }

  // `checks=` is the size of the suite (usertest::kBufferCheckCount, which the
  // static_assert in user/usertest.cpp ties to the program): every process
  // above exited with 0, which kBufferChecks does only after passing all of
  // them.
  UartWrite("[TEST] user_mode_syscall_buffer_checks: checks=");
  UartWriteDec(static_cast<uint64_t>(usertest::kBufferCheckCount));
  UartWrite(smap ? " smap=on" : " smap=off");
  UartWrite(" processes=");
  UartWriteDec(static_cast<uint64_t>(2 * (1 + count)));
  UartWrite(" user_ticks>=");
  UartWriteDec(static_cast<uint64_t>(min_user_ticks));
  UartWrite(" smap_probe_cpus=");
  UartWriteDec(static_cast<uint64_t>(smap_cpus));
  UartWrite("\n");

  SELFTEST_EXPECT(PagingActiveRoot() == PagingKernelRoot());
  SELFTEST_EXPECT(PagingUserSpaceIsEmpty());
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  SELFTEST_EXPECT(HeapTotalFreeBytes() == heap_free_before);
  return true;
}

// Starts `count` kFpuInitialState processes at once: each must find that it
// started with the architectural initial x87/SSE/AVX state.
static bool RunFpuInitialStateCheckers(const int count) {
  const ProcessArray checkers = MakeProcessArray(count);
  SELFTEST_EXPECT(StartProcesses(checkers.get(), count, [](const int) {
    return usertest::Arg(usertest::Mode::kFpuInitialState);
  }));
  SELFTEST_EXPECT(
      WaitForProcesses(checkers.get(), count, HasEnded, WaitMode::kYield));
  for (int i = 0; i < count; ++i) {
    SELFTEST_EXPECT(ExitedWith(checkers[i]->Wait(), 0));
  }
  return true;
}

// The first seeds of the kFpuHold processes of TestUserModeFpuLifecycle()
// (other than those of TestUserModeFpuIsolation()), and of its
// kFpuLongSyscall processes (below 2^20).
constexpr uint64_t kLifecycleHoldSeedBase = 0xF1F1'0000;
constexpr uint64_t kLifecycleSyscallSeedBase = 0x1000;
// The timer ticks of each spin() call of the kFpuLongSyscall processes of
// RunFpuLongSyscallBatches(), and the most batches of them that it runs.
constexpr int64_t kLifecycleSyscallTicks = 6;
constexpr int kMaxLifecycleBatches = 5;

// What TestUserModeFpuLifecycle() observed.
struct FpuLifecycleReport {
  // The spin() calls of the kFpuLongSyscall processes of (c) that the
  // scheduler switched out in the middle, and those that ended on another CPU
  // than they started on; the batches that it took.
  int64_t switched = 0;
  int64_t migrated = 0;
  int batches = 0;
  // Processes killed in the middle of a spin() call with live x87/SSE/AVX
  // state (d).
  int killed_mid_syscall = 0;
  // Processes that died of their pending x87 exception (e), and the fewest
  // timer interrupts that interrupted the user code of one of them.
  int mf = 0;
  int64_t mf_user_ticks = 0;
  // The TSC cycles that each round of computation of those processes took at
  // least (usertest::X87PendingArg()).
  int64_t mf_round_cycles = 0;
};

// (a) Starts `count` kFpuHold processes, with seeds from `seed_base`, waits
// until each has taken two timer interrupts in user mode, and kills them all
// at once: each ends as killed, none with a corruption status.
static bool KillFpuHolds(const int count, const uint64_t seed_base) {
  const ProcessArray holds = MakeProcessArray(count);
  SELFTEST_EXPECT(StartProcesses(holds.get(), count, [seed_base](const int i) {
    return usertest::Arg(usertest::Mode::kFpuHold,
                         seed_base + static_cast<uint64_t>(i));
  }));
  SELFTEST_EXPECT(WaitForProcesses(holds.get(),              //
                                   count,                    //
                                   HasRunInUserModeOrEnded,  //
                                   WaitMode::kYield));
  SELFTEST_EXPECT(NoneEnded(holds.get(), count));
  return KillAll(holds.get(), count, nullptr);
}

// The spin() calls of kFpuLongSyscall process `i` of `batch` of
// RunFpuLongSyscallBatches() on `cpu_count` CPUs: 2 to 4, so that the
// processes on at least one CPU finish before another CPU ends its
// two-process phase and the idle CPU steals a waiting mid-syscall process.
static int64_t LifecycleRounds(const int i,          //
                               const int cpu_count,  //
                               const int batch) {
  return FpuLifecycleRounds(i, cpu_count, batch);
}

// (c) Runs batches of 2 × CPUs kFpuLongSyscall processes, each with values of
// its own in the registers, until the spin() calls of the batches were
// switched out in the middle at least twice and, with more than one CPU,
// ended on another CPU than they started on at least once, at most
// kMaxLifecycleBatches batches. A CPU that has run all of its own processes
// takes one that waits in the middle of a call on another CPU, whose
// registers it must then load; a process that never ends, such as kFpuHold,
// would be taken instead, so they run next to the kFpuLongSyscall processes
// only in TestUserModeMixedWorkload(). Each kFpuLongSyscall process must exit
// with 0: after every call it found all of its x87/MMX/SSE/AVX registers,
// MXCSR and x87 control word, its general-purpose registers and the checksum
// as they must be.
static bool RunFpuLongSyscallBatches(const int cpu_count,
                                     FpuLifecycleReport* const report) {
  const int count = 2 * cpu_count;
  const bool expect_migration = cpu_count > 1;
  for (int batch = 0;
       batch < kMaxLifecycleBatches &&
       (report->switched < 2 || (expect_migration && report->migrated == 0));
       ++batch) {
    const ProcessArray processes = MakeProcessArray(count);
    const uint64_t seed_base =
        kLifecycleSyscallSeedBase + static_cast<uint64_t>(batch * count);
    SELFTEST_EXPECT(StartProcesses(
        processes.get(), count, [seed_base, cpu_count, batch](const int i) {
          return usertest::FpuLongSyscallArg(
              kLifecycleSyscallTicks,                //
              LifecycleRounds(i, cpu_count, batch),  //
              seed_base + static_cast<uint64_t>(i));
        }));
    SELFTEST_EXPECT(
        WaitForProcesses(processes.get(), count, HasEnded, WaitMode::kYield));
    for (int i = 0; i < count; ++i) {
      SELFTEST_EXPECT(ExitedWith(processes[i]->Wait(), 0));
      const UserProcessStats stats = processes[i]->stats();
      // The spin() calls and exit().
      SELFTEST_EXPECT(stats.syscalls ==
                      LifecycleRounds(i, cpu_count, batch) + 1);
      report->switched += stats.syscalls_switched_out;
      report->migrated += stats.syscalls_migrated;
    }
    ++report->batches;
  }
  SELFTEST_EXPECT(report->switched >= 2);
  SELFTEST_EXPECT(!expect_migration || report->migrated >= 1);
  return true;
}

// (d) Kills as many kFpuLongSyscall processes as CPUs in the middle of a
// spin() call that would last more than an hour, with values of their own in
// the x87/SSE/AVX registers: each ends as killed, on its way back from the
// call.
static bool KillFpuLongSyscalls(const int cpu_count,
                                FpuLifecycleReport* const report) {
  const int count = cpu_count;
  const ProcessArray processes = MakeProcessArray(count);
  SELFTEST_EXPECT(StartProcesses(processes.get(), count, [](const int i) {
    return usertest::FpuLongSyscallArg(
        kSyscallSpinMaxTicks,  //
        1,                     //
        kLifecycleSyscallSeedBase + 0x800 + static_cast<uint64_t>(i));
  }));
  SELFTEST_EXPECT(WaitForProcesses(processes.get(),     //
                                   count,               //
                                   IsInSyscallOrEnded,  //
                                   WaitMode::kYield));
  const std::unique_ptr<UserProcessStats[]> stats =
      std::make_unique<UserProcessStats[]>(count);
  SELFTEST_EXPECT(KillAll(processes.get(), count, stats.get()));
  for (int i = 0; i < count; ++i) {
    SELFTEST_EXPECT(stats[i].kill_in_syscall);
    SELFTEST_EXPECT(stats[i].syscalls == 1);
    SELFTEST_EXPECT(stats[i].end_vector == kVectorSyscall);
  }
  report->killed_mid_syscall += count;
  return true;
}

// The timer ticks of CPU 0 that each kX87PendingException process of
// RunX87PendingExceptions() computes for at least, in TSC cycles measured
// before it starts, and the most TSC cycles that this may come to, so that a
// measurement that the host stretched can't run the processes into the wait
// timeout.
constexpr int64_t kX87PendingWindowTicks = 8;
constexpr int64_t kX87PendingMaxWindowCycles =
    static_cast<int64_t>(kSelfTestWaitTimeoutTscCycles / 10);

// (e) Runs 2 × CPUs kX87PendingException processes next to as many kFpuHold
// processes as CPUs. Each of the former keeps an unmasked x87 exception
// pending across timer interrupts, system calls and task switches and must
// die of it (#MF) at its FWAIT, after all its system calls; the kFpuHold
// processes, which check their x87 status word in every round, must never
// see it and run until they are killed; and as many kFpuInitialState
// processes as CPUs started afterwards must find the initial state, with a
// clean status word. The former compute for kX87PendingWindowTicks timer
// ticks' worth of TSC cycles at least, also where their fixed iterations
// take much less, so that timer interrupts come while the exception is
// pending.
static bool RunX87PendingExceptions(const int cpu_count,
                                    FpuLifecycleReport* const report) {
  const int count = 2 * cpu_count;
  int64_t tick_cycles = 0;
  SELFTEST_EXPECT(MeasureTscCyclesPerTick(&tick_cycles));
  const int64_t window = kX87PendingWindowTicks * tick_cycles;
  const int64_t round_cycles =
      ((window < kX87PendingMaxWindowCycles) ? window
                                             : kX87PendingMaxWindowCycles) /
      usertest::kX87PendingRounds;
  // At least one TSC cycle per round: a measurement of 0, or a TSC that
  // doesn't advance, would leave only the fixed iterations.
  SELFTEST_EXPECT(round_cycles >= 1 &&
                  round_cycles <= usertest::kX87PendingMaxRoundCycles);
  const ProcessArray holds = MakeProcessArray(cpu_count);
  SELFTEST_EXPECT(StartProcesses(holds.get(), cpu_count, [](const int i) {
    return usertest::Arg(
        usertest::Mode::kFpuHold,
        kLifecycleHoldSeedBase + 0x2000 + static_cast<uint64_t>(i));
  }));
  const ProcessArray pending = MakeProcessArray(count);
  SELFTEST_EXPECT(
      StartProcesses(pending.get(), count, [round_cycles](const int) {
        return usertest::X87PendingArg(round_cycles);
      }));
  SELFTEST_EXPECT(
      WaitForProcesses(pending.get(), count, HasEnded, WaitMode::kYield));
  int64_t min_user_ticks = -1;
  for (int i = 0; i < count; ++i) {
    SELFTEST_EXPECT(
        FaultedWith(pending[i]->Wait(), kVectorX87FloatingPoint, 0, 0));
    const UserProcessStats stats = pending[i]->stats();
    SELFTEST_EXPECT(stats.syscalls == usertest::kX87PendingRounds);
    // Its computation with the exception pending is almost all of its time
    // in user mode.
    SELFTEST_EXPECT(stats.user_timer_ticks >= 1);
    if (min_user_ticks < 0 || stats.user_timer_ticks < min_user_ticks) {
      min_user_ticks = stats.user_timer_ticks;
    }
  }
  SELFTEST_EXPECT(NoneEnded(holds.get(), cpu_count));
  SELFTEST_EXPECT(RunFpuInitialStateCheckers(cpu_count));
  SELFTEST_EXPECT(KillAll(holds.get(), cpu_count, nullptr));
  report->mf += count;
  report->mf_user_ticks = min_user_ticks;
  report->mf_round_cycles = round_cycles;
  return true;
}

// Checks the life cycle of the x87/SSE/AVX state of processes together with
// kills and preemptible system calls: (a) processes killed while
// they hold values in all registers end as killed, and (b) the processes
// started right after them, which may get their freed state memory, start
// with the initial state (user_mode_fpu_initial_state checks one that does,
// on the CPU whose registers the killed process left); (c) processes keep
// all their registers across spin() calls that the scheduler switches out in
// the middle and resumes on another CPU, among other processes with values
// of their own; (d) processes in the middle of such a call with live state
// end as killed; and (e) a pending x87 exception stays with its process
// across switches and kills only it. A warm-up with the most processes at
// once comes first, and afterwards every frame and heap byte is back.
static bool TestUserModeFpuLifecycle() {
  SELFTEST_EXPECT(GetTaskScheduler() != nullptr);
  const int cpu_count = SmpCpuCount();
  SELFTEST_EXPECT(cpu_count >= 1 && cpu_count <= kMaxSupportedCpus);
  SELFTEST_EXPECT(KillFpuHolds(3 * cpu_count, kLifecycleHoldSeedBase));
  const int64_t pmm_free_before = PmmFreeFrameCount();
  const int64_t heap_free_before = HeapTotalFreeBytes();

  FpuLifecycleReport report;
  SELFTEST_EXPECT(KillFpuHolds(2 * cpu_count, kLifecycleHoldSeedBase + 0x3000));
  SELFTEST_EXPECT(RunFpuInitialStateCheckers(2 * cpu_count));
  SELFTEST_EXPECT(RunFpuLongSyscallBatches(cpu_count, &report));
  SELFTEST_EXPECT(KillFpuLongSyscalls(cpu_count, &report));
  SELFTEST_EXPECT(RunX87PendingExceptions(cpu_count, &report));

  UartWrite("[TEST] user_mode_fpu_lifecycle: switched=");
  UartWriteDec(static_cast<uint64_t>(report.switched));
  UartWrite(" migrated=");
  UartWriteDec(static_cast<uint64_t>(report.migrated));
  UartWrite(" batches=");
  UartWriteDec(static_cast<uint64_t>(report.batches));
  UartWrite(" killed_mid_syscall=");
  UartWriteDec(static_cast<uint64_t>(report.killed_mid_syscall));
  UartWrite(" mf=");
  UartWriteDec(static_cast<uint64_t>(report.mf));
  UartWrite(" mf_user_ticks>=");
  UartWriteDec(static_cast<uint64_t>(report.mf_user_ticks));
  UartWrite(" mf_round_cycles=");
  UartWriteDec(static_cast<uint64_t>(report.mf_round_cycles));
  UartWrite("\n");

  SELFTEST_EXPECT(PagingActiveRoot() == PagingKernelRoot());
  SELFTEST_EXPECT(PagingUserSpaceIsEmpty());
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  SELFTEST_EXPECT(HeapTotalFreeBytes() == heap_free_before);
  return true;
}

// The kinds of long-running processes (tenants) of
// TestUserModeMixedWorkload().
enum class TenantKind : uint8_t {
  // Loops on spin() calls that read user buffers (kKernelSpin).
  kLooper,
  // Computes without ever making a system call (kSpinWithoutSyscalls).
  kSpinner,
  // Holds values in all x87/SSE/AVX registers (kFpuHold).
  kHolder,
  // Holds values in them across spin() calls (kFpuLongSyscall).
  kLongSyscall,
};
constexpr int kTenantKinds = 4;
// The timer ticks of each spin() call of the loopers and of the
// kFpuLongSyscall tenants.
constexpr int64_t kTenantSyscallTicks = 2;
// The batches of short processes that TestUserModeMixedWorkload() runs.
constexpr int kChurnBatches = 4;
// The first seed of the tenants (below 2^20), and what the kFpuHold tenants
// add to theirs.
constexpr uint64_t kTenantSeedBase = 0x2000;
constexpr uint64_t kTenantHoldSeedBase = 0xF2F2'0000;

// Returns how many tenants of `kind` run on `cpu_count` CPUs: as many loopers
// and kFpuHold tenants as CPUs, and half as many (at least one) spinners and
// kFpuLongSyscall tenants.
static int TenantCount(const TenantKind kind, const int cpu_count) {
  const int half = (cpu_count / 2 > 1) ? cpu_count / 2 : 1;
  return (kind == TenantKind::kLooper || kind == TenantKind::kHolder)
             ? cpu_count
             : half;
}

// Returns the usertest argument of the tenant of `kind` with index `i`.
static uint64_t TenantArg(const TenantKind kind, const int i) {
  const uint64_t seed = kTenantSeedBase + static_cast<uint64_t>(i);
  switch (kind) {
    case TenantKind::kLooper:
      return usertest::KernelSpinArg(kTenantSyscallTicks, 0, seed);
    case TenantKind::kSpinner:
      return usertest::Arg(usertest::Mode::kSpinWithoutSyscalls, seed);
    case TenantKind::kHolder:
      return usertest::Arg(usertest::Mode::kFpuHold,
                           kTenantHoldSeedBase + seed);
    case TenantKind::kLongSyscall:
      return usertest::FpuLongSyscallArg(kTenantSyscallTicks, 0, seed);
  }
  return 0;
}

// Returns true once a tenant of `kind` has made progress: a looper has made
// 3 system calls, a kFpuLongSyscall tenant 2 (so each finished a checked
// spin() call), and the others took 2 timer interrupts in user mode.
static bool TenantMadeProgress(const TenantKind kind,
                               const UserProcessStats& stats) {
  switch (kind) {
    case TenantKind::kLooper:
      return stats.syscalls >= 3;
    case TenantKind::kLongSyscall:
      return stats.syscalls >= 2;
    case TenantKind::kSpinner:
    case TenantKind::kHolder:
      return stats.user_timer_ticks >= 2;
  }
  return false;
}

// The tenants of TestUserModeMixedWorkload(). On an early return, the
// destructors kill them and wait for them.
struct MixedWorkload {
  ProcessArray tenants;
  std::unique_ptr<TenantKind[]> kinds;
  int count = 0;
};

// P1: starts the tenants of every kind, each with a seed of its own.
static bool StartTenants(const int cpu_count, MixedWorkload* const workload) {
  int total = 0;
  for (int kind = 0; kind < kTenantKinds; ++kind) {
    total += TenantCount(static_cast<TenantKind>(kind), cpu_count);
  }
  workload->tenants = MakeProcessArray(total);
  workload->kinds = std::make_unique<TenantKind[]>(total);
  workload->count = 0;
  for (int kind = 0; kind < kTenantKinds; ++kind) {
    const TenantKind tenant_kind = static_cast<TenantKind>(kind);
    for (int i = 0; i < TenantCount(tenant_kind, cpu_count); ++i) {
      const int index = workload->count;
      workload->kinds[index] = tenant_kind;
      workload->tenants[index] =
          UserProcess::Start(UserTestProgram(), TenantArg(tenant_kind, index));
      SELFTEST_EXPECT(workload->tenants[index] != nullptr);
      ++workload->count;
    }
  }
  return true;
}

// P2: runs `batches` batches of as many short processes as CPUs,
// kFpuInitialState and kSyscalls in turn, each batch at once and waited for:
// each must exit with 0, so it started with the initial x87/SSE/AVX state and
// saw the documented system call results. Adds their number to `*count`.
static bool RunChurn(const int cpu_count, const int batches, int* const count) {
  const int batch_size = cpu_count;
  for (int batch = 0; batch < batches; ++batch) {
    const ProcessArray processes = MakeProcessArray(batch_size);
    SELFTEST_EXPECT(StartProcesses(
        processes.get(), batch_size, [batch, batch_size](const int i) {
          return usertest::Arg(((batch * batch_size + i) % 2 == 0)
                                   ? usertest::Mode::kFpuInitialState
                                   : usertest::Mode::kSyscalls);
        }));
    SELFTEST_EXPECT(WaitForProcesses(processes.get(),  //
                                     batch_size,       //
                                     HasEnded,         //
                                     WaitMode::kYield));
    for (int i = 0; i < batch_size; ++i) {
      SELFTEST_EXPECT(ExitedWith(processes[i]->Wait(), 0));
    }
    *count += batch_size;
  }
  return true;
}

// P3: if the CPUs have SMAP or SMEP, runs the probes of the enabled
// protections (RunUserPageProbesOnEveryCpu()) on every CPU while the tenants
// run, the loopers and kFpuLongSyscall tenants in the middle of spin() calls
// that read user buffers: every probe must page-fault as expected and
// recover, on every CPU, and the user-copy helpers must work. Stores the
// probes that ran in `*probes` and the CPUs on which every probe of an
// enabled protection faulted as expected in `*cpus` (-1: skipped).
static bool RunHardeningProbes(const int cpu_count,            //
                               const MixedWorkload& workload,  //
                               int* const probes,              //
                               int* const cpus) {
  const CpuConfig& config = CpuGetConfig();
  *probes = 0;
  *cpus = -1;
  if (!config.smap && !config.smep) {
    return true;
  }
  uintptr_t data_frame = 0;
  const std::unique_ptr<AddressSpace> space =
      CreateUserPageProbeSpace(cpu_count, &data_frame);
  SELFTEST_EXPECT(space != nullptr);
  UserPageProbeSetup setup;
  setup.space = space.get();
  setup.data_frame = data_frame;
  setup.smap = config.smap;
  setup.smep = config.smep;
  setup.watched = workload.tenants.get();
  setup.watched_count = workload.count;
  SELFTEST_EXPECT(
      RunUserPageProbesUntilCovered(setup, cpu_count, probes, cpus));
  return true;
}

// P4: waits until every tenant has made progress (TenantMadeProgress()).
// Fails if a tenant ends (each runs until it is killed, so it found a
// corrupted register or a wrong result) or the wait times out.
static bool WaitForTenantProgress(const MixedWorkload& workload) {
  TaskScheduler* const scheduler = GetTaskScheduler();
  const TscDeadline deadline(kSelfTestWaitTimeoutTscCycles);
  for (;;) {
    SELFTEST_EXPECT(NoneEnded(workload.tenants.get(), workload.count));
    bool done = true;
    for (int i = 0; i < workload.count && done; ++i) {
      done =
          TenantMadeProgress(workload.kinds[i], workload.tenants[i]->stats());
    }
    if (done) {
      return true;
    }
    SELFTEST_EXPECT(!deadline.Expired());
    scheduler->Yield();
  }
}

// What the real run of TestUserModeMixedWorkload() observed.
struct MixedWorkloadReport {
  int tenants = 0;
  int short_processes = 0;
  int probes = 0;
  int probe_cpus = -1;
  // Timer ticks from the kill request to the end of the runaway spinner (P5)
  // and the most of any tenant at the shutdown (P6).
  int64_t runaway_ticks = 0;
  int64_t shutdown_ticks = 0;
};

// Runs P1-P6 of TestUserModeMixedWorkload() (only P1-P3 and a shutdown, with
// one batch of short processes, if `warm_up`).
static bool RunMixedWorkload(const int cpu_count,  //
                             const bool warm_up,   //
                             MixedWorkloadReport* const report) {
  MixedWorkload workload;
  SELFTEST_EXPECT(StartTenants(cpu_count, &workload));
  report->tenants = workload.count;
  SELFTEST_EXPECT(RunChurn(cpu_count,                    //
                           warm_up ? 1 : kChurnBatches,  //
                           &report->short_processes));
  SELFTEST_EXPECT(RunHardeningProbes(cpu_count,        //
                                     workload,         //
                                     &report->probes,  //
                                     &report->probe_cpus));
  if (warm_up) {
    return KillAll(workload.tenants.get(), workload.count, nullptr);
  }
  SELFTEST_EXPECT(WaitForTenantProgress(workload));

  // P5: an operator kills a runaway job, a pure spinner, alone; the other
  // tenants keep running.
  int runaway = -1;
  for (int i = 0; i < workload.count && runaway < 0; ++i) {
    runaway = (workload.kinds[i] == TenantKind::kSpinner) ? i : -1;
  }
  SELFTEST_EXPECT(runaway >= 0);
  if constexpr (kInjectedKillDelay == KillDelayInjection::kRunaway) {
    workload.tenants[runaway]->ArmInjectedKillDelay(2 * kE2eKillTicks + 5);
  }
  workload.tenants[runaway]->Kill();
  UserProcessStats runaway_stats;
  SELFTEST_EXPECT(EndedKilled(workload.tenants[runaway], &runaway_stats));
  SELFTEST_EXPECT(runaway_stats.syscalls == 0);
  report->runaway_ticks = KillTicks(runaway_stats);
  SELFTEST_EXPECT(report->runaway_ticks <= kE2eKillTicks);
  // The others, which follow it in the array, and those before it.
  const int after = workload.count - runaway - 1;
  SELFTEST_EXPECT(NoneEnded(workload.tenants.get(), runaway));
  SELFTEST_EXPECT(NoneEnded(workload.tenants.get() + runaway + 1, after));

  // P6: the shutdown kills every other tenant back to back, in the middle of
  // computations and system calls: each ends as killed (an exit or a fault
  // would mean that it found a corrupted register or result).
  const std::unique_ptr<UserProcessStats[]> stats =
      std::make_unique<UserProcessStats[]>(workload.count);
  bool armed_shutdown_delay = false;
  for (int i = 0; i < workload.count; ++i) {
    if (i != runaway) {
      if constexpr (kInjectedKillDelay == KillDelayInjection::kShutdown) {
        if (!armed_shutdown_delay) {
          workload.tenants[i]->ArmInjectedKillDelay(2 * kE2eShutdownTicks + 5);
          armed_shutdown_delay = true;
        }
      }
      workload.tenants[i]->Kill();
    }
  }
  for (int i = 0; i < workload.count; ++i) {
    if (i != runaway) {
      SELFTEST_EXPECT(EndedKilled(workload.tenants[i], &stats[i]));
      report->shutdown_ticks = Max(report->shutdown_ticks, KillTicks(stats[i]));
    }
  }
  SELFTEST_EXPECT(report->shutdown_ticks <= kE2eShutdownTicks);
  return true;
}

// A realistic multi-tenant scenario on all CPUs: 3 × CPUs tenants of
// four kinds (loopers on spin() calls that read user buffers, pure spinners,
// kFpuHold processes and kFpuLongSyscall processes, each checking its own
// results and registers) share the CPUs while 4 × CPUs short processes come
// and go (P2: each starts with the initial FPU state and passes the system
// call checks), the kernel's SMAP/SMEP protections are probed on every CPU
// if the CPUs have them (P3), and every tenant makes progress (P4). Then an
// operator kills a runaway spinner (P5: it ends within kE2eKillTicks while
// the others keep running) and shuts everything down (P6: every tenant ends
// as killed within kE2eShutdownTicks). A warm-up run comes first; afterwards
// every frame and heap byte is back, and new processes start with the
// initial FPU state (P7).
static bool TestUserModeMixedWorkload() {
  SELFTEST_EXPECT(GetTaskScheduler() != nullptr);
  const int cpu_count = SmpCpuCount();
  const CpuConfig& config = CpuGetConfig();
  MixedWorkloadReport warm_up;
  SELFTEST_EXPECT(RunMixedWorkload(cpu_count, /*warm_up=*/true, &warm_up));
  const int64_t pmm_free_before = PmmFreeFrameCount();
  const int64_t heap_free_before = HeapTotalFreeBytes();
  MixedWorkloadReport report;
  SELFTEST_EXPECT(RunMixedWorkload(cpu_count, /*warm_up=*/false, &report));

  UartWrite("[TEST] user_mode_mixed_workload: tenants=");
  UartWriteDec(static_cast<uint64_t>(report.tenants));
  UartWrite(" short=");
  UartWriteDec(static_cast<uint64_t>(report.short_processes));
  UartWrite(" runaway_kill_ticks=");
  UartWriteDec(static_cast<uint64_t>(report.runaway_ticks));
  UartWrite(" shutdown_ticks=");
  UartWriteDec(static_cast<uint64_t>(report.shutdown_ticks));
  if (report.probe_cpus < 0) {
    UartWrite(" probes=skipped\n");
    UartWrite(
        "[TEST] user_mode_mixed_workload: hardening probes skipped (no SMAP "
        "or SMEP)\n");
  } else {
    UartWrite(" probes=");
    UartWriteDec(static_cast<uint64_t>(report.probes));
    UartWrite("\n[TEST] user_mode_mixed_workload: hardening probes smap=");
    UartWrite(config.smap ? "on" : "off");
    UartWrite(" smep=");
    UartWrite(config.smep ? "on" : "off");
    UartWrite(" recovered on ");
    UartWriteDec(static_cast<uint64_t>(report.probe_cpus));
    UartWrite("/");
    UartWriteDec(static_cast<uint64_t>(cpu_count));
    UartWrite(" CPUs\n");
  }

  // P7
  SELFTEST_EXPECT(PagingActiveRoot() == PagingKernelRoot());
  SELFTEST_EXPECT(PagingUserSpaceIsEmpty());
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  SELFTEST_EXPECT(HeapTotalFreeBytes() == heap_free_before);
  SELFTEST_EXPECT(RunFpuInitialStateCheckers(cpu_count));
  return true;
}

struct BootTest {
  const char* name;
  bool (*run)();
};

// The boot tests in the order they run. The first one must pass for the
// others to be meaningful.
constexpr BootTest kBootTests[] = {
    {"pmm_memory_map_init", TestPmmMemoryMapInit},
    {"pmm_alloc_and_bounds", TestPmmAllocAndBounds},
    {"pmm_free_and_reuse", TestPmmFreeAndReuse},
    {"heap_varied_sizes_and_alignment", TestHeapVariedSizesAndAlignment},
    {"heap_pattern_isolation", TestHeapPatternIsolation},
    {"cpp_new_delete_lifecycle", TestCppNewDeleteLifecycle},
    {"heap_stress_reuse", TestHeapStressReuse},
    {"edge_cases_and_oom", TestEdgeCasesAndOom},
    {"pmm_heap_exhaustion", TestPmmAndHeapExhaustion},
    {"paging_edge_cases", TestPagingEdgeCases},
    {"rbtree_edge_cases", TestRbTreeEdgeCases},
    {"multiboot_mmap_edge_cases", TestMultibootMmapEdgeCases},
    {"smp_discovery_and_ap_bringup", TestSmpDiscoveryAndApBringup},
    {"smp_multicpu_sync_stress", TestSmpMultiCpuSyncStress},
    {"idt_percpu_and_lapic_timer", TestIdtPerCpuAndLapicTimer},
    {"task_scheduler_smp", TestTaskSchedulerSmp},
    {"tlb_shootdown", TestTlbShootdown},
    {"task_stack_guard_pages", TestTaskStackGuardPages},
    {"address_spaces", TestAddressSpaces},
    {"address_space_switching", TestAddressSpaceSwitching},
    {"user_mode_programs", TestUserModePrograms},
    {"user_mode_preemption", TestUserModePreemption},
    {"user_mode_preemptible_syscalls", TestUserModePreemptibleSyscalls},
    {"user_mode_kill", TestUserModeKill},
    {"user_mode_fpu_isolation", TestUserModeFpuIsolation},
    {"user_mode_fpu_initial_state", TestUserModeFpuInitialState},
    {"cpu_protection_features", TestCpuProtectionFeatures},
    {"smap_smep_protection", TestSmapSmepProtection},
    // End-to-end tests.
    {"user_mode_kill_edge_cases", TestUserModeKillEdgeCases},
    {"user_mode_syscall_buffer_checks", TestUserModeSyscallBufferChecks},
    {"user_mode_fpu_lifecycle", TestUserModeFpuLifecycle},
    {"user_mode_mixed_workload", TestUserModeMixedWorkload},
};
constexpr int kBootTestCount = sizeof(kBootTests) / sizeof(kBootTests[0]);

}  // namespace

bool RunBootSelfTests() {
  const char* failed_tests[kBootTestCount];
  int failed_count = 0;
  for (int i = 0; i < kBootTestCount; ++i) {
    const bool passed = kBootTests[i].run();
    LogTestResult(kBootTests[i].name, passed);
    if (!passed) {
      failed_tests[failed_count++] = kBootTests[i].name;
      if (i == 0) {
        // Without a usable memory map, the other tests would only crash.
        break;
      }
    }
  }

  // The one VGA line of the suite; it ends with a newline so that the stress
  // test's progress line goes to the next row and doesn't overwrite it.
  if (failed_count == 0) {
    UartWrite("[TEST] ALL BOOT SELF-TESTS PASSED\n");
    VgaWrite("[TEST] ALL BOOT SELF-TESTS PASSED (");
    VgaWriteDec(static_cast<uint64_t>(kBootTestCount));
    VgaWrite(" tests)\n");
    return true;
  }
  UartWrite("[TEST] BOOT SELF-TESTS FAILED\n");
  VgaWrite("[TEST] BOOT SELF-TESTS FAILED:");
  for (int i = 0; i < failed_count; ++i) {
    VgaWrite(" ");
    VgaWrite(failed_tests[i]);
  }
  VgaWrite("\n");
  return false;
}

}  // namespace protos
