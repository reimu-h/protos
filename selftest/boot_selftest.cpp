#include "selftest/boot_selftest.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <new>

#include "address_space.h"
#include "check.h"
#include "format.h"
#include "gdt.h"
#include "heap.h"
#include "idt.h"
#include "multiboot.h"
#include "paging.h"
#include "pmm.h"
#include "rbtree.h"
#include "selftest/stress.h"
#include "smp.h"
#include "spinlock.h"
#include "task.h"
#include "task_stack.h"
#include "uart.h"
#include "user/usertest.h"
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

// Fixed-capacity text line, so that a line printed by one CPU goes out in a
// single write instead of interleaving with the output of other CPUs.
class LineBuffer {
 public:
  LineBuffer() { buf_[0] = '\0'; }

  LineBuffer& Append(const char* str) {
    while (*str != '\0' && len_ + 1 < kCapacity) {
      buf_[len_++] = *str++;
    }
    buf_[len_] = '\0';
    return *this;
  }

  LineBuffer& AppendDec(const uint64_t value) {
    char digits[kFormatDecBufferSize];
    return Append(FormatDec(value, digits));
  }

  const char* c_str() const { return buf_; }

 private:
  static constexpr int kCapacity = 96;
  char buf_[kCapacity];
  int len_ = 0;
};

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
  // Indices of the spinners that have started (not CPUs).
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

static void StealWorker(void* const raw_arg) {
  StealWorkerArg* const arg = static_cast<StealWorkerArg*>(raw_arg);
  if (arg == nullptr || arg->ctx == nullptr) {
    return;
  }

  TaskScheduler* const scheduler = GetTaskScheduler();
  for (int s = 0; s < kStealWorkerSlices; ++s) {
    const int cpu = CurrentCpuId();
    arg->ctx->stolen_cpu_mask.Add(cpu);
    arg->ctx->observed_cpu_mask.Add(cpu);
    if (cpu != 0) {
      arg->ctx->remote_steal_slices.fetch_add(1, std::memory_order_acq_rel);
    }
    for (int spin = 0; spin < 2000; ++spin) {
      asm volatile("pause" : : : "memory");
    }
    scheduler->Yield();
  }

  const int cpu = CurrentCpuId();
  arg->ctx->stolen_cpu_mask.Add(cpu);
  arg->ctx->observed_cpu_mask.Add(cpu);
  if (cpu != 0) {
    arg->ctx->remote_steal_slices.fetch_add(1, std::memory_order_acq_rel);
  }
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
  ctx->spinner_started_mask.Add(idx);

  // CPU-bound spinner that NEVER calls Yield(); relies strictly on Local APIC
  // timer (0x20) preemption so PreemptProgressWorker tasks can run.
  const TscDeadline deadline(kSelfTestWaitTimeoutTscCycles);
  while (!ctx->stop_spinners.load(std::memory_order_acquire) &&
         !deadline.Expired()) {
    ctx->observed_cpu_mask.Add(CurrentCpuId());
    const int64_t iters =
        ctx->spinner_iters[idx].fetch_add(1, std::memory_order_relaxed) + 1;
    if (iters >= 64 && AllSpinnersStarted(ctx) &&
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
    const uint64_t first_word =
        *reinterpret_cast<const volatile uint64_t*>(kFirstPage);
    const uint64_t last_word =
        *reinterpret_cast<const volatile uint64_t*>(kLastPage + kPageSize - 8);
    void* const heap_block = Kmalloc(64);
    if (heap_block != nullptr) {
      *static_cast<volatile uint64_t*>(heap_block) = kAddressSpaceTestPattern;
      Kfree(heap_block);
    }
    scheduler->SetAddressSpace(nullptr);

    SELFTEST_EXPECT(active_root == space->root());
    SELFTEST_EXPECT(!user_space_empty);
    SELFTEST_EXPECT(first_word == kAddressSpaceTestPattern);
    SELFTEST_EXPECT(last_word == kAddressSpaceTestPattern + 1);
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
};

static void AddressSpaceSwitchingWorker(void* const raw_arg) {
  const AddressSpaceSwitchingArg* const arg =
      static_cast<const AddressSpaceSwitchingArg*>(raw_arg);
  TaskScheduler* const scheduler = GetTaskScheduler();
  scheduler->SetAddressSpace(arg->space);
  for (int slice = 0; slice < kAddressSpaceSwitchingSlices; ++slice) {
    // Whichever CPU runs the task, it must use the task's page tables.
    arg->cpus->Add(CurrentCpuId());
    if (PagingActiveRoot() != arg->space->root() ||
        *reinterpret_cast<const volatile uint64_t*>(kUserSpaceBase) !=
            arg->expected) {
      arg->mismatches->fetch_add(1, std::memory_order_relaxed);
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

// Runs user/usertest.cpp in `mode` in a process of its own until it ends, and
// stores how it ended in `*result`. Returns false if the process can't be
// started.
static bool RunUserTest(const usertest::Mode mode,
                        UserProgramResult* const result) {
  const std::unique_ptr<UserProcess> process =
      UserProcess::Start(UserTestProgram(), usertest::Arg(mode));
  if (process == nullptr) {
    return false;
  }
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

// Checks user processes with user/usertest.cpp: the exit status reaches the
// kernel, system calls work and reject bad arguments, and each kind of fault
// kills the program that raised it, reported with the right vector, error
// code and address, while the kernel carries on. An invalid executable does
// not load. Afterwards the kernel's page tables are active again and every
// frame is back.
static bool TestUserModePrograms() {
  using usertest::Mode;
  SELFTEST_EXPECT(GetTaskScheduler() != nullptr);
  const int64_t pmm_free_before = PmmFreeFrameCount();
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

  SELFTEST_EXPECT(PagingActiveRoot() == PagingKernelRoot());
  SELFTEST_EXPECT(PagingUserSpaceIsEmpty());
  SELFTEST_EXPECT(PmmFreeFrameCount() == pmm_free_before);
  return true;
}

// The most processes that TestUserModePreemption() runs.
constexpr int kMaxUserSpinners = 16;
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
  const int process_count =
      (2 * cpu_count < kMaxUserSpinners) ? 2 * cpu_count : kMaxUserSpinners;
  const int64_t pmm_free_before = PmmFreeFrameCount();
  AtomicCpuMask128 cpus;
  {
    // On an early return, the destructors kill the processes and wait for
    // them.
    std::unique_ptr<UserProcess> processes[kMaxUserSpinners];
    for (int i = 0; i < process_count; ++i) {
      processes[i] = UserProcess::Start(
          UserTestProgram(),
          usertest::Arg(usertest::Mode::kSpin,
                        0x5EED0000 + static_cast<uint64_t>(i)));
      SELFTEST_EXPECT(processes[i] != nullptr);
    }
    // Without preemption, this loop would never end: the CPUs would never
    // switch away from the first spinners, or back to this task.
    for (bool done = false; !done;) {
      done = true;
      for (int i = 0; i < process_count; ++i) {
        const int cpu = processes[i]->last_syscall_cpu();
        if (cpu >= 0) {
          cpus.Add(cpu);
        }
        done = done && processes[i]->syscall_count() >= kUserSpinnerSyscalls;
      }
      scheduler->Yield();
    }
    for (int i = 0; i < process_count; ++i) {
      processes[i]->Kill();
    }
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
