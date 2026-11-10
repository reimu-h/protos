#ifndef PROTOS_USER_NVME_NVME_PLATFORM_H_
#define PROTOS_USER_NVME_NVME_PLATFORM_H_

#include <cstdint>
#include <type_traits>

// Services needed by the freestanding NVMe controller core
// (`nvme_regs.h`, `nvme_controller.h`) and IPC server (`nvme_server.h`).
// Follows the same freestanding rules as `user/usbkbd/xhci_platform.h`:
// header-only, no heap, `constinit`-friendly, trivially destructible globals,
// and protected non-virtual destructors on interfaces.

namespace protos::nvme {

struct DmaBuffer {
  void* va = nullptr;
  uint64_t phys = 0;
  int64_t size = 0;
};

class Platform {
 public:
  // Allocates `size` bytes (> 0, a multiple of 4096) of zeroed, physically
  // contiguous, page-aligned DMA memory, below 4 GiB if `below_4gib`.
  virtual DmaBuffer DmaAlloc(int64_t size, bool below_4gib) = 0;
  virtual int64_t NowTicks() = 0;
  virtual int64_t TicksPerSecond() = 0;
  // Lets time advance by up to one tick while polling reset/enable transitions.
  virtual void Relax() = 0;
  // Waits until any bit in `irq_mask` fires or at most `timeout_ticks` timer
  // ticks elapse (when `timeout_ticks > 0`).
  virtual void WaitForIrq(uint64_t irq_mask, int64_t timeout_ticks) = 0;
  virtual void Log(const char* line, int length) = 0;

 protected:
  constexpr Platform() = default;
  ~Platform() = default;
};

constexpr uint32_t kMmioAllOnes = 0xFFFFFFFFu;

class MmioWindow {
 public:
  virtual uint32_t Read32(int64_t offset) = 0;
  virtual void Write32(int64_t offset, uint32_t value) = 0;
  virtual int64_t Size() const = 0;
  // True if `[offset, offset + size)` is backed by mapped registers without
  // falling into an unmapped MSI-X hole.
  virtual bool Covers(int64_t offset, int64_t size) const = 0;

  uint64_t Read64(const int64_t offset) {
    if (offset < 0 || offset > INT64_MAX - 4) {
      return (static_cast<uint64_t>(kMmioAllOnes) << 32) | kMmioAllOnes;
    }
    const uint32_t lo = Read32(offset);
    const uint32_t hi = Read32(offset + 4);
    return (static_cast<uint64_t>(hi) << 32) | lo;
  }

  void Write64(const int64_t offset, const uint64_t value) {
    if (offset < 0 || offset > INT64_MAX - 4) {
      return;
    }
    Write32(offset, static_cast<uint32_t>(value));
    Write32(offset + 4, static_cast<uint32_t>(value >> 32));
  }

 protected:
  constexpr MmioWindow() = default;
  ~MmioWindow() = default;
};

class MappedMmioWindow final : public MmioWindow {
 public:
  // Holds page 0 plus binary-halved spans up to CAP.DSTRD = 15 (97 pages)
  // across up to 2 MSI-X Table/PBA holes without running out of slots.
  static constexpr int kMaxSegments = 32;

  constexpr MappedMmioWindow() = default;

  void Init(volatile uint8_t* const base, const int64_t size) {
    segment_count_ = 0;
    (void)AddSegment(0, base, size);
  }

  bool AddSegment(const int64_t offset,          //
                  volatile uint8_t* const base,  //
                  const int64_t size) {
    if (base == nullptr || offset < Size() || size <= 0 ||
        ((offset | size) & 3) != 0 || offset > INT64_MAX - size ||
        segment_count_ >= kMaxSegments) {
      return false;
    }
    segments_[segment_count_++] = Segment{offset, size, base};
    return true;
  }

  uint32_t Read32(const int64_t offset) override {
    volatile uint8_t* const address = Translate(offset, 4);
    if (address == nullptr) {
      return kMmioAllOnes;
    }
    asm volatile("" ::: "memory");
    const uint32_t value = *reinterpret_cast<volatile uint32_t*>(address);
    asm volatile("" ::: "memory");
    return value;
  }

  void Write32(const int64_t offset, const uint32_t value) override {
    volatile uint8_t* const address = Translate(offset, 4);
    if (address == nullptr) {
      return;
    }
    asm volatile("" ::: "memory");
    *reinterpret_cast<volatile uint32_t*>(address) = value;
    asm volatile("" ::: "memory");
  }

  int64_t Size() const override {
    if (segment_count_ == 0) {
      return 0;
    }
    const Segment& last = segments_[segment_count_ - 1];
    return last.offset + last.size;
  }

  bool Covers(const int64_t offset, const int64_t size) const override {
    if (offset < 0 || size <= 0 || offset > INT64_MAX - size) {
      return false;
    }
    const int64_t end = offset + size;
    for (int i = 0; i < segment_count_; ++i) {
      const Segment& seg = segments_[i];
      if (offset >= seg.offset && end <= seg.offset + seg.size) {
        return true;
      }
    }
    return false;
  }

 private:
  struct Segment {
    int64_t offset = 0;
    int64_t size = 0;
    volatile uint8_t* base = nullptr;
  };

  volatile uint8_t* Translate(const int64_t offset, const int64_t width) const {
    if (offset < 0 || (offset & 3) != 0 || width <= 0 ||
        offset > INT64_MAX - width) {
      return nullptr;
    }
    const int64_t end = offset + width;
    for (int i = 0; i < segment_count_; ++i) {
      const Segment& seg = segments_[i];
      if (offset >= seg.offset && end <= seg.offset + seg.size) {
        return seg.base + (offset - seg.offset);
      }
    }
    return nullptr;
  }

  Segment segments_[kMaxSegments]{};
  int segment_count_ = 0;
};

static_assert(std::is_trivially_destructible_v<MappedMmioWindow>);

constexpr char kLogPrefix[] = "[NVME] ";
constexpr int kMaxLogLineLength = 128;

class LogLine {
 public:
  constexpr LogLine() = default;

  LogLine& Str(const char* const text) {
    if (text == nullptr) {
      return *this;
    }
    for (int i = 0; text[i] != '\0' && length_ < kMaxLogLineLength - 1; ++i) {
      buffer_[length_++] = text[i];
    }
    return *this;
  }

  LogLine& Dec(const uint64_t value) {
    char digits[20];
    int n = 0;
    uint64_t v = value;
    do {
      digits[n++] = static_cast<char>('0' + (v % 10));
      v /= 10;
    } while (v != 0);
    while (n > 0 && length_ < kMaxLogLineLength - 1) {
      buffer_[length_++] = digits[--n];
    }
    return *this;
  }

  void Emit(Platform& platform) {
    if (length_ < 0) {
      length_ = 0;
    } else if (length_ >= kMaxLogLineLength) {
      length_ = kMaxLogLineLength - 1;
    }
    buffer_[length_++] = '\n';
    platform.Log(buffer_, length_);
  }

 private:
  char buffer_[kMaxLogLineLength] = {};
  int length_ = 0;
};

inline void FormatBdf(const uint8_t bus,     //
                      const uint8_t device,  //
                      const uint8_t func,    //
                      char (&out)[8]) {
  constexpr char kHex[] = "0123456789abcdef";
  out[0] = kHex[(bus >> 4) & 0xF];
  out[1] = kHex[bus & 0xF];
  out[2] = ':';
  out[3] = kHex[(device >> 4) & 0x1];
  out[4] = kHex[device & 0xF];
  out[5] = '.';
  out[6] = kHex[func & 0x7];
  out[7] = '\0';
}

inline int64_t TicksFromMs(const int64_t ms, const int64_t ticks_per_sec) {
  if (ms <= 0 || ticks_per_sec <= 0) {
    return 0;
  }
  constexpr int64_t kMaxTicks = INT64_MAX / 2;
  const int64_t whole_sec = ms / 1000;
  const int64_t rem_ms = ms % 1000;
  if (ticks_per_sec > kMaxTicks ||
      (whole_sec > 0 && whole_sec > kMaxTicks / ticks_per_sec)) {
    return kMaxTicks;
  }
  const int64_t whole_ticks = whole_sec * ticks_per_sec;
  const int64_t q_tps = ticks_per_sec / 1000;
  const int64_t r_tps = ticks_per_sec % 1000;
  if (q_tps > 0 && rem_ms > (kMaxTicks - whole_ticks) / q_tps) {
    return kMaxTicks;
  }
  const int64_t mid_ticks = whole_ticks + rem_ms * q_tps;
  const int64_t rem_ticks = (rem_ms * r_tps + 999) / 1000;
  if (rem_ticks >= kMaxTicks - mid_ticks) {
    return kMaxTicks;
  }
  return mid_ticks + rem_ticks + 1;
}

inline int64_t ClampAndAlignDmaSize(const int64_t size,
                                    const int64_t max_size) {
  constexpr int64_t kDmaPageSize = 4096;
  if (size <= 0 || max_size < kDmaPageSize) {
    return 0;
  }
  const int64_t aligned_max = max_size & ~(kDmaPageSize - 1);
  if (size > aligned_max) {
    return 0;
  }
  return (size + (kDmaPageSize - 1)) & ~(kDmaPageSize - 1);
}

}  // namespace protos::nvme

#endif  // PROTOS_USER_NVME_NVME_PLATFORM_H_
