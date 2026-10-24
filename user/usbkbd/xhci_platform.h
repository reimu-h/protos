#ifndef PROTOS_USER_USBKBD_XHCI_PLATFORM_H_
#define PROTOS_USER_USBKBD_XHCI_PLATFORM_H_

#include <cstdint>
#include <type_traits>

// The services that the freestanding xHCI USB keyboard driver core
// (user/usbkbd/xhci_regs.h, user/usbkbd/xhci_controller.h,
// user/usbkbd/usb_hid.h) needs from the program that hosts it, and small
// helpers built on them. The ring-3 driver implements `Platform` with system
// calls and wraps its mmio_map() mapping in a `MappedMmioWindow`; the host unit
// tests implement both with a simulated controller (user/usbkbd/xhci_sim.h).
// The core itself makes no system calls.
//
// Rules for all core headers. User programs link one translation unit with
// user/crt.o and get no C++ runtime (no operator new/delete, no __cxa_*
// functions, and crt never runs .init_array), so:
// - The core is header-only.
// - No heap: no new/delete, std::vector, std::function or std::shared_ptr.
//   Dependencies are passed as references that must outlive the objects that
//   use them (instead of the usual std::shared_ptr for long-lived
//   dependencies).
// - Interfaces have protected non-virtual destructors: a virtual destructor
//   needs operator delete.
// - Objects of static storage duration must be constant-initialized: core
//   types have constexpr default constructors and are trivially destructible,
//   and programs declare their globals `constinit`. No function-local statics
//   with dynamic initialization (they need __cxa_guard_*).
// - No exceptions or RTTI: errors are [[nodiscard]] status values.
// - Small stack frames: user stacks are 16 KiB.

namespace protos::usb {

// Physically contiguous, page-aligned, zeroed memory that a controller can
// access by DMA at `phys`. `va == nullptr` means that the allocation failed.
struct DmaBuffer {
  void* va = nullptr;
  uint64_t phys = 0;
  int64_t size = 0;
};

// Services of the host program. Must outlive every controller that uses it.
class Platform {
 public:
  // Allocates `size` bytes (> 0, a multiple of 4096) of zeroed, physically
  // contiguous, page-aligned DMA memory, below 4 GiB if `below_4gib` (for
  // controllers without 64-bit addressing). The memory is never freed
  // individually; it is released when the program ends.
  virtual DmaBuffer DmaAlloc(int64_t size, bool below_4gib) = 0;
  // A monotonic tick counter and its rate (ring 3: get_ticks(), 250 Hz).
  virtual int64_t NowTicks() = 0;
  virtual int64_t TicksPerSecond() = 0;
  // Called once between two polls of a bounded wait. Must let time pass, at
  // most about one tick (ring 3: sleep until the next tick).
  virtual void Relax() = 0;
  // Appends `count` (>= 1; callers skip empty reports) decoded keyboard bytes
  // to the console input.
  virtual void PushInput(const char* bytes, int count) = 0;
  // Writes one complete log line: `length` (<= kMaxLogLineLength) bytes that
  // end with '\n'. Ring 3 must write it with one write() call so that the
  // line is never interleaved with other output.
  virtual void Log(const char* line, int length) = 0;

 protected:
  constexpr Platform() = default;
  ~Platform() = default;
};

// 32-bit access to the registers of one controller: its memory BAR 0, from
// byte 0 to Size(). Offsets are 4-byte aligned. The core only accesses
// offsets that it has checked against Size().
class MmioWindow {
 public:
  virtual uint32_t Read32(int64_t offset) = 0;
  virtual void Write32(int64_t offset, uint32_t value) = 0;
  virtual int64_t Size() const = 0;

 protected:
  constexpr MmioWindow() = default;
  ~MmioWindow() = default;
};

// What a register read returns from a device that does not decode its BAR
// (memory space disabled, in D3hot, or removed), and what `MappedMmioWindow`
// returns for an offset outside the window.
constexpr uint32_t kMmioAllOnes = 0xFFFFFFFFu;

// An `MmioWindow` over mapped registers (ring 3: the uncacheable mappings that
// mmio_map() returns): one mapping of the whole window, or segments mapped
// separately with holes between them (the kernel does not map pages that
// hold the device's MSI-X table or PBA). Size() is the end of the last
// segment. Accesses outside every segment or unaligned ones are not
// performed: reads return kMmioAllOnes and writes are dropped, so a bad
// offset can't fault the driver.
class MappedMmioWindow final : public MmioWindow {
 public:
  // A PCI function has at most two contiguous MSI-X structures (one MSI-X Table
  // and one PBA), which can create at most 2 unmappable holes in BAR0 and split
  // it into at most 2 + 1 = 3 mappable segments. Fixed inline storage is used
  // because freestanding user programs have no heap.
  static constexpr int kMaxSegments = 4;

  constexpr MappedMmioWindow() = default;

  // One mapping of registers [0, size) at `base` (none if `base` is null).
  void Init(volatile uint8_t* const base, const int64_t size) {
    segment_count_ = 0;
    (void)AddSegment(0, base, size);
  }

  // Adds the mapping of registers [offset, offset + size) at `base`, above
  // the segments added before. Returns false and adds nothing if `base` is
  // null, the range is empty, not dword-aligned or not above the previous
  // segment, or the window has kMaxSegments segments already.
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
    volatile uint8_t* const address = Translate(offset);
    if (address == nullptr) {
      return kMmioAllOnes;
    }
    // Every MMIO access is a two-sided compiler barrier so DMA reads/writes
    // cannot be reordered across it.
    asm volatile("" ::: "memory");
    const uint32_t value = *reinterpret_cast<volatile uint32_t*>(address);
    asm volatile("" ::: "memory");
    return value;
  }

  void Write32(const int64_t offset, const uint32_t value) override {
    volatile uint8_t* const address = Translate(offset);
    if (address == nullptr) {
      return;
    }
    // Earlier stores to DMA memory (e.g. TRBs before a doorbell write) and
    // later loads/stores must not move across this write.
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

 private:
  struct Segment {
    int64_t offset = 0;
    int64_t size = 0;
    volatile uint8_t* base = nullptr;
  };

  // The mapped address of the dword at `offset`, or nullptr.
  volatile uint8_t* Translate(const int64_t offset) const {
    if ((offset & 3) != 0) {
      return nullptr;
    }
    for (int i = 0; i < segment_count_; ++i) {
      const Segment& segment = segments_[i];
      if (offset >= segment.offset &&
          offset - segment.offset <= segment.size - 4) {
        return segment.base + (offset - segment.offset);
      }
    }
    return nullptr;
  }

  Segment segments_[kMaxSegments] = {};
  int segment_count_ = 0;
};

static_assert(std::is_trivially_destructible_v<MappedMmioWindow>);

// The number of ticks to wait so that at least `ms` milliseconds pass: `ms`
// rounded up to ticks, plus one tick because the current tick may end right
// after the start was read.
inline int64_t TicksForMs(Platform& platform, const int64_t ms) {
  constexpr int64_t kMaxTicksPerSecond = 1'000'000'000;
  constexpr int64_t kMaxMs = 86'400'000;
  int64_t ticks_per_second = platform.TicksPerSecond();
  if (ticks_per_second < 1) {
    ticks_per_second = 1;
  } else if (ticks_per_second > kMaxTicksPerSecond) {
    ticks_per_second = kMaxTicksPerSecond;
  }
  const int64_t clamped_ms = ms < 0 ? 0 : (ms > kMaxMs ? kMaxMs : ms);
  return (clamped_ms * ticks_per_second + 999) / 1000 + 1;
}

// Converts a tick count into milliseconds (rounded down), for log lines.
inline int64_t TicksToMs(Platform& platform, const int64_t ticks) {
  const int64_t ticks_per_second = platform.TicksPerSecond();
  if (ticks_per_second < 1 || ticks < 0) {
    return 0;
  }
  return ticks * 1000 / ticks_per_second;
}

// Polls `done()` until it returns true or `timeout_ms` has passed, calling
// `platform.Relax()` once between two polls. The deadline is in ticks, not in
// polls. After the deadline `done()` is checked once more, so a caller that
// was preempted past the deadline does not report a false timeout. Returns
// the last result of `done()`.
template <typename Done>
bool WaitUntil(Platform& platform, const int64_t timeout_ms, const Done& done) {
  const int64_t deadline =
      platform.NowTicks() + TicksForMs(platform, timeout_ms);
  for (;;) {
    if (done()) {
      return true;
    }
    if (platform.NowTicks() >= deadline) {
      return done();
    }
    platform.Relax();
  }
}

// Waits at least `ms` milliseconds without polling anything.
inline void DelayMs(Platform& platform, const int64_t ms) {
  const int64_t deadline = platform.NowTicks() + TicksForMs(platform, ms);
  while (platform.NowTicks() < deadline) {
    platform.Relax();
  }
}

// The longest log line, including its '\n'. One write() system call of this
// size is written to the serial console in one piece.
constexpr int kMaxLogLineLength = 128;
// Every line that the driver logs starts with this prefix.
constexpr char kLogPrefix[] = "[USBKBD] ";

inline constexpr char kLowercaseHexDigits[] = "0123456789abcdef";

// Builds one log line in a fixed buffer. Text beyond kMaxLogLineLength - 1
// characters is dropped; Emit() always ends the line with '\n'.
class LogLine {
 public:
  constexpr LogLine() = default;

  LogLine& Str(const char* const text) {
    if (text != nullptr) {
      for (int i = 0; text[i] != '\0'; ++i) {
        Put(text[i]);
      }
    }
    return *this;
  }

  LogLine& Dec(const int64_t value) {
    uint64_t magnitude = static_cast<uint64_t>(value);
    if (value < 0) {
      Put('-');
      magnitude = 0 - magnitude;
    }
    char digits[20] = {};
    int count = 0;
    do {
      digits[count++] = static_cast<char>('0' + magnitude % 10);
      magnitude /= 10;
    } while (magnitude != 0);
    while (count > 0) {
      Put(digits[--count]);
    }
    return *this;
  }

  // Lowercase hexadecimal without a prefix, at least `min_digits` digits.
  LogLine& Hex(const uint64_t value, const int min_digits) {
    int digits = 1;
    while (digits < 16 && (value >> (4 * digits)) != 0) {
      ++digits;
    }
    if (digits < min_digits) {
      digits = min_digits > 16 ? 16 : min_digits;
    }
    for (int i = digits - 1; i >= 0; --i) {
      Put(kLowercaseHexDigits[(value >> (4 * i)) & 0xF]);
    }
    return *this;
  }

  // Passes the line, ended with '\n', to `platform.Log()`.
  void Emit(Platform& platform) {
    data_[length_] = '\n';
    data_[length_ + 1] = '\0';
    platform.Log(data_, length_ + 1);
  }

  const char* data() const { return data_; }
  int length() const { return length_; }

 private:
  void Put(const char ch) {
    if (length_ < kMaxLogLineLength - 1) {
      data_[length_++] = ch;
    }
  }

  char data_[kMaxLogLineLength + 1] = {};
  int length_ = 0;
};

// Formats a PCI address as "bb:dd.f" (lowercase hexadecimal), the form that
// the driver's log lines use.
inline void FormatBdf(const int bus,       //
                      const int device,    //
                      const int function,  //
                      char (&out)[8]) {
  out[0] = kLowercaseHexDigits[(bus >> 4) & 0xF];
  out[1] = kLowercaseHexDigits[bus & 0xF];
  out[2] = ':';
  out[3] = kLowercaseHexDigits[(device >> 4) & 0x1];
  out[4] = kLowercaseHexDigits[device & 0xF];
  out[5] = '.';
  out[6] = kLowercaseHexDigits[function & 0x7];
  out[7] = '\0';
}

}  // namespace protos::usb

#endif  // PROTOS_USER_USBKBD_XHCI_PLATFORM_H_
