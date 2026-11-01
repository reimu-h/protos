#ifndef PROTOS_USER_PS2KBD_I8042_H_
#define PROTOS_USER_PS2KBD_I8042_H_

#include <cstdint>
#include <type_traits>

#include "user/ps2kbd/ps2_scancode.h"

// Freestanding, header-only Intel 8042 (`i8042`) PS/2 keyboard controller core
// for `user/ps2kbd`. Parameterized over `ps2::Platform` so host unit tests can
// exercise bring-up, hardware quirk handling, and IRQ-driven byte draining
// without system calls.
//
// Hardware safety invariants:
// - Never issues `0xAA` (Controller Self-Test), which resets laptop embedded
//   controllers (`EC`) on physical x86 machines and breaks internal keyboards;
//   neither Hyper-V Gen 1 nor QEMU requires `0xAA`.
// - Keeps PS/2 Port 2 (AUX mouse) disabled (`0xA7`, `CCB` bit 1 = 0, bit 5 = 1)
//   and drops any byte read while `status & 0x20` (`AUXB`) or `status & 0xC0`
//   (timeout/parity error) is set.
// - Detects absent `i8042` controllers (e.g. Hyper-V Gen 2 UEFI VMs) both via
//   floating-bus `status == 0xFF` and via bounded timeout on `0x20` (Read CCB).
// - Requests controller Set 2 -> Set 1 translation (`CCB` bit 6) and reads back
//   `CCB` to fall back to raw Set 2 decoding if the controller keeps bit 6 = 0.
// - Tolerates controllers/emulators that do not ACK `0xF4` (Enable Scanning).
// - Drains `OBF` immediately after setting `CCB` bit 0 (`KBD_INT`) in
//   `EnableIrq1()` and in a loop on every `HandleInterrupt()` call so an
//   edge-triggered ISA IRQ 1 line never stalls with `OBF` stuck high.

namespace protos::ps2 {

constexpr uint16_t kDataPort = 0x60;
constexpr uint16_t kStatusCmdPort = 0x64;

// Status register (`inb(0x64)`) bits.
constexpr uint8_t kStatusOutputFull = 1u << 0;
constexpr uint8_t kStatusInputFull = 1u << 1;
constexpr uint8_t kStatusAuxData = 1u << 5;
constexpr uint8_t kStatusTimeoutError = 1u << 6;
constexpr uint8_t kStatusParityError = 1u << 7;
constexpr uint8_t kStatusErrorMask = kStatusTimeoutError | kStatusParityError;
constexpr uint8_t kStatusFloatingBus = 0xFFu;

// Controller commands (`outb(0x64, cmd)`).
constexpr uint8_t kCmdReadConfig = 0x20;
constexpr uint8_t kCmdWriteConfig = 0x60;
constexpr uint8_t kCmdDisableAux = 0xA7;
constexpr uint8_t kCmdDisableKbd = 0xAD;
constexpr uint8_t kCmdEnableKbd = 0xAE;

// Controller Configuration Byte (`CCB`) bits.
constexpr uint8_t kConfigKbdIrqEnable = 1u << 0;
constexpr uint8_t kConfigAuxIrqEnable = 1u << 1;
constexpr uint8_t kConfigKbdClockDisable = 1u << 4;
constexpr uint8_t kConfigAuxClockDisable = 1u << 5;
constexpr uint8_t kConfigKbdTranslate = 1u << 6;

// Keyboard device commands (`outb(0x60, cmd)`) and replies (`inb(0x60)`).
constexpr uint8_t kDevCmdEnableScan = 0xF4;
constexpr uint8_t kDevReplyAck = 0xFA;
constexpr uint8_t kDevReplyResend = 0xFE;

enum class I8042Status : uint8_t {
  kOk = 0,
  kNotPresent = 1,
  kIoTimeout = 2,
};

// Host services required by `I8042Controller`. Protected non-virtual destructor
// avoids requiring `operator delete` in freestanding ring-3 binaries.
class Platform {
 public:
  virtual uint8_t In8(uint16_t port) = 0;
  virtual void Out8(uint16_t port, uint8_t value) = 0;
  virtual int64_t NowTicks() = 0;
  virtual int64_t TicksPerSecond() = 0;
  virtual void Relax() = 0;
  virtual void PushInput(const char* bytes, int count) = 0;

 protected:
  constexpr Platform() = default;
  ~Platform() = default;
};

class I8042Controller {
 public:
  static constexpr int kFastSpinPolls = 64;
  static constexpr int64_t kPollTimeoutMs = 50;
  static constexpr int kMaxFlushBytes = 64;
  static constexpr int kDrainChunkSize = 64;
  static constexpr int kMaxDrainChunksPerInterrupt = 4;

  constexpr I8042Controller() = default;

  // Quiesces the `i8042` controller, verifies controller presence via `0x20`
  // Read CCB, configures Set 1 translation (falling back to Set 2 if bit 6 is
  // unsupported) with Port 2 disabled and IRQ 1 still masked in CCB, enables
  // Port 1 clock (`0xAE`), and sends `0xF4` Enable Scanning.
  [[nodiscard]] I8042Status Initialize(Platform& platform) {
    platform_ = &platform;
    irq_enabled_ = false;
    ccb_ = 0;
    scancode_set_ = Ps2ScancodeSet::kSet1;
    decoder_.Reset(Ps2ScancodeSet::kSet1);

    if (platform_->In8(kStatusCmdPort) == kStatusFloatingBus) {
      return I8042Status::kNotPresent;
    }

    if (!WriteCommand(kCmdDisableKbd) || !WriteCommand(kCmdDisableAux)) {
      return I8042Status::kNotPresent;
    }

    FlushOutputBuffer();

    uint8_t initial_ccb = 0;
    if (!WriteCommand(kCmdReadConfig) || !ReadControllerByte(&initial_ccb)) {
      return I8042Status::kNotPresent;
    }
    if (initial_ccb == kStatusFloatingBus &&
        platform_->In8(kStatusCmdPort) == kStatusFloatingBus) {
      return I8042Status::kNotPresent;
    }

    // Port 1 stays disabled (`CCB` bit 4) until `0xAE` below: a keyboard sends
    // the bytes that it buffered while its clock was inhibited as soon as the
    // clock runs again, and such a byte could arrive ahead of the `0x20`
    // response and be read back as the `CCB`.
    uint8_t desired_ccb = initial_ccb;
    desired_ccb &=
        static_cast<uint8_t>(~(kConfigKbdIrqEnable | kConfigAuxIrqEnable));
    desired_ccb |= static_cast<uint8_t>(
        kConfigKbdClockDisable | kConfigAuxClockDisable | kConfigKbdTranslate);
    if (!WriteConfigByte(desired_ccb)) {
      return I8042Status::kIoTimeout;
    }

    uint8_t verify_ccb = desired_ccb;
    if (WriteCommand(kCmdReadConfig)) {
      uint8_t readback = 0;
      if (ReadControllerByte(&readback)) {
        verify_ccb = readback;
      }
    }
    ccb_ = verify_ccb;
    scancode_set_ = ((ccb_ & kConfigKbdTranslate) != 0) ? Ps2ScancodeSet::kSet1
                                                        : Ps2ScancodeSet::kSet2;
    decoder_.Reset(scancode_set_);

    if (!WriteCommand(kCmdEnableKbd)) {
      return I8042Status::kIoTimeout;
    }
    ccb_ &= static_cast<uint8_t>(~kConfigKbdClockDisable);
    FlushOutputBuffer();

    // Enable keyboard scanning (`0xF4`). Retry once on `0xFE` Resend, and
    // proceed if the controller/emulator does not synthesize `0xFA`.
    for (int attempt = 0; attempt < 2; ++attempt) {
      if (!WriteData(kDevCmdEnableScan)) {
        break;
      }
      const DeviceReply reply = WaitDeviceAck();
      if (reply == DeviceReply::kAck || reply == DeviceReply::kTimeout) {
        break;
      }
    }
    FlushOutputBuffer();
    return I8042Status::kOk;
  }

  // Sets `CCB` bit 0 (`KBD_INT`) after the kernel has programmed and unmasked
  // the IOAPIC redirection entry for ISA IRQ 1, then drains any byte already
  // sitting in `0x60` so edge-triggered IRQ 1 cannot stall with `OBF` high.
  [[nodiscard]] I8042Status EnableIrq1() {
    if (platform_ == nullptr) {
      return I8042Status::kIoTimeout;
    }
    uint8_t enabled_ccb = ccb_;
    enabled_ccb &=
        static_cast<uint8_t>(~(kConfigAuxIrqEnable | kConfigKbdClockDisable));
    enabled_ccb |=
        static_cast<uint8_t>(kConfigKbdIrqEnable | kConfigAuxClockDisable);
    if (!WriteConfigByte(enabled_ccb)) {
      return I8042Status::kIoTimeout;
    }
    ccb_ = enabled_ccb;
    irq_enabled_ = true;
    HandleInterrupt();
    return I8042Status::kOk;
  }

  // Drains pending bytes from `0x60` while `OBF` is set, ignoring AUX mouse and
  // parity/timeout-error bytes and pushing translated console characters.
  void HandleInterrupt() {
    if (platform_ == nullptr) {
      return;
    }
    for (int chunk = 0; chunk < kMaxDrainChunksPerInterrupt; ++chunk) {
      char out[kDrainChunkSize];
      int out_count = 0;
      bool drained_full_chunk = false;
      for (int i = 0; i < kDrainChunkSize; ++i) {
        const uint8_t status = platform_->In8(kStatusCmdPort);
        if (status == kStatusFloatingBus || (status & kStatusOutputFull) == 0) {
          drained_full_chunk = false;
          break;
        }
        drained_full_chunk = (i + 1 == kDrainChunkSize);
        const uint8_t data = platform_->In8(kDataPort);
        if ((status & (kStatusAuxData | kStatusErrorMask)) != 0) {
          continue;
        }
        const char ch = decoder_.FeedByte(data);
        if (ch != 0) {
          out[out_count++] = ch;
        }
      }
      if (out_count > 0) {
        platform_->PushInput(out, out_count);
      }
      if (!drained_full_chunk) {
        break;
      }
    }
  }

  constexpr Ps2ScancodeSet scancode_set() const { return scancode_set_; }
  constexpr uint8_t config_byte() const { return ccb_; }
  constexpr bool irq_enabled() const { return irq_enabled_; }
  constexpr const Ps2ScancodeDecoder& decoder() const { return decoder_; }

 private:
  enum class DeviceReply : uint8_t {
    kAck = 0,
    kResend = 1,
    kTimeout = 2,
  };

  int64_t PollBudgetTicks() const {
    const int64_t hz = platform_->TicksPerSecond();
    if (hz <= 0) {
      return 2;
    }
    const int64_t ticks = (hz * kPollTimeoutMs + 999) / 1000;
    return ticks < 2 ? 2 : ticks;
  }

  bool WaitInputReady() {
    for (int i = 0; i < kFastSpinPolls; ++i) {
      const uint8_t status = platform_->In8(kStatusCmdPort);
      if (status == kStatusFloatingBus) {
        return false;
      }
      if ((status & kStatusInputFull) == 0) {
        return true;
      }
    }
    const int64_t deadline = platform_->NowTicks() + PollBudgetTicks();
    for (;;) {
      const uint8_t status = platform_->In8(kStatusCmdPort);
      if (status == kStatusFloatingBus) {
        return false;
      }
      if ((status & kStatusInputFull) == 0) {
        return true;
      }
      if (platform_->NowTicks() >= deadline) {
        return false;
      }
      platform_->Relax();
    }
  }

  bool WriteCommand(const uint8_t cmd) {
    if (!WaitInputReady()) {
      return false;
    }
    platform_->Out8(kStatusCmdPort, cmd);
    return true;
  }

  bool WriteData(const uint8_t data) {
    if (!WaitInputReady()) {
      return false;
    }
    platform_->Out8(kDataPort, data);
    return true;
  }

  bool WriteConfigByte(const uint8_t ccb) {
    return WriteCommand(kCmdWriteConfig) && WriteData(ccb);
  }

  bool ReadControllerByte(uint8_t* const out_byte) {
    for (int i = 0; i < kFastSpinPolls; ++i) {
      const uint8_t status = platform_->In8(kStatusCmdPort);
      if (status == kStatusFloatingBus) {
        return false;
      }
      if ((status & kStatusOutputFull) != 0) {
        const uint8_t data = platform_->In8(kDataPort);
        if ((status & kStatusAuxData) != 0) {
          continue;
        }
        if ((status & kStatusErrorMask) != 0) {
          return false;
        }
        *out_byte = data;
        return true;
      }
    }
    const int64_t deadline = platform_->NowTicks() + PollBudgetTicks();
    for (;;) {
      const uint8_t status = platform_->In8(kStatusCmdPort);
      if (status == kStatusFloatingBus) {
        return false;
      }
      // Unlike in the spin above, a discarded AUX byte goes on to the deadline
      // check, so that a status stuck at `OBF | AUXB` cannot poll forever.
      if ((status & kStatusOutputFull) != 0) {
        const uint8_t data = platform_->In8(kDataPort);
        if ((status & kStatusAuxData) == 0) {
          if ((status & kStatusErrorMask) != 0) {
            return false;
          }
          *out_byte = data;
          return true;
        }
      }
      if (platform_->NowTicks() >= deadline) {
        return false;
      }
      platform_->Relax();
    }
  }

  DeviceReply WaitDeviceAck() {
    for (int i = 0; i < kFastSpinPolls; ++i) {
      const uint8_t status = platform_->In8(kStatusCmdPort);
      if (status == kStatusFloatingBus) {
        return DeviceReply::kTimeout;
      }
      if ((status & kStatusOutputFull) != 0) {
        const uint8_t data = platform_->In8(kDataPort);
        if ((status & (kStatusAuxData | kStatusErrorMask)) != 0) {
          continue;
        }
        if (data == kDevReplyAck) {
          return DeviceReply::kAck;
        }
        if (data == kDevReplyResend) {
          return DeviceReply::kResend;
        }
      }
    }
    const int64_t deadline = platform_->NowTicks() + PollBudgetTicks();
    for (;;) {
      const uint8_t status = platform_->In8(kStatusCmdPort);
      if (status == kStatusFloatingBus) {
        return DeviceReply::kTimeout;
      }
      // As in ReadControllerByte(), a discarded byte goes on to the deadline
      // check like any other byte that is not a reply.
      if ((status & kStatusOutputFull) != 0) {
        const uint8_t data = platform_->In8(kDataPort);
        if ((status & (kStatusAuxData | kStatusErrorMask)) == 0) {
          if (data == kDevReplyAck) {
            return DeviceReply::kAck;
          }
          if (data == kDevReplyResend) {
            return DeviceReply::kResend;
          }
        }
      }
      if (platform_->NowTicks() >= deadline) {
        return DeviceReply::kTimeout;
      }
      platform_->Relax();
    }
  }

  void FlushOutputBuffer() {
    for (int i = 0; i < kMaxFlushBytes; ++i) {
      const uint8_t status = platform_->In8(kStatusCmdPort);
      if (status == kStatusFloatingBus || (status & kStatusOutputFull) == 0) {
        break;
      }
      (void)platform_->In8(kDataPort);
    }
  }

  Platform* platform_ = nullptr;
  Ps2ScancodeDecoder decoder_{};
  Ps2ScancodeSet scancode_set_ = Ps2ScancodeSet::kSet1;
  uint8_t ccb_ = 0;
  bool irq_enabled_ = false;
};

static_assert(std::is_trivially_destructible_v<I8042Controller>);

}  // namespace protos::ps2

#endif  // PROTOS_USER_PS2KBD_I8042_H_
