#ifndef PROTOS_XHCI_SIM_WORLD_H_
#define PROTOS_XHCI_SIM_WORLD_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "user/usb_hid.h"
#include "user/xhci_controller.h"
#include "user/xhci_regs.h"
#include "xhci_sim.h"
#include "xhci_sim_usb.h"

namespace protos::xhci_sim {

// Multi-controller deterministic test harness for xHCI Phase 1 + Phase 2 tests.
// Owns the shared SimClock, SimDmaArena, FakePlatform, and KeyboardLocks state,
// plus up to kMaxControllers (XhciSim, XhciController) pairs and attached
// simulated USB devices.
class SimWorld {
 public:
  static constexpr int kMaxControllers = 4;
  static constexpr int kMaxKeyboards = 256;

  SimWorld() : platform_(&clock_, &arena_) {}

  SimWorld(const SimWorld&) = delete;
  SimWorld& operator=(const SimWorld&) = delete;

  SimClock& clock() { return clock_; }
  const SimClock& clock() const { return clock_; }
  SimDmaArena& arena() { return arena_; }
  const SimDmaArena& arena() const { return arena_; }
  FakePlatform& platform() { return platform_; }
  const FakePlatform& platform() const { return platform_; }
  usb::KeyboardLocks& locks() { return locks_; }
  const usb::KeyboardLocks& locks() const { return locks_; }

  int AddController(const char* const label, const XhciSimConfig& cfg = {},
                    const usb::IrqMode mode = usb::IrqMode::kMsix) {
    const int idx = controller_count_++;
    Slot& slot = slots_[static_cast<size_t>(idx)];
    slot.label = label;
    slot.mode = mode;
    slot.sim.emplace(&clock_, cfg);
    slot.sim->set_dma_arena(&arena_);
    slot.controller.Init(platform_, *slot.sim, slot.label.c_str(), &locks_);
    return idx;
  }

  int controller_count() const { return controller_count_; }

  XhciSim& sim(const int idx = 0) {
    return *slots_[static_cast<size_t>(idx)].sim;
  }
  const XhciSim& sim(const int idx = 0) const {
    return *slots_[static_cast<size_t>(idx)].sim;
  }

  usb::XhciController& controller(const int idx = 0) {
    return slots_[static_cast<size_t>(idx)].controller;
  }
  const usb::XhciController& controller(const int idx = 0) const {
    return slots_[static_cast<size_t>(idx)].controller;
  }

  usb::XhciStatus BringUp(const int idx = 0) {
    Slot& slot = slots_[static_cast<size_t>(idx)];
    const usb::XhciStatus s1 = slot.controller.TakeOwnershipAndReset();
    if (s1 != usb::XhciStatus::kOk) {
      return s1;
    }
    return slot.controller.Start(slot.mode);
  }

  usb::XhciStatus BringUpAll() {
    for (int i = 0; i < controller_count_; ++i) {
      const usb::XhciStatus st = BringUp(i);
      if (st != usb::XhciStatus::kOk) {
        return st;
      }
    }
    return usb::XhciStatus::kOk;
  }

  SimUsbKeyboard& CreateKeyboard(const SimKeyboardConfig& cfg = {}) {
    const int idx = keyboard_count_++;
    keyboards_[static_cast<size_t>(idx)].emplace(cfg);
    return *keyboards_[static_cast<size_t>(idx)];
  }

  SimUsbKeyboard& AttachKeyboard(const int ctrl_idx, const int port,
                                 const SimKeyboardConfig& cfg = {}) {
    SimUsbKeyboard& kbd = CreateKeyboard(cfg);
    sim(ctrl_idx).AttachDevice(port, &kbd);
    return kbd;
  }

  void Detach(const int ctrl_idx, const int port) {
    sim(ctrl_idx).DetachDevice(port);
  }

  // Delivers any pending hardware interrupts or expired NextDeadline() timers
  // across all controllers without advancing the clock.
  int ServiceReady() {
    int calls = 0;
    bool progress = true;
    int guard = 0;
    while (progress && guard < 256) {
      progress = false;
      ++guard;
      for (int i = 0; i < controller_count_; ++i) {
        Slot& slot = slots_[static_cast<size_t>(i)];
        slot.sim->Step();
        const int64_t deadline = slot.controller.NextDeadline();
        const bool deadline_due =
            (deadline != usb::kNoDeadline && clock_.now() >= deadline);
        if (slot.sim->ConsumeIrq() || deadline_due) {
          slot.controller.HandleInterrupt();
          ++calls;
          progress = true;
        }
      }
    }
    return calls;
  }

  // Advances time by `ticks` (1 tick = 4 ms at 250 Hz), delivering only real
  // simulator IRQs and expired NextDeadline() wakeups.
  void RunForTicks(const int64_t ticks) {
    ServiceReady();
    for (int64_t t = 0; t < ticks; ++t) {
      clock_.Advance(1);
      ServiceReady();
    }
  }

  void RunForMs(const int64_t ms) {
    const int64_t ticks = (ms + 3) / 4;
    RunForTicks(ticks);
  }

  // Steps time until `pred()` is true or `max_ticks` elapse.
  template <typename Pred>
  bool RunUntil(Pred pred, const int64_t max_ticks = 2000) {
    ServiceReady();
    if (pred()) {
      return true;
    }
    for (int64_t t = 0; t < max_ticks; ++t) {
      clock_.Advance(1);
      ServiceReady();
      if (pred()) {
        return true;
      }
    }
    return pred();
  }

  // Calls HandleInterrupt() on every controller unconditionally (spurious
  // wake).
  void SpuriousWakeAll() {
    for (int i = 0; i < controller_count_; ++i) {
      slots_[static_cast<size_t>(i)].controller.HandleInterrupt();
    }
  }

  int TotalAttachedKeyboards() const {
    int total = 0;
    for (int i = 0; i < controller_count_; ++i) {
      total +=
          slots_[static_cast<size_t>(i)].controller.attached_keyboard_count();
    }
    return total;
  }

  // Maps an ASCII character to its US QWERTY (modifier, HID usage) pair and
  // taps it on `kbd`, servicing the resulting interrupt IN transfer events.
  void TypeAscii(SimUsbKeyboard& kbd, const std::string_view text) {
    for (const char ch : text) {
      uint8_t mod = 0;
      uint8_t usage = 0;
      if (AsciiToHid(ch, &mod, &usage)) {
        kbd.TapKey(usage, mod);
        ServiceReady();
      }
    }
  }

  static bool AsciiToHid(const char ch, uint8_t* const mod_out,
                         uint8_t* const usage_out) {
    *mod_out = 0;
    *usage_out = 0;
    if (ch >= 'a' && ch <= 'z') {
      *usage_out = static_cast<uint8_t>(0x04 + (ch - 'a'));
      return true;
    }
    if (ch >= 'A' && ch <= 'Z') {
      *mod_out = usb::kHidModLeftShift;
      *usage_out = static_cast<uint8_t>(0x04 + (ch - 'A'));
      return true;
    }
    if (ch >= '1' && ch <= '9') {
      *usage_out = static_cast<uint8_t>(0x1E + (ch - '1'));
      return true;
    }
    if (ch == '0') {
      *usage_out = 0x27;
      return true;
    }
    switch (ch) {
      case '\r':
      case '\n':
        *usage_out = 0x28;
        return true;
      case '\x1b':
        *usage_out = 0x29;
        return true;
      case '\b':
        *usage_out = 0x2A;
        return true;
      case '\t':
        *usage_out = 0x2B;
        return true;
      case ' ':
        *usage_out = 0x2C;
        return true;
      case '-':
        *usage_out = 0x2D;
        return true;
      case '_':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x2D;
        return true;
      case '=':
        *usage_out = 0x2E;
        return true;
      case '+':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x2E;
        return true;
      case '[':
        *usage_out = 0x2F;
        return true;
      case '{':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x2F;
        return true;
      case ']':
        *usage_out = 0x30;
        return true;
      case '}':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x30;
        return true;
      case '\\':
        *usage_out = 0x31;
        return true;
      case '|':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x31;
        return true;
      case ';':
        *usage_out = 0x33;
        return true;
      case ':':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x33;
        return true;
      case '\'':
        *usage_out = 0x34;
        return true;
      case '"':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x34;
        return true;
      case '`':
        *usage_out = 0x35;
        return true;
      case '~':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x35;
        return true;
      case ',':
        *usage_out = 0x36;
        return true;
      case '<':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x36;
        return true;
      case '.':
        *usage_out = 0x37;
        return true;
      case '>':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x37;
        return true;
      case '/':
        *usage_out = 0x38;
        return true;
      case '?':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x38;
        return true;
      case '!':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x1E;
        return true;
      case '@':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x1F;
        return true;
      case '#':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x20;
        return true;
      case '$':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x21;
        return true;
      case '%':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x22;
        return true;
      case '^':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x23;
        return true;
      case '&':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x24;
        return true;
      case '*':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x25;
        return true;
      case '(':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x26;
        return true;
      case ')':
        *mod_out = usb::kHidModLeftShift;
        *usage_out = 0x27;
        return true;
      default:
        return false;
    }
  }

 private:
  struct Slot {
    std::string label;
    usb::IrqMode mode = usb::IrqMode::kMsix;
    std::optional<XhciSim> sim;
    usb::XhciController controller{};
  };

  SimClock clock_{};
  SimDmaArena arena_{};
  FakePlatform platform_;
  usb::KeyboardLocks locks_{};
  int controller_count_ = 0;
  std::array<Slot, kMaxControllers> slots_{};
  int keyboard_count_ = 0;
  std::array<std::optional<SimUsbKeyboard>, kMaxKeyboards> keyboards_{};
};

}  // namespace protos::xhci_sim

#endif  // PROTOS_XHCI_SIM_WORLD_H_
