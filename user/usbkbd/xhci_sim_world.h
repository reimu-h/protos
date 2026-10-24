#ifndef PROTOS_USER_USBKBD_XHCI_SIM_WORLD_H_
#define PROTOS_USER_USBKBD_XHCI_SIM_WORLD_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "user/usbkbd/usb_hid.h"
#include "user/usbkbd/xhci_controller.h"
#include "user/usbkbd/xhci_regs.h"
#include "user/usbkbd/xhci_sim.h"
#include "user/usbkbd/xhci_sim_usb.h"

namespace protos::xhci_sim {

// Multi-controller deterministic test harness for xHCI Phase 1 + Phase 2 tests.
// Owns the shared SimClock, SimDmaArena, FakePlatform, and KeyboardLocks state,
// plus up to kMaxControllers (XhciSim, XhciController) pairs and attached
// simulated USB devices.
class SimWorld {
 public:
  static constexpr int kMaxControllers = 4;
  static constexpr int kMaxKeyboards = 256;
  static constexpr int kMaxHubs = 64;

  SimWorld() : platform_(&clock_, &arena_) {}

  // Checks every hub: a status-change report that repeats an uncleared change
  // means the driver re-armed the status-change IN before clearing what it
  // was told (a storm on real hubs), and a hub class request before
  // SET_CONFIGURATION has an undefined response (USB 2.0 11.24.2.5).
  ~SimWorld() {
    for (int i = 0; i < hub_count_; ++i) {
      const SimUsbHub& hub = *hubs_[static_cast<size_t>(i)];
      const int repeats = hub.redundant_status_reports();
      if (repeats != 0 && !allow_redundant_hub_reports_) {
        ADD_FAILURE() << "hub #" << i
                      << " re-reported an uncleared status change " << repeats
                      << " times";
      }
      if (hub.class_requests_before_config() != 0) {
        ADD_FAILURE() << "hub #" << i << " got "
                      << hub.class_requests_before_config()
                      << " class requests before SET_CONFIGURATION";
      }
    }
  }

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

  SimUsbHub& CreateHub(const SimHubConfig& cfg = {}) {
    const int idx = hub_count_++;
    hubs_[static_cast<size_t>(idx)].emplace(cfg);
    return *hubs_[static_cast<size_t>(idx)];
  }

  SimUsbHub& AttachHub(const int ctrl_idx,  //
                       const int port,      //
                       const SimHubConfig& cfg = {}) {
    SimUsbHub& hub = CreateHub(cfg);
    sim(ctrl_idx).AttachDevice(port, &hub);
    return hub;
  }

  SimUsbHub& AttachHubToHub(SimUsbHub& parent_hub,  //
                            const int port_1based,  //
                            const SimHubConfig& cfg = {}) {
    SimUsbHub& child = CreateHub(cfg);
    parent_hub.AttachDevice(port_1based, &child);
    return child;
  }

  SimUsbKeyboard& AttachKeyboardToHub(SimUsbHub& hub,         //
                                      const int port_1based,  //
                                      const SimKeyboardConfig& cfg = {}) {
    SimUsbKeyboard& kbd = CreateKeyboard(cfg);
    hub.AttachDevice(port_1based, &kbd);
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
          ++wakeups_;
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

  // Steps time until controller `idx` has no pending deadline.
  bool RunUntilIdle(const int idx, const int64_t max_ticks = 2000) {
    return RunUntil(
        [&] { return controller(idx).NextDeadline() == usb::kNoDeadline; },
        max_ticks);
  }

  // The enabled slot of controller `idx` below `root_port` with Route String
  // `route`, or -1.
  int FindSlot(const int idx, const int root_port, const uint32_t route) {
    for (int s = 1; s < sim(idx).slot_capacity(); ++s) {
      const XhciSim::SimSlot& sl = sim(idx).slot(s);
      if (sl.enabled && sl.port == root_port && sl.route_string == route) {
        return s;
      }
    }
    return -1;
  }

  // HandleInterrupt() calls made by ServiceReady(): driver wakeups.
  int64_t wakeups() const { return wakeups_; }

  // For tests with a hostile hub whose change bits cannot be cleared.
  void AllowRedundantHubReports() { allow_redundant_hub_reports_ = true; }

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

  struct HidKeyStroke {
    uint8_t modifier = 0;
    uint8_t usage = 0;
  };

  // Maps an ASCII character to its US QWERTY (modifier, HID usage) pair and
  // taps it on `kbd`, servicing the resulting interrupt IN transfer events.
  void TypeAscii(SimUsbKeyboard& kbd, const std::string_view text) {
    for (const char ch : text) {
      const StatusOr<HidKeyStroke> stroke = AsciiToHid(ch);
      if (stroke.ok()) {
        kbd.TapKey(stroke->usage, stroke->modifier);
        ServiceReady();
      }
    }
  }

  static StatusOr<HidKeyStroke> AsciiToHid(const char ch) {
    if (ch >= 'a' && ch <= 'z') {
      return HidKeyStroke{.modifier = 0,
                          .usage = static_cast<uint8_t>(0x04 + (ch - 'a'))};
    }
    if (ch >= 'A' && ch <= 'Z') {
      return HidKeyStroke{.modifier = usb::kHidModLeftShift,
                          .usage = static_cast<uint8_t>(0x04 + (ch - 'A'))};
    }
    if (ch >= '1' && ch <= '9') {
      return HidKeyStroke{.modifier = 0,
                          .usage = static_cast<uint8_t>(0x1E + (ch - '1'))};
    }
    if (ch == '0') {
      return HidKeyStroke{.modifier = 0, .usage = 0x27};
    }
    switch (ch) {
      case '\r':
      case '\n':
        return HidKeyStroke{.modifier = 0, .usage = 0x28};
      case '\x1b':
        return HidKeyStroke{.modifier = 0, .usage = 0x29};
      case '\b':
        return HidKeyStroke{.modifier = 0, .usage = 0x2A};
      case '\t':
        return HidKeyStroke{.modifier = 0, .usage = 0x2B};
      case ' ':
        return HidKeyStroke{.modifier = 0, .usage = 0x2C};
      case '-':
        return HidKeyStroke{.modifier = 0, .usage = 0x2D};
      case '_':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x2D};
      case '=':
        return HidKeyStroke{.modifier = 0, .usage = 0x2E};
      case '+':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x2E};
      case '[':
        return HidKeyStroke{.modifier = 0, .usage = 0x2F};
      case '{':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x2F};
      case ']':
        return HidKeyStroke{.modifier = 0, .usage = 0x30};
      case '}':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x30};
      case '\\':
        return HidKeyStroke{.modifier = 0, .usage = 0x31};
      case '|':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x31};
      case ';':
        return HidKeyStroke{.modifier = 0, .usage = 0x33};
      case ':':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x33};
      case '\'':
        return HidKeyStroke{.modifier = 0, .usage = 0x34};
      case '"':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x34};
      case '`':
        return HidKeyStroke{.modifier = 0, .usage = 0x35};
      case '~':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x35};
      case ',':
        return HidKeyStroke{.modifier = 0, .usage = 0x36};
      case '<':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x36};
      case '.':
        return HidKeyStroke{.modifier = 0, .usage = 0x37};
      case '>':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x37};
      case '/':
        return HidKeyStroke{.modifier = 0, .usage = 0x38};
      case '?':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x38};
      case '!':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x1E};
      case '@':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x1F};
      case '#':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x20};
      case '$':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x21};
      case '%':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x22};
      case '^':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x23};
      case '&':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x24};
      case '*':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x25};
      case '(':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x26};
      case ')':
        return HidKeyStroke{.modifier = usb::kHidModLeftShift, .usage = 0x27};
      default:
        return kStatusError;
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
  int hub_count_ = 0;
  std::array<std::optional<SimUsbHub>, kMaxHubs> hubs_{};
  int64_t wakeups_ = 0;
  bool allow_redundant_hub_reports_ = false;
};

}  // namespace protos::xhci_sim

#endif  // PROTOS_USER_USBKBD_XHCI_SIM_WORLD_H_
