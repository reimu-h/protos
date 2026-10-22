#ifndef PROTOS_XHCI_SIM_USB_H_
#define PROTOS_XHCI_SIM_USB_H_

// Simulated USB 2.0/3.x devices and USB HID Boot Protocol Keyboard device
// model for host-side testing of the ring-3 xHCI driver.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>

namespace protos::xhci_sim {

struct SetupPacket {
  uint8_t bm_request_type = 0;
  uint8_t b_request = 0;
  uint16_t w_value = 0;
  uint16_t w_index = 0;
  uint16_t w_length = 0;
};

enum class ControlOutcome : uint8_t {
  kAck = 0,
  kStall,
  kTransactionError,
  kBabble,
  kNoResponse,
};

enum class IntrOutcome : uint8_t {
  kNak = 0,
  kData,
  kStall,
  kTransactionError,
  kBabble,
};

class SimUsbDevice {
 public:
  virtual ~SimUsbDevice() = default;
  virtual int speed() const = 0;
  virtual uint8_t ep0_max_packet_size() const = 0;
  virtual void OnBusReset() = 0;
  virtual void OnSetAddress(uint8_t address) = 0;
  virtual ControlOutcome OnControl(const SetupPacket& setup,  //
                                   uint8_t* data,             //
                                   uint16_t max_len,          //
                                   uint16_t* actual_len) = 0;
  virtual IntrOutcome PollInterruptIn(uint8_t ep_dci,        //
                                      uint8_t* buf,          //
                                      uint16_t max_len,      //
                                      uint16_t* actual_len,  //
                                      int64_t now_tick) = 0;
  virtual bool stall_on_setup_stage() const { return false; }
  virtual int64_t control_delay_ticks() const { return 0; }
};

struct SimKeyboardConfig {
  int speed = 3;                    // 1=FS, 2=LS, 3=HS, 4=SS, 5=SSP
  uint8_t ep0_max_packet_size = 0;  // 0 = speed default
  uint8_t device_class = 0;         // 9 = USB Hub
  uint8_t num_configurations = 1;
  uint8_t config_value = 1;
  uint8_t kbd_interface_number = 0;
  uint8_t kbd_interface_class = 3;     // 3 = HID
  uint8_t kbd_interface_subclass = 1;  // 1 = Boot
  uint8_t kbd_interface_protocol = 1;  // 1 = Keyboard (2 = Mouse)
  uint8_t kbd_ep_address = 0x81;
  uint16_t intr_max_packet_size = 8;
  uint8_t intr_b_interval = 0;  // 0 = speed default (10 FS/LS, 4 HS/SS)
  uint8_t ss_max_burst = 0;
  bool composite_mouse_on_iface0 = false;
  bool include_hid_descriptor = true;
  bool stall_set_protocol = false;
  bool stall_set_idle = false;
  bool stall_on_setup_stage = false;
  int fail_get_device_desc_attempts = 0;
  int drop_control_transfer_attempts = 0;
  int64_t control_delay_ticks = 0;
  int64_t ticks_per_second = 250;
  std::vector<uint8_t> custom_config_descriptor;
};

inline SimKeyboardConfig LowSpeedKeyboardConfig() {
  SimKeyboardConfig cfg;
  cfg.speed = 2;
  cfg.ep0_max_packet_size = 8;
  cfg.intr_b_interval = 10;
  return cfg;
}

inline SimKeyboardConfig FullSpeedKeyboardConfig(const uint8_t ep0_mps = 8) {
  SimKeyboardConfig cfg;
  cfg.speed = 1;
  cfg.ep0_max_packet_size = ep0_mps;
  cfg.intr_b_interval = 10;
  return cfg;
}

inline SimKeyboardConfig HighSpeedKeyboardConfig() {
  SimKeyboardConfig cfg;
  cfg.speed = 3;
  cfg.ep0_max_packet_size = 64;
  cfg.intr_b_interval = 4;
  return cfg;
}

inline SimKeyboardConfig SuperSpeedKeyboardConfig(const uint8_t max_burst = 0) {
  SimKeyboardConfig cfg;
  cfg.speed = 4;
  cfg.ep0_max_packet_size = 64;  // Encoded as 9 (2^9 = 512) in descriptor
  cfg.intr_b_interval = 4;
  cfg.ss_max_burst = max_burst;
  return cfg;
}

inline SimKeyboardConfig CompositeKeyboardConfig() {
  SimKeyboardConfig cfg;
  cfg.speed = 3;
  cfg.composite_mouse_on_iface0 = true;
  cfg.kbd_interface_number = 1;
  cfg.kbd_ep_address = 0x82;  // DCI 5
  cfg.intr_b_interval = 4;
  return cfg;
}

inline SimKeyboardConfig MouseOnlyDeviceConfig() {
  SimKeyboardConfig cfg;
  cfg.speed = 3;
  cfg.kbd_interface_protocol = 2;  // Mouse
  return cfg;
}

inline SimKeyboardConfig HubDeviceConfig() {
  SimKeyboardConfig cfg;
  cfg.speed = 3;
  cfg.device_class = 9;  // Hub
  return cfg;
}

class SimUsbKeyboard final : public SimUsbDevice {
 public:
  static constexpr int kMaxQueuedReports = 16;

  explicit SimUsbKeyboard(const SimKeyboardConfig& config = SimKeyboardConfig{})
      : config_(config) {
    ResetState();
  }

  int speed() const override { return config_.speed; }

  uint8_t ep0_max_packet_size() const override {
    if (config_.speed >= 4) {
      return 64;  // Descriptor byte 7 is 9 (2^9 = 512); wire MPS is 512
    }
    if (config_.speed == 3) {
      return 64;
    }
    if (config_.speed == 2) {
      return 8;
    }
    if (config_.ep0_max_packet_size != 0) {
      return config_.ep0_max_packet_size;
    }
    return 64;
  }

  bool stall_on_setup_stage() const override {
    return config_.stall_on_setup_stage;
  }

  int64_t control_delay_ticks() const override {
    return config_.control_delay_ticks;
  }

  void OnBusReset() override {
    ++bus_reset_count_;
    ResetState();
  }

  void OnSetAddress(const uint8_t address) override { address_ = address; }

  ControlOutcome OnControl(const SetupPacket& setup,  //
                           uint8_t* const data,       //
                           const uint16_t max_len,    //
                           uint16_t* const actual_len) override {
    *actual_len = 0;
    if (dropped_controls_ < config_.drop_control_transfer_attempts) {
      ++dropped_controls_;
      return ControlOutcome::kNoResponse;
    }

    // Standard IN: GET_DESCRIPTOR (bmRequestType 0x80, bRequest 6).
    if (setup.bm_request_type == 0x80 && setup.b_request == 6) {
      const uint8_t desc_type = static_cast<uint8_t>(setup.w_value >> 8);
      if (desc_type == 1) {
        if (failed_dev_desc_ < config_.fail_get_device_desc_attempts) {
          ++failed_dev_desc_;
          return ControlOutcome::kTransactionError;
        }
        const std::vector<uint8_t> dev = BuildDeviceDescriptor();
        const uint16_t copy_len =
            max_len < dev.size() ? max_len : static_cast<uint16_t>(dev.size());
        std::memcpy(data, dev.data(), copy_len);
        *actual_len = copy_len;
        return ControlOutcome::kAck;
      }
      if (desc_type == 2) {
        const std::vector<uint8_t> cfg = BuildConfigDescriptor();
        const uint16_t copy_len =
            max_len < cfg.size() ? max_len : static_cast<uint16_t>(cfg.size());
        std::memcpy(data, cfg.data(), copy_len);
        *actual_len = copy_len;
        return ControlOutcome::kAck;
      }
      return ControlOutcome::kStall;
    }

    // Standard OUT: SET_CONFIGURATION (bmRequestType 0x00, bRequest 9).
    if (setup.bm_request_type == 0x00 && setup.b_request == 9) {
      configured_ = (setup.w_value == config_.config_value);
      protocol_ = 1;  // Default to Report Protocol on configuration
      ++set_config_count_;
      return ControlOutcome::kAck;
    }

    // Class Interface OUT: SET_PROTOCOL (bmRequestType 0x21, bRequest 0x0B).
    if (setup.bm_request_type == 0x21 && setup.b_request == 0x0B) {
      ++set_protocol_count_;
      last_protocol_iface_ = setup.w_index;
      if (config_.stall_set_protocol) {
        return ControlOutcome::kStall;
      }
      protocol_ = static_cast<uint8_t>(setup.w_value & 0xFFu);
      return ControlOutcome::kAck;
    }

    // Class Interface OUT: SET_IDLE (bmRequestType 0x21, bRequest 0x0A).
    if (setup.bm_request_type == 0x21 && setup.b_request == 0x0A) {
      ++set_idle_count_;
      last_idle_iface_ = setup.w_index;
      if (config_.stall_set_idle) {
        return ControlOutcome::kStall;
      }
      idle_rate_4ms_ = static_cast<uint8_t>((setup.w_value >> 8) & 0xFFu);
      return ControlOutcome::kAck;
    }

    return ControlOutcome::kStall;
  }

  IntrOutcome PollInterruptIn(const uint8_t ep_dci,        //
                              uint8_t* const buf,          //
                              const uint16_t max_len,      //
                              uint16_t* const actual_len,  //
                              const int64_t now_tick) override {
    *actual_len = 0;
    const uint8_t expected_dci =
        static_cast<uint8_t>((config_.kbd_ep_address & 0x0Fu) * 2u + 1u);
    if (!configured_ || ep_dci != expected_dci) {
      return IntrOutcome::kNak;
    }
    if (!injected_outcomes_.empty()) {
      const IntrOutcome forced = injected_outcomes_.front();
      injected_outcomes_.pop_front();
      return forced;
    }
    if (!reports_.empty()) {
      const std::array<uint8_t, 8> report = reports_.front();
      reports_.pop_front();
      last_report_ = report;
      last_report_tick_ = now_tick;
      const uint16_t copy_len = max_len < 8 ? max_len : 8;
      std::memcpy(buf, report.data(), copy_len);
      *actual_len = copy_len;
      return IntrOutcome::kData;
    }
    if (idle_rate_4ms_ > 0) {
      const int64_t period_ms = static_cast<int64_t>(idle_rate_4ms_) * 4;
      const int64_t period_ticks =
          (period_ms * config_.ticks_per_second + 999) / 1000;
      if (now_tick - last_report_tick_ >=
          (period_ticks > 0 ? period_ticks : 1)) {
        last_report_tick_ = now_tick;
        const uint16_t copy_len = max_len < 8 ? max_len : 8;
        std::memcpy(buf, last_report_.data(), copy_len);
        *actual_len = copy_len;
        return IntrOutcome::kData;
      }
    }
    return IntrOutcome::kNak;
  }

  void PushReport(const std::array<uint8_t, 8>& report) {
    if (static_cast<int>(reports_.size()) >= kMaxQueuedReports) {
      reports_.pop_front();
    }
    reports_.push_back(report);
  }

  void PressKey(const uint8_t usage, const uint8_t modifiers = 0) {
    held_modifiers_ = modifiers;
    bool present = false;
    for (int i = 0; i < 6; ++i) {
      if (held_keys_[static_cast<size_t>(i)] == usage) {
        present = true;
        break;
      }
    }
    if (!present) {
      for (int i = 0; i < 6; ++i) {
        if (held_keys_[static_cast<size_t>(i)] == 0) {
          held_keys_[static_cast<size_t>(i)] = usage;
          break;
        }
      }
    }
    EnqueueCurrentState();
  }

  void ReleaseKey(const uint8_t usage, const uint8_t modifiers = 0) {
    held_modifiers_ = modifiers;
    for (int i = 0; i < 6; ++i) {
      if (held_keys_[static_cast<size_t>(i)] == usage) {
        held_keys_[static_cast<size_t>(i)] = 0;
      }
    }
    EnqueueCurrentState();
  }

  void ReleaseAllKeys() {
    held_modifiers_ = 0;
    held_keys_.fill(0);
    EnqueueCurrentState();
  }

  void TapKey(const uint8_t usage, const uint8_t modifiers = 0) {
    PressKey(usage, modifiers);
    ReleaseKey(usage, 0);
  }

  void InjectInterruptError(const IntrOutcome outcome, const int count = 1) {
    for (int i = 0; i < count; ++i) {
      injected_outcomes_.push_back(outcome);
    }
  }

  bool has_pending_reports() const {
    return !reports_.empty() || !injected_outcomes_.empty();
  }
  bool configured() const { return configured_; }
  uint8_t address() const { return address_; }
  uint8_t protocol() const { return protocol_; }
  uint8_t idle_rate_4ms() const { return idle_rate_4ms_; }
  int bus_reset_count() const { return bus_reset_count_; }
  int set_config_count() const { return set_config_count_; }
  int set_protocol_count() const { return set_protocol_count_; }
  int set_idle_count() const { return set_idle_count_; }
  uint16_t last_protocol_interface() const { return last_protocol_iface_; }
  uint16_t last_idle_interface() const { return last_idle_iface_; }

 private:
  void ResetState() {
    address_ = 0;
    configured_ = false;
    protocol_ = 1;
    idle_rate_4ms_ = 125;  // Default 500 ms before SET_IDLE(0)
    last_report_.fill(0);
    last_report_tick_ = 0;
    held_modifiers_ = 0;
    held_keys_.fill(0);
    reports_.clear();
  }

  void EnqueueCurrentState() {
    std::array<uint8_t, 8> r = {};
    r[0] = held_modifiers_;
    r[1] = 0;
    int out = 2;
    for (int i = 0; i < 6; ++i) {
      if (held_keys_[static_cast<size_t>(i)] != 0) {
        r[static_cast<size_t>(out++)] = held_keys_[static_cast<size_t>(i)];
      }
    }
    PushReport(r);
  }

  std::vector<uint8_t> BuildDeviceDescriptor() const {
    const uint16_t bcd_usb = config_.speed >= 4 ? 0x0300 : 0x0200;
    uint8_t b_mps0 = ep0_max_packet_size();
    if (config_.speed >= 4) {
      b_mps0 = 9;  // 2^9 = 512 bytes for SuperSpeed
    }
    return {
        18,
        1,  // bDescriptorType = Device
        static_cast<uint8_t>(bcd_usb & 0xFFu),
        static_cast<uint8_t>(bcd_usb >> 8),
        config_.device_class,
        0,
        0,
        b_mps0,
        0x27,
        0x06,  // idVendor = 0x0627 (QEMU)
        0x01,
        0x00,  // idProduct = 0x0001
        0x00,
        0x01,
        1,
        2,
        3,
        config_.num_configurations,
    };
  }

  std::vector<uint8_t> BuildConfigDescriptor() const {
    if (!config_.custom_config_descriptor.empty()) {
      return config_.custom_config_descriptor;
    }
    std::vector<uint8_t> body;
    uint8_t num_ifaces = 1;
    if (config_.composite_mouse_on_iface0) {
      num_ifaces = 2;
      // Interface 0: Boot Mouse on 0x81.
      const uint8_t mouse_iface[] = {
          9, 4, 0, 0, 1, 3, 1, 2, 0, 7, 5, 0x81, 0x03, 4, 0, 10,
      };
      body.insert(body.end(), std::begin(mouse_iface), std::end(mouse_iface));
    }

    const uint8_t b_interval =
        config_.intr_b_interval != 0
            ? config_.intr_b_interval
            : static_cast<uint8_t>(config_.speed <= 2 ? 10 : 4);
    const uint8_t kbd_iface[] = {
        9,
        4,
        config_.kbd_interface_number,
        0,
        1,
        config_.kbd_interface_class,
        config_.kbd_interface_subclass,
        config_.kbd_interface_protocol,
        0,
    };
    body.insert(body.end(), std::begin(kbd_iface), std::end(kbd_iface));

    if (config_.include_hid_descriptor) {
      const uint8_t hid_desc[] = {9, 0x21, 0x11, 0x01, 0, 1, 0x22, 63, 0};
      body.insert(body.end(), std::begin(hid_desc), std::end(hid_desc));
    }

    const uint8_t ep_desc[] = {
        7,
        5,
        config_.kbd_ep_address,
        0x03,
        static_cast<uint8_t>(config_.intr_max_packet_size & 0xFFu),
        static_cast<uint8_t>((config_.intr_max_packet_size >> 8) & 0xFFu),
        b_interval,
    };
    body.insert(body.end(), std::begin(ep_desc), std::end(ep_desc));

    if (config_.speed >= 4) {
      const uint16_t bpi = static_cast<uint16_t>(config_.intr_max_packet_size *
                                                 (1u + config_.ss_max_burst));
      const uint8_t ss_comp[] = {
          6,
          0x30,
          config_.ss_max_burst,
          0,
          static_cast<uint8_t>(bpi & 0xFFu),
          static_cast<uint8_t>(bpi >> 8),
      };
      body.insert(body.end(), std::begin(ss_comp), std::end(ss_comp));
    }

    const uint16_t total_len = static_cast<uint16_t>(9 + body.size());
    std::vector<uint8_t> full = {
        9,
        2,
        static_cast<uint8_t>(total_len & 0xFFu),
        static_cast<uint8_t>(total_len >> 8),
        num_ifaces,
        config_.config_value,
        0,
        0xA0,
        50,
    };
    full.insert(full.end(), body.begin(), body.end());
    return full;
  }

  SimKeyboardConfig config_;
  uint8_t address_ = 0;
  bool configured_ = false;
  uint8_t protocol_ = 1;
  uint8_t idle_rate_4ms_ = 125;
  int bus_reset_count_ = 0;
  int set_config_count_ = 0;
  int set_protocol_count_ = 0;
  int set_idle_count_ = 0;
  uint16_t last_protocol_iface_ = 0xFFFFu;
  uint16_t last_idle_iface_ = 0xFFFFu;
  int failed_dev_desc_ = 0;
  int dropped_controls_ = 0;
  int64_t last_report_tick_ = 0;
  std::array<uint8_t, 8> last_report_ = {};
  uint8_t held_modifiers_ = 0;
  std::array<uint8_t, 6> held_keys_ = {};
  std::deque<std::array<uint8_t, 8>> reports_;
  std::deque<IntrOutcome> injected_outcomes_;
};

}  // namespace protos::xhci_sim

#endif  // PROTOS_XHCI_SIM_USB_H_
