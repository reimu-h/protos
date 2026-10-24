#ifndef PROTOS_USER_USBKBD_XHCI_SIM_USB_H_
#define PROTOS_USER_USBKBD_XHCI_SIM_USB_H_

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
  virtual SimUsbDevice* ResolveDownstreamDevice(int /*port_1based*/) {
    return nullptr;
  }
  // The device's USB address; 0 is the default address (after a bus reset).
  virtual uint8_t address() const = 0;
  // Appends the addresses of the devices behind this hub's connected and
  // enabled downstream ports (recursively) that share its USB 2.0 broadcast
  // segment.
  virtual void AppendUsb2SegmentAddresses(std::vector<uint8_t>* /*out*/) const {
  }
  // The data toggle (SuperSpeed: the sequence number, modeled by its parity)
  // of the next data packet of IN endpoint `ep_dci`, or -1 if the device
  // does not model it. OnInterruptInAck() advances it: the host ACKed a
  // packet (USB 2.0 8.6.4).
  virtual int InterruptInToggle(uint8_t /*ep_dci*/) const { return -1; }
  virtual void OnInterruptInAck(uint8_t /*ep_dci*/) {}
  // The device NAKs the status stage of SET_ADDRESS forever (broken firmware
  // or a hostile device), so that Address Device never completes.
  virtual bool naks_set_address() const { return false; }
};

// The device side of an interrupt IN endpoint: its data toggle (sequence
// number parity) and its Halt feature, which a functional STALL sets and only
// CLEAR_FEATURE(ENDPOINT_HALT), SET_CONFIGURATION or a bus reset clears (USB
// 2.0 8.4.5, 9.4.5); the first also resets the toggle, halted or not.
struct SimInEndpointState {
  uint8_t toggle = 0;
  bool halted = false;
};

// Standard CLEAR_FEATURE(ENDPOINT_HALT) (bmRequestType 0x02, bRequest 1,
// wValue 0) to endpoint address `w_index`.
inline bool IsClearEndpointHalt(const SetupPacket& setup) {
  return setup.bm_request_type == 0x02 && setup.b_request == 1 &&
         setup.w_value == 0;
}

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
  // NAKs the status stage of SET_ADDRESS forever.
  bool nak_set_address = false;
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

  bool naks_set_address() const override { return config_.nak_set_address; }

  int InterruptInToggle(const uint8_t ep_dci) const override {
    return ep_dci == InterruptDci() ? intr_ep_.toggle : -1;
  }

  void OnInterruptInAck(const uint8_t ep_dci) override {
    if (ep_dci == InterruptDci()) {
      intr_ep_.toggle ^= 1u;
    }
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
      intr_ep_ = SimInEndpointState{};
      ++set_config_count_;
      return ControlOutcome::kAck;
    }

    // Standard Endpoint OUT: CLEAR_FEATURE(ENDPOINT_HALT).
    if (IsClearEndpointHalt(setup)) {
      ++clear_halt_count_;
      if (setup.w_index == config_.kbd_ep_address) {
        intr_ep_ = SimInEndpointState{};
        return ControlOutcome::kAck;
      }
      return (setup.w_index & 0x7Fu) == 0 ? ControlOutcome::kAck
                                          : ControlOutcome::kStall;
    }

    // Class Interface OUT: SET_PROTOCOL (bmRequestType 0x21, bRequest 0x0B).
    if (setup.bm_request_type == 0x21 && setup.b_request == 0x0B) {
      ++set_protocol_count_;
      last_protocol_iface_ = setup.w_index;
      if (!configured_ || setup.w_index != config_.kbd_interface_number ||
          config_.stall_set_protocol) {
        return ControlOutcome::kStall;
      }
      protocol_ = static_cast<uint8_t>(setup.w_value & 0xFFu);
      return ControlOutcome::kAck;
    }

    // Class Interface OUT: SET_IDLE (bmRequestType 0x21, bRequest 0x0A).
    if (setup.bm_request_type == 0x21 && setup.b_request == 0x0A) {
      ++set_idle_count_;
      last_idle_iface_ = setup.w_index;
      if (!configured_ || setup.w_index != config_.kbd_interface_number ||
          config_.stall_set_idle) {
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
    const uint8_t expected_dci = InterruptDci();
    if (!configured_ || ep_dci != expected_dci) {
      return IntrOutcome::kNak;
    }
    if (intr_ep_.halted) {
      return IntrOutcome::kStall;  // Until CLEAR_FEATURE(ENDPOINT_HALT)
    }
    if (!injected_outcomes_.empty()) {
      const IntrOutcome forced = injected_outcomes_.front();
      injected_outcomes_.pop_front();
      if (forced == IntrOutcome::kStall) {
        intr_ep_.halted = true;
      }
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
  uint8_t address() const override { return address_; }
  uint8_t protocol() const { return protocol_; }
  uint8_t idle_rate_4ms() const { return idle_rate_4ms_; }
  int bus_reset_count() const { return bus_reset_count_; }
  int set_config_count() const { return set_config_count_; }
  int set_protocol_count() const { return set_protocol_count_; }
  int set_idle_count() const { return set_idle_count_; }
  uint16_t last_protocol_interface() const { return last_protocol_iface_; }
  uint16_t last_idle_interface() const { return last_idle_iface_; }
  // The device side of the interrupt IN endpoint.
  bool interrupt_halted() const { return intr_ep_.halted; }
  uint8_t interrupt_toggle() const { return intr_ep_.toggle; }
  int clear_halt_count() const { return clear_halt_count_; }

 private:
  uint8_t InterruptDci() const {
    return static_cast<uint8_t>((config_.kbd_ep_address & 0x0Fu) * 2u + 1u);
  }

  void ResetState() {
    address_ = 0;
    configured_ = false;
    protocol_ = 1;
    idle_rate_4ms_ = 125;  // Default 500 ms before SET_IDLE(0)
    intr_ep_ = SimInEndpointState{};
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
  int clear_halt_count_ = 0;
  SimInEndpointState intr_ep_;
  int64_t last_report_tick_ = 0;
  std::array<uint8_t, 8> last_report_ = {};
  uint8_t held_modifiers_ = 0;
  std::array<uint8_t, 6> held_keys_ = {};
  std::deque<std::array<uint8_t, 8>> reports_;
  std::deque<IntrOutcome> injected_outcomes_;
};

struct SimHubConfig {
  int speed = 3;                    // 1=FS, 3=HS, 4=SS
  uint8_t ep0_max_packet_size = 0;  // 0 = speed default (64)
  int num_ports = 4;                // 1..15
  uint8_t device_protocol =
      0xFF;  // 0xFF = auto (0=FS, 1=HS STT, 2=HS MTT, 3=SS)
  uint8_t interface_protocol = 0xFF;  // alt 0; 0xFF = auto (0=FS/STT/SS, 1=MTT)
  bool alt1_multi_tt = false;         // Also alt 1 with bInterfaceProtocol 2
  bool multi_tt = false;
  uint8_t ttt = 0;  // TT Think Time (0..3 in wHubCharacteristics[6:5])
  uint8_t pwr_on_2_pwr_good = 10;     // in 2 ms units (10 -> 20 ms)
  uint8_t ep_address = 0x81;          // Interrupt IN endpoint (DCI 3)
  uint16_t intr_max_packet_size = 0;  // 0 = auto ((num_ports + 8) / 8)
  uint8_t intr_b_interval = 0;        // 0 = auto (255 FS, 12 HS, 8 SS)
  uint8_t ss_max_burst = 0;
  uint16_t vendor_id = 0x0bda;
  uint16_t product_id = 0x0411;
  bool stall_set_hub_depth = false;
  int fail_get_device_desc_attempts = 0;
  int fail_get_hub_desc_attempts = 0;
  int fail_get_port_status_attempts = 0;
  int fail_set_port_feature_attempts = 0;
  int fail_clear_port_feature_attempts = 0;
  int fail_clear_hub_feature_attempts = 0;
  // Fails only SET_FEATURE(PORT_RESET) this many times, with this outcome.
  int fail_port_reset_attempts = 0;
  ControlOutcome port_reset_error = ControlOutcome::kStall;
  int port_reset_delay_polls = 0;
  bool port_reset_drops_connection_while_active = false;
  bool port_reset_never_completes = false;
  bool corrupt_hub_descriptor = false;
  // USB 2.0 11.24.2.5: the response to hub class requests before
  // SET_CONFIGURATION is undefined; a strict hub STALLs them.
  bool stall_class_requests_until_configured = false;
  ControlOutcome injected_ctrl_error = ControlOutcome::kTransactionError;
};

using SimUsbHubConfig = SimHubConfig;

inline SimHubConfig Usb2FullSpeedHubConfig(const int num_ports = 4) {
  SimHubConfig cfg;
  cfg.speed = 1;
  cfg.num_ports = num_ports;
  cfg.device_protocol = 0;
  cfg.interface_protocol = 0;
  cfg.intr_b_interval = 255;
  return cfg;
}

inline SimHubConfig Usb2HighSpeedSingleTtHubConfig(const int num_ports = 4,
                                                   const uint8_t ttt = 0) {
  SimHubConfig cfg;
  cfg.speed = 3;
  cfg.num_ports = num_ports;
  cfg.device_protocol = 1;
  cfg.interface_protocol = 0;
  cfg.multi_tt = false;
  cfg.ttt = static_cast<uint8_t>(ttt & 0x3u);
  cfg.intr_b_interval = 12;
  return cfg;
}

// USB 2.0 11.23.1: a multi-TT hub runs single-TT in alternate setting 0
// (bInterfaceProtocol 1) until the host selects alternate setting 1
// (bInterfaceProtocol 2).
inline SimHubConfig Usb2HighSpeedMultiTtHubConfig(const int num_ports = 4,
                                                  const uint8_t ttt = 2) {
  SimHubConfig cfg;
  cfg.speed = 3;
  cfg.num_ports = num_ports;
  cfg.device_protocol = 2;
  cfg.interface_protocol = 1;
  cfg.alt1_multi_tt = true;
  cfg.multi_tt = true;
  cfg.ttt = static_cast<uint8_t>(ttt & 0x3u);
  cfg.intr_b_interval = 12;
  return cfg;
}

// A non-compliant multi-TT hub that reports bInterfaceProtocol 2 on its only
// alternate setting (the driver then programs MTT = 1).
inline SimHubConfig Usb2HighSpeedMultiTtOnAlt0HubConfig(const int num_ports = 4,
                                                        const uint8_t ttt = 2) {
  SimHubConfig cfg = Usb2HighSpeedMultiTtHubConfig(num_ports, ttt);
  cfg.interface_protocol = 2;
  cfg.alt1_multi_tt = false;
  return cfg;
}

inline SimHubConfig Usb3SuperSpeedHubConfig(const int num_ports = 4) {
  SimHubConfig cfg;
  cfg.speed = 4;
  cfg.num_ports = num_ports;
  cfg.device_protocol = 3;
  cfg.interface_protocol = 0;
  cfg.intr_b_interval = 8;
  return cfg;
}

class SimUsbHub final : public SimUsbDevice {
 public:
  static constexpr int kMaxSimHubPorts = 15;

  struct HubPort {
    SimUsbDevice* device = nullptr;
    bool powered = false;
    uint16_t status = 0;
    uint16_t change = 0;
    int reset_polls_remaining = 0;
    int reset_count = 0;
  };

  explicit SimUsbHub(const SimHubConfig& config = SimHubConfig{})
      : config_(config) {
    if (config_.num_ports < 1) {
      config_.num_ports = 1;
    } else if (config_.num_ports > kMaxSimHubPorts) {
      config_.num_ports = kMaxSimHubPorts;
    }
    ResetProtocolState();
  }

  int speed() const override { return config_.speed; }

  uint8_t ep0_max_packet_size() const override {
    if (config_.speed >= 4 || config_.speed == 3) {
      return 64;
    }
    if (config_.ep0_max_packet_size != 0) {
      return config_.ep0_max_packet_size;
    }
    return 64;
  }

  void OnBusReset() override {
    ++bus_reset_count_;
    ResetProtocolState();
    for (int i = 0; i < config_.num_ports; ++i) {
      if (ports_[static_cast<size_t>(i)].device != nullptr) {
        ports_[static_cast<size_t>(i)].device->OnBusReset();
      }
    }
  }

  void OnSetAddress(const uint8_t address) override { address_ = address; }

  int InterruptInToggle(const uint8_t ep_dci) const override {
    return ep_dci == InterruptDci() ? intr_ep_.toggle : -1;
  }

  void OnInterruptInAck(const uint8_t ep_dci) override {
    if (ep_dci == InterruptDci()) {
      intr_ep_.toggle ^= 1u;
    }
  }

  SimUsbDevice* ResolveDownstreamDevice(const int port_1based) override {
    if (!configured_ || port_1based < 1 || port_1based > config_.num_ports) {
      return nullptr;
    }
    const HubPort& hp = ports_[static_cast<size_t>(port_1based - 1)];
    if (!hp.powered || (hp.status & 0x0003u) != 0x0003u) {
      return nullptr;
    }
    return hp.device;
  }

  void AppendUsb2SegmentAddresses(
      std::vector<uint8_t>* const out) const override {
    if (config_.speed >= 4) {
      return;  // SuperSpeed hubs route packets by Route String.
    }
    for (int p = 1; p <= config_.num_ports; ++p) {
      const HubPort& hp = ports_[static_cast<size_t>(p - 1)];
      if (hp.device == nullptr || !hp.powered ||
          (hp.status & 0x0003u) != 0x0003u) {
        continue;  // Only connected and enabled ports repeat traffic.
      }
      out->push_back(hp.device->address());
      hp.device->AppendUsb2SegmentAddresses(out);
    }
  }

  void AttachDevice(const int port_1based, SimUsbDevice* const dev) {
    if (port_1based < 1 || port_1based > config_.num_ports) {
      return;
    }
    HubPort& hp = ports_[static_cast<size_t>(port_1based - 1)];
    hp.device = dev;
    if (dev != nullptr) {
      dev->OnBusReset();
    }
    if (hp.powered && dev != nullptr) {
      ApplyAttachedPortStatus(hp);
      LatchPortChange(hp, 0x0001u);  // C_PORT_CONNECTION
    }
  }

  void AttachDeviceWithoutChange(const int port_1based,
                                 SimUsbDevice* const dev) {
    if (port_1based < 1 || port_1based > config_.num_ports) {
      return;
    }
    HubPort& hp = ports_[static_cast<size_t>(port_1based - 1)];
    hp.device = dev;
    if (dev != nullptr) {
      dev->OnBusReset();
    }
    if (hp.powered && dev != nullptr) {
      ApplyAttachedPortStatus(hp);
    }
  }

  void DetachDevice(const int port_1based) {
    if (port_1based < 1 || port_1based > config_.num_ports) {
      return;
    }
    HubPort& hp = ports_[static_cast<size_t>(port_1based - 1)];
    hp.device = nullptr;
    hp.reset_polls_remaining = 0;
    if (hp.powered) {
      hp.status = PowerBit();
      LatchPortChange(hp, 0x0001u);  // C_PORT_CONNECTION
    } else {
      hp.status = 0;
      SetPortChange(hp, 0);
    }
  }

  void TriggerConnectBounce(const int port_1based) {
    if (port_1based < 1 || port_1based > config_.num_ports) {
      return;
    }
    HubPort& hp = ports_[static_cast<size_t>(port_1based - 1)];
    if (hp.powered) {
      LatchPortChange(hp, 0x0001u);  // C_PORT_CONNECTION
    }
  }

  // A transient over-current on downstream port `port_1based` cuts its power
  // and latches C_PORT_OVER_CURRENT (USB 2.0 11.12.5).
  void PulsePortOverCurrent(const int port_1based) {
    if (port_1based < 1 || port_1based > config_.num_ports) {
      return;
    }
    HubPort& hp = ports_[static_cast<size_t>(port_1based - 1)];
    if (hp.device != nullptr) {
      hp.device->OnBusReset();
    }
    hp.powered = false;
    hp.reset_polls_remaining = 0;
    hp.status = 0;
    LatchPortChange(hp, 1u << 3);  // C_PORT_OVER_CURRENT
  }

  ControlOutcome OnControl(const SetupPacket& setup,  //
                           uint8_t* const data,       //
                           const uint16_t max_len,    //
                           uint16_t* const actual_len) override {
    *actual_len = 0;

    // Standard IN: GET_DESCRIPTOR (0x80, 6).
    if (setup.bm_request_type == 0x80 && setup.b_request == 6) {
      const uint8_t desc_type = static_cast<uint8_t>(setup.w_value >> 8);
      if (desc_type == 1) {
        if (failed_dev_desc_ < config_.fail_get_device_desc_attempts) {
          ++failed_dev_desc_;
          return config_.injected_ctrl_error;
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

    // Standard OUT: SET_CONFIGURATION (0x00, 9).
    if (setup.bm_request_type == 0x00 && setup.b_request == 9) {
      configured_ = (setup.w_value == 1);
      intr_ep_ = SimInEndpointState{};
      ++set_config_count_;
      return ControlOutcome::kAck;
    }

    // Standard Endpoint OUT: CLEAR_FEATURE(ENDPOINT_HALT).
    if (IsClearEndpointHalt(setup)) {
      ++clear_halt_count_;
      if (setup.w_index == config_.ep_address) {
        intr_ep_ = SimInEndpointState{};
        return ControlOutcome::kAck;
      }
      return (setup.w_index & 0x7Fu) == 0 ? ControlOutcome::kAck
                                          : ControlOutcome::kStall;
    }

    // Standard Interface OUT: SET_INTERFACE (0x01, 11).
    if (setup.bm_request_type == 0x01 && setup.b_request == 11) {
      alt_setting_ = static_cast<uint8_t>(setup.w_value & 0xFFu);
      ++set_interface_count_;
      return ControlOutcome::kAck;
    }

    // Hub class requests (USB 2.0 11.24.2) before SET_CONFIGURATION: counted
    // whether or not this hub STALLs them.
    if ((setup.bm_request_type & 0x60u) == 0x20u && !configured_) {
      ++class_requests_before_config_;
      if (config_.stall_class_requests_until_configured) {
        return ControlOutcome::kStall;
      }
    }

    // Hub Class Device IN: GET_DESCRIPTOR (0xA0, 6).
    if (setup.bm_request_type == 0xA0 && setup.b_request == 6) {
      if (failed_hub_desc_ < config_.fail_get_hub_desc_attempts) {
        ++failed_hub_desc_;
        return config_.injected_ctrl_error;
      }
      const uint8_t desc_type = static_cast<uint8_t>(setup.w_value >> 8);
      const uint8_t expected_type = config_.speed >= 4 ? 0x2Au : 0x29u;
      if (desc_type != expected_type && desc_type != 0x00u) {
        return ControlOutcome::kStall;
      }
      ++get_hub_desc_count_;
      std::vector<uint8_t> desc = BuildHubDescriptor();
      if (config_.corrupt_hub_descriptor && !desc.empty()) {
        desc[2] = 0;  // bNbrPorts = 0 (invalid)
      }
      const uint16_t copy_len =
          max_len < desc.size() ? max_len : static_cast<uint16_t>(desc.size());
      std::memcpy(data, desc.data(), copy_len);
      *actual_len = copy_len;
      return ControlOutcome::kAck;
    }

    // Hub Class Device IN: GET_HUB_STATUS (0xA0, 0): wHubStatus then
    // wHubChange (USB 2.0 11.24.2.6: bit 0 local power, bit 1 over-current).
    if (setup.bm_request_type == 0xA0 && setup.b_request == 0) {
      ++get_hub_status_count_;
      const uint8_t status_bytes[4] = {
          static_cast<uint8_t>(hub_status_ & 0xFFu),
          static_cast<uint8_t>(hub_status_ >> 8),
          static_cast<uint8_t>(hub_change_ & 0xFFu),
          static_cast<uint8_t>(hub_change_ >> 8),
      };
      const uint16_t copy_len = max_len < 4 ? max_len : 4;
      std::memcpy(data, status_bytes, copy_len);
      *actual_len = copy_len;
      return ControlOutcome::kAck;
    }

    // Hub Class Device OUT: CLEAR_HUB_FEATURE (0x20, 1): C_HUB_LOCAL_POWER (0)
    // or C_HUB_OVER_CURRENT (1); other selectors are a Request Error.
    if (setup.bm_request_type == 0x20 && setup.b_request == 1) {
      ++clear_hub_feature_count_;
      if (failed_clear_hub_feature_ < config_.fail_clear_hub_feature_attempts) {
        ++failed_clear_hub_feature_;
        return config_.injected_ctrl_error;
      }
      if (setup.w_value > 1 || setup.w_index != 0) {
        return ControlOutcome::kStall;
      }
      SetHubChange(static_cast<uint16_t>(hub_change_ & ~(1u << setup.w_value)));
      return ControlOutcome::kAck;
    }

    // Hub Class Device OUT: SET_HUB_DEPTH (0x20, 0x0C).
    if (setup.bm_request_type == 0x20 && setup.b_request == 0x0C) {
      ++set_hub_depth_count_;
      if (config_.stall_set_hub_depth) {
        return ControlOutcome::kStall;
      }
      hub_depth_ = setup.w_value;
      return ControlOutcome::kAck;
    }

    // Hub Class Port IN: GET_PORT_STATUS (0xA3, 0).
    if (setup.bm_request_type == 0xA3 && setup.b_request == 0) {
      const int port = static_cast<int>(setup.w_index & 0xFFu);
      if (port < 1 || port > config_.num_ports) {
        return ControlOutcome::kStall;
      }
      ++get_port_status_count_;
      ++port_status_reads_[static_cast<size_t>(port - 1)];
      if (failed_port_status_ < config_.fail_get_port_status_attempts) {
        ++failed_port_status_;
        return config_.injected_ctrl_error;
      }
      HubPort& hp = ports_[static_cast<size_t>(port - 1)];
      if (hp.reset_polls_remaining > 0) {
        if (--hp.reset_polls_remaining == 0) {
          CompletePortReset(hp);
        }
      }
      const uint8_t raw[4] = {
          static_cast<uint8_t>(hp.status & 0xFFu),
          static_cast<uint8_t>((hp.status >> 8) & 0xFFu),
          static_cast<uint8_t>(hp.change & 0xFFu),
          static_cast<uint8_t>((hp.change >> 8) & 0xFFu),
      };
      const uint16_t copy_len = max_len < 4 ? max_len : 4;
      std::memcpy(data, raw, copy_len);
      *actual_len = copy_len;
      return ControlOutcome::kAck;
    }

    // Hub Class Port OUT: SET_PORT_FEATURE (0x23, 3).
    if (setup.bm_request_type == 0x23 && setup.b_request == 3) {
      const int port = static_cast<int>(setup.w_index & 0xFFu);
      if (port < 1 || port > config_.num_ports) {
        return ControlOutcome::kStall;
      }
      if (failed_set_feature_ < config_.fail_set_port_feature_attempts) {
        ++failed_set_feature_;
        return config_.injected_ctrl_error;
      }
      HubPort& hp = ports_[static_cast<size_t>(port - 1)];
      const uint16_t feat = setup.w_value;
      if (feat == 8) {  // PORT_POWER
        hp.powered = true;
        hp.status |= PowerBit();
        if (hp.device != nullptr && (hp.status & 0x0001u) == 0) {
          ApplyAttachedPortStatus(hp);
          LatchPortChange(hp, 0x0001u);  // C_PORT_CONNECTION
        }
        return ControlOutcome::kAck;
      }
      if (feat == 4 || feat == 28) {  // PORT_RESET or BH_PORT_RESET
        if (failed_port_reset_ < config_.fail_port_reset_attempts) {
          ++failed_port_reset_;
          return config_.port_reset_error;
        }
        ++hp.reset_count;
        if (!hp.powered || hp.device == nullptr) {
          return ControlOutcome::kAck;
        }
        if (config_.port_reset_never_completes) {
          hp.status |= 0x0010u;   // PORT_RESET
          hp.status &= ~0x0002u;  // PORT_ENABLE cleared
        } else if (config_.port_reset_delay_polls > 0) {
          hp.status |= 0x0010u;
          hp.status &= ~0x0002u;
          if (config_.port_reset_drops_connection_while_active) {
            hp.status &= ~0x0001u;
            LatchPortChange(hp, 0x0001u);  // C_PORT_CONNECTION
          }
          hp.reset_polls_remaining = config_.port_reset_delay_polls;
        } else {
          CompletePortReset(hp);
        }
        return ControlOutcome::kAck;
      }
      return ControlOutcome::kAck;
    }

    // Hub Class Port OUT: CLEAR_PORT_FEATURE (0x23, 1).
    if (setup.bm_request_type == 0x23 && setup.b_request == 1) {
      const int port = static_cast<int>(setup.w_index & 0xFFu);
      if (port < 1 || port > config_.num_ports) {
        return ControlOutcome::kStall;
      }
      if (failed_clear_feature_ < config_.fail_clear_port_feature_attempts) {
        ++failed_clear_feature_;
        return config_.injected_ctrl_error;
      }
      HubPort& hp = ports_[static_cast<size_t>(port - 1)];
      const uint16_t feat = setup.w_value;
      uint16_t next = hp.change;
      switch (feat) {
        case 1:  // PORT_ENABLE
          hp.status &= ~0x0002u;
          break;
        case 8:  // PORT_POWER
          hp.powered = false;
          hp.status = 0;
          next = 0;
          break;
        case 16:  // C_PORT_CONNECTION
          next &= ~(1u << 0);
          break;
        case 17:  // C_PORT_ENABLE
          next &= ~(1u << 1);
          break;
        case 18:  // C_PORT_SUSPEND
          next &= ~(1u << 2);
          break;
        case 19:  // C_PORT_OVER_CURRENT
          next &= ~(1u << 3);
          break;
        case 20:  // C_PORT_RESET
          next &= ~(1u << 4);
          break;
        case 25:  // C_PORT_LINK_STATE
          next &= ~(1u << 6);
          break;
        case 26:  // C_PORT_CONFIG_ERROR
          next &= ~(1u << 7);
          break;
        case 29:  // C_BH_PORT_RESET
          next &= ~(1u << 5);
          break;
        default:
          break;
      }
      SetPortChange(hp, next);
      return ControlOutcome::kAck;
    }

    return ControlOutcome::kStall;
  }

  IntrOutcome PollInterruptIn(const uint8_t ep_dci,        //
                              uint8_t* const buf,          //
                              const uint16_t max_len,      //
                              uint16_t* const actual_len,  //
                              const int64_t /*now_tick*/) override {
    *actual_len = 0;
    const uint8_t expected_dci = InterruptDci();
    if (!configured_ || ep_dci != expected_dci) {
      return IntrOutcome::kNak;
    }
    if (intr_ep_.halted) {
      return IntrOutcome::kStall;  // Until CLEAR_FEATURE(ENDPOINT_HALT)
    }
    if (!injected_outcomes_.empty()) {
      const IntrOutcome forced = injected_outcomes_.front();
      injected_outcomes_.pop_front();
      if (forced == IntrOutcome::kStall) {
        intr_ep_.halted = true;
      }
      return forced;
    }
    // Level-triggered like real hubs (USB 2.0 11.12.4) and QEMU's usb-hub:
    // the bitmap is returned on every poll while any hub or port change bit
    // is set.
    uint8_t bitmap[4] = {};
    bool any_change = false;
    if (hub_change_ != 0) {
      bitmap[0] |= 0x01u;  // Bit 0: hub status change
      any_change = true;
    }
    for (int p = 1; p <= config_.num_ports; ++p) {
      if (ports_[static_cast<size_t>(p - 1)].change != 0) {
        bitmap[p / 8] |= static_cast<uint8_t>(1u << (p % 8));
        any_change = true;
      }
    }
    if (!any_change) {
      return IntrOutcome::kNak;
    }
    // Nothing was latched or cleared since the last report: the host re-armed
    // the status-change IN before clearing what it was told (a storm on real
    // hubs).
    const bool redundant = change_epoch_ == last_report_epoch_;
    if (redundant && redundant_status_reports_ >= kMaxRedundantReports) {
      return IntrOutcome::kNak;  // Stop feeding a storm the test already fails
    }
    if (redundant) {
      ++redundant_status_reports_;
    }
    last_report_epoch_ = change_epoch_;
    ++intr_data_count_;
    const uint16_t bitmap_bytes =
        static_cast<uint16_t>((config_.num_ports + 8) / 8);
    const uint16_t copy_len = max_len < bitmap_bytes ? max_len : bitmap_bytes;
    std::memcpy(buf, bitmap, copy_len);
    *actual_len = copy_len;
    return IntrOutcome::kData;
  }

  void InjectInterruptError(const IntrOutcome outcome, const int count = 1) {
    for (int i = 0; i < count; ++i) {
      injected_outcomes_.push_back(outcome);
    }
  }

  // Sets wHubStatus (bit 0 local power lost, bit 1 over-current) and latches
  // the wHubChange bit of every status bit that changed.
  void SetHubStatus(const uint16_t status) {
    const uint16_t next = static_cast<uint16_t>(status & 0x0003u);
    const uint16_t changed = static_cast<uint16_t>(hub_status_ ^ next);
    hub_status_ = next;
    SetHubChange(static_cast<uint16_t>(hub_change_ | changed));
  }
  // Latches wHubChange bits (0x1 C_HUB_LOCAL_POWER, 0x2 C_HUB_OVER_CURRENT)
  // without a status change.
  void InjectHubChange(const uint16_t change) {
    SetHubChange(static_cast<uint16_t>(hub_change_ | (change & 0x0003u)));
  }
  // The next `count` SET_FEATURE(PORT_RESET) requests fail with `outcome`.
  void ArmPortResetFailures(
      const int count, const ControlOutcome outcome = ControlOutcome::kStall) {
    config_.fail_port_reset_attempts = failed_port_reset_ + count;
    config_.port_reset_error = outcome;
  }

  bool configured() const { return configured_; }
  uint8_t address() const override { return address_; }
  int num_ports() const { return config_.num_ports; }
  uint16_t hub_depth() const { return hub_depth_; }
  uint8_t alt_setting() const { return alt_setting_; }
  int bus_reset_count() const { return bus_reset_count_; }
  int set_config_count() const { return set_config_count_; }
  int set_interface_count() const { return set_interface_count_; }
  int set_hub_depth_count() const { return set_hub_depth_count_; }
  int get_hub_desc_count() const { return get_hub_desc_count_; }
  uint16_t hub_status() const { return hub_status_; }
  uint16_t hub_change() const { return hub_change_; }
  int get_hub_status_count() const { return get_hub_status_count_; }
  int clear_hub_feature_count() const { return clear_hub_feature_count_; }
  // GET_PORT_STATUS requests (including failed ones), never reset.
  int get_port_status_count() const { return get_port_status_count_; }
  int get_port_status_count(const int port_1based) const {
    return port_status_reads_[static_cast<size_t>(port_1based - 1)];
  }
  // SET_FEATURE(PORT_RESET) requests that reached the port since the last bus
  // reset of this hub.
  int port_reset_count(const int port_1based) const {
    return ports_[static_cast<size_t>(port_1based - 1)].reset_count;
  }
  // Status-change reports (kData completions), never reset.
  int intr_data_count() const { return intr_data_count_; }
  // Reports that repeated an uncleared change (see PollInterruptIn()).
  int redundant_status_reports() const { return redundant_status_reports_; }
  int class_requests_before_config() const {
    return class_requests_before_config_;
  }
  const HubPort& port(const int port_1based) const {
    return ports_[static_cast<size_t>(port_1based - 1)];
  }
  // The device side of the status-change endpoint.
  bool interrupt_halted() const { return intr_ep_.halted; }
  int clear_halt_count() const { return clear_halt_count_; }

 private:
  static constexpr int kMaxRedundantReports = 32;

  uint8_t InterruptDci() const {
    return static_cast<uint8_t>((config_.ep_address & 0x0Fu) * 2u + 1u);
  }

  // Every write of a change field goes through these, so that a report can
  // tell new information from a repeat (see PollInterruptIn()).
  void SetPortChange(HubPort& hp, const uint16_t change) {
    if (change != hp.change) {
      ++change_epoch_;
    }
    hp.change = change;
  }
  void LatchPortChange(HubPort& hp, const uint16_t bits) {
    SetPortChange(hp, static_cast<uint16_t>(hp.change | bits));
  }
  void SetHubChange(const uint16_t change) {
    if (change != hub_change_) {
      ++change_epoch_;
    }
    hub_change_ = change;
  }

  uint16_t PowerBit() const { return config_.speed >= 4 ? 0x0200u : 0x0100u; }

  void ApplyAttachedPortStatus(HubPort& hp) {
    hp.status = PowerBit() | 0x0001u;  // PORT_POWER | PORT_CONNECTION
    if (config_.speed >= 4) {
      hp.status |= 0x0002u;  // USB 3.0 links train directly to Enabled (U0)
    }
  }

  void CompletePortReset(HubPort& hp) {
    if (hp.device != nullptr) {
      hp.device->OnBusReset();
    }
    hp.status = PowerBit() | 0x0001u | 0x0002u;  // Power | Connected | Enabled
    if (config_.speed <= 3 && hp.device != nullptr) {
      if (hp.device->speed() == 2) {
        hp.status |= 0x0200u;  // Low-Speed
      } else if (hp.device->speed() == 3) {
        hp.status |= 0x0400u;  // High-Speed
      }
    }
    LatchPortChange(hp, 0x0010u);  // C_PORT_RESET
  }

  void ResetProtocolState() {
    address_ = 0;
    configured_ = false;
    alt_setting_ = 0;
    hub_depth_ = 0;
    intr_ep_ = SimInEndpointState{};
    for (int i = 0; i < kMaxSimHubPorts; ++i) {
      SimUsbDevice* const attached = ports_[static_cast<size_t>(i)].device;
      ports_[static_cast<size_t>(i)] = HubPort{};
      ports_[static_cast<size_t>(i)].device = attached;
    }
    hub_status_ = 0;
    hub_change_ = 0;
    ++change_epoch_;
    last_report_epoch_ = -1;
  }

  std::vector<uint8_t> BuildDeviceDescriptor() const {
    const uint16_t bcd_usb = config_.speed >= 4 ? 0x0300 : 0x0200;
    const uint8_t b_mps0 = config_.speed >= 4 ? 9 : ep0_max_packet_size();
    const uint8_t dev_proto =
        config_.device_protocol != 0xFF
            ? config_.device_protocol
            : static_cast<uint8_t>(
                  config_.speed >= 4
                      ? 3
                      : (config_.speed == 3 ? (config_.multi_tt ? 2 : 1) : 0));
    return {
        18,
        1,  // bDescriptorType = Device
        static_cast<uint8_t>(bcd_usb & 0xFFu),
        static_cast<uint8_t>(bcd_usb >> 8),
        9,  // bDeviceClass = 9 (Hub)
        0,  // bDeviceSubClass = 0
        dev_proto,
        b_mps0,
        static_cast<uint8_t>(config_.vendor_id & 0xFFu),
        static_cast<uint8_t>(config_.vendor_id >> 8),
        static_cast<uint8_t>(config_.product_id & 0xFFu),
        static_cast<uint8_t>(config_.product_id >> 8),
        0x00,
        0x01,
        1,
        2,
        3,
        1,  // bNumConfigurations = 1
    };
  }

  std::vector<uint8_t> BuildConfigDescriptor() const {
    const uint8_t iface_proto =
        config_.interface_protocol != 0xFF
            ? config_.interface_protocol
            : static_cast<uint8_t>(
                  (config_.speed == 3 && config_.multi_tt) ? 1 : 0);
    const uint16_t intr_mps =
        config_.intr_max_packet_size != 0
            ? config_.intr_max_packet_size
            : static_cast<uint16_t>((config_.num_ports + 8) / 8);
    const uint8_t b_interval =
        config_.intr_b_interval != 0
            ? config_.intr_b_interval
            : static_cast<uint8_t>(
                  config_.speed == 1 ? 255 : (config_.speed >= 4 ? 8 : 12));
    const uint8_t ep_attr = config_.speed >= 4 ? 0x13u : 0x03u;
    std::vector<uint8_t> full = {
        // Configuration Descriptor (9B); wTotalLength is patched below.
        9,
        2,
        0,
        0,
        1,
        1,
        0,
        0xE0,
        0,
        // Interface Descriptor (9B)
        9,
        4,
        0,
        0,
        1,
        9,  // bInterfaceClass = 9 (Hub)
        0,
        iface_proto,
        0,
        // Endpoint Descriptor (7B)
        7,
        5,
        config_.ep_address,
        ep_attr,
        static_cast<uint8_t>(intr_mps & 0xFFu),
        static_cast<uint8_t>((intr_mps >> 8) & 0xFFu),
        b_interval,
    };
    if (config_.speed >= 4) {
      const uint16_t bpi =
          static_cast<uint16_t>(intr_mps * (1u + config_.ss_max_burst));
      const uint8_t ss_comp[6] = {
          6,
          0x30,
          config_.ss_max_burst,
          0,
          static_cast<uint8_t>(bpi & 0xFFu),
          static_cast<uint8_t>((bpi >> 8) & 0xFFu),
      };
      full.insert(full.end(), std::begin(ss_comp), std::end(ss_comp));
    }
    if (config_.alt1_multi_tt) {
      const uint8_t alt1[] = {
          // Interface 0, alternate setting 1, bInterfaceProtocol 2 (multi-TT)
          9,
          4,
          0,
          1,
          1,
          9,
          0,
          2,
          0,
          7,
          5,
          config_.ep_address,
          ep_attr,
          static_cast<uint8_t>(intr_mps & 0xFFu),
          static_cast<uint8_t>((intr_mps >> 8) & 0xFFu),
          b_interval,
      };
      full.insert(full.end(), std::begin(alt1), std::end(alt1));
    }
    full[2] = static_cast<uint8_t>(full.size() & 0xFFu);
    full[3] = static_cast<uint8_t>(full.size() >> 8);
    return full;
  }

  std::vector<uint8_t> BuildHubDescriptor() const {
    if (config_.speed >= 4) {
      // 12-byte USB 3.0 SuperSpeed Hub Descriptor (type 0x2A).
      return {
          12,   0x2A, static_cast<uint8_t>(config_.num_ports),
          0x09, 0x00, config_.pwr_on_2_pwr_good,
          100,  4,    0x90,
          0x01, 0x00, 0x00,
      };
    }
    // USB 2.0 Hub Descriptor (type 0x29).
    const int mask_bytes = (config_.num_ports + 8) / 8;
    const uint8_t desc_len = static_cast<uint8_t>(7 + 2 * mask_bytes);
    const uint16_t chars =
        static_cast<uint16_t>(((config_.ttt & 0x3u) << 5) | 0x0009u);
    std::vector<uint8_t> desc = {
        desc_len,
        0x29,
        static_cast<uint8_t>(config_.num_ports),
        static_cast<uint8_t>(chars & 0xFFu),
        static_cast<uint8_t>((chars >> 8) & 0xFFu),
        config_.pwr_on_2_pwr_good,
        100,
    };
    for (int i = 0; i < mask_bytes; ++i) {
      desc.push_back(0x00u);  // DeviceRemovable
    }
    for (int i = 0; i < mask_bytes; ++i) {
      desc.push_back(0xFFu);  // PortPwrCtrlMask
    }
    return desc;
  }

  SimHubConfig config_;
  uint8_t address_ = 0;
  bool configured_ = false;
  uint8_t alt_setting_ = 0;
  uint16_t hub_depth_ = 0;
  uint16_t hub_status_ = 0;
  uint16_t hub_change_ = 0;
  // Bumped whenever a hub or port change field changes value; a report with
  // the same epoch as the previous one carries no new information.
  int64_t change_epoch_ = 0;
  int64_t last_report_epoch_ = -1;
  int redundant_status_reports_ = 0;
  int intr_data_count_ = 0;
  int bus_reset_count_ = 0;
  int set_config_count_ = 0;
  int set_interface_count_ = 0;
  int set_hub_depth_count_ = 0;
  int get_hub_desc_count_ = 0;
  int get_hub_status_count_ = 0;
  int clear_hub_feature_count_ = 0;
  int get_port_status_count_ = 0;
  int class_requests_before_config_ = 0;
  int failed_dev_desc_ = 0;
  int failed_hub_desc_ = 0;
  int failed_port_status_ = 0;
  int failed_set_feature_ = 0;
  int failed_clear_feature_ = 0;
  int failed_clear_hub_feature_ = 0;
  int failed_port_reset_ = 0;
  int clear_halt_count_ = 0;
  SimInEndpointState intr_ep_;
  std::array<int, kMaxSimHubPorts> port_status_reads_ = {};
  std::array<HubPort, kMaxSimHubPorts> ports_ = {};
  std::deque<IntrOutcome> injected_outcomes_;
};

}  // namespace protos::xhci_sim

#endif  // PROTOS_USER_USBKBD_XHCI_SIM_USB_H_
