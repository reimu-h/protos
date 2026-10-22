#ifndef PROTOS_USER_USB_DESCRIPTORS_H_
#define PROTOS_USER_USB_DESCRIPTORS_H_

// Freestanding USB 2.0/3.0 standard and HID class descriptor constants and
// defensive parser for locating a USB HID Boot Protocol Keyboard interface
// and its Interrupt IN endpoint.

#include <cstdint>

namespace protos::usb {

// USB Descriptor Types (USB 2.0 Table 9-5, USB 3.2 Table 9-6, HID 1.11 7.1).
constexpr uint8_t kUsbDescDevice = 1;
constexpr uint8_t kUsbDescConfiguration = 2;
constexpr uint8_t kUsbDescString = 3;
constexpr uint8_t kUsbDescInterface = 4;
constexpr uint8_t kUsbDescEndpoint = 5;
constexpr uint8_t kUsbDescHid = 0x21;
constexpr uint8_t kUsbDescHidReport = 0x22;
constexpr uint8_t kUsbDescSsEpCompanion = 0x30;

// USB Control Setup request types and request codes.
constexpr uint8_t kUsbReqTypeStdDevOut = 0x00;
constexpr uint8_t kUsbReqTypeStdDevIn = 0x80;
constexpr uint8_t kUsbReqTypeClassIfaceOut = 0x21;
constexpr uint8_t kUsbReqGetDescriptor = 6;
constexpr uint8_t kUsbReqSetConfiguration = 9;
constexpr uint8_t kHidReqSetIdle = 0x0A;
constexpr uint8_t kHidReqSetProtocol = 0x0B;

// USB Class, SubClass, Protocol, and Endpoint attribute constants.
constexpr uint8_t kUsbClassHid = 3;
constexpr uint8_t kUsbClassHub = 9;
constexpr uint8_t kHidSubclassBoot = 1;
constexpr uint8_t kHidProtocolKeyboard = 1;
constexpr uint8_t kHidProtocolMouse = 2;
constexpr uint8_t kUsbEpAttrTransferMask = 0x03;
constexpr uint8_t kUsbEpAttrInterrupt = 0x03;
constexpr uint8_t kUsbEpDirInMask = 0x80;

struct BootKeyboardMatch {
  uint8_t config_value = 0;
  uint8_t interface_number = 0;
  uint8_t ep_address = 0;
  uint8_t ep_dci = 0;
  uint16_t max_packet_size = 0;
  uint8_t max_burst = 0;
  uint8_t b_interval = 0;
};

constexpr uint16_t ReadLe16(const uint8_t* const p) {
  return static_cast<uint16_t>(p[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8);
}

inline bool ParseDeviceDescriptorPrefix(const uint8_t* const buf,  //
                                        const int len,             //
                                        uint8_t* const out_max_packet_size0) {
  if (buf == nullptr || len < 8 || buf[0] < 18 || buf[1] != kUsbDescDevice) {
    return false;
  }
  *out_max_packet_size0 = buf[7];
  return true;
}

inline bool ParseDeviceDescriptorFull(const uint8_t* const buf,         //
                                      const int len,                    //
                                      uint8_t* const out_device_class,  //
                                      uint8_t* const out_num_configs) {
  if (buf == nullptr || len < 18 || buf[0] < 18 || buf[1] != kUsbDescDevice) {
    return false;
  }
  *out_device_class = buf[4];
  *out_num_configs = buf[17];
  return true;
}

inline bool ParseConfigDescriptorHeader(const uint8_t* const buf,          //
                                        const int len,                     //
                                        uint16_t* const out_total_length,  //
                                        uint8_t* const out_config_value) {
  if (buf == nullptr || len < 9 || buf[0] < 9 ||
      buf[1] != kUsbDescConfiguration) {
    return false;
  }
  const uint16_t total = ReadLe16(buf + 2);
  const uint8_t config_val = buf[5];
  if (total < 9 || config_val == 0) {
    return false;
  }
  *out_total_length = total;
  *out_config_value = config_val;
  return true;
}

// Walks a Configuration Descriptor blob of `len` bytes and extracts the first
// HID Boot Keyboard interface (`bAlternateSetting == 0`, class 3, subclass 1,
// protocol 1) and its first Interrupt IN endpoint (plus optional SuperSpeed
// Endpoint Companion `bMaxBurst`). Never reads past `min(len, wTotalLength)`.
inline bool FindBootKeyboard(const uint8_t* const buf,  //
                             const int len,             //
                             BootKeyboardMatch* const out) {
  uint16_t w_total_length = 0;
  uint8_t config_value = 0;
  if (out == nullptr ||
      !ParseConfigDescriptorHeader(buf, len, &w_total_length, &config_value)) {
    return false;
  }
  const int total =
      len < static_cast<int>(w_total_length) ? len : w_total_length;
  int off = buf[0];
  bool in_kbd_iface = false;
  bool found_ep = false;
  bool just_matched_ep = false;
  uint8_t kbd_iface_num = 0;

  while (off + 2 <= total) {
    const int dlen = buf[off];
    const uint8_t dtype = buf[off + 1];
    if (dlen < 2 || off + dlen > total) {
      break;
    }
    if (dtype == kUsbDescInterface) {
      if (found_ep) {
        break;
      }
      just_matched_ep = false;
      if (dlen >= 9 && buf[off + 3] == 0 && buf[off + 5] == kUsbClassHid &&
          buf[off + 6] == kHidSubclassBoot &&
          buf[off + 7] == kHidProtocolKeyboard) {
        in_kbd_iface = true;
        kbd_iface_num = buf[off + 2];
      } else {
        in_kbd_iface = false;
      }
    } else if (dtype == kUsbDescEndpoint) {
      just_matched_ep = false;
      if (in_kbd_iface && !found_ep && dlen >= 7) {
        const uint8_t addr = buf[off + 2];
        const uint8_t attr = buf[off + 3];
        const uint8_t ep_num = static_cast<uint8_t>(addr & 0x0Fu);
        if ((addr & kUsbEpDirInMask) != 0 && ep_num != 0 &&
            (attr & kUsbEpAttrTransferMask) == kUsbEpAttrInterrupt) {
          const uint16_t raw_mps = ReadLe16(buf + off + 4);
          const uint16_t mps = static_cast<uint16_t>(raw_mps & 0x07FFu);
          if (mps > 0) {
            *out = BootKeyboardMatch{};
            out->config_value = config_value;
            out->interface_number = kbd_iface_num;
            out->ep_address = addr;
            out->ep_dci = static_cast<uint8_t>(ep_num * 2u + 1u);
            out->max_packet_size = mps;
            out->max_burst = static_cast<uint8_t>((raw_mps >> 11) & 0x03u);
            out->b_interval = buf[off + 6];
            found_ep = true;
            just_matched_ep = true;
          }
        }
      }
    } else if (dtype == kUsbDescSsEpCompanion && just_matched_ep) {
      if (dlen >= 6) {
        out->max_burst = buf[off + 2];
      }
      just_matched_ep = false;
    }
    off += dlen;
  }
  return found_ep;
}

}  // namespace protos::usb

#endif  // PROTOS_USER_USB_DESCRIPTORS_H_
