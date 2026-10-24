#ifndef PROTOS_USER_USBKBD_USB_DESCRIPTORS_H_
#define PROTOS_USER_USBKBD_USB_DESCRIPTORS_H_

// Freestanding USB 2.0/3.0 standard and HID class descriptor constants and
// defensive parser for locating a USB HID Boot Protocol Keyboard interface
// and its Interrupt IN endpoint.

#include <cstdint>

#include "status_or.h"

namespace protos::usb {

// USB Descriptor Types (USB 2.0 Table 9-5, USB 3.2 Table 9-6, HID 1.11 7.1).
constexpr uint8_t kUsbDescDevice = 1;
constexpr uint8_t kUsbDescConfiguration = 2;
constexpr uint8_t kUsbDescString = 3;
constexpr uint8_t kUsbDescInterface = 4;
constexpr uint8_t kUsbDescEndpoint = 5;
constexpr uint8_t kUsbDescHid = 0x21;
constexpr uint8_t kUsbDescHidReport = 0x22;
constexpr uint8_t kUsbDescHub = 0x29;
constexpr uint8_t kUsbDescSsHub = 0x2A;
constexpr uint8_t kUsbDescSsEpCompanion = 0x30;

// USB Control Setup request types and request codes.
constexpr uint8_t kUsbReqTypeStdDevOut = 0x00;
constexpr uint8_t kUsbReqTypeStdDevIn = 0x80;
constexpr uint8_t kUsbReqTypeStdEndpointOut = 0x02;
constexpr uint8_t kUsbReqTypeClassDevOut = 0x20;
constexpr uint8_t kUsbReqTypeClassIfaceOut = 0x21;
constexpr uint8_t kUsbReqTypeClassPortOut = 0x23;
constexpr uint8_t kUsbReqTypeClassDevIn = 0xA0;
constexpr uint8_t kUsbReqTypeClassPortIn = 0xA3;
constexpr uint8_t kUsbReqGetStatus = 0;
constexpr uint8_t kUsbReqClearFeature = 1;
constexpr uint8_t kUsbReqSetFeature = 3;
constexpr uint8_t kUsbReqGetDescriptor = 6;
constexpr uint8_t kUsbReqSetConfiguration = 9;
constexpr uint8_t kHidReqSetIdle = 0x0A;
constexpr uint8_t kHidReqSetProtocol = 0x0B;
constexpr uint8_t kHubReqSetHubDepth = 0x0C;

// Standard feature selector ENDPOINT_HALT (USB 2.0 Table 9-6): with
// CLEAR_FEATURE it clears an endpoint's Halt feature and resets its data
// toggle (SuperSpeed: sequence number), halted or not (USB 2.0 9.4.5).
constexpr uint16_t kUsbFeatEndpointHalt = 0;

// USB Hub Class Port Feature Selectors (USB 2.0 Table 11-17, USB 3.2 Table
// 10-7).
constexpr uint16_t kHubPortFeatConnection = 0;
constexpr uint16_t kHubPortFeatEnable = 1;
constexpr uint16_t kHubPortFeatSuspend = 2;
constexpr uint16_t kHubPortFeatOverCurrent = 3;
constexpr uint16_t kHubPortFeatReset = 4;
constexpr uint16_t kHubPortFeatPower = 8;
constexpr uint16_t kHubPortFeatLowSpeed = 9;
constexpr uint16_t kHubPortFeatCConnection = 16;
constexpr uint16_t kHubPortFeatCEnable = 17;
constexpr uint16_t kHubPortFeatCSuspend = 18;
constexpr uint16_t kHubPortFeatCOverCurrent = 19;
constexpr uint16_t kHubPortFeatCReset = 20;
constexpr uint16_t kHubPortFeatCLinkState = 25;
constexpr uint16_t kHubPortFeatCConfigError = 26;
constexpr uint16_t kHubPortFeatBhPortReset = 28;
constexpr uint16_t kHubPortFeatCBhPortReset = 29;

// USB Hub wPortStatus bits (USB 2.0 Table 11-21, USB 3.2 Table 10-10).
constexpr uint16_t kHubPortStatConnection = 1u << 0;
constexpr uint16_t kHubPortStatEnable = 1u << 1;
constexpr uint16_t kHubPortStatSuspend = 1u << 2;
constexpr uint16_t kHubPortStatOverCurrent = 1u << 3;
constexpr uint16_t kHubPortStatReset = 1u << 4;
constexpr uint16_t kHubPortStatUsb2Power = 1u << 8;
constexpr uint16_t kHubPortStatUsb2LowSpeed = 1u << 9;
constexpr uint16_t kHubPortStatUsb2HighSpeed = 1u << 10;
constexpr uint16_t kHubPortStatUsb3LinkStateMask = 0xFu << 5;
constexpr uint16_t kHubPortStatUsb3Power = 1u << 9;
constexpr uint16_t kHubPortStatUsb3SpeedMask = 0x7u << 10;

// USB Hub wPortChange bits (USB 2.0 Table 11-22, USB 3.2 Table 10-11).
constexpr uint16_t kHubPortChangeConnection = 1u << 0;
constexpr uint16_t kHubPortChangeEnable = 1u << 1;
constexpr uint16_t kHubPortChangeSuspend = 1u << 2;
constexpr uint16_t kHubPortChangeOverCurrent = 1u << 3;
constexpr uint16_t kHubPortChangeReset = 1u << 4;
constexpr uint16_t kHubPortChangeBhReset = 1u << 5;
constexpr uint16_t kHubPortChangeLinkState = 1u << 6;
constexpr uint16_t kHubPortChangeConfigError = 1u << 7;

// USB Hub Class hub feature selectors and wHubStatus / wHubChange bits (USB
// 2.0 11.24.2.1, 11.24.2.6, Tables 11-17, 11-19, 11-20; USB 3.2 10.16.2).
constexpr uint16_t kHubFeatCHubLocalPower = 0;
constexpr uint16_t kHubFeatCHubOverCurrent = 1;
constexpr uint16_t kHubStatLocalPower = 1u << 0;
constexpr uint16_t kHubStatOverCurrent = 1u << 1;
constexpr uint16_t kHubChangeLocalPower = 1u << 0;
constexpr uint16_t kHubChangeOverCurrent = 1u << 1;

// USB Class, SubClass, Protocol, and Endpoint attribute constants.
constexpr uint8_t kUsbClassHid = 3;
constexpr uint8_t kUsbClassHub = 9;
constexpr uint8_t kHubProtocolFullSpeed = 0;
constexpr uint8_t kHubProtocolHighSpeedSingleTt = 1;
constexpr uint8_t kHubProtocolHighSpeedMultiTt = 2;
constexpr uint8_t kHubProtocolSuperSpeed = 3;
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

inline StatusOr<uint8_t> ParseDeviceDescriptorPrefix(const uint8_t* const buf,
                                                     const int len) {
  if (buf == nullptr || len < 8 || buf[0] < 18 || buf[1] != kUsbDescDevice) {
    return kStatusError;
  }
  return buf[7];
}

struct DeviceDescriptorInfo {
  uint8_t device_class = 0;
  uint16_t vendor_id = 0;
  uint16_t product_id = 0;
  uint8_t num_configs = 0;
};

inline StatusOr<DeviceDescriptorInfo> ParseDeviceDescriptorFull(
    const uint8_t* const buf, const int len) {
  if (buf == nullptr || len < 18 || buf[0] < 18 || buf[1] != kUsbDescDevice) {
    return kStatusError;
  }
  return DeviceDescriptorInfo{
      .device_class = buf[4],
      .vendor_id = ReadLe16(buf + 8),
      .product_id = ReadLe16(buf + 10),
      .num_configs = buf[17],
  };
}

struct ConfigDescriptorHeader {
  uint16_t total_length = 0;
  uint8_t config_value = 0;
};

inline StatusOr<ConfigDescriptorHeader> ParseConfigDescriptorHeader(
    const uint8_t* const buf, const int len) {
  if (buf == nullptr || len < 9 || buf[0] < 9 ||
      buf[1] != kUsbDescConfiguration) {
    return kStatusError;
  }
  const uint16_t total = ReadLe16(buf + 2);
  const uint8_t config_val = buf[5];
  if (total < 9 || config_val == 0) {
    return kStatusError;
  }
  return ConfigDescriptorHeader{
      .total_length = total,
      .config_value = config_val,
  };
}

// Walks a Configuration Descriptor blob of `len` bytes and extracts the first
// HID Boot Keyboard interface (`bAlternateSetting == 0`, class 3, subclass 1,
// protocol 1) and its first Interrupt IN endpoint (plus optional SuperSpeed
// Endpoint Companion `bMaxBurst`). Never reads past `min(len, wTotalLength)`.
inline StatusOr<BootKeyboardMatch> FindBootKeyboard(const uint8_t* const buf,
                                                    const int len) {
  const StatusOr<ConfigDescriptorHeader> hdr =
      ParseConfigDescriptorHeader(buf, len);
  if (!hdr.ok()) {
    return kStatusError;
  }
  const int total = len < static_cast<int>(hdr->total_length)
                        ? len
                        : static_cast<int>(hdr->total_length);
  int off = buf[0];
  bool in_kbd_iface = false;
  bool found_ep = false;
  bool just_matched_ep = false;
  uint8_t kbd_iface_num = 0;
  BootKeyboardMatch match{};

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
            match = BootKeyboardMatch{};
            match.config_value = hdr->config_value;
            match.interface_number = kbd_iface_num;
            match.ep_address = addr;
            match.ep_dci = static_cast<uint8_t>(ep_num * 2u + 1u);
            match.max_packet_size = mps;
            match.max_burst = static_cast<uint8_t>((raw_mps >> 11) & 0x03u);
            match.b_interval = buf[off + 6];
            found_ep = true;
            just_matched_ep = true;
          }
        }
      }
    } else if (dtype == kUsbDescSsEpCompanion && just_matched_ep) {
      if (dlen >= 6) {
        match.max_burst = buf[off + 2];
      }
      just_matched_ep = false;
    }
    off += dlen;
  }
  if (!found_ep) {
    return kStatusError;
  }
  return match;
}

struct HubMatch {
  uint8_t config_value = 0;
  uint8_t interface_number = 0;
  uint8_t interface_protocol = 0;
  uint8_t ep_address = 0;
  uint8_t ep_dci = 0;
  uint16_t max_packet_size = 0;
  uint8_t max_burst = 0;
  uint8_t b_interval = 0;
};

struct HubDescriptorInfo {
  uint8_t desc_type = 0;
  uint8_t num_ports = 0;
  uint16_t characteristics = 0;
  uint8_t ttt = 0;
  uint8_t pwr_on_to_pwr_good = 0;
};

// Walks a Configuration Descriptor blob of `len` bytes and extracts the
// first USB Hub interface (`bAlternateSetting == 0`, `bInterfaceClass == 9`)
// and its Interrupt IN status-change endpoint (plus optional SuperSpeed
// Endpoint Companion `bMaxBurst`). Never reads past `min(len, wTotalLength)`.
inline StatusOr<HubMatch> FindUsbHub(const uint8_t* const buf, const int len) {
  const StatusOr<ConfigDescriptorHeader> hdr =
      ParseConfigDescriptorHeader(buf, len);
  if (!hdr.ok()) {
    return kStatusError;
  }
  const int total = len < static_cast<int>(hdr->total_length)
                        ? len
                        : static_cast<int>(hdr->total_length);
  int off = buf[0];
  bool in_hub_iface = false;
  bool found_ep = false;
  bool just_matched_ep = false;
  uint8_t hub_iface_num = 0;
  uint8_t hub_iface_proto = 0;
  HubMatch match{};

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
      if (dlen >= 9 && buf[off + 3] == 0 && buf[off + 5] == kUsbClassHub) {
        in_hub_iface = true;
        hub_iface_num = buf[off + 2];
        hub_iface_proto = buf[off + 7];
      } else {
        in_hub_iface = false;
      }
    } else if (dtype == kUsbDescEndpoint) {
      just_matched_ep = false;
      if (in_hub_iface && !found_ep && dlen >= 7) {
        const uint8_t addr = buf[off + 2];
        const uint8_t attr = buf[off + 3];
        const uint8_t ep_num = static_cast<uint8_t>(addr & 0x0Fu);
        if ((addr & kUsbEpDirInMask) != 0 && ep_num != 0 &&
            (attr & kUsbEpAttrTransferMask) == kUsbEpAttrInterrupt) {
          const uint16_t raw_mps = ReadLe16(buf + off + 4);
          const uint16_t mps = static_cast<uint16_t>(raw_mps & 0x07FFu);
          if (mps > 0) {
            match = HubMatch{};
            match.config_value = hdr->config_value;
            match.interface_number = hub_iface_num;
            match.interface_protocol = hub_iface_proto;
            match.ep_address = addr;
            match.ep_dci = static_cast<uint8_t>(ep_num * 2u + 1u);
            match.max_packet_size = mps;
            match.max_burst = static_cast<uint8_t>((raw_mps >> 11) & 0x03u);
            match.b_interval = buf[off + 6];
            found_ep = true;
            just_matched_ep = true;
          }
        }
      }
    } else if (dtype == kUsbDescSsEpCompanion && just_matched_ep) {
      if (dlen >= 6) {
        match.max_burst = buf[off + 2];
      }
      just_matched_ep = false;
    }
    off += dlen;
  }
  if (!found_ep) {
    return kStatusError;
  }
  return match;
}

// Parses a USB 2.0 Hub Descriptor (`0x29`, min 7 bytes) or USB 3.0
// SuperSpeed Hub Descriptor (`0x2A`, min 12 bytes). If `expected_type` is
// non-zero, requires `buf[1] == expected_type`. Rejects `bNbrPorts == 0` or
// `bNbrPorts > 16`.
inline StatusOr<HubDescriptorInfo> ParseHubDescriptor(
    const uint8_t* const buf,  //
    const int len,             //
    const uint8_t expected_type) {
  if (buf == nullptr || len < 7) {
    return kStatusError;
  }
  const uint8_t dtype = buf[1];
  if (expected_type != 0 && dtype != expected_type) {
    return kStatusError;
  }
  if (dtype != kUsbDescHub && dtype != kUsbDescSsHub) {
    return kStatusError;
  }
  const int min_len = (dtype == kUsbDescSsHub) ? 12 : 7;
  if (len < min_len || buf[0] < min_len) {
    return kStatusError;
  }
  const uint8_t num_ports = buf[2];
  if (num_ports < 1 || num_ports > 16) {
    return kStatusError;
  }
  const uint16_t chars = ReadLe16(buf + 3);
  return HubDescriptorInfo{
      .desc_type = dtype,
      .num_ports = num_ports,
      .characteristics = chars,
      .ttt = static_cast<uint8_t>((chars >> 5) & 0x3u),
      .pwr_on_to_pwr_good = buf[5],
  };
}

}  // namespace protos::usb

#endif  // PROTOS_USER_USBKBD_USB_DESCRIPTORS_H_
