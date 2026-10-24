#ifndef PROTOS_USER_USBKBD_XHCI_CONTEXT_H_
#define PROTOS_USER_USBKBD_XHCI_CONTEXT_H_

// Freestanding xHCI Slot, Endpoint, and Input Control Context structures and
// CSZ-aware (32-byte vs 64-byte) accessors and parameter encoders (xHCI 1.2
// section 6.2).

#include <cstdint>

namespace protos::usb {

// Endpoint Type values (xHCI 6.2.3 Table 6-9).
constexpr uint8_t kEpTypeIsochOut = 1;
constexpr uint8_t kEpTypeBulkOut = 2;
constexpr uint8_t kEpTypeInterruptOut = 3;
constexpr uint8_t kEpTypeControlBiDir = 4;
constexpr uint8_t kEpTypeIsochIn = 5;
constexpr uint8_t kEpTypeBulkIn = 6;
constexpr uint8_t kEpTypeInterruptIn = 7;

// Slot State values (xHCI 6.2.2 Table 6-7).
constexpr uint8_t kSlotStateDisabledOrEnabled = 0;
constexpr uint8_t kSlotStateDefault = 1;
constexpr uint8_t kSlotStateAddressed = 2;
constexpr uint8_t kSlotStateConfigured = 3;

// Endpoint State values (xHCI 6.2.3 Table 6-8).
constexpr uint8_t kEpStateDisabled = 0;
constexpr uint8_t kEpStateRunning = 1;
constexpr uint8_t kEpStateHalted = 2;
constexpr uint8_t kEpStateStopped = 3;
constexpr uint8_t kEpStateError = 4;

struct InputControlContext {
  uint32_t drop_flags = 0;
  uint32_t add_flags = 0;
  uint32_t rsvd[5] = {};
  uint32_t config_info = 0;
};
static_assert(sizeof(InputControlContext) == 32);

struct SlotContext {
  uint32_t dw0 = 0;
  uint32_t dw1 = 0;
  uint32_t dw2 = 0;
  uint32_t dw3 = 0;
  uint32_t rsvd[4] = {};

  constexpr uint32_t route_string() const { return dw0 & 0xFFFFFu; }
  constexpr void SetRouteString(const uint32_t route) {
    dw0 = (dw0 & ~0xFFFFFu) | (route & 0xFFFFFu);
  }
  constexpr int speed() const { return static_cast<int>((dw0 >> 20) & 0xFu); }
  constexpr bool mtt() const { return ((dw0 >> 25) & 1u) != 0; }
  constexpr void SetMtt(const bool enabled) {
    if (enabled) {
      dw0 |= (1u << 25);
    } else {
      dw0 &= ~(1u << 25);
    }
  }
  constexpr bool is_hub() const { return ((dw0 >> 26) & 1u) != 0; }
  constexpr void SetHub(const bool hub) {
    if (hub) {
      dw0 |= (1u << 26);
    } else {
      dw0 &= ~(1u << 26);
    }
  }
  constexpr int context_entries() const {
    return static_cast<int>((dw0 >> 27) & 0x1Fu);
  }
  constexpr int root_hub_port() const {
    return static_cast<int>((dw1 >> 16) & 0xFFu);
  }
  constexpr int num_ports() const {
    return static_cast<int>((dw1 >> 24) & 0xFFu);
  }
  constexpr void SetNumPorts(const int count) {
    dw1 =
        (dw1 & ~(0xFFu << 24)) | ((static_cast<uint32_t>(count) & 0xFFu) << 24);
  }
  constexpr uint8_t parent_hub_slot_id() const {
    return static_cast<uint8_t>(dw2 & 0xFFu);
  }
  constexpr uint8_t parent_port_number() const {
    return static_cast<uint8_t>((dw2 >> 8) & 0xFFu);
  }
  constexpr void SetTtInfo(const uint8_t hub_slot_id,
                           const uint8_t port_number) {
    dw2 = (dw2 & ~0xFFFFu) | static_cast<uint32_t>(hub_slot_id) |
          (static_cast<uint32_t>(port_number) << 8);
  }
  constexpr uint8_t ttt() const {
    return static_cast<uint8_t>((dw2 >> 16) & 0x3u);
  }
  constexpr void SetTtThinkTime(const uint8_t think_time) {
    dw2 = (dw2 & ~(0x3u << 16)) |
          ((static_cast<uint32_t>(think_time) & 0x3u) << 16);
  }
};
static_assert(sizeof(SlotContext) == 32);

struct EndpointContext {
  uint32_t dw0 = 0;
  uint32_t dw1 = 0;
  uint32_t tr_dequeue_low = 0;
  uint32_t tr_dequeue_high = 0;
  uint32_t dw4 = 0;
  uint32_t rsvd[3] = {};
};
static_assert(sizeof(EndpointContext) == 32);

inline void ZeroContextBuffer(uint8_t* const base, const int bytes) {
  for (int i = 0; i < bytes; ++i) {
    *reinterpret_cast<volatile uint8_t*>(base + i) = 0;
  }
}

// CSZ-aware Input Context accessors (33 context entries: index 0 = ICC,
// index 1 = Slot Context, index dci+1 = Endpoint Context for DCI 1..31).
inline InputControlContext* InputCtrlCtx(uint8_t* const input_ctx) {
  return reinterpret_cast<InputControlContext*>(input_ctx);
}

inline const InputControlContext* InputCtrlCtx(const uint8_t* const input_ctx) {
  return reinterpret_cast<const InputControlContext*>(input_ctx);
}

inline SlotContext* InputSlotCtx(uint8_t* const input_ctx,
                                 const int context_size) {
  return reinterpret_cast<SlotContext*>(input_ctx + context_size);
}

inline const SlotContext* InputSlotCtx(const uint8_t* const input_ctx,
                                       const int context_size) {
  return reinterpret_cast<const SlotContext*>(input_ctx + context_size);
}

inline EndpointContext* InputEpCtx(uint8_t* const input_ctx,  //
                                   const int context_size,    //
                                   const int dci) {
  return reinterpret_cast<EndpointContext*>(input_ctx +
                                            (dci + 1) * context_size);
}

inline const EndpointContext* InputEpCtx(const uint8_t* const input_ctx,  //
                                         const int context_size,          //
                                         const int dci) {
  return reinterpret_cast<const EndpointContext*>(input_ctx +
                                                  (dci + 1) * context_size);
}

// CSZ-aware Output Device Context accessors (32 context entries: index 0 =
// Slot Context, index dci = Endpoint Context for DCI 1..31).
inline SlotContext* OutputSlotCtx(uint8_t* const dev_ctx,
                                  const int context_size) {
  (void)context_size;
  return reinterpret_cast<SlotContext*>(dev_ctx);
}

inline const SlotContext* OutputSlotCtx(const uint8_t* const dev_ctx,
                                        const int context_size) {
  (void)context_size;
  return reinterpret_cast<const SlotContext*>(dev_ctx);
}

inline EndpointContext* OutputEpCtx(uint8_t* const dev_ctx,  //
                                    const int context_size,  //
                                    const int dci) {
  return reinterpret_cast<EndpointContext*>(dev_ctx + dci * context_size);
}

inline const EndpointContext* OutputEpCtx(const uint8_t* const dev_ctx,  //
                                          const int context_size,        //
                                          const int dci) {
  return reinterpret_cast<const EndpointContext*>(dev_ctx + dci * context_size);
}

// Slot Context field extractors.
constexpr uint8_t SlotContextState(const SlotContext& slot) {
  return static_cast<uint8_t>((slot.dw3 >> 27) & 0x1Fu);
}

constexpr uint8_t SlotContextAddress(const SlotContext& slot) {
  return static_cast<uint8_t>(slot.dw3 & 0xFFu);
}

constexpr int SlotContextEntries(const SlotContext& slot) {
  return static_cast<int>((slot.dw0 >> 27) & 0x1Fu);
}

constexpr int SlotContextSpeed(const SlotContext& slot) {
  return static_cast<int>((slot.dw0 >> 20) & 0xFu);
}

constexpr int SlotContextRootHubPort(const SlotContext& slot) {
  return static_cast<int>((slot.dw1 >> 16) & 0xFFu);
}

constexpr uint32_t SlotContextRouteString(const SlotContext& slot) {
  return slot.route_string();
}

constexpr bool SlotContextMtt(const SlotContext& slot) { return slot.mtt(); }

constexpr bool SlotContextHub(const SlotContext& slot) { return slot.is_hub(); }

constexpr bool SlotContextIsHub(const SlotContext& slot) {
  return slot.is_hub();
}

constexpr int SlotContextNumPorts(const SlotContext& slot) {
  return slot.num_ports();
}

constexpr uint8_t SlotContextParentHubSlotId(const SlotContext& slot) {
  return slot.parent_hub_slot_id();
}

constexpr uint8_t SlotContextParentPortNumber(const SlotContext& slot) {
  return slot.parent_port_number();
}

constexpr uint8_t SlotContextTtt(const SlotContext& slot) { return slot.ttt(); }

// Appends a 1-based downstream port number (clamped to 15 per xHCI 6.2.2)
// at tier `hub_depth` (0..4) to a 20-bit xHCI Route String.
constexpr uint32_t AppendRouteStringPort(const uint32_t parent_route_string,
                                         const int hub_depth,
                                         const int downstream_port) {
  if (hub_depth < 0 || hub_depth >= 5 || downstream_port <= 0) {
    return parent_route_string & 0xFFFFFu;
  }
  const uint32_t nibble =
      static_cast<uint32_t>(downstream_port > 15 ? 15 : downstream_port);
  const uint32_t shift = static_cast<uint32_t>(hub_depth * 4);
  return ((parent_route_string & 0xFFFFFu) & ~(0xFu << shift)) |
         (nibble << shift);
}

// Endpoint Context field extractors.
constexpr uint8_t EpContextState(const EndpointContext& ep) {
  return static_cast<uint8_t>(ep.dw0 & 0x7u);
}

constexpr uint8_t EpContextInterval(const EndpointContext& ep) {
  return static_cast<uint8_t>((ep.dw0 >> 16) & 0xFFu);
}

constexpr uint8_t EpContextCerr(const EndpointContext& ep) {
  return static_cast<uint8_t>((ep.dw1 >> 1) & 0x3u);
}

constexpr uint8_t EpContextType(const EndpointContext& ep) {
  return static_cast<uint8_t>((ep.dw1 >> 3) & 0x7u);
}

constexpr uint8_t EpContextMaxBurst(const EndpointContext& ep) {
  return static_cast<uint8_t>((ep.dw1 >> 8) & 0xFFu);
}

constexpr uint16_t EpContextMaxPacketSize(const EndpointContext& ep) {
  return static_cast<uint16_t>((ep.dw1 >> 16) & 0xFFFFu);
}

constexpr uint64_t EpContextDequeuePtr(const EndpointContext& ep) {
  return (static_cast<uint64_t>(ep.tr_dequeue_high) << 32) |
         (ep.tr_dequeue_low & ~0xFull);
}

constexpr uint32_t EpContextDcs(const EndpointContext& ep) {
  return ep.tr_dequeue_low & 1u;
}

constexpr uint16_t EpContextAvgTrbLength(const EndpointContext& ep) {
  return static_cast<uint16_t>(ep.dw4 & 0xFFFFu);
}

constexpr uint32_t EpContextMaxEsitPayload(const EndpointContext& ep) {
  return ((ep.dw0 >> 24) & 0xFFu) << 16 | ((ep.dw4 >> 16) & 0xFFFFu);
}

// Converts USB bEndpointAddress to xHCI Device Context Index (DCI 1..31).
constexpr int EndpointDci(const uint8_t b_endpoint_address) {
  const int ep_num = static_cast<int>(b_endpoint_address & 0x0Fu);
  if (ep_num == 0) {
    return 1;
  }
  const int dir_in = (b_endpoint_address & 0x80u) != 0 ? 1 : 0;
  return ep_num * 2 + dir_in;
}

// Default EP0 Max Packet Size before reading the Device Descriptor (xHCI 4.3).
// Speed IDs: 1 = Full, 2 = Low, 3 = High, 4 = Super, 5 = SuperPlus.
constexpr uint16_t InitialEp0MaxPacketSize(const int port_speed) {
  if (port_speed >= 4) {
    return 512;
  }
  if (port_speed == 3) {
    return 64;
  }
  return 8;
}

// Decodes the actual EP0 Max Packet Size from DeviceDescriptor.bMaxPacketSize0.
constexpr uint16_t DecodeEp0MaxPacketSize(const int port_speed,
                                          const uint8_t b_max_packet_size0) {
  if (port_speed >= 4) {
    return 512;
  }
  if (port_speed == 3) {
    return 64;
  }
  if (port_speed == 2) {
    return 8;
  }
  if (b_max_packet_size0 == 8 || b_max_packet_size0 == 16 ||
      b_max_packet_size0 == 32 || b_max_packet_size0 == 64) {
    return b_max_packet_size0;
  }
  return 8;
}

// Encodes EndpointContext.Interval (xHCI 6.2.3.6 Table 6-12) for an Interrupt
// endpoint. For LS/FS (speeds 1 and 2), converts bInterval (1..255 ms frames)
// to floor(log2(bInterval * 8)) clamped to [3, 10]. For HS/SS/SSP (speeds >=
// 3), converts bInterval (1..16) to clamp(bInterval, 1, 16) - 1 in [0, 15].
constexpr uint8_t EncodeInterruptInterval(const int port_speed,
                                          const uint8_t b_interval) {
  if (port_speed == 1 || port_speed == 2) {
    const uint32_t frames = b_interval < 1u ? 1u : b_interval;
    const uint32_t uframes = frames * 8u;
    uint8_t exponent = 3;
    for (uint8_t e = 3; e <= 10; ++e) {
      if ((1u << e) <= uframes) {
        exponent = e;
      }
    }
    return exponent;
  }
  const uint8_t clamped =
      b_interval < 1u ? 1u : (b_interval > 16u ? 16u : b_interval);
  return static_cast<uint8_t>(clamped - 1u);
}

// Formats InputSlotCtx and InputEpCtx(1) for Address Device (ICC Add = 0x3).
inline void FormatAddressDeviceInputContext(
    uint8_t* const input_ctx,               //
    const int context_size,                 //
    const int port_number,                  //
    const int port_speed,                   //
    const uint64_t ep0_ring_phys,           //
    const uint32_t ep0_cycle = 1u,          //
    const uint32_t route_string = 0u,       //
    const uint8_t parent_hub_slot_id = 0u,  //
    const uint8_t parent_port_number = 0u,  //
    const bool mtt = false) {
  ZeroContextBuffer(input_ctx, 33 * context_size);
  InputControlContext* const icc = InputCtrlCtx(input_ctx);
  icc->drop_flags = 0u;
  icc->add_flags = 0x3u;  // A0 (Slot) | A1 (EP0)

  SlotContext* const slot = InputSlotCtx(input_ctx, context_size);
  slot->dw0 = (route_string & 0xFFFFFu) |
              ((static_cast<uint32_t>(port_speed) & 0xFu) << 20) |
              (mtt ? (1u << 25) : 0u) | (1u << 27);
  slot->dw1 = (static_cast<uint32_t>(port_number) & 0xFFu) << 16;
  slot->SetTtInfo(parent_hub_slot_id, parent_port_number);

  EndpointContext* const ep0 = InputEpCtx(input_ctx, context_size, 1);
  const uint16_t mps = InitialEp0MaxPacketSize(port_speed);
  ep0->dw1 = (3u << 1) | (static_cast<uint32_t>(kEpTypeControlBiDir) << 3) |
             (static_cast<uint32_t>(mps) << 16);
  ep0->tr_dequeue_low =
      static_cast<uint32_t>(ep0_ring_phys & ~0xFull) | (ep0_cycle & 1u);
  ep0->tr_dequeue_high = static_cast<uint32_t>(ep0_ring_phys >> 32);
  ep0->dw4 = 8u;  // Average TRB Length = 8 for Control endpoints
}

// Formats InputContext for Evaluate Context when a Full-Speed device's
// bMaxPacketSize0 differs from the initial 8-byte MPS (ICC Add = 0x2).
inline void FormatEvaluateEp0MpsInputContext(uint8_t* const input_ctx,      //
                                             const int context_size,        //
                                             const uint8_t* const dev_ctx,  //
                                             const uint16_t new_mps) {
  ZeroContextBuffer(input_ctx, 33 * context_size);
  InputControlContext* const icc = InputCtrlCtx(input_ctx);
  icc->drop_flags = 0u;
  icc->add_flags = 0x2u;  // A1 (EP0 only)

  *InputSlotCtx(input_ctx, context_size) =
      *OutputSlotCtx(dev_ctx, context_size);
  // Output-only fields are cleared in an Input Context: Slot State and USB
  // Device Address (xHCI 6.2.2) and the Endpoint State (6.2.3).
  InputSlotCtx(input_ctx, context_size)->dw3 = 0u;
  EndpointContext* const ep0 = InputEpCtx(input_ctx, context_size, 1);
  *ep0 = *OutputEpCtx(dev_ctx, context_size, 1);
  ep0->dw0 &= ~0x7u;
  ep0->dw1 = (ep0->dw1 & 0x0000FFFFu) | (static_cast<uint32_t>(new_mps) << 16);
}

// Formats InputContext for Configure Endpoint of a Boot Keyboard or USB Hub
// Interrupt IN endpoint (ICC Add = (1 << 0) | (1 << dci)).
inline void FormatConfigureInterruptEndpointInputContext(
    uint8_t* const input_ctx,        //
    const int context_size,          //
    const uint8_t* const dev_ctx,    //
    const int dci,                   //
    const int port_speed,            //
    const uint16_t max_packet_size,  //
    const uint8_t max_burst,         //
    const uint8_t b_interval,        //
    const uint64_t intr_ring_phys,   //
    const uint32_t intr_cycle = 1u,  //
    const bool is_hub = false,       //
    const int num_ports = 0,         //
    const uint8_t ttt = 0u,          //
    const bool mtt = false) {
  ZeroContextBuffer(input_ctx, 33 * context_size);
  InputControlContext* const icc = InputCtrlCtx(input_ctx);
  icc->drop_flags = 0u;
  icc->add_flags = 1u | (1u << dci);

  SlotContext* const slot = InputSlotCtx(input_ctx, context_size);
  *slot = *OutputSlotCtx(dev_ctx, context_size);
  slot->dw0 =
      (slot->dw0 & ~(0x1Fu << 27)) | (static_cast<uint32_t>(dci & 0x1F) << 27);
  if (is_hub) {
    slot->SetHub(true);
    slot->SetMtt(slot->mtt() || mtt);
    slot->SetNumPorts(num_ports);
    slot->SetTtThinkTime(ttt);
  }
  slot->dw3 = 0u;  // Output-only fields cleared in Input Slot Context

  EndpointContext* const ep = InputEpCtx(input_ctx, context_size, dci);
  const uint8_t interval = EncodeInterruptInterval(port_speed, b_interval);
  const uint32_t esit =
      static_cast<uint32_t>(max_packet_size) * (1u + max_burst);
  ep->dw0 =
      (static_cast<uint32_t>(interval) << 16) | (((esit >> 16) & 0xFFu) << 24);
  ep->dw1 = (3u << 1) | (static_cast<uint32_t>(kEpTypeInterruptIn) << 3) |
            (static_cast<uint32_t>(max_burst) << 8) |
            (static_cast<uint32_t>(max_packet_size) << 16);
  ep->tr_dequeue_low =
      static_cast<uint32_t>(intr_ring_phys & ~0xFull) | (intr_cycle & 1u);
  ep->tr_dequeue_high = static_cast<uint32_t>(intr_ring_phys >> 32);
  ep->dw4 = (esit & 0xFFFFu) | ((esit & 0xFFFFu) << 16);
}

}  // namespace protos::usb

#endif  // PROTOS_USER_USBKBD_XHCI_CONTEXT_H_
