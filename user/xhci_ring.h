#ifndef PROTOS_USER_XHCI_RING_H_
#define PROTOS_USER_XHCI_RING_H_

// Freestanding xHCI Transfer Request Block (TRB) definitions, single-segment
// ProducerRing (Command, EP0, Interrupt IN), and single-segment EventRing
// consumer (xHCI 1.2 sections 4.9 and 6.4).

#include <cstdint>

namespace protos::usb {

struct Trb {
  uint32_t dw0 = 0;
  uint32_t dw1 = 0;
  uint32_t dw2 = 0;
  uint32_t dw3 = 0;
};
static_assert(sizeof(Trb) == 16);

struct ErstEntry {
  uint64_t ring_segment_base = 0;
  uint32_t ring_segment_size = 0;
  uint32_t rsvd = 0;
};
static_assert(sizeof(ErstEntry) == 16);

// TRB Type IDs (xHCI 6.4.6 Table 6-91).
constexpr uint8_t kTrbNormal = 1;
constexpr uint8_t kTrbSetupStage = 2;
constexpr uint8_t kTrbDataStage = 3;
constexpr uint8_t kTrbStatusStage = 4;
constexpr uint8_t kTrbLink = 6;
constexpr uint8_t kTrbEnableSlot = 9;
constexpr uint8_t kTrbDisableSlot = 10;
constexpr uint8_t kTrbAddressDevice = 11;
constexpr uint8_t kTrbConfigureEndpoint = 12;
constexpr uint8_t kTrbEvaluateContext = 13;
constexpr uint8_t kTrbResetEndpoint = 14;
constexpr uint8_t kTrbStopEndpoint = 15;
constexpr uint8_t kTrbSetTrDequeue = 16;
constexpr uint8_t kTrbNoOpCmd = 23;
constexpr uint8_t kTrbTransferEvent = 32;
constexpr uint8_t kTrbCommandCompletion = 33;
constexpr uint8_t kTrbPortStatusChange = 34;
constexpr uint8_t kTrbHostControllerEvent = 37;

// TRB dw3 control bits and shifts (xHCI 6.4.1 - 6.4.4).
constexpr uint32_t kTrbCycle = 1u << 0;
constexpr uint32_t kTrbToggleCycle = 1u << 1;
constexpr uint32_t kTrbIsp = 1u << 2;
constexpr uint32_t kTrbChain = 1u << 4;
constexpr uint32_t kTrbIoc = 1u << 5;
constexpr uint32_t kTrbIdt = 1u << 6;
constexpr uint32_t kTrbBsr = 1u << 9;
constexpr uint32_t kTrbTsp = 1u << 9;
constexpr uint32_t kTrbDeconfigure = 1u << 9;
constexpr int kTrbTypeShift = 10;
constexpr uint32_t kTrbTypeMask = 0x3Fu << 10;
constexpr uint32_t kTrbDirIn = 1u << 16;
constexpr int kTrbTrtShift = 16;
constexpr uint32_t kTrtNoData = 0u;
constexpr uint32_t kTrtOutData = 2u;
constexpr uint32_t kTrtInData = 3u;
constexpr int kTrbEndpointIdShift = 16;
constexpr int kTrbSlotTypeShift = 16;
constexpr int kTrbSlotIdShift = 24;

// TRB Completion Codes (xHCI 6.4.5 Table 6-90).
constexpr uint8_t kCcInvalid = 0;
constexpr uint8_t kCcSuccess = 1;
constexpr uint8_t kCcDataBufferError = 2;
constexpr uint8_t kCcBabbleDetected = 3;
constexpr uint8_t kCcUsbTransactionError = 4;
constexpr uint8_t kCcTrbError = 5;
constexpr uint8_t kCcStallError = 6;
constexpr uint8_t kCcResourceError = 7;
constexpr uint8_t kCcBandwidthError = 8;
constexpr uint8_t kCcNoSlotsAvailable = 9;
constexpr uint8_t kCcSlotNotEnabled = 11;
constexpr uint8_t kCcEndpointNotEnabled = 12;
constexpr uint8_t kCcShortPacket = 13;
constexpr uint8_t kCcRingUnderrun = 14;
constexpr uint8_t kCcRingOverrun = 15;
constexpr uint8_t kCcParameterError = 17;
constexpr uint8_t kCcContextStateError = 19;
constexpr uint8_t kCcEventRingFull = 21;
constexpr uint8_t kCcCommandRingStopped = 24;
constexpr uint8_t kCcCommandAborted = 25;
constexpr uint8_t kCcStopped = 26;
constexpr uint8_t kCcStoppedLengthInvalid = 27;

// Event TRB field extractors.
constexpr uint8_t TrbType(const Trb& trb) {
  return static_cast<uint8_t>((trb.dw3 >> kTrbTypeShift) & 0x3Fu);
}

constexpr uint8_t TrbCompletionCode(const Trb& trb) {
  return static_cast<uint8_t>((trb.dw2 >> 24) & 0xFFu);
}

constexpr uint32_t TrbTransferResidual(const Trb& trb) {
  return trb.dw2 & 0x00FFFFFFu;
}

constexpr uint8_t TrbSlotId(const Trb& trb) {
  return static_cast<uint8_t>((trb.dw3 >> kTrbSlotIdShift) & 0xFFu);
}

constexpr uint8_t TrbEndpointId(const Trb& trb) {
  return static_cast<uint8_t>((trb.dw3 >> kTrbEndpointIdShift) & 0x1Fu);
}

constexpr uint64_t TrbPointer(const Trb& trb) {
  return (static_cast<uint64_t>(trb.dw1) << 32) | trb.dw0;
}

constexpr int TrbPortId(const Trb& trb) {
  return static_cast<int>((trb.dw0 >> 24) & 0xFFu);
}

// Command TRB builders (cycle bit left 0; ProducerRing sets it on commit).
constexpr Trb MakeEnableSlotTrb(const uint8_t slot_type = 0) {
  return Trb{
      0,
      0,
      0,
      (static_cast<uint32_t>(kTrbEnableSlot) << kTrbTypeShift) |
          (static_cast<uint32_t>(slot_type & 0x1Fu) << kTrbSlotTypeShift),
  };
}

constexpr Trb MakeDisableSlotTrb(const uint8_t slot_id) {
  return Trb{
      0,
      0,
      0,
      (static_cast<uint32_t>(kTrbDisableSlot) << kTrbTypeShift) |
          (static_cast<uint32_t>(slot_id) << kTrbSlotIdShift),
  };
}

constexpr Trb MakeAddressDeviceTrb(const uint64_t input_ctx_phys,  //
                                   const uint8_t slot_id,          //
                                   const bool bsr = false) {
  return Trb{
      static_cast<uint32_t>(input_ctx_phys & ~0xFull),
      static_cast<uint32_t>(input_ctx_phys >> 32),
      0,
      (bsr ? kTrbBsr : 0u) |
          (static_cast<uint32_t>(kTrbAddressDevice) << kTrbTypeShift) |
          (static_cast<uint32_t>(slot_id) << kTrbSlotIdShift),
  };
}

constexpr Trb MakeConfigureEndpointTrb(const uint64_t input_ctx_phys,  //
                                       const uint8_t slot_id,          //
                                       const bool deconfigure = false) {
  return Trb{
      static_cast<uint32_t>(input_ctx_phys & ~0xFull),
      static_cast<uint32_t>(input_ctx_phys >> 32),
      0,
      (deconfigure ? kTrbDeconfigure : 0u) |
          (static_cast<uint32_t>(kTrbConfigureEndpoint) << kTrbTypeShift) |
          (static_cast<uint32_t>(slot_id) << kTrbSlotIdShift),
  };
}

constexpr Trb MakeEvaluateContextTrb(const uint64_t input_ctx_phys,
                                     const uint8_t slot_id) {
  return Trb{
      static_cast<uint32_t>(input_ctx_phys & ~0xFull),
      static_cast<uint32_t>(input_ctx_phys >> 32),
      0,
      (static_cast<uint32_t>(kTrbEvaluateContext) << kTrbTypeShift) |
          (static_cast<uint32_t>(slot_id) << kTrbSlotIdShift),
  };
}

constexpr Trb MakeResetEndpointTrb(const uint8_t slot_id,  //
                                   const uint8_t dci,      //
                                   const bool tsp = false) {
  return Trb{
      0,
      0,
      0,
      (tsp ? kTrbTsp : 0u) |
          (static_cast<uint32_t>(kTrbResetEndpoint) << kTrbTypeShift) |
          (static_cast<uint32_t>(dci & 0x1Fu) << kTrbEndpointIdShift) |
          (static_cast<uint32_t>(slot_id) << kTrbSlotIdShift),
  };
}

constexpr Trb MakeStopEndpointTrb(const uint8_t slot_id, const uint8_t dci) {
  return Trb{
      0,
      0,
      0,
      (static_cast<uint32_t>(kTrbStopEndpoint) << kTrbTypeShift) |
          (static_cast<uint32_t>(dci & 0x1Fu) << kTrbEndpointIdShift) |
          (static_cast<uint32_t>(slot_id) << kTrbSlotIdShift),
  };
}

constexpr Trb MakeSetTrDequeueTrb(const uint64_t new_deq_phys_with_dcs,  //
                                  const uint8_t slot_id,                 //
                                  const uint8_t dci) {
  return Trb{
      static_cast<uint32_t>(new_deq_phys_with_dcs & ~0xEull),
      static_cast<uint32_t>(new_deq_phys_with_dcs >> 32),
      0,
      (static_cast<uint32_t>(kTrbSetTrDequeue) << kTrbTypeShift) |
          (static_cast<uint32_t>(dci & 0x1Fu) << kTrbEndpointIdShift) |
          (static_cast<uint32_t>(slot_id) << kTrbSlotIdShift),
  };
}

// Transfer TRB builders (xHCI 6.4.1.1 - 6.4.1.2).
constexpr Trb MakeSetupStageTrb(const uint8_t bm_request_type,  //
                                const uint8_t b_request,        //
                                const uint16_t w_value,         //
                                const uint16_t w_index,         //
                                const uint16_t w_length,        //
                                const uint32_t trt) {
  return Trb{
      static_cast<uint32_t>(bm_request_type) |
          (static_cast<uint32_t>(b_request) << 8) |
          (static_cast<uint32_t>(w_value) << 16),
      static_cast<uint32_t>(w_index) | (static_cast<uint32_t>(w_length) << 16),
      8u,
      kTrbIdt | (static_cast<uint32_t>(kTrbSetupStage) << kTrbTypeShift) |
          ((trt & 0x3u) << kTrbTrtShift),
  };
}

constexpr Trb MakeDataStageTrb(const uint64_t buf_phys,  //
                               const uint32_t len,       //
                               const bool dir_in) {
  return Trb{
      static_cast<uint32_t>(buf_phys),
      static_cast<uint32_t>(buf_phys >> 32),
      len & 0x1FFFFu,
      kTrbIsp | (static_cast<uint32_t>(kTrbDataStage) << kTrbTypeShift) |
          (dir_in ? kTrbDirIn : 0u),
  };
}

constexpr Trb MakeStatusStageTrb(const bool dir_in) {
  return Trb{
      0,
      0,
      0,
      kTrbIoc | (static_cast<uint32_t>(kTrbStatusStage) << kTrbTypeShift) |
          (dir_in ? kTrbDirIn : 0u),
  };
}

constexpr Trb MakeNormalTrb(const uint64_t buf_phys,  //
                            const uint32_t len,       //
                            const bool ioc = true,    //
                            const bool isp = true) {
  return Trb{
      static_cast<uint32_t>(buf_phys),
      static_cast<uint32_t>(buf_phys >> 32),
      len & 0x1FFFFu,
      (isp ? kTrbIsp : 0u) | (ioc ? kTrbIoc : 0u) |
          (static_cast<uint32_t>(kTrbNormal) << kTrbTypeShift),
  };
}

inline void DmaBarrier() { asm volatile("" ::: "memory"); }

namespace ring_internal {

inline void WriteVolatile32(uint32_t* const dst, const uint32_t val) {
  *reinterpret_cast<volatile uint32_t*>(dst) = val;
}

inline uint32_t ReadVolatile32(const uint32_t* const src) {
  return *reinterpret_cast<const volatile uint32_t*>(src);
}

inline void WriteTrbWithCycle(Trb* const dst,  //
                              const Trb& src,  //
                              const uint32_t cycle) {
  WriteVolatile32(&dst->dw0, src.dw0);
  WriteVolatile32(&dst->dw1, src.dw1);
  WriteVolatile32(&dst->dw2, src.dw2);
  DmaBarrier();
  WriteVolatile32(&dst->dw3, (src.dw3 & ~kTrbCycle) | (cycle & kTrbCycle));
  DmaBarrier();
}

}  // namespace ring_internal

// Single-segment producer ring (Command Ring, EP0 Transfer Ring, Interrupt IN
// Transfer Ring). Slot `Capacity - 1` holds the Link TRB pointing back to
// `phys_` with Toggle Cycle set.
template <int Capacity = 256>
class ProducerRing {
  static_assert(Capacity >= 16 && (Capacity & (Capacity - 1)) == 0,
                "ProducerRing capacity must be a power of two >= 16");

 public:
  static constexpr int kCapacity = Capacity;
  static constexpr int kUsableSlots = Capacity - 1;

  void Init(Trb* const base, const uint64_t phys) {
    base_ = base;
    phys_ = phys;
    enqueue_ = 0;
    cycle_ = 1u;
    for (int i = 0; i < Capacity; ++i) {
      ring_internal::WriteVolatile32(&base_[i].dw0, 0u);
      ring_internal::WriteVolatile32(&base_[i].dw1, 0u);
      ring_internal::WriteVolatile32(&base_[i].dw2, 0u);
      ring_internal::WriteVolatile32(&base_[i].dw3, 0u);
    }
    // Format the Link TRB at the end of the segment with Cycle = 0 (opposite
    // to initial PCS = 1) so the consumer stops if it reaches the end early.
    Trb& link = base_[Capacity - 1];
    ring_internal::WriteVolatile32(&link.dw0, static_cast<uint32_t>(phys_));
    ring_internal::WriteVolatile32(&link.dw1,
                                   static_cast<uint32_t>(phys_ >> 32));
    ring_internal::WriteVolatile32(&link.dw2, 0u);
    ring_internal::WriteVolatile32(
        &link.dw3,
        (static_cast<uint32_t>(kTrbLink) << kTrbTypeShift) | kTrbToggleCycle);
    DmaBarrier();
  }

  // Enqueues one TRB and returns its guest physical address.
  uint64_t Enqueue(const Trb& trb) {
    const int idx = enqueue_;
    const uint64_t trb_phys = phys_ + static_cast<uint64_t>(idx) * sizeof(Trb);
    ring_internal::WriteTrbWithCycle(&base_[idx], trb, cycle_);
    AdvanceAndCrossLinkIfNeeded();
    return trb_phys;
  }

  // Enqueues a multi-TRB TD (e.g. Setup + optional Data + Status) using the
  // xHCI 4.11.7.1 deferred first-TRB cycle-bit handoff: the first TRB is
  // written with an inverted cycle bit until all subsequent TRBs (and any
  // Link TRB crossed in between) are in place.
  void EnqueueTd(const Trb* const trbs,  //
                 const int count,        //
                 uint64_t* const out_phys = nullptr) {
    if (count <= 0) {
      return;
    }
    const int first_idx = enqueue_;
    const uint32_t first_cycle = cycle_;
    if (out_phys != nullptr) {
      out_phys[0] = phys_ + static_cast<uint64_t>(first_idx) * sizeof(Trb);
    }
    ring_internal::WriteTrbWithCycle(&base_[first_idx], trbs[0],
                                     first_cycle ^ 1u);
    AdvanceAndCrossLinkIfNeeded();
    for (int i = 1; i < count; ++i) {
      const int idx = enqueue_;
      if (out_phys != nullptr) {
        out_phys[i] = phys_ + static_cast<uint64_t>(idx) * sizeof(Trb);
      }
      ring_internal::WriteTrbWithCycle(&base_[idx], trbs[i], cycle_);
      AdvanceAndCrossLinkIfNeeded();
    }
    DmaBarrier();
    const uint32_t dw3 = (trbs[0].dw3 & ~kTrbCycle) | (first_cycle & kTrbCycle);
    ring_internal::WriteVolatile32(&base_[first_idx].dw3, dw3);
    DmaBarrier();
  }

  uint64_t phys() const { return phys_; }
  int enqueue_index() const { return enqueue_; }
  uint32_t cycle() const { return cycle_; }
  uint64_t enqueue_phys() const {
    return phys_ + static_cast<uint64_t>(enqueue_) * sizeof(Trb);
  }
  const Trb& trb(const int index) const { return base_[index]; }

 private:
  void AdvanceAndCrossLinkIfNeeded() {
    ++enqueue_;
    if (enqueue_ == Capacity - 1) {
      Trb& link = base_[Capacity - 1];
      ring_internal::WriteVolatile32(&link.dw0, static_cast<uint32_t>(phys_));
      ring_internal::WriteVolatile32(&link.dw1,
                                     static_cast<uint32_t>(phys_ >> 32));
      ring_internal::WriteVolatile32(&link.dw2, 0u);
      DmaBarrier();
      ring_internal::WriteVolatile32(
          &link.dw3, (static_cast<uint32_t>(kTrbLink) << kTrbTypeShift) |
                         kTrbToggleCycle | (cycle_ & kTrbCycle));
      DmaBarrier();
      enqueue_ = 0;
      cycle_ ^= 1u;
    }
  }

  Trb* base_ = nullptr;
  uint64_t phys_ = 0;
  int enqueue_ = 0;
  uint32_t cycle_ = 1u;
};

// Single-segment Event Ring consumer (Interrupter 0, xHCI 4.9.4).
template <int Capacity = 256>
class EventRing {
  static_assert(Capacity >= 16 && (Capacity & (Capacity - 1)) == 0,
                "EventRing capacity must be a power of two >= 16");

 public:
  static constexpr int kCapacity = Capacity;

  void Init(Trb* const base,          //
            const uint64_t seg_phys,  //
            ErstEntry* const erst,    //
            const uint64_t erst_phys) {
    base_ = base;
    seg_phys_ = seg_phys;
    erst_ = erst;
    erst_phys_ = erst_phys;
    dequeue_ = 0;
    cycle_ = 1u;
    for (int i = 0; i < Capacity; ++i) {
      ring_internal::WriteVolatile32(&base_[i].dw0, 0u);
      ring_internal::WriteVolatile32(&base_[i].dw1, 0u);
      ring_internal::WriteVolatile32(&base_[i].dw2, 0u);
      ring_internal::WriteVolatile32(&base_[i].dw3, 0u);
    }
    erst_[0].ring_segment_base = seg_phys_;
    erst_[0].ring_segment_size = static_cast<uint32_t>(Capacity);
    erst_[0].rsvd = 0u;
    DmaBarrier();
  }

  bool HasEvent() const {
    const uint32_t dw3 = ring_internal::ReadVolatile32(&base_[dequeue_].dw3);
    return (dw3 & kTrbCycle) == cycle_;
  }

  bool Pop(Trb* const out) {
    const Trb& slot = base_[dequeue_];
    const uint32_t dw3 = ring_internal::ReadVolatile32(&slot.dw3);
    if ((dw3 & kTrbCycle) != cycle_) {
      return false;
    }
    DmaBarrier();
    out->dw0 = ring_internal::ReadVolatile32(&slot.dw0);
    out->dw1 = ring_internal::ReadVolatile32(&slot.dw1);
    out->dw2 = ring_internal::ReadVolatile32(&slot.dw2);
    out->dw3 = dw3;
    ++dequeue_;
    if (dequeue_ == Capacity) {
      dequeue_ = 0;
      cycle_ ^= 1u;
    }
    return true;
  }

  uint64_t seg_phys() const { return seg_phys_; }
  uint64_t erst_phys() const { return erst_phys_; }
  int dequeue_index() const { return dequeue_; }
  uint32_t cycle() const { return cycle_; }
  uint64_t dequeue_phys() const {
    return seg_phys_ + static_cast<uint64_t>(dequeue_) * sizeof(Trb);
  }

 private:
  Trb* base_ = nullptr;
  uint64_t seg_phys_ = 0;
  ErstEntry* erst_ = nullptr;
  uint64_t erst_phys_ = 0;
  int dequeue_ = 0;
  uint32_t cycle_ = 1u;
};

}  // namespace protos::usb

#endif  // PROTOS_USER_XHCI_RING_H_
