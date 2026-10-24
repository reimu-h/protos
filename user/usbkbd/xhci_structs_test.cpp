// Unit tests for user/usbkbd/xhci_ring.h, user/usbkbd/xhci_context.h, and
// user/usbkbd/usb_descriptors.h: TRB builders, ProducerRing and EventRing wrap
// and cycle-bit invariants, CSZ=32/64 context layout and parameter encoders,
// and defensive USB configuration descriptor parsing.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "user/usbkbd/usb_descriptors.h"
#include "user/usbkbd/xhci_context.h"
#include "user/usbkbd/xhci_regs.h"
#include "user/usbkbd/xhci_ring.h"

namespace protos::usb {
namespace {

namespace t = ::testing;

TEST(XhciRingTest, TrbBuildersEncodeExpectedFieldsWithoutCycleBit) {
  const Trb enable = MakeEnableSlotTrb(3);
  EXPECT_THAT(TrbType(enable), t::Eq(kTrbEnableSlot));
  EXPECT_THAT((enable.dw3 >> kTrbSlotTypeShift) & 0x1Fu, t::Eq(3u));
  EXPECT_THAT(enable.dw3 & kTrbCycle, t::Eq(0u));

  const Trb disable = MakeDisableSlotTrb(5);
  EXPECT_THAT(TrbType(disable), t::Eq(kTrbDisableSlot));
  EXPECT_THAT(TrbSlotId(disable), t::Eq(5u));

  const Trb addr = MakeAddressDeviceTrb(0x1'2345'6000ULL, 2, true);
  EXPECT_THAT(TrbType(addr), t::Eq(kTrbAddressDevice));
  EXPECT_THAT(TrbPointer(addr), t::Eq(0x1'2345'6000ULL));
  EXPECT_THAT(TrbSlotId(addr), t::Eq(2u));
  EXPECT_THAT(addr.dw3 & kTrbBsr, t::Ne(0u));

  const Trb cfg = MakeConfigureEndpointTrb(0x8000'1000ULL, 4, false);
  EXPECT_THAT(TrbType(cfg), t::Eq(kTrbConfigureEndpoint));
  EXPECT_THAT(TrbPointer(cfg), t::Eq(0x8000'1000ULL));
  EXPECT_THAT(TrbSlotId(cfg), t::Eq(4u));

  const Trb eval = MakeEvaluateContextTrb(0x9000'2000ULL, 7);
  EXPECT_THAT(TrbType(eval), t::Eq(kTrbEvaluateContext));
  EXPECT_THAT(TrbPointer(eval), t::Eq(0x9000'2000ULL));
  EXPECT_THAT(TrbSlotId(eval), t::Eq(7u));

  const Trb reset_ep = MakeResetEndpointTrb(3, 1, false);
  EXPECT_THAT(TrbType(reset_ep), t::Eq(kTrbResetEndpoint));
  EXPECT_THAT(TrbSlotId(reset_ep), t::Eq(3u));
  EXPECT_THAT(TrbEndpointId(reset_ep), t::Eq(1u));

  const Trb stop_ep = MakeStopEndpointTrb(3, 5);
  EXPECT_THAT(TrbType(stop_ep), t::Eq(kTrbStopEndpoint));
  EXPECT_THAT(TrbSlotId(stop_ep), t::Eq(3u));
  EXPECT_THAT(TrbEndpointId(stop_ep), t::Eq(5u));

  const Trb set_deq = MakeSetTrDequeueTrb(0xABCD'0011ULL, 3, 1);
  EXPECT_THAT(TrbType(set_deq), t::Eq(kTrbSetTrDequeue));
  EXPECT_THAT(TrbPointer(set_deq), t::Eq(0xABCD'0011ULL));
  EXPECT_THAT(TrbSlotId(set_deq), t::Eq(3u));
  EXPECT_THAT(TrbEndpointId(set_deq), t::Eq(1u));

  const Trb setup =
      MakeSetupStageTrb(0x80, kUsbReqGetDescriptor, 0x0100, 0, 18, kTrtInData);
  EXPECT_THAT(TrbType(setup), t::Eq(kTrbSetupStage));
  EXPECT_THAT(setup.dw0, t::Eq(0x01000680u));
  EXPECT_THAT(setup.dw1, t::Eq(0x00120000u));
  EXPECT_THAT(setup.dw2, t::Eq(8u));
  EXPECT_THAT(setup.dw3 & kTrbIdt, t::Ne(0u));
  EXPECT_THAT((setup.dw3 >> kTrbTrtShift) & 0x3u, t::Eq(kTrtInData));

  const Trb data = MakeDataStageTrb(0x4000'0800ULL, 18, true);
  EXPECT_THAT(TrbType(data), t::Eq(kTrbDataStage));
  EXPECT_THAT(TrbPointer(data), t::Eq(0x4000'0800ULL));
  EXPECT_THAT(data.dw2, t::Eq(18u));
  EXPECT_THAT(data.dw3 & kTrbDirIn, t::Ne(0u));
  EXPECT_THAT(data.dw3 & kTrbIsp, t::Ne(0u));

  const Trb status = MakeStatusStageTrb(false);
  EXPECT_THAT(TrbType(status), t::Eq(kTrbStatusStage));
  EXPECT_THAT(status.dw3 & kTrbIoc, t::Ne(0u));
  EXPECT_THAT(status.dw3 & kTrbDirIn, t::Eq(0u));

  const Trb normal = MakeNormalTrb(0x5000'0000ULL, 8);
  EXPECT_THAT(TrbType(normal), t::Eq(kTrbNormal));
  EXPECT_THAT(TrbPointer(normal), t::Eq(0x5000'0000ULL));
  EXPECT_THAT(normal.dw2, t::Eq(8u));
  EXPECT_THAT(normal.dw3 & (kTrbIoc | kTrbIsp), t::Eq(kTrbIoc | kTrbIsp));

  const Trb noop = MakeNoOpCommandTrb();
  EXPECT_THAT(TrbType(noop), t::Eq(kTrbNoOpCmd));
  EXPECT_THAT(noop.dw3 & kTrbCycle, t::Eq(0u));
}

TEST(XhciRingTest, ProducerRingWrapsAndTogglesCycleAcrossMultipleLaps) {
  alignas(64) Trb storage[16] = {};
  constexpr uint64_t kRingPhys = 0x2000'0000ULL;
  ProducerRing<16> ring;
  ring.Init(storage, kRingPhys);

  EXPECT_THAT(ring.enqueue_index(), t::Eq(0));
  EXPECT_THAT(ring.cycle(), t::Eq(1u));
  // Link TRB at index 15 starts with TC=1 and Cycle=0.
  EXPECT_THAT(TrbType(storage[15]), t::Eq(kTrbLink));
  EXPECT_THAT(TrbPointer(storage[15]), t::Eq(kRingPhys));
  EXPECT_THAT(storage[15].dw3 & kTrbToggleCycle, t::Ne(0u));
  EXPECT_THAT(storage[15].dw3 & kTrbCycle, t::Eq(0u));

  // Enqueue 14 TRBs (indices 0..13): still on lap 0 (cycle 1).
  for (int i = 0; i < 14; ++i) {
    const uint64_t phys = ring.Enqueue(MakeEnableSlotTrb(i));
    EXPECT_THAT(phys, t::Eq(kRingPhys + static_cast<uint64_t>(i) * 16));
    EXPECT_THAT(storage[i].dw3 & kTrbCycle, t::Eq(1u));
  }
  EXPECT_THAT(ring.enqueue_index(), t::Eq(14));
  EXPECT_THAT(ring.cycle(), t::Eq(1u));
  EXPECT_THAT(storage[15].dw3 & kTrbCycle, t::Eq(0u));

  // The 15th TRB lands at index 14 and immediately crosses the Link TRB at
  // index 15, flipping Link's cycle to 1, wrapping enqueue to 0, and flipping
  // producer cycle to 0.
  const uint64_t phys14 = ring.Enqueue(MakeEnableSlotTrb(14));
  EXPECT_THAT(phys14, t::Eq(kRingPhys + 14 * 16));
  EXPECT_THAT(storage[14].dw3 & kTrbCycle, t::Eq(1u));
  EXPECT_THAT(storage[15].dw3 & kTrbToggleCycle, t::Ne(0u));
  EXPECT_THAT(storage[15].dw3 & kTrbCycle, t::Eq(1u));
  EXPECT_THAT(ring.enqueue_index(), t::Eq(0));
  EXPECT_THAT(ring.cycle(), t::Eq(0u));

  // Lap 1: enqueue 15 more TRBs with cycle 0; after the 15th, Link TRB at 15
  // flips cycle to 0 and producer cycle returns to 1.
  for (int i = 0; i < 15; ++i) {
    ring.Enqueue(MakeDisableSlotTrb(static_cast<uint8_t>(i + 1)));
    EXPECT_THAT(storage[i].dw3 & kTrbCycle, t::Eq(0u));
  }
  EXPECT_THAT(storage[15].dw3 & kTrbCycle, t::Eq(0u));
  EXPECT_THAT(ring.enqueue_index(), t::Eq(0));
  EXPECT_THAT(ring.cycle(), t::Eq(1u));

  // Rewrite replaces an existing TRB in-place while preserving its cycle bit.
  ring.Rewrite(kRingPhys + 3 * 16, MakeNoOpCommandTrb());
  EXPECT_THAT(TrbType(storage[3]), t::Eq(kTrbNoOpCmd));
  EXPECT_THAT(storage[3].dw3 & kTrbCycle, t::Eq(0u));
}

TEST(XhciRingTest, EnqueueTdCommitsAcrossLinkTrbWithCorrectCycles) {
  alignas(64) Trb storage[16] = {};
  constexpr uint64_t kRingPhys = 0x3000'0000ULL;
  ProducerRing<16> ring;
  ring.Init(storage, kRingPhys);

  // Advance to index 13 (one slot before the last usable slot 14).
  for (int i = 0; i < 13; ++i) {
    ring.Enqueue(MakeEnableSlotTrb(0));
  }
  ASSERT_THAT(ring.enqueue_index(), t::Eq(13));
  ASSERT_THAT(ring.cycle(), t::Eq(1u));

  // Enqueue a 3-TRB control TD:
  // - Setup lands at 13 (cycle 1)
  // - Data lands at 14 (cycle 1), crossing Link at 15 (cycle 1 -> wraps to 0)
  // - Status lands at 0 (cycle 0)
  const Trb td[3] = {
      MakeSetupStageTrb(0x80, kUsbReqGetDescriptor, 0x0100, 0, 8, kTrtInData),
      MakeDataStageTrb(0x4000'0000ULL, 8, true),
      MakeStatusStageTrb(false),
  };
  uint64_t phys[3] = {};
  ring.EnqueueTd(td, 3, phys);

  EXPECT_THAT(phys[0], t::Eq(kRingPhys + 13 * 16));
  EXPECT_THAT(phys[1], t::Eq(kRingPhys + 14 * 16));
  EXPECT_THAT(phys[2], t::Eq(kRingPhys + 0 * 16));
  EXPECT_THAT(TrbType(storage[13]), t::Eq(kTrbSetupStage));
  EXPECT_THAT(storage[13].dw3 & kTrbCycle, t::Eq(1u));
  EXPECT_THAT(TrbType(storage[14]), t::Eq(kTrbDataStage));
  EXPECT_THAT(storage[14].dw3 & kTrbCycle, t::Eq(1u));
  EXPECT_THAT(TrbType(storage[15]), t::Eq(kTrbLink));
  EXPECT_THAT(storage[15].dw3 & kTrbCycle, t::Eq(1u));
  EXPECT_THAT(TrbType(storage[0]), t::Eq(kTrbStatusStage));
  EXPECT_THAT(storage[0].dw3 & kTrbCycle, t::Eq(0u));
  EXPECT_THAT(ring.enqueue_index(), t::Eq(1));
  EXPECT_THAT(ring.cycle(), t::Eq(0u));
}

TEST(XhciRingTest, EventRingPopsEventsAndTogglesCycleOnWrap) {
  alignas(64) Trb seg[16] = {};
  alignas(64) ErstEntry erst = {};
  constexpr uint64_t kSegPhys = 0x5000'0000ULL;
  constexpr uint64_t kErstPhys = 0x5000'1000ULL;

  EventRing<16> ring;
  ring.Init(seg, kSegPhys, &erst, kErstPhys);
  EXPECT_THAT(erst.ring_segment_base, t::Eq(kSegPhys));
  EXPECT_THAT(erst.ring_segment_size, t::Eq(16u));
  EXPECT_FALSE(ring.HasEvent());

  // Simulate hardware producing 16 events on lap 0 (cycle 1) and 3 events on
  // lap 1 (cycle 0).
  for (int i = 0; i < 16; ++i) {
    seg[i] = Trb{
        static_cast<uint32_t>(i), 0, static_cast<uint32_t>(kCcSuccess) << 24,
        (static_cast<uint32_t>(kTrbCommandCompletion) << kTrbTypeShift) |
            kTrbCycle};
  }
  for (int i = 0; i < 16; ++i) {
    ASSERT_TRUE(ring.HasEvent());
    const StatusOr<Trb> ev = ring.Pop();
    ASSERT_TRUE(ev.ok());
    EXPECT_THAT(ev->dw0, t::Eq(static_cast<uint32_t>(i)));
    EXPECT_THAT(TrbCompletionCode(*ev), t::Eq(kCcSuccess));
  }
  // Now wrapped to index 0 with consumer cycle 0; seg[0] still has cycle 1
  // from lap 0, so HasEvent() is false!
  EXPECT_THAT(ring.dequeue_index(), t::Eq(0));
  EXPECT_THAT(ring.cycle(), t::Eq(0u));
  EXPECT_FALSE(ring.HasEvent());

  // Hardware overwrites seg[0] with cycle 0.
  seg[0] = Trb{0x99u, 0, static_cast<uint32_t>(kCcSuccess) << 24,
               static_cast<uint32_t>(kTrbPortStatusChange) << kTrbTypeShift};
  EXPECT_TRUE(ring.HasEvent());
  const StatusOr<Trb> ev = ring.Pop();
  ASSERT_TRUE(ev.ok());
  EXPECT_THAT(ev->dw0, t::Eq(0x99u));
  EXPECT_THAT(ring.dequeue_index(), t::Eq(1));
  EXPECT_THAT(ring.dequeue_phys(), t::Eq(kSegPhys + 16));
}

TEST(XhciContextTest, ContextOffsetsAndBuildersRespectCsz32AndCsz64) {
  for (const int csz : {32, 64}) {
    std::vector<uint8_t> in_buf(static_cast<size_t>(33 * csz), 0xFF);
    std::vector<uint8_t> out_buf(static_cast<size_t>(32 * csz), 0);

    FormatAddressDeviceInputContext(in_buf.data(), csz, 5, kPortSpeedHigh,
                                    0x1234'5000ULL, 1u);
    const InputControlContext* const icc = InputCtrlCtx(in_buf.data());
    const SlotContext* const slot = InputSlotCtx(in_buf.data(), csz);
    const EndpointContext* const ep0 = InputEpCtx(in_buf.data(), csz, 1);

    EXPECT_THAT(reinterpret_cast<const uint8_t*>(slot) - in_buf.data(),
                t::Eq(csz));
    EXPECT_THAT(reinterpret_cast<const uint8_t*>(ep0) - in_buf.data(),
                t::Eq(2 * csz));
    EXPECT_THAT(icc->drop_flags, t::Eq(0u));
    EXPECT_THAT(icc->add_flags, t::Eq(0x3u));
    EXPECT_THAT(SlotContextEntries(*slot), t::Eq(1));
    EXPECT_THAT(SlotContextSpeed(*slot), t::Eq(kPortSpeedHigh));
    EXPECT_THAT(SlotContextRootHubPort(*slot), t::Eq(5));
    EXPECT_THAT(EpContextType(*ep0), t::Eq(kEpTypeControlBiDir));
    EXPECT_THAT(EpContextCerr(*ep0), t::Eq(3u));
    EXPECT_THAT(EpContextMaxPacketSize(*ep0), t::Eq(64u));
    EXPECT_THAT(EpContextDequeuePtr(*ep0), t::Eq(0x1234'5000ULL));
    EXPECT_THAT(EpContextDcs(*ep0), t::Eq(1u));
    EXPECT_THAT(EpContextAvgTrbLength(*ep0), t::Eq(8u));
    if (csz == 64) {
      // Upper 32 bytes of each 64-byte context slot must be zeroed.
      for (int b = 32; b < 64; ++b) {
        EXPECT_THAT(in_buf[static_cast<size_t>(csz + b)], t::Eq(0u));
        EXPECT_THAT(in_buf[static_cast<size_t>(2 * csz + b)], t::Eq(0u));
      }
    }

    // Copy Slot + EP0 to Output Device Context and test Configure Endpoint for
    // DCI 3 (EP1 IN).
    *OutputSlotCtx(out_buf.data(), csz) = *slot;
    *OutputEpCtx(out_buf.data(), csz, 1) = *ep0;
    // Simulate hardware-populated Slot State, USB Device Address, and EP0
    // Running state in Output Device Context, and verify Evaluate Context
    // clears RsvdZ output fields in Input Context (xHCI 6.2.2 / 6.2.3).
    OutputSlotCtx(out_buf.data(), csz)->dw3 = (2u << 27) | 5u;
    OutputEpCtx(out_buf.data(), csz, 1)->dw0 |= kEpStateRunning;

    FormatEvaluateEp0MpsInputContext(in_buf.data(), csz, out_buf.data(), 64);
    EXPECT_THAT(icc->drop_flags, t::Eq(0u));
    EXPECT_THAT(icc->add_flags, t::Eq(0x2u));
    EXPECT_THAT(slot->dw3, t::Eq(0u));
    EXPECT_THAT(EpContextState(*ep0), t::Eq(0u));
    EXPECT_THAT(EpContextMaxPacketSize(*ep0), t::Eq(64u));

    FormatConfigureInterruptEndpointInputContext(
        in_buf.data(), csz, out_buf.data(), 3, kPortSpeedFull, 8, 0, 10,
        0xabcd'0000ULL, 1u);
    EXPECT_THAT(icc->drop_flags, t::Eq(0u));
    EXPECT_THAT(icc->add_flags, t::Eq(1u | (1u << 3)));
    EXPECT_THAT(SlotContextEntries(*slot), t::Eq(3));
    const EndpointContext* const ep_in = InputEpCtx(in_buf.data(), csz, 3);
    EXPECT_THAT(reinterpret_cast<const uint8_t*>(ep_in) - in_buf.data(),
                t::Eq(4 * csz));
    EXPECT_THAT(EpContextType(*ep_in), t::Eq(kEpTypeInterruptIn));
    EXPECT_THAT(EpContextCerr(*ep_in), t::Eq(3u));
    EXPECT_THAT(EpContextMaxPacketSize(*ep_in), t::Eq(8u));
    EXPECT_THAT(EpContextInterval(*ep_in), t::Eq(6u));  // 10 ms * 8 = 80 -> 2^6
    EXPECT_THAT(EpContextAvgTrbLength(*ep_in), t::Eq(8u));
    EXPECT_THAT(EpContextMaxEsitPayload(*ep_in), t::Eq(8u));
  }
}

TEST(XhciContextTest, EndpointDciMpsAndIntervalFormulasMatchSpec) {
  EXPECT_THAT(EndpointDci(0x00), t::Eq(1));
  EXPECT_THAT(EndpointDci(0x81), t::Eq(3));
  EXPECT_THAT(EndpointDci(0x01), t::Eq(2));
  EXPECT_THAT(EndpointDci(0x82), t::Eq(5));
  EXPECT_THAT(EndpointDci(0x8F), t::Eq(31));

  EXPECT_THAT(InitialEp0MaxPacketSize(kPortSpeedLow), t::Eq(8u));
  EXPECT_THAT(InitialEp0MaxPacketSize(kPortSpeedFull), t::Eq(8u));
  EXPECT_THAT(InitialEp0MaxPacketSize(kPortSpeedHigh), t::Eq(64u));
  EXPECT_THAT(InitialEp0MaxPacketSize(kPortSpeedSuper), t::Eq(512u));
  EXPECT_THAT(InitialEp0MaxPacketSize(kPortSpeedSuperPlus), t::Eq(512u));

  EXPECT_THAT(DecodeEp0MaxPacketSize(kPortSpeedFull, 8), t::Eq(8u));
  EXPECT_THAT(DecodeEp0MaxPacketSize(kPortSpeedFull, 16), t::Eq(16u));
  EXPECT_THAT(DecodeEp0MaxPacketSize(kPortSpeedFull, 32), t::Eq(32u));
  EXPECT_THAT(DecodeEp0MaxPacketSize(kPortSpeedFull, 64), t::Eq(64u));
  EXPECT_THAT(DecodeEp0MaxPacketSize(kPortSpeedFull, 9), t::Eq(8u));
  EXPECT_THAT(DecodeEp0MaxPacketSize(kPortSpeedLow, 64), t::Eq(8u));
  EXPECT_THAT(DecodeEp0MaxPacketSize(kPortSpeedHigh, 8), t::Eq(64u));
  EXPECT_THAT(DecodeEp0MaxPacketSize(kPortSpeedSuper, 9), t::Eq(512u));

  // FS/LS: bInterval in ms frames -> floor(log2(frames * 8)) in [3, 10].
  EXPECT_THAT(EncodeInterruptInterval(kPortSpeedFull, 0), t::Eq(3u));
  EXPECT_THAT(EncodeInterruptInterval(kPortSpeedFull, 1), t::Eq(3u));
  EXPECT_THAT(EncodeInterruptInterval(kPortSpeedFull, 2), t::Eq(4u));
  EXPECT_THAT(EncodeInterruptInterval(kPortSpeedFull, 8), t::Eq(6u));
  EXPECT_THAT(EncodeInterruptInterval(kPortSpeedFull, 10), t::Eq(6u));
  EXPECT_THAT(EncodeInterruptInterval(kPortSpeedLow, 10), t::Eq(6u));
  EXPECT_THAT(EncodeInterruptInterval(kPortSpeedFull, 32), t::Eq(8u));
  EXPECT_THAT(EncodeInterruptInterval(kPortSpeedFull, 255), t::Eq(10u));

  // HS/SS: bInterval 1..16 -> bInterval - 1 in [0, 15].
  EXPECT_THAT(EncodeInterruptInterval(kPortSpeedHigh, 0), t::Eq(0u));
  EXPECT_THAT(EncodeInterruptInterval(kPortSpeedHigh, 1), t::Eq(0u));
  EXPECT_THAT(EncodeInterruptInterval(kPortSpeedHigh, 4), t::Eq(3u));
  EXPECT_THAT(EncodeInterruptInterval(kPortSpeedSuper, 4), t::Eq(3u));
  EXPECT_THAT(EncodeInterruptInterval(kPortSpeedHigh, 16), t::Eq(15u));
  EXPECT_THAT(EncodeInterruptInterval(kPortSpeedHigh, 20), t::Eq(15u));
}

TEST(UsbDescriptorsTest, FindsSingleAndCompositeBootKeyboardsAndSsCompanion) {
  // Composite Configuration:
  // - Interface 0: Boot Mouse (class 3, subclass 1, protocol 2), EP1 IN (0x81)
  // - Interface 1: Boot Keyboard (class 3, subclass 1, protocol 1),
  //   with an Interrupt OUT endpoint (0x02, ignored) followed by Interrupt IN
  //   endpoint (0x82, DCI 5, MPS 8, interval 4) + SS Companion (bMaxBurst = 2).
  const uint8_t composite[] = {
      // Configuration Descriptor (9B)
      9,
      kUsbDescConfiguration,
      63,
      0,
      2,
      1,
      0,
      0xA0,
      50,
      // Interface 0: Boot Mouse (9B)
      9,
      kUsbDescInterface,
      0,
      0,
      1,
      3,
      1,
      2,
      0,
      // Endpoint 0x81 Interrupt IN (7B)
      7,
      kUsbDescEndpoint,
      0x81,
      0x03,
      4,
      0,
      10,
      // Interface 1: Boot Keyboard (9B)
      9,
      kUsbDescInterface,
      1,
      0,
      2,
      3,
      1,
      1,
      0,
      // HID Descriptor (9B, skipped)
      9,
      kUsbDescHid,
      0x11,
      0x01,
      0,
      1,
      0x22,
      63,
      0,
      // Endpoint 0x02 Interrupt OUT (7B, skipped because OUT)
      7,
      kUsbDescEndpoint,
      0x02,
      0x03,
      8,
      0,
      10,
      // Endpoint 0x82 Interrupt IN (7B, matched!)
      7,
      kUsbDescEndpoint,
      0x82,
      0x03,
      8,
      0,
      4,
      // SS Endpoint Companion (6B, bMaxBurst = 2)
      6,
      kUsbDescSsEpCompanion,
      2,
      0,
      8,
      0,
  };
  const StatusOr<BootKeyboardMatch> match =
      FindBootKeyboard(composite, static_cast<int>(sizeof(composite)));
  ASSERT_TRUE(match.ok());
  EXPECT_THAT(match->config_value, t::Eq(1u));
  EXPECT_THAT(match->interface_number, t::Eq(1u));
  EXPECT_THAT(match->ep_address, t::Eq(0x82u));
  EXPECT_THAT(match->ep_dci, t::Eq(5u));
  EXPECT_THAT(match->max_packet_size, t::Eq(8u));
  EXPECT_THAT(match->max_burst, t::Eq(2u));
  EXPECT_THAT(match->b_interval, t::Eq(4u));
}

TEST(UsbDescriptorsTest, RejectsNonKeyboardsAndMalformedDescriptorsSafely) {
  // Mouse-only configuration.
  const uint8_t mouse_only[] = {
      9, kUsbDescConfiguration, 25,   0,    1, 1, 0,  0xA0, 50,
      9, kUsbDescInterface,     0,    0,    1, 3, 1,  2,    0,
      7, kUsbDescEndpoint,      0x81, 0x03, 4, 0, 10,
  };
  EXPECT_FALSE(
      FindBootKeyboard(mouse_only, static_cast<int>(sizeof(mouse_only))).ok());

  // Alternate setting 1 keyboard without alt 0 keyboard.
  const uint8_t alt1_only[] = {
      9, kUsbDescConfiguration, 25,   0,    1, 1, 0,  0xA0, 50,
      9, kUsbDescInterface,     0,    1,    1, 3, 1,  1,    0,
      7, kUsbDescEndpoint,      0x81, 0x03, 8, 0, 10,
  };
  EXPECT_FALSE(
      FindBootKeyboard(alt1_only, static_cast<int>(sizeof(alt1_only))).ok());

  // Zero-length sub-descriptor (bLength = 0) before the endpoint: terminates
  // the walk without infinite-looping.
  const uint8_t zero_length_sub[] = {
      9,
      kUsbDescConfiguration,
      27,
      0,
      1,
      1,
      0,
      0xA0,
      50,
      9,
      kUsbDescInterface,
      0,
      0,
      1,
      3,
      1,
      1,
      0,
      0,
      kUsbDescHid,
      7,
      kUsbDescEndpoint,
      0x81,
      0x03,
      8,
      0,
      10,
  };
  EXPECT_FALSE(FindBootKeyboard(zero_length_sub,
                                static_cast<int>(sizeof(zero_length_sub)))
                   .ok());

  // Sub-descriptor with bLength = 1: rejected cleanly.
  const uint8_t one_length_sub[] = {
      9,
      kUsbDescConfiguration,
      26,
      0,
      1,
      1,
      0,
      0xA0,
      50,
      9,
      kUsbDescInterface,
      0,
      0,
      1,
      3,
      1,
      1,
      0,
      1,
      7,
      kUsbDescEndpoint,
      0x81,
      0x03,
      8,
      0,
      10,
  };
  EXPECT_FALSE(
      FindBootKeyboard(one_length_sub, static_cast<int>(sizeof(one_length_sub)))
          .ok());

  // Endpoint descriptor truncated past buffer end.
  const uint8_t truncated_ep[] = {
      9, kUsbDescConfiguration, 25,   0,    1, 1, 0, 0xA0, 50,
      9, kUsbDescInterface,     0,    0,    1, 3, 1, 1,    0,
      7, kUsbDescEndpoint,      0x81, 0x03,
  };
  EXPECT_FALSE(
      FindBootKeyboard(truncated_ep, static_cast<int>(sizeof(truncated_ep)))
          .ok());
}

TEST(XhciContextTest,
     EncodesRouteStringTtFieldsAndHubSlotContextAcrossCsz32And64) {
  // Multi-tier Route String calculation across tiers 0..4 and clamping to 15.
  uint32_t route = 0;
  route = AppendRouteStringPort(route, 0, 1);
  EXPECT_THAT(route, t::Eq(0x00001u));
  route = AppendRouteStringPort(route, 1, 2);
  EXPECT_THAT(route, t::Eq(0x00021u));
  route = AppendRouteStringPort(route, 2, 3);
  EXPECT_THAT(route, t::Eq(0x00321u));
  route = AppendRouteStringPort(route, 3, 4);
  EXPECT_THAT(route, t::Eq(0x04321u));
  route = AppendRouteStringPort(route, 4, 20);  // Clamped to 15 (0xF).
  EXPECT_THAT(route, t::Eq(0xF4321u));
  // Out-of-bounds tiers (< 0 or >= 5) or non-positive ports are no-ops.
  EXPECT_THAT(AppendRouteStringPort(route, 5, 1), t::Eq(0xF4321u));
  EXPECT_THAT(AppendRouteStringPort(route, 0, 0), t::Eq(0xF4321u));

  for (const int csz : {32, 64}) {
    alignas(64) uint8_t buf[33 * 64] = {};
    FormatAddressDeviceInputContext(buf, csz, /*port_number=*/7,
                                    /*port_speed=*/2,
                                    /*ep0_ring_phys=*/0x100000u,
                                    /*ep0_cycle=*/1u,
                                    /*route_string=*/0x54321u,
                                    /*parent_hub_slot_id=*/2,
                                    /*parent_port_number=*/3, /*mtt=*/true);
    const SlotContext* const slot = InputSlotCtx(buf, csz);
    EXPECT_THAT(SlotContextRouteString(*slot), t::Eq(0x54321u));
    EXPECT_THAT(SlotContextSpeed(*slot), t::Eq(2));
    EXPECT_THAT(SlotContextEntries(*slot), t::Eq(1));
    EXPECT_TRUE(SlotContextMtt(*slot));
    EXPECT_FALSE(SlotContextIsHub(*slot));
    EXPECT_THAT(SlotContextRootHubPort(*slot), t::Eq(7));
    EXPECT_THAT(SlotContextParentHubSlotId(*slot), t::Eq(2));
    EXPECT_THAT(SlotContextParentPortNumber(*slot), t::Eq(3));

    alignas(64) uint8_t dev_ctx[32 * 64] = {};
    *OutputSlotCtx(dev_ctx, csz) = *slot;
    FormatConfigureInterruptEndpointInputContext(
        buf, csz, dev_ctx, /*dci=*/3, /*port_speed=*/3, /*max_packet_size=*/1,
        /*max_burst=*/0, /*b_interval=*/12, /*intr_ring_phys=*/0x200000u,
        /*intr_cycle=*/1u, /*is_hub=*/true, /*num_ports=*/7, /*ttt=*/2,
        /*mtt=*/true);
    EXPECT_THAT(SlotContextEntries(*slot), t::Eq(3));
    EXPECT_THAT(SlotContextRouteString(*slot), t::Eq(0x54321u));
    EXPECT_TRUE(SlotContextIsHub(*slot));
    EXPECT_TRUE(SlotContextMtt(*slot));
    EXPECT_THAT(SlotContextNumPorts(*slot), t::Eq(7));
    EXPECT_THAT(SlotContextTtt(*slot), t::Eq(2));
  }
}

TEST(UsbDescriptorsTest, FindsUsb2AndUsb3HubInterfacesAndEndpoints) {
  // USB 2.0 High-Speed Multi-TT Hub Configuration Descriptor (25 bytes).
  const uint8_t usb2_hub_cfg[] = {
      9,
      kUsbDescConfiguration,
      25,
      0,
      1,
      1,
      0,
      0xE0,
      0,
      9,
      kUsbDescInterface,
      0,
      0,
      1,
      kUsbClassHub,
      0,
      kHubProtocolHighSpeedMultiTt,
      0,
      7,
      kUsbDescEndpoint,
      0x81,
      0x03,
      1,
      0,
      12,
  };
  const StatusOr<HubMatch> match =
      FindUsbHub(usb2_hub_cfg, static_cast<int>(sizeof(usb2_hub_cfg)));
  ASSERT_TRUE(match.ok());
  EXPECT_THAT(match->config_value, t::Eq(1));
  EXPECT_THAT(match->interface_number, t::Eq(0));
  EXPECT_THAT(match->interface_protocol, t::Eq(kHubProtocolHighSpeedMultiTt));
  EXPECT_THAT(match->ep_address, t::Eq(0x81));
  EXPECT_THAT(match->ep_dci, t::Eq(3));
  EXPECT_THAT(match->max_packet_size, t::Eq(1));
  EXPECT_THAT(match->b_interval, t::Eq(12));
  EXPECT_THAT(match->max_burst, t::Eq(0));

  // USB 3.0 SuperSpeed Hub Configuration Descriptor (31 bytes, with SS
  // Endpoint Companion and Notification Interrupt IN bmAttributes = 0x13).
  const uint8_t usb3_hub_cfg[] = {
      9,
      kUsbDescConfiguration,
      31,
      0,
      1,
      1,
      0,
      0xE0,
      0,
      9,
      kUsbDescInterface,
      0,
      0,
      1,
      kUsbClassHub,
      0,
      0,
      0,
      7,
      kUsbDescEndpoint,
      0x81,
      0x13,
      2,
      0,
      8,
      6,
      kUsbDescSsEpCompanion,
      1,
      0,
      4,
      0,
  };
  const StatusOr<HubMatch> ss_match =
      FindUsbHub(usb3_hub_cfg, static_cast<int>(sizeof(usb3_hub_cfg)));
  ASSERT_TRUE(ss_match.ok());
  EXPECT_THAT(ss_match->ep_dci, t::Eq(3));
  EXPECT_THAT(ss_match->max_packet_size, t::Eq(2));
  EXPECT_THAT(ss_match->b_interval, t::Eq(8));
  EXPECT_THAT(ss_match->max_burst, t::Eq(1));

  // HID keyboard configuration descriptor is rejected by FindUsbHub.
  const uint8_t kbd_cfg[] = {
      9, kUsbDescConfiguration, 25,   0,    1, 1, 0,  0xA0, 50,
      9, kUsbDescInterface,     0,    0,    1, 3, 1,  1,    0,
      7, kUsbDescEndpoint,      0x81, 0x03, 8, 0, 10,
  };
  EXPECT_FALSE(FindUsbHub(kbd_cfg, static_cast<int>(sizeof(kbd_cfg))).ok());
}

TEST(UsbDescriptorsTest, ParsesUsb2AndUsb3HubDescriptorsAndRejectsMalformed) {
  // 9-byte USB 2.0 Hub Descriptor (0x29): 4 ports, TTT = 2 (0x0049), 50 * 2ms.
  const uint8_t usb2_desc[] = {9, kUsbDescHub, 4, 0x49, 0x00, 50, 100, 0, 0xFF};
  const StatusOr<HubDescriptorInfo> info = ParseHubDescriptor(
      usb2_desc, static_cast<int>(sizeof(usb2_desc)), kUsbDescHub);
  ASSERT_TRUE(info.ok());
  EXPECT_THAT(info->num_ports, t::Eq(4));
  EXPECT_THAT(info->ttt, t::Eq(2));
  EXPECT_THAT(info->pwr_on_to_pwr_good, t::Eq(50));

  // 12-byte USB 3.0 SuperSpeed Hub Descriptor (0x2A): 4 ports, 25 * 2ms.
  const uint8_t usb3_desc[] = {12, kUsbDescSsHub, 4,    0x09, 0x00, 25, 100,
                               4,  0x90,          0x01, 0x00, 0x00};
  const StatusOr<HubDescriptorInfo> ss_info = ParseHubDescriptor(
      usb3_desc, static_cast<int>(sizeof(usb3_desc)), kUsbDescSsHub);
  ASSERT_TRUE(ss_info.ok());
  EXPECT_THAT(ss_info->num_ports, t::Eq(4));
  EXPECT_THAT(ss_info->ttt, t::Eq(0));
  EXPECT_THAT(ss_info->pwr_on_to_pwr_good, t::Eq(25));

  // Reject wrong descriptor type, truncated length, or bNbrPorts == 0.
  EXPECT_FALSE(ParseHubDescriptor(usb2_desc,
                                  static_cast<int>(sizeof(usb2_desc)),
                                  kUsbDescSsHub)
                   .ok());
  EXPECT_FALSE(ParseHubDescriptor(usb2_desc, 6, kUsbDescHub).ok());
  EXPECT_FALSE(ParseHubDescriptor(usb3_desc, 10, kUsbDescSsHub).ok());
  const uint8_t zero_ports[] = {9,  kUsbDescHub, 0, 0x09, 0x00,
                                10, 100,         0, 0xFF};
  EXPECT_FALSE(ParseHubDescriptor(zero_ports,
                                  static_cast<int>(sizeof(zero_ports)),
                                  kUsbDescHub)
                   .ok());
}

TEST(UsbDescriptorsTest, HubDescriptorParserEnforcesPortCountAndLengthBounds) {
  // bNbrPorts 16 is the most a hub may have (USB 2.0 11.23.2.1).
  const uint8_t d16[] = {13, kUsbDescHub, 16, 0x09, 0x00, 10,  100,
                         0,  0,           0,  0xFF, 0xFF, 0xFF};
  const StatusOr<HubDescriptorInfo> info16 =
      ParseHubDescriptor(d16, static_cast<int>(sizeof(d16)), kUsbDescHub);
  ASSERT_TRUE(info16.ok());
  EXPECT_THAT(info16->num_ports, t::Eq(16));
  const uint8_t d17[] = {13, kUsbDescHub, 17, 0x09, 0x00, 10,  100,
                         0,  0,           0,  0xFF, 0xFF, 0xFF};
  EXPECT_FALSE(
      ParseHubDescriptor(d17, static_cast<int>(sizeof(d17)), kUsbDescHub).ok());
  const uint8_t d255[39] = {39, kUsbDescHub, 255, 0x09, 0x00, 10, 100};
  EXPECT_FALSE(
      ParseHubDescriptor(d255, static_cast<int>(sizeof(d255)), kUsbDescHub)
          .ok());
  // bLength below the minimum while the buffer itself is long enough.
  const uint8_t short_len[] = {6, kUsbDescHub, 4, 0x09, 0x00, 10, 100, 0, 0xFF};
  EXPECT_FALSE(ParseHubDescriptor(
                   short_len, static_cast<int>(sizeof(short_len)), kUsbDescHub)
                   .ok());
  const uint8_t ss_short_len[] = {11, kUsbDescSsHub, 4,    0x09, 0x00, 10, 100,
                                  4,  0x90,          0x01, 0,    0};
  EXPECT_FALSE(ParseHubDescriptor(ss_short_len,
                                  static_cast<int>(sizeof(ss_short_len)),
                                  kUsbDescSsHub)
                   .ok());
  // expected_type 0 accepts only the hub descriptor types 0x29 and 0x2A.
  const uint8_t other[] = {9, 0x24, 4, 0x09, 0x00, 10, 100, 0, 0xFF};
  EXPECT_FALSE(
      ParseHubDescriptor(other, static_cast<int>(sizeof(other)), 0).ok());
  const uint8_t ok29[] = {9, kUsbDescHub, 4, 0x09, 0x00, 10, 100, 0, 0xFF};
  EXPECT_TRUE(ParseHubDescriptor(ok29, static_cast<int>(sizeof(ok29)), 0).ok());
  EXPECT_FALSE(ParseHubDescriptor(nullptr, 9, kUsbDescHub).ok());

  // A truncated hub configuration descriptor (endpoint cut off) has no hub
  // interrupt endpoint.
  const uint8_t trunc_cfg[] = {
      9,
      kUsbDescConfiguration,
      25,
      0,
      1,
      1,
      0,
      0xE0,
      0,
      9,
      kUsbDescInterface,
      0,
      0,
      1,
      kUsbClassHub,
      0,
      0,
      0,
      7,
      kUsbDescEndpoint,
      0x81,
  };
  EXPECT_FALSE(FindUsbHub(trunc_cfg, static_cast<int>(sizeof(trunc_cfg))).ok());

  // A compliant multi-TT hub (USB 2.0 11.23.1): alternate setting 0 is
  // single-TT and alternate setting 1 multi-TT; the driver uses setting 0.
  constexpr uint8_t kStt = kHubProtocolHighSpeedSingleTt;
  constexpr uint8_t kMtt = kHubProtocolHighSpeedMultiTt;
  const uint8_t mtt_cfg[] = {
      9,
      kUsbDescConfiguration,
      41,
      0,
      1,
      1,
      0,
      0xE0,
      0,
      9,
      kUsbDescInterface,
      0,
      0,
      1,
      kUsbClassHub,
      0,
      kStt,
      0,
      7,
      kUsbDescEndpoint,
      0x81,
      0x03,
      1,
      0,
      12,
      9,
      kUsbDescInterface,
      0,
      1,
      1,
      kUsbClassHub,
      0,
      kMtt,
      0,
      7,
      kUsbDescEndpoint,
      0x81,
      0x03,
      1,
      0,
      11,
  };
  const StatusOr<HubMatch> mtt_match =
      FindUsbHub(mtt_cfg, static_cast<int>(sizeof(mtt_cfg)));
  ASSERT_TRUE(mtt_match.ok());
  EXPECT_THAT(mtt_match->interface_protocol,
              t::Eq(kHubProtocolHighSpeedSingleTt));
  EXPECT_THAT(mtt_match->b_interval, t::Eq(12));
}

}  // namespace
}  // namespace protos::usb
