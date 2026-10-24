#include <cstdint>
#include <cstring>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "user/usbkbd/xhci_controller.h"
#include "user/usbkbd/xhci_regs.h"
#include "user/usbkbd/xhci_ring.h"
#include "user/usbkbd/xhci_sim.h"
#include "user/usbkbd/xhci_sim_usb.h"
#include "user/usbkbd/xhci_sim_world.h"

namespace protos::xhci_sim {
namespace {

namespace t = ::testing;

TEST(XhciSimRingsTest, SimDmaArenaAllocatesHighLowAndEnforcesGuardGaps) {
  SimDmaArena arena;
  const usb::DmaBuffer b1 = arena.Alloc(4096, /*below_4gib=*/false);
  ASSERT_THAT(b1.va, t::NotNull());
  EXPECT_THAT(b1.phys, t::Eq(SimDmaArena::kHighBase));
  EXPECT_THAT(arena.Translate(b1.phys, 4096, /*require_below_4gib=*/false),
              t::NotNull());
  // With AC64=0 (require_below_4gib), translating an address >= 4 GiB fails.
  EXPECT_THAT(arena.Translate(b1.phys, 4096, /*require_below_4gib=*/true),
              t::IsNull());

  const usb::DmaBuffer b2 = arena.Alloc(4096, /*below_4gib=*/false);
  ASSERT_THAT(b2.va, t::NotNull());
  // 4096-byte allocation + 4096-byte guard gap.
  EXPECT_THAT(b2.phys, t::Eq(SimDmaArena::kHighBase + 0x2000ULL));
  // Guard page between b1 and b2 is unmapped.
  EXPECT_THAT(arena.Translate(SimDmaArena::kHighBase + 0x1000ULL, 16, false),
              t::IsNull());
  // Crossing from b1 into the guard page is rejected.
  EXPECT_THAT(arena.Translate(SimDmaArena::kHighBase + 0x0FF8ULL, 16, false),
              t::IsNull());

  const usb::DmaBuffer low = arena.Alloc(4096, /*below_4gib=*/true);
  ASSERT_THAT(low.va, t::NotNull());
  EXPECT_THAT(low.phys, t::Lt(0x1'0000'0000ULL));
  EXPECT_THAT(arena.Translate(low.phys, 4096, /*require_below_4gib=*/true),
              t::NotNull());

  SimClock clock;
  FakePlatform platform(&clock, &arena);
  platform.set_fail_dma_after(1);
  EXPECT_THAT(platform.DmaAlloc(4096, false).va, t::NotNull());
  EXPECT_THAT(platform.DmaAlloc(4096, false).va, t::IsNull());
}

TEST(XhciSimRingsTest, DmaWhileBusMasterOffIsAViolation) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  ASSERT_THAT(world.controller(idx).TakeOwnershipAndReset(),
              t::Eq(usb::XhciStatus::kOk));
  world.sim(idx).set_bus_master(false);
  world.sim(idx).AllowViolation(Violation::kDmaWhileBusMasterOff);
  EXPECT_THAT(world.controller(idx).Start(usb::IrqMode::kMsix),
              t::Eq(usb::XhciStatus::kOk));
  EXPECT_TRUE(world.sim(idx).HasViolation(Violation::kDmaWhileBusMasterOff));
}

TEST(XhciSimRingsTest, InvalidDmaAddressOnStartIsAViolation) {
  SimClock clock;
  SimDmaArena arena;
  XhciSim sim(&clock, XhciSimConfig{});
  sim.set_dma_arena(&arena);
  sim.AllowViolation(Violation::kInvalidDmaAddress);

  const int64_t usbcmd = sim.op(usb::kOpUsbCmd);
  const int64_t usbsts = sim.op(usb::kOpUsbSts);
  sim.Write32(usbcmd, usb::kUsbCmdHcrst);
  clock.Advance(2);
  ASSERT_THAT(sim.Read32(usbcmd) & usb::kUsbCmdHcrst, t::Eq(0u));
  ASSERT_THAT(sim.Read32(usbsts) & usb::kUsbStsHch, t::Eq(usb::kUsbStsHch));

  // Program unmapped DCBAAP, CRCR, ERSTBA and start the controller.
  sim.Write32(sim.op(usb::kOpDcbaap), 0xDEAD'0000u);
  sim.Write32(sim.op(usb::kOpDcbaap + 4), 0u);
  sim.Write32(sim.op(usb::kOpCrcr), 0xBEEF'0001u);
  sim.Write32(sim.op(usb::kOpCrcr + 4), 0u);
  sim.Write32(sim.interrupter(0, usb::kIrErstSz), 1u);
  sim.Write32(sim.interrupter(0, usb::kIrErstBa), 0xCAFE'0000u);
  sim.Write32(sim.interrupter(0, usb::kIrErstBa + 4), 0u);
  sim.Write32(usbcmd, usb::kUsbCmdRs);
  EXPECT_GE(sim.CountViolations(Violation::kInvalidDmaAddress), 3);
}

TEST(XhciSimRingsTest, ErstBaWrittenBeforeErstSzIsAViolation) {
  SimClock clock;
  XhciSim sim(&clock, XhciSimConfig{});
  sim.AllowViolation(Violation::kErstBaBeforeSize);

  const int64_t usbcmd = sim.op(usb::kOpUsbCmd);
  const int64_t usbsts = sim.op(usb::kOpUsbSts);
  sim.Write32(usbcmd, usb::kUsbCmdHcrst);
  clock.Advance(2);
  ASSERT_THAT(sim.Read32(usbcmd) & usb::kUsbCmdHcrst, t::Eq(0u));
  ASSERT_THAT(sim.Read32(usbsts) & usb::kUsbStsHch, t::Eq(usb::kUsbStsHch));

  sim.Write32(sim.interrupter(0, usb::kIrErstBa), 0x1000u);
  sim.Write32(sim.interrupter(0, usb::kIrErstBa + 4), 0u);
  EXPECT_THAT(sim.CountViolations(Violation::kErstBaBeforeSize), t::Eq(1));
}

TEST(XhciSimRingsTest, RingModifiedWhileRunningIsAViolationExceptCrcrAbort) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));

  XhciSim& sim = world.sim(idx);
  sim.AllowViolation(Violation::kRingModifiedWhileRunning);
  sim.Write32(sim.op(usb::kOpConfig), 2u);
  sim.Write32(sim.op(usb::kOpDcbaap), 0x2000u);
  sim.Write32(sim.op(usb::kOpDcbaap + 4), 0u);
  sim.Write32(sim.interrupter(0, usb::kIrErstSz), 1u);
  sim.Write32(sim.interrupter(0, usb::kIrErstBa), 0x3000u);
  sim.Write32(sim.interrupter(0, usb::kIrErstBa + 4), 0u);
  sim.Write32(sim.op(usb::kOpCrcr), 0x4001u);
  sim.Write32(sim.op(usb::kOpCrcr + 4), 0u);
  EXPECT_THAT(sim.CountViolations(Violation::kRingModifiedWhileRunning),
              t::Eq(8));

  // Writing CRCR Command Abort (bit 2) while preserving the upper 32 bits of
  // the Command Ring pointer while running is valid.
  const int before = sim.CountViolations(Violation::kRingModifiedWhileRunning);
  sim.Write32(sim.op(usb::kOpCrcr), usb::kCrcrCa);
  sim.Write32(sim.op(usb::kOpCrcr + 4),
              static_cast<uint32_t>(SimDmaArena::kHighBase >> 32));
  EXPECT_THAT(sim.CountViolations(Violation::kRingModifiedWhileRunning),
              t::Eq(before));
}

TEST(XhciSimRingsTest,
     Usb3HotResetIsValidAndUsb2WarmResetOrCrcrHighMismatchIsViolation) {
  XhciSimConfig cfg;
  cfg.attached_speed = {4};  // SuperSpeed on USB 3 port 1.
  SimWorld world;
  const int idx = world.AddController("00:04.0", cfg);
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));

  XhciSim& sim = world.sim(idx);
  // Hot Reset (PR) on a USB 3 port is legal per xHCI 5.4.8.
  const int64_t p1 = sim.portsc(1);
  const uint32_t portsc1 = sim.Read32(p1);
  sim.Write32(p1, (portsc1 & usb::kPortScPreserveMask) | usb::kPortScPr);
  EXPECT_THAT(sim.violations(), t::IsEmpty());

  // Warm Port Reset (WPR, bit 31) on a USB 2 port is RsvdZ.
  sim.AllowViolation(Violation::kRsvdZWrittenOne);
  const int64_t p5 = sim.portsc(5);
  const uint32_t portsc5 = sim.Read32(p5);
  sim.Write32(p5, (portsc5 & usb::kPortScPreserveMask) | usb::kPortScWpr);
  EXPECT_THAT(sim.CountViolations(Violation::kRsvdZWrittenOne), t::Eq(1));

  // Writing a mismatched upper 32-bit CRCR address after CRCR.CA stops the
  // Command Ring corrupts the high dword of CRCR.
  sim.AllowViolation(Violation::kCommandRingPointerCorrupted);
  sim.Write32(sim.op(usb::kOpCrcr), usb::kCrcrCa);
  sim.Write32(sim.op(usb::kOpCrcr + 4), 0xDEAD'BEEFu);
  EXPECT_THAT(sim.CountViolations(Violation::kCommandRingPointerCorrupted),
              t::Eq(1));
}

TEST(XhciSimRingsTest, EventRingEhbGatesIrqUntilClearedAndReassertsIfNonEmpty) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  XhciSim& sim = world.sim(idx);
  ASSERT_FALSE(sim.irq_pending());
  const int base_irqs = sim.irq_count();

  usb::Trb ev1{};
  ev1.dw2 = static_cast<uint32_t>(usb::kCcSuccess) << 24;
  ev1.dw3 = static_cast<uint32_t>(usb::kTrbPortStatusChange)
            << usb::kTrbTypeShift;
  ev1.dw0 = 5u << 24;
  sim.PushRawEvent(ev1);
  EXPECT_TRUE(sim.irq_pending());
  EXPECT_THAT(sim.irq_count(), t::Eq(base_irqs + 1));

  // Pushing a second event while EHB is still set does not assert a second IRQ.
  sim.PushRawEvent(ev1);
  EXPECT_THAT(sim.irq_count(), t::Eq(base_irqs + 1));

  // Clear IMAN.IP / USBSTS.EINT and advance ERDP by only 1 TRB (leaving 1
  // unconsumed event on the ring) with EHB=1: simulator immediately re-asserts.
  (void)sim.ConsumeIrq();
  sim.Write32(sim.op(usb::kOpUsbSts), usb::kUsbStsEint);
  sim.Write32(sim.interrupter(0, usb::kIrIman), usb::kImanIp | usb::kImanIe);
  const uint64_t erdp0 =
      (static_cast<uint64_t>(sim.Peek(sim.interrupter(0, usb::kIrErdp + 4)))
       << 32) |
      (sim.Peek(sim.interrupter(0, usb::kIrErdp)) & ~0xFu);
  const uint64_t erdp1 = erdp0 + sizeof(usb::Trb);
  sim.Write32(sim.interrupter(0, usb::kIrErdp),
              static_cast<uint32_t>(erdp1) | usb::kErdpEhb);
  sim.Write32(sim.interrupter(0, usb::kIrErdp + 4),
              static_cast<uint32_t>(erdp1 >> 32));
  EXPECT_TRUE(sim.irq_pending());
  EXPECT_THAT(sim.irq_count(), t::Eq(base_irqs + 2));

  // Drain the remaining event via HandleInterrupt; EHB and USBSTS.EINT clear
  // and no IRQ remains pending.
  (void)sim.ConsumeIrq();
  world.controller(idx).HandleInterrupt();
  EXPECT_FALSE(sim.irq_pending());
  EXPECT_THAT(sim.Read32(sim.op(usb::kOpUsbSts)) & usb::kUsbStsEint, t::Eq(0u));
  EXPECT_THAT(sim.Read32(sim.interrupter(0, usb::kIrErdp)) & usb::kErdpEhb,
              t::Eq(0u));
  EXPECT_THAT(sim.CountViolations(Violation::kImanClearedBeforeEint), t::Eq(0));

  // Clearing IMAN.IP while USBSTS.EINT is still set is flagged as a violation.
  sim.AllowViolation(Violation::kImanClearedBeforeEint);
  sim.PushRawEvent(ev1);
  (void)sim.ConsumeIrq();
  sim.Write32(sim.interrupter(0, usb::kIrIman), usb::kImanIp | usb::kImanIe);
  EXPECT_THAT(sim.CountViolations(Violation::kImanClearedBeforeEint), t::Eq(1));
}

TEST(XhciSimRingsTest,
     M5TwoPhaseBringUpWithBusMasterOffDuringResetAndEarlySpuriousIrq) {
  // Mirrors user/usbkbd/usbkbd.cpp BringUpController: Phase 1
  // (TakeOwnershipAndReset)
  // runs before sys_pci_enable_bus_master(), then Bus Master and IRQ are
  // enabled, and Phase 2 (Start) programs DMA and starts the controller.
  SimClock clock;
  SimDmaArena arena;
  FakePlatform platform(&clock, &arena);
  XhciSimConfig cfg = IntelLikeConfig();
  cfg.running = true;
  cfg.legacy.bios_owned = true;
  cfg.legacy.behavior = BiosBehavior::kReleasesAfterTicks;
  cfg.legacy.release_after_ticks = 2;
  XhciSim sim(&clock, cfg);
  sim.set_dma_arena(&arena);
  sim.set_bus_master(false);

  usb::XhciController ctrl;
  ctrl.Init(platform, sim, "00:04.0");
  EXPECT_THAT(ctrl.NextDeadline(), t::Eq(usb::kNoDeadline));

  ASSERT_THAT(ctrl.TakeOwnershipAndReset(), t::Eq(usb::XhciStatus::kOk));
  EXPECT_THAT(ctrl.state(), t::Eq(usb::XhciState::kReset));
  EXPECT_FALSE(sim.HasViolation(Violation::kDmaWhileBusMasterOff));

  // Enable PCI Bus Master and verify that spurious HandleInterrupt() /
  // NextDeadline() calls while still in kReset before Phase 2 Start() are safe
  // no-ops.
  sim.set_bus_master(true);
  ctrl.HandleInterrupt();
  EXPECT_THAT(ctrl.NextDeadline(), t::Eq(usb::kNoDeadline));

  ASSERT_THAT(ctrl.Start(usb::IrqMode::kMsix), t::Eq(usb::XhciStatus::kOk));
  EXPECT_THAT(ctrl.state(), t::Eq(usb::XhciState::kRunning));
  EXPECT_THAT(sim.violations(), t::IsEmpty());
}

TEST(XhciSimRingsTest, ControlTransfersWrapAcrossEp0RingLinkTrbOnMultipleLaps) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbKeyboard& kbd = world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));

  XhciSim& sim = world.sim(idx);
  const XhciSim::SimSlot& slot = sim.slot(1);
  ASSERT_TRUE(slot.enabled);
  ASSERT_THAT(slot.ep[1].state, t::Eq(usb::kEpStateRunning));

  const usb::DmaBuffer data_buf = world.arena().Alloc(4096, false);
  ASSERT_THAT(data_buf.va, t::NotNull());

  // Synchronize a ProducerRing<256> with Slot 1 EP0's existing DMA ring at its
  // current dequeue pointer so we can issue 200 multi-TRB control transfers
  // (534 TRBs, > 2 full laps across the Link TRB at index 255).
  const uint64_t existing_deq = slot.ep[1].dequeue;
  const uint64_t existing_base = existing_deq & ~0xFFFull;
  const int initial_index =
      static_cast<int>((existing_deq - existing_base) / sizeof(usb::Trb));
  auto* const existing_trbs = reinterpret_cast<usb::Trb*>(
      world.arena().Translate(existing_base, 4096, false));
  ASSERT_THAT(existing_trbs, t::NotNull());

  usb::ProducerRing<256> synced_ring;
  synced_ring.Init(existing_trbs, existing_base);
  for (int i = 0; i < initial_index; ++i) {
    (void)synced_ring.Enqueue(usb::Trb{});
  }
  ASSERT_THAT(synced_ring.enqueue_phys(), t::Eq(slot.ep[1].dequeue));
  ASSERT_THAT(synced_ring.cycle(), t::Eq(slot.ep[1].dcs));

  // Issue 200 control transfers alternating 3-TRB GET_DESCRIPTOR(Device, 18B)
  // and 2-TRB SET_IDLE requests (534 TRBs total, crossing Link TRB 255 twice
  // at different TD alignments so Setup/Data/Status straddle index 255).
  auto* const data_bytes = static_cast<uint8_t*>(data_buf.va);
  const int base_idle_count = kbd.set_idle_count();
  for (int iter = 0; iter < 200; ++iter) {
    if ((iter % 3) != 2) {
      std::memset(data_bytes, 0, 18);
      const usb::Trb td[3] = {
          usb::MakeSetupStageTrb(
              usb::kUsbReqTypeStdDevIn, usb::kUsbReqGetDescriptor,
              static_cast<uint16_t>(usb::kUsbDescDevice << 8), 0, 18,
              usb::kTrtInData),
          usb::MakeDataStageTrb(data_buf.phys, 18, true),
          usb::MakeStatusStageTrb(false),
      };
      uint64_t phys[3] = {};
      synced_ring.EnqueueTd(td, 3, phys);
      sim.Write32(sim.doorbell(1), 1u);
      world.ServiceReady();
      EXPECT_THAT(data_bytes[0], t::Eq(18u));
      EXPECT_THAT(data_bytes[1], t::Eq(usb::kUsbDescDevice));
    } else {
      const usb::Trb td[2] = {
          usb::MakeSetupStageTrb(usb::kUsbReqTypeClassIfaceOut,
                                 usb::kHidReqSetIdle, 0, 0, 0, usb::kTrtNoData),
          usb::MakeStatusStageTrb(true),
      };
      uint64_t phys[2] = {};
      synced_ring.EnqueueTd(td, 2, phys);
      sim.Write32(sim.doorbell(1), 1u);
      world.ServiceReady();
    }
  }
  EXPECT_THAT(kbd.set_idle_count(), t::Eq(base_idle_count + 66));
  EXPECT_THAT(sim.violations(), t::IsEmpty());
}

}  // namespace
}  // namespace protos::xhci_sim
