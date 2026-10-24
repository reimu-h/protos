#include <cstdint>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "user/usbkbd/usb_descriptors.h"
#include "user/usbkbd/xhci_controller.h"
#include "user/usbkbd/xhci_regs.h"
#include "user/usbkbd/xhci_sim.h"
#include "user/usbkbd/xhci_sim_usb.h"
#include "user/usbkbd/xhci_sim_world.h"

namespace protos::xhci_sim {
namespace {

namespace t = ::testing;

TEST(XhciEnumTest, StartValidatesPreconditionsAndAllocationFailures) {
  SimClock clock;
  SimDmaArena arena;
  FakePlatform platform(&clock, &arena);
  XhciSim sim(&clock, XhciSimConfig{});
  sim.set_dma_arena(&arena);

  usb::XhciController ctrl;
  // Calling Start() before Init() fails with kNotBound.
  EXPECT_THAT(ctrl.Start(usb::IrqMode::kMsix),
              t::Eq(usb::XhciStatus::kNotBound));
  EXPECT_THAT(ctrl.state(), t::Eq(usb::XhciState::kUnbound));

  // Calling Start() before TakeOwnershipAndReset() fails with kBadState.
  ctrl.Init(platform, sim, "00:04.0");
  EXPECT_THAT(ctrl.Start(usb::IrqMode::kMsix),
              t::Eq(usb::XhciStatus::kBadState));
  EXPECT_THAT(ctrl.state(), t::Eq(usb::XhciState::kError));

  // PAGESIZE without 4 KiB bit (bit 0) fails with kUnsupportedPageSize.
  {
    XhciSimConfig bad_page;
    bad_page.page_size_raw = 0x2;  // 8 KiB only
    XhciSim sim_page(&clock, bad_page);
    sim_page.set_dma_arena(&arena);
    usb::XhciController c;
    c.Init(platform, sim_page, "00:04.0");
    ASSERT_THAT(c.TakeOwnershipAndReset(), t::Eq(usb::XhciStatus::kOk));
    EXPECT_THAT(c.Start(usb::IrqMode::kMsix),
                t::Eq(usb::XhciStatus::kUnsupportedPageSize));
    EXPECT_THAT(c.state(), t::Eq(usb::XhciState::kError));
  }

  // Core DMA allocation failure returns kNoMemory.
  {
    FakePlatform fail_plat(&clock, &arena);
    fail_plat.set_fail_dma_after(0);
    XhciSim s(&clock, XhciSimConfig{});
    s.set_dma_arena(&arena);
    usb::XhciController c;
    c.Init(fail_plat, s, "00:04.0");
    ASSERT_THAT(c.TakeOwnershipAndReset(), t::Eq(usb::XhciStatus::kOk));
    EXPECT_THAT(c.Start(usb::IrqMode::kMsix),
                t::Eq(usb::XhciStatus::kNoMemory));
    EXPECT_THAT(c.state(), t::Eq(usb::XhciState::kError));
  }

  // Scratchpad buffer allocation failure on Intel profile returns kNoMemory.
  {
    FakePlatform fail_plat(&clock, &arena);
    fail_plat.set_fail_dma_after(1);  // core+array ok, scratchpad buffer fails
    XhciSim s(&clock, IntelLikeConfig());
    s.set_dma_arena(&arena);
    usb::XhciController c;
    c.Init(fail_plat, s, "00:14.0");
    ASSERT_THAT(c.TakeOwnershipAndReset(), t::Eq(usb::XhciStatus::kOk));
    EXPECT_THAT(c.Start(usb::IrqMode::kMsi), t::Eq(usb::XhciStatus::kNoMemory));
    EXPECT_THAT(c.state(), t::Eq(usb::XhciState::kError));
  }

  // Unaligned DMA physical address is rejected with kNoMemory.
  {
    FakePlatform bad_align(&clock, &arena);
    bad_align.set_bad_dma_alignment(true);
    XhciSim s(&clock, XhciSimConfig{});
    s.set_dma_arena(&arena);
    usb::XhciController c;
    c.Init(bad_align, s, "00:04.0");
    ASSERT_THAT(c.TakeOwnershipAndReset(), t::Eq(usb::XhciStatus::kOk));
    EXPECT_THAT(c.Start(usb::IrqMode::kMsix),
                t::Eq(usb::XhciStatus::kNoMemory));
    EXPECT_THAT(c.state(), t::Eq(usb::XhciState::kError));
  }

  // AC64 = 0 rejects >= 4 GiB DMA addresses, and succeeds when below_4gib is
  // honored by the platform.
  {
    XhciSimConfig ac32;
    ac32.ac64 = false;
    FakePlatform high_plat(&clock, &arena);
    high_plat.set_ignore_below_4gib(true);
    XhciSim s1(&clock, ac32);
    s1.set_dma_arena(&arena);
    usb::XhciController c1;
    c1.Init(high_plat, s1, "00:04.0");
    ASSERT_THAT(c1.TakeOwnershipAndReset(), t::Eq(usb::XhciStatus::kOk));
    EXPECT_THAT(c1.Start(usb::IrqMode::kMsix),
                t::Eq(usb::XhciStatus::kNoMemory));

    FakePlatform low_plat(&clock, &arena);
    XhciSim s2(&clock, ac32);
    s2.set_dma_arena(&arena);
    usb::XhciController c2;
    c2.Init(low_plat, s2, "00:04.0");
    ASSERT_THAT(c2.TakeOwnershipAndReset(), t::Eq(usb::XhciStatus::kOk));
    EXPECT_THAT(c2.Start(usb::IrqMode::kMsix), t::Eq(usb::XhciStatus::kOk));
    EXPECT_THAT(c2.state(), t::Eq(usb::XhciState::kRunning));
  }

  // Controller that never clears HCH after setting RS fails with kStartTimeout.
  {
    XhciSimConfig never_starts;
    never_starts.never_starts = true;
    XhciSim s(&clock, never_starts);
    s.set_dma_arena(&arena);
    usb::XhciController c;
    c.Init(platform, s, "00:04.0");
    ASSERT_THAT(c.TakeOwnershipAndReset(), t::Eq(usb::XhciStatus::kOk));
    EXPECT_THAT(c.Start(usb::IrqMode::kMsix),
                t::Eq(usb::XhciStatus::kStartTimeout));
    EXPECT_THAT(c.state(), t::Eq(usb::XhciState::kError));
  }
}

TEST(XhciEnumTest, ReprogramsCleanlyAfterSeaBiosOrOvmfPrebootState) {
  XhciSimConfig intel_running = IntelLikeConfig();
  intel_running.running = true;
  intel_running.usbcmd_extra = usb::kUsbCmdInte | usb::kUsbCmdHsee;
  intel_running.usbsts_pending = usb::kUsbStsEint | usb::kUsbStsPcd;

  struct Case {
    XhciSimConfig cfg;
    usb::IrqMode mode;
    int port;
  };
  const Case cases[] = {
      {SeaBiosLeftRunningConfig(), usb::IrqMode::kMsix, 5},
      {OvmfLeftHaltedConfig(), usb::IrqMode::kMsix, 5},
      {intel_running, usb::IrqMode::kMsi, 1},
  };
  for (const Case& tc : cases) {
    SimWorld world;
    const int idx = world.AddController("00:04.0", tc.cfg, tc.mode);
    SimUsbKeyboard& kbd =
        world.AttachKeyboard(idx, tc.port, HighSpeedKeyboardConfig());
    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    XhciSim& sim = world.sim(idx);
    if (tc.cfg.running) {
      EXPECT_THAT(sim.halt_observed_seq(), t::Gt(0));
      EXPECT_THAT(sim.halt_observed_seq(), t::Lt(sim.reset_write_seq()));
    }
    EXPECT_THAT(sim.reset_write_seq(), t::Lt(sim.reset_observed_seq()));
    // First Phase-2 programming write to CONFIG occurs strictly after reset
    // completion was observed.
    const std::vector<Access> cfg_writes = sim.WritesTo(sim.op(usb::kOpConfig));
    ASSERT_FALSE(cfg_writes.empty());
    EXPECT_THAT(cfg_writes.front().seq, t::Gt(sim.reset_observed_seq()));

    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
    world.TypeAscii(kbd, "ok");
    EXPECT_THAT(world.platform().input(), t::Eq("ok"));
  }
}

struct EnumProfile {
  const char* name;
  const char* label;
  XhciSimConfig (*make_config)();
  usb::IrqMode irq_mode;
  int usb2_port;
  int usb3_port;
};

static XhciSimConfig DefaultQemuConfig() { return XhciSimConfig{}; }

constexpr EnumProfile kProfiles[] = {
    {"QemuMsix", "00:04.0", &DefaultQemuConfig, usb::IrqMode::kMsix, 5, 1},
    {"IntelMsi", "00:14.0", &IntelLikeConfig, usb::IrqMode::kMsi, 1, 13},
};

class XhciEnumProfileTest : public t::TestWithParam<EnumProfile> {};

TEST_P(XhciEnumProfileTest, EnumeratesHighSpeedKeyboardAndProgramsContexts) {
  const EnumProfile p = GetParam();
  SimWorld world;
  const int idx = world.AddController(p.label, p.make_config(), p.irq_mode);
  SimUsbKeyboard& kbd =
      world.AttachKeyboard(idx, p.usb2_port, HighSpeedKeyboardConfig());

  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  const std::string ready_line = std::string("[USBKBD] controller ") + p.label +
                                 " ready (" + usb::IrqModeName(p.irq_mode) +
                                 ")";
  EXPECT_THAT(world.platform().logs(), t::Contains(ready_line));

  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));

  const std::string attach_line = std::string("[USBKBD] keyboard attached: ") +
                                  p.label + " port " +
                                  std::to_string(p.usb2_port) + " (hs)";
  EXPECT_THAT(world.platform().logs(), t::Contains(attach_line));

  const XhciSim::SimSlot& slot = world.sim(idx).slot(1);
  EXPECT_TRUE(slot.enabled);
  EXPECT_THAT(slot.slot_state, t::Eq(3));  // Configured
  EXPECT_THAT(slot.ep[3].interval, t::Eq(3));
  EXPECT_THAT(slot.ep[3].max_packet_size, t::Eq(8));
  EXPECT_THAT(slot.ep[3].avg_trb_len, t::Eq(8));
  EXPECT_THAT(slot.ep[3].max_esit, t::Eq(8u));

  EXPECT_TRUE(kbd.configured());
  EXPECT_THAT(kbd.protocol(), t::Eq(0));  // 0 = Boot Protocol
  EXPECT_THAT(kbd.set_idle_count(), t::Ge(1));
  EXPECT_THAT(kbd.idle_rate_4ms(), t::Eq(0));
}

TEST_P(XhciEnumProfileTest,
       EnumeratesFullSpeedKeyboardsWithAndWithoutEp0MpsEvaluateContext) {
  const EnumProfile p = GetParam();
  // Case 1: Full-Speed device with bMaxPacketSize0 = 8 (matches initial
  // 8-byte FS default; no Evaluate Context needed).
  {
    SimWorld world;
    const int idx = world.AddController(p.label, p.make_config(), p.irq_mode);
    SimUsbKeyboard& kbd = world.AttachKeyboard(
        idx, p.usb2_port, FullSpeedKeyboardConfig(/*ep0_mps=*/8));

    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
    const XhciSim::SimSlot& slot = world.sim(idx).slot(1);
    EXPECT_THAT(slot.ep[1].max_packet_size, t::Eq(8));
    EXPECT_THAT(slot.ep[3].interval, t::Eq(6));
    world.TypeAscii(kbd, "fs");
    EXPECT_THAT(world.platform().input(), t::Eq("fs"));
  }
  // Case 2: Full-Speed device with bMaxPacketSize0 = 64 (differs from initial
  // 8-byte FS default; triggers Evaluate Context on EP0 before 18-byte read).
  {
    SimWorld world;
    const int idx = world.AddController(p.label, p.make_config(), p.irq_mode);
    world.AttachKeyboard(idx, p.usb2_port,
                         FullSpeedKeyboardConfig(/*ep0_mps=*/64));

    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
    EXPECT_THAT(world.sim(idx).slot(1).ep[1].max_packet_size, t::Eq(64));
  }
}

TEST_P(XhciEnumProfileTest, EnumeratesLowSpeedAndSuperSpeedKeyboards) {
  const EnumProfile p = GetParam();
  // Low-Speed keyboard on USB 2 port.
  {
    SimWorld world;
    const int idx = world.AddController(p.label, p.make_config(), p.irq_mode);
    world.AttachKeyboard(idx, p.usb2_port, LowSpeedKeyboardConfig());

    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
    const std::string attach_line =
        std::string("[USBKBD] keyboard attached: ") + p.label + " port " +
        std::to_string(p.usb2_port) + " (ls)";
    EXPECT_THAT(world.platform().logs(), t::Contains(attach_line));
    EXPECT_THAT(world.sim(idx).slot(1).ep[1].max_packet_size, t::Eq(8));
  }
  // SuperSpeed keyboard on USB 3 port (no PORTSC.PR issued, bMaxBurst=2).
  {
    SimWorld world;
    const int idx = world.AddController(p.label, p.make_config(), p.irq_mode);
    world.AttachKeyboard(idx, p.usb3_port,
                         SuperSpeedKeyboardConfig(/*max_burst=*/2));

    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
    const std::string attach_line =
        std::string("[USBKBD] keyboard attached: ") + p.label + " port " +
        std::to_string(p.usb3_port) + " (ss)";
    EXPECT_THAT(world.platform().logs(), t::Contains(attach_line));
    const XhciSim::SimSlot& slot = world.sim(idx).slot(1);
    EXPECT_THAT(slot.ep[1].max_packet_size, t::Eq(512));
    EXPECT_THAT(slot.ep[3].max_burst, t::Eq(2));
    EXPECT_THAT(slot.ep[3].max_esit, t::Eq(24u));
  }
}

TEST_P(XhciEnumProfileTest,
       EnumeratesCompositeDeviceAndRecoversFromSetProtocolAndSetIdleStalls) {
  const EnumProfile p = GetParam();
  for (const bool stall_on_setup : {true, false}) {
    SimWorld world;
    const int idx = world.AddController(p.label, p.make_config(), p.irq_mode);
    SimKeyboardConfig kcfg = CompositeKeyboardConfig();
    kcfg.stall_set_protocol = true;
    kcfg.stall_set_idle = true;
    kcfg.stall_on_setup_stage = stall_on_setup;
    SimUsbKeyboard& kbd = world.AttachKeyboard(idx, p.usb2_port, kcfg);

    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; }));

    const std::string attach_line =
        std::string("[USBKBD] keyboard attached: ") + p.label + " port " +
        std::to_string(p.usb2_port) + " (hs)";
    EXPECT_THAT(world.platform().logs(), t::Contains(attach_line));
    EXPECT_TRUE(world.sim(idx).slot(1).ep[5].doorbell_rung);
    EXPECT_THAT(kbd.last_protocol_interface(), t::Eq(1));
    EXPECT_THAT(kbd.last_idle_interface(), t::Eq(1));

    world.TypeAscii(kbd, "c");
    EXPECT_THAT(world.platform().input(), t::Eq("c"));
  }
}

TEST_P(XhciEnumProfileTest,
       IgnoresNonKeyboardDevicesAndRetriesTransientDescriptorErrors) {
  const EnumProfile p = GetParam();
  SimWorld world;
  const int idx = world.AddController(p.label, p.make_config(), p.irq_mode);

  // 1. Attach a Boot Mouse (protocol 2): slot should be disabled and parked.
  world.AttachKeyboard(idx, p.usb2_port, MouseOnlyDeviceConfig());
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  world.RunForMs(400);
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(0));

  // 2. Detach mouse and attach a USB Hub (class 9): disabled and parked.
  world.Detach(idx, p.usb2_port);
  world.RunForMs(50);
  world.AttachKeyboard(idx, p.usb2_port, HubDeviceConfig());
  world.RunForMs(400);
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(0));

  // 3. Detach hub and attach a keyboard that fails GET_DESCRIPTOR once then
  // succeeds on the 2nd attempt.
  world.Detach(idx, p.usb2_port);
  world.RunForMs(50);
  SimKeyboardConfig retry_cfg = HighSpeedKeyboardConfig();
  retry_cfg.fail_get_device_desc_attempts = 1;
  SimUsbKeyboard& kbd = world.AttachKeyboard(idx, p.usb2_port, retry_cfg);
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  world.TypeAscii(kbd, "r");
  EXPECT_THAT(world.platform().input(), t::Eq("r"));

  // 4. Detach and attach a keyboard that fails GET_DESCRIPTOR 10 times (> 3
  // retry budget): gives up and disables the slot cleanly.
  world.Detach(idx, p.usb2_port);
  world.RunForMs(50);
  SimKeyboardConfig always_fail_cfg = HighSpeedKeyboardConfig();
  always_fail_cfg.fail_get_device_desc_attempts = 10;
  world.AttachKeyboard(idx, p.usb2_port, always_fail_cfg);
  world.RunForMs(1500);
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(0));
}

TEST(XhciEnumTest,
     GarbageEventsNoSlotsAvailableAndControlTimeoutDoNotBreakActiveKeyboard) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  // Enable Slot fails with kCcNoSlotsAvailable when no slots are available.
  world.sim(idx).set_no_slots_available(true);
  world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  world.RunForMs(1500);
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));

  // Re-enable slots and re-attach on port 5: enumerates cleanly.
  world.Detach(idx, 5);
  world.ServiceReady();
  world.sim(idx).set_no_slots_available(false);
  SimUsbKeyboard& kbd = world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));

  // Attach a hanging device on port 6 whose control transfers never complete
  // (dropped), and verify it times out and frees its slot while port 5 types.
  SimKeyboardConfig hang_cfg = HighSpeedKeyboardConfig();
  hang_cfg.drop_control_transfer_attempts = 10;
  world.AttachKeyboard(idx, 6, hang_cfg);
  world.RunForMs(16000);
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(1));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(1));

  // Push malformed/out-of-bounds events (slot 0, slot 200, port 0, port 99,
  // DCI 31, unmapped TRB pointer, unknown completion code 33) and verify they
  // are ignored without affecting the active keyboard.
  usb::Trb bad_psc{};
  bad_psc.dw0 = 99u << 24;
  bad_psc.dw2 = static_cast<uint32_t>(usb::kCcSuccess) << 24;
  bad_psc.dw3 = static_cast<uint32_t>(usb::kTrbPortStatusChange)
                << usb::kTrbTypeShift;
  world.sim(idx).PushRawEvent(bad_psc);

  usb::Trb bad_xfer{};
  bad_xfer.dw0 = 0xDEAD'BEE0u;
  bad_xfer.dw2 = 33u << 24;
  bad_xfer.dw3 =
      (static_cast<uint32_t>(usb::kTrbTransferEvent) << usb::kTrbTypeShift) |
      (31u << 16) | (200u << 24);
  world.sim(idx).PushRawEvent(bad_xfer);
  world.ServiceReady();

  world.TypeAscii(kbd, "ok");
  EXPECT_THAT(world.platform().input(), t::Eq("ok"));
}

TEST(XhciEnumTest, StalledSetProtocolOnlyVersusStalledSetIdleOnlyRecovery) {
  // Case 1: Only SET_PROTOCOL stalls; SET_IDLE must succeed on the recovered
  // EP0 ring after Reset Endpoint + Set TR Dequeue Pointer.
  {
    SimWorld world;
    const int idx = world.AddController("00:04.0");
    SimKeyboardConfig cfg = HighSpeedKeyboardConfig();
    cfg.stall_set_protocol = true;
    cfg.stall_set_idle = false;
    SimUsbKeyboard& kbd = world.AttachKeyboard(idx, 5, cfg);

    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
    EXPECT_THAT(world.platform().logs(),
                t::Contains("[USBKBD] 00:04.0: port 5: SET_PROTOCOL stalled, "
                            "recovering ep0"));
    EXPECT_THAT(kbd.set_protocol_count(), t::Eq(1));
    EXPECT_THAT(kbd.set_idle_count(), t::Eq(1));
    EXPECT_THAT(kbd.idle_rate_4ms(), t::Eq(0));
    world.TypeAscii(kbd, "p");
    EXPECT_THAT(world.platform().input(), t::Eq("p"));
  }

  // Case 2: SET_PROTOCOL succeeds (protocol == 0), then only SET_IDLE stalls;
  // EP0 recovers via Reset Endpoint + Set TR Dequeue Pointer and finishes
  // keyboard attach.
  {
    SimWorld world;
    const int idx = world.AddController("00:04.0");
    SimKeyboardConfig cfg = HighSpeedKeyboardConfig();
    cfg.stall_set_protocol = false;
    cfg.stall_set_idle = true;
    SimUsbKeyboard& kbd = world.AttachKeyboard(idx, 5, cfg);

    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
    EXPECT_THAT(world.platform().logs(),
                t::Contains("[USBKBD] 00:04.0: port 5: SET_IDLE stalled, "
                            "recovering ep0"));
    EXPECT_THAT(kbd.protocol(), t::Eq(0));
    EXPECT_THAT(kbd.set_protocol_count(), t::Eq(1));
    EXPECT_THAT(kbd.set_idle_count(), t::Eq(1));
    world.TypeAscii(kbd, "i");
    EXPECT_THAT(world.platform().input(), t::Eq("i"));
  }

  // Case 3: Spurious STALL Transfer Event on a running interrupt endpoint
  // (where hardware endpoint state is still Running, not Halted) checks
  // EpContextState, issues Stop Endpoint + Set TR Dequeue Pointer +
  // CLEAR_FEATURE(ENDPOINT_HALT), and recovers in-place without detaching.
  {
    SimWorld world;
    const int idx = world.AddController("00:04.0");
    SimUsbKeyboard& kbd =
        world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; }));

    usb::Trb stall_intr{};
    stall_intr.dw2 = static_cast<uint32_t>(usb::kCcStallError) << 24;
    stall_intr.dw3 =
        (static_cast<uint32_t>(usb::kTrbTransferEvent) << usb::kTrbTypeShift) |
        (3u << 16) | (1u << 24);
    world.sim(idx).PushRawEvent(stall_intr);
    world.ServiceReady();

    EXPECT_THAT(world.platform().logs(),
                t::Contains("[USBKBD] 00:04.0: port 5: interrupt endpoint "
                            "error, recovering"));
    EXPECT_THAT(world.sim(idx).stop_endpoint_count(), t::Eq(1));
    EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(1));
    world.TypeAscii(kbd, "z");
    EXPECT_THAT(world.platform().input(), t::Eq("z"));
  }
}

class StageFaultUsbDevice final : public SimUsbDevice {
 public:
  enum class FaultStage {
    kGetDesc18Babble,
    kGetConfig9Babble,
    kGetConfigFullBabble,
    kSetConfigBabble,
    kSetProtocolBabble,
    kSetIdleBabble,
    kCorruptConfigHeader,
  };

  StageFaultUsbDevice(const FaultStage stage, const int fail_attempts)
      : inner_(HighSpeedKeyboardConfig()),
        stage_(stage),
        fail_attempts_(fail_attempts) {}

  int speed() const override { return inner_.speed(); }
  uint8_t ep0_max_packet_size() const override {
    return inner_.ep0_max_packet_size();
  }
  void OnBusReset() override {
    ++bus_resets_;
    inner_.OnBusReset();
  }
  void OnSetAddress(const uint8_t address) override {
    inner_.OnSetAddress(address);
  }
  uint8_t address() const override { return inner_.address(); }
  ControlOutcome OnControl(const SetupPacket& setup,  //
                           uint8_t* const data,       //
                           const uint16_t max_len,    //
                           uint16_t* const actual_len) override {
    if (failures_ < fail_attempts_) {
      if (stage_ == FaultStage::kGetDesc18Babble &&
          setup.bm_request_type == 0x80 && setup.b_request == 6 &&
          (setup.w_value >> 8) == 1 && max_len == 18) {
        ++failures_;
        return ControlOutcome::kBabble;
      }
      if (stage_ == FaultStage::kGetConfig9Babble &&
          setup.bm_request_type == 0x80 && setup.b_request == 6 &&
          (setup.w_value >> 8) == 2 && max_len == 9) {
        ++failures_;
        return ControlOutcome::kBabble;
      }
      if (stage_ == FaultStage::kGetConfigFullBabble &&
          setup.bm_request_type == 0x80 && setup.b_request == 6 &&
          (setup.w_value >> 8) == 2 && max_len > 9) {
        ++failures_;
        return ControlOutcome::kBabble;
      }
      if (stage_ == FaultStage::kSetConfigBabble &&
          setup.bm_request_type == 0x00 && setup.b_request == 9) {
        ++failures_;
        return ControlOutcome::kBabble;
      }
      if (stage_ == FaultStage::kSetProtocolBabble &&
          setup.bm_request_type == 0x21 && setup.b_request == 0x0B) {
        ++failures_;
        return ControlOutcome::kBabble;
      }
      if (stage_ == FaultStage::kSetIdleBabble &&
          setup.bm_request_type == 0x21 && setup.b_request == 0x0A) {
        ++failures_;
        return ControlOutcome::kBabble;
      }
    }
    const ControlOutcome res =
        inner_.OnControl(setup, data, max_len, actual_len);
    if (stage_ == FaultStage::kCorruptConfigHeader &&
        setup.bm_request_type == 0x80 && setup.b_request == 6 &&
        (setup.w_value >> 8) == 2 && max_len == 9 &&
        res == ControlOutcome::kAck) {
      // Corrupt bDescriptorType so ParseConfigDescriptorHeader rejects it.
      data[1] = 0xFFu;
    }
    return res;
  }
  IntrOutcome PollInterruptIn(const uint8_t ep_dci,        //
                              uint8_t* const buf,          //
                              const uint16_t max_len,      //
                              uint16_t* const actual_len,  //
                              const int64_t now_tick) override {
    return inner_.PollInterruptIn(ep_dci, buf, max_len, actual_len, now_tick);
  }

  SimUsbKeyboard& inner() { return inner_; }
  int bus_resets() const { return bus_resets_; }

 private:
  SimUsbKeyboard inner_;
  FaultStage stage_;
  int fail_attempts_ = 0;
  int failures_ = 0;
  int bus_resets_ = 0;
};

TEST(XhciEnumTest,
     BabbleAtEachLaterEnumerationStageRetriesAndDisablesSlotOnExhaustion) {
  using FaultStage = StageFaultUsbDevice::FaultStage;
  for (const FaultStage stage : {
           FaultStage::kGetDesc18Babble,
           FaultStage::kGetConfig9Babble,
           FaultStage::kGetConfigFullBabble,
           FaultStage::kSetConfigBabble,
           FaultStage::kSetProtocolBabble,
           FaultStage::kSetIdleBabble,
       }) {
    // 2 babble failures (< 3 retry limit) recover via DisableSlot + Port Reset
    // and enumerate on the 3rd attempt (1 attach + 1 HCRST + 3 port resets = 5
    // total OnBusReset notifications).
    {
      SimWorld world;
      const int idx = world.AddController("00:04.0");
      StageFaultUsbDevice dev(stage, /*fail_attempts=*/2);
      world.sim(idx).AttachDevice(5, &dev);
      ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
      ASSERT_TRUE(world.RunUntil(
          [&] { return world.controller(idx).attached_keyboard_count() == 1; },
          /*max_ticks=*/5000));
      EXPECT_THAT(dev.bus_resets(), t::Eq(5));
      EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(1));
      world.TypeAscii(dev.inner(), "k");
      EXPECT_THAT(world.platform().input(), t::Eq("k"));
    }

    // 3 babble failures exhaust the 3-attempt budget, issue DisableSlot, and
    // park the port in kDisabled with NextDeadline() == kNoDeadline.
    {
      SimWorld world;
      const int idx = world.AddController("00:04.0");
      StageFaultUsbDevice dev(stage, /*fail_attempts=*/3);
      world.sim(idx).AttachDevice(5, &dev);
      ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
      world.RunForMs(2000);
      EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));
      EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(0));
      EXPECT_THAT(world.controller(idx).NextDeadline(),
                  t::Eq(usb::kNoDeadline));
    }
  }

  // Corrupt 9-byte config descriptor header disables the slot immediately via
  // DisableUnsupportedDevice without retrying.
  {
    SimWorld world;
    const int idx = world.AddController("00:04.0");
    StageFaultUsbDevice bad_cfg(FaultStage::kCorruptConfigHeader, 0);
    world.sim(idx).AttachDevice(5, &bad_cfg);
    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    world.RunForMs(500);
    EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));
    EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(0));
    EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));
    // The line names the device by the IDs of its device descriptor (those
    // of QEMU's usb-kbd).
    EXPECT_THAT(world.platform().logs(),
                t::Contains("[USBKBD] 00:04.0: port 5: invalid config "
                            "descriptor header (0627:0001)"));
  }
}

TEST_P(XhciEnumProfileTest,
       EnumeratesUsb2HighSpeedHubAndDownstreamLsFsHsKeyboardsWithTtFields) {
  const EnumProfile& p = GetParam();
  SimWorld world;
  const int idx = world.AddController("00:04.0", p.make_config(), p.irq_mode);
  SimUsbHub& hub = world.AttachHub(
      idx, p.usb2_port, Usb2HighSpeedSingleTtHubConfig(/*num_ports=*/4, 2));
  SimUsbKeyboard& kbd_ls =
      world.AttachKeyboardToHub(hub, 1, LowSpeedKeyboardConfig());
  SimUsbKeyboard& kbd_fs =
      world.AttachKeyboardToHub(hub, 2, FullSpeedKeyboardConfig(64));
  SimUsbKeyboard& kbd_hs =
      world.AttachKeyboardToHub(hub, 4, HighSpeedKeyboardConfig());

  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 3; }));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(4));
  EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));

  // Slot 1 is the High-Speed Single-TT Hub on root port p.usb2_port.
  const XhciSim::SimSlot& hub_slot = world.sim(idx).slot(1);
  EXPECT_TRUE(hub_slot.enabled);
  EXPECT_TRUE(hub_slot.hub);
  EXPECT_FALSE(hub_slot.mtt);
  EXPECT_THAT(hub_slot.num_ports, t::Eq(4));
  EXPECT_THAT(hub_slot.ttt, t::Eq(2));
  EXPECT_THAT(hub_slot.port, t::Eq(p.usb2_port));
  EXPECT_THAT(hub_slot.route_string, t::Eq(0u));
  EXPECT_THAT(hub_slot.parent_hub_slot_id, t::Eq(0));
  EXPECT_THAT(hub_slot.parent_port_number, t::Eq(0));

  // Verify downstream LS (port 1), FS (port 2), and HS (port 4) keyboard slots.
  const XhciSim::SimSlot& ls_slot = world.sim(idx).slot(2);
  EXPECT_THAT(ls_slot.port, t::Eq(p.usb2_port));
  EXPECT_THAT(ls_slot.route_string, t::Eq(0x00001u));
  EXPECT_THAT(ls_slot.parent_hub_slot_id, t::Eq(1));
  EXPECT_THAT(ls_slot.parent_port_number, t::Eq(1));

  const XhciSim::SimSlot& fs_slot = world.sim(idx).slot(3);
  EXPECT_THAT(fs_slot.port, t::Eq(p.usb2_port));
  EXPECT_THAT(fs_slot.route_string, t::Eq(0x00002u));
  EXPECT_THAT(fs_slot.parent_hub_slot_id, t::Eq(1));
  EXPECT_THAT(fs_slot.parent_port_number, t::Eq(2));
  EXPECT_THAT(fs_slot.ep[1].max_packet_size, t::Eq(64));

  const XhciSim::SimSlot& hs_slot = world.sim(idx).slot(4);
  EXPECT_THAT(hs_slot.port, t::Eq(p.usb2_port));
  EXPECT_THAT(hs_slot.route_string, t::Eq(0x00004u));
  EXPECT_THAT(hs_slot.parent_hub_slot_id, t::Eq(0));
  EXPECT_THAT(hs_slot.parent_port_number, t::Eq(0));

  EXPECT_THAT(world.platform().logs(),
              t::Contains("[USBKBD] keyboard attached: 00:04.0 port " +
                          std::to_string(p.usb2_port) + ".1 (ls)"));
  EXPECT_THAT(world.platform().logs(),
              t::Contains("[USBKBD] keyboard attached: 00:04.0 port " +
                          std::to_string(p.usb2_port) + ".2 (fs)"));
  EXPECT_THAT(world.platform().logs(),
              t::Contains("[USBKBD] keyboard attached: 00:04.0 port " +
                          std::to_string(p.usb2_port) + ".4 (hs)"));

  world.TypeAscii(kbd_ls, "l");
  world.TypeAscii(kbd_fs, "f");
  world.TypeAscii(kbd_hs, "h");
  EXPECT_THAT(world.platform().input(), t::Eq("lfh"));
}

TEST_P(XhciEnumProfileTest,
       EnumeratesUsb3SuperSpeedHubWithSetHubDepthAndDownstreamSsKeyboard) {
  const EnumProfile& p = GetParam();
  SimWorld world;
  const int idx = world.AddController("00:04.0", p.make_config(), p.irq_mode);
  SimUsbHub& hub1 =
      world.AttachHub(idx, p.usb3_port, Usb3SuperSpeedHubConfig(4));
  SimUsbHub& hub2 = world.AttachHubToHub(hub1, 2, Usb3SuperSpeedHubConfig(4));
  SimUsbKeyboard& kbd_ss =
      world.AttachKeyboardToHub(hub2, 3, SuperSpeedKeyboardConfig(2));

  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(3));
  EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));

  EXPECT_THAT(hub1.set_hub_depth_count(), t::Eq(1));
  EXPECT_THAT(hub1.hub_depth(), t::Eq(0));
  EXPECT_THAT(hub2.set_hub_depth_count(), t::Eq(1));
  EXPECT_THAT(hub2.hub_depth(), t::Eq(1));

  const XhciSim::SimSlot& hub2_slot = world.sim(idx).slot(2);
  EXPECT_TRUE(hub2_slot.hub);
  EXPECT_THAT(hub2_slot.port, t::Eq(p.usb3_port));
  EXPECT_THAT(hub2_slot.route_string, t::Eq(0x00002u));

  const XhciSim::SimSlot& kbd_slot = world.sim(idx).slot(3);
  EXPECT_FALSE(kbd_slot.hub);
  EXPECT_THAT(kbd_slot.port, t::Eq(p.usb3_port));
  EXPECT_THAT(kbd_slot.route_string, t::Eq(0x00032u));
  EXPECT_THAT(kbd_slot.parent_hub_slot_id, t::Eq(0));
  EXPECT_THAT(kbd_slot.parent_port_number, t::Eq(0));
  EXPECT_THAT(kbd_slot.ep[3].max_burst, t::Eq(2));

  world.TypeAscii(kbd_ss, "s3");
  EXPECT_THAT(world.platform().input(), t::Eq("s3"));
}

TEST(
    XhciEnumTest,
    PropagatesHighSpeedTtThroughIntermediateFullSpeedHubAndMultiTierRouteStrings) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  // Root port 5: High-Speed Multi-TT Hub that reports multi-TT on alternate
  // setting 0 (so the driver programs MTT) -> port 3: Full-Speed Hub -> port
  // 2: Low-Speed Keyboard.
  SimUsbHub& hs_hub = world.AttachHub(
      idx, 5, Usb2HighSpeedMultiTtOnAlt0HubConfig(4, /*ttt=*/1));
  SimUsbHub& fs_hub =
      world.AttachHubToHub(hs_hub, 3, Usb2FullSpeedHubConfig(4));
  SimUsbKeyboard& ls_kbd =
      world.AttachKeyboardToHub(fs_hub, 2, LowSpeedKeyboardConfig());

  // Root port 6: Full-Speed Hub directly on root port -> port 1: Full-Speed
  // Keyboard (no HS ancestor in chain, so TT fields must remain 0).
  SimUsbHub& root_fs_hub = world.AttachHub(idx, 6, Usb2FullSpeedHubConfig(4));
  SimUsbKeyboard& root_fs_kbd =
      world.AttachKeyboardToHub(root_fs_hub, 1, FullSpeedKeyboardConfig(64));

  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 2; }));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(5));
  EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));

  // Find the slots for hs_hub, fs_hub, ls_kbd on root port 5 and root_fs_kbd on
  // root port 6.
  uint8_t hs_hub_slot_id = 0;
  for (int s = 1; s <= 16; ++s) {
    const XhciSim::SimSlot& sl = world.sim(idx).slot(s);
    if (sl.enabled && sl.port == 5 && sl.route_string == 0u) {
      hs_hub_slot_id = static_cast<uint8_t>(s);
      EXPECT_TRUE(sl.hub);
      EXPECT_TRUE(sl.mtt);
      EXPECT_THAT(sl.ttt, t::Eq(1));
    }
  }
  ASSERT_THAT(hs_hub_slot_id, t::Gt(0));

  for (int s = 1; s <= 16; ++s) {
    const XhciSim::SimSlot& sl = world.sim(idx).slot(s);
    if (!sl.enabled) {
      continue;
    }
    if (sl.port == 5 && sl.route_string == 0x00003u) {
      // Intermediate FS hub directly behind HS Multi-TT hub port 3.
      EXPECT_TRUE(sl.hub);
      EXPECT_TRUE(sl.mtt);
      EXPECT_THAT(sl.parent_hub_slot_id, t::Eq(hs_hub_slot_id));
      EXPECT_THAT(sl.parent_port_number, t::Eq(3));
    } else if (sl.port == 5 && sl.route_string == 0x00023u) {
      // LS keyboard behind FS hub port 2 inherits TT from hs_hub port 3!
      EXPECT_FALSE(sl.hub);
      EXPECT_TRUE(sl.mtt);
      EXPECT_THAT(sl.parent_hub_slot_id, t::Eq(hs_hub_slot_id));
      EXPECT_THAT(sl.parent_port_number, t::Eq(3));
    } else if (sl.port == 6 && sl.route_string == 0x00001u) {
      // FS keyboard behind root-attached FS hub has no HS TT ancestor.
      EXPECT_FALSE(sl.hub);
      EXPECT_FALSE(sl.mtt);
      EXPECT_THAT(sl.parent_hub_slot_id, t::Eq(0));
      EXPECT_THAT(sl.parent_port_number, t::Eq(0));
    }
  }

  world.TypeAscii(ls_kbd, "1");
  world.TypeAscii(root_fs_kbd, "2");
  EXPECT_THAT(world.platform().input(), t::Eq("12"));
}

TEST(
    XhciEnumTest,
    RecoversFromTransientHubDescriptorAndPortStatusErrorsAndDisablesCorruptHub) {
  {
    SimWorld world;
    const int idx = world.AddController("00:04.0");
    SimHubConfig hub_cfg = Usb3SuperSpeedHubConfig(4);
    hub_cfg.fail_get_hub_desc_attempts = 1;
    hub_cfg.fail_get_port_status_attempts = 1;
    hub_cfg.port_reset_delay_polls = 2;
    hub_cfg.stall_set_hub_depth = true;
    SimUsbHub& hub = world.AttachHub(idx, 1, hub_cfg);
    SimUsbKeyboard& kbd =
        world.AttachKeyboardToHub(hub, 2, SuperSpeedKeyboardConfig());

    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
    EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));
    world.TypeAscii(kbd, "r");
    EXPECT_THAT(world.platform().input(), t::Eq("r"));
  }

  // Corrupt hub descriptor (bNbrPorts = 0) disables the hub slot cleanly.
  {
    SimWorld world;
    const int idx = world.AddController("00:04.0");
    SimHubConfig bad_hub = Usb2HighSpeedSingleTtHubConfig(4);
    bad_hub.corrupt_hub_descriptor = true;
    world.AttachHub(idx, 5, bad_hub);

    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    world.RunForMs(600);
    EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(0));
    EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(0));
    EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));
  }
}

static bool HasLogLine(const SimWorld& world, const std::string& line) {
  for (const std::string& logged : world.platform().logs()) {
    if (logged == line) {
      return true;
    }
  }
  return false;
}

TEST_P(XhciEnumProfileTest,
       HubLevelStatusChangesAreAcknowledgedAndHubStaysIdle) {
  const EnumProfile& p = GetParam();
  struct Case {
    int root_port;
    SimHubConfig hub;
    SimKeyboardConfig kbd;
  };
  const Case cases[] = {
      {p.usb2_port, Usb2HighSpeedSingleTtHubConfig(4),
       FullSpeedKeyboardConfig(64)},
      {p.usb3_port, Usb3SuperSpeedHubConfig(4), SuperSpeedKeyboardConfig()},
  };
  for (const Case& tc : cases) {
    SCOPED_TRACE("root port " + std::to_string(tc.root_port));
    SimWorld world;
    const int idx = world.AddController(p.label, p.make_config(), p.irq_mode);
    SimUsbHub& hub = world.AttachHub(idx, tc.root_port, tc.hub);
    SimUsbKeyboard& kbd = world.AttachKeyboardToHub(hub, 1, tc.kbd);
    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    ASSERT_TRUE(world.RunUntil([&] {
      return world.controller(idx).attached_keyboard_count() == 1 &&
             world.controller(idx).NextDeadline() == usb::kNoDeadline;
    }));
    const std::string hub_line = std::string("[USBKBD] ") + p.label +
                                 ": port " + std::to_string(tc.root_port) +
                                 ": ";

    // C_HUB_LOCAL_POWER alone (bitmap bit 0) is acknowledged with one
    // GET_HUB_STATUS and one CLEAR_HUB_FEATURE, without scanning the ports.
    const int reports0 = hub.intr_data_count();
    const int port_reads0 = hub.get_port_status_count();
    hub.InjectHubChange(0x0001);
    ASSERT_TRUE(world.RunUntil([&] { return hub.hub_change() == 0; }, 125));
    ASSERT_TRUE(world.RunUntilIdle(idx));
    EXPECT_THAT(hub.get_hub_status_count(), t::Eq(1));
    EXPECT_THAT(hub.clear_hub_feature_count(), t::Eq(1));
    EXPECT_THAT(hub.get_port_status_count(), t::Eq(port_reads0));
    EXPECT_THAT(hub.intr_data_count() - reports0, t::Le(2));

    // Both hub change bits at once: one CLEAR_HUB_FEATURE each.
    hub.InjectHubChange(0x0003);
    ASSERT_TRUE(world.RunUntil([&] { return hub.hub_change() == 0; }, 125));
    ASSERT_TRUE(world.RunUntilIdle(idx));
    EXPECT_THAT(hub.get_hub_status_count(), t::Eq(2));
    EXPECT_THAT(hub.clear_hub_feature_count(), t::Eq(3));

    // An over-current that persists is acknowledged and logged once; the
    // status bit stays set, but with its change cleared the hub stays quiet.
    hub.SetHubStatus(0x0002);
    ASSERT_TRUE(world.RunUntil([&] { return hub.hub_change() == 0; }, 125));
    ASSERT_TRUE(world.RunUntilIdle(idx));
    EXPECT_THAT(hub.hub_status(), t::Eq(0x0002));
    EXPECT_TRUE(HasLogLine(world, hub_line + "hub over-current"));
    const int reports1 = hub.intr_data_count();
    const int port_reads1 = hub.get_port_status_count();
    const int64_t wakeups1 = world.wakeups();
    world.RunForMs(2000);
    EXPECT_THAT(hub.intr_data_count(), t::Eq(reports1));
    EXPECT_THAT(hub.get_port_status_count(), t::Eq(port_reads1));
    EXPECT_THAT(world.wakeups(), t::Eq(wakeups1));
    EXPECT_THAT(world.controller(idx).NextDeadline(), t::Eq(usb::kNoDeadline));

    // The over-current ends while a keyboard is plugged into port 3: the hub
    // and port changes arrive in one bitmap and are both serviced.
    hub.SetHubStatus(0);
    SimUsbKeyboard& kbd2 = world.AttachKeyboardToHub(hub, 3, tc.kbd);
    ASSERT_TRUE(world.RunUntil([&] {
      return world.controller(idx).attached_keyboard_count() == 2 &&
             world.controller(idx).NextDeadline() == usb::kNoDeadline;
    }));
    EXPECT_THAT(hub.hub_change(), t::Eq(0));
    EXPECT_TRUE(HasLogLine(world, hub_line + "hub over-current cleared"));
    EXPECT_THAT(hub.redundant_status_reports(), t::Eq(0));
    world.TypeAscii(kbd, "o");
    world.TypeAscii(kbd2, "k");
    EXPECT_THAT(world.platform().input(), t::Eq("ok"));
  }
}

TEST(XhciEnumTest, FiveUsb2HubTiersUseAllRouteNibblesAndRefuseSixthHub) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  // Tier 1 is High-Speed; tiers 2..5 are Full-Speed hubs, so the TT of the
  // Low-Speed keyboard is inherited through four hubs.
  SimUsbHub& h1 = world.AttachHub(idx, 5, Usb2HighSpeedSingleTtHubConfig(7));
  SimUsbHub& h2 = world.AttachHubToHub(h1, 1, Usb2FullSpeedHubConfig(7));
  SimUsbHub& h3 = world.AttachHubToHub(h2, 2, Usb2FullSpeedHubConfig(7));
  SimUsbHub& h4 = world.AttachHubToHub(h3, 3, Usb2FullSpeedHubConfig(7));
  SimUsbHub& h5 = world.AttachHubToHub(h4, 4, Usb2FullSpeedHubConfig(7));
  SimUsbKeyboard& kbd =
      world.AttachKeyboardToHub(h5, 5, LowSpeedKeyboardConfig());
  SimUsbHub& h6 = world.AttachHubToHub(h5, 6, Usb2FullSpeedHubConfig(4));
  world.AttachKeyboardToHub(h6, 1, FullSpeedKeyboardConfig(8));

  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  const std::string refusal =
      "[USBKBD] 00:04.0: port 5.1.2.3.4.6: max hub depth exceeded (0bda:0411)";
  ASSERT_TRUE(world.RunUntil(
      [&] {
        return world.controller(idx).attached_keyboard_count() == 1 &&
               HasLogLine(world, refusal) &&
               world.controller(idx).NextDeadline() == usb::kNoDeadline;
      },
      5000));
  EXPECT_FALSE(h6.configured());
  EXPECT_THAT(h6.set_config_count(), t::Eq(0));
  EXPECT_FALSE(h6.port(1).powered);
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(6));
  EXPECT_TRUE(HasLogLine(
      world, "[USBKBD] keyboard attached: 00:04.0 port 5.1.2.3.4.5 (ls)"));

  const int h1_slot = world.FindSlot(idx, 5, 0x0u);
  const int h5_slot = world.FindSlot(idx, 5, 0x4321u);
  const int kbd_slot = world.FindSlot(idx, 5, 0x54321u);
  ASSERT_THAT(h1_slot, t::Gt(0));
  ASSERT_THAT(h5_slot, t::Gt(0));
  ASSERT_THAT(kbd_slot, t::Gt(0));
  const XhciSim::SimSlot& ks = world.sim(idx).slot(kbd_slot);
  EXPECT_FALSE(ks.hub);
  EXPECT_FALSE(ks.mtt);
  EXPECT_THAT(ks.parent_hub_slot_id, t::Eq(h1_slot));
  EXPECT_THAT(ks.parent_port_number, t::Eq(1));
  const XhciSim::SimSlot& h5s = world.sim(idx).slot(h5_slot);
  EXPECT_TRUE(h5s.hub);
  EXPECT_THAT(h5s.num_ports, t::Eq(7));
  EXPECT_THAT(h5s.ttt, t::Eq(0));
  EXPECT_THAT(h5s.parent_hub_slot_id, t::Eq(h1_slot));
  EXPECT_THAT(h5s.parent_port_number, t::Eq(1));
  world.TypeAscii(kbd, "5");
  EXPECT_THAT(world.platform().input(), t::Eq("5"));

  // Unplug the refused hub, then attach a keyboard to the 5th tier: nothing
  // of the refused hub is left behind.
  h5.DetachDevice(6);
  ASSERT_TRUE(world.RunUntilIdle(idx));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(6));
  SimUsbKeyboard& late =
      world.AttachKeyboardToHub(h5, 7, FullSpeedKeyboardConfig(8));
  ASSERT_TRUE(world.RunUntil([&] {
    return world.controller(idx).attached_keyboard_count() == 2 &&
           world.controller(idx).NextDeadline() == usb::kNoDeadline;
  }));
  EXPECT_THAT(world.FindSlot(idx, 5, 0x74321u), t::Gt(0));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(7));
  world.TypeAscii(late, "7");
  world.TypeAscii(kbd, "8");
  EXPECT_THAT(world.platform().input(), t::Eq("578"));
  EXPECT_THAT(world.sim(idx).CountViolations(Violation::kUsbAddressCollision),
              t::Eq(0));
}

TEST(XhciEnumTest,
     FiveSuperSpeedHubTiersSetHubDepthZeroToFourAndRefuseSixthHub) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbHub& h1 = world.AttachHub(idx, 1, Usb3SuperSpeedHubConfig(7));
  SimUsbHub& h2 = world.AttachHubToHub(h1, 7, Usb3SuperSpeedHubConfig(7));
  SimUsbHub& h3 = world.AttachHubToHub(h2, 6, Usb3SuperSpeedHubConfig(7));
  SimUsbHub& h4 = world.AttachHubToHub(h3, 5, Usb3SuperSpeedHubConfig(7));
  SimUsbHub& h5 = world.AttachHubToHub(h4, 4, Usb3SuperSpeedHubConfig(7));
  SimUsbKeyboard& kbd =
      world.AttachKeyboardToHub(h5, 3, SuperSpeedKeyboardConfig());
  SimUsbHub& h6 = world.AttachHubToHub(h5, 2, Usb3SuperSpeedHubConfig(4));

  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  const std::string refusal =
      "[USBKBD] 00:04.0: port 1.7.6.5.4.2: max hub depth exceeded (0bda:0411)";
  ASSERT_TRUE(world.RunUntil(
      [&] {
        return world.controller(idx).attached_keyboard_count() == 1 &&
               HasLogLine(world, refusal) &&
               world.controller(idx).NextDeadline() == usb::kNoDeadline;
      },
      5000));
  EXPECT_THAT(h1.hub_depth(), t::Eq(0));
  EXPECT_THAT(h2.hub_depth(), t::Eq(1));
  EXPECT_THAT(h3.hub_depth(), t::Eq(2));
  EXPECT_THAT(h4.hub_depth(), t::Eq(3));
  EXPECT_THAT(h5.hub_depth(), t::Eq(4));
  EXPECT_THAT(h6.set_hub_depth_count(), t::Eq(0));
  EXPECT_FALSE(h6.configured());
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(6));
  EXPECT_THAT(world.FindSlot(idx, 1, 0x34567u), t::Gt(0));
  world.TypeAscii(kbd, "s");
  EXPECT_THAT(world.platform().input(), t::Eq("s"));
}

TEST(XhciEnumTest, CompliantMultiTtHubRunsSingleTtWithoutSetInterface) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  // A compliant multi-TT hub runs single-TT in alternate setting 0; the
  // driver keeps it there, so MTT stays 0 for the hub and its children.
  SimUsbHub& hub =
      world.AttachHub(idx, 5, Usb2HighSpeedMultiTtHubConfig(4, /*ttt=*/1));
  SimUsbKeyboard& fs_kbd =
      world.AttachKeyboardToHub(hub, 1, FullSpeedKeyboardConfig(64));
  SimUsbHub& fs_hub = world.AttachHubToHub(hub, 3, Usb2FullSpeedHubConfig(4));
  SimUsbKeyboard& ls_kbd =
      world.AttachKeyboardToHub(fs_hub, 2, LowSpeedKeyboardConfig());

  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil([&] {
    return world.controller(idx).attached_keyboard_count() == 2 &&
           world.controller(idx).NextDeadline() == usb::kNoDeadline;
  }));
  const int hub_slot = world.FindSlot(idx, 5, 0x0u);
  ASSERT_THAT(hub_slot, t::Gt(0));
  const XhciSim::SimSlot& hs = world.sim(idx).slot(hub_slot);
  EXPECT_TRUE(hs.hub);
  EXPECT_FALSE(hs.mtt);
  EXPECT_THAT(hs.ttt, t::Eq(1));
  EXPECT_THAT(hs.num_ports, t::Eq(4));
  struct Child {
    uint32_t route;
    bool hub;
    uint8_t tt_port;
  };
  const Child children[] = {
      {0x1u, false, 1},   // Full-Speed keyboard on port 1
      {0x3u, true, 3},    // Full-Speed hub on port 3
      {0x23u, false, 3},  // Low-Speed keyboard behind it
  };
  for (const Child& c : children) {
    SCOPED_TRACE("route " + std::to_string(c.route));
    const int s = world.FindSlot(idx, 5, c.route);
    ASSERT_THAT(s, t::Gt(0));
    const XhciSim::SimSlot& sl = world.sim(idx).slot(s);
    EXPECT_THAT(sl.hub, t::Eq(c.hub));
    EXPECT_FALSE(sl.mtt);
    EXPECT_THAT(sl.parent_hub_slot_id, t::Eq(hub_slot));
    EXPECT_THAT(sl.parent_port_number, t::Eq(c.tt_port));
  }
  EXPECT_THAT(hub.set_interface_count(), t::Eq(0));
  EXPECT_THAT(hub.alt_setting(), t::Eq(0));
  world.TypeAscii(fs_kbd, "f");
  world.TypeAscii(ls_kbd, "l");
  EXPECT_THAT(world.platform().input(), t::Eq("fl"));
}

TEST(XhciEnumTest,
     AbandonedDeviceOnUsb2HubPortIsDisabledBeforeTheNextEnumeration) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbHub& hub = world.AttachHub(idx, 5, Usb2HighSpeedSingleTtHubConfig(4));
  // A Low-Speed mouse on port 1 is enumerated first and rejected; its slot
  // (and so its USB address) is reused for the keyboard on port 2.
  SimKeyboardConfig mouse_cfg = MouseOnlyDeviceConfig();
  mouse_cfg.speed = 2;
  mouse_cfg.ep0_max_packet_size = 8;
  mouse_cfg.intr_b_interval = 10;
  world.AttachKeyboardToHub(hub, 1, mouse_cfg);
  SimUsbKeyboard& kbd =
      world.AttachKeyboardToHub(hub, 2, LowSpeedKeyboardConfig());

  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil([&] {
    return world.controller(idx).attached_keyboard_count() == 1 &&
           world.controller(idx).NextDeadline() == usb::kNoDeadline;
  }));
  EXPECT_TRUE(HasLogLine(
      world,
      "[USBKBD] 00:04.0: port 5.1: no boot keyboard interface (0627:0001)"));
  // The abandoned mouse sits on a disabled port, so only the keyboard
  // answers the reused address.
  EXPECT_THAT(hub.port(1).status & 0x0002u, t::Eq(0u));
  EXPECT_THAT(world.sim(idx).CountViolations(Violation::kUsbAddressCollision),
              t::Eq(0));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(2));
  world.TypeAscii(kbd, "m");
  EXPECT_THAT(world.platform().input(), t::Eq("m"));

  // A keyboard hot-plugged on port 3 attaches without a collision, too.
  SimUsbKeyboard& kbd3 =
      world.AttachKeyboardToHub(hub, 3, FullSpeedKeyboardConfig(64));
  ASSERT_TRUE(world.RunUntil([&] {
    return world.controller(idx).attached_keyboard_count() == 2 &&
           world.controller(idx).NextDeadline() == usb::kNoDeadline;
  }));
  world.TypeAscii(kbd3, "n");
  EXPECT_THAT(world.platform().input(), t::Eq("mn"));
  EXPECT_THAT(world.sim(idx).CountViolations(Violation::kUsbAddressCollision),
              t::Eq(0));
}

TEST(XhciEnumTest, HubDescriptorIsReadOnlyAfterSetConfiguration) {
  struct Case {
    int root_port;
    SimHubConfig hub;
    SimKeyboardConfig kbd;
  };
  const Case cases[] = {
      {5, Usb2HighSpeedSingleTtHubConfig(4), LowSpeedKeyboardConfig()},
      {1, Usb3SuperSpeedHubConfig(4), SuperSpeedKeyboardConfig()},
  };
  for (const Case& tc : cases) {
    SCOPED_TRACE("root port " + std::to_string(tc.root_port));
    SimWorld world;
    const int idx = world.AddController("00:04.0");
    // USB 2.0 11.24.2.5: before SET_CONFIGURATION the response to a hub
    // class request is undefined; this hub STALLs them.
    SimHubConfig hub_cfg = tc.hub;
    hub_cfg.stall_class_requests_until_configured = true;
    SimUsbHub& hub = world.AttachHub(idx, tc.root_port, hub_cfg);
    SimUsbKeyboard& kbd = world.AttachKeyboardToHub(hub, 2, tc.kbd);

    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    ASSERT_TRUE(world.RunUntil([&] {
      return world.controller(idx).attached_keyboard_count() == 1 &&
             world.controller(idx).NextDeadline() == usb::kNoDeadline;
    }));
    EXPECT_THAT(hub.class_requests_before_config(), t::Eq(0));
    EXPECT_THAT(hub.get_hub_desc_count(), t::Eq(1));
    EXPECT_THAT(hub.set_config_count(), t::Eq(1));
    if (tc.hub.speed >= 4) {
      EXPECT_THAT(hub.set_hub_depth_count(), t::Eq(1));
    }
    EXPECT_THAT(world.platform().logs(),
                t::Not(t::Contains(t::HasSubstr("get hub descriptor error"))));
    // Configure Endpoint still carried the hub fields.
    const int hub_slot = world.FindSlot(idx, tc.root_port, 0x0u);
    ASSERT_THAT(hub_slot, t::Gt(0));
    EXPECT_TRUE(world.sim(idx).slot(hub_slot).hub);
    EXPECT_THAT(world.sim(idx).slot(hub_slot).num_ports, t::Eq(4));
    world.TypeAscii(kbd, "c");
    EXPECT_THAT(world.platform().input(), t::Eq("c"));
  }
}

TEST(XhciEnumTest,
     RetryingHubPortIsDisabledDuringItsBackoffSoAddressesStayUnique) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbHub& hub = world.AttachHub(idx, 5, Usb2HighSpeedSingleTtHubConfig(4));
  // The keyboard on port 1 fails its first GET_DESCRIPTOR after it was
  // addressed; port 1 retries after a back-off while port 2 enumerates.
  SimKeyboardConfig k1_cfg = LowSpeedKeyboardConfig();
  k1_cfg.fail_get_device_desc_attempts = 1;
  SimUsbKeyboard& k1 = world.AttachKeyboardToHub(hub, 1, k1_cfg);
  SimUsbKeyboard& k2 =
      world.AttachKeyboardToHub(hub, 2, LowSpeedKeyboardConfig());

  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] {
        return world.controller(idx).attached_keyboard_count() == 2 &&
               world.controller(idx).NextDeadline() == usb::kNoDeadline;
      },
      3000));
  EXPECT_THAT(world.sim(idx).CountViolations(Violation::kUsbAddressCollision),
              t::Eq(0));
  EXPECT_THAT(
      world.sim(idx).CountViolations(Violation::kDefaultAddressCollision),
      t::Eq(0));
  EXPECT_TRUE(HasLogLine(world,
                         "[USBKBD] 00:04.0: port 5.1: get device descriptor "
                         "prefix error (attempt 1)"));
  world.TypeAscii(k1, "1");
  world.TypeAscii(k2, "2");
  EXPECT_THAT(world.platform().input(), t::Eq("12"));
}

TEST(
    XhciEnumTest,
    Usb3RootPortRetryResetsAddressedDeviceAndRecoversInactiveLinkViaWarmReset) {
  // 1. A SuperSpeed device on USB 3 port 1 that fails GET_DESCRIPTOR after
  // Address Device is Hot-Reset (PORTSC.PR) on retry even though PED=1,
  // returning it to USB address 0 so the retry Address Device succeeds.
  {
    SimWorld world;
    const int idx = world.AddController("00:04.0");
    SimKeyboardConfig cfg = SuperSpeedKeyboardConfig();
    cfg.fail_get_device_desc_attempts = 1;
    SimUsbKeyboard& kbd = world.AttachKeyboard(idx, 1, cfg);
    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; },
        3000));
    EXPECT_THAT(kbd.bus_reset_count(), t::Ge(3));
    EXPECT_THAT(world.sim(idx).violations(), t::IsEmpty());
    world.TypeAscii(kbd, "u");
    EXPECT_THAT(world.platform().input(), t::Eq("u"));
  }

  // 2. A USB 3 port in SS.Inactive recovers via Warm Port Reset (PORTSC.WPR).
  {
    SimWorld world;
    const int idx = world.AddController("00:04.0");
    ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
    SimUsbKeyboard& kbd =
        world.AttachKeyboard(idx, 1, SuperSpeedKeyboardConfig());
    world.sim(idx).SetUsb3LinkInactive(1);
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; },
        3000));
    EXPECT_THAT(world.sim(idx).violations(), t::IsEmpty());
    world.TypeAscii(kbd, "w");
    EXPECT_THAT(world.platform().input(), t::Eq("w"));
  }
}

TEST(XhciEnumTest, Hci096ControllerUsesTrtZeroOnSetupStageTrbs) {
  XhciSimConfig cfg;
  cfg.hci_version = 0x0096;
  SimWorld world;
  const int idx = world.AddController("00:04.0", cfg);
  SimUsbKeyboard& kbd = world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  EXPECT_THAT(world.sim(idx).violations(), t::IsEmpty());
  world.TypeAscii(kbd, "9");
  EXPECT_THAT(world.platform().input(), t::Eq("9"));
}

TEST(XhciEnumTest,
     StuckAddressDeviceAbortsCommandWithoutResettingControllerOrOtherPorts) {
  SimWorld world;
  const int idx = world.AddController("00:04.0");
  SimUsbKeyboard& good_kbd =
      world.AttachKeyboard(idx, 5, HighSpeedKeyboardConfig());
  ASSERT_THAT(world.BringUp(idx), t::Eq(usb::XhciStatus::kOk));
  ASSERT_TRUE(world.RunUntil(
      [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
  const int good_resets = good_kbd.bus_reset_count();

  // Plug a broken device on port 6 that NAKs SET_ADDRESS forever, hanging
  // Address Device on the Command Ring. Each 5 s command timeout aborts only
  // that command via CRCR.CA and fails port 6 without resetting the controller.
  SimKeyboardConfig bad_cfg = HighSpeedKeyboardConfig();
  bad_cfg.nak_set_address = true;
  world.AttachKeyboard(idx, 6, bad_cfg);
  world.RunForMs(20000);

  EXPECT_THAT(world.controller(idx).state(), t::Eq(usb::XhciState::kRunning));
  EXPECT_THAT(world.controller(idx).attached_keyboard_count(), t::Eq(1));
  EXPECT_THAT(good_kbd.bus_reset_count(), t::Eq(good_resets));
  EXPECT_THAT(world.sim(idx).enabled_slot_count(), t::Eq(1));
  EXPECT_TRUE(
      HasLogLine(world, "[USBKBD] 00:04.0: command timeout, aborting it"));
  EXPECT_TRUE(HasLogLine(
      world, "[USBKBD] 00:04.0: port 6: command timeout (attempt 3)"));
  world.TypeAscii(good_kbd, "g");
  EXPECT_THAT(world.platform().input(), t::Eq("g"));
}

INSTANTIATE_TEST_SUITE_P(Profiles, XhciEnumProfileTest, t::ValuesIn(kProfiles),
                         [](const t::TestParamInfo<EnumProfile>& info) {
                           return info.param.name;
                         });

}  // namespace
}  // namespace protos::xhci_sim
