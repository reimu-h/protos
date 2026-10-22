#include <cstdint>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "user/usb_descriptors.h"
#include "user/xhci_controller.h"
#include "user/xhci_regs.h"
#include "xhci_sim.h"
#include "xhci_sim_usb.h"
#include "xhci_sim_world.h"

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
  // (where hardware endpoint state is still Running, not Halted) causes
  // Reset Endpoint + Set TR Dequeue Pointer to fail with Context State Error,
  // exercising the kWaitingSetDeqIntrEp error recovery branch ("interrupt ep
  // recovery error") and re-enumerating the keyboard cleanly.
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
                t::Contains("[USBKBD] 00:04.0: port 5: interrupt ep recovery "
                            "error (attempt 1)"));
    ASSERT_TRUE(world.RunUntil(
        [&] { return world.controller(idx).attached_keyboard_count() == 1; }));
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
    EXPECT_THAT(world.platform().logs(),
                t::Contains("[USBKBD] 00:04.0: port 5: invalid config "
                            "descriptor header"));
  }
}

INSTANTIATE_TEST_SUITE_P(Profiles, XhciEnumProfileTest, t::ValuesIn(kProfiles),
                         [](const t::TestParamInfo<EnumProfile>& info) {
                           return info.param.name;
                         });

}  // namespace
}  // namespace protos::xhci_sim
