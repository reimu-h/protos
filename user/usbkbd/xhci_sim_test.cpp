#include "user/usbkbd/xhci_sim.h"

#include <gmock/gmock.h>
#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

namespace protos::xhci_sim {
namespace {

namespace t = ::testing;

// Register offsets of the default (QEMU) layout, written out by hand.
constexpr int64_t kUsbCmd = 0x40;
constexpr int64_t kUsbSts = 0x44;
constexpr int64_t kPageSize = 0x48;
constexpr int64_t kDnCtrl = 0x54;
constexpr int64_t kCrcr = 0x58;
constexpr int64_t kDcbaap = 0x70;
constexpr int64_t kConfig = 0x78;
constexpr int64_t kPortsc1 = 0x440;
constexpr int64_t kPortsc5 = 0x480;
constexpr int64_t kIman0 = 0x1020;
constexpr int64_t kImod0 = 0x1024;
constexpr int64_t kErstBa0 = 0x1030;
constexpr int64_t kErdp0 = 0x1038;
constexpr int64_t kDoorbell0 = 0x2000;
// The legacy capability of IntelLikeConfig().
constexpr int64_t kLegSup = 0x8000;
constexpr int64_t kLegCtlSts = 0x8004;

// The register bits used below, written out by hand.
constexpr uint32_t kRunStop = 0x1;
constexpr uint32_t kHcReset = 0x2;
constexpr uint32_t kHalted = 0x1;
constexpr uint32_t kNotReady = 0x800;
constexpr uint32_t kBiosOwned = 0x10000;
constexpr uint32_t kOsOwned = 0x1000000;

class XhciSimTest : public t::Test {
 protected:
  // Resets a halted simulated controller the way a driver should, including
  // the reads that observe the end of the reset.
  void ResetDirectly(XhciSim& sim, const int64_t usbcmd, const int64_t usbsts) {
    sim.Write32(usbcmd, kHcReset);
    clock_.Advance(2);  // No access right after HCRST
    for (int i = 0; i < 100 && (sim.Read32(usbcmd) & kHcReset) != 0; ++i) {
      clock_.Advance(1);
    }
    for (int i = 0; i < 100 && (sim.Read32(usbsts) & kNotReady) != 0; ++i) {
      clock_.Advance(1);
    }
    ASSERT_TRUE(sim.reset_observed());
  }

  SimClock clock_;
};

TEST_F(XhciSimTest, CapabilityRegistersMatchQemu) {
  XhciSim sim(&clock_, XhciSimConfig{});
  EXPECT_THAT(sim.Read32(0x00), t::Eq(0x01000040u));
  EXPECT_THAT(sim.Read32(0x04), t::Eq(0x08001040u));
  EXPECT_THAT(sim.Read32(0x08), t::Eq(0x0000000Fu));
  EXPECT_THAT(sim.Read32(0x0C), t::Eq(0u));
  EXPECT_THAT(sim.Read32(0x10), t::Eq(0x00087001u));
  EXPECT_THAT(sim.Read32(0x14), t::Eq(0x2000u));
  EXPECT_THAT(sim.Read32(0x18), t::Eq(0x1000u));
  EXPECT_THAT(sim.Read32(0x20), t::Eq(0x02000402u));
  EXPECT_THAT(sim.Read32(0x24), t::Eq(0x20425355u));
  EXPECT_THAT(sim.Read32(0x28), t::Eq(0x00000405u));
  EXPECT_THAT(sim.Read32(0x2C), t::Eq(0u));
  EXPECT_THAT(sim.Read32(0x30), t::Eq(0x03000002u));
  EXPECT_THAT(sim.Read32(0x38), t::Eq(0x00000401u));
  EXPECT_THAT(sim.Read32(kUsbCmd), t::Eq(0u));
  EXPECT_THAT(sim.Read32(kUsbSts), t::Eq(kHalted));
  EXPECT_THAT(sim.Read32(kPageSize), t::Eq(1u));
  EXPECT_THAT(sim.Read32(kPortsc5), t::Eq(0x2A0u));  // PP, RxDetect
  EXPECT_THAT(sim.Size(), t::Eq(0x4000));
  EXPECT_THAT(sim.legacy_offset(), t::Eq(0));
}

TEST_F(XhciSimTest, CapabilitiesOfARealHardwareProfile) {
  XhciSimConfig config = IntelLikeConfig();
  config.db_off_raw = 0x3003;
  config.rts_off_raw = 0x201F;
  XhciSim sim(&clock_, config);
  EXPECT_THAT(sim.Read32(0x00), t::Eq(0x01100080u));
  EXPECT_THAT(sim.Read32(0x04), t::Eq(0x10000840u));
  // 35 scratchpads = Hi 1 [25:21], Lo 3 [31:27]; SPR; ERST Max 15; IST 1.
  EXPECT_THAT(sim.Read32(0x08),
              t::Eq((3u << 27) | (1u << 26) | (1u << 21) | (0xFu << 4) | 0x1u));
  // xECP 0x8000 / 4; PPC, CSZ, AC64.
  EXPECT_THAT(sim.Read32(0x10), t::Eq(0x2000020Du));
  EXPECT_THAT(sim.Read32(0x14), t::Eq(0x3003u));
  EXPECT_THAT(sim.Read32(0x18), t::Eq(0x201Fu));
  EXPECT_THAT(sim.rt_base(), t::Eq(0x2000));
  EXPECT_THAT(sim.db_base(), t::Eq(0x3000));
  // Port power control: ports are unpowered after a reset.
  EXPECT_THAT(sim.Read32(0x480), t::Eq(0u));
}

TEST_F(XhciSimTest, ExtendedCapabilityListIsWalkableByRawReads) {
  XhciSim sim(&clock_, IntelLikeConfig());
  EXPECT_THAT(sim.legacy_offset(), t::Eq(kLegSup));
  // Legacy (next 8 dwords), BIOS owned.
  EXPECT_THAT(sim.Read32(0x8000), t::Eq(0x00010801u));
  // USB 2.0 (next 8 dwords), ports 1-10.
  EXPECT_THAT(sim.Read32(0x8020), t::Eq(0x02000802u));
  EXPECT_THAT(sim.Read32(0x8028), t::Eq(0x00000A01u));
  // USB 3.1 (next 12 dwords), ports 11-16.
  EXPECT_THAT(sim.Read32(0x8040), t::Eq(0x03100C02u));
  EXPECT_THAT(sim.Read32(0x8048), t::Eq(0x0000060Bu));
  // Vendor capability, last.
  EXPECT_THAT(sim.Read32(0x8070), t::Eq(0x5A5A00C0u));
}

TEST_F(XhciSimTest, UsbStsWriteOneClearsOnlyTheEventBits) {
  XhciSimConfig config;
  config.usbsts_pending = 0x41C;  // HSE, EINT, PCD, SRE
  config.hce = true;
  XhciSim sim(&clock_, config);
  EXPECT_THAT(sim.Read32(kUsbSts), t::Eq(0x141Du));
  sim.Write32(kUsbSts, 0x8);  // EINT only
  EXPECT_THAT(sim.Read32(kUsbSts), t::Eq(0x1415u));
  sim.AllowViolation(Violation::kRsvdZWrittenOne);
  sim.Write32(kUsbSts, 0xFFFFFFFF);
  // HCH and HCE are read-only.
  EXPECT_THAT(sim.Read32(kUsbSts), t::Eq(0x1001u));
  EXPECT_TRUE(sim.HasViolation(Violation::kRsvdZWrittenOne));
  EXPECT_THAT(sim.CountViolations(Violation::kProgrammingBeforeReset),
              t::Eq(0));
}

TEST_F(XhciSimTest, ClearingRunStopHaltsAfterTheDelay) {
  XhciSimConfig config;
  config.running = true;
  config.halt_delay_ticks = 3;
  XhciSim sim(&clock_, config);
  EXPECT_THAT(sim.Read32(kUsbCmd), t::Eq(kRunStop));
  EXPECT_THAT(sim.Read32(kUsbSts) & kHalted, t::Eq(0u));
  sim.Write32(kUsbCmd, 0);
  for (int tick = 0; tick < 3; ++tick) {
    EXPECT_THAT(sim.Read32(kUsbSts) & kHalted, t::Eq(0u)) << "tick " << tick;
    clock_.Advance(1);
  }
  EXPECT_THAT(sim.halt_observed_seq(), t::Eq(-1));
  EXPECT_THAT(sim.Read32(kUsbSts) & kHalted, t::Eq(kHalted));
  EXPECT_THAT(sim.halt_observed_seq(), t::Eq(sim.accesses().back().seq));
}

TEST_F(XhciSimTest, NeverHaltingControllerStaysRunning) {
  XhciSimConfig config;
  config.running = true;
  config.never_halts = true;
  XhciSim sim(&clock_, config);
  sim.Write32(kUsbCmd, 0);
  clock_.Advance(100000);
  EXPECT_THAT(sim.Read32(kUsbSts) & kHalted, t::Eq(0u));
}

TEST_F(XhciSimTest, ResetRunsHcrstThenCnrAndRestoresDefaults) {
  XhciSimConfig config = OvmfLeftHaltedConfig();
  config.reset_delay_ticks = 2;
  config.not_ready_after_reset_ticks = 3;
  config.imod_reset_value = 0x0FA0;
  XhciSim sim(&clock_, config);
  EXPECT_THAT(sim.Read32(kDcbaap), t::Eq(0xBF000000u));
  EXPECT_THAT(sim.Read32(kConfig), t::Eq(8u));
  EXPECT_THAT(sim.Read32(kErstBa0), t::Eq(0xBF002000u));

  sim.Write32(kUsbCmd, kHcReset);
  EXPECT_THAT(sim.reset_count(), t::Eq(1));
  EXPECT_THAT(sim.reset_write_seq(), t::Eq(sim.accesses().back().seq));
  EXPECT_THAT(sim.Read32(kUsbCmd), t::Eq(kHcReset));
  EXPECT_THAT(sim.Read32(kUsbSts), t::Eq(kHalted | kNotReady));
  clock_.Advance(1);
  EXPECT_THAT(sim.Read32(kUsbCmd), t::Eq(kHcReset));
  clock_.Advance(1);
  EXPECT_THAT(sim.Read32(kUsbCmd), t::Eq(0u));
  EXPECT_THAT(sim.Read32(kUsbSts), t::Eq(kHalted | kNotReady));
  EXPECT_FALSE(sim.reset_observed());
  clock_.Advance(3);
  EXPECT_THAT(sim.Read32(kUsbSts), t::Eq(kHalted));
  EXPECT_TRUE(sim.reset_observed());
  EXPECT_THAT(sim.reset_observed_seq(), t::Eq(sim.accesses().back().seq));
  // Defaults: EINT cleared, stale pointers gone, IMOD at its reset value.
  EXPECT_THAT(sim.Read32(kDcbaap), t::Eq(0u));
  EXPECT_THAT(sim.Read32(kConfig), t::Eq(0u));
  EXPECT_THAT(sim.Read32(kErstBa0), t::Eq(0u));
  EXPECT_THAT(sim.Read32(kErdp0), t::Eq(0u));
  EXPECT_THAT(sim.Read32(kImod0), t::Eq(0x0FA0u));
}

TEST_F(XhciSimTest, HcrstWhileRunningIsAViolationAndIsIgnored) {
  XhciSimConfig config;
  config.running = true;
  XhciSim sim(&clock_, config);
  sim.AllowViolation(Violation::kResetWhileRunning);
  sim.AllowViolation(Violation::kProgrammingBeforeReset);  // R/S = 1 written
  sim.Write32(kUsbCmd, kRunStop | kHcReset);
  EXPECT_THAT(sim.CountViolations(Violation::kResetWhileRunning), t::Eq(1));
  EXPECT_THAT(sim.reset_count(), t::Eq(0));
  // Clearing Run/Stop in the same write is not enough: HCH is still 0.
  sim.Write32(kUsbCmd, kHcReset);
  EXPECT_THAT(sim.CountViolations(Violation::kResetWhileRunning), t::Eq(2));
  EXPECT_THAT(sim.reset_count(), t::Eq(0));
}

TEST_F(XhciSimTest, HcrstDuringAResetIsAViolation) {
  XhciSimConfig config;
  config.reset_delay_ticks = 5;
  XhciSim sim(&clock_, config);
  sim.AllowViolation(Violation::kHcrstRetriggered);
  sim.AllowViolation(Violation::kOpWriteWhileNotReady);  // CNR = 1 meanwhile
  sim.Write32(kUsbCmd, kHcReset);
  clock_.Advance(1);
  sim.Write32(kUsbCmd, kHcReset);
  EXPECT_TRUE(sim.HasViolation(Violation::kHcrstRetriggered));
  EXPECT_THAT(sim.reset_count(), t::Eq(1));
}

TEST_F(XhciSimTest, AccessesRightAfterHcrstAreViolations) {
  XhciSimConfig config;
  config.hcrst_quiet_ticks = 2;
  XhciSim sim(&clock_, config);
  sim.AllowViolation(Violation::kAccessTooSoonAfterReset);
  sim.Write32(kUsbCmd, kHcReset);
  sim.Read32(kUsbCmd);
  clock_.Advance(1);
  sim.Read32(kUsbCmd);
  EXPECT_THAT(sim.CountViolations(Violation::kAccessTooSoonAfterReset),
              t::Eq(2));
  clock_.Advance(1);
  sim.Read32(kUsbCmd);
  EXPECT_THAT(sim.CountViolations(Violation::kAccessTooSoonAfterReset),
              t::Eq(2));
}

TEST_F(XhciSimTest, ProgrammingBeforeACompletedResetIsAViolation) {
  XhciSim sim(&clock_, XhciSimConfig{});
  sim.AllowViolation(Violation::kProgrammingBeforeReset);
  const std::vector<int64_t> setup_registers = {
      kDnCtrl,  kCrcr,  kCrcr + 4, kDcbaap,  kDcbaap + 4, kConfig,
      kPortsc1, kIman0, kImod0,    kErstBa0, kErdp0,      kDoorbell0};
  int expected = 0;
  for (const int64_t offset : setup_registers) {
    sim.Write32(offset, 0);
    ++expected;
    EXPECT_THAT(sim.CountViolations(Violation::kProgrammingBeforeReset),
                t::Eq(expected))
        << "offset 0x" << std::hex << offset;
  }
  // Starting the controller or enabling interrupts is setup too; clearing
  // them is not.
  sim.Write32(kUsbCmd, 0x4);
  sim.Write32(kUsbCmd, kRunStop);
  sim.Write32(kUsbCmd, 0);
  EXPECT_THAT(sim.CountViolations(Violation::kProgrammingBeforeReset),
              t::Eq(expected + 2));
  sim.Write32(kUsbCmd, 0);
  clock_.Advance(1);
  ASSERT_THAT(sim.Read32(kUsbSts) & kHalted, t::Eq(kHalted));

  ResetDirectly(sim, kUsbCmd, kUsbSts);
  const int before = sim.CountViolations(Violation::kProgrammingBeforeReset);
  sim.Write32(kDcbaap, 0x1000);
  sim.Write32(kDcbaap + 4, 0);
  sim.Write32(kConfig, 4);
  EXPECT_THAT(sim.CountViolations(Violation::kProgrammingBeforeReset),
              t::Eq(before));
  EXPECT_THAT(sim.Read32(kDcbaap), t::Eq(0x1000u));
}

TEST_F(XhciSimTest, OperationalWritesWhileNotReadyAreViolations) {
  XhciSimConfig config;
  config.power_on_not_ready_ticks = 5;
  XhciSim sim(&clock_, config);
  sim.AllowViolation(Violation::kOpWriteWhileNotReady);
  sim.AllowViolation(Violation::kProgrammingBeforeReset);
  clock_.Advance(1);
  EXPECT_THAT(sim.Read32(kUsbSts) & kNotReady, t::Eq(kNotReady));
  sim.Write32(kUsbSts, 0x8);  // USBSTS may be written
  EXPECT_FALSE(sim.HasViolation(Violation::kOpWriteWhileNotReady));
  sim.Write32(kConfig, 4);
  sim.Write32(kDoorbell0, 0);
  EXPECT_THAT(sim.CountViolations(Violation::kOpWriteWhileNotReady), t::Eq(2));
  clock_.Advance(4);
  EXPECT_THAT(sim.Read32(kUsbSts) & kNotReady, t::Eq(0u));
  sim.Write32(kConfig, 4);
  EXPECT_THAT(sim.CountViolations(Violation::kOpWriteWhileNotReady), t::Eq(2));
}

TEST_F(XhciSimTest, LegacyFirmwareReleasesOnTheNthPollAfterTheRequest) {
  XhciSim sim(&clock_, IntelLikeConfig());
  const uint32_t initial = sim.Read32(kLegSup);
  EXPECT_THAT(initial & (kBiosOwned | kOsOwned), t::Eq(kBiosOwned));
  // Reads before the request don't count.
  sim.Read32(kLegSup);
  EXPECT_THAT(sim.Read32(kLegSup) & kBiosOwned, t::Eq(kBiosOwned));

  sim.Write32(kLegSup, initial | kOsOwned);
  EXPECT_THAT(sim.request_seq(), t::Eq(sim.accesses().back().seq));
  EXPECT_THAT(sim.Read32(kLegCtlSts) & (1u << 29), t::Ne(0u));
  EXPECT_THAT(sim.Read32(kLegSup) & kBiosOwned, t::Eq(kBiosOwned));
  EXPECT_THAT(sim.Read32(kLegSup) & kBiosOwned, t::Eq(kBiosOwned));
  EXPECT_THAT(sim.Read32(kLegSup) & (kBiosOwned | kOsOwned), t::Eq(kOsOwned));
  EXPECT_TRUE(sim.released_by_bios());
  EXPECT_FALSE(sim.forced());
}

TEST_F(XhciSimTest, LegacyFirmwareReleasesAfterTicks) {
  XhciSimConfig config = IntelLikeConfig();
  config.legacy.behavior = BiosBehavior::kReleasesAfterTicks;
  config.legacy.release_after_ticks = 10;
  XhciSim sim(&clock_, config);
  sim.Write32(kLegSup, sim.Read32(kLegSup) | kOsOwned);
  for (int i = 0; i < 50; ++i) {
    sim.Read32(kLegSup);
  }
  clock_.Advance(9);
  EXPECT_THAT(sim.Read32(kLegSup) & kBiosOwned, t::Eq(kBiosOwned));
  clock_.Advance(1);
  EXPECT_THAT(sim.Peek(kLegSup) & kBiosOwned, t::Eq(0u));
  EXPECT_TRUE(sim.released_by_bios());
}

TEST_F(XhciSimTest, LegacyFirmwareThatNeverReleases) {
  XhciSimConfig config = IntelLikeConfig();
  config.legacy.behavior = BiosBehavior::kNeverReleases;
  XhciSim sim(&clock_, config);
  sim.Write32(kLegSup, sim.Read32(kLegSup) | kOsOwned);
  for (int i = 0; i < 1000; ++i) {
    sim.Read32(kLegSup);
    clock_.Advance(10);
  }
  EXPECT_THAT(sim.Read32(kLegSup) & kBiosOwned, t::Eq(kBiosOwned));
  EXPECT_FALSE(sim.released_by_bios());
}

TEST_F(XhciSimTest, LegacyFirmwareWithoutTheOwnershipSmiNeverReleases) {
  XhciSim sim(&clock_, IntelLikeConfig());
  // Disabling the SMIs before asking keeps firmware from noticing.
  const uint32_t control = sim.Read32(kLegCtlSts);
  sim.Write32(kLegCtlSts, (control & 0x000E1FEEu) | 0xE0000000u);
  sim.Write32(kLegSup, sim.Read32(kLegSup) | kOsOwned);
  for (int i = 0; i < 100; ++i) {
    sim.Read32(kLegSup);
  }
  EXPECT_THAT(sim.Read32(kLegSup) & kBiosOwned, t::Eq(kBiosOwned));
}

TEST_F(XhciSimTest, LegacyControlRegisterSemantics) {
  XhciSimConfig config = IntelLikeConfig();
  config.usbsts_pending = 0x4 | 0x8;  // HSE, EINT
  XhciSim sim(&clock_, config);
  // Enables 0, 4, 13-15, RsvdP pattern bits 2, 7, 18, events 29-31, and
  // the read-only shadows of EINT (16) and HSE (20).
  EXPECT_THAT(sim.Read32(kLegCtlSts), t::Eq(0xE015E095u));
  sim.Write32(sim.op(0x04), 0x8);  // USBSTS: clear EINT
  EXPECT_THAT(sim.Read32(kLegCtlSts), t::Eq(0xE014E095u));
  // Disable the SMIs, clear the events, keep RsvdP.
  sim.Write32(kLegCtlSts, 0xE0040084u);
  EXPECT_THAT(sim.Read32(kLegCtlSts), t::Eq(0x00140084u));
  EXPECT_THAT(sim.violations(), t::IsEmpty());

  sim.AllowViolation(Violation::kRsvdPChanged);
  sim.AllowViolation(Violation::kRsvdZWrittenOne);
  sim.Write32(kLegCtlSts, 0x00000080u);  // RsvdP bits 2 and 18 dropped
  EXPECT_THAT(sim.CountViolations(Violation::kRsvdPChanged), t::Eq(1));
  sim.Write32(kLegCtlSts, 0x00240084u);  // RsvdZ bit 21
  EXPECT_THAT(sim.CountViolations(Violation::kRsvdZWrittenOne), t::Eq(1));
}

TEST_F(XhciSimTest, ClearingBiosOwnedEarlyIsAViolationLateIsAForcedTakeover) {
  {
    XhciSim sim(&clock_, IntelLikeConfig());
    sim.AllowViolation(Violation::kBiosSemaphoreClearedEarly);
    // "Write OS Owned alone": asks and clears BIOS Owned in one write.
    sim.Write32(kLegSup, (sim.Read32(kLegSup) | kOsOwned) & ~kBiosOwned);
    EXPECT_THAT(sim.CountViolations(Violation::kBiosSemaphoreClearedEarly),
                t::Eq(1));
    EXPECT_FALSE(sim.forced());
  }
  XhciSimConfig config = IntelLikeConfig();
  config.legacy.behavior = BiosBehavior::kNeverReleases;
  XhciSim sim(&clock_, config);
  sim.Write32(kLegSup, sim.Read32(kLegSup) | kOsOwned);
  clock_.Advance(config.legacy.force_floor_ticks);
  sim.Write32(kLegSup, (sim.Read32(kLegSup) | kOsOwned) & ~kBiosOwned);
  EXPECT_TRUE(sim.forced());
  EXPECT_THAT(sim.violations(), t::IsEmpty());
}

TEST_F(XhciSimTest, SettingBiosOwnedOrChangingRsvdPInLegSupAreViolations) {
  XhciSimConfig config = IntelLikeConfig();
  config.legacy.bios_owned = false;
  config.legacy.legsup_rsvdp = 0x00020000;  // bit 17
  XhciSim sim(&clock_, config);
  sim.AllowViolation(Violation::kBiosSemaphoreSetBySoftware);
  sim.AllowViolation(Violation::kRsvdPChanged);
  const uint32_t value = sim.Read32(kLegSup);
  EXPECT_THAT(value, t::Eq(0x00020801u));
  sim.Write32(kLegSup, value | kBiosOwned);
  EXPECT_THAT(sim.CountViolations(Violation::kBiosSemaphoreSetBySoftware),
              t::Eq(1));
  sim.Write32(kLegSup, (value & ~0x00020000u) | kOsOwned | kBiosOwned);
  EXPECT_THAT(sim.CountViolations(Violation::kRsvdPChanged), t::Eq(1));
}

TEST_F(XhciSimTest, OutOfRangeAndUnalignedAccessesAreViolations) {
  XhciSim sim(&clock_, XhciSimConfig{});
  sim.AllowViolation(Violation::kOutOfRangeAccess);
  sim.AllowViolation(Violation::kUnalignedAccess);
  EXPECT_THAT(sim.Read32(0x4000), t::Eq(0xFFFFFFFFu));
  EXPECT_THAT(sim.Read32(-4), t::Eq(0xFFFFFFFFu));
  EXPECT_THAT(sim.Read32(0x3FFE), t::Eq(0xFFFFFFFFu));
  EXPECT_THAT(sim.Read32(0x42), t::Eq(0xFFFFFFFFu));
  sim.Write32(0x4000, 0);
  EXPECT_THAT(sim.CountViolations(Violation::kOutOfRangeAccess), t::Eq(4));
  EXPECT_THAT(sim.CountViolations(Violation::kUnalignedAccess), t::Eq(1));
}

TEST_F(XhciSimTest, WritesToCapabilitiesAndReservedSpaceAreViolations) {
  XhciSim sim(&clock_, XhciSimConfig{});
  sim.AllowViolation(Violation::kWriteToReadOnly);
  sim.AllowViolation(Violation::kWriteToReserved);
  sim.Write32(0x04, 0);            // HCSPARAMS1
  sim.Write32(0x20, 0);            // Supported Protocol capability
  sim.Write32(kPageSize, 0);       // PAGESIZE
  sim.Write32(0x3000, 0);          // Nothing there
  sim.Write32(kUsbCmd + 0x20, 0);  // Reserved operational register
  EXPECT_THAT(sim.CountViolations(Violation::kWriteToReadOnly), t::Eq(3));
  EXPECT_THAT(sim.CountViolations(Violation::kWriteToReserved), t::Eq(2));
  EXPECT_THAT(sim.Read32(0x04), t::Eq(0x08001040u));
}

TEST_F(XhciSimTest, HighDwordMustFollowTheLowDword) {
  XhciSim sim(&clock_, XhciSimConfig{});
  ResetDirectly(sim, kUsbCmd, kUsbSts);
  sim.AllowViolation(Violation::kHighDwordBeforeLow);
  sim.Write32(kDcbaap + 4, 0x1);
  EXPECT_THAT(sim.CountViolations(Violation::kHighDwordBeforeLow), t::Eq(1));
  sim.Write32(kDcbaap, 0x2000);
  sim.Write32(kDcbaap + 4, 0x1);
  EXPECT_THAT(sim.CountViolations(Violation::kHighDwordBeforeLow), t::Eq(1));
  // Any access between the halves breaks the pair.
  sim.Write32(kErstBa0, 0x3000);
  sim.Read32(kUsbSts);
  sim.Write32(kErstBa0 + 4, 0);
  EXPECT_THAT(sim.CountViolations(Violation::kHighDwordBeforeLow), t::Eq(2));
  EXPECT_THAT(sim.Read32(kDcbaap + 4), t::Eq(1u));
}

TEST_F(XhciSimTest, ControllerThatDoesNotDecodeReadsAllOnes) {
  XhciSimConfig config;
  config.all_ones = true;
  XhciSim sim(&clock_, config);
  EXPECT_THAT(sim.Read32(0x00), t::Eq(0xFFFFFFFFu));
  EXPECT_THAT(sim.Read32(kUsbSts), t::Eq(0xFFFFFFFFu));
  sim.Write32(kUsbCmd, kHcReset);
  EXPECT_THAT(sim.reset_count(), t::Eq(0));
}

TEST_F(XhciSimTest, ControllerCanStopDecodingLater) {
  XhciSimConfig config;
  config.stop_decoding_after_ticks = 3;
  XhciSim sim(&clock_, config);
  EXPECT_THAT(sim.Read32(kUsbSts), t::Eq(kHalted));
  clock_.Advance(3);
  EXPECT_THAT(sim.Read32(kUsbSts), t::Eq(0xFFFFFFFFu));
  EXPECT_THAT(sim.Read32(0x00), t::Eq(0xFFFFFFFFu));
}

TEST_F(XhciSimTest, AccessLogRecordsOrderTicksAndValues) {
  XhciSim sim(&clock_, XhciSimConfig{});
  sim.Read32(0x00);
  clock_.Advance(2);
  sim.Write32(kUsbCmd, kHcReset);
  const std::vector<Access>& log = sim.accesses();
  ASSERT_THAT(log.size(), t::Eq(2u));
  EXPECT_THAT(log[0].seq, t::Lt(log[1].seq));
  EXPECT_FALSE(log[0].write);
  EXPECT_THAT(log[0].offset, t::Eq(0));
  EXPECT_THAT(log[0].value, t::Eq(0x01000040u));
  EXPECT_THAT(log[0].tick, t::Eq(SimClock::kStartTick));
  EXPECT_TRUE(log[1].write);
  EXPECT_THAT(log[1].tick, t::Eq(SimClock::kStartTick + 2));
  EXPECT_THAT(sim.WritesTo(kUsbCmd).size(), t::Eq(1u));
  EXPECT_THAT(sim.FirstWriteSeq(kUsbCmd, kHcReset), t::Eq(log[1].seq));
  EXPECT_THAT(sim.FirstWriteSeq(kUsbCmd, kRunStop), t::Eq(-1));
  EXPECT_THAT(sim.CountReads(0x00), t::Eq(1));
  EXPECT_THAT(sim.TraceString(),
              t::HasSubstr("W 0x00040 = 0x00000002 op USBCMD"));
}

TEST_F(XhciSimTest, PortscWriteSemantics) {
  XhciSimConfig config;
  config.attached_speed = {4, 0, 0, 0, 3};  // SuperSpeed on 1, high speed on 5
  XhciSim sim(&clock_, config);
  ResetDirectly(sim, kUsbCmd, kUsbSts);
  // After the reset: USB 3 device enabled in U0; USB 2 device connected,
  // polling, waiting for a port reset. Both report a connect change.
  EXPECT_THAT(sim.Read32(kPortsc1), t::Eq(0x00021203u));
  EXPECT_THAT(sim.Read32(kPortsc5), t::Eq(0x00020EE1u));
  // Writing back the RO/RWS bits plus CSC clears only CSC.
  sim.Write32(kPortsc5, (0x00020EE1u & 0x4E00FFE9u) | 0x00020000u);
  EXPECT_THAT(sim.Read32(kPortsc5), t::Eq(0x00000EE1u));
  // PED is RW1C: writing back the value read disables the port.
  sim.AllowViolation(Violation::kPortDisabledBySoftware);
  sim.Write32(kPortsc1, 0x00001203u);
  EXPECT_THAT(sim.Read32(kPortsc1), t::Eq(0x00021201u));
  EXPECT_TRUE(sim.HasViolation(Violation::kPortDisabledBySoftware));
  // Verify unmodeled warm-reset / link-state write violations.
  sim.AllowViolation(Violation::kUnmodeledWrite);
  sim.Write32(kPortsc5, 0x00000EE1u | 0x10u);
  EXPECT_TRUE(sim.HasViolation(Violation::kUnmodeledWrite));
}

TEST_F(XhciSimTest, PortPowerControlPortsAreUnpoweredAfterResetAndKeepPower) {
  XhciSimConfig config = IntelLikeConfig();
  config.attached_speed = {3};
  XhciSim sim(&clock_, config);
  const int64_t usbcmd = sim.op(0x00);
  const int64_t usbsts = sim.op(0x04);
  ResetDirectly(sim, usbcmd, usbsts);
  const int64_t portsc = sim.portsc(1);
  EXPECT_THAT(sim.Read32(portsc), t::Eq(0u));
  sim.Write32(portsc, 0x200);  // PP
  EXPECT_THAT(sim.Read32(portsc), t::Eq(0x200u));
  sim.AllowViolation(Violation::kPortPowerCleared);
  sim.Write32(portsc, 0);
  EXPECT_TRUE(sim.HasViolation(Violation::kPortPowerCleared));
}

TEST_F(XhciSimTest, ResetRestoresThePortsThatFirmwareLeftEnabled) {
  XhciSim sim(&clock_, SeaBiosLeftRunningConfig());
  // SeaBIOS left the keyboard on port 5 enabled.
  EXPECT_THAT(sim.Read32(kPortsc5), t::Eq(0x00000E03u));
  sim.Write32(kUsbCmd, 0);
  clock_.Advance(1);
  ASSERT_THAT(sim.Read32(kUsbSts) & kHalted, t::Eq(kHalted));
  ResetDirectly(sim, kUsbCmd, kUsbSts);
  EXPECT_THAT(sim.Read32(kPortsc5), t::Eq(0x00020EE1u));
  EXPECT_THAT(sim.Read32(kUsbSts), t::Eq(kHalted));
}

TEST_F(XhciSimTest, BusyLoopWithoutRelaxIsReportedAndEndsWaits) {
  FakePlatform platform(&clock_);
  const int64_t start = clock_.now();
  EXPECT_NONFATAL_FAILURE(
      {
        for (int64_t i = 0; i <= SimClock::kBusyLoopLimit; ++i) {
          platform.NowTicks();
        }
      },
      "busy loop");
  EXPECT_TRUE(clock_.busy_loop_detected());
  EXPECT_THAT(clock_.now() - start, t::Ge(int64_t{1} << 20));
}

TEST_F(XhciSimTest, FakePlatformTimeInputAndDma) {
  FakePlatform platform(&clock_);
  EXPECT_THAT(platform.TicksPerSecond(), t::Eq(250));
  EXPECT_THAT(platform.NowTicks(), t::Eq(SimClock::kStartTick));
  platform.Relax();
  EXPECT_THAT(platform.NowTicks(), t::Eq(SimClock::kStartTick + 1));
  platform.set_relaxes_per_tick(3);
  platform.Relax();
  platform.Relax();
  EXPECT_THAT(platform.NowTicks(), t::Eq(SimClock::kStartTick + 1));
  platform.Relax();
  EXPECT_THAT(platform.NowTicks(), t::Eq(SimClock::kStartTick + 2));
  EXPECT_THAT(platform.relax_calls(), t::Eq(4));

  platform.PushInput("ab", 2);
  platform.PushInput("c", 1);
  EXPECT_THAT(platform.input(), t::Eq("abc"));

  const usb::DmaBuffer buffer = platform.DmaAlloc(4096, false);
  EXPECT_THAT(buffer.va, t::IsNull());
  EXPECT_THAT(platform.dma_alloc_calls(), t::Eq(1));
}

TEST_F(XhciSimTest, FakePlatformChecksEveryLogLine) {
  FakePlatform platform(&clock_);
  platform.Log("[USBKBD] ok\n", 12);
  EXPECT_THAT(platform.logs(), t::ElementsAre("[USBKBD] ok"));
  EXPECT_TRUE(platform.LogContains("ok"));
  EXPECT_NONFATAL_FAILURE(platform.Log("[USBKBD] no newline", 19),
                          "not one line");
  EXPECT_NONFATAL_FAILURE(platform.Log("[USBKBD] FAIL\n", 14),
                          "failure marker");
  EXPECT_NONFATAL_FAILURE(platform.Log("[USBKBD] [FATAL]\n", 17),
                          "failure marker");
  EXPECT_NONFATAL_FAILURE(platform.Log("no prefix\n", 10), "prefix");
  const std::string long_line = "[USBKBD] " + std::string(120, 'x') + "\n";
  EXPECT_NONFATAL_FAILURE(
      platform.Log(long_line.data(), static_cast<int>(long_line.size())),
      "invalid length");
}

TEST_F(XhciSimTest, UnexpectedViolationsFailTheTestAtDestruction) {
  EXPECT_NONFATAL_FAILURE(
      {
        XhciSim sim(&clock_, XhciSimConfig{});
        sim.Write32(kConfig, 1);
      },
      "unexpected xHCI simulator violations");
  // Allowed violations don't fail.
  XhciSim sim(&clock_, XhciSimConfig{});
  sim.AllowViolation(Violation::kProgrammingBeforeReset);
  sim.Write32(kConfig, 1);
}

TEST_F(XhciSimTest, UsbCmdRsvdPAndUnsupportedCommandsAreViolations) {
  XhciSimConfig config;
  config.usbcmd_rsvdp = 0x00A00010u;
  XhciSim sim(&clock_, config);
  sim.Write32(kUsbCmd, config.usbcmd_rsvdp | kHcReset);
  clock_.Advance(2);
  ASSERT_THAT(sim.Read32(kUsbCmd) & kHcReset, t::Eq(0u));
  ASSERT_THAT(sim.Read32(kUsbSts), t::Eq(kHalted));
  sim.AllowViolation(Violation::kRsvdPChanged);
  sim.AllowViolation(Violation::kUnsupportedCommand);
  // Dropping RsvdP bits in USBCMD is a kRsvdPChanged violation.
  sim.Write32(kUsbCmd, 0u);
  EXPECT_THAT(sim.CountViolations(Violation::kRsvdPChanged), t::Eq(1));
  // Writing Light HC Reset (bit 7), Save State (bit 8), or Restore State
  // (bit 9) is a kUnsupportedCommand violation.
  for (const uint32_t bit : {1u << 7, 1u << 8, 1u << 9}) {
    sim.Write32(kUsbCmd, config.usbcmd_rsvdp | bit);
  }
  EXPECT_THAT(sim.CountViolations(Violation::kUnsupportedCommand), t::Eq(3));
}

}  // namespace
}  // namespace protos::xhci_sim
