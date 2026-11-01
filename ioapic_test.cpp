#include "ioapic.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace protos {
namespace {

namespace t = ::testing;

constexpr uintptr_t kOwnerA = 0x1001;
constexpr uintptr_t kOwnerB = 0x2002;

class FakeIoApicHardware final : public IoApicHardware {
 public:
  struct Controller {
    bool map_ok = true;
    uintptr_t mapped_phys = 0;
    uint32_t regs[256] = {};
  };

  explicit FakeIoApicHardware(const int pins = 24) {
    for (Controller& ctrl : controllers_) {
      ctrl.regs[kIoApicRegVer] =
          0x00000011u | (static_cast<uint32_t>(pins - 1) << 16);
      // Leave redirection entries initially unmasked with stale bits to verify
      // IoApicInit() masks every pin.
      for (int p = 0; p < pins; ++p) {
        ctrl.regs[IoApicRedirLowReg(p)] =
            0x00000020u + static_cast<uint32_t>(p);
        ctrl.regs[IoApicRedirHighReg(p)] = 0xAA000000u;
      }
    }
  }

  bool MapController(const int index, const uintptr_t phys_addr) override {
    if (index < 0 || index >= kMaxSupportedIoApics ||
        !controllers_[index].map_ok) {
      return false;
    }
    controllers_[index].mapped_phys = phys_addr;
    return true;
  }

  uint32_t ReadReg(const int index, const uint8_t reg) override {
    return controllers_[index].regs[reg];
  }

  void WriteReg(const int index,    //
                const uint8_t reg,  //
                const uint32_t value) override {
    writes_.emplace_back(index, reg, value);
    controllers_[index].regs[reg] = value;
  }

  Controller controllers_[kMaxSupportedIoApics] = {};
  std::vector<std::tuple<int, uint8_t, uint32_t>> writes_;
};

class IoApicTest : public t::Test {
 protected:
  void SetUp() override { IoApicResetForTest(); }
  void TearDown() override { IoApicResetForTest(); }
};

TEST_F(IoApicTest, InitMasksAllPinsSkipsInvalidControllersAndRoutesGsis) {
  auto hw = std::make_shared<FakeIoApicHardware>(24);
  // Controller 1 returns 0xFFFFFFFF (non-decoding MMIO) and must be skipped.
  hw->controllers_[1].regs[kIoApicRegVer] = 0xFFFFFFFFu;
  // Controller 2 fails MapController and must be skipped.
  hw->controllers_[2].map_ok = false;
  // Controller 3 covers GSIs 24..47 with 24 pins.

  const IoApicInfo ioapics[4] = {
      {.ioapic_id = 1, .phys_addr = 0xFEC00000u, .gsi_base = 0},
      {.ioapic_id = 2, .phys_addr = 0xFEC01000u, .gsi_base = 48},
      {.ioapic_id = 3, .phys_addr = 0xFEC02000u, .gsi_base = 72},
      {.ioapic_id = 4, .phys_addr = 0xFEC03000u, .gsi_base = 24},
  };
  IsaIrqRoute routes[kIsaIrqCount] = {};
  for (int irq = 0; irq < kIsaIrqCount; ++irq) {
    routes[irq].gsi = static_cast<uint32_t>(irq);
  }
  // Route ISA IRQ 1 to GSI 26 on controller 3 (pin 2), active-low, edge.
  routes[1] = {.gsi = 26, .active_low = true, .level_triggered = false};
  // Route ISA IRQ 9 to GSI 9 on controller 0 (pin 9), active-high, level.
  routes[9] = {.gsi = 9, .active_low = false, .level_triggered = true};
  // Route ISA IRQ 12 to unbacked GSI 60.
  routes[12] = {.gsi = 60, .active_low = false, .level_triggered = false};

  IoApicInit(hw, ioapics, 4, routes);

  // Every pin on active controllers 0 and 3 was masked at init.
  for (int p = 0; p < 24; ++p) {
    EXPECT_NE(
        hw->controllers_[0].regs[IoApicRedirLowReg(p)] & kIoApicRedirMaskBit,
        0u);
    EXPECT_NE(
        hw->controllers_[3].regs[IoApicRedirLowReg(p)] & kIoApicRedirMaskBit,
        0u);
  }

  EXPECT_TRUE(IoApicHasIsaIrq(0));
  EXPECT_TRUE(IoApicHasIsaIrq(1));
  EXPECT_TRUE(IoApicHasIsaIrq(9));
  EXPECT_FALSE(IoApicHasIsaIrq(12));
  EXPECT_FALSE(IoApicHasIsaIrq(-1));
  EXPECT_FALSE(IoApicHasIsaIrq(16));

  bool level = true;
  ASSERT_THAT(IoApicProgramIsaIrq(1, 0x52, 7, &level),
              t::Eq(IoApicStatus::kOk));
  EXPECT_FALSE(level);
  // Pin 2 on controller 3: vector 0x52, active-low, edge, unmasked, dest 7.
  EXPECT_THAT(hw->controllers_[3].regs[IoApicRedirHighReg(2)],
              t::Eq(0x07000000u));
  EXPECT_THAT(hw->controllers_[3].regs[IoApicRedirLowReg(2)],
              t::Eq(0x52u | kIoApicRedirActiveLow));

  ASSERT_THAT(IoApicProgramIsaIrq(9, 0x53, 0, &level),
              t::Eq(IoApicStatus::kOk));
  EXPECT_TRUE(level);
  EXPECT_THAT(hw->controllers_[0].regs[IoApicRedirHighReg(9)],
              t::Eq(0x00000000u));
  // Level-triggered pins stay masked on programming so the caller can arm its
  // level-mask tracking before `DeviceIrqWait` unmasks the pin.
  EXPECT_THAT(hw->controllers_[0].regs[IoApicRedirLowReg(9)],
              t::Eq(0x53u | kIoApicRedirLevelTriggered | kIoApicRedirMaskBit));

  // Masking and unmasking toggles bit 16 cleanly while preserving vector/flags.
  EXPECT_TRUE(IoApicSetIsaIrqMasked(1, true));
  EXPECT_THAT(hw->controllers_[3].regs[IoApicRedirLowReg(2)],
              t::Eq(0x52u | kIoApicRedirActiveLow | kIoApicRedirMaskBit));
  EXPECT_TRUE(IoApicSetIsaIrqMasked(1, false));
  EXPECT_THAT(hw->controllers_[3].regs[IoApicRedirLowReg(2)],
              t::Eq(0x52u | kIoApicRedirActiveLow));

  // Invalid / unbacked cases fail cleanly.
  EXPECT_THAT(IoApicProgramIsaIrq(12, 0x54, 0, nullptr),
              t::Eq(IoApicStatus::kNoDevice));
  EXPECT_FALSE(IoApicSetIsaIrqMasked(12, true));
  EXPECT_THAT(IoApicProgramIsaIrq(1, 0x1F, 0, nullptr),
              t::Eq(IoApicStatus::kInvalid));
  EXPECT_THAT(IoApicProgramIsaIrq(1, 0xFF, 0, nullptr),
              t::Eq(IoApicStatus::kInvalid));
  EXPECT_THAT(IoApicProgramIsaIrq(1, 0x52, 0xFF, nullptr),
              t::Eq(IoApicStatus::kInvalid));
}

TEST_F(IoApicTest, IsaDeviceOwnershipAndPortTableEnforceExclusiveAccess) {
  auto hw = std::make_shared<FakeIoApicHardware>(24);
  IoApicInit(hw, nullptr, 0, nullptr);

  EXPECT_THAT(IsaClaimedCount(), t::Eq(0));
  EXPECT_THAT(IsaDeviceIrq(kIsaDeviceI8042), t::Eq(1));
  EXPECT_THAT(IsaDeviceIrq(-1), t::Eq(-1));
  EXPECT_THAT(IsaDeviceIrq(kIsaDeviceCount), t::Eq(-1));

  EXPECT_TRUE(IsaDeviceOwnsPort(kIsaDeviceI8042, 0x60));
  EXPECT_TRUE(IsaDeviceOwnsPort(kIsaDeviceI8042, 0x64));
  EXPECT_FALSE(IsaDeviceOwnsPort(kIsaDeviceI8042, 0x61));
  EXPECT_FALSE(IsaDeviceOwnsPort(kIsaDeviceI8042, 0x3F8));
  EXPECT_THAT(IsaFindDeviceForPort(0x60), t::Eq(kIsaDeviceI8042));
  EXPECT_THAT(IsaFindDeviceForPort(0x64), t::Eq(kIsaDeviceI8042));
  EXPECT_THAT(IsaFindDeviceForPort(0x62), t::Eq(-1));

  EXPECT_THAT(IsaClaim(-1, kOwnerA), t::Eq(IsaStatus::kNoDevice));
  EXPECT_THAT(IsaClaim(kIsaDeviceCount, kOwnerA), t::Eq(IsaStatus::kNoDevice));

  ASSERT_THAT(IsaClaim(kIsaDeviceI8042, kOwnerA), t::Eq(IsaStatus::kOk));
  // Idempotent for same owner; busy for another owner.
  EXPECT_THAT(IsaClaim(kIsaDeviceI8042, kOwnerA), t::Eq(IsaStatus::kOk));
  EXPECT_THAT(IsaClaim(kIsaDeviceI8042, kOwnerB), t::Eq(IsaStatus::kBusy));
  EXPECT_THAT(IsaOwner(kIsaDeviceI8042), t::Eq(kOwnerA));
  EXPECT_THAT(IsaClaimedCount(), t::Eq(1));

  // Non-owner cannot release; owner can release and another owner can re-claim.
  EXPECT_THAT(IsaRelease(kIsaDeviceI8042, kOwnerB),
              t::Eq(IsaStatus::kNotOwner));
  ASSERT_THAT(IsaRelease(kIsaDeviceI8042, kOwnerA), t::Eq(IsaStatus::kOk));
  EXPECT_THAT(IsaClaimedCount(), t::Eq(0));
  ASSERT_THAT(IsaClaim(kIsaDeviceI8042, kOwnerB), t::Eq(IsaStatus::kOk));
  EXPECT_THAT(IsaOwner(kIsaDeviceI8042), t::Eq(kOwnerB));

  // When ACPI FADT reports no 8042 controller (`has_i8042 == false`), claiming
  // `kIsaDeviceI8042` fails with `IsaStatus::kNoDevice`.
  IoApicInit(hw, nullptr, 0, nullptr, /*has_i8042=*/false);
  EXPECT_THAT(IsaClaim(kIsaDeviceI8042, kOwnerA), t::Eq(IsaStatus::kNoDevice));
  EXPECT_THAT(IsaClaimedCount(), t::Eq(0));
}

}  // namespace
}  // namespace protos
