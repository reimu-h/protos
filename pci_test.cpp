#include "pci.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <climits>
#include <cstdint>
#include <ios>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "pci_config_fake.h"

namespace protos {
namespace {

namespace t = ::testing;

using Access = FakePciConfigSpace::Access;
using BarKind = FakePciConfigSpace::BarKind;
using FunctionSpec = FakePciConfigSpace::FunctionSpec;

constexpr uintptr_t kOwnerA = 0xA000;
constexpr uintptr_t kOwnerB = 0xB000;
// The write-1-to-clear error bits of the Status register.
constexpr uint16_t kStatusErrorBits = 0xF900;
constexpr uint16_t kMemAndBusMaster =
    kPciCommandMemorySpace | kPciCommandBusMaster;

std::string g_report;

static void CaptureReport(const char* const text) { g_report += text; }

// The boot report, one string per line.
static std::vector<std::string> ReportLines() {
  g_report.clear();
  PciWriteBootReport(CaptureReport);
  std::vector<std::string> lines;
  std::string line;
  for (const char c : g_report) {
    if (c == '\n') {
      lines.push_back(line);
      line.clear();
    } else {
      line.push_back(c);
    }
  }
  EXPECT_THAT(line, t::IsEmpty()) << "the report ends without a newline";
  return lines;
}

// An endpoint function (an Ethernet controller).
static FunctionSpec EndpointSpec(const uint16_t command = 0,
                                 const uint16_t status = 0) {
  return {.vendor_id = 0x8086,
          .device_id = 0x10D3,
          .class_code = 0x02,
          .subclass = 0x00,
          .prog_if = 0x00,
          .revision = 0x01,
          .header_type = 0x00,
          .command = command,
          .status = status};
}

// Function 0 of a multi-function endpoint device.
static FunctionSpec MultiFunctionEndpointSpec() {
  FunctionSpec spec = EndpointSpec();
  spec.header_type = kPciHeaderTypeMultiFunction;
  return spec;
}

// A PCI-to-PCI bridge.
static FunctionSpec BridgeSpec(const uint16_t command = 0,
                               const uint16_t status = 0) {
  return {.vendor_id = 0x8086,
          .device_id = 0x244E,
          .class_code = 0x06,
          .subclass = 0x04,
          .prog_if = 0x00,
          .revision = 0x01,
          .header_type = 0x01,
          .command = command,
          .status = status};
}

MATCHER_P4(IsBar, type, prefetchable, base, size, "") {
  return arg.type == type && arg.prefetchable == prefetchable &&
         arg.base == static_cast<uint64_t>(base) &&
         arg.size == static_cast<int64_t>(size);
}

MATCHER_P3(IsAccess, write, width, offset, "") {
  return arg.write == write && arg.width == width && arg.offset == offset;
}

MATCHER(IsUnusedBar, "") {
  return arg.type == PciBarType::kNone && !arg.prefetchable && arg.base == 0 &&
         arg.size == 0;
}

// The record of device `index`; if there is none, fails the test and returns
// an empty record, so that the test goes on instead of crashing.
static const PciDevice& DeviceAt(const int index) {
  static const PciDevice kMissing;
  const PciDevice* const device = PciGetDevice(index);
  if (device == nullptr) {
    ADD_FAILURE() << "no device with index " << index;
    return kMissing;
  }
  return *device;
}

// The record of the function at `address`, like DeviceAt(int).
static const PciDevice& DeviceAt(const PciAddress address) {
  const int index = PciFindDevice(address);
  EXPECT_THAT(index, t::Ge(0)) << "no device at " << t::PrintToString(address);
  return DeviceAt(index);
}

class PciTest : public t::Test {
 protected:
  void SetUp() override {
    PciResetForTest();
    g_report.clear();
  }

  void TearDown() override { PciResetForTest(); }

  void Init() { PciInit(fake_); }

  std::vector<Access> AccessesTo(const PciAddress address) const {
    std::vector<Access> result;
    for (const Access& access : fake_->Accesses()) {
      if (access.address == address) {
        result.push_back(access);
      }
    }
    return result;
  }

  std::vector<Access> Writes() const {
    std::vector<Access> result;
    for (const Access& access : fake_->Accesses()) {
      if (access.write) {
        result.push_back(access);
      }
    }
    return result;
  }

  std::vector<Access> WritesTo(const PciAddress address) const {
    std::vector<Access> result;
    for (const Access& access : AccessesTo(address)) {
      if (access.write) {
        result.push_back(access);
      }
    }
    return result;
  }

  // How often the vendor ID of `address` was read with a 16-bit access: once
  // per scan of its bus (for function 0).
  int VendorIdReads(const PciAddress address) const {
    int reads = 0;
    for (const Access& access : AccessesTo(address)) {
      if (!access.write && access.width == 2 &&
          access.offset == kPciVendorIdOffset) {
        ++reads;
      }
    }
    return reads;
  }

  uint16_t Command(const PciAddress address) const {
    return fake_->Peek16(address, kPciCommandOffset);
  }

  std::shared_ptr<FakePciConfigSpace> fake_ =
      std::make_shared<FakePciConfigSpace>();
};

// --- Configuration mechanism #1 ---

TEST(PciMechanism1Test, AddressHasEnableBitBusDeviceFunctionAndDword) {
  EXPECT_THAT(PciMechanism1Address({0, 0, 0}, 0x00), t::Eq(0x80000000u));
  EXPECT_THAT(PciMechanism1Address({1, 1, 0}, 0x10), t::Eq(0x80010810u));
  EXPECT_THAT(PciMechanism1Address({0xFF, 31, 7}, 0xFC), t::Eq(0x80FFFFFCu));
  // The byte within the dword is selected by the data port instead.
  EXPECT_THAT(PciMechanism1Address({0, 4, 0}, 0x06), t::Eq(0x80002004u));
  EXPECT_THAT(PciMechanism1Address({0, 4, 0}, 0x07), t::Eq(0x80002004u));
}

TEST(PciMechanism1Test, DataPortIsTheByteLaneOfTheOffset) {
  EXPECT_THAT(PciMechanism1DataPort(0x00), t::Eq(0xCFC));
  EXPECT_THAT(PciMechanism1DataPort(0x02), t::Eq(0xCFE));
  EXPECT_THAT(PciMechanism1DataPort(0x0E), t::Eq(0xCFE));
  EXPECT_THAT(PciMechanism1DataPort(0x19), t::Eq(0xCFD));
  EXPECT_THAT(PciMechanism1DataPort(0x3F), t::Eq(0xCFF));
}

// --- Enumeration ---

TEST_F(PciTest, EmptyMachineIsScannedOnceWithoutWrites) {
  Init();
  EXPECT_THAT(PciDeviceCount(), t::Eq(0));
  // Bus 0 and every possible peer root bus are scanned once: one vendor ID
  // read per device number, and nothing else.
  const std::vector<Access> accesses = fake_->Accesses();
  EXPECT_THAT(accesses.size(), t::Eq(256u * 32u));
  for (const Access& access : accesses) {
    EXPECT_THAT(access, IsAccess(false, 2, kPciVendorIdOffset));
    EXPECT_THAT(access.address.function, t::Eq(0));
  }
  EXPECT_THAT(fake_->WritesToAbsentFunctions(), t::Eq(0));
  EXPECT_THAT(ReportLines(), t::ElementsAre("[PCI] 0 functions"));
}

TEST_F(PciTest, SingleFunctionDeviceIsNotProbedAtOtherFunctionNumbers) {
  // Some single-function devices answer at every function number.
  for (uint8_t function = 0; function < 8; ++function) {
    fake_->AddFunction({0, 3, function}, EndpointSpec());
  }
  Init();
  ASSERT_THAT(PciDeviceCount(), t::Eq(1));
  EXPECT_FALSE(DeviceAt(0).multi_function);
  for (const Access& access : fake_->Accesses()) {
    EXPECT_THAT(access.address.function, t::Eq(0)) << t::PrintToString(access);
  }
}

TEST_F(PciTest, MultiFunctionDeviceIsScannedAtEveryFunctionNumber) {
  // Like QEMU's PIIX3: functions 0, 1 and 3; function 2 is a hole.
  fake_->AddFunction({0, 1, 0}, MultiFunctionEndpointSpec());
  fake_->AddFunction({0, 1, 1}, EndpointSpec());
  fake_->AddFunction({0, 1, 3}, EndpointSpec());
  Init();
  ASSERT_THAT(PciDeviceCount(), t::Eq(3));
  EXPECT_THAT(DeviceAt(0).address, t::Eq(PciAddress{0, 1, 0}));
  EXPECT_THAT(DeviceAt(1).address, t::Eq(PciAddress{0, 1, 1}));
  EXPECT_THAT(DeviceAt(2).address, t::Eq(PciAddress{0, 1, 3}));
  for (int index = 0; index < 3; ++index) {
    EXPECT_TRUE(DeviceAt(index).multi_function);
  }
  // Absent functions only had their vendor ID read.
  for (const uint8_t function : {2, 4, 5, 6, 7}) {
    EXPECT_THAT(AccessesTo({0, 1, function}),
                t::ElementsAre(IsAccess(false, 2, kPciVendorIdOffset)));
  }
}

TEST_F(PciTest, DeviceWithoutFunctionZeroIsAbsent) {
  fake_->AddFunction({0, 5, 1}, EndpointSpec());
  Init();
  EXPECT_THAT(PciDeviceCount(), t::Eq(0));
  EXPECT_THAT(AccessesTo({0, 5, 1}), t::IsEmpty());
}

TEST_F(PciTest, VendorIdsZeroAndRetryStatusMeanAbsent) {
  FunctionSpec zero = EndpointSpec();
  zero.vendor_id = 0x0000;
  FunctionSpec retry = EndpointSpec();
  retry.vendor_id = 0x0001;
  fake_->AddFunction({0, 2, 0}, zero);
  fake_->AddFunction({0, 3, 0}, retry);
  fake_->AddFunction({0, 4, 0}, EndpointSpec());
  Init();
  ASSERT_THAT(PciDeviceCount(), t::Eq(1));
  EXPECT_THAT(DeviceAt(0).address, t::Eq(PciAddress{0, 4, 0}));
}

TEST_F(PciTest, RecordsIdsClassRevisionAndHeaderType) {
  fake_->AddFunction({0, 7, 0}, {.vendor_id = 0x1234,
                                 .device_id = 0x5678,
                                 .class_code = 0x0C,
                                 .subclass = 0x03,
                                 .prog_if = 0x30,
                                 .revision = 0x9A,
                                 .header_type = 0x80});
  fake_->AddFunction({0, 7, 2}, {.vendor_id = 0xABCD,
                                 .device_id = 0xEF01,
                                 .class_code = 0x01,
                                 .subclass = 0x06,
                                 .prog_if = 0x01,
                                 .revision = 0x02,
                                 .header_type = 0x00});
  Init();
  ASSERT_THAT(PciDeviceCount(), t::Eq(2));
  const PciDevice& first = DeviceAt(0);
  EXPECT_THAT(first.address, t::Eq(PciAddress{0, 7, 0}));
  EXPECT_THAT(first.vendor_id, t::Eq(0x1234));
  EXPECT_THAT(first.device_id, t::Eq(0x5678));
  EXPECT_THAT(first.class_code, t::Eq(0x0C));
  EXPECT_THAT(first.subclass, t::Eq(0x03));
  EXPECT_THAT(first.prog_if, t::Eq(0x30));
  EXPECT_THAT(first.revision, t::Eq(0x9A));
  EXPECT_THAT(first.header_type, t::Eq(kPciHeaderTypeEndpoint));
  EXPECT_TRUE(first.multi_function);
  EXPECT_TRUE(first.claimable);
  EXPECT_THAT(first.parent_index, t::Eq(kPciNoParent));
  const PciDevice& second = DeviceAt(1);
  EXPECT_THAT(second.vendor_id, t::Eq(0xABCD));
  EXPECT_THAT(second.device_id, t::Eq(0xEF01));
  EXPECT_THAT(second.class_code, t::Eq(0x01));
  EXPECT_THAT(second.subclass, t::Eq(0x06));
  EXPECT_THAT(second.prog_if, t::Eq(0x01));
  EXPECT_THAT(second.revision, t::Eq(0x02));
  // The multi-function bit is function 0's.
  EXPECT_TRUE(second.multi_function);
}

TEST_F(PciTest, FunctionBehindBridgeIsFoundWithItsParent) {
  fake_->AddFunction({0, 5, 0}, BridgeSpec());
  fake_->SetBridgeBuses({0, 5, 0}, 0, 1, 1);
  fake_->AddFunction({1, 1, 0}, EndpointSpec());
  Init();
  ASSERT_THAT(PciDeviceCount(), t::Eq(2));
  const PciDevice& bridge = DeviceAt(0);
  EXPECT_THAT(bridge.address, t::Eq(PciAddress{0, 5, 0}));
  EXPECT_THAT(bridge.header_type, t::Eq(kPciHeaderTypeBridge));
  EXPECT_THAT(bridge.secondary_bus, t::Eq(1));
  EXPECT_THAT(bridge.subordinate_bus, t::Eq(1));
  EXPECT_THAT(bridge.parent_index, t::Eq(kPciNoParent));
  EXPECT_FALSE(bridge.claimable);
  const PciDevice& child = DeviceAt(1);
  EXPECT_THAT(child.address, t::Eq(PciAddress{1, 1, 0}));
  EXPECT_THAT(child.parent_index, t::Eq(0));
}

TEST_F(PciTest, BridgeSecondaryBusDecidesTheParentAndOtherBusesAreRoots) {
  fake_->AddFunction({0, 0x1E, 0}, BridgeSpec());
  fake_->SetBridgeBuses({0, 0x1E, 0}, 0, 7, 7);
  fake_->AddFunction({7, 0, 0}, EndpointSpec());
  // No bridge leads to bus 3: a peer root bus, as of a second host bridge.
  fake_->AddFunction({3, 0, 0}, EndpointSpec());
  Init();
  ASSERT_THAT(PciDeviceCount(), t::Eq(3));
  EXPECT_THAT(DeviceAt(0).address, t::Eq(PciAddress{0, 0x1E, 0}));
  EXPECT_THAT(DeviceAt(1).address, t::Eq(PciAddress{7, 0, 0}));
  EXPECT_THAT(DeviceAt(1).parent_index, t::Eq(0));
  EXPECT_THAT(DeviceAt(2).address, t::Eq(PciAddress{3, 0, 0}));
  EXPECT_THAT(DeviceAt(2).parent_index, t::Eq(kPciNoParent));
  EXPECT_THAT(VendorIdReads({7, 0, 0}), t::Eq(1));
}

TEST_F(PciTest, NestedBridgesAreWalkedBusByBusWithParentChain) {
  fake_->AddFunction({0, 5, 0}, BridgeSpec());
  fake_->SetBridgeBuses({0, 5, 0}, 0, 1, 2);
  fake_->AddFunction({0, 6, 0}, EndpointSpec());
  fake_->AddFunction({1, 0, 0}, BridgeSpec());
  fake_->SetBridgeBuses({1, 0, 0}, 1, 2, 2);
  fake_->AddFunction({2, 3, 0}, EndpointSpec());
  Init();
  ASSERT_THAT(PciDeviceCount(), t::Eq(4));
  EXPECT_THAT(DeviceAt(0).address, t::Eq(PciAddress{0, 5, 0}));
  EXPECT_THAT(DeviceAt(1).address, t::Eq(PciAddress{0, 6, 0}));
  EXPECT_THAT(DeviceAt(2).address, t::Eq(PciAddress{1, 0, 0}));
  EXPECT_THAT(DeviceAt(3).address, t::Eq(PciAddress{2, 3, 0}));
  EXPECT_THAT(DeviceAt(0).parent_index, t::Eq(kPciNoParent));
  EXPECT_THAT(DeviceAt(1).parent_index, t::Eq(kPciNoParent));
  EXPECT_THAT(DeviceAt(2).parent_index, t::Eq(0));
  EXPECT_THAT(DeviceAt(3).parent_index, t::Eq(2));
}

TEST_F(PciTest, BridgeLoopsAndInvalidBusNumbersEndTheWalk) {
  fake_->AddFunction({0, 5, 0}, BridgeSpec());
  fake_->SetBridgeBuses({0, 5, 0}, 0, 1, 3);
  // Bus 0 is scanned before bus 1, so 00:06.0 reaches bus 2 first.
  fake_->AddFunction({0, 6, 0}, BridgeSpec());
  fake_->SetBridgeBuses({0, 6, 0}, 0, 2, 2);
  // On bus 1: a bridge back to its own bus, an unconfigured one, and one to
  // bus 2, which is taken.
  fake_->AddFunction({1, 0, 0}, BridgeSpec());
  fake_->SetBridgeBuses({1, 0, 0}, 1, 1, 1);
  fake_->AddFunction({1, 1, 0}, BridgeSpec());
  fake_->SetBridgeBuses({1, 1, 0}, 0, 0, 0);
  fake_->AddFunction({1, 2, 0}, BridgeSpec());
  fake_->SetBridgeBuses({1, 2, 0}, 1, 2, 3);
  // On bus 2: a bridge back up to bus 1, and an endpoint.
  fake_->AddFunction({2, 0, 0}, BridgeSpec());
  fake_->SetBridgeBuses({2, 0, 0}, 2, 1, 1);
  fake_->AddFunction({2, 4, 0}, EndpointSpec());
  Init();

  ASSERT_THAT(PciDeviceCount(), t::Eq(7));
  for (const PciAddress address :
       {PciAddress{0, 5, 0}, PciAddress{0, 6, 0}, PciAddress{1, 0, 0},
        PciAddress{1, 1, 0}, PciAddress{1, 2, 0}, PciAddress{2, 0, 0},
        PciAddress{2, 4, 0}}) {
    EXPECT_THAT(PciFindDevice(address), t::Ge(0)) << t::PrintToString(address);
  }
  for (int bus = 0; bus < 256; ++bus) {
    EXPECT_THAT(VendorIdReads({static_cast<uint8_t>(bus), 0, 0}), t::Eq(1))
        << "bus " << bus;
  }
  EXPECT_THAT(DeviceAt({2, 4, 0}).parent_index,
              t::Eq(PciFindDevice({0, 6, 0})));
  EXPECT_THAT(Writes(), t::IsEmpty());
}

TEST_F(PciTest,
       BridgeWithSecondaryBusNotAboveOwnBusIsNotFollowedToUnvisitedBus) {
  // Bus 0 reaches bus 2 directly while bus 1 is still unvisited; the bridge on
  // bus 2 whose secondary bus is 1 (<= own bus 2) must be ignored by the
  // bridge walk, so bus 1 is only discovered later as a peer root bus.
  fake_->AddFunction({0, 5, 0}, BridgeSpec());
  fake_->SetBridgeBuses({0, 5, 0}, 0, 2, 2);
  fake_->AddFunction({2, 0, 0}, BridgeSpec());
  fake_->SetBridgeBuses({2, 0, 0}, 2, 1, 1);
  fake_->AddFunction({1, 4, 0}, EndpointSpec());
  Init();
  ASSERT_THAT(PciDeviceCount(), t::Eq(3));
  EXPECT_THAT(DeviceAt(0).address, t::Eq(PciAddress{0, 5, 0}));
  EXPECT_THAT(DeviceAt(1).address, t::Eq(PciAddress{2, 0, 0}));
  EXPECT_THAT(DeviceAt(1).parent_index, t::Eq(0));
  EXPECT_THAT(DeviceAt(2).address, t::Eq(PciAddress{1, 4, 0}));
  EXPECT_THAT(DeviceAt(2).parent_index, t::Eq(kPciNoParent));
}

TEST_F(PciTest, SiblingBridgesEachParentTheirOwnBus) {
  fake_->AddFunction({0, 5, 0}, BridgeSpec());
  fake_->SetBridgeBuses({0, 5, 0}, 0, 1, 1);
  fake_->AddFunction({0, 6, 0}, BridgeSpec());
  fake_->SetBridgeBuses({0, 6, 0}, 0, 2, 2);
  fake_->AddFunction({1, 0, 0}, EndpointSpec());
  fake_->AddFunction({2, 0, 0}, EndpointSpec());
  Init();
  ASSERT_THAT(PciDeviceCount(), t::Eq(4));
  EXPECT_THAT(DeviceAt({1, 0, 0}).parent_index,
              t::Eq(PciFindDevice({0, 5, 0})));
  EXPECT_THAT(DeviceAt({2, 0, 0}).parent_index,
              t::Eq(PciFindDevice({0, 6, 0})));
}

TEST_F(PciTest, CardBusBridgeIsRecordedButItsBusIsNeitherScannedNorARoot) {
  FunctionSpec cardbus = BridgeSpec();
  cardbus.subclass = 0x07;
  cardbus.header_type = kPciHeaderTypeCardBus;
  fake_->AddFunction({0, 0x0A, 0}, cardbus);
  fake_->SetBridgeBuses({0, 0x0A, 0}, 0, 3, 3);
  fake_->AddFunction({3, 0, 0}, EndpointSpec());
  Init();
  ASSERT_THAT(PciDeviceCount(), t::Eq(1));
  const PciDevice& device = DeviceAt(0);
  EXPECT_THAT(device.header_type, t::Eq(kPciHeaderTypeCardBus));
  EXPECT_FALSE(device.claimable);
  EXPECT_THAT(device.bars, t::Each(IsUnusedBar()));
  EXPECT_THAT(AccessesTo({3, 0, 0}), t::IsEmpty());
  EXPECT_THAT(Writes(), t::IsEmpty());
}

TEST_F(PciTest, PeerRootBusIsScannedWithTheBusesBehindItsBridges) {
  fake_->AddFunction({0, 0, 0}, EndpointSpec());
  fake_->AddFunction({0x80, 0, 0}, EndpointSpec());
  fake_->AddFunction({0x80, 1, 0}, BridgeSpec());
  fake_->SetBridgeBuses({0x80, 1, 0}, 0x80, 0x81, 0x81);
  fake_->AddFunction({0x81, 0, 0}, EndpointSpec());
  Init();
  ASSERT_THAT(PciDeviceCount(), t::Eq(4));
  EXPECT_THAT(DeviceAt(1).address, t::Eq(PciAddress{0x80, 0, 0}));
  EXPECT_THAT(DeviceAt(1).parent_index, t::Eq(kPciNoParent));
  EXPECT_THAT(DeviceAt(2).address, t::Eq(PciAddress{0x80, 1, 0}));
  EXPECT_THAT(DeviceAt(3).address, t::Eq(PciAddress{0x81, 0, 0}));
  EXPECT_THAT(DeviceAt(3).parent_index, t::Eq(2));
  EXPECT_THAT(VendorIdReads({0x81, 0, 0}), t::Eq(1));
}

TEST_F(PciTest, FunctionsBeyondTheTableAreCountedButNotRecordedOrSized) {
  for (uint8_t device = 0; device < 32; ++device) {
    for (uint8_t function = 0; function < 8; ++function) {
      fake_->AddFunction(
          {0, device, function},
          function == 0 ? MultiFunctionEndpointSpec() : EndpointSpec());
    }
  }
  fake_->AddFunction({1, 0, 0}, EndpointSpec(kPciCommandMemorySpace));
  fake_->SetBar({1, 0, 0},  //
                0,          //
                {.kind = BarKind::kMem32, .base = 0xFE000000, .size = 0x1000});
  Init();
  EXPECT_THAT(PciDeviceCount(), t::Eq(kPciMaxDevices));
  EXPECT_THAT(PciFindDevice({1, 0, 0}), t::Eq(-1));
  EXPECT_THAT(WritesTo({1, 0, 0}), t::IsEmpty());
  const std::vector<std::string> lines = ReportLines();
  ASSERT_THAT(lines.size(), t::Eq(static_cast<size_t>(kPciMaxDevices) + 1));
  EXPECT_THAT(lines.back(),
              t::Eq("[PCI] 256 functions, 1 not recorded (table full)"));
}

TEST_F(PciTest, UnknownHeaderLayoutIsOnlyIdentified) {
  FunctionSpec odd = EndpointSpec(kPciCommandMemorySpace, 0x0010);
  odd.header_type = 0x05;
  fake_->AddFunction({0, 9, 0}, odd);
  Init();
  ASSERT_THAT(PciDeviceCount(), t::Eq(1));
  EXPECT_THAT(DeviceAt(0).header_type, t::Eq(0x05));
  EXPECT_FALSE(DeviceAt(0).claimable);
  for (const Access& access : AccessesTo({0, 9, 0})) {
    EXPECT_FALSE(access.write);
    EXPECT_THAT(access.offset, t::Lt(kPciBar0Offset))
        << t::PrintToString(access);
  }
}

TEST_F(PciTest, ConfigAccessesAreNaturallyAligned) {
  AddQemuPcMachine(fake_.get(), QemuFirmware::kSeaBios, QemuTopology::kMulti);
  Init();
  const int index = PciFindDevice({1, 1, 0});
  ASSERT_THAT(PciClaim(index, kOwnerA), t::Eq(PciStatus::kOk));
  ASSERT_THAT(PciSetBusMaster(index, kOwnerA, true), t::Eq(PciStatus::kOk));
  PciQuiesce(index);
  for (const Access& access : fake_->Accesses()) {
    EXPECT_THAT(access.offset % access.width, t::Eq(0))
        << t::PrintToString(access);
    EXPECT_THAT(access.offset + access.width, t::Le(kPciConfigSpaceSize));
  }
}

TEST_F(PciTest, LookupsOutsideTheTableFindNothing) {
  AddQemuPcMachine(fake_.get(), QemuFirmware::kSeaBios, QemuTopology::kNone);
  Init();
  ASSERT_THAT(PciDeviceCount(), t::Eq(6));
  EXPECT_THAT(PciGetDevice(-1), t::IsNull());
  EXPECT_THAT(PciGetDevice(6), t::IsNull());
  EXPECT_THAT(PciGetDevice(INT_MAX), t::IsNull());
  EXPECT_THAT(PciFindDevice({0, 4, 0}), t::Eq(-1));
  EXPECT_THAT(PciFindDevice({0, 1, 3}), t::Eq(3));
}

TEST_F(PciTest, InitAgainAfterResetRebuildsTheTable) {
  AddQemuPcMachine(fake_.get(), QemuFirmware::kSeaBios, QemuTopology::kMulti);
  Init();
  ASSERT_THAT(PciClaim(PciFindDevice({0, 4, 0}), kOwnerA),
              t::Eq(PciStatus::kOk));
  PciResetForTest();
  EXPECT_THAT(PciDeviceCount(), t::Eq(0));
  EXPECT_THAT(PciClaimedCount(), t::Eq(0));
  fake_ = std::make_shared<FakePciConfigSpace>();
  AddQemuPcMachine(fake_.get(), QemuFirmware::kOvmf, QemuTopology::kSingle);
  Init();
  EXPECT_THAT(PciDeviceCount(), t::Eq(7));
  EXPECT_THAT(PciOwner(PciFindDevice({0, 4, 0})), t::Eq(kPciNoOwner));
}

TEST_F(PciTest, InitChecksItsArgumentAndThatNoDeviceIsClaimed) {
  EXPECT_DEATH(PciInit(nullptr), "Check failed");
  AddQemuPcMachine(fake_.get(), QemuFirmware::kSeaBios, QemuTopology::kSingle);
  Init();
  ASSERT_THAT(PciClaim(PciFindDevice({0, 4, 0}), kOwnerA),
              t::Eq(PciStatus::kOk));
  EXPECT_DEATH(PciInit(fake_), "Check failed: claimed_count == 0");
}

// --- BARs ---

TEST_F(PciTest, Mem32BarIsDecodedAndSized) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0x0103));
  fake_->SetBar({0, 3, 0},  //
                0,          //
                {.kind = BarKind::kMem32, .base = 0xFEA40000, .size = 0x20000});
  Init();
  ASSERT_THAT(PciDeviceCount(), t::Eq(1));
  const PciDevice& device = DeviceAt(0);
  EXPECT_THAT(device.bars[0],
              IsBar(PciBarType::kMem32, false, 0xFEA40000, 0x20000));
  for (int i = 1; i < kPciMaxBars; ++i) {
    EXPECT_THAT(device.bars[i], IsUnusedBar()) << "BAR " << i;
  }
}

TEST_F(PciTest, PrefetchableMem32Bar) {
  fake_->AddFunction({0, 2, 0}, EndpointSpec(0x0103));
  fake_->SetBar({0, 2, 0},  //
                0,          //
                {.kind = BarKind::kMem32,
                 .base = 0xFD000000,
                 .size = 0x1000000,
                 .prefetchable = true});
  Init();
  EXPECT_THAT(DeviceAt(0).bars[0],
              IsBar(PciBarType::kMem32, true, 0xFD000000, 0x1000000));
}

TEST_F(PciTest, Mem64BarTakesTwoSlots) {
  fake_->AddFunction({0, 4, 0}, EndpointSpec(0x0107));
  fake_->SetBar({0, 4, 0},  //
                0,          //
                {.kind = BarKind::kMem64, .base = 0xFEA70000, .size = 0x4000});
  fake_->SetBar({0, 4, 0},  //
                2,          //
                {.kind = BarKind::kMem32, .base = 0xFEB00000, .size = 0x1000});
  Init();
  const PciDevice& device = DeviceAt(0);
  EXPECT_THAT(device.bars[0],
              IsBar(PciBarType::kMem64, false, 0xFEA70000, 0x4000));
  EXPECT_THAT(device.bars[1], IsUnusedBar());
  EXPECT_THAT(device.bars[2],
              IsBar(PciBarType::kMem32, false, 0xFEB00000, 0x1000));
}

TEST_F(PciTest, Mem64BarAbove4GiB) {
  fake_->AddFunction({0, 4, 0}, EndpointSpec(0x0007));
  fake_->SetBar(
      {0, 4, 0},  //
      0,          //
      {.kind = BarKind::kMem64, .base = 0xC020000000, .size = 0x4000});
  Init();
  EXPECT_THAT(DeviceAt(0).bars[0],
              IsBar(PciBarType::kMem64, false, 0xC020000000, 0x4000));
}

TEST_F(PciTest, Mem64BarWhoseLowAddressDwordIsZeroIsAssigned) {
  // OVMF with one root xHCI: BAR0 low dword 0x00000004, high dword 0xC0.
  fake_->AddFunction({0, 4, 0}, EndpointSpec(0x0007));
  fake_->SetBar(
      {0, 4, 0},  //
      0,          //
      {.kind = BarKind::kMem64, .base = 0xC000000000, .size = 0x4000});
  Init();
  EXPECT_THAT(fake_->Peek32({0, 4, 0}, kPciBar0Offset), t::Eq(0x00000004u));
  EXPECT_THAT(DeviceAt(0).bars[0],
              IsBar(PciBarType::kMem64, false, 0xC000000000, 0x4000));
}

TEST_F(PciTest, Mem64BarLargerThan4GiB) {
  fake_->AddFunction({0, 4, 0}, EndpointSpec(0x0002));
  fake_->SetBar({0, 4, 0},  //
                2,          //
                {.kind = BarKind::kMem64,
                 .base = 0x400000000,
                 .size = 0x200000000,
                 .prefetchable = true});
  Init();
  EXPECT_THAT(DeviceAt(0).bars[2],
              IsBar(PciBarType::kMem64, true, 0x400000000, 0x200000000));
}

TEST_F(PciTest, IoBar) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0x0103));
  fake_->SetBar({0, 3, 0},  //
                1,          //
                {.kind = BarKind::kIo, .base = 0xD000, .size = 0x40});
  Init();
  EXPECT_THAT(DeviceAt(0).bars[1], IsBar(PciBarType::kIo, false, 0xD000, 0x40));
}

TEST_F(PciTest, IoBarWithSixteenBitDecoder) {
  // Bits 31:16 read 0 after all ones are written.
  fake_->AddFunction({0, 1, 0}, EndpointSpec(0x0001));
  fake_->SetBar({0, 1, 0},  //
                4,          //
                {.kind = BarKind::kIo,
                 .base = 0xC040,
                 .size = 0x10,
                 .io_16bit_decode = true});
  Init();
  EXPECT_THAT(DeviceAt(0).bars[4], IsBar(PciBarType::kIo, false, 0xC040, 0x10));
}

TEST_F(PciTest, SmallIoBars) {
  // Like the 4-byte control block of a legacy IDE channel, and a UART.
  fake_->AddFunction({0, 1, 0}, EndpointSpec(0x0001));
  fake_->SetBar({0, 1, 0},  //
                0,          //
                {.kind = BarKind::kIo, .base = 0xD004, .size = 0x4});
  fake_->SetBar({0, 1, 0},  //
                1,          //
                {.kind = BarKind::kIo, .base = 0xD008, .size = 0x8});
  Init();
  EXPECT_THAT(DeviceAt(0).bars[0], IsBar(PciBarType::kIo, false, 0xD004, 0x4));
  EXPECT_THAT(DeviceAt(0).bars[1], IsBar(PciBarType::kIo, false, 0xD008, 0x8));
}

TEST_F(PciTest, SmallestMemoryBar) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0x0002));
  fake_->SetBar({0, 3, 0},  //
                0,          //
                {.kind = BarKind::kMem32, .base = 0xFEB00010, .size = 0x10});
  Init();
  EXPECT_THAT(DeviceAt(0).bars[0],
              IsBar(PciBarType::kMem32, false, 0xFEB00010, 0x10));
}

TEST_F(PciTest, FunctionWithoutBarsIsNotWritten) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0x0103));
  Init();
  EXPECT_THAT(DeviceAt(0).bars, t::Each(IsUnusedBar()));
  EXPECT_THAT(Writes(), t::IsEmpty());
}

TEST_F(PciTest, UnassignedBarsAreRecordedButNotSized) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0x0103));
  fake_->SetBar({0, 3, 0},  //
                0,          //
                {.kind = BarKind::kMem64, .base = 0, .size = 0x4000});
  fake_->SetBar({0, 3, 0}, 2, {.kind = BarKind::kIo, .base = 0, .size = 0x20});
  Init();
  const PciDevice& device = DeviceAt(0);
  EXPECT_THAT(device.bars[0], IsBar(PciBarType::kMem64, false, 0, 0));
  EXPECT_THAT(device.bars[2], IsBar(PciBarType::kIo, false, 0, 0));
  EXPECT_THAT(Writes(), t::IsEmpty());
}

TEST_F(PciTest, BridgeBarsAreDecodedButNeitherSizedNorWritten) {
  fake_->AddFunction({0, 5, 0}, BridgeSpec(0x0103, 0x00B0));
  fake_->SetBar({0, 5, 0},  //
                0,          //
                {.kind = BarKind::kMem64, .base = 0xFEA75000, .size = 0x100});
  fake_->SetBridgeBuses({0, 5, 0}, 0, 1, 1);
  Init();
  const PciDevice& bridge = DeviceAt({0, 5, 0});
  EXPECT_THAT(bridge.bars[0], IsBar(PciBarType::kMem64, false, 0xFEA75000, 0));
  for (int i = 1; i < kPciMaxBars; ++i) {
    EXPECT_THAT(bridge.bars[i], IsUnusedBar()) << "BAR " << i;
  }
  EXPECT_THAT(Writes(), t::IsEmpty());
}

TEST_F(PciTest, BridgeClassEndpointsAreNeitherSizedNorClaimable) {
  // A host bridge (class 06h, subclass 00h) with a header type 0.
  FunctionSpec host_bridge = EndpointSpec(0x0006);
  host_bridge.class_code = kPciClassBridge;
  fake_->AddFunction({0, 0, 0}, host_bridge);
  fake_->SetBar({0, 0, 0},  //
                0,          //
                {.kind = BarKind::kMem32, .base = 0xFED00000, .size = 0x1000});
  Init();
  const PciDevice& device = DeviceAt(0);
  EXPECT_FALSE(device.claimable);
  EXPECT_THAT(device.bars[0], IsBar(PciBarType::kMem32, false, 0xFED00000, 0));
  EXPECT_THAT(Writes(), t::IsEmpty());
}

TEST_F(PciTest, DecodingIsOffWhileBarsAreSizedAndCommandIsRestored) {
  for (const uint16_t command :
       {0x0000, 0x0001, 0x0002, 0x0007, 0x0103, 0x0107, 0x0547}) {
    SCOPED_TRACE(t::Message() << "command 0x" << std::hex << command);
    PciResetForTest();
    fake_ = std::make_shared<FakePciConfigSpace>();
    const PciAddress address = {0, 3, 0};
    fake_->AddFunction(address, EndpointSpec(command, kStatusErrorBits));
    fake_->SetBar(
        address,  //
        0,        //
        {.kind = BarKind::kMem32, .base = 0xFEA40000, .size = 0x20000});
    fake_->SetBar(address,  //
                  1,        //
                  {.kind = BarKind::kIo, .base = 0xD000, .size = 0x40});
    fake_->SetBar(
        address,  //
        2,        //
        {.kind = BarKind::kMem64, .base = 0xC020000000, .size = 0x4000});
    Init();

    EXPECT_THAT(fake_->BarWritesWhileDecoding(), t::Eq(0));
    EXPECT_THAT(Command(address), t::Eq(command));
    EXPECT_THAT(fake_->Peek16(address, kPciStatusOffset),
                t::Eq(kStatusErrorBits));
    std::vector<uint32_t> command_writes;
    for (const Access& access : Writes()) {
      if (access.offset <= kPciStatusOffset + 1 &&
          access.offset + access.width > kPciCommandOffset) {
        EXPECT_THAT(access, IsAccess(true, 2, kPciCommandOffset));
        command_writes.push_back(access.value);
      }
    }
    const uint16_t decode = kPciCommandIoSpace | kPciCommandMemorySpace;
    if ((command & decode) != 0) {
      EXPECT_THAT(
          command_writes,
          t::ElementsAre(static_cast<uint16_t>(command & ~decode), command));
    } else {
      EXPECT_THAT(command_writes, t::IsEmpty());
    }
    const PciDevice& device = DeviceAt(0);
    EXPECT_THAT(device.bars[0],
                IsBar(PciBarType::kMem32, false, 0xFEA40000, 0x20000));
    EXPECT_THAT(device.bars[1], IsBar(PciBarType::kIo, false, 0xD000, 0x40));
    EXPECT_THAT(device.bars[2],
                IsBar(PciBarType::kMem64, false, 0xC020000000, 0x4000));
  }
}

TEST_F(PciTest, EnumerationLeavesQemuMachinesUnchanged) {
  const PciAddress kAddresses[] = {{0, 0, 0}, {0, 1, 0}, {0, 1, 1},
                                   {0, 1, 3}, {0, 2, 0}, {0, 3, 0},
                                   {0, 4, 0}, {0, 5, 0}, {1, 1, 0}};
  for (const QemuFirmware firmware :
       {QemuFirmware::kSeaBios, QemuFirmware::kOvmf}) {
    for (const QemuTopology topology :
         {QemuTopology::kNone, QemuTopology::kSingle, QemuTopology::kMulti}) {
      SCOPED_TRACE(t::Message() << "firmware " << static_cast<int>(firmware)
                                << " topology " << static_cast<int>(topology));
      PciResetForTest();
      fake_ = std::make_shared<FakePciConfigSpace>();
      AddQemuPcMachine(fake_.get(), firmware, topology);
      std::vector<uint32_t> before;
      for (const PciAddress address : kAddresses) {
        for (int offset = 0; offset < kPciConfigSpaceSize; offset += 4) {
          before.push_back(fake_->Peek32(address, offset));
        }
      }
      Init();
      std::vector<uint32_t> after;
      for (const PciAddress address : kAddresses) {
        for (int offset = 0; offset < kPciConfigSpaceSize; offset += 4) {
          after.push_back(fake_->Peek32(address, offset));
        }
      }
      EXPECT_THAT(after, t::Eq(before));
      EXPECT_THAT(fake_->BarWritesWhileDecoding(), t::Eq(0));
    }
  }
}

TEST_F(PciTest, Mem64BarInTheLastSlotIsNotDecodable) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0x0103));
  fake_->SetBar({0, 3, 0},  //
                0,          //
                {.kind = BarKind::kMem32, .base = 0xFEA40000, .size = 0x1000});
  // BAR5 claims to be 64-bit: its upper half would be the CardBus CIS pointer.
  fake_->SetRaw32({0, 3, 0}, kPciBar0Offset + 5 * 4, 0xFEB00004);
  Init();
  EXPECT_THAT(DeviceAt(0).bars[5], IsUnusedBar());
  for (const Access& access : AccessesTo({0, 3, 0})) {
    EXPECT_THAT(access.offset, t::Ne(0x28)) << t::PrintToString(access);
  }
}

TEST_F(PciTest, ReservedMemoryTypeIsNotDecodable) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0x0103));
  fake_->SetRaw32({0, 3, 0}, kPciBar0Offset, 0xFEB00006);
  fake_->SetBar({0, 3, 0},  //
                1,          //
                {.kind = BarKind::kMem32, .base = 0xFEA40000, .size = 0x1000});
  Init();
  EXPECT_THAT(DeviceAt(0).bars[0], IsUnusedBar());
  for (const Access& access : WritesTo({0, 3, 0})) {
    EXPECT_THAT(access.offset, t::Ne(kPciBar0Offset));
  }
}

TEST_F(PciTest, BarWhoseSizeDoesNotFitInt64IsNotDecodable) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0x0002));
  fake_->SetBar({0, 3, 0},  //
                0,          //
                {.kind = BarKind::kMem64,
                 .base = 0x8000000000000000u,
                 .size = 0x8000000000000000u});
  Init();
  EXPECT_THAT(DeviceAt(0).bars[0], IsUnusedBar());
}

TEST_F(PciTest, StatusErrorBitsSurviveEnumeration) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0x0103, kStatusErrorBits));
  fake_->SetBar({0, 3, 0},  //
                0,          //
                {.kind = BarKind::kMem32, .base = 0xFEA40000, .size = 0x1000});
  Init();
  EXPECT_THAT(fake_->Peek16({0, 3, 0}, kPciStatusOffset),
              t::Eq(kStatusErrorBits));
  for (const Access& access : Writes()) {
    EXPECT_FALSE(access.offset <= kPciStatusOffset + 1 &&
                 access.offset + access.width > kPciStatusOffset)
        << t::PrintToString(access);
  }
}

// --- Capabilities ---

TEST_F(PciTest, CapabilityListIsIgnoredWithoutTheStatusBit) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec());
  fake_->SetRaw8({0, 3, 0}, kPciCapabilitiesPointerOffset, 0x40);
  fake_->SetRaw16({0, 3, 0}, 0x40, kPciCapabilityMsi);
  Init();
  EXPECT_THAT(DeviceAt(0).msi.offset, t::Eq(0));
  for (const Access& access : AccessesTo({0, 3, 0})) {
    EXPECT_THAT(access.offset, t::Lt(0x40)) << t::PrintToString(access);
  }
}

TEST_F(PciTest, Msi64BitWithoutMasking) {
  fake_->AddFunction({1, 1, 0}, EndpointSpec());
  fake_->AddMsi(
      {1, 1, 0},  //
      0x70,       //
      {.is_64bit = true, .per_vector_masking = false, .vectors_log2 = 4});
  Init();
  const PciMsi& msi = DeviceAt(0).msi;
  EXPECT_THAT(msi.offset, t::Eq(0x70));
  EXPECT_TRUE(msi.is_64bit);
  EXPECT_FALSE(msi.per_vector_masking);
  EXPECT_THAT(msi.vectors_log2, t::Eq(4));
  EXPECT_THAT(msi.data_offset, t::Eq(0x7C));
  EXPECT_THAT(msi.mask_offset, t::Eq(0));
  EXPECT_THAT(DeviceAt(0).msix.offset, t::Eq(0));
}

TEST_F(PciTest, Msi32BitLayouts) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec());
  fake_->AddMsi({0, 3, 0}, 0x50, {.is_64bit = false});
  fake_->AddFunction({0, 4, 0}, EndpointSpec());
  fake_->AddMsi(
      {0, 4, 0},  //
      0x60,       //
      {.is_64bit = false, .per_vector_masking = true, .vectors_log2 = 1});
  Init();
  const PciMsi& plain = DeviceAt({0, 3, 0}).msi;
  EXPECT_THAT(plain.offset, t::Eq(0x50));
  EXPECT_FALSE(plain.is_64bit);
  EXPECT_THAT(plain.data_offset, t::Eq(0x58));
  EXPECT_THAT(plain.mask_offset, t::Eq(0));
  const PciMsi& masked = DeviceAt({0, 4, 0}).msi;
  EXPECT_TRUE(masked.per_vector_masking);
  EXPECT_THAT(masked.vectors_log2, t::Eq(1));
  EXPECT_THAT(masked.data_offset, t::Eq(0x68));
  EXPECT_THAT(masked.mask_offset, t::Eq(0x6C));
}

TEST_F(PciTest, Msi64BitWithMaskingOnABridge) {
  // QEMU's pci-bridge: MSI at 0x4C, then slot ID and SHPC capabilities.
  fake_->AddFunction({0, 5, 0}, BridgeSpec());
  fake_->SetBridgeBuses({0, 5, 0}, 0, 1, 1);
  fake_->AddMsi(
      {0, 5, 0},  //
      0x4C,       //
      {.is_64bit = true, .per_vector_masking = true, .vectors_log2 = 0});
  fake_->AddCapability({0, 5, 0}, 0x48, 0x04);
  fake_->AddCapability({0, 5, 0}, 0x40, 0x0C);
  Init();
  const PciMsi& msi = DeviceAt(0).msi;
  EXPECT_THAT(msi.offset, t::Eq(0x4C));
  EXPECT_TRUE(msi.is_64bit);
  EXPECT_TRUE(msi.per_vector_masking);
  EXPECT_THAT(msi.data_offset, t::Eq(0x58));
  EXPECT_THAT(msi.mask_offset, t::Eq(0x5C));
}

TEST_F(PciTest, MsixTableAndPbaAreValidatedAgainstTheBar) {
  fake_->AddFunction({0, 4, 0}, EndpointSpec(0x0107));
  fake_->SetBar({0, 4, 0},  //
                0,          //
                {.kind = BarKind::kMem64, .base = 0xFEA70000, .size = 0x4000});
  fake_->AddMsix({0, 4, 0},  //
                 0x90,       //
                 {.table_size = 16,
                  .table_bar = 0,
                  .table_offset = 0x3000,
                  .pba_bar = 0,
                  .pba_offset = 0x3800});
  Init();
  const PciMsix& msix = DeviceAt(0).msix;
  EXPECT_THAT(msix.offset, t::Eq(0x90));
  EXPECT_THAT(msix.table_size, t::Eq(16));
  EXPECT_THAT(msix.table_bar, t::Eq(0));
  EXPECT_THAT(msix.table_offset, t::Eq(0x3000u));
  EXPECT_THAT(msix.pba_bar, t::Eq(0));
  EXPECT_THAT(msix.pba_offset, t::Eq(0x3800u));
  EXPECT_TRUE(msix.valid);
  EXPECT_THAT(msix.table_phys, t::Eq(0xFEA73000u));
  EXPECT_THAT(msix.table_bytes, t::Eq(16 * 16));
  EXPECT_THAT(msix.pba_phys, t::Eq(0xFEA73800u));
  EXPECT_THAT(msix.pba_bytes, t::Eq(8));
}

TEST_F(PciTest, MsixTableAndPbaInOtherBars) {
  fake_->AddFunction({0, 4, 0}, EndpointSpec(0x0002));
  fake_->SetBar({0, 4, 0},  //
                2,          //
                {.kind = BarKind::kMem32, .base = 0xFE000000, .size = 0x10000});
  fake_->SetBar(
      {0, 4, 0},  //
      4,          //
      {.kind = BarKind::kMem64, .base = 0xC000000000, .size = 0x4000});
  fake_->AddMsix({0, 4, 0},  //
                 0xB0,       //
                 {.table_size = 65,
                  .table_bar = 2,
                  .table_offset = 0x2000,
                  .pba_bar = 4,
                  .pba_offset = 0x1000});
  Init();
  const PciMsix& msix = DeviceAt(0).msix;
  EXPECT_TRUE(msix.valid);
  EXPECT_THAT(msix.table_bar, t::Eq(2));
  EXPECT_THAT(msix.table_offset, t::Eq(0x2000u));
  EXPECT_THAT(msix.pba_bar, t::Eq(4));
  EXPECT_THAT(msix.pba_offset, t::Eq(0x1000u));
  EXPECT_THAT(msix.table_phys, t::Eq(0xFE002000u));
  EXPECT_THAT(msix.table_bytes, t::Eq(65 * 16));
  EXPECT_THAT(msix.pba_phys, t::Eq(0xC000001000u));
  // 65 vectors need two 64-bit PBA entries.
  EXPECT_THAT(msix.pba_bytes, t::Eq(16));
}

TEST_F(PciTest, MsixTableSizeIgnoresTheEnableAndMaskBits) {
  fake_->AddFunction({0, 4, 0}, EndpointSpec(0x0002));
  fake_->SetBar({0, 4, 0},  //
                0,          //
                {.kind = BarKind::kMem32, .base = 0xFE000000, .size = 0x10000});
  fake_->AddMsix({0, 4, 0},  //
                 0x90,       //
                 {.table_size = 2048,
                  .table_bar = 0,
                  .table_offset = 0x0,
                  .pba_bar = 0,
                  .pba_offset = 0x8000});
  fake_->SetRaw16({0, 4, 0}, 0x92, 0xC7FF);
  Init();
  const PciMsix& msix = DeviceAt(0).msix;
  EXPECT_THAT(msix.table_size, t::Eq(2048));
  EXPECT_THAT(msix.table_bytes, t::Eq(2048 * 16));
  EXPECT_THAT(msix.pba_bytes, t::Eq(256));
  EXPECT_TRUE(msix.valid);
}

TEST_F(PciTest, CapabilitiesAreFoundInAnyListOrder) {
  // QEMU's nec-usb-xhci on PCI Express: MSI-X at 0x90, PCIe at 0xA0, MSI at
  // 0x70, in that list order.
  fake_->AddFunction({0, 4, 0}, EndpointSpec(0x0002));
  fake_->SetBar({0, 4, 0},  //
                0,          //
                {.kind = BarKind::kMem64, .base = 0xFEA70000, .size = 0x4000});
  fake_->AddMsix({0, 4, 0},  //
                 0x90,       //
                 {.table_size = 16,
                  .table_bar = 0,
                  .table_offset = 0x3000,
                  .pba_bar = 0,
                  .pba_offset = 0x3800});
  fake_->AddCapability({0, 4, 0}, 0xA0, 0x10);
  fake_->AddMsi({0, 4, 0}, 0x70, {.is_64bit = true, .vectors_log2 = 4});
  Init();
  EXPECT_THAT(DeviceAt(0).msix.offset, t::Eq(0x90));
  EXPECT_THAT(DeviceAt(0).msi.offset, t::Eq(0x70));
}

TEST_F(PciTest, LoopingCapabilityListEnds) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0, 0x0010));
  fake_->SetRaw8({0, 3, 0}, kPciCapabilitiesPointerOffset, 0x40);
  fake_->SetRaw16({0, 3, 0}, 0x40, 0x5009);  // vendor-specific, next 0x50
  fake_->SetRaw16({0, 3, 0}, 0x50, 0x4009);  // vendor-specific, next 0x40
  Init();
  int capability_reads = 0;
  for (const Access& access : AccessesTo({0, 3, 0})) {
    if (access.offset == 0x40 || access.offset == 0x50) {
      ++capability_reads;
    }
  }
  EXPECT_THAT(capability_reads, t::Eq((kPciConfigSpaceSize - 0x40) / 4));
  EXPECT_THAT(DeviceAt(0).msi.offset, t::Eq(0));
}

TEST_F(PciTest, CapabilityPointerLowBitsAreIgnored) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0, 0x0010));
  fake_->SetRaw8({0, 3, 0}, kPciCapabilitiesPointerOffset, 0x93);
  fake_->SetRaw16({0, 3, 0}, 0x90, 0x7309);  // vendor-specific, next 0x73
  fake_->SetRaw16({0, 3, 0}, 0x70, kPciCapabilityMsi);
  fake_->SetRaw16({0, 3, 0}, 0x72, kPciMsiControl64Bit);
  Init();
  EXPECT_THAT(DeviceAt(0).msi.offset, t::Eq(0x70));
  EXPECT_TRUE(DeviceAt(0).msi.is_64bit);
}

TEST_F(PciTest, CapabilityPointerIntoTheHeaderEndsTheList) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0, 0x0010));
  fake_->SetRaw8({0, 3, 0}, kPciCapabilitiesPointerOffset, 0x40);
  fake_->SetRaw16({0, 3, 0}, 0x40, 0x1009);  // vendor-specific, next 0x10
  fake_->SetRaw16({0, 3, 0}, 0x10, kPciCapabilityMsi);
  Init();
  EXPECT_THAT(DeviceAt(0).msi.offset, t::Eq(0));
  for (const Access& access : AccessesTo({0, 3, 0})) {
    EXPECT_FALSE(access.offset == kPciBar0Offset && access.width == 2);
  }
}

TEST_F(PciTest, CapabilitiesThatDoNotFitAreIgnored) {
  // A 64-bit MSI capability at 0xF8 would end at 0x106; an MSI-X capability
  // at 0xFC at 0x108.
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0, 0x0010));
  fake_->SetRaw8({0, 3, 0}, kPciCapabilitiesPointerOffset, 0xF8);
  fake_->SetRaw16({0, 3, 0}, 0xF8, 0xFC00 | kPciCapabilityMsi);
  fake_->SetRaw16({0, 3, 0}, 0xFA, kPciMsiControl64Bit);
  fake_->SetRaw16({0, 3, 0}, 0xFC, kPciCapabilityMsix);
  Init();
  EXPECT_THAT(DeviceAt(0).msi.offset, t::Eq(0));
  EXPECT_THAT(DeviceAt(0).msix.offset, t::Eq(0));
}

TEST_F(PciTest, MsiCapabilitySizeBoundariesFor64BitAndPerVectorMasking) {
  const auto add_raw_msi = [this](const uint8_t device,  //
                                  const int offset,      //
                                  const uint16_t control) {
    fake_->AddFunction({0, device, 0},
                       EndpointSpec(0, kPciStatusCapabilitiesList));
    fake_->SetRaw8({0, device, 0}, kPciCapabilitiesPointerOffset,
                   static_cast<uint8_t>(offset));
    fake_->SetRaw16({0, device, 0}, offset, kPciCapabilityMsi);
    fake_->SetRaw16({0, device, 0}, offset + kPciMsiControlOffset, control);
  };
  // 32-bit without PVM (10 bytes): fits at 0xF4 (ends 0xFE), overflows at 0xF8.
  add_raw_msi(1, 0xF4, 0);
  add_raw_msi(2, 0xF8, 0);
  // 64-bit without PVM (14 bytes): fits at 0xF0 (ends 0xFE), overflows at 0xF4.
  add_raw_msi(3, 0xF0, kPciMsiControl64Bit);
  add_raw_msi(4, 0xF4, kPciMsiControl64Bit);
  // 32-bit with PVM (20 bytes): fits at 0xEC (ends 0x100), overflows at 0xF0.
  add_raw_msi(5, 0xEC, kPciMsiControlPerVectorMasking);
  add_raw_msi(6, 0xF0, kPciMsiControlPerVectorMasking);
  // 64-bit with PVM (24 bytes): fits at 0xE8 (ends 0x100), overflows at 0xEC.
  add_raw_msi(7, 0xE8, kPciMsiControl64Bit | kPciMsiControlPerVectorMasking);
  add_raw_msi(8, 0xEC, kPciMsiControl64Bit | kPciMsiControlPerVectorMasking);
  Init();
  EXPECT_THAT(DeviceAt({0, 1, 0}).msi.offset, t::Eq(0xF4));
  EXPECT_THAT(DeviceAt({0, 2, 0}).msi.offset, t::Eq(0));
  EXPECT_THAT(DeviceAt({0, 3, 0}).msi.offset, t::Eq(0xF0));
  EXPECT_THAT(DeviceAt({0, 4, 0}).msi.offset, t::Eq(0));
  EXPECT_THAT(DeviceAt({0, 5, 0}).msi.offset, t::Eq(0xEC));
  EXPECT_THAT(DeviceAt({0, 6, 0}).msi.offset, t::Eq(0));
  EXPECT_THAT(DeviceAt({0, 7, 0}).msi.offset, t::Eq(0xE8));
  EXPECT_THAT(DeviceAt({0, 8, 0}).msi.offset, t::Eq(0));
}

TEST_F(PciTest, OnlyTheFirstMsiCapabilityCountsAndReservedVectorCountsAreOne) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec());
  fake_->AddMsi({0, 3, 0}, 0x50, {.is_64bit = false, .vectors_log2 = 6});
  fake_->AddMsi({0, 3, 0}, 0x60, {.is_64bit = true, .vectors_log2 = 2});
  Init();
  EXPECT_THAT(DeviceAt(0).msi.offset, t::Eq(0x50));
  EXPECT_THAT(DeviceAt(0).msi.vectors_log2, t::Eq(0));
}

TEST_F(PciTest, MsixWithTableOrPbaOutsideItsBarsIsInvalid) {
  const auto add = [this](const uint8_t device,
                          const FakePciConfigSpace::MsixSpec& msix) {
    fake_->AddFunction({0, device, 0}, EndpointSpec(0x0002));
    fake_->SetBar(
        {0, device, 0},  //
        0,               //
        {.kind = BarKind::kMem64, .base = 0xFEA70000, .size = 0x4000});
    fake_->SetBar({0, device, 0},  //
                  2,               //
                  {.kind = BarKind::kIo, .base = 0xD000, .size = 0x40});
    fake_->SetBar({0, device, 0},  //
                  3,               //
                  {.kind = BarKind::kMem32, .base = 0, .size = 0x1000});
    fake_->AddMsix({0, device, 0}, 0x90, msix);
  };
  // In the I/O BAR, the upper half of the 64-bit BAR, and an unassigned BAR.
  add(1, {.table_size = 4, .table_bar = 2, .pba_bar = 0, .pba_offset = 0x800});
  add(2, {.table_size = 4, .table_bar = 1, .pba_bar = 0, .pba_offset = 0x800});
  add(3, {.table_size = 4, .table_bar = 3, .pba_bar = 0, .pba_offset = 0x800});
  // Past the end of BAR0, with the table and with the PBA.
  add(4, {.table_size = 4, .table_offset = 0x3FC8, .pba_offset = 0x800});
  add(5, {.table_size = 4, .table_offset = 0x0, .pba_offset = 0x4000});
  // Table and PBA overlap.
  add(6, {.table_size = 4, .table_offset = 0x0, .pba_offset = 0x38});
  // A BIR that names no BAR.
  add(7, {.table_size = 4, .table_offset = 0x0, .pba_offset = 0x800});
  fake_->SetRaw32({0, 7, 0}, 0x94, 0x6);
  // Valid geometry, for comparison.
  add(8, {.table_size = 4, .table_offset = 0x0, .pba_offset = 0x40});
  Init();
  for (uint8_t device = 1; device <= 7; ++device) {
    const PciMsix& msix = DeviceAt({0, device, 0}).msix;
    EXPECT_THAT(msix.offset, t::Eq(0x90)) << "device " << int{device};
    EXPECT_FALSE(msix.valid) << "device " << int{device};
    EXPECT_THAT(msix.table_phys, t::Eq(0u)) << "device " << int{device};
    EXPECT_THAT(msix.pba_phys, t::Eq(0u)) << "device " << int{device};
  }
  EXPECT_TRUE(DeviceAt({0, 8, 0}).msix.valid);
  const std::vector<std::string> lines = ReportLines();
  EXPECT_THAT(lines[0], t::HasSubstr(" msix-ignored"));
  EXPECT_THAT(lines[7], t::HasSubstr(" msix=4"));
}

// --- Configuration access and command helpers ---

TEST_F(PciTest, UpdateCommandIsOne16BitReadModifyWriteWithReadBack) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0x0103, kStatusErrorBits));
  Init();
  fake_->ClearAccesses();
  EXPECT_THAT(PciUpdateCommand(0, kPciCommandBusMaster, kPciCommandIoSpace),
              t::Eq(0x0106));
  EXPECT_THAT(fake_->Accesses(),
              t::ElementsAre(IsAccess(false, 2, kPciCommandOffset),
                             IsAccess(true, 2, kPciCommandOffset),
                             IsAccess(false, 2, kPciCommandOffset)));
  EXPECT_THAT(Writes()[0].value, t::Eq(0x0106u));
  EXPECT_THAT(fake_->Peek16({0, 3, 0}, kPciStatusOffset),
              t::Eq(kStatusErrorBits));
}

TEST_F(PciTest, UpdateThatChangesNothingDoesNotWrite) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0x0106));
  Init();
  fake_->ClearAccesses();
  EXPECT_THAT(PciUpdateCommand(0, kPciCommandMemorySpace, kPciCommandIoSpace),
              t::Eq(0x0106));
  EXPECT_THAT(Writes(), t::IsEmpty());
}

TEST_F(PciTest, ConfigReadsAndWritesReachTheFunctionWithTheirWidth) {
  AddQemuPcMachine(fake_.get(), QemuFirmware::kOvmf, QemuTopology::kMulti);
  Init();
  const int index = PciFindDevice({1, 1, 0});
  fake_->ClearAccesses();
  EXPECT_THAT(PciConfigRead32(index, 0x00), t::Eq(0x01941033u));
  EXPECT_THAT(PciConfigRead16(index, kPciDeviceIdOffset), t::Eq(0x0194));
  EXPECT_THAT(PciConfigRead8(index, kPciRevisionOffset), t::Eq(0x03));
  PciConfigWrite8(index, 0x3C, 0x0B);
  PciConfigWrite32(index, 0x74, 0xFEE00000);
  PciConfigWrite16(index, 0x7C, 0x4041);
  EXPECT_THAT(fake_->Peek8({1, 1, 0}, 0x3C), t::Eq(0x0B));
  EXPECT_THAT(fake_->Peek32({1, 1, 0}, 0x74), t::Eq(0xFEE00000u));
  EXPECT_THAT(fake_->Peek16({1, 1, 0}, 0x7C), t::Eq(0x4041));
  const std::vector<Access> accesses = fake_->Accesses();
  ASSERT_THAT(accesses.size(), t::Eq(6u));
  EXPECT_THAT(accesses,
              t::ElementsAre(IsAccess(false, 4, 0x00),
                             IsAccess(false, 2, kPciDeviceIdOffset),
                             IsAccess(false, 1, kPciRevisionOffset),
                             IsAccess(true, 1, 0x3C), IsAccess(true, 4, 0x74),
                             IsAccess(true, 2, 0x7C)));
  for (const Access& access : accesses) {
    EXPECT_THAT(access.address, t::Eq(PciAddress{1, 1, 0}));
  }
}

TEST_F(PciTest, ConfigUpdate16WorksOnCapabilityRegisters) {
  AddQemuPcMachine(fake_.get(), QemuFirmware::kSeaBios, QemuTopology::kMulti);
  Init();
  const int index = PciFindDevice({1, 1, 0});
  const PciMsi& msi = DeviceAt(index).msi;
  EXPECT_THAT(PciConfigUpdate16(index,                              //
                                msi.offset + kPciMsiControlOffset,  //
                                kPciMsiControlEnable,               //
                                0),
              t::Eq(0x0089));
  EXPECT_THAT(PciConfigUpdate16(index,                              //
                                msi.offset + kPciMsiControlOffset,  //
                                0,                                  //
                                kPciMsiControlEnable),
              t::Eq(0x0088));
}

TEST_F(PciTest, ConfigHelpersCheckIndexOffsetAndWidth) {
  AddQemuPcMachine(fake_.get(), QemuFirmware::kSeaBios, QemuTopology::kSingle);
  Init();
  EXPECT_DEATH(PciConfigRead8(-1, 0), "Check failed");
  EXPECT_DEATH(PciConfigRead16(PciDeviceCount(), 0), "Check failed");
  EXPECT_DEATH(PciConfigRead16(0, 1), "Check failed");
  EXPECT_DEATH(PciConfigRead32(0, 2), "Check failed");
  EXPECT_DEATH(PciConfigRead8(0, kPciConfigSpaceSize), "Check failed");
  EXPECT_DEATH(PciConfigWrite16(0, -2, 0), "Check failed");
  EXPECT_DEATH(PciConfigWrite32(0, kPciCommandOffset, 0),
               "Check failed: offset != kPciCommandOffset");
  EXPECT_DEATH(PciUpdateCommand(0, kPciCommandBusMaster, kPciCommandBusMaster),
               "Check failed: \\(set & clear\\) == 0");
  EXPECT_DEATH(PciQuiesce(-1), "Check failed");
}

TEST_F(PciTest, ConfigAccessesFromManyThreadsAreSerialized) {
  AddQemuPcMachine(fake_.get(), QemuFirmware::kSeaBios, QemuTopology::kMulti);
  Init();
  const int count = PciDeviceCount();
  std::vector<std::thread> threads;
  for (int thread = 0; thread < 4; ++thread) {
    threads.emplace_back([thread, count] {
      for (int i = 0; i < 200; ++i) {
        const int index = (thread + i) % count;
        PciConfigRead32(index, 0x00);
        PciConfigRead16(index, kPciStatusOffset);
        PciUpdateCommand(index, 0, 0);
      }
    });
  }
  for (std::thread& thread : threads) {
    thread.join();
  }
  EXPECT_THAT(fake_->MaxConcurrentAccesses(), t::Eq(1));
}

TEST_F(PciTest, ConcurrentUpdatesOfOneRegisterLoseNoBits) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec());
  Init();
  const auto toggle = [](const uint16_t bit) {
    for (int i = 0; i < 300; ++i) {
      PciUpdateCommand(0, bit, 0);
      PciUpdateCommand(0, 0, bit);
    }
    PciUpdateCommand(0, bit, 0);
  };
  std::thread memory(toggle, kPciCommandMemorySpace);
  std::thread bus_master(toggle, kPciCommandBusMaster);
  std::thread intx(toggle, kPciCommandInterruptDisable);
  memory.join();
  bus_master.join();
  intx.join();
  EXPECT_THAT(Command({0, 3, 0}),
              t::Eq(kPciCommandMemorySpace | kPciCommandBusMaster |
                    kPciCommandInterruptDisable));
  EXPECT_THAT(fake_->MaxConcurrentAccesses(), t::Eq(1));
}

// --- Claims ---

TEST_F(PciTest, ClaimSetsMemorySpaceAndInterruptDisableButNotBusMaster) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0x0000, kStatusErrorBits));
  fake_->AddFunction({0, 4, 0}, EndpointSpec(0x0107, kStatusErrorBits));
  Init();
  fake_->ClearAccesses();
  EXPECT_THAT(PciClaim(0, kOwnerA), t::Eq(PciStatus::kOk));
  EXPECT_THAT(PciClaim(1, kOwnerB), t::Eq(PciStatus::kOk));
  EXPECT_THAT(Command({0, 3, 0}), t::Eq(0x0402));
  EXPECT_THAT(Command({0, 4, 0}), t::Eq(0x0507));
  for (const Access& access : Writes()) {
    EXPECT_THAT(access, IsAccess(true, 2, kPciCommandOffset));
  }
  EXPECT_THAT(fake_->Peek16({0, 3, 0}, kPciStatusOffset),
              t::Eq(kStatusErrorBits));
  EXPECT_THAT(fake_->Peek16({0, 4, 0}, kPciStatusOffset),
              t::Eq(kStatusErrorBits));
  EXPECT_THAT(PciOwner(0), t::Eq(kOwnerA));
  EXPECT_THAT(PciOwner(1), t::Eq(kOwnerB));
  EXPECT_THAT(PciClaimedCount(), t::Eq(2));
}

TEST_F(PciTest, ClaimOfAnotherOwnersDeviceIsBusyAndChangesNothing) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec());
  Init();
  ASSERT_THAT(PciClaim(0, kOwnerA), t::Eq(PciStatus::kOk));
  fake_->ClearAccesses();
  EXPECT_THAT(PciClaim(0, kOwnerB), t::Eq(PciStatus::kBusy));
  EXPECT_THAT(fake_->Accesses(), t::IsEmpty());
  EXPECT_THAT(PciOwner(0), t::Eq(kOwnerA));
  EXPECT_THAT(PciClaimedCount(), t::Eq(1));
}

TEST_F(PciTest, ClaimIsIdempotentForTheOwner) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec());
  Init();
  ASSERT_THAT(PciClaim(0, kOwnerA), t::Eq(PciStatus::kOk));
  // Even if the command changed in between, a repeated claim reprograms
  // nothing.
  PciUpdateCommand(0, 0, kPciCommandInterruptDisable);
  fake_->ClearAccesses();
  EXPECT_THAT(PciClaim(0, kOwnerA), t::Eq(PciStatus::kOk));
  EXPECT_THAT(fake_->Accesses(), t::IsEmpty());
  EXPECT_THAT(PciClaimedCount(), t::Eq(1));
}

TEST_F(PciTest, ClaimOutOfRangeIsNoDevice) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec());
  Init();
  fake_->ClearAccesses();
  for (const int index : {-1, 1, INT_MAX, INT_MIN}) {
    EXPECT_THAT(PciClaim(index, kOwnerA), t::Eq(PciStatus::kNoDevice));
    EXPECT_THAT(PciRelease(index, kOwnerA), t::Eq(PciStatus::kNoDevice));
    EXPECT_THAT(PciSetBusMaster(index, kOwnerA, true),
                t::Eq(PciStatus::kNoDevice));
    EXPECT_THAT(PciOwner(index), t::Eq(kPciNoOwner));
  }
  EXPECT_THAT(fake_->Accesses(), t::IsEmpty());
  EXPECT_THAT(PciClaimedCount(), t::Eq(0));
}

TEST_F(PciTest, OwnerTokenZeroIsRejected) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec());
  Init();
  EXPECT_DEATH(PciClaim(0, kPciNoOwner), "Check failed");
  EXPECT_DEATH(PciRelease(0, kPciNoOwner), "Check failed");
  EXPECT_DEATH(PciReleaseAll(kPciNoOwner), "Check failed");
  EXPECT_DEATH(PciSetBusMaster(0, kPciNoOwner, true), "Check failed");
}

TEST_F(PciTest, OnlyEndpointsThatAreNotBridgesAreClaimable) {
  AddQemuPcMachine(fake_.get(), QemuFirmware::kSeaBios, QemuTopology::kMulti);
  Init();
  for (const PciAddress address : {PciAddress{0, 0, 0}, PciAddress{0, 1, 0},
                                   PciAddress{0, 1, 3}, PciAddress{0, 5, 0}}) {
    SCOPED_TRACE(t::PrintToString(address));
    EXPECT_FALSE(DeviceAt(address).claimable);
    fake_->ClearAccesses();
    EXPECT_THAT(PciClaim(PciFindDevice(address), kOwnerA),
                t::Eq(PciStatus::kNotClaimable));
    EXPECT_THAT(fake_->Accesses(), t::IsEmpty());
  }
  for (const PciAddress address :
       {PciAddress{0, 1, 1}, PciAddress{0, 2, 0}, PciAddress{0, 3, 0},
        PciAddress{0, 4, 0}, PciAddress{1, 1, 0}}) {
    SCOPED_TRACE(t::PrintToString(address));
    EXPECT_TRUE(DeviceAt(address).claimable);
    EXPECT_THAT(PciClaim(PciFindDevice(address), kOwnerA),
                t::Eq(PciStatus::kOk));
  }
  EXPECT_THAT(PciClaimedCount(), t::Eq(5));
}

TEST_F(PciTest, ReleaseFreesTheClaimForTheNextOwner) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec());
  Init();
  ASSERT_THAT(PciClaim(0, kOwnerA), t::Eq(PciStatus::kOk));
  EXPECT_THAT(PciRelease(0, kOwnerA), t::Eq(PciStatus::kOk));
  EXPECT_THAT(PciOwner(0), t::Eq(kPciNoOwner));
  EXPECT_THAT(PciClaimedCount(), t::Eq(0));
  EXPECT_THAT(PciClaim(0, kOwnerB), t::Eq(PciStatus::kOk));
  EXPECT_THAT(PciOwner(0), t::Eq(kOwnerB));
}

TEST_F(PciTest, ReleaseByAnyoneButTheOwnerChangesNothing) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec());
  fake_->AddFunction({0, 4, 0}, EndpointSpec());
  Init();
  ASSERT_THAT(PciClaim(0, kOwnerA), t::Eq(PciStatus::kOk));
  fake_->ClearAccesses();
  EXPECT_THAT(PciRelease(0, kOwnerB), t::Eq(PciStatus::kNotOwner));
  EXPECT_THAT(PciRelease(1, kOwnerA), t::Eq(PciStatus::kNotOwner));
  EXPECT_THAT(fake_->Accesses(), t::IsEmpty());
  EXPECT_THAT(PciOwner(0), t::Eq(kOwnerA));
  EXPECT_THAT(PciClaimedCount(), t::Eq(1));
}

TEST_F(PciTest, ReleaseAllReleasesExactlyTheOwnersDevices) {
  for (uint8_t device = 1; device <= 3; ++device) {
    fake_->AddFunction({0, device, 0}, EndpointSpec());
  }
  Init();
  ASSERT_THAT(PciClaim(0, kOwnerA), t::Eq(PciStatus::kOk));
  ASSERT_THAT(PciClaim(1, kOwnerB), t::Eq(PciStatus::kOk));
  ASSERT_THAT(PciClaim(2, kOwnerA), t::Eq(PciStatus::kOk));
  EXPECT_THAT(PciReleaseAll(kOwnerA), t::Eq(2));
  EXPECT_THAT(PciClaimedCount(), t::Eq(1));
  EXPECT_THAT(PciOwner(0), t::Eq(kPciNoOwner));
  EXPECT_THAT(PciOwner(1), t::Eq(kOwnerB));
  EXPECT_THAT(PciOwner(2), t::Eq(kPciNoOwner));
  EXPECT_THAT(PciReleaseAll(kOwnerA), t::Eq(0));
}

TEST_F(PciTest, ReleaseQuiescesTheDevice) {
  AddQemuPcMachine(fake_.get(), QemuFirmware::kSeaBios, QemuTopology::kMulti);
  Init();
  const int index = PciFindDevice({0, 4, 0});
  ASSERT_THAT(PciClaim(index, kOwnerA), t::Eq(PciStatus::kOk));
  ASSERT_THAT(PciSetBusMaster(index, kOwnerA, true), t::Eq(PciStatus::kOk));
  PciConfigUpdate16(index, 0x92, kPciMsixControlEnable, 0);
  EXPECT_THAT(PciRelease(index, kOwnerA), t::Eq(PciStatus::kOk));
  EXPECT_THAT(Command({0, 4, 0}), t::Eq(0x0103 | kPciCommandInterruptDisable));
  EXPECT_THAT(fake_->Peek16({0, 4, 0}, 0x92),
              t::Eq(0x000F | kPciMsixControlFunctionMask));
}

TEST_F(PciTest, ClaimOpensMemorySpaceOnUpstreamBridgesButNotBusMaster) {
  fake_->AddFunction({0, 5, 0}, BridgeSpec(0x0000, kStatusErrorBits));
  fake_->SetBridgeBuses({0, 5, 0}, 0, 1, 2);
  fake_->AddFunction({1, 0, 0}, BridgeSpec(0x0100));
  fake_->SetBridgeBuses({1, 0, 0}, 1, 2, 2);
  fake_->AddFunction({2, 3, 0}, EndpointSpec());
  fake_->AddFunction({0, 6, 0}, EndpointSpec());
  Init();
  fake_->ClearAccesses();
  ASSERT_THAT(PciClaim(PciFindDevice({2, 3, 0}), kOwnerA),
              t::Eq(PciStatus::kOk));
  EXPECT_THAT(Command({0, 5, 0}), t::Eq(0x0002));
  EXPECT_THAT(Command({1, 0, 0}), t::Eq(0x0102));
  EXPECT_THAT(Command({2, 3, 0}), t::Eq(0x0402));
  EXPECT_THAT(fake_->Peek16({0, 5, 0}, kPciStatusOffset),
              t::Eq(kStatusErrorBits));
  EXPECT_THAT(AccessesTo({0, 6, 0}), t::IsEmpty());
}

TEST_F(PciTest, ConcurrentClaimsOfOneDeviceHaveExactlyOneWinner) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec());
  Init();
  constexpr int kThreads = 8;
  for (int round = 0; round < 25; ++round) {
    std::vector<PciStatus> results(kThreads, PciStatus::kNoDevice);
    std::vector<std::thread> threads;
    for (int i = 0; i < kThreads; ++i) {
      threads.emplace_back(
          [i, &results] { results[i] = PciClaim(0, kOwnerA + i); });
    }
    for (std::thread& thread : threads) {
      thread.join();
    }
    int winners = 0;
    uintptr_t winner = kPciNoOwner;
    for (int i = 0; i < kThreads; ++i) {
      if (results[i] == PciStatus::kOk) {
        ++winners;
        winner = kOwnerA + i;
      } else {
        EXPECT_THAT(results[i], t::Eq(PciStatus::kBusy));
      }
    }
    ASSERT_THAT(winners, t::Eq(1)) << "round " << round;
    EXPECT_THAT(PciOwner(0), t::Eq(winner));
    EXPECT_THAT(PciClaimedCount(), t::Eq(1));
    ASSERT_THAT(PciRelease(0, winner), t::Eq(PciStatus::kOk));
  }
  EXPECT_THAT(fake_->MaxConcurrentAccesses(), t::Eq(1));
}

// --- Bus Master ---

TEST_F(PciTest, BusMasterOpensEveryUpstreamBridgeBeforeTheDevice) {
  fake_->AddFunction({0, 5, 0}, BridgeSpec(0x0103, kStatusErrorBits));
  fake_->SetBridgeBuses({0, 5, 0}, 0, 1, 2);
  fake_->AddFunction({1, 0, 0}, BridgeSpec(0x0000));
  fake_->SetBridgeBuses({1, 0, 0}, 1, 2, 2);
  fake_->AddFunction({2, 3, 0}, EndpointSpec());
  Init();
  const int index = PciFindDevice({2, 3, 0});
  ASSERT_THAT(PciClaim(index, kOwnerA), t::Eq(PciStatus::kOk));
  fake_->ClearAccesses();
  EXPECT_THAT(PciSetBusMaster(index, kOwnerA, true), t::Eq(PciStatus::kOk));
  EXPECT_THAT(Command({1, 0, 0}), t::Eq(0x0006));
  EXPECT_THAT(Command({0, 5, 0}), t::Eq(0x0107));
  EXPECT_THAT(Command({2, 3, 0}), t::Eq(0x0406));
  EXPECT_THAT(fake_->Peek16({0, 5, 0}, kPciStatusOffset),
              t::Eq(kStatusErrorBits));
  const std::vector<Access> writes = Writes();
  ASSERT_THAT(writes.size(), t::Eq(3u));
  EXPECT_THAT(writes.back().address, t::Eq(PciAddress{2, 3, 0}));
  for (const Access& access : writes) {
    EXPECT_THAT(access, IsAccess(true, 2, kPciCommandOffset));
  }
}

TEST_F(PciTest, BusMasterOffClearsOnlyTheDevice) {
  fake_->AddFunction({0, 5, 0}, BridgeSpec(0x0000));
  fake_->SetBridgeBuses({0, 5, 0}, 0, 1, 1);
  fake_->AddFunction({1, 0, 0}, EndpointSpec());
  Init();
  const int index = PciFindDevice({1, 0, 0});
  ASSERT_THAT(PciClaim(index, kOwnerA), t::Eq(PciStatus::kOk));
  ASSERT_THAT(PciSetBusMaster(index, kOwnerA, true), t::Eq(PciStatus::kOk));
  fake_->ClearAccesses();
  EXPECT_THAT(PciSetBusMaster(index, kOwnerA, false), t::Eq(PciStatus::kOk));
  EXPECT_THAT(Command({1, 0, 0}), t::Eq(0x0402));
  // Bridges are shared: their bits stay.
  EXPECT_THAT(Command({0, 5, 0}), t::Eq(kMemAndBusMaster));
  EXPECT_THAT(WritesTo({0, 5, 0}), t::IsEmpty());
}

TEST_F(PciTest, BusMasterOnARootBusDeviceTouchesOnlyIt) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec());
  fake_->AddFunction({0, 4, 0}, EndpointSpec());
  Init();
  ASSERT_THAT(PciClaim(0, kOwnerA), t::Eq(PciStatus::kOk));
  fake_->ClearAccesses();
  EXPECT_THAT(PciSetBusMaster(0, kOwnerA, true), t::Eq(PciStatus::kOk));
  EXPECT_THAT(Command({0, 3, 0}), t::Eq(0x0406));
  EXPECT_THAT(AccessesTo({0, 4, 0}), t::IsEmpty());
  fake_->ClearAccesses();
  EXPECT_THAT(PciSetBusMaster(0, kOwnerA, true), t::Eq(PciStatus::kOk));
  EXPECT_THAT(Writes(), t::IsEmpty());
}

TEST_F(PciTest, BusMasterNeedsTheOwner) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec());
  Init();
  fake_->ClearAccesses();
  EXPECT_THAT(PciSetBusMaster(0, kOwnerA, true), t::Eq(PciStatus::kNotOwner));
  ASSERT_THAT(PciClaim(0, kOwnerA), t::Eq(PciStatus::kOk));
  fake_->ClearAccesses();
  EXPECT_THAT(PciSetBusMaster(0, kOwnerB, true), t::Eq(PciStatus::kNotOwner));
  EXPECT_THAT(PciSetBusMaster(0, kOwnerB, false), t::Eq(PciStatus::kNotOwner));
  EXPECT_THAT(fake_->Accesses(), t::IsEmpty());
}

TEST_F(PciTest, BusMasterOpensTheBridgeThatSeaBiosLeavesClosed) {
  AddQemuPcMachine(fake_.get(), QemuFirmware::kSeaBios, QemuTopology::kMulti);
  Init();
  const int index = PciFindDevice({1, 1, 0});
  ASSERT_THAT(Command({0, 5, 0}), t::Eq(0x0103));
  ASSERT_THAT(PciClaim(index, kOwnerA), t::Eq(PciStatus::kOk));
  EXPECT_THAT(Command({0, 5, 0}), t::Eq(0x0103));
  EXPECT_THAT(Command({1, 1, 0}), t::Eq(0x0507));
  ASSERT_THAT(PciSetBusMaster(index, kOwnerA, true), t::Eq(PciStatus::kOk));
  EXPECT_THAT(Command({0, 5, 0}), t::Eq(0x0107));
}

// --- Quiesce ---

TEST_F(PciTest, QuiesceStopsDmaFirstThenMsixAndMsiAndReadsBack) {
  const PciAddress address = {0, 4, 0};
  fake_->AddFunction(address, EndpointSpec(0x0006, 0x0010));
  fake_->SetBar(address,  //
                0,        //
                {.kind = BarKind::kMem64, .base = 0xFEA70000, .size = 0x4000});
  fake_->AddMsix(address,  //
                 0x90,     //
                 {.table_size = 16,
                  .table_bar = 0,
                  .table_offset = 0x3000,
                  .pba_bar = 0,
                  .pba_offset = 0x3800});
  fake_->AddMsi(address, 0x70, {.is_64bit = true, .vectors_log2 = 4});
  Init();
  const int index = PciFindDevice(address);
  PciConfigUpdate16(index, 0x92, kPciMsixControlEnable, 0);
  PciConfigUpdate16(index, 0x72, kPciMsiControlEnable, 0);
  fake_->ClearAccesses();

  const PciQuiesceResult result = PciQuiesce(index);
  EXPECT_THAT(result.command, t::Eq(0x0402));
  EXPECT_THAT(result.msix_control, t::Eq(0x400F));
  EXPECT_THAT(result.msi_control, t::Eq(0x0088));
  EXPECT_THAT(Command(address), t::Eq(0x0402));
  EXPECT_THAT(fake_->Peek16(address, 0x92), t::Eq(0x400F));
  EXPECT_THAT(fake_->Peek16(address, 0x72), t::Eq(0x0088));
  // Command first, and each register read back after its write.
  EXPECT_THAT(fake_->Accesses(),
              t::ElementsAre(IsAccess(false, 2, 0x04), IsAccess(true, 2, 0x04),
                             IsAccess(false, 2, 0x04), IsAccess(false, 2, 0x92),
                             IsAccess(true, 2, 0x92), IsAccess(false, 2, 0x92),
                             IsAccess(false, 2, 0x72), IsAccess(true, 2, 0x72),
                             IsAccess(false, 2, 0x72)));
}

TEST_F(PciTest, QuiesceOfADeviceWithoutMsiTouchesOnlyCommand) {
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0x0007));
  Init();
  fake_->ClearAccesses();
  const PciQuiesceResult result = PciQuiesce(0);
  EXPECT_THAT(result.command, t::Eq(0x0403));
  EXPECT_THAT(result.msi_control, t::Eq(0));
  EXPECT_THAT(result.msix_control, t::Eq(0));
  for (const Access& access : fake_->Accesses()) {
    EXPECT_THAT(access.offset, t::Eq(kPciCommandOffset));
  }
}

TEST_F(PciTest, QuiesceAlsoDisablesMsixWhoseGeometryIsInvalid) {
  fake_->AddFunction({0, 4, 0}, EndpointSpec(0x0006));
  fake_->AddMsix({0, 4, 0}, 0x90, {.table_size = 4});
  fake_->SetRaw16({0, 4, 0}, 0x92, kPciMsixControlEnable | 3);
  Init();
  ASSERT_FALSE(DeviceAt(0).msix.valid);
  EXPECT_THAT(PciQuiesce(0).msix_control,
              t::Eq(kPciMsixControlFunctionMask | 3));
}

TEST_F(PciTest, UpdateAndQuiesceSkipWriteWhenRegisterReadsAllOnes) {
  const PciAddress address = {0, 4, 0};
  fake_->AddFunction(address, EndpointSpec(0x0006));
  fake_->AddMsi(address, 0x70, {.is_64bit = true});
  Init();
  // Simulate device disappearance (register reads 0xFFFF): PciQuiesce and
  // PciConfigUpdate16 must not write 0xFFFB back.
  fake_->SetRaw16(address, kPciCommandOffset, 0xFFFF);
  fake_->SetRaw16(address, 0x72, 0xFFFF);
  fake_->ClearAccesses();

  const PciQuiesceResult result = PciQuiesce(0);
  EXPECT_THAT(result.command, t::Eq(0xFFFF));
  EXPECT_THAT(result.msi_control, t::Eq(0xFFFF));
  EXPECT_THAT(Writes(), t::IsEmpty());
}

// --- Boot report ---

TEST_F(PciTest, BootReportOfQemuSeaBiosMultiMachine) {
  AddQemuPcMachine(fake_.get(), QemuFirmware::kSeaBios, QemuTopology::kMulti);
  Init();
  EXPECT_THAT(
      ReportLines(),
      t::ElementsAre(
          "[PCI] 00:00.0 8086:1237 class 06.00.00",
          "[PCI] 00:01.0 8086:7000 class 06.01.00",
          "[PCI] 00:01.1 8086:7010 class 01.01.80 bar4=io:0xd040+0x10",
          "[PCI] 00:01.3 8086:7113 class 06.80.00",
          "[PCI] 00:02.0 1234:1111 class 03.00.00 "
          "bar0=mem32p:0xfd000000+0x1000000 bar2=mem32:0xfea74000+0x1000",
          "[PCI] 00:03.0 8086:100e class 02.00.00 "
          "bar0=mem32:0xfea40000+0x20000 bar1=io:0xd000+0x40",
          "[PCI] 00:04.0 1b36:000d class 0c.03.30 "
          "bar0=mem64:0xfea70000+0x4000 msix=16",
          "[PCI] 00:05.0 1b36:0001 class 06.04.00 bridge=01-01 "
          "bar0=mem64:0xfea75000 msi64 pvm",
          "[PCI] 01:01.0 1033:0194 class 0c.03.30 "
          "bar0=mem64:0xfe800000+0x4000 msi64",
          "[PCI] 9 functions"));
  const PciDevice& root_xhci = DeviceAt({0, 4, 0});
  EXPECT_TRUE(root_xhci.msix.valid);
  EXPECT_THAT(root_xhci.msix.table_phys, t::Eq(0xFEA73000u));
  EXPECT_THAT(root_xhci.msix.pba_phys, t::Eq(0xFEA73800u));
  EXPECT_THAT(root_xhci.msi.offset, t::Eq(0));
  const PciDevice& bridged_xhci = DeviceAt({1, 1, 0});
  EXPECT_THAT(bridged_xhci.parent_index, t::Eq(PciFindDevice({0, 5, 0})));
  EXPECT_THAT(bridged_xhci.msi.offset, t::Eq(0x70));
  EXPECT_THAT(bridged_xhci.msi.vectors_log2, t::Eq(4));
  EXPECT_THAT(bridged_xhci.msix.offset, t::Eq(0));
}

TEST_F(PciTest, BootReportOfQemuOvmfMultiMachine) {
  AddQemuPcMachine(fake_.get(), QemuFirmware::kOvmf, QemuTopology::kMulti);
  Init();
  EXPECT_THAT(
      ReportLines(),
      t::ElementsAre(
          "[PCI] 00:00.0 8086:1237 class 06.00.00",
          "[PCI] 00:01.0 8086:7000 class 06.01.00",
          "[PCI] 00:01.1 8086:7010 class 01.01.80 bar4=io:0xc040+0x10",
          "[PCI] 00:01.3 8086:7113 class 06.80.00",
          "[PCI] 00:02.0 1234:1111 class 03.00.00 "
          "bar0=mem32p:0xc0000000+0x1000000 bar2=mem32:0xc1260000+0x1000",
          "[PCI] 00:03.0 8086:100e class 02.00.00 "
          "bar0=mem32:0xc1240000+0x20000 bar1=io:0xc000+0x40",
          "[PCI] 00:04.0 1b36:000d class 0c.03.30 "
          "bar0=mem64:0xc020000000+0x4000 msix=16",
          "[PCI] 00:05.0 1b36:0001 class 06.04.00 bridge=01-01 "
          "bar0=mem64:0xc020004000 msi64 pvm",
          "[PCI] 01:01.0 1033:0194 class 0c.03.30 "
          "bar0=mem64:0xc1000000+0x4000 msi64",
          "[PCI] 9 functions"));
  EXPECT_THAT(DeviceAt({0, 4, 0}).msix.table_phys, t::Eq(0xC020003000u));
}

TEST_F(PciTest, BootReportOfQemuSingleMachines) {
  AddQemuPcMachine(fake_.get(), QemuFirmware::kOvmf, QemuTopology::kSingle);
  Init();
  const std::vector<std::string> ovmf = ReportLines();
  ASSERT_THAT(ovmf.size(), t::Eq(8u));
  EXPECT_THAT(ovmf[6], t::Eq("[PCI] 00:04.0 1b36:000d class 0c.03.30 "
                             "bar0=mem64:0xc000000000+0x4000 msix=16"));
  EXPECT_THAT(ovmf[7], t::Eq("[PCI] 7 functions"));

  PciResetForTest();
  fake_ = std::make_shared<FakePciConfigSpace>();
  AddQemuPcMachine(fake_.get(), QemuFirmware::kSeaBios, QemuTopology::kSingle);
  Init();
  const std::vector<std::string> seabios = ReportLines();
  ASSERT_THAT(seabios.size(), t::Eq(8u));
  EXPECT_THAT(seabios[4], t::Eq("[PCI] 00:02.0 1234:1111 class 03.00.00 "
                                "bar0=mem32p:0xfd000000+0x1000000 "
                                "bar2=mem32:0xfebf4000+0x1000"));
  EXPECT_THAT(seabios[6], t::Eq("[PCI] 00:04.0 1b36:000d class 0c.03.30 "
                                "bar0=mem64:0xfebf0000+0x4000 msix=16"));
}

TEST_F(PciTest, BootReportOfQemuMachineWithoutExtraDevices) {
  AddQemuPcMachine(fake_.get(), QemuFirmware::kSeaBios, QemuTopology::kNone);
  Init();
  EXPECT_THAT(
      ReportLines(),
      t::ElementsAre(
          "[PCI] 00:00.0 8086:1237 class 06.00.00",
          "[PCI] 00:01.0 8086:7000 class 06.01.00",
          "[PCI] 00:01.1 8086:7010 class 01.01.80 bar4=io:0xc040+0x10",
          "[PCI] 00:01.3 8086:7113 class 06.80.00",
          "[PCI] 00:02.0 1234:1111 class 03.00.00 "
          "bar0=mem32p:0xfd000000+0x1000000 bar2=mem32:0xfebf0000+0x1000",
          "[PCI] 00:03.0 8086:100e class 02.00.00 "
          "bar0=mem32:0xfebc0000+0x20000 bar1=io:0xc000+0x40",
          "[PCI] 6 functions"));
}

TEST_F(PciTest, BootReportNamesLayoutsAndUnsizedOrUnassignedBars) {
  FunctionSpec cardbus = BridgeSpec();
  cardbus.header_type = kPciHeaderTypeCardBus;
  fake_->AddFunction({0, 1, 0}, cardbus);
  FunctionSpec odd = EndpointSpec();
  odd.header_type = 0x7F;
  fake_->AddFunction({0, 2, 0}, odd);
  fake_->AddFunction({0, 3, 0}, EndpointSpec(0x0001));
  fake_->SetBar({0, 3, 0}, 1, {.kind = BarKind::kIo, .base = 0, .size = 0x8});
  fake_->AddMsi({0, 3, 0}, 0x40, {.is_64bit = false});
  Init();
  EXPECT_THAT(ReportLines(),
              t::ElementsAre("[PCI] 00:01.0 8086:244e class 06.04.00 hdr=02",
                             "[PCI] 00:02.0 8086:10d3 class 02.00.00 hdr=7f",
                             "[PCI] 00:03.0 8086:10d3 class 02.00.00 "
                             "bar1=io:0x0 msi32",
                             "[PCI] 3 functions"));
}

TEST_F(PciTest, BootReportSummaryOfOneFunction) {
  fake_->AddFunction({0x12, 0x1F, 0}, EndpointSpec());
  Init();
  EXPECT_THAT(ReportLines(),
              t::ElementsAre("[PCI] 12:1f.0 8086:10d3 class 02.00.00",
                             "[PCI] 1 function"));
}

TEST_F(PciTest, BootReportNeverContainsFailureMarkers) {
  for (const QemuFirmware firmware :
       {QemuFirmware::kSeaBios, QemuFirmware::kOvmf}) {
    for (const QemuTopology topology :
         {QemuTopology::kNone, QemuTopology::kSingle, QemuTopology::kMulti}) {
      PciResetForTest();
      fake_ = std::make_shared<FakePciConfigSpace>();
      AddQemuPcMachine(fake_.get(), firmware, topology);
      // And malformed devices on another bus.
      fake_->AddFunction({0x40, 0, 0}, EndpointSpec(0x0006));
      fake_->AddMsix({0x40, 0, 0}, 0x90, {.table_size = 2048, .table_bar = 5});
      fake_->SetRaw32({0x40, 0, 0}, kPciBar0Offset, 0xFAFAFAF6);
      fake_->AddFunction({0x40, 1, 0}, BridgeSpec());
      fake_->SetBridgeBuses({0x40, 1, 0}, 0x40, 0x40, 0x40);
      Init();
      const std::vector<std::string> lines = ReportLines();
      for (const std::string& line : lines) {
        EXPECT_THAT(line, t::Not(t::HasSubstr("FAIL")));
        EXPECT_THAT(line, t::Not(t::HasSubstr("[FATAL]")));
        EXPECT_THAT(line, t::StartsWith("[PCI] "));
      }
    }
  }
}

// --- PciBarContainsRange ---

TEST(PciBarContainsRangeTest, AcceptsOnlyRangesInsideSizedMemoryBars) {
  const PciBar bar = {.type = PciBarType::kMem64,
                      .prefetchable = false,
                      .base = 0xFEA70000,
                      .size = 0x4000};
  EXPECT_TRUE(PciBarContainsRange(bar, 0xFEA70000, 0x4000));
  EXPECT_TRUE(PciBarContainsRange(bar, 0xFEA70000, 0x1000));
  EXPECT_TRUE(PciBarContainsRange(bar, 0xFEA73000, 0x1000));
  EXPECT_TRUE(PciBarContainsRange(bar, 0xFEA73FFF, 1));
  EXPECT_FALSE(PciBarContainsRange(bar, 0xFEA73000, 0x1001));
  EXPECT_FALSE(PciBarContainsRange(bar, 0xFEA6F000, 0x1000));
  EXPECT_FALSE(PciBarContainsRange(bar, 0xFEA74000, 0x1000));
  EXPECT_FALSE(PciBarContainsRange(bar, 0xFEA70000, 0x8000));
  EXPECT_FALSE(PciBarContainsRange(bar, 0xFEA70000, 0));
  EXPECT_FALSE(PciBarContainsRange(bar, 0xFEA70000, -0x1000));
  EXPECT_FALSE(PciBarContainsRange(bar, 0, 0x1000));

  PciBar io = bar;
  io.type = PciBarType::kIo;
  EXPECT_FALSE(PciBarContainsRange(io, 0xFEA70000, 0x1000));
  PciBar none = bar;
  none.type = PciBarType::kNone;
  EXPECT_FALSE(PciBarContainsRange(none, 0xFEA70000, 0x1000));
  PciBar unsized = bar;
  unsized.size = 0;
  EXPECT_FALSE(PciBarContainsRange(unsized, 0xFEA70000, 0x1000));
  PciBar unassigned = bar;
  unassigned.base = 0;
  EXPECT_FALSE(PciBarContainsRange(unassigned, 0, 0x1000));
  PciBar mem32 = bar;
  mem32.type = PciBarType::kMem32;
  EXPECT_TRUE(PciBarContainsRange(mem32, 0xFEA71000, 0x1000));
}

TEST(PciBarContainsRangeTest, NoOverflowAtTheTopOfTheAddressSpace) {
  const PciBar top = {.type = PciBarType::kMem64,
                      .prefetchable = true,
                      .base = 0xFFFFFFFFFFFFF000u,
                      .size = 0x1000};
  EXPECT_TRUE(PciBarContainsRange(top, 0xFFFFFFFFFFFFF000u, 0x1000));
  EXPECT_FALSE(PciBarContainsRange(top, 0xFFFFFFFFFFFFF800u, 0x1000));
  EXPECT_FALSE(PciBarContainsRange(top, 0xFFFFFFFFFFFFFFFFu, INT64_MAX));
  const PciBar bar = {.type = PciBarType::kMem32,
                      .prefetchable = false,
                      .base = 0xFE000000,
                      .size = 0x1000};
  EXPECT_FALSE(PciBarContainsRange(bar, 0xFE000800, INT64_MAX));
}

}  // namespace
}  // namespace protos
