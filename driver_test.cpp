#include "driver.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "device_irq.h"
#include "idt.h"
#include "ioapic.h"
#include "paging.h"
#include "pci.h"
#include "pci_config_fake.h"
#include "pmm.h"
#include "syscall_abi.h"

namespace protos {
namespace {

namespace t = ::testing;

struct MappedPageRecord {
  uintptr_t user_addr = 0;
  uintptr_t phys_addr = 0;
  bool device_mmio = false;
};

class FakeDriverEnvironment final : public DriverEnvironment {
 public:
  bool RangeOverlapsUsableRam(const uint64_t phys_addr,
                              const int64_t size) override {
    if (ram_range_size_ <= 0) {
      return false;
    }
    const uint64_t end = phys_addr + static_cast<uint64_t>(size);
    const uint64_t ram_end =
        ram_range_base_ + static_cast<uint64_t>(ram_range_size_);
    return phys_addr < ram_end && ram_range_base_ < end;
  }

  bool MapUserDevicePage(AddressSpace* const /*space*/,
                         const uintptr_t user_addr,
                         const uintptr_t phys_addr) override {
    if (fail_map_after_ == 0) {
      return false;
    }
    if (fail_map_after_ > 0) {
      --fail_map_after_;
    }
    mapped_pages_.push_back({user_addr, phys_addr, /*device_mmio=*/true});
    return true;
  }

  uintptr_t AllocContiguousFrames(const int64_t frame_count,
                                  const uintptr_t max_phys_addr) override {
    last_alloc_max_phys_ = max_phys_addr;
    if (fail_alloc_frames_ || frame_count <= 0) {
      return 0;
    }
    if (next_dma_phys_ + static_cast<uintptr_t>(frame_count) * kPageSize >
        max_phys_addr) {
      return 0;
    }
    const uintptr_t base = next_dma_phys_;
    next_dma_phys_ += static_cast<uintptr_t>(frame_count) * kPageSize;
    allocated_frames_ += frame_count;
    return base;
  }

  void FreeContiguousFrames(const uintptr_t /*base_phys*/,
                            const int64_t frame_count) override {
    freed_frames_ += frame_count;
  }

  void ZeroPhysicalRange(const uintptr_t base_phys,
                         const int64_t size) override {
    zeroed_ranges_.emplace_back(base_phys, size);
  }

  bool MapUserOwnedFrame(AddressSpace* const /*space*/,
                         const uintptr_t user_addr,
                         const uintptr_t frame_phys) override {
    if (fail_map_after_ == 0) {
      return false;
    }
    if (fail_map_after_ > 0) {
      --fail_map_after_;
    }
    mapped_pages_.push_back({user_addr, frame_phys, /*device_mmio=*/false});
    return true;
  }

  void SleepMicroseconds(const int64_t microseconds) override {
    slept_us_.push_back(microseconds);
  }

  void Log(const char* const text) override { logs_.push_back(text); }

  uint8_t PortRead8(const uint16_t port) override {
    port_ops_.emplace_back(/*write=*/false, port, port_in_[port]);
    return port_in_[port];
  }

  void PortWrite8(const uint16_t port, const uint8_t value) override {
    port_ops_.emplace_back(/*write=*/true, port, value);
    port_out_[port] = value;
  }

  uint64_t ram_range_base_ = 0;
  int64_t ram_range_size_ = 0;
  int fail_map_after_ = -1;
  bool fail_alloc_frames_ = false;
  uintptr_t next_dma_phys_ = 0x00200000ULL;
  uintptr_t last_alloc_max_phys_ = 0;
  int64_t allocated_frames_ = 0;
  int64_t freed_frames_ = 0;
  std::vector<MappedPageRecord> mapped_pages_;
  std::vector<std::pair<uintptr_t, int64_t>> zeroed_ranges_;
  std::vector<int64_t> slept_us_;
  std::vector<std::string> logs_;
  uint8_t port_in_[65536] = {};
  uint8_t port_out_[65536] = {};
  std::vector<std::tuple<bool, uint16_t, uint8_t>> port_ops_;
};

class FakeIrqEnv final : public DeviceIrqEnvironment {
 public:
  void RegisterIdtHandler(const int vector,
                          const InterruptHandlerFn handler) override {
    handlers_[vector] = handler;
  }

  void UnregisterIdtHandler(const int vector) override {
    handlers_[vector] = nullptr;
  }

  int MsiDestinationApicId() override { return msi_destination_; }

  volatile uint32_t* MapMsixTable(const int device_index,
                                  const PciMsix& msix) override {
    msix_words_[device_index].assign(msix.table_size * 4, 0);
    return msix_words_[device_index].data();
  }

  void UnmapMsixTable(const int device_index) override {
    ++unmapped_[device_index];
  }

  int64_t NowTicks() override { return ticks_; }
  void Block() override {}
  void BlockUntil(const int64_t deadline_tick) override {
    ticks_ = deadline_tick;
  }
  void Wake(Task* const /*task*/) override {}

  InterruptHandlerFn handlers_[kIdtEntryCount] = {};
  std::vector<uint32_t> msix_words_[kPciMaxDevices];
  int unmapped_[kPciMaxDevices] = {};
  int64_t ticks_ = 50;
  int msi_destination_ = 0;
};

class DriverTest : public t::Test {
 protected:
  void SetUp() override {
    PciResetForTest();
    DriverResetForTest();
    pci_ = std::make_shared<FakePciConfigSpace>();
    driver_env_ = std::make_shared<FakeDriverEnvironment>();
    irq_env_ = std::make_shared<FakeIrqEnv>();
    AddQemuPcMachine(pci_.get(), QemuFirmware::kSeaBios, QemuTopology::kMulti);
    PciInit(pci_);
    DriverInit(driver_env_, irq_env_);
  }

  void TearDown() override {
    DriverResetForTest();
    PciResetForTest();
  }

  std::shared_ptr<FakePciConfigSpace> pci_;
  std::shared_ptr<FakeDriverEnvironment> driver_env_;
  std::shared_ptr<FakeIrqEnv> irq_env_;
};

TEST_F(DriverTest, MayMakeSyscallEnforcesCapabilityGates) {
  // Unprivileged processes may call 0..6 and 16 (get_ticks), but not 7..15.
  for (int64_t num = 0; num <= 6; ++num) {
    EXPECT_TRUE(MayMakeSyscall(kUserCapNone, num)) << "syscall " << num;
  }
  EXPECT_FALSE(MayMakeSyscall(kUserCapNone, kSyscallSpawnDriver));
  for (int64_t num : {kSyscallPciInfo, kSyscallPciClaim, kSyscallPciBusMaster,
                      kSyscallMmioMap, kSyscallDmaAlloc, kSyscallIrqAlloc,
                      kSyscallIrqWait, kSyscallConsolePush, kSyscallIsaClaim,
                      kSyscallIsaPortIo, kSyscallIsaIrqAlloc}) {
    EXPECT_FALSE(MayMakeSyscall(kUserCapNone, num)) << "syscall " << num;
    EXPECT_FALSE(MayMakeSyscall(kUserCapSpawnDrivers, num))
        << "syscall " << num;
    EXPECT_TRUE(MayMakeSyscall(kUserCapDriver, num)) << "syscall " << num;
  }
  EXPECT_TRUE(MayMakeSyscall(kUserCapSpawnDrivers, kSyscallSpawnDriver));
  EXPECT_FALSE(MayMakeSyscall(kUserCapDriver, kSyscallSpawnDriver));
  EXPECT_TRUE(MayMakeSyscall(kUserCapNone, kSyscallGetTicks));
  EXPECT_TRUE(MayMakeSyscall(kUserCapDriver, kSyscallGetTicks));
}

TEST_F(DriverTest, PciInfoQueriesCountAndPopulatesAbiStruct) {
  DriverContext ctx;
  EXPECT_THAT(PciDeviceCount(), t::Eq(9));
  EXPECT_THAT(ctx.PciInfo(-1).status(), t::Eq(kSyscallErrorNoDevice));
  EXPECT_THAT(ctx.PciInfo(9).status(), t::Eq(kSyscallErrorNoDevice));

  // Host bridge 00:00.0 is not claimable.
  const StatusOr<DriverPciInfoResult> host_bridge = ctx.PciInfo(0);
  ASSERT_TRUE(host_bridge.ok());
  EXPECT_THAT(host_bridge->count, t::Eq(9));
  EXPECT_THAT(host_bridge->info.vendor_id, t::Eq(0x8086));
  EXPECT_THAT(host_bridge->info.class_code, t::Eq(kPciClassBridge));
  EXPECT_THAT(host_bridge->info.claimable, t::Eq(0));

  // Root MSI-X controller at 00:04.0.
  const int msix_idx = PciFindDevice({0, 4, 0});
  const StatusOr<DriverPciInfoResult> msix_res = ctx.PciInfo(msix_idx);
  ASSERT_TRUE(msix_res.ok());
  EXPECT_THAT(msix_res->count, t::Eq(9));
  const PciDeviceInfo& msix_info = msix_res->info;
  EXPECT_THAT(msix_info.bus, t::Eq(0));
  EXPECT_THAT(msix_info.device, t::Eq(4));
  EXPECT_THAT(msix_info.function, t::Eq(0));
  EXPECT_THAT(msix_info.vendor_id, t::Eq(0x1B36));
  EXPECT_THAT(msix_info.device_id, t::Eq(0x000D));
  EXPECT_THAT(msix_info.class_code, t::Eq(0x0C));
  EXPECT_THAT(msix_info.subclass, t::Eq(0x03));
  EXPECT_THAT(msix_info.prog_if, t::Eq(0x30));
  EXPECT_THAT(msix_info.claimable, t::Eq(1));
  EXPECT_THAT(msix_info.msix_vectors, t::Eq(16));
  EXPECT_THAT(msix_info.msi_vectors, t::Eq(0));
  EXPECT_THAT(msix_info.bars[0].type, t::Eq(PciBarInfoType::kMemory64));
  EXPECT_THAT(msix_info.bars[0].phys_base, t::Eq(0xFEA70000ULL));
  EXPECT_THAT(msix_info.bars[0].size, t::Eq(0x4000));
  EXPECT_THAT(msix_info.parent_index, t::Eq(-1));

  // Bridged MSI controller at 01:01.0 behind bridge 00:05.0.
  const int msi_idx = PciFindDevice({1, 1, 0});
  const StatusOr<DriverPciInfoResult> msi_res = ctx.PciInfo(msi_idx);
  ASSERT_TRUE(msi_res.ok());
  EXPECT_THAT(msi_res->count, t::Eq(9));
  const PciDeviceInfo& msi_info = msi_res->info;
  EXPECT_THAT(msi_info.claimable, t::Eq(1));
  EXPECT_THAT(msi_info.msi_vectors, t::Eq(16));
  EXPECT_THAT(msi_info.msix_vectors, t::Eq(0));
  EXPECT_THAT(msi_info.parent_index, t::Eq(PciFindDevice({0, 5, 0})));
}

TEST_F(DriverTest, ClaimOwnershipAndBusMasterSemantics) {
  DriverContext ctx_a;
  DriverContext ctx_b;

  // Out-of-range and unclaimable bridge functions.
  EXPECT_THAT(ctx_a.Claim(-1), t::Eq(kSyscallErrorNoDevice));
  EXPECT_THAT(ctx_a.Claim(99), t::Eq(kSyscallErrorNoDevice));
  EXPECT_THAT(ctx_a.Claim(PciFindDevice({0, 0, 0})),
              t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(ctx_a.Claim(PciFindDevice({0, 5, 0})),
              t::Eq(kSyscallErrorInvalid));

  const int bridged_dev = PciFindDevice({1, 1, 0});
  ASSERT_THAT(ctx_a.Claim(bridged_dev), t::Eq(0));
  // Idempotent for same owner.
  EXPECT_THAT(ctx_a.Claim(bridged_dev), t::Eq(0));
  EXPECT_THAT(ctx_a.ClaimedCount(), t::Eq(1));
  // Busy for another driver process.
  EXPECT_THAT(ctx_b.Claim(bridged_dev), t::Eq(kSyscallErrorBusy));

  // SetBusMaster requires ownership and enable in {0, 1}.
  EXPECT_THAT(ctx_b.SetBusMaster(bridged_dev, 1), t::Eq(kSyscallErrorAccess));
  EXPECT_THAT(ctx_a.SetBusMaster(bridged_dev, 2), t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(ctx_a.SetBusMaster(bridged_dev, -1), t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(ctx_a.SetBusMaster(99, 1), t::Eq(kSyscallErrorNoDevice));

  ASSERT_THAT(ctx_a.SetBusMaster(bridged_dev, 1), t::Eq(0));
  EXPECT_THAT(pci_->Peek16({0, 5, 0}, kPciCommandOffset) & kPciCommandBusMaster,
              t::Ne(0U));
  EXPECT_THAT(pci_->Peek16({1, 1, 0}, kPciCommandOffset) & kPciCommandBusMaster,
              t::Ne(0U));

  ASSERT_THAT(ctx_a.SetBusMaster(bridged_dev, 0), t::Eq(0));
  EXPECT_THAT(pci_->Peek16({1, 1, 0}, kPciCommandOffset) & kPciCommandBusMaster,
              t::Eq(0U));
  // Upstream bridge Bus Master stays enabled because bridges are shared.
  EXPECT_THAT(pci_->Peek16({0, 5, 0}, kPciCommandOffset) & kPciCommandBusMaster,
              t::Ne(0U));
  // No QEMU function has a Power Management capability: nothing to wake.
  EXPECT_THAT(driver_env_->slept_us_, t::IsEmpty());
  EXPECT_THAT(driver_env_->logs_, t::IsEmpty());
}

TEST_F(DriverTest, ClaimWakesADeviceThatFirmwareLeftInD3hot) {
  // Two Ethernet controllers with a Power Management capability at 0x40: one
  // in D3hot that resets itself on the way to D0, one already in D0.
  constexpr PciAddress kAsleep = {0, 6, 0};
  constexpr PciAddress kAwake = {0, 7, 0};
  constexpr int kPmcsr = 0x40 + kPciPmControlStatusOffset;
  for (const PciAddress address : {kAsleep, kAwake}) {
    pci_->AddFunction(
        address,
        {.vendor_id = 0x8086, .device_id = 0x10D3, .class_code = 0x02});
    pci_->SetBar(address,  //
                 0,        //
                 {.kind = FakePciConfigSpace::BarKind::kMem64,
                  .base = (address == kAsleep) ? 0xFC000000u : 0xFC100000u,
                  .size = 0x20000});
  }
  pci_->AddPowerManagement(kAsleep, 0x40, {.power_state = 3});
  pci_->AddPowerManagement(kAwake, 0x40, {.power_state = 0});
  PciResetForTest();
  PciInit(pci_);
  const int asleep = PciFindDevice(kAsleep);
  const int awake = PciFindDevice(kAwake);
  ASSERT_THAT(asleep, t::Ge(0));
  ASSERT_THAT(awake, t::Ge(0));
  const uint32_t bar0 = pci_->Peek32(kAsleep, kPciBar0Offset);

  DriverContext ctx;
  ASSERT_THAT(ctx.Claim(asleep), t::Eq(0));
  EXPECT_THAT(pci_->Peek16(kAsleep, kPmcsr) & kPciPmcsrPowerStateMask,
              t::Eq(0U));
  // 10 ms of recovery time from D3hot before the device is touched again.
  EXPECT_THAT(driver_env_->slept_us_, t::ElementsAre(10000));
  // The BARs and the Command bits that the reset cleared are back.
  EXPECT_THAT(pci_->Peek32(kAsleep, kPciBar0Offset), t::Eq(bar0));
  EXPECT_THAT(pci_->Peek16(kAsleep, kPciCommandOffset),
              t::Eq(kPciCommandMemorySpace | kPciCommandInterruptDisable));
  EXPECT_THAT(pci_->BarWritesWhileDecoding(), t::Eq(0));
  // The console names the device and the state that it left.
  EXPECT_THAT(driver_env_->logs_,
              t::ElementsAre("[PCI] 00:06.0: woke from D3hot\n"));

  ASSERT_THAT(ctx.Claim(awake), t::Eq(0));
  EXPECT_THAT(driver_env_->slept_us_, t::ElementsAre(10000));
  EXPECT_THAT(driver_env_->logs_, t::SizeIs(1));
}

TEST_F(DriverTest, MmioMapEnforcesBarOwnershipMsixWithholdingAndGuardPages) {
  DriverContext ctx;
  const int msix_dev = PciFindDevice({0, 4, 0});
  const int msi_dev = PciFindDevice({1, 1, 0});
  const int ide_dev = PciFindDevice({0, 1, 1});
  ASSERT_THAT(ctx.Claim(msix_dev), t::Eq(0));
  ASSERT_THAT(ctx.Claim(ide_dev), t::Eq(0));

  // Unaligned or zero arguments -> EINVAL.
  EXPECT_THAT(ctx.MmioMap(nullptr, 0, 0x1000), t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(ctx.MmioMap(nullptr, 0xFEA70100, 0x1000),
              t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(ctx.MmioMap(nullptr, 0xFEA70000, 0), t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(ctx.MmioMap(nullptr, 0xFEA70000, 0x1800),
              t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(ctx.MmioMap(nullptr, 0xFEA70000, kSyscallMmioMaxSize + 0x1000),
              t::Eq(kSyscallErrorInvalid));

  // Unclaimed device BAR (01:01.0 at 0xFE800000) -> EACCES.
  EXPECT_THAT(ctx.MmioMap(nullptr, 0xFE800000, 0x1000),
              t::Eq(kSyscallErrorAccess));
  // I/O BAR (00:01.1 BAR4 at 0xD040) -> EACCES.
  EXPECT_THAT(ctx.MmioMap(nullptr, 0xD000, 0x1000), t::Eq(kSyscallErrorAccess));
  // Outside claimed BAR bounds -> EACCES.
  EXPECT_THAT(ctx.MmioMap(nullptr, 0xFEA74000, 0x1000),
              t::Eq(kSyscallErrorAccess));
  // MSI-X table (0xFEA73000) and PBA (0xFEA73800) lie on page 0xFEA73000 of
  // BAR0 [0xFEA70000, 0xFEA74000): mapping all 4 pages or just page 3 fails
  // with EACCES.
  EXPECT_THAT(ctx.MmioMap(nullptr, 0xFEA70000, 0x4000),
              t::Eq(kSyscallErrorAccess));
  EXPECT_THAT(ctx.MmioMap(nullptr, 0xFEA73000, 0x1000),
              t::Eq(kSyscallErrorAccess));

  // If the physical range overlaps usable RAM, MmioMap rejects with EACCES.
  driver_env_->ram_range_base_ = 0xFEA70000;
  driver_env_->ram_range_size_ = 0x1000;
  EXPECT_THAT(ctx.MmioMap(nullptr, 0xFEA70000, 0x1000),
              t::Eq(kSyscallErrorAccess));
  driver_env_->ram_range_size_ = 0;

  // Mapping the first 3 pages [0xFEA70000, 0xFEA73000) succeeds!
  const int64_t va1 = ctx.MmioMap(nullptr, 0xFEA70000, 0x3000);
  EXPECT_FALSE(SyscallIsError(va1));
  EXPECT_THAT(static_cast<uintptr_t>(va1), t::Eq(kDriverMmioWindowBase));
  ASSERT_THAT(driver_env_->mapped_pages_.size(), t::Eq(3U));
  for (int i = 0; i < 3; ++i) {
    EXPECT_THAT(driver_env_->mapped_pages_[i].user_addr,
                t::Eq(kDriverMmioWindowBase + i * 0x1000ULL));
    EXPECT_THAT(driver_env_->mapped_pages_[i].phys_addr,
                t::Eq(0xFEA70000ULL + i * 0x1000ULL));
    EXPECT_TRUE(driver_env_->mapped_pages_[i].device_mmio);
  }

  // Next mapping starts after a 1-page unmapped guard gap (+ 4 * 0x1000).
  ASSERT_THAT(ctx.Claim(msi_dev), t::Eq(0));
  const int64_t va2 = ctx.MmioMap(nullptr, 0xFE800000, 0x4000);
  EXPECT_FALSE(SyscallIsError(va2));
  EXPECT_THAT(static_cast<uintptr_t>(va2),
              t::Eq(kDriverMmioWindowBase + 4 * 0x1000ULL));
}

TEST_F(DriverTest, DmaAllocEnforcesAlignmentFlagsZeroingAndQuota) {
  DriverContext ctx;

  EXPECT_THAT(ctx.DmaAlloc(nullptr, 0, 0).status(),
              t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(ctx.DmaAlloc(nullptr, 100, 0).status(),
              t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(ctx.DmaAlloc(nullptr, kSyscallDmaMaxSize + 0x1000, 0).status(),
              t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(ctx.DmaAlloc(nullptr, 0x1000, 0x2).status(),
              t::Eq(kSyscallErrorInvalid));

  // Allocate 2 pages with kSyscallDmaBelow4GiB.
  const StatusOr<DriverDmaAllocation> alloc1 =
      ctx.DmaAlloc(nullptr, 0x2000, kSyscallDmaBelow4GiB);
  ASSERT_TRUE(alloc1.ok());
  EXPECT_THAT(alloc1->user_addr, t::Eq(kDriverDmaWindowBase));
  EXPECT_THAT(alloc1->phys_addr, t::Eq(0x00200000ULL));
  EXPECT_THAT(driver_env_->last_alloc_max_phys_, t::Eq(0x100000000ULL));
  ASSERT_THAT(driver_env_->zeroed_ranges_.size(), t::Eq(1U));
  EXPECT_THAT(driver_env_->zeroed_ranges_[0],
              t::Pair(0x00200000ULL, int64_t{0x2000}));

  // Next allocation starts after 1 guard page (+ 3 * 0x1000) and uses full
  // identity-map limit when flags == 0.
  const StatusOr<DriverDmaAllocation> alloc2 = ctx.DmaAlloc(nullptr, 0x1000, 0);
  ASSERT_TRUE(alloc2.ok());
  EXPECT_THAT(alloc2->user_addr, t::Eq(kDriverDmaWindowBase + 3 * 0x1000ULL));
  EXPECT_THAT(driver_env_->last_alloc_max_phys_, t::Eq(kIdentityMapLimit));

  // Per-process 16 MiB quota: allocating 16 * 1 MiB more exceeds the quota on
  // the 16th 1 MiB call because 3 pages are already allocated.
  for (int i = 0; i < 15; ++i) {
    ASSERT_TRUE(ctx.DmaAlloc(nullptr, kSyscallDmaMaxSize, 0).ok());
  }
  EXPECT_THAT(ctx.DmaAlloc(nullptr, kSyscallDmaMaxSize, 0).status(),
              t::Eq(kSyscallErrorNoMemory));
}

TEST_F(DriverTest, IrqAllocRollbackAndOrderedRelease) {
  const int msix_dev = PciFindDevice({0, 4, 0});
  const int msi_dev = PciFindDevice({1, 1, 0});

  {
    DriverContext ctx;
    ASSERT_THAT(ctx.Claim(msix_dev), t::Eq(0));
    ASSERT_THAT(ctx.Claim(msi_dev), t::Eq(0));
    ASSERT_THAT(ctx.SetBusMaster(msix_dev, 1), t::Eq(0));
    ASSERT_THAT(ctx.SetBusMaster(msi_dev, 1), t::Eq(0));

    // Allocate 2 MSI-X vectors on 00:04.0 -> bits 0..1.
    EXPECT_THAT(ctx.IrqAlloc(msix_dev, 2, kSyscallIrqModeAuto, nullptr),
                t::Eq(0));
    EXPECT_THAT(ctx.NextIrqBit(), t::Eq(2));
    EXPECT_THAT(DeviceIrqUsedVectorCount(), t::Eq(2));

    // Requesting 2 MSI vectors on MSI-only 01:01.0 fails with EINVAL and rolls
    // back only 01:01.0's temporary vectors, leaving 00:04.0's 2 vectors
    // intact!
    EXPECT_THAT(ctx.IrqAlloc(msi_dev, 2, kSyscallIrqModeMsi, nullptr),
                t::Eq(kSyscallErrorInvalid));
    EXPECT_THAT(ctx.NextIrqBit(), t::Eq(2));
    EXPECT_THAT(DeviceIrqUsedVectorCount(), t::Eq(2));

    // Now allocate 1 MSI vector on 01:01.0 -> bit 2.
    EXPECT_THAT(ctx.IrqAlloc(msi_dev, 1, kSyscallIrqModeAuto, nullptr),
                t::Eq(2));
    EXPECT_THAT(ctx.NextIrqBit(), t::Eq(3));
    EXPECT_THAT(DeviceIrqUsedVectorCount(), t::Eq(3));

    // Duplicate IrqAlloc on already-configured 01:01.0 returns EBUSY without
    // leaking vectors.
    EXPECT_THAT(ctx.IrqAlloc(msi_dev, 1, kSyscallIrqModeAuto, nullptr),
                t::Eq(kSyscallErrorBusy));
    EXPECT_THAT(DeviceIrqUsedVectorCount(), t::Eq(3));

    const DriverReleaseRecord rec = ctx.Release(/*address_space_alive=*/true);
    EXPECT_THAT(rec.quiesced_count, t::Eq(2));
    EXPECT_THAT(rec.freed_vectors, t::Eq(3));
    EXPECT_THAT(rec.released_claims, t::Eq(2));
    EXPECT_TRUE(rec.quiesced_before_vectors_freed);
    EXPECT_TRUE(rec.quiesced_before_address_space_destroyed);
  }

  // All vectors and PCI claims are back to baseline, and both devices are
  // quiesced (Bus Master cleared, MSI/MSI-X disabled).
  EXPECT_THAT(DeviceIrqUsedVectorCount(), t::Eq(0));
  EXPECT_THAT(PciClaimedCount(), t::Eq(0));
  EXPECT_THAT(pci_->Peek16({0, 4, 0}, kPciCommandOffset) & kPciCommandBusMaster,
              t::Eq(0U));
  EXPECT_THAT(pci_->Peek16({1, 1, 0}, kPciCommandOffset) & kPciCommandBusMaster,
              t::Eq(0U));

  const DriverReleaseRecord last = DriverLastReleaseRecord();
  EXPECT_THAT(last.quiesced_count, t::Eq(2));
  EXPECT_THAT(last.freed_vectors, t::Eq(3));
  EXPECT_THAT(last.released_claims, t::Eq(2));
}

TEST_F(DriverTest, ClaimEnforcesMaxClaimsLimitAndIdempotencyAtCap) {
  DriverResetForTest();
  PciResetForTest();
  pci_ = std::make_shared<FakePciConfigSpace>();
  for (uint8_t dev = 0; dev < 18; ++dev) {
    pci_->AddFunction({0, dev, 0}, {.vendor_id = 0x1B36,
                                    .device_id = 0x000D,
                                    .class_code = 0x0C,
                                    .subclass = 0x03,
                                    .prog_if = 0x30});
  }
  PciInit(pci_);
  DriverInit(driver_env_, irq_env_);
  ASSERT_THAT(PciDeviceCount(), t::Eq(18));

  DriverContext ctx_a;
  DriverContext ctx_b;
  for (int i = 0; i < kSyscallMaxClaims; ++i) {
    ASSERT_THAT(ctx_a.Claim(i), t::Eq(0)) << "claim " << i;
  }
  EXPECT_THAT(ctx_a.ClaimedCount(), t::Eq(kSyscallMaxClaims));

  // Re-claiming an already-claimed device at the cap remains idempotent (0).
  EXPECT_THAT(ctx_a.Claim(0), t::Eq(0));
  EXPECT_THAT(ctx_a.Claim(kSyscallMaxClaims - 1), t::Eq(0));

  // A 17th device claimed by another driver returns EBUSY, while an unclaimed
  // 17th device returns ENOMEM.
  ASSERT_THAT(ctx_b.Claim(kSyscallMaxClaims), t::Eq(0));
  EXPECT_THAT(ctx_a.Claim(kSyscallMaxClaims), t::Eq(kSyscallErrorBusy));
  EXPECT_THAT(ctx_a.Claim(kSyscallMaxClaims + 1), t::Eq(kSyscallErrorNoMemory));
}

TEST_F(DriverTest,
       MmioMapHandlesAddressOverflowInvalidMsixSeparatePbaAndMapFailures) {
  DriverResetForTest();
  PciResetForTest();
  pci_ = std::make_shared<FakePciConfigSpace>();
  pci_->AddFunction({0, 4, 0}, {.vendor_id = 0x1B36,
                                .device_id = 0x000D,
                                .class_code = 0x0C,
                                .subclass = 0x03,
                                .prog_if = 0x30,
                                .command = kPciCommandMemorySpace});
  pci_->SetBar({0, 4, 0}, 0,
               {.kind = FakePciConfigSpace::BarKind::kMem64,
                .base = 0xFEA70000ULL,
                .size = 0x4000});
  pci_->AddMsix({0, 4, 0}, 0x90,
                {.table_size = 16,
                 .table_bar = 0,
                 .table_offset = 0x1000,
                 .pba_bar = 0,
                 .pba_offset = 0x3000});

  pci_->AddFunction({0, 6, 0}, {.vendor_id = 0x1B36,
                                .device_id = 0x000D,
                                .class_code = 0x0C,
                                .subclass = 0x03,
                                .prog_if = 0x30,
                                .command = kPciCommandMemorySpace});
  pci_->SetBar({0, 6, 0}, 0,
               {.kind = FakePciConfigSpace::BarKind::kMem64,
                .base = 0xFEA80000ULL,
                .size = 0x2000});
  // Invalid MSI-X (table_bar = 4, which is unimplemented).
  pci_->AddMsix({0, 6, 0}, 0x90,
                {.table_size = 16,
                 .table_bar = 4,
                 .table_offset = 0,
                 .pba_bar = 4,
                 .pba_offset = 0x800});
  PciInit(pci_);
  DriverInit(driver_env_, irq_env_);

  DriverContext ctx;
  // Unclaimed DriverFindClaimedBarForRange with 0 claims returns EACCES.
  EXPECT_THAT(ctx.MmioMap(nullptr, 0xFEA70000ULL, 0x1000),
              t::Eq(kSyscallErrorAccess));
  // 64-bit phys_addr + size overflow returns EINVAL.
  EXPECT_THAT(ctx.MmioMap(nullptr, 0xFFFFFFFFFFFFF000ULL, 0x2000),
              t::Eq(kSyscallErrorInvalid));

  ASSERT_THAT(ctx.Claim(0), t::Eq(0));
  ASSERT_THAT(ctx.Claim(1), t::Eq(0));

  // Dev 0: pages at +0x0000 and +0x2000 are mappable; +0x1000 (table) and
  // +0x3000 (PBA) are withheld with EACCES.
  EXPECT_FALSE(SyscallIsError(ctx.MmioMap(nullptr, 0xFEA70000ULL, 0x1000)));
  EXPECT_THAT(ctx.MmioMap(nullptr, 0xFEA71000ULL, 0x1000),
              t::Eq(kSyscallErrorAccess));
  EXPECT_FALSE(SyscallIsError(ctx.MmioMap(nullptr, 0xFEA72000ULL, 0x1000)));
  EXPECT_THAT(ctx.MmioMap(nullptr, 0xFEA73000ULL, 0x1000),
              t::Eq(kSyscallErrorAccess));

  // Dev 1 has msix.valid == false, so its BAR pages are not withheld.
  EXPECT_FALSE(SyscallIsError(ctx.MmioMap(nullptr, 0xFEA80000ULL, 0x2000)));

  // MapUserDevicePage failure returns ENOMEM.
  driver_env_->fail_map_after_ = 0;
  EXPECT_THAT(ctx.MmioMap(nullptr, 0xFEA70000ULL, 0x1000),
              t::Eq(kSyscallErrorNoMemory));
  driver_env_->fail_map_after_ = -1;
}

TEST_F(DriverTest, DmaAllocRollsBackUnmappedFramesAndHandlesAllocFailure) {
  DriverContext ctx;

  // Frame allocation failure returns ENOMEM without advancing quota.
  driver_env_->fail_alloc_frames_ = true;
  EXPECT_THAT(ctx.DmaAlloc(nullptr, 0x1000, 0).status(),
              t::Eq(kSyscallErrorNoMemory));
  EXPECT_THAT(ctx.DmaTotalBytes(), t::Eq(0));
  driver_env_->fail_alloc_frames_ = false;

  // Partial mapping failure on page 1 of a 3-page allocation frees the 2
  // remaining unmapped frames and charges quota for the 1 page already mapped
  // into `space` (reclaimed on process teardown).
  driver_env_->fail_map_after_ = 1;
  EXPECT_THAT(ctx.DmaAlloc(nullptr, 0x3000, 0).status(),
              t::Eq(kSyscallErrorNoMemory));
  EXPECT_THAT(driver_env_->allocated_frames_, t::Eq(3));
  EXPECT_THAT(driver_env_->freed_frames_, t::Eq(2));
  EXPECT_THAT(ctx.DmaTotalBytes(), t::Eq(0x1000));
  driver_env_->fail_map_after_ = -1;

  // Mapping failure on page 0 frees all frames and charges 0 bytes.
  driver_env_->fail_map_after_ = 0;
  EXPECT_THAT(ctx.DmaAlloc(nullptr, 0x2000, 0).status(),
              t::Eq(kSyscallErrorNoMemory));
  EXPECT_THAT(driver_env_->allocated_frames_, t::Eq(5));
  EXPECT_THAT(driver_env_->freed_frames_, t::Eq(4));
  EXPECT_THAT(ctx.DmaTotalBytes(), t::Eq(0x1000));
  driver_env_->fail_map_after_ = -1;

  // Remaining quota (16 MiB - 4 KiB) can be allocated cleanly, and exceeding
  // 16 MiB returns ENOMEM.
  for (int i = 0; i < 15; ++i) {
    ASSERT_TRUE(ctx.DmaAlloc(nullptr, kSyscallDmaMaxSize, 0).ok());
  }
  ASSERT_TRUE(ctx.DmaAlloc(nullptr, kSyscallDmaMaxSize - 0x1000, 0).ok());
  EXPECT_THAT(ctx.DmaTotalBytes(), t::Eq(kSyscallDmaMaxTotalSize));
  EXPECT_THAT(ctx.DmaAlloc(nullptr, 0x1000, 0).status(),
              t::Eq(kSyscallErrorNoMemory));
}

TEST_F(DriverTest,
       IrqAllocValidationBitExhaustionBindSelftestAndReleaseIdempotency) {
  DriverContext ctx;
  const int msix_dev = PciFindDevice({0, 4, 0});
  const int msi_dev = PciFindDevice({1, 1, 0});

  // Out-of-range and unclaimed devices.
  EXPECT_THAT(ctx.IrqAlloc(-1, 1, kSyscallIrqModeAuto, nullptr),
              t::Eq(kSyscallErrorNoDevice));
  EXPECT_THAT(ctx.IrqAlloc(99, 1, kSyscallIrqModeAuto, nullptr),
              t::Eq(kSyscallErrorNoDevice));
  EXPECT_THAT(ctx.IrqAlloc(msix_dev, 1, kSyscallIrqModeAuto, nullptr),
              t::Eq(kSyscallErrorAccess));
  EXPECT_FALSE(ctx.BindVectorsForSelftest(msix_dev, 1, nullptr));

  ASSERT_THAT(ctx.Claim(msix_dev), t::Eq(0));
  ASSERT_THAT(ctx.Claim(msi_dev), t::Eq(0));

  // Invalid count or mode.
  EXPECT_THAT(ctx.IrqAlloc(msix_dev, 0, kSyscallIrqModeAuto, nullptr),
              t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(ctx.IrqAlloc(msix_dev, kSyscallIrqMaxCount + 1,
                           kSyscallIrqModeAuto, nullptr),
              t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(ctx.IrqAlloc(msix_dev, 1, -1, nullptr),
              t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(ctx.IrqAlloc(msix_dev, 1, 3, nullptr),
              t::Eq(kSyscallErrorInvalid));

  // BindVectorsForSelftest validates bounds and binds 31 bits (0..30) on
  // msix_dev.
  EXPECT_FALSE(ctx.BindVectorsForSelftest(msix_dev, 0, nullptr));
  EXPECT_FALSE(
      ctx.BindVectorsForSelftest(msix_dev, kSyscallIrqMaxCount + 1, nullptr));
  ASSERT_TRUE(ctx.BindVectorsForSelftest(msix_dev, 31, nullptr));
  EXPECT_THAT(ctx.NextIrqBit(), t::Eq(31));

  // Requesting 2 vectors when only 1 bit remains in the 32-bit per-process
  // mask fails with ENOMEM.
  EXPECT_THAT(ctx.IrqAlloc(msi_dev, 2, kSyscallIrqModeAuto, nullptr),
              t::Eq(kSyscallErrorNoMemory));
  EXPECT_FALSE(ctx.BindVectorsForSelftest(msi_dev, 2, nullptr));

  // Allocating the final bit (bit 31) on msi_dev succeeds.
  EXPECT_THAT(ctx.IrqAlloc(msi_dev, 1, kSyscallIrqModeAuto, nullptr),
              t::Eq(31));
  EXPECT_THAT(ctx.NextIrqBit(), t::Eq(32));

  // Synthetic interrupt on bit 0 is observed and cleared by IrqWait.
  const int vec0 = DeviceIrqVectorForBit(ctx.OwnerToken(), 0);
  ASSERT_THAT(vec0, t::Ge(kDeviceIrqVectorFirst));
  EXPECT_TRUE(DeviceIrqDispatch(vec0));
  EXPECT_THAT(ctx.IrqWait(1ULL, 0, nullptr, nullptr), t::Eq(1));

  // First Release records the teardown metrics; a second Release is a no-op.
  const DriverReleaseRecord first_rec =
      ctx.Release(/*address_space_alive=*/true);
  EXPECT_THAT(first_rec.quiesced_count, t::Eq(2));
  EXPECT_THAT(first_rec.freed_vectors, t::Eq(32));
  EXPECT_THAT(first_rec.released_claims, t::Eq(2));
  EXPECT_TRUE(first_rec.quiesced_before_vectors_freed);
  EXPECT_TRUE(first_rec.quiesced_before_address_space_destroyed);

  const DriverReleaseRecord second_rec =
      ctx.Release(/*address_space_alive=*/false);
  EXPECT_THAT(second_rec.quiesced_count, t::Eq(0));
  EXPECT_TRUE(
      DriverLastReleaseRecord().quiesced_before_address_space_destroyed);
}

TEST_F(DriverTest, ClaimWakesTheBridgesAboveADeviceFromTheRootBusDown) {
  // Root port 00:1c.0 (buses 2-3), a switch port 02:00.0 below it (bus 3) and
  // an Ethernet controller 03:00.0, each with a Power Management capability.
  constexpr PciAddress kRootPort = {0, 0x1C, 0};
  constexpr PciAddress kSwitchPort = {2, 0, 0};
  constexpr PciAddress kNic = {3, 0, 0};
  constexpr int kPmcsr = 0x40 + kPciPmControlStatusOffset;
  pci_->AddFunction(kRootPort, {.vendor_id = 0x8086,
                                .device_id = 0x1E10,
                                .class_code = 0x06,
                                .subclass = 0x04,
                                .header_type = 0x01,
                                .command = 0x0007});
  pci_->SetBridgeBuses(kRootPort, 0, 2, 3);
  pci_->AddFunction(kSwitchPort, {.vendor_id = 0x10B5,
                                  .device_id = 0x8606,
                                  .class_code = 0x06,
                                  .subclass = 0x04,
                                  .header_type = 0x01,
                                  .command = 0x0000});
  pci_->SetBridgeBuses(kSwitchPort, 2, 3, 3);
  pci_->AddFunction(
      kNic, {.vendor_id = 0x8086, .device_id = 0x10D3, .class_code = 0x02});
  pci_->SetBar(kNic,  //
               0,     //
               {.kind = FakePciConfigSpace::BarKind::kMem64,
                .base = 0xFC000000u,
                .size = 0x20000});
  pci_->AddMsix(
      kNic, 0x90,
      {.table_size = 4, .table_bar = 0, .pba_bar = 0, .pba_offset = 0x1000});
  pci_->SetRaw16(kNic, 0x92, kPciMsixControlEnable | 0x0003);
  for (const PciAddress address : {kRootPort, kSwitchPort}) {
    pci_->AddPowerManagement(address, 0x40, {.power_state = 0});
  }
  pci_->AddPowerManagement(kNic, 0x40, {.power_state = 3});
  PciResetForTest();
  PciInit(pci_);
  const int nic = PciFindDevice(kNic);
  ASSERT_THAT(nic, t::Ge(0));
  const uint32_t bar0 = pci_->Peek32(kNic, kPciBar0Offset);
  // The bridges went to D3hot after the enumeration (they reset when they
  // leave it): the controller is out of reach.
  pci_->SetRaw16(kRootPort, kPmcsr, 3);
  pci_->SetRaw16(kSwitchPort, kPmcsr, 3);

  DriverContext ctx;
  ASSERT_THAT(ctx.Claim(nic), t::Eq(0));
  for (const PciAddress address : {kRootPort, kSwitchPort, kNic}) {
    EXPECT_THAT(pci_->Peek16(address, kPmcsr) & kPciPmcsrPowerStateMask,
                t::Eq(0U))
        << t::PrintToString(address);
  }
  EXPECT_THAT(driver_env_->slept_us_, t::ElementsAre(10000, 10000, 10000));
  EXPECT_THAT(driver_env_->logs_,
              t::ElementsAre("[PCI] 00:1c.0: woke from D3hot\n",
                             "[PCI] 02:00.0: woke from D3hot\n",
                             "[PCI] 03:00.0: woke from D3hot\n"));
  // The bridges route bus 3 again, and the controller has its BAR back.
  EXPECT_THAT(pci_->Peek8(kRootPort, kPciSecondaryBusOffset), t::Eq(2));
  EXPECT_THAT(pci_->Peek8(kRootPort, kPciSubordinateBusOffset), t::Eq(3));
  EXPECT_THAT(pci_->Peek8(kSwitchPort, kPciSecondaryBusOffset), t::Eq(3));
  EXPECT_THAT(pci_->Peek16(kRootPort, kPciCommandOffset), t::Eq(0x0007));
  EXPECT_THAT(pci_->Peek16(kSwitchPort, kPciCommandOffset),
              t::Eq(kPciCommandMemorySpace | kPciCommandBusMaster));
  EXPECT_THAT(pci_->Peek32(kNic, kPciBar0Offset), t::Eq(bar0));
  EXPECT_THAT(pci_->Peek16(kNic, kPciCommandOffset),
              t::Eq(kPciCommandMemorySpace | kPciCommandInterruptDisable));
  EXPECT_THAT(pci_->Peek16(kNic, 0x92) & kPciMsixControlEnable, t::Eq(0U));
  EXPECT_THAT(pci_->BarWritesWhileDecoding(), t::Eq(0));

  // If the endpoint itself stays in D0 while an upstream bridge goes to D3hot,
  // claiming it wakes the bridge and still configures the endpoint once the
  // bridge forwards transactions again.
  ctx.Release(/*address_space_alive=*/true);
  pci_->SetRaw16(kNic, kPciCommandOffset, 0);
  pci_->SetRaw16(kNic, 0x92, kPciMsixControlEnable | 0x0003);
  pci_->SetRaw16(kSwitchPort, kPciCommandOffset, 0);
  pci_->SetRaw16(kRootPort, kPmcsr, 3);
  DriverContext ctx2;
  ASSERT_THAT(ctx2.Claim(nic), t::Eq(0));
  EXPECT_THAT(pci_->Peek16(kSwitchPort, kPciCommandOffset),
              t::Eq(kPciCommandMemorySpace | kPciCommandBusMaster));
  EXPECT_THAT(pci_->Peek16(kNic, kPciCommandOffset),
              t::Eq(kPciCommandMemorySpace | kPciCommandInterruptDisable));
  EXPECT_THAT(pci_->Peek16(kNic, 0x92) & kPciMsixControlEnable, t::Eq(0U));
}

TEST_F(DriverTest, ClaimWaitsUntilAWokenDeviceAnswersForAtMostAboutASecond) {
  constexpr PciAddress kSlow = {0, 6, 0};
  constexpr PciAddress kDead = {0, 7, 0};
  for (const PciAddress address : {kSlow, kDead}) {
    pci_->AddFunction(
        address,
        {.vendor_id = 0x8086, .device_id = 0x10D3, .class_code = 0x02});
    pci_->SetBar(address,  //
                 0,        //
                 {.kind = FakePciConfigSpace::BarKind::kMem32,
                  .base = (address == kSlow) ? 0xFC000000u : 0xFC100000u,
                  .size = 0x1000});
  }
  // Still initializing for 3 reads after the recovery time; and for good.
  pci_->AddPowerManagement(kSlow,  //
                           0x40,   //
                           {.power_state = 3, .reads_until_ready = 3});
  pci_->AddPowerManagement(kDead,  //
                           0x40,   //
                           {.power_state = 3, .reads_until_ready = 1000});
  PciResetForTest();
  PciInit(pci_);

  DriverContext ctx;
  ASSERT_THAT(ctx.Claim(PciFindDevice(kSlow)), t::Eq(0));
  EXPECT_THAT(driver_env_->slept_us_, t::ElementsAre(10000, 1000, 2000, 4000));
  EXPECT_THAT(pci_->Peek32(kSlow, kPciBar0Offset), t::Eq(0xFC000000u));
  EXPECT_THAT(driver_env_->logs_,
              t::ElementsAre("[PCI] 00:06.0: woke from D3hot\n"));

  driver_env_->slept_us_.clear();
  driver_env_->logs_.clear();
  ASSERT_THAT(ctx.Claim(PciFindDevice(kDead)), t::Eq(0));
  int64_t polled_us = 0;
  for (int i = 1; i < static_cast<int>(driver_env_->slept_us_.size()); ++i) {
    polled_us += driver_env_->slept_us_[i];
  }
  EXPECT_THAT(driver_env_->slept_us_, t::SizeIs(11));
  EXPECT_THAT(polled_us, t::AllOf(t::Ge(1000000), t::Lt(1100000)));
  EXPECT_THAT(driver_env_->logs_,
              t::ElementsAre("[PCI] 00:07.0: not ready after leaving D3hot\n",
                             "[PCI] 00:07.0: woke from D3hot\n"));
}

TEST_F(DriverTest, ClaimCorrectsBarsThatPciInitSizedWronglyInD3hot) {
  // A controller that ignores BAR writes in D3hot looks like it had a 64 KiB
  // BAR at 0xFC010000.
  constexpr PciAddress kAddress = {0, 6, 0};
  pci_->AddFunction(
      kAddress, {.vendor_id = 0x8086, .device_id = 0x10D3, .class_code = 0x02});
  pci_->SetBar(kAddress,  //
               0,         //
               {.kind = FakePciConfigSpace::BarKind::kMem32,
                .base = 0xFC010000u,
                .size = 0x1000});
  pci_->AddPowerManagement(
      kAddress, 0x40, {.power_state = 3, .ignores_bar_writes_in_d3hot = true});
  PciResetForTest();
  PciInit(pci_);
  const int index = PciFindDevice(kAddress);
  DriverContext ctx;
  StatusOr<DriverPciInfoResult> info = ctx.PciInfo(index);
  ASSERT_TRUE(info.ok());
  ASSERT_THAT(info->info.bars[0].size, t::Eq(0x10000));

  ASSERT_THAT(ctx.Claim(index), t::Eq(0));
  EXPECT_THAT(driver_env_->logs_,
              t::ElementsAre("[PCI] 00:06.0: woke from D3hot\n",
                             "[PCI] 00:06.0: corrected BAR sizes in D0\n"));
  info = ctx.PciInfo(index);
  ASSERT_TRUE(info.ok());
  EXPECT_THAT(info->info.bars[0].phys_base, t::Eq(0xFC010000u));
  EXPECT_THAT(info->info.bars[0].size, t::Eq(0x1000));
  // Beyond the real BAR, mmio_map now refuses.
  EXPECT_THAT(ctx.MmioMap(nullptr, 0xFC011000u, 0x1000),
              t::Eq(kSyscallErrorAccess));
  EXPECT_FALSE(SyscallIsError(ctx.MmioMap(nullptr, 0xFC010000u, 0x1000)));
}

TEST_F(DriverTest, ClaimRoutesTheUsbPortsOfAnIntelXhciFromEhci) {
  constexpr PciAddress kXhci = {0, 0x14, 0};
  constexpr PciAddress kEhci = {0, 0x1A, 0};
  pci_->AddFunction(kXhci, {.vendor_id = 0x8086,
                            .device_id = 0x8C31,
                            .class_code = 0x0C,
                            .subclass = 0x03,
                            .prog_if = 0x30});
  pci_->SetWritable(kXhci, 0xD0, 4, 0x00007FFF);
  pci_->SetWritable(kXhci, 0xD8, 4, 0x0000003F);
  pci_->SetRaw32(kXhci, 0xD4, 0x00007FFF);
  pci_->SetRaw32(kXhci, 0xDC, 0x0000003F);
  pci_->AddFunction(kEhci, {.vendor_id = 0x8086,
                            .device_id = 0x8C2D,
                            .class_code = 0x0C,
                            .subclass = 0x03,
                            .prog_if = 0x20});
  PciResetForTest();
  PciInit(pci_);

  DriverContext ctx;
  ASSERT_THAT(ctx.Claim(PciFindDevice(kXhci)), t::Eq(0));
  EXPECT_THAT(pci_->Peek32(kXhci, 0xD8), t::Eq(0x0000003Fu));
  EXPECT_THAT(pci_->Peek32(kXhci, 0xD0), t::Eq(0x00007FFFu));
  EXPECT_THAT(driver_env_->logs_,
              t::ElementsAre("[PCI] 00:14.0: routed USB ports from EHCI: "
                             "SuperSpeed 0x0000003f, USB 2.0 0x00007fff\n"));
  // QEMU's xHCI needs nothing.
  ASSERT_THAT(ctx.Claim(PciFindDevice({0, 4, 0})), t::Eq(0));
  EXPECT_THAT(driver_env_->logs_, t::SizeIs(1));
}

TEST_F(DriverTest, IrqAllocFailsCleanlyWithoutACpuThatMsisCanAddress) {
  DriverContext ctx;
  const int msix_dev = PciFindDevice({0, 4, 0});
  const int msi_dev = PciFindDevice({1, 1, 0});
  ASSERT_THAT(ctx.Claim(msix_dev), t::Eq(0));
  ASSERT_THAT(ctx.Claim(msi_dev), t::Eq(0));
  // E.g. only x2APIC IDs of 0x100 and up, and no interrupt remapping.
  irq_env_->msi_destination_ = -1;
  EXPECT_THAT(ctx.IrqAlloc(msix_dev, 2, kSyscallIrqModeAuto, nullptr),
              t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(ctx.IrqAlloc(msi_dev, 1, kSyscallIrqModeAuto, nullptr),
              t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(DeviceIrqUsedVectorCount(), t::Eq(0));
  EXPECT_THAT(ctx.NextIrqBit(), t::Eq(0));
  EXPECT_THAT(pci_->Peek16({0, 4, 0}, 0x92) & kPciMsixControlEnable, t::Eq(0U));

  irq_env_->msi_destination_ = 1;
  EXPECT_THAT(ctx.IrqAlloc(msix_dev, 2, kSyscallIrqModeAuto, nullptr),
              t::Eq(0));
  EXPECT_THAT(irq_env_->msix_words_[msix_dev][0], t::Eq(0xFEE01000U));
}

TEST_F(DriverTest, ADeviceWithVectorsGetsNoSecondSetOfThem) {
  DriverContext ctx;
  const int msix_dev = PciFindDevice({0, 4, 0});
  const int msi_dev = PciFindDevice({1, 1, 0});
  ASSERT_THAT(ctx.Claim(msix_dev), t::Eq(0));
  ASSERT_THAT(ctx.Claim(msi_dev), t::Eq(0));

  // Vectors from IrqAlloc.
  ASSERT_THAT(ctx.IrqAlloc(msix_dev, 2, kSyscallIrqModeAuto, nullptr),
              t::Eq(0));
  EXPECT_THAT(ctx.IrqAlloc(msix_dev, 1, kSyscallIrqModeAuto, nullptr),
              t::Eq(kSyscallErrorBusy));
  EXPECT_FALSE(ctx.BindVectorsForSelftest(msix_dev, 1, nullptr));

  // Vectors from BindVectorsForSelftest, which leaves the device unprogrammed.
  ASSERT_TRUE(ctx.BindVectorsForSelftest(msi_dev, 1, nullptr));
  EXPECT_FALSE(ctx.BindVectorsForSelftest(msi_dev, 1, nullptr));
  EXPECT_THAT(ctx.IrqAlloc(msi_dev, 1, kSyscallIrqModeAuto, nullptr),
              t::Eq(kSyscallErrorBusy));

  EXPECT_THAT(ctx.NextIrqBit(), t::Eq(3));
  EXPECT_THAT(DeviceIrqUsedVectorCount(), t::Eq(3));
  EXPECT_THAT(DeviceIrqVectorForBit(ctx.OwnerToken(), 2), t::Ge(0));
  EXPECT_THAT(DeviceIrqVectorForBit(ctx.OwnerToken(), 3), t::Lt(0));
}

class DriverFakeIoApic final : public IoApicHardware {
 public:
  DriverFakeIoApic() { regs_[kIoApicRegVer] = 0x00170011u; }
  bool MapController(const int /*index*/,
                     const uintptr_t /*phys_addr*/) override {
    return true;
  }
  uint32_t ReadReg(const int /*index*/, const uint8_t reg) override {
    return regs_[reg];
  }
  void WriteReg(const int /*index*/,  //
                const uint8_t reg,    //
                const uint32_t value) override {
    regs_[reg] = value;
  }
  uint32_t regs_[256] = {};
};

TEST_F(DriverTest, IsaClaimPortIoIrqAllocAndTeardownOrdering) {
  auto ioapic = std::make_shared<DriverFakeIoApic>();
  const IoApicInfo info = {.ioapic_id = 1, .phys_addr = 0xFEC00000u};
  IoApicInit(ioapic, &info, 1, nullptr);

  DriverContext ctx_a;
  DriverContext ctx_b;

  // Before claiming kIsaDeviceI8042, port I/O and IRQ alloc fail with EACCES.
  EXPECT_THAT(ctx_a.IsaPortIo(kSyscallPortIoRead8, 0x60, 0),
              t::Eq(kSyscallErrorAccess));
  EXPECT_THAT(ctx_a.IsaPortIo(kSyscallPortIoWrite8, 0x64, 0xAD),
              t::Eq(kSyscallErrorAccess));
  EXPECT_THAT(ctx_a.IsaIrqAlloc(kIsaDeviceI8042, nullptr),
              t::Eq(kSyscallErrorAccess));
  EXPECT_THAT(ctx_a.IsaClaim(-1), t::Eq(kSyscallErrorNoDevice));
  EXPECT_THAT(ctx_a.IsaClaim(kIsaDeviceCount), t::Eq(kSyscallErrorNoDevice));

  ASSERT_THAT(ctx_a.IsaClaim(kIsaDeviceI8042), t::Eq(0));
  EXPECT_THAT(ctx_a.IsaClaim(kIsaDeviceI8042), t::Eq(0));
  EXPECT_THAT(ctx_b.IsaClaim(kIsaDeviceI8042), t::Eq(kSyscallErrorBusy));
  EXPECT_THAT(IsaClaimedCount(), t::Eq(1));

  // Port I/O allows 0x60 and 0x64, rejects unauthorized ports and bad args.
  driver_env_->port_in_[0x64] = 0x1C;
  driver_env_->port_in_[0x60] = 0x1E;
  EXPECT_THAT(ctx_a.IsaPortIo(kSyscallPortIoRead8, 0x64, 0), t::Eq(0x1C));
  EXPECT_THAT(ctx_a.IsaPortIo(kSyscallPortIoRead8, 0x60, 0), t::Eq(0x1E));
  EXPECT_THAT(ctx_a.IsaPortIo(kSyscallPortIoWrite8, 0x64, 0xAD), t::Eq(0));
  EXPECT_THAT(driver_env_->port_out_[0x64], t::Eq(0xAD));

  EXPECT_THAT(ctx_a.IsaPortIo(kSyscallPortIoRead8, 0x61, 0),
              t::Eq(kSyscallErrorAccess));
  EXPECT_THAT(ctx_a.IsaPortIo(kSyscallPortIoWrite8, 0xCF8, 0),
              t::Eq(kSyscallErrorAccess));
  EXPECT_THAT(ctx_b.IsaPortIo(kSyscallPortIoRead8, 0x60, 0),
              t::Eq(kSyscallErrorAccess));
  EXPECT_THAT(ctx_a.IsaPortIo(2, 0x60, 0), t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(ctx_a.IsaPortIo(kSyscallPortIoRead8, 0x10000, 0),
              t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(ctx_a.IsaPortIo(kSyscallPortIoRead8, 0x60, 1),
              t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(ctx_a.IsaPortIo(kSyscallPortIoWrite8, 0x60, 0x100),
              t::Eq(kSyscallErrorInvalid));

  // Allocate ISA IRQ 1 and verify IOAPIC pin 1 is unmasked and routes to
  // vector.
  const int64_t bit = ctx_a.IsaIrqAlloc(kIsaDeviceI8042, nullptr);
  ASSERT_THAT(bit, t::Eq(0));
  EXPECT_THAT(ctx_a.IsaIrqAlloc(kIsaDeviceI8042, nullptr),
              t::Eq(kSyscallErrorBusy));
  const int vector = DeviceIrqVectorForBit(ctx_a.OwnerToken(), 0);
  ASSERT_THAT(vector, t::Ge(kDeviceIrqVectorFirst));
  EXPECT_THAT(ioapic->regs_[IoApicRedirLowReg(1)],
              t::Eq(static_cast<uint32_t>(vector)));

  // Release masks IOAPIC pin 1 before freeing IDT vectors and releases claim.
  const DriverReleaseRecord rec = ctx_a.Release(/*address_space_alive=*/true);
  EXPECT_THAT(rec.quiesced_count, t::Eq(1));
  EXPECT_THAT(rec.freed_vectors, t::Eq(1));
  EXPECT_THAT(rec.released_claims, t::Eq(1));
  EXPECT_TRUE(rec.quiesced_before_vectors_freed);
  EXPECT_TRUE(rec.quiesced_before_address_space_destroyed);
  EXPECT_NE(ioapic->regs_[IoApicRedirLowReg(1)] & kIoApicRedirMaskBit, 0u);
  EXPECT_THAT(IsaClaimedCount(), t::Eq(0));
  EXPECT_THAT(DeviceIrqUsedVectorCount(), t::Eq(0));

  // Another DriverContext can now claim and allocate IRQ 1 cleanly.
  ASSERT_THAT(ctx_b.IsaClaim(kIsaDeviceI8042), t::Eq(0));
  ASSERT_THAT(ctx_b.IsaIrqAlloc(kIsaDeviceI8042, nullptr), t::Eq(0));

  // Without an IOAPIC, IsaIrqAlloc returns ENODEV without leaking vectors/bits.
  (void)ctx_b.Release();
  IoApicInit(ioapic, nullptr, 0, nullptr);
  DriverContext ctx_c;
  ASSERT_THAT(ctx_c.IsaClaim(kIsaDeviceI8042), t::Eq(0));
  EXPECT_THAT(ctx_c.IsaIrqAlloc(kIsaDeviceI8042, nullptr),
              t::Eq(kSyscallErrorNoDevice));
  EXPECT_THAT(ctx_c.NextIrqBit(), t::Eq(0));
  EXPECT_THAT(DeviceIrqUsedVectorCount(), t::Eq(0));
}

}  // namespace
}  // namespace protos
