#include "device_irq.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "idt.h"
#include "pci.h"
#include "pci_config_fake.h"
#include "syscall_abi.h"

namespace protos {
namespace {

namespace t = ::testing;

constexpr uintptr_t kOwnerA = 0xA000;
constexpr uintptr_t kOwnerB = 0xB000;

class FakeDeviceIrqEnvironment final : public DeviceIrqEnvironment {
 public:
  void RegisterIdtHandler(const int vector,
                          const InterruptHandlerFn handler) override {
    ASSERT_THAT(vector, t::AllOf(t::Ge(0), t::Lt(kIdtEntryCount)));
    handlers_[vector] = handler;
    ++register_calls_;
  }

  void UnregisterIdtHandler(const int vector) override {
    ASSERT_THAT(vector, t::AllOf(t::Ge(0), t::Lt(kIdtEntryCount)));
    handlers_[vector] = nullptr;
    ++unregister_calls_;
  }

  uint8_t BspApicId() override { return bsp_apic_id_; }

  volatile uint32_t* MapMsixTable(const int device_index,
                                  const PciMsix& msix) override {
    if (fail_map_msix_ || !msix.valid || device_index < 0 ||
        device_index >= kPciMaxDevices) {
      return nullptr;
    }
    msix_words_[device_index].assign(msix.table_size * 4, 0xDEADBEEFU);
    msix_mapped_[device_index] = true;
    return msix_words_[device_index].data();
  }

  void UnmapMsixTable(const int device_index) override {
    ASSERT_THAT(device_index, t::AllOf(t::Ge(0), t::Lt(kPciMaxDevices)));
    msix_mapped_[device_index] = false;
    ++unmap_calls_;
  }

  int64_t NowTicks() override { return ticks_.load(std::memory_order_acquire); }

  void Block() override {
    ++block_calls_;
    if (on_block_) {
      on_block_(this);
    }
  }

  void BlockUntil(const int64_t deadline_tick) override {
    ++block_until_calls_;
    last_deadline_ = deadline_tick;
    if (on_block_until_) {
      on_block_until_(this, deadline_tick);
    }
  }

  void Wake(Task* const task) override {
    wake_count_.fetch_add(1, std::memory_order_acq_rel);
    last_woken_task_.store(task, std::memory_order_release);
  }

  void TriggerIdtInterrupt(const int vector) {
    ASSERT_THAT(vector, t::AllOf(t::Ge(0), t::Lt(kIdtEntryCount)));
    ASSERT_THAT(handlers_[vector], t::NotNull());
    InterruptFrame frame{};
    frame.vector = static_cast<uint64_t>(vector);
    handlers_[vector](&frame);
  }

  uint8_t bsp_apic_id_ = 3;
  bool fail_map_msix_ = false;
  InterruptHandlerFn handlers_[kIdtEntryCount] = {};
  int register_calls_ = 0;
  int unregister_calls_ = 0;
  std::vector<uint32_t> msix_words_[kPciMaxDevices];
  bool msix_mapped_[kPciMaxDevices] = {};
  int unmap_calls_ = 0;

  std::atomic<int64_t> ticks_{100};
  int block_calls_ = 0;
  int block_until_calls_ = 0;
  int64_t last_deadline_ = 0;
  std::atomic<int> wake_count_{0};
  std::atomic<Task*> last_woken_task_{nullptr};

  void (*on_block_)(FakeDeviceIrqEnvironment* env) = nullptr;
  void (*on_block_until_)(FakeDeviceIrqEnvironment* env,
                          int64_t deadline) = nullptr;
};

class DeviceIrqTest : public t::Test {
 protected:
  void SetUp() override {
    PciResetForTest();
    DeviceIrqResetForTest();
    pci_ = std::make_shared<FakePciConfigSpace>();
    env_ = std::make_shared<FakeDeviceIrqEnvironment>();
    AddQemuPcMachine(pci_.get(), QemuFirmware::kSeaBios, QemuTopology::kMulti);
    PciInit(pci_);
    DeviceIrqInit(env_);
  }

  void TearDown() override {
    DeviceIrqResetForTest();
    PciResetForTest();
  }

  std::shared_ptr<FakePciConfigSpace> pci_;
  std::shared_ptr<FakeDeviceIrqEnvironment> env_;
};

TEST_F(DeviceIrqTest, MsiAddressAndDataEncodingFollowLocalApicFormat) {
  EXPECT_THAT(DeviceIrqMsiAddressLow(0), t::Eq(0xFEE00000U));
  EXPECT_THAT(DeviceIrqMsiAddressLow(3), t::Eq(0xFEE03000U));
  EXPECT_THAT(DeviceIrqMsiAddressLow(0xFF), t::Eq(0xFEEFF000U));
  EXPECT_THAT(DeviceIrqMsiData(0x50), t::Eq(0x0050U));
  EXPECT_THAT(DeviceIrqMsiData(0x7F), t::Eq(0x007FU));
}

TEST_F(DeviceIrqTest, VectorPoolAllocatesRoundRobinAndExhaustsCleanly) {
  std::atomic<uint64_t> pending{0};
  Task* const fake_task = reinterpret_cast<Task*>(uintptr_t{0x1234});

  int vectors_a[16] = {};
  ASSERT_THAT(DeviceIrqAllocateVectors(kOwnerA, 0, 16, 0, &pending, fake_task,
                                       vectors_a),
              t::Eq(DeviceIrqStatus::kOk));
  EXPECT_THAT(DeviceIrqUsedVectorCount(), t::Eq(16));
  for (int i = 0; i < 16; ++i) {
    EXPECT_THAT(vectors_a[i], t::Eq(kDeviceIrqVectorFirst + i));
    EXPECT_THAT(DeviceIrqVectorOwner(vectors_a[i]), t::Eq(kOwnerA));
    EXPECT_THAT(env_->handlers_[vectors_a[i]], t::NotNull());
  }

  int vectors_b[32] = {};
  ASSERT_THAT(DeviceIrqAllocateVectors(kOwnerB, 1, 32, 0, &pending, fake_task,
                                       vectors_b),
              t::Eq(DeviceIrqStatus::kOk));
  EXPECT_THAT(DeviceIrqUsedVectorCount(), t::Eq(kDeviceIrqVectorCount));

  // Pool is full (48/48 used): even a 1-vector request fails with kNoMemory.
  int extra = 0;
  EXPECT_THAT(
      DeviceIrqAllocateVectors(kOwnerA, 0, 1, 16, &pending, fake_task, &extra),
      t::Eq(DeviceIrqStatus::kNoMemory));

  // Freeing OwnerA releases 16 vectors and unregisters their IDT handlers.
  EXPECT_THAT(DeviceIrqFreeVectors(kOwnerA), t::Eq(16));
  EXPECT_THAT(DeviceIrqUsedVectorCount(), t::Eq(32));
  for (int i = 0; i < 16; ++i) {
    EXPECT_THAT(DeviceIrqVectorOwner(vectors_a[i]), t::Eq(kPciNoOwner));
    EXPECT_THAT(env_->handlers_[vectors_a[i]], t::IsNull());
  }

  // Requesting 17 vectors when only 16 are free fails without partial
  // allocation.
  int too_many[17] = {};
  EXPECT_THAT(DeviceIrqAllocateVectors(kOwnerA, 0, 17, 0, &pending, fake_task,
                                       too_many),
              t::Eq(DeviceIrqStatus::kNoMemory));
  EXPECT_THAT(DeviceIrqUsedVectorCount(), t::Eq(32));

  EXPECT_THAT(DeviceIrqFreeVectors(kOwnerB), t::Eq(32));
  EXPECT_THAT(DeviceIrqUsedVectorCount(), t::Eq(0));
}

TEST_F(DeviceIrqTest, TopHalfDispatchSetsPendingBitAndWakesTask) {
  std::atomic<uint64_t> pending{0};
  Task* const fake_task = reinterpret_cast<Task*>(uintptr_t{0xCAFE});
  int vectors[2] = {};
  ASSERT_THAT(
      DeviceIrqAllocateVectors(kOwnerA, 0, 2, 5, &pending, fake_task, vectors),
      t::Eq(DeviceIrqStatus::kOk));

  // Stray interrupt on an unallocated vector is a safe no-op.
  EXPECT_FALSE(DeviceIrqDispatch(vectors[1] + 1));
  EXPECT_FALSE(DeviceIrqDispatch(0x20));
  EXPECT_THAT(pending.load(), t::Eq(0ULL));
  EXPECT_THAT(env_->wake_count_.load(), t::Eq(0));

  // Triggering the registered IDT handler sets bit (5 + i) and wakes the task.
  env_->TriggerIdtInterrupt(vectors[0]);
  EXPECT_THAT(pending.load(), t::Eq(uint64_t{1} << 5));
  EXPECT_THAT(env_->wake_count_.load(), t::Eq(1));
  EXPECT_THAT(env_->last_woken_task_.load(), t::Eq(fake_task));

  env_->TriggerIdtInterrupt(vectors[1]);
  EXPECT_THAT(pending.load(), t::Eq((uint64_t{1} << 5) | (uint64_t{1} << 6)));
  EXPECT_THAT(env_->wake_count_.load(), t::Eq(2));

  // After freeing vectors, a stray late interrupt on the old vector is a no-op.
  DeviceIrqFreeVectors(kOwnerA);
  EXPECT_FALSE(DeviceIrqDispatch(vectors[0]));
}

TEST_F(DeviceIrqTest, ProgramsMsi64AndRejectsMultiVectorMsi) {
  const int msi_dev = PciFindDevice({1, 1, 0});
  ASSERT_THAT(msi_dev, t::Ge(0));
  ASSERT_THAT(PciClaim(msi_dev, kOwnerA), t::Eq(PciStatus::kOk));

  const int vectors[2] = {0x50, 0x51};
  // Multi-vector MSI is rejected.
  EXPECT_THAT(DeviceIrqProgramDevice(msi_dev, 2, kSyscallIrqModeMsi, vectors),
              t::Eq(DeviceIrqStatus::kInvalid));
  // MSI-only device rejects explicit MSI-X mode.
  EXPECT_THAT(DeviceIrqProgramDevice(msi_dev, 1, kSyscallIrqModeMsix, vectors),
              t::Eq(DeviceIrqStatus::kInvalid));

  // Auto mode selects MSI on an MSI-only device.
  ASSERT_THAT(DeviceIrqProgramDevice(msi_dev, 1, kSyscallIrqModeAuto, vectors),
              t::Eq(DeviceIrqStatus::kOk));
  const PciMsi& msi = PciGetDevice(msi_dev)->msi;
  EXPECT_THAT(pci_->Peek32({1, 1, 0}, msi.offset + 4),
              t::Eq(DeviceIrqMsiAddressLow(env_->bsp_apic_id_)));
  EXPECT_THAT(pci_->Peek32({1, 1, 0}, msi.offset + 8), t::Eq(0U));
  EXPECT_THAT(pci_->Peek16({1, 1, 0}, msi.data_offset), t::Eq(0x0050U));
  EXPECT_THAT(pci_->Peek16({1, 1, 0}, msi.offset + kPciMsiControlOffset) &
                  kPciMsiControlEnable,
              t::Ne(0U));
  EXPECT_THAT(
      pci_->Peek16({1, 1, 0}, kPciCommandOffset) & kPciCommandInterruptDisable,
      t::Ne(0U));

  // Second irq_alloc on the same claim returns kBusy.
  EXPECT_THAT(DeviceIrqProgramDevice(msi_dev, 1, kSyscallIrqModeAuto, vectors),
              t::Eq(DeviceIrqStatus::kBusy));

  // Release resets the per-claim state so a subsequent claim can program again.
  DeviceIrqReleaseDevice(msi_dev);
  ASSERT_THAT(DeviceIrqProgramDevice(msi_dev, 1, kSyscallIrqModeMsi, vectors),
              t::Eq(DeviceIrqStatus::kOk));
}

TEST_F(DeviceIrqTest, ProgramsMsiWithPerVectorMaskingAndMasksOnRelease) {
  pci_->AddFunction({0, 9, 0}, {.vendor_id = 0x1234,
                                .device_id = 0x5678,
                                .class_code = 0x0C,
                                .subclass = 0x03,
                                .prog_if = 0x30});
  pci_->AddMsi({0, 9, 0}, 0x60,
               {.is_64bit = false, .per_vector_masking = true});
  DeviceIrqResetForTest();
  PciResetForTest();
  pci_ = std::make_shared<FakePciConfigSpace>();
  pci_->AddFunction({0, 9, 0}, {.vendor_id = 0x1234,
                                .device_id = 0x5678,
                                .class_code = 0x0C,
                                .subclass = 0x03,
                                .prog_if = 0x30});
  pci_->AddMsi({0, 9, 0}, 0x60,
               {.is_64bit = false, .per_vector_masking = true});
  PciInit(pci_);
  DeviceIrqInit(env_);

  ASSERT_THAT(PciClaim(0, kOwnerA), t::Eq(PciStatus::kOk));
  const int vector = 0x55;
  ASSERT_THAT(DeviceIrqProgramDevice(0, 1, kSyscallIrqModeMsi, &vector),
              t::Eq(DeviceIrqStatus::kOk));
  const PciMsi& msi = PciGetDevice(0)->msi;
  EXPECT_THAT(pci_->Peek32({0, 9, 0}, msi.mask_offset), t::Eq(0xFFFFFFFEU));

  DeviceIrqReleaseDevice(0);
  EXPECT_THAT(pci_->Peek32({0, 9, 0}, msi.mask_offset), t::Eq(0xFFFFFFFFU));
}

TEST_F(DeviceIrqTest, ProgramsMsixTableAndMasksAndUnmapsOnRelease) {
  const int msix_dev = PciFindDevice({0, 4, 0});
  ASSERT_THAT(msix_dev, t::Ge(0));
  ASSERT_THAT(PciClaim(msix_dev, kOwnerA), t::Eq(PciStatus::kOk));

  const int vectors[4] = {0x50, 0x51, 0x52, 0x53};
  // Count exceeding msix.table_size (16) is rejected.
  int many[17] = {};
  for (int i = 0; i < 17; ++i) {
    many[i] = 0x50 + i;
  }
  EXPECT_THAT(DeviceIrqProgramDevice(msix_dev, 17, kSyscallIrqModeMsix, many),
              t::Eq(DeviceIrqStatus::kInvalid));

  // Auto mode prefers MSI-X when valid.
  ASSERT_THAT(DeviceIrqProgramDevice(msix_dev, 4, kSyscallIrqModeAuto, vectors),
              t::Eq(DeviceIrqStatus::kOk));
  EXPECT_TRUE(env_->msix_mapped_[msix_dev]);

  const std::vector<uint32_t>& table = env_->msix_words_[msix_dev];
  ASSERT_THAT(table.size(), t::Eq(16U * 4U));
  for (int i = 0; i < 4; ++i) {
    EXPECT_THAT(table[i * 4 + 0],
                t::Eq(DeviceIrqMsiAddressLow(env_->bsp_apic_id_)));
    EXPECT_THAT(table[i * 4 + 1], t::Eq(0U));
    EXPECT_THAT(table[i * 4 + 2], t::Eq(static_cast<uint32_t>(vectors[i])))
        << "entry " << i;
    EXPECT_THAT(table[i * 4 + 3], t::Eq(0U)) << "entry " << i;
  }
  for (int i = 4; i < 16; ++i) {
    EXPECT_THAT(table[i * 4 + 3], t::Eq(1U)) << "entry " << i;
  }
  const uint16_t msix_ctl =
      pci_->Peek16({0, 4, 0}, PciGetDevice(msix_dev)->msix.offset + 2);
  EXPECT_THAT(msix_ctl & kPciMsixControlEnable, t::Ne(0U));
  EXPECT_THAT(msix_ctl & kPciMsixControlFunctionMask, t::Eq(0U));

  // Release masks all 16 entries and unmaps the kernel MSI-X table mapping.
  DeviceIrqReleaseDevice(msix_dev);
  for (int i = 0; i < 16; ++i) {
    EXPECT_THAT(table[i * 4 + 3], t::Eq(1U)) << "entry " << i;
  }
  EXPECT_FALSE(env_->msix_mapped_[msix_dev]);
  EXPECT_THAT(env_->unmap_calls_, t::Eq(1));
}

TEST_F(DeviceIrqTest, WaitValidatesArgumentsPollsSleepsAndConsumesAtomically) {
  std::atomic<uint64_t> pending{0};

  // Bits >= 32 or timeout > kSyscallIrqWaitMaxTicks fail with EINVAL.
  EXPECT_THAT(DeviceIrqWait(&pending, uint64_t{1} << 32, 0, nullptr, nullptr),
              t::Eq(kSyscallErrorInvalid));
  EXPECT_THAT(
      DeviceIrqWait(&pending, 1, kSyscallIrqWaitMaxTicks + 1, nullptr, nullptr),
      t::Eq(kSyscallErrorInvalid));

  // Non-blocking poll (timeout == 0) returns 0 without blocking when empty.
  EXPECT_THAT(DeviceIrqWait(&pending, 0x3, 0, nullptr, nullptr), t::Eq(0));
  EXPECT_THAT(env_->block_calls_, t::Eq(0));
  EXPECT_THAT(env_->block_until_calls_, t::Eq(0));

  // Fast path: pending bits matching mask are returned and cleared; non-mask
  // bits remain intact.
  pending.store(0b1011, std::memory_order_release);
  EXPECT_THAT(DeviceIrqWait(&pending, 0b0011, 0, nullptr, nullptr),
              t::Eq(0b0011));
  EXPECT_THAT(pending.load(), t::Eq(0b1000ULL));

  // Pure tick sleep: mask == 0, timeout > 0 advances until deadline and returns
  // 0 without consuming pending bits.
  env_->on_block_until_ = [](FakeDeviceIrqEnvironment* const e,
                             const int64_t deadline) {
    e->ticks_.store(deadline, std::memory_order_release);
  };
  EXPECT_THAT(DeviceIrqWait(&pending, 0, 5, nullptr, nullptr), t::Eq(0));
  EXPECT_THAT(env_->block_until_calls_, t::Eq(1));
  EXPECT_THAT(env_->last_deadline_, t::Eq(105));
  EXPECT_THAT(pending.load(), t::Eq(0b1000ULL));

  // Infinite sleep with mask == 0, timeout == 0 blocks until should_stop is
  // true (used by kDriverHold when killed).
  int polls = 0;
  env_->on_block_ = [](FakeDeviceIrqEnvironment*) {};
  const auto stop_after_two = [](void* const ctx) -> bool {
    int* const count = static_cast<int*>(ctx);
    return ++(*count) >= 2;
  };
  EXPECT_THAT(DeviceIrqWait(&pending, 0, 0, stop_after_two, &polls), t::Eq(0));
  EXPECT_THAT(env_->block_calls_, t::Eq(1));
}

TEST_F(DeviceIrqTest, ConcurrentDispatchAndWaitLoseNoInterrupts) {
  std::atomic<uint64_t> pending{0};
  int vectors[4] = {};
  ASSERT_THAT(
      DeviceIrqAllocateVectors(kOwnerA, 0, 4, 0, &pending, nullptr, vectors),
      t::Eq(DeviceIrqStatus::kOk));

  std::atomic<bool> producer_done{false};
  std::atomic<uint64_t> seen_mask{0};
  std::thread producer([this, &vectors, &producer_done] {
    for (int round = 0; round < 200; ++round) {
      for (int i = 0; i < 4; ++i) {
        env_->TriggerIdtInterrupt(vectors[i]);
      }
    }
    producer_done.store(true, std::memory_order_release);
  });

  while (!producer_done.load(std::memory_order_acquire) ||
         pending.load(std::memory_order_acquire) != 0) {
    const int64_t fired = DeviceIrqWait(&pending, 0xF, 0, nullptr, nullptr);
    ASSERT_THAT(fired, t::Ge(0));
    seen_mask.fetch_or(static_cast<uint64_t>(fired), std::memory_order_relaxed);
  }
  producer.join();
  EXPECT_THAT(seen_mask.load(), t::Eq(0xFULL));
}

TEST_F(DeviceIrqTest,
       AllocateAndProgramValidateEdgeCasesMapFailuresAndInspectionHelpers) {
  std::atomic<uint64_t> pending{0};
  int vectors[4] = {};

  // Invalid count and first_bit combinations in DeviceIrqAllocateVectors.
  EXPECT_THAT(
      DeviceIrqAllocateVectors(kOwnerA, 0, 0, 0, &pending, nullptr, vectors),
      t::Eq(DeviceIrqStatus::kInvalid));
  EXPECT_THAT(
      DeviceIrqAllocateVectors(kOwnerA, 0, 33, 0, &pending, nullptr, vectors),
      t::Eq(DeviceIrqStatus::kInvalid));
  EXPECT_THAT(
      DeviceIrqAllocateVectors(kOwnerA, 0, 1, -1, &pending, nullptr, vectors),
      t::Eq(DeviceIrqStatus::kInvalid));
  EXPECT_THAT(
      DeviceIrqAllocateVectors(kOwnerA, 0, 4, 30, &pending, nullptr, vectors),
      t::Eq(DeviceIrqStatus::kInvalid));

  // DeviceIrqProgramDevice rejects invalid device indices, unclaimable bridges,
  // endpoints without MSI/MSI-X, and out-of-range vector numbers.
  const int valid_vec = kDeviceIrqVectorFirst;
  EXPECT_THAT(DeviceIrqProgramDevice(-1, 1, kSyscallIrqModeAuto, &valid_vec),
              t::Eq(DeviceIrqStatus::kInvalid));
  EXPECT_THAT(DeviceIrqProgramDevice(99, 1, kSyscallIrqModeAuto, &valid_vec),
              t::Eq(DeviceIrqStatus::kInvalid));
  EXPECT_THAT(DeviceIrqProgramDevice(PciFindDevice({0, 0, 0}), 1,
                                     kSyscallIrqModeAuto, &valid_vec),
              t::Eq(DeviceIrqStatus::kInvalid));
  EXPECT_THAT(DeviceIrqProgramDevice(PciFindDevice({0, 2, 0}), 1,
                                     kSyscallIrqModeAuto, &valid_vec),
              t::Eq(DeviceIrqStatus::kInvalid));

  const int msix_dev = PciFindDevice({0, 4, 0});
  const int msi_dev = PciFindDevice({1, 1, 0});
  ASSERT_THAT(PciClaim(msix_dev, kOwnerA), t::Eq(PciStatus::kOk));
  ASSERT_THAT(PciClaim(msi_dev, kOwnerA), t::Eq(PciStatus::kOk));

  const int low_vec = kDeviceIrqVectorFirst - 1;
  const int high_vec = kDeviceIrqVectorLast + 1;
  EXPECT_THAT(
      DeviceIrqProgramDevice(msix_dev, 1, kSyscallIrqModeAuto, &low_vec),
      t::Eq(DeviceIrqStatus::kInvalid));
  EXPECT_THAT(
      DeviceIrqProgramDevice(msix_dev, 1, kSyscallIrqModeAuto, &high_vec),
      t::Eq(DeviceIrqStatus::kInvalid));

  // If MapMsixTable fails, DeviceIrqProgramDevice returns kNoMemory and clears
  // g_device_configured so a subsequent retry succeeds.
  env_->fail_map_msix_ = true;
  EXPECT_THAT(
      DeviceIrqProgramDevice(msix_dev, 1, kSyscallIrqModeMsix, &valid_vec),
      t::Eq(DeviceIrqStatus::kNoMemory));
  env_->fail_map_msix_ = false;

  ASSERT_THAT(DeviceIrqAllocateVectors(kOwnerA, msix_dev, 2, 3, &pending,
                                       nullptr, vectors),
              t::Eq(DeviceIrqStatus::kOk));
  int msi_vec = 0;
  ASSERT_THAT(DeviceIrqAllocateVectors(kOwnerA, msi_dev, 1, 7, &pending,
                                       nullptr, &msi_vec),
              t::Eq(DeviceIrqStatus::kOk));
  ASSERT_THAT(DeviceIrqProgramDevice(msix_dev, 2, kSyscallIrqModeMsix, vectors),
              t::Eq(DeviceIrqStatus::kOk));
  // DeviceIrqAllocateVectors on an already-configured device returns kBusy.
  int busy_vec = 0;
  EXPECT_THAT(DeviceIrqAllocateVectors(kOwnerA, msix_dev, 1, 10, &pending,
                                       nullptr, &busy_vec),
              t::Eq(DeviceIrqStatus::kBusy));

  // Inspection helpers: DeviceIrqVectorOwner, DeviceIrqVectorForBit, and
  // DeviceIrqReadMsixTableWord.
  EXPECT_THAT(DeviceIrqVectorOwner(kDeviceIrqVectorFirst - 1),
              t::Eq(kPciNoOwner));
  EXPECT_THAT(DeviceIrqVectorOwner(kDeviceIrqVectorLast + 1),
              t::Eq(kPciNoOwner));
  EXPECT_THAT(DeviceIrqVectorForBit(kPciNoOwner, 3), t::Eq(-1));
  EXPECT_THAT(DeviceIrqVectorForBit(kOwnerA, -1), t::Eq(-1));
  EXPECT_THAT(DeviceIrqVectorForBit(kOwnerA, 32), t::Eq(-1));
  EXPECT_THAT(DeviceIrqVectorForBit(kOwnerA, 0), t::Eq(-1));
  EXPECT_THAT(DeviceIrqVectorForBit(kOwnerA, 3), t::Eq(vectors[0]));
  EXPECT_THAT(DeviceIrqVectorForBit(kOwnerA, 4), t::Eq(vectors[1]));
  EXPECT_THAT(DeviceIrqVectorForBit(kOwnerA, 7), t::Eq(msi_vec));

  EXPECT_THAT(DeviceIrqReadMsixTableWord(-1, 0), t::Eq(0U));
  EXPECT_THAT(DeviceIrqReadMsixTableWord(kPciMaxDevices, 0), t::Eq(0U));
  EXPECT_THAT(DeviceIrqReadMsixTableWord(msix_dev, -1), t::Eq(0U));
  EXPECT_THAT(DeviceIrqReadMsixTableWord(msi_dev, 0), t::Eq(0U));
  EXPECT_THAT(DeviceIrqReadMsixTableWord(msix_dev, 2),
              t::Eq(static_cast<uint32_t>(vectors[0])));

  // DeviceIrqFreeDeviceVectors frees only msix_dev's 2 vectors, keeping
  // msi_dev's vector intact.
  EXPECT_THAT(DeviceIrqFreeDeviceVectors(kOwnerA, msix_dev), t::Eq(2));
  EXPECT_THAT(DeviceIrqVectorForBit(kOwnerA, 3), t::Eq(-1));
  EXPECT_THAT(DeviceIrqVectorForBit(kOwnerA, 7), t::Eq(msi_vec));
}

TEST_F(DeviceIrqTest,
       WaitHandlesTimedExpiryWakeupDuringBlockAndStopPredicates) {
  static std::atomic<uint64_t> pending{0};
  pending.store(0, std::memory_order_release);

  // 1. Timed wait where deadline expires with no matching interrupt returns 0.
  env_->on_block_until_ = [](FakeDeviceIrqEnvironment* const e,
                             const int64_t deadline) {
    e->ticks_.store(deadline, std::memory_order_release);
  };
  EXPECT_THAT(DeviceIrqWait(&pending, 0x3, 10, nullptr, nullptr), t::Eq(0));
  EXPECT_THAT(env_->block_until_calls_, t::Eq(1));

  // 2. Timed wait where interrupt fires before deadline returns fired mask.
  env_->on_block_until_ = [](FakeDeviceIrqEnvironment* const e,
                             const int64_t /*deadline*/) {
    pending.store(0b0110, std::memory_order_release);
    e->ticks_.fetch_add(1, std::memory_order_acq_rel);
  };
  EXPECT_THAT(DeviceIrqWait(&pending, 0b0011, 10, nullptr, nullptr),
              t::Eq(0b0010));
  EXPECT_THAT(pending.load(), t::Eq(0b0100ULL));

  // 3. Indefinite wait (timeout_ticks < 0) blocks via Block() and wakes when
  //    matching bit is set.
  pending.store(0, std::memory_order_release);
  env_->on_block_ = [](FakeDeviceIrqEnvironment*) {
    pending.store(0b1000, std::memory_order_release);
  };
  EXPECT_THAT(DeviceIrqWait(&pending, 0b1000, -1, nullptr, nullptr),
              t::Eq(0b1000));
  EXPECT_THAT(env_->block_calls_, t::Eq(1));

  // 4. Timed wait interrupted by should_stop (process kill) returns 0 early.
  env_->on_block_until_ = [](FakeDeviceIrqEnvironment*, int64_t) {};
  int checks = 0;
  const auto stop_after_block = [](void* const ctx) -> bool {
    return ++(*static_cast<int*>(ctx)) >= 2;
  };
  EXPECT_THAT(DeviceIrqWait(&pending, 0x1, 50, stop_after_block, &checks),
              t::Eq(0));
}

}  // namespace
}  // namespace protos
