#include "device_irq.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <utility>

#include "check.h"
#include "idt.h"
#include "ioapic.h"
#include "pci.h"
#include "spinlock.h"
#include "syscall_abi.h"
#if !__STDC_HOSTED__
#include "paging.h"
#include "pmm.h"
#include "smp.h"
#include "task.h"
#endif

namespace protos {
namespace {

constexpr uint16_t kMsiControlMmeMask = 7U << 4;
constexpr uint32_t kMsixVectorControlMaskBit = 1U << 0;

struct VectorSlot {
  uintptr_t owner = kPciNoOwner;
  int device_index = -1;
  int level_isa_irq = -1;
  std::atomic<uint64_t>* pending = nullptr;
  uint64_t bit = 0;
  Task* task = nullptr;
};

// Guards `g_slots`, `g_next_slot`, `g_device_configured`,
// `g_isa_device_configured`, and `g_msix_tables`. Leaf lock with respect to PCI
// and everything except `g_ioapic_lock` and the scheduler runqueue lock taken
// inside `DeviceIrqEnvironment::Wake()`.
IrqSpinLock g_device_irq_lock;
std::shared_ptr<DeviceIrqEnvironment> g_env;
VectorSlot g_slots[kDeviceIrqVectorCount] = {};
int g_next_slot = 0;
bool g_device_configured[kPciMaxDevices] = {};
bool g_isa_device_configured[kIsaDeviceCount] = {};
std::atomic<int> g_level_isa_slot_count{0};
volatile uint32_t* g_msix_tables[kPciMaxDevices] = {};

static constexpr int SlotIdToIsaDevice(const int slot_id) {
  const int isa_device = -2 - slot_id;
  return (isa_device >= 0 && isa_device < kIsaDeviceCount) ? isa_device : -1;
}

static void ClearVectorSlotLocked(const int slot_index) {
  if (g_slots[slot_index].level_isa_irq >= 0) {
    g_level_isa_slot_count.fetch_sub(1, std::memory_order_release);
  }
  g_slots[slot_index] = VectorSlot{};
}

static void UnmaskWatchedLevelIrqs(std::atomic<uint64_t>* const pending,
                                   const uint64_t mask) {
  if (mask == 0 ||
      g_level_isa_slot_count.load(std::memory_order_acquire) <= 0) {
    return;
  }
  const IrqSpinLockGuard guard(g_device_irq_lock);
  for (int i = 0; i < kDeviceIrqVectorCount; ++i) {
    const VectorSlot& slot = g_slots[i];
    if (slot.owner != kPciNoOwner && slot.pending == pending &&
        (slot.bit & mask) != 0 && slot.level_isa_irq >= 0 &&
        (pending->load(std::memory_order_acquire) & slot.bit) == 0) {
      IoApicSetIsaIrqMasked(slot.level_isa_irq, false);
    }
  }
}

static void DeviceIrqInterruptHandler(InterruptFrame* const frame) {
  DCHECK(frame != nullptr);
  DeviceIrqDispatch(static_cast<int>(frame->vector));
}

static uint64_t TryConsumePending(std::atomic<uint64_t>* const pending,
                                  const uint64_t mask) {
  if (mask == 0) {
    return 0;
  }
  uint64_t current = pending->load(std::memory_order_acquire);
  while ((current & mask) != 0) {
    if (pending->compare_exchange_weak(current, current & ~mask,
                                       std::memory_order_acq_rel,
                                       std::memory_order_acquire)) {
      return current & mask;
    }
  }
  return 0;
}

static DeviceIrqStatus SelectEffectiveMode(const PciDevice& device,  //
                                           const int count,          //
                                           const int64_t mode,       //
                                           int64_t* const out_mode) {
  if (count < 1 || count > kSyscallIrqMaxCount) {
    return DeviceIrqStatus::kInvalid;
  }
  int64_t effective = mode;
  if (mode == kSyscallIrqModeAuto) {
    if (device.msix.offset != 0 && device.msix.valid) {
      effective = kSyscallIrqModeMsix;
    } else if (device.msi.offset != 0) {
      effective = kSyscallIrqModeMsi;
    } else {
      return DeviceIrqStatus::kInvalid;
    }
  } else if (mode != kSyscallIrqModeMsi && mode != kSyscallIrqModeMsix) {
    return DeviceIrqStatus::kInvalid;
  }

  if (effective == kSyscallIrqModeMsi) {
    // x86 Local APIC physical destination mode delivers the vector written in
    // Message Data[7:0]; multi-message MSI requires power-of-two aligned
    // contiguous vectors, so MSI supports `count == 1`.
    if (device.msi.offset == 0 || count != 1) {
      return DeviceIrqStatus::kInvalid;
    }
  } else {
    if (device.msix.offset == 0 || !device.msix.valid ||
        count > device.msix.table_size) {
      return DeviceIrqStatus::kInvalid;
    }
  }
  *out_mode = effective;
  return DeviceIrqStatus::kOk;
}

// The bits of the MSI Mask Bits register that belong to the vectors that the
// function implements (Multiple Message Capable); the others are reserved.
static uint32_t MsiImplementedMaskBits(const PciMsi& msi) {
  return (msi.vectors_log2 >= 5) ? 0xFFFFFFFFU
                                 : (uint32_t{1} << (1 << msi.vectors_log2)) - 1;
}

// Masks or unmasks entry `entry` of the MSI-X table that `table` maps through
// bit 0 of its Vector Control word alone: bits 31:1 are reserved, and software
// must preserve them [PCI Local Bus 3.0 §6.8.2.9] (TLP Processing Hints keep
// Steering Tags there).
static void SetMsixEntryMasked(volatile uint32_t* const table,  //
                               const int entry,                 //
                               const bool masked) {
  volatile uint32_t& vector_control = table[entry * 4 + 3];
  const uint32_t old_value = vector_control;
  const uint32_t new_value = masked ? (old_value | kMsixVectorControlMaskBit)
                                    : (old_value & ~kMsixVectorControlMaskBit);
  if (new_value != old_value) {
    vector_control = new_value;
  }
}

// Programs MSI on `device` (index `device_index`) for one vector
// (DeviceIrqProgramDevice()).
static void ProgramMsi(const int device_index,      //
                       const PciDevice& device,     //
                       const uint32_t address_low,  //
                       const int vector) {
  const PciMsi& msi = device.msi;
  const int control = msi.offset + kPciMsiControlOffset;
  // 1. Turn MSI-X off (with its Function Mask set, as PciQuiesce() does): a
  //    function must not have MSI and MSI-X on at once [PCI Local Bus 3.0
  //    §6.8].
  if (device.msix.offset != 0) {
    PciConfigUpdate16(device_index,                               //
                      device.msix.offset + kPciMsiControlOffset,  //
                      kPciMsixControlFunctionMask,                //
                      kPciMsixControlEnable);
  }
  // 2. Disable MSI and clear Multiple Message Enable before programming.
  PciConfigUpdate16(device_index, control, 0,
                    kPciMsiControlEnable | kMsiControlMmeMask);
  // 3. Write Message Address (and Upper Address = 0 if 64-bit).
  PciConfigWrite32(device_index, msi.offset + 4, address_low);
  if (msi.is_64bit) {
    PciConfigWrite32(device_index, msi.offset + 8, 0);
  }
  // 4. Write Message Data.
  PciConfigWrite16(device_index, msi.data_offset, DeviceIrqMsiData(vector));
  // 5. If per-vector masking is present, unmask vector 0 and mask the other
  //    implemented vectors, leaving the reserved bits as they are.
  if (msi.per_vector_masking) {
    const uint32_t implemented = MsiImplementedMaskBits(msi);
    const uint32_t mask_bits = PciConfigRead32(device_index, msi.mask_offset);
    PciConfigWrite32(device_index, msi.mask_offset,
                     (mask_bits & ~implemented) | (implemented & ~uint32_t{1}));
  }
  // 6. Disable legacy INTx and enable MSI with MME = 0 (1 vector).
  PciUpdateCommand(device_index, kPciCommandInterruptDisable, 0);
  PciConfigUpdate16(device_index, control, kPciMsiControlEnable,
                    kMsiControlMmeMask);
}

// Programs MSI-X entries 0..count - 1 of `device` (index `device_index`),
// whose table `table` maps, with `vectors` (DeviceIrqProgramDevice()).
static void ProgramMsix(const int device_index,          //
                        const PciDevice& device,         //
                        volatile uint32_t* const table,  //
                        const uint32_t address_low,      //
                        const int count,                 //
                        const int* const vectors) {
  const PciMsix& msix = device.msix;
  const int control = msix.offset + kPciMsiControlOffset;
  // 1. Turn MSI off: a function must not have MSI and MSI-X on at once [PCI
  //    Local Bus 3.0 §6.8].
  if (device.msi.offset != 0) {
    PciConfigUpdate16(device_index,                              //
                      device.msi.offset + kPciMsiControlOffset,  //
                      0,                                         //
                      kPciMsiControlEnable);
  }
  // 2. Ensure BAR Memory Space decoding is enabled and legacy INTx is disabled.
  PciUpdateCommand(device_index,
                   kPciCommandMemorySpace | kPciCommandInterruptDisable, 0);
  // 3. Set Enable and Function Mask in one write before touching the table:
  //    some devices ignore table accesses while MSI-X Enable is 0 (Linux's
  //    msix_capability_init() does the same), and the Function Mask keeps
  //    every vector from being signaled until step 6.
  PciConfigUpdate16(device_index, control,
                    kPciMsixControlEnable | kPciMsixControlFunctionMask, 0);
  // 4. Mask all MSI-X table entries.
  for (int e = 0; e < msix.table_size; ++e) {
    SetMsixEntryMasked(table, e, /*masked=*/true);
  }
  // 5. Program entries 0..count - 1 and unmask them.
  for (int i = 0; i < count; ++i) {
    table[i * 4 + 0] = address_low;
    table[i * 4 + 1] = 0;
    table[i * 4 + 2] = DeviceIrqMsiData(vectors[i]);
    SetMsixEntryMasked(table, i, /*masked=*/false);
  }
  // 6. Read back entry 0's Vector Control word to flush posted MMIO writes,
  //    then clear Function Mask.
  (void)table[3];
  PciConfigUpdate16(device_index, control, 0, kPciMsixControlFunctionMask);
}

}  // namespace

void DeviceIrqInit(std::shared_ptr<DeviceIrqEnvironment> env) {
  CHECK(env != nullptr);
  const IrqSpinLockGuard guard(g_device_irq_lock);
  for (int i = 0; i < kDeviceIrqVectorCount; ++i) {
    CHECK(g_slots[i].owner == kPciNoOwner);
  }
  g_env = std::move(env);
  g_next_slot = 0;
  g_level_isa_slot_count.store(0, std::memory_order_release);
  for (int i = 0; i < kPciMaxDevices; ++i) {
    g_device_configured[i] = false;
    g_msix_tables[i] = nullptr;
  }
  for (int i = 0; i < kIsaDeviceCount; ++i) {
    g_isa_device_configured[i] = false;
  }
}

int DeviceIrqMsiDestinationApicId() {
  CHECK(g_env != nullptr);
  return g_env->MsiDestinationApicId();
}

DeviceIrqStatus DeviceIrqAllocateVectors(
    const uintptr_t owner,                 //
    const int device_index,                //
    const int count,                       //
    const int first_bit,                   //
    std::atomic<uint64_t>* const pending,  //
    Task* const task,                      //
    int* const out_vectors) {
  CHECK(g_env != nullptr);
  CHECK(owner != kPciNoOwner);
  CHECK(pending != nullptr);
  CHECK(out_vectors != nullptr);
  if (count < 1 || count > kSyscallIrqMaxCount || first_bit < 0 ||
      first_bit + count > kSyscallIrqMaxCount) {
    return DeviceIrqStatus::kInvalid;
  }

  const IrqSpinLockGuard guard(g_device_irq_lock);
  if (device_index >= 0 && device_index < kPciMaxDevices &&
      g_device_configured[device_index]) {
    return DeviceIrqStatus::kBusy;
  }
  const int isa_device = SlotIdToIsaDevice(device_index);
  if (isa_device >= 0 && g_isa_device_configured[isa_device]) {
    return DeviceIrqStatus::kBusy;
  }
  int free_count = 0;
  for (int i = 0; i < kDeviceIrqVectorCount; ++i) {
    if (g_slots[i].owner == kPciNoOwner) {
      ++free_count;
    }
  }
  if (free_count < count) {
    return DeviceIrqStatus::kNoMemory;
  }

  for (int i = 0; i < count; ++i) {
    while (g_slots[g_next_slot].owner != kPciNoOwner) {
      g_next_slot = (g_next_slot + 1) % kDeviceIrqVectorCount;
    }
    const int slot_index = g_next_slot;
    g_next_slot = (g_next_slot + 1) % kDeviceIrqVectorCount;
    const int vector = kDeviceIrqVectorFirst + slot_index;
    g_slots[slot_index] = VectorSlot{
        .owner = owner,
        .device_index = device_index,
        .pending = pending,
        .bit = uint64_t{1} << (first_bit + i),
        .task = task,
    };
    g_env->RegisterIdtHandler(vector, DeviceIrqInterruptHandler);
    out_vectors[i] = vector;
  }
  return DeviceIrqStatus::kOk;
}

DeviceIrqStatus DeviceIrqProgramDevice(const int device_index,  //
                                       const int count,         //
                                       const int64_t mode,      //
                                       const int* const vectors) {
  CHECK(g_env != nullptr);
  CHECK(vectors != nullptr);
  const PciDevice* const device = PciGetDevice(device_index);
  if (device == nullptr || !device->claimable) {
    return DeviceIrqStatus::kInvalid;
  }

  int64_t effective_mode = kSyscallIrqModeAuto;
  const DeviceIrqStatus mode_status =
      SelectEffectiveMode(*device, count, mode, &effective_mode);
  if (mode_status != DeviceIrqStatus::kOk) {
    return mode_status;
  }
  for (int i = 0; i < count; ++i) {
    if (vectors[i] < kDeviceIrqVectorFirst ||
        vectors[i] > kDeviceIrqVectorLast) {
      return DeviceIrqStatus::kInvalid;
    }
  }
  const int destination = g_env->MsiDestinationApicId();
  if (destination < 0 ||
      !DeviceIrqMsiCanTarget(static_cast<uint32_t>(destination))) {
    return DeviceIrqStatus::kInvalid;
  }

  {
    const IrqSpinLockGuard guard(g_device_irq_lock);
    if (g_device_configured[device_index]) {
      return DeviceIrqStatus::kBusy;
    }
    g_device_configured[device_index] = true;
  }

  const uint32_t address_low =
      DeviceIrqMsiAddressLow(static_cast<uint8_t>(destination));
  if (effective_mode == kSyscallIrqModeMsi) {
    ProgramMsi(device_index, *device, address_low, vectors[0]);
    return DeviceIrqStatus::kOk;
  }

  volatile uint32_t* const table =
      g_env->MapMsixTable(device_index, device->msix);
  if (table == nullptr) {
    const IrqSpinLockGuard guard(g_device_irq_lock);
    g_device_configured[device_index] = false;
    return DeviceIrqStatus::kNoMemory;
  }
  {
    const IrqSpinLockGuard guard(g_device_irq_lock);
    g_msix_tables[device_index] = table;
  }
  ProgramMsix(device_index, *device, table, address_low, count, vectors);
  return DeviceIrqStatus::kOk;
}

void DeviceIrqReleaseDevice(const int device_index) {
  const PciDevice* const device = PciGetDevice(device_index);
  if (device == nullptr) {
    return;
  }
  bool was_configured = false;
  volatile uint32_t* table = nullptr;
  {
    const IrqSpinLockGuard guard(g_device_irq_lock);
    was_configured = g_device_configured[device_index];
    table = g_msix_tables[device_index];
    g_device_configured[device_index] = false;
    g_msix_tables[device_index] = nullptr;
  }
  if (!was_configured) {
    return;
  }

  if (table != nullptr && device->msix.offset != 0 && device->msix.valid) {
    // PciQuiesce() has cleared MSI-X Enable. As when programming the table,
    // set it with the Function Mask (which, like the cleared Bus Master, keeps
    // the function from signaling any vector) while the entries are masked,
    // for the devices that ignore table accesses otherwise; then put Message
    // Control back.
    constexpr uint16_t kEnableAndMask =
        kPciMsixControlEnable | kPciMsixControlFunctionMask;
    const int control = device->msix.offset + kPciMsiControlOffset;
    const uint16_t old_control = PciConfigRead16(device_index, control);
    PciConfigUpdate16(device_index, control, kEnableAndMask, 0);
    for (int e = 0; e < device->msix.table_size; ++e) {
      SetMsixEntryMasked(table, e, /*masked=*/true);
    }
    (void)table[3];
    PciConfigUpdate16(device_index,                                         //
                      control,                                              //
                      static_cast<uint16_t>(old_control & kEnableAndMask),  //
                      static_cast<uint16_t>(~old_control & kEnableAndMask));
    if (g_env != nullptr) {
      g_env->UnmapMsixTable(device_index);
    }
  }
  if (device->msi.offset != 0 && device->msi.per_vector_masking) {
    const uint32_t mask_bits =
        PciConfigRead32(device_index, device->msi.mask_offset);
    PciConfigWrite32(device_index, device->msi.mask_offset,
                     mask_bits | MsiImplementedMaskBits(device->msi));
  }
}

void DeviceIrqResetDevice(const int device_index) {
  if (device_index < 0 || device_index >= kPciMaxDevices) {
    return;
  }
  const IrqSpinLockGuard guard(g_device_irq_lock);
  g_device_configured[device_index] = false;
  g_msix_tables[device_index] = nullptr;
}

DeviceIrqStatus DeviceIrqProgramIsaDevice(const int isa_device,
                                          const int vector) {
  CHECK(g_env != nullptr);
  if (isa_device < 0 || isa_device >= kIsaDeviceCount) {
    return DeviceIrqStatus::kNoDevice;
  }
  if (vector < kDeviceIrqVectorFirst || vector > kDeviceIrqVectorLast) {
    return DeviceIrqStatus::kInvalid;
  }
  const int isa_irq = IsaDeviceIrq(isa_device);
  if (isa_irq < 0 || !IoApicHasIsaIrq(isa_irq)) {
    return DeviceIrqStatus::kNoDevice;
  }
  const int destination = g_env->MsiDestinationApicId();
  if (destination < 0 ||
      !DeviceIrqMsiCanTarget(static_cast<uint32_t>(destination))) {
    return DeviceIrqStatus::kInvalid;
  }

  {
    const IrqSpinLockGuard guard(g_device_irq_lock);
    const VectorSlot& slot = g_slots[vector - kDeviceIrqVectorFirst];
    if (slot.owner == kPciNoOwner ||
        slot.device_index != DeviceIrqIsaSlotId(isa_device)) {
      return DeviceIrqStatus::kInvalid;
    }
    if (g_isa_device_configured[isa_device]) {
      return DeviceIrqStatus::kBusy;
    }
    g_isa_device_configured[isa_device] = true;
  }

  bool level_triggered = false;
  const IoApicStatus status =
      IoApicProgramIsaIrq(isa_irq,                            //
                          vector,                             //
                          static_cast<uint8_t>(destination),  //
                          &level_triggered);
  if (status != IoApicStatus::kOk) {
    const IrqSpinLockGuard guard(g_device_irq_lock);
    g_isa_device_configured[isa_device] = false;
    return status == IoApicStatus::kNoDevice ? DeviceIrqStatus::kNoDevice
                                             : DeviceIrqStatus::kInvalid;
  }

  if (level_triggered) {
    const IrqSpinLockGuard guard(g_device_irq_lock);
    VectorSlot& slot = g_slots[vector - kDeviceIrqVectorFirst];
    if (slot.owner != kPciNoOwner && slot.level_isa_irq < 0) {
      slot.level_isa_irq = isa_irq;
      g_level_isa_slot_count.fetch_add(1, std::memory_order_release);
    }
  }
  return DeviceIrqStatus::kOk;
}

void DeviceIrqReleaseIsaDevice(const int isa_device) {
  if (isa_device < 0 || isa_device >= kIsaDeviceCount) {
    return;
  }
  bool was_configured = false;
  {
    const IrqSpinLockGuard guard(g_device_irq_lock);
    was_configured = g_isa_device_configured[isa_device];
    g_isa_device_configured[isa_device] = false;
  }
  if (!was_configured) {
    return;
  }
  const int isa_irq = IsaDeviceIrq(isa_device);
  if (isa_irq >= 0) {
    IoApicSetIsaIrqMasked(isa_irq, true);
  }
}

void DeviceIrqResetIsaDevice(const int isa_device) {
  if (isa_device < 0 || isa_device >= kIsaDeviceCount) {
    return;
  }
  const IrqSpinLockGuard guard(g_device_irq_lock);
  g_isa_device_configured[isa_device] = false;
}

int DeviceIrqFreeVectors(const uintptr_t owner) {
  CHECK(owner != kPciNoOwner);
  const IrqSpinLockGuard guard(g_device_irq_lock);
  int freed = 0;
  for (int i = 0; i < kDeviceIrqVectorCount; ++i) {
    if (g_slots[i].owner == owner) {
      ClearVectorSlotLocked(i);
      if (g_env != nullptr) {
        g_env->UnregisterIdtHandler(kDeviceIrqVectorFirst + i);
      }
      ++freed;
    }
  }
  return freed;
}

int DeviceIrqFreeDeviceVectors(const uintptr_t owner, const int device_index) {
  CHECK(owner != kPciNoOwner);
  const IrqSpinLockGuard guard(g_device_irq_lock);
  int freed = 0;
  for (int i = 0; i < kDeviceIrqVectorCount; ++i) {
    if (g_slots[i].owner == owner && g_slots[i].device_index == device_index) {
      ClearVectorSlotLocked(i);
      if (g_env != nullptr) {
        g_env->UnregisterIdtHandler(kDeviceIrqVectorFirst + i);
      }
      ++freed;
    }
  }
  return freed;
}

int DeviceIrqUsedVectorCount() {
  const IrqSpinLockGuard guard(g_device_irq_lock);
  int used = 0;
  for (int i = 0; i < kDeviceIrqVectorCount; ++i) {
    if (g_slots[i].owner != kPciNoOwner) {
      ++used;
    }
  }
  return used;
}

uintptr_t DeviceIrqVectorOwner(const int vector) {
  if (vector < kDeviceIrqVectorFirst || vector > kDeviceIrqVectorLast) {
    return kPciNoOwner;
  }
  const IrqSpinLockGuard guard(g_device_irq_lock);
  return g_slots[vector - kDeviceIrqVectorFirst].owner;
}

int DeviceIrqVectorForBit(const uintptr_t owner, const int bit_index) {
  if (owner == kPciNoOwner || bit_index < 0 ||
      bit_index >= kSyscallIrqMaxCount) {
    return -1;
  }
  const uint64_t bit = uint64_t{1} << bit_index;
  const IrqSpinLockGuard guard(g_device_irq_lock);
  for (int i = 0; i < kDeviceIrqVectorCount; ++i) {
    if (g_slots[i].owner == owner && g_slots[i].bit == bit) {
      return kDeviceIrqVectorFirst + i;
    }
  }
  return -1;
}

uint32_t DeviceIrqReadMsixTableWord(const int device_index,
                                    const int word_index) {
  if (device_index < 0 || device_index >= kPciMaxDevices || word_index < 0) {
    return 0;
  }
  const PciDevice* const device = PciGetDevice(device_index);
  if (device == nullptr || word_index >= device->msix.table_size * 4) {
    return 0;
  }
  const IrqSpinLockGuard guard(g_device_irq_lock);
  volatile uint32_t* const table = g_msix_tables[device_index];
  return (table != nullptr) ? table[word_index] : 0;
}

bool DeviceIrqDispatch(const int vector) {
  if (vector < kDeviceIrqVectorFirst || vector > kDeviceIrqVectorLast) {
    return false;
  }
  const IrqSpinLockGuard guard(g_device_irq_lock);
  const VectorSlot& slot = g_slots[vector - kDeviceIrqVectorFirst];
  if (slot.owner == kPciNoOwner || slot.pending == nullptr) {
    return false;
  }
  if (slot.level_isa_irq >= 0) {
    IoApicSetIsaIrqMasked(slot.level_isa_irq, true);
  }
  slot.pending->fetch_or(slot.bit, std::memory_order_release);
  if (slot.task != nullptr && g_env != nullptr) {
    g_env->Wake(slot.task);
  }
  return true;
}

int64_t DeviceIrqWait(std::atomic<uint64_t>* const pending,       //
                      const uint64_t mask,                        //
                      const int64_t timeout_ticks,                //
                      bool (*const should_stop)(void* stop_ctx),  //
                      void* const stop_ctx) {
  CHECK(g_env != nullptr);
  CHECK(pending != nullptr);
  if ((mask >> kSyscallIrqMaxCount) != 0 ||
      timeout_ticks > kSyscallIrqWaitMaxTicks) {
    return kSyscallErrorInvalid;
  }

  const uint64_t immediate = TryConsumePending(pending, mask);
  if (immediate != 0) {
    return static_cast<int64_t>(immediate);
  }
  if (mask != 0 && timeout_ticks == 0) {
    UnmaskWatchedLevelIrqs(pending, mask);
    return static_cast<int64_t>(TryConsumePending(pending, mask));
  }

  const bool has_deadline = timeout_ticks > 0;
  const int64_t deadline =
      has_deadline ? (g_env->NowTicks() + timeout_ticks) : 0;

  for (;;) {
    const uint64_t fired = TryConsumePending(pending, mask);
    if (fired != 0) {
      return static_cast<int64_t>(fired);
    }
    if (should_stop != nullptr && should_stop(stop_ctx)) {
      return 0;
    }
    if (has_deadline && g_env->NowTicks() >= deadline) {
      return static_cast<int64_t>(TryConsumePending(pending, mask));
    }
    UnmaskWatchedLevelIrqs(pending, mask);
    if ((pending->load(std::memory_order_acquire) & mask) != 0) {
      continue;
    }
    if (has_deadline) {
      g_env->BlockUntil(deadline);
    } else {
      g_env->Block();
    }
  }
}

#if __STDC_HOSTED__
void DeviceIrqResetForTest() {
  {
    const IrqSpinLockGuard guard(g_device_irq_lock);
    for (int i = 0; i < kDeviceIrqVectorCount; ++i) {
      if (g_slots[i].owner != kPciNoOwner && g_env != nullptr) {
        g_env->UnregisterIdtHandler(kDeviceIrqVectorFirst + i);
      }
      g_slots[i] = VectorSlot{};
    }
    g_next_slot = 0;
    g_level_isa_slot_count.store(0, std::memory_order_release);
    for (int i = 0; i < kPciMaxDevices; ++i) {
      g_device_configured[i] = false;
      g_msix_tables[i] = nullptr;
    }
    for (int i = 0; i < kIsaDeviceCount; ++i) {
      g_isa_device_configured[i] = false;
    }
  }
  g_env.reset();
}
#endif

#if !__STDC_HOSTED__
namespace {

// Reserved kernel virtual window (`0x00007D0000000000`, 64 pages = 256 KiB)
// used to map MSI-X tables uncacheable (`PagingMapKernelMmioPage`) while a
// device has MSI-X configured. Page tables for the window are allocated once at
// boot by `PagingReserveKernelRange`, so mapping and unmapping pages inside it
// never allocates or frees PMM frames.
constexpr uintptr_t kDeviceIrqMmioWindowBase = 0x00007D0000000000ULL;
constexpr int kDeviceIrqMmioWindowPages = 64;

struct KernelMsixMapping {
  uintptr_t virt_base = 0;
  int first_window_page = 0;
  int page_count = 0;
  volatile uint32_t* table = nullptr;
};

IrqSpinLock g_kernel_msix_map_lock;
bool g_window_page_used[kDeviceIrqMmioWindowPages] = {};
KernelMsixMapping g_kernel_msix_mappings[kPciMaxDevices] = {};

// Marks window pages `first .. first + count - 1` free once no CPU can still
// hold a translation of them: after a TLB shootdown that starts after they
// were unmapped (`PagingUnmapKernelPage` in paging.h). Until then they stay
// marked used, so that `MapMsixTable` can't map them to another table yet.
// Called without `g_kernel_msix_map_lock`: a shootdown must not run under an
// `IrqSpinLock`.
static void FreeWindowPagesAfterShootdown(const int first, const int count) {
  SmpTlbShootdown();
  const IrqSpinLockGuard guard(g_kernel_msix_map_lock);
  for (int j = 0; j < count; ++j) {
    g_window_page_used[first + j] = false;
  }
}

class KernelDeviceIrqEnvironment final : public DeviceIrqEnvironment {
 public:
  KernelDeviceIrqEnvironment() {
    CHECK(PagingReserveKernelRange(
        kDeviceIrqMmioWindowBase,
        static_cast<int64_t>(kDeviceIrqMmioWindowPages) * kPageSize));
  }

  void RegisterIdtHandler(const int vector,
                          const InterruptHandlerFn handler) override {
    IdtRegisterHandler(vector, handler);
  }

  void UnregisterIdtHandler(const int vector) override {
    IdtUnregisterHandler(vector);
  }

  int MsiDestinationApicId() override {
    // CPU 0 is the BSP (smp.h).
    const int cpu_count = SmpCpuCount();
    for (int cpu = 0; cpu < cpu_count; ++cpu) {
      const uint32_t apic_id = SmpGetCpuInfo(cpu)->apic_id;
      if (IsCpuOnline(cpu, cpu_count) && DeviceIrqMsiCanTarget(apic_id)) {
        return static_cast<int>(apic_id);
      }
    }
    return -1;
  }

  volatile uint32_t* MapMsixTable(const int device_index,
                                  const PciMsix& msix) override {
    if (device_index < 0 || device_index >= kPciMaxDevices || !msix.valid ||
        msix.table_phys == 0 || msix.table_bytes <= 0) {
      return nullptr;
    }
    constexpr uint64_t kPageMask = static_cast<uint64_t>(kPageSize) - 1;
    const uint64_t first_phys = msix.table_phys & ~kPageMask;
    const uint64_t last_phys =
        (msix.table_phys + static_cast<uint64_t>(msix.table_bytes) - 1) &
        ~kPageMask;
    const int pages =
        static_cast<int>((last_phys - first_phys) / kPageSize) + 1;
    if (pages <= 0 || pages > kDeviceIrqMmioWindowPages) {
      return nullptr;
    }

    int start_page = -1;
    {
      const IrqSpinLockGuard guard(g_kernel_msix_map_lock);
      if (g_kernel_msix_mappings[device_index].table != nullptr) {
        return g_kernel_msix_mappings[device_index].table;
      }
      for (int i = 0; i <= kDeviceIrqMmioWindowPages - pages; ++i) {
        bool ok = true;
        for (int j = 0; j < pages; ++j) {
          if (g_window_page_used[i + j]) {
            ok = false;
            break;
          }
        }
        if (ok) {
          start_page = i;
          break;
        }
      }
      if (start_page < 0) {
        return nullptr;
      }
      // Reserved before they are mapped: if a mapping fails, the pages mapped
      // so far may only be reused after a TLB shootdown.
      for (int j = 0; j < pages; ++j) {
        g_window_page_used[start_page + j] = true;
      }

      const uintptr_t virt_base =
          kDeviceIrqMmioWindowBase +
          static_cast<uintptr_t>(start_page) * kPageSize;
      int mapped = 0;
      while (mapped < pages &&
             PagingMapKernelMmioPage(
                 virt_base + static_cast<uintptr_t>(mapped) * kPageSize,
                 static_cast<uintptr_t>(first_phys) +
                     static_cast<uintptr_t>(mapped) * kPageSize)) {
        ++mapped;
      }
      if (mapped == pages) {
        const uintptr_t offset_in_page =
            static_cast<uintptr_t>(msix.table_phys - first_phys);
        volatile uint32_t* const table =
            reinterpret_cast<volatile uint32_t*>(virt_base + offset_in_page);
        g_kernel_msix_mappings[device_index] = KernelMsixMapping{
            .virt_base = virt_base,
            .first_window_page = start_page,
            .page_count = pages,
            .table = table,
        };
        return table;
      }
      for (int k = 0; k < mapped; ++k) {
        PagingUnmapKernelPage(virt_base +
                              static_cast<uintptr_t>(k) * kPageSize);
      }
    }
    FreeWindowPagesAfterShootdown(start_page, pages);
    return nullptr;
  }

  void UnmapMsixTable(const int device_index) override {
    if (device_index < 0 || device_index >= kPciMaxDevices) {
      return;
    }
    KernelMsixMapping mapping{};
    {
      const IrqSpinLockGuard guard(g_kernel_msix_map_lock);
      mapping = g_kernel_msix_mappings[device_index];
      if (mapping.table == nullptr) {
        return;
      }
      for (int j = 0; j < mapping.page_count; ++j) {
        PagingUnmapKernelPage(mapping.virt_base +
                              static_cast<uintptr_t>(j) * kPageSize);
      }
      g_kernel_msix_mappings[device_index] = KernelMsixMapping{};
    }
    FreeWindowPagesAfterShootdown(mapping.first_window_page,
                                  mapping.page_count);
  }

  int64_t NowTicks() override {
    const CpuLocal* const cpu0 = SmpGetCpuLocal(0);
    return cpu0 != nullptr ? cpu0->timer_ticks.load(std::memory_order_acquire)
                           : 0;
  }

  void Block() override {
    TaskScheduler* const scheduler = GetTaskScheduler();
    CHECK(scheduler != nullptr);
    scheduler->Block();
  }

  void BlockUntil(const int64_t deadline_tick) override {
    TaskScheduler* const scheduler = GetTaskScheduler();
    CHECK(scheduler != nullptr);
    scheduler->BlockUntil(deadline_tick);
  }

  void Wake(Task* const task) override {
    TaskScheduler* const scheduler = GetTaskScheduler();
    if (scheduler != nullptr && task != nullptr) {
      scheduler->Wake(task);
    }
  }
};

}  // namespace

std::shared_ptr<DeviceIrqEnvironment> CreateKernelDeviceIrqEnvironment() {
  return std::make_shared<KernelDeviceIrqEnvironment>();
}
#endif

}  // namespace protos
