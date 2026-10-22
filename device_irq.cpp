#include "device_irq.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <utility>

#include "check.h"
#include "idt.h"
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
  std::atomic<uint64_t>* pending = nullptr;
  uint64_t bit = 0;
  Task* task = nullptr;
};

// Guards `g_slots`, `g_next_slot`, `g_device_configured`, and `g_msix_tables`.
// Leaf lock with respect to PCI and everything except the scheduler runqueue
// lock taken inside `DeviceIrqEnvironment::Wake()`.
IrqSpinLock g_device_irq_lock;
std::shared_ptr<DeviceIrqEnvironment> g_env;
VectorSlot g_slots[kDeviceIrqVectorCount] = {};
int g_next_slot = 0;
bool g_device_configured[kPciMaxDevices] = {};
volatile uint32_t* g_msix_tables[kPciMaxDevices] = {};

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

}  // namespace

void DeviceIrqInit(std::shared_ptr<DeviceIrqEnvironment> env) {
  CHECK(env != nullptr);
  const IrqSpinLockGuard guard(g_device_irq_lock);
  for (int i = 0; i < kDeviceIrqVectorCount; ++i) {
    CHECK(g_slots[i].owner == kPciNoOwner);
  }
  g_env = std::move(env);
  g_next_slot = 0;
  for (int i = 0; i < kPciMaxDevices; ++i) {
    g_device_configured[i] = false;
    g_msix_tables[i] = nullptr;
  }
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

  {
    const IrqSpinLockGuard guard(g_device_irq_lock);
    if (g_device_configured[device_index]) {
      return DeviceIrqStatus::kBusy;
    }
    g_device_configured[device_index] = true;
  }

  const uint32_t addr_lo = DeviceIrqMsiAddressLow(g_env->BspApicId());
  if (effective_mode == kSyscallIrqModeMsi) {
    const PciMsi& msi = device->msi;
    // 1. Disable MSI and clear Multiple Message Enable before programming.
    PciConfigUpdate16(device_index, msi.offset + kPciMsiControlOffset, 0,
                      kPciMsiControlEnable | kMsiControlMmeMask);
    // 2. Write Message Address (and Upper Address = 0 if 64-bit).
    PciConfigWrite32(device_index, msi.offset + 4, addr_lo);
    if (msi.is_64bit) {
      PciConfigWrite32(device_index, msi.offset + 8, 0);
    }
    // 3. Write Message Data.
    PciConfigWrite16(device_index, msi.data_offset,
                     DeviceIrqMsiData(vectors[0]));
    // 4. If per-vector masking is present, unmask vector 0 and mask bits 1..31.
    if (msi.per_vector_masking) {
      PciConfigWrite32(device_index, msi.mask_offset, ~uint32_t{1});
    }
    // 5. Disable legacy INTx and enable MSI with MME = 0 (1 vector).
    PciUpdateCommand(device_index, kPciCommandInterruptDisable, 0);
    PciConfigUpdate16(device_index, msi.offset + kPciMsiControlOffset,
                      kPciMsiControlEnable, kMsiControlMmeMask);
    return DeviceIrqStatus::kOk;
  }

  const PciMsix& msix = device->msix;
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

  // 1. Set Function Mask = 1 and Enable = 0 while programming the table.
  PciConfigUpdate16(device_index, msix.offset + kPciMsiControlOffset,
                    kPciMsixControlFunctionMask, kPciMsixControlEnable);
  // 2. Ensure BAR Memory Space decoding is enabled and legacy INTx is disabled.
  PciUpdateCommand(device_index,
                   kPciCommandMemorySpace | kPciCommandInterruptDisable, 0);
  // 3. Mask all MSI-X table entries.
  for (int e = 0; e < msix.table_size; ++e) {
    table[e * 4 + 3] = kMsixVectorControlMaskBit;
  }
  // 4. Program entries 0..count - 1 and unmask them.
  for (int i = 0; i < count; ++i) {
    table[i * 4 + 0] = addr_lo;
    table[i * 4 + 1] = 0;
    table[i * 4 + 2] = DeviceIrqMsiData(vectors[i]);
    table[i * 4 + 3] = 0;
  }
  // 5. Read back entry 0's Vector Control word to flush posted MMIO writes.
  (void)table[3];
  // 6. Enable MSI-X and clear Function Mask.
  PciConfigUpdate16(device_index, msix.offset + kPciMsiControlOffset,
                    kPciMsixControlEnable, kPciMsixControlFunctionMask);
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
    for (int e = 0; e < device->msix.table_size; ++e) {
      table[e * 4 + 3] = kMsixVectorControlMaskBit;
    }
    (void)table[3];
    if (g_env != nullptr) {
      g_env->UnmapMsixTable(device_index);
    }
  }
  if (device->msi.offset != 0 && device->msi.per_vector_masking) {
    PciConfigWrite32(device_index, device->msi.mask_offset, 0xFFFFFFFFU);
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

int DeviceIrqFreeVectors(const uintptr_t owner) {
  CHECK(owner != kPciNoOwner);
  const IrqSpinLockGuard guard(g_device_irq_lock);
  int freed = 0;
  for (int i = 0; i < kDeviceIrqVectorCount; ++i) {
    if (g_slots[i].owner == owner) {
      g_slots[i] = VectorSlot{};
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
      g_slots[i] = VectorSlot{};
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
    return 0;
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
    for (int i = 0; i < kPciMaxDevices; ++i) {
      g_device_configured[i] = false;
      g_msix_tables[i] = nullptr;
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

  uint8_t BspApicId() override {
    const CpuInfo* const bsp = SmpGetCpuInfo(0);
    return bsp != nullptr ? static_cast<uint8_t>(bsp->apic_id & 0xFFU) : 0;
  }

  volatile uint32_t* MapMsixTable(const int device_index,
                                  const PciMsix& msix) override {
    if (device_index < 0 || device_index >= kPciMaxDevices || !msix.valid ||
        msix.table_phys == 0 || msix.table_bytes <= 0) {
      return nullptr;
    }
    const IrqSpinLockGuard guard(g_kernel_msix_map_lock);
    if (g_kernel_msix_mappings[device_index].table != nullptr) {
      return g_kernel_msix_mappings[device_index].table;
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

    const uintptr_t virt_base = kDeviceIrqMmioWindowBase +
                                static_cast<uintptr_t>(start_page) * kPageSize;
    for (int j = 0; j < pages; ++j) {
      const uintptr_t va = virt_base + static_cast<uintptr_t>(j) * kPageSize;
      const uintptr_t pa = static_cast<uintptr_t>(first_phys) +
                           static_cast<uintptr_t>(j) * kPageSize;
      if (!PagingMapKernelMmioPage(va, pa)) {
        for (int k = 0; k < j; ++k) {
          PagingUnmapKernelPage(virt_base +
                                static_cast<uintptr_t>(k) * kPageSize);
        }
        return nullptr;
      }
    }
    for (int j = 0; j < pages; ++j) {
      g_window_page_used[start_page + j] = true;
    }
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
        g_window_page_used[mapping.first_window_page + j] = false;
      }
      g_kernel_msix_mappings[device_index] = KernelMsixMapping{};
    }
    // Flush TLBs across all CPUs before those virtual pages can be reused.
    SmpTlbShootdown();
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
