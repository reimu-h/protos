// nvme: ring-3 NVM Express PCIe block device driver (PCI class 01/08/02).
//
// Discovers and claims all PCI NVMe controllers, performs two-phase bring-up
// (Phase 1: BAR0 MMIO map + CC=0 reset before touching endpoint Bus Master;
// Phase 2: Bus Master on + MSI-X/MSI interrupt + queue enable + namespace
// discovery), claims `kIpcPortBlock`, and serves `BlockRpcOp` requests
// (`block_protocol.h`).
//
// Started without `kUserCapDriver` (e.g. `run nvme` in `sh`), exits with
// `kDriverExitNotDriver`. When no NVMe controllers or usable namespaces are
// present, reports that and exits cleanly with status 0 in `kDriverArgNormal`.

#include <cstdint>
#include <type_traits>

#include "syscall_abi.h"
#include "user/nvme/nvme_controller.h"
#include "user/nvme/nvme_platform.h"
#include "user/nvme/nvme_server.h"
#include "user/syscall.h"

namespace protos::user {
namespace {

constexpr int kMaxControllers = kSyscallMaxClaims;
constexpr int64_t kPageSize = 4096;

class Ring3NvmePlatform final : public nvme::Platform {
 public:
  constexpr Ring3NvmePlatform() = default;

  nvme::DmaBuffer DmaAlloc(const int64_t size, const bool below_4gib) override {
    const int64_t aligned_size =
        nvme::ClampAndAlignDmaSize(size, kSyscallDmaMaxSize);
    if (aligned_size <= 0) {
      return {};
    }
    const uint64_t flags = below_4gib ? kSyscallDmaBelow4GiB : 0;
    const StatusOr<DmaAllocation> dma =
        ::protos::user::DmaAlloc(aligned_size, flags);
    if (!dma.ok() || dma->va == 0 || dma->phys == 0 ||
        (dma->va & (kPageSize - 1)) != 0 ||
        (dma->phys & (kPageSize - 1)) != 0) {
      return {};
    }
    return nvme::DmaBuffer{
        .va = reinterpret_cast<void*>(dma->va),
        .phys = dma->phys,
        .size = aligned_size,
    };
  }

  int64_t NowTicks() override { return GetTicks(); }

  int64_t TicksPerSecond() override { return kSyscallTicksPerSecond; }

  void Relax() override { (void)IrqWait(/*mask=*/0, /*timeout_ticks=*/1); }

  void WaitForIrq(const uint64_t irq_mask,
                  const int64_t timeout_ticks) override {
    if (timeout_ticks <= 0) {
      return;
    }
    const int64_t clamped = (timeout_ticks > kSyscallIrqWaitMaxTicks)
                                ? kSyscallIrqWaitMaxTicks
                                : timeout_ticks;
    (void)IrqWait(irq_mask, clamped);
  }

  void Log(const char* const line, const int length) override {
    if (line != nullptr && length > 0) {
      (void)Write(line, length);
    }
  }
};

struct ControllerSlot {
  int pci_index = -1;
  uint64_t irq_bit_mask = 0;
  nvme::IrqMode irq_mode = nvme::IrqMode::kMsix;
  bool phase1_ok = false;
  bool active = false;
  char label[8] = {};
};

constinit Ring3NvmePlatform g_platform{};
constinit nvme::MappedMmioWindow g_mmio[kMaxControllers]{};
constinit nvme::NvmeController g_controllers[kMaxControllers]{};
constinit ControllerSlot g_slots[kMaxControllers]{};
constinit nvme::BlockServer g_server{};
alignas(8) constinit uint8_t g_req_buf[kIpcMaxMessageSize]{};
alignas(8) constinit uint8_t g_reply_buf[kIpcMaxMessageSize]{};
static_assert(std::is_trivially_destructible_v<decltype(g_platform)>);
static_assert(std::is_trivially_destructible_v<decltype(g_mmio)>);
static_assert(std::is_trivially_destructible_v<decltype(g_controllers)>);
static_assert(std::is_trivially_destructible_v<decltype(g_slots)>);
static_assert(std::is_trivially_destructible_v<decltype(g_server)>);
static_assert(std::is_trivially_destructible_v<decltype(g_req_buf)>);
static_assert(std::is_trivially_destructible_v<decltype(g_reply_buf)>);

static StatusOr<nvme::MappedMmioWindow> MapControllerBar0(
    const PciBarInfo& bar0) {
  const nvme::MmioMapEnv map_env{
      .ctx = nullptr,
      .map = [](void* const /*ctx*/,       //
                const uint64_t phys_addr,  //
                const int64_t size,        //
                uintptr_t* const out_va) -> int64_t {
        const StatusOr<uintptr_t> va = MmioMap(phys_addr, size);
        if (!va.ok()) {
          return va.status();
        }
        *out_va = *va;
        return 0;
      },
  };
  nvme::MappedMmioWindow window{};
  const int64_t rc = nvme::MapNvmeBar0(bar0, map_env, &window);
  if (rc != 0) {
    return rc;
  }
  return window;
}

struct ControllerIrq {
  uint64_t bit_mask = 0;
  nvme::IrqMode mode = nvme::IrqMode::kMsix;
};

static StatusOr<ControllerIrq> AllocateControllerIrq(
    const int pci_index, const PciDeviceInfo& info) {
  if (info.msix_vectors > 0) {
    const int64_t bit = IrqAlloc(pci_index, /*count=*/1, kSyscallIrqModeMsix);
    if (!IsError(bit) && bit >= 0 && bit < kSyscallIrqMaxCount) {
      return ControllerIrq{
          .bit_mask = uint64_t{1} << static_cast<uint32_t>(bit),
          .mode = nvme::IrqMode::kMsix,
      };
    }
  }
  if (info.msi_vectors > 0) {
    const int64_t bit = IrqAlloc(pci_index, /*count=*/1, kSyscallIrqModeMsi);
    if (!IsError(bit) && bit >= 0 && bit < kSyscallIrqMaxCount) {
      return ControllerIrq{
          .bit_mask = uint64_t{1} << static_cast<uint32_t>(bit),
          .mode = nvme::IrqMode::kMsi,
      };
    }
  }
  return kStatusError;
}

static void LogBringUpError(const char* const label, const char* const step) {
  nvme::LogLine line;
  line.Str(nvme::kLogPrefix)
      .Str(label)
      .Str(": error: ")
      .Str(step)
      .Emit(g_platform);
}

static void ShutdownActiveControllers(const int discovered_count) {
  for (int i = 0; i < discovered_count; ++i) {
    if (g_slots[i].active) {
      (void)g_controllers[i].Shutdown();
      (void)PciBusMaster(g_slots[i].pci_index, /*enable=*/0);
      g_slots[i].active = false;
    }
  }
}

}  // namespace

int64_t Main(const uint64_t arg) {
  const int64_t pci_count = GetPciDeviceCount();
  if (pci_count == kSyscallErrorPermission) {
    nvme::LogLine line;
    line.Str(nvme::kLogPrefix)
        .Str("not started as a driver (init starts nvme at boot)")
        .Emit(g_platform);
    return kDriverExitNotDriver;
  }
  if (arg != kDriverArgNormal && arg != kDriverArgExitAfterReady &&
      arg != kDriverArgFaultAfterReady) {
    nvme::LogLine line;
    line.Str(nvme::kLogPrefix).Str("error: invalid mode").Emit(g_platform);
    return 1;
  }
  if (pci_count <= 0) {
    nvme::LogLine line;
    line.Str(nvme::kLogPrefix)
        .Str("no NVMe controllers found")
        .Emit(g_platform);
    return (arg == kDriverArgNormal) ? 0 : 1;
  }

  int discovered_indices[kMaxControllers] = {};
  int discovered_count = 0;
  bool limit_warned = false;
  for (int64_t idx = 0; idx < pci_count; ++idx) {
    const StatusOr<PciDeviceInfo> info = GetPciInfo(idx);
    if (!info.ok() || !nvme::IsNvmePciDevice(*info)) {
      continue;
    }
    if (discovered_count >= kMaxControllers) {
      if (!limit_warned) {
        nvme::LogLine warn;
        warn.Str(nvme::kLogPrefix)
            .Str("controller limit reached, ignoring extra controllers")
            .Emit(g_platform);
        limit_warned = true;
      }
      break;
    }
    const int slot_idx = discovered_count++;
    discovered_indices[slot_idx] = static_cast<int>(idx);
    g_slots[slot_idx] = ControllerSlot{};
    g_slots[slot_idx].pci_index = static_cast<int>(idx);
    nvme::FormatBdf(info->bus,       //
                    info->device,    //
                    info->function,  //
                    g_slots[slot_idx].label);
  }

  if (discovered_count == 0) {
    nvme::LogLine line;
    line.Str(nvme::kLogPrefix)
        .Str("no NVMe controllers found")
        .Emit(g_platform);
    return (arg == kDriverArgNormal) ? 0 : 1;
  }

  // Claim `kIpcPortBlock` before controller bring-up (except in test modes that
  // exit or fault right after ready) so early callers queue cleanly rather than
  // racing bring-up.
  const bool serve_ipc = (arg == kDriverArgNormal);
  if (serve_ipc) {
    const int64_t listen_rc = IpcListen(kIpcPortBlock);
    if (listen_rc != 0) {
      nvme::LogLine line;
      line.Str(nvme::kLogPrefix).Str("error: listen").Emit(g_platform);
      return 1;
    }
  }

  // Phase 1: Claim device, map BAR0, disable controller (`CC = 0`) before
  // touching endpoint Bus Master (disabling Bus Master on error).
  int decoding_count = 0;
  for (int slot_idx = 0; slot_idx < discovered_count; ++slot_idx) {
    const int pci_idx = discovered_indices[slot_idx];
    ControllerSlot& slot = g_slots[slot_idx];

    if (PciClaim(pci_idx) != 0) {
      LogBringUpError(slot.label, "claim");
      continue;
    }
    const StatusOr<PciDeviceInfo> info = GetPciInfo(pci_idx);
    if (!info.ok() || !nvme::IsNvmePciDevice(*info)) {
      (void)PciBusMaster(pci_idx, /*enable=*/0);
      LogBringUpError(slot.label, "pci_info");
      continue;
    }
    nvme::FormatBdf(info->bus, info->device, info->function, slot.label);

    const StatusOr<nvme::MappedMmioWindow> mmio =
        MapControllerBar0(info->bars[0]);
    if (!mmio.ok()) {
      (void)PciBusMaster(pci_idx, /*enable=*/0);
      LogBringUpError(slot.label, "mmio_map");
      continue;
    }
    g_mmio[slot_idx] = *mmio;

    g_controllers[slot_idx].Init(g_platform,        //
                                 g_mmio[slot_idx],  //
                                 info->bus,         //
                                 info->device,      //
                                 info->function);
    const nvme::NvmeStatus phase1 =
        g_controllers[slot_idx].TakeOwnershipAndReset();
    if (phase1 != nvme::NvmeStatus::kNotDecoding) {
      ++decoding_count;
    }
    if (phase1 != nvme::NvmeStatus::kOk) {
      (void)PciBusMaster(pci_idx, /*enable=*/0);
      LogBringUpError(slot.label, "reset");
      continue;
    }
    slot.phase1_ok = true;
  }

  // Phase 2: Enable Bus Master, allocate MSI-X/MSI, bring up Admin & I/O
  // queues.
  g_server.Reset();
  int started_count = 0;
  for (int i = 0; i < discovered_count; ++i) {
    ControllerSlot& slot = g_slots[i];
    if (!slot.phase1_ok) {
      continue;
    }
    const StatusOr<PciDeviceInfo> info = GetPciInfo(slot.pci_index);
    if (!info.ok()) {
      (void)PciBusMaster(slot.pci_index, /*enable=*/0);
      LogBringUpError(slot.label, "pci_info");
      continue;
    }
    if (PciBusMaster(slot.pci_index, /*enable=*/1) != 0) {
      LogBringUpError(slot.label, "bus_master");
      continue;
    }
    const StatusOr<ControllerIrq> irq =
        AllocateControllerIrq(slot.pci_index, *info);
    if (!irq.ok()) {
      (void)PciBusMaster(slot.pci_index, /*enable=*/0);
      LogBringUpError(slot.label, "irq_alloc");
      continue;
    }
    slot.irq_bit_mask = irq->bit_mask;
    slot.irq_mode = irq->mode;

    if (g_controllers[i].Start(slot.irq_bit_mask, slot.irq_mode) !=
        nvme::NvmeStatus::kOk) {
      (void)g_controllers[i].Shutdown();
      (void)PciBusMaster(slot.pci_index, /*enable=*/0);
      LogBringUpError(slot.label, "start");
      continue;
    }
    ++started_count;

    const int disks_before = g_server.disk_count();
    const bool added_all = g_server.AddController(&g_controllers[i]);
    const int disks_added = g_server.disk_count() - disks_before;
    if (!added_all) {
      nvme::LogLine warn;
      warn.Str(nvme::kLogPrefix)
          .Str(slot.label)
          .Str(": disk limit reached, registered ")
          .Dec(static_cast<uint64_t>(disks_added))
          .Str("/")
          .Dec(static_cast<uint64_t>(g_controllers[i].namespace_count()))
          .Emit(g_platform);
    }
    if (arg == kDriverArgNormal && !added_all && disks_added == 0) {
      (void)g_controllers[i].Shutdown();
      (void)PciBusMaster(slot.pci_index, /*enable=*/0);
      continue;
    }
    slot.active = true;

    nvme::LogLine line;
    line.Str(nvme::kLogPrefix)
        .Str("controller ")
        .Str(slot.label)
        .Str(" ready (")
        .Str(slot.irq_mode == nvme::IrqMode::kMsix ? "msix" : "msi")
        .Str("), disks=")
        .Dec(static_cast<uint64_t>(disks_added))
        .Emit(g_platform);
  }

  if (started_count == 0) {
    return 1;
  }
  if (arg != kDriverArgNormal && started_count != decoding_count) {
    ShutdownActiveControllers(discovered_count);
    return 2;
  }

  if (arg == kDriverArgFaultAfterReady) {
    asm volatile("ud2");
    return 0;
  }
  if (arg == kDriverArgExitAfterReady) {
    ShutdownActiveControllers(discovered_count);
    return 0;
  }

  if (g_server.disk_count() == 0) {
    ShutdownActiveControllers(discovered_count);
    nvme::LogLine empty_line;
    empty_line.Str(nvme::kLogPrefix)
        .Str("no usable namespaces")
        .Emit(g_platform);
    return 0;
  }

  nvme::LogLine ready_line;
  ready_line.Str(nvme::kLogPrefix).Str("ready").Emit(g_platform);

  const nvme::ServeEnv serve_env{
      .ctx = nullptr,
      .recv =
          [](void* const /*ctx*/,            //
             const int64_t port,             //
             void* const buf,                //
             const int64_t cap,              //
             IpcReceivedMessage* const msg,  //
             const int64_t timeout_ticks) {
            return IpcRecv(port, buf, cap, msg, timeout_ticks);
          },
      .reply =
          [](void* const /*ctx*/,    //
             const int64_t port,     //
             const uint64_t token,   //
             const void* const buf,  //
             const int64_t size,     //
             const int64_t status) {
            return IpcReply(port, token, buf, size, status);
          },
  };
  while (nvme::ServeOne(g_server,           //
                        serve_env,          //
                        g_req_buf,          //
                        sizeof(g_req_buf),  //
                        g_reply_buf,        //
                        sizeof(g_reply_buf))) {
    for (int i = 0; i < discovered_count; ++i) {
      if (g_slots[i].active &&
          g_controllers[i].state() != nvme::NvmeState::kRunning) {
        (void)g_controllers[i].Shutdown();
        (void)PciBusMaster(g_slots[i].pci_index, /*enable=*/0);
        g_slots[i].active = false;
      }
    }
    if (g_server.RunningDiskCount() == 0) {
      break;
    }
  }
  ShutdownActiveControllers(discovered_count);
  return 1;
}

}  // namespace protos::user
