#ifndef PROTOS_USER_NVME_NVME_SERVER_H_
#define PROTOS_USER_NVME_NVME_SERVER_H_

#include <cstdint>
#include <type_traits>

#include "block_protocol.h"
#include "syscall_abi.h"
#include "user/nvme/nvme_controller.h"

namespace protos::nvme {

static_assert(kBlockMaxIoBytes <= kIpcMaxMessageSize);
static_assert(static_cast<int64_t>(sizeof(BlockWriteRequest)) +
                  kBlockMaxIoBytes <=
              kIpcMaxMessageSize);

// Pure PCI NVMe device predicate shared by the ring-3 driver and unit tests.
inline bool IsNvmePciDevice(const PciDeviceInfo& info) {
  constexpr uint8_t kPciClassMassStorage = 0x01;
  constexpr uint8_t kPciSubclassNvm = 0x08;
  constexpr uint8_t kPciProgIfNvme = 0x02;
  constexpr int64_t kNvmePageSize = 4096;
  if (info.claimable == 0 || info.class_code != kPciClassMassStorage ||
      info.subclass != kPciSubclassNvm || info.prog_if != kPciProgIfNvme) {
    return false;
  }
  const PciBarInfo& bar0 = info.bars[0];
  const bool is_mem_bar = (bar0.type == PciBarInfoType::kMemory32 ||
                           bar0.type == PciBarInfoType::kMemory64);
  return is_mem_bar && bar0.phys_base != 0 &&
         (bar0.phys_base & static_cast<uint64_t>(kNvmePageSize - 1)) == 0 &&
         bar0.size >= 2 * kNvmePageSize;
}

// MMIO mapping seam for `MapNvmeBar0()`, allowing both the ring-3 driver
// (`MmioMap`) and host unit tests to exercise the exact 2-step `CAP.DSTRD` +
// binary-halving mapping algorithm.
struct MmioMapEnv {
  void* ctx = nullptr;
  int64_t (*map)(void* ctx,           //
                 uint64_t phys_addr,  //
                 int64_t size,        //
                 uintptr_t* out_va) = nullptr;
};

// Maps controller registers (page 0) and the required doorbell span (`[0x1000,
// PageAlignUp(CqHeadDoorbellOffset(1, CAP.DSTRD) + 4))`) using binary halving
// around unmappable MSI-X Table/PBA holes. Returns 0 on success or a negative
// `kSyscallError*` status.
inline int64_t MapNvmeBar0(const PciBarInfo& bar0,              //
                           const MmioMapEnv& env,               //
                           MappedMmioWindow* const out_window,  //
                           const int64_t max_mmio_size = kSyscallMmioMaxSize) {
  if (env.map == nullptr || out_window == nullptr) {
    return kSyscallErrorInvalid;
  }
  out_window->Init(nullptr, 0);
  constexpr int64_t kNvmePageSize = 4096;
  const bool is_mem_bar = (bar0.type == PciBarInfoType::kMemory32 ||
                           bar0.type == PciBarInfoType::kMemory64);
  if (!is_mem_bar || bar0.phys_base == 0 ||
      (bar0.phys_base & static_cast<uint64_t>(kNvmePageSize - 1)) != 0 ||
      bar0.size < 2 * kNvmePageSize || max_mmio_size < 2 * kNvmePageSize) {
    return kSyscallErrorInvalid;
  }

  int64_t bar0_limit = bar0.size;
  if (bar0_limit > max_mmio_size) {
    bar0_limit = max_mmio_size;
  }
  const uint64_t max_phys_span = UINT64_MAX - bar0.phys_base;
  if (static_cast<uint64_t>(bar0_limit) > max_phys_span) {
    bar0_limit = static_cast<int64_t>(max_phys_span);
  }
  bar0_limit &= ~(kNvmePageSize - 1);
  if (bar0_limit < 2 * kNvmePageSize) {
    return kSyscallErrorInvalid;
  }

  // Step 1: Map page 0 to inspect `CAP.DSTRD`.
  uintptr_t va0 = 0;
  const int64_t page0_rc =
      env.map(env.ctx, bar0.phys_base, kNvmePageSize, &va0);
  if (page0_rc != 0) {
    return page0_rc;
  }
  if (!out_window->AddSegment(0,                                         //
                              reinterpret_cast<volatile uint8_t*>(va0),  //
                              kNvmePageSize)) {
    return kSyscallErrorInvalid;
  }

  const uint32_t cap_lo = out_window->Read32(kRegCap);
  const uint32_t cap_hi = out_window->Read32(kRegCap + 4);
  if (cap_lo == kMmioAllOnes || cap_hi == kMmioAllOnes) {
    // Return the page-0 window so `TakeOwnershipAndReset()` classifies
    // `NvmeStatus::kNotDecoding` accurately.
    return 0;
  }
  const uint64_t cap = (static_cast<uint64_t>(cap_hi) << 32) | cap_lo;
  const int dstrd = CapDstrd(cap);
  const int64_t db_end = CqHeadDoorbellOffset(1, dstrd) + 4;
  const int64_t required_end =
      (db_end + kNvmePageSize - 1) & ~(kNvmePageSize - 1);
  const int64_t limit = (bar0_limit < required_end) ? bar0_limit : required_end;

  // Step 2: Map `[kNvmePageSize, limit)` using binary halving over MSI-X holes.
  int64_t offset = kNvmePageSize;
  while (offset < limit) {
    int64_t size = limit - offset;
    uintptr_t va = 0;
    int64_t map_rc = kSyscallErrorAccess;
    while (size >= kNvmePageSize) {
      map_rc = env.map(env.ctx,                                         //
                       bar0.phys_base + static_cast<uint64_t>(offset),  //
                       size,                                            //
                       &va);
      if (map_rc != kSyscallErrorAccess) {
        break;
      }
      const int64_t pages = size / kNvmePageSize;
      if (pages <= 1) {
        size = 0;
        break;
      }
      size = (pages / 2) * kNvmePageSize;
    }
    if (map_rc != 0 && map_rc != kSyscallErrorAccess) {
      return map_rc;
    }
    if (size < kNvmePageSize) {
      offset += kNvmePageSize;
      continue;
    }
    if (!out_window->AddSegment(offset,                                   //
                                reinterpret_cast<volatile uint8_t*>(va),  //
                                size)) {
      break;
    }
    offset += size;
  }
  return 0;
}

// IPC callbacks for `ServeOne()`, mirroring `user/ramfs/ramfs.h` so host tests
// and the ring-3 driver run the exact same request validator and dispatcher.
struct ServeEnv {
  void* ctx = nullptr;
  int64_t (*recv)(void* ctx,                //
                  int64_t port,             //
                  void* buf,                //
                  int64_t cap,              //
                  IpcReceivedMessage* msg,  //
                  int64_t timeout_ticks) = nullptr;
  int64_t (*reply)(void* ctx,        //
                   int64_t port,     //
                   uint64_t token,   //
                   const void* buf,  //
                   int64_t size,     //
                   int64_t status) = nullptr;
};

struct DiskEntry {
  NvmeController* controller = nullptr;
  int ns_index = 0;
};

class BlockServer {
 public:
  static constexpr int kMaxDisks = 16;

  constexpr BlockServer() = default;

  void Reset() { disk_count_ = 0; }

  bool AddController(NvmeController* const ctrl) {
    if (ctrl == nullptr || ctrl->state() != NvmeState::kRunning) {
      return false;
    }
    const int n = ctrl->namespace_count();
    if (n < 0 || n > NvmeController::kMaxNamespaces) {
      return false;
    }
    for (int i = 0; i < n; ++i) {
      if (disk_count_ >= kMaxDisks) {
        return false;
      }
      disks_[disk_count_++] = DiskEntry{
          .controller = ctrl,
          .ns_index = i,
      };
    }
    return true;
  }

  int disk_count() const { return disk_count_; }

  int RunningDiskCount() const {
    int running = 0;
    for (int i = 0; i < disk_count_; ++i) {
      if (disks_[i].controller != nullptr &&
          disks_[i].controller->state() == NvmeState::kRunning) {
        ++running;
      }
    }
    return running;
  }

  // Dispatches one pre-received request `[req_buf, req_buf + req_size)`.
  // Writes at most `reply_cap` bytes to `reply_buf` (`*out_reply_size` bytes),
  // and returns the IPC reply status (`>= 0` on success, `< 0` on error).
  int64_t Dispatch(const uint8_t* const req_buf,  //
                   const int64_t req_size,        //
                   const int32_t reply_cap,       //
                   uint8_t* const reply_buf,      //
                   int64_t* const out_reply_size) {
    if (out_reply_size == nullptr) {
      return kSyscallErrorInvalid;
    }
    *out_reply_size = 0;
    if (req_size < 4 || req_buf == nullptr || reply_cap < 0) {
      return kSyscallErrorInvalid;
    }
    BlockRpcOp op = BlockRpcOp::kNone;
    __builtin_memcpy(&op, req_buf, sizeof(op));

    switch (op) {
      case BlockRpcOp::kGetInfo:
        return HandleGetInfo(req_buf,    //
                             req_size,   //
                             reply_cap,  //
                             reply_buf,  //
                             out_reply_size);
      case BlockRpcOp::kRead:
        return HandleRead(req_buf,    //
                          req_size,   //
                          reply_cap,  //
                          reply_buf,  //
                          out_reply_size);
      case BlockRpcOp::kWrite:
        return HandleWrite(req_buf, req_size);
      case BlockRpcOp::kFlush:
        return HandleFlush(req_buf, req_size);
      case BlockRpcOp::kNone:
        return kSyscallErrorInvalid;
    }
    return kSyscallErrorInvalid;
  }

 private:
  static int64_t MapNvmeError(const NvmeStatus status) {
    switch (status) {
      case NvmeStatus::kOk:
        return 0;
      case NvmeStatus::kInvalidArgument:
        return kSyscallErrorInvalid;
      case NvmeStatus::kReadOnly:
        return kSyscallErrorPermission;
      case NvmeStatus::kNotRunning:
        return kSyscallErrorNoDevice;
      case NvmeStatus::kNotDecoding:
      case NvmeStatus::kBadCapabilities:
      case NvmeStatus::kTimeout:
      case NvmeStatus::kDmaAllocFailed:
      case NvmeStatus::kControllerFatal:
      case NvmeStatus::kCommandFailed:
        return kSyscallErrorIo;
    }
    return kSyscallErrorIo;
  }

  static void CopyString(const char* const src,  //
                         char* const dst,        //
                         const int cap) {
    if (dst == nullptr || cap <= 0) {
      return;
    }
    int i = 0;
    if (src != nullptr) {
      while (i < cap - 1 && src[i] != '\0') {
        dst[i] = src[i];
        ++i;
      }
    }
    while (i < cap) {
      dst[i++] = '\0';
    }
  }

  int64_t HandleGetInfo(const uint8_t* const req_buf,  //
                        const int64_t req_size,        //
                        const int32_t reply_cap,       //
                        uint8_t* const reply_buf,      //
                        int64_t* const out_reply_size) const {
    if (req_size != static_cast<int64_t>(sizeof(BlockGetInfoRequest))) {
      return kSyscallErrorInvalid;
    }
    BlockGetInfoRequest req{};
    __builtin_memcpy(&req, req_buf, sizeof(req));
    if (req.disk_index >= static_cast<uint32_t>(disk_count_)) {
      return kSyscallErrorNoDevice;
    }
    if (reply_cap < static_cast<int32_t>(sizeof(BlockDeviceInfo)) ||
        reply_buf == nullptr) {
      return kSyscallErrorInvalid;
    }
    const DiskEntry& entry = disks_[req.disk_index];
    if (entry.controller == nullptr || entry.ns_index < 0 ||
        entry.ns_index >= entry.controller->namespace_count()) {
      return kSyscallErrorNoDevice;
    }
    const NamespaceInfo& ns = entry.controller->ns(entry.ns_index);
    if (!ns.valid) {
      return kSyscallErrorNoDevice;
    }
    // Keep serving static discovered metadata even if the controller entered
    // `kError` so secondary disks remain discoverable.
    BlockDeviceInfo info{};
    info.block_count = ns.block_count;
    info.block_size = ns.block_size;
    info.max_transfer_blocks = ns.max_transfer_blocks;
    info.bus = entry.controller->bus();
    info.device = entry.controller->device();
    info.function = entry.controller->function();
    info.read_only = ns.read_only ? 1 : 0;
    info.nsid = ns.nsid;
    CopyString(entry.controller->model(), info.model, kBlockModelCapacity);
    CopyString(entry.controller->serial(), info.serial, kBlockSerialCapacity);
    __builtin_memcpy(reply_buf, &info, sizeof(info));
    *out_reply_size = static_cast<int64_t>(sizeof(info));
    return disk_count_;
  }

  int64_t HandleRead(const uint8_t* const req_buf,  //
                     const int64_t req_size,        //
                     const int32_t reply_cap,       //
                     uint8_t* const reply_buf,      //
                     int64_t* const out_reply_size) {
    if (req_size != static_cast<int64_t>(sizeof(BlockReadRequest))) {
      return kSyscallErrorInvalid;
    }
    BlockReadRequest req{};
    __builtin_memcpy(&req, req_buf, sizeof(req));
    if (req.reserved != 0) {
      return kSyscallErrorInvalid;
    }
    if (req.disk_index >= static_cast<uint32_t>(disk_count_)) {
      return kSyscallErrorNoDevice;
    }
    const DiskEntry& entry = disks_[req.disk_index];
    if (entry.controller == nullptr) {
      return kSyscallErrorNoDevice;
    }
    if (entry.controller->state() != NvmeState::kRunning) {
      return (entry.controller->state() == NvmeState::kError)
                 ? kSyscallErrorIo
                 : kSyscallErrorNoDevice;
    }
    if (entry.ns_index < 0 ||
        entry.ns_index >= entry.controller->namespace_count()) {
      return kSyscallErrorIo;
    }
    const NamespaceInfo& ns = entry.controller->ns(entry.ns_index);
    if (!ns.valid || ns.block_size <= 0 || ns.max_transfer_blocks <= 0 ||
        static_cast<int64_t>(ns.max_transfer_blocks) * ns.block_size >
            kBlockMaxIoBytes) {
      return kSyscallErrorIo;
    }
    if (req.block_count < 0 || req.block_count > ns.max_transfer_blocks ||
        req.lba > ns.block_count ||
        static_cast<uint64_t>(req.block_count) > ns.block_count - req.lba) {
      return kSyscallErrorInvalid;
    }
    const int64_t bytes = static_cast<int64_t>(req.block_count) * ns.block_size;
    if (bytes > reply_cap || (bytes > 0 && reply_buf == nullptr)) {
      return kSyscallErrorInvalid;
    }
    const NvmeStatus rc = entry.controller->ReadBlocks(
        entry.ns_index, req.lba, req.block_count, reply_buf);
    if (rc != NvmeStatus::kOk) {
      return MapNvmeError(rc);
    }
    *out_reply_size = bytes;
    return req.block_count;
  }

  int64_t HandleWrite(const uint8_t* const req_buf, const int64_t req_size) {
    if (req_size < static_cast<int64_t>(sizeof(BlockWriteRequest))) {
      return kSyscallErrorInvalid;
    }
    BlockWriteRequest req{};
    __builtin_memcpy(&req, req_buf, sizeof(req));
    if (req.reserved != 0) {
      return kSyscallErrorInvalid;
    }
    if (req.disk_index >= static_cast<uint32_t>(disk_count_)) {
      return kSyscallErrorNoDevice;
    }
    const DiskEntry& entry = disks_[req.disk_index];
    if (entry.controller == nullptr) {
      return kSyscallErrorNoDevice;
    }
    if (entry.controller->state() != NvmeState::kRunning) {
      return (entry.controller->state() == NvmeState::kError)
                 ? kSyscallErrorIo
                 : kSyscallErrorNoDevice;
    }
    if (entry.ns_index < 0 ||
        entry.ns_index >= entry.controller->namespace_count()) {
      return kSyscallErrorIo;
    }
    const NamespaceInfo& ns = entry.controller->ns(entry.ns_index);
    if (!ns.valid || ns.block_size <= 0 || ns.max_transfer_blocks <= 0 ||
        static_cast<int64_t>(ns.max_transfer_blocks) * ns.block_size >
            kBlockMaxIoBytes) {
      return kSyscallErrorIo;
    }
    if (req.block_count < 0 || req.block_count > ns.max_transfer_blocks ||
        req.lba > ns.block_count ||
        static_cast<uint64_t>(req.block_count) > ns.block_count - req.lba) {
      return kSyscallErrorInvalid;
    }
    const int64_t expected_bytes =
        static_cast<int64_t>(req.block_count) * ns.block_size;
    if (req_size - static_cast<int64_t>(sizeof(req)) != expected_bytes) {
      return kSyscallErrorInvalid;
    }
    if (ns.read_only && req.block_count > 0) {
      return kSyscallErrorPermission;
    }
    const NvmeStatus rc = entry.controller->WriteBlocks(
        entry.ns_index, req.lba, req.block_count, req_buf + sizeof(req));
    if (rc != NvmeStatus::kOk) {
      return MapNvmeError(rc);
    }
    return req.block_count;
  }

  int64_t HandleFlush(const uint8_t* const req_buf, const int64_t req_size) {
    if (req_size != static_cast<int64_t>(sizeof(BlockFlushRequest))) {
      return kSyscallErrorInvalid;
    }
    BlockFlushRequest req{};
    __builtin_memcpy(&req, req_buf, sizeof(req));
    if (req.disk_index >= static_cast<uint32_t>(disk_count_)) {
      return kSyscallErrorNoDevice;
    }
    const DiskEntry& entry = disks_[req.disk_index];
    if (entry.controller == nullptr) {
      return kSyscallErrorNoDevice;
    }
    if (entry.controller->state() != NvmeState::kRunning) {
      return (entry.controller->state() == NvmeState::kError)
                 ? kSyscallErrorIo
                 : kSyscallErrorNoDevice;
    }
    if (entry.ns_index < 0 ||
        entry.ns_index >= entry.controller->namespace_count() ||
        !entry.controller->ns(entry.ns_index).valid) {
      return kSyscallErrorIo;
    }
    const NvmeStatus rc = entry.controller->Flush(entry.ns_index);
    return (rc == NvmeStatus::kOk) ? 0 : MapNvmeError(rc);
  }

  DiskEntry disks_[kMaxDisks]{};
  int disk_count_ = 0;
};

static_assert(std::is_trivially_destructible_v<BlockServer>);

inline bool ServeOne(BlockServer& server,       //
                     const ServeEnv& env,       //
                     uint8_t* const req_buf,    //
                     const int64_t req_cap,     //
                     uint8_t* const reply_buf,  //
                     const int64_t reply_cap) {
  if (env.recv == nullptr || env.reply == nullptr || req_buf == nullptr ||
      req_cap <= 0 || reply_buf == nullptr || reply_cap <= 0) {
    return false;
  }
  IpcReceivedMessage msg{};
  const int64_t rc = env.recv(env.ctx,        //
                              kIpcPortBlock,  //
                              req_buf,        //
                              req_cap,        //
                              &msg,           //
                              /*timeout_ticks=*/-1);
  if (rc != 1) {
    return false;
  }
  if (msg.kind == IpcMessageKind::kClientDisconnect) {
    return true;
  }
  if (msg.kind != IpcMessageKind::kCall) {
    return false;
  }
  int64_t reply_size = 0;
  int64_t capped_reply_cap =
      (msg.reply_cap < reply_cap) ? msg.reply_cap : reply_cap;
  if (capped_reply_cap < 0) {
    capped_reply_cap = 0;
  }
  if (capped_reply_cap > kIpcMaxMessageSize) {
    capped_reply_cap = kIpcMaxMessageSize;
  }
  const int32_t effective_cap = static_cast<int32_t>(capped_reply_cap);
  int64_t status = kSyscallErrorInvalid;
  if (msg.size >= 0 && msg.size <= req_cap) {
    status = server.Dispatch(req_buf,        //
                             msg.size,       //
                             effective_cap,  //
                             reply_buf,      //
                             &reply_size);
  }
  if (env.reply(env.ctx,        //
                kIpcPortBlock,  //
                msg.token,      //
                reply_buf,      //
                reply_size,     //
                status) != 0) {
    (void)env.reply(env.ctx,        //
                    kIpcPortBlock,  //
                    msg.token,      //
                    nullptr,        //
                    0,              //
                    kSyscallErrorIo);
  }
  return true;
}

}  // namespace protos::nvme

#endif  // PROTOS_USER_NVME_NVME_SERVER_H_
