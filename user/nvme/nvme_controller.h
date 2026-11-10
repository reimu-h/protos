#ifndef PROTOS_USER_NVME_NVME_CONTROLLER_H_
#define PROTOS_USER_NVME_NVME_CONTROLLER_H_

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "block_protocol.h"
#include "user/nvme/nvme_platform.h"
#include "user/nvme/nvme_regs.h"

namespace protos::nvme {

enum class NvmeState : uint8_t {
  kUninitialized = 0,
  kHalted = 1,
  kRunning = 2,
  kError = 3,
};

enum class NvmeStatus : uint8_t {
  kOk = 0,
  kNotDecoding = 1,
  kBadCapabilities = 2,
  kTimeout = 3,
  kDmaAllocFailed = 4,
  kControllerFatal = 5,
  kCommandFailed = 6,
  kInvalidArgument = 7,
  kReadOnly = 8,
  kNotRunning = 9,
};

enum class IrqMode : uint8_t {
  kMsix = 0,
  kMsi = 1,
};

struct NamespaceInfo {
  uint32_t nsid = 0;
  uint64_t block_count = 0;
  int32_t block_size = 0;
  int32_t max_transfer_blocks = 0;
  bool valid = false;
  bool read_only = false;
};

class NvmeController {
 public:
  static constexpr int kQueueDepth = 32;
  static constexpr int kMaxNamespaces = 8;
  static constexpr int64_t kPageSize = 4096;
  static constexpr int64_t kCommandTimeoutMs = 5000;
  static constexpr int64_t kIoMaxBytes =
      (kBlockMaxIoBytes < kPageSize) ? kBlockMaxIoBytes : kPageSize;

  static_assert(kBlockMaxIoBytes > 0 && kBlockMaxIoBytes <= kPageSize);
  static_assert(kIdentifyDataBytes == kPageSize);
  static_assert(kQueueDepth >= 2);
  static_assert(int64_t{kQueueDepth} *
                    static_cast<int64_t>(sizeof(SubmissionEntry)) <=
                kPageSize);
  static_assert(int64_t{kQueueDepth} *
                    static_cast<int64_t>(sizeof(CompletionEntry)) <=
                kPageSize);

  constexpr NvmeController() = default;
  NvmeController(const NvmeController&) = delete;
  NvmeController& operator=(const NvmeController&) = delete;

  void Init(Platform& platform,        //
            MmioWindow& mmio,          //
            const uint8_t bus = 0,     //
            const uint8_t device = 0,  //
            const uint8_t function = 0) {
    platform_ = &platform;
    mmio_ = &mmio;
    bus_ = bus;
    device_ = device;
    function_ = function;
    state_ = NvmeState::kUninitialized;
    dstrd_ = 0;
    admin_depth_ = 0;
    io_depth_ = 0;
    enable_timeout_ms_ = 2000;
    irq_mask_ = 0;
    irq_mode_ = IrqMode::kMsix;
    admin_sq_tail_ = 0;
    admin_cq_head_ = 0;
    admin_cq_phase_ = 1;
    io_sq_tail_ = 0;
    io_cq_head_ = 0;
    io_cq_phase_ = 1;
    next_cid_ = 1;
    nn_ = 0;
    max_transfer_bytes_ = kIoMaxBytes;
    has_vwc_ = false;
    model_[0] = '\0';
    serial_[0] = '\0';
    namespace_count_ = 0;
    for (int i = 0; i < kMaxNamespaces; ++i) {
      namespaces_[i] = NamespaceInfo{};
    }
  }

  NvmeState state() const { return state_; }
  int namespace_count() const { return namespace_count_; }
  const NamespaceInfo& ns(const int index) const { return namespaces_[index]; }
  const char* model() const { return model_; }
  const char* serial() const { return serial_; }
  bool has_volatile_write_cache() const { return has_vwc_; }
  uint8_t bus() const { return bus_; }
  uint8_t device() const { return device_; }
  uint8_t function() const { return function_; }

  // Phase 1: Validate controller registers, disable `CC` and wait until
  // `CSTS.RDY == 0` (before touching endpoint Bus Master, leaving no DMA
  // running), and allocate DMA ring/scratch pages once.
  [[nodiscard]] NvmeStatus TakeOwnershipAndReset() {
    if (platform_ == nullptr || mmio_ == nullptr) {
      return Fail(NvmeStatus::kInvalidArgument);
    }
    if (!mmio_->Covers(kRegCap, 0x38)) {
      return Fail(NvmeStatus::kNotDecoding);
    }
    const uint64_t cap = mmio_->Read64(kRegCap);
    const uint32_t cap_lo = static_cast<uint32_t>(cap);
    const uint32_t cap_hi = static_cast<uint32_t>(cap >> 32);
    if (cap_lo == kMmioAllOnes || cap_hi == kMmioAllOnes) {
      return Fail(NvmeStatus::kNotDecoding);
    }
    const int mqes = CapMqes(cap);
    const int dstrd = CapDstrd(cap);
    const int mps_min = CapMpsMin(cap);
    const int mps_max = CapMpsMax(cap);
    if ((cap & kCapCssNvm) == 0 || mps_min != 0 || mps_max < mps_min ||
        mqes < 2) {
      return Fail(NvmeStatus::kBadCapabilities);
    }
    if (!mmio_->Covers(SqTailDoorbellOffset(0, dstrd), 4) ||
        !mmio_->Covers(CqHeadDoorbellOffset(0, dstrd), 4) ||
        !mmio_->Covers(SqTailDoorbellOffset(1, dstrd), 4) ||
        !mmio_->Covers(CqHeadDoorbellOffset(1, dstrd), 4)) {
      return Fail(NvmeStatus::kBadCapabilities);
    }

    dstrd_ = dstrd;
    const int depth = (mqes < kQueueDepth) ? mqes : kQueueDepth;
    admin_depth_ = depth;
    io_depth_ = depth;
    const int raw_to_ms = CapToMs(cap);
    enable_timeout_ms_ = (raw_to_ms > 2000) ? raw_to_ms : 2000;

    // Reset controller (`CC = 0`) and wait for `CSTS.RDY == 0`.
    const uint32_t cc = mmio_->Read32(kRegCc);
    if (cc == kMmioAllOnes) {
      return Fail(NvmeStatus::kNotDecoding);
    }
    if (cc != 0) {
      mmio_->Write32(kRegCc, 0);
    }
    const NvmeStatus wait_rc =
        WaitReadyBit(/*want_ready=*/false, enable_timeout_ms_);
    if (wait_rc != NvmeStatus::kOk) {
      return Fail(wait_rc);
    }

    if (!EnsureDmaBuffers()) {
      return Fail(NvmeStatus::kDmaAllocFailed);
    }
    state_ = NvmeState::kHalted;
    return NvmeStatus::kOk;
  }

  // Phase 2 (after `PciBusMaster` and `IrqAlloc`): program Admin queues, enable
  // `CC.EN`, unmask `INTMC` when in MSI mode, identify the controller and
  // active namespaces, and create I/O queue pair 1.
  [[nodiscard]] NvmeStatus Start(const uint64_t irq_mask,
                                 const IrqMode irq_mode = IrqMode::kMsix) {
    if (platform_ == nullptr || mmio_ == nullptr ||
        state_ != NvmeState::kHalted ||
        (irq_mode != IrqMode::kMsix && irq_mode != IrqMode::kMsi)) {
      return NvmeStatus::kInvalidArgument;
    }
    irq_mask_ = irq_mask;
    irq_mode_ = irq_mode;

    ZeroBuffer(admin_sq_);
    ZeroBuffer(admin_cq_);
    ZeroBuffer(io_sq_);
    ZeroBuffer(io_cq_);
    ZeroBuffer(identify_buf_);
    ZeroBuffer(io_buf_);

    admin_sq_tail_ = 0;
    admin_cq_head_ = 0;
    admin_cq_phase_ = 1;
    io_sq_tail_ = 0;
    io_cq_head_ = 0;
    io_cq_phase_ = 1;
    next_cid_ = 1;
    namespace_count_ = 0;
    for (int i = 0; i < kMaxNamespaces; ++i) {
      namespaces_[i] = NamespaceInfo{};
    }
    max_transfer_bytes_ = kIoMaxBytes;
    has_vwc_ = false;
    model_[0] = '\0';
    serial_[0] = '\0';

    const uint32_t aqa = (static_cast<uint32_t>(admin_depth_ - 1) << 16) |
                         static_cast<uint32_t>(admin_depth_ - 1);
    mmio_->Write32(kRegAqa, aqa);
    mmio_->Write64(kRegAsq, admin_sq_.phys);
    mmio_->Write64(kRegAcq, admin_cq_.phys);

    const uint32_t cc = kCcEn | kCcCssNvm | kCcMps4K | kCcAmsRoundRobin |
                        kCcIoSqes64 | kCcIoCqes16;
    mmio_->Write32(kRegCc, cc);
    const NvmeStatus rdy_rc =
        WaitReadyBit(/*want_ready=*/true, enable_timeout_ms_);
    if (rdy_rc != NvmeStatus::kOk) {
      return FailAndDisable(rdy_rc);
    }

    if (irq_mode_ == IrqMode::kMsi) {
      mmio_->Write32(kRegIntmc, 0xFFFFFFFFu);
    }

    const NvmeStatus ctrl_rc = IdentifyController();
    if (ctrl_rc != NvmeStatus::kOk) {
      return FailAndDisable(ctrl_rc);
    }

    const NvmeStatus ns_rc = DiscoverNamespaces();
    if (ns_rc != NvmeStatus::kOk) {
      return FailAndDisable(ns_rc);
    }

    // Per NVMe Base Spec §5.21.1.7, allocate queue count (1 SQ + 1 CQ = 0x0)
    // and check reserved 0xFFFF allocation before creating I/O queues.
    {
      SubmissionEntry cmd{};
      cmd.cdw0 = MakeSqCdw0(kAdminSetFeatures, NextCid());
      cmd.cdw10 = kFeatureNumberOfQueues;
      cmd.cdw11 = 0;
      CompletionEntry feat_cqe{};
      const NvmeStatus feat_rc = SubmitAdminAndWait(cmd, &feat_cqe);
      if (feat_rc != NvmeStatus::kOk) {
        return FailAndDisable(feat_rc);
      }
      const uint32_t nsqa_raw = feat_cqe.dw0 & 0xFFFFu;
      const uint32_t ncqa_raw = (feat_cqe.dw0 >> 16) & 0xFFFFu;
      if (nsqa_raw == 0xFFFFu || ncqa_raw == 0xFFFFu) {
        return FailAndDisable(NvmeStatus::kCommandFailed);
      }
    }

    {
      SubmissionEntry cmd{};
      cmd.cdw0 = MakeSqCdw0(kAdminCreateIoCq, NextCid());
      cmd.prp1 = io_cq_.phys;
      cmd.cdw10 = (static_cast<uint32_t>(io_depth_ - 1) << 16) | 1u;
      cmd.cdw11 = kCqInterruptEnabled | kQueuePhysicallyContiguous;
      const NvmeStatus cq_rc = SubmitAdminAndWait(cmd, nullptr);
      if (cq_rc != NvmeStatus::kOk) {
        return FailAndDisable(cq_rc);
      }
    }

    {
      SubmissionEntry cmd{};
      cmd.cdw0 = MakeSqCdw0(kAdminCreateIoSq, NextCid());
      cmd.prp1 = io_sq_.phys;
      cmd.cdw10 = (static_cast<uint32_t>(io_depth_ - 1) << 16) | 1u;
      cmd.cdw11 = (1u << 16) | kQueuePhysicallyContiguous;
      const NvmeStatus sq_rc = SubmitAdminAndWait(cmd, nullptr);
      if (sq_rc != NvmeStatus::kOk) {
        return FailAndDisable(sq_rc);
      }
    }

    state_ = NvmeState::kRunning;
    return NvmeStatus::kOk;
  }

  // Performs NVMe normal shutdown (`CC.SHN = 01b` -> wait `CSTS.SHST == 10b`
  // -> `CC = 0` -> wait `CSTS.RDY == 0`) when running, or best-effort `CC = 0`
  // reset when halted/faulted. Preserves discovered `model_`, `serial_`, and
  // `namespaces_` metadata.
  [[nodiscard]] NvmeStatus Shutdown() {
    if (platform_ == nullptr || mmio_ == nullptr ||
        state_ == NvmeState::kUninitialized) {
      return NvmeStatus::kInvalidArgument;
    }
    if (!mmio_->Covers(kRegCc, 4) || !mmio_->Covers(kRegCsts, 4)) {
      state_ = NvmeState::kError;
      return NvmeStatus::kNotDecoding;
    }
    const uint32_t cc = mmio_->Read32(kRegCc);
    const uint32_t csts = mmio_->Read32(kRegCsts);
    if (cc == kMmioAllOnes || csts == kMmioAllOnes) {
      state_ = NvmeState::kError;
      return NvmeStatus::kNotDecoding;
    }
    if (state_ == NvmeState::kError) {
      return DisableControllerBestEffort();
    }
    if (state_ == NvmeState::kHalted) {
      if (cc != 0 || (csts & kCstsRdy) != 0) {
        const NvmeStatus reset_rc = DisableControllerBestEffort();
        if (reset_rc != NvmeStatus::kOk) {
          state_ = NvmeState::kError;
          return reset_rc;
        }
      }
      return NvmeStatus::kOk;
    }

    NvmeStatus first_err = NvmeStatus::kOk;
    if ((csts & kCstsCfs) != 0) {
      first_err = NvmeStatus::kControllerFatal;
    } else if ((cc & kCcEn) != 0 && (csts & kCstsRdy) != 0) {
      mmio_->Write32(kRegCc, (cc & ~kCcShnMask) | kCcShnNormal);
      first_err = WaitShutdownComplete(enable_timeout_ms_);
    }
    if (first_err != NvmeStatus::kNotDecoding) {
      const NvmeStatus reset_rc = DisableControllerBestEffort();
      if (first_err == NvmeStatus::kOk) {
        first_err = reset_rc;
      }
    }
    if (first_err != NvmeStatus::kOk) {
      state_ = NvmeState::kError;
      return first_err;
    }
    state_ = NvmeState::kHalted;
    return NvmeStatus::kOk;
  }

  // Reads `block_count` (`0 <= block_count <=
  // ns(ns_index).max_transfer_blocks`) blocks starting at `lba` into `dst`.
  [[nodiscard]] NvmeStatus ReadBlocks(const int ns_index,    //
                                      const uint64_t lba,    //
                                      const int32_t blocks,  //
                                      void* const dst) {
    const NvmeStatus check = ValidateIoArgs(ns_index, lba, blocks, dst);
    if (check != NvmeStatus::kOk) {
      return check;
    }
    if (blocks == 0) {
      return NvmeStatus::kOk;
    }
    const NamespaceInfo& info = namespaces_[ns_index];
    const int64_t bytes =
        static_cast<int64_t>(blocks) * static_cast<int64_t>(info.block_size);

    SubmissionEntry cmd{};
    cmd.cdw0 = MakeSqCdw0(kNvmRead, NextCid());
    cmd.nsid = info.nsid;
    cmd.prp1 = io_buf_.phys;
    cmd.cdw10 = static_cast<uint32_t>(lba);
    cmd.cdw11 = static_cast<uint32_t>(lba >> 32);
    cmd.cdw12 = static_cast<uint32_t>(blocks - 1);
    const NvmeStatus rc = SubmitIoAndWait(cmd);
    if (rc != NvmeStatus::kOk) {
      return rc;
    }
    asm volatile("" ::: "memory");
    __builtin_memcpy(dst, io_buf_.va, static_cast<size_t>(bytes));
    return NvmeStatus::kOk;
  }

  // Writes `blocks` blocks from `src` starting at `lba`.
  [[nodiscard]] NvmeStatus WriteBlocks(const int ns_index,    //
                                       const uint64_t lba,    //
                                       const int32_t blocks,  //
                                       const void* const src) {
    const NvmeStatus check = ValidateIoArgs(ns_index, lba, blocks, src);
    if (check != NvmeStatus::kOk) {
      return check;
    }
    if (blocks == 0) {
      return NvmeStatus::kOk;
    }
    const NamespaceInfo& info = namespaces_[ns_index];
    if (info.read_only) {
      return NvmeStatus::kReadOnly;
    }
    const int64_t bytes =
        static_cast<int64_t>(blocks) * static_cast<int64_t>(info.block_size);

    __builtin_memcpy(io_buf_.va, src, static_cast<size_t>(bytes));
    asm volatile("" ::: "memory");

    SubmissionEntry cmd{};
    cmd.cdw0 = MakeSqCdw0(kNvmWrite, NextCid());
    cmd.nsid = info.nsid;
    cmd.prp1 = io_buf_.phys;
    cmd.cdw10 = static_cast<uint32_t>(lba);
    cmd.cdw11 = static_cast<uint32_t>(lba >> 32);
    cmd.cdw12 = static_cast<uint32_t>(blocks - 1);
    return SubmitIoAndWait(cmd);
  }

  [[nodiscard]] NvmeStatus Flush(const int ns_index) {
    if (state_ != NvmeState::kRunning) {
      return NvmeStatus::kNotRunning;
    }
    if (ns_index < 0 || ns_index >= namespace_count_ ||
        !namespaces_[ns_index].valid) {
      return NvmeStatus::kInvalidArgument;
    }
    SubmissionEntry cmd{};
    cmd.cdw0 = MakeSqCdw0(kNvmFlush, NextCid());
    cmd.nsid = namespaces_[ns_index].nsid;
    return SubmitIoAndWait(cmd);
  }

 private:
  static constexpr int64_t kMaxWaitChunkTicks = 20;

  NvmeStatus Fail(const NvmeStatus status) {
    state_ = NvmeState::kError;
    return status;
  }

  NvmeStatus FailAndDisable(const NvmeStatus status) {
    if (status != NvmeStatus::kNotDecoding) {
      (void)DisableControllerBestEffort();
    }
    state_ = NvmeState::kError;
    return status;
  }

  NvmeStatus DisableControllerBestEffort() {
    if (platform_ == nullptr || mmio_ == nullptr || !mmio_->Covers(kRegCc, 4) ||
        !mmio_->Covers(kRegCsts, 4)) {
      return NvmeStatus::kNotDecoding;
    }
    const uint32_t cc = mmio_->Read32(kRegCc);
    if (cc == kMmioAllOnes) {
      return NvmeStatus::kNotDecoding;
    }
    if (cc != 0) {
      mmio_->Write32(kRegCc, 0);
    }
    return WaitReadyBit(/*want_ready=*/false, enable_timeout_ms_);
  }

  uint16_t NextCid() {
    const uint16_t cid = next_cid_++;
    if (next_cid_ == 0) {
      next_cid_ = 1;
    }
    return cid;
  }

  bool EnsureDmaBuffers() {
    auto alloc_once = [this](DmaBuffer& buf) -> bool {
      if (buf.va != nullptr) {
        return true;
      }
      const DmaBuffer allocated =
          platform_->DmaAlloc(kPageSize, /*below_4gib=*/false);
      if (allocated.va == nullptr || allocated.phys == 0 ||
          allocated.size < kPageSize ||
          (allocated.phys & static_cast<uint64_t>(kPageSize - 1)) != 0) {
        return false;
      }
      buf = allocated;
      return true;
    };
    return alloc_once(admin_sq_) && alloc_once(admin_cq_) &&
           alloc_once(io_sq_) && alloc_once(io_cq_) &&
           alloc_once(identify_buf_) && alloc_once(io_buf_);
  }

  static void ZeroBuffer(const DmaBuffer& buf) {
    if (buf.va != nullptr && buf.size > 0) {
      __builtin_memset(buf.va, 0, static_cast<size_t>(buf.size));
    }
  }

  NvmeStatus WaitReadyBit(const bool want_ready, const int64_t timeout_ms) {
    const int64_t raw_ticks =
        TicksFromMs(timeout_ms, platform_->TicksPerSecond());
    const uint64_t wait_ticks =
        static_cast<uint64_t>(raw_ticks > 0 ? raw_ticks : 1);
    const uint64_t start = static_cast<uint64_t>(platform_->NowTicks());
    for (;;) {
      const uint64_t now = static_cast<uint64_t>(platform_->NowTicks());
      const bool last_poll = (now - start) >= wait_ticks;
      const uint32_t csts = mmio_->Read32(kRegCsts);
      if (csts == kMmioAllOnes) {
        return NvmeStatus::kNotDecoding;
      }
      if (want_ready && (csts & kCstsCfs) != 0) {
        return NvmeStatus::kControllerFatal;
      }
      const bool is_ready = (csts & kCstsRdy) != 0;
      if (is_ready == want_ready) {
        return NvmeStatus::kOk;
      }
      if (last_poll) {
        return NvmeStatus::kTimeout;
      }
      platform_->Relax();
    }
  }

  NvmeStatus WaitShutdownComplete(const int64_t timeout_ms) {
    const int64_t raw_ticks =
        TicksFromMs(timeout_ms, platform_->TicksPerSecond());
    const uint64_t wait_ticks =
        static_cast<uint64_t>(raw_ticks > 0 ? raw_ticks : 1);
    const uint64_t start = static_cast<uint64_t>(platform_->NowTicks());
    for (;;) {
      const uint64_t now = static_cast<uint64_t>(platform_->NowTicks());
      const bool last_poll = (now - start) >= wait_ticks;
      const uint32_t csts = mmio_->Read32(kRegCsts);
      if (csts == kMmioAllOnes) {
        return NvmeStatus::kNotDecoding;
      }
      if ((csts & kCstsCfs) != 0) {
        return NvmeStatus::kControllerFatal;
      }
      if ((csts & kCstsShstMask) == kCstsShstComplete) {
        return NvmeStatus::kOk;
      }
      if (last_poll) {
        return NvmeStatus::kTimeout;
      }
      platform_->Relax();
    }
  }

  NvmeStatus SubmitAndWait(const int qid,               //
                           const DmaBuffer& sq_buf,     //
                           const DmaBuffer& cq_buf,     //
                           const int depth,             //
                           int& sq_tail,                //
                           int& cq_head,                //
                           uint16_t& cq_phase,          //
                           const SubmissionEntry& cmd,  //
                           CompletionEntry* const out_cqe) {
    SubmissionEntry* const sq = static_cast<SubmissionEntry*>(sq_buf.va);
    const volatile CompletionEntry* const cq =
        static_cast<const volatile CompletionEntry*>(cq_buf.va);
    sq[sq_tail] = cmd;
    asm volatile("" ::: "memory");
    sq_tail = (sq_tail + 1) % depth;
    mmio_->Write32(SqTailDoorbellOffset(qid, dstrd_),
                   static_cast<uint32_t>(sq_tail));

    const uint16_t expected_cid = static_cast<uint16_t>(cmd.cdw0 >> 16);
    const int64_t raw_ticks =
        TicksFromMs(kCommandTimeoutMs, platform_->TicksPerSecond());
    const uint64_t wait_ticks =
        static_cast<uint64_t>(raw_ticks > 0 ? raw_ticks : 1);
    const uint64_t start = static_cast<uint64_t>(platform_->NowTicks());

    for (;;) {
      const uint64_t now = static_cast<uint64_t>(platform_->NowTicks());
      const bool last_poll = (now - start) >= wait_ticks;

      const uint32_t csts = mmio_->Read32(kRegCsts);
      if (csts == kMmioAllOnes) {
        state_ = NvmeState::kError;
        return NvmeStatus::kNotDecoding;
      }
      if ((csts & kCstsCfs) != 0) {
        (void)DisableControllerBestEffort();
        state_ = NvmeState::kError;
        return NvmeStatus::kControllerFatal;
      }

      const volatile uint16_t* const status_ptr = &cq[cq_head].status;
      asm volatile("" ::: "memory");
      const uint16_t raw_status = *status_ptr;
      if ((raw_status & kCqePhaseMask) == cq_phase) {
        asm volatile("" ::: "memory");
        CompletionEntry entry;
        entry.dw0 = cq[cq_head].dw0;
        entry.dw1 = cq[cq_head].dw1;
        entry.sq_head = cq[cq_head].sq_head;
        entry.sq_id = cq[cq_head].sq_id;
        entry.cid = cq[cq_head].cid;
        entry.status = raw_status;

        if (entry.cid != expected_cid ||
            entry.sq_id != static_cast<uint16_t>(qid)) {
          (void)DisableControllerBestEffort();
          state_ = NvmeState::kError;
          return NvmeStatus::kCommandFailed;
        }

        cq_head = (cq_head + 1) % depth;
        if (cq_head == 0) {
          cq_phase ^= 1u;
        }
        mmio_->Write32(CqHeadDoorbellOffset(qid, dstrd_),
                       static_cast<uint32_t>(cq_head));

        if (out_cqe != nullptr) {
          *out_cqe = entry;
        }
        if (!CqeOk(entry.status)) {
          if (CqeStatusType(entry.status) == kStatusTypeCommandSpecific &&
              CqeStatusCode(entry.status) == kStatusCodeWriteToReadOnlyRange) {
            return NvmeStatus::kReadOnly;
          }
          if (CqeStatusType(entry.status) == kStatusTypeGeneric &&
              (CqeStatusCode(entry.status) == kStatusCodeInvalidField ||
               CqeStatusCode(entry.status) == kStatusCodeLbaOutOfRange)) {
            return NvmeStatus::kInvalidArgument;
          }
          return NvmeStatus::kCommandFailed;
        }
        return NvmeStatus::kOk;
      }

      if (last_poll) {
        (void)DisableControllerBestEffort();
        state_ = NvmeState::kError;
        return NvmeStatus::kTimeout;
      }

      const uint64_t now_before_wait =
          static_cast<uint64_t>(platform_->NowTicks());
      const uint64_t elapsed_before_wait = now_before_wait - start;
      if (elapsed_before_wait >= wait_ticks) {
        continue;
      }
      const uint64_t remaining = wait_ticks - elapsed_before_wait;
      const int64_t wait_chunk =
          (remaining > static_cast<uint64_t>(kMaxWaitChunkTicks))
              ? kMaxWaitChunkTicks
              : static_cast<int64_t>(remaining);
      platform_->WaitForIrq(irq_mask_, wait_chunk);
    }
  }

  NvmeStatus SubmitAdminAndWait(const SubmissionEntry& cmd,
                                CompletionEntry* const out_cqe) {
    return SubmitAndWait(/*qid=*/0,        //
                         admin_sq_,        //
                         admin_cq_,        //
                         admin_depth_,     //
                         admin_sq_tail_,   //
                         admin_cq_head_,   //
                         admin_cq_phase_,  //
                         cmd,              //
                         out_cqe);
  }

  NvmeStatus SubmitIoAndWait(const SubmissionEntry& cmd) {
    return SubmitAndWait(/*qid=*/1,     //
                         io_sq_,        //
                         io_cq_,        //
                         io_depth_,     //
                         io_sq_tail_,   //
                         io_cq_head_,   //
                         io_cq_phase_,  //
                         cmd,           //
                         nullptr);
  }

  static uint16_t ReadLe16(const uint8_t* const p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
  }

  static uint32_t ReadLe32(const uint8_t* const p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
  }

  static uint64_t ReadLe64(const uint8_t* const p) {
    return static_cast<uint64_t>(ReadLe32(p)) |
           (static_cast<uint64_t>(ReadLe32(p + 4)) << 32);
  }

  static void CopyTrimmedAscii(const uint8_t* const src,  //
                               const int src_len,         //
                               char* const dst,           //
                               const int dst_cap) {
    if (dst_cap <= 0) {
      return;
    }
    int end = src_len;
    while (end > 0 && (src[end - 1] == ' ' || src[end - 1] == '\0')) {
      --end;
    }
    if (end >= dst_cap) {
      end = dst_cap - 1;
    }
    for (int i = 0; i < end; ++i) {
      const uint8_t ch = src[i];
      dst[i] = (ch >= 0x20 && ch <= 0x7E) ? static_cast<char>(ch) : '?';
    }
    for (int i = end; i < dst_cap; ++i) {
      dst[i] = '\0';
    }
  }

  NvmeStatus IdentifyController() {
    __builtin_memset(identify_buf_.va, 0, kIdentifyDataBytes);
    SubmissionEntry cmd{};
    cmd.cdw0 = MakeSqCdw0(kAdminIdentify, NextCid());
    cmd.prp1 = identify_buf_.phys;
    cmd.cdw10 = kIdentifyCnsController;
    const NvmeStatus rc = SubmitAdminAndWait(cmd, nullptr);
    if (rc != NvmeStatus::kOk) {
      return rc;
    }
    const uint8_t* const data = static_cast<const uint8_t*>(identify_buf_.va);
    CopyTrimmedAscii(data + kIdCtrlSerialOffset,  //
                     kIdCtrlSerialLength,         //
                     serial_,                     //
                     kBlockSerialCapacity);
    CopyTrimmedAscii(data + kIdCtrlModelOffset,  //
                     kIdCtrlModelLength,         //
                     model_,                     //
                     kBlockModelCapacity);
    nn_ = ReadLe32(data + kIdCtrlNnOffset);
    has_vwc_ = (data[kIdCtrlVwcOffset] & 1u) != 0;
    const uint8_t mdts = data[kIdCtrlMdtsOffset];
    max_transfer_bytes_ = kIoMaxBytes;
    if (mdts > 0 && mdts < 20) {
      const int64_t mdts_bytes = kPageSize << mdts;
      if (mdts_bytes < max_transfer_bytes_) {
        max_transfer_bytes_ = mdts_bytes;
      }
    }
    return NvmeStatus::kOk;
  }

  NvmeStatus ProbeNamespace(const uint32_t nsid) {
    if (namespace_count_ >= kMaxNamespaces || nsid == 0 ||
        nsid == 0xFFFFFFFFu) {
      return NvmeStatus::kOk;
    }
    __builtin_memset(identify_buf_.va, 0, kIdentifyDataBytes);
    SubmissionEntry cmd{};
    cmd.cdw0 = MakeSqCdw0(kAdminIdentify, NextCid());
    cmd.nsid = nsid;
    cmd.prp1 = identify_buf_.phys;
    cmd.cdw10 = kIdentifyCnsNamespace;
    const NvmeStatus rc = SubmitAdminAndWait(cmd, nullptr);
    if (rc == NvmeStatus::kCommandFailed ||
        rc == NvmeStatus::kInvalidArgument) {
      return NvmeStatus::kOk;  // Inactive/unallocated NSID
    }
    if (rc != NvmeStatus::kOk) {
      return rc;
    }

    const uint8_t* const data = static_cast<const uint8_t*>(identify_buf_.va);
    const uint64_t nsze = ReadLe64(data + kIdNsSizeOffset);
    const uint64_t ncap = ReadLe64(data + kIdNsCapOffset);
    if (nsze == 0 || ncap == 0) {
      return NvmeStatus::kOk;
    }
    const int nlbaf = static_cast<int>(data[kIdNsNlbafOffset]);
    if (nlbaf >= 64) {
      return NvmeStatus::kOk;
    }
    const uint8_t flbas = data[kIdNsFlbasOffset];
    int lba_idx = static_cast<int>(flbas & 0xFu);
    if (nlbaf >= 16) {
      lba_idx |= static_cast<int>((flbas >> 5) & 0x3u) << 4;
    }
    if (lba_idx > nlbaf) {
      return NvmeStatus::kOk;
    }
    const uint32_t lbaf = ReadLe32(data + kIdNsLbaf0Offset + 4 * lba_idx);
    const uint32_t ms = lbaf & 0xFFFFu;
    const int lbads = static_cast<int>((lbaf >> 16) & 0xFFu);
    if (ms != 0 || lbads < 9 || lbads > 12) {
      return NvmeStatus::kOk;
    }
    const int32_t block_size = int32_t{1} << lbads;
    const int64_t max_bytes =
        (max_transfer_bytes_ < kIoMaxBytes) ? max_transfer_bytes_ : kIoMaxBytes;
    const int32_t max_blocks = static_cast<int32_t>(max_bytes / block_size);
    if (max_blocks < 1) {
      return NvmeStatus::kOk;
    }
    const bool read_only =
        (data[kIdNsAttrOffset] & kIdNsAttrWriteProtected) != 0;

    namespaces_[namespace_count_++] = NamespaceInfo{
        .nsid = nsid,
        .block_count = nsze,
        .block_size = block_size,
        .max_transfer_blocks = max_blocks,
        .valid = true,
        .read_only = read_only,
    };
    return NvmeStatus::kOk;
  }

  NvmeStatus DiscoverNamespaces() {
    uint32_t cursor = 0;
    bool first_page = true;
    while (namespace_count_ < kMaxNamespaces) {
      __builtin_memset(identify_buf_.va, 0, kIdentifyDataBytes);
      SubmissionEntry cmd{};
      cmd.cdw0 = MakeSqCdw0(kAdminIdentify, NextCid());
      cmd.nsid = cursor;
      cmd.prp1 = identify_buf_.phys;
      cmd.cdw10 = kIdentifyCnsActiveNsList;
      const NvmeStatus list_rc = SubmitAdminAndWait(cmd, nullptr);
      if (list_rc != NvmeStatus::kOk) {
        if (first_page && (list_rc == NvmeStatus::kInvalidArgument ||
                           list_rc == NvmeStatus::kCommandFailed)) {
          break;  // Fall back to legacy 1..NN probe.
        }
        return list_rc;
      }
      first_page = false;

      // Copy a bounded batch of active NSIDs first because ProbeNamespace()
      // reuses `identify_buf_`, then paginate via `cmd.nsid = max_seen`.
      constexpr int kBatchCap = 32;
      uint32_t nsids[kBatchCap] = {};
      int batch_count = 0;
      uint32_t max_seen = cursor;
      bool reached_end = false;
      const uint8_t* const list = static_cast<const uint8_t*>(identify_buf_.va);
      for (int i = 0; i < 1024 && batch_count < kBatchCap; ++i) {
        const uint32_t nsid = ReadLe32(list + 4 * i);
        if (nsid == 0) {
          reached_end = true;
          break;
        }
        if (nsid == 0xFFFFFFFFu || nsid <= max_seen) {
          continue;
        }
        nsids[batch_count++] = nsid;
        max_seen = nsid;
      }
      for (int i = 0; i < batch_count && namespace_count_ < kMaxNamespaces;
           ++i) {
        const NvmeStatus rc = ProbeNamespace(nsids[i]);
        if (rc != NvmeStatus::kOk) {
          return rc;
        }
      }
      if (reached_end || max_seen <= cursor || max_seen >= 0xFFFFFFFEu) {
        return NvmeStatus::kOk;
      }
      cursor = max_seen;
    }

    if (!first_page) {
      return NvmeStatus::kOk;
    }

    const uint32_t probe_limit = (nn_ < 1024u) ? nn_ : 1024u;
    for (uint32_t nsid = 1;
         nsid <= probe_limit && namespace_count_ < kMaxNamespaces; ++nsid) {
      const NvmeStatus rc = ProbeNamespace(nsid);
      if (rc != NvmeStatus::kOk) {
        return rc;
      }
    }
    return NvmeStatus::kOk;
  }

  NvmeStatus ValidateIoArgs(const int ns_index,    //
                            const uint64_t lba,    //
                            const int32_t blocks,  //
                            const void* const buf) const {
    if (state_ != NvmeState::kRunning) {
      return NvmeStatus::kNotRunning;
    }
    if (ns_index < 0 || ns_index >= namespace_count_ ||
        !namespaces_[ns_index].valid) {
      return NvmeStatus::kInvalidArgument;
    }
    const NamespaceInfo& info = namespaces_[ns_index];
    if (blocks < 0 || blocks > info.max_transfer_blocks ||
        lba > info.block_count ||
        static_cast<uint64_t>(blocks) > info.block_count - lba ||
        (blocks > 0 && buf == nullptr)) {
      return NvmeStatus::kInvalidArgument;
    }
    return NvmeStatus::kOk;
  }

  Platform* platform_ = nullptr;
  MmioWindow* mmio_ = nullptr;
  uint8_t bus_ = 0;
  uint8_t device_ = 0;
  uint8_t function_ = 0;

  NvmeState state_ = NvmeState::kUninitialized;
  int dstrd_ = 0;
  int admin_depth_ = 0;
  int io_depth_ = 0;
  int64_t enable_timeout_ms_ = 2000;
  uint64_t irq_mask_ = 0;
  IrqMode irq_mode_ = IrqMode::kMsix;

  DmaBuffer admin_sq_{};
  DmaBuffer admin_cq_{};
  DmaBuffer io_sq_{};
  DmaBuffer io_cq_{};
  DmaBuffer identify_buf_{};
  DmaBuffer io_buf_{};

  int admin_sq_tail_ = 0;
  int admin_cq_head_ = 0;
  uint16_t admin_cq_phase_ = 1;
  int io_sq_tail_ = 0;
  int io_cq_head_ = 0;
  uint16_t io_cq_phase_ = 1;
  uint16_t next_cid_ = 1;

  uint32_t nn_ = 0;
  int64_t max_transfer_bytes_ = kIoMaxBytes;
  bool has_vwc_ = false;
  char model_[kBlockModelCapacity] = {};
  char serial_[kBlockSerialCapacity] = {};
  NamespaceInfo namespaces_[kMaxNamespaces]{};
  int namespace_count_ = 0;
};

static_assert(std::is_trivially_destructible_v<NvmeController>);

}  // namespace protos::nvme

#endif  // PROTOS_USER_NVME_NVME_CONTROLLER_H_
