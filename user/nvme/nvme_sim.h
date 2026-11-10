#ifndef PROTOS_USER_NVME_NVME_SIM_H_
#define PROTOS_USER_NVME_NVME_SIM_H_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <new>
#include <string>
#include <utility>
#include <vector>

#include "user/nvme/nvme_platform.h"
#include "user/nvme/nvme_regs.h"

namespace protos::nvme::sim {

// Host-side simulated NVMe controller + DMA memory space for unit testing
// `NvmeController` and `BlockServer` without real hardware.
class SimNvmeDevice final : public Platform, public MmioWindow {
 public:
  struct NamespaceConfig {
    uint32_t nsid = 1;
    uint64_t block_count = 64;
    int32_t block_size = 512;
    uint16_t metadata_size = 0;
    bool read_only = false;
    uint8_t nlbaf = 0;
    uint8_t flbas_index = 0;
    uint8_t raw_flbas_reserved = 0;
    uint8_t lbads_override = 0;
    bool advertise_nsattr_ro = true;
    std::vector<uint8_t> storage;

    void InitStorage() {
      const uint64_t total = block_count * static_cast<uint64_t>(block_size);
      storage.assign(total, 0);
    }
  };

  SimNvmeDevice() {
    ResetRegisters(/*initially_enabled=*/false);
    NamespaceConfig ns;
    ns.nsid = 1;
    ns.block_count = 128;
    ns.block_size = 512;
    ns.InitStorage();
    namespaces.push_back(std::move(ns));
  }

  void ResetRegisters(const bool initially_enabled) {
    // MQES = 63 (64 entries), TO = 2 (1000ms), DSTRD = 0, CSS = NVM, MPSMIN =
    // 0, MPSMAX = 0.
    cap = (63ULL << 0) | (2ULL << 24) | kCapCssNvm;
    vs = 0x00010400u;
    cc = initially_enabled ? kCcEn : 0u;
    csts = initially_enabled ? kCstsRdy : 0u;
    intms = 0;
    aqa = 0;
    asq = 0;
    acq = 0;
    asqs = 0;
    acqs = 0;
    asq_head = 0;
    acq_tail = 0;
    acq_head_seen = 0;
    acq_head_doorbell_writes = 0;
    acq_phase = 1;
    io_cq_created = false;
    io_sq_created = false;
    io_cq_phys = 0;
    io_sq_phys = 0;
    io_cq_size = 0;
    io_sq_size = 0;
    io_sq_head = 0;
    io_cq_tail = 0;
    io_cq_head_seen = 0;
    io_cq_head_doorbell_writes = 0;
    io_cq_phase = 1;
    flush_count = 0;
    set_features_count = 0;
    admin_commands_seen = 0;
    io_commands_seen = 0;
    intms_writes_seen = 0;
    intmc_writes_seen = 0;
    saw_shn_normal = false;
    saw_shn_abrupt = false;
    saw_cc_zero_after_shn = false;
    shutdown_count = 0;
    wait_for_irq_calls = 0;
    wait_for_irq_ticks_total = 0;
    last_wait_irq_mask = 0;
    last_wait_timeout_ticks = 0;
    disable_transitions_left = initially_enabled ? disable_delay_ticks : 0;
    enable_transitions_left = enable_delay_ticks;
    shutdown_transitions_left = 0;
    pending_delay_ticks = 0;
    delayed_action = {};
  }

  void SetSizeAndHole(const int64_t size,        //
                      const int64_t hole_start,  //
                      const int64_t hole_end) {
    mmio_size_ = size;
    hole_start_ = hole_start;
    hole_end_ = hole_end;
  }

  DmaBuffer DmaAlloc(const int64_t size, const bool below_4gib) override {
    constexpr int64_t kMaxSimDmaBytes = 1LL << 20;
    if (fail_dma_alloc || size <= 0 || (size & 0xFFFLL) != 0 ||
        size > kMaxSimDmaBytes) {
      return {};
    }
    if (dma_alloc_limit > 0 && dma_allocs_done >= dma_alloc_limit) {
      return {};
    }
    std::unique_ptr<uint8_t[], AlignedDelete> storage(static_cast<uint8_t*>(
        ::operator new[](static_cast<size_t>(size), std::align_val_t{4096})));
    std::memset(storage.get(), 0, static_cast<size_t>(size));
    // Place buffers above 4 GiB by default when `below_4gib` is false to verify
    // 64-bit DMA address handling end to end.
    const uint64_t phys = below_4gib ? next_low_phys_ : next_high_phys_;
    if (below_4gib) {
      next_low_phys_ += static_cast<uint64_t>(size) + 0x1000ULL;
    } else {
      next_high_phys_ += static_cast<uint64_t>(size) + 0x1000ULL;
    }
    void* const va = storage.get();
    dma_regions_[phys] = DmaRegion{std::move(storage), size};
    ++dma_allocs_done;
    const uint64_t returned_phys =
        force_unaligned_dma_phys ? (phys + 4ULL) : phys;
    return DmaBuffer{va, returned_phys, size};
  }

  int64_t NowTicks() override {
    if (on_now_ticks) {
      on_now_ticks();
    }
    return now_ticks;
  }

  int64_t TicksPerSecond() override { return ticks_per_second; }
  void Relax() override { StepTimeAndTransitions(1); }

  void WaitForIrq(const uint64_t irq_mask,
                  const int64_t timeout_ticks) override {
    if (timeout_ticks <= 0) {
      return;
    }
    ++wait_for_irq_calls;
    last_wait_irq_mask = irq_mask;
    last_wait_timeout_ticks = timeout_ticks;
    const bool irq_enabled = irq_mask != 0 && (!msi_mode || intms == 0);
    if (pending_delay_ticks > 0) {
      if (pending_delay_ticks <= timeout_ticks) {
        const int64_t adv = irq_enabled ? pending_delay_ticks : timeout_ticks;
        pending_delay_ticks = 0;
        StepTimeAndTransitions(adv);
        wait_for_irq_ticks_total += adv;
        if (delayed_action) {
          auto fn = std::move(delayed_action);
          delayed_action = {};
          fn();
        }
        return;
      }
      pending_delay_ticks -= timeout_ticks;
    }
    StepTimeAndTransitions(timeout_ticks);
    wait_for_irq_ticks_total += timeout_ticks;
  }

  void Log(const char* const line, const int length) override {
    if (line != nullptr && length > 0) {
      logs.emplace_back(line, static_cast<size_t>(length));
    }
  }

  int64_t Size() const override { return mmio_size_; }

  bool Covers(const int64_t offset, const int64_t size) const override {
    if (mmio_size_ <= 0 || offset < 0 || size <= 0 || size > mmio_size_ ||
        offset > mmio_size_ - size) {
      return false;
    }
    if (hole_end_ > hole_start_ && offset < hole_end_ &&
        offset + size > hole_start_) {
      return false;
    }
    return true;
  }

  uint32_t Read32(const int64_t offset) override {
    uint32_t value = kMmioAllOnes;
    if (Covers(offset, 4) && (offset & 3) == 0 && !not_decoding) {
      switch (offset) {
        case kRegCap:
          value = static_cast<uint32_t>(cap);
          break;
        case kRegCap + 4:
          value = static_cast<uint32_t>(cap >> 32);
          break;
        case kRegVs:
          value = vs;
          break;
        case kRegIntms:
        case kRegIntmc:
          value = intms;
          break;
        case kRegCc:
          value = cc;
          break;
        case kRegCsts:
          value = csts;
          break;
        case kRegAqa:
          value = aqa;
          break;
        case kRegAsq:
          value = static_cast<uint32_t>(asq);
          break;
        case kRegAsq + 4:
          value = static_cast<uint32_t>(asq >> 32);
          break;
        case kRegAcq:
          value = static_cast<uint32_t>(acq);
          break;
        case kRegAcq + 4:
          value = static_cast<uint32_t>(acq >> 32);
          break;
        default:
          value = 0;
          break;
      }
    }
    if (on_read32) {
      on_read32(offset, value);
    }
    return value;
  }

  void Write32(const int64_t offset, const uint32_t value) override {
    if (!Covers(offset, 4) || (offset & 3) != 0 || not_decoding) {
      return;
    }
    switch (offset) {
      case kRegIntms:
        intms |= value;
        ++intms_writes_seen;
        return;
      case kRegIntmc:
        intms &= ~value;
        ++intmc_writes_seen;
        return;
      case kRegCc:
        HandleWriteCc(value);
        return;
      case kRegAqa:
        aqa = value;
        asqs = static_cast<int>(value & 0xFFFu) + 1;
        acqs = static_cast<int>((value >> 16) & 0xFFFu) + 1;
        return;
      case kRegAsq:
        asq = (asq & ~0xFFFFFFFFULL) | value;
        return;
      case kRegAsq + 4:
        asq = (asq & 0xFFFFFFFFULL) | (static_cast<uint64_t>(value) << 32);
        return;
      case kRegAcq:
        acq = (acq & ~0xFFFFFFFFULL) | value;
        return;
      case kRegAcq + 4:
        acq = (acq & 0xFFFFFFFFULL) | (static_cast<uint64_t>(value) << 32);
        return;
      default:
        break;
    }
    if (offset == SqTailDoorbellOffset(0, CapDstrd(cap))) {
      ProcessAdminDoorbell(static_cast<int>(value));
    } else if (offset == CqHeadDoorbellOffset(0, CapDstrd(cap))) {
      acq_head_seen = static_cast<int>(value);
      ++acq_head_doorbell_writes;
    } else if (offset == SqTailDoorbellOffset(1, CapDstrd(cap))) {
      ProcessIoDoorbell(static_cast<int>(value));
    } else if (offset == CqHeadDoorbellOffset(1, CapDstrd(cap))) {
      io_cq_head_seen = static_cast<int>(value);
      ++io_cq_head_doorbell_writes;
    }
  }

  uint8_t* PhysToPtr(const uint64_t phys, const int64_t needed) {
    if (phys == 0 || needed <= 0) {
      return nullptr;
    }
    const uint64_t u_needed = static_cast<uint64_t>(needed);
    for (auto& [base, reg] : dma_regions_) {
      if (reg.size <= 0 || u_needed > static_cast<uint64_t>(reg.size)) {
        continue;
      }
      if (phys >= base &&
          phys - base <= static_cast<uint64_t>(reg.size) - u_needed) {
        return reg.bytes.get() + (phys - base);
      }
    }
    return nullptr;
  }

  uint8_t* Prp1ToPtr(const uint64_t prp1,   //
                     const int64_t needed,  //
                     const bool require_page_aligned) {
    if (prp1 == 0 || needed <= 0) {
      return nullptr;
    }
    const uint64_t page_off = prp1 & 0xFFFULL;
    if (require_page_aligned && page_off != 0) {
      return nullptr;
    }
    if (static_cast<uint64_t>(needed) > 4096ULL - page_off) {
      return nullptr;
    }
    return PhysToPtr(prp1, needed);
  }

  std::string model = "SimNVMe 1.4";
  std::string serial = "SIM0001";
  std::vector<NamespaceConfig> namespaces;
  uint32_t nn_override = 0;  // 0 => `namespaces.size()` (or max nsid)
  uint8_t mdts = 5;
  bool support_active_ns_list = true;
  bool corrupt_cns2_nonmonotonic = false;
  bool not_decoding = false;
  bool fail_dma_alloc = false;
  bool force_unaligned_dma_phys = false;
  bool hang_disable = false;
  bool hang_enable = false;
  bool hang_shutdown = false;
  bool hang_admin_cmd = false;
  bool hang_io_cmd = false;
  bool fatal_on_enable = false;
  bool fatal_on_io = false;
  bool corrupt_cqe_cid = false;
  bool corrupt_cqe_sq_id = false;
  bool fail_flush = false;
  bool msi_mode = false;
  uint16_t inject_next_io_status = 0;
  uint32_t set_features_num_queues_dw0 = 0;

  uint64_t cap = 0;
  uint32_t vs = 0;
  uint32_t cc = 0;
  uint32_t csts = 0;
  uint32_t intms = 0;
  uint32_t aqa = 0;
  uint64_t asq = 0;
  uint64_t acq = 0;
  int asqs = 0;
  int acqs = 0;
  int asq_head = 0;
  int acq_tail = 0;
  int acq_head_seen = 0;
  int acq_head_doorbell_writes = 0;
  uint16_t acq_phase = 1;
  bool io_cq_created = false;
  bool io_sq_created = false;
  uint64_t io_cq_phys = 0;
  uint64_t io_sq_phys = 0;
  int io_cq_size = 0;
  int io_sq_size = 0;
  int io_sq_head = 0;
  int io_cq_tail = 0;
  int io_cq_head_seen = 0;
  int io_cq_head_doorbell_writes = 0;
  uint16_t io_cq_phase = 1;
  int flush_count = 0;
  int set_features_count = 0;
  int admin_commands_seen = 0;
  int io_commands_seen = 0;
  int intms_writes_seen = 0;
  int intmc_writes_seen = 0;
  bool saw_shn_normal = false;
  bool saw_shn_abrupt = false;
  bool saw_cc_zero_after_shn = false;
  int shutdown_count = 0;
  int dma_alloc_limit = 0;
  int dma_allocs_done = 0;
  int64_t ticks_per_second = 250;
  int64_t now_ticks = 0;
  int disable_delay_ticks = 2;
  int enable_delay_ticks = 1;
  int shutdown_delay_ticks = 2;
  int64_t async_admin_delay_ticks = 0;
  int64_t async_io_delay_ticks = 0;
  int wait_for_irq_calls = 0;
  int64_t wait_for_irq_ticks_total = 0;
  uint64_t last_wait_irq_mask = 0;
  int64_t last_wait_timeout_ticks = 0;
  int64_t pending_delay_ticks = 0;
  std::function<void()> delayed_action;
  std::function<void()> on_now_ticks;
  std::function<void(int64_t offset, uint32_t value)> on_read32;
  std::vector<std::string> logs;

 private:
  struct AlignedDelete {
    void operator()(uint8_t* const ptr) const {
      ::operator delete[](ptr, std::align_val_t{4096});
    }
  };

  struct DmaRegion {
    std::unique_ptr<uint8_t[], AlignedDelete> bytes;
    int64_t size = 0;
  };

  bool ValidateEnableConfig() {
    const uint32_t css = cc & (0x7u << 4);
    const uint32_t mps = cc & (0xFu << 7);
    const uint32_t iosqes = cc & (0xFu << 16);
    const uint32_t iocqes = cc & (0xFu << 20);
    if (css != kCcCssNvm || mps != kCcMps4K || iosqes != kCcIoSqes64 ||
        iocqes != kCcIoCqes16) {
      return false;
    }
    const int mqes = CapMqes(cap);
    if (asqs < 2 || asqs > mqes || acqs < 2 || acqs > mqes) {
      return false;
    }
    const int64_t asq_bytes = static_cast<int64_t>(asqs) *
                              static_cast<int64_t>(sizeof(SubmissionEntry));
    const int64_t acq_bytes = static_cast<int64_t>(acqs) *
                              static_cast<int64_t>(sizeof(CompletionEntry));
    return Prp1ToPtr(asq, asq_bytes, /*require_page_aligned=*/true) !=
               nullptr &&
           Prp1ToPtr(acq, acq_bytes, /*require_page_aligned=*/true) != nullptr;
  }

  void StepTimeAndTransitions(const int64_t dt) {
    now_ticks = static_cast<int64_t>(static_cast<uint64_t>(now_ticks) +
                                     static_cast<uint64_t>(dt));
    if ((cc & kCcEn) != 0 && (cc & kCcShnMask) != 0 &&
        (csts & kCstsShstMask) == kCstsShstOccurring && !hang_shutdown) {
      if (shutdown_transitions_left > dt) {
        shutdown_transitions_left -= static_cast<int>(dt);
      } else {
        shutdown_transitions_left = 0;
        csts = (csts & ~kCstsShstMask) | kCstsShstComplete;
        ++shutdown_count;
      }
    }
    if ((cc & kCcEn) == 0 &&
        (csts & (kCstsRdy | kCstsCfs | kCstsShstMask)) != 0 && !hang_disable) {
      if (disable_transitions_left > dt) {
        disable_transitions_left -= static_cast<int>(dt);
      } else {
        disable_transitions_left = 0;
        csts &= ~(kCstsRdy | kCstsCfs | kCstsShstMask);
        intms = 0;
        asq_head = 0;
        acq_tail = 0;
        acq_phase = 1;
        io_sq_head = 0;
        io_cq_tail = 0;
        io_cq_phase = 1;
        io_cq_created = false;
        io_sq_created = false;
        io_cq_phys = 0;
        io_sq_phys = 0;
        io_cq_size = 0;
        io_sq_size = 0;
      }
    }
    if ((cc & kCcEn) != 0 && (csts & kCstsRdy) == 0 && (csts & kCstsCfs) == 0 &&
        !hang_enable) {
      if (enable_transitions_left > dt) {
        enable_transitions_left -= static_cast<int>(dt);
      } else {
        enable_transitions_left = 0;
        if (fatal_on_enable || !ValidateEnableConfig()) {
          csts |= kCstsCfs;
        } else {
          csts |= kCstsRdy;
        }
      }
    }
  }

  void HandleWriteCc(const uint32_t value) {
    const bool was_en = (cc & kCcEn) != 0;
    const bool now_en = (value & kCcEn) != 0;
    const uint32_t prev_shn = cc & kCcShnMask;
    const uint32_t new_shn = value & kCcShnMask;
    cc = value;
    if (was_en && !now_en) {
      if ((prev_shn != 0 || (csts & kCstsShstMask) == kCstsShstComplete) &&
          value == 0) {
        saw_cc_zero_after_shn = true;
      }
      disable_transitions_left =
          (disable_delay_ticks > 0) ? disable_delay_ticks : 1;
    } else if (!was_en && now_en) {
      enable_transitions_left =
          (enable_delay_ticks > 0) ? enable_delay_ticks : 1;
    }
    if (now_en && new_shn != 0 && prev_shn == 0) {
      if (new_shn == kCcShnNormal) {
        saw_shn_normal = true;
      } else if (new_shn == kCcShnAbrupt) {
        saw_shn_abrupt = true;
      }
      csts = (csts & ~kCstsShstMask) | kCstsShstOccurring;
      shutdown_transitions_left =
          (shutdown_delay_ticks > 0) ? shutdown_delay_ticks : 1;
    }
  }

  static void WriteLe16(uint8_t* const p, const uint16_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
  }

  static void WriteLe32(uint8_t* const p, const uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
  }

  static void WriteLe64(uint8_t* const p, const uint64_t v) {
    WriteLe32(p, static_cast<uint32_t>(v));
    WriteLe32(p + 4, static_cast<uint32_t>(v >> 32));
  }

  static uint16_t MakeStatus(const int sct, const int sc) {
    return static_cast<uint16_t>(((sct & 0x7) << 9) | ((sc & 0xFF) << 1));
  }

  void PostCqe(const uint64_t cq_base,  //
               const int cq_size,       //
               int& cq_tail,            //
               uint16_t& cq_phase,      //
               const uint16_t sq_id,    //
               const int sq_head,       //
               const uint16_t cid,      //
               const uint32_t dw0,      //
               const uint16_t status_field) {
    CompletionEntry* const cq = reinterpret_cast<CompletionEntry*>(
        Prp1ToPtr(cq_base,                                                  //
                  cq_size * static_cast<int64_t>(sizeof(CompletionEntry)),  //
                  /*require_page_aligned=*/true));
    if (cq == nullptr) {
      csts |= kCstsCfs;
      return;
    }
    CompletionEntry cqe{};
    cqe.dw0 = dw0;
    cqe.sq_head = static_cast<uint16_t>(sq_head);
    cqe.sq_id = corrupt_cqe_sq_id ? static_cast<uint16_t>(sq_id ^ 1u) : sq_id;
    cqe.cid = corrupt_cqe_cid ? static_cast<uint16_t>(cid ^ 0xFFu) : cid;
    cqe.status = static_cast<uint16_t>(status_field | cq_phase);
    cq[cq_tail] = cqe;
    cq_tail = (cq_tail + 1) % cq_size;
    if (cq_tail == 0) {
      cq_phase ^= 1u;
    }
  }

  uint16_t ExecAdminIdentify(const SubmissionEntry& cmd) {
    uint8_t* const out =
        Prp1ToPtr(cmd.prp1, kIdentifyDataBytes, /*require_page_aligned=*/true);
    if (out == nullptr) {
      return MakeStatus(kStatusTypeGeneric, kStatusCodeInvalidField);
    }
    std::memset(out, 0, kIdentifyDataBytes);
    const uint32_t cns = cmd.cdw10 & 0xFFu;
    if (cns == kIdentifyCnsController) {
      WriteLe16(out, 0x1B36);
      WriteLe16(out + 2, 0x0010);
      std::memset(out + kIdCtrlSerialOffset, ' ', kIdCtrlSerialLength);
      const int serial_len =
          (static_cast<int64_t>(serial.size()) < kIdCtrlSerialLength)
              ? static_cast<int>(serial.size())
              : kIdCtrlSerialLength;
      if (serial_len > 0) {
        std::memcpy(out + kIdCtrlSerialOffset, serial.data(), serial_len);
      }
      std::memset(out + kIdCtrlModelOffset, ' ', kIdCtrlModelLength);
      const int model_len =
          (static_cast<int64_t>(model.size()) < kIdCtrlModelLength)
              ? static_cast<int>(model.size())
              : kIdCtrlModelLength;
      if (model_len > 0) {
        std::memcpy(out + kIdCtrlModelOffset, model.data(), model_len);
      }
      out[kIdCtrlMdtsOffset] = mdts;
      uint32_t max_nsid = 0;
      for (const NamespaceConfig& ns : namespaces) {
        if (ns.nsid != 0xFFFFFFFFu && ns.nsid > max_nsid) {
          max_nsid = ns.nsid;
        }
      }
      WriteLe32(out + kIdCtrlNnOffset,
                nn_override > 0 ? nn_override : max_nsid);
      out[kIdCtrlVwcOffset] = 1;
      return 0;
    }
    if (cns == kIdentifyCnsActiveNsList) {
      if (!support_active_ns_list || cmd.nsid == 0xFFFFFFFFu) {
        return MakeStatus(kStatusTypeGeneric, kStatusCodeInvalidField);
      }
      if (corrupt_cns2_nonmonotonic) {
        for (int i = 0; i < 1024; ++i) {
          WriteLe32(out + 4 * i, (i == 0) ? 10u : 5u);
        }
        return 0;
      }
      std::vector<uint32_t> active;
      for (const NamespaceConfig& ns : namespaces) {
        if (ns.nsid != 0 && ns.nsid != 0xFFFFFFFFu && ns.nsid > cmd.nsid) {
          active.push_back(ns.nsid);
        }
      }
      std::sort(active.begin(), active.end());
      int idx = 0;
      uint32_t prev = 0;
      for (const uint32_t id : active) {
        if (id == prev) {
          continue;
        }
        if (idx >= 1024) {
          break;
        }
        WriteLe32(out + 4 * idx, id);
        prev = id;
        ++idx;
      }
      return 0;
    }
    if (cns == kIdentifyCnsNamespace) {
      if (cmd.nsid == 0 || cmd.nsid == 0xFFFFFFFFu) {
        return MakeStatus(kStatusTypeGeneric, kStatusCodeInvalidField);
      }
      for (const NamespaceConfig& ns : namespaces) {
        if (ns.nsid == cmd.nsid) {
          WriteLe64(out + kIdNsSizeOffset, ns.block_count);
          WriteLe64(out + kIdNsCapOffset, ns.block_count);
          out[kIdNsNlbafOffset] = ns.nlbaf;
          uint8_t flbas = static_cast<uint8_t>(ns.flbas_index & 0xFu);
          if (ns.nlbaf >= 16) {
            flbas |= static_cast<uint8_t>(((ns.flbas_index >> 4) & 0x3u) << 5);
          }
          flbas |= ns.raw_flbas_reserved;
          out[kIdNsFlbasOffset] = flbas;
          out[kIdNsAttrOffset] = (ns.read_only && ns.advertise_nsattr_ro)
                                     ? kIdNsAttrWriteProtected
                                     : 0;
          int lbads = 0;
          if (ns.lbads_override > 0) {
            lbads = static_cast<int>(ns.lbads_override);
          } else {
            int32_t b = ns.block_size;
            while (b > 1) {
              b >>= 1;
              ++lbads;
            }
          }
          const uint32_t lbaf = static_cast<uint32_t>(ns.metadata_size) |
                                (static_cast<uint32_t>(lbads) << 16);
          const int fmt_slot = static_cast<int>(ns.flbas_index & 0x3Fu);
          WriteLe32(out + kIdNsLbaf0Offset + 4 * fmt_slot, lbaf);
          return 0;
        }
      }
      return MakeStatus(kStatusTypeGeneric, kStatusCodeInvalidField);
    }
    return MakeStatus(kStatusTypeGeneric, kStatusCodeInvalidField);
  }

  void ProcessAdminDoorbell(const int new_tail) {
    if ((cc & kCcEn) == 0 || (csts & kCstsRdy) == 0 ||
        (csts & (kCstsCfs | kCstsShstMask)) != 0 || asqs <= 0 || acqs <= 0) {
      return;
    }
    if (new_tail < 0 || new_tail >= asqs) {
      csts |= kCstsCfs;
      return;
    }
    const SubmissionEntry* const sq = reinterpret_cast<const SubmissionEntry*>(
        Prp1ToPtr(asq,                                                   //
                  asqs * static_cast<int64_t>(sizeof(SubmissionEntry)),  //
                  /*require_page_aligned=*/true));
    if (sq == nullptr) {
      csts |= kCstsCfs;
      return;
    }
    while (asq_head != new_tail) {
      const SubmissionEntry cmd = sq[asq_head];
      asq_head = (asq_head + 1) % asqs;
      ++admin_commands_seen;
      if (hang_admin_cmd) {
        continue;
      }
      const uint8_t opc = static_cast<uint8_t>(cmd.cdw0 & 0xFFu);
      const uint16_t cid = static_cast<uint16_t>(cmd.cdw0 >> 16);
      uint32_t dw0 = 0;
      uint16_t status_field = 0;
      if (opc == kAdminIdentify) {
        status_field = ExecAdminIdentify(cmd);
      } else if (opc == kAdminSetFeatures) {
        ++set_features_count;
        if ((cmd.cdw10 & 0xFFu) == kFeatureNumberOfQueues) {
          dw0 = set_features_num_queues_dw0;
        }
      } else if (opc == kAdminCreateIoCq) {
        const int cq_size = static_cast<int>((cmd.cdw10 >> 16) & 0xFFFFu) + 1;
        const uint32_t qid = cmd.cdw10 & 0xFFFFu;
        const int64_t cq_bytes = static_cast<int64_t>(cq_size) *
                                 static_cast<int64_t>(sizeof(CompletionEntry));
        if (qid != 1u || cq_size < 2 || cq_size > CapMqes(cap) ||
            (cmd.cdw11 & kQueuePhysicallyContiguous) == 0 ||
            Prp1ToPtr(cmd.prp1, cq_bytes, /*require_page_aligned=*/true) ==
                nullptr) {
          status_field =
              MakeStatus(kStatusTypeGeneric, kStatusCodeInvalidField);
        } else {
          io_cq_phys = cmd.prp1;
          io_cq_size = cq_size;
          io_cq_tail = 0;
          io_cq_phase = 1;
          io_cq_created = true;
        }
      } else if (opc == kAdminCreateIoSq) {
        const int sq_size = static_cast<int>((cmd.cdw10 >> 16) & 0xFFFFu) + 1;
        const uint32_t qid = cmd.cdw10 & 0xFFFFu;
        const uint32_t cqid = (cmd.cdw11 >> 16) & 0xFFFFu;
        const int64_t sq_bytes = static_cast<int64_t>(sq_size) *
                                 static_cast<int64_t>(sizeof(SubmissionEntry));
        if (!io_cq_created || qid != 1u || cqid != 1u || sq_size < 2 ||
            sq_size > CapMqes(cap) ||
            (cmd.cdw11 & kQueuePhysicallyContiguous) == 0 ||
            Prp1ToPtr(cmd.prp1, sq_bytes, /*require_page_aligned=*/true) ==
                nullptr) {
          status_field =
              MakeStatus(kStatusTypeGeneric, kStatusCodeInvalidField);
        } else {
          io_sq_phys = cmd.prp1;
          io_sq_size = sq_size;
          io_sq_head = 0;
          io_sq_created = true;
        }
      } else {
        status_field = MakeStatus(kStatusTypeGeneric, 0x01);  // Invalid Opcode
      }
      if (async_admin_delay_ticks > 0) {
        const int sq_head_snap = asq_head;
        pending_delay_ticks = async_admin_delay_ticks;
        delayed_action = [this, sq_head_snap, cid, dw0, status_field]() {
          PostCqe(acq,           //
                  acqs,          //
                  acq_tail,      //
                  acq_phase,     //
                  0,             //
                  sq_head_snap,  //
                  cid,           //
                  dw0,           //
                  status_field);
        };
      } else {
        PostCqe(acq,        //
                acqs,       //
                acq_tail,   //
                acq_phase,  //
                0,          //
                asq_head,   //
                cid,        //
                dw0,        //
                status_field);
      }
    }
  }

  NamespaceConfig* FindNamespace(const uint32_t nsid) {
    for (NamespaceConfig& ns : namespaces) {
      if (ns.nsid == nsid) {
        return &ns;
      }
    }
    return nullptr;
  }

  uint16_t ExecIoCommand(const SubmissionEntry& cmd) {
    if (inject_next_io_status != 0) {
      const uint16_t injected = inject_next_io_status;
      inject_next_io_status = 0;
      return injected;
    }
    NamespaceConfig* const ns = FindNamespace(cmd.nsid);
    if (ns == nullptr) {
      return MakeStatus(kStatusTypeGeneric, kStatusCodeInvalidField);
    }
    const uint8_t opc = static_cast<uint8_t>(cmd.cdw0 & 0xFFu);
    if (opc == kNvmFlush) {
      ++flush_count;
      if (fail_flush) {
        return MakeStatus(kStatusTypeGeneric, 0x06);
      }
      return 0;
    }
    const uint64_t lba = static_cast<uint64_t>(cmd.cdw10) |
                         (static_cast<uint64_t>(cmd.cdw11) << 32);
    const int64_t blocks = static_cast<int64_t>(cmd.cdw12 & 0xFFFFu) + 1;
    if (lba > ns->block_count ||
        static_cast<uint64_t>(blocks) > ns->block_count - lba) {
      return MakeStatus(kStatusTypeGeneric, kStatusCodeLbaOutOfRange);
    }
    if (ns->block_size <= 0 ||
        blocks > INT64_MAX / static_cast<int64_t>(ns->block_size)) {
      return MakeStatus(kStatusTypeGeneric, kStatusCodeInvalidField);
    }
    const int64_t bytes = blocks * ns->block_size;
    uint8_t* const dma =
        Prp1ToPtr(cmd.prp1, bytes, /*require_page_aligned=*/false);
    if (dma == nullptr) {
      return MakeStatus(kStatusTypeGeneric, kStatusCodeInvalidField);
    }
    if (lba > UINT64_MAX / static_cast<uint64_t>(ns->block_size)) {
      return MakeStatus(kStatusTypeMedia, 0x81);
    }
    const uint64_t byte_offset = lba * static_cast<uint64_t>(ns->block_size);
    const uint64_t storage_bytes = static_cast<uint64_t>(ns->storage.size());
    if (byte_offset > storage_bytes ||
        static_cast<uint64_t>(bytes) > storage_bytes - byte_offset) {
      return MakeStatus(kStatusTypeMedia, 0x81);
    }
    if (opc == kNvmRead) {
      std::memcpy(dma, ns->storage.data() + byte_offset, bytes);
      return 0;
    }
    if (opc == kNvmWrite) {
      if (ns->read_only) {
        return MakeStatus(kStatusTypeCommandSpecific,
                          kStatusCodeWriteToReadOnlyRange);
      }
      std::memcpy(ns->storage.data() + byte_offset, dma, bytes);
      return 0;
    }
    return MakeStatus(kStatusTypeGeneric, 0x01);
  }

  void ProcessIoDoorbell(const int new_tail) {
    if ((cc & kCcEn) == 0 || (csts & kCstsRdy) == 0 ||
        (csts & (kCstsCfs | kCstsShstMask)) != 0 || !io_sq_created ||
        !io_cq_created || io_sq_size <= 0 || io_cq_size <= 0) {
      return;
    }
    if (new_tail < 0 || new_tail >= io_sq_size) {
      csts |= kCstsCfs;
      return;
    }
    const SubmissionEntry* const sq = reinterpret_cast<const SubmissionEntry*>(
        Prp1ToPtr(io_sq_phys,
                  io_sq_size * static_cast<int64_t>(sizeof(SubmissionEntry)),
                  /*require_page_aligned=*/true));
    if (sq == nullptr) {
      csts |= kCstsCfs;
      return;
    }
    while (io_sq_head != new_tail) {
      const SubmissionEntry cmd = sq[io_sq_head];
      io_sq_head = (io_sq_head + 1) % io_sq_size;
      ++io_commands_seen;
      if (fatal_on_io) {
        csts |= kCstsCfs;
        return;
      }
      if (hang_io_cmd) {
        continue;
      }
      const uint16_t cid = static_cast<uint16_t>(cmd.cdw0 >> 16);
      const uint16_t status_field = ExecIoCommand(cmd);
      if (async_io_delay_ticks > 0) {
        const int sq_head_snap = io_sq_head;
        pending_delay_ticks = async_io_delay_ticks;
        delayed_action = [this, sq_head_snap, cid, status_field]() {
          PostCqe(io_cq_phys,    //
                  io_cq_size,    //
                  io_cq_tail,    //
                  io_cq_phase,   //
                  1,             //
                  sq_head_snap,  //
                  cid,           //
                  0,             //
                  status_field);
        };
      } else {
        PostCqe(io_cq_phys,   //
                io_cq_size,   //
                io_cq_tail,   //
                io_cq_phase,  //
                1,            //
                io_sq_head,   //
                cid,          //
                0,            //
                status_field);
      }
    }
  }

  int64_t mmio_size_ = 0x4000;
  int64_t hole_start_ = 0x2000;
  int64_t hole_end_ = 0x4000;
  uint64_t next_low_phys_ = 0x00100000ULL;
  uint64_t next_high_phys_ = 0x200000000ULL;
  std::map<uint64_t, DmaRegion> dma_regions_;
  int disable_transitions_left = 0;
  int enable_transitions_left = 0;
  int shutdown_transitions_left = 0;
};

}  // namespace protos::nvme::sim

#endif  // PROTOS_USER_NVME_NVME_SIM_H_
