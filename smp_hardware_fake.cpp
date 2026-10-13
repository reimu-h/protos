#include "smp_hardware_fake.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <bitset>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <utility>

#include "idt.h"
#include "interrupts.h"
#include "smp.h"

namespace protos {
namespace {

constexpr uint32_t kMsrApicBase = 0x1B;
constexpr uint32_t kMsrX2ApicFirst = 0x800;
constexpr uint32_t kMsrX2ApicLast = 0x8FF;
constexpr uint32_t kMsrEfer = 0xC0000080;
constexpr uint64_t kEferLongMode = (1ULL << 8) | (1ULL << 10);
constexpr uint64_t kApicBaseEnable = 1ULL << 11;
constexpr uint64_t kApicBaseX2Apic = 1ULL << 10;

constexpr int kLapicId = 0x020;
constexpr int kLapicEoi = 0x0B0;
constexpr int kLapicSvr = 0x0F0;
constexpr int kLapicIcrLow = 0x300;
constexpr int kLapicIcrHigh = 0x310;
constexpr int kLapicTimerInitCount = 0x380;
constexpr int kLapicTimerCurrCount = 0x390;
constexpr uint32_t kIcrSendPending = 1u << 12;
constexpr uint32_t kIcrDeliveryModeMask = 0x700;
constexpr uint32_t kIcrFixed = 0x000;
constexpr uint32_t kIcrInit = 0x500;
constexpr uint32_t kIcrStartup = 0x600;
constexpr uint32_t kIcrLevelAssert = 1u << 14;
constexpr uint32_t kIcrShorthandMask = 3u << 18;

constexpr uint16_t kPitChannel2Port = 0x42;
constexpr uint16_t kPitCommandPort = 0x43;
constexpr uint16_t kPitSpeakerPort = 0x61;
// Channel 2, low byte then high byte, mode 0, binary.
constexpr uint8_t kPitChannel2Mode0 = 0xB0;
constexpr uint8_t kPitGate2 = 1u << 0;
constexpr uint8_t kPitOut2 = 1u << 5;
constexpr uint64_t kPitHz = 1193182;

// What the fake trampoline install writes to physical memory.
constexpr int kTrampolineImageBytes = 64;
constexpr uint8_t kTrampolineImageByte = 0xF4;

constexpr uintptr_t kGuardPageBytes = 4096;

// How far the TSC jumps in a SpinPause() while no AP can make progress.
constexpr uint64_t kIdleSpinStepNs = 1000000;

// The CPU that the calling thread is, if the fake started it.
thread_local const FakeSmpHardware* t_ap_owner = nullptr;
thread_local uint32_t t_ap_apic_id = 0;

static bool IsX2ApicMsr(const uint32_t msr) {
  return msr >= kMsrX2ApicFirst && msr <= kMsrX2ApicLast;
}

}  // namespace

struct FakeSmpHardware::Cpu {
  enum class State {
    kWaitForSipi,
    kRunning,
    kHalted,
    kOff,
  };

  uint32_t apic_id = 0;
  bool is_bsp = false;
  uint64_t apic_base_msr = 0;
  std::map<int, uint32_t> lapic;
  State state = State::kWaitForSipi;
  ApStart start = ApStart::kNormal;
  bool missed_sipi = false;
  bool power_off = false;
  std::bitset<256> pending;
  uintptr_t stack_top = 0;
  std::thread thread;

  bool x2apic() const { return (apic_base_msr & kApicBaseX2Apic) != 0; }
  bool enabled() const { return (apic_base_msr & kApicBaseEnable) != 0; }
};

FakeSmpHardware::FakeSmpHardware(const Options& options)
    : options_(options), start_time_(std::chrono::steady_clock::now()) {
  const std::lock_guard<std::mutex> lock(mutex_);
  Cpu& bsp = CpuLocked(options_.bsp_apic_id);
  bsp.is_bsp = true;
  bsp.apic_base_msr = options_.bsp_apic_base_msr;
  bsp.state = Cpu::State::kRunning;
}

FakeSmpHardware::~FakeSmpHardware() { PowerOffAps(); }

void FakeSmpHardware::SetApStart(const uint32_t apic_id, const ApStart start) {
  const std::lock_guard<std::mutex> lock(mutex_);
  CpuLocked(apic_id).start = start;
}

void FakeSmpHardware::SetApicBaseMsr(const uint32_t apic_id,
                                     const uint64_t value) {
  const std::lock_guard<std::mutex> lock(mutex_);
  CpuLocked(apic_id).apic_base_msr = value;
}

void FakeSmpHardware::SetIcrStuckPending(const bool stuck) {
  const std::lock_guard<std::mutex> lock(mutex_);
  icr_stuck_pending_ = stuck;
}

void FakeSmpHardware::PowerOffAps() {
  std::unique_lock<std::mutex> lock(mutex_);
  for (auto& [apic_id, cpu] : cpus_) {
    if (!cpu->is_bsp) {
      PowerOffLocked(lock, *cpu);
    }
  }
}

std::vector<FakeSmpHardware::Event> FakeSmpHardware::Events() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return events_;
}

std::vector<FakeSmpHardware::Event> FakeSmpHardware::Ipis() const {
  std::vector<Event> ipis;
  for (const Event& event : Events()) {
    if (event.kind == Event::Kind::kIpi) {
      ipis.push_back(event);
    }
  }
  return ipis;
}

std::vector<FakeSmpHardware::Event> FakeSmpHardware::LapicWrites(
    const uint32_t apic_id) const {
  std::vector<Event> writes;
  for (const Event& event : Events()) {
    if (event.kind == Event::Kind::kLapicWrite && event.cpu == apic_id) {
      writes.push_back(event);
    }
  }
  return writes;
}

std::vector<FakeSmpHardware::Event> FakeSmpHardware::MsrWrites(
    const uint32_t apic_id) const {
  std::vector<Event> writes;
  for (const Event& event : Events()) {
    if (event.kind == Event::Kind::kMsrWrite && event.cpu == apic_id) {
      writes.push_back(event);
    }
  }
  return writes;
}

int FakeSmpHardware::EoiCount(const uint32_t apic_id) const {
  int count = 0;
  for (const Event& event : LapicWrites(apic_id)) {
    if (event.reg == kLapicEoi) {
      ++count;
    }
  }
  return count;
}

std::vector<uintptr_t> FakeSmpHardware::GuardPages() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return std::vector<uintptr_t>(guard_pages_.begin(), guard_pages_.end());
}

uint64_t FakeSmpHardware::NowNs() const {
  const auto elapsed = std::chrono::steady_clock::now() - start_time_;
  return static_cast<uint64_t>(
             std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed)
                 .count()) +
         skipped_ns_.load(std::memory_order_acquire);
}

FakeSmpHardware::Cpu& FakeSmpHardware::CpuLocked(const uint32_t apic_id) {
  std::unique_ptr<Cpu>& cpu = cpus_[apic_id];
  if (cpu == nullptr) {
    cpu = std::make_unique<Cpu>();
    cpu->apic_id = apic_id;
    cpu->apic_base_msr = options_.ap_apic_base_msr;
  }
  return *cpu;
}

FakeSmpHardware::Cpu& FakeSmpHardware::CurrentCpuLocked() {
  return CpuLocked((t_ap_owner == this) ? t_ap_apic_id : options_.bsp_apic_id);
}

void FakeSmpHardware::RecordLocked(const Event::Kind kind,  //
                                   const uint32_t cpu,      //
                                   const uint32_t reg,      //
                                   const uint64_t value) {
  events_.push_back(Event{kind, NowNs(), cpu, reg, value});
}

uint32_t FakeSmpHardware::ReadLapicLocked(Cpu& cpu, const int reg_offset) {
  EXPECT_TRUE(cpu.enabled())
      << "Local APIC of CPU " << cpu.apic_id << " read while disabled";
  switch (reg_offset) {
    case kLapicId:
      return cpu.x2apic() ? cpu.apic_id : (cpu.apic_id << 24);
    case kLapicIcrLow: {
      const uint32_t icr = cpu.lapic[kLapicIcrLow] & ~kIcrSendPending;
      return (cpu.is_bsp && icr_stuck_pending_) ? (icr | kIcrSendPending) : icr;
    }
    case kLapicTimerCurrCount: {
      const uint32_t initial = cpu.lapic[kLapicTimerInitCount];
      return initial -
             std::min(initial, options_.lapic_timer_calibration_ticks);
    }
    default: {
      const auto it = cpu.lapic.find(reg_offset);
      if (it != cpu.lapic.end()) {
        return it->second;
      }
      // The Spurious Interrupt Vector Register resets to 0xFF.
      return (reg_offset == kLapicSvr) ? 0xFFu : 0u;
    }
  }
}

void FakeSmpHardware::WriteLapicLocked(std::unique_lock<std::mutex>& lock,  //
                                       Cpu& cpu,                            //
                                       const int reg_offset,                //
                                       const uint64_t value) {
  EXPECT_TRUE(cpu.enabled())
      << "Local APIC of CPU " << cpu.apic_id << " written while disabled";
  RecordLocked(Event::Kind::kLapicWrite, cpu.apic_id, reg_offset, value);
  cpu.lapic[reg_offset] = static_cast<uint32_t>(value);
  if (reg_offset != kLapicIcrLow) {
    return;
  }
  const uint32_t destination = cpu.x2apic() ? static_cast<uint32_t>(value >> 32)
                                            : (cpu.lapic[kLapicIcrHigh] >> 24);
  SendIpiLocked(lock, cpu, destination, static_cast<uint32_t>(value));
}

void FakeSmpHardware::SendIpiLocked(std::unique_lock<std::mutex>& lock,  //
                                    const Cpu& sender,                   //
                                    const uint32_t destination,          //
                                    const uint32_t icr_low) {
  RecordLocked(Event::Kind::kIpi,  //
               sender.apic_id,     //
               0,                  //
               (static_cast<uint64_t>(destination) << 32) | icr_low);
  if ((icr_low & kIcrShorthandMask) != 0) {
    return;
  }
  Cpu& target = CpuLocked(destination);
  switch (icr_low & kIcrDeliveryModeMask) {
    case kIcrInit:
      // The level-triggered INIT de-assert that xAPIC mode sends after the
      // assert has no effect.
      if ((icr_low & kIcrLevelAssert) == 0) {
        return;
      }
      EXPECT_FALSE(target.is_bsp) << "INIT sent to the BSP";
      if (!target.is_bsp) {
        PowerOffLocked(lock, target);
        target.state = Cpu::State::kWaitForSipi;
        target.pending.reset();
        target.lapic.clear();
      }
      return;
    case kIcrStartup:
      StartApLocked(lock, target, static_cast<uint8_t>(icr_low & 0xFF));
      return;
    case kIcrFixed:
      if (target.state == Cpu::State::kRunning ||
          target.state == Cpu::State::kHalted) {
        target.pending.set(icr_low & 0xFF);
        cpu_state_changed_.notify_all();
      }
      return;
    default:
      return;
  }
}

void FakeSmpHardware::StartApLocked(std::unique_lock<std::mutex>& lock,  //
                                    Cpu& cpu,                            //
                                    const uint8_t vector) {
  // A CPU that is not waiting for a SIPI ignores it.
  if (cpu.is_bsp || cpu.state != Cpu::State::kWaitForSipi ||
      cpu.start == ApStart::kNever) {
    return;
  }
  if (cpu.start == ApStart::kMissesFirstSipi && !cpu.missed_sipi) {
    cpu.missed_sipi = true;
    return;
  }
  const uintptr_t start_addr = static_cast<uintptr_t>(vector) << 12;
  EXPECT_TRUE(TrampolineAtLocked(start_addr))
      << "SIPI to CPU " << cpu.apic_id << " starts it at 0x" << std::hex
      << start_addr << ", where no trampoline is installed";
  if (!TrampolineAtLocked(start_addr)) {
    return;
  }
  cpu.state = Cpu::State::kRunning;
  const uint32_t apic_id = cpu.apic_id;
  cpu.thread = std::thread([this, apic_id] { RunAp(apic_id); });
  cpu_state_changed_.wait(lock, [&cpu] {
    return cpu.state == Cpu::State::kHalted || cpu.state == Cpu::State::kOff;
  });
}

void FakeSmpHardware::PowerOffLocked(std::unique_lock<std::mutex>& lock,
                                     Cpu& cpu) {
  if (!cpu.thread.joinable()) {
    return;
  }
  cpu.power_off = true;
  cpu_state_changed_.notify_all();
  std::thread thread = std::move(cpu.thread);
  lock.unlock();
  thread.join();
  lock.lock();
  cpu.power_off = false;
  cpu.state = Cpu::State::kOff;
}

bool FakeSmpHardware::AllApsIdleLocked() const {
  for (const auto& [apic_id, cpu] : cpus_) {
    if (cpu->is_bsp) {
      continue;
    }
    const bool idle =
        cpu->state == Cpu::State::kWaitForSipi ||
        cpu->state == Cpu::State::kOff ||
        (cpu->state == Cpu::State::kHalted && cpu->pending.none());
    if (!idle) {
      return false;
    }
  }
  return true;
}

bool FakeSmpHardware::TrampolineAtLocked(const uintptr_t phys_addr) const {
  if (!trampoline_installed_) {
    return false;
  }
  if (options_.physical_memory == nullptr ||
      static_cast<int64_t>(phys_addr) + kTrampolineImageBytes >
          options_.physical_memory_size) {
    return phys_addr == kApTrampolinePhysAddr;
  }
  const uint8_t* const image = options_.physical_memory + phys_addr;
  return std::all_of(image, image + kTrampolineImageBytes,
                     [](const uint8_t b) { return b == kTrampolineImageByte; });
}

void FakeSmpHardware::RunAp(const uint32_t apic_id) {
  t_ap_owner = this;
  t_ap_apic_id = apic_id;
  // What the boot.S trampoline does before it calls ApKernelEntry().
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    CpuLocked(apic_id).stack_top = g_ap_boot_stack;
  }
  g_ap_boot_started.store(1, std::memory_order_seq_cst);
  SetInterruptsEnabledForTest(false);
  BindCpuLocal(
      reinterpret_cast<CpuLocal*>(static_cast<uintptr_t>(g_ap_boot_cpu_local)));
  ApKernelEntry(g_ap_boot_cpu_index);

  ResetCpuLocalForTest();
  const std::lock_guard<std::mutex> lock(mutex_);
  CpuLocked(apic_id).state = Cpu::State::kOff;
  cpu_state_changed_.notify_all();
}

CpuidResult FakeSmpHardware::Cpuid(const uint32_t leaf,
                                   const uint32_t subleaf) {
  const std::lock_guard<std::mutex> lock(mutex_);
  const Cpu& cpu = CurrentCpuLocked();
  const uint32_t initial_apic_id =
      cpu.is_bsp ? options_.bsp_cpuid_apic_id.value_or(cpu.apic_id)
                 : cpu.apic_id;
  CpuidResult result;
  if (leaf == 0) {
    result.eax = options_.cpuid_max_leaf;
  } else if (leaf > options_.cpuid_max_leaf) {
    return result;
  } else if (leaf == 1) {
    result.ebx = (initial_apic_id & 0xFFu) << 24;
    result.ecx = options_.x2apic_supported ? (1u << 21) : 0;
  } else if (leaf == 0x0B && options_.cpuid_leaf_0b && subleaf == 0) {
    // Level 0: SMT, one logical processor.
    result.eax = 0;
    result.ebx = 1;
    result.ecx = 1u << 8;
    result.edx = initial_apic_id;
  } else if (leaf == 0x15) {
    result.eax = 1;
    result.ebx = options_.cpuid_leaf15_ebx;
    result.ecx = options_.cpuid_leaf15_ecx;
  } else if (leaf == 0x16) {
    result.ecx = options_.cpuid_leaf16_ecx;
  }
  return result;
}

uint64_t FakeSmpHardware::ReadMsr(const uint32_t msr) {
  const std::lock_guard<std::mutex> lock(mutex_);
  Cpu& cpu = CurrentCpuLocked();
  if (msr == kMsrApicBase) {
    return cpu.apic_base_msr;
  }
  if (msr == kMsrEfer) {
    return kEferLongMode;
  }
  if (IsX2ApicMsr(msr)) {
    EXPECT_TRUE(cpu.x2apic())
        << "x2APIC MSR 0x" << std::hex << msr << " read in xAPIC mode";
    return ReadLapicLocked(cpu, static_cast<int>(msr - kMsrX2ApicFirst) << 4);
  }
  return 0;
}

void FakeSmpHardware::WriteMsr(const uint32_t msr, const uint64_t value) {
  std::unique_lock<std::mutex> lock(mutex_);
  Cpu& cpu = CurrentCpuLocked();
  if (IsX2ApicMsr(msr)) {
    EXPECT_TRUE(cpu.x2apic())
        << "x2APIC MSR 0x" << std::hex << msr << " written in xAPIC mode";
    WriteLapicLocked(lock,                                          //
                     cpu,                                           //
                     static_cast<int>(msr - kMsrX2ApicFirst) << 4,  //
                     value);
    return;
  }
  RecordLocked(Event::Kind::kMsrWrite, cpu.apic_id, msr, value);
  if (msr != kMsrApicBase) {
    return;
  }
  // The valid transitions are disabled <-> xAPIC, xAPIC -> x2APIC, and
  // x2APIC -> disabled (Intel SDM, "x2APIC State Transitions").
  const bool was_enabled = cpu.enabled();
  const bool was_x2apic = cpu.x2apic();
  const bool enabled = (value & kApicBaseEnable) != 0;
  const bool x2apic = (value & kApicBaseX2Apic) != 0;
  const bool valid = !(x2apic && !enabled) &&
                     !(was_x2apic && enabled && !x2apic) &&
                     !(!was_enabled && x2apic);
  EXPECT_TRUE(valid) << "CPU " << cpu.apic_id
                     << ": invalid IA32_APIC_BASE transition 0x" << std::hex
                     << cpu.apic_base_msr << " -> 0x" << value;
  cpu.apic_base_msr = value;
}

uint32_t FakeSmpHardware::ReadLapicMmio(const uintptr_t /*lapic_base*/,
                                        const int reg_offset) {
  const std::lock_guard<std::mutex> lock(mutex_);
  Cpu& cpu = CurrentCpuLocked();
  EXPECT_FALSE(cpu.x2apic()) << "xAPIC MMIO read in x2APIC mode";
  return ReadLapicLocked(cpu, reg_offset);
}

void FakeSmpHardware::WriteLapicMmio(const uintptr_t /*lapic_base*/,
                                     const int reg_offset,
                                     const uint32_t value) {
  std::unique_lock<std::mutex> lock(mutex_);
  Cpu& cpu = CurrentCpuLocked();
  EXPECT_FALSE(cpu.x2apic()) << "xAPIC MMIO write in x2APIC mode";
  WriteLapicLocked(lock, cpu, reg_offset, value);
}

uint8_t FakeSmpHardware::InByte(const uint16_t port) {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (port != kPitSpeakerPort) {
    return 0xFF;
  }
  if (pit_counting_ && !pit_out2_ && options_.pit_responds) {
    // The countdown runs out now if it has not already.
    const uint64_t end_ns =
        pit_start_ns_ + pit_reload_ * 1000000000ULL / kPitHz;
    const uint64_t now_ns = NowNs();
    if (now_ns < end_ns) {
      skipped_ns_.fetch_add(end_ns - now_ns, std::memory_order_acq_rel);
    }
    pit_out2_ = true;
  }
  return static_cast<uint8_t>(port61_ | (pit_out2_ ? kPitOut2 : 0));
}

void FakeSmpHardware::OutByte(const uint16_t port, const uint8_t value) {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (port == kPitCommandPort) {
    pit_command_ = value;
    pit_reload_bytes_ = 0;
    pit_reload_ = 0;
  } else if (port == kPitChannel2Port) {
    pit_reload_ |= static_cast<uint32_t>(value) << (8 * pit_reload_bytes_);
    ++pit_reload_bytes_;
  } else if (port == kPitSpeakerPort) {
    const bool gate_rises = (port61_ & kPitGate2) == 0 && (value & kPitGate2);
    port61_ = static_cast<uint8_t>(value & 0x0F);
    if (gate_rises && pit_command_ == kPitChannel2Mode0 &&
        pit_reload_bytes_ == 2) {
      pit_counting_ = true;
      pit_out2_ = false;
      pit_start_ns_ = NowNs();
    } else if ((value & kPitGate2) == 0) {
      pit_counting_ = false;
    }
  }
}

uint64_t FakeSmpHardware::ReadTimestampCounter() { return NowNs(); }

void FakeSmpHardware::SpinPause() {
  bool idle = false;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    idle = AllApsIdleLocked();
  }
  if (idle) {
    skipped_ns_.fetch_add(kIdleSpinStepNs, std::memory_order_acq_rel);
  } else {
    std::this_thread::yield();
  }
}

const void* FakeSmpHardware::MapPhysicalMemory(const uintptr_t phys_addr,
                                               const int64_t size) {
  if ((options_.map_failure_addr != 0 &&
       phys_addr == options_.map_failure_addr) ||
      options_.physical_memory == nullptr || size < 0 ||
      phys_addr > static_cast<uintptr_t>(options_.physical_memory_size) ||
      size > options_.physical_memory_size - static_cast<int64_t>(phys_addr)) {
    return nullptr;
  }
  return options_.physical_memory + phys_addr;
}

bool FakeSmpHardware::MapLocalApicPage(const uintptr_t phys_addr) {
  return options_.map_failure_addr == 0 ||
         phys_addr != options_.map_failure_addr;
}

bool FakeSmpHardware::UnmapGuardPage(const uintptr_t addr) {
  const std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_EQ(addr % kGuardPageBytes, 0u) << "unaligned guard page " << addr;
  EXPECT_FALSE(trampoline_installed_)
      << "guard page unmapped while APs may be starting";
  if (options_.map_failure_addr != 0 && addr == options_.map_failure_addr) {
    return false;
  }
  EXPECT_TRUE(guard_pages_.insert(addr).second)
      << "guard page " << addr << " unmapped twice";
  RecordLocked(Event::Kind::kGuardPageUnmapped,  //
               CurrentCpuLocked().apic_id,       //
               0,                                //
               addr);
  return true;
}

bool FakeSmpHardware::RemapGuardPage(const uintptr_t addr) {
  const std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_EQ(guard_pages_.erase(addr), 1u) << addr << " is not a guard page";
  RecordLocked(Event::Kind::kGuardPageRemapped,  //
               CurrentCpuLocked().apic_id,       //
               0,                                //
               addr);
  return true;
}

uintptr_t FakeSmpHardware::ReadRsp() {
  const std::lock_guard<std::mutex> lock(mutex_);
  const Cpu& cpu = CurrentCpuLocked();
  return (cpu.is_bsp ? options_.bsp_stack.top : cpu.stack_top) - 64;
}

uintptr_t FakeSmpHardware::ReadCr3() { return options_.cr3; }

uint16_t FakeSmpHardware::ReadCs() { return 0x08; }

StackBounds FakeSmpHardware::BspStack() { return options_.bsp_stack; }

void FakeSmpHardware::InstallApTrampoline() {
  const std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_FALSE(trampoline_installed_) << "trampoline installed twice";
  RecordLocked(Event::Kind::kTrampolineInstalled, options_.bsp_apic_id, 0, 0);
  trampoline_installed_ = true;
  if (options_.physical_memory != nullptr &&
      static_cast<int64_t>(kApTrampolinePhysAddr) + kTrampolineImageBytes <=
          options_.physical_memory_size) {
    uint8_t* const dst = options_.physical_memory + kApTrampolinePhysAddr;
    saved_trampoline_bytes_.assign(dst, dst + kTrampolineImageBytes);
    std::memset(dst, kTrampolineImageByte, kTrampolineImageBytes);
  }
}

void FakeSmpHardware::RestoreApTrampoline() {
  const std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_TRUE(trampoline_installed_) << "trampoline restored, not installed";
  RecordLocked(Event::Kind::kTrampolineRestored, options_.bsp_apic_id, 0, 0);
  trampoline_installed_ = false;
  if (!saved_trampoline_bytes_.empty()) {
    std::memcpy(options_.physical_memory + kApTrampolinePhysAddr,  //
                saved_trampoline_bytes_.data(),                    //
                saved_trampoline_bytes_.size());
    saved_trampoline_bytes_.clear();
  }
}

bool FakeSmpHardware::HaltUntilInterrupt() {
  // `sti; hlt`
  RestoreInterrupts(true);
  int vector = 0;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    Cpu& cpu = CurrentCpuLocked();
    cpu.state = Cpu::State::kHalted;
    cpu_state_changed_.notify_all();
    cpu_state_changed_.wait(
        lock, [&cpu] { return cpu.power_off || cpu.pending.any(); });
    if (cpu.power_off) {
      return false;
    }
    vector = 255;
    while (!cpu.pending.test(vector)) {
      --vector;
    }
    cpu.pending.reset(vector);
    cpu.state = Cpu::State::kRunning;
  }
  // The interrupt gate clears IF for the handler, and `iretq` sets it again.
  (void)SaveAndDisableInterrupts();
  InterruptFrame frame = {};
  frame.vector = static_cast<uint64_t>(vector);
  IdtDispatch(&frame);
  RestoreInterrupts(true);
  return true;
}

void FakeSmpHardware::HaltForever() {
  std::fprintf(stderr, "FakeSmpHardware: a CPU halted for good\n");
  std::abort();
}

}  // namespace protos
