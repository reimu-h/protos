#ifndef PROTOS_SMP_HARDWARE_FAKE_H_
#define PROTOS_SMP_HARDWARE_FAKE_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <vector>

#include "smp_hardware.h"

namespace protos {

// Host model of the machine behind SmpHardware, for unit tests of smp.cpp.
//
// - CPUs are identified by APIC ID. The host threads that the fake starts are
//   APs; every other thread acts as the BSP (`Options::bsp_apic_id`).
// - Each CPU has its own IA32_APIC_BASE MSR and Local APIC registers, reached
//   through xAPIC MMIO or x2APIC MSRs depending on that MSR, as on hardware.
//   A register access in the wrong mode or while the Local APIC is disabled,
//   and an invalid APIC mode transition, fail the test.
// - Writing the Interrupt Command Register sends an IPI. INIT resets an AP (a
//   running AP is powered off). A SIPI to an AP that waits for one starts it on
//   a new host thread, which does what the boot.S trampoline does (sets
//   `g_ap_boot_started`, binds the `CpuLocal` at `g_ap_boot_cpu_local`) and
//   calls `ApKernelEntry(g_ap_boot_cpu_index)`; the SIPI returns once that AP
//   halts in its idle loop. A fixed IPI becomes pending at its target, whose
//   thread handles it (`IdtDispatch()`) in `HaltUntilInterrupt()`. IPIs with a
//   destination shorthand and NMIs are only recorded.
// - The TSC runs at 1 GHz and follows the host clock, except that it jumps
//   ahead when nothing but the spinning CPU can make progress (`SpinPause()`
//   while every AP is halted with nothing pending, waiting for a SIPI, or off),
//   so that timeouts expire at once, and when the PIT calibration countdown is
//   polled before it has run out.
// - Physical memory is `Options::physical_memory`, at physical address 0.
//   UnmapGuardPage() and RemapGuardPage() only track which pages are guard
//   pages; unmapping one while the trampoline is installed (APs may be
//   starting) fails the test. FlushTlb() only records the flush.
//
// Records every Local APIC register write, other MSR write, IPI, trampoline
// install and restore, guard page change, and TLB flush, in order and with the
// time it happened.
class FakeSmpHardware final : public SmpHardware {
 public:
  // How an AP responds to SIPIs.
  enum class ApStart {
    kNormal,
    // Ignores the first SIPI after power-on and starts on the next one.
    kMissesFirstSipi,
    // Never starts.
    kNever,
  };

  struct Options {
    uint8_t* physical_memory = nullptr;
    int64_t physical_memory_size = 0;
    uint32_t bsp_apic_id = 0;
    // Initial APIC ID that CPUID reports on the BSP (leaf 0x01 EBX[31:24], and
    // leaf 0x0B EDX); `bsp_apic_id` if unset. APs report their APIC ID.
    std::optional<uint32_t> bsp_cpuid_apic_id;
    // CPUID.01H:ECX[21].
    bool x2apic_supported = false;
    uint32_t cpuid_max_leaf = 0x0B;
    // Whether CPUID leaf 0x0B enumerates the topology.
    bool cpuid_leaf_0b = true;
    uint32_t cpuid_leaf15_ebx = 0;
    uint32_t cpuid_leaf15_ecx = 0;
    uint32_t cpuid_leaf16_ecx = 0;
    // IA32_APIC_BASE at power-on: base 0xFEE00000, enabled, BSP flag on the
    // BSP. SetApicBaseMsr() overrides it for one CPU.
    uint64_t bsp_apic_base_msr = 0xFEE00900;
    uint64_t ap_apic_base_msr = 0xFEE00800;
    // Ticks that a Local APIC timer counts down while the PIT calibration
    // countdown runs.
    uint32_t lapic_timer_calibration_ticks = 625000;
    // Whether PIT channel 2 ever reaches terminal count.
    bool pit_responds = true;
    // Physical address at which MapPhysicalMemory(), MapLocalApicPage() and
    // UnmapGuardPage() fail (0: none).
    uintptr_t map_failure_addr = 0;
    StackBounds bsp_stack = {0x100000, 0x104000};
    uintptr_t cr3 = 0x105000;
  };

  struct Event {
    enum class Kind {
      kLapicWrite,
      kMsrWrite,
      kIpi,
      kTrampolineInstalled,
      kTrampolineRestored,
      kGuardPageUnmapped,
      kGuardPageRemapped,
      kTlbFlush,
    };
    Kind kind;
    uint64_t time_ns;
    // APIC ID of the CPU that caused the event.
    uint32_t cpu;
    // kLapicWrite: the register's xAPIC offset (in both modes); kMsrWrite: the
    // MSR.
    uint32_t reg;
    // kLapicWrite and kMsrWrite: the value written; kIpi: the destination APIC
    // ID << 32 | ICR bits 0-31; kGuardPageUnmapped and kGuardPageRemapped: the
    // page's address.
    uint64_t value;
  };

  explicit FakeSmpHardware(const Options& options);
  // Powers off the APs.
  ~FakeSmpHardware() override;

  FakeSmpHardware(const FakeSmpHardware&) = delete;
  FakeSmpHardware& operator=(const FakeSmpHardware&) = delete;

  void SetApStart(uint32_t apic_id, ApStart start);
  void SetApicBaseMsr(uint32_t apic_id, uint64_t value);
  // Makes the BSP's xAPIC ICR report Send Pending (bit 12) for good.
  void SetIcrStuckPending(bool stuck);
  // Powers off every AP and waits for its thread to exit.
  void PowerOffAps();

  std::vector<Event> Events() const;
  std::vector<Event> Ipis() const;
  // The Local APIC register writes and other MSR writes of the CPU with
  // `apic_id`.
  std::vector<Event> LapicWrites(uint32_t apic_id) const;
  std::vector<Event> MsrWrites(uint32_t apic_id) const;
  // Number of End-Of-Interrupt writes by the CPU with `apic_id`.
  int EoiCount(uint32_t apic_id) const;
  // Number of FlushTlb() calls by the CPU with `apic_id`.
  int TlbFlushCount(uint32_t apic_id) const;
  // The pages that are unmapped as guard pages, in ascending order.
  std::vector<uintptr_t> GuardPages() const;
  uint64_t NowNs() const;

  CpuidResult Cpuid(uint32_t leaf, uint32_t subleaf) override;
  uint64_t ReadMsr(uint32_t msr) override;
  void WriteMsr(uint32_t msr, uint64_t value) override;
  uint32_t ReadLapicMmio(uintptr_t lapic_base, int reg_offset) override;
  void WriteLapicMmio(uintptr_t lapic_base,  //
                      int reg_offset,        //
                      uint32_t value) override;
  uint8_t InByte(uint16_t port) override;
  void OutByte(uint16_t port, uint8_t value) override;
  uint64_t ReadTimestampCounter() override;
  void SpinPause() override;
  const void* MapPhysicalMemory(uintptr_t phys_addr, int64_t size) override;
  bool MapLocalApicPage(uintptr_t phys_addr) override;
  bool UnmapGuardPage(uintptr_t addr) override;
  bool RemapGuardPage(uintptr_t addr) override;
  void FlushTlb() override;
  uintptr_t ReadRsp() override;
  uintptr_t ReadCr3() override;
  uint16_t ReadCs() override;
  StackBounds BspStack() override;
  void InstallApTrampoline() override;
  void RestoreApTrampoline() override;
  bool HaltUntilInterrupt() override;
  void HaltForever() override;

 private:
  struct Cpu;

  Cpu& CpuLocked(uint32_t apic_id);
  Cpu& CurrentCpuLocked();
  void RecordLocked(Event::Kind kind,  //
                    uint32_t cpu,      //
                    uint32_t reg,      //
                    uint64_t value);
  uint32_t ReadLapicLocked(Cpu& cpu, int reg_offset);
  void WriteLapicLocked(std::unique_lock<std::mutex>& lock,  //
                        Cpu& cpu,                            //
                        int reg_offset,                      //
                        uint64_t value);
  void SendIpiLocked(std::unique_lock<std::mutex>& lock,  //
                     const Cpu& sender,                   //
                     uint32_t destination,                //
                     uint32_t icr_low);
  void StartApLocked(std::unique_lock<std::mutex>& lock,  //
                     Cpu& cpu,                            //
                     uint8_t vector);
  void PowerOffLocked(std::unique_lock<std::mutex>& lock, Cpu& cpu);
  bool AllApsIdleLocked() const;
  bool TrampolineAtLocked(uintptr_t phys_addr) const;
  void RunAp(uint32_t apic_id);

  const Options options_;
  const std::chrono::steady_clock::time_point start_time_;
  // Nanoseconds by which the TSC is ahead of the host clock.
  std::atomic<uint64_t> skipped_ns_{0};

  mutable std::mutex mutex_;
  std::condition_variable cpu_state_changed_;
  std::map<uint32_t, std::unique_ptr<Cpu>> cpus_;
  std::vector<Event> events_;
  bool icr_stuck_pending_ = false;
  bool trampoline_installed_ = false;
  std::vector<uint8_t> saved_trampoline_bytes_;
  std::set<uintptr_t> guard_pages_;
  // PIT channel 2 and port 0x61.
  uint8_t pit_command_ = 0;
  int pit_reload_bytes_ = 0;
  uint32_t pit_reload_ = 0;
  uint8_t port61_ = 0;
  bool pit_counting_ = false;
  bool pit_out2_ = false;
  uint64_t pit_start_ns_ = 0;
};

}  // namespace protos

#endif  // PROTOS_SMP_HARDWARE_FAKE_H_
