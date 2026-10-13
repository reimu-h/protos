#ifndef PROTOS_SMP_H_
#define PROTOS_SMP_H_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <type_traits>

#include "check.h"

namespace protos {

// Number of 4 KiB physical frames allocated per Application Processor stack
// (4 * 4 KiB = 16 KiB per AP).
constexpr int64_t kApStackFrames = 4;
// Size in bytes of each CPU's kernel stack (16 KiB).
constexpr int64_t kApStackSize = kApStackFrames * 4096;
// Default physical base address of the x86 Local APIC MMIO registers.
constexpr uintptr_t kDefaultLocalApicPhysAddr = 0xFEE00000;
// Physical address below 1 MiB where the 16-bit real-mode AP startup
// trampoline is copied before sending INIT-SIPI-SIPI.
constexpr uintptr_t kApTrampolinePhysAddr = 0x8000;
// Startup IPI (SIPI) vector corresponding to `kApTrampolinePhysAddr >> 12`.
constexpr uint8_t kApTrampolineVector = 0x08;
static_assert(static_cast<uintptr_t>(kApTrampolineVector) ==
              (kApTrampolinePhysAddr >> 12));

// Architectural IA32_GS_BASE MSR address used for per-CPU `CpuLocal` pointer.
constexpr uint32_t kMsrIa32GsBase = 0xC0000101u;

// 8254 Programmable Interval Timer (PIT) base oscillator frequency in Hz.
constexpr int64_t kPitBaseFrequencyHz = 1193182;
// PIT Channel 2 reload count used during BSP Local APIC timer calibration
// (~10.00015 ms window at 1,193,182 Hz).
constexpr int kPitCalibrationReloadCount = 11932;
// Periodic Local APIC timer target interrupt frequency in Hz.
constexpr int kApicTimerTargetHz = 250;
// Maximum number of logical CPUs the kernel brings up, BSP included.
// SmpParseMadt() ignores the processor entries beyond this limit.
constexpr int kMaxSupportedCpus = 128;
// Reserved 8-bit xAPIC broadcast/invalid APIC ID.
constexpr uint8_t kInvalidXapicId = 0xFFu;
// Reserved 32-bit x2APIC broadcast/invalid APIC ID.
constexpr uint32_t kInvalidX2ApicId = 0xFFFFFFFFu;

// Per-CPU descriptor populated during ACPI MADT discovery and AP bring-up.
struct CpuInfo {
  uint32_t apic_id;
  uint32_t acpi_processor_id;
  uint32_t observed_apic_id;
  bool is_bsp;
  bool online;
  bool long_mode_active;
  uintptr_t stack_base;
  uintptr_t stack_top;
  uintptr_t observed_rsp;
  uintptr_t observed_cr3;
};

class IrqSpinLock;

// Cache-line-aligned per-CPU runtime state bound to `IA32_GS_BASE` on bare
// metal (`gs:[0]`) and a `thread_local` pointer in host unit tests.
struct alignas(64) CpuLocal {
  CpuLocal* self = nullptr;
  int cpu_id = 0;
  uint32_t apic_id = 0;
  bool online = false;
  uintptr_t stack_base = 0;
  uintptr_t stack_top = 0;
  IrqSpinLock* top_held_lock = nullptr;
  std::atomic<int64_t> timer_ticks{0};
  std::atomic<int64_t> ipi_count{0};
};
static_assert(offsetof(CpuLocal, self) == 0);
static_assert(alignof(CpuLocal) == 64);
static_assert(std::is_standard_layout_v<CpuLocal>);

namespace internal {

#if !__STDC_HOSTED__
// Set once the BSP has bound its `CpuLocal`. Before that, IA32_GS_BASE is 0
// and `gs:[0]` would read physical address 0.
inline std::atomic<bool> g_cpu_local_bound{false};
#else
// Host tests run scheduler tasks on their own stacks, and a task can resume on
// a different host thread. Keeping the `thread_local` lookup out of line makes
// each call find the slot of the thread it runs on (an inlined lookup lets the
// compiler reuse a slot address computed on another thread), and the asm
// statement gives the function a side effect so that the compiler cannot merge
// or hoist calls to it either.
[[gnu::noinline]] inline CpuLocal** HostCurrentCpuSlot() {
  thread_local CpuLocal* current_cpu = nullptr;
  asm volatile("" : "+m"(current_cpu));
  return &current_cpu;
}
#endif

}  // namespace internal

// Binds `cpu` as the calling CPU's (or host thread's) active `CpuLocal` state
// (`IA32_GS_BASE` MSR `0xC0000101` on bare metal, `thread_local` pointer on
// host). Validates `cpu != nullptr`, `cpu->self == cpu`, and `cpu->cpu_id >= 0`
// with `DCHECK`.
inline void BindCpuLocal(CpuLocal* const cpu) {
  DCHECK(cpu != nullptr);
  DCHECK(cpu->self == cpu);
  DCHECK(cpu->cpu_id >= 0);
#if !__STDC_HOSTED__
  const uint64_t addr = reinterpret_cast<uintptr_t>(cpu);
  const uint32_t low = static_cast<uint32_t>(addr & 0xFFFFFFFFu);
  const uint32_t high = static_cast<uint32_t>(addr >> 32);
  asm volatile("wrmsr" : : "c"(kMsrIa32GsBase), "a"(low), "d"(high) : "memory");
  internal::g_cpu_local_bound.store(true, std::memory_order_release);
#else
  *internal::HostCurrentCpuSlot() = cpu;
#endif
}

// Returns the calling CPU's `CpuLocal*` if bound and valid, or `nullptr` if no
// `CpuLocal` has been bound yet. On bare metal, "bound" is tracked globally:
// once the BSP has bound its `CpuLocal`, every CPU trusts its own `gs:[0]`.
// That is safe because ApKernelEntry() binds before an AP loads the kernel IDT
// or takes a lock; until then the AP runs only boot.S code, whose exception
// stubs do not call into C++. The panic path, which can run before that point,
// uses CurrentCpuOrNullForPanic() instead.
inline CpuLocal* CurrentCpuOrNull() {
#if !__STDC_HOSTED__
  if (!internal::g_cpu_local_bound.load(std::memory_order_acquire)) {
    return nullptr;
  }
  CpuLocal* cpu = nullptr;
  asm volatile("mov %0, qword ptr gs:[0]" : "=r"(cpu) : : "memory");
  if (cpu != nullptr && cpu->self == cpu) {
    return cpu;
  }
  return nullptr;
#else
  CpuLocal* const cpu = *internal::HostCurrentCpuSlot();
  if (cpu != nullptr && cpu->self == cpu) {
    return cpu;
  }
  return nullptr;
#endif
}

// Like CurrentCpuOrNull(), but reads the calling CPU's IA32_GS_BASE MSR
// instead of relying on the global "bound" flag, so it also returns `nullptr`
// on an AP that has not called BindCpuLocal() yet (boot.S leaves its GS base
// at 0). Slower (`rdmsr`); meant for the panic path.
inline CpuLocal* CurrentCpuOrNullForPanic() {
#if !__STDC_HOSTED__
  uint32_t low = 0;
  uint32_t high = 0;
  asm volatile("rdmsr" : "=a"(low), "=d"(high) : "c"(kMsrIa32GsBase));
  CpuLocal* const cpu =
      reinterpret_cast<CpuLocal*>((static_cast<uint64_t>(high) << 32) | low);
  if (cpu != nullptr && cpu->self == cpu) {
    return cpu;
  }
  return nullptr;
#else
  return CurrentCpuOrNull();
#endif
}

// Returns the calling CPU's `CpuLocal*` in O(1) (`gs:[0]` on bare metal,
// `thread_local` pointer on host). Validates with `DCHECK` that
// `cpu != nullptr` and `cpu->self == cpu`.
inline CpuLocal* CurrentCpu() {
#if !__STDC_HOSTED__
  CpuLocal* cpu = nullptr;
  asm volatile("mov %0, qword ptr gs:[0]" : "=r"(cpu) : : "memory");
#else
  CpuLocal* const cpu = *internal::HostCurrentCpuSlot();
#endif
  DCHECK(cpu != nullptr);
  DCHECK(cpu->self == cpu);
  return cpu;
}

// Returns the logical CPU index (`0 .. SmpCpuCount() - 1`) of the calling CPU
// in O(1) via `CurrentCpu()->cpu_id`.
inline int CurrentCpuId() { return CurrentCpu()->cpu_id; }

#if __STDC_HOSTED__
// Clears the calling host thread's bound `CpuLocal*` pointer for unit tests.
inline void ResetCpuLocalForTest() {
  *internal::HostCurrentCpuSlot() = nullptr;
}
#endif

// Discovered multiprocessor topology from ACPI MADT firmware tables.
struct SmpTopology {
  uintptr_t local_apic_phys_addr = 0;
  uint32_t bsp_apic_id = 0;
  int cpu_count = 0;
  // Number of enabled processor entries left out because of the
  // `kMaxSupportedCpus` limit.
  int ignored_cpu_count = 0;
  std::unique_ptr<CpuInfo[]> cpus;
};

// Scans 16-byte-aligned physical addresses in `[range_start, range_start +
// range_length)` (`range_start != 0`, `range_length > 0`) for a valid ACPI
// RSDP structure ("RSD PTR " signature and valid v1/v2 checksums). Returns the
// physical address of the RSDP, or 0 if none is found. Validates preconditions
// with `DCHECK`.
uintptr_t SmpFindRsdpInMemoryRange(uintptr_t range_start, int64_t range_length);

// Searches the ACPI XSDT (at `xsdt_addr`, if non-zero and valid) and/or RSDT
// (at `rsdt_addr`, if non-zero and valid) for a Multiple APIC Description
// Table (MADT, signature "APIC") with a valid ACPI checksum. Requires at least
// one of `rsdt_addr` or `xsdt_addr` to be non-zero and `max_physical_addr > 0`
// via `DCHECK`. Returns the physical address of the MADT, or 0 if not found.
uintptr_t SmpFindMadtInSdt(uintptr_t rsdt_addr,  //
                           uintptr_t xsdt_addr,  //
                           uintptr_t max_physical_addr);

// Parses the ACPI MADT at non-zero `madt_addr` (below non-zero
// `max_physical_addr`) into non-null `*out_topology`: the Local APIC MMIO base
// address (including any Type 5 64-bit override) and the processors whose
// Type 0 (Local APIC) or Type 9 (Local x2APIC) entry has the Enabled flag set.
// Entries that are only Online Capable describe hot-pluggable processors that
// are not present, and are ignored. The BSP (`bsp_apic_id`,
// `!= kInvalidX2ApicId`) is placed at index 0, followed by the other
// processors in table order, at most `kMaxSupportedCpus` CPUs in total; the
// number of enabled entries left out is stored in `ignored_cpu_count`.
// Validates preconditions with `DCHECK`. Returns true if the MADT is valid and
// lists at least one enabled processor.
bool SmpParseMadt(uintptr_t madt_addr,          //
                  uintptr_t max_physical_addr,  //
                  uint32_t bsp_apic_id,         //
                  SmpTopology* out_topology);

// Discovers the system CPU topology from Multiboot2 ACPI tags (if
// `multiboot_magic` is Multiboot2) or legacy BIOS memory ranges (`ebda_base`
// and `bios_rom_base`), populating non-null `*out_topology`. Validates
// preconditions with `DCHECK`. Returns true if a valid MADT was found and
// parsed.
bool SmpDiscoverTopology(uint32_t multiboot_magic,      //
                         uint64_t multiboot_info_addr,  //
                         uintptr_t ebda_base,           //
                         uintptr_t bios_rom_base,       //
                         uintptr_t max_physical_addr,   //
                         uint32_t bsp_apic_id,          //
                         SmpTopology* out_topology);

// Computes the periodic Local APIC timer initial count for `target_hz`
// (`0 < target_hz <= 1193182`) given `elapsed_apic_ticks` measured over
// `pit_reload_count` (`0 < pit_reload_count <= 0xFFFF`) 8254 PIT ticks at
// 1,193,182 Hz. Validates preconditions with `DCHECK`.
uint32_t ComputeApicTimerInitialCount(uint64_t elapsed_apic_ticks,  //
                                      int pit_reload_count,         //
                                      int target_hz);

// Maximum number of port 0x61 poll iterations before PIT calibration gives up.
constexpr int kPitCalibrationPollLimit = 1000000;
// Last-resort periodic Local APIC timer initial count, used when neither the
// PIT nor CPUID provides the timer frequency. It yields 250 Hz only with the
// 1 GHz APIC timer clock that QEMU and KVM emulate; real Local APIC timers run
// at about 19-100 MHz, where it yields roughly 5-25 Hz. SmpInit() prints a
// warning when it has to use it.
constexpr uint32_t kFallbackApicTimerInitialCount = 250000;

using PitSpeakerReadFn = uint8_t (*)();
using ApicTimerCurrentCountReadFn = uint32_t (*)(uintptr_t lapic_base);

// Polls `read_pit_speaker()` (bit 5, `0x20`, is PIT channel 2's OUT2) for at
// most `max_polls` iterations (`max_polls > 0`) until the PIT reaches terminal
// count, then reads `read_apic_current_count(lapic_base)` and returns the
// periodic Local APIC timer initial count for `kApicTimerTargetHz`. Returns 0,
// instead of hanging, if the PIT does not reach terminal count within
// `max_polls` iterations or no APIC timer ticks elapsed.
uint32_t CalibrateApicTimerCount(
    uintptr_t lapic_base,               //
    int max_polls,                      //
    PitSpeakerReadFn read_pit_speaker,  //
    ApicTimerCurrentCountReadFn read_apic_current_count);

// Discovers all CPUs, initializes the BSP Local APIC (in x2APIC mode when the
// CPU supports it, otherwise in xAPIC mode) and `CpuLocal` state, and
// calibrates the Local APIC timer against PIT channel 2 (or, failing that,
// from the timer clock that CPUID reports, or with
// `kFallbackApicTimerInitialCount` and a warning). The BSP's Local APIC ID
// register, not CPUID, identifies its MADT entry. Then allocates a 16 KiB stack
// per AP from the PMM and starts the APs one by one with INIT-SIPI-SIPI through
// the real-mode trampoline at 0x8000. Each AP enters 64-bit Long Mode, binds
// its `CpuLocal` via `IA32_GS_BASE`, loads the 256-entry IDT, starts its
// periodic Local APIC timer, and parks in a maskable `sti; hlt` idle loop. An
// AP that does not report in within the startup timeout, or whose APIC ID is
// not addressable in xAPIC mode, is left out of the topology (and of
// `SmpCpuCount()`). Finally enables interrupts on the BSP (`sti`). Panics via
// `CHECK` if topology discovery, MMIO mapping, or stack allocation fails.
void SmpInit(uint32_t multiboot_magic, uint64_t multiboot_info_addr);

// Returns the total number of CPUs discovered during `SmpInit`. Validates with
// `DCHECK` that `SmpInit` has completed.
int SmpCpuCount();

// Returns the number of CPUs (BSP + APs) currently online in 64-bit mode.
// Validates with `DCHECK` that `SmpInit` has completed.
int SmpOnlineCpuCount();

// Returns a non-null pointer to the descriptor for CPU `index`. Validates with
// `DCHECK` that `SmpInit` has completed and `0 <= index < SmpCpuCount()`.
const CpuInfo* SmpGetCpuInfo(int index);

// Returns a non-null pointer to the per-CPU `CpuLocal` state for CPU
// `cpu_index`. Validates with `DCHECK` that `SmpInit` has completed and
// `0 <= cpu_index < SmpCpuCount()`.
CpuLocal* SmpGetCpuLocal(int cpu_index);

// Returns the physical base address of the Local APIC MMIO region discovered
// during `SmpInit`. Validates with `DCHECK` that `SmpInit` has completed.
uintptr_t SmpLocalApicPhysAddr();

// Returns true if the Local APIC is operating in MSR-based x2APIC mode.
bool SmpIsX2ApicEnabled();

// Returns the periodic Local APIC timer initial count that `SmpInit` chose
// (see `SmpInit` for its sources). Validates with `DCHECK` that `SmpInit` has
// completed.
uint32_t SmpCalibratedTimerInitialCount();

// Programs the calling CPU's Local APIC timer initial count register (`0x380`
// in xAPIC MMIO mode or MSR `0x838` in x2APIC mode) to `initial_count` if
// `initial_count > 0`.
void SmpSetLocalApicTimerInitialCount(uint32_t initial_count);

// Writes End-Of-Interrupt (`0`) to the calling CPU's Local APIC EOI register
// (MMIO offset `0x0B0` in xAPIC mode, MSR `0x80B` in x2APIC mode).
void SmpSendLocalApicEoi();

// Sends a fixed-delivery Inter-Processor Interrupt (IPI) on `vector` to online
// CPU `target_cpu_index` (`0 <= target_cpu_index < SmpCpuCount()`). `vector`
// must be in `32 .. 254`: IdtDispatch() never sends an EOI for the spurious
// vector `0xFF`, so a fixed IPI on it would block every lower-priority
// interrupt on the target CPU for good. Validates preconditions with `DCHECK`.
void SmpSendIpi(int target_cpu_index, uint8_t vector);

// Callback signature for multi-CPU work dispatched via `SmpRunOnAllCpus`.
using SmpWorkFn = void (*)(int cpu_index, void* context);

// Runs non-null `work_fn(cpu_index, context)` on every CPU
// (`0 .. SmpCpuCount() - 1`) at the same time and returns when all calls have
// returned. CPU 0 runs `work_fn(0, context)` itself; each AP is woken from its
// idle loop by the maskable IPI vector `0x21`, waits at a start barrier, runs
// its call, and returns to its `sti; hlt` idle loop. `work_fn` runs with
// interrupts enabled on every CPU.
//
// Requirements, checked with `DCHECK` unless noted otherwise:
// - `SmpInit` has completed.
// - The caller runs on CPU 0 (in practice the bootstrap task, which is never
//   migrated) with interrupts enabled, and holds no `IrqSpinLock`, which an
//   AP's `work_fn` might need.
// - Every AP is idle rather than running a scheduler task: APs pick up work
//   only in their idle loop, and an AP that does not acknowledge the wakeup
//   within 10 s fails a `CHECK`.
// - Not re-entrant: `work_fn` must not call `SmpRunOnAllCpus` (`CHECK`).
// The wait for the calls to return has no timeout.
void SmpRunOnAllCpus(SmpWorkFn work_fn, void* context);

// Stops the other CPUs for a kernel panic: sends them an NMI, which parks them
// for good in the NMI handler that `SmpInit` installs, and waits up to 100 ms
// for them to park. Called by the panic path (idt.cpp) on the panicking CPU
// with interrupts disabled. Does nothing until `SmpInit` has enabled the BSP's
// Local APIC. Never fails a `CHECK`, which would re-enter the panic path.
void SmpStopOtherCpusForPanic();

#if __STDC_HOSTED__
// Host unit-test hooks for overriding hardware AP bring-up behavior, 32-bit
// ACPI table pointer base, and simulated PIT/APIC timer calibration ticks.
using SmpHostApBootSimFn = bool (*)(int cpu_index, CpuInfo* cpu_info);
using SmpHostCpuidFn = void (*)(uint32_t leaf,     //
                                uint32_t subleaf,  //
                                uint32_t* eax,     //
                                uint32_t* ebx,     //
                                uint32_t* ecx,     //
                                uint32_t* edx);
using SmpHostReadMsrFn = uint64_t (*)(uint32_t msr);
using SmpHostWriteMsrFn = void (*)(uint32_t msr, uint64_t value);

// Resets all SMP state and the CPUID/MSR hooks, and configures the simulated
// firmware: the BSP's APIC ID (used as both its CPUID initial APIC ID and, in
// xAPIC mode, its Local APIC ID), the BIOS scan ranges, and the high 32 bits
// of 32-bit ACPI table pointers.
void SmpSetHostTestHooks(uint32_t bsp_apic_id,     //
                         uintptr_t ebda_base,      //
                         uintptr_t bios_rom_base,  //
                         uintptr_t acpi32_arena,   //
                         SmpHostApBootSimFn sim_fn);
// Installs CPUID/MSR hooks, which `SmpInit` uses to detect x2APIC support and,
// in x2APIC mode, to enable the BSP's Local APIC and read its APIC ID. Also
// sets the current x2APIC mode for the Local APIC helpers below.
void SmpSetHostHardwareHooksForTest(bool x2apic_mode,              //
                                    SmpHostCpuidFn cpuid_fn,       //
                                    SmpHostReadMsrFn read_msr_fn,  //
                                    SmpHostWriteMsrFn write_msr_fn);
// Wrappers around internal helpers of `SmpDiscoverTopology`.
uintptr_t SmpFindRsdpInMultiboot2ForTest(uintptr_t multiboot_info_addr,
                                         uintptr_t max_physical_addr);
bool SmpParseRsdpForTest(uintptr_t rsdp_addr,          //
                         uintptr_t max_physical_addr,  //
                         uintptr_t* out_rsdt_addr,     //
                         uintptr_t* out_xsdt_addr);
uint32_t SmpReadCpuidInitialApicIdForTest();
uint32_t SmpReadLapicIdForTest(uintptr_t lapic_base);
void SmpEnableLocalApicForTest(uintptr_t lapic_base);
void SmpLapicWriteForTest(uintptr_t lapic_base, int reg_offset, uint32_t value);
void SmpLapicSendIpiCommandForTest(uintptr_t lapic_base,  //
                                   uint32_t apic_id,      //
                                   uint32_t icr_low);
void SmpSetHostTimerCalibrationTicksForTest(uint64_t elapsed_apic_ticks);
int64_t SmpGetHostEoiCountForTest();
void SmpResetHostEoiCountForTest();
#endif

}  // namespace protos

#endif  // PROTOS_SMP_H_
