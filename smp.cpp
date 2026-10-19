#include "smp.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <new>
#include <type_traits>
#include <utility>

#include "check.h"
#include "console.h"
#include "cpu.h"
#include "gdt.h"
#include "idt.h"
#include "multiboot.h"
#include "paging.h"
#include "pmm.h"
#include "smp_hardware.h"
#include "spinlock.h"
#include "steal_time.h"
#include "task.h"
#include "uart.h"

// The AP boot handoff (see smp_hardware.h).
extern "C" {
volatile uint64_t g_ap_boot_stack = 0;
volatile uint64_t g_ap_boot_cpu_local = 0;
volatile int g_ap_boot_cpu_index = 0;
std::atomic<int> g_ap_boot_started{0};
static_assert(sizeof(g_ap_boot_started) == sizeof(int));
static_assert(std::is_standard_layout_v<std::atomic<int>>);
}

namespace protos {

namespace {

constexpr int64_t kMaxAcpiTableBytes = 256 * 1024;
constexpr int64_t kMaxRsdpBytes = 4096;
constexpr int64_t kEbdaScanBytes = 1024;
constexpr uintptr_t kEbdaMinAddress = 0x00080000;
constexpr uintptr_t kEbdaMaxAddress = 0x000A0000;
constexpr uintptr_t kBiosRomSearchBase = 0x000E0000;
constexpr int64_t kBiosRomScanBytes = 0x00020000;
constexpr int64_t kRsdpScanAlignment = 16;

constexpr uint8_t kMadtTypeLocalApic = 0;
constexpr uint8_t kMadtTypeLocalApicOverride = 5;
constexpr uint8_t kMadtTypeLocalX2Apic = 9;
// Enabled flag (bit 0) of Local APIC and Local x2APIC entries. Bit 1, Online
// Capable, marks a processor that is not present but can be hot-added later;
// such entries are not started.
constexpr uint32_t kMadtLapicFlagEnabled = 1u << 0;
constexpr int kMaxPitReloadCount = 0xFFFF;
constexpr uint8_t kPitSpeakerOut2Bit = 1u << 5;

constexpr uint32_t kMsrIa32ApicBase = 0x0000001B;
constexpr uint32_t kMsrX2ApicBase = 0x00000800;
constexpr uint32_t kCpuidEcxX2ApicBit = 1u << 21;
constexpr uint64_t kApicBaseX2ApicEnable = 1ULL << 10;
constexpr uint64_t kApicBaseGlobalEnable = 1ULL << 11;

constexpr int kLapicRegId = 0x020;
constexpr int kLapicRegTpr = 0x080;
constexpr int kLapicRegEoi = 0x0B0;
constexpr int kLapicRegSvr = 0x0F0;
constexpr int kLapicRegEsr = 0x280;
constexpr int kLapicRegIcrLow = 0x300;
constexpr int kLapicRegIcrHigh = 0x310;
constexpr int kLapicRegLvtTimer = 0x320;
constexpr int kLapicRegTimerInitCount = 0x380;
constexpr int kLapicRegTimerCurrCount = 0x390;
constexpr int kLapicRegTimerDivConfig = 0x3E0;

constexpr uint32_t kLapicSvrSoftwareEnable = 1u << 8;
constexpr uint32_t kLapicSpuriousVector = 0xFFu;
constexpr uint32_t kLapicTimerDivBy16 = 0x03u;
constexpr uint32_t kLapicTimerMaskBit = 1u << 16;
constexpr uint32_t kLapicTimerPeriodicMode = 1u << 17;

constexpr uint32_t kIcrDeliveryFixed = 0x00000000u;
constexpr uint32_t kIcrDeliveryNmi = 0x00000400u;
constexpr uint32_t kIcrDeliveryInit = 0x00000500u;
constexpr uint32_t kIcrDeliveryStartup = 0x00000600u;
constexpr uint32_t kIcrDeliveryPending = 1u << 12;
constexpr uint32_t kIcrLevelAssert = 1u << 14;
constexpr uint32_t kIcrTriggerLevel = 1u << 15;
constexpr uint32_t kIcrDestAllExcludingSelf = 3u << 18;
constexpr int kIcrIdlePollLimit = 1000000;
// ICR value of the NMI that stops the other CPUs when one CPU panics.
constexpr uint32_t kPanicStopIcr =
    kIcrDeliveryNmi | kIcrLevelAssert | kIcrDestAllExcludingSelf;

constexpr uint32_t kMsrIa32Efer = 0xC0000080;
constexpr uint64_t kEferLongModeActive = 1ULL << 10;
constexpr uint16_t kKernelCodeSelector = 0x08;

constexpr uint16_t kPitChannel2DataPort = 0x42;
constexpr uint16_t kPitCommandPort = 0x43;
constexpr uint16_t kPitSpeakerPort = 0x61;
constexpr uint8_t kPitCmdChannel2Mode0 = 0xB0;
constexpr uint8_t kPitSpeakerGate2Bit = 1u << 0;
constexpr uint8_t kPitSpeakerEnableMask = 0x03u;

// Physical address of the BIOS Data Area word that holds the EBDA segment.
constexpr uintptr_t kBdaEbdaSegmentAddr = 0x040E;

// INIT-SIPI-SIPI delays from the Intel SDM's MP initialization sequence: 10 ms
// after INIT and 200 us after each SIPI.
constexpr int64_t kInitDelayUs = 10000;
constexpr int64_t kSipiDelayUs = 200;
// Time an AP gets, after its last SIPI, to report in from ApKernelEntry() (the
// same limit as Linux).
constexpr int64_t kApStartupTimeoutMs = 10000;
// TSC rate assumed until CalibrateLocalApicTimer() measures it, and if the PIT
// does not respond. It is deliberately high (8 GHz), so that delays err on the
// long side.
constexpr uint64_t kFallbackTscTicksPerMs = 8000000;
// TSC rates that a calibration may report: 100 MHz to 20 GHz. A rate below
// would shorten every delay and timeout.
constexpr uint64_t kMinPlausibleTscTicksPerMs = 100000;
constexpr uint64_t kMaxPlausibleTscTicksPerMs = 20000000;

// Hyper-V (Microsoft's Hypervisor Top Level Functional Specification), whose
// generation 2 VMs have no PIT: CPUID.01H:ECX[31] (a hypervisor is present),
// the hypervisor vendor leaf with the signature "Microsoft Hv", the features
// leaf, whose EAX bit 11 grants access to the frequency MSRs and whose EDX
// bit 8 says that the TSC frequency MSR is available, and those MSRs, which
// hold the TSC and Local APIC timer frequencies in Hz.
constexpr uint32_t kCpuidEcxHypervisorBit = 1u << 31;
constexpr uint32_t kCpuidHypervisorVendorLeaf = 0x40000000;
constexpr uint32_t kHypervSignatureEbx = 0x7263694D;  // "Micr"
constexpr uint32_t kHypervSignatureEcx = 0x666F736F;  // "osof"
constexpr uint32_t kHypervSignatureEdx = 0x76482074;  // "t Hv"
constexpr uint32_t kCpuidHypervFeaturesLeaf = 0x40000003;
constexpr uint32_t kHypervAccessFrequencyMsrs = 1u << 11;
constexpr uint32_t kHypervTscFrequencyMsrAvailable = 1u << 8;
constexpr uint32_t kMsrHypervTscFrequency = 0x40000022;
constexpr uint32_t kMsrHypervApicFrequency = 0x40000023;
// Time idle APs get to acknowledge a SmpRunOnAllCpus() wakeup IPI, and how
// often the IPI is resent meanwhile.
constexpr int64_t kDispatchAckTimeoutMs = 10000;
constexpr int64_t kWakeupIpiResendIntervalUs = 1000;
// Time the panicking CPU waits for the other CPUs to park.
constexpr int64_t kPanicParkTimeoutMs = 100;
// Time the other CPUs get to handle a TLB shootdown request.
constexpr int64_t kTlbShootdownTimeoutMs = 10000;

struct [[gnu::packed]] AcpiRsdpV1 {
  char signature[8];
  uint8_t checksum;
  char oem_id[6];
  uint8_t revision;
  uint32_t rsdt_address;
};

struct [[gnu::packed]] AcpiRsdpV2 {
  AcpiRsdpV1 v1;
  uint32_t length;
  uint64_t xsdt_address;
  uint8_t extended_checksum;
  uint8_t reserved[3];
};

struct [[gnu::packed]] AcpiSdtHeader {
  char signature[4];
  uint32_t length;
  uint8_t revision;
  uint8_t checksum;
  char oem_id[6];
  char oem_table_id[8];
  uint32_t oem_revision;
  uint32_t creator_id;
  uint32_t creator_revision;
};

struct [[gnu::packed]] AcpiMadt {
  AcpiSdtHeader header;
  uint32_t local_apic_address;
  uint32_t flags;
};

struct [[gnu::packed]] AcpiMadtEntryHeader {
  uint8_t type;
  uint8_t length;
};

struct [[gnu::packed]] AcpiMadtLocalApic {
  AcpiMadtEntryHeader header;
  uint8_t acpi_processor_id;
  uint8_t apic_id;
  uint32_t flags;
};

struct [[gnu::packed]] AcpiMadtLocalApicOverride {
  AcpiMadtEntryHeader header;
  uint16_t reserved;
  uint64_t local_apic_address;
};

struct [[gnu::packed]] AcpiMadtLocalX2Apic {
  AcpiMadtEntryHeader header;
  uint16_t reserved;
  uint32_t x2apic_id;
  uint32_t flags;
  uint32_t acpi_processor_uid;
};

// The machine installed by SmpSetHardware().
std::shared_ptr<SmpHardware> g_hardware;
// Discovered CPU topology and per-CPU runtime state populated during SmpInit().
SmpTopology g_topology = {};
// Cache-line-aligned per-CPU state blocks (`0 .. SmpCpuCount() - 1`).
std::unique_ptr<CpuLocal[]> g_cpu_locals;
// Calibrated periodic Local APIC timer initial count computed on the BSP.
uint32_t g_calibrated_timer_initial_count = 0;
// Number of CPUs (BSP + APs) that have completed initialization and come
// online.
std::atomic<int> g_online_cpu_count{0};
// Handshake flag set to 1 by the currently booting AP in ApKernelEntry() once
// it has initialized its Local APIC and recorded its hardware state, signaling
// the BSP that it may proceed to wake the next AP.
std::atomic<int> g_ap_boot_done{0};
// True once SmpInit() has completed topology discovery and AP bring-up.
bool g_smp_initialized = false;
// True if the Local APICs run in x2APIC (MSR) mode. SmpInit() selects it
// whenever the CPU supports x2APIC (or firmware already enabled it), and
// EnableLocalApic() switches the BSP and then every AP to it; all Local APIC
// accesses then use the 0x800+ MSR range.
bool g_x2apic_mode = false;
// TSC ticks per millisecond, which time the delays and timeouts (see
// ReadClockTicks()).
uint64_t g_tsc_ticks_per_ms = kFallbackTscTicksPerMs;
// True if KVM paravirtualized steal time (`KVM_FEATURE_STEAL_TIME`) is
// supported and registered on every online CPU.
bool g_steal_time = false;

// Panic-stop state (see SmpStopOtherCpusForPanic()):
// True once the BSP's Local APIC is enabled, so that the panic path can send
// IPIs.
std::atomic<bool> g_panic_ipi_ready{false};
// Set before the stop NMI is sent; makes the NMI handler and starting APs park.
std::atomic<bool> g_panic_stop_requested{false};
// Number of CPUs parked by ParkCpuForPanic().
std::atomic<int> g_panic_parked_cpu_count{0};

// State for synchronous multi-CPU work dispatch in SmpRunOnAllCpus():
// Guards against concurrent or re-entrant SmpRunOnAllCpus() calls.
std::atomic<bool> g_dispatch_in_progress{false};
// Callback function and opaque context pointer for the current dispatch round.
std::atomic<SmpWorkFn> g_work_fn{nullptr};
std::atomic<void*> g_work_context{nullptr};
// Monotonically increasing dispatch epoch incremented by the BSP to announce a
// new work item before waking halted APs via maskable `0x21` IPIs.
std::atomic<int64_t> g_work_epoch{0};
// Set by the BSP to `g_work_epoch` once all APs have acknowledged the wakeup,
// releasing all CPUs to execute `g_work_fn` simultaneously.
std::atomic<int64_t> g_work_start_epoch{0};
// Per-CPU epoch acknowledgment written by AP `i` upon waking from `hlt`, so the
// BSP knows whether an AP received the wakeup IPI or needs a retry.
std::unique_ptr<std::atomic<int64_t>[]> g_ap_ack_epoch;
// Per-CPU completion epoch written by AP `i` after `g_work_fn` returns, before
// the AP goes back to `sti; hlt`.
std::unique_ptr<std::atomic<int64_t>[]> g_ap_done_epoch;

// TLB shootdown state (see SmpTlbShootdown()):
// Number of shootdowns requested so far. A CPU increments it after it has
// changed the page tables, and then waits until every CPU has handled it.
std::atomic<int64_t> g_tlb_shootdown_requests{0};
// Per CPU: the value of `g_tlb_shootdown_requests` that CPU `i` read before
// its last TLB flush, so it has handled every request up to that one. Only
// CPU `i` writes it, always with interrupts disabled.
std::unique_ptr<std::atomic<int64_t>[]> g_tlb_shootdown_handled;

static constexpr int64_t AlignUp(const int64_t value, const int64_t alignment) {
  return (value + alignment - 1) & ~(alignment - 1);
}

static SmpHardware& Hardware() {
  DCHECK(g_hardware != nullptr);
  return *g_hardware;
}

// MultibootPhysicalMapper through the installed hardware.
static const void* MapPhysicalMemory(const uintptr_t phys_addr,
                                     const int64_t size) {
  return Hardware().MapPhysicalMemory(phys_addr, size);
}

// Returns a pointer to the `size` bytes of physical memory at `phys_addr`, or
// nullptr if they cannot be mapped.
template <typename T>
static const T* MapPhysical(const uintptr_t phys_addr, const int64_t size) {
  return static_cast<const T*>(MapPhysicalMemory(phys_addr, size));
}

static bool BytesMatch(const char* const actual,    //
                       const char* const expected,  //
                       const int length) {
  DCHECK(actual != nullptr);
  DCHECK(expected != nullptr);
  DCHECK(length > 0);
  for (int i = 0; i < length; ++i) {
    if (actual[i] != expected[i]) {
      return false;
    }
  }
  return true;
}

static bool VerifyChecksum(const void* const data, const int64_t length) {
  DCHECK(data != nullptr);
  DCHECK(length > 0);
  const uint8_t* const bytes = static_cast<const uint8_t*>(data);
  uint8_t sum = 0;
  for (int64_t i = 0; i < length; ++i) {
    sum = static_cast<uint8_t>(sum + bytes[i]);
  }
  return sum == 0;
}

static bool ValidateRsdpV1Bounded(const uintptr_t rsdp_addr,
                                  const uintptr_t rsdp_max_end_addr,
                                  const uintptr_t max_physical_addr,
                                  uintptr_t* const out_rsdt_addr) {
  DCHECK(rsdp_addr != 0);
  DCHECK(max_physical_addr > 0);
  DCHECK(out_rsdt_addr != nullptr);
  *out_rsdt_addr = 0;

  constexpr int64_t kV1Size = sizeof(AcpiRsdpV1);
  const uintptr_t structure_limit = (rsdp_max_end_addr < max_physical_addr)
                                        ? rsdp_max_end_addr
                                        : max_physical_addr;
  if (rsdp_addr >= structure_limit ||
      (structure_limit - rsdp_addr) < static_cast<uintptr_t>(kV1Size)) {
    return false;
  }
  const AcpiRsdpV1* const rsdp = MapPhysical<AcpiRsdpV1>(rsdp_addr, kV1Size);
  if (rsdp == nullptr || !BytesMatch(rsdp->signature, "RSD PTR ", 8) ||
      !VerifyChecksum(rsdp, kV1Size)) {
    return false;
  }
  const uintptr_t rsdt_addr = rsdp->rsdt_address;
  if (rsdt_addr == 0 || rsdt_addr >= max_physical_addr) {
    return false;
  }
  *out_rsdt_addr = rsdt_addr;
  return true;
}

static bool ParseRsdpBounded(const uintptr_t rsdp_addr,
                             const uintptr_t rsdp_max_end_addr,
                             const uintptr_t max_physical_addr,
                             uintptr_t* const out_rsdt_addr,
                             uintptr_t* const out_xsdt_addr) {
  DCHECK(rsdp_addr != 0);
  DCHECK(max_physical_addr > 0);
  DCHECK(out_rsdt_addr != nullptr);
  DCHECK(out_xsdt_addr != nullptr);
  *out_rsdt_addr = 0;
  *out_xsdt_addr = 0;

  constexpr int64_t kV1Size = sizeof(AcpiRsdpV1);
  constexpr int64_t kV2Size = sizeof(AcpiRsdpV2);
  const uintptr_t structure_limit = (rsdp_max_end_addr < max_physical_addr)
                                        ? rsdp_max_end_addr
                                        : max_physical_addr;
  if (rsdp_addr >= structure_limit ||
      (structure_limit - rsdp_addr) < static_cast<uintptr_t>(kV1Size)) {
    return false;
  }

  const AcpiRsdpV1* const rsdp_v1 = MapPhysical<AcpiRsdpV1>(rsdp_addr, kV1Size);
  if (rsdp_v1 == nullptr || !BytesMatch(rsdp_v1->signature, "RSD PTR ", 8) ||
      !VerifyChecksum(rsdp_v1, kV1Size)) {
    return false;
  }

  const uintptr_t rsdt_addr = rsdp_v1->rsdt_address;
  if (rsdt_addr != 0 && rsdt_addr < max_physical_addr) {
    *out_rsdt_addr = rsdt_addr;
  }

  if (rsdp_v1->revision >= 2) {
    const AcpiRsdpV2* const rsdp_v2 =
        (structure_limit - rsdp_addr) < static_cast<uintptr_t>(kV2Size)
            ? nullptr
            : MapPhysical<AcpiRsdpV2>(rsdp_addr, kV2Size);
    if (rsdp_v2 == nullptr) {
      *out_rsdt_addr = 0;
      return false;
    }
    const int64_t v2_len = rsdp_v2->length;
    const AcpiRsdpV2* const full_rsdp_v2 =
        (v2_len < kV2Size || v2_len > kMaxRsdpBytes ||
         (structure_limit - rsdp_addr) < static_cast<uintptr_t>(v2_len))
            ? nullptr
            : MapPhysical<AcpiRsdpV2>(rsdp_addr, v2_len);
    if (full_rsdp_v2 == nullptr || !VerifyChecksum(full_rsdp_v2, v2_len)) {
      *out_rsdt_addr = 0;
      return false;
    }
    const uintptr_t xsdt_addr = full_rsdp_v2->xsdt_address;
    if (xsdt_addr != 0 && xsdt_addr < max_physical_addr) {
      *out_xsdt_addr = xsdt_addr;
    }
  }

  return *out_rsdt_addr != 0 || *out_xsdt_addr != 0;
}

static bool FindRsdpTagsInMultiboot2(const uintptr_t multiboot_info_addr,
                                     const uintptr_t max_physical_addr,
                                     uintptr_t* const out_new_addr,
                                     uintptr_t* const out_new_end_addr,
                                     uintptr_t* const out_old_addr,
                                     uintptr_t* const out_old_end_addr) {
  DCHECK(multiboot_info_addr != 0);
  DCHECK(max_physical_addr > 0);
  DCHECK(out_new_addr != nullptr);
  DCHECK(out_new_end_addr != nullptr);
  DCHECK(out_old_addr != nullptr);
  DCHECK(out_old_end_addr != nullptr);
  *out_new_addr = 0;
  *out_new_end_addr = 0;
  *out_old_addr = 0;
  *out_old_end_addr = 0;

  int64_t total_size = 0;
  const uint8_t* const info = Multiboot2MapInfo(
      multiboot_info_addr, max_physical_addr, &MapPhysicalMemory, &total_size);
  if (info == nullptr) {
    return false;
  }

  constexpr int64_t kTagHeaderSize = sizeof(Multiboot2Tag);
  constexpr int64_t kMinOldTagSize = kTagHeaderSize + sizeof(AcpiRsdpV1);
  constexpr int64_t kMinNewTagSize = kTagHeaderSize + sizeof(AcpiRsdpV2);
  for (Multiboot2TagIterator it(info, total_size); !it.Done(); it.Next()) {
    const Multiboot2Tag& tag = it.tag();
    const int64_t tag_size = tag.size;
    const uintptr_t tag_addr = multiboot_info_addr + it.offset();
    const uintptr_t payload_addr = tag_addr + kTagHeaderSize;
    const uintptr_t tag_end_addr = tag_addr + tag_size;
    if (tag.type == kMultiboot2TagAcpiNew && tag_size >= kMinNewTagSize &&
        *out_new_addr == 0) {
      uintptr_t rsdt = 0;
      uintptr_t xsdt = 0;
      if (ParseRsdpBounded(payload_addr,       //
                           tag_end_addr,       //
                           max_physical_addr,  //
                           &rsdt,              //
                           &xsdt)) {
        *out_new_addr = payload_addr;
        *out_new_end_addr = tag_end_addr;
      } else if (*out_old_addr == 0 &&
                 ValidateRsdpV1Bounded(payload_addr,       //
                                       tag_end_addr,       //
                                       max_physical_addr,  //
                                       &rsdt)) {
        *out_old_addr = payload_addr;
        *out_old_end_addr = tag_end_addr;
      }
    } else if (tag.type == kMultiboot2TagAcpiOld &&
               tag_size >= kMinOldTagSize && *out_old_addr == 0) {
      uintptr_t rsdt = 0;
      if (ValidateRsdpV1Bounded(payload_addr,       //
                                tag_end_addr,       //
                                max_physical_addr,  //
                                &rsdt)) {
        *out_old_addr = payload_addr;
        *out_old_end_addr = tag_end_addr;
      }
    }
  }

  return *out_new_addr != 0 || *out_old_addr != 0;
}

static const AcpiSdtHeader* MapAndValidateSdt(
    const uintptr_t table_addr,         //
    const uintptr_t max_physical_addr,  //
    const char* const expected_sig) {
  DCHECK(max_physical_addr > 0);
  DCHECK(expected_sig != nullptr);
  constexpr int64_t kHeaderSize = sizeof(AcpiSdtHeader);
  if (table_addr == 0 || table_addr >= max_physical_addr ||
      (max_physical_addr - table_addr) < static_cast<uintptr_t>(kHeaderSize)) {
    return nullptr;
  }
  const AcpiSdtHeader* const header =
      MapPhysical<AcpiSdtHeader>(table_addr, kHeaderSize);
  if (header == nullptr || !BytesMatch(header->signature, expected_sig, 4)) {
    return nullptr;
  }
  const int64_t total_len = header->length;
  if (total_len < kHeaderSize || total_len > kMaxAcpiTableBytes ||
      (max_physical_addr - table_addr) < static_cast<uintptr_t>(total_len)) {
    return nullptr;
  }
  const AcpiSdtHeader* const table =
      MapPhysical<AcpiSdtHeader>(table_addr, total_len);
  if (table == nullptr || !VerifyChecksum(table, total_len)) {
    return nullptr;
  }
  return table;
}

static bool IsValidLocalApicAddress(const uintptr_t addr,
                                    const uintptr_t max_physical_addr) {
  return addr != 0 && (addr & (kPageSize - 1)) == 0 &&
         addr < max_physical_addr &&
         (max_physical_addr - addr) >= static_cast<uintptr_t>(kPageSize);
}

// Records an enabled MADT processor entry in `topology`, whose index 0 is
// reserved for the BSP (`topology->bsp_apic_id`); `*bsp_listed` tracks whether
// the BSP's own entry has been seen. Skips duplicate APIC IDs, and counts the
// entries that do not fit into `capacity` in `ignored_cpu_count`.
static void AddDiscoveredCpu(SmpTopology* const topology,       //
                             const int capacity,                //
                             const uint32_t apic_id,            //
                             const uint32_t acpi_processor_id,  //
                             bool* const bsp_listed) {
  DCHECK(topology != nullptr);
  DCHECK(topology->cpus != nullptr);
  DCHECK(topology->cpu_count >= 1);
  DCHECK(apic_id != kInvalidX2ApicId);
  DCHECK(bsp_listed != nullptr);
  if (apic_id == topology->bsp_apic_id) {
    if (!*bsp_listed) {
      topology->cpus[0].acpi_processor_id = acpi_processor_id;
      *bsp_listed = true;
    }
    return;
  }
  for (int i = 1; i < topology->cpu_count; ++i) {
    if (topology->cpus[i].apic_id == apic_id) {
      return;
    }
  }
  if (topology->cpu_count >= capacity) {
    ++topology->ignored_cpu_count;
    return;
  }
  CpuInfo& cpu = topology->cpus[topology->cpu_count];
  cpu = {};
  cpu.apic_id = apic_id;
  cpu.acpi_processor_id = acpi_processor_id;
  ++topology->cpu_count;
}

static bool TryDiscoverFromRsdp(const uintptr_t rsdp_addr,
                                const uintptr_t rsdp_max_end_addr,
                                const uintptr_t max_physical_addr,
                                const uint32_t bsp_apic_id,
                                const bool force_v1_only,
                                SmpTopology* const out_topology) {
  DCHECK(rsdp_addr != 0);
  DCHECK(max_physical_addr > 0);
  DCHECK(bsp_apic_id != kInvalidX2ApicId);
  DCHECK(out_topology != nullptr);
  uintptr_t rsdt_addr = 0;
  uintptr_t xsdt_addr = 0;
  if (force_v1_only) {
    if (!ValidateRsdpV1Bounded(rsdp_addr,          //
                               rsdp_max_end_addr,  //
                               max_physical_addr,  //
                               &rsdt_addr)) {
      return false;
    }
  } else if (!ParseRsdpBounded(rsdp_addr,          //
                               rsdp_max_end_addr,  //
                               max_physical_addr,  //
                               &rsdt_addr,         //
                               &xsdt_addr)) {
    return false;
  }

  const uintptr_t madt_addr =
      SmpFindMadtInSdt(rsdt_addr, xsdt_addr, max_physical_addr);
  if (madt_addr != 0 &&
      SmpParseMadt(madt_addr, max_physical_addr, bsp_apic_id, out_topology)) {
    return true;
  }

  if (xsdt_addr != 0 && rsdt_addr != 0) {
    const uintptr_t rsdt_madt_addr =
        SmpFindMadtInSdt(rsdt_addr, 0, max_physical_addr);
    if (rsdt_madt_addr != 0 && rsdt_madt_addr != madt_addr) {
      return SmpParseMadt(rsdt_madt_addr,     //
                          max_physical_addr,  //
                          bsp_apic_id,        //
                          out_topology);
    }
  }
  return false;
}

static bool TryDiscoverFromMemoryRange(const uintptr_t range_start,
                                       const int64_t range_length,
                                       const uintptr_t max_physical_addr,
                                       const uint32_t bsp_apic_id,
                                       SmpTopology* const out_topology) {
  DCHECK(range_start != 0);
  DCHECK(range_length > 0);
  DCHECK(max_physical_addr > 0);
  DCHECK(bsp_apic_id != kInvalidX2ApicId);
  DCHECK(out_topology != nullptr);
  if (range_start >= max_physical_addr ||
      (max_physical_addr - range_start) <
          static_cast<uintptr_t>(range_length)) {
    return false;
  }
  const uintptr_t range_end = range_start + range_length;
  uintptr_t cursor = AlignUp(range_start, kRsdpScanAlignment);
  while (cursor < range_end) {
    const uintptr_t rsdp_addr =
        SmpFindRsdpInMemoryRange(cursor, range_end - cursor);
    if (rsdp_addr == 0) {
      break;
    }
    if (TryDiscoverFromRsdp(rsdp_addr,          //
                            range_end,          //
                            max_physical_addr,  //
                            bsp_apic_id,        //
                            false,              //
                            out_topology)) {
      return true;
    }
    cursor = rsdp_addr + kRsdpScanAlignment;
  }
  return false;
}

static void InitCpuLocalSlot(const int cpu_index,              //
                             const uint32_t apic_id,           //
                             const bool online,                //
                             const uintptr_t stack_base_addr,  //
                             const uintptr_t stack_top_addr) {
  DCHECK(g_cpu_locals != nullptr);
  DCHECK(cpu_index >= 0 && cpu_index < g_topology.cpu_count);
  CpuLocal& slot = g_cpu_locals[cpu_index];
  slot.self = &slot;
  slot.cpu_id = cpu_index;
  slot.apic_id = apic_id;
  slot.online = online;
  slot.stack_base = stack_base_addr;
  slot.stack_top = stack_top_addr;
  slot.top_held_lock = nullptr;
  slot.fault_probe = nullptr;
  slot.timer_ticks.store(0, std::memory_order_relaxed);
  slot.ipi_count.store(0, std::memory_order_relaxed);
  slot.context_switches.store(0, std::memory_order_relaxed);
  slot.task_timer_ticks.store(0, std::memory_order_relaxed);
  slot.kvm_steal_time = {};
}

// The stacks of an AP, which SmpInit() allocates as one block of
// kApStackBlockFrames frames: the kernel stack, then one stack per IST slot,
// each directly above a guard page.
struct ApStackBlock {
  uintptr_t base = 0;
  uintptr_t guard_pages[1 + kIstStackCount] = {};
  uintptr_t stack_base = 0;
  uintptr_t stack_top = 0;
  uintptr_t ist_stack_tops[kIstStackCount] = {};
};

static ApStackBlock ApStackBlockAt(const uintptr_t base) {
  ApStackBlock block;
  block.base = base;
  block.guard_pages[0] = base;
  block.stack_base = base + kPageSize;
  block.stack_top = block.stack_base + kApStackSize;
  uintptr_t next = block.stack_top;
  for (int i = 0; i < kIstStackCount; ++i) {
    block.guard_pages[1 + i] = next;
    block.ist_stack_tops[i] = next + kPageSize + kIstStackSize;
    next = block.ist_stack_tops[i];
  }
  DCHECK(next == base + kApStackBlockFrames * kPageSize);
  return block;
}

// The stack block of AP `cpu_index`, which starts with the guard page below
// its kernel stack.
static ApStackBlock ApStackBlockOf(const int cpu_index) {
  DCHECK(cpu_index > 0 && cpu_index < g_topology.cpu_count);
  return ApStackBlockAt(g_topology.cpus[cpu_index].stack_base - kPageSize);
}

static void ResetSmpState() {
  // Unbinds the calling CPU (the BSP), whose CpuLocal is freed below.
#if __STDC_HOSTED__
  ResetCpuLocalForTest();
#else
  internal::g_cpu_local_bound.store(false, std::memory_order_release);
#endif
  g_tsc_ticks_per_ms = kFallbackTscTicksPerMs;
  g_steal_time = false;
  g_x2apic_mode = false;
  g_panic_ipi_ready.store(false, std::memory_order_seq_cst);
  g_panic_stop_requested.store(false, std::memory_order_seq_cst);
  g_panic_parked_cpu_count.store(0, std::memory_order_seq_cst);
  g_topology = {};
  g_cpu_locals.reset();
  g_calibrated_timer_initial_count = 0;
  g_online_cpu_count.store(0, std::memory_order_seq_cst);
  g_ap_boot_done.store(0, std::memory_order_seq_cst);
  g_ap_boot_started.store(0, std::memory_order_seq_cst);
  g_ap_boot_stack = 0;
  g_ap_boot_cpu_local = 0;
  g_ap_boot_cpu_index = 0;
  g_smp_initialized = false;
  g_dispatch_in_progress.store(false, std::memory_order_seq_cst);
  g_work_fn.store(nullptr, std::memory_order_seq_cst);
  g_work_context.store(nullptr, std::memory_order_seq_cst);
  g_work_epoch.store(0, std::memory_order_seq_cst);
  g_work_start_epoch.store(0, std::memory_order_seq_cst);
  g_ap_ack_epoch.reset();
  g_ap_done_epoch.reset();
  g_tlb_shootdown_requests.store(0, std::memory_order_seq_cst);
  g_tlb_shootdown_handled.reset();
}

static uint64_t ReadMsr(const uint32_t msr) { return Hardware().ReadMsr(msr); }

static void WriteMsr(const uint32_t msr, const uint64_t value) {
  Hardware().WriteMsr(msr, value);
}

static void RegisterStealTimeForCurrentCpu() {
  if (!g_steal_time) {
    return;
  }
  CpuLocal* const cpu = CurrentCpuOrNull();
  DCHECK(cpu != nullptr);
  const uintptr_t addr = reinterpret_cast<uintptr_t>(&cpu->kvm_steal_time);
  DCHECK((addr & 63u) == 0);
  WriteMsr(kMsrKvmStealTime,
           static_cast<uint64_t>(addr) | kKvmStealTimeSystemEnabled);
}

static bool IsX2ApicSupported() {
  const CpuidResult leaf1 = Hardware().Cpuid(1, 0);
  return (leaf1.ecx & kCpuidEcxX2ApicBit) != 0 ||
         (ReadMsr(kMsrIa32ApicBase) & kApicBaseX2ApicEnable) != 0;
}

static uint32_t ReadCpuidInitialApicId() {
  SmpHardware& hw = Hardware();
  const uint32_t xapic_id = (hw.Cpuid(1, 0).ebx >> 24) & 0xFFu;
  if (IsX2ApicSupported() && hw.Cpuid(0, 0).eax >= 0x0Bu) {
    const CpuidResult leaf_b = hw.Cpuid(0x0Bu, 0);
    if ((leaf_b.ebx & 0xFFFFu) != 0 || ((leaf_b.ecx >> 8) & 0xFFu) != 0) {
      return leaf_b.edx;
    }
  }
  return xapic_id;
}

// Returns the Local APIC timer's input clock in Hz as reported by CPUID, or 0
// if CPUID does not report it (as on AMD CPUs and most hypervisors). Per the
// Intel SDM ("APIC Timer"), the timer runs at the core crystal clock when leaf
// 0x15 enumerates the TSC/crystal ratio (leaf 0x15 ECX holds the crystal
// frequency, or 0 if that is not enumerated), and at the bus clock otherwise
// (leaf 0x16 ECX[15:0], in MHz).
static int64_t ApicTimerClockHzFromCpuid() {
  SmpHardware& hw = Hardware();
  const uint32_t max_leaf = hw.Cpuid(0, 0).eax;
  if (max_leaf >= 0x15u) {
    const CpuidResult leaf15 = hw.Cpuid(0x15u, 0);
    if (leaf15.ebx != 0) {
      return leaf15.ecx;
    }
  }
  if (max_leaf >= 0x16u) {
    return static_cast<int64_t>(hw.Cpuid(0x16u, 0).ecx & 0xFFFFu) * 1000000;
  }
  return 0;
}

// The TSC and Local APIC timer frequencies in Hz that Hyper-V reports, each 0
// if the kernel does not run on Hyper-V or Hyper-V does not report it.
struct HypervFrequencies {
  int64_t tsc_hz = 0;
  int64_t apic_timer_hz = 0;
};

static HypervFrequencies ReadHypervFrequencies() {
  SmpHardware& hw = Hardware();
  HypervFrequencies frequencies;
  if ((hw.Cpuid(1, 0).ecx & kCpuidEcxHypervisorBit) == 0) {
    return frequencies;
  }
  const CpuidResult vendor = hw.Cpuid(kCpuidHypervisorVendorLeaf, 0);
  if (vendor.eax < kCpuidHypervFeaturesLeaf ||
      vendor.ebx != kHypervSignatureEbx || vendor.ecx != kHypervSignatureEcx ||
      vendor.edx != kHypervSignatureEdx) {
    return frequencies;
  }
  const CpuidResult features = hw.Cpuid(kCpuidHypervFeaturesLeaf, 0);
  if ((features.eax & kHypervAccessFrequencyMsrs) == 0) {
    return frequencies;
  }
  // Reading a synthetic MSR that the partition may not access faults, so the
  // TSC frequency MSR needs its own feature bit (as in Linux).
  if ((features.edx & kHypervTscFrequencyMsrAvailable) != 0) {
    frequencies.tsc_hz =
        static_cast<int64_t>(ReadMsr(kMsrHypervTscFrequency) & INT64_MAX);
  }
  frequencies.apic_timer_hz =
      static_cast<int64_t>(ReadMsr(kMsrHypervApicFrequency) & INT64_MAX);
  return frequencies;
}

static bool IsPlausibleTscRate(const uint64_t tsc_ticks_per_ms) {
  return tsc_ticks_per_ms >= kMinPlausibleTscTicksPerMs &&
         tsc_ticks_per_ms <= kMaxPlausibleTscTicksPerMs;
}

// Returns the periodic Local APIC timer initial count for kApicTimerTargetHz:
// `pit_calibrated_count` if PIT calibration produced one (non-zero), else the
// count for the timer clock that Hyper-V or else CPUID reports, else
// kFallbackApicTimerInitialCount, with a warning since the resulting timer
// rate is unknown. Without PIT calibration, it also takes the TSC rate from
// Hyper-V if Hyper-V reports one.
static uint32_t ChooseApicTimerInitialCount(
    const uint32_t pit_calibrated_count) {
  if (pit_calibrated_count > 0) {
    return pit_calibrated_count;
  }
  const HypervFrequencies hyperv = ReadHypervFrequencies();
  const uint64_t hyperv_tsc_ticks_per_ms =
      static_cast<uint64_t>(hyperv.tsc_hz) / 1000;
  if (IsPlausibleTscRate(hyperv_tsc_ticks_per_ms)) {
    g_tsc_ticks_per_ms = hyperv_tsc_ticks_per_ms;
    UartWrite("[SMP] No PIT calibration; TSC rate from Hyper-V: ");
    UartWriteDec(static_cast<uint64_t>(hyperv.tsc_hz));
    UartWrite(" Hz\n");
  }
  // The timer divides its input clock by 16 (kLapicTimerDivBy16).
  const int64_t hyperv_count = hyperv.apic_timer_hz / (16 * kApicTimerTargetHz);
  if (hyperv_count > 0 && hyperv_count <= static_cast<int64_t>(UINT32_MAX)) {
    UartWrite(
        "[SMP] No PIT calibration; Local APIC timer clock from Hyper-V: ");
    UartWriteDec(static_cast<uint64_t>(hyperv.apic_timer_hz));
    UartWrite(" Hz\n");
    return static_cast<uint32_t>(hyperv_count);
  }
  const int64_t clock_hz = ApicTimerClockHzFromCpuid();
  const int64_t count = clock_hz / (16 * kApicTimerTargetHz);
  if (count > 0 && count <= static_cast<int64_t>(UINT32_MAX)) {
    UartWrite("[SMP] No PIT calibration; Local APIC timer clock from CPUID: ");
    UartWriteDec(static_cast<uint64_t>(clock_hz));
    UartWrite(" Hz\n");
    return static_cast<uint32_t>(count);
  }
  ConsoleWrite(
      "[SMP] WARNING: no PIT or CPUID timer frequency; LAPIC timer rate is "
      "uncalibrated\n");
  return kFallbackApicTimerInitialCount;
}

// Reads the clock that times delays and timeouts: the TSC, at
// g_tsc_ticks_per_ms.
static uint64_t ReadClockTicks() { return Hardware().ReadTimestampCounter(); }

// Converts `micros` (>= 0) to ReadClockTicks() ticks.
static uint64_t ClockTicksForMicros(const int64_t micros) {
  DCHECK(micros >= 0);
  return static_cast<uint64_t>(micros) * g_tsc_ticks_per_ms / 1000;
}

// One iteration of a spin-wait loop.
static void SpinPause() { Hardware().SpinPause(); }

static uint32_t LapicOffsetToX2ApicMsr(const int reg_offset) {
  DCHECK(reg_offset >= 0 && (reg_offset & 0xF) == 0);
  return kMsrX2ApicBase + static_cast<uint32_t>(reg_offset >> 4);
}

static uint32_t LapicRead(const uintptr_t lapic_base, const int reg_offset) {
  DCHECK(g_x2apic_mode || lapic_base != 0);
  DCHECK(reg_offset >= 0);
  if (g_x2apic_mode) {
    return static_cast<uint32_t>(ReadMsr(LapicOffsetToX2ApicMsr(reg_offset)));
  }
  return Hardware().ReadLapicMmio(lapic_base, reg_offset);
}

static void LapicWrite(const uintptr_t lapic_base,  //
                       const int reg_offset,        //
                       const uint32_t value) {
  DCHECK(g_x2apic_mode || lapic_base != 0);
  DCHECK(reg_offset >= 0);
  if (g_x2apic_mode) {
    asm volatile("mfence; lfence" : : : "memory");
    WriteMsr(LapicOffsetToX2ApicMsr(reg_offset), value);
    return;
  }
  Hardware().WriteLapicMmio(lapic_base, reg_offset, value);
  (void)LapicRead(lapic_base, kLapicRegId);
}

static uint32_t ReadLapicId(const uintptr_t lapic_base) {
  DCHECK(g_x2apic_mode || lapic_base != 0);
  const uint32_t raw_id = LapicRead(lapic_base, kLapicRegId);
  if (g_x2apic_mode) {
    return raw_id;
  }
  return (raw_id >> 24) & 0xFFu;
}

static bool WaitForIcrIdle(const uintptr_t lapic_base) {
  DCHECK(g_x2apic_mode || lapic_base != 0);
  if (g_x2apic_mode) {
    return true;
  }
  for (int i = 0; i < kIcrIdlePollLimit; ++i) {
    if ((LapicRead(lapic_base, kLapicRegIcrLow) & kIcrDeliveryPending) == 0) {
      return true;
    }
    SpinPause();
  }
  return (LapicRead(lapic_base, kLapicRegIcrLow) & kIcrDeliveryPending) == 0;
}

static void LapicSendIpiCommand(const uintptr_t lapic_base,
                                const uint32_t apic_id,
                                const uint32_t icr_low) {
  DCHECK(g_x2apic_mode || lapic_base != 0);
  DCHECK(g_x2apic_mode ? (apic_id != kInvalidX2ApicId)
                       : (apic_id < kInvalidXapicId));
  const bool saved_irq = SaveAndDisableInterrupts();
  CHECK(WaitForIcrIdle(lapic_base));
  LapicWrite(lapic_base, kLapicRegEsr, 0);
  (void)LapicRead(lapic_base, kLapicRegEsr);
  if (g_x2apic_mode) {
    const uint64_t icr = (static_cast<uint64_t>(apic_id) << 32) | icr_low;
    asm volatile("mfence; lfence" : : : "memory");
    WriteMsr(LapicOffsetToX2ApicMsr(kLapicRegIcrLow), icr);
    RestoreInterrupts(saved_irq);
    return;
  }
  const uint32_t icr_dest = apic_id << 24;
  LapicWrite(lapic_base, kLapicRegIcrHigh, icr_dest);
  LapicWrite(lapic_base, kLapicRegIcrLow, icr_low);
  CHECK(WaitForIcrIdle(lapic_base));
  RestoreInterrupts(saved_irq);
}

static void EnableLocalApic(const uintptr_t lapic_base) {
  DCHECK(g_x2apic_mode || lapic_base != 0);
  uint64_t apic_base_msr = ReadMsr(kMsrIa32ApicBase);
  if (!g_x2apic_mode && (apic_base_msr & kApicBaseX2ApicEnable) != 0) {
    apic_base_msr &= ~(kApicBaseGlobalEnable | kApicBaseX2ApicEnable);
    WriteMsr(kMsrIa32ApicBase, apic_base_msr);
  }
  if ((apic_base_msr & kApicBaseGlobalEnable) == 0) {
    apic_base_msr |= kApicBaseGlobalEnable;
    WriteMsr(kMsrIa32ApicBase, apic_base_msr);
  }
  if (g_x2apic_mode && (apic_base_msr & kApicBaseX2ApicEnable) == 0) {
    apic_base_msr |= kApicBaseX2ApicEnable;
    WriteMsr(kMsrIa32ApicBase, apic_base_msr);
  }
  LapicWrite(lapic_base, kLapicRegTpr, 0);
  const uint32_t svr = LapicRead(lapic_base, kLapicRegSvr);
  LapicWrite(lapic_base,    //
             kLapicRegSvr,  //
             svr | kLapicSvrSoftwareEnable | kLapicSpuriousVector);
}

static bool IsLongModeActive() {
  return (ReadMsr(kMsrIa32Efer) & kEferLongModeActive) != 0 &&
         Hardware().ReadCs() == kKernelCodeSelector;
}

static uint8_t ReadPitSpeakerPort() {
  return Hardware().InByte(kPitSpeakerPort);
}

static uint32_t ReadLapicTimerCurrentCount(const uintptr_t lapic_base) {
  return LapicRead(lapic_base, kLapicRegTimerCurrCount);
}

// Measures the Local APIC timer, and the TSC rate for ReadClockTicks(), against
// a one-shot ~10 ms countdown of PIT channel 2. Returns the periodic initial
// count for kApicTimerTargetHz, or 0 if there is no PIT, it did not respond, or
// the TSC rate it measured is implausible (the TSC rate then stays at
// kFallbackTscTicksPerMs).
static uint32_t CalibrateLocalApicTimer(const uintptr_t lapic_base) {
  DCHECK(g_x2apic_mode || lapic_base != 0);
  SmpHardware& hw = Hardware();
  LapicWrite(lapic_base, kLapicRegTimerDivConfig, kLapicTimerDivBy16);
  LapicWrite(lapic_base,         //
             kLapicRegLvtTimer,  //
             kLapicTimerMaskBit | static_cast<uint32_t>(kVectorApicTimer));

  const uint8_t speaker_initial = hw.InByte(kPitSpeakerPort);
  const uint8_t gate_disabled =
      static_cast<uint8_t>(speaker_initial & ~kPitSpeakerEnableMask);
  hw.OutByte(kPitSpeakerPort, gate_disabled);

  hw.OutByte(kPitCommandPort, kPitCmdChannel2Mode0);
  hw.OutByte(kPitChannel2DataPort,
             static_cast<uint8_t>(kPitCalibrationReloadCount & 0xFF));
  hw.OutByte(kPitChannel2DataPort,
             static_cast<uint8_t>((kPitCalibrationReloadCount >> 8) & 0xFF));
  // In mode 0, writing the count drives OUT2 low, and with GATE2 low the
  // countdown has not started. A port that reads OUT2 high anyway has no PIT
  // behind it: it reads 0xFF in Hyper-V generation 2 VMs, for example, and
  // would otherwise pass for a countdown that ran out at once.
  if ((hw.InByte(kPitSpeakerPort) & kPitSpeakerOut2Bit) != 0) {
    return 0;
  }

  // Rising edge on GATE2 starts the 8254 Channel 2 Mode 0 one-shot countdown.
  hw.OutByte(kPitSpeakerPort,
             static_cast<uint8_t>(gate_disabled | kPitSpeakerGate2Bit));
  const uint64_t tsc_start = ReadClockTicks();
  LapicWrite(lapic_base, kLapicRegTimerInitCount, 0xFFFFFFFFu);

  const uint32_t initial_count =
      CalibrateApicTimerCount(lapic_base,                //
                              kPitCalibrationPollLimit,  //
                              ReadPitSpeakerPort,        //
                              ReadLapicTimerCurrentCount);
  const uint64_t tsc_elapsed = ReadClockTicks() - tsc_start;

  LapicWrite(lapic_base, kLapicRegTimerInitCount, 0);
  hw.OutByte(kPitSpeakerPort, gate_disabled);
  if (initial_count > 0) {
    // The countdown took kPitCalibrationReloadCount ticks of the PIT clock.
    const uint64_t tsc_ticks_per_ms =
        tsc_elapsed * static_cast<uint64_t>(kPitBaseFrequencyHz) /
        (static_cast<uint64_t>(kPitCalibrationReloadCount) * 1000);
    if (!IsPlausibleTscRate(tsc_ticks_per_ms)) {
      // The PIT did not count at its rate, so neither result holds.
      return 0;
    }
    g_tsc_ticks_per_ms = tsc_ticks_per_ms;
  }
  return initial_count;
}

static void StartLocalApicTimer(const uintptr_t lapic_base,
                                const uint32_t initial_count) {
  DCHECK(g_x2apic_mode || lapic_base != 0);
  DCHECK(initial_count > 0);
  LapicWrite(lapic_base, kLapicRegTimerDivConfig, kLapicTimerDivBy16);
  LapicWrite(lapic_base,         //
             kLapicRegLvtTimer,  //
             kLapicTimerPeriodicMode | static_cast<uint32_t>(kVectorApicTimer));
  LapicWrite(lapic_base, kLapicRegTimerInitCount, initial_count);
}

static void SendFixedIpi(const uintptr_t lapic_base,  //
                         const uint32_t apic_id,      //
                         const uint8_t vector) {
  DCHECK(g_x2apic_mode || lapic_base != 0);
  DCHECK(g_x2apic_mode ? (apic_id != kInvalidX2ApicId)
                       : (apic_id < kInvalidXapicId));
  DCHECK(vector >= static_cast<uint8_t>(kCpuExceptionCount));
  DCHECK(vector != kVectorSpurious);
  LapicSendIpiCommand(
      lapic_base,  //
      apic_id,     //
      kIcrDeliveryFixed | kIcrLevelAssert | static_cast<uint32_t>(vector));
}

static void DelayMicros(const int64_t micros) {
  const uint64_t start = ReadClockTicks();
  const uint64_t ticks = ClockTicksForMicros(micros);
  while (ReadClockTicks() - start < ticks) {
    SpinPause();
  }
}

// Returns true as soon as `flag` is non-zero, or false if it is still 0 after
// `timeout_micros`.
static bool WaitForFlag(const std::atomic<int>& flag,
                        const int64_t timeout_micros) {
  const uint64_t start = ReadClockTicks();
  const uint64_t timeout_ticks = ClockTicksForMicros(timeout_micros);
  while (flag.load(std::memory_order_acquire) == 0) {
    if (ReadClockTicks() - start >= timeout_ticks) {
      return flag.load(std::memory_order_acquire) != 0;
    }
    SpinPause();
  }
  return true;
}

// Sends INIT to the AP with `apic_id`, which resets it into the wait-for-SIPI
// state (assert and de-assert in xAPIC mode; x2APIC has no INIT de-assert),
// and waits the 10 ms that the SDM requires before a SIPI.
static void SendInitIpi(const uintptr_t lapic_base, const uint32_t apic_id) {
  if (g_x2apic_mode) {
    LapicSendIpiCommand(lapic_base,  //
                        apic_id,     //
                        kIcrDeliveryInit | kIcrLevelAssert);
  } else {
    LapicSendIpiCommand(lapic_base,  //
                        apic_id,     //
                        kIcrDeliveryInit | kIcrLevelAssert | kIcrTriggerLevel);
    LapicSendIpiCommand(lapic_base,  //
                        apic_id,     //
                        kIcrDeliveryInit | kIcrTriggerLevel);
  }
  DelayMicros(kInitDelayUs);
}

// Starts the AP with `apic_id` as logical CPU `cpu_index` on the stack that
// ends at `stack_top_addr` with the SDM's INIT-SIPI-SIPI sequence, and waits
// for it to report in from ApKernelEntry(). If it does not do so within
// kApStartupTimeoutMs, sends INIT again, so that the AP cannot start late with
// a stale CPU index, and returns false.
static bool WakeApplicationProcessor(const uintptr_t lapic_base,
                                     const int cpu_index,
                                     const uint32_t apic_id,
                                     const uintptr_t stack_top_addr) {
  DCHECK(g_x2apic_mode || lapic_base != 0);
  DCHECK(cpu_index > 0 && cpu_index < g_topology.cpu_count);
  DCHECK(g_x2apic_mode ? (apic_id != kInvalidX2ApicId)
                       : (apic_id < kInvalidXapicId));
  DCHECK(stack_top_addr != 0 && (stack_top_addr & 0xF) == 0);
  g_ap_boot_stack = stack_top_addr;
  g_ap_boot_cpu_local = reinterpret_cast<uintptr_t>(&g_cpu_locals[cpu_index]);
  g_ap_boot_cpu_index = cpu_index;
  g_ap_boot_started.store(0, std::memory_order_seq_cst);
  g_ap_boot_done.store(0, std::memory_order_seq_cst);

  SendInitIpi(lapic_base, apic_id);
  // Startup IPI (SIPI) with vector 0x08: the AP starts at 0x8000.
  constexpr uint32_t kStartupIcr =
      kIcrDeliveryStartup | kIcrLevelAssert | kApTrampolineVector;
  LapicSendIpiCommand(lapic_base, apic_id, kStartupIcr);
  // The second SIPI is only for an AP that missed the first one; an AP that
  // has already entered the trampoline would ignore it anyway.
  if (!WaitForFlag(g_ap_boot_started, kSipiDelayUs)) {
    LapicSendIpiCommand(lapic_base, apic_id, kStartupIcr);
  }
  if (WaitForFlag(g_ap_boot_done, kApStartupTimeoutMs * 1000)) {
    return true;
  }
  SendInitIpi(lapic_base, apic_id);
  return false;
}

// Parks the calling CPU for good once another CPU has started a panic: counts
// it in g_panic_parked_cpu_count, so that the panicking CPU knows it stopped,
// and halts with interrupts disabled.
[[noreturn]] static void ParkCpuForPanic() {
  g_panic_parked_cpu_count.fetch_add(1, std::memory_order_acq_rel);
  for (;;) {
    Hardware().HaltForever();
  }
}

// NMI handler installed by SmpInit(): parks the CPU if another CPU requested a
// panic stop (see SmpStopOtherCpusForPanic()), and ignores the NMI otherwise.
static void OnPanicStopNmi(InterruptFrame* const /*frame*/) {
  if (g_panic_stop_requested.load(std::memory_order_acquire)) {
    ParkCpuForPanic();
  }
}

// Flushes the TLB of the calling CPU, `cpu_index`, if a TLB shootdown was
// requested since its last flush, and records which requests the flush
// handled. Runs with interrupts disabled.
static void HandleTlbShootdownRequests(const int cpu_index) {
  std::atomic<int64_t>& handled = g_tlb_shootdown_handled[cpu_index];
  // Read the request count first: the flush that follows then also drops
  // every translation that the requesting CPUs removed before they counted
  // their requests (the acquire load makes those page table writes visible).
  const int64_t requests =
      g_tlb_shootdown_requests.load(std::memory_order_acquire);
  if (requests == handled.load(std::memory_order_relaxed)) {
    return;
  }
  Hardware().FlushTlb();
  handled.store(requests, std::memory_order_release);
}

// Handler of the kVectorTlbShootdown IPI that SmpTlbShootdown() sends.
static void OnTlbShootdownIpi(InterruptFrame* const /*frame*/) {
  const CpuLocal* const cpu = CurrentCpuOrNull();
  if (cpu != nullptr && g_tlb_shootdown_handled != nullptr) {
    HandleTlbShootdownRequests(cpu->cpu_id);
  }
}

static bool RunPendingApWork(const int cpu_index,
                             int64_t* const completed_epoch) {
  DCHECK(cpu_index > 0 && cpu_index < g_topology.cpu_count);
  DCHECK(completed_epoch != nullptr);
  const int64_t epoch = g_work_epoch.load(std::memory_order_acquire);
  if (epoch <= *completed_epoch) {
    return false;
  }
  // `work_fn` runs with interrupts enabled (`sti`).
  RestoreInterrupts(true);
  g_ap_ack_epoch[cpu_index].store(epoch, std::memory_order_release);
  while (g_work_start_epoch.load(std::memory_order_acquire) != epoch) {
    SpinPause();
  }
  const SmpWorkFn work_fn = g_work_fn.load(std::memory_order_acquire);
  void* const context = g_work_context.load(std::memory_order_acquire);
  DCHECK(work_fn != nullptr);
  work_fn(cpu_index, context);
  *completed_epoch = epoch;
  g_ap_done_epoch[cpu_index].store(epoch, std::memory_order_release);
  return true;
}

// Waits until every AP has acknowledged dispatch `epoch` from its idle loop,
// resending the wakeup IPI to the stragglers every kWakeupIpiResendIntervalUs.
static void WaitForDispatchAcks(const uintptr_t lapic_base,
                                const int64_t epoch) {
  const int cpu_count = g_topology.cpu_count;
  const uint64_t start = ReadClockTicks();
  const uint64_t timeout_ticks =
      ClockTicksForMicros(kDispatchAckTimeoutMs * 1000);
  const uint64_t resend_ticks = ClockTicksForMicros(kWakeupIpiResendIntervalUs);
  uint64_t last_send = start;
  for (;;) {
    bool all_acked = true;
    for (int i = 1; i < cpu_count && all_acked; ++i) {
      all_acked = g_ap_ack_epoch[i].load(std::memory_order_acquire) == epoch;
    }
    if (all_acked) {
      return;
    }
    const uint64_t now = ReadClockTicks();
    // An AP that is running a task instead of idling never acknowledges.
    const bool aps_acknowledged_in_time = now - start < timeout_ticks;
    CHECK(aps_acknowledged_in_time);
    if (now - last_send >= resend_ticks) {
      for (int i = 1; i < cpu_count; ++i) {
        if (g_ap_ack_epoch[i].load(std::memory_order_acquire) != epoch) {
          SendFixedIpi(lapic_base,                  //
                       g_topology.cpus[i].apic_id,  //
                       kVectorWakeupIpi);
        }
      }
      last_send = now;
    }
    SpinPause();
  }
}

// Removes entry `cpu_index` (> 0) from g_topology.cpus during SmpInit(), before
// the per-CPU arrays exist, moving the entries after it down by one index.
static void EraseTopologyCpu(const int cpu_index) {
  DCHECK(cpu_index > 0 && cpu_index < g_topology.cpu_count);
  for (int i = cpu_index; i + 1 < g_topology.cpu_count; ++i) {
    g_topology.cpus[i] = g_topology.cpus[i + 1];
  }
  --g_topology.cpu_count;
  g_topology.cpus[g_topology.cpu_count] = {};
}

// Drops AP `cpu_index`, which did not start, during SmpInit()'s bring-up loop:
// frees its stack block and moves the APs after it, none of which has been
// started yet, down by one index together with their stacks and CpuLocal
// slots. CPUs 0 .. cpu_index - 1 are online at this point.
static void DropApThatDidNotStart(const int cpu_index) {
  DCHECK(cpu_index > 0 && cpu_index < g_topology.cpu_count);
  // The PMM keeps its bookkeeping in free frames, so map the guard pages again
  // first.
  const ApStackBlock block = ApStackBlockOf(cpu_index);
  for (const uintptr_t guard_page : block.guard_pages) {
    CHECK(Hardware().RemapGuardPage(guard_page));
  }
  PmmFreeFrames(block.base, kApStackBlockFrames);
  EraseTopologyCpu(cpu_index);
  for (int i = cpu_index; i < g_topology.cpu_count; ++i) {
    InitCpuLocalSlot(i,                              //
                     g_topology.cpus[i].apic_id,     //
                     false,                          //
                     g_topology.cpus[i].stack_base,  //
                     g_topology.cpus[i].stack_top);
  }
  CpuLocal& unused_slot = g_cpu_locals[g_topology.cpu_count];
  unused_slot.self = nullptr;
  unused_slot.online = false;
  // An AP that reported in just after the timeout counted itself before INIT
  // stopped it.
  g_online_cpu_count.store(cpu_index, std::memory_order_seq_cst);
}

// Reads the EBDA base address from the BIOS Data Area, or returns 0 if it is
// not a plausible EBDA location.
static uintptr_t ReadEbdaBase() {
  const uint16_t* const ebda_segment =
      MapPhysical<uint16_t>(kBdaEbdaSegmentAddr, sizeof(uint16_t));
  if (ebda_segment == nullptr) {
    return 0;
  }
  const uintptr_t ebda_base = static_cast<uintptr_t>(*ebda_segment) << 4;
  return (ebda_base >= kEbdaMinAddress && ebda_base < kEbdaMaxAddress)
             ? ebda_base
             : 0;
}

}  // namespace

extern "C" void ApKernelEntry(const int cpu_index) {
  DCHECK(cpu_index > 0 && cpu_index < g_topology.cpu_count);
  // The trampoline has bound this AP's CpuLocal (g_ap_boot_cpu_local).
  DCHECK(CurrentCpuOrNull() == &g_cpu_locals[cpu_index]);
  SmpHardware& hw = Hardware();
  const uintptr_t lapic_base = g_topology.local_apic_phys_addr;
  DCHECK(g_x2apic_mode || lapic_base != 0);

  // The TSS first: the kernel IDT's #DF, NMI and #MC gates switch to its IST
  // stacks.
  GdtLoadForCpu(cpu_index, ApStackBlockOf(cpu_index).ist_stack_tops);
  IdtLoad();
  // A CPU that panicked while this one was in the trampoline sent its stop NMI
  // before the kernel IDT was loaded here.
  if (g_panic_stop_requested.load(std::memory_order_acquire)) {
    ParkCpuForPanic();
  }
  // Before this AP runs any task: the x87/SSE/AVX state of user processes,
  // which may move between CPUs, needs the BSP's configuration on every CPU,
  // and so do SMEP and SMAP, which must be on before the AP runs a system
  // call.
  CpuInitAp();
  RegisterStealTimeForCurrentCpu();
  g_cpu_locals[cpu_index].online = true;
  EnableLocalApic(lapic_base);

  CpuInfo& cpu = g_topology.cpus[cpu_index];
  cpu.observed_apic_id = ReadLapicId(lapic_base);
  cpu.observed_rsp = hw.ReadRsp();
  cpu.observed_cr3 = hw.ReadCr3();
  cpu.long_mode_active = IsLongModeActive();
  cpu.online = true;

  StartLocalApicTimer(lapic_base, g_calibrated_timer_initial_count);

  g_online_cpu_count.fetch_add(1, std::memory_order_seq_cst);
  g_ap_boot_done.store(1, std::memory_order_release);

  // The idle loop: runs SmpRunOnAllCpus() work and scheduler tasks, and halts
  // with interrupts enabled when there is neither.
  int64_t completed_epoch = 0;
  for (;;) {
    (void)SaveAndDisableInterrupts();
    if (RunPendingApWork(cpu_index, &completed_epoch)) {
      continue;
    }
    if (TaskPollIdleCpu(cpu_index)) {
      continue;
    }
    if (!hw.HaltUntilInterrupt()) {
      return;
    }
  }
}

uintptr_t SmpFindRsdpInMemoryRange(const uintptr_t range_start,
                                   const int64_t range_length) {
  DCHECK(range_start != 0);
  DCHECK(range_length > 0);
  constexpr int64_t kV1Size = sizeof(AcpiRsdpV1);
  if (range_length < kV1Size || range_start >= kIdentityMapLimit ||
      (kIdentityMapLimit - range_start) <
          static_cast<uintptr_t>(range_length) ||
      MapPhysical<uint8_t>(range_start, range_length) == nullptr) {
    return 0;
  }

  const uintptr_t aligned_start = AlignUp(range_start, kRsdpScanAlignment);
  const uintptr_t range_end = range_start + range_length;
  for (uintptr_t addr = aligned_start; addr + kV1Size <= range_end;
       addr += kRsdpScanAlignment) {
    uintptr_t rsdt = 0;
    uintptr_t xsdt = 0;
    if (ParseRsdpBounded(addr, range_end, kIdentityMapLimit, &rsdt, &xsdt)) {
      return addr;
    }
  }
  return 0;
}

uintptr_t SmpFindMadtInSdt(const uintptr_t rsdt_addr,  //
                           const uintptr_t xsdt_addr,  //
                           const uintptr_t max_physical_addr) {
  DCHECK(rsdt_addr != 0 || xsdt_addr != 0);
  DCHECK(max_physical_addr > 0);
  constexpr int64_t kHeaderSize = sizeof(AcpiSdtHeader);

  if (xsdt_addr != 0) {
    const AcpiSdtHeader* const xsdt =
        MapAndValidateSdt(xsdt_addr, max_physical_addr, "XSDT");
    if (xsdt != nullptr) {
      const int64_t payload_bytes = xsdt->length - kHeaderSize;
      const int entry_count = payload_bytes / sizeof(uint64_t);
      const uint8_t* const entries_base =
          reinterpret_cast<const uint8_t*>(xsdt) + kHeaderSize;
      for (int i = 0; i < entry_count; ++i) {
        uint64_t entry_phys = 0;
        const uint8_t* const src = entries_base + i * sizeof(uint64_t);
        for (int b = 0; b < static_cast<int>(sizeof(uint64_t)); ++b) {
          entry_phys |= static_cast<uint64_t>(src[b]) << (b * 8);
        }
        const AcpiSdtHeader* const candidate =
            MapAndValidateSdt(entry_phys, max_physical_addr, "APIC");
        if (candidate != nullptr && candidate->length >= sizeof(AcpiMadt)) {
          return entry_phys;
        }
      }
    }
  }

  if (rsdt_addr != 0) {
    const AcpiSdtHeader* const rsdt =
        MapAndValidateSdt(rsdt_addr, max_physical_addr, "RSDT");
    if (rsdt != nullptr) {
      const int64_t payload_bytes = rsdt->length - kHeaderSize;
      const int entry_count = payload_bytes / sizeof(uint32_t);
      const uint8_t* const entries_base =
          reinterpret_cast<const uint8_t*>(rsdt) + kHeaderSize;
      for (int i = 0; i < entry_count; ++i) {
        uint32_t raw_entry = 0;
        const uint8_t* const src = entries_base + i * sizeof(uint32_t);
        for (int b = 0; b < static_cast<int>(sizeof(uint32_t)); ++b) {
          raw_entry |= static_cast<uint32_t>(src[b]) << (b * 8);
        }
        const uintptr_t entry_phys = raw_entry;
        const AcpiSdtHeader* const candidate =
            MapAndValidateSdt(entry_phys, max_physical_addr, "APIC");
        if (candidate != nullptr && candidate->length >= sizeof(AcpiMadt)) {
          return entry_phys;
        }
      }
    }
  }

  return 0;
}

bool SmpParseMadt(const uintptr_t madt_addr,          //
                  const uintptr_t max_physical_addr,  //
                  const uint32_t bsp_apic_id,         //
                  SmpTopology* const out_topology) {
  DCHECK(madt_addr != 0);
  DCHECK(max_physical_addr > 0);
  DCHECK(bsp_apic_id != kInvalidX2ApicId);
  DCHECK(out_topology != nullptr);
  *out_topology = {};

  const AcpiSdtHeader* const header =
      MapAndValidateSdt(madt_addr, max_physical_addr, "APIC");
  constexpr int64_t kMadtHeaderSize = sizeof(AcpiMadt);
  if (header == nullptr || header->length < kMadtHeaderSize) {
    return false;
  }

  const AcpiMadt* const madt = reinterpret_cast<const AcpiMadt*>(header);
  const uint8_t* const madt_bytes = reinterpret_cast<const uint8_t*>(header);
  const int64_t madt_length = madt->header.length;
  out_topology->local_apic_phys_addr =
      IsValidLocalApicAddress(madt->local_apic_address, max_physical_addr)
          ? static_cast<uintptr_t>(madt->local_apic_address)
          : kDefaultLocalApicPhysAddr;
  out_topology->bsp_apic_id = bsp_apic_id;

  int candidate_count = 0;
  int64_t offset = kMadtHeaderSize;
  constexpr int64_t kEntryHeaderSize = sizeof(AcpiMadtEntryHeader);
  while (offset + kEntryHeaderSize <= madt_length) {
    const AcpiMadtEntryHeader* const entry =
        reinterpret_cast<const AcpiMadtEntryHeader*>(madt_bytes + offset);
    const int64_t entry_len = entry->length;
    if (entry_len < kEntryHeaderSize || offset + entry_len > madt_length) {
      break;
    }

    if (entry->type == kMadtTypeLocalApic) {
      if (entry_len < static_cast<int64_t>(sizeof(AcpiMadtLocalApic))) {
        break;
      }
      const AcpiMadtLocalApic* const lapic =
          reinterpret_cast<const AcpiMadtLocalApic*>(entry);
      if ((lapic->flags & kMadtLapicFlagEnabled) != 0 &&
          lapic->apic_id != kInvalidXapicId) {
        ++candidate_count;
      }
    } else if (entry->type == kMadtTypeLocalApicOverride) {
      if (entry_len < static_cast<int64_t>(sizeof(AcpiMadtLocalApicOverride))) {
        break;
      }
      const AcpiMadtLocalApicOverride* const override_entry =
          reinterpret_cast<const AcpiMadtLocalApicOverride*>(entry);
      if (IsValidLocalApicAddress(override_entry->local_apic_address,
                                  max_physical_addr)) {
        out_topology->local_apic_phys_addr = override_entry->local_apic_address;
      }
    } else if (entry->type == kMadtTypeLocalX2Apic) {
      if (entry_len < static_cast<int64_t>(sizeof(AcpiMadtLocalX2Apic))) {
        break;
      }
      const AcpiMadtLocalX2Apic* const x2apic =
          reinterpret_cast<const AcpiMadtLocalX2Apic*>(entry);
      if ((x2apic->flags & kMadtLapicFlagEnabled) != 0 &&
          x2apic->x2apic_id != kInvalidX2ApicId) {
        ++candidate_count;
      }
    }

    offset += entry_len;
  }

  // Index 0 is reserved for the BSP, so at most kMaxSupportedCpus - 1 other
  // processors fit.
  if (candidate_count > kMaxSupportedCpus - 1) {
    candidate_count = kMaxSupportedCpus - 1;
  }
  if (candidate_count <= 0) {
    *out_topology = {};
    return false;
  }

  const int capacity = candidate_count + 1;
  out_topology->cpus.reset(new (std::nothrow) CpuInfo[capacity]());
  if (out_topology->cpus == nullptr) {
    *out_topology = {};
    return false;
  }

  out_topology->cpus[0].apic_id = bsp_apic_id;
  out_topology->cpu_count = 1;
  bool bsp_listed = false;
  offset = kMadtHeaderSize;
  while (offset + kEntryHeaderSize <= madt_length) {
    const AcpiMadtEntryHeader* const entry =
        reinterpret_cast<const AcpiMadtEntryHeader*>(madt_bytes + offset);
    const int64_t entry_len = entry->length;
    if (entry_len < kEntryHeaderSize || offset + entry_len > madt_length) {
      break;
    }

    if (entry->type == kMadtTypeLocalApic) {
      if (entry_len < static_cast<int64_t>(sizeof(AcpiMadtLocalApic))) {
        break;
      }
      const AcpiMadtLocalApic* const lapic =
          reinterpret_cast<const AcpiMadtLocalApic*>(entry);
      if ((lapic->flags & kMadtLapicFlagEnabled) != 0 &&
          lapic->apic_id != kInvalidXapicId) {
        AddDiscoveredCpu(out_topology,              //
                         capacity,                  //
                         lapic->apic_id,            //
                         lapic->acpi_processor_id,  //
                         &bsp_listed);
      }
    } else if (entry->type == kMadtTypeLocalApicOverride) {
      if (entry_len < static_cast<int64_t>(sizeof(AcpiMadtLocalApicOverride))) {
        break;
      }
    } else if (entry->type == kMadtTypeLocalX2Apic) {
      if (entry_len < static_cast<int64_t>(sizeof(AcpiMadtLocalX2Apic))) {
        break;
      }
      const AcpiMadtLocalX2Apic* const x2apic =
          reinterpret_cast<const AcpiMadtLocalX2Apic*>(entry);
      if ((x2apic->flags & kMadtLapicFlagEnabled) != 0 &&
          x2apic->x2apic_id != kInvalidX2ApicId) {
        AddDiscoveredCpu(out_topology,                //
                         capacity,                    //
                         x2apic->x2apic_id,           //
                         x2apic->acpi_processor_uid,  //
                         &bsp_listed);
      }
    }

    offset += entry_len;
  }

  out_topology->cpus[0].is_bsp = true;
  out_topology->cpus[0].online = true;
  out_topology->cpus[0].observed_apic_id = bsp_apic_id;
  return true;
}

bool SmpDiscoverTopology(const uint32_t multiboot_magic,
                         const uint64_t multiboot_info_addr,
                         const uintptr_t ebda_base,
                         const uintptr_t bios_rom_base,
                         const uintptr_t max_physical_addr,
                         const uint32_t bsp_apic_id,
                         SmpTopology* const out_topology) {
  DCHECK(max_physical_addr > 0);
  DCHECK(bsp_apic_id != kInvalidX2ApicId);
  DCHECK(out_topology != nullptr);
  *out_topology = {};

  if (multiboot_magic == kMultiboot2Magic && multiboot_info_addr != 0) {
    uintptr_t rsdp_new_addr = 0;
    uintptr_t rsdp_new_end_addr = 0;
    uintptr_t rsdp_old_addr = 0;
    uintptr_t rsdp_old_end_addr = 0;
    if (FindRsdpTagsInMultiboot2(multiboot_info_addr,  //
                                 max_physical_addr,    //
                                 &rsdp_new_addr,       //
                                 &rsdp_new_end_addr,   //
                                 &rsdp_old_addr,       //
                                 &rsdp_old_end_addr)) {
      if (rsdp_new_addr != 0 && TryDiscoverFromRsdp(rsdp_new_addr,      //
                                                    rsdp_new_end_addr,  //
                                                    max_physical_addr,  //
                                                    bsp_apic_id,        //
                                                    false,              //
                                                    out_topology)) {
        return true;
      }
      if (rsdp_old_addr != 0 && TryDiscoverFromRsdp(rsdp_old_addr,      //
                                                    rsdp_old_end_addr,  //
                                                    max_physical_addr,  //
                                                    bsp_apic_id,        //
                                                    true,               //
                                                    out_topology)) {
        return true;
      }
    }
  }

  if (ebda_base != 0) {
    int64_t ebda_scan_len = kEbdaScanBytes;
    if (ebda_base >= kEbdaMinAddress && ebda_base < kEbdaMaxAddress) {
      const int64_t max_ebda_bytes = kEbdaMaxAddress - ebda_base;
      if (ebda_scan_len > max_ebda_bytes) {
        ebda_scan_len = max_ebda_bytes;
      }
    }
    if (TryDiscoverFromMemoryRange(ebda_base,          //
                                   ebda_scan_len,      //
                                   max_physical_addr,  //
                                   bsp_apic_id,        //
                                   out_topology)) {
      return true;
    }
  }

  if (bios_rom_base != 0) {
    if (TryDiscoverFromMemoryRange(bios_rom_base,      //
                                   kBiosRomScanBytes,  //
                                   max_physical_addr,  //
                                   bsp_apic_id,        //
                                   out_topology)) {
      return true;
    }
  }

  return false;
}

uint32_t ComputeApicTimerInitialCount(const uint64_t elapsed_apic_ticks,
                                      const int pit_reload_count,
                                      const int target_hz) {
  DCHECK(pit_reload_count > 0 && pit_reload_count <= kMaxPitReloadCount);
  DCHECK(target_hz > 0 && target_hz <= kPitBaseFrequencyHz);
  if (elapsed_apic_ticks == 0) {
    return 0;
  }
  const uint64_t freq_hz = static_cast<uint64_t>(kPitBaseFrequencyHz);
  if (elapsed_apic_ticks > UINT64_MAX / freq_hz) {
    return 0xFFFFFFFFu;
  }
  const uint64_t numerator = elapsed_apic_ticks * freq_hz;
  const uint64_t denominator = static_cast<uint64_t>(pit_reload_count) *
                               static_cast<uint64_t>(target_hz);
  const uint64_t ticks_per_period = numerator / denominator;
  if (ticks_per_period > 0xFFFFFFFFULL) {
    return 0xFFFFFFFFu;
  }
  return static_cast<uint32_t>(ticks_per_period);
}

uint32_t CalibrateApicTimerCount(
    const uintptr_t lapic_base,               //
    const int max_polls,                      //
    const PitSpeakerReadFn read_pit_speaker,  //
    const ApicTimerCurrentCountReadFn read_apic_current_count) {
  DCHECK(max_polls > 0);
  DCHECK(read_pit_speaker != nullptr);
  DCHECK(read_apic_current_count != nullptr);

  bool terminal_count_reached = false;
  for (int poll = 0; poll < max_polls; ++poll) {
    if ((read_pit_speaker() & kPitSpeakerOut2Bit) != 0) {
      terminal_count_reached = true;
      break;
    }
    asm volatile("pause" : : : "memory");
  }

  const uint32_t current_count = read_apic_current_count(lapic_base);
  if (!terminal_count_reached) {
    return 0;
  }

  const uint64_t elapsed_ticks = 0xFFFFFFFFULL - current_count;
  return ComputeApicTimerInitialCount(elapsed_ticks,               //
                                      kPitCalibrationReloadCount,  //
                                      kApicTimerTargetHz);
}

void SmpSetHardware(std::shared_ptr<SmpHardware> hardware) {
  DCHECK(hardware != nullptr);
  g_hardware = std::move(hardware);
  ResetSmpState();
}

void SmpInit(const uint32_t multiboot_magic,
             const uint64_t multiboot_info_addr) {
  ResetSmpState();
  if (!IdtIsInitialized()) {
    IdtInit();
  }
  SmpHardware& hw = Hardware();

  const uint32_t bsp_initial_apic_id = ReadCpuidInitialApicId();
  const uintptr_t ebda_base = ReadEbdaBase();
  const uintptr_t bios_rom_base = kBiosRomSearchBase;

  // The CPUID initial APIC ID only selects a provisional BSP entry, which is
  // enough to find the Local APIC. Its ID register is authoritative.
  CHECK(SmpDiscoverTopology(multiboot_magic,      //
                            multiboot_info_addr,  //
                            ebda_base,            //
                            bios_rom_base,        //
                            kIdentityMapLimit,    //
                            bsp_initial_apic_id,  //
                            &g_topology));

  g_x2apic_mode = IsX2ApicSupported();
  const uintptr_t lapic_base = g_topology.local_apic_phys_addr;
  if (!g_x2apic_mode) {
    CHECK(hw.MapLocalApicPage(lapic_base));
  }
  EnableLocalApic(lapic_base);
  const uint32_t bsp_apic_id = ReadLapicId(lapic_base);
  IdtRegisterHandler(kVectorNmi, OnPanicStopNmi);
  g_panic_ipi_ready.store(true, std::memory_order_release);
  if (bsp_apic_id != g_topology.bsp_apic_id) {
    // Otherwise the BSP's own MADT entry would be taken for an AP, and the BSP
    // would send INIT to itself.
    UartWrite("[SMP] BSP Local APIC ID ");
    UartWriteDec(bsp_apic_id);
    UartWrite(" differs from its CPUID initial APIC ID ");
    UartWriteDec(g_topology.bsp_apic_id);
    UartWrite("; using the Local APIC ID\n");
    CHECK(SmpDiscoverTopology(multiboot_magic,      //
                              multiboot_info_addr,  //
                              ebda_base,            //
                              bios_rom_base,        //
                              kIdentityMapLimit,    //
                              bsp_apic_id,          //
                              &g_topology));
    DCHECK(g_topology.local_apic_phys_addr == lapic_base);
  }
  // In xAPIC mode, IPI destinations are 8-bit APIC IDs.
  for (int i = 1; !g_x2apic_mode && i < g_topology.cpu_count;) {
    if (g_topology.cpus[i].apic_id < kInvalidXapicId) {
      ++i;
      continue;
    }
    UartWrite("[SMP] Skipping APIC ID ");
    UartWriteDec(g_topology.cpus[i].apic_id);
    UartWrite(": not addressable in xAPIC mode\n");
    EraseTopologyCpu(i);
  }

  const int cpu_count = g_topology.cpu_count;
  CHECK(cpu_count >= 1);
  g_cpu_locals.reset(new CpuLocal[cpu_count]());
  CHECK(g_cpu_locals != nullptr);
  g_ap_ack_epoch.reset(new std::atomic<int64_t>[cpu_count]());
  CHECK(g_ap_ack_epoch != nullptr);
  g_ap_done_epoch.reset(new std::atomic<int64_t>[cpu_count]());
  CHECK(g_ap_done_epoch != nullptr);
  g_tlb_shootdown_handled.reset(new std::atomic<int64_t>[cpu_count]());
  CHECK(g_tlb_shootdown_handled != nullptr);
  IdtRegisterHandler(kVectorTlbShootdown, OnTlbShootdownIpi);

  const StackBounds bsp_stack = hw.BspStack();
  g_topology.cpus[0].stack_base = bsp_stack.base;
  g_topology.cpus[0].stack_top = bsp_stack.top;
  g_topology.cpus[0].observed_rsp = hw.ReadRsp();
  g_topology.cpus[0].observed_cr3 = hw.ReadCr3();
  g_topology.cpus[0].long_mode_active = IsLongModeActive();
  DCHECK(g_topology.cpus[0].apic_id == bsp_apic_id);
  g_topology.cpus[0].observed_apic_id = bsp_apic_id;
  g_topology.cpus[0].is_bsp = true;
  g_topology.cpus[0].online = true;
  InitCpuLocalSlot(0,                              //
                   g_topology.cpus[0].apic_id,     //
                   true,                           //
                   g_topology.cpus[0].stack_base,  //
                   g_topology.cpus[0].stack_top);
  BindCpuLocal(&g_cpu_locals[0]);
  g_steal_time =
      HasKvmStealTime([&hw](const uint32_t leaf, const uint32_t subleaf) {
        return hw.Cpuid(leaf, subleaf);
      });
  RegisterStealTimeForCurrentCpu();
  g_online_cpu_count.store(1, std::memory_order_seq_cst);

  g_calibrated_timer_initial_count =
      ChooseApicTimerInitialCount(CalibrateLocalApicTimer(lapic_base));

  ConsoleWrite("[SMP] Discovered CPUs: ");
  ConsoleWriteDec(g_topology.cpu_count);
  ConsoleWrite(", BSP APIC ID: ");
  ConsoleWriteDec(g_topology.bsp_apic_id);
  ConsoleWrite(", LAPIC base: ");
  ConsoleWriteHex(g_topology.local_apic_phys_addr);
  ConsoleWrite("\n");
  if (g_topology.ignored_cpu_count > 0) {
    UartWrite("[SMP] Ignoring ");
    UartWriteDec(g_topology.ignored_cpu_count);
    UartWrite(" more processor entries (limit: ");
    UartWriteDec(kMaxSupportedCpus);
    UartWrite(" CPUs)\n");
  }

  UartWrite("[SMP] CPU 0 (APIC ID ");
  UartWriteDec(g_topology.cpus[0].apic_id);
  UartWrite(", BSP): online\n");

  if (g_topology.cpu_count > 1) {
    // Every guard page is unmapped before the first AP starts, because only
    // this CPU's TLB is flushed.
    for (int i = 1; i < g_topology.cpu_count; ++i) {
      const uintptr_t stack_block = PmmAllocFrames(kApStackBlockFrames);
      CHECK(stack_block != 0);
      const ApStackBlock block = ApStackBlockAt(stack_block);
      for (const uintptr_t guard_page : block.guard_pages) {
        CHECK(hw.UnmapGuardPage(guard_page));
      }
      g_topology.cpus[i].stack_base = block.stack_base;
      g_topology.cpus[i].stack_top = block.stack_top;
      InitCpuLocalSlot(i,                              //
                       g_topology.cpus[i].apic_id,     //
                       false,                          //
                       g_topology.cpus[i].stack_base,  //
                       g_topology.cpus[i].stack_top);
    }

    hw.InstallApTrampoline();
    for (int i = 1; i < g_topology.cpu_count;) {
      const uint32_t apic_id = g_topology.cpus[i].apic_id;
      // An INIT to the BSP's own APIC ID would halt the BSP.
      CHECK(apic_id != g_topology.cpus[0].apic_id);
      const bool started = WakeApplicationProcessor(
          lapic_base, i, apic_id, g_topology.cpus[i].stack_top);
      if (!started) {
        UartWrite("[SMP] CPU ");
        UartWriteDec(i);
        UartWrite(" (APIC ID ");
        UartWriteDec(apic_id);
        UartWrite("): did not start; continuing without it\n");
        DropApThatDidNotStart(i);
        continue;
      }
      CHECK(g_topology.cpus[i].online);
      CHECK(g_cpu_locals[i].online);

      UartWrite("[SMP] CPU ");
      UartWriteDec(i);
      UartWrite(" (APIC ID ");
      UartWriteDec(apic_id);
      UartWrite(", AP): online\n");
      ++i;
    }
    hw.RestoreApTrampoline();
  }

  StartLocalApicTimer(lapic_base, g_calibrated_timer_initial_count);

  g_smp_initialized = true;
  CHECK(SmpOnlineCpuCount() == g_topology.cpu_count);

  // `sti`
  RestoreInterrupts(true);

  UartWrite("[SMP] Local APIC timer calibrated: initial_count=");
  UartWriteDec(g_calibrated_timer_initial_count);
  UartWrite(", target_hz=");
  UartWriteDec(kApicTimerTargetHz);
  UartWrite("\n");

  UartWrite("[SMP] Online CPUs: ");
  UartWriteDec(SmpOnlineCpuCount());
  UartWrite("/");
  UartWriteDec(g_topology.cpu_count);
  UartWrite("\n");
  UartWrite(g_steal_time ? "[SMP] Steal time: KVM\n"
                         : "[SMP] Steal time: none\n");
}

int SmpCpuCount() {
  DCHECK(g_smp_initialized);
  return g_topology.cpu_count;
}

int SmpOnlineCpuCount() {
  DCHECK(g_smp_initialized);
  return g_online_cpu_count.load(std::memory_order_acquire);
}

const CpuInfo* SmpGetCpuInfo(const int index) {
  DCHECK(g_smp_initialized);
  DCHECK(index >= 0 && index < g_topology.cpu_count);
  return &g_topology.cpus[index];
}

CpuLocal* SmpGetCpuLocal(const int cpu_index) {
  DCHECK(g_smp_initialized);
  DCHECK(cpu_index >= 0 && cpu_index < g_topology.cpu_count);
  return &g_cpu_locals[cpu_index];
}

uintptr_t SmpLocalApicPhysAddr() {
  DCHECK(g_smp_initialized);
  return g_topology.local_apic_phys_addr;
}

bool SmpIsX2ApicEnabled() { return g_x2apic_mode; }

uint32_t SmpCalibratedTimerInitialCount() {
  DCHECK(g_smp_initialized);
  return g_calibrated_timer_initial_count;
}

void SmpSetLocalApicTimerInitialCount(const uint32_t initial_count) {
  const uintptr_t lapic_base = g_topology.local_apic_phys_addr;
  if ((!g_x2apic_mode && lapic_base == 0) || initial_count == 0) {
    return;
  }
  LapicWrite(lapic_base, kLapicRegTimerInitCount, initial_count);
}

void SmpSendLocalApicEoi() {
  const uintptr_t lapic_base = g_topology.local_apic_phys_addr;
  DCHECK(g_x2apic_mode || lapic_base != 0);
  LapicWrite(lapic_base, kLapicRegEoi, 0);
}

void SmpSendIpi(const int target_cpu_index, const uint8_t vector) {
  DCHECK(g_smp_initialized);
  DCHECK(target_cpu_index >= 0 && target_cpu_index < g_topology.cpu_count);
  DCHECK(vector >= static_cast<uint8_t>(kCpuExceptionCount));
  DCHECK(vector != kVectorSpurious);
  SendFixedIpi(g_topology.local_apic_phys_addr,            //
               g_topology.cpus[target_cpu_index].apic_id,  //
               vector);
}

void SmpSendNmi(const int target_cpu_index) {
  DCHECK(g_smp_initialized);
  DCHECK(target_cpu_index >= 0 && target_cpu_index < g_topology.cpu_count);
  LapicSendIpiCommand(g_topology.local_apic_phys_addr,
                      g_topology.cpus[target_cpu_index].apic_id,
                      kIcrDeliveryNmi | kIcrLevelAssert);
}

void SmpRunOnAllCpus(const SmpWorkFn work_fn, void* const context) {
  DCHECK(g_smp_initialized);
  DCHECK(work_fn != nullptr);
  DCHECK(CurrentCpuId() == 0);
  DCHECK(!HoldsAnyIrqSpinLock());
  DCHECK(AreInterruptsEnabled());
  const int cpu_count = g_topology.cpu_count;
  DCHECK(cpu_count >= 1);
  DCHECK(SmpOnlineCpuCount() == cpu_count);
  const bool dispatch_already_running =
      g_dispatch_in_progress.exchange(true, std::memory_order_acq_rel);
  CHECK(!dispatch_already_running);

  int64_t epoch = 0;
  if (cpu_count > 1) {
    // Interrupts stay disabled until every AP has picked up the work, so that
    // the caller cannot be preempted and resumed on an AP, which would then
    // never acknowledge.
    const bool irq_was_enabled = SaveAndDisableInterrupts();
    const uintptr_t lapic_base = g_topology.local_apic_phys_addr;
    DCHECK(g_x2apic_mode || lapic_base != 0);
    g_work_fn.store(work_fn, std::memory_order_release);
    g_work_context.store(context, std::memory_order_release);
    epoch = g_work_epoch.fetch_add(1, std::memory_order_acq_rel) + 1;
    for (int i = 1; i < cpu_count; ++i) {
      SendFixedIpi(lapic_base, g_topology.cpus[i].apic_id, kVectorWakeupIpi);
    }
    WaitForDispatchAcks(lapic_base, epoch);
    g_work_start_epoch.store(epoch, std::memory_order_release);
    RestoreInterrupts(irq_was_enabled);
  }

  work_fn(0, context);

  // No timeout: `work_fn` may legitimately run for a long time.
  for (int i = 1; i < cpu_count; ++i) {
    while (g_ap_done_epoch[i].load(std::memory_order_acquire) != epoch) {
      SpinPause();
    }
  }

  g_dispatch_in_progress.store(false, std::memory_order_release);
}

void SmpStopOtherCpusForPanic() {
  if (!g_panic_ipi_ready.load(std::memory_order_acquire)) {
    return;
  }
  g_panic_stop_requested.store(true, std::memory_order_seq_cst);
  const uintptr_t lapic_base = g_topology.local_apic_phys_addr;
  if (g_x2apic_mode) {
    asm volatile("mfence; lfence" : : : "memory");
    WriteMsr(LapicOffsetToX2ApicMsr(kLapicRegIcrLow), kPanicStopIcr);
  } else {
    // Unlike LapicSendIpiCommand(), do not CHECK that the ICR becomes idle.
    (void)WaitForIcrIdle(lapic_base);
    LapicWrite(lapic_base, kLapicRegIcrHigh, 0);
    LapicWrite(lapic_base, kLapicRegIcrLow, kPanicStopIcr);
  }
  // Give the other CPUs a moment to park, so that they stop writing to the
  // consoles before the panic message goes out.
  const int other_cpu_count =
      g_online_cpu_count.load(std::memory_order_acquire) - 1;
  const uint64_t start = ReadClockTicks();
  const uint64_t timeout_ticks =
      ClockTicksForMicros(kPanicParkTimeoutMs * 1000);
  while (g_panic_parked_cpu_count.load(std::memory_order_acquire) <
             other_cpu_count &&
         ReadClockTicks() - start < timeout_ticks) {
    SpinPause();
  }
}

void SmpTlbShootdown() {
  DCHECK(!HoldsAnyIrqSpinLock());
  const ScopedIrqDisable irq_disable;
  SmpHardware& hw = Hardware();
  if (!g_smp_initialized) {
    // Only this CPU runs before SmpInit() starts the APs.
    DCHECK(g_online_cpu_count.load(std::memory_order_acquire) <= 1);
    hw.FlushTlb();
    return;
  }
  const int cpu_count = g_topology.cpu_count;
  const int self = CurrentCpuId();
  // The caller's page table changes happen before this increment (acq_rel),
  // and every CPU reads the count before it flushes for it.
  const int64_t request =
      g_tlb_shootdown_requests.fetch_add(1, std::memory_order_acq_rel) + 1;
  HandleTlbShootdownRequests(self);
  const uintptr_t lapic_base = g_topology.local_apic_phys_addr;
  for (int i = 0; i < cpu_count; ++i) {
    if (i != self) {
      SendFixedIpi(lapic_base, g_topology.cpus[i].apic_id, kVectorTlbShootdown);
    }
  }

  const uint64_t start = ReadClockTicks();
  const uint64_t timeout_ticks =
      ClockTicksForMicros(kTlbShootdownTimeoutMs * 1000);
  for (int i = 0; i < cpu_count; ++i) {
    while (g_tlb_shootdown_handled[i].load(std::memory_order_acquire) <
           request) {
      // A CPU that waits in its own shootdown with interrupts disabled can't
      // take this CPU's IPI, and this CPU can't take its IPI either: each
      // handles the other's request here instead.
      HandleTlbShootdownRequests(self);
      const bool cpus_responded_in_time =
          ReadClockTicks() - start < timeout_ticks;
      CHECK(cpus_responded_in_time);
      SpinPause();
    }
  }
}

bool SmpHasStealTime() { return g_steal_time; }

int64_t SmpOwnStealNsIrqsOff() {
  DCHECK(!AreInterruptsEnabled());
  if (!g_steal_time) {
    return 0;
  }
  const CpuLocal* const cpu = CurrentCpuOrNullIrqsOff();
  return (cpu != nullptr) ? ReadKvmStealNs(cpu->kvm_steal_time) : -1;
}

int64_t SmpStealNsOfCpu(const int cpu_index) {
  if (!g_smp_initialized || cpu_index < 0 ||
      cpu_index >= g_topology.cpu_count || g_cpu_locals == nullptr) {
    return -1;
  }
  if (!g_steal_time) {
    return 0;
  }
  return ReadKvmStealNs(g_cpu_locals[cpu_index].kvm_steal_time);
}

int64_t SmpTimerTickNsUpperBound() {
  const uint32_t count = (g_calibrated_timer_initial_count > 0)
                             ? g_calibrated_timer_initial_count
                             : kFallbackApicTimerInitialCount;
  return TimerTickNsUpperBound(count);
}

int64_t SmpTimerTickNsLowerBound() {
  const uint32_t count = (g_calibrated_timer_initial_count > 0)
                             ? g_calibrated_timer_initial_count
                             : kFallbackApicTimerInitialCount;
  return TimerTickNsLowerBound(count);
}

#if __STDC_HOSTED__
uintptr_t SmpFindRsdpInMultiboot2ForTest(const uintptr_t multiboot_info_addr,
                                         const uintptr_t max_physical_addr) {
  DCHECK(multiboot_info_addr != 0);
  DCHECK(max_physical_addr > 0);
  uintptr_t rsdp_new_addr = 0;
  uintptr_t rsdp_new_end_addr = 0;
  uintptr_t rsdp_old_addr = 0;
  uintptr_t rsdp_old_end_addr = 0;
  if (!FindRsdpTagsInMultiboot2(multiboot_info_addr,  //
                                max_physical_addr,    //
                                &rsdp_new_addr,       //
                                &rsdp_new_end_addr,   //
                                &rsdp_old_addr,       //
                                &rsdp_old_end_addr)) {
    return 0;
  }
  return (rsdp_new_addr != 0) ? rsdp_new_addr : rsdp_old_addr;
}

bool SmpParseRsdpForTest(const uintptr_t rsdp_addr,          //
                         const uintptr_t max_physical_addr,  //
                         uintptr_t* const out_rsdt_addr,     //
                         uintptr_t* const out_xsdt_addr) {
  DCHECK(rsdp_addr != 0);
  DCHECK(max_physical_addr > 0);
  DCHECK(out_rsdt_addr != nullptr);
  DCHECK(out_xsdt_addr != nullptr);
  return ParseRsdpBounded(rsdp_addr,          //
                          max_physical_addr,  //
                          max_physical_addr,  //
                          out_rsdt_addr,      //
                          out_xsdt_addr);
}

uint32_t SmpReadCpuidInitialApicIdForTest() { return ReadCpuidInitialApicId(); }
#endif

}  // namespace protos
