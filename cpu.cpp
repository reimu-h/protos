#include "cpu.h"

#include <atomic>
#include <cstdint>
#include <type_traits>

#include "check.h"
#include "format.h"
#include "fpu.h"
#include "uart.h"

// boot.S reads the flag with `cmp byte ptr`.
extern "C" {
std::atomic<bool> g_cpu_smap_enabled{false};
static_assert(sizeof(g_cpu_smap_enabled) == 1);
static_assert(std::is_standard_layout_v<std::atomic<bool>>);
}

namespace protos {

namespace {

// The name of a group of XCR0 state components in the boot report.
struct XstateComponentName {
  uint64_t xcr0_bits = 0;
  const char* name = nullptr;
};

constexpr XstateComponentName kXstateComponentNames[] = {
    {kXcr0X87, "x87"},
    {kXcr0Sse, "SSE"},
    {kXcr0Avx, "AVX"},
    {kXcr0Avx512, "AVX-512"},
};

// Chosen by CpuInitBsp() on the BSP before SmpInit() starts the APs, which
// read it in CpuInitAp().
CpuConfig g_config;

#if __STDC_HOSTED__
std::atomic<int> g_host_ap_init_count{0};
#endif

}  // namespace

CpuidLeaves CpuReadCpuidLeaves(const CpuidFn cpuid) {
  DCHECK(cpuid != nullptr);
  CpuidLeaves leaves;
  const CpuidResult leaf0 = cpuid(0, 0);
  leaves.max_basic_leaf = leaf0.eax;
  leaves.vendor_ebx = leaf0.ebx;
  leaves.vendor_edx = leaf0.edx;
  leaves.vendor_ecx = leaf0.ecx;
  if (leaves.max_basic_leaf >= 1) {
    leaves.leaf1 = cpuid(1, 0);
  }
  if (leaves.max_basic_leaf >= 7) {
    leaves.leaf7 = cpuid(7, 0);
  }
  if (leaves.max_basic_leaf >= 0xD) {
    leaves.leaf_xsave = cpuid(0xD, 0);
  }
  leaves.max_extended_leaf = cpuid(kCpuidExtendedLeafMax, 0).eax;
  if (leaves.max_extended_leaf >= kCpuidExtendedLeaf8) {
    leaves.leaf_ext8 = cpuid(kCpuidExtendedLeaf8, 0);
  }
  return leaves;
}

// Returns true if CPUID.0 names AMD ("AuthenticAMD").
static bool IsAmd(const CpuidLeaves& leaves) {
  return leaves.vendor_ebx == kCpuidVendorAmdEbx &&
         leaves.vendor_edx == kCpuidVendorAmdEdx &&
         leaves.vendor_ecx == kCpuidVendorAmdEcx;
}

CpuConfig CpuChooseConfig(const CpuidLeaves& leaves) {
  CpuConfig config;
  constexpr uint32_t kFxsrSse = kCpuid1EdxFxsr | kCpuid1EdxSse | kCpuid1EdxSse2;
  config.fxsr = (leaves.leaf1.edx & kFxsrSse) == kFxsrSse;
  // Like Linux (X86_BUG_FXSAVE_LEAK, which also requires family 6 or later,
  // as every 64-bit AMD CPU is), for FXSAVE and XSAVE alike.
  config.reset_x87_pointers =
      IsAmd(leaves) && (leaves.leaf_ext8.ebx & kCpuidExt8EbxXsaveErPtr) == 0;
  // SMEP and SMAP depend neither on each other nor on the register-state
  // format, so they come before the FXSAVE returns below. Only leaf 7 reports
  // them.
  const uint32_t leaf7_ebx =
      (leaves.max_basic_leaf >= 7) ? leaves.leaf7.ebx : 0;
  config.smep = (leaf7_ebx & kCpuid7EbxSmep) != 0;
  config.smap = (leaf7_ebx & kCpuid7EbxSmap) != 0;
  if (!config.fxsr || (leaves.leaf1.ecx & kCpuid1EcxXsave) == 0 ||
      leaves.max_basic_leaf < 0xD) {
    return config;
  }
  const uint64_t supported =
      (static_cast<uint64_t>(leaves.leaf_xsave.edx) << 32) |
      leaves.leaf_xsave.eax;
  // Like Linux, keep FXSAVE if XSAVE can't hold the x87 and SSE state.
  constexpr uint64_t kX87Sse = kXcr0X87 | kXcr0Sse;
  if ((supported & kX87Sse) != kX87Sse) {
    return config;
  }
  uint64_t xcr0 = kX87Sse;
  // XSETBV takes AVX only with SSE, and the AVX-512 components only all
  // together and with AVX.
  if ((supported & kXcr0Avx) != 0 && (leaves.leaf1.ecx & kCpuid1EcxAvx) != 0) {
    xcr0 |= kXcr0Avx;
    if ((supported & kXcr0Avx512) == kXcr0Avx512 &&
        (leaves.leaf7.ebx & kCpuid7EbxAvx512f) != 0) {
      xcr0 |= kXcr0Avx512;
    }
  }
  config.xsave = true;
  config.xcr0 = xcr0;
  return config;
}

bool CpuSupportsConfig(const CpuidLeaves& leaves, const CpuConfig& config) {
  // The configuration that the CPU would choose for itself enables every
  // feature that it supports.
  const CpuConfig own = CpuChooseConfig(leaves);
  if (config.fxsr && !own.fxsr) {
    return false;
  }
  if (config.xsave && (!own.xsave || (own.xcr0 & config.xcr0) != config.xcr0)) {
    return false;
  }
  if (own.reset_x87_pointers && !config.reset_x87_pointers) {
    return false;
  }
  if ((config.smep && !own.smep) || (config.smap && !own.smap)) {
    return false;
  }
  return true;
}

uint64_t CpuCr4ForConfig(const uint64_t cr4, const CpuConfig& config) {
  constexpr uint64_t kManaged =
      kCr4Osxsave | kCr4Smep | kCr4Smap | kCr4Fsgsbase;
  uint64_t value = (cr4 & ~kManaged) | kCr4Osfxsr | kCr4Osxmmexcpt;
  if (config.xsave) {
    value |= kCr4Osxsave;
  }
  if (config.smep) {
    value |= kCr4Smep;
  }
  if (config.smap) {
    value |= kCr4Smap;
  }
  return value;
}

void CpuWriteBootReport(const CpuConfig& config,   //
                        const int fpu_state_size,  //
                        const uint64_t cr4,        //
                        void (*const write)(const char* text)) {
  DCHECK(write != nullptr);
  char hex[kFormatHexBufferSize];
  char dec[kFormatDecBufferSize];
  write("[CPU] FPU state: ");
  if (config.xsave) {
    write("XSAVE, XCR0=");
    write(FormatHex(config.xcr0, hex));
    write(" (");
    const char* separator = "";
    for (const XstateComponentName& component : kXstateComponentNames) {
      if ((config.xcr0 & component.xcr0_bits) == component.xcr0_bits) {
        write(separator);
        write(component.name);
        separator = " ";
      }
    }
    write("), ");
  } else {
    write("FXSAVE, ");
  }
  write(FormatDec(static_cast<uint64_t>(fpu_state_size), dec));
  write(" bytes\n");
  write(config.reset_x87_pointers ? "[CPU] x87 pointer reset: enabled\n"
                                  : "[CPU] x87 pointer reset: disabled\n");
  write((cr4 & kCr4Smep) != 0 ? "[CPU] SMEP: enabled" : "[CPU] SMEP: disabled");
  write((cr4 & kCr4Smap) != 0 ? ", SMAP: enabled\n" : ", SMAP: disabled\n");
}

#if !__STDC_HOSTED__
static CpuidResult ExecuteCpuid(const uint32_t leaf, const uint32_t subleaf) {
  CpuidResult result;
  asm volatile("cpuid"
               : "=a"(result.eax), "=b"(result.ebx), "=c"(result.ecx),
                 "=d"(result.edx)
               : "a"(leaf), "c"(subleaf));
  return result;
}

static uint64_t ReadCr4() {
  uint64_t value = 0;
  asm volatile("mov %0, cr4" : "=r"(value));
  return value;
}

static void WriteCr4(const uint64_t value) {
  asm volatile("mov cr4, %0" : : "r"(value) : "memory");
}

static void WriteXcr0(const uint64_t value) {
  asm volatile("xsetbv"
               :
               : "c"(0), "a"(static_cast<uint32_t>(value)),
                 "d"(static_cast<uint32_t>(value >> 32))
               : "memory");
}

// Applies `config` to the calling CPU and returns the size of its register
// state (fpu.h). CR4 comes first (with SMEP and SMAP as chosen), because
// XSETBV raises #UD while CR4.OSXSAVE is clear, and the size is read last,
// because CPUID.(EAX=0DH,ECX=0):EBX is the XSAVE area size of the XCR0 in
// effect.
static int ApplyConfig(const CpuConfig& config) {
  WriteCr4(CpuCr4ForConfig(ReadCr4(), config));
  if (!config.xsave) {
    return kFpuFxsaveSize;
  }
  WriteXcr0(config.xcr0);
  return static_cast<int>(ExecuteCpuid(0xD, 0).ebx);
}

void CpuInitBsp() {
  const CpuConfig config = CpuChooseConfig(CpuReadCpuidLeaves(ExecuteCpuid));
  const bool cpu_has_fxsr_sse_sse2 = config.fxsr;
  CHECK(cpu_has_fxsr_sse_sse2);
  // Before CR4.SMAP: once it is set, a copy without STAC would fault.
  g_cpu_smap_enabled.store(config.smap, std::memory_order_relaxed);
  const int fpu_state_size = ApplyConfig(config);
  const FpuConfig fpu_config = {
      .xsave = config.xsave,
      .xcr0 = config.xcr0,
      .state_size = fpu_state_size,
      .reset_x87_pointers = config.reset_x87_pointers,
  };
  // CHECK-fails if CPUID reported an implausible XSAVE area size.
  FpuInit(fpu_config);
  g_config = config;
  // What the BSP runs with: CR4 as read back after ApplyConfig().
  CpuWriteBootReport(config, fpu_state_size, ReadCr4(), UartWrite);
}

void CpuInitAp() {
  const bool ap_supports_bsp_config =
      CpuSupportsConfig(CpuReadCpuidLeaves(ExecuteCpuid), g_config);
  CHECK(ap_supports_bsp_config);
  const bool ap_fpu_state_fits = ApplyConfig(g_config) <= FpuStateSize();
  CHECK(ap_fpu_state_fits);
}
#else
void CpuInitAp() {
  g_host_ap_init_count.fetch_add(1, std::memory_order_relaxed);
}

void CpuResetForTest() {
  g_host_ap_init_count.store(0, std::memory_order_relaxed);
  g_cpu_smap_enabled.store(false, std::memory_order_relaxed);
}

int CpuGetHostApInitCountForTest() {
  return g_host_ap_init_count.load(std::memory_order_relaxed);
}

void CpuSetSmapEnabledForTest(const bool enabled) {
  g_cpu_smap_enabled.store(enabled, std::memory_order_relaxed);
}
#endif

const CpuConfig& CpuGetConfig() { return g_config; }

}  // namespace protos
