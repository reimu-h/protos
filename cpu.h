#ifndef PROTOS_CPU_H_
#define PROTOS_CPU_H_

#include <atomic>
#include <cstdint>

#include "smp_hardware.h"

// Whether SMAP is enabled (`CpuConfig::smap`): CpuInitBsp() sets it right
// before it enables SMAP on the BSP, which is before SmpInit() starts the APs
// (each enables SMAP in CpuInitAp()) and before any user page exists, and
// nothing changes it afterwards. The interrupt entry stubs (boot.S) execute
// CLAC, and the user-copy helpers (user_access.h) STAC and CLAC, only if it is
// set: both instructions raise #UD on CPUs without SMAP. (They are valid on a
// CPU with SMAP whether or not CR4.SMAP is set yet.) Always false in host
// builds unless a test sets it (CpuSetSmapEnabledForTest()).
extern "C" std::atomic<bool> g_cpu_smap_enabled;

namespace protos {

// CPU configuration
// =================
//
// The kernel configures every CPU the same way, from the features that CPUID
// reports on the BSP: CpuInitBsp() chooses the configuration and applies it to
// the BSP, and CpuInitAp() applies the same configuration to each AP. The
// choice is a pure function of CPUID values (CpuChooseConfig()); only the two
// appliers touch the CPU.
//
// The configuration says how the kernel keeps the x87/MMX/SSE/AVX register
// state of user processes (fpu.h): with XSAVE, and the state components that
// it enables in XCR0, if the CPU has XSAVE, else with FXSAVE.
//
// It also turns on SMEP and SMAP if the CPU has them, each on its own: with
// SMEP, the kernel faults if it executes code in a user page; with SMAP, if
// it reads or writes a user page other than through the user-copy helpers
// (user_access.h).
//
// The control state is the kernel's own, whatever the firmware or the boot
// loader left: boot.S loads complete values into CR0, CR4 and IA32_EFER, the
// configuration decides every bit of CR4 (CpuCr4ForConfig()), and the
// appliers load MSRs that would otherwise keep stale values, such as the
// SYSCALL and SYSENTER entry points and the PAT (CpuMsrValuesFor()).

// CR4 bits (Intel SDM Vol. 3A, 2.5).
constexpr uint64_t kCr4Pae = uint64_t{1} << 5;
constexpr uint64_t kCr4Mce = uint64_t{1} << 6;
constexpr uint64_t kCr4Osfxsr = uint64_t{1} << 9;
constexpr uint64_t kCr4Osxmmexcpt = uint64_t{1} << 10;
constexpr uint64_t kCr4Fsgsbase = uint64_t{1} << 16;
constexpr uint64_t kCr4Osxsave = uint64_t{1} << 18;
constexpr uint64_t kCr4Smep = uint64_t{1} << 20;
constexpr uint64_t kCr4Smap = uint64_t{1} << 21;

// The CPUID feature flags that the configuration depends on.
constexpr uint32_t kCpuid1EdxMce = uint32_t{1} << 7;
constexpr uint32_t kCpuid1EdxSep = uint32_t{1} << 11;
constexpr uint32_t kCpuid1EdxPat = uint32_t{1} << 16;
constexpr uint32_t kCpuid1EdxFxsr = uint32_t{1} << 24;
constexpr uint32_t kCpuid1EdxSse = uint32_t{1} << 25;
constexpr uint32_t kCpuid1EdxSse2 = uint32_t{1} << 26;
constexpr uint32_t kCpuid1EcxXsave = uint32_t{1} << 26;
constexpr uint32_t kCpuid1EcxAvx = uint32_t{1} << 28;
constexpr uint32_t kCpuid7EbxSmep = uint32_t{1} << 7;
constexpr uint32_t kCpuid7EbxAvx512f = uint32_t{1} << 16;
constexpr uint32_t kCpuid7EbxSmap = uint32_t{1} << 20;
// CPUID.0:EBX, EDX and ECX of AMD CPUs: "Auth", "enti", "cAMD".
constexpr uint32_t kCpuidVendorAmdEbx = 0x68747541;
constexpr uint32_t kCpuidVendorAmdEdx = 0x69746E65;
constexpr uint32_t kCpuidVendorAmdEcx = 0x444D4163;
// The extended CPUID leaf whose EAX is the highest extended leaf, the
// extended leaf with NX, and the extended leaf with XSaveErPtr.
constexpr uint32_t kCpuidExtendedLeafMax = 0x80000000;
constexpr uint32_t kCpuidExtendedLeaf1 = 0x80000001;
constexpr uint32_t kCpuidExtendedLeaf8 = 0x80000008;
// CPUID.80000001H:EDX.NX: the no-execute bit of page-table entries, which is
// reserved unless IA32_EFER.NXE is set.
constexpr uint32_t kCpuidExt1EdxNx = uint32_t{1} << 20;
// CPUID.80000008H:EBX.XSaveErPtr (AMD): FXSAVE and XSAVE always save, and
// FXRSTOR and XRSTOR always restore, the last x87 instruction and operand
// pointers and opcode.
constexpr uint32_t kCpuidExt8EbxXsaveErPtr = uint32_t{1} << 2;

// The CPUID leaves that the configuration depends on, as one CPU reports them.
struct CpuidLeaves {
  // CPUID.0:EAX, the highest basic leaf.
  uint32_t max_basic_leaf = 0;
  // CPUID.0:EBX, EDX and ECX: the vendor string.
  uint32_t vendor_ebx = 0;
  uint32_t vendor_edx = 0;
  uint32_t vendor_ecx = 0;
  // CPUID.1.
  CpuidResult leaf1;
  // CPUID.(EAX=7,ECX=0); all zero if `max_basic_leaf < 7`.
  CpuidResult leaf7;
  // CPUID.(EAX=0DH,ECX=0); all zero if `max_basic_leaf < 0xD`. EDX:EAX holds
  // the XCR0 bits that the CPU supports.
  CpuidResult leaf_xsave;
  // CPUID.80000000H:EAX, the highest extended leaf.
  uint32_t max_extended_leaf = 0;
  // CPUID.80000001H; all zero if `max_extended_leaf < 0x80000001`.
  CpuidResult leaf_ext1;
  // CPUID.80000008H; all zero if `max_extended_leaf < 0x80000008`.
  CpuidResult leaf_ext8;
};

// Executes CPUID with `leaf` in EAX and `subleaf` in ECX on the calling CPU.
using CpuidFn = CpuidResult (*)(uint32_t leaf, uint32_t subleaf);

// Reads the leaves of `CpuidLeaves` with `cpuid`. Leaves above the highest
// basic leaf, or above the highest extended leaf, stay zero instead of being
// read: CPUs return the values of another leaf for them.
CpuidLeaves CpuReadCpuidLeaves(CpuidFn cpuid);

// How the kernel configures every CPU.
struct CpuConfig {
  // Whether the CPU has FXSAVE/FXRSTOR, SSE and SSE2, which the kernel
  // requires (every x86-64 CPU has them).
  bool fxsr = false;
  // Whether the kernel keeps the register state with XSAVE/XRSTOR (CR4.OSXSAVE
  // set, XCR0 = `xcr0`) instead of FXSAVE/FXRSTOR.
  bool xsave = false;
  // XCR0 if `xsave`, else 0: the x87 and SSE components, AVX if the CPU has
  // it, and the three AVX-512 components if it has all of them and AVX.
  uint64_t xcr0 = 0;
  // Whether FpuRestore() resets the last x87 instruction and operand pointers
  // and opcode before it loads a state (FpuConfig, fpu.h): AMD CPUs without
  // XSaveErPtr save and restore them only while an x87 exception is pending,
  // so a process could otherwise read another process's.
  bool reset_x87_pointers = false;
  // Whether CR4.SMEP is set, so that the kernel faults if it fetches an
  // instruction from a user page: the CPU has SMEP.
  bool smep = false;
  // Whether CR4.SMAP is set, so that the kernel faults if it reads or writes a
  // user page other than through the user-copy helpers (user_access.h): the
  // CPU has SMAP. The helpers lift the protection with STAC and CLAC, which
  // raise #UD on CPUs without SMAP, so the kernel executes them only if this
  // is true (`g_cpu_smap_enabled`), not if only `smep` is.
  bool smap = false;
  // Whether page-table entries may set the no-execute bit: the CPU has NX.
  // boot.S sets IA32_EFER.NXE on every CPU that has it, without which the bit
  // is reserved, and paging.cpp sets it in the page tables that all CPUs share
  // if the BSP's EFER.NXE is set (PagingNoExecuteEnabled(), paging.h), so an
  // AP without NX would take a reserved-bit page fault at the first such
  // entry.
  bool nx = false;
  // Whether CR4.MCE is set, so that a machine check raises #MC, which the
  // kernel reports as a panic (on its own IST stack, gdt.h), instead of
  // shutting the CPU down, and with it the machine: the CPU has the
  // machine-check exception (CPUID.1:EDX.MCE). Intel CPUs signal many machine
  // checks to every CPU, so every CPU needs it.
  bool mce = false;
};

// Chooses the configuration of a CPU whose CPUID reports `leaves`: XSAVE if
// it has XSAVE (CPUID.1:ECX.XSAVE) and leaf 0DH, whose XSAVE supports the x87
// and SSE components, else FXSAVE. A component is enabled only if both XCR0
// support (leaf 0DH) and its CPUID feature flag say that the CPU has it. It
// sets `reset_x87_pointers` on AMD CPUs without XSaveErPtr, like Linux's
// X86_BUG_FXSAVE_LEAK, `smep` and `smap` if leaf 7 reports SMEP and SMAP,
// each on its own and whatever the register-state format, and `nx` and `mce`
// if the CPU has NX and the machine-check exception.
CpuConfig CpuChooseConfig(const CpuidLeaves& leaves);

// Returns true if a CPU whose CPUID reports `leaves` can run with `config`,
// which was chosen for the BSP: it has every feature that `config` enables,
// SMEP, SMAP, NX and the machine-check exception included, and needs no x87
// pointer reset that `config` lacks. (Every CPU must have SMAP if the BSP
// does: a system call switched out in the middle of a user copy may resume on
// another CPU, which then executes the copy's CLAC.)
bool CpuSupportsConfig(const CpuidLeaves& leaves, const CpuConfig& config);

// Returns the CR4 value of a CPU with `config`, every bit of which this
// decides, whatever the firmware or boot loader left: PAE (long mode), OSFXSR
// and OSXMMEXCPT set, MCE, OSXSAVE, SMEP and SMAP set if and only if
// `config.mce`, `config.xsave`, `config.smep` and `config.smap`, and every
// other bit clear. Among those are LA57 (the page tables have 4 levels;
// boot.S clears it before it enables long mode, after which it can't change),
// PCIDE and PGE (CR3 loads flush the whole TLB; no page is global), PKE (XCR0
// never holds PKRU), FSGSBASE (the ISR stubs in boot.S rely on it), CET,
// UMIP, TSD and PCE.
uint64_t CpuCr4ForConfig(const CpuConfig& config);

// The MSRs that CpuMsrValuesFor() loads (Intel SDM Vol. 4).
constexpr uint32_t kMsrSysenterCs = 0x174;
constexpr uint32_t kMsrSysenterEsp = 0x175;
constexpr uint32_t kMsrSysenterEip = 0x176;
constexpr uint32_t kMsrPat = 0x277;
constexpr uint32_t kMsrStar = 0xC0000081;
constexpr uint32_t kMsrLstar = 0xC0000082;
constexpr uint32_t kMsrFmask = 0xC0000084;

// The IA32_PAT value of every CPU: its power-on value, whose entries 0 to 7
// are the memory types WB, WT, UC-, UC, WB, WT, UC- and UC. A page-table
// entry selects entry 4 * PAT + 2 * PCD + PWT; paging.cpp maps RAM with entry
// 0 (WB) and MMIO with PCD and PWT set, entry 3 (UC). On CPUs without PAT,
// PCD and PWT select the same types as entries 0 to 3.
constexpr uint64_t kCpuPat = 0x0007040600070406;

// An MSR and the value that the kernel loads into it.
struct CpuMsrValue {
  uint32_t msr = 0;
  uint64_t value = 0;
};

// The MSR values that CpuMsrValuesFor() returns: `values[0]` to
// `values[count - 1]`, to be loaded in this order.
struct CpuMsrValues {
  int count = 0;
  // Room for every MSR above.
  CpuMsrValue values[7];
};

// Returns the values that CpuInitBsp() and CpuInitAp() load into MSRs of a
// CPU whose CPUID reports `leaves`, whatever the firmware or boot loader left
// in them (INIT, which starts the APs, keeps them too):
// - IA32_SYSENTER_CS, IA32_SYSENTER_ESP and IA32_SYSENTER_EIP: 0, if the CPU
//   has SYSENTER (CPUID.1:EDX.SEP). With a null IA32_SYSENTER_CS, SYSENTER
//   raises #GP instead of entering the kernel at a stale address (AMD CPUs
//   raise #UD for it in long mode anyway).
// - IA32_STAR, IA32_LSTAR and IA32_FMASK: 0, so that no stale SYSCALL entry
//   point is left. User code enters the kernel only through `int 0x80`
//   (idt.h): SYSCALL raises #UD, because boot.S clears IA32_EFER.SCE. (Like
//   Linux, the kernel leaves IA32_CSTAR alone: Intel CPUs don't use it, and
//   some hypervisors fault on writes to it.)
// - IA32_PAT: `kCpuPat`, if the CPU has PAT (CPUID.1:EDX.PAT), so that every
//   CPU uses the same memory types, as the SDM requires.
CpuMsrValues CpuMsrValuesFor(const CpuidLeaves& leaves);

// Writes the boot report of `config` with `write` (for example UartWrite()),
// one line per feature: how the kernel keeps the register state, with the
// size of an FpuState (`fpu_state_size`, fpu.h), such as
// "[CPU] FPU state: XSAVE, XCR0=0x7 (x87 SSE AVX), 832 bytes" or
// "[CPU] FPU state: FXSAVE, 512 bytes", whether FpuRestore() resets the x87
// pointers ("[CPU] x87 pointer reset: enabled"), and whether SMEP and SMAP
// are enabled as `cr4` says, such as "[CPU] SMEP: enabled, SMAP: disabled"
// (CpuInitBsp() passes CR4 as it reads it back from the BSP after applying
// `config`).
void CpuWriteBootReport(const CpuConfig& config, int fpu_state_size,
                        uint64_t cr4, void (*write)(const char* text));

#if !__STDC_HOSTED__
// Chooses the configuration from the BSP's CPUID, loads the MSR values of
// CpuMsrValuesFor() on the BSP, applies the configuration to it (CR4, then
// XCR0), sets the format of the register state (FpuInit(), fpu.h) and writes
// the boot report over the UART. CHECK-fails if the CPU lacks FXSR, SSE or
// SSE2. Must be called once, on the BSP, after IdtInit() and before PmmInit()
// (so before the first uncacheable mapping, which needs `kCpuPat`), SmpInit()
// and the first user process: from then on, no user page exists yet, the
// kernel faults on executing a user page if the CPU has SMEP, and on touching
// one outside the user-copy helpers if it has SMAP. It sets
// `g_cpu_smap_enabled` before it enables SMAP.
void CpuInitBsp();
#endif

// Applies the BSP's configuration (CpuInitBsp()) to the calling AP, after
// loading the AP's MSR values (CpuMsrValuesFor()). CHECK-fails, before it
// changes an MSR or CR4, if the AP lacks a feature that the configuration
// enables (SMEP, SMAP, NX and the machine-check exception included) or needs a
// larger XSAVE area than the BSP: the kernel requires every CPU to have the
// BSP's features. ApKernelEntry() calls it after IdtLoad(), before the AP maps
// its Local APIC or runs tasks. Host builds only count the calls
// (CpuGetHostApInitCountForTest()).
void CpuInitAp();

// Returns the configuration that CpuInitBsp() chose: all false before it, and
// always in host builds.
const CpuConfig& CpuGetConfig();

// Returns `g_cpu_smap_enabled`: whether the kernel runs with SMAP, so that
// only the user-copy helpers reach user pages, with STAC and CLAC.
inline bool CpuSmapEnabled() {
  return g_cpu_smap_enabled.load(std::memory_order_relaxed);
}

#if __STDC_HOSTED__
// Host unit-test hooks: reset the count of CpuInitAp() calls and
// `g_cpu_smap_enabled`, return the count, and set `g_cpu_smap_enabled`.
void CpuResetForTest();
int CpuGetHostApInitCountForTest();
void CpuSetSmapEnabledForTest(bool enabled);
#endif

}  // namespace protos

#endif  // PROTOS_CPU_H_
