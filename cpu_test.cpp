#include "cpu.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "fpu.h"
#include "smp_hardware.h"

namespace protos {
namespace {

namespace t = ::testing;

// Other CPUID.1:EDX and ECX flags that real CPUs report (x87 FPU, TSC, APIC,
// SSE3, CX16), which the configuration must ignore.
constexpr uint32_t kCpuid1EdxOther = (1u << 0) | (1u << 4) | (1u << 9);
constexpr uint32_t kCpuid1EcxOther = (1u << 0) | (1u << 13);
constexpr uint32_t kCpuid1EdxFxsrSse =
    kCpuid1EdxFxsr | kCpuid1EdxSse | kCpuid1EdxSse2;
// CPUID.(EAX=7,ECX=0):EBX flags that the configuration ignores: FSGSBASE,
// AVX2.
constexpr uint32_t kCpuid7EbxOther = (1u << 0) | (1u << 5);
// MPX (XCR0 bits 3 and 4) and PKRU (bit 9).
constexpr uint32_t kXcr0Mpx = (1u << 3) | (1u << 4);
constexpr uint32_t kXcr0Pkru = 1u << 9;

// QEMU's qemu64 model (the bios, uefi and mb1 boot passes): SSE2, no XSAVE,
// no SMEP or SMAP. Under TCG, QEMU 7.2's models are AuthenticAMD (family 15)
// and report no CPUID.80000008H:EBX flags, so no XSaveErPtr.
static CpuidLeaves Qemu64Leaves() {
  CpuidLeaves leaves;
  leaves.max_basic_leaf = 0xD;
  leaves.vendor_ebx = kCpuidVendorAmdEbx;
  leaves.vendor_edx = kCpuidVendorAmdEdx;
  leaves.vendor_ecx = kCpuidVendorAmdEcx;
  leaves.leaf1.edx = kCpuid1EdxFxsrSse | kCpuid1EdxOther;
  leaves.leaf1.ecx = kCpuid1EcxOther;
  leaves.max_extended_leaf = 0x8000000A;
  return leaves;
}

// QEMU's qemu64,+smep,+xsave model (the smep boot pass): SMEP without SMAP,
// and XSAVE with the x87 and SSE components only, no AVX.
static CpuidLeaves Qemu64XsaveLeaves() {
  CpuidLeaves leaves = Qemu64Leaves();
  leaves.leaf1.ecx |= kCpuid1EcxXsave;
  leaves.leaf7.ebx = kCpuid7EbxSmep;
  leaves.leaf_xsave.eax = static_cast<uint32_t>(kXcr0X87 | kXcr0Sse);
  leaves.leaf_xsave.ebx = 576;
  leaves.leaf_xsave.ecx = 576;
  return leaves;
}

// QEMU's qemu64,+smap model: SMAP without SMEP, and no XSAVE.
static CpuidLeaves Qemu64SmapLeaves() {
  CpuidLeaves leaves = Qemu64Leaves();
  leaves.leaf7.ebx = kCpuid7EbxSmap;
  return leaves;
}

// QEMU's max model (the max boot pass): SMEP and SMAP, XSAVE, AVX, and XCR0
// support for MPX and PKRU as well, which the kernel doesn't enable.
static CpuidLeaves MaxLeaves() {
  CpuidLeaves leaves = Qemu64Leaves();
  leaves.leaf1.ecx |= kCpuid1EcxXsave | kCpuid1EcxAvx;
  leaves.leaf7.ebx = kCpuid7EbxOther | kCpuid7EbxSmep | kCpuid7EbxSmap;
  leaves.leaf_xsave.eax =
      static_cast<uint32_t>(kXcr0X87 | kXcr0Sse | kXcr0Avx) | kXcr0Mpx |
      kXcr0Pkru;
  return leaves;
}

// A CPU with AVX-512.
static CpuidLeaves Avx512Leaves() {
  CpuidLeaves leaves = MaxLeaves();
  leaves.leaf7.ebx |= kCpuid7EbxAvx512f;
  leaves.leaf_xsave.eax |= static_cast<uint32_t>(kXcr0Avx512);
  return leaves;
}

// An AMD CPU with XSaveErPtr (and CLZERO), such as Zen 2.
static CpuidLeaves AmdXsaveErPtrLeaves() {
  CpuidLeaves leaves = MaxLeaves();
  leaves.leaf_ext8.ebx = (1u << 0) | kCpuidExt8EbxXsaveErPtr;
  return leaves;
}

// An Intel CPU ("GenuineIntel").
static CpuidLeaves IntelLeaves() {
  CpuidLeaves leaves = MaxLeaves();
  leaves.vendor_ebx = 0x756E6547;
  leaves.vendor_edx = 0x49656E69;
  leaves.vendor_ecx = 0x6C65746E;
  leaves.max_extended_leaf = 0x80000008;
  return leaves;
}

static CpuConfig MakeConfig(const bool xsave, const uint64_t xcr0) {
  CpuConfig config;
  config.fxsr = true;
  config.xsave = xsave;
  config.xcr0 = xcr0;
  return config;
}

// Returns `config` with SMEP and SMAP enabled as given.
static CpuConfig WithProtections(CpuConfig config,  //
                                 const bool smep,   //
                                 const bool smap) {
  config.smep = smep;
  config.smap = smap;
  return config;
}

TEST(CpuTest, ChoosesFxsaveWithoutXsave) {
  const CpuConfig config = CpuChooseConfig(Qemu64Leaves());
  EXPECT_TRUE(config.fxsr);
  EXPECT_FALSE(config.xsave);
  EXPECT_THAT(config.xcr0, t::Eq(0u));
}

TEST(CpuTest, EnablesOnlyX87AndSseWithoutAvx) {
  const CpuConfig config = CpuChooseConfig(Qemu64XsaveLeaves());
  EXPECT_TRUE(config.fxsr);
  EXPECT_TRUE(config.xsave);
  EXPECT_THAT(config.xcr0, t::Eq(0x3u));
}

TEST(CpuTest, EnablesAvxButNeverMpxOrPkru) {
  const CpuConfig config = CpuChooseConfig(MaxLeaves());
  EXPECT_TRUE(config.xsave);
  EXPECT_THAT(config.xcr0, t::Eq(0x7u));
}

TEST(CpuTest, EnablesAvx512OnlyWithAllThreeComponentsTheFlagAndAvx) {
  EXPECT_THAT(CpuChooseConfig(Avx512Leaves()).xcr0, t::Eq(0xE7u));

  // Hi16_ZMM missing.
  CpuidLeaves partial = Avx512Leaves();
  partial.leaf_xsave.eax &= ~static_cast<uint32_t>(kXcr0Hi16Zmm);
  EXPECT_THAT(CpuChooseConfig(partial).xcr0, t::Eq(0x7u));
  // The components without the AVX512F flag.
  CpuidLeaves no_flag = Avx512Leaves();
  no_flag.leaf7.ebx &= ~kCpuid7EbxAvx512f;
  EXPECT_THAT(CpuChooseConfig(no_flag).xcr0, t::Eq(0x7u));
  // The AVX512F flag without the components.
  CpuidLeaves no_components = MaxLeaves();
  no_components.leaf7.ebx |= kCpuid7EbxAvx512f;
  EXPECT_THAT(CpuChooseConfig(no_components).xcr0, t::Eq(0x7u));
  // AVX-512 without AVX.
  CpuidLeaves no_avx = Avx512Leaves();
  no_avx.leaf1.ecx &= ~kCpuid1EcxAvx;
  EXPECT_THAT(CpuChooseConfig(no_avx).xcr0, t::Eq(0x3u));
}

TEST(CpuTest, EnablesAvxOnlyWithBothXcr0SupportAndTheFlag) {
  CpuidLeaves no_flag = MaxLeaves();
  no_flag.leaf1.ecx &= ~kCpuid1EcxAvx;
  EXPECT_THAT(CpuChooseConfig(no_flag).xcr0, t::Eq(0x3u));

  CpuidLeaves no_support = MaxLeaves();
  no_support.leaf_xsave.eax &= ~static_cast<uint32_t>(kXcr0Avx);
  EXPECT_THAT(CpuChooseConfig(no_support).xcr0, t::Eq(0x3u));

  // The AVX flag without XSAVE.
  CpuidLeaves no_xsave = MaxLeaves();
  no_xsave.leaf1.ecx &= ~kCpuid1EcxXsave;
  const CpuConfig config = CpuChooseConfig(no_xsave);
  EXPECT_FALSE(config.xsave);
  EXPECT_THAT(config.xcr0, t::Eq(0u));
}

TEST(CpuTest, KeepsFxsaveIfXsaveCannotHoldTheX87AndSseState) {
  for (const uint32_t supported : {0x0u, 0x1u, 0x2u, 0x4u, 0x5u}) {
    CpuidLeaves leaves = MaxLeaves();
    leaves.leaf_xsave.eax = supported;
    const CpuConfig config = CpuChooseConfig(leaves);
    EXPECT_FALSE(config.xsave) << "supported XCR0 " << supported;
    EXPECT_THAT(config.xcr0, t::Eq(0u));
  }
}

TEST(CpuTest, KeepsFxsaveWithoutLeafD) {
  CpuidLeaves leaves = MaxLeaves();
  leaves.max_basic_leaf = 0xB;
  EXPECT_FALSE(CpuChooseConfig(leaves).xsave);
}

TEST(CpuTest, ReportsMissingFxsrSseOrSse2) {
  for (const uint32_t missing :
       {kCpuid1EdxFxsr, kCpuid1EdxSse, kCpuid1EdxSse2}) {
    CpuidLeaves leaves = MaxLeaves();
    leaves.leaf1.edx &= ~missing;
    const CpuConfig config = CpuChooseConfig(leaves);
    EXPECT_FALSE(config.fxsr) << "missing EDX bit " << missing;
    EXPECT_FALSE(config.xsave);
  }
}

TEST(CpuTest, ResetsX87PointersOnlyOnAmdCpusWithoutXsaveErPtr) {
  // FXSAVE and XSAVE alike.
  EXPECT_TRUE(CpuChooseConfig(Qemu64Leaves()).reset_x87_pointers);
  EXPECT_TRUE(CpuChooseConfig(Qemu64XsaveLeaves()).reset_x87_pointers);
  EXPECT_TRUE(CpuChooseConfig(MaxLeaves()).reset_x87_pointers);
  EXPECT_FALSE(CpuChooseConfig(AmdXsaveErPtrLeaves()).reset_x87_pointers);
  EXPECT_FALSE(CpuChooseConfig(IntelLeaves()).reset_x87_pointers);
  // All of the vendor string counts.
  CpuidLeaves almost_amd = MaxLeaves();
  almost_amd.vendor_ecx = 0x6C65746E;
  EXPECT_FALSE(CpuChooseConfig(almost_amd).reset_x87_pointers);
  // The format doesn't depend on it.
  EXPECT_THAT(CpuChooseConfig(AmdXsaveErPtrLeaves()).xcr0, t::Eq(0x7u));
}

TEST(CpuTest, ChoosesSmepAndSmapEachOnItsOwn) {
  const CpuConfig neither = CpuChooseConfig(Qemu64Leaves());
  EXPECT_FALSE(neither.smep);
  EXPECT_FALSE(neither.smap);

  const CpuConfig smep_only = CpuChooseConfig(Qemu64XsaveLeaves());
  EXPECT_TRUE(smep_only.smep);
  EXPECT_FALSE(smep_only.smap);

  const CpuConfig smap_only = CpuChooseConfig(Qemu64SmapLeaves());
  EXPECT_FALSE(smap_only.smep);
  EXPECT_TRUE(smap_only.smap);
  EXPECT_FALSE(smap_only.xsave);

  const CpuConfig both = CpuChooseConfig(MaxLeaves());
  EXPECT_TRUE(both.smep);
  EXPECT_TRUE(both.smap);
  EXPECT_THAT(both.xcr0, t::Eq(0x7u));

  // Without XSAVE, which the chooser checks after them.
  CpuidLeaves no_xsave = Qemu64Leaves();
  no_xsave.leaf7.ebx = kCpuid7EbxSmep | kCpuid7EbxSmap;
  const CpuConfig both_without_xsave = CpuChooseConfig(no_xsave);
  EXPECT_TRUE(both_without_xsave.smep);
  EXPECT_TRUE(both_without_xsave.smap);
  EXPECT_FALSE(both_without_xsave.xsave);

  // Other leaf 7 flags don't count.
  CpuidLeaves others = Qemu64Leaves();
  others.leaf7.ebx = kCpuid7EbxOther | kCpuid7EbxAvx512f;
  EXPECT_FALSE(CpuChooseConfig(others).smep);
  EXPECT_FALSE(CpuChooseConfig(others).smap);
}

TEST(CpuTest, IgnoresLeaf7AboveTheHighestBasicLeaf) {
  // Leaf 7's flags without leaf 7.
  CpuidLeaves old_cpu = MaxLeaves();
  old_cpu.max_basic_leaf = 6;
  EXPECT_FALSE(CpuChooseConfig(old_cpu).smep);
  EXPECT_FALSE(CpuChooseConfig(old_cpu).smap);

  // Leaf 7 is enough: SMEP and SMAP don't need leaf 0DH.
  CpuidLeaves leaf7_cpu = MaxLeaves();
  leaf7_cpu.max_basic_leaf = 7;
  const CpuConfig config = CpuChooseConfig(leaf7_cpu);
  EXPECT_TRUE(config.smep);
  EXPECT_TRUE(config.smap);
  EXPECT_FALSE(config.xsave);
}

// The CPUID leaves that FakeCpuid() answers and the leaves it was asked for.
// Like Intel CPUs, it answers a basic leaf above the highest basic leaf with
// the highest basic leaf's values. It answers the extended leaves up to
// 80000008H at least, those above the highest extended leaf with garbage.
std::vector<CpuidResult> g_fake_cpuid_leaves;
std::vector<CpuidResult> g_fake_extended_leaves;
std::vector<uint32_t> g_queried_leaves;

static CpuidResult FakeCpuid(const uint32_t leaf, const uint32_t subleaf) {
  EXPECT_THAT(subleaf, t::Eq(0u));
  g_queried_leaves.push_back(leaf);
  if (leaf >= kCpuidExtendedLeafMax &&
      leaf - kCpuidExtendedLeafMax < g_fake_extended_leaves.size()) {
    return g_fake_extended_leaves[leaf - kCpuidExtendedLeafMax];
  }
  const uint32_t max_leaf = g_fake_cpuid_leaves[0].eax;
  return g_fake_cpuid_leaves[leaf <= max_leaf ? leaf : max_leaf];
}

// Makes FakeCpuid() answer like a CPU with `leaves`, whose leaves that
// `CpuidLeaves` does not hold are distinct garbage.
static void SetFakeCpuid(const CpuidLeaves& leaves) {
  g_fake_cpuid_leaves.assign(leaves.max_basic_leaf + 1, CpuidResult{});
  for (uint32_t leaf = 0; leaf <= leaves.max_basic_leaf; ++leaf) {
    g_fake_cpuid_leaves[leaf].eax = 0xCAFE0000 + leaf;
    g_fake_cpuid_leaves[leaf].ebx = 0xFFFFFFFF;
    g_fake_cpuid_leaves[leaf].ecx = 0xFFFFFFFF;
    g_fake_cpuid_leaves[leaf].edx = 0xFFFFFFFF;
  }
  g_fake_cpuid_leaves[0].eax = leaves.max_basic_leaf;
  g_fake_cpuid_leaves[0].ebx = leaves.vendor_ebx;
  g_fake_cpuid_leaves[0].edx = leaves.vendor_edx;
  g_fake_cpuid_leaves[0].ecx = leaves.vendor_ecx;
  if (leaves.max_basic_leaf >= 1) {
    g_fake_cpuid_leaves[1] = leaves.leaf1;
  }
  if (leaves.max_basic_leaf >= 7) {
    g_fake_cpuid_leaves[7] = leaves.leaf7;
  }
  if (leaves.max_basic_leaf >= 0xD) {
    g_fake_cpuid_leaves[0xD] = leaves.leaf_xsave;
  }
  // Leaf 80000008H also on a CPU without it, so that reading it there
  // returns garbage (in which XSaveErPtr is set).
  constexpr uint32_t kExt8Index = kCpuidExtendedLeaf8 - kCpuidExtendedLeafMax;
  const int extended_count = static_cast<int>(
      (leaves.max_extended_leaf > kCpuidExtendedLeaf8)
          ? leaves.max_extended_leaf - kCpuidExtendedLeafMax + 1
          : kExt8Index + 1);
  g_fake_extended_leaves.assign(extended_count, CpuidResult{});
  for (int i = 0; i < extended_count; ++i) {
    g_fake_extended_leaves[i].eax = 0xBEEF0000 + static_cast<uint32_t>(i);
    g_fake_extended_leaves[i].ebx = 0xFFFFFFFF;
    g_fake_extended_leaves[i].ecx = 0xFFFFFFFF;
    g_fake_extended_leaves[i].edx = 0xFFFFFFFF;
  }
  g_fake_extended_leaves[0].eax = leaves.max_extended_leaf;
  if (leaves.max_extended_leaf >= kCpuidExtendedLeaf8) {
    g_fake_extended_leaves[kExt8Index] = leaves.leaf_ext8;
  }
  g_queried_leaves.clear();
}

TEST(CpuTest, ReaderReadsTheLeavesUpToTheHighestBasicLeaf) {
  const CpuidLeaves amd = AmdXsaveErPtrLeaves();
  SetFakeCpuid(amd);
  const CpuidLeaves read = CpuReadCpuidLeaves(FakeCpuid);
  EXPECT_THAT(g_queried_leaves,
              t::ElementsAre(0u, 1u, 7u, 0xDu, 0x80000000u, 0x80000008u));
  EXPECT_THAT(read.max_basic_leaf, t::Eq(0xDu));
  EXPECT_THAT(read.vendor_ebx, t::Eq(kCpuidVendorAmdEbx));
  EXPECT_THAT(read.vendor_edx, t::Eq(kCpuidVendorAmdEdx));
  EXPECT_THAT(read.vendor_ecx, t::Eq(kCpuidVendorAmdEcx));
  EXPECT_THAT(read.leaf1.ecx, t::Eq(amd.leaf1.ecx));
  EXPECT_THAT(read.leaf1.edx, t::Eq(amd.leaf1.edx));
  EXPECT_THAT(read.leaf7.ebx, t::Eq(amd.leaf7.ebx));
  EXPECT_THAT(read.leaf_xsave.eax, t::Eq(amd.leaf_xsave.eax));
  EXPECT_THAT(read.max_extended_leaf, t::Eq(0x8000000Au));
  EXPECT_THAT(read.leaf_ext8.ebx, t::Eq(amd.leaf_ext8.ebx));
  const CpuConfig config = CpuChooseConfig(read);
  EXPECT_THAT(config.xcr0, t::Eq(0x7u));
  EXPECT_FALSE(config.reset_x87_pointers);
  EXPECT_TRUE(config.smep);
  EXPECT_TRUE(config.smap);
}

TEST(CpuTest, ReaderSkipsTheLeavesAboveTheHighestBasicLeaf) {
  CpuidLeaves old_cpu = MaxLeaves();
  old_cpu.max_basic_leaf = 6;
  SetFakeCpuid(old_cpu);
  const CpuidLeaves read = CpuReadCpuidLeaves(FakeCpuid);
  // Leaves 7 and 0DH would have returned leaf 6's garbage.
  EXPECT_THAT(g_queried_leaves,
              t::ElementsAre(0u, 1u, 0x80000000u, 0x80000008u));
  EXPECT_THAT(read.leaf7.ebx, t::Eq(0u));
  EXPECT_THAT(read.leaf_xsave.eax, t::Eq(0u));
  EXPECT_THAT(read.leaf_xsave.edx, t::Eq(0u));
  const CpuConfig config = CpuChooseConfig(read);
  EXPECT_FALSE(config.xsave);
  // That garbage has the SMEP and SMAP bits set.
  EXPECT_FALSE(config.smep);
  EXPECT_FALSE(config.smap);
}

TEST(CpuTest, ReaderReadsLeaf7WithoutLeafD) {
  CpuidLeaves leaf7_cpu = Qemu64XsaveLeaves();
  leaf7_cpu.max_basic_leaf = 7;
  SetFakeCpuid(leaf7_cpu);
  const CpuidLeaves read = CpuReadCpuidLeaves(FakeCpuid);
  EXPECT_THAT(g_queried_leaves, t::Contains(7u));
  EXPECT_THAT(g_queried_leaves, t::Not(t::Contains(0xDu)));
  EXPECT_THAT(read.leaf7.ebx, t::Eq(kCpuid7EbxSmep));
  const CpuConfig config = CpuChooseConfig(read);
  EXPECT_TRUE(config.smep);
  EXPECT_FALSE(config.smap);
  EXPECT_FALSE(config.xsave);
}

TEST(CpuTest, ReaderSkipsLeaf80000008HAboveTheHighestExtendedLeaf) {
  CpuidLeaves old_cpu = AmdXsaveErPtrLeaves();
  old_cpu.max_extended_leaf = 0x80000007;
  SetFakeCpuid(old_cpu);
  const CpuidLeaves read = CpuReadCpuidLeaves(FakeCpuid);
  // Leaf 80000008H would have returned garbage with XSaveErPtr set.
  EXPECT_THAT(g_queried_leaves, t::ElementsAre(0u, 1u, 7u, 0xDu, 0x80000000u));
  EXPECT_THAT(read.max_extended_leaf, t::Eq(0x80000007u));
  EXPECT_THAT(read.leaf_ext8.ebx, t::Eq(0u));
  EXPECT_TRUE(CpuChooseConfig(read).reset_x87_pointers);
}

TEST(CpuTest, Cr4KeepsOtherBitsSetsOsxsaveForXsaveAndClearsFsgsbase) {
  // What boot.S sets: PAE, OSFXSR and OSXMMEXCPT.
  constexpr uint64_t kPae = uint64_t{1} << 5;
  constexpr uint64_t kPge = uint64_t{1} << 7;
  constexpr uint64_t kBoot = kPae | kCr4Osfxsr | kCr4Osxmmexcpt;
  const CpuConfig xsave = MakeConfig(true, 0x7);
  const CpuConfig fxsave = MakeConfig(false, 0);

  EXPECT_THAT(CpuCr4ForConfig(kBoot, xsave), t::Eq(kBoot | kCr4Osxsave));
  EXPECT_THAT(CpuCr4ForConfig(kBoot, fxsave), t::Eq(kBoot));
  EXPECT_THAT(CpuCr4ForConfig(kBoot | kCr4Fsgsbase, xsave),
              t::Eq(kBoot | kCr4Osxsave));
  EXPECT_THAT(CpuCr4ForConfig(kBoot | kCr4Osxsave, fxsave), t::Eq(kBoot));
  // Missing bits are set, and unrelated bits are kept.
  EXPECT_THAT(CpuCr4ForConfig(kPae, fxsave), t::Eq(kBoot));
  EXPECT_THAT(CpuCr4ForConfig(kBoot | kPge, xsave),
              t::Eq(kBoot | kPge | kCr4Osxsave));
}

TEST(CpuTest, Cr4SetsSmepAndSmapAsConfigured) {
  constexpr uint64_t kPae = uint64_t{1} << 5;
  constexpr uint64_t kBoot = kPae | kCr4Osfxsr | kCr4Osxmmexcpt;
  const CpuConfig fxsave = MakeConfig(false, 0);
  const CpuConfig smep_only = WithProtections(fxsave, true, false);
  const CpuConfig smap_only = WithProtections(fxsave, false, true);
  const CpuConfig both = WithProtections(MakeConfig(true, 0x7), true, true);

  EXPECT_THAT(CpuCr4ForConfig(kBoot, smep_only), t::Eq(kBoot | kCr4Smep));
  EXPECT_THAT(CpuCr4ForConfig(kBoot, smap_only), t::Eq(kBoot | kCr4Smap));
  EXPECT_THAT(CpuCr4ForConfig(kBoot, both),
              t::Eq(kBoot | kCr4Osxsave | kCr4Smep | kCr4Smap));
  // Bits that the configuration doesn't enable are cleared.
  EXPECT_THAT(CpuCr4ForConfig(kBoot | kCr4Smep | kCr4Smap, fxsave),
              t::Eq(kBoot));
  EXPECT_THAT(CpuCr4ForConfig(kBoot | kCr4Smap, smep_only),
              t::Eq(kBoot | kCr4Smep));
  EXPECT_THAT(CpuCr4ForConfig(kBoot | kCr4Smep, smap_only),
              t::Eq(kBoot | kCr4Smap));
}

TEST(CpuTest, SupportsConfigOnlyIfTheCpuHasEveryEnabledFeature) {
  const CpuConfig fxsave = CpuChooseConfig(Qemu64Leaves());
  const CpuConfig xsave_sse = CpuChooseConfig(Qemu64XsaveLeaves());
  const CpuConfig xsave_avx = CpuChooseConfig(MaxLeaves());

  EXPECT_TRUE(CpuSupportsConfig(Qemu64Leaves(), fxsave));
  EXPECT_TRUE(CpuSupportsConfig(MaxLeaves(), xsave_avx));
  // A CPU with more features can run with fewer.
  EXPECT_TRUE(CpuSupportsConfig(MaxLeaves(), fxsave));
  EXPECT_TRUE(CpuSupportsConfig(MaxLeaves(), xsave_sse));
  EXPECT_TRUE(CpuSupportsConfig(Avx512Leaves(), xsave_avx));
  // A CPU with fewer features can't run with more.
  EXPECT_FALSE(CpuSupportsConfig(Qemu64Leaves(), xsave_sse));
  EXPECT_FALSE(CpuSupportsConfig(Qemu64XsaveLeaves(), xsave_avx));
  EXPECT_FALSE(CpuSupportsConfig(MaxLeaves(), MakeConfig(true, 0xE7)));
  CpuidLeaves no_sse2 = Qemu64Leaves();
  no_sse2.leaf1.edx &= ~kCpuid1EdxSse2;
  EXPECT_FALSE(CpuSupportsConfig(no_sse2, fxsave));
  // A CPU that needs the x87 pointer reset can't run without it, and one that
  // doesn't need it runs with it too.
  EXPECT_FALSE(
      CpuSupportsConfig(MaxLeaves(), CpuChooseConfig(AmdXsaveErPtrLeaves())));
  EXPECT_TRUE(CpuSupportsConfig(AmdXsaveErPtrLeaves(), xsave_avx));
}

TEST(CpuTest, SupportsConfigOnlyIfTheCpuHasTheEnabledSmepAndSmap) {
  // The configurations differ from what the CPUs choose only in SMEP and SMAP.
  const CpuConfig max = CpuChooseConfig(MaxLeaves());
  const CpuConfig qemu64 = CpuChooseConfig(Qemu64Leaves());
  EXPECT_TRUE(CpuSupportsConfig(MaxLeaves(), max));
  EXPECT_TRUE(
      CpuSupportsConfig(MaxLeaves(), WithProtections(max, true, false)));
  EXPECT_TRUE(
      CpuSupportsConfig(MaxLeaves(), WithProtections(max, false, true)));
  EXPECT_TRUE(CpuSupportsConfig(Qemu64SmapLeaves(), qemu64));
  EXPECT_TRUE(CpuSupportsConfig(Qemu64SmapLeaves(),
                                CpuChooseConfig(Qemu64SmapLeaves())));

  EXPECT_FALSE(
      CpuSupportsConfig(Qemu64Leaves(), WithProtections(qemu64, false, true)));
  EXPECT_FALSE(
      CpuSupportsConfig(Qemu64Leaves(), WithProtections(qemu64, true, false)));
  EXPECT_FALSE(CpuSupportsConfig(Qemu64SmapLeaves(),
                                 WithProtections(qemu64, true, false)));
  CpuidLeaves no_smap = MaxLeaves();
  no_smap.leaf7.ebx &= ~kCpuid7EbxSmap;
  EXPECT_FALSE(CpuSupportsConfig(no_smap, max));
  CpuidLeaves no_smep = MaxLeaves();
  no_smep.leaf7.ebx &= ~kCpuid7EbxSmep;
  EXPECT_FALSE(CpuSupportsConfig(no_smep, max));
  // Leaf 7 above the highest basic leaf doesn't count.
  CpuidLeaves old_cpu = Qemu64Leaves();
  old_cpu.max_basic_leaf = 6;
  old_cpu.leaf7.ebx = kCpuid7EbxSmep | kCpuid7EbxSmap;
  EXPECT_TRUE(CpuSupportsConfig(old_cpu, qemu64));
  EXPECT_FALSE(
      CpuSupportsConfig(old_cpu, WithProtections(qemu64, true, false)));
}

std::string g_report;

static void AppendToReport(const char* const text) { g_report += text; }

// Returns the boot report of `config` with an FpuState of `fpu_state_size`
// bytes, and CR4 as `config` sets it.
static std::string BootReport(const CpuConfig& config,
                              const int fpu_state_size) {
  g_report.clear();
  const uint64_t cr4 = CpuCr4ForConfig(0, config);
  CpuWriteBootReport(config, fpu_state_size, cr4, AppendToReport);
  return g_report;
}

// Returns the boot report of `config` with an FpuState of 512 bytes and CR4
// `cr4`.
static std::string BootReportWithCr4(const CpuConfig& config,
                                     const uint64_t cr4) {
  g_report.clear();
  CpuWriteBootReport(config, 512, cr4, AppendToReport);
  return g_report;
}

TEST(CpuTest, BootReportNamesTheFormatTheComponentsAndTheSize) {
  EXPECT_THAT(BootReport(CpuChooseConfig(Qemu64Leaves()), 512),
              t::StartsWith("[CPU] FPU state: FXSAVE, 512 bytes\n"));
  EXPECT_THAT(BootReport(CpuChooseConfig(Qemu64XsaveLeaves()), 576),
              t::StartsWith("[CPU] FPU state: XSAVE, XCR0=0x3 (x87 SSE), 576 "
                            "bytes\n"));
  EXPECT_THAT(
      BootReport(CpuChooseConfig(MaxLeaves()), 832),
      t::StartsWith("[CPU] FPU state: XSAVE, XCR0=0x7 (x87 SSE AVX), 832 "
                    "bytes\n"));
  EXPECT_THAT(BootReport(CpuChooseConfig(Avx512Leaves()), 2688),
              t::StartsWith("[CPU] FPU state: XSAVE, XCR0=0xE7 (x87 SSE AVX "
                            "AVX-512), 2688 bytes\n"));
  // The size comes from the caller (CPUID), not from the components.
  EXPECT_THAT(
      BootReport(CpuChooseConfig(MaxLeaves()), 1088),
      t::StartsWith("[CPU] FPU state: XSAVE, XCR0=0x7 (x87 SSE AVX), 1088 "
                    "bytes\n"));
  // test_boot.sh fails a boot whose serial output contains "FAIL".
  EXPECT_THAT(BootReport(CpuChooseConfig(Avx512Leaves()), 2688),
              t::Not(t::HasSubstr("FAIL")));
}

TEST(CpuTest, BootReportNamesSmepAndSmapAfterTheFpuLines) {
  // The models of the boot passes (test_boot.sh checks these lines).
  EXPECT_THAT(BootReport(CpuChooseConfig(Qemu64Leaves()), 512),
              t::StrEq("[CPU] FPU state: FXSAVE, 512 bytes\n"
                       "[CPU] x87 pointer reset: enabled\n"
                       "[CPU] SMEP: disabled, SMAP: disabled\n"));
  EXPECT_THAT(BootReport(CpuChooseConfig(Qemu64XsaveLeaves()), 576),
              t::StrEq("[CPU] FPU state: XSAVE, XCR0=0x3 (x87 SSE), 576 bytes\n"
                       "[CPU] x87 pointer reset: enabled\n"
                       "[CPU] SMEP: enabled, SMAP: disabled\n"));
  EXPECT_THAT(
      BootReport(CpuChooseConfig(MaxLeaves()), 832),
      t::StrEq("[CPU] FPU state: XSAVE, XCR0=0x7 (x87 SSE AVX), 832 bytes\n"
               "[CPU] x87 pointer reset: enabled\n"
               "[CPU] SMEP: enabled, SMAP: enabled\n"));
  EXPECT_THAT(BootReport(CpuChooseConfig(Qemu64SmapLeaves()), 512),
              t::EndsWith("\n[CPU] SMEP: disabled, SMAP: enabled\n"));
  for (const bool smep : {false, true}) {
    for (const bool smap : {false, true}) {
      EXPECT_THAT(
          BootReport(WithProtections(MakeConfig(false, 0), smep, smap), 512),
          t::Not(t::HasSubstr("FAIL")));
    }
  }
}

TEST(CpuTest, BootReportNamesSmepAndSmapAsCr4HasThem) {
  // CR4 as read back decides, not the configuration: a CPU on which the write
  // of CR4 left SMEP or SMAP clear reports it as disabled.
  const CpuConfig max = CpuChooseConfig(MaxLeaves());
  EXPECT_THAT(BootReportWithCr4(max, 0),
              t::EndsWith("\n[CPU] SMEP: disabled, SMAP: disabled\n"));
  EXPECT_THAT(BootReportWithCr4(max, kCr4Smep),
              t::EndsWith("\n[CPU] SMEP: enabled, SMAP: disabled\n"));
  EXPECT_THAT(BootReportWithCr4(max, kCr4Smap),
              t::EndsWith("\n[CPU] SMEP: disabled, SMAP: enabled\n"));
  const CpuConfig qemu64 = CpuChooseConfig(Qemu64Leaves());
  EXPECT_THAT(BootReportWithCr4(qemu64, kCr4Smep | kCr4Smap),
              t::EndsWith("\n[CPU] SMEP: enabled, SMAP: enabled\n"));
  // The other bits of CR4 don't matter.
  EXPECT_THAT(BootReportWithCr4(qemu64, ~(kCr4Smep | kCr4Smap)),
              t::EndsWith("\n[CPU] SMEP: disabled, SMAP: disabled\n"));
}

TEST(CpuTest, BootReportSaysWhetherFpuRestoreResetsTheX87Pointers) {
  EXPECT_THAT(BootReport(CpuChooseConfig(MaxLeaves()), 832),
              t::HasSubstr("\n[CPU] x87 pointer reset: enabled\n"));
  EXPECT_THAT(BootReport(CpuChooseConfig(AmdXsaveErPtrLeaves()), 832),
              t::HasSubstr("\n[CPU] x87 pointer reset: disabled\n"));
  EXPECT_THAT(BootReport(CpuChooseConfig(IntelLeaves()), 832),
              t::HasSubstr("\n[CPU] x87 pointer reset: disabled\n"));
}

TEST(CpuTest, HostApInitOnlyCountsTheCalls) {
  CpuResetForTest();
  EXPECT_THAT(CpuGetHostApInitCountForTest(), t::Eq(0));
  CpuInitAp();
  CpuInitAp();
  EXPECT_THAT(CpuGetHostApInitCountForTest(), t::Eq(2));
  // Host builds never choose a configuration.
  EXPECT_FALSE(CpuGetConfig().fxsr);
  EXPECT_FALSE(CpuGetConfig().xsave);
  EXPECT_FALSE(CpuGetConfig().smep);
  EXPECT_FALSE(CpuGetConfig().smap);
  CpuResetForTest();
  EXPECT_THAT(CpuGetHostApInitCountForTest(), t::Eq(0));
}

TEST(CpuTest, SmapFlagIsClearInHostBuildsUnlessATestSetsIt) {
  CpuResetForTest();
  EXPECT_FALSE(CpuSmapEnabled());
  CpuSetSmapEnabledForTest(true);
  EXPECT_TRUE(CpuSmapEnabled());
  EXPECT_TRUE(g_cpu_smap_enabled.load());
  CpuResetForTest();
  EXPECT_FALSE(CpuSmapEnabled());
}

}  // namespace
}  // namespace protos
