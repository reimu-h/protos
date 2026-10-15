#include "gdt.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>

#include "smp.h"

namespace protos {
namespace {

namespace t = ::testing;

// The GDT entries before the first TSS descriptor: null, kernel code, kernel
// data, null, user data, user code.
constexpr int kFirstTssEntry = 6;
constexpr int kGdtEntryCount = kFirstTssEntry + 2 * kMaxSupportedCpus;

static const uint64_t* GdtEntries() {
  return reinterpret_cast<const uint64_t*>(GdtGetPointer().base);
}

// The fields of a TSS, by value: matchers take references, which cannot bind
// to fields of a packed struct.
static uint64_t Ist(const TaskStateSegment* const tss, const int index) {
  return tss->ist[index];
}

static uint64_t Rsp(const TaskStateSegment* const tss, const int index) {
  return tss->rsp[index];
}

static uint16_t IoMapBase(const TaskStateSegment* const tss) {
  return tss->io_map_base;
}

class GdtTest : public t::Test {
 protected:
  void SetUp() override { GdtResetForTest(); }
};

TEST_F(GdtTest, EncodeTssDescriptorSplitsBaseAndLimitAcrossBothEntries) {
  // Lower-half base: limit 15:0 in bits 0-15, base 23:0 in bits 16-39, type
  // 0x9 (available 64-bit TSS) and P in bits 40-47, limit 19:16 in bits 48-51
  // with AVL, L, D/B and G clear, base 31:24 in bits 56-63, and base 63:32 in
  // the low half of the second entry.
  const GdtTssDescriptor tss = GdtEncodeTssDescriptor(
      0x0000123456789ABCULL, sizeof(TaskStateSegment) - 1);
  EXPECT_THAT(tss.low, t::Eq(0x560089789ABC0067ULL));
  EXPECT_THAT(tss.high, t::Eq(0x0000000000001234ULL));

  // Upper-half base and the largest byte-granular limit.
  const GdtTssDescriptor high_tss =
      GdtEncodeTssDescriptor(0xFFFF8000DEADB000ULL, 0xFFFFF);
  EXPECT_THAT(high_tss.low, t::Eq(0xDE0F89ADB000FFFFULL));
  EXPECT_THAT(high_tss.high, t::Eq(0x00000000FFFF8000ULL));
}

TEST_F(GdtTest, EncodeTssDescriptorRejectsNonCanonicalBasesAndLongLimits) {
  EXPECT_DEATH(GdtEncodeTssDescriptor(0x0000800000000000ULL, 103),
               "Check failed: IsCanonicalAddress\\(base\\)");
  EXPECT_DEATH(GdtEncodeTssDescriptor(0x1000, 0x100000),
               "Check failed: limit <= kMaxDescriptorLimit");
}

TEST_F(GdtTest, TssSelectorsFollowTheUserSegmentsSixteenBytesApart) {
  EXPECT_THAT(GdtTssSelector(0), t::Eq(kGdtFirstTssSelector));
  EXPECT_THAT(GdtTssSelector(0), t::Eq(0x30));
  EXPECT_THAT(GdtTssSelector(1), t::Eq(0x40));
  EXPECT_THAT(GdtTssSelector(kMaxSupportedCpus - 1),
              t::Eq(0x30 + 16 * (kMaxSupportedCpus - 1)));
  EXPECT_DEATH(GdtTssSelector(-1), "Check failed: cpu_index >= 0");
  EXPECT_DEATH(GdtTssSelector(kMaxSupportedCpus),
               "Check failed: cpu_index >= 0");
}

TEST_F(GdtTest, GdtHasTheKernelAndUserSegmentsAndATssSlotForEveryCpu) {
  const GdtPointer gdtr = GdtGetPointer();
  EXPECT_THAT(gdtr.limit, t::Eq(kGdtEntryCount * 8 - 1));
  EXPECT_THAT(gdtr.base % 16, t::Eq(0u));
  // The kernel segments are the same as boot.S's gdt64, at the same selectors.
  const uint64_t* const entries = GdtEntries();
  EXPECT_THAT(entries[0], t::Eq(0u));
  EXPECT_THAT(entries[kGdtKernelCodeSelector / 8],
              t::Eq(0x00AF9B000000FFFFULL));
  EXPECT_THAT(entries[kGdtKernelDataSelector / 8],
              t::Eq(0x00CF93000000FFFFULL));
  // The user segments: DPL 3, and where SYSRET expects them, user data 8 bytes
  // and user code 16 bytes after the (null) 32-bit user code entry.
  EXPECT_THAT(kGdtUserDataSelector & 3, t::Eq(3));
  EXPECT_THAT(kGdtUserCodeSelector & 3, t::Eq(3));
  EXPECT_THAT(entries[3], t::Eq(0u));
  EXPECT_THAT(entries[kGdtUserDataSelector / 8], t::Eq(0x00CFF3000000FFFFULL));
  EXPECT_THAT(entries[kGdtUserCodeSelector / 8], t::Eq(0x00AFFB000000FFFFULL));
  EXPECT_THAT(kGdtUserDataSelector / 8, t::Eq(3 + 1));
  EXPECT_THAT(kGdtUserCodeSelector / 8, t::Eq(3 + 2));
  // No TSS descriptor before a CPU loads its TSS.
  for (int i = kFirstTssEntry; i < kGdtEntryCount; ++i) {
    EXPECT_THAT(entries[i], t::Eq(0u)) << "entry " << i;
  }
}

TEST_F(GdtTest, SetKernelStackSetsTheCallingCpusRsp0) {
  CpuLocal cpus[2];
  for (int i = 0; i < 2; ++i) {
    cpus[i].self = &cpus[i];
    cpus[i].cpu_id = i;
  }
  SetInterruptsEnabledForTest(false);
  BindCpuLocal(&cpus[1]);
  GdtSetKernelStack(0x7FF0);
  EXPECT_THAT(Rsp(GdtGetTss(1), 0), t::Eq(0x7FF0u));
  EXPECT_THAT(Rsp(GdtGetTss(0), 0), t::Eq(0u));
  BindCpuLocal(&cpus[0]);
  GdtSetKernelStack(0x9000);
  EXPECT_THAT(Rsp(GdtGetTss(0), 0), t::Eq(0x9000u));
  EXPECT_THAT(Rsp(GdtGetTss(1), 0), t::Eq(0x7FF0u));
  EXPECT_DEATH(GdtSetKernelStack(0), "Check failed: rsp0 != 0");
  EXPECT_DEATH(GdtSetKernelStack(0x9008), "Check failed: rsp0 != 0");
  SetInterruptsEnabledForTest(true);
  EXPECT_DEATH(GdtSetKernelStack(0x9000),
               "Check failed: !AreInterruptsEnabled\\(\\)");
  ResetCpuLocalForTest();
}

TEST_F(GdtTest, LoadForCpuSetsUpItsTssAndDescriptorAndLoadsIt) {
  constexpr int kCpu = 5;
  const uintptr_t ist_stack_tops[kIstStackCount] = {0x10000, 0x20000, 0x30000};
  GdtLoadForCpu(kCpu, ist_stack_tops);

  const TaskStateSegment* const tss = GdtGetTss(kCpu);
  EXPECT_THAT(Ist(tss, kIstDoubleFault - 1), t::Eq(0x10000u));
  EXPECT_THAT(Ist(tss, kIstNmi - 1), t::Eq(0x20000u));
  EXPECT_THAT(Ist(tss, kIstMachineCheck - 1), t::Eq(0x30000u));
  for (int i = kIstStackCount; i < 7; ++i) {
    EXPECT_THAT(Ist(tss, i), t::Eq(0u));
  }
  for (int i = 0; i < 3; ++i) {
    EXPECT_THAT(Rsp(tss, i), t::Eq(0u));
  }
  // The I/O permission bitmap would start beyond the TSS limit: there is none.
  EXPECT_THAT(IoMapBase(tss), t::Eq(sizeof(TaskStateSegment)));

  const GdtTssDescriptor expected = GdtEncodeTssDescriptor(
      reinterpret_cast<uintptr_t>(tss), sizeof(TaskStateSegment) - 1);
  const uint64_t* const entries = GdtEntries();
  const int entry = GdtTssSelector(kCpu) / 8;
  EXPECT_THAT(entries[entry], t::Eq(expected.low));
  EXPECT_THAT(entries[entry + 1], t::Eq(expected.high));
  // Other CPUs' descriptors are untouched.
  EXPECT_THAT(entries[GdtTssSelector(kCpu - 1) / 8], t::Eq(0u));
  EXPECT_THAT(entries[GdtTssSelector(kCpu + 1) / 8], t::Eq(0u));

  EXPECT_THAT(GdtGetHostLoadCountForTest(), t::Eq(1));
  EXPECT_THAT(GdtGetHostTaskRegisterForTest(), t::Eq(GdtTssSelector(kCpu)));
}

TEST_F(GdtTest, EveryCpuGetsItsOwnTssAndCanLoadItAgain) {
  const uintptr_t cpu0_stacks[kIstStackCount] = {0x1000, 0x2000, 0x3000};
  const uintptr_t last_cpu_stacks[kIstStackCount] = {0x7000, 0x8000, 0x9000};
  GdtLoadForCpu(0, cpu0_stacks);
  GdtLoadForCpu(kMaxSupportedCpus - 1, last_cpu_stacks);
  EXPECT_THAT(Ist(GdtGetTss(0), 0), t::Eq(0x1000u));
  EXPECT_THAT(Ist(GdtGetTss(kMaxSupportedCpus - 1), 0), t::Eq(0x7000u));
  EXPECT_THAT(GdtGetHostTaskRegisterForTest(),
              t::Eq(GdtTssSelector(kMaxSupportedCpus - 1)));

  // Loading again rewrites the descriptor as available (`ltr` faults on a busy
  // one) and replaces the IST stacks.
  const uintptr_t new_cpu0_stacks[kIstStackCount] = {0x4000, 0x5000, 0x6000};
  GdtLoadForCpu(0, new_cpu0_stacks);
  const GdtTssDescriptor expected = GdtEncodeTssDescriptor(
      reinterpret_cast<uintptr_t>(GdtGetTss(0)), sizeof(TaskStateSegment) - 1);
  EXPECT_THAT(GdtEntries()[GdtTssSelector(0) / 8], t::Eq(expected.low));
  EXPECT_THAT(Ist(GdtGetTss(0), 2), t::Eq(0x6000u));
  EXPECT_THAT(GdtGetHostLoadCountForTest(), t::Eq(3));
  EXPECT_THAT(GdtGetHostTaskRegisterForTest(), t::Eq(GdtTssSelector(0)));
}

TEST_F(GdtTest, LoadForCpuValidatesItsArguments) {
  const uintptr_t stacks[kIstStackCount] = {0x1000, 0x2000, 0x3000};
  EXPECT_DEATH(GdtLoadForCpu(-1, stacks), "Check failed: cpu_index >= 0");
  EXPECT_DEATH(GdtLoadForCpu(kMaxSupportedCpus, stacks),
               "Check failed: cpu_index >= 0");
  const uintptr_t missing_stack[kIstStackCount] = {0x1000, 0, 0x3000};
  EXPECT_DEATH(GdtLoadForCpu(0, missing_stack),
               "Check failed: ist_stack_tops\\[i\\] != 0");
  const uintptr_t unaligned_stack[kIstStackCount] = {0x1000, 0x2008, 0x3000};
  EXPECT_DEATH(GdtLoadForCpu(0, unaligned_stack),
               "Check failed: ist_stack_tops\\[i\\] != 0");
  EXPECT_DEATH(GdtGetTss(kMaxSupportedCpus), "Check failed: cpu_index >= 0");
}

}  // namespace
}  // namespace protos
