#include "smp.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "idt.h"
#include "paging.h"
#include "pmm.h"
#include "spinlock.h"
#include "uart.h"
#include "vga.h"

namespace protos {
namespace {

namespace t = ::testing;

constexpr uint32_t kMultiboot1Magic = 0x2BADB002;
constexpr uint32_t kMultiboot2Magic = 0x36D76289;
// Large enough for the AP stacks of kMaxSupportedCpus CPUs.
constexpr int64_t kFakeRamFrames = 1024;
constexpr int64_t kFakeRamBytes = kFakeRamFrames * kPageSize;

alignas(kPageSize) uint8_t g_fake_ram[kFakeRamBytes];

struct [[gnu::packed]] RawRsdpV1 {
  char signature[8];
  uint8_t checksum;
  char oem_id[6];
  uint8_t revision;
  uint32_t rsdt_address;
};

struct [[gnu::packed]] RawRsdpV2 {
  RawRsdpV1 v1;
  uint32_t length;
  uint64_t xsdt_address;
  uint8_t extended_checksum;
  uint8_t reserved[3];
};

struct [[gnu::packed]] RawSdtHeader {
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

struct [[gnu::packed]] RawMadtHeader {
  RawSdtHeader sdt;
  uint32_t local_apic_address;
  uint32_t flags;
};

struct [[gnu::packed]] RawMadtLapic {
  uint8_t type;
  uint8_t length;
  uint8_t acpi_processor_id;
  uint8_t apic_id;
  uint32_t flags;
};

struct [[gnu::packed]] RawMadtLapicOverride {
  uint8_t type;
  uint8_t length;
  uint16_t reserved;
  uint64_t local_apic_address;
};

struct [[gnu::packed]] RawMadtX2Apic {
  uint8_t type;
  uint8_t length;
  uint16_t reserved;
  uint32_t x2apic_id;
  uint32_t flags;
  uint32_t acpi_processor_uid;
};

struct FakeHostState {
  bool map_ok = true;
  uintptr_t fail_map_addr = 0;
  int64_t next_pmm_frame = 32;
  int64_t max_pmm_frame = kFakeRamFrames;
  int pmm_alloc_calls = 0;
  int fail_pmm_on_call = -1;
  std::vector<std::pair<uintptr_t, int64_t>> freed_frames;
  std::string uart_log;
  std::string vga_log;
};

FakeHostState g_state;

// CPUID and MSR state behind the hooks installed by InstallFakeCpuHooks().
struct FakeCpu {
  bool x2apic_supported = false;
  uint32_t max_leaf = 0x0B;
  uint32_t leaf15_ebx = 0;
  uint32_t leaf15_ecx = 0;
  uint32_t leaf16_ecx = 0;
  uint64_t apic_base_msr = 0xFEE00900;
  uint32_t x2apic_id_msr = 0;
  std::vector<std::pair<uint32_t, uint64_t>> msr_writes;
};

FakeCpu g_cpu;

static void FakeCpuid(const uint32_t leaf,     //
                      const uint32_t subleaf,  //
                      uint32_t* const eax,     //
                      uint32_t* const ebx,     //
                      uint32_t* const ecx,     //
                      uint32_t* const /*edx*/) {
  if (subleaf != 0) {
    return;
  }
  if (leaf == 0) {
    *eax = g_cpu.max_leaf;
  } else if (leaf == 1) {
    *ecx = g_cpu.x2apic_supported ? (1u << 21) : 0;
  } else if (leaf == 0x15 && g_cpu.max_leaf >= 0x15) {
    *eax = 1;
    *ebx = g_cpu.leaf15_ebx;
    *ecx = g_cpu.leaf15_ecx;
  } else if (leaf == 0x16 && g_cpu.max_leaf >= 0x16) {
    *ecx = g_cpu.leaf16_ecx;
  }
}

static uint64_t FakeReadMsr(const uint32_t msr) {
  if (msr == 0x1B) {
    return g_cpu.apic_base_msr;
  }
  if (msr == 0x802) {
    return g_cpu.x2apic_id_msr;
  }
  return 0;
}

static void FakeWriteMsr(const uint32_t msr, const uint64_t value) {
  g_cpu.msr_writes.emplace_back(msr, value);
  if (msr == 0x1B) {
    g_cpu.apic_base_msr = value;
  }
}

// Installs the g_cpu hooks; call after SmpSetHostTestHooks(), which clears
// them.
static void InstallFakeCpuHooks() {
  SmpSetHostHardwareHooksForTest(g_cpu.x2apic_supported,  //
                                 FakeCpuid,               //
                                 FakeReadMsr,             //
                                 FakeWriteMsr);
}

static uintptr_t BaseAddr() { return reinterpret_cast<uintptr_t>(g_fake_ram); }

static uintptr_t OffsetAddr(const int64_t offset) {
  return BaseAddr() + offset;
}

static void ResetState(const uintptr_t acpi32_arena = 0) {
  g_state = FakeHostState{};
  g_cpu = FakeCpu{};
  std::memset(g_fake_ram, 0, sizeof(g_fake_ram));
  IdtResetForTest();
  SmpSetHostTestHooks(0, 0, 0, acpi32_arena, nullptr);
}

static uint8_t ComputeChecksumByte(const void* const data,
                                   const int64_t length) {
  const uint8_t* const bytes = static_cast<const uint8_t*>(data);
  uint8_t sum = 0;
  for (int64_t i = 0; i < length; ++i) {
    sum = static_cast<uint8_t>(sum + bytes[i]);
  }
  return static_cast<uint8_t>(0u - sum);
}

static void WriteRsdpV1(const uintptr_t addr, const uint32_t rsdt_addr) {
  RawRsdpV1* const rsdp = reinterpret_cast<RawRsdpV1*>(addr);
  std::memset(rsdp, 0, sizeof(RawRsdpV1));
  std::memcpy(rsdp->signature, "RSD PTR ", 8);
  std::memcpy(rsdp->oem_id, "PROTOS", 6);
  rsdp->revision = 0;
  rsdp->rsdt_address = rsdt_addr;
  rsdp->checksum = ComputeChecksumByte(rsdp, sizeof(RawRsdpV1));
}

static void WriteRsdpV2(const uintptr_t addr,      //
                        const uint32_t rsdt_addr,  //
                        const uint64_t xsdt_addr) {
  RawRsdpV2* const rsdp = reinterpret_cast<RawRsdpV2*>(addr);
  std::memset(rsdp, 0, sizeof(RawRsdpV2));
  std::memcpy(rsdp->v1.signature, "RSD PTR ", 8);
  std::memcpy(rsdp->v1.oem_id, "PROTOS", 6);
  rsdp->v1.revision = 2;
  rsdp->v1.rsdt_address = rsdt_addr;
  rsdp->v1.checksum = ComputeChecksumByte(&rsdp->v1, sizeof(RawRsdpV1));
  rsdp->length = sizeof(RawRsdpV2);
  rsdp->xsdt_address = xsdt_addr;
  rsdp->extended_checksum = ComputeChecksumByte(rsdp, sizeof(RawRsdpV2));
}

static void FinalizeSdt(RawSdtHeader* const sdt,  //
                        const char* const sig,    //
                        const int64_t total_length) {
  std::memcpy(sdt->signature, sig, 4);
  sdt->length = static_cast<uint32_t>(total_length);
  sdt->revision = 1;
  std::memcpy(sdt->oem_id, "PROTOS", 6);
  std::memcpy(sdt->oem_table_id, "SMPTABLE", 8);
  sdt->checksum = 0;
  sdt->checksum = ComputeChecksumByte(sdt, total_length);
}

static void WriteRsdt(const uintptr_t addr,
                      const std::vector<uint32_t>& entries) {
  RawSdtHeader* const sdt = reinterpret_cast<RawSdtHeader*>(addr);
  const int64_t total_len =
      sizeof(RawSdtHeader) + entries.size() * sizeof(uint32_t);
  std::memset(sdt, 0, total_len);
  uint8_t* const payload =
      reinterpret_cast<uint8_t*>(addr) + sizeof(RawSdtHeader);
  for (int i = 0; i < static_cast<int>(entries.size()); ++i) {
    const uint32_t val = entries[i];
    std::memcpy(payload + i * sizeof(uint32_t), &val, sizeof(uint32_t));
  }
  FinalizeSdt(sdt, "RSDT", total_len);
}

static void WriteXsdt(const uintptr_t addr,
                      const std::vector<uint64_t>& entries) {
  RawSdtHeader* const sdt = reinterpret_cast<RawSdtHeader*>(addr);
  const int64_t total_len =
      sizeof(RawSdtHeader) + entries.size() * sizeof(uint64_t);
  std::memset(sdt, 0, total_len);
  uint8_t* const payload =
      reinterpret_cast<uint8_t*>(addr) + sizeof(RawSdtHeader);
  for (int i = 0; i < static_cast<int>(entries.size()); ++i) {
    const uint64_t val = entries[i];
    std::memcpy(payload + i * sizeof(uint64_t), &val, sizeof(uint64_t));
  }
  FinalizeSdt(sdt, "XSDT", total_len);
}

static void WriteMadt(const uintptr_t addr,            //
                      const uint32_t local_apic_addr,  //
                      const std::vector<uint8_t>& records) {
  RawMadtHeader* const madt = reinterpret_cast<RawMadtHeader*>(addr);
  const int64_t total_len = sizeof(RawMadtHeader) + records.size();
  std::memset(madt, 0, total_len);
  madt->local_apic_address = local_apic_addr;
  madt->flags = 1;
  if (!records.empty()) {
    std::memcpy(reinterpret_cast<uint8_t*>(addr) + sizeof(RawMadtHeader),  //
                records.data(),                                            //
                records.size());
  }
  FinalizeSdt(&madt->sdt, "APIC", total_len);
}

static void AppendLapicRecord(std::vector<uint8_t>* const out,  //
                              const uint8_t acpi_id,            //
                              const uint8_t apic_id,            //
                              const uint32_t flags) {
  RawMadtLapic rec = {};
  rec.type = 0;
  rec.length = sizeof(RawMadtLapic);
  rec.acpi_processor_id = acpi_id;
  rec.apic_id = apic_id;
  rec.flags = flags;
  const uint8_t* const bytes = reinterpret_cast<const uint8_t*>(&rec);
  out->insert(out->end(), bytes, bytes + sizeof(rec));
}

static void AppendLapicOverrideRecord(std::vector<uint8_t>* const out,
                                      const uint64_t lapic_phys_addr) {
  RawMadtLapicOverride rec = {};
  rec.type = 5;
  rec.length = sizeof(RawMadtLapicOverride);
  rec.reserved = 0;
  rec.local_apic_address = lapic_phys_addr;
  const uint8_t* const bytes = reinterpret_cast<const uint8_t*>(&rec);
  out->insert(out->end(), bytes, bytes + sizeof(rec));
}

static void AppendX2ApicRecord(std::vector<uint8_t>* const out,  //
                               const uint32_t x2apic_id,         //
                               const uint32_t uid,               //
                               const uint32_t flags) {
  RawMadtX2Apic rec = {};
  rec.type = 9;
  rec.length = sizeof(RawMadtX2Apic);
  rec.reserved = 0;
  rec.x2apic_id = x2apic_id;
  rec.flags = flags;
  rec.acpi_processor_uid = uid;
  const uint8_t* const bytes = reinterpret_cast<const uint8_t*>(&rec);
  out->insert(out->end(), bytes, bytes + sizeof(rec));
}

// Writes an MADT with `records`, an RSDT that points to it, and an RSDP v1 in
// the simulated BIOS ROM area, and returns the BIOS ROM base address for
// SmpSetHostTestHooks(). Requires ResetState(BaseAddr()).
static uintptr_t WriteBiosAcpiTables(const std::vector<uint8_t>& records) {
  const uintptr_t rsdt_addr = OffsetAddr(0x6100);
  const uintptr_t madt_addr = OffsetAddr(0x6400);
  const uintptr_t bios_rom_base = OffsetAddr(0x7000);
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, records);
  WriteRsdt(rsdt_addr, {static_cast<uint32_t>(madt_addr)});
  WriteRsdpV1(bios_rom_base + 64, static_cast<uint32_t>(rsdt_addr));
  return bios_rom_base;
}

// Returns MADT records for CPUs with Local APIC IDs 0 .. count - 1.
static std::vector<uint8_t> LapicRecords(const int count) {
  std::vector<uint8_t> records;
  for (int i = 0; i < count; ++i) {
    AppendLapicRecord(&records,                 //
                      static_cast<uint8_t>(i),  //
                      static_cast<uint8_t>(i),  //
                      1);
  }
  return records;
}

// Simulated AP bring-up that succeeds, as SmpInit() does by default.
static bool StartAp(CpuInfo* const cpu_info) {
  cpu_info->observed_apic_id = cpu_info->apic_id;
  cpu_info->observed_rsp = cpu_info->stack_top - 64;
  cpu_info->observed_cr3 = 0x105000;
  cpu_info->long_mode_active = true;
  cpu_info->online = true;
  return true;
}

}  // namespace

bool PagingMapBootstrapRange(const uintptr_t phys_addr, const int64_t size) {
  (void)size;
  if (!g_state.map_ok) {
    return false;
  }
  if (g_state.fail_map_addr != 0 && phys_addr == g_state.fail_map_addr) {
    return false;
  }
  return true;
}

uintptr_t PmmAllocFrames(const int64_t count) {
  ++g_state.pmm_alloc_calls;
  if (g_state.fail_pmm_on_call == g_state.pmm_alloc_calls) {
    return 0;
  }
  if (count <= 0 || g_state.next_pmm_frame + count > g_state.max_pmm_frame) {
    return 0;
  }
  const uintptr_t addr = BaseAddr() + g_state.next_pmm_frame * kPageSize;
  g_state.next_pmm_frame += count;
  return addr;
}

void PmmFreeFrames(const uintptr_t base_addr, const int64_t count) {
  g_state.freed_frames.emplace_back(base_addr, count);
}

void UartWrite(const char* const str) { g_state.uart_log.append(str); }

void UartWriteHex(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "0x%llx",     //
                static_cast<unsigned long long>(value));
  g_state.uart_log.append(buf);
}

void UartWriteDec(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "%llu",       //
                static_cast<unsigned long long>(value));
  g_state.uart_log.append(buf);
}

void VgaWrite(const char* const str) { g_state.vga_log.append(str); }

void VgaWriteHex(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "0x%llx",     //
                static_cast<unsigned long long>(value));
  g_state.vga_log.append(buf);
}

void VgaWriteDec(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "%llu",       //
                static_cast<unsigned long long>(value));
  g_state.vga_log.append(buf);
}

void UartPanicWrite(const char* const str) { UartWrite(str); }

void UartPanicWriteDec(const uint64_t value) { UartWriteDec(value); }

void VgaPanicWrite(const char* const str) { VgaWrite(str); }

void VgaPanicWriteDec(const uint64_t value) { VgaWriteDec(value); }

namespace {

TEST(SmpTest, ParseRsdpValidatesV1AndV2AndRejectsMalformed) {
  ResetState();
  const uintptr_t v1_addr = OffsetAddr(0x100);
  const uintptr_t v2_addr = OffsetAddr(0x200);
  WriteRsdpV1(v1_addr, 0x12345000);
  WriteRsdpV2(v2_addr, 0x12345000, 0x9876543200ULL);

  uintptr_t rsdt = 0;
  uintptr_t xsdt = 0;
  EXPECT_TRUE(
      SmpParseRsdpForTest(v1_addr, kMaxCanonicalIdentityAddress, &rsdt, &xsdt));
  EXPECT_THAT(rsdt, t::Eq(0x12345000u));
  EXPECT_THAT(xsdt, t::Eq(0u));

  EXPECT_TRUE(
      SmpParseRsdpForTest(v2_addr, kMaxCanonicalIdentityAddress, &rsdt, &xsdt));
  EXPECT_THAT(rsdt, t::Eq(0x12345000u));
  EXPECT_THAT(xsdt, t::Eq(0x9876543200ULL));

  // Verify that an extended RSDP v2 with length > 36 (e.g. 40 bytes with
  // non-zero trailing vendor bytes and a valid 40-byte extended checksum) is
  // accepted per ACPI 2.0+ specification.
  RawRsdpV2* const raw_v2 = reinterpret_cast<RawRsdpV2*>(v2_addr);
  raw_v2->length = 40;
  reinterpret_cast<uint8_t*>(v2_addr)[36] = 0x37;
  reinterpret_cast<uint8_t*>(v2_addr)[37] = 0x13;
  raw_v2->extended_checksum = 0;
  raw_v2->extended_checksum = ComputeChecksumByte(raw_v2, 40);
  EXPECT_TRUE(
      SmpParseRsdpForTest(v2_addr, kMaxCanonicalIdentityAddress, &rsdt, &xsdt));
  EXPECT_THAT(xsdt, t::Eq(0x9876543200ULL));
  WriteRsdpV2(v2_addr, 0x12345000, 0x9876543200ULL);

  // Out-of-bounds physical limit must fail cleanly.
  EXPECT_FALSE(SmpParseRsdpForTest(v1_addr, v1_addr + 10, &rsdt, &xsdt));

  // Corrupted v2 extended checksum must be rejected.
  raw_v2->extended_checksum ^= 0xFF;
  EXPECT_FALSE(
      SmpParseRsdpForTest(v2_addr, kMaxCanonicalIdentityAddress, &rsdt, &xsdt));
  raw_v2->extended_checksum ^= 0xFF;

  // Invalid v2 length (< 36) must be rejected.
  raw_v2->length = 20;
  raw_v2->extended_checksum = ComputeChecksumByte(raw_v2, sizeof(RawRsdpV2));
  EXPECT_FALSE(
      SmpParseRsdpForTest(v2_addr, kMaxCanonicalIdentityAddress, &rsdt, &xsdt));

  // Corrupted v1 checksum or signature must be rejected.
  RawRsdpV1* const raw_v1 = reinterpret_cast<RawRsdpV1*>(v1_addr);
  raw_v1->checksum ^= 0x01;
  EXPECT_FALSE(
      SmpParseRsdpForTest(v1_addr, kMaxCanonicalIdentityAddress, &rsdt, &xsdt));

  WriteRsdpV1(v1_addr, 0);
  EXPECT_FALSE(
      SmpParseRsdpForTest(v1_addr, kMaxCanonicalIdentityAddress, &rsdt, &xsdt));
}

TEST(SmpTest, FindRsdpInMemoryRangeHonorsAlignmentAndChecksums) {
  ResetState();
  const uintptr_t scan_base = OffsetAddr(0x1000);

  // Place a misaligned RSDP at +8 (must be skipped) and a corrupted RSDP at
  // +32 (must be skipped), followed by a valid RSDP at +64.
  WriteRsdpV1(scan_base + 8, 0x11110000);
  WriteRsdpV1(scan_base + 32, 0x22220000);
  reinterpret_cast<RawRsdpV1*>(scan_base + 32)->checksum ^= 0x55;
  WriteRsdpV1(scan_base + 64, 0x33330000);

  EXPECT_THAT(SmpFindRsdpInMemoryRange(scan_base, 1024), t::Eq(scan_base + 64));
  EXPECT_THAT(SmpFindRsdpInMemoryRange(scan_base, 64), t::Eq(0u));
  EXPECT_THAT(SmpFindRsdpInMemoryRange(scan_base, 16), t::Eq(0u));

  // Verify that a 36-byte v2 RSDP placed at +64 is rejected if the scan range
  // ends at +64 + 24 (truncating the v2 structure before 36 bytes), or if its
  // v2 extended checksum is corrupted even when its v1 checksum is valid.
  WriteRsdpV2(scan_base + 64, 0x33330000, 0x44440000ULL);
  EXPECT_THAT(SmpFindRsdpInMemoryRange(scan_base + 64, 24), t::Eq(0u));
  EXPECT_THAT(SmpFindRsdpInMemoryRange(scan_base + 64, 36),
              t::Eq(scan_base + 64));
  reinterpret_cast<RawRsdpV2*>(scan_base + 64)->extended_checksum ^= 0xFF;
  EXPECT_THAT(SmpFindRsdpInMemoryRange(scan_base + 64, 36), t::Eq(0u));
}

TEST(SmpTest, FindRsdpInMultiboot2PrefersAcpiNewAndFallsBackToOld) {
  ResetState();
  const uintptr_t mb_base = OffsetAddr(0x2000);
  uint8_t* const buf = reinterpret_cast<uint8_t*>(mb_base);

  // Build a Multiboot2 info structure with:
  // - dummy tag (type 6, size 12, padded to 16)
  // - Tag 14 (ACPI_OLD, size 28, padded to 32)
  // - Tag 15 (ACPI_NEW, size 44, padded to 48)
  // - End tag (type 0, size 8)
  int64_t pos = 8;

  // Dummy tag at offset 8
  *reinterpret_cast<uint32_t*>(buf + pos) = 6;
  *reinterpret_cast<uint32_t*>(buf + pos + 4) = 12;
  pos += 16;

  // Tag 14 (ACPI_OLD) at offset 24
  const uintptr_t old_rsdp_addr = mb_base + pos + 8;
  *reinterpret_cast<uint32_t*>(buf + pos) = 14;
  *reinterpret_cast<uint32_t*>(buf + pos + 4) = 8 + sizeof(RawRsdpV1);
  WriteRsdpV1(old_rsdp_addr, 0xAAAA0000);
  pos += 32;

  // Tag 15 (ACPI_NEW) at offset 56
  const uintptr_t new_rsdp_addr = mb_base + pos + 8;
  *reinterpret_cast<uint32_t*>(buf + pos) = 15;
  *reinterpret_cast<uint32_t*>(buf + pos + 4) = 8 + sizeof(RawRsdpV2);
  WriteRsdpV2(new_rsdp_addr, 0xAAAA0000, 0xBBBB0000ULL);
  pos += 48;

  // End tag
  *reinterpret_cast<uint32_t*>(buf + pos) = 0;
  *reinterpret_cast<uint32_t*>(buf + pos + 4) = 8;
  pos += 8;

  *reinterpret_cast<uint32_t*>(buf) = static_cast<uint32_t>(pos);
  *reinterpret_cast<uint32_t*>(buf + 4) = 0;

  // When both Tag 14 and Tag 15 are valid, Tag 15 (ACPI 2.0+) is preferred.
  EXPECT_THAT(
      SmpFindRsdpInMultiboot2ForTest(mb_base, kMaxCanonicalIdentityAddress),
      t::Eq(new_rsdp_addr));

  // If Tag 15 claims an RSDP v2 length (e.g. 512) larger than the Multiboot2
  // tag payload size (36), it must be rejected without reading out-of-bounds,
  // falling back to Tag 14.
  reinterpret_cast<RawRsdpV2*>(new_rsdp_addr)->length = 512;
  EXPECT_THAT(
      SmpFindRsdpInMultiboot2ForTest(mb_base, kMaxCanonicalIdentityAddress),
      t::Eq(old_rsdp_addr));
  WriteRsdpV2(new_rsdp_addr, 0xAAAA0000, 0xBBBB0000ULL);

  // If Tag 15 has a corrupted extended checksum, fallback to Tag 14 succeeds.
  reinterpret_cast<RawRsdpV2*>(new_rsdp_addr)->extended_checksum ^= 0xFF;
  EXPECT_THAT(
      SmpFindRsdpInMultiboot2ForTest(mb_base, kMaxCanonicalIdentityAddress),
      t::Eq(old_rsdp_addr));

  // If Tag 14 is also corrupted, but Tag 15's v1 header (rsdt_address != 0) is
  // still valid, Tag 15's v1 header is used as fallback.
  reinterpret_cast<RawRsdpV1*>(old_rsdp_addr)->checksum ^= 0xFF;
  EXPECT_THAT(
      SmpFindRsdpInMultiboot2ForTest(mb_base, kMaxCanonicalIdentityAddress),
      t::Eq(new_rsdp_addr));

  // If Tag 15's v1 checksum is also corrupted, returns 0.
  reinterpret_cast<RawRsdpV2*>(new_rsdp_addr)->v1.checksum ^= 0xFF;
  EXPECT_THAT(
      SmpFindRsdpInMultiboot2ForTest(mb_base, kMaxCanonicalIdentityAddress),
      t::Eq(0u));

  // Malformed Multiboot2 total_size (< 8 or > 4 MiB) must return 0.
  *reinterpret_cast<uint32_t*>(buf) = 4;
  EXPECT_THAT(
      SmpFindRsdpInMultiboot2ForTest(mb_base, kMaxCanonicalIdentityAddress),
      t::Eq(0u));
  *reinterpret_cast<uint32_t*>(buf) = 8 * 1024 * 1024;
  EXPECT_THAT(
      SmpFindRsdpInMultiboot2ForTest(mb_base, kMaxCanonicalIdentityAddress),
      t::Eq(0u));
}

TEST(SmpTest, FindMadtInSdtSearchesXsdtAndFallsBackToRsdt) {
  ResetState(BaseAddr());
  const uintptr_t facp_addr = OffsetAddr(0x3000);
  const uintptr_t bad_madt_addr = OffsetAddr(0x3200);
  const uintptr_t good_madt_addr = OffsetAddr(0x3400);
  const uintptr_t rsdt_addr = OffsetAddr(0x3600);
  const uintptr_t xsdt_addr = OffsetAddr(0x3800);

  // Non-MADT table ("FACP").
  FinalizeSdt(reinterpret_cast<RawSdtHeader*>(facp_addr),  //
              "FACP",                                      //
              sizeof(RawSdtHeader));

  // Corrupted MADT ("APIC" with bad checksum).
  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, 0, 1);
  WriteMadt(bad_madt_addr, kDefaultLocalApicPhysAddr, recs);
  reinterpret_cast<RawSdtHeader*>(bad_madt_addr)->checksum ^= 0x42;

  // Valid MADT ("APIC").
  WriteMadt(good_madt_addr, kDefaultLocalApicPhysAddr, recs);

  WriteXsdt(xsdt_addr, {facp_addr, bad_madt_addr, good_madt_addr});
  WriteRsdt(rsdt_addr, {static_cast<uint32_t>(facp_addr),
                        static_cast<uint32_t>(bad_madt_addr),
                        static_cast<uint32_t>(good_madt_addr)});

  // Valid XSDT resolves good_madt_addr (skipping FACP and bad_madt_addr).
  EXPECT_THAT(SmpFindMadtInSdt(0, xsdt_addr, kMaxCanonicalIdentityAddress),
              t::Eq(good_madt_addr));

  // If XSDT checksum is corrupted, falls back to 32-bit RSDT and resolves
  // good_madt_addr.
  reinterpret_cast<RawSdtHeader*>(xsdt_addr)->checksum ^= 0xFF;
  EXPECT_THAT(
      SmpFindMadtInSdt(rsdt_addr, xsdt_addr, kMaxCanonicalIdentityAddress),
      t::Eq(good_madt_addr));

  // If RSDT checksum is also corrupted, returns 0.
  reinterpret_cast<RawSdtHeader*>(rsdt_addr)->checksum ^= 0xFF;
  EXPECT_THAT(
      SmpFindMadtInSdt(rsdt_addr, xsdt_addr, kMaxCanonicalIdentityAddress),
      t::Eq(0u));
}

TEST(SmpTest, ParseMadtHandlesBspReorderingFilteringDuplicatesAndOverrides) {
  ResetState();
  const uintptr_t madt_addr = OffsetAddr(0x4000);

  std::vector<uint8_t> recs;
  // CPU with APIC ID 2 (Enabled)
  AppendLapicRecord(&recs, 10, 2, 1);
  // Disabled CPU with APIC ID 9 (flags == 0, must be ignored)
  AppendLapicRecord(&recs, 11, 9, 0);
  // Broadcast/invalid 8-bit xAPIC ID 0xFF in Type 0 (must be ignored)
  AppendLapicRecord(&recs, 99, 0xFF, 1);
  // Unknown entry type 2 (Interrupt Source Override, 10 bytes, must be skipped)
  const uint8_t iso_entry[10] = {2, 10, 0, 0, 2, 0, 0, 0, 0, 0};
  recs.insert(recs.end(), iso_entry, iso_entry + sizeof(iso_entry));
  // BSP with APIC ID 4 (Enabled)
  AppendLapicRecord(&recs, 12, 4, 1);
  // Duplicate APIC ID 2 (must be deduplicated)
  AppendLapicRecord(&recs, 13, 2, 1);
  // CPU with APIC ID 7 via Local x2APIC (Type 9)
  AppendX2ApicRecord(&recs, 7, 14, 1);
  // Broadcast/invalid 32-bit x2APIC entry 0xFFFFFFFF (must be ignored)
  AppendX2ApicRecord(&recs, kInvalidX2ApicId, 15, 1);
  // 64-bit Local APIC Address Override (Type 5)
  AppendLapicOverrideRecord(&recs, 0xFEE01000ULL);
  // Misaligned 64-bit Local APIC Address Override (must be ignored)
  AppendLapicOverrideRecord(&recs, 0xFEE02123ULL);

  // Also pass a misaligned 32-bit local_apic_address in the MADT header to
  // verify it falls back to kDefaultLocalApicPhysAddr before the valid Type 5
  // override replaces it with 0xFEE01000.
  WriteMadt(madt_addr, 0xFEE00123u, recs);

  SmpTopology topo = {};
  ASSERT_TRUE(SmpParseMadt(madt_addr, kMaxCanonicalIdentityAddress, 4, &topo));
  EXPECT_THAT(topo.local_apic_phys_addr, t::Eq(0xFEE01000ULL));
  EXPECT_THAT(topo.bsp_apic_id, t::Eq(4u));
  ASSERT_THAT(topo.cpu_count, t::Eq(3));
  EXPECT_THAT(topo.ignored_cpu_count, t::Eq(0));

  // BSP (APIC ID 4) must be placed at index 0, preserving relative order of APs
  // (APIC ID 2 at index 1, APIC ID 7 at index 2).
  EXPECT_THAT(topo.cpus[0].apic_id, t::Eq(4u));
  EXPECT_THAT(topo.cpus[0].acpi_processor_id, t::Eq(12u));
  EXPECT_TRUE(topo.cpus[0].is_bsp);
  EXPECT_TRUE(topo.cpus[0].online);

  EXPECT_THAT(topo.cpus[1].apic_id, t::Eq(2u));
  EXPECT_THAT(topo.cpus[1].acpi_processor_id, t::Eq(10u));
  EXPECT_FALSE(topo.cpus[1].is_bsp);

  EXPECT_THAT(topo.cpus[2].apic_id, t::Eq(7u));
  EXPECT_THAT(topo.cpus[2].acpi_processor_id, t::Eq(14u));
  EXPECT_FALSE(topo.cpus[2].is_bsp);

  // If bsp_apic_id (e.g. 99) was omitted from the MADT, it is inserted at
  // index 0 and existing CPUs are shifted right.
  ASSERT_TRUE(SmpParseMadt(madt_addr, kMaxCanonicalIdentityAddress, 99, &topo));
  ASSERT_THAT(topo.cpu_count, t::Eq(4));
  EXPECT_THAT(topo.cpus[0].apic_id, t::Eq(99u));
  EXPECT_THAT(topo.cpus[0].acpi_processor_id, t::Eq(0u));
  EXPECT_TRUE(topo.cpus[0].is_bsp);
  EXPECT_THAT(topo.cpus[1].apic_id, t::Eq(2u));
  EXPECT_THAT(topo.cpus[2].apic_id, t::Eq(4u));
  EXPECT_THAT(topo.cpus[3].apic_id, t::Eq(7u));
}

TEST(SmpTest, ParseMadtIgnoresProcessorsThatAreOnlyOnlineCapable) {
  ResetState();
  const uintptr_t madt_addr = OffsetAddr(0x4000);

  // Online Capable (flags bit 1) without Enabled (bit 0) marks a hot-pluggable
  // processor that is not present. Enabled wins if both bits are set.
  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, 0, 1);
  AppendLapicRecord(&recs, 1, 1, 2);
  AppendLapicRecord(&recs, 2, 2, 1);
  AppendX2ApicRecord(&recs, 3, 3, 2);
  AppendLapicRecord(&recs, 4, 4, 3);
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, recs);

  SmpTopology topo = {};
  ASSERT_TRUE(SmpParseMadt(madt_addr, kMaxCanonicalIdentityAddress, 0, &topo));
  ASSERT_THAT(topo.cpu_count, t::Eq(3));
  EXPECT_THAT(topo.cpus[0].apic_id, t::Eq(0u));
  EXPECT_THAT(topo.cpus[1].apic_id, t::Eq(2u));
  EXPECT_THAT(topo.cpus[2].apic_id, t::Eq(4u));

  // A BSP that is listed only as Online Capable is still placed at index 0.
  ASSERT_TRUE(SmpParseMadt(madt_addr, kMaxCanonicalIdentityAddress, 1, &topo));
  ASSERT_THAT(topo.cpu_count, t::Eq(4));
  EXPECT_THAT(topo.cpus[0].apic_id, t::Eq(1u));
  EXPECT_THAT(topo.cpus[0].acpi_processor_id, t::Eq(0u));

  // A table whose processors are all only Online Capable lists no CPU.
  recs.clear();
  AppendLapicRecord(&recs, 0, 0, 2);
  AppendX2ApicRecord(&recs, 1, 1, 2);
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, recs);
  EXPECT_FALSE(SmpParseMadt(madt_addr, kMaxCanonicalIdentityAddress, 0, &topo));
}

TEST(SmpTest, SmpInitDoesNotStartProcessorsThatAreOnlyOnlineCapable) {
  ResetState(BaseAddr());
  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, 0, 1);
  AppendLapicRecord(&recs, 1, 1, 2);
  AppendLapicRecord(&recs, 2, 2, 1);
  AppendX2ApicRecord(&recs, 3, 3, 2);
  const uintptr_t bios_rom_base = WriteBiosAcpiTables(recs);
  IdtInit();
  SmpSetHostTestHooks(0,              //
                      0,              //
                      bios_rom_base,  //
                      BaseAddr(),     //
                      [](const int /*cpu_index*/, CpuInfo* const cpu_info) {
                        EXPECT_THAT(cpu_info->apic_id, t::Eq(2u));
                        return StartAp(cpu_info);
                      });
  SmpInit(kMultiboot1Magic, 0);

  ASSERT_THAT(SmpCpuCount(), t::Eq(2));
  EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(2));
  EXPECT_THAT(SmpGetCpuInfo(1)->apic_id, t::Eq(2u));
  // Only one AP stack was allocated.
  EXPECT_THAT(g_state.pmm_alloc_calls, t::Eq(1));
  EXPECT_THAT(g_state.uart_log, t::HasSubstr("[SMP] Online CPUs: 2/2"));
}

TEST(SmpTest, ParseMadtHandlesMalformedEntriesAndCapsCpuCount) {
  ResetState();
  const uintptr_t madt_addr = OffsetAddr(0x5000);

  // Entry with length == 0 after two valid CPUs must terminate parsing cleanly
  // without an infinite loop.
  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, 0, 1);
  AppendLapicRecord(&recs, 1, 1, 1);
  const uint8_t zero_len_entry[2] = {0, 0};
  recs.insert(recs.end(), zero_len_entry, zero_len_entry + 2);
  AppendLapicRecord(&recs, 2, 2, 1);
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, recs);

  SmpTopology topo = {};
  ASSERT_TRUE(SmpParseMadt(madt_addr, kMaxCanonicalIdentityAddress, 0, &topo));
  EXPECT_THAT(topo.cpu_count, t::Eq(2));

  // Truncated Type 0 entry (length == 4 < 8) stops parsing cleanly.
  recs.clear();
  AppendLapicRecord(&recs, 0, 0, 1);
  const uint8_t short_type0[4] = {0, 4, 1, 1};
  recs.insert(recs.end(), short_type0, short_type0 + sizeof(short_type0));
  AppendLapicRecord(&recs, 2, 2, 1);
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, recs);
  ASSERT_TRUE(SmpParseMadt(madt_addr, kMaxCanonicalIdentityAddress, 0, &topo));
  EXPECT_THAT(topo.cpu_count, t::Eq(1));

  // MADT with only disabled CPUs must return false.
  recs.clear();
  AppendLapicRecord(&recs, 0, 0, 0);
  AppendLapicRecord(&recs, 1, 1, 0);
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, recs);
  EXPECT_FALSE(SmpParseMadt(madt_addr, kMaxCanonicalIdentityAddress, 0, &topo));

  // MADT with more enabled processors than kMaxSupportedCpus: the BSP, listed
  // last, still gets index 0, followed by the first kMaxSupportedCpus - 1 other
  // processors in table order; the remaining entries are counted as ignored.
  recs.clear();
  constexpr int kListedCpuCount = 200;
  for (int i = 1; i < kListedCpuCount; ++i) {
    AppendLapicRecord(&recs,                    //
                      static_cast<uint8_t>(i),  //
                      static_cast<uint8_t>(i),  //
                      1);
  }
  AppendLapicRecord(&recs, 0x77, 0, 1);
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, recs);
  ASSERT_TRUE(SmpParseMadt(madt_addr, kMaxCanonicalIdentityAddress, 0, &topo));
  ASSERT_THAT(topo.cpu_count, t::Eq(kMaxSupportedCpus));
  EXPECT_THAT(topo.ignored_cpu_count,
              t::Eq(kListedCpuCount - kMaxSupportedCpus));
  EXPECT_THAT(topo.cpus[0].apic_id, t::Eq(0u));
  EXPECT_THAT(topo.cpus[0].acpi_processor_id, t::Eq(0x77u));
  EXPECT_TRUE(topo.cpus[0].is_bsp);
  for (int i = 1; i < kMaxSupportedCpus; ++i) {
    EXPECT_THAT(topo.cpus[i].apic_id, t::Eq(static_cast<uint32_t>(i)));
    EXPECT_FALSE(topo.cpus[i].is_bsp);
  }

  // Exactly kMaxSupportedCpus processors fit without ignoring any.
  recs.clear();
  for (int i = 0; i < kMaxSupportedCpus; ++i) {
    AppendLapicRecord(&recs,                    //
                      static_cast<uint8_t>(i),  //
                      static_cast<uint8_t>(i),  //
                      1);
  }
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, recs);
  ASSERT_TRUE(SmpParseMadt(madt_addr, kMaxCanonicalIdentityAddress, 0, &topo));
  EXPECT_THAT(topo.cpu_count, t::Eq(kMaxSupportedCpus));
  EXPECT_THAT(topo.ignored_cpu_count, t::Eq(0));
}

TEST(SmpTest, SmpInitStartsAtMostMaxSupportedCpusAndLogsTheRest) {
  ResetState(BaseAddr());
  std::vector<uint8_t> recs;
  constexpr int kListedCpuCount = kMaxSupportedCpus + 5;
  for (int i = 0; i < kListedCpuCount; ++i) {
    AppendX2ApicRecord(&recs,                     //
                       static_cast<uint32_t>(i),  //
                       static_cast<uint32_t>(i),  //
                       1);
  }
  const uintptr_t bios_rom_base = WriteBiosAcpiTables(recs);
  IdtInit();
  SmpSetHostTestHooks(0, 0, bios_rom_base, BaseAddr(), nullptr);
  SmpInit(kMultiboot1Magic, 0);

  EXPECT_THAT(SmpCpuCount(), t::Eq(kMaxSupportedCpus));
  EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(kMaxSupportedCpus));
  EXPECT_THAT(g_state.pmm_alloc_calls, t::Eq(kMaxSupportedCpus - 1));
  EXPECT_THAT(g_state.uart_log,
              t::HasSubstr("[SMP] Ignoring 5 more processor entries (limit: "
                           "128 CPUs)"));
}

TEST(SmpTest, DiscoverTopologyAndSmpInitEndToEndAndPanicsOnFailure) {
  ResetState(BaseAddr());
  const uintptr_t mb_base = OffsetAddr(0x6000);
  const uintptr_t rsdt_addr = OffsetAddr(0x6100);
  const uintptr_t xsdt_addr = OffsetAddr(0x6200);
  const uintptr_t bad_rsdt_addr = OffsetAddr(0x6300);
  const uintptr_t madt_addr = OffsetAddr(0x6400);
  const uintptr_t bios_rom_base = OffsetAddr(0x7000);

  std::vector<uint8_t> recs;
  for (int i = 0; i < 4; ++i) {
    AppendLapicRecord(&recs,                    //
                      static_cast<uint8_t>(i),  //
                      static_cast<uint8_t>(i),  //
                      1);
  }
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, recs);
  WriteXsdt(xsdt_addr, {madt_addr});
  WriteRsdt(rsdt_addr, {static_cast<uint32_t>(madt_addr)});
  WriteRsdt(bad_rsdt_addr, {});

  // Build Multiboot2 structure with Tag 15 (ACPI_NEW) and Tag 14 (ACPI_OLD).
  uint8_t* const mb_buf = reinterpret_cast<uint8_t*>(mb_base);
  *reinterpret_cast<uint32_t*>(mb_buf) = 96;
  *reinterpret_cast<uint32_t*>(mb_buf + 4) = 0;
  *reinterpret_cast<uint32_t*>(mb_buf + 8) = 15;
  *reinterpret_cast<uint32_t*>(mb_buf + 12) = 8 + sizeof(RawRsdpV2);
  WriteRsdpV2(mb_base + 16, 0, xsdt_addr);
  *reinterpret_cast<uint32_t*>(mb_buf + 56) = 14;
  *reinterpret_cast<uint32_t*>(mb_buf + 60) = 8 + sizeof(RawRsdpV1);
  WriteRsdpV1(mb_base + 64, static_cast<uint32_t>(rsdt_addr));
  *reinterpret_cast<uint32_t*>(mb_buf + 88) = 0;
  *reinterpret_cast<uint32_t*>(mb_buf + 92) = 8;

  // Place a stale RSDP v1 pointing to an empty RSDT at bios_rom_base + 64,
  // followed by the valid RSDP v1 at bios_rom_base + 128, to verify that
  // SmpDiscoverTopology continues scanning the BIOS ROM range past a stale
  // RSDP whose SDT lacks a MADT.
  WriteRsdpV1(bios_rom_base + 64, static_cast<uint32_t>(bad_rsdt_addr));
  WriteRsdpV1(bios_rom_base + 128, static_cast<uint32_t>(rsdt_addr));
  SmpSetHostTestHooks(0, 0, bios_rom_base, BaseAddr(), nullptr);

  // Verify SmpInit succeeds via Multiboot2 Tag 15 and brings all 4 CPUs online.
  SmpInit(kMultiboot2Magic, mb_base);
  EXPECT_THAT(SmpCpuCount(), t::Eq(4));
  EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(4));
  EXPECT_THAT(SmpLocalApicPhysAddr(), t::Eq(kDefaultLocalApicPhysAddr));
  EXPECT_THAT(g_state.pmm_alloc_calls, t::Eq(3));

  // If Tag 15's XSDT is corrupted and Tag 15 has rsdt_address == 0,
  // SmpDiscoverTopology must fall back to Multiboot2 Tag 14 (ACPI_OLD).
  reinterpret_cast<RawSdtHeader*>(xsdt_addr)->checksum ^= 0xFF;
  SmpInit(kMultiboot2Magic, mb_base);
  EXPECT_THAT(SmpCpuCount(), t::Eq(4));
  EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(4));
  reinterpret_cast<RawSdtHeader*>(xsdt_addr)->checksum ^= 0xFF;

  for (int i = 0; i < 4; ++i) {
    const CpuInfo* const info = SmpGetCpuInfo(i);
    ASSERT_THAT(info, t::NotNull());
    EXPECT_THAT(info->apic_id, t::Eq(i));
    EXPECT_THAT(info->observed_apic_id, t::Eq(i));
    EXPECT_THAT(info->is_bsp, t::Eq(i == 0));
    EXPECT_TRUE(info->online);
    EXPECT_TRUE(info->long_mode_active);
    EXPECT_THAT(info->stack_top - info->stack_base, t::Eq(kApStackSize));
  }
  EXPECT_DEATH(SmpGetCpuInfo(-1),
               "Check failed: index >= 0 && index < g_topology\\.cpu_count");
  EXPECT_DEATH(SmpGetCpuInfo(4),
               "Check failed: index >= 0 && index < g_topology\\.cpu_count");

  // Verify SmpInit also succeeds via Legacy BIOS ROM scan (skipping the stale
  // RSDP at +64 and discovering the valid RSDP v1 -> RSDT at +128) under
  // Multiboot1.
  SmpInit(kMultiboot1Magic, 0);
  EXPECT_THAT(SmpCpuCount(), t::Eq(4));
  EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(4));

  // Verify SmpInit panics via CHECK if topology discovery fails, if mapping
  // the Local APIC MMIO page fails, or if PMM stack allocation fails.
  SmpSetHostTestHooks(0, 0, 0, BaseAddr(), nullptr);
  EXPECT_DEATH(SmpInit(kMultiboot1Magic, 0),
               "Check failed: SmpDiscoverTopology\\(multiboot_magic");

  SmpSetHostTestHooks(0, 0, bios_rom_base, BaseAddr(), nullptr);
  g_state.fail_map_addr = kDefaultLocalApicPhysAddr;
  EXPECT_DEATH(
      SmpInit(kMultiboot2Magic, mb_base),
      "Check failed: PagingMapBootstrapRange\\(lapic_base, kPageSize\\)");
  g_state.fail_map_addr = 0;

  g_state.pmm_alloc_calls = 0;
  g_state.fail_pmm_on_call = 2;
  EXPECT_DEATH(SmpInit(kMultiboot2Magic, mb_base),
               "Check failed: stack_phys != 0");
  g_state.fail_pmm_on_call = -1;
}

TEST(SmpTest, SmpInitLeavesOutApsThatDoNotStart) {
  ResetState(BaseAddr());
  const uintptr_t bios_rom_base = WriteBiosAcpiTables(LapicRecords(5));
  IdtInit();
  SmpSetHostTestHooks(0,              //
                      0,              //
                      bios_rom_base,  //
                      BaseAddr(),     //
                      [](const int /*cpu_index*/, CpuInfo* const cpu_info) {
                        // The CPUs with APIC IDs 2 and 4 never report in.
                        if (cpu_info->apic_id == 2 || cpu_info->apic_id == 4) {
                          return false;
                        }
                        return StartAp(cpu_info);
                      });
  SmpInit(kMultiboot1Magic, 0);

  ASSERT_THAT(SmpCpuCount(), t::Eq(3));
  EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(3));
  constexpr uint32_t kExpectedApicIds[] = {0, 1, 3};
  for (int i = 0; i < 3; ++i) {
    const CpuInfo* const info = SmpGetCpuInfo(i);
    const CpuLocal* const local = SmpGetCpuLocal(i);
    EXPECT_THAT(info->apic_id, t::Eq(kExpectedApicIds[i]));
    EXPECT_TRUE(info->online);
    EXPECT_THAT(info->stack_top - info->stack_base, t::Eq(kApStackSize));
    EXPECT_THAT(local->self, t::Eq(local));
    EXPECT_THAT(local->cpu_id, t::Eq(i));
    EXPECT_THAT(local->apic_id, t::Eq(kExpectedApicIds[i]));
    EXPECT_THAT(local->stack_base, t::Eq(info->stack_base));
    EXPECT_TRUE(local->online);
  }
  // The stacks of the CPUs that did not start were returned to the PMM: the
  // second and fourth of the four AP stacks, allocated from frame 32 on.
  EXPECT_THAT(
      g_state.freed_frames,
      t::ElementsAre(t::Pair(OffsetAddr(36 * kPageSize), kApStackFrames),
                     t::Pair(OffsetAddr(44 * kPageSize), kApStackFrames)));
  EXPECT_THAT(SmpGetCpuInfo(2)->stack_base, t::Eq(OffsetAddr(40 * kPageSize)));
  EXPECT_THAT(
      g_state.uart_log,
      t::HasSubstr("[SMP] CPU 2 (APIC ID 2): did not start; continuing without "
                   "it\n[SMP] CPU 2 (APIC ID 3, AP): online\n[SMP] CPU 3 (APIC "
                   "ID 4): did not start; continuing without it\n"));
  EXPECT_THAT(g_state.uart_log, t::HasSubstr("[SMP] Online CPUs: 3/3"));

  // The remaining CPUs work as a normal topology.
  std::atomic<int> ran_on_cpus{0};
  SmpRunOnAllCpus(
      [](const int cpu_index, void* const ctx) {
        EXPECT_THAT(CurrentCpuId(), t::Eq(cpu_index));
        static_cast<std::atomic<int>*>(ctx)->fetch_add(1);
      },
      &ran_on_cpus);
  EXPECT_THAT(ran_on_cpus.load(), t::Eq(3));
}

TEST(SmpTest, PreconditionViolationsTriggerDcheck) {
  ResetState(BaseAddr());
  const uintptr_t addr = OffsetAddr(0x100);
  uintptr_t rsdt = 0;
  uintptr_t xsdt = 0;
  SmpTopology topo = {};
  constexpr const char* kZeroMaxPhysicalAddr =
      "Check failed: max_physical_addr > 0";
  constexpr const char* kInvalidBspApicId =
      "Check failed: bsp_apic_id != kInvalidX2ApicId";
  constexpr const char* kNullTopology = "Check failed: out_topology != nullptr";

  EXPECT_DEATH(SmpFindRsdpInMultiboot2ForTest(0, kMaxCanonicalIdentityAddress),
               "Check failed: multiboot_info_addr != 0");
  EXPECT_DEATH(SmpFindRsdpInMultiboot2ForTest(addr, 0), kZeroMaxPhysicalAddr);

  EXPECT_DEATH(SmpFindRsdpInMemoryRange(0, 1024),
               "Check failed: range_start != 0");
  EXPECT_DEATH(SmpFindRsdpInMemoryRange(addr, 0),
               "Check failed: range_length > 0");
  EXPECT_DEATH(SmpFindRsdpInMemoryRange(addr, -16),
               "Check failed: range_length > 0");

  EXPECT_DEATH(
      SmpParseRsdpForTest(0, kMaxCanonicalIdentityAddress, &rsdt, &xsdt),
      "Check failed: rsdp_addr != 0");
  EXPECT_DEATH(SmpParseRsdpForTest(addr, 0, &rsdt, &xsdt),
               kZeroMaxPhysicalAddr);
  EXPECT_DEATH(
      SmpParseRsdpForTest(addr, kMaxCanonicalIdentityAddress, nullptr, &xsdt),
      "Check failed: out_rsdt_addr != nullptr");
  EXPECT_DEATH(
      SmpParseRsdpForTest(addr, kMaxCanonicalIdentityAddress, &rsdt, nullptr),
      "Check failed: out_xsdt_addr != nullptr");

  EXPECT_DEATH(SmpFindMadtInSdt(0, 0, kMaxCanonicalIdentityAddress),
               "Check failed: rsdt_addr != 0 \\|\\| xsdt_addr != 0");
  EXPECT_DEATH(SmpFindMadtInSdt(addr, 0, 0), kZeroMaxPhysicalAddr);

  EXPECT_DEATH(SmpParseMadt(0, kMaxCanonicalIdentityAddress, 0, &topo),
               "Check failed: madt_addr != 0");
  EXPECT_DEATH(SmpParseMadt(addr, 0, 0, &topo), kZeroMaxPhysicalAddr);
  EXPECT_DEATH(
      SmpParseMadt(addr, kMaxCanonicalIdentityAddress, kInvalidX2ApicId, &topo),
      kInvalidBspApicId);
  EXPECT_DEATH(SmpParseMadt(addr, kMaxCanonicalIdentityAddress, 0, nullptr),
               kNullTopology);

  EXPECT_DEATH(SmpDiscoverTopology(kMultiboot2Magic, 0, 0, 0, 0, 0, &topo),
               kZeroMaxPhysicalAddr);
  EXPECT_DEATH(SmpDiscoverTopology(kMultiboot2Magic,              //
                                   0,                             //
                                   0,                             //
                                   0,                             //
                                   kMaxCanonicalIdentityAddress,  //
                                   kInvalidX2ApicId,              //
                                   &topo),
               kInvalidBspApicId);
  EXPECT_DEATH(SmpDiscoverTopology(kMultiboot2Magic,              //
                                   0,                             //
                                   0,                             //
                                   0,                             //
                                   kMaxCanonicalIdentityAddress,  //
                                   0,                             //
                                   nullptr),
               kNullTopology);

  // Calling SMP query accessors or SmpRunOnAllCpus before SmpInit succeeds
  // must trigger DCHECK.
  constexpr const char* kNotInitialized = "Check failed: g_smp_initialized";
  EXPECT_DEATH(SmpCpuCount(), kNotInitialized);
  EXPECT_DEATH(SmpOnlineCpuCount(), kNotInitialized);
  EXPECT_DEATH(SmpGetCpuInfo(0), kNotInitialized);
  EXPECT_DEATH(SmpGetCpuLocal(0), kNotInitialized);
  EXPECT_DEATH(SmpLocalApicPhysAddr(), kNotInitialized);
  EXPECT_DEATH(SmpCalibratedTimerInitialCount(), kNotInitialized);
  EXPECT_DEATH(SmpSendIpi(0, kVectorWakeupIpi), kNotInitialized);
  EXPECT_DEATH(SmpRunOnAllCpus([](const int, void* const) {}, nullptr),
               kNotInitialized);
  EXPECT_DEATH(BindCpuLocal(nullptr), "Check failed: cpu != nullptr");
  CpuLocal invalid_self = {};
  EXPECT_DEATH(BindCpuLocal(&invalid_self), "Check failed: cpu->self == cpu");
  EXPECT_DEATH(CurrentCpu(), "Check failed: cpu != nullptr");
  EXPECT_DEATH(CurrentCpuId(), "Check failed: cpu != nullptr");
  constexpr const char* kBadPitReloadCount =
      "Check failed: pit_reload_count > 0 && pit_reload_count <= "
      "kMaxPitReloadCount";
  EXPECT_DEATH(ComputeApicTimerInitialCount(1000, 0, 100), kBadPitReloadCount);
  EXPECT_DEATH(ComputeApicTimerInitialCount(1000, 0x10000, 100),
               kBadPitReloadCount);
  EXPECT_DEATH(
      ComputeApicTimerInitialCount(1000, 10000, 0),
      "Check failed: target_hz > 0 && target_hz <= kPitBaseFrequencyHz");
}

TEST(SmpTest, CpuLocalLayoutAlignmentAndThreadLocalBinding) {
  static_assert(alignof(CpuLocal) == 64, "CpuLocal must be 64-byte aligned");
  static_assert(offsetof(CpuLocal, self) == 0,
                "CpuLocal::self must be at offset 0 for gs:[0] access");

  ResetState();
  EXPECT_THAT(CurrentCpuOrNull(), t::IsNull());
  EXPECT_THAT(CurrentCpuOrNullForPanic(), t::IsNull());

  constexpr int kThreadCount = 4;
  alignas(64) CpuLocal locals[kThreadCount] = {};
  std::atomic<int> ready{0};
  std::vector<std::thread> threads;
  threads.reserve(kThreadCount);

  for (int i = 0; i < kThreadCount; ++i) {
    locals[i].self = &locals[i];
    locals[i].cpu_id = i;
    locals[i].apic_id = static_cast<uint32_t>(i + 10);
    locals[i].online = true;
    threads.emplace_back([i, &locals, &ready]() {
      BindCpuLocal(&locals[i]);
      ready.fetch_add(1, std::memory_order_acq_rel);
      while (ready.load(std::memory_order_acquire) < kThreadCount) {
        asm volatile("pause" : : : "memory");
      }
      for (int iter = 0; iter < 200; ++iter) {
        ASSERT_THAT(CurrentCpuOrNull(), t::Eq(&locals[i]));
        ASSERT_THAT(CurrentCpuOrNullForPanic(), t::Eq(&locals[i]));
        ASSERT_THAT(CurrentCpu(), t::Eq(&locals[i]));
        ASSERT_THAT(CurrentCpu()->self, t::Eq(&locals[i]));
        ASSERT_THAT(CurrentCpuId(), t::Eq(i));
        ASSERT_THAT(CurrentCpu()->apic_id,
                    t::Eq(static_cast<uint32_t>(i + 10)));
        CurrentCpu()->timer_ticks.fetch_add(1, std::memory_order_relaxed);
      }
      ResetCpuLocalForTest();
    });
  }

  for (std::thread& th : threads) {
    th.join();
  }
  for (int i = 0; i < kThreadCount; ++i) {
    EXPECT_THAT(locals[i].timer_ticks.load(std::memory_order_acquire),
                t::Eq(200));
  }
}

TEST(SmpTest,
     CalibratedTimerIpiAndEoiUpdatePerCpuStateAndEnforcePreconditions) {
  ResetState(BaseAddr());
  EXPECT_THAT(ComputeApicTimerInitialCount(0, 11932, 100), t::Eq(0u));
  EXPECT_THAT(ComputeApicTimerInitialCount(50000, 11932, 100), t::Eq(49999u));
  EXPECT_THAT(ComputeApicTimerInitialCount(0xFFFFFFFFu, 1, 1),
              t::Eq(0xFFFFFFFFu));
  EXPECT_THAT(ComputeApicTimerInitialCount(UINT64_MAX, 11932, 250),
              t::Eq(0xFFFFFFFFu));

  const uintptr_t rsdt_addr = OffsetAddr(0x6100);
  const uintptr_t madt_addr = OffsetAddr(0x6400);
  const uintptr_t bios_rom_base = OffsetAddr(0x7000);

  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, 2, 1);
  AppendLapicRecord(&recs, 1, 5, 1);
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, recs);
  WriteRsdt(rsdt_addr, {static_cast<uint32_t>(madt_addr)});
  WriteRsdpV1(bios_rom_base + 64, static_cast<uint32_t>(rsdt_addr));

  IdtInit();
  SmpSetHostTestHooks(2, 0, bios_rom_base, BaseAddr(), nullptr);
  SmpSetHostTimerCalibrationTicksForTest(60000);
  SmpInit(kMultiboot1Magic, 0);

  const uint32_t expected_initial = ComputeApicTimerInitialCount(
      60000, kPitCalibrationReloadCount, kApicTimerTargetHz);
  EXPECT_THAT(SmpCalibratedTimerInitialCount(), t::Eq(expected_initial));
  // IdtLoad called once by IdtInit (BSP) and once by SmpInit for AP 1.
  EXPECT_THAT(IdtGetHostLoadCountForTest(), t::Eq(2));

  for (int i = 0; i < SmpCpuCount(); ++i) {
    const CpuLocal* const cpu_local = SmpGetCpuLocal(i);
    ASSERT_THAT(cpu_local, t::NotNull());
    EXPECT_THAT(cpu_local->self, t::Eq(cpu_local));
    EXPECT_THAT(cpu_local->cpu_id, t::Eq(i));
    EXPECT_THAT(cpu_local->apic_id, t::Eq(SmpGetCpuInfo(i)->apic_id));
    EXPECT_THAT(cpu_local->stack_base, t::Eq(SmpGetCpuInfo(i)->stack_base));
    EXPECT_THAT(cpu_local->stack_top, t::Eq(SmpGetCpuInfo(i)->stack_top));
    EXPECT_TRUE(cpu_local->online);
    EXPECT_GE(cpu_local->timer_ticks.load(std::memory_order_acquire), 1);
  }
  EXPECT_THAT(CurrentCpu(), t::Eq(SmpGetCpuLocal(0)));
  EXPECT_THAT(CurrentCpuId(), t::Eq(0));

  // Verify SmpSendLocalApicEoi increments host EOI count.
  SmpResetHostEoiCountForTest();
  SmpSendLocalApicEoi();
  EXPECT_THAT(SmpGetHostEoiCountForTest(), t::Eq(1));

  // Verify SmpSendIpi increments target CpuLocal::ipi_count on vector 0x21 and
  // sends EOI.
  const int64_t ap_ipi_before =
      SmpGetCpuLocal(1)->ipi_count.load(std::memory_order_acquire);
  SmpSendIpi(1, kVectorWakeupIpi);
  EXPECT_THAT(SmpGetCpuLocal(1)->ipi_count.load(std::memory_order_acquire),
              t::Eq(ap_ipi_before + 1));
  EXPECT_THAT(SmpGetHostEoiCountForTest(), t::Eq(2));

  // Out-of-range SmpGetCpuLocal or SmpSendIpi arguments must trigger DCHECK.
  constexpr const char* kBadCpuLocalIndex =
      "Check failed: cpu_index >= 0 && cpu_index < g_topology\\.cpu_count";
  constexpr const char* kBadIpiTarget =
      "Check failed: target_cpu_index >= 0 && target_cpu_index < "
      "g_topology\\.cpu_count";
  EXPECT_DEATH(SmpGetCpuLocal(-1), kBadCpuLocalIndex);
  EXPECT_DEATH(SmpGetCpuLocal(SmpCpuCount()), kBadCpuLocalIndex);
  EXPECT_DEATH(SmpSendIpi(-1, kVectorWakeupIpi), kBadIpiTarget);
  EXPECT_DEATH(SmpSendIpi(SmpCpuCount(), kVectorWakeupIpi), kBadIpiTarget);
  EXPECT_DEATH(SmpSendIpi(1, 0x1F),
               "Check failed: vector >= "
               "static_cast<uint8_t>\\(kCpuExceptionCount\\)");
  // The spurious vector never gets an EOI, so a fixed IPI on it would block
  // the target's lower-priority interrupts for good.
  EXPECT_DEATH(SmpSendIpi(1, kVectorSpurious),
               "Check failed: vector != kVectorSpurious");
}

TEST(SmpTest, TimerCountFallsBackToCpuidClockThenToFixedCountWithoutPit) {
  const std::vector<uint8_t> recs = LapicRecords(1);
  struct Case {
    const char* name;
    uint64_t pit_calibration_ticks;
    uint32_t max_leaf;
    uint32_t leaf15_ebx;
    uint32_t leaf15_ecx;
    uint32_t leaf16_ecx;
    uint32_t expected_count;
    // Expected fallback message, or nullptr if PIT calibration worked.
    const char* expected_log;
  };
  const Case kCases[] = {
      {"PIT calibration", 60000, 0x16, 2, 24000000, 100,
       ComputeApicTimerInitialCount(60000, kPitCalibrationReloadCount,
                                    kApicTimerTargetHz),
       nullptr},
      // 24 MHz core crystal clock / 16 / 250 Hz.
      {"CPUID crystal clock", 0, 0x15, 2, 24000000, 0, 6000,
       "[SMP] No PIT calibration; Local APIC timer clock from CPUID: 24000000 "
       "Hz"},
      // Without a TSC/crystal ratio in leaf 0x15, the timer runs at the
      // 100 MHz bus clock of leaf 0x16.
      {"CPUID bus clock", 0, 0x16, 0, 24000000, 100, 25000,
       "[SMP] No PIT calibration; Local APIC timer clock from CPUID: "
       "100000000 Hz"},
      // Leaf 0x15 says the timer runs at the crystal clock, but does not
      // enumerate its frequency.
      {"unknown crystal clock", 0, 0x16, 2, 0, 100,
       kFallbackApicTimerInitialCount,
       "[SMP] WARNING: no PIT or CPUID timer frequency"},
      {"no CPUID data", 0, 0x0B, 0, 0, 0, kFallbackApicTimerInitialCount,
       "[SMP] WARNING: no PIT or CPUID timer frequency; LAPIC timer rate is "
       "uncalibrated"},
  };
  for (const Case& c : kCases) {
    SCOPED_TRACE(c.name);
    ResetState(BaseAddr());
    const uintptr_t bios_rom_base = WriteBiosAcpiTables(recs);
    IdtInit();
    SmpSetHostTestHooks(0, 0, bios_rom_base, BaseAddr(), nullptr);
    g_cpu.max_leaf = c.max_leaf;
    g_cpu.leaf15_ebx = c.leaf15_ebx;
    g_cpu.leaf15_ecx = c.leaf15_ecx;
    g_cpu.leaf16_ecx = c.leaf16_ecx;
    InstallFakeCpuHooks();
    SmpSetHostTimerCalibrationTicksForTest(c.pit_calibration_ticks);
    SmpInit(kMultiboot1Magic, 0);
    EXPECT_FALSE(SmpIsX2ApicEnabled());
    EXPECT_THAT(SmpCalibratedTimerInitialCount(), t::Eq(c.expected_count));
    if (c.expected_log == nullptr) {
      EXPECT_THAT(g_state.uart_log, t::Not(t::HasSubstr("No PIT calibration")));
      EXPECT_THAT(g_state.uart_log, t::Not(t::HasSubstr("WARNING")));
    } else {
      EXPECT_THAT(g_state.uart_log, t::HasSubstr(c.expected_log));
    }
    // The last-resort warning also goes to the screen.
    if (c.expected_count == kFallbackApicTimerInitialCount) {
      EXPECT_THAT(g_state.vga_log,
                  t::HasSubstr("[SMP] WARNING: no PIT or CPUID"));
    } else {
      EXPECT_THAT(g_state.vga_log, t::Not(t::HasSubstr("WARNING")));
    }
  }
}

TEST(SmpTest,
     SmpRunOnAllCpusDispatchesConcurrentlyAcrossSingleAndMultipleCpus) {
  ResetState(BaseAddr());
  const uintptr_t rsdt_addr = OffsetAddr(0x6100);
  const uintptr_t madt_addr = OffsetAddr(0x6400);
  const uintptr_t bios_rom_base = OffsetAddr(0x7000);

  // 1. Single-CPU topology: SmpRunOnAllCpus executes work_fn on CPU 0.
  std::vector<uint8_t> single_recs;
  AppendLapicRecord(&single_recs, 0, 0, 1);
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, single_recs);
  WriteRsdt(rsdt_addr, {static_cast<uint32_t>(madt_addr)});
  WriteRsdpV1(bios_rom_base + 64, static_cast<uint32_t>(rsdt_addr));
  IdtInit();
  SmpSetHostTestHooks(0, 0, bios_rom_base, BaseAddr(), nullptr);
  SmpInit(kMultiboot1Magic, 0);
  ASSERT_THAT(SmpCpuCount(), t::Eq(1));

  int single_cpu_calls = 0;
  SmpRunOnAllCpus(
      [](const int cpu_index, void* const ctx) {
        EXPECT_THAT(cpu_index, t::Eq(0));
        EXPECT_THAT(CurrentCpuId(), t::Eq(0));
        EXPECT_THAT(CurrentCpu(), t::Eq(SmpGetCpuLocal(0)));
        ++(*static_cast<int*>(ctx));
      },
      &single_cpu_calls);
  EXPECT_THAT(single_cpu_calls, t::Eq(1));

  // Null work_fn or nested SmpRunOnAllCpus must fail a check.
  EXPECT_DEATH(SmpRunOnAllCpus(nullptr, nullptr),
               "Check failed: work_fn != nullptr");
  EXPECT_DEATH(SmpRunOnAllCpus(
                   [](const int, void* const) {
                     SmpRunOnAllCpus([](const int, void* const) {}, nullptr);
                   },
                   nullptr),
               "Check failed: !dispatch_already_running");

  // 2. Multi-CPU topology (4 CPUs): SmpRunOnAllCpus executes concurrently
  //    across all 4 CPUs with a simultaneous barrier and IrqSpinLock
  //    synchronization across multiple dispatch rounds.
  ResetState(BaseAddr());
  std::vector<uint8_t> multi_recs;
  for (int i = 0; i < 4; ++i) {
    AppendLapicRecord(&multi_recs,              //
                      static_cast<uint8_t>(i),  //
                      static_cast<uint8_t>(i),  //
                      1);
  }
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, multi_recs);
  WriteRsdt(rsdt_addr, {static_cast<uint32_t>(madt_addr)});
  WriteRsdpV1(bios_rom_base + 64, static_cast<uint32_t>(rsdt_addr));
  IdtInit();
  SmpSetHostTestHooks(0, 0, bios_rom_base, BaseAddr(), nullptr);
  SmpInit(kMultiboot1Magic, 0);
  ASSERT_THAT(SmpCpuCount(), t::Eq(4));

  struct DispatchTestState {
    IrqSpinLock lock;
    std::atomic<int> arrived{0};
    int64_t total_counter = 0;
    int per_cpu_runs[4] = {};
  };

  DispatchTestState dispatch_state = {};
  for (int round = 0; round < 3; ++round) {
    dispatch_state.arrived.store(0, std::memory_order_release);
    SmpRunOnAllCpus(
        [](const int cpu_index, void* const raw_ctx) {
          DispatchTestState* const st =
              static_cast<DispatchTestState*>(raw_ctx);
          ASSERT_GE(cpu_index, 0);
          ASSERT_LT(cpu_index, 4);
          EXPECT_THAT(CurrentCpu(), t::Eq(SmpGetCpuLocal(cpu_index)));
          EXPECT_THAT(CurrentCpu()->self, t::Eq(CurrentCpu()));
          EXPECT_THAT(CurrentCpuId(), t::Eq(cpu_index));

          // Wait for all 4 CPUs to arrive concurrently.
          st->arrived.fetch_add(1, std::memory_order_acq_rel);
          while (st->arrived.load(std::memory_order_acquire) < 4) {
            asm volatile("pause" : : : "memory");
          }

          for (int iter = 0; iter < 500; ++iter) {
            const IrqSpinLockGuard guard(st->lock);
            ++st->total_counter;
          }
          const IrqSpinLockGuard guard(st->lock);
          ++st->per_cpu_runs[cpu_index];
        },
        &dispatch_state);
  }

  EXPECT_THAT(dispatch_state.total_counter, t::Eq(3 * 4 * 500));
  EXPECT_THAT(dispatch_state.per_cpu_runs, t::ElementsAre(3, 3, 3, 3));
  for (int i = 1; i < 4; ++i) {
    EXPECT_GE(SmpGetCpuLocal(i)->ipi_count.load(std::memory_order_acquire), 3);
  }
}

TEST(SmpTest, SmpRunOnAllCpusChecksItsCallingContext) {
  ResetState(BaseAddr());
  const uintptr_t bios_rom_base = WriteBiosAcpiTables(LapicRecords(4));
  IdtInit();
  SmpSetHostTestHooks(0, 0, bios_rom_base, BaseAddr(), nullptr);
  SmpInit(kMultiboot1Magic, 0);
  ASSERT_THAT(SmpCpuCount(), t::Eq(4));

  // Only CPU 0 can dispatch: APs pick up work only in their idle loop.
  EXPECT_DEATH(
      {
        BindCpuLocal(SmpGetCpuLocal(2));
        SmpRunOnAllCpus([](const int, void* const) {}, nullptr);
      },
      "Check failed: CurrentCpuId\\(\\) == 0");
  // An AP's work_fn might need a lock that the caller holds.
  EXPECT_DEATH(
      {
        IrqSpinLock lock;
        const IrqSpinLockGuard guard(lock);
        SmpRunOnAllCpus([](const int, void* const) {}, nullptr);
      },
      "Check failed: !HoldsAnyIrqSpinLock\\(\\)");
  EXPECT_DEATH(
      {
        SetInterruptsEnabledForTest(false);
        SmpRunOnAllCpus([](const int, void* const) {}, nullptr);
      },
      "Check failed: AreInterruptsEnabled\\(\\)");

  // work_fn runs with interrupts enabled on every CPU, CPU 0 included, and the
  // caller's interrupt state is restored afterwards.
  std::atomic<int> interrupts_enabled_count{0};
  SmpRunOnAllCpus(
      [](const int, void* const ctx) {
        if (AreInterruptsEnabled()) {
          static_cast<std::atomic<int>*>(ctx)->fetch_add(1);
        }
      },
      &interrupts_enabled_count);
  EXPECT_THAT(interrupts_enabled_count.load(), t::Eq(4));
  EXPECT_TRUE(AreInterruptsEnabled());
  EXPECT_THAT(CurrentCpuId(), t::Eq(0));
}

TEST(SmpTest, CalibrateApicTimerCountBoundedLoopGuardAndZeroOnFailure) {
  static int g_speaker_polls = 0;
  static int g_ready_after_poll = 0;
  static uint32_t g_mock_current_count = 0;
  g_speaker_polls = 0;
  g_ready_after_poll = 0;
  g_mock_current_count = 0;

  const auto read_speaker_timeout = +[]() -> uint8_t {
    ++g_speaker_polls;
    return 0x00;  // Bit 5 (0x20) is never set.
  };
  const auto read_apic_count =
      +[](const uintptr_t) -> uint32_t { return g_mock_current_count; };

  // 1. Timeout when PIT port 0x61 bit 5 never sets: bounded to max_polls and
  // returns 0 without hanging.
  g_speaker_polls = 0;
  g_mock_current_count = 0xFFFFFFFFu - 50000u;
  const uint32_t timeout_count = CalibrateApicTimerCount(
      kDefaultLocalApicPhysAddr, 256, read_speaker_timeout, read_apic_count);
  EXPECT_THAT(g_speaker_polls, t::Eq(256));
  EXPECT_THAT(timeout_count, t::Eq(0u));

  // 2. Normal completion before max_polls: returns calibrated count.
  const auto read_speaker_ready = +[]() -> uint8_t {
    ++g_speaker_polls;
    return (g_speaker_polls >= g_ready_after_poll) ? 0x20 : 0x00;
  };
  g_speaker_polls = 0;
  g_ready_after_poll = 7;
  g_mock_current_count = 0xFFFFFFFFu - 60000u;
  const uint32_t calibrated_count = CalibrateApicTimerCount(
      kDefaultLocalApicPhysAddr, 256, read_speaker_ready, read_apic_count);
  EXPECT_THAT(g_speaker_polls, t::Eq(7));
  EXPECT_THAT(calibrated_count,
              t::Eq(ComputeApicTimerInitialCount(
                  60000, kPitCalibrationReloadCount, kApicTimerTargetHz)));

  // 3. Zero elapsed APIC ticks even when terminal count reached returns 0.
  g_speaker_polls = 0;
  g_ready_after_poll = 1;
  g_mock_current_count = 0xFFFFFFFFu;
  const uint32_t zero_elapsed_count = CalibrateApicTimerCount(
      kDefaultLocalApicPhysAddr, 256, read_speaker_ready, read_apic_count);
  EXPECT_THAT(zero_elapsed_count, t::Eq(0u));
}

TEST(SmpTest,
     ParseMadtAndX2ApicSupport32BitApicIdsCpuidLeaf0BAndMsrOperations) {
  ResetState(BaseAddr());
  const uintptr_t rsdt_addr = OffsetAddr(0x6100);
  const uintptr_t madt_addr = OffsetAddr(0x6400);
  const uintptr_t bios_rom_base = OffsetAddr(0x7000);

  // 1. MADT Type 9 (Local x2APIC) entries with 32-bit APIC IDs >= 0xFF (0xFF,
  // 0x100, 0x12345678, 0xFFFFFFFE) and 32-bit ACPI processor UIDs are
  // preserved without 8-bit truncation, while 0xFFFFFFFF (kInvalidX2ApicId) is
  // rejected and Type 0 0xFF (kInvalidXapicId) is rejected.
  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, kInvalidXapicId, 1);
  AppendX2ApicRecord(&recs, 0xFFu, 0x10000001u, 1);
  AppendX2ApicRecord(&recs, 0x100u, 0x20000002u, 1);
  AppendX2ApicRecord(&recs, 0x12345678u, 0x30000003u, 1);
  AppendX2ApicRecord(&recs, 0xFFFFFFFEu, 0x40000004u, 1);
  AppendX2ApicRecord(&recs, kInvalidX2ApicId, 0x50000005u, 1);
  // Duplicate 32-bit x2APIC ID must be deduplicated without colliding on low 8
  // bits (0x200 vs 0x100 have identical low 8 bits 0x00, so both must remain!).
  AppendX2ApicRecord(&recs, 0x100u, 0x99999999u, 1);
  AppendX2ApicRecord(&recs, 0x200u, 0x60000006u, 1);
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, recs);

  SmpTopology topo = {};
  ASSERT_TRUE(SmpParseMadt(madt_addr,                     //
                           kMaxCanonicalIdentityAddress,  //
                           0x12345678u,                   //
                           &topo));
  EXPECT_THAT(topo.bsp_apic_id, t::Eq(0x12345678u));
  ASSERT_THAT(topo.cpu_count, t::Eq(5));

  EXPECT_THAT(topo.cpus[0].apic_id, t::Eq(0x12345678u));
  EXPECT_THAT(topo.cpus[0].acpi_processor_id, t::Eq(0x30000003u));
  EXPECT_THAT(topo.cpus[0].observed_apic_id, t::Eq(0x12345678u));
  EXPECT_TRUE(topo.cpus[0].is_bsp);

  EXPECT_THAT(topo.cpus[1].apic_id, t::Eq(0xFFu));
  EXPECT_THAT(topo.cpus[1].acpi_processor_id, t::Eq(0x10000001u));
  EXPECT_THAT(topo.cpus[2].apic_id, t::Eq(0x100u));
  EXPECT_THAT(topo.cpus[2].acpi_processor_id, t::Eq(0x20000002u));
  EXPECT_THAT(topo.cpus[3].apic_id, t::Eq(0xFFFFFFFEu));
  EXPECT_THAT(topo.cpus[3].acpi_processor_id, t::Eq(0x40000004u));
  EXPECT_THAT(topo.cpus[4].apic_id, t::Eq(0x200u));
  EXPECT_THAT(topo.cpus[4].acpi_processor_id, t::Eq(0x60000006u));

  // 2. ReadCpuidInitialApicId queries CPUID leaf 0x0B EDX when x2APIC is
  // supported and leaf 0x0B is populated, and falls back to
  // CPUID.01H:EBX[31:24] otherwise.
  static uint32_t g_cpuid_max_leaf = 0;
  static uint32_t g_leaf1_ebx = 0;
  static uint32_t g_leaf1_ecx = 0;
  static uint32_t g_leafb_ebx = 0;
  static uint32_t g_leafb_ecx = 0;
  static uint32_t g_leafb_edx = 0;
  static uint64_t g_apic_base_msr = 0;
  static uint32_t g_msr_lapic_id = 0;
  static std::vector<std::pair<uint32_t, uint64_t>> g_msr_writes;
  g_cpuid_max_leaf = 0x0Bu;
  g_leaf1_ebx = 0x78u << 24;
  g_leaf1_ecx = 1u << 21;  // x2APIC supported
  g_leafb_ebx = 2u;        // 2 logical processors at level 0
  g_leafb_ecx = 1u << 8;   // level type = SMT (1)
  g_leafb_edx = 0x12345678u;
  g_apic_base_msr = 0xFEE00900ULL;
  g_msr_lapic_id = 0x12345678u;
  g_msr_writes.clear();

  const auto cpuid_hook = +[](const uint32_t leaf,     //
                              const uint32_t subleaf,  //
                              uint32_t* const eax,     //
                              uint32_t* const ebx,     //
                              uint32_t* const ecx,     //
                              uint32_t* const edx) {
    if (leaf == 0) {
      *eax = g_cpuid_max_leaf;
      *ebx = 0;
      *ecx = 0;
      *edx = 0;
    } else if (leaf == 1) {
      *eax = 0x000306A9u;
      *ebx = g_leaf1_ebx;
      *ecx = g_leaf1_ecx;
      *edx = 1u << 9;
    } else if (leaf == 0x0Bu && subleaf == 0) {
      *eax = 1u;
      *ebx = g_leafb_ebx;
      *ecx = g_leafb_ecx;
      *edx = g_leafb_edx;
    }
  };
  const auto read_msr_hook = +[](const uint32_t msr) -> uint64_t {
    if (msr == 0x1Bu) {
      return g_apic_base_msr;
    }
    if (msr == 0x802u) {
      return g_msr_lapic_id;
    }
    return 0;
  };
  const auto write_msr_hook = +[](const uint32_t msr, const uint64_t value) {
    g_msr_writes.emplace_back(msr, value);
  };

  SmpSetHostHardwareHooksForTest(true,           //
                                 cpuid_hook,     //
                                 read_msr_hook,  //
                                 write_msr_hook);
  EXPECT_THAT(SmpReadCpuidInitialApicIdForTest(), t::Eq(0x12345678u));

  // If leaf 0x0B is unpopulated (ebx[15:0] == 0 && ecx[15:8] == 0), falls back
  // to CPUID.01H:EBX[31:24] (0x78).
  g_leafb_ebx = 0;
  g_leafb_ecx = 0;
  EXPECT_THAT(SmpReadCpuidInitialApicIdForTest(), t::Eq(0x78u));

  // If max_leaf < 0x0B, falls back to CPUID.01H:EBX[31:24].
  g_leafb_ebx = 2u;
  g_leafb_ecx = 1u << 8;
  g_cpuid_max_leaf = 0x0Au;
  EXPECT_THAT(SmpReadCpuidInitialApicIdForTest(), t::Eq(0x78u));

  // If x2APIC is not supported in CPUID.01H:ECX[21] nor enabled in
  // IA32_APIC_BASE, falls back to CPUID.01H:EBX[31:24].
  g_cpuid_max_leaf = 0x0Bu;
  g_leaf1_ecx = 0;
  g_apic_base_msr = 0xFEE00900ULL;
  EXPECT_THAT(SmpReadCpuidInitialApicIdForTest(), t::Eq(0x78u));

  // If x2APIC is already enabled in IA32_APIC_BASE (bit 10 EXTD = 1), leaf 0x0B
  // is used and returns 0x100.
  g_apic_base_msr = 0xFEE00D00ULL;
  g_leafb_edx = 0x100u;
  EXPECT_THAT(SmpReadCpuidInitialApicIdForTest(), t::Eq(0x100u));

  // 3. In x2APIC mode, ReadLapicId, LapicWrite, and LapicSendIpiCommand do not
  // require non-zero lapic_base and preserve full 32-bit APIC IDs.
  g_msr_lapic_id = 0x12345678u;
  EXPECT_THAT(SmpReadLapicIdForTest(/*lapic_base=*/0), t::Eq(0x12345678u));

  g_msr_writes.clear();
  SmpLapicWriteForTest(/*lapic_base=*/0, /*reg_offset=*/0x0B0, /*value=*/0);
  ASSERT_THAT(g_msr_writes.size(), t::Eq(1u));
  EXPECT_THAT(g_msr_writes[0].first, t::Eq(0x80Bu));
  EXPECT_THAT(g_msr_writes[0].second, t::Eq(0u));

  g_msr_writes.clear();
  SmpLapicSendIpiCommandForTest(/*lapic_base=*/0,
                                /*apic_id=*/0x12345678u,
                                /*icr_low=*/0x00004021u);
  ASSERT_THAT(g_msr_writes.size(), t::Eq(2u));
  EXPECT_THAT(g_msr_writes[0].first, t::Eq(0x828u));
  EXPECT_THAT(g_msr_writes[0].second, t::Eq(0u));
  EXPECT_THAT(g_msr_writes[1].first, t::Eq(0x830u));
  EXPECT_THAT(g_msr_writes[1].second,
              t::Eq((0x12345678ULL << 32) | 0x00004021ULL));

  // In xAPIC MMIO mode, ReadLapicId shifts right by 24 bits and requires
  // non-zero lapic_base, and LapicSendIpiCommand waits for ICR delivery-pending
  // before and after programming ICR_HIGH/ICR_LOW while preserving the caller's
  // interrupt enable state.
  uint32_t fake_mmio_lapic[256] = {};
  fake_mmio_lapic[0x020 / sizeof(uint32_t)] = 0x42000000u;
  SmpSetHostHardwareHooksForTest(false,          //
                                 cpuid_hook,     //
                                 read_msr_hook,  //
                                 write_msr_hook);
  const uintptr_t fake_lapic_base =
      reinterpret_cast<uintptr_t>(fake_mmio_lapic);
  EXPECT_THAT(SmpReadLapicIdForTest(fake_lapic_base), t::Eq(0x42u));
  EXPECT_DEATH(SmpReadLapicIdForTest(0),
               "Check failed: g_x2apic_mode \\|\\| lapic_base != 0");

  const bool irq_before_ipi = AreInterruptsEnabled();
  SmpLapicSendIpiCommandForTest(fake_lapic_base, 0x42u, 0x00004021u);
  EXPECT_THAT(AreInterruptsEnabled(), t::Eq(irq_before_ipi));
  EXPECT_THAT(fake_mmio_lapic[0x310 / sizeof(uint32_t)], t::Eq(0x42u << 24));
  EXPECT_THAT(fake_mmio_lapic[0x300 / sizeof(uint32_t)], t::Eq(0x00004021u));

  // If ICR_LOW bit 12 (Delivery Status = Send Pending) is stuck prior to
  // sending an IPI in xAPIC mode, LapicSendIpiCommand must fail the pre-write
  // WaitForIcrIdle check before overwriting ICR_HIGH.
  fake_mmio_lapic[0x300 / sizeof(uint32_t)] = (1u << 12);
  EXPECT_DEATH(
      SmpLapicSendIpiCommandForTest(fake_lapic_base, 0x11u, 0x00004021u),
      "Check failed: WaitForIcrIdle\\(lapic_base\\)");
  fake_mmio_lapic[0x300 / sizeof(uint32_t)] = 0;

  // Verify EnableLocalApic steps down from x2APIC (EXTD=1, EN=1) to xAPIC
  // (EXTD=0, EN=1) via the Architecturally Disabled state (EXTD=0, EN=0) and
  // programs TPR=0 and SVR=(kLapicSvrSoftwareEnable | kLapicSpuriousVector).
  g_apic_base_msr = 0xFEE00D00ULL;
  fake_mmio_lapic[0x080 / sizeof(uint32_t)] = 0xFFu;
  fake_mmio_lapic[0x0F0 / sizeof(uint32_t)] = 0x00001000u;
  g_msr_writes.clear();
  SmpEnableLocalApicForTest(fake_lapic_base);
  ASSERT_THAT(g_msr_writes.size(), t::Eq(2u));
  EXPECT_THAT(g_msr_writes[0].first, t::Eq(0x1Bu));
  EXPECT_THAT(g_msr_writes[0].second, t::Eq(0xFEE00100ULL));
  EXPECT_THAT(g_msr_writes[1].first, t::Eq(0x1Bu));
  EXPECT_THAT(g_msr_writes[1].second, t::Eq(0xFEE00900ULL));
  EXPECT_THAT(fake_mmio_lapic[0x080 / sizeof(uint32_t)], t::Eq(0u));
  EXPECT_THAT(fake_mmio_lapic[0x0F0 / sizeof(uint32_t)], t::Eq(0x000011FFu));
  EXPECT_DEATH(SmpEnableLocalApicForTest(0),
               "Check failed: g_x2apic_mode \\|\\| lapic_base != 0");

  // Also verify EnableLocalApic in x2APIC mode enables EN (bit 11) then EXTD
  // (bit 10) and writes TPR (MSR 0x808) and SVR (MSR 0x80F).
  SmpSetHostHardwareHooksForTest(true,           //
                                 cpuid_hook,     //
                                 read_msr_hook,  //
                                 write_msr_hook);
  g_apic_base_msr = 0xFEE00100ULL;
  g_msr_writes.clear();
  SmpEnableLocalApicForTest(/*lapic_base=*/0);
  ASSERT_THAT(g_msr_writes.size(), t::Eq(4u));
  EXPECT_THAT(g_msr_writes[0].first, t::Eq(0x1Bu));
  EXPECT_THAT(g_msr_writes[0].second, t::Eq(0xFEE00900ULL));
  EXPECT_THAT(g_msr_writes[1].first, t::Eq(0x1Bu));
  EXPECT_THAT(g_msr_writes[1].second, t::Eq(0xFEE00D00ULL));
  EXPECT_THAT(g_msr_writes[2].first, t::Eq(0x808u));
  EXPECT_THAT(g_msr_writes[2].second, t::Eq(0u));
  EXPECT_THAT(g_msr_writes[3].first, t::Eq(0x80Fu));
  EXPECT_THAT(g_msr_writes[3].second, t::Eq(0x1FFu));
  g_apic_base_msr = 0xFEE00D00ULL;

  // 4. End-to-end SmpInit and SmpRunOnAllCpus in x2APIC mode with 32-bit APIC
  // IDs. SmpInit detects x2APIC through the hooks (IA32_APIC_BASE.EXTD is
  // already set), so it must run after SmpSetHostTestHooks(), which clears
  // them.
  WriteRsdt(rsdt_addr, {static_cast<uint32_t>(madt_addr)});
  WriteRsdpV1(bios_rom_base + 64, static_cast<uint32_t>(rsdt_addr));
  IdtInit();
  SmpSetHostTestHooks(0x100u, 0, bios_rom_base, BaseAddr(), nullptr);
  SmpSetHostHardwareHooksForTest(false,          //
                                 cpuid_hook,     //
                                 read_msr_hook,  //
                                 write_msr_hook);
  g_msr_lapic_id = 0x100u;
  g_msr_writes.clear();
  SmpInit(kMultiboot1Magic, 0);
  ASSERT_TRUE(SmpIsX2ApicEnabled());
  // SmpInit enabled the BSP's Local APIC through the x2APIC TPR and SVR MSRs.
  EXPECT_THAT(g_msr_writes,
              t::ElementsAre(t::Pair(0x808u, 0u), t::Pair(0x80Fu, 0x1FFu)));
  ASSERT_THAT(SmpCpuCount(), t::Eq(5));
  EXPECT_THAT(SmpGetCpuInfo(0)->apic_id, t::Eq(0x100u));
  EXPECT_THAT(SmpGetCpuLocal(0)->apic_id, t::Eq(0x100u));
  EXPECT_THAT(SmpGetCpuInfo(2)->apic_id, t::Eq(0x12345678u));
  EXPECT_THAT(SmpGetCpuLocal(2)->apic_id, t::Eq(0x12345678u));

  std::atomic<int> high_apic_seen{0};
  SmpRunOnAllCpus(
      [](const int cpu_index, void* const ctx) {
        if (SmpGetCpuLocal(cpu_index)->apic_id >= 0x100u) {
          static_cast<std::atomic<int>*>(ctx)->fetch_add(
              1, std::memory_order_acq_rel);
        }
      },
      &high_apic_seen);
  EXPECT_THAT(high_apic_seen.load(std::memory_order_acquire), t::Eq(4));
}

TEST(SmpTest, SmpInitIdentifiesTheBspByItsLocalApicIdRegister) {
  ResetState(BaseAddr());
  std::vector<uint8_t> recs;
  for (uint32_t id = 0; id < 4; ++id) {
    AppendX2ApicRecord(&recs, id, id, 1);
  }
  const uintptr_t bios_rom_base = WriteBiosAcpiTables(recs);
  IdtInit();
  // CPUID reports initial APIC ID 0, but the BSP's x2APIC ID register (MSR
  // 0x802) and the MADT agree on 2. Matching by the CPUID value would leave
  // the real BSP entry in the AP list, and SmpInit would send INIT to it.
  SmpSetHostTestHooks(0, 0, bios_rom_base, BaseAddr(), nullptr);
  g_cpu.x2apic_supported = true;
  g_cpu.x2apic_id_msr = 2;
  InstallFakeCpuHooks();
  SmpInit(kMultiboot1Magic, 0);

  ASSERT_TRUE(SmpIsX2ApicEnabled());
  ASSERT_THAT(SmpCpuCount(), t::Eq(4));
  constexpr uint32_t kExpectedApicIds[] = {2, 0, 1, 3};
  for (int i = 0; i < 4; ++i) {
    EXPECT_THAT(SmpGetCpuInfo(i)->apic_id, t::Eq(kExpectedApicIds[i]));
    EXPECT_THAT(SmpGetCpuLocal(i)->apic_id, t::Eq(kExpectedApicIds[i]));
    EXPECT_THAT(SmpGetCpuInfo(i)->is_bsp, t::Eq(i == 0));
  }
  EXPECT_THAT(SmpGetCpuInfo(0)->observed_apic_id, t::Eq(2u));
  EXPECT_THAT(g_state.uart_log,
              t::HasSubstr("[SMP] BSP Local APIC ID 2 differs from its CPUID "
                           "initial APIC ID 0; using the Local APIC ID\n"));
  EXPECT_THAT(g_state.uart_log, t::HasSubstr("[SMP] CPU 0 (APIC ID 2, BSP)"));
}

TEST(SmpTest, SmpInitSkipsApicIdsThatXapicModeCannotAddress) {
  ResetState(BaseAddr());
  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, 0, 1);
  AppendX2ApicRecord(&recs, 0xFFu, 1, 1);
  AppendLapicRecord(&recs, 2, 1, 1);
  AppendX2ApicRecord(&recs, 0x100u, 3, 1);
  AppendLapicRecord(&recs, 4, 2, 1);
  const uintptr_t bios_rom_base = WriteBiosAcpiTables(recs);
  IdtInit();
  SmpSetHostTestHooks(0, 0, bios_rom_base, BaseAddr(), nullptr);
  SmpInit(kMultiboot1Magic, 0);

  // ICR destinations are 8 bits wide in xAPIC mode, so APIC IDs >= 0xFF
  // cannot be started (0x100 would be truncated to the BSP's ID 0).
  ASSERT_FALSE(SmpIsX2ApicEnabled());
  ASSERT_THAT(SmpCpuCount(), t::Eq(3));
  EXPECT_THAT(SmpGetCpuInfo(1)->apic_id, t::Eq(1u));
  EXPECT_THAT(SmpGetCpuInfo(2)->apic_id, t::Eq(2u));
  EXPECT_THAT(g_state.pmm_alloc_calls, t::Eq(2));
  EXPECT_THAT(g_state.uart_log,
              t::HasSubstr("[SMP] Skipping APIC ID 255: not addressable in "
                           "xAPIC mode\n[SMP] Skipping APIC ID 256: not "
                           "addressable in xAPIC mode\n"));
  EXPECT_THAT(g_state.uart_log, t::HasSubstr("[SMP] Online CPUs: 3/3"));
}

TEST(SmpTest, StopOtherCpusForPanicSendsAnNmiToAllOtherCpus) {
  ResetState(BaseAddr());
  // Before SmpInit has enabled the Local APIC, there is nothing to stop.
  g_cpu.x2apic_supported = true;
  InstallFakeCpuHooks();
  SmpStopOtherCpusForPanic();
  EXPECT_THAT(g_cpu.msr_writes, t::IsEmpty());

  const uintptr_t bios_rom_base = WriteBiosAcpiTables(LapicRecords(4));
  IdtInit();
  SmpSetHostTestHooks(0, 0, bios_rom_base, BaseAddr(), nullptr);
  InstallFakeCpuHooks();
  SmpInit(kMultiboot1Magic, 0);
  ASSERT_TRUE(SmpIsX2ApicEnabled());
  ASSERT_THAT(SmpCpuCount(), t::Eq(4));

  g_cpu.msr_writes.clear();
  SmpStopOtherCpusForPanic();
  // One x2APIC ICR write (MSR 0x830): delivery mode NMI (0x400), level assert
  // (bit 14), destination shorthand "all excluding self" (bits 18-19).
  EXPECT_THAT(g_cpu.msr_writes, t::ElementsAre(t::Pair(0x830u, 0x000C4400ULL)));
}

}  // namespace
}  // namespace protos
