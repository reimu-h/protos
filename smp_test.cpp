#include "smp.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cpu.h"
#include "gdt.h"
#include "idt.h"
#include "multiboot.h"
#include "paging.h"
#include "pmm.h"
#include "smp_hardware.h"
#include "smp_hardware_fake.h"
#include "spinlock.h"
#include "steal_time.h"
#include "task.h"
#include "uart.h"
#include "vga.h"

namespace protos {
namespace {

namespace t = ::testing;

using Event = FakeSmpHardware::Event;
// A register or MSR write: (register offset or MSR, value).
using RegisterWrite = std::pair<uint32_t, uint64_t>;

// Fake physical memory: 8 MiB at physical address 0, enough for the BIOS ROM
// area that SmpInit() scans and for the AP stack blocks of kMaxSupportedCpus
// CPUs.
constexpr int64_t kFakeRamFrames = 2048;
constexpr int64_t kFakeRamBytes = kFakeRamFrames * kPageSize;
// The fake PMM hands out AP stack blocks from 1 MiB on.
constexpr int64_t kFirstPmmFrame = 256;
constexpr uintptr_t kBiosRomRsdpAddr = 0xE0040;

// ICR bits 0-31 that smp.cpp sends.
constexpr uint32_t kIcrXapicInitAssert = 0x0000C500;
constexpr uint32_t kIcrXapicInitDeassert = 0x00008500;
constexpr uint32_t kIcrX2ApicInit = 0x00004500;
constexpr uint32_t kIcrStartupAt0x8000 = 0x00004608;
constexpr uint32_t kIcrWakeupIpi = 0x00004021;
constexpr uint32_t kIcrTlbShootdownIpi = 0x000040F0;
// An NMI (delivery mode 100b, no vector).
constexpr uint32_t kIcrNmi = 0x00004400;
// Local APIC registers (xAPIC offsets).
constexpr uint32_t kLapicTpr = 0x080;
constexpr uint32_t kLapicEoi = 0x0B0;
constexpr uint32_t kLapicSvr = 0x0F0;
constexpr uint32_t kLapicEsr = 0x280;
constexpr uint32_t kLapicIcrLow = 0x300;
constexpr uint32_t kLapicIcrHigh = 0x310;
constexpr uint32_t kLapicLvtTimer = 0x320;
constexpr uint32_t kLapicTimerInitCount = 0x380;
constexpr uint32_t kLapicTimerDivide = 0x3E0;

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
  int64_t next_pmm_frame = kFirstPmmFrame;
  int64_t max_pmm_frame = kFakeRamFrames;
  int pmm_alloc_calls = 0;
  int fail_pmm_on_call = -1;
  // PmmAllocFrames() call in which another CPU "panics": the call runs
  // SmpStopOtherCpusForPanic() first.
  int panic_stop_on_pmm_call = -1;
  std::vector<std::pair<uintptr_t, int64_t>> freed_frames;
  std::string uart_log;
  std::string vga_log;
};

FakeHostState g_state;

// Scheduler work for the AP idle loop: TaskPollIdleCpu(cpu) runs one unit of
// g_idle_work[cpu] per call, invoking g_idle_work_hook(cpu) if set.
std::atomic<int> g_idle_work[kMaxSupportedCpus];
std::atomic<void (*)(int)> g_idle_work_hook{nullptr};
// TaskPollIdleCpu() calls on another CPU than their argument, or with
// interrupts enabled.
std::atomic<int> g_idle_poll_violations{0};

// The machine that smp.cpp runs on in the current test.
std::shared_ptr<FakeSmpHardware> g_hw;

// Host pointer to the fake physical memory at `phys_addr`.
template <typename T = uint8_t>
static T* At(const uintptr_t phys_addr) {
  return reinterpret_cast<T*>(g_fake_ram + phys_addr);
}

static FakeSmpHardware::Options DefaultOptions() {
  FakeSmpHardware::Options options;
  options.physical_memory = g_fake_ram;
  options.physical_memory_size = sizeof(g_fake_ram);
  options.bsp_stack = {0x70000, 0x74000};
  return options;
}

// Powers off the APs of the previous fake machine, resets the other fakes, and
// installs a new FakeSmpHardware with `options` and zeroed physical memory.
static FakeSmpHardware& ResetState(
    const FakeSmpHardware::Options& options = DefaultOptions()) {
  if (g_hw != nullptr) {
    g_hw->PowerOffAps();
  }
  g_state = FakeHostState{};
  for (std::atomic<int>& work : g_idle_work) {
    work.store(0);
  }
  g_idle_work_hook.store(nullptr);
  g_idle_poll_violations.store(0);
  std::memset(g_fake_ram, 0, sizeof(g_fake_ram));
  IdtResetForTest();
  GdtResetForTest();
  CpuResetForTest();
  g_hw = std::make_shared<FakeSmpHardware>(options);
  SmpSetHardware(g_hw);
  return *g_hw;
}

class SmpTest : public t::Test {
 protected:
  void TearDown() override {
    if (g_hw != nullptr) {
      g_hw->PowerOffAps();
    }
  }
};

// Waits up to 10 s of host time for `condition()` to hold.
template <typename Condition>
static bool WaitUntil(const Condition& condition) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!condition()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::yield();
  }
  return true;
}

// The IPIs sent to `apic_id`, as ICR bits 0-31.
static std::vector<uint32_t> IcrsTo(const FakeSmpHardware& hw,
                                    const uint32_t apic_id) {
  std::vector<uint32_t> icrs;
  for (const Event& ipi : hw.Ipis()) {
    if ((ipi.value >> 32) == apic_id) {
      icrs.push_back(static_cast<uint32_t>(ipi.value));
    }
  }
  return icrs;
}

// The IPIs sent to `apic_id`, as events.
static std::vector<Event> IpisTo(const FakeSmpHardware& hw,
                                 const uint32_t apic_id) {
  std::vector<Event> ipis;
  for (const Event& ipi : hw.Ipis()) {
    if ((ipi.value >> 32) == apic_id) {
      ipis.push_back(ipi);
    }
  }
  return ipis;
}

// The Local APIC register writes of the CPU with `apic_id`, as (register
// offset, value) pairs.
static std::vector<RegisterWrite> RegisterWrites(const FakeSmpHardware& hw,
                                                 const uint32_t apic_id) {
  std::vector<RegisterWrite> writes;
  for (const Event& write : hw.LapicWrites(apic_id)) {
    writes.emplace_back(write.reg, write.value);
  }
  return writes;
}

// The MSR writes of the CPU with `apic_id` other than to x2APIC registers, as
// (MSR, value) pairs.
static std::vector<RegisterWrite> MsrWrites(const FakeSmpHardware& hw,
                                            const uint32_t apic_id) {
  std::vector<RegisterWrite> writes;
  for (const Event& write : hw.MsrWrites(apic_id)) {
    writes.emplace_back(write.reg, write.value);
  }
  return writes;
}

// Index of the first event of `kind` in `events`, or -1.
static int IndexOf(const std::vector<Event>& events, const Event::Kind kind) {
  for (int i = 0; i < static_cast<int>(events.size()); ++i) {
    if (events[i].kind == kind) {
      return i;
    }
  }
  return -1;
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
  RawRsdpV1* const rsdp = At<RawRsdpV1>(addr);
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
  RawRsdpV2* const rsdp = At<RawRsdpV2>(addr);
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
  RawSdtHeader* const sdt = At<RawSdtHeader>(addr);
  const int64_t total_len =
      sizeof(RawSdtHeader) + entries.size() * sizeof(uint32_t);
  std::memset(sdt, 0, total_len);
  uint8_t* const payload = At(addr) + sizeof(RawSdtHeader);
  for (int i = 0; i < static_cast<int>(entries.size()); ++i) {
    const uint32_t val = entries[i];
    std::memcpy(payload + i * sizeof(uint32_t), &val, sizeof(uint32_t));
  }
  FinalizeSdt(sdt, "RSDT", total_len);
}

static void WriteXsdt(const uintptr_t addr,
                      const std::vector<uint64_t>& entries) {
  RawSdtHeader* const sdt = At<RawSdtHeader>(addr);
  const int64_t total_len =
      sizeof(RawSdtHeader) + entries.size() * sizeof(uint64_t);
  std::memset(sdt, 0, total_len);
  uint8_t* const payload = At(addr) + sizeof(RawSdtHeader);
  for (int i = 0; i < static_cast<int>(entries.size()); ++i) {
    const uint64_t val = entries[i];
    std::memcpy(payload + i * sizeof(uint64_t), &val, sizeof(uint64_t));
  }
  FinalizeSdt(sdt, "XSDT", total_len);
}

static void WriteMadt(const uintptr_t addr,            //
                      const uint32_t local_apic_addr,  //
                      const std::vector<uint8_t>& records) {
  RawMadtHeader* const madt = At<RawMadtHeader>(addr);
  const int64_t total_len = sizeof(RawMadtHeader) + records.size();
  std::memset(madt, 0, total_len);
  madt->local_apic_address = local_apic_addr;
  madt->flags = 1;
  if (!records.empty()) {
    std::memcpy(At(addr) + sizeof(RawMadtHeader),  //
                records.data(),                    //
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
// the BIOS ROM area that SmpInit() scans under Multiboot1.
static void WriteBiosAcpiTables(const std::vector<uint8_t>& records) {
  constexpr uintptr_t kRsdtAddr = 0x6100;
  constexpr uintptr_t kMadtAddr = 0x6400;
  WriteMadt(kMadtAddr, kDefaultLocalApicPhysAddr, records);
  WriteRsdt(kRsdtAddr, {static_cast<uint32_t>(kMadtAddr)});
  WriteRsdpV1(kBiosRomRsdpAddr, static_cast<uint32_t>(kRsdtAddr));
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

// Returns MADT records for CPUs with x2APIC IDs 0 .. count - 1.
static std::vector<uint8_t> X2ApicRecords(const int count) {
  std::vector<uint8_t> records;
  for (int i = 0; i < count; ++i) {
    AppendX2ApicRecord(&records,                  //
                       static_cast<uint32_t>(i),  //
                       static_cast<uint32_t>(i),  //
                       1);
  }
  return records;
}

// Installs a fake machine with `options` whose MADT lists `records`, and runs
// SmpInit() on it under Multiboot1.
static FakeSmpHardware& InitSmp(const std::vector<uint8_t>& records,
                                const FakeSmpHardware::Options& options) {
  FakeSmpHardware& hw = ResetState(options);
  WriteBiosAcpiTables(records);
  IdtInit();
  SmpInit(kMultiboot1Magic, 0);
  return hw;
}

static FakeSmpHardware& InitSmp(const std::vector<uint8_t>& records) {
  return InitSmp(records, DefaultOptions());
}

static FakeSmpHardware::Options X2ApicOptions() {
  FakeSmpHardware::Options options = DefaultOptions();
  options.x2apic_supported = true;
  return options;
}

// Top of the stack of IST slot `slot` in `tss`.
static uint64_t IstStackTop(const TaskStateSegment* const tss, const int slot) {
  return tss->ist[slot - 1];
}

// The guard pages of the AP stack blocks at `blocks`, in ascending order: the
// one below the kernel stack at the start of each block, and one below each IST
// stack.
static std::vector<uintptr_t> GuardPagesOfBlocks(
    const std::vector<uintptr_t>& blocks) {
  std::vector<uintptr_t> guard_pages;
  for (const uintptr_t block : blocks) {
    guard_pages.push_back(block);
    uintptr_t next = block + kPageSize + kApStackSize;
    for (int i = 0; i < kIstStackCount; ++i) {
      guard_pages.push_back(next);
      next += kPageSize + kIstStackSize;
    }
  }
  std::sort(guard_pages.begin(), guard_pages.end());
  return guard_pages;
}

}  // namespace

uintptr_t PmmAllocFrames(const int64_t count) {
  ++g_state.pmm_alloc_calls;
  if (g_state.panic_stop_on_pmm_call == g_state.pmm_alloc_calls) {
    SmpStopOtherCpusForPanic();
  }
  if (g_state.fail_pmm_on_call == g_state.pmm_alloc_calls) {
    return 0;
  }
  if (count <= 0 || g_state.next_pmm_frame + count > g_state.max_pmm_frame) {
    return 0;
  }
  const uintptr_t addr = g_state.next_pmm_frame * kPageSize;
  g_state.next_pmm_frame += count;
  return addr;
}

void PmmFreeFrames(const uintptr_t base_addr, const int64_t count) {
  g_state.freed_frames.emplace_back(base_addr, count);
}

bool TaskPollIdleCpu(const int cpu_index) {
  if (CurrentCpuId() != cpu_index || AreInterruptsEnabled()) {
    g_idle_poll_violations.fetch_add(1);
  }
  std::atomic<int>& work = g_idle_work[cpu_index];
  int pending = work.load();
  while (pending > 0) {
    if (work.compare_exchange_weak(pending, pending - 1)) {
      void (*const hook)(int) = g_idle_work_hook.load();
      if (hook != nullptr) {
        hook(cpu_index);
      }
      return true;
    }
  }
  return false;
}

// multiboot.cpp's memory map parser maps memory through this. smp.cpp reads
// the Multiboot2 tags through SmpHardware instead, so no test calls it.
bool PagingMapBootstrapRange(const uintptr_t /*phys_addr*/,
                             const int64_t /*size*/) {
  ADD_FAILURE() << "unexpected PagingMapBootstrapRange() call";
  return false;
}

void UartWrite(const char* const str) { g_state.uart_log.append(str); }

void UartWriteDec(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "%llu",       //
                static_cast<unsigned long long>(value));
  g_state.uart_log.append(buf);
}

void VgaWrite(const char* const str) { g_state.vga_log.append(str); }

void UartPanicWrite(const char* const str) { UartWrite(str); }

void VgaPanicWrite(const char* const str) { VgaWrite(str); }

namespace {

TEST_F(SmpTest, ParseRsdpValidatesV1AndV2AndRejectsMalformed) {
  ResetState();
  constexpr uintptr_t kV1Addr = 0x100;
  constexpr uintptr_t kV2Addr = 0x200;
  WriteRsdpV1(kV1Addr, 0x12345000);
  WriteRsdpV2(kV2Addr, 0x12345000, 0x9876543200ULL);

  StatusOr<RsdpTables> tables = SmpParseRsdpForTest(kV1Addr, kIdentityMapLimit);
  ASSERT_TRUE(tables.ok());
  EXPECT_THAT(tables->rsdt_addr, t::Eq(0x12345000u));
  EXPECT_THAT(tables->xsdt_addr, t::Eq(0u));

  tables = SmpParseRsdpForTest(kV2Addr, kIdentityMapLimit);
  ASSERT_TRUE(tables.ok());
  EXPECT_THAT(tables->rsdt_addr, t::Eq(0x12345000u));
  EXPECT_THAT(tables->xsdt_addr, t::Eq(0x9876543200ULL));

  // Verify that an extended RSDP v2 with length > 36 (e.g. 40 bytes with
  // non-zero trailing vendor bytes and a valid 40-byte extended checksum) is
  // accepted per ACPI 2.0+ specification.
  RawRsdpV2* const raw_v2 = At<RawRsdpV2>(kV2Addr);
  raw_v2->length = 40;
  At(kV2Addr)[36] = 0x37;
  At(kV2Addr)[37] = 0x13;
  raw_v2->extended_checksum = 0;
  raw_v2->extended_checksum = ComputeChecksumByte(raw_v2, 40);
  tables = SmpParseRsdpForTest(kV2Addr, kIdentityMapLimit);
  ASSERT_TRUE(tables.ok());
  EXPECT_THAT(tables->xsdt_addr, t::Eq(0x9876543200ULL));
  WriteRsdpV2(kV2Addr, 0x12345000, 0x9876543200ULL);

  // Out-of-bounds physical limit must fail cleanly.
  EXPECT_FALSE(SmpParseRsdpForTest(kV1Addr, kV1Addr + 10).ok());

  // Corrupted v2 extended checksum must be rejected.
  raw_v2->extended_checksum ^= 0xFF;
  EXPECT_FALSE(SmpParseRsdpForTest(kV2Addr, kIdentityMapLimit).ok());
  raw_v2->extended_checksum ^= 0xFF;

  // Invalid v2 length (< 36) must be rejected.
  raw_v2->length = 20;
  raw_v2->extended_checksum = ComputeChecksumByte(raw_v2, sizeof(RawRsdpV2));
  EXPECT_FALSE(SmpParseRsdpForTest(kV2Addr, kIdentityMapLimit).ok());

  // Corrupted v1 checksum or signature must be rejected.
  At<RawRsdpV1>(kV1Addr)->checksum ^= 0x01;
  EXPECT_FALSE(SmpParseRsdpForTest(kV1Addr, kIdentityMapLimit).ok());

  WriteRsdpV1(kV1Addr, 0);
  EXPECT_FALSE(SmpParseRsdpForTest(kV1Addr, kIdentityMapLimit).ok());
}

TEST_F(SmpTest, FindRsdpInMemoryRangeHonorsAlignmentAndChecksums) {
  ResetState();
  constexpr uintptr_t kScanBase = 0x1000;

  // Place a misaligned RSDP at +8 (must be skipped) and a corrupted RSDP at
  // +32 (must be skipped), followed by a valid RSDP at +64.
  WriteRsdpV1(kScanBase + 8, 0x11110000);
  WriteRsdpV1(kScanBase + 32, 0x22220000);
  At<RawRsdpV1>(kScanBase + 32)->checksum ^= 0x55;
  WriteRsdpV1(kScanBase + 64, 0x33330000);

  EXPECT_THAT(SmpFindRsdpInMemoryRange(kScanBase, 1024), t::Eq(kScanBase + 64));
  EXPECT_THAT(SmpFindRsdpInMemoryRange(kScanBase, 64), t::Eq(0u));
  EXPECT_THAT(SmpFindRsdpInMemoryRange(kScanBase, 16), t::Eq(0u));

  // Verify that a 36-byte v2 RSDP placed at +64 is rejected if the scan range
  // ends at +64 + 24 (truncating the v2 structure before 36 bytes), or if its
  // v2 extended checksum is corrupted even when its v1 checksum is valid.
  WriteRsdpV2(kScanBase + 64, 0x33330000, 0x44440000ULL);
  EXPECT_THAT(SmpFindRsdpInMemoryRange(kScanBase + 64, 24), t::Eq(0u));
  EXPECT_THAT(SmpFindRsdpInMemoryRange(kScanBase + 64, 36),
              t::Eq(kScanBase + 64));
  At<RawRsdpV2>(kScanBase + 64)->extended_checksum ^= 0xFF;
  EXPECT_THAT(SmpFindRsdpInMemoryRange(kScanBase + 64, 36), t::Eq(0u));

  // A range beyond the physical memory that can be mapped holds no RSDP.
  EXPECT_THAT(SmpFindRsdpInMemoryRange(kFakeRamBytes - 512, 1024), t::Eq(0u));
}

TEST_F(SmpTest, FindRsdpInMultiboot2PrefersAcpiNewAndFallsBackToOld) {
  ResetState();
  constexpr uintptr_t kMbBase = 0x2000;
  uint8_t* const buf = At(kMbBase);

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
  const uintptr_t old_rsdp_addr = kMbBase + pos + 8;
  *reinterpret_cast<uint32_t*>(buf + pos) = kMultiboot2TagAcpiOld;
  *reinterpret_cast<uint32_t*>(buf + pos + 4) = 8 + sizeof(RawRsdpV1);
  WriteRsdpV1(old_rsdp_addr, 0xAAAA0000);
  pos += 32;

  // Tag 15 (ACPI_NEW) at offset 56
  const uintptr_t new_rsdp_addr = kMbBase + pos + 8;
  *reinterpret_cast<uint32_t*>(buf + pos) = kMultiboot2TagAcpiNew;
  *reinterpret_cast<uint32_t*>(buf + pos + 4) = 8 + sizeof(RawRsdpV2);
  WriteRsdpV2(new_rsdp_addr, 0xAAAA0000, 0xBBBB0000ULL);
  pos += 48;

  // End tag
  *reinterpret_cast<uint32_t*>(buf + pos) = kMultiboot2TagEnd;
  *reinterpret_cast<uint32_t*>(buf + pos + 4) = 8;
  pos += 8;

  *reinterpret_cast<uint32_t*>(buf) = static_cast<uint32_t>(pos);
  *reinterpret_cast<uint32_t*>(buf + 4) = 0;

  // When both Tag 14 and Tag 15 are valid, Tag 15 (ACPI 2.0+) is preferred.
  EXPECT_THAT(SmpFindRsdpInMultiboot2ForTest(kMbBase, kIdentityMapLimit),
              t::Eq(new_rsdp_addr));

  // If Tag 15 claims an RSDP v2 length (e.g. 512) larger than the Multiboot2
  // tag payload size (36), it must be rejected without reading out-of-bounds,
  // falling back to Tag 14.
  At<RawRsdpV2>(new_rsdp_addr)->length = 512;
  EXPECT_THAT(SmpFindRsdpInMultiboot2ForTest(kMbBase, kIdentityMapLimit),
              t::Eq(old_rsdp_addr));
  WriteRsdpV2(new_rsdp_addr, 0xAAAA0000, 0xBBBB0000ULL);

  // If Tag 15 has a corrupted extended checksum, fallback to Tag 14 succeeds.
  At<RawRsdpV2>(new_rsdp_addr)->extended_checksum ^= 0xFF;
  EXPECT_THAT(SmpFindRsdpInMultiboot2ForTest(kMbBase, kIdentityMapLimit),
              t::Eq(old_rsdp_addr));

  // If Tag 14 is also corrupted, but Tag 15's v1 header (rsdt_address != 0) is
  // still valid, Tag 15's v1 header is used as fallback.
  At<RawRsdpV1>(old_rsdp_addr)->checksum ^= 0xFF;
  EXPECT_THAT(SmpFindRsdpInMultiboot2ForTest(kMbBase, kIdentityMapLimit),
              t::Eq(new_rsdp_addr));

  // If Tag 15's v1 checksum is also corrupted, returns 0.
  At<RawRsdpV2>(new_rsdp_addr)->v1.checksum ^= 0xFF;
  EXPECT_THAT(SmpFindRsdpInMultiboot2ForTest(kMbBase, kIdentityMapLimit),
              t::Eq(0u));

  // Malformed Multiboot2 total_size (< 8 or > 4 MiB) must return 0.
  *reinterpret_cast<uint32_t*>(buf) = 4;
  EXPECT_THAT(SmpFindRsdpInMultiboot2ForTest(kMbBase, kIdentityMapLimit),
              t::Eq(0u));
  *reinterpret_cast<uint32_t*>(buf) = 8 * 1024 * 1024;
  EXPECT_THAT(SmpFindRsdpInMultiboot2ForTest(kMbBase, kIdentityMapLimit),
              t::Eq(0u));
}

TEST_F(SmpTest, FindMadtInSdtSearchesXsdtAndFallsBackToRsdt) {
  ResetState();
  constexpr uintptr_t kFacpAddr = 0x3000;
  constexpr uintptr_t kBadMadtAddr = 0x3200;
  constexpr uintptr_t kGoodMadtAddr = 0x3400;
  constexpr uintptr_t kRsdtAddr = 0x3600;
  constexpr uintptr_t kXsdtAddr = 0x3800;

  // Non-MADT table ("FACP").
  FinalizeSdt(At<RawSdtHeader>(kFacpAddr), "FACP", sizeof(RawSdtHeader));

  // Corrupted MADT ("APIC" with bad checksum).
  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, 0, 1);
  WriteMadt(kBadMadtAddr, kDefaultLocalApicPhysAddr, recs);
  At<RawSdtHeader>(kBadMadtAddr)->checksum ^= 0x42;

  // Valid MADT ("APIC").
  WriteMadt(kGoodMadtAddr, kDefaultLocalApicPhysAddr, recs);

  WriteXsdt(kXsdtAddr, {kFacpAddr, kBadMadtAddr, kGoodMadtAddr});
  WriteRsdt(kRsdtAddr, {kFacpAddr, kBadMadtAddr, kGoodMadtAddr});

  // Valid XSDT resolves kGoodMadtAddr (skipping FACP and kBadMadtAddr).
  EXPECT_THAT(SmpFindMadtInSdt(0, kXsdtAddr, kIdentityMapLimit),
              t::Eq(kGoodMadtAddr));

  // If XSDT checksum is corrupted, falls back to 32-bit RSDT and resolves
  // kGoodMadtAddr.
  At<RawSdtHeader>(kXsdtAddr)->checksum ^= 0xFF;
  EXPECT_THAT(SmpFindMadtInSdt(kRsdtAddr, kXsdtAddr, kIdentityMapLimit),
              t::Eq(kGoodMadtAddr));

  // If RSDT checksum is also corrupted, returns 0.
  At<RawSdtHeader>(kRsdtAddr)->checksum ^= 0xFF;
  EXPECT_THAT(SmpFindMadtInSdt(kRsdtAddr, kXsdtAddr, kIdentityMapLimit),
              t::Eq(0u));
}

TEST_F(SmpTest, ParseMadtHandlesBspReorderingFilteringDuplicatesAndOverrides) {
  ResetState();
  constexpr uintptr_t kMadtAddr = 0x4000;

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
  WriteMadt(kMadtAddr, 0xFEE00123u, recs);

  StatusOr<SmpTopology> parsed = SmpParseMadt(kMadtAddr, kIdentityMapLimit, 4);
  ASSERT_TRUE(parsed.ok());
  SmpTopology& topo = *parsed;
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
  parsed = SmpParseMadt(kMadtAddr, kIdentityMapLimit, 99);
  ASSERT_TRUE(parsed.ok());
  ASSERT_THAT(topo.cpu_count, t::Eq(4));
  EXPECT_THAT(topo.cpus[0].apic_id, t::Eq(99u));
  EXPECT_THAT(topo.cpus[0].acpi_processor_id, t::Eq(0u));
  EXPECT_TRUE(topo.cpus[0].is_bsp);
  EXPECT_THAT(topo.cpus[1].apic_id, t::Eq(2u));
  EXPECT_THAT(topo.cpus[2].apic_id, t::Eq(4u));
  EXPECT_THAT(topo.cpus[3].apic_id, t::Eq(7u));
}

TEST_F(SmpTest, ParseMadtIgnoresProcessorsThatAreOnlyOnlineCapable) {
  ResetState();
  constexpr uintptr_t kMadtAddr = 0x4000;

  // Online Capable (flags bit 1) without Enabled (bit 0) marks a hot-pluggable
  // processor that is not present. Enabled wins if both bits are set.
  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, 0, 1);
  AppendLapicRecord(&recs, 1, 1, 2);
  AppendLapicRecord(&recs, 2, 2, 1);
  AppendX2ApicRecord(&recs, 3, 3, 2);
  AppendLapicRecord(&recs, 4, 4, 3);
  WriteMadt(kMadtAddr, kDefaultLocalApicPhysAddr, recs);

  StatusOr<SmpTopology> parsed = SmpParseMadt(kMadtAddr, kIdentityMapLimit, 0);
  ASSERT_TRUE(parsed.ok());
  SmpTopology& topo = *parsed;
  ASSERT_THAT(topo.cpu_count, t::Eq(3));
  EXPECT_THAT(topo.cpus[0].apic_id, t::Eq(0u));
  EXPECT_THAT(topo.cpus[1].apic_id, t::Eq(2u));
  EXPECT_THAT(topo.cpus[2].apic_id, t::Eq(4u));

  // A BSP that is listed only as Online Capable is still placed at index 0.
  parsed = SmpParseMadt(kMadtAddr, kIdentityMapLimit, 1);
  ASSERT_TRUE(parsed.ok());
  ASSERT_THAT(topo.cpu_count, t::Eq(4));
  EXPECT_THAT(topo.cpus[0].apic_id, t::Eq(1u));
  EXPECT_THAT(topo.cpus[0].acpi_processor_id, t::Eq(0u));

  // A table whose processors are all only Online Capable lists no CPU.
  recs.clear();
  AppendLapicRecord(&recs, 0, 0, 2);
  AppendX2ApicRecord(&recs, 1, 1, 2);
  WriteMadt(kMadtAddr, kDefaultLocalApicPhysAddr, recs);
  EXPECT_FALSE(SmpParseMadt(kMadtAddr, kIdentityMapLimit, 0).ok());
}

TEST_F(SmpTest, ParseMadtHandlesMalformedEntriesAndCapsCpuCount) {
  ResetState();
  constexpr uintptr_t kMadtAddr = 0x5000;

  // Entry with length == 0 after two valid CPUs must terminate parsing cleanly
  // without an infinite loop.
  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, 0, 1);
  AppendLapicRecord(&recs, 1, 1, 1);
  const uint8_t zero_len_entry[2] = {0, 0};
  recs.insert(recs.end(), zero_len_entry, zero_len_entry + 2);
  AppendLapicRecord(&recs, 2, 2, 1);
  WriteMadt(kMadtAddr, kDefaultLocalApicPhysAddr, recs);

  StatusOr<SmpTopology> parsed = SmpParseMadt(kMadtAddr, kIdentityMapLimit, 0);
  ASSERT_TRUE(parsed.ok());
  SmpTopology& topo = *parsed;
  EXPECT_THAT(topo.cpu_count, t::Eq(2));

  // Truncated Type 0 entry (length == 4 < 8) stops parsing cleanly.
  recs.clear();
  AppendLapicRecord(&recs, 0, 0, 1);
  const uint8_t short_type0[4] = {0, 4, 1, 1};
  recs.insert(recs.end(), short_type0, short_type0 + sizeof(short_type0));
  AppendLapicRecord(&recs, 2, 2, 1);
  WriteMadt(kMadtAddr, kDefaultLocalApicPhysAddr, recs);
  parsed = SmpParseMadt(kMadtAddr, kIdentityMapLimit, 0);
  ASSERT_TRUE(parsed.ok());
  EXPECT_THAT(topo.cpu_count, t::Eq(1));

  // MADT with only disabled CPUs must fail.
  recs.clear();
  AppendLapicRecord(&recs, 0, 0, 0);
  AppendLapicRecord(&recs, 1, 1, 0);
  WriteMadt(kMadtAddr, kDefaultLocalApicPhysAddr, recs);
  EXPECT_FALSE(SmpParseMadt(kMadtAddr, kIdentityMapLimit, 0).ok());

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
  WriteMadt(kMadtAddr, kDefaultLocalApicPhysAddr, recs);
  parsed = SmpParseMadt(kMadtAddr, kIdentityMapLimit, 0);
  ASSERT_TRUE(parsed.ok());
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
  WriteMadt(kMadtAddr, kDefaultLocalApicPhysAddr,
            LapicRecords(kMaxSupportedCpus));
  parsed = SmpParseMadt(kMadtAddr, kIdentityMapLimit, 0);
  ASSERT_TRUE(parsed.ok());
  EXPECT_THAT(topo.cpu_count, t::Eq(kMaxSupportedCpus));
  EXPECT_THAT(topo.ignored_cpu_count, t::Eq(0));
}

TEST_F(SmpTest, ParseMadtKeeps32BitX2ApicIdsAndProcessorUids) {
  ResetState();
  constexpr uintptr_t kMadtAddr = 0x6400;

  // MADT Type 9 (Local x2APIC) entries with 32-bit APIC IDs >= 0xFF (0xFF,
  // 0x100, 0x12345678, 0xFFFFFFFE) and 32-bit ACPI processor UIDs are preserved
  // without 8-bit truncation, while 0xFFFFFFFF (kInvalidX2ApicId) is rejected
  // and Type 0 0xFF (kInvalidXapicId) is rejected.
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
  WriteMadt(kMadtAddr, kDefaultLocalApicPhysAddr, recs);

  const StatusOr<SmpTopology> parsed =
      SmpParseMadt(kMadtAddr, kIdentityMapLimit, 0x12345678u);
  ASSERT_TRUE(parsed.ok());
  const SmpTopology& topo = *parsed;
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
}

TEST_F(SmpTest, CpuidInitialApicIdComesFromLeaf0BWhenX2ApicIsAvailable) {
  // The BSP's CPUID initial APIC ID is 0x12345678: leaf 0x01 reports its low 8
  // bits (0x78), leaf 0x0B all 32.
  struct Case {
    const char* name;
    bool x2apic_supported;
    uint64_t apic_base_msr;
    uint32_t max_leaf;
    bool leaf_0b;
    uint32_t expected;
  };
  const Case kCases[] = {
      {"x2APIC supported", true, 0xFEE00900, 0x0B, true, 0x12345678u},
      {"leaf 0x0B not populated", true, 0xFEE00900, 0x0B, false, 0x78u},
      {"no leaf 0x0B", true, 0xFEE00900, 0x0A, true, 0x78u},
      {"no x2APIC", false, 0xFEE00900, 0x0B, true, 0x78u},
      // IA32_APIC_BASE.EXTD: the firmware already enabled x2APIC mode.
      {"x2APIC enabled", false, 0xFEE00D00, 0x0B, true, 0x12345678u},
  };
  for (const Case& c : kCases) {
    SCOPED_TRACE(c.name);
    FakeSmpHardware::Options options = DefaultOptions();
    options.bsp_cpuid_apic_id = 0x12345678u;
    options.x2apic_supported = c.x2apic_supported;
    options.bsp_apic_base_msr = c.apic_base_msr;
    options.cpuid_max_leaf = c.max_leaf;
    options.cpuid_leaf_0b = c.leaf_0b;
    ResetState(options);
    EXPECT_THAT(SmpReadCpuidInitialApicIdForTest(), t::Eq(c.expected));
  }
}

TEST_F(SmpTest, ApsStartWithInitSipiAndTheSdmDelaysThroughTheTrampoline) {
  FakeSmpHardware& hw = ResetState();
  WriteBiosAcpiTables(LapicRecords(3));
  // What the trampoline overwrites at 0x8000 must be restored afterwards.
  std::memset(At(kApTrampolinePhysAddr), 0x5A, 64);
  IdtInit();
  SmpInit(kMultiboot1Magic, 0);
  ASSERT_THAT(SmpOnlineCpuCount(), t::Eq(3));

  for (uint32_t apic_id = 1; apic_id <= 2; ++apic_id) {
    SCOPED_TRACE(apic_id);
    // INIT assert and de-assert, then a SIPI with vector 0x08 (start at
    // 0x8000). The AP entered the trampoline at once, so the BSP skipped the
    // second SIPI.
    EXPECT_THAT(IcrsTo(hw, apic_id),
                t::ElementsAre(kIcrXapicInitAssert, kIcrXapicInitDeassert,
                               kIcrStartupAt0x8000));
    const std::vector<Event> ipis = IpisTo(hw, apic_id);
    ASSERT_THAT(ipis.size(), t::Eq(3u));
    EXPECT_THAT(ipis[0].cpu, t::Eq(0u));
    // The SDM's 10 ms between INIT and SIPI.
    EXPECT_GE(ipis[2].time_ns - ipis[1].time_ns, 10000000u);
  }
  // The APs were started one by one while the trampoline was installed.
  const std::vector<Event> events = hw.Events();
  const int installed = IndexOf(events, Event::Kind::kTrampolineInstalled);
  const int restored = IndexOf(events, Event::Kind::kTrampolineRestored);
  ASSERT_GE(installed, 0);
  ASSERT_GT(restored, installed);
  for (int i = 0; i < static_cast<int>(events.size()); ++i) {
    if (events[i].kind == Event::Kind::kIpi) {
      EXPECT_GT(i, installed);
      EXPECT_LT(i, restored);
    }
  }
  EXPECT_THAT(std::vector<uint8_t>(At(kApTrampolinePhysAddr),
                                   At(kApTrampolinePhysAddr) + 64),
              t::Each(0x5A));

  // Each AP ran ApKernelEntry() on the stack and with the CpuLocal that the BSP
  // handed to it, loaded the IDT (IdtInit() loaded it on the BSP), and applied
  // the BSP's CPU configuration.
  for (int i = 1; i < 3; ++i) {
    const CpuInfo* const info = SmpGetCpuInfo(i);
    EXPECT_GT(info->observed_rsp, info->stack_base);
    EXPECT_LE(info->observed_rsp, info->stack_top);
    EXPECT_THAT(info->observed_apic_id, t::Eq(info->apic_id));
    EXPECT_THAT(info->observed_cr3, t::Eq(SmpGetCpuInfo(0)->observed_cr3));
    EXPECT_TRUE(info->long_mode_active);
    EXPECT_TRUE(info->online);
    EXPECT_TRUE(SmpGetCpuLocal(i)->online);
  }
  EXPECT_THAT(IdtGetHostLoadCountForTest(), t::Eq(3));
  EXPECT_THAT(CpuGetHostApInitCountForTest(), t::Eq(2));
}

TEST_F(SmpTest, EveryApGetsStacksAboveGuardPagesAndLoadsItsTss) {
  const FakeSmpHardware& hw = InitSmp(LapicRecords(3));
  ASSERT_THAT(SmpOnlineCpuCount(), t::Eq(3));

  // Each AP's stacks are one block from the PMM: the kernel stack, then the
  // #DF, NMI and #MC stacks, each above a guard page.
  constexpr uintptr_t kIstStride = kPageSize + kIstStackSize;
  std::vector<uintptr_t> blocks;
  for (int i = 1; i < 3; ++i) {
    SCOPED_TRACE(i);
    const uintptr_t block = static_cast<uintptr_t>(
        (kFirstPmmFrame + (i - 1) * kApStackBlockFrames) * kPageSize);
    blocks.push_back(block);
    const CpuInfo* const info = SmpGetCpuInfo(i);
    EXPECT_THAT(info->stack_base, t::Eq(block + kPageSize));
    EXPECT_THAT(info->stack_top, t::Eq(info->stack_base + kApStackSize));
    // The AP loaded its TSS, which points at the IST stacks of its block.
    const TaskStateSegment* const tss = GdtGetTss(i);
    EXPECT_THAT(IstStackTop(tss, kIstDoubleFault),
                t::Eq(info->stack_top + kIstStride));
    EXPECT_THAT(IstStackTop(tss, kIstNmi),
                t::Eq(info->stack_top + 2 * kIstStride));
    EXPECT_THAT(IstStackTop(tss, kIstMachineCheck),
                t::Eq(info->stack_top + 3 * kIstStride));
    EXPECT_THAT(IstStackTop(tss, kIstMachineCheck),
                t::Eq(block + kApStackBlockFrames * kPageSize));
  }
  EXPECT_THAT(hw.GuardPages(), t::ElementsAreArray(GuardPagesOfBlocks(blocks)));
  EXPECT_THAT(GdtGetHostLoadCountForTest(), t::Eq(2));

  // The BSP unmapped every guard page before the first AP could start, because
  // only its own TLB is flushed.
  const std::vector<Event> events = hw.Events();
  const int installed = IndexOf(events, Event::Kind::kTrampolineInstalled);
  ASSERT_GE(installed, 0);
  int unmapped = 0;
  for (int i = 0; i < static_cast<int>(events.size()); ++i) {
    if (events[i].kind == Event::Kind::kGuardPageUnmapped) {
      EXPECT_LT(i, installed);
      EXPECT_THAT(events[i].cpu, t::Eq(0u));
      ++unmapped;
    }
  }
  EXPECT_THAT(unmapped, t::Eq(2 * (1 + kIstStackCount)));
}

TEST_F(SmpTest, X2ApicModeStartsApsWithInitSipiAndClearsEsrBeforeEachIcr) {
  const FakeSmpHardware& hw = InitSmp(X2ApicRecords(2), X2ApicOptions());
  ASSERT_TRUE(SmpIsX2ApicEnabled());
  ASSERT_THAT(SmpOnlineCpuCount(), t::Eq(2));

  // x2APIC mode has no INIT de-assert.
  EXPECT_THAT(IcrsTo(hw, 1),
              t::ElementsAre(kIcrX2ApicInit, kIcrStartupAt0x8000));
  // Each command clears the Error Status Register and then writes the 64-bit
  // ICR, with the destination in bits 32-63.
  std::vector<RegisterWrite> icr_writes;
  for (const auto& write : RegisterWrites(hw, 0)) {
    if (write.first == kLapicEsr || write.first == kLapicIcrLow) {
      icr_writes.push_back(write);
    }
  }
  EXPECT_THAT(icr_writes,
              t::ElementsAre(t::Pair(kLapicEsr, 0u),
                             t::Pair(kLapicIcrLow, (1ULL << 32) | 0x4500u),
                             t::Pair(kLapicEsr, 0u),
                             t::Pair(kLapicIcrLow, (1ULL << 32) | 0x4608u)));
}

TEST_F(SmpTest, ApThatMissesTheFirstSipiGetsASecondOne) {
  FakeSmpHardware& hw = ResetState();
  hw.SetApStart(1, FakeSmpHardware::ApStart::kMissesFirstSipi);
  WriteBiosAcpiTables(LapicRecords(2));
  IdtInit();
  SmpInit(kMultiboot1Magic, 0);

  ASSERT_THAT(SmpOnlineCpuCount(), t::Eq(2));
  EXPECT_THAT(IcrsTo(hw, 1),
              t::ElementsAre(kIcrXapicInitAssert, kIcrXapicInitDeassert,
                             kIcrStartupAt0x8000, kIcrStartupAt0x8000));
  // The second SIPI follows once the AP has not entered the trampoline within
  // 200 us of the first.
  const std::vector<Event> ipis = IpisTo(hw, 1);
  ASSERT_THAT(ipis.size(), t::Eq(4u));
  EXPECT_GE(ipis[3].time_ns - ipis[2].time_ns, 200000u);
}

TEST_F(SmpTest, DelaysUseTheTscRateMeasuredAgainstThePit) {
  // The PIT countdown measures the fake's 1 GHz TSC, so the 10 ms INIT-to-SIPI
  // delay takes 10 ms of it.
  {
    const FakeSmpHardware& hw = InitSmp(LapicRecords(2));
    const std::vector<Event> ipis = IpisTo(hw, 1);
    ASSERT_THAT(ipis.size(), t::Eq(3u));
    EXPECT_GE(ipis[2].time_ns - ipis[1].time_ns, 10000000u);
  }
  // Without the PIT, delays assume an 8 GHz TSC and take eight times as long.
  FakeSmpHardware::Options options = DefaultOptions();
  options.pit_responds = false;
  const FakeSmpHardware& hw = InitSmp(LapicRecords(2), options);
  const std::vector<Event> ipis = IpisTo(hw, 1);
  ASSERT_THAT(ipis.size(), t::Eq(3u));
  EXPECT_GE(ipis[2].time_ns - ipis[1].time_ns, 80000000u);
  EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(2));
}

TEST_F(SmpTest, WithoutAPitTheRatesComeFromHyperVOrStayAtTheirFallbacks) {
  // Hyper-V generation 2 VMs have no PIT: port 0x61 reads 0xFF, so OUT2 reads
  // high before any countdown. That must not pass for a finished calibration,
  // which would shrink every delay and timeout.
  FakeSmpHardware::Options options = DefaultOptions();
  options.pit_present = false;
  {
    const FakeSmpHardware& hw = InitSmp(LapicRecords(2), options);
    const std::vector<Event> ipis = IpisTo(hw, 1);
    ASSERT_THAT(ipis.size(), t::Eq(3u));
    EXPECT_GE(ipis[2].time_ns - ipis[1].time_ns, 80000000u);
    EXPECT_THAT(SmpCalibratedTimerInitialCount(),
                t::Eq(kFallbackApicTimerInitialCount));
    EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(2));
  }
  // Hyper-V reports a 2 GHz TSC (twice the fake's clock), so the 10 ms delay
  // takes 20 ms, and a 200 MHz Local APIC timer clock.
  options.hyperv_tsc_hz = 2000000000;
  options.hyperv_apic_timer_hz = 200000000;
  const FakeSmpHardware& hw = InitSmp(LapicRecords(2), options);
  const std::vector<Event> ipis = IpisTo(hw, 1);
  ASSERT_THAT(ipis.size(), t::Eq(3u));
  EXPECT_GE(ipis[2].time_ns - ipis[1].time_ns, 20000000u);
  EXPECT_THAT(SmpCalibratedTimerInitialCount(),
              t::Eq(200000000u / 16 / kApicTimerTargetHz));
  EXPECT_THAT(g_state.uart_log,
              t::HasSubstr("TSC rate from Hyper-V: 2000000000 Hz"));
  EXPECT_THAT(
      g_state.uart_log,
      t::HasSubstr("Local APIC timer clock from Hyper-V: 200000000 Hz"));
  EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(2));
}

TEST_F(SmpTest, WithoutAPitTheTscRateComesFromCpuid) {
  struct Case {
    const char* name;
    uint32_t leaf15_eax;
    uint32_t leaf15_ebx;
    uint32_t leaf15_ecx;
    uint32_t leaf16_eax;
    // The TSC rate that delays assume (0: the 8 GHz fallback).
    uint64_t expected_tsc_hz;
  };
  const Case kCases[] = {
      // The 24 MHz crystal clock times the TSC/crystal ratio 200/2.
      {"crystal clock and ratio", 2, 200, 24000000, 0, 2400000000},
      // The crystal clock derived from the 2.4 GHz base frequency (Skylake to
      // Coffee Lake client CPUs enumerate the ratio but not the crystal).
      {"derived crystal clock", 2, 200, 0, 2400, 2400000000},
      // Without the ratio, the TSC runs at the base frequency.
      {"base frequency", 1, 0, 0, 3000, 3000000000},
      // A 50 MHz TSC is implausible, and would shorten every delay.
      {"implausible base frequency", 1, 0, 0, 50, 0},
  };
  for (const Case& c : kCases) {
    SCOPED_TRACE(c.name);
    FakeSmpHardware::Options options = DefaultOptions();
    options.pit_present = false;
    options.cpuid_max_leaf = 0x16;
    options.cpuid_leaf15_eax = c.leaf15_eax;
    options.cpuid_leaf15_ebx = c.leaf15_ebx;
    options.cpuid_leaf15_ecx = c.leaf15_ecx;
    options.cpuid_leaf16_eax = c.leaf16_eax;
    const FakeSmpHardware& hw = InitSmp(LapicRecords(2), options);
    ASSERT_THAT(SmpOnlineCpuCount(), t::Eq(2));
    // The 10 ms INIT-to-SIPI delay at that rate, on the fake's 1 GHz TSC.
    const std::vector<Event> ipis = IpisTo(hw, 1);
    ASSERT_THAT(ipis.size(), t::Eq(3u));
    const uint64_t delay_ns = ipis[2].time_ns - ipis[1].time_ns;
    if (c.expected_tsc_hz != 0) {
      EXPECT_GE(delay_ns, c.expected_tsc_hz / 100);
      EXPECT_LT(delay_ns, 80000000u);
      EXPECT_THAT(
          g_state.uart_log,
          t::HasSubstr("[SMP] No PIT calibration; TSC rate from CPUID: " +
                       std::to_string(c.expected_tsc_hz) + " Hz\n"));
    } else {
      EXPECT_GE(delay_ns, 80000000u);
      EXPECT_THAT(g_state.uart_log, t::Not(t::HasSubstr("TSC rate from")));
    }
  }
}

TEST_F(SmpTest, SmpInitDoesNotStartProcessorsThatAreOnlyOnlineCapable) {
  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, 0, 1);
  AppendLapicRecord(&recs, 1, 1, 2);
  AppendLapicRecord(&recs, 2, 2, 1);
  AppendX2ApicRecord(&recs, 3, 3, 2);
  const FakeSmpHardware& hw = InitSmp(recs);

  ASSERT_THAT(SmpCpuCount(), t::Eq(2));
  EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(2));
  EXPECT_THAT(SmpGetCpuInfo(1)->apic_id, t::Eq(2u));
  // Only the enabled AP got IPIs, and only one AP stack was allocated.
  for (const Event& ipi : hw.Ipis()) {
    EXPECT_THAT(ipi.value >> 32, t::Eq(2u));
  }
  EXPECT_THAT(IcrsTo(hw, 2), t::Contains(kIcrStartupAt0x8000));
  EXPECT_THAT(g_state.pmm_alloc_calls, t::Eq(1));
  EXPECT_THAT(g_state.uart_log, t::HasSubstr("[SMP] Online CPUs: 2/2"));
}

TEST_F(SmpTest, SmpInitStartsAtMostMaxSupportedCpusAndLogsTheRest) {
  const FakeSmpHardware& hw = InitSmp(X2ApicRecords(kMaxSupportedCpus + 5));

  EXPECT_THAT(SmpCpuCount(), t::Eq(kMaxSupportedCpus));
  EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(kMaxSupportedCpus));
  EXPECT_THAT(g_state.pmm_alloc_calls, t::Eq(kMaxSupportedCpus - 1));
  EXPECT_THAT(IcrsTo(hw, kMaxSupportedCpus - 1),
              t::Contains(kIcrStartupAt0x8000));
  EXPECT_THAT(IcrsTo(hw, kMaxSupportedCpus), t::IsEmpty());
  EXPECT_THAT(g_state.uart_log,
              t::HasSubstr("[SMP] Ignoring 5 more processor entries (limit: "
                           "128 CPUs)"));
}

TEST_F(SmpTest, DiscoverTopologyAndSmpInitEndToEndAndPanicsOnFailure) {
  constexpr uintptr_t kMbBase = 0x6000;
  constexpr uintptr_t kRsdtAddr = 0x6100;
  constexpr uintptr_t kXsdtAddr = 0x6200;
  constexpr uintptr_t kBadRsdtAddr = 0x6300;
  constexpr uintptr_t kMadtAddr = 0x6400;
  const auto write_tables = [] {
    WriteMadt(kMadtAddr, kDefaultLocalApicPhysAddr, LapicRecords(4));
    WriteXsdt(kXsdtAddr, {kMadtAddr});
    WriteRsdt(kRsdtAddr, {static_cast<uint32_t>(kMadtAddr)});
    WriteRsdt(kBadRsdtAddr, {});

    // Multiboot2 structure with Tag 15 (ACPI_NEW) and Tag 14 (ACPI_OLD).
    uint8_t* const mb_buf = At(kMbBase);
    *reinterpret_cast<uint32_t*>(mb_buf) = 96;
    *reinterpret_cast<uint32_t*>(mb_buf + 4) = 0;
    *reinterpret_cast<uint32_t*>(mb_buf + 8) = kMultiboot2TagAcpiNew;
    *reinterpret_cast<uint32_t*>(mb_buf + 12) = 8 + sizeof(RawRsdpV2);
    WriteRsdpV2(kMbBase + 16, 0, kXsdtAddr);
    *reinterpret_cast<uint32_t*>(mb_buf + 56) = kMultiboot2TagAcpiOld;
    *reinterpret_cast<uint32_t*>(mb_buf + 60) = 8 + sizeof(RawRsdpV1);
    WriteRsdpV1(kMbBase + 64, static_cast<uint32_t>(kRsdtAddr));
    *reinterpret_cast<uint32_t*>(mb_buf + 88) = kMultiboot2TagEnd;
    *reinterpret_cast<uint32_t*>(mb_buf + 92) = 8;

    // A stale RSDP v1 pointing to an empty RSDT at the start of the BIOS ROM
    // area, followed by the valid RSDP v1, to verify that SmpDiscoverTopology
    // continues scanning past a stale RSDP whose SDT lacks a MADT.
    WriteRsdpV1(0xE0000, static_cast<uint32_t>(kBadRsdtAddr));
    WriteRsdpV1(0xE0040, static_cast<uint32_t>(kRsdtAddr));
  };
  ResetState();
  write_tables();
  IdtInit();

  // Verify SmpInit succeeds via Multiboot2 Tag 15 and brings all 4 CPUs online.
  SmpInit(kMultiboot2Magic, kMbBase);
  EXPECT_THAT(SmpCpuCount(), t::Eq(4));
  EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(4));
  EXPECT_THAT(SmpLocalApicPhysAddr(), t::Eq(kDefaultLocalApicPhysAddr));
  EXPECT_THAT(g_state.pmm_alloc_calls, t::Eq(3));

  // If Tag 15's XSDT is corrupted and Tag 15 has rsdt_address == 0,
  // SmpDiscoverTopology must fall back to Multiboot2 Tag 14 (ACPI_OLD). The
  // APs started before are reset with INIT and started again.
  At<RawSdtHeader>(kXsdtAddr)->checksum ^= 0xFF;
  SmpInit(kMultiboot2Magic, kMbBase);
  EXPECT_THAT(SmpCpuCount(), t::Eq(4));
  EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(4));
  At<RawSdtHeader>(kXsdtAddr)->checksum ^= 0xFF;

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

  // Verify SmpInit also succeeds via the legacy BIOS ROM scan (skipping the
  // stale RSDP and finding the valid RSDP v1 -> RSDT) under Multiboot1.
  SmpInit(kMultiboot1Magic, 0);
  EXPECT_THAT(SmpCpuCount(), t::Eq(4));
  EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(4));

  // Verify SmpInit panics via CHECK if mapping the Local APIC MMIO page fails,
  // or if PMM stack allocation fails. (Without a valid MADT, it runs on the
  // BSP alone: see SmpInitRunsOnTheBspAloneWithoutAValidMadt.)
  FakeSmpHardware::Options options = DefaultOptions();
  options.map_failure_addr = kDefaultLocalApicPhysAddr;
  ResetState(options);
  write_tables();
  EXPECT_DEATH(SmpInit(kMultiboot2Magic, kMbBase),
               "Check failed: hw\\.MapLocalApicPage\\(lapic_base\\)");

  ResetState();
  write_tables();
  g_state.fail_pmm_on_call = 2;
  EXPECT_DEATH(SmpInit(kMultiboot2Magic, kMbBase),
               "Check failed: stack_block != 0");

  // Unmapping a guard page fails: here the one below the first AP's stack.
  options = DefaultOptions();
  options.map_failure_addr = kFirstPmmFrame * kPageSize;
  ResetState(options);
  write_tables();
  EXPECT_DEATH(SmpInit(kMultiboot2Magic, kMbBase),
               "Check failed: hw\\.UnmapGuardPage\\(guard_page\\)");
}

TEST_F(SmpTest, SmpInitFindsTheRsdpInTheEbda) {
  ResetState();
  // The BIOS Data Area word at 0x40E holds the EBDA segment.
  constexpr uintptr_t kEbdaBase = 0x9FC00;
  *At<uint16_t>(0x40E) = static_cast<uint16_t>(kEbdaBase >> 4);
  WriteMadt(0x6400, kDefaultLocalApicPhysAddr, LapicRecords(2));
  WriteRsdt(0x6100, {0x6400});
  WriteRsdpV1(kEbdaBase + 16, 0x6100);
  IdtInit();
  SmpInit(kMultiboot1Magic, 0);

  EXPECT_THAT(SmpCpuCount(), t::Eq(2));
  EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(2));
}

TEST_F(SmpTest, SmpInitRunsOnTheBspAloneWithoutAValidMadt) {
  struct Case {
    const char* name;
    bool x2apic;
    // Writes BIOS ACPI tables whose MADT has a bad checksum (else none).
    bool corrupt_madt;
    // IA32_APIC_BASE of the BSP, and the Local APIC base SmpInit() takes.
    uint64_t apic_base_msr;
    uintptr_t expected_lapic_base;
  };
  const Case kCases[] = {
      // Multiboot1 under UEFI: no RSDP where SmpInit() looks, since UEFI
      // firmware publishes it only in the EFI system table.
      {"no RSDP, xAPIC", false, false, 0xFED00900, 0xFED00000},
      {"no RSDP, x2APIC", true, false, 0xFED00900, 0xFED00000},
      // IA32_APIC_BASE with no valid base: the architectural default.
      {"no RSDP, no Local APIC base", false, false, 0x900,
       kDefaultLocalApicPhysAddr},
      {"MADT with a bad checksum", false, true, 0xFEE00900,
       kDefaultLocalApicPhysAddr},
  };
  for (const Case& c : kCases) {
    SCOPED_TRACE(c.name);
    FakeSmpHardware::Options options =
        c.x2apic ? X2ApicOptions() : DefaultOptions();
    // The machine has 4 CPUs; the BSP's Local APIC ID is 2, its CPUID initial
    // APIC ID 7.
    options.bsp_apic_id = 2;
    options.bsp_cpuid_apic_id = 7;
    options.bsp_apic_base_msr = c.apic_base_msr;
    FakeSmpHardware& hw = ResetState(options);
    if (c.corrupt_madt) {
      WriteBiosAcpiTables(LapicRecords(4));
      At<RawSdtHeader>(0x6400)->checksum ^= 0xFF;
    }
    IdtInit();
    SmpInit(kMultiboot1Magic, 0);

    ASSERT_THAT(SmpIsX2ApicEnabled(), t::Eq(c.x2apic));
    ASSERT_THAT(SmpCpuCount(), t::Eq(1));
    EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(1));
    EXPECT_THAT(SmpLocalApicPhysAddr(), t::Eq(c.expected_lapic_base));
    const CpuInfo* const bsp = SmpGetCpuInfo(0);
    EXPECT_THAT(bsp->apic_id, t::Eq(2u));
    EXPECT_THAT(bsp->observed_apic_id, t::Eq(2u));
    EXPECT_TRUE(bsp->is_bsp);
    EXPECT_TRUE(bsp->online);
    EXPECT_TRUE(bsp->long_mode_active);
    EXPECT_THAT(bsp->stack_top, t::Eq(DefaultOptions().bsp_stack.top));
    EXPECT_THAT(CurrentCpu(), t::Eq(SmpGetCpuLocal(0)));
    EXPECT_THAT(SmpGetCpuLocal(0)->apic_id, t::Eq(2u));
    EXPECT_THAT(g_state.uart_log,
                t::HasSubstr("[SMP] WARNING: no valid ACPI MADT; using the "
                             "BSP only\n"));
    EXPECT_THAT(g_state.vga_log,
                t::HasSubstr("[SMP] WARNING: no valid ACPI MADT"));
    EXPECT_THAT(g_state.uart_log,
                t::HasSubstr("[SMP] BSP Local APIC ID 2 differs from its CPUID "
                             "initial APIC ID 7; using the Local APIC ID\n"));
    EXPECT_THAT(g_state.uart_log, t::HasSubstr("[SMP] Online CPUs: 1/1"));
    // No AP is started: no stacks, trampoline or IPIs.
    EXPECT_THAT(g_state.pmm_alloc_calls, t::Eq(0));
    EXPECT_THAT(IndexOf(hw.Events(), Event::Kind::kTrampolineInstalled),
                t::Eq(-1));
    EXPECT_THAT(hw.Ipis(), t::IsEmpty());
    // The timer is calibrated against the PIT and runs as on any machine.
    const uint32_t count = SmpCalibratedTimerInitialCount();
    EXPECT_THAT(count, t::Eq(ComputeApicTimerInitialCount(
                           options.lapic_timer_calibration_ticks,
                           kPitCalibrationReloadCount, kApicTimerTargetHz)));
    const std::vector<RegisterWrite> writes = RegisterWrites(hw, 2);
    ASSERT_GE(writes.size(), 3u);
    EXPECT_THAT(std::vector<RegisterWrite>(writes.end() - 3, writes.end()),
                t::ElementsAre(t::Pair(kLapicTimerDivide, 0x3u),
                               t::Pair(kLapicLvtTimer, 0x20020u),
                               t::Pair(kLapicTimerInitCount, count)));
    EXPECT_TRUE(AreInterruptsEnabled());
    // The other CPUs, which the kernel does not know, get no IPI either when
    // the kernel panics.
    SmpStopOtherCpusForPanic();
    EXPECT_THAT(hw.Ipis(), t::IsEmpty());
  }
}

TEST_F(SmpTest, SmpInitLeavesOutApsThatDoNotStart) {
  FakeSmpHardware& hw = ResetState();
  // The CPUs with APIC IDs 2 and 4 never report in.
  hw.SetApStart(2, FakeSmpHardware::ApStart::kNever);
  hw.SetApStart(4, FakeSmpHardware::ApStart::kNever);
  WriteBiosAcpiTables(LapicRecords(5));
  IdtInit();
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
  // A CPU that does not report in gets both SIPIs, and INIT again after the
  // 10 s startup timeout, so that it cannot start late with a stale CPU index.
  for (const uint32_t apic_id : {2u, 4u}) {
    SCOPED_TRACE(apic_id);
    EXPECT_THAT(IcrsTo(hw, apic_id),
                t::ElementsAre(kIcrXapicInitAssert, kIcrXapicInitDeassert,
                               kIcrStartupAt0x8000, kIcrStartupAt0x8000,
                               kIcrXapicInitAssert, kIcrXapicInitDeassert));
    const std::vector<Event> ipis = IpisTo(hw, apic_id);
    ASSERT_THAT(ipis.size(), t::Eq(6u));
    EXPECT_GE(ipis[4].time_ns - ipis[3].time_ns, 10000000000u);
  }
  // The stack blocks of the CPUs that did not start went back to the PMM, the
  // second and fourth of the four, with their guard pages mapped again.
  const auto block_addr = [](const int block) {
    return static_cast<uintptr_t>(
        (kFirstPmmFrame + block * kApStackBlockFrames) * kPageSize);
  };
  EXPECT_THAT(g_state.freed_frames,
              t::ElementsAre(t::Pair(block_addr(1), kApStackBlockFrames),
                             t::Pair(block_addr(3), kApStackBlockFrames)));
  EXPECT_THAT(
      hw.GuardPages(),
      t::ElementsAreArray(GuardPagesOfBlocks({block_addr(0), block_addr(2)})));
  // CPU 2 (APIC ID 3) kept its stacks when it moved down: the third block.
  EXPECT_THAT(SmpGetCpuInfo(2)->stack_base, t::Eq(block_addr(2) + kPageSize));
  EXPECT_THAT(IstStackTop(GdtGetTss(2), kIstMachineCheck),
              t::Eq(block_addr(3)));
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

TEST_F(SmpTest, SmpInitIdentifiesTheBspByItsLocalApicIdRegister) {
  // CPUID reports initial APIC ID 0, but the BSP's x2APIC ID register (MSR
  // 0x802) and the MADT agree on 2. Matching by the CPUID value would leave
  // the real BSP entry in the AP list, and SmpInit would send INIT to it.
  FakeSmpHardware::Options options = X2ApicOptions();
  options.bsp_apic_id = 2;
  options.bsp_cpuid_apic_id = 0;
  const FakeSmpHardware& hw = InitSmp(X2ApicRecords(4), options);

  ASSERT_TRUE(SmpIsX2ApicEnabled());
  ASSERT_THAT(SmpCpuCount(), t::Eq(4));
  constexpr uint32_t kExpectedApicIds[] = {2, 0, 1, 3};
  for (int i = 0; i < 4; ++i) {
    EXPECT_THAT(SmpGetCpuInfo(i)->apic_id, t::Eq(kExpectedApicIds[i]));
    EXPECT_THAT(SmpGetCpuLocal(i)->apic_id, t::Eq(kExpectedApicIds[i]));
    EXPECT_THAT(SmpGetCpuInfo(i)->is_bsp, t::Eq(i == 0));
  }
  EXPECT_THAT(SmpGetCpuInfo(0)->observed_apic_id, t::Eq(2u));
  EXPECT_THAT(IcrsTo(hw, 2), t::IsEmpty());
  EXPECT_THAT(g_state.uart_log,
              t::HasSubstr("[SMP] BSP Local APIC ID 2 differs from its CPUID "
                           "initial APIC ID 0; using the Local APIC ID\n"));
  EXPECT_THAT(g_state.uart_log, t::HasSubstr("[SMP] CPU 0 (APIC ID 2, BSP)"));
}

TEST_F(SmpTest, SmpInitSkipsApicIdsThatXapicModeCannotAddress) {
  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, 0, 1);
  AppendX2ApicRecord(&recs, 0xFFu, 1, 1);
  AppendLapicRecord(&recs, 2, 1, 1);
  AppendX2ApicRecord(&recs, 0x100u, 3, 1);
  AppendLapicRecord(&recs, 4, 2, 1);
  const FakeSmpHardware& hw = InitSmp(recs);

  // ICR destinations are 8 bits wide in xAPIC mode, so APIC IDs >= 0xFF
  // cannot be started (0x100 would be truncated to the BSP's ID 0).
  ASSERT_FALSE(SmpIsX2ApicEnabled());
  ASSERT_THAT(SmpCpuCount(), t::Eq(3));
  EXPECT_THAT(SmpGetCpuInfo(1)->apic_id, t::Eq(1u));
  EXPECT_THAT(SmpGetCpuInfo(2)->apic_id, t::Eq(2u));
  EXPECT_THAT(g_state.pmm_alloc_calls, t::Eq(2));
  EXPECT_THAT(IcrsTo(hw, 0), t::IsEmpty());
  EXPECT_THAT(g_state.uart_log,
              t::HasSubstr("[SMP] Skipping APIC ID 255: not addressable in "
                           "xAPIC mode\n[SMP] Skipping APIC ID 256: not "
                           "addressable in xAPIC mode\n"));
  EXPECT_THAT(g_state.uart_log, t::HasSubstr("[SMP] Online CPUs: 3/3"));
}

TEST_F(SmpTest, EveryCpuEnablesItsLocalApicAndStartsThePeriodicTimer) {
  FakeSmpHardware::Options options = DefaultOptions();
  options.lapic_timer_calibration_ticks = 60000;
  const FakeSmpHardware& hw = InitSmp(LapicRecords(3), options);
  const uint32_t count = SmpCalibratedTimerInitialCount();
  EXPECT_THAT(count,
              t::Eq(ComputeApicTimerInitialCount(
                  60000, kPitCalibrationReloadCount, kApicTimerTargetHz)));

  for (uint32_t apic_id = 0; apic_id < 3; ++apic_id) {
    SCOPED_TRACE(apic_id);
    const std::vector<RegisterWrite> writes = RegisterWrites(hw, apic_id);
    // TPR 0, then the SVR (reset value 0xFF) with the software-enable bit and
    // spurious vector 0xFF.
    ASSERT_GE(writes.size(), 2u);
    EXPECT_THAT(writes[0], t::Pair(kLapicTpr, 0u));
    EXPECT_THAT(writes[1], t::Pair(kLapicSvr, 0x1FFu));
    // Last, the periodic timer on vector 0x20, its clock divided by 16, with
    // the calibrated count.
    ASSERT_GE(writes.size(), 5u);
    EXPECT_THAT(std::vector<RegisterWrite>(writes.end() - 3, writes.end()),
                t::ElementsAre(t::Pair(kLapicTimerDivide, 0x3u),
                               t::Pair(kLapicLvtTimer, 0x20020u),
                               t::Pair(kLapicTimerInitCount, count)));
  }
}

TEST_F(SmpTest, ApLeftInX2ApicModeStepsDownToXapicModeThroughDisabled) {
  FakeSmpHardware& hw = ResetState();
  // The firmware left the AP with APIC ID 1 in x2APIC mode (EN | EXTD), but
  // the BSP does not support it.
  hw.SetApicBaseMsr(1, 0xFEE00C00);
  WriteBiosAcpiTables(LapicRecords(2));
  IdtInit();
  SmpInit(kMultiboot1Magic, 0);

  ASSERT_FALSE(SmpIsX2ApicEnabled());
  ASSERT_THAT(SmpOnlineCpuCount(), t::Eq(2));
  // x2APIC -> xAPIC directly is invalid; the fake fails the test on it.
  EXPECT_THAT(MsrWrites(hw, 1), t::ElementsAre(t::Pair(0x1Bu, 0xFEE00000u),
                                               t::Pair(0x1Bu, 0xFEE00800u)));
  EXPECT_THAT(SmpGetCpuInfo(1)->observed_apic_id, t::Eq(1u));
}

TEST_F(SmpTest, BspEnablesX2ApicModeThroughXapicMode) {
  FakeSmpHardware::Options options = X2ApicOptions();
  // Local APIC disabled at boot (only the BSP flag is set).
  options.bsp_apic_base_msr = 0xFEE00100;
  const FakeSmpHardware& hw = InitSmp(X2ApicRecords(1), options);

  ASSERT_TRUE(SmpIsX2ApicEnabled());
  // EN (bit 11) first, then EXTD (bit 10): disabled -> x2APIC directly is
  // invalid.
  EXPECT_THAT(MsrWrites(hw, 0), t::ElementsAre(t::Pair(0x1Bu, 0xFEE00900u),
                                               t::Pair(0x1Bu, 0xFEE00D00u)));
  // TPR and SVR through their x2APIC MSRs (0x808 and 0x80F).
  const std::vector<RegisterWrite> writes = RegisterWrites(hw, 0);
  ASSERT_GE(writes.size(), 2u);
  EXPECT_THAT(writes[0], t::Pair(kLapicTpr, 0u));
  EXPECT_THAT(writes[1], t::Pair(kLapicSvr, 0x1FFu));
}

TEST_F(SmpTest, EoiAndTimerCountGoToTheLocalApicOfTheCallingCpu) {
  for (const bool x2apic : {false, true}) {
    SCOPED_TRACE(x2apic ? "x2APIC" : "xAPIC");
    const FakeSmpHardware& hw =
        InitSmp(LapicRecords(2), x2apic ? X2ApicOptions() : DefaultOptions());
    ASSERT_THAT(SmpIsX2ApicEnabled(), t::Eq(x2apic));

    SmpSendLocalApicEoi();
    EXPECT_THAT(hw.EoiCount(0), t::Eq(1));
    EXPECT_THAT(hw.EoiCount(1), t::Eq(0));

    SmpSetLocalApicTimerInitialCount(12345);
    EXPECT_THAT(RegisterWrites(hw, 0).back(),
                t::Pair(kLapicTimerInitCount, 12345u));
    // A zero count leaves the timer alone.
    const int write_count = static_cast<int>(RegisterWrites(hw, 0).size());
    SmpSetLocalApicTimerInitialCount(0);
    EXPECT_THAT(static_cast<int>(RegisterWrites(hw, 0).size()),
                t::Eq(write_count));
  }
}

TEST_F(SmpTest, SmpSendIpiTargetsTheApicIdAndTheReceivingCpuHandlesIt) {
  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, 0, 1);
  AppendLapicRecord(&recs, 1, 3, 1);
  AppendLapicRecord(&recs, 2, 5, 1);
  for (const bool x2apic : {false, true}) {
    SCOPED_TRACE(x2apic ? "x2APIC" : "xAPIC");
    const FakeSmpHardware& hw =
        InitSmp(recs, x2apic ? X2ApicOptions() : DefaultOptions());
    ASSERT_THAT(SmpCpuCount(), t::Eq(3));
    const CpuLocal* const target = SmpGetCpuLocal(2);
    ASSERT_THAT(target->ipi_count.load(), t::Eq(0));

    const bool irq_before = AreInterruptsEnabled();
    SmpSendIpi(2, kVectorWakeupIpi);
    EXPECT_THAT(AreInterruptsEnabled(), t::Eq(irq_before));

    // A fixed IPI with vector 0x21 from the BSP to APIC ID 5.
    const std::vector<Event> ipis = hw.Ipis();
    ASSERT_FALSE(ipis.empty());
    EXPECT_THAT(ipis.back().cpu, t::Eq(0u));
    EXPECT_THAT(ipis.back().value, t::Eq((5ULL << 32) | kIcrWakeupIpi));
    // CPU 2 handles it: IdtDispatch() counts it there and writes the EOI to
    // CPU 2's Local APIC.
    EXPECT_TRUE(WaitUntil([&] { return target->ipi_count.load() == 1; }));
    EXPECT_TRUE(WaitUntil([&] { return hw.EoiCount(5) == 1; }));
    EXPECT_THAT(hw.EoiCount(0), t::Eq(0));
    EXPECT_THAT(hw.EoiCount(3), t::Eq(0));
    EXPECT_THAT(SmpGetCpuLocal(1)->ipi_count.load(), t::Eq(0));
    EXPECT_THAT(SmpGetCpuLocal(0)->ipi_count.load(), t::Eq(0));
  }
}

TEST_F(SmpTest, SmpSendNmiSendsAnNmiToTheApicIdOfTheTargetCpu) {
  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, 0, 1);
  AppendLapicRecord(&recs, 1, 3, 1);
  AppendLapicRecord(&recs, 2, 5, 1);
  for (const bool x2apic : {false, true}) {
    SCOPED_TRACE(x2apic ? "x2APIC" : "xAPIC");
    const FakeSmpHardware& hw =
        InitSmp(recs, x2apic ? X2ApicOptions() : DefaultOptions());
    ASSERT_THAT(SmpCpuCount(), t::Eq(3));

    const bool irq_before = AreInterruptsEnabled();
    SmpSendNmi(1);
    EXPECT_THAT(AreInterruptsEnabled(), t::Eq(irq_before));

    // An NMI, with no vector, from the BSP to APIC ID 3.
    const std::vector<Event> ipis = hw.Ipis();
    ASSERT_FALSE(ipis.empty());
    EXPECT_THAT(ipis.back().cpu, t::Eq(0u));
    EXPECT_THAT(ipis.back().value, t::Eq((3ULL << 32) | kIcrNmi));
  }
}

TEST_F(SmpTest, XapicIpiWritesTheIcrHighThenLowAndFailsWhenItStaysBusy) {
  const FakeSmpHardware& hw = InitSmp(LapicRecords(2));
  const int writes_before = static_cast<int>(RegisterWrites(hw, 0).size());
  SmpSendIpi(1, kVectorWakeupIpi);
  const std::vector<RegisterWrite> writes = RegisterWrites(hw, 0);
  EXPECT_THAT(
      std::vector<RegisterWrite>(writes.begin() + writes_before, writes.end()),
      t::ElementsAre(t::Pair(kLapicEsr, 0u), t::Pair(kLapicIcrHigh, 1u << 24),
                     t::Pair(kLapicIcrLow, kIcrWakeupIpi)));

  // If ICR bit 12 (Delivery Status: Send Pending) is stuck, the IPI fails the
  // check that the previous one was delivered, before touching the ICR.
  FakeSmpHardware& single_cpu_hw = InitSmp(LapicRecords(1));
  single_cpu_hw.SetIcrStuckPending(true);
  EXPECT_DEATH(SmpSendIpi(0, kVectorWakeupIpi),
               "Check failed: WaitForIcrIdle\\(lapic_base\\)");
}

TEST_F(SmpTest, ApIdleLoopRunsSchedulerWorkWithInterruptsDisabled) {
  InitSmp(LapicRecords(3));
  ASSERT_THAT(SmpCpuCount(), t::Eq(3));

  // Work that the scheduler queued for CPU 2 while it was halted. The wakeup
  // IPI ends CPU 2's `hlt`, and its idle loop runs the work.
  g_idle_work[2].store(3);
  SmpSendIpi(2, kVectorWakeupIpi);
  EXPECT_TRUE(WaitUntil([] { return g_idle_work[2].load() == 0; }));
  // Then it halts again until the next interrupt.
  EXPECT_TRUE(
      WaitUntil([] { return SmpGetCpuLocal(2)->ipi_count.load() == 1; }));
  SmpSendIpi(2, kVectorWakeupIpi);
  EXPECT_TRUE(
      WaitUntil([] { return SmpGetCpuLocal(2)->ipi_count.load() == 2; }));
  EXPECT_THAT(g_idle_poll_violations.load(), t::Eq(0));
}

TEST_F(SmpTest,
       SmpRunOnAllCpusDispatchesConcurrentlyAcrossSingleAndMultipleCpus) {
  // 1. Single-CPU topology: SmpRunOnAllCpus executes work_fn on CPU 0.
  const FakeSmpHardware& single_hw = InitSmp(LapicRecords(1));
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
  EXPECT_THAT(single_hw.Ipis(), t::IsEmpty());

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
  //    across all 4 CPUs, each on its own thread with its CpuLocal bound, with
  //    a simultaneous barrier and IrqSpinLock synchronization across multiple
  //    dispatch rounds.
  const FakeSmpHardware& hw = InitSmp(LapicRecords(4));
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
          EXPECT_TRUE(AreInterruptsEnabled());

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
  // Each round woke every AP with the maskable IPI 0x21 (resent to an AP that
  // was slow to acknowledge).
  for (uint32_t apic_id = 1; apic_id < 4; ++apic_id) {
    const std::vector<uint32_t> icrs = IcrsTo(hw, apic_id);
    EXPECT_GE(std::count(icrs.begin(), icrs.end(), kIcrWakeupIpi), 3);
  }
}

TEST_F(SmpTest, SmpRunOnAllCpusChecksItsCallingContext) {
  InitSmp(LapicRecords(4));
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

TEST_F(SmpTest, TlbShootdownMakesEveryCpuFlushBeforeItReturns) {
  for (const bool x2apic : {false, true}) {
    SCOPED_TRACE(x2apic ? "x2APIC" : "xAPIC");
    const FakeSmpHardware& hw =
        InitSmp(LapicRecords(4), x2apic ? X2ApicOptions() : DefaultOptions());
    ASSERT_THAT(SmpCpuCount(), t::Eq(4));
    const int events_before = static_cast<int>(hw.Events().size());

    SmpTlbShootdown();
    EXPECT_TRUE(AreInterruptsEnabled());

    // CPU 0 flushed its own TLB and sent each AP a fixed IPI on vector 0xF0,
    // and each AP flushed its TLB when it handled the IPI, all before
    // SmpTlbShootdown() returned.
    const std::vector<Event> events = hw.Events();
    int ipi_index[4] = {-1, -1, -1, -1};
    int flush_index[4] = {-1, -1, -1, -1};
    for (int i = events_before; i < static_cast<int>(events.size()); ++i) {
      const Event& event = events[i];
      if (event.kind == Event::Kind::kIpi &&
          static_cast<uint32_t>(event.value) == kIcrTlbShootdownIpi) {
        EXPECT_THAT(event.cpu, t::Eq(0u));
        ipi_index[event.value >> 32] = i;
      } else if (event.kind == Event::Kind::kTlbFlush) {
        flush_index[event.cpu] = i;
      }
    }
    EXPECT_THAT(ipi_index[0], t::Eq(-1));
    EXPECT_THAT(flush_index[0], t::Ge(events_before));
    for (int apic_id = 1; apic_id < 4; ++apic_id) {
      SCOPED_TRACE(apic_id);
      EXPECT_THAT(ipi_index[apic_id], t::Ge(events_before));
      EXPECT_THAT(flush_index[apic_id], t::Gt(ipi_index[apic_id]));
    }
    for (uint32_t apic_id = 0; apic_id < 4; ++apic_id) {
      EXPECT_THAT(hw.TlbFlushCount(apic_id), t::Eq(1));
    }

    // Each shootdown flushes every CPU again.
    SmpTlbShootdown();
    for (uint32_t apic_id = 0; apic_id < 4; ++apic_id) {
      EXPECT_THAT(hw.TlbFlushCount(apic_id), t::Eq(2));
    }
  }
}

TEST_F(SmpTest, TlbShootdownFlushesOnlyTheCallingCpuWhenNoOtherCpuRuns) {
  // Before SmpInit().
  const FakeSmpHardware& hw = ResetState();
  SmpTlbShootdown();
  EXPECT_THAT(hw.TlbFlushCount(0), t::Eq(1));
  EXPECT_THAT(hw.Ipis(), t::IsEmpty());

  // On a single CPU.
  const FakeSmpHardware& single_cpu_hw = InitSmp(LapicRecords(1));
  SmpTlbShootdown();
  EXPECT_THAT(single_cpu_hw.TlbFlushCount(0), t::Eq(1));
  EXPECT_THAT(single_cpu_hw.Ipis(), t::IsEmpty());
}

TEST_F(SmpTest, ConcurrentTlbShootdownsWithInterruptsDisabledAllComplete) {
  const FakeSmpHardware& hw = InitSmp(LapicRecords(4));
  ASSERT_THAT(SmpCpuCount(), t::Eq(4));

  struct ShootdownState {
    std::atomic<int> calls{0};
    std::atomic<int> aps_done{0};
  };
  ShootdownState state;
  SmpRunOnAllCpus(
      [](const int cpu_index, void* const raw_state) {
        ShootdownState* const st = static_cast<ShootdownState*>(raw_state);
        // The fake interrupts a CPU only while it halts, so no CPU here takes
        // an IPI, as if all of them had interrupts disabled: the shootdowns
        // complete only because each waiting CPU handles the requests of the
        // others.
        const ScopedIrqDisable irq_disable;
        for (int i = 0; i < 50; ++i) {
          SmpTlbShootdown();
          st->calls.fetch_add(1);
        }
        EXPECT_FALSE(AreInterruptsEnabled());
        if (cpu_index != 0) {
          // A halted AP takes the IPIs of later shootdowns.
          st->aps_done.fetch_add(1);
          return;
        }
        // The fake never interrupts CPU 0, so it keeps handling the requests
        // of the APs until they are done.
        while (st->aps_done.load() < 3) {
          SmpTlbShootdown();
        }
      },
      &state);

  EXPECT_THAT(state.calls.load(), t::Eq(4 * 50));
  for (uint32_t apic_id = 0; apic_id < 4; ++apic_id) {
    EXPECT_THAT(hw.TlbFlushCount(apic_id), t::Gt(0));
  }
  EXPECT_TRUE(AreInterruptsEnabled());
}

TEST_F(SmpTest, SmpRunOnAllCpusServicesTlbShootdownFromApBusyInIdlePoll) {
  // CPU 1 runs scheduler work that issues a TLB shootdown while CPU 0 waits,
  // with interrupts disabled, for CPU 1 to acknowledge a dispatch: CPU 1 can't
  // acknowledge before its shootdown completes, so CPU 0 must handle CPU 1's
  // request while it waits.
  const FakeSmpHardware& hw = InitSmp(LapicRecords(2));
  ASSERT_THAT(SmpCpuCount(), t::Eq(2));

  static std::atomic<bool> ap1_in_work{false};
  static std::atomic<bool> shootdown_done{false};
  ap1_in_work.store(false);
  shootdown_done.store(false);
  g_idle_work_hook.store([](const int cpu_index) {
    if (cpu_index == 1) {
      ap1_in_work.store(true, std::memory_order_release);
      SmpTlbShootdown();
      shootdown_done.store(true, std::memory_order_release);
    }
  });
  g_idle_work[1].store(1, std::memory_order_release);
  SmpSendIpi(1, kVectorWakeupIpi);
  ASSERT_TRUE(
      WaitUntil([] { return ap1_in_work.load(std::memory_order_acquire); }));

  std::atomic<int> ran{0};
  SmpRunOnAllCpus(
      [](const int, void* const ctx) {
        static_cast<std::atomic<int>*>(ctx)->fetch_add(1);
      },
      &ran);
  EXPECT_TRUE(shootdown_done.load(std::memory_order_acquire));
  EXPECT_THAT(ran.load(), t::Eq(2));
  for (uint32_t apic_id = 0; apic_id < 2; ++apic_id) {
    EXPECT_THAT(hw.TlbFlushCount(apic_id), t::Ge(1));
  }
  EXPECT_THAT(g_idle_poll_violations.load(), t::Eq(0));
}

TEST_F(SmpTest, TlbShootdownChecksItsCallingContextAndThatEveryCpuResponds) {
  FakeSmpHardware& hw = InitSmp(LapicRecords(2));
  // Another CPU might be spinning on the lock with interrupts disabled.
  EXPECT_DEATH(
      {
        IrqSpinLock lock;
        const IrqSpinLockGuard guard(lock);
        SmpTlbShootdown();
      },
      "Check failed: !HoldsAnyIrqSpinLock\\(\\)");
  // A CPU that is off never handles the IPI.
  hw.PowerOffAps();
  EXPECT_DEATH(SmpTlbShootdown(), "Check failed: cpus_responded_in_time");
}

TEST_F(SmpTest, X2ApicModeRunsWith32BitApicIds) {
  std::vector<uint8_t> recs;
  AppendX2ApicRecord(&recs, 0x100u, 0, 1);
  AppendX2ApicRecord(&recs, 0xFFu, 1, 1);
  AppendX2ApicRecord(&recs, 0x12345678u, 2, 1);
  AppendX2ApicRecord(&recs, 0xFFFFFFFEu, 3, 1);
  FakeSmpHardware::Options options = X2ApicOptions();
  options.bsp_apic_id = 0x100u;
  const FakeSmpHardware& hw = InitSmp(recs, options);

  ASSERT_TRUE(SmpIsX2ApicEnabled());
  ASSERT_THAT(SmpCpuCount(), t::Eq(4));
  EXPECT_THAT(SmpGetCpuInfo(0)->apic_id, t::Eq(0x100u));
  EXPECT_THAT(SmpGetCpuLocal(0)->apic_id, t::Eq(0x100u));
  EXPECT_THAT(SmpGetCpuInfo(2)->apic_id, t::Eq(0x12345678u));
  EXPECT_THAT(SmpGetCpuInfo(2)->observed_apic_id, t::Eq(0x12345678u));
  EXPECT_THAT(SmpGetCpuLocal(2)->apic_id, t::Eq(0x12345678u));
  EXPECT_THAT(IcrsTo(hw, 0xFFFFFFFEu),
              t::ElementsAre(kIcrX2ApicInit, kIcrStartupAt0x8000));

  std::atomic<int> high_apic_seen{0};
  SmpRunOnAllCpus(
      [](const int cpu_index, void* const ctx) {
        if (SmpGetCpuLocal(cpu_index)->apic_id >= 0xFFu &&
            CurrentCpu()->apic_id == SmpGetCpuLocal(cpu_index)->apic_id) {
          static_cast<std::atomic<int>*>(ctx)->fetch_add(
              1, std::memory_order_acq_rel);
        }
      },
      &high_apic_seen);
  EXPECT_THAT(high_apic_seen.load(std::memory_order_acquire), t::Eq(4));
}

TEST_F(SmpTest, PreconditionViolationsTriggerDcheck) {
  ResetState();
  constexpr uintptr_t kAddr = 0x100;
  constexpr const char* kZeroMaxPhysicalAddr =
      "Check failed: max_physical_addr > 0";
  constexpr const char* kInvalidBspApicId =
      "Check failed: bsp_apic_id != kInvalidX2ApicId";

  EXPECT_DEATH(SmpFindRsdpInMultiboot2ForTest(0, kIdentityMapLimit),
               "Check failed: multiboot_info_addr != 0");
  EXPECT_DEATH(SmpFindRsdpInMultiboot2ForTest(kAddr, 0), kZeroMaxPhysicalAddr);

  EXPECT_DEATH(SmpFindRsdpInMemoryRange(0, 1024),
               "Check failed: range_start != 0");
  EXPECT_DEATH(SmpFindRsdpInMemoryRange(kAddr, 0),
               "Check failed: range_length > 0");
  EXPECT_DEATH(SmpFindRsdpInMemoryRange(kAddr, -16),
               "Check failed: range_length > 0");

  EXPECT_DEATH((void)SmpParseRsdpForTest(0, kIdentityMapLimit),
               "Check failed: rsdp_addr != 0");
  EXPECT_DEATH((void)SmpParseRsdpForTest(kAddr, 0), kZeroMaxPhysicalAddr);

  EXPECT_DEATH(SmpFindMadtInSdt(0, 0, kIdentityMapLimit),
               "Check failed: rsdt_addr != 0 \\|\\| xsdt_addr != 0");
  EXPECT_DEATH(SmpFindMadtInSdt(kAddr, 0, 0), kZeroMaxPhysicalAddr);

  EXPECT_DEATH((void)SmpParseMadt(0, kIdentityMapLimit, 0),
               "Check failed: madt_addr != 0");
  EXPECT_DEATH((void)SmpParseMadt(kAddr, 0, 0), kZeroMaxPhysicalAddr);
  EXPECT_DEATH((void)SmpParseMadt(kAddr, kIdentityMapLimit, kInvalidX2ApicId),
               kInvalidBspApicId);

  EXPECT_DEATH((void)SmpDiscoverTopology(kMultiboot2Magic, 0, 0, 0, 0, 0),
               kZeroMaxPhysicalAddr);
  EXPECT_DEATH((void)SmpDiscoverTopology(kMultiboot2Magic,   //
                                         0,                  //
                                         0,                  //
                                         0,                  //
                                         kIdentityMapLimit,  //
                                         kInvalidX2ApicId),
               kInvalidBspApicId);

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
  EXPECT_DEATH(SmpSetHardware(nullptr), "Check failed: hardware != nullptr");
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

TEST_F(SmpTest, CpuLocalLayoutAlignmentAndThreadLocalBinding) {
  static_assert(alignof(CpuLocal) == 64, "CpuLocal must be 64-byte aligned");
  static_assert(offsetof(CpuLocal, self) == 0,
                "CpuLocal::self must be at offset 0 for gs:[0] access");

  ResetState();
  EXPECT_THAT(CurrentCpuOrNull(), t::IsNull());

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

TEST_F(SmpTest, CalibratedTimerCountAndPerCpuStateAndPreconditions) {
  EXPECT_THAT(ComputeApicTimerInitialCount(0, 11932, 100), t::Eq(0u));
  EXPECT_THAT(ComputeApicTimerInitialCount(50000, 11932, 100), t::Eq(49999u));
  EXPECT_THAT(ComputeApicTimerInitialCount(0xFFFFFFFFu, 1, 1),
              t::Eq(0xFFFFFFFFu));
  EXPECT_THAT(ComputeApicTimerInitialCount(UINT64_MAX, 11932, 250),
              t::Eq(0xFFFFFFFFu));

  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, 2, 1);
  AppendLapicRecord(&recs, 1, 5, 1);
  FakeSmpHardware::Options options = DefaultOptions();
  options.bsp_apic_id = 2;
  options.lapic_timer_calibration_ticks = 60000;
  InitSmp(recs, options);

  const uint32_t expected_initial = ComputeApicTimerInitialCount(
      60000, kPitCalibrationReloadCount, kApicTimerTargetHz);
  EXPECT_THAT(SmpCalibratedTimerInitialCount(), t::Eq(expected_initial));

  for (int i = 0; i < SmpCpuCount(); ++i) {
    const CpuLocal* const cpu_local = SmpGetCpuLocal(i);
    ASSERT_THAT(cpu_local, t::NotNull());
    EXPECT_THAT(cpu_local->self, t::Eq(cpu_local));
    EXPECT_THAT(cpu_local->cpu_id, t::Eq(i));
    EXPECT_THAT(cpu_local->apic_id, t::Eq(SmpGetCpuInfo(i)->apic_id));
    EXPECT_THAT(cpu_local->stack_base, t::Eq(SmpGetCpuInfo(i)->stack_base));
    EXPECT_THAT(cpu_local->stack_top, t::Eq(SmpGetCpuInfo(i)->stack_top));
    EXPECT_TRUE(cpu_local->online);
  }
  EXPECT_THAT(CurrentCpu(), t::Eq(SmpGetCpuLocal(0)));
  EXPECT_THAT(CurrentCpuId(), t::Eq(0));

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

TEST_F(SmpTest, TimerCountFallsBackToCpuidClockThenToFixedCountWithoutPit) {
  struct Case {
    const char* name;
    // Local APIC timer ticks during the PIT countdown (0: the timer did not
    // count).
    uint32_t calibration_ticks;
    bool pit_responds;
    uint32_t max_leaf;
    uint32_t leaf15_eax;
    uint32_t leaf15_ebx;
    uint32_t leaf15_ecx;
    uint32_t leaf16_eax;
    uint32_t leaf16_ecx;
    uint32_t expected_count;
    // Expected fallback message, or nullptr if PIT calibration worked.
    const char* expected_log;
  };
  const Case kCases[] = {
      {"PIT calibration", 60000, true, 0x16, 2, 200, 24000000, 2400, 100,
       ComputeApicTimerInitialCount(60000, kPitCalibrationReloadCount,
                                    kApicTimerTargetHz),
       nullptr},
      // 24 MHz core crystal clock / 16 / 250 Hz.
      {"CPUID crystal clock", 0, true, 0x15, 2, 200, 24000000, 0, 0, 6000,
       "[SMP] No PIT calibration; Local APIC timer clock from CPUID: 24000000 "
       "Hz"},
      // Without a TSC/crystal ratio in leaf 0x15, the timer runs at the
      // 100 MHz bus clock of leaf 0x16.
      {"CPUID bus clock", 0, true, 0x16, 2, 0, 24000000, 2400, 100, 25000,
       "[SMP] No PIT calibration; Local APIC timer clock from CPUID: "
       "100000000 Hz"},
      // Leaf 0x15 says the timer runs at the crystal clock, but does not
      // enumerate its frequency (Skylake to Coffee Lake client CPUs): it is
      // the 2.4 GHz base frequency of leaf 0x16 over the TSC/crystal ratio
      // 200/2.
      {"derived crystal clock", 0, true, 0x16, 2, 200, 0, 2400, 100, 6000,
       "[SMP] No PIT calibration; Local APIC timer clock from CPUID: 24000000 "
       "Hz"},
      // ... which is unknown without the base frequency.
      {"unknown crystal clock", 0, true, 0x16, 2, 200, 0, 0, 100,
       kFallbackApicTimerInitialCount,
       "[SMP] WARNING: no PIT or CPUID timer frequency"},
      // PIT channel 2 never reaches terminal count.
      {"no PIT and no CPUID data", 60000, false, 0x0B, 1, 0, 0, 0, 0,
       kFallbackApicTimerInitialCount,
       "[SMP] WARNING: no PIT or CPUID timer frequency; LAPIC timer rate is "
       "uncalibrated"},
  };
  for (const Case& c : kCases) {
    SCOPED_TRACE(c.name);
    FakeSmpHardware::Options options = DefaultOptions();
    options.lapic_timer_calibration_ticks = c.calibration_ticks;
    options.pit_responds = c.pit_responds;
    options.cpuid_max_leaf = c.max_leaf;
    options.cpuid_leaf15_eax = c.leaf15_eax;
    options.cpuid_leaf15_ebx = c.leaf15_ebx;
    options.cpuid_leaf15_ecx = c.leaf15_ecx;
    options.cpuid_leaf16_eax = c.leaf16_eax;
    options.cpuid_leaf16_ecx = c.leaf16_ecx;
    InitSmp(LapicRecords(1), options);
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

TEST_F(SmpTest, TimerTickBoundsFollowTheCalibratedTimerClock) {
  struct Case {
    const char* name;
    // Local APIC timer ticks during the PIT countdown (0: no PIT).
    uint32_t calibration_ticks;
    // The timer clock that CPUID (leaf 0x15) or Hyper-V report (0: none).
    uint32_t cpuid_crystal_hz;
    uint64_t hyperv_apic_timer_hz;
  };
  const Case kCases[] = {
      // PIT calibration of the 1 GHz clock of QEMU and KVM, and of a 24 MHz
      // crystal clock.
      {"PIT, 1 GHz", 625000, 0, 0},
      {"PIT, 24 MHz", 15000, 0, 0},
      {"CPUID crystal clock, 24 MHz", 0, 24000000, 0},
      {"Hyper-V, 200 MHz", 0, 0, 200000000},
  };
  for (const Case& c : kCases) {
    SCOPED_TRACE(c.name);
    FakeSmpHardware::Options options = DefaultOptions();
    options.pit_present = c.calibration_ticks != 0;
    options.lapic_timer_calibration_ticks = c.calibration_ticks;
    options.cpuid_max_leaf = 0x15;
    options.cpuid_leaf15_eax = 2;
    options.cpuid_leaf15_ebx = (c.cpuid_crystal_hz != 0) ? 200 : 0;
    options.cpuid_leaf15_ecx = c.cpuid_crystal_hz;
    options.hyperv_apic_timer_hz = c.hyperv_apic_timer_hz;
    InitSmp(LapicRecords(1), options);
    // The timer clock: over the PIT countdown, which lasts
    // kPitCalibrationReloadCount ticks of the PIT, it counted the calibration
    // ticks at 1/16 of its rate.
    const int64_t clock_hz =
        (c.calibration_ticks != 0)
            ? int64_t{c.calibration_ticks} * 16 * kPitBaseFrequencyHz /
                  kPitCalibrationReloadCount
            : static_cast<int64_t>(c.cpuid_crystal_hz + c.hyperv_apic_timer_hz);
    const uint32_t count = SmpCalibratedTimerInitialCount();
    ASSERT_THAT(count, t::Ne(kFallbackApicTimerInitialCount));
    // The bounds bracket the length of a tick of `count` at that clock, and
    // stay within 2% of the nominal 4 ms whatever the clock.
    const int64_t tick_ns = 16LL * count * 1000000000 / clock_hz;
    EXPECT_GE(SmpTimerTickNsUpperBound(), tick_ns);
    EXPECT_LE(SmpTimerTickNsLowerBound(), tick_ns);
    EXPECT_LE(SmpTimerTickNsUpperBound(), 4080000);
    EXPECT_GE(SmpTimerTickNsLowerBound(), 3920000);
  }

  // Before SmpInit(), and with the uncalibrated fallback count, a tick lasts
  // from 4 ms (at the 1 GHz timer clock of QEMU and KVM) to 400 ms (10 MHz).
  ResetState();
  EXPECT_GE(SmpTimerTickNsUpperBound(), 400000000);
  EXPECT_LE(SmpTimerTickNsLowerBound(), 4000000);
  EXPECT_GE(SmpTimerTickNsLowerBound(), 3990000);
  FakeSmpHardware::Options options = DefaultOptions();
  options.pit_present = false;
  InitSmp(LapicRecords(1), options);
  ASSERT_THAT(SmpCalibratedTimerInitialCount(),
              t::Eq(kFallbackApicTimerInitialCount));
  EXPECT_GE(SmpTimerTickNsUpperBound(), 400000000);
  EXPECT_LE(SmpTimerTickNsLowerBound(), 4000000);
  EXPECT_GE(SmpTimerTickNsLowerBound(), 3990000);
}

TEST_F(SmpTest, CalibrateApicTimerCountBoundedLoopGuardAndZeroOnFailure) {
  static int g_speaker_polls = 0;
  static int g_ready_after_poll = 0;
  static uint32_t g_mock_current_count = 0;
  static uint64_t g_mock_clock = 0;
  g_speaker_polls = 0;
  g_ready_after_poll = 0;
  g_mock_current_count = 0;
  g_mock_clock = 0;

  const auto read_speaker_timeout = +[]() -> uint8_t {
    ++g_speaker_polls;
    return 0x00;  // Bit 5 (0x20) is never set.
  };
  const auto read_apic_count =
      +[](const uintptr_t) -> uint32_t { return g_mock_current_count; };
  // A clock that advances by 10 ticks with every read.
  const auto read_clock = +[]() -> uint64_t {
    const uint64_t now = g_mock_clock;
    g_mock_clock += 10;
    return now;
  };
  constexpr uint64_t kNoClockLimit = UINT64_MAX;

  // 1. Timeout when PIT port 0x61 bit 5 never sets: bounded to max_polls and
  // returns 0 without hanging.
  g_speaker_polls = 0;
  g_mock_current_count = 0xFFFFFFFFu - 50000u;
  const uint32_t timeout_count =
      CalibrateApicTimerCount(kDefaultLocalApicPhysAddr,  //
                              256,                        //
                              kNoClockLimit,              //
                              read_speaker_timeout,       //
                              read_clock,                 //
                              read_apic_count);
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
  const uint32_t calibrated_count =
      CalibrateApicTimerCount(kDefaultLocalApicPhysAddr,  //
                              256,                        //
                              kNoClockLimit,              //
                              read_speaker_ready,         //
                              read_clock,                 //
                              read_apic_count);
  EXPECT_THAT(g_speaker_polls, t::Eq(7));
  EXPECT_THAT(calibrated_count,
              t::Eq(ComputeApicTimerInitialCount(
                  60000, kPitCalibrationReloadCount, kApicTimerTargetHz)));

  // 3. Zero elapsed APIC ticks even when terminal count reached returns 0.
  g_speaker_polls = 0;
  g_ready_after_poll = 1;
  g_mock_current_count = 0xFFFFFFFFu;
  const uint32_t zero_elapsed_count =
      CalibrateApicTimerCount(kDefaultLocalApicPhysAddr,  //
                              256,                        //
                              kNoClockLimit,              //
                              read_speaker_ready,         //
                              read_clock,                 //
                              read_apic_count);
  EXPECT_THAT(zero_elapsed_count, t::Eq(0u));

  // 4. The poll also gives up once the clock has advanced by max_clock_ticks
  // (here after 100 polls), however many polls max_polls allows: each poll
  // can take microseconds where port reads exit to a hypervisor.
  g_speaker_polls = 0;
  g_mock_current_count = 0xFFFFFFFFu - 50000u;
  const uint32_t clock_timeout_count =
      CalibrateApicTimerCount(kDefaultLocalApicPhysAddr,  //
                              kPitCalibrationPollLimit,   //
                              1000,                       //
                              read_speaker_timeout,       //
                              read_clock,                 //
                              read_apic_count);
  EXPECT_THAT(g_speaker_polls, t::Eq(100));
  EXPECT_THAT(clock_timeout_count, t::Eq(0u));
  // A countdown that runs out at the last poll still counts.
  g_speaker_polls = 0;
  g_ready_after_poll = 100;
  g_mock_current_count = 0xFFFFFFFFu - 60000u;
  const uint32_t last_poll_count =
      CalibrateApicTimerCount(kDefaultLocalApicPhysAddr,  //
                              kPitCalibrationPollLimit,   //
                              1000,                       //
                              read_speaker_ready,         //
                              read_clock,                 //
                              read_apic_count);
  EXPECT_THAT(g_speaker_polls, t::Eq(100));
  EXPECT_THAT(last_poll_count, t::Eq(calibrated_count));
}

TEST_F(SmpTest, StopOtherCpusForPanicSendsAnNmiToEachOtherOnlineCpu) {
  // Before SmpInit has enabled the Local APIC, there is nothing to stop.
  const FakeSmpHardware& uninitialized_hw = ResetState(X2ApicOptions());
  SmpStopOtherCpusForPanic();
  EXPECT_THAT(uninitialized_hw.Events(), t::IsEmpty());

  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, 0, 1);
  AppendLapicRecord(&recs, 1, 3, 1);
  AppendLapicRecord(&recs, 2, 4, 1);
  AppendLapicRecord(&recs, 3, 5, 1);
  AppendLapicRecord(&recs, 4, 6, 1);
  for (const bool x2apic : {false, true}) {
    SCOPED_TRACE(x2apic ? "x2APIC" : "xAPIC");
    FakeSmpHardware& hw =
        ResetState(x2apic ? X2ApicOptions() : DefaultOptions());
    // The CPU with APIC ID 4 never starts: on hardware, it may still run a
    // firmware loop, whose IDT is gone.
    hw.SetApStart(4, FakeSmpHardware::ApStart::kNever);
    WriteBiosAcpiTables(recs);
    IdtInit();
    SmpInit(kMultiboot1Magic, 0);
    ASSERT_THAT(SmpIsX2ApicEnabled(), t::Eq(x2apic));
    ASSERT_THAT(SmpCpuCount(), t::Eq(4));

    // One NMI (delivery mode 0x400, level assert, no destination shorthand)
    // to each other CPU that runs the kernel, by APIC ID, and none to the CPU
    // that did not start.
    const auto nmis_sent_from = [&hw](const int cpu) {
      const int ipi_count = static_cast<int>(hw.Ipis().size());
      BindCpuLocal(SmpGetCpuLocal(cpu));
      SmpStopOtherCpusForPanic();
      BindCpuLocal(SmpGetCpuLocal(0));
      const std::vector<Event> ipis = hw.Ipis();
      const int total_ipis = static_cast<int>(ipis.size());
      std::vector<uint64_t> nmis;
      for (int i = ipi_count; i < total_ipis; ++i) {
        nmis.push_back(ipis[i].value);
      }
      return nmis;
    };
    const auto nmi_to = [](const uint64_t apic_id) {
      return (apic_id << 32) | kIcrNmi;
    };
    const int write_count = static_cast<int>(RegisterWrites(hw, 0).size());
    EXPECT_THAT(nmis_sent_from(0),
                t::ElementsAre(nmi_to(3), nmi_to(5), nmi_to(6)));
    if (!x2apic) {
      // The destination in ICR high, then ICR low.
      const std::vector<RegisterWrite> writes = RegisterWrites(hw, 0);
      EXPECT_THAT(std::vector<RegisterWrite>(writes.begin() + write_count,
                                             writes.end()),
                  t::ElementsAre(t::Pair(kLapicIcrHigh, 3u << 24),
                                 t::Pair(kLapicIcrLow, kIcrNmi),
                                 t::Pair(kLapicIcrHigh, 5u << 24),
                                 t::Pair(kLapicIcrLow, kIcrNmi),
                                 t::Pair(kLapicIcrHigh, 6u << 24),
                                 t::Pair(kLapicIcrLow, kIcrNmi)));
    }
    // An AP that panics stops the BSP and the other APs.
    EXPECT_THAT(nmis_sent_from(2),
                t::ElementsAre(nmi_to(0), nmi_to(3), nmi_to(6)));
  }
}

TEST_F(SmpTest, PanicStopNmiParksEveryCpuButThePanickingOne) {
  InitSmp(LapicRecords(2));
  SmpStopOtherCpusForPanic();
  InterruptFrame frame = {};
  frame.vector = kVectorNmi;
  // An NMI that reaches the panicking CPU itself (from the hardware, or a
  // self-test that sends one) does not park it: it must go on to print the
  // panic message.
  SetInterruptsEnabledForTest(false);
  IdtDispatch(&frame);
  SetInterruptsEnabledForTest(true);
  // Any other CPU parks for good.
  EXPECT_DEATH(
      {
        BindCpuLocal(SmpGetCpuLocal(1));
        SetInterruptsEnabledForTest(false);
        IdtDispatch(&frame);
      },
      "FakeSmpHardware: a CPU halted for good");
}

TEST_F(SmpTest, PanicDuringApBringUpSendsNoNmiToCpusThatAreNotOnline) {
  // Another CPU panics while SmpInit() allocates the AP stacks, before any AP
  // runs kernel code. The panic sends no NMI to the APs, which may still run
  // a firmware loop (here, the AP never starts).
  FakeSmpHardware& hw = ResetState();
  hw.SetApStart(1, FakeSmpHardware::ApStart::kNever);
  WriteBiosAcpiTables(LapicRecords(2));
  IdtInit();
  g_state.panic_stop_on_pmm_call = 1;
  SmpInit(kMultiboot1Magic, 0);
  ASSERT_THAT(SmpCpuCount(), t::Eq(1));
  // Only the INIT-SIPI-SIPI sequence and the INIT after the startup timeout.
  EXPECT_THAT(IcrsTo(hw, 1),
              t::ElementsAre(kIcrXapicInitAssert, kIcrXapicInitDeassert,
                             kIcrStartupAt0x8000, kIcrStartupAt0x8000,
                             kIcrXapicInitAssert, kIcrXapicInitDeassert));
  EXPECT_THAT(hw.Ipis().size(), t::Eq(6u));

  // An AP that starts after the panic parks itself as soon as it is online.
  ResetState();
  WriteBiosAcpiTables(LapicRecords(2));
  IdtInit();
  g_state.panic_stop_on_pmm_call = 1;
  EXPECT_DEATH(SmpInit(kMultiboot1Magic, 0),
               "FakeSmpHardware: a CPU halted for good");
}

TEST_F(SmpTest, KvmStealTimeRegistrationAndQueriesOnBspAndAps) {
  // Without KVM steal-time CPUID feature, SmpHasStealTime() is false,
  // SmpOwnStealNsIrqsOff() and SmpStealNsOfCpu() return 0, and MSR 0x4b564d03
  // is never written.
  {
    const FakeSmpHardware& hw = InitSmp(LapicRecords(2));
    EXPECT_FALSE(SmpHasStealTime());
    SetInterruptsEnabledForTest(false);
    EXPECT_THAT(SmpOwnStealNsIrqsOff(), t::Eq(0));
    SetInterruptsEnabledForTest(true);
    EXPECT_THAT(SmpStealNsOfCpu(0), t::Eq(0));
    EXPECT_THAT(SmpStealNsOfCpu(1), t::Eq(0));
    EXPECT_THAT(g_state.uart_log, t::HasSubstr("[SMP] Steal time: none"));
    for (const Event& ev : hw.Events()) {
      if (ev.kind == Event::Kind::kMsrWrite) {
        EXPECT_THAT(ev.reg, t::Ne(kMsrKvmStealTime));
      }
    }
  }

  // With KVM steal-time exposed at 0x40000100 (e.g. when Hyper-V occupies
  // 0x40000000), both BSP and APs write MSR_KVM_STEAL_TIME with the 64-byte
  // aligned address of their CpuLocal::kvm_steal_time |
  // kKvmStealTimeSystemEnabled.
  FakeSmpHardware::Options options = DefaultOptions();
  options.kvm_base_leaf = 0x40000100;
  options.kvm_steal_time = true;
  const FakeSmpHardware& hw = InitSmp(LapicRecords(3), options);
  EXPECT_TRUE(SmpHasStealTime());
  EXPECT_THAT(g_state.uart_log, t::HasSubstr("[SMP] Steal time: KVM"));

  for (int cpu = 0; cpu < 3; ++cpu) {
    SCOPED_TRACE(cpu);
    CpuLocal* const local = SmpGetCpuLocal(cpu);
    ASSERT_THAT(local, t::NotNull());
    const uint64_t expected_msr =
        reinterpret_cast<uintptr_t>(&local->kvm_steal_time) |
        kKvmStealTimeSystemEnabled;
    EXPECT_THAT(MsrWrites(hw, cpu),
                t::Contains(t::Pair(kMsrKvmStealTime, expected_msr)));

    local->kvm_steal_time.version = 2;
    local->kvm_steal_time.steal = 4000000ULL * (cpu + 1);
    EXPECT_THAT(SmpStealNsOfCpu(cpu), t::Eq(4000000LL * (cpu + 1)));
  }

  // Out-of-bounds CPU returns -1.
  EXPECT_THAT(SmpStealNsOfCpu(-1), t::Eq(-1));
  EXPECT_THAT(SmpStealNsOfCpu(3), t::Eq(-1));

  // SmpOwnStealNsIrqsOff reads the bound current CPU's steal time.
  BindCpuLocal(SmpGetCpuLocal(1));
  SetInterruptsEnabledForTest(false);
  EXPECT_THAT(SmpOwnStealNsIrqsOff(), t::Eq(8000000));
  SetInterruptsEnabledForTest(true);
  BindCpuLocal(SmpGetCpuLocal(0));

  // Timer tick bounds are positive and conservatively bracket 4,000,000 ns.
  EXPECT_GT(SmpTimerTickNsUpperBound(), 4000000);
  EXPECT_LT(SmpTimerTickNsLowerBound(), 4000000);
  EXPECT_GT(SmpTimerTickNsLowerBound(), 0);
}

}  // namespace
}  // namespace protos
