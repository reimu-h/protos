// Host fakes for the PMM, heap, SMP, UART and VGA modules, used by the
// scheduler unit tests. See scheduler_test_support.h.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>

#include "check.h"
#include "heap.h"
#include "idt.h"
#include "pmm.h"
#include "scheduler_test_support.h"
#include "smp.h"
#include "uart.h"
#include "vga.h"

namespace protos {
namespace {

// Header placed in front of every fake Kmalloc() allocation.
struct HeapAllocHeader {
  int64_t payload_size = 0;
  void* raw_base = nullptr;
  uint64_t magic = 0;
};
constexpr uint64_t kHeapHeaderMagic = 0xC0DEC0DE12345678ULL;

static void AppendLog(std::string* const log, const char* const str) {
  if (str != nullptr) {
    const std::lock_guard<std::mutex> lock(g_env.log_mutex);
    log->append(str);
  }
}

static void AppendLogFormatted(std::string* const log,    //
                               const char* const format,  //
                               const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                format,       //
                static_cast<unsigned long long>(value));
  AppendLog(log, buf);
}

}  // namespace

uintptr_t PmmAllocFrames(const int64_t count) {
  DCHECK(count > 0);
  if (g_env.fail_pmm_alloc) {
    return 0;
  }
  const int64_t bytes = count * kPageSize;
  void* const mem = std::aligned_alloc(static_cast<size_t>(kPageSize),
                                       static_cast<size_t>(bytes));
  if (mem == nullptr) {
    return 0;
  }
  g_env.allocated_frames.fetch_add(count, std::memory_order_acq_rel);
  return reinterpret_cast<uintptr_t>(mem);
}

uintptr_t PmmAllocFrame() { return PmmAllocFrames(1); }

void PmmFreeFrames(const uintptr_t base_addr, const int64_t count) {
  DCHECK(base_addr != 0);
  DCHECK((base_addr & (kPageSize - 1)) == 0);
  DCHECK(count > 0);
  std::free(reinterpret_cast<void*>(base_addr));
  const int64_t prev =
      g_env.allocated_frames.fetch_sub(count, std::memory_order_acq_rel);
  DCHECK(prev >= count);
}

void PmmFreeFrame(const uintptr_t frame_addr) { PmmFreeFrames(frame_addr, 1); }

int64_t PmmFreeFrameCount() {
  return kInitialFakeFreeFrames -
         g_env.allocated_frames.load(std::memory_order_acquire);
}

void* KmallocAligned(const int64_t size, const int64_t alignment) {
  if (g_env.fail_kmalloc || size < 0 || alignment < 1 ||
      (alignment & (alignment - 1)) != 0 || alignment > kPageSize) {
    return nullptr;
  }
  const int64_t payload = (size > 0) ? size : kHeapAlignment;
  const int64_t align =
      (alignment > kHeapAlignment) ? alignment : kHeapAlignment;
  const int64_t header_size = static_cast<int64_t>(sizeof(HeapAllocHeader));
  const int64_t header_offset = (header_size + align - 1) & ~(align - 1);
  const int64_t total_bytes =
      (header_offset + payload + align - 1) & ~(align - 1);

  void* const raw = std::aligned_alloc(static_cast<size_t>(align),
                                       static_cast<size_t>(total_bytes));
  if (raw == nullptr) {
    return nullptr;
  }
  uint8_t* const user_ptr = static_cast<uint8_t*>(raw) + header_offset;
  HeapAllocHeader* const hdr =
      reinterpret_cast<HeapAllocHeader*>(user_ptr - sizeof(HeapAllocHeader));
  hdr->payload_size = payload;
  hdr->raw_base = raw;
  hdr->magic = kHeapHeaderMagic;

  g_env.allocated_heap_bytes.fetch_add(payload, std::memory_order_acq_rel);
  g_env.allocated_heap_blocks.fetch_add(1, std::memory_order_acq_rel);
  return user_ptr;
}

void* Kmalloc(const int64_t size) {
  return KmallocAligned(size, kHeapAlignment);
}

void Kfree(void* const ptr) {
  if (ptr == nullptr) {
    return;
  }
  uint8_t* const user_ptr = static_cast<uint8_t*>(ptr);
  HeapAllocHeader* const hdr =
      reinterpret_cast<HeapAllocHeader*>(user_ptr - sizeof(HeapAllocHeader));
  DCHECK(hdr->magic == kHeapHeaderMagic);
  const int64_t payload = hdr->payload_size;
  void* const raw = hdr->raw_base;
  hdr->magic = 0;
  std::free(raw);

  const int64_t prev_bytes =
      g_env.allocated_heap_bytes.fetch_sub(payload, std::memory_order_acq_rel);
  const int64_t prev_blocks =
      g_env.allocated_heap_blocks.fetch_sub(1, std::memory_order_acq_rel);
  DCHECK(prev_bytes >= payload);
  DCHECK(prev_blocks >= 1);
}

int64_t HeapTotalFreeBytes() {
  return kInitialFakeHeapBytes -
         g_env.allocated_heap_bytes.load(std::memory_order_acquire);
}

int SmpCpuCount() { return g_env.cpu_count; }

int SmpOnlineCpuCount() {
  int online = 0;
  for (int i = 0; i < g_env.cpu_count; ++i) {
    if (g_env.cpu_locals[i].online) {
      ++online;
    }
  }
  return online;
}

const CpuInfo* SmpGetCpuInfo(const int index) {
  DCHECK(index >= 0 && index < g_env.cpu_count);
  return &g_env.cpu_infos[index];
}

CpuLocal* SmpGetCpuLocal(const int cpu_index) {
  DCHECK(cpu_index >= 0 && cpu_index < g_env.cpu_count);
  if (g_env.null_cpu_local != nullptr && g_env.null_cpu_local[cpu_index]) {
    return nullptr;
  }
  return &g_env.cpu_locals[cpu_index];
}

void SmpSendLocalApicEoi() {
  g_env.eoi_count.fetch_add(1, std::memory_order_acq_rel);
}

void SmpSendIpi(const int target_cpu_index, const uint8_t vector) {
  DCHECK(target_cpu_index >= 0 && target_cpu_index < g_env.cpu_count);
  DCHECK(vector >= static_cast<uint8_t>(kCpuExceptionCount));
  g_env.ipi_sent_count[target_cpu_index].fetch_add(1,
                                                   std::memory_order_acq_rel);
  g_env.last_ipi_vector[target_cpu_index].store(vector,
                                                std::memory_order_release);
}

void UartWrite(const char* const str) { AppendLog(&g_env.uart_log, str); }

void UartWriteHex(const uint64_t value) {
  AppendLogFormatted(&g_env.uart_log, "0x%llX", value);
}

void UartWriteDec(const uint64_t value) {
  AppendLogFormatted(&g_env.uart_log, "%llu", value);
}

void UartPanicWrite(const char* const str) {
  AppendLog(&g_env.uart_panic_log, str);
}

void VgaWrite(const char* const str) { AppendLog(&g_env.vga_log, str); }

void VgaWriteHex(const uint64_t value) {
  AppendLogFormatted(&g_env.vga_log, "0x%llX", value);
}

void VgaWriteDec(const uint64_t value) {
  AppendLogFormatted(&g_env.vga_log, "%llu", value);
}

void VgaPanicWrite(const char* const str) {
  AppendLog(&g_env.vga_panic_log, str);
}

}  // namespace protos
