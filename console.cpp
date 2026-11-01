#include "console.h"

#include <atomic>
#include <cstdint>

#include "format.h"
#include "spinlock.h"
#include "uart.h"
#include "vga.h"

namespace protos {

namespace {

constexpr int64_t kConsoleCaptureBufferSize = 8192;

IrqSpinLock g_console_lock;
std::atomic<bool> g_vga_echo_enabled{true};
std::atomic<void (*)()> g_input_wake_callback{nullptr};
char g_capture_buf[kConsoleCaptureBufferSize];
int64_t g_capture_head = 0;
int64_t g_capture_tail = 0;
int64_t g_capture_count = 0;

static void CaptureAppendLocked(const char* const str) {
  if (str == nullptr) {
    return;
  }
  for (int64_t i = 0; str[i] != '\0'; ++i) {
    g_capture_buf[g_capture_tail] = str[i];
    g_capture_tail = (g_capture_tail + 1) % kConsoleCaptureBufferSize;
    if (g_capture_count < kConsoleCaptureBufferSize) {
      ++g_capture_count;
    } else {
      g_capture_head = (g_capture_head + 1) % kConsoleCaptureBufferSize;
    }
  }
}

}  // namespace

#if __STDC_HOSTED__
// Default no-op UART RX stubs for host unit tests (multiboot_test, pmm_test,
// smp_test) that link console.cpp without uart.cpp. console_test.cpp overrides
// these with strong definitions.
[[gnu::weak, gnu::noinline]] StatusOr<char> UartTryGetc() {
  asm volatile("" : : : "memory");
  return kStatusError;
}

[[gnu::weak, gnu::noinline]] void UartPushRx(const char* const data,
                                             const int64_t size) {
  asm volatile("" : : "r"(data), "r"(size) : "memory");
}

[[gnu::weak, gnu::noinline]] void UartClearRx() {
  asm volatile("" : : : "memory");
}
#endif

void ConsoleWrite(const char* const str) {
  {
    const IrqSpinLockGuard lock_guard(g_console_lock);
    CaptureAppendLocked(str);
  }
  UartWrite(str);
  if (g_vga_echo_enabled.load(std::memory_order_acquire)) {
    VgaWrite(str);
  }
}

void ConsoleWriteHex(const uint64_t value) {
  char buffer[kFormatHexBufferSize];
  ConsoleWrite(FormatHex(value, buffer));
}

void ConsoleWriteDec(const uint64_t value) {
  char buffer[kFormatDecBufferSize];
  ConsoleWrite(FormatDec(value, buffer));
}

StatusOr<char> ConsoleTryReadc() { return UartTryGetc(); }

void ConsolePushInput(const char* const data, const int64_t size) {
  if (data == nullptr || size <= 0) {
    return;
  }
  UartPushRx(data, size);
  void (*const wake)() = g_input_wake_callback.load(std::memory_order_acquire);
  if (wake != nullptr) {
    wake();
  }
}

void ConsoleClearInput() { UartClearRx(); }

void ConsoleSetInputWakeCallback(void (*const callback)()) {
  g_input_wake_callback.store(callback, std::memory_order_release);
}

void ConsoleSetVgaEcho(const bool enabled) {
  g_vga_echo_enabled.store(enabled, std::memory_order_release);
}

bool ConsoleVgaEchoEnabled() {
  return g_vga_echo_enabled.load(std::memory_order_acquire);
}

void ConsoleResetOutputCapture() {
  const IrqSpinLockGuard lock_guard(g_console_lock);
  g_capture_head = 0;
  g_capture_tail = 0;
  g_capture_count = 0;
}

int64_t ConsoleCopyOutputCapture(char* const dst, const int64_t capacity) {
  if (dst == nullptr || capacity <= 0) {
    return 0;
  }
  const IrqSpinLockGuard lock_guard(g_console_lock);
  const int64_t max_copy = capacity - 1;
  const int64_t to_copy =
      (g_capture_count < max_copy) ? g_capture_count : max_copy;
  for (int64_t i = 0; i < to_copy; ++i) {
    const int64_t idx = (g_capture_head + i) % kConsoleCaptureBufferSize;
    dst[i] = g_capture_buf[idx];
  }
  dst[to_copy] = '\0';
  return to_copy;
}

}  // namespace protos
