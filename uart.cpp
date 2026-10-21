#include "uart.h"

#include <cstdint>

#include "format.h"
#include "spinlock.h"

namespace protos {

namespace {

constexpr uint16_t kCom1Port = 0x3F8;
constexpr int kMaxTransmitPollIterations = 100000;
constexpr int64_t kUartRxBufferSize = 4096;

IrqSpinLock g_uart_lock;
bool g_uart_present = false;
char g_uart_rx_buf[kUartRxBufferSize];
int64_t g_uart_rx_head = 0;
int64_t g_uart_rx_tail = 0;
int64_t g_uart_rx_count = 0;

static inline void Outb(const uint16_t port, const uint8_t value) {
  asm volatile("out %1, %0" : : "a"(value), "Nd"(port) : "memory");
}

static inline uint8_t Inb(const uint16_t port) {
  uint8_t value = 0;
  asm volatile("in %0, %1" : "=a"(value) : "Nd"(port) : "memory");
  return value;
}

static void UartPushRxByteLocked(const char ch) {
  if (g_uart_rx_count >= kUartRxBufferSize) {
    return;
  }
  g_uart_rx_buf[g_uart_rx_tail] = ch;
  g_uart_rx_tail = (g_uart_rx_tail + 1) % kUartRxBufferSize;
  ++g_uart_rx_count;
}

static void UartDrainRxLocked() {
  if (!g_uart_present) {
    return;
  }
  for (int64_t i = 0; i < kUartRxBufferSize; ++i) {
    const uint8_t lsr = Inb(kCom1Port + 5);
    if (lsr == 0xFF || (lsr & 0x01) == 0) {
      break;
    }
    const char ch = static_cast<char>(Inb(kCom1Port + 0));
    UartPushRxByteLocked(ch);
  }
}

static void UartPutcLocked(const char c) {
  if (!g_uart_present) {
    return;
  }
  const bool lock_held = g_uart_lock.IsLockedByCurrentCpu();
  if (lock_held) {
    UartDrainRxLocked();
  }
  for (int spin = 0; spin < kMaxTransmitPollIterations; ++spin) {
    if ((Inb(kCom1Port + 5) & 0x20) != 0) {
      Outb(kCom1Port, c);
      return;
    }
    if (lock_held) {
      UartDrainRxLocked();
    }
    asm volatile("pause");
  }
  // Transmit holding register never became ready (e.g., unclocked Super I/O
  // UART); disable subsequent UART polling to avoid stalling boot.
  g_uart_present = false;
}

static void UartWriteLocked(const char* const str) {
  if (str == nullptr) {
    return;
  }
  for (int i = 0; str[i] != '\0'; ++i) {
    if (str[i] == '\n') {
      UartPutcLocked('\r');
    }
    UartPutcLocked(str[i]);
  }
}

}  // namespace

void UartInit() {
  const IrqSpinLockGuard lock_guard(g_uart_lock);
  Outb(kCom1Port + 1, 0x00);
  Outb(kCom1Port + 3, 0x80);
  Outb(kCom1Port + 0, 0x03);
  Outb(kCom1Port + 1, 0x00);
  Outb(kCom1Port + 3, 0x03);
  Outb(kCom1Port + 2, 0xC7);
  Outb(kCom1Port + 4, 0x0B);

  // If the Line Status Register floats high (0xFF), no UART is decoding 0x3F8.
  g_uart_present = (Inb(kCom1Port + 5) != 0xFF);
  g_uart_rx_head = 0;
  g_uart_rx_tail = 0;
  g_uart_rx_count = 0;
}

void UartPutc(const char c) {
  const IrqSpinLockGuard lock_guard(g_uart_lock);
  UartPutcLocked(c);
}

void UartWrite(const char* const str) {
  const IrqSpinLockGuard lock_guard(g_uart_lock);
  UartWriteLocked(str);
}

void UartWriteHex(const uint64_t value) {
  char buffer[kFormatHexBufferSize];
  UartWrite(FormatHex(value, buffer));
}

void UartWriteDec(const uint64_t value) {
  char buffer[kFormatDecBufferSize];
  UartWrite(FormatDec(value, buffer));
}

void UartPanicWrite(const char* const str) { UartWriteLocked(str); }

bool UartTryGetc(char* const out) {
  if (out == nullptr) {
    return false;
  }
  const IrqSpinLockGuard lock_guard(g_uart_lock);
  UartDrainRxLocked();
  if (g_uart_rx_count <= 0) {
    return false;
  }
  *out = g_uart_rx_buf[g_uart_rx_head];
  g_uart_rx_head = (g_uart_rx_head + 1) % kUartRxBufferSize;
  --g_uart_rx_count;
  return true;
}

void UartPushRx(const char* const data, const int64_t size) {
  if (data == nullptr || size <= 0) {
    return;
  }
  const IrqSpinLockGuard lock_guard(g_uart_lock);
  for (int64_t i = 0; i < size; ++i) {
    UartPushRxByteLocked(data[i]);
  }
}

void UartClearRx() {
  const IrqSpinLockGuard lock_guard(g_uart_lock);
  UartDrainRxLocked();
  g_uart_rx_head = 0;
  g_uart_rx_tail = 0;
  g_uart_rx_count = 0;
}

}  // namespace protos
