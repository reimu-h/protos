#include "uart.h"

#include <cstdint>

#include "format.h"
#include "spinlock.h"

namespace protos {

namespace {

constexpr uint16_t kCom1Port = 0x3F8;
constexpr int kMaxTransmitPollIterations = 100000;

IrqSpinLock g_uart_lock;
bool g_uart_present = false;

static inline void Outb(const uint16_t port, const uint8_t value) {
  asm volatile("out %1, %0" : : "a"(value), "Nd"(port) : "memory");
}

static inline uint8_t Inb(const uint16_t port) {
  uint8_t value = 0;
  asm volatile("in %0, %1" : "=a"(value) : "Nd"(port) : "memory");
  return value;
}

static void UartPutcLocked(const char c) {
  if (!g_uart_present) {
    return;
  }
  for (int spin = 0; spin < kMaxTransmitPollIterations; ++spin) {
    if ((Inb(kCom1Port + 5) & 0x20) != 0) {
      Outb(kCom1Port, c);
      return;
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

}  // namespace protos
