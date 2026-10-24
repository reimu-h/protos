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
// Whether a UART answered the probe in UartInit(). Only UartInit() changes it.
bool g_uart_present = false;
// Set when a byte timed out waiting for the transmitter, cleared once the
// transmitter is seen ready again. While it is set, each byte gets a single
// poll instead of the whole poll window and is dropped if the transmitter is
// still busy: a UART whose transmitter is not clocked (an unclocked Super I/O
// UART, for example) must not cost every byte a timeout, and one whose output
// is merely stalled must resume as soon as it drains. The latter happens in
// QEMU, whose 16550 keeps LSR.THRE clear while the host side of the serial
// port does not accept data (a terminal or SSH session that falls behind with
// -serial mon:stdio, or a socket client that stops reading).
bool g_uart_tx_stalled = false;
// Whether UartPanicWrite() has run.
bool g_uart_panic_started = false;
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
  // A panic may have interrupted this CPU in a critical section of the
  // receive buffer: leave the buffer alone then.
  const bool lock_held =
      !g_uart_panic_started && g_uart_lock.IsLockedByCurrentCpu();
  if (lock_held) {
    UartDrainRxLocked();
  }
  const int max_polls = g_uart_tx_stalled ? 1 : kMaxTransmitPollIterations;
  for (int spin = 0; spin < max_polls; ++spin) {
    if ((Inb(kCom1Port + 5) & 0x20) != 0) {
      Outb(kCom1Port, c);
      g_uart_tx_stalled = false;
      return;
    }
    if (lock_held) {
      UartDrainRxLocked();
    }
    asm volatile("pause");
  }
  // The transmit holding register did not empty: drop the byte, but keep the
  // UART, and its receiver, in use (see g_uart_tx_stalled).
  g_uart_tx_stalled = true;
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

  // Presence probe, as Linux's 8250 driver does it: the low four bits of the
  // Interrupt Enable Register (DLAB is clear now) read back what was written,
  // while a port that nothing decodes reads as 0xFF. Absence is decided here
  // only; a transmitter that stops later is not taken for a missing UART (see
  // g_uart_tx_stalled). The interrupt request this may raise goes nowhere: the
  // kernel masks every 8259 PIC line and uses no IRQ of the UART.
  Outb(kCom1Port + 1, 0x00);
  const uint8_t ier_cleared = Inb(kCom1Port + 1) & 0x0F;
  Outb(kCom1Port + 1, 0x0F);
  const uint8_t ier_set = Inb(kCom1Port + 1) & 0x0F;
  Outb(kCom1Port + 1, 0x00);

  Outb(kCom1Port + 2, 0xC7);
  Outb(kCom1Port + 4, 0x0B);

  g_uart_present = ier_cleared == 0x00 && ier_set == 0x0F;
  g_uart_tx_stalled = false;
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

void UartPanicWrite(const char* const str) {
  // The other CPUs are stopped, so the UART state is used without the lock.
  // The panic's first byte gets the whole poll window even if the transmitter
  // has stalled (see g_uart_tx_stalled), so that a stall that clears within it
  // does not cost the panic message. An unclocked UART costs one window per
  // panic.
  if (!g_uart_panic_started) {
    g_uart_panic_started = true;
    g_uart_tx_stalled = false;
  }
  UartWriteLocked(str);
}

StatusOr<char> UartTryGetc() {
  const IrqSpinLockGuard lock_guard(g_uart_lock);
  UartDrainRxLocked();
  if (g_uart_rx_count <= 0) {
    return kStatusError;
  }
  const char ch = g_uart_rx_buf[g_uart_rx_head];
  g_uart_rx_head = (g_uart_rx_head + 1) % kUartRxBufferSize;
  --g_uart_rx_count;
  return ch;
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
