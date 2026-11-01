// ps2kbd: ring-3 Intel 8042 (`i8042`) PS/2 keyboard driver (ports 0x60/0x64,
// ISA IRQ 1 via IOAPIC).
//
// Claims `kIsaDeviceI8042`, initializes Port 1 without issuing `0xAA`
// controller self-test (preserving laptop embedded controllers and coexisting
// with `usbkbd`), allocates IOAPIC-routed ISA IRQ 1, enables IRQ 1 in the
// Controller Configuration Byte, and blocks indefinitely in `IrqWait(mask, -1)`
// between keystrokes while feeding US-layout console bytes via `ConsolePush()`.
//
// On systems without an `i8042` controller or IOAPIC IRQ 1 route (such as
// Hyper-V Generation 2 UEFI VMs), exits cleanly with status 0 in normal boot
// mode (`kDriverArgNormal`). Started without `kUserCapDriver` (e.g. `run
// ps2kbd` in `sh`), reports that and exits with `kDriverExitNotDriver`.

#include <cstdint>
#include <type_traits>

#include "syscall_abi.h"
#include "user/ps2kbd/i8042.h"
#include "user/syscall.h"

namespace protos::user {
namespace {

class Ring3Ps2Platform final : public ps2::Platform {
 public:
  constexpr Ring3Ps2Platform() = default;

  uint8_t In8(const uint16_t port) override {
    const StatusOr<uint8_t> val = IsaPortIn8(port);
    return val.ok() ? *val : ps2::kStatusFloatingBus;
  }

  void Out8(const uint16_t port, const uint8_t value) override {
    (void)IsaPortOut8(port, value);
  }

  int64_t NowTicks() override { return GetTicks(); }

  int64_t TicksPerSecond() override { return kSyscallTicksPerSecond; }

  void Relax() override { (void)IrqWait(/*mask=*/0, /*timeout_ticks=*/1); }

  void PushInput(const char* const bytes, const int count) override {
    if (bytes == nullptr || count <= 0) {
      return;
    }
    int offset = 0;
    while (offset < count) {
      int chunk = count - offset;
      if (chunk > static_cast<int>(kSyscallConsolePushMaxSize)) {
        chunk = static_cast<int>(kSyscallConsolePushMaxSize);
      }
      (void)ConsolePush(bytes + offset, chunk);
      offset += chunk;
    }
  }
};

constinit Ring3Ps2Platform g_platform{};
constinit ps2::I8042Controller g_controller{};
static_assert(std::is_trivially_destructible_v<decltype(g_platform)>);
static_assert(std::is_trivially_destructible_v<decltype(g_controller)>);

}  // namespace

int64_t Main(const uint64_t arg) {
  const int64_t claim_rc = IsaClaim(kIsaDeviceI8042);
  if (claim_rc == kSyscallErrorPermission) {
    (void)Print(
        "[PS2KBD] not started as a driver (init starts ps2kbd at boot)\n");
    return kDriverExitNotDriver;
  }
  if (claim_rc == kSyscallErrorNoDevice) {
    (void)Print("[PS2KBD] no i8042 controller found\n");
    return (arg == kDriverArgNormal) ? 0 : 1;
  }
  if (claim_rc != 0) {
    (void)Print("[PS2KBD] i8042: error: claim\n");
    return 1;
  }

  const ps2::I8042Status init_status = g_controller.Initialize(g_platform);
  if (init_status == ps2::I8042Status::kNotPresent) {
    (void)Print("[PS2KBD] no i8042 controller found\n");
    return (arg == kDriverArgNormal) ? 0 : 1;
  }
  if (init_status != ps2::I8042Status::kOk) {
    (void)Print("[PS2KBD] i8042: error: init\n");
    return 1;
  }

  const int64_t irq_bit = IsaIrqAlloc(kIsaDeviceI8042);
  if (irq_bit == kSyscallErrorNoDevice) {
    (void)Print("[PS2KBD] no i8042 controller found\n");
    return (arg == kDriverArgNormal) ? 0 : 1;
  }
  if (IsError(irq_bit) || irq_bit < 0 || irq_bit >= kSyscallIrqMaxCount) {
    (void)Print("[PS2KBD] i8042: error: irq_alloc\n");
    return 1;
  }

  if (g_controller.EnableIrq1() != ps2::I8042Status::kOk) {
    (void)Print("[PS2KBD] i8042: error: enable_irq\n");
    return 1;
  }

  (void)Print("[PS2KBD] controller i8042 ready (irq1)\n");

  if (arg == kDriverArgExitAfterReady || arg == kDriverArgFaultAfterReady) {
    g_controller.HandleInterrupt();
    if (arg == kDriverArgFaultAfterReady) {
      asm volatile("ud2");
    }
    return 0;
  }

  const uint64_t irq_mask = uint64_t{1} << static_cast<uint32_t>(irq_bit);
  for (;;) {
    (void)IrqWait(irq_mask, /*timeout_ticks=*/-1);
    g_controller.HandleInterrupt();
  }
}

}  // namespace protos::user
