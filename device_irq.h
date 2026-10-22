#ifndef PROTOS_DEVICE_IRQ_H_
#define PROTOS_DEVICE_IRQ_H_

#include <atomic>
#include <cstdint>
#include <memory>

#include "idt.h"
#include "pci.h"
#include "syscall_abi.h"

namespace protos {

struct Task;

// Device MSI and MSI-X interrupts, vector pool [0x50, 0x7F], and the wait loop
// of `irq_wait`. Nothing here knows a device class or device register layout
// beyond standard PCI MSI / MSI-X capabilities.

enum class DeviceIrqStatus {
  kOk = 0,
  kInvalid,   // count/mode/capability mismatch, or MSI with count > 1
  kBusy,      // irq_alloc already succeeded for this device during this claim
  kNoMemory,  // fewer than `count` free IDT vectors in [0x50, 0x7F]
};

// Environment seams for host unit tests and the kernel implementation.
class DeviceIrqEnvironment {
 public:
  virtual ~DeviceIrqEnvironment() = default;

  virtual void RegisterIdtHandler(int vector, InterruptHandlerFn handler) = 0;
  virtual void UnregisterIdtHandler(int vector) = 0;
  virtual uint8_t BspApicId() = 0;

  // Returns a writable 32-bit word pointer to the MSI-X table of `device_index`
  // (`msix.table_size * 4` 32-bit words), or nullptr if it cannot be mapped.
  virtual volatile uint32_t* MapMsixTable(int device_index,
                                          const PciMsix& msix) = 0;
  // Unmaps any kernel mapping created by `MapMsixTable` for `device_index`.
  virtual void UnmapMsixTable(int device_index) = 0;

  virtual int64_t NowTicks() = 0;
  virtual void Block() = 0;
  virtual void BlockUntil(int64_t deadline_tick) = 0;
  virtual void Wake(Task* task) = 0;
};

#if !__STDC_HOSTED__
std::shared_ptr<DeviceIrqEnvironment> CreateKernelDeviceIrqEnvironment();
#endif

// Initializes the device interrupt subsystem with `env` (non-null). In the
// kernel, `DriverInit()` calls this once at boot after `PciInit()`, reserving
// the kernel virtual MMIO window used to program MSI-X tables.
void DeviceIrqInit(std::shared_ptr<DeviceIrqEnvironment> env);

// Encodes the x86 Local APIC MSI 32-bit Message Address (`0xFEE00000 |
// (apic_id << 12)`, physical destination, fixed delivery) and 16-bit Message
// Data (`vector & 0xFF`, edge-triggered, fixed delivery).
constexpr uint32_t DeviceIrqMsiAddressLow(const uint8_t apic_id) {
  return 0xFEE00000U | (static_cast<uint32_t>(apic_id) << 12);
}

constexpr uint16_t DeviceIrqMsiData(const int vector) {
  return static_cast<uint16_t>(vector & 0xFF);
}

// Allocates `count` IDT vectors in `[kDeviceIrqVectorFirst,
// kDeviceIrqVectorLast]` for `owner`, binding vector `i` (`0 <= i < count`) to
// `(pending, 1ULL << (first_bit + i), task)`, registers the IDT handler on
// each vector, and writes the allocated vector numbers to
// `out_vectors[0..count - 1]`. All-or-nothing.
DeviceIrqStatus DeviceIrqAllocateVectors(uintptr_t owner,                 //
                                         int device_index,                //
                                         int count,                       //
                                         int first_bit,                   //
                                         std::atomic<uint64_t>* pending,  //
                                         Task* task,                      //
                                         int* out_vectors);

// Programs MSI or MSI-X on claimed PCI device `device_index` (`0 <=
// device_index < PciDeviceCount()`) with `vectors[0..count - 1]` according to
// `mode` (`kSyscallIrqModeAuto`, `kSyscallIrqModeMsi`, `kSyscallIrqModeMsix`).
// Returns `kBusy` if interrupts were already programmed on `device_index`
// during its current claim.
DeviceIrqStatus DeviceIrqProgramDevice(int device_index,  //
                                       int count,         //
                                       int64_t mode,      //
                                       const int* vectors);

// Masks any MSI-X table entries (and MSI per-vector mask bits) programmed for
// `device_index`, unmaps its kernel MSI-X table mapping if any, and clears its
// per-claim configured state. Call during teardown after `PciQuiesce()` while
// `Command.Memory Space` is still enabled.
void DeviceIrqReleaseDevice(int device_index);

// Clears the per-claim configured state of `device_index` when it is newly
// claimed.
void DeviceIrqResetDevice(int device_index);

// Unregisters and frees every IDT vector owned by `owner`. Returns how many
// vectors were freed.
int DeviceIrqFreeVectors(uintptr_t owner);

// Unregisters and frees every IDT vector owned by `owner` for `device_index`.
int DeviceIrqFreeDeviceVectors(uintptr_t owner, int device_index);

// Returns how many vectors in `[kDeviceIrqVectorFirst, kDeviceIrqVectorLast]`
// are currently allocated, and the owner of `vector` (`kPciNoOwner` if free or
// out of range).
int DeviceIrqUsedVectorCount();
uintptr_t DeviceIrqVectorOwner(int vector);

// Returns the IDT vector (`0x50..0x7F`) allocated to `owner` for pending bit
// `bit_index` (`0 <= bit_index < kSyscallIrqMaxCount`), or `-1` if none.
int DeviceIrqVectorForBit(uintptr_t owner, int bit_index);

// Reads 32-bit word `word_index` of the kernel-mapped MSI-X table of
// `device_index`, or returns 0 if `device_index` does not currently have a
// mapped MSI-X table.
uint32_t DeviceIrqReadMsixTableWord(int device_index, int word_index);

// Top-half interrupt dispatch for `vector` (`0x50..0x7F`): under the device
// IRQ lock, sets `slot.bit` in `*slot.pending` and wakes `slot.task`. Returns
// true if `vector` was bound to an active owner, or false if `vector` was
// unallocated or out of range (safe no-op for stray interrupts). Note that
// `IdtDispatch` sends the Local APIC EOI after the registered IDT handler
// returns.
bool DeviceIrqDispatch(int vector);

// Waits for any bit in `mask` (`(mask >> 32) == 0`) to become set in
// `*pending`, or until `timeout_ticks` (`<= kSyscallIrqWaitMaxTicks`) timer
// ticks have elapsed, or until `should_stop(stop_ctx)` returns true. Atomically
// clears and returns the fired bits (`*pending & mask`), or returns `0` on
// timeout or stop, or `kSyscallErrorInvalid` if `mask` or `timeout_ticks` is
// invalid.
// - `mask != 0 && timeout_ticks == 0`: non-blocking poll.
// - `mask == 0 && timeout_ticks > 0`: pure tick sleep (returns 0).
// - `mask == 0 && timeout_ticks <= 0`: blocks until `should_stop` (returns 0).
int64_t DeviceIrqWait(std::atomic<uint64_t>* pending,  //
                      uint64_t mask,                   //
                      int64_t timeout_ticks,           //
                      bool (*should_stop)(void* ctx),  //
                      void* stop_ctx);

#if __STDC_HOSTED__
void DeviceIrqResetForTest();
#endif

}  // namespace protos

#endif  // PROTOS_DEVICE_IRQ_H_
