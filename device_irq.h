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
  kInvalid,   // count/mode/capability mismatch, MSI with count > 1, or no CPU
              // that an MSI can address (DeviceIrqMsiCanTarget())
  kBusy,      // irq_alloc already succeeded for this device during this claim
  kNoDevice,  // unknown ISA device or no I/O APIC routes its ISA IRQ
  kNoMemory,  // fewer than `count` free IDT vectors in [0x50, 0x7F]
};

// Negative `device_index` tag encoding legacy ISA device `isa_device`
// (`0 <= isa_device < kIsaDeviceCount`) in `DeviceIrqAllocateVectors` and
// `DeviceIrqFreeDeviceVectors`, disjoint from PCI indices (`>= 0`) and `-1`.
constexpr int DeviceIrqIsaSlotId(const int isa_device) {
  return -2 - isa_device;
}

// Environment seams for host unit tests and the kernel implementation.
class DeviceIrqEnvironment {
 public:
  virtual ~DeviceIrqEnvironment() = default;

  virtual void RegisterIdtHandler(int vector, InterruptHandlerFn handler) = 0;
  virtual void UnregisterIdtHandler(int vector) = 0;
  // The Local APIC ID of the online CPU that device MSIs are sent to: the
  // BSP's if DeviceIrqMsiCanTarget() it, else that of another online CPU that
  // it can, or -1 if there is none. Every device vector is registered in the
  // IDT that all CPUs share, so any CPU can take the interrupts.
  virtual int MsiDestinationApicId() = 0;

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

// Whether a compatibility-format MSI can be sent to the CPU whose Local APIC
// ID is `apic_id`: its Destination ID field (Message Address bits 19:12) has
// 8 bits, and in physical destination mode 0xFF addresses every CPU (Intel SDM
// Vol. 3A, "Message Address Register Format"). Larger x2APIC IDs would need
// interrupt remapping, or the extended destination ID of some hypervisors,
// which the kernel doesn't support.
constexpr bool DeviceIrqMsiCanTarget(const uint32_t apic_id) {
  return apic_id < 0xFF;
}

// The Local APIC ID that DeviceIrqProgramDevice() would send MSIs to
// (DeviceIrqEnvironment::MsiDestinationApicId()), or -1 if no CPU qualifies.
int DeviceIrqMsiDestinationApicId();

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
// `mode` (`kSyscallIrqModeAuto`, `kSyscallIrqModeMsi`, `kSyscallIrqModeMsix`),
// for the CPU of `DeviceIrqEnvironment::MsiDestinationApicId()`. Turns the
// other mechanism off first. Programs MSI-X with MSI-X Enable and the Function
// Mask set, which some devices need to accept table writes, and changes only
// the Mask bit of each entry's Vector Control (its other bits are reserved).
// Returns `kBusy` if interrupts were already programmed on `device_index`
// during its current claim, and `kInvalid` without touching the device if no
// CPU can be addressed.
DeviceIrqStatus DeviceIrqProgramDevice(int device_index,  //
                                       int count,         //
                                       int64_t mode,      //
                                       const int* vectors);

// Masks any MSI-X table entries (and MSI per-vector mask bits) programmed for
// `device_index`, unmaps its kernel MSI-X table mapping if any, and clears its
// per-claim configured state. Call during teardown after `PciQuiesce()` while
// `Command.Memory Space` is still enabled. Sets MSI-X Enable (with the
// Function Mask) while it masks the table entries, then puts Message Control
// back as it found it.
void DeviceIrqReleaseDevice(int device_index);

// Clears the per-claim configured state of `device_index` when it is newly
// claimed.
void DeviceIrqResetDevice(int device_index);

// Programs and unmasks the I/O APIC redirection entry for claimed legacy ISA
// device `isa_device` (`0 <= isa_device < kIsaDeviceCount`) with `vector`
// (`kDeviceIrqVectorFirst..kDeviceIrqVectorLast`), targeting
// `DeviceIrqEnvironment::MsiDestinationApicId()`.
DeviceIrqStatus DeviceIrqProgramIsaDevice(int isa_device, int vector);

// Masks the I/O APIC redirection entry programmed for `isa_device` (flushing
// the posted MMIO write) and clears its per-claim configured state.
void DeviceIrqReleaseIsaDevice(int isa_device);

// Clears the per-claim configured state of `isa_device` when it is newly
// claimed.
void DeviceIrqResetIsaDevice(int isa_device);

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
// mapped MSI-X table or the word lies outside the table (`table_size * 4`
// words).
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
