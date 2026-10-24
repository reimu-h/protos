#ifndef PROTOS_PCI_CONFIG_FAKE_H_
#define PROTOS_PCI_CONFIG_FAKE_H_

#include <array>
#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <ostream>
#include <vector>

#include "pci.h"

namespace protos {

// Host model of PCI configuration space behind PciConfigSpace (pci.h), for
// unit tests of pci.cpp and of the code built on it.
//
// - A function that was not added reads as all ones at every width, as through
//   0xCF8/0xCFC, and ignores writes (counted).
// - Every bit of a function's 256 bytes is read-only, writable, or
//   write-1-to-clear, and writes honor that: AddFunction() makes Command's
//   I/O Space, Memory Space, Bus Master, Parity Error Response, SERR# Enable
//   and Interrupt Disable bits, Cache Line Size and Latency Timer writable and
//   Status's error bits write-1-to-clear (so a 32-bit write to Command clears
//   them, as on hardware); everything else is read-only unless a builder below
//   makes it writable.
// - A BAR is modeled by its size: its address bits below the size and its flag
//   bits are read-only, so writing all ones to it reads back the size mask, and
//   writing the saved value back restores it.
// - A function on a bus behind a PCI-to-PCI bridge (within the bridge's
//   secondary and subordinate bus numbers) that is in D1, D2 or D3hot reads
//   as all ones and ignores writes (counted as absent): such a bridge forwards
//   no configuration requests [PCIe Base 6.0 §5.3.1.4.1].
// - An access that is not naturally aligned or not inside the 256 bytes fails
//   the test (mechanism #1 cannot express it).
//
// Records every access, in order. Counts writes to a function's BAR registers
// while it decodes them (Command I/O Space or Memory Space set), and the
// largest number of accesses in progress at once: pci.cpp serializes them, so
// it must stay 1. Thread-safe.
class FakePciConfigSpace final : public PciConfigSpace {
 public:
  enum class BarKind { kNone, kIo, kMem32, kMem64 };

  struct BarSpec {
    BarKind kind = BarKind::kNone;
    uint64_t base = 0;
    // A power of two: at least 16 for memory BARs, 4 for I/O BARs.
    uint64_t size = 0;
    bool prefetchable = false;
    // An I/O BAR whose bits 31:16 are hardwired to zero (a 16-bit decoder).
    bool io_16bit_decode = false;
  };

  struct FunctionSpec {
    uint16_t vendor_id = 0;
    uint16_t device_id = 0;
    uint8_t class_code = 0;
    uint8_t subclass = 0;
    uint8_t prog_if = 0;
    uint8_t revision = 0;
    // Including the multi-function bit 7.
    uint8_t header_type = 0;
    uint16_t command = 0;
    uint16_t status = 0;
  };

  struct MsiSpec {
    bool is_64bit = false;
    bool per_vector_masking = false;
    // log2 of the number of vectors (Multiple Message Capable).
    int vectors_log2 = 0;
  };

  struct MsixSpec {
    int table_size = 1;
    int table_bar = 0;
    uint32_t table_offset = 0;
    int pba_bar = 0;
    uint32_t pba_offset = 0;
  };

  struct PmSpec {
    // PMCSR bits 1:0 at the start: 0 (D0) to 3 (D3hot).
    uint8_t power_state = 0;
    // PMCSR No_Soft_Reset. Without it, a write that takes the function from
    // D3hot to D0 resets it, as on hardware: every writable bit except those
    // of PMCSR returns to its reset value (0, or the one that AddPcie() gives).
    bool no_soft_reset = false;
    // After it leaves D3hot, the function reads as all ones and ignores writes
    // for this many reads, like a function that is still initializing (it
    // answers with Configuration Request Retry Status, which the Root Complex
    // finally completes as all ones).
    int reads_until_ready = 0;
    // A function that ignores writes to its BARs while it is in D3hot, so that
    // sizing them there reads back their base (which the spec forbids).
    bool ignores_bar_writes_in_d3hot = false;
  };

  // The PCI Express capability: its version, Device/Port Type and Slot
  // Implemented bit.
  struct PcieSpec {
    int version = 2;
    int port_type = 0;  // PCI Express Endpoint
    bool slot_implemented = false;
  };

  struct Access {
    bool write = false;
    int width = 0;  // bytes
    PciAddress address;
    int offset = 0;
    uint32_t value = 0;
  };

  FakePciConfigSpace() = default;
  FakePciConfigSpace(const FakePciConfigSpace&) = delete;
  FakePciConfigSpace& operator=(const FakePciConfigSpace&) = delete;

  // Adds a function (replacing any at `address`) with the IDs, class, header
  // type, Command and Status of `spec`, and nothing else.
  void AddFunction(PciAddress address, const FunctionSpec& spec);
  // Models BAR `index` (0..5; header type 1: 0..1) as `bar`. A 64-bit BAR also
  // takes BAR `index` + 1.
  void SetBar(PciAddress address, int index, const BarSpec& bar);
  // Sets the bus number registers of a PCI-to-PCI (or CardBus) bridge, which
  // are writable.
  void SetBridgeBuses(PciAddress address,  //
                      uint8_t primary,     //
                      uint8_t secondary,   //
                      uint8_t subordinate);
  // Appends a capability with `id` at `offset` (aligned, at least 0x40) to the
  // function's capability list; the first one sets the Capabilities Pointer
  // and the Status Capabilities List bit.
  void AddCapability(PciAddress address, int offset, uint8_t id);
  // Appends an MSI capability: Message Control's Enable and Multiple Message
  // Enable bits, Message Address, Message Data and (with per-vector masking)
  // Mask Bits are writable.
  void AddMsi(PciAddress address, int offset, const MsiSpec& msi);
  // Appends an MSI-X capability: Message Control's Function Mask and Enable
  // bits are writable.
  void AddMsix(PciAddress address, int offset, const MsixSpec& msix);
  // Appends a Power Management capability: PMCSR's power state and PME_En
  // bits are writable and PME_Status is write-1-to-clear.
  void AddPowerManagement(PciAddress address, int offset, const PmSpec& pm);
  // Appends a PCI Express capability [PCIe Base 6.0 §7.5.3] whose registers
  // that hold configuration are writable, as far as `pcie` has them: Device
  // Control (bits 14:0; reset value 0x2810: Relaxed Ordering, No Snoop, a
  // 512-byte Max_Read_Request_Size), Link Control (except Retrain Link), Slot
  // Control, Root Control and Device Control 2.
  void AddPcie(PciAddress address, int offset, const PcieSpec& pcie);
  // Makes the bits `mask` of the `width`-byte register at `offset` writable
  // (e.g. a bridge's windows, or vendor-specific registers).
  void SetWritable(PciAddress address, int offset, int width, uint32_t mask);
  // Set registers of an added function directly, ignoring the write masks (for
  // malformed devices), without recording an access.
  void SetRaw8(PciAddress address, int offset, uint8_t value);
  void SetRaw16(PciAddress address, int offset, uint16_t value);
  void SetRaw32(PciAddress address, int offset, uint32_t value);

  // Read registers without recording an access (all ones if absent).
  uint8_t Peek8(PciAddress address, int offset) const;
  uint16_t Peek16(PciAddress address, int offset) const;
  uint32_t Peek32(PciAddress address, int offset) const;

  std::vector<Access> Accesses() const;
  void ClearAccesses();
  int BarWritesWhileDecoding() const;
  int WritesToAbsentFunctions() const;
  int MaxConcurrentAccesses() const;

  uint8_t Read8(PciAddress address, int offset) override;
  uint16_t Read16(PciAddress address, int offset) override;
  uint32_t Read32(PciAddress address, int offset) override;
  void Write8(PciAddress address, int offset, uint8_t value) override;
  void Write16(PciAddress address, int offset, uint16_t value) override;
  void Write32(PciAddress address, int offset, uint32_t value) override;

 private:
  struct Function {
    std::array<uint8_t, kPciConfigSpaceSize> bytes = {};
    std::array<uint8_t, kPciConfigSpaceSize> writable = {};
    std::array<uint8_t, kPciConfigSpaceSize> write_one_to_clear = {};
    // The values that writable bits take when the function resets.
    std::array<uint8_t, kPciConfigSpaceSize> reset_values = {};
    std::vector<int> capability_offsets;
    // Offset of the Power Management capability, 0 if none.
    int pm_offset = 0;
    // PmSpec::reads_until_ready and ignores_bar_writes_in_d3hot, and how many
    // more reads the function answers as all ones.
    int reads_until_ready = 0;
    bool ignores_bar_writes_in_d3hot = false;
    int reads_until_ready_left = 0;
  };

  static uint32_t Key(PciAddress address);
  Function& FunctionLocked(PciAddress address);
  const Function* FindLocked(PciAddress address) const;
  // Whether a bridge in D1, D2 or D3hot keeps requests from reaching `bus`.
  bool BehindSleepingBridgeLocked(int bus) const;
  static int PowerState(const Function& function);
  static void LinkCapabilityLocked(Function* function, int offset, uint8_t id);
  static uint32_t Load(const Function& function, int offset, int width);
  static void Store(std::array<uint8_t, kPciConfigSpaceSize>* bytes,  //
                    int offset,                                       //
                    int width,                                        //
                    uint32_t value);
  uint32_t Read(PciAddress address, int offset, int width);
  void Write(PciAddress address, int offset, int width, uint32_t value);

  mutable std::mutex mutex_;
  std::map<uint32_t, Function> functions_;
  std::vector<Access> accesses_;
  int bar_writes_while_decoding_ = 0;
  int writes_to_absent_functions_ = 0;
  std::atomic<int> accesses_in_progress_{0};
  std::atomic<int> max_concurrent_accesses_{0};
};

// Presets with the configuration space of QEMU 11.1's `pc` (i440FX) machine at
// kernel entry, as SeaBIOS (`-m 128M`, the default) or OVMF (`-m 4G`) leaves
// it: host bridge 00:00.0, PIIX3 ISA bridge 00:01.0 (multi-function), IDE
// 00:01.1, PIIX4 ACPI 00:01.3, std VGA 00:02.0, e1000 00:03.0, and per
// topology:
// - kSingle: `-device qemu-xhci` at 00:04.0 (MSI-X, 16 vectors);
// - kMulti: kSingle plus `-device pci-bridge,chassis_nr=1` at 00:05.0 (bus 1)
//   and `-device nec-usb-xhci,bus=br1,addr=1,msi=on,msix=off` at 01:01.0 (MSI
//   only).
enum class QemuFirmware { kSeaBios, kOvmf };
enum class QemuTopology { kNone, kSingle, kMulti };
void AddQemuPcMachine(FakePciConfigSpace* fake,  //
                      QemuFirmware firmware,     //
                      QemuTopology topology);

// Printers for test failure messages (found by gtest through ADL).
void PrintTo(const PciAddress& address, std::ostream* os);
void PrintTo(const PciBar& bar, std::ostream* os);
void PrintTo(PciStatus status, std::ostream* os);
void PrintTo(const FakePciConfigSpace::Access& access, std::ostream* os);

}  // namespace protos

#endif  // PROTOS_PCI_CONFIG_FAKE_H_
