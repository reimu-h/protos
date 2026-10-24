#ifndef PROTOS_USER_USBKBD_XHCI_CONTROLLER_H_
#define PROTOS_USER_USBKBD_XHCI_CONTROLLER_H_

#include <cstdint>
#include <type_traits>

#include "user/usbkbd/usb_descriptors.h"
#include "user/usbkbd/usb_hid.h"
#include "user/usbkbd/xhci_context.h"
#include "user/usbkbd/xhci_platform.h"
#include "user/usbkbd/xhci_regs.h"
#include "user/usbkbd/xhci_ring.h"

// One xHCI controller of the ring-3 USB keyboard driver. Freestanding,
// header-only and without system calls (see user/usbkbd/xhci_platform.h).
//
// Bring-up has two phases, so that the program enables bus mastering only
// after the controller has stopped using whatever state firmware left in it:
// 1. TakeOwnershipAndReset() takes the controller over from firmware (USB
//    Legacy Support handoff), halts it if it runs, and resets it. No DMA.
//    Note: pci_claim leaves PCI Bus Master as firmware set it; if Phase 1
//    returns an error after handoff, the caller clears Bus Master.
// 2. The caller enables PCI Bus Master and allocates the controller's
//    interrupt (MSI-X preferred, then MSI) BEFORE calling Start(IrqMode),
//    which allocates DMA rings/contexts, starts the controller, powers and
//    scans ports, and drives non-blocking enumeration and HID input via
//    HandleInterrupt() and NextDeadline().

namespace protos::usb {

// Phase-1 timeouts in milliseconds.
// Firmware must release the HC BIOS Owned semaphore within this time after
// the OS asks for ownership; otherwise the driver takes ownership by force
// (like Linux).
constexpr int64_t kHandoffTimeoutMs = 1000;
// USBSTS.CNR must clear before the first operational register write. On a
// timeout the driver logs it and continues, like Linux; the reset that follows
// has bounded waits of its own.
constexpr int64_t kControllerReadyTimeoutMs = 5000;
// A running controller halts within 16 ms after Run/Stop is cleared (xHCI
// 5.4.1); Linux allows 32 ms. A controller that does not halt is not reset.
constexpr int64_t kHaltTimeoutMs = 32;
// No register access for at least this long after HCRST is set (Intel
// controllers need 1 ms; EDK2 waits on all controllers).
constexpr int64_t kPostResetDelayMs = 1;
// After HCRST is set, HCRST must clear and then CNR must clear, each within
// this time (Linux XHCI_RESET_LONG_USEC).
constexpr int64_t kResetTimeoutMs = 10000;
constexpr int64_t kResetReadyTimeoutMs = 10000;

// Phase-2 timeouts and limits.
constexpr int64_t kNoDeadline = INT64_MAX;
constexpr int64_t kStartTimeoutMs = 32;
constexpr int64_t kPortPowerSettleMs = 20;
constexpr int64_t kPortDebounceMs = 100;
constexpr int64_t kPortDebounceMaxMs = 2000;
constexpr int64_t kPortResetTimeoutMs = 800;
constexpr int64_t kPortResetRecoveryMs = 50;
constexpr int64_t kAddressSettleMs = 10;
constexpr int64_t kCommandTimeoutMs = 5000;
// After CRCR.CA the Command Ring must stop (CRR = 0) within this time, else
// the controller is reset (xHCI 4.6.1.2; Linux xhci_abort_cmd_ring()).
constexpr int64_t kCommandAbortTimeoutMs = 5000;
constexpr int64_t kControlTimeoutMs = 5000;

constexpr int kMaxPorts = 32;
constexpr int kMaxDevices = 16;
// Slot IDs 1..kMaxSlotIds: CONFIG.MaxSlotsEn is capped at this.
constexpr int kMaxSlotIds = 64;
constexpr int kMaxHubPorts = 16;
constexpr int kMaxHubDepth = 5;
// USB 2.0 7.1.7.3 / Linux hub_power_on(): wait at least 100 ms after powering
// hub ports even when bPwrOn2PwrGood is 0 (e.g. Realtek 0bda:5411).
constexpr int64_t kMinHubPowerSettleMs = 100;
constexpr int64_t kMaxHubPowerSettleMs = 510;
constexpr int64_t kHubColdRescanIntervalMs = 150;
constexpr int kMaxHubColdRescans = 2;
constexpr int64_t kHubResetPollMs = 20;
constexpr int kRingTrbs = 256;
constexpr int kReportTrbs = 8;
constexpr int kReportBufferSize = 512;
constexpr int kControlBufferSize = 1024;
constexpr int kMaxEnumerationAttempts = 3;
// A port whose device stayed configured this long starts with a fresh
// kMaxEnumerationAttempts budget when it fails again, so that rare unrelated
// errors don't add up to a permanently disabled port.
constexpr int64_t kPortHealthyMs = 60000;
constexpr int kMaxInterruptErrors = 3;
// Interrupt endpoint errors further apart than this start a new count toward
// kMaxInterruptErrors: an idle keyboard (SET_IDLE(0)) has no successful
// transfer that would reset it.
constexpr int64_t kInterruptErrorWindowMs = 1000;
// USB Transaction Errors on an interrupt endpoint are first recovered with a
// Soft Retry (Reset Endpoint with TSP = 1, xHCI 4.6.8.1; Linux MAX_SOFT_RETRY).
constexpr int kMaxSoftRetries = 3;
// More than kMaxRecoveries controller resets within kRecoveryWindowMs (from
// the first of them) leave the controller in kError; later ones start anew.
constexpr int kMaxRecoveries = 3;
constexpr int64_t kRecoveryWindowMs = 60000;
// A Disable Slot that fails is retried; after this many attempts only a
// controller reset can free the slot's contexts and rings safely.
constexpr int kMaxDisableSlotAttempts = 3;
// A hub whose control requests fail this often within the window is
// re-enumerated through its parent port (whose attempts are limited too).
constexpr int kMaxHubControlFailures = 3 * kMaxEnumerationAttempts;
constexpr int64_t kHubControlFailureWindowMs = 60000;

// The longest controller label (e.g. "bb:dd.f") kept for log lines.
constexpr int kMaxControllerLabelLength = 15;

enum class XhciState : uint8_t {
  kUnbound = 0,  // Init() not called yet
  kBound,        // Bound to its platform and registers
  kReset,        // TakeOwnershipAndReset() succeeded: OS-owned, halted, reset
  kError,        // Unrecoverable error: leave the controller alone
  kRunning,      // Start() succeeded: rings active, interrupts enabled
};

// How TakeOwnershipAndReset() took the controller over from firmware.
enum class HandoffOutcome : uint8_t {
  kNotRun = 0,
  kNoLegacyCapability,  // No USB Legacy Support capability (e.g. QEMU)
  kNotBiosOwned,        // Firmware did not own it: claimed without waiting
  kReleased,            // Firmware released it within kHandoffTimeoutMs
  kForced,              // Firmware did not release it: ownership forced
};

enum class IrqMode : uint8_t {
  kMsix = 0,
  kMsi,
};

inline const char* IrqModeName(const IrqMode mode) {
  return mode == IrqMode::kMsix ? "msix" : "msi";
}

inline const char* SpeedName(const int speed) {
  switch (speed) {
    case kPortSpeedLow:
      return "ls";
    case kPortSpeedFull:
      return "fs";
    case kPortSpeedHigh:
      return "hs";
    case kPortSpeedSuper:
    case kPortSpeedSuperPlus:
      return "ss";
    default:
      return "unknown";
  }
}

inline LogLine& AppendPortPath(LogLine& line,        //
                               const int root_port,  //
                               const uint32_t route_string) {
  line.Dec(root_port);
  for (int tier = 0; tier < 5; ++tier) {
    const uint32_t hub_port = (route_string >> (4 * tier)) & 0xFu;
    if (hub_port == 0) {
      break;
    }
    line.Str(".").Dec(static_cast<int>(hub_port));
  }
  return line;
}

class XhciController {
 public:
  // constexpr, so that a program can keep its controllers in a constinit
  // array (crt never runs dynamic initializers).
  constexpr XhciController() = default;
  // A controller object stands for the hardware; never copy it.
  XhciController(const XhciController&) = delete;
  XhciController& operator=(const XhciController&) = delete;

  // Binds the controller to the program's services and to its registers;
  // both must outlive this object. `label` (e.g. "00:04.0") names the
  // controller in log lines; its first kMaxControllerLabelLength characters
  // are kept. `locks` optionally shares Caps Lock state across controllers
  // (defaults to process-wide `g_default_keyboard_locks`).
  void Init(Platform& platform,  //
            MmioWindow& mmio,    //
            const char* label,   //
            KeyboardLocks* locks = nullptr);

  // Phase 1 of the bring-up, in this order:
  // 1. Decodes and checks the capability registers and the extended
  //    capabilities (DecodeCaps(), ParseExtendedCaps()).
  // 2. USB Legacy Support handoff, if the capability exists: sets HC OS
  //    Owned, waits up to kHandoffTimeoutMs for firmware to clear HC BIOS
  //    Owned and on a timeout clears it itself (keeping OS Owned set), then
  //    disables all legacy SMIs and clears the SMI events.
  // 3. Waits up to kControllerReadyTimeoutMs for USBSTS.CNR = 0 (logs a
  //    timeout and continues).
  // 4. Clears USBCMD Run/Stop and the interrupt enables and, if the
  //    controller runs, waits up to kHaltTimeoutMs for USBSTS.HCH = 1. A
  //    controller that does not halt is abandoned without a reset.
  // 5. Sets USBCMD.HCRST, accesses no register for kPostResetDelayMs, then
  //    waits for HCRST = 0 and CNR = 0 and checks that HCH = 1.
  // Allocates no DMA memory and needs no bus mastering. Every wait is
  // bounded and stops early when registers read 0xFFFFFFFF. On an error,
  // logs "[USBKBD] <label>: error: <status name>", sets state kError and
  // writes nothing more. Can be called again in any state (the handoff then
  // finds the controller OS-owned and does not wait).
  [[nodiscard]] XhciStatus TakeOwnershipAndReset();

  // Phase 2 of the bring-up: requires state() == XhciState::kReset, PCI Bus
  // Master enabled, and the controller's interrupt allocated. Allocates and
  // zeroes DMA structures (DCBAA, scratchpads, command/event/transfer rings,
  // device contexts), programs registers, sets USBCMD.INTE | USBCMD.RS,
  // powers ports if PPC=1, emits "[USBKBD] controller <label> ready (<mode>)",
  // and scans all root-hub ports.
  [[nodiscard]] XhciStatus Start(IrqMode mode);

  // Drains Interrupter 0's event ring (acknowledging USBSTS.EINT, IMAN.IP, and
  // ERDP.EHB) and advances all port, command, and device state machines whose
  // deadlines are due at Platform::NowTicks(). Safe to call on an interrupt,
  // a deadline expiry, or a spurious wakeup.
  void HandleInterrupt();

  // Earliest pending absolute tick deadline across all ports, commands, and
  // device state machines, or kNoDeadline (INT64_MAX) when idle.
  int64_t NextDeadline() const;

  XhciState state() const { return state_; }
  const XhciCaps& caps() const { return caps_; }
  const char* label() const { return label_; }
  HandoffOutcome handoff_outcome() const { return handoff_; }
  IrqMode irq_mode() const { return irq_mode_; }
  int attached_keyboard_count() const { return attached_keyboards_; }

 private:
  enum class PollResult : uint8_t { kMatched, kTimedOut, kNotDecoding };

  enum class PortFsm : uint8_t {
    kDisconnected = 0,
    kDebouncing,
    kResetting,
    kResetRecovery,
    kEnumerating,
    kActive,
    kDisabled,
  };

  enum class DevFsm : uint8_t {
    kFree = 0,
    kWantEnableSlot,
    kWaitingEnableSlot,
    kWantAddressDevice,
    kWaitingAddressDevice,
    kAddressSettle,
    kWaitingGetDesc8,
    kWantEvaluateEp0,
    kWaitingEvaluateEp0,
    kWaitingGetDesc18,
    kWaitingGetConfig9,
    kWaitingGetConfigFull,
    kWaitingGetHubDesc,
    kWantConfigureEndpoint,
    kWaitingConfigureEndpoint,
    kWaitingSetConfig,
    kWaitingSetHubDepth,
    kWaitingHubPortPower,
    kHubPowerSettle,
    kWaitingSetProtocol,
    kWaitingSetIdle,
    kConfigured,
    kWantResetEp0,
    kWaitingResetEp0,
    kWantSetDeqEp0,
    kWaitingSetDeqEp0,
    kWantResetIntrEp,
    kWaitingResetIntrEp,
    kWantSetDeqIntrEp,
    kWaitingSetDeqIntrEp,
    kWantStopEp0,
    kWaitingStopEp0,
    kWaitingHubSetConfig,
    kWantStopIntrEp,
    kWaitingStopIntrEp,
    kWaitingClearIntrHalt,
  };

  enum class Ep0RecoveryNext : uint8_t {
    kNone = 0,
    kSetIdle,
    kFinishAttach,
    kHubPortPower,
    kHubControl,
  };

  enum class HubPortFsm : uint8_t {
    kDisconnected = 0,
    kDebouncing,
    kWantReset,
    kResetting,
    kResetRecovery,
    kEnumerating,
    kActive,
    kDisabled,
  };

  enum class HubCtrlOp : uint8_t {
    kNone = 0,
    kGetPortStatus,
    kClearPortFeature,
    kSetPortReset,
    kGetHubStatus,
    kClearHubFeature,
    kDisablePort,
    kPowerPort,
    kClearEndpointHalt,
  };

  struct HubPortState {
    constexpr HubPortState() = default;
    HubPortFsm fsm = HubPortFsm::kDisconnected;
    int speed = 0;
    int attempts = 0;
    int ctrl_errors = 0;
    int dev_idx = -1;
    uint16_t pending_clear_mask = 0;
    // CLEAR_FEATURE(PORT_ENABLE) is due: the driver gave up on the device of
    // this USB 2.0 hub port, which would otherwise keep answering its address.
    bool disable_pending = false;
    // SET_FEATURE(PORT_POWER) is due: over-current powered the port off and
    // has since cleared.
    bool power_pending = false;
    int64_t debounce_start = 0;
    int64_t deadline = kNoDeadline;
    // When the port's device was configured (0: not active).
    int64_t active_since = 0;
  };

  struct PortState {
    constexpr PortState() = default;
    PortFsm fsm = PortFsm::kDisconnected;
    int speed = 0;
    int attempts = 0;
    int dev_idx = -1;
    int64_t debounce_start = 0;
    int64_t deadline = kNoDeadline;
    // When the port's device was configured (0: not active).
    int64_t active_since = 0;
  };

  struct DeviceSlot {
    constexpr DeviceSlot() = default;
    bool in_use = false;
    bool announced = false;
    // Not in use, but the xHC still owns this DeviceSlot's contexts, rings and
    // buffers for this slot ID until its Disable Slot completes (xHCI 4.6.4,
    // 6.1): not allocated again until then (0: free).
    uint8_t disabling_slot_id = 0;
    int port = 0;               // 1-based root hub port
    int parent_dev_idx = -1;    // -1 if root-attached, else hub DeviceSlot idx
    int parent_port = 0;        // 1-based port on parent hub (0 if root)
    uint32_t route_string = 0;  // 20-bit xHCI Route String
    int hub_depth = 0;          // 0 for root-attached, parent.hub_depth + 1
    uint8_t tt_hub_slot_id = 0;
    uint8_t tt_port_number = 0;
    bool tt_mtt = false;
    int speed = 0;
    uint8_t slot_id = 0;
    uint16_t ep0_mps = 8;
    uint8_t device_class = 0;
    uint8_t device_protocol = 0;
    // From the device descriptor once it was read, else 0.
    uint16_t vendor_id = 0;
    uint16_t product_id = 0;
    bool is_hub = false;
    DevFsm state = DevFsm::kFree;
    Ep0RecoveryNext after_ep0_recovery = Ep0RecoveryNext::kNone;

    uint8_t* out_ctx_va = nullptr;
    uint64_t out_ctx_phys = 0;
    uint8_t* in_ctx_va = nullptr;
    uint64_t in_ctx_phys = 0;
    uint8_t* ctrl_buf_va = nullptr;
    uint64_t ctrl_buf_phys = 0;
    Trb* ep0_ring_va = nullptr;
    uint64_t ep0_ring_phys = 0;
    Trb* intr_ring_va = nullptr;
    uint64_t intr_ring_phys = 0;
    uint8_t* report_buf_va = nullptr;
    uint64_t report_buf_phys = 0;

    ProducerRing<kRingTrbs> ep0_ring = {};
    ProducerRing<kRingTrbs> intr_ring = {};

    bool ctrl_in_flight = false;
    uint64_t ctrl_setup_phys = 0;
    uint64_t ctrl_data_phys = 0;
    uint64_t ctrl_status_phys = 0;
    uint16_t ctrl_req_len = 0;
    uint16_t ctrl_actual_len = 0;
    int64_t deadline = kNoDeadline;

    BootKeyboardMatch kbd = {};
    uint64_t report_trb_phys[kReportTrbs] = {};
    int next_rearm_idx = 0;
    int intr_errors = 0;
    int64_t last_intr_error = 0;
    // Interrupt endpoint recovery: Soft Retries in a row, whether the pending
    // Reset Endpoint is one (TSP = 1), and whether the host's data toggle /
    // sequence number was reset (TSP = 0), so that the device's must be reset
    // too, by CLEAR_FEATURE(ENDPOINT_HALT), before the endpoint is re-armed.
    int intr_soft_retries = 0;
    bool intr_soft_retry = false;
    bool intr_clear_halt_due = false;
    HidKeyboardState hid_state = {};

    // Hub-specific state (used when is_hub == true):
    HubMatch hub_match = {};
    HubDescriptorInfo hub_desc = {};
    uint8_t hub_num_ports = 0;
    uint8_t hub_ttt = 0;
    bool hub_mtt = false;
    uint8_t hub_init_port = 0;
    int64_t hub_power_settle_ms = kMinHubPowerSettleMs;
    uint8_t hub_cold_rescans_left = 0;
    int64_t hub_rescan_deadline = kNoDeadline;
    uint32_t hub_pending_scan_mask = 0;
    HubCtrlOp hub_ctrl_op = HubCtrlOp::kNone;
    uint8_t hub_ctrl_port = 0;
    uint16_t hub_ctrl_feature = 0;
    bool hub_intr_armed = false;
    uint64_t hub_intr_trb_phys = 0;
    // Hub-level changes (bit 0 of the status-change bitmap): GET_STATUS(hub)
    // is due, then CLEAR_FEATURE(C_HUB_*) for the wHubChange bits in the mask.
    bool hub_status_pending = false;
    uint16_t hub_pending_hub_clear_mask = 0;
    int hub_status_errors = 0;
    bool hub_status_broken = false;  // Hub-level changes are ignored
    // The status-change endpoint needs Stop/Reset Endpoint + Set TR Dequeue,
    // started once EP0 is idle (PumpHubControl() step 0b).
    bool intr_recovery_pending = false;
    // CLEAR_FEATURE(ENDPOINT_HALT) of the status-change endpoint is due
    // before it is re-armed (PumpHubControl() step 0c).
    bool hub_clear_halt_pending = false;
    bool ep0_stop_retried = false;
    int hub_ctrl_failures = 0;
    int64_t hub_fail_window_start = 0;
    HubPortState hub_ports[kMaxHubPorts];
  };

  struct CommandState {
    bool busy = false;
    bool is_disable_slot = false;
    bool abandoned = false;
    // The command timed out and CRCR.CA was written: no command is queued
    // until the ring has stopped (FinishCommandAbort()).
    bool aborting = false;
    int dev_idx = -1;
    uint8_t disable_slot_id = 0;
    uint64_t trb_phys = 0;
    int64_t deadline = kNoDeadline;
  };

  XhciStatus RunPhase1();
  XhciStatus TakeOwnershipFromFirmware();
  XhciStatus WaitUntilReady();
  XhciStatus HaltIfRunning();
  XhciStatus ResetHalted();
  PollResult Poll(int64_t offset,     //
                  uint32_t mask,      //
                  uint32_t expected,  //
                  int64_t timeout_ms);
  uint32_t ReadOp(const int64_t reg) { return mmio_->Read32(caps_.Op(reg)); }
  void WriteOp(const int64_t reg, const uint32_t value) {
    mmio_->Write32(caps_.Op(reg), value);
  }
  uint32_t ReadPortSc(const int port) {
    return mmio_->Read32(caps_.PortReg(port, kPortSc));
  }
  void WritePortSc(const int port, const uint32_t value) {
    mmio_->Write32(caps_.PortReg(port, kPortSc), value);
  }
  LogLine StartLogLine() const;

  // Phase-2 helpers.
  XhciStatus RunStart(IrqMode mode);
  XhciStatus EnsureDmaAllocated();
  void FormatCoreDma();
  void ScanPort(int port, int64_t now);
  void HandlePortDisconnect(int port, int64_t now);
  void StartPortResetOrUsb3(int port, int64_t now);
  void RetryOrDisablePort(int port, int64_t now, const char* reason);
  void RetryOrDisableHubPort(int hub_idx,  //
                             int port,     //
                             int64_t now,  //
                             const char* reason);
  void RetryOrDisableDevice(DeviceSlot& dev, int64_t now, const char* reason);
  void DisableUnsupportedDevice(DeviceSlot& dev, const char* reason);
  void BeginDeviceEnumeration(int port, int64_t now);
  void BeginHubDeviceEnumeration(int hub_idx, int port, int64_t now);
  int FindFreeDevice() const;
  bool AnyDeviceDisabling() const;
  void DetachDeviceTree(int dev_idx);
  void ReleaseDevice(int dev_idx);
  void QueueDisableSlot(uint8_t slot_id, bool front);
  void OnDisableSlotCompletion(uint8_t slot_id, uint8_t cc);
  void RetryDisableSlot(uint8_t slot_id, const char* reason);
  void AnnounceKeyboardAttach(DeviceSlot& dev);
  void AnnounceKeyboardDetach(DeviceSlot& dev);
  void AnnounceHubAttach(const DeviceSlot& dev);
  void MarkParentPortActive(DeviceSlot& dev, int64_t now);

  void StartHubConfiguration(DeviceSlot& dev,         //
                             const HubMatch& hmatch,  //
                             int64_t now);
  void StartHubDescriptorFetch(DeviceSlot& dev, int64_t now);
  void StartHubPortPower(DeviceSlot& dev, int64_t now);
  void FinishHubAttach(int dev_idx, int64_t now);
  void ArmHubInterruptTransfer(DeviceSlot& hub);
  bool HubAllPortsDisconnected(const DeviceSlot& hub) const;
  void PumpHubControl(int hub_idx, int64_t now);
  void OnHubPortStatus(int hub_idx,      //
                       int port,         //
                       uint16_t status,  //
                       uint16_t change,  //
                       int64_t now);
  void OnHubStatus(int hub_idx, uint16_t status, uint16_t change);
  void IgnoreHubStatusChanges(DeviceSlot& hub);
  void OnHubControlFailure(int hub_idx,       //
                           HubCtrlOp op,      //
                           int port,          //
                           uint16_t feature,  //
                           bool timed_out,    //
                           int64_t now);
  bool HubControlBudgetExhausted(DeviceSlot& hub, int64_t now);
  bool HubAddressingBusy(int root_port) const;
  void HandleHubPortDisconnect(int hub_idx, int port, int64_t now);

  void DispatchEvent(const Trb& ev);
  void OnCommandCompletion(const Trb& ev, int64_t now);
  void OnTransferEvent(const Trb& ev, int64_t now);
  void OnEp0TransferEvent(DeviceSlot& dev, const Trb& ev, int64_t now);
  void OnControlTransferComplete(DeviceSlot& dev,  //
                                 bool ok,          //
                                 uint8_t cc,       //
                                 int64_t now);
  void OnInterruptTransferEvent(DeviceSlot& dev, const Trb& ev, int64_t now);
  bool CountInterruptError(DeviceSlot& dev, uint8_t cc, int64_t now);
  void StartInterruptEndpointRecovery(DeviceSlot& dev);
  void ResumeInterruptEndpoint(DeviceSlot& dev, int64_t now);

  void SubmitCommand(const Trb& trb,           //
                     int dev_idx,              //
                     bool is_disable_slot,     //
                     uint8_t disable_slot_id,  //
                     int64_t now);
  void PumpCommandRing(int64_t now);
  void SubmitControlTransfer(DeviceSlot& dev,          //
                             uint8_t bm_request_type,  //
                             uint8_t b_request,        //
                             uint16_t w_value,         //
                             uint16_t w_index,         //
                             uint16_t w_length,        //
                             int64_t now);
  void StartSetIdle(DeviceSlot& dev, int64_t now);
  void ArmInterruptTransfers(DeviceSlot& dev);
  void FinishKeyboardAttach(DeviceSlot& dev, int64_t now);
  void AdvanceStateMachines(int64_t now);
  void AbortCommand(int64_t now);
  void FinishCommandAbort(int64_t now);
  void Recover(const char* reason);
  void DetachAllKeyboards();

  Platform* platform_ = nullptr;
  MmioWindow* mmio_ = nullptr;
  KeyboardLocks* locks_ = &g_default_keyboard_locks;
  char label_[kMaxControllerLabelLength + 1] = {};
  XhciState state_ = XhciState::kUnbound;
  HandoffOutcome handoff_ = HandoffOutcome::kNotRun;
  IrqMode irq_mode_ = IrqMode::kMsix;
  XhciCaps caps_ = {};

  bool dma_ready_ = false;
  uint8_t* core_dma_va_ = nullptr;
  uint64_t core_dma_phys_ = 0;
  int64_t core_dma_bytes_ = 0;
  uint64_t* dcbaa_ = nullptr;
  uint64_t dcbaa_phys_ = 0;
  uint64_t sp_array_phys_ = 0;
  ProducerRing<kRingTrbs> cmd_ring_ = {};
  EventRing<kRingTrbs> event_ring_ = {};

  int ports_ = 0;
  int slots_ = 0;
  int attached_keyboards_ = 0;
  int recoveries_ = 0;
  int64_t recovery_window_start_ = 0;
  bool in_recovery_ = false;
  bool event_ring_full_rescan_ = false;
  // Set where a controller reset can't run (inside the event loop); done by
  // the next AdvanceStateMachines().
  const char* pending_recovery_ = nullptr;
  // Slot IDs whose Disable Slot is due, in the post-order DetachDeviceTree()
  // queued them: children before their hub (e.g. its TT clients). A slot is
  // queued once (disable_queued_[]) until its Disable Slot completes.
  uint8_t pending_disable_fifo_[kMaxSlotIds] = {};
  int pending_disable_count_ = 0;
  bool disable_queued_[kMaxSlotIds + 1] = {};
  uint8_t disable_attempts_[kMaxSlotIds + 1] = {};
  CommandState cmd_state_ = {};
  PortState port_states_[kMaxPorts];
  DeviceSlot devices_[kMaxDevices];
};

static_assert(std::is_trivially_destructible_v<XhciController>);

inline void XhciController::Init(Platform& platform,       //
                                 MmioWindow& mmio,         //
                                 const char* const label,  //
                                 KeyboardLocks* const locks) {
  platform_ = &platform;
  mmio_ = &mmio;
  locks_ = locks != nullptr ? locks : &g_default_keyboard_locks;
  int length = 0;
  if (label != nullptr) {
    while (length < kMaxControllerLabelLength && label[length] != '\0') {
      label_[length] = label[length];
      ++length;
    }
  }
  label_[length] = '\0';
  state_ = XhciState::kBound;
  handoff_ = HandoffOutcome::kNotRun;
  irq_mode_ = IrqMode::kMsix;
  caps_ = XhciCaps{};
  dma_ready_ = false;
  core_dma_va_ = nullptr;
  core_dma_phys_ = 0;
  core_dma_bytes_ = 0;
  dcbaa_ = nullptr;
  dcbaa_phys_ = 0;
  sp_array_phys_ = 0;
  ports_ = 0;
  slots_ = 0;
  attached_keyboards_ = 0;
  recoveries_ = 0;
  recovery_window_start_ = 0;
  in_recovery_ = false;
  event_ring_full_rescan_ = false;
  pending_recovery_ = nullptr;
  pending_disable_count_ = 0;
  for (int s = 0; s <= kMaxSlotIds; ++s) {
    disable_queued_[s] = false;
    disable_attempts_[s] = 0;
  }
  cmd_state_ = CommandState{};
  for (int i = 0; i < kMaxPorts; ++i) {
    port_states_[i] = PortState{};
  }
  for (int i = 0; i < kMaxDevices; ++i) {
    devices_[i] = DeviceSlot{};
  }
}

inline XhciStatus XhciController::TakeOwnershipAndReset() {
  if (platform_ == nullptr || mmio_ == nullptr) {
    return XhciStatus::kNotBound;
  }
  state_ = XhciState::kBound;
  handoff_ = HandoffOutcome::kNotRun;
  const XhciStatus status = RunPhase1();
  if (status != XhciStatus::kOk) {
    state_ = XhciState::kError;
    StartLogLine().Str("error: ").Str(XhciStatusName(status)).Emit(*platform_);
    return status;
  }
  state_ = XhciState::kReset;
  return XhciStatus::kOk;
}

inline XhciStatus XhciController::RunPhase1() {
  const StatusOr<XhciCaps, XhciStatus> decoded = DecodeCaps(*mmio_);
  if (!decoded.ok()) {
    return decoded.status();
  }
  caps_ = *decoded;
  if (const XhciStatus status = ParseExtendedCaps(*mmio_, &caps_);
      status != XhciStatus::kOk) {
    return status;
  }
  if (ReadOp(kOpUsbSts) == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  if (const XhciStatus status = TakeOwnershipFromFirmware();
      status != XhciStatus::kOk) {
    return status;
  }
  if (const XhciStatus status = WaitUntilReady(); status != XhciStatus::kOk) {
    return status;
  }
  if (const XhciStatus status = HaltIfRunning(); status != XhciStatus::kOk) {
    return status;
  }
  return ResetHalted();
}

inline XhciStatus XhciController::TakeOwnershipFromFirmware() {
  const int64_t legsup = caps_.legacy_offset;
  if (legsup == 0) {
    handoff_ = HandoffOutcome::kNoLegacyCapability;
    return XhciStatus::kOk;
  }
  const uint32_t value = mmio_->Read32(legsup + kLegSup);
  if (value == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  if ((value & kLegSupOsOwned) == 0) {
    mmio_->Write32(legsup + kLegSup, value | kLegSupOsOwned);
  }
  if ((value & kLegSupBiosOwned) == 0) {
    handoff_ = HandoffOutcome::kNotBiosOwned;
  } else {
    const int64_t start = platform_->NowTicks();
    const PollResult released =
        Poll(legsup + kLegSup, kLegSupBiosOwned, 0, kHandoffTimeoutMs);
    if (released == PollResult::kNotDecoding) {
      return XhciStatus::kNotDecoding;
    }
    if (released == PollResult::kMatched) {
      handoff_ = HandoffOutcome::kReleased;
      StartLogLine()
          .Str("firmware released ownership after ")
          .Dec(TicksToMs(*platform_, platform_->NowTicks() - start))
          .Str(" ms")
          .Emit(*platform_);
    } else {
      const uint32_t current = mmio_->Read32(legsup + kLegSup);
      if (current == kMmioAllOnes) {
        return XhciStatus::kNotDecoding;
      }
      mmio_->Write32(legsup + kLegSup,
                     (current | kLegSupOsOwned) & ~kLegSupBiosOwned);
      handoff_ = HandoffOutcome::kForced;
      StartLogLine()
          .Str("firmware handoff timeout after ")
          .Dec(kHandoffTimeoutMs)
          .Str(" ms, forcing OS ownership")
          .Emit(*platform_);
    }
  }
  const uint32_t control = mmio_->Read32(legsup + kLegCtlSts);
  if (control == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  mmio_->Write32(legsup + kLegCtlSts, LegacyControlDisablingSmis(control));
  return XhciStatus::kOk;
}

inline XhciStatus XhciController::WaitUntilReady() {
  const PollResult ready =
      Poll(caps_.Op(kOpUsbSts), kUsbStsCnr, 0, kControllerReadyTimeoutMs);
  if (ready == PollResult::kNotDecoding) {
    return XhciStatus::kNotDecoding;
  }
  if (ready == PollResult::kTimedOut) {
    StartLogLine()
        .Str("controller not ready after ")
        .Dec(kControllerReadyTimeoutMs)
        .Str(" ms, continuing")
        .Emit(*platform_);
  }
  return XhciStatus::kOk;
}

inline XhciStatus XhciController::HaltIfRunning() {
  const uint32_t command = ReadOp(kOpUsbCmd);
  if (command == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  if ((command & kUsbCmdHaltMask) != 0) {
    WriteOp(kOpUsbCmd, command & ~kUsbCmdHaltMask);
  }
  const uint32_t status = ReadOp(kOpUsbSts);
  if (status == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  if ((status & kUsbStsHch) != 0) {
    return XhciStatus::kOk;
  }
  const PollResult halted =
      Poll(caps_.Op(kOpUsbSts), kUsbStsHch, kUsbStsHch, kHaltTimeoutMs);
  if (halted == PollResult::kNotDecoding) {
    return XhciStatus::kNotDecoding;
  }
  if (halted == PollResult::kTimedOut) {
    return XhciStatus::kHaltTimeout;
  }
  StartLogLine().Str("controller was running, halted").Emit(*platform_);
  return XhciStatus::kOk;
}

inline XhciStatus XhciController::ResetHalted() {
  const uint32_t command = ReadOp(kOpUsbCmd);
  if (command == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  WriteOp(kOpUsbCmd, (command & ~(kUsbCmdHaltMask | kUsbCmdLhcrst | kUsbCmdCss |
                                  kUsbCmdCrs)) |
                         kUsbCmdHcrst);
  DelayMs(*platform_, kPostResetDelayMs);
  const PollResult reset =
      Poll(caps_.Op(kOpUsbCmd), kUsbCmdHcrst, 0, kResetTimeoutMs);
  if (reset == PollResult::kNotDecoding) {
    return XhciStatus::kNotDecoding;
  }
  if (reset == PollResult::kTimedOut) {
    return XhciStatus::kResetTimeout;
  }
  const PollResult ready =
      Poll(caps_.Op(kOpUsbSts), kUsbStsCnr, 0, kResetReadyTimeoutMs);
  if (ready == PollResult::kNotDecoding) {
    return XhciStatus::kNotDecoding;
  }
  if (ready == PollResult::kTimedOut) {
    return XhciStatus::kNotReadyAfterReset;
  }
  const uint32_t status = ReadOp(kOpUsbSts);
  if (status == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  if ((status & kUsbStsHch) == 0) {
    return XhciStatus::kRunningAfterReset;
  }
  return XhciStatus::kOk;
}

inline XhciController::PollResult XhciController::Poll(
    const int64_t offset,     //
    const uint32_t mask,      //
    const uint32_t expected,  //
    const int64_t timeout_ms) {
  PollResult result = PollResult::kTimedOut;
  WaitUntil(*platform_, timeout_ms, [&]() {
    const uint32_t value = mmio_->Read32(offset);
    if (value == kMmioAllOnes) {
      result = PollResult::kNotDecoding;
      return true;
    }
    if ((value & mask) == expected) {
      result = PollResult::kMatched;
      return true;
    }
    return false;
  });
  return result;
}

inline LogLine XhciController::StartLogLine() const {
  LogLine line;
  line.Str(kLogPrefix).Str(label_).Str(": ");
  return line;
}

namespace ctrl_internal {

constexpr int64_t kPageBytes = 4096;
constexpr int kFixedCorePages = 3 + 5 * kMaxDevices;  // 83 pages

inline bool ValidDmaBuffer(const DmaBuffer& buf,  //
                           const int64_t bytes,   //
                           const bool below_4gib) {
  if (buf.va == nullptr || bytes <= 0 || (buf.phys & 0xFFFull) != 0) {
    return false;
  }
  const uint64_t ubytes = static_cast<uint64_t>(bytes);
  if (buf.phys + ubytes < buf.phys) {
    return false;
  }
  if (below_4gib && buf.phys > (0x1'0000'0000ULL - ubytes)) {
    return false;
  }
  return true;
}

inline void ZeroBytesVolatile(uint8_t* const base, const int64_t bytes) {
  for (int64_t i = 0; i < bytes; ++i) {
    *reinterpret_cast<volatile uint8_t*>(base + i) = 0;
  }
}

inline StatusOr<uint16_t> PopHubPortClearFeature(uint16_t* const mask) {
  if (mask == nullptr || *mask == 0) {
    return kStatusError;
  }
  constexpr uint16_t kChangeBitToFeature[8] = {
      kHubPortFeatCConnection,   // bit 0: C_PORT_CONNECTION (16)
      kHubPortFeatCEnable,       // bit 1: C_PORT_ENABLE (17)
      kHubPortFeatCSuspend,      // bit 2: C_PORT_SUSPEND (18)
      kHubPortFeatCOverCurrent,  // bit 3: C_PORT_OVER_CURRENT (19)
      kHubPortFeatCReset,        // bit 4: C_PORT_RESET (20)
      kHubPortFeatCBhPortReset,  // bit 5: C_BH_PORT_RESET (29)
      kHubPortFeatCLinkState,    // bit 6: C_PORT_LINK_STATE (25)
      kHubPortFeatCConfigError,  // bit 7: C_PORT_CONFIG_ERROR (26)
  };
  for (int bit = 0; bit < 8; ++bit) {
    const uint16_t bit_mask = static_cast<uint16_t>(1u << bit);
    if ((*mask & bit_mask) != 0) {
      *mask &= ~bit_mask;
      return kChangeBitToFeature[bit];
    }
  }
  *mask = 0;
  return kStatusError;
}

inline uint16_t ChangeBitForHubFeature(const uint16_t feature) {
  switch (feature) {
    case kHubPortFeatCConnection:
      return kHubPortChangeConnection;
    case kHubPortFeatCEnable:
      return kHubPortChangeEnable;
    case kHubPortFeatCSuspend:
      return kHubPortChangeSuspend;
    case kHubPortFeatCOverCurrent:
      return kHubPortChangeOverCurrent;
    case kHubPortFeatCReset:
      return kHubPortChangeReset;
    case kHubPortFeatCBhPortReset:
      return kHubPortChangeBhReset;
    case kHubPortFeatCLinkState:
      return kHubPortChangeLinkState;
    case kHubPortFeatCConfigError:
      return kHubPortChangeConfigError;
    default:
      return 0;
  }
}

inline int DecodeHubPortSpeed(const int hub_speed, const uint16_t status) {
  if (hub_speed >= kPortSpeedSuper) {
    return kPortSpeedSuper;
  }
  if ((status & kHubPortStatUsb2LowSpeed) != 0) {
    return kPortSpeedLow;
  }
  if ((status & kHubPortStatUsb2HighSpeed) != 0) {
    return kPortSpeedHigh;
  }
  return kPortSpeedFull;
}

}  // namespace ctrl_internal

inline XhciStatus XhciController::EnsureDmaAllocated() {
  if (dma_ready_) {
    return XhciStatus::kOk;
  }
  const bool below_4gib = !caps_.ac64;
  const int sp_count = caps_.max_scratchpads;
  const int sp_array_pages = sp_count > 0 ? (sp_count * 8 + 4095) / 4096 : 0;
  const int total_core_pages = ctrl_internal::kFixedCorePages + sp_array_pages;
  const int64_t core_bytes =
      static_cast<int64_t>(total_core_pages) * ctrl_internal::kPageBytes;

  const DmaBuffer core = platform_->DmaAlloc(core_bytes, below_4gib);
  if (!ctrl_internal::ValidDmaBuffer(core, core_bytes, below_4gib)) {
    return XhciStatus::kNoMemory;
  }
  ctrl_internal::ZeroBytesVolatile(static_cast<uint8_t*>(core.va), core_bytes);

  uint64_t sp_array_phys = 0;
  if (sp_count > 0) {
    uint64_t* const sp_array = reinterpret_cast<uint64_t*>(
        static_cast<uint8_t*>(core.va) +
        ctrl_internal::kFixedCorePages * ctrl_internal::kPageBytes);
    sp_array_phys =
        core.phys + static_cast<uint64_t>(ctrl_internal::kFixedCorePages) *
                        ctrl_internal::kPageBytes;
    int allocated = 0;
    while (allocated < sp_count) {
      const int batch =
          (sp_count - allocated) > 256 ? 256 : (sp_count - allocated);
      const int64_t batch_bytes =
          static_cast<int64_t>(batch) * ctrl_internal::kPageBytes;
      const DmaBuffer sp_buf = platform_->DmaAlloc(batch_bytes, below_4gib);
      if (!ctrl_internal::ValidDmaBuffer(sp_buf, batch_bytes, below_4gib)) {
        return XhciStatus::kNoMemory;
      }
      ctrl_internal::ZeroBytesVolatile(static_cast<uint8_t*>(sp_buf.va),
                                       batch_bytes);
      for (int j = 0; j < batch; ++j) {
        sp_array[allocated + j] =
            sp_buf.phys + static_cast<uint64_t>(j) * ctrl_internal::kPageBytes;
      }
      allocated += batch;
    }
  }

  core_dma_va_ = static_cast<uint8_t*>(core.va);
  core_dma_phys_ = core.phys;
  core_dma_bytes_ = static_cast<int64_t>(ctrl_internal::kFixedCorePages) *
                    ctrl_internal::kPageBytes;
  dcbaa_ = reinterpret_cast<uint64_t*>(core_dma_va_);
  dcbaa_phys_ = core_dma_phys_;
  sp_array_phys_ = sp_array_phys;
  dma_ready_ = true;
  return XhciStatus::kOk;
}

inline void XhciController::FormatCoreDma() {
  ctrl_internal::ZeroBytesVolatile(core_dma_va_, core_dma_bytes_);
  dcbaa_[0] = sp_array_phys_;

  ErstEntry* const erst = reinterpret_cast<ErstEntry*>(core_dma_va_ + 0x800);
  const uint64_t erst_phys = core_dma_phys_ + 0x800;

  Trb* const cmd_va =
      reinterpret_cast<Trb*>(core_dma_va_ + 1 * ctrl_internal::kPageBytes);
  const uint64_t cmd_phys = core_dma_phys_ + 1 * ctrl_internal::kPageBytes;
  cmd_ring_.Init(cmd_va, cmd_phys);

  Trb* const evt_va =
      reinterpret_cast<Trb*>(core_dma_va_ + 2 * ctrl_internal::kPageBytes);
  const uint64_t evt_phys = core_dma_phys_ + 2 * ctrl_internal::kPageBytes;
  event_ring_.Init(evt_va, evt_phys, erst, erst_phys);

  for (int i = 0; i < kMaxDevices; ++i) {
    DeviceSlot& dev = devices_[i];
    dev = DeviceSlot{};
    const int base_page = 3 + 5 * i;
    uint8_t* const p0_va = core_dma_va_ + base_page * ctrl_internal::kPageBytes;
    const uint64_t p0_phys = core_dma_phys_ + static_cast<uint64_t>(base_page) *
                                                  ctrl_internal::kPageBytes;
    dev.out_ctx_va = p0_va;
    dev.out_ctx_phys = p0_phys;
    dev.ctrl_buf_va = p0_va + 0x800;
    dev.ctrl_buf_phys = p0_phys + 0x800;
    dev.in_ctx_va = core_dma_va_ + (base_page + 1) * ctrl_internal::kPageBytes;
    dev.in_ctx_phys = core_dma_phys_ + static_cast<uint64_t>(base_page + 1) *
                                           ctrl_internal::kPageBytes;
    dev.ep0_ring_va = reinterpret_cast<Trb*>(
        core_dma_va_ + (base_page + 2) * ctrl_internal::kPageBytes);
    dev.ep0_ring_phys = core_dma_phys_ + static_cast<uint64_t>(base_page + 2) *
                                             ctrl_internal::kPageBytes;
    dev.intr_ring_va = reinterpret_cast<Trb*>(
        core_dma_va_ + (base_page + 3) * ctrl_internal::kPageBytes);
    dev.intr_ring_phys = core_dma_phys_ + static_cast<uint64_t>(base_page + 3) *
                                              ctrl_internal::kPageBytes;
    dev.report_buf_va =
        core_dma_va_ + (base_page + 4) * ctrl_internal::kPageBytes;
    dev.report_buf_phys =
        core_dma_phys_ +
        static_cast<uint64_t>(base_page + 4) * ctrl_internal::kPageBytes;
  }
  DmaBarrier();
}

inline XhciStatus XhciController::Start(const IrqMode mode) {
  if (platform_ == nullptr || mmio_ == nullptr) {
    return XhciStatus::kNotBound;
  }
  if (state_ != XhciState::kReset) {
    state_ = XhciState::kError;
    StartLogLine()
        .Str("error: ")
        .Str(XhciStatusName(XhciStatus::kBadState))
        .Emit(*platform_);
    return XhciStatus::kBadState;
  }
  const XhciStatus status = RunStart(mode);
  if (status != XhciStatus::kOk) {
    state_ = XhciState::kError;
    StartLogLine().Str("error: ").Str(XhciStatusName(status)).Emit(*platform_);
    return status;
  }
  return XhciStatus::kOk;
}

inline XhciStatus XhciController::RunStart(const IrqMode mode) {
  const uint32_t page_size = ReadOp(kOpPageSize);
  if (page_size == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  if ((page_size & kPageSize4K) == 0) {
    return XhciStatus::kUnsupportedPageSize;
  }
  const uint32_t sts = ReadOp(kOpUsbSts);
  if (sts == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  if ((sts & (kUsbStsCnr | kUsbStsHch)) != kUsbStsHch) {
    return XhciStatus::kBadState;
  }

  if (const XhciStatus dma_status = EnsureDmaAllocated();
      dma_status != XhciStatus::kOk) {
    return dma_status;
  }
  FormatCoreDma();

  ports_ = caps_.max_ports < kMaxPorts ? caps_.max_ports : kMaxPorts;
  slots_ = caps_.max_slots < kMaxSlotIds ? caps_.max_slots : kMaxSlotIds;
  attached_keyboards_ = 0;
  event_ring_full_rescan_ = false;
  pending_recovery_ = nullptr;
  // HCRST disabled every slot, and FormatCoreDma() freed every DeviceSlot.
  pending_disable_count_ = 0;
  for (int s = 0; s <= kMaxSlotIds; ++s) {
    disable_queued_[s] = false;
    disable_attempts_[s] = 0;
  }
  cmd_state_ = CommandState{};
  for (int i = 0; i < kMaxPorts; ++i) {
    port_states_[i] = PortState{};
  }
  if (!in_recovery_) {
    recoveries_ = 0;
  }

  const uint32_t cfg = ReadOp(kOpConfig);
  if (cfg == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  WriteOp(kOpConfig,
          (cfg & ~(kConfigMaxSlotsEnMask | kConfigU3e | kConfigCie)) |
              static_cast<uint32_t>(slots_));

  Write64LoHi(*mmio_, caps_.Op(kOpDcbaap), dcbaa_phys_);
  Write64LoHi(*mmio_, caps_.Op(kOpCrcr),
              cmd_ring_.phys() | static_cast<uint64_t>(kCrcrRcs));

  mmio_->Write32(caps_.InterrupterReg(0, kIrImod), kImodDefault);
  const uint32_t erstsz = mmio_->Read32(caps_.InterrupterReg(0, kIrErstSz));
  if (erstsz == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  mmio_->Write32(caps_.InterrupterReg(0, kIrErstSz),
                 (erstsz & ~kErstSzMask) | 1u);
  Write64LoHi(*mmio_, caps_.InterrupterReg(0, kIrErdp),
              event_ring_.dequeue_phys());
  Write64LoHi(*mmio_, caps_.InterrupterReg(0, kIrErstBa),
              event_ring_.erst_phys());

  const uint32_t iman = mmio_->Read32(caps_.InterrupterReg(0, kIrIman));
  if (iman == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  mmio_->Write32(caps_.InterrupterReg(0, kIrIman),
                 (iman & ~kImanIp) | kImanIe | kImanIp);

  WriteOp(kOpUsbSts, kUsbStsEint | kUsbStsPcd | kUsbStsHse);

  const uint32_t cmd = ReadOp(kOpUsbCmd);
  if (cmd == kMmioAllOnes) {
    return XhciStatus::kNotDecoding;
  }
  WriteOp(kOpUsbCmd,
          (cmd & ~(kUsbCmdLhcrst | kUsbCmdCss | kUsbCmdCrs | kUsbCmdHcrst)) |
              kUsbCmdInte | kUsbCmdHsee | kUsbCmdRs);

  const PollResult running =
      Poll(caps_.Op(kOpUsbSts), kUsbStsHch, 0, kStartTimeoutMs);
  if (running == PollResult::kNotDecoding) {
    return XhciStatus::kNotDecoding;
  }
  if (running == PollResult::kTimedOut) {
    return XhciStatus::kStartTimeout;
  }

  if (caps_.ppc) {
    for (int port = 1; port <= ports_; ++port) {
      const uint32_t ps = ReadPortSc(port);
      if (ps == kMmioAllOnes) {
        return XhciStatus::kNotDecoding;
      }
      if ((ps & kPortScPp) == 0) {
        WritePortSc(port, (ps & kPortScNeutralMask) | kPortScPp);
      }
    }
    DelayMs(*platform_, kPortPowerSettleMs);
  }

  irq_mode_ = mode;
  state_ = XhciState::kRunning;
  LogLine()
      .Str(kLogPrefix)
      .Str("controller ")
      .Str(label_)
      .Str(" ready (")
      .Str(IrqModeName(mode))
      .Str(")")
      .Emit(*platform_);

  const int64_t now = platform_->NowTicks();
  for (int port = 1; port <= ports_; ++port) {
    ScanPort(port, now);
  }
  AdvanceStateMachines(now);
  return XhciStatus::kOk;
}

inline void XhciController::HandleInterrupt() {
  if (state_ != XhciState::kRunning || platform_ == nullptr ||
      mmio_ == nullptr) {
    return;
  }
  const uint32_t sts = ReadOp(kOpUsbSts);
  if (sts == kMmioAllOnes) {
    DetachAllKeyboards();
    StartLogLine()
        .Str("error: ")
        .Str(XhciStatusName(XhciStatus::kNotDecoding))
        .Emit(*platform_);
    state_ = XhciState::kError;
    return;
  }
  if ((sts & (kUsbStsHse | kUsbStsHce)) != 0) {
    if ((sts & kUsbStsHse) != 0) {
      WriteOp(kOpUsbSts, kUsbStsHse);
    }
    Recover((sts & kUsbStsHce) != 0 ? "host controller error"
                                    : "host system error");
    return;
  }
  if ((sts & kUsbStsEint) != 0) {
    WriteOp(kOpUsbSts, kUsbStsEint);
  }

  const uint32_t iman = mmio_->Read32(caps_.InterrupterReg(0, kIrIman));
  if (iman == kMmioAllOnes) {
    DetachAllKeyboards();
    StartLogLine()
        .Str("error: ")
        .Str(XhciStatusName(XhciStatus::kNotDecoding))
        .Emit(*platform_);
    state_ = XhciState::kError;
    return;
  }
  mmio_->Write32(caps_.InterrupterReg(0, kIrIman),
                 (iman & ~kImanIp) | kImanIe | kImanIp);

  int drained = 0;
  while (drained < kRingTrbs) {
    const StatusOr<Trb> ev = event_ring_.Pop();
    if (!ev.ok()) {
      break;
    }
    ++drained;
    DispatchEvent(*ev);
    if (state_ != XhciState::kRunning) {
      return;
    }
    if (pending_recovery_ != nullptr) {
      break;  // The reset below discards the rest of the ring.
    }
  }

  Write64LoHi(*mmio_, caps_.InterrupterReg(0, kIrErdp),
              event_ring_.dequeue_phys() | static_cast<uint64_t>(kErdpEhb));

  AdvanceStateMachines(platform_->NowTicks());
}

inline int64_t XhciController::NextDeadline() const {
  if (state_ != XhciState::kRunning) {
    return kNoDeadline;
  }
  int64_t earliest = kNoDeadline;
  if (cmd_state_.busy && cmd_state_.deadline < earliest) {
    earliest = cmd_state_.deadline;
  }
  for (int p = 0; p < ports_; ++p) {
    if (port_states_[p].deadline < earliest) {
      earliest = port_states_[p].deadline;
    }
  }
  for (int i = 0; i < kMaxDevices; ++i) {
    if (!devices_[i].in_use) {
      continue;
    }
    if (devices_[i].deadline < earliest) {
      earliest = devices_[i].deadline;
    }
    // AdvanceStateMachines() services hub ports only while the hub is
    // configured (not while it recovers an endpoint).
    if (devices_[i].is_hub && devices_[i].state == DevFsm::kConfigured) {
      if (devices_[i].hub_rescan_deadline < earliest) {
        earliest = devices_[i].hub_rescan_deadline;
      }
      for (int p = 0; p < devices_[i].hub_num_ports; ++p) {
        if (devices_[i].hub_ports[p].deadline < earliest) {
          earliest = devices_[i].hub_ports[p].deadline;
        }
      }
    }
  }
  return earliest;
}

inline void XhciController::DispatchEvent(const Trb& ev) {
  const int64_t now = platform_->NowTicks();
  switch (TrbType(ev)) {
    case kTrbPortStatusChange: {
      const int port = TrbPortId(ev);
      if (port >= 1 && port <= ports_) {
        ScanPort(port, now);
      } else {
        for (int p = 1; p <= ports_; ++p) {
          ScanPort(p, now);
        }
      }
      break;
    }
    case kTrbCommandCompletion:
      OnCommandCompletion(ev, now);
      break;
    case kTrbTransferEvent:
      OnTransferEvent(ev, now);
      break;
    case kTrbHostControllerEvent:
      if (TrbCompletionCode(ev) == kCcEventRingFull) {
        event_ring_full_rescan_ = true;
      }
      break;
    default:
      break;
  }
}

inline void XhciController::ScanPort(const int port, const int64_t now) {
  if (port < 1 || port > ports_) {
    return;
  }
  const uint32_t ps = ReadPortSc(port);
  if (ps == kMmioAllOnes) {
    return;
  }
  const uint32_t changes = ps & kPortScChangeMask;
  // Over-current on a port with PPC = 1 clears PP (xHCI 4.19.4); power it
  // back on once OCA is 0 again.
  const uint32_t power =
      caps_.ppc && (ps & kPortScPp) == 0 && (ps & kPortScOca) == 0 ? kPortScPp
                                                                   : 0u;
  if (changes != 0 || power != 0) {
    WritePortSc(port, (ps & kPortScNeutralMask) | changes | power);
  }
  PortState& p = port_states_[port - 1];
  const bool connected = (ps & kPortScCcs) != 0;
  const bool csc = (changes & kPortScCsc) != 0;
  const bool prc = (changes & (kPortScPrc | kPortScWrc)) != 0;

  if (!connected) {
    if (p.fsm != PortFsm::kDisconnected || p.dev_idx >= 0) {
      HandlePortDisconnect(port, now);
    }
    return;
  }

  if (csc && p.fsm != PortFsm::kDisconnected && p.fsm != PortFsm::kDebouncing) {
    HandlePortDisconnect(port, now);
  }

  if (p.fsm == PortFsm::kDisconnected) {
    p.fsm = PortFsm::kDebouncing;
    p.debounce_start = now;
    p.deadline = now + TicksForMs(*platform_, kPortDebounceMs);
    p.attempts = 0;
    return;
  }

  if (p.fsm == PortFsm::kDebouncing && csc) {
    if (now - p.debounce_start >= TicksForMs(*platform_, kPortDebounceMaxMs)) {
      StartLogLine()
          .Str("port ")
          .Dec(port)
          .Str(": debounce timeout")
          .Emit(*platform_);
      p.fsm = PortFsm::kDisabled;
      p.deadline = kNoDeadline;
      return;
    }
    p.deadline = now + TicksForMs(*platform_, kPortDebounceMs);
    return;
  }

  if (p.fsm == PortFsm::kResetting &&
      (prc || ((ps & kPortScPr) == 0 && (ps & kPortScPed) != 0))) {
    p.speed = PortSpeedOf(ps);
    if ((ps & kPortScPed) == 0 || p.speed == 0) {
      RetryOrDisablePort(port, now, "port reset did not enable port");
    } else {
      p.fsm = PortFsm::kResetRecovery;
      p.deadline = now + TicksForMs(*platform_, kPortResetRecoveryMs);
    }
  }
}

inline void XhciController::AnnounceKeyboardAttach(DeviceSlot& dev) {
  if (!dev.announced) {
    dev.announced = true;
    ++attached_keyboards_;
    LogLine line;
    line.Str(kLogPrefix).Str("keyboard attached: ").Str(label_).Str(" port ");
    AppendPortPath(line, dev.port, dev.route_string)
        .Str(" (")
        .Str(SpeedName(dev.speed))
        .Str(")")
        .Emit(*platform_);
  }
}

inline void XhciController::AnnounceKeyboardDetach(DeviceSlot& dev) {
  if (dev.announced) {
    dev.announced = false;
    if (attached_keyboards_ > 0) {
      --attached_keyboards_;
    }
    LogLine line;
    line.Str(kLogPrefix).Str("keyboard detached: ").Str(label_).Str(" port ");
    AppendPortPath(line, dev.port, dev.route_string).Emit(*platform_);
  }
}

inline void XhciController::MarkParentPortActive(DeviceSlot& dev,
                                                 const int64_t now) {
  if (dev.parent_dev_idx >= 0 && dev.parent_dev_idx < kMaxDevices) {
    DeviceSlot& parent = devices_[dev.parent_dev_idx];
    if (parent.in_use && parent.is_hub && dev.parent_port >= 1 &&
        dev.parent_port <= parent.hub_num_ports) {
      HubPortState& hp = parent.hub_ports[dev.parent_port - 1];
      hp.fsm = HubPortFsm::kActive;
      hp.deadline = kNoDeadline;
      hp.active_since = now;
    }
  } else if (dev.port >= 1 && dev.port <= ports_) {
    port_states_[dev.port - 1].fsm = PortFsm::kActive;
    port_states_[dev.port - 1].deadline = kNoDeadline;
    port_states_[dev.port - 1].active_since = now;
  }
}

inline void XhciController::DetachDeviceTree(const int dev_idx) {
  if (dev_idx < 0 || dev_idx >= kMaxDevices) {
    return;
  }
  DeviceSlot& dev = devices_[dev_idx];
  if (!dev.in_use) {
    return;
  }
  if (dev.is_hub) {
    for (int p = 0; p < kMaxHubPorts; ++p) {
      const int child_idx = dev.hub_ports[p].dev_idx;
      dev.hub_ports[p] = HubPortState{};
      if (child_idx >= 0 && child_idx < kMaxDevices) {
        DetachDeviceTree(child_idx);
      }
    }
    for (int i = 0; i < kMaxDevices; ++i) {
      if (i != dev_idx && devices_[i].in_use &&
          devices_[i].parent_dev_idx == dev_idx) {
        DetachDeviceTree(i);
      }
    }
  }
  AnnounceKeyboardDetach(dev);
  if (cmd_state_.busy && cmd_state_.dev_idx == dev_idx) {
    cmd_state_.abandoned = true;
  }
  const uint8_t slot_id = dev.slot_id;
  ReleaseDevice(dev_idx);
  if (slot_id != 0) {
    // The slot's DCBAA entry, Output Device Context and transfer rings (and
    // so the buffers their TRBs point to) belong to the xHC until its Disable
    // Slot completes (xHCI 4.6.4, 6.1): park the DeviceSlot until then.
    devices_[dev_idx].disabling_slot_id = slot_id;
    QueueDisableSlot(slot_id, /*front=*/false);
  }
}

inline void XhciController::QueueDisableSlot(const uint8_t slot_id,
                                             const bool front) {
  if (slot_id < 1 || slot_id > kMaxSlotIds || disable_queued_[slot_id] ||
      pending_disable_count_ >= kMaxSlotIds) {
    return;
  }
  disable_queued_[slot_id] = true;
  if (front) {
    for (int k = pending_disable_count_; k > 0; --k) {
      pending_disable_fifo_[k] = pending_disable_fifo_[k - 1];
    }
    pending_disable_fifo_[0] = slot_id;
    ++pending_disable_count_;
  } else {
    pending_disable_fifo_[pending_disable_count_++] = slot_id;
  }
}

inline void XhciController::OnDisableSlotCompletion(const uint8_t slot_id,
                                                    const uint8_t cc) {
  if (slot_id < 1 || slot_id > kMaxSlotIds) {
    return;
  }
  if (cc != kCcSuccess && cc != kCcSlotNotEnabled) {
    RetryDisableSlot(slot_id, "disable slot command error");
    return;
  }
  // The slot is disabled (Slot Not Enabled: it was already): now software
  // clears its DCBAA entry and may reuse its contexts and rings (xHCI 4.6.4).
  disable_queued_[slot_id] = false;
  disable_attempts_[slot_id] = 0;
  dcbaa_[slot_id] = 0;
  DmaBarrier();
  for (int i = 0; i < kMaxDevices; ++i) {
    if (devices_[i].disabling_slot_id == slot_id) {
      devices_[i].disabling_slot_id = 0;
    }
  }
}

// A Disable Slot failed or timed out: the slot may still be enabled, so its
// DeviceSlot stays parked and the command is queued again, ahead of the rest
// (children before their hub). If it keeps failing, only a controller reset
// disables the slot safely.
inline void XhciController::RetryDisableSlot(const uint8_t slot_id,
                                             const char* const reason) {
  StartLogLine()
      .Str("slot ")
      .Dec(slot_id)
      .Str(": ")
      .Str(reason)
      .Str(" (attempt ")
      .Dec(disable_attempts_[slot_id] + 1)
      .Str(")")
      .Emit(*platform_);
  if (++disable_attempts_[slot_id] >= kMaxDisableSlotAttempts) {
    pending_recovery_ = "disable slot error limit";
    return;
  }
  disable_queued_[slot_id] = false;
  QueueDisableSlot(slot_id, /*front=*/true);
}

inline int XhciController::FindFreeDevice() const {
  for (int i = 0; i < kMaxDevices; ++i) {
    if (!devices_[i].in_use && devices_[i].disabling_slot_id == 0) {
      return i;
    }
  }
  return -1;
}

inline bool XhciController::AnyDeviceDisabling() const {
  for (int i = 0; i < kMaxDevices; ++i) {
    if (devices_[i].disabling_slot_id != 0) {
      return true;
    }
  }
  return false;
}

inline void XhciController::HandlePortDisconnect(const int port,
                                                 const int64_t now) {
  (void)now;
  PortState& p = port_states_[port - 1];
  if (p.dev_idx >= 0 && p.dev_idx < kMaxDevices) {
    const int dev_idx = p.dev_idx;
    p.dev_idx = -1;
    DetachDeviceTree(dev_idx);
  }
  for (int i = 0; i < kMaxDevices; ++i) {
    if (devices_[i].in_use && devices_[i].port == port) {
      DetachDeviceTree(i);
    }
  }
  p.fsm = PortFsm::kDisconnected;
  p.speed = 0;
  p.attempts = 0;
  p.dev_idx = -1;
  p.debounce_start = 0;
  p.deadline = kNoDeadline;
}

inline void XhciController::ReleaseDevice(const int dev_idx) {
  if (dev_idx < 0 || dev_idx >= kMaxDevices) {
    return;
  }
  DeviceSlot& dev = devices_[dev_idx];
  uint8_t* const out_ctx_va = dev.out_ctx_va;
  const uint64_t out_ctx_phys = dev.out_ctx_phys;
  uint8_t* const in_ctx_va = dev.in_ctx_va;
  const uint64_t in_ctx_phys = dev.in_ctx_phys;
  uint8_t* const ctrl_buf_va = dev.ctrl_buf_va;
  const uint64_t ctrl_buf_phys = dev.ctrl_buf_phys;
  Trb* const ep0_ring_va = dev.ep0_ring_va;
  const uint64_t ep0_ring_phys = dev.ep0_ring_phys;
  Trb* const intr_ring_va = dev.intr_ring_va;
  const uint64_t intr_ring_phys = dev.intr_ring_phys;
  uint8_t* const report_buf_va = dev.report_buf_va;
  const uint64_t report_buf_phys = dev.report_buf_phys;

  dev = DeviceSlot{};
  dev.out_ctx_va = out_ctx_va;
  dev.out_ctx_phys = out_ctx_phys;
  dev.in_ctx_va = in_ctx_va;
  dev.in_ctx_phys = in_ctx_phys;
  dev.ctrl_buf_va = ctrl_buf_va;
  dev.ctrl_buf_phys = ctrl_buf_phys;
  dev.ep0_ring_va = ep0_ring_va;
  dev.ep0_ring_phys = ep0_ring_phys;
  dev.intr_ring_va = intr_ring_va;
  dev.intr_ring_phys = intr_ring_phys;
  dev.report_buf_va = report_buf_va;
  dev.report_buf_phys = report_buf_phys;
}

inline void XhciController::StartPortResetOrUsb3(const int port,
                                                 const int64_t now) {
  const uint32_t ps = ReadPortSc(port);
  if (ps == kMmioAllOnes || (ps & kPortScCcs) == 0) {
    HandlePortDisconnect(port, now);
    return;
  }
  if ((ps & kPortScChangeMask) != 0) {
    WritePortSc(port, (ps & kPortScNeutralMask) | (ps & kPortScChangeMask));
  }
  PortState& p = port_states_[port - 1];
  const int major = caps_.PortProtocolMajor(port);
  const int speed = PortSpeedOf(ps);
  const bool usb3 = major == 3 || speed >= kPortSpeedSuper;
  const uint32_t pls = PortLinkStateOf(ps);
  if (usb3 && (pls == kPlsInactive || pls == kPlsComplianceMode)) {
    // Only a Warm Reset recovers such a link (xHCI 4.19.5.1).
    WritePortSc(port, (ps & kPortScNeutralMask) | kPortScWpr);
    p.fsm = PortFsm::kResetting;
    p.deadline = now + TicksForMs(*platform_, kPortResetTimeoutMs);
    return;
  }
  // Link training (and HCRST) leaves a USB 3 device in the Default state, so
  // the first attempt needs no reset. A retry does: Disable Slot doesn't
  // touch the device, which still has the address of the failed attempt and
  // would ignore the next SET_ADDRESS to the default address. PR on a USB 3
  // port is a Hot Reset (xHCI 5.4.8), as Linux issues before every attempt.
  if (usb3 && p.attempts == 0 && (ps & kPortScPed) != 0 && speed != 0) {
    p.speed = speed;
    p.fsm = PortFsm::kEnumerating;
    p.deadline = kNoDeadline;
    BeginDeviceEnumeration(port, now);
    return;
  }
  WritePortSc(port, (ps & kPortScNeutralMask) | kPortScPr);
  p.fsm = PortFsm::kResetting;
  p.deadline = now + TicksForMs(*platform_, kPortResetTimeoutMs);
}

inline void XhciController::RetryOrDisablePort(const int port,     //
                                               const int64_t now,  //
                                               const char* const reason) {
  PortState& p = port_states_[port - 1];
  if (p.dev_idx >= 0 && p.dev_idx < kMaxDevices) {
    const int dev_idx = p.dev_idx;
    p.dev_idx = -1;
    DetachDeviceTree(dev_idx);
  }

  if (p.active_since != 0 &&
      now - p.active_since >= TicksForMs(*platform_, kPortHealthyMs)) {
    p.attempts = 0;  // A failure long after a successful configuration
  }
  p.active_since = 0;
  ++p.attempts;
  StartLogLine()
      .Str("port ")
      .Dec(port)
      .Str(": ")
      .Str(reason)
      .Str(" (attempt ")
      .Dec(p.attempts)
      .Str(")")
      .Emit(*platform_);

  const uint32_t ps = ReadPortSc(port);
  if (ps == kMmioAllOnes || (ps & kPortScCcs) == 0) {
    HandlePortDisconnect(port, now);
    return;
  }
  if (p.attempts < kMaxEnumerationAttempts) {
    p.fsm = PortFsm::kDebouncing;
    p.debounce_start = now;
    p.deadline = now + TicksForMs(*platform_, kPortDebounceMs);
  } else {
    p.fsm = PortFsm::kDisabled;
    p.deadline = kNoDeadline;
  }
}

inline void XhciController::RetryOrDisableHubPort(const int hub_idx,  //
                                                  const int port,     //
                                                  const int64_t now,  //
                                                  const char* const reason) {
  if (hub_idx < 0 || hub_idx >= kMaxDevices) {
    return;
  }
  DeviceSlot& hub = devices_[hub_idx];
  if (!hub.in_use || !hub.is_hub || port < 1 || port > hub.hub_num_ports) {
    return;
  }
  HubPortState& hp = hub.hub_ports[port - 1];
  if (hp.dev_idx >= 0 && hp.dev_idx < kMaxDevices) {
    const int child_idx = hp.dev_idx;
    hp.dev_idx = -1;
    DetachDeviceTree(child_idx);
  }

  if (hp.active_since != 0 &&
      now - hp.active_since >= TicksForMs(*platform_, kPortHealthyMs)) {
    hp.attempts = 0;  // A failure long after a successful configuration
  }
  hp.active_since = 0;
  ++hp.attempts;
  const uint32_t child_route =
      AppendRouteStringPort(hub.route_string, hub.hub_depth, port);
  LogLine line = StartLogLine();
  line.Str("port ");
  AppendPortPath(line, hub.port, child_route)
      .Str(": ")
      .Str(reason)
      .Str(" (attempt ")
      .Dec(hp.attempts)
      .Str(")")
      .Emit(*platform_);

  if (hp.attempts < kMaxEnumerationAttempts) {
    hp.fsm = HubPortFsm::kDebouncing;
    hp.debounce_start = now;
    hp.deadline = now + TicksForMs(*platform_, kPortDebounceMs);
  } else {
    hp.fsm = HubPortFsm::kDisabled;
    hp.deadline = kNoDeadline;
  }
  // Until the retry resets it, the abandoned device must not answer the
  // address the xHC may hand to the next device (PumpHubControl() step 0a).
  if (hub.speed < kPortSpeedSuper) {
    hp.disable_pending = true;
  }
  PumpCommandRing(now);
  PumpHubControl(hub_idx, now);
}

inline void XhciController::RetryOrDisableDevice(DeviceSlot& dev,    //
                                                 const int64_t now,  //
                                                 const char* const reason) {
  if (dev.parent_dev_idx >= 0) {
    RetryOrDisableHubPort(dev.parent_dev_idx, dev.parent_port, now, reason);
  } else {
    RetryOrDisablePort(dev.port, now, reason);
  }
}

inline void XhciController::DisableUnsupportedDevice(DeviceSlot& dev,
                                                     const char* const reason) {
  const int dev_idx = static_cast<int>(&dev - devices_);
  const int port = dev.port;
  const int parent_idx = dev.parent_dev_idx;
  const int parent_port = dev.parent_port;
  // The IDs (once known) tell what the device is, e.g. on a laptop without a
  // serial console.
  LogLine line = StartLogLine();
  line.Str("port ");
  AppendPortPath(line, port, dev.route_string).Str(": ").Str(reason);
  if (dev.vendor_id != 0 || dev.product_id != 0) {
    line.Str(" (")
        .Hex(dev.vendor_id, 4)
        .Str(":")
        .Hex(dev.product_id, 4)
        .Str(")");
  }
  line.Emit(*platform_);
  DetachDeviceTree(dev_idx);
  if (parent_idx >= 0 && parent_idx < kMaxDevices &&
      devices_[parent_idx].in_use && devices_[parent_idx].is_hub &&
      parent_port >= 1 && parent_port <= devices_[parent_idx].hub_num_ports) {
    HubPortState& hp = devices_[parent_idx].hub_ports[parent_port - 1];
    hp.dev_idx = -1;
    hp.fsm = HubPortFsm::kDisabled;
    hp.deadline = kNoDeadline;
    // The rejected device keeps its USB address, which the xHC may reassign:
    // a USB 2.0 hub would still repeat that traffic to it.
    if (devices_[parent_idx].speed < kPortSpeedSuper) {
      hp.disable_pending = true;
    }
  } else if (port >= 1 && port <= ports_) {
    PortState& p = port_states_[port - 1];
    p.dev_idx = -1;
    p.fsm = PortFsm::kDisabled;
    p.deadline = kNoDeadline;
  }
}

inline void XhciController::BeginDeviceEnumeration(const int port,
                                                   const int64_t now) {
  const int free_idx = FindFreeDevice();
  PortState& p = port_states_[port - 1];
  if (free_idx < 0 && AnyDeviceDisabling()) {
    // One is freed when its Disable Slot completes: try again shortly.
    p.fsm = PortFsm::kResetRecovery;
    p.deadline = now + TicksForMs(*platform_, kAddressSettleMs);
    return;
  }
  if (free_idx < 0) {
    StartLogLine()
        .Str("port ")
        .Dec(port)
        .Str(": no free device slots")
        .Emit(*platform_);
    p.fsm = PortFsm::kDisabled;
    p.deadline = kNoDeadline;
    return;
  }

  ReleaseDevice(free_idx);
  DeviceSlot& dev = devices_[free_idx];
  dev.in_use = true;
  dev.announced = false;
  dev.port = port;
  dev.parent_dev_idx = -1;
  dev.parent_port = 0;
  dev.route_string = 0;
  dev.hub_depth = 0;
  dev.tt_hub_slot_id = 0;
  dev.tt_port_number = 0;
  dev.tt_mtt = false;
  dev.speed = p.speed;
  dev.slot_id = 0;
  dev.ep0_mps = InitialEp0MaxPacketSize(p.speed);
  dev.device_class = 0;
  dev.device_protocol = 0;
  dev.vendor_id = 0;
  dev.product_id = 0;
  dev.is_hub = false;
  dev.state = DevFsm::kWantEnableSlot;
  dev.after_ep0_recovery = Ep0RecoveryNext::kNone;
  dev.ep0_ring.Init(dev.ep0_ring_va, dev.ep0_ring_phys);
  dev.intr_ring.Init(dev.intr_ring_va, dev.intr_ring_phys);
  ZeroContextBuffer(dev.out_ctx_va, 32 * caps_.context_size);
  ZeroContextBuffer(dev.in_ctx_va, 33 * caps_.context_size);
  dev.ctrl_in_flight = false;
  dev.deadline = kNoDeadline;
  dev.kbd = BootKeyboardMatch{};
  dev.next_rearm_idx = 0;
  dev.intr_errors = 0;
  dev.hid_state = HidKeyboardState{};

  p.dev_idx = free_idx;
  PumpCommandRing(now);
}

inline void XhciController::BeginHubDeviceEnumeration(const int hub_idx,
                                                      const int port,
                                                      const int64_t now) {
  if (hub_idx < 0 || hub_idx >= kMaxDevices) {
    return;
  }
  DeviceSlot& hub = devices_[hub_idx];
  if (!hub.in_use || !hub.is_hub || port < 1 || port > hub.hub_num_ports) {
    return;
  }
  HubPortState& hp = hub.hub_ports[port - 1];
  const int free_idx = FindFreeDevice();
  const uint32_t child_route =
      AppendRouteStringPort(hub.route_string, hub.hub_depth, port);
  if (free_idx < 0 && AnyDeviceDisabling()) {
    // One is freed when its Disable Slot completes: try again shortly.
    hp.fsm = HubPortFsm::kResetRecovery;
    hp.deadline = now + TicksForMs(*platform_, kAddressSettleMs);
    return;
  }
  if (free_idx < 0) {
    LogLine line = StartLogLine();
    line.Str("port ");
    AppendPortPath(line, hub.port, child_route)
        .Str(": no free device slots")
        .Emit(*platform_);
    hp.fsm = HubPortFsm::kDisabled;
    hp.deadline = kNoDeadline;
    // The reset device would keep answering the default address 0.
    if (hub.speed < kPortSpeedSuper) {
      hp.disable_pending = true;
    }
    PumpHubControl(hub_idx, now);
    return;
  }

  ReleaseDevice(free_idx);
  DeviceSlot& dev = devices_[free_idx];
  dev.in_use = true;
  dev.announced = false;
  dev.port = hub.port;
  dev.parent_dev_idx = hub_idx;
  dev.parent_port = port;
  dev.route_string = child_route;
  dev.hub_depth = hub.hub_depth + 1;
  dev.speed = hp.speed;
  if (dev.speed == kPortSpeedLow || dev.speed == kPortSpeedFull) {
    if (hub.speed == kPortSpeedHigh) {
      dev.tt_hub_slot_id = hub.slot_id;
      dev.tt_port_number = static_cast<uint8_t>(port);
      dev.tt_mtt = hub.hub_mtt;
    } else {
      dev.tt_hub_slot_id = hub.tt_hub_slot_id;
      dev.tt_port_number = hub.tt_port_number;
      dev.tt_mtt = hub.tt_mtt;
    }
  } else {
    dev.tt_hub_slot_id = 0;
    dev.tt_port_number = 0;
    dev.tt_mtt = false;
  }
  dev.slot_id = 0;
  dev.ep0_mps = InitialEp0MaxPacketSize(dev.speed);
  dev.device_class = 0;
  dev.device_protocol = 0;
  dev.vendor_id = 0;
  dev.product_id = 0;
  dev.is_hub = false;
  dev.state = DevFsm::kWantEnableSlot;
  dev.after_ep0_recovery = Ep0RecoveryNext::kNone;
  dev.ep0_ring.Init(dev.ep0_ring_va, dev.ep0_ring_phys);
  dev.intr_ring.Init(dev.intr_ring_va, dev.intr_ring_phys);
  ZeroContextBuffer(dev.out_ctx_va, 32 * caps_.context_size);
  ZeroContextBuffer(dev.in_ctx_va, 33 * caps_.context_size);
  dev.ctrl_in_flight = false;
  dev.deadline = kNoDeadline;
  dev.kbd = BootKeyboardMatch{};
  dev.next_rearm_idx = 0;
  dev.intr_errors = 0;
  dev.hid_state = HidKeyboardState{};

  hp.dev_idx = free_idx;
  PumpCommandRing(now);
}

inline void XhciController::StartHubConfiguration(DeviceSlot& dev,         //
                                                  const HubMatch& hmatch,  //
                                                  const int64_t now) {
  dev.is_hub = true;
  dev.hub_match = hmatch;
  dev.kbd = BootKeyboardMatch{
      .config_value = hmatch.config_value,
      .interface_number = hmatch.interface_number,
      .ep_address = hmatch.ep_address,
      .ep_dci = hmatch.ep_dci,
      .max_packet_size = hmatch.max_packet_size,
      .max_burst = hmatch.max_burst,
      .b_interval = hmatch.b_interval,
  };
  // A hub's response to its class requests is undefined until it is
  // configured (USB 2.0 11.24.2.5), so SET_CONFIGURATION comes before the
  // hub descriptor, as in Linux and EDK2. The xHC schedules the interrupt
  // endpoint only after Configure Endpoint.
  dev.state = DevFsm::kWaitingHubSetConfig;
  SubmitControlTransfer(dev, kUsbReqTypeStdDevOut, kUsbReqSetConfiguration,
                        hmatch.config_value, 0, 0, now);
}

inline void XhciController::StartHubDescriptorFetch(DeviceSlot& dev,
                                                    const int64_t now) {
  const uint8_t desc_type =
      dev.speed >= kPortSpeedSuper ? kUsbDescSsHub : kUsbDescHub;
  dev.state = DevFsm::kWaitingGetHubDesc;
  SubmitControlTransfer(dev, kUsbReqTypeClassDevIn, kUsbReqGetDescriptor,
                        static_cast<uint16_t>(desc_type << 8), 0, 32, now);
}

inline void XhciController::StartHubPortPower(DeviceSlot& dev,
                                              const int64_t now) {
  dev.hub_init_port = 1;
  dev.state = DevFsm::kWaitingHubPortPower;
  SubmitControlTransfer(dev, kUsbReqTypeClassPortOut, kUsbReqSetFeature,
                        kHubPortFeatPower, 1, 0, now);
}

inline void XhciController::AnnounceHubAttach(const DeviceSlot& dev) {
  LogLine line;
  line.Str(kLogPrefix).Str("hub attached: ").Str(label_).Str(" port ");
  AppendPortPath(line, dev.port, dev.route_string)
      .Str(" (")
      .Str(SpeedName(dev.speed))
      .Str(", ")
      .Dec(dev.hub_num_ports)
      .Str(" ports)");
  if (dev.vendor_id != 0 || dev.product_id != 0) {
    line.Str(" (")
        .Hex(dev.vendor_id, 4)
        .Str(":")
        .Hex(dev.product_id, 4)
        .Str(")");
  }
  line.Emit(*platform_);
}

inline void XhciController::FinishHubAttach(const int dev_idx,
                                            const int64_t now) {
  if (dev_idx < 0 || dev_idx >= kMaxDevices) {
    return;
  }
  DeviceSlot& dev = devices_[dev_idx];
  dev.intr_errors = 0;
  dev.deadline = kNoDeadline;
  dev.state = DevFsm::kConfigured;
  MarkParentPortActive(dev, now);
  dev.hub_ctrl_op = HubCtrlOp::kNone;
  dev.hub_ctrl_port = 0;
  dev.hub_ctrl_feature = 0;
  dev.hub_intr_armed = false;
  dev.hub_status_pending = false;
  dev.hub_pending_hub_clear_mask = 0;
  dev.hub_status_errors = 0;
  dev.hub_status_broken = false;
  dev.intr_recovery_pending = false;
  dev.hub_clear_halt_pending = false;
  dev.ep0_stop_retried = false;
  dev.hub_ctrl_failures = 0;
  dev.hub_fail_window_start = now;
  dev.hub_cold_rescans_left = kMaxHubColdRescans;
  dev.hub_rescan_deadline = kNoDeadline;
  for (int p = 0; p < kMaxHubPorts; ++p) {
    dev.hub_ports[p] = HubPortState{};
  }
  AnnounceHubAttach(dev);
  dev.hub_pending_scan_mask = (uint32_t{1} << dev.hub_num_ports) - 1u;
  PumpHubControl(dev_idx, now);
  if (dev.parent_dev_idx >= 0) {
    PumpHubControl(dev.parent_dev_idx, now);
  }
}

inline bool XhciController::HubAllPortsDisconnected(
    const DeviceSlot& hub) const {
  for (int p = 0; p < hub.hub_num_ports; ++p) {
    const HubPortState& hp = hub.hub_ports[p];
    if (hp.fsm != HubPortFsm::kDisconnected || hp.pending_clear_mask != 0 ||
        hp.disable_pending || hp.power_pending || hp.dev_idx >= 0) {
      return false;
    }
  }
  return true;
}

inline void XhciController::ArmHubInterruptTransfer(DeviceSlot& hub) {
  if (!hub.in_use || !hub.is_hub || hub.state != DevFsm::kConfigured ||
      hub.hub_intr_armed) {
    return;
  }
  const uint32_t xfer_len = hub.kbd.max_packet_size < kReportBufferSize
                                ? hub.kbd.max_packet_size
                                : kReportBufferSize;
  hub.hub_intr_trb_phys = hub.intr_ring.Enqueue(
      MakeNormalTrb(hub.report_buf_phys, xfer_len, true, true));
  hub.hub_intr_armed = true;
  mmio_->Write32(caps_.Doorbell(hub.slot_id), hub.kbd.ep_dci);
}

inline void XhciController::PumpHubControl(const int hub_idx,
                                           const int64_t now) {
  if (hub_idx < 0 || hub_idx >= kMaxDevices) {
    return;
  }
  DeviceSlot& hub = devices_[hub_idx];
  if (!hub.in_use || !hub.is_hub || hub.state != DevFsm::kConfigured ||
      hub.ctrl_in_flight) {
    return;
  }

  // 0a. Disable the port of a device the driver gave up on: a USB 2.0 hub
  //     repeats traffic to every enabled port, so the device would answer
  //     the USB address the xHC reassigns (USB 2.0 11.24.2.2).
  for (int p = 1; p <= hub.hub_num_ports; ++p) {
    if (hub.hub_ports[p - 1].disable_pending) {
      hub.hub_ctrl_op = HubCtrlOp::kDisablePort;
      hub.hub_ctrl_port = static_cast<uint8_t>(p);
      hub.hub_ctrl_feature = kHubPortFeatEnable;
      SubmitControlTransfer(hub, kUsbReqTypeClassPortOut, kUsbReqClearFeature,
                            kHubPortFeatEnable, static_cast<uint16_t>(p), 0,
                            now);
      return;
    }
  }

  // 0b. Re-synchronize the status-change endpoint (after an error or a lost
  //     completion) only now that EP0 is idle, so that its recovery never
  //     overwrites the state of a hub request in flight. The endpoint state
  //     picks the command: Reset Endpoint is valid only on a Halted endpoint
  //     and Set TR Dequeue only on a Stopped one (xHCI 4.6.8, 4.6.10).
  if (hub.intr_recovery_pending) {
    hub.intr_recovery_pending = false;
    StartInterruptEndpointRecovery(hub);
    PumpCommandRing(now);
    return;
  }

  // 0c. A Reset Endpoint with TSP = 0 restarted the host side of the
  //     status-change endpoint at DATA0 (sequence number 0): before it is
  //     re-armed (step 4), CLEAR_FEATURE(ENDPOINT_HALT) clears the hub's Halt
  //     feature (after a STALL) and resets its data toggle too (USB 2.0
  //     9.4.5, xHCI 4.6.8).
  if (hub.hub_clear_halt_pending) {
    hub.hub_clear_halt_pending = false;
    hub.hub_ctrl_op = HubCtrlOp::kClearEndpointHalt;
    hub.hub_ctrl_port = 0;
    hub.hub_ctrl_feature = kUsbFeatEndpointHalt;
    SubmitControlTransfer(hub, kUsbReqTypeStdEndpointOut, kUsbReqClearFeature,
                          kUsbFeatEndpointHalt, hub.kbd.ep_address, 0, now);
    return;
  }

  // 0d. Clear the hub-level change bits that GET_STATUS(hub) returned.
  if (hub.hub_pending_hub_clear_mask != 0) {
    const bool local_power =
        (hub.hub_pending_hub_clear_mask & kHubChangeLocalPower) != 0;
    const uint16_t feature =
        local_power ? kHubFeatCHubLocalPower : kHubFeatCHubOverCurrent;
    hub.hub_pending_hub_clear_mask &= static_cast<uint16_t>(
        ~(local_power ? kHubChangeLocalPower : kHubChangeOverCurrent));
    hub.hub_ctrl_op = HubCtrlOp::kClearHubFeature;
    hub.hub_ctrl_port = 0;
    hub.hub_ctrl_feature = feature;
    SubmitControlTransfer(hub, kUsbReqTypeClassDevOut, kUsbReqClearFeature,
                          feature, 0, 0, now);
    return;
  }

  // 0e. Bit 0 of the status-change bitmap: read wHubStatus / wHubChange.
  if (hub.hub_status_pending) {
    hub.hub_status_pending = false;
    hub.hub_ctrl_op = HubCtrlOp::kGetHubStatus;
    hub.hub_ctrl_port = 0;
    hub.hub_ctrl_feature = 0;
    SubmitControlTransfer(hub, kUsbReqTypeClassDevIn, kUsbReqGetStatus, 0, 0, 4,
                          now);
    return;
  }

  // 1. Clear any pending C_PORT_* bits on any downstream port.
  for (int p = 1; p <= hub.hub_num_ports; ++p) {
    HubPortState& hp = hub.hub_ports[p - 1];
    const StatusOr<uint16_t> feature =
        ctrl_internal::PopHubPortClearFeature(&hp.pending_clear_mask);
    if (feature.ok()) {
      hub.hub_ctrl_op = HubCtrlOp::kClearPortFeature;
      hub.hub_ctrl_port = static_cast<uint8_t>(p);
      hub.hub_ctrl_feature = *feature;
      SubmitControlTransfer(hub, kUsbReqTypeClassPortOut, kUsbReqClearFeature,
                            *feature, static_cast<uint16_t>(p), 0, now);
      return;
    }
  }

  // 1b. Power a port back on once an over-current condition has cleared.
  for (int p = 1; p <= hub.hub_num_ports; ++p) {
    if (hub.hub_ports[p - 1].power_pending) {
      hub.hub_ctrl_op = HubCtrlOp::kPowerPort;
      hub.hub_ctrl_port = static_cast<uint8_t>(p);
      hub.hub_ctrl_feature = kHubPortFeatPower;
      SubmitControlTransfer(hub, kUsbReqTypeClassPortOut, kUsbReqSetFeature,
                            kHubPortFeatPower, static_cast<uint16_t>(p), 0,
                            now);
      return;
    }
  }

  // 2. Poll GET_PORT_STATUS for any flagged port in hub_pending_scan_mask.
  if (hub.hub_pending_scan_mask != 0) {
    for (int p = 1; p <= hub.hub_num_ports; ++p) {
      const uint32_t bit = uint32_t{1} << (p - 1);
      if ((hub.hub_pending_scan_mask & bit) != 0) {
        hub.hub_pending_scan_mask &= ~bit;
        hub.hub_ctrl_op = HubCtrlOp::kGetPortStatus;
        hub.hub_ctrl_port = static_cast<uint8_t>(p);
        hub.hub_ctrl_feature = 0;
        SubmitControlTransfer(hub, kUsbReqTypeClassPortIn, kUsbReqGetStatus, 0,
                              static_cast<uint16_t>(p), 4, now);
        return;
      }
    }
  }

  // 3. Start PORT_RESET on the next kWantReset port if no other port on this
  //    hub is currently resetting or enumerating, and no device of this root
  //    port's tree may answer the default address 0 (a waiting port is
  //    resumed by AdvanceStateMachines()).
  bool busy_reset_or_enum = false;
  for (int p = 1; p <= hub.hub_num_ports; ++p) {
    const HubPortFsm fsm = hub.hub_ports[p - 1].fsm;
    if (fsm == HubPortFsm::kResetting || fsm == HubPortFsm::kResetRecovery ||
        fsm == HubPortFsm::kEnumerating) {
      busy_reset_or_enum = true;
      break;
    }
  }
  if (!busy_reset_or_enum && !HubAddressingBusy(hub.port)) {
    for (int p = 1; p <= hub.hub_num_ports; ++p) {
      HubPortState& hp = hub.hub_ports[p - 1];
      if (hp.fsm == HubPortFsm::kWantReset) {
        hub.hub_ctrl_op = HubCtrlOp::kSetPortReset;
        hub.hub_ctrl_port = static_cast<uint8_t>(p);
        hub.hub_ctrl_feature = kHubPortFeatReset;
        SubmitControlTransfer(hub, kUsbReqTypeClassPortOut, kUsbReqSetFeature,
                              kHubPortFeatReset, static_cast<uint16_t>(p), 0,
                              now);
        return;
      }
    }
  }

  // 4. All pending hub and port status/clear/reset ops are quiescent: ensure
  //    the single Interrupt IN status-change TRB is armed.
  ArmHubInterruptTransfer(hub);
  if (hub.hub_cold_rescans_left > 0 && HubAllPortsDisconnected(hub)) {
    if (hub.hub_rescan_deadline == kNoDeadline) {
      hub.hub_rescan_deadline =
          now + TicksForMs(*platform_, kHubColdRescanIntervalMs);
    }
  } else {
    hub.hub_cold_rescans_left = 0;
    hub.hub_rescan_deadline = kNoDeadline;
  }
}

// True while a device of root port `root_port`'s tree may answer the default
// address 0 (from SET_FEATURE(PORT_RESET) until Address Device), or while an
// abandoned device there is still enabled: on a USB 2.0 segment only one
// device may be addressed at a time (Linux: address0_mutex). Derived from the
// state machines, so detach, retries, timeouts and controller recovery
// release it without bookkeeping.
inline bool XhciController::HubAddressingBusy(const int root_port) const {
  for (int i = 0; i < kMaxDevices; ++i) {
    const DeviceSlot& d = devices_[i];
    if (!d.in_use || d.port != root_port) {
      continue;
    }
    if (d.state == DevFsm::kWantEnableSlot ||
        d.state == DevFsm::kWaitingEnableSlot ||
        d.state == DevFsm::kWantAddressDevice ||
        d.state == DevFsm::kWaitingAddressDevice) {
      return true;  // Reset, not addressed yet
    }
    if (!d.is_hub) {
      continue;
    }
    if (d.ctrl_in_flight && d.hub_ctrl_op == HubCtrlOp::kSetPortReset) {
      return true;
    }
    for (int p = 0; p < d.hub_num_ports; ++p) {
      const HubPortState& hp = d.hub_ports[p];
      if (hp.fsm == HubPortFsm::kResetting ||
          hp.fsm == HubPortFsm::kResetRecovery || hp.disable_pending) {
        return true;
      }
    }
  }
  return false;
}

inline void XhciController::HandleHubPortDisconnect(const int hub_idx,
                                                    const int port,
                                                    const int64_t now) {
  if (hub_idx < 0 || hub_idx >= kMaxDevices) {
    return;
  }
  DeviceSlot& hub = devices_[hub_idx];
  if (!hub.in_use || !hub.is_hub || port < 1 || port > hub.hub_num_ports) {
    return;
  }
  HubPortState& hp = hub.hub_ports[port - 1];
  const uint16_t saved_clear = hp.pending_clear_mask;
  const bool saved_power = hp.power_pending;
  if (hp.dev_idx >= 0 && hp.dev_idx < kMaxDevices) {
    const int child_idx = hp.dev_idx;
    hp.dev_idx = -1;
    DetachDeviceTree(child_idx);
  }
  hp = HubPortState{};
  hp.pending_clear_mask = saved_clear;
  hp.power_pending = saved_power;
  PumpCommandRing(now);
  PumpHubControl(hub_idx, now);
}

inline void XhciController::OnHubPortStatus(const int hub_idx,      //
                                            const int port,         //
                                            const uint16_t status,  //
                                            const uint16_t change,  //
                                            const int64_t now) {
  if (hub_idx < 0 || hub_idx >= kMaxDevices) {
    return;
  }
  DeviceSlot& hub = devices_[hub_idx];
  if (!hub.in_use || !hub.is_hub || port < 1 || port > hub.hub_num_ports) {
    return;
  }
  HubPortState& hp = hub.hub_ports[port - 1];
  const uint16_t valid_change_mask =
      hub.speed >= kPortSpeedSuper ? 0x00F9u : 0x001Fu;
  hp.pending_clear_mask |= static_cast<uint16_t>(change & valid_change_mask);
  const uint16_t power_bit = hub.speed >= kPortSpeedSuper
                                 ? kHubPortStatUsb3Power
                                 : kHubPortStatUsb2Power;
  if ((status & power_bit) == 0 && (status & kHubPortStatOverCurrent) == 0) {
    hp.power_pending = true;
  }

  const bool connected = (status & kHubPortStatConnection) != 0;
  const bool csc = (change & kHubPortChangeConnection) != 0;
  const bool prc =
      (change & (kHubPortChangeReset | kHubPortChangeBhReset)) != 0;

  if (connected) {
    hub.hub_cold_rescans_left = 0;
    hub.hub_rescan_deadline = kNoDeadline;
  }

  if (hp.fsm == HubPortFsm::kResetting) {
    if (prc || ((status & kHubPortStatReset) == 0 &&
                (status & kHubPortStatEnable) != 0)) {
      if (!connected) {
        HandleHubPortDisconnect(hub_idx, port, now);
        return;
      }
      hp.speed = ctrl_internal::DecodeHubPortSpeed(hub.speed, status);
      if ((status & kHubPortStatEnable) == 0 || hp.speed == 0) {
        RetryOrDisableHubPort(hub_idx, port, now,
                              "hub port reset did not enable port");
      } else {
        hp.fsm = HubPortFsm::kResetRecovery;
        hp.deadline = now + TicksForMs(*platform_, kPortResetRecoveryMs);
      }
    } else if (now - hp.debounce_start >=
               TicksForMs(*platform_, kPortResetTimeoutMs)) {
      if (!connected) {
        HandleHubPortDisconnect(hub_idx, port, now);
      } else {
        RetryOrDisableHubPort(hub_idx, port, now, "hub port reset timeout");
      }
    } else if (!connected && (status & kHubPortStatReset) == 0) {
      HandleHubPortDisconnect(hub_idx, port, now);
    } else {
      hp.deadline = now + TicksForMs(*platform_, kHubResetPollMs);
    }
    return;
  }

  if (!connected) {
    if (hp.fsm != HubPortFsm::kDisconnected || hp.dev_idx >= 0) {
      HandleHubPortDisconnect(hub_idx, port, now);
    }
    return;
  }

  if (csc && hp.fsm != HubPortFsm::kDisconnected &&
      hp.fsm != HubPortFsm::kDebouncing) {
    HandleHubPortDisconnect(hub_idx, port, now);
  }

  if (hp.fsm == HubPortFsm::kDisconnected) {
    hp.fsm = HubPortFsm::kDebouncing;
    hp.debounce_start = now;
    hp.deadline = now + TicksForMs(*platform_, kPortDebounceMs);
    hp.attempts = 0;
    return;
  }

  if (hp.fsm == HubPortFsm::kDebouncing && csc) {
    if (now - hp.debounce_start >= TicksForMs(*platform_, kPortDebounceMaxMs)) {
      const uint32_t child_route =
          AppendRouteStringPort(hub.route_string, hub.hub_depth, port);
      LogLine line = StartLogLine();
      line.Str("port ");
      AppendPortPath(line, hub.port, child_route)
          .Str(": debounce timeout")
          .Emit(*platform_);
      hp.fsm = HubPortFsm::kDisabled;
      hp.deadline = kNoDeadline;
      return;
    }
    hp.deadline = now + TicksForMs(*platform_, kPortDebounceMs);
    return;
  }
}

inline void XhciController::OnHubStatus(const int hub_idx,      //
                                        const uint16_t status,  //
                                        const uint16_t change) {
  DeviceSlot& hub = devices_[hub_idx];
  // A hub keeps reporting bit 0 while a wHubChange bit is set (USB 2.0
  // 11.12.4): clear each one before the status-change IN is re-armed.
  hub.hub_pending_hub_clear_mask |= static_cast<uint16_t>(
      change & (kHubChangeLocalPower | kHubChangeOverCurrent));
  if ((change & kHubChangeOverCurrent) != 0) {
    LogLine line = StartLogLine();
    line.Str("port ");
    AppendPortPath(line, hub.port, hub.route_string)
        .Str((status & kHubStatOverCurrent) != 0 ? ": hub over-current"
                                                 : ": hub over-current cleared")
        .Emit(*platform_);
  }
}

// A hub whose hub-level change cannot be acknowledged keeps reporting it.
// Ignore it rather than drop the hub and every keyboard behind it: the
// status-change IN then completes once per polling interval, without EP0
// traffic.
inline void XhciController::IgnoreHubStatusChanges(DeviceSlot& hub) {
  hub.hub_status_broken = true;
  hub.hub_status_pending = false;
  hub.hub_pending_hub_clear_mask = 0;
  LogLine line = StartLogLine();
  line.Str("port ");
  AppendPortPath(line, hub.port, hub.route_string)
      .Str(": hub status change not acknowledged, ignoring it")
      .Emit(*platform_);
}

inline bool XhciController::HubControlBudgetExhausted(DeviceSlot& hub,
                                                      const int64_t now) {
  if (now - hub.hub_fail_window_start >
      TicksForMs(*platform_, kHubControlFailureWindowMs)) {
    hub.hub_fail_window_start = now;
    hub.hub_ctrl_failures = 0;
  }
  return ++hub.hub_ctrl_failures > kMaxHubControlFailures;
}

inline void XhciController::OnHubControlFailure(const int hub_idx,       //
                                                const HubCtrlOp op,      //
                                                const int port,          //
                                                const uint16_t feature,  //
                                                const bool timed_out,    //
                                                const int64_t now) {
  DeviceSlot& hub = devices_[hub_idx];
  if (HubControlBudgetExhausted(hub, now)) {
    // Persistent errors: re-enumerate the hub through its parent port, whose
    // attempts are limited, so that a broken hub ends quiet and disabled.
    RetryOrDisableDevice(hub, now, "hub control error limit");
    PumpCommandRing(now);
    return;
  }
  // Enter EP0 recovery before any bookkeeping: RetryOrDisableHubPort() pumps
  // this hub, and while it was still kConfigured that queued the next request
  // on the halted (or stuck) EP0, where Set TR Dequeue skipped it.
  hub.after_ep0_recovery = Ep0RecoveryNext::kHubControl;
  hub.ep0_stop_retried = false;
  // A STALL, transaction error or babble halts EP0: Reset Endpoint. After a
  // timeout EP0 still runs the stuck TD: Stop Endpoint first (xHCI 4.6.9).
  hub.state = timed_out ? DevFsm::kWantStopEp0 : DevFsm::kWantResetEp0;
  if (op == HubCtrlOp::kGetHubStatus || op == HubCtrlOp::kClearHubFeature) {
    // Hub-level requests are retried a few times; no port is charged.
    if (++hub.hub_status_errors < kMaxEnumerationAttempts) {
      if (op == HubCtrlOp::kGetHubStatus) {
        hub.hub_status_pending = true;
      } else {
        hub.hub_pending_hub_clear_mask |= feature == kHubFeatCHubLocalPower
                                              ? kHubChangeLocalPower
                                              : kHubChangeOverCurrent;
      }
    } else {
      IgnoreHubStatusChanges(hub);
    }
  } else if (op == HubCtrlOp::kClearEndpointHalt) {
    // Retried until the hub's failure budget above re-enumerates it.
    hub.hub_clear_halt_pending = true;
  } else if (port >= 1 && port <= hub.hub_num_ports) {
    // Only the port whose own request failed is charged.
    HubPortState& hp = hub.hub_ports[port - 1];
    ++hp.ctrl_errors;
    const bool retry = hp.ctrl_errors < kMaxEnumerationAttempts;
    if (op == HubCtrlOp::kGetPortStatus && retry) {
      hub.hub_pending_scan_mask |= (uint32_t{1} << (port - 1));
    } else if (op == HubCtrlOp::kClearPortFeature && retry) {
      hp.pending_clear_mask |= ctrl_internal::ChangeBitForHubFeature(feature);
    } else if (op == HubCtrlOp::kDisablePort) {
      if (!retry) {
        hp.disable_pending = false;  // The hub's failure budget bounds it.
      }
    } else if (op == HubCtrlOp::kPowerPort) {
      if (!retry) {
        hp.power_pending = false;
      }
    } else {
      RetryOrDisableHubPort(hub_idx, port, now,
                            timed_out ? "hub control transfer timeout"
                                      : "hub port control transfer error");
    }
  }
  PumpCommandRing(now);
}

inline void XhciController::SubmitCommand(const Trb& trb,                 //
                                          const int dev_idx,              //
                                          const bool is_disable_slot,     //
                                          const uint8_t disable_slot_id,  //
                                          const int64_t now) {
  cmd_state_.busy = true;
  cmd_state_.is_disable_slot = is_disable_slot;
  cmd_state_.abandoned = false;
  cmd_state_.dev_idx = dev_idx;
  cmd_state_.disable_slot_id = disable_slot_id;
  cmd_state_.trb_phys = cmd_ring_.Enqueue(trb);
  cmd_state_.deadline = now + TicksForMs(*platform_, kCommandTimeoutMs);
  mmio_->Write32(caps_.Doorbell(0), 0u);
}

inline void XhciController::PumpCommandRing(const int64_t now) {
  if (cmd_state_.busy || state_ != XhciState::kRunning) {
    return;
  }
  // Leaf first: a hub's slot after the slots behind it (e.g. its TT clients).
  // DCBAA[s] stays valid until the command completes (xHCI 4.6.4).
  if (pending_disable_count_ > 0) {
    const uint8_t s = pending_disable_fifo_[0];
    --pending_disable_count_;
    for (int k = 0; k < pending_disable_count_; ++k) {
      pending_disable_fifo_[k] = pending_disable_fifo_[k + 1];
    }
    SubmitCommand(MakeDisableSlotTrb(s), -1, true, s, now);
    return;
  }

  for (int i = 0; i < kMaxDevices; ++i) {
    DeviceSlot& dev = devices_[i];
    if (!dev.in_use) {
      continue;
    }
    switch (dev.state) {
      case DevFsm::kWantEnableSlot: {
        const uint8_t slot_type =
            static_cast<uint8_t>(caps_.PortSlotType(dev.port));
        dev.state = DevFsm::kWaitingEnableSlot;
        SubmitCommand(MakeEnableSlotTrb(slot_type), i, false, 0, now);
        return;
      }
      case DevFsm::kWantAddressDevice: {
        FormatAddressDeviceInputContext(
            dev.in_ctx_va, caps_.context_size, dev.port, dev.speed,
            dev.ep0_ring.phys(), dev.ep0_ring.cycle(), dev.route_string,
            dev.tt_hub_slot_id, dev.tt_port_number, dev.tt_mtt);
        dcbaa_[dev.slot_id] = dev.out_ctx_phys;
        DmaBarrier();
        dev.state = DevFsm::kWaitingAddressDevice;
        SubmitCommand(MakeAddressDeviceTrb(dev.in_ctx_phys, dev.slot_id, false),
                      i, false, 0, now);
        return;
      }
      case DevFsm::kWantEvaluateEp0: {
        FormatEvaluateEp0MpsInputContext(dev.in_ctx_va, caps_.context_size,
                                         dev.out_ctx_va, dev.ep0_mps);
        DmaBarrier();
        dev.state = DevFsm::kWaitingEvaluateEp0;
        SubmitCommand(MakeEvaluateContextTrb(dev.in_ctx_phys, dev.slot_id), i,
                      false, 0, now);
        return;
      }
      case DevFsm::kWantConfigureEndpoint: {
        FormatConfigureInterruptEndpointInputContext(
            dev.in_ctx_va, caps_.context_size, dev.out_ctx_va, dev.kbd.ep_dci,
            dev.speed, dev.kbd.max_packet_size, dev.kbd.max_burst,
            dev.kbd.b_interval, dev.intr_ring.phys(), dev.intr_ring.cycle(),
            dev.is_hub, dev.hub_num_ports, dev.hub_ttt, dev.hub_mtt);
        DmaBarrier();
        dev.state = DevFsm::kWaitingConfigureEndpoint;
        SubmitCommand(
            MakeConfigureEndpointTrb(dev.in_ctx_phys, dev.slot_id, false), i,
            false, 0, now);
        return;
      }
      case DevFsm::kWantResetEp0: {
        dev.state = DevFsm::kWaitingResetEp0;
        SubmitCommand(MakeResetEndpointTrb(dev.slot_id, 1, false), i, false, 0,
                      now);
        return;
      }
      case DevFsm::kWantSetDeqEp0: {
        const uint64_t deq = dev.ep0_ring.enqueue_phys() | dev.ep0_ring.cycle();
        dev.state = DevFsm::kWaitingSetDeqEp0;
        SubmitCommand(MakeSetTrDequeueTrb(deq, dev.slot_id, 1), i, false, 0,
                      now);
        return;
      }
      case DevFsm::kWantResetIntrEp: {
        dev.state = DevFsm::kWaitingResetIntrEp;
        SubmitCommand(MakeResetEndpointTrb(dev.slot_id, dev.kbd.ep_dci,
                                           dev.intr_soft_retry),
                      i, false, 0, now);
        return;
      }
      case DevFsm::kWantSetDeqIntrEp: {
        dev.intr_ring.Init(dev.intr_ring_va, dev.intr_ring_phys);
        const uint64_t deq =
            dev.intr_ring.enqueue_phys() | dev.intr_ring.cycle();
        dev.state = DevFsm::kWaitingSetDeqIntrEp;
        SubmitCommand(MakeSetTrDequeueTrb(deq, dev.slot_id, dev.kbd.ep_dci), i,
                      false, 0, now);
        return;
      }
      case DevFsm::kWantStopEp0: {
        dev.state = DevFsm::kWaitingStopEp0;
        SubmitCommand(MakeStopEndpointTrb(dev.slot_id, 1), i, false, 0, now);
        return;
      }
      case DevFsm::kWantStopIntrEp: {
        dev.state = DevFsm::kWaitingStopIntrEp;
        SubmitCommand(MakeStopEndpointTrb(dev.slot_id, dev.kbd.ep_dci), i,
                      false, 0, now);
        return;
      }
      default:
        break;
    }
  }
}

inline void XhciController::OnCommandCompletion(const Trb& ev,
                                                const int64_t now) {
  const uint8_t cc = TrbCompletionCode(ev);
  const uint8_t slot_id = TrbSlotId(ev);
  if (cc == kCcCommandRingStopped) {
    // CRCR.CA stopped the ring (its TRB pointer is where it stopped).
    if (cmd_state_.busy && cmd_state_.aborting) {
      FinishCommandAbort(now);
    }
    return;
  }
  if (!cmd_state_.busy || TrbPointer(ev) != cmd_state_.trb_phys) {
    return;
  }
  if (cmd_state_.aborting) {
    // Command Aborted, or a completion that raced the abort: the ring stops
    // next. The command's owner was already failed (AbortCommand()), but a
    // slot that the command enabled must still be disabled.
    if (cc == kCcSuccess && !cmd_state_.is_disable_slot) {
      QueueDisableSlot(slot_id, /*front=*/false);
    }
    return;
  }
  const CommandState completed = cmd_state_;
  cmd_state_ = CommandState{};

  if (completed.is_disable_slot) {
    OnDisableSlotCompletion(completed.disable_slot_id, cc);
    PumpCommandRing(now);
    return;
  }

  if (completed.abandoned || completed.dev_idx < 0 ||
      completed.dev_idx >= kMaxDevices || !devices_[completed.dev_idx].in_use) {
    // The device went away while its command ran. A slot that the command
    // enabled is disabled; one the device had is queued already.
    if (cc == kCcSuccess) {
      QueueDisableSlot(slot_id, /*front=*/false);
    }
    PumpCommandRing(now);
    return;
  }

  DeviceSlot& dev = devices_[completed.dev_idx];
  switch (dev.state) {
    case DevFsm::kWaitingEnableSlot:
      if (cc != kCcSuccess || slot_id < 1 || slot_id > slots_) {
        RetryOrDisableDevice(dev, now, "enable slot command error");
      } else {
        dev.slot_id = slot_id;
        dev.state = DevFsm::kWantAddressDevice;
      }
      break;
    case DevFsm::kWaitingAddressDevice:
      if (cc != kCcSuccess) {
        RetryOrDisableDevice(dev, now, "address device command error");
      } else {
        dev.state = DevFsm::kAddressSettle;
        dev.deadline = now + TicksForMs(*platform_, kAddressSettleMs);
      }
      break;
    case DevFsm::kWaitingEvaluateEp0:
      if (cc != kCcSuccess) {
        RetryOrDisableDevice(dev, now, "evaluate context command error");
      } else {
        dev.state = DevFsm::kWaitingGetDesc18;
        SubmitControlTransfer(dev, kUsbReqTypeStdDevIn, kUsbReqGetDescriptor,
                              static_cast<uint16_t>(kUsbDescDevice << 8), 0, 18,
                              now);
      }
      break;
    case DevFsm::kWaitingConfigureEndpoint:
      if (cc != kCcSuccess) {
        RetryOrDisableDevice(dev, now, "configure endpoint command error");
      } else if (dev.is_hub) {
        // The hub was configured before its descriptor was read
        // (StartHubConfiguration()).
        if (dev.speed >= kPortSpeedSuper) {
          dev.state = DevFsm::kWaitingSetHubDepth;
          SubmitControlTransfer(dev, kUsbReqTypeClassDevOut, kHubReqSetHubDepth,
                                static_cast<uint16_t>(dev.hub_depth), 0, 0,
                                now);
        } else {
          StartHubPortPower(dev, now);
        }
      } else {
        dev.state = DevFsm::kWaitingSetConfig;
        SubmitControlTransfer(dev, kUsbReqTypeStdDevOut,
                              kUsbReqSetConfiguration, dev.kbd.config_value, 0,
                              0, now);
      }
      break;
    case DevFsm::kWaitingResetEp0: {
      if (cc == kCcSuccess) {
        dev.state = DevFsm::kWantSetDeqEp0;
        break;
      }
      // Context State Error: EP0 was not Halted (xHCI 4.6.8).
      DmaBarrier();
      const uint8_t ep_state =
          EpContextState(*OutputEpCtx(dev.out_ctx_va, caps_.context_size, 1));
      if (cc == kCcContextStateError &&
          (ep_state == kEpStateStopped || ep_state == kEpStateError)) {
        dev.state = DevFsm::kWantSetDeqEp0;
      } else if (cc == kCcContextStateError && ep_state == kEpStateRunning) {
        dev.state = DevFsm::kWantStopEp0;
      } else {
        RetryOrDisableDevice(dev, now, "ep0 reset endpoint error");
      }
      break;
    }
    case DevFsm::kWaitingStopEp0: {
      if (cc == kCcSuccess) {
        dev.state = DevFsm::kWantSetDeqEp0;
        break;
      }
      // Context State Error: EP0 halted just before the stop, was stopped
      // already, or (a hardware race) still runs (xHCI 4.6.9).
      DmaBarrier();
      const uint8_t ep_state =
          EpContextState(*OutputEpCtx(dev.out_ctx_va, caps_.context_size, 1));
      if (ep_state == kEpStateHalted) {
        dev.state = DevFsm::kWantResetEp0;
      } else if (ep_state == kEpStateStopped || ep_state == kEpStateError) {
        dev.state = DevFsm::kWantSetDeqEp0;
      } else if (!dev.ep0_stop_retried) {
        dev.ep0_stop_retried = true;
        dev.state = DevFsm::kWantStopEp0;
      } else {
        RetryOrDisableDevice(dev, now, "ep0 stop endpoint error");
      }
      break;
    }
    case DevFsm::kWaitingSetDeqEp0:
      if (cc != kCcSuccess) {
        RetryOrDisableDevice(dev, now, "ep0 set dequeue command error");
      } else if (dev.after_ep0_recovery == Ep0RecoveryNext::kSetIdle) {
        StartSetIdle(dev, now);
      } else if (dev.after_ep0_recovery == Ep0RecoveryNext::kHubPortPower) {
        StartHubPortPower(dev, now);
      } else if (dev.after_ep0_recovery == Ep0RecoveryNext::kHubControl) {
        dev.state = DevFsm::kConfigured;
        PumpHubControl(completed.dev_idx, now);
      } else {
        FinishKeyboardAttach(dev, now);
      }
      break;
    case DevFsm::kWaitingResetIntrEp: {
      if (cc == kCcSuccess && dev.intr_soft_retry) {
        // Soft Retry: the endpoint kept its transfer state (data toggle /
        // sequence number) and retries the TD that failed (xHCI 4.6.8.1).
        ResumeInterruptEndpoint(dev, now);
        break;
      }
      if (cc == kCcSuccess) {
        // TSP = 0 reset the host's data toggle (sequence number): the
        // device's is reset by CLEAR_FEATURE(ENDPOINT_HALT) once the ring is
        // moved past the halted TD, before the endpoint is re-armed.
        dev.intr_clear_halt_due = true;
        dev.state = DevFsm::kWantSetDeqIntrEp;
        break;
      }
      // Context State Error: the endpoint was not Halted (xHCI 4.6.8).
      DmaBarrier();
      const uint8_t ep_state = EpContextState(
          *OutputEpCtx(dev.out_ctx_va, caps_.context_size, dev.kbd.ep_dci));
      if (cc == kCcContextStateError &&
          (ep_state == kEpStateStopped || ep_state == kEpStateError)) {
        dev.state = DevFsm::kWantSetDeqIntrEp;
      } else if (cc == kCcContextStateError && ep_state == kEpStateRunning) {
        dev.state = DevFsm::kWantStopIntrEp;
      } else {
        RetryOrDisableDevice(dev, now, "interrupt ep recovery error");
      }
      break;
    }
    case DevFsm::kWaitingStopIntrEp: {
      DmaBarrier();
      const uint8_t ep_state = EpContextState(
          *OutputEpCtx(dev.out_ctx_va, caps_.context_size, dev.kbd.ep_dci));
      if (cc == kCcSuccess || ep_state == kEpStateStopped ||
          ep_state == kEpStateError) {
        dev.state = DevFsm::kWantSetDeqIntrEp;
      } else if (ep_state == kEpStateHalted) {
        dev.state = DevFsm::kWantResetIntrEp;
      } else {
        RetryOrDisableDevice(dev, now, "interrupt ep recovery error");
      }
      break;
    }
    case DevFsm::kWaitingSetDeqIntrEp:
      if (cc != kCcSuccess) {
        RetryOrDisableDevice(dev, now, "interrupt ep recovery error");
      } else if (dev.is_hub) {
        dev.state = DevFsm::kConfigured;
        dev.hub_intr_armed = false;
        dev.hub_clear_halt_pending = dev.intr_clear_halt_due;
        dev.intr_clear_halt_due = false;
        PumpHubControl(completed.dev_idx, now);
      } else if (dev.intr_clear_halt_due) {
        dev.intr_clear_halt_due = false;
        dev.state = DevFsm::kWaitingClearIntrHalt;
        SubmitControlTransfer(dev, kUsbReqTypeStdEndpointOut,
                              kUsbReqClearFeature, kUsbFeatEndpointHalt,
                              dev.kbd.ep_address, 0, now);
      } else {
        dev.state = DevFsm::kConfigured;
        ArmInterruptTransfers(dev);
      }
      break;
    default:
      break;
  }

  PumpCommandRing(now);
}

inline void XhciController::SubmitControlTransfer(
    DeviceSlot& dev,                //
    const uint8_t bm_request_type,  //
    const uint8_t b_request,        //
    const uint16_t w_value,         //
    const uint16_t w_index,         //
    const uint16_t w_length,        //
    const int64_t now) {
  ctrl_internal::ZeroBytesVolatile(dev.ctrl_buf_va, kControlBufferSize);
  if (w_length > 0) {
    const bool dir_in = (bm_request_type & 0x80u) != 0;
    // The Transfer Type field exists since xHCI 1.0; on 0.96 controllers
    // those bits are RsvdZ (Linux xhci_queue_ctrl_tx()).
    const uint32_t trt = caps_.hci_version < 0x0100
                             ? kTrtNoData
                             : (dir_in ? kTrtInData : kTrtOutData);
    const Trb td[3] = {
        MakeSetupStageTrb(bm_request_type, b_request, w_value, w_index,
                          w_length, trt),
        MakeDataStageTrb(dev.ctrl_buf_phys, w_length, dir_in),
        MakeStatusStageTrb(!dir_in),
    };
    uint64_t phys[3] = {};
    dev.ep0_ring.EnqueueTd(td, 3, phys);
    dev.ctrl_setup_phys = phys[0];
    dev.ctrl_data_phys = phys[1];
    dev.ctrl_status_phys = phys[2];
  } else {
    const Trb td[2] = {
        MakeSetupStageTrb(bm_request_type, b_request, w_value, w_index, 0,
                          kTrtNoData),
        MakeStatusStageTrb(true),
    };
    uint64_t phys[2] = {};
    dev.ep0_ring.EnqueueTd(td, 2, phys);
    dev.ctrl_setup_phys = phys[0];
    dev.ctrl_data_phys = 0;
    dev.ctrl_status_phys = phys[1];
  }
  dev.ctrl_req_len = w_length;
  dev.ctrl_actual_len = w_length;
  dev.ctrl_in_flight = true;
  dev.deadline = now + TicksForMs(*platform_, kControlTimeoutMs);
  mmio_->Write32(caps_.Doorbell(dev.slot_id), 1u);
}

inline void XhciController::OnTransferEvent(const Trb& ev, const int64_t now) {
  const uint8_t slot_id = TrbSlotId(ev);
  const uint8_t dci = TrbEndpointId(ev);
  if (slot_id == 0) {
    return;
  }
  for (int i = 0; i < kMaxDevices; ++i) {
    DeviceSlot& dev = devices_[i];
    if (!dev.in_use || dev.slot_id != slot_id) {
      continue;
    }
    if (dci == 1) {
      OnEp0TransferEvent(dev, ev, now);
    } else if (dci == dev.kbd.ep_dci) {
      OnInterruptTransferEvent(dev, ev, now);
    }
    return;
  }
}

inline void XhciController::OnEp0TransferEvent(DeviceSlot& dev,  //
                                               const Trb& ev,    //
                                               const int64_t now) {
  if (!dev.ctrl_in_flight) {
    return;
  }
  const uint64_t ptr = TrbPointer(ev);
  const uint8_t cc = TrbCompletionCode(ev);
  if (cc == kCcStopped || cc == kCcStoppedLengthInvalid) {
    return;  // From Stop Endpoint, for the TD that already timed out.
  }

  if (dev.ctrl_data_phys != 0 && ptr == dev.ctrl_data_phys &&
      (cc == kCcShortPacket || cc == kCcSuccess)) {
    const uint32_t res = TrbTransferResidual(ev);
    dev.ctrl_actual_len = res < dev.ctrl_req_len
                              ? static_cast<uint16_t>(dev.ctrl_req_len - res)
                              : 0u;
    return;
  }

  if (ptr == dev.ctrl_status_phys &&
      (cc == kCcSuccess || cc == kCcShortPacket)) {
    dev.ctrl_in_flight = false;
    dev.deadline = kNoDeadline;
    OnControlTransferComplete(dev, true, cc, now);
    return;
  }

  if (cc != kCcSuccess) {
    dev.ctrl_in_flight = false;
    dev.deadline = kNoDeadline;
    OnControlTransferComplete(dev, false, cc, now);
  }
}

inline void XhciController::StartSetIdle(DeviceSlot& dev, const int64_t now) {
  dev.state = DevFsm::kWaitingSetIdle;
  SubmitControlTransfer(dev, kUsbReqTypeClassIfaceOut, kHidReqSetIdle, 0,
                        dev.kbd.interface_number, 0, now);
}

inline void XhciController::OnControlTransferComplete(DeviceSlot& dev,   //
                                                      const bool ok,     //
                                                      const uint8_t cc,  //
                                                      const int64_t now) {
  switch (dev.state) {
    case DevFsm::kWaitingGetDesc8: {
      if (!ok) {
        RetryOrDisableDevice(dev, now, "get device descriptor prefix error");
        return;
      }
      const StatusOr<uint8_t> b_mps0 =
          ParseDeviceDescriptorPrefix(dev.ctrl_buf_va, dev.ctrl_actual_len);
      if (!b_mps0.ok()) {
        RetryOrDisableDevice(dev, now, "invalid device descriptor prefix");
        return;
      }
      const uint16_t actual_mps = DecodeEp0MaxPacketSize(dev.speed, *b_mps0);
      if (actual_mps != dev.ep0_mps) {
        dev.ep0_mps = actual_mps;
        dev.state = DevFsm::kWantEvaluateEp0;
        PumpCommandRing(now);
      } else {
        dev.state = DevFsm::kWaitingGetDesc18;
        SubmitControlTransfer(dev, kUsbReqTypeStdDevIn, kUsbReqGetDescriptor,
                              static_cast<uint16_t>(kUsbDescDevice << 8), 0, 18,
                              now);
      }
      return;
    }
    case DevFsm::kWaitingGetDesc18: {
      if (!ok) {
        RetryOrDisableDevice(dev, now, "get device descriptor error");
        return;
      }
      const StatusOr<DeviceDescriptorInfo> ddesc =
          ParseDeviceDescriptorFull(dev.ctrl_buf_va, dev.ctrl_actual_len);
      if (!ddesc.ok() || ddesc->num_configs == 0) {
        RetryOrDisableDevice(dev, now, "invalid full device descriptor");
        return;
      }
      dev.device_class = ddesc->device_class;
      dev.vendor_id = ddesc->vendor_id;
      dev.product_id = ddesc->product_id;
      dev.device_protocol = dev.ctrl_buf_va[6];
      dev.state = DevFsm::kWaitingGetConfig9;
      SubmitControlTransfer(dev, kUsbReqTypeStdDevIn, kUsbReqGetDescriptor,
                            static_cast<uint16_t>(kUsbDescConfiguration << 8),
                            0, 9, now);
      return;
    }
    case DevFsm::kWaitingGetConfig9: {
      if (!ok) {
        RetryOrDisableDevice(dev, now, "get config header error");
        return;
      }
      const StatusOr<ConfigDescriptorHeader> chdr =
          ParseConfigDescriptorHeader(dev.ctrl_buf_va, dev.ctrl_actual_len);
      if (!chdr.ok()) {
        const int parent_idx = dev.parent_dev_idx;
        DisableUnsupportedDevice(dev, "invalid config descriptor header");
        PumpCommandRing(now);
        if (parent_idx >= 0) {
          PumpHubControl(parent_idx, now);
        }
        return;
      }
      const uint16_t req_len = chdr->total_length > kControlBufferSize
                                   ? kControlBufferSize
                                   : chdr->total_length;
      dev.state = DevFsm::kWaitingGetConfigFull;
      SubmitControlTransfer(dev, kUsbReqTypeStdDevIn, kUsbReqGetDescriptor,
                            static_cast<uint16_t>(kUsbDescConfiguration << 8),
                            0, req_len, now);
      return;
    }
    case DevFsm::kWaitingGetConfigFull: {
      if (!ok) {
        RetryOrDisableDevice(dev, now, "get full config descriptor error");
        return;
      }
      if (dev.device_class == kUsbClassHub) {
        const StatusOr<HubMatch> hmatch =
            FindUsbHub(dev.ctrl_buf_va, dev.ctrl_actual_len);
        if (!hmatch.ok()) {
          const int parent_idx = dev.parent_dev_idx;
          DisableUnsupportedDevice(dev, "no usb hub interface");
          PumpCommandRing(now);
          if (parent_idx >= 0) {
            PumpHubControl(parent_idx, now);
          }
          return;
        }
        if (dev.hub_depth >= kMaxHubDepth) {
          const int parent_idx = dev.parent_dev_idx;
          DisableUnsupportedDevice(dev, "max hub depth exceeded");
          PumpCommandRing(now);
          if (parent_idx >= 0) {
            PumpHubControl(parent_idx, now);
          }
          return;
        }
        StartHubConfiguration(dev, *hmatch, now);
        return;
      }
      const StatusOr<BootKeyboardMatch> kmatch =
          FindBootKeyboard(dev.ctrl_buf_va, dev.ctrl_actual_len);
      if (kmatch.ok()) {
        dev.is_hub = false;
        dev.kbd = *kmatch;
        dev.state = DevFsm::kWantConfigureEndpoint;
        PumpCommandRing(now);
        return;
      }
      const StatusOr<HubMatch> hmatch =
          FindUsbHub(dev.ctrl_buf_va, dev.ctrl_actual_len);
      if (hmatch.ok() && dev.hub_depth < kMaxHubDepth) {
        StartHubConfiguration(dev, *hmatch, now);
        return;
      }
      const int parent_idx = dev.parent_dev_idx;
      DisableUnsupportedDevice(dev, "no boot keyboard interface");
      PumpCommandRing(now);
      if (parent_idx >= 0) {
        PumpHubControl(parent_idx, now);
      }
      return;
    }
    case DevFsm::kWaitingHubSetConfig: {
      if (!ok) {
        RetryOrDisableDevice(dev, now, "set configuration error");
        return;
      }
      StartHubDescriptorFetch(dev, now);
      return;
    }
    case DevFsm::kWaitingGetHubDesc: {
      if (!ok) {
        RetryOrDisableDevice(dev, now, "get hub descriptor error");
        return;
      }
      const uint8_t expected_type =
          dev.speed >= kPortSpeedSuper ? kUsbDescSsHub : kUsbDescHub;
      const StatusOr<HubDescriptorInfo> hdesc = ParseHubDescriptor(
          dev.ctrl_buf_va, dev.ctrl_actual_len, expected_type);
      if (!hdesc.ok()) {
        const int parent_idx = dev.parent_dev_idx;
        DisableUnsupportedDevice(dev, "invalid hub descriptor");
        PumpCommandRing(now);
        if (parent_idx >= 0) {
          PumpHubControl(parent_idx, now);
        }
        return;
      }
      dev.hub_desc = *hdesc;
      dev.hub_num_ports =
          hdesc->num_ports > kMaxHubPorts ? kMaxHubPorts : hdesc->num_ports;
      dev.hub_ttt = (dev.speed == kPortSpeedHigh) ? hdesc->ttt : 0u;
      dev.hub_mtt = (dev.speed == kPortSpeedHigh && dev.device_protocol == 2 &&
                     dev.hub_match.interface_protocol == 2);
      int64_t settle_ms = static_cast<int64_t>(hdesc->pwr_on_to_pwr_good) * 2;
      if (settle_ms < kMinHubPowerSettleMs) {
        settle_ms = kMinHubPowerSettleMs;
      } else if (settle_ms > kMaxHubPowerSettleMs) {
        settle_ms = kMaxHubPowerSettleMs;
      }
      dev.hub_power_settle_ms = settle_ms;
      dev.state = DevFsm::kWantConfigureEndpoint;
      PumpCommandRing(now);
      return;
    }
    case DevFsm::kWaitingSetConfig: {
      if (!ok) {
        RetryOrDisableDevice(dev, now, "set configuration error");
        return;
      }
      dev.state = DevFsm::kWaitingSetProtocol;
      SubmitControlTransfer(dev, kUsbReqTypeClassIfaceOut, kHidReqSetProtocol,
                            0, dev.kbd.interface_number, 0, now);
      return;
    }
    case DevFsm::kWaitingSetHubDepth: {
      if (ok) {
        StartHubPortPower(dev, now);
      } else if (cc == kCcStallError) {
        LogLine line = StartLogLine();
        line.Str("port ");
        AppendPortPath(line, dev.port, dev.route_string)
            .Str(": SET_HUB_DEPTH stalled, recovering ep0")
            .Emit(*platform_);
        dev.after_ep0_recovery = Ep0RecoveryNext::kHubPortPower;
        dev.ep0_stop_retried = false;
        dev.state = DevFsm::kWantResetEp0;
        PumpCommandRing(now);
      } else {
        RetryOrDisableDevice(dev, now, "set hub depth error");
      }
      return;
    }
    case DevFsm::kWaitingHubPortPower: {
      if (!ok) {
        RetryOrDisableDevice(dev, now, "hub port power error");
        return;
      }
      if (dev.hub_init_port < dev.hub_num_ports) {
        ++dev.hub_init_port;
        SubmitControlTransfer(dev, kUsbReqTypeClassPortOut, kUsbReqSetFeature,
                              kHubPortFeatPower, dev.hub_init_port, 0, now);
      } else {
        dev.state = DevFsm::kHubPowerSettle;
        dev.deadline = now + TicksForMs(*platform_, dev.hub_power_settle_ms);
      }
      return;
    }
    case DevFsm::kWaitingSetProtocol: {
      if (ok) {
        StartSetIdle(dev, now);
      } else if (cc == kCcStallError) {
        LogLine line = StartLogLine();
        line.Str("port ");
        AppendPortPath(line, dev.port, dev.route_string)
            .Str(": SET_PROTOCOL stalled, recovering ep0")
            .Emit(*platform_);
        dev.after_ep0_recovery = Ep0RecoveryNext::kSetIdle;
        dev.ep0_stop_retried = false;
        dev.state = DevFsm::kWantResetEp0;
        PumpCommandRing(now);
      } else {
        RetryOrDisableDevice(dev, now, "set protocol error");
      }
      return;
    }
    case DevFsm::kWaitingSetIdle: {
      if (ok) {
        FinishKeyboardAttach(dev, now);
      } else if (cc == kCcStallError) {
        LogLine line = StartLogLine();
        line.Str("port ");
        AppendPortPath(line, dev.port, dev.route_string)
            .Str(": SET_IDLE stalled, recovering ep0")
            .Emit(*platform_);
        dev.after_ep0_recovery = Ep0RecoveryNext::kFinishAttach;
        dev.ep0_stop_retried = false;
        dev.state = DevFsm::kWantResetEp0;
        PumpCommandRing(now);
      } else {
        RetryOrDisableDevice(dev, now, "set idle error");
      }
      return;
    }
    case DevFsm::kWaitingClearIntrHalt: {
      if (!ok) {
        RetryOrDisableDevice(dev, now, "clear endpoint halt error");
        return;
      }
      dev.state = DevFsm::kConfigured;
      ArmInterruptTransfers(dev);
      return;
    }
    case DevFsm::kConfigured: {
      if (!dev.is_hub) {
        return;
      }
      const int hub_idx = static_cast<int>(&dev - devices_);
      const HubCtrlOp op = dev.hub_ctrl_op;
      const int port = dev.hub_ctrl_port;
      const uint16_t feat = dev.hub_ctrl_feature;
      dev.hub_ctrl_op = HubCtrlOp::kNone;
      dev.hub_ctrl_port = 0;
      dev.hub_ctrl_feature = 0;

      if (!ok) {
        OnHubControlFailure(hub_idx, op, port, feat, /*timed_out=*/false, now);
        return;
      }

      if (port >= 1 && port <= dev.hub_num_ports) {
        dev.hub_ports[port - 1].ctrl_errors = 0;
      }
      if (op == HubCtrlOp::kGetHubStatus) {
        if (dev.ctrl_actual_len >= 4) {
          dev.hub_status_errors = 0;
          OnHubStatus(hub_idx, ReadLe16(dev.ctrl_buf_va),
                      ReadLe16(dev.ctrl_buf_va + 2));
        } else if (++dev.hub_status_errors < kMaxEnumerationAttempts) {
          dev.hub_status_pending = true;  // Short reply: read it again.
        } else {
          IgnoreHubStatusChanges(dev);
        }
      } else if (op == HubCtrlOp::kClearHubFeature) {
        dev.hub_status_errors = 0;
      } else if (op == HubCtrlOp::kDisablePort && port >= 1 &&
                 port <= dev.hub_num_ports) {
        dev.hub_ports[port - 1].disable_pending = false;
      } else if (op == HubCtrlOp::kPowerPort && port >= 1 &&
                 port <= dev.hub_num_ports) {
        dev.hub_ports[port - 1].power_pending = false;
        dev.hub_pending_scan_mask |= (uint32_t{1} << (port - 1));
      } else if (op == HubCtrlOp::kGetPortStatus && port >= 1 &&
                 port <= dev.hub_num_ports && dev.ctrl_actual_len >= 4) {
        const uint16_t status = ReadLe16(dev.ctrl_buf_va);
        const uint16_t change = ReadLe16(dev.ctrl_buf_va + 2);
        OnHubPortStatus(hub_idx, port, status, change, now);
      } else if (op == HubCtrlOp::kSetPortReset && port >= 1 &&
                 port <= dev.hub_num_ports) {
        HubPortState& hp = dev.hub_ports[port - 1];
        hp.fsm = HubPortFsm::kResetting;
        hp.debounce_start = now;
        hp.deadline = now + TicksForMs(*platform_, kHubResetPollMs);
        dev.hub_pending_scan_mask |= (uint32_t{1} << (port - 1));
      }
      PumpHubControl(hub_idx, now);
      return;
    }
    default:
      return;
  }
}

inline void XhciController::ArmInterruptTransfers(DeviceSlot& dev) {
  const uint32_t xfer_len = dev.kbd.max_packet_size < kReportBufferSize
                                ? dev.kbd.max_packet_size
                                : kReportBufferSize;
  dev.next_rearm_idx = 0;
  for (int i = 0; i < kReportTrbs; ++i) {
    const uint64_t buf_phys =
        dev.report_buf_phys + static_cast<uint64_t>(i * kReportBufferSize);
    dev.report_trb_phys[i] =
        dev.intr_ring.Enqueue(MakeNormalTrb(buf_phys, xfer_len, true, true));
  }
  mmio_->Write32(caps_.Doorbell(dev.slot_id), dev.kbd.ep_dci);
}

inline void XhciController::FinishKeyboardAttach(DeviceSlot& dev,
                                                 const int64_t now) {
  dev.hid_state = HidKeyboardState{};
  dev.intr_errors = 0;
  dev.deadline = kNoDeadline;
  dev.state = DevFsm::kConfigured;
  MarkParentPortActive(dev, now);
  ArmInterruptTransfers(dev);
  AnnounceKeyboardAttach(dev);
  if (dev.parent_dev_idx >= 0) {
    PumpHubControl(dev.parent_dev_idx, now);
  }
}

inline void XhciController::OnInterruptTransferEvent(DeviceSlot& dev,  //
                                                     const Trb& ev,    //
                                                     const int64_t now) {
  const uint64_t ptr = TrbPointer(ev);
  const uint8_t cc = TrbCompletionCode(ev);

  // A hub's status-change TRB may complete while the hub recovers EP0 or
  // waits for a hub request; it is handled in any state, or the hub would
  // never re-arm and stay deaf.
  if (dev.is_hub) {
    if (cc == kCcRingUnderrun || cc == kCcRingOverrun || cc == kCcStopped ||
        cc == kCcStoppedLengthInvalid) {
      return;  // The TRB is not retired: it stays armed.
    }
    dev.hub_intr_armed = false;
    const int hub_idx = static_cast<int>(&dev - devices_);
    if (cc == kCcSuccess || cc == kCcShortPacket) {
      dev.intr_errors = 0;
      dev.intr_soft_retries = 0;
      const uint32_t req_len = dev.kbd.max_packet_size < kReportBufferSize
                                   ? dev.kbd.max_packet_size
                                   : kReportBufferSize;
      const uint32_t res = TrbTransferResidual(ev);
      const uint32_t actual = res < req_len ? req_len - res : 0u;
      // Bit 0: a hub-level change (C_HUB_LOCAL_POWER, C_HUB_OVER_CURRENT).
      const bool hub_change = actual > 0 && (dev.report_buf_va[0] & 0x01u) != 0;
      if (hub_change && !dev.hub_status_broken) {
        dev.hub_status_pending = true;
      }
      bool any_port = false;
      for (int p = 1; p <= dev.hub_num_ports; ++p) {
        const uint32_t byte_idx = static_cast<uint32_t>(p / 8);
        const uint8_t bit_mask = static_cast<uint8_t>(1u << (p % 8));
        if (byte_idx < actual &&
            (dev.report_buf_va[byte_idx] & bit_mask) != 0) {
          dev.hub_pending_scan_mask |= (uint32_t{1} << (p - 1));
          any_port = true;
        }
      }
      if (!any_port && !hub_change && actual > 0) {
        dev.hub_pending_scan_mask |= (uint32_t{1} << dev.hub_num_ports) - 1u;
      }
      PumpHubControl(hub_idx, now);
      return;
    }
    if (dev.intr_recovery_pending || dev.state == DevFsm::kWantStopIntrEp ||
        dev.state == DevFsm::kWaitingStopIntrEp ||
        dev.state == DevFsm::kWantResetIntrEp ||
        dev.state == DevFsm::kWaitingResetIntrEp ||
        dev.state == DevFsm::kWantSetDeqIntrEp ||
        dev.state == DevFsm::kWaitingSetDeqIntrEp) {
      return;  // This endpoint's recovery is pending or running already.
    }
    const uint32_t ps = ReadPortSc(dev.port);
    if (ps == kMmioAllOnes || (ps & kPortScCcs) == 0) {
      HandlePortDisconnect(dev.port, now);
      PumpCommandRing(now);
      return;
    }
    if (!CountInterruptError(dev, cc, now)) {
      RetryOrDisableDevice(dev, now, "interrupt endpoint error limit");
      PumpCommandRing(now);
      return;
    }
    LogLine line = StartLogLine();
    line.Str("port ");
    AppendPortPath(line, dev.port, dev.route_string)
        .Str(": interrupt endpoint error, recovering")
        .Emit(*platform_);
    // Recovered from PumpHubControl() once EP0 is idle: never overwrite the
    // state of a hub request in flight.
    dev.intr_recovery_pending = true;
    PumpHubControl(hub_idx, now);
    return;
  }

  if (dev.state != DevFsm::kConfigured) {
    return;
  }

  int buf_idx = -1;
  for (int i = 0; i < kReportTrbs; ++i) {
    if (dev.report_trb_phys[i] == ptr) {
      buf_idx = i;
      break;
    }
  }

  if (cc == kCcSuccess || cc == kCcShortPacket) {
    dev.intr_errors = 0;
    dev.intr_soft_retries = 0;
    const uint32_t req_len = dev.kbd.max_packet_size < kReportBufferSize
                                 ? dev.kbd.max_packet_size
                                 : kReportBufferSize;
    const uint32_t res = TrbTransferResidual(ev);
    const uint32_t actual = res < req_len ? req_len - res : 0u;
    const int idx = buf_idx >= 0 ? buf_idx : dev.next_rearm_idx;
    dev.next_rearm_idx = (idx + 1) % kReportTrbs;
    if (actual >= static_cast<uint32_t>(kBootReportMinSize)) {
      char out[kBootReportKeys] = {};
      const int n = TranslateBootReport(
          &dev.hid_state, locks_, dev.report_buf_va + idx * kReportBufferSize,
          static_cast<int>(actual), out, static_cast<int>(sizeof(out)));
      if (n > 0) {
        platform_->PushInput(out, n);
      }
    }
    const uint64_t buf_phys =
        dev.report_buf_phys + static_cast<uint64_t>(idx * kReportBufferSize);
    dev.report_trb_phys[idx] =
        dev.intr_ring.Enqueue(MakeNormalTrb(buf_phys, req_len, true, true));
    mmio_->Write32(caps_.Doorbell(dev.slot_id), dev.kbd.ep_dci);
    return;
  }

  if (cc == kCcRingUnderrun || cc == kCcRingOverrun || cc == kCcStopped ||
      cc == kCcStoppedLengthInvalid) {
    return;
  }

  // Check if port disconnected before attempting endpoint recovery.
  const uint32_t ps = ReadPortSc(dev.port);
  if (ps == kMmioAllOnes || (ps & kPortScCcs) == 0) {
    HandlePortDisconnect(dev.port, now);
    PumpCommandRing(now);
    return;
  }

  if (!CountInterruptError(dev, cc, now)) {
    RetryOrDisableDevice(dev, now, "interrupt endpoint error limit");
    PumpCommandRing(now);
    return;
  }
  LogLine line = StartLogLine();
  line.Str("port ");
  AppendPortPath(line, dev.port, dev.route_string)
      .Str(": interrupt endpoint error, recovering")
      .Emit(*platform_);
  StartInterruptEndpointRecovery(dev);
  PumpCommandRing(now);
}

// Counts an interrupt endpoint error toward kMaxInterruptErrors (anew after
// kInterruptErrorWindowMs without one) and picks the recovery: a Soft Retry
// for a USB Transaction Error, a few times in a row and not for LS/FS
// devices behind a TT (as Linux), else Reset Endpoint with TSP = 0 followed
// by CLEAR_FEATURE(ENDPOINT_HALT) (xHCI 4.6.8, USB 2.0 9.4.5). Returns false
// once the limit is exceeded.
inline bool XhciController::CountInterruptError(DeviceSlot& dev,
                                                const uint8_t cc,
                                                const int64_t now) {
  if (dev.intr_errors > 0 &&
      now - dev.last_intr_error >
          TicksForMs(*platform_, kInterruptErrorWindowMs)) {
    dev.intr_errors = 0;
    dev.intr_soft_retries = 0;
  }
  dev.last_intr_error = now;
  if (++dev.intr_errors > kMaxInterruptErrors) {
    return false;
  }
  dev.intr_soft_retry = cc == kCcUsbTransactionError &&
                        dev.tt_hub_slot_id == 0 &&
                        dev.intr_soft_retries < kMaxSoftRetries;
  dev.intr_soft_retries = dev.intr_soft_retry ? dev.intr_soft_retries + 1 : 0;
  return true;
}

// Starts the recovery of `dev`'s interrupt IN endpoint by its state: Reset
// Endpoint is valid only on a Halted endpoint, Set TR Dequeue only on a
// Stopped (or Error) one, and a Running one is stopped first (xHCI 4.6.8 -
// 4.6.10). Not every error halts it: a spurious or late event may arrive for
// an endpoint that still runs.
inline void XhciController::StartInterruptEndpointRecovery(DeviceSlot& dev) {
  dev.intr_clear_halt_due = false;
  DmaBarrier();
  const uint8_t ep_state = EpContextState(
      *OutputEpCtx(dev.out_ctx_va, caps_.context_size, dev.kbd.ep_dci));
  if (ep_state == kEpStateRunning) {
    dev.state = DevFsm::kWantStopIntrEp;
  } else if (ep_state == kEpStateStopped || ep_state == kEpStateError) {
    dev.state = DevFsm::kWantSetDeqIntrEp;
  } else {
    dev.state = DevFsm::kWantResetIntrEp;
  }
}

// After a Soft Retry: the TRBs stay armed, and the doorbell makes the xHC
// retry the TD at the endpoint's dequeue pointer.
inline void XhciController::ResumeInterruptEndpoint(DeviceSlot& dev,
                                                    const int64_t now) {
  dev.state = DevFsm::kConfigured;
  if (dev.is_hub) {
    dev.hub_intr_armed = true;
  }
  mmio_->Write32(caps_.Doorbell(dev.slot_id), dev.kbd.ep_dci);
  if (dev.is_hub) {
    PumpHubControl(static_cast<int>(&dev - devices_), now);
  }
}

inline void XhciController::AdvanceStateMachines(const int64_t now) {
  if (state_ != XhciState::kRunning) {
    return;
  }
  if (pending_recovery_ != nullptr) {
    const char* const reason = pending_recovery_;
    pending_recovery_ = nullptr;
    Recover(reason);
    return;
  }

  if (event_ring_full_rescan_) {
    event_ring_full_rescan_ = false;
    for (int port = 1; port <= ports_; ++port) {
      ScanPort(port, now);
    }
    // A dropped status-change or report completion would leave a hub or
    // keyboard deaf: re-sync each configured hub's status-change endpoint
    // (one TRB armed again) and re-read its hub status and ports, and re-sync
    // each configured keyboard's interrupt endpoint. Hubs and keyboards still
    // being set up skip it (FinishHubAttach() / FinishKeyboardAttach()).
    for (int i = 0; i < kMaxDevices; ++i) {
      DeviceSlot& dev = devices_[i];
      if (!dev.in_use) {
        continue;
      }
      if (dev.is_hub) {
        const bool in_intr_recovery =
            dev.state == DevFsm::kWantStopIntrEp ||
            dev.state == DevFsm::kWaitingStopIntrEp ||
            dev.state == DevFsm::kWantResetIntrEp ||
            dev.state == DevFsm::kWaitingResetIntrEp ||
            dev.state == DevFsm::kWantSetDeqIntrEp ||
            dev.state == DevFsm::kWaitingSetDeqIntrEp;
        if (dev.state == DevFsm::kConfigured ||
            dev.after_ep0_recovery == Ep0RecoveryNext::kHubControl ||
            in_intr_recovery) {
          if (!in_intr_recovery) {
            dev.intr_recovery_pending = true;
            dev.intr_soft_retry = false;
          }
          dev.hub_status_pending = !dev.hub_status_broken;
          dev.hub_pending_scan_mask |= (uint32_t{1} << dev.hub_num_ports) - 1u;
          PumpHubControl(i, now);
        }
      } else if (dev.state == DevFsm::kConfigured) {
        dev.intr_soft_retry = false;
        StartInterruptEndpointRecovery(dev);
        PumpCommandRing(now);
      }
    }
  }

  if (cmd_state_.busy && now >= cmd_state_.deadline) {
    if (!cmd_state_.aborting) {
      AbortCommand(now);
    } else {
      const uint32_t crcr = ReadOp(kOpCrcr);
      if (crcr == kMmioAllOnes || (crcr & kCrcrCrr) != 0) {
        Recover("command abort timeout");
        return;
      }
      // Stopped, but without a Command Ring Stopped event (Linux
      // xhci_abort_cmd_ring() copes with such controllers too).
      FinishCommandAbort(now);
    }
    if (pending_recovery_ != nullptr) {
      const char* const reason = pending_recovery_;
      pending_recovery_ = nullptr;
      Recover(reason);
      return;
    }
  }

  for (int port = 1; port <= ports_; ++port) {
    PortState& p = port_states_[port - 1];
    if (p.deadline == kNoDeadline || now < p.deadline) {
      continue;
    }
    if (p.fsm == PortFsm::kDebouncing) {
      StartPortResetOrUsb3(port, now);
    } else if (p.fsm == PortFsm::kResetting) {
      const uint32_t ps = ReadPortSc(port);
      if (ps != kMmioAllOnes && (ps & kPortScCcs) != 0 &&
          (ps & kPortScPr) == 0 && (ps & kPortScPed) != 0) {
        if ((ps & kPortScChangeMask) != 0) {
          WritePortSc(port,
                      (ps & kPortScNeutralMask) | (ps & kPortScChangeMask));
        }
        p.speed = PortSpeedOf(ps);
        p.fsm = PortFsm::kResetRecovery;
        p.deadline = now + TicksForMs(*platform_, kPortResetRecoveryMs);
      } else {
        RetryOrDisablePort(port, now, "port reset timeout");
      }
    } else if (p.fsm == PortFsm::kResetRecovery) {
      const uint32_t ps = ReadPortSc(port);
      if (ps == kMmioAllOnes || (ps & kPortScCcs) == 0) {
        HandlePortDisconnect(port, now);
      } else if ((ps & kPortScPed) == 0) {
        RetryOrDisablePort(port, now, "port disabled after reset");
      } else {
        if (PortSpeedOf(ps) != 0) {
          p.speed = PortSpeedOf(ps);
        }
        p.fsm = PortFsm::kEnumerating;
        p.deadline = kNoDeadline;
        BeginDeviceEnumeration(port, now);
      }
    }
  }

  for (int i = 0; i < kMaxDevices; ++i) {
    DeviceSlot& dev = devices_[i];
    if (!dev.in_use) {
      continue;
    }
    if (dev.deadline != kNoDeadline && now >= dev.deadline) {
      if (dev.state == DevFsm::kAddressSettle) {
        dev.deadline = kNoDeadline;
        dev.state = DevFsm::kWaitingGetDesc8;
        SubmitControlTransfer(dev, kUsbReqTypeStdDevIn, kUsbReqGetDescriptor,
                              static_cast<uint16_t>(kUsbDescDevice << 8), 0, 8,
                              now);
      } else if (dev.state == DevFsm::kHubPowerSettle) {
        FinishHubAttach(i, now);
      } else if (dev.ctrl_in_flight) {
        dev.ctrl_in_flight = false;
        dev.deadline = kNoDeadline;
        if (dev.is_hub && dev.state == DevFsm::kConfigured) {
          const HubCtrlOp op = dev.hub_ctrl_op;
          const int port = dev.hub_ctrl_port;
          const uint16_t feat = dev.hub_ctrl_feature;
          dev.hub_ctrl_op = HubCtrlOp::kNone;
          dev.hub_ctrl_port = 0;
          dev.hub_ctrl_feature = 0;
          OnHubControlFailure(i, op, port, feat, /*timed_out=*/true, now);
        } else {
          RetryOrDisableDevice(dev, now, "control transfer timeout");
        }
      }
    }
    if (dev.in_use && dev.is_hub && dev.state == DevFsm::kConfigured) {
      if (dev.hub_rescan_deadline != kNoDeadline &&
          now >= dev.hub_rescan_deadline) {
        dev.hub_rescan_deadline = kNoDeadline;
        if (dev.hub_cold_rescans_left > 0 && !dev.ctrl_in_flight &&
            dev.hub_pending_scan_mask == 0 && HubAllPortsDisconnected(dev)) {
          --dev.hub_cold_rescans_left;
          dev.hub_pending_scan_mask |= (uint32_t{1} << dev.hub_num_ports) - 1u;
          PumpHubControl(i, now);
        } else if (!HubAllPortsDisconnected(dev)) {
          dev.hub_cold_rescans_left = 0;
        }
      }
      for (int p = 1; p <= dev.hub_num_ports; ++p) {
        if (!dev.in_use || dev.state != DevFsm::kConfigured) {
          break;
        }
        HubPortState& hp = dev.hub_ports[p - 1];
        if (hp.deadline == kNoDeadline || now < hp.deadline) {
          continue;
        }
        if (hp.fsm == HubPortFsm::kDebouncing) {
          hp.deadline = kNoDeadline;
          hp.fsm = HubPortFsm::kWantReset;
          PumpHubControl(i, now);
        } else if (hp.fsm == HubPortFsm::kResetting) {
          if (now - hp.debounce_start >=
              TicksForMs(*platform_, kPortResetTimeoutMs)) {
            RetryOrDisableHubPort(i, p, now, "hub port reset timeout");
          } else {
            // Poll the port; keep the absolute reset timeout in case the
            // poll's result is lost.
            hp.deadline =
                hp.debounce_start + TicksForMs(*platform_, kPortResetTimeoutMs);
            dev.hub_pending_scan_mask |= (uint32_t{1} << (p - 1));
            PumpHubControl(i, now);
          }
        } else if (hp.fsm == HubPortFsm::kResetRecovery) {
          hp.deadline = kNoDeadline;
          hp.fsm = HubPortFsm::kEnumerating;
          BeginHubDeviceEnumeration(i, p, now);
        }
      }
    }
  }

  // Resume hub ports that waited in kWantReset for their root port's
  // addressing (PumpHubControl() step 3). Whatever released it ran in this
  // call, so no deadline is needed; when idle there is no such port.
  for (int i = 0; i < kMaxDevices; ++i) {
    const DeviceSlot& d = devices_[i];
    if (!d.in_use || !d.is_hub || d.state != DevFsm::kConfigured ||
        d.ctrl_in_flight) {
      continue;
    }
    for (int p = 0; p < d.hub_num_ports; ++p) {
      if (d.hub_ports[p].fsm == HubPortFsm::kWantReset) {
        PumpHubControl(i, now);
        break;
      }
    }
  }

  PumpCommandRing(now);
}

// A command did not complete in time (e.g. Address Device to a device that
// NAKs SET_ADDRESS forever): abort it, and fail only its device, through its
// port's retry budget. The controller is reset only if the abort fails
// (AdvanceStateMachines()), as in Linux (xhci_handle_command_timeout()).
inline void XhciController::AbortCommand(const int64_t now) {
  StartLogLine().Str("command timeout, aborting it").Emit(*platform_);
  const CommandState stuck = cmd_state_;
  cmd_state_.aborting = true;
  cmd_state_.abandoned = true;
  cmd_state_.deadline = now + TicksForMs(*platform_, kCommandAbortTimeoutMs);
  // CA is in the low dword. The pointer bits written with it are ignored
  // while the ring runs (CRR = 1), but a controller that latches all 64 bits
  // on the high dword write, or whose ring stopped before it, takes them as
  // the new dequeue pointer: write the pointer where the stopped ring resumes
  // (the TRB after the stuck one) rather than 0, as Linux has done since
  // commit 09f736aa9547 (which superseded the low-dword-only write of
  // ff0e50d3564f: some controllers abort only on the 64-bit write).
  const uint64_t resume =
      cmd_ring_.enqueue_phys() | static_cast<uint64_t>(cmd_ring_.cycle());
  Write64LoHi(*mmio_, caps_.Op(kOpCrcr),
              resume | static_cast<uint64_t>(kCrcrCa));
  if (stuck.is_disable_slot) {
    RetryDisableSlot(stuck.disable_slot_id, "disable slot command timeout");
  } else if (!stuck.abandoned && stuck.dev_idx >= 0 &&
             stuck.dev_idx < kMaxDevices && devices_[stuck.dev_idx].in_use) {
    RetryOrDisableDevice(devices_[stuck.dev_idx], now, "command timeout");
  }
}

// The ring stopped after the abort: turn the stuck TRB into a No-Op (Linux
// xhci_handle_stopped_cmd_ring()). If the ring stopped on it rather than
// past it, the next doorbell then completes a No-Op instead of executing a
// command for a device the driver has given up on; its completion matches no
// command and is ignored. The next command restarts the ring (doorbell 0).
inline void XhciController::FinishCommandAbort(const int64_t now) {
  cmd_ring_.Rewrite(cmd_state_.trb_phys, MakeNoOpCommandTrb());
  cmd_state_ = CommandState{};
  PumpCommandRing(now);
}

inline void XhciController::DetachAllKeyboards() {
  for (int i = 0; i < kMaxDevices; ++i) {
    DeviceSlot& dev = devices_[i];
    if (dev.in_use && dev.announced) {
      AnnounceKeyboardDetach(dev);
    }
  }
  attached_keyboards_ = 0;
}

inline void XhciController::Recover(const char* const reason) {
  StartLogLine()
      .Str("controller error (")
      .Str(reason)
      .Str("), recovering")
      .Emit(*platform_);
  DetachAllKeyboards();

  // A budget per window, so that rare errors days apart never add up to a
  // dead controller, while a controller that keeps failing is given up on.
  const int64_t now = platform_->NowTicks();
  if (recoveries_ == 0 || now - recovery_window_start_ >
                              TicksForMs(*platform_, kRecoveryWindowMs)) {
    recoveries_ = 0;
    recovery_window_start_ = now;
  }
  ++recoveries_;
  if (recoveries_ > kMaxRecoveries) {
    StartLogLine().Str("error: recovery limit exceeded").Emit(*platform_);
    state_ = XhciState::kError;
    return;
  }

  const IrqMode saved_mode = irq_mode_;
  in_recovery_ = true;
  if (TakeOwnershipAndReset() != XhciStatus::kOk) {
    in_recovery_ = false;
    return;
  }
  (void)Start(saved_mode);
  in_recovery_ = false;
}

}  // namespace protos::usb

#endif  // PROTOS_USER_USBKBD_XHCI_CONTROLLER_H_
