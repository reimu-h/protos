#ifndef PROTOS_SYSCALL_ABI_H_
#define PROTOS_SYSCALL_ABI_H_

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace protos {

// The system call interface between user programs (user/) and the kernel
// (user_mode.h).
//
// User code makes a system call with `int 0x80` (`kVectorSyscall` in idt.h):
// the system call number goes in rax and up to six arguments go in rdi, rsi,
// rdx, r10, r8 and r9, and the result comes back in rax. All other registers,
// including the x87/MMX/SSE/AVX state with MXCSR and the x87 control word, and
// the flags are preserved. Results from -4095 to -1 are errors
// (`kSyscallError*`).
//
// The kernel serves system calls with interrupts enabled, like other kernel
// code: the program may be switched out in the middle of one and continue on
// another CPU.

// How a user program ended.
enum class UserProgramOutcome : uint8_t {
  // It called Exit(); `UserProgramResult::exit_status` holds the status.
  kExited = 0,
  // It raised a CPU exception, which killed it; the `fault_*` fields of
  // `UserProgramResult` describe the exception.
  kFaulted = 1,
  // `UserProcess::Kill()` killed it.
  kKilled = 2,
  // It never ran: its executable is invalid (elf.h, user_mode.h) or memory ran
  // out.
  kLoadFailed = 3,
};

// How a user program ended, as Wait() writes it to user memory. It has no
// implicit padding, which would carry stale kernel bytes to user space: the
// gaps are reserved fields, which are always zero (see the static_asserts
// below).
struct UserProgramResult {
  UserProgramOutcome outcome = UserProgramOutcome::kLoadFailed;
  uint8_t reserved0[7] = {};
  int64_t exit_status = 0;
  // The exception vector (idt.h), error code and the address of the faulting
  // instruction.
  int fault_vector = -1;
  uint32_t reserved1 = 0;
  uint64_t fault_error_code = 0;
  uint64_t fault_rip = 0;
  // For page faults, the address whose access faulted (CR2), else 0.
  uint64_t fault_address = 0;
};
static_assert(sizeof(UserProgramResult) == 48);
static_assert(alignof(UserProgramResult) == 8);
static_assert(offsetof(UserProgramResult, reserved0) == 1);
static_assert(offsetof(UserProgramResult, exit_status) == 8);
static_assert(offsetof(UserProgramResult, fault_vector) == 16);
static_assert(offsetof(UserProgramResult, reserved1) == 20);
static_assert(offsetof(UserProgramResult, fault_error_code) == 24);
static_assert(offsetof(UserProgramResult, fault_rip) == 32);
static_assert(offsetof(UserProgramResult, fault_address) == 40);
// Every byte belongs to a field.
static_assert(std::has_unique_object_representations_v<UserProgramResult>);

// Type of a PCI Base Address Register (`PciBarInfo::type`).
enum class PciBarInfoType : uint8_t {
  // Not implemented (or not decodable), or the upper 32-bit slot of the 64-bit
  // memory BAR at the preceding index.
  kNone = 0,
  // 32-bit memory-space BAR (MMIO, mapped below 4 GiB).
  kMemory32 = 1,
  // 64-bit memory-space BAR (MMIO, consumes two consecutive BAR slots;
  // recorded at the lower index).
  kMemory64 = 2,
  // x86 I/O-port BAR (`in`/`out` port space).
  kIo = 3,
};

// One Base Address Register of a PCI function (`PciDeviceInfo::bars`).
struct PciBarInfo {
  PciBarInfoType type = PciBarInfoType::kNone;
  // 1 if prefetchable (memory BARs), else 0.
  uint8_t prefetchable = 0;
  uint8_t reserved[6] = {};
  // Physical (or I/O port) base address assigned by firmware without flag bits,
  // or 0 if unassigned.
  uint64_t phys_base = 0;
  // Size in bytes (a power of two), or 0 if unassigned or not sized (only
  // claimable functions have their BARs sized).
  int64_t size = 0;
};
static_assert(sizeof(PciBarInfo) == 24);
static_assert(alignof(PciBarInfo) == 8);
static_assert(std::has_unique_object_representations_v<PciBarInfo>);

// Description of one PCI function, as PciInfo() writes it to user memory. Like
// `UserProgramResult`, it has no implicit padding: `reserved` is always zero.
struct PciDeviceInfo {
  // PCI location: bus 0..255, device 0..31, function 0..7.
  uint8_t bus = 0;
  uint8_t device = 0;
  uint8_t function = 0;
  // Header Type register: layout in bits 6:0 (0 = endpoint, 1 = PCI-to-PCI
  // bridge, 2 = CardBus) and multi-function flag (of function 0) in bit 7
  // (0x80).
  uint8_t header_type = 0;
  uint16_t vendor_id = 0;
  uint16_t device_id = 0;
  uint8_t class_code = 0;
  uint8_t subclass = 0;
  uint8_t prog_if = 0;
  uint8_t revision = 0;
  // 1 if a driver may claim the function with PciClaim() (header type 0 and
  // not a bridge of class 0x06), else 0.
  uint8_t claimable = 0;
  // Number of MSI vectors the function can request (1..32, a power of two), or
  // 0 if it has no MSI capability.
  uint8_t msi_vectors = 0;
  // MSI-X table size (1..2048) if the function has a valid MSI-X capability
  // whose table and PBA lie inside sized memory BARs, else 0.
  uint16_t msix_vectors = 0;
  // BAR0..5 for header type 0, BAR0..1 for header type 1. A 64-bit BAR is at
  // its lower index; the upper slot has `PciBarInfoType::kNone`.
  PciBarInfo bars[6] = {};
  // Index of the PCI-to-PCI bridge whose secondary bus the function is on, or
  // -1 on a root bus.
  int32_t parent_index = -1;
  // 1 if the MSI capability supports per-vector masking, else 0.
  uint8_t msi_masking = 0;
  uint8_t reserved[11] = {};
};
static_assert(sizeof(PciDeviceInfo) == 176);
static_assert(alignof(PciDeviceInfo) == 8);
static_assert(offsetof(PciDeviceInfo, vendor_id) == 4);
static_assert(offsetof(PciDeviceInfo, class_code) == 8);
static_assert(offsetof(PciDeviceInfo, claimable) == 12);
static_assert(offsetof(PciDeviceInfo, msix_vectors) == 14);
static_assert(offsetof(PciDeviceInfo, bars) == 16);
static_assert(offsetof(PciDeviceInfo, parent_index) == 160);
static_assert(offsetof(PciDeviceInfo, msi_masking) == 164);
static_assert(std::has_unique_object_representations_v<PciDeviceInfo>);

// Kind of message that `IpcRecv()` receives (`IpcReceivedMessage::kind`).
enum class IpcMessageKind : uint32_t {
  kNone = 0,
  // A call from a client's `IpcCall()`: `token` (non-zero) names it in the
  // server's `IpcReply()`, `client_id` identifies the calling process, `size`
  // (`0 <= size <= kIpcMaxMessageSize`) is the number of request bytes written
  // to the receive buffer, and `reply_cap` (`0 <= reply_cap <=
  // kIpcMaxMessageSize`) is the longest reply that the client accepts.
  kCall = 1,
  // Notification that client process `client_id` ended after the server had
  // received one of its calls (on `kIpcPortFs`: while it may have had files
  // open), so that the server can drop what it keeps for the client. `token`,
  // `size` and `reply_cap` are 0, and the server doesn't reply. It comes before
  // the calls that are queued when it is posted. A process makes one call at a
  // time, and if it ends in the middle of one, the kernel cancels the call
  // before it posts the notification. So a call of `client_id` that the server
  // has received and not answered yet when the notification comes was
  // canceled: `IpcReply()` drops its reply, and a server that answers calls
  // out of order should drop what it keeps for that call too.
  kClientDisconnect = 2,
};

// What `IpcRecv()` writes to user memory when it returns 1 (it writes nothing
// otherwise). `client_id` values are unique: the kernel never reuses one for
// another process. Like `UserProgramResult`, it has no implicit padding.
struct IpcReceivedMessage {
  uint64_t token = 0;
  uint64_t client_id = 0;
  int64_t size = 0;
  IpcMessageKind kind = IpcMessageKind::kNone;
  int32_t reply_cap = 0;
};
static_assert(sizeof(IpcReceivedMessage) == 32);
static_assert(alignof(IpcReceivedMessage) == 8);
static_assert(offsetof(IpcReceivedMessage, token) == 0);
static_assert(offsetof(IpcReceivedMessage, client_id) == 8);
static_assert(offsetof(IpcReceivedMessage, size) == 16);
static_assert(offsetof(IpcReceivedMessage, kind) == 24);
static_assert(offsetof(IpcReceivedMessage, reply_cap) == 28);
static_assert(std::has_unique_object_representations_v<IpcReceivedMessage>);

// Each file and directory of a filesystem has an fnode ("file node", the
// counterpart of a Unix inode), which holds everything about it but its name:
// directory entries map names to fnodes.
//
// Type of an fnode (`FsStat::type`, `FsDirEntry::type`).
enum class FsNodeType : uint8_t {
  kNone = 0,
  kRegular = 1,
  kDirectory = 2,
};

// Metadata of a file or directory, as `Stat()` writes it to user memory. Like
// `UserProgramResult`, it has no implicit padding: `reserved` is always zero.
struct FsStat {
  FsNodeType type = FsNodeType::kNone;
  uint8_t reserved[7] = {};
  // The fnode's ID, which no other file or directory of the same filesystem
  // server has had (the root's is 1).
  uint64_t fnode_id = 0;
  // Size in bytes; for a directory, `sizeof(FsDirEntry)` per entry.
  int64_t size = 0;
  // Number of links: 1 for a regular file, and 2 plus the number of
  // subdirectories for a directory.
  int64_t nlink = 0;
};
static_assert(sizeof(FsStat) == 32);
static_assert(alignof(FsStat) == 8);
static_assert(offsetof(FsStat, type) == 0);
static_assert(offsetof(FsStat, reserved) == 1);
static_assert(offsetof(FsStat, fnode_id) == 8);
static_assert(offsetof(FsStat, size) == 16);
static_assert(offsetof(FsStat, nlink) == 24);
static_assert(std::has_unique_object_representations_v<FsStat>);

// Maximum length of a single directory entry name (excluding NUL) and of a
// pathname passed to filesystem system calls.
constexpr int64_t kFsMaxNameLength = 63;
constexpr int64_t kFsMaxPathLength = 255;

// One directory entry returned by `ReadDir()`, written to user memory with no
// implicit padding. `name` holds `name_len` (`1 <= name_len <=
// kFsMaxNameLength`) bytes followed by NUL bytes through the end of the array.
struct FsDirEntry {
  // The entry's fnode (`FsStat::fnode_id`).
  uint64_t fnode_id = 0;
  FsNodeType type = FsNodeType::kNone;
  uint8_t reserved[3] = {};
  int32_t name_len = 0;
  char name[kFsMaxNameLength + 1] = {};
};
static_assert(sizeof(FsDirEntry) == 80);
static_assert(alignof(FsDirEntry) == 8);
static_assert(offsetof(FsDirEntry, fnode_id) == 0);
static_assert(offsetof(FsDirEntry, type) == 8);
static_assert(offsetof(FsDirEntry, reserved) == 9);
static_assert(offsetof(FsDirEntry, name_len) == 12);
static_assert(offsetof(FsDirEntry, name) == 16);
static_assert(std::has_unique_object_representations_v<FsDirEntry>);

// The longest boot module name (`BootModuleInfo::name`), without the NUL
// terminator.
constexpr int64_t kBootModuleMaxNameLength = 63;

// A boot module, a file that the boot loader loaded into memory next to the
// kernel, as `BootModuleInfo()` writes it to user memory. Like
// `UserProgramResult`, it has no implicit padding.
struct BootModuleInfo {
  // Size in bytes.
  int64_t size = 0;
  // The module's name: the last word (words are separated by spaces and tabs)
  // of the command line that the boot loader passed with it, cut to
  // `kBootModuleMaxNameLength` bytes and followed by NUL bytes through the end
  // of the array. GRUB 2 passes only the arguments after the file name of a
  // `module2` or `module` command, so `module2 /boot/rootfs.tar rootfs` names
  // the module `rootfs`, and so does a loader that passes the file name first,
  // as QEMU's `-initrd` and GRUB Legacy do (`/boot/rootfs.tar rootfs`).
  char name[kBootModuleMaxNameLength + 1] = {};
};
static_assert(sizeof(BootModuleInfo) == 72);
static_assert(alignof(BootModuleInfo) == 8);
static_assert(offsetof(BootModuleInfo, size) == 0);
static_assert(offsetof(BootModuleInfo, name) == 8);
static_assert(std::has_unique_object_representations_v<BootModuleInfo>);

// The name of the boot module that holds the boot filesystem: a POSIX ustar
// archive whose directories and regular files `ramfs` creates when it starts
// (`kDriverArgNormal`), so that the filesystem starts with them. The build
// packs the repository's `rootfs/` directory into it.
constexpr char kRootfsBootModuleName[] = "rootfs";

// One non-NUL-terminated byte slice passed as an argument string to
// `SpawnFile()`, which copies and NUL-terminates each argument onto the child's
// initial stack (`argv[0 .. argc - 1]`, followed by `argv[argc] == nullptr`).
struct SyscallArgSlice {
  uint64_t ptr = 0;
  int64_t len = 0;
};
static_assert(sizeof(SyscallArgSlice) == 16);
static_assert(alignof(SyscallArgSlice) == 8);
static_assert(offsetof(SyscallArgSlice, ptr) == 0);
static_assert(offsetof(SyscallArgSlice, len) == 8);
static_assert(std::has_unique_object_representations_v<SyscallArgSlice>);

// Exit(status): ends the program with exit status `status` (rdi, int64_t).
// Does not return.
constexpr int64_t kSyscallExit = 0;
// Write(buffer, size): writes the `size` (rsi) bytes at `buffer` (rdi) to the
// console, leaving out NUL bytes, and returns `size`. Fails with
// `kSyscallErrorInvalid` unless `0 <= size <= kSyscallWriteMaxSize`, and with
// `kSyscallErrorFault` if user code may not read all of the buffer.
constexpr int64_t kSyscallWrite = 1;
// GetCpu(): returns the index of the CPU on which the call started. The
// program may run on another CPU by the time it gets the result.
constexpr int64_t kSyscallGetCpu = 2;
// Spin(buffer, size, ticks): reads the `size` (rsi) bytes at `buffer` (rdi)
// over and over until the CPUs that run the call have taken `ticks` (rdx)
// timer interrupts while running it, and returns
// `SyscallSpinChecksum(buffer, size)`, which is never negative. Every timer
// interrupt that interrupts the call counts, also the one after which the
// program is switched out, so a call that shares its CPU with k - 1 other
// runnable tasks lasts about k * `ticks` timer ticks. A long system call for
// tests: it keeps the kernel busy reading user memory with interrupts
// enabled, and may be switched out and continue on another CPU meanwhile.
// `ticks == 0` reads the buffer once. If the program is killed in the middle
// of the call, the call stops early and the program ends instead of getting a
// result (user_mode.h). Fails with `kSyscallErrorInvalid` unless
// `0 <= size <= kSyscallSpinMaxSize` and `0 <= ticks <= kSyscallSpinMaxTicks`,
// with `kSyscallErrorFault` if user code may not read all of the buffer, and
// with `kSyscallErrorBusy`, once it notices, if the bytes change while the
// call reads them: the program, which waits for the call, can't change them,
// but a device can (one that writes a driver's DMA buffer, for example).
constexpr int64_t kSyscallSpin = 3;
// Read(buffer, size): reads between 1 and `size` (rsi) bytes of console input
// into `buffer` (rdi), waiting if none is available yet, and returns the
// number of bytes read (`0` when `size == 0`). If the program is killed while
// waiting, the call stops early and the program ends instead of getting a
// result. Fails with `kSyscallErrorInvalid` unless
// `0 <= size <= kSyscallReadMaxSize`, and with `kSyscallErrorFault` if
// `size > 0` and user code may not write all of the buffer.
constexpr int64_t kSyscallRead = 4;
// Spawn(name, name_len, arg): starts the embedded user program whose name is
// the `name_len` (rsi) bytes at `name` (rdi) in a new child user process with
// `arg` (rdx) in its rdi, and returns the child's process ID
// (`1 <= pid <= kMaxUserChildProcesses`). Fails with `kSyscallErrorInvalid`
// unless `1 <= name_len <= kSyscallSpawnMaxNameLength`, the name matches an
// embedded user program, a child slot is free, and the child process can be
// started, and with `kSyscallErrorFault` if user code may not read all of
// `[name, name + name_len)`.
constexpr int64_t kSyscallSpawn = 5;
// Wait(pid, result): waits until child process `pid` (rdi) of the calling
// process has ended, writes its `UserProgramResult` to `result` (rsi),
// reclaims the child slot, and returns 0. If the calling process is killed
// while waiting, the call stops early and the calling process ends as killed.
// Fails with `kSyscallErrorInvalid` unless `1 <= pid <= kMaxUserChildProcesses`
// and `pid` names an active child of the calling process, and with
// `kSyscallErrorFault` if user code may not write `sizeof(UserProgramResult)`
// bytes at `result`.
constexpr int64_t kSyscallWait = 6;
// SpawnDriver(name, name_len, arg): like Spawn(), but grants the child
// `kUserCapDriver` (device-driver capability). Requires `kUserCapSpawnDrivers`
// on the caller; otherwise fails with `kSyscallErrorPermission`.
constexpr int64_t kSyscallSpawnDriver = 7;
// PciInfo(index, out_info): returns the number of PCI functions when
// `out_info == 0 && index == 0`; otherwise writes `PciDeviceInfo` for `index`
// to `out_info` and returns the number of PCI functions. Driver-only.
constexpr int64_t kSyscallPciInfo = 8;
// PciClaim(index): claims PCI function `index` for the calling driver process
// and returns 0 on success. Fails with `kSyscallErrorNoDevice` if `index` is
// out of range, `kSyscallErrorInvalid` if the function is not claimable,
// `kSyscallErrorBusy` if another process owns it, or `kSyscallErrorNoMemory` if
// the caller already holds `kSyscallMaxClaims` claims. Driver-only.
constexpr int64_t kSyscallPciClaim = 9;
// PciBusMaster(index, enable): enables (`enable == 1`) or disables
// (`enable == 0`) Bus Master on claimed PCI function `index` and returns 0 on
// success. Fails with `kSyscallErrorInvalid` unless `enable` is 0 or 1,
// `kSyscallErrorNoDevice` if `index` is out of range, or `kSyscallErrorAccess`
// if the calling process does not own `index`. Driver-only.
constexpr int64_t kSyscallPciBusMaster = 10;
// MmioMap(phys_addr, size): maps `[phys_addr, phys_addr + size)` from a memory
// BAR of a claimed PCI function into the driver's user MMIO window (UC+NX) and
// returns the user virtual address. Driver-only.
constexpr int64_t kSyscallMmioMap = 11;
// DmaAlloc(size, out_phys, flags): allocates `size` bytes of contiguous zeroed
// physical RAM, writes the physical base to `*out_phys`, maps the pages into
// the driver's user DMA window (WB+NX), and returns the user virtual address.
// Driver-only.
constexpr int64_t kSyscallDmaAlloc = 12;
// IrqAlloc(index, count, mode): allocates `count` interrupt vectors for
// claimed PCI function `index` using `mode` (`kSyscallIrqMode*`) and returns
// the starting bit index `b` (`0..31`) in the process's `IrqWait` mask.
// Driver-only.
constexpr int64_t kSyscallIrqAlloc = 13;
// IrqWait(mask, timeout_ticks): waits for any interrupt bit in `mask` or
// `timeout_ticks` timer ticks, atomically clearing and returning the fired
// bits (`0` on timeout). Driver-only.
constexpr int64_t kSyscallIrqWait = 14;
// ConsolePush(buf, size): pushes up to `size` bytes of console input into the
// kernel console ring and returns the number of bytes pushed. Driver-only.
constexpr int64_t kSyscallConsolePush = 15;
// GetTicks(): returns the monotonic 250 Hz timer tick counter. Available to
// all processes.
constexpr int64_t kSyscallGetTicks = 16;
// IsaClaim(device): claims legacy ISA device `device` (`kIsaDevice*`)
// exclusively for the calling driver process and returns 0 on success. Fails
// with `kSyscallErrorNoDevice` if `device` is out of range,
// `kSyscallErrorBusy` if another process owns it, or `kSyscallErrorNoMemory` if
// the caller already holds `kSyscallMaxClaims` claims. Driver-only.
constexpr int64_t kSyscallIsaClaim = 17;
// IsaPortIo(op, port, value): executes an 8-bit x86 I/O-port read
// (`op == kSyscallPortIoRead8`, `value == 0`, returns `0..255`) or write
// (`op == kSyscallPortIoWrite8`, `0 <= value <= 255`, returns `0`) on `port`
// (`0..0xFFFF`). Fails with `kSyscallErrorInvalid` if `op`, `port`, or `value`
// is invalid, or `kSyscallErrorAccess` unless `port` belongs to a legacy ISA
// device currently claimed by the calling driver process. Driver-only.
constexpr int64_t kSyscallIsaPortIo = 18;
// IsaIrqAlloc(device): allocates 1 interrupt vector for claimed legacy ISA
// device `device`, programs and unmasks its I/O APIC redirection entry, and
// returns the bit index `b` (`0..31`) in the process's `IrqWait` mask. Fails
// with `kSyscallErrorNoDevice` if `device` is out of range or no I/O APIC
// routes its ISA IRQ, `kSyscallErrorAccess` if the caller has not claimed
// `device`, `kSyscallErrorBusy` if already allocated during this claim, or
// `kSyscallErrorNoMemory` if no vector or mask bit is free. Driver-only.
constexpr int64_t kSyscallIsaIrqAlloc = 19;
// IpcListen(port): claims IPC port `port` (rdi) for the calling process, which
// then receives the port's calls with IpcRecv() and answers them with
// IpcReply() until it ends, and returns 0 (also if the caller already owns
// `port`). Fails with `kSyscallErrorInvalid` unless
// `0 <= port < kIpcPortCount`, with `kSyscallErrorPermission` if `port` is
// `kIpcPortFs` and the caller lacks `kUserCapDriver`, with `kSyscallErrorBusy`
// if another process owns `port`, and with `kSyscallErrorNoMemory` if the
// kernel can't allocate the caller's receive buffer.
constexpr int64_t kSyscallIpcListen = 20;
// IpcRecv(port, buf, buf_cap, out_msg, timeout_ticks): receives the next
// message on port `port` (rdi), which the caller owns: a pending client
// disconnect notification first, else the oldest queued call. On a message,
// writes the call's request bytes to `buf` (rsi) and the message's
// `IpcReceivedMessage` to `out_msg` (r10), and returns 1. Waits for a message
// indefinitely if `timeout_ticks` (r8) is negative, for at most
// `timeout_ticks` timer ticks if it is positive, and not at all if it is 0,
// and returns 0 if none arrives in time. A queued call whose request is longer
// than `buf_cap` (rdx) bytes fails with `kSyscallErrorInvalid` for its client
// instead of being received. If the program is killed while waiting, the call
// stops early and the program ends instead of getting a result. Fails with
// `kSyscallErrorInvalid` unless `0 <= port < kIpcPortCount`,
// `0 <= buf_cap <= kIpcMaxMessageSize` and
// `timeout_ticks <= kSyscallIpcTimeoutMaxTicks`, with `kSyscallErrorFault` if
// user code may not write all of `out_msg` and of the `buf_cap` bytes at
// `buf`, and with `kSyscallErrorAccess` if the caller doesn't own `port`.
constexpr int64_t kSyscallIpcRecv = 21;
// IpcReply(port, token, reply_buf, reply_size, status): answers the call
// `token` (rsi) that the caller received on port `port` (rdi): the client's
// IpcCall() returns `status` (r8) with the `reply_size` (r10) bytes at
// `reply_buf` (rdx) as its reply. `status` is either `>= 0` or a
// `kSyscallError*` value with `reply_size == 0`. Returns 0, also if the reply
// is dropped because the client was killed after the call was received, which
// is the case for every call that is still unanswered when the client's
// `IpcMessageKind::kClientDisconnect` comes (the kernel remembers the 64 most
// recent such calls of each port, and answering an older one fails with
// `kSyscallErrorInvalid`). Fails with `kSyscallErrorInvalid` unless
// `0 <= port < kIpcPortCount`, `token` names a call received on `port` and not
// answered yet, `status` is valid and `0 <= reply_size <=` the call's
// `IpcReceivedMessage::reply_cap` (the call then stays unanswered), with
// `kSyscallErrorFault` if user code may not read all of the reply, and with
// `kSyscallErrorAccess` if the caller doesn't own `port`.
constexpr int64_t kSyscallIpcReply = 22;
// IpcCall(port, req_buf, req_size, reply_buf, reply_cap, out_reply_size):
// sends the `req_size` (rdx) bytes at `req_buf` (rsi) as a call to the owner
// of port `port` (rdi), waits until the owner answers it with IpcReply(),
// writes the reply to `reply_buf` (r10) and its size to `*out_reply_size` (r9,
// an `int64_t*`; skipped if 0), and returns the owner's `status`. The owner
// sees `reply_cap` (r8) as `IpcReceivedMessage::reply_cap` and can't send a
// longer reply. If the program is killed while waiting, the call stops early
// and the program ends instead of getting a result. Fails with
// `kSyscallErrorInvalid` unless `0 <= port < kIpcPortCount`,
// `0 <= req_size <= kIpcMaxMessageSize` and
// `0 <= reply_cap <= kIpcMaxMessageSize`, or if the request is longer than the
// owner's receive buffer; with `kSyscallErrorPermission` for `kIpcPortFs`,
// which only the kernel's filesystem system calls call; with
// `kSyscallErrorFault` if user code may not read all of the request or write
// all of the `reply_cap` bytes at `reply_buf` and of `*out_reply_size`; with
// `kSyscallErrorNoDevice` if no process owns `port`, or if its owner ends
// before answering; with `kSyscallErrorDeadlock` if the caller owns `port`;
// and with `kSyscallErrorNoMemory` if the kernel runs out of memory.
constexpr int64_t kSyscallIpcCall = 23;
// MemMap(size): maps `size` (rdi) bytes of zeroed, writable, non-executable
// memory into the calling process's anonymous memory window, followed by an
// unmapped guard page, and returns its user virtual address. The memory stays
// mapped until the program ends. Fails with `kSyscallErrorInvalid` unless
// `size` is a positive multiple of 4096 and at most `kSyscallMemMapMaxSize`,
// and with `kSyscallErrorNoMemory`, mapping nothing, if the process's
// mappings would exceed `kSyscallMemMapMaxTotalSize` bytes or its window, or
// if physical memory runs out.
constexpr int64_t kSyscallMemMap = 24;
// Filesystem system calls (Open() through Rename()). The user-space filesystem
// server that owns `kIpcPortFs` (`ramfs`, which `init` starts) serves them; the
// kernel checks their arguments and forwards them. A path is `path_len` bytes
// (without a NUL terminator) of '/'-separated components, where `.` and `..`
// work as usual; a path is resolved from the root `/` whether or not it starts
// with '/'. Besides the errors listed for each call, they fail with:
// - `kSyscallErrorFault` if user code may not read all of a path or access all
//   of a buffer;
// - `kSyscallErrorNoEntry` if `path_len == 0` or a path component doesn't
//   exist;
// - `kSyscallErrorNameTooLong` if `path_len > kFsMaxPathLength` or a component
//   is longer than `kFsMaxNameLength` bytes;
// - `kSyscallErrorInvalid` if `path_len < 0` or a path holds a NUL byte;
// - `kSyscallErrorNotDir` if a component before the last is not a directory;
// - `kSyscallErrorBadFd` unless `fd` is an open file descriptor;
// - `kSyscallErrorStale` for a descriptor opened by a filesystem server that
//   has ended since (and been replaced);
// - `kSyscallErrorNoDevice` if no filesystem server runs;
// - `kSyscallErrorDeadlock` if the caller is the filesystem server;
// - `kSyscallErrorNoMemory` if the kernel runs out of memory;
// - `kSyscallErrorIo` if the server's reply is malformed.
// If the program is killed while the server works on its call, the call stops
// early and the program ends instead of getting a result.
//
// Open(path, path_len, flags): opens the file or directory at the path of
// `path_len` (rsi) bytes at `path` (rdi) with `flags` (rdx, `kFsOpen*`), and
// returns the lowest free file descriptor (`0 <= fd < kMaxOpenFiles`), whose
// offset starts at 0. With `kFsOpenCreate`, creates an empty regular file if
// the path doesn't exist (Open() never creates directories; see MkDir()).
// Fails with `kSyscallErrorInvalid` unless `FsOpenFlagsValid(flags)`; with
// `kSyscallErrorNoEntry` if the path doesn't exist and `kFsOpenCreate` is
// clear; with `kSyscallErrorExists` if it exists and `flags` has both
// `kFsOpenCreate` and `kFsOpenExclusive`; with `kSyscallErrorIsDir` if it is a
// directory and `flags` asks for write access, `kFsOpenCreate`,
// `kFsOpenTruncate` or `kFsOpenAppend`, or if a file would be created for a
// path that ends with '/' or with `kFsOpenDirectory`; with
// `kSyscallErrorNotDir` if it is a regular file and the path ends with '/' or
// `flags` has `kFsOpenDirectory`; with `kSyscallErrorTooManyFiles` if the
// caller has `kMaxOpenFiles` open descriptors; with
// `kSyscallErrorFileTableFull` if the server's table of open files is full;
// and with `kSyscallErrorNoSpace` if the server has no room for a new file.
constexpr int64_t kSyscallOpen = 25;
// Close(fd): closes file descriptor `fd` (rdi) and returns 0. The descriptor
// is free afterwards even if Close() fails with an error of the server or the
// kernel, and Close() returns 0 if the server that opened it has ended.
constexpr int64_t kSyscallClose = 26;
// FileRead(fd, buf, size): reads up to `size` (rdx) bytes of regular file `fd`
// (rdi), from its offset on, into `buf` (rsi), advances the offset past them,
// and returns their number (`0` at or past the end of the file). Holes read as
// zeros. Fails with `kSyscallErrorInvalid` unless
// `0 <= size <= kSyscallFsMaxIoSize`, with `kSyscallErrorBadFd` if `fd` was
// opened write-only, and with `kSyscallErrorIsDir` if `fd` is a directory.
constexpr int64_t kSyscallFileRead = 27;
// FileWrite(fd, buf, size): writes the `size` (rdx) bytes at `buf` (rsi) to
// regular file `fd` (rdi) at its offset (at the end of the file if `fd` was
// opened with `kFsOpenAppend`), advances the offset past them, and returns the
// number of bytes written: fewer than `size` if the server runs out of space
// or the file reaches `kFsMaxFileSize` bytes. Writing past the end of the file
// extends it with a hole. Fails with `kSyscallErrorInvalid` unless
// `0 <= size <= kSyscallFsMaxIoSize`, with `kSyscallErrorBadFd` if `fd` was
// opened read-only, with `kSyscallErrorIsDir` if `fd` is a directory, with
// `kSyscallErrorFileTooBig` if `size > 0` and the offset is at or past
// `kFsMaxFileSize`, and with `kSyscallErrorNoSpace` if the server has no space
// for any of the bytes.
constexpr int64_t kSyscallFileWrite = 28;
// Seek(fd, offset, whence): sets the offset of regular file `fd` (rdi) to
// `offset` (rsi) bytes past its start (`whence` (rdx) is `kFsSeekSet`), its
// current offset (`kFsSeekCur`) or its end (`kFsSeekEnd`), and returns the new
// offset. For a directory, only `Seek(fd, 0, kFsSeekSet)`, which rewinds
// ReadDir(), and `Seek(fd, 0, kFsSeekCur)`, which returns an opaque position,
// work. Fails with `kSyscallErrorInvalid` if `whence` is invalid or the new
// offset would be negative or past `kFsMaxFileSize`.
constexpr int64_t kSyscallSeek = 29;
// Stat(path, path_len, out_stat): writes the `FsStat` of the file or directory
// at the path of `path_len` (rsi) bytes at `path` (rdi) to `out_stat` (rdx),
// and returns 0. Checks `out_stat` (`kSyscallErrorFault`) before the path.
constexpr int64_t kSyscallStat = 30;
// ReadDir(fd, out_entries, max_entries): reads up to `max_entries` (rdx)
// entries of directory `fd` (rdi) into `out_entries` (rsi), continuing where
// the previous ReadDir() of `fd` stopped, and returns their number (`0` at the
// end of the directory). There are no `.` and `..` entries. Each entry that
// stays unchanged throughout the reads appears once; an entry added, removed or
// renamed meanwhile (Rename() removes the old entry and adds a new one, also
// when replacing an existing target) may appear zero or more times. Fails with
// `kSyscallErrorInvalid` unless `0 <= max_entries <= kSyscallFsMaxDirEntries`,
// and with `kSyscallErrorNotDir` if `fd` is a regular file.
constexpr int64_t kSyscallReadDir = 31;
// MkDir(path, path_len): creates an empty directory at the path of `path_len`
// (rsi) bytes at `path` (rdi) and returns 0. Fails with `kSyscallErrorExists`
// if the path exists, and with `kSyscallErrorNoSpace` if the server has no
// room for a new directory.
constexpr int64_t kSyscallMkDir = 32;
// Unlink(path, path_len): removes the regular file or empty directory at the
// path of `path_len` (rsi) bytes at `path` (rdi) and returns 0. A removed file
// stays readable and writable through the descriptors that have it open until
// they are closed. Fails with `kSyscallErrorBusy` for the root `/`, with
// `kSyscallErrorInvalid` if the last component is `.` or `..`, with
// `kSyscallErrorNotEmpty` for a directory that has entries, and with
// `kSyscallErrorNotDir` for a regular file whose path ends with '/'.
constexpr int64_t kSyscallUnlink = 33;
// Rename(old_path, old_len, new_path, new_len): moves the file or directory at
// the path of `old_len` (rsi) bytes at `old_path` (rdi) to the path of
// `new_len` (r10) bytes at `new_path` (rdx), replacing a regular file or empty
// directory there, and returns 0 (also if both paths name the same entry).
// Fails with `kSyscallErrorBusy` if either path is the root `/`; with
// `kSyscallErrorInvalid` if either last component is `.` or `..`, or if a
// directory would move into itself or one of its subdirectories; with
// `kSyscallErrorIsDir` if a regular file would replace a directory; with
// `kSyscallErrorNotDir` if a directory would replace a regular file, or a
// regular file's path ends with '/'; and with `kSyscallErrorNotEmpty` if the
// directory to replace has entries.
constexpr int64_t kSyscallRename = 34;
// BootModuleInfo(index, out_info): returns the number of boot modules (files
// that the boot loader loaded next to the kernel, such as the boot filesystem
// `kRootfsBootModuleName`) when `index` (rdi) and `out_info` (rsi) are both 0;
// otherwise writes the `BootModuleInfo` of module `index` to `out_info` and
// returns the number of modules. Fails with `kSyscallErrorInvalid` unless
// `0 <= index <` the number of modules, and otherwise with
// `kSyscallErrorFault` if user code may not write all of `out_info`.
// Driver-only.
constexpr int64_t kSyscallBootModuleInfo = 35;
// BootModuleRead(index, offset, buffer, size): copies up to `size` (r10) bytes
// of boot module `index` (rdi), from byte `offset` (rsi) on, to `buffer`
// (rdx), and returns their number (`0` at or past the end of the module).
// Fails with `kSyscallErrorInvalid` unless `0 <= index <` the number of
// modules, `offset >= 0` and `0 <= size <= kSyscallBootModuleReadMaxSize`, and
// otherwise with `kSyscallErrorFault` if `size > 0` and user code may not
// write all of the `size` bytes at `buffer`. Driver-only.
constexpr int64_t kSyscallBootModuleRead = 36;
// SpawnFile(path, path_len, args, argc, arg): reads the ELF executable at the
// path of `path_len` (rsi) bytes at `path` (rdi) from the filesystem server,
// copies the `argc` (r10) `SyscallArgSlice` strings at `args` (rdx) onto the
// child's initial user stack as NUL-terminated `argv[0 .. argc - 1]` (with
// `argv[argc] == nullptr`), starts the child in a new unprivileged
// (`kUserCapNone`) user process with `arg` (r8) in rdi, `argc` in rsi and
// `argv` in rdx, and returns the child's process ID
// (`1 <= pid <= kMaxUserChildProcesses`), without taking one of the caller's
// open file descriptors. Fails with the path and server errors of the
// filesystem system calls (and `kSyscallErrorFileTableFull` if the server's
// table of open files is full); with `kSyscallErrorInvalid` unless
// `0 <= argc <= kSyscallSpawnMaxArgs`, each slice has `len >= 0` with no NUL
// byte, and the sum of slice lengths is at most `kSyscallSpawnMaxArgBytes`;
// with `kSyscallErrorFault` if user code may not read the slice array or each
// slice's bytes; with `kSyscallErrorBusy` if all `kMaxUserChildProcesses`
// child slots of the caller are in use; with `kSyscallErrorIsDir` if the path
// names a directory; with `kSyscallErrorExecFormat` if the file is not a valid
// user-space ELF executable (elf.h, user_mode.h) or ends before its segments
// do; and with `kSyscallErrorNoMemory` if memory or tasks run out.
constexpr int64_t kSyscallSpawnFile = 37;

// Total number of defined system calls (`0 .. kSyscallCount - 1`).
constexpr int64_t kSyscallCount = 38;

// The most bytes that one Write() writes.
constexpr int64_t kSyscallWriteMaxSize = 4096;
// The most bytes that one Spin() reads, and the most timer ticks that it lasts
// (1000000 ticks are more than an hour at 250 Hz).
constexpr int64_t kSyscallSpinMaxSize = 4096;
constexpr int64_t kSyscallSpinMaxTicks = 1000000;
// The most bytes that one Read() reads.
constexpr int64_t kSyscallReadMaxSize = 4096;
// The longest program name that Spawn() accepts, the most arguments and total
// argument bytes that SpawnFile() accepts, and the most unwaited child
// processes that one process may have at a time.
constexpr int64_t kSyscallSpawnMaxNameLength = 64;
constexpr int64_t kSyscallSpawnMaxArgs = 16;
constexpr int64_t kSyscallSpawnMaxArgBytes = 1024;
constexpr int64_t kMaxUserChildProcesses = 8;

// Driver syscall limits, flags and modes.
constexpr int64_t kSyscallConsolePushMaxSize = 256;
constexpr int64_t kSyscallMmioMaxSize = 16LL << 20;
constexpr int64_t kSyscallDmaMaxSize = 1LL << 20;
constexpr int64_t kSyscallDmaMaxTotalSize = 16LL << 20;
constexpr int kSyscallMaxClaims = 16;

constexpr uint64_t kSyscallDmaBelow4GiB = 1ULL << 0;
constexpr uint64_t kSyscallDmaFlagsMask = kSyscallDmaBelow4GiB;

constexpr int64_t kSyscallIrqModeAuto = 0;
constexpr int64_t kSyscallIrqModeMsi = 1;
constexpr int64_t kSyscallIrqModeMsix = 2;
constexpr int64_t kSyscallIrqMaxCount = 32;
constexpr int64_t kSyscallIrqWaitMaxTicks = 1000000;
constexpr int64_t kSyscallTicksPerSecond = 250;

// Legacy ISA device identifiers (`IsaClaim`, `IsaIrqAlloc`) and port I/O
// operations (`IsaPortIo`).
constexpr int kIsaDeviceI8042 = 0;
constexpr int kIsaDeviceCount = 1;

constexpr int64_t kSyscallPortIoRead8 = 0;
constexpr int64_t kSyscallPortIoWrite8 = 1;

// IPC ports and limits (`IpcListen()`, `IpcRecv()`, `IpcReply()`,
// `IpcCall()`). Port `kIpcPortFs` belongs to the filesystem server: only
// `kUserCapDriver` processes may claim it, and only the kernel's filesystem
// system calls call it. Any process may claim and call the other ports.
constexpr int kIpcPortFs = 0;
constexpr int kIpcFirstUserPort = 1;
constexpr int kIpcPortCount = 16;
// The longest request or reply: room for the longest filesystem request, an
// `FsWriteRequest` followed by `kSyscallFsMaxIoSize` data bytes
// (fs_protocol.h), and some to spare.
constexpr int64_t kIpcMaxMessageSize = 4352;
constexpr int64_t kSyscallIpcTimeoutMaxTicks = 1000000;

// Anonymous user memory mapping limits (`MemMap`).
constexpr int64_t kSyscallMemMapMaxSize = 1LL << 20;
constexpr int64_t kSyscallMemMapMaxTotalSize = 64LL << 20;

// The most bytes that one BootModuleRead() copies.
constexpr int64_t kSyscallBootModuleReadMaxSize = 4096;

// Filesystem limits, `Open()` flags, and `Seek()` modes.
constexpr int kMaxOpenFiles = 32;
constexpr int64_t kSyscallFsMaxIoSize = 4096;
constexpr int64_t kSyscallFsMaxDirEntries = 32;
constexpr int64_t kFsMaxFileSize = 32LL << 20;

constexpr uint64_t kFsOpenAccessMask = 0x3ULL;
constexpr uint64_t kFsOpenRead = 0x0ULL;          // O_RDONLY: read-only
constexpr uint64_t kFsOpenWrite = 0x1ULL;         // O_WRONLY: write-only
constexpr uint64_t kFsOpenReadWrite = 0x2ULL;     // O_RDWR: read and write
constexpr uint64_t kFsOpenCreate = 1ULL << 2;     // O_CREAT: create if missing
constexpr uint64_t kFsOpenTruncate = 1ULL << 3;   // O_TRUNC: truncate to 0
constexpr uint64_t kFsOpenAppend = 1ULL << 4;     // O_APPEND: append writes
constexpr uint64_t kFsOpenExclusive = 1ULL << 5;  // O_EXCL: fail if exists
constexpr uint64_t kFsOpenDirectory = 1ULL << 6;  // O_DIRECTORY: require dir
constexpr uint64_t kFsOpenFlagsMask = kFsOpenAccessMask | kFsOpenCreate |
                                      kFsOpenTruncate | kFsOpenAppend |
                                      kFsOpenExclusive | kFsOpenDirectory;

// True if `flags` is a valid `Open()` flag combination: only `kFsOpen*` bits,
// one of the three access modes, `kFsOpenExclusive` only with
// `kFsOpenCreate`, and `kFsOpenTruncate` only with write access.
constexpr bool FsOpenFlagsValid(const uint64_t flags) {
  const uint64_t access = flags & kFsOpenAccessMask;
  return (flags & ~kFsOpenFlagsMask) == 0 &&
         (access == kFsOpenRead || access == kFsOpenWrite ||
          access == kFsOpenReadWrite) &&
         ((flags & kFsOpenExclusive) == 0 || (flags & kFsOpenCreate) != 0) &&
         ((flags & kFsOpenTruncate) == 0 || access != kFsOpenRead);
}

constexpr int64_t kFsSeekSet = 0;  // SEEK_SET: offset from start of file
constexpr int64_t kFsSeekCur = 1;  // SEEK_CUR: offset from current position
constexpr int64_t kFsSeekEnd = 2;  // SEEK_END: offset from end of file

// Errors, with the numbers of the corresponding Linux errno values.
constexpr int64_t kSyscallErrorPermission = -1;      // EPERM: not allowed
constexpr int64_t kSyscallErrorNoEntry = -2;         // ENOENT: no such entry
constexpr int64_t kSyscallErrorIo = -5;              // EIO: I/O or RPC error
constexpr int64_t kSyscallErrorExecFormat = -8;      // ENOEXEC: bad executable
constexpr int64_t kSyscallErrorBadFd = -9;           // EBADF: bad descriptor
constexpr int64_t kSyscallErrorNoMemory = -12;       // ENOMEM: out of memory
constexpr int64_t kSyscallErrorAccess = -13;         // EACCES: access denied
constexpr int64_t kSyscallErrorFault = -14;          // EFAULT: bad address
constexpr int64_t kSyscallErrorBusy = -16;           // EBUSY: resource busy
constexpr int64_t kSyscallErrorExists = -17;         // EEXIST: file exists
constexpr int64_t kSyscallErrorNoDevice = -19;       // ENODEV: no such device
constexpr int64_t kSyscallErrorNotDir = -20;         // ENOTDIR: not a directory
constexpr int64_t kSyscallErrorIsDir = -21;          // EISDIR: is a directory
constexpr int64_t kSyscallErrorInvalid = -22;        // EINVAL: invalid argument
constexpr int64_t kSyscallErrorFileTableFull = -23;  // ENFILE: file table full
constexpr int64_t kSyscallErrorTooManyFiles = -24;   // EMFILE: too many files
constexpr int64_t kSyscallErrorFileTooBig = -27;     // EFBIG: file too large
constexpr int64_t kSyscallErrorNoSpace = -28;        // ENOSPC: no space left
constexpr int64_t kSyscallErrorDeadlock = -35;       // EDEADLK: self-IPC call
constexpr int64_t kSyscallErrorNameTooLong = -36;    // ENAMETOOLONG: long name
constexpr int64_t kSyscallErrorNoSys = -38;          // ENOSYS: unknown syscall
constexpr int64_t kSyscallErrorNotEmpty = -39;       // ENOTEMPTY: dir not empty
constexpr int64_t kSyscallErrorStale = -116;         // ESTALE: stale descriptor

// Process capability bits (user_mode.h).
constexpr uint64_t kUserCapNone = 0;
constexpr uint64_t kUserCapDriver = 1ULL << 0;
constexpr uint64_t kUserCapSpawnDrivers = 1ULL << 1;

// Bit flags in `init`'s `arg` (`rdi` at entry):
// - `kInitArgStartDrivers`: spawn user-space hardware drivers (`usbkbd`,
//   `ps2kbd`) AND the user-space filesystem server (`ramfs`) via `SpawnDriver`
//   before starting the shell.
// - `kInitArgStartFs`: spawn `ramfs` via `SpawnDriver` before starting the
//   shell even when hardware drivers are omitted (for isolated tests).
// - `kInitArgEmptyFs`: start `ramfs` with `kRamfsArgEmpty`, so that the
//   filesystem starts empty instead of with the boot filesystem (for tests).
constexpr uint64_t kInitArgStartDrivers = 1ULL << 0;
constexpr uint64_t kInitArgStartFs = 1ULL << 1;
constexpr uint64_t kInitArgEmptyFs = 1ULL << 2;

// Standard argument modes for driver processes (`Main(uint64_t arg)`).
// - `kDriverArgNormal` (0): normal interrupt-driven daemon loop (exits with 0
//   if no matching hardware devices are present). `ramfs` first creates the
//   files of the boot filesystem (`kRootfsBootModuleName`), if the boot loader
//   loaded it.
// - `kDriverArgExitAfterReady` (1): claims all matching devices, completes
//   two-phase hardware bring-up and one initial interrupt poll, then exits
//   with status 0 (or non-zero if any discovered device failed).
// - `kDriverArgFaultAfterReady` (2): same as `kDriverArgExitAfterReady`, but
//   executes `ud2` (`#UD`) after all controllers are running with Bus Master,
//   MMIO, DMA, and MSI/MSI-X active.
// - `kDriverArgHandoffOnly` (3): claims all matching controllers, completes
//   firmware/SMM ownership handoff and halts any running controller without
//   issuing a full hardware reset, then exits with status 0 so legacy-port
//   drivers (such as `ps2kbd` on `0x60`/`0x64`) can safely probe afterwards.
// - `kRamfsArgEmpty` (4, `ramfs` only): like `kDriverArgNormal`, but the
//   filesystem starts empty, without the boot filesystem (for tests).
constexpr uint64_t kDriverArgNormal = 0;
constexpr uint64_t kDriverArgExitAfterReady = 1;
constexpr uint64_t kDriverArgFaultAfterReady = 2;
constexpr uint64_t kDriverArgHandoffOnly = 3;
constexpr uint64_t kRamfsArgEmpty = 4;

// Exit status of a driver program started without `kUserCapDriver`, that is by
// `Spawn()` (as `sh`'s `run <program>` does) rather than by `SpawnDriver()`:
// its driver system calls fail with `kSyscallErrorPermission`.
constexpr int64_t kDriverExitNotDriver = 3;

// True if `result` is in the system call error range `[-4095, -1]`. User
// virtual addresses in the higher half (`>= 0xFFFF800000000000`) cast to
// negative `int64_t` values `< -4095`, so only `SyscallIsError()` distinguishes
// errors from valid pointer results.
constexpr bool SyscallIsError(const int64_t result) {
  return result >= -4095 && result < 0;
}

// True if system call `number` requires `kUserCapDriver` (`8..15`, `17..19`
// or `35..36`).
constexpr bool SyscallIsDriverOnly(const int64_t number) {
  return (number >= kSyscallPciInfo && number <= kSyscallConsolePush) ||
         (number >= kSyscallIsaClaim && number <= kSyscallIsaIrqAlloc) ||
         (number >= kSyscallBootModuleInfo && number <= kSyscallBootModuleRead);
}

// True if a process with `capabilities` is allowed to invoke system call
// `number` (before any per-syscall argument validation).
constexpr bool MayMakeSyscall(const uint64_t capabilities,
                              const int64_t number) {
  if (number == kSyscallSpawnDriver) {
    return (capabilities & kUserCapSpawnDrivers) != 0;
  }
  if (SyscallIsDriverOnly(number)) {
    return (capabilities & kUserCapDriver) != 0;
  }
  return true;
}

// The checksum of no bytes; see `SyscallSpinChecksum()`.
constexpr int64_t kSyscallSpinChecksumInit = 0x4BF29CE484222325;

// Returns Spin()'s checksum of the bytes whose checksum is `checksum`,
// followed by the `size` (>= 0) bytes at `data`: the 64-bit FNV-1a hash of all
// of them with bit 63 cleared, so that it is never negative and never an
// error. Clearing bit 63 after every byte gives the same result, so a buffer
// can be checksummed piece by piece.
constexpr int64_t SyscallSpinChecksum(
    const uint8_t* const data,  //
    const int64_t size,         //
    const int64_t checksum = kSyscallSpinChecksumInit) {
  uint64_t hash = static_cast<uint64_t>(checksum);
  for (int64_t i = 0; i < size; ++i) {
    hash = (hash ^ data[i]) * 0x100000001B3ULL;
  }
  return static_cast<int64_t>(hash & 0x7FFFFFFFFFFFFFFFULL);
}

}  // namespace protos

#endif  // PROTOS_SYSCALL_ABI_H_
