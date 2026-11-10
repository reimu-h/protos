#ifndef PROTOS_BLOCK_PROTOCOL_H_
#define PROTOS_BLOCK_PROTOCOL_H_

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "syscall_abi.h"

namespace protos {

// Binary IPC protocol between block clients (user programs or storage servers)
// and the user-space block device driver (`user/nvme/`) over `kIpcPortBlock`.
// Every request begins with a 32-bit `BlockRpcOp` opcode, every struct has no
// implicit padding (`std::has_unique_object_representations_v`), and `reserved`
// fields must be zero.
//
// On failure, the server replies with a negative `kSyscallError*` status and 0
// reply bytes:
// - `kSyscallErrorNoDevice`: `disk_index` is out of range (`>= disk_count`).
// - `kSyscallErrorInvalid`: malformed request size, unknown opcode, non-zero
//   `reserved`, invalid `lba` / `block_count`, or insufficient `reply_cap`.
// - `kSyscallErrorPermission`: write attempted on a `read_only` device.
// - `kSyscallErrorIo`: controller/media failure or internal device error.

// Logical block size bounds (`block_size` is a power of two in
// `[kBlockMinBlockSize, kBlockMaxIoBytes]`: 512, 1024, 2048, or 4096) and most
// payload bytes transferred by a single `kRead` or `kWrite` RPC: chosen so a
// `BlockWriteRequest` plus payload fits inside one `kIpcMaxMessageSize`
// message (`4352` bytes) and never spans two 4 KiB physical pages in DMA.
constexpr int32_t kBlockMinBlockSize = 512;
constexpr int64_t kBlockMaxIoBytes = 4096;
static_assert(kBlockMinBlockSize <= kBlockMaxIoBytes);
static_assert(kBlockMaxIoBytes % kBlockMinBlockSize == 0);

// Capacities of `BlockDeviceInfo::model` and `BlockDeviceInfo::serial`,
// including the mandatory trailing NUL byte (so ceilings are 39 and 23
// non-NUL ASCII characters respectively).
constexpr int kBlockModelCapacity = 40;
constexpr int kBlockSerialCapacity = 24;

// A block device whose serial number starts with `kBlockTestDiskSerialPrefix`
// is a disposable test image: QEMU's `-device nvme,serial=protos0`, as `make
// run` and the boot tests attach it. The self-tests write only to such
// devices, and `/bin/disk` writes to any other one, such as a real disk when
// the kernel runs on a real machine, only if `-f` tells it to.
constexpr char kBlockTestDiskSerialPrefix[] = "protos";

// Whether `serial`, a NUL-terminated string such as the `serial` of a valid
// `BlockDeviceInfo`, is the serial number of a disposable test image.
constexpr bool IsBlockTestDiskSerial(const char* const serial) {
  for (int i = 0; kBlockTestDiskSerialPrefix[i] != '\0'; ++i) {
    if (serial[i] != kBlockTestDiskSerialPrefix[i]) {
      return false;
    }
  }
  return true;
}
static_assert(IsBlockTestDiskSerial("protos0"));
static_assert(!IsBlockTestDiskSerial("proto"));

enum class BlockRpcOp : uint32_t {
  kNone = 0,
  kGetInfo = 1,
  kRead = 2,
  kWrite = 3,
  kFlush = 4,
};

// `kGetInfo`: describes block device `disk_index`. Replies with exact
// `sizeof(BlockDeviceInfo)` (88) bytes and status equal to the total number of
// discovered usable block devices (`>= 1`, so `0 <= disk_index < status`), or
// `kSyscallErrorNoDevice` if `disk_index` is out of range (or no usable disks
// exist).
struct BlockGetInfoRequest {
  BlockRpcOp op = BlockRpcOp::kGetInfo;
  uint32_t disk_index = 0;
};
static_assert(sizeof(BlockGetInfoRequest) == 8);
static_assert(alignof(BlockGetInfoRequest) == 4);
static_assert(offsetof(BlockGetInfoRequest, op) == 0);
static_assert(offsetof(BlockGetInfoRequest, disk_index) == 4);
static_assert(std::has_unique_object_representations_v<BlockGetInfoRequest>);

// Wire metadata describing one block device (`sizeof == 88` ABI). Valid
// responses enforce:
// - `block_count >= 1`;
// - `block_size` is a power of two in `[kBlockMinBlockSize, kBlockMaxIoBytes]`;
// - `max_transfer_blocks >= 1` and
//   `int64_t{max_transfer_blocks} * block_size <= kBlockMaxIoBytes`;
// - `bus`, `device`, `function`: informational PCI BDF location;
// - `read_only` in `{0, 1}` (0 = read-write, 1 = read-only);
// - `nsid`: informational namespace identifier (`>= 1` on NVMe namespaces);
// - `model` (max 39 chars) and `serial` (max 23 chars) are NUL-terminated
//   within their arrays (producers emit printable ASCII padded with NUL
//   bytes; wire validation checks NUL termination).
struct BlockDeviceInfo {
  uint64_t block_count = 0;
  int32_t block_size = 0;
  int32_t max_transfer_blocks = 0;
  uint8_t bus = 0;
  uint8_t device = 0;
  uint8_t function = 0;
  uint8_t read_only = 0;
  uint32_t nsid = 0;
  // NUL-terminated strings (servers emit printable ASCII, NUL-padded).
  char model[kBlockModelCapacity] = {};
  char serial[kBlockSerialCapacity] = {};
};
static_assert(sizeof(BlockDeviceInfo) == 88);
static_assert(alignof(BlockDeviceInfo) == 8);
static_assert(offsetof(BlockDeviceInfo, block_count) == 0);
static_assert(offsetof(BlockDeviceInfo, block_size) == 8);
static_assert(offsetof(BlockDeviceInfo, max_transfer_blocks) == 12);
static_assert(offsetof(BlockDeviceInfo, bus) == 16);
static_assert(offsetof(BlockDeviceInfo, device) == 17);
static_assert(offsetof(BlockDeviceInfo, function) == 18);
static_assert(offsetof(BlockDeviceInfo, read_only) == 19);
static_assert(offsetof(BlockDeviceInfo, nsid) == 20);
static_assert(offsetof(BlockDeviceInfo, model) == 24);
static_assert(offsetof(BlockDeviceInfo, serial) == 64);
static_assert(std::has_unique_object_representations_v<BlockDeviceInfo>);

// `kRead`: reads `block_count` (`>= 0`) logical blocks starting at `lba` from
// device `disk_index`.
// - `block_count == 0` is a valid no-op when `disk_index` is valid and
//   `lba <= info.block_count`: accepts `reply_cap == 0`, replies with 0 bytes
//   and status 0, and does not touch caller buffers.
// - `block_count > 0` requires `block_count <= max_transfer_blocks`,
//   `lba <= info.block_count - block_count`, and
//   `reply_cap >= int64_t{block_count} * info.block_size`; replies with
//   `int64_t{block_count} * info.block_size` bytes and status `block_count`.
struct BlockReadRequest {
  BlockRpcOp op = BlockRpcOp::kRead;
  uint32_t disk_index = 0;
  uint64_t lba = 0;
  int32_t block_count = 0;
  uint32_t reserved = 0;
};
static_assert(sizeof(BlockReadRequest) == 24);
static_assert(alignof(BlockReadRequest) == 8);
static_assert(offsetof(BlockReadRequest, op) == 0);
static_assert(offsetof(BlockReadRequest, disk_index) == 4);
static_assert(offsetof(BlockReadRequest, lba) == 8);
static_assert(offsetof(BlockReadRequest, block_count) == 16);
static_assert(offsetof(BlockReadRequest, reserved) == 20);
static_assert(std::has_unique_object_representations_v<BlockReadRequest>);
static_assert(kBlockMaxIoBytes <= kIpcMaxMessageSize);

// `kWrite`: followed immediately by `int64_t{block_count} * info.block_size`
// raw bytes to write starting at `lba` on device `disk_index` (total request
// size must equal `sizeof(BlockWriteRequest) + payload_bytes`).
// - `block_count == 0` is a valid no-op when `disk_index` is valid and
//   `lba <= info.block_count` (24-byte header, 0 payload bytes): replies with 0
//   bytes and status 0.
// - `block_count > 0` requires `block_count <= max_transfer_blocks` and
//   `lba <= info.block_count - block_count`; replies with 0 bytes and status
//   `block_count`.
struct BlockWriteRequest {
  BlockRpcOp op = BlockRpcOp::kWrite;
  uint32_t disk_index = 0;
  uint64_t lba = 0;
  int32_t block_count = 0;
  uint32_t reserved = 0;
};
static_assert(sizeof(BlockWriteRequest) == 24);
static_assert(alignof(BlockWriteRequest) == 8);
static_assert(offsetof(BlockWriteRequest, op) == 0);
static_assert(offsetof(BlockWriteRequest, disk_index) == 4);
static_assert(offsetof(BlockWriteRequest, lba) == 8);
static_assert(offsetof(BlockWriteRequest, block_count) == 16);
static_assert(offsetof(BlockWriteRequest, reserved) == 20);
static_assert(std::has_unique_object_representations_v<BlockWriteRequest>);
static_assert(static_cast<int64_t>(sizeof(BlockWriteRequest)) +
                  kBlockMaxIoBytes <=
              kIpcMaxMessageSize);

// `kFlush`: flushes volatile write cache for device `disk_index`. Replies with
// 0 bytes and status 0 on success.
struct BlockFlushRequest {
  BlockRpcOp op = BlockRpcOp::kFlush;
  uint32_t disk_index = 0;
};
static_assert(sizeof(BlockFlushRequest) == 8);
static_assert(alignof(BlockFlushRequest) == 4);
static_assert(offsetof(BlockFlushRequest, op) == 0);
static_assert(offsetof(BlockFlushRequest, disk_index) == 4);
static_assert(std::has_unique_object_representations_v<BlockFlushRequest>);

}  // namespace protos

#endif  // PROTOS_BLOCK_PROTOCOL_H_
