#ifndef PROTOS_USER_BLOCK_H_
#define PROTOS_USER_BLOCK_H_

#include <cstddef>
#include <cstdint>

#include "block_protocol.h"
#include "status_or.h"
#include "syscall_abi.h"
#include "user/syscall.h"

// User-space block client wrappers over `kIpcPortBlock` (`block_protocol.h`).
// Parameterized over `BlockClientEnv` so host unit tests can exercise the exact
// request encoding and reply-validation logic without system calls.

namespace protos::user {

// Size in bytes of the message staging buffer needed by
// `WriteBlocksWithScratch()` for any transfer up to `kBlockMaxIoBytes` (`24 +
// 4096 = 4120`).
constexpr int64_t kBlockWriteScratchBytes =
    static_cast<int64_t>(sizeof(BlockWriteRequest)) + kBlockMaxIoBytes;
static_assert(kBlockWriteScratchBytes <= kIpcMaxMessageSize);

// Transport seam for block client helpers. Ring-3 wrappers default to
// `IpcCall()` over `kIpcPortBlock`.
struct BlockClientEnv {
  void* ctx = nullptr;
  int64_t (*ipc_call)(void* ctx,            //
                      int64_t port,         //
                      const void* req_buf,  //
                      int64_t req_size,     //
                      void* reply_buf,      //
                      int64_t reply_cap,    //
                      int64_t* out_reply_size) = nullptr;
};

// Returns true if `block_size` is a valid power-of-two block size in
// `[kBlockMinBlockSize, kBlockMaxIoBytes]` (`512`, `1024`, `2048`, or `4096`).
constexpr bool IsValidBlockSize(const int32_t block_size) {
  return block_size >= kBlockMinBlockSize && block_size <= kBlockMaxIoBytes &&
         (block_size & (block_size - 1)) == 0;
}

// Returns true if `[buf, buf + cap)` contains at least one `'\0'` byte.
constexpr bool IsNulTerminated(const char* const buf, const int cap) {
  if (buf == nullptr || cap <= 0) {
    return false;
  }
  for (int i = 0; i < cap; ++i) {
    if (buf[i] == '\0') {
      return true;
    }
  }
  return false;
}

// Returns true if `info` satisfies the `block_protocol.h` geometry/flag
// invariants and has NUL-terminated `model` and `serial` arrays (servers
// emit printable ASCII padded with NUL bytes; clients only check NUL
// termination).
constexpr bool IsValidBlockDeviceInfo(const BlockDeviceInfo& info) {
  if (info.block_count < 1 || !IsValidBlockSize(info.block_size) ||
      info.max_transfer_blocks < 1 ||
      info.max_transfer_blocks > kBlockMaxIoBytes / info.block_size ||
      info.read_only > 1) {
    return false;
  }
  return IsNulTerminated(info.model, kBlockModelCapacity) &&
         IsNulTerminated(info.serial, kBlockSerialCapacity);
}

inline int64_t SyscallBlockIpcCall(void* const /*ctx*/,        //
                                   const int64_t port,         //
                                   const void* const req_buf,  //
                                   const int64_t req_size,     //
                                   void* const reply_buf,      //
                                   const int64_t reply_cap,    //
                                   int64_t* const out_reply_size) {
  return IpcCall(port, req_buf, req_size, reply_buf, reply_cap, out_reply_size);
}

inline BlockClientEnv SyscallBlockClientEnv() {
  return BlockClientEnv{
      .ctx = nullptr,
      .ipc_call = &SyscallBlockIpcCall,
  };
}

// Returns metadata for block device `disk_index` and, if `out_disk_count` is
// non-null, writes the number of available block devices (`>= 1`) to
// `*out_disk_count` (leaving `*out_disk_count` untouched on failure). Validates
// that `disk_index < disk_count`, the reply is exact `sizeof(BlockDeviceInfo)`
// bytes, and `IsValidBlockDeviceInfo(info)` holds (`kSyscallErrorIo` on
// malformed server replies).
inline StatusOr<BlockDeviceInfo> GetBlockDeviceInfo(
    const BlockClientEnv& env,  //
    const uint32_t disk_index,  //
    int64_t* const out_disk_count = nullptr) {
  if (env.ipc_call == nullptr) {
    return Error(kSyscallErrorInvalid);
  }
  const BlockGetInfoRequest req{
      .op = BlockRpcOp::kGetInfo,
      .disk_index = disk_index,
  };
  BlockDeviceInfo info{};
  int64_t reply_size = 0;
  const int64_t rc = env.ipc_call(env.ctx,        //
                                  kIpcPortBlock,  //
                                  &req,           //
                                  sizeof(req),    //
                                  &info,          //
                                  sizeof(info),   //
                                  &reply_size);
  if (rc < 0) {
    return Error(rc);
  }
  if (rc == 0 ||
      static_cast<uint64_t>(disk_index) >= static_cast<uint64_t>(rc) ||
      reply_size != static_cast<int64_t>(sizeof(BlockDeviceInfo)) ||
      !IsValidBlockDeviceInfo(info)) {
    return Error(kSyscallErrorIo);
  }
  if (out_disk_count != nullptr) {
    *out_disk_count = rc;
  }
  return info;
}

inline StatusOr<BlockDeviceInfo> GetBlockDeviceInfo(
    const uint32_t disk_index, int64_t* const out_disk_count = nullptr) {
  return GetBlockDeviceInfo(SyscallBlockClientEnv(),  //
                            disk_index,               //
                            out_disk_count);
}

// Reads `block_count` blocks from `disk_index` starting at `lba` into the
// `byte_cap` bytes at `buffer`. Returns `block_count` on success, or a negative
// `kSyscallError*`. When `block_count == 0`, passes a zero-byte reply window so
// caller buffers are never touched and requires `reply_size == 0`; when
// `block_count > 0`, verifies `reply_size` is positive, `<= byte_cap`, an exact
// multiple of `block_count`, and yields a valid `IsValidBlockSize()` block
// size.
inline int64_t ReadBlocks(const BlockClientEnv& env,  //
                          const uint32_t disk_index,  //
                          const uint64_t lba,         //
                          const int32_t block_count,  //
                          void* const buffer,         //
                          const int64_t byte_cap) {
  if (env.ipc_call == nullptr || block_count < 0 || byte_cap < 0 ||
      byte_cap > kBlockMaxIoBytes || (block_count > 0 && byte_cap == 0) ||
      (byte_cap > 0 && buffer == nullptr)) {
    return kSyscallErrorInvalid;
  }
  const BlockReadRequest req{
      .op = BlockRpcOp::kRead,
      .disk_index = disk_index,
      .lba = lba,
      .block_count = block_count,
      .reserved = 0,
  };
  void* const reply_buf = (block_count == 0) ? nullptr : buffer;
  const int64_t reply_cap = (block_count == 0) ? 0 : byte_cap;
  int64_t reply_size = 0;
  const int64_t rc = env.ipc_call(env.ctx,        //
                                  kIpcPortBlock,  //
                                  &req,           //
                                  sizeof(req),    //
                                  reply_buf,      //
                                  reply_cap,      //
                                  &reply_size);
  if (rc < 0) {
    return rc;
  }
  if (rc != block_count) {
    return kSyscallErrorIo;
  }
  if (block_count == 0) {
    return (reply_size == 0) ? 0 : kSyscallErrorIo;
  }
  if (reply_size <= 0 || reply_size > byte_cap ||
      (reply_size % block_count) != 0) {
    return kSyscallErrorIo;
  }
  const int64_t inferred_block_size = reply_size / block_count;
  if (inferred_block_size > kBlockMaxIoBytes ||
      !IsValidBlockSize(static_cast<int32_t>(inferred_block_size))) {
    return kSyscallErrorIo;
  }
  return rc;
}

inline int64_t ReadBlocks(const uint32_t disk_index,  //
                          const uint64_t lba,         //
                          const int32_t block_count,  //
                          void* const buffer,         //
                          const int64_t byte_cap) {
  return ReadBlocks(SyscallBlockClientEnv(),  //
                    disk_index,               //
                    lba,                      //
                    block_count,              //
                    buffer,                   //
                    byte_cap);
}

// Like `ReadBlocks()`, but additionally requires a valid `expected_block_size`
// up front, verifies `byte_cap >= int64_t{block_count} * expected_block_size`,
// and rejects replies whose byte count differs from that expected total.
inline int64_t ReadBlocksChecked(const BlockClientEnv& env,          //
                                 const uint32_t disk_index,          //
                                 const uint64_t lba,                 //
                                 const int32_t block_count,          //
                                 const int32_t expected_block_size,  //
                                 void* const buffer,                 //
                                 const int64_t byte_cap) {
  if (env.ipc_call == nullptr || !IsValidBlockSize(expected_block_size) ||
      block_count < 0 || byte_cap < 0 || byte_cap > kBlockMaxIoBytes ||
      (byte_cap > 0 && buffer == nullptr)) {
    return kSyscallErrorInvalid;
  }
  const int64_t expected_bytes =
      static_cast<int64_t>(block_count) * expected_block_size;
  if (expected_bytes > byte_cap) {
    return kSyscallErrorInvalid;
  }
  const BlockReadRequest req{
      .op = BlockRpcOp::kRead,
      .disk_index = disk_index,
      .lba = lba,
      .block_count = block_count,
      .reserved = 0,
  };
  void* const reply_buf = (block_count == 0) ? nullptr : buffer;
  const int64_t reply_cap = (block_count == 0) ? 0 : byte_cap;
  int64_t reply_size = 0;
  const int64_t rc = env.ipc_call(env.ctx,        //
                                  kIpcPortBlock,  //
                                  &req,           //
                                  sizeof(req),    //
                                  reply_buf,      //
                                  reply_cap,      //
                                  &reply_size);
  if (rc < 0) {
    return rc;
  }
  return (rc == block_count && reply_size == expected_bytes) ? rc
                                                             : kSyscallErrorIo;
}

inline int64_t ReadBlocksChecked(const uint32_t disk_index,          //
                                 const uint64_t lba,                 //
                                 const int32_t block_count,          //
                                 const int32_t expected_block_size,  //
                                 void* const buffer,                 //
                                 const int64_t byte_cap) {
  return ReadBlocksChecked(SyscallBlockClientEnv(),  //
                           disk_index,               //
                           lba,                      //
                           block_count,              //
                           expected_block_size,      //
                           buffer,                   //
                           byte_cap);
}

// Writes `block_count` blocks from the `byte_count` bytes at `data` to
// `disk_index` starting at `lba`, staging the RPC header and payload into
// `[scratch_buf, scratch_buf + scratch_cap)` instead of allocating a 4 KiB+
// stack buffer. Supports disjoint `data`/`scratch_buf`, zero-copy
// `data == static_cast<const uint8_t*>(scratch_buf) +
// sizeof(BlockWriteRequest)`, and overlapping buffers. Returns `block_count` on
// success or a negative `kSyscallError*`.
inline int64_t WriteBlocksWithScratch(const BlockClientEnv& env,  //
                                      const uint32_t disk_index,  //
                                      const uint64_t lba,         //
                                      const int32_t block_count,  //
                                      const void* const data,     //
                                      const int64_t byte_count,   //
                                      void* const scratch_buf,    //
                                      const int64_t scratch_cap) {
  if (env.ipc_call == nullptr || block_count < 0 || byte_count < 0 ||
      byte_count > kBlockMaxIoBytes || (block_count == 0 && byte_count != 0) ||
      (block_count > 0 && (byte_count == 0 || data == nullptr))) {
    return kSyscallErrorInvalid;
  }
  const int64_t msg_size =
      static_cast<int64_t>(sizeof(BlockWriteRequest)) + byte_count;
  if (scratch_buf == nullptr || scratch_cap < msg_size) {
    return kSyscallErrorInvalid;
  }
  const BlockWriteRequest req{
      .op = BlockRpcOp::kWrite,
      .disk_index = disk_index,
      .lba = lba,
      .block_count = block_count,
      .reserved = 0,
  };
  uint8_t* const msg = static_cast<uint8_t*>(scratch_buf);
  if (byte_count > 0) {
    uint8_t* const payload_dst = msg + sizeof(req);
    if (payload_dst != data) {
      __builtin_memmove(payload_dst, data, static_cast<size_t>(byte_count));
    }
  }
  __builtin_memcpy(msg, &req, sizeof(req));
  int64_t reply_size = 0;
  const int64_t rc = env.ipc_call(env.ctx,        //
                                  kIpcPortBlock,  //
                                  msg,            //
                                  msg_size,       //
                                  nullptr,        //
                                  0,              //
                                  &reply_size);
  if (rc < 0) {
    return rc;
  }
  return (rc == block_count && reply_size == 0) ? rc : kSyscallErrorIo;
}

inline int64_t WriteBlocksWithScratch(const uint32_t disk_index,  //
                                      const uint64_t lba,         //
                                      const int32_t block_count,  //
                                      const void* const data,     //
                                      const int64_t byte_count,   //
                                      void* const scratch_buf,    //
                                      const int64_t scratch_cap) {
  return WriteBlocksWithScratch(SyscallBlockClientEnv(),  //
                                disk_index,               //
                                lba,                      //
                                block_count,              //
                                data,                     //
                                byte_count,               //
                                scratch_buf,              //
                                scratch_cap);
}

// Writes `block_count` blocks from the `byte_count` bytes at `data` to
// `disk_index` starting at `lba` using a stack-allocated staging buffer (24
// bytes for 0-block no-ops, `kBlockWriteScratchBytes` for non-empty writes).
// Returns `block_count` on success or a negative `kSyscallError*`.
inline int64_t WriteBlocks(const BlockClientEnv& env,  //
                           const uint32_t disk_index,  //
                           const uint64_t lba,         //
                           const int32_t block_count,  //
                           const void* const data,     //
                           const int64_t byte_count) {
  if (env.ipc_call == nullptr || block_count < 0 || byte_count < 0 ||
      byte_count > kBlockMaxIoBytes || (block_count == 0 && byte_count != 0) ||
      (block_count > 0 && (byte_count == 0 || data == nullptr))) {
    return kSyscallErrorInvalid;
  }
  if (block_count == 0) {
    uint8_t msg[sizeof(BlockWriteRequest)];
    return WriteBlocksWithScratch(env,         //
                                  disk_index,  //
                                  lba,         //
                                  0,           //
                                  nullptr,     //
                                  0,           //
                                  msg,         //
                                  sizeof(msg));
  }
  uint8_t msg[kBlockWriteScratchBytes];
  return WriteBlocksWithScratch(env,          //
                                disk_index,   //
                                lba,          //
                                block_count,  //
                                data,         //
                                byte_count,   //
                                msg,          //
                                sizeof(msg));
}

inline int64_t WriteBlocks(const uint32_t disk_index,  //
                           const uint64_t lba,         //
                           const int32_t block_count,  //
                           const void* const data,     //
                           const int64_t byte_count) {
  return WriteBlocks(SyscallBlockClientEnv(),  //
                     disk_index,               //
                     lba,                      //
                     block_count,              //
                     data,                     //
                     byte_count);
}

// Flushes the volatile write cache of `disk_index`. Returns 0 on success or a
// negative `kSyscallError*` (`kSyscallErrorIo` if the server returns an
// unexpected positive status or non-empty reply).
inline int64_t FlushDisk(const BlockClientEnv& env, const uint32_t disk_index) {
  if (env.ipc_call == nullptr) {
    return kSyscallErrorInvalid;
  }
  const BlockFlushRequest req{
      .op = BlockRpcOp::kFlush,
      .disk_index = disk_index,
  };
  int64_t reply_size = 0;
  const int64_t rc = env.ipc_call(env.ctx,        //
                                  kIpcPortBlock,  //
                                  &req,           //
                                  sizeof(req),    //
                                  nullptr,        //
                                  0,              //
                                  &reply_size);
  if (rc < 0) {
    return rc;
  }
  return (rc == 0 && reply_size == 0) ? 0 : kSyscallErrorIo;
}

inline int64_t FlushDisk(const uint32_t disk_index) {
  return FlushDisk(SyscallBlockClientEnv(), disk_index);
}

}  // namespace protos::user

#endif  // PROTOS_USER_BLOCK_H_
