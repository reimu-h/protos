#ifndef PROTOS_FS_PROTOCOL_H_
#define PROTOS_FS_PROTOCOL_H_

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "syscall_abi.h"

namespace protos {

// Wire protocol between the kernel's filesystem system calls (`user_mode.cpp`)
// and the user-space filesystem server (`user/ramfs/`) over `kIpcPortFs`. Only
// the kernel sends these requests (user processes can't call `kIpcPortFs`),
// tagging each with the calling process's client id, but the server checks
// every field anyway. Every request begins with a 32-bit `FsRpcOp` opcode,
// every header has no implicit padding
// (`std::has_unique_object_representations_v`), and `reserved` fields must be
// zero. On failure, the server answers with a negative `kSyscallError*` status
// and no reply bytes.

enum class FsRpcOp : uint32_t {
  kNone = 0,
  kOpen = 1,
  kClose = 2,
  kRead = 3,
  kWrite = 4,
  kSeek = 5,
  kStat = 6,
  kReadDir = 7,
  kMkDir = 8,
  kUnlink = 9,
  kRename = 10,
};

// `kOpen`: followed immediately by `path_len` (`1 <= path_len <=
// kFsMaxPathLength`) path bytes. Replies with `FsOpenReply` and status 0.
struct FsOpenRequest {
  FsRpcOp op = FsRpcOp::kOpen;
  int32_t path_len = 0;
  uint64_t flags = 0;
};
static_assert(sizeof(FsOpenRequest) == 16);
static_assert(alignof(FsOpenRequest) == 8);
static_assert(offsetof(FsOpenRequest, op) == 0);
static_assert(offsetof(FsOpenRequest, path_len) == 4);
static_assert(offsetof(FsOpenRequest, flags) == 8);
static_assert(std::has_unique_object_representations_v<FsOpenRequest>);
static_assert(static_cast<int64_t>(sizeof(FsOpenRequest)) + kFsMaxPathLength <=
              kIpcMaxMessageSize);

// The reply to `kOpen`: `handle` is a non-zero value, opaque to the kernel,
// that names the open file in the client's later requests, and `type` is the
// type of the opened file or directory.
struct FsOpenReply {
  uint64_t handle = 0;
  FsNodeType type = FsNodeType::kNone;
  uint8_t reserved[7] = {};
};
static_assert(sizeof(FsOpenReply) == 16);
static_assert(alignof(FsOpenReply) == 8);
static_assert(offsetof(FsOpenReply, handle) == 0);
static_assert(offsetof(FsOpenReply, type) == 8);
static_assert(offsetof(FsOpenReply, reserved) == 9);
static_assert(std::has_unique_object_representations_v<FsOpenReply>);

// `kClose`: closes `handle` owned by `msg.client_id`. Replies with 0 bytes and
// status 0.
struct FsCloseRequest {
  FsRpcOp op = FsRpcOp::kClose;
  uint32_t reserved = 0;
  uint64_t handle = 0;
};
static_assert(sizeof(FsCloseRequest) == 16);
static_assert(alignof(FsCloseRequest) == 8);
static_assert(std::has_unique_object_representations_v<FsCloseRequest>);

// `kRead`: reads up to `size` (`0 <= size <= kSyscallFsMaxIoSize`) bytes from
// `handle`. Replies with `n` raw data bytes and status `n` (`0 <= n <= size`).
struct FsReadRequest {
  FsRpcOp op = FsRpcOp::kRead;
  uint32_t reserved = 0;
  uint64_t handle = 0;
  int64_t size = 0;
};
static_assert(sizeof(FsReadRequest) == 24);
static_assert(alignof(FsReadRequest) == 8);
static_assert(std::has_unique_object_representations_v<FsReadRequest>);
static_assert(kSyscallFsMaxIoSize <= kIpcMaxMessageSize);

// `kWrite`: followed immediately by `size` (`0 <= size <= kSyscallFsMaxIoSize`)
// raw data bytes. Replies with 0 bytes and status `n` (`0 <= n <= size`, and
// `n > 0` if `size > 0`: a server that can write none of the bytes fails with
// `kSyscallErrorFileTooBig` or `kSyscallErrorNoSpace` instead).
struct FsWriteRequest {
  FsRpcOp op = FsRpcOp::kWrite;
  uint32_t reserved = 0;
  uint64_t handle = 0;
  int64_t size = 0;
};
static_assert(sizeof(FsWriteRequest) == 24);
static_assert(alignof(FsWriteRequest) == 8);
static_assert(std::has_unique_object_representations_v<FsWriteRequest>);
static_assert(static_cast<int64_t>(sizeof(FsWriteRequest)) +
                  kSyscallFsMaxIoSize <=
              kIpcMaxMessageSize);

// `kSeek`: repositions `handle` by `(offset, whence)`. Replies with 0 bytes and
// status equal to the resulting offset (`>= 0`): `offset` for `kFsSeekSet`,
// and at most `kFsMaxFileSize` for a regular file. A directory accepts only
// `(0, kFsSeekSet)`, which returns 0, and `(0, kFsSeekCur)`, which returns an
// opaque position (Seek() in syscall_abi.h).
struct FsSeekRequest {
  FsRpcOp op = FsRpcOp::kSeek;
  int32_t whence = 0;
  uint64_t handle = 0;
  int64_t offset = 0;
};
static_assert(sizeof(FsSeekRequest) == 24);
static_assert(alignof(FsSeekRequest) == 8);
static_assert(std::has_unique_object_representations_v<FsSeekRequest>);

// `kStat`, `kMkDir`, `kUnlink`: followed immediately by `path_len` (`1 <=
// path_len <= kFsMaxPathLength`) path bytes. `kStat` replies with `FsStat` and
// status 0; `kMkDir` and `kUnlink` reply with 0 bytes and status 0.
struct FsPathRequest {
  FsRpcOp op = FsRpcOp::kNone;
  int32_t path_len = 0;
};
static_assert(sizeof(FsPathRequest) == 8);
static_assert(alignof(FsPathRequest) == 4);
static_assert(std::has_unique_object_representations_v<FsPathRequest>);
static_assert(static_cast<int64_t>(sizeof(FsPathRequest)) + kFsMaxPathLength <=
              kIpcMaxMessageSize);

// `kReadDir`: reads up to `max_entries` (`0 <= max_entries <=
// kSyscallFsMaxDirEntries`) entries from directory `handle`. Replies with
// `count * sizeof(FsDirEntry)` bytes and status `count`.
struct FsReadDirRequest {
  FsRpcOp op = FsRpcOp::kReadDir;
  int32_t max_entries = 0;
  uint64_t handle = 0;
};
static_assert(sizeof(FsReadDirRequest) == 16);
static_assert(alignof(FsReadDirRequest) == 8);
static_assert(std::has_unique_object_representations_v<FsReadDirRequest>);
static_assert(kSyscallFsMaxDirEntries *
                  static_cast<int64_t>(sizeof(FsDirEntry)) <=
              kIpcMaxMessageSize);

// `kRename`: followed immediately by `old_len` (`1 <= old_len <=
// kFsMaxPathLength`) bytes of old path and `new_len` (`1 <= new_len <=
// kFsMaxPathLength`) bytes of new path. Replies with 0 bytes and status 0.
struct FsRenameRequest {
  FsRpcOp op = FsRpcOp::kRename;
  int32_t old_len = 0;
  int32_t new_len = 0;
  uint32_t reserved = 0;
};
static_assert(sizeof(FsRenameRequest) == 16);
static_assert(alignof(FsRenameRequest) == 4);
static_assert(std::has_unique_object_representations_v<FsRenameRequest>);
static_assert(static_cast<int64_t>(sizeof(FsRenameRequest)) +
                  2 * kFsMaxPathLength <=
              kIpcMaxMessageSize);

}  // namespace protos

#endif  // PROTOS_FS_PROTOCOL_H_
