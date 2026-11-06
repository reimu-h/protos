// ramfs: ring-3 in-memory (`tmpfs`-style) filesystem server on `kIpcPortFs`.
//
// Claims `kIpcPortFs`, allocates 4 KiB blocks inside its own anonymous user
// address space via `MemMap()`, and blocks indefinitely in `IpcRecv(..., -1)`
// between filesystem RPCs and client-disconnect notifications. Started without
// `kUserCapDriver` (for example `run ramfs` in `sh`), reports that and exits
// with `kDriverExitNotDriver`.

#include "user/ramfs/ramfs.h"

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "syscall_abi.h"
#include "user/syscall.h"

namespace protos::user {
namespace {

StatusOr<uintptr_t> MapAnonymousPages(const int64_t size, void* const /*ctx*/) {
  return MemMap(size);
}

constinit ramfs::SlabBlockAllocator g_alloc{&MapAnonymousPages};
constinit ramfs::FileSystem g_fs{};
alignas(8) constinit uint8_t g_req_buf[kIpcMaxMessageSize]{};
alignas(8) constinit uint8_t g_reply_buf[kIpcMaxMessageSize]{};

static_assert(std::is_trivially_destructible_v<decltype(g_alloc)>);
static_assert(std::is_trivially_destructible_v<decltype(g_fs)>);
static_assert(std::is_trivially_destructible_v<decltype(g_req_buf)>);
static_assert(std::is_trivially_destructible_v<decltype(g_reply_buf)>);

}  // namespace

int64_t Main(const uint64_t arg) {
  const int64_t listen_rc = IpcListen(kIpcPortFs);
  if (listen_rc == kSyscallErrorPermission) {
    (void)Print(
        "[RAMFS] not started as a driver (init starts ramfs at boot)\n");
    return kDriverExitNotDriver;
  }
  if (listen_rc != 0) {
    (void)Print("[RAMFS] error: listen\n");
    return 1;
  }

  g_fs.Initialize(&g_alloc);
  (void)Print("[RAMFS] ready\n");

  if (arg == kDriverArgExitAfterReady || arg == kDriverArgFaultAfterReady) {
    uint8_t* const blk = g_alloc.AllocBlock();
    if (blk == nullptr) {
      (void)Print("[RAMFS] error: alloc\n");
      return 1;
    }
    g_alloc.FreeBlock(blk);

    IpcReceivedMessage msg{};
    (void)IpcRecv(kIpcPortFs,         //
                  g_req_buf,          //
                  sizeof(g_req_buf),  //
                  &msg,               //
                  /*timeout_ticks=*/0);
    if (arg == kDriverArgFaultAfterReady) {
      asm volatile("ud2");
    }
    return 0;
  }

  for (;;) {
    IpcReceivedMessage msg{};
    const int64_t rc = IpcRecv(kIpcPortFs,         //
                               g_req_buf,          //
                               sizeof(g_req_buf),  //
                               &msg,               //
                               /*timeout_ticks=*/-1);
    if (rc == kSyscallErrorAccess || rc == kSyscallErrorInvalid) {
      // Every later IpcRecv() would fail the same way.
      return 1;
    }
    if (rc <= 0) {
      // No message, or another error: receive again.
      continue;
    }
    if (msg.kind == IpcMessageKind::kClientDisconnect) {
      g_fs.CloseAllForClient(msg.client_id);
      continue;
    }
    if (msg.kind == IpcMessageKind::kCall) {
      // The longest reply that both the client and `g_reply_buf` take.
      const int64_t buf_cap = static_cast<int64_t>(sizeof(g_reply_buf));
      const int64_t reply_cap =
          msg.reply_cap < buf_cap ? msg.reply_cap : buf_cap;
      int64_t reply_size = 0;
      const int64_t status = ramfs::DispatchRpc(g_fs,           //
                                                msg.client_id,  //
                                                g_req_buf,      //
                                                msg.size,       //
                                                g_reply_buf,    //
                                                reply_cap,      //
                                                &reply_size);
      const int64_t reply_rc =
          IpcReply(kIpcPortFs, msg.token, g_reply_buf, reply_size, status);
      if (reply_rc != 0) {
        // Fail the call rather than leave the client waiting for a reply.
        (void)IpcReply(kIpcPortFs, msg.token, nullptr, 0, kSyscallErrorIo);
      }
    }
  }
}

}  // namespace protos::user
