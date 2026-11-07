// ramfs: ring-3 in-memory (`tmpfs`-style) filesystem server on `kIpcPortFs`.
//
// Claims `kIpcPortFs`, then, with `kDriverArgNormal`, creates the directories
// and regular files of the boot filesystem, the ustar archive that the boot
// loader loaded as the boot module `kRootfsBootModuleName` (if it did), before
// it serves the first request, so that requests that come meanwhile wait for
// them (`kRamfsArgEmpty` starts empty instead). Allocates 4 KiB blocks inside
// its own anonymous user address space via `MemMap()`, and blocks indefinitely
// in `IpcRecv(..., -1)` between filesystem RPCs and client-disconnect
// notifications. Started without `kUserCapDriver` (for example `run ramfs` in
// `sh`), reports that and exits with `kDriverExitNotDriver`.

#include "user/ramfs/ramfs.h"

#include <cstdint>
#include <type_traits>

#include "syscall_abi.h"
#include "user/ramfs/boot_module_source.h"
#include "user/ramfs/rootfs.h"
#include "user/ramfs/tar.h"
#include "user/syscall.h"

namespace protos::user {
namespace {

static StatusOr<uintptr_t> MapAnonymousPages(const int64_t size,
                                             void* const /*ctx*/) {
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

// A line for the console, cut to fit its buffer.
class LogLine {
 public:
  LogLine& Str(const char* const text) {
    for (int64_t i = 0; text[i] != '\0' && length_ < kCapacity; ++i) {
      text_[length_++] = text[i];
    }
    return *this;
  }

  LogLine& Dec(const int64_t value) {
    if (value < 0) {
      Str("-");
    }
    uint64_t magnitude = (value < 0) ? 0 - static_cast<uint64_t>(value)
                                     : static_cast<uint64_t>(value);
    char digits[20];
    int count = 0;
    do {
      digits[count++] = static_cast<char>('0' + magnitude % 10);
      magnitude /= 10;
    } while (magnitude != 0);
    while (count > 0 && length_ < kCapacity) {
      text_[length_++] = digits[--count];
    }
    return *this;
  }

  void Print() const { (void)Write(text_, length_); }

 private:
  static constexpr int64_t kCapacity = 96;
  char text_[kCapacity] = {};
  int64_t length_ = 0;
};

// Creates the directories and regular files of the boot filesystem in `g_fs`
// and reports what it did: the files and directories that it created, the
// archive members that it skipped (links and devices, for example) and those
// that it could not create, and where the archive is damaged, if it is.
static void LoadRootfs() {
  const int64_t index = FindBootModule(kRootfsBootModuleName);
  if (index == kSyscallErrorNoEntry) {
    (void)Print("[RAMFS] no rootfs boot module\n");
    return;
  }
  if (index < 0) {
    LogLine().Str("[RAMFS] rootfs: error ").Dec(index).Str("\n").Print();
    return;
  }
  BootModuleSource source(index);
  tar::Reader reader(&source);
  ramfs::LoadStats stats;
  const int64_t rc = ramfs::LoadArchive(g_fs, reader, &stats);
  LogLine line;
  line.Str("[RAMFS] rootfs: files=")
      .Dec(stats.files)
      .Str(" dirs=")
      .Dec(stats.directories)
      .Str(" skipped=")
      .Dec(stats.skipped)
      .Str(" errors=")
      .Dec(stats.failed);
  if (rc < 0) {
    line.Str(" damaged at byte ").Dec(reader.header_offset());
  }
  line.Str("\n").Print();
}

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
  if (arg == kDriverArgNormal) {
    LoadRootfs();
  }
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

  const ramfs::ServeEnv serve_env{
      .ctx = nullptr,
      .recv =
          [](void* const /*ctx*/,            //
             const int64_t port,             //
             void* const buf,                //
             const int64_t cap,              //
             IpcReceivedMessage* const msg,  //
             const int64_t timeout_ticks) {
            return IpcRecv(port, buf, cap, msg, timeout_ticks);
          },
      .reply =
          [](void* const /*ctx*/,    //
             const int64_t port,     //
             const uint64_t token,   //
             const void* const buf,  //
             const int64_t size,     //
             const int64_t status) {
            return IpcReply(port, token, buf, size, status);
          },
  };
  while (ramfs::ServeOne(g_fs,               //
                         serve_env,          //
                         g_req_buf,          //
                         sizeof(g_req_buf),  //
                         g_reply_buf,        //
                         sizeof(g_reply_buf))) {
  }
  return 1;
}

}  // namespace protos::user
