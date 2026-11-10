// disk: standalone block storage inspection and read/write utility
// (`/bin/disk`) talking directly to the ring-3 block driver over
// `kIpcPortBlock`.

#include <cstdint>

#include "user/block.h"
#include "user/disk/disk_lib.h"
#include "user/syscall.h"

namespace protos::user {
namespace {

alignas(8) uint8_t g_io_scratch[kBlockWriteScratchBytes];

static int64_t DiskSysWrite(void* const /*ctx*/,     //
                            const char* const data,  //
                            const int64_t size) {
  return Write(data, size);
}

}  // namespace

int64_t Main(const int argc, char** const argv) {
  const disk::DiskEnv env{
      .block = SyscallBlockClientEnv(),
      .write_ctx = nullptr,
      .write = &DiskSysWrite,
  };
  return disk::RunDisk(env,           //
                       argc,          //
                       argv,          //
                       g_io_scratch,  //
                       static_cast<int64_t>(sizeof(g_io_scratch)));
}

}  // namespace protos::user
