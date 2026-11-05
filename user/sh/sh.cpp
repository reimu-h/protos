// sh: the interactive ring-3 user-space shell spawned by init
// (user/init/init.cpp).

#include <cstdint>

#include "user/sh/sh_lib.h"
#include "user/syscall.h"

namespace protos::user {

static int CurrentPrivilegeLevel() {
  uint16_t cs = 0;
  asm volatile("mov %0, cs" : "=r"(cs));
  return cs & 3;
}

static int64_t ShellSysRead(void* const /*ctx*/,  //
                            char* const buffer,   //
                            const int64_t size) {
  return Read(buffer, size);
}

static int64_t ShellSysWrite(void* const /*ctx*/,       //
                             const char* const buffer,  //
                             const int64_t size) {
  return Write(buffer, size);
}

static int ShellSysGetCpu(void* const /*ctx*/) { return GetCpu(); }

static int64_t ShellSysSpawn(void* const /*ctx*/,     //
                             const char* const name,  //
                             const int64_t name_len,  //
                             const uint64_t arg) {
  return Spawn(name, name_len, arg);
}

static StatusOr<UserProgramResult> ShellSysWait(void* const /*ctx*/,
                                                const int64_t pid) {
  return Wait(pid);
}

static int64_t ShellSysOpen(void* const /*ctx*/,     //
                            const char* const path,  //
                            const int64_t path_len,  //
                            const uint64_t flags) {
  return Open(path, path_len, flags);
}

static int64_t ShellSysClose(void* const /*ctx*/, const int64_t fd) {
  return Close(fd);
}

static int64_t ShellSysFileRead(void* const /*ctx*/,  //
                                const int64_t fd,     //
                                void* const buffer,   //
                                const int64_t size) {
  return FileRead(fd, buffer, size);
}

static int64_t ShellSysStat(void* const /*ctx*/,     //
                            const char* const path,  //
                            const int64_t path_len,  //
                            FsStat* const out_stat) {
  return Stat(path, path_len, out_stat);
}

static int64_t ShellSysReadDir(void* const /*ctx*/,            //
                               const int64_t fd,               //
                               FsDirEntry* const out_entries,  //
                               const int64_t max_entries) {
  return ReadDir(fd, out_entries, max_entries);
}

static int64_t ShellSysMkDir(void* const /*ctx*/,     //
                             const char* const path,  //
                             const int64_t path_len) {
  return MkDir(path, path_len);
}

static int64_t ShellSysUnlink(void* const /*ctx*/,     //
                              const char* const path,  //
                              const int64_t path_len) {
  return Unlink(path, path_len);
}

int64_t Main(const uint64_t /*arg*/) {
  if (CurrentPrivilegeLevel() != 3) {
    return 1;
  }
  const sh::ShellEnv env{
      .ctx = nullptr,
      .read = ShellSysRead,
      .write = ShellSysWrite,
      .get_cpu = ShellSysGetCpu,
      .spawn = ShellSysSpawn,
      .wait = ShellSysWait,
      .open = ShellSysOpen,
      .close = ShellSysClose,
      .file_read = ShellSysFileRead,
      .stat = ShellSysStat,
      .read_dir = ShellSysReadDir,
      .mkdir = ShellSysMkDir,
      .unlink = ShellSysUnlink,
  };
  return sh::RunShell(env);
}

}  // namespace protos::user
