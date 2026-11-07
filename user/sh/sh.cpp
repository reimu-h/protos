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

static int64_t ShellSysSpawnFile(void* const /*ctx*/,                //
                                 const char* const path,             //
                                 const int64_t path_len,             //
                                 const SyscallArgSlice* const args,  //
                                 const int64_t argc) {
  return SpawnFile(path, path_len, args, argc);
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

static int64_t ShellSysFileWrite(void* const /*ctx*/,       //
                                 const int64_t fd,          //
                                 const void* const buffer,  //
                                 const int64_t size) {
  return FileWrite(fd, buffer, size);
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

static int64_t ShellSysRename(void* const /*ctx*/,         //
                              const char* const old_path,  //
                              const int64_t old_len,       //
                              const char* const new_path,  //
                              const int64_t new_len) {
  return Rename(old_path, old_len, new_path, new_len);
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
      .spawn_file = ShellSysSpawnFile,
      .wait = ShellSysWait,
      .open = ShellSysOpen,
      .close = ShellSysClose,
      .file_read = ShellSysFileRead,
      .file_write = ShellSysFileWrite,
      .stat = ShellSysStat,
      .read_dir = ShellSysReadDir,
      .mkdir = ShellSysMkDir,
      .unlink = ShellSysUnlink,
      .rename = ShellSysRename,
  };
  return sh::RunShell(env);
}

}  // namespace protos::user
