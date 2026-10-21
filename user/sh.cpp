// sh: the interactive ring-3 user-space shell spawned by init (user/init.cpp).

#include <cstdint>

#include "user/sh_lib.h"
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

static int64_t ShellSysWait(void* const /*ctx*/,  //
                            const int64_t pid,    //
                            UserProgramResult* const out) {
  return Wait(pid, out);
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
  };
  return sh::RunShell(env);
}

}  // namespace protos::user
