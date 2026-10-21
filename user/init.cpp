// init: the first user process, which the kernel starts once it has booted
// (kernel.cpp). It greets from user mode, spawns the interactive shell (sh),
// waits for the shell to finish, and exits.

#include <cstdint>

#include "user/syscall.h"

namespace protos::user {

static int CurrentPrivilegeLevel() {
  uint16_t cs = 0;
  asm volatile("mov %0, cs" : "=r"(cs));
  return cs & 3;
}

int64_t Main(const uint64_t arg) {
  if (CurrentPrivilegeLevel() != 3) {
    return 1;
  }
  if (Print("[INIT] hello from ring 3\n") < 0) {
    return 2;
  }
  const int64_t sh_pid = Spawn("sh", arg);
  if (sh_pid < 0) {
    return 3;
  }
  UserProgramResult sh_result;
  if (Wait(sh_pid, &sh_result) != 0) {
    return 4;
  }
  if (sh_result.outcome != UserProgramOutcome::kExited) {
    return 5;
  }
  if (sh_result.exit_status != 0) {
    return 6;
  }
  return 0;
}

}  // namespace protos::user
