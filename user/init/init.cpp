// init: the first user process, which the kernel starts once it has booted
// (kernel.cpp). It greets from user mode, optionally starts user-space hardware
// drivers when `kInitArgStartDrivers` is set in `arg`, spawns the interactive
// shell (sh), waits for the shell to finish, and exits.

#include <cstdint>

#include "syscall_abi.h"
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
  if ((arg & kInitArgStartDrivers) != 0) {
    // Release any xHCI SMM USB Legacy Support (including Port 60h/64h
    // emulation SMIs) and halt firmware-running xHCI controllers before
    // `ps2kbd` touches I/O ports 0x60/0x64. If this helper faults, is killed
    // or reports an error, the drivers and the shell start anyway: a keyboard
    // may then not work, but the serial console does.
    const int64_t handoff_pid = SpawnDriver("usbkbd", kDriverArgHandoffOnly);
    if (handoff_pid < 0) {
      return 7;
    }
    const StatusOr<UserProgramResult> handoff_result = Wait(handoff_pid);
    if (!handoff_result.ok() ||
        handoff_result->outcome != UserProgramOutcome::kExited ||
        handoff_result->exit_status != 0) {
      (void)Print("[INIT] usbkbd handoff did not complete; continuing\n");
    }
    if (SpawnDriver("usbkbd", kDriverArgNormal) < 0) {
      return 7;
    }
    if (SpawnDriver("ps2kbd", kDriverArgNormal) < 0) {
      return 8;
    }
  }
  const int64_t sh_pid = Spawn("sh", arg & ~kInitArgStartDrivers);
  if (sh_pid < 0) {
    return 3;
  }
  const StatusOr<UserProgramResult> sh_result = Wait(sh_pid);
  if (!sh_result.ok()) {
    return 4;
  }
  if (sh_result->outcome != UserProgramOutcome::kExited) {
    return 5;
  }
  if (sh_result->exit_status != 0) {
    return 6;
  }
  return 0;
}

}  // namespace protos::user
