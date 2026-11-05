// init: the first user process, which the kernel starts once it has booted
// (kernel.cpp). It greets from user mode, starts the user-space hardware
// drivers (`usbkbd` and `ps2kbd`) if `kInitArgStartDrivers` is set in `arg`,
// and the `ramfs` filesystem server if `kInitArgStartDrivers` or
// `kInitArgStartFs` is set, waiting until ramfs serves requests. It then
// spawns the interactive shell (sh), waits for the shell to finish, and exits
// with 0 if the shell exited with 0. Otherwise it exits with:
//   1 if it does not run in ring 3;
//   2 if it can't print;
//   3 if it can't spawn sh;
//   4 if waiting for sh fails;
//   5 if sh faulted or was killed;
//   6 if sh exited with a status other than 0;
//   7 if it can't spawn usbkbd (for the handoff or as the driver);
//   8 if it can't spawn ps2kbd.
// If ramfs can't be spawned or doesn't serve requests within
// `kRamfsReadyTimeoutTicks`, init says so and starts the shell anyway (its
// filesystem commands fail while no ramfs runs).

#include <cstdint>

#include "syscall_abi.h"
#include "user/syscall.h"

namespace protos::user {

// How long init waits for ramfs to serve requests: far longer than ramfs
// takes to start, and no longer than the boot self-tests wait for a process
// to end (at least 2 s), so that they see what init does without ramfs.
constexpr int64_t kRamfsReadyTimeoutTicks = 2 * kSyscallTicksPerSecond;

static int CurrentPrivilegeLevel() {
  uint16_t cs = 0;
  asm volatile("mov %0, cs" : "=r"(cs));
  return cs & 3;
}

// Spawns ramfs and waits until it answers a Stat() of the root, for at most
// kRamfsReadyTimeoutTicks. Returns false if ramfs can't be spawned or isn't
// ready in time. A Stat() that ramfs has received but doesn't answer waits
// for it (Stat() has no timeout).
static bool StartRamfs() {
  if (SpawnDriver("ramfs", kDriverArgNormal) < 0) {
    return false;
  }
  const int64_t deadline = GetTicks() + kRamfsReadyTimeoutTicks;
  FsStat root_stat{};
  // Stat() fails with kSyscallErrorNoDevice until ramfs claims its port.
  while (Stat("/", &root_stat) != 0) {
    if (GetTicks() >= deadline) {
      return false;
    }
    // A sleep of about a timer tick (4 ms), for want of a sleep system call:
    // a Spin() of no bytes for one tick busy-waits in the kernel, where the
    // scheduler may switch init out, until a timer interrupt interrupts it.
    (void)SpinOnBuffer(nullptr, 0, 1);
  }
  return true;
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
  // Like a keyboard, the filesystem is optional: without it, the shell still
  // runs its other commands.
  if ((arg & (kInitArgStartDrivers | kInitArgStartFs)) != 0 && !StartRamfs()) {
    (void)Print("[INIT] ramfs not ready; continuing without a filesystem\n");
  }
  const int64_t sh_pid =
      Spawn("sh", arg & ~(kInitArgStartDrivers | kInitArgStartFs));
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
