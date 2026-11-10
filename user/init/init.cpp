// init: the first user process, which the kernel starts once it has booted
// (kernel.cpp). It greets from user mode, starts the user-space hardware
// drivers (`usbkbd`, `ps2kbd`, and `nvme`) if `kInitArgStartDrivers` is set in
// `arg`, and the `ramfs` filesystem server if `kInitArgStartDrivers` or
// `kInitArgStartFs` is set (with `kRamfsArgEmpty` if `kInitArgEmptyFs` is set,
// so that the filesystem starts empty instead of with the boot filesystem),
// waiting until ramfs serves requests and until nvme either serves block
// requests or exits (up to `kNvmeReadyTimeoutTicks`). It then spawns the
// interactive shell (sh), waits for the shell to finish, and exits with 0 if
// the shell exited with 0. Otherwise it exits with:
//   1 if it does not run in ring 3;
//   2 if it can't print;
//   3 if it can't spawn sh;
//   4 if waiting for sh fails;
//   5 if sh faulted, was killed, or failed to load;
//   6 if sh exited with a status other than 0;
//   7 if it can't spawn usbkbd (for the handoff or as the driver);
//   8 if it can't spawn ps2kbd;
//   9 if it can't spawn nvme.
// If ramfs can't be spawned or doesn't serve requests within
// `kRamfsReadyTimeoutTicks`, init says so and why, and starts the shell anyway
// (its filesystem commands fail while no ramfs runs).

#include <cstdint>

#include "block_protocol.h"
#include "syscall_abi.h"
#include "user/block.h"
#include "user/syscall.h"

namespace protos::user {

// How long init waits for ramfs to serve requests: far longer than ramfs
// takes to start, yet short enough that a boot whose ramfs doesn't come up
// still reaches the shell soon.
constexpr int64_t kRamfsReadyTimeoutTicks = 2 * kSyscallTicksPerSecond;

// How long init waits for nvme to claim its port before it starts `sh` without
// it. Once nvme has claimed the port, init waits until nvme has finished
// probing its controllers and entered its request loop (or exited), however
// long that takes, so `[NVME]` startup messages do not interleave with
// scripted shell output (see WaitForNvmeReady()).
constexpr int64_t kNvmeReadyTimeoutTicks = 2 * kSyscallTicksPerSecond;

// StartRamfs()'s result when ramfs doesn't serve requests in time.
constexpr int64_t kRamfsTimedOut = 1;

static int CurrentPrivilegeLevel() {
  uint16_t cs = 0;
  asm volatile("mov %0, cs" : "=r"(cs));
  return cs & 3;
}

// Spawns ramfs in `mode` (`kDriverArgNormal` or `kRamfsArgEmpty`) and waits
// until it answers a Stat() of the root, for at most kRamfsReadyTimeoutTicks.
// Returns 0 once ramfs is ready, SpawnDriver()'s error if ramfs can't be
// spawned, or `kRamfsTimedOut` if it isn't ready in time. A Stat() that ramfs
// has received but doesn't answer waits for it (Stat() has no timeout), as
// does one that comes while ramfs creates the files of the boot filesystem.
static int64_t StartRamfs(const uint64_t mode) {
  const int64_t pid = SpawnDriver("ramfs", mode);
  if (pid < 0) {
    return pid;
  }
  const int64_t deadline = GetTicks() + kRamfsReadyTimeoutTicks;
  FsStat root_stat{};
  // Stat() fails with kSyscallErrorNoDevice until ramfs claims its port.
  while (Stat("/", &root_stat) != 0) {
    if (GetTicks() >= deadline) {
      return kRamfsTimedOut;
    }
    // A sleep of about a timer tick (4 ms), for want of a sleep system call:
    // a Spin() of no bytes for one tick busy-waits in the kernel, where the
    // scheduler may switch init out, until a timer interrupt interrupts it.
    (void)SpinOnBuffer(nullptr, 0, 1);
  }
  return 0;
}

// Waits until `nvme` (`nvme_pid`) either answers `GetBlockDeviceInfo(0)` (after
// printing its `[NVME]` readiness lines) or exits. Gives up after
// `kNvmeReadyTimeoutTicks` only while nvme has not claimed its port: nvme
// claims it before it brings its controllers up, and a GetBlockDeviceInfo()
// that reaches the port waits for nvme's answer (it has no timeout, like
// StartRamfs()'s Stat()). nvme answers once it has brought every controller
// up or given up on it, which its own timeouts bound: a controller's CAP.TO
// (at least 2 s) for each reset and enable, and 5 s for each command.
static void WaitForNvmeReady(const int64_t nvme_pid) {
  const int64_t deadline = GetTicks() + kNvmeReadyTimeoutTicks;
  for (;;) {
    UserProgramResult nvme_result{};
    if (TryWait(nvme_pid, &nvme_result) != 0) {
      return;
    }
    int64_t disk_count = 0;
    if (GetBlockDeviceInfo(0, &disk_count).ok()) {
      return;
    }
    if (GetTicks() >= deadline) {
      if (TryWait(nvme_pid, &nvme_result) == 0) {
        (void)Print("[INIT] nvme not ready (timed out); continuing\n");
      }
      return;
    }
    (void)SpinOnBuffer(nullptr, 0, 1);
  }
}

// Says why init continues without ramfs, for StartRamfs()'s result `rc`, in
// one line: "[INIT] ramfs not ready (timed out); continuing without a
// filesystem", or "(spawn error -1)" and so on for a SpawnDriver() error.
static void ReportRamfsNotReady(const int64_t rc) {
  constexpr int kLineCapacity = 96;
  char line[kLineCapacity] = {};
  int length = 0;
  const auto append = [&line, &length](const char* const text) {
    for (int i = 0; text[i] != '\0' && length < kLineCapacity - 1; ++i) {
      line[length++] = text[i];
    }
  };
  append("[INIT] ramfs not ready (");
  if (rc == kRamfsTimedOut) {
    append("timed out");
  } else {
    append("spawn error -");
    // The digits of the error's magnitude, from the end of `digits`.
    char digits[21] = {};
    int pos = 20;
    uint64_t magnitude = 0 - static_cast<uint64_t>(rc);
    do {
      digits[--pos] = static_cast<char>('0' + magnitude % 10);
      magnitude /= 10;
    } while (magnitude != 0);
    append(&digits[pos]);
  }
  append("); continuing without a filesystem\n");
  (void)Print(line);
}

int64_t Main(const uint64_t arg) {
  if (CurrentPrivilegeLevel() != 3) {
    return 1;
  }
  if (Print("[INIT] hello from ring 3\n") < 0) {
    return 2;
  }
  int64_t nvme_pid = -1;
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
    nvme_pid = SpawnDriver("nvme", kDriverArgNormal);
    if (nvme_pid < 0) {
      return 9;
    }
  }
  // Like a keyboard, the filesystem is optional: without it, the shell still
  // runs its other commands.
  const uint64_t ramfs_mode =
      (arg & kInitArgEmptyFs) != 0 ? kRamfsArgEmpty : kDriverArgNormal;
  if ((arg & (kInitArgStartDrivers | kInitArgStartFs)) != 0) {
    const int64_t ramfs_rc = StartRamfs(ramfs_mode);
    if (ramfs_rc != 0) {
      ReportRamfsNotReady(ramfs_rc);
    }
  }
  if (nvme_pid >= 0) {
    WaitForNvmeReady(nvme_pid);
  }
  const int64_t sh_pid = Spawn(
      "sh", arg & ~(kInitArgStartDrivers | kInitArgStartFs | kInitArgEmptyFs));
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
