// init: the first user process, which the kernel starts once it has booted
// (kernel.cpp). It greets from user mode and exits.

#include <cstdint>

#include "user/syscall.h"

namespace protos::user {

static int CurrentPrivilegeLevel() {
  uint16_t cs = 0;
  asm volatile("mov %0, cs" : "=r"(cs));
  return cs & 3;
}

int64_t Main(const uint64_t /*arg*/) {
  if (CurrentPrivilegeLevel() != 3) {
    return 1;
  }
  if (Print("[INIT] hello from ring 3\n") < 0) {
    return 2;
  }
  return 0;
}

}  // namespace protos::user

