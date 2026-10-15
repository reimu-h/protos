#ifndef PROTOS_USER_PROGRAMS_H_
#define PROTOS_USER_PROGRAMS_H_

#include <cstdint>

#include "user_mode.h"

// The user programs that the kernel image embeds (user_programs.S).
extern "C" const uint8_t g_init_elf_start[];
extern "C" const uint8_t g_init_elf_end[];
extern "C" const uint8_t g_usertest_elf_start[];
extern "C" const uint8_t g_usertest_elf_end[];

namespace protos {

// user/init.cpp: the first user process (kernel.cpp).
inline UserProgram InitProgram() {
  return UserProgram{g_init_elf_start, g_init_elf_end - g_init_elf_start};
}

// user/usertest.cpp: the user-mode boot self-tests' program (user/usertest.h).
inline UserProgram UserTestProgram() {
  return UserProgram{g_usertest_elf_start,
                     g_usertest_elf_end - g_usertest_elf_start};
}

}  // namespace protos

#endif  // PROTOS_USER_PROGRAMS_H_

