#ifndef PROTOS_USER_PROGRAMS_H_
#define PROTOS_USER_PROGRAMS_H_

#include <cstdint>

#include "user_mode.h"

// The user programs that the kernel image embeds (user_programs.S).
extern "C" const uint8_t g_init_elf_start[];
extern "C" const uint8_t g_init_elf_end[];
extern "C" const uint8_t g_sh_elf_start[];
extern "C" const uint8_t g_sh_elf_end[];
extern "C" const uint8_t g_usertest_elf_start[];
extern "C" const uint8_t g_usertest_elf_end[];
extern "C" const uint8_t g_usbkbd_elf_start[];
extern "C" const uint8_t g_usbkbd_elf_end[];

namespace protos {

// user/init.cpp: the first user process (kernel.cpp).
inline UserProgram InitProgram() {
  return UserProgram{g_init_elf_start, g_init_elf_end - g_init_elf_start};
}
inline UserProgram InitUserProgram() { return InitProgram(); }

// user/sh.cpp: the interactive user-space shell.
inline UserProgram ShProgram() {
  return UserProgram{g_sh_elf_start, g_sh_elf_end - g_sh_elf_start};
}
inline UserProgram ShUserProgram() { return ShProgram(); }
inline UserProgram ShellProgram() { return ShProgram(); }

// user/usertest.cpp: the user-mode boot self-tests' program (user/usertest.h).
inline UserProgram UserTestProgram() {
  return UserProgram{g_usertest_elf_start,
                     g_usertest_elf_end - g_usertest_elf_start};
}
inline UserProgram UsertestUserProgram() { return UserTestProgram(); }

// user/usbkbd.cpp: the ring-3 xHCI USB keyboard driver.
inline UserProgram UsbkbdProgram() {
  return UserProgram{g_usbkbd_elf_start, g_usbkbd_elf_end - g_usbkbd_elf_start};
}
inline UserProgram UsbKbdProgram() { return UsbkbdProgram(); }
inline UserProgram UsbkbdUserProgram() { return UsbkbdProgram(); }

// Returns true if `[name, name + name_len)` equals the null-terminated string
// `literal`.
constexpr bool UserProgramNameEquals(const char* const name,  //
                                     const int64_t name_len,  //
                                     const char* const literal) {
  if (name == nullptr || name_len <= 0 || literal == nullptr) {
    return false;
  }
  int64_t i = 0;
  while (i < name_len && literal[i] != '\0') {
    if (name[i] != literal[i]) {
      return false;
    }
    ++i;
  }
  return i == name_len && literal[i] == '\0';
}

// Looks up the embedded user program whose name is `[name, name + name_len)`.
// If found, writes the program descriptor to `*out` (when non-null) and
// returns true; otherwise returns false.
inline bool FindUserProgram(const char* const name,  //
                            const int64_t name_len,  //
                            UserProgram* const out) {
  UserProgram found{};
  if (UserProgramNameEquals(name, name_len, "init")) {
    found = InitProgram();
  } else if (UserProgramNameEquals(name, name_len, "sh")) {
    found = ShProgram();
  } else if (UserProgramNameEquals(name, name_len, "usertest")) {
    found = UserTestProgram();
  } else if (UserProgramNameEquals(name, name_len, "usbkbd")) {
    found = UsbkbdProgram();
  } else {
    return false;
  }
  if (out != nullptr) {
    *out = found;
  }
  return true;
}

}  // namespace protos

#endif  // PROTOS_USER_PROGRAMS_H_
