/* user_programs.S — the user programs (user/) in the kernel image, as ELF
 * executables between start and end symbols (user_programs.h). The Makefile
 * builds them into $(BUILD_DIR)/user/ and passes $(BUILD_DIR) to the
 * assembler as an include directory, where .incbin finds them. */

.section .rodata

.balign 16
.global g_init_elf_start
.global g_init_elf_end
g_init_elf_start:
    .incbin "user/init.elf"
g_init_elf_end:

.balign 16
.global g_sh_elf_start
.global g_sh_elf_end
g_sh_elf_start:
    .incbin "user/sh.elf"
g_sh_elf_end:

.balign 16
.global g_usertest_elf_start
.global g_usertest_elf_end
g_usertest_elf_start:
    .incbin "user/usertest.elf"
g_usertest_elf_end:

.section .note.GNU-stack,"",@progbits

