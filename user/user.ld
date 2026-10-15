/* Linker script of the user programs (user/). A program runs in user space,
 * the upper half of the address space (paging.h). It links at the start of
 * the top 2 GiB, which the kernel code model (-mcmodel=kernel) addresses with
 * sign-extended 32-bit values, above the user stack (kUserStackTop in
 * user_mode.h). Code, read-only data and data each get a segment of their own
 * that starts on a page boundary, so that the kernel can map their pages
 * executable, read-only and writable (elf.h). The file and program headers
 * are not loaded. */

OUTPUT_FORMAT(elf64-x86-64)
OUTPUT_ARCH(i386:x86-64)
ENTRY(_start)

PHDRS
{
    text PT_LOAD FLAGS(5);      /* PF_R | PF_X */
    rodata PT_LOAD FLAGS(4);    /* PF_R */
    data PT_LOAD FLAGS(6);      /* PF_R | PF_W */
}

SECTIONS
{
    . = 0xFFFFFFFF80000000;

    .text : { *(.text .text.*) } :text

    . = ALIGN(4K);
    .rodata : { *(.rodata .rodata.*) } :rodata

    . = ALIGN(4K);
    .data : { *(.data .data.*) } :data
    .bss : { *(.bss .bss.*) *(COMMON) } :data

    /DISCARD/ :
    {
        *(.comment)
        *(.note*)
        *(.eh_frame*)
    }
}

