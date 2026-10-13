OUTPUT_FORMAT(elf64-x86-64)
OUTPUT_ARCH(i386:x86-64)
ENTRY(_start)

SECTIONS
{
    /* Load kernel at physical address 1 MiB (0x100000) */
    . = 1M;
    g_kernel_start = .;

    /* Ensure Multiboot headers appear at the very start of the binary (within first 8 KiB) */
    .text ALIGN(4K) : AT(ADDR(.text))
    {
        KEEP(*(.multiboot))
        *(.text .text.*)
    }

    .rodata ALIGN(4K) : AT(ADDR(.rodata))
    {
        *(.rodata .rodata.*)
    }

    /* C++ dynamic initializers. _start (boot.S) calls every entry, in order,
     * before kernel_main. */
    .init_array ALIGN(8) : AT(ADDR(.init_array))
    {
        g_init_array_start = .;
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*)))
        KEEP(*(.init_array))
        g_init_array_end = .;
    }

    .data ALIGN(4K) : AT(ADDR(.data))
    {
        *(.data .data.*)
    }

    .bss ALIGN(4K) : AT(ADDR(.bss))
    {
        g_bss_start = .;
        *(COMMON)
        *(.bss .bss.*)
        . = ALIGN(4K);
        g_bss_end = .;
    }

    . = ALIGN(4K);
    g_kernel_end = .;

    /* boot.S identity-maps only the first 64 MiB (kBootstrapIdentityMapSize in
     * paging.h) before kernel_main, and the whole image must be reachable
     * through that mapping. */
    ASSERT(g_kernel_end <= 0x4000000, "kernel image exceeds the 64 MiB bootstrap identity map")

    /DISCARD/ :
    {
        *(.comment)
        *(.note*)
        *(.eh_frame*)
    }
}
