/* boot.S — x86-64 Multiboot1/Multiboot2 Bootstrap Assembly (GNU as) */

.intel_syntax noprefix

/* CR0 value loaded by both the BSP and the APs when they enable paging: PG (31),
 * WP (16), NE (5), ET (4), MP (1) and PE (0). Loading a complete value instead
 * of OR-ing bits in clears CD/NW (caches disabled, which RESET sets and INIT
 * leaves unchanged), AM, TS and EM. */
.set KERNEL_CR0, ((1 << 31) | (1 << 16) | (1 << 5) | (1 << 4) | (1 << 1) | (1 << 0))

/* IA32_EFER MSR and its bits; NXE is reserved (WRMSR faults) unless CPUID
 * 0x80000001 reports EDX.NX. */
.set MSR_IA32_EFER, 0xC0000080
.set MSR_IA32_GS_BASE, 0xC0000101
.set EFER_LME, (1 << 8)
.set EFER_NXE, (1 << 11)
.set CPUID_80000001_EDX_NX, (1 << 20)
.set CPUID_80000001_EDX_LM, (1 << 29)

/* Vector bases of the legacy 8259 PICs after remapping (must match
 * kVectorLegacyPicBase in idt.h). */
.set PIC_MASTER_VECTOR_BASE, 0xE0
.set PIC_SLAVE_VECTOR_BASE, 0xE8

.section .multiboot, "a"
.align 8

/* =========================================================================
 * Multiboot2 Header (Specification version 2.0)
 * Must be 8-byte aligned and within first 32 KiB of file.
 * ========================================================================= */
multiboot2_header_start:
    .long 0xE85250D6                                      /* Magic number (Multiboot2) */
    .long 0                                               /* Architecture: 0 (i386 32-bit protected mode) */
    .long multiboot2_header_end - multiboot2_header_start /* Header length */
    .long -(0xE85250D6 + 0 + (multiboot2_header_end - multiboot2_header_start)) /* Checksum */

    /* Framebuffer tag (type = 5, flags = 1 [optional], size = 20)
     * Allows UEFI GRUB to hand off a GOP linear pixel framebuffer while
     * permitting BIOS GRUB (with gfxpayload=text) to stay in 0xB8000 text mode. */
    .align 8
    .word 5
    .word 1
    .long 20
    .long 0                                               /* Preferred width (0 = bootloader default) */
    .long 0                                               /* Preferred height (0 = bootloader default) */
    .long 0                                               /* Preferred depth (0 = bootloader default) */

    /* End tag (type = 0, flags = 0, size = 8) */
    .align 8
    .word 0
    .word 0
    .long 8
multiboot2_header_end:

/* =========================================================================
 * Multiboot1 Header (Specification version 1)
 * Must be 4-byte aligned and within first 8 KiB of file.
 * ========================================================================= */
.align 4
multiboot1_header_start:
    .set MB1_MAGIC,    0x1BADB002
    .set MB1_FLAGS,    0x00000003                         /* Align modules on 4KB + provide mem info */
    .set MB1_CHECKSUM, -(MB1_MAGIC + MB1_FLAGS)
    .long MB1_MAGIC
    .long MB1_FLAGS
    .long MB1_CHECKSUM
multiboot1_header_end:


/* =========================================================================
 * 32-Bit Bootstrap Code (.code32)
 * Executed immediately after GRUB hands off control in 32-bit flat mode.
 * ========================================================================= */
.section .text
.global _start
.type _start, @function

/* Writes \value to legacy 8259 PIC I/O port \port, then writes to the unused
 * POST diagnostic port 0x80 to give the PIC time to settle (as Linux does). */
.macro PIC_WRITE port, value
    mov al, \value
    out \port, al
    out 0x80, al
.endm

.code32
_start:
    /* Disable interrupts and clear direction flag */
    cli
    cld

    /* Preserve the Multiboot magic (eax) in ebp and the Multiboot info physical
     * address (ebx) in esi; CPUID below clobbers eax, ebx, ecx and edx. */
    mov ebp, eax
    mov esi, ebx

    /* Zero the entire .bss section (page tables, IDT, stack, and C++ globals) */
    .extern g_bss_start
    .extern g_bss_end
    xor eax, eax
    mov edi, offset g_bss_start
    mov ecx, offset g_bss_end
    sub ecx, edi
    shr ecx, 2
    rep stosd

    /* Multiboot leaves ESP undefined; the checks below need a stack. */
    mov esp, offset stack_top

    /* Long mode needs CPUID (EFLAGS.ID is writable), extended leaf 0x80000001,
     * and its EDX.LM bit. Without them, enabling long mode would fault before
     * any IDT exists and reset the machine, so report the problem and halt. */
    pushfd
    pop eax
    mov ecx, eax
    xor eax, (1 << 21)          /* Try to toggle EFLAGS.ID */
    push eax
    popfd
    pushfd
    pop eax
    push ecx                    /* Restore the original EFLAGS */
    popfd
    xor eax, ecx
    test eax, (1 << 21)
    jz .Lno_long_mode
    mov eax, 0x80000000
    cpuid
    cmp eax, 0x80000001
    jb .Lno_long_mode
    mov eax, 0x80000001
    cpuid
    test edx, CPUID_80000001_EDX_LM
    jz .Lno_long_mode
    mov edi, edx                /* Keep the feature bits for the EFER write */

    /* Reinitialize both 8259 PICs (ICW1-ICW4) so that their vectors no longer
     * overlap the CPU exceptions (the BIOS leaves the master at 0x08-0x0F),
     * then mask all lines. The kernel uses only the Local APIC; IdtDispatch
     * ignores the remapped range except for registered handlers. */
    PIC_WRITE 0x20, 0x11        /* ICW1 (master): edge triggered, cascade, ICW4 */
    PIC_WRITE 0xA0, 0x11        /* ICW1 (slave) */
    PIC_WRITE 0x21, PIC_MASTER_VECTOR_BASE /* ICW2: IRQ0-7 -> 0xE0-0xE7 */
    PIC_WRITE 0xA1, PIC_SLAVE_VECTOR_BASE  /* ICW2: IRQ8-15 -> 0xE8-0xEF */
    PIC_WRITE 0x21, 0x04        /* ICW3 (master): slave on IRQ2 */
    PIC_WRITE 0xA1, 0x02        /* ICW3 (slave): cascade identity 2 */
    PIC_WRITE 0x21, 0x01        /* ICW4: 8086 mode */
    PIC_WRITE 0xA1, 0x01
    PIC_WRITE 0x21, 0xFF        /* OCW1: mask all master lines */
    PIC_WRITE 0xA1, 0xFF        /* OCW1: mask all slave lines */

    /* Set up bootstrap identity paging using 2 MiB huge pages for first 64 MiB
     * PML4[0] -> PDPT
     * PDPT[0] -> PD
     * PD[0..31] -> 0x00000000 .. 0x03FFFFFF (2 MiB pages, Present + RW + PS)
     * Remaining physical RAM is identity-mapped dynamically once PMM initializes.
     */
    mov eax, offset pdpt_table
    or  eax, 0x03               /* Present (bit 0) | Read/Write (bit 1) */
    mov dword ptr [pml4_table], eax

    mov eax, offset pd_table
    or  eax, 0x03               /* Present (bit 0) | Read/Write (bit 1) */
    mov dword ptr [pdpt_table], eax

    mov ecx, 0
.Lmap_pd_loop:
    mov eax, ecx
    shl eax, 21                 /* eax = ecx * 2 MiB (0x200000) */
    or  eax, 0x83               /* Present (0x01) | RW (0x02) | Page Size 2MiB (0x80) */
    mov dword ptr [pd_table + ecx * 8], eax
    mov dword ptr [pd_table + ecx * 8 + 4], 0
    inc ecx
    cmp ecx, 32
    jne .Lmap_pd_loop

    /* Load CR3 with physical base address of PML4 */
    mov eax, offset pml4_table
    mov cr3, eax

    /* Enable PAE (bit 5), OSFXSR (bit 9), and OSXMMEXCPT (bit 10) in CR4 */
    mov eax, cr4
    or  eax, ((1 << 5) | (1 << 9) | (1 << 10))
    mov cr4, eax

    /* Enable long mode (EFER.LME) and, if the CPU supports it, no-execute
     * pages (EFER.NXE) */
    mov ecx, MSR_IA32_EFER
    rdmsr
    or  eax, EFER_LME
    test edi, CPUID_80000001_EDX_NX
    jz .Lwrite_efer
    or  eax, EFER_NXE
.Lwrite_efer:
    wrmsr

    /* Enable paging (see KERNEL_CR0) */
    mov eax, KERNEL_CR0
    mov cr0, eax

    /* Load 64-bit Global Descriptor Table (GDT) */
    lgdt [gdt64_pointer]

    /* Far jump to 64-bit Kernel Code Segment (selector 0x08) to enter 64-bit mode */
    jmp 0x08:long_mode_start

/* The CPU cannot run this kernel: print no_long_mode_message on the VGA text
 * screen and COM1, then halt. */
.Lno_long_mode:
    /* COM1: 38400 baud, 8N1, FIFOs enabled (as UartInit programs it) */
    mov dx, 0x3F9               /* Interrupt enable: none */
    xor al, al
    out dx, al
    mov dx, 0x3FB               /* Line control: divisor latch access */
    mov al, 0x80
    out dx, al
    mov dx, 0x3F8               /* Divisor low byte: 3 (38400 baud) */
    mov al, 0x03
    out dx, al
    mov dx, 0x3F9               /* Divisor high byte: 0 */
    xor al, al
    out dx, al
    mov dx, 0x3FB               /* Line control: 8 data bits, no parity, 1 stop */
    mov al, 0x03
    out dx, al
    mov dx, 0x3FA               /* FIFO control: enable and clear */
    mov al, 0xC7
    out dx, al
    mov dx, 0x3FC               /* Modem control: DTR, RTS, OUT2 */
    mov al, 0x0B
    out dx, al

    mov esi, offset no_long_mode_message
    mov edi, 0xB8000
.Lno_long_mode_putc:
    lodsb
    test al, al
    jz .Lno_long_mode_halt
    mov bl, al
    cmp al, 0x20
    jb .Lno_long_mode_serial    /* Control characters go to COM1 only */
    mov ah, 0x4F                /* White on red */
    stosw
.Lno_long_mode_serial:
    mov ecx, 100000             /* Bounded wait in case COM1 is absent */
    mov dx, 0x3FD               /* Line status */
.Lno_long_mode_tx_wait:
    in al, dx
    test al, 0x20               /* Transmit holding register empty */
    jnz .Lno_long_mode_tx
    dec ecx
    jnz .Lno_long_mode_tx_wait
.Lno_long_mode_tx:
    mov dx, 0x3F8
    mov al, bl
    out dx, al
    jmp .Lno_long_mode_putc
.Lno_long_mode_halt:
    cli
    hlt
    jmp .Lno_long_mode_halt


/* =========================================================================
 * 64-Bit Long Mode Entry Point (.code64)
 * ========================================================================= */
.code64
long_mode_start:
    /* Reload all data segment registers with Kernel Data Segment selector (0x10) */
    mov ax, 0x10
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    mov ss, ax

    /* Set up 64-bit stack pointer (16-byte aligned) and reset RFLAGS */
    mov rsp, offset stack_top
    push 0
    popfq

    /* Populate 32-entry 64-bit IDT (vectors 0..31) so bare-metal NMIs do not
     * cause a double/triple fault reset. */
    lea rdi, [idt64]
    mov ecx, 32
    lea rdx, [exception_halt_stub]
.Lidt_init_loop:
    mov eax, edx
    and eax, 0xFFFF
    or  eax, (0x08 << 16)       /* Selector 0x08 (64-bit Kernel Code) */
    mov dword ptr [rdi], eax
    mov eax, edx
    and eax, 0xFFFF0000
    or  eax, 0x8E00             /* Present=1, DPL=0, 64-bit Interrupt Gate */
    mov dword ptr [rdi + 4], eax
    mov qword ptr [rdi + 8], 0
    add rdi, 16
    dec ecx
    jnz .Lidt_init_loop

    /* Override Vector 2 (NMI) to return cleanly via iretq */
    lea rdx, [nmi_isr_stub]
    lea rdi, [idt64 + 2 * 16]
    mov eax, edx
    and eax, 0xFFFF
    or  eax, (0x08 << 16)
    mov dword ptr [rdi], eax
    mov eax, edx
    and eax, 0xFFFF0000
    or  eax, 0x8E00
    mov dword ptr [rdi + 4], eax

    lidt [idt64_pointer]

    /* Keep the Multiboot magic and info address in callee-saved registers
     * across the calls below. The 32-bit moves also clear the upper halves,
     * which are undefined after the switch from 32-bit mode. */
    mov r12d, ebp
    mov r13d, esi
    xor rbp, rbp

    /* Run the C++ dynamic initializers that linker.ld collects in .init_array.
     * They run before kernel_main initializes any kernel service (console,
     * heap, interrupts), so they must not depend on one. */
    .extern g_init_array_start
    .extern g_init_array_end
    mov ebx, offset g_init_array_start
.Linit_array_loop:
    cmp rbx, offset g_init_array_end
    jae .Linit_array_done
    call qword ptr [rbx]
    add rbx, 8
    jmp .Linit_array_loop
.Linit_array_done:

    /* Pass (uint32_t magic, uint64_t mb_info_phys) in (edi, rsi) per System V AMD64 ABI */
    mov edi, r12d
    mov esi, r13d

    /* Call freestanding C++ kernel entry point */
    .extern kernel_main
    call kernel_main

    /* Clean CPU halt loop if kernel_main returns */
.Lhalt_loop:
    cli
    hlt
    jmp .Lhalt_loop

.align 16
nmi_isr_stub:
    iretq

.align 16
exception_halt_stub:
    cli
    hlt
    jmp exception_halt_stub


/* =========================================================================
 * SMP Application Processor (AP) Real-Mode Trampoline
 * Copied to physical address 0x8000 (SIPI vector 0x08) by the BSP.
 * Transitions each AP from 16-bit Real Mode -> 32-bit Protected Mode ->
 * 64-bit Long Mode using the kernel's PML4 page table and a per-AP stack.
 * ========================================================================= */
.align 16
.global ap_trampoline_start
.global ap_trampoline_end

.code16
ap_trampoline_start:
    cli
    cld
    xor ax, ax
    mov ds, ax
    mov es, ax
    mov ss, ax

    /* Load temporary GDT from inside the 0x8000 trampoline page */
    mov bx, 0x8000 + (ap_trampoline_gdtr - ap_trampoline_start)
    lgdt [bx]

    /* Enable Protected Mode (CR0.PE = bit 0) */
    mov eax, cr0
    or  al, 1
    mov cr0, eax

    /* Far jump to 32-bit Protected Mode entry (selector 0x18) */
    jmp 0x18:(0x8000 + ap_pm32_entry - ap_trampoline_start)

.align 8
.code32
ap_pm32_entry:
    mov ax, 0x10
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    mov ss, ax

    /* Signal to the BSP that this AP has started executing the trampoline */
    .extern g_ap_boot_started
    mov dword ptr [g_ap_boot_started], 1

    /* Enable PAE (bit 5), OSFXSR (bit 9), and OSXMMEXCPT (bit 10) in CR4 */
    mov eax, cr4
    or  eax, ((1 << 5) | (1 << 9) | (1 << 10))
    mov cr4, eax

    /* Load CR3 with physical base address of the shared kernel PML4 table */
    mov eax, offset pml4_table
    mov cr3, eax

    /* Enable long mode (EFER.LME) and, if this CPU supports it, no-execute
     * pages (EFER.NXE). Each AP checks its own CPUID because firmware can hide
     * NX per logical processor (IA32_MISC_ENABLE). The BSP already verified
     * that CPUID leaf 0x80000001 exists. */
    mov eax, 0x80000001
    cpuid
    test edx, CPUID_80000001_EDX_LM
    jz .Lap_no_long_mode
    mov edi, edx
    mov ecx, MSR_IA32_EFER
    rdmsr
    or  eax, EFER_LME
    test edi, CPUID_80000001_EDX_NX
    jz .Lap_write_efer
    or  eax, EFER_NXE
.Lap_write_efer:
    wrmsr

    /* Enable paging (see KERNEL_CR0) */
    mov eax, KERNEL_CR0
    mov cr0, eax

    /* Far jump to 64-bit Long Mode entry in kernel .text (selector 0x08) */
    jmp 0x08:ap_long_mode_entry

    /* This AP lacks long mode: halt it. SmpInit times out waiting for it to
     * come online. */
.Lap_no_long_mode:
    cli
    hlt
    jmp .Lap_no_long_mode

/* Accessed bits preset as in gdt64. */
.align 16
ap_trampoline_gdt:
    .quad 0x0000000000000000    /* 0x00: Null Descriptor */
    .quad 0x00AF9B000000FFFF    /* 0x08: 64-bit Kernel Code */
    .quad 0x00CF93000000FFFF    /* 0x10: 32/64-bit Kernel Data */
    .quad 0x00CF9B000000FFFF    /* 0x18: 32-bit Protected Mode Code */
ap_trampoline_gdt_end:

.align 8
ap_trampoline_gdtr:
    .word ap_trampoline_gdt_end - ap_trampoline_gdt - 1
    .long 0x8000 + (ap_trampoline_gdt - ap_trampoline_start)
ap_trampoline_end:

.align 16
.code64
ap_long_mode_entry:
    /* Switch to the permanent 64-bit GDT in kernel memory */
    lgdt [gdt64_pointer]

    mov ax, 0x10
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    mov ss, ax

    /* Bind this AP's CpuLocal: point IA32_GS_BASE at it (after the GS load
     * above, which zeroes the base). This precedes loading the kernel IDT, so
     * no C++ code, exception handlers included, ever runs on this AP with GS
     * base 0. */
    .extern g_ap_boot_cpu_local
    mov ecx, MSR_IA32_GS_BASE
    mov eax, dword ptr [g_ap_boot_cpu_local]
    mov edx, dword ptr [g_ap_boot_cpu_local + 4]
    wrmsr

    /* Switch to the kernel IDT */
    lidt [idt64_pointer]

    /* Load this AP's dedicated 64-bit stack pointer, reset RFLAGS, and load CPU slot index */
    .extern g_ap_boot_stack
    .extern g_ap_boot_cpu_index
    mov rsp, qword ptr [g_ap_boot_stack]
    push 0
    popfq
    mov edi, dword ptr [g_ap_boot_cpu_index]
    xor rbp, rbp

    .extern ApKernelEntry
    call ApKernelEntry

.Lap_halt_loop:
    cli
    hlt
    jmp .Lap_halt_loop


/* =========================================================================
 * 256-Entry 64-Bit Interrupt & Exception Service Routine (ISR) Stubs
 * Normalizes error codes across all 256 vectors, saves all 15 general-purpose
 * registers into an InterruptFrame, aligns RSP to 16 bytes per the System V
 * AMD64 ABI, calls IdtDispatch(InterruptFrame* frame), restores registers,
 * and returns via iretq.
 * ========================================================================= */
.macro ISR_NOERR_STUB vec
.align 16
.global isr_stub_\vec
isr_stub_\vec:
    push 0
    push \vec
    jmp isr_common_stub
.endm

.macro ISR_ERR_STUB vec
.align 16
.global isr_stub_\vec
isr_stub_\vec:
    push \vec
    jmp isr_common_stub
.endm

.align 16
.extern IdtDispatch
isr_common_stub:
    push rax
    push rbx
    push rcx
    push rdx
    push rsi
    push rdi
    push rbp
    push r8
    push r9
    push r10
    push r11
    push r12
    push r13
    push r14
    push r15

    cld
    mov rdi, rsp
    mov rbp, rsp
    and rsp, -16
    call IdtDispatch
    mov rsp, rbp

    pop r15
    pop r14
    pop r13
    pop r12
    pop r11
    pop r10
    pop r9
    pop r8
    pop rbp
    pop rdi
    pop rsi
    pop rdx
    pop rcx
    pop rbx
    pop rax
    add rsp, 16
    iretq

/* CPU Exceptions 0..31 (error code pushed by CPU on 8, 10..14, 17, 21, 29, 30) */
ISR_NOERR_STUB 0
ISR_NOERR_STUB 1
ISR_NOERR_STUB 2
ISR_NOERR_STUB 3
ISR_NOERR_STUB 4
ISR_NOERR_STUB 5
ISR_NOERR_STUB 6
ISR_NOERR_STUB 7
ISR_ERR_STUB   8
ISR_NOERR_STUB 9
ISR_ERR_STUB   10
ISR_ERR_STUB   11
ISR_ERR_STUB   12
ISR_ERR_STUB   13
ISR_ERR_STUB   14
ISR_NOERR_STUB 15
ISR_NOERR_STUB 16
ISR_ERR_STUB   17
ISR_NOERR_STUB 18
ISR_NOERR_STUB 19
ISR_NOERR_STUB 20
ISR_ERR_STUB   21
ISR_NOERR_STUB 22
ISR_NOERR_STUB 23
ISR_NOERR_STUB 24
ISR_NOERR_STUB 25
ISR_NOERR_STUB 26
ISR_NOERR_STUB 27
ISR_NOERR_STUB 28
ISR_ERR_STUB   29
ISR_ERR_STUB   30
ISR_NOERR_STUB 31

/* External / APIC / Software Interrupt Vectors 32..255 */
.irp vec, 32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63
    ISR_NOERR_STUB \vec
.endr
.irp vec, 64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95
    ISR_NOERR_STUB \vec
.endr
.irp vec, 96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127
    ISR_NOERR_STUB \vec
.endr
.irp vec, 128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159
    ISR_NOERR_STUB \vec
.endr
.irp vec, 160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191
    ISR_NOERR_STUB \vec
.endr
.irp vec, 192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223
    ISR_NOERR_STUB \vec
.endr
.irp vec, 224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255
    ISR_NOERR_STUB \vec
.endr


/* =========================================================================
 * 64-Bit Global Descriptor Table & IDT Pointer (Read-Only Data)
 * ========================================================================= */
.section .rodata
/* The boot GDT. Each CPU switches to the kernel GDT (gdt.cpp), which has the
 * same code and data segments plus the CPU's TSS, before it loads the kernel
 * IDT. The preset Accessed bits (bit 40) keep segment register loads from
 * writing to this table, so it can live in read-only memory. */
.align 16
gdt64:
    .quad 0x0000000000000000    /* 0x00: Null Descriptor */
    .quad 0x00AF9B000000FFFF    /* 0x08: 64-bit Kernel Code (P=1, DPL=0, S=1, Exec=1, R=1, A=1, L=1, D=0, G=1) */
    .quad 0x00CF93000000FFFF    /* 0x10: 64-bit Kernel Data (P=1, DPL=0, S=1, Exec=0, W=1, A=1, L=0, D/B=1, G=1) */
gdt64_end:

.align 16
gdt64_pointer:
    .word gdt64_end - gdt64 - 1 /* 16-bit Limit */
    .quad gdt64                 /* 64-bit Base Address */

.align 16
idt64_pointer:
    .word 32 * 16 - 1           /* 16-bit Limit (32 gate descriptors) */
    .quad idt64                 /* 64-bit Base Address */

.align 16
.global g_isr_stub_table
g_isr_stub_table:
.irp vec, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31
    .quad isr_stub_\vec
.endr
.irp vec, 32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63
    .quad isr_stub_\vec
.endr
.irp vec, 64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95
    .quad isr_stub_\vec
.endr
.irp vec, 96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127
    .quad isr_stub_\vec
.endr
.irp vec, 128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159
    .quad isr_stub_\vec
.endr
.irp vec, 160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191
    .quad isr_stub_\vec
.endr
.irp vec, 192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223
    .quad isr_stub_\vec
.endr
.irp vec, 224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255
    .quad isr_stub_\vec
.endr

no_long_mode_message:
    .asciz "[FATAL] This CPU does not support x86-64 long mode; halting.\r\n"


/* =========================================================================
 * BSS Section: Kernel Stack, Page Tables (4 KiB aligned), and IDT
 * ========================================================================= */
.section .bss
/* CPU 0's 16 KiB boot stack, directly above a 4 KiB guard page that GdtInit()
 * unmaps, so that an overflow faults instead of overwriting the memory below.
 * The page tables follow the stack top, out of an overflow's way. */
.align 4096
.global stack_guard
stack_guard:
    .skip 4096
.global stack_bottom
.global stack_top
stack_bottom:
    .skip 16384                 /* 16 KiB Kernel Stack */
stack_top:

.align 4096
pml4_table:
    .skip 4096
pdpt_table:
    .skip 4096
pd_table:
    .skip 4096

.align 16
idt64:
    .skip 32 * 16

.section .note.GNU-stack,"",@progbits
