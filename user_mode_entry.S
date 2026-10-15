/* user_mode_entry.S — entering and leaving user mode (GNU as, Intel syntax,
 * System V AMD64 ABI). See user_mode.h. */

.intel_syntax noprefix

/* Selectors and flags of the user context (kGdtUserDataSelector and
 * kGdtUserCodeSelector in gdt.h; user_mode.cpp checks that they match). User
 * code starts with interrupts enabled (IF) and I/O privilege level 0, so that
 * cli, sti, in and out fault. */
.set USER_DATA_SELECTOR, 0x23
.set USER_CODE_SELECTOR, 0x2B
.set USER_RFLAGS, 0x202

.text

/* void UserModeEnter(uint64_t rip, uint64_t rsp, uint64_t arg,
 *                    UserProcess* process)
 *   rdi = entry point, rsi = user stack pointer, rdx = argument for the
 *   program (its rdi), rcx = process
 *
 * Saves the kernel context on the task's kernel stack and enters user mode at
 * `rip` with stack pointer `rsp`, rdi = `arg`, and every other general-purpose
 * register 0. Returns when UserModeExit() resumes the saved context.
 *
 * The saved context, from the lowest address up, is: `process`, a pad word,
 * RFLAGS, r15, r14, r13, r12, rbx, rbp, and the return address. The address
 * of `process`, the kernel stack pointer while the program runs, is 16-byte
 * aligned (the entry RSP is 8 mod 16, and 9 words follow), which TSS.rsp0
 * requires. The CPU pushes the interrupt frame of every interrupt from user
 * mode right below it, so that `process` is the word above the frame. */
.align 16
.global UserModeEnter
.type UserModeEnter, @function
UserModeEnter:
    push rbp
    push rbx
    push r12
    push r13
    push r14
    push r15
    pushfq
    sub rsp, 8
    push rcx

    /* With interrupts disabled from here on, the task stays on this CPU, whose
     * TSS.rsp0 must point at the saved context when the CPU enters user mode. */
    cli
    mov r12, rdi
    mov r13, rsi
    mov r14, rdx
    mov rdi, rsp
    call UserModeSetKernelStack

    /* The interrupt return frame. */
    push USER_DATA_SELECTOR
    push r13
    push USER_RFLAGS
    push USER_CODE_SELECTOR
    push r12

    mov rdi, r14
    xor eax, eax
    xor ebx, ebx
    xor ecx, ecx
    xor edx, edx
    xor esi, esi
    xor ebp, ebp
    xor r8d, r8d
    xor r9d, r9d
    xor r10d, r10d
    xor r11d, r11d
    xor r12d, r12d
    xor r13d, r13d
    xor r14d, r14d
    xor r15d, r15d
    /* Load the user GS base (see the ISR stubs in boot.S). */
    swapgs
    iretq
.size UserModeEnter, . - UserModeEnter

/* [[noreturn]] void UserModeExit(uintptr_t kernel_rsp)
 *   rdi = the kernel stack pointer of the running program: the address of
 *   the saved context
 *
 * Called in the kernel, on the task's kernel stack, by an interrupt handler of
 * the running program. Drops the stack above `kernel_rsp`, restores the
 * kernel context that UserModeEnter() saved (RFLAGS included, which enables
 * interrupts again), and returns from UserModeEnter(). */
.align 16
.global UserModeExit
.type UserModeExit, @function
UserModeExit:
    lea rsp, [rdi + 16]
    popfq
    pop r15
    pop r14
    pop r13
    pop r12
    pop rbx
    pop rbp
    ret
.size UserModeExit, . - UserModeExit

.section .note.GNU-stack,"",@progbits

