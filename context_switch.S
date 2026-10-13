/* context_switch.S — task context switch (GNU as, Intel syntax, System V
 * AMD64 ABI). The kernel and the host scheduler tests assemble this same file;
 * core_runqueue_scheduler.cpp declares both routines and calls them. */

.intel_syntax noprefix

.text

/* void TaskContextSwitch(uintptr_t* prev_rsp, uintptr_t next_rsp)
 *   rdi = &prev->rsp
 *   rsi = next->rsp
 *
 * Pushes the callee-saved registers (rbp, rbx, r12..r15) on the outgoing
 * stack, stores RSP into *prev_rsp, loads next_rsp into RSP, pops the incoming
 * context's callee-saved registers, and returns to its saved RIP: into the
 * TaskContextSwitch() call that suspended it, or, for a new task, to
 * TaskEntryTrampoline. The frame at next_rsp therefore holds r15, r14, r13,
 * r12, rbx, rbp and the return address, from the lowest address up;
 * InitialSwitchFrame in core_runqueue_scheduler.cpp follows this pop order.
 *
 * Both stacks hold the same frame layout, so the CFI below describes the
 * incoming context's frame once RSP has been switched. */
.align 16
.global TaskContextSwitch
.type TaskContextSwitch, @function
TaskContextSwitch:
    .cfi_startproc
    push rbp
    .cfi_adjust_cfa_offset 8
    .cfi_rel_offset rbp, 0
    push rbx
    .cfi_adjust_cfa_offset 8
    .cfi_rel_offset rbx, 0
    push r12
    .cfi_adjust_cfa_offset 8
    .cfi_rel_offset r12, 0
    push r13
    .cfi_adjust_cfa_offset 8
    .cfi_rel_offset r13, 0
    push r14
    .cfi_adjust_cfa_offset 8
    .cfi_rel_offset r14, 0
    push r15
    .cfi_adjust_cfa_offset 8
    .cfi_rel_offset r15, 0

    mov qword ptr [rdi], rsp
    mov rsp, rsi

    pop r15
    .cfi_adjust_cfa_offset -8
    .cfi_restore r15
    pop r14
    .cfi_adjust_cfa_offset -8
    .cfi_restore r14
    pop r13
    .cfi_adjust_cfa_offset -8
    .cfi_restore r13
    pop r12
    .cfi_adjust_cfa_offset -8
    .cfi_restore r12
    pop rbx
    .cfi_adjust_cfa_offset -8
    .cfi_restore rbx
    pop rbp
    .cfi_adjust_cfa_offset -8
    .cfi_restore rbp
    ret
    .cfi_endproc
.size TaskContextSwitch, . - TaskContextSwitch

/* First code of a new task, entered by the `ret` of TaskContextSwitch with r12
 * holding the task's Task*. Clears rbp and marks the return address undefined,
 * so that frame-pointer and CFI unwinders stop here, aligns the stack to 16
 * bytes, and calls TaskStartupEntry(r12). TaskStartupEntry does not return; if
 * it ever did, ud2 would raise #UD (SIGILL in host tests). */
.align 16
.global TaskEntryTrampoline
.type TaskEntryTrampoline, @function
TaskEntryTrampoline:
    .cfi_startproc
    .cfi_undefined rip
    xor rbp, rbp
    and rsp, -16
    mov rdi, r12
    call TaskStartupEntry
    ud2
    .cfi_endproc
.size TaskEntryTrampoline, . - TaskEntryTrampoline

.section .note.GNU-stack,"",@progbits
