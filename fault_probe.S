/* fault_probe.S — calling a function whose page fault the kernel recovers
 * from (RunFaultProbe() in idt.h). GNU as, Intel syntax, System V AMD64 ABI.
 * Kernel only: idt.cpp declares FaultProbeCall and calls it. */

.intel_syntax noprefix

/* Offsets in FaultProbe (idt.h checks them with static_assert). */
.set FAULT_PROBE_RESUME_RIP, 0
.set FAULT_PROBE_RESUME_RSP, 8

.text

/* bool FaultProbeCall(void (*fn)(uintptr_t), uintptr_t arg, FaultProbe* probe)
 *   rdi = fn
 *   rsi = arg
 *   rdx = probe
 *
 * Saves the callee-saved registers, stores the resume point (the label 1:
 * below and the RSP that it expects) in `probe` and calls fn(arg), with RSP
 * 16-byte aligned at the call. Returns false when fn returns. When fn
 * page-faults, IdtDispatch() resumes at 1: with RSP set to the stored resume
 * RSP, and the call returns true with the caller's callee-saved registers
 * restored from this frame (fn may have changed them). The caller keeps
 * interrupts disabled throughout. */
.align 16
.global FaultProbeCall
.type FaultProbeCall, @function
FaultProbeCall:
    push rbp
    push rbx
    push r12
    push r13
    push r14
    push r15
    /* RSP was 8 mod 16 on entry; after 6 pushes and this, it is 0 mod 16. */
    sub rsp, 8
    mov qword ptr [rdx + FAULT_PROBE_RESUME_RSP], rsp
    lea rax, [rip + 1f]
    mov qword ptr [rdx + FAULT_PROBE_RESUME_RIP], rax
    mov rax, rdi
    mov rdi, rsi
    call rax
    xor eax, eax
    jmp 2f
1:
    mov eax, 1
2:
    add rsp, 8
    pop r15
    pop r14
    pop r13
    pop r12
    pop rbx
    pop rbp
    ret
.size FaultProbeCall, . - FaultProbeCall

.section .note.GNU-stack,"",@progbits
