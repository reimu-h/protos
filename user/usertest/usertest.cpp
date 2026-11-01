// usertest: a user program that the boot self-tests run in many ways to check
// user mode (see user/usertest/usertest.h).

#include "user/usertest/usertest.h"

#include <cstddef>
#include <cstdint>

#include "syscall_abi.h"
#include "user/syscall.h"
#include "user/usbkbd/xhci_platform.h"
#include "user/usbkbd/xhci_regs.h"

static_assert(protos::kSyscallGetCpu == 2);
static_assert(protos::kSyscallSpin == 3);
static_assert(protos::kSyscallRead == 4);
static_assert(protos::kSyscallSpawn == 5);
static_assert(protos::kSyscallWait == 6);
// `KernelSpinArg()` can ask for the longest spin() call.
static_assert(protos::kSyscallSpinMaxTicks <=
              protos::usertest::kKernelSpinMaxTicks);

// Makes a get_cpu() system call with distinct values in every general-purpose
// register but rax and rsp, and returns the bits of these registers that the
// call changed: 0 if it preserved them all.
extern "C" uint64_t SyscallChangedRegisterBits();

// What `FpuHoldRound()` does. The offsets of the fields are part of its code.
struct FpuHoldParams {
  // The x87/SSE/AVX state to load, in the XSAVE format (FXSAVE format if
  // `components` is 0). 64-byte aligned.
  const void* image;
  // nullptr, or the values to load into MM0-MM7 after `image`.
  const uint64_t* mmx_values;
  // Where to save the state after loading it, and after the system calls.
  // 64-byte aligned.
  void* before;
  void* after;
  // The XSAVE state components to load and save (EDX:EAX of XRSTOR/XSAVE),
  // or 0 to use FXRSTOR/FXSAVE.
  uint64_t components;
  // The get_cpu() system calls to make, and the iterations of integer-only
  // computation before each.
  int64_t syscalls;
  int64_t iterations;
};
static_assert(offsetof(FpuHoldParams, image) == 0);
static_assert(offsetof(FpuHoldParams, mmx_values) == 8);
static_assert(offsetof(FpuHoldParams, before) == 16);
static_assert(offsetof(FpuHoldParams, after) == 24);
static_assert(offsetof(FpuHoldParams, components) == 32);
static_assert(offsetof(FpuHoldParams, syscalls) == 40);
static_assert(offsetof(FpuHoldParams, iterations) == 48);

// Loads the state that `*params` gives, saves it into `params->before`, makes
// the system calls with nothing but integer computation in between, saves the
// state into `params->after`, and then leaves the x87 FPU and MXCSR in their
// initial state, as the ABI expects at a return. All in one routine, so that
// no compiled code runs while the registers hold the loaded values.
extern "C" void FpuHoldRound(const FpuHoldParams* params);

asm(R"(
    .pushsection .text
    .globl FpuHoldRound
    .type FpuHoldRound, @function
FpuHoldRound:
    push rbx
    push r12
    push r13
    push r14
    push r15
    mov r12, rdi
    mov r13, [r12 + 32]
    mov rsi, [r12]
    test r13, r13
    jz .Lfpu_hold_fxrstor
    mov eax, r13d
    xor edx, edx
    xrstor64 [rsi]
    jmp .Lfpu_hold_mmx
.Lfpu_hold_fxrstor:
    fxrstor64 [rsi]
.Lfpu_hold_mmx:
    mov rsi, [r12 + 8]
    test rsi, rsi
    jz .Lfpu_hold_save_before
    movq mm0, [rsi]
    movq mm1, [rsi + 8]
    movq mm2, [rsi + 16]
    movq mm3, [rsi + 24]
    movq mm4, [rsi + 32]
    movq mm5, [rsi + 40]
    movq mm6, [rsi + 48]
    movq mm7, [rsi + 56]
.Lfpu_hold_save_before:
    mov rdi, [r12 + 16]
    test r13, r13
    jz .Lfpu_hold_fxsave_before
    mov eax, r13d
    xor edx, edx
    xsave64 [rdi]
    jmp .Lfpu_hold_hold
.Lfpu_hold_fxsave_before:
    fxsave64 [rdi]
.Lfpu_hold_hold:
    mov r14, [r12 + 40]
    mov rbx, 0x2545F4914F6CDD1D
.Lfpu_hold_syscall:
    mov r15, [r12 + 48]
.Lfpu_hold_compute:
    mov rcx, rbx
    shl rcx, 13
    xor rbx, rcx
    mov rcx, rbx
    shr rcx, 7
    xor rbx, rcx
    mov rcx, rbx
    shl rcx, 17
    xor rbx, rcx
    dec r15
    jnz .Lfpu_hold_compute
    mov eax, 2
    int 0x80
    dec r14
    jnz .Lfpu_hold_syscall
    mov rdi, [r12 + 24]
    test r13, r13
    jz .Lfpu_hold_fxsave_after
    mov eax, r13d
    xor edx, edx
    xsave64 [rdi]
    jmp .Lfpu_hold_reset
.Lfpu_hold_fxsave_after:
    fxsave64 [rdi]
.Lfpu_hold_reset:
    test r13d, 4
    jz .Lfpu_hold_no_avx
    vzeroupper
.Lfpu_hold_no_avx:
    fninit
    push 0x1F80
    ldmxcsr [rsp]
    add rsp, 8
    pop r15
    pop r14
    pop r13
    pop r12
    pop rbx
    ret
    .size FpuHoldRound, . - FpuHoldRound
    .popsection
)");

asm(R"(
    .pushsection .text
    .globl SyscallChangedRegisterBits
    .type SyscallChangedRegisterBits, @function
SyscallChangedRegisterBits:
    push rbx
    push rbp
    push r12
    push r13
    push r14
    push r15
    mov rbx, 0x11111111
    mov rcx, -0x22222222
    mov rdx, 0x33333333
    mov rsi, -0x44444444
    mov rdi, 0x55555555
    mov rbp, -0x66666666
    mov r8, 0x77777777
    mov r9, -0x08888888
    mov r10, 0x19999999
    mov r11, -0x2AAAAAAA
    mov r12, 0x3BBBBBBB
    mov r13, -0x4CCCCCCC
    mov r14, 0x5DDDDDDD
    mov r15, -0x6EEEEEEE
    mov eax, 2
    int 0x80
    xor rbx, 0x11111111
    xor rcx, -0x22222222
    xor rdx, 0x33333333
    xor rsi, -0x44444444
    xor rdi, 0x55555555
    xor rbp, -0x66666666
    xor r8, 0x77777777
    xor r9, -0x08888888
    xor r10, 0x19999999
    xor r11, -0x2AAAAAAA
    xor r12, 0x3BBBBBBB
    xor r13, -0x4CCCCCCC
    xor r14, 0x5DDDDDDD
    xor r15, -0x6EEEEEEE
    mov rax, rbx
    or rax, rcx
    or rax, rdx
    or rax, rsi
    or rax, rdi
    or rax, rbp
    or rax, r8
    or rax, r9
    or rax, r10
    or rax, r11
    or rax, r12
    or rax, r13
    or rax, r14
    or rax, r15
    pop r15
    pop r14
    pop r13
    pop r12
    pop rbp
    pop rbx
    ret
    .size SyscallChangedRegisterBits, . - SyscallChangedRegisterBits
    .popsection
)");

// Makes a spin(buffer, size, ticks) system call with values that depend on
// `seed` and differ from register to register in every other general-purpose
// register but rsp, stores its result in `*result`, and returns the bits of
// the registers but rax that the call changed: 0 if it preserved them all.
extern "C" uint64_t SyscallSpinChangedRegisterBits(const void* buffer,  //
                                                   int64_t size,        //
                                                   int64_t ticks,       //
                                                   int64_t* result,     //
                                                   uint64_t seed);

// The seed arrives in r8 and stays at [rsp]; ticks, size, buffer and result
// follow at [rsp + 8] to [rsp + 32].
asm(R"(
    .pushsection .text
    .globl SyscallSpinChangedRegisterBits
    .type SyscallSpinChangedRegisterBits, @function
SyscallSpinChangedRegisterBits:
    push rbx
    push rbp
    push r12
    push r13
    push r14
    push r15
    push rcx
    push rdi
    push rsi
    push rdx
    push r8
    mov rbx, 0x11111111
    xor rbx, r8
    mov rcx, -0x22222222
    xor rcx, r8
    mov rbp, -0x66666666
    xor rbp, r8
    mov r9, -0x08888888
    xor r9, r8
    mov r10, 0x19999999
    xor r10, r8
    mov r11, -0x2AAAAAAA
    xor r11, r8
    mov r12, 0x3BBBBBBB
    xor r12, r8
    mov r13, -0x4CCCCCCC
    xor r13, r8
    mov r14, 0x5DDDDDDD
    xor r14, r8
    mov r15, -0x6EEEEEEE
    xor r15, r8
    xor r8, 0x77777777
    mov eax, 3
    int 0x80
    xor rdx, [rsp + 8]
    xor rsi, [rsp + 16]
    xor rdi, [rsp + 24]
    xor rbx, [rsp]
    xor rbx, 0x11111111
    xor rcx, [rsp]
    xor rcx, -0x22222222
    xor rbp, [rsp]
    xor rbp, -0x66666666
    xor r8, [rsp]
    xor r8, 0x77777777
    xor r9, [rsp]
    xor r9, -0x08888888
    xor r10, [rsp]
    xor r10, 0x19999999
    xor r11, [rsp]
    xor r11, -0x2AAAAAAA
    xor r12, [rsp]
    xor r12, 0x3BBBBBBB
    xor r13, [rsp]
    xor r13, -0x4CCCCCCC
    xor r14, [rsp]
    xor r14, 0x5DDDDDDD
    xor r15, [rsp]
    xor r15, -0x6EEEEEEE
    or rbx, rcx
    or rbx, rdx
    or rbx, rsi
    or rbx, rdi
    or rbx, rbp
    or rbx, r8
    or rbx, r9
    or rbx, r10
    or rbx, r11
    or rbx, r12
    or rbx, r13
    or rbx, r14
    or rbx, r15
    mov rcx, [rsp + 32]
    mov [rcx], rax
    mov rax, rbx
    add rsp, 40
    pop r15
    pop r14
    pop r13
    pop r12
    pop rbp
    pop rbx
    ret
    .size SyscallSpinChangedRegisterBits, . - SyscallSpinChangedRegisterBits
    .popsection
)");

// Counts `iterations` (> 0) down to 0 with values that depend on `seed` and
// differ from register to register in every other general-purpose register
// but rsp, and returns the bits of these registers that changed meanwhile: 0
// if none did. Makes no system call.
extern "C" uint64_t SpinChangedRegisterBits(int64_t iterations, uint64_t seed);

// The seed arrives in rsi and stays at [rsp].
asm(R"(
    .pushsection .text
    .globl SpinChangedRegisterBits
    .type SpinChangedRegisterBits, @function
SpinChangedRegisterBits:
    push rbx
    push rbp
    push r12
    push r13
    push r14
    push r15
    push rsi
    mov rax, rdi
    mov rbx, 0x11111111
    xor rbx, rsi
    mov rcx, -0x22222222
    xor rcx, rsi
    mov rdx, 0x33333333
    xor rdx, rsi
    mov rdi, 0x55555555
    xor rdi, rsi
    mov rbp, -0x66666666
    xor rbp, rsi
    mov r8, 0x77777777
    xor r8, rsi
    mov r9, -0x08888888
    xor r9, rsi
    mov r10, 0x19999999
    xor r10, rsi
    mov r11, -0x2AAAAAAA
    xor r11, rsi
    mov r12, 0x3BBBBBBB
    xor r12, rsi
    mov r13, -0x4CCCCCCC
    xor r13, rsi
    mov r14, 0x5DDDDDDD
    xor r14, rsi
    mov r15, -0x6EEEEEEE
    xor r15, rsi
    xor rsi, -0x44444444
1:
    sub rax, 1
    jnz 1b
    xor rbx, [rsp]
    xor rbx, 0x11111111
    xor rcx, [rsp]
    xor rcx, -0x22222222
    xor rdx, [rsp]
    xor rdx, 0x33333333
    xor rsi, [rsp]
    xor rsi, -0x44444444
    xor rdi, [rsp]
    xor rdi, 0x55555555
    xor rbp, [rsp]
    xor rbp, -0x66666666
    xor r8, [rsp]
    xor r8, 0x77777777
    xor r9, [rsp]
    xor r9, -0x08888888
    xor r10, [rsp]
    xor r10, 0x19999999
    xor r11, [rsp]
    xor r11, -0x2AAAAAAA
    xor r12, [rsp]
    xor r12, 0x3BBBBBBB
    xor r13, [rsp]
    xor r13, -0x4CCCCCCC
    xor r14, [rsp]
    xor r14, 0x5DDDDDDD
    xor r15, [rsp]
    xor r15, -0x6EEEEEEE
    mov rax, rbx
    or rax, rcx
    or rax, rdx
    or rax, rsi
    or rax, rdi
    or rax, rbp
    or rax, r8
    or rax, r9
    or rax, r10
    or rax, r11
    or rax, r12
    or rax, r13
    or rax, r14
    or rax, r15
    add rsp, 8
    pop r15
    pop r14
    pop r13
    pop r12
    pop rbp
    pop rbx
    ret
    .size SpinChangedRegisterBits, . - SpinChangedRegisterBits
    .popsection
)");

// What `FpuSpinRound()` does. The offsets of the fields are part of its code.
struct FpuSpinParams {
  // The x87/SSE/AVX state to load, in the XSAVE format (FXSAVE format if
  // `components` is 0), and nullptr or the values to load into MM0-MM7 after
  // it. 64-byte aligned.
  const void* image;
  const uint64_t* mmx_values;
  // Where to save the state after loading it, and after the system call.
  // 64-byte aligned.
  void* before;
  void* after;
  // The XSAVE state components to load and save (EDX:EAX of XRSTOR/XSAVE),
  // or 0 to use FXRSTOR/FXSAVE.
  uint64_t components;
  // The arguments of the spin() system call, and the seed of the values in
  // the other general-purpose registers (`SyscallSpinChangedRegisterBits()`).
  const void* buffer;
  int64_t size;
  int64_t ticks;
  uint64_t register_seed;
  // Results: what the call returned, and the bits of the general-purpose
  // registers but rax that it changed.
  int64_t result;
  uint64_t changed_bits;
};
static_assert(offsetof(FpuSpinParams, image) == 0);
static_assert(offsetof(FpuSpinParams, mmx_values) == 8);
static_assert(offsetof(FpuSpinParams, before) == 16);
static_assert(offsetof(FpuSpinParams, after) == 24);
static_assert(offsetof(FpuSpinParams, components) == 32);
static_assert(offsetof(FpuSpinParams, buffer) == 40);
static_assert(offsetof(FpuSpinParams, size) == 48);
static_assert(offsetof(FpuSpinParams, ticks) == 56);
static_assert(offsetof(FpuSpinParams, register_seed) == 64);
static_assert(offsetof(FpuSpinParams, result) == 72);
static_assert(offsetof(FpuSpinParams, changed_bits) == 80);

// Like `FpuHoldRound()`, but with one spin() system call instead of the
// get_cpu() calls: loads the state that `*params` gives, saves it into
// `params->before`, makes the call with `SyscallSpinChangedRegisterBits()`
// (which touches no x87/SSE/AVX register) and stores its results in
// `*params`, saves the state into `params->after`, and then leaves the x87
// FPU and MXCSR in their initial state. All in one routine, so that no
// compiled code runs while the registers hold the loaded values.
extern "C" void FpuSpinRound(FpuSpinParams* params);

asm(R"(
    .pushsection .text
    .globl FpuSpinRound
    .type FpuSpinRound, @function
FpuSpinRound:
    push rbx
    push r12
    push r13
    mov r12, rdi
    mov r13, [r12 + 32]
    mov rsi, [r12]
    test r13, r13
    jz .Lfpu_spin_fxrstor
    mov eax, r13d
    xor edx, edx
    xrstor64 [rsi]
    jmp .Lfpu_spin_mmx
.Lfpu_spin_fxrstor:
    fxrstor64 [rsi]
.Lfpu_spin_mmx:
    mov rsi, [r12 + 8]
    test rsi, rsi
    jz .Lfpu_spin_save_before
    movq mm0, [rsi]
    movq mm1, [rsi + 8]
    movq mm2, [rsi + 16]
    movq mm3, [rsi + 24]
    movq mm4, [rsi + 32]
    movq mm5, [rsi + 40]
    movq mm6, [rsi + 48]
    movq mm7, [rsi + 56]
.Lfpu_spin_save_before:
    mov rdi, [r12 + 16]
    test r13, r13
    jz .Lfpu_spin_fxsave_before
    mov eax, r13d
    xor edx, edx
    xsave64 [rdi]
    jmp .Lfpu_spin_call
.Lfpu_spin_fxsave_before:
    fxsave64 [rdi]
.Lfpu_spin_call:
    mov rdi, [r12 + 40]
    mov rsi, [r12 + 48]
    mov rdx, [r12 + 56]
    lea rcx, [r12 + 72]
    mov r8, [r12 + 64]
    call SyscallSpinChangedRegisterBits
    mov [r12 + 80], rax
    mov rdi, [r12 + 24]
    test r13, r13
    jz .Lfpu_spin_fxsave_after
    mov eax, r13d
    xor edx, edx
    xsave64 [rdi]
    jmp .Lfpu_spin_reset
.Lfpu_spin_fxsave_after:
    fxsave64 [rdi]
.Lfpu_spin_reset:
    test r13d, 4
    jz .Lfpu_spin_no_avx
    vzeroupper
.Lfpu_spin_no_avx:
    fninit
    push 0x1F80
    ldmxcsr [rsp]
    add rsp, 8
    pop r13
    pop r12
    pop rbx
    ret
    .size FpuSpinRound, . - FpuSpinRound
    .popsection
)");

// Unmasks the x87 zero-divide exception (control word 0x037B) and divides 1
// by 0, which leaves the exception pending, and stores the x87 status word at
// `*status_word`. Then makes `rounds` (> 0) get_cpu() system calls, each
// after chunks of `iterations` (> 0) iterations of integer computation until
// the TSC has advanced by at least `round_cycles` since the round began (one
// chunk if `round_cycles` is 0), and executes FWAIT, which raises the
// pending exception (#MF). Returns only if FWAIT did not. Nothing but FNSTSW
// touches the x87 FPU in between (RDTSC is no x87 instruction).
extern "C" void X87PendingZeroDivide(uint16_t* status_word, int64_t rounds,
                                     int64_t iterations, int64_t round_cycles);

// The system calls preserve every register but rax.
asm(R"(
    .pushsection .text
    .globl X87PendingZeroDivide
    .type X87PendingZeroDivide, @function
X87PendingZeroDivide:
    push rbx
    mov r8, rdi
    mov r9, rsi
    mov r10, rdx
    mov r11, rcx
    push 0x037B
    fldcw word ptr [rsp]
    add rsp, 8
    fldz
    fld1
    fdiv st(0), st(1)
    fnstsw word ptr [r8]
    xor ebx, ebx
.Lx87_pending_round:
    rdtsc
    shl rdx, 32
    or rax, rdx
    mov rsi, rax
.Lx87_pending_chunk:
    mov rcx, r10
.Lx87_pending_compute:
    add rbx, rcx
    dec rcx
    jnz .Lx87_pending_compute
    rdtsc
    shl rdx, 32
    or rax, rdx
    sub rax, rsi
    cmp rax, r11
    jb .Lx87_pending_chunk
    mov eax, 2
    int 0x80
    dec r9
    jnz .Lx87_pending_round
    fwait
    pop rbx
    ret
    .size X87PendingZeroDivide, . - X87PendingZeroDivide
    .popsection
)");

namespace protos::user {
namespace {

using usertest::Mode;

constexpr uint64_t kRflagsInterruptEnable = 1 << 9;
constexpr uint64_t kRflagsIoPrivilegeLevel = 3 << 12;
// kMaxSupportedCpus in smp.h.
constexpr int kMaxCpus = 128;
// Odd, so that it maps different seeds to different register values, which
// spread over all 64 bits.
constexpr uint64_t kRegisterSeedMultiplier = 0x9E3779B97F4A7C15;

const char kNulBytes[8] = {};
constexpr int64_t kNulByteCount = sizeof(kNulBytes);
const uint64_t kReadOnlyWord = 0x0123456789ABCDEF;
// `ret`, in writable data.
uint8_t g_return_instruction[16] = {0xC3};
// What `kSpin` writes to its data: each process has its own copy.
uint64_t g_spin_seed = 0;
// Never set: `Recurse()` recurses until the stack overflows.
volatile uint64_t g_recursion_limit = 0;

constexpr int64_t kPageSize = 4096;
// The buffer of `Mode::kKernelSpin`: two pages, so that the bytes of a spin()
// call may cross a page boundary. Each process has its own copy.
alignas(kPageSize) uint8_t g_kernel_spin_buffer[2 * kPageSize];
// The iterations of the register loop of `Mode::kSpinWithoutSyscalls` between
// two checks of its stack and data.
constexpr int64_t kQuietSpinIterations = int64_t{1} << 16;

// Hides from the compiler where `pointer` points.
template <typename T>
static T* Launder(T* pointer) {
  asm volatile("" : "+r"(pointer));
  return pointer;
}

static int CurrentPrivilegeLevel() {
  uint16_t cs = 0;
  asm volatile("mov %0, cs" : "=r"(cs));
  return cs & 3;
}

static uint64_t ReadRflags() {
  uint64_t rflags = 0;
  asm volatile("pushfq\n\tpop %0" : "=r"(rflags));
  return rflags;
}

struct SegmentSelectors {
  uint16_t ds = 0;
  uint16_t es = 0;
  uint16_t fs = 0;
  uint16_t gs = 0;
};

static SegmentSelectors ReadDataSelectors() {
  SegmentSelectors s;
  asm volatile(
      "mov %0, ds\n\t"
      "mov %1, es\n\t"
      "mov %2, fs\n\t"
      "mov %3, gs"
      : "=r"(s.ds), "=r"(s.es), "=r"(s.fs), "=r"(s.gs));
  return s;
}

static void WriteDataSelectors(const SegmentSelectors& s) {
  asm volatile(
      "mov ds, %0\n\t"
      "mov es, %1\n\t"
      "mov fs, %2\n\t"
      "mov gs, %3"
      :
      : "r"(s.ds), "r"(s.es), "r"(s.fs), "r"(s.gs)
      : "memory");
}

// Returns true if a spin() call of the `size` bytes at `buffer` fails with
// `kSyscallErrorFault`.
static bool SpinFaults(const uintptr_t buffer, const int64_t size) {
  return SpinOnBuffer(reinterpret_cast<const void*>(buffer), size, 0) ==
         kSyscallErrorFault;
}

// Runs the checks of `Mode::kSyscalls` and returns 0 if all pass, else the
// number of the first failed check.
static int64_t RunSyscallChecks() {
  constexpr uint16_t kUserDataSelector = 0x23;
  const SegmentSelectors initial_selectors = ReadDataSelectors();
  int64_t check = 1;
  if (CurrentPrivilegeLevel() != 3 ||
      initial_selectors.ds != kUserDataSelector ||
      initial_selectors.es != kUserDataSelector ||
      initial_selectors.fs != kUserDataSelector ||
      initial_selectors.gs != kUserDataSelector) {
    return check;
  }
  ++check;
  const uint64_t rflags = ReadRflags();
  if ((rflags & kRflagsInterruptEnable) == 0 ||
      (rflags & kRflagsIoPrivilegeLevel) != 0) {
    return check;
  }
  ++check;
  if (SyscallChangedRegisterBits() != 0) {
    return check;
  }
  ++check;
  // write() leaves out NUL bytes, so these writes print nothing.
  if (Write(kNulBytes, kNulByteCount) != kNulByteCount ||
      Write(nullptr, 0) != 0) {
    return check;
  }
  ++check;
  if (Write(kNulBytes, -1) != kSyscallErrorInvalid ||
      Write(kNulBytes, kSyscallWriteMaxSize + 1) != kSyscallErrorInvalid) {
    return check;
  }
  ++check;
  // Kernel memory, an unmapped user page, and a range that wraps around.
  if (Write(reinterpret_cast<const void*>(usertest::kKernelAddress), 8) !=
          kSyscallErrorFault ||
      Write(reinterpret_cast<const void*>(0xFFFF800000000000ULL), 8) !=
          kSyscallErrorFault ||
      Write(reinterpret_cast<const void*>(0xFFFFFFFFFFFFFFF8ULL), 16) !=
          kSyscallErrorFault) {
    return check;
  }
  ++check;
  // The stack is small, so this frame is in its top page, above which nothing
  // is mapped. The top 8 bytes hold the zero return address of the entry
  // point.
  const uintptr_t stack_top =
      (reinterpret_cast<uintptr_t>(__builtin_frame_address(0)) | 0xFFF) + 1;
  if (Write(reinterpret_cast<const void*>(stack_top - 8), 8) != 8 ||
      Write(reinterpret_cast<const void*>(stack_top - 4), 8) !=
          kSyscallErrorFault) {
    return check;
  }
  ++check;
  if (Syscall(1000) != kSyscallErrorNoSys ||
      Syscall(-1) != kSyscallErrorNoSys) {
    return check;
  }
  ++check;
  const int cpu = GetCpu();
  if (cpu < 0 || cpu >= kMaxCpus) {
    return check;
  }
  ++check;
  if (SpinOnBuffer(kNulBytes, -1, 0) != kSyscallErrorInvalid ||
      SpinOnBuffer(kNulBytes, kSyscallSpinMaxSize + 1, 0) !=
          kSyscallErrorInvalid ||
      SpinOnBuffer(kNulBytes, kNulByteCount, -1) != kSyscallErrorInvalid ||
      SpinOnBuffer(kNulBytes, kNulByteCount, kSyscallSpinMaxTicks + 1) !=
          kSyscallErrorInvalid) {
    return check;
  }
  ++check;
  // As for write() above, and a range that runs past the top of the stack.
  if (!SpinFaults(usertest::kKernelAddress, 8) ||
      !SpinFaults(0xFFFF800000000000ULL, 8) ||
      !SpinFaults(0xFFFFFFFFFFFFFFF8ULL, 16) || !SpinFaults(stack_top - 4, 8)) {
    return check;
  }
  ++check;
  if (SpinOnBuffer(nullptr, 0, 0) != kSyscallSpinChecksumInit ||
      SpinOnBuffer(kNulBytes, kNulByteCount, 1) !=
          SyscallSpinChecksum(reinterpret_cast<const uint8_t*>(kNulBytes),
                              kNulByteCount)) {
    return check;
  }
  ++check;
  char scratch[8] = {};
  if (Read(nullptr, 0) != 0 || Read(scratch, -1) != kSyscallErrorInvalid ||
      Read(scratch, kSyscallReadMaxSize + 1) != kSyscallErrorInvalid ||
      Read(nullptr, -1) != kSyscallErrorInvalid ||
      Read(nullptr, 8) != kSyscallErrorFault ||
      Read(reinterpret_cast<void*>(usertest::kKernelAddress), 8) !=
          kSyscallErrorFault ||
      Read(const_cast<char*>(kNulBytes), 8) != kSyscallErrorFault ||
      Spawn(nullptr, 0, 0) != kSyscallErrorInvalid ||
      Spawn("usertest", -1, 0) != kSyscallErrorInvalid ||
      Spawn("usertest", kSyscallSpawnMaxNameLength + 1, 0) !=
          kSyscallErrorInvalid ||
      Spawn("no_such_program", 15, 0) != kSyscallErrorInvalid ||
      Spawn(reinterpret_cast<const char*>(usertest::kKernelAddress), 4, 0) !=
          kSyscallErrorFault ||
      Wait(0).status() != kSyscallErrorInvalid ||
      Wait(-1).status() != kSyscallErrorInvalid ||
      Wait(kMaxUserChildProcesses + 1).status() != kSyscallErrorInvalid ||
      Wait(1).status() != kSyscallErrorInvalid) {
    return check;
  }
  ++check;
  const SegmentSelectors before_dirty = ReadDataSelectors();
  if (before_dirty.ds != kUserDataSelector ||
      before_dirty.es != kUserDataSelector ||
      before_dirty.fs != kUserDataSelector ||
      before_dirty.gs != kUserDataSelector) {
    return check;
  }
  // Load distinct combinations of null (`0x00`) and DPL-3 (`0x23`) selectors
  // across DS/ES/FS/GS so every register has a unique signature across two
  // preemptible syscalls, then leave all four zeroed on exit so the next
  // process verifies `UserModeEnter` resets all four to `0x23`.
  constexpr SegmentSelectors kPatterns[] = {
      {.ds = 0, .es = kUserDataSelector, .fs = 0, .gs = kUserDataSelector},
      {.ds = 0, .es = 0, .fs = kUserDataSelector, .gs = kUserDataSelector},
      {.ds = 0, .es = 0, .fs = 0, .gs = 0},
  };
  const uint64_t expected_checksum = SyscallSpinChecksum(
      reinterpret_cast<const uint8_t*>(kNulBytes), kNulByteCount);
  for (const SegmentSelectors& pat : kPatterns) {
    WriteDataSelectors(pat);
    if (SpinOnBuffer(kNulBytes, kNulByteCount, 1) !=
        static_cast<int64_t>(expected_checksum)) {
      return check;
    }
    const SegmentSelectors got = ReadDataSelectors();
    if (got.ds != pat.ds || got.es != pat.es || got.fs != pat.fs ||
        got.gs != pat.gs) {
      return check;
    }
  }
  return 0;
}

[[gnu::noinline]] static uint64_t Recurse(const uint64_t depth) {
  volatile uint8_t frame[256];
  frame[0] = static_cast<uint8_t>(depth);
  if (g_recursion_limit != 0 && depth >= g_recursion_limit) {
    return frame[0];
  }
  return Recurse(depth + 1) + frame[0];
}

static uint64_t XorShift(uint64_t x) {
  x ^= x << 13;
  x ^= x >> 7;
  x ^= x << 17;
  return x;
}

static int64_t Spin(const uint64_t seed) {
  g_spin_seed = seed;
  uint64_t in_register = seed | 1;
  volatile uint64_t on_stack = seed | 1;
  for (;;) {
    // Each system call reloads `g_spin_seed` (the wrapper clobbers memory).
    GetCpu();
    for (int i = 0; i < usertest::kSpinRoundIterations; ++i) {
      in_register = XorShift(in_register);
      on_stack = XorShift(on_stack);
    }
    if (in_register != on_stack || g_spin_seed != seed) {
      return usertest::kSpinCorruptedStatus;
    }
  }
}

// Fills the `size` bytes at `buffer` with values derived from `state`, and
// returns the next state.
static uint64_t FillBuffer(uint8_t* const buffer,  //
                           const int64_t size,     //
                           uint64_t state) {
  for (int64_t i = 0; i < size; ++i) {
    state = XorShift(state);
    buffer[i] = static_cast<uint8_t>(state >> 24);
  }
  return state;
}

// Runs `Mode::kKernelSpin` as `arg` (`usertest::KernelSpinArg()`) says.
static int64_t KernelSpin(const uint64_t arg) {
  const int64_t ticks = usertest::KernelSpinTicks(arg);
  const int64_t calls = usertest::KernelSpinCalls(arg);
  // Odd, because XorShift() must not start from 0.
  const uint64_t seed = (usertest::KernelSpinSeed(arg) << 1) | 1;
  // Registers of another process, with another seed, fail the check.
  const uint64_t register_seed = seed * kRegisterSeedMultiplier;
  g_spin_seed = seed;
  uint64_t state = seed;
  uint64_t in_register = seed;
  volatile uint64_t on_stack = seed;
  for (int64_t call = 0; calls == 0 || call < calls; ++call) {
    state = XorShift(state);
    const int64_t offset =
        static_cast<int64_t>(state % static_cast<uint64_t>(kPageSize));
    const int64_t size = static_cast<int64_t>(
        (state >> 12) % static_cast<uint64_t>(kSyscallSpinMaxSize + 1));
    uint8_t* const buffer = g_kernel_spin_buffer + offset;
    state = FillBuffer(buffer, size, state);
    const int64_t checksum = SyscallSpinChecksum(buffer, size);
    int64_t result = 0;
    if (SyscallSpinChangedRegisterBits(buffer,   //
                                       size,     //
                                       ticks,    //
                                       &result,  //
                                       register_seed) != 0) {
      return usertest::kKernelSpinRegistersChangedStatus;
    }
    if (result < 0 && result >= -4095) {
      return usertest::kKernelSpinErrorStatus;
    }
    if (result != checksum) {
      return usertest::kKernelSpinChecksumStatus;
    }
    if (SyscallSpinChecksum(buffer, size) != checksum) {
      return usertest::kKernelSpinBufferChangedStatus;
    }
    in_register = XorShift(in_register);
    on_stack = XorShift(on_stack);
    if (in_register != on_stack || *Launder(&g_spin_seed) != seed) {
      return usertest::kKernelSpinCorruptedStatus;
    }
  }
  return 0;
}

// Runs `Mode::kSpinWithoutSyscalls` with `seed`.
static int64_t SpinWithoutSyscalls(const uint64_t seed) {
  // Registers of another process, with another seed, fail the check.
  const uint64_t register_seed = seed * kRegisterSeedMultiplier;
  g_spin_seed = seed;
  uint64_t in_register = seed | 1;
  volatile uint64_t on_stack = seed | 1;
  for (;;) {
    if (SpinChangedRegisterBits(kQuietSpinIterations, register_seed) != 0) {
      return usertest::kSpinRegistersChangedStatus;
    }
    for (int i = 0; i < usertest::kSpinRoundIterations; ++i) {
      in_register = XorShift(in_register);
      on_stack = XorShift(on_stack);
    }
    // Launder() makes the compiler load `g_spin_seed` from memory.
    if (in_register != on_stack || *Launder(&g_spin_seed) != seed) {
      return usertest::kSpinCorruptedStatus;
    }
  }
}

// The layout of an XSAVE area in the standard format, whose first 512 bytes
// (the legacy region) have the layout of an FXSAVE area (Intel SDM Vol. 1,
// 10.5.1 and 13.4.1). The x87 control word, status word and abridged tag word
// (bit i set: physical register i is valid).
constexpr int kFpuFcwOffset = 0;
constexpr int kFpuFswOffset = 2;
constexpr int kFpuFtwOffset = 4;
// The last x87 opcode, instruction pointer and operand pointer.
constexpr int kFpuLastInstructionOffset = 6;
constexpr int kFpuLastInstructionSize = 18;
constexpr int kFpuMxcsrOffset = 24;
// ST0-ST7 (MM0-MM7 in their low 8 bytes): 10 bytes in 16-byte slots.
constexpr int kFpuStOffset = 32;
constexpr int kFpuStSlotSize = 16;
constexpr int kFpuStSize = 10;
// XMM0-XMM15.
constexpr int kFpuXmmOffset = 160;
constexpr int kFpuXmmEnd = 416;
constexpr int kFpuXmmSize = kFpuXmmEnd - kFpuXmmOffset;
// XSTATE_BV in the XSAVE header: the state components that the area holds;
// the others are in their initial configuration.
constexpr int kFpuXstateBvOffset = 512;
// The AVX state, the upper halves of YMM0-YMM15, lies at the offset that
// CPUID.(EAX=0DH,ECX=2):EBX reports (YmmOffset()).
constexpr int kFpuYmmSize = 256;
constexpr int kFpuAreaSize = protos::user::kInitialFpuStateSize;

// XCR0 state components.
constexpr uint64_t kXstateX87 = 1 << 0;
constexpr uint64_t kXstateSse = 1 << 1;
constexpr uint64_t kXstateAvx = 1 << 2;
// Set if the kernel set CR4.OSXSAVE, which lets user code use XGETBV.
constexpr uint32_t kCpuid1EcxOsxsave = 1u << 27;
// The architectural initial x87 control word and MXCSR.
constexpr uint16_t kInitialFcw = 0x037F;
constexpr uint32_t kInitialMxcsr = 0x1F80;

// An XSAVE or FXSAVE area that holds the x87, SSE and AVX state.
struct alignas(64) FpuArea {
  uint8_t bytes[kFpuAreaSize];
};

// What `kFpuHold` and `kFpuDirty` load (`g_fpu_image`, and in the MMX rounds
// of `kFpuHold` also `g_fpu_mmx_values`) and save.
FpuArea g_fpu_image;
uint64_t g_fpu_mmx_values[8];
FpuArea g_fpu_before;
FpuArea g_fpu_after;

// Returns the `size` bytes at `bytes` as a little-endian number.
static uint64_t LoadLittleEndian(const uint8_t* const bytes, const int size) {
  uint64_t value = 0;
  for (int i = size - 1; i >= 0; --i) {
    value = (value << 8) | bytes[i];
  }
  return value;
}

// Stores the low `size` bytes of `value` at `bytes`, least significant first.
static void StoreLittleEndian(uint8_t* const bytes,  //
                              uint64_t value,        //
                              const int size) {
  for (int i = 0; i < size; ++i) {
    bytes[i] = static_cast<uint8_t>(value);
    value >>= 8;
  }
}

static void ZeroBytes(uint8_t* const bytes, const int size) {
  for (int i = 0; i < size; ++i) {
    bytes[i] = 0;
  }
}

static bool AllZero(const uint8_t* const bytes, const int size) {
  for (int i = 0; i < size; ++i) {
    if (bytes[i] != 0) {
      return false;
    }
  }
  return true;
}

// Executes CPUID with `leaf` in EAX and `subleaf` in ECX, and stores EAX, EBX,
// ECX and EDX in `regs[0..3]`.
static void Cpuid(const uint32_t leaf,     //
                  const uint32_t subleaf,  //
                  uint32_t* const regs) {
  asm volatile("cpuid"
               : "=a"(regs[0]), "=b"(regs[1]), "=c"(regs[2]), "=d"(regs[3])
               : "a"(leaf), "c"(subleaf));
}

// Returns XCR0, the state components that the kernel enabled for XSAVE, or 0
// if it did not enable XSAVE.
static uint64_t EnabledXstateComponents() {
  uint32_t regs[4];
  Cpuid(1, 0, regs);
  if ((regs[2] & kCpuid1EcxOsxsave) == 0) {
    return 0;
  }
  uint32_t low = 0;
  uint32_t high = 0;
  asm volatile("xgetbv" : "=a"(low), "=d"(high) : "c"(0));
  return (static_cast<uint64_t>(high) << 32) | low;
}

// Returns the state components that the FPU modes load and save with XRSTOR
// and XSAVE, or 0 if they use FXRSTOR and FXSAVE: the x87, SSE and AVX
// components that the kernel enabled, like the entry point (user/crt.cpp).
static uint64_t FpuComponents() {
  return EnabledXstateComponents() & (kXstateX87 | kXstateSse | kXstateAvx);
}

// Returns the offset of the AVX state in an XSAVE area, or 0 if it doesn't
// fit in an `FpuArea` after the legacy region and the header.
static int YmmOffset() {
  uint32_t regs[4];
  Cpuid(0xD, 2, regs);
  const int64_t offset = regs[1];
  return (offset >= 576 && offset + kFpuYmmSize <= kFpuAreaSize)
             ? static_cast<int>(offset)
             : 0;
}

// Writes the state that round `round` of `kFpuHold` with `seed` loads (and
// `kFpuDirty`, with round 0) into `*image`, in the format that `components`
// selects (FpuComponents()), and the values that `kFpuHold` loads into
// MM0-MM7 in its MMX rounds into `mmx_values[0..7]`. The values depend on the
// seed and the round: they differ from register to register and from one
// round to the next, and none of the registers holds its initial value.
static void BuildFpuImage(const uint64_t seed,        //
                          const int64_t round,        //
                          const uint64_t components,  //
                          const int ymm_offset,       //
                          FpuArea* const image,       //
                          uint64_t* const mmx_values) {
  uint8_t* const bytes = image->bytes;
  ZeroBytes(bytes, kFpuAreaSize);
  // The control fields follow `control`, which changes every round; the
  // registers take random values from both the seed and the round.
  const uint64_t round_bits = static_cast<uint64_t>(round);
  const uint64_t control = seed + round_bits;
  const uint64_t round_mix = round_bits * 0xBF58476D1CE4E5B9ULL;
  uint64_t random = ((seed * kRegisterSeedMultiplier) ^ round_mix) | 1;
  // All x87 exceptions masked; precision single, double or extended (another
  // one every round); rounding never to nearest, so the control word is never
  // the initial 0x37F.
  constexpr uint64_t kPrecisions[3] = {0, 2, 3};
  const uint64_t precision = kPrecisions[control % 3];
  const uint64_t rounding = 1 + (control / 3) % 3;
  const uint64_t fcw = 0x007F | (precision << 8) | (rounding << 10);
  StoreLittleEndian(bytes + kFpuFcwOffset, fcw, 2);
  // Condition codes C0-C3 (never all clear), TOP 0, no exceptions.
  const uint64_t conditions = 1 + control % 15;
  const uint64_t fsw = ((conditions & 7) << 8) | ((conditions >> 3) << 14);
  StoreLittleEndian(bytes + kFpuFswOffset, fsw, 2);
  // All eight registers valid.
  bytes[kFpuFtwOffset] = 0xFF;
  // All SIMD exceptions masked, some exception flags, rounding never to
  // nearest, flush to zero in every other round; never denormals-are-zero,
  // which MXCSR_MASK may not allow.
  const uint64_t mxcsr = kInitialMxcsr | (control & 0x3F) |
                         ((1 + control % 3) << 13) | ((control & 1) << 15);
  StoreLittleEndian(bytes + kFpuMxcsrOffset, mxcsr, 4);
  for (int i = 0; i < 8; ++i) {
    random = XorShift(random);
    uint8_t* const st = bytes + kFpuStOffset + i * kFpuStSlotSize;
    // A normal 80-bit number: integer bit set, exponent neither 0 nor 0x7FFF.
    StoreLittleEndian(st, random | (uint64_t{1} << 63), 8);
    const uint64_t sign = (random >> 7) & 0x8000;
    const uint64_t exponent = 0x3000 + i * 0x101 + (control & 0xFF);
    StoreLittleEndian(st + 8, sign | exponent, 2);
  }
  for (int i = kFpuXmmOffset; i < kFpuXmmEnd; i += 8) {
    random = XorShift(random);
    StoreLittleEndian(bytes + i, random, 8);
  }
  if ((components & kXstateAvx) != 0) {
    for (int i = 0; i < kFpuYmmSize; i += 8) {
      random = XorShift(random);
      StoreLittleEndian(bytes + ymm_offset + i, random, 8);
    }
  }
  // XRSTOR loads every component that XSTATE_BV names from the image.
  StoreLittleEndian(bytes + kFpuXstateBvOffset, components, 8);
  for (int i = 0; i < 8; ++i) {
    random = XorShift(random);
    mmx_values[i] = random;
  }
}

// Returns the `kFpuCorrupted*Status` for a difference at `offset` in an
// `FpuArea`.
static int64_t FpuCorruptionStatus(const int offset) {
  if (offset < kFpuMxcsrOffset) {
    return usertest::kFpuCorruptedX87ControlStatus;
  }
  if (offset < kFpuStOffset) {
    return usertest::kFpuCorruptedMxcsrStatus;
  }
  if (offset < kFpuXmmOffset) {
    return usertest::kFpuCorruptedX87RegistersStatus;
  }
  if (offset < kFpuXmmEnd) {
    return usertest::kFpuCorruptedXmmStatus;
  }
  return usertest::kFpuCorruptedYmmStatus;
}

// Returns 0 if bytes `[offset, offset + size)` of `a` and `b` are equal, else
// the `kFpuCorrupted*Status` of the first difference.
static int64_t CompareFpuBytes(const FpuArea& a,  //
                               const FpuArea& b,  //
                               const int offset,  //
                               const int size) {
  for (int i = offset; i < offset + size; ++i) {
    if (a.bytes[i] != b.bytes[i]) {
      return FpuCorruptionStatus(i);
    }
  }
  return 0;
}

// Returns 0 if `a` and `b` name the same state components in XSTATE_BV, else
// the `kFpuCorrupted*Status` of a component that only one of them names.
static int64_t CompareXstateBv(const FpuArea& a, const FpuArea& b) {
  const uint64_t differences =
      LoadLittleEndian(a.bytes + kFpuXstateBvOffset, 8) ^
      LoadLittleEndian(b.bytes + kFpuXstateBvOffset, 8);
  if ((differences & kXstateX87) != 0) {
    return usertest::kFpuCorruptedX87RegistersStatus;
  }
  if ((differences & kXstateSse) != 0) {
    return usertest::kFpuCorruptedXmmStatus;
  }
  return (differences != 0) ? usertest::kFpuCorruptedYmmStatus : 0;
}

// Checks one round of `kFpuHold`: `before` must hold the values that the
// round loaded (`image`, with `mmx_values` in the low 8 bytes of ST0-ST7
// unless it is nullptr), and `after` everything that `before` holds,
// byte for byte, including fields whose contents depend on the CPU (the MMX
// exponents and the last x87 instruction pointers). `components` and
// `ymm_offset` are those that built the image. Returns 0, or the
// `kFpuCorrupted*Status` of the first difference.
static int64_t CheckFpuHoldRound(const FpuArea& image,              //
                                 const uint64_t* const mmx_values,  //
                                 const FpuArea& before,             //
                                 const FpuArea& after,              //
                                 const uint64_t components,         //
                                 const int ymm_offset) {
  // MMX instructions leave their values in the low 8 bytes of the x87
  // registers, all registers valid and TOP 0, and set the exponents.
  FpuArea expected = image;
  int loaded_st_size = kFpuStSize;
  if (mmx_values != nullptr) {
    loaded_st_size = 8;
    for (int i = 0; i < 8; ++i) {
      uint8_t* const st = expected.bytes + kFpuStOffset + i * kFpuStSlotSize;
      StoreLittleEndian(st, mmx_values[i], 8);
    }
  }
  const bool avx = (components & kXstateAvx) != 0;
  // The loaded values: the x87 control, status and tag words, MXCSR, ST0-ST7,
  // XMM0-XMM15 and the upper halves of YMM0-YMM15.
  int64_t status = CompareFpuBytes(expected, before, kFpuFcwOffset, 5);
  if (status == 0) {
    status = CompareFpuBytes(expected, before, kFpuMxcsrOffset, 4);
  }
  for (int i = 0; i < 8 && status == 0; ++i) {
    const int st_offset = kFpuStOffset + i * kFpuStSlotSize;
    status = CompareFpuBytes(expected, before, st_offset, loaded_st_size);
  }
  if (status == 0) {
    status = CompareFpuBytes(expected, before, kFpuXmmOffset, kFpuXmmSize);
  }
  if (status == 0 && avx) {
    const uint64_t saved =
        LoadLittleEndian(before.bytes + kFpuXstateBvOffset, 8);
    status = ((saved & kXstateAvx) != 0)
                 ? CompareFpuBytes(expected, before, ymm_offset, kFpuYmmSize)
                 : usertest::kFpuCorruptedYmmStatus;
  }
  // Everything that the second save stored: the legacy region up to MXCSR
  // and MXCSR_MASK, ST0-ST7, XMM0-XMM15, XSTATE_BV and the AVX state.
  if (status == 0) {
    status = CompareFpuBytes(before, after, 0, kFpuStOffset);
  }
  for (int i = 0; i < 8 && status == 0; ++i) {
    const int st_offset = kFpuStOffset + i * kFpuStSlotSize;
    status = CompareFpuBytes(before, after, st_offset, kFpuStSize);
  }
  if (status == 0) {
    status = CompareFpuBytes(before, after, kFpuXmmOffset, kFpuXmmSize);
  }
  if (status == 0 && components != 0) {
    status = CompareXstateBv(before, after);
  }
  if (status == 0 && avx) {
    status = CompareFpuBytes(before, after, ymm_offset, kFpuYmmSize);
  }
  return status;
}

// Runs `Mode::kFpuHold` with `seed`.
static int64_t FpuHold(const uint64_t seed) {
  const uint64_t components = FpuComponents();
  const int ymm_offset = YmmOffset();
  if ((components & kXstateAvx) != 0 && ymm_offset == 0) {
    return usertest::kFpuLayoutUnsupportedStatus;
  }
  for (int64_t round = 0;; ++round) {
    // New values every round, so that registers that the kernel restored
    // from a save of an earlier round can't pass the check.
    BuildFpuImage(seed,          //
                  round,         //
                  components,    //
                  ymm_offset,    //
                  &g_fpu_image,  //
                  g_fpu_mmx_values);
    // Odd rounds put MMX values into the x87 registers.
    const uint64_t* const mmx_values =
        ((round & 1) != 0) ? g_fpu_mmx_values : nullptr;
    ZeroBytes(g_fpu_before.bytes, kFpuAreaSize);
    ZeroBytes(g_fpu_after.bytes, kFpuAreaSize);
    const FpuHoldParams params = {
        .image = &g_fpu_image,
        .mmx_values = mmx_values,
        .before = &g_fpu_before,
        .after = &g_fpu_after,
        .components = components,
        .syscalls = usertest::kFpuHoldSyscalls,
        .iterations = usertest::kFpuHoldIterations,
    };
    FpuHoldRound(&params);
    const int64_t status = CheckFpuHoldRound(g_fpu_image,   //
                                             mmx_values,    //
                                             g_fpu_before,  //
                                             g_fpu_after,   //
                                             components,    //
                                             ymm_offset);
    if (status != 0) {
      return status;
    }
  }
}

// Runs `Mode::kFpuDirty` with `seed`.
static int64_t FpuDirty(const uint64_t seed) {
  const uint64_t components = FpuComponents();
  const int ymm_offset = YmmOffset();
  if ((components & kXstateAvx) != 0 && ymm_offset == 0) {
    return usertest::kFpuLayoutUnsupportedStatus;
  }
  BuildFpuImage(seed,          //
                /*round=*/0,   //
                components,    //
                ymm_offset,    //
                &g_fpu_image,  //
                g_fpu_mmx_values);
  const FpuHoldParams params = {
      .image = &g_fpu_image,
      .mmx_values = nullptr,
      .before = &g_fpu_before,
      .after = &g_fpu_after,
      .components = components,
      .syscalls = INT64_MAX,
      .iterations = usertest::kFpuHoldIterations,
  };
  // Returns only after 2^63 - 1 system calls.
  FpuHoldRound(&params);
  return -1;
}

// Runs the checks of `Mode::kFpuInitialState` on `g_initial_fpu_state` and
// returns 0 if all pass, else the number of the first failed check. XSAVE
// leaves out a state component in its initial configuration (XSTATE_BV bit
// clear), whose fields then need no check; MXCSR it always saves.
static int64_t CheckInitialFpuState() {
  const uint8_t* const state = g_initial_fpu_state;
  const uint64_t components = FpuComponents();
  const uint64_t saved = (components == 0)
                             ? (kXstateX87 | kXstateSse)
                             : LoadLittleEndian(state + kFpuXstateBvOffset, 8);
  const bool x87 = (saved & kXstateX87) != 0;
  const bool sse = (saved & kXstateSse) != 0;
  const bool avx = (components & kXstateAvx) != 0 && (saved & kXstateAvx) != 0;
  if (x87 && LoadLittleEndian(state + kFpuFcwOffset, 2) != kInitialFcw) {
    return 1;
  }
  if (x87 && LoadLittleEndian(state + kFpuFswOffset, 2) != 0) {
    return 2;
  }
  // All registers empty.
  if (x87 && state[kFpuFtwOffset] != 0) {
    return 3;
  }
  if (x87 &&
      !AllZero(state + kFpuLastInstructionOffset, kFpuLastInstructionSize)) {
    return 4;
  }
  for (int i = 0; i < 8; ++i) {
    if (x87 &&
        !AllZero(state + kFpuStOffset + i * kFpuStSlotSize, kFpuStSize)) {
      return 5;
    }
  }
  if (LoadLittleEndian(state + kFpuMxcsrOffset, 4) != kInitialMxcsr) {
    return 6;
  }
  if (sse && !AllZero(state + kFpuXmmOffset, kFpuXmmSize)) {
    return 7;
  }
  if (avx) {
    const int ymm_offset = YmmOffset();
    if (ymm_offset == 0 || !AllZero(state + ymm_offset, kFpuYmmSize)) {
      return 8;
    }
  }
  return 0;
}

// RFLAGS.AC (alignment check), which user code may set and clear. The kernel
// leaves CR0.AM clear, so the flag has no effect in user mode, and preserves
// it across system calls like the other flags (syscall_abi.h).
constexpr uint64_t kRflagsAlignmentCheck = uint64_t{1} << 18;
// Neither a user nor a kernel address: between the halves of the address
// space.
constexpr uintptr_t kNonCanonicalAddress = 0x0000800000000000ULL;
// The start of the user half (paging.h), where usertest maps nothing.
constexpr uintptr_t kUnmappedUserAddress = 0xFFFF800000000000ULL;
// The lowest page of the stack, and the unmapped guard page below it.
constexpr uintptr_t kStackBottom =
    usertest::kStackTop - usertest::kStackPages * kPageSize;
constexpr uintptr_t kStackGuardPage = kStackBottom - kPageSize;
// The rounds of computation of `kBufferChecks` with RFLAGS.AC set, and the
// iterations of each: about 8 timer ticks of computation under QEMU's TCG,
// almost all of the program's time in user mode, so that timer interrupts
// arrive while the flag is set (the boot self-test checks that some did).
constexpr int kAlignmentCheckRounds = 512;
constexpr int64_t kAlignmentCheckIterations = int64_t{1} << 16;
// What `kFpuLongSyscall` adds to its seed for the values of its x87/SSE/AVX
// registers, so that they differ from those of the `kFpuHold` processes of
// the boot self-tests, whose seeds are below 2^32.
constexpr uint64_t kFpuLongSyscallSeedBase = uint64_t{0x10C5} << 40;
// The x87 status word bits: zero-divide exception, and exception summary
// (an unmasked exception is pending).
constexpr uint16_t kX87StatusZeroDivide = 1 << 2;
constexpr uint16_t kX87StatusErrorSummary = 1 << 7;

// The 4096 NUL bytes that `kBufferChecks` writes from its data.
const char g_nul_page[kSyscallWriteMaxSize] = {};

// Returns true if a spin() call of `ticks` timer ticks on the `size` bytes at
// `buffer` returns their checksum.
static bool SpinMatches(const uint8_t* const buffer,  //
                        const int64_t size,           //
                        const int64_t ticks) {
  return SpinOnBuffer(buffer, size, ticks) == SyscallSpinChecksum(buffer, size);
}

static bool AlignmentCheckFlagSet() {
  return (ReadRflags() & kRflagsAlignmentCheck) != 0;
}

static void SetAlignmentCheckFlag() {
  asm volatile("pushfq\n\tor dword ptr [rsp], 0x40000\n\tpopfq"
               :
               :
               : "memory", "cc");
}

static void ClearAlignmentCheckFlag() {
  asm volatile("pushfq\n\tand dword ptr [rsp], 0xFFFBFFFF\n\tpopfq"
               :
               :
               : "memory", "cc");
}

// Runs the checks of `Mode::kBufferChecks` and returns 0 if all pass, else
// the number of the first failed check.
static int64_t RunBufferChecks() {
  const void* const kernel_address =
      reinterpret_cast<const void*>(usertest::kKernelAddress);
  int64_t check = 1;
  if (SpinOnBuffer(nullptr, 8, 0) != kSyscallErrorFault) {
    return check;
  }
  ++check;
  // Mapped, but for the kernel only: the kernel must not read it for the
  // program.
  if (!SpinFaults(usertest::kKernelAddress, 8)) {
    return check;
  }
  ++check;
  if (!SpinFaults(kNonCanonicalAddress, 8) ||
      Write(reinterpret_cast<const void*>(kNonCanonicalAddress), 8) !=
          kSyscallErrorFault) {
    return check;
  }
  ++check;
  if (!SpinFaults(kUnmappedUserAddress, 8)) {
    return check;
  }
  ++check;
  if (!SpinFaults(0xFFFFFFFFFFFFFFF8ULL, 16)) {
    return check;
  }
  ++check;
  // From the top bytes of the stack into the unmapped page above.
  if (!SpinFaults(usertest::kStackTop - 4, 8) ||
      !SpinFaults(usertest::kStackTop, 1)) {
    return check;
  }
  ++check;
  // In the guard page, and from its end into the stack.
  if (!SpinFaults(kStackGuardPage, 8) || !SpinFaults(kStackBottom - 8, 16)) {
    return check;
  }
  ++check;
  // Invalid sizes and ticks fail as invalid, even with a buffer that user
  // code may not read.
  if (SpinOnBuffer(kNulBytes, -1, 0) != kSyscallErrorInvalid ||
      SpinOnBuffer(kNulBytes, INT64_MIN, 0) != kSyscallErrorInvalid ||
      SpinOnBuffer(kNulBytes, kSyscallSpinMaxSize + 1, 0) !=
          kSyscallErrorInvalid ||
      SpinOnBuffer(kNulBytes, kNulByteCount, -1) != kSyscallErrorInvalid ||
      SpinOnBuffer(kNulBytes, kNulByteCount, kSyscallSpinMaxTicks + 1) !=
          kSyscallErrorInvalid ||
      SpinOnBuffer(nullptr, -1, 0) != kSyscallErrorInvalid ||
      SpinOnBuffer(kernel_address, 8, -1) != kSyscallErrorInvalid) {
    return check;
  }
  ++check;
  // Read-only data, the program's code, an odd address, ranges across a page
  // boundary (the longest one too), the lowest and the highest bytes of the
  // stack, and no bytes at a kernel address, which the kernel must not read.
  FillBuffer(g_kernel_spin_buffer, 2 * kPageSize, 0x0B0F'FE45);
  const uint8_t* const read_only =
      reinterpret_cast<const uint8_t*>(&kReadOnlyWord);
  const uint8_t* const code =
      reinterpret_cast<const uint8_t*>(&SyscallChangedRegisterBits);
  const uint8_t* const stack_bottom =
      reinterpret_cast<const uint8_t*>(kStackBottom);
  const uint8_t* const stack_top =
      reinterpret_cast<const uint8_t*>(usertest::kStackTop - 8);
  if (!SpinMatches(read_only, sizeof(kReadOnlyWord), 0) ||
      !SpinMatches(code, 64, 0) ||
      !SpinMatches(g_kernel_spin_buffer + 1, 7, 0) ||
      !SpinMatches(g_kernel_spin_buffer + kPageSize - 3, 6, 1) ||
      !SpinMatches(g_kernel_spin_buffer + 1, kSyscallSpinMaxSize, 0) ||
      !SpinMatches(stack_bottom, 8, 0) || !SpinMatches(stack_top, 8, 0) ||
      SpinOnBuffer(kernel_address, 0, 0) != kSyscallSpinChecksumInit) {
    return check;
  }
  ++check;
  // write() leaves out NUL bytes, so these print nothing.
  char on_stack[kSyscallWriteMaxSize];
  ZeroBytes(reinterpret_cast<uint8_t*>(on_stack), kSyscallWriteMaxSize);
  if (Write(g_nul_page, kSyscallWriteMaxSize) != kSyscallWriteMaxSize ||
      Write(Launder(on_stack), kSyscallWriteMaxSize) != kSyscallWriteMaxSize) {
    return check;
  }
  ++check;
  if (Write(nullptr, kNulByteCount) != kSyscallErrorFault ||
      Read(nullptr, kNulByteCount) != kSyscallErrorFault ||
      Read(const_cast<uint64_t*>(&kReadOnlyWord), sizeof(kReadOnlyWord)) !=
          kSyscallErrorFault ||
      Read(reinterpret_cast<void*>(&SyscallChangedRegisterBits), 8) !=
          kSyscallErrorFault ||
      Read(nullptr, -1) != kSyscallErrorInvalid) {
    return check;
  }
  ++check;
  // Failing calls preserve every register but rax too.
  int64_t fault_result = 0;
  int64_t invalid_result = 0;
  const uint64_t fault_changed =
      SyscallSpinChangedRegisterBits(kernel_address, 8, 0, &fault_result, 0x12);
  const uint64_t invalid_changed =
      SyscallSpinChangedRegisterBits(kNulBytes, -1, 0, &invalid_result, 0x34);
  if (fault_changed != 0 || fault_result != kSyscallErrorFault ||
      invalid_changed != 0 || invalid_result != kSyscallErrorInvalid) {
    return check;
  }
  ++check;
  // RFLAGS.AC stays set across system calls, also one that lasts 2 timer
  // ticks, and across the timer interrupts during computation in user mode.
  SetAlignmentCheckFlag();
  bool kept = AlignmentCheckFlagSet();
  kept = kept && SpinMatches(g_kernel_spin_buffer + 5, 100, 2) &&
         AlignmentCheckFlagSet();
  kept = kept && Write(kNulBytes, kNulByteCount) == kNulByteCount &&
         AlignmentCheckFlagSet();
  const int cpu = GetCpu();
  kept = kept && cpu >= 0 && cpu < kMaxCpus && AlignmentCheckFlagSet();
  for (int i = 0; i < kAlignmentCheckRounds && kept; ++i) {
    kept = SpinChangedRegisterBits(kAlignmentCheckIterations,
                                   static_cast<uint64_t>(i)) == 0 &&
           AlignmentCheckFlagSet();
  }
  ClearAlignmentCheckFlag();
  if (!kept || AlignmentCheckFlagSet()) {
    return check;
  }
  static_assert(usertest::kBufferCheckCount == 13);
  return 0;
}

// Runs `Mode::kFpuLongSyscall` as `arg` (`usertest::FpuLongSyscallArg()`)
// says.
static int64_t FpuLongSyscall(const uint64_t arg) {
  const int64_t ticks = usertest::KernelSpinTicks(arg);
  const int64_t rounds = usertest::KernelSpinCalls(arg);
  const uint64_t seed = usertest::KernelSpinSeed(arg);
  const uint64_t components = FpuComponents();
  const int ymm_offset = YmmOffset();
  if ((components & kXstateAvx) != 0 && ymm_offset == 0) {
    return usertest::kFpuLayoutUnsupportedStatus;
  }
  // Odd, because XorShift() must not start from 0.
  uint64_t state = (seed << 1) | 1;
  // Registers of another process, with another seed, fail the check.
  const uint64_t register_seed = state * kRegisterSeedMultiplier;
  for (int64_t round = 0; rounds == 0 || round < rounds; ++round) {
    BuildFpuImage(kFpuLongSyscallSeedBase + seed,  //
                  round,                           //
                  components,                      //
                  ymm_offset,                      //
                  &g_fpu_image,                    //
                  g_fpu_mmx_values);
    // Odd rounds put MMX values into the x87 registers.
    const uint64_t* const mmx_values =
        ((round & 1) != 0) ? g_fpu_mmx_values : nullptr;
    state = XorShift(state);
    const int64_t offset =
        static_cast<int64_t>(state % static_cast<uint64_t>(kPageSize));
    const int64_t size =
        1 + static_cast<int64_t>((state >> 12) %
                                 static_cast<uint64_t>(kSyscallSpinMaxSize));
    uint8_t* const buffer = g_kernel_spin_buffer + offset;
    state = FillBuffer(buffer, size, state);
    const int64_t checksum = SyscallSpinChecksum(buffer, size);
    ZeroBytes(g_fpu_before.bytes, kFpuAreaSize);
    ZeroBytes(g_fpu_after.bytes, kFpuAreaSize);
    FpuSpinParams params = {
        .image = &g_fpu_image,
        .mmx_values = mmx_values,
        .before = &g_fpu_before,
        .after = &g_fpu_after,
        .components = components,
        .buffer = buffer,
        .size = size,
        .ticks = ticks,
        .register_seed = register_seed + static_cast<uint64_t>(round),
        .result = 0,
        .changed_bits = 0,
    };
    FpuSpinRound(&params);
    if (params.changed_bits != 0) {
      return usertest::kFpuLongSyscallRegistersChangedStatus;
    }
    if (params.result < 0 && params.result >= -4095) {
      return usertest::kFpuLongSyscallErrorStatus;
    }
    if (params.result != checksum) {
      return usertest::kFpuLongSyscallChecksumStatus;
    }
    if (SyscallSpinChecksum(buffer, size) != checksum) {
      return usertest::kFpuLongSyscallBufferChangedStatus;
    }
    const int64_t status = CheckFpuHoldRound(g_fpu_image,   //
                                             mmx_values,    //
                                             g_fpu_before,  //
                                             g_fpu_after,   //
                                             components,    //
                                             ymm_offset);
    if (status == usertest::kFpuCorruptedX87RegistersStatus &&
        mmx_values != nullptr) {
      return usertest::kFpuLongSyscallMmxStatus;
    }
    if (status != 0) {
      return status;
    }
  }
  return 0;
}

// Runs `Mode::kX87PendingException` with rounds of at least `round_cycles`
// TSC cycles (`usertest::X87PendingArg()`).
static int64_t X87PendingException(const int64_t round_cycles) {
  uint16_t status_word = 0;
  asm volatile("fnstsw %0" : "=a"(status_word));
  if (status_word != 0) {
    return usertest::kX87PendingInitialStatusWordStatus;
  }
  X87PendingZeroDivide(&status_word, usertest::kX87PendingRounds,
                       usertest::kX87PendingIterations, round_cycles);
  // FWAIT returned.
  constexpr uint16_t kPending = kX87StatusZeroDivide | kX87StatusErrorSummary;
  if ((status_word & kPending) != kPending) {
    return usertest::kX87PendingNotRaisedStatus;
  }
  return usertest::kX87PendingContinuedStatus;
}

// Runs `Mode::kConsoleRead`: if `expected_bytes == 0`, blocks immediately in
// Read() so kill-in-syscall tests see `stats.syscalls == 1` and
// `stats.in_syscall == true`. Otherwise validates Read() error paths first,
// then reads `expected_bytes` from the console and returns their
// `SyscallSpinChecksum()`.
static int64_t RunConsoleRead(const int64_t expected_bytes) {
  uint8_t buffer[128] = {};
  if (expected_bytes <= 0) {
    (void)Read(buffer, sizeof(buffer));
    return -100;
  }
  if (expected_bytes > static_cast<int64_t>(sizeof(buffer))) {
    return -1;
  }
  // 1. Invalid sizes fail with kSyscallErrorInvalid, even with bad pointers.
  if (Read(buffer, -1) != kSyscallErrorInvalid ||
      Read(buffer, INT64_MIN) != kSyscallErrorInvalid ||
      Read(buffer, kSyscallReadMaxSize + 1) != kSyscallErrorInvalid ||
      Read(nullptr, -1) != kSyscallErrorInvalid ||
      Read(reinterpret_cast<void*>(usertest::kKernelAddress), -1) !=
          kSyscallErrorInvalid) {
    return -2;
  }
  // 2. Zero-byte read succeeds immediately with 0.
  if (Read(nullptr, 0) != 0 || Read(buffer, 0) != 0) {
    return -3;
  }
  // 3. Unmapped, kernel, non-canonical, wrapping, guard-page, and read-only
  // (.rodata and .text) pointers fail with kSyscallErrorFault.
  if (Read(nullptr, 8) != kSyscallErrorFault ||
      Read(reinterpret_cast<void*>(usertest::kKernelAddress), 8) !=
          kSyscallErrorFault ||
      Read(reinterpret_cast<void*>(kNonCanonicalAddress), 8) !=
          kSyscallErrorFault ||
      Read(reinterpret_cast<void*>(kUnmappedUserAddress), 8) !=
          kSyscallErrorFault ||
      Read(reinterpret_cast<void*>(0xFFFFFFFFFFFFFFF8ULL), 16) !=
          kSyscallErrorFault ||
      Read(reinterpret_cast<void*>(kStackGuardPage), 8) != kSyscallErrorFault ||
      Read(reinterpret_cast<void*>(usertest::kStackTop - 4), 8) !=
          kSyscallErrorFault ||
      Read(const_cast<uint64_t*>(&kReadOnlyWord), sizeof(kReadOnlyWord)) !=
          kSyscallErrorFault ||
      Read(reinterpret_cast<void*>(&SyscallChangedRegisterBits), 8) !=
          kSyscallErrorFault) {
    return -4;
  }
  // 4. Read `expected_bytes` from console input (potentially across multiple
  // chunks) and return their FNV-1a spin checksum.
  int64_t total = 0;
  while (total < expected_bytes) {
    const int64_t n = Read(buffer + total, expected_bytes - total);
    if (n <= 0 || n > expected_bytes - total) {
      return -5;
    }
    total += n;
  }
  return SyscallSpinChecksum(buffer, total);
}

// Runs `Mode::kSpawnAndWait` according to `submode` (`arg >> 8`).
static int64_t RunSpawnAndWait(const usertest::SpawnTestSubmode submode) {
  using usertest::SpawnTestSubmode;
  if (submode == SpawnTestSubmode::kBlockInWait) {
    const int64_t pid =
        Spawn("usertest", usertest::Arg(Mode::kSpinWithoutSyscalls, 0x777));
    if (pid < 1) {
      return 1;
    }
    (void)Wait(pid);
    return 2;
  }
  if (submode == SpawnTestSubmode::kOrphanOnExit) {
    const int64_t pid0 =
        Spawn("usertest", usertest::Arg(Mode::kSpinWithoutSyscalls, 0x888));
    const int64_t pid1 = Spawn(
        "usertest", usertest::KernelSpinArg(kSyscallSpinMaxTicks, 1, 0x999));
    if (pid0 < 1 || pid1 < 1 || pid0 == pid1) {
      return 1;
    }
    return 0;
  }
  if (submode == SpawnTestSubmode::kOrphanOnFault) {
    const int64_t pid =
        Spawn("usertest", usertest::Arg(Mode::kSpinWithoutSyscalls, 0xAAA));
    if (pid < 1) {
      return 1;
    }
    asm volatile("ud2");
    return 2;
  }

  // submode == SpawnTestSubmode::kBasicAndErrors:
  int64_t check = 1;
  if (Spawn(nullptr, 0, 0) != kSyscallErrorInvalid ||
      Spawn("usertest", -1, 0) != kSyscallErrorInvalid ||
      Spawn("usertest", kSyscallSpawnMaxNameLength + 1, 0) !=
          kSyscallErrorInvalid ||
      Spawn(nullptr, -1, 0) != kSyscallErrorInvalid ||
      Spawn("nonexistent_prog", 16, 0) != kSyscallErrorInvalid) {
    return check;
  }
  ++check;
  if (Spawn(nullptr, 4, 0) != kSyscallErrorFault ||
      Spawn(reinterpret_cast<const char*>(usertest::kKernelAddress), 4, 0) !=
          kSyscallErrorFault ||
      Spawn(reinterpret_cast<const char*>(kNonCanonicalAddress), 4, 0) !=
          kSyscallErrorFault ||
      Spawn(reinterpret_cast<const char*>(kUnmappedUserAddress), 4, 0) !=
          kSyscallErrorFault ||
      Spawn(reinterpret_cast<const char*>(0xFFFFFFFFFFFFFFF8ULL), 16, 0) !=
          kSyscallErrorFault ||
      Spawn(reinterpret_cast<const char*>(kStackGuardPage), 4, 0) !=
          kSyscallErrorFault) {
    return check;
  }
  ++check;
  if (Wait(0).status() != kSyscallErrorInvalid ||
      Wait(-1).status() != kSyscallErrorInvalid ||
      Wait(kMaxUserChildProcesses + 1).status() != kSyscallErrorInvalid ||
      Wait(1).status() != kSyscallErrorInvalid) {
    return check;
  }
  ++check;
  // Spawn a child that exits with `usertest::kExitStatus`.
  const int64_t exit_pid = Spawn("usertest", usertest::Arg(Mode::kExit));
  if (exit_pid < 1 || exit_pid > kMaxUserChildProcesses) {
    return check;
  }
  ++check;
  // Bad / read-only `result` pointers on a valid PID must return
  // kSyscallErrorFault without reaping the child.
  const uint64_t exit_pid_u = static_cast<uint64_t>(exit_pid);
  if (Syscall(kSyscallWait, exit_pid_u, 0) != kSyscallErrorFault ||
      Syscall(kSyscallWait, exit_pid_u, usertest::kKernelAddress) !=
          kSyscallErrorFault ||
      Syscall(kSyscallWait, exit_pid_u, kNonCanonicalAddress) !=
          kSyscallErrorFault ||
      Syscall(kSyscallWait, exit_pid_u, kUnmappedUserAddress) !=
          kSyscallErrorFault ||
      Syscall(kSyscallWait, exit_pid_u,
              reinterpret_cast<uintptr_t>(&kReadOnlyWord)) !=
          kSyscallErrorFault ||
      Syscall(kSyscallWait, exit_pid_u,
              reinterpret_cast<uintptr_t>(&SyscallChangedRegisterBits)) !=
          kSyscallErrorFault) {
    return check;
  }
  ++check;
  const StatusOr<UserProgramResult> exit_res = Wait(exit_pid);
  if (!exit_res.ok() || exit_res->outcome != UserProgramOutcome::kExited ||
      exit_res->exit_status != usertest::kExitStatus) {
    return check;
  }
  ++check;
  // Waiting a second time on an already-reaped PID must fail with
  // kSyscallErrorInvalid.
  if (Wait(exit_pid).status() != kSyscallErrorInvalid) {
    return check;
  }
  ++check;
  // Spawn a child that faults with #UD (vector 6).
  const int64_t ud_pid = Spawn("usertest", usertest::Arg(Mode::kInvalidOpcode));
  if (ud_pid < 1) {
    return check;
  }
  const StatusOr<UserProgramResult> ud_res = Wait(ud_pid);
  if (!ud_res.ok() || ud_res->outcome != UserProgramOutcome::kFaulted ||
      ud_res->fault_vector != 6) {
    return check;
  }
  ++check;
  // Spawn a child that faults with #PF (vector 14) on kKernelAddress.
  const int64_t pf_pid =
      Spawn("usertest", usertest::Arg(Mode::kReadKernelMemory));
  if (pf_pid < 1) {
    return check;
  }
  const StatusOr<UserProgramResult> pf_res = Wait(pf_pid);
  if (!pf_res.ok() || pf_res->outcome != UserProgramOutcome::kFaulted ||
      pf_res->fault_vector != 14 ||
      pf_res->fault_address != usertest::kKernelAddress) {
    return check;
  }
  ++check;
  // Fill all `kMaxUserChildProcesses` child slots simultaneously, verify
  // the (kMaxUserChildProcesses + 1)-th spawn returns kSyscallErrorInvalid,
  // then wait for all children.
  int64_t pids[kMaxUserChildProcesses] = {};
  for (int i = 0; i < kMaxUserChildProcesses; ++i) {
    pids[i] = Spawn("usertest", usertest::Arg(Mode::kFpuInitialState));
    if (pids[i] < 1 || pids[i] > kMaxUserChildProcesses) {
      return check;
    }
  }
  if (Spawn("usertest", usertest::Arg(Mode::kExit)) != kSyscallErrorInvalid) {
    return check;
  }
  for (int i = 0; i < kMaxUserChildProcesses; ++i) {
    const StatusOr<UserProgramResult> child_res = Wait(pids[i]);
    if (!child_res.ok() || child_res->outcome != UserProgramOutcome::kExited ||
        child_res->exit_status != 0) {
      return check;
    }
  }
  return 0;
}

static int64_t RunDriverDenied() {
  if (SpawnDriver("usertest", 8, 0) != kSyscallErrorPermission) {
    return 1;
  }
  if (GetPciDeviceCount() != kSyscallErrorPermission) {
    return 2;
  }
  if (PciClaim(0) != kSyscallErrorPermission) {
    return 3;
  }
  if (PciBusMaster(0, 1) != kSyscallErrorPermission) {
    return 4;
  }
  if (MmioMap(0x1000, 4096).status() != kSyscallErrorPermission) {
    return 5;
  }
  if (DmaAlloc(4096, 0).status() != kSyscallErrorPermission) {
    return 6;
  }
  if (IrqAlloc(0, 1, kSyscallIrqModeAuto) != kSyscallErrorPermission) {
    return 7;
  }
  if (IrqWait(1, 0) != kSyscallErrorPermission) {
    return 8;
  }
  if (ConsolePush("a", 1) != kSyscallErrorPermission) {
    return 9;
  }
  // GetTicks() (syscall 16) is available to all processes.
  if (GetTicks() < 0) {
    return 10;
  }
  if (IsaClaim(kIsaDeviceI8042) != kSyscallErrorPermission) {
    return 13;
  }
  if (IsaPortIo(kSyscallPortIoRead8, 0x60, 0) != kSyscallErrorPermission) {
    return 14;
  }
  if (IsaIrqAlloc(kIsaDeviceI8042) != kSyscallErrorPermission) {
    return 15;
  }
  if (Syscall(kSyscallIsaIrqAlloc + 1, 0, 0, 0) != kSyscallErrorNoSys) {
    return 11;
  }
  if (Syscall(-1, 0, 0, 0) != kSyscallErrorNoSys) {
    return 12;
  }
  return 0;
}

static int64_t RunDriverCapProbe(const uint64_t arg) {
  using usertest::DriverCapProbeSubmode;
  const DriverCapProbeSubmode submode =
      static_cast<DriverCapProbeSubmode>(usertest::DriverSubmode(arg));
  if (submode == DriverCapProbeSubmode::kProbeDriverRole) {
    // Allowed for a driver: returns the number of PCI functions, which is 0 on
    // a machine without PCI (such as a Hyper-V Generation 2 VM).
    if (GetPciDeviceCount() < 0) {
      return 1;
    }
    if (SpawnDriver("usertest", 8, 0) != kSyscallErrorPermission) {
      return 2;
    }
    if (GetTicks() < 0) {
      return 3;
    }
    if (IsaClaim(-1) != kSyscallErrorNoDevice ||
        IsaClaim(kIsaDeviceCount) != kSyscallErrorNoDevice ||
        IsaPortIo(kSyscallPortIoRead8, 0x3F8, 0) != kSyscallErrorAccess ||
        IsaPortIo(kSyscallPortIoRead8, 0x60, 0) != kSyscallErrorAccess ||
        IsaIrqAlloc(kIsaDeviceI8042) != kSyscallErrorAccess ||
        Syscall(kSyscallIsaIrqAlloc + 1, 0, 0, 0) != kSyscallErrorNoSys) {
      return 6;
    }
    const int64_t child_pid =
        Spawn("usertest", usertest::Arg(Mode::kDriverDenied));
    if (child_pid < 1) {
      return 4;
    }
    const StatusOr<UserProgramResult> child_result = Wait(child_pid);
    if (!child_result.ok() ||
        child_result->outcome != UserProgramOutcome::kExited ||
        child_result->exit_status != 0) {
      return 5;
    }
    return 0;
  }

  // kProbeSpawnerRole: process has kUserCapSpawnDrivers only.
  if (GetPciDeviceCount() != kSyscallErrorPermission ||
      PciClaim(0) != kSyscallErrorPermission ||
      PciBusMaster(0, 1) != kSyscallErrorPermission ||
      MmioMap(0x1000, 4096).status() != kSyscallErrorPermission ||
      DmaAlloc(4096, 0).status() != kSyscallErrorPermission ||
      IrqAlloc(0, 1, kSyscallIrqModeAuto) != kSyscallErrorPermission ||
      IrqWait(1, 0) != kSyscallErrorPermission ||
      ConsolePush("a", 1) != kSyscallErrorPermission ||
      IsaClaim(kIsaDeviceI8042) != kSyscallErrorPermission ||
      IsaPortIo(kSyscallPortIoRead8, 0x60, 0) != kSyscallErrorPermission ||
      IsaIrqAlloc(kIsaDeviceI8042) != kSyscallErrorPermission ||
      GetTicks() < 0) {
    return 10;
  }
  if (SpawnDriver(nullptr, 0, 0) != kSyscallErrorInvalid ||
      SpawnDriver("usertest", -1, 0) != kSyscallErrorInvalid ||
      SpawnDriver("usertest", kSyscallSpawnMaxNameLength + 1, 0) !=
          kSyscallErrorInvalid ||
      SpawnDriver("nonexistent_drv", 15, 0) != kSyscallErrorInvalid ||
      SpawnDriver(nullptr, 4, 0) != kSyscallErrorFault ||
      SpawnDriver(reinterpret_cast<const char*>(usertest::kKernelAddress), 4,
                  0) != kSyscallErrorFault) {
    return 11;
  }
  // A regular spawn() from kUserCapSpawnDrivers still creates an unprivileged
  // child (kUserCapNone).
  const int64_t unpriv_pid =
      Spawn("usertest", usertest::Arg(Mode::kDriverDenied));
  if (unpriv_pid < 1) {
    return 12;
  }
  const StatusOr<UserProgramResult> unpriv_result = Wait(unpriv_pid);
  if (!unpriv_result.ok() ||
      unpriv_result->outcome != UserProgramOutcome::kExited ||
      unpriv_result->exit_status != 0) {
    return 13;
  }
  const int64_t drv_pid = SpawnDriver(
      "usertest",
      usertest::DriverArg(
          Mode::kDriverCapProbe,
          static_cast<uint8_t>(DriverCapProbeSubmode::kProbeDriverRole)));
  if (drv_pid < 1) {
    return 14;
  }
  const StatusOr<UserProgramResult> drv_result = Wait(drv_pid);
  if (!drv_result.ok() || drv_result->outcome != UserProgramOutcome::kExited ||
      drv_result->exit_status != 0) {
    return 15;
  }
  return 0;
}

static bool IsMemoryBar(const PciBarInfo& bar) {
  return (bar.type == PciBarInfoType::kMemory32 ||
          bar.type == PciBarInfoType::kMemory64) &&
         bar.phys_base != 0 && bar.size >= 4096;
}

static int64_t RunDriverHold(const uint64_t arg) {
  using usertest::DriverHoldSubmode;
  const DriverHoldSubmode submode =
      static_cast<DriverHoldSubmode>(usertest::DriverSubmode(arg));
  const int dev_idx = usertest::DriverDeviceIndex(arg);
  const int bar_idx = usertest::DriverBarIndex(arg);

  if (PciClaim(dev_idx) != 0) {
    return 1;
  }
  if (PciBusMaster(dev_idx, 1) != 0) {
    return 2;
  }
  const StatusOr<PciDeviceInfo> info = GetPciInfo(dev_idx);
  if (!info.ok()) {
    return 3;
  }
  if (bar_idx >= 0 && bar_idx < 6 && IsMemoryBar(info->bars[bar_idx])) {
    if (!MmioMap(info->bars[bar_idx].phys_base, 4096).ok()) {
      return 4;
    }
  }
  if (submode == DriverHoldSubmode::kHoldClaimOnly) {
    (void)IrqWait(0, kSyscallIrqWaitMaxTicks);
    return 0;
  }

  const StatusOr<DmaAllocation> dma = DmaAlloc(4 * 4096, 0);
  if (!dma.ok() || dma->phys == 0) {
    return 5;
  }
  if (info->msi_vectors != 0 || info->msix_vectors > 0) {
    if (IrqAlloc(dev_idx, 1, kSyscallIrqModeAuto) != 0) {
      return 6;
    }
  }
  const int64_t child_pid =
      Spawn("usertest", usertest::Arg(Mode::kSpinWithoutSyscalls, 0x444));
  if (child_pid < 1) {
    return 7;
  }
  if (submode == DriverHoldSubmode::kHoldExit) {
    return 0;
  }
  if (submode == DriverHoldSubmode::kHoldFault) {
    asm volatile("ud2");
    return 8;
  }
  (void)IrqWait(0, kSyscallIrqWaitMaxTicks);
  return 0;
}

static int64_t RunDriverForeign(const uint64_t arg) {
  using usertest::DriverForeignSubmode;
  const DriverForeignSubmode submode =
      static_cast<DriverForeignSubmode>(usertest::DriverSubmode(arg));
  const int dev_idx = usertest::DriverDeviceIndex(arg);
  const int bar_idx = usertest::DriverBarIndex(arg);

  const StatusOr<PciDeviceInfo> info = GetPciInfo(dev_idx);
  if (!info.ok()) {
    return 1;
  }
  const uint64_t bar_phys = info->bars[bar_idx].phys_base;

  if (submode == DriverForeignSubmode::kUnclaimedDevice) {
    if (PciBusMaster(dev_idx, 1) != kSyscallErrorAccess ||
        PciBusMaster(dev_idx, 0) != kSyscallErrorAccess ||
        MmioMap(bar_phys, 4096).status() != kSyscallErrorAccess ||
        IrqAlloc(dev_idx, 1, kSyscallIrqModeAuto) != kSyscallErrorAccess) {
      return 2;
    }
    return 0;
  }

  if (submode == DriverForeignSubmode::kOwnedByOtherProcess) {
    if (PciClaim(dev_idx) != kSyscallErrorBusy ||
        PciBusMaster(dev_idx, 0) != kSyscallErrorAccess ||
        PciBusMaster(dev_idx, 1) != kSyscallErrorAccess ||
        MmioMap(bar_phys, 4096).status() != kSyscallErrorAccess ||
        IrqAlloc(dev_idx, 1, kSyscallIrqModeAuto) != kSyscallErrorAccess) {
      return 3;
    }
    return 0;
  }

  // kValidationAndClaimToggle:
  const int64_t count = GetPciDeviceCount();
  if (count < 1) {
    return 10;
  }
  const uint64_t dev_idx_u = static_cast<uint64_t>(dev_idx);
  if (GetPciInfo(-1).status() != kSyscallErrorNoDevice ||
      GetPciInfo(count).status() != kSyscallErrorNoDevice ||
      Syscall(kSyscallPciInfo, 1, 0) != kSyscallErrorFault) {
    return 11;
  }
  if (Syscall(kSyscallPciInfo, dev_idx_u, usertest::kKernelAddress) !=
          kSyscallErrorFault ||
      Syscall(kSyscallPciInfo, dev_idx_u, kNonCanonicalAddress) !=
          kSyscallErrorFault ||
      Syscall(kSyscallPciInfo, dev_idx_u, kUnmappedUserAddress) !=
          kSyscallErrorFault ||
      Syscall(kSyscallPciInfo, dev_idx_u,
              reinterpret_cast<uintptr_t>(&kReadOnlyWord)) !=
          kSyscallErrorFault) {
    return 12;
  }
  if (PciClaim(-1) != kSyscallErrorNoDevice ||
      PciClaim(count) != kSyscallErrorNoDevice ||
      PciClaim(0) != kSyscallErrorInvalid) {
    return 13;
  }
  // Claiming the same device a second time from the same owner is idempotent.
  if (PciClaim(dev_idx) != 0 || PciClaim(dev_idx) != 0) {
    return 14;
  }
  if (PciBusMaster(-1, 1) != kSyscallErrorNoDevice ||
      PciBusMaster(count, 1) != kSyscallErrorNoDevice ||
      PciBusMaster(dev_idx, -1) != kSyscallErrorInvalid ||
      PciBusMaster(dev_idx, 2) != kSyscallErrorInvalid) {
    return 15;
  }
  if (PciBusMaster(dev_idx, 1) != 0 || PciBusMaster(dev_idx, 0) != 0 ||
      PciBusMaster(dev_idx, 1) != 0) {
    return 16;
  }
  return 0;
}

static int64_t RunDriverMmioChecks(const uint64_t arg) {
  const uint8_t submode = usertest::DriverSubmode(arg);
  const int dev_idx = usertest::DriverDeviceIndex(arg);
  const int bar_idx = usertest::DriverBarIndex(arg);

  if (PciClaim(dev_idx) != 0) {
    return 1;
  }
  const StatusOr<PciDeviceInfo> info = GetPciInfo(dev_idx);
  if (!info.ok()) {
    return 2;
  }
  const uint64_t bar_phys = info->bars[bar_idx].phys_base;
  const int64_t bar_size = info->bars[bar_idx].size;
  if (bar_phys == 0 || bar_size < 4096) {
    return 3;
  }
  // Invalid alignment, size, or overflow -> kSyscallErrorInvalid.
  if (MmioMap(0, 4096).status() != kSyscallErrorInvalid ||
      MmioMap(bar_phys + 1, 4096).status() != kSyscallErrorInvalid ||
      MmioMap(bar_phys, 0).status() != kSyscallErrorInvalid ||
      MmioMap(bar_phys, -4096).status() != kSyscallErrorInvalid ||
      MmioMap(bar_phys, 100).status() != kSyscallErrorInvalid ||
      MmioMap(bar_phys, kSyscallMmioMaxSize + 4096).status() !=
          kSyscallErrorInvalid) {
    return 4;
  }
  uint64_t unmapped_phys = bar_phys + static_cast<uint64_t>(bar_size);
  for (int pass = 0; pass < 6; ++pass) {
    for (int b = 0; b < 6; ++b) {
      const uint64_t b_base = info->bars[b].phys_base;
      const uint64_t b_size = static_cast<uint64_t>(info->bars[b].size);
      if (IsMemoryBar(info->bars[b]) && unmapped_phys >= b_base &&
          unmapped_phys - b_base < b_size) {
        unmapped_phys = b_base + b_size;
      }
    }
  }
  // Physical address outside any claimed BAR -> kSyscallErrorAccess.
  if (MmioMap(unmapped_phys, 4096).status() != kSyscallErrorAccess ||
      MmioMap(0x100000ULL, 4096).status() != kSyscallErrorAccess) {
    return 5;
  }
  // If this BAR holds an MSI-X structure and fits in kSyscallMmioMaxSize,
  // attempting to map the entire BAR (including the MSI-X table/PBA page) must
  // be rejected with kSyscallErrorAccess.
  if ((submode & 2) != 0) {
    if (MmioMap(bar_phys, bar_size).status() != kSyscallErrorAccess) {
      return 6;
    }
  }
  // Map the first 4 KiB page of the BAR (non-MSI-X register page).
  const StatusOr<uintptr_t> va1 = MmioMap(bar_phys, 4096);
  if (!va1.ok() || *va1 != 0xFFFF900000000000ULL) {
    return 7;
  }
  // Mapping the same page a second time places it after the first mapping's
  // 4 KiB guard page (`va1 + 2 * 4096`).
  const StatusOr<uintptr_t> va2 = MmioMap(bar_phys, 4096);
  if (!va2.ok() || *va2 != *va1 + 2 * 4096) {
    return 8;
  }
  const uint32_t reg0 =
      *Launder(reinterpret_cast<const volatile uint32_t*>(*va1));
  (void)reg0;
  // Device MMIO pages carry kPteNotOwned, so kernel user-copy helpers
  // (MayRead/MayWrite -> PagingUserRangeAccessible) reject them with EFAULT.
  if (Write(reinterpret_cast<const char*>(*va1), 4) != kSyscallErrorFault ||
      ConsolePush(reinterpret_cast<const char*>(*va1), 4) !=
          kSyscallErrorFault ||
      Read(reinterpret_cast<char*>(*va1), 4) != kSyscallErrorFault) {
    return 9;
  }
  if ((submode & 1) != 0) {
    (void)IrqWait(0, kSyscallIrqWaitMaxTicks);
  }
  return 0;
}

static int64_t RunDriverMmioFault(const uint64_t arg) {
  using usertest::DriverMmioFaultSubmode;
  const DriverMmioFaultSubmode submode =
      static_cast<DriverMmioFaultSubmode>(usertest::DriverSubmode(arg));
  const int dev_idx = usertest::DriverDeviceIndex(arg);
  const int bar_idx = usertest::DriverBarIndex(arg);

  if (PciClaim(dev_idx) != 0) {
    return 1;
  }
  const StatusOr<PciDeviceInfo> info = GetPciInfo(dev_idx);
  if (!info.ok()) {
    return 2;
  }
  const StatusOr<uintptr_t> va = MmioMap(info->bars[bar_idx].phys_base, 4096);
  if (!va.ok()) {
    return 3;
  }
  if (submode == DriverMmioFaultSubmode::kFaultExecuteMmio) {
    Launder(reinterpret_cast<void (*)()>(*va))();
    return 4;
  }
  // Read the unmapped guard page right after the 1-page mapping (`va + 4096`).
  const uintptr_t guard_page = *va + 4096;
  const uint32_t word =
      *Launder(reinterpret_cast<const volatile uint32_t*>(guard_page));
  (void)word;
  return 5;
}

static int64_t RunDriverDmaChecks(const uint64_t arg) {
  const uint8_t submode = usertest::DriverSubmode(arg);
  if (DmaAlloc(0, 0).status() != kSyscallErrorInvalid ||
      DmaAlloc(-4096, 0).status() != kSyscallErrorInvalid ||
      DmaAlloc(100, 0).status() != kSyscallErrorInvalid ||
      DmaAlloc(kSyscallDmaMaxSize + 4096, 0).status() != kSyscallErrorInvalid ||
      DmaAlloc(4096, 2).status() != kSyscallErrorInvalid) {
    return 1;
  }
  if (Syscall(kSyscallDmaAlloc, 4096, 0, 0) != kSyscallErrorFault ||
      Syscall(kSyscallDmaAlloc, 4096, usertest::kKernelAddress, 0) !=
          kSyscallErrorFault ||
      Syscall(kSyscallDmaAlloc, 4096, kNonCanonicalAddress, 0) !=
          kSyscallErrorFault ||
      Syscall(kSyscallDmaAlloc, 4096, kUnmappedUserAddress, 0) !=
          kSyscallErrorFault ||
      Syscall(kSyscallDmaAlloc, 4096,
              reinterpret_cast<uintptr_t>(&kReadOnlyWord),
              0) != kSyscallErrorFault) {
    return 2;
  }
  const StatusOr<DmaAllocation> dma1 = DmaAlloc(4 * 4096, kSyscallDmaBelow4GiB);
  if (!dma1.ok() || dma1->va != 0xFFFFA00000000000ULL || dma1->phys == 0 ||
      (dma1->phys & 0xFFFULL) != 0 || dma1->phys + 4 * 4096 > 0x100000000ULL) {
    return 3;
  }
  // Second allocation sits after the 4 pages of `va1` plus its 1 guard page
  // (`va1 + 5 * 4096`).
  const StatusOr<DmaAllocation> dma2 = DmaAlloc(4096, 0);
  if (!dma2.ok() || dma2->va != dma1->va + 5 * 4096 || dma2->phys == 0 ||
      (dma2->phys & 0xFFFULL) != 0 || dma2->phys == dma1->phys) {
    return 4;
  }
  const uintptr_t va1 = dma1->va;
  const uint64_t phys1 = dma1->phys;
  const uintptr_t va2 = dma2->va;
  const uint64_t phys2 = dma2->phys;
  const volatile uint64_t* const words1 =
      reinterpret_cast<const volatile uint64_t*>(va1);
  for (int i = 0; i < (4 * 4096) / 8; ++i) {
    if (words1[i] != 0) {
      return 5;
    }
  }
  const volatile uint64_t* const words2 =
      reinterpret_cast<const volatile uint64_t*>(va2);
  for (int i = 0; i < 4096 / 8; ++i) {
    if (words2[i] != 0) {
      return 6;
    }
  }
  volatile uint64_t* const mut1 = reinterpret_cast<volatile uint64_t*>(va1);
  mut1[0] = phys1;
  for (int p = 1; p < 4; ++p) {
    mut1[p * (4096 / 8)] = 0xD0A0000000000000ULL + static_cast<uint64_t>(p);
  }
  volatile uint64_t* const mut2 = reinterpret_cast<volatile uint64_t*>(va2);
  mut2[0] = phys2;

  if (submode == 1) {
    // Write a RET instruction (0xC3) at va1 and jump to it to trigger an NX
    // instruction-fetch #PF on the DMA page.
    *reinterpret_cast<volatile uint8_t*>(va1) = 0xC3;
    Launder(reinterpret_cast<void (*)()>(va1))();
    return 7;
  }
  (void)IrqWait(0, kSyscallIrqWaitMaxTicks);
  return 0;
}

static int64_t RunDriverConsolePush() {
  if (ConsolePush("a", -1) != kSyscallErrorInvalid ||
      ConsolePush("a", kSyscallConsolePushMaxSize + 1) !=
          kSyscallErrorInvalid) {
    return 1;
  }
  if (ConsolePush(nullptr, 0) != 0) {
    return 2;
  }
  if (ConsolePush(nullptr, 4) != kSyscallErrorFault ||
      ConsolePush(reinterpret_cast<const char*>(usertest::kKernelAddress), 4) !=
          kSyscallErrorFault ||
      ConsolePush(reinterpret_cast<const char*>(kNonCanonicalAddress), 4) !=
          kSyscallErrorFault ||
      ConsolePush(reinterpret_cast<const char*>(kUnmappedUserAddress), 4) !=
          kSyscallErrorFault) {
    return 3;
  }
  if (ConsolePush("drv_in!\n", 8) != 8) {
    return 4;
  }
  return 0;
}

static int64_t RunDriverIrqWait(const uint64_t arg) {
  using usertest::DriverIrqSubmode;
  const DriverIrqSubmode submode =
      static_cast<DriverIrqSubmode>(usertest::DriverSubmode(arg));
  const int dev_idx = usertest::DriverDeviceIndex(arg);

  if (PciClaim(dev_idx) != 0) {
    return 1;
  }
  const StatusOr<PciDeviceInfo> info_or = GetPciInfo(dev_idx);
  if (!info_or.ok()) {
    return 2;
  }
  const PciDeviceInfo& info = *info_or;
  const bool has_hw_irq = info.msi_vectors != 0 || info.msix_vectors != 0;
  const int64_t count = (!has_hw_irq || info.msix_vectors >= 2) ? 2 : 1;

  if (submode == DriverIrqSubmode::kIrqValidationAndTimeout) {
    if (IrqAlloc(-1, 1, kSyscallIrqModeAuto) != kSyscallErrorNoDevice ||
        IrqAlloc(0, 1, kSyscallIrqModeAuto) != kSyscallErrorAccess ||
        IrqAlloc(dev_idx, 0, kSyscallIrqModeAuto) != kSyscallErrorInvalid ||
        IrqAlloc(dev_idx, -1, kSyscallIrqModeAuto) != kSyscallErrorInvalid ||
        IrqAlloc(dev_idx, kSyscallIrqMaxCount + 1, kSyscallIrqModeAuto) !=
            kSyscallErrorInvalid ||
        IrqAlloc(dev_idx, 1, 3) != kSyscallErrorInvalid ||
        IrqAlloc(dev_idx, 1, -1) != kSyscallErrorInvalid) {
      return 10;
    }
    if (info.msi_vectors == 0) {
      if (IrqAlloc(dev_idx, 1, kSyscallIrqModeMsi) != kSyscallErrorInvalid) {
        return 11;
      }
    } else if (IrqAlloc(dev_idx, 2, kSyscallIrqModeMsi) !=
               kSyscallErrorInvalid) {
      return 12;
    }
    if (info.msix_vectors == 0) {
      if (IrqAlloc(dev_idx, 1, kSyscallIrqModeMsix) != kSyscallErrorInvalid) {
        return 13;
      }
    } else if (IrqAlloc(dev_idx, static_cast<int64_t>(info.msix_vectors) + 1,
                        kSyscallIrqModeMsix) != kSyscallErrorInvalid) {
      return 14;
    }
    if (has_hw_irq) {
      if (IrqAlloc(dev_idx, count, kSyscallIrqModeAuto) != 0) {
        return 15;
      }
      if (IrqAlloc(dev_idx, 1, kSyscallIrqModeAuto) != kSyscallErrorBusy) {
        return 16;
      }
    } else if (IrqAlloc(dev_idx, 1, kSyscallIrqModeAuto) !=
               kSyscallErrorInvalid) {
      return 15;
    }
    if (IrqWait(uint64_t{1} << 32, 0) != kSyscallErrorInvalid ||
        IrqWait(~uint64_t{0}, 0) != kSyscallErrorInvalid ||
        IrqWait(1, kSyscallIrqWaitMaxTicks + 1) != kSyscallErrorInvalid) {
      return 17;
    }
    if (IrqWait((uint64_t{1} << count) - 1, 0) != 0) {
      return 18;
    }
    if (IrqWait(1, 5) != 0) {
      return 19;
    }
    if (IrqWait(0, 2) != 0) {
      return 20;
    }
    return 0;
  }

  if (submode == DriverIrqSubmode::kIrqPrePendingAndSelectiveMask) {
    if (has_hw_irq && IrqAlloc(dev_idx, count, kSyscallIrqModeAuto) != 0) {
      return 30;
    }
    char sync = 0;
    if (Read(&sync, 1) != 1 || sync != 'G') {
      return 31;
    }
    if (IrqWait(uint64_t{1} << 0, 100) != 1 ||
        IrqWait(uint64_t{1} << 0, 0) != 0) {
      return 32;
    }
    if (count == 2) {
      if (IrqWait(uint64_t{1} << 1, 0) != 2 || IrqWait(3, 0) != 0) {
        return 33;
      }
    }
    return 0;
  }

  if (submode == DriverIrqSubmode::kIrqBlockingWakeup) {
    if (has_hw_irq && IrqAlloc(dev_idx, count, kSyscallIrqModeAuto) != 0) {
      return 40;
    }
    for (int64_t k = 0; k < count; ++k) {
      const uint64_t bit = uint64_t{1} << k;
      if (IrqWait(bit, 5000) != static_cast<int64_t>(bit) ||
          IrqWait(bit, 0) != 0) {
        return 41 + k;
      }
      if (k + 1 < count) {
        char sync = 0;
        if (Read(&sync, 1) != 1 || sync != 'G') {
          return 45;
        }
      }
    }
    return 0;
  }

  // kIrqKillWhileBlocked:
  if (has_hw_irq && IrqAlloc(dev_idx, 1, kSyscallIrqModeAuto) != 0) {
    return 50;
  }
  (void)IrqWait(1, 5000);
  return 51;
}

// The xHCI programming interface: PCI class 0C (serial bus controller),
// subclass 03 (USB), programming interface 30.
constexpr uint8_t kPciClassSerialBus = 0x0C;
constexpr uint8_t kPciSubclassUsb = 0x03;
constexpr uint8_t kPciProgIfXhci = 0x30;

// The timer ticks to wait so that at least `ms` milliseconds pass: rounded
// up, plus one tick because the current tick may end right after the start
// was read (usb::TicksForMs()).
static constexpr int64_t TicksForMs(const int64_t ms) {
  return (ms * kSyscallTicksPerSecond + 999) / 1000 + 1;
}

// The bounds of the steps of `kDriverXhciReset`, those of usbkbd's bring-up
// (user/usbkbd/xhci_controller.h): firmware gets 1 s to release the controller
// before the program takes it by force (like Linux); CNR gets 5 s to clear
// before the first operational register write, and the program goes on
// after a timeout (like Linux); a running controller halts within 16 ms
// after Run/Stop clears (xHCI 5.4.1; Linux allows 32 ms); no register is
// accessed for 1 ms after HCRST is set (Intel controllers need that); then
// HCRST and CNR must clear within 10 s each (Linux XHCI_RESET_LONG_USEC).
constexpr int64_t kXhciHandoffTicks = TicksForMs(1000);
constexpr int64_t kXhciReadyTicks = TicksForMs(5000);
constexpr int64_t kXhciHaltTicks = TicksForMs(32);
constexpr int64_t kXhciPostResetTicks = TicksForMs(1);
constexpr int64_t kXhciResetTicks = TicksForMs(10000);

// The registers of a memory BAR of a claimed device, each 4 KiB page mapped
// with mmio_map() on its first access. mmio_map() withholds the pages of the
// MSI-X table and PBA, which may lie anywhere in the BAR, so the BAR may not
// map in one piece, while the xHCI reset touches only a few pages (the
// capability, operational and extended capability registers). Accesses that
// are unaligned, outside the BAR or to a page that doesn't map are not
// performed: reads return all-ones and writes are dropped.
class PagedMmioWindow final : public usb::MmioWindow {
 public:
  PagedMmioWindow(const uint64_t phys_base, const int64_t size)
      : phys_base_(phys_base), size_(size) {}

  uint32_t Read32(const int64_t offset) override {
    volatile uint32_t* const reg = Register(offset);
    if (reg == nullptr) {
      return usb::kMmioAllOnes;
    }
    asm volatile("" ::: "memory");
    const uint32_t value = *reg;
    asm volatile("" ::: "memory");
    return value;
  }

  void Write32(const int64_t offset, const uint32_t value) override {
    volatile uint32_t* const reg = Register(offset);
    if (reg == nullptr) {
      return;
    }
    asm volatile("" ::: "memory");
    *reg = value;
    asm volatile("" ::: "memory");
  }

  int64_t Size() const override { return size_; }

 private:
  static constexpr int kMaxPages = 8;

  volatile uint32_t* Register(const int64_t offset) {
    if (offset < 0 || (offset & 3) != 0 || offset > size_ - 4) {
      return nullptr;
    }
    const int64_t page = offset / kPageSize;
    int slot = 0;
    while (slot < page_count_ && pages_[slot] != page) {
      ++slot;
    }
    if (slot == page_count_) {
      if (page_count_ == kMaxPages) {
        return nullptr;
      }
      const StatusOr<uintptr_t> va =
          MmioMap(phys_base_ + static_cast<uint64_t>(page * kPageSize),  //
                  kPageSize);
      if (!va.ok()) {
        return nullptr;
      }
      pages_[slot] = page;
      vas_[slot] = *va;
      ++page_count_;
    }
    return reinterpret_cast<volatile uint32_t*>(
        vas_[slot] + static_cast<uintptr_t>(offset % kPageSize));
  }

  const uint64_t phys_base_;
  const int64_t size_;
  int64_t pages_[kMaxPages] = {};
  uintptr_t vas_[kMaxPages] = {};
  int page_count_ = 0;
};

enum class PollResult { kMatched, kTimedOut, kNotDecoding };

// Polls the register at `offset` until its `mask` bits read `expected` or
// `timeout_ticks` timer ticks have passed, sleeping until the next tick
// between two polls. Polls once more after the deadline, so that a program
// that was preempted past it reports no false timeout. A register that reads
// all-ones means that the controller stopped decoding its BAR.
static PollResult PollRegister(usb::MmioWindow& mmio,    //
                               const int64_t offset,     //
                               const uint32_t mask,      //
                               const uint32_t expected,  //
                               const int64_t timeout_ticks) {
  const int64_t deadline = GetTicks() + timeout_ticks;
  for (;;) {
    const bool last_poll = GetTicks() >= deadline;
    const uint32_t value = mmio.Read32(offset);
    if (value == usb::kMmioAllOnes) {
      return PollResult::kNotDecoding;
    }
    if ((value & mask) == expected) {
      return PollResult::kMatched;
    }
    if (last_poll) {
      return PollResult::kTimedOut;
    }
    (void)IrqWait(0, 1);
  }
}

// Takes the xHCI controller behind `mmio` over from firmware (xHCI 4.22.1):
// sets the OS Owned semaphore, waits until firmware clears its BIOS Owned
// semaphore (or clears it by force after kXhciHandoffTicks), and disables
// and acknowledges every firmware SMI.
static int64_t TakeXhciFromFirmware(usb::MmioWindow& mmio,
                                    const usb::XhciCaps& caps) {
  if (caps.legacy_offset == 0) {
    return 0;
  }
  const int64_t legsup = caps.legacy_offset + usb::kLegSup;
  const uint32_t value = mmio.Read32(legsup);
  if (value == usb::kMmioAllOnes) {
    return usertest::kXhciResetNotDecodingStatus;
  }
  if ((value & usb::kLegSupOsOwned) == 0) {
    mmio.Write32(legsup, value | usb::kLegSupOsOwned);
  }
  if ((value & usb::kLegSupBiosOwned) != 0) {
    const PollResult released = PollRegister(mmio,                   //
                                             legsup,                 //
                                             usb::kLegSupBiosOwned,  //
                                             0,                      //
                                             kXhciHandoffTicks);
    if (released == PollResult::kNotDecoding) {
      return usertest::kXhciResetNotDecodingStatus;
    }
    if (released == PollResult::kTimedOut) {
      const uint32_t current = mmio.Read32(legsup);
      if (current == usb::kMmioAllOnes) {
        return usertest::kXhciResetNotDecodingStatus;
      }
      mmio.Write32(legsup,
                   (current | usb::kLegSupOsOwned) & ~usb::kLegSupBiosOwned);
    }
  }
  const int64_t legctlsts = caps.legacy_offset + usb::kLegCtlSts;
  const uint32_t control = mmio.Read32(legctlsts);
  if (control == usb::kMmioAllOnes) {
    return usertest::kXhciResetNotDecodingStatus;
  }
  mmio.Write32(legctlsts, usb::LegacyControlDisablingSmis(control));
  return 0;
}

// Halts the xHCI controller behind `mmio` if it runs (xHCI 5.4.1: Run/Stop
// cleared, the controller halts once its current work is done) and resets it
// (HCRST, which a controller must not get while it runs). Leaves Bus Master
// alone: a controller that runs on with Bus Master off fails its next memory
// access and halts with Host System Error, which the reset clears too.
static int64_t HaltAndResetXhci(usb::MmioWindow& mmio,
                                const usb::XhciCaps& caps) {
  const int64_t usbcmd = caps.Op(usb::kOpUsbCmd);
  const int64_t usbsts = caps.Op(usb::kOpUsbSts);
  // No operational register may be written before CNR clears (xHCI 4.2).
  if (PollRegister(mmio, usbsts, usb::kUsbStsCnr, 0, kXhciReadyTicks) ==
      PollResult::kNotDecoding) {
    return usertest::kXhciResetNotDecodingStatus;
  }
  const uint32_t command = mmio.Read32(usbcmd);
  const uint32_t status = mmio.Read32(usbsts);
  if (command == usb::kMmioAllOnes || status == usb::kMmioAllOnes) {
    return usertest::kXhciResetNotDecodingStatus;
  }
  const bool was_running = (status & usb::kUsbStsHch) == 0;
  if ((command & usb::kUsbCmdHaltMask) != 0) {
    mmio.Write32(usbcmd, command & ~usb::kUsbCmdHaltMask);
  }
  const PollResult halted = PollRegister(mmio,             //
                                         usbsts,           //
                                         usb::kUsbStsHch,  //
                                         usb::kUsbStsHch,  //
                                         kXhciHaltTicks);
  if (halted != PollResult::kMatched) {
    return (halted == PollResult::kNotDecoding)
               ? usertest::kXhciResetNotDecodingStatus
               : usertest::kXhciResetHaltTimeoutStatus;
  }
  const uint32_t halted_command = mmio.Read32(usbcmd);
  if (halted_command == usb::kMmioAllOnes) {
    return usertest::kXhciResetNotDecodingStatus;
  }
  mmio.Write32(usbcmd,
               (halted_command & ~(usb::kUsbCmdHaltMask | usb::kUsbCmdLhcrst |
                                   usb::kUsbCmdCss | usb::kUsbCmdCrs)) |
                   usb::kUsbCmdHcrst);
  const int64_t delay_end = GetTicks() + kXhciPostResetTicks;
  while (GetTicks() < delay_end) {
    (void)IrqWait(0, 1);
  }
  PollResult reset =
      PollRegister(mmio, usbcmd, usb::kUsbCmdHcrst, 0, kXhciResetTicks);
  if (reset == PollResult::kMatched) {
    reset = PollRegister(mmio, usbsts, usb::kUsbStsCnr, 0, kXhciResetTicks);
  }
  if (reset != PollResult::kMatched) {
    return (reset == PollResult::kNotDecoding)
               ? usertest::kXhciResetNotDecodingStatus
               : usertest::kXhciResetTimeoutStatus;
  }
  const uint32_t reset_status = mmio.Read32(usbsts);
  if (reset_status == usb::kMmioAllOnes) {
    return usertest::kXhciResetNotDecodingStatus;
  }
  if ((reset_status & usb::kUsbStsHch) == 0) {
    return usertest::kXhciResetTimeoutStatus;
  }
  return was_running ? usertest::kXhciResetWasRunningStatus : 0;
}

static int64_t RunDriverXhciReset(const uint64_t arg) {
  const int dev_idx = usertest::DriverDeviceIndex(arg);
  if (PciClaim(dev_idx) != 0) {
    return usertest::kXhciResetNotXhciStatus;
  }
  const StatusOr<PciDeviceInfo> info = GetPciInfo(dev_idx);
  if (!info.ok() || info->class_code != kPciClassSerialBus ||
      info->subclass != kPciSubclassUsb || info->prog_if != kPciProgIfXhci ||
      !IsMemoryBar(info->bars[0])) {
    return usertest::kXhciResetNotXhciStatus;
  }
  PagedMmioWindow mmio(info->bars[0].phys_base, info->bars[0].size);
  StatusOr<usb::XhciCaps, usb::XhciStatus> caps = usb::DecodeCaps(mmio);
  usb::XhciStatus decoded =
      caps.ok() ? usb::ParseExtendedCaps(mmio, &*caps) : caps.status();
  if (decoded != usb::XhciStatus::kOk) {
    return (decoded == usb::XhciStatus::kNotDecoding)
               ? usertest::kXhciResetNotDecodingStatus
               : usertest::kXhciResetBadCapabilitiesStatus;
  }
  const int64_t handoff = TakeXhciFromFirmware(mmio, *caps);
  if (handoff != 0) {
    return handoff;
  }
  return HaltAndResetXhci(mmio, *caps);
}

}  // namespace

int64_t Main(const uint64_t arg) {
  const Mode mode = static_cast<Mode>(arg & 0xFF);
  // The state that the entry point saved before any compiled code ran, which
  // kFpuInitialState checks as its mode (usertest.h).
  if (mode != Mode::kFpuInitialState) {
    const int64_t failed_check = CheckInitialFpuState();
    if (failed_check != 0) {
      return usertest::kFpuInitialStateMismatchStatusBase + failed_check;
    }
  }
  switch (mode) {
    case Mode::kExit:
      return usertest::kExitStatus;
    case Mode::kSyscalls:
      return RunSyscallChecks();
    case Mode::kPrivilegedInstruction:
      asm volatile("cli");
      break;
    case Mode::kReadKernelMemory:
      return *Launder(
          reinterpret_cast<const volatile int64_t*>(usertest::kKernelAddress));
    case Mode::kWriteReadOnlyData:
      *Launder(const_cast<volatile uint64_t*>(&kReadOnlyWord)) = 0;
      break;
    case Mode::kExecuteData:
      Launder(reinterpret_cast<void (*)()>(g_return_instruction))();
      return 0;
    case Mode::kInvalidOpcode:
      asm volatile("ud2");
      break;
    case Mode::kKernelInterrupt:
      asm volatile("int %0" : : "i"(usertest::kKernelOnlyVector));
      break;
    case Mode::kStackOverflow:
      return static_cast<int64_t>(Recurse(0));
    case Mode::kSpin:
      return Spin(arg >> 8);
    case Mode::kKernelSpin:
      return KernelSpin(arg);
    case Mode::kSpinWithoutSyscalls:
      return SpinWithoutSyscalls(arg >> 8);
    case Mode::kFpuHold:
      return FpuHold(arg >> 8);
    case Mode::kFpuDirty:
      return FpuDirty(arg >> 8);
    case Mode::kFpuInitialState:
      return CheckInitialFpuState();
    case Mode::kFpuFeatures:
      return static_cast<int64_t>(EnabledXstateComponents() & 0xFFFFFFFF);
    case Mode::kBufferChecks:
      return RunBufferChecks();
    case Mode::kFpuLongSyscall:
      return FpuLongSyscall(arg);
    case Mode::kX87PendingException:
      return X87PendingException(usertest::X87PendingRoundCycles(arg));
    case Mode::kConsoleRead:
      return RunConsoleRead(static_cast<int64_t>(arg >> 8));
    case Mode::kSpawnAndWait:
      return RunSpawnAndWait(
          static_cast<usertest::SpawnTestSubmode>((arg >> 8) & 0xFF));
    case Mode::kDriverDenied:
      return RunDriverDenied();
    case Mode::kDriverCapProbe:
      return RunDriverCapProbe(arg);
    case Mode::kDriverHold:
      return RunDriverHold(arg);
    case Mode::kDriverForeign:
      return RunDriverForeign(arg);
    case Mode::kDriverMmioChecks:
      return RunDriverMmioChecks(arg);
    case Mode::kDriverMmioFault:
      return RunDriverMmioFault(arg);
    case Mode::kDriverDmaChecks:
      return RunDriverDmaChecks(arg);
    case Mode::kDriverConsolePush:
      return RunDriverConsolePush();
    case Mode::kDriverIrqWait:
      return RunDriverIrqWait(arg);
    case Mode::kDriverXhciReset:
      return RunDriverXhciReset(arg);
  }
  // The mode should have faulted, or it is unknown.
  return -1;
}

}  // namespace protos::user
