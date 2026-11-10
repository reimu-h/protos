// usertest: a user program that the boot self-tests run in many ways to check
// user mode (see user/usertest/usertest.h).

#include "user/usertest/usertest.h"

#include <cstddef>
#include <cstdint>

#include "block_protocol.h"
#include "syscall_abi.h"
#include "user/block.h"
#include "user/nvme/nvme_regs.h"
#include "user/ramfs/boot_module_source.h"
#include "user/ramfs/tar.h"
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
extern "C" const uint8_t g_fs_test_prog_start[];
extern "C" const uint8_t g_fs_test_prog_end[];

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
  UserProgramResult try_wait_res{};
  if (TryWait(0, &try_wait_res) != kSyscallErrorInvalid ||
      TryWait(-1, &try_wait_res) != kSyscallErrorInvalid ||
      TryWait(kMaxUserChildProcesses + 1, &try_wait_res) !=
          kSyscallErrorInvalid ||
      TryWait(1, &try_wait_res) != kSyscallErrorInvalid) {
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
  UserProgramResult try_wait_res{};
  if (Wait(0).status() != kSyscallErrorInvalid ||
      Wait(-1).status() != kSyscallErrorInvalid ||
      Wait(kMaxUserChildProcesses + 1).status() != kSyscallErrorInvalid ||
      Wait(1).status() != kSyscallErrorInvalid ||
      TryWait(0, &try_wait_res) != kSyscallErrorInvalid ||
      TryWait(-1, &try_wait_res) != kSyscallErrorInvalid ||
      TryWait(kMaxUserChildProcesses + 1, &try_wait_res) !=
          kSyscallErrorInvalid ||
      TryWait(1, &try_wait_res) != kSyscallErrorInvalid) {
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
  ++check;
  // COV-07 Points 3-7: live ring-3 TryWait matrix (running Fault without reap,
  // running -> 0 with sentinel untouched + slot held, ended -> 1 + slot freed,
  // already-reaped -> Invalid, ended Fault pre-check without reap -> 1).
  const int64_t spin_pid =
      Spawn("usertest", usertest::KernelSpinArg(100, 1, 0x42));
  if (spin_pid < 1 || spin_pid > kMaxUserChildProcesses) {
    return check;
  }
  ++check;
  const uint64_t spin_pid_u = static_cast<uint64_t>(spin_pid);
  const uintptr_t ro_addr = reinterpret_cast<uintptr_t>(&kReadOnlyWord);
  const uintptr_t text_addr =
      reinterpret_cast<uintptr_t>(&SyscallChangedRegisterBits);
  if (Syscall(kSyscallTryWait, spin_pid_u, 0) != kSyscallErrorFault ||
      Syscall(kSyscallTryWait, spin_pid_u, usertest::kKernelAddress) !=
          kSyscallErrorFault ||
      Syscall(kSyscallTryWait, spin_pid_u, kNonCanonicalAddress) !=
          kSyscallErrorFault ||
      Syscall(kSyscallTryWait, spin_pid_u, kUnmappedUserAddress) !=
          kSyscallErrorFault ||
      Syscall(kSyscallTryWait, spin_pid_u, ro_addr) != kSyscallErrorFault ||
      Syscall(kSyscallTryWait, spin_pid_u, text_addr) != kSyscallErrorFault ||
      Syscall(kSyscallTryWait, spin_pid_u, kStackGuardPage) !=
          kSyscallErrorFault ||
      Syscall(kSyscallTryWait, spin_pid_u, usertest::kStackTop - 4) !=
          kSyscallErrorFault) {
    return check;
  }
  ++check;
  UserProgramResult tw_res{};
  uint8_t* const tw_bytes = reinterpret_cast<uint8_t*>(&tw_res);
  constexpr int kResultBytes = static_cast<int>(sizeof(tw_res));
  for (int i = 0; i < kResultBytes; ++i) {
    tw_bytes[i] = 0xA5;
  }
  if (TryWait(spin_pid, &tw_res) != 0) {
    return check;
  }
  for (int i = 0; i < kResultBytes; ++i) {
    if (tw_bytes[i] != 0xA5) {
      return check;
    }
  }
  ++check;
  // Remaining kMaxUserChildProcesses - 1 slots prove spin_pid still occupies
  // its child slot.
  int64_t extra_pids[kMaxUserChildProcesses - 1] = {};
  for (int i = 0; i < kMaxUserChildProcesses - 1; ++i) {
    extra_pids[i] = Spawn("usertest", usertest::Arg(Mode::kExit));
    if (extra_pids[i] < 1 || extra_pids[i] > kMaxUserChildProcesses) {
      return check;
    }
  }
  if (Spawn("usertest", usertest::Arg(Mode::kExit)) != kSyscallErrorInvalid) {
    return check;
  }
  for (int i = 0; i < kMaxUserChildProcesses - 1; ++i) {
    const StatusOr<UserProgramResult> r = Wait(extra_pids[i]);
    if (!r.ok() || r->outcome != UserProgramOutcome::kExited ||
        r->exit_status != usertest::kExitStatus) {
      return check;
    }
  }
  ++check;
  int64_t tw_rc = 0;
  for (int poll = 0; poll < 5000; ++poll) {
    tw_rc = TryWait(spin_pid, &tw_res);
    if (tw_rc != 0) {
      break;
    }
    (void)SpinOnBuffer(nullptr, 0, 1);
  }
  if (tw_rc != 1 || tw_res.outcome != UserProgramOutcome::kExited ||
      tw_res.exit_status != 0) {
    return check;
  }
  if (TryWait(spin_pid, &tw_res) != kSyscallErrorInvalid) {
    return check;
  }
  ++check;
  const int64_t ud_tw_pid =
      Spawn("usertest", usertest::Arg(Mode::kInvalidOpcode));
  if (ud_tw_pid < 1 || ud_tw_pid > kMaxUserChildProcesses) {
    return check;
  }
  tw_rc = 0;
  for (int poll = 0; poll < 5000; ++poll) {
    tw_rc = TryWait(ud_tw_pid, &tw_res);
    if (tw_rc != 0) {
      break;
    }
    (void)SpinOnBuffer(nullptr, 0, 1);
  }
  if (tw_rc != 1 || tw_res.outcome != UserProgramOutcome::kFaulted ||
      tw_res.fault_vector != 6 ||
      TryWait(ud_tw_pid, &tw_res) != kSyscallErrorInvalid) {
    return check;
  }
  ++check;
  // Re-allocate all kMaxUserChildProcesses slots to verify spin_pid and
  // ud_tw_pid freed their slots cleanly.
  for (int i = 0; i < kMaxUserChildProcesses; ++i) {
    pids[i] = Spawn("usertest", usertest::Arg(Mode::kExit));
    if (pids[i] < 1 || pids[i] > kMaxUserChildProcesses) {
      return check;
    }
  }
  for (int i = 0; i < kMaxUserChildProcesses; ++i) {
    if (!Wait(pids[i]).ok()) {
      return check;
    }
  }
  ++check;
  // Point 7: non-writable out_result on an ALREADY-ENDED child must fail with
  // kSyscallErrorFault before HasEnded()/ReapChild() and leave the child
  // reapable afterward.
  const int64_t ended_pid = Spawn("usertest", usertest::Arg(Mode::kExit));
  if (ended_pid < 1 || ended_pid > kMaxUserChildProcesses) {
    return check;
  }
  (void)SpinOnBuffer(nullptr, 0, 25);
  const uint64_t ended_pid_u = static_cast<uint64_t>(ended_pid);
  if (Syscall(kSyscallTryWait, ended_pid_u, 0) != kSyscallErrorFault ||
      Syscall(kSyscallTryWait, ended_pid_u, usertest::kKernelAddress) !=
          kSyscallErrorFault ||
      Syscall(kSyscallTryWait, ended_pid_u, kUnmappedUserAddress) !=
          kSyscallErrorFault ||
      Syscall(kSyscallTryWait, ended_pid_u, ro_addr) != kSyscallErrorFault) {
    return check;
  }
  tw_rc = 0;
  for (int poll = 0; poll < 5000; ++poll) {
    tw_rc = TryWait(ended_pid, &tw_res);
    if (tw_rc != 0) {
      break;
    }
    (void)SpinOnBuffer(nullptr, 0, 1);
  }
  if (tw_rc != 1 || tw_res.outcome != UserProgramOutcome::kExited ||
      tw_res.exit_status != usertest::kExitStatus ||
      TryWait(ended_pid, &tw_res) != kSyscallErrorInvalid) {
    return check;
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
  if (IpcListen(kIpcPortFs) != kSyscallErrorPermission) {
    return 16;
  }
  if (IpcListen(kIpcPortBlock) != kSyscallErrorPermission) {
    return 19;
  }
  if (GetBootModuleCount() != kSyscallErrorPermission) {
    return 17;
  }
  if (ReadBootModule(0, 0, nullptr, 0) != kSyscallErrorPermission) {
    return 18;
  }
  if (Syscall(kSyscallCount, 0, 0, 0) != kSyscallErrorNoSys) {
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
        Syscall(kSyscallCount, 0, 0, 0) != kSyscallErrorNoSys) {
      return 6;
    }
    // Allowed for a driver: returns the number of boot modules.
    if (GetBootModuleCount() < 0) {
      return 7;
    }
    if (IpcListen(kIpcPortBlock) != 0 || IpcListen(kIpcPortBlock) != 0 ||
        IpcCall(kIpcPortBlock, nullptr, 0, nullptr, 0) !=
            kSyscallErrorDeadlock) {
      return 8;
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
      IpcListen(kIpcPortFs) != kSyscallErrorPermission ||
      IpcListen(kIpcPortBlock) != kSyscallErrorPermission ||
      GetBootModuleCount() != kSyscallErrorPermission ||
      ReadBootModule(0, 0, nullptr, 0) != kSyscallErrorPermission ||
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

static int64_t RunDriverNvmeReset(const uint64_t arg) {
  constexpr uint8_t kPciClassMassStorage = 0x01;
  constexpr uint8_t kPciSubclassNvm = 0x08;
  constexpr uint8_t kPciProgIfNvme = 0x02;
  constexpr uint32_t kNvmeMmioAllOnes = 0xFFFFFFFFu;
  const int dev_idx = usertest::DriverDeviceIndex(arg);
  if (PciClaim(dev_idx) != 0) {
    return usertest::kNvmeResetNotNvmeStatus;
  }
  const StatusOr<PciDeviceInfo> info = GetPciInfo(dev_idx);
  if (!info.ok() || info->class_code != kPciClassMassStorage ||
      info->subclass != kPciSubclassNvm || info->prog_if != kPciProgIfNvme ||
      !IsMemoryBar(info->bars[0]) || info->bars[0].size < 2 * 4096) {
    return usertest::kNvmeResetNotNvmeStatus;
  }
  PagedMmioWindow mmio(info->bars[0].phys_base, info->bars[0].size);
  const uint32_t cap_lo = mmio.Read32(nvme::kRegCap);
  const uint32_t cap_hi = mmio.Read32(nvme::kRegCap + 4);
  if (cap_lo == kNvmeMmioAllOnes || cap_hi == kNvmeMmioAllOnes) {
    return usertest::kNvmeResetNotDecodingStatus;
  }
  const uint64_t cap = (uint64_t{cap_hi} << 32) | cap_lo;
  if ((cap & nvme::kCapCssNvm) == 0 || nvme::CapMpsMin(cap) != 0 ||
      nvme::CapMpsMax(cap) < nvme::CapMpsMin(cap) || nvme::CapMqes(cap) < 2) {
    return usertest::kNvmeResetBadCapabilitiesStatus;
  }
  const uint32_t cc = mmio.Read32(nvme::kRegCc);
  const uint32_t csts = mmio.Read32(nvme::kRegCsts);
  if (cc == kNvmeMmioAllOnes || csts == kNvmeMmioAllOnes) {
    return usertest::kNvmeResetNotDecodingStatus;
  }
  const bool was_running =
      ((cc & nvme::kCcEn) != 0) || ((csts & nvme::kCstsRdy) != 0);
  if (cc != 0) {
    mmio.Write32(nvme::kRegCc, 0);
  }
  int64_t timeout_ms = nvme::CapToMs(cap);
  if (timeout_ms < 2000) {
    timeout_ms = 2000;
  }
  const int64_t timeout_ticks = TicksForMs(timeout_ms);
  const PollResult reset =
      PollRegister(mmio, nvme::kRegCsts, nvme::kCstsRdy, 0, timeout_ticks);
  if (reset == PollResult::kNotDecoding) {
    return usertest::kNvmeResetNotDecodingStatus;
  }
  if (reset != PollResult::kMatched) {
    return usertest::kNvmeResetTimeoutStatus;
  }
  return was_running ? usertest::kNvmeResetWasRunningStatus : 0;
}

// The buffers of `Mode::kIpcChecks` and `Mode::kFsChecks`, which are too big
// for the 16 KiB stack: requests and data to write go in `g_ipc_buf_a`,
// replies and data read in `g_ipc_buf_b`. Each process has its own copy.
alignas(8) uint8_t g_ipc_buf_a[kIpcMaxMessageSize];
alignas(8) uint8_t g_ipc_buf_b[kIpcMaxMessageSize];

// How long the servers of `Mode::kIpcChecks` wait for a message that should
// come: far longer than starting a process takes, and no longer than the boot
// self-tests wait for a process to end (at least 2 s), so that a message that
// doesn't come shows in the server's exit status.
constexpr int64_t kIpcTestTimeoutTicks = 2 * kSyscallTicksPerSecond;

// The calls of `IpcTestSubmode::kClient`, which `kEndToEnd` serves: a small
// request, answered with its bytes XORed with `kIpcSmallReplyXor` and status
// `kIpcSmallReplyStatus`; the longest request, answered with its bytes XORed
// with `kIpcLongReplyXor`; a request answered with `kSyscallErrorNoEntry`;
// and a request whose reply may have at most `kIpcCappedReplySize` bytes,
// answered with that many `kIpcCappedReplyByte` bytes. `kClientCallOnce`
// sends the small request too.
constexpr char kIpcSmallRequest[] = "ipc-hello-world";
constexpr int64_t kIpcSmallRequestSize =
    static_cast<int64_t>(sizeof(kIpcSmallRequest) - 1);
constexpr uint8_t kIpcSmallReplyXor = 0x20;
constexpr int64_t kIpcSmallReplyStatus = 123;
constexpr uint8_t kIpcLongReplyXor = 0xA5;
constexpr char kIpcErrorRequest[] = "err";
constexpr int64_t kIpcErrorRequestSize =
    static_cast<int64_t>(sizeof(kIpcErrorRequest) - 1);
constexpr char kIpcCappedRequest[] = "cap";
constexpr int64_t kIpcCappedRequestSize =
    static_cast<int64_t>(sizeof(kIpcCappedRequest) - 1);
constexpr int64_t kIpcCappedReplySize = 8;
constexpr uint8_t kIpcCappedReplyByte = 0x5C;

// The vector of #UD (`kVectorInvalidOpcode` in idt.h).
constexpr int kInvalidOpcodeVector = 6;

using usertest::FsTestSubmode;
using usertest::IpcTestSubmode;

// Returns read-only data as a `T*`, for system calls that must fail to write
// to it.
template <typename T>
static T* ReadOnlyAs() {
  return reinterpret_cast<T*>(const_cast<uint64_t*>(&kReadOnlyWord));
}

// Returns true if the `size` bytes at `a` and at `b` are the same.
static bool BytesEqual(const void* const a,  //
                       const void* const b,  //
                       const int64_t size) {
  const uint8_t* const a_bytes = static_cast<const uint8_t*>(a);
  const uint8_t* const b_bytes = static_cast<const uint8_t*>(b);
  for (int64_t i = 0; i < size; ++i) {
    if (a_bytes[i] != b_bytes[i]) {
      return false;
    }
  }
  return true;
}

// Waits for child process `pid` and returns how it ended as an exit status of
// its parent: `kChildStatusBase` plus its exit status if it exited, else
// `kChildCrashedStatus`.
static int64_t ChildStatus(const int64_t pid) {
  const StatusOr<UserProgramResult> result = Wait(pid);
  if (!result.ok() || result->outcome != UserProgramOutcome::kExited) {
    return usertest::kChildCrashedStatus;
  }
  return usertest::kChildStatusBase + result->exit_status;
}

// Waits at most `kIpcTestTimeoutTicks` for the next message on `port`, writes
// it to `*msg` (and the request of a call to `g_ipc_buf_a`), and returns true
// if it is a call.
static bool ReceiveCall(const int port, IpcReceivedMessage* const msg) {
  return IpcRecv(port,                //
                 g_ipc_buf_a,         //
                 kIpcMaxMessageSize,  //
                 msg,                 //
                 kIpcTestTimeoutTicks) == 1 &&
         msg->kind == IpcMessageKind::kCall;
}

// `IpcTestSubmode::kEndToEnd` on `port`.
static int64_t RunIpcEndToEnd(const int port) {
  // MemMap(): its argument errors, then two mappings of zeroed, writable
  // memory, each followed by an unmapped guard page.
  if (MemMap(0).status() != kSyscallErrorInvalid ||
      MemMap(-kPageSize).status() != kSyscallErrorInvalid ||
      MemMap(kPageSize + 1).status() != kSyscallErrorInvalid ||
      MemMap(kSyscallMemMapMaxSize + kPageSize).status() !=
          kSyscallErrorInvalid) {
    return 1;
  }
  const StatusOr<uintptr_t> map1 = MemMap(2 * kPageSize);
  const StatusOr<uintptr_t> map2 = MemMap(kPageSize);
  if (!map1.ok() || !map2.ok() || *map1 % kPageSize != 0 ||
      *map2 % kPageSize != 0 || *map2 < *map1 + 3 * kPageSize) {
    return 2;
  }
  uint8_t* const p1 = reinterpret_cast<uint8_t*>(*map1);
  for (int64_t i = 0; i < 2 * kPageSize; ++i) {
    if (p1[i] != 0) {
      return 3;
    }
    p1[i] = static_cast<uint8_t>(i & 0xFF);
  }

  // Bad ports, `kIpcPortFs`/`kIpcPortBlock` without driver capability, a port
  // that no process owns, and one that the caller doesn't.
  IpcReceivedMessage msg{};
  if (IpcListen(-1) != kSyscallErrorInvalid ||
      IpcListen(kIpcPortCount) != kSyscallErrorInvalid ||
      IpcListen(kIpcPortFs) != kSyscallErrorPermission ||
      IpcListen(kIpcPortBlock) != kSyscallErrorPermission ||
      IpcRecv(port, g_ipc_buf_a, kIpcMaxMessageSize, &msg, 0) !=
          kSyscallErrorAccess ||
      IpcCall(port, "x", 1, nullptr, 0) != kSyscallErrorNoDevice) {
    return 4;
  }
  // Claiming the port again succeeds.
  if (IpcListen(port) != 0 || IpcListen(port) != 0) {
    return 5;
  }
  // A call to a port that the caller owns fails instead of waiting forever.
  if (IpcCall(port, nullptr, 0, nullptr, 0) != kSyscallErrorDeadlock) {
    return 6;
  }
  // A poll of an empty port returns 0, and bad user buffers fail.
  if (IpcRecv(port, g_ipc_buf_a, kIpcMaxMessageSize, &msg, 0) != 0 ||
      IpcRecv(port, nullptr, 16, &msg, 0) != kSyscallErrorFault ||
      IpcRecv(port, g_ipc_buf_a, 16, ReadOnlyAs<IpcReceivedMessage>(), 0) !=
          kSyscallErrorFault) {
    return 7;
  }

  const int64_t child_pid =
      Spawn("usertest", usertest::IpcChecksArg(IpcTestSubmode::kClient, port));
  if (child_pid < 1) {
    return 8;
  }
  // The child's calls. One that doesn't come means that the child failed.
  // An empty request, with no room for a reply.
  if (!ReceiveCall(port, &msg)) {
    return ChildStatus(child_pid);
  }
  const uint64_t client_id = msg.client_id;
  if (client_id == 0 || msg.size != 0 || msg.reply_cap != 0 ||
      IpcReply(port, msg.token, nullptr, 0, 0) != 0) {
    return 20;
  }
  // The small request.
  if (!ReceiveCall(port, &msg)) {
    return ChildStatus(child_pid);
  }
  if (msg.client_id != client_id || msg.size != kIpcSmallRequestSize ||
      msg.reply_cap != kIpcMaxMessageSize) {
    return 21;
  }
  for (int64_t i = 0; i < msg.size; ++i) {
    g_ipc_buf_b[i] = static_cast<uint8_t>(g_ipc_buf_a[i] ^ kIpcSmallReplyXor);
  }
  if (IpcReply(port,         //
               msg.token,    //
               g_ipc_buf_b,  //
               msg.size,     //
               kIpcSmallReplyStatus) != 0) {
    return 22;
  }
  // The longest request and reply, with the reply's size as the status.
  if (!ReceiveCall(port, &msg)) {
    return ChildStatus(child_pid);
  }
  if (msg.client_id != client_id || msg.size != kIpcMaxMessageSize ||
      msg.reply_cap != kIpcMaxMessageSize) {
    return 23;
  }
  for (int64_t i = 0; i < msg.size; ++i) {
    g_ipc_buf_b[i] = static_cast<uint8_t>(g_ipc_buf_a[i] ^ kIpcLongReplyXor);
  }
  if (IpcReply(port, msg.token, g_ipc_buf_b, msg.size, msg.size) != 0) {
    return 24;
  }
  // An error status.
  if (!ReceiveCall(port, &msg)) {
    return ChildStatus(child_pid);
  }
  if (msg.client_id != client_id || msg.size != kIpcErrorRequestSize ||
      IpcReply(port, msg.token, nullptr, 0, kSyscallErrorNoEntry) != 0) {
    return 25;
  }
  // A reply longer than the client accepts fails, and the call stays in
  // service for one that fits.
  if (!ReceiveCall(port, &msg)) {
    return ChildStatus(child_pid);
  }
  if (msg.client_id != client_id || msg.size != kIpcCappedRequestSize ||
      msg.reply_cap != kIpcCappedReplySize) {
    return 26;
  }
  for (int64_t i = 0; i <= kIpcCappedReplySize; ++i) {
    g_ipc_buf_b[i] = kIpcCappedReplyByte;
  }
  if (IpcReply(port,                     //
               msg.token,                //
               g_ipc_buf_b,              //
               kIpcCappedReplySize + 1,  //
               kIpcCappedReplySize + 1) != kSyscallErrorInvalid) {
    return 27;
  }
  if (IpcReply(port,                 //
               msg.token,            //
               g_ipc_buf_b,          //
               kIpcCappedReplySize,  //
               kIpcCappedReplySize) != 0) {
    return 28;
  }

  // Once the child has ended, the server, which received its calls, gets one
  // notification that it disconnected.
  const int64_t child_status = ChildStatus(child_pid);
  if (child_status != usertest::kChildStatusBase) {
    return child_status;
  }
  if (IpcRecv(port, g_ipc_buf_a, kIpcMaxMessageSize, &msg, 0) != 1 ||
      msg.kind != IpcMessageKind::kClientDisconnect ||
      msg.client_id != client_id || msg.token != 0 || msg.size != 0 ||
      msg.reply_cap != 0) {
    return 29;
  }
  if (IpcRecv(port, g_ipc_buf_a, kIpcMaxMessageSize, &msg, 0) != 0) {
    return 30;
  }
  return 0;
}

// `IpcTestSubmode::kClient` on `port`: the calls that `RunIpcEndToEnd()`
// serves.
static int64_t RunIpcClient(const int port) {
  // The parent owns the port.
  if (IpcListen(port) != kSyscallErrorBusy) {
    return 9;
  }
  int64_t reply_size = -1;
  if (IpcCall(port, nullptr, 0, nullptr, 0, &reply_size) != 0 ||
      reply_size != 0) {
    return 10;
  }
  if (IpcCall(port,                  //
              kIpcSmallRequest,      //
              kIpcSmallRequestSize,  //
              g_ipc_buf_b,           //
              kIpcMaxMessageSize,    //
              &reply_size) != kIpcSmallReplyStatus ||
      reply_size != kIpcSmallRequestSize) {
    return 11;
  }
  for (int64_t i = 0; i < kIpcSmallRequestSize; ++i) {
    if (g_ipc_buf_b[i] !=
        static_cast<uint8_t>(kIpcSmallRequest[i] ^ kIpcSmallReplyXor)) {
      return 12;
    }
  }
  for (int64_t i = 0; i < kIpcMaxMessageSize; ++i) {
    g_ipc_buf_a[i] = static_cast<uint8_t>((i * 17 + 5) & 0xFF);
  }
  if (IpcCall(port,                //
              g_ipc_buf_a,         //
              kIpcMaxMessageSize,  //
              g_ipc_buf_b,         //
              kIpcMaxMessageSize,  //
              &reply_size) != kIpcMaxMessageSize ||
      reply_size != kIpcMaxMessageSize) {
    return 13;
  }
  for (int64_t i = 0; i < kIpcMaxMessageSize; ++i) {
    if (g_ipc_buf_b[i] !=
        static_cast<uint8_t>(g_ipc_buf_a[i] ^ kIpcLongReplyXor)) {
      return 14;
    }
  }
  if (IpcCall(port,                  //
              kIpcErrorRequest,      //
              kIpcErrorRequestSize,  //
              g_ipc_buf_b,           //
              kIpcMaxMessageSize) != kSyscallErrorNoEntry) {
    return 15;
  }
  // The byte after the room for the reply stays as it is.
  g_ipc_buf_b[kIpcCappedReplySize] = 0;
  if (IpcCall(port,                   //
              kIpcCappedRequest,      //
              kIpcCappedRequestSize,  //
              g_ipc_buf_b,            //
              kIpcCappedReplySize,    //
              &reply_size) != kIpcCappedReplySize ||
      reply_size != kIpcCappedReplySize) {
    return 16;
  }
  for (int64_t i = 0; i < kIpcCappedReplySize; ++i) {
    if (g_ipc_buf_b[i] != kIpcCappedReplyByte) {
      return 17;
    }
  }
  if (g_ipc_buf_b[kIpcCappedReplySize] != 0) {
    return 17;
  }
  return 0;
}

// `IpcTestSubmode::kServerPark` on `port`.
static int64_t RunIpcServerPark(const int port) {
  if (IpcListen(port) != 0) {
    return 1;
  }
  IpcReceivedMessage msg{};
  // Waits until the program is killed.
  (void)IpcRecv(port, g_ipc_buf_a, kIpcMaxMessageSize, &msg, -1);
  return 2;
}

// `IpcTestSubmode::kServerFaultMidRpc` on `port`.
static int64_t RunIpcServerFaultMidRpc(const int port) {
  if (IpcListen(port) != 0) {
    return 1;
  }
  IpcReceivedMessage msg{};
  if (!ReceiveCall(port, &msg)) {
    return 2;
  }
  asm volatile("ud2");
  return 3;
}

// `IpcTestSubmode::kClientCallOnce` on `port`.
static int64_t RunIpcClientCallOnce(const int port) {
  const int64_t rc = IpcCall(port,                  //
                             kIpcSmallRequest,      //
                             kIpcSmallRequestSize,  //
                             g_ipc_buf_b,           //
                             kIpcSmallRequestSize);
  return rc < 0 ? -rc : usertest::kIpcCallAnsweredStatus;
}

// `IpcTestSubmode::kMemMapGuardFault`.
static int64_t RunMemMapGuardFault() {
  const StatusOr<uintptr_t> mapped = MemMap(kPageSize);
  if (!mapped.ok()) {
    return 1;
  }
  *Launder(reinterpret_cast<volatile uint8_t*>(*mapped + kPageSize)) = 0x42;
  return 2;
}

// `IpcTestSubmode::kServerHoldCall` on `port` and `port + 1`.
static int64_t RunIpcServerHoldCall(const int port) {
  // System calls 1 and 2.
  if (IpcListen(port) != 0 || IpcListen(port + 1) != 0) {
    return 1;
  }
  // System call 3.
  IpcReceivedMessage msg{};
  if (!ReceiveCall(port, &msg)) {
    return 2;
  }
  // System call 4 (`kServerHoldCallParkedSyscall`): waits, with the call
  // unanswered, until the program is killed.
  (void)IpcRecv(port + 1, g_ipc_buf_a, kIpcMaxMessageSize, &msg, -1);
  return 3;
}

// `IpcTestSubmode::kServerLateReply` on `port`.
static int64_t RunIpcServerLateReply(const int port) {
  // System calls 1 and 2.
  if (IpcListen(port) != 0) {
    return 1;
  }
  IpcReceivedMessage call{};
  if (!ReceiveCall(port, &call)) {
    return 2;
  }
  // System call 3 (`kServerLateReplyWaitSyscall`): the caller ends instead of
  // getting a reply.
  IpcReceivedMessage msg{};
  if (IpcRecv(port,                //
              g_ipc_buf_a,         //
              kIpcMaxMessageSize,  //
              &msg,                //
              kIpcTestTimeoutTicks) != 1 ||
      msg.kind != IpcMessageKind::kClientDisconnect ||
      msg.client_id != call.client_id) {
    return 3;
  }
  // A reply longer than `reply_cap` fails without consuming the canceled call;
  // a reply that fits is dropped and ends the call.
  if (call.reply_cap != kIpcSmallRequestSize ||
      IpcReply(port,                //
               call.token,          //
               g_ipc_buf_b,         //
               call.reply_cap + 1,  //
               0) != kSyscallErrorInvalid ||
      IpcReply(port,                  //
               call.token,            //
               kIpcSmallRequest,      //
               kIpcSmallRequestSize,  //
               0) != 0) {
    return 4;
  }
  if (IpcReply(port, call.token, nullptr, 0, 0) != kSyscallErrorInvalid) {
    return 5;
  }
  return 0;
}

// `IpcTestSubmode::kMemMapLimits`.
static int64_t RunMemMapLimits() {
  constexpr int kChunks =
      static_cast<int>(kSyscallMemMapMaxTotalSize / kSyscallMemMapMaxSize);
  static_assert(kChunks * kSyscallMemMapMaxSize == kSyscallMemMapMaxTotalSize);
  for (int i = 0; i < kChunks; ++i) {
    const StatusOr<uintptr_t> mapped = MemMap(kSyscallMemMapMaxSize);
    if (!mapped.ok() || (*mapped % kPageSize) != 0) {
      return 1;
    }
    volatile uint8_t* const bytes =
        reinterpret_cast<volatile uint8_t*>(*mapped);
    if (bytes[0] != 0 || bytes[kSyscallMemMapMaxSize - 1] != 0) {
      return 2;
    }
    bytes[0] = static_cast<uint8_t>(i + 1);
    bytes[kSyscallMemMapMaxSize - 1] = static_cast<uint8_t>(0xFF - i);
    if (bytes[0] != static_cast<uint8_t>(i + 1) ||
        bytes[kSyscallMemMapMaxSize - 1] != static_cast<uint8_t>(0xFF - i)) {
      return 3;
    }
  }
  if (MemMap(kPageSize).status() != kSyscallErrorNoMemory) {
    return 4;
  }
  return 0;
}

// `IpcTestSubmode::kMemMapPmmRollback`.
static int64_t RunMemMapPmmRollback() {
  // System call 1: map 1 page so the page table for this 2 MiB window exists.
  const StatusOr<uintptr_t> first = MemMap(kPageSize);
  if (!first.ok()) {
    return 1;
  }
  // System call 2: wait until the boot self-test drains PMM to 2 free frames.
  char byte = 0;
  if (Read(&byte, 1) != 1) {
    return 1;
  }
  // Needs 4 frames; only 2 are free, so all allocated frames must roll back.
  if (MemMap(4 * kPageSize).status() != kSyscallErrorNoMemory) {
    return 2;
  }
  // If both frames were freed back to the PMM, mapping 2 pages succeeds.
  const StatusOr<uintptr_t> recovered = MemMap(2 * kPageSize);
  if (!recovered.ok()) {
    return 3;
  }
  return 0;
}

// Runs `Mode::kIpcChecks` as `arg` says.
static int64_t RunIpcChecks(const uint64_t arg) {
  const IpcTestSubmode submode =
      static_cast<IpcTestSubmode>(usertest::DriverSubmode(arg));
  const int port = usertest::DriverDeviceIndex(arg);
  // The ports that the submode uses: none, `port`, or `port` and `port + 1`.
  const bool no_port = (submode == IpcTestSubmode::kMemMapGuardFault ||
                        submode == IpcTestSubmode::kMemMapLimits ||
                        submode == IpcTestSubmode::kMemMapPmmRollback);
  const int ports = no_port                                        ? 0
                    : (submode == IpcTestSubmode::kServerHoldCall) ? 2
                                                                   : 1;
  if (ports > 0 && (port < kIpcFirstUserPort || port + ports > kIpcPortCount)) {
    return usertest::kBadSubmodeStatus;
  }
  switch (submode) {
    case IpcTestSubmode::kEndToEnd:
      return RunIpcEndToEnd(port);
    case IpcTestSubmode::kClient:
      return RunIpcClient(port);
    case IpcTestSubmode::kServerPark:
      return RunIpcServerPark(port);
    case IpcTestSubmode::kServerFaultMidRpc:
      return RunIpcServerFaultMidRpc(port);
    case IpcTestSubmode::kClientCallOnce:
      return RunIpcClientCallOnce(port);
    case IpcTestSubmode::kMemMapGuardFault:
      return RunMemMapGuardFault();
    case IpcTestSubmode::kServerHoldCall:
      return RunIpcServerHoldCall(port);
    case IpcTestSubmode::kServerLateReply:
      return RunIpcServerLateReply(port);
    case IpcTestSubmode::kMemMapLimits:
      return RunMemMapLimits();
    case IpcTestSubmode::kMemMapPmmRollback:
      return RunMemMapPmmRollback();
  }
  return usertest::kBadSubmodeStatus;
}

// The file data of `FsTestSubmode::kFullSuite`: `kFsDataBlocks` blocks of
// `FsPatternByte()` bytes, which fill ramfs's direct blocks
// (`kFsDirectBlocks`, ramfs::kDirectBlocks) and go on into its first
// single-indirect block, and `kFsSparseData` at `kFsSparseOffset`, in the
// range of its double-indirect blocks, after a hole.
constexpr int64_t kFsDirectBlocks = 12;
constexpr int64_t kFsDataBlocks = kFsDirectBlocks + 2;
constexpr int64_t kFsSparseOffset = int64_t{8} << 20;
constexpr char kFsSparseData[] = "sparse tail";
constexpr int64_t kFsSparseDataSize =
    static_cast<int64_t>(sizeof(kFsSparseData) - 1);
// The bytes of a read across the boundary of the direct and indirect blocks.
constexpr int64_t kFsStraddleSize = 200;
// Open() flags that create a file that must not exist yet, for reading and
// writing.
constexpr uint64_t kFsCreateNewFlags =
    kFsOpenReadWrite | kFsOpenCreate | kFsOpenExclusive;
// The `kLeakAndFault` children of `kFullSuite`, which open 512 files in all
// (`kMaxOpenFiles` each): twice as many as ramfs has handles and fnodes for
// (ramfs::kMaxOpenHandles and ramfs::kMaxFnodes), so that ramfs runs out of
// them unless it frees the files of each child when the child ends.
constexpr int kFsLeakRounds = 16;
static_assert(kMaxOpenFiles < 100, "kLeakAndFault names files with 2 digits");

// The byte at `offset` of the data that `kFullSuite` writes.
static uint8_t FsPatternByte(const int64_t offset) {
  return static_cast<uint8_t>((offset / kPageSize * 29 + offset % kPageSize) &
                              0xFF);
}

// Returns true if `entry`, which ReadDir() wrote, describes the regular file
// `name` (null-terminated) with fnode ID `fnode_id`, with zeros after the name
// and in the reserved bytes.
static bool IsFileEntry(const FsDirEntry& entry,  //
                        const char* const name,   //
                        const uint64_t fnode_id) {
  const int64_t name_len = StringLength(name);
  if (entry.fnode_id != fnode_id || entry.type != FsNodeType::kRegular ||
      entry.name_len != name_len) {
    return false;
  }
  for (int64_t i = 0; i <= kFsMaxNameLength; ++i) {
    if (entry.name[i] != (i < name_len ? name[i] : '\0')) {
      return false;
    }
  }
  for (const uint8_t byte : entry.reserved) {
    if (byte != 0) {
      return false;
    }
  }
  return true;
}

// The checks of `kFullSuite` that need no file: arguments that the kernel
// rejects before it calls the server, and calls of the server's port.
static int64_t CheckFsArguments() {
  // Buffers that user code may not access.
  FsStat st{};
  if (Open(nullptr, 4, kFsOpenRead) != kSyscallErrorFault ||
      Open(reinterpret_cast<const char*>(usertest::kKernelAddress),  //
           4,                                                        //
           kFsOpenRead) != kSyscallErrorFault ||
      Stat(nullptr, 4, &st) != kSyscallErrorFault ||
      Stat("/", 1, ReadOnlyAs<FsStat>()) != kSyscallErrorFault ||
      MkDir(nullptr, 4) != kSyscallErrorFault ||
      Unlink(nullptr, 4) != kSyscallErrorFault ||
      Rename(nullptr, 4, "/b", 2) != kSyscallErrorFault ||
      Rename("/a", 2, nullptr, 4) != kSyscallErrorFault) {
    return 1;
  }
  // Invalid flags: an unknown bit, access mode 3, kFsOpenExclusive without
  // kFsOpenCreate, and kFsOpenTruncate without write access.
  constexpr uint64_t kInvalidFlags[] = {
      kFsOpenFlagsMask + 1,
      kFsOpenAccessMask,
      kFsOpenReadWrite | kFsOpenExclusive,
      kFsOpenRead | kFsOpenTruncate,
  };
  for (const uint64_t flags : kInvalidFlags) {
    if (Open("/", flags) != kSyscallErrorInvalid) {
      return 2;
    }
  }
  // Path lengths of 0, kFsMaxPathLength (which the kernel passes on: the
  // server finds no "a"), kFsMaxPathLength + 1 and -1, and a NUL byte in a
  // path.
  for (int64_t i = 0; i <= kFsMaxPathLength; ++i) {
    g_ipc_buf_a[i] = static_cast<uint8_t>(i % 2 == 0 ? '/' : 'a');
  }
  const char* const long_path = reinterpret_cast<const char*>(g_ipc_buf_a);
  if (Stat(long_path, 0, &st) != kSyscallErrorNoEntry ||
      Stat(nullptr, 0, &st) != kSyscallErrorNoEntry ||
      Open(nullptr, 0, kFsOpenRead) != kSyscallErrorNoEntry ||
      MkDir(nullptr, 0) != kSyscallErrorNoEntry ||
      Unlink(nullptr, 0) != kSyscallErrorNoEntry ||
      Rename(nullptr, 0, "/b", 2) != kSyscallErrorNoEntry ||
      Rename("/a", 2, nullptr, 0) != kSyscallErrorNoEntry ||
      Stat(long_path, kFsMaxPathLength, &st) != kSyscallErrorNoEntry ||
      Open(long_path, kFsMaxPathLength + 1, kFsOpenRead) !=
          kSyscallErrorNameTooLong ||
      Rename("/a", 2, long_path, kFsMaxPathLength + 1) !=
          kSyscallErrorNameTooLong ||
      MkDir(long_path, -1) != kSyscallErrorInvalid ||
      Unlink("/a\0b", 4) != kSyscallErrorInvalid) {
    return 3;
  }
  // Only the kernel's filesystem system calls call the filesystem server.
  if (IpcCall(kIpcPortFs, g_ipc_buf_a, 1, g_ipc_buf_b, kIpcMaxMessageSize) !=
      kSyscallErrorPermission) {
    return 4;
  }
  return 0;
}

// The checks of `kFullSuite` of bad descriptors and of the per-process limit
// of `kMaxOpenFiles` descriptors, which Open() assigns lowest first.
static int64_t CheckFsDescriptorLimit() {
  if (Close(-1) != kSyscallErrorBadFd ||
      Close(kMaxOpenFiles) != kSyscallErrorBadFd ||
      Close(0) != kSyscallErrorBadFd ||
      FileRead(0, g_ipc_buf_b, 4) != kSyscallErrorBadFd ||
      FileWrite(0, "x", 1) != kSyscallErrorBadFd ||
      Seek(0, 0, kFsSeekSet) != kSyscallErrorBadFd ||
      ReadDir(0, nullptr, 0) != kSyscallErrorBadFd) {
    return 5;
  }
  for (int fd = 0; fd < kMaxOpenFiles; ++fd) {
    if (Open("/", kFsOpenRead | kFsOpenDirectory) != fd) {
      return 6;
    }
  }
  if (Open("/", kFsOpenRead | kFsOpenDirectory) != kSyscallErrorTooManyFiles) {
    return 7;
  }
  for (int fd = 0; fd < kMaxOpenFiles; ++fd) {
    if (Close(fd) != 0) {
      return 8;
    }
  }
  return 0;
}

// The checks of `kFullSuite` of arguments that need descriptors: of a
// regular file, read-write and read-only, and of a directory.
static int64_t CheckFsDescriptorArguments() {
  if (MkDir("/suite") != 0 || MkDir("/suite/sub") != 0) {
    return 9;
  }
  const int64_t file_fd = Open("/suite/args", kFsCreateNewFlags);
  const int64_t read_only_fd = Open("/suite/args", kFsOpenRead);
  const int64_t dir_fd = Open("/suite", kFsOpenRead | kFsOpenDirectory);
  if (file_fd < 0 || read_only_fd < 0 || dir_fd < 0) {
    return 10;
  }
  // Sizes beyond kSyscallFsMaxIoSize, and buffers that user code may not
  // access.
  if (FileRead(file_fd, g_ipc_buf_b, kSyscallFsMaxIoSize + 1) !=
          kSyscallErrorInvalid ||
      FileWrite(file_fd, g_ipc_buf_a, kSyscallFsMaxIoSize + 1) !=
          kSyscallErrorInvalid ||
      FileRead(file_fd, g_ipc_buf_b, -1) != kSyscallErrorInvalid ||
      FileRead(file_fd, ReadOnlyAs<void>(), 8) != kSyscallErrorFault ||
      FileWrite(file_fd,                                                  //
                reinterpret_cast<const void*>(usertest::kKernelAddress),  //
                8) != kSyscallErrorFault ||
      FileWrite(file_fd, nullptr, 1) != kSyscallErrorFault) {
    return 11;
  }
  // ReadDir() of more than kSyscallFsMaxDirEntries entries or into a buffer
  // that user code may not write fails, and of no entries returns 0 without
  // moving on: the directory's two entries come next.
  FsDirEntry entries[2];
  if (ReadDir(dir_fd, entries, kSyscallFsMaxDirEntries + 1) !=
          kSyscallErrorInvalid ||
      ReadDir(dir_fd, ReadOnlyAs<FsDirEntry>(), 1) != kSyscallErrorFault ||
      ReadDir(dir_fd, nullptr, 0) != 0 || ReadDir(dir_fd, entries, 2) != 2 ||
      ReadDir(dir_fd, entries, 2) != 0) {
    return 12;
  }
  // Seek() with an invalid whence or to an offset out of range.
  if (Seek(file_fd, 0, kFsSeekEnd + 1) != kSyscallErrorInvalid ||
      Seek(file_fd, 0, -1) != kSyscallErrorInvalid ||
      Seek(file_fd, -1, kFsSeekSet) != kSyscallErrorInvalid ||
      Seek(file_fd, kFsMaxFileSize + 1, kFsSeekSet) != kSyscallErrorInvalid) {
    return 13;
  }
  // Descriptors of the wrong kind, or without the access needed.
  if (FileRead(dir_fd, g_ipc_buf_b, 1) != kSyscallErrorIsDir ||
      ReadDir(file_fd, entries, 1) != kSyscallErrorNotDir ||
      FileWrite(read_only_fd, "x", 1) != kSyscallErrorBadFd) {
    return 14;
  }
  // At kFsMaxFileSize, an empty write returns 0 and a longer one fails; the
  // file stays empty.
  FsStat st{};
  if (Seek(file_fd, kFsMaxFileSize, kFsSeekSet) != kFsMaxFileSize ||
      FileWrite(file_fd, "x", 0) != 0 ||
      FileWrite(file_fd, "x", 1) != kSyscallErrorFileTooBig ||
      Stat("/suite/args", &st) != 0 || st.size != 0) {
    return 15;
  }
  if (Close(file_fd) != 0 || Close(read_only_fd) != 0 || Close(dir_fd) != 0 ||
      Unlink("/suite/args") != 0) {
    return 16;
  }
  return 0;
}

// The checks of `kFullSuite` of file data: written, then read back whole and
// across the boundary of the direct and indirect blocks, and a hole.
static int64_t CheckFsFileData() {
  const int64_t fd =
      Open("/suite/sub/../data.bin", kFsOpenReadWrite | kFsOpenCreate);
  if (fd < 0) {
    return 17;
  }
  for (int64_t block = 0; block < kFsDataBlocks; ++block) {
    for (int64_t i = 0; i < kPageSize; ++i) {
      g_ipc_buf_a[i] = FsPatternByte(block * kPageSize + i);
    }
    if (FileWrite(fd, g_ipc_buf_a, kPageSize) != kPageSize) {
      return 18;
    }
  }
  // At the end of the file, a read returns 0.
  if (FileRead(fd, g_ipc_buf_b, 1) != 0 || Seek(fd, 0, kFsSeekSet) != 0) {
    return 19;
  }
  for (int64_t block = 0; block < kFsDataBlocks; ++block) {
    if (FileRead(fd, g_ipc_buf_b, kPageSize) != kPageSize) {
      return 20;
    }
    for (int64_t i = 0; i < kPageSize; ++i) {
      if (g_ipc_buf_b[i] != FsPatternByte(block * kPageSize + i)) {
        return 20;
      }
    }
  }
  constexpr int64_t kStraddleOffset =
      kFsDirectBlocks * kPageSize - kFsStraddleSize / 2;
  if (Seek(fd, kStraddleOffset, kFsSeekSet) != kStraddleOffset ||
      FileRead(fd, g_ipc_buf_b, kFsStraddleSize) != kFsStraddleSize) {
    return 21;
  }
  for (int64_t i = 0; i < kFsStraddleSize; ++i) {
    if (g_ipc_buf_b[i] != FsPatternByte(kStraddleOffset + i)) {
      return 21;
    }
  }
  if (Seek(fd, kFsSparseOffset, kFsSeekSet) != kFsSparseOffset ||
      FileWrite(fd, kFsSparseData, kFsSparseDataSize) != kFsSparseDataSize) {
    return 22;
  }
  FsStat st{};
  if (Stat("/suite/data.bin", &st) != 0 || st.type != FsNodeType::kRegular ||
      st.size != kFsSparseOffset + kFsSparseDataSize) {
    return 23;
  }
  // The hole before the data reads as zeros.
  constexpr int64_t kHoleBytes = 4;
  if (Seek(fd, kFsSparseOffset - kHoleBytes, kFsSeekSet) !=
          kFsSparseOffset - kHoleBytes ||
      FileRead(fd, g_ipc_buf_b, kHoleBytes + kFsSparseDataSize) !=
          kHoleBytes + kFsSparseDataSize) {
    return 24;
  }
  for (int64_t i = 0; i < kHoleBytes; ++i) {
    if (g_ipc_buf_b[i] != 0) {
      return 24;
    }
  }
  if (!BytesEqual(g_ipc_buf_b + kHoleBytes, kFsSparseData, kFsSparseDataSize)) {
    return 24;
  }
  if (Close(fd) != 0) {
    return 25;
  }
  return 0;
}

// The checks of `kFullSuite` of Rename(), of ReadDir() entries (read twice,
// with a rewind in between), and of reading a file that was unlinked while
// open.
static int64_t CheckFsRenameAndUnlink() {
  constexpr char kMovedPath[] = "/suite/sub/moved.bin";
  FsStat st{};
  if (Rename("/suite/data.bin", kMovedPath) != 0 ||
      Stat(kMovedPath, &st) != 0) {
    return 26;
  }
  const int64_t dir_fd = Open("/suite/sub", kFsOpenRead | kFsOpenDirectory);
  if (dir_fd < 0) {
    return 27;
  }
  for (int pass = 0; pass < 2; ++pass) {
    // The kernel writes every byte of the entry.
    FsDirEntry entry;
    __builtin_memset(static_cast<void*>(&entry), 0xA5, sizeof(entry));
    if (ReadDir(dir_fd, &entry, 1) != 1 ||
        !IsFileEntry(entry, "moved.bin", st.fnode_id) ||
        ReadDir(dir_fd, &entry, 1) != 0 || Seek(dir_fd, 0, kFsSeekSet) != 0) {
      return 28;
    }
  }
  if (Close(dir_fd) != 0) {
    return 29;
  }
  const int64_t fd = Open(kMovedPath, kFsOpenRead);
  if (fd < 0 || Unlink(kMovedPath) != 0 ||
      Stat(kMovedPath, &st) != kSyscallErrorNoEntry) {
    return 30;
  }
  // The data is still there.
  if (FileRead(fd, g_ipc_buf_b, kPageSize) != kPageSize) {
    return 31;
  }
  for (int64_t i = 0; i < kPageSize; ++i) {
    if (g_ipc_buf_b[i] != FsPatternByte(i)) {
      return 31;
    }
  }
  if (Seek(fd, kFsSparseOffset, kFsSeekSet) != kFsSparseOffset ||
      FileRead(fd, g_ipc_buf_b, kFsSparseDataSize) != kFsSparseDataSize ||
      !BytesEqual(g_ipc_buf_b, kFsSparseData, kFsSparseDataSize) ||
      Close(fd) != 0) {
    return 31;
  }
  return 0;
}

// The check of `kFullSuite` that ramfs frees the files of processes that end
// with files open: `kFsLeakRounds` `kLeakAndFault` children, one after the
// other, which a ramfs that keeps their files can't all serve.
static int64_t CheckFsCrashReclamation() {
  for (int round = 0; round < kFsLeakRounds; ++round) {
    const int64_t pid =
        Spawn("usertest", usertest::FsChecksArg(FsTestSubmode::kLeakAndFault));
    if (pid < 1) {
      return 32;
    }
    const StatusOr<UserProgramResult> result = Wait(pid);
    if (!result.ok()) {
      return 32;
    }
    if (result->outcome == UserProgramOutcome::kExited) {
      return usertest::kChildStatusBase + result->exit_status;
    }
    if (result->outcome != UserProgramOutcome::kFaulted ||
        result->fault_vector != kInvalidOpcodeVector) {
      return usertest::kChildCrashedStatus;
    }
  }
  return 0;
}

// `FsTestSubmode::kFullSuite`.
static int64_t RunFsFullSuite() {
  constexpr int64_t (*const kSteps[])() = {
      CheckFsArguments, CheckFsDescriptorLimit, CheckFsDescriptorArguments,
      CheckFsFileData,  CheckFsRenameAndUnlink, CheckFsCrashReclamation,
  };
  for (int64_t (*const step)() : kSteps) {
    const int64_t status = step();
    if (status != 0) {
      return status;
    }
  }
  if (Unlink("/suite/sub") != 0 || Unlink("/suite") != 0) {
    return 33;
  }
  return 0;
}

// `FsTestSubmode::kLeakAndFault`.
static int64_t RunFsLeakAndFault() {
  for (int i = 0; i < kMaxOpenFiles; ++i) {
    char path[] = "/leak00";
    path[5] = static_cast<char>('0' + i / 10);
    path[6] = static_cast<char>('0' + i % 10);
    const int64_t fd = Open(path, kFsCreateNewFlags);
    if (fd < 0) {
      return 1;
    }
    if (FileWrite(fd, path, 1) != 1) {
      return 2;
    }
    if (Unlink(path) != 0) {
      return 3;
    }
  }
  asm volatile("ud2");
  return 4;
}

// `FsTestSubmode::kSeedDocs`.
static int64_t RunFsSeedDocs() {
  if (MkDir("/docs") != 0) {
    return 1;
  }
  const int64_t fd =
      Open("/docs/readme.txt", kFsOpenWrite | kFsOpenCreate | kFsOpenTruncate);
  if (fd < 0) {
    return 2;
  }
  constexpr char kText[] = "hello from ramfs\n";
  constexpr int64_t kTextSize = static_cast<int64_t>(sizeof(kText) - 1);
  if (FileWrite(fd, kText, kTextSize) != kTextSize) {
    return 3;
  }
  if (Close(fd) != 0) {
    return 4;
  }
  return 0;
}

// `FsTestSubmode::kStaleFd`.
static int64_t RunFsStaleFd() {
  constexpr char kPath[] = "/stale";
  constexpr char kData[] = "data";
  constexpr int64_t kDataSize = static_cast<int64_t>(sizeof(kData) - 1);
  // System calls 1 and 2.
  const int64_t fd = Open(kPath, kFsCreateNewFlags);
  if (fd < 0 || FileWrite(fd, kData, kDataSize) != kDataSize) {
    return 1;
  }
  // System call 3 (`kStaleFdReadSyscall`): the boot self-test replaces ramfs,
  // then types a byte.
  char byte = 0;
  if (Read(&byte, 1) != 1) {
    return 2;
  }
  // The file belonged to the ramfs that ended.
  if (Seek(fd, 0, kFsSeekSet) != kSyscallErrorStale ||
      FileRead(fd, g_ipc_buf_b, kDataSize) != kSyscallErrorStale ||
      FileWrite(fd, kData, kDataSize) != kSyscallErrorStale ||
      ReadDir(fd, nullptr, 0) != kSyscallErrorStale) {
    return 3;
  }
  // Close() frees the descriptor all the same.
  if (Close(fd) != 0) {
    return 4;
  }
  if (Close(fd) != kSyscallErrorBadFd) {
    return 5;
  }
  // The new ramfs has no files, and serves new ones with the freed
  // descriptor.
  FsStat st{};
  if (Stat(kPath, &st) != kSyscallErrorNoEntry) {
    return 6;
  }
  const int64_t new_fd = Open(kPath, kFsCreateNewFlags);
  if (new_fd != fd) {
    return 7;
  }
  if (FileWrite(new_fd, kData, kDataSize) != kDataSize ||
      Seek(new_fd, 0, kFsSeekSet) != 0 ||
      FileRead(new_fd, g_ipc_buf_b, kDataSize) != kDataSize ||
      !BytesEqual(g_ipc_buf_b, kData, kDataSize)) {
    return 8;
  }
  if (Close(new_fd) != 0 || Unlink(kPath) != 0) {
    return 9;
  }
  return 0;
}

// `FsTestSubmode::kFailedOpenOnly`.
static int64_t RunFsFailedOpenOnly() {
  return Open("/missing", kFsOpenRead) == kSyscallErrorNoEntry ? 0 : 1;
}

// `FsTestSubmode::kMalformedServer`.
static int64_t RunFsMalformedServer() {
  // System calls 1 and 2: malformed FsOpenReply (type kNone and handle 0).
  if (Open("/bad_type", kFsOpenRead) != kSyscallErrorIo) {
    return 1;
  }
  if (Open("/zero_handle", kFsOpenRead) != kSyscallErrorIo) {
    return 2;
  }
  // System calls 3..6: malformed FsStat (type kNone, fnode_id 0, size -1,
  // nlink 0).
  FsStat st{};
  for (int i = 0; i < 4; ++i) {
    if (Stat("/s", &st) != kSyscallErrorIo) {
      return 3;
    }
  }
  // System call 7: open a directory on the initial server.
  const int64_t dir_fd = Open("/dir", kFsOpenRead | kFsOpenDirectory);
  if (dir_fd != 0) {
    return 4;
  }
  // System calls 8..13: malformed FsDirEntry (fnode_id 0, type kNone, ".",
  // "..", '/' in name, and NUL in name).
  FsDirEntry entry{};
  for (int i = 0; i < 6; ++i) {
    if (ReadDir(dir_fd, &entry, 1) != kSyscallErrorIo) {
      return 5;
    }
  }
  // System call 14: open a regular file on the initial server.
  const int64_t file_fd = Open("/file", kFsOpenReadWrite);
  if (file_fd != 1) {
    return 6;
  }
  // System calls 15..16: malformed FileRead (status > size, reply_size !=
  // status).
  if (FileRead(file_fd, g_ipc_buf_a, 4) != kSyscallErrorIo ||
      FileRead(file_fd, g_ipc_buf_a, 4) != kSyscallErrorIo) {
    return 7;
  }
  // System calls 17..18: malformed FileWrite (status > size, 0 for size > 0).
  if (FileWrite(file_fd, "abcd", 4) != kSyscallErrorIo ||
      FileWrite(file_fd, "abcd", 4) != kSyscallErrorIo) {
    return 8;
  }
  // System calls 19..21: malformed Seek (wrong kFsSeekSet offset, past
  // kFsMaxFileSize, nonzero kFsSeekSet on directory).
  if (Seek(file_fd, 4, kFsSeekSet) != kSyscallErrorIo) {
    return 9;
  }
  if (Seek(file_fd, 0, kFsSeekEnd) != kSyscallErrorIo) {
    return 10;
  }
  if (Seek(dir_fd, 0, kFsSeekSet) != kSyscallErrorIo) {
    return 11;
  }
  // System calls 22..24: malformed positive status on MkDir, Unlink, Rename.
  if (MkDir("/d") != kSyscallErrorIo) {
    return 12;
  }
  if (Unlink("/u") != kSyscallErrorIo) {
    return 13;
  }
  if (Rename("/a", "/b") != kSyscallErrorIo) {
    return 14;
  }
  // System call 25 (`kMalformedServerReadSyscall`): wait until the boot
  // self-test replaces the server and types a byte, then exit with `dir_fd`
  // and `file_fd` still open.
  char byte = 0;
  if (Read(&byte, 1) != 1) {
    return 15;
  }
  return 0;
}

// `FsTestSubmode::kInterleavedServers`.
static int64_t RunFsInterleavedServers() {
  // System calls 1..3: descriptors 0-2 on the initial server.
  if (Open("/a", kFsOpenRead) != 0) {
    return 1;
  }
  if (Open("/b", kFsOpenRead) != 1) {
    return 2;
  }
  if (Open("/c", kFsOpenRead) != 2) {
    return 3;
  }
  // System call 4 (`kInterleavedServersReadSyscall`): the boot self-test
  // replaces the server, then types a byte.
  char byte = 0;
  if (Read(&byte, 1) != 1) {
    return 4;
  }
  // Descriptors 0 and 2 belong to the server that is gone: Close() frees them
  // without a request to the new one.
  if (Close(0) != 0) {
    return 5;
  }
  if (Close(2) != 0) {
    return 6;
  }
  // The new server's files get them back, so that the servers of descriptors
  // 0-2 alternate: the new one, the one that is gone, the new one.
  if (Open("/d", kFsOpenRead) != 0) {
    return 7;
  }
  if (Open("/e", kFsOpenRead) != 2) {
    return 8;
  }
  return 0;
}

// Returns the `SyscallSpinChecksum()` of boot module `index`, read with
// BootModuleRead() calls of `chunk` bytes into `g_ipc_buf_a`, or -1 if a call
// fails or returns more than `chunk` bytes, or if the module doesn't hold
// `size` bytes.
static int64_t ChecksumBootModule(const int64_t index,  //
                                  const int64_t size,   //
                                  const int64_t chunk) {
  int64_t checksum = kSyscallSpinChecksumInit;
  int64_t offset = 0;
  for (;;) {
    const int64_t got = ReadBootModule(index, offset, g_ipc_buf_a, chunk);
    if (got < 0 || got > chunk) {
      return -1;
    }
    if (got == 0) {
      break;
    }
    checksum = SyscallSpinChecksum(g_ipc_buf_a, got, checksum);
    offset += got;
  }
  return offset == size ? checksum : -1;
}

// Compares regular file `fd`, at offset 0, with the data of archive member
// `entry`, which `reader` read. Returns 0 if they are the same, 12 if a read
// fails, or 13 if they differ or the file is longer.
static int64_t CompareFileWithArchive(const int64_t fd,     //
                                      tar::Reader& reader,  //
                                      const tar::Entry& entry) {
  for (int64_t pos = 0; pos < entry.size;) {
    const int64_t left = entry.size - pos;
    const int64_t chunk =
        (left < kSyscallFsMaxIoSize) ? left : kSyscallFsMaxIoSize;
    if (reader.ReadData(entry, pos, g_ipc_buf_a, chunk) != chunk ||
        FileRead(fd, g_ipc_buf_b, chunk) != chunk) {
      return 12;
    }
    if (!BytesEqual(g_ipc_buf_a, g_ipc_buf_b, chunk)) {
      return 13;
    }
    pos += chunk;
  }
  return FileRead(fd, g_ipc_buf_b, 1) == 0 ? 0 : 13;
}

// `FsTestSubmode::kVerifyRootfs`.
static int64_t RunFsVerifyRootfs() {
  const int64_t index = FindBootModule(kRootfsBootModuleName);
  if (index < 0) {
    return 1;
  }
  const int64_t count = GetBootModuleCount();
  const StatusOr<BootModuleInfo> info = GetBootModuleInfo(index);
  if (count <= index || !info.ok() || info->size <= 0) {
    return 2;
  }
  const int64_t size = info->size;

  // BootModuleInfo() checks the index before the buffer.
  const auto get_info = [](const int64_t i, const uintptr_t out_info) {
    return Syscall(kSyscallBootModuleInfo, static_cast<uint64_t>(i), out_info);
  };
  BootModuleInfo scratch{};
  const uintptr_t scratch_addr = reinterpret_cast<uintptr_t>(&scratch);
  const uintptr_t read_only_addr =
      reinterpret_cast<uintptr_t>(ReadOnlyAs<BootModuleInfo>());
  if (get_info(count, scratch_addr) != kSyscallErrorInvalid ||
      get_info(-1, scratch_addr) != kSyscallErrorInvalid ||
      get_info(count, 0) != kSyscallErrorInvalid ||
      get_info(index, usertest::kKernelAddress) != kSyscallErrorFault ||
      get_info(index, read_only_addr) != kSyscallErrorFault) {
    return 3;
  }

  // BootModuleRead() checks its arguments, then the whole buffer (even where
  // the module has fewer bytes left), and reads nothing at or past the end.
  void* const buffer = g_ipc_buf_b;
  void* const kernel_buffer = reinterpret_cast<void*>(usertest::kKernelAddress);
  void* const past_stack_top =
      reinterpret_cast<void*>(usertest::kStackTop - 16);
  if (ReadBootModule(count, 0, buffer, 1) != kSyscallErrorInvalid ||
      ReadBootModule(-1, 0, buffer, 1) != kSyscallErrorInvalid ||
      ReadBootModule(index, -1, buffer, 1) != kSyscallErrorInvalid ||
      ReadBootModule(index, 0, buffer, -1) != kSyscallErrorInvalid ||
      ReadBootModule(index, 0, buffer, kSyscallBootModuleReadMaxSize + 1) !=
          kSyscallErrorInvalid ||
      ReadBootModule(index, 0, nullptr, 1) != kSyscallErrorFault ||
      ReadBootModule(index, 0, kernel_buffer, 1) != kSyscallErrorFault ||
      ReadBootModule(index, 0, ReadOnlyAs<void>(), 1) != kSyscallErrorFault ||
      ReadBootModule(index, size - 1, past_stack_top, 32) !=
          kSyscallErrorFault ||
      ReadBootModule(index, 0, nullptr, 0) != 0 ||
      ReadBootModule(index, size, buffer, 16) != 0 ||
      ReadBootModule(index, INT64_MAX, buffer, 16) != 0 ||
      ReadBootModule(index, size - 1, buffer, 16) != 1) {
    return 4;
  }

  // Reads of the largest size and of an odd one return the same bytes.
  constexpr int64_t kOddChunkSize = 1000;
  const int64_t checksum =
      ChecksumBootModule(index, size, kSyscallBootModuleReadMaxSize);
  if (checksum < 0 ||
      ChecksumBootModule(index, size, kOddChunkSize) != checksum) {
    return 5;
  }

  // The filesystem holds every directory and regular file of the archive.
  BootModuleSource source(index);
  tar::Reader reader(&source);
  int64_t files = 0;
  for (;;) {
    tar::Entry entry;
    const int64_t next = reader.Next(&entry);
    if (next == 0) {
      break;
    }
    if (next < 0) {
      return 6;
    }
    if (entry.type == tar::EntryType::kSkipped) {
      continue;
    }
    FsStat st{};
    if (Stat(entry.path, entry.path_len, &st) != 0) {
      return 7;
    }
    if (entry.type == tar::EntryType::kDirectory) {
      if (st.type != FsNodeType::kDirectory) {
        return 8;
      }
      continue;
    }
    if (st.type != FsNodeType::kRegular || st.size != entry.size) {
      return 9;
    }
    const int64_t fd = Open(entry.path, entry.path_len, kFsOpenRead);
    if (fd < 0) {
      return 10;
    }
    const int64_t status = CompareFileWithArchive(fd, reader, entry);
    if (Close(fd) != 0 && status == 0) {
      return 11;
    }
    if (status != 0) {
      return status;
    }
    ++files;
  }
  // The build's boot filesystem has files, including `/bin/hello`, which runs
  // via `SpawnFile()`.
  if (files <= 0) {
    return 14;
  }
  {
    const int64_t pid = SpawnFile("/bin/hello");
    if (pid < 1) {
      return 15;
    }
    const StatusOr<UserProgramResult> res = Wait(pid);
    if (!res.ok() || res->outcome != UserProgramOutcome::kExited ||
        res->exit_status != 0) {
      return 16;
    }
  }
  {
    const char* const argv[] = {"hello", "world", "test"};
    const int64_t pid = SpawnFile("/bin/hello", 3, argv);
    if (pid < 1) {
      return 15;
    }
    const StatusOr<UserProgramResult> res = Wait(pid);
    if (!res.ok() || res->outcome != UserProgramOutcome::kExited ||
        res->exit_status != 0) {
      return 16;
    }
  }
  return 0;
}

asm(R"(
    .pushsection .rodata
    .globl g_fs_test_prog_start
    .globl g_fs_test_prog_end
g_fs_test_prog_start:
    // rdi holds `arg`, rsi holds `argc`, rdx holds `argv`, and rsp is 8 mod 16
    // pointing at a zero return address:
    //   0xDEAD: execute ud2 (#UD)
    //   0xBEEF: spin in user mode until killed
    //   otherwise: verify initial stack + `(argc, argv)`, code on page 1,
    //   unaligned rodata, multi-page unaligned data + zeroed bss, user stack,
    //   and kUserCapNone, then Exit(arg + 100 + argv_tag).
    mov r12, rdi
    mov r13, rsi
    mov r14, rdx
    mov r15, rsp
    cmp r12, 0xDEAD
    jne 1f
    ud2
1:
    cmp r12, 0xBEEF
    jne 3f
2:
    pause
    jmp 2b
3:
    // Verify initial stack pointer alignment (8 mod 16), zero return address,
    // plus `argc >= 0`, `argv != nullptr`, and `argv[argc] == nullptr`.
    mov rax, r15
    and eax, 0xF
    cmp eax, 8
    jne 9f
    cmp qword ptr [r15], 0
    jne 9f
    test r13, r13
    js 9f
    test r14, r14
    jz 9f
    cmp qword ptr [r14 + r13 * 8], 0
    jne 9f

    // Compute `ExpectedArgvTag(argc, argv)` in rbx (0 when `argc == 0`).
    xor ebx, ebx
    xor ecx, ecx
4:
    cmp rcx, r13
    jge 6f
    mov rsi, [r14 + rcx * 8]
    test rsi, rsi
    jz 9f
    imul rbx, rbx, 131
    lea rbx, [rbx + rcx + 1]
5:
    movzx eax, byte ptr [rsi]
    test eax, eax
    jz 55f
    imul rbx, rbx, 31
    add rbx, rax
    inc rsi
    jmp 5b
55:
    inc rcx
    jmp 4b
6:
    and ebx, 0xFFFF

    // Call the `ret` instruction placed at the start of `.text` page 1
    // (0xFFFFFFFF80001000).
    mov rax, 0xFFFFFFFF80001000
    call rax
    cmp eax, 0x5A
    jne 9f

    // Verify unaligned `.rodata` at 0xFFFFFFFF80002080 and zeroed leading
    // sub-page bytes at 0xFFFFFFFF80002000.
    mov rax, 0xFFFFFFFF80002000
    cmp qword ptr [rax], 0
    jne 9f
    mov rcx, [rax + 0x80]
    mov rdx, 0xCAFEBABE12345678
    cmp rcx, rdx
    jne 9f

    // Verify unaligned `.data`+`.bss` starting at 0xFFFFFFFF800031A0:
    //   - leading sub-page bytes [0x...3000, 0x...31A0) are mapped R+W and 0
    //   - 42 at 0x...31A0 (page 0) and 0x1122334455667788 at 0x...41A0 (page 1)
    //   - zeroed `.bss` tail at 0x...41A8 and pure `.bss` page 2 (0x...5000)
    mov rax, 0xFFFFFFFF80003000
    cmp qword ptr [rax], 0
    jne 9f
    cmp qword ptr [rax + 0x198], 0
    jne 9f
    mov qword ptr [rax], 11
    cmp qword ptr [rax], 11
    jne 9f
    cmp qword ptr [rax + 0x1A0], 42
    jne 9f
    mov qword ptr [rax + 0x1A0], 99
    cmp qword ptr [rax + 0x1A0], 99
    jne 9f
    mov rcx, [rax + 0x11A0]
    mov rdx, 0x1122334455667788
    cmp rcx, rdx
    jne 9f
    cmp qword ptr [rax + 0x11A8], 0
    jne 9f
    cmp qword ptr [rax + 0x2000], 0
    jne 9f
    cmp qword ptr [rax + 0x2FF8], 0
    jne 9f
    mov qword ptr [rax + 0x2FF8], 77

    // Verify the user stack is writable.
    push r12
    pop rax
    cmp rax, r12
    jne 9f

    // Verify `kUserCapNone`: driver-only PciInfo (8) and SpawnDriver (7) fail
    // with EPERM (-1), even when the parent had driver capabilities.
    mov eax, 8
    xor edi, edi
    xor esi, esi
    int 0x80
    cmp rax, -1
    jne 9f
    mov eax, 7
    xor edi, edi
    xor esi, esi
    xor edx, edx
    int 0x80
    cmp rax, -1
    jne 9f

    // Exit(arg + 100 + argv_tag).
    xor eax, eax
    lea rdi, [r12 + rbx + 100]
    int 0x80
9:
    xor eax, eax
    mov edi, 999
    int 0x80
g_fs_test_prog_end:
    .popsection
)");

struct TestElf64Header {
  uint8_t ident[16] = {0x7F, 'E', 'L', 'F', 2, 1, 1};
  uint16_t type = 2;      // ET_EXEC
  uint16_t machine = 62;  // EM_X86_64
  uint32_t version = 1;
  uint64_t entry = 0xFFFFFFFF80000000ULL;
  uint64_t phoff = 64;
  uint64_t shoff = 0;
  uint32_t flags = 0;
  uint16_t ehsize = 64;
  uint16_t phentsize = 56;
  uint16_t phnum = 0;
  uint16_t shentsize = 0;
  uint16_t shnum = 0;
  uint16_t shstrndx = 0;
};
static_assert(sizeof(TestElf64Header) == 64);

struct TestElf64Phdr {
  uint32_t type = 1;  // PT_LOAD
  uint32_t flags = 0;
  uint64_t offset = 0;
  uint64_t vaddr = 0;
  uint64_t paddr = 0;
  uint64_t filesz = 0;
  uint64_t memsz = 0;
  uint64_t align = 0x1000;
};
static_assert(sizeof(TestElf64Phdr) == 56);

static bool WriteZerosToFd(const int64_t fd, const int64_t total_bytes) {
  for (int64_t i = 0; i < kSyscallFsMaxIoSize; ++i) {
    g_ipc_buf_b[i] = 0;
  }
  for (int64_t left = total_bytes; left > 0;) {
    const int64_t chunk =
        (left < kSyscallFsMaxIoSize) ? left : kSyscallFsMaxIoSize;
    if (FileWrite(fd, g_ipc_buf_b, chunk) != chunk) {
      return false;
    }
    left -= chunk;
  }
  return true;
}

static int64_t ExpectedArgvTag(const int argc, const char* const* const argv) {
  uint64_t acc = 0;
  for (int i = 0; i < argc; ++i) {
    acc = acc * 131 + static_cast<uint64_t>(i + 1);
    for (int64_t j = 0; argv[i][j] != '\0'; ++j) {
      acc = acc * 31 + static_cast<uint8_t>(argv[i][j]);
    }
  }
  return static_cast<int64_t>(acc & 0xFFFF);
}

// Writes the multi-page 3-segment test executable to `path`:
//   - `.text` at 0xFFFFFFFF80000000 (R+X, file offset 0x1000, size 0x1010):
//     page 0 holds `g_fs_test_prog_start..end`; page 1 (+0x1000) holds
//     `mov eax, 0x5A; ret`.
//   - `.rodata` at non-page-aligned 0xFFFFFFFF80002080 (R, file offset 0x2080,
//     size 8): 0xCAFEBABE12345678.
//   - `.data`+`.bss` at non-page-aligned 0xFFFFFFFF800031A0 (R+W, file offset
//     0x21A0, filesz 0x1008, memsz 0x2E60 -> [0x...3000, 0x...6000)): 42 at
//     +0, 0x1122334455667788 at +0x1000, with non-zero file bytes right before
//     offset 0x21A0 to verify leading sub-page bytes stay zeroed.
static bool WriteValidTestElf(const char* const path) {
  const int64_t fd = Open(path, kFsOpenWrite | kFsOpenCreate | kFsOpenTruncate);
  if (fd < 0) {
    return false;
  }
  const int64_t code_len = g_fs_test_prog_end - g_fs_test_prog_start;
  if (code_len <= 0 || code_len > 0x1000) {
    (void)Close(fd);
    return false;
  }
  if (!WriteZerosToFd(fd, 0x31A8)) {
    (void)Close(fd);
    return false;
  }
  TestElf64Header ehdr{};
  ehdr.phnum = 3;
  const TestElf64Phdr phdrs[3] = {
      {.type = 1,
       .flags = 5,
       .offset = 0x1000,
       .vaddr = 0xFFFFFFFF80000000ULL,
       .paddr = 0xFFFFFFFF80000000ULL,
       .filesz = 0x1010,
       .memsz = 0x1010},
      {.type = 1,
       .flags = 4,
       .offset = 0x2080,
       .vaddr = 0xFFFFFFFF80002080ULL,
       .paddr = 0xFFFFFFFF80002080ULL,
       .filesz = 8,
       .memsz = 8},
      {.type = 1,
       .flags = 6,
       .offset = 0x21A0,
       .vaddr = 0xFFFFFFFF800031A0ULL,
       .paddr = 0xFFFFFFFF800031A0ULL,
       .filesz = 0x1008,
       .memsz = 0x2E60},
  };
  // `mov eax, 0x5A; ret` at the start of `.text` page 1.
  const uint8_t page1_fn[6] = {0xB8, 0x5A, 0x00, 0x00, 0x00, 0xC3};
  const uint64_t rodata_val = 0xCAFEBABE12345678ULL;
  const uint64_t guard_noise = 0xAAAAAAAAAAAAAAAAULL;
  const uint64_t data0_val = 42;
  const uint64_t data1_val = 0x1122334455667788ULL;
  const bool ok = Seek(fd, 0, kFsSeekSet) == 0 &&
                  FileWrite(fd, &ehdr, sizeof(ehdr)) ==
                      static_cast<int64_t>(sizeof(ehdr)) &&
                  FileWrite(fd, phdrs, sizeof(phdrs)) ==
                      static_cast<int64_t>(sizeof(phdrs)) &&
                  Seek(fd, 0x1000, kFsSeekSet) == 0x1000 &&
                  FileWrite(fd, g_fs_test_prog_start, code_len) == code_len &&
                  Seek(fd, 0x2000, kFsSeekSet) == 0x2000 &&
                  FileWrite(fd, page1_fn, sizeof(page1_fn)) ==
                      static_cast<int64_t>(sizeof(page1_fn)) &&
                  Seek(fd, 0x2078, kFsSeekSet) == 0x2078 &&
                  FileWrite(fd, &guard_noise, sizeof(guard_noise)) ==
                      static_cast<int64_t>(sizeof(guard_noise)) &&
                  FileWrite(fd, &rodata_val, sizeof(rodata_val)) ==
                      static_cast<int64_t>(sizeof(rodata_val)) &&
                  Seek(fd, 0x2198, kFsSeekSet) == 0x2198 &&
                  FileWrite(fd, &guard_noise, sizeof(guard_noise)) ==
                      static_cast<int64_t>(sizeof(guard_noise)) &&
                  FileWrite(fd, &data0_val, sizeof(data0_val)) ==
                      static_cast<int64_t>(sizeof(data0_val)) &&
                  Seek(fd, 0x31A0, kFsSeekSet) == 0x31A0 &&
                  FileWrite(fd, &data1_val, sizeof(data1_val)) ==
                      static_cast<int64_t>(sizeof(data1_val));
  return Close(fd) == 0 && ok;
}

// Writes a compact single-segment ELF with `ehdr` and `phdrs[0..phnum - 1]` of
// `file_size` total bytes (`<= kSyscallFsMaxIoSize`) to `/bad_elf`.
static bool WriteCustomElf(const TestElf64Header& ehdr,       //
                           const TestElf64Phdr* const phdrs,  //
                           const int phnum,                   //
                           const int64_t file_size) {
  for (int64_t i = 0; i < file_size; ++i) {
    g_ipc_buf_a[i] = 0xC3;  // `ret`
  }
  TestElf64Header h = ehdr;
  h.phnum = static_cast<uint16_t>(phnum);
  __builtin_memcpy(g_ipc_buf_a, &h, sizeof(h));
  __builtin_memcpy(g_ipc_buf_a + sizeof(h), phdrs,
                   phnum * static_cast<int64_t>(sizeof(TestElf64Phdr)));
  const int64_t fd =
      Open("/bad_elf", kFsOpenWrite | kFsOpenCreate | kFsOpenTruncate);
  if (fd < 0) {
    return false;
  }
  const bool ok = FileWrite(fd, g_ipc_buf_a, file_size) == file_size;
  return Close(fd) == 0 && ok;
}

// `FsTestSubmode::kSpawnFileNoServer`.
static int64_t RunFsSpawnFileNoServer() {
  if (SpawnFile("/bin/hello", 0) != kSyscallErrorNoDevice) {
    return 1;
  }
  if (IpcListen(kIpcPortFs) != 0) {
    return 2;
  }
  if (SpawnFile("/bin/hello", 0) != kSyscallErrorDeadlock) {
    return 3;
  }
  return 0;
}

// `FsTestSubmode::kSpawnFileOrphanSpinner`.
static int64_t RunFsSpawnFileOrphanSpinner() {
  return SpawnFile("/prog", 0xBEEF) >= 1 ? 0 : 1;
}

// `FsTestSubmode::kOpenWaitOnly`.
static int64_t RunFsOpenWaitOnly() {
  (void)Open("/wait", kFsOpenRead);
  return 1;
}

// `FsTestSubmode::kSpawnFileWaitOnly`.
static int64_t RunFsSpawnFileWaitOnly() {
  (void)SpawnFile("/prog", 0);
  return 1;
}

// `FsTestSubmode::kCloseErrorOnly`.
static int64_t RunFsCloseErrorOnly() {
  const int64_t fd = Open("/c", kFsOpenRead);
  if (fd != 0) {
    return 1;
  }
  if (Close(fd) != kSyscallErrorIo) {
    return 2;
  }
  return 0;
}

// `FsTestSubmode::kSpawnFileSuite`.
static int64_t RunFsSpawnFileSuite() {
  if (!WriteValidTestElf("/prog") || MkDir("/dir") != 0) {
    return 1;
  }

  // 1. Path and pointer validation.
  const char with_nul[3] = {'/', '\0', 'a'};
  char long_path[kFsMaxPathLength + 2];
  for (int64_t i = 0; i < kFsMaxPathLength + 1; ++i) {
    long_path[i] = (i % 32 == 0) ? '/' : 'a';
  }
  char long_comp[kFsMaxNameLength + 3];
  long_comp[0] = '/';
  for (int64_t i = 1; i <= kFsMaxNameLength + 1; ++i) {
    long_comp[i] = 'b';
  }
  if (SpawnFile(nullptr, 4, 0) != kSyscallErrorFault ||
      SpawnFile(reinterpret_cast<const char*>(usertest::kKernelAddress), 4,
                0) != kSyscallErrorFault ||
      SpawnFile(reinterpret_cast<const char*>(kNonCanonicalAddress), 4, 0) !=
          kSyscallErrorFault ||
      SpawnFile(reinterpret_cast<const char*>(kUnmappedUserAddress), 4, 0) !=
          kSyscallErrorFault ||
      SpawnFile("/prog", -1, 0) != kSyscallErrorInvalid ||
      SpawnFile(with_nul, 3, 0) != kSyscallErrorInvalid ||
      SpawnFile("", 0, 0) != kSyscallErrorNoEntry ||
      SpawnFile("/no_such_prog", 0) != kSyscallErrorNoEntry ||
      SpawnFile("/prog/child", 0) != kSyscallErrorNotDir ||
      SpawnFile(long_path, kFsMaxPathLength + 1, 0) !=
          kSyscallErrorNameTooLong ||
      SpawnFile(long_comp, kFsMaxNameLength + 2, 0) !=
          kSyscallErrorNameTooLong ||
      SpawnFile("/", 0) != kSyscallErrorIsDir ||
      SpawnFile("/dir", 0) != kSyscallErrorIsDir) {
    return 2;
  }

  // 2. Non-ELF and rejected ELF formats/segments -> `kSyscallErrorExecFormat`.
  {
    const int64_t empty_fd =
        Open("/empty", kFsOpenWrite | kFsOpenCreate | kFsOpenTruncate);
    const int64_t text_fd =
        Open("/text", kFsOpenWrite | kFsOpenCreate | kFsOpenTruncate);
    if (empty_fd < 0 || text_fd < 0 || Close(empty_fd) != 0 ||
        FileWrite(text_fd, "not an elf\n", 11) != 11 || Close(text_fd) != 0) {
      return 3;
    }
    if (SpawnFile("/empty", 0) != kSyscallErrorExecFormat ||
        SpawnFile("/text", 0) != kSyscallErrorExecFormat) {
      return 4;
    }
    (void)Unlink("/empty");
    (void)Unlink("/text");
  }

  // Base valid single-segment header & phdr for negative format mutations.
  const TestElf64Phdr valid_code_phdr{
      .type = 1,
      .flags = 5,
      .offset = 0x200,
      .vaddr = 0xFFFFFFFF80000000ULL,
      .paddr = 0xFFFFFFFF80000000ULL,
      .filesz = 0x100,
      .memsz = 0x100,
  };
  // Shared object (ET_DYN).
  {
    TestElf64Header dyn_hdr{};
    dyn_hdr.type = 3;
    if (!WriteCustomElf(dyn_hdr, &valid_code_phdr, 1, 0x300) ||
        SpawnFile("/bad_elf", 0) != kSyscallErrorExecFormat) {
      return 5;
    }
  }
  // Dynamic executable with PT_INTERP.
  {
    const TestElf64Phdr phdrs[2] = {
        valid_code_phdr,
        {.type = 3, .flags = 4, .offset = 0x200, .filesz = 8, .memsz = 8},
    };
    if (!WriteCustomElf(TestElf64Header{}, phdrs, 2, 0x300) ||
        SpawnFile("/bad_elf", 0) != kSyscallErrorExecFormat) {
      return 6;
    }
  }
  // Writable + executable segment (PF_R | PF_W | PF_X = 7).
  {
    TestElf64Phdr wx = valid_code_phdr;
    wx.flags = 7;
    if (!WriteCustomElf(TestElf64Header{}, &wx, 1, 0x300) ||
        SpawnFile("/bad_elf", 0) != kSyscallErrorExecFormat) {
      return 7;
    }
  }
  // Segment overlapping the reserved user stack, the anonymous MemMap window,
  // and exceeding `kUserProgramMaxPages` (4096 pages).
  const uint64_t bad_vaddrs[2] = {
      usertest::kStackTop - 0x1000,
      0xFFFFB00000000000ULL,
  };
  for (const uint64_t bad_vaddr : bad_vaddrs) {
    TestElf64Phdr overlap = valid_code_phdr;
    overlap.vaddr = bad_vaddr;
    overlap.paddr = bad_vaddr;
    TestElf64Header hdr{};
    hdr.entry = bad_vaddr;
    if (!WriteCustomElf(hdr, &overlap, 1, 0x300) ||
        SpawnFile("/bad_elf", 0) != kSyscallErrorExecFormat) {
      return 8;
    }
  }
  {
    TestElf64Phdr huge = valid_code_phdr;
    huge.memsz = 4097ULL * 0x1000ULL;
    if (!WriteCustomElf(TestElf64Header{}, &huge, 1, 0x300) ||
        SpawnFile("/bad_elf", 0) != kSyscallErrorExecFormat) {
      return 9;
    }
  }
  // Segment whose `offset + filesz` runs past the end of the file, and two
  // non-page-aligned segments that share a 4 KiB page.
  {
    TestElf64Phdr cut = valid_code_phdr;
    cut.filesz = 0x200;
    cut.memsz = 0x200;
    const TestElf64Phdr same_page[2] = {
        valid_code_phdr,
        {.type = 1,
         .flags = 4,
         .offset = 0x200,
         .vaddr = 0xFFFFFFFF80000800ULL,
         .paddr = 0xFFFFFFFF80000800ULL,
         .filesz = 8,
         .memsz = 8},
    };
    if (!WriteCustomElf(TestElf64Header{}, &cut, 1, 0x300) ||
        SpawnFile("/bad_elf", 0) != kSyscallErrorExecFormat ||
        !WriteCustomElf(TestElf64Header{}, same_page, 2, 0x300) ||
        SpawnFile("/bad_elf", 0) != kSyscallErrorExecFormat) {
      return 10;
    }
  }
  (void)Unlink("/bad_elf");

  // 3. `args` / `argc` validation, valid execution, `(argc, argv)` passing
  //    (odd, even, empty-string, and max-limits), and `#UD` fault handling.
  {
    const char ok_str[] = "ok";
    const char nul_str[4] = {'a', 'b', '\0', 'c'};
    char big_a[512];
    char big_b[513];
    for (int i = 0; i < 512; ++i) {
      big_a[i] = 'a';
    }
    for (int i = 0; i < 513; ++i) {
      big_b[i] = 'b';
    }
    const SyscallArgSlice ok_slice{
        .ptr = reinterpret_cast<uintptr_t>(ok_str),
        .len = 2,
    };
    const SyscallArgSlice neg_len_slice{
        .ptr = reinterpret_cast<uintptr_t>(ok_str),
        .len = -1,
    };
    const SyscallArgSlice null_ptr_slice{.ptr = 0, .len = 2};
    const SyscallArgSlice kernel_ptr_slice{
        .ptr = usertest::kKernelAddress,
        .len = 2,
    };
    const SyscallArgSlice unmapped_ptr_slice{
        .ptr = kUnmappedUserAddress,
        .len = 2,
    };
    const SyscallArgSlice nul_byte_slice{
        .ptr = reinterpret_cast<uintptr_t>(nul_str),
        .len = 4,
    };
    const SyscallArgSlice too_long_slices[2] = {
        {.ptr = reinterpret_cast<uintptr_t>(big_a), .len = 512},
        {.ptr = reinterpret_cast<uintptr_t>(big_b), .len = 513},
    };
    if (SpawnFile("/prog", 5, &ok_slice, -1, 0) != kSyscallErrorInvalid ||
        SpawnFile("/prog", 5, &ok_slice, kSyscallSpawnMaxArgs + 1, 0) !=
            kSyscallErrorInvalid ||
        SpawnFile("/prog", 5, nullptr, 1, 0) != kSyscallErrorFault ||
        SpawnFile(
            "/prog", 5,
            reinterpret_cast<const SyscallArgSlice*>(usertest::kKernelAddress),
            1, 0) != kSyscallErrorFault ||
        SpawnFile(
            "/prog", 5,
            reinterpret_cast<const SyscallArgSlice*>(kUnmappedUserAddress), 1,
            0) != kSyscallErrorFault ||
        SpawnFile("/prog", 5, &neg_len_slice, 1, 0) != kSyscallErrorInvalid ||
        SpawnFile("/prog", 5, &null_ptr_slice, 1, 0) != kSyscallErrorFault ||
        SpawnFile("/prog", 5, &kernel_ptr_slice, 1, 0) != kSyscallErrorFault ||
        SpawnFile("/prog", 5, &unmapped_ptr_slice, 1, 0) !=
            kSyscallErrorFault ||
        SpawnFile("/prog", 5, &nul_byte_slice, 1, 0) != kSyscallErrorInvalid ||
        SpawnFile("/prog", 5, too_long_slices, 2, 0) != kSyscallErrorInvalid) {
      return 28;
    }
  }
  {
    // When `argc == 0`, `args` is ignored even if it is not a user address.
    const int64_t pid = SpawnFile(
        "/prog", 5,
        reinterpret_cast<const SyscallArgSlice*>(usertest::kKernelAddress), 0,
        23);
    if (pid < 1) {
      return 11;
    }
    const StatusOr<UserProgramResult> res = Wait(pid);
    if (!res.ok() || res->outcome != UserProgramOutcome::kExited ||
        res->exit_status != 123) {
      return 12;
    }
  }
  {
    // Odd `argc` (with an empty string), even `argc`, and
    // `kSyscallSpawnMaxArgs` totaling exactly `kSyscallSpawnMaxArgBytes` bytes.
    const char* const odd_argv[3] = {"/prog", "", "alpha"};
    const int64_t odd_pid = SpawnFile("/prog", 3, odd_argv);
    const StatusOr<UserProgramResult> odd_res =
        (odd_pid >= 1) ? Wait(odd_pid) : StatusOr<UserProgramResult>(-1);
    if (!odd_res.ok() || odd_res->outcome != UserProgramOutcome::kExited ||
        odd_res->exit_status != 100 + ExpectedArgvTag(3, odd_argv)) {
      return 29;
    }

    const char* const even_argv[4] = {"/prog", "one", "two", "three"};
    const int64_t even_pid = SpawnFile("/prog", 4, even_argv);
    const StatusOr<UserProgramResult> even_res =
        (even_pid >= 1) ? Wait(even_pid) : StatusOr<UserProgramResult>(-1);
    if (!even_res.ok() || even_res->outcome != UserProgramOutcome::kExited ||
        even_res->exit_status != 100 + ExpectedArgvTag(4, even_argv)) {
      return 29;
    }

    constexpr int kChunkLen = kSyscallSpawnMaxArgBytes / kSyscallSpawnMaxArgs;
    static_assert(kChunkLen * kSyscallSpawnMaxArgs == kSyscallSpawnMaxArgBytes);
    char max_storage[kSyscallSpawnMaxArgs][kChunkLen + 1];
    const char* max_argv[kSyscallSpawnMaxArgs];
    SyscallArgSlice max_slices[kSyscallSpawnMaxArgs];
    for (int i = 0; i < kSyscallSpawnMaxArgs; ++i) {
      for (int j = 0; j < kChunkLen; ++j) {
        max_storage[i][j] = static_cast<char>('a' + ((i + j) % 26));
      }
      max_storage[i][kChunkLen] = '\0';
      max_argv[i] = max_storage[i];
      max_slices[i] = SyscallArgSlice{
          .ptr = reinterpret_cast<uintptr_t>(max_storage[i]),
          .len = kChunkLen,
      };
    }
    const int64_t max_pid =
        SpawnFile("/prog", 5, max_slices, kSyscallSpawnMaxArgs, 7);
    const StatusOr<UserProgramResult> max_res =
        (max_pid >= 1) ? Wait(max_pid) : StatusOr<UserProgramResult>(-1);
    if (!max_res.ok() || max_res->outcome != UserProgramOutcome::kExited ||
        max_res->exit_status !=
            107 + ExpectedArgvTag(kSyscallSpawnMaxArgs, max_argv)) {
      return 30;
    }
  }
  {
    const int64_t pid = SpawnFile("/prog", 0xDEAD);
    if (pid < 1) {
      return 13;
    }
    const StatusOr<UserProgramResult> res = Wait(pid);
    if (!res.ok() || res->outcome != UserProgramOutcome::kFaulted ||
        res->fault_vector != 6) {
      return 14;
    }
  }

  // 4. `SpawnFile()` does not consume any of the caller's `kMaxOpenFiles`
  //    descriptors: it succeeds even when all 32 are open.
  {
    int fds[kMaxOpenFiles];
    for (int i = 0; i < kMaxOpenFiles; ++i) {
      fds[i] = static_cast<int>(Open("/prog", kFsOpenRead));
      if (fds[i] < 0) {
        return 15;
      }
    }
    if (Open("/prog", kFsOpenRead) != kSyscallErrorTooManyFiles) {
      return 16;
    }
    const int64_t pid = SpawnFile("/prog", 9);
    for (int i = 0; i < kMaxOpenFiles; ++i) {
      if (Close(fds[i]) != 0) {
        return 17;
      }
    }
    if (pid < 1) {
      return 18;
    }
    const StatusOr<UserProgramResult> res = Wait(pid);
    if (!res.ok() || res->outcome != UserProgramOutcome::kExited ||
        res->exit_status != 109) {
      return 19;
    }
  }

  // 5. All `kMaxUserChildProcesses` slots full -> `kSyscallErrorBusy`.
  {
    int64_t pids[kMaxUserChildProcesses];
    for (int i = 0; i < kMaxUserChildProcesses; ++i) {
      pids[i] = SpawnFile("/prog", static_cast<uint64_t>(i));
      if (pids[i] < 1) {
        return 20;
      }
    }
    if (SpawnFile("/prog", 99) != kSyscallErrorBusy) {
      return 21;
    }
    for (int i = 0; i < kMaxUserChildProcesses; ++i) {
      const StatusOr<UserProgramResult> res = Wait(pids[i]);
      if (!res.ok() || res->outcome != UserProgramOutcome::kExited ||
          res->exit_status != 100 + i) {
        return 22;
      }
    }
  }

  // 6. Orphaned spinning `SpawnFile()` grandchild is killed and reaped when
  //    its parent exits.
  {
    const int64_t helper_pid =
        Spawn("usertest",
              usertest::FsChecksArg(FsTestSubmode::kSpawnFileOrphanSpinner));
    if (helper_pid < 1) {
      return 23;
    }
    const StatusOr<UserProgramResult> res = Wait(helper_pid);
    if (!res.ok() || res->outcome != UserProgramOutcome::kExited ||
        res->exit_status != 0) {
      return 24;
    }
  }

  // 7. Renaming and unlinking after `SpawnFile()` returns leaves the already
  //    loaded child process unaffected.
  if (Rename("/prog", "/dir/moved_prog") != 0) {
    return 25;
  }
  {
    const int64_t pid = SpawnFile("/dir/moved_prog", 50);
    if (pid < 1 || Unlink("/dir/moved_prog") != 0 || Unlink("/dir") != 0) {
      return 26;
    }
    const StatusOr<UserProgramResult> res = Wait(pid);
    if (!res.ok() || res->outcome != UserProgramOutcome::kExited ||
        res->exit_status != 150) {
      return 27;
    }
  }
  return 0;
}

// Runs `Mode::kFsChecks` as `arg` says.
static int64_t RunFsChecks(const uint64_t arg) {
  switch (static_cast<FsTestSubmode>(usertest::DriverSubmode(arg))) {
    case FsTestSubmode::kFullSuite:
      return RunFsFullSuite();
    case FsTestSubmode::kLeakAndFault:
      return RunFsLeakAndFault();
    case FsTestSubmode::kSeedDocs:
      return RunFsSeedDocs();
    case FsTestSubmode::kStaleFd:
      return RunFsStaleFd();
    case FsTestSubmode::kFailedOpenOnly:
      return RunFsFailedOpenOnly();
    case FsTestSubmode::kMalformedServer:
      return RunFsMalformedServer();
    case FsTestSubmode::kVerifyRootfs:
      return RunFsVerifyRootfs();
    case FsTestSubmode::kInterleavedServers:
      return RunFsInterleavedServers();
    case FsTestSubmode::kSpawnFileSuite:
      return RunFsSpawnFileSuite();
    case FsTestSubmode::kSpawnFileNoServer:
      return RunFsSpawnFileNoServer();
    case FsTestSubmode::kSpawnFileOrphanSpinner:
      return RunFsSpawnFileOrphanSpinner();
    case FsTestSubmode::kOpenWaitOnly:
      return RunFsOpenWaitOnly();
    case FsTestSubmode::kSpawnFileWaitOnly:
      return RunFsSpawnFileWaitOnly();
    case FsTestSubmode::kCloseErrorOnly:
      return RunFsCloseErrorOnly();
  }
  return usertest::kBadSubmodeStatus;
}

static void FillBlockPattern(uint8_t* const dst,  //
                             const int64_t len,   //
                             const uint64_t seed) {
  for (int64_t i = 0; i < len; ++i) {
    dst[i] = static_cast<uint8_t>(
        (seed * 131ULL + static_cast<uint64_t>(i) * 17ULL + 0x5AULL) & 0xFFULL);
  }
}

static bool VerifyBlockPattern(const uint8_t* const buf,  //
                               const int64_t len,         //
                               const uint64_t seed) {
  for (int64_t i = 0; i < len; ++i) {
    const uint8_t expected = static_cast<uint8_t>(
        (seed * 131ULL + static_cast<uint64_t>(i) * 17ULL + 0x5AULL) & 0xFFULL);
    if (buf[i] != expected) {
      return false;
    }
  }
  return true;
}

static bool BufferIsAllZero(const uint8_t* const buf, const int64_t len) {
  for (int64_t i = 0; i < len; ++i) {
    if (buf[i] != 0) {
      return false;
    }
  }
  return true;
}

static bool RunDiskCli(const int argc,                 //
                       const char* const* const argv,  //
                       const int64_t expected_status) {
  const int64_t pid = SpawnFile("/bin/disk", argc, argv);
  if (pid < 1) {
    return false;
  }
  const StatusOr<UserProgramResult> res = Wait(pid);
  return res.ok() && res->outcome == UserProgramOutcome::kExited &&
         res->exit_status == expected_status;
}

alignas(8) uint8_t g_block_scratch[kBlockWriteScratchBytes];

static void FormatU64Dec(const uint64_t val, char (&out)[21]) {
  char digits[20] = {};
  int n = 0;
  uint64_t v = val;
  do {
    digits[n++] = static_cast<char>('0' + (v % 10));
    v /= 10;
  } while (v != 0);
  int pos = 0;
  while (n > 0) {
    out[pos++] = digits[--n];
  }
  out[pos] = '\0';
}

static int64_t RunBlockNoServer(const uint32_t disk_index) {
  if (GetBlockDeviceInfo(disk_index).status() != kSyscallErrorNoDevice) {
    return 1;
  }
  if (ReadBlocksChecked(disk_index, 0, 1, 512, g_ipc_buf_a, 512) !=
      kSyscallErrorNoDevice) {
    return 2;
  }
  if (WriteBlocksWithScratch(disk_index,       //
                             0,                //
                             1,                //
                             g_ipc_buf_a,      //
                             512,              //
                             g_block_scratch,  //
                             sizeof(g_block_scratch)) !=
      kSyscallErrorNoDevice) {
    return 3;
  }
  if (FlushDisk(disk_index) != kSyscallErrorNoDevice) {
    return 4;
  }
  return 0;
}

static int64_t RunBlockWriteReadVerify(const uint32_t disk_index) {
  static_assert(sizeof(g_ipc_buf_a) >= kBlockMaxIoBytes);
  static_assert(sizeof(g_ipc_buf_b) >= kBlockMaxIoBytes);
  static_assert(sizeof(g_block_scratch) >= kBlockWriteScratchBytes);

  if (IpcListen(kIpcPortBlock) != kSyscallErrorPermission) {
    return 1;
  }

  // Raw malformed IPC validation against live server.
  {
    const uint32_t bad_op = 99;
    const BlockGetInfoRequest truncated_info{};
    BlockReadRequest bad_reserved{};
    bad_reserved.op = BlockRpcOp::kRead;
    bad_reserved.disk_index = disk_index;
    bad_reserved.reserved = 1;
    if (IpcCall(kIpcPortBlock, nullptr, 0, nullptr, 0) !=
            kSyscallErrorInvalid ||
        IpcCall(kIpcPortBlock, &bad_op, sizeof(bad_op), nullptr, 0) !=
            kSyscallErrorInvalid ||
        IpcCall(kIpcPortBlock,    //
                &truncated_info,  //
                4,                //
                g_ipc_buf_a,      //
                sizeof(BlockDeviceInfo)) != kSyscallErrorInvalid ||
        IpcCall(kIpcPortBlock,         //
                &bad_reserved,         //
                sizeof(bad_reserved),  //
                g_ipc_buf_a,           //
                kBlockMaxIoBytes) != kSyscallErrorInvalid) {
      return 2;
    }
  }

  int64_t disk_count = 0;
  const StatusOr<BlockDeviceInfo> info =
      GetBlockDeviceInfo(disk_index, &disk_count);
  if (!info.ok() || disk_count < 1 ||
      disk_index >= static_cast<uint32_t>(disk_count)) {
    return 3;
  }
  const int32_t bs = info->block_size;
  if (!IsValidBlockDeviceInfo(*info) || info->block_count < 16 ||
      info->nsid < 1 ||
      info->block_count <=
          static_cast<uint64_t>(4 + info->max_transfer_blocks)) {
    return 4;
  }
  if (GetBlockDeviceInfo(static_cast<uint32_t>(disk_count)).status() !=
          kSyscallErrorNoDevice ||
      GetBlockDeviceInfo(UINT32_MAX).status() != kSyscallErrorNoDevice) {
    return 5;
  }
  const int32_t over_blocks = info->max_transfer_blocks + 1;
  const BlockReadRequest over_read{
      .op = BlockRpcOp::kRead,
      .disk_index = disk_index,
      .lba = 0,
      .block_count = over_blocks,
      .reserved = 0,
  };
  const BlockWriteRequest over_write{
      .op = BlockRpcOp::kWrite,
      .disk_index = disk_index,
      .lba = 0,
      .block_count = over_blocks,
      .reserved = 0,
  };
  constexpr int64_t kScratchCap = static_cast<int64_t>(sizeof(g_block_scratch));
  if (ReadBlocksChecked(disk_index,         //
                        info->block_count,  //
                        1,                  //
                        bs,                 //
                        g_ipc_buf_a,        //
                        bs) != kSyscallErrorInvalid ||
      ReadBlocksChecked(disk_index, UINT64_MAX, 1, bs, g_ipc_buf_a, bs) !=
          kSyscallErrorInvalid ||
      ReadBlocks(disk_index, 0, 1, g_ipc_buf_a, bs - 1) !=
          kSyscallErrorInvalid ||
      ReadBlocksChecked(disk_index, 0, 1, bs, g_ipc_buf_a, bs - 1) !=
          kSyscallErrorInvalid ||
      ReadBlocksChecked(disk_index, 0, 1, 256, g_ipc_buf_a, 512) !=
          kSyscallErrorInvalid ||
      ReadBlocksChecked(disk_index, 0, -1, bs, g_ipc_buf_a, bs) !=
          kSyscallErrorInvalid ||
      ReadBlocksChecked(disk_index,   //
                        0,            //
                        over_blocks,  //
                        bs,           //
                        g_ipc_buf_a,  //
                        kBlockMaxIoBytes) != kSyscallErrorInvalid ||
      IpcCall(kIpcPortBlock,      //
              &over_read,         //
              sizeof(over_read),  //
              g_ipc_buf_a,        //
              kBlockMaxIoBytes) != kSyscallErrorInvalid ||
      IpcCall(kIpcPortBlock, &over_write, sizeof(over_write), nullptr, 0) !=
          kSyscallErrorInvalid ||
      (info->max_transfer_blocks >= 2 &&
       (ReadBlocksChecked(disk_index,             //
                          info->block_count - 1,  //
                          2,                      //
                          bs,                     //
                          g_ipc_buf_a,            //
                          2 * int64_t{bs}) != kSyscallErrorInvalid ||
        ReadBlocksChecked(disk_index,      //
                          UINT64_MAX - 1,  //
                          2,               //
                          bs,              //
                          g_ipc_buf_a,     //
                          2 * int64_t{bs}) != kSyscallErrorInvalid)) ||
      WriteBlocksWithScratch(disk_index,       //
                             0,                //
                             1,                //
                             g_ipc_buf_a,      //
                             bs - 1,           //
                             g_block_scratch,  //
                             kScratchCap) != kSyscallErrorInvalid ||
      WriteBlocksWithScratch(disk_index,         //
                             info->block_count,  //
                             1,                  //
                             g_ipc_buf_a,        //
                             bs,                 //
                             g_block_scratch,    //
                             kScratchCap) != kSyscallErrorInvalid) {
    return 6;
  }
  const uint32_t bad_disk = static_cast<uint32_t>(disk_count);
  if (ReadBlocksChecked(bad_disk, 0, 1, bs, g_ipc_buf_a, bs) !=
          kSyscallErrorNoDevice ||
      WriteBlocksWithScratch(bad_disk,         //
                             0,                //
                             1,                //
                             g_ipc_buf_a,      //
                             bs,               //
                             g_block_scratch,  //
                             kScratchCap) != kSyscallErrorNoDevice ||
      FlushDisk(bad_disk) != kSyscallErrorNoDevice) {
    return 7;
  }

  // Non-destructive read of LBA 0 and zero-block I/O always succeed.
  if (ReadBlocksChecked(disk_index, 0, 1, bs, g_ipc_buf_a, bs) != 1 ||
      ReadBlocksChecked(disk_index, 0, 0, bs, nullptr, 0) != 0) {
    return 8;
  }
  if (!IsBlockTestDiskSerial(info->serial)) {
    return 0;
  }
  if (info->read_only != 0) {
    const int32_t multi = info->max_transfer_blocks;
    const int64_t multi_bytes = static_cast<int64_t>(multi) * bs;
    const uint64_t last_lba = info->block_count - 1;
    if (WriteBlocksWithScratch(disk_index,       //
                               0,                //
                               1,                //
                               g_ipc_buf_a,      //
                               bs,               //
                               g_block_scratch,  //
                               kScratchCap) != kSyscallErrorPermission ||
        WriteBlocks(disk_index, 0, 1, g_ipc_buf_a, bs) !=
            kSyscallErrorPermission ||
        WriteBlocksWithScratch(disk_index,       //
                               0,                //
                               0,                //
                               nullptr,          //
                               0,                //
                               g_block_scratch,  //
                               kScratchCap) != 0 ||
        FlushDisk(disk_index) > 0 ||
        ReadBlocks(disk_index, 0, 0, nullptr, 0) != 0 ||
        ReadBlocks(disk_index, 0, 1, g_ipc_buf_b, bs) != 1 ||
        ReadBlocksChecked(disk_index, 4, multi, bs, g_ipc_buf_b, multi_bytes) !=
            multi ||
        ReadBlocksChecked(disk_index, last_lba, 1, bs, g_ipc_buf_b, bs) != 1) {
      return 15;
    }
    return 0;
  }

  // Single block LBA 0 (exercising zero-copy staging at g_block_scratch +
  // sizeof(BlockWriteRequest)).
  uint8_t* const zero_copy_buf = g_block_scratch + sizeof(BlockWriteRequest);
  FillBlockPattern(zero_copy_buf, bs, 0xA100);
  if (WriteBlocksWithScratch(disk_index,       //
                             0,                //
                             1,                //
                             zero_copy_buf,    //
                             bs,               //
                             g_block_scratch,  //
                             kScratchCap) != 1 ||
      FlushDisk(disk_index) != 0) {
    return 9;
  }
  if (ReadBlocksChecked(disk_index, 0, 1, bs, g_ipc_buf_b, bs) != 1 ||
      !VerifyBlockPattern(g_ipc_buf_b, bs, 0xA100) ||
      ReadBlocks(disk_index, 0, 0, nullptr, 0) != 0 ||
      ReadBlocks(disk_index, 0, 1, g_ipc_buf_b, bs) != 1 ||
      !VerifyBlockPattern(g_ipc_buf_b, bs, 0xA100)) {
    return 10;
  }

  // Exercise 0-block write no-ops and the 5-arg stack-staging `WriteBlocks`
  // wrapper in ring 3 under `-fstack-clash-protection`, restoring pattern
  // `0xA100` on LBA 0 for `RunBlockVerifyPersisted`.
  if (WriteBlocksWithScratch(disk_index,       //
                             0,                //
                             0,                //
                             nullptr,          //
                             0,                //
                             g_block_scratch,  //
                             kScratchCap) != 0 ||
      WriteBlocks(disk_index, 0, 0, nullptr, 0) != 0) {
    return 16;
  }
  FillBlockPattern(g_ipc_buf_a, bs, 0xD400);
  if (WriteBlocks(disk_index, 0, 1, g_ipc_buf_a, bs) != 1 ||
      FlushDisk(disk_index) != 0 ||
      ReadBlocks(disk_index, 0, 1, g_ipc_buf_b, bs) != 1 ||
      !VerifyBlockPattern(g_ipc_buf_b, bs, 0xD400)) {
    return 17;
  }
  FillBlockPattern(g_ipc_buf_a, bs, 0xA100);
  if (WriteBlocks(disk_index, 0, 1, g_ipc_buf_a, bs) != 1 ||
      FlushDisk(disk_index) != 0 ||
      ReadBlocksChecked(disk_index, 0, 1, bs, g_ipc_buf_b, bs) != 1 ||
      !VerifyBlockPattern(g_ipc_buf_b, bs, 0xA100)) {
    return 18;
  }

  // Multi-block range LBA 4..4+multi-1 across full max_transfer_blocks.
  const int32_t multi = info->max_transfer_blocks;
  const int64_t multi_bytes = static_cast<int64_t>(multi) * bs;
  FillBlockPattern(g_ipc_buf_a, multi_bytes, 0xB204);
  if (WriteBlocksWithScratch(disk_index,       //
                             4,                //
                             multi,            //
                             g_ipc_buf_a,      //
                             multi_bytes,      //
                             g_block_scratch,  //
                             kScratchCap) != multi ||
      FlushDisk(disk_index) != 0) {
    return 11;
  }
  if (ReadBlocksChecked(disk_index, 4, multi, bs, g_ipc_buf_b, multi_bytes) !=
          multi ||
      !VerifyBlockPattern(g_ipc_buf_b, multi_bytes, 0xB204)) {
    return 12;
  }

  // Last block on the namespace.
  const uint64_t last_lba = info->block_count - 1;
  FillBlockPattern(g_ipc_buf_a, bs, 0xC3FF);
  if (WriteBlocksWithScratch(disk_index,       //
                             last_lba,         //
                             1,                //
                             g_ipc_buf_a,      //
                             bs,               //
                             g_block_scratch,  //
                             kScratchCap) != 1 ||
      FlushDisk(disk_index) != 0) {
    return 13;
  }
  if (ReadBlocksChecked(disk_index, last_lba, 1, bs, g_ipc_buf_b, bs) != 1 ||
      !VerifyBlockPattern(g_ipc_buf_b, bs, 0xC3FF)) {
    return 14;
  }
  return 0;
}

static int64_t RunBlockVerifyPersisted(const uint32_t disk_index) {
  const StatusOr<BlockDeviceInfo> info = GetBlockDeviceInfo(disk_index);
  if (!info.ok() || !IsValidBlockDeviceInfo(*info) || info->block_count < 16 ||
      info->block_count <=
          static_cast<uint64_t>(4 + info->max_transfer_blocks)) {
    return 1;
  }
  if (!IsBlockTestDiskSerial(info->serial) || info->read_only != 0) {
    return 0;
  }
  const int32_t bs = info->block_size;
  const int32_t multi = info->max_transfer_blocks;
  const int64_t multi_bytes = static_cast<int64_t>(multi) * bs;
  const uint64_t last_lba = info->block_count - 1;

  if (ReadBlocksChecked(disk_index, 0, 1, bs, g_ipc_buf_b, bs) != 1 ||
      !VerifyBlockPattern(g_ipc_buf_b, bs, 0xA100)) {
    return 2;
  }
  if (ReadBlocksChecked(disk_index, 4, multi, bs, g_ipc_buf_b, multi_bytes) !=
          multi ||
      !VerifyBlockPattern(g_ipc_buf_b, multi_bytes, 0xB204)) {
    return 3;
  }
  if (ReadBlocksChecked(disk_index, last_lba, 1, bs, g_ipc_buf_b, bs) != 1 ||
      !VerifyBlockPattern(g_ipc_buf_b, bs, 0xC3FF)) {
    return 4;
  }

  for (int64_t i = 0; i < multi_bytes; ++i) {
    g_ipc_buf_a[i] = 0;
  }
  constexpr int64_t kScratchCap = static_cast<int64_t>(sizeof(g_block_scratch));
  if (WriteBlocksWithScratch(disk_index,       //
                             0,                //
                             1,                //
                             g_ipc_buf_a,      //
                             bs,               //
                             g_block_scratch,  //
                             kScratchCap) != 1 ||
      WriteBlocksWithScratch(disk_index,       //
                             4,                //
                             multi,            //
                             g_ipc_buf_a,      //
                             multi_bytes,      //
                             g_block_scratch,  //
                             kScratchCap) != multi ||
      WriteBlocksWithScratch(disk_index,       //
                             last_lba,         //
                             1,                //
                             g_ipc_buf_a,      //
                             bs,               //
                             g_block_scratch,  //
                             kScratchCap) != 1) {
    return 5;
  }
  if (FlushDisk(disk_index) != 0) {
    return 6;
  }
  if (ReadBlocksChecked(disk_index, 0, 1, bs, g_ipc_buf_b, bs) != 1 ||
      !BufferIsAllZero(g_ipc_buf_b, bs) ||
      ReadBlocksChecked(disk_index, 4, multi, bs, g_ipc_buf_b, multi_bytes) !=
          multi ||
      !BufferIsAllZero(g_ipc_buf_b, multi_bytes)) {
    return 7;
  }
  if (ReadBlocksChecked(disk_index, last_lba, 1, bs, g_ipc_buf_b, bs) != 1 ||
      !BufferIsAllZero(g_ipc_buf_b, bs)) {
    return 8;
  }
  return 0;
}

static int64_t RunBlockCliDiskChecks(const uint32_t disk_index) {
  const StatusOr<BlockDeviceInfo> info = GetBlockDeviceInfo(disk_index);
  if (!info.ok() || !IsValidBlockDeviceInfo(*info)) {
    return 3;
  }
  char disk_str[21] = {};
  char oob_lba[21] = {};
  FormatU64Dec(disk_index, disk_str);
  FormatU64Dec(info->block_count, oob_lba);

  if (disk_index == 0) {
    const char* const argv_default[] = {"/bin/disk"};
    const char* const argv_info[] = {"/bin/disk", "info"};
    const char* const argv_d_info[] = {"/bin/disk", "-d", "0", "info"};
    if (!RunDiskCli(1, argv_default, 0) || !RunDiskCli(2, argv_info, 0) ||
        !RunDiskCli(4, argv_d_info, 0)) {
      return 1;
    }
    const char* const bad1[] = {"/bin/disk", "badcmd"};
    const char* const bad2[] = {"/bin/disk", "read"};
    const char* const bad3[] = {"/bin/disk", "read", "abc"};
    const char* const bad4[] = {"/bin/disk", "read", "0", "99"};
    const char* const bad5[] = {"/bin/disk", "write", "2"};
    const char* const bad6[] = {"/bin/disk", "read", oob_lba};
    const char* const bad7[] = {"/bin/disk", "read", "18446744073709551615"};
    const char* const bad8[] = {"/bin/disk", "info", "extra"};
    const char* const bad9[] = {"/bin/disk", "read", "0", "0"};
    const char* const bad10[] = {"/bin/disk", "-d"};
    const char* const bad11[] = {"/bin/disk", "-d", "4294967296"};
    if (!RunDiskCli(2, bad1, 1) || !RunDiskCli(2, bad2, 1) ||
        !RunDiskCli(3, bad3, 1) || !RunDiskCli(4, bad4, 1) ||
        !RunDiskCli(3, bad5, 1) || !RunDiskCli(3, bad6, 1) ||
        !RunDiskCli(3, bad7, 1) || !RunDiskCli(3, bad8, 1) ||
        !RunDiskCli(4, bad9, 1) || !RunDiskCli(2, bad10, 1) ||
        !RunDiskCli(3, bad11, 1)) {
      return 2;
    }
  } else {
    const char* const argv_d_info[] = {"/bin/disk", "-d", disk_str, "info"};
    const char* const bad6[] = {"/bin/disk", "-d", disk_str, "read", oob_lba};
    const char* const bad7[] = {"/bin/disk", "-d", disk_str, "read",
                                "18446744073709551615"};
    if (!RunDiskCli(4, argv_d_info, 0)) {
      return 1;
    }
    if (!RunDiskCli(5, bad6, 1) || !RunDiskCli(5, bad7, 1)) {
      return 2;
    }
  }

  if (!IsBlockTestDiskSerial(info->serial)) {
    return 0;
  }
  if (info->read_only != 0) {
    const char* const write_cmd[] = {"/bin/disk", "-d",    disk_str, "write",
                                     "2",         "hello", "from",   "nvme"};
    const char* const zero_cmd[] = {"/bin/disk", "-d", disk_str, "zero", "2"};
    const char* const read_cmd[] = {"/bin/disk", "-d", disk_str, "read", "2"};
    if (!RunDiskCli(8, write_cmd, 1) || !RunDiskCli(5, zero_cmd, 1) ||
        !RunDiskCli(5, read_cmd, 0)) {
      return 4;
    }
    return 0;
  }
  const int32_t bs = info->block_size;
  const char* const write_cmd[] = {"/bin/disk", "-d",    disk_str, "write",
                                   "2",         "hello", "from",   "nvme"};
  if (!RunDiskCli(8, write_cmd, 0)) {
    return 4;
  }
  if (ReadBlocksChecked(disk_index, 2, 1, bs, g_ipc_buf_a, bs) != 1) {
    return 5;
  }
  constexpr char kExpectedLine[] = "hello from nvme\n";
  constexpr int64_t kExpectedLen =
      static_cast<int64_t>(sizeof(kExpectedLine)) - 1;
  for (int64_t i = 0; i < kExpectedLen; ++i) {
    if (g_ipc_buf_a[i] != static_cast<uint8_t>(kExpectedLine[i])) {
      return 5;
    }
  }
  if (!BufferIsAllZero(g_ipc_buf_a + kExpectedLen, bs - kExpectedLen)) {
    return 5;
  }
  const char* const read_cmd[] = {"/bin/disk", "-d", disk_str, "read", "2"};
  if (!RunDiskCli(5, read_cmd, 0)) {
    return 6;
  }
  const char* const zero_cmd[] = {"/bin/disk", "-d", disk_str, "zero", "2"};
  if (!RunDiskCli(5, zero_cmd, 0)) {
    return 7;
  }
  if (ReadBlocksChecked(disk_index, 2, 1, bs, g_ipc_buf_a, bs) != 1 ||
      !BufferIsAllZero(g_ipc_buf_a, bs)) {
    return 8;
  }
  return 0;
}

static int64_t RunBlockMockMalformedReplies() {
  constexpr int64_t kScratchCap = static_cast<int64_t>(sizeof(g_block_scratch));
  // 10 bad GetBlockDeviceInfo replies -> all must return kSyscallErrorIo.
  for (int i = 0; i < 10; ++i) {
    if (GetBlockDeviceInfo(0).status() != kSyscallErrorIo) {
      return 1 + i;
    }
  }
  // ReadBlocks: rc != block_count and unaligned reply_size -> kSyscallErrorIo.
  if (ReadBlocks(0, 0, 1, g_ipc_buf_a, 512) != kSyscallErrorIo ||
      ReadBlocks(0, 0, 1, g_ipc_buf_a, 1024) != kSyscallErrorIo) {
    return 11;
  }
  // ReadBlocksChecked: reply_size (1024) != expected_bytes (512).
  if (ReadBlocksChecked(0, 0, 1, 512, g_ipc_buf_a, 1024) != kSyscallErrorIo) {
    return 12;
  }
  // WriteBlocksWithScratch: rc != block_count -> kSyscallErrorIo.
  if (WriteBlocksWithScratch(0,                //
                             0,                //
                             1,                //
                             g_ipc_buf_a,      //
                             512,              //
                             g_block_scratch,  //
                             kScratchCap) != kSyscallErrorIo) {
    return 13;
  }
  // FlushDisk: rc != 0 -> kSyscallErrorIo.
  if (FlushDisk(0) != kSyscallErrorIo) {
    return 14;
  }
  // Client-side invalid arguments.
  constexpr int64_t kTooSmallCap =
      static_cast<int64_t>(sizeof(BlockWriteRequest)) + 511;
  if (WriteBlocksWithScratch(0, 0, 1, g_ipc_buf_a, 512, nullptr, 0) !=
          kSyscallErrorInvalid ||
      WriteBlocksWithScratch(0,                //
                             0,                //
                             1,                //
                             g_ipc_buf_a,      //
                             512,              //
                             g_block_scratch,  //
                             kTooSmallCap) != kSyscallErrorInvalid ||
      ReadBlocksChecked(0, 0, 1, 256, g_ipc_buf_a, 512) !=
          kSyscallErrorInvalid ||
      ReadBlocksChecked(0, 0, 1, 512, g_ipc_buf_a, 511) !=
          kSyscallErrorInvalid) {
    return 15;
  }
  return 0;
}

static int64_t RunBlockMockClientGetInfoOnly() {
  return GetBlockDeviceInfo(0).ok() ? 0 : 1;
}

static int64_t RunBlockMockClientCallWait() {
  const StatusOr<BlockDeviceInfo> info = GetBlockDeviceInfo(0);
  return info.ok() ? 0 : -info.status();
}

static int64_t RunBlockMockServerHoldCall() {
  if (IpcListen(kIpcPortBlock) != 0 || IpcListen(kIpcFirstUserPort) != 0) {
    return 1;
  }
  IpcReceivedMessage msg{};
  if (!ReceiveCall(kIpcPortBlock, &msg)) {
    return 2;
  }
  // Hold `msg` unanswered and park in syscalls == 4 until killed.
  (void)IpcRecv(kIpcFirstUserPort, g_ipc_buf_b, kIpcMaxMessageSize, &msg, -1);
  return 3;
}

static int64_t RunBlockMockDeadlock() {
  if (IpcListen(kIpcPortBlock) != 0) {
    return 1;
  }
  constexpr int64_t kScratchCap = static_cast<int64_t>(sizeof(g_block_scratch));
  if (GetBlockDeviceInfo(0).status() != kSyscallErrorDeadlock ||
      ReadBlocksChecked(0, 0, 1, 512, g_ipc_buf_a, 512) !=
          kSyscallErrorDeadlock ||
      WriteBlocksWithScratch(0,                //
                             0,                //
                             1,                //
                             g_ipc_buf_a,      //
                             512,              //
                             g_block_scratch,  //
                             kScratchCap) != kSyscallErrorDeadlock ||
      FlushDisk(0) != kSyscallErrorDeadlock) {
    return 2;
  }
  return 0;
}

static int64_t RunBlockChecks(const uint64_t arg) {
  using usertest::BlockTestSubmode;
  const uint32_t disk_index = usertest::BlockDiskIndex(arg);
  switch (static_cast<BlockTestSubmode>(usertest::DriverSubmode(arg))) {
    case BlockTestSubmode::kWriteReadVerify:
      return RunBlockWriteReadVerify(disk_index);
    case BlockTestSubmode::kVerifyPersisted:
      return RunBlockVerifyPersisted(disk_index);
    case BlockTestSubmode::kNoServer:
      return RunBlockNoServer(disk_index);
    case BlockTestSubmode::kCliDiskChecks:
      return RunBlockCliDiskChecks(disk_index);
    case BlockTestSubmode::kMockMalformedReplies:
      return RunBlockMockMalformedReplies();
    case BlockTestSubmode::kMockClientGetInfoOnly:
      return RunBlockMockClientGetInfoOnly();
    case BlockTestSubmode::kMockClientCallWait:
      return RunBlockMockClientCallWait();
    case BlockTestSubmode::kMockServerHoldCall:
      return RunBlockMockServerHoldCall();
    case BlockTestSubmode::kMockDeadlock:
      return RunBlockMockDeadlock();
  }
  return usertest::kBadSubmodeStatus;
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
    case Mode::kDriverNvmeReset:
      return RunDriverNvmeReset(arg);
    case Mode::kBlockChecks:
      return RunBlockChecks(arg);
    case Mode::kIpcChecks:
      return RunIpcChecks(arg);
    case Mode::kFsChecks:
      return RunFsChecks(arg);
  }
  // The mode should have faulted, or it is unknown.
  return -1;
}

}  // namespace protos::user
