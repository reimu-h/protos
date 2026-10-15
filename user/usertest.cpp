// usertest: a user program that the boot self-tests run in many ways to check
// user mode (see user/usertest.h).

#include "user/usertest.h"

#include <cstdint>

#include "syscall_abi.h"
#include "user/syscall.h"

static_assert(protos::kSyscallGetCpu == 2);

// Makes a get_cpu() system call with distinct values in every general-purpose
// register but rax and rsp, and returns the bits of these registers that the
// call changed: 0 if it preserved them all.
extern "C" uint64_t SyscallChangedRegisterBits();

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

namespace protos::user {
namespace {

using usertest::Mode;

constexpr uint64_t kRflagsInterruptEnable = 1 << 9;
constexpr uint64_t kRflagsIoPrivilegeLevel = 3 << 12;
// kMaxSupportedCpus in smp.h.
constexpr int kMaxCpus = 128;

const char kNulBytes[8] = {};
constexpr int64_t kNulByteCount = sizeof(kNulBytes);
const uint64_t kReadOnlyWord = 0x0123456789ABCDEF;
// `ret`, in writable data.
uint8_t g_return_instruction[16] = {0xC3};
// What `kSpin` writes to its data: each process has its own copy.
uint64_t g_spin_seed = 0;
// Never set: `Recurse()` recurses until the stack overflows.
volatile uint64_t g_recursion_limit = 0;

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

// Runs the checks of `Mode::kSyscalls` and returns 0 if all pass, else the
// number of the first failed check.
static int64_t RunSyscallChecks() {
  int64_t check = 1;
  if (CurrentPrivilegeLevel() != 3) {
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

}  // namespace

int64_t Main(const uint64_t arg) {
  switch (static_cast<Mode>(arg & 0xFF)) {
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
  }
  // The mode should have faulted, or it is unknown.
  return -1;
}

}  // namespace protos::user
