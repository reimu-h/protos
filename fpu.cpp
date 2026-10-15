#include "fpu.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>

#include "check.h"
#include "interrupts.h"

namespace protos {

namespace {

// Set by FpuInit() on the BSP before the APs start; read on every task switch.
FpuConfig g_fpu_config;

}  // namespace

bool FpuConfigIsValid(const FpuConfig& config) {
  if (!config.xsave) {
    return config.xcr0 == 0 && config.state_size == kFpuFxsaveSize;
  }
  constexpr uint64_t kX87Sse = kXcr0X87 | kXcr0Sse;
  const uint64_t avx512 = config.xcr0 & kXcr0Avx512;
  return (config.xcr0 & ~kFpuSupportedXcr0) == 0 &&
         (config.xcr0 & kX87Sse) == kX87Sse &&
         (avx512 == 0 ||
          (avx512 == kXcr0Avx512 && (config.xcr0 & kXcr0Avx) != 0)) &&
         config.state_size >= kFpuXsaveMinSize &&
         config.state_size <= kFpuMaxStateSize;
}

void FpuInit(const FpuConfig& config) {
  const bool fpu_config_valid = FpuConfigIsValid(config);
  CHECK(fpu_config_valid);
  g_fpu_config = config;
}

const FpuConfig& FpuGetConfig() { return g_fpu_config; }

int FpuStateSize() { return g_fpu_config.state_size; }

void FpuBuildInitialImage(uint8_t* const image, const int size) {
  DCHECK(image != nullptr);
  DCHECK(size >= kFpuFxsaveSize);
  __builtin_memset(image, 0, static_cast<size_t>(size));
  const uint16_t fcw = kFpuInitialFcw;
  const uint32_t mxcsr = kFpuInitialMxcsr;
  __builtin_memcpy(image + kFpuFcwOffset, &fcw, sizeof(fcw));
  __builtin_memcpy(image + kFpuMxcsrOffset, &mxcsr, sizeof(mxcsr));
}

void FpuStateDeleter::operator()(FpuState* const state) const {
  ::operator delete(static_cast<void*>(state),
                    static_cast<std::align_val_t>(kFpuStateAlignment));
}

FpuStatePtr FpuStateCreate() {
  const int size = FpuStateSize();
  // In the kernel, the aligned nothrow operator new is KmallocAligned()
  // (heap.h).
  void* const raw =
      ::operator new(static_cast<size_t>(size),                          //
                     static_cast<std::align_val_t>(kFpuStateAlignment),  //
                     std::nothrow);
  if (raw == nullptr) {
    return nullptr;
  }
  DCHECK((reinterpret_cast<uintptr_t>(raw) & (kFpuStateAlignment - 1)) == 0);
  // The whole image is written: heap memory is not zeroed, and XRSTOR faults
  // unless the XSAVE header past XSTATE_BV is zero, which XSAVE never writes.
  FpuBuildInitialImage(static_cast<uint8_t*>(raw), size);
  return FpuStatePtr(static_cast<FpuState*>(raw));
}

#if !__STDC_HOSTED__
// XSAVE64 and FXSAVE64 never raise #MF: a pending x87 exception stays pending
// in the saved status word, and the program that left it pending raises it at
// its next waiting x87 instruction, in user mode. XRSTOR64 and FXRSTOR64 raise
// #GP only for a malformed XSAVE header or reserved MXCSR bits, which a state
// that only FpuStateCreate() and these saves wrote never has.
void FpuSave(FpuState* const state) {
  DCHECK(state != nullptr);
  DCHECK(!AreInterruptsEnabled());
  if (g_fpu_config.xsave) {
    asm volatile("xsave64 [%0]"
                 :
                 : "r"(state), "a"(static_cast<uint32_t>(g_fpu_config.xcr0)),
                   "d"(static_cast<uint32_t>(g_fpu_config.xcr0 >> 32))
                 : "memory");
  } else {
    asm volatile("fxsave64 [%0]" : : "r"(state) : "memory");
  }
}

void FpuRestore(const FpuState* const state) {
  DCHECK(state != nullptr);
  DCHECK(!AreInterruptsEnabled());
  if (g_fpu_config.reset_x87_pointers) {
    // These CPUs load the last x87 instruction and operand pointers and
    // opcode only from a state with an x87 exception pending, and otherwise
    // keep those of the last x87 instruction, which another process ran. An
    // integer load from kernel memory makes them the kernel's (as in Linux):
    // FNCLEX first discards a pending exception, at which EMMS and FILD would
    // raise #MF (the previous task's registers are saved already), and EMMS
    // empties the register stack, which FILD would otherwise overflow. (QEMU's
    // TCG raises no #MF at EMMS, so only hardware and KVM show a missing
    // FNCLEX.) Only the assembler sees these instructions
    // (-mgeneral-regs-only).
    static constexpr int32_t kResetOperand = 0;
    asm volatile("fnclex\n\temms\n\tfild dword ptr [%0]"
                 :
                 : "r"(&kResetOperand)
                 : "memory");
  }
  if (g_fpu_config.xsave) {
    asm volatile("xrstor64 [%0]"
                 :
                 : "r"(state), "a"(static_cast<uint32_t>(g_fpu_config.xcr0)),
                   "d"(static_cast<uint32_t>(g_fpu_config.xcr0 >> 32))
                 : "memory");
  } else {
    asm volatile("fxrstor64 [%0]" : : "r"(state) : "memory");
  }
}
#endif

}  // namespace protos
