#ifndef PROTOS_FPU_H_
#define PROTOS_FPU_H_

#include <cstdint>
#include <memory>

namespace protos {

// x87/MMX/SSE/AVX register state of user processes
// ================================================
//
// Kernel code never uses the x87, MMX, SSE or AVX registers (it is built with
// -mgeneral-regs-only), so they keep the values of the user program that ran
// last until another program's values are loaded. Each user process has an
// `FpuState`: the scheduler saves the registers into it whenever it switches
// the process's task out, and loads them from it whenever a CPU resumes the
// task (TaskScheduler::SetFpuState(), task.h). A new state holds the
// architectural initial state.
//
// A state has the standard (non-compacted) format of XSAVE, with the state
// components that the kernel enabled in XCR0, if the CPUs support XSAVE, and
// the FXSAVE format otherwise. The cpu module (cpu.h) chooses the format at
// boot and passes it to FpuInit().

// XCR0 state components (Intel SDM Vol. 1, 13.1).
constexpr uint64_t kXcr0X87 = uint64_t{1} << 0;
constexpr uint64_t kXcr0Sse = uint64_t{1} << 1;
constexpr uint64_t kXcr0Avx = uint64_t{1} << 2;
// The AVX-512 components: the opmask registers, the upper halves of
// ZMM0-ZMM15, and ZMM16-ZMM31. XCR0 enables all three or none of them.
constexpr uint64_t kXcr0Opmask = uint64_t{1} << 5;
constexpr uint64_t kXcr0ZmmHi256 = uint64_t{1} << 6;
constexpr uint64_t kXcr0Hi16Zmm = uint64_t{1} << 7;
constexpr uint64_t kXcr0Avx512 = kXcr0Opmask | kXcr0ZmmHi256 | kXcr0Hi16Zmm;
// The components that the kernel may enable (never MPX, PKRU, AMX or others).
constexpr uint64_t kFpuSupportedXcr0 =
    kXcr0X87 | kXcr0Sse | kXcr0Avx | kXcr0Avx512;

// Offsets of the x87 control word and of MXCSR in the legacy region, the
// first 512 bytes of an XSAVE area, which have the FXSAVE format (SDM Vol. 1,
// Table 10-2).
constexpr int kFpuFcwOffset = 0;
constexpr int kFpuMxcsrOffset = 24;
// The size of an FXSAVE area, which is also the size of the legacy region.
constexpr int kFpuFxsaveSize = 512;
// The legacy region and the 64-byte XSAVE header: the smallest XSAVE area.
constexpr int kFpuXsaveMinSize = 576;
// The largest state that the kernel accepts. The components that it may
// enable take 2688 bytes.
constexpr int kFpuMaxStateSize = 16384;
// XSAVE and XRSTOR need a 64-byte-aligned area (FXSAVE and FXRSTOR 16 bytes).
constexpr int kFpuStateAlignment = 64;

// The architectural initial state, which FNINIT and XRSTOR of a component in
// its initial configuration set up, and which the x86-64 System V ABI gives a
// new process: x87 control word 0x037F (all x87 exceptions masked, 64-bit
// precision, rounding to nearest), MXCSR 0x1F80 (all SIMD exceptions masked,
// rounding to nearest), empty x87 registers, and every other register zero.
constexpr uint16_t kFpuInitialFcw = 0x037F;
constexpr uint32_t kFpuInitialMxcsr = 0x1F80;

// The format of every FpuState.
struct FpuConfig {
  // Whether states are saved with XSAVE and loaded with XRSTOR (standard
  // format, with EDX:EAX = `xcr0`) instead of with FXSAVE and FXRSTOR.
  bool xsave = false;
  // XCR0 if `xsave`, else 0.
  uint64_t xcr0 = 0;
  // The size of a state in bytes: the XSAVE area size that CPUID.(EAX=0DH,
  // ECX=0):EBX reports for `xcr0`, or `kFpuFxsaveSize`.
  int state_size = kFpuFxsaveSize;
  // Whether FpuRestore() first gives the last x87 instruction and operand
  // pointers and opcode fixed values, the kernel's (CpuConfig, cpu.h).
  bool reset_x87_pointers = false;
};

// Returns true if `config` is consistent: FXSAVE with `xcr0` 0 and
// `kFpuFxsaveSize` bytes, or XSAVE with the x87 and SSE components, no
// component outside `kFpuSupportedXcr0`, the AVX-512 components all or none
// and only with AVX, and a size from `kFpuXsaveMinSize` to `kFpuMaxStateSize`.
bool FpuConfigIsValid(const FpuConfig& config);

// Sets the format of every FpuState. CpuInitBsp() (cpu.h) calls it once at
// boot, before the first FpuStateCreate(). Until then the format is FXSAVE.
// CHECK-fails if `config` is not valid (FpuConfigIsValid()).
void FpuInit(const FpuConfig& config);

// Returns the format that FpuInit() set.
const FpuConfig& FpuGetConfig();

// Returns the size of an FpuState in bytes: `FpuGetConfig().state_size`.
int FpuStateSize();

// Writes the architectural initial state into the `size` bytes at `image`
// (`size >= kFpuFxsaveSize`): all bytes zero except the x87 control word
// (`kFpuInitialFcw`) and MXCSR (`kFpuInitialMxcsr`). With the zero XSAVE
// header (XSTATE_BV = 0), XRSTOR puts every component into its initial
// configuration, but it still loads MXCSR from the image, which must
// therefore hold 0x1F80 as well; FXRSTOR loads every field from the image.
void FpuBuildInitialImage(uint8_t* image, int size);

// The saved registers of a task: `FpuStateSize()` bytes, 64-byte aligned,
// whose contents only the CPU interprets.
struct FpuState;

// Frees an FpuState that FpuStateCreate() allocated.
struct FpuStateDeleter {
  void operator()(FpuState* state) const;
};

using FpuStatePtr = std::unique_ptr<FpuState, FpuStateDeleter>;

// Allocates an FpuState that holds the architectural initial state
// (FpuBuildInitialImage()), or returns nullptr if memory runs out.
FpuStatePtr FpuStateCreate();

// Saves the calling CPU's x87/MMX/SSE/AVX registers into `state`, or loads
// them from it, with XSAVE64/XRSTOR64 (every component that XCR0 enables) or
// FXSAVE64/FXRSTOR64, as FpuInit() set. If `reset_x87_pointers`,
// FpuRestore() first executes FNCLEX, EMMS and an FILD from kernel memory, as
// Linux does on such CPUs. The scheduler calls them with interrupts disabled.
// Host builds of fpu.cpp don't define them: the scheduler tests link
// recording fakes instead (scheduler_test_fakes.cpp).
void FpuSave(FpuState* state);
void FpuRestore(const FpuState* state);

}  // namespace protos

#endif  // PROTOS_FPU_H_
