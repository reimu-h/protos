#ifndef PROTOS_CXX_RUNTIME_H_
#define PROTOS_CXX_RUNTIME_H_

// Implementations of the C library and Itanium C++ ABI functions that the
// freestanding kernel provides itself (see cxx_runtime.cpp). Kernel builds
// export them under their standard names (`memcpy`, `__cxa_guard_acquire`,
// ...); host builds compile only these functions, so that unit tests can call
// them without replacing the host's C and C++ runtime.

#include <cstddef>
#include <cstdint>

namespace protos {

// `memcpy`: copies `count` bytes from `src` to `dest` (the ranges must not
// overlap) and returns `dest`.
void* RuntimeMemcpy(void* dest, const void* src, size_t count) noexcept;

// `memmove`: copies `count` bytes from `src` to `dest`, which may overlap, and
// returns `dest`.
void* RuntimeMemmove(void* dest, const void* src, size_t count) noexcept;

// `memset`: fills `count` bytes at `dest` with `static_cast<uint8_t>(value)`
// and returns `dest`.
void* RuntimeMemset(void* dest, int value, size_t count) noexcept;

// `memcmp`: compares `count` bytes as unsigned chars and returns a negative
// value, zero, or a positive value if `lhs` is less than, equal to, or greater
// than `rhs`.
int RuntimeMemcmp(const void* lhs, const void* rhs, size_t count) noexcept;

// `__cxa_guard_acquire`: called by compiler-generated code before the dynamic
// initialization of a function-local static whose 64-bit guard object is
// `*guard`. Returns 1 if the caller must run the initializer (and then call
// `RuntimeGuardRelease` or, if it is abandoned, `RuntimeGuardAbort`), or 0 if
// the object is already initialized. Concurrent callers spin until the
// initializing CPU releases or aborts the guard, so an initializer must not
// recursively use its own static, and statics used from interrupt handlers
// must not need dynamic initialization.
int RuntimeGuardAcquire(int64_t* guard) noexcept;

// `__cxa_guard_release`: marks the object guarded by `*guard` as initialized.
void RuntimeGuardRelease(int64_t* guard) noexcept;

// `__cxa_guard_abort`: lets another caller retry the initialization guarded by
// `*guard`.
void RuntimeGuardAbort(int64_t* guard) noexcept;

}  // namespace protos

#endif  // PROTOS_CXX_RUNTIME_H_
