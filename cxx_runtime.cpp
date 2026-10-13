// Freestanding C and C++ runtime support for the kernel.
//
// The kernel is built with -ffreestanding -nostdlib and links neither libc nor
// libstdc++/libsupc++. This file provides what the compiler and the
// header-only parts of libstdc++ used by the kernel still expect from them:
// - `memcpy`, `memmove`, `memset` and `memcmp`, which GCC may call even in
//   freestanding code, for example for large struct copies or for loops that
//   it recognizes as one of them. They use `rep movsb`/`rep stosb` and plain
//   byte loops that GCC never turns back into calls to themselves.
// - The Itanium C++ ABI hooks: `__cxa_pure_virtual`/`__cxa_deleted_virtual`,
//   `__cxa_atexit`/`__dso_handle`, and `__cxa_guard_*`, which make the dynamic
//   initialization of function-local statics thread-safe.
// - Symbols referenced by libstdc++ headers: `__libc_single_threaded`,
//   `std::nothrow` and `std::_Sp_make_shared_tag::_S_eq`.
// The global `operator new`/`operator delete` forms live in heap.cpp, next to
// the allocator they wrap. Host builds compile only the `protos::Runtime*`
// functions (see cxx_runtime.h) for unit tests.

#include "cxx_runtime.h"

#include <cstddef>
#include <cstdint>

#if !__STDC_HOSTED__
#include <memory>
#include <new>

#include "check.h"
#endif

namespace protos {

namespace {

// Byte 0 of an Itanium C++ ABI guard object is nonzero once the guarded object
// is initialized; compiler-generated code checks it before calling
// `__cxa_guard_acquire`. Byte 1 is used here as the "initialization in
// progress" flag.
constexpr int kGuardInitializedByte = 0;
constexpr int kGuardInProgressByte = 1;

static uint8_t* GuardByte(int64_t* const guard, const int index) {
  return reinterpret_cast<uint8_t*>(guard) + index;
}

}  // namespace

void* RuntimeMemcpy(void* const dest,       //
                    const void* const src,  //
                    const size_t count) noexcept {
  void* dest_cursor = dest;
  const void* src_cursor = src;
  size_t remaining = count;
  asm volatile("rep movsb"
               : "+D"(dest_cursor), "+S"(src_cursor), "+c"(remaining)
               :
               : "memory");
  return dest;
}

void* RuntimeMemmove(void* const dest,       //
                     const void* const src,  //
                     const size_t count) noexcept {
  const uintptr_t dest_addr = reinterpret_cast<uintptr_t>(dest);
  const uintptr_t src_addr = reinterpret_cast<uintptr_t>(src);
  // A forward copy is safe unless `dest` starts inside `(src, src + count)`;
  // the unsigned difference wraps around when `dest < src`.
  if (dest_addr == src_addr || dest_addr - src_addr >= count) {
    return RuntimeMemcpy(dest, src, count);
  }
  // Copy backwards, last byte first, with the direction flag set. Interrupt
  // and signal handlers run with DF clear and IRET restores the interrupted
  // DF, so the temporarily set flag cannot leak into other code.
  uintptr_t dest_cursor = dest_addr + count - 1;
  uintptr_t src_cursor = src_addr + count - 1;
  size_t remaining = count;
  asm volatile("std\n\trep movsb\n\tcld"
               : "+D"(dest_cursor), "+S"(src_cursor), "+c"(remaining)
               :
               : "memory", "cc");
  return dest;
}

void* RuntimeMemset(void* const dest,  //
                    const int value,   //
                    const size_t count) noexcept {
  void* dest_cursor = dest;
  size_t remaining = count;
  asm volatile("rep stosb"
               : "+D"(dest_cursor), "+c"(remaining)
               : "a"(value)
               : "memory");
  return dest;
}

int RuntimeMemcmp(const void* const lhs,  //
                  const void* const rhs,  //
                  const size_t count) noexcept {
  const uint8_t* const lhs_bytes = static_cast<const uint8_t*>(lhs);
  const uint8_t* const rhs_bytes = static_cast<const uint8_t*>(rhs);
  for (size_t i = 0; i < count; ++i) {
    if (lhs_bytes[i] != rhs_bytes[i]) {
      return (lhs_bytes[i] < rhs_bytes[i]) ? -1 : 1;
    }
  }
  return 0;
}

int RuntimeGuardAcquire(int64_t* const guard) noexcept {
  uint8_t* const initialized = GuardByte(guard, kGuardInitializedByte);
  uint8_t* const in_progress = GuardByte(guard, kGuardInProgressByte);
  for (;;) {
    if (__atomic_load_n(initialized, __ATOMIC_ACQUIRE) != 0) {
      return 0;
    }
    uint8_t expected = 0;
    if (__atomic_compare_exchange_n(in_progress,       //
                                    &expected,         //
                                    1,                 //
                                    false,             //
                                    __ATOMIC_ACQUIRE,  //
                                    __ATOMIC_RELAXED)) {
      // Another CPU may have finished the initialization between the check
      // above and taking the flag.
      if (__atomic_load_n(initialized, __ATOMIC_ACQUIRE) != 0) {
        __atomic_store_n(in_progress, 0, __ATOMIC_RELEASE);
        return 0;
      }
      return 1;
    }
    asm volatile("pause" : : : "memory");
  }
}

void RuntimeGuardRelease(int64_t* const guard) noexcept {
  uint8_t* const initialized = GuardByte(guard, kGuardInitializedByte);
  uint8_t* const in_progress = GuardByte(guard, kGuardInProgressByte);
  __atomic_store_n(initialized, 1, __ATOMIC_RELEASE);
  __atomic_store_n(in_progress, 0, __ATOMIC_RELEASE);
}

void RuntimeGuardAbort(int64_t* const guard) noexcept {
  __atomic_store_n(GuardByte(guard, kGuardInProgressByte), 0, __ATOMIC_RELEASE);
}

}  // namespace protos

#if !__STDC_HOSTED__
extern "C" {

void* memcpy(void* const dest,       //
             const void* const src,  //
             const size_t count) noexcept {
  return protos::RuntimeMemcpy(dest, src, count);
}

void* memmove(void* const dest,       //
              const void* const src,  //
              const size_t count) noexcept {
  return protos::RuntimeMemmove(dest, src, count);
}

void* memset(void* const dest, const int value, const size_t count) noexcept {
  return protos::RuntimeMemset(dest, value, count);
}

int memcmp(const void* const lhs,  //
           const void* const rhs,  //
           const size_t count) noexcept {
  return protos::RuntimeMemcmp(lhs, rhs, count);
}

int __cxa_guard_acquire(int64_t* const guard) noexcept {
  return protos::RuntimeGuardAcquire(guard);
}

void __cxa_guard_release(int64_t* const guard) noexcept {
  protos::RuntimeGuardRelease(guard);
}

void __cxa_guard_abort(int64_t* const guard) noexcept {
  protos::RuntimeGuardAbort(guard);
}

// Vtable entries for pure virtual (`= 0`) and deleted virtual functions point
// here; reaching one means a call through a partially constructed or
// destroyed object.
void __cxa_pure_virtual() { CHECK(false && "pure virtual function called"); }

void __cxa_deleted_virtual() {
  CHECK(false && "deleted virtual function called");
}

// GCC registers the destructors of objects with static storage duration via
// `__cxa_atexit(dtor, object, &__dso_handle)`. The kernel never exits, so the
// destructors never run and registration is a no-op.
void* __dso_handle = nullptr;

int __cxa_atexit(void (*const func)(void*),  //
                 void* const arg,            //
                 void* const dso_handle) noexcept {
  (void)func;
  (void)arg;
  (void)dso_handle;
  return 0;
}

// libstdc++ skips atomic instructions in reference counting (e.g. in
// `std::shared_ptr`) while this is nonzero. The kernel is multi-threaded.
char __libc_single_threaded = 0;

}  // extern "C"

namespace std {

const nothrow_t nothrow{};

// `std::make_shared`'s control block (`_Sp_counted_ptr_inplace`) calls this
// libstdc++-internal function (declared in <bits/shared_ptr_base.h> since GCC
// 9 and normally defined in libstdc++) under -fno-rtti, as a fallback after its
// `_M_get_deleter` compared the requested `type_info` against
// `_Sp_make_shared_tag::_S_ti()` by address. In a single static binary without
// RTTI no other `type_info` can denote the tag, so the fallback never matches.
bool _Sp_make_shared_tag::_S_eq(const type_info&) noexcept { return false; }

}  // namespace std
#endif  // !__STDC_HOSTED__
