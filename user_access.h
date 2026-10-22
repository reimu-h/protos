#ifndef PROTOS_USER_ACCESS_H_
#define PROTOS_USER_ACCESS_H_

#include <cstdint>

#include "cpu.h"
#include "interrupts.h"

namespace protos {

// Kernel access to user memory
// ============================
//
// With SMAP enabled (cpu.h), every kernel access to a user page faults except
// the copies of CopyFromUser() and CopyToUser(): they lift the protection
// (STAC) for exactly their `rep movsb` and restore it (CLAC) in the same asm
// statement, so that no other code, not even a call or spill that the
// compiler adds, runs with it lifted. Without SMAP they copy the same way
// without STAC and CLAC, which raise #UD there.
//
// The copies run with interrupts enabled. An interrupt in the middle of one
// saves RFLAGS.AC (set) in its frame, and the entry stubs (boot.S) clear AC,
// so that the handler, and any task that the scheduler switches to, runs with
// SMAP enforced; iretq restores the copy's AC, also when the task was switched
// out meanwhile and resumes on another CPU (in its own address space, task.h).
// Everywhere else UserAccessWindowOpen() is false, which IdtDispatch() and
// every task switch DCHECK.
//
// The helpers check only that the range lies in user space. The caller first
// checks that the running program may access every page of it as owned user
// RAM (PagingUserRangeAccessible() in its address space, which also rejects
// device MMIO pages, as UserProcess::MayRead() does) and fails with
// kSyscallErrorFault otherwise. A program's mappings don't change while one of
// its system calls runs (only its own task maps and unmaps them, before it
// enters user mode and after it has left), so a copy of a checked range can't
// fault; if it does, the kernel panics as for any other fault of kernel code.

// Copies the `size` bytes at `user_src` in the active address space to `dst`.
// Returns false, copying nothing, unless `[user_src, user_src + size)` lies in
// user space (IsUserRange(), paging.h, which also rejects `size < 0`).
bool CopyFromUser(void* dst, uintptr_t user_src, int64_t size);

// Copies the `size` bytes at `src` to `user_dst` in the active address space,
// whose pages the caller has checked to be mapped and writable. Returns false,
// copying nothing, unless `[user_dst, user_dst + size)` lies in user space.
bool CopyToUser(uintptr_t user_dst, const void* src, int64_t size);

namespace internal {
#if __STDC_HOSTED__
// Host builds simulate the window per thread; out of line and opaque for the
// reasons given at HostInterruptEnabledSlot() (interrupts.h).
[[gnu::noinline]] inline bool* HostUserAccessWindowSlot() {
  thread_local bool open = false;
  asm volatile("" : "+m"(open));
  return &open;
}
#endif
}  // namespace internal

// Returns true if kernel code on the calling CPU can reach user pages now:
// SMAP is enabled and RFLAGS.AC is set, which only the copies of the helpers
// above do. (Without SMAP, AC means nothing to kernel code, and user code may
// have left it set.) Host builds: simulated per thread
// (SetUserAccessWindowOpenForTest()).
inline bool UserAccessWindowOpen() {
#if !__STDC_HOSTED__
  if (!CpuSmapEnabled()) {
    return false;
  }
  uint64_t rflags = 0;
  asm volatile(
      "pushfq\n\t"
      "pop %0"
      : "=r"(rflags)
      :
      : "memory");
  return (rflags & kRflagsAlignmentCheckBit) != 0;
#else
  return *internal::HostUserAccessWindowSlot();
#endif
}

#if __STDC_HOSTED__
// Host builds have no user space: the helpers reach the user memory at
// `[user_base, user_base + size)` through `memory` instead, which the test
// installs (nullptr uninstalls); a copy of other user memory CHECK-fails
// (tests only). Installing resets the counters below.
void UserAccessSetHostMemoryForTest(uintptr_t user_base,  //
                                    void* memory,         //
                                    int64_t size);
// The number of copies that reached the installed memory, and whether the
// last one lifted SMAP, which a copy does if CpuSmapEnabled() (tests only).
int UserAccessGetHostCopyCountForTest();
bool UserAccessGetHostLastCopyLiftedSmapForTest();
// Sets the simulated UserAccessWindowOpen() of the calling thread (tests
// only).
inline void SetUserAccessWindowOpenForTest(const bool open) {
  *internal::HostUserAccessWindowSlot() = open;
}
#endif

}  // namespace protos

#endif  // PROTOS_USER_ACCESS_H_
