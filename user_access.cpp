#include "user_access.h"

#include <cstdint>
#if __STDC_HOSTED__
#include <cstring>
#endif

#include "check.h"
#include "cpu.h"
#include "paging.h"

namespace protos {

namespace {

#if !__STDC_HOSTED__
// Returns the address through which the kernel reaches the user memory at
// `user_addr`: the address itself, in the active address space.
static void* UserMemory(const uintptr_t user_addr, int64_t /*size*/) {
  return reinterpret_cast<void*>(user_addr);
}

// Copies `size` (> 0) bytes from `src` to `dst`, one of which is user memory.
static void CopyBytes(void* const dst,        //
                      const void* const src,  //
                      const int64_t size) {
  void* dst_cursor = dst;
  const void* src_cursor = src;
  int64_t remaining = size;
  if (CpuSmapEnabled()) {
    // One asm statement: nothing but `rep movsb` runs with SMAP lifted (see
    // user_access.h about interrupts in the middle).
    asm volatile(
        "stac\n\t"
        "rep movsb\n\t"
        "clac"
        : "+D"(dst_cursor), "+S"(src_cursor), "+c"(remaining)
        :
        : "memory", "cc");
  } else {
    asm volatile("rep movsb"
                 : "+D"(dst_cursor), "+S"(src_cursor), "+c"(remaining)
                 :
                 : "memory");
  }
}
#else
// See UserAccessSetHostMemoryForTest().
uintptr_t g_host_user_base = 0;
uint8_t* g_host_memory = nullptr;
int64_t g_host_memory_size = 0;
int g_host_copy_count = 0;
bool g_host_last_copy_lifted_smap = false;

static void* UserMemory(const uintptr_t user_addr, const int64_t size) {
  const bool in_host_memory =
      g_host_memory != nullptr && user_addr >= g_host_user_base &&
      size <= g_host_memory_size &&
      user_addr - g_host_user_base <=
          static_cast<uint64_t>(g_host_memory_size - size);
  CHECK(in_host_memory);
  return g_host_memory + (user_addr - g_host_user_base);
}

static void CopyBytes(void* const dst,        //
                      const void* const src,  //
                      const int64_t size) {
  ++g_host_copy_count;
  g_host_last_copy_lifted_smap = CpuSmapEnabled();
  std::memcpy(dst, src, static_cast<size_t>(size));
}
#endif

}  // namespace

bool CopyFromUser(void* const dst,           //
                  const uintptr_t user_src,  //
                  const int64_t size) {
  if (!IsUserRange(user_src, size)) {
    return false;
  }
  if (size > 0) {
    DCHECK(dst != nullptr);
    CopyBytes(dst, UserMemory(user_src, size), size);
  }
  return true;
}

bool CopyToUser(const uintptr_t user_dst,  //
                const void* const src,     //
                const int64_t size) {
  if (!IsUserRange(user_dst, size)) {
    return false;
  }
  if (size > 0) {
    DCHECK(src != nullptr);
    CopyBytes(UserMemory(user_dst, size), src, size);
  }
  return true;
}

#if __STDC_HOSTED__
void UserAccessSetHostMemoryForTest(const uintptr_t user_base,  //
                                    void* const memory,         //
                                    const int64_t size) {
  g_host_user_base = user_base;
  g_host_memory = static_cast<uint8_t*>(memory);
  g_host_memory_size = (memory != nullptr) ? size : 0;
  g_host_copy_count = 0;
  g_host_last_copy_lifted_smap = false;
}

int UserAccessGetHostCopyCountForTest() { return g_host_copy_count; }

bool UserAccessGetHostLastCopyLiftedSmapForTest() {
  return g_host_last_copy_lifted_smap;
}
#endif

}  // namespace protos
