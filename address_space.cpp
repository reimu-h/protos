#include "address_space.h"

#include <cstdint>
#include <memory>
#include <new>

#include "check.h"
#include "paging.h"
#include "pmm.h"

namespace protos {

std::unique_ptr<AddressSpace> AddressSpace::Create() {
  const uintptr_t root = PagingCreateUserRoot();
  if (root == 0) {
    return nullptr;
  }
  AddressSpace* const address_space = new (std::nothrow) AddressSpace(root);
  if (address_space == nullptr) {
    PagingDestroyUserRoot(root);
    return nullptr;
  }
  return std::unique_ptr<AddressSpace>(address_space);
}

AddressSpace::AddressSpace(const uintptr_t root) : root_(root) {
  DCHECK(root_ != 0);
}

AddressSpace::~AddressSpace() { PagingDestroyUserRoot(root_); }

uintptr_t AddressSpace::MapNewPage(const uintptr_t user_addr,
                                   const UserPageAccess access) {
  const uintptr_t frame = PmmAllocFrame();
  if (frame == 0) {
    return 0;
  }
  constexpr int64_t kWordsPerPage = kPageSize / sizeof(uint64_t);
  uint64_t* const words = reinterpret_cast<uint64_t*>(frame);
  for (int64_t i = 0; i < kWordsPerPage; ++i) {
    words[i] = 0;
  }
  if (!PagingMapUserPage(root_, user_addr, frame, access)) {
    PmmFreeFrame(frame);
    return 0;
  }
  return frame;
}

uintptr_t AddressSpace::PageFrame(const uintptr_t user_addr) const {
  return PagingUserPageFrame(root_, user_addr);
}

void ActivateAddressSpace(const AddressSpace* const address_space) {
  PagingActivateRoot(address_space != nullptr ? address_space->root()
                                              : PagingKernelRoot());
}

}  // namespace protos
