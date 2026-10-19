#ifndef PROTOS_ADDRESS_SPACE_H_
#define PROTOS_ADDRESS_SPACE_H_

#include <cstdint>
#include <memory>

#include "paging.h"

namespace protos {

// A user address space: the page tables of one process, whose upper half maps
// the process's user pages and whose lower half maps the kernel the same way
// as every other address space (see the layout and "Page-table roots and user
// pages" in paging.h). It owns the frames of its user pages.
//
// A task runs in an address space through `TaskScheduler::SetAddressSpace()`
// (task.h), which makes every CPU that runs the task use the address space's
// page tables. Tasks start in the kernel's page tables, which map no user
// page.
//
// Methods may be called while tasks run in the address space and on several
// CPUs at once. The address space must outlive its use by every task.
class AddressSpace {
 public:
  // Creates an address space that maps no user page, or returns nullptr if out
  // of memory.
  static std::unique_ptr<AddressSpace> Create();

  // Frees every user page, the page tables and the root. No task may run in
  // the address space anymore: each task that used it must have switched to
  // another address space or exited.
  ~AddressSpace();

  AddressSpace(const AddressSpace&) = delete;
  AddressSpace& operator=(const AddressSpace&) = delete;

  // Maps a new zeroed page at page-aligned `user_addr`, which user code may
  // access as `access` allows, and returns the physical address of its frame.
  // The identity map reaches the frame, so the kernel can fill the page in any
  // address space. Returns 0 if `user_addr` is not a page-aligned user address
  // (paging.h) or is mapped already, or if memory runs out.
  uintptr_t MapNewPage(uintptr_t user_addr, UserPageAccess access);

  // Returns the physical address of the frame that the page at page-aligned
  // `user_addr` maps to, or 0 if it is not mapped.
  uintptr_t PageFrame(uintptr_t user_addr) const;

  // The root of the page tables: the physical address of the PML4.
  uintptr_t root() const { return root_; }

  void SetInjectTeardownDelayTicks(const int64_t ticks) {
    inject_teardown_delay_ticks_ = ticks;
  }

 private:
  explicit AddressSpace(uintptr_t root);

  const uintptr_t root_;
  int64_t inject_teardown_delay_ticks_ = 0;
};

// Switches the calling CPU to the page tables of `address_space`, or to the
// kernel's if it is nullptr. The scheduler calls it when a CPU switches between
// tasks in different address spaces; host tests of the scheduler replace it
// with a fake (scheduler_test_fakes.cpp).
void ActivateAddressSpace(const AddressSpace* address_space);

}  // namespace protos

#endif  // PROTOS_ADDRESS_SPACE_H_
