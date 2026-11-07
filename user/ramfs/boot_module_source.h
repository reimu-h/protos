#ifndef PROTOS_USER_RAMFS_BOOT_MODULE_SOURCE_H_
#define PROTOS_USER_RAMFS_BOOT_MODULE_SOURCE_H_

#include <cstdint>

#include "syscall_abi.h"
#include "user/ramfs/tar.h"
#include "user/syscall.h"

namespace protos::user {

// A `tar::Source` that reads a boot module with BootModuleRead(), which only
// drivers may make: the boot filesystem (`kRootfsBootModuleName`) for `ramfs`,
// which unpacks it, and for the test program that compares the filesystem
// with it.
class BootModuleSource final : public tar::Source {
 public:
  // Reads boot module `index`.
  constexpr explicit BootModuleSource(const int64_t index) : index_(index) {}

  int64_t Read(const int64_t offset,  //
               void* const buffer,    //
               const int64_t size) override {
    const int64_t chunk = (size < kSyscallBootModuleReadMaxSize)
                              ? size
                              : kSyscallBootModuleReadMaxSize;
    return ReadBootModule(index_, offset, buffer, chunk);
  }

 private:
  int64_t index_;
};

}  // namespace protos::user

#endif  // PROTOS_USER_RAMFS_BOOT_MODULE_SOURCE_H_
