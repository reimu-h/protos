#ifndef PROTOS_ELF_H_
#define PROTOS_ELF_H_

#include <cstdint>

#include "paging.h"

namespace protos {

// The most loadable segments that an executable may have.
constexpr int kElfMaxSegments = 8;

// A loadable segment of an executable: `mem_size` bytes at `vaddr` in user
// space, of which the first `file_size` bytes come from the file at
// `file_offset` and the rest are zero.
struct ElfSegment {
  // Page-aligned.
  uintptr_t vaddr = 0;
  // Positive.
  int64_t mem_size = 0;
  int64_t file_offset = 0;
  // At most `mem_size`.
  int64_t file_size = 0;
  // From the segment's flags: writable segments are `kReadWrite`, executable
  // ones `kReadExecute`, and others `kRead`.
  UserPageAccess access = UserPageAccess::kRead;
};

// What a user program's executable says about how to load and start it.
struct ElfExecutable {
  // The address of the first instruction, in an executable segment.
  uintptr_t entry = 0;
  int segment_count = 0;
  // The loadable segments in file order. No two share a page.
  ElfSegment segments[kElfMaxSegments];
};

// Why `ElfParseExecutable()` rejected a file.
enum class ElfError : uint8_t {
  kNone,
  // The file ends before its headers do.
  kTruncated,
  // Not a little-endian 64-bit ELF file of the current version.
  kNotElf64,
  // Not a statically linked x86-64 executable.
  kNotExecutable,
  // The program header table has entries of the wrong size.
  kBadProgramHeaders,
  // More than `kElfMaxSegments` loadable segments.
  kTooManySegments,
  // A loadable segment's address is not page-aligned, its file size exceeds
  // its memory size, or its contents lie outside the file.
  kBadSegment,
  // A loadable segment lies outside user space (paging.h).
  kNotInUserSpace,
  // A loadable segment is both writable and executable.
  kWritableAndExecutable,
  // Two loadable segments share a page.
  kOverlappingSegments,
  // The entry point is not in an executable segment.
  kBadEntry,
};

// Parses the ELF executable `[data, data + size)` (`size >= 0`) and checks
// that it can run as a user program: a statically linked little-endian x86-64
// executable (ELF64, `ET_EXEC`) whose loadable segments start at page-aligned
// user-space addresses, take their contents from inside the file, share no
// page, and are not both writable and executable, and whose entry point lies
// in an executable segment. Loadable segments of size 0 are left out. On
// success, stores the result in `*out` and returns `ElfError::kNone`;
// otherwise returns the first problem found. The file may be unaligned.
ElfError ElfParseExecutable(const uint8_t* data,  //
                            int64_t size,         //
                            ElfExecutable* out);

}  // namespace protos

#endif  // PROTOS_ELF_H_

