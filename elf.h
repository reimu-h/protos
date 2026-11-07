#ifndef PROTOS_ELF_H_
#define PROTOS_ELF_H_

#include <cstdint>

#include "paging.h"
#include "status_or.h"

namespace protos {

// The most loadable segments that an executable may have.
constexpr int kElfMaxSegments = 8;

// A loadable segment of an executable: `mem_size` bytes at `vaddr` in user
// space, of which the first `file_size` bytes come from the file at
// `file_offset` and the rest are zero. Its mapped pages start at
// `vaddr & ~0xFFF`, with any bytes below `vaddr` zeroed.
struct ElfSegment {
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
  // A loadable segment's file size exceeds its memory size, or its contents lie
  // outside the file.
  kBadSegment,
  // A loadable segment lies outside user space (paging.h).
  kNotInUserSpace,
  // A loadable segment is both writable and executable.
  kWritableAndExecutable,
  // Two loadable segments share a page.
  kOverlappingSegments,
  // The entry point is not in an executable segment.
  kBadEntry,
  // Reading the file failed (`*out_read_error` holds the error).
  kReadError,
};

// Copies `size` (> 0) bytes of an executable of `file_size` bytes from
// `offset` (`0 <= offset` and `offset + size <= file_size`) into `buffer`,
// returning 0 on success, `> 0` if the file ended before `offset + size`
// (`ElfError::kTruncated`), or `< 0` for a read error (`ElfError::kReadError`).
using ElfReadFn = int64_t (*)(void* ctx,       //
                              int64_t offset,  //
                              void* buffer,    //
                              int64_t size);

// Parses the ELF executable `[data, data + size)` (`size >= 0`) and checks
// that it can run as a user program: a statically linked little-endian x86-64
// executable (ELF64, `ET_EXEC`) whose loadable segments lie in user space, take
// their contents from inside the file, share no page, and are not both
// writable and executable, and whose entry point lies in an executable
// segment. Loadable segments of size 0 are left out. On success, returns the
// parsed executable; otherwise returns the first problem found. The file may
// be unaligned.
StatusOr<ElfExecutable, ElfError> ElfParseExecutable(const uint8_t* data,
                                                     int64_t size);

// Like `ElfParseExecutable(data, size)`, for an executable of `size` (>= 0)
// bytes whose header and program header table `read_fn` reads on demand. If
// `read_fn` returns a negative error, stores it in `*out_read_error` (unless
// null) and fails with `ElfError::kReadError`.
StatusOr<ElfExecutable, ElfError> ElfParseExecutable(
    int64_t size,       //
    ElfReadFn read_fn,  //
    void* ctx,          //
    int64_t* out_read_error = nullptr);

}  // namespace protos

#endif  // PROTOS_ELF_H_
