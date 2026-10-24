#include "elf.h"

#include <cstdint>

#include "check.h"
#include "paging.h"

namespace protos {
namespace {

// ELF file header (System V ABI, "ELF Header").
struct Elf64Header {
  uint8_t ident[16];
  uint16_t type;
  uint16_t machine;
  uint32_t version;
  uint64_t entry;
  uint64_t program_header_offset;
  uint64_t section_header_offset;
  uint32_t flags;
  uint16_t header_size;
  uint16_t program_header_size;
  uint16_t program_header_count;
  uint16_t section_header_size;
  uint16_t section_header_count;
  uint16_t section_name_index;
};
static_assert(sizeof(Elf64Header) == 64);

// Program header table entry (System V ABI, "Program Header").
struct Elf64ProgramHeader {
  uint32_t type;
  uint32_t flags;
  uint64_t offset;
  uint64_t vaddr;
  uint64_t paddr;
  uint64_t file_size;
  uint64_t mem_size;
  uint64_t align;
};
static_assert(sizeof(Elf64ProgramHeader) == 56);

// `ident` bytes.
constexpr uint8_t kMagic[4] = {0x7F, 'E', 'L', 'F'};
constexpr int kIdentClass = 4;
constexpr int kIdentData = 5;
constexpr int kIdentVersion = 6;
constexpr uint8_t kClass64 = 2;           // ELFCLASS64
constexpr uint8_t kDataLittleEndian = 1;  // ELFDATA2LSB
constexpr uint8_t kVersionCurrent = 1;    // EV_CURRENT

constexpr uint16_t kTypeExecutable = 2;  // ET_EXEC
constexpr uint16_t kMachineX86_64 = 62;  // EM_X86_64

// Segment types and flags.
constexpr uint32_t kSegmentLoad = 1;         // PT_LOAD
constexpr uint32_t kSegmentDynamic = 2;      // PT_DYNAMIC
constexpr uint32_t kSegmentInterpreter = 3;  // PT_INTERP
constexpr uint32_t kSegmentExecutable = 1;   // PF_X
constexpr uint32_t kSegmentWritable = 2;     // PF_W

constexpr uint64_t kPageMask = 0xFFF;

// Reads a `T` from `data + offset`, which need not be aligned.
template <typename T>
static T ReadAt(const uint8_t* const data, const uint64_t offset) {
  T value;
  __builtin_memcpy(&value, data + offset, sizeof(T));
  return value;
}

// Returns the end of the last page that `segment` overlaps. It does not
// overflow: user space ends at a page boundary.
static uint64_t SegmentPageEnd(const ElfSegment& segment) {
  return (segment.vaddr + static_cast<uint64_t>(segment.mem_size) + kPageMask) &
         ~kPageMask;
}

static bool SharePage(const ElfSegment& a, const ElfSegment& b) {
  return a.vaddr < SegmentPageEnd(b) && b.vaddr < SegmentPageEnd(a);
}

// Checks the loadable segment `header`, whose memory size is not 0, of a file
// of `file_size` bytes.
static StatusOr<ElfSegment, ElfError> ParseSegment(
    const Elf64ProgramHeader& header, const uint64_t file_size) {
  if ((header.vaddr & kPageMask) != 0 || header.file_size > header.mem_size ||
      header.offset > file_size ||
      header.file_size > file_size - header.offset) {
    return ElfError::kBadSegment;
  }
  if (header.mem_size > kUserSpaceEnd - kUserSpaceBase ||
      !IsUserRange(header.vaddr, static_cast<int64_t>(header.mem_size))) {
    return ElfError::kNotInUserSpace;
  }
  const bool writable = (header.flags & kSegmentWritable) != 0;
  const bool executable = (header.flags & kSegmentExecutable) != 0;
  if (writable && executable) {
    return ElfError::kWritableAndExecutable;
  }
  ElfSegment out{};
  out.vaddr = header.vaddr;
  out.mem_size = static_cast<int64_t>(header.mem_size);
  out.file_offset = static_cast<int64_t>(header.offset);
  out.file_size = static_cast<int64_t>(header.file_size);
  out.access = writable     ? UserPageAccess::kReadWrite
               : executable ? UserPageAccess::kReadExecute
                            : UserPageAccess::kRead;
  return out;
}

}  // namespace

StatusOr<ElfExecutable, ElfError> ElfParseExecutable(const uint8_t* const data,
                                                     const int64_t size) {
  DCHECK(size >= 0);
  DCHECK(data != nullptr || size == 0);
  const uint64_t file_size = static_cast<uint64_t>(size);
  if (file_size < sizeof(Elf64Header)) {
    return ElfError::kTruncated;
  }
  const Elf64Header header = ReadAt<Elf64Header>(data, 0);
  for (int i = 0; i < 4; ++i) {
    if (header.ident[i] != kMagic[i]) {
      return ElfError::kNotElf64;
    }
  }
  if (header.ident[kIdentClass] != kClass64 ||
      header.ident[kIdentData] != kDataLittleEndian ||
      header.ident[kIdentVersion] != kVersionCurrent ||
      header.version != kVersionCurrent) {
    return ElfError::kNotElf64;
  }
  if (header.type != kTypeExecutable || header.machine != kMachineX86_64) {
    return ElfError::kNotExecutable;
  }
  if (header.program_header_size != sizeof(Elf64ProgramHeader)) {
    return ElfError::kBadProgramHeaders;
  }
  if (header.program_header_offset > file_size ||
      header.program_header_count > (file_size - header.program_header_offset) /
                                        sizeof(Elf64ProgramHeader)) {
    return ElfError::kTruncated;
  }

  ElfExecutable executable;
  executable.entry = header.entry;
  for (int i = 0; i < header.program_header_count; ++i) {
    const Elf64ProgramHeader program_header = ReadAt<Elf64ProgramHeader>(
        data, header.program_header_offset +
                  static_cast<uint64_t>(i) * sizeof(Elf64ProgramHeader));
    if (program_header.type == kSegmentDynamic ||
        program_header.type == kSegmentInterpreter) {
      return ElfError::kNotExecutable;
    }
    if (program_header.type != kSegmentLoad || program_header.mem_size == 0) {
      continue;
    }
    if (executable.segment_count == kElfMaxSegments) {
      return ElfError::kTooManySegments;
    }
    const StatusOr<ElfSegment, ElfError> segment =
        ParseSegment(program_header, file_size);
    if (!segment.ok()) {
      return segment.status();
    }
    for (int j = 0; j < executable.segment_count; ++j) {
      if (SharePage(*segment, executable.segments[j])) {
        return ElfError::kOverlappingSegments;
      }
    }
    executable.segments[executable.segment_count++] = *segment;
  }

  bool entry_in_code = false;
  for (int i = 0; i < executable.segment_count; ++i) {
    const ElfSegment& segment = executable.segments[i];
    entry_in_code =
        entry_in_code || (segment.access == UserPageAccess::kReadExecute &&
                          executable.entry >= segment.vaddr &&
                          executable.entry - segment.vaddr <
                              static_cast<uint64_t>(segment.mem_size));
  }
  if (!entry_in_code) {
    return ElfError::kBadEntry;
  }
  return executable;
}

}  // namespace protos
