#include "elf.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "paging.h"

namespace protos {
namespace {

namespace t = ::testing;

// The ELF64 file header and program header, as the test writes them.
struct TestElfHeader {
  uint8_t ident[16] = {0x7F, 'E', 'L', 'F', 2, 1, 1};
  uint16_t type = 2;      // ET_EXEC
  uint16_t machine = 62;  // EM_X86_64
  uint32_t version = 1;
  uint64_t entry = 0;
  uint64_t program_header_offset = 64;
  uint64_t section_header_offset = 0;
  uint32_t flags = 0;
  uint16_t header_size = 64;
  uint16_t program_header_size = 56;
  uint16_t program_header_count = 0;
  uint16_t section_header_size = 0;
  uint16_t section_header_count = 0;
  uint16_t section_name_index = 0;
};
static_assert(sizeof(TestElfHeader) == 64);

struct TestProgramHeader {
  uint32_t type = 0;
  uint32_t flags = 0;
  uint64_t offset = 0;
  uint64_t vaddr = 0;
  uint64_t paddr = 0;
  uint64_t file_size = 0;
  uint64_t mem_size = 0;
  uint64_t align = 0x1000;
};
static_assert(sizeof(TestProgramHeader) == 56);

constexpr uint32_t kLoad = 1;         // PT_LOAD
constexpr uint32_t kDynamic = 2;      // PT_DYNAMIC
constexpr uint32_t kInterpreter = 3;  // PT_INTERP
constexpr uint32_t kNote = 4;         // PT_NOTE
constexpr uint32_t kGnuStack = 0x6474E551;
constexpr uint32_t kExecute = 1;  // PF_X
constexpr uint32_t kWrite = 2;    // PF_W
constexpr uint32_t kRead = 4;     // PF_R

// Where the user programs link (user/user.ld).
constexpr uint64_t kText = 0xFFFFFFFF80000000ULL;
constexpr uint64_t kFileSize = 0x4000;

// Writes an ELF file of `kFileSize` bytes with `header` and the program
// headers `segments`, whose count it stores in the header. Program headers
// that would end beyond the file are left out. The rest of the file holds a
// byte pattern.
static std::vector<uint8_t> BuildElf(
    TestElfHeader header, const std::vector<TestProgramHeader>& segments) {
  header.program_header_count = static_cast<uint16_t>(segments.size());
  std::vector<uint8_t> file(kFileSize);
  for (uint64_t i = 0; i < kFileSize; ++i) {
    file[i] = static_cast<uint8_t>(i * 7);
  }
  std::memcpy(file.data(), &header, sizeof(header));
  constexpr uint64_t kEntrySize = sizeof(TestProgramHeader);
  for (size_t i = 0; i < segments.size(); ++i) {
    const uint64_t offset = header.program_header_offset + i * kEntrySize;
    if (offset >= header.program_header_offset && offset <= kFileSize &&
        kEntrySize <= kFileSize - offset) {
      std::memcpy(file.data() + offset, &segments[i], kEntrySize);
    }
  }
  return file;
}

static TestProgramHeader Segment(const uint32_t flags,      //
                                 const uint64_t offset,     //
                                 const uint64_t vaddr,      //
                                 const uint64_t file_size,  //
                                 const uint64_t mem_size) {
  TestProgramHeader segment;
  segment.type = kLoad;
  segment.flags = flags;
  segment.offset = offset;
  segment.vaddr = vaddr;
  segment.paddr = vaddr;
  segment.file_size = file_size;
  segment.mem_size = mem_size;
  return segment;
}

// A header whose entry point is in the code segment of `ValidSegments()`.
static TestElfHeader ValidHeader() {
  TestElfHeader header;
  header.entry = kText + 0x10;
  return header;
}

// Code, read-only data, and data followed by zero-initialized data, like the
// segments of a user program.
static std::vector<TestProgramHeader> ValidSegments() {
  return {
      Segment(kRead | kExecute, 0x1000, kText, 0x1234, 0x1234),
      Segment(kRead, 0x2234, kText + 0x2000, 0x100, 0x100),
      Segment(kRead | kWrite, 0x3000, kText + 0x3000, 0x10, 0x2000),
  };
}

static StatusOr<ElfExecutable, ElfError> Parse(
    const std::vector<uint8_t>& file) {
  const int64_t size = static_cast<int64_t>(file.size());
  return ElfParseExecutable(file.data(), size);
}

static ElfError ParseError(const std::vector<uint8_t>& file) {
  return Parse(file).status();
}

static auto IsSegment(const uintptr_t vaddr,      //
                      const int64_t mem_size,     //
                      const int64_t file_offset,  //
                      const int64_t file_size,    //
                      const UserPageAccess access) {
  return t::AllOf(t::Field(&ElfSegment::vaddr, vaddr),
                  t::Field(&ElfSegment::mem_size, mem_size),
                  t::Field(&ElfSegment::file_offset, file_offset),
                  t::Field(&ElfSegment::file_size, file_size),
                  t::Field(&ElfSegment::access, access));
}

TEST(ElfTest, ParsesTheSegmentsAndEntryOfAnExecutable) {
  const StatusOr<ElfExecutable, ElfError> executable =
      Parse(BuildElf(ValidHeader(), ValidSegments()));
  ASSERT_TRUE(executable.ok());
  EXPECT_THAT(executable->entry, t::Eq(kText + 0x10));
  ASSERT_THAT(executable->segment_count, t::Eq(3));
  EXPECT_THAT(executable->segments[0],
              IsSegment(kText,   //
                        0x1234,  //
                        0x1000,  //
                        0x1234,  //
                        UserPageAccess::kReadExecute));
  EXPECT_THAT(executable->segments[1],
              IsSegment(kText + 0x2000,  //
                        0x100,           //
                        0x2234,          //
                        0x100,           //
                        UserPageAccess::kRead));
  EXPECT_THAT(executable->segments[2],
              IsSegment(kText + 0x3000,  //
                        0x2000,          //
                        0x3000,          //
                        0x10,            //
                        UserPageAccess::kReadWrite));
}

TEST(ElfTest, LeavesOutEmptyAndNonLoadableSegments) {
  std::vector<TestProgramHeader> segments = ValidSegments();
  TestProgramHeader note;
  note.type = kNote;
  note.offset = 0x200;
  note.file_size = 0x20;
  TestProgramHeader stack;
  stack.type = kGnuStack;
  stack.flags = kRead | kWrite;
  segments.insert(segments.begin(), note);
  segments.push_back(stack);
  // An empty loadable segment, even a writable and executable one outside
  // user space, takes no memory.
  segments.push_back(Segment(kRead | kWrite | kExecute, 0, 0x1000, 0, 0));
  const StatusOr<ElfExecutable, ElfError> executable =
      Parse(BuildElf(ValidHeader(), segments));
  ASSERT_TRUE(executable.ok());
  ASSERT_THAT(executable->segment_count, t::Eq(3));
  EXPECT_THAT(executable->segments[0].vaddr, t::Eq(kText));
  EXPECT_THAT(executable->segments[2].vaddr, t::Eq(kText + 0x3000));
}

TEST(ElfTest, AcceptsAnUnalignedFile) {
  const std::vector<uint8_t> file = BuildElf(ValidHeader(), ValidSegments());
  std::vector<uint8_t> buffer(file.size() + 1);
  std::memcpy(buffer.data() + 1, file.data(), file.size());
  const int64_t size = static_cast<int64_t>(file.size());
  const StatusOr<ElfExecutable, ElfError> executable =
      ElfParseExecutable(buffer.data() + 1, size);
  ASSERT_TRUE(executable.ok());
  EXPECT_THAT(executable->segment_count, t::Eq(3));
}

TEST(ElfTest, RejectsTruncatedFiles) {
  const std::vector<uint8_t> file = BuildElf(ValidHeader(), ValidSegments());
  EXPECT_THAT(ElfParseExecutable(nullptr, 0).status(),
              t::Eq(ElfError::kTruncated));
  EXPECT_THAT(ElfParseExecutable(file.data(), 63).status(),
              t::Eq(ElfError::kTruncated));
  // The third program header ends at byte 64 + 3 * 56 = 232.
  EXPECT_THAT(ElfParseExecutable(file.data(), 231).status(),
              t::Eq(ElfError::kTruncated));

  TestElfHeader header = ValidHeader();
  header.program_header_offset = kFileSize + 8;
  EXPECT_THAT(ParseError(BuildElf(header, ValidSegments())),
              t::Eq(ElfError::kTruncated));
  header.program_header_offset = UINT64_MAX - 8;
  EXPECT_THAT(ParseError(BuildElf(header, ValidSegments())),
              t::Eq(ElfError::kTruncated));
}

TEST(ElfTest, RejectsFilesThatAreNotLittleEndianElf64) {
  const auto with_ident_byte = [](const int index, const uint8_t value) {
    TestElfHeader header = ValidHeader();
    header.ident[index] = value;
    return BuildElf(header, ValidSegments());
  };
  EXPECT_THAT(ParseError(with_ident_byte(0, 0x7E)), t::Eq(ElfError::kNotElf64));
  EXPECT_THAT(ParseError(with_ident_byte(3, 'G')), t::Eq(ElfError::kNotElf64));
  // ELFCLASS32, big-endian, and an unknown version.
  EXPECT_THAT(ParseError(with_ident_byte(4, 1)), t::Eq(ElfError::kNotElf64));
  EXPECT_THAT(ParseError(with_ident_byte(5, 2)), t::Eq(ElfError::kNotElf64));
  EXPECT_THAT(ParseError(with_ident_byte(6, 2)), t::Eq(ElfError::kNotElf64));
  TestElfHeader header = ValidHeader();
  header.version = 2;
  EXPECT_THAT(ParseError(BuildElf(header, ValidSegments())),
              t::Eq(ElfError::kNotElf64));
}

TEST(ElfTest, RejectsFilesThatAreNotStaticX86_64Executables) {
  TestElfHeader shared_object = ValidHeader();
  shared_object.type = 3;  // ET_DYN
  EXPECT_THAT(ParseError(BuildElf(shared_object, ValidSegments())),
              t::Eq(ElfError::kNotExecutable));
  TestElfHeader i386 = ValidHeader();
  i386.machine = 3;  // EM_386
  EXPECT_THAT(ParseError(BuildElf(i386, ValidSegments())),
              t::Eq(ElfError::kNotExecutable));

  for (const uint32_t type : {kInterpreter, kDynamic}) {
    std::vector<TestProgramHeader> segments = ValidSegments();
    TestProgramHeader dynamic;
    dynamic.type = type;
    dynamic.offset = 0x200;
    dynamic.file_size = 0x10;
    segments.push_back(dynamic);
    EXPECT_THAT(ParseError(BuildElf(ValidHeader(), segments)),
                t::Eq(ElfError::kNotExecutable));
  }
}

TEST(ElfTest, RejectsProgramHeadersOfTheWrongSize) {
  TestElfHeader header = ValidHeader();
  header.program_header_size = 32;
  EXPECT_THAT(ParseError(BuildElf(header, ValidSegments())),
              t::Eq(ElfError::kBadProgramHeaders));
}

TEST(ElfTest, AcceptsUpToTheMaximumNumberOfSegments) {
  std::vector<TestProgramHeader> segments;
  for (int i = 0; i < kElfMaxSegments; ++i) {
    const uint64_t vaddr = kText + static_cast<uint64_t>(i) * 0x1000;
    segments.push_back(Segment(kRead | kExecute, 0x1000, vaddr, 0x100, 0x1000));
  }
  const StatusOr<ElfExecutable, ElfError> executable =
      Parse(BuildElf(ValidHeader(), segments));
  ASSERT_TRUE(executable.ok());
  EXPECT_THAT(executable->segment_count, t::Eq(kElfMaxSegments));

  segments.push_back(Segment(kRead, 0x1000, kText + 0x100000, 0x100, 0x100));
  EXPECT_THAT(ParseError(BuildElf(ValidHeader(), segments)),
              t::Eq(ElfError::kTooManySegments));
}

TEST(ElfTest, RejectsSegmentsWithBadAddressesOrContents) {
  const auto with_data_segment = [](const uint64_t offset,     //
                                    const uint64_t vaddr,      //
                                    const uint64_t file_size,  //
                                    const uint64_t mem_size) {
    std::vector<TestProgramHeader> segments = ValidSegments();
    segments[2] = Segment(kRead | kWrite, offset, vaddr, file_size, mem_size);
    return ParseError(BuildElf(ValidHeader(), segments));
  };
  constexpr uint64_t kData = kText + 0x3000;
  EXPECT_THAT(with_data_segment(0x3000, kData, 0x10, 0x10),
              t::Eq(ElfError::kNone));
  // Contents up to the end of the file.
  EXPECT_THAT(with_data_segment(0x3000, kData, 0x1000, 0x1000),
              t::Eq(ElfError::kNone));
  EXPECT_THAT(with_data_segment(0x3000, kData + 8, 0x10, 0x10),
              t::Eq(ElfError::kBadSegment));
  EXPECT_THAT(with_data_segment(0x3000, kData, 0x20, 0x10),
              t::Eq(ElfError::kBadSegment));
  EXPECT_THAT(with_data_segment(0x3000, kData, 0x1001, 0x2000),
              t::Eq(ElfError::kBadSegment));
  EXPECT_THAT(with_data_segment(kFileSize + 1, kData, 0, 0x10),
              t::Eq(ElfError::kBadSegment));
  EXPECT_THAT(with_data_segment(UINT64_MAX - 8, kData, 0x10, 0x10),
              t::Eq(ElfError::kBadSegment));
}

TEST(ElfTest, RejectsSegmentsOutsideUserSpace) {
  const auto with_data_segment = [](const uint64_t vaddr,
                                    const uint64_t mem_size) {
    std::vector<TestProgramHeader> segments = ValidSegments();
    segments[2] = Segment(kRead | kWrite, 0x3000, vaddr, 0x10, mem_size);
    return ParseError(BuildElf(ValidHeader(), segments));
  };
  EXPECT_THAT(with_data_segment(kUserSpaceBase, 0x1000),
              t::Eq(ElfError::kNone));
  EXPECT_THAT(with_data_segment(kUserSpaceEnd - 0x1000, 0x1000),
              t::Eq(ElfError::kNone));
  // The kernel half, the top page of the address space, a range that would
  // wrap around, and one that starts below user space.
  EXPECT_THAT(with_data_segment(0x400000, 0x1000),
              t::Eq(ElfError::kNotInUserSpace));
  EXPECT_THAT(with_data_segment(kUserSpaceEnd, 0x1000),
              t::Eq(ElfError::kNotInUserSpace));
  EXPECT_THAT(with_data_segment(kUserSpaceEnd - 0x1000, 0x1001),
              t::Eq(ElfError::kNotInUserSpace));
  EXPECT_THAT(with_data_segment(kText, UINT64_MAX - 0x1000),
              t::Eq(ElfError::kNotInUserSpace));
  EXPECT_THAT(with_data_segment(kUserSpaceBase - 0x1000, 0x2000),
              t::Eq(ElfError::kNotInUserSpace));
}

TEST(ElfTest, RejectsWritableAndExecutableSegments) {
  std::vector<TestProgramHeader> segments = ValidSegments();
  segments[2].flags = kRead | kWrite | kExecute;
  EXPECT_THAT(ParseError(BuildElf(ValidHeader(), segments)),
              t::Eq(ElfError::kWritableAndExecutable));
}

TEST(ElfTest, RejectsSegmentsThatSharePages) {
  // The code segment ends at kText + 0x1234, in the page at kText + 0x1000.
  const auto with_data_at = [](const uint64_t vaddr) {
    std::vector<TestProgramHeader> segments = ValidSegments();
    segments[2] = Segment(kRead | kWrite, 0x3000, vaddr, 0x10, 0x10);
    return ParseError(BuildElf(ValidHeader(), segments));
  };
  EXPECT_THAT(with_data_at(kText + 0x1000),
              t::Eq(ElfError::kOverlappingSegments));
  EXPECT_THAT(with_data_at(kText), t::Eq(ElfError::kOverlappingSegments));
  EXPECT_THAT(with_data_at(kText - 0x1000), t::Eq(ElfError::kNone));

  // A segment that covers another one from below.
  std::vector<TestProgramHeader> segments = ValidSegments();
  segments[2] = Segment(kRead | kWrite, 0x3000, kText - 0x1000, 0x10, 0x10000);
  EXPECT_THAT(ParseError(BuildElf(ValidHeader(), segments)),
              t::Eq(ElfError::kOverlappingSegments));
}

TEST(ElfTest, RejectsEntryPointsOutsideTheCode) {
  const auto with_entry = [](const uint64_t entry) {
    TestElfHeader header = ValidHeader();
    header.entry = entry;
    return ParseError(BuildElf(header, ValidSegments()));
  };
  EXPECT_THAT(with_entry(kText), t::Eq(ElfError::kNone));
  EXPECT_THAT(with_entry(kText + 0x1233), t::Eq(ElfError::kNone));
  EXPECT_THAT(with_entry(kText + 0x1234), t::Eq(ElfError::kBadEntry));
  EXPECT_THAT(with_entry(kText - 1), t::Eq(ElfError::kBadEntry));
  // In the read-only data and the data segment.
  EXPECT_THAT(with_entry(kText + 0x2000), t::Eq(ElfError::kBadEntry));
  EXPECT_THAT(with_entry(kText + 0x3000), t::Eq(ElfError::kBadEntry));
  EXPECT_THAT(with_entry(0), t::Eq(ElfError::kBadEntry));

  // Without loadable segments, nothing can run.
  EXPECT_THAT(ParseError(BuildElf(ValidHeader(), {})),
              t::Eq(ElfError::kBadEntry));
}

TEST(ElfTest, ChecksItsArguments) {
  const std::vector<uint8_t> file = BuildElf(ValidHeader(), ValidSegments());
  EXPECT_DEATH((void)ElfParseExecutable(file.data(), -1),
               "Check failed: size >= 0");
  EXPECT_DEATH((void)ElfParseExecutable(nullptr, 64),
               "Check failed: data != nullptr");
}

}  // namespace
}  // namespace protos
