#include "user/ramfs/tar.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "syscall_abi.h"

namespace protos::tar {
namespace {

namespace t = ::testing;

constexpr int64_t kInt64Max = std::numeric_limits<int64_t>::max();

// The offsets of the ustar header fields that the tests write.
constexpr int kNameOffset = 0;
constexpr int kModeOffset = 100;
constexpr int kUidOffset = 108;
constexpr int kGidOffset = 116;
constexpr int kSizeOffset = 124;
constexpr int kMtimeOffset = 136;
constexpr int kChecksumOffset = 148;
constexpr int kTypeflagOffset = 156;
constexpr int kLinknameOffset = 157;
constexpr int kMagicOffset = 257;
constexpr int kPrefixOffset = 345;

using Block = std::array<uint8_t, kBlockSize>;

// `size` bytes that depend on `seed`, with NULs and bytes >= 0x80 among them.
static std::string Pattern(const int64_t size, const int seed) {
  std::string data(size, '\0');
  for (int64_t i = 0; i < size; ++i) {
    data[i] = static_cast<char>((i * 7 + int64_t{seed} * 31) % 251);
  }
  return data;
}

static int64_t RoundUpToBlock(const int64_t size) {
  return (size + kBlockSize - 1) / kBlockSize * kBlockSize;
}

// `value` in octal, with zeros before it up to `digits` digits.
static std::string Octal(const int64_t value, const int digits = 1) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "%0*llo",     //
                digits,       //
                static_cast<unsigned long long>(value));
  return buf;
}

// The magic and version fields: "ustar\0" and "00" for POSIX ustar, or
// "ustar  \0" for the old GNU format.
static std::string Magic(const bool gnu) {
  return gnu ? std::string("ustar  \0", 8) : std::string("ustar\0", 6) + "00";
}

// A PAX extended header record, whose length counts its own digits.
static std::string PaxRecord(const std::string& key, const std::string& value) {
  const std::string body = " " + key + "=" + value + "\n";
  for (int digits = 1;; ++digits) {
    const std::string length =
        std::to_string(digits + static_cast<int>(body.size()));
    if (static_cast<int>(length.size()) == digits) {
      return length + body;
    }
  }
}

// Copies `bytes`, which must fit into the `length`-byte field at `offset`, to
// `block`.
static void PutField(Block& block,      //
                     const int offset,  //
                     const int length,  //
                     const std::string& bytes) {
  ASSERT_LE(static_cast<int>(bytes.size()), length);
  std::copy(bytes.begin(), bytes.end(), block.begin() + offset);
}

// The sum of the bytes of `block` as unsigned or signed chars, with the
// checksum field counted as spaces.
static int64_t HeaderSum(const Block& block, const bool as_signed) {
  int64_t sum = 0;
  for (int i = 0; i < kBlockSize; ++i) {
    const uint8_t byte = (i >= kChecksumOffset && i < kChecksumOffset + 8)
                             ? uint8_t{' '}
                             : block[i];
    sum += as_signed ? int64_t{static_cast<int8_t>(byte)} : int64_t{byte};
  }
  return sum;
}

// Stores the checksum of `block` as GNU tar does: six octal digits, a NUL and
// a space.
static void SetChecksum(Block& block, const bool as_signed = false) {
  const int64_t sum = HeaderSum(block, as_signed);
  PutField(block, kChecksumOffset, 8, Octal(sum, 6) + std::string("\0 ", 2));
}

// The fields of a member header for MakeHeader(). Strings are stored without
// a NUL, and the rest of their fields stays zero.
struct HeaderSpec {
  std::string name = "";
  char typeflag = '0';
  int64_t size = 0;
  std::string linkname = "";
  std::string prefix = "";
  // The magic of the old GNU format instead of the POSIX one.
  bool gnu = false;
};

// A header with the fields of `spec` and a checksum like GNU tar's.
static Block MakeHeader(const HeaderSpec& spec) {
  Block block{};
  PutField(block, kNameOffset, 100, spec.name);
  PutField(block, kModeOffset, 8, Octal(spec.typeflag == '5' ? 0755 : 0644, 7));
  PutField(block, kUidOffset, 8, Octal(0, 7));
  PutField(block, kGidOffset, 8, Octal(0, 7));
  PutField(block, kSizeOffset, 12, Octal(spec.size, 11));
  PutField(block, kMtimeOffset, 12, Octal(0, 11));
  block[kTypeflagOffset] = static_cast<uint8_t>(spec.typeflag);
  PutField(block, kLinknameOffset, 100, spec.linkname);
  PutField(block, kMagicOffset, 8, Magic(spec.gnu));
  PutField(block, kPrefixOffset, 155, spec.prefix);
  SetChecksum(block);
  return block;
}

// Builds an archive in memory.
class TarBuilder {
 public:
  // Appends `block`.
  TarBuilder& Add(const Block& block) {
    bytes_.insert(bytes_.end(), block.begin(), block.end());
    return *this;
  }

  // Appends `header` and `data`, padded with zeros to whole blocks.
  TarBuilder& Add(const Block& header, const std::string& data) {
    Add(header);
    Raw(data);
    PadTo(RoundUpToBlock(size()));
    return *this;
  }

  // Appends a member whose header `spec` describes, with `data` and its size.
  TarBuilder& Member(HeaderSpec spec, const std::string& data) {
    spec.size = static_cast<int64_t>(data.size());
    return Add(MakeHeader(spec), data);
  }

  TarBuilder& File(const std::string& name, const std::string& data) {
    return Member({.name = name}, data);
  }

  TarBuilder& Dir(const std::string& name) {
    return Member({.name = name, .typeflag = '5'}, "");
  }

  // Appends `bytes` as they are.
  TarBuilder& Raw(const std::string& bytes) {
    bytes_.insert(bytes_.end(), bytes.begin(), bytes.end());
    return *this;
  }

  TarBuilder& ZeroBlocks(const int count) {
    return PadTo(size() + count * kBlockSize);
  }

  // Appends zeros up to `total` bytes, as tar pads an archive to whole
  // records.
  TarBuilder& PadTo(const int64_t total) {
    bytes_.resize(std::max(total, size()));
    return *this;
  }

  const std::vector<uint8_t>& bytes() const { return bytes_; }
  int64_t size() const { return static_cast<int64_t>(bytes_.size()); }

 private:
  std::vector<uint8_t> bytes_;
};

// An archive in memory, which can return fewer bytes than asked for, and
// fail.
class MemorySource final : public Source {
 public:
  explicit MemorySource(std::vector<uint8_t> bytes)
      : bytes_(std::move(bytes)) {}

  int64_t Read(const int64_t offset,  //
               void* const buffer,    //
               const int64_t size) override {
    ++reads_;
    EXPECT_GE(offset, 0);
    EXPECT_GT(size, 0);
    EXPECT_NE(buffer, nullptr);
    if (offset >= fail_begin_ && offset < fail_end_) {
      return fail_error_;
    }
    const int64_t total = static_cast<int64_t>(bytes_.size());
    if (offset >= total) {
      return 0;
    }
    int64_t end = offset + std::min({size, max_read_, total - offset});
    if (offset < fail_begin_) {
      end = std::min(end, fail_begin_);
    }
    const int64_t count = end - offset;
    std::memcpy(buffer, bytes_.data() + offset, count);
    if (offset >= extra_after_) {
      return count + extra_;
    }
    return count;
  }

  // Makes every Read() return at most `max_read` bytes.
  void set_max_read(const int64_t max_read) { max_read_ = max_read; }

  // Makes Read() at offsets >= `after` return `extra` more than the number of
  // bytes that it copied, as a broken source would.
  void set_extra(const int64_t extra, const int64_t after = 0) {
    extra_ = extra;
    extra_after_ = after;
  }

  // Makes the reads that start in `[begin, end)` fail with `error`, and those
  // before it stop at `begin`.
  void Fail(const int64_t begin, const int64_t end, const int64_t error) {
    fail_begin_ = begin;
    fail_end_ = end;
    fail_error_ = error;
  }

  // The number of Read() calls.
  int reads() const { return reads_; }

 private:
  std::vector<uint8_t> bytes_;
  int64_t max_read_ = kInt64Max;
  int64_t extra_ = 0;
  int64_t extra_after_ = 0;
  int64_t fail_begin_ = kInt64Max;
  int64_t fail_end_ = kInt64Max;
  int64_t fail_error_ = 0;
  int reads_ = 0;
};

// A member as the tests compare it. `data` holds the data of a regular file,
// which ReadAll() reads with ReadData().
struct Member {
  EntryType type = EntryType::kSkipped;
  std::string path = "";
  int64_t size = 0;
  std::string data = "";

  friend bool operator==(const Member&, const Member&) = default;

  friend void PrintTo(const Member& member, std::ostream* const os) {
    switch (member.type) {
      case EntryType::kRegular:
        *os << "regular";
        break;
      case EntryType::kDirectory:
        *os << "directory";
        break;
      case EntryType::kSkipped:
        *os << "skipped";
        break;
    }
    *os << " \"" << member.path << "\" of " << member.size << " bytes";
    if (!member.data.empty()) {
      *os << " with data \"" << member.data.substr(0, 40) << "\"";
    }
  }
};

static Member Regular(const std::string& path, const std::string& data) {
  return Member{
      .type = EntryType::kRegular,
      .path = path,
      .size = static_cast<int64_t>(data.size()),
      .data = data,
  };
}

static Member Directory(const std::string& path, const int64_t size = 0) {
  return Member{.type = EntryType::kDirectory, .path = path, .size = size};
}

static Member Skipped(const int64_t size = 0) {
  return Member{.type = EntryType::kSkipped, .size = size};
}

static std::string PathOf(const Entry& entry) {
  return std::string(entry.path, entry.path_len);
}

// Checks what holds for every entry that Next() returns.
static void ExpectValidEntry(const Entry& entry, const int64_t header_offset) {
  if (entry.type == EntryType::kSkipped) {
    EXPECT_GE(entry.data_offset, header_offset + kBlockSize);
    EXPECT_EQ((entry.data_offset - header_offset) % kBlockSize, 0);
  } else {
    EXPECT_EQ(entry.data_offset, header_offset + kBlockSize);
  }
  EXPECT_GE(entry.size, 0);
  ASSERT_GE(entry.path_len, 0);
  ASSERT_LE(entry.path_len, kFsMaxPathLength);
  for (int64_t i = entry.path_len; i <= kFsMaxPathLength; ++i) {
    ASSERT_EQ(entry.path[i], '\0') << "at " << i;
  }
  if (entry.type == EntryType::kSkipped) {
    EXPECT_EQ(entry.path_len, 0);
  } else {
    ASSERT_GT(entry.path_len, 0);
    EXPECT_EQ(entry.path[0], '/');
    EXPECT_EQ(static_cast<int64_t>(std::strlen(entry.path)), entry.path_len);
  }
}

// Reads the data of `entry` with ReadData() calls of up to `chunk` bytes.
static std::string ReadEntryData(Reader& reader,      //
                                 const Entry& entry,  //
                                 const int64_t chunk) {
  std::string data;
  std::vector<char> buffer(chunk);
  while (static_cast<int64_t>(data.size()) < entry.size) {
    const int64_t pos = static_cast<int64_t>(data.size());
    const int64_t n = reader.ReadData(entry, pos, buffer.data(), chunk);
    EXPECT_EQ(n, std::min(chunk, entry.size - pos));
    if (n <= 0) {
      break;
    }
    data.append(buffer.data(), n);
  }
  return data;
}

// Reads the rest of the archive with Next(), and the data of its regular files
// with ReadData() calls of up to `chunk` bytes, and returns its members. Stores
// the result of the Next() call that ended it in `*end`.
static std::vector<Member> ReadAll(Reader& reader,      //
                                   int64_t* const end,  //
                                   const int64_t chunk = 100) {
  std::vector<Member> members;
  // Next() overwrites all of a reused entry.
  Entry entry;
  for (;;) {
    const int64_t rc = reader.Next(&entry);
    if (rc != 1) {
      *end = rc;
      return members;
    }
    ExpectValidEntry(entry, reader.header_offset());
    Member member{
        .type = entry.type,
        .path = PathOf(entry),
        .size = entry.size,
    };
    if (entry.type == EntryType::kRegular) {
      member.data = ReadEntryData(reader, entry, chunk);
    }
    members.push_back(member);
  }
}

static std::vector<Member> ReadArchive(const std::vector<uint8_t>& archive,
                                       int64_t* const end) {
  MemorySource source(archive);
  Reader reader(&source);
  return ReadAll(reader, end);
}

// Returns an archive of the header `block` (whose size field must be 0),
// followed by a regular file "after" and the end.
static std::vector<uint8_t> ArchiveWithHeader(const Block& block) {
  TarBuilder builder;
  builder.Add(block).File("after", "data").ZeroBlocks(2);
  return builder.bytes();
}

// Expects Next() to reject the first header of `archive` with
// `kSyscallErrorInvalid`, also when called again, without reading again.
static void ExpectInvalidHeader(const std::vector<uint8_t>& archive) {
  MemorySource source(archive);
  Reader reader(&source);
  Entry entry;
  EXPECT_EQ(reader.Next(&entry), kSyscallErrorInvalid);
  EXPECT_EQ(reader.header_offset(), 0);
  const int reads = source.reads();
  EXPECT_EQ(reader.Next(&entry), kSyscallErrorInvalid);
  EXPECT_EQ(source.reads(), reads);
}

TEST(TarReaderTest, ReadsRegularFilesOfAnySizeWithTheirOffsetsAndData) {
  const std::vector<int64_t> sizes = {0, 1, 511, 512, 513, 5 * kBlockSize + 77};
  const int count = static_cast<int>(sizes.size());
  TarBuilder builder;
  for (int i = 0; i < count; ++i) {
    builder.File("file" + std::to_string(i), Pattern(sizes[i], i));
  }
  builder.ZeroBlocks(2);

  MemorySource source(builder.bytes());
  Reader reader(&source);
  EXPECT_EQ(reader.header_offset(), 0);
  int64_t header_offset = 0;
  for (int i = 0; i < count; ++i) {
    SCOPED_TRACE(i);
    Entry entry;
    ASSERT_EQ(reader.Next(&entry), 1);
    EXPECT_EQ(reader.header_offset(), header_offset);
    ExpectValidEntry(entry, header_offset);
    EXPECT_EQ(entry.type, EntryType::kRegular);
    EXPECT_EQ(PathOf(entry), "/file" + std::to_string(i));
    EXPECT_EQ(entry.size, sizes[i]);
    EXPECT_EQ(entry.data_offset, header_offset + kBlockSize);
    EXPECT_EQ(ReadEntryData(reader, entry, 100), Pattern(sizes[i], i));
    header_offset += kBlockSize + RoundUpToBlock(sizes[i]);
  }
  Entry entry;
  EXPECT_EQ(reader.Next(&entry), 0);
  EXPECT_EQ(reader.header_offset(), header_offset);
  EXPECT_EQ(header_offset + 2 * kBlockSize, builder.size());
}

TEST(TarReaderTest, ReadsAnArchiveLikeTheRootfsArchiveOfTheBuild) {
  // What `tar --format=ustar --sort=name ... -C rootfs -cf rootfs.tar .`
  // writes, padded to whole 10240-byte records.
  const std::string readme = "Welcome to protos.\n";
  const std::string shell = Pattern(1500, 1);
  TarBuilder builder;
  builder.Dir("./")
      .File("./README.txt", readme)
      .Dir("./doc/")
      .File("./doc/shell.txt", shell)
      .ZeroBlocks(2)
      .PadTo(10240);
  int64_t end = 1;
  EXPECT_THAT(ReadArchive(builder.bytes(), &end),
              t::ElementsAre(Directory("/"),                  //
                             Regular("/README.txt", readme),  //
                             Directory("/doc"),               //
                             Regular("/doc/shell.txt", shell)));
  EXPECT_EQ(end, 0);
}

TEST(TarReaderTest, ReadsDirectoriesTheRootAndOldStyleDirectories) {
  const std::string dir_data = "directory data is skipped";
  // Old tars wrote directories as regular files whose names end with '/', and
  // "." and "" name regular files at the root.
  TarBuilder builder;
  builder.Dir("./")
      .Dir("/")
      .Dir("doc/")
      .Dir("doc/sub")
      .Member({.name = "data/", .typeflag = '5'}, dir_data)
      .Member({.name = "old/", .typeflag = '0'}, "")
      .Member({.name = "older/", .typeflag = '\0'}, "")
      .Member({.name = "contiguous/", .typeflag = '7'}, "")
      .Member({.name = "./", .typeflag = '0'}, "")
      .Member({.name = ".", .typeflag = '0'}, "")
      .Member({.name = "", .typeflag = '0'}, "x")
      .File("after", "x")
      .ZeroBlocks(2);
  const int64_t dir_size = static_cast<int64_t>(dir_data.size());
  int64_t end = 1;
  EXPECT_THAT(ReadArchive(builder.bytes(), &end),
              t::ElementsAre(Directory("/"),                //
                             Directory("/"),                //
                             Directory("/doc"),             //
                             Directory("/doc/sub"),         //
                             Directory("/data", dir_size),  //
                             Directory("/old"),             //
                             Directory("/older"),           //
                             Directory("/contiguous"),      //
                             Directory("/"),                //
                             Skipped(),                     //
                             Skipped(1),                    //
                             Regular("/after", "x")));
  EXPECT_EQ(end, 0);
}

TEST(TarReaderTest, NormalizesPaths) {
  // Fills the name field: no NUL ends it.
  const std::string name100 = std::string(50, 'n') + "/" + std::string(49, 'm');
  ASSERT_EQ(static_cast<int>(name100.size()), 100);
  TarBuilder builder;
  builder.File("./a/b", "1")
      .File("a//b/./c", "2")
      .File("/abs//path", "3")
      .File("./././x/.", "4")
      .File(".../..a/b..", "5")
      .File("with space/caf\xC3\xA9", "6")
      .File(name100, "7")
      .Dir("dir//sub/./")
      .ZeroBlocks(2);
  int64_t end = 1;
  EXPECT_THAT(ReadArchive(builder.bytes(), &end),
              t::ElementsAre(Regular("/a/b", "1"),                     //
                             Regular("/a/b/c", "2"),                   //
                             Regular("/abs/path", "3"),                //
                             Regular("/x", "4"),                       //
                             Regular("/.../..a/b..", "5"),             //
                             Regular("/with space/caf\xC3\xA9", "6"),  //
                             Regular("/" + name100, "7"),              //
                             Directory("/dir/sub")));
  EXPECT_EQ(end, 0);
}

TEST(TarReaderTest, JoinsThePosixPrefixAndNameAndIgnoresTheGnuFieldsThere) {
  // Fills the prefix field: no NUL ends it.
  const std::string prefix155 = std::string(63, 'a') + "/" +
                                std::string(63, 'b') + "/" +
                                std::string(27, 'c');
  ASSERT_EQ(static_cast<int>(prefix155.size()), 155);
  // The old GNU format keeps the access and change times there.
  const std::string gnu_times =
      std::string("14567012345\0", 12) + "14567012345";
  TarBuilder builder;
  builder.Member({.name = "doc/readme", .prefix = "usr/share"}, "1")
      .Member({.name = "file", .prefix = prefix155}, "2")
      .Member({.name = "/name", .prefix = "./pre/"}, "3")
      .Member({.name = "sub/", .prefix = "top"}, "")
      .Member({.name = "gnu", .prefix = gnu_times, .gnu = true}, "4")
      .Member({.name = "gnu/", .typeflag = '5', .prefix = "x", .gnu = true}, "")
      .ZeroBlocks(2);
  int64_t end = 1;
  EXPECT_THAT(ReadArchive(builder.bytes(), &end),
              t::ElementsAre(Regular("/usr/share/doc/readme", "1"),    //
                             Regular("/" + prefix155 + "/file", "2"),  //
                             Regular("/pre/name", "3"),                //
                             Directory("/top/sub"),                    //
                             Regular("/gnu", "4"),                     //
                             Directory("/gnu")));
  EXPECT_EQ(end, 0);
}

TEST(TarReaderTest, AcceptsUnsignedAndSignedChecksumsInAnyOctalFormat) {
  // Bytes >= 0x80 in the name make the signed sum differ from the unsigned
  // one.
  const Block header = MakeHeader({.name = "caf\xC3\xA9", .size = 1});
  const int64_t sum = HeaderSum(header, /*as_signed=*/false);
  const int64_t signed_sum = HeaderSum(header, /*as_signed=*/true);
  ASSERT_NE(sum, signed_sum);
  ASSERT_GT(signed_sum, 0);
  const std::string digits = Octal(sum);
  const int digits_len = static_cast<int>(digits.size());
  ASSERT_LE(digits_len, 6);
  const std::vector<std::string> fields = {
      // GNU tar, bsdtar and Python.
      Octal(sum, 6) + std::string("\0 ", 2),
      Octal(signed_sum, 6) + std::string("\0 ", 2),
      Octal(sum, 7) + std::string(1, '\0'),
      Octal(sum, 7) + " ",
      // No terminator.
      Octal(sum, 8),
      std::string(7 - digits_len, ' ') + digits + " ",
      std::string(7 - digits_len, '\0') + digits + std::string(1, '\0'),
      " " + Octal(sum, 6) + " ",
      digits + std::string(8 - digits_len, '\0'),
  };
  for (const std::string& field : fields) {
    SCOPED_TRACE(t::PrintToString(field));
    Block block = header;
    PutField(block, kChecksumOffset, 8, field);
    TarBuilder builder;
    builder.Add(block, "x").ZeroBlocks(2);
    int64_t end = 1;
    EXPECT_THAT(ReadArchive(builder.bytes(), &end),
                t::ElementsAre(Regular("/caf\xC3\xA9", "x")));
    EXPECT_EQ(end, 0);
  }
}

TEST(TarReaderTest, RejectsBadChecksums) {
  const Block header = MakeHeader({.name = "f"});
  const int64_t sum = HeaderSum(header, /*as_signed=*/false);
  std::string not_octal = Octal(sum, 6);
  not_octal[5] = '9';
  const std::vector<std::string> fields = {
      Octal(sum + 1, 6) + std::string("\0 ", 2),
      Octal(sum - 1, 6) + std::string("\0 ", 2),
      Octal(sum + 512, 6) + std::string("\0 ", 2),
      not_octal + std::string("\0 ", 2),
      Octal(sum, 6) + "x ",
      "-" + Octal(sum, 6) + std::string(1, '\0'),
      // No digits.
      std::string(8, ' '),
      std::string(8, '\0'),
  };
  for (const std::string& field : fields) {
    SCOPED_TRACE(t::PrintToString(field));
    Block block = header;
    PutField(block, kChecksumOffset, 8, field);
    ExpectInvalidHeader(ArchiveWithHeader(block));
  }

  // A header that changed after its checksum was computed.
  Block changed = header;
  changed[kNameOffset] = 'g';
  ExpectInvalidHeader(ArchiveWithHeader(changed));
}

TEST(TarReaderTest, RejectsHeadersWithoutTheUstarMagic) {
  const std::vector<std::string> magics = {
      // Unix V7 tar wrote none.
      std::string(8, '\0'),
      std::string("USTAR\0", 6) + "00",
      std::string("ustax\0", 6) + "00",
      std::string(" ustar\0", 7) + "0",
      std::string("ustr\0\0", 6) + "00",
  };
  for (const std::string& magic : magics) {
    SCOPED_TRACE(t::PrintToString(magic));
    Block block = MakeHeader({.name = "f"});
    PutField(block, kMagicOffset, 8, magic);
    SetChecksum(block);
    ExpectInvalidHeader(ArchiveWithHeader(block));
  }
}

TEST(TarReaderTest, ParsesSizesInAnyOctalFormat) {
  const std::vector<std::string> fields = {
      "00000000012" + std::string(1, '\0'),
      "         12 ",
      // No terminator.
      "000000000012",
      "12" + std::string(10, '\0'),
      "12" + std::string(10, ' '),
      "      12" + std::string("\0 \0 ", 4),
  };
  for (const std::string& field : fields) {
    SCOPED_TRACE(t::PrintToString(field));
    Block block = MakeHeader({.name = "f"});
    PutField(block, kSizeOffset, 12, field);
    SetChecksum(block);
    TarBuilder builder;
    builder.Add(block, "0123456789").File("after", "x").ZeroBlocks(2);
    int64_t end = 1;
    EXPECT_THAT(
        ReadArchive(builder.bytes(), &end),
        t::ElementsAre(Regular("/f", "0123456789"), Regular("/after", "x")));
    EXPECT_EQ(end, 0);
  }
}

TEST(TarReaderTest, ReadsTheLargestSizes) {
  const std::vector<std::pair<std::string, int64_t>> fields = {
      {"77777777777" + std::string(1, '\0'), (int64_t{1} << 33) - 1},
      {"777777777777", (int64_t{1} << 36) - 1},
  };
  for (const auto& [field, size] : fields) {
    SCOPED_TRACE(size);
    Block block = MakeHeader({.name = "big"});
    PutField(block, kSizeOffset, 12, field);
    SetChecksum(block);
    TarBuilder builder;
    builder.Add(block).Raw("data");
    MemorySource source(builder.bytes());
    Reader reader(&source);
    Entry entry;
    ASSERT_EQ(reader.Next(&entry), 1);
    EXPECT_EQ(entry.type, EntryType::kRegular);
    EXPECT_EQ(entry.size, size);
    EXPECT_EQ(entry.data_offset, kBlockSize);
    char buffer[8] = {};
    EXPECT_EQ(reader.ReadData(entry, 0, buffer, 4), 4);
    EXPECT_EQ(std::string(buffer, 4), "data");
    EXPECT_EQ(reader.ReadData(entry, 0, buffer, 5), kSyscallErrorIo);
    EXPECT_EQ(reader.ReadData(entry, size, buffer, 5), 0);
    // The archive ends inside its data.
    EXPECT_EQ(reader.Next(&entry), kSyscallErrorIo);
  }
}

TEST(TarReaderTest, RejectsBadSizes) {
  const std::vector<std::string> fields = {
      "0000000001x" + std::string(1, '\0'),
      "00000000018" + std::string(1, '\0'),
      "0000000001 2",
      "-0000000001" + std::string(1, '\0'),
      // Only spaces may come before the digits.
      std::string(10, '\0') + "12",
      // No digits.
      std::string(12, '\0'),
      std::string(12, ' '),
      // Base-256 numbers, positive and negative.
      std::string("\x80\0\0\0\0\0\0\0\0\0\0\x0a", 12),
      std::string(12, '\xff'),
  };
  for (const std::string& field : fields) {
    SCOPED_TRACE(t::PrintToString(field));
    Block block = MakeHeader({.name = "f"});
    PutField(block, kSizeOffset, 12, field);
    SetChecksum(block);
    ExpectInvalidHeader(ArchiveWithHeader(block));
  }
}

TEST(TarReaderTest, SkipsMembersWithUnusablePathsAndReadsTheMembersAfterThem) {
  const std::string c63(63, 'c');
  const std::string c64(64, 'c');
  // "/" + `prefix` + "/" + `name255` is `kFsMaxPathLength` bytes long.
  const std::string prefix = std::string(63, 'a') + "/" + std::string(63, 'b') +
                             "/" + std::string(27, 'c');
  const std::string name255 = std::string(63, 'd') + "/" + std::string(34, 'e');
  const std::string name256 = name255 + "e";
  const std::string path255 = "/" + prefix + "/" + name255;
  ASSERT_EQ(static_cast<int64_t>(path255.size()), kFsMaxPathLength);
  TarBuilder builder;
  builder.File("a/../b", "1")
      .File("../escape", "22")
      .File("x/..", Pattern(1000, 2))
      .Dir("dir/../")
      .File(c63, "4")
      .File(c64, "55")
      .File("d/" + c64 + "/f", "6")
      .Member({.name = name255, .prefix = prefix}, "7")
      .Member({.name = name256, .prefix = prefix}, "88")
      .File("last", Pattern(600, 3))
      .ZeroBlocks(2);
  int64_t end = 1;
  EXPECT_THAT(ReadArchive(builder.bytes(), &end),
              t::ElementsAre(Skipped(1),               //
                             Skipped(2),               //
                             Skipped(1000),            //
                             Skipped(),                //
                             Regular("/" + c63, "4"),  //
                             Skipped(2),               //
                             Skipped(1),               //
                             Regular(path255, "7"),    //
                             Skipped(2),               //
                             Regular("/last", Pattern(600, 3))));
  EXPECT_EQ(end, 0);
}

TEST(TarReaderTest, SkipsLinksDevicesFifosAndUnknownTypes) {
  TarBuilder builder;
  builder.Member({.name = "hard", .typeflag = '1', .linkname = "after"}, "")
      .Member({.name = "sym", .typeflag = '2', .linkname = "after"}, "")
      .Member({.name = "chr", .typeflag = '3'}, "")
      .Member({.name = "blk", .typeflag = '4'}, "")
      .Member({.name = "fifo", .typeflag = '6'}, "")
      .Member({.name = "vendor", .typeflag = 'A'}, Pattern(700, 4))
      .Member({.name = "lower", .typeflag = 'z'}, "z")
      .Member({.name = "contiguous", .typeflag = '7'}, "contiguous file")
      .File("after", "ok")
      .ZeroBlocks(2);
  int64_t end = 1;
  EXPECT_THAT(ReadArchive(builder.bytes(), &end),
              t::ElementsAre(Skipped(),                                  //
                             Skipped(),                                  //
                             Skipped(),                                  //
                             Skipped(),                                  //
                             Skipped(),                                  //
                             Skipped(700),                               //
                             Skipped(1),                                 //
                             Regular("/contiguous", "contiguous file"),  //
                             Regular("/after", "ok")));
  EXPECT_EQ(end, 0);
}

TEST(TarReaderTest, SkipsMembersAfterExtendedHeadersAndReadsGlobalHeaders) {
  const std::string long_name =
      std::string(80, 'l') + "/" + std::string(60, 'm');
  const std::string long_name_data = long_name + std::string(1, '\0');
  const std::string pax = PaxRecord("path", long_name);
  TarBuilder builder;
  builder.Member({.name = "./PaxHeaders/l", .typeflag = 'x'}, pax)
      .File(long_name.substr(0, 100), "pax")
      .Member({.name = "././@LongLink", .typeflag = 'L', .gnu = true},
              long_name_data)
      .Member({.name = long_name.substr(0, 100), .gnu = true}, "gnu")
      .Member({.name = "././@LongLink", .typeflag = 'K', .gnu = true},
              long_name_data)
      .Member({.name = "regular", .gnu = true}, "k")
      .Member({.name = "./PaxHeaders/global", .typeflag = 'g'},
              PaxRecord("comment", "protos"))
      .File("plain", "kept")
      .Member({.name = "x", .typeflag = 'x'}, pax)
      .Member({.name = "g", .typeflag = 'g'}, pax)
      .Dir("amended/")
      .Member({.name = "trailing", .typeflag = 'x'}, pax)
      .ZeroBlocks(2);
  int64_t end = 1;
  EXPECT_THAT(ReadArchive(builder.bytes(), &end),
              t::ElementsAre(Skipped(3),                 //
                             Skipped(3),                 //
                             Skipped(1),                 //
                             Regular("/plain", "kept"),  //
                             Skipped()));
  EXPECT_EQ(end, 0);

  // The member's header is the one after the extended header and its data.
  MemorySource source(builder.bytes());
  Reader reader(&source);
  Entry entry;
  ASSERT_EQ(reader.Next(&entry), 1);
  EXPECT_EQ(reader.header_offset(), 2 * kBlockSize);
  EXPECT_EQ(entry.data_offset, 3 * kBlockSize);
}

TEST(TarReaderTest, HandlesPaxMetadataSizeOverrideDeletionAndSparseHeaders) {
  // 1. Metadata-only PAX 'x' headers (atime/ctime/mtime/uid/gname/comment)
  //    keep regular files and directories intact, including across >512-byte
  //    headers and chunk boundaries.
  const std::string big_comment(700, 'c');
  const std::string meta_pax = PaxRecord("atime", "1700000000.123456") +
                               PaxRecord("ctime", "1700000000.654321") +
                               PaxRecord("mtime", "0") +
                               PaxRecord("comment", big_comment);
  ASSERT_GT(static_cast<int>(meta_pax.size()), kBlockSize);
  const std::string dir_pax = PaxRecord("uid", "1000") +
                              PaxRecord("gname", "users") +
                              PaxRecord("linkpath", "ignored");
  // 2. PAX 'size' overrides the ustar header's size (both shrinking and
  //    growing, and taking the last value across accumulated 'x' headers), and
  //    empty 'size=' / 'path=' values delete earlier overrides.
  const std::string sized_data = Pattern(700, 12);
  // 3. 'GNU.sparse.*' skips the member; empty 'GNU.sparse.*=' does not.
  TarBuilder builder;
  builder.Member({.name = "PaxHeaders/dir", .typeflag = 'x'}, dir_pax)
      .Dir("d/")
      .Member({.name = "PaxHeaders/meta", .typeflag = 'x'}, meta_pax)
      .File("d/kept", "hello")
      .Member({.name = "PaxHeaders/s1", .typeflag = 'x'},
              PaxRecord("size", "12") + PaxRecord("path", "skip_me"))
      .Member({.name = "PaxHeaders/s2", .typeflag = 'x'},
              PaxRecord("path", "") + PaxRecord("size", "700"))
      .Member({.name = "overridden", .size = 5}, sized_data)
      .Member({.name = "PaxHeaders/shrink", .typeflag = 'x'},
              PaxRecord("size", "0"))
      .Add(MakeHeader({.name = "zeroed", .size = 999}))
      .Member({.name = "PaxHeaders/unset_sz", .typeflag = 'x'},
              PaxRecord("size", "500") + PaxRecord("size", ""))
      .File("fallback", "abc")
      .Member({.name = "PaxHeaders/sp", .typeflag = 'x'},
              PaxRecord("GNU.sparse.major", "1") +
                  PaxRecord("GNU.sparse.realsize", "9999999999"))
      .File("sparse_file", "spdata")
      .Member({.name = "PaxHeaders/empty_sp", .typeflag = 'x'},
              PaxRecord("GNU.sparse.major", ""))
      .File("after_empty_sp", "ok")
      .Member({.name = "PaxHeaders/empty_x", .typeflag = 'x'}, "")
      .File("after_empty_x", "yes")
      .ZeroBlocks(2);

  for (const int64_t max_read : {int64_t{1}, int64_t{13}, kInt64Max}) {
    SCOPED_TRACE(max_read);
    MemorySource source(builder.bytes());
    source.set_max_read(max_read);
    Reader reader(&source);
    int64_t end = 1;
    EXPECT_THAT(ReadAll(reader, &end),
                t::ElementsAre(Directory("/d"),                     //
                               Regular("/d/kept", "hello"),         //
                               Regular("/overridden", sized_data),  //
                               Regular("/zeroed", ""),              //
                               Regular("/fallback", "abc"),         //
                               Skipped(6),                          //
                               Regular("/after_empty_sp", "ok"),    //
                               Regular("/after_empty_x", "yes")));
    EXPECT_EQ(end, 0);
  }
}

TEST(TarReaderTest, RejectsMalformedAndTruncatedPaxHeaders) {
  const std::vector<std::string> bad_records = {
      "0 a=\n",
      "05 a=\n",
      "abc a=\n",
      "99999999999999999999 a=\n",
      "5a=\n",
      "4 a=\n",
      "50 a=b\n",
      "6 abc\n",
      "5 =a\n",
      std::string("6 a\0b=1\n", 7),
      "6 a\nb=1\n",
      "6 a=bc",
      "6 a=b",
      "10 size=-1\n",
      "10 size=+5\n",
      "10 size=ab\n",
      "29 size=99999999999999999999\n",
  };
  for (const std::string& rec : bad_records) {
    SCOPED_TRACE(t::PrintToString(rec));
    TarBuilder builder;
    builder.Member({.name = "PaxHeaders/bad", .typeflag = 'x'}, rec)
        .File("a", "1")
        .ZeroBlocks(2);
    ExpectInvalidHeader(builder.bytes());
  }

  // PAX size exceeding int64_t bounds in FollowingHeaderOffset on the member
  // header (offset 2 * kBlockSize).
  {
    TarBuilder builder;
    builder
        .Member({.name = "PaxHeaders/huge", .typeflag = 'x'},
                PaxRecord("size", std::to_string(kInt64Max)))
        .File("a", "")
        .ZeroBlocks(2);
    MemorySource source(builder.bytes());
    Reader reader(&source);
    Entry entry;
    EXPECT_EQ(reader.Next(&entry), kSyscallErrorInvalid);
    EXPECT_EQ(reader.header_offset(), 2 * kBlockSize);
  }

  // Truncated in the middle of the 'x' header data.
  {
    const std::string pax = PaxRecord("mtime", "1");
    TarBuilder builder;
    builder.Add(MakeHeader({.name = "PaxHeaders/eof",
                            .typeflag = 'x',
                            .size = static_cast<int64_t>(pax.size())}));
    builder.Raw(pax.substr(0, pax.size() - 2));
    MemorySource source(builder.bytes());
    Reader reader(&source);
    Entry entry;
    EXPECT_EQ(reader.Next(&entry), kSyscallErrorIo);
  }

  // Over-returning source while reading 'x' header data (not the header).
  {
    TarBuilder builder;
    builder
        .Member({.name = "PaxHeaders/x", .typeflag = 'x'},
                PaxRecord("mtime", "1"))
        .File("a", "1")
        .ZeroBlocks(2);
    MemorySource source(builder.bytes());
    source.set_extra(1, kBlockSize);
    Reader reader(&source);
    Entry entry;
    EXPECT_EQ(reader.Next(&entry), kSyscallErrorIo);
  }
}

TEST(TarReaderTest, FollowsOldGnuSparseExtensionBlocksAndIgnoresPosixByte482) {
  auto make_ext_block = [](const bool is_extended) {
    Block ext{};
    ext[504] = is_extended ? 1 : 0;
    return ext;
  };

  // 0, 1, and 3 extension blocks followed by a regular file.
  for (const int ext_count : {0, 1, 3}) {
    SCOPED_TRACE(ext_count);
    const std::string sparse_data = Pattern(700, 14);
    Block hdr = MakeHeader({.name = "sparse.bin",
                            .typeflag = 'S',
                            .size = static_cast<int64_t>(sparse_data.size()),
                            .gnu = true});
    hdr[482] = ext_count > 0 ? 1 : 0;
    SetChecksum(hdr);

    TarBuilder builder;
    builder.Add(hdr);
    for (int i = 0; i < ext_count; ++i) {
      builder.Add(make_ext_block(i + 1 < ext_count));
    }
    builder.Raw(sparse_data);
    builder.PadTo(RoundUpToBlock(builder.size()));
    builder.File("after.txt", "ok").ZeroBlocks(2);

    int64_t end = 1;
    EXPECT_THAT(ReadArchive(builder.bytes(), &end),
                t::ElementsAre(Skipped(700), Regular("/after.txt", "ok")));
    EXPECT_EQ(end, 0);
  }

  // Truncated inside or before the extension chain -> kSyscallErrorIo.
  {
    Block hdr = MakeHeader(
        {.name = "sparse.bin", .typeflag = 'S', .size = 0, .gnu = true});
    hdr[482] = 1;
    SetChecksum(hdr);
    TarBuilder missing_ext;
    missing_ext.Add(hdr);
    MemorySource s1(missing_ext.bytes());
    Reader r1(&s1);
    Entry entry;
    EXPECT_EQ(r1.Next(&entry), kSyscallErrorIo);

    TarBuilder partial_ext;
    partial_ext.Add(hdr).Add(make_ext_block(true)).Raw("partial");
    MemorySource s2(partial_ext.bytes());
    Reader r2(&s2);
    EXPECT_EQ(r2.Next(&entry), kSyscallErrorIo);
  }

  // POSIX ustar header with typeflag 'S' and nonzero byte 482 (in prefix):
  // must NOT read the data block as an extension block.
  {
    const std::string prefix(150, 'p');
    ASSERT_GT(static_cast<int>(prefix.size()), 482 - kPrefixOffset);
    TarBuilder builder;
    builder
        .Member({.name = "posix_s",
                 .typeflag = 'S',
                 .prefix = prefix,
                 .gnu = false},
                "data")
        .File("next", "good")
        .ZeroBlocks(2);
    int64_t end = 1;
    EXPECT_THAT(ReadArchive(builder.bytes(), &end),
                t::ElementsAre(Skipped(4), Regular("/next", "good")));
    EXPECT_EQ(end, 0);
  }
}

TEST(TarReaderTest, EndsAtAZeroBlockOrWhereTheSourceEndsAtABlockBoundary) {
  // "a" at 0 with its data at 512, and "d" at 1024: the end is at 1536.
  TarBuilder members;
  members.File("a", "1").Dir("d/");
  TarBuilder two_zero_blocks = members;
  two_zero_blocks.ZeroBlocks(2);
  TarBuilder one_zero_block = members;
  one_zero_block.ZeroBlocks(1).File("ignored", "x").Raw("garbage");
  TarBuilder records = members;
  records.ZeroBlocks(2).PadTo(10240);
  for (const TarBuilder& archive :
       {two_zero_blocks, one_zero_block, members, records}) {
    MemorySource source(archive.bytes());
    Reader reader(&source);
    int64_t end = 1;
    EXPECT_THAT(ReadAll(reader, &end),
                t::ElementsAre(Regular("/a", "1"), Directory("/d")));
    EXPECT_EQ(end, 0);
    EXPECT_EQ(reader.header_offset(), 3 * kBlockSize);
  }

  // Data that ends without padding.
  TarBuilder unpadded;
  unpadded.Dir("d/").Add(MakeHeader({.name = "a", .size = 3})).Raw("abc");
  MemorySource unpadded_source(unpadded.bytes());
  Reader unpadded_reader(&unpadded_source);
  int64_t end = 1;
  EXPECT_THAT(ReadAll(unpadded_reader, &end),
              t::ElementsAre(Directory("/d"), Regular("/a", "abc")));
  EXPECT_EQ(end, 0);
  EXPECT_EQ(unpadded_reader.header_offset(), 3 * kBlockSize);

  // No bytes at all.
  MemorySource empty(std::vector<uint8_t>{});
  Reader empty_reader(&empty);
  Entry entry;
  EXPECT_EQ(empty_reader.Next(&entry), 0);
  EXPECT_EQ(empty_reader.header_offset(), 0);
}

TEST(TarReaderTest, FailsWithIoErrorIfTheArchiveEndsInsideAHeader) {
  const Block header = MakeHeader({.name = "b"});
  TarBuilder builder;
  builder.File("a", "1").Raw(std::string(header.begin(), header.begin() + 100));
  for (const int64_t max_read : {kInt64Max, int64_t{1}}) {
    SCOPED_TRACE(max_read);
    MemorySource source(builder.bytes());
    source.set_max_read(max_read);
    Reader reader(&source);
    int64_t end = 1;
    EXPECT_THAT(ReadAll(reader, &end), t::ElementsAre(Regular("/a", "1")));
    EXPECT_EQ(end, kSyscallErrorIo);
    EXPECT_EQ(reader.header_offset(), 2 * kBlockSize);
  }

  MemorySource one_byte(std::vector<uint8_t>{'0'});
  Reader reader(&one_byte);
  Entry entry;
  EXPECT_EQ(reader.Next(&entry), kSyscallErrorIo);
  EXPECT_EQ(reader.header_offset(), 0);
}

TEST(TarReaderTest, FailsWithIoErrorIfTheArchiveEndsInsideTheDataOfAMember) {
  // A regular file with 600 of its 1000 data bytes: ReadData() fails for the
  // missing ones, and Next() fails too.
  const std::string data = Pattern(1000, 5);
  TarBuilder cut_file;
  cut_file.Add(MakeHeader({.name = "cut", .size = 1000}))
      .Raw(data.substr(0, 600));
  MemorySource source(cut_file.bytes());
  Reader reader(&source);
  Entry entry;
  ASSERT_EQ(reader.Next(&entry), 1);
  char buffer[1000] = {};
  EXPECT_EQ(reader.ReadData(entry, 0, buffer, 1000), kSyscallErrorIo);
  EXPECT_EQ(reader.ReadData(entry, 550, buffer, 100), kSyscallErrorIo);
  ASSERT_EQ(reader.ReadData(entry, 0, buffer, 600), 600);
  EXPECT_EQ(std::string(buffer, 600), data.substr(0, 600));
  EXPECT_EQ(reader.Next(&entry), kSyscallErrorIo);
  EXPECT_EQ(reader.header_offset(), 3 * kBlockSize);

  // Also the data of directories and skipped members, which nobody reads.
  const std::vector<std::pair<char, Member>> cases = {
      {'5', Directory("/m", 700)},
      {'2', Skipped(700)},
      {'A', Skipped(700)},
  };
  for (const auto& [typeflag, member] : cases) {
    SCOPED_TRACE(typeflag);
    TarBuilder builder;
    builder.File("a", "1")
        .Add(MakeHeader({.name = "m", .typeflag = typeflag, .size = 700}))
        .Raw(Pattern(699, 6));
    int64_t end = 1;
    EXPECT_THAT(ReadArchive(builder.bytes(), &end),
                t::ElementsAre(Regular("/a", "1"), member));
    EXPECT_EQ(end, kSyscallErrorIo);
  }
}

TEST(TarReaderTest, PassesTheErrorsOfTheSourceOn) {
  // "a" at 0 with 1000 data bytes at 512, "b" at 1536 with its data at 2048,
  // and the end at 2560.
  const std::string data = Pattern(1000, 7);
  TarBuilder builder;
  builder.File("a", data).File("b", "2").ZeroBlocks(2);
  {
    MemorySource source(builder.bytes());
    source.Fail(0, kInt64Max, kSyscallErrorFault);
    Reader reader(&source);
    Entry entry;
    EXPECT_EQ(reader.Next(&entry), kSyscallErrorFault);
  }
  {
    // Inside the second header.
    MemorySource source(builder.bytes());
    source.Fail(3 * kBlockSize + 100, kInt64Max, kSyscallErrorNoMemory);
    Reader reader(&source);
    int64_t end = 1;
    EXPECT_THAT(ReadAll(reader, &end), t::ElementsAre(Regular("/a", data)));
    EXPECT_EQ(end, kSyscallErrorNoMemory);
    EXPECT_EQ(reader.header_offset(), 3 * kBlockSize);
  }
  {
    // Inside the data of "a", 188 bytes after its start.
    MemorySource source(builder.bytes());
    source.Fail(700, kInt64Max, kSyscallErrorBusy);
    Reader reader(&source);
    Entry entry;
    ASSERT_EQ(reader.Next(&entry), 1);
    char buffer[1000] = {};
    EXPECT_EQ(reader.ReadData(entry, 0, buffer, 1000), kSyscallErrorBusy);
    ASSERT_EQ(reader.ReadData(entry, 0, buffer, 188), 188);
    EXPECT_EQ(std::string(buffer, 188), data.substr(0, 188));
    EXPECT_EQ(reader.Next(&entry), kSyscallErrorBusy);
  }
  {
    // While Next() checks that the source holds the data before its end.
    TarBuilder no_end;
    no_end.Member({.name = "v", .typeflag = 'A'}, Pattern(100, 8));
    MemorySource source(no_end.bytes());
    source.Fail(600, 700, kSyscallErrorAccess);
    Reader reader(&source);
    Entry entry;
    ASSERT_EQ(reader.Next(&entry), 1);
    EXPECT_EQ(entry.type, EntryType::kSkipped);
    EXPECT_EQ(reader.Next(&entry), kSyscallErrorAccess);
  }
}

TEST(TarReaderTest, FailsWithIoErrorIfTheSourceReturnsMoreThanAskedFor) {
  TarBuilder builder;
  builder.File("a", "123").ZeroBlocks(2);
  {
    MemorySource source(builder.bytes());
    source.set_extra(1);
    Reader reader(&source);
    Entry entry;
    EXPECT_EQ(reader.Next(&entry), kSyscallErrorIo);
  }
  MemorySource source(builder.bytes());
  Reader reader(&source);
  Entry entry;
  ASSERT_EQ(reader.Next(&entry), 1);
  source.set_extra(1);
  char buffer[8] = {};
  EXPECT_EQ(reader.ReadData(entry, 0, buffer, 3), kSyscallErrorIo);

  // Over-returning on the 1-byte CheckLastDataByte() read at archive end.
  TarBuilder unpadded;
  unpadded.Add(MakeHeader({.name = "a", .size = 3})).Raw("abc");
  MemorySource unpadded_source(unpadded.bytes());
  Reader unpadded_reader(&unpadded_source);
  ASSERT_EQ(unpadded_reader.Next(&entry), 1);
  unpadded_source.set_extra(1);
  EXPECT_EQ(unpadded_reader.Next(&entry), kSyscallErrorIo);
}

TEST(TarReaderTest, KeepsReturningTheEndOrTheErrorWithoutReading) {
  TarBuilder builder;
  builder.File("a", "1").ZeroBlocks(2);
  {
    MemorySource source(builder.bytes());
    Reader reader(&source);
    Entry entry;
    ASSERT_EQ(reader.Next(&entry), 1);
    ASSERT_EQ(reader.Next(&entry), 0);
    const int reads = source.reads();
    for (int i = 0; i < 3; ++i) {
      EXPECT_EQ(reader.Next(&entry), 0);
    }
    EXPECT_EQ(source.reads(), reads);
    EXPECT_EQ(reader.header_offset(), 2 * kBlockSize);
  }
  {
    // Also once the source works again.
    MemorySource source(builder.bytes());
    source.Fail(0, kInt64Max, kSyscallErrorFault);
    Reader reader(&source);
    Entry entry;
    ASSERT_EQ(reader.Next(&entry), kSyscallErrorFault);
    source.Fail(kInt64Max, kInt64Max, 0);
    const int reads = source.reads();
    for (int i = 0; i < 3; ++i) {
      EXPECT_EQ(reader.Next(&entry), kSyscallErrorFault);
    }
    EXPECT_EQ(source.reads(), reads);
  }
}

TEST(TarReaderTest, ReadDataChecksItsArgumentsAndReadsOnlyTheDataOfTheEntry) {
  const std::string data = Pattern(700, 9);
  TarBuilder builder;
  builder.File("a", data).File("b", "next").ZeroBlocks(2);
  MemorySource source(builder.bytes());
  Reader reader(&source);
  Entry a;
  ASSERT_EQ(reader.Next(&a), 1);
  char buffer[1000] = {};
  EXPECT_EQ(reader.ReadData(a, -1, buffer, 1), kSyscallErrorInvalid);
  EXPECT_EQ(reader.ReadData(a, 701, buffer, 1), kSyscallErrorInvalid);
  EXPECT_EQ(reader.ReadData(a, 0, buffer, -1), kSyscallErrorInvalid);
  EXPECT_EQ(reader.ReadData(a, 0, nullptr, 1), kSyscallErrorInvalid);
  Entry bad = a;
  bad.size = -1;
  EXPECT_EQ(reader.ReadData(bad, 0, buffer, 1), kSyscallErrorInvalid);
  bad = a;
  bad.data_offset = -1;
  EXPECT_EQ(reader.ReadData(bad, 0, buffer, 1), kSyscallErrorInvalid);
  bad = a;
  bad.data_offset = kInt64Max - 10;
  EXPECT_EQ(reader.ReadData(bad, 0, buffer, 1), kSyscallErrorInvalid);

  // Nothing to read.
  const int reads = source.reads();
  EXPECT_EQ(reader.ReadData(a, 0, nullptr, 0), 0);
  EXPECT_EQ(reader.ReadData(a, 100, buffer, 0), 0);
  EXPECT_EQ(reader.ReadData(a, 700, buffer, 10), 0);
  EXPECT_EQ(source.reads(), reads);

  // Fewer bytes are left than asked for: the read stops at the end of the
  // data.
  std::memset(buffer, 0x5A, sizeof(buffer));
  ASSERT_EQ(reader.ReadData(a, 650, buffer, 1000), 50);
  EXPECT_EQ(std::string(buffer, 50), data.substr(650));
  EXPECT_EQ(buffer[50], 0x5A);
  ASSERT_EQ(reader.ReadData(a, 10, buffer, 20), 20);
  EXPECT_EQ(std::string(buffer, 20), data.substr(10, 20));

  // The data of an entry stays readable after Next() moved on.
  Entry b;
  ASSERT_EQ(reader.Next(&b), 1);
  ASSERT_EQ(reader.ReadData(a, 0, buffer, 1000), 700);
  EXPECT_EQ(std::string(buffer, 700), data);
  ASSERT_EQ(reader.ReadData(b, 0, buffer, 1000), 4);
  EXPECT_EQ(std::string(buffer, 4), "next");
}

TEST(TarReaderTest, HandlesSourcesThatReturnOneByteAtATime) {
  const std::string data = Pattern(1300, 10);
  TarBuilder builder;
  builder.Dir("./")
      .File("a", data)
      .Member({.name = "x", .typeflag = 'x'}, PaxRecord("path", "long"))
      .File("long", "pax")
      .Member({.name = "sym", .typeflag = '2', .linkname = "a"}, "")
      .File("empty", "")
      .ZeroBlocks(2);
  const std::vector<Member> expected = {
      Directory("/"), Regular("/a", data),   Skipped(3),
      Skipped(),      Regular("/empty", ""),
  };
  for (const int64_t max_read :
       {int64_t{1}, int64_t{7}, int64_t{kBlockSize}, kInt64Max}) {
    for (const int64_t chunk : {int64_t{1}, int64_t{333}, int64_t{4096}}) {
      SCOPED_TRACE(t::Message() << max_read << " " << chunk);
      MemorySource source(builder.bytes());
      source.set_max_read(max_read);
      Reader reader(&source);
      int64_t end = 1;
      EXPECT_EQ(ReadAll(reader, &end, chunk), expected);
      EXPECT_EQ(end, 0);
    }
  }
}

TEST(TarReaderTest, RejectsANullEntryOrSource) {
  // Like a `constinit` global of a ring-3 program.
  static constinit Reader no_source(nullptr);
  Entry entry;
  char buffer[1] = {};
  EXPECT_EQ(no_source.Next(&entry), kSyscallErrorInvalid);
  EXPECT_EQ(no_source.ReadData(entry, 0, buffer, 1), kSyscallErrorInvalid);

  TarBuilder builder;
  builder.File("a", "1").ZeroBlocks(2);
  MemorySource source(builder.bytes());
  Reader reader(&source);
  EXPECT_EQ(reader.Next(nullptr), kSyscallErrorInvalid);
  EXPECT_EQ(source.reads(), 0);
  // A null entry doesn't end the archive.
  ASSERT_EQ(reader.Next(&entry), 1);
  EXPECT_EQ(PathOf(entry), "/a");
}

}  // namespace
}  // namespace protos::tar
