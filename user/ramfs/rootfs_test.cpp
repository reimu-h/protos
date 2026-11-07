#include "user/ramfs/rootfs.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "syscall_abi.h"
#include "user/ramfs/ramfs.h"
#include "user/ramfs/tar.h"

namespace protos::ramfs {
namespace {

namespace t = ::testing;

constexpr int64_t kInt64Max = std::numeric_limits<int64_t>::max();
// The client with which the tests read files.
constexpr uint64_t kTestClient = 1;

class CountingBlockAllocator final : public BlockAllocator {
 public:
  uint8_t* AllocBlock() override {
    if (max_blocks_ >= 0 && live_blocks() >= max_blocks_) {
      return nullptr;
    }
    auto blk = std::make_unique<AlignedBlock>();
    std::memset(blk->bytes, 0, sizeof(blk->bytes));
    uint8_t* const raw = blk->bytes;
    live_[raw] = std::move(blk);
    peak_blocks_ = std::max(peak_blocks_, live_blocks());
    return raw;
  }

  void FreeBlock(uint8_t* const block) override {
    ASSERT_NE(block, nullptr);
    const auto it = live_.find(block);
    ASSERT_NE(it, live_.end()) << "double or invalid FreeBlock";
    live_.erase(it);
  }

  void set_max_blocks(const int max_blocks) { max_blocks_ = max_blocks; }
  int live_blocks() const { return static_cast<int>(live_.size()); }
  int peak_blocks() const { return peak_blocks_; }

 private:
  struct alignas(64) AlignedBlock {
    uint8_t bytes[kBlockSize];
  };

  int max_blocks_ = -1;
  int peak_blocks_ = 0;
  std::unordered_map<uint8_t*, std::unique_ptr<AlignedBlock>> live_;
};

using Block = std::array<uint8_t, tar::kBlockSize>;

// `size` bytes that depend on `seed`, with NULs and bytes >= 0x80 among them.
static std::string Pattern(const int64_t size, const int seed) {
  std::string data(size, '\0');
  for (int64_t i = 0; i < size; ++i) {
    data[i] = static_cast<char>((i * 7 + int64_t{seed} * 31) % 251);
  }
  return data;
}

static int64_t RoundUpToBlock(const int64_t size) {
  return (size + tar::kBlockSize - 1) / tar::kBlockSize * tar::kBlockSize;
}

// Copies `bytes` to `block` at `offset`.
static void Put(Block& block, const int offset, const std::string& bytes) {
  ASSERT_LE(offset + static_cast<int>(bytes.size()), tar::kBlockSize);
  std::copy(bytes.begin(), bytes.end(), block.begin() + offset);
}

// `value` as `digits` octal digits followed by a NUL.
static std::string OctalField(const int64_t value, const int digits) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "%0*llo",     //
                digits,       //
                static_cast<unsigned long long>(value));
  return std::string(buf) + std::string(1, '\0');
}

// Stores the checksum of `block` as GNU tar does.
static void SetChecksum(Block& block) {
  Put(block, 148, std::string(8, ' '));
  int64_t sum = 0;
  for (const uint8_t byte : block) {
    sum += byte;
  }
  Put(block, 148, OctalField(sum, 6) + " ");
}

// The fields of a member header for MakeHeader(), stored without NULs.
struct HeaderSpec {
  std::string name = "";
  char typeflag = '0';
  int64_t size = 0;
  std::string linkname = "";
};

// A POSIX ustar header with the fields of `spec`.
static Block MakeHeader(const HeaderSpec& spec) {
  Block block{};
  Put(block, 0, spec.name);
  Put(block, 100, OctalField(spec.typeflag == '5' ? 0755 : 0644, 7));
  Put(block, 108, OctalField(0, 7));
  Put(block, 116, OctalField(0, 7));
  Put(block, 124, OctalField(spec.size, 11));
  Put(block, 136, OctalField(0, 11));
  block[156] = static_cast<uint8_t>(spec.typeflag);
  Put(block, 157, spec.linkname);
  Put(block, 257, std::string("ustar\0", 6) + "00");
  SetChecksum(block);
  return block;
}

// Builds an archive in memory.
class TarBuilder {
 public:
  // Appends `header` and `data`, padded with zeros to whole blocks.
  TarBuilder& Add(const Block& header, const std::string& data = "") {
    bytes_.insert(bytes_.end(), header.begin(), header.end());
    Raw(data);
    bytes_.resize(RoundUpToBlock(size()));
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

  // Appends the two zero blocks that end an archive, and pads it to whole
  // 10240-byte records, as tar does.
  TarBuilder& End() {
    bytes_.resize(size() + 2 * tar::kBlockSize);
    bytes_.resize((size() + 10239) / 10240 * 10240);
    return *this;
  }

  const std::vector<uint8_t>& bytes() const { return bytes_; }
  int64_t size() const { return static_cast<int64_t>(bytes_.size()); }

 private:
  std::vector<uint8_t> bytes_;
};

// An archive in memory, which can return fewer bytes than asked for, and
// fail.
class MemorySource final : public tar::Source {
 public:
  explicit MemorySource(std::vector<uint8_t> bytes)
      : bytes_(std::move(bytes)) {}

  int64_t Read(const int64_t offset,  //
               void* const buffer,    //
               const int64_t size) override {
    EXPECT_GE(offset, 0);
    EXPECT_GT(size, 0);
    if (offset >= fail_offset_) {
      return fail_error_;
    }
    const int64_t total = static_cast<int64_t>(bytes_.size());
    if (offset >= total) {
      return 0;
    }
    const int64_t count =
        std::min({size, max_read_, total - offset, fail_offset_ - offset});
    std::memcpy(buffer, bytes_.data() + offset, count);
    return count;
  }

  // Makes every Read() return at most `max_read` bytes.
  void set_max_read(const int64_t max_read) { max_read_ = max_read; }

  // Makes the reads at or past `offset` fail with `error`, and those before
  // it stop at `offset`.
  void FailAt(const int64_t offset, const int64_t error) {
    fail_offset_ = offset;
    fail_error_ = error;
  }

 private:
  std::vector<uint8_t> bytes_;
  int64_t max_read_ = kInt64Max;
  int64_t fail_offset_ = kInt64Max;
  int64_t fail_error_ = 0;
};

// An archive whose first member is a regular file of `file_size` bytes, which
// it doesn't keep in memory, followed by the members and the end in `tail`.
class BigFileSource final : public tar::Source {
 public:
  BigFileSource(const std::string& name,  //
                const int64_t file_size,  //
                std::vector<uint8_t> tail)
      : header_(MakeHeader({.name = name, .size = file_size})),
        data_end_(tar::kBlockSize + file_size),
        tail_offset_(tar::kBlockSize + RoundUpToBlock(file_size)),
        tail_(std::move(tail)) {}

  int64_t Read(const int64_t offset,  //
               void* const buffer,    //
               const int64_t size) override {
    uint8_t* const out = static_cast<uint8_t*>(buffer);
    if (offset < tar::kBlockSize) {
      const int64_t count = std::min(size, tar::kBlockSize - offset);
      std::memcpy(out, header_.data() + offset, count);
      return count;
    }
    if (offset < data_end_) {
      ++data_reads_;
      const int64_t count = std::min(size, data_end_ - offset);
      std::memset(out, 'z', count);
      return count;
    }
    if (offset < tail_offset_) {
      // The padding after the data.
      const int64_t count = std::min(size, tail_offset_ - offset);
      std::memset(out, 0, count);
      return count;
    }
    const int64_t tail_pos = offset - tail_offset_;
    const int64_t tail_size = static_cast<int64_t>(tail_.size());
    if (tail_pos >= tail_size) {
      return 0;
    }
    const int64_t count = std::min(size, tail_size - tail_pos);
    std::memcpy(out, tail_.data() + tail_pos, count);
    return count;
  }

  int data_reads() const { return data_reads_; }

 private:
  Block header_;
  int64_t data_end_ = 0;
  int64_t tail_offset_ = 0;
  std::vector<uint8_t> tail_;
  int data_reads_ = 0;
};

class RootfsTest : public t::Test {
 protected:
  void SetUp() override { Reset(); }

  // Starts over with an empty filesystem, a new allocator and no stats.
  void Reset() {
    fs_.reset();
    alloc_ = std::make_unique<CountingBlockAllocator>();
    fs_ = std::make_unique<FileSystem>();
    fs_->Initialize(alloc_.get());
    stats_ = LoadStats{};
  }

  // Loads the archive that `source` reads into `fs_`, adding to `stats_`, and
  // returns the result of LoadArchive().
  int64_t Load(tar::Source* const source) {
    tar::Reader reader(source);
    const int64_t rc = LoadArchive(*fs_, reader, &stats_);
    EXPECT_EQ(fs_->ActiveHandleCount(), 0);
    return rc;
  }

  int64_t Load(const TarBuilder& archive, const int64_t max_read = kInt64Max) {
    MemorySource source(archive.bytes());
    source.set_max_read(max_read);
    return Load(&source);
  }

  // The type of `path`, or `FsNodeType::kNone` if nothing is there.
  FsNodeType TypeOf(const std::string& path) const {
    FsStat stat{};
    const int64_t rc =
        fs_->Stat(path.data(), static_cast<int64_t>(path.size()), &stat);
    return (rc == 0) ? stat.type : FsNodeType::kNone;
  }

  // The contents of the regular file `path`, read with FileSystem::Read().
  std::string ReadFile(const std::string& path) {
    FsOpenReply reply{};
    const int64_t open_rc = fs_->Open(kTestClient,                        //
                                      path.data(),                        //
                                      static_cast<int64_t>(path.size()),  //
                                      kFsOpenRead,                        //
                                      &reply);
    if (open_rc != 0) {
      ADD_FAILURE() << "can't open " << path << ": " << open_rc;
      return "";
    }
    std::string contents;
    std::vector<char> buffer(kSyscallFsMaxIoSize);
    for (;;) {
      const int64_t n = fs_->Read(kTestClient,    //
                                  reply.handle,   //
                                  buffer.data(),  //
                                  kSyscallFsMaxIoSize);
      EXPECT_GE(n, 0);
      if (n <= 0) {
        break;
      }
      contents.append(buffer.data(), n);
    }
    EXPECT_EQ(fs_->Close(kTestClient, reply.handle), 0);
    return contents;
  }

  std::unique_ptr<CountingBlockAllocator> alloc_;
  std::unique_ptr<FileSystem> fs_;
  LoadStats stats_;
};

TEST_F(RootfsTest, LoadsAnArchiveLikeTheRootfsArchiveOfTheBuild) {
  // What `tar --format=ustar --sort=name ... -C rootfs -cf rootfs.tar .`
  // writes.
  const std::string readme = "Welcome to protos.\n";
  const std::string shell = Pattern(5000, 1);
  TarBuilder archive;
  archive.Dir("./")
      .File("./README.txt", readme)
      .Dir("./doc/")
      .File("./doc/shell.txt", shell)
      .End();
  EXPECT_EQ(Load(archive), 0);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/2,
                                   /*directories=*/1,
                                   /*skipped=*/0,
                                   /*failed=*/0));
  EXPECT_EQ(TypeOf("/doc"), FsNodeType::kDirectory);
  EXPECT_EQ(ReadFile("/README.txt"), readme);
  EXPECT_EQ(ReadFile("/doc/shell.txt"), shell);
  // The root, the two files and "/doc".
  EXPECT_EQ(fs_->ActiveFnodeCount(), 4);
}

TEST_F(RootfsTest, CreatesMissingParentDirectories) {
  TarBuilder archive;
  archive.File("a/b/c.txt", "abc")
      .File("a/b/d.txt", "abd")
      .Dir("x/y/z/")
      .File("./a/e/f.txt", "f")
      .End();
  EXPECT_EQ(Load(archive), 0);
  // "/a", "/a/b", "/x", "/x/y", "/x/y/z" and "/a/e".
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/3,
                                   /*directories=*/6,
                                   /*skipped=*/0,
                                   /*failed=*/0));
  for (const char* const dir : {"/a", "/a/b", "/x", "/x/y", "/x/y/z", "/a/e"}) {
    EXPECT_EQ(TypeOf(dir), FsNodeType::kDirectory) << dir;
  }
  EXPECT_EQ(ReadFile("/a/b/c.txt"), "abc");
  EXPECT_EQ(ReadFile("/a/b/d.txt"), "abd");
  EXPECT_EQ(ReadFile("/a/e/f.txt"), "f");
}

TEST_F(RootfsTest, LaterMembersOverwriteFilesAndFindDirectories) {
  TarBuilder archive;
  archive.File("f", "the first version, which is longer")
      .File("f", "second")
      .Dir("d/")
      .Dir("./d")
      .File("d/g", "1")
      .Dir("d/")
      .End();
  EXPECT_EQ(Load(archive), 0);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/3,
                                   /*directories=*/1,
                                   /*skipped=*/0,
                                   /*failed=*/0));
  EXPECT_EQ(ReadFile("/f"), "second");
  EXPECT_EQ(ReadFile("/d/g"), "1");
}

TEST_F(RootfsTest, CountsFileAndDirectoryConflictsAsFailedAndLoadsTheRest) {
  // A file where a directory is (which stays), a directory where a file is,
  // and members below a file.
  TarBuilder archive;
  archive.Dir("a/")
      .File("a", "a file where a directory is")
      .File("b", "b")
      .Dir("b/")
      .File("b/c.txt", "c")
      .Dir("b/d/")
      .File("a/inner.txt", "inner")
      .End();
  EXPECT_EQ(Load(archive), 0);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/2,
                                   /*directories=*/1,
                                   /*skipped=*/0,
                                   /*failed=*/4));
  EXPECT_EQ(TypeOf("/a"), FsNodeType::kDirectory);
  EXPECT_EQ(ReadFile("/a/inner.txt"), "inner");
  EXPECT_EQ(ReadFile("/b"), "b");
  EXPECT_EQ(fs_->ActiveFnodeCount(), 4);
}

TEST_F(RootfsTest, CountsFilesThatRunOutOfSpaceAsFailedAndRemovesThem) {
  TarBuilder archive;
  archive.File("small1", Pattern(100, 1))
      .File("big", Pattern(100 * 1024, 2))
      .File("small2", Pattern(100, 3))
      .End();
  // Room for the root's directory block, the block of "small1", and 8 of the
  // 25 blocks of "big".
  alloc_->set_max_blocks(10);
  EXPECT_EQ(Load(archive), 0);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/2,
                                   /*directories=*/0,
                                   /*skipped=*/0,
                                   /*failed=*/1));
  EXPECT_EQ(TypeOf("/big"), FsNodeType::kNone);
  EXPECT_EQ(ReadFile("/small1"), Pattern(100, 1));
  EXPECT_EQ(ReadFile("/small2"), Pattern(100, 3));
  EXPECT_EQ(alloc_->live_blocks(), 3);
  EXPECT_EQ(fs_->ActiveFnodeCount(), 3);
}

TEST_F(RootfsTest, CountsEntriesThatRunOutOfSpaceAsFailed) {
  // Room for the root's directory block only: "/d" gets no directory block,
  // and the files in the root no data blocks.
  TarBuilder archive;
  archive.Dir("d/")
      .Dir("d/e/")
      .File("d/f.txt", "f")
      .File("empty.txt", "")
      .File("top.txt", "top")
      .End();
  alloc_->set_max_blocks(1);
  EXPECT_EQ(Load(archive), 0);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/1,
                                   /*directories=*/1,
                                   /*skipped=*/0,
                                   /*failed=*/3));
  EXPECT_EQ(TypeOf("/d"), FsNodeType::kDirectory);
  EXPECT_EQ(TypeOf("/d/e"), FsNodeType::kNone);
  EXPECT_EQ(TypeOf("/d/f.txt"), FsNodeType::kNone);
  EXPECT_EQ(ReadFile("/empty.txt"), "");
  EXPECT_EQ(TypeOf("/top.txt"), FsNodeType::kNone);
  EXPECT_EQ(alloc_->live_blocks(), 1);
}

TEST_F(RootfsTest, CountsMembersThatFindNoFreeFnodeAsFailed) {
  constexpr int kFiles = kMaxFnodes + 10;
  // The root takes one of the `kMaxFnodes` fnodes.
  constexpr int kLoaded = kMaxFnodes - 1;
  const auto name = [](const int i) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "/f%03d", i);
    return std::string(buf);
  };
  const auto contents = [](const int i) {
    return std::string(1, static_cast<char>('a' + i % 26));
  };
  TarBuilder archive;
  for (int i = 0; i < kFiles; ++i) {
    archive.File(name(i), contents(i));
  }
  archive.Dir("dir/").File("sub/file", "x").End();
  EXPECT_EQ(Load(archive), 0);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/kLoaded,
                                   /*directories=*/0,
                                   /*skipped=*/0,
                                   /*failed=*/kFiles - kLoaded + 2));
  EXPECT_EQ(fs_->ActiveFnodeCount(), kMaxFnodes);
  EXPECT_EQ(ReadFile(name(0)), contents(0));
  EXPECT_EQ(ReadFile(name(kLoaded - 1)), contents(kLoaded - 1));
  EXPECT_EQ(TypeOf(name(kLoaded)), FsNodeType::kNone);
  EXPECT_EQ(TypeOf("/dir"), FsNodeType::kNone);
  EXPECT_EQ(TypeOf("/sub"), FsNodeType::kNone);
}

TEST_F(RootfsTest, LoadsFilesThatSpanIndirectBlocks) {
  // 25 blocks: the 12 direct ones and 13 of the single-indirect block.
  const std::string big = Pattern(100 * 1024, 4);
  TarBuilder archive;
  archive.File("big.bin", big).End();
  EXPECT_EQ(Load(archive), 0);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/1,
                                   /*directories=*/0,
                                   /*skipped=*/0,
                                   /*failed=*/0));
  EXPECT_EQ(ReadFile("/big.bin"), big);
  // The root's directory block, the data blocks and the single-indirect one.
  EXPECT_EQ(alloc_->live_blocks(), 1 + 25 + 1);
}

TEST_F(RootfsTest, LoadsFromSourcesThatReturnFewerBytesThanAskedFor) {
  const std::string big = Pattern(10 * 1024 + 3, 5);
  TarBuilder archive;
  archive.Dir("./").File("big.bin", big).Dir("d/").File("d/small", "s").End();
  for (const int64_t max_read : {int64_t{1}, int64_t{333}}) {
    SCOPED_TRACE(max_read);
    Reset();
    EXPECT_EQ(Load(archive, max_read), 0);
    EXPECT_THAT(stats_, t::FieldsAre(/*files=*/2,
                                     /*directories=*/1,
                                     /*skipped=*/0,
                                     /*failed=*/0));
    EXPECT_EQ(ReadFile("/big.bin"), big);
    EXPECT_EQ(ReadFile("/d/small"), "s");
  }
}

TEST_F(RootfsTest, CountsAFileLargerThanTheLargestFileAsFailed) {
  TarBuilder tail;
  tail.File("after.txt", "after").End();
  BigFileSource source("huge", kFsMaxFileSize + 1, tail.bytes());
  EXPECT_EQ(Load(&source), 0);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/1,
                                   /*directories=*/0,
                                   /*skipped=*/0,
                                   /*failed=*/1));
  EXPECT_EQ(source.data_reads(), 0);
  EXPECT_EQ(TypeOf("/huge"), FsNodeType::kNone);
  EXPECT_EQ(ReadFile("/after.txt"), "after");
  EXPECT_EQ(alloc_->live_blocks(), 2);
  EXPECT_EQ(alloc_->peak_blocks(), 2);

  // Also removes an earlier regular file at the same path without allocating
  // blocks, while leaving an existing directory intact.
  Reset();
  TarBuilder earlier_file;
  earlier_file.File("huge", "old").End();
  EXPECT_EQ(Load(earlier_file), 0);
  ASSERT_EQ(TypeOf("/huge"), FsNodeType::kRegular);
  BigFileSource over_file("huge", kFsMaxFileSize + 1, tail.bytes());
  stats_ = LoadStats{};
  EXPECT_EQ(Load(&over_file), 0);
  EXPECT_EQ(over_file.data_reads(), 0);
  EXPECT_EQ(TypeOf("/huge"), FsNodeType::kNone);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/1,
                                   /*directories=*/0,
                                   /*skipped=*/0,
                                   /*failed=*/1));

  Reset();
  TarBuilder earlier_dir;
  earlier_dir.Dir("huge/").End();
  EXPECT_EQ(Load(earlier_dir), 0);
  BigFileSource over_dir("huge", kFsMaxFileSize + 1, tail.bytes());
  stats_ = LoadStats{};
  EXPECT_EQ(Load(&over_dir), 0);
  EXPECT_EQ(over_dir.data_reads(), 0);
  EXPECT_EQ(TypeOf("/huge"), FsNodeType::kDirectory);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/1,
                                   /*directories=*/0,
                                   /*skipped=*/0,
                                   /*failed=*/1));
}

TEST_F(RootfsTest, LoadsAFileOfExactlyTheLargestFileSize) {
  TarBuilder tail;
  tail.End();
  BigFileSource source("max.bin", kFsMaxFileSize, tail.bytes());
  EXPECT_EQ(Load(&source), 0);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/1,
                                   /*directories=*/0,
                                   /*skipped=*/0,
                                   /*failed=*/0));
  FsStat st{};
  ASSERT_EQ(fs_->Stat("/max.bin", 8, &st), 0);
  EXPECT_EQ(st.type, FsNodeType::kRegular);
  EXPECT_EQ(st.size, kFsMaxFileSize);

  FsOpenReply reply{};
  ASSERT_EQ(fs_->Open(kTestClient, "/max.bin", 8, kFsOpenRead, &reply), 0);
  char edge[16] = {};
  ASSERT_EQ(fs_->Read(kTestClient, reply.handle, edge, sizeof(edge)),
            static_cast<int64_t>(sizeof(edge)));
  EXPECT_EQ(std::string(edge, sizeof(edge)), std::string(sizeof(edge), 'z'));
  ASSERT_EQ(
      fs_->Seek(kTestClient, reply.handle, kFsMaxFileSize - 4, kFsSeekSet),
      kFsMaxFileSize - 4);
  ASSERT_EQ(fs_->Read(kTestClient, reply.handle, edge, sizeof(edge)), 4);
  EXPECT_EQ(std::string(edge, 4), "zzzz");
  EXPECT_EQ(fs_->Close(kTestClient, reply.handle), 0);
}

TEST_F(RootfsTest, StopsAtATruncatedArchiveAndKeepsTheMembersBefore) {
  TarBuilder archive;
  archive.File("keep.txt", "kept")
      .Dir("dir/")
      .Add(MakeHeader({.name = "dir/cut.bin", .size = 5000}))
      .Raw(Pattern(2000, 6));
  EXPECT_EQ(Load(archive), kSyscallErrorIo);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/1,
                                   /*directories=*/1,
                                   /*skipped=*/0,
                                   /*failed=*/1));
  EXPECT_EQ(ReadFile("/keep.txt"), "kept");
  EXPECT_EQ(TypeOf("/dir"), FsNodeType::kDirectory);
  EXPECT_EQ(TypeOf("/dir/cut.bin"), FsNodeType::kNone);
  // The root's directory block and the block of "/keep.txt".
  EXPECT_EQ(alloc_->live_blocks(), 2);
}

TEST_F(RootfsTest, StopsAtAnArchiveThatEndsInsideAHeader) {
  const Block header = MakeHeader({.name = "next"});
  TarBuilder archive;
  archive.File("a", "1").Raw(std::string(header.begin(), header.begin() + 200));
  EXPECT_EQ(Load(archive), kSyscallErrorIo);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/1,
                                   /*directories=*/0,
                                   /*skipped=*/0,
                                   /*failed=*/0));
  EXPECT_EQ(ReadFile("/a"), "1");
}

TEST_F(RootfsTest, StopsAtACorruptHeaderAndKeepsTheMembersBefore) {
  Block corrupt = MakeHeader({.name = "b.txt", .size = 1});
  // A change after the checksum was computed.
  corrupt[0] = 'B';
  TarBuilder archive;
  archive.File("a.txt", "a")
      .Dir("dir/")
      .Add(corrupt, "b")
      .File("c.txt", "c")
      .End();
  EXPECT_EQ(Load(archive), kSyscallErrorInvalid);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/1,
                                   /*directories=*/1,
                                   /*skipped=*/0,
                                   /*failed=*/0));
  EXPECT_EQ(ReadFile("/a.txt"), "a");
  EXPECT_EQ(TypeOf("/dir"), FsNodeType::kDirectory);
  EXPECT_EQ(TypeOf("/b.txt"), FsNodeType::kNone);
  EXPECT_EQ(TypeOf("/B.txt"), FsNodeType::kNone);
  EXPECT_EQ(TypeOf("/c.txt"), FsNodeType::kNone);
}

TEST_F(RootfsTest, StopsAtAnErrorOfTheSource) {
  // "a" at 0 with its data at 512, "b" at 1024 with 3000 data bytes at 1536,
  // and "c" at 4608.
  TarBuilder archive;
  archive.File("a", "1").File("b", Pattern(3000, 7)).File("c", "3").End();

  // Inside the header of "c".
  {
    MemorySource source(archive.bytes());
    source.FailAt(4608 + 10, kSyscallErrorFault);
    EXPECT_EQ(Load(&source), kSyscallErrorFault);
    EXPECT_THAT(stats_, t::FieldsAre(/*files=*/2,
                                     /*directories=*/0,
                                     /*skipped=*/0,
                                     /*failed=*/0));
    EXPECT_EQ(ReadFile("/b"), Pattern(3000, 7));
    EXPECT_EQ(TypeOf("/c"), FsNodeType::kNone);
  }

  // Inside the data of "b", which doesn't stay.
  Reset();
  {
    MemorySource source(archive.bytes());
    source.FailAt(2000, kSyscallErrorBusy);
    EXPECT_EQ(Load(&source), kSyscallErrorBusy);
    EXPECT_THAT(stats_, t::FieldsAre(/*files=*/1,
                                     /*directories=*/0,
                                     /*skipped=*/0,
                                     /*failed=*/1));
    EXPECT_EQ(ReadFile("/a"), "1");
    EXPECT_EQ(TypeOf("/b"), FsNodeType::kNone);
  }
}

TEST_F(RootfsTest, CountsSkippedMembers) {
  // Links, a FIFO, unusable paths, and a member whose real name is in a PAX
  // extended header; PAX global headers aren't members.
  TarBuilder archive;
  archive.Member({.name = "link", .typeflag = '2', .linkname = "a.txt"}, "")
      .Member({.name = "hard", .typeflag = '1', .linkname = "a.txt"}, "")
      .Member({.name = "fifo", .typeflag = '6'}, "")
      .File("../escape.txt", "x")
      .File(std::string(kFsMaxNameLength + 1, 'n'), "x")
      .Member({.name = "PaxHeaders/long", .typeflag = 'x'}, "13 path=long\n")
      .File("long", "x")
      .Member({.name = "PaxHeaders/global", .typeflag = 'g'}, "10 a=bcde\n")
      .File("a.txt", "a")
      .End();
  EXPECT_EQ(Load(archive), 0);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/1,
                                   /*directories=*/0,
                                   /*skipped=*/6,
                                   /*failed=*/0));
  EXPECT_EQ(ReadFile("/a.txt"), "a");
  EXPECT_EQ(fs_->ActiveFnodeCount(), 2);
}

TEST_F(RootfsTest, LoadsAnEmptyArchive) {
  EXPECT_EQ(Load(TarBuilder()), 0);
  TarBuilder end_only;
  end_only.End();
  EXPECT_EQ(Load(end_only), 0);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/0,
                                   /*directories=*/0,
                                   /*skipped=*/0,
                                   /*failed=*/0));
  EXPECT_EQ(fs_->ActiveFnodeCount(), 1);
}

TEST_F(RootfsTest, AddsToTheStatsAndAcceptsNoStats) {
  TarBuilder archive;
  archive.Dir("d/")
      .File("d/f", "1")
      .Member({.name = "link", .typeflag = '2', .linkname = "d/f"}, "")
      .End();
  EXPECT_EQ(Load(archive), 0);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/1,
                                   /*directories=*/1,
                                   /*skipped=*/1,
                                   /*failed=*/0));
  // The second time, the directory exists.
  EXPECT_EQ(Load(archive), 0);
  EXPECT_THAT(stats_, t::FieldsAre(/*files=*/2,
                                   /*directories=*/1,
                                   /*skipped=*/2,
                                   /*failed=*/0));

  TarBuilder other;
  other.File("d/g", "2").End();
  MemorySource source(other.bytes());
  tar::Reader reader(&source);
  EXPECT_EQ(LoadArchive(*fs_, reader, nullptr), 0);
  EXPECT_EQ(ReadFile("/d/g"), "2");
}

TEST_F(RootfsTest, LeavesTheFilesOfOtherClientsOpen) {
  const std::string path = "/open.txt";
  FsOpenReply reply{};
  ASSERT_EQ(fs_->Open(kTestClient,                        //
                      path.data(),                        //
                      static_cast<int64_t>(path.size()),  //
                      kFsOpenRead | kFsOpenCreate,        //
                      &reply),
            0);
  TarBuilder archive;
  archive.File("open.txt", "replaced").File("other.txt", "other").End();
  MemorySource source(archive.bytes());
  tar::Reader reader(&source);
  EXPECT_EQ(LoadArchive(*fs_, reader, &stats_), 0);
  EXPECT_EQ(stats_.files, 2);
  EXPECT_EQ(fs_->ActiveHandleCount(), 1);
  char buffer[16] = {};
  ASSERT_EQ(fs_->Read(kTestClient, reply.handle, buffer, sizeof(buffer)), 8);
  EXPECT_EQ(std::string(buffer, 8), "replaced");
  EXPECT_EQ(fs_->Close(kTestClient, reply.handle), 0);
}

}  // namespace
}  // namespace protos::ramfs
