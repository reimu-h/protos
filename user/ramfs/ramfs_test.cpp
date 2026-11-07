#include "user/ramfs/ramfs.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace protos::ramfs {
namespace {

namespace t = ::testing;

constexpr int64_t kInt64Max = std::numeric_limits<int64_t>::max();
constexpr int64_t kInt64Min = std::numeric_limits<int64_t>::min();
constexpr int32_t kInt32Max = std::numeric_limits<int32_t>::max();

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

 private:
  struct alignas(64) AlignedBlock {
    uint8_t bytes[kBlockSize];
  };

  int max_blocks_ = -1;
  std::unordered_map<uint8_t*, std::unique_ptr<AlignedBlock>> live_;
};

class RamfsTest : public t::Test {
 protected:
  void SetUp() override {
    fs_ = std::make_unique<FileSystem>();
    fs_->Initialize(&alloc_);
  }

  int64_t MkDirStr(const std::string& path) {
    return fs_->MkDir(path.data(), static_cast<int64_t>(path.size()));
  }

  int64_t UnlinkStr(const std::string& path) {
    return fs_->Unlink(path.data(), static_cast<int64_t>(path.size()));
  }

  int64_t StatStr(const std::string& path, FsStat* const st) {
    return fs_->Stat(path.data(), static_cast<int64_t>(path.size()), st);
  }

  int64_t RenameStr(const std::string& old_p, const std::string& new_p) {
    return fs_->Rename(old_p.data(), static_cast<int64_t>(old_p.size()),
                       new_p.data(), static_cast<int64_t>(new_p.size()));
  }

  int64_t OpenStr(const uint64_t client_id,    //
                  const std::string& path,     //
                  const uint64_t flags,        //
                  uint64_t* const out_handle,  //
                  FsNodeType* const out_type = nullptr) {
    FsOpenReply reply{};
    const int64_t rc = fs_->Open(client_id,                          //
                                 path.data(),                        //
                                 static_cast<int64_t>(path.size()),  //
                                 flags,                              //
                                 &reply);
    if (rc == 0) {
      if (out_handle != nullptr) {
        *out_handle = reply.handle;
      }
      if (out_type != nullptr) {
        *out_type = reply.type;
      }
    }
    return rc;
  }

  // Creates the empty regular file `path` and returns 0, or returns the
  // error.
  int64_t CreateFile(const std::string& path) {
    uint64_t h = 0;
    const int64_t rc = OpenStr(1, path, kFsOpenWrite | kFsOpenCreate, &h);
    return rc < 0 ? rc : fs_->Close(1, h);
  }

  // Reads up to `max_entries` entries of client 1's directory handle `dir_h`
  // and returns their names.
  std::vector<std::string> ReadDirNames(const uint64_t dir_h,
                                        const int64_t max_entries) {
    std::vector<FsDirEntry> page(static_cast<size_t>(max_entries));
    const int64_t n = fs_->ReadDir(1, dir_h, page.data(), max_entries);
    EXPECT_GE(n, 0);
    std::vector<std::string> names;
    for (int64_t i = 0; i < n; ++i) {
      const FsDirEntry& entry = page[static_cast<size_t>(i)];
      names.emplace_back(entry.name, static_cast<size_t>(entry.name_len));
    }
    return names;
  }

  CountingBlockAllocator alloc_;
  std::unique_ptr<FileSystem> fs_;
};

TEST_F(RamfsTest, RootStatAndPathResolution) {
  FsStat st{};
  ASSERT_EQ(StatStr("/", &st), 0);
  EXPECT_EQ(st.type, FsNodeType::kDirectory);
  EXPECT_EQ(st.fnode_id, 1u);
  EXPECT_EQ(st.nlink, 2);
  EXPECT_EQ(st.size, 0);

  FsStat st2{};
  ASSERT_EQ(StatStr("///", &st2), 0);
  EXPECT_EQ(st2.fnode_id, 1u);
  ASSERT_EQ(StatStr("/.", &st2), 0);
  EXPECT_EQ(st2.fnode_id, 1u);
  ASSERT_EQ(StatStr("/..", &st2), 0);
  EXPECT_EQ(st2.fnode_id, 1u);
  ASSERT_EQ(StatStr(".", &st2), 0);
  EXPECT_EQ(st2.fnode_id, 1u);

  EXPECT_EQ(StatStr("", &st2), kSyscallErrorNoEntry);
  const char with_nul[] = {'/', 'a', '\0', 'b'};
  EXPECT_EQ(fs_->Stat(with_nul, sizeof(with_nul), &st2), kSyscallErrorInvalid);

  const std::string too_long_name =
      "/" + std::string(static_cast<size_t>(kFsMaxNameLength + 1), 'x');
  EXPECT_EQ(MkDirStr(too_long_name), kSyscallErrorNameTooLong);

  std::string too_long_path;
  while (static_cast<int64_t>(too_long_path.size()) <= kFsMaxPathLength) {
    too_long_path += "/abcde";
  }
  EXPECT_EQ(StatStr(too_long_path, &st2), kSyscallErrorNameTooLong);

  ASSERT_EQ(MkDirStr("/dir"), 0);
  ASSERT_EQ(MkDirStr("dir/sub/"), 0);
  ASSERT_EQ(StatStr("/dir/sub/..", &st2), 0);
  FsStat dir_st{};
  ASSERT_EQ(StatStr("/dir", &dir_st), 0);
  EXPECT_EQ(st2.fnode_id, dir_st.fnode_id);
  EXPECT_EQ(dir_st.nlink, 3);

  uint64_t h = 0;
  ASSERT_EQ(OpenStr(1, "/dir/file", kFsOpenWrite | kFsOpenCreate, &h), 0);
  ASSERT_EQ(fs_->Close(1, h), 0);
  EXPECT_EQ(StatStr("/dir/file/", &st2), kSyscallErrorNotDir);
  EXPECT_EQ(StatStr("/dir/file/child", &st2), kSyscallErrorNotDir);
  EXPECT_EQ(MkDirStr("/dir/file/child"), kSyscallErrorNotDir);
}

TEST_F(RamfsTest, LongestNameAndPathAreAccepted) {
  // "/" + 63 + "/" + 63 + "/" + 63 + "/" + 62 bytes: `kFsMaxPathLength`.
  const std::string dir1 = "/" + std::string(63, 'a');
  const std::string dir2 = dir1 + "/" + std::string(63, 'b');
  const std::string dir3 = dir2 + "/" + std::string(63, 'c');
  const std::string longest_path = dir3 + "/" + std::string(62, 'd');
  ASSERT_EQ(static_cast<int64_t>(longest_path.size()), kFsMaxPathLength);
  ASSERT_EQ(MkDirStr(dir1), 0);
  ASSERT_EQ(MkDirStr(dir2), 0);
  ASSERT_EQ(MkDirStr(dir3), 0);
  ASSERT_EQ(CreateFile(longest_path), 0);
  FsStat st{};
  EXPECT_EQ(StatStr(longest_path, &st), 0);
  EXPECT_EQ(st.type, FsNodeType::kRegular);
  EXPECT_EQ(StatStr(longest_path + "/", &st), kSyscallErrorNameTooLong);
  EXPECT_EQ(StatStr(dir3 + "/" + std::string(63, 'd'), &st),
            kSyscallErrorNameTooLong);

  // A 64-byte component is too long, also before the last one.
  EXPECT_EQ(MkDirStr("/" + std::string(64, 'e')), kSyscallErrorNameTooLong);
  EXPECT_EQ(StatStr("/" + std::string(64, 'e') + "/f", &st),
            kSyscallErrorNameTooLong);

  // ReadDir() returns a 63-byte name with a NUL after it.
  uint64_t dir_h = 0;
  ASSERT_EQ(OpenStr(1, "/", kFsOpenRead | kFsOpenDirectory, &dir_h), 0);
  FsDirEntry entry{};
  ASSERT_EQ(fs_->ReadDir(1, dir_h, &entry, 1), 1);
  EXPECT_EQ(entry.name_len, kFsMaxNameLength);
  EXPECT_EQ(std::string(entry.name, static_cast<size_t>(kFsMaxNameLength)),
            std::string(63, 'a'));
  EXPECT_EQ(entry.name[kFsMaxNameLength], '\0');
  ASSERT_EQ(fs_->Close(1, dir_h), 0);
}

TEST_F(RamfsTest, MkDirAndUnlinkDirectoryReclaimAllBlocks) {
  ASSERT_EQ(MkDirStr("/a"), 0);
  EXPECT_EQ(MkDirStr("/a"), kSyscallErrorExists);
  EXPECT_EQ(MkDirStr("/"), kSyscallErrorExists);
  EXPECT_EQ(MkDirStr("/a/."), kSyscallErrorExists);
  EXPECT_EQ(MkDirStr("/a/.."), kSyscallErrorExists);
  ASSERT_EQ(MkDirStr("/a/b"), 0);

  FsStat root_st{};
  ASSERT_EQ(StatStr("/", &root_st), 0);
  EXPECT_EQ(root_st.nlink, 3);

  EXPECT_EQ(UnlinkStr("/"), kSyscallErrorBusy);
  EXPECT_EQ(UnlinkStr("/a/."), kSyscallErrorInvalid);
  EXPECT_EQ(UnlinkStr("/a/.."), kSyscallErrorInvalid);
  EXPECT_EQ(UnlinkStr("/a"), kSyscallErrorNotEmpty);

  ASSERT_EQ(UnlinkStr("/a/b"), 0);
  ASSERT_EQ(UnlinkStr("/a"), 0);
  ASSERT_EQ(StatStr("/", &root_st), 0);
  EXPECT_EQ(root_st.nlink, 2);
  EXPECT_EQ(alloc_.live_blocks(), 0);
  EXPECT_EQ(fs_->ActiveFnodeCount(), 1);
}

TEST_F(RamfsTest, OpenFlagsAndAccessModes) {
  uint64_t h = 0;
  FsNodeType type = FsNodeType::kNone;

  EXPECT_EQ(OpenStr(1, "/missing", kFsOpenRead, &h), kSyscallErrorNoEntry);
  EXPECT_EQ(OpenStr(1, "/f", kFsOpenRead | kFsOpenTruncate, &h),
            kSyscallErrorInvalid);
  EXPECT_EQ(OpenStr(1, "/f", kFsOpenWrite | kFsOpenExclusive, &h),
            kSyscallErrorInvalid);
  EXPECT_EQ(OpenStr(1, "/f", kFsOpenAccessMask | kFsOpenCreate, &h),
            kSyscallErrorInvalid);
  EXPECT_EQ(OpenStr(1, "/f", kFsOpenWrite | kFsOpenCreate | (1ULL << 7), &h),
            kSyscallErrorInvalid);
  EXPECT_EQ(OpenStr(0, "/f", kFsOpenWrite | kFsOpenCreate, &h),
            kSyscallErrorInvalid);
  EXPECT_EQ(OpenStr(1, "/f", kFsOpenCreate | kFsOpenDirectory, &h),
            kSyscallErrorIsDir);
  EXPECT_EQ(fs_->ActiveFnodeCount(), 1);

  ASSERT_EQ(OpenStr(1,                                                    //
                    "/f",                                                 //
                    kFsOpenReadWrite | kFsOpenCreate | kFsOpenExclusive,  //
                    &h,                                                   //
                    &type),
            0);
  EXPECT_EQ(type, FsNodeType::kRegular);
  EXPECT_EQ(
      OpenStr(1, "/f", kFsOpenReadWrite | kFsOpenCreate | kFsOpenExclusive, &h),
      kSyscallErrorExists);

  ASSERT_EQ(fs_->Write(1, h, "hello", 5), 5);
  ASSERT_EQ(fs_->Close(1, h), 0);

  // Write-only cannot read; read-only cannot write.
  ASSERT_EQ(OpenStr(1, "/f", kFsOpenWrite | kFsOpenAppend, &h), 0);
  char buf[16]{};
  EXPECT_EQ(fs_->Read(1, h, buf, 5), kSyscallErrorBadFd);
  ASSERT_EQ(fs_->Seek(1, h, 0, kFsSeekSet), 0);
  // A zero-length write doesn't move to the end, like on Linux.
  EXPECT_EQ(fs_->Write(1, h, "", 0), 0);
  EXPECT_EQ(fs_->Seek(1, h, 0, kFsSeekCur), 0);
  ASSERT_EQ(fs_->Write(1, h, "!", 1), 1);  // O_APPEND writes at end (offset 5).
  EXPECT_EQ(fs_->Seek(1, h, 0, kFsSeekCur), 6);
  ASSERT_EQ(fs_->Close(1, h), 0);

  ASSERT_EQ(OpenStr(1, "/f", kFsOpenRead, &h), 0);
  EXPECT_EQ(fs_->Write(1, h, "x", 1), kSyscallErrorBadFd);
  EXPECT_EQ(OpenStr(1, "/f", kFsOpenRead | kFsOpenDirectory, &h),
            kSyscallErrorNotDir);
  ASSERT_EQ(fs_->Read(1, h, buf, sizeof(buf)), 6);
  EXPECT_EQ(std::string(buf, 6), "hello!");
  FsDirEntry entry{};
  EXPECT_EQ(fs_->ReadDir(1, h, &entry, 1), kSyscallErrorNotDir);
  ASSERT_EQ(fs_->Close(1, h), 0);

  // Truncate frees blocks and resets size to 0.
  ASSERT_GT(alloc_.live_blocks(), 0);
  ASSERT_EQ(OpenStr(1, "/f", kFsOpenWrite | kFsOpenTruncate, &h), 0);
  ASSERT_EQ(fs_->Close(1, h), 0);
  FsStat st{};
  ASSERT_EQ(StatStr("/f", &st), 0);
  EXPECT_EQ(st.size, 0);

  // Directories reject write access, create, truncate and append, and file
  // Read/Write.
  ASSERT_EQ(MkDirStr("/d"), 0);
  EXPECT_EQ(OpenStr(1, "/d", kFsOpenWrite, &h), kSyscallErrorIsDir);
  EXPECT_EQ(OpenStr(1, "/d", kFsOpenReadWrite, &h), kSyscallErrorIsDir);
  EXPECT_EQ(OpenStr(1, "/d", kFsOpenRead | kFsOpenCreate, &h),
            kSyscallErrorIsDir);
  EXPECT_EQ(OpenStr(1, "/d", kFsOpenWrite | kFsOpenTruncate, &h),
            kSyscallErrorIsDir);
  EXPECT_EQ(OpenStr(1, "/d", kFsOpenRead | kFsOpenAppend, &h),
            kSyscallErrorIsDir);
  ASSERT_EQ(OpenStr(1, "/d", kFsOpenRead | kFsOpenDirectory, &h, &type), 0);
  EXPECT_EQ(type, FsNodeType::kDirectory);
  EXPECT_EQ(fs_->Read(1, h, buf, 4), kSyscallErrorIsDir);
  EXPECT_EQ(fs_->Write(1, h, "x", 1), kSyscallErrorIsDir);
  ASSERT_EQ(fs_->Close(1, h), 0);

  ASSERT_EQ(UnlinkStr("/f"), 0);
  ASSERT_EQ(UnlinkStr("/d"), 0);
  EXPECT_EQ(alloc_.live_blocks(), 0);
}

TEST_F(RamfsTest, TruncateLeavesOtherHandlesPastTheEndOfFile) {
  // `writer` writes a block and 5 bytes, and `reader` reads the block.
  uint64_t writer = 0;
  uint64_t reader = 0;
  ASSERT_EQ(OpenStr(1, "/f", kFsOpenReadWrite | kFsOpenCreate, &writer), 0);
  ASSERT_EQ(OpenStr(1, "/f", kFsOpenRead, &reader), 0);
  std::vector<char> buf(static_cast<size_t>(kBlockSize), 'o');
  ASSERT_EQ(fs_->Write(1, writer, buf.data(), kBlockSize), kBlockSize);
  ASSERT_EQ(fs_->Write(1, writer, "old!!", 5), 5);
  ASSERT_EQ(fs_->Read(1, reader, buf.data(), kBlockSize), kBlockSize);
  EXPECT_EQ(alloc_.live_blocks(), 3);  // The root's DirBlock and 2 of `/f`.

  // Another client's Open() truncates the file and frees its blocks. Both
  // handles keep their offsets, past the end of the file now.
  uint64_t truncating = 0;
  ASSERT_EQ(OpenStr(2, "/f", kFsOpenWrite | kFsOpenTruncate, &truncating), 0);
  ASSERT_EQ(fs_->Close(2, truncating), 0);
  FsStat st{};
  ASSERT_EQ(StatStr("/f", &st), 0);
  EXPECT_EQ(st.size, 0);
  EXPECT_EQ(alloc_.live_blocks(), 1);
  EXPECT_EQ(fs_->Seek(1, writer, 0, kFsSeekCur), kBlockSize + 5);
  EXPECT_EQ(fs_->Seek(1, reader, 0, kFsSeekCur), kBlockSize);

  // `reader` reads nothing and stays where it is.
  EXPECT_EQ(fs_->Read(1, reader, buf.data(), kBlockSize), 0);
  EXPECT_EQ(fs_->Seek(1, reader, 0, kFsSeekCur), kBlockSize);

  // `writer` writes after a hole of zeros, whose first block takes no space.
  ASSERT_EQ(fs_->Write(1, writer, "new", 3), 3);
  ASSERT_EQ(StatStr("/f", &st), 0);
  EXPECT_EQ(st.size, kBlockSize + 8);
  EXPECT_EQ(alloc_.live_blocks(), 2);
  ASSERT_EQ(fs_->Read(1, reader, buf.data(), kBlockSize), 8);
  EXPECT_EQ(std::string(buf.data(), 8), std::string(5, '\0') + "new");
  ASSERT_EQ(fs_->Seek(1, reader, 0, kFsSeekSet), 0);
  ASSERT_EQ(fs_->Read(1, reader, buf.data(), kBlockSize), kBlockSize);
  EXPECT_THAT(buf, t::Each('\0'));

  ASSERT_EQ(fs_->Close(1, writer), 0);
  ASSERT_EQ(fs_->Close(1, reader), 0);
  ASSERT_EQ(UnlinkStr("/f"), 0);
  EXPECT_EQ(alloc_.live_blocks(), 0);
}

TEST_F(RamfsTest, HandlesAreGenerationCheckedAndOwnedByOneClient) {
  ASSERT_EQ(MkDirStr("/d"), 0);
  uint64_t stale = 0;
  ASSERT_EQ(OpenStr(1, "/d/f", kFsOpenReadWrite | kFsOpenCreate, &stale), 0);
  ASSERT_EQ(fs_->Close(1, stale), 0);

  // A closed handle is stale.
  char buf[8]{};
  FsDirEntry entry{};
  EXPECT_EQ(fs_->Close(1, stale), kSyscallErrorBadFd);
  EXPECT_EQ(fs_->Read(1, stale, buf, 1), kSyscallErrorBadFd);
  EXPECT_EQ(fs_->Write(1, stale, "x", 1), kSyscallErrorBadFd);
  EXPECT_EQ(fs_->Seek(1, stale, 0, kFsSeekSet), kSyscallErrorBadFd);

  // The next Open() reuses the slot with a new generation, so the stale
  // handle stays stale.
  uint64_t dir_h = 0;
  ASSERT_EQ(OpenStr(1, "/d", kFsOpenRead | kFsOpenDirectory, &dir_h), 0);
  EXPECT_EQ(dir_h & 0xFFFF, stale & 0xFFFF);
  EXPECT_NE(dir_h, stale);
  EXPECT_EQ(fs_->ReadDir(1, stale, &entry, 1), kSyscallErrorBadFd);
  EXPECT_EQ(fs_->Close(1, stale), kSyscallErrorBadFd);
  EXPECT_EQ(fs_->ActiveHandleCount(), 1);

  // Handles with a zero or out-of-range slot never name an open file.
  EXPECT_EQ(fs_->Close(1, 0), kSyscallErrorBadFd);
  EXPECT_EQ(fs_->Close(1, dir_h & ~0xFFFFULL), kSyscallErrorBadFd);
  EXPECT_EQ(fs_->Close(1, (dir_h & ~0xFFFFULL) | (kMaxOpenHandles + 1)),
            kSyscallErrorBadFd);

  // Another client can't use client 1's handles, and trying doesn't affect
  // them.
  uint64_t file_h = 0;
  ASSERT_EQ(OpenStr(1, "/d/f", kFsOpenReadWrite, &file_h), 0);
  ASSERT_EQ(fs_->Write(1, file_h, "abc", 3), 3);
  EXPECT_EQ(fs_->Read(2, file_h, buf, 1), kSyscallErrorBadFd);
  EXPECT_EQ(fs_->Write(2, file_h, "x", 1), kSyscallErrorBadFd);
  EXPECT_EQ(fs_->Seek(2, file_h, 0, kFsSeekSet), kSyscallErrorBadFd);
  EXPECT_EQ(fs_->ReadDir(2, dir_h, &entry, 1), kSyscallErrorBadFd);
  EXPECT_EQ(fs_->Close(2, file_h), kSyscallErrorBadFd);
  EXPECT_EQ(fs_->Close(2, dir_h), kSyscallErrorBadFd);
  EXPECT_EQ(fs_->Close(0, file_h), kSyscallErrorBadFd);
  fs_->CloseAllForClient(2);
  EXPECT_EQ(fs_->ActiveHandleCount(), 2);
  EXPECT_EQ(fs_->Seek(1, file_h, 0, kFsSeekCur), 3);
  ASSERT_EQ(fs_->ReadDir(1, dir_h, &entry, 1), 1);
  EXPECT_EQ(std::string(entry.name, static_cast<size_t>(entry.name_len)), "f");

  ASSERT_EQ(fs_->Close(1, file_h), 0);
  ASSERT_EQ(fs_->Close(1, dir_h), 0);
  EXPECT_EQ(fs_->ActiveHandleCount(), 0);
}

TEST_F(RamfsTest, FullHandleTableFailsWithFileTableFullAndChangesNothing) {
  uint64_t h = 0;
  ASSERT_EQ(OpenStr(1, "/f", kFsOpenWrite | kFsOpenCreate, &h), 0);
  ASSERT_EQ(fs_->Write(1, h, "keep", 4), 4);
  for (int i = 1; i < kMaxOpenHandles; ++i) {
    ASSERT_EQ(OpenStr(1, "/f", kFsOpenRead, &h), 0);
  }
  ASSERT_EQ(fs_->ActiveHandleCount(), kMaxOpenHandles);
  const uint64_t last = h;

  // The table is shared by all clients, and a failed Open() neither creates
  // nor truncates a file.
  EXPECT_EQ(OpenStr(2, "/f", kFsOpenRead, &h), kSyscallErrorFileTableFull);
  EXPECT_EQ(OpenStr(1, "/g", kFsOpenWrite | kFsOpenCreate, &h),
            kSyscallErrorFileTableFull);
  EXPECT_EQ(OpenStr(1, "/f", kFsOpenWrite | kFsOpenTruncate, &h),
            kSyscallErrorFileTableFull);
  FsStat st{};
  EXPECT_EQ(StatStr("/g", &st), kSyscallErrorNoEntry);
  ASSERT_EQ(StatStr("/f", &st), 0);
  EXPECT_EQ(st.size, 4);
  EXPECT_EQ(fs_->ActiveFnodeCount(), 2);

  // Closing a handle makes room for one more.
  ASSERT_EQ(fs_->Close(1, last), 0);
  ASSERT_EQ(OpenStr(2, "/g", kFsOpenWrite | kFsOpenCreate, &h), 0);
  EXPECT_EQ(OpenStr(2, "/g", kFsOpenRead, &h), kSyscallErrorFileTableFull);
  fs_->CloseAllForClient(1);
  fs_->CloseAllForClient(2);
  EXPECT_EQ(fs_->ActiveHandleCount(), 0);
}

TEST_F(RamfsTest, FullFnodeTableFailsWithNoSpace) {
  // The root takes one of the `kMaxFnodes` fnodes.
  for (int i = 1; i < kMaxFnodes; ++i) {
    ASSERT_EQ(CreateFile("/f" + std::to_string(i)), 0);
  }
  ASSERT_EQ(fs_->ActiveFnodeCount(), kMaxFnodes);
  const int blocks = alloc_.live_blocks();

  uint64_t h = 0;
  EXPECT_EQ(OpenStr(1, "/extra", kFsOpenWrite | kFsOpenCreate, &h),
            kSyscallErrorNoSpace);
  EXPECT_EQ(MkDirStr("/extra"), kSyscallErrorNoSpace);
  EXPECT_EQ(fs_->ActiveHandleCount(), 0);
  EXPECT_EQ(alloc_.live_blocks(), blocks);
  FsStat st{};
  EXPECT_EQ(StatStr("/extra", &st), kSyscallErrorNoEntry);

  // Existing files still open, and unlinking one frees its fnode.
  ASSERT_EQ(OpenStr(1, "/f1", kFsOpenRead, &h), 0);
  ASSERT_EQ(fs_->Close(1, h), 0);
  ASSERT_EQ(UnlinkStr("/f1"), 0);
  ASSERT_EQ(MkDirStr("/extra"), 0);
  EXPECT_EQ(fs_->ActiveFnodeCount(), kMaxFnodes);
}

TEST_F(RamfsTest, FailedDirectoryInsertRollsBackTheNewFnode) {
  ASSERT_EQ(MkDirStr("/d"), 0);
  // Only the root has a `DirBlock` (the empty `/d` has none), and no more
  // blocks fit.
  ASSERT_EQ(alloc_.live_blocks(), 1);
  alloc_.set_max_blocks(1);

  uint64_t h = 0;
  EXPECT_EQ(OpenStr(1, "/d/f", kFsOpenWrite | kFsOpenCreate, &h),
            kSyscallErrorNoSpace);
  EXPECT_EQ(MkDirStr("/d/sub"), kSyscallErrorNoSpace);
  EXPECT_EQ(fs_->ActiveFnodeCount(), 2);
  EXPECT_EQ(fs_->ActiveHandleCount(), 0);
  EXPECT_EQ(alloc_.live_blocks(), 1);
  FsStat st{};
  EXPECT_EQ(StatStr("/d/f", &st), kSyscallErrorNoEntry);
  EXPECT_EQ(StatStr("/d/sub", &st), kSyscallErrorNoEntry);
  ASSERT_EQ(StatStr("/d", &st), 0);
  EXPECT_EQ(st.nlink, 2);
  EXPECT_EQ(st.size, 0);

  // With room, both work.
  alloc_.set_max_blocks(-1);
  ASSERT_EQ(CreateFile("/d/f"), 0);
  ASSERT_EQ(MkDirStr("/d/sub"), 0);
  EXPECT_EQ(fs_->ActiveFnodeCount(), 4);
  ASSERT_EQ(StatStr("/d", &st), 0);
  EXPECT_EQ(st.nlink, 3);
}

TEST_F(RamfsTest, DirectSingleIndirectDoubleIndirectAndSparseHoles) {
  uint64_t h = 0;
  ASSERT_EQ(OpenStr(1, "/big", kFsOpenReadWrite | kFsOpenCreate, &h), 0);

  // Write across direct blocks (0..11) and into single-indirect (12..14).
  std::vector<uint8_t> pattern(static_cast<size_t>(kSyscallFsMaxIoSize));
  const int64_t sequential_blocks = 15;
  for (int64_t b = 0; b < sequential_blocks; ++b) {
    for (int i = 0; i < static_cast<int>(pattern.size()); ++i) {
      pattern[static_cast<size_t>(i)] =
          static_cast<uint8_t>((b * 31 + i) & 0xFF);
    }
    ASSERT_EQ(fs_->Write(1, h, pattern.data(), kSyscallFsMaxIoSize),
              kSyscallFsMaxIoSize);
  }

  // Seek far into the double-indirect region (e.g. 24 MiB - 100 bytes) without
  // allocating intermediate sparse hole blocks.
  const int64_t high_off = (24LL << 20) - 100;
  ASSERT_EQ(fs_->Seek(1, h, high_off, kFsSeekSet), high_off);
  const std::string tail_msg = "double-indirect-sparse-tail-payload";
  ASSERT_EQ(
      fs_->Write(1, h, tail_msg.data(), static_cast<int64_t>(tail_msg.size())),
      static_cast<int64_t>(tail_msg.size()));

  // Sparse verification: 1 root DirBlock + 15 data blocks + 1 single_indirect +
  // 1 double_indirect L1 + 1 double_indirect L2 + 1 tail data block = 20 blocks
  // total for a 24 MiB file.
  EXPECT_EQ(alloc_.live_blocks(), 20);

  // Read a sparse hole in the middle: must return all zeros.
  const int64_t hole_off = 8LL << 20;
  ASSERT_EQ(fs_->Seek(1, h, hole_off, kFsSeekSet), hole_off);
  std::vector<uint8_t> read_buf(512, 0xAA);
  ASSERT_EQ(fs_->Read(1, h, read_buf.data(), 512), 512);
  for (const uint8_t byte : read_buf) {
    EXPECT_EQ(byte, 0u);
  }

  // Read across the hole-to-data boundary around `high_off`.
  ASSERT_EQ(fs_->Seek(1, h, high_off - 16, kFsSeekSet), high_off - 16);
  std::vector<uint8_t> cross(16 + tail_msg.size(), 0xFF);
  ASSERT_EQ(fs_->Read(1, h, cross.data(), static_cast<int64_t>(cross.size())),
            static_cast<int64_t>(cross.size()));
  for (int i = 0; i < 16; ++i) {
    EXPECT_EQ(cross[static_cast<size_t>(i)], 0u);
  }
  EXPECT_EQ(std::string(reinterpret_cast<const char*>(cross.data() + 16),
                        tail_msg.size()),
            tail_msg);

  // Verify direct + single-indirect blocks at start are intact.
  ASSERT_EQ(fs_->Seek(1, h, 0, kFsSeekSet), 0);
  std::vector<uint8_t> verify_blk(static_cast<size_t>(kSyscallFsMaxIoSize));
  for (int64_t b = 0; b < sequential_blocks; ++b) {
    ASSERT_EQ(fs_->Read(1, h, verify_blk.data(), kSyscallFsMaxIoSize),
              kSyscallFsMaxIoSize);
    for (int i = 0; i < static_cast<int>(verify_blk.size()); ++i) {
      const uint8_t expected = static_cast<uint8_t>((b * 31 + i) & 0xFF);
      ASSERT_EQ(verify_blk[static_cast<size_t>(i)], expected)
          << "block " << b << " byte " << i;
    }
  }

  // Max file size boundary check (`kFsMaxFileSize = 32 MiB`).
  EXPECT_EQ(fs_->Seek(1, h, kFsMaxFileSize + 1, kFsSeekSet),
            kSyscallErrorInvalid);
  ASSERT_EQ(fs_->Seek(1, h, kFsMaxFileSize, kFsSeekSet), kFsMaxFileSize);
  EXPECT_EQ(fs_->Write(1, h, "x", 1), kSyscallErrorFileTooBig);

  ASSERT_EQ(fs_->Close(1, h), 0);
  ASSERT_EQ(UnlinkStr("/big"), 0);
  EXPECT_EQ(alloc_.live_blocks(), 0);
}

TEST_F(RamfsTest, FileSizeLimitShortWriteFileTooBigAndZeroLengthWrites) {
  uint64_t h = 0;
  ASSERT_EQ(OpenStr(1, "/f", kFsOpenReadWrite | kFsOpenCreate, &h), 0);

  // A write stops at `kFsMaxFileSize`.
  ASSERT_EQ(fs_->Seek(1, h, kFsMaxFileSize - 3, kFsSeekSet),
            kFsMaxFileSize - 3);
  EXPECT_EQ(fs_->Write(1, h, "0123456789", 10), 3);
  EXPECT_EQ(fs_->Seek(1, h, 0, kFsSeekCur), kFsMaxFileSize);
  FsStat st{};
  ASSERT_EQ(StatStr("/f", &st), 0);
  EXPECT_EQ(st.size, kFsMaxFileSize);
  ASSERT_EQ(fs_->Seek(1, h, -3, kFsSeekEnd), kFsMaxFileSize - 3);
  char buf[8]{};
  ASSERT_EQ(fs_->Read(1, h, buf, sizeof(buf)), 3);
  EXPECT_EQ(std::string(buf, 3), "012");

  // At the limit, a write of at least one byte fails with EFBIG, also with
  // `kFsOpenAppend`, and a zero-length one returns 0.
  EXPECT_EQ(fs_->Write(1, h, "x", 1), kSyscallErrorFileTooBig);
  EXPECT_EQ(fs_->Write(1, h, nullptr, 0), 0);
  ASSERT_EQ(fs_->Close(1, h), 0);
  ASSERT_EQ(OpenStr(1, "/f", kFsOpenWrite | kFsOpenAppend, &h), 0);
  EXPECT_EQ(fs_->Write(1, h, "x", 1), kSyscallErrorFileTooBig);
  ASSERT_EQ(fs_->Close(1, h), 0);
  ASSERT_EQ(StatStr("/f", &st), 0);
  EXPECT_EQ(st.size, kFsMaxFileSize);

  // A zero-length write past the end doesn't extend the file.
  ASSERT_EQ(OpenStr(1, "/g", kFsOpenWrite | kFsOpenCreate, &h), 0);
  ASSERT_EQ(fs_->Seek(1, h, 100, kFsSeekSet), 100);
  EXPECT_EQ(fs_->Write(1, h, "", 0), 0);
  EXPECT_EQ(fs_->Seek(1, h, 0, kFsSeekCur), 100);
  ASSERT_EQ(StatStr("/g", &st), 0);
  EXPECT_EQ(st.size, 0);
  ASSERT_EQ(fs_->Close(1, h), 0);

  ASSERT_EQ(UnlinkStr("/f"), 0);
  ASSERT_EQ(UnlinkStr("/g"), 0);
  EXPECT_EQ(alloc_.live_blocks(), 0);
}

TEST_F(RamfsTest, SeekWhenceBoundsAndOverflow) {
  uint64_t h = 0;
  ASSERT_EQ(OpenStr(1, "/f", kFsOpenReadWrite | kFsOpenCreate, &h), 0);
  ASSERT_EQ(fs_->Write(1, h, "0123456789", 10), 10);

  EXPECT_EQ(fs_->Seek(1, h, -3, kFsSeekEnd), 7);
  EXPECT_EQ(fs_->Seek(1, h, 2, kFsSeekCur), 9);
  EXPECT_EQ(fs_->Seek(1, h, -9, kFsSeekCur), 0);
  EXPECT_EQ(fs_->Seek(1, h, 5, kFsSeekEnd), 15);
  EXPECT_EQ(fs_->Seek(1, h, kFsMaxFileSize - 10, kFsSeekEnd), kFsMaxFileSize);

  // A failed Seek() keeps the offset.
  ASSERT_EQ(fs_->Seek(1, h, 4, kFsSeekSet), 4);
  EXPECT_EQ(fs_->Seek(1, h, -1, kFsSeekSet), kSyscallErrorInvalid);
  EXPECT_EQ(fs_->Seek(1, h, -5, kFsSeekCur), kSyscallErrorInvalid);
  EXPECT_EQ(fs_->Seek(1, h, -11, kFsSeekEnd), kSyscallErrorInvalid);
  EXPECT_EQ(fs_->Seek(1, h, kFsMaxFileSize + 1, kFsSeekSet),
            kSyscallErrorInvalid);
  EXPECT_EQ(fs_->Seek(1, h, kFsMaxFileSize - 9, kFsSeekEnd),
            kSyscallErrorInvalid);
  EXPECT_EQ(fs_->Seek(1, h, kInt64Max, kFsSeekCur), kSyscallErrorInvalid);
  EXPECT_EQ(fs_->Seek(1, h, kInt64Max, kFsSeekEnd), kSyscallErrorInvalid);
  EXPECT_EQ(fs_->Seek(1, h, kInt64Min, kFsSeekCur), kSyscallErrorInvalid);
  EXPECT_EQ(fs_->Seek(1, h, kInt64Min, kFsSeekSet), kSyscallErrorInvalid);
  EXPECT_EQ(fs_->Seek(1, h, 0, kFsSeekEnd + 1), kSyscallErrorInvalid);
  EXPECT_EQ(fs_->Seek(1, h, 0, -1), kSyscallErrorInvalid);
  EXPECT_EQ(fs_->Seek(1, h, 0, kFsSeekCur), 4);
  ASSERT_EQ(fs_->Close(1, h), 0);

  // A directory accepts only `(0, kFsSeekSet)` and `(0, kFsSeekCur)`.
  ASSERT_EQ(MkDirStr("/d"), 0);
  ASSERT_EQ(CreateFile("/d/a"), 0);
  ASSERT_EQ(CreateFile("/d/b"), 0);
  uint64_t dir_h = 0;
  ASSERT_EQ(OpenStr(1, "/d", kFsOpenRead | kFsOpenDirectory, &dir_h), 0);
  EXPECT_EQ(fs_->Seek(1, dir_h, 0, kFsSeekCur), 0);
  EXPECT_EQ(ReadDirNames(dir_h, 1), std::vector<std::string>{"a"});
  const int64_t cookie = fs_->Seek(1, dir_h, 0, kFsSeekCur);
  EXPECT_GT(cookie, 0);
  EXPECT_EQ(fs_->Seek(1, dir_h, cookie, kFsSeekSet), kSyscallErrorInvalid);
  EXPECT_EQ(fs_->Seek(1, dir_h, 1, kFsSeekCur), kSyscallErrorInvalid);
  EXPECT_EQ(fs_->Seek(1, dir_h, 0, kFsSeekEnd), kSyscallErrorInvalid);
  EXPECT_EQ(fs_->Seek(1, dir_h, 0, kFsSeekCur), cookie);
  EXPECT_EQ(ReadDirNames(dir_h, 1), std::vector<std::string>{"b"});
  EXPECT_EQ(fs_->Seek(1, dir_h, 0, kFsSeekSet), 0);
  EXPECT_EQ(ReadDirNames(dir_h, 2), (std::vector<std::string>{"a", "b"}));
  ASSERT_EQ(fs_->Close(1, dir_h), 0);
}

TEST_F(RamfsTest, ReadDirPaginationRewindAndDirBlockReclamation) {
  ASSERT_EQ(MkDirStr("/many"), 0);
  constexpr int kSlots = DirBlock::kSlotsPerBlock;
  constexpr int kFileCount = 2 * kSlots + 2;  // Spans 3 `DirBlock`s.
  std::set<std::string> expected;
  for (int i = 0; i < kFileCount; ++i) {
    const std::string name = "entry_" + std::to_string(i);
    expected.insert(name);
    uint64_t fh = 0;
    ASSERT_EQ(OpenStr(1, "/many/" + name, kFsOpenWrite | kFsOpenCreate, &fh),
              0);
    ASSERT_EQ(fs_->Close(1, fh), 0);
  }
  const int blocks_with_all = alloc_.live_blocks();
  EXPECT_EQ(blocks_with_all, 4);  // 1 root DirBlock + 3 `/many` DirBlocks.

  uint64_t dir_h = 0;
  ASSERT_EQ(OpenStr(1, "/many", kFsOpenRead | kFsOpenDirectory, &dir_h), 0);

  auto collect_all = [&]() {
    std::set<std::string> seen;
    for (;;) {
      FsDirEntry page[7]{};
      const int64_t n = fs_->ReadDir(1, dir_h, page, 7);
      EXPECT_GE(n, 0);
      if (n <= 0) {
        break;
      }
      for (int64_t i = 0; i < n; ++i) {
        EXPECT_EQ(page[i].type, FsNodeType::kRegular);
        EXPECT_GT(page[i].fnode_id, 1u);
        for (int64_t p = page[i].name_len; p <= kFsMaxNameLength; ++p) {
          EXPECT_EQ(page[i].name[p], '\0');
        }
        seen.emplace(page[i].name, static_cast<size_t>(page[i].name_len));
      }
    }
    return seen;
  };

  EXPECT_EQ(collect_all(), expected);
  // At EOF, another ReadDir returns 0; Seek(0, kFsSeekSet) rewinds.
  FsDirEntry one{};
  EXPECT_EQ(fs_->ReadDir(1, dir_h, &one, 1), 0);
  ASSERT_EQ(fs_->Seek(1, dir_h, 0, kFsSeekSet), 0);
  EXPECT_EQ(collect_all(), expected);

  // Unlink the middle `DirBlock`'s entries: must immediately free that
  // `DirBlock`.
  for (int i = kSlots; i < 2 * kSlots; ++i) {
    const std::string name = "entry_" + std::to_string(i);
    expected.erase(name);
    ASSERT_EQ(UnlinkStr("/many/" + name), 0);
  }
  EXPECT_EQ(alloc_.live_blocks(), 3);

  ASSERT_EQ(fs_->Seek(1, dir_h, 0, kFsSeekSet), 0);
  EXPECT_EQ(collect_all(), expected);
  ASSERT_EQ(fs_->Close(1, dir_h), 0);

  for (const std::string& name : expected) {
    ASSERT_EQ(UnlinkStr("/many/" + name), 0);
  }
  // `/many` now has 0 entries -> all of `/many`'s DirBlocks are reclaimed.
  EXPECT_EQ(alloc_.live_blocks(), 1);
  ASSERT_EQ(UnlinkStr("/many"), 0);
  EXPECT_EQ(alloc_.live_blocks(), 0);
}

TEST_F(RamfsTest, ReadDirAcrossChangesBetweenPages) {
  ASSERT_EQ(MkDirStr("/d"), 0);
  ASSERT_EQ(MkDirStr("/other"), 0);
  for (int i = 0; i < 10; ++i) {
    ASSERT_EQ(CreateFile("/d/e" + std::to_string(i)), 0);
  }
  uint64_t dir_h = 0;
  ASSERT_EQ(OpenStr(1, "/d", kFsOpenRead | kFsOpenDirectory, &dir_h), 0);
  EXPECT_EQ(ReadDirNames(dir_h, 3),
            (std::vector<std::string>{"e0", "e1", "e2"}));

  // Between pages: unlink a returned (e1) and a pending (e4) entry, create an
  // entry (in e1's slot), rename a returned (e0) and a pending (e5) entry
  // within the directory (into e4's and e0's slots), move a pending entry (e6)
  // out of it, and replace both a returned (e2 <- e8) and a pending (e7 <- e9)
  // entry with Rename().
  ASSERT_EQ(UnlinkStr("/d/e1"), 0);
  ASSERT_EQ(UnlinkStr("/d/e4"), 0);
  ASSERT_EQ(CreateFile("/d/new"), 0);
  ASSERT_EQ(RenameStr("/d/e0", "/d/r0"), 0);
  ASSERT_EQ(RenameStr("/d/e5", "/d/r5"), 0);
  ASSERT_EQ(RenameStr("/d/e6", "/other/e6"), 0);
  ASSERT_EQ(RenameStr("/d/e8", "/d/e2"), 0);
  ASSERT_EQ(RenameStr("/d/e9", "/d/e7"), 0);

  // The pending entry that stayed unchanged (e3) appears once, removed ones
  // don't, and created or renamed entries (including Rename() replacements)
  // follow in the order they were added, whatever their slots.
  EXPECT_EQ(ReadDirNames(dir_h, kSyscallFsMaxDirEntries),
            (std::vector<std::string>{"e3", "new", "r0", "r5", "e2", "e7"}));
  EXPECT_EQ(ReadDirNames(dir_h, kSyscallFsMaxDirEntries),
            std::vector<std::string>{});

  ASSERT_EQ(fs_->Seek(1, dir_h, 0, kFsSeekSet), 0);
  EXPECT_EQ(ReadDirNames(dir_h, kSyscallFsMaxDirEntries),
            (std::vector<std::string>{"e3", "new", "r0", "r5", "e2", "e7"}));
  FsStat st{};
  ASSERT_EQ(StatStr("/d", &st), 0);
  EXPECT_EQ(st.size, 6 * static_cast<int64_t>(sizeof(FsDirEntry)));
  ASSERT_EQ(fs_->Close(1, dir_h), 0);
}

TEST_F(RamfsTest, ReadDirContinuesAfterTheDirectoryIsRenamed) {
  ASSERT_EQ(MkDirStr("/d"), 0);
  ASSERT_EQ(MkDirStr("/other"), 0);
  ASSERT_EQ(MkDirStr("/other/empty"), 0);
  std::vector<std::string> names;
  for (int i = 0; i < 10; ++i) {
    names.push_back("e" + std::to_string(i));
    ASSERT_EQ(CreateFile("/d/" + names.back()), 0);
  }
  uint64_t dir_h = 0;
  ASSERT_EQ(OpenStr(1, "/d", kFsOpenRead | kFsOpenDirectory, &dir_h), 0);

  // Between pages, the directory moves within its parent, to another
  // directory, and onto an empty directory, which it replaces. Each entry
  // appears once, in order.
  std::vector<std::string> seen = ReadDirNames(dir_h, 3);
  for (const auto& [old_path, new_path] :
       std::vector<std::pair<std::string, std::string>>{
           {"/d", "/renamed"},
           {"/renamed", "/other/moved"},
           {"/other/moved", "/other/empty"},
       }) {
    ASSERT_EQ(RenameStr(old_path, new_path), 0);
    const std::vector<std::string> page = ReadDirNames(dir_h, 3);
    seen.insert(seen.end(), page.begin(), page.end());
  }
  EXPECT_EQ(seen, names);
  EXPECT_EQ(ReadDirNames(dir_h, kSyscallFsMaxDirEntries),
            std::vector<std::string>{});

  // The handle follows the directory to its new path.
  ASSERT_EQ(CreateFile("/other/empty/late"), 0);
  EXPECT_EQ(ReadDirNames(dir_h, kSyscallFsMaxDirEntries),
            std::vector<std::string>{"late"});
  ASSERT_EQ(fs_->Close(1, dir_h), 0);
}

TEST_F(RamfsTest, UnlinkedOrReplacedOpenDirectoryReadsAsEmpty) {
  ASSERT_EQ(MkDirStr("/gone"), 0);
  uint64_t gone_h = 0;
  ASSERT_EQ(OpenStr(1, "/gone", kFsOpenRead | kFsOpenDirectory, &gone_h), 0);
  ASSERT_EQ(UnlinkStr("/gone"), 0);
  EXPECT_EQ(fs_->ActiveFnodeCount(), 2);

  // A new directory at the same path is another one.
  ASSERT_EQ(MkDirStr("/gone"), 0);
  ASSERT_EQ(CreateFile("/gone/child"), 0);
  EXPECT_EQ(ReadDirNames(gone_h, 1), std::vector<std::string>{});
  ASSERT_EQ(fs_->Seek(1, gone_h, 0, kFsSeekSet), 0);
  EXPECT_EQ(ReadDirNames(gone_h, 1), std::vector<std::string>{});
  ASSERT_EQ(fs_->Close(1, gone_h), 0);
  EXPECT_EQ(fs_->ActiveFnodeCount(), 3);

  // So is a directory that a rename replaced.
  ASSERT_EQ(MkDirStr("/replaced"), 0);
  uint64_t replaced_h = 0;
  ASSERT_EQ(
      OpenStr(1, "/replaced", kFsOpenRead | kFsOpenDirectory, &replaced_h), 0);
  ASSERT_EQ(UnlinkStr("/gone/child"), 0);
  ASSERT_EQ(RenameStr("/gone", "/replaced"), 0);
  ASSERT_EQ(CreateFile("/replaced/child"), 0);
  EXPECT_EQ(ReadDirNames(replaced_h, 1), std::vector<std::string>{});
  EXPECT_EQ(fs_->ActiveFnodeCount(), 4);
  ASSERT_EQ(fs_->Close(1, replaced_h), 0);
  EXPECT_EQ(fs_->ActiveFnodeCount(), 3);
}

TEST_F(RamfsTest, UnlinkWhileOpenAndClientDisconnectReclamation) {
  uint64_t h1 = 0;
  uint64_t h2 = 0;
  ASSERT_EQ(OpenStr(10, "/temp", kFsOpenReadWrite | kFsOpenCreate, &h1), 0);
  ASSERT_EQ(OpenStr(20, "/temp", kFsOpenRead, &h2), 0);
  ASSERT_EQ(fs_->Write(10, h1, "persists-until-last-close", 25), 25);

  // Client isolation: client 20 cannot use client 10's handle.
  char buf[32]{};
  EXPECT_EQ(fs_->Read(20, h1, buf, 4), kSyscallErrorBadFd);
  EXPECT_EQ(fs_->Close(20, h1), kSyscallErrorBadFd);

  // Unlink while open by both clients: path disappears immediately, but blocks
  // stay live and readable until both clients close/disconnect.
  ASSERT_EQ(UnlinkStr("/temp"), 0);
  FsStat st{};
  EXPECT_EQ(StatStr("/temp", &st), kSyscallErrorNoEntry);
  EXPECT_EQ(alloc_.live_blocks(), 1);
  ASSERT_EQ(fs_->Read(20, h2, buf, 25), 25);
  EXPECT_EQ(std::string(buf, 25), "persists-until-last-close");

  // Simulate client 20 crashing (`kClientDisconnect`) while client 10 still
  // has `/temp` open: only client 20's handles close.
  uint64_t h3 = 0;
  ASSERT_EQ(OpenStr(20, "/other", kFsOpenWrite | kFsOpenCreate, &h3), 0);
  fs_->CloseAllForClient(20);
  EXPECT_EQ(fs_->ActiveHandleCount(), 1);
  EXPECT_EQ(fs_->Read(20, h2, buf, 1), kSyscallErrorBadFd);
  EXPECT_EQ(fs_->Close(20, h3), kSyscallErrorBadFd);
  EXPECT_EQ(fs_->Seek(10, h1, 0, kFsSeekCur), 25);
  ASSERT_EQ(fs_->Seek(10, h1, 0, kFsSeekSet), 0);
  ASSERT_EQ(fs_->Read(10, h1, buf, 25), 25);
  EXPECT_EQ(std::string(buf, 25), "persists-until-last-close");
  EXPECT_EQ(alloc_.live_blocks(), 2);  // The root's DirBlock and `/temp`'s.

  // The last close frees the unlinked file.
  ASSERT_EQ(fs_->Close(10, h1), 0);
  EXPECT_EQ(alloc_.live_blocks(), 1);
  EXPECT_EQ(fs_->ActiveHandleCount(), 0);
  EXPECT_EQ(fs_->ActiveFnodeCount(), 2);
}

TEST_F(RamfsTest, RenameCrossDirOverwriteAndCycleDetection) {
  ASSERT_EQ(MkDirStr("/src"), 0);
  ASSERT_EQ(MkDirStr("/src/sub"), 0);
  ASSERT_EQ(MkDirStr("/dst"), 0);

  uint64_t fh = 0;
  ASSERT_EQ(OpenStr(1, "/src/sub/item", kFsOpenWrite | kFsOpenCreate, &fh), 0);
  ASSERT_EQ(fs_->Write(1, fh, "new-data", 8), 8);
  ASSERT_EQ(fs_->Close(1, fh), 0);

  // Cycle rejection: cannot move `/src` under `/src/sub/moved`.
  EXPECT_EQ(RenameStr("/src", "/src/sub/moved"), kSyscallErrorInvalid);
  EXPECT_EQ(RenameStr("/", "/dst/root"), kSyscallErrorBusy);

  // Cross-directory move of non-empty `/src/sub` -> `/dst/sub` updates `..` and
  // parent `nlink` counts.
  ASSERT_EQ(RenameStr("/src/sub", "/dst/sub"), 0);
  FsStat src_st{};
  FsStat dst_st{};
  FsStat via_dotdot{};
  ASSERT_EQ(StatStr("/src", &src_st), 0);
  ASSERT_EQ(StatStr("/dst", &dst_st), 0);
  ASSERT_EQ(StatStr("/dst/sub/..", &via_dotdot), 0);
  EXPECT_EQ(src_st.nlink, 2);
  EXPECT_EQ(dst_st.nlink, 3);
  EXPECT_EQ(via_dotdot.fnode_id, dst_st.fnode_id);

  // Atomic file overwrite while old target is held open by a reader.
  uint64_t old_reader = 0;
  ASSERT_EQ(
      OpenStr(1, "/target", kFsOpenReadWrite | kFsOpenCreate, &old_reader), 0);
  ASSERT_EQ(fs_->Write(1, old_reader, "old-value", 9), 9);
  ASSERT_EQ(fs_->Seek(1, old_reader, 0, kFsSeekSet), 0);

  // Same-fnode rename is a no-op.
  ASSERT_EQ(RenameStr("/target", "/./target"), 0);

  // Type mismatch rejections.
  EXPECT_EQ(RenameStr("/dst/sub/item", "/dst/sub"), kSyscallErrorIsDir);
  EXPECT_EQ(RenameStr("/dst/sub", "/target"), kSyscallErrorNotDir);

  ASSERT_EQ(RenameStr("/dst/sub/item", "/target"), 0);

  // Old open handle still reads `"old-value"`, while opening `"/target"` reads
  // `"new-data"`.
  char old_buf[16]{};
  ASSERT_EQ(fs_->Read(1, old_reader, old_buf, 9), 9);
  EXPECT_EQ(std::string(old_buf, 9), "old-value");
  ASSERT_EQ(fs_->Close(1, old_reader), 0);

  uint64_t new_reader = 0;
  ASSERT_EQ(OpenStr(1, "/target", kFsOpenRead, &new_reader), 0);
  char new_buf[16]{};
  ASSERT_EQ(fs_->Read(1, new_reader, new_buf, 8), 8);
  EXPECT_EQ(std::string(new_buf, 8), "new-data");
  ASSERT_EQ(fs_->Close(1, new_reader), 0);

  // Overwriting non-empty directory fails with ENOTEMPTY; overwriting empty
  // directory succeeds.
  ASSERT_EQ(MkDirStr("/empty_dir"), 0);
  ASSERT_EQ(MkDirStr("/other_dir"), 0);
  ASSERT_EQ(MkDirStr("/nonempty_dir"), 0);
  ASSERT_EQ(MkDirStr("/nonempty_dir/child"), 0);
  EXPECT_EQ(RenameStr("/other_dir", "/nonempty_dir"), kSyscallErrorNotEmpty);
  FsStat root_st{};
  ASSERT_EQ(StatStr("/", &root_st), 0);
  const int64_t root_nlink = root_st.nlink;
  const int fnodes = fs_->ActiveFnodeCount();
  ASSERT_EQ(RenameStr("/other_dir", "/empty_dir"), 0);
  // The replaced directory is gone: the root has one subdirectory less.
  ASSERT_EQ(StatStr("/", &root_st), 0);
  EXPECT_EQ(root_st.nlink, root_nlink - 1);
  EXPECT_EQ(fs_->ActiveFnodeCount(), fnodes - 1);
  FsStat st{};
  EXPECT_EQ(StatStr("/other_dir", &st), kSyscallErrorNoEntry);

  // Across directories, the old parent loses a subdirectory, and the new one
  // keeps its count: the empty `/dst/sub` replaces the empty `/src/victim`.
  ASSERT_EQ(MkDirStr("/src/victim"), 0);
  ASSERT_EQ(RenameStr("/dst/sub", "/src/victim"), 0);
  ASSERT_EQ(StatStr("/src", &src_st), 0);
  ASSERT_EQ(StatStr("/dst", &dst_st), 0);
  ASSERT_EQ(StatStr("/src/victim/..", &via_dotdot), 0);
  EXPECT_EQ(src_st.nlink, 3);
  EXPECT_EQ(dst_st.nlink, 2);
  EXPECT_EQ(via_dotdot.fnode_id, src_st.fnode_id);
  EXPECT_EQ(fs_->ActiveFnodeCount(), fnodes - 1);

  // Clean up everything and confirm zero leaked blocks.
  ASSERT_EQ(UnlinkStr("/target"), 0);
  ASSERT_EQ(UnlinkStr("/src/victim"), 0);
  ASSERT_EQ(UnlinkStr("/dst"), 0);
  ASSERT_EQ(UnlinkStr("/src"), 0);
  ASSERT_EQ(UnlinkStr("/empty_dir"), 0);
  ASSERT_EQ(UnlinkStr("/nonempty_dir/child"), 0);
  ASSERT_EQ(UnlinkStr("/nonempty_dir"), 0);
  EXPECT_EQ(alloc_.live_blocks(), 0);
  EXPECT_EQ(fs_->ActiveFnodeCount(), 1);
}

TEST_F(RamfsTest, RenameThroughTheSourceIsNotAMoveIntoItself) {
  ASSERT_EQ(MkDirStr("/a"), 0);
  ASSERT_EQ(MkDirStr("/a/sub"), 0);
  FsStat a_st{};
  ASSERT_EQ(StatStr("/a", &a_st), 0);

  // `/a/../b` passes through `/a` but names `/b`, outside of `/a`: the
  // directory moves, and the root keeps one subdirectory.
  ASSERT_EQ(RenameStr("/a", "/a/../b"), 0);
  FsStat st{};
  EXPECT_EQ(StatStr("/a", &st), kSyscallErrorNoEntry);
  ASSERT_EQ(StatStr("/b", &st), 0);
  EXPECT_EQ(st.fnode_id, a_st.fnode_id);
  ASSERT_EQ(StatStr("/b/sub/..", &st), 0);
  EXPECT_EQ(st.fnode_id, a_st.fnode_id);
  ASSERT_EQ(StatStr("/", &st), 0);
  EXPECT_EQ(st.nlink, 3);

  // A new path may also pass through a subdirectory of the source, but one
  // that leads back into the source is a move into itself.
  ASSERT_EQ(RenameStr("/b", "/b/sub/../../c"), 0);
  EXPECT_EQ(RenameStr("/c", "/c/sub/../d"), kSyscallErrorInvalid);
  ASSERT_EQ(StatStr("/c/sub", &st), 0);
  EXPECT_EQ(StatStr("/c/d", &st), kSyscallErrorNoEntry);
  EXPECT_EQ(fs_->ActiveFnodeCount(), 3);
}

TEST_F(RamfsTest, RenameOfRegularFileWithTrailingSlashFailsWithNotDir) {
  uint64_t h = 0;
  ASSERT_EQ(OpenStr(1, "/f", kFsOpenWrite | kFsOpenCreate, &h), 0);
  ASSERT_EQ(fs_->Write(1, h, "f", 1), 1);
  ASSERT_EQ(fs_->Close(1, h), 0);
  ASSERT_EQ(CreateFile("/g"), 0);

  // Neither path of a regular file may end with '/', whether or not the new
  // path exists, and the failures change nothing.
  EXPECT_EQ(RenameStr("/f", "/new/"), kSyscallErrorNotDir);
  EXPECT_EQ(RenameStr("/f", "/g/"), kSyscallErrorNotDir);
  EXPECT_EQ(RenameStr("/f/", "/new"), kSyscallErrorNotDir);
  FsStat st{};
  EXPECT_EQ(StatStr("/new", &st), kSyscallErrorNoEntry);
  ASSERT_EQ(StatStr("/f", &st), 0);
  EXPECT_EQ(st.size, 1);
  ASSERT_EQ(StatStr("/g", &st), 0);
  EXPECT_EQ(st.size, 0);
  EXPECT_EQ(fs_->ActiveFnodeCount(), 3);

  // A directory's paths may end with '/'.
  ASSERT_EQ(MkDirStr("/d"), 0);
  ASSERT_EQ(RenameStr("/d/", "/e/"), 0);
  ASSERT_EQ(StatStr("/e", &st), 0);
  EXPECT_EQ(st.type, FsNodeType::kDirectory);
}

TEST_F(RamfsTest, OutOfSpaceRollbackLeavesZeroLeakedBlocks) {
  uint64_t h = 0;
  ASSERT_EQ(OpenStr(1, "/oom", kFsOpenReadWrite | kFsOpenCreate, &h), 0);
  // Root DirBlock took 1 block (`live_blocks() == 1`).
  ASSERT_EQ(alloc_.live_blocks(), 1);

  // Cap at 2 blocks total: jumping into double-indirect requires L1 + L2 + data
  // (3 new blocks), so `EnsureFileBlock` allocates L1 (`live == 2`), fails on
  // L2, and must roll back L1 immediately.
  alloc_.set_max_blocks(2);
  const int64_t dbl_off = (4LL << 20);
  ASSERT_EQ(fs_->Seek(1, h, dbl_off, kFsSeekSet), dbl_off);
  EXPECT_EQ(fs_->Write(1, h, "x", 1), kSyscallErrorNoSpace);
  EXPECT_EQ(alloc_.live_blocks(), 1);

  // Cap at 3 blocks total: allocates L1 + L2 (`live == 3`), fails on data
  // block, and rolls back BOTH L2 and L1.
  alloc_.set_max_blocks(3);
  EXPECT_EQ(fs_->Write(1, h, "x", 1), kSyscallErrorNoSpace);
  EXPECT_EQ(alloc_.live_blocks(), 1);

  // Rename into a new directory whose `DirBlock` allocation fails leaves source
  // untouched.
  ASSERT_EQ(MkDirStr("/dest_dir"), 0);
  alloc_.set_max_blocks(1);
  EXPECT_EQ(RenameStr("/oom", "/dest_dir/oom"), kSyscallErrorNoSpace);
  FsStat st{};
  EXPECT_EQ(StatStr("/oom", &st), 0);

  alloc_.set_max_blocks(-1);
  ASSERT_EQ(fs_->Close(1, h), 0);
  ASSERT_EQ(UnlinkStr("/oom"), 0);
  ASSERT_EQ(UnlinkStr("/dest_dir"), 0);
  EXPECT_EQ(alloc_.live_blocks(), 0);
}

TEST_F(RamfsTest, OutOfSpaceShortWriteAndSingleIndirectRollback) {
  uint64_t h = 0;
  ASSERT_EQ(OpenStr(1, "/f", kFsOpenReadWrite | kFsOpenCreate, &h), 0);
  ASSERT_EQ(alloc_.live_blocks(), 1);  // The root's DirBlock.

  // Room for one more block: a write across blocks 0 and 1 stops at the end
  // of block 0.
  alloc_.set_max_blocks(2);
  const std::vector<char> data(static_cast<size_t>(kBlockSize), 'x');
  ASSERT_EQ(fs_->Seek(1, h, kBlockSize / 2, kFsSeekSet), kBlockSize / 2);
  EXPECT_EQ(fs_->Write(1, h, data.data(), kBlockSize), kBlockSize / 2);
  EXPECT_EQ(fs_->Seek(1, h, 0, kFsSeekCur), kBlockSize);
  FsStat st{};
  ASSERT_EQ(StatStr("/f", &st), 0);
  EXPECT_EQ(st.size, kBlockSize);
  // Without room for any of the bytes, a write fails and changes nothing.
  EXPECT_EQ(fs_->Write(1, h, data.data(), 1), kSyscallErrorNoSpace);
  EXPECT_EQ(fs_->Seek(1, h, 0, kFsSeekCur), kBlockSize);
  ASSERT_EQ(StatStr("/f", &st), 0);
  EXPECT_EQ(st.size, kBlockSize);

  // Block 12 is the first one that the single-indirect block maps. With room
  // for one block, the single-indirect block fits but the data block doesn't,
  // so the single-indirect block is freed again.
  const int64_t single_off = kDirectBlocks * kBlockSize;
  alloc_.set_max_blocks(3);
  ASSERT_EQ(fs_->Seek(1, h, single_off, kFsSeekSet), single_off);
  EXPECT_EQ(fs_->Write(1, h, "y", 1), kSyscallErrorNoSpace);
  EXPECT_EQ(alloc_.live_blocks(), 2);

  // With room for both, block 12 is written. Then block 13 gets no data
  // block, but the single-indirect block stays: block 12 uses it.
  alloc_.set_max_blocks(4);
  EXPECT_EQ(fs_->Write(1, h, "y", 1), 1);
  EXPECT_EQ(alloc_.live_blocks(), 4);
  ASSERT_EQ(fs_->Seek(1, h, single_off + kBlockSize, kFsSeekSet),
            single_off + kBlockSize);
  EXPECT_EQ(fs_->Write(1, h, "z", 1), kSyscallErrorNoSpace);
  EXPECT_EQ(alloc_.live_blocks(), 4);
  ASSERT_EQ(fs_->Seek(1, h, single_off, kFsSeekSet), single_off);
  char c = 0;
  ASSERT_EQ(fs_->Read(1, h, &c, 1), 1);
  EXPECT_EQ(c, 'y');
  ASSERT_EQ(StatStr("/f", &st), 0);
  EXPECT_EQ(st.size, single_off + 1);

  alloc_.set_max_blocks(-1);
  ASSERT_EQ(fs_->Close(1, h), 0);
  ASSERT_EQ(UnlinkStr("/f"), 0);
  EXPECT_EQ(alloc_.live_blocks(), 0);
}

// Deterministic splitmix64 generator, so that every run of the randomized test
// performs the same operations.
class TestRng {
 public:
  explicit TestRng(const uint64_t seed) : state_(seed) {}

  // Returns a pseudo-random value in [0, bound).
  int64_t Uniform(const int64_t bound) {
    state_ += 0x9E3779B97F4A7C15ULL;
    uint64_t z = state_;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    z ^= z >> 31;
    return static_cast<int64_t>(z % static_cast<uint64_t>(bound));
  }

  // Returns true with probability 1 / `n`.
  bool OneIn(const int64_t n) { return Uniform(n) == 0; }

 private:
  uint64_t state_;
};

// A reference model of `FileSystem` for `RamfsModelTest`: the tree, the file
// contents and the handles in standard containers, with the behavior that
// syscall_abi.h and ramfs.h document; where several errors apply, it returns
// the one that `FileSystem` checks first. It also counts the blocks that
// `FileSystem` allocates, to predict out-of-space failures under a block limit:
// a regular file takes its data blocks and the indirect blocks that map them,
// and a directory one `DirBlock` while it has entries (the test keeps
// directories far smaller than a `DirBlock`). The test never passes invalid
// client IDs, flags or sizes, which the tests above cover.
class FsModel {
 public:
  static constexpr int kRoot = 0;

  struct Entry {
    std::string name;
    int node = -1;
    // Larger than the cookies of the entries added to the directory before.
    int64_t cookie = 0;
  };

  struct Node {
    FsNodeType type = FsNodeType::kNone;
    // The fnode ID that `FileSystem` gave the node (the test learns it).
    uint64_t fnode_id = 0;
    // The directory with the node's entry, and the entry's name.
    int parent = kRoot;
    std::string name;
    // False once the node's entry is gone (never for the root).
    bool linked = true;
    int open_refs = 0;
    // Directories: the entries in cookie order.
    std::vector<Entry> entries;
    int64_t next_cookie = 1;
    // Regular files: the size, and the blocks written since the file was
    // created or truncated, by index. All other bytes read as zeros.
    int64_t size = 0;
    std::map<int64_t, std::vector<uint8_t>> blocks;
  };

  struct Handle {
    uint64_t client = 0;
    int node = -1;
    uint64_t flags = 0;
    // A regular file's offset, or the cookie of the last entry that
    // ReadDir() returned.
    int64_t offset = 0;
  };

  FsModel() {
    Node& root = nodes_[kRoot];
    root.type = FsNodeType::kDirectory;
    root.fnode_id = 1;
  }

  // The operations of `FileSystem`: each returns what `FileSystem` should
  // return and, on success, changes the model the same way. Open() takes the
  // handle that `FileSystem` returned, and Open() and MkDir() store the file
  // or directory that they create in `created()`.
  int64_t Open(const uint64_t client,    //
               const std::string& path,  //
               const uint64_t flags,     //
               const uint64_t handle) {
    created_ = -1;
    if (std::ssize(handles_) >= kMaxOpenHandles) {
      return kSyscallErrorFileTableFull;
    }
    const Parsed p = Resolve(path);
    if (p.error < 0) {
      return p.error;
    }
    const bool want_write = (flags & kFsOpenAccessMask) != kFsOpenRead;
    const bool want_create = (flags & kFsOpenCreate) != 0;
    const bool want_dir = (flags & kFsOpenDirectory) != 0;
    int idx = p.name.empty() ? p.dir : Lookup(p.dir, p.name);
    if (idx >= 0) {
      Node& node = nodes_.at(idx);
      if (want_create && (flags & kFsOpenExclusive) != 0) {
        return kSyscallErrorExists;
      }
      if (node.type == FsNodeType::kDirectory) {
        if (want_write || want_create ||
            (flags & (kFsOpenTruncate | kFsOpenAppend)) != 0) {
          return kSyscallErrorIsDir;
        }
      } else if (want_dir || p.trailing_slash) {
        return kSyscallErrorNotDir;
      } else if ((flags & kFsOpenTruncate) != 0) {
        node.size = 0;
        node.blocks.clear();
      }
    } else {
      if (!want_create) {
        return kSyscallErrorNoEntry;
      }
      if (want_dir || p.trailing_slash) {
        return kSyscallErrorIsDir;
      }
      if (!CanAddNode(p.dir)) {
        return kSyscallErrorNoSpace;
      }
      idx = AddNode(FsNodeType::kRegular, p.dir, p.name);
    }
    ++nodes_.at(idx).open_refs;
    handles_[handle] = Handle{.client = client, .node = idx, .flags = flags};
    return 0;
  }

  int64_t Close(const uint64_t client, const uint64_t handle) {
    const Handle* const h = FindHandle(client, handle);
    if (h == nullptr) {
      return kSyscallErrorBadFd;
    }
    const int idx = h->node;
    handles_.erase(handle);
    --nodes_.at(idx).open_refs;
    MaybeDrop(idx);
    return 0;
  }

  void CloseAllForClient(const uint64_t client) {
    std::vector<uint64_t> client_handles;
    for (const auto& [handle, h] : handles_) {
      if (h.client == client) {
        client_handles.push_back(handle);
      }
    }
    for (const uint64_t handle : client_handles) {
      Close(client, handle);
    }
  }

  int64_t Read(const uint64_t client,  //
               const uint64_t handle,  //
               const int64_t size,     //
               std::vector<uint8_t>* const out) {
    out->clear();
    Handle* const h = FindHandle(client, handle);
    if (h == nullptr) {
      return kSyscallErrorBadFd;
    }
    const Node& file = nodes_.at(h->node);
    if (file.type == FsNodeType::kDirectory) {
      return kSyscallErrorIsDir;
    }
    if ((h->flags & kFsOpenAccessMask) == kFsOpenWrite) {
      return kSyscallErrorBadFd;
    }
    const int64_t n = std::clamp<int64_t>(file.size - h->offset, 0, size);
    out->resize(static_cast<size_t>(n));
    for (int64_t done = 0; done < n;) {
      const int64_t pos = h->offset + done;
      const int64_t chunk = std::min(n - done, kBlockSize - pos % kBlockSize);
      const auto it = file.blocks.find(pos / kBlockSize);
      if (it != file.blocks.end()) {
        std::copy_n(it->second.begin() + pos % kBlockSize,  //
                    chunk,                                  //
                    out->begin() + done);
      }
      done += chunk;
    }
    h->offset += n;
    return n;
  }

  int64_t Write(const uint64_t client,  //
                const uint64_t handle,  //
                const std::vector<uint8_t>& data) {
    Handle* const h = FindHandle(client, handle);
    if (h == nullptr) {
      return kSyscallErrorBadFd;
    }
    Node& file = nodes_.at(h->node);
    if (file.type == FsNodeType::kDirectory) {
      return kSyscallErrorIsDir;
    }
    if ((h->flags & kFsOpenAccessMask) == kFsOpenRead) {
      return kSyscallErrorBadFd;
    }
    if (data.empty()) {
      return 0;
    }
    const int64_t pos =
        ((h->flags & kFsOpenAppend) != 0) ? file.size : h->offset;
    if (pos >= kFsMaxFileSize) {
      return kSyscallErrorFileTooBig;
    }
    // The write stops at the first block whose blocks don't fit.
    const int64_t limit = std::min(std::ssize(data), kFsMaxFileSize - pos);
    int64_t free_blocks = FreeBlocks();
    int64_t done = 0;
    while (done < limit) {
      const int64_t block = (pos + done) / kBlockSize;
      const int64_t offset = (pos + done) % kBlockSize;
      const int64_t chunk = std::min(limit - done, kBlockSize - offset);
      const int64_t cost = BlockCost(file, block);
      if (cost > free_blocks) {
        break;
      }
      free_blocks -= cost;
      std::vector<uint8_t>& bytes = file.blocks[block];
      bytes.resize(static_cast<size_t>(kBlockSize));
      std::copy_n(data.begin() + done, chunk, bytes.begin() + offset);
      done += chunk;
    }
    if (done == 0) {
      return kSyscallErrorNoSpace;
    }
    h->offset = pos + done;
    file.size = std::max(file.size, h->offset);
    return done;
  }

  // The test seeks a directory only with `(0, kFsSeekSet)` or invalid
  // arguments: the position that `(0, kFsSeekCur)` returns is opaque.
  int64_t Seek(const uint64_t client,  //
               const uint64_t handle,  //
               const int64_t offset,   //
               const int64_t whence) {
    Handle* const h = FindHandle(client, handle);
    if (h == nullptr) {
      return kSyscallErrorBadFd;
    }
    const Node& node = nodes_.at(h->node);
    if (node.type == FsNodeType::kDirectory) {
      if (offset != 0 || whence != kFsSeekSet) {
        return kSyscallErrorInvalid;
      }
      h->offset = 0;
      return 0;
    }
    int64_t base = 0;
    if (whence == kFsSeekCur) {
      base = h->offset;
    } else if (whence == kFsSeekEnd) {
      base = node.size;
    } else if (whence != kFsSeekSet) {
      return kSyscallErrorInvalid;
    }
    if (offset < -base || offset > kFsMaxFileSize - base) {
      return kSyscallErrorInvalid;
    }
    h->offset = base + offset;
    return h->offset;
  }

  int64_t Stat(const std::string& path, FsStat* const out) const {
    const Parsed p = Resolve(path);
    if (p.error < 0) {
      return p.error;
    }
    const int idx = p.name.empty() ? p.dir : Lookup(p.dir, p.name);
    if (idx < 0) {
      return kSyscallErrorNoEntry;
    }
    if (p.trailing_slash && !IsDir(idx)) {
      return kSyscallErrorNotDir;
    }
    *out = StatOf(idx);
    return 0;
  }

  int64_t ReadDir(const uint64_t client,      //
                  const uint64_t handle,      //
                  const int64_t max_entries,  //
                  std::vector<FsDirEntry>* const out) {
    out->clear();
    Handle* const h = FindHandle(client, handle);
    if (h == nullptr) {
      return kSyscallErrorBadFd;
    }
    const Node& dir = nodes_.at(h->node);
    if (dir.type != FsNodeType::kDirectory) {
      return kSyscallErrorNotDir;
    }
    for (const Entry& entry : dir.entries) {
      if (std::ssize(*out) == max_entries) {
        break;
      }
      if (entry.cookie > h->offset) {
        out->push_back(DirEntry(entry));
        h->offset = entry.cookie;
      }
    }
    return std::ssize(*out);
  }

  int64_t MkDir(const std::string& path) {
    created_ = -1;
    const Parsed p = Resolve(path);
    if (p.error < 0) {
      return p.error;
    }
    if (p.name.empty() || Lookup(p.dir, p.name) >= 0) {
      return kSyscallErrorExists;
    }
    if (!CanAddNode(p.dir)) {
      return kSyscallErrorNoSpace;
    }
    AddNode(FsNodeType::kDirectory, p.dir, p.name);
    return 0;
  }

  int64_t Unlink(const std::string& path) {
    const Parsed p = Resolve(path);
    if (p.error < 0) {
      return p.error;
    }
    if (p.name.empty()) {
      return p.root_only ? kSyscallErrorBusy : kSyscallErrorInvalid;
    }
    const int idx = Lookup(p.dir, p.name);
    if (idx < 0) {
      return kSyscallErrorNoEntry;
    }
    if (IsDir(idx)) {
      if (!nodes_.at(idx).entries.empty()) {
        return kSyscallErrorNotEmpty;
      }
    } else if (p.trailing_slash) {
      return kSyscallErrorNotDir;
    }
    RemoveEntry(p.dir, p.name);
    Unlinked(idx);
    return 0;
  }

  int64_t Rename(const std::string& old_path, const std::string& new_path) {
    const Parsed from = Resolve(old_path);
    if (from.error < 0) {
      return from.error;
    }
    if (from.name.empty()) {
      return from.root_only ? kSyscallErrorBusy : kSyscallErrorInvalid;
    }
    const Parsed to = Resolve(new_path);
    if (to.error < 0) {
      return to.error;
    }
    if (to.name.empty()) {
      return to.root_only ? kSyscallErrorBusy : kSyscallErrorInvalid;
    }
    const int src = Lookup(from.dir, from.name);
    if (src < 0) {
      return kSyscallErrorNoEntry;
    }
    if (!IsDir(src) && (from.trailing_slash || to.trailing_slash)) {
      return kSyscallErrorNotDir;
    }
    if (IsDir(src) && IsInside(to.dir, src)) {
      return kSyscallErrorInvalid;
    }
    const int dst = Lookup(to.dir, to.name);
    if (dst == src) {
      return 0;
    }
    if (dst >= 0) {
      if (IsDir(src) && !IsDir(dst)) {
        return kSyscallErrorNotDir;
      }
      if (!IsDir(src) && IsDir(dst)) {
        return kSyscallErrorIsDir;
      }
      if (IsDir(dst) && !nodes_.at(dst).entries.empty()) {
        return kSyscallErrorNotEmpty;
      }
      RemoveEntry(to.dir, to.name);
      Unlinked(dst);
    } else if (nodes_.at(to.dir).entries.empty() && FreeBlocks() <= 0) {
      return kSyscallErrorNoSpace;
    }
    RemoveEntry(from.dir, from.name);
    AddEntry(to.dir, to.name, src);
    return 0;
  }

  // The file or directory that the last Open() or MkDir() created, or -1.
  int created() const { return created_; }

  const Node& node(const int idx) const { return nodes_.at(idx); }

  void set_fnode_id(const int idx, const uint64_t fnode_id) {
    nodes_.at(idx).fnode_id = fnode_id;
  }

  // The open handles, by `FileSystem`'s handle.
  const std::map<uint64_t, Handle>& handles() const { return handles_; }

  // The block limit, like `CountingBlockAllocator::set_max_blocks()`.
  int64_t max_blocks() const { return max_blocks_; }
  void set_max_blocks(const int64_t max_blocks) { max_blocks_ = max_blocks; }

  // The number of files and directories that are linked or open.
  int64_t FnodeCount() const { return std::ssize(nodes_); }

  // The number of blocks that `FileSystem` holds.
  int64_t LiveBlocks() const {
    int64_t count = 0;
    for (const auto& [idx, node] : nodes_) {
      if (node.type == FsNodeType::kRegular) {
        count += FileBlocks(node);
      } else if (!node.entries.empty()) {
        ++count;
      }
    }
    return count;
  }

  // The linked files and directories, each after its parent.
  std::vector<int> Tree() const {
    std::vector<int> nodes = {kRoot};
    for (int i = 0; i < std::ssize(nodes); ++i) {
      for (const Entry& entry : nodes_.at(nodes[i]).entries) {
        nodes.push_back(entry.node);
      }
    }
    return nodes;
  }

  // The names on the path of the linked file or directory `idx`.
  std::vector<std::string> PathNames(int idx) const {
    std::vector<std::string> names;
    for (; idx != kRoot; idx = nodes_.at(idx).parent) {
      names.insert(names.begin(), nodes_.at(idx).name);
    }
    return names;
  }

  std::string PathOf(const int idx) const {
    std::string path;
    for (const std::string& name : PathNames(idx)) {
      path += "/" + name;
    }
    return path.empty() ? "/" : path;
  }

  // What Stat() returns for the linked file or directory `idx`.
  FsStat StatOf(const int idx) const {
    const Node& node = nodes_.at(idx);
    if (!IsDir(idx)) {
      return FsStat{
          .type = node.type,
          .fnode_id = node.fnode_id,
          .size = node.size,
          .nlink = 1,
      };
    }
    const int64_t subdirs = std::ranges::count_if(
        node.entries, [&](const Entry& entry) { return IsDir(entry.node); });
    return FsStat{
        .type = node.type,
        .fnode_id = node.fnode_id,
        .size =
            std::ssize(node.entries) * static_cast<int64_t>(sizeof(FsDirEntry)),
        .nlink = 2 + subdirs,
    };
  }

  // What ReadDir() returns for `entry`.
  FsDirEntry DirEntry(const Entry& entry) const {
    FsDirEntry out{};
    out.fnode_id = nodes_.at(entry.node).fnode_id;
    out.type = nodes_.at(entry.node).type;
    out.name_len = static_cast<int32_t>(entry.name.size());
    std::memcpy(out.name, entry.name.data(), entry.name.size());
    return out;
  }

 private:
  // The first blocks that the single-indirect and the double-indirect block
  // map.
  static constexpr int64_t kFirstSingleBlock = kDirectBlocks;
  static constexpr int64_t kFirstDoubleBlock =
      kDirectBlocks + kPtrsPerIndirectBlock;

  // A resolved path: its last component `name` in directory `dir`, or an
  // empty `name` if the path names `dir` itself (with only '/'s, or with a
  // last component `.` or `..`); or a negative `error`.
  struct Parsed {
    int64_t error = 0;
    int dir = kRoot;
    std::string name;
    bool root_only = false;
    bool trailing_slash = false;
  };

  Parsed Resolve(const std::string& path) const {
    Parsed p;
    if (path.empty()) {
      p.error = kSyscallErrorNoEntry;
      return p;
    }
    p.trailing_slash = (path.back() == '/');
    std::vector<std::string> names;
    std::string name;
    for (const char c : path + "/") {
      if (c != '/') {
        name += c;
      } else if (!name.empty()) {
        names.push_back(name);
        name.clear();
      }
    }
    p.root_only = names.empty();
    for (int i = 0; i < std::ssize(names); ++i) {
      if (names[i] == "..") {
        p.dir = nodes_.at(p.dir).parent;
      } else if (names[i] != ".") {
        if (i == std::ssize(names) - 1) {
          p.name = names[i];
          break;
        }
        const int child = Lookup(p.dir, names[i]);
        if (child < 0) {
          p.error = kSyscallErrorNoEntry;
          break;
        }
        if (!IsDir(child)) {
          p.error = kSyscallErrorNotDir;
          break;
        }
        p.dir = child;
      }
    }
    return p;
  }

  int Lookup(const int dir, const std::string& name) const {
    for (const Entry& entry : nodes_.at(dir).entries) {
      if (entry.name == name) {
        return entry.node;
      }
    }
    return -1;
  }

  bool IsDir(const int idx) const {
    return nodes_.at(idx).type == FsNodeType::kDirectory;
  }

  // Whether directory `dir` is `ancestor` or inside it.
  bool IsInside(int dir, const int ancestor) const {
    for (; dir != ancestor; dir = nodes_.at(dir).parent) {
      if (dir == kRoot) {
        return false;
      }
    }
    return true;
  }

  Handle* FindHandle(const uint64_t client, const uint64_t handle) {
    const auto it = handles_.find(handle);
    if (it == handles_.end() || it->second.client != client) {
      return nullptr;
    }
    return &it->second;
  }

  // The number of blocks that `FileSystem` may still allocate.
  int64_t FreeBlocks() const {
    return (max_blocks_ < 0) ? kInt64Max : max_blocks_ - LiveBlocks();
  }

  // Whether a new file or directory fits into the fnode table and into
  // directory `dir`, which needs a `DirBlock` for it if it is empty.
  bool CanAddNode(const int dir) const {
    return FnodeCount() < kMaxFnodes &&
           (!nodes_.at(dir).entries.empty() || FreeBlocks() > 0);
  }

  int AddNode(const FsNodeType type, const int dir, const std::string& name) {
    const int idx = next_node_++;
    nodes_[idx].type = type;
    AddEntry(dir, name, idx);
    created_ = idx;
    return idx;
  }

  // Adds the entry `name` for `idx` to directory `dir`, with a new cookie.
  void AddEntry(const int dir, const std::string& name, const int idx) {
    Node& d = nodes_.at(dir);
    d.entries.push_back(
        Entry{.name = name, .node = idx, .cookie = d.next_cookie++});
    Node& node = nodes_.at(idx);
    node.parent = dir;
    node.name = name;
  }

  void RemoveEntry(const int dir, const std::string& name) {
    std::erase_if(nodes_.at(dir).entries,
                  [&](const Entry& entry) { return entry.name == name; });
  }

  // Marks `idx`, whose entry is gone, as removed.
  void Unlinked(const int idx) {
    nodes_.at(idx).linked = false;
    MaybeDrop(idx);
  }

  // Drops `idx`, with its blocks, once it is neither linked nor open.
  void MaybeDrop(const int idx) {
    const Node& node = nodes_.at(idx);
    if (!node.linked && node.open_refs == 0) {
      nodes_.erase(idx);
    }
  }

  // Whether `file` has data blocks in [first, end).
  static bool HasBlocks(const Node& file,     //
                        const int64_t first,  //
                        const int64_t end) {
    const auto it = file.blocks.lower_bound(first);
    return it != file.blocks.end() && it->first < end;
  }

  // The blocks of `file`: its data blocks, the single-indirect block while it
  // maps data blocks, and the double-indirect block and its second-level
  // blocks while they do.
  static int64_t FileBlocks(const Node& file) {
    int64_t count = std::ssize(file.blocks);
    if (HasBlocks(file, kFirstSingleBlock, kFirstDoubleBlock)) {
      ++count;
    }
    // The data blocks of a second-level block come one after another.
    int64_t last_second_level = -1;
    for (auto it = file.blocks.lower_bound(kFirstDoubleBlock);
         it != file.blocks.end(); ++it) {
      const int64_t second_level =
          (it->first - kFirstDoubleBlock) / kPtrsPerIndirectBlock;
      if (second_level != last_second_level) {
        // With the double-indirect block for the first second-level block.
        count += (last_second_level < 0) ? 2 : 1;
        last_second_level = second_level;
      }
    }
    return count;
  }

  // The number of blocks that a write into block `block` of `file` allocates:
  // the data block and the indirect blocks that would map it, if missing.
  static int64_t BlockCost(const Node& file, const int64_t block) {
    if (file.blocks.contains(block)) {
      return 0;
    }
    if (block < kFirstSingleBlock) {
      return 1;
    }
    if (block < kFirstDoubleBlock) {
      return HasBlocks(file, kFirstSingleBlock, kFirstDoubleBlock) ? 1 : 2;
    }
    const int64_t first_of_second_level =
        block - (block - kFirstDoubleBlock) % kPtrsPerIndirectBlock;
    const bool has_double = HasBlocks(file, kFirstDoubleBlock, kInt64Max);
    const bool has_second_level =
        HasBlocks(file,                   //
                  first_of_second_level,  //
                  first_of_second_level + kPtrsPerIndirectBlock);
    return 1 + (has_double ? 0 : 1) + (has_second_level ? 0 : 1);
  }

  // The linked or open files and directories, by a number that is never
  // reused.
  std::map<int, Node> nodes_;
  std::map<uint64_t, Handle> handles_;
  int next_node_ = kRoot + 1;
  int created_ = -1;
  int64_t max_blocks_ = -1;
};

// Runs random operations on `FileSystem` and on `FsModel` and compares their
// results and, after each operation, the tree (Stat() of every file and
// directory, and ReadDir() of every directory) and the numbers of open
// handles, fnodes and blocks. Paths name files and directories of the tree or
// new entries in its directories, sometimes with a `.` component, a detour
// through `<name>/..` or a trailing '/'. Reads and writes often seek first,
// writes favor the first blocks that the single-indirect and double-indirect
// blocks map and the file size limit, and phases with a block limit inject
// out-of-space failures.
class RamfsModelTest : public RamfsTest {
 protected:
  // The names of entries: few, so that no directory needs a second
  // `DirBlock`, not even with the extra entry during a Rename() within it.
  static constexpr const char* kNames[] = {"a", "b", "c", "d", "e"};
  static_assert(std::ssize(kNames) < DirBlock::kSlotsPerBlock);
  // Clients 1 to `kClients` open files, and `kCheckClient` checks the tree.
  static constexpr int kClients = 3;
  static constexpr uint64_t kCheckClient = kClients + 1;
  // Beyond these, the test closes instead of opening, and removes instead of
  // creating.
  static constexpr int kMaxHandles = 16;
  static constexpr int kMaxTreeNodes = 24;
  // Writes often seek to within a block of the start of one of these blocks:
  // the first one, the first ones that the single-indirect block, the
  // double-indirect block and its second and third second-level blocks map,
  // and the one at `kFsMaxFileSize`.
  static constexpr int64_t kWriteTargetBlocks[] = {
      0,
      kDirectBlocks,
      kDirectBlocks + kPtrsPerIndirectBlock,
      kDirectBlocks + 2 * kPtrsPerIndirectBlock,
      kDirectBlocks + 3 * kPtrsPerIndirectBlock,
      kFsMaxFileSize / kBlockSize,
  };

  // What RandomPath() names: a file or directory of the tree, a regular file
  // of the tree (or a new entry if there is none), a directory of the tree,
  // or a new entry (a random name in a directory of the tree).
  enum class PathTo { kNode, kFile, kDir, kName };
  // Which handles PickHandle() prefers.
  enum class Prefer { kAny, kReadableFile, kWritableFile, kDirectory };

  struct ClientHandle {
    uint64_t client = 0;
    uint64_t handle = 0;
  };

  // Runs one random operation.
  void RandomStep(TestRng& rng) {
    const int64_t pick = rng.Uniform(100);
    const int64_t handles = std::ssize(model_.handles());
    const bool crowded = std::ssize(model_.Tree()) > kMaxTreeNodes;
    if (pick < 2) {
      ToggleBlockLimit(rng);
    } else if (pick < 3) {
      DoCloseAllForClient(RandomClient(rng));
    } else if (handles == 0 || (pick < 17 && handles < kMaxHandles)) {
      RandomOpen(rng, crowded);
    } else if (pick < 24) {
      const ClientHandle h = PickHandle(rng, Prefer::kAny);
      DoClose(h.client, h.handle);
    } else if (pick < 44) {
      RandomWrite(rng);
    } else if (pick < 56) {
      RandomRead(rng);
    } else if (pick < 61) {
      RandomSeek(rng);
    } else if (pick < 67) {
      static constexpr int64_t kMaxEntries[] = {0, 1, 2, 3,
                                                kSyscallFsMaxDirEntries};
      const ClientHandle h = PickHandle(rng, Prefer::kDirectory);
      const int64_t max_entries =
          kMaxEntries[rng.Uniform(std::ssize(kMaxEntries))];
      DoReadDir(h.client, h.handle, max_entries);
    } else if (pick < 72) {
      DoStat(RandomPath(rng, rng.OneIn(2) ? PathTo::kNode : PathTo::kName));
    } else if (pick < 76 && !crowded) {
      DoMkDir(RandomPath(rng, rng.OneIn(8) ? PathTo::kNode : PathTo::kName));
    } else if (pick < 86) {
      DoUnlink(RandomPath(rng, rng.OneIn(8) ? PathTo::kName : PathTo::kNode));
    } else {
      const std::string old_path =
          RandomPath(rng, rng.OneIn(8) ? PathTo::kName : PathTo::kNode);
      // Sometimes a path through the source, like `/a/../b` for `/a`.
      const std::string new_path =
          rng.OneIn(8)
              ? old_path + "/../" + RandomName(rng)
              : RandomPath(rng, rng.OneIn(3) ? PathTo::kFile : PathTo::kName);
      DoRename(old_path, new_path);
    }
  }

  // Starts a phase with room for at most 3 more blocks, or ends it.
  void ToggleBlockLimit(TestRng& rng) {
    const int max_blocks =
        (model_.max_blocks() < 0)
            ? static_cast<int>(model_.LiveBlocks() + rng.Uniform(4))
            : -1;
    alloc_.set_max_blocks(max_blocks);
    model_.set_max_blocks(max_blocks);
  }

  void RandomOpen(TestRng& rng, const bool crowded) {
    const uint64_t client = RandomClient(rng);
    if (rng.OneIn(5)) {
      const uint64_t flags =
          kFsOpenRead | (rng.OneIn(2) ? kFsOpenDirectory : 0);
      DoOpen(client, RandomPath(rng, PathTo::kDir), flags);
      return;
    }
    static constexpr uint64_t kAccessModes[] = {kFsOpenRead, kFsOpenWrite,
                                                kFsOpenReadWrite};
    const uint64_t access = kAccessModes[rng.Uniform(std::ssize(kAccessModes))];
    uint64_t flags = access;
    if (rng.OneIn(2)) {
      flags |= kFsOpenCreate;
      if (rng.OneIn(4)) {
        flags |= kFsOpenExclusive;
      }
    }
    if (access != kFsOpenRead && rng.OneIn(4)) {
      flags |= kFsOpenTruncate;
    }
    if (rng.OneIn(4)) {
      flags |= kFsOpenAppend;
    }
    if (rng.OneIn(16)) {
      flags |= kFsOpenDirectory;
    }
    PathTo to = PathTo::kFile;
    if (!crowded && rng.OneIn(2)) {
      to = PathTo::kName;
    } else if (rng.OneIn(8)) {
      to = PathTo::kNode;
    }
    DoOpen(client, RandomPath(rng, to), flags);
  }

  void RandomWrite(TestRng& rng) {
    const ClientHandle h = PickHandle(rng, Prefer::kWritableFile);
    if (rng.OneIn(2)) {
      const int64_t block =
          kWriteTargetBlocks[rng.Uniform(std::ssize(kWriteTargetBlocks))];
      const int64_t offset =
          block * kBlockSize + rng.Uniform(2 * kBlockSize + 1) - kBlockSize;
      DoSeek(h.client, h.handle, offset, kFsSeekSet);
    }
    const int64_t size = rng.OneIn(2) ? kSyscallFsMaxIoSize
                                      : rng.Uniform(kSyscallFsMaxIoSize + 1);
    // Bytes that differ from write to write, and from the zeros of holes.
    const int64_t tag = rng.Uniform(251);
    std::vector<uint8_t> data(static_cast<size_t>(size));
    for (int64_t i = 0; i < size; ++i) {
      data[static_cast<size_t>(i)] =
          static_cast<uint8_t>(1 + (tag + 7 * i) % 251);
    }
    DoWrite(h.client, h.handle, data);
  }

  void RandomRead(TestRng& rng) {
    const ClientHandle h = PickHandle(rng, Prefer::kReadableFile);
    if (rng.OneIn(2)) {
      DoSeek(h.client, h.handle, rng.Uniform(FileSize(h) + 1), kFsSeekSet);
    }
    const int64_t size = rng.OneIn(2) ? kSyscallFsMaxIoSize
                                      : rng.Uniform(kSyscallFsMaxIoSize + 1);
    DoRead(h.client, h.handle, size);
  }

  void RandomSeek(TestRng& rng) {
    const ClientHandle h = PickHandle(rng, Prefer::kAny);
    const auto it = model_.handles().find(h.handle);
    if (it != model_.handles().end() &&
        model_.node(it->second.node).type == FsNodeType::kDirectory) {
      // Rewinds or fails: the position that `(0, kFsSeekCur)` returns is
      // opaque.
      static constexpr std::pair<int64_t, int64_t> kDirSeeks[] = {
          {0, kFsSeekSet}, {1, kFsSeekSet}, {0, kFsSeekEnd}, {-1, kFsSeekCur}};
      const auto& [offset, whence] =
          kDirSeeks[rng.Uniform(std::ssize(kDirSeeks))];
      DoSeek(h.client, h.handle, offset, whence);
      return;
    }
    static constexpr int64_t kWhences[] = {kFsSeekSet, kFsSeekCur, kFsSeekEnd};
    const int64_t whence = kWhences[rng.Uniform(std::ssize(kWhences))];
    const int64_t offset = (whence == kFsSeekSet)
                               ? rng.Uniform(FileSize(h) + 2)
                               : rng.Uniform(2 * kBlockSize + 1) - kBlockSize;
    DoSeek(h.client, h.handle, offset, whence);
  }

  // The size of the file of handle `h`, or 0 if `h` isn't open.
  int64_t FileSize(const ClientHandle& h) const {
    const auto it = model_.handles().find(h.handle);
    return (it == model_.handles().end()) ? 0
                                          : model_.node(it->second.node).size;
  }

  static uint64_t RandomClient(TestRng& rng) {
    return 1 + static_cast<uint64_t>(rng.Uniform(kClients));
  }

  static std::string RandomName(TestRng& rng) {
    return kNames[rng.Uniform(std::ssize(kNames))];
  }

  // Returns a path to what `to` says, sometimes with a `.` component, a
  // detour through `<name>/..` or a trailing '/'.
  std::string RandomPath(TestRng& rng, const PathTo to) const {
    std::vector<int> nodes;
    for (const int node : model_.Tree()) {
      const bool is_dir = model_.node(node).type == FsNodeType::kDirectory;
      if (to == PathTo::kNode || (to == PathTo::kFile ? !is_dir : is_dir)) {
        nodes.push_back(node);
      }
    }
    if (nodes.empty()) {
      return RandomPath(rng, PathTo::kName);
    }
    std::vector<std::string> names =
        model_.PathNames(nodes[rng.Uniform(std::ssize(nodes))]);
    if (to == PathTo::kName) {
      names.push_back(RandomName(rng));
    }
    if (rng.OneIn(16)) {
      names.insert(names.begin() + rng.Uniform(std::ssize(names) + 1), ".");
    }
    if (rng.OneIn(16)) {
      const std::string detour = RandomName(rng);
      names.insert(names.begin() + rng.Uniform(std::ssize(names) + 1),
                   {detour, ".."});
    }
    std::string path;
    for (const std::string& name : names) {
      path += "/" + name;
    }
    if (path.empty() || rng.OneIn(8)) {
      path += "/";
    }
    return path;
  }

  // Returns an open handle with its client, most often one that `prefer`
  // describes; or sometimes a closed handle, or an open one with another
  // client, which fail with `kSyscallErrorBadFd`.
  ClientHandle PickHandle(TestRng& rng, const Prefer prefer) const {
    if (!closed_.empty() && rng.OneIn(32)) {
      return closed_[rng.Uniform(std::ssize(closed_))];
    }
    std::vector<ClientHandle> all;
    std::vector<ClientHandle> preferred;
    for (const auto& [handle, h] : model_.handles()) {
      all.push_back({h.client, handle});
      if (IsPreferred(h, prefer)) {
        preferred.push_back({h.client, handle});
      }
    }
    ClientHandle picked = (preferred.empty() || rng.OneIn(16))
                              ? all[rng.Uniform(std::ssize(all))]
                              : preferred[rng.Uniform(std::ssize(preferred))];
    if (rng.OneIn(32)) {
      picked.client = picked.client % kClients + 1;
    }
    return picked;
  }

  bool IsPreferred(const FsModel::Handle& h, const Prefer prefer) const {
    const bool is_dir = model_.node(h.node).type == FsNodeType::kDirectory;
    const uint64_t access = h.flags & kFsOpenAccessMask;
    switch (prefer) {
      case Prefer::kAny:
        return true;
      case Prefer::kReadableFile:
        return !is_dir && access != kFsOpenWrite;
      case Prefer::kWritableFile:
        return !is_dir && access != kFsOpenRead;
      case Prefer::kDirectory:
        return is_dir;
    }
    return false;
  }

  // The operations: each runs on both `FileSystem` and the model and compares
  // the results.
  void DoOpen(const uint64_t client,    //
              const std::string& path,  //
              const uint64_t flags) {
    SCOPED_TRACE(t::Message() << "Open(" << client << ", \"" << path << "\", 0x"
                              << std::hex << flags << ")");
    FsOpenReply reply{};
    const int64_t rc =
        fs_->Open(client, path.data(), std::ssize(path), flags, &reply);
    if (rc == 0) {
      // Each Open() returns a new handle.
      ASSERT_NE(reply.handle, 0u);
      ASSERT_TRUE(handles_seen_.insert(reply.handle).second) << reply.handle;
    }
    ASSERT_EQ(rc, model_.Open(client, path, flags, reply.handle));
    if (rc == 0) {
      ASSERT_EQ(reply.type,
                model_.node(model_.handles().at(reply.handle).node).type);
      if (model_.created() >= 0) {
        LearnFnodeId(model_.created());
      }
    }
  }

  void DoClose(const uint64_t client, const uint64_t handle) {
    SCOPED_TRACE(t::Message() << "Close(" << client << ", " << handle << ")");
    const int64_t rc = fs_->Close(client, handle);
    ASSERT_EQ(rc, model_.Close(client, handle));
    if (rc == 0) {
      closed_.push_back({client, handle});
    }
  }

  void DoCloseAllForClient(const uint64_t client) {
    for (const auto& [handle, h] : model_.handles()) {
      if (h.client == client) {
        closed_.push_back({client, handle});
      }
    }
    fs_->CloseAllForClient(client);
    model_.CloseAllForClient(client);
  }

  void DoRead(const uint64_t client,  //
              const uint64_t handle,  //
              const int64_t size) {
    SCOPED_TRACE(t::Message()
                 << "Read(" << client << ", " << handle << ", " << size << ")");
    std::vector<uint8_t> data(static_cast<size_t>(size));
    const int64_t rc = fs_->Read(client, handle, data.data(), size);
    std::vector<uint8_t> want;
    ASSERT_EQ(rc, model_.Read(client, handle, size, &want));
    data.resize(want.size());
    ASSERT_TRUE(data == want) << "The data differs.";
  }

  void DoWrite(const uint64_t client,  //
               const uint64_t handle,  //
               const std::vector<uint8_t>& data) {
    SCOPED_TRACE(t::Message() << "Write(" << client << ", " << handle << ", "
                              << data.size() << " bytes)");
    ASSERT_EQ(fs_->Write(client, handle, data.data(), std::ssize(data)),
              model_.Write(client, handle, data));
  }

  void DoSeek(const uint64_t client,  //
              const uint64_t handle,  //
              const int64_t offset,   //
              const int64_t whence) {
    SCOPED_TRACE(t::Message() << "Seek(" << client << ", " << handle << ", "
                              << offset << ", " << whence << ")");
    ASSERT_EQ(fs_->Seek(client, handle, offset, whence),
              model_.Seek(client, handle, offset, whence));
  }

  void DoReadDir(const uint64_t client,  //
                 const uint64_t handle,  //
                 const int64_t max_entries) {
    SCOPED_TRACE(t::Message() << "ReadDir(" << client << ", " << handle << ", "
                              << max_entries << ")");
    std::vector<FsDirEntry> entries(static_cast<size_t>(max_entries));
    const int64_t rc =
        fs_->ReadDir(client, handle, entries.data(), max_entries);
    std::vector<FsDirEntry> want;
    ASSERT_EQ(rc, model_.ReadDir(client, handle, max_entries, &want));
    for (int i = 0; i < std::ssize(want); ++i) {
      ASSERT_NO_FATAL_FAILURE(AssertSameEntry(entries[static_cast<size_t>(i)],
                                              want[static_cast<size_t>(i)]));
    }
  }

  void DoStat(const std::string& path) {
    SCOPED_TRACE(t::Message() << "Stat(\"" << path << "\")");
    FsStat st{};
    FsStat want{};
    const int64_t rc = StatStr(path, &st);
    ASSERT_EQ(rc, model_.Stat(path, &want));
    if (rc == 0) {
      AssertSameStat(st, want);
    }
  }

  void DoMkDir(const std::string& path) {
    SCOPED_TRACE(t::Message() << "MkDir(\"" << path << "\")");
    const int64_t rc = MkDirStr(path);
    ASSERT_EQ(rc, model_.MkDir(path));
    if (rc == 0) {
      LearnFnodeId(model_.created());
    }
  }

  void DoUnlink(const std::string& path) {
    SCOPED_TRACE(t::Message() << "Unlink(\"" << path << "\")");
    ASSERT_EQ(UnlinkStr(path), model_.Unlink(path));
  }

  void DoRename(const std::string& old_path, const std::string& new_path) {
    SCOPED_TRACE(t::Message()
                 << "Rename(\"" << old_path << "\", \"" << new_path << "\")");
    ASSERT_EQ(RenameStr(old_path, new_path), model_.Rename(old_path, new_path));
  }

  // Learns the fnode ID of `node`, which the operation created, and checks
  // that no file or directory had it before.
  void LearnFnodeId(const int node) {
    FsStat st{};
    ASSERT_EQ(StatStr(model_.PathOf(node), &st), 0);
    ASSERT_TRUE(fnode_ids_seen_.insert(st.fnode_id).second)
        << "fnode ID " << st.fnode_id << " is reused.";
    model_.set_fnode_id(node, st.fnode_id);
  }

  static void AssertSameStat(const FsStat& st, const FsStat& want) {
    ASSERT_EQ(st.type, want.type);
    ASSERT_EQ(st.fnode_id, want.fnode_id);
    ASSERT_EQ(st.size, want.size);
    ASSERT_EQ(st.nlink, want.nlink);
  }

  // Also checks the NUL bytes after the name and the reserved bytes.
  static void AssertSameEntry(const FsDirEntry& entry, const FsDirEntry& want) {
    ASSERT_EQ(std::memcmp(&entry, &want, sizeof(FsDirEntry)), 0)
        << "fnode " << entry.fnode_id << " instead of " << want.name
        << " (fnode " << want.fnode_id << ")";
  }

  // Compares Stat() of the file or directory `node` at `path` and, for a
  // directory, ReadDir() of it and everything below it with the model.
  void VerifyTree(const int node, const std::string& path) {
    FsStat st{};
    ASSERT_EQ(StatStr(path, &st), 0) << path;
    ASSERT_NO_FATAL_FAILURE(AssertSameStat(st, model_.StatOf(node))) << path;
    if (st.type != FsNodeType::kDirectory) {
      return;
    }
    uint64_t h = 0;
    ASSERT_EQ(OpenStr(kCheckClient, path, kFsOpenRead | kFsOpenDirectory, &h),
              0)
        << path;
    FsDirEntry entries[kSyscallFsMaxDirEntries]{};
    const int64_t count =
        fs_->ReadDir(kCheckClient, h, entries, kSyscallFsMaxDirEntries);
    ASSERT_EQ(fs_->Close(kCheckClient, h), 0);
    const std::vector<FsModel::Entry>& want = model_.node(node).entries;
    ASSERT_EQ(count, std::ssize(want)) << path;
    for (int i = 0; i < count; ++i) {
      ASSERT_NO_FATAL_FAILURE(AssertSameEntry(
          entries[i], model_.DirEntry(want[static_cast<size_t>(i)])))
          << path;
    }
    for (const FsModel::Entry& entry : want) {
      ASSERT_NO_FATAL_FAILURE(VerifyTree(
          entry.node, (node == FsModel::kRoot ? "" : path) + "/" + entry.name));
    }
  }

  // Compares the tree and the numbers of open handles, fnodes and blocks with
  // the model.
  void VerifyAll() {
    ASSERT_NO_FATAL_FAILURE(VerifyTree(FsModel::kRoot, "/"));
    ASSERT_EQ(fs_->ActiveHandleCount(), std::ssize(model_.handles()));
    ASSERT_EQ(fs_->ActiveFnodeCount(), model_.FnodeCount());
    ASSERT_EQ(alloc_.live_blocks(), model_.LiveBlocks());
  }

  // Reads back the blocks of every file of the tree. Then closes all handles
  // and removes the tree without room for another block: neither allocates.
  void VerifyFilesAndRemoveAll() {
    for (const int node : model_.Tree()) {
      const FsModel::Node& file = model_.node(node);
      if (file.type != FsNodeType::kRegular) {
        continue;
      }
      const std::string path = model_.PathOf(node);
      uint64_t h = 0;
      ASSERT_EQ(OpenStr(kCheckClient, path, kFsOpenRead, &h), 0) << path;
      for (const auto& [block, bytes] : file.blocks) {
        const int64_t offset = block * kBlockSize;
        const int64_t size = std::min(kBlockSize, file.size - offset);
        std::vector<uint8_t> data(static_cast<size_t>(size));
        ASSERT_EQ(fs_->Seek(kCheckClient, h, offset, kFsSeekSet), offset);
        ASSERT_EQ(fs_->Read(kCheckClient, h, data.data(), size), size);
        ASSERT_TRUE(std::equal(data.begin(), data.end(), bytes.begin()))
            << path << " block " << block;
      }
      ASSERT_EQ(fs_->Close(kCheckClient, h), 0);
    }

    alloc_.set_max_blocks(alloc_.live_blocks());
    model_.set_max_blocks(model_.LiveBlocks());
    for (int client = 1; client <= kClients; ++client) {
      DoCloseAllForClient(static_cast<uint64_t>(client));
    }
    const std::vector<int> nodes = model_.Tree();
    for (int i = static_cast<int>(std::ssize(nodes)) - 1; i > 0; --i) {
      ASSERT_NO_FATAL_FAILURE(DoUnlink(model_.PathOf(nodes[i])));
    }
  }

  FsModel model_;
  // Every handle that Open() returned and every fnode ID that Stat() showed.
  std::set<uint64_t> handles_seen_;
  std::set<uint64_t> fnode_ids_seen_ = {1};
  std::vector<ClientHandle> closed_;
};

TEST_F(RamfsModelTest, RandomOperationsMatchAReferenceModel) {
  constexpr uint64_t kSeed = 2026;
  constexpr int kSteps = 3000;
  TestRng rng(kSeed);
  for (int step = 0; step < kSteps; ++step) {
    SCOPED_TRACE(t::Message() << "seed " << kSeed << ", step " << step);
    ASSERT_NO_FATAL_FAILURE(RandomStep(rng));
    ASSERT_NO_FATAL_FAILURE(VerifyAll());
  }
  ASSERT_NO_FATAL_FAILURE(VerifyFilesAndRemoveAll());
  EXPECT_EQ(alloc_.live_blocks(), 0);
  EXPECT_EQ(fs_->ActiveFnodeCount(), 1);
  EXPECT_EQ(fs_->ActiveHandleCount(), 0);
}

// Returns the request `hdr` followed by `tail`.
template <typename Header>
static std::vector<uint8_t> Req(const Header& hdr,
                                const std::string& tail = std::string()) {
  std::vector<uint8_t> req(sizeof(hdr));
  std::memcpy(req.data(), &hdr, sizeof(hdr));
  req.insert(req.end(), tail.begin(), tail.end());
  return req;
}

static std::vector<uint8_t> OpenReq(const std::string& path,
                                    const uint64_t flags) {
  return Req(
      FsOpenRequest{
          .op = FsRpcOp::kOpen,
          .path_len = static_cast<int32_t>(path.size()),
          .flags = flags,
      },
      path);
}

static std::vector<uint8_t> PathReq(const FsRpcOp op, const std::string& path) {
  return Req(
      FsPathRequest{.op = op, .path_len = static_cast<int32_t>(path.size())},
      path);
}

static std::vector<uint8_t> CloseReq(const uint64_t handle) {
  return Req(FsCloseRequest{.op = FsRpcOp::kClose, .handle = handle});
}

static std::vector<uint8_t> ReadReq(const uint64_t handle, const int64_t size) {
  return Req(
      FsReadRequest{.op = FsRpcOp::kRead, .handle = handle, .size = size});
}

static std::vector<uint8_t> WriteReq(const uint64_t handle,
                                     const std::string& data) {
  return Req(
      FsWriteRequest{
          .op = FsRpcOp::kWrite,
          .handle = handle,
          .size = static_cast<int64_t>(data.size()),
      },
      data);
}

static std::vector<uint8_t> SeekReq(const uint64_t handle,  //
                                    const int64_t offset,   //
                                    const int64_t whence) {
  return Req(FsSeekRequest{
      .op = FsRpcOp::kSeek,
      .whence = static_cast<int32_t>(whence),
      .handle = handle,
      .offset = offset,
  });
}

static std::vector<uint8_t> ReadDirReq(const uint64_t handle,
                                       const int32_t max_entries) {
  return Req(FsReadDirRequest{
      .op = FsRpcOp::kReadDir,
      .max_entries = max_entries,
      .handle = handle,
  });
}

static std::vector<uint8_t> RenameReq(const std::string& old_path,
                                      const std::string& new_path) {
  return Req(
      FsRenameRequest{
          .op = FsRpcOp::kRename,
          .old_len = static_cast<int32_t>(old_path.size()),
          .new_len = static_cast<int32_t>(new_path.size()),
      },
      old_path + new_path);
}

// `DispatchRpc()` tests. The reply buffer is misaligned by one byte, so that
// UBSan catches replies written through misaligned pointers.
class RamfsRpcTest : public RamfsTest {
 protected:
  static constexpr uint64_t kClient = 7;

  // Runs `DispatchRpc()` on `req` for `kClient`, with room for `reply_cap`
  // reply bytes, and stores the reply size in `reply_size_`.
  int64_t Dispatch(const std::vector<uint8_t>& req,
                   const int64_t reply_cap = kIpcMaxMessageSize) {
    return DispatchRpc(*fs_,                              //
                       kClient,                           //
                       req.data(),                        //
                       static_cast<int64_t>(req.size()),  //
                       reply_buf_,                        //
                       reply_cap,                         //
                       &reply_size_);
  }

  template <typename Reply>
  Reply ReplyAs() const {
    Reply reply;
    std::memcpy(&reply, reply_buf_, sizeof(reply));
    return reply;
  }

  // Fills the reply buffer with a pattern that `ReplyBufUntouched()` checks,
  // and sets `reply_size_` to -1, which `DispatchRpc()` must overwrite.
  void PoisonReplyBuf() {
    std::memset(raw_reply_, 0xA5, sizeof(raw_reply_));
    reply_size_ = -1;
  }

  bool ReplyBufUntouched() const {
    for (const uint8_t byte : raw_reply_) {
      if (byte != 0xA5) {
        return false;
      }
    }
    return true;
  }

  // Expects `DispatchRpc()` to reject `req` with `kSyscallErrorInvalid`,
  // without reporting or writing reply bytes. `what` names the request in
  // failure messages.
  void ExpectInvalid(const std::vector<uint8_t>& req, const std::string& what) {
    PoisonReplyBuf();
    EXPECT_EQ(Dispatch(req), kSyscallErrorInvalid) << what;
    EXPECT_EQ(reply_size_, 0) << what;
    EXPECT_TRUE(ReplyBufUntouched()) << what;
  }

  alignas(8) uint8_t raw_reply_[kIpcMaxMessageSize + 8] = {};
  uint8_t* const reply_buf_ = raw_reply_ + 1;
  int64_t reply_size_ = -1;
};

TEST_F(RamfsRpcTest, WireProtocolAndUnalignedReplySafety) {
  ASSERT_EQ(Dispatch(PathReq(FsRpcOp::kMkDir, "/rpc")), 0);
  EXPECT_EQ(reply_size_, 0);

  // Open `/rpc/hello`
  const std::string fpath = "/rpc/hello";
  ASSERT_EQ(Dispatch(OpenReq(fpath, kFsOpenReadWrite | kFsOpenCreate)), 0);
  ASSERT_EQ(reply_size_, static_cast<int64_t>(sizeof(FsOpenReply)));
  const FsOpenReply open_rep = ReplyAs<FsOpenReply>();
  EXPECT_NE(open_rep.handle, 0u);
  EXPECT_EQ(open_rep.type, FsNodeType::kRegular);

  // Write + Seek + Read
  const std::string payload = "rpc-payload";
  ASSERT_EQ(Dispatch(WriteReq(open_rep.handle, payload)),
            static_cast<int64_t>(payload.size()));
  EXPECT_EQ(reply_size_, 0);
  ASSERT_EQ(Dispatch(SeekReq(open_rep.handle, 0, kFsSeekSet)), 0);
  ASSERT_EQ(Dispatch(ReadReq(open_rep.handle, 64)),
            static_cast<int64_t>(payload.size()));
  EXPECT_EQ(std::string(reinterpret_cast<const char*>(reply_buf_),
                        static_cast<size_t>(reply_size_)),
            payload);

  // Close file, open `/rpc` directory, ReadDir into unaligned `reply_buf_`.
  ASSERT_EQ(Dispatch(CloseReq(open_rep.handle)), 0);
  ASSERT_EQ(Dispatch(OpenReq("/rpc", kFsOpenRead | kFsOpenDirectory)), 0);
  const uint64_t dir_h = ReplyAs<FsOpenReply>().handle;
  ASSERT_EQ(Dispatch(ReadDirReq(dir_h, 4)), 1);
  ASSERT_EQ(reply_size_, static_cast<int64_t>(sizeof(FsDirEntry)));
  const FsDirEntry dent = ReplyAs<FsDirEntry>();
  EXPECT_EQ(std::string(dent.name, static_cast<size_t>(dent.name_len)),
            "hello");
  ASSERT_EQ(Dispatch(CloseReq(dir_h)), 0);

  // A failed request has no reply bytes.
  EXPECT_EQ(Dispatch(ReadReq(dir_h, 64)), kSyscallErrorBadFd);
  EXPECT_EQ(reply_size_, 0);

  // Rename `/rpc/hello` -> `/rpc/renamed`, Stat, and Unlink.
  const std::string rdst = "/rpc/renamed";
  ASSERT_EQ(Dispatch(RenameReq(fpath, rdst)), 0);
  ASSERT_EQ(Dispatch(PathReq(FsRpcOp::kStat, rdst)), 0);
  ASSERT_EQ(reply_size_, static_cast<int64_t>(sizeof(FsStat)));
  EXPECT_EQ(ReplyAs<FsStat>().size, static_cast<int64_t>(payload.size()));

  ASSERT_EQ(Dispatch(PathReq(FsRpcOp::kUnlink, rdst)), 0);
  ASSERT_EQ(Dispatch(PathReq(FsRpcOp::kUnlink, "/rpc")), 0);
  EXPECT_EQ(alloc_.live_blocks(), 0);
}

TEST_F(RamfsRpcTest, RejectsMalformedRequestsWithoutSideEffects) {
  uint64_t h = 0;
  ASSERT_EQ(OpenStr(kClient, "/f", kFsOpenReadWrite | kFsOpenCreate, &h), 0);
  ASSERT_EQ(fs_->Write(kClient, h, "data", 4), 4);
  ASSERT_EQ(fs_->Seek(kClient, h, 0, kFsSeekSet), 0);
  uint64_t dir_h = 0;
  ASSERT_EQ(OpenStr(kClient, "/", kFsOpenRead | kFsOpenDirectory, &dir_h), 0);

  // No room for the opcode, no request, and `kNone` or unknown opcodes.
  ExpectInvalid({1, 2}, "2-byte request");
  PoisonReplyBuf();
  EXPECT_EQ(DispatchRpc(*fs_,                //
                        kClient,             //
                        nullptr,             //
                        16,                  //
                        reply_buf_,          //
                        kIpcMaxMessageSize,  //
                        &reply_size_),
            kSyscallErrorInvalid)
      << "null request";
  EXPECT_EQ(reply_size_, 0) << "null request";
  EXPECT_TRUE(ReplyBufUntouched()) << "null request";
  ExpectInvalid(Req(FsRpcOp::kNone), "kNone");
  ExpectInvalid(Req(FsPathRequest{.op = FsRpcOp::kNone, .path_len = 2},
                    std::string("/g")),
                "kNone with a path");
  ExpectInvalid(Req(static_cast<FsRpcOp>(11)), "opcode 11");
  ExpectInvalid(Req(static_cast<FsRpcOp>(0xFFFFFFFF)), "opcode 0xFFFFFFFF");

  // Fixed-size requests one byte short or long.
  for (const auto& [name, req] :
       std::vector<std::pair<std::string, std::vector<uint8_t>>>{
           {"Close", CloseReq(h)},
           {"Read", ReadReq(h, 1)},
           {"Seek", SeekReq(h, 0, kFsSeekSet)},
           {"ReadDir", ReadDirReq(dir_h, 1)},
       }) {
    std::vector<uint8_t> longer = req;
    longer.push_back(0);
    ExpectInvalid(std::vector<uint8_t>(req.begin(), req.end() - 1),
                  name + " one byte short");
    ExpectInvalid(longer, name + " one byte long");
  }
  // Headers of variable-size requests one byte short.
  for (const auto& [name, req] :
       std::vector<std::pair<std::string, std::vector<uint8_t>>>{
           {"Open", Req(FsOpenRequest{.op = FsRpcOp::kOpen})},
           {"Write", Req(FsWriteRequest{.op = FsRpcOp::kWrite, .handle = h})},
           {"Stat", Req(FsPathRequest{.op = FsRpcOp::kStat})},
           {"Rename", Req(FsRenameRequest{.op = FsRpcOp::kRename})},
       }) {
    ExpectInvalid(std::vector<uint8_t>(req.begin(), req.end() - 1),
                  name + " header one byte short");
  }

  // Non-zero reserved fields.
  ExpectInvalid(
      Req(FsCloseRequest{.op = FsRpcOp::kClose, .reserved = 1, .handle = h}),
      "Close with reserved 1");
  ExpectInvalid(
      Req(FsReadRequest{
          .op = FsRpcOp::kRead, .reserved = 1, .handle = h, .size = 1}),
      "Read with reserved 1");
  ExpectInvalid(
      Req(
          FsWriteRequest{
              .op = FsRpcOp::kWrite, .reserved = 1, .handle = h, .size = 1},
          std::string("x")),
      "Write with reserved 1");
  ExpectInvalid(Req(FsRenameRequest{.op = FsRpcOp::kRename,
                                    .old_len = 2,
                                    .new_len = 2,
                                    .reserved = 1},
                    std::string("/f/g")),
                "Rename with reserved 1");

  // Path lengths that are negative or don't match the request size.
  for (const int32_t path_len : {-1, 1, 3, kInt32Max}) {
    const std::string with_len = " with path_len " + std::to_string(path_len);
    ExpectInvalid(Req(FsOpenRequest{.op = FsRpcOp::kOpen,
                                    .path_len = path_len,
                                    .flags = kFsOpenRead},
                      std::string("/f")),
                  "Open" + with_len);
    for (const auto& [op, name] : std::vector<std::pair<FsRpcOp, std::string>>{
             {FsRpcOp::kStat, "Stat"},
             {FsRpcOp::kMkDir, "MkDir"},
             {FsRpcOp::kUnlink, "Unlink"},
         }) {
      ExpectInvalid(
          Req(FsPathRequest{.op = op, .path_len = path_len}, std::string("/g")),
          name + with_len);
    }
  }
  ExpectInvalid(Req(FsOpenRequest{.op = FsRpcOp::kOpen, .path_len = -1}),
                "Open with path_len -1 and no path");

  // Write sizes that are out of range or don't match the data.
  for (const int64_t size :
       {int64_t{-1}, int64_t{3}, int64_t{5}, kInt64Max, kInt64Min}) {
    ExpectInvalid(
        Req(FsWriteRequest{.op = FsRpcOp::kWrite, .handle = h, .size = size},
            std::string("abcd")),
        "Write with size " + std::to_string(size) + " and 4 data bytes");
  }
  ExpectInvalid(Req(FsWriteRequest{
                    .op = FsRpcOp::kWrite, .handle = h, .size = kInt64Max}),
                "Write with size kInt64Max and no data");
  const std::string too_much(static_cast<size_t>(kSyscallFsMaxIoSize + 1), 'x');
  ExpectInvalid(WriteReq(h, too_much),
                "Write of kSyscallFsMaxIoSize + 1 bytes");

  // Rename lengths that are negative or don't add up to the request size.
  for (const auto& [old_len, new_len] :
       std::vector<std::pair<int32_t, int32_t>>{
           {-1, 5}, {5, -1}, {2, 1}, {2, 3}, {kInt32Max, kInt32Max}}) {
    ExpectInvalid(Req(FsRenameRequest{.op = FsRpcOp::kRename,
                                      .old_len = old_len,
                                      .new_len = new_len},
                      std::string("/f/g")),
                  "Rename with old_len " + std::to_string(old_len) +
                      " and new_len " + std::to_string(new_len));
  }

  // Read sizes and ReadDir counts out of range.
  for (const int64_t size :
       {int64_t{-1}, kSyscallFsMaxIoSize + 1, kInt64Max, kInt64Min}) {
    ExpectInvalid(ReadReq(h, size), "Read with size " + std::to_string(size));
  }
  for (const int32_t max_entries :
       {int32_t{-1}, static_cast<int32_t>(kSyscallFsMaxDirEntries + 1),
        kInt32Max}) {
    ExpectInvalid(ReadDirReq(dir_h, max_entries),
                  "ReadDir with max_entries " + std::to_string(max_entries));
  }

  // Nothing changed: no file was created, renamed or removed, no data was
  // written, and neither offset moved.
  FsStat st{};
  EXPECT_EQ(StatStr("/g", &st), kSyscallErrorNoEntry);
  ASSERT_EQ(StatStr("/f", &st), 0);
  EXPECT_EQ(st.size, 4);
  EXPECT_EQ(fs_->ActiveFnodeCount(), 2);
  EXPECT_EQ(fs_->ActiveHandleCount(), 2);
  char buf[8]{};
  ASSERT_EQ(fs_->Read(kClient, h, buf, sizeof(buf)), 4);
  EXPECT_EQ(std::string(buf, 4), "data");
  FsDirEntry entry{};
  ASSERT_EQ(fs_->ReadDir(kClient, dir_h, &entry, 1), 1);
  EXPECT_EQ(std::string(entry.name, static_cast<size_t>(entry.name_len)), "f");
}

TEST_F(RamfsRpcTest, ReadDirAcceptsZeroAndRejectsTooManyEntries) {
  ASSERT_EQ(MkDirStr("/d"), 0);
  ASSERT_EQ(CreateFile("/d/a"), 0);
  uint64_t file_h = 0;
  ASSERT_EQ(OpenStr(kClient, "/d/a", kFsOpenRead, &file_h), 0);
  uint64_t dir_h = 0;
  ASSERT_EQ(OpenStr(kClient, "/d", kFsOpenRead | kFsOpenDirectory, &dir_h), 0);

  // Zero entries check the handle, read nothing, and need no reply room.
  EXPECT_EQ(Dispatch(ReadDirReq(dir_h, 0), /*reply_cap=*/0), 0);
  EXPECT_EQ(reply_size_, 0);
  EXPECT_EQ(Dispatch(ReadDirReq(file_h, 0)), kSyscallErrorNotDir);
  EXPECT_EQ(Dispatch(ReadDirReq(dir_h ^ (1ULL << 16), 0)), kSyscallErrorBadFd);

  // `kSyscallFsMaxDirEntries + 1` entries are too many.
  EXPECT_EQ(Dispatch(ReadDirReq(
                dir_h, static_cast<int32_t>(kSyscallFsMaxDirEntries + 1))),
            kSyscallErrorInvalid);

  // Neither moved the cursor.
  ASSERT_EQ(Dispatch(ReadDirReq(dir_h,
                                static_cast<int32_t>(kSyscallFsMaxDirEntries))),
            1);
  EXPECT_EQ(reply_size_, static_cast<int64_t>(sizeof(FsDirEntry)));
  const FsDirEntry entry = ReplyAs<FsDirEntry>();
  EXPECT_EQ(std::string(entry.name, static_cast<size_t>(entry.name_len)), "a");
}

TEST_F(RamfsRpcTest, RejectsRequestsWhoseReplyMayExceedReplyCap) {
  // Open: with `kFsOpenCreate`, the file must not be created either.
  const int64_t open_reply = static_cast<int64_t>(sizeof(FsOpenReply));
  PoisonReplyBuf();
  EXPECT_EQ(
      Dispatch(OpenReq("/f", kFsOpenReadWrite | kFsOpenCreate), open_reply - 1),
      kSyscallErrorInvalid);
  EXPECT_EQ(reply_size_, 0);
  EXPECT_TRUE(ReplyBufUntouched());
  FsStat st{};
  EXPECT_EQ(StatStr("/f", &st), kSyscallErrorNoEntry);
  EXPECT_EQ(fs_->ActiveHandleCount(), 0);
  ASSERT_EQ(
      Dispatch(OpenReq("/f", kFsOpenReadWrite | kFsOpenCreate), open_reply), 0);
  const uint64_t h = ReplyAs<FsOpenReply>().handle;

  // Stat
  const int64_t stat_reply = static_cast<int64_t>(sizeof(FsStat));
  PoisonReplyBuf();
  EXPECT_EQ(Dispatch(PathReq(FsRpcOp::kStat, "/f"), stat_reply - 1),
            kSyscallErrorInvalid);
  EXPECT_TRUE(ReplyBufUntouched());
  EXPECT_EQ(Dispatch(PathReq(FsRpcOp::kStat, "/f"), stat_reply), 0);
  EXPECT_EQ(reply_size_, stat_reply);

  // Read: `size` must fit, even if fewer bytes remain, and the offset stays.
  ASSERT_EQ(fs_->Write(kClient, h, "0123456789", 10), 10);
  ASSERT_EQ(fs_->Seek(kClient, h, 0, kFsSeekSet), 0);
  PoisonReplyBuf();
  EXPECT_EQ(Dispatch(ReadReq(h, 64), 63), kSyscallErrorInvalid);
  EXPECT_TRUE(ReplyBufUntouched());
  EXPECT_EQ(fs_->Seek(kClient, h, 0, kFsSeekCur), 0);
  EXPECT_EQ(Dispatch(ReadReq(h, 4), 4), 4);
  EXPECT_EQ(Dispatch(ReadReq(h, 0), 0), 0);
  EXPECT_EQ(fs_->Seek(kClient, h, 0, kFsSeekCur), 4);

  // ReadDir: room for `max_entries` entries, and the cursor stays.
  uint64_t dir_h = 0;
  ASSERT_EQ(OpenStr(kClient, "/", kFsOpenRead | kFsOpenDirectory, &dir_h), 0);
  const int64_t two_entries = 2 * static_cast<int64_t>(sizeof(FsDirEntry));
  PoisonReplyBuf();
  EXPECT_EQ(Dispatch(ReadDirReq(dir_h, 2), two_entries - 1),
            kSyscallErrorInvalid);
  EXPECT_TRUE(ReplyBufUntouched());
  EXPECT_EQ(fs_->Seek(kClient, dir_h, 0, kFsSeekCur), 0);
  EXPECT_EQ(Dispatch(ReadDirReq(dir_h, 2), two_entries), 1);

  // Requests without reply bytes need no room.
  EXPECT_EQ(Dispatch(WriteReq(h, "ab"), 0), 2);
  EXPECT_EQ(Dispatch(SeekReq(h, 0, kFsSeekEnd), 0), 10);
  EXPECT_EQ(Dispatch(PathReq(FsRpcOp::kMkDir, "/d"), 0), 0);
  EXPECT_EQ(Dispatch(RenameReq("/d", "/e"), 0), 0);
  EXPECT_EQ(Dispatch(PathReq(FsRpcOp::kUnlink, "/e"), 0), 0);
  EXPECT_EQ(Dispatch(CloseReq(h), 0), 0);

  // A negative capacity, or a positive one without a buffer, is invalid.
  EXPECT_EQ(Dispatch(CloseReq(dir_h), -1), kSyscallErrorInvalid);
  const std::vector<uint8_t> close_dir = CloseReq(dir_h);
  EXPECT_EQ(DispatchRpc(*fs_,                                    //
                        kClient,                                 //
                        close_dir.data(),                        //
                        static_cast<int64_t>(close_dir.size()),  //
                        nullptr,                                 //
                        1,                                       //
                        &reply_size_),
            kSyscallErrorInvalid);
  EXPECT_EQ(fs_->ActiveHandleCount(), 1);
  EXPECT_EQ(Dispatch(CloseReq(dir_h), 0), 0);
}

TEST_F(RamfsRpcTest, ServeOneHandlesRecvErrorsDisconnectsCapAndReplyFallback) {
  struct FakeIpc {
    int64_t recv_rc = 0;
    IpcReceivedMessage recv_msg{};
    std::vector<uint8_t> recv_payload;
    std::vector<int64_t> reply_results;

    struct SentReply {
      int64_t port = -1;
      uint64_t token = 0;
      std::vector<uint8_t> bytes;
      int64_t status = 0;
    };
    std::vector<SentReply> replies;
    int64_t last_recv_port = -1;
    int64_t last_recv_timeout = 0;

    static int64_t Recv(void* const ctx,                    //
                        const int64_t port,                 //
                        void* const buf,                    //
                        const int64_t cap,                  //
                        IpcReceivedMessage* const out_msg,  //
                        const int64_t timeout_ticks) {
      auto* const self = static_cast<FakeIpc*>(ctx);
      self->last_recv_port = port;
      self->last_recv_timeout = timeout_ticks;
      if (self->recv_rc <= 0) {
        return self->recv_rc;
      }
      const int64_t copy_len = std::min<int64_t>(
          cap, static_cast<int64_t>(self->recv_payload.size()));
      if (copy_len > 0) {
        std::memcpy(buf, self->recv_payload.data(),
                    static_cast<size_t>(copy_len));
      }
      *out_msg = self->recv_msg;
      out_msg->size = copy_len;
      return self->recv_rc;
    }

    static int64_t Reply(void* const ctx,        //
                         const int64_t port,     //
                         const uint64_t token,   //
                         const void* const buf,  //
                         const int64_t size,     //
                         const int64_t status) {
      auto* const self = static_cast<FakeIpc*>(ctx);
      SentReply sent;
      sent.port = port;
      sent.token = token;
      sent.status = status;
      if (size > 0 && buf != nullptr) {
        const auto* const bytes = static_cast<const uint8_t*>(buf);
        sent.bytes.assign(bytes, bytes + size);
      }
      self->replies.push_back(std::move(sent));
      if (!self->reply_results.empty()) {
        const int64_t res = self->reply_results.front();
        self->reply_results.erase(self->reply_results.begin());
        return res;
      }
      return 0;
    }
  };

  FakeIpc ipc;
  const ServeEnv env{
      .ctx = &ipc,
      .recv = &FakeIpc::Recv,
      .reply = &FakeIpc::Reply,
  };
  uint8_t req_buf[kIpcMaxMessageSize] = {};
  uint8_t reply_buf[kIpcMaxMessageSize] = {};

  // Fatal recv errors stop the serve loop; transient ones keep going.
  ipc.recv_rc = kSyscallErrorAccess;
  EXPECT_FALSE(ServeOne(*fs_, env, req_buf, sizeof(req_buf), reply_buf,
                        sizeof(reply_buf)));
  EXPECT_EQ(ipc.last_recv_port, kIpcPortFs);
  EXPECT_EQ(ipc.last_recv_timeout, -1);

  ipc.recv_rc = kSyscallErrorInvalid;
  EXPECT_FALSE(ServeOne(*fs_, env, req_buf, sizeof(req_buf), reply_buf,
                        sizeof(reply_buf)));

  for (const int64_t transient : {int64_t{0}, kSyscallErrorNoMemory}) {
    ipc.recv_rc = transient;
    EXPECT_TRUE(ServeOne(*fs_, env, req_buf, sizeof(req_buf), reply_buf,
                         sizeof(reply_buf)));
  }
  EXPECT_TRUE(ipc.replies.empty());

  // kCall opens a file and replies with FsOpenReply.
  ipc.recv_rc = 1;
  ipc.recv_payload = OpenReq("/a", kFsOpenReadWrite | kFsOpenCreate);
  ipc.recv_msg = IpcReceivedMessage{
      .token = 11,
      .client_id = kClient,
      .size = static_cast<int64_t>(ipc.recv_payload.size()),
      .kind = IpcMessageKind::kCall,
      .reply_cap = static_cast<int32_t>(sizeof(FsOpenReply)),
  };
  EXPECT_TRUE(ServeOne(*fs_, env, req_buf, sizeof(req_buf), reply_buf,
                       sizeof(reply_buf)));
  ASSERT_EQ(ipc.replies.size(), 1u);
  EXPECT_EQ(ipc.replies[0].port, kIpcPortFs);
  EXPECT_EQ(ipc.replies[0].token, 11u);
  EXPECT_EQ(ipc.replies[0].status, 0);
  ASSERT_EQ(ipc.replies[0].bytes.size(), sizeof(FsOpenReply));
  EXPECT_EQ(fs_->ActiveHandleCount(), 1);

  // Open a handle for a second client directly so disconnect isolation shows.
  uint64_t other_h = 0;
  ASSERT_EQ(OpenStr(kClient + 1, "/a", kFsOpenRead, &other_h), 0);
  EXPECT_EQ(fs_->ActiveHandleCount(), 2);

  // kClientDisconnect closes only that client's handles and sends no reply.
  ipc.replies.clear();
  ipc.recv_payload.clear();
  ipc.recv_msg = IpcReceivedMessage{
      .token = 0,
      .client_id = kClient,
      .size = 0,
      .kind = IpcMessageKind::kClientDisconnect,
      .reply_cap = 0,
  };
  EXPECT_TRUE(ServeOne(*fs_, env, req_buf, sizeof(req_buf), reply_buf,
                       sizeof(reply_buf)));
  EXPECT_TRUE(ipc.replies.empty());
  EXPECT_EQ(fs_->ActiveHandleCount(), 1);

  // Clamps reply_cap to min(msg.reply_cap, reply_buf_cap) before dispatching.
  ipc.recv_payload = PathReq(FsRpcOp::kStat, "/a");
  ipc.recv_msg = IpcReceivedMessage{
      .token = 12,
      .client_id = kClient + 1,
      .size = static_cast<int64_t>(ipc.recv_payload.size()),
      .kind = IpcMessageKind::kCall,
      .reply_cap = static_cast<int32_t>(sizeof(FsStat)),
  };
  EXPECT_TRUE(ServeOne(*fs_, env, req_buf, sizeof(req_buf), reply_buf,
                       /*reply_buf_cap=*/sizeof(FsStat) - 1));
  ASSERT_EQ(ipc.replies.size(), 1u);
  EXPECT_EQ(ipc.replies[0].status, kSyscallErrorInvalid);
  EXPECT_TRUE(ipc.replies[0].bytes.empty());

  ipc.replies.clear();
  ipc.recv_msg.token = 13;
  ipc.recv_msg.reply_cap = static_cast<int32_t>(sizeof(FsStat) - 1);
  EXPECT_TRUE(ServeOne(*fs_, env, req_buf, sizeof(req_buf), reply_buf,
                       sizeof(reply_buf)));
  ASSERT_EQ(ipc.replies.size(), 1u);
  EXPECT_EQ(ipc.replies[0].status, kSyscallErrorInvalid);

  // When the primary reply fails, ServeOne falls back to kSyscallErrorIo.
  ipc.replies.clear();
  ipc.reply_results = {kSyscallErrorInvalid, 0};
  ipc.recv_msg.token = 14;
  ipc.recv_msg.reply_cap = static_cast<int32_t>(sizeof(FsStat));
  EXPECT_TRUE(ServeOne(*fs_, env, req_buf, sizeof(req_buf), reply_buf,
                       sizeof(reply_buf)));
  ASSERT_EQ(ipc.replies.size(), 2u);
  EXPECT_EQ(ipc.replies[0].token, 14u);
  EXPECT_EQ(ipc.replies[0].status, 0);
  EXPECT_EQ(ipc.replies[0].bytes.size(), sizeof(FsStat));
  EXPECT_EQ(ipc.replies[1].token, 14u);
  EXPECT_EQ(ipc.replies[1].status, kSyscallErrorIo);
  EXPECT_TRUE(ipc.replies[1].bytes.empty());

  EXPECT_EQ(fs_->Close(kClient + 1, other_h), 0);
}

TEST_F(RamfsTest, RenameWithinOneDirectoryHandlesBlockGrowthAndShrink) {
  ASSERT_EQ(MkDirStr("/d"), 0);
  // Fill one DirBlock completely, rename the last slot into a new name (forcing
  // InsertInDir to append a second block), then rename that entry onto the
  // first entry (freeing the second block cleanly).
  const int count = DirBlock::kSlotsPerBlock;
  for (int i = 0; i < count; ++i) {
    ASSERT_EQ(CreateFile("/d/f" + std::to_string(i)), 0);
  }
  const int blocks_full = alloc_.live_blocks();
  const std::string last = "/d/f" + std::to_string(count - 1);
  ASSERT_EQ(RenameStr(last, "/d/moved"), 0);
  EXPECT_EQ(alloc_.live_blocks(), blocks_full + 1);
  FsStat st{};
  EXPECT_EQ(StatStr(last, &st), kSyscallErrorNoEntry);
  EXPECT_EQ(StatStr("/d/moved", &st), 0);

  ASSERT_EQ(RenameStr("/d/moved", "/d/f0"), 0);
  EXPECT_EQ(alloc_.live_blocks(), blocks_full);
  EXPECT_EQ(StatStr("/d/moved", &st), kSyscallErrorNoEntry);
  EXPECT_EQ(StatStr("/d/f0", &st), 0);
}

TEST(SlabBlockAllocatorTest, BatchesSlabsFallsBackToSingleBlocksAndRezeroes) {
  static_assert(std::is_trivially_destructible_v<SlabBlockAllocator>);

  SlabBlockAllocator null_alloc{nullptr};
  EXPECT_EQ(null_alloc.AllocBlock(), nullptr);
  null_alloc.FreeBlock(nullptr);

  struct FakeMapper {
    int64_t remaining_blocks = SlabBlockAllocator::kSlabBlocks + 2;
    std::vector<int64_t> requested_sizes;
    std::vector<std::unique_ptr<uint8_t[]>> regions;

    static StatusOr<uintptr_t> Map(const int64_t size, void* const ctx) {
      auto* const self = static_cast<FakeMapper*>(ctx);
      self->requested_sizes.push_back(size);
      const int64_t blocks = size / kBlockSize;
      if (size <= 0 || (size % kBlockSize) != 0 ||
          blocks > self->remaining_blocks) {
        return Error(kSyscallErrorNoMemory);
      }
      self->remaining_blocks -= blocks;
      auto mem = std::make_unique<uint8_t[]>(static_cast<size_t>(size));
      __builtin_memset(mem.get(), 0, static_cast<size_t>(size));
      const uintptr_t addr = reinterpret_cast<uintptr_t>(mem.get());
      self->regions.push_back(std::move(mem));
      return addr;
    }
  } mapper;

  SlabBlockAllocator alloc{&FakeMapper::Map, &mapper};

  // First allocation maps one 16-block slab; the next 15 come from its free
  // list without invoking `Map`.
  std::vector<uint8_t*> first_slab;
  for (int i = 0; i < SlabBlockAllocator::kSlabBlocks; ++i) {
    uint8_t* const blk = alloc.AllocBlock();
    ASSERT_NE(blk, nullptr);
    for (int64_t b = 0; b < kBlockSize; ++b) {
      ASSERT_EQ(blk[b], 0u);
    }
    first_slab.push_back(blk);
  }
  EXPECT_THAT(mapper.requested_sizes,
              t::ElementsAre(SlabBlockAllocator::kSlabBlocks * kBlockSize));
  for (int i = 1; i < SlabBlockAllocator::kSlabBlocks; ++i) {
    EXPECT_EQ(first_slab[i], first_slab[0] + i * kBlockSize);
  }

  // With only 2 blocks left, a 16-block map fails, falls back to 1 block, and
  // stays in single-block mode for the next block until space runs out.
  uint8_t* const single1 = alloc.AllocBlock();
  ASSERT_NE(single1, nullptr);
  uint8_t* const single2 = alloc.AllocBlock();
  ASSERT_NE(single2, nullptr);
  EXPECT_EQ(alloc.AllocBlock(), nullptr);
  EXPECT_THAT(mapper.requested_sizes,
              t::ElementsAre(SlabBlockAllocator::kSlabBlocks * kBlockSize,
                             SlabBlockAllocator::kSlabBlocks * kBlockSize,
                             kBlockSize, kBlockSize, kBlockSize));

  // Dirtying and freeing blocks recycles them in LIFO order and zeroes every
  // byte (including the intrusive `FreeNode::next` pointer at offset 0).
  alloc.FreeBlock(nullptr);
  __builtin_memset(single1, 0xA5, static_cast<size_t>(kBlockSize));
  __builtin_memset(single2, 0x5A, static_cast<size_t>(kBlockSize));
  alloc.FreeBlock(single1);
  alloc.FreeBlock(single2);
  const int maps_before_reuse = static_cast<int>(mapper.requested_sizes.size());

  uint8_t* const reused2 = alloc.AllocBlock();
  uint8_t* const reused1 = alloc.AllocBlock();
  EXPECT_EQ(reused2, single2);
  EXPECT_EQ(reused1, single1);
  EXPECT_EQ(static_cast<int>(mapper.requested_sizes.size()), maps_before_reuse);
  for (int64_t b = 0; b < kBlockSize; ++b) {
    ASSERT_EQ(reused2[b], 0u);
    ASSERT_EQ(reused1[b], 0u);
  }
}

}  // namespace
}  // namespace protos::ramfs
