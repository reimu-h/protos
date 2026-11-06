#include "user/ramfs/ramfs.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
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

  // Fills the reply buffer with a pattern that `ReplyBufUntouched()` checks.
  void PoisonReplyBuf() { std::memset(raw_reply_, 0xA5, sizeof(raw_reply_)); }

  bool ReplyBufUntouched() const {
    for (const uint8_t byte : raw_reply_) {
      if (byte != 0xA5) {
        return false;
      }
    }
    return true;
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
  EXPECT_EQ(Dispatch({1, 2}), kSyscallErrorInvalid);
  EXPECT_EQ(DispatchRpc(*fs_,                //
                        kClient,             //
                        nullptr,             //
                        16,                  //
                        reply_buf_,          //
                        kIpcMaxMessageSize,  //
                        &reply_size_),
            kSyscallErrorInvalid);
  EXPECT_EQ(Dispatch(Req(FsRpcOp::kNone)), kSyscallErrorInvalid);
  EXPECT_EQ(Dispatch(Req(FsPathRequest{.op = FsRpcOp::kNone, .path_len = 2},
                         std::string("/g"))),
            kSyscallErrorInvalid);
  EXPECT_EQ(Dispatch(Req(static_cast<FsRpcOp>(11))), kSyscallErrorInvalid);
  EXPECT_EQ(Dispatch(Req(static_cast<FsRpcOp>(0xFFFFFFFF))),
            kSyscallErrorInvalid);

  // Fixed-size requests one byte short or long.
  for (const std::vector<uint8_t>& req :
       {CloseReq(h), ReadReq(h, 1), SeekReq(h, 0, kFsSeekSet),
        ReadDirReq(dir_h, 1)}) {
    const std::vector<uint8_t> shorter(req.begin(), req.end() - 1);
    std::vector<uint8_t> longer = req;
    longer.push_back(0);
    EXPECT_EQ(Dispatch(shorter), kSyscallErrorInvalid);
    EXPECT_EQ(Dispatch(longer), kSyscallErrorInvalid);
  }
  // Headers of variable-size requests one byte short.
  for (const std::vector<uint8_t>& req :
       {Req(FsOpenRequest{.op = FsRpcOp::kOpen}),
        Req(FsWriteRequest{.op = FsRpcOp::kWrite, .handle = h}),
        Req(FsPathRequest{.op = FsRpcOp::kStat}),
        Req(FsRenameRequest{.op = FsRpcOp::kRename})}) {
    EXPECT_EQ(Dispatch(std::vector<uint8_t>(req.begin(), req.end() - 1)),
              kSyscallErrorInvalid);
  }

  // Non-zero reserved fields.
  EXPECT_EQ(Dispatch(Req(FsCloseRequest{
                .op = FsRpcOp::kClose, .reserved = 1, .handle = h})),
            kSyscallErrorInvalid);
  EXPECT_EQ(Dispatch(Req(FsReadRequest{
                .op = FsRpcOp::kRead, .reserved = 1, .handle = h, .size = 1})),
            kSyscallErrorInvalid);
  EXPECT_EQ(
      Dispatch(Req(
          FsWriteRequest{
              .op = FsRpcOp::kWrite, .reserved = 1, .handle = h, .size = 1},
          std::string("x"))),
      kSyscallErrorInvalid);
  EXPECT_EQ(Dispatch(Req(FsRenameRequest{.op = FsRpcOp::kRename,
                                         .old_len = 2,
                                         .new_len = 2,
                                         .reserved = 1},
                         std::string("/f/g"))),
            kSyscallErrorInvalid);

  // Path lengths that are negative or don't match the request size.
  for (const int32_t path_len : {-1, 1, 3, kInt32Max}) {
    EXPECT_EQ(Dispatch(Req(FsOpenRequest{.op = FsRpcOp::kOpen,
                                         .path_len = path_len,
                                         .flags = kFsOpenRead},
                           std::string("/f"))),
              kSyscallErrorInvalid);
    for (const FsRpcOp op :
         {FsRpcOp::kStat, FsRpcOp::kMkDir, FsRpcOp::kUnlink}) {
      EXPECT_EQ(Dispatch(Req(FsPathRequest{.op = op, .path_len = path_len},
                             std::string("/g"))),
                kSyscallErrorInvalid);
    }
  }
  EXPECT_EQ(Dispatch(Req(FsOpenRequest{.op = FsRpcOp::kOpen, .path_len = -1})),
            kSyscallErrorInvalid);

  // Write sizes that are out of range or don't match the data.
  for (const int64_t size :
       {int64_t{-1}, int64_t{3}, int64_t{5}, kInt64Max, kInt64Min}) {
    EXPECT_EQ(
        Dispatch(Req(
            FsWriteRequest{.op = FsRpcOp::kWrite, .handle = h, .size = size},
            std::string("abcd"))),
        kSyscallErrorInvalid);
  }
  EXPECT_EQ(Dispatch(Req(FsWriteRequest{
                .op = FsRpcOp::kWrite, .handle = h, .size = kInt64Max})),
            kSyscallErrorInvalid);
  const std::string too_much(static_cast<size_t>(kSyscallFsMaxIoSize + 1), 'x');
  EXPECT_EQ(Dispatch(WriteReq(h, too_much)), kSyscallErrorInvalid);

  // Rename lengths that are negative or don't add up to the request size.
  for (const auto& [old_len, new_len] :
       std::vector<std::pair<int32_t, int32_t>>{
           {-1, 5}, {5, -1}, {2, 1}, {2, 3}, {kInt32Max, kInt32Max}}) {
    EXPECT_EQ(Dispatch(Req(FsRenameRequest{.op = FsRpcOp::kRename,
                                           .old_len = old_len,
                                           .new_len = new_len},
                           std::string("/f/g"))),
              kSyscallErrorInvalid);
  }

  // Read sizes and ReadDir counts out of range.
  for (const int64_t size :
       {int64_t{-1}, kSyscallFsMaxIoSize + 1, kInt64Max, kInt64Min}) {
    EXPECT_EQ(Dispatch(ReadReq(h, size)), kSyscallErrorInvalid);
  }
  for (const int32_t max_entries :
       {int32_t{-1}, static_cast<int32_t>(kSyscallFsMaxDirEntries + 1),
        kInt32Max}) {
    EXPECT_EQ(Dispatch(ReadDirReq(dir_h, max_entries)), kSyscallErrorInvalid);
  }
  EXPECT_EQ(reply_size_, 0);

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
