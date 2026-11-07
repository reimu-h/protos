#ifndef PROTOS_USER_RAMFS_ROOTFS_H_
#define PROTOS_USER_RAMFS_ROOTFS_H_

#include <cstdint>

#include "syscall_abi.h"
#include "user/ramfs/ramfs.h"
#include "user/ramfs/tar.h"

// Freestanding, header-only loader of the root filesystem
// (`ramfs::LoadArchive()`): at startup, `ramfs` unpacks the ustar archive of
// the repository's rootfs/ directory, which the boot loader loads as a boot
// module, into its `ramfs::FileSystem`. Host unit tests load archives from
// memory.

namespace protos::ramfs {

// The client ID with which LoadArchive() opens files: the kernel never assigns
// it.
constexpr uint64_t kLoaderClientId = ~uint64_t{0};

// What LoadArchive() did.
struct LoadStats {
  // Regular files written.
  int files = 0;
  // Directories created, the missing parents of members included.
  int directories = 0;
  // Members that `tar::Reader` returned as `tar::EntryType::kSkipped`.
  int skipped = 0;
  // Members that could not be created or written.
  int failed = 0;
};

namespace rootfs_internal {

// The most bytes that LoadArchive() copies at a time. Its buffer is on the
// stack, which has 16 KiB in a user program.
constexpr int64_t kCopyChunkSize = 1024;
static_assert(kCopyChunkSize <= kSyscallFsMaxIoSize);

// How LoadArchive() opens the file of a member.
constexpr uint64_t kFileOpenFlags =
    kFsOpenWrite | kFsOpenCreate | kFsOpenTruncate;

// Creates the directories that the `path_len`-byte normalized absolute path
// `path` (`tar::Entry::path`) names, from the outermost on: its ancestors but
// the root, and the path itself if `including_path`. Directories that exist
// already are fine. Adds the number of directories that it created to
// `*created`, and returns false if one of the paths is a regular file or
// MkDir() fails otherwise (with `kSyscallErrorNoSpace`, for example).
inline bool MakeDirectories(FileSystem& fs,             //
                            const char* const path,     //
                            const int64_t path_len,     //
                            const bool including_path,  //
                            int* const created) {
  // The shortest path below the root is "/" and one byte.
  for (int64_t len = 2; len <= path_len; ++len) {
    // The first `len` bytes name an ancestor if a '/' follows them.
    const bool ancestor = len < path_len && path[len] == '/';
    if (!ancestor && !(len == path_len && including_path)) {
      continue;
    }
    const int64_t rc = fs.MkDir(path, len);
    if (rc == 0) {
      ++*created;
      continue;
    }
    FsStat existing{};
    if (rc != kSyscallErrorExists || fs.Stat(path, len, &existing) != 0 ||
        existing.type != FsNodeType::kDirectory) {
      return false;
    }
  }
  return true;
}

// Writes the regular file `entry`, whose parent directory exists, to `fs` with
// the data that `reader` reads, and counts it in `*stats`, as a file or as
// failed. A file that failed (because it doesn't fit, for example) is removed,
// also if a file of an earlier member had its path. Returns 0, also for a file
// that failed, or the error of `reader.ReadData()`.
inline int64_t LoadFile(FileSystem& fs,           //
                        tar::Reader& reader,      //
                        const tar::Entry& entry,  //
                        LoadStats* const stats) {
  if (entry.size > kFsMaxFileSize) {
    // Reject oversized files before Open()/Write() so we neither stream 32 MiB
    // nor grow the slab allocator for blocks that will be freed immediately.
    FsStat existing{};
    if (fs.Stat(entry.path, entry.path_len, &existing) == 0 &&
        existing.type == FsNodeType::kRegular) {
      (void)fs.Unlink(entry.path, entry.path_len);
    }
    ++stats->failed;
    return 0;
  }

  FsOpenReply reply{};
  const int64_t open_rc = fs.Open(kLoaderClientId,  //
                                  entry.path,       //
                                  entry.path_len,   //
                                  kFileOpenFlags,   //
                                  &reply);
  if (open_rc < 0) {
    // A failed Open() changes nothing.
    ++stats->failed;
    return 0;
  }

  int64_t read_error = 0;
  bool written = true;
  uint8_t chunk[kCopyChunkSize];
  for (int64_t pos = 0; pos < entry.size;) {
    const int64_t left = entry.size - pos;
    const int64_t want = (left < kCopyChunkSize) ? left : kCopyChunkSize;
    const int64_t got = reader.ReadData(entry, pos, chunk, want);
    if (got <= 0) {
      read_error = (got < 0) ? got : kSyscallErrorIo;
      break;
    }
    // A short or failed write means that `fs` is out of space or that the
    // file reached `kFsMaxFileSize` bytes.
    if (fs.Write(kLoaderClientId, reply.handle, chunk, got) != got) {
      written = false;
      break;
    }
    pos += got;
  }
  const bool closed = fs.Close(kLoaderClientId, reply.handle) == 0;
  if (read_error == 0 && written && closed) {
    ++stats->files;
    return 0;
  }
  (void)fs.Unlink(entry.path, entry.path_len);
  ++stats->failed;
  return read_error;
}

}  // namespace rootfs_internal

// Unpacks the archive that `reader` reads into `fs` (which must be
// initialized), adding to `*stats` unless `stats` is nullptr: creates its
// directories, writes its regular files (over the file of an earlier member
// with the same path), and creates their missing ancestors. A member that
// fails (one that doesn't fit, or a file where a directory is, for example)
// doesn't stop it, and leaves no partial file. Returns 0 once the reader
// reached the end of the archive, or the negative error of the reader that
// stopped it (the members before it stay). Leaves no file of
// `kLoaderClientId` open.
[[nodiscard]] inline int64_t LoadArchive(FileSystem& fs,       //
                                         tar::Reader& reader,  //
                                         LoadStats* const stats) {
  LoadStats unused{};
  LoadStats* const out = (stats != nullptr) ? stats : &unused;
  tar::Entry entry{};
  int64_t result = 0;
  while (result == 0) {
    const int64_t next_rc = reader.Next(&entry);
    if (next_rc <= 0) {
      result = next_rc;
      break;
    }
    if (entry.type == tar::EntryType::kSkipped) {
      ++out->skipped;
      continue;
    }
    // Creates the directory, or the missing ancestors of the file. Doing so
    // here rather than in LoadFile(), whose frame holds the copy buffer, keeps
    // the stack small.
    const bool is_file = entry.type == tar::EntryType::kRegular;
    if (!rootfs_internal::MakeDirectories(fs,                           //
                                          entry.path,                   //
                                          entry.path_len,               //
                                          /*including_path=*/!is_file,  //
                                          &out->directories)) {
      ++out->failed;
    } else if (is_file) {
      // 0, or an error of the reader, which ends the loop.
      result = rootfs_internal::LoadFile(fs, reader, entry, out);
    }
  }
  // LoadFile() closes its files; this is a safety net.
  fs.CloseAllForClient(kLoaderClientId);
  return result;
}

}  // namespace protos::ramfs

#endif  // PROTOS_USER_RAMFS_ROOTFS_H_
