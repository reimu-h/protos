#ifndef PROTOS_USER_RAMFS_RAMFS_H_
#define PROTOS_USER_RAMFS_RAMFS_H_

#include <cstddef>
#include <cstdint>
#include <new>

#include "fs_protocol.h"
#include "status_or.h"
#include "syscall_abi.h"

// Freestanding, header-only in-memory filesystem core (`ramfs::FileSystem`) and
// wire-protocol RPC dispatcher (`ramfs::DispatchRpc`) for `user/ramfs`.
// Parameterized over `ramfs::BlockAllocator` so host unit tests can exercise
// path resolution, sparse and indirect block trees, directory cookies, atomic
// rename, unlink-while-open, client-disconnect reclamation, and out-of-space
// rollback without system calls.

namespace protos::ramfs {

constexpr int64_t kBlockSize = 4096;
constexpr int kMaxFnodes = 256;
constexpr int kMaxOpenHandles = 256;
constexpr int kDirectBlocks = 12;
constexpr int kPtrsPerIndirectBlock =
    static_cast<int>(kBlockSize / static_cast<int64_t>(sizeof(uint8_t*)));
static_assert(kPtrsPerIndirectBlock == 512);
// The direct, single-indirect and double-indirect blocks cover the largest
// file.
static_assert((kDirectBlocks + kPtrsPerIndirectBlock +
               int64_t{kPtrsPerIndirectBlock} * kPtrsPerIndirectBlock) *
                  kBlockSize >=
              kFsMaxFileSize);

// Host/ring-3 block allocator seam. Protected non-virtual destructor avoids
// requiring `operator delete` in freestanding ring-3 binaries.
class BlockAllocator {
 public:
  // Returns a zeroed 4 KiB block, or nullptr if out of space.
  virtual uint8_t* AllocBlock() = 0;
  // Returns a 4 KiB block previously obtained from `AllocBlock()`.
  virtual void FreeBlock(uint8_t* block) = 0;

 protected:
  constexpr BlockAllocator() = default;
  ~BlockAllocator() = default;
};

// Block allocator backed by a page-mapping callback (`MapPagesFn`), such as
// `user::MemMap` in ring 3. Maps `kSlabBlocks` 4 KiB blocks at a time, falls
// back permanently to single-block mappings once a slab map fails, and recycles
// freed blocks on an intrusive free list (zeroing them on reuse). Trivially
// destructible and `constexpr`-constructible for `constinit` globals in
// freestanding ring-3 binaries.
class SlabBlockAllocator final : public BlockAllocator {
 public:
  static constexpr int kSlabBlocks = 16;
  using MapPagesFn = StatusOr<uintptr_t> (*)(int64_t size, void* ctx);

  constexpr explicit SlabBlockAllocator(const MapPagesFn map_pages,
                                        void* const ctx = nullptr)
      : map_pages_(map_pages), ctx_(ctx) {}

  uint8_t* AllocBlock() override {
    if (free_head_ != nullptr) {
      FreeNode* const node = free_head_;
      free_head_ = node->next;
      uint8_t* const blk = reinterpret_cast<uint8_t*>(node);
      __builtin_memset(blk, 0, static_cast<size_t>(kBlockSize));
      return blk;
    }
    if (map_pages_ == nullptr) {
      return nullptr;
    }

    // Map `kSlabBlocks` blocks at a time, and single blocks once that fails:
    // near the mapping limit or with little physical memory left, one block
    // may still fit.
    int slab_blocks = single_block_maps_ ? 1 : kSlabBlocks;
    StatusOr<uintptr_t> mapped = map_pages_(slab_blocks * kBlockSize, ctx_);
    if (!mapped.ok() && slab_blocks > 1) {
      single_block_maps_ = true;
      slab_blocks = 1;
      mapped = map_pages_(kBlockSize, ctx_);
    }
    if (!mapped.ok()) {
      return nullptr;
    }

    uint8_t* const base = reinterpret_cast<uint8_t*>(*mapped);
    for (int i = slab_blocks - 1; i >= 1; --i) {
      FreeBlock(base + static_cast<int64_t>(i) * kBlockSize);
    }
    return base;
  }

  void FreeBlock(uint8_t* const block) override {
    if (block == nullptr) {
      return;
    }
    FreeNode* const node = reinterpret_cast<FreeNode*>(block);
    node->next = free_head_;
    free_head_ = node;
  }

 private:
  struct FreeNode {
    FreeNode* next = nullptr;
  };

  MapPagesFn map_pages_ = nullptr;
  void* ctx_ = nullptr;
  FreeNode* free_head_ = nullptr;
  // Set once mapping a slab failed.
  bool single_block_maps_ = false;
};

// One block of a directory's entries. A directory keeps a list of them and
// frees each one that becomes empty.
struct DirBlock {
  struct Slot {
    // 0 = free; > 0 = monotonically increasing directory entry cookie.
    uint64_t seq = 0;
    int32_t fnode_idx = -1;
    int32_t name_len = 0;
    char name[kFsMaxNameLength + 1] = {};
  };

  // The bytes of `next`, `used_count` and `reserved`, before `slots`.
  static constexpr int64_t kHeaderSize = 16;
  // As many slots as fit into the rest of the block.
  static constexpr int kSlotsPerBlock = static_cast<int>(
      (kBlockSize - kHeaderSize) / static_cast<int64_t>(sizeof(Slot)));

  DirBlock* next = nullptr;
  int32_t used_count = 0;
  int32_t reserved = 0;
  Slot slots[kSlotsPerBlock] = {};
};
static_assert(static_cast<int64_t>(offsetof(DirBlock, slots)) ==
              DirBlock::kHeaderSize);
static_assert(sizeof(DirBlock) <= static_cast<size_t>(kBlockSize));
static_assert(alignof(DirBlock) <= 8);

// The filesystem: up to `kMaxFnodes` files and directories, the root `/`
// included, and one table of up to `kMaxOpenHandles` open files and
// directories for all clients. The methods from Open() to Rename() implement
// the filesystem system calls (syscall_abi.h), with server handles in place of
// file descriptors.
//
// A handle is `generation << 16 | (slot + 1)`: `slot` indexes the handle
// table, and `generation` is new for each Open(), so that a closed handle
// stays invalid when its slot is reused. A handle belongs to the client that
// opened it: other clients' requests with it fail with `kSyscallErrorBadFd`.
//
// A regular file holds up to `kFsMaxFileSize` bytes in direct, single-indirect
// and double-indirect blocks; holes take no blocks. A write stops at
// `kFsMaxFileSize` bytes (a short write), and one of `size > 0` that starts
// there fails with `kSyscallErrorFileTooBig`.
//
// A directory's size is `sizeof(FsDirEntry)` per entry. Each entry gets a
// cookie larger than those of all entries added to the directory before it
// (Rename() adds the entry anew), and ReadDir() returns entries in cookie
// order and keeps the last one's cookie as the handle's offset. So across
// ReadDir() calls, entries that stay appear exactly once, removed ones stop
// appearing, and added ones appear in later calls. On a directory, Seek() only
// accepts `(0, kFsSeekSet)`, which rewinds, and `(0, kFsSeekCur)`, which
// returns the offset as an opaque cookie.
class FileSystem {
 public:
  constexpr FileSystem() = default;

  void Initialize(BlockAllocator* const alloc) {
    alloc_ = alloc;
    for (int i = 0; i < kMaxFnodes; ++i) {
      fnodes_[i] = Fnode{};
    }
    for (int i = 0; i < kMaxOpenHandles; ++i) {
      handles_[i] = OpenHandle{};
    }
    next_fnode_id_ = 1;
    next_handle_gen_ = 1;
    // Allocate root directory `/` at fnode slot 0 (`fnode_id == 1`).
    Fnode& root = fnodes_[kRootFnodeIdx];
    root.in_use = true;
    root.type = FsNodeType::kDirectory;
    root.fnode_id = next_fnode_id_++;
    root.nlink = 2;
    root.parent_idx = kRootFnodeIdx;
    root.next_dir_seq = 1;
  }

  // Opens or creates `[path, path + path_len)` for `client_id` (`!= 0`). Writes
  // the non-zero server handle and node type on success and returns 0, or a
  // negative `kSyscallError*`.
  int64_t Open(const uint64_t client_id,  //
               const char* const path,    //
               const int64_t path_len,    //
               const uint64_t flags,      //
               FsOpenReply* const out_reply) {
    if (client_id == 0 || out_reply == nullptr || !FsOpenFlagsValid(flags)) {
      return kSyscallErrorInvalid;
    }

    // A failed Open() changes nothing, so this comes before any change.
    const int handle_slot = FindFreeHandleSlot();
    if (handle_slot < 0) {
      return kSyscallErrorFileTableFull;
    }

    const uint64_t access = flags & kFsOpenAccessMask;
    const bool want_write =
        (access == kFsOpenWrite || access == kFsOpenReadWrite);
    const bool want_create = (flags & kFsOpenCreate) != 0;
    const bool want_trunc = (flags & kFsOpenTruncate) != 0;
    const bool want_excl = (flags & kFsOpenExclusive) != 0;
    const bool want_dir = (flags & kFsOpenDirectory) != 0;
    const bool want_append = (flags & kFsOpenAppend) != 0;

    ParsedParent parsed{};
    const int64_t parse_rc = ResolveParent(path, path_len, &parsed);
    if (parse_rc < 0) {
      return parse_rc;
    }

    int target_idx = -1;
    if (parsed.is_self_or_dotdot) {
      target_idx = parsed.resolved_dir_idx;
    } else {
      target_idx = LookupInDir(parsed.parent_idx,  //
                               parsed.name,        //
                               parsed.name_len,    //
                               nullptr,            //
                               nullptr);
    }

    if (target_idx >= 0) {
      if (want_create && want_excl) {
        return kSyscallErrorExists;
      }
      Fnode& node = fnodes_[target_idx];
      if (node.type == FsNodeType::kDirectory) {
        if (want_write || want_create || want_trunc || want_append) {
          return kSyscallErrorIsDir;
        }
      } else {
        if (want_dir || parsed.trailing_slash) {
          return kSyscallErrorNotDir;
        }
        if (want_trunc) {
          FreeAllFileBlocks(node);
          node.size = 0;
        }
      }
    } else {
      if (!want_create) {
        return kSyscallErrorNoEntry;
      }
      if (want_dir || parsed.trailing_slash) {
        return kSyscallErrorIsDir;
      }
      const int new_idx = AllocFnode(FsNodeType::kRegular, parsed.parent_idx);
      if (new_idx < 0) {
        return kSyscallErrorNoSpace;
      }
      const int64_t link_rc =
          InsertInDir(parsed.parent_idx, parsed.name, parsed.name_len, new_idx);
      if (link_rc < 0) {
        fnodes_[new_idx] = Fnode{};
        return link_rc;
      }
      fnodes_[new_idx].nlink = 1;
      target_idx = new_idx;
    }

    Fnode& node = fnodes_[target_idx];
    ++node.open_refs;
    const uint64_t gen = AllocateHandleGen();
    const uint64_t handle =
        (gen << kHandleSlotBits) | static_cast<uint64_t>(handle_slot + 1);
    handles_[handle_slot] = OpenHandle{
        .in_use = true,
        .client_id = client_id,
        .handle = handle,
        .fnode_idx = target_idx,
        .flags = flags,
        .offset = 0,
    };

    __builtin_memset(static_cast<void*>(out_reply), 0, sizeof(*out_reply));
    out_reply->handle = handle;
    out_reply->type = node.type;
    return 0;
  }

  int64_t Close(const uint64_t client_id, const uint64_t handle) {
    OpenHandle* const h = LookupHandle(client_id, handle);
    if (h == nullptr) {
      return kSyscallErrorBadFd;
    }
    CloseHandleSlot(*h);
    return 0;
  }

  // Closes all handles of `client_id`, for example when the client ends.
  void CloseAllForClient(const uint64_t client_id) {
    if (client_id == 0) {
      return;
    }
    for (int i = 0; i < kMaxOpenHandles; ++i) {
      if (handles_[i].in_use && handles_[i].client_id == client_id) {
        CloseHandleSlot(handles_[i]);
      }
    }
  }

  int64_t Read(const uint64_t client_id,  //
               const uint64_t handle,     //
               void* const out_buf,       //
               const int64_t size) {
    if (size < 0 || size > kSyscallFsMaxIoSize ||
        (size > 0 && out_buf == nullptr)) {
      return kSyscallErrorInvalid;
    }
    OpenHandle* const h = LookupHandle(client_id, handle);
    if (h == nullptr) {
      return kSyscallErrorBadFd;
    }
    const Fnode& node = fnodes_[h->fnode_idx];
    if (node.type == FsNodeType::kDirectory) {
      return kSyscallErrorIsDir;
    }
    const uint64_t access = h->flags & kFsOpenAccessMask;
    if (access == kFsOpenWrite) {
      return kSyscallErrorBadFd;
    }
    if (size == 0 || h->offset >= node.size) {
      return 0;
    }

    const int64_t avail = node.size - h->offset;
    const int64_t to_read = (size < avail) ? size : avail;
    uint8_t* const dst = static_cast<uint8_t*>(out_buf);
    int64_t done = 0;
    while (done < to_read) {
      const int64_t pos = h->offset + done;
      const int64_t blk_idx = pos / kBlockSize;
      const int64_t blk_off = pos % kBlockSize;
      const int64_t chunk = (to_read - done < kBlockSize - blk_off)
                                ? (to_read - done)
                                : (kBlockSize - blk_off);
      const uint8_t* const blk = LookupFileBlock(node, blk_idx);
      if (blk != nullptr) {
        __builtin_memcpy(dst + done, blk + blk_off, static_cast<size_t>(chunk));
      } else {
        __builtin_memset(dst + done, 0, static_cast<size_t>(chunk));
      }
      done += chunk;
    }
    h->offset += done;
    return done;
  }

  int64_t Write(const uint64_t client_id,  //
                const uint64_t handle,     //
                const void* const buf,     //
                const int64_t size) {
    if (size < 0 || size > kSyscallFsMaxIoSize ||
        (size > 0 && buf == nullptr)) {
      return kSyscallErrorInvalid;
    }
    OpenHandle* const h = LookupHandle(client_id, handle);
    if (h == nullptr) {
      return kSyscallErrorBadFd;
    }
    Fnode& node = fnodes_[h->fnode_idx];
    if (node.type == FsNodeType::kDirectory) {
      return kSyscallErrorIsDir;
    }
    const uint64_t access = h->flags & kFsOpenAccessMask;
    if (access == kFsOpenRead) {
      return kSyscallErrorBadFd;
    }
    if (size == 0) {
      // Like on Linux, without side effects, also with `kFsOpenAppend`.
      return 0;
    }

    const int64_t write_pos =
        ((h->flags & kFsOpenAppend) != 0) ? node.size : h->offset;
    if (write_pos >= kFsMaxFileSize) {
      return kSyscallErrorFileTooBig;
    }
    const int64_t max_writable = kFsMaxFileSize - write_pos;
    const int64_t limit = (size < max_writable) ? size : max_writable;

    const uint8_t* const src = static_cast<const uint8_t*>(buf);
    int64_t done = 0;
    while (done < limit) {
      const int64_t pos = write_pos + done;
      const int64_t blk_idx = pos / kBlockSize;
      const int64_t blk_off = pos % kBlockSize;
      const int64_t chunk = (limit - done < kBlockSize - blk_off)
                                ? (limit - done)
                                : (kBlockSize - blk_off);
      uint8_t* const blk = EnsureFileBlock(node, blk_idx);
      if (blk == nullptr) {
        if (done == 0) {
          return kSyscallErrorNoSpace;
        }
        break;
      }
      __builtin_memcpy(blk + blk_off, src + done, static_cast<size_t>(chunk));
      done += chunk;
    }
    h->offset = write_pos + done;
    if (h->offset > node.size) {
      node.size = h->offset;
    }
    return done;
  }

  int64_t Seek(const uint64_t client_id,  //
               const uint64_t handle,     //
               const int64_t offset,      //
               const int64_t whence) {
    OpenHandle* const h = LookupHandle(client_id, handle);
    if (h == nullptr) {
      return kSyscallErrorBadFd;
    }
    const Fnode& node = fnodes_[h->fnode_idx];
    if (node.type == FsNodeType::kDirectory) {
      if (whence == kFsSeekSet && offset == 0) {
        h->offset = 0;
        return 0;
      }
      if (whence == kFsSeekCur && offset == 0) {
        return h->offset;
      }
      return kSyscallErrorInvalid;
    }

    int64_t base = 0;
    if (whence == kFsSeekSet) {
      base = 0;
    } else if (whence == kFsSeekCur) {
      base = h->offset;
    } else if (whence == kFsSeekEnd) {
      base = node.size;
    } else {
      return kSyscallErrorInvalid;
    }

    int64_t target = 0;
    if (__builtin_add_overflow(base, offset, &target) || target < 0 ||
        target > kFsMaxFileSize) {
      return kSyscallErrorInvalid;
    }
    h->offset = target;
    return target;
  }

  int64_t Stat(const char* const path,  //
               const int64_t path_len,  //
               FsStat* const out_stat) const {
    if (out_stat == nullptr) {
      return kSyscallErrorInvalid;
    }
    int idx = -1;
    const int64_t rc = ResolvePath(path, path_len, &idx);
    if (rc < 0) {
      return rc;
    }
    const Fnode& node = fnodes_[idx];
    __builtin_memset(static_cast<void*>(out_stat), 0, sizeof(*out_stat));
    out_stat->type = node.type;
    out_stat->fnode_id = node.fnode_id;
    out_stat->size = (node.type == FsNodeType::kDirectory)
                         ? static_cast<int64_t>(node.entry_count) *
                               static_cast<int64_t>(sizeof(FsDirEntry))
                         : node.size;
    out_stat->nlink = node.nlink;
    return 0;
  }

  int64_t ReadDir(const uint64_t client_id,       //
                  const uint64_t handle,          //
                  FsDirEntry* const out_entries,  //
                  const int64_t max_entries) {
    if (max_entries < 0 || max_entries > kSyscallFsMaxDirEntries ||
        (max_entries > 0 && out_entries == nullptr)) {
      return kSyscallErrorInvalid;
    }
    OpenHandle* const h = LookupHandle(client_id, handle);
    if (h == nullptr) {
      return kSyscallErrorBadFd;
    }
    const Fnode& dir = fnodes_[h->fnode_idx];
    if (dir.type != FsNodeType::kDirectory) {
      return kSyscallErrorNotDir;
    }
    if (max_entries == 0 || dir.nlink == 0) {
      return 0;
    }

    uint64_t cursor = static_cast<uint64_t>(h->offset);
    int64_t count = 0;
    while (count < max_entries) {
      const DirBlock::Slot* best = nullptr;
      for (const DirBlock* blk = dir.dir_head; blk != nullptr;
           blk = blk->next) {
        for (int i = 0; i < DirBlock::kSlotsPerBlock; ++i) {
          const DirBlock::Slot& s = blk->slots[i];
          if (s.seq > cursor && (best == nullptr || s.seq < best->seq)) {
            best = &s;
          }
        }
      }
      if (best == nullptr) {
        break;
      }
      const Fnode& child = fnodes_[best->fnode_idx];
      FsDirEntry entry;
      __builtin_memset(static_cast<void*>(&entry), 0, sizeof(entry));
      entry.fnode_id = child.fnode_id;
      entry.type = child.type;
      entry.name_len = best->name_len;
      __builtin_memcpy(entry.name,  //
                       best->name,  //
                       static_cast<size_t>(best->name_len));
      out_entries[count++] = entry;
      cursor = best->seq;
    }
    h->offset = static_cast<int64_t>(cursor);
    return count;
  }

  int64_t MkDir(const char* const path, const int64_t path_len) {
    ParsedParent parsed{};
    const int64_t parse_rc = ResolveParent(path, path_len, &parsed);
    if (parse_rc < 0) {
      return parse_rc;
    }
    if (parsed.is_self_or_dotdot) {
      return kSyscallErrorExists;
    }
    const int existing_idx = LookupInDir(parsed.parent_idx,  //
                                         parsed.name,        //
                                         parsed.name_len,    //
                                         nullptr,            //
                                         nullptr);
    if (existing_idx >= 0) {
      return kSyscallErrorExists;
    }

    const int new_idx = AllocFnode(FsNodeType::kDirectory, parsed.parent_idx);
    if (new_idx < 0) {
      return kSyscallErrorNoSpace;
    }
    const int64_t link_rc =
        InsertInDir(parsed.parent_idx, parsed.name, parsed.name_len, new_idx);
    if (link_rc < 0) {
      fnodes_[new_idx] = Fnode{};
      return link_rc;
    }
    fnodes_[new_idx].nlink = 2;
    ++fnodes_[parsed.parent_idx].nlink;
    return 0;
  }

  int64_t Unlink(const char* const path, const int64_t path_len) {
    ParsedParent parsed{};
    const int64_t parse_rc = ResolveParent(path, path_len, &parsed);
    if (parse_rc < 0) {
      return parse_rc;
    }
    if (parsed.is_self_or_dotdot) {
      return parsed.is_root_slash_only ? kSyscallErrorBusy
                                       : kSyscallErrorInvalid;
    }

    DirBlock* blk = nullptr;
    int slot_in_blk = -1;
    const int target_idx = LookupInDir(parsed.parent_idx,  //
                                       parsed.name,        //
                                       parsed.name_len,    //
                                       &blk,               //
                                       &slot_in_blk);
    if (target_idx < 0) {
      return kSyscallErrorNoEntry;
    }

    Fnode& target = fnodes_[target_idx];
    if (target.type == FsNodeType::kDirectory) {
      if (target.entry_count > 0) {
        return kSyscallErrorNotEmpty;
      }
      RemoveDirSlot(parsed.parent_idx, blk, slot_in_blk);
      --fnodes_[parsed.parent_idx].nlink;
      target.nlink = 0;
      MaybeReclaimFnode(target_idx);
      return 0;
    }

    if (parsed.trailing_slash) {
      return kSyscallErrorNotDir;
    }
    RemoveDirSlot(parsed.parent_idx, blk, slot_in_blk);
    --target.nlink;
    MaybeReclaimFnode(target_idx);
    return 0;
  }

  int64_t Rename(const char* const old_path,  //
                 const int64_t old_len,       //
                 const char* const new_path,  //
                 const int64_t new_len) {
    ParsedParent old_p{};
    const int64_t old_rc = ResolveParent(old_path, old_len, &old_p);
    if (old_rc < 0) {
      return old_rc;
    }
    if (old_p.is_self_or_dotdot) {
      return old_p.is_root_slash_only ? kSyscallErrorBusy
                                      : kSyscallErrorInvalid;
    }

    ParsedParent new_p{};
    const int64_t new_rc = ResolveParent(new_path, new_len, &new_p);
    if (new_rc < 0) {
      return new_rc;
    }
    if (new_p.is_self_or_dotdot) {
      return new_p.is_root_slash_only ? kSyscallErrorBusy
                                      : kSyscallErrorInvalid;
    }

    DirBlock* old_blk = nullptr;
    int old_slot = -1;
    const int src_idx = LookupInDir(old_p.parent_idx,  //
                                    old_p.name,        //
                                    old_p.name_len,    //
                                    &old_blk,          //
                                    &old_slot);
    if (src_idx < 0) {
      return kSyscallErrorNoEntry;
    }
    Fnode& src = fnodes_[src_idx];
    if (src.type != FsNodeType::kDirectory &&
        (old_p.trailing_slash || new_p.trailing_slash)) {
      return kSyscallErrorNotDir;
    }

    if (src.type == FsNodeType::kDirectory &&
        IsAncestorOrSelf(src_idx, new_p.parent_idx)) {
      return kSyscallErrorInvalid;
    }

    DirBlock* new_blk = nullptr;
    int new_slot = -1;
    const int dst_idx = LookupInDir(new_p.parent_idx,  //
                                    new_p.name,        //
                                    new_p.name_len,    //
                                    &new_blk,          //
                                    &new_slot);
    if (dst_idx == src_idx) {
      return 0;
    }

    if (dst_idx >= 0) {
      Fnode& dst = fnodes_[dst_idx];
      if (src.type == FsNodeType::kDirectory &&
          dst.type != FsNodeType::kDirectory) {
        return kSyscallErrorNotDir;
      }
      if (src.type != FsNodeType::kDirectory &&
          dst.type == FsNodeType::kDirectory) {
        return kSyscallErrorIsDir;
      }
      if (dst.type == FsNodeType::kDirectory && dst.entry_count > 0) {
        return kSyscallErrorNotEmpty;
      }

      // Point the destination's slot at the source in place: this allocates
      // nothing, so it can't fail halfway. The new cookie makes it a new entry
      // for ReadDir().
      new_blk->slots[new_slot].fnode_idx = src_idx;
      new_blk->slots[new_slot].seq = fnodes_[new_p.parent_idx].next_dir_seq++;
      RemoveDirSlot(old_p.parent_idx, old_blk, old_slot);
      if (src.type == FsNodeType::kDirectory) {
        --fnodes_[old_p.parent_idx].nlink;
        src.parent_idx = new_p.parent_idx;
        dst.nlink = 0;
      } else {
        --dst.nlink;
      }
      MaybeReclaimFnode(dst_idx);
      return 0;
    }

    // Destination does not exist yet: insert in `new_p.parent_idx` first so an
    // out-of-space failure leaves `old_path` untouched. InsertInDir() only
    // fills a free slot or appends a block, so `old_blk` and `old_slot` still
    // name the source entry afterwards (also within one directory).
    const int64_t ins_rc =
        InsertInDir(new_p.parent_idx, new_p.name, new_p.name_len, src_idx);
    if (ins_rc < 0) {
      return ins_rc;
    }
    RemoveDirSlot(old_p.parent_idx, old_blk, old_slot);
    if (src.type == FsNodeType::kDirectory) {
      --fnodes_[old_p.parent_idx].nlink;
      ++fnodes_[new_p.parent_idx].nlink;
      src.parent_idx = new_p.parent_idx;
    }
    return 0;
  }

  // The number of open handles and of files and directories (for tests).
  int ActiveHandleCount() const {
    int count = 0;
    for (int i = 0; i < kMaxOpenHandles; ++i) {
      if (handles_[i].in_use) {
        ++count;
      }
    }
    return count;
  }

  int ActiveFnodeCount() const {
    int count = 0;
    for (int i = 0; i < kMaxFnodes; ++i) {
      if (fnodes_[i].in_use) {
        ++count;
      }
    }
    return count;
  }

 private:
  static constexpr int kRootFnodeIdx = 0;
  // A handle's low `kHandleSlotBits` bits hold its slot plus 1, and the bits
  // above them its generation.
  static constexpr int kHandleSlotBits = 16;
  static_assert(kMaxOpenHandles < (1 << kHandleSlotBits));

  // The fnode (syscall_abi.h) of a file or directory, in slot `fnode_idx` of
  // `fnodes_`, which directory entries point to. Unlike the slot, `fnode_id` is
  // never reused.
  struct Fnode {
    bool in_use = false;
    FsNodeType type = FsNodeType::kNone;
    uint64_t fnode_id = 0;
    int64_t size = 0;
    int64_t nlink = 0;
    int open_refs = 0;
    // The directory that `..` names. Kept up to date only for directories.
    int parent_idx = 0;

    // Regular-file data blocks.
    uint8_t* direct[kDirectBlocks] = {};
    uint8_t* single_indirect = nullptr;
    uint8_t* double_indirect = nullptr;

    // Directory entries.
    DirBlock* dir_head = nullptr;
    int entry_count = 0;
    // The cookie of the next entry, from 1 on (`DirBlock::Slot::seq`).
    uint64_t next_dir_seq = 0;
  };

  struct OpenHandle {
    bool in_use = false;
    uint64_t client_id = 0;
    uint64_t handle = 0;
    int fnode_idx = 0;
    uint64_t flags = 0;
    int64_t offset = 0;
  };

  struct ParsedParent {
    int parent_idx = kRootFnodeIdx;
    int resolved_dir_idx = kRootFnodeIdx;
    bool is_self_or_dotdot = false;
    bool is_root_slash_only = false;
    bool trailing_slash = false;
    int32_t name_len = 0;
    char name[kFsMaxNameLength + 1] = {};
  };

  static bool NameEquals(const char* const a,  //
                         const int32_t a_len,  //
                         const char* const b,  //
                         const int32_t b_len) {
    if (a_len != b_len) {
      return false;
    }
    for (int32_t i = 0; i < a_len; ++i) {
      if (a[i] != b[i]) {
        return false;
      }
    }
    return true;
  }

  static bool IsDot(const char* const s, const int32_t len) {
    return len == 1 && s[0] == '.';
  }

  static bool IsDotDot(const char* const s, const int32_t len) {
    return len == 2 && s[0] == '.' && s[1] == '.';
  }

  uint64_t AllocateHandleGen() {
    uint64_t gen = next_handle_gen_++;
    if (gen == 0) {
      gen = next_handle_gen_++;
    }
    return gen;
  }

  int FindFreeHandleSlot() const {
    for (int i = 0; i < kMaxOpenHandles; ++i) {
      if (!handles_[i].in_use) {
        return i;
      }
    }
    return -1;
  }

  OpenHandle* LookupHandle(const uint64_t client_id, const uint64_t handle) {
    if (client_id == 0) {
      return nullptr;
    }
    const int slot =
        static_cast<int>(handle & ((uint64_t{1} << kHandleSlotBits) - 1)) - 1;
    if (slot < 0 || slot >= kMaxOpenHandles) {
      return nullptr;
    }
    OpenHandle& h = handles_[slot];
    if (!h.in_use || h.handle != handle || h.client_id != client_id) {
      return nullptr;
    }
    return &h;
  }

  void CloseHandleSlot(OpenHandle& h) {
    if (!h.in_use) {
      return;
    }
    const int idx = h.fnode_idx;
    h = OpenHandle{};
    if (idx >= 0 && idx < kMaxFnodes && fnodes_[idx].in_use) {
      --fnodes_[idx].open_refs;
      MaybeReclaimFnode(idx);
    }
  }

  int AllocFnode(const FsNodeType type, const int parent_idx) {
    for (int i = 1; i < kMaxFnodes; ++i) {
      if (!fnodes_[i].in_use) {
        fnodes_[i] = Fnode{};
        fnodes_[i].in_use = true;
        fnodes_[i].type = type;
        fnodes_[i].fnode_id = next_fnode_id_++;
        fnodes_[i].parent_idx = parent_idx;
        // Cookies start at 1: `seq == 0` marks a free slot.
        fnodes_[i].next_dir_seq = 1;
        return i;
      }
    }
    return -1;
  }

  void MaybeReclaimFnode(const int idx) {
    if (idx <= kRootFnodeIdx || idx >= kMaxFnodes) {
      return;
    }
    Fnode& node = fnodes_[idx];
    if (!node.in_use || node.nlink > 0 || node.open_refs > 0) {
      return;
    }
    if (node.type == FsNodeType::kRegular) {
      FreeAllFileBlocks(node);
    } else if (node.type == FsNodeType::kDirectory) {
      FreeAllDirBlocks(node);
    }
    node = Fnode{};
  }

  const uint8_t* LookupFileBlock(const Fnode& node,
                                 const int64_t blk_idx) const {
    if (blk_idx < 0) {
      return nullptr;
    }
    if (blk_idx < kDirectBlocks) {
      return node.direct[blk_idx];
    }
    const int64_t single_rel = blk_idx - kDirectBlocks;
    if (single_rel < kPtrsPerIndirectBlock) {
      if (node.single_indirect == nullptr) {
        return nullptr;
      }
      auto* const ptrs =
          reinterpret_cast<uint8_t* const*>(node.single_indirect);
      return ptrs[single_rel];
    }
    const int64_t double_rel = single_rel - kPtrsPerIndirectBlock;
    const int64_t l1 = double_rel / kPtrsPerIndirectBlock;
    const int64_t l2 = double_rel % kPtrsPerIndirectBlock;
    if (l1 >= kPtrsPerIndirectBlock || node.double_indirect == nullptr) {
      return nullptr;
    }
    auto* const l1_ptrs =
        reinterpret_cast<uint8_t* const*>(node.double_indirect);
    if (l1_ptrs[l1] == nullptr) {
      return nullptr;
    }
    auto* const l2_ptrs = reinterpret_cast<uint8_t* const*>(l1_ptrs[l1]);
    return l2_ptrs[l2];
  }

  uint8_t* EnsureFileBlock(Fnode& node, const int64_t blk_idx) {
    if (blk_idx < 0 || alloc_ == nullptr) {
      return nullptr;
    }
    if (blk_idx < kDirectBlocks) {
      if (node.direct[blk_idx] == nullptr) {
        node.direct[blk_idx] = alloc_->AllocBlock();
      }
      return node.direct[blk_idx];
    }
    const int64_t single_rel = blk_idx - kDirectBlocks;
    if (single_rel < kPtrsPerIndirectBlock) {
      bool created_single = false;
      if (node.single_indirect == nullptr) {
        node.single_indirect = alloc_->AllocBlock();
        if (node.single_indirect == nullptr) {
          return nullptr;
        }
        created_single = true;
      }
      auto* const ptrs = reinterpret_cast<uint8_t**>(node.single_indirect);
      if (ptrs[single_rel] == nullptr) {
        ptrs[single_rel] = alloc_->AllocBlock();
        if (ptrs[single_rel] == nullptr) {
          if (created_single) {
            alloc_->FreeBlock(node.single_indirect);
            node.single_indirect = nullptr;
          }
          return nullptr;
        }
      }
      return ptrs[single_rel];
    }
    const int64_t double_rel = single_rel - kPtrsPerIndirectBlock;
    const int64_t l1 = double_rel / kPtrsPerIndirectBlock;
    const int64_t l2 = double_rel % kPtrsPerIndirectBlock;
    if (l1 >= kPtrsPerIndirectBlock) {
      return nullptr;
    }
    bool created_l1 = false;
    if (node.double_indirect == nullptr) {
      node.double_indirect = alloc_->AllocBlock();
      if (node.double_indirect == nullptr) {
        return nullptr;
      }
      created_l1 = true;
    }
    auto* const l1_ptrs = reinterpret_cast<uint8_t**>(node.double_indirect);
    bool created_l2 = false;
    if (l1_ptrs[l1] == nullptr) {
      l1_ptrs[l1] = alloc_->AllocBlock();
      if (l1_ptrs[l1] == nullptr) {
        if (created_l1) {
          alloc_->FreeBlock(node.double_indirect);
          node.double_indirect = nullptr;
        }
        return nullptr;
      }
      created_l2 = true;
    }
    auto* const l2_ptrs = reinterpret_cast<uint8_t**>(l1_ptrs[l1]);
    if (l2_ptrs[l2] == nullptr) {
      l2_ptrs[l2] = alloc_->AllocBlock();
      if (l2_ptrs[l2] == nullptr) {
        if (created_l2) {
          alloc_->FreeBlock(l1_ptrs[l1]);
          l1_ptrs[l1] = nullptr;
        }
        if (created_l1) {
          alloc_->FreeBlock(node.double_indirect);
          node.double_indirect = nullptr;
        }
        return nullptr;
      }
    }
    return l2_ptrs[l2];
  }

  void FreeAllFileBlocks(Fnode& node) {
    if (alloc_ == nullptr) {
      return;
    }
    for (int i = 0; i < kDirectBlocks; ++i) {
      if (node.direct[i] != nullptr) {
        alloc_->FreeBlock(node.direct[i]);
        node.direct[i] = nullptr;
      }
    }
    if (node.single_indirect != nullptr) {
      auto* const ptrs = reinterpret_cast<uint8_t**>(node.single_indirect);
      for (int i = 0; i < kPtrsPerIndirectBlock; ++i) {
        if (ptrs[i] != nullptr) {
          alloc_->FreeBlock(ptrs[i]);
        }
      }
      alloc_->FreeBlock(node.single_indirect);
      node.single_indirect = nullptr;
    }
    if (node.double_indirect != nullptr) {
      auto* const l1_ptrs = reinterpret_cast<uint8_t**>(node.double_indirect);
      for (int i = 0; i < kPtrsPerIndirectBlock; ++i) {
        if (l1_ptrs[i] != nullptr) {
          auto* const l2_ptrs = reinterpret_cast<uint8_t**>(l1_ptrs[i]);
          for (int j = 0; j < kPtrsPerIndirectBlock; ++j) {
            if (l2_ptrs[j] != nullptr) {
              alloc_->FreeBlock(l2_ptrs[j]);
            }
          }
          alloc_->FreeBlock(l1_ptrs[i]);
        }
      }
      alloc_->FreeBlock(node.double_indirect);
      node.double_indirect = nullptr;
    }
  }

  void FreeAllDirBlocks(Fnode& dir) {
    if (alloc_ == nullptr) {
      return;
    }
    DirBlock* cur = dir.dir_head;
    dir.dir_head = nullptr;
    dir.entry_count = 0;
    while (cur != nullptr) {
      DirBlock* const next = cur->next;
      alloc_->FreeBlock(reinterpret_cast<uint8_t*>(cur));
      cur = next;
    }
  }

  int LookupInDir(const int dir_idx,         //
                  const char* const name,    //
                  const int32_t name_len,    //
                  DirBlock** const out_blk,  //
                  int* const out_slot) const {
    const Fnode& dir = fnodes_[dir_idx];
    for (DirBlock* blk = dir.dir_head; blk != nullptr; blk = blk->next) {
      for (int i = 0; i < DirBlock::kSlotsPerBlock; ++i) {
        const DirBlock::Slot& s = blk->slots[i];
        if (s.seq != 0 && NameEquals(s.name, s.name_len, name, name_len)) {
          if (out_blk != nullptr) {
            *out_blk = blk;
          }
          if (out_slot != nullptr) {
            *out_slot = i;
          }
          return s.fnode_idx;
        }
      }
    }
    return -1;
  }

  int64_t InsertInDir(const int dir_idx,       //
                      const char* const name,  //
                      const int32_t name_len,  //
                      const int child_idx) {
    if (alloc_ == nullptr) {
      return kSyscallErrorNoSpace;
    }
    Fnode& dir = fnodes_[dir_idx];
    DirBlock* target_blk = nullptr;
    int target_slot = -1;
    DirBlock* tail = nullptr;
    for (DirBlock* blk = dir.dir_head; blk != nullptr; blk = blk->next) {
      tail = blk;
      if (target_blk == nullptr && blk->used_count < DirBlock::kSlotsPerBlock) {
        for (int i = 0; i < DirBlock::kSlotsPerBlock; ++i) {
          if (blk->slots[i].seq == 0) {
            target_blk = blk;
            target_slot = i;
            break;
          }
        }
      }
    }
    if (target_blk == nullptr) {
      uint8_t* const raw = alloc_->AllocBlock();
      if (raw == nullptr) {
        return kSyscallErrorNoSpace;
      }
      target_blk = new (raw) DirBlock;
      target_slot = 0;
      if (tail != nullptr) {
        tail->next = target_blk;
      } else {
        dir.dir_head = target_blk;
      }
    }

    DirBlock::Slot& s = target_blk->slots[target_slot];
    __builtin_memset(static_cast<void*>(&s), 0, sizeof(s));
    s.seq = dir.next_dir_seq++;
    s.fnode_idx = child_idx;
    s.name_len = name_len;
    __builtin_memcpy(s.name, name, static_cast<size_t>(name_len));
    ++target_blk->used_count;
    ++dir.entry_count;
    return 0;
  }

  void RemoveDirSlot(const int dir_idx, DirBlock* const blk, const int slot) {
    Fnode& dir = fnodes_[dir_idx];
    blk->slots[slot] = DirBlock::Slot{};
    --blk->used_count;
    --dir.entry_count;
    if (blk->used_count == 0 && alloc_ != nullptr) {
      DirBlock** link = &dir.dir_head;
      while (*link != nullptr) {
        if (*link == blk) {
          *link = blk->next;
          alloc_->FreeBlock(reinterpret_cast<uint8_t*>(blk));
          break;
        }
        link = &(*link)->next;
      }
    }
  }

  bool IsAncestorOrSelf(const int ancestor_idx, int cur_idx) const {
    for (int depth = 0; depth <= kMaxFnodes; ++depth) {
      if (cur_idx == ancestor_idx) {
        return true;
      }
      if (cur_idx == kRootFnodeIdx) {
        return false;
      }
      cur_idx = fnodes_[cur_idx].parent_idx;
    }
    return true;
  }

  int64_t ResolveParent(const char* const path,  //
                        const int64_t path_len,  //
                        ParsedParent* const out) const {
    if (path == nullptr || path_len < 0) {
      return kSyscallErrorInvalid;
    }
    if (path_len == 0) {
      return kSyscallErrorNoEntry;
    }
    if (path_len > kFsMaxPathLength) {
      return kSyscallErrorNameTooLong;
    }
    for (int64_t i = 0; i < path_len; ++i) {
      if (path[i] == '\0') {
        return kSyscallErrorInvalid;
      }
    }

    *out = ParsedParent{};
    out->trailing_slash = (path[path_len - 1] == '/');

    int64_t end = path_len;
    while (end > 0 && path[end - 1] == '/') {
      --end;
    }
    if (end == 0) {
      // Path consisted solely of one or more `/` characters.
      out->is_self_or_dotdot = true;
      out->is_root_slash_only = true;
      out->resolved_dir_idx = kRootFnodeIdx;
      return 0;
    }

    int cur_dir = kRootFnodeIdx;
    int64_t pos = 0;
    while (pos < end) {
      while (pos < end && path[pos] == '/') {
        ++pos;
      }
      int64_t comp_end = pos;
      while (comp_end < end && path[comp_end] != '/') {
        ++comp_end;
      }
      const int64_t comp_len = comp_end - pos;
      if (comp_len > kFsMaxNameLength) {
        return kSyscallErrorNameTooLong;
      }
      const char* const comp = path + pos;
      const int32_t clen = static_cast<int32_t>(comp_len);

      int64_t next_pos = comp_end;
      while (next_pos < end && path[next_pos] == '/') {
        ++next_pos;
      }
      const bool is_last = (next_pos == end);

      if (fnodes_[cur_dir].type != FsNodeType::kDirectory) {
        return kSyscallErrorNotDir;
      }

      if (is_last) {
        if (IsDot(comp, clen)) {
          out->is_self_or_dotdot = true;
          out->resolved_dir_idx = cur_dir;
          return 0;
        }
        if (IsDotDot(comp, clen)) {
          out->is_self_or_dotdot = true;
          out->resolved_dir_idx = fnodes_[cur_dir].parent_idx;
          return 0;
        }
        out->parent_idx = cur_dir;
        out->name_len = clen;
        __builtin_memcpy(out->name, comp, static_cast<size_t>(clen));
        out->name[clen] = '\0';
        return 0;
      }

      // Intermediate component: must resolve to an existing directory.
      if (IsDot(comp, clen)) {
        // Stay in `cur_dir`.
      } else if (IsDotDot(comp, clen)) {
        cur_dir = fnodes_[cur_dir].parent_idx;
      } else {
        const int next_idx = LookupInDir(cur_dir, comp, clen, nullptr, nullptr);
        if (next_idx < 0) {
          return kSyscallErrorNoEntry;
        }
        if (fnodes_[next_idx].type != FsNodeType::kDirectory) {
          return kSyscallErrorNotDir;
        }
        cur_dir = next_idx;
      }
      pos = next_pos;
    }
    return kSyscallErrorInvalid;
  }

  int64_t ResolvePath(const char* const path,  //
                      const int64_t path_len,  //
                      int* const out_idx) const {
    ParsedParent parsed{};
    const int64_t rc = ResolveParent(path, path_len, &parsed);
    if (rc < 0) {
      return rc;
    }
    if (parsed.is_self_or_dotdot) {
      *out_idx = parsed.resolved_dir_idx;
      return 0;
    }
    const int idx = LookupInDir(parsed.parent_idx,  //
                                parsed.name,        //
                                parsed.name_len,    //
                                nullptr,            //
                                nullptr);
    if (idx < 0) {
      return kSyscallErrorNoEntry;
    }
    if (parsed.trailing_slash && fnodes_[idx].type != FsNodeType::kDirectory) {
      return kSyscallErrorNotDir;
    }
    *out_idx = idx;
    return 0;
  }

  BlockAllocator* alloc_ = nullptr;
  Fnode fnodes_[kMaxFnodes] = {};
  OpenHandle handles_[kMaxOpenHandles] = {};
  uint64_t next_fnode_id_ = 0;
  uint64_t next_handle_gen_ = 0;
};

namespace rpc_internal {

// Copies the `Header` at the start of the request `[req_buf, req_buf +
// req_size)` to `*out` and returns the number of bytes after it, or returns -1
// if the request is shorter than a `Header`.
template <typename Header>
inline int64_t ReadHeader(const uint8_t* const req_buf,  //
                          const int64_t req_size,        //
                          Header* const out) {
  const int64_t header_size = static_cast<int64_t>(sizeof(Header));
  if (req_size < header_size) {
    return -1;
  }
  __builtin_memcpy(static_cast<void*>(out), req_buf, sizeof(Header));
  return req_size - header_size;
}

// The path of a request: `len` bytes at `data`, or `len == -1` if the request
// is malformed.
struct RequestPath {
  const char* data = nullptr;
  int64_t len = -1;
};

// Decodes a `kStat`, `kMkDir` or `kUnlink` request: an `FsPathRequest`
// followed by exactly `path_len` path bytes.
inline RequestPath ReadPathRequest(const uint8_t* const req_buf,
                                   const int64_t req_size) {
  FsPathRequest hdr{};
  const int64_t path_len = ReadHeader(req_buf, req_size, &hdr);
  if (path_len < 0 || hdr.path_len != path_len) {
    return RequestPath{};
  }
  return RequestPath{
      .data = reinterpret_cast<const char*>(req_buf + sizeof(hdr)),
      .len = path_len,
  };
}

}  // namespace rpc_internal

// Decodes one RPC message `[req_buf, req_buf + req_size)` from `client_id`,
// executes it against `fs`, writes any reply payload into `reply_buf`, stores
// the reply byte count in `*out_reply_size`, and returns the status to pass to
// `IpcReply()`. `reply_cap` is the longest reply that both `reply_buf` and the
// client take: a request whose reply may be longer (for `kRead`, one for more
// than `reply_cap` bytes) fails with `kSyscallErrorInvalid` before it changes
// anything, like a malformed request (fs_protocol.h). A failed request has no
// reply bytes.
inline int64_t DispatchRpc(FileSystem& fs,                //
                           const uint64_t client_id,      //
                           const uint8_t* const req_buf,  //
                           const int64_t req_size,        //
                           uint8_t* const reply_buf,      //
                           const int64_t reply_cap,       //
                           int64_t* const out_reply_size) {
  *out_reply_size = 0;
  FsRpcOp op = FsRpcOp::kNone;
  if (req_buf == nullptr || reply_cap < 0 ||
      (reply_cap > 0 && reply_buf == nullptr) ||
      rpc_internal::ReadHeader(req_buf, req_size, &op) < 0) {
    return kSyscallErrorInvalid;
  }

  // Request lengths are untrusted: they are compared with the byte counts that
  // `ReadHeader()` returns, without arithmetic that could overflow.
  switch (op) {
    case FsRpcOp::kOpen: {
      FsOpenRequest hdr{};
      const int64_t path_len =
          rpc_internal::ReadHeader(req_buf, req_size, &hdr);
      if (path_len < 0 || hdr.path_len != path_len ||
          reply_cap < static_cast<int64_t>(sizeof(FsOpenReply))) {
        return kSyscallErrorInvalid;
      }
      const char* const path =
          reinterpret_cast<const char*>(req_buf + sizeof(hdr));
      FsOpenReply reply{};
      const int64_t rc = fs.Open(client_id, path, path_len, hdr.flags, &reply);
      if (rc < 0) {
        return rc;
      }
      __builtin_memcpy(reply_buf, &reply, sizeof(reply));
      *out_reply_size = static_cast<int64_t>(sizeof(reply));
      return 0;
    }

    case FsRpcOp::kClose: {
      FsCloseRequest hdr{};
      if (rpc_internal::ReadHeader(req_buf, req_size, &hdr) != 0 ||
          hdr.reserved != 0) {
        return kSyscallErrorInvalid;
      }
      return fs.Close(client_id, hdr.handle);
    }

    case FsRpcOp::kRead: {
      FsReadRequest hdr{};
      if (rpc_internal::ReadHeader(req_buf, req_size, &hdr) != 0 ||
          hdr.reserved != 0 || hdr.size < 0 || hdr.size > kSyscallFsMaxIoSize ||
          hdr.size > reply_cap) {
        return kSyscallErrorInvalid;
      }
      const int64_t rc = fs.Read(client_id, hdr.handle, reply_buf, hdr.size);
      if (rc < 0) {
        return rc;
      }
      *out_reply_size = rc;
      return rc;
    }

    case FsRpcOp::kWrite: {
      FsWriteRequest hdr{};
      const int64_t data_size =
          rpc_internal::ReadHeader(req_buf, req_size, &hdr);
      if (data_size < 0 || hdr.reserved != 0 || hdr.size < 0 ||
          hdr.size > kSyscallFsMaxIoSize || hdr.size != data_size) {
        return kSyscallErrorInvalid;
      }
      return fs.Write(client_id, hdr.handle, req_buf + sizeof(hdr), data_size);
    }

    case FsRpcOp::kSeek: {
      FsSeekRequest hdr{};
      if (rpc_internal::ReadHeader(req_buf, req_size, &hdr) != 0) {
        return kSyscallErrorInvalid;
      }
      return fs.Seek(client_id, hdr.handle, hdr.offset, hdr.whence);
    }

    case FsRpcOp::kStat: {
      const rpc_internal::RequestPath path =
          rpc_internal::ReadPathRequest(req_buf, req_size);
      if (path.len < 0 || reply_cap < static_cast<int64_t>(sizeof(FsStat))) {
        return kSyscallErrorInvalid;
      }
      FsStat st{};
      const int64_t rc = fs.Stat(path.data, path.len, &st);
      if (rc < 0) {
        return rc;
      }
      __builtin_memcpy(reply_buf, &st, sizeof(st));
      *out_reply_size = static_cast<int64_t>(sizeof(st));
      return 0;
    }

    case FsRpcOp::kReadDir: {
      FsReadDirRequest hdr{};
      if (rpc_internal::ReadHeader(req_buf, req_size, &hdr) != 0 ||
          hdr.max_entries < 0 || hdr.max_entries > kSyscallFsMaxDirEntries ||
          hdr.max_entries * static_cast<int64_t>(sizeof(FsDirEntry)) >
              reply_cap) {
        return kSyscallErrorInvalid;
      }
      FsDirEntry entries[kSyscallFsMaxDirEntries]{};
      const int64_t rc =
          fs.ReadDir(client_id, hdr.handle, entries, hdr.max_entries);
      if (rc < 0) {
        return rc;
      }
      const int64_t bytes = rc * static_cast<int64_t>(sizeof(FsDirEntry));
      if (bytes > 0) {
        __builtin_memcpy(reply_buf, entries, static_cast<size_t>(bytes));
      }
      *out_reply_size = bytes;
      return rc;
    }

    case FsRpcOp::kMkDir: {
      const rpc_internal::RequestPath path =
          rpc_internal::ReadPathRequest(req_buf, req_size);
      if (path.len < 0) {
        return kSyscallErrorInvalid;
      }
      return fs.MkDir(path.data, path.len);
    }

    case FsRpcOp::kUnlink: {
      const rpc_internal::RequestPath path =
          rpc_internal::ReadPathRequest(req_buf, req_size);
      if (path.len < 0) {
        return kSyscallErrorInvalid;
      }
      return fs.Unlink(path.data, path.len);
    }

    case FsRpcOp::kRename: {
      FsRenameRequest hdr{};
      const int64_t paths_size =
          rpc_internal::ReadHeader(req_buf, req_size, &hdr);
      if (paths_size < 0 || hdr.reserved != 0 || hdr.old_len < 0 ||
          hdr.new_len < 0 ||
          static_cast<int64_t>(hdr.old_len) + hdr.new_len != paths_size) {
        return kSyscallErrorInvalid;
      }
      const char* const old_path =
          reinterpret_cast<const char*>(req_buf + sizeof(hdr));
      const char* const new_path = old_path + hdr.old_len;
      return fs.Rename(old_path, hdr.old_len, new_path, hdr.new_len);
    }

    case FsRpcOp::kNone:
      break;
  }
  // `kNone` and unknown opcodes.
  return kSyscallErrorInvalid;
}

}  // namespace protos::ramfs

#endif  // PROTOS_USER_RAMFS_RAMFS_H_
