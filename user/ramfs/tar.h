#ifndef PROTOS_USER_RAMFS_TAR_H_
#define PROTOS_USER_RAMFS_TAR_H_

#include <cstdint>
#include <type_traits>

#include "syscall_abi.h"

// Freestanding, header-only reader of POSIX ustar archives (`tar::Reader`),
// with which `ramfs` unpacks the root filesystem archive that the boot loader
// loads as a boot module (user/ramfs/rootfs.h). It reads the archive in pieces
// through a `tar::Source`, so the archive needn't be in memory, allocates
// nothing, and keeps one 512-byte header block on the stack.
//
// It reads what `tar --format=ustar` writes, the headers of the old GNU
// format, and PAX extended headers (`'x'`) for `size`, `path` and
// `GNU.sparse.*`. It returns regular files and directories, with their paths
// normalized for `ramfs::FileSystem`, and skips everything else: links,
// devices, FIFOs, members of unknown types, members with unusable paths, old
// GNU sparse (`'S'`) members, and members whose real names or sparse layouts
// are in PAX extended (`path`, `GNU.sparse.*`) or GNU long name headers, which
// it doesn't interpret.

namespace protos::tar {

// The size of a header block. A member's data is padded with zeros to whole
// blocks.
constexpr int64_t kBlockSize = 512;

// The type of an archive member (`Entry::type`).
enum class EntryType : uint8_t {
  // A regular file: `Entry::size` data bytes follow its header.
  kRegular = 1,
  // A directory.
  kDirectory = 2,
  // Anything else, which readers skip: a link, a device, a FIFO, a member of
  // an unknown type, one with an unusable path (see `Entry::path`), an old GNU
  // sparse (`'S'`) member, or one whose real name or sparse layout is in a
  // PAX extended (`path`, `GNU.sparse.*`) or GNU long name header before it.
  kSkipped = 3,
};

// The header of an archive member, as `Reader::Next()` reads it.
struct Entry {
  EntryType type = EntryType::kSkipped;
  // The number of bytes of `path` before its NUL.
  int64_t path_len = 0;
  // The member's normalized absolute path, NUL-terminated and NUL-padded: '/'
  // followed by the components of its name joined with '/', without empty and
  // `.` components ("/" alone for the root). A member whose name has a `..`
  // component or a component longer than `kFsMaxNameLength` bytes, or whose
  // path would be longer than `kFsMaxPathLength` bytes, is skipped, and so is
  // a regular file whose path is "/". Empty for `EntryType::kSkipped`.
  char path[kFsMaxPathLength + 1] = {};
  // The header's size field: the number of data bytes after the header.
  int64_t size = 0;
  // The archive offset of the first data byte: the header's offset plus
  // `kBlockSize` (after any old GNU sparse extension blocks).
  int64_t data_offset = 0;
};

// The archive that a `Reader` reads: a boot module in ring 3, memory in host
// tests. Protected non-virtual destructor avoids requiring `operator delete`
// in freestanding ring-3 binaries.
class Source {
 public:
  // Copies up to `size` (> 0) bytes of the archive, from offset `offset`
  // (>= 0) on, to `buffer`, and returns their number: 1 to `size` (fewer than
  // asked for is fine), 0 at or past the end of the archive, or a negative
  // `kSyscallError*` value.
  virtual int64_t Read(int64_t offset, void* buffer, int64_t size) = 0;

 protected:
  constexpr Source() = default;
  ~Source() = default;
};

// Reads the members of a ustar archive from a `Source`, in archive order.
//
// An archive is a sequence of members, each a 512-byte header block followed
// by its data, and ends with a zero block (writers append two) or where the
// source ends. A header holds the member's name (up to 100 bytes, after a
// prefix of up to 155 bytes and a '/' in POSIX ustar), its type, and octal
// numbers such as its size and the header's checksum. The reader checks the
// checksum, the "ustar" magic and the size of every header. A source that
// returns more bytes than asked for makes it fail with `kSyscallErrorIo`.
class Reader {
 public:
  // `source` must outlive the reader.
  constexpr explicit Reader(Source* const source) : source_(source) {}

  // Reads the header of the next member into `*entry` and returns 1, or
  // returns 0 at the end of the archive, or a negative error:
  // `kSyscallErrorInvalid` for a malformed header (bad checksum, magic or
  // size), malformed PAX extended header data, or an overflowing offset;
  // `kSyscallErrorIo` if the archive ends inside a header, inside PAX extended
  // header data, inside an old GNU sparse extension block chain, or inside the
  // data of the member before; or the error of the source. Interprets PAX
  // extended headers (`'x'`) for `size` (overriding the member header's size),
  // `path` and `GNU.sparse.*` (returning the member as `EntryType::kSkipped`),
  // while ignoring other PAX keywords such as timestamps; skips members
  // preceded by GNU long name/link headers (`'L'`/`'K'`) as
  // `EntryType::kSkipped`; follows old GNU sparse (`'S'`) extension chains;
  // and reads PAX global headers (`'g'`) without returning them. Once it has
  // returned 0 or an error (except `kSyscallErrorInvalid` for a null `entry`),
  // it returns the same again without reading.
  [[nodiscard]] int64_t Next(Entry* const entry) {
    if (finished_) {
      return result_;
    }
    if (entry == nullptr || source_ == nullptr) {
      return kSyscallErrorInvalid;
    }
    // Whether a GNU long name/link header (`'L'`/`'K'`) precedes the member.
    bool has_long_name = false;
    PaxState pax{};
    for (;;) {
      header_offset_ = next_offset_;
      uint8_t block[kBlockSize] = {};
      const int64_t read_rc = ReadBlock(header_offset_, block);
      if (read_rc < 0) {
        return Finish(read_rc);
      }
      if (read_rc == 0) {
        return Finish(CheckLastDataByte());
      }
      if (IsZeroBlock(block)) {
        return Finish(0);
      }
      int64_t size = 0;
      if (!ChecksumMatches(block) || !HasUstarMagic(block) ||
          !ParseSize(block, &size)) {
        return Finish(kSyscallErrorInvalid);
      }
      // `next_offset_` leaves room for a header block, so this can't overflow.
      const int64_t data_offset = header_offset_ + kBlockSize;
      int64_t next_offset = 0;

      const uint8_t typeflag = block[kTypeflagOffset];
      if (typeflag == 'L' || typeflag == 'K' || typeflag == 'g' ||
          typeflag == 'x') {
        if (!FollowingHeaderOffset(data_offset, size, &next_offset)) {
          return Finish(kSyscallErrorInvalid);
        }
        next_offset_ = next_offset;
        last_data_byte_ = (size > 0) ? data_offset + size - 1 : -1;
        if (typeflag == 'x') {
          const int64_t pax_rc = ParsePaxHeader(data_offset, size, block, &pax);
          if (pax_rc < 0) {
            return Finish(pax_rc);
          }
        } else if (typeflag == 'L' || typeflag == 'K') {
          has_long_name = true;
        }
        continue;
      }

      if (pax.has_size) {
        size = pax.size;
      }
      int64_t member_data_offset = data_offset;
      const bool gnu_sparse_ext = typeflag == 'S' &&
                                  block[kMagicOffset + 5] != 0 &&
                                  block[kGnuHeaderIsExtendedOffset] != 0;
      const bool extended = has_long_name || pax.has_path || pax.has_gnu_sparse;
      if (gnu_sparse_ext) {
        for (;;) {
          const int64_t ext_rc = ReadBlock(member_data_offset, block);
          if (ext_rc < 0) {
            return Finish(ext_rc);
          }
          if (ext_rc == 0) {
            return Finish(kSyscallErrorIo);
          }
          if (__builtin_add_overflow(member_data_offset, kBlockSize,
                                     &member_data_offset)) {
            return Finish(kSyscallErrorInvalid);
          }
          if (block[kGnuExtIsExtendedOffset] == 0) {
            break;
          }
        }
      }
      if (!FollowingHeaderOffset(member_data_offset, size, &next_offset)) {
        return Finish(kSyscallErrorInvalid);
      }
      next_offset_ = next_offset;
      last_data_byte_ = (size > 0) ? member_data_offset + size - 1 : -1;
      if (gnu_sparse_ext) {
        // `block` was reused for the extension blocks; `'S'` is always skipped.
        entry->type = EntryType::kSkipped;
        entry->path_len = 0;
        __builtin_memset(entry->path, 0, sizeof(entry->path));
        entry->size = size;
        entry->data_offset = member_data_offset;
      } else {
        FillEntry(block, extended, size, member_data_offset, entry);
      }
      return 1;
    }
  }

  // Copies up to `size` (>= 0) bytes of the data of `entry`, which Next()
  // returned, from data byte `pos` (`0 <= pos <= entry.size`) on, to `buffer`,
  // and returns their number: `size` or the `entry.size - pos` bytes left,
  // whichever is less. Fails with `kSyscallErrorInvalid` for invalid
  // arguments, with `kSyscallErrorIo` if the archive ends before the bytes,
  // and with the error of the source.
  [[nodiscard]] int64_t ReadData(const Entry& entry,  //
                                 const int64_t pos,   //
                                 void* const buffer,  //
                                 const int64_t size) {
    int64_t data_end = 0;
    if (source_ == nullptr || entry.size < 0 || entry.data_offset < 0 ||
        __builtin_add_overflow(entry.data_offset, entry.size, &data_end) ||
        pos < 0 || pos > entry.size || size < 0 ||
        (size > 0 && buffer == nullptr)) {
      return kSyscallErrorInvalid;
    }
    const int64_t left = entry.size - pos;
    const int64_t count = (size < left) ? size : left;
    uint8_t* const out = static_cast<uint8_t*>(buffer);
    int64_t done = 0;
    while (done < count) {
      const int64_t rc = source_->Read(entry.data_offset + pos + done,  //
                                       out + done,                      //
                                       count - done);
      if (rc < 0) {
        return rc;
      }
      if (rc == 0 || rc > count - done) {
        return kSyscallErrorIo;
      }
      done += rc;
    }
    return done;
  }

  // The archive offset of the header that the last Next() read (0 before the
  // first call), for messages.
  int64_t header_offset() const { return header_offset_; }

 private:
  // The offsets and lengths of the header fields that the reader uses.
  static constexpr int kNameOffset = 0;
  static constexpr int kNameLength = 100;
  static constexpr int kSizeOffset = 124;
  static constexpr int kSizeLength = 12;
  static constexpr int kChecksumOffset = 148;
  static constexpr int kChecksumLength = 8;
  static constexpr int kTypeflagOffset = 156;
  static constexpr int kMagicOffset = 257;
  static constexpr int kPrefixOffset = 345;
  static constexpr int kPrefixLength = 155;
  static constexpr int kGnuHeaderIsExtendedOffset = 482;
  static constexpr int kGnuExtIsExtendedOffset = 504;

  // The per-file overrides accumulated from PAX extended headers (`'x'`)
  // before a member.
  struct PaxState {
    bool has_path = false;
    bool has_size = false;
    bool has_gnu_sparse = false;
    int64_t size = 0;
  };

  // Makes every later Next() return `result`, and returns it.
  int64_t Finish(const int64_t result) {
    finished_ = true;
    result_ = result;
    return result;
  }

  // Reads the `kBlockSize` bytes at `offset` into `block` and returns 1, or
  // returns 0 if the source ends at `offset`, or a negative error
  // (`kSyscallErrorIo` if the source ends inside the block).
  int64_t ReadBlock(const int64_t offset, uint8_t* const block) {
    int64_t done = 0;
    while (done < kBlockSize) {
      const int64_t rc =
          source_->Read(offset + done, block + done, kBlockSize - done);
      if (rc < 0) {
        return rc;
      }
      if (rc == 0) {
        return (done == 0) ? 0 : kSyscallErrorIo;
      }
      if (rc > kBlockSize - done) {
        return kSyscallErrorIo;
      }
      done += rc;
    }
    return 1;
  }

  // Called when the source ends where the next header would start: returns 0
  // (the end of the archive) if the source holds the last data byte of the
  // header before, or `kSyscallErrorIo` or the error of the source if it
  // doesn't. An archive may end without zero blocks, but not inside a member.
  int64_t CheckLastDataByte() {
    if (last_data_byte_ < 0) {
      return 0;
    }
    uint8_t byte = 0;
    const int64_t rc = source_->Read(last_data_byte_, &byte, 1);
    if (rc < 0) {
      return rc;
    }
    return (rc == 1) ? 0 : kSyscallErrorIo;
  }

  static bool BytesEqual(const char* const a,  //
                         const char* const b,  //
                         const int length) {
    for (int i = 0; i < length; ++i) {
      if (a[i] != b[i]) {
        return false;
      }
    }
    return true;
  }

  // Streams and parses the `pax_size` bytes of a PAX extended header (`'x'`) at
  // `data_offset` into `*pax`, reusing the 512-byte `scratch` buffer. Updates
  // `has_path`, `has_size` / `size`, and `has_gnu_sparse`; empty values unset
  // the keyword; unknown keywords are ignored. Returns 0 on success,
  // `kSyscallErrorInvalid` on malformed record syntax or overflowing size,
  // `kSyscallErrorIo` on short or over-sized reads, or the source's error.
  int64_t ParsePaxHeader(const int64_t data_offset,  //
                         const int64_t pax_size,     //
                         uint8_t* const scratch,     //
                         PaxState* const pax) {
    enum class PaxField : uint8_t { kLength, kKeyword, kValue };
    enum class PaxKey : uint8_t { kOther, kPath, kSize, kGnuSparse };

    PaxField state = PaxField::kLength;
    PaxKey key_kind = PaxKey::kOther;
    int64_t rec_bytes = 0;
    int64_t rec_len = 0;
    char key_buf[12] = {};
    int key_len = 0;
    bool val_nonempty = false;
    int64_t val_size = 0;
    bool val_size_valid = true;

    int64_t pos = 0;
    while (pos < pax_size) {
      const int64_t left = pax_size - pos;
      const int64_t want = (left < kBlockSize) ? left : kBlockSize;
      const int64_t got = source_->Read(data_offset + pos, scratch, want);
      if (got < 0) {
        return got;
      }
      if (got == 0 || got > want) {
        return kSyscallErrorIo;
      }
      for (int64_t i = 0; i < got; ++i) {
        const uint8_t byte = scratch[i];
        const int64_t cur_pos = pos + i;
        ++rec_bytes;
        switch (state) {
          case PaxField::kLength:
            if (byte == ' ') {
              // At least 1 keyword byte, '=' and '\n' must follow the space.
              if (rec_bytes <= 1 || rec_len - rec_bytes < 3 ||
                  rec_len - rec_bytes > pax_size - 1 - cur_pos) {
                return kSyscallErrorInvalid;
              }
              state = PaxField::kKeyword;
              key_len = 0;
              break;
            }
            if (byte < '0' || byte > '9' || (rec_bytes == 1 && byte == '0')) {
              return kSyscallErrorInvalid;
            }
            if (__builtin_mul_overflow(rec_len, 10, &rec_len) ||
                __builtin_add_overflow(rec_len, byte - '0', &rec_len)) {
              return kSyscallErrorInvalid;
            }
            break;

          case PaxField::kKeyword:
            if (rec_bytes == rec_len || byte == '\n' || byte == '\0') {
              return kSyscallErrorInvalid;
            }
            if (byte == '=') {
              if (key_len == 0) {
                return kSyscallErrorInvalid;
              }
              if (key_len == 4 && BytesEqual(key_buf, "path", 4)) {
                key_kind = PaxKey::kPath;
              } else if (key_len == 4 && BytesEqual(key_buf, "size", 4)) {
                key_kind = PaxKey::kSize;
              } else if (key_len >= 11 &&
                         BytesEqual(key_buf, "GNU.sparse.", 11)) {
                key_kind = PaxKey::kGnuSparse;
              } else {
                key_kind = PaxKey::kOther;
              }
              state = PaxField::kValue;
              val_nonempty = false;
              val_size = 0;
              val_size_valid = true;
              break;
            }
            if (key_len < 12) {
              key_buf[key_len++] = static_cast<char>(byte);
            }
            break;

          case PaxField::kValue:
            if (rec_bytes == rec_len) {
              if (byte != '\n') {
                return kSyscallErrorInvalid;
              }
              if (key_kind == PaxKey::kPath) {
                pax->has_path = val_nonempty;
              } else if (key_kind == PaxKey::kSize) {
                if (!val_nonempty) {
                  pax->has_size = false;
                  pax->size = 0;
                } else if (!val_size_valid) {
                  return kSyscallErrorInvalid;
                } else {
                  pax->has_size = true;
                  pax->size = val_size;
                }
              } else if (key_kind == PaxKey::kGnuSparse) {
                if (val_nonempty) {
                  pax->has_gnu_sparse = true;
                }
              }
              state = PaxField::kLength;
              rec_bytes = 0;
              rec_len = 0;
              break;
            }
            val_nonempty = true;
            if (key_kind == PaxKey::kSize && val_size_valid) {
              if (byte < '0' || byte > '9' ||
                  __builtin_mul_overflow(val_size, 10, &val_size) ||
                  __builtin_add_overflow(val_size, byte - '0', &val_size)) {
                val_size_valid = false;
              }
            }
            break;
        }
      }
      pos += got;
    }
    return (state == PaxField::kLength && rec_bytes == 0)
               ? 0
               : kSyscallErrorInvalid;
  }

  static bool IsZeroBlock(const uint8_t* const block) {
    for (int i = 0; i < kBlockSize; ++i) {
      if (block[i] != 0) {
        return false;
      }
    }
    return true;
  }

  // Parses the octal number in the `length`-byte field `field`: spaces (and
  // NULs if `leading_nuls`), at least one octal digit, and only NULs and
  // spaces after the digits, if any. Stores it in `*out` and returns true, or
  // returns false for anything else, such as a base-256 number (whose first
  // byte has bit 7 set). The longest field has 12 digits, which fit.
  static bool ParseOctal(const uint8_t* const field,  //
                         const int length,            //
                         const bool leading_nuls,     //
                         int64_t* const out) {
    int i = 0;
    while (i < length && (field[i] == ' ' || (leading_nuls && field[i] == 0))) {
      ++i;
    }
    const int first_digit = i;
    int64_t value = 0;
    while (i < length && field[i] >= '0' && field[i] <= '7') {
      value = value * 8 + (field[i] - '0');
      ++i;
    }
    if (i == first_digit) {
      return false;
    }
    for (; i < length; ++i) {
      if (field[i] != 0 && field[i] != ' ') {
        return false;
      }
    }
    *out = value;
    return true;
  }

  // Parses the size field, which may start with spaces.
  static bool ParseSize(const uint8_t* const block, int64_t* const out) {
    return ParseOctal(block + kSizeOffset,     //
                      kSizeLength,             //
                      /*leading_nuls=*/false,  //
                      out);
  }

  // Whether the checksum field, which may start with spaces and NULs, holds
  // the sum of the header's bytes, with the checksum field's bytes counted as
  // spaces, as unsigned chars (POSIX) or as signed chars (what some old
  // writers stored).
  static bool ChecksumMatches(const uint8_t* const block) {
    int64_t stored = 0;
    if (!ParseOctal(block + kChecksumOffset,  //
                    kChecksumLength,          //
                    /*leading_nuls=*/true,    //
                    &stored)) {
      return false;
    }
    int64_t unsigned_sum = 0;
    int64_t signed_sum = 0;
    for (int i = 0; i < kBlockSize; ++i) {
      const bool in_checksum =
          i >= kChecksumOffset && i < kChecksumOffset + kChecksumLength;
      const uint8_t byte = in_checksum ? uint8_t{' '} : block[i];
      unsigned_sum += byte;
      signed_sum += static_cast<int8_t>(byte);
    }
    return stored == unsigned_sum || stored == signed_sum;
  }

  // Whether the magic field starts with "ustar", as both the POSIX magic
  // ("ustar\0", version "00") and the old GNU one ("ustar  \0") do.
  static bool HasUstarMagic(const uint8_t* const block) {
    const char magic[] = "ustar";
    for (int i = 0; i < 5; ++i) {
      if (block[kMagicOffset + i] != static_cast<uint8_t>(magic[i])) {
        return false;
      }
    }
    return true;
  }

  // Stores the offset of the header after a member whose `size` (>= 0) data
  // bytes start at `data_offset` in `*out` and returns true, or returns false
  // if the end of that header block would overflow `int64_t`.
  static bool FollowingHeaderOffset(const int64_t data_offset,  //
                                    const int64_t size,         //
                                    int64_t* const out) {
    int64_t padded = 0;
    if (__builtin_add_overflow(size, kBlockSize - 1, &padded)) {
      return false;
    }
    padded -= padded % kBlockSize;
    int64_t next = 0;
    int64_t next_end = 0;
    if (__builtin_add_overflow(data_offset, padded, &next) ||
        __builtin_add_overflow(next, kBlockSize, &next_end)) {
      return false;
    }
    *out = next;
    return true;
  }

  // The type of a member with the type flag `typeflag`, before its name is
  // checked.
  static EntryType TypeOf(const uint8_t typeflag) {
    switch (typeflag) {
      case '0':
      case '\0':
      case '7':
        return EntryType::kRegular;
      case '5':
        return EntryType::kDirectory;
      default:
        return EntryType::kSkipped;
    }
  }

  // The number of bytes of the `length`-byte field `field` before its first
  // NUL, or `length` if it has none.
  static int FieldLength(const uint8_t* const field, const int length) {
    int len = 0;
    while (len < length && field[len] != 0) {
      ++len;
    }
    return len;
  }

  // Appends '/' and a component of the `length`-byte name `name` to
  // `entry->path` for each component but empty and `.` ones, and returns
  // true, or returns false if the name has a `..` component or a component
  // longer than `kFsMaxNameLength` bytes, or if the path would get longer than
  // `kFsMaxPathLength` bytes.
  static bool AppendComponents(const uint8_t* const name,  //
                               const int length,           //
                               Entry* const entry) {
    int pos = 0;
    while (pos < length) {
      if (name[pos] == '/') {
        ++pos;
        continue;
      }
      int end = pos;
      while (end < length && name[end] != '/') {
        ++end;
      }
      const int comp_len = end - pos;
      const bool dot = comp_len == 1 && name[pos] == '.';
      const bool dot_dot =
          comp_len == 2 && name[pos] == '.' && name[pos + 1] == '.';
      if (!dot) {
        if (dot_dot || comp_len > kFsMaxNameLength ||
            entry->path_len + 1 + comp_len > kFsMaxPathLength) {
          return false;
        }
        entry->path[entry->path_len++] = '/';
        for (int i = pos; i < end; ++i) {
          entry->path[entry->path_len++] = static_cast<char>(name[i]);
        }
      }
      pos = end;
    }
    return true;
  }

  // Fills `*entry` from the member header `block`, whose `size` data bytes
  // start at `data_offset`. `extended` means that an extended header before
  // it holds its real name or sparse layout.
  static void FillEntry(const uint8_t* const block,  //
                        const bool extended,         //
                        const int64_t size,          //
                        const int64_t data_offset,   //
                        Entry* const entry) {
    entry->type = EntryType::kSkipped;
    entry->path_len = 0;
    __builtin_memset(entry->path, 0, sizeof(entry->path));
    entry->size = size;
    entry->data_offset = data_offset;
    if (extended) {
      return;
    }

    const int name_len = FieldLength(block + kNameOffset, kNameLength);
    // The name is prefix + "/" + name if the prefix isn't empty. The old GNU
    // format, whose magic is followed by a space instead of a NUL, keeps other
    // data where POSIX ustar has the prefix.
    const int prefix_len =
        (block[kMagicOffset + 5] == 0)
            ? FieldLength(block + kPrefixOffset, kPrefixLength)
            : 0;
    EntryType type = TypeOf(block[kTypeflagOffset]);
    // Old tars wrote directories as regular files whose names end with '/'.
    const bool trailing_slash = (name_len > 0)
                                    ? block[kNameOffset + name_len - 1] == '/'
                                    : prefix_len > 0;
    if (type == EntryType::kRegular && trailing_slash) {
      type = EntryType::kDirectory;
    }
    if (type == EntryType::kSkipped) {
      return;
    }

    const bool usable =
        AppendComponents(block + kPrefixOffset, prefix_len, entry) &&
        AppendComponents(block + kNameOffset, name_len, entry);
    if (usable && entry->path_len == 0) {
      entry->path[entry->path_len++] = '/';
    }
    if (!usable || (type == EntryType::kRegular && entry->path_len == 1)) {
      entry->path_len = 0;
      __builtin_memset(entry->path, 0, sizeof(entry->path));
      return;
    }
    entry->type = type;
  }

  Source* source_ = nullptr;
  // The archive offset of the next header block.
  int64_t next_offset_ = 0;
  int64_t header_offset_ = 0;
  // The archive offset of the last data byte of the header that Next() read
  // last, or -1 if it has no data.
  int64_t last_data_byte_ = -1;
  // Whether Next() has returned 0 or an error, and which.
  bool finished_ = false;
  int64_t result_ = 0;
};

// A reader can be a `constinit` global of a freestanding ring-3 binary.
static_assert(std::is_trivially_destructible_v<Reader>);

}  // namespace protos::tar

#endif  // PROTOS_USER_RAMFS_TAR_H_
