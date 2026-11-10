#ifndef PROTOS_USER_DISK_DISK_LIB_H_
#define PROTOS_USER_DISK_DISK_LIB_H_

#include <cstdint>

#include "block_protocol.h"
#include "status_or.h"
#include "user/block.h"
#include "user/syscall.h"

// Freestanding implementation of `/bin/disk`, parameterized over `DiskEnv` so
// host unit tests can exercise CLI parsing, capacity formatting, hex/ASCII
// dumping, bounds validation, and block RPC error handling without syscalls.

namespace protos::user::disk {

// `write` and `zero` overwrite blocks of the disk itself, whatever they hold.
// They leave a disk that is not a disposable test image
// (`IsBlockTestDiskSerial()`) alone, such as a real disk when the kernel runs
// on a real machine, unless `-f` forces them.
constexpr char kUsageGeneral[] =
    "disk: usage:\n"
    "  disk [-d <disk>] [info]\n"
    "  disk [-d <disk>] read <lba> [blocks]\n"
    "  disk [-d <disk>] [-f] write <lba> <text...>\n"
    "  disk [-d <disk>] [-f] zero <lba> [blocks]\n"
    "write and zero need -f for a disk whose serial does not start with "
    "\"protos\"\n";
constexpr char kUsageRead[] =
    "disk: usage: disk [-d <disk>] read <lba> [blocks]\n";
constexpr char kUsageWrite[] =
    "disk: usage: disk [-d <disk>] [-f] write <lba> <text...>\n";
constexpr char kUsageZero[] =
    "disk: usage: disk [-d <disk>] [-f] zero <lba> [blocks]\n";

struct DiskEnv {
  BlockClientEnv block{};
  void* write_ctx = nullptr;
  int64_t (*write)(void* ctx, const char* data, int64_t size) = nullptr;
};

inline void WriteRaw(const DiskEnv& env,      //
                     const char* const data,  //
                     const int64_t size) {
  if (env.write != nullptr && data != nullptr && size > 0) {
    (void)env.write(env.write_ctx, data, size);
  }
}

inline void WriteCStr(const DiskEnv& env, const char* const text) {
  if (text == nullptr) {
    return;
  }
  int64_t len = 0;
  while (text[len] != '\0') {
    ++len;
  }
  WriteRaw(env, text, len);
}

class Out {
 public:
  explicit Out(const DiskEnv& env) : env_(env) {}

  Out& Str(const char* const text) {
    if (text == nullptr) {
      return *this;
    }
    for (int i = 0; text[i] != '\0'; ++i) {
      Char(text[i]);
    }
    return *this;
  }

  Out& StrN(const char* const text, const int cap) {
    if (text == nullptr || cap <= 0) {
      return *this;
    }
    for (int i = 0; i < cap && text[i] != '\0'; ++i) {
      Char(text[i]);
    }
    return *this;
  }

  Out& Char(const char ch) {
    if (len_ == kCapacity) {
      Flush();
    }
    buf_[len_++] = ch;
    return *this;
  }

  Out& Dec(const uint64_t val) {
    char digits[20];
    int n = 0;
    uint64_t v = val;
    do {
      digits[n++] = static_cast<char>('0' + (v % 10));
      v /= 10;
    } while (v != 0);
    while (n > 0) {
      Char(digits[--n]);
    }
    return *this;
  }

  Out& Hex(const uint64_t val, const int digits) {
    constexpr char kHex[] = "0123456789abcdef";
    const int clamped = (digits < 0) ? 0 : (digits > 16 ? 16 : digits);
    for (int i = clamped - 1; i >= 0; --i) {
      Char(kHex[(val >> (4 * i)) & 0xFu]);
    }
    return *this;
  }

  void Flush() {
    if (len_ > 0) {
      WriteRaw(env_, buf_, len_);
      len_ = 0;
    }
  }

 private:
  static constexpr int64_t kCapacity = 512;
  const DiskEnv& env_;
  char buf_[kCapacity] = {};
  int64_t len_ = 0;
};

inline bool StrEq(const char* const a, const char* const b) {
  if (a == nullptr || b == nullptr) {
    return a == b;
  }
  int i = 0;
  while (a[i] != '\0' && b[i] != '\0') {
    if (a[i] != b[i]) {
      return false;
    }
    ++i;
  }
  return a[i] == b[i];
}

inline bool ParseU64(const char* const s, uint64_t* const out) {
  if (s == nullptr || out == nullptr || s[0] == '\0') {
    return false;
  }
  uint64_t v = 0;
  for (int i = 0; s[i] != '\0'; ++i) {
    const char ch = s[i];
    if (ch < '0' || ch > '9') {
      return false;
    }
    const uint64_t digit = static_cast<uint64_t>(ch - '0');
    if (v > (UINT64_MAX - digit) / 10) {
      return false;
    }
    v = v * 10 + digit;
  }
  *out = v;
  return true;
}

inline bool ParseU32(const char* const s, uint32_t* const out) {
  if (out == nullptr) {
    return false;
  }
  uint64_t v = 0;
  if (!ParseU64(s, &v) || v > UINT32_MAX) {
    return false;
  }
  *out = static_cast<uint32_t>(v);
  return true;
}

enum class DiskSubcommand : uint8_t {
  kInfo = 0,
  kRead = 1,
  kWrite = 2,
  kZero = 3,
};

struct ParsedDiskCommand {
  DiskSubcommand op = DiskSubcommand::kInfo;
  bool has_disk = false;
  uint32_t disk_index = 0;
  // `-f`: write and zero may overwrite a disk that is not a test image.
  bool force = false;
  uint64_t lba = 0;
  int32_t count = 1;
  int text_argc = 0;
  const char* const* text_argv = nullptr;
};

inline bool ParseDiskCommand(const int argc,                 //
                             const char* const* const argv,  //
                             ParsedDiskCommand* const out,   //
                             const char** const out_usage) {
  const auto fail = [out_usage](const char* const msg) {
    if (out_usage != nullptr) {
      *out_usage = msg;
    }
    return false;
  };
  if (out == nullptr || argc < 0 || (argc > 0 && argv == nullptr)) {
    return fail(kUsageGeneral);
  }
  for (int i = 0; i < argc; ++i) {
    if (argv[i] == nullptr) {
      return fail(kUsageGeneral);
    }
  }
  *out = ParsedDiskCommand{};
  if (argc <= 1) {
    out->op = DiskSubcommand::kInfo;
    return true;
  }

  // The options come before the subcommand, each at most once.
  int arg_idx = 1;
  while (arg_idx < argc) {
    if (StrEq(argv[arg_idx], "-d")) {
      if (out->has_disk || arg_idx + 1 >= argc ||
          !ParseU32(argv[arg_idx + 1], &out->disk_index)) {
        return fail(kUsageGeneral);
      }
      out->has_disk = true;
      arg_idx += 2;
    } else if (StrEq(argv[arg_idx], "-f")) {
      if (out->force) {
        return fail(kUsageGeneral);
      }
      out->force = true;
      ++arg_idx;
    } else {
      break;
    }
  }
  // -f goes only with the subcommands that write.
  const bool writes = arg_idx < argc && (StrEq(argv[arg_idx], "write") ||
                                         StrEq(argv[arg_idx], "zero"));
  if (out->force && !writes) {
    return fail(kUsageGeneral);
  }
  if (arg_idx >= argc) {
    out->op = DiskSubcommand::kInfo;
    return true;
  }
  if (StrEq(argv[arg_idx], "info")) {
    if (argc != arg_idx + 1) {
      return fail(kUsageGeneral);
    }
    out->op = DiskSubcommand::kInfo;
    return true;
  }
  if (StrEq(argv[arg_idx], "read")) {
    uint64_t count64 = 1;
    if ((argc != arg_idx + 2 && argc != arg_idx + 3) ||
        !ParseU64(argv[arg_idx + 1], &out->lba) ||
        (argc == arg_idx + 3 && (!ParseU64(argv[arg_idx + 2], &count64) ||
                                 count64 < 1 || count64 > INT32_MAX))) {
      return fail(kUsageRead);
    }
    out->op = DiskSubcommand::kRead;
    out->count = static_cast<int32_t>(count64);
    return true;
  }
  if (StrEq(argv[arg_idx], "write")) {
    if (argc < arg_idx + 3 || !ParseU64(argv[arg_idx + 1], &out->lba)) {
      return fail(kUsageWrite);
    }
    out->op = DiskSubcommand::kWrite;
    out->text_argc = argc - (arg_idx + 2);
    out->text_argv = argv + (arg_idx + 2);
    return true;
  }
  if (StrEq(argv[arg_idx], "zero")) {
    uint64_t count64 = 1;
    if ((argc != arg_idx + 2 && argc != arg_idx + 3) ||
        !ParseU64(argv[arg_idx + 1], &out->lba) ||
        (argc == arg_idx + 3 && (!ParseU64(argv[arg_idx + 2], &count64) ||
                                 count64 < 1 || count64 > INT32_MAX))) {
      return fail(kUsageZero);
    }
    out->op = DiskSubcommand::kZero;
    out->count = static_cast<int32_t>(count64);
    return true;
  }
  return fail(kUsageGeneral);
}

inline void PrintBlockError(const DiskEnv& env,    //
                            const char* const op,  //
                            const int64_t rc) {
  Out out(env);
  out.Str("disk: ").Str(op).Str(": ");
  switch (rc) {
    case kSyscallErrorNoDevice:
      out.Str("no block device\n");
      break;
    case kSyscallErrorPermission:
      out.Str("read-only or permission denied\n");
      break;
    case kSyscallErrorInvalid:
      out.Str("invalid argument or LBA out of range\n");
      break;
    case kSyscallErrorIo:
      out.Str("I/O error\n");
      break;
    default:
      if (rc < 0) {
        out.Str("error ").Dec(0ULL - static_cast<uint64_t>(rc)).Str("\n");
      } else {
        out.Str("unexpected status ").Dec(static_cast<uint64_t>(rc)).Str("\n");
      }
      break;
  }
  out.Flush();
}

// Says that `op` ("write" or "zero") leaves the disk alone because it is not a
// test image (`IsBlockTestDiskSerial()`) and `-f` did not force `op`.
inline void PrintNotTestDisk(const DiskEnv& env, const char* const op) {
  Out out(env);
  out.Str("disk: ")
      .Str(op)
      .Str(": not a test disk (serial is not ")
      .Str(kBlockTestDiskSerialPrefix)
      .Str("*); -f overwrites it\n");
  out.Flush();
}

// Formats `block_count * block_size` without 64-bit multiplication overflow.
inline void FormatCapacity(const uint64_t block_count,  //
                           const int32_t block_size,    //
                           Out& out) {
  if (block_count == 0 || !IsValidBlockSize(block_size)) {
    out.Str("0 B");
    return;
  }
  if (block_size == 512) {
    if (block_count == 1) {
      out.Str("512 B");
      return;
    }
    const uint64_t total_kib = block_count / 2ULL;
    if (total_kib < 1024ULL) {
      out.Dec(total_kib).Str(" KiB");
    } else {
      out.Dec(block_count / 2048ULL).Str(" MiB");
    }
    return;
  }
  const uint64_t kib_per_block = static_cast<uint64_t>(block_size) / 1024ULL;
  if (block_count < 1024ULL / kib_per_block) {
    out.Dec(block_count * kib_per_block).Str(" KiB");
    return;
  }
  const uint64_t total_mib =
      (block_count / 1024ULL) * kib_per_block +
      ((block_count % 1024ULL) * kib_per_block) / 1024ULL;
  out.Dec(total_mib).Str(" MiB");
}

inline bool FormatDeviceInfo(const uint32_t disk_index,    //
                             const BlockDeviceInfo& info,  //
                             Out& out) {
  if (!IsValidBlockDeviceInfo(info)) {
    return false;
  }
  out.Str("disk ")
      .Dec(disk_index)
      .Str(": ")
      .StrN(info.model, kBlockModelCapacity)
      .Str(" (serial=")
      .StrN(info.serial, kBlockSerialCapacity)
      .Str(", nsid=")
      .Dec(info.nsid)
      .Str(", pci=")
      .Hex(info.bus, 2)
      .Str(":")
      .Hex(info.device, 2)
      .Str(".")
      .Hex(info.function, 1)
      .Str(", ro=")
      .Dec(info.read_only != 0 ? 1 : 0)
      .Str(")\n  ")
      .Dec(info.block_count)
      .Str(" blocks x ")
      .Dec(static_cast<uint64_t>(info.block_size))
      .Str(" B = ");
  FormatCapacity(info.block_count, info.block_size, out);
  out.Str(" (max ")
      .Dec(static_cast<uint64_t>(info.max_transfer_blocks))
      .Str(" blocks/call)\n");
  return true;
}

inline int CmdInfo(const DiskEnv& env,   //
                   const bool has_disk,  //
                   const uint32_t disk_index) {
  if (has_disk) {
    const StatusOr<BlockDeviceInfo> info =
        GetBlockDeviceInfo(env.block, disk_index, nullptr);
    if (!info.ok()) {
      PrintBlockError(env, "info", info.status());
      return 1;
    }
    Out out(env);
    if (!FormatDeviceInfo(disk_index, *info, out)) {
      PrintBlockError(env, "info", kSyscallErrorIo);
      return 1;
    }
    out.Flush();
    return 0;
  }

  int64_t disk_count = 0;
  const StatusOr<BlockDeviceInfo> first =
      GetBlockDeviceInfo(env.block, 0, &disk_count);
  if (!first.ok()) {
    PrintBlockError(env, "info", first.status());
    return 1;
  }
  if (disk_count < 1 || disk_count > static_cast<int64_t>(UINT32_MAX) + 1) {
    PrintBlockError(env, "info", kSyscallErrorIo);
    return 1;
  }
  Out out(env);
  for (int64_t d = 0; d < disk_count; ++d) {
    const uint32_t idx = static_cast<uint32_t>(d);
    const StatusOr<BlockDeviceInfo> res =
        (d == 0) ? first : GetBlockDeviceInfo(env.block, idx, nullptr);
    if (!res.ok()) {
      out.Flush();
      PrintBlockError(env, "info", res.status());
      return 1;
    }
    if (!FormatDeviceInfo(idx, *res, out)) {
      out.Flush();
      PrintBlockError(env, "info", kSyscallErrorIo);
      return 1;
    }
  }
  out.Flush();
  return 0;
}

inline int CmdRead(const DiskEnv& env,         //
                   const uint32_t disk_index,  //
                   const uint64_t lba,         //
                   const int32_t count,        //
                   uint8_t* const io_buf,      //
                   const int64_t io_cap) {
  if (io_buf == nullptr || io_cap <= 0) {
    PrintBlockError(env, "read", kSyscallErrorInvalid);
    return 1;
  }
  const StatusOr<BlockDeviceInfo> info =
      GetBlockDeviceInfo(env.block, disk_index, nullptr);
  if (!info.ok()) {
    PrintBlockError(env, "read", info.status());
    return 1;
  }
  if (!IsValidBlockDeviceInfo(*info)) {
    PrintBlockError(env, "read", kSyscallErrorIo);
    return 1;
  }
  if (count < 1 || count > info->max_transfer_blocks) {
    WriteCStr(env, "disk: read: block count out of range\n");
    return 1;
  }
  const int64_t bytes =
      static_cast<int64_t>(count) * static_cast<int64_t>(info->block_size);
  if (bytes <= 0 || bytes > kBlockMaxIoBytes || bytes > io_cap) {
    PrintBlockError(env, "read", kSyscallErrorIo);
    return 1;
  }
  const int64_t rc = ReadBlocksChecked(env.block,         //
                                       disk_index,        //
                                       lba,               //
                                       count,             //
                                       info->block_size,  //
                                       io_buf,            //
                                       bytes);
  if (rc != count) {
    PrintBlockError(env, "read", rc < 0 ? rc : kSyscallErrorIo);
    return 1;
  }

  Out out(env);
  bool skipping_zero = false;
  bool any_nonzero = false;
  for (int64_t off = 0; off < bytes; off += 16) {
    bool all_zero = true;
    for (int64_t i = 0; i < 16 && off + i < bytes; ++i) {
      if (io_buf[off + i] != 0) {
        all_zero = false;
        any_nonzero = true;
        break;
      }
    }
    if (all_zero) {
      if (!skipping_zero) {
        out.Str("...\n");
        skipping_zero = true;
      }
      continue;
    }
    skipping_zero = false;
    out.Hex(static_cast<uint64_t>(off), 4).Str(": ");
    for (int64_t i = 0; i < 16; ++i) {
      if (off + i < bytes) {
        out.Hex(io_buf[off + i], 2).Char(' ');
      } else {
        out.Str("   ");
      }
    }
    out.Char('|');
    for (int64_t i = 0; i < 16 && off + i < bytes; ++i) {
      const uint8_t ch = io_buf[off + i];
      out.Char(ch >= 0x20 && ch <= 0x7E ? static_cast<char>(ch) : '.');
    }
    out.Str("|\n");
  }
  if (!any_nonzero && bytes > 0) {
    out.Str("(zero through ").Dec(static_cast<uint64_t>(bytes)).Str(" B)\n");
  }
  out.Flush();
  return 0;
}

inline int CmdWriteText(const DiskEnv& env,                  //
                        const uint32_t disk_index,           //
                        const bool force,                    //
                        const uint64_t lba,                  //
                        const int text_argc,                 //
                        const char* const* const text_argv,  //
                        uint8_t* const io_scratch,           //
                        const int64_t scratch_cap) {
  if (io_scratch == nullptr || scratch_cap < kBlockWriteScratchBytes ||
      text_argc < 1 || text_argv == nullptr) {
    PrintBlockError(env, "write", kSyscallErrorInvalid);
    return 1;
  }
  const StatusOr<BlockDeviceInfo> info =
      GetBlockDeviceInfo(env.block, disk_index, nullptr);
  if (!info.ok()) {
    PrintBlockError(env, "write", info.status());
    return 1;
  }
  if (!IsValidBlockDeviceInfo(*info)) {
    PrintBlockError(env, "write", kSyscallErrorIo);
    return 1;
  }
  if (info->read_only != 0) {
    PrintBlockError(env, "write", kSyscallErrorPermission);
    return 1;
  }
  if (!force && !IsBlockTestDiskSerial(info->serial)) {
    PrintNotTestDisk(env, "write");
    return 1;
  }
  const int64_t max_bytes =
      static_cast<int64_t>(info->max_transfer_blocks) * info->block_size;
  if (max_bytes <= 0 || max_bytes > kBlockMaxIoBytes) {
    PrintBlockError(env, "write", kSyscallErrorIo);
    return 1;
  }
  uint8_t* const io_buf = io_scratch + sizeof(BlockWriteRequest);
  for (int64_t i = 0; i < max_bytes; ++i) {
    io_buf[i] = 0;
  }
  int64_t pos = 0;
  for (int i = 0; i < text_argc; ++i) {
    if (text_argv[i] == nullptr) {
      PrintBlockError(env, "write", kSyscallErrorInvalid);
      return 1;
    }
    if (i > 0) {
      if (pos >= max_bytes) {
        WriteCStr(env, "disk: write: text too long\n");
        return 1;
      }
      io_buf[pos++] = ' ';
    }
    for (int j = 0; text_argv[i][j] != '\0'; ++j) {
      if (pos >= max_bytes) {
        WriteCStr(env, "disk: write: text too long\n");
        return 1;
      }
      io_buf[pos++] = static_cast<uint8_t>(text_argv[i][j]);
    }
  }
  if (pos >= max_bytes) {
    WriteCStr(env, "disk: write: text too long\n");
    return 1;
  }
  io_buf[pos++] = '\n';

  const int32_t bs = info->block_size;
  const int64_t blocks64 = (pos + bs - 1) / bs;
  if (blocks64 < 1 || blocks64 > info->max_transfer_blocks) {
    PrintBlockError(env, "write", kSyscallErrorIo);
    return 1;
  }
  const int32_t blocks = static_cast<int32_t>(blocks64);
  const int64_t write_bytes = static_cast<int64_t>(blocks) * bs;
  if (write_bytes <= 0 || write_bytes > kBlockMaxIoBytes) {
    PrintBlockError(env, "write", kSyscallErrorIo);
    return 1;
  }
  const int64_t rc = WriteBlocksWithScratch(env.block,    //
                                            disk_index,   //
                                            lba,          //
                                            blocks,       //
                                            io_buf,       //
                                            write_bytes,  //
                                            io_scratch,   //
                                            scratch_cap);
  if (rc != blocks) {
    PrintBlockError(env, "write", rc < 0 ? rc : kSyscallErrorIo);
    return 1;
  }
  const int64_t flush_rc = FlushDisk(env.block, disk_index);
  if (flush_rc != 0) {
    PrintBlockError(env, "flush", flush_rc);
    return 1;
  }
  Out out(env);
  out.Str("wrote ")
      .Dec(static_cast<uint64_t>(blocks))
      .Str(" block(s) at lba ")
      .Dec(lba)
      .Str("\n");
  out.Flush();
  return 0;
}

inline int CmdZero(const DiskEnv& env,         //
                   const uint32_t disk_index,  //
                   const bool force,           //
                   const uint64_t lba,         //
                   const int32_t count,        //
                   uint8_t* const io_scratch,  //
                   const int64_t scratch_cap) {
  if (io_scratch == nullptr || scratch_cap < kBlockWriteScratchBytes) {
    PrintBlockError(env, "zero", kSyscallErrorInvalid);
    return 1;
  }
  const StatusOr<BlockDeviceInfo> info =
      GetBlockDeviceInfo(env.block, disk_index, nullptr);
  if (!info.ok()) {
    PrintBlockError(env, "zero", info.status());
    return 1;
  }
  if (!IsValidBlockDeviceInfo(*info)) {
    PrintBlockError(env, "zero", kSyscallErrorIo);
    return 1;
  }
  if (info->read_only != 0) {
    PrintBlockError(env, "zero", kSyscallErrorPermission);
    return 1;
  }
  if (!force && !IsBlockTestDiskSerial(info->serial)) {
    PrintNotTestDisk(env, "zero");
    return 1;
  }
  if (count < 1 || count > info->max_transfer_blocks) {
    WriteCStr(env, "disk: zero: block count out of range\n");
    return 1;
  }
  const int64_t bytes =
      static_cast<int64_t>(count) * static_cast<int64_t>(info->block_size);
  if (bytes <= 0 || bytes > kBlockMaxIoBytes) {
    PrintBlockError(env, "zero", kSyscallErrorIo);
    return 1;
  }
  uint8_t* const io_buf = io_scratch + sizeof(BlockWriteRequest);
  for (int64_t i = 0; i < bytes; ++i) {
    io_buf[i] = 0;
  }
  const int64_t rc = WriteBlocksWithScratch(env.block,   //
                                            disk_index,  //
                                            lba,         //
                                            count,       //
                                            io_buf,      //
                                            bytes,       //
                                            io_scratch,  //
                                            scratch_cap);
  if (rc != count) {
    PrintBlockError(env, "zero", rc < 0 ? rc : kSyscallErrorIo);
    return 1;
  }
  const int64_t flush_rc = FlushDisk(env.block, disk_index);
  if (flush_rc != 0) {
    PrintBlockError(env, "flush", flush_rc);
    return 1;
  }
  Out out(env);
  out.Str("zeroed ")
      .Dec(static_cast<uint64_t>(count))
      .Str(" block(s) at lba ")
      .Dec(lba)
      .Str("\n");
  out.Flush();
  return 0;
}

inline int RunDisk(const DiskEnv& env,             //
                   const int argc,                 //
                   const char* const* const argv,  //
                   uint8_t* const io_scratch,      //
                   const int64_t scratch_cap) {
  ParsedDiskCommand cmd{};
  const char* usage = kUsageGeneral;
  if (!ParseDiskCommand(argc, argv, &cmd, &usage)) {
    WriteCStr(env, usage);
    return 1;
  }
  switch (cmd.op) {
    case DiskSubcommand::kInfo:
      return CmdInfo(env, cmd.has_disk, cmd.disk_index);
    case DiskSubcommand::kRead: {
      uint8_t* const read_buf =
          (io_scratch != nullptr &&
           scratch_cap > static_cast<int64_t>(sizeof(BlockWriteRequest)))
              ? (io_scratch + sizeof(BlockWriteRequest))
              : nullptr;
      const int64_t read_cap =
          (read_buf != nullptr)
              ? (scratch_cap - static_cast<int64_t>(sizeof(BlockWriteRequest)))
              : 0;
      return CmdRead(env,             //
                     cmd.disk_index,  //
                     cmd.lba,         //
                     cmd.count,       //
                     read_buf,        //
                     read_cap);
    }
    case DiskSubcommand::kWrite:
      return CmdWriteText(env,             //
                          cmd.disk_index,  //
                          cmd.force,       //
                          cmd.lba,         //
                          cmd.text_argc,   //
                          cmd.text_argv,   //
                          io_scratch,      //
                          scratch_cap);
    case DiskSubcommand::kZero:
      return CmdZero(env,             //
                     cmd.disk_index,  //
                     cmd.force,       //
                     cmd.lba,         //
                     cmd.count,       //
                     io_scratch,      //
                     scratch_cap);
  }
  return 1;
}

}  // namespace protos::user::disk

#endif  // PROTOS_USER_DISK_DISK_LIB_H_
