#ifndef PROTOS_USER_SH_SH_LIB_H_
#define PROTOS_USER_SH_SH_LIB_H_

#include <cstdint>

#include "status_or.h"
#include "syscall_abi.h"

// Freestanding line editor, command parser, and command dispatcher for the
// user-space shell (user/sh/sh.cpp). Parameterized over a callback environment
// (`ShellEnv`) so host unit tests (syscall_abi_test.cpp) can exercise the
// exact shell implementation directly.

namespace protos::sh {

// The longest line that the shell runs, in characters: room for any command
// with two operands of `kFsMaxPathLength` bytes, such as `cp` and `mv`. The
// shell refuses a longer line ("sh: line too long") rather than run part of it.
constexpr int kShellMaxLineLength = 1024;
static_assert(kShellMaxLineLength >= 8 + 2 * (kFsMaxPathLength + 1));
constexpr int kShellMaxTokens = 16;
// Room for the longest line that the shell builds in one buffer: the words
// of a line (echo, write and append), or an error message about one of them.
// Commands with an unbounded number of entries, like `programs`, flush in
// pieces.
constexpr int kShellOutputBufferSize = kShellMaxLineLength + 64;
constexpr const char* kShellPrompt = "sh> ";
// The words of any line that the shell reads, with a newline, fit in a
// `ShellOutBuffer` (echo, write and append).
static_assert(kShellMaxLineLength + 1 < kShellOutputBufferSize);
// The most directory entries that `ls` reads with one ReadDir() call and the
// most bytes that `cat` and `cp` read with one FileRead() call. Their buffers
// are on the stack, which has 16 KiB in a user program (`kUserStackPages` in
// user_mode.h), so they are smaller than the system calls allow.
constexpr int kShellReadDirBatch = 8;
constexpr int kShellFileChunkSize = 2048;
static_assert(kShellReadDirBatch <= kSyscallFsMaxDirEntries);
static_assert(kShellFileChunkSize <= kSyscallFsMaxIoSize);

// Action taken by `ShellLineEditor::FeedChar()` for one input byte.
enum class ShellCharAction : uint8_t {
  kNone = 0,
  kEchoChar = 1,
  kBackspaceErase = 2,
  kSubmitLine = 3,
  kIgnoreLfAfterCr = 4,
};

// Stateful single-line editor supporting printable ASCII echoing, backspace
// ('\b' = 0x08 and DEL = 0x7F) with column-0 underflow protection, fixed-size
// buffer clamping, and '\r' / '\n' / "\r\n" line submission.
struct ShellLineEditor {
  char buffer[kShellMaxLineLength + 1] = {};
  int length = 0;
  // Whether the line lost a printable character that didn't fit in `buffer`
  // (which isn't echoed), so that the shell must not run it. Backspace doesn't
  // bring the character back, so it doesn't clear this.
  bool overflowed = false;
  bool last_was_cr = false;

  void ResetLine() {
    length = 0;
    buffer[0] = '\0';
    overflowed = false;
  }

  ShellCharAction FeedChar(char* const io_ch) {
    const char ch = *io_ch;
    if (ch == '\n' && last_was_cr) {
      last_was_cr = false;
      return ShellCharAction::kIgnoreLfAfterCr;
    }
    last_was_cr = (ch == '\r');
    if (ch == '\r' || ch == '\n') {
      buffer[length] = '\0';
      return ShellCharAction::kSubmitLine;
    }
    if (ch == '\b' || static_cast<uint8_t>(ch) == 0x7F) {
      if (length > 0) {
        --length;
        buffer[length] = '\0';
        return ShellCharAction::kBackspaceErase;
      }
      return ShellCharAction::kNone;
    }
    char normalized = ch;
    if (normalized == '\t') {
      normalized = ' ';
    }
    const uint8_t byte = static_cast<uint8_t>(normalized);
    if (byte >= 0x20 && byte <= 0x7E) {
      if (length < kShellMaxLineLength) {
        buffer[length] = normalized;
        ++length;
        buffer[length] = '\0';
        *io_ch = normalized;
        return ShellCharAction::kEchoChar;
      }
      overflowed = true;
    }
    return ShellCharAction::kNone;
  }
};

struct ShellToken {
  const char* ptr = nullptr;
  int length = 0;
};

struct ShellCommand {
  ShellToken tokens[kShellMaxTokens] = {};
  int token_count = 0;
  // Whether the line had more than `kShellMaxTokens` tokens, of which `tokens`
  // holds the first `kShellMaxTokens`.
  bool truncated = false;
};

inline int StringLength(const char* const text) {
  if (text == nullptr) {
    return 0;
  }
  int len = 0;
  while (text[len] != '\0') {
    ++len;
  }
  return len;
}

inline bool TokenEquals(const ShellToken& token, const char* const literal) {
  if (token.ptr == nullptr || literal == nullptr) {
    return false;
  }
  int i = 0;
  while (i < token.length && literal[i] != '\0') {
    if (token.ptr[i] != literal[i]) {
      return false;
    }
    ++i;
  }
  return i == token.length && literal[i] == '\0';
}

inline ShellCommand TokenizeLine(const char* const line,
                                 const int line_length) {
  ShellCommand out{};
  if (line == nullptr || line_length <= 0) {
    return out;
  }
  int i = 0;
  while (i < line_length) {
    while (i < line_length && (line[i] == ' ' || line[i] == '\t')) {
      ++i;
    }
    if (i >= line_length) {
      break;
    }
    const int start = i;
    while (i < line_length && line[i] != ' ' && line[i] != '\t') {
      ++i;
    }
    if (out.token_count < kShellMaxTokens) {
      out.tokens[out.token_count].ptr = line + start;
      out.tokens[out.token_count].length = i - start;
      ++out.token_count;
    } else {
      out.truncated = true;
    }
  }
  return out;
}

// Parses a signed 64-bit integer in decimal or 0x-prefixed hexadecimal from
// `token` without signed overflow undefined behavior.
inline StatusOr<int64_t> ParseInt64(const ShellToken& token) {
  if (token.ptr == nullptr || token.length <= 0) {
    return kStatusError;
  }
  int index = 0;
  bool negative = false;
  if (token.ptr[0] == '-') {
    negative = true;
    index = 1;
  } else if (token.ptr[0] == '+') {
    index = 1;
  }
  if (index >= token.length) {
    return kStatusError;
  }
  uint64_t base = 10;
  if (index + 2 <= token.length && token.ptr[index] == '0' &&
      (token.ptr[index + 1] == 'x' || token.ptr[index + 1] == 'X')) {
    base = 16;
    index += 2;
    if (index >= token.length) {
      return kStatusError;
    }
  }
  uint64_t value = 0;
  for (; index < token.length; ++index) {
    const char ch = token.ptr[index];
    uint64_t digit = 0;
    if (ch >= '0' && ch <= '9') {
      digit = static_cast<uint64_t>(ch - '0');
    } else if (base == 16 && ch >= 'a' && ch <= 'f') {
      digit = static_cast<uint64_t>(10 + (ch - 'a'));
    } else if (base == 16 && ch >= 'A' && ch <= 'F') {
      digit = static_cast<uint64_t>(10 + (ch - 'A'));
    } else {
      return kStatusError;
    }
    if (value > (UINT64_MAX - digit) / base) {
      return kStatusError;
    }
    value = value * base + digit;
  }
  if (negative) {
    constexpr uint64_t kMaxNegativeMagnitude =
        static_cast<uint64_t>(INT64_MAX) + 1ULL;
    if (value > kMaxNegativeMagnitude) {
      return kStatusError;
    }
    return (value == kMaxNegativeMagnitude) ? INT64_MIN
                                            : -static_cast<int64_t>(value);
  }
  if (value > static_cast<uint64_t>(INT64_MAX)) {
    return kStatusError;
  }
  return static_cast<int64_t>(value);
}

// Parses a program argument token: either an integer or a named `usertest`
// mode alias ("exit" -> 0, "syscalls" -> 1, "fault" / "ud2" -> 6,
// "buffers" -> 16).
inline StatusOr<uint64_t> ParseProgramArg(const ShellToken& token) {
  const StatusOr<int64_t> parsed = ParseInt64(token);
  if (parsed.ok()) {
    return static_cast<uint64_t>(*parsed);
  }
  if (TokenEquals(token, "exit")) {
    return uint64_t{0};
  }
  if (TokenEquals(token, "syscalls")) {
    return uint64_t{1};
  }
  if (TokenEquals(token, "fault") || TokenEquals(token, "ud2")) {
    return uint64_t{6};
  }
  if (TokenEquals(token, "buffers")) {
    return uint64_t{16};
  }
  return kStatusError;
}

inline bool IsKnownUserProgram(const ShellToken& token) {
  return TokenEquals(token, "init") || TokenEquals(token, "sh") ||
         TokenEquals(token, "usertest");
}

struct ShellOutBuffer {
  char data[kShellOutputBufferSize] = {};
  int length = 0;

  void Clear() {
    length = 0;
    data[0] = '\0';
  }

  void AppendChar(const char ch) {
    if (length < kShellOutputBufferSize - 1) {
      data[length++] = ch;
      data[length] = '\0';
    }
  }

  void AppendBytes(const char* const bytes, const int count) {
    for (int i = 0; i < count; ++i) {
      AppendChar(bytes[i]);
    }
  }

  void AppendStr(const char* const text) {
    if (text == nullptr) {
      return;
    }
    for (int i = 0; text[i] != '\0'; ++i) {
      AppendChar(text[i]);
    }
  }

  void AppendUint64Dec(const uint64_t value) {
    char digits[21] = {};
    int pos = 20;
    uint64_t remaining = value;
    do {
      --pos;
      digits[pos] = static_cast<char>('0' + (remaining % 10));
      remaining /= 10;
    } while (remaining != 0);
    AppendBytes(digits + pos, 20 - pos);
  }

  void AppendInt64Dec(const int64_t value) {
    if (value < 0) {
      AppendChar('-');
      const uint64_t magnitude = 0ULL - static_cast<uint64_t>(value);
      AppendUint64Dec(magnitude);
    } else {
      AppendUint64Dec(static_cast<uint64_t>(value));
    }
  }

  void AppendHex64(const uint64_t value) {
    AppendStr("0x");
    char digits[16] = {};
    int pos = 16;
    uint64_t remaining = value;
    do {
      --pos;
      digits[pos] = "0123456789ABCDEF"[remaining & 0xF];
      remaining >>= 4;
    } while (remaining != 0);
    AppendBytes(digits + pos, 16 - pos);
  }
};

// System call callbacks used by the shell. A filesystem built-in that lacks a
// callback that it needs (the filesystem callbacks, `open` through `rename`,
// may be null) reports the filesystem unavailable instead of calling the
// others.
struct ShellEnv {
  // MkDir() or Unlink() of the path of `path_len` bytes at `path`.
  using PathOp = int64_t (*)(void* ctx, const char* path, int64_t path_len);

  void* ctx = nullptr;
  int64_t (*read)(void* ctx, char* buffer, int64_t size) = nullptr;
  int64_t (*write)(void* ctx, const char* buffer, int64_t size) = nullptr;
  int (*get_cpu)(void* ctx) = nullptr;
  int64_t (*spawn)(void* ctx, const char* name, int64_t name_len,
                   uint64_t arg) = nullptr;
  int64_t (*spawn_file)(void* ctx, const char* path, int64_t path_len,
                        const SyscallArgSlice* args, int64_t argc) = nullptr;
  StatusOr<UserProgramResult> (*wait)(void* ctx, int64_t pid) = nullptr;
  int64_t (*open)(void* ctx, const char* path, int64_t path_len,
                  uint64_t flags) = nullptr;
  int64_t (*close)(void* ctx, int64_t fd) = nullptr;
  int64_t (*file_read)(void* ctx, int64_t fd, void* buffer,
                       int64_t size) = nullptr;
  int64_t (*file_write)(void* ctx, int64_t fd, const void* buffer,
                        int64_t size) = nullptr;
  int64_t (*stat)(void* ctx, const char* path, int64_t path_len,
                  FsStat* out_stat) = nullptr;
  int64_t (*read_dir)(void* ctx, int64_t fd, FsDirEntry* entries,
                      int64_t max_entries) = nullptr;
  PathOp mkdir = nullptr;
  PathOp unlink = nullptr;
  int64_t (*rename)(void* ctx, const char* old_path, int64_t old_len,
                    const char* new_path, int64_t new_len) = nullptr;
};

inline void WriteStr(const ShellEnv& env, const char* const text) {
  const int len = StringLength(text);
  if (len > 0 && env.write != nullptr) {
    (void)env.write(env.ctx, text, len);
  }
}

inline void WriteBuf(const ShellEnv& env, const ShellOutBuffer& buf) {
  if (buf.length > 0 && env.write != nullptr) {
    (void)env.write(env.ctx, buf.data, buf.length);
  }
}

// Returns the description of filesystem error `rc` (a `kSyscallError*` value),
// or nullptr if it has none.
inline const char* FsErrorDescription(const int64_t rc) {
  switch (rc) {
    case kSyscallErrorNoEntry:
      return "no such file or directory";
    case kSyscallErrorIo:
      return "io error";
    case kSyscallErrorExecFormat:
      return "executable format error";
    case kSyscallErrorBadFd:
      return "bad file descriptor";
    case kSyscallErrorNoMemory:
      return "out of memory";
    case kSyscallErrorFault:
      return "bad address";
    case kSyscallErrorBusy:
      return "device or resource busy";
    case kSyscallErrorExists:
      return "file exists";
    case kSyscallErrorNoDevice:
      return "filesystem unavailable";
    case kSyscallErrorNotDir:
      return "not a directory";
    case kSyscallErrorIsDir:
      return "is a directory";
    case kSyscallErrorInvalid:
      return "invalid argument";
    case kSyscallErrorFileTableFull:
      return "too many open files in system";
    case kSyscallErrorTooManyFiles:
      return "too many open files";
    case kSyscallErrorFileTooBig:
      return "file too large";
    case kSyscallErrorNoSpace:
      return "no space left";
    case kSyscallErrorNameTooLong:
      return "name too long";
    case kSyscallErrorNotEmpty:
      return "directory not empty";
    case kSyscallErrorStale:
      return "stale file descriptor";
    default:
      return nullptr;
  }
}

// Appends the message for filesystem error `rc`: its description, or
// "error <rc>" (for example "error -13") if it has none.
inline void AppendFsErrorMessage(ShellOutBuffer& out, const int64_t rc) {
  const char* const description = FsErrorDescription(rc);
  if (description != nullptr) {
    out.AppendStr(description);
    return;
  }
  out.AppendStr("error ");
  out.AppendInt64Dec(rc);
}

// Writes "sh: <cmd_name>: <path>: <message>\n" for filesystem error `rc`.
inline void WriteFsError(const ShellEnv& env,         //
                         const char* const cmd_name,  //
                         const ShellToken& path,      //
                         const int64_t rc) {
  ShellOutBuffer err;
  err.AppendStr("sh: ");
  err.AppendStr(cmd_name);
  err.AppendStr(": ");
  err.AppendBytes(path.ptr, path.length);
  err.AppendStr(": ");
  AppendFsErrorMessage(err, rc);
  err.AppendChar('\n');
  WriteBuf(env, err);
}

// Writes "sh: <path>: <message>\n" for filesystem error `rc`.
inline void WritePathError(const ShellEnv& env,     //
                           const ShellToken& path,  //
                           const int64_t rc) {
  ShellOutBuffer err;
  err.AppendStr("sh: ");
  err.AppendBytes(path.ptr, path.length);
  err.AppendStr(": ");
  AppendFsErrorMessage(err, rc);
  err.AppendChar('\n');
  WriteBuf(env, err);
}

// Returns `count` limited to `[0, max]`. The shell clamps the counts that the
// callbacks return to the bounds of its buffers because host tests plug fakes
// into `ShellEnv`.
inline int ClampCount(const int64_t count, const int64_t max) {
  return static_cast<int>(count < 0 ? 0 : (count < max ? count : max));
}

// Writes the entries of directory `path` for `ls`, one per line, with a '/'
// after the names of subdirectories.
inline void ListDirectory(const ShellEnv& env, const ShellToken& path) {
  const int64_t fd =
      env.open(env.ctx, path.ptr, path.length, kFsOpenRead | kFsOpenDirectory);
  if (fd < 0) {
    WriteFsError(env, "ls", path, fd);
    return;
  }
  for (;;) {
    FsDirEntry entries[kShellReadDirBatch]{};
    const int64_t rc = env.read_dir(env.ctx, fd, entries, kShellReadDirBatch);
    if (rc < 0) {
      WriteFsError(env, "ls", path, rc);
      break;
    }
    if (rc == 0) {
      break;
    }
    const int count = ClampCount(rc, kShellReadDirBatch);
    for (int i = 0; i < count; ++i) {
      ShellOutBuffer line;
      line.AppendBytes(entries[i].name,
                       ClampCount(entries[i].name_len, kFsMaxNameLength));
      if (entries[i].type == FsNodeType::kDirectory) {
        line.AppendChar('/');
      }
      line.AppendChar('\n');
      WriteBuf(env, line);
    }
  }
  (void)env.close(env.ctx, fd);
}

// `ls [path ...]`, or `ls /` without operands: writes the path of each operand
// that isn't a directory, then the entries of each one that is
// (ListDirectory()). With several operands, as in POSIX ls, each directory's
// entries follow a "<path>:" line, and a blank line separates the groups of
// lines: the paths, and each directory. Errors of Stat() go in operand order
// with the paths but make no group by themselves (no blank line follows
// them), and errors in listing a directory go in its group.
inline void RunLs(const ShellEnv& env, const ShellCommand& cmd) {
  const bool fs_available = env.stat != nullptr && env.open != nullptr &&
                            env.read_dir != nullptr && env.close != nullptr;
  const ShellToken root{"/", 1};
  const bool has_operands = cmd.token_count > 1;
  const ShellToken* const operands = has_operands ? &cmd.tokens[1] : &root;
  const int operand_count = has_operands ? cmd.token_count - 1 : 1;
  bool is_dir[kShellMaxTokens] = {};
  bool wrote_group = false;
  for (int i = 0; i < operand_count; ++i) {
    const ShellToken& path = operands[i];
    FsStat st{};
    const int64_t rc = fs_available
                           ? env.stat(env.ctx, path.ptr, path.length, &st)
                           : kSyscallErrorNoDevice;
    if (rc < 0) {
      WriteFsError(env, "ls", path, rc);
    } else if (st.type == FsNodeType::kDirectory) {
      is_dir[i] = true;
    } else {
      ShellOutBuffer line;
      line.AppendBytes(path.ptr, path.length);
      line.AppendChar('\n');
      WriteBuf(env, line);
      wrote_group = true;
    }
  }
  for (int i = 0; i < operand_count; ++i) {
    if (!is_dir[i]) {
      continue;
    }
    if (operand_count > 1) {
      ShellOutBuffer header;
      if (wrote_group) {
        header.AppendChar('\n');
      }
      header.AppendBytes(operands[i].ptr, operands[i].length);
      header.AppendStr(":\n");
      WriteBuf(env, header);
    }
    ListDirectory(env, operands[i]);
    wrote_group = true;
  }
}

// `cat`: writes the contents of file `path`.
inline void RunCatOne(const ShellEnv& env, const ShellToken& path) {
  if (env.open == nullptr || env.file_read == nullptr || env.close == nullptr) {
    WriteFsError(env, "cat", path, kSyscallErrorNoDevice);
    return;
  }
  const int64_t fd = env.open(env.ctx, path.ptr, path.length, kFsOpenRead);
  if (fd < 0) {
    WriteFsError(env, "cat", path, fd);
    return;
  }
  for (;;) {
    char chunk[kShellFileChunkSize];
    const int64_t rc = env.file_read(env.ctx, fd, chunk, kShellFileChunkSize);
    if (rc < 0) {
      WriteFsError(env, "cat", path, rc);
      break;
    }
    if (rc == 0) {
      break;
    }
    if (env.write != nullptr) {
      (void)env.write(env.ctx, chunk, ClampCount(rc, kShellFileChunkSize));
    }
  }
  (void)env.close(env.ctx, fd);
}

// `touch`: creates an empty file at `path` unless something exists there (the
// filesystem keeps no times to update).
inline void RunTouchOne(const ShellEnv& env, const ShellToken& path) {
  if (env.open == nullptr || env.close == nullptr) {
    WriteFsError(env, "touch", path, kSyscallErrorNoDevice);
    return;
  }
  const int64_t fd =
      env.open(env.ctx, path.ptr, path.length, kFsOpenWrite | kFsOpenCreate);
  if (fd >= 0) {
    (void)env.close(env.ctx, fd);
    return;
  }
  // Open() refuses to write to a directory, which exists all the same.
  if (fd == kSyscallErrorIsDir && env.stat != nullptr) {
    FsStat st{};
    if (env.stat(env.ctx, path.ptr, path.length, &st) == 0 &&
        st.type == FsNodeType::kDirectory) {
      return;
    }
  }
  WriteFsError(env, "touch", path, fd);
}

// `mkdir` and `rm`: calls `op` (MkDir() or Unlink()) for each operand of `cmd`
// and reports each failure.
inline void RunPathOp(const ShellEnv& env,         //
                      const ShellCommand& cmd,     //
                      const char* const cmd_name,  //
                      const ShellEnv::PathOp op) {
  for (int i = 1; i < cmd.token_count; ++i) {
    const ShellToken& path = cmd.tokens[i];
    const int64_t rc = (op != nullptr) ? op(env.ctx, path.ptr, path.length)
                                       : kSyscallErrorNoDevice;
    if (rc < 0) {
      WriteFsError(env, cmd_name, path, rc);
    }
  }
}

// Writes the `size` bytes at `data` to file `fd` with as many FileWrite()
// calls as it takes, and returns 0, or the error of the call that failed. A
// call that writes nothing fails with `kSyscallErrorIo` rather than loop.
inline int64_t WriteAll(const ShellEnv& env,     //
                        const int64_t fd,        //
                        const char* const data,  //
                        const int64_t size) {
  int64_t done = 0;
  while (done < size) {
    const int64_t rc = env.file_write(env.ctx, fd, data + done, size - done);
    if (rc < 0) {
      return rc;
    }
    if (rc == 0) {
      return kSyscallErrorIo;
    }
    done += ClampCount(rc, size - done);
  }
  return 0;
}

// `write` and `append`: writes the operands of `cmd` after the first one, the
// path, separated by spaces and followed by a newline as echo writes them, to
// the file at the path, which `flags` opens: in place of its contents
// (`write`) or after them (`append`). Creates the file if it doesn't exist.
inline void RunWriteText(const ShellEnv& env,         //
                         const ShellCommand& cmd,     //
                         const char* const cmd_name,  //
                         const uint64_t flags) {
  const ShellToken& path = cmd.tokens[1];
  if (env.open == nullptr || env.file_write == nullptr ||
      env.close == nullptr) {
    WriteFsError(env, cmd_name, path, kSyscallErrorNoDevice);
    return;
  }
  ShellOutBuffer text;
  for (int i = 2; i < cmd.token_count; ++i) {
    if (i > 2) {
      text.AppendChar(' ');
    }
    text.AppendBytes(cmd.tokens[i].ptr, cmd.tokens[i].length);
  }
  text.AppendChar('\n');
  const int64_t fd = env.open(env.ctx, path.ptr, path.length, flags);
  if (fd < 0) {
    WriteFsError(env, cmd_name, path, fd);
    return;
  }
  const int64_t rc = WriteAll(env, fd, text.data, text.length);
  if (rc < 0) {
    WriteFsError(env, cmd_name, path, rc);
  }
  (void)env.close(env.ctx, fd);
}

// A path that the shell builds.
struct ShellPathBuffer {
  char data[kFsMaxPathLength] = {};
  int length = 0;

  ShellToken token() const { return ShellToken{data, length}; }
};

// Returns the last component of `path`, without the '/' characters after it:
// empty if `path` has no other characters.
inline ShellToken LastComponent(const ShellToken& path) {
  int end = path.length;
  while (end > 0 && path.ptr[end - 1] == '/') {
    --end;
  }
  int start = end;
  while (start > 0 && path.ptr[start - 1] != '/') {
    --start;
  }
  return ShellToken{path.ptr + start, end - start};
}

// Writes the path of `name` in directory `dir` to `*out`: `dir`, a '/' unless
// `dir` ends with one, and `name`. Returns false if it would be longer than
// `kFsMaxPathLength` bytes, the most that a filesystem call accepts.
inline bool JoinPath(const ShellToken& dir,   //
                     const ShellToken& name,  //
                     ShellPathBuffer* const out) {
  const bool dir_ends_with_slash =
      dir.length > 0 && dir.ptr[dir.length - 1] == '/';
  const int64_t length =
      int64_t{dir.length} + (dir_ends_with_slash ? 0 : 1) + name.length;
  if (length > kFsMaxPathLength) {
    return false;
  }
  int pos = 0;
  for (int i = 0; i < dir.length; ++i) {
    out->data[pos++] = dir.ptr[i];
  }
  if (!dir_ends_with_slash) {
    out->data[pos++] = '/';
  }
  for (int i = 0; i < name.length; ++i) {
    out->data[pos++] = name.ptr[i];
  }
  out->length = pos;
  return true;
}

// `cp` of one file: copies the contents of regular file `src` to `dst`,
// creating `dst` or replacing its contents.
inline void CopyFile(const ShellEnv& env,    //
                     const ShellToken& src,  //
                     const ShellToken& dst) {
  FsStat src_stat{};
  const int64_t src_rc = env.stat(env.ctx, src.ptr, src.length, &src_stat);
  if (src_rc < 0 || src_stat.type == FsNodeType::kDirectory) {
    // Like POSIX cp without -R, cp copies no directories.
    WriteFsError(env, "cp", src, src_rc < 0 ? src_rc : kSyscallErrorIsDir);
    return;
  }
  FsStat dst_stat{};
  if (env.stat(env.ctx, dst.ptr, dst.length, &dst_stat) == 0 &&
      dst_stat.fnode_id == src_stat.fnode_id) {
    // Opening `dst` would empty `src`.
    ShellOutBuffer err;
    err.AppendStr("sh: cp: ");
    err.AppendBytes(src.ptr, src.length);
    err.AppendStr(" and ");
    WriteBuf(env, err);
    ShellOutBuffer rest;
    rest.AppendBytes(dst.ptr, dst.length);
    rest.AppendStr(" are the same file\n");
    WriteBuf(env, rest);
    return;
  }
  const int64_t in = env.open(env.ctx, src.ptr, src.length, kFsOpenRead);
  if (in < 0) {
    WriteFsError(env, "cp", src, in);
    return;
  }
  const int64_t out = env.open(env.ctx,     //
                               dst.ptr,     //
                               dst.length,  //
                               kFsOpenWrite | kFsOpenCreate | kFsOpenTruncate);
  if (out < 0) {
    WriteFsError(env, "cp", dst, out);
  } else {
    for (;;) {
      char chunk[kShellFileChunkSize];
      const int64_t n = env.file_read(env.ctx, in, chunk, kShellFileChunkSize);
      if (n < 0) {
        WriteFsError(env, "cp", src, n);
        break;
      }
      if (n == 0) {
        break;
      }
      const int64_t rc =
          WriteAll(env, out, chunk, ClampCount(n, kShellFileChunkSize));
      if (rc < 0) {
        WriteFsError(env, "cp", dst, rc);
        break;
      }
    }
    (void)env.close(env.ctx, out);
  }
  (void)env.close(env.ctx, in);
}

// Returns whether path component `name` is `.` or `..`.
inline bool IsDotOrDotDot(const ShellToken& name) {
  return TokenEquals(name, ".") || TokenEquals(name, "..");
}

// `mv` of one file or directory: renames `src` to `dst` (Rename()), replacing
// a file or empty directory there.
inline void MoveFile(const ShellEnv& env,    //
                     const ShellToken& src,  //
                     const ShellToken& dst) {
  FsStat st{};
  const int64_t src_rc = env.stat(env.ctx, src.ptr, src.length, &st);
  if (src_rc < 0) {
    WriteFsError(env, "mv", src, src_rc);
    return;
  }
  const int64_t rc =
      env.rename(env.ctx, src.ptr, src.length, dst.ptr, dst.length);
  if (rc < 0) {
    // `src` exists, so an error is about `dst`, except two that Rename()
    // returns for either path. `kSyscallErrorBusy` means that `src` is the
    // root: RunTransfer() moves a source into the root, not onto it.
    // `kSyscallErrorInvalid` means that a last component is `.` or `..`, or
    // that directory `src` would move into itself or a subdirectory: it is
    // about `dst` only if the last component of `dst` is `.` or `..` and that
    // of `src` isn't.
    const bool about_src =
        rc == kSyscallErrorBusy ||
        (rc == kSyscallErrorInvalid && (IsDotOrDotDot(LastComponent(src)) ||
                                        !IsDotOrDotDot(LastComponent(dst))));
    WriteFsError(env, "mv", about_src ? src : dst, rc);
  }
}

// `CopyFile()` or `MoveFile()`.
using ShellTransfer = void (*)(const ShellEnv& env,    //
                               const ShellToken& src,  //
                               const ShellToken& dst);

// `cp` and `mv`: like POSIX cp and mv, copies or moves (`transfer`) each
// operand of `cmd` but the last, a source, to the last operand, `target`: into
// `target` if it is a directory, as `<target>/<last component of the
// source>`, else to `target` itself, which a single source then replaces.
// Reports the filesystem unavailable for each source unless `fs_available`.
inline void RunTransfer(const ShellEnv& env,         //
                        const ShellCommand& cmd,     //
                        const char* const cmd_name,  //
                        const bool fs_available,     //
                        const ShellTransfer transfer) {
  const int source_count = cmd.token_count - 2;
  const ShellToken& target = cmd.tokens[cmd.token_count - 1];
  if (!fs_available) {
    for (int i = 1; i <= source_count; ++i) {
      WriteFsError(env, cmd_name, cmd.tokens[i], kSyscallErrorNoDevice);
    }
    return;
  }
  FsStat target_stat{};
  const int64_t target_rc =
      env.stat(env.ctx, target.ptr, target.length, &target_stat);
  const bool into_dir =
      target_rc == 0 && target_stat.type == FsNodeType::kDirectory;
  if (source_count > 1 && !into_dir) {
    // Several sources need a directory to go to.
    WriteFsError(env,       //
                 cmd_name,  //
                 target,    //
                 target_rc < 0 ? target_rc : kSyscallErrorNotDir);
    return;
  }
  for (int i = 1; i <= source_count; ++i) {
    const ShellToken& src = cmd.tokens[i];
    ShellPathBuffer joined;
    if (into_dir && !JoinPath(target, LastComponent(src), &joined)) {
      // No filesystem call would take the destination's path.
      WriteFsError(env, cmd_name, src, kSyscallErrorNameTooLong);
      continue;
    }
    transfer(env, src, into_dir ? joined.token() : target);
  }
}

inline void RunCp(const ShellEnv& env, const ShellCommand& cmd) {
  const bool fs_available = env.stat != nullptr && env.open != nullptr &&
                            env.file_read != nullptr &&
                            env.file_write != nullptr && env.close != nullptr;
  RunTransfer(env, cmd, "cp", fs_available, CopyFile);
}

inline void RunMv(const ShellEnv& env, const ShellCommand& cmd) {
  const bool fs_available = env.stat != nullptr && env.rename != nullptr;
  RunTransfer(env, cmd, "mv", fs_available, MoveFile);
}

// Returns the name of node type `type` for `stat`.
inline const char* FsNodeTypeName(const FsNodeType type) {
  switch (type) {
    case FsNodeType::kRegular:
      return "regular file";
    case FsNodeType::kDirectory:
      return "directory";
    case FsNodeType::kNone:
      break;
  }
  return "unknown";
}

// `stat`: writes the type, size, fnode ID and number of links of the file or
// directory at `path`, one per line with the labels of GNU stat (`Fnode` in
// place of `Inode`).
inline void RunStatOne(const ShellEnv& env, const ShellToken& path) {
  FsStat st{};
  const int64_t rc = (env.stat != nullptr)
                         ? env.stat(env.ctx, path.ptr, path.length, &st)
                         : kSyscallErrorNoDevice;
  if (rc < 0) {
    WriteFsError(env, "stat", path, rc);
    return;
  }
  ShellOutBuffer file_line;
  file_line.AppendStr("  File: ");
  file_line.AppendBytes(path.ptr, path.length);
  file_line.AppendChar('\n');
  WriteBuf(env, file_line);
  ShellOutBuffer out;
  out.AppendStr("  Type: ");
  out.AppendStr(FsNodeTypeName(st.type));
  out.AppendStr("\n  Size: ");
  out.AppendInt64Dec(st.size);
  out.AppendStr("\n Fnode: ");
  out.AppendUint64Dec(st.fnode_id);
  out.AppendStr("\n Links: ");
  out.AppendInt64Dec(st.nlink);
  out.AppendChar('\n');
  WriteBuf(env, out);
}

// Returns true if filesystem built-in `cmd` may run. Otherwise writes why not
// and returns false: the line had more than `kShellMaxTokens` tokens, so the
// built-in would skip operands, or `cmd` has fewer than `min_operands`
// operands, for which it writes `usage`.
inline bool CheckFsOperands(const ShellEnv& env,      //
                            const ShellCommand& cmd,  //
                            const int min_operands,   //
                            const char* const usage) {
  if (cmd.truncated) {
    WriteStr(env, "sh: too many arguments\n");
    return false;
  }
  if (cmd.token_count - 1 < min_operands) {
    WriteStr(env, usage);
    return false;
  }
  return true;
}

// Executes a single parsed command line. Returns the exit status if the shell
// should exit, or `kStatusError` to continue the REPL loop.
inline StatusOr<int64_t> ExecuteLine(const ShellEnv& env,     //
                                     const char* const line,  //
                                     const int line_length) {
  const ShellCommand cmd = TokenizeLine(line, line_length);
  if (cmd.token_count == 0) {
    return kStatusError;
  }
  const ShellToken& first = cmd.tokens[0];

  if (TokenEquals(first, "help")) {
    // Each line fits the 80 columns of the VGA text console.
    WriteStr(env,
             "built-in commands: help echo cpu programs run exit\n"
             "built-in file commands: ls cat stat mkdir rm touch write append "
             "cp mv\n"
             "user programs: init sh usertest, or files in /bin or by path\n");
    return kStatusError;
  }

  if (TokenEquals(first, "echo")) {
    // Unlike the filesystem built-ins, echo ignores `cmd.truncated`: it writes
    // the first `kShellMaxTokens - 1` arguments of a longer line (the boot
    // test's model of the shell expects that: ECHO_MAX_TOKENS in
    // test_boot.sh).
    ShellOutBuffer out;
    for (int i = 1; i < cmd.token_count; ++i) {
      if (i > 1) {
        out.AppendChar(' ');
      }
      out.AppendBytes(cmd.tokens[i].ptr, cmd.tokens[i].length);
    }
    out.AppendChar('\n');
    WriteBuf(env, out);
    return kStatusError;
  }

  if (TokenEquals(first, "cpu")) {
    const int cpu = (env.get_cpu != nullptr) ? env.get_cpu(env.ctx) : 0;
    ShellOutBuffer out;
    out.AppendStr("cpu: ");
    out.AppendInt64Dec(cpu);
    out.AppendChar('\n');
    WriteBuf(env, out);
    return kStatusError;
  }

  if (TokenEquals(first, "ls")) {
    if (CheckFsOperands(env, cmd, 0, nullptr)) {
      RunLs(env, cmd);
    }
    return kStatusError;
  }

  if (TokenEquals(first, "cat")) {
    if (CheckFsOperands(env, cmd, 1, "sh: usage: cat <path> ...\n")) {
      for (int i = 1; i < cmd.token_count; ++i) {
        RunCatOne(env, cmd.tokens[i]);
      }
    }
    return kStatusError;
  }

  if (TokenEquals(first, "stat")) {
    if (CheckFsOperands(env, cmd, 1, "sh: usage: stat <path> ...\n")) {
      for (int i = 1; i < cmd.token_count; ++i) {
        RunStatOne(env, cmd.tokens[i]);
      }
    }
    return kStatusError;
  }

  if (TokenEquals(first, "mkdir")) {
    if (CheckFsOperands(env, cmd, 1, "sh: usage: mkdir <path> ...\n")) {
      RunPathOp(env, cmd, "mkdir", env.mkdir);
    }
    return kStatusError;
  }

  // Unlike POSIX rm, which needs -d or -r for a directory, rm also removes
  // empty directories: Unlink() removes both files and empty directories.
  if (TokenEquals(first, "rm")) {
    if (CheckFsOperands(env, cmd, 1, "sh: usage: rm <path> ...\n")) {
      RunPathOp(env, cmd, "rm", env.unlink);
    }
    return kStatusError;
  }

  if (TokenEquals(first, "touch")) {
    if (CheckFsOperands(env, cmd, 1, "sh: usage: touch <path> ...\n")) {
      for (int i = 1; i < cmd.token_count; ++i) {
        RunTouchOne(env, cmd.tokens[i]);
      }
    }
    return kStatusError;
  }

  // write and append take the place of echo's `>` and `>>`, which the shell
  // doesn't have.
  if (TokenEquals(first, "write")) {
    if (CheckFsOperands(env, cmd, 1, "sh: usage: write <path> [word ...]\n")) {
      RunWriteText(env,      //
                   cmd,      //
                   "write",  //
                   kFsOpenWrite | kFsOpenCreate | kFsOpenTruncate);
    }
    return kStatusError;
  }

  if (TokenEquals(first, "append")) {
    if (CheckFsOperands(env, cmd, 1, "sh: usage: append <path> [word ...]\n")) {
      RunWriteText(env,       //
                   cmd,       //
                   "append",  //
                   kFsOpenWrite | kFsOpenCreate | kFsOpenAppend);
    }
    return kStatusError;
  }

  if (TokenEquals(first, "cp")) {
    if (CheckFsOperands(env, cmd, 2, "sh: usage: cp <source> ... <target>\n")) {
      RunCp(env, cmd);
    }
    return kStatusError;
  }

  if (TokenEquals(first, "mv")) {
    if (CheckFsOperands(env, cmd, 2, "sh: usage: mv <source> ... <target>\n")) {
      RunMv(env, cmd);
    }
    return kStatusError;
  }

  if (TokenEquals(first, "programs")) {
    WriteStr(env, "programs: init sh usertest\n");
    if (env.open != nullptr && env.read_dir != nullptr &&
        env.close != nullptr) {
      const int64_t fd =
          env.open(env.ctx, "/bin", 4, kFsOpenRead | kFsOpenDirectory);
      if (fd >= 0) {
        ShellOutBuffer bin_line;
        bin_line.AppendStr("/bin:");
        bool any = false;
        for (;;) {
          FsDirEntry entries[kShellReadDirBatch]{};
          const int64_t rc =
              env.read_dir(env.ctx, fd, entries, kShellReadDirBatch);
          if (rc <= 0) {
            break;
          }
          const int count = ClampCount(rc, kShellReadDirBatch);
          for (int i = 0; i < count; ++i) {
            if (entries[i].type == FsNodeType::kRegular) {
              const int name_len =
                  ClampCount(entries[i].name_len, kFsMaxNameLength);
              // Leave room for `' '` + `name_len` + the closing `'\n'`.
              if (bin_line.length + 1 + name_len + 1 >=
                  kShellOutputBufferSize) {
                WriteBuf(env, bin_line);
                bin_line.Clear();
              }
              bin_line.AppendChar(' ');
              bin_line.AppendBytes(entries[i].name, name_len);
              any = true;
            }
          }
        }
        (void)env.close(env.ctx, fd);
        if (any) {
          bin_line.AppendChar('\n');
          WriteBuf(env, bin_line);
        }
      }
    }
    return kStatusError;
  }

  if (TokenEquals(first, "exit")) {
    if (cmd.token_count == 1) {
      return int64_t{0};
    }
    if (cmd.token_count == 2) {
      const StatusOr<int64_t> parsed_status = ParseInt64(cmd.tokens[1]);
      if (parsed_status.ok()) {
        return *parsed_status;
      }
    }
    WriteStr(env, "sh: invalid exit status\n");
    return kStatusError;
  }

  const auto has_slash = [](const ShellToken& token) {
    for (int i = 0; i < token.length; ++i) {
      if (token.ptr[i] == '/') {
        return true;
      }
    }
    return false;
  };
  const auto parse_optional_arg = [&](const int arg_index,
                                      uint64_t* const out_arg) {
    if (cmd.truncated || cmd.token_count > arg_index + 1) {
      WriteStr(env, "sh: too many arguments\n");
      return false;
    }
    if (cmd.token_count == arg_index + 1) {
      const StatusOr<uint64_t> parsed = ParseProgramArg(cmd.tokens[arg_index]);
      if (!parsed.ok()) {
        ShellOutBuffer err;
        err.AppendStr("sh: invalid argument: ");
        err.AppendBytes(cmd.tokens[arg_index].ptr,
                        cmd.tokens[arg_index].length);
        err.AppendChar('\n');
        WriteBuf(env, err);
        return false;
      }
      *out_arg = *parsed;
    } else {
      *out_arg = 0;
    }
    return true;
  };
  const auto make_bin_path = [](const ShellToken& name,
                                char (&buf)[5 + kFsMaxNameLength]) {
    buf[0] = '/';
    buf[1] = 'b';
    buf[2] = 'i';
    buf[3] = 'n';
    buf[4] = '/';
    for (int i = 0; i < name.length; ++i) {
      buf[5 + i] = name.ptr[i];
    }
    return ShellToken{buf, 5 + name.length};
  };
  const auto make_arg_slices = [](const ShellToken* const tokens,
                                  const int count,
                                  SyscallArgSlice(&out)[kShellMaxTokens]) {
    for (int i = 0; i < count; ++i) {
      out[i] = SyscallArgSlice{
          .ptr = reinterpret_cast<uintptr_t>(tokens[i].ptr),
          .len = tokens[i].length,
      };
    }
  };
  const auto is_embedded_program = [](const ShellToken& token) {
    return IsKnownUserProgram(token) || TokenEquals(token, "usbkbd") ||
           TokenEquals(token, "ps2kbd") || TokenEquals(token, "ramfs") ||
           TokenEquals(token, "nvme");
  };
  const auto is_missing_bin_error = [](const int64_t rc) {
    return rc == kSyscallErrorNoEntry || rc == kSyscallErrorNoDevice ||
           rc == kSyscallErrorNotDir;
  };
  const auto wait_and_report = [&](const ShellToken& display_name,
                                   const int64_t pid) {
    const StatusOr<UserProgramResult> result = env.wait(env.ctx, pid);
    if (!result.ok()) {
      WriteStr(env, "sh: wait error\n");
      return;
    }
    ShellOutBuffer report;
    report.AppendStr("sh: ");
    report.AppendBytes(display_name.ptr, display_name.length);
    switch (result->outcome) {
      case UserProgramOutcome::kExited:
        report.AppendStr(" exited with status ");
        report.AppendInt64Dec(result->exit_status);
        break;
      case UserProgramOutcome::kFaulted:
        report.AppendStr(" faulted vector=");
        report.AppendInt64Dec(result->fault_vector);
        report.AppendStr(" error=");
        report.AppendHex64(result->fault_error_code);
        break;
      case UserProgramOutcome::kKilled:
        report.AppendStr(" killed");
        break;
      case UserProgramOutcome::kLoadFailed:
        report.AppendStr(" load error");
        break;
    }
    report.AppendChar('\n');
    WriteBuf(env, report);
  };

  if (TokenEquals(first, "run")) {
    if (cmd.token_count < 2) {
      WriteStr(env, "sh: usage: run <program> [arg ...]\n");
      return kStatusError;
    }
    const ShellToken& prog_token = cmd.tokens[1];
    if (has_slash(prog_token)) {
      if (cmd.truncated) {
        WriteStr(env, "sh: too many arguments\n");
        return kStatusError;
      }
      if (env.spawn_file == nullptr) {
        WriteFsError(env, "run", prog_token, kSyscallErrorNoDevice);
        return kStatusError;
      }
      if (env.wait == nullptr) {
        WriteStr(env, "sh: spawn unavailable\n");
        return kStatusError;
      }
      SyscallArgSlice args[kShellMaxTokens]{};
      const int argc = cmd.token_count - 1;
      make_arg_slices(&cmd.tokens[1], argc, args);
      const int64_t pid = env.spawn_file(env.ctx, prog_token.ptr,
                                         prog_token.length, args, argc);
      if (pid < 0) {
        WriteFsError(env, "run", prog_token, pid);
        return kStatusError;
      }
      wait_and_report(prog_token, pid);
      return kStatusError;
    }

    if ((env.spawn == nullptr && env.spawn_file == nullptr) ||
        env.wait == nullptr) {
      WriteStr(env, "sh: spawn unavailable\n");
      return kStatusError;
    }
    if (is_embedded_program(prog_token) || env.spawn_file == nullptr) {
      uint64_t prog_arg = 0;
      if (!parse_optional_arg(2, &prog_arg)) {
        return kStatusError;
      }
      const int64_t pid =
          (env.spawn != nullptr)
              ? env.spawn(env.ctx, prog_token.ptr, prog_token.length, prog_arg)
              : kSyscallErrorInvalid;
      if (pid < 0) {
        ShellOutBuffer err;
        err.AppendStr("sh: unknown program: ");
        err.AppendBytes(prog_token.ptr, prog_token.length);
        err.AppendChar('\n');
        WriteBuf(env, err);
        return kStatusError;
      }
      wait_and_report(prog_token, pid);
      return kStatusError;
    }

    int64_t pid = kSyscallErrorInvalid;
    if (env.spawn != nullptr && !cmd.truncated && cmd.token_count <= 3) {
      StatusOr<uint64_t> parsed = uint64_t{0};
      if (cmd.token_count == 3) {
        parsed = ParseProgramArg(cmd.tokens[2]);
      }
      if (parsed.ok()) {
        pid = env.spawn(env.ctx, prog_token.ptr, prog_token.length, *parsed);
      }
    }
    if (pid < 0 && prog_token.length <= kFsMaxNameLength) {
      char bin_buf[5 + kFsMaxNameLength];
      const ShellToken bin_token = make_bin_path(prog_token, bin_buf);
      if (cmd.truncated) {
        if (env.stat != nullptr) {
          FsStat st{};
          if (env.stat(env.ctx, bin_token.ptr, bin_token.length, &st) == 0) {
            WriteStr(env, "sh: too many arguments\n");
            return kStatusError;
          }
        }
      } else {
        SyscallArgSlice args[kShellMaxTokens]{};
        const int argc = cmd.token_count - 1;
        make_arg_slices(&cmd.tokens[1], argc, args);
        pid = env.spawn_file(env.ctx, bin_token.ptr, bin_token.length, args,
                             argc);
        if (pid < 0 && !is_missing_bin_error(pid)) {
          WriteFsError(env, "run", bin_token, pid);
          return kStatusError;
        }
      }
    }
    if (pid < 0) {
      ShellOutBuffer err;
      err.AppendStr("sh: unknown program: ");
      err.AppendBytes(prog_token.ptr, prog_token.length);
      err.AppendChar('\n');
      WriteBuf(env, err);
      return kStatusError;
    }
    wait_and_report(prog_token, pid);
    return kStatusError;
  }

  if (has_slash(first)) {
    if (cmd.truncated) {
      WriteStr(env, "sh: too many arguments\n");
      return kStatusError;
    }
    if (env.spawn_file == nullptr) {
      WritePathError(env, first, kSyscallErrorNoDevice);
      return kStatusError;
    }
    if (env.wait == nullptr) {
      WriteStr(env, "sh: spawn unavailable\n");
      return kStatusError;
    }
    SyscallArgSlice args[kShellMaxTokens]{};
    make_arg_slices(&cmd.tokens[0], cmd.token_count, args);
    const int64_t pid =
        env.spawn_file(env.ctx, first.ptr, first.length, args, cmd.token_count);
    if (pid < 0) {
      WritePathError(env, first, pid);
      return kStatusError;
    }
    wait_and_report(first, pid);
    return kStatusError;
  }

  if (IsKnownUserProgram(first)) {
    uint64_t prog_arg = 0;
    if (!parse_optional_arg(1, &prog_arg)) {
      return kStatusError;
    }
    if (env.spawn == nullptr || env.wait == nullptr) {
      WriteStr(env, "sh: spawn unavailable\n");
      return kStatusError;
    }
    const int64_t pid = env.spawn(env.ctx, first.ptr, first.length, prog_arg);
    if (pid < 0) {
      ShellOutBuffer err;
      err.AppendStr("sh: unknown program: ");
      err.AppendBytes(first.ptr, first.length);
      err.AppendChar('\n');
      WriteBuf(env, err);
      return kStatusError;
    }
    wait_and_report(first, pid);
    return kStatusError;
  }

  if (env.spawn_file != nullptr && env.wait != nullptr &&
      first.length <= kFsMaxNameLength) {
    char bin_buf[5 + kFsMaxNameLength];
    const ShellToken bin_token = make_bin_path(first, bin_buf);
    if (!cmd.truncated) {
      SyscallArgSlice args[kShellMaxTokens]{};
      make_arg_slices(&cmd.tokens[0], cmd.token_count, args);
      const int64_t pid = env.spawn_file(
          env.ctx, bin_token.ptr, bin_token.length, args, cmd.token_count);
      if (pid > 0) {
        wait_and_report(first, pid);
        return kStatusError;
      }
      if (!is_missing_bin_error(pid)) {
        WritePathError(env, bin_token, pid);
        return kStatusError;
      }
    } else if (env.stat != nullptr) {
      FsStat st{};
      if (env.stat(env.ctx, bin_token.ptr, bin_token.length, &st) == 0) {
        WriteStr(env, "sh: too many arguments\n");
        return kStatusError;
      }
    }
  }

  ShellOutBuffer err;
  err.AppendStr("sh: unknown command: ");
  err.AppendBytes(first.ptr, first.length);
  err.AppendChar('\n');
  WriteBuf(env, err);
  return kStatusError;
}

// Runs the line in `editor` (ExecuteLine()), unless it lost characters that
// didn't fit (`ShellLineEditor::overflowed`), which the shell reports instead.
// Returns the exit status if the shell should exit, else `kStatusError`.
inline StatusOr<int64_t> SubmitLine(const ShellEnv& env,
                                    const ShellLineEditor& editor) {
  if (editor.overflowed) {
    WriteStr(env, "sh: line too long\n");
    return kStatusError;
  }
  return ExecuteLine(env, editor.buffer, editor.length);
}

// Runs the interactive Read-Eval-Print Loop until `exit` is executed or `read`
// returns <= 0 (EOF).
inline int64_t RunShell(const ShellEnv& env) {
  ShellLineEditor editor;
  WriteStr(env, kShellPrompt);

  for (;;) {
    char ch = 0;
    const int64_t n = (env.read != nullptr) ? env.read(env.ctx, &ch, 1) : 0;
    if (n <= 0) {
      if (editor.length > 0 || editor.overflowed) {
        WriteStr(env, "\n");
        const StatusOr<int64_t> exit_status = SubmitLine(env, editor);
        if (exit_status.ok()) {
          return *exit_status;
        }
      }
      return 0;
    }

    const ShellCharAction action = editor.FeedChar(&ch);
    switch (action) {
      case ShellCharAction::kNone:
      case ShellCharAction::kIgnoreLfAfterCr:
        break;
      case ShellCharAction::kEchoChar:
        if (env.write != nullptr) {
          (void)env.write(env.ctx, &ch, 1);
        }
        break;
      case ShellCharAction::kBackspaceErase:
        WriteStr(env, "\b \b");
        break;
      case ShellCharAction::kSubmitLine: {
        WriteStr(env, "\n");
        const StatusOr<int64_t> exit_status = SubmitLine(env, editor);
        editor.ResetLine();
        if (exit_status.ok()) {
          return *exit_status;
        }
        WriteStr(env, kShellPrompt);
        break;
      }
    }
  }
}

}  // namespace protos::sh

#endif  // PROTOS_USER_SH_SH_LIB_H_
