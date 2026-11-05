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

constexpr int kShellMaxLineLength = 128;
constexpr int kShellMaxTokens = 16;
constexpr int kShellOutputBufferSize = 192;
constexpr const char* kShellPrompt = "sh> ";
// The most directory entries that `ls` reads with one ReadDir() call and the
// most bytes that `cat` reads with one FileRead() call. Their buffers are on
// the stack, which has 16 KiB in a user program (`kUserStackPages` in
// user_mode.h), so they are smaller than the system calls allow.
constexpr int kShellReadDirBatch = 8;
constexpr int kShellCatChunkSize = 2048;
static_assert(kShellReadDirBatch <= kSyscallFsMaxDirEntries);
static_assert(kShellCatChunkSize <= kSyscallFsMaxIoSize);

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
  bool last_was_cr = false;

  void ResetLine() {
    length = 0;
    buffer[0] = '\0';
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
// callback that it needs (`open` through `unlink` may be null) reports the
// filesystem unavailable instead of calling the others.
struct ShellEnv {
  // MkDir() or Unlink() of the path of `path_len` bytes at `path`.
  using PathOp = int64_t (*)(void* ctx, const char* path, int64_t path_len);

  void* ctx = nullptr;
  int64_t (*read)(void* ctx, char* buffer, int64_t size) = nullptr;
  int64_t (*write)(void* ctx, const char* buffer, int64_t size) = nullptr;
  int (*get_cpu)(void* ctx) = nullptr;
  int64_t (*spawn)(void* ctx, const char* name, int64_t name_len,
                   uint64_t arg) = nullptr;
  StatusOr<UserProgramResult> (*wait)(void* ctx, int64_t pid) = nullptr;
  int64_t (*open)(void* ctx, const char* path, int64_t path_len,
                  uint64_t flags) = nullptr;
  int64_t (*close)(void* ctx, int64_t fd) = nullptr;
  int64_t (*file_read)(void* ctx, int64_t fd, void* buffer,
                       int64_t size) = nullptr;
  int64_t (*stat)(void* ctx, const char* path, int64_t path_len,
                  FsStat* out_stat) = nullptr;
  int64_t (*read_dir)(void* ctx, int64_t fd, FsDirEntry* entries,
                      int64_t max_entries) = nullptr;
  PathOp mkdir = nullptr;
  PathOp unlink = nullptr;
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
// lines: the paths, and each directory. Errors, written in operand order with
// the paths, belong to no group.
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
    char chunk[kShellCatChunkSize];
    const int64_t rc = env.file_read(env.ctx, fd, chunk, kShellCatChunkSize);
    if (rc < 0) {
      WriteFsError(env, "cat", path, rc);
      break;
    }
    if (rc == 0) {
      break;
    }
    if (env.write != nullptr) {
      (void)env.write(env.ctx, chunk, ClampCount(rc, kShellCatChunkSize));
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

// Returns true if filesystem built-in `cmd` may run. Otherwise writes why not
// and returns false: the line had more than `kShellMaxTokens` tokens, so the
// built-in would skip operands, or `cmd` has no operands and `usage`, the
// usage message of a built-in that needs some, isn't null.
inline bool CheckFsOperands(const ShellEnv& env,      //
                            const ShellCommand& cmd,  //
                            const char* const usage) {
  if (cmd.truncated) {
    WriteStr(env, "sh: too many arguments\n");
    return false;
  }
  if (cmd.token_count < 2 && usage != nullptr) {
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
    WriteStr(env,
             "built-in commands: help echo cpu ls cat mkdir rm touch programs "
             "run exit\n"
             "user programs: init sh usertest\n");
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
    if (CheckFsOperands(env, cmd, nullptr)) {
      RunLs(env, cmd);
    }
    return kStatusError;
  }

  if (TokenEquals(first, "cat")) {
    if (CheckFsOperands(env, cmd, "sh: usage: cat <path> ...\n")) {
      for (int i = 1; i < cmd.token_count; ++i) {
        RunCatOne(env, cmd.tokens[i]);
      }
    }
    return kStatusError;
  }

  if (TokenEquals(first, "mkdir")) {
    if (CheckFsOperands(env, cmd, "sh: usage: mkdir <path> ...\n")) {
      RunPathOp(env, cmd, "mkdir", env.mkdir);
    }
    return kStatusError;
  }

  // Unlike POSIX rm, which needs -d or -r for a directory, rm also removes
  // empty directories: Unlink() removes both files and empty directories.
  if (TokenEquals(first, "rm")) {
    if (CheckFsOperands(env, cmd, "sh: usage: rm <path> ...\n")) {
      RunPathOp(env, cmd, "rm", env.unlink);
    }
    return kStatusError;
  }

  if (TokenEquals(first, "touch")) {
    if (CheckFsOperands(env, cmd, "sh: usage: touch <path> ...\n")) {
      for (int i = 1; i < cmd.token_count; ++i) {
        RunTouchOne(env, cmd.tokens[i]);
      }
    }
    return kStatusError;
  }

  if (TokenEquals(first, "programs")) {
    WriteStr(env, "programs: init sh usertest\n");
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

  ShellToken prog_token;
  uint64_t prog_arg = 0;
  if (TokenEquals(first, "run")) {
    if (cmd.token_count < 2) {
      WriteStr(env, "sh: usage: run <program> [arg]\n");
      return kStatusError;
    }
    if (cmd.token_count > 3) {
      WriteStr(env, "sh: too many arguments\n");
      return kStatusError;
    }
    prog_token = cmd.tokens[1];
    if (cmd.token_count == 3) {
      const StatusOr<uint64_t> parsed_arg = ParseProgramArg(cmd.tokens[2]);
      if (!parsed_arg.ok()) {
        ShellOutBuffer err;
        err.AppendStr("sh: invalid argument: ");
        err.AppendBytes(cmd.tokens[2].ptr, cmd.tokens[2].length);
        err.AppendChar('\n');
        WriteBuf(env, err);
        return kStatusError;
      }
      prog_arg = *parsed_arg;
    }
  } else if (IsKnownUserProgram(first)) {
    if (cmd.token_count > 2) {
      WriteStr(env, "sh: too many arguments\n");
      return kStatusError;
    }
    prog_token = first;
    if (cmd.token_count == 2) {
      const StatusOr<uint64_t> parsed_arg = ParseProgramArg(cmd.tokens[1]);
      if (!parsed_arg.ok()) {
        ShellOutBuffer err;
        err.AppendStr("sh: invalid argument: ");
        err.AppendBytes(cmd.tokens[1].ptr, cmd.tokens[1].length);
        err.AppendChar('\n');
        WriteBuf(env, err);
        return kStatusError;
      }
      prog_arg = *parsed_arg;
    }
  } else {
    ShellOutBuffer err;
    err.AppendStr("sh: unknown command: ");
    err.AppendBytes(first.ptr, first.length);
    err.AppendChar('\n');
    WriteBuf(env, err);
    return kStatusError;
  }

  if (env.spawn == nullptr || env.wait == nullptr) {
    WriteStr(env, "sh: spawn unavailable\n");
    return kStatusError;
  }
  const int64_t pid =
      env.spawn(env.ctx, prog_token.ptr, prog_token.length, prog_arg);
  if (pid < 0) {
    ShellOutBuffer err;
    err.AppendStr("sh: unknown program: ");
    err.AppendBytes(prog_token.ptr, prog_token.length);
    err.AppendChar('\n');
    WriteBuf(env, err);
    return kStatusError;
  }

  const StatusOr<UserProgramResult> result = env.wait(env.ctx, pid);
  if (!result.ok()) {
    WriteStr(env, "sh: wait error\n");
    return kStatusError;
  }

  ShellOutBuffer report;
  report.AppendStr("sh: ");
  report.AppendBytes(prog_token.ptr, prog_token.length);
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
  return kStatusError;
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
      if (editor.length > 0) {
        WriteStr(env, "\n");
        const StatusOr<int64_t> exit_status =
            ExecuteLine(env, editor.buffer, editor.length);
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
        const StatusOr<int64_t> exit_status =
            ExecuteLine(env, editor.buffer, editor.length);
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
