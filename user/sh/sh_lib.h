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

// System call callbacks used by the shell.
struct ShellEnv {
  void* ctx = nullptr;
  int64_t (*read)(void* ctx, char* buffer, int64_t size) = nullptr;
  int64_t (*write)(void* ctx, const char* buffer, int64_t size) = nullptr;
  int (*get_cpu)(void* ctx) = nullptr;
  int64_t (*spawn)(void* ctx, const char* name, int64_t name_len,
                   uint64_t arg) = nullptr;
  StatusOr<UserProgramResult> (*wait)(void* ctx, int64_t pid) = nullptr;
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
             "built-in commands: help echo cpu programs run exit\n"
             "user programs: init sh usertest\n");
    return kStatusError;
  }

  if (TokenEquals(first, "echo")) {
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
