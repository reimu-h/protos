#include "syscall_abi.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "user/sh_lib.h"
#include "user_programs.h"

namespace protos {
namespace {

namespace t = ::testing;

// User programs verify spin()'s result with the same function at compile time
// or at run time.
constexpr uint8_t kAbc[] = {'a', 'b', 'c'};
static_assert(SyscallSpinChecksum(kAbc, 0) == kSyscallSpinChecksumInit);
static_assert(SyscallSpinChecksum(kAbc + 1, 2, SyscallSpinChecksum(kAbc, 1)) ==
              SyscallSpinChecksum(kAbc, 3));

// Returns the bytes of `text`.
static const uint8_t* Bytes(const char* const text) {
  return reinterpret_cast<const uint8_t*>(text);
}

// The 64-bit FNV-1a hash of the `size` bytes at `data`, computed in one go,
// as a reference.
static uint64_t Fnv1a64(const uint8_t* const data, const int64_t size) {
  uint64_t hash = 0xCBF29CE484222325ULL;
  for (int64_t i = 0; i < size; ++i) {
    hash ^= data[i];
    hash *= 0x100000001B3ULL;
  }
  return hash;
}

constexpr uint64_t kBit63 = uint64_t{1} << 63;

TEST(SyscallAbiTest, SpinChecksumIsTheFnv1aHashWithBit63Cleared) {
  EXPECT_THAT(SyscallSpinChecksum(nullptr, 0), t::Eq(0x4BF29CE484222325));
  EXPECT_THAT(SyscallSpinChecksum(Bytes("a"), 1), t::Eq(0x2F63DC4C8601EC8C));
  EXPECT_THAT(SyscallSpinChecksum(Bytes("foobar"), 6),
              t::Eq(0x05944171F73967E8));
  uint8_t pattern[300];
  for (int i = 0; i < 300; ++i) {
    pattern[i] = static_cast<uint8_t>(i * 37 + 11);
  }
  EXPECT_THAT(static_cast<uint64_t>(SyscallSpinChecksum(pattern, 300)),
              t::Eq(Fnv1a64(pattern, 300) & ~kBit63));
}

TEST(SyscallAbiTest, SpinChecksumCanBeComputedPieceByPiece) {
  uint8_t pattern[64];
  for (int i = 0; i < 64; ++i) {
    pattern[i] = static_cast<uint8_t>(0xF0 ^ (i * 13));
  }
  const int64_t whole = SyscallSpinChecksum(pattern, 64);
  for (int split = 0; split <= 64; ++split) {
    const int64_t head = SyscallSpinChecksum(pattern, split);
    EXPECT_THAT(SyscallSpinChecksum(pattern + split, 64 - split, head),
                t::Eq(whole))
        << "split at " << split;
  }
}

TEST(SyscallAbiTest, SpinChecksumIsNeverNegativeSoNeverAnError) {
  static uint8_t buffer[kSyscallSpinMaxSize];
  for (const uint8_t fill : {uint8_t{0x00}, uint8_t{0xFF}}) {
    for (int64_t i = 0; i < kSyscallSpinMaxSize; ++i) {
      buffer[i] = fill;
    }
    EXPECT_THAT(SyscallSpinChecksum(buffer, kSyscallSpinMaxSize), t::Ge(0));
  }
  // Pseudo-random contents and sizes; about half of their FNV-1a hashes have
  // bit 63 set.
  uint64_t state = 0x9E3779B97F4A7C15ULL;
  int high_hashes = 0;
  for (int round = 0; round < 256; ++round) {
    for (int64_t i = 0; i < kSyscallSpinMaxSize; ++i) {
      state = state * 6364136223846793005ULL + 1442695040888963407ULL;
      buffer[i] = static_cast<uint8_t>(state >> 56);
    }
    const int64_t size =
        static_cast<int64_t>(state >> 40) % kSyscallSpinMaxSize;
    high_hashes += ((Fnv1a64(buffer, size) & kBit63) != 0) ? 1 : 0;
    EXPECT_THAT(SyscallSpinChecksum(buffer, size), t::Ge(0));
  }
  EXPECT_THAT(high_hashes, t::Gt(0));
}

TEST(SyscallAbiTest, NumbersAndLimitsStayTheSame) {
  EXPECT_THAT(kSyscallExit, t::Eq(0));
  EXPECT_THAT(kSyscallWrite, t::Eq(1));
  EXPECT_THAT(kSyscallGetCpu, t::Eq(2));
  EXPECT_THAT(kSyscallSpin, t::Eq(3));
  EXPECT_THAT(kSyscallRead, t::Eq(4));
  EXPECT_THAT(kSyscallSpawn, t::Eq(5));
  EXPECT_THAT(kSyscallWait, t::Eq(6));
  EXPECT_THAT(kSyscallSpawnDriver, t::Eq(7));
  EXPECT_THAT(kSyscallPciInfo, t::Eq(8));
  EXPECT_THAT(kSyscallPciClaim, t::Eq(9));
  EXPECT_THAT(kSyscallPciBusMaster, t::Eq(10));
  EXPECT_THAT(kSyscallMmioMap, t::Eq(11));
  EXPECT_THAT(kSyscallDmaAlloc, t::Eq(12));
  EXPECT_THAT(kSyscallIrqAlloc, t::Eq(13));
  EXPECT_THAT(kSyscallIrqWait, t::Eq(14));
  EXPECT_THAT(kSyscallConsolePush, t::Eq(15));
  EXPECT_THAT(kSyscallGetTicks, t::Eq(16));
  EXPECT_THAT(kSyscallWriteMaxSize, t::Eq(4096));
  EXPECT_THAT(kSyscallSpinMaxSize, t::Eq(4096));
  EXPECT_THAT(kSyscallSpinMaxTicks, t::Eq(1000000));
  EXPECT_THAT(kSyscallReadMaxSize, t::Eq(4096));
  EXPECT_THAT(kSyscallSpawnMaxNameLength, t::Eq(64));
  EXPECT_THAT(kMaxUserChildProcesses, t::Eq(8));
  EXPECT_THAT(kSyscallConsolePushMaxSize, t::Eq(256));
  EXPECT_THAT(kSyscallMmioMaxSize, t::Eq(16LL << 20));
  EXPECT_THAT(kSyscallDmaMaxSize, t::Eq(1LL << 20));
  EXPECT_THAT(kSyscallDmaMaxTotalSize, t::Eq(16LL << 20));
  EXPECT_THAT(kSyscallMaxClaims, t::Eq(16));
  EXPECT_THAT(kSyscallDmaBelow4GiB, t::Eq(1ULL));
  EXPECT_THAT(kSyscallDmaFlagsMask, t::Eq(kSyscallDmaBelow4GiB));
  EXPECT_THAT(kSyscallIrqModeAuto, t::Eq(0));
  EXPECT_THAT(kSyscallIrqModeMsi, t::Eq(1));
  EXPECT_THAT(kSyscallIrqModeMsix, t::Eq(2));
  EXPECT_THAT(kSyscallIrqMaxCount, t::Eq(32));
  EXPECT_THAT(kSyscallIrqWaitMaxTicks, t::Eq(1000000));
  EXPECT_THAT(kSyscallTicksPerSecond, t::Eq(250));
  EXPECT_THAT(kSyscallErrorPermission, t::Eq(-1));
  EXPECT_THAT(kSyscallErrorNoMemory, t::Eq(-12));
  EXPECT_THAT(kSyscallErrorAccess, t::Eq(-13));
  EXPECT_THAT(kSyscallErrorFault, t::Eq(-14));
  EXPECT_THAT(kSyscallErrorBusy, t::Eq(-16));
  EXPECT_THAT(kSyscallErrorNoDevice, t::Eq(-19));
  EXPECT_THAT(kSyscallErrorInvalid, t::Eq(-22));
  EXPECT_THAT(kSyscallErrorNoSys, t::Eq(-38));
  EXPECT_THAT(kUserCapNone, t::Eq(0ULL));
  EXPECT_THAT(kUserCapDriver, t::Eq(1ULL << 0));
  EXPECT_THAT(kUserCapSpawnDrivers, t::Eq(1ULL << 1));
  EXPECT_THAT(kInitArgStartDrivers, t::Eq(1ULL << 0));
  EXPECT_THAT(kDriverArgNormal, t::Eq(0ULL));
  EXPECT_THAT(kDriverArgExitAfterReady, t::Eq(1ULL));
  EXPECT_THAT(kDriverArgFaultAfterReady, t::Eq(2ULL));
}

TEST(SyscallAbiTest, DriverAbiLayoutsAndHelpers) {
  EXPECT_TRUE(SyscallIsError(-1));
  EXPECT_TRUE(SyscallIsError(-4095));
  EXPECT_TRUE(SyscallIsError(kSyscallErrorPermission));
  EXPECT_TRUE(SyscallIsError(kSyscallErrorNoSys));
  EXPECT_FALSE(SyscallIsError(0));
  EXPECT_FALSE(SyscallIsError(1));
  EXPECT_FALSE(SyscallIsError(-4096));
  // Higher-half user virtual addresses (MMIO / DMA windows) cast to negative
  // int64_t values outside [-4095, -1] and are not errors.
  EXPECT_FALSE(SyscallIsError(static_cast<int64_t>(0xFFFF900000000000ULL)));
  EXPECT_FALSE(SyscallIsError(static_cast<int64_t>(0xFFFFA00000000000ULL)));

  for (int64_t num = -1; num <= 20; ++num) {
    const bool expected = (num >= 8 && num <= 15);
    EXPECT_THAT(SyscallIsDriverOnly(num), t::Eq(expected)) << "syscall " << num;
  }

  for (int64_t num = -1; num <= 20; ++num) {
    const bool is_unpriv = (num >= 0 && num <= 6) || num == kSyscallGetTicks;
    const bool is_spawn_drv = (num == kSyscallSpawnDriver);
    const bool is_drv_only =
        (num >= kSyscallPciInfo && num <= kSyscallConsolePush);
    EXPECT_THAT(MayMakeSyscall(kUserCapNone, num),
                t::Eq(is_unpriv || (!is_spawn_drv && !is_drv_only)))
        << "syscall " << num;
    EXPECT_THAT(MayMakeSyscall(kUserCapDriver, num), t::Eq(!is_spawn_drv))
        << "syscall " << num;
    EXPECT_THAT(MayMakeSyscall(kUserCapSpawnDrivers, num), t::Eq(!is_drv_only))
        << "syscall " << num;
    EXPECT_TRUE(MayMakeSyscall(kUserCapDriver | kUserCapSpawnDrivers, num))
        << "syscall " << num;
  }

  const PciDeviceInfo info{};
  EXPECT_THAT(sizeof(info), t::Eq(176U));
  EXPECT_THAT(alignof(PciDeviceInfo), t::Eq(8U));
  EXPECT_THAT(sizeof(PciBarInfo), t::Eq(24U));
  EXPECT_THAT(alignof(PciBarInfo), t::Eq(8U));
  EXPECT_THAT(info.parent_index, t::Eq(-1));
  EXPECT_THAT(info.bars[0].type, t::Eq(PciBarInfoType::kNone));
}

TEST(SyscallAbiTest, UserProgramResultDefaultsAndOutcomes) {
  EXPECT_THAT(static_cast<uint8_t>(UserProgramOutcome::kExited), t::Eq(0));
  EXPECT_THAT(static_cast<uint8_t>(UserProgramOutcome::kFaulted), t::Eq(1));
  EXPECT_THAT(static_cast<uint8_t>(UserProgramOutcome::kKilled), t::Eq(2));
  EXPECT_THAT(static_cast<uint8_t>(UserProgramOutcome::kLoadFailed), t::Eq(3));

  const UserProgramResult result{};
  EXPECT_THAT(result.outcome, t::Eq(UserProgramOutcome::kLoadFailed));
  EXPECT_THAT(result.exit_status, t::Eq(0));
  EXPECT_THAT(result.fault_vector, t::Eq(-1));
  EXPECT_THAT(result.fault_error_code, t::Eq(0U));
  EXPECT_THAT(result.fault_rip, t::Eq(0U));
  EXPECT_THAT(result.fault_address, t::Eq(0U));
}

TEST(SyscallAbiTest, UserProgramNameEqualsMatchesExactLengthStrings) {
  EXPECT_TRUE(UserProgramNameEquals("sh", 2, "sh"));
  EXPECT_TRUE(UserProgramNameEquals("init", 4, "init"));
  EXPECT_TRUE(UserProgramNameEquals("usertest", 8, "usertest"));
  // Substring of longer buffer with exact length matches.
  EXPECT_TRUE(UserProgramNameEquals("sh_extra", 2, "sh"));
  // Mismatched lengths, embedded NULs, or null pointers do not match.
  EXPECT_FALSE(UserProgramNameEquals("sh", 1, "sh"));
  EXPECT_FALSE(UserProgramNameEquals("sh\0x", 4, "sh"));
  EXPECT_FALSE(UserProgramNameEquals("sh", 0, "sh"));
  EXPECT_FALSE(UserProgramNameEquals(nullptr, 2, "sh"));
  EXPECT_FALSE(UserProgramNameEquals("unknown", 7, "sh"));
}

struct FakeShellState {
  std::string input;
  size_t read_pos = 0;
  std::string output;
  int cpu_id = 3;
  struct SpawnCall {
    std::string name;
    uint64_t arg = 0;
  };
  std::vector<SpawnCall> spawns;

  static int64_t Read(void* const ctx, char* const buf, const int64_t size) {
    FakeShellState* const s = static_cast<FakeShellState*>(ctx);
    if (size <= 0 || s->read_pos >= s->input.size()) {
      return 0;
    }
    buf[0] = s->input[s->read_pos++];
    return 1;
  }

  static int64_t Write(void* const ctx,        //
                       const char* const buf,  //
                       const int64_t size) {
    FakeShellState* const s = static_cast<FakeShellState*>(ctx);
    if (buf != nullptr && size > 0) {
      s->output.append(buf, static_cast<size_t>(size));
    }
    return size;
  }

  static int GetCpu(void* const ctx) {
    return static_cast<FakeShellState*>(ctx)->cpu_id;
  }

  static int64_t Spawn(void* const ctx,         //
                       const char* const name,  //
                       const int64_t name_len,  //
                       const uint64_t arg) {
    FakeShellState* const s = static_cast<FakeShellState*>(ctx);
    const std::string prog(name, static_cast<size_t>(name_len));
    if (prog != "init" && prog != "sh" && prog != "usertest") {
      return kSyscallErrorInvalid;
    }
    s->spawns.push_back(SpawnCall{prog, arg});
    return static_cast<int64_t>(s->spawns.size());
  }

  static int64_t Wait(void* const ctx,    //
                      const int64_t pid,  //
                      UserProgramResult* const result) {
    FakeShellState* const s = static_cast<FakeShellState*>(ctx);
    if (pid < 1 || pid > static_cast<int64_t>(s->spawns.size()) ||
        result == nullptr) {
      return kSyscallErrorInvalid;
    }
    const SpawnCall& call = s->spawns[static_cast<size_t>(pid - 1)];
    if (call.arg == 6) {
      result->outcome = UserProgramOutcome::kFaulted;
      result->fault_vector = 6;
      result->fault_error_code = 0;
    } else if (call.arg == 99) {
      result->outcome = UserProgramOutcome::kKilled;
    } else {
      result->outcome = UserProgramOutcome::kExited;
      result->exit_status = static_cast<int64_t>(call.arg);
    }
    return 0;
  }

  sh::ShellEnv Env() {
    return sh::ShellEnv{
        .ctx = this,
        .read = Read,
        .write = Write,
        .get_cpu = GetCpu,
        .spawn = Spawn,
        .wait = Wait,
    };
  }
};

TEST(ShellLibTest, LineEditorHandlesBackspaceCrLfAndClamping) {
  sh::ShellLineEditor editor;
  char ch = '\b';
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kNone));
  ch = static_cast<char>(0x7F);
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kNone));
  EXPECT_THAT(editor.length, t::Eq(0));

  ch = 'a';
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kEchoChar));
  ch = '\t';
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kEchoChar));
  EXPECT_THAT(ch, t::Eq(' '));
  ch = 'b';
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kEchoChar));
  ch = '\b';
  EXPECT_THAT(editor.FeedChar(&ch),
              t::Eq(sh::ShellCharAction::kBackspaceErase));
  ch = 'c';
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kEchoChar));
  ch = '\r';
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kSubmitLine));
  EXPECT_THAT(std::string(editor.buffer), t::Eq("a c"));
  editor.ResetLine();

  // '\n' right after '\r' is ignored rather than submitting an empty line.
  ch = '\n';
  EXPECT_THAT(editor.FeedChar(&ch),
              t::Eq(sh::ShellCharAction::kIgnoreLfAfterCr));
  // A subsequent '\n' submits an empty line.
  EXPECT_THAT(editor.FeedChar(&ch), t::Eq(sh::ShellCharAction::kSubmitLine));

  // Clamping at kShellMaxLineLength.
  editor.ResetLine();
  for (int i = 0; i < sh::kShellMaxLineLength + 10; ++i) {
    char c = 'x';
    const sh::ShellCharAction action = editor.FeedChar(&c);
    if (i < sh::kShellMaxLineLength) {
      EXPECT_THAT(action, t::Eq(sh::ShellCharAction::kEchoChar));
    } else {
      EXPECT_THAT(action, t::Eq(sh::ShellCharAction::kNone));
    }
  }
  EXPECT_THAT(editor.length, t::Eq(sh::kShellMaxLineLength));
}

TEST(ShellLibTest, TokenizeAndParseInt64HandleEdgeCases) {
  sh::ShellCommand cmd;
  constexpr const char kLine[] = "  echo   hello\tworld  ";
  EXPECT_THAT(sh::TokenizeLine(kLine, sizeof(kLine) - 1, &cmd), t::Eq(3));
  EXPECT_TRUE(sh::TokenEquals(cmd.tokens[0], "echo"));
  EXPECT_TRUE(sh::TokenEquals(cmd.tokens[1], "hello"));
  EXPECT_TRUE(sh::TokenEquals(cmd.tokens[2], "world"));

  int64_t val = 0;
  EXPECT_TRUE(sh::ParseInt64(sh::ShellToken{"0", 1}, &val));
  EXPECT_THAT(val, t::Eq(0));
  EXPECT_TRUE(sh::ParseInt64(sh::ShellToken{"-42", 3}, &val));
  EXPECT_THAT(val, t::Eq(-42));
  EXPECT_TRUE(sh::ParseInt64(sh::ShellToken{"0x10", 4}, &val));
  EXPECT_THAT(val, t::Eq(16));
  EXPECT_TRUE(sh::ParseInt64(sh::ShellToken{"-9223372036854775808", 20}, &val));
  EXPECT_THAT(val, t::Eq(INT64_MIN));
  EXPECT_FALSE(sh::ParseInt64(sh::ShellToken{"9223372036854775808", 19}, &val));
  EXPECT_FALSE(sh::ParseInt64(sh::ShellToken{"0x", 2}, &val));
  EXPECT_FALSE(sh::ParseInt64(sh::ShellToken{"-", 1}, &val));
  EXPECT_FALSE(sh::ParseInt64(sh::ShellToken{"12a", 3}, &val));

  uint64_t prog_arg = 0;
  EXPECT_TRUE(sh::ParseProgramArg(sh::ShellToken{"syscalls", 8}, &prog_arg));
  EXPECT_THAT(prog_arg, t::Eq(1U));
  EXPECT_TRUE(sh::ParseProgramArg(sh::ShellToken{"fault", 5}, &prog_arg));
  EXPECT_THAT(prog_arg, t::Eq(6U));
  EXPECT_TRUE(sh::ParseProgramArg(sh::ShellToken{"buffers", 7}, &prog_arg));
  EXPECT_THAT(prog_arg, t::Eq(16U));
  EXPECT_FALSE(sh::ParseProgramArg(sh::ShellToken{"bad_alias", 9}, &prog_arg));
}

TEST(ShellLibTest, RunShellExecutesBuiltinsSpawnsProgramsAndExitsCleanly) {
  FakeShellState state;
  state.input =
      "\n"
      "   \n"
      "\b\x7f"
      "ecxx\b\bho hello shell\r\n"
      "help\n"
      "cpu\n"
      "programs\n"
      "usertest 0\n"
      "run usertest 6\n"
      "run usertest 99\n"
      "run\n"
      "usertest badarg\n"
      "unknown_cmd\n"
      "exit 7\n";
  const int64_t status = sh::RunShell(state.Env());
  EXPECT_THAT(status, t::Eq(7));
  EXPECT_THAT(state.output, t::HasSubstr("sh> "));
  EXPECT_THAT(state.output, t::HasSubstr("\b \b"));
  EXPECT_THAT(state.output, t::HasSubstr("hello shell\n"));
  EXPECT_THAT(
      state.output,
      t::HasSubstr("built-in commands: help echo cpu programs run exit"));
  EXPECT_THAT(state.output, t::HasSubstr("cpu: 3\n"));
  EXPECT_THAT(state.output, t::HasSubstr("programs: init sh usertest\n"));
  EXPECT_THAT(state.output,
              t::HasSubstr("sh: usertest exited with status 0\n"));
  EXPECT_THAT(state.output,
              t::HasSubstr("sh: usertest faulted vector=6 error=0x0\n"));
  EXPECT_THAT(state.output, t::HasSubstr("sh: usertest killed\n"));
  EXPECT_THAT(state.output, t::HasSubstr("sh: usage: run <program> [arg]\n"));
  EXPECT_THAT(state.output, t::HasSubstr("sh: invalid argument: badarg\n"));
  EXPECT_THAT(state.output, t::HasSubstr("sh: unknown command: unknown_cmd\n"));
}

}  // namespace
}  // namespace protos
