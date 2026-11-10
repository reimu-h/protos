#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "block_protocol.h"
#include "gtest/gtest.h"
#include "syscall_abi.h"
#include "user/block.h"
#include "user/disk/disk_lib.h"

namespace protos::user::disk {
namespace {

namespace t = ::testing;

struct FakeDisk {
  BlockDeviceInfo info{};
  std::vector<uint8_t> storage;
  int64_t get_info_override_rc = 0;
  bool override_get_info = false;
  int64_t read_override_rc = 0;
  bool override_read = false;
  int64_t read_reply_size_override = -1;
  int64_t write_override_rc = 0;
  bool override_write = false;
  int64_t flush_override_rc = 0;
  bool override_flush = false;
  int flush_calls = 0;
};

static void SetFixedField(char* const dst,  //
                          const int cap,    //
                          const char* const src) {
  std::memset(dst, 0, static_cast<size_t>(cap));
  for (int i = 0; i + 1 < cap && src[i] != '\0'; ++i) {
    dst[i] = src[i];
  }
}

static BlockDeviceInfo MakeInfo(const uint64_t block_count,
                                const int32_t block_size,
                                const int32_t max_transfer_blocks,
                                const uint8_t read_only = 0,
                                const uint32_t nsid = 1,
                                const char* const model = "QEMU NVMe",
                                const char* const serial = "protos0") {
  BlockDeviceInfo info{};
  info.block_count = block_count;
  info.block_size = block_size;
  info.max_transfer_blocks = max_transfer_blocks;
  info.bus = 0;
  info.device = 4;
  info.function = 0;
  info.read_only = read_only;
  info.nsid = nsid;
  SetFixedField(info.model, kBlockModelCapacity, model);
  SetFixedField(info.serial, kBlockSerialCapacity, serial);
  return info;
}

class FakeBlockHarness {
 public:
  void AddDisk(const BlockDeviceInfo& info, const uint64_t storage_blocks = 0) {
    FakeDisk d;
    d.info = info;
    const uint64_t blocks = (storage_blocks > 0) ? storage_blocks : 32;
    d.storage.assign(
        static_cast<size_t>(blocks) * static_cast<size_t>(info.block_size), 0);
    disks_.push_back(std::move(d));
  }

  FakeDisk& disk(const int idx) { return disks_[static_cast<size_t>(idx)]; }

  DiskEnv Env() {
    return DiskEnv{
        .block =
            BlockClientEnv{
                .ctx = this,
                .ipc_call = &HandleIpcCall,
            },
        .write_ctx = this,
        .write = &HandleWrite,
    };
  }

  int Run(const std::vector<const char*>& args) {
    output_.clear();
    return RunDisk(Env(),                                 //
                   static_cast<int>(args.size()),         //
                   args.empty() ? nullptr : args.data(),  //
                   scratch_,                              //
                   static_cast<int64_t>(sizeof(scratch_)));
  }

  const std::string& output() const { return output_; }
  int info_calls() const { return info_calls_; }
  int read_calls() const { return read_calls_; }
  int write_calls() const { return write_calls_; }
  int flush_calls() const { return flush_calls_; }
  int64_t last_port() const { return last_port_; }

 private:
  static int64_t HandleWrite(void* const ctx,         //
                             const char* const data,  //
                             const int64_t size) {
    auto* const self = static_cast<FakeBlockHarness*>(ctx);
    self->output_.append(data, static_cast<size_t>(size));
    return size;
  }

  static int64_t HandleIpcCall(void* const ctx,            //
                               const int64_t port,         //
                               const void* const req_buf,  //
                               const int64_t req_size,     //
                               void* const reply_buf,      //
                               const int64_t reply_cap,    //
                               int64_t* const out_reply_size) {
    auto* const self = static_cast<FakeBlockHarness*>(ctx);
    return self->OnIpcCall(port,       //
                           req_buf,    //
                           req_size,   //
                           reply_buf,  //
                           reply_cap,  //
                           out_reply_size);
  }

  int64_t OnIpcCall(const int64_t port,         //
                    const void* const req_buf,  //
                    const int64_t req_size,     //
                    void* const reply_buf,      //
                    const int64_t reply_cap,    //
                    int64_t* const out_reply_size) {
    last_port_ = port;
    if (out_reply_size != nullptr) {
      *out_reply_size = 0;
    }
    if (port != kIpcPortBlock || req_buf == nullptr ||
        req_size < static_cast<int64_t>(sizeof(BlockRpcOp))) {
      return kSyscallErrorInvalid;
    }
    BlockRpcOp op = BlockRpcOp::kNone;
    std::memcpy(&op, req_buf, sizeof(op));

    if (op == BlockRpcOp::kGetInfo) {
      ++info_calls_;
      if (req_size != static_cast<int64_t>(sizeof(BlockGetInfoRequest))) {
        return kSyscallErrorInvalid;
      }
      BlockGetInfoRequest req{};
      std::memcpy(&req, req_buf, sizeof(req));
      if (disks_.empty()) {
        return kSyscallErrorNoDevice;
      }
      if (req.disk_index < disks_.size()) {
        const FakeDisk& d = disks_[req.disk_index];
        if (d.override_get_info) {
          return d.get_info_override_rc;
        }
        if (reply_buf == nullptr ||
            reply_cap < static_cast<int64_t>(sizeof(BlockDeviceInfo))) {
          return kSyscallErrorInvalid;
        }
        std::memcpy(reply_buf, &d.info, sizeof(BlockDeviceInfo));
        if (out_reply_size != nullptr) {
          *out_reply_size = static_cast<int64_t>(sizeof(BlockDeviceInfo));
        }
        return static_cast<int64_t>(disks_.size());
      }
      return kSyscallErrorNoDevice;
    }

    if (op == BlockRpcOp::kRead) {
      ++read_calls_;
      if (req_size != static_cast<int64_t>(sizeof(BlockReadRequest))) {
        return kSyscallErrorInvalid;
      }
      BlockReadRequest req{};
      std::memcpy(&req, req_buf, sizeof(req));
      if (req.reserved != 0) {
        return kSyscallErrorInvalid;
      }
      if (req.disk_index >= disks_.size()) {
        return kSyscallErrorNoDevice;
      }
      FakeDisk& d = disks_[req.disk_index];
      if (d.override_read) {
        if (d.read_reply_size_override >= 0 && out_reply_size != nullptr) {
          *out_reply_size = d.read_reply_size_override;
        }
        return d.read_override_rc;
      }
      if (req.block_count < 0 || req.block_count > d.info.max_transfer_blocks ||
          req.lba > d.info.block_count ||
          static_cast<uint64_t>(req.block_count) >
              d.info.block_count - req.lba) {
        return kSyscallErrorInvalid;
      }
      const int64_t bytes =
          static_cast<int64_t>(req.block_count) * d.info.block_size;
      if (bytes > reply_cap || (bytes > 0 && reply_buf == nullptr)) {
        return kSyscallErrorInvalid;
      }
      const uint64_t byte_off =
          req.lba * static_cast<uint64_t>(d.info.block_size);
      if (bytes > 0) {
        if (byte_off + static_cast<uint64_t>(bytes) > d.storage.size()) {
          std::memset(reply_buf, 0, static_cast<size_t>(bytes));
        } else {
          const uint8_t* const src_ptr = d.storage.data() + byte_off;
          std::memcpy(reply_buf, src_ptr, static_cast<size_t>(bytes));
        }
      }
      if (out_reply_size != nullptr) {
        *out_reply_size = (d.read_reply_size_override >= 0)
                              ? d.read_reply_size_override
                              : bytes;
      }
      return req.block_count;
    }

    if (op == BlockRpcOp::kWrite) {
      ++write_calls_;
      if (req_size < static_cast<int64_t>(sizeof(BlockWriteRequest))) {
        return kSyscallErrorInvalid;
      }
      BlockWriteRequest req{};
      std::memcpy(&req, req_buf, sizeof(req));
      if (req.reserved != 0) {
        return kSyscallErrorInvalid;
      }
      if (req.disk_index >= disks_.size()) {
        return kSyscallErrorNoDevice;
      }
      FakeDisk& d = disks_[req.disk_index];
      if (d.override_write) {
        return d.write_override_rc;
      }
      if (d.info.read_only != 0) {
        return kSyscallErrorPermission;
      }
      if (req.block_count < 0 || req.block_count > d.info.max_transfer_blocks ||
          req.lba > d.info.block_count ||
          static_cast<uint64_t>(req.block_count) >
              d.info.block_count - req.lba) {
        return kSyscallErrorInvalid;
      }
      const int64_t bytes =
          static_cast<int64_t>(req.block_count) * d.info.block_size;
      if (req_size != static_cast<int64_t>(sizeof(BlockWriteRequest)) + bytes) {
        return kSyscallErrorInvalid;
      }
      const uint64_t byte_off =
          req.lba * static_cast<uint64_t>(d.info.block_size);
      if (bytes > 0 &&
          byte_off + static_cast<uint64_t>(bytes) <= d.storage.size()) {
        const auto* const payload =
            static_cast<const uint8_t*>(req_buf) + sizeof(BlockWriteRequest);
        uint8_t* const dst_ptr = d.storage.data() + byte_off;
        std::memcpy(dst_ptr, payload, static_cast<size_t>(bytes));
      }
      return req.block_count;
    }

    if (op == BlockRpcOp::kFlush) {
      ++flush_calls_;
      if (req_size != static_cast<int64_t>(sizeof(BlockFlushRequest))) {
        return kSyscallErrorInvalid;
      }
      BlockFlushRequest req{};
      std::memcpy(&req, req_buf, sizeof(req));
      if (req.disk_index >= disks_.size()) {
        return kSyscallErrorNoDevice;
      }
      FakeDisk& d = disks_[req.disk_index];
      ++d.flush_calls;
      if (d.override_flush) {
        return d.flush_override_rc;
      }
      return 0;
    }

    return kSyscallErrorInvalid;
  }

  std::vector<FakeDisk> disks_;
  alignas(8) uint8_t scratch_[kBlockWriteScratchBytes]{};
  std::string output_;
  int info_calls_ = 0;
  int read_calls_ = 0;
  int write_calls_ = 0;
  int flush_calls_ = 0;
  int64_t last_port_ = -1;
};

TEST(DiskTest, ParseCliValidAndInvalidForms) {
  ParsedDiskCommand cmd{};
  const char* usage = nullptr;

  const char* const def_argv[] = {"disk"};
  EXPECT_TRUE(ParseDiskCommand(1, def_argv, &cmd, &usage));
  EXPECT_EQ(cmd.op, DiskSubcommand::kInfo);
  EXPECT_FALSE(cmd.has_disk);

  const char* const info_argv[] = {"disk", "info"};
  EXPECT_TRUE(ParseDiskCommand(2, info_argv, &cmd, &usage));
  EXPECT_EQ(cmd.op, DiskSubcommand::kInfo);

  const char* const disk_only_argv[] = {"disk", "-d", "2"};
  EXPECT_TRUE(ParseDiskCommand(3, disk_only_argv, &cmd, &usage));
  EXPECT_EQ(cmd.op, DiskSubcommand::kInfo);
  EXPECT_TRUE(cmd.has_disk);
  EXPECT_EQ(cmd.disk_index, 2U);

  const char* const disk_read_argv[] = {"disk", "-d", "1", "read", "7", "4"};
  EXPECT_TRUE(ParseDiskCommand(6, disk_read_argv, &cmd, &usage));
  EXPECT_EQ(cmd.op, DiskSubcommand::kRead);
  EXPECT_TRUE(cmd.has_disk);
  EXPECT_EQ(cmd.disk_index, 1U);
  EXPECT_EQ(cmd.lba, 7ULL);
  EXPECT_EQ(cmd.count, 4);

  const char* const disk_write_argv[] = {"disk", "-d", "3",    "write",
                                         "9",    "hi", "there"};
  EXPECT_TRUE(ParseDiskCommand(7, disk_write_argv, &cmd, &usage));
  EXPECT_EQ(cmd.op, DiskSubcommand::kWrite);
  EXPECT_EQ(cmd.disk_index, 3U);
  EXPECT_EQ(cmd.lba, 9ULL);
  EXPECT_EQ(cmd.text_argc, 2);
  EXPECT_STREQ(cmd.text_argv[0], "hi");

  const char* const disk_zero_argv[] = {"disk", "-d", "0", "zero", "4", "8"};
  EXPECT_TRUE(ParseDiskCommand(6, disk_zero_argv, &cmd, &usage));
  EXPECT_EQ(cmd.op, DiskSubcommand::kZero);
  EXPECT_EQ(cmd.lba, 4ULL);
  EXPECT_EQ(cmd.count, 8);

  // D-09: trailing arguments to `info` must fail.
  const char* const bad_info_extra[] = {"disk", "info", "extra"};
  EXPECT_FALSE(ParseDiskCommand(3, bad_info_extra, &cmd, &usage));
  EXPECT_FALSE(ParseDiskCommand(3, bad_info_extra, &cmd, nullptr));

  // D-10: count == 0 and overflow must fail in parser; count == 17 parses so
  // dynamic max_transfer_blocks check handles it.
  const char* const bad_read_zero[] = {"disk", "read", "0", "0"};
  const char* const bad_zero_zero[] = {"disk", "zero", "0", "0"};
  const char* const bad_read_huge[] = {"disk", "read", "0", "2147483648"};
  const char* const ok_read_17[] = {"disk", "read", "0", "17"};
  EXPECT_FALSE(ParseDiskCommand(4, bad_read_zero, &cmd, &usage));
  EXPECT_FALSE(ParseDiskCommand(4, bad_zero_zero, &cmd, &usage));
  EXPECT_FALSE(ParseDiskCommand(4, bad_read_huge, &cmd, &usage));
  EXPECT_TRUE(ParseDiskCommand(4, ok_read_17, &cmd, &usage));
  EXPECT_EQ(cmd.count, 17);

  // D-11 / D-13: -d edge cases and null argv elements.
  const char* const bad_d_missing[] = {"disk", "-d"};
  const char* const bad_d_range[] = {"disk", "-d", "4294967296"};
  const char* const bad_null_elem[] = {"disk", nullptr};
  EXPECT_FALSE(ParseDiskCommand(2, bad_d_missing, &cmd, &usage));
  EXPECT_FALSE(ParseDiskCommand(3, bad_d_range, &cmd, &usage));
  EXPECT_FALSE(ParseDiskCommand(2, bad_null_elem, &cmd, &usage));
  EXPECT_FALSE(ParseDiskCommand(1, nullptr, &cmd, &usage));
  EXPECT_FALSE(ParseDiskCommand(-1, def_argv, &cmd, &usage));
  EXPECT_TRUE(ParseDiskCommand(0, nullptr, &cmd, &usage));
  EXPECT_EQ(cmd.op, DiskSubcommand::kInfo);

  // ParseU64 / ParseU32 boundary & failure paths.
  uint64_t u64 = 0;
  EXPECT_FALSE(ParseU64(nullptr, &u64));
  EXPECT_FALSE(ParseU64("", &u64));
  EXPECT_FALSE(ParseU64("0", nullptr));
  EXPECT_FALSE(ParseU64("12a", &u64));
  EXPECT_FALSE(ParseU64("-1", &u64));
  EXPECT_FALSE(ParseU64("18446744073709551616", &u64));
  EXPECT_FALSE(ParseU64("184467440737095516150", &u64));
  EXPECT_TRUE(ParseU64("18446744073709551615", &u64));
  EXPECT_EQ(u64, UINT64_MAX);

  uint32_t u32 = 0;
  EXPECT_FALSE(ParseU32("1", nullptr));
  EXPECT_TRUE(ParseU32("4294967295", &u32));
  EXPECT_EQ(u32, UINT32_MAX);
  EXPECT_FALSE(ParseU32("4294967296", &u32));

  // RunDisk usage paths (kUsageGeneral, kUsageRead, kUsageWrite, kUsageZero).
  FakeBlockHarness h_usage;
  EXPECT_EQ(h_usage.Run({"disk", "bogus"}), 1);
  EXPECT_EQ(h_usage.output(), kUsageGeneral);
  EXPECT_EQ(h_usage.Run({"disk", "read", "bad"}), 1);
  EXPECT_EQ(h_usage.output(), kUsageRead);
  EXPECT_EQ(h_usage.Run({"disk", "write", "0"}), 1);
  EXPECT_EQ(h_usage.output(), kUsageWrite);
  EXPECT_EQ(h_usage.Run({"disk", "write", "bad", "hi"}), 1);
  EXPECT_EQ(h_usage.output(), kUsageWrite);
  EXPECT_EQ(h_usage.Run({"disk", "zero", "bad"}), 1);
  EXPECT_EQ(h_usage.output(), kUsageZero);
}

TEST(DiskTest, InfoMultiDiskReadOnlyOverflowSafeAndErrors) {
  FakeBlockHarness h;
  h.AddDisk(MakeInfo(8192, 512, 8, /*read_only=*/0, 1, "NVMe A", "serA"));
  h.AddDisk(
      MakeInfo(UINT64_MAX, 4096, 1, /*read_only=*/1, 2, "NVMe B", "serB"));

  EXPECT_EQ(h.Run({"disk", "info"}), 0);
  EXPECT_NE(h.output().find("disk 0: NVMe A (serial=serA, nsid=1, pci=00:04.0, "
                            "ro=0)\n  8192 blocks x 512 B = 4 MiB "
                            "(max 8 blocks/call)\n"),
            std::string::npos);
  // D-06: UINT64_MAX * 4096 must not overflow; 4096 B/block = 4 KiB/block ->
  // (UINT64_MAX * 4) / 1024 = 72057594037927935 MiB.
  EXPECT_NE(h.output().find("disk 1: NVMe B (serial=serB, nsid=2, pci=00:04.0, "
                            "ro=1)\n  18446744073709551615 blocks x 4096 B = "
                            "72057594037927935 MiB (max 1 blocks/call)\n"),
            std::string::npos);

  // D-11: -d 1 selects only disk 1.
  EXPECT_EQ(h.Run({"disk", "-d", "1", "info"}), 0);
  EXPECT_EQ(h.output().find("disk 0:"), std::string::npos);
  EXPECT_NE(h.output().find("disk 1: NVMe B"), std::string::npos);

  // Nonexistent disk and empty harness errors.
  EXPECT_EQ(h.Run({"disk", "-d", "99", "info"}), 1);
  EXPECT_EQ(h.output(), "disk: info: no block device\n");
  FakeBlockHarness h_empty;
  EXPECT_EQ(h_empty.Run({"disk", "info"}), 1);
  EXPECT_EQ(h_empty.output(), "disk: info: no block device\n");

  // D-07: d > 0 error in `disk info` must fail with exit status 1.
  h.disk(1).override_get_info = true;
  h.disk(1).get_info_override_rc = kSyscallErrorIo;
  EXPECT_EQ(h.Run({"disk", "info"}), 1);
  EXPECT_NE(h.output().find("disk: info: I/O error\n"), std::string::npos);
  h.disk(1).override_get_info = false;

  // D-07: disk_count == 0 and disk_count > UINT32_MAX must fail cleanly.
  struct CountCase {
    std::string out;
    int64_t count = 0;
  } count_case;
  const DiskEnv env{
      .block =
          BlockClientEnv{
              .ctx = &count_case,
              .ipc_call = +[](void* const ctx,                //
                              const int64_t /*port*/,         //
                              const void* const /*req_buf*/,  //
                              const int64_t /*req_size*/,     //
                              void* const reply_buf,          //
                              const int64_t /*reply_cap*/,    //
                              int64_t* const out_reply_size) -> int64_t {
                const BlockDeviceInfo info = MakeInfo(1, 512, 1);
                std::memcpy(reply_buf, &info, sizeof(info));
                *out_reply_size = static_cast<int64_t>(sizeof(info));
                return static_cast<const CountCase*>(ctx)->count;
              },
          },
      .write_ctx = &count_case,
      .write = +[](void* const ctx,         //
                   const char* const data,  //
                   const int64_t size) -> int64_t {
        static_cast<CountCase*>(ctx)->out.append(data,
                                                 static_cast<size_t>(size));
        return size;
      },
  };
  count_case.count = int64_t{UINT32_MAX} + 2;
  EXPECT_EQ(CmdInfo(env, false, 0), 1);
  EXPECT_EQ(count_case.out, "disk: info: I/O error\n");
  count_case.out.clear();
  count_case.count = 0;
  EXPECT_EQ(CmdInfo(env, false, 0), 1);
  EXPECT_EQ(count_case.out, "disk: info: I/O error\n");

  FakeBlockHarness h_one;
  h_one.AddDisk(MakeInfo(1, 512, 1));
  EXPECT_EQ(h_one.Run({"disk", "-d", "0", "info"}), 0);
  EXPECT_NE(h_one.output().find("1 blocks x 512 B = 512 B"), std::string::npos);

  // FormatCapacity: 0 B, 512 B KiB, 1024 B, 2048 B, 4096 B KiB/MiB.
  FakeBlockHarness h_sizes;
  h_sizes.AddDisk(MakeInfo(1, 1024, 4, 0, 1, "Disk 1K", "s1k"));
  h_sizes.AddDisk(MakeInfo(512, 2048, 2, 0, 2, "Disk 2K", "s2k"));
  EXPECT_EQ(h_sizes.Run({"disk", "info"}), 0);
  EXPECT_NE(h_sizes.output().find("1 blocks x 1024 B = 1 KiB"),
            std::string::npos);
  EXPECT_NE(h_sizes.output().find("512 blocks x 2048 B = 1 MiB"),
            std::string::npos);

  const auto format_cap = [&](const uint64_t blocks, const int32_t bs) {
    count_case.out.clear();
    Out cap_out(env);
    FormatCapacity(blocks, bs, cap_out);
    cap_out.Flush();
    return count_case.out;
  };
  EXPECT_EQ(format_cap(0, 512), "0 B");
  EXPECT_EQ(format_cap(1, 513), "0 B");
  EXPECT_EQ(format_cap(2, 512), "1 KiB");
  EXPECT_EQ(format_cap(2047, 512), "1023 KiB");
  EXPECT_EQ(format_cap(2048, 512), "1 MiB");
  EXPECT_EQ(format_cap(1023, 1024), "1023 KiB");
  EXPECT_EQ(format_cap(1024, 1024), "1 MiB");
  EXPECT_EQ(format_cap(1, 2048), "2 KiB");
  EXPECT_EQ(format_cap(511, 2048), "1022 KiB");
  EXPECT_EQ(format_cap(1, 4096), "4 KiB");
  EXPECT_EQ(format_cap(255, 4096), "1020 KiB");
  EXPECT_EQ(format_cap(256, 4096), "1 MiB");

  // D-08: FormatDeviceInfo enforces IsValidBlockDeviceInfo and bounded StrN.
  BlockDeviceInfo bad_info = MakeInfo(16, 512, 8);
  std::memset(bad_info.model, 'M', kBlockModelCapacity);
  Out fmt_out(env);
  EXPECT_FALSE(FormatDeviceInfo(0, bad_info, fmt_out));
}

TEST(DiskTest, ReadZeroPrefixNonzeroMiddleZeroSuffixRegressionD01) {
  FakeBlockHarness h;
  h.AddDisk(MakeInfo(64, 512, 8));

  // All zero block emits "(zero through 512 B)".
  EXPECT_EQ(h.Run({"disk", "read", "0"}), 0);
  EXPECT_NE(h.output().find("...\n(zero through 512 B)\n"), std::string::npos);

  // D-01 regression: bytes 0..15 zero, bytes 16..31 non-zero, bytes 32..511
  // zero. Must NOT emit "(zero through 512 B)".
  std::vector<uint8_t>& st = h.disk(0).storage;
  st[16] = 'A';
  st[17] = 'B';
  EXPECT_EQ(h.Run({"disk", "read", "0"}), 0);
  EXPECT_NE(h.output().find("0010: 41 42 00"), std::string::npos);
  EXPECT_NE(h.output().find("|AB..............|"), std::string::npos);
  EXPECT_EQ(h.output().find("(zero through"), std::string::npos);
}

TEST(DiskTest, ReadCheckedMismatchAndDynamicCountBounds) {
  FakeBlockHarness h;
  h.AddDisk(MakeInfo(64, 512, 8));
  h.AddDisk(MakeInfo(64, 4096, 1));

  // Nonexistent disk fails cleanly.
  EXPECT_EQ(h.Run({"disk", "-d", "99", "read", "0"}), 1);
  EXPECT_EQ(h.output(), "disk: read: no block device\n");

  // Count > dynamic max_transfer_blocks fails cleanly on both 512 B and 4 KiB
  // disks, while count == 8 succeeds on disk 0.
  EXPECT_EQ(h.Run({"disk", "read", "0", "9"}), 1);
  EXPECT_NE(h.output().find("disk: read: block count out of range\n"),
            std::string::npos);
  EXPECT_EQ(h.Run({"disk", "-d", "1", "read", "0", "2"}), 1);
  EXPECT_NE(h.output().find("disk: read: block count out of range\n"),
            std::string::npos);
  EXPECT_EQ(h.Run({"disk", "read", "0", "8"}), 0);
  EXPECT_NE(h.output().find("(zero through 4096 B)\n"), std::string::npos);

  // D-03: ReadBlocksChecked rejects mismatched reply_size even if rc == count.
  h.disk(0).read_reply_size_override = 1024;
  EXPECT_EQ(h.Run({"disk", "read", "0", "1"}), 1);
  EXPECT_NE(h.output().find("disk: read: I/O error\n"), std::string::npos);
  h.disk(0).read_reply_size_override = -1;

  // D-03: ReadBlocksChecked rejects rc != count.
  h.disk(0).override_read = true;
  h.disk(0).read_override_rc = 2;
  EXPECT_EQ(h.Run({"disk", "read", "0", "1"}), 1);
  EXPECT_NE(h.output().find("disk: read: I/O error\n"), std::string::npos);
}

TEST(DiskTest, WriteAndZeroPersistenceMultiDiskReadOnlyAndErrors) {
  FakeBlockHarness h;
  h.AddDisk(MakeInfo(64, 512, 8, /*read_only=*/0));
  h.AddDisk(MakeInfo(64, 512, 8, /*read_only=*/0));
  h.AddDisk(MakeInfo(64, 512, 8, /*read_only=*/1));

  // Write across two 512-B blocks (> 512 bytes) on disk 1 via `-d 1`.
  const std::string long_arg(600, 'x');
  EXPECT_EQ(h.Run({"disk", "-d", "1", "write", "3", long_arg.c_str()}), 0);
  EXPECT_EQ(h.output(), "wrote 2 block(s) at lba 3\n");
  EXPECT_EQ(h.disk(1).flush_calls, 1);

  EXPECT_EQ(h.Run({"disk", "-d", "1", "read", "3", "2"}), 0);
  EXPECT_NE(h.output().find("0000: 78 78"), std::string::npos);

  // Multi-arg text write inserts a single space between tokens and trailing LF.
  EXPECT_EQ(h.Run({"disk", "write", "10", "hello", "world"}), 0);
  EXPECT_EQ(h.output(), "wrote 1 block(s) at lba 10\n");
  EXPECT_EQ(h.Run({"disk", "read", "10", "1"}), 0);
  EXPECT_NE(h.output().find("|hello world.....|"), std::string::npos);

  // Exact max_bytes boundaries (4095 + '\n' = 4096; 4093 + ' ' + 'x' + '\n').
  const std::string exact_4095(4095, 'A');
  EXPECT_EQ(h.Run({"disk", "write", "12", exact_4095.c_str()}), 0);
  EXPECT_EQ(h.output(), "wrote 8 block(s) at lba 12\n");
  EXPECT_EQ(h.Run({"disk", "read", "12", "8"}), 0);
  EXPECT_NE(
      h.output().find("0ff0: 41 41 41 41 41 41 41 41 41 41 41 41 41 41 41 0a"),
      std::string::npos);

  const std::string exact_4093(4093, 'B');
  EXPECT_EQ(h.Run({"disk", "write", "12", exact_4093.c_str(), "x"}), 0);
  EXPECT_EQ(h.output(), "wrote 8 block(s) at lba 12\n");

  // Text overflow on newline, space separator, and character loop.
  const std::string too_long(kBlockMaxIoBytes, 'z');
  const std::string over_4097(kBlockMaxIoBytes + 1, 'D');
  const int writes_before = h.write_calls();
  EXPECT_EQ(h.Run({"disk", "write", "0", too_long.c_str()}), 1);
  EXPECT_EQ(h.output(), "disk: write: text too long\n");
  EXPECT_EQ(h.Run({"disk", "write", "0", too_long.c_str(), "x"}), 1);
  EXPECT_EQ(h.output(), "disk: write: text too long\n");
  EXPECT_EQ(h.Run({"disk", "write", "0", over_4097.c_str()}), 1);
  EXPECT_EQ(h.output(), "disk: write: text too long\n");
  EXPECT_EQ(h.write_calls(), writes_before);

  // Zero on disk 1 clears both blocks and flushes.
  EXPECT_EQ(h.Run({"disk", "-d", "1", "zero", "3", "2"}), 0);
  EXPECT_EQ(h.output(), "zeroed 2 block(s) at lba 3\n");
  EXPECT_EQ(h.disk(1).flush_calls, 2);
  EXPECT_EQ(h.Run({"disk", "-d", "1", "read", "3", "2"}), 0);
  EXPECT_NE(h.output().find("(zero through 1024 B)\n"), std::string::npos);

  // Zero count out of range and GetBlockDeviceInfo errors for write/zero.
  EXPECT_EQ(h.Run({"disk", "zero", "0", "9"}), 1);
  EXPECT_EQ(h.output(), "disk: zero: block count out of range\n");
  EXPECT_EQ(h.Run({"disk", "-d", "99", "write", "0", "hi"}), 1);
  EXPECT_EQ(h.output(), "disk: write: no block device\n");
  EXPECT_EQ(h.Run({"disk", "-d", "99", "zero", "0", "1"}), 1);
  EXPECT_EQ(h.output(), "disk: zero: no block device\n");

  // Read-only disk 2 rejects write and zero before issuing kWrite.
  EXPECT_EQ(h.Run({"disk", "-d", "2", "write", "0", "hi"}), 1);
  EXPECT_NE(h.output().find("read-only or permission denied\n"),
            std::string::npos);
  EXPECT_EQ(h.Run({"disk", "-d", "2", "zero", "0"}), 1);
  EXPECT_NE(h.output().find("read-only or permission denied\n"),
            std::string::npos);

  // D-05: positive mismatched write rc and nonzero flush_rc must fail.
  h.disk(0).override_write = true;
  h.disk(0).write_override_rc = 2;
  EXPECT_EQ(h.Run({"disk", "write", "0", "hi"}), 1);
  EXPECT_NE(h.output().find("disk: write: I/O error\n"), std::string::npos);
  EXPECT_EQ(h.Run({"disk", "zero", "0", "1"}), 1);
  EXPECT_NE(h.output().find("disk: zero: I/O error\n"), std::string::npos);
  h.disk(0).override_write = false;

  h.disk(0).override_flush = true;
  h.disk(0).flush_override_rc = 1;
  EXPECT_EQ(h.Run({"disk", "write", "0", "hi"}), 1);
  EXPECT_NE(h.output().find("disk: flush: I/O error\n"), std::string::npos);
  EXPECT_EQ(h.Run({"disk", "zero", "0", "1"}), 1);
  EXPECT_NE(h.output().find("disk: flush: I/O error\n"), std::string::npos);
}

TEST(DiskTest, DefensiveGeometryScratchAndFormattingEdgeCases) {
  // D-02: malicious BlockDeviceInfo with max_transfer_blocks * block_size >
  // kBlockMaxIoBytes must be rejected by IsValidBlockDeviceInfo before any .bss
  // write.
  std::string out;
  const BlockDeviceInfo evil_info{
      .block_count = 64,
      .block_size = 512,
      .max_transfer_blocks = 16,
      .bus = 0,
      .device = 0,
      .function = 0,
      .read_only = 0,
      .nsid = 1,
      .model = "Evil",
      .serial = "evil",
  };
  const DiskEnv evil_env{
      .block =
          BlockClientEnv{
              .ctx = const_cast<BlockDeviceInfo*>(&evil_info),
              .ipc_call = +[](void* const ctx,                //
                              const int64_t /*port*/,         //
                              const void* const /*req_buf*/,  //
                              const int64_t /*req_size*/,     //
                              void* const reply_buf,          //
                              const int64_t reply_cap,        //
                              int64_t* const out_reply_size) -> int64_t {
                constexpr int64_t kInfoBytes =
                    static_cast<int64_t>(sizeof(BlockDeviceInfo));
                if (reply_buf != nullptr && reply_cap >= kInfoBytes) {
                  std::memcpy(reply_buf, ctx, sizeof(BlockDeviceInfo));
                  if (out_reply_size != nullptr) {
                    *out_reply_size = kInfoBytes;
                  }
                  return 1;
                }
                return kSyscallErrorInvalid;
              },
          },
      .write_ctx = &out,
      .write = +[](void* const ctx,         //
                   const char* const data,  //
                   const int64_t size) -> int64_t {
        static_cast<std::string*>(ctx)->append(data, static_cast<size_t>(size));
        return size;
      },
  };
  EXPECT_FALSE(IsValidBlockDeviceInfo(evil_info));

  // Commands must reject invalid BlockDeviceInfo from the server.
  const char* const info_argv[] = {"disk", "info"};
  const char* const info_d0_argv[] = {"disk", "-d", "0", "info"};
  const char* const read_argv[] = {"disk", "read", "0"};
  const char* const write_argv[] = {"disk", "write", "0", "hi"};
  const char* const zero_argv[] = {"disk", "zero", "0", "1"};
  alignas(8) uint8_t scratch[kBlockWriteScratchBytes]{};
  out.clear();
  EXPECT_EQ(RunDisk(evil_env, 2, info_argv, scratch, sizeof(scratch)), 1);
  EXPECT_EQ(out, "disk: info: I/O error\n");
  out.clear();
  EXPECT_EQ(RunDisk(evil_env, 4, info_d0_argv, scratch, sizeof(scratch)), 1);
  EXPECT_EQ(out, "disk: info: I/O error\n");
  out.clear();
  EXPECT_EQ(RunDisk(evil_env, 3, read_argv, scratch, sizeof(scratch)), 1);
  EXPECT_EQ(out, "disk: read: I/O error\n");
  out.clear();
  EXPECT_EQ(RunDisk(evil_env, 4, write_argv, scratch, sizeof(scratch)), 1);
  EXPECT_EQ(out, "disk: write: I/O error\n");
  out.clear();
  EXPECT_EQ(RunDisk(evil_env, 4, zero_argv, scratch, sizeof(scratch)), 1);
  EXPECT_EQ(out, "disk: zero: I/O error\n");

  // D-12 & D-13: Out::Hex clamps digits, StrEq/WriteCStr/StrN handle nullptr,
  // PrintBlockError covers kSyscallErrorNoDevice, positive status, and
  // INT64_MIN, and RunDisk/CmdWriteText/CmdZero guard null/undersized scratch.
  out.clear();
  WriteCStr(evil_env, nullptr);
  Out o(evil_env);
  o.Hex(0x123456789ABCDEF0ULL, 32)
      .Char('/')
      .Hex(0xAB, -3)
      .Str(nullptr)
      .StrN(nullptr, 4)
      .StrN("abc", 0)
      .StrN("abcdef", 3);
  o.Flush();
  EXPECT_EQ(out, "123456789abcdef0/abc");

  out.clear();
  PrintBlockError(evil_env, "op", INT64_MIN);
  EXPECT_EQ(out, "disk: op: error 9223372036854775808\n");

  out.clear();
  PrintBlockError(evil_env, "op", kSyscallErrorNoDevice);
  EXPECT_EQ(out, "disk: op: no block device\n");

  out.clear();
  PrintBlockError(evil_env, "op", 42);
  EXPECT_EQ(out, "disk: op: unexpected status 42\n");

  EXPECT_TRUE(StrEq(nullptr, nullptr));
  EXPECT_FALSE(StrEq(nullptr, "a"));
  EXPECT_FALSE(StrEq("a", nullptr));

  out.clear();
  EXPECT_EQ(RunDisk(evil_env, 3, read_argv, nullptr, 0), 1);
  EXPECT_NE(out.find("disk: read: invalid argument or LBA out of range\n"),
            std::string::npos);

  out.clear();
  EXPECT_EQ(RunDisk(evil_env, 4, write_argv, nullptr, 0), 1);
  EXPECT_NE(out.find("disk: write: invalid argument or LBA out of range\n"),
            std::string::npos);

  out.clear();
  EXPECT_EQ(RunDisk(evil_env, 4, zero_argv, nullptr, 0), 1);
  EXPECT_NE(out.find("disk: zero: invalid argument or LBA out of range\n"),
            std::string::npos);

  FakeBlockHarness h_cap;
  h_cap.AddDisk(MakeInfo(64, 512, 8));
  alignas(8) uint8_t small_scratch[sizeof(BlockWriteRequest) + 512]{};
  const char* const read2_argv[] = {"disk", "read", "0", "2"};
  const int small_rc = RunDisk(h_cap.Env(),    //
                               4,              //
                               read2_argv,     //
                               small_scratch,  //
                               static_cast<int64_t>(sizeof(small_scratch)));
  EXPECT_EQ(small_rc, 1);
  EXPECT_EQ(h_cap.output(), "disk: read: I/O error\n");

  FakeBlockHarness h_null_arg;
  h_null_arg.AddDisk(MakeInfo(64, 512, 8));
  const char* const null_text_argv[] = {"hi", nullptr};
  const int null_arg_rc = CmdWriteText(h_null_arg.Env(),  //
                                       0,                 //
                                       /*force=*/false,   //
                                       0,                 //
                                       2,                 //
                                       null_text_argv,    //
                                       scratch,           //
                                       static_cast<int64_t>(sizeof(scratch)));
  EXPECT_EQ(null_arg_rc, 1);
  EXPECT_EQ(h_null_arg.output(),
            "disk: write: invalid argument or LBA out of range\n");
}

TEST(DiskTest, ForceOptionGoesOnlyBeforeWriteAndZero) {
  ParsedDiskCommand cmd{};
  const char* usage = nullptr;

  const char* const write_argv[] = {"disk", "write", "9", "hi"};
  EXPECT_TRUE(ParseDiskCommand(4, write_argv, &cmd, &usage));
  EXPECT_FALSE(cmd.force);

  const char* const force_write_argv[] = {"disk", "-f", "write", "9", "hi"};
  EXPECT_TRUE(ParseDiskCommand(5, force_write_argv, &cmd, &usage));
  EXPECT_EQ(cmd.op, DiskSubcommand::kWrite);
  EXPECT_TRUE(cmd.force);
  EXPECT_FALSE(cmd.has_disk);
  EXPECT_EQ(cmd.lba, 9ULL);
  EXPECT_EQ(cmd.text_argc, 1);
  EXPECT_STREQ(cmd.text_argv[0], "hi");

  // -d and -f come in either order.
  const char* const both_argv[] = {"disk", "-d", "3", "-f", "zero", "4", "8"};
  EXPECT_TRUE(ParseDiskCommand(7, both_argv, &cmd, &usage));
  EXPECT_EQ(cmd.op, DiskSubcommand::kZero);
  EXPECT_TRUE(cmd.force);
  EXPECT_TRUE(cmd.has_disk);
  EXPECT_EQ(cmd.disk_index, 3U);
  EXPECT_EQ(cmd.lba, 4ULL);
  EXPECT_EQ(cmd.count, 8);
  const char* const swapped_argv[] = {"disk", "-f", "-d", "3", "zero", "4"};
  EXPECT_TRUE(ParseDiskCommand(6, swapped_argv, &cmd, &usage));
  EXPECT_EQ(cmd.op, DiskSubcommand::kZero);
  EXPECT_TRUE(cmd.force);
  EXPECT_EQ(cmd.disk_index, 3U);

  // After the subcommand, -f is text for write and no block count for zero.
  const char* const f_as_text_argv[] = {"disk", "write", "0", "-f"};
  EXPECT_TRUE(ParseDiskCommand(4, f_as_text_argv, &cmd, &usage));
  EXPECT_FALSE(cmd.force);
  EXPECT_STREQ(cmd.text_argv[0], "-f");
  const char* const f_as_count_argv[] = {"disk", "zero", "0", "-f"};
  EXPECT_FALSE(ParseDiskCommand(4, f_as_count_argv, &cmd, &usage));
  EXPECT_STREQ(usage, kUsageZero);

  // -f goes with write and zero only, and each option at most once.
  FakeBlockHarness h;
  h.AddDisk(MakeInfo(64, 512, 8));
  const std::vector<std::vector<const char*>> bad_forms = {
      {"disk", "-f"},
      {"disk", "-f", "info"},
      {"disk", "-f", "read", "0"},
      {"disk", "-d", "0", "-f"},
      {"disk", "-f", "-f", "write", "0", "hi"},
      {"disk", "-d", "0", "-d", "0", "info"},
  };
  for (const std::vector<const char*>& args : bad_forms) {
    EXPECT_EQ(h.Run(args), 1) << args.size() << " arguments";
    EXPECT_EQ(h.output(), kUsageGeneral);
  }
  EXPECT_EQ(h.info_calls(), 0);
}

TEST(DiskTest, WriteAndZeroNeedForceOnADiskThatIsNoTestImage) {
  FakeBlockHarness h;
  h.AddDisk(MakeInfo(64, 512, 8, /*read_only=*/0, 1, "Real SSD", "S4EWNX0M1"));
  h.AddDisk(MakeInfo(64, 512, 8, /*read_only=*/1, 1, "Real RO", "S4EWNX0M2"));
  h.AddDisk(MakeInfo(64, 512, 8, /*read_only=*/0, 1, "QEMU NVMe", "protos1"));
  std::vector<uint8_t>& real = h.disk(0).storage;
  real[0] = 'M';
  real[1] = 'B';
  real[2] = 'R';
  const std::vector<uint8_t> real_before = real;

  // Without -f, write and zero refuse before they send a write.
  EXPECT_EQ(h.Run({"disk", "write", "0", "hi"}), 1);
  EXPECT_EQ(h.output(),
            "disk: write: not a test disk (serial is not protos*); "
            "-f overwrites it\n");
  EXPECT_EQ(h.Run({"disk", "zero", "0"}), 1);
  EXPECT_EQ(h.output(),
            "disk: zero: not a test disk (serial is not protos*); "
            "-f overwrites it\n");
  EXPECT_EQ(h.write_calls(), 0);
  EXPECT_EQ(h.flush_calls(), 0);
  EXPECT_EQ(real, real_before);

  // info and read need no -f.
  EXPECT_EQ(h.Run({"disk", "-d", "0", "info"}), 0);
  EXPECT_NE(h.output().find("serial=S4EWNX0M1,"), std::string::npos);
  EXPECT_EQ(h.Run({"disk", "read", "0"}), 0);
  EXPECT_NE(h.output().find("|MBR.............|"), std::string::npos);

  // A read-only disk says that it is one, with or without -f.
  EXPECT_EQ(h.Run({"disk", "-d", "1", "write", "0", "hi"}), 1);
  EXPECT_EQ(h.output(), "disk: write: read-only or permission denied\n");
  EXPECT_EQ(h.Run({"disk", "-d", "1", "-f", "zero", "0"}), 1);
  EXPECT_EQ(h.output(), "disk: zero: read-only or permission denied\n");
  EXPECT_EQ(h.write_calls(), 0);

  // -f overwrites the disk.
  EXPECT_EQ(h.Run({"disk", "-f", "write", "0", "hi"}), 0);
  EXPECT_EQ(h.output(), "wrote 1 block(s) at lba 0\n");
  EXPECT_EQ(real[0], 'h');
  EXPECT_EQ(real[2], '\n');
  EXPECT_EQ(h.Run({"disk", "-f", "zero", "0"}), 0);
  EXPECT_EQ(h.output(), "zeroed 1 block(s) at lba 0\n");
  EXPECT_EQ(real, std::vector<uint8_t>(real.size(), 0));
  EXPECT_EQ(h.disk(0).flush_calls, 2);

  // A test image needs no -f, and takes one.
  EXPECT_EQ(h.Run({"disk", "-d", "2", "write", "1", "hi"}), 0);
  EXPECT_EQ(h.output(), "wrote 1 block(s) at lba 1\n");
  EXPECT_EQ(h.Run({"disk", "-d", "2", "-f", "zero", "1"}), 0);
  EXPECT_EQ(h.output(), "zeroed 1 block(s) at lba 1\n");

  // Only a serial that starts with "protos" names a test image.
  EXPECT_TRUE(IsBlockTestDiskSerial("protos"));
  EXPECT_TRUE(IsBlockTestDiskSerial("protos0"));
  EXPECT_TRUE(IsBlockTestDiskSerial("protos-scratch"));
  EXPECT_FALSE(IsBlockTestDiskSerial(""));
  EXPECT_FALSE(IsBlockTestDiskSerial("proto"));
  EXPECT_FALSE(IsBlockTestDiskSerial("Protos0"));
  EXPECT_FALSE(IsBlockTestDiskSerial("xprotos0"));
}

}  // namespace
}  // namespace protos::user::disk
