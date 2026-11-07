// hello: sample standalone user program packed into `/bin/hello` in the boot
// filesystem (`build/rootfs.tar`) rather than embedded in the kernel image.

#include <cstdint>

#include "user/syscall.h"

namespace protos::user {
namespace {

int g_initialized_data = 1;

static int CurrentPrivilegeLevel() {
  uint16_t cs = 0;
  asm volatile("mov %0, cs" : "=r"(cs));
  return cs & 3;
}

class Line {
 public:
  Line& Str(const char* const text) {
    for (int64_t i = 0; text[i] != '\0' && length_ < kCapacity; ++i) {
      buffer_[length_++] = text[i];
    }
    return *this;
  }

  int64_t Print() const { return Write(buffer_, length_); }

 private:
  static constexpr int64_t kCapacity = 1088;
  char buffer_[kCapacity] = {};
  int64_t length_ = 0;
};

}  // namespace

int64_t Main(const int argc, char** const argv) {
  if (CurrentPrivilegeLevel() != 3 || g_initialized_data != 1 || argc < 0 ||
      argv == nullptr || argv[argc] != nullptr) {
    return 1;
  }
  for (int i = 0; i < argc; ++i) {
    if (argv[i] == nullptr) {
      return 1;
    }
  }
  g_initialized_data = 2;

  Line line;
  if (argc <= 1) {
    line.Str("hello from /bin/hello\n");
  } else {
    line.Str("hello:");
    for (int i = 1; i < argc; ++i) {
      line.Str(" ").Str(argv[i]);
    }
    line.Str("\n");
  }
  return line.Print() < 0 ? 1 : 0;
}

}  // namespace protos::user
