#include "selftest/selftest.h"

#include "selftest/boot_selftest.h"
#include "selftest/stress.h"

namespace protos {

void RunSelfTests() {
  if (!RunBootSelfTests()) {
    return;
  }
  for (;;) {
    RunRandomMultiCpuStressTest();
  }
}

}  // namespace protos
