#ifndef PROTOS_SELFTEST_SELFTEST_H_
#define PROTOS_SELFTEST_SELFTEST_H_

namespace protos {

// Runs the in-kernel self-tests: the boot self-test suite (`RunBootSelfTests()`
// in boot_selftest.h) once and then, if it passed, the randomized multi-CPU
// stress test (`RunRandomMultiCpuStressTest()` in stress.h) in an endless loop
// that keeps every CPU busy. Returns only if the boot self-tests failed.
//
// `kernel_main` calls it in self-test builds (`PROTOS_SELFTEST`, see SELFTEST
// in the Makefile), after `TaskInit()`. The preconditions and side effects of
// the two suites are documented with their entry points.
void RunSelfTests();

}  // namespace protos

#endif  // PROTOS_SELFTEST_SELFTEST_H_
