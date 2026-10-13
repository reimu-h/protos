// Host fakes for the parts of idt.h and task.h that SetupEnv() and the
// scheduler layers use, for scheduler unit tests that link neither idt.cpp
// nor task.cpp. See scheduler_test_support.h.

#include "idt.h"
#include "scheduler_test_support.h"
#include "task.h"

namespace protos {

void IdtInit() { g_env.idt_initialized = true; }

bool IdtIsInitialized() { return g_env.idt_initialized; }

void IdtResetForTest() { g_env.idt_initialized = false; }

// No TaskInit() exists in these tests, so there is no scheduler to destroy.
void TaskResetForTest() {}

}  // namespace protos
