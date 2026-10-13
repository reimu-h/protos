#ifndef PROTOS_IDT_HOST_FAKES_H_
#define PROTOS_IDT_HOST_FAKES_H_

#include <cstdint>

// Host-test stand-ins for the kernel-only symbols that idt.cpp uses. Every host
// test binary that links idt.cpp also links idt_host_fakes.cpp, which defines:
// - `g_isr_stub_table` (boot.S in the kernel): 256 distinct, non-zero addresses
//   spaced 16 bytes apart like the real ISR stubs.
// - `ReadCr2()`: returns the value last passed to `IdtSetHostCr2ForTest()`
//   (initially 0).

namespace protos {

// Sets the page-fault address that the fake `ReadCr2()` returns, e.g. for the
// `cr2=` field of the unhandled-exception panic message.
void IdtSetHostCr2ForTest(uint64_t cr2);

}  // namespace protos

#endif  // PROTOS_IDT_HOST_FAKES_H_
