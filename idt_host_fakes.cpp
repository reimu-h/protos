#include "idt_host_fakes.h"

#include <atomic>
#include <cstdint>

#include "idt.h"

static_assert(protos::kIdtEntryCount == 256);

// Stand-in for the table of ISR stub addresses in boot.S.
asm(R"(
    .pushsection .rodata
    .balign 16
    .globl g_isr_stub_table
    .type g_isr_stub_table, @object
    .size g_isr_stub_table, 256 * 8
g_isr_stub_table:
    .set .Lfake_isr_stub_addr, 0x100000
    .rept 256
    .quad .Lfake_isr_stub_addr
    .set .Lfake_isr_stub_addr, .Lfake_isr_stub_addr + 16
    .endr
    .popsection
)");

namespace protos {

namespace {

std::atomic<uint64_t> g_fake_cr2{0};

}  // namespace

// Replaces the kernel's CR2 reader (declared in idt.cpp).
uint64_t ReadCr2() { return g_fake_cr2.load(std::memory_order_relaxed); }

void IdtSetHostCr2ForTest(const uint64_t cr2) {
  g_fake_cr2.store(cr2, std::memory_order_relaxed);
}

}  // namespace protos
