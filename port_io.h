#ifndef PROTOS_PORT_IO_H_
#define PROTOS_PORT_IO_H_

#include <cstdint>

namespace protos {

// x86 port I/O for kernel code. Each function accesses the port with the width
// of its value type. Never reach a multi-byte register with narrower accesses
// or a byte register with wider ones: for example, a byte write to port
// 0xCF9, inside the 32-bit PCI CONFIG_ADDRESS register at 0xCF8, is the Reset
// Control register of Intel chipsets (and QEMU's PIIX) and reboots the
// machine. User code runs with IOPL 0, where these instructions fault.
//
// Kernel builds only (Intel syntax, -masm=intel): host builds, which compile
// with the AT&T syntax and must not execute port I/O, see no declarations, so
// using these in host code fails to compile.
#if !__STDC_HOSTED__
inline uint8_t PortIn8(const uint16_t port) {
  uint8_t value = 0;
  asm volatile("in %0, %1" : "=a"(value) : "Nd"(port) : "memory");
  return value;
}

inline uint16_t PortIn16(const uint16_t port) {
  uint16_t value = 0;
  asm volatile("in %0, %1" : "=a"(value) : "Nd"(port) : "memory");
  return value;
}

inline uint32_t PortIn32(const uint16_t port) {
  uint32_t value = 0;
  asm volatile("in %0, %1" : "=a"(value) : "Nd"(port) : "memory");
  return value;
}

inline void PortOut8(const uint16_t port, const uint8_t value) {
  asm volatile("out %1, %0" : : "a"(value), "Nd"(port) : "memory");
}

inline void PortOut16(const uint16_t port, const uint16_t value) {
  asm volatile("out %1, %0" : : "a"(value), "Nd"(port) : "memory");
}

inline void PortOut32(const uint16_t port, const uint32_t value) {
  asm volatile("out %1, %0" : : "a"(value), "Nd"(port) : "memory");
}
#endif

}  // namespace protos

#endif  // PROTOS_PORT_IO_H_
