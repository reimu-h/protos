#ifndef PROTOS_VGA_H_
#define PROTOS_VGA_H_

#include <cstdint>

namespace protos {

// Clears the 80x25 console (both the legacy VGA text buffer at 0xB8000 and any
// attached linear pixel framebuffer) and resets the cursor to (0, 0).
void VgaClear();

// Writes a single character to the console at the current cursor position,
// advancing the cursor and scrolling as needed.
void VgaPutc(char ch);

// Writes a null-terminated string to the console at the current cursor
// position, advancing the cursor and scrolling as needed.
void VgaWrite(const char* str);

// Writes a 64-bit unsigned integer in hexadecimal ("0x...") to the console.
void VgaWriteHex(uint64_t value);

// Writes a 64-bit unsigned integer in base-10 decimal to the console.
void VgaWriteDec(uint64_t value);

// Returns the current 0-based cursor row (`0 <= row < 25`) or column
// (`0 <= col < 80`) under `g_vga_lock`.
int VgaGetCursorRow();
int VgaGetCursorCol();

// Console writer for the kernel panic path (idt.cpp); call it only from the
// CPU that owns the panic, with interrupts disabled. It does not take the
// console lock, which a stopped CPU may hold, and works on a clamped copy of
// the cursor.
void VgaPanicWrite(const char* str);

// Switches the console to a direct-color linear pixel framebuffer (such as the
// UEFI GOP framebuffer from Multiboot2, for machines without the legacy 0xB8000
// text mode), identity-maps it, and renders the current 80x25 text onto it with
// an 8x16 font. Afterwards, legacy VGA text memory (0xB8000) and the CRT cursor
// I/O ports (0x3D4/0x3D5) are no longer touched.
//
// Supported framebuffers are 32 or 24 bits per pixel, at least 640x400 pixels
// (80x25 cells), with `pitch >= width * bpp / 8`, and, for 32 bpp, a 4-byte
// aligned address and pitch. Returns false, logs the rejected parameters to the
// serial port, and leaves the console unchanged if the framebuffer is not
// supported or cannot be mapped.
bool VgaInitFramebuffer(uintptr_t fb_phys_addr,  //
                        int pitch,               //
                        int width,               //
                        int height,              //
                        int bpp);

}  // namespace protos

#endif  // PROTOS_VGA_H_
