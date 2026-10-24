#ifndef PROTOS_VGA_H_
#define PROTOS_VGA_H_

#include <cstdint>

namespace protos {

// The console keeps 80x25 cells of text and shows them on the screen that the
// boot loader left: the legacy VGA text buffer at 0xB8000, with the cursor of
// the CRT controller at I/O ports 0x3D4/0x3D5, once VgaEnableLegacyText() has
// reported VGA text mode, or a linear pixel framebuffer that
// VgaInitFramebuffer() attaches, which takes precedence. Until either happens,
// the text exists only in memory: machines without a VGA (UEFI class 3
// systems) need not decode the legacy VGA addresses at all, and a framebuffer
// shows the text written before it was attached.

// Clears the 80x25 console (and the screen that shows it) and resets the cursor
// to (0, 0).
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

// Reports that the screen is in VGA text mode (see MultibootHasVgaTextMode()):
// unless a framebuffer is attached, the console then shows its text in the
// legacy VGA text buffer at 0xB8000 and moves the CRT cursor, starting with
// the text written so far. Call it only when VGA text mode is known to be
// present: other machines may decode those addresses as something else.
void VgaEnableLegacyText();

// Layout of the color channels in a pixel of a direct-color framebuffer: for
// red, green and blue, the bit position of the channel's least significant bit
// and the channel's width in bits (as in the Multiboot color information).
struct VgaPixelFormat {
  int red_position;
  int red_size;
  int green_position;
  int green_size;
  int blue_position;
  int blue_size;
};

// 8 bits per channel with blue in the lowest byte, the layout of most 24- and
// 32-bpp modes (such as the UEFI GOP's PixelBlueGreenRedReserved8BitPerColor).
inline constexpr VgaPixelFormat kVgaDefaultPixelFormat = {16, 8, 8, 8, 0, 8};

// Switches the console to a direct-color linear pixel framebuffer (such as the
// UEFI GOP framebuffer from Multiboot2, for machines without the legacy 0xB8000
// text mode), identity-maps it, and renders the current 80x25 text onto it with
// an 8x16 font, white (every bit of every color channel of `format` set) on
// black (all bits clear). Afterwards, legacy VGA text memory (0xB8000) and the
// CRT cursor I/O ports (0x3D4/0x3D5) are no longer touched.
//
// Supported framebuffers are 32 or 24 bits per pixel, at least 640x400 pixels
// (80x25 cells), with `pitch >= width * bpp / 8`, for 32 bpp a 4-byte aligned
// address and pitch, and color channels that are not empty, lie within the
// pixel and do not overlap. Returns false, logs the rejected parameters to the
// serial port, and leaves the console unchanged if the framebuffer is not
// supported or cannot be mapped.
bool VgaInitFramebuffer(uintptr_t fb_phys_addr,  //
                        int pitch,               //
                        int width,               //
                        int height,              //
                        int bpp,                 //
                        const VgaPixelFormat& format);

// Same as above for a framebuffer with `kVgaDefaultPixelFormat`.
bool VgaInitFramebuffer(uintptr_t fb_phys_addr,  //
                        int pitch,               //
                        int width,               //
                        int height,              //
                        int bpp);

}  // namespace protos

#endif  // PROTOS_VGA_H_
