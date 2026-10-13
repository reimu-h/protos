#include "vga.h"

#include <cstdint>

#include "format.h"
#include "paging.h"
#include "spinlock.h"
#include "uart.h"

namespace protos {

namespace {

constexpr uintptr_t kVgaBufferAddress = 0xB8000;
constexpr int kVgaWidth = 80;
constexpr int kVgaHeight = 25;
constexpr uint8_t kVgaColorWhiteOnBlack = 0x0F;
constexpr int kGlyphWidth = 8;
constexpr int kGlyphHeight = 16;
// Lit glyph pixel in a 32-bpp framebuffer (white; the top byte is unused).
constexpr uint32_t kWhitePixel32 = 0x00FFFFFF;

IrqSpinLock g_vga_lock;
int g_cursor_row = 0;
int g_cursor_col = 0;
char g_text_shadow[kVgaHeight][kVgaWidth];

bool g_fb_active = false;
uintptr_t g_fb_addr = 0;
int g_fb_pitch = 0;
int g_fb_width = 0;
int g_fb_height = 0;
int g_fb_bpp = 0;

// Compact 8x8 bitmap font for ASCII 0x20..0x7E (each uint64_t packs 8 rows of
// 8 horizontal pixels, byte 0 on top, bit 0 on the left; rendered at 2x
// vertical scale -> 8x16).
//
// Derived from font8x8_basic (https://github.com/dhepper/font8x8, public
// domain), which is based on the IBM PC BIOS 8x8 font. Glyphs are moved down
// one row (letters and digits sit on row 7), and many are shifted one pixel
// right or slightly reshaped.
constexpr uint64_t kAsciiFont8x8[95] = {
    0x0000000000000000ULL,  // ' '
    0x180018183C3C3C18ULL,  // '!'
    0x0000000000666666ULL,  // '"'
    0x36367F367F363600ULL,  // '#'
    0x183E603C067C1800ULL,  // '$'
    0x63660C1833630000ULL,  // '%'
    0x6E333B6E1C361C00ULL,  // '&'
    0x0000000000181818ULL,  // '\''
    0x30180C0C0C183000ULL,  // '('
    0x0C18303030180C00ULL,  // ')'
    0x00663CFF3C660000ULL,  // '*'
    0x0018187E18180000ULL,  // '+'
    0x0C18180000000000ULL,  // ','
    0x0000007E00000000ULL,  // '-'
    0x0018180000000000ULL,  // '.'
    0x02060C1830604000ULL,  // '/'
    0x3E676F7B73633E00ULL,  // '0'
    0x7E181818181C1800ULL,  // '1'
    0x7F060C3060663C00ULL,  // '2'
    0x3C66603860663C00ULL,  // '3'
    0x30307F33363C3800ULL,  // '4'
    0x3C6660603E067E00ULL,  // '5'
    0x3C66663E06663C00ULL,  // '6'
    0x1818183060667E00ULL,  // '7'
    0x3C66663C66663C00ULL,  // '8'
    0x3C66607C66663C00ULL,  // '9'
    0x0018180000181800ULL,  // ':'
    0x0C18180000181800ULL,  // ';'
    0x30180C060C183000ULL,  // '<'
    0x00007E007E000000ULL,  // '='
    0x060C1830180C0600ULL,  // '>'
    0x1800183060663C00ULL,  // '?'
    0x3E037B7B7B633E00ULL,  // '@'
    0x66667E66663C1800ULL,  // 'A'
    0x3E66663E66663E00ULL,  // 'B'
    0x3C66060606663C00ULL,  // 'C'
    0x3E66666666663E00ULL,  // 'D'
    0x7E06063E06067E00ULL,  // 'E'
    0x0606063E06067E00ULL,  // 'F'
    0x7C66760606663C00ULL,  // 'G'
    0x6666667E66666600ULL,  // 'H'
    0x3C18181818183C00ULL,  // 'I'
    0x1E33333030307800ULL,  // 'J'
    0x66361E0E1E366600ULL,  // 'K'
    0x7E06060606060600ULL,  // 'L'
    0x6363636B7F776300ULL,  // 'M'
    0x6363737B6F676300ULL,  // 'N'
    0x3C66666666663C00ULL,  // 'O'
    0x06063E6666663E00ULL,  // 'P'
    0x603C766666663C00ULL,  // 'Q'
    0x66361E3E66663E00ULL,  // 'R'
    0x3C66603C06663C00ULL,  // 'S'
    0x1818181818187E00ULL,  // 'T'
    0x3C66666666666600ULL,  // 'U'
    0x183C666666666600ULL,  // 'V'
    0x63777F6B63636300ULL,  // 'W'
    0x66663C183C666600ULL,  // 'X'
    0x1818183C66666600ULL,  // 'Y'
    0x7E060C1830607E00ULL,  // 'Z'
    0x3C0C0C0C0C0C3C00ULL,  // '['
    0x406030180C060200ULL,  // '\\'
    0x3C30303030303C00ULL,  // ']'
    0x0000000063361C00ULL,  // '^'
    0x007F000000000000ULL,  // '_'
    0x0000000000180C0CULL,  // '`'
    0x7C667C603C000000ULL,  // 'a'
    0x3E6666663E060600ULL,  // 'b'
    0x3C6606663C000000ULL,  // 'c'
    0x7C6666667C606000ULL,  // 'd'
    0x3C067E663C000000ULL,  // 'e'
    0x1C18187C18381800ULL,  // 'f'
    0x3C607C66667C0000ULL,  // 'g'
    0x666666663E060600ULL,  // 'h'
    0x3C1818181C001800ULL,  // 'i'
    0x1C36303030380030ULL,  // 'j'
    0x66361E3666060600ULL,  // 'k'
    0x3C18181818181C00ULL,  // 'l'
    0x6B6B7F7763000000ULL,  // 'm'
    0x666666663E000000ULL,  // 'n'
    0x3C6666663C000000ULL,  // 'o'
    0x06063E66663E0000ULL,  // 'p'
    0x60607C66667C0000ULL,  // 'q'
    0x060606663B000000ULL,  // 'r'
    0x3E603C067C000000ULL,  // 's'
    0x381818187C181800ULL,  // 't'
    0x7C66666666000000ULL,  // 'u'
    0x183C666666000000ULL,  // 'v'
    0x367F6B6363000000ULL,  // 'w'
    0x663C183C66000000ULL,  // 'x'
    0x3C607C6666000000ULL,  // 'y'
    0x7E0C18307E000000ULL,  // 'z'
    0x3818180C18183800ULL,  // '{'
    0x1818180018181800ULL,  // '|'
    0x0E18183018180E00ULL,  // '}'
    0x0000003B6E000000ULL,  // '~'
};

static void RenderCellToFramebuffer(const int row,  //
                                    const int col,  //
                                    const char ch) {
  if (!g_fb_active || g_fb_addr == 0 || (g_fb_bpp != 32 && g_fb_bpp != 24)) {
    return;
  }
  const int base_x = col * kGlyphWidth;
  const int base_y = row * kGlyphHeight;
  if (base_x + kGlyphWidth > g_fb_width ||
      base_y + kGlyphHeight > g_fb_height) {
    return;
  }

  const uint8_t u = static_cast<uint8_t>(ch);
  const uint64_t glyph =
      (u >= 0x20 && u <= 0x7E) ? kAsciiFont8x8[u - 0x20] : 0ULL;

  volatile uint8_t* const fb_base =
      reinterpret_cast<volatile uint8_t*>(g_fb_addr);

  for (int py = 0; py < kGlyphHeight; ++py) {
    const uint8_t row_bits = (glyph >> ((py / 2) * 8)) & 0xFF;
    volatile uint8_t* const line =
        fb_base + static_cast<int64_t>(base_y + py) * g_fb_pitch;
    if (g_fb_bpp == 32) {
      // One aligned 32-bit store per pixel (VgaInitFramebuffer checks the
      // alignment).
      volatile uint32_t* const pixels =
          reinterpret_cast<volatile uint32_t*>(line) + base_x;
      for (int px = 0; px < kGlyphWidth; ++px) {
        pixels[px] = ((row_bits >> px) & 1u) != 0 ? kWhitePixel32 : 0u;
      }
    } else {
      volatile uint8_t* const pixels = line + base_x * 3;
      for (int px = 0; px < kGlyphWidth; ++px) {
        const uint8_t intensity = ((row_bits >> px) & 1u) != 0 ? 0xFF : 0x00;
        pixels[px * 3 + 0] = intensity;
        pixels[px * 3 + 1] = intensity;
        pixels[px * 3 + 2] = intensity;
      }
    }
  }
}

static void Outb(const uint16_t port, const uint8_t value) {
  // `{AT&T|Intel}` dialect alternatives: the kernel is compiled with
  // -masm=intel, while host tests use GCC's default AT&T syntax.
  asm volatile("out{b %0, %w1| %w1, %0}" : : "a"(value), "Nd"(port));
}

// Returns true if the 80x25 console fits on the described framebuffer and
// RenderCellToFramebuffer can draw on it (see VgaInitFramebuffer in vga.h).
static bool IsSupportedFramebuffer(const uintptr_t addr,  //
                                   const int pitch,       //
                                   const int width,       //
                                   const int height,      //
                                   const int bpp) {
  if (addr == 0 || (bpp != 32 && bpp != 24) ||
      width < kVgaWidth * kGlyphWidth || height < kVgaHeight * kGlyphHeight ||
      static_cast<int64_t>(pitch) < static_cast<int64_t>(width) * (bpp / 8)) {
    return false;
  }
  if (bpp == 32 && ((addr % 4) != 0 || (pitch % 4) != 0)) {
    return false;
  }
  const uint64_t fb_bytes = static_cast<uint64_t>(pitch) * height;
  return addr < kMaxCanonicalIdentityAddress &&
         fb_bytes <= kMaxCanonicalIdentityAddress - addr;
}

static void UartWriteInt(const int value) {
  if (value < 0) {
    UartWrite("-");
    UartWriteDec(static_cast<uint64_t>(-static_cast<int64_t>(value)));
  } else {
    UartWriteDec(static_cast<uint64_t>(value));
  }
}

static void UpdateCursor() {
  if (g_fb_active) {
    return;
  }
  const uint16_t pos =
      static_cast<uint16_t>(g_cursor_row * kVgaWidth + g_cursor_col);
  Outb(0x3D4, 0x0F);
  Outb(0x3D5, static_cast<uint8_t>(pos & 0xFFu));
  Outb(0x3D4, 0x0E);
  Outb(0x3D5, static_cast<uint8_t>((pos >> 8) & 0xFFu));
}

static void WriteCell(const int row, const int col, const char ch) {
  if (g_fb_active) {
    // The framebuffer always shows g_text_shadow (VgaInitFramebuffer renders
    // all of it). Redrawing a cell takes 128 pixel stores to memory that is
    // often uncached, so cells that already show `ch` are skipped, which makes
    // scrolling much cheaper.
    if (g_text_shadow[row][col] != ch) {
      g_text_shadow[row][col] = ch;
      RenderCellToFramebuffer(row, col, ch);
    }
    return;
  }
  g_text_shadow[row][col] = ch;
  volatile uint16_t* const vga =
      reinterpret_cast<volatile uint16_t*>(kVgaBufferAddress);
  vga[row * kVgaWidth + col] =
      (static_cast<uint16_t>(kVgaColorWhiteOnBlack) << 8) |
      static_cast<uint8_t>(ch);
}

static void ScrollUpOneRow() {
  for (int r = 0; r + 1 < kVgaHeight; ++r) {
    for (int c = 0; c < kVgaWidth; ++c) {
      WriteCell(r, c, g_text_shadow[r + 1][c]);
    }
  }
  for (int c = 0; c < kVgaWidth; ++c) {
    WriteCell(kVgaHeight - 1, c, ' ');
  }
}

static void VgaPutcLocked(const char ch) {
  if (g_cursor_row < 0 || g_cursor_row >= kVgaHeight) {
    g_cursor_row = kVgaHeight - 1;
  }
  if (g_cursor_col < 0 || g_cursor_col >= kVgaWidth) {
    g_cursor_col = 0;
  }
  if (ch == '\r') {
    g_cursor_col = 0;
    return;
  }
  if (ch == '\n') {
    g_cursor_col = 0;
    ++g_cursor_row;
    if (g_cursor_row >= kVgaHeight) {
      ScrollUpOneRow();
      g_cursor_row = kVgaHeight - 1;
    }
    return;
  }
  if (ch == '\t') {
    const int spaces = 8 - (g_cursor_col & 7);
    for (int i = 0; i < spaces; ++i) {
      WriteCell(g_cursor_row, g_cursor_col, ' ');
      ++g_cursor_col;
      if (g_cursor_col >= kVgaWidth) {
        g_cursor_col = 0;
        ++g_cursor_row;
        if (g_cursor_row >= kVgaHeight) {
          ScrollUpOneRow();
          g_cursor_row = kVgaHeight - 1;
        }
        break;
      }
    }
    return;
  }
  WriteCell(g_cursor_row, g_cursor_col, ch);
  ++g_cursor_col;
  if (g_cursor_col >= kVgaWidth) {
    g_cursor_col = 0;
    ++g_cursor_row;
    if (g_cursor_row >= kVgaHeight) {
      ScrollUpOneRow();
      g_cursor_row = kVgaHeight - 1;
    }
  }
}

static void VgaWriteLocked(const char* const str) {
  if (str == nullptr) {
    return;
  }
  for (int i = 0; str[i] != '\0'; ++i) {
    VgaPutcLocked(str[i]);
  }
  UpdateCursor();
}

}  // namespace

void VgaClear() {
  const IrqSpinLockGuard lock_guard(g_vga_lock);
  g_cursor_row = 0;
  g_cursor_col = 0;
  for (int r = 0; r < kVgaHeight; ++r) {
    for (int c = 0; c < kVgaWidth; ++c) {
      WriteCell(r, c, ' ');
    }
  }
  UpdateCursor();
}

void VgaPutc(const char ch) {
  const IrqSpinLockGuard lock_guard(g_vga_lock);
  VgaPutcLocked(ch);
  UpdateCursor();
}

void VgaWrite(const char* const str) {
  const IrqSpinLockGuard lock_guard(g_vga_lock);
  VgaWriteLocked(str);
}

void VgaWriteHex(const uint64_t value) {
  char buffer[kFormatHexBufferSize];
  VgaWrite(FormatHex(value, buffer));
}

void VgaWriteDec(const uint64_t value) {
  char buffer[kFormatDecBufferSize];
  VgaWrite(FormatDec(value, buffer));
}

int VgaGetCursorRow() {
  const IrqSpinLockGuard lock_guard(g_vga_lock);
  return g_cursor_row;
}

int VgaGetCursorCol() {
  const IrqSpinLockGuard lock_guard(g_vga_lock);
  return g_cursor_col;
}

// Panic-path counterpart of VgaPutcLocked() that advances the caller's copy of
// the cursor instead of the shared one.
static void PanicPutcAt(const char ch, int* const row, int* const col) {
  if (ch == '\r') {
    *col = 0;
    return;
  }
  if (ch != '\n') {
    WriteCell(*row, *col, (ch == '\t') ? ' ' : ch);
    ++*col;
    if (*col < kVgaWidth) {
      return;
    }
  }
  *col = 0;
  if (*row + 1 < kVgaHeight) {
    ++*row;
  } else {
    ScrollUpOneRow();
  }
}

void VgaPanicWrite(const char* const str) {
  if (str == nullptr) {
    return;
  }
  // Read the shared cursor once and clamp it: a CPU stopped in the middle of a
  // locked write may have left it out of range.
  int row = *static_cast<const volatile int*>(&g_cursor_row);
  int col = *static_cast<const volatile int*>(&g_cursor_col);
  if (row < 0 || row >= kVgaHeight) {
    row = kVgaHeight - 1;
  }
  if (col < 0 || col >= kVgaWidth) {
    col = 0;
  }
  for (int i = 0; str[i] != '\0'; ++i) {
    PanicPutcAt(str[i], &row, &col);
  }
  g_cursor_row = row;
  g_cursor_col = col;
  UpdateCursor();
}

bool VgaInitFramebuffer(const uintptr_t fb_phys_addr,  //
                        const int pitch,               //
                        const int width,               //
                        const int height,              //
                        const int bpp) {
  const char* rejection = nullptr;
  if (!IsSupportedFramebuffer(fb_phys_addr, pitch, width, height, bpp)) {
    rejection = "unsupported";
  } else if (!PagingMapBootstrapRange(fb_phys_addr,
                                      static_cast<int64_t>(pitch) * height)) {
    rejection = "unmappable";
  }
  if (rejection != nullptr) {
    // The serial port is the only console that is sure to be visible here.
    UartWrite("[VGA] Ignoring ");
    UartWrite(rejection);
    UartWrite(" framebuffer: addr=");
    UartWriteHex(fb_phys_addr);
    UartWrite(" pitch=");
    UartWriteInt(pitch);
    UartWrite(" width=");
    UartWriteInt(width);
    UartWrite(" height=");
    UartWriteInt(height);
    UartWrite(" bpp=");
    UartWriteInt(bpp);
    UartWrite("\n");
    return false;
  }

  const IrqSpinLockGuard lock_guard(g_vga_lock);
  g_fb_addr = fb_phys_addr;
  g_fb_pitch = pitch;
  g_fb_width = width;
  g_fb_height = height;
  g_fb_bpp = bpp;
  g_fb_active = true;

  for (int r = 0; r < kVgaHeight; ++r) {
    for (int c = 0; c < kVgaWidth; ++c) {
      if (g_text_shadow[r][c] == '\0') {
        g_text_shadow[r][c] = ' ';
      }
      RenderCellToFramebuffer(r, c, g_text_shadow[r][c]);
    }
  }
  return true;
}

}  // namespace protos
