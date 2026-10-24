#include "vga.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "paging.h"
#include "uart.h"

namespace protos {
namespace {

namespace t = ::testing;

struct FakeVgaEnv {
  bool map_ok = true;
  int map_calls = 0;
  uintptr_t last_map_addr = 0;
  int64_t last_map_size = 0;
  std::string uart_log;
};

FakeVgaEnv g_env;

static void AppendFormatted(const char* const format, const uint64_t value) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), format,
                static_cast<unsigned long long>(value));
  g_env.uart_log.append(buf);
}

}  // namespace

bool PagingMapBootstrapRange(const uintptr_t phys_addr, const int64_t size) {
  ++g_env.map_calls;
  g_env.last_map_addr = phys_addr;
  g_env.last_map_size = size;
  return g_env.map_ok;
}

void UartInit() {}

void UartPutc(const char c) { g_env.uart_log.push_back(c); }

void UartWrite(const char* const str) { g_env.uart_log.append(str); }

void UartWriteHex(const uint64_t value) { AppendFormatted("0x%llX", value); }

void UartWriteDec(const uint64_t value) { AppendFormatted("%llu", value); }

void UartPanicWrite(const char* const str) { UartWrite(str); }

namespace {

constexpr int kConsoleCols = 80;
constexpr int kConsoleRows = 25;
constexpr int kGlyphWidth = 8;
constexpr int kGlyphHeight = 16;
constexpr int kFbWidth = kConsoleCols * kGlyphWidth;
constexpr int kFbHeight = kConsoleRows * kGlyphHeight;
// Rows are padded past the visible pixels so that pitch handling is tested.
constexpr int kPitch32 = kFbWidth * 4 + 64;
constexpr int kPitch24 = kFbWidth * 3 + 12;
constexpr int64_t kFb32Bytes = static_cast<int64_t>(kPitch32) * kFbHeight;
constexpr int64_t kFb24Bytes = static_cast<int64_t>(kPitch24) * kFbHeight;
constexpr uint8_t kUntouchedByte = 0xAA;

// Host-memory framebuffers. They have static storage because the console keeps
// drawing into the last attached framebuffer after a test ends; every test
// attaches one of them before writing anything (except the first one, which
// writes before any framebuffer is attached).
alignas(64) uint8_t g_fb32_a[kFb32Bytes];
alignas(64) uint8_t g_fb32_b[kFb32Bytes];
alignas(64) uint8_t g_fb24[kFb24Bytes];
alignas(64) uint8_t g_sentinel[kFb32Bytes];

struct Framebuffer {
  uint8_t* bytes;
  int pitch;
  int bpp;

  uintptr_t addr() const { return reinterpret_cast<uintptr_t>(bytes); }
  int64_t size() const { return static_cast<int64_t>(pitch) * kFbHeight; }
};

constexpr Framebuffer kFb32A = {g_fb32_a, kPitch32, 32};
constexpr Framebuffer kFb32B = {g_fb32_b, kPitch32, 32};
constexpr Framebuffer kFb24 = {g_fb24, kPitch24, 24};

// Fills `fb` with kUntouchedByte, attaches the console to it and clears the
// console.
static void AttachAndClear(const Framebuffer& fb) {
  std::memset(fb.bytes, kUntouchedByte, fb.size());
  ASSERT_TRUE(VgaInitFramebuffer(fb.addr(),  //
                                 fb.pitch,   //
                                 kFbWidth,   //
                                 kFbHeight,  //
                                 fb.bpp));
  VgaClear();
}

// Returns the 8 glyph rows drawn in console cell (`row`, `col`) of `fb`, top
// first, as strings of '#' (white) and '.' (black) pixels, leftmost pixel
// first. Any other pixel value shows up as '?'. Each glyph row covers two
// framebuffer lines, which must be identical.
static std::vector<std::string> CellRows(const Framebuffer& fb,  //
                                         const int row,          //
                                         const int col) {
  const int bytes_per_pixel = fb.bpp / 8;
  std::vector<std::string> rows;
  for (int glyph_row = 0; glyph_row < kGlyphHeight / 2; ++glyph_row) {
    std::string lines[2];
    for (int half = 0; half < 2; ++half) {
      const int y = row * kGlyphHeight + glyph_row * 2 + half;
      const uint8_t* const line = fb.bytes + static_cast<int64_t>(y) * fb.pitch;
      for (int px = 0; px < kGlyphWidth; ++px) {
        const uint8_t* const pixel =
            line + (col * kGlyphWidth + px) * bytes_per_pixel;
        const bool unused_byte_clear = bytes_per_pixel == 3 || pixel[3] == 0;
        if (pixel[0] == 0xFF && pixel[1] == 0xFF && pixel[2] == 0xFF &&
            unused_byte_clear) {
          lines[half].push_back('#');
        } else if (pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0 &&
                   unused_byte_clear) {
          lines[half].push_back('.');
        } else {
          lines[half].push_back('?');
        }
      }
    }
    EXPECT_THAT(lines[1], t::Eq(lines[0]))
        << "cell (" << row << ", " << col << ") glyph row " << glyph_row;
    rows.push_back(lines[0]);
  }
  return rows;
}

// Returns true if the padding bytes past the visible pixels of every line of
// `fb` still hold kUntouchedByte.
static bool RowPaddingUntouched(const Framebuffer& fb) {
  const int visible_bytes = kFbWidth * (fb.bpp / 8);
  for (int y = 0; y < kFbHeight; ++y) {
    const uint8_t* const line = fb.bytes + static_cast<int64_t>(y) * fb.pitch;
    for (int x = visible_bytes; x < fb.pitch; ++x) {
      if (line[x] != kUntouchedByte) {
        return false;
      }
    }
  }
  return true;
}

// Returns the distinct pixel values in console cell (`row`, `col`) of `fb`
// (little-endian, 3 or 4 bytes per pixel).
static std::set<uint32_t> CellPixelValues(const Framebuffer& fb,  //
                                          const int row,          //
                                          const int col) {
  const int bytes_per_pixel = fb.bpp / 8;
  std::set<uint32_t> values;
  for (int y = row * kGlyphHeight; y < (row + 1) * kGlyphHeight; ++y) {
    const uint8_t* const line = fb.bytes + static_cast<int64_t>(y) * fb.pitch;
    for (int x = col * kGlyphWidth; x < (col + 1) * kGlyphWidth; ++x) {
      uint32_t value = 0;
      std::memcpy(&value, line + x * bytes_per_pixel, bytes_per_pixel);
      values.insert(value);
    }
  }
  return values;
}

static void ResetEnv() { g_env = FakeVgaEnv{}; }

// Runs first, before any test attaches a framebuffer: until the console knows
// its screen, text goes only to its memory (writing to the legacy VGA text
// buffer at 0xB8000 or the CRT ports would crash this host process), and a
// framebuffer attached later shows it.
TEST(VgaTest, TextWrittenBeforeTheScreenIsKnownShowsOnTheFramebufferLater) {
  ResetEnv();
  VgaClear();
  VgaWrite("$%0F");
  EXPECT_THAT(VgaGetCursorCol(), t::Eq(4));

  std::memset(g_fb32_b, kUntouchedByte, sizeof(g_fb32_b));
  ASSERT_TRUE(VgaInitFramebuffer(kFb32B.addr(),  //
                                 kPitch32,       //
                                 kFbWidth,       //
                                 kFbHeight,      //
                                 32));
  EXPECT_THAT(CellRows(kFb32B, 0, 3), t::Contains(".######."));
  EXPECT_THAT(CellRows(kFb32B, 0, 4), t::Each(t::Eq("........")));
  EXPECT_THAT(CellRows(kFb32B, 1, 0), t::Each(t::Eq("........")));
}

TEST(VgaTest, GlyphsAreDrawnTopRowFirstWithBitZeroOnTheLeft) {
  ResetEnv();
  AttachAndClear(kFb32A);
  VgaWrite("$%0F");

  // Vertically and horizontally asymmetric glyphs, as they must appear on
  // screen.
  EXPECT_THAT(CellRows(kFb32A, 0, 0), t::ElementsAre("........",  //
                                                     "...##...",  //
                                                     "..#####.",  //
                                                     ".##.....",  //
                                                     "..####..",  //
                                                     ".....##.",  //
                                                     ".#####..",  //
                                                     "...##..."));
  EXPECT_THAT(CellRows(kFb32A, 0, 1), t::ElementsAre("........",  //
                                                     "........",  //
                                                     "##...##.",  //
                                                     "##..##..",  //
                                                     "...##...",  //
                                                     "..##....",  //
                                                     ".##..##.",  //
                                                     "##...##."));
  EXPECT_THAT(CellRows(kFb32A, 0, 2), t::ElementsAre("........",  //
                                                     ".#####..",  //
                                                     "##...##.",  //
                                                     "##..###.",  //
                                                     "##.####.",  //
                                                     "####.##.",  //
                                                     "###..##.",  //
                                                     ".#####.."));
  EXPECT_THAT(CellRows(kFb32A, 0, 3), t::ElementsAre("........",  //
                                                     ".######.",  //
                                                     ".##.....",  //
                                                     ".##.....",  //
                                                     ".#####..",  //
                                                     ".##.....",  //
                                                     ".##.....",  //
                                                     ".##....."));
  EXPECT_THAT(CellRows(kFb32A, 0, 4), t::Each(t::Eq("........")));
  EXPECT_THAT(VgaGetCursorRow(), t::Eq(0));
  EXPECT_THAT(VgaGetCursorCol(), t::Eq(4));
  EXPECT_TRUE(RowPaddingUntouched(kFb32A));
}

TEST(VgaTest, RendersOnto24BitFramebuffers) {
  ResetEnv();
  AttachAndClear(kFb24);
  VgaWrite("\n 0");

  EXPECT_THAT(CellRows(kFb24, 0, 1), t::Each(t::Eq("........")));
  EXPECT_THAT(CellRows(kFb24, 1, 1), t::ElementsAre("........",  //
                                                    ".#####..",  //
                                                    "##...##.",  //
                                                    "##..###.",  //
                                                    "##.####.",  //
                                                    "####.##.",  //
                                                    "###..##.",  //
                                                    ".#####.."));
  EXPECT_TRUE(RowPaddingUntouched(kFb24));
}

TEST(VgaTest, DrawsWhiteWithEveryChannelBitOfThePixelFormat) {
  struct FormatCase {
    const char* name;
    Framebuffer fb;
    VgaPixelFormat format;
    uint32_t white;
  };
  const FormatCase kCases[] = {
      // The usual layouts: white has all 24 color bits set either way.
      {"32-bpp BGRX", kFb32A, {16, 8, 8, 8, 0, 8}, 0x00FFFFFF},
      {"32-bpp RGBX", kFb32A, {0, 8, 8, 8, 16, 8}, 0x00FFFFFF},
      {"24-bpp RGB", kFb24, {0, 8, 8, 8, 16, 8}, 0x00FFFFFF},
      // A GOP bitmask mode with the reserved byte at the bottom: white must
      // leave it clear and set the top byte.
      {"32-bpp XBGR", kFb32A, {24, 8, 16, 8, 8, 8}, 0xFFFFFF00},
      {"32-bpp 10:10:10", kFb32A, {20, 10, 10, 10, 0, 10}, 0x3FFFFFFF},
      {"24-bpp 6:6:6", kFb24, {12, 6, 6, 6, 0, 6}, 0x0003FFFF},
  };
  for (const FormatCase& format_case : kCases) {
    ResetEnv();
    const Framebuffer& fb = format_case.fb;
    std::memset(fb.bytes, kUntouchedByte, fb.size());
    ASSERT_TRUE(VgaInitFramebuffer(fb.addr(),  //
                                   fb.pitch,   //
                                   kFbWidth,   //
                                   kFbHeight,  //
                                   fb.bpp,     //
                                   format_case.format))
        << format_case.name;
    VgaClear();
    VgaWrite("#");
    EXPECT_THAT(CellPixelValues(fb, 0, 0),
                t::ElementsAre(0u, format_case.white))
        << format_case.name;
    EXPECT_THAT(CellPixelValues(fb, 0, 1), t::ElementsAre(0u))
        << format_case.name;
    EXPECT_TRUE(RowPaddingUntouched(fb)) << format_case.name;
  }
}

TEST(VgaTest, InitFramebufferRendersCurrentTextAndMapsWholeFramebuffer) {
  ResetEnv();
  AttachAndClear(kFb32A);
  VgaWrite("Hello\nWorld");

  std::memset(g_fb32_b, kUntouchedByte, sizeof(g_fb32_b));
  g_env.map_calls = 0;
  ASSERT_TRUE(VgaInitFramebuffer(kFb32B.addr(),  //
                                 kPitch32,       //
                                 kFbWidth,       //
                                 kFbHeight,      //
                                 32));
  EXPECT_THAT(g_env.map_calls, t::Eq(1));
  EXPECT_THAT(g_env.last_map_addr, t::Eq(kFb32B.addr()));
  EXPECT_THAT(g_env.last_map_size, t::Eq(kFb32Bytes));
  EXPECT_THAT(g_env.uart_log, t::IsEmpty());
  EXPECT_THAT(std::memcmp(g_fb32_a, g_fb32_b, sizeof(g_fb32_a)), t::Eq(0));

  // Further output goes only to the newly attached framebuffer.
  VgaPutc('!');
  EXPECT_THAT(CellRows(kFb32B, 1, 5), t::Not(t::Each(t::Eq("........"))));
  EXPECT_THAT(CellRows(kFb32A, 1, 5), t::Each(t::Eq("........")));
}

TEST(VgaTest, InitFramebufferRejectsUnsupportedFramebuffersWithoutSideEffects) {
  ResetEnv();
  AttachAndClear(kFb32A);
  std::memset(g_sentinel, 0x55, sizeof(g_sentinel));
  const uintptr_t sentinel = reinterpret_cast<uintptr_t>(g_sentinel);

  struct BadFramebuffer {
    const char* name;
    uintptr_t addr;
    int pitch;
    int width;
    int height;
    int bpp;
  };
  const BadFramebuffer kBadFramebuffers[] = {
      {"null address", 0, kFbWidth * 4, kFbWidth, kFbHeight, 32},
      {"16 bpp", sentinel, kFbWidth * 2, kFbWidth, kFbHeight, 16},
      {"8 bpp", sentinel, kFbWidth, kFbWidth, kFbHeight, 8},
      {"narrower than 80 cells", sentinel, kFbWidth * 4, kFbWidth - 1,
       kFbHeight, 32},
      {"zero width", sentinel, kFbWidth * 4, 0, kFbHeight, 32},
      {"shorter than 25 cells", sentinel, kFbWidth * 4, kFbWidth, kFbHeight - 1,
       32},
      {"negative height", sentinel, kFbWidth * 4, kFbWidth, -kFbHeight, 32},
      {"32-bpp pitch below width", sentinel, kFbWidth * 4 - 4, kFbWidth,
       kFbHeight, 32},
      {"24-bpp pitch below width", sentinel, kFbWidth * 3 - 1, kFbWidth,
       kFbHeight, 24},
      {"negative pitch", sentinel, -kFbWidth * 4, kFbWidth, kFbHeight, 32},
      {"misaligned 32-bpp address", sentinel + 2, kFbWidth * 4, kFbWidth,
       kFbHeight, 32},
      {"misaligned 32-bpp pitch", sentinel, kFbWidth * 4 + 2, kFbWidth,
       kFbHeight, 32},
      {"address beyond the identity map", kIdentityMapLimit, kFbWidth * 4,
       kFbWidth, kFbHeight, 32},
      {"end past the identity map limit", kIdentityMapLimit - 4096,
       kFbWidth * 4, kFbWidth, kFbHeight, 32},
  };
  for (const BadFramebuffer& bad : kBadFramebuffers) {
    g_env.uart_log.clear();
    EXPECT_FALSE(VgaInitFramebuffer(bad.addr,    //
                                    bad.pitch,   //
                                    bad.width,   //
                                    bad.height,  //
                                    bad.bpp))
        << bad.name;
    EXPECT_THAT(g_env.uart_log,
                t::StartsWith("[VGA] Ignoring unsupported framebuffer: addr="))
        << bad.name;
  }
  EXPECT_THAT(g_env.uart_log,
              t::HasSubstr(" pitch=2560 width=640 height=400 bpp=32 "
                           "size=8:8:8 shift=16:8:0\n"));
  EXPECT_THAT(g_env.map_calls, t::Eq(1));  // AttachAndClear only.

  const std::vector<uint8_t> expected_sentinel(sizeof(g_sentinel), 0x55);
  EXPECT_THAT(
      std::memcmp(g_sentinel, expected_sentinel.data(), sizeof(g_sentinel)),
      t::Eq(0));

  // The console still draws on the framebuffer that was attached before.
  VgaPutc('F');
  EXPECT_THAT(CellRows(kFb32A, 0, 0), t::Contains(".######."));
}

TEST(VgaTest, InitFramebufferRejectsUnsupportedPixelFormats) {
  ResetEnv();
  AttachAndClear(kFb32A);
  std::memset(g_sentinel, 0x55, sizeof(g_sentinel));
  const uintptr_t sentinel = reinterpret_cast<uintptr_t>(g_sentinel);

  struct BadFormat {
    const char* name;
    int bpp;
    VgaPixelFormat format;
  };
  const BadFormat kBadFormats[] = {
      {"empty channel", 32, {16, 8, 8, 0, 0, 8}},
      {"negative size", 32, {16, 8, 8, -8, 0, 8}},
      {"negative position", 32, {16, 8, 8, 8, -1, 8}},
      {"overlapping channels", 32, {16, 8, 8, 9, 0, 8}},
      {"channel past 32 bits", 32, {28, 8, 8, 8, 0, 8}},
      {"channel past 24 bits", 24, {24, 8, 8, 8, 0, 8}},
      {"position past the pixel", 32, {255, 8, 8, 8, 0, 8}},
      {"no color information", 32, {0, 0, 0, 0, 0, 0}},
  };
  for (const BadFormat& bad : kBadFormats) {
    g_env.uart_log.clear();
    EXPECT_FALSE(VgaInitFramebuffer(sentinel,      //
                                    kFbWidth * 4,  //
                                    kFbWidth,      //
                                    kFbHeight,     //
                                    bad.bpp,       //
                                    bad.format))
        << bad.name;
    EXPECT_THAT(g_env.uart_log,
                t::StartsWith("[VGA] Ignoring unsupported framebuffer: addr="))
        << bad.name;
  }
  EXPECT_THAT(g_env.uart_log, t::EndsWith(" size=0:0:0 shift=0:0:0\n"));
  EXPECT_THAT(g_env.map_calls, t::Eq(1));  // AttachAndClear only.
  const std::vector<uint8_t> expected_sentinel(sizeof(g_sentinel), 0x55);
  EXPECT_THAT(
      std::memcmp(g_sentinel, expected_sentinel.data(), sizeof(g_sentinel)),
      t::Eq(0));

  VgaPutc('F');
  EXPECT_THAT(CellRows(kFb32A, 0, 0), t::Contains(".######."));
}

TEST(VgaTest, InitFramebufferRejectsFramebuffersThatCannotBeMapped) {
  ResetEnv();
  AttachAndClear(kFb32A);
  std::memset(g_fb32_b, kUntouchedByte, sizeof(g_fb32_b));
  g_env.map_ok = false;
  g_env.uart_log.clear();

  EXPECT_FALSE(VgaInitFramebuffer(kFb32B.addr(),  //
                                  kPitch32,       //
                                  kFbWidth,       //
                                  kFbHeight,      //
                                  32));
  EXPECT_THAT(g_env.uart_log,
              t::StartsWith("[VGA] Ignoring unmappable framebuffer: addr=0x"));
  const std::vector<uint8_t> untouched(sizeof(g_fb32_b), kUntouchedByte);
  EXPECT_THAT(std::memcmp(g_fb32_b, untouched.data(), sizeof(g_fb32_b)),
              t::Eq(0));

  VgaPutc('F');
  EXPECT_THAT(CellRows(kFb32A, 0, 0), t::Contains(".######."));
}

TEST(VgaTest, ScrolledFramebufferMatchesAFreshRenderOfTheText) {
  ResetEnv();
  AttachAndClear(kFb32A);

  // Lines of different lengths and contents, so that scrolling moves many
  // different glyphs over each other, plus a line that wraps.
  for (int i = 0; i < 60; ++i) {
    std::string line = "line " + std::to_string(i) + ":";
    for (int j = 0; j < (i * 7) % 50; ++j) {
      line.push_back(static_cast<char>('!' + (i * 13 + j * 5) % 94));
    }
    if (i == 45) {
      line.append(kConsoleCols, '#');
    }
    line.push_back('\n');
    VgaWrite(line.c_str());
  }
  VgaWrite("\ttab\r$%0");
  EXPECT_THAT(VgaGetCursorRow(), t::Eq(kConsoleRows - 1));
  EXPECT_THAT(VgaGetCursorCol(), t::Eq(3));

  // VgaInitFramebuffer redraws every cell from the console text, so after
  // scrolling (which redraws only the cells that change) both framebuffers
  // must be identical.
  std::memset(g_fb32_b, kUntouchedByte, sizeof(g_fb32_b));
  ASSERT_TRUE(VgaInitFramebuffer(kFb32B.addr(),  //
                                 kPitch32,       //
                                 kFbWidth,       //
                                 kFbHeight,      //
                                 32));
  EXPECT_THAT(std::memcmp(g_fb32_a, g_fb32_b, sizeof(g_fb32_a)), t::Eq(0));
  EXPECT_THAT(CellRows(kFb32B, kConsoleRows - 1, 2), t::Contains("##.####."));
}

TEST(VgaTest, ConcurrentWritersKeepTheCursorInBounds) {
  ResetEnv();
  AttachAndClear(kFb32A);

  constexpr int kThreadCount = 4;
  constexpr int kLinesPerThread = 100;
  std::vector<std::thread> writers;
  writers.reserve(kThreadCount);
  for (int i = 0; i < kThreadCount; ++i) {
    writers.emplace_back([i]() {
      const std::string line = "writer " + std::to_string(i) + " 0123456789\n";
      for (int n = 0; n < kLinesPerThread; ++n) {
        VgaWrite(line.c_str());
        VgaWriteHex(static_cast<uint64_t>(n));
        VgaPutc('\n');
      }
    });
  }
  for (std::thread& writer : writers) {
    writer.join();
  }

  EXPECT_THAT(VgaGetCursorRow(), t::Eq(kConsoleRows - 1));
  EXPECT_THAT(VgaGetCursorCol(), t::Eq(0));
  std::memset(g_fb32_b, kUntouchedByte, sizeof(g_fb32_b));
  ASSERT_TRUE(VgaInitFramebuffer(kFb32B.addr(),  //
                                 kPitch32,       //
                                 kFbWidth,       //
                                 kFbHeight,      //
                                 32));
  EXPECT_THAT(std::memcmp(g_fb32_a, g_fb32_b, sizeof(g_fb32_a)), t::Eq(0));
}

TEST(VgaTest, BackspaceMovesCursorLeftWithinRowAndAllowsOverwriting) {
  ResetEnv();
  AttachAndClear(kFb32A);

  // Backspace at column 0 is a no-op and keeps the cursor at (0, 0).
  VgaWrite("\b\b");
  EXPECT_THAT(VgaGetCursorRow(), t::Eq(0));
  EXPECT_THAT(VgaGetCursorCol(), t::Eq(0));

  // Write "F0", erase '0' with "\b \b", and overwrite column 1 with '$'.
  VgaWrite("F0");
  EXPECT_THAT(VgaGetCursorRow(), t::Eq(0));
  EXPECT_THAT(VgaGetCursorCol(), t::Eq(2));
  VgaWrite("\b \b");
  EXPECT_THAT(VgaGetCursorRow(), t::Eq(0));
  EXPECT_THAT(VgaGetCursorCol(), t::Eq(1));
  EXPECT_THAT(CellRows(kFb32A, 0, 0), t::Contains(".######."));
  EXPECT_THAT(CellRows(kFb32A, 0, 1), t::Each(t::Eq("........")));

  VgaPutc('$');
  EXPECT_THAT(VgaGetCursorRow(), t::Eq(0));
  EXPECT_THAT(VgaGetCursorCol(), t::Eq(2));
  EXPECT_THAT(CellRows(kFb32A, 0, 1), t::Contains("..#####."));

  // Backspace at column 0 of row 1 stays on row 1, column 0.
  VgaWrite("\n\b");
  EXPECT_THAT(VgaGetCursorRow(), t::Eq(1));
  EXPECT_THAT(VgaGetCursorCol(), t::Eq(0));

  std::memset(g_fb32_b, kUntouchedByte, sizeof(g_fb32_b));
  ASSERT_TRUE(VgaInitFramebuffer(kFb32B.addr(),  //
                                 kPitch32,       //
                                 kFbWidth,       //
                                 kFbHeight,      //
                                 32));
  EXPECT_THAT(std::memcmp(g_fb32_a, g_fb32_b, sizeof(g_fb32_a)), t::Eq(0));
}

}  // namespace
}  // namespace protos
