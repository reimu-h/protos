#include "fpu.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace protos {
namespace {

namespace t = ::testing;

// The bytes that FpuBuildInitialImage() must write into an area of `size`
// bytes: zero except the x87 control word 0x037F and MXCSR 0x1F80, both
// little-endian.
static std::vector<uint8_t> ExpectedInitialImage(const int size) {
  std::vector<uint8_t> image(size, 0);
  image[0] = 0x7F;
  image[1] = 0x03;
  image[24] = 0x80;
  image[25] = 0x1F;
  return image;
}

static std::vector<uint8_t> Bytes(const void* const data, const int size) {
  const uint8_t* const bytes = static_cast<const uint8_t*>(data);
  return std::vector<uint8_t>(bytes, bytes + size);
}

static FpuConfig MakeConfig(const bool xsave,     //
                            const uint64_t xcr0,  //
                            const int state_size) {
  FpuConfig config;
  config.xsave = xsave;
  config.xcr0 = xcr0;
  config.state_size = state_size;
  return config;
}

// Every test starts and ends with the default format, FXSAVE.
class FpuTest : public t::Test {
 protected:
  void SetUp() override { FpuInit(FpuConfig{}); }
  void TearDown() override { FpuInit(FpuConfig{}); }
};

TEST_F(FpuTest, InitialImageHoldsOnlyTheInitialControlWordAndMxcsr) {
  // FXSAVE, XSAVE with x87|SSE, x87|SSE|AVX and x87|SSE|AVX|AVX-512. The
  // buffer starts out filled, as heap memory may be, so the image must
  // overwrite every byte, the XSAVE header (bytes 512-575) included.
  for (const int size : {512, 576, 832, 2688}) {
    std::vector<uint8_t> buffer(size, 0xAA);
    FpuBuildInitialImage(buffer.data(), size);
    EXPECT_THAT(buffer, t::ElementsAreArray(ExpectedInitialImage(size)))
        << "size " << size;
  }
}

TEST_F(FpuTest, InitialImageUsesTheArchitecturalInitialValues) {
  EXPECT_THAT(kFpuInitialFcw, t::Eq(0x037F));
  EXPECT_THAT(kFpuInitialMxcsr, t::Eq(0x1F80u));
  EXPECT_THAT(kFpuFcwOffset, t::Eq(0));
  EXPECT_THAT(kFpuMxcsrOffset, t::Eq(24));
}

TEST_F(FpuTest, ConfigIsValidOnlyForConsistentFormats) {
  // FXSAVE, and XSAVE with x87|SSE (QEMU qemu64,+xsave), x87|SSE|AVX (QEMU
  // max) and x87|SSE|AVX|AVX-512.
  EXPECT_TRUE(FpuConfigIsValid(MakeConfig(false, 0, 512)));
  EXPECT_TRUE(FpuConfigIsValid(MakeConfig(true, 0x3, 576)));
  EXPECT_TRUE(FpuConfigIsValid(MakeConfig(true, 0x7, 832)));
  EXPECT_TRUE(FpuConfigIsValid(MakeConfig(true, 0xE7, 2688)));
  EXPECT_TRUE(FpuConfigIsValid(MakeConfig(true, 0x7, kFpuMaxStateSize)));

  // FXSAVE with components or another size.
  EXPECT_FALSE(FpuConfigIsValid(MakeConfig(false, 0x7, 512)));
  EXPECT_FALSE(FpuConfigIsValid(MakeConfig(false, 0, 832)));
  // XSAVE without the SSE or the x87 component.
  EXPECT_FALSE(FpuConfigIsValid(MakeConfig(true, 0x1, 576)));
  EXPECT_FALSE(FpuConfigIsValid(MakeConfig(true, 0x6, 832)));
  // AVX without SSE.
  EXPECT_FALSE(FpuConfigIsValid(MakeConfig(true, 0x5, 832)));
  // Part of AVX-512, and AVX-512 without AVX.
  EXPECT_FALSE(FpuConfigIsValid(MakeConfig(true, 0x27, 832)));
  EXPECT_FALSE(FpuConfigIsValid(MakeConfig(true, 0x67, 2688)));
  EXPECT_FALSE(FpuConfigIsValid(MakeConfig(true, 0xE3, 2688)));
  // MPX (bits 3 and 4) and PKRU (bit 9), which the kernel never enables.
  EXPECT_FALSE(FpuConfigIsValid(MakeConfig(true, 0x1F, 1088)));
  EXPECT_FALSE(FpuConfigIsValid(MakeConfig(true, 0x207, 840)));
  // Sizes outside [576, 16384].
  EXPECT_FALSE(FpuConfigIsValid(MakeConfig(true, 0x7, 512)));
  EXPECT_FALSE(FpuConfigIsValid(MakeConfig(true, 0x3, 575)));
  EXPECT_FALSE(FpuConfigIsValid(MakeConfig(true, 0x7, kFpuMaxStateSize + 1)));
}

TEST_F(FpuTest, InitSetsTheFormatAndTheStateSize) {
  EXPECT_FALSE(FpuGetConfig().xsave);
  EXPECT_THAT(FpuStateSize(), t::Eq(kFpuFxsaveSize));

  FpuInit(MakeConfig(true, 0x7, 832));
  EXPECT_TRUE(FpuGetConfig().xsave);
  EXPECT_THAT(FpuGetConfig().xcr0, t::Eq(0x7u));
  EXPECT_THAT(FpuStateSize(), t::Eq(832));

  FpuInit(MakeConfig(true, 0x3, 576));
  EXPECT_THAT(FpuGetConfig().xcr0, t::Eq(0x3u));
  EXPECT_THAT(FpuStateSize(), t::Eq(576));

  FpuInit(MakeConfig(false, 0, 512));
  EXPECT_FALSE(FpuGetConfig().xsave);
  EXPECT_THAT(FpuGetConfig().xcr0, t::Eq(0u));
  EXPECT_THAT(FpuStateSize(), t::Eq(512));
  EXPECT_FALSE(FpuGetConfig().reset_x87_pointers);

  // With either format, FpuRestore() may first reset the x87 pointers.
  for (const bool xsave : {false, true}) {
    FpuConfig config =
        xsave ? MakeConfig(true, 0x7, 832) : MakeConfig(false, 0, 512);
    config.reset_x87_pointers = true;
    EXPECT_TRUE(FpuConfigIsValid(config));
    FpuInit(config);
    EXPECT_TRUE(FpuGetConfig().reset_x87_pointers);
    EXPECT_THAT(FpuGetConfig().xsave, t::Eq(xsave));
  }
}

TEST_F(FpuTest, CreateReturnsDistinctAlignedStatesHoldingTheInitialImage) {
  for (const FpuConfig& config :
       {MakeConfig(false, 0, 512), MakeConfig(true, 0x3, 576),
        MakeConfig(true, 0x7, 832), MakeConfig(true, 0xE7, 2688)}) {
    FpuInit(config);
    const FpuStatePtr first = FpuStateCreate();
    const FpuStatePtr second = FpuStateCreate();
    ASSERT_THAT(first, t::NotNull());
    ASSERT_THAT(second, t::NotNull());
    EXPECT_THAT(first.get(), t::Ne(second.get()));
    for (const FpuState* const state : {first.get(), second.get()}) {
      EXPECT_THAT(reinterpret_cast<uintptr_t>(state) % kFpuStateAlignment,
                  t::Eq(0u));
      // Every byte of the state holds the initial image, also where the
      // allocator left garbage.
      EXPECT_THAT(Bytes(state, config.state_size),
                  t::ElementsAreArray(ExpectedInitialImage(config.state_size)))
          << "state size " << config.state_size;
    }
  }
}

TEST(FpuDeathTest, InitRejectsAnInvalidConfig) {
  EXPECT_DEATH(FpuInit(MakeConfig(true, 0x7, 100)),
               "Check failed: fpu_config_valid");
  EXPECT_DEATH(FpuInit(MakeConfig(false, 0x3, 512)),
               "Check failed: fpu_config_valid");
}

}  // namespace
}  // namespace protos
