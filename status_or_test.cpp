#include "status_or.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

namespace protos {
namespace {

namespace t = ::testing;

enum class TestStatus : uint8_t {
  kOk = 0,
  kInvalidArgument = 1,
  kNotFound = 2,
};

struct SampleStruct {
  int a = 0;
  uint64_t b = 0;
};

static_assert(std::is_trivially_destructible_v<StatusOr<SampleStruct>>);
static_assert(std::is_trivially_copyable_v<StatusOr<SampleStruct>>);
static_assert(std::is_trivially_destructible_v<StatusOr<int64_t>>);
static_assert(
    std::is_trivially_destructible_v<StatusOr<SampleStruct, TestStatus>>);

TEST(StatusOrTest, HoldsStructValueAndSupportsAccessors) {
  const StatusOr<SampleStruct> res = SampleStruct{.a = 7, .b = 99};
  EXPECT_TRUE(res.ok());
  EXPECT_TRUE(static_cast<bool>(res));
  EXPECT_THAT(res.status(), t::Eq(0));
  EXPECT_THAT(res.value().a, t::Eq(7));
  EXPECT_THAT(res.value().b, t::Eq(99u));
  EXPECT_THAT((*res).a, t::Eq(7));
  EXPECT_THAT(res->b, t::Eq(99u));
  EXPECT_THAT(res.value_or(SampleStruct{.a = 1, .b = 2}).a, t::Eq(7));
}

TEST(StatusOrTest, HoldsInt64ErrorStatusForStructValue) {
  const StatusOr<SampleStruct> res = -22;
  EXPECT_FALSE(res.ok());
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_THAT(res.status(), t::Eq(-22));
  EXPECT_THAT(res.value_or(SampleStruct{.a = 42, .b = 100}).a, t::Eq(42));
}

TEST(StatusOrTest, SupportsEnumStatusType) {
  const StatusOr<SampleStruct, TestStatus> ok_res =
      SampleStruct{.a = 10, .b = 20};
  EXPECT_TRUE(ok_res.ok());
  EXPECT_THAT(ok_res.status(), t::Eq(TestStatus::kOk));
  EXPECT_THAT(ok_res->a, t::Eq(10));

  const StatusOr<SampleStruct, TestStatus> err_res = TestStatus::kNotFound;
  EXPECT_FALSE(err_res.ok());
  EXPECT_THAT(err_res.status(), t::Eq(TestStatus::kNotFound));
}

TEST(StatusOrTest, DisambiguatesIntegralValueTypesFromStatus) {
  const StatusOr<int64_t> zero_val = 0;
  EXPECT_TRUE(zero_val.ok());
  EXPECT_THAT(zero_val.status(), t::Eq(0));
  EXPECT_THAT(*zero_val, t::Eq(0));

  const StatusOr<int64_t> neg_val = -1250999896491LL;
  EXPECT_TRUE(neg_val.ok());
  EXPECT_THAT(*neg_val, t::Eq(-1250999896491LL));

  const StatusOr<int64_t> generic_err = kStatusError;
  EXPECT_FALSE(generic_err.ok());
  EXPECT_THAT(generic_err.status(), t::Eq(-22));
  EXPECT_THAT(generic_err.value_or(123), t::Eq(123));

  const StatusOr<int64_t> specific_err = Error(-14);
  EXPECT_FALSE(specific_err.ok());
  EXPECT_THAT(specific_err.status(), t::Eq(-14));

  const StatusOr<uint8_t> byte_val = uint8_t{0xAB};
  EXPECT_TRUE(byte_val.ok());
  EXPECT_THAT(*byte_val, t::Eq(0xABu));

  const StatusOr<uint8_t> byte_err = kStatusError;
  EXPECT_FALSE(byte_err.ok());
}

TEST(StatusOrTest, SupportsMoveOnlyAndNonTrivialTypes) {
  StatusOr<std::string> str_res = std::string("hello status_or");
  ASSERT_TRUE(str_res.ok());
  EXPECT_THAT(*str_res, t::StrEq("hello status_or"));

  const StatusOr<std::string> copy_res = str_res;
  ASSERT_TRUE(copy_res.ok());
  EXPECT_THAT(*copy_res, t::StrEq("hello status_or"));

  const std::string moved = *std::move(str_res);
  EXPECT_THAT(moved, t::StrEq("hello status_or"));

  StatusOr<std::unique_ptr<int>> ptr_res = std::make_unique<int>(55);
  ASSERT_TRUE(ptr_res.ok());
  EXPECT_THAT(**ptr_res, t::Eq(55));
}

TEST(StatusOrTest, SupportsConstexprEvaluation) {
  constexpr StatusOr<SampleStruct> kOk = SampleStruct{.a = 3, .b = 4};
  static_assert(kOk.ok());
  static_assert(kOk->a == 3);
  static_assert((*kOk).b == 4);

  constexpr StatusOr<SampleStruct, TestStatus> kErr =
      TestStatus::kInvalidArgument;
  static_assert(!kErr.ok());
  static_assert(kErr.status() == TestStatus::kInvalidArgument);
}

}  // namespace
}  // namespace protos
