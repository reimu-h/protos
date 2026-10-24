#ifndef PROTOS_STATUS_OR_H_
#define PROTOS_STATUS_OR_H_

#include <cstdint>
#include <new>
#include <type_traits>
#include <utility>

namespace protos {

// Generic error tag for functions that do not distinguish between failure
// reasons or where the value type T is convertible to/from the status type
// (e.g. StatusOr<int64_t> or StatusOr<uint8_t>).
enum class StatusError : uint8_t {
  kOk = 0,
  kError = 1,
};

inline constexpr StatusError kStatusError = StatusError::kError;

// Explicit error wrapper used to return a specific status code when T and
// Status are both integral or convertible to each other:
//   return Error(-kSyscallErrorInvalid);
template <typename S = int64_t>
struct ErrorStatus {
  S status;
};

template <typename S>
[[nodiscard]] constexpr ErrorStatus<S> Error(S status) {
  return ErrorStatus<S>{status};
}

[[nodiscard]] constexpr ErrorStatus<int64_t> Error() {
  return ErrorStatus<int64_t>{-22};
}

namespace internal {

struct StatusOrValueTag {};
struct StatusOrErrorTag {};

template <typename T, typename Status,
          bool kTrivial = std::is_trivially_destructible_v<T> &&
                          std::is_trivially_destructible_v<Status>>
struct StatusOrStorage {
  union {
    Status status_;
    T value_;
  };
  bool ok_ = false;

  constexpr StatusOrStorage(StatusOrValueTag, const T& value)
      : value_(value), ok_(true) {}
  constexpr StatusOrStorage(StatusOrValueTag, T&& value)
      : value_(std::move(value)), ok_(true) {}
  constexpr StatusOrStorage(StatusOrErrorTag, Status status)
      : status_(status), ok_(false) {}
};

template <typename T, typename Status>
struct StatusOrStorage<T, Status, false> {
  union {
    Status status_;
    T value_;
  };
  bool ok_ = false;

  StatusOrStorage(StatusOrValueTag, const T& value)
      : value_(value), ok_(true) {}
  StatusOrStorage(StatusOrValueTag, T&& value)
      : value_(std::move(value)), ok_(true) {}
  StatusOrStorage(StatusOrErrorTag, Status status)
      : status_(std::move(status)), ok_(false) {}

  StatusOrStorage(const StatusOrStorage& other) : ok_(other.ok_) {
    if (ok_) {
      new (&value_) T(other.value_);
    } else {
      new (&status_) Status(other.status_);
    }
  }

  StatusOrStorage(StatusOrStorage&& other) noexcept : ok_(other.ok_) {
    if (ok_) {
      new (&value_) T(std::move(other.value_));
    } else {
      new (&status_) Status(std::move(other.status_));
    }
  }

  StatusOrStorage& operator=(const StatusOrStorage& other) {
    if (this == &other) {
      return *this;
    }
    Destroy();
    ok_ = other.ok_;
    if (ok_) {
      new (&value_) T(other.value_);
    } else {
      new (&status_) Status(other.status_);
    }
    return *this;
  }

  StatusOrStorage& operator=(StatusOrStorage&& other) noexcept {
    if (this == &other) {
      return *this;
    }
    Destroy();
    ok_ = other.ok_;
    if (ok_) {
      new (&value_) T(std::move(other.value_));
    } else {
      new (&status_) Status(std::move(other.status_));
    }
    return *this;
  }

  ~StatusOrStorage() { Destroy(); }

 private:
  void Destroy() {
    if (ok_) {
      value_.~T();
    } else {
      status_.~Status();
    }
  }
};

}  // namespace internal

// Holds either a usable value of type T or a non-OK error status of type
// Status (where Status{} represents OK, such as 0 for int64_t or kOk/kNone = 0
// for status enums).
template <typename T, typename Status = int64_t>
class [[nodiscard]] StatusOr : private internal::StatusOrStorage<T, Status> {
  static_assert(!std::is_reference_v<T>,
                "StatusOr does not support reference types");
  static_assert(!std::is_void_v<T>, "StatusOr does not support void");

  using Base = internal::StatusOrStorage<T, Status>;

  static constexpr bool kAmbiguousStatus =
      std::is_same_v<std::remove_cv_t<T>, std::remove_cv_t<Status>> ||
      std::is_convertible_v<Status, T> || std::is_convertible_v<T, Status>;

 public:
  using value_type = T;
  using status_type = Status;

  StatusOr() = delete;

  constexpr StatusOr(const T& value)
      : Base(internal::StatusOrValueTag{}, value) {}
  constexpr StatusOr(T&& value)
      : Base(internal::StatusOrValueTag{}, std::move(value)) {}

  constexpr StatusOr(Status status)
    requires(!kAmbiguousStatus)
      : Base(internal::StatusOrErrorTag{}, ValidateError(status)) {}

  constexpr StatusOr(StatusError error)
      : Base(internal::StatusOrErrorTag{}, DefaultErrorStatus(error)) {}

  template <typename S>
  constexpr StatusOr(ErrorStatus<S> error)
    requires std::is_convertible_v<S, Status>
      : Base(internal::StatusOrErrorTag{},
             ValidateError(static_cast<Status>(error.status))) {}

  [[nodiscard]] constexpr bool ok() const { return this->ok_; }
  constexpr explicit operator bool() const { return this->ok_; }

  [[nodiscard]] constexpr Status status() const {
    return this->ok_ ? Status{} : this->status_;
  }

  constexpr T& value() & {
    CheckOk();
    return this->value_;
  }

  constexpr const T& value() const& {
    CheckOk();
    return this->value_;
  }

  constexpr T&& value() && {
    CheckOk();
    return std::move(this->value_);
  }

  constexpr const T&& value() const&& {
    CheckOk();
    return std::move(this->value_);
  }

  constexpr T& operator*() & {
    CheckOk();
    return this->value_;
  }

  constexpr const T& operator*() const& {
    CheckOk();
    return this->value_;
  }

  constexpr T&& operator*() && {
    CheckOk();
    return std::move(this->value_);
  }

  constexpr const T&& operator*() const&& {
    CheckOk();
    return std::move(this->value_);
  }

  constexpr T* operator->() {
    CheckOk();
    return &this->value_;
  }

  constexpr const T* operator->() const {
    CheckOk();
    return &this->value_;
  }

  template <typename U>
  constexpr T value_or(U&& default_value) const& {
    return this->ok_ ? this->value_
                     : static_cast<T>(std::forward<U>(default_value));
  }

  template <typename U>
  constexpr T value_or(U&& default_value) && {
    return this->ok_ ? std::move(this->value_)
                     : static_cast<T>(std::forward<U>(default_value));
  }

 private:
  constexpr void CheckOk() const {
    if (!this->ok_) {
      __builtin_trap();
    }
  }

  static constexpr Status ValidateError(Status status) {
    if (status == Status{}) {
      __builtin_trap();
    }
    return status;
  }

  static constexpr Status DefaultErrorStatus(StatusError error) {
    if (error == StatusError::kOk) {
      __builtin_trap();
    }
    if constexpr (std::is_enum_v<Status>) {
      return static_cast<Status>(1);
    } else if constexpr (std::is_signed_v<Status>) {
      return static_cast<Status>(-22);
    } else {
      return static_cast<Status>(1);
    }
  }
};

}  // namespace protos

using protos::Error;
using protos::ErrorStatus;
using protos::kStatusError;
using protos::StatusError;
using protos::StatusOr;

#endif  // PROTOS_STATUS_OR_H_
