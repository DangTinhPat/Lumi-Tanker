#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "tap_fw/FastFormat.hpp"

namespace tap_fw {

// Copy a telemetry snapshot without converting numbers on the control task.
// No pointers or references survive enqueue: strings and scalar values are
// owned by the record. The consumer uses the original formatter/rounding.
class DeferredLine {
 public:
  DeferredLine(char* data, std::size_t capacity) noexcept : data_(data), capacity_(capacity) {}
  DeferredLine& str(const char* s) noexcept {
    while (*s) put(static_cast<std::uint8_t>(*s++));
    return *this;
  }
  DeferredLine& chr(char c) noexcept {
    if (c == '\0') overflow_ = true;  // zero is the scalar escape
    else put(static_cast<std::uint8_t>(c));
    return *this;
  }
  DeferredLine& u32(std::uint32_t v) noexcept { return scalar(1, v); }
  DeferredLine& i32(std::int32_t v) noexcept { return scalar(2, v); }
  DeferredLine& hex(std::uint32_t v) noexcept { return scalar(3, v); }
  DeferredLine& fix(float v, int decimals, bool plus = false) noexcept {
    const auto d = static_cast<std::uint8_t>(decimals < 0 ? 0 : decimals > 5 ? 5 : decimals);
    return scalar(static_cast<std::uint8_t>(16U + d + (plus ? 8U : 0U)), v);
  }
  const char* data() const noexcept { return data_; }
  std::size_t size() const noexcept { return size_; }
  bool overflowed() const noexcept { return overflow_; }

  // Returns zero on invalid/truncated data or output overflow. Whole lines
  // are discarded rather than leaking binary bytes or partial GUI records.
  static std::size_t render(const char* data, std::size_t size,
                            char* text, std::size_t capacity) noexcept {
    LineBuf b(text, capacity);
    for (std::size_t i = 0; i < size;) {
      const auto c = static_cast<std::uint8_t>(data[i++]);
      if (c != 0U) { b.chr(static_cast<char>(c)); continue; }
      if (size - i < 5U) return 0U;
      const auto op = static_cast<std::uint8_t>(data[i++]);
      std::uint32_t u;
      std::memcpy(&u, data + i, sizeof u);
      if (op == 1U) b.u32(u);
      else if (op == 2U) {
        std::int32_t v;
        std::memcpy(&v, data + i, sizeof v);
        b.i32(v);
      } else if (op == 3U) b.hex(u);
      else if ((op >= 16U && op <= 21U) || (op >= 24U && op <= 29U)) {
        float v;
        std::memcpy(&v, data + i, sizeof v);
        b.fix(v, (op - 16U) & 7U, op >= 24U);
      } else return 0U;
      i += 4U;
    }
    b.str("\r\n");
    return b.overflowed() ? 0U : b.size();
  }

 private:
  template <typename T> DeferredLine& scalar(std::uint8_t op, T value) noexcept {
    static_assert(sizeof(T) == 4U, "telemetry scalars must be 32 bit");
    if (capacity_ - size_ < 6U) { overflow_ = true; return *this; }
    put(0U);
    put(op);
    std::memcpy(data_ + size_, &value, sizeof value);
    size_ += sizeof value;
    return *this;
  }
  void put(std::uint8_t value) noexcept {
    if (size_ < capacity_) data_[size_++] = static_cast<char>(value);
    else overflow_ = true;
  }
  char* data_;
  std::size_t capacity_, size_ = 0U;
  bool overflow_ = false;
};

}  // namespace tap_fw
