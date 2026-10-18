#pragma once
// Fixed-point text formatting for the hot telemetry lines (TR, SENS).
// printf's float conversion cost the control tick most of its budget on the
// ESP32 (a TR line of ~40 floats every 40 ms: the tick's worst case was
// 2.6-2.8 ms against a 2 ms period). Same text as "%.Nf" / "%+.Nf" for the
// ranges telemetry uses (|v| < 4e9 / 10^N); nan/inf print as "nan"/"inf".
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace tap_fw {

class LineBuf {
 public:
  LineBuf(char* buf, std::size_t cap) noexcept : buf_(buf), cap_(cap) {
    if (cap_ > 0U) buf_[0] = '\0';
  }
  LineBuf& str(const char* s) noexcept {
    while (*s != '\0') put(*s++);
    return *this;
  }
  LineBuf& chr(char c) noexcept {
    put(c);
    return *this;
  }
  LineBuf& u32(std::uint32_t v) noexcept {
    char tmp[10];
    int n = 0;
    do {
      tmp[n++] = static_cast<char>('0' + v % 10U);
      v /= 10U;
    } while (v != 0U);
    while (n > 0) put(tmp[--n]);
    return *this;
  }
  LineBuf& i32(std::int32_t v) noexcept {
    if (v < 0) {
      put('-');
      return u32(static_cast<std::uint32_t>(-(static_cast<std::int64_t>(v))));
    }
    return u32(static_cast<std::uint32_t>(v));
  }
  LineBuf& hex(std::uint32_t v) noexcept {  // "%lx"
    char tmp[8];
    int n = 0;
    do {
      const std::uint32_t d = v & 0xFU;
      tmp[n++] = static_cast<char>(d < 10U ? '0' + d : 'a' + (d - 10U));
      v >>= 4U;
    } while (v != 0U);
    while (n > 0) put(tmp[--n]);
    return *this;
  }
  // "%.<decimals>f", or "%+.<decimals>f" with plus = true.
  LineBuf& fix(float v, int decimals, bool plus = false) noexcept {
    if (std::isnan(v)) return str("nan");
    if (std::isinf(v)) return str(v < 0.0F ? "-inf" : "inf");
    static constexpr std::uint32_t kPow[] = {1U, 10U, 100U, 1000U, 10000U, 100000U};
    const int d = decimals < 0 ? 0 : decimals > 5 ? 5 : decimals;
    const double scaled = std::fabs(static_cast<double>(v)) * kPow[d];
    // Round half to even like printf on the exact binary value: nearbyint.
    const double r = std::nearbyint(scaled);
    const std::uint64_t q = r < 1.8e19 ? static_cast<std::uint64_t>(r) : 0U;
    // printf keeps the sign even when the digits round to zero ("-0.000").
    if (std::signbit(v)) {
      put('-');
    } else if (plus) {
      put('+');
    }
    const std::uint64_t ip = q / kPow[d];
    const std::uint64_t fp = q % kPow[d];
    u64(ip);
    if (d > 0) {
      put('.');
      for (int i = d - 1; i >= 0; --i) put(static_cast<char>('0' + (fp / kPow[i]) % 10U));
    }
    return *this;
  }
  [[nodiscard]] const char* c_str() const noexcept { return buf_; }
  [[nodiscard]] std::size_t size() const noexcept { return n_; }
  [[nodiscard]] bool overflowed() const noexcept { return over_; }

 private:
  void u64(std::uint64_t v) noexcept {
    char tmp[20];
    int n = 0;
    do {
      tmp[n++] = static_cast<char>('0' + v % 10U);
      v /= 10U;
    } while (v != 0U);
    while (n > 0) put(tmp[--n]);
  }
  void put(char c) noexcept {
    if (n_ + 1U < cap_) {
      buf_[n_++] = c;
      buf_[n_] = '\0';
    } else {
      over_ = true;
    }
  }
  char* buf_;
  std::size_t cap_;
  std::size_t n_ = 0U;
  bool over_ = false;
};

}  // namespace tap_fw
