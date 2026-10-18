#pragma once
// Text output of the control task without blocking it: lines go into a
// FreeRTOS stream buffer (single writer: the control task) and the Arduino
// loop task drains it to USB serial. Before the stream exists (setup) and
// in the single-loop fallback it writes to Serial directly, as before.
#include <algorithm>
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstring>

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>

namespace tap_hw {

class OutSink {
 public:
  bool begin(std::size_t bytes) noexcept {
    sb_ = xStreamBufferCreate(bytes, 1);
    return sb_ != nullptr;
  }
  [[nodiscard]] bool buffered() const noexcept { return sb_ != nullptr; }
  std::size_t write(const char* s, std::size_t n) noexcept {
    if (sb_ == nullptr) return Serial.write(reinterpret_cast<const std::uint8_t*>(s), n);
    // Whole lines or nothing: a partial line would corrupt the parsers.
    if (xStreamBufferSpacesAvailable(sb_) < n) {
      ++dropped_;
      return 0U;
    }
    return xStreamBufferSend(sb_, s, n, 0);
  }
  void print(const char* s) noexcept { (void)write(s, std::strlen(s)); }
  void println(const char* s) noexcept {
    // One write so a line is never split by a full buffer (Serial's "\r\n").
    char buf[1024];
    const std::size_t n = std::strlen(s);
    if (n + 2U < sizeof(buf)) {
      std::memcpy(buf, s, n);
      buf[n] = '\r';
      buf[n + 1U] = '\n';
      (void)write(buf, n + 2U);
    } else {
      print(s);
      (void)write("\r\n", 2U);
    }
  }
  int printf(const char* fmt, ...) noexcept __attribute__((format(printf, 2, 3))) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    const int n = std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) (void)write(buf, static_cast<std::size_t>(std::min(n, static_cast<int>(sizeof(buf)) - 1)));
    return n;
  }
  [[nodiscard]] std::size_t availableForWrite() const noexcept {
    return sb_ != nullptr ? xStreamBufferSpacesAvailable(sb_)
                          : static_cast<std::size_t>(Serial.availableForWrite());
  }
  // Loop task: move what the serial TX buffer can take now.
  void drainTo(HardwareSerial& out) noexcept {
    if (sb_ == nullptr) return;
    char chunk[256];
    int room = out.availableForWrite();
    while (room > 0) {
      const std::size_t want = std::min<std::size_t>(sizeof(chunk), static_cast<std::size_t>(room));
      const std::size_t got = xStreamBufferReceive(sb_, chunk, want, 0);
      if (got == 0U) break;
      out.write(reinterpret_cast<const std::uint8_t*>(chunk), got);
      room -= static_cast<int>(got);
    }
  }
  [[nodiscard]] std::uint32_t dropped() const noexcept { return dropped_; }

 private:
  StreamBufferHandle_t sb_ = nullptr;
  volatile std::uint32_t dropped_ = 0U;
};

}  // namespace tap_hw
