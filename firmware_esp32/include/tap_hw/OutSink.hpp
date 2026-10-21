#pragma once
// Output of the control task without blocking it: framed records go into a
// FreeRTOS stream buffer (single writer: the control task) and the Arduino
// loop task drains it to USB serial. Before the stream exists (setup) and
// in the single-loop fallback it writes to Serial directly, as before.
#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstring>

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include "tap_fw/DeferredFormat.hpp"
#include "tap_fw/TelemetryCodec.hpp"

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
    return enqueue(s, n, 0U) ? n : 0U;
  }
  void println(const tap_fw::DeferredLine& line) noexcept {
    if (line.overflowed()) { ++dropped_; return; }
    if (sb_ != nullptr) { (void)enqueue(line.data(), line.size(), 1U); return; }
    char text[1024];
    const auto n = tap_fw::DeferredLine::render(line.data(), line.size(), text, sizeof text);
    if (n != 0U) Serial.write(reinterpret_cast<const std::uint8_t*>(text), n);
    else ++dropped_;
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
  // Consumer only. Numeric conversion happens here on core 0, below the
  // wheel workers' priority. Preserve FIFO ordering with ordinary text, and
  // retain a partial serial write without ever waiting for UART space.
  void drainTo(HardwareSerial& out) noexcept {
    if (sb_ == nullptr) return;
    if (!uart_text_.load(std::memory_order_relaxed)) {
      drainTeeOnly();
      return;
    }
    int room = out.availableForWrite();
    while (room > 0) {
      if (text_pos_ == text_size_) {
        text_pos_ = text_size_ = 0U;
        if (!pullRecord()) break;
        const bool tee_live = tee_ != nullptr && tee_on_.load(std::memory_order_relaxed);
        if (tee_live && tee_binary_.load(std::memory_order_relaxed)) teeBinary();
        renderRecord();
        payload_size_ = 0U;
        // Bound work even when a record cannot be rendered.
        if (text_size_ == 0U) break;
        if (tee_live && !tee_binary_.load(std::memory_order_relaxed)) teeText();
      }
      const auto want = std::min(text_size_ - text_pos_, static_cast<std::size_t>(room));
      const auto sent = out.write(reinterpret_cast<const std::uint8_t*>(text_ + text_pos_), want);
      text_pos_ += sent;
      room -= static_cast<int>(sent);
      if (sent == 0U) break;
    }
  }
  // UART text off: records go only to the Wi-Fi copy and no number is turned into text at all
  // when that copy is binary (the default). Bounded work per call.
  void drainTeeOnly() noexcept {
    for (int n = 0; n < 24; ++n) {
      if (!pullRecord()) break;
      if (tee_ != nullptr && tee_on_.load(std::memory_order_relaxed)) {
        if (tee_binary_.load(std::memory_order_relaxed)) {
          teeBinary();
        } else {
          renderRecord();
          if (text_size_ != 0U) teeText();
        }
      }
      payload_size_ = 0U;
      text_size_ = text_pos_ = 0U;
    }
  }
  // UART console on/off (SET uartlog) and the Wi-Fi copy: on only while a client is connected,
  // binary (tap_fw::TelemetryEncoder) or plain text (SET wifilog).
  void setUartText(bool on) noexcept { uart_text_.store(on, std::memory_order_relaxed); }
  [[nodiscard]] bool uartText() const noexcept { return uart_text_.load(std::memory_order_relaxed); }
  void setTeeBinary(bool on) noexcept { tee_binary_.store(on, std::memory_order_relaxed); }
  [[nodiscard]] bool teeBinaryMode() const noexcept { return tee_binary_.load(std::memory_order_relaxed); }
  void setTeeActive(bool on) noexcept {
    if (on) encoder_.resendTemplates();
    tee_on_.store(on, std::memory_order_relaxed);
  }
  void setTee(StreamBufferHandle_t tee) noexcept { tee_ = tee; }
  // While the robot stands up or balances the Wi-Fi copy carries less: no WT/WBUS diagnostics and
  // every other SU/SC/SW/SV/SF snapshot (the UART copy is unchanged). Less radio traffic means
  // fewer Wi-Fi task and interrupt slices on the core that runs the wheel bus workers.
  void setTeeQuiet(bool quiet) noexcept { tee_quiet_.store(quiet, std::memory_order_relaxed); }
  [[nodiscard]] std::uint32_t teeDropped() const noexcept { return tee_dropped_; }
  [[nodiscard]] std::uint32_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }

 private:
  bool teeSkips(const char* t, std::size_t n) noexcept {
    // Only the text copy is thinned out; the compact binary copy carries every record.
    if (tee_binary_.load(std::memory_order_relaxed) || !tee_quiet_.load(std::memory_order_relaxed) || n < 3U) return false;
    const char a = t[0], b = t[1];
    if ((a == 'W' && b == 'T' && t[2] == ' ') || (a == 'W' && b == 'B' && n > 4U && t[2] == 'U')) return true;
    if (a == 'S' && (b == 'U' || b == 'C' || b == 'W' || b == 'V' || b == 'F') && t[2] == ' ') return (tee_snapshot_++ & 1U) != 0U;
    return false;
  }
  // Next complete record from the stream buffer into kind_/payload_/payload_size_; the caller
  // clears payload_size_ when it is done with it.
  bool pullRecord() noexcept {
    if (payload_size_ == 0U) {
      if (xStreamBufferBytesAvailable(sb_) < 3U) return false;
      std::uint8_t header[3];
      (void)xStreamBufferReceive(sb_, header, sizeof header, 0);
      kind_ = header[0];
      payload_size_ = header[1] | (static_cast<std::size_t>(header[2]) << 8U);
    }
    // Producer may have published the header but not the payload yet.
    if (xStreamBufferBytesAvailable(sb_) < payload_size_) return false;
    (void)xStreamBufferReceive(sb_, payload_, payload_size_, 0);
    return true;
  }
  void renderRecord() noexcept {
    if (kind_ == 1U) {
      text_size_ = tap_fw::DeferredLine::render(payload_, payload_size_, text_, sizeof text_);
      if (text_size_ == 0U) ++dropped_;
    } else {
      std::memcpy(text_, payload_, payload_size_);
      text_size_ = payload_size_;
    }
  }
  void teeText() noexcept {
    if (teeSkips(text_, text_size_)) return;
    if (xStreamBufferSpacesAvailable(tee_) >= text_size_) (void)xStreamBufferSend(tee_, text_, text_size_, 0);
    else ++tee_dropped_;
  }
  void teeBinary() noexcept {
    if (teeSkips(payload_, payload_size_)) return;
    std::size_t w = 0U;
    if (kind_ == 1U) w = encoder_.encodeDeferred(payload_, payload_size_, millis(), frame_, sizeof frame_);
    if (w == 0U) {  // not encodable (or plain text): a text frame
      std::size_t n = payload_size_;
      const char* src = payload_;
      if (kind_ == 1U) {
        n = tap_fw::DeferredLine::render(payload_, payload_size_, text_, sizeof text_);
        src = text_;
      }
      if (n == 0U) return;
      w = encoder_.encodeText(src, n, frame_, sizeof frame_);
    }
    if (w != 0U && xStreamBufferSpacesAvailable(tee_) >= w) (void)xStreamBufferSend(tee_, frame_, w, 0);
    else ++tee_dropped_;
  }
  bool enqueue(const char* data, std::size_t n, std::uint8_t kind) noexcept {
    if (n == 0U) return true;
    if (n > sizeof payload_ || (kind == 0U && n > sizeof text_) ||
        xStreamBufferSpacesAvailable(sb_) < n + 3U) {
      ++dropped_;
      return false;
    }
    const std::uint8_t header[3] = {kind, static_cast<std::uint8_t>(n), static_cast<std::uint8_t>(n >> 8U)};
    // Single producer; consumer only frees space. After the capacity check
    // both nonblocking sends fit, including if the reader wakes in between.
    (void)xStreamBufferSend(sb_, header, sizeof header, 0);
    (void)xStreamBufferSend(sb_, data, n, 0);
    return true;
  }
  StreamBufferHandle_t sb_ = nullptr;
  StreamBufferHandle_t tee_ = nullptr;
  std::atomic<bool> tee_quiet_{false};
  std::atomic<bool> uart_text_{true}, tee_on_{false}, tee_binary_{true};
  tap_fw::TelemetryEncoder encoder_{};
  std::uint8_t frame_[2048]{};
  std::uint32_t tee_snapshot_ = 0U;
  std::uint32_t tee_dropped_ = 0U;
  std::atomic<std::uint32_t> dropped_{0U};
  char payload_[1536]{}, text_[1024]{};
  std::size_t payload_size_ = 0U, text_size_ = 0U, text_pos_ = 0U;
  std::uint8_t kind_ = 0U;
};

}  // namespace tap_hw
