#pragma once
// Compact wire form of the telemetry lines (Wi-Fi). A DeferredLine record is literal text plus
// 4-byte scalars. The literal text is the same on every record of a kind, so it is sent once as a
// "template" (id = 16-bit hash of the literal/op skeleton, repeated every kTemplateEveryMs and for
// each new client) and each record then carries only the id and its values as varints:
//   u32 / hex            varint
//   i32                  zigzag varint
//   fix(v, d)            varint (64 bit) of (round(|v| * 10^d) << 1 | signbit): what the text
//                        formatter prints, so the PC rebuilds the identical line.
// The PC side is tools/tap_telemetry.py. No number-to-text conversion runs on the robot.
//
// Frame: 0xA5, type, len (u16 LE), payload, crc8 (poly 0x07, over type..payload).
//   type 1 template  payload = id u16, skeleton bytes (literals; 0x00 op = value slot)
//   type 2 data      payload = id u16, one varint per value slot, in order
//   type 3 text      payload = plain text line(s) as written to the console
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace tap_fw {

class TelemetryEncoder {
 public:
  static constexpr std::uint8_t kSync = 0xA5U;
  static constexpr std::uint32_t kTemplateEveryMs = 2000U;
  static constexpr std::size_t kMaxRecord = 1536U;

  // Send every template again (new client).
  void resendTemplates() noexcept { ++epoch_; }

  // Encodes one deferred record (DeferredLine::data()/size()) into out. Returns the bytes written
  // (a template frame, when due, plus the data frame), or 0 when it does not fit or cannot be
  // encoded (caller falls back to a text frame).
  std::size_t encodeDeferred(const char* data, std::size_t size, std::uint32_t now_ms,
                             std::uint8_t* out, std::size_t cap) noexcept {
    if (size == 0U || size > kMaxRecord) return 0U;
    // Skeleton and hash in one pass.
    std::size_t skel = 0U;
    std::uint32_t h = 2166136261U;
    for (std::size_t i = 0U; i < size;) {
      const auto c = static_cast<std::uint8_t>(data[i++]);
      skeleton_[skel++] = c;
      h = (h ^ c) * 16777619U;
      if (c != 0U) continue;
      if (size - i < 5U) return 0U;
      const auto op = static_cast<std::uint8_t>(data[i]);
      skeleton_[skel++] = op;
      h = (h ^ op) * 16777619U;
      i += 5U;
    }
    const auto id = static_cast<std::uint16_t>((h ^ (h >> 16U)) & 0xFFFFU);
    Entry* e = nullptr;
    for (auto& t : table_) {
      if (t.used && t.id == id) { e = &t; break; }
    }
    if (e == nullptr) {
      for (auto& t : table_) {
        if (!t.used) { e = &t; t = Entry{true, id, h, 0U, 0U, false}; break; }
      }
      if (e == nullptr) return 0U;  // table full
    } else if (e->hash != h) {
      return 0U;  // id collision: caller sends text
    }
    std::size_t n = 0U;
    const bool due = !e->sent || e->epoch != epoch_ || now_ms - e->last_ms >= kTemplateEveryMs;
    if (due) {
      const std::size_t w = frame(1U, out + n, cap - n, id, skeleton_, skel);
      if (w == 0U) return 0U;
      n += w;
    }
    // Values.
    std::size_t v = 0U;
    values_[v++] = static_cast<std::uint8_t>(id & 0xFFU);
    values_[v++] = static_cast<std::uint8_t>(id >> 8U);
    for (std::size_t i = 0U; i < size;) {
      const auto c = static_cast<std::uint8_t>(data[i++]);
      if (c != 0U) continue;
      const auto op = static_cast<std::uint8_t>(data[i++]);
      std::uint32_t raw;
      std::memcpy(&raw, data + i, sizeof raw);
      i += 4U;
      if (v + 10U > sizeof values_) return 0U;
      if (op == 1U || op == 3U) {
        v += varint(values_ + v, raw);
      } else if (op == 2U) {
        const auto s = static_cast<std::int32_t>(raw);
        v += varint(values_ + v, (static_cast<std::uint32_t>(s) << 1U) ^ static_cast<std::uint32_t>(s >> 31));
      } else if ((op >= 16U && op <= 21U) || (op >= 24U && op <= 29U)) {
        float f;
        std::memcpy(&f, &raw, sizeof f);
        v += varint(values_ + v, fixCode(f, (op - 16U) & 7U));  // up to 10 bytes
      } else {
        return 0U;
      }
    }
    const std::size_t w = frame(2U, out + n, cap - n, 0xFFFFU, values_, v);
    if (w == 0U) return 0U;
    n += w;
    if (due) {
      e->sent = true;
      e->epoch = epoch_;
      e->last_ms = now_ms;
    }
    return n;
  }

  std::size_t encodeText(const char* text, std::size_t size, std::uint8_t* out, std::size_t cap) noexcept {
    if (size > kMaxRecord) size = kMaxRecord;
    return frame(3U, out, cap, 0xFFFFU, reinterpret_cast<const std::uint8_t*>(text), size);
  }

  static std::uint8_t crc8(const std::uint8_t* d, std::size_t n) noexcept {
    std::uint8_t c = 0U;
    for (std::size_t i = 0U; i < n; ++i) {
      c ^= d[i];
      for (int b = 0; b < 8; ++b) c = static_cast<std::uint8_t>((c & 0x80U) ? (c << 1U) ^ 0x07U : (c << 1U));
    }
    return c;
  }

  // Code of a fixed-point value; the PC prints it with `d` decimals. Same arithmetic as
  // LineBuf::fix (double, round half to even) so the rebuilt text is identical.
  static std::uint64_t fixCode(float v, unsigned d) noexcept {
    static constexpr double kPow[] = {1.0, 10.0, 100.0, 1000.0, 10000.0, 100000.0};
    if (std::isnan(v)) return kNan;
    if (std::isinf(v)) return v < 0.0F ? kNegInf : kPosInf;
    if (d > 5U) d = 5U;
    const double scaled = std::fabs(static_cast<double>(v)) * kPow[d];
    if (!(scaled < 4.0e18)) return kOverflow;
    const auto q = static_cast<std::uint64_t>(std::nearbyint(scaled));
    return (q << 1U) | (std::signbit(v) ? 1U : 0U);
  }
  static constexpr std::uint64_t kOverflow = ~0ULL, kNan = ~0ULL - 1U, kPosInf = ~0ULL - 2U, kNegInf = ~0ULL - 3U;

 private:
  struct Entry {
    bool used;
    std::uint16_t id;
    std::uint32_t hash;
    std::uint32_t last_ms;
    std::uint32_t epoch;
    bool sent;
  };
  static std::size_t varint(std::uint8_t* out, std::uint64_t v) noexcept {
    std::size_t n = 0U;
    while (v >= 0x80U) {
      out[n++] = static_cast<std::uint8_t>(v | 0x80U);
      v >>= 7U;
    }
    out[n++] = static_cast<std::uint8_t>(v);
    return n;
  }
  // payload0: template frames carry the id first; data frames carry it inside `body` already.
  static std::size_t frame(std::uint8_t type, std::uint8_t* out, std::size_t cap, std::uint16_t id,
                           const std::uint8_t* body, std::size_t len) noexcept {
    const std::size_t extra = type == 1U ? 2U : 0U;
    const std::size_t payload = len + extra;
    if (payload > 0xFFFFU || cap < payload + 5U) return 0U;
    out[0] = kSync;
    out[1] = type;
    out[2] = static_cast<std::uint8_t>(payload & 0xFFU);
    out[3] = static_cast<std::uint8_t>(payload >> 8U);
    std::size_t p = 4U;
    if (type == 1U) {
      out[p++] = static_cast<std::uint8_t>(id & 0xFFU);
      out[p++] = static_cast<std::uint8_t>(id >> 8U);
    }
    std::memcpy(out + p, body, len);
    p += len;
    out[p] = crc8(out + 1, p - 1U);
    return p + 1U;
  }

  Entry table_[24]{};
  std::uint32_t epoch_ = 1U;
  std::uint8_t skeleton_[kMaxRecord]{};
  std::uint8_t values_[kMaxRecord]{};
};

}  // namespace tap_fw
