# Protocol UART động cơ bánh xe — Kpower BL4012 (EC4012 / R1-POC)

2026-10-12: thay thế hoàn toàn 2 động cơ bánh xe CJC BL4820M12 trước đây
bằng 2 động cơ Kpower BL4012 (gọi tắt "EC4012"/"R1-POC" trong các công cụ
bench ở `TEST1/`). Cùng kiểu khung gói giống Dynamixel v1.0 và CÙNG bảng
thanh ghi (0x18/0x20/0x24) nên toàn bộ mục 3–5 dưới đây không đổi — đã test
lại thực tế trên 2 động cơ mới và vẫn đúng. Thứ thay đổi là: tên/datasheet
động cơ (mục 1), và **topology bus** (mục 2): 2 động cơ giờ đấu SONG SONG
trên CHUNG một bus UART bán song công (half-duplex), phân biệt bằng motor
id, thay vì mỗi động cơ một bus UART song công riêng như trước.

Nguồn số liệu mục 1: datasheet chính hãng `Kpower-4012-outer rotror
brushless.pdf` (Kpower Technology, sales@kpower.com) — 性能参数/Performance
Specification. Mục 3–5 vẫn từ nguồn cũ (`Motor_Communication_Guide.html` +
`motor_controller.html`, reverse-engineer từ log thật) vì đã verify lại
khớp với động cơ mới.

**Đây không phải protocol chuẩn Dynamixel gốc** — chỉ có khung gói giống
Dynamixel v1.0. Cách encode tốc độ (signed, không phải magnitude+bit
hướng) và quirk echo 3 byte trong response là tùy biến riêng của firmware
này.

## 1. Thông số động cơ (Kpower BL4012)

| | |
|---|---|
| Điện áp danh định / dải điện áp | 12V / 9–16V |
| Tốc độ danh định | 400 RPM |
| Tốc độ không tải (@12V) | 950 RPM |
| Mô-men danh định | 0.13 N·m |
| Mô-men khởi động (stall) | 0.2 N·m |
| Dòng danh định / khởi động | 0.95 A / 2 A |
| Encoder | Từ tính (magnetic), 14 bit — độ phân giải vật lý; **thanh ghi
Present Position vẫn 0~1023/360° (mục 4), không đổi theo bit encoder vật lý** |
| Đầu nối | SH1.0, 3 hoặc 4 chân |
| Hộp số | Không có (truyền động trực tiếp) |

## 2. Thông số bus

| | |
|---|---|
| Baud | 1,000,000 bps, 8N1, không parity |
| Topology | **Chung 1 bus half-duplex** (ESP32 GPIO17 trên `leg_jog_fw`), 2 động cơ đấu song song, TX/RX chung 1 dây — không còn 1 bus/1 động cơ như bản BL4820M12 cũ |
| Motor ID | Đã set qua Type-C: bánh phải = `0x01`, bánh trái = `0x02` (bắt buộc khác ID vì chung bus, không còn mặc định `0x01` cho cả hai) |
| Mode | Wheel Mode (CW Limit = CCW Limit = 0), đã set sẵn trong EEPROM |

## 3. Khung gói

```
Host → Motor:   FF FF ID LEN INS [PARAMS...] CHK
Motor → Host:   FF FF ID LEN ERR [PARAMS...] CHK
```

- `LEN` = số byte tính từ field sau LEN đến CHK (bao gồm CHK).
- `CHK` = `~(ID + LEN + INS/ERR + PARAMS...) & 0xFF`.

**Quirk firmware (không chuẩn Dynamixel):** mọi gói phản hồi luôn chèn lại
3 byte `[LEN, INS, ADDR]` của request vào đầu phần data, trước data thật:

```
FF FF ID LEN ERR echoLEN echoINS echoADDR <data thật...> CHK
                  ^^^^^^^^^^^^^^^^^^^^^^^ bỏ qua khi parse
```

Với lệnh READ 6 byte ở địa chỉ 0x24, data thật bắt đầu ở **byte offset 8**,
không phải offset 5 như Dynamixel chuẩn.

## 4. Thanh ghi (register)

| Địa chỉ | Tên | R/W | Kích thước | Ghi chú |
|---|---|---|---|---|
| `0x18` | Torque Enable | W | 1 byte | `0x00`=disable (coast, thả lỏng), `0x01`=enable |
| `0x20` | Moving Speed | W | 2 byte, signed int16 LE | range `[-400, 400]`. Dương = CCW, âm = CW |
| `0x24` | Present Position/Speed/Load | R (đọc 6 byte 1 lần) | 6 byte | Position (u16, 0~1023 = 0~360°, 1 vòng, không multi-turn), Speed (s16), Load (s16, đơn vị không xác định) |

## 5. Ví dụ khung (checksum đã verify)

```
Tiến tốc độ 200:   FF FF 01 05 03 20 C8 00 0E
Lùi tốc độ 200:    FF FF 01 05 03 20 38 FF 9F
Dừng (Speed=0):    FF FF 01 05 03 20 00 00 D6
Torque Disable:    FF FF 01 04 03 18 00 DF
READ Present Data: FF FF 01 04 02 24 06 CE
```

## 6. Quy trình điều khiển

- **Chạy**: Torque Enable (`0x18=0x01`) → chờ ổn định (mục 7) → Moving
  Speed (`0x20=±N`).
- **Dừng hẳn (thả lỏng)**: Torque Disable (`0x18=0x00`) — không cần zero
  speed trước.
- **Hold (giữ cứng)**: Torque Enable + Speed=0 (active braking, không thả
  lỏng).
- **Đọc feedback**: gửi READ liên tục theo chu kỳ (request/response, không
  có telemetry tự phát).

## 7. Độ trễ bắt buộc sau Torque Enable

Không được gửi Moving Speed ngay sau Torque Enable. Tham chiếu đã verify
chạy được trên phần cứng thật (`motor_controller.html`, hàm `sendForward`/
`sendBackward`) luôn chèn **20ms** giữa 2 lệnh này, không bao giờ gửi liền
nhau. Tài liệu giao thức gốc không ghi rõ con số này, nhưng coi là bắt buộc
vì là hành vi đã kiểm chứng thực tế (khả năng cao: firmware còn đang xử
lý/ghi thanh ghi Torque Enable, chưa sẵn sàng nhận lệnh tiếp theo ngay).

Quy trình bật động cơ đầy đủ, có độ trễ:

```
1) Gửi Torque Enable (0x18=0x01)
2) Đợi ≥ 20ms
3) Gửi Moving Speed (0x20=±N)
```

## 8. Hệ số quy đổi RPM ↔ raw speed

Dải raw speed của protocol (`±400`) trùng đúng số với tốc độ danh định
động cơ (400 RPM, xác nhận lại đúng với datasheet Kpower BL4012 mục 1).
Kết luận: **raw = RPM**.

2026-10-12 (theo yêu cầu người dùng): `tools/leg_jog_fw` (`Config::wheel_raw_max`)
và `tools/leg_jog_gui.py` (`WHEEL_RAW_MAX`) nâng trần lệnh lên **950** (tốc độ
không tải @12V) thay vì giữ ở 400 (tốc độ danh định). Đây là đánh đổi của
người vận hành, không phải sửa giới hạn phần cứng: trên 400, chạy liên tục
**có tải** có nguy cơ quá nhiệt/quá dòng vì ngoài vùng hoạt động liên tục của
motor. Không có hộp số → RPM trục bánh xe trực tiếp, khớp với Present
Position cũng là 1 vòng quay motor không qua giảm tốc.

```
rad/s = RPM × 2π / 60     (≈ RPM × 0.10472)
RPM   = rad/s × 60 / 2π   (≈ rad/s × 9.5493), clamp vào [-950, 950] (operator ceiling, xem trên)
```

Đây là suy luận từ trùng khớp số liệu datasheet (400=400), không phải câu
ghi rõ "1 raw = 1 rpm" trong tài liệu hãng hay log reverse-engineer — cần
verify lại bằng 1 phép đo thật (gửi raw cố định, đếm thời gian bánh xe quay
N vòng) nếu tốc độ bánh xe thực tế không khớp kỳ vọng.

---

## Phụ lục: tích hợp trong firmware dự án này (không thuộc protocol chuẩn)

Phần dưới đây mô tả cách file
`form/components/robot_base/src/uart_motor.c` trong repo này dùng protocol
ở trên — chỉ liên quan nếu đang đọc/sửa code đó, không cần thiết để tự
triển khai protocol từ đầu.

- Độ trễ 20ms ở mục 7 được cài bằng bộ đếm tick không chặn
  (`settle_ticks_left`, ~25ms = 5 tick × 5ms), không dùng `sleep()` vì
  control loop chạy 200Hz/5ms, chặn 20ms sẽ phá vỡ real-time budget.
- Map vào `robot_motor_command_t`:

  | Field | Nguồn |
  |---|---|
  | `joints->position_rad[wheel]` | Present Position × `2π/1024` — wrap mỗi vòng |
  | `joints->velocity_rad_s[wheel]` | Present Speed raw × hệ số mục 8 |
  | `joints->effort_nm[wheel]` | Không map — Present Load không rõ đơn vị |

- `enable_motors=false` → gửi Torque Disable mỗi tick (không im lặng, cùng
  lý do với `can_motor.c`: im lặng không đảm bảo động cơ dừng nếu có
  watchdog giữ lệnh cuối).

### Chiều quay đã kiểm tra trên robot

**Bản ghi cũ (BL4820M12, 2026-09-25, không còn đúng):** cả hai bánh dùng
`driver_sign=+1`, mỗi bánh một bus UART riêng.

**Bản ghi mới (Kpower BL4012, 2026-10-12):** sau khi đổi sang bus chung +
motor id, lệnh "tiến" ban đầu làm bánh phải (id=1) quay đúng nhưng bánh
trái (id=2) quay ngược. `MotorMapping.hpp` hiện tại: bánh trái (id=2)
`driver_sign=+1`, bánh phải (id=1) `driver_sign=-1`. Phép kiểm tra này xác
nhận chiều, không xác nhận hệ số quy đổi raw speed sang RPM ở mục 8.
