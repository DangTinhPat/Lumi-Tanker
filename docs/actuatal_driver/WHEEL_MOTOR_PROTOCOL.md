# Protocol UART động cơ bánh xe — CJC BL4820M12

Dùng cho 2 động cơ bánh xe 12V, mỗi động cơ trên 1 bus UART riêng (UART1 =
LeftWheel, UART2 = RightWheel — full-duplex, TX/RX riêng, không phải bus
chia sẻ). Nguồn: `Motor_Communication_Guide.html` + `motor_controller.html`
(reverse-engineer từ log thật, đã verify chạy được trên phần cứng thật),
đối chiếu với datasheet động cơ tại cjcmotor.com/motor/bl4820m12.

**Đây không phải protocol chuẩn Dynamixel gốc** — chỉ có khung gói giống
Dynamixel v1.0. Cách encode tốc độ (signed, không phải magnitude+bit
hướng) và quirk echo 3 byte trong response là tùy biến riêng của firmware
này.

## 1. Thông số động cơ (BL4820M12)

| | |
|---|---|
| Điện áp danh định | 12V |
| Tốc độ danh định | 400 RPM |
| Tốc độ không tải | 1000 RPM |
| Mô-men | 130 mN·m |
| Hộp số | Không có (truyền động trực tiếp) |

## 2. Thông số bus

| | |
|---|---|
| Baud | 1,000,000 bps, 8N1, không parity |
| Motor ID | `0x01` (mặc định — không cần đổi, mỗi bus chỉ có 1 motor) |
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
động cơ BL4820M12 (400 RPM). Kết luận: **raw = RPM**, giới hạn ở tốc độ
danh định (an toàn khi chạy liên tục), dù motor vật lý có thể lên tới
1000 RPM không tải. Không có hộp số → RPM trục bánh xe trực tiếp, khớp với
Present Position cũng là 1 vòng quay motor không qua giảm tốc.

```
rad/s = RPM × 2π / 60     (≈ RPM × 0.10472)
RPM   = rad/s × 60 / 2π   (≈ rad/s × 9.5493), clamp vào [-400, 400]
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
