# TAP v3.3beta POS (9/10/2026) — nền điều khiển POS + bánh Kpower BL4012

Gốc: commit `c4701a6` ("Create v3.3beta POS on GIM model", 9/10/2026 08:47), ngày đầu dùng CAD GIM mới.
Chân: GDS68 chế độ vị trí (POS), động cơ bánh khi đó là CJC BL4820M12.

Thay đổi so với bản gốc (chỉ tầng phần cứng bánh, giữ nguyên bộ điều khiển):
- Động cơ bánh đổi sang Kpower BL4012: chép từ v3.5.2 các file `Esp32Bus.{hpp,cpp}`, `MotorFrameCodec.hpp`,
  `MotorMapping.hpp`, `BoardConfig.hpp`, `MotorFeedback.{hpp,cpp}`; hai bus bánh đơn dây, mỗi bánh một bus
  (khai báo `uart1_`/`uart2_` trong `AppV3.hpp`).
- `src_fw/tests/motor_feedback_tests.cpp` thay bằng bản khớp driver mới.

Kiểm tra: `pio run -e esp32-s3-v3` thành công. Chưa nạp, chưa thử trên robot.
Lưu ý: `v3_tests` đã fail sẵn ở commit gốc `c4701a6` (assert tại v3_tests.cpp:525), không do bản chuyển bánh.

## Kết quả trên robot (18/10/2026)

Điều kiện: gá thân, chân xếp thẳng hết cỡ, bật nguồn motor trong tư thế đó, `START` rồi `GO`, thả tay.
Firmware: commit `785f845` (khung chân thẳng: pose khởi động gối 1,9722 rad / 184,8 mm, `AppV3` cộng mốc hông -0,9861 và gối 1,9722).
Log: `~/tap_logs/tap_20261018_123233.log` (không nằm trong git).

| Lần | Cân bằng liên tục | Ghi chú |
| --- | --- | --- |
| 1 | 152,5 s | 20 s đầu dao động (lệnh bánh tới 20 rad/s), sau đó pitch std 0,0003 rad |
| 2 | 97,4 s | lệnh bánh tối đa 4,4 rad/s trong 10 s đầu; đợt 20 rad/s ở 80-97 s chưa rõ nguyên nhân |

Cả hai lần kết thúc bằng một trục chân báo `axis_state=1`, `axis_error=0x40` ngay trước khi `can_legs=0`;
nhiều khả năng do tắt nguồn motor, chưa xác nhận. Trim học được lớn (+0,15 đến +0,19 rad), chưa rõ nguyên nhân.
Các bản trước cùng phần cứng (v3.5.2, d5644d6, fw 5a91a8c) đều ngã trong 1,7-5 s.
Tham số bù vùng chết, lead, trễ vẫn là giá trị thời BL4820, chưa chỉnh cho BL4012.
