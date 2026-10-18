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
