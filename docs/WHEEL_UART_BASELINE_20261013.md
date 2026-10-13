# Wheel UART baseline — 2026-10-13

Áp dụng cho firmware v3.3beta và nhánh v3.5, bao gồm v3.5.1 (commit
`33ca61d` đã nằm trong lịch sử v3.5; không có nhánh v3.5.1 riêng).

## Cơ chế mặc định

Nguồn tham chiếu: bản thử nghiệm của người vận hành
`ROBOT-POC-ESP32_UART_HALF_DUPLEX-main/esp32s3_uart2`.
Hai motor Kpower BL4012 dùng hai bus độc lập, 1 Mbaud, 8N1.

| Bánh / transport bus | UART phần cứng | DATA TX + RX | Chân RX riêng để input floating | ID | driver_sign |
| --- | --- | --- | --- | --- | --- |
| Trái / 0 / LW (joint 2) | UART2 | GPIO15 | GPIO16 | 2 | +1 |
| Phải / 1 / RW (joint 5) | UART1 | GPIO17 | GPIO18 | 1 | -1 |

DATA truyền push-pull, sau stop bit cuối ngắt output matrix và tắt output
để nhận trên chính DATA. DATA giữ pull-up; GPIO16/18 không kéo lên/xuống.
Mỗi bus có một worker sở hữu TX, RX qua IDF; TX không dùng software buffer.
Gói tối đa 16 byte được gửi trực tiếp FIFO, chờ hardware idle rồi nhả bus
trong critical section giới hạn 500 µs. Không dùng thời điểm task thức dậy
sau `uart_wait_tx_done` để nhả DATA.

Lọc echo toàn bộ request; khi prefix khớp một phần rồi khác, đưa prefix
trở lại parser để không mất đầu phản hồi thật. Codec giữ checksum và quirk
ba byte metadata trong response. Boot đọc/kiểm tra speed mode trước khi
cho phép torque enable. Hàng đợi giữ thứ tự torque/speed, gộp speed mới
nhất; giữ kiểm tra freshness, stale feedback và fault của controller.
`WBUS` báo timeout, bad frame, gap, `tx_hold_us` và `tx_late`.

Port v3.5 chỉ thay transport, mapping và diagnostics; không đổi điều khiển
dòng chân, Get_Iq, jump, hình học, gains hay trần raw speed 1000 của codec
v3.5. Công cụ bench `tools/leg_jog_fw` có luồng riêng, không bị thay bằng
transport của app V3 trong lượt port này.

## Bằng chứng robot v3.3beta

Firmware đã nạp: SHA256
`6ac2bfa5b71db1b5df9830b450d00108e34d2e3a0b658cec43ca4fa6a00e9fdb`.
Log người vận hành: `tap_20261013_150747.log` trong thư mục `tap_logs`.

- Control case `tap_20261013_150521.log`: chờ task TX-done giữ DATA
  180–309 µs cho READ 80 µs, gây nhiều timeout. Sau sửa, thời gian giữ TX
  khoảng 83–110 µs và `tx_late=0` trong phần đã kiểm tra.
- Idle sau boot hơn 80 s: 401 báo cáo SENS sau giây 3 đều
  `uart_wheels=1`; hơn 32.000 phản hồi mỗi bánh, chưa có timeout/bad frame.
- Lượt Balance cuối 216.152–246.573 s: khoảng 30.421 s rồi người vận hành
  DISABLE; 151 SENS đều `uart_wheels=1`, max |pitch| log 0.147 rad.
  Không tăng timeout/bad frame trong cửa sổ này; người vận hành xác nhận
  robot cân bằng thành công.
- Hai lượt trước vẫn fault mất feedback tại 170.445 s (Balancing) và
  191.974 s (Ready). Lượt thành công là baseline được chấp nhận, chưa
  chứng minh hết mất feedback trong mọi điều kiện tải hoặc thời gian dài.

## Kiểm tra port v3.5

Worktree `Lumi-Tanker-v35`, nhánh v3.5, nền `c0ec069`.
Ngày 2026-10-13:

```sh
~/.platformio/penv/bin/pio run -e esp32-s3-v3
~/.platformio/penv/bin/pio run -e esp32-s3-v3-single-loop
cmake -S . -B /tmp/tap-v35-uart-host -DBUILD_TESTING=ON -DTAP_BUILD_SIMULATOR=OFF
cmake --build /tmp/tap-v35-uart-host --target motor_frame_codec_tests motor_feedback_tests v3_tests v3_current_torque_tests -j4
ctest --test-dir /tmp/tap-v35-uart-host -R '^(motor_frame_codec_tests|motor_feedback_tests|v3_tests|v3_current_torque_tests)$' --output-on-failure
```

Hai firmware build pass. RAM 157980/327680 byte; flash bản RTOS
1115497/6553600 byte, single-loop 1114889/6553600 byte. Bốn test host pass; codec có 41/41 kiểm tra.
Fixture feedback kiểm tra đúng ID2 trái/ID1 phải, dấu +1/-1 và từ chối
response sai ID. Test host không chạy GPIO/FIFO/turnaround ESP32.

**v3.5/v3.5.1 chưa nạp và chưa thử robot với port này.** Không chạy planar
sim hoặc MuJoCo cho thay đổi bus; mô phỏng controller không chứng minh
timing điện của UART.
