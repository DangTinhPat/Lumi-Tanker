# Kiểm chứng tầng dưới

## Tích hợp motor ngày 2026-10-30 (trạng thái mới nhất)

Firmware đã dùng `BlMotorBackend`/`EspBlMotor` cho cả hai bus; LEFT UART2/GPIO15,
RIGHT UART1/GPIO17 theo đính chính của người dùng. Có homing thủ công, encoder
unwrap/freshness, gửi MIT/RPM, applied ACK sau hai bus và disable bất đồng bộ.
Chi tiết [MOTOR_INTEGRATION.md](MOTOR_INTEGRATION.md); kết quả build/test mới
và hashes ở `reports/motor_integration_20261030/` cấp gốc worktree.

`check_bl_motor.py --runtime ...` chạy backend/codec motor thật với mock driver
và LF Core qua 2.700 chu kỳ, HOME/STANDUP/WARMUP/MODEL, E-stop/reset. Kiểm cả
lỗi write/enable ACK, CRC/status, TTL khi service ngừng, chỉ một bus xong,
reference sau quantization/filter, range, unwrap, gain max và enable bits.
Không chạy motor vật lý, không khẳng định gain hiện tại tương đương MuJoCo.

Các số liệu dưới đây là lịch sử trước tích hợp motor, không phải trạng thái
backend hoặc kích thước binary mới.

## Bổ sung kiểm tra luồng ngày 2026-10-30

Đã đối chiếu lại `4 góc hip/crank + 2 vận tốc bánh`, 4 velocity FF của recovery,
payload TARGET 52 B, actor obs32, knee hinge feedback và applied ACK. Không có
torque command LF→MCU. Bài C++ mới kiểm tra mọi target/FF đi qua parser→backend
và TELEMETRY.applied_target đúng thứ tự, ACK chỉ sau apply, không cần TORQUE_VALID.

`check_hybrid_gateway.py --runtime ...` chạy lại PASS với ASan/UBSan,
2.700 chu kỳ liên thông LF Core–MCU Core và UART PTY. Build hybrid chạy lại
PASS, RAM 52.596 B, flash 755.553 B. Runtime host có 22 bài test PASS,
gồm ONNX thật 100 bước; codec `joint_bus.c` của BL4012e cũng qua mock I/O.

HIL LF/NPU–MuJoCo có lượt đủ 33 s khi không có torque feedback, nhưng một lượt
trước đó fault vì NPU inference 16,8 ms. Không ghi tất cả HIL là PASS hoặc coi
đây là kiểm tra motor thật. Chi tiết ở [HYBRID_FLOW_AUDIT.md](HYBRID_FLOW_AUDIT.md)
và `hybrid_runtime/reports/flow_audit_20261030/SUMMARY.md`.

Trong đợt audit trước tích hợp này, BL4012e chưa được ghép vào backend HCR; phần UART vật lý, homing, sign/zero, timing motor và gain
thực tế tiếp tục chưa xác minh.

## Build

PlatformIO Espressif32 7.1.3, Arduino ESP32 2.0.17, Xtensa GCC 8.4.0,
board `4d_systems_esp32s3_gen4_r8n16`, C++17, -O2; PSRAM không được yêu cầu.

| Target | Kết quả |
|---|---|
| `esp32-s3-hybrid-mcu` (mặc định) | PASS; RAM static 52.596 B; flash 755.553 B |
| `esp32-s3-v3` | PASS; kiểm tra build đường firmware cũ |
| `esp32-s3-v3-single-loop` | PASS; kiểm tra build cấu hình vòng đơn cũ |

RAM trên là báo cáo link, chưa gồm heap cấp cho queue/task/Wi-Fi lúc chạy.
Không đo thời gian chu kỳ hoặc heap tối thiểu trên ESP32 trong lượt này.

## Host/contract

Lệnh:

```sh
python3 firmware/esp32/tests/check_hybrid_gateway.py \
  --runtime '/home/dvt/contoller hybrid/deploy/hybrid_runtime'
```

Kết quả cuối cùng: PASS, có AddressSanitizer và UndefinedBehaviorSanitizer.

- IMU: seed stationary, bias ba trục, dấu pitch/roll, gravity khi nghiêng,
  mất mẫu cần seed lại, NaN bị loại.
- Wire: CRC, parser byte stream/resync, sequence wrap/replay, epoch mismatch,
  timestamp rollback, source sequence phải có trong history và còn mới.
- Motor boundary: thiếu driver không ARM; ACK sau khi apply; không clamp target
  vào [-1,1]; driver từ chối/nonfinite feedback gây lỗi; telemetry giữ float finite.
- Safety/session: deadline 20 ms không được gia hạn bởi duplicate/gói trễ;
  DISARM không xóa lỗi; reset/epoch mới không tự ARM; E-stop giữ latch cả khi
  mất link rồi reconnect; disarmed link timeout yêu cầu phiên mới.
- Bench CLI qua pseudoterminal: nhận telemetry MCU, kiểm tra mọi TARGET đều
  DISABLED; không mở cổng UART vật lý.
- Lõi LF thật `hybrid.core.Core` ↔ lõi MCU C++: 2.700 chu kỳ telemetry 200 Hz,
  13,5 s thời gian giả lập; đi qua HOME/STANDUP/WARMUP/MODEL, applied ACK,
  E-stop và reset. Policy, logic và motor là test doubles; không chạy RKNN
  hoặc mô phỏng cơ học trong bài này.
- MCU thiếu motor: chỉ bật IMU_OK; lõi LF trả invalid telemetry FAULT=2;
  ARM bị từ chối. Kết quả này được giữ trung thực, không giả encoder.
- `git diff --check`: PASS; Python tools biên dịch cú pháp thành công.

## Nguồn giao thức

Codec C và Python được chép nguyên bản từ gói LF local
`contoller hybrid/deploy/hybrid_runtime`, lần lượt `mcu/hcr_protocol.h` và
`hybrid/protocol.py`. Không sửa wire format.

SHA256 codec C:
`15fe9f82a8ce47fb00d1c5ff29c03c1c951a454426466c991eba2c8737e15462`

SHA256 codec Python:
`42c5b774ec5b82a117362274493aa16e7699d8379deaec2402d30bf04a941127`

## Còn cần phần cứng

Chưa flash ESP32; chưa đo UART giữa LF/ESP32, hướng IMU trên board, nhiễu/jitter,
hoặc đáp ứng motor. Cần xử lý kết nối CP2102 chung UART0 trước khi đấu LF,
chọn UART serial ở LF thay config Unix socket, và kiểm tra mapping/homing/gain/timing của backend đã tích hợp
trước khi vận hành hybrid. Tài liệu thao tác ở
[`HYBRID_MCU.md`](HYBRID_MCU.md).
