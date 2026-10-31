# Luckfox Gateway — firmware ESP32-S3 (tầng dưới)

Firmware chạy trên board ESP32-S3 N16R8 của robot hai bánh chân RB R1. Nó là
**tầng dưới**: đọc cảm biến, điều khiển motor và chuyển dữ liệu cho **tầng trên**
chạy trên Luckfox (LF). Mọi quyết định điều khiển (đứng dậy, cân bằng, policy)
nằm ở LF. ESP32 chỉ thực hiện lệnh và bảo vệ phần cứng.

```
Luckfox (LF) ── UART0 921600 8N1, giao thức HCR1 ──▶ ESP32-S3 ──┬─ bus trái  UART2/GPIO15 ─ hip ID2, crank ID3, bánh ID1
  controller đứng dậy + policy                      MPU6500 (SPI) │
  gửi: 4 góc hip/crank + 2 vận tốc bánh              Console TCP  └─ bus phải  UART1/GPIO17 ─ hip ID2, crank ID3, bánh ID1
  nhận: IMU, encoder, trạng thái (200 Hz)            8023, OTA 3232
```

- LF → ESP32: bốn góc tuyệt đối hip/crank (rad) và hai vận tốc bánh (rad/s). **Không có torque command.**
- ESP32 → LF: telemetry 200 Hz (gyro, vector trọng lực, accel, góc và vận tốc khớp, cờ hợp lệ).
- Lệnh TARGET hết hạn sau 20 ms. Mất lệnh thì ESP32 tự tắt motor.

## Bố cục thư mục

| Thư mục / file | Vai trò |
|---|---|
| `src/` | Mã nguồn C++ của firmware (5 file, xem bảng bên dưới) |
| `include/gateway/` | Header của firmware và giao thức: `hcr/hcr_protocol.h` (khung HCR1, CRC), `BlMotor.hpp`, `HybridGatewayCore.hpp`, `HybridAttitude.hpp`, `ImuBiasStore.hpp`… Chứa cả file bí mật (xem mục Cấu hình) |
| `include/board/` | Cấu hình riêng của board: chân GPIO (`BoardConfig.hpp`), hướng lắp IMU (`ImuMount.hpp`), mapping dấu motor (`MotorMapping.hpp`), hình học (`GeneratedGeometry.hpp`) |
| `drivers/bl4012e/` | Driver bus một dây cho motor BL4012e (`src/joint_bus.c`). Bản kế thừa từ dự án bench `3in1`; `upstream/` là bản gốc nguyên văn, `PROVENANCE.md` ghi những gì đã sửa |
| `scripts/` | Hook của PlatformIO khi build: biên dịch driver và sinh manifest OTA (`*.ota.json`) |
| `tools/` | Công cụ chạy trên laptop: `ota_upload.py` (nạp qua Wi-Fi), `check_hcr_uart.py` và `hcr_uart_peer.py` (kiểm UART với LF), `decode_motor_wave.py` |
| `tests/` | Test chạy trên máy tính (không cần robot): giao thức, backend motor, driver bus, công cụ OTA |
| `docs/` | Tài liệu thiết kế và kết quả kiểm chứng (xem mục Tài liệu) |
| `reports/` | Nhật ký thực nghiệm theo ngày (log, kết quả đo, bản chụp mã trước khi sửa trong `before/`). Là bằng chứng, không phải mã đang dùng |
| `platformio.ini` | Cấu hình build, chỉ có môi trường `esp32-s3-hybrid-mcu` |
| `.local/` (ẩn, git-ignored) | Khóa OTA của laptop và MAC đích |
| `.pio/` (ẩn) | Thư mục build do PlatformIO sinh ra |

### Các file trong `src/`

| File | Việc |
|---|---|
| `HybridGatewayApp.cpp` | Điểm vào chương trình: UART0 tới LF, hàng đợi, console USB/TCP 8023, Wi-Fi, OTA, tác vụ service 2 kHz |
| `HybridGatewayCore.cpp` | Logic giao thức: bắt tay HELLO/ACK, epoch, telemetry, nhận TARGET, TTL, E-stop, latch lỗi |
| `BlMotor.cpp` | Giao thức motor BL4012e: đọc/ghi thanh ghi, mapping dấu và zero, retry, ACK theo batch của cả hai bus |
| `EspBlMotor.cpp` | Gắn backend motor vào phần cứng: UART2 (trái) và UART1 (phải), hai tác vụ worker |
| `Mpu6500Spi.cpp` | Đọc IMU MPU6500 qua SPI |

Header `HybridAttitude.hpp` ước lượng vector trọng lực và bias gyro; `ImuBiasStore.hpp`
lưu bias vào flash (NVS).

## Phần cứng và chân

| Chức năng | Chân |
|---|---|
| Liên kết LF (HCR1) | UART0, TX GPIO43, RX GPIO44, 921600 baud |
| Bus motor trái | UART2, GPIO15 (DATA, 1 Mbaud, một dây) |
| Bus motor phải | UART1, GPIO17 (DATA, 1 Mbaud, một dây) |
| IMU MPU6500 | SPI, GPIO10–14 |
| Console | TCP 8023 (Wi-Fi LUMI), hoặc USB native GPIO19/20 |

CP2102 cũng nối GPIO43/44. Khi dùng UART0 cho LF thì phải tách TX của CP2102
khỏi GPIO44, và tách LF khỏi RX0 khi nạp bằng cáp USB.

## Build và kiểm tra

Chạy từ thư mục gốc của repository (nếu `pio` không có trong PATH thì dùng
`~/.platformio/penv/bin/pio`):

```bash
pio run -e esp32-s3-hybrid-mcu          # build, ra .pio/build/esp32-s3-hybrid-mcu/firmware.bin

python3 tests/check_hybrid_gateway.py   # giao thức, ACK, TTL, E-stop, UART giả lập
python3 tests/check_bl_motor.py         # backend motor hai bus với driver giả lập
python3 tests/check_inherited_bus.py    # driver bus: echo, phản hồi muộn, deadline
python3 -m unittest discover -s tests -p 'test_*.py'
```

Các test trên chạy bằng mô phỏng phần mềm. Chúng không chứng minh thời gian thực
của bus, đáp ứng cơ học của motor hay khả năng cân bằng của robot.

## Cấu hình thông tin riêng

Ba file trong `include/gateway/` chứa mật khẩu và không được commit. Tạo từ bản mẫu:

```bash
cd include/gateway
cp WifiSecrets.example.hpp WifiSecrets.hpp          # Wi-Fi mặc định
cp HcrWifiSecrets.example.hpp HcrWifiSecrets.hpp    # mạng LUMI (ưu tiên)
cp OtaSecrets.example.hpp OtaSecrets.hpp            # hash khóa OTA
```

Khóa OTA phía laptop nằm ở `.local/hybrid_ota.json`. Thiếu các file này vẫn build
được nhưng bản build không có Wi-Fi và OTA.

## Vận hành

Console (TCP 8023, `nc <ip> 8023`, `HELP` để xem lệnh): `STATUS`, `CAL_IMU`,
`MOTOR_HOME`, `MOTOR_ZERO`, `CLEAR_FAULT`, `ESTOP`, `MOTOR_BUS_TEST`.
ARM, DISARM, RESET và VELOCITY không đi qua ESP32: chúng được gửi tới LF.

Trình tự trước khi ARM (sau mỗi lần ESP32 reset phải làm lại):

1. `CAL_IMU` khi robot được giữ hoàn toàn yên, để đo bias gyro.
2. `CLEAR_FAULT` nếu còn lỗi cục bộ (lệnh này xóa cả home).
3. `MOTOR_HOME` khi hai chân đã ở tư thế home.
4. `RESET` trên LF để thoát FAULT, rồi ARM từ GUI của LF.

Nạp qua Wi-Fi:

```bash
python3 tools/ota_upload.py --status --host <ip>          # chỉ đọc trạng thái
python3 tools/ota_upload.py --build --host <ip>           # build, kiểm, rồi nạp
```

Công cụ từ chối nạp khi robot đang ARM hoặc motor chưa xác nhận OFF.

## Giới hạn và vấn đề đã biết

- Bias IMU và zero home chỉ nằm trong RAM, mất khi ESP32 reset. Bản có lưu bias vào NVS
  đã viết và qua test host, **chưa được nạp lên robot**.
- Bản đang chạy trên robot không nhận được OTA (báo `Aborted`, `error=4` ở bước kết thúc,
  kể cả khi nạp lại chính nó), nên lần nạp tiếp theo phải bằng USB. Xem
  [docs/validation-bus-recovery.md](docs/validation-bus-recovery.md).
- Một lỗi bus hoặc lỗi feedback bị latch cho tới `CLEAR_FAULT` (và làm mất home). Hai bus từng
  mất phản hồi luân phiên, nghi do nguồn hoặc đầu nối. Phân tích trong
  [docs/validation-bus-recovery.md](docs/validation-bus-recovery.md).
- Chưa ARM, chưa chạy chuyển động và chưa kiểm đáp ứng motor trên robot trong các đợt tích hợp.

## Tài liệu

| File | Nội dung |
|---|---|
| [docs/overview.md](docs/overview.md) | Tổng quan tầng dưới, UART, console, luồng chạy, bring-up |
| [docs/hcr1-protocol.md](docs/hcr1-protocol.md) | Giao thức HCR1 giữa LF và ESP32 |
| [docs/motor-integration.md](docs/motor-integration.md) | Luồng motor, mapping, gain, quy trình zero |
| [docs/ota.md](docs/ota.md) | Nạp firmware qua Wi-Fi |
| [docs/audit-data-flow.md](docs/audit-data-flow.md) | Đối chiếu luồng dữ liệu theo mã nguồn |
| [docs/validation-gateway.md](docs/validation-gateway.md) | Kết quả kiểm chứng gateway |
| [docs/validation-bus-recovery.md](docs/validation-bus-recovery.md) | Phục hồi bus, các lần OTA lỗi |
| [docs/validation-bl4012e-reuse.md](docs/validation-bl4012e-reuse.md) | Kiểm chứng việc tái dùng driver BL4012e |
| [drivers/bl4012e/PROVENANCE.md](drivers/bl4012e/PROVENANCE.md) | Nguồn gốc và thay đổi của driver bus |
