# Trạng thái tích hợp firmware ESP32-S3

Cập nhật: 2026-09-25 (luồng tháo fixture qua BLE)
Phạm vi: code cho PCB ESP32-S3-WROOM-1-N16R8; chưa flash hoặc vận hành động cơ.

## Kiến trúc hiện tại

Firmware được build bằng PlatformIO với Arduino framework trên Espressif32.
`firmware_esp32/src/main.cpp` chỉ là entry point. Ứng dụng nằm trong
`firmware_esp32/src/FirmwareApp.cpp`; sketch thử MPU6500/BLE cũ đã được thay
bằng tích hợp các peripheral và `FirmwareControlLoop`.

```text
MPU6500 SPI -----------+
GDS68 x4 / TWAI -------+--> MotorFeedback --> RobotState
BL4820 x2 / UART1/2 ---+                         |
                                                  v
Dabble BLE --> CommandEndpoint --> FirmwareControlLoop
                                                  |
                                                  v
                                      ControllerActuatorBridge
```

## Đã triển khai trong code

| Phần | Module | Tình trạng |
|---|---|---|
| TWAI classic CAN | `Esp32TwaiPort` | GPIO4/5, standard 11-bit frames, bitrate cấu hình 500 kbit/s, RX/TX queue |
| Hai UART motor | `Esp32UartPort` | UART1 GPIO17/18 và UART2 GPIO15/16, 1 Mbaud 8N1, parser response có kiểm checksum |
| Feedback encoder | `MotorFeedback` | GDS68 MIT feedback đổi theo mapping/sign; BL4820 position được unwrap và speed đổi sang rad/s |
| MPU6500 | `Mpu6500Spi` | SPI GPIO10-13, INT GPIO14, WHO_AM_I, cấu hình 500 Hz và burst read |
| Control runtime | `FirmwareApp` | Lấy sensor, dựng `RobotState`, gọi `FirmwareControlLoop`/actuator bridge ở nhịp mục tiêu 500 Hz |
| An toàn khởi động | `FirmwareApp` | Boot gửi trạng thái disable; enable cần sensor mới, dấu bánh/scale raw và frame IMU được xác nhận |
| Xác nhận fixture | Dabble GamePad Start | Một lần nhấn Start gửi `ENABLE` và yêu cầu tự tháo fixture sau khi sensor qualify |
| Dabble BLE | `DabbleGamepadMapper` | Điều khiển enable/tháo fixture/drive/pose/stop qua BLE; timeout/disconnect yêu cầu disable |

Trong lúc chưa enable, firmware thăm dò UART Present Data và gửi CAN MIT probe
với `Kp=Kd=T_ff=0` để lấy feedback theo frame hai chiều đã ghi trong
`GDS68_DRIVER.md`. Các probe này không yêu cầu axis closed-loop. Nếu firmware
driver trên motor không phản hồi MIT khi axis đang IDLE, readiness gate sẽ
không qua; khi đó cần cấu hình telemetry encoder của GDS68 hoặc điều chỉnh
quy trình bring-up sau khi xác nhận giao thức trên bench.

## Khóa enable còn hiệu lực

- Wheel UART ID là `1` trên hai bus riêng. Người dùng đã xác nhận trên robot
  ngày 2026-09-25: với cả hai bánh, tốc độ driver dương làm robot tiến và âm
  làm robot lùi. `driver_sign=+1` cho cả hai; `kWheelDirectionsVerified=true`.
- `raw = RPM` của BL4820 là suy luận từ dải ±400 và tốc độ danh định 400 RPM,
  chưa được đo. `kWheelSpeedScaleVerified` cũng đang `false` và khóa enable.
- Mapping CAN GDS68 đã xác nhận trên robot: RH node 1/sign -1, RK node 2/sign
  +1, LH node 3/sign +1, LK node 4/sign -1.
- GDS68 đã được bring-up trên USB-CAN và ESP32 thử nghiệm ở 500 kbit/s; firmware
  robot cũng đặt 500 kbit/s để khớp với cấu hình điều khiển đã hoạt động.
- Driver GDS68 phải được cấu hình CAN đúng node/baud, hiệu chuẩn motor và
  encoder, và lưu cấu hình trước khi vào closed-loop. Firmware không tự chạy
  quy trình hiệu chuẩn driver. Payload/giá trị `Set_Limits` chưa có trong tài
  liệu hiện tại, nên firmware không tự gửi frame giới hạn.
- Dấu IMU đang theo PCB/core hiện tại: pitch từ accel X/Z và gyro Y.
  `kImuFrameVerified` hiện là `false`; cần xác nhận chiều trục lắp thực tế
  bằng log trước khi cho phép enable.
- Nút GPIO6 chỉ xác nhận tháo fixture; nó không phải emergency stop phần
  cứng. ESTOP hiện là lệnh phần mềm qua Dabble.

## Mapping chân PCB

Nguồn đối chiếu: `docs/data/POC - Robot version 1/Images_attachments/R1-POC-V1.0.PDF`
và bảng mapping trong `POC - Robot version 1.md`.

| Ngoại vi | Chân ESP32-S3 | Cấu hình |
|---|---|---|
| TWAI TX/RX | GPIO4/GPIO5 | Classic CAN, 500 kbit/s trong firmware |
| UART1 TX/RX | GPIO17/GPIO18 | Motor trái, 1 Mbaud 8N1 |
| UART2 TX/RX | GPIO15/GPIO16 | Motor phải, 1 Mbaud 8N1 |
| MPU6500 CS/MOSI/SCLK/MISO | GPIO10/11/12/13 | SPI mode 0, 1 MHz |
| MPU6500 INT | GPIO14 | Input, polling hiện dùng làm đường đọc chính |
| UART0 debug | GPIO43/GPIO44 | CP2102, 115200 baud |
| Nút / LEDs | GPIO6 / GPIO39-41 | GPIO6 không tham gia luồng điều khiển fixture; LEDs active-high |

## Build và bằng chứng

Lệnh build firmware:

```bash
pio run -e esp32-s3-poc
```

Đã build pass ngày 2026-09-25 cho profile `ESP32S3-R8N16` bằng PlatformIO
7.1.3 / Arduino-ESP32 2.0.17. PlatformIO được cài trong môi trường tạm của
phiên làm việc, nên trên máy chưa có lệnh `pio` toàn cục. Kết quả target:

- RAM tĩnh: 49,500 / 327,680 byte (15.1%).
- Flash: 987,409 / 6,553,600 byte (15.1%); firmware image được tạo thành công.
- Host tests `controller_actuator_bridge_tests`, `ble_command_endpoint_tests`
  và `motor_feedback_tests` đều compile/run pass trực tiếp với g++.
- Chưa chạy CTest từ build tree: thư mục `build/` chỉ còn cache, còn configure
  build sạch bị chặn vì môi trường hiện không có MuJoCo.

Firmware CAN 500 kbit/s khớp với bitrate đã dùng khi điều khiển GDS68 qua
USB-CAN và ESP32 thử nghiệm; build và upload qua CP2102 `/dev/ttyUSB0` đã pass,
flash được xác minh và ESP reset. Cần kiểm tra lại feedback trên harness robot;
build/upload pass không chứng minh đủ phản hồi từ bốn driver hoặc khả năng cân
bằng vật lý. Các khóa enable cho wheel speed scale và IMU frame vẫn giữ.

## Bring-up phần cứng còn lại

1. Kiểm tra lại `pio run -e esp32-s3-poc`; chỉ nạp khi người vận hành quyết định.
2. Kiểm tra boot log, MPU6500 WHO_AM_I, seed estimator và feedback CAN/UART
   trong trạng thái disable.
3. Xác nhận cấu hình node/baud/calibration từng GDS68; xác nhận CAN 500 kbit/s
   trên harness.
4. Chiều hai bánh đã được xác nhận. Đo raw-speed scale trên bench rồi mới đặt
   `kWheelSpeedScaleVerified=true` trong `MotorMapping.hpp`.
5. Xác nhận chiều trục IMU, timestamp/freshness và mất frame; sau đó mới xem
   xét một motor trên bench với lệnh nhỏ.

Luồng tháo fixture được đưa vào Dabble GamePad: một lần nhấn Start gửi cả
`ENABLE` và yêu cầu `FIXTURE_RELEASE`. State machine vẫn chờ `SensorQualify`
hoàn tất trước khi rời `FixtureHold` và bắt đầu `BalanceRamp`; không cần nút
GPIO6 hay lần nhấn BLE thứ hai. Các cờ xác nhận wheel speed scale và IMU frame
vẫn khóa `ENABLE` cho đến khi có bằng chứng hiệu chuẩn phần cứng.

Không có bước nào trong lượt code này flash board hay phát lệnh tới động cơ
thật.

## Chẩn đoán CAN trên robot (2026-09-25)

Triệu chứng: `can_legs=0`. ESP32 nhận heartbeat `0x001`/encoder `0x009` của cả
4 node nhưng mọi frame ESP32 phát đều lỗi, TWAI vào `BUS_OFF`.

Nguyên nhân (đã xác nhận): **bus bị terminate quá mức**. Cả 4 GDS68 đều bật
`can.config.enable_r120=True` (thiết lập từ lúc bring-up từng driver), cộng R5
120Ω trên board → khoảng 24Ω. Frame ESP32 phát không được driver nào ACK.
Bằng chứng: test 1 frame single-shot/giây tới ID không dùng `0x7C0` thất bại
29/29; ở error-passive TEC đứng yên (lỗi ACK, không phải lỗi bit), trong khi
các driver vẫn ACK nhau.

Khắc phục: tắt `enable_r120` trên node 1-4 qua USB (`odrivetool_legacy`,
`save_configuration()`), đã đọc lại sau reboot. Sau đó test TX 11/11 thành
công, TEC=0. Việc còn lại: bật lại `enable_r120` trên đúng driver ở đầu xa
nhất của dây để bus về ~60Ω.

Kết quả ở chế độ bình thường: bus RUNNING, TEC=0, không bus-off; mỗi node trả
MIT `0x008` ~87/s (driver trả MIT probe cả khi IDLE) và `0x009` 100/s;
`can_legs=1`.

Thay đổi firmware giữ lại:

- `Esp32TwaiPort::service()` phục hồi sau `BUS_OFF`.
- `MotorFeedback` nhận phản hồi MIT với DLC 6: driver chỉ gửi byte 0-5 và
  kiểm tra `dlc == 8` cũ đã bỏ qua toàn bộ phản hồi MIT (xác nhận trên robot:
  byte 0 = node ID đúng, không có echo sai).
- `MotorFeedback` nhận thêm `0x009` khi không có MIT mới.
- Tùy chọn bench `kCanListenOnly` và `kCanTxSingleFrameTest` (mặc định
  `false`; cả hai luôn khóa enable).
- Log `CANDIAG` mỗi giây: trạng thái bus, ID nhận được, heartbeat và estimate.

Lưu ý quan trọng: `pos_estimate` (0x009) — và theo quan sát cả vị trí MIT — là
độ dịch chuyển tính từ lúc driver bật nguồn (reset về 0 sau mỗi reboot driver);
encoder tuyệt đối `pos_abs` là một vòng rotor trước hộp số 8:1. Góc khớp tuyệt
đối cần bước homing/offset trước khi enable;
`kGds68EncoderEstimateScaleVerified=false` giữ khóa enable.

Tải bus: ở trạng thái disable firmware gửi Set_Axis_State(IDLE) cho 4 node mỗi
tick 2 ms cộng MIT probe 10 ms (~2000 frame/s, `arb_lost` cao). Nên giảm nhịp
gửi IDLE trước khi thêm tải điều khiển.
