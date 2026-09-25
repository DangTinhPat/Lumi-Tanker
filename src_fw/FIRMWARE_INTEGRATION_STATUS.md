# Trạng thái tích hợp firmware ESP32-S3

Ngày cập nhật: 2026-09-24  
Phạm vi: nhánh `fw`, chỉ đánh giá code và mô phỏng host; chưa flash hoặc chạy phần cứng.

## Kết luận ngắn

Khối controller portable đã được nối với một lớp firmware boundary có thể
phát frame xuống driver động cơ. Tuy nhiên `src_fw` hiện vẫn là lớp tích hợp
được build trên máy tính, chưa phải firmware ESP-IDF hoàn chỉnh.

Đã sẵn sàng ở mức code:

- Controller core LQR/FSM và estimator IMU.
- Chuẩn hóa `RobotState -> MotorCommand`.
- Chuyển lệnh chân sang frame GDS68 CAN MIT.
- Mapping GDS68 thực tế đã kiểm chứng: RH=ID1 sign -1, RK=ID2 sign +1,
  LH=ID3 sign +1, LK=ID4 sign -1.
- Chuyển lệnh bánh sang frame UART BL4820.
- Watchdog lệnh BLE và mapping Dabble GamePad.
- Khai báo GPIO theo sơ đồ PCB.

Chưa sẵn sàng để nạp ESP32-S3:

- Driver SPI đọc MPU6500.
- Driver sáu encoder.
- Adapter ESP-IDF cho TWAI/CAN và UART thật.
- Task FreeRTOS, đồng bộ mailbox giữa BLE và control loop.
- Hiệu chuẩn sensor/motor trên phần cứng.
- CMake/partition/boot target ESP-IDF hoàn chỉnh.

Đã setup thêm PlatformIO:

- `platformio.ini` ở thư mục gốc, environment `esp32-s3-poc`.
- Board profile: ESP32-S3 R8N16 tương thích PCB N16R8.
- Platform: Espressif32 `7.1.3`.
- DabbleESP32: `1.5.1`, pin cố định để tương thích Arduino framework hiện tại.
- Firmware bring-up nằm ở `firmware_esp32/src/main.cpp`; hiện chỉ khởi tạo
  Serial, đọc MPU6500 qua SPI, chạy `ImuStateEstimator` và quảng bá Dabble
  `TAP-RB`; không phát lệnh motor.
- Build PlatformIO đã pass trên host; cấu hình hiện compile cả source
  `src_fw` và `tap_control/core`, không copy code vào project Arduino.

Thiết lập hệ điều hành còn thiếu quyền serial: user `dvt` chưa thuộc nhóm
`dialout`, nên cần chạy `sudo usermod -aG dialout dvt` rồi đăng nhập lại.

## Bring-up IMU và BLE

Firmware test hiện tại dùng đúng GPIO trong sơ đồ PCB:

```text
CS=10, MOSI=11, SCLK=12, MISO=13, INT=14
```

Khi chạy trên PCB:

```bash
pio run -e esp32-s3-poc -t upload
pio device monitor -e esp32-s3-poc
```

Kết quả mong đợi:

- `WHO_AM_I=0x70` hoặc `0x71` và `PASS`: SPI/CS/MPU6500 phản hồi.
- Sau khoảng 250 ms đứng yên: `seed=1`, LED2 sáng; đây là dấu hiệu bộ lọc
  đã nhận đủ cửa sổ khởi tạo, không phải xác nhận hiệu chuẩn cơ khí.
- Dòng `IMU ...` ổn định khi để yên; xoay nhẹ quanh trục pitch thì `gy` và
  `pitch` đổi cùng chiều theo quy ước hiện tại.
- Mở Dabble GamePad, chọn thiết bị `TAP-RB`: LED3 sáng và Serial báo
  `BLE connected`. Nút Start/Select chỉ kiểm tra command endpoint; không có
  motor nào được gọi trong firmware test này.

Đèn LED chỉ là chỉ báo bring-up: LED1 = nhận diện MPU, LED2 = estimator đã
seed, LED3 = Dabble đã kết nối. Không có oscilloscope/loopback nên chưa thể
chứng minh chất lượng mức điện áp hay continuity của từng đường SPI bằng code.

## Luồng điều khiển dự kiến

```text
MPU6500 SPI + 6 encoder
        |
        v
Sensor adapter / timestamp / calibration
        |
        v
RobotState
        |
        v
EmbeddedController (FSM + LQR + pose planner + IMU estimator)
        |
        v
MotorCommand
        |
        v
ControllerActuatorBridge
        |----------------------|
        v                      v
GDS68 CAN MIT             BL4820 UART
4 leg motors             2 wheel motors
```

`FirmwareControlLoop::step()` hiện nhận `RobotState`, chạy controller rồi
đưa `MotorCommand` vào actuator bridge. Vì vậy phần đọc sensor vẫn là trách
nhiệm của firmware application, chưa nằm trong `src_fw`.

## Các module hiện có

| Module | Trạng thái | Ghi chú |
|---|---|---|
| `BoardConfig.hpp` | Có | GPIO, bitrate CAN, baud UART, motor IDs và dấu GDS68 đã kiểm chứng |
| `BusTransport` | Có khung | Callback transport, chưa gắn ESP-IDF driver |
| `FirmwareControlLoop` | Có | Boundary một bước: sensor vào, frame ra |
| `ControllerActuatorBridge` | Có | Gửi frame CAN/UART theo nhịp 2 ms |
| `BleCommandEndpoint` | Có | Watchdog 500 ms, enable/disable/estop/drive/height |
| `DabbleGamepadMapper` | Có | Mapping nút GamePad sang command endpoint |
| MPU6500 driver | Chưa có | Chỉ mới khai báo GPIO SPI |
| Encoder driver | Chưa có | Chưa có capture/đọc sáu encoder |
| ESP-IDF application | Chưa có | Chưa có `app_main`, task và peripheral init |

## GPIO hiện tại

### MPU6500 SPI

| Tín hiệu | GPIO | Trạng thái |
|---|---:|---|
| CS | 10 | Đã khai báo |
| MOSI | 11 | Đã khai báo |
| SCLK | 12 | Đã khai báo |
| MISO | 13 | Đã khai báo |
| INT | 14 | Đã khai báo, tùy chọn |

### Motor và giao tiếp phụ

| Giao tiếp | GPIO | Cấu hình |
|---|---|---|
| CAN/TWAI TX/RX | 4 / 5 | 1 Mbit/s |
| Motor UART1 TX/RX | 17 / 18 | 1 Mbaud, 8N1 |
| Motor UART2 TX/RX | 15 / 16 | 1 Mbaud, 8N1 |
| BLE/debug UART0 TX/RX | 43 / 44 | CP2102 |

Nguồn khai báo: [`BoardConfig.hpp`](include/tap_fw/BoardConfig.hpp) và sơ đồ
PCB trong `docs/data/POC - Robot version 1/`.

## Estimator IMU

`ImuStateEstimator` trong core đã có:

- Seed pitch bằng cửa sổ gia tốc ban đầu.
- Ước lượng gyro bias khi robot đứng yên.
- Tích phân gyro cho đáp ứng nhanh.
- Hiệu chỉnh pitch bằng accelerometer.
- Lọc vận tốc encoder trước khi vi phân gia tốc.
- Bù gia tốc động tại vị trí IMU từ encoder/FK.
- Trọng số tin cậy liên tục thay cho cổng bật/tắt cứng.
- Gate mẫu gia tốc sai và latch va chạm.

Đây là thuật toán đã build/test trên host. Chưa có bằng chứng đọc đúng dữ liệu
MPU6500 thật vì driver SPI và hiệu chuẩn trục chưa được triển khai.

## Mapping Dabble GamePad

| Nút | Chức năng |
|---|---|
| Up | Tiến `+0.25 m/s` |
| Down | Lùi `-0.25 m/s` |
| Left/Right | Quay trái/phải |
| Start | `ENABLE` |
| Select | `DISABLE` |
| Triangle | Max stand, `HEIGHT 0.245` |
| Circle | Deep squat, `HEIGHT 0.140` |
| Cross | Natural, `HEIGHT 0.148` |
| Square | `ESTOP` |

Khi nhấn đổi pose, mapper gửi thêm `STOP`. Các lệnh `RESET`,
`CLEAR_ESTOP`, `FIXTURE_RELEASE` vẫn tồn tại ở command endpoint nhưng không
chiếm mapping GamePad chính.

## Bằng chứng hiện tại

Đã đạt:

```text
cmake --build build -j2
core_tests                         PASS
controller_actuator_bridge_tests  PASS
ble_command_endpoint_tests        PASS
git diff --check                   PASS
```

Đây là bằng chứng build và unit/contract test trên host. Chưa phải bằng chứng
ESP32 boot, SPI, CAN, UART, BLE Dabble hoặc động cơ thật hoạt động.

## Việc cần làm tiếp theo

1. Viết `Mpu6500Spi` tối thiểu: reset, `WHO_AM_I`, cấu hình gyro/accel, đọc
   burst, timestamp và kiểm tra lỗi bus.
2. Viết adapter sáu encoder theo đúng frame/đường dây của PCB.
3. Gắn TWAI và hai UART ESP-IDF vào `BusTransport`.
4. Tạo `app_main` với thứ tự task: sensor -> controller -> actuator; BLE chỉ
   ghi command mailbox.
5. Thêm calibration runtime cho zero encoder, scale MPU6500 và hướng trục.
6. Chạy bring-up không tải: đọc sensor, kiểm tra frame, rồi mới cho phép
   `ENABLE`.

Không được xem mô phỏng host hoặc test frame là xác nhận an toàn cho motor
thật. Việc cấp nguồn, flash và chạy động cơ cần một quy trình bring-up riêng.
