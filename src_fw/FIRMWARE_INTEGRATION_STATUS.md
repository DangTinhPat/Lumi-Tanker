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

## Controller lên robot: Start → cân bằng (2026-09-26)

Thay đổi trong firmware controller:

- **Cấu hình đã tune**: `FirmwareControlLoop` dùng
  `tap_fw::firmwareControllerConfig()` = `robot_control::tunedControllerConfig()`
  cộng các thay đổi phần cứng ở mục "Góc cân bằng tự động" bên dưới.
  `tunedControllerConfig()` (core `TunedConfig.cpp`) là bản sao
  nguyên văn `configureController()` của `native_runner.cpp` (không sửa code sim
  vì máy không build được MuJoCo; hai bản phải đổi cùng nhau). Trước đây
  firmware chạy `Config{}` mặc định (mô-men chân 1.0 N·m, hông [-0.2, 1.0]).
  `tuned_config_tests` cố định các giá trị chính và chạy chuỗi fixture → Balance.
- **Mốc góc chân (homing)**: GDS68 báo vị trí tương đối từ lúc driver bật nguồn.
  Lần bấm Start đầu tiên khi robot nằm yên trên giá (tư thế neutral: hông
  1.0105, gối −1.0683 rad) chụp offset: `core = sign·raw + offset`; bridge đổi
  ngược `driver = sign·(core − offset)`. Từ chối nếu thiếu feedback MIT, chân
  đang chuyển động (> 0.1 rad/s) hoặc IMU chưa yên. Mất feedback chân > 300 ms
  (driver khởi động lại) hoặc ESP khởi động lại → mất mốc, phải gá lại.
  `0x009` không còn là nguồn góc khớp.
- **Giám sát driver**: đọc heartbeat `0x001` của 4 GDS68. ENABLE bị từ chối
  nếu mất heartbeat hoặc có `axis_error`; sau khi enable 0.5 s mà một khớp
  chưa ở CLOSED_LOOP (hoặc báo lỗi) thì DISABLE và log tên khớp — phản hồi MIT
  vẫn về từ driver đã từ chối closed loop nên không thể dựa vào nó. Lấy mốc
  dùng gyro + tốc độ khớp để kiểm tra đứng yên (không dùng |a| − g vì offset
  accel). LED2 sáng khi đã có mốc.
- **Tải CAN 500 kbit/s**: lệnh MIT chân 250 Hz (controller vẫn 500 Hz), bánh
  500 Hz; khi disable lặp IDLE/Torque Disable 50 Hz (gửi ngay ở cạnh disable).
- **Giới hạn mô-men chân trong bridge**: kẹp T_ff và sai số vị trí sao cho
  |kp·e + kd·(v − dq) + T_ff| ≤ `torque_nm` (3 N·m), giống `SimGds68Joint`.
- **BLE lease**: D-pad chỉ có hiệu lực khi điện thoại vừa gửi gói trong
  `kBleMotionLeaseUs` (tạm 500 ms, chờ đo); log `BLE rx packets/gap_max`.
- **Chẩn đoán**: SENS thêm `homed`, pitch rate, yaw rate, accel và thời gian
  chu kỳ điều khiển `cycle_us max/avg` (Rule 4). CANDIAG thưa hơn khi đang chạy.
- **`kAutoFixtureRelease = true`** (luồng người vận hành chọn): gá robot ở tư
  thế natural → Start → lấy mốc + cân bằng → tháo giá kê. Đặt `false` nếu chỉ
  muốn kiểm tra giữ chân ở FixtureHold.

Khóa enable còn giữ: `kWheelSpeedScaleVerified`, `kImuFrameVerified`, và mốc
chân (runtime).

Kiểm tra trên robot (robot trên giá, có người giữ, mỗi bước mở khóa bước sau):

1. **C1 Dabble**: kết nối BLE, giữ một nút D-pad, đọc `BLE rx ... gap_max`
   → chọn `kBleMotionLeaseUs`.
2. **C2 Tốc độ bánh**: firmware jog + GUI, chế độ quay liên tục 60 và 120
   raw, so cột "RPM đo" với lệnh raw → đặt `kWheelSpeedScaleVerified=true`
   hoặc sửa hệ số trong `MotorFrameCodec.hpp`.
3. **C3 Chiều IMU**: firmware chính, disable. Nghiêng mũi về phía robot chạy
   tới (chiều bánh dương) → `pitch` dương, `pitch_rate` dương khi đang nghiêng;
   xoay ngược chiều kim đồng hồ nhìn từ trên → `yaw_rate` dương → đặt
   `kImuFrameVerified=true` hoặc thêm phép đổi trục trong `Mpu6500Spi`.
4. **C4 Mốc góc**: Start trên giá → log `LEG ZERO homed`, `joint_q` ≈ neutral,
   hai bên đối xứng; lắc robot khi bấm Start → phải bị từ chối.
5. **C5 Giữ chân**: `kAutoFixtureRelease=false` → FixtureHold, chân giữ không
   giật, `cycle_us` trong ngân sách 2000 µs.
6. **C6 Cân bằng**: `kAutoFixtureRelease=true`, Start → cân bằng; ESTOP (nút
   vuông) sẵn sàng, tốc độ chạy thấp.

Test lõi có sẵn đang fail cả ở HEAD trước thay đổi này (build `src/tap_control/core`
độc lập): `core_allocation_tests`, `jump_tests`, `jump_quality_tests`,
`lqr_tests` ("station return reference governor failed").

## Góc cân bằng tự động (2026-09-26)

Log run7/run8: robot chạy tới rồi chúi đầu. Nguyên nhân: controller giữ góc
pitch tham chiếu cố định 0 (theo CAD), trong khi điểm cân bằng thật lệch vài độ
(IMU gắn nghiêng, khối lượng CAD ước lượng, sai mốc khớp do homing trên giá).
Lúc khởi động chưa có vòng vị trí, nên lệch θ chỉ có một trạng thái dừng:
nghiêng θ và lăn đều → trôi tới giới hạn tốc độ. Lệnh vận tốc còn bị "wind-up"
khi bánh UART không theo kịp.

Thuật toán (giống vòng tốc độ ngoài của Impulse, `LqrBalancer`, mặc định TẮT
nên sim không đổi):

- `balance_trim_rad`: góc cân bằng học được, cộng vào pitch tham chiếu.
  Khi robot trôi tới (v_đo > v_đích) thì giảm trim (ngả về sau), tích phân
  `trim -= k·(v_đo − v_đích)·dt`, kẹp ±`balance_trim_max_rad`, chỉ học khi
  |pitch − ref| < 0.25 rad và bánh chạm đất.
- Mồi (seed) trim = pitch đo lúc thả khỏi giá (`seed_balance_trim_from_pitch`).
- Anti-windup: lệnh vận tốc LQR kẹp trong ±margin quanh vận tốc bánh đo.
- Trim giữ qua `reset()` (thuộc tính của robot), `TRIM RESET` để xóa.

Profile firmware (`FirmwareProfile.cpp`): k = 0.30 rad/m, max 0.20 rad,
margin 0.25 m/s, gia tốc giữ 1.4 m/s² (0.8 không kịp đỡ trên mô hình rút gọn),
tắt stance COM task (chân giữ tư thế natural + bù trọng lực, như Impulse).
`balance_trim_tests` (mô hình con lắc bánh xe rút gọn, servo bánh τ 40 ms):
tắt trim → trôi; mồi + k 0.3 → hội tụ với điểm cân bằng ±5°/±8° và sai tay
±2° lúc thả; bánh chỉ theo 30 % → lệnh không vượt đo + margin.

Quy trình trên robot (`kAutoFixtureRelease = false`):

1. Gá robot tư thế natural, cấp nguồn, chạy console:
   `python3 tools/controller_console.py --log runN.log --fifo /tmp/tap_console`
   (mở cổng sẽ reset ESP — mở trước khi bấm Start).
2. Kết nối BLE, **Start lần 1**: lấy mốc khớp, chân giữ cứng (FixtureHold).
3. Nhấc robot khỏi giá, dựng bánh chạm đất ở điểm "nhẹ tay" (cân bằng).
4. **Start lần 2**: thả vào cân bằng; log in `RELEASE pitch=… seeded_trim=…`
   và `TR` 10 s ở 25 Hz (thêm `vcmd`, `pref`, `trim`).
5. Không giữ thân robot; trim hội tụ về hằng số. Select = dừng.

Chỉnh trực tiếp qua console (không cần nạp lại): `GET`,
`SET trim_gain|trim_max|trim|margin|accel <giá trị>`, `TRIM RESET`.
Điều kiện tiên quyết: hai bánh phải theo được lệnh khi có tải (log trước cho
thấy bánh UART1 kẹt 34–46 % thời gian) — phần mềm không bù được bánh không quay.

## Hướng IMU trên PCB (2026-09-26)

Người vận hành cập nhật PCB đã hàn MPU6500 theo hướng làm hoán đổi vai trò
Pitch/Roll: trục roll dương của chip tương ứng robot chúi mũi xuống. Cấu hình
hiện ánh xạ `body X = -chip Y`, `body Y = +chip X`, `body Z = chip Z` qua
`BoardConfig::kImuAxisMap` (`kChipXLeftYBack`) trước estimator. Vì vậy pitch
và pitch-rate của controller lấy từ trục roll/+X của chip với dấu dương là
chúi mũi. Ma trận vẫn là phép quay thuận tay phải (định thức +1). SENS in cả
`acc` (thân) và `chip_acc` (chip); khi kiểm tra tĩnh, chúc đầu robot xuống thì
pitch phải dương và `chip_acc` Y phải dương theo mapping hiện tại.
