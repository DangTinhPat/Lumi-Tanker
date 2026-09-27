# Đặc tả FW v2 theo ý tưởng Impulse

## Mục tiêu

Tạo một phiên bản firmware TAP riêng, lấy cảm hứng từ `Impulse.zip` ở cách
chia giai đoạn khởi động và đưa chân tới mục tiêu bằng quỹ đạo mượt. Giữ nguyên
phần cứng và quy ước TAP; không chép nguyên firmware Teensy/ODrive của Impulse.
FW hiện tại tiếp tục được bảo toàn làm bản tham chiếu.

## Ý tưởng lấy từ Impulse

Trong `Robot_Assembly_Code/Functions.ino`, `coldStart()` thực hiện lần lượt:

1. Chuyển driver sang closed-loop.
2. Đưa chân về tư thế khởi động bằng quỹ đạo trong không gian X-Z.
3. Lấy pitch IMU lúc đó làm `pitchSet`.
4. Ổn định với vòng vận tốc bánh chưa bật.
5. Khởi tạo vòng giữ vị trí/tốc độ bánh và đi tới độ cao vận hành.

Đây là một pitch reference được mồi lúc startup, không phải thuật toán tự học
góc cân bằng theo thời gian. Impulse không có bước homing; mã giả định encoder
đã có mốc và dùng encoder estimate trực tiếp trong FK/IK.

## Phần cứng đích

- ESP32-S3 và BLE Dabble giữ nguyên.
- Bốn khớp chân GDS68 trên CAN 500 kbit/s.
- Hai động cơ bánh BL4820 qua UART 1 Mbaud.
- MPU6500 qua SPI, dùng trục IMU đã ánh xạ sang body frame TAP.
- Giữ mapping/sign theo CAD TAP; không chép dấu hai bên của Impulse.

## Các khối phần mềm

| Khối | Trách nhiệm |
|---|---|
| `FirmwareApp` | Vòng lặp phần cứng không chặn: lấy IMU, BLE, CAN/UART feedback; gọi state machine và phát lệnh. |
| `MotorFeedback` | Freshness, heartbeat, encoder, dấu và offset zero; không dùng `0x009` làm góc tuyệt đối nếu chưa hiệu chuẩn. |
| `WholeBodyModel` / planner hiện có | Hình học và quỹ đạo khớp theo CAD TAP; không mang FK/IK linkage của Impulse sang. |
| `ImpulseStartupSequencer` | Quản lý các pha cold-start, timeout, điều kiện chuyển pha và hủy an toàn. |
| `BalanceController` | Dùng bộ cân bằng TAP hiện có; seed góc cân bằng từ pitch khi thả robot khỏi trạng thái giữ. |
| `BleCommandEndpoint` | Một lệnh Start khởi động chuỗi; D-pad chỉ có hiệu lực khi sequence vào trạng thái vận hành. STOP/DISABLE luôn có ưu tiên. |

## Trình tự startup

1. **BOOT_SAFE**: gửi disable/idle; xác nhận sensor mới, heartbeat và feedback
   bốn khớp + hai bánh.
2. **NEEDS_ZERO**: nếu driver vừa khởi động và không có offset hợp lệ, người
   vận hành nâng thân, duỗi hai chân thẳng xuống, đưa gối tới chặn duỗi và giữ
   thân cùng bốn khớp yên liên tục 0.5 giây. Firmware chụp offset khi bấm Start.
   Góc hông trong pose capture
   được tính theo gravity pose của model; đây là giả thiết phải đo xác nhận,
   không phải cảm biến absolute.
3. **LEG_PREPOSITION**: `FixtureHold` giữ nguyên pose duỗi đã chụp; không gập
   chân trước khi controller cân bằng.
4. **RELEASE**: cho phép release sau preposition. Controller TAP seed balance
   trim từ pitch tại cạnh release, tương tự cách Impulse lấy `pitchSet = pitch`.
5. **BALANCE_WARMUP**: sau khi state machine vào Balance, giữ lệnh drive bằng
   không trong một giây và chặn lệnh điều hướng gamepad. Log `BALANCE MODE active`
   chỉ xác nhận state machine đang ở Balance; firmware không đo tiếp xúc bánh và
   không chứng minh thân robot tự đứng ổn định. Người vận hành chỉ tháo vật đỡ
   sau khi quan sát robot tự chịu tải ổn định.
6. **RUN**: cho phép BLE drive/pose. Lệnh STOP, timeout BLE, IMU stale, lỗi axis
   hoặc feedback stale đưa hệ thống về disable/fault theo safety policy TAP.

Không dùng công tắc tiếp xúc chân hoặc telemetry chỉ có trong mô phỏng để quyết
định pha startup; inventory phần cứng TAP không có các sensor đó.

## Triển khai trong source

- Build profile mới: `esp32-s3-impulse-v2`; default `esp32-s3-poc` được giữ.
- Profile `esp32-s3-impulse-v2-full` là build opt-in đặt hai khóa Enable thành
  `true`. Start có thể đưa motor vào closed-loop; build thành công không chứng
  minh mapping tư thế chân hay cân bằng vật lý.
- Một lần Start yêu cầu enable và release. Firmware chặn release đến khi
  `FixtureHold` giữ ổn định pose duỗi; sau đó chạy ramp cân bằng, giữ drive bằng
  không trong một giây, log `BALANCE MODE reached`, rồi mới cho BLE điều hướng.
- Profile v2 chọn homing `ManualExtended`: người vận hành nâng thân, giữ hai
  chân thẳng xuống và gối ở chặn duỗi trước Start. Firmware chụp encoder làm
  offset cho extended pose rồi giữ nguyên pose đó trong suốt startup; đây không
  phải góc encoder tuyệt đối. V2 không tự tìm góc đùi từ góc bất kỳ.
- Enable của target v2 hiện bị khóa bởi
  `kManualExtendedPoseMappingVerified=false`: góc cơ khí knee-stop chưa được
  ánh xạ sang CAD và pose hông model chưa được xác nhận trên robot; đồng thời
  `kImpulseExtendedLegBalanceVerified=false` khóa cân bằng khi chân duỗi đến khi
  profile này được xác nhận trên TAP.
- Timeout khi preposition/release/balance ramp kết thúc bằng software disable.
- `FirmwareApp` dùng feedback/CAN/UART/IMU/BLE TAP; lớp
  `ImpulseStartupSequencer` giữ riêng logic các pha khỏi phần cứng.

## Khác biệt không được chép từ Impulse

- Không chép Teensy 4.1, BNO086, CAN 1 Mbit/s, ODrive library, gear ratio 8,
  linkage `b=140 mm`, signs trái/phải hoặc các gain PID của Impulse.
- Không chép các vị trí chân 87/150/240 mm; phải tính từ CAD TAP, tư thế natural
  hiện hành và giới hạn cơ khí đã hiệu chuẩn.
- Không dùng vòng pitch PID điều chỉnh độc lập độ cao hai chân thay cho LQR
  cân bằng bánh của TAP.
- Không dùng các vòng `while` blocking của `coldStart()`/`setState()`; sequence
  TAP chạy từng bước trong control loop, luôn xử lý BLE, sensor watchdog và STOP.
- Không xem `Pos_Estimate` reset về gần zero khi driver bật nguồn là góc khớp
  tuyệt đối.

## Điều kiện kiểm chứng tiếp theo

- Đo/hiệu chuẩn góc gối tại chặn duỗi và mapping encoder 30..145 độ sang hệ CAD
  TAP; dải native hiện tại chưa phải hành trình cơ khí đã xác nhận.
- Xác nhận pose hông khi chân thẳng xuống theo IMU/body pitch và so sánh với
  gravity pose tính từ model TAP.
- Chốt các tư thế preposition/stance và giới hạn tốc độ, gia tốc theo CAD TAP.
- Mô phỏng chuỗi state machine trên host; sau đó review code và build firmware.
  Build không chứng minh được khả năng vật lý. Không flash hoặc phát lệnh motor
  thật nếu chưa có yêu cầu riêng.

## Nguồn tham khảo

- `/home/dvt/Downloads/Impulse.zip`,
  `Impulse/Robot_Assembly_Code/Functions.ino`: `coldStart()`, FK/IK, ramp,
  đọc IMU và controller.
- `src_fw/FIRMWARE_INTEGRATION_STATUS.md` và `src_fw/include/tap_fw/BoardConfig.hpp`:
  hardware/mapping TAP và trạng thái bring-up hiện tại.
