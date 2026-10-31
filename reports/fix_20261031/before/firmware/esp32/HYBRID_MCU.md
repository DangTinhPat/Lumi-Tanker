# Tầng dưới Hybrid Control Runtime — ESP32-S3 N16R8

Bản build `esp32-s3-hybrid-mcu` chạy IMU, ước lượng vector trọng lực, HCR1 qua
UART0, console USB native/Wi-Fi và backend BL4012e thật: `BlMotorBackend` cùng
hai worker bus UART2/GPIO15 (trái), UART1/GPIO17 (phải). Đã build/test phần mềm;
chưa flash hoặc vận hành robot. Manual home và feedback hợp lệ là điều kiện ARM.
Chi tiết ở [MOTOR_INTEGRATION.md](MOTOR_INTEGRATION.md).

Logic đứng dậy, model và quyết định chuyển pha nằm trên LF. Bản build này không
khởi động `AppV3`, CAN motor hoặc gateway UDP LFG1 của bản nháp.
Bản nháp vẫn có ở target `esp32-s3-luckfox-gateway`.

**HCR1 nhận từ LF 4 góc hip/crank (rad) + 2 vận tốc bánh (rad/s), không có torque
command.** Recovery còn có 4 vận tốc feedforward hip/knee hinge. Torque feedback
về LF là tùy chọn. Luồng trước tích hợp được đối chiếu theo mã
trong [HYBRID_FLOW_AUDIT.md](HYBRID_FLOW_AUDIT.md).

## 1. UART và console

| ESP32-S3 | Luckfox |
|---|---|
| TX0 / GPIO43 | UART2 RX / GPIO1_B3 |
| RX0 / GPIO44 | UART2 TX / GPIO1_B2 |
| GND | GND |

UART hai chiều, **921600 baud, 8N1**, không flow control. UART0 chỉ mang frame
nhị phân HCR1. `Serial` được chuyển sang USB native CDC (GPIO19/20). Boot ROM có
thể phát log trên TX0 khi reset; parser HCR1 bỏ byte ngoài frame và bắt tay lại.

Theo cấu hình PCB trong repository, GPIO43/44 còn nối CP2102. Khi dùng UART0
cho LF phải **tách CP2102 TX khỏi GPIO44** (jumper/đường mạch theo board thực tế).
Rút cáp USB chưa chứng minh chân CP2102 đã high impedance. Hai nguồn TX không
được cùng lái RX0. Việc nạp qua CP2102 cần tách LF khỏi RX0 trong lúc nạp.

Console:

- Wi-Fi TCP port **8023**, hostname `lf-mcu.local`; discovery UDP port **8024**.
- USB **native** GPIO19/20 nếu board có đưa ra cổng. Cổng USB nối CP2102 không
  còn là console của bản build này.
- Wi-Fi dùng `include/tap_hw/WifiSecrets.hpp` (git-ignored), như bản nháp;
  hỗ trợ primary/ALT/ALT2, ALT2 ưu tiên cao nhất khi thấy lúc boot. File cấu hình
  local được sao chép từ workspace Lumi-Tanker; không đưa mật khẩu vào tài liệu.
  Nếu clone mới, chép `WifiSecrets.example.hpp` thành `WifiSecrets.hpp` và cấu hình.

| Lệnh console | Ý nghĩa |
|---|---|
| `STATUS` / `GET` | Link/epoch, validity, mode/fault LF, local fault, bộ đếm và gravity |
| `HELP` | Danh sách lệnh |
| `CAL_IMU` | Hiệu chuẩn lại khi disarmed; giữ IMU yên ít nhất 0,5 s |
| `MOTOR_HOME` | Chụp zero ở tư thế home model khi OFF và đứng yên; không làm motor di chuyển |
| `MOTOR_ZERO hipL hipR kneeL kneeR` | Chụp zero tại tư thế model đã đo, đơn vị rad |
| `ESTOP` | Latch E-stop cục bộ; LF nhận cờ E-stop trong telemetry |
| `CLEAR_FAULT` | Xóa lỗi cục bộ chỉ khi motor đã xác nhận OFF; LF cần RESET riêng |

ARM/DISARM/RESET/VELOCITY là lệnh người vận hành gửi trực tiếp tới LF qua
`tools/hybrid_command.py`; chúng không phải frame UART. ARM trên LF chỉ bắt đầu
pha HOME khi telemetry và home hợp lệ. ESP vẫn lặp lại kiểm tra driver, cảm biến,
tư thế home trước khi nhận TARGET bật đầu tiên.

Lệnh vận tốc hết hạn 500 ms tự gửi `(0,0)`. ARM bị từ chối khi chưa có động cơ;
`cmd=1 result=2` trong STATUS nghĩa là yêu cầu ARM đã bị từ chối. `result=1`
chỉ là MCU chấp nhận/gửi yêu cầu; xem `lf_mode` để biết LF đã chấp nhận chưa.

## 2. Tổ chức firmware

- `HybridGatewayApp.cpp`: UART0, parser, queue giới hạn, console USB/Wi-Fi,
  task service được đánh thức mỗi 500 µs. UART và console không chạy trong task
  này; deadline motor được kiểm tra dù đường UART hoặc Wi-Fi bị nghẽn.
- `HybridGatewayCore.cpp`: bắt tay, epoch/sequence/timestamp, telemetry 200 Hz,
  TARGET/STATUS, latch fault, hạn TARGET **20 ms**. Chỉ nhận target tham
  chiếu chính xác telemetry đã gửi và còn mới ≤30 ms. Gói trùng/CRC sai không
  kéo dài hạn. ACK cập nhật khi backend trả `Applied` cho cả hai bus hoặc
  `disabled()` xác nhận mọi OFF; Pending không ACK.
  Mất phản hồi LF 500 ms kể cả khi disarmed sẽ yêu cầu phiên mới; reconnect
  không tự ARM.
- `HybridAttitude.hpp`: gyro bias ba trục, gravity `[gx,gy,gz]` có cả roll/pitch;
  đứng thẳng `[0,0,-1]`. Theo cách lắp hybrid người dùng xác nhận: chip +X hướng
  trước robot, +Y hướng phải, mặt linh kiện úp xuống (+Z hướng xuống). Firmware
  đổi sang body bằng `[chipX,-chipY,-chipZ]`, đơn vị rad/s và m/s². Quy ước
  này cần kiểm chứng bằng số đo tĩnh/nghiêng trên đúng board trước khi ARM.
- `HybridMotorBackend.hpp`: giao diện submit/poll bất đồng bộ.
- `BlMotor.cpp`: codec motor, hai worker, mapping/zero/unwrap, TTL và batch ACK.
- `EspBlMotor.cpp`: UART một dây và task worker; không chạy I/O trong service.
  `UnavailableMotors` còn dùng cho test thiếu driver, không phải backend firmware.
- `hcr/hcr_protocol.h`: codec lấy nguyên bản từ LF runtime. Wrapper MCU kiểm
  tra history/ARM/driver chặt hơn `mcu/hcr_link.h` tham khảo của LF.

`tick_max_us` và `missed` là số đo trên ESP32 khi chạy. Lịch task 2 kHz không
chứng minh bus đạt servo 2 kHz; I/O chạy ở worker có watchdog riêng.
Watchdog nội tại driver khi ESP mất nguồn chưa được xác minh. Lịch telemetry không gửi bù dồn sau khi trễ.

Local fault: `0=None`, `1=TargetTimeout`, `2=Sensors`, `3=Driver`, `4=Estop`,
`5=Transport`, `6=RemoteFault`. DISARM không xóa fault. STATUS từ LF không gia
hạn motor. Cờ driver fault không được dùng để biến RemoteFault thành lỗi driver.

## 3. Backend động cơ đã tích hợp

`EspBlMotor` khởi tạo hai bus và hai worker; `GatewayCore` dùng `BlMotorBackend`.
`apply(target,seq,deadline,now,applied)` trả Pending/Applied/Fault. Deadline lấy
nguyên từ thời điểm nhận gói, không gia hạn bởi gọi lại mỗi tick. Completed batch
có epoch/sequence riêng và reference sau quantization/lọc. `disable()` hủy việc
đang chờ; `disabled()` mới xác nhận OFF. `ready()` cần configured + homed; core
kiểm thêm tuổi sensor, E-stop, pose home và phiên LF.

Motor nhận hip/crank target tuyệt đối; feedback knee hinge=`phi-hip`. Crank
velocity FF=hip FF+knee FF. Wheel rad/s đổi sang RPM có sign. Gain mặc định
10/.7 theo GUI và tách theo HOME/RECOVERY/POLICY; nếu vượt gain max driver thì
fault, không tự kẹp. Chưa có torque feedback thật nên TORQUE_VALID luôn clear.

Home LF là `[2.35706715,2.35706715,0,0]` rad (hipL/R,kneeL/R); cả MCU và LF
kiểm tra ±.15 rad khi ARM. `MOTOR_HOME` chỉ capture zero từ pose này;
`MOTOR_ZERO` cho pose model đo khác. Zero giữ trong RAM, mất sau reboot hoặc
lỗi motor. RESET thông thường sau calibration giữ zero; reset lỗi motor bắt
buộc OFF/configure/capture lại. Luồng chi tiết, gains/limits, khác biệt với
servo mô phỏng và thứ tự bring-up: [MOTOR_INTEGRATION.md](MOTOR_INTEGRATION.md).

## 4. Kiểm tra khi chưa có động cơ

Vẫn kiểm tra được: bắt tay, telemetry IMU/gravity, CRC, sequence, nhận TARGET
DISABLED và STATUS, DISARM/RESET/E-stop. Backend mới thật sự dò bus; thiếu
motor sẽ báo lỗi bus/driver và không khai encoder/calibrated/applied hợp lệ.
Trong host test `UnavailableMotors`, chỉ có IMU_OK; đó là test double riêng.

**Runtime LF hiện hành kiểm tra encoder/calibration ngay cả khi DISARMED**.
Vì vậy thiếu calibration làm LF báo `FAULT=2 invalid_telemetry`, còn lỗi bus
làm LF báo driver fault; đây không
phải bằng chứng UART hỏng. RESET không khắc phục việc thiếu encoder. Không bật
cờ hợp lệ giả để ép LF chạy model. Khi driver/encoder sẵn sàng, cần RESET lại.

Để kiểm tra đường truyền riêng, có peer chỉ gửi DISABLED, không cần model:

```sh
python3 firmware/esp32/tools/hcr_uart_peer.py --port /dev/ttyS2 --seconds 30
```

Chép cả `hcr_uart_peer.py` và `hcr_protocol.py` vào cùng thư mục trên LF nếu
chạy tại LF. Chỉ chạy khi UART không có runtime/dashboard khác sở hữu; công cụ
từ chối cổng đang mở. In flags, IMU, số frame và tốc độ telemetry nhận được.

Để chạy runtime LF thật, entrypoint hỗ trợ `runtime.py --serial /dev/ttyS2`.
Snapshot config local đang chọn Unix socket; phải cấu hình serial lúc tích hợp
phần cứng. Không chạy peer và runtime đồng thời trên cùng UART.

Từ máy phát triển, chạy tool kiểm tra cả UART và console ESP:

```sh
python3 firmware/esp32/tools/check_hcr_uart.py \
  --lf-ip 10.10.60.232 --esp-ip 10.10.61.85 --seconds 15
```

Tool SSH vào LF, chép peer vào `/tmp`, khóa độc quyền `/dev/ttyS2`, rồi xóa
peer sau khi chạy. Nó bắt tay HCR1, đếm HELLO_ACK/telemetry/CRC, chỉ gửi
TARGET DISABLED, và đọc STATUS ESP qua TCP 8023. SSH password được hỏi không
hiện trên màn hình. PASS cần có HELLO_ACK, telemetry và zero CRC error. Dòng
`IO rx_bytes` được thu nếu firmware ESP đã có chẩn đoán UART; bản cũ có thể
chưa xuất dòng này. Tool từ chối khi một runtime khác đang giữ UART.

## 5. Build và kiểm tra trên máy tính

```sh
pio run -e esp32-s3-hybrid-mcu
python3 firmware/esp32/tests/check_hybrid_gateway.py \
  --runtime '/path/to/contoller hybrid/deploy/hybrid_runtime'
python3 firmware/esp32/tests/check_bl_motor.py \
  --runtime '/path/to/contoller hybrid/deploy/hybrid_runtime'
```

Nếu `pio` chưa nằm trong PATH, dùng `~/.platformio/penv/bin/pio`.
Binary: `.pio/build/esp32-s3-hybrid-mcu/firmware.bin`. Target hybrid là mặc định
của worktree này; các target V3 cũ vẫn có thể chọn rõ tên. Upload/monitor port
cần chỉ định theo cáp thực tế.

Host test dùng ASan/UBSan: parser/CRC, replay/wrap, source history, TTL, ACK,
E-stop/reset, thiếu driver, NaN, mapping và dấu gravity. Integration sử dụng
**lõi runtime LF thật**, fake policy/logic và fake motor chỉ trong executable
test, đi qua 2.700 chu kỳ / 13,5 s thời gian giả lập. Bài `check_bl_motor.py`
thay fake backend bằng backend/codec motor được build vào firmware và chỉ giả
lập hai driver bus, cũng đi qua 2.700 chu kỳ. Đây là kiểm tra hợp đồng
phần mềm, không phải mô phỏng cơ học, inference RKNN hay kiểm chứng robot.
Công cụ UART peer cũng được chạy qua pseudoterminal trên máy tính: nhận telemetry
và chỉ trả TARGET DISABLED, không dùng cổng serial phần cứng.

Chưa nạp board hoặc đo UART/IMU trên phần cứng trong lượt triển khai này.
