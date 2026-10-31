# Luồng động cơ HCR–BL4012e — 2026-10-30

Backend motor đã được tích hợp vào target `esp32-s3-hybrid-mcu`. App khởi tạo
`EspBlMotor`, hai worker bus và `BlMotorBackend`; không còn dùng
`UnavailableMotors` trong bản firmware này. Bản tích hợp đã được flash và bắt đầu kiểm tra kết nối, chưa kiểm chứng đáp ứng
chuyển động. Bản sửa gain và chẩn đoán bên dưới đã build/test, đang chờ nạp lại.

## Ánh xạ đã chốt

Người dùng đính chính: **UART2 là chân trái, UART1 là chân phải**.
ID/chiều lấy từ `BL4012e/3in1/src/main.c`, `gui.py` và file cài đặt GUI
`~/.config/3in1_gui.json`: cả sáu `invert=false`. GUI tự gửi Kp=10, Kd=.7,
nhưng bench cũ clamp theo giới hạn driver. Đọc phần cứng ngày 2026-10-30:
hip ID2 cả hai bus báo Kp_max=20, Kd_max=.120. Backend hiện đặt rõ
Kp=10, Kd=.120 cho cả ba profile, vẫn kiểm giới hạn từng driver; giới hạn
crank ID3 còn phải xác minh sau khi nạp bản sửa.

| Kênh HCR | Motor | Bus DATA | ID | Dấu driver / tọa độ dùng trong backend |
|---|---|---|---:|---:|
| target[0], hip trái | hip | UART2 / GPIO15 | 2 | +1 |
| target[1], hip phải | hip | UART1 / GPIO17 | 2 | -1 |
| target[2], crank trái | crank/gối | UART2 / GPIO15 | 3 | -1 |
| target[3], crank phải | crank/gối | UART1 / GPIO17 | 3 | +1 |
| target[4], bánh trái | wheel | UART2 / GPIO15 | 1 | +1 |
| target[5], bánh phải | wheel | UART1 / GPIO17 | 1 | -1 |

GPIO16/18 là pad RX nối cùng DATA trong PCB, được đặt input-floating. Hai bus
motor 1 dây 1 Mbaud/8N1; UART0 TX43/RX44 vẫn là HCR1 921600/8N1 tới LF.
MPU6500 dùng GPIO10–14, không trùng chân motor.

Dấu ở bảng kế thừa chiều gập/duỗi và tiến đã được người dùng kiểm trên motor.
Đợt này chưa đo lại quan hệ dấu/mốc đó với tư thế MJCF; `MOTOR_HOME` xác lập
zero bằng một tư thế model đã biết, không tự coi tư thế bật nguồn là home.
Encoder ID3 được xử lý là góc crank của cơ cấu trong gói hybrid.

## Luồng chạy

```text
LF TARGET [hipL hipR phiL phiR omegaL omegaR], profile/enables/FF
  -> GatewayCore: CRC, epoch, source history, TTL 20 ms, ARM/sensors
  -> BlMotorBackend: ánh xạ, kiểm range, tạo batch cùng sequence cho hai bus
  -> worker trái UART2 và worker phải UART1
  -> MIT absolute position/velocity (ID2,ID3), wheel speed RPM (ID1)
  -> reply hợp lệ từ cả hai bus -> completed batch -> applied_seq/applied_target
  -> TELEMETRY về LF

Encoder joint trong reply; wheel READ 0x24
  -> timestamp riêng từng motor -> đổi sign/zero, unwrap bánh
  -> q=[hipL hipR kneeL kneeR wheelL wheelR], qd tương ứng
```

Công thức đối xứng:

```text
model_angle = sign * raw_counts * (2*pi/16384) + zero
raw_target  = round(sign * (model_target-zero) * 16384/(2*pi))
wheel_rpm   = round(sign * omega_rad_s * 60/(2*pi))
q_knee      = phi_measured - hip_measured
qd_knee     = phi_dot_measured - hip_dot_measured
crank FF    = hip FF + knee hinge FF
```

Khớp nhận MIT block Kp/Kd/position/RPM/tau, trong đó **tau_ff luôn bằng 0**.
Không gọi `leg_move()` (delta/trajectory) của bench. LF vẫn gửi 4 góc và 2 tốc
độ, không gửi torque. `Torque Enable 0x18` dùng bật/tắt driver.

`apply()` trả `Pending`, `Applied` hoặc `Fault`; bản ghi Applied chứa epoch,
sequence và reference của batch đã hoàn tất, có thể là batch trước lệnh mới
nhất. Chỉ một batch đang chờ được giữ cho cả hai bus; lệnh mới nhất sẽ được
chọn cho batch tiếp theo. Không ACK khi enqueue hoặc khi chỉ một bus xong.
Không cần đợi trục vật lý tới đích. `applied_target` phản ánh counts/RPM đã làm
tròn và wheel reference sau lọc, không sao chép mù raw target từ LF.

Tắt cũng bất đồng bộ: `disable()` hủy generation và yêu cầu OFF; `disabled()`
chỉ true sau reply OFF của cả sáu motor. Kênh bị bỏ enable không được ghi lệnh
motion; wheel applied=0, các góc giữ reference cuối đã chốt (hoặc góc đo nếu
chưa có reference). Không diễn giải zero của DISABLED thành góc đích 0.

## Khởi động bánh và watchdog

BL4012e bench ghi nhận phải chờ >=20 ms sau bật lực trước ghi tốc độ mới;
backend giữ khoảng chờ 25 ms. Quy trình: ghi speed=0 khi OFF, bật bánh, đọc lại
thanh ghi speed target 0x20 để xác nhận 0. Trong thời gian chờ chỉ được ACK
reference bánh bằng 0 đã kiểm tra; **nonzero chưa được ghi/ACK**. HOME của LF
có hai tốc độ bằng 0, nên không phải tiêu gần hết ngân sách ACK 30 ms để chờ
25 ms này. Nếu đọc lại không đúng 0, fault và OFF. Khả năng READ 0x20 trên driver
thật vẫn cần xác nhận khi thử bus; test hiện tại dùng driver giả lập.

Service chạy mỗi 500 us, không gọi I/O motor. Mỗi bus có task riêng priority18
trên core1; service priority20. Worker kiểm deadline độc lập giữa các giao dịch,
kể cả khi service/UART/console bị treo. Lệnh trùng không gia hạn lease; batch cũ
quá deadline bị hủy dù vẫn có lệnh mới tới. Lỗi một bus hủy cả hai bus.

Adapter UART có tổng ngân sách mỗi giao dịch: joint 4 ms, wheel 8 ms, kiểm echo,
checksum, ID, length, opcode/address echo và status. Đây là giới hạn chờ phần
mềm, không phải số đo phần cứng. OFF có thể phải đợi giao dịch đang chạy; khi
bus đứt, không thể bảo đảm driver đã tắt chỉ nhờ gửi OFF. Chưa có bằng chứng về
watchdog nội tại driver hoặc ngắt lực độc lập khi ESP mất nguồn/reset.

Timestamp encoder bánh chỉ cập nhật sau READ 0x24; ACK ghi tốc độ không làm
mới nó. Sáu encoder phải mới <=20 ms. Bánh unwrap modulo1024; khoảng cách mẫu
và cap950RPM giữ chuyển động dưới nửa vòng/mẫu. Mất mẫu vượt cửa sổ làm fault,
không đoán số vòng qua một khoảng đứt dài. Hip/crank dùng encoder signed
multi-turn, qd từ sai phân timestamp; không tự wrap biên int16 thành target hợp lệ.

## Độ khớp và những khác biệt được công khai

| Phần | Kết luận |
|---|---|
| 4 absolute angles + 2 wheel angular rates | Khớp; đã test cả producer/codec/consumer |
| IDs, GPIO và trái/phải | Dùng hiệu chuẩn GUI và đính chính của người dùng |
| Recovery FF | Đổi hip/knee hinge thành hip/crank velocity trước RPM |
| Wheel recovery filter | tau=.075 s; reference sau lọc/làm tròn trả về LF; không thêm 50 ms delay |
| Joint gains HOME/RECOVERY/POLICY | Hiện đều Kp=10,Kd=.120 theo giới hạn hip đọc thực tế, cấu hình tách từng profile |
| Gain max của driver | Đọc 0x32; nếu gain yêu cầu vượt max thì lỗi Configuration, không tự clamp |
| Giới hạn | Hip/knee lấy rb_dynamic.xml; counts/RPM kiểm biểu diễn; wheel cap950RPM kế thừa bench |
| Quỹ đạo/accel limit bench | Không nhập `leg_move`, jog ramp hoặc wheel ramp600RPM/s vào HCR |
| Feedback torque | Không tạo ước lượng giả; TORQUE_VALID=0 |
| Đáp ứng servo | Chưa chứng minh tương đương servo PD/PI và torque cap của MuJoCo |

Không thể kết luận driver MIT Kp10/Kd.120 giống mô phỏng policy Kp30/Kd1 hoặc
recovery PI có coupling hip/knee. Đây là adapter trực tiếp cho motor hiện có;
fit động học/gain/giới hạn lực phải được đo và thử sau khi nạp. Không thay
trọng số model hoặc thuật toán LF để che khác biệt đó.

Một lượt batch mỗi bus cần 97 byte cả TX/RX: 2 MIT+reply, wheel write+reply,
wheel read+reply. Tối thiểu 0.97 ms truyền trên dây ở 1 Mbaud, chưa kể reply delay
và lịch task. Có thể phục vụ đầu vào200Hz, nhưng tần số áp dụng thực phải đọc
`cycle_max_us`, tuổi encoder và ACK khi thử thật; build/mock không chứng minh200Hz.

Cấu hình nằm trong `include/tap_hw/BlMotor.hpp` (`BlMotorConfig`). Sign và
limit/gain không nhận thay đổi trong lúc ARM; chỉnh cấu hình cần build lại.
Không ghi EEPROM/flash của driver hoặc đổi ID/baud/response level tự động.

## Chuẩn bị vận hành sau khi nạp

1. `STATUS`: cả hai motor bus configured=1, off=1, error=0; IMU hợp lệ.
2. Đặt cơ khí vào tư thế home của model, hip L/R=2.35706715 rad, knee hinge L/R=0.
   Gửi `MOTOR_HOME`. Lệnh chỉ chụp zero; không ra lệnh di chuyển.
   Hoặc `MOTOR_ZERO hipL hipR kneeL kneeR` khi đang ở một tư thế model đã đo khác.
   Cả sáu motor phải OFF, feedback mới và tốc độ tuyệt đối <.15rad/s.
3. Kiểm tra chiều/mốc feedback theo model bằng dữ liệu thật trước khi ARM.
   Zero chỉ giữ trong RAM: reboot/mất encoder/driver fault phải hiệu chuẩn lại.
4. Nếu LF đã fault vì trước đó thiếu calibration, gửi `RESET` sau home, đợi
   DISARMED rồi `ARM`. RESET thông thường giữ home hợp lệ; RESET sau lỗi motor
   xóa home và yêu cầu worker OFF/configure lại. Khi đó làm lại bước2 rồi RESET LF.
5. `DISARM` hoặc `ESTOP` yêu cầu OFF mọi motor. RESET không tự ARM.

Bằng chứng tích hợp ban đầu nằm tại `reports/motor_integration_20261030/`;
bằng chứng flash/kiểm tra kết nối và bản sửa gain nằm tại
`reports/bringup_20261030/` của worktree `luckfox_gateway_esp32`.
Chỉ đã dùng lệnh OFF/RESET và kiểm tra DISABLED, chưa HOME hoặc ARM.
ESP→LF đã nhận 24 HELLO hợp lệ, chiều LF→ESP chưa bắt tay; IMU chưa hiệu chuẩn.
Bản sửa gain đã qua mock tests và build, chưa flash: xem
`gain120_artifacts/checksums.json`. Kết quả mock không chứng minh đáp ứng motor thật.
