# Luồng động cơ HCR–BL4012e — 2026-10-30

Backend motor dùng `BlMotorBackend`/`EspBlMotor`. Bản ứng viên hiện tại **biên dịch
trực tiếp module `joint_bus.c` kế thừa từ BL4012e/3in1**, qua adapter C++ mỏng;
bản upstream mới dùng FIFO TX + nhả pad khi TX idle, gap100us và drain late reply.
Adapter không tự thêm OEN/RMT; deadline HCR được đưa xuống module. Build dùng Arduino 3.3.0 / ESP-IDF 5.5.0.
Xem [bản sửa và mức kiểm chứng](validation-bus-recovery.md),
[nguồn module và delta](../drivers/bl4012e/PROVENANCE.md).

Các kết quả nạp/đo trong lịch sử thuộc firmware trước. Bản kế thừa này chưa được
nạp hoặc thử chuyển động. Không coi các lần đọc encoder khi OFF thành công là
bằng chứng lỗi khi bắt đầu gập chân đã được sửa.

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
chỉ true sau READ thanh ghi Torque Enable `0x18` bằng 0 của cả sáu motor.
E-stop buộc ghi/đọc lại mọi driver; disarmed kiểm lại định kỳ 100 ms. OFF là
trạng thái thanh ghi, không phải phép đo lực cơ khí hoặc bánh quay tự do. Kênh bị bỏ enable không được ghi lệnh
motion; wheel applied=0, các góc giữ reference cuối đã chốt (hoặc góc đo nếu
chưa có reference). Không diễn giải zero của DISABLED thành góc đích 0.

## Sửa tích hợp ngày 2026-10-31

Boot/configure khi disarmed không ghi wheel speed, chỉ OFF và đọc feedback.
Khi đã được phép điều khiển, nếu speed write thiếu reply thì đọc lại target
`0x20`; RPM phải khớp mới công nhận. Echo/TX/CRC/status lỗi, readback không
khớp hoặc timeout vẫn fault. READ present `0x24` chứng minh feedback, không
chứng minh target đã latch. TTL 20 ms và điều kiện cả hai bus hoàn tất giữ
nguyên; nhánh readback thêm độ trễ cần đo thực tế.

`MOTOR_DIAG` tách lỗi từng bus khỏi lỗi chung. `first_fail_id/inst/addr/bus_stage`
giữ giao dịch lỗi đầu tiên tới khi reset backend; OFF retries không ghi đè.
Stages: 0=none/status, 1=unavailable, 2=packet, 3=TX, 4=echo timeout,
5=echo mismatch, 6=reply timeout, 7=reply invalid, 8=readback mismatch,
9=deadline, 10=cancelled, 11=bus chưa yên sau drain.
`recovered_writes` đếm mất ACK nhưng readback đúng. `enable_readback` theo
hip/crank/wheel; 255=unknown. OFF mất ACK nhưng readback đúng có thể xuất hiện
trong first_fail mà không latch lỗi; đối chiếu cả local_fault và off.

Xem [báo cáo](../reports/continue_20261031/RESULTS.md) cho mức kiểm chứng hiện tại.

## Khởi động bánh và watchdog

BL4012e bench ghi nhận phải chờ >=20 ms sau bật lực trước ghi tốc độ mới;
backend giữ khoảng chờ 25 ms. Quy trình: ghi speed=0 khi OFF, bật bánh, đọc lại
thanh ghi speed target 0x20 để xác nhận 0. Trong thời gian chờ chỉ được ACK
reference bánh bằng 0 đã kiểm tra; **nonzero chưa được ghi/ACK**. HOME của LF
có hai tốc độ bằng 0, nên không phải tiêu gần hết ngân sách ACK 30 ms để chờ
25 ms này. Nếu đọc lại không đúng 0, fault và OFF. Khả năng READ 0x20 trên driver
thật đã xác nhận bằng READ khi OFF: cả hai bánh đọc target=0.
Chuỗi bật bánh/chờ 25 ms/ghi tốc độ vẫn chỉ có bằng chứng mock, chưa thử chuyển động.

Service chạy mỗi 500 us, không gọi I/O motor. Mỗi bus có task riêng priority18
trên core1; service priority20. Worker kiểm deadline độc lập giữa các giao dịch,
kể cả khi service/UART/console bị treo. Lệnh trùng không gia hạn lease; batch cũ
quá deadline bị hủy dù vẫn có lệnh mới tới. Lỗi một bus hủy cả hai bus.

Module mới dùng TX spin có trần1ms, echo/joint reply3ticks, wheel reply9ticks
và tối đa3ticks cho byte cuối khi cần. HCR cap thời gian chờ theo deadline
batch/lease, kiểm lại sau drain/gap và trước TX/ACK; không gia hạn khi retry.
OFF/configuration cũng có budget20ms mỗi lần gọi module. Đây là điều kiện phần
mềm, không phải hard deadline cắt lực; OFF có thể phải đợi read/TX đang chạy.
Xem báo cáo mới cho khác biệt giữa source/build/mock và timing vật lý chưa đo.

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

Khi RPM không đổi và đã được xác nhận trong cùng phiên enable, gateway bỏ qua
WRITE lặp cho tới keepalive100ms; encoder vẫn được đọc mỗi batch/poll. Batch
bỏ WRITE còn79byte/bus. RPM thay đổi được gửi ngay; không nhập slot10ms của bench.
`BUS_RECOVERY` thống kê raw errors, late/sync, retries/recovered và skipped writes;
first-failure vẫn giữ khi retry thành công, không tự mang nghĩa local_fault.

Cấu hình nằm trong `include/gateway/BlMotor.hpp` (`BlMotorConfig`). Sign và
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
   DISARMED rồi `ARM`. RESET thông thường giữ home hợp lệ; CLEAR_FAULT cục bộ sau lỗi motor
   xóa home và yêu cầu worker OFF/configure lại. RESET LF không xóa E-stop/lỗi
   driver cục bộ; CLEAR_FAULT chỉ được chấp nhận sau verified OFF. Khi đó làm lại bước2 rồi RESET LF.
5. `DISARM` hoặc `ESTOP` yêu cầu OFF mọi motor. RESET không tự ARM.

Bằng chứng tích hợp ban đầu nằm tại `reports/motor_integration_20261030/`;
bằng chứng flash/kiểm tra kết nối và bản sửa gain nằm tại
`reports/bringup_20261030/` của worktree `luckfox_gateway_esp32`.
Chỉ đã dùng lệnh OFF/RESET và kiểm tra DISABLED, chưa HOME hoặc ARM.
ESP→LF đã nhận 24 HELLO hợp lệ, chiều LF→ESP chưa bắt tay; IMU chưa hiệu chuẩn.
Bản sửa gain đã qua mock tests và build, chưa flash: xem
`gain120_artifacts/checksums.json`. Kết quả mock không chứng minh đáp ứng motor thật.

## Lịch sử: thử sửa turnaround ngày 2026-10-31 (đã thay thế)

Giả thuyết của đợt sửa cũ là đợi TX qua scheduler rồi nhả pad làm mất đầu gói.
Đây không phải kết luận nguyên nhân cuối cùng; bản FIFO sau đó vẫn lỗi khi chuyển động.
Adapter nạp FIFO và đợi TX idle trong critical section có giới hạn, sau đó
gọi `gpio_ll_output_disable` để nhả chân. Bỏ lần đổi GPIO matrix dư trước
output-disable vì có thể nuốt start bit đầu. Không bỏ checksum, không thêm retry
để che lỗi. BUS_TRACE lưu timing/byte thô của lỗi đầu; UART_DIAG tách lỗi queue.
Các số liệu bring-up phía trên là lịch sử; trạng thái mới xem report 2026-10-31.

## Lịch sử chẩn đoán lỗi khi bắt đầu chuyển động (2026-10-31)

ARM thực đã qua HOME3s và STANDUP~0.14s, sau đó UART1/phải, wheelID1,
WRITE0x20 mất echo (0 byte), timeout~8ms. LF FAULT3 là phản hồi lỗi MCU;
không phải lỗi token hoặc lỗi tính bảng LQR đã sửa. Firmware bổ sung BUS_HW:
packetTX/echo, mức pad trước/phát xong/kết thúc, RX FIFO lúc nhả pad, ring bytes,
raw/enable interrupt, status và bitmask uart_event_type_t. Trace chỉ ghi nhận,
không bỏ kiểm echo/checksum, không retry để che lỗi.

`MOTOR_BUS_TEST`: chỉ nhận khi cả hai bus configured và verifiedOFF, không
maintenance/lỗi/batch hoạt động. Hủy HOME; mỗi bus chạy2000chu kỳ ghi wheel
speed0, READ0x20 kiểm0 và đọc encoder. Không gửi MIT hoặc enable. Trong test
không được HOME/ARM; sau test phải chụp HOME đúng tư thế. OTA/maintenance dừng
traffic. Đã chạy thật4000chu kỳ tổng, không lỗi; chưa tái hiện lỗi có chuyển động.
Bằng chứng: `reports/arm_deep_20261031/off_test.txt`.

Bản kế thừa hiện tại thay `BUS_HW`/`BUS_PAD` bằng `BUS_BYTES` và `BUS_TRACE`.
`BUS_DRIVER` ghi nguồn module và phiên bản IDF. `MOTOR_WAVE` báo unavailable;
không đưa các số 0 ra như thể đã đo mức điện tại chân.
