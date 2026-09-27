# Firmware v3 — con lắc ngược hai bánh, chân khóa tại tư thế cấp nguồn

Build/nạp: `pio run -e esp32-s3-v3 -t upload`. Điều khiển: `python3 tools/tap_serial_control_gui.py`
(tự kết nối, log ở `~/tap_logs/`). Không dùng BLE. v1 (`esp32-s3-poc`) và v2 giữ nguyên.

## Ý tưởng

- Người vận hành xếp chân (duỗi tự nhiên theo trọng lực) **trước khi cấp nguồn motor**.
  Encoder mỗi GDS68 bằng 0 tại lúc cấp nguồn, nên tư thế đó chính là mốc 0 — không offset,
  không mô hình, không homing.
- Bốn khớp chân được khóa tại encoder = 0 bằng **chế độ vị trí của driver**
  (`Set_Controller_Mode(position, passthrough)` + `Set_Input_Pos`), như robot Impulse.
  Vòng giữ vị trí chạy trong driver (có tích phân). MIT (`LEGMODE MIT`) chỉ là dự phòng.
- Robot còn lại là con lắc ngược hai bánh: `LqrBalancer` với chiều dài con lắc cố định cho
  tư thế chân duỗi, góc cân bằng tự động (seed = pitch lúc thả + tích phân chậm), chống
  tích lũy lệnh, trần tốc độ 0,8 m/s.

## Trình tự (một nút START, robot đặt trên giá)

1. Xếp chân, bật nguồn motor, đặt thân robot lên giá (bánh không chạm đất).
2. **START**: kiểm tra IMU/bánh/chân, mỗi khớp cách mốc 0 không quá 0,35 rad.
3. **Locking**: chân vào closed loop và đi êm về 0 trong 1 s, chờ tổng 1,2 s.
4. **Ready**: bánh bật mô-men nhưng giữ tốc độ 0, chưa chạy vòng cân bằng (không tự
   chạy vọt trên giá). Người vận hành bưng robot xuống sàn, giữ yên ở điểm đứng tốt nhất
   ≥ 0,7 s (tự "armed", góc giữ đó thành seed), rồi buông: khi robot bắt đầu nghiêng
   (tốc độ nghiêng > 0,12 rad/s hoặc lệch > 0,02 rad) thì vào Balancing. Nút **GO** vào
   ngay. `SET auto 0` tắt tự động.
5. **Balancing**: bộ ước lượng pitch chuyển sang chế độ tin gyro (xem dưới), PID hai vòng.
6. DISABLE / ESTOP / mất lệnh serial 0,5 s / nghiêng quá 0,6 rad / bánh > 15 rad/s trong
   0,5 s / mất cảm biến → dừng bánh (chân vẫn khóa tới DISABLE).

## Bộ cân bằng (PID hai vòng, mặc định)

- Vòng trong: `a = kp·(θ − ref) + kd·θ̇`, tích phân thành tốc độ bánh (±8 m/s², ±0,8 m/s,
  lệnh không vượt tốc độ đo quá 0,3 m/s).
- Vòng ngoài: `u = kv·(v − v_lệnh) + kx·(x − x_đích)`, `ref = θ_b − clamp(u, ±0,2)`.
- Góc cân bằng động: `θ_b −= kix·(x − x_đích)·dt` (dừng khi `u` bão hòa hoặc đang DRIVE),
  kẹp seed ± 0,2 rad. Hội tụ về góc cân bằng thật trong ~10–15 s, robot quay về chỗ thả.
- Bù trễ bánh: lệnh gửi = tốc độ tích phân + a·lead (lead 0,12 s), vì driver là servo tốc độ
  trễ 0,1–0,15 s.
- Mặc định kp 30, kd 4, lead 0,12, kv 0,1, kx 0,1, kix 0,02. Quét bền vững trên mô phỏng
  vật lý (khối lượng ±30 %, quán tính −30/+40 %, chiều cao COM ±20 %, trễ bánh 0,05–0,2 s,
  trễ lệnh 4–40 ms): 318/324 đứng, không ngã; không có lead thì kp 30/kd 5 chỉ 53/108.
- Bộ ước lượng khi Balancing (`balancingEstimatorConfig`): hiệu chỉnh gia tốc kế 0,2 Hz,
  tắt nhánh Kalman nhanh, tắt học bias gyro lúc "quiet". Lý do: khi bánh đẩy trên sàn,
  gia tốc kế chỉ thấy ~17 % độ nghiêng thật (IMU gần tâm va chạm), cấu hình gốc (55 /s)
  làm vòng mù và ngã (`v3_physics_tests` T1).

## Lệnh console

`START`, `GO`, `TILTTEST`, `BALANCE`, `BALANCE CLEAR`, `DISABLE`, `ESTOP`, `CLEAR_ESTOP`,
`STOP` (dừng lái), `DRIVE f y`, `GET`, `SET kp|kd|kv|kx|kix|lead|amax|auto|algo|balance v`,
`LEGMODE POS|MIT` (khi đang Idle), `SET trim_gain|trim_max|trim|margin|accel|pitch_scale v`,
`SET leg_kp|leg_kd v` (chỉ cho MIT).

## Log

- `CONTROL mode=Idle|Locking|Balancing|Fault` khi đổi trạng thái; `RELEASE pitch=… seeded_trim=…`.
- `SENS` 5 Hz: `can_legs` (0x009 của cả 4 chân mới), `uart_wheels`, `homed` (=chân đang khóa),
  `joint_q` (chân: rad khớp so với mốc cấp nguồn), pitch, gyro, gia tốc thân/chip, `vbus_min`.
- `TR` 25 Hz khi không Idle: pitch, lệnh/đo bánh, vận tốc/vị trí trục, trim, góc 4 khớp.
- `V3 …`: kết quả lệnh, lý do từ chối, lỗi.

## Kiểm chứng

`src_fw/tests/v3_physics_tests.cpp`: con lắc bánh-thân (Lagrange, khối lượng CAD), servo
bánh trễ 80/120 ms, lực đặc trưng tại IMU qua đúng `ImuStateEstimator`: cấu hình gốc ngã;
cấu hình Balancing đứng vững với góc cân bằng ±0,1 rad, seed lệch ±2°/±4,6°, bias trôi,
đẩy 1 N, dốc 0,03 rad; θ_b về đúng góc thật (< 0,001 rad), robot về chỗ thả (< 3 mm).

`src_fw/tests/v3_tests.cpp`: frame khóa chân (vị trí, ramp về 0, resend, IDLE; MIT dự phòng),
trình tự Idle→Locking→Balancing/Fault, và cân bằng vòng kín trên mô hình con lắc bánh-trên-chân
cứng (servo bánh 40 ms, nhiễu cảm biến, điểm cân bằng lệch 0/+5/−6°, tay lệch ±2°): 9/9 đứng
vững, trim hội tụ đúng điểm cân bằng. Chưa chứng minh trên robot.

Việc cần xác nhận trên robot: chế độ vị trí dùng gain đang lưu trong driver
(`pos_gain`, `vel_gain`, `vel_integrator_gain`) — nếu chân vẫn mềm, xem bằng odrivetool.
