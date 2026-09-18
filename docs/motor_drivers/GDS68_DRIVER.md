# Driver GDS68 (động cơ GIM6010-8) — SteadyWin

Dùng cho 4 khớp CAN 24V: LeftHip, LeftKnee, RightHip, RightKnee. Nguồn:
`SteadyWin GIM6010-8 Motor Manual_rev2.2.pdf` (trong
`Document-of-GDS68-Driver.zip`, steadywin-motor.com). Firmware GDS68 dựa
trên **ODrive**, không phải protocol riêng của SteadyWin.

## 1. Thông số

| | |
|---|---|
| Điện áp danh định / min-max | 15~48V DC / 12~72V DC |
| Dòng danh định / dòng pha tối đa | 6A / 120A |
| Hộp số | 8:1 |
| Tốc độ danh định / tối đa | 120 rpm / 420 rpm |
| Mô-men danh định | 5 N·m |
| Mô-men kẹt trục (stall) | 11 N·m |
| Dòng danh định / kẹt trục | 10.5A / 25A |
| Hằng số mô-men (Kt) | 0.47 N·m/A |
| Encoder | 14-bit tuyệt đối, 1 vòng |
| Khối lượng | 388g |
| CAN | tối đa 1 Mbps, mặc định xuất xưởng 500 kbps |

Nguồn 24V của robot nằm trong dải điện áp danh định → tương thích.

**Chưa xác minh:** 5 N·m danh định / 11 N·m stall có đủ cho khớp đùi/gối
robot hay không — cần khối lượng robot + chiều dài tay đòn để tính. Robot
hiện chưa có dữ liệu mass (URDF chỉ là mesh preview, không có `<inertial>`).

## 2. Bring-up bắt buộc trước khi chạy động cơ thật

Thứ tự thực hiện, mỗi mục là điều kiện tiên quyết của mục sau:

1. Cài **driver libusb** cho cổng USB của mỗi driver (driver WinUSB mặc
   định của Windows không dùng được).
2. Firmware động cơ phải **≥ 0.5.13**.
3. Set `odrv0.axis0.config.can.node_id` **riêng cho từng driver** qua
   MotorWizard/odrivetool — mặc định cả 4 driver đều là `0`, đấu chung bus
   mà không đổi sẽ đụng CAN ID:

   | Khớp | node_id |
   |---|---|
   | LeftHip | 1 |
   | LeftKnee | 2 |
   | RightHip | 3 |
   | RightKnee | 4 |

4. **Hiệu chuẩn động cơ + encoder** (1 lần/driver, bắt buộc trước khi có
   thể vào closed-loop control):
   - Chạy `requested_state = 4` (hiệu chuẩn động cơ), sau đó
     `requested_state = 7` (hiệu chuẩn encoder) — hoặc dùng nút "hiệu
     chuẩn" trong MotorWizard.
   - Xác nhận không có lỗi hiệu chuẩn (`dump_errors`).
   - Ghi `odrv0.axis0.motor.config.pre_calibrated = 1` và
     `odrv0.axis0.encoder.config.pre_calibrated = 1`, để các lần khởi
     động sau không cần hiệu chuẩn lại.
   - `odrv0.save_configuration()`.

   Nếu bỏ qua bước này: lệnh `Set_Axis_State = 8` (CLOSED_LOOP_CONTROL) mà
   firmware gửi khi bật `enable_motors` sẽ bị từ chối/báo lỗi, động cơ
   không chạy.

## 3. Giao thức CAN — "CAN Simple"

Frame chuẩn 11-bit ID, 8 byte data, little-endian (trừ MIT_Control, mục 4):

```
CAN ID (11 bit) = (node_id << 5) | cmd_id
```

### cmd_id đã dùng trong firmware này

| cmd_id | Tên | Chiều | Vai trò trong `can_motor.c` |
|---|---|---|---|
| 0x007 | Set_Axis_State | host→motor | IDLE (1) khi disable, CLOSED_LOOP_CONTROL (8) khi bật |
| 0x008 | Mit_Control | 2 chiều | lệnh điều khiển + feedback chính |
| 0x00B | Set_Controller_Mode | host→motor | set control_mode=3, input_mode=9 (MIT) khi bật |

### cmd_id khác có sẵn trong firmware, chưa dùng

| cmd_id | Tên | Chiều |
|---|---|---|
| 0x001 | Heartbeat | motor→host |
| 0x002 | Estop | host→motor |
| 0x003 | Get_Error | motor→host |
| 0x006 | Set_Axis_Node_ID | host→motor |
| 0x009 | Get_Encoder_Estimates | motor→host |
| 0x00C/D/E | Set_Input_Pos/Vel/Torque | host→motor |
| 0x00F | Set_Limits | host→motor |
| 0x014 | Get_Iq | motor→host |
| 0x016 | Reboot | host→motor |
| 0x017 | Get_Bus_Voltage_Current | motor→host |
| 0x018 | Clear_Errors | host→motor |
| 0x01C | Get_Torques | motor→host |
| 0x01D | Get_Powers | motor→host |
| 0x01E | Disable_Can | host→motor |
| 0x01F | Save_Configuration | host→motor |

## 4. Mit_Control (0x008) — dùng cho cả 4 khớp

4 chế độ điều khiển driver hỗ trợ: Position / Velocity / Torque / Motion
(MIT). Firmware này dùng MIT — công thức chuẩn cho khớp cần phản ứng lực
nhanh:

```
T_target = Kp × (Pos_target − Pos) + Kd × (Vel_target − Vel) + T_ff
```

Bật MIT: `control_mode = 3` (position), `input_mode = 9` (MIT) qua
Set_Controller_Mode (0x00B), rồi Set_Axis_State (0x007) = 8. Thứ tự này
khớp với ví dụ thực chiến trong manual (mục 4.1.3.2/4.1.3.3): gửi
Set_Controller_Mode trước, Set_Axis_State(8) sau, không có độ trễ bắt buộc
nào được ghi giữa 2 lệnh.

### Khung host→motor (8 byte, bit-packed, không phải little-endian byte-wise)

| Byte | Nội dung | Range |
|---|---|---|
| 0–1 | Position, 16 bit | -12.5 .. 12.5 rad |
| 2, 3[7:4] | Velocity, 12 bit | -65 .. 65 rad/s |
| 3[3:0], 4 | Kp, 12 bit | 0 .. 500 |
| 5, 6[7:4] | Kd, 12 bit | 0 .. 5 |
| 6[3:0], 7 | Torque feedforward, 12 bit | -50 .. 50 N·m |

### Khung motor→host (feedback)

| Byte | Nội dung |
|---|---|
| 0 | node_id (echo, không dùng để lọc — đã lọc bằng CAN arbitration ID) |
| 1–2 | Position, 16 bit, cùng range trên |
| 3, 4[7:4] | Velocity, 12 bit |
| 4[3:0], 5 | Torque (ước tính từ dòng Iq×Kt), 12 bit |

Vị trí/tốc độ/mô-men trong khung MIT là ở **trục ra** (output shaft, sau
hộp số 8:1) — khác với USB/odrivetool (ở đó là trục rotor, trước hộp số).

Tất cả 5 trường trên đều dùng chung 1 công thức lượng tử hoá tuyến tính:
`raw = (x − x_min) × (2^bits − 1) / (x_max − x_min)`, clamp `x` vào
`[x_min, x_max]` trước khi quy đổi — khớp trực tiếp với công thức manual
cho từng trường (vd `pos_int = (pos + 12.5) × 65535 / 25`).

### Ví dụ số cụ thể

Node_id = 1, muốn set Position = 0 rad (giữa dải), Velocity = 0 rad/s,
Kp = 250 (giữa dải 0..500), Kd = 2.5 (giữa dải 0..5), Torque feedforward
= 0 N·m — cố ý chọn toàn giá trị giữa dải để ra số tròn, dễ verify:

```
pos_int = (0 + 12.5) × 65535 / 25   = 32767  = 0x7FFF
vel_int = (0 + 65) × 4095 / 130     = 2047   = 0x7FF
kp_int  = 250 × 4095 / 500          = 2047   = 0x7FF
kd_int  = 2.5 × 4095 / 5            = 2047   = 0x7FF
t_int   = (0 + 50) × 4095 / 100     = 2047   = 0x7FF

Byte0 = 0x7FFF >> 8                   = 0x7F
Byte1 = 0x7FFF & 0xFF                 = 0xFF
Byte2 = 0x7FF >> 4                    = 0x7F
Byte3 = ((0x7FF&0xF)<<4) | (0x7FF>>8) = 0xF0 | 0x07 = 0xF7
Byte4 = 0x7FF & 0xFF                  = 0xFF
Byte5 = 0x7FF >> 4                    = 0x7F
Byte6 = ((0x7FF&0xF)<<4) | (0x7FF>>8) = 0xF0 | 0x07 = 0xF7
Byte7 = 0x7FF & 0xFF                  = 0xFF

CAN ID = (1 << 5) | 0x008 = 0x028
Frame:  ID=0x028  Data= 7F FF 7F F7 FF 7F F7 FF
```

### Quickstart — bật 1 khớp (node_id=1) từ trạng thái nguội

Giả sử đã hiệu chuẩn xong (mục 2, bước 4). Gửi tuần tự 3 khung CAN sau:

```
1) ID=0x02B  Data=03 00 00 00 09 00 00 00    (Set_Controller_Mode: control_mode=3, input_mode=9)
2) ID=0x027  Data=08 00 00 00 00 00 00 00    (Set_Axis_State: requested_state=8, closed-loop)
3) ID=0x028  Data=7F FF 7F F7 FF 7F F7 FF    (Mit_Control, ví dụ mục 4)
```

`(node_id=1)<<5 = 0x020`; cộng cmd_id: 0x020|0x00B=0x02B, 0x020|0x007=0x027,
0x020|0x008=0x028.

## 5. Đo dòng → ước lượng mô-men (effort_nm)

Driver dùng FOC nên luôn đo được dòng trục Q (Iq). Mô-men ước tính =
`Iq_measured × Kt` (Kt = 0.47 N·m/A). Giá trị này được motor tự tính và trả
về trong trường "Torque" của khung Mit_Control feedback — không cần đọc
thêm Get_Iq (0x014) hay Get_Torques (0x01C).

Giới hạn của cách ước lượng này:
- Không trừ ma sát/hiệu suất hộp số 8:1 → sai lệch so với mô-men ngoại lực
  thực, đặc biệt lúc va chạm/tải động.
- Không có cảm biến lực riêng trên board để đối chiếu.
- Dùng làm proxy phát hiện phản lực/va chạm cần kết hợp ngưỡng + đạo hàm,
  không dùng 1 giá trị tức thời.

---

## Phụ lục: tích hợp trong firmware dự án này (không thuộc protocol chuẩn)

Phần dưới đây mô tả cách file `form/components/robot_base/src/can_motor.c`
trong repo này dùng protocol ở trên — chỉ liên quan nếu đang đọc/sửa code
đó, không cần thiết để tự triển khai protocol từ đầu.

### Map vào `robot_motor_command_t` (`can_motor.c`)

| Field | Vai trò trong khung MIT |
|---|---|
| `position_rad[joint]` | Position |
| `velocity_rad_s[joint]` | Velocity |
| `kp_scale[joint]` | Kp |
| `kd_scale[joint]` | Kd |
| `feedforward_nm[joint]` | Torque feedforward (T_ff) |
| `torque_nm[joint]` | Không dùng cho khớp CAN — đây là field điều khiển mô-men trực tiếp (ứng với Set_Input_Torque, mode khác MIT), không có chỗ trong khung MIT |

`enable_motors=false` → gửi Set_Axis_State IDLE (1) mỗi tick (không phải im
lặng — im lặng không đảm bảo động cơ dừng nếu driver có watchdog giữ lệnh
cuối). Chuyển `false→true` → gửi Set_Controller_Mode + Set_Axis_State
CLOSED_LOOP đúng 1 lần ở cạnh chuyển trạng thái, sau đó bắt đầu gửi khung
MIT ngay trong cùng tick (manual không ghi yêu cầu độ trễ giữa các lệnh
này).

Feedback (`position_rad`/`velocity_rad_s`/`effort_nm`) chỉ được cập nhật
khi driver đã đóng vòng (closed-loop) và có gửi khung MIT phản hồi.
`robot_can_motor_feedback_ready()` dùng để chặn `enable_motors` cho tới khi
có feedback thật từ cả 4 khớp, tránh thuật toán đọc nhầm state zero-init
làm state thật.
