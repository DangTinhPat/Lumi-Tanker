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
chỉ có khối lượng ước lượng từ CAD/BOM (4,621 kg, xem `src/tap_sim/model/PHYSICS.md`), chưa có phép đo thật.

## 1.5. Kết nối vật lý & cảnh báo an toàn

Nguồn: `SteadyWin GIM6010-8 Motor Manual_rev2.2.pdf`, mục 2.4.1/2.4.2 và
mục "准备工作" (chuẩn bị trước khi chạy).

**Connector trên board** (tùy batch/board revision, có thể tách rời hoặc
gộp chung — kiểm tra thực tế trước khi đấu):
- **XT30PB(2+2)**: 2 chân to (nguồn V+/GND) + 2 chân nhỏ (CANH/CANL) — có
  thể là 1 connector tích hợp 4 chân, hoặc 2 connector rời tùy board.
- **Type-C**: **CHỈ để debug/data qua odrivetool, KHÔNG cấp được điện**.
  Nguyên văn manual: *"Type-C 数据线并不能给驱动器供电，更不能驱动电机转
  动！"* (Type-C data cable KHÔNG THỂ cấp nguồn cho driver, càng không thể
  làm motor quay). Toàn bộ board (cả logic lẫn tầng công suất) chỉ có
  **1 nguồn điện duy nhất** — qua connector XT30 24V — không có khái niệm
  "nguồn riêng cho driver" tách khỏi "nguồn cho motor".
- Nếu connector H/L không có chân GND riêng: lấy GND từ chân GND của
  connector nguồn XT30 (GND luôn là 0V bất kể chân V+ cạnh nó là bao
  nhiêu volt — an toàn để dùng chung).

**Cảnh báo an toàn (nguyên văn từ manual, mục "准备工作")**:
- ⚠️ **Không có bảo vệ đảo cực nguồn** — đấu sai V+/GND có thể **cháy
  driver ngay lập tức**. Đo bằng VOM xác nhận đúng cực trước khi cắm.
- ⚠️ **Không cắm/rút nguồn khi đang có điện** (no hot-plug nguồn).
- ⚠️ **Không dùng CB/aptomat (空开) làm công tắc nguồn** — có nguy cơ đánh
  hỏng chip nguồn trên driver.
- ⚠️ **Không chạm tay trực tiếp vào dây bus CAN** (H/L) — tĩnh điện từ tay
  có thể đánh hỏng chip giao tiếp, đặc biệt ở nơi khô/mùa hanh.
- ⚠️ Điện áp nguồn không vượt quá **72V**.
- Đảo H↔L: **không gây hư hỏng** (khác với đảo cực nguồn), chỉ khiến giao
  tiếp không hoạt động — an toàn để thử đảo khi debug.
- 120Ω termination: board có sẵn điện trở 120Ω on-board, bật/tắt qua
  `odrv0.can.config.enable_r120` + `odrv0.can.config.r120_gpio_num` (không
  phải jumper vật lý trên GDS68). Với bus chỉ 2 node (adapter + 1 driver),
  cả 2 đầu nên bật termination (adapter thường có switch/jumper vật lý
  riêng, khác cơ chế).

## 2. Bring-up bắt buộc trước khi chạy động cơ thật

Thứ tự thực hiện, mỗi mục là điều kiện tiên quyết của mục sau:

1. Cài **driver libusb** cho cổng USB của mỗi driver (driver WinUSB mặc
   định của Windows không dùng được) — dùng `zadig-2.9.exe` (có sẵn trong
   `Document-of-GDS68-Driver.zip`). **Trên Linux không cần Zadig**, nhưng
   cần cài udev rules để user thường (không sudo) truy cập được USB:
   ```
   sudo bash -c "curl -fsSL https://cdn.odriverobotics.com/files/odrive-udev-rules.rules \
     > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"
   ```
   Thiết bị debug xuất hiện dạng `/dev/ttyACM*`, VID:PID `1209:0d32`,
   product string là tên fork firmware thực tế (vd `"CyberBeast BL72 -
   Protocol"`), không phải "ODrive" — bình thường, đây vẫn đúng thiết bị
   (`1209:0d32` là mã ODrive chuẩn).

   **Quan trọng — chọn đúng bản `odrivetool`:** package `odrive` trên PyPI
   bản mới nhất (0.6.x) dùng protocol **libfibre** mới (target ODrive
   Pro/S1), **không tương thích** với firmware dòng cũ (protocol JSON-
   endpoint/fibre v1) mà GDS68 (hw_version 3.x) đang chạy — `odrivetool`
   0.6.x sẽ "found Device" nhưng **không bao giờ bind được biến `odrv0`**
   (treo vô thời hạn ở "Please connect your ODrive", không báo lỗi rõ
   ràng). Phải dùng **`odrive==0.5.4`** (hoặc trong khoảng 0.5.x) để bind
   được `odrv0` thành công. Cài song song không đụng bản khác (nếu đã lỡ
   cài 0.6.x cho việc khác):
   ```
   pipx install odrive==0.5.4 --suffix _legacy
   odrivetool_legacy shell --no-ipython   # dùng --no-ipython nếu ipython
                                            # cài kèm bị lỗi tương thích
   ```
2. Firmware động cơ phải **≥ 0.5.13**. Đọc bằng
   `odrv0.fw_version_major/minor/revision` — lưu ý số này **khác** số
   version của package `odrivetool` (mục 1), đừng nhầm lẫn 2 con số.
3. Set `odrv0.axis0.config.can.node_id` **riêng cho từng driver** qua
   MotorWizard/odrivetool — mặc định cả 4 driver đều là `0`, đấu chung bus
   mà không đổi sẽ đụng CAN ID:

   Mapping thực tế đã kiểm chứng trên robot ngày 2026-09-25:

   | Khớp | node_id | Dấu driver so với core |
   |---|---:|---:|
   | RightHip | 1 | -1 |
   | RightKnee | 2 | +1 |
   | LeftHip | 3 | +1 |
   | LeftKnee | 4 | -1 |

   Xem `GDS68_MOTOR_MAPPING.md` và
   `src/tap_control/hal/include/robot_hal/MotorMapping.hpp`. Không dùng lại
   mapping ý định cũ `LeftHip=1, LeftKnee=2, RightHip=3, RightKnee=4`.

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

5. **Chỉ áp dụng nếu `odrv0.hw_version_major/minor` ≥ 3.10**: chân H/L
   trên board này được multiplex dùng chung cho CAN/RS485/Pulse-Direction/
   PWM. Phải xác nhận (và set nếu cần) `odrv0.config.comm_intf_mux = 0` để
   đảm bảo chân đang ở chế độ CAN — nếu không, dù `enable_can_a = True`
   và mọi config CAN khác đúng, board vẫn hoàn toàn im lặng trên bus vì
   tín hiệu vật lý đang đi hướng khác (không phải lỗi dây/wiring).
6. Kiểm tra `odrv0.axis0.config.can.heartbeat_rate_ms` (mặc định thường
   ~100ms) — driver tự phát Heartbeat (cmd 0x001) định kỳ không cần host
   yêu cầu, ngay khi CAN đã enable đúng. Đây là cách nhanh nhất để xác
   nhận CAN bus đã thông: lắng nghe thụ động vài giây, nếu thấy Heartbeat
   lặp lại đúng chu kỳ là bus OK.

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

## 6. Troubleshooting — CAN bus im lặng hoàn toàn (không nhận được gì)

Thứ tự chẩn đoán khi bus không có bất kỳ frame nào (không cả Heartbeat
tự động), rút ra từ 1 lần debug thực tế mất nhiều giờ:

**⚠️ Làm bước 1-2 TRƯỚC KHI cắm USB-CAN-A / bắt đầu debug CAN bus** — bài
học rút ra từ thực tế: lần đầu debug đã cắm thẳng USB-CAN-A và **đoán mù
`node_id`** (thử lần lượt 0-4) trước, mất nhiều giờ nghi ngờ dây/hardware,
trong khi lẽ ra chỉ cần cắm Type-C đọc `node_id` thật (hóa ra đã có sẵn
= 1, không phải mặc định 0) là biết ngay phải target ID nào, tiết kiệm
toàn bộ thời gian đoán mù đó. Với **4 động cơ giống hệt nhau dùng chung
1 bus**, bắt buộc phải làm bước này cho **từng con riêng lẻ qua Type-C**
trước khi ghép chung CAN bus (xem thêm mục 2 bước 3 — set node_id khác
nhau cho mỗi khớp, dán nhãn vật lý ngay sau khi set).

1. **Xác nhận driver còn sống**: đèn nguồn (LED trắng trên board) sáng +
   `odrv0.vbus_voltage` đọc đúng qua USB debug. **Chỉ cần Type-C + nguồn
   24V, KHÔNG cần USB-CAN-A ở bước này.**
2. **Đọc `node_id` thật + xác nhận config phần mềm phía driver** (qua
   `odrivetool_legacy`, xem mục 2 bước 1): `axis0.config.can.node_id`
   (đừng giả định là 0!), `enable_can_a=True`, `can.config.baud_rate`
   khớp với adapter, `config.comm_intf_mux=0` (nếu hw≥3.10), `can.config.
   enable_r120=True`, không có lỗi (`odrv0.can.error`, `axis0.error`,
   `axis0.motor.error`, `axis0.encoder.error` đều 0), đã calibrate
   (`axis0.motor.is_calibrated` + `axis0.encoder.is_ready` đều True).
3. **Test adapter CAN độc lập bằng Loopback** trước khi nghi ngờ dây/motor:
   set `mode=0x01` (Loopback) trong config frame của adapter, gửi 1 data
   frame bất kỳ — nếu nhận lại y hệt, code/protocol/checksum phía host
   đã đúng 100%. **Lưu ý**: Loopback chỉ test nội bộ chip, KHÔNG chạm tới
   transceiver CAN vật lý → đèn TX/RX trên adapter sẽ KHÔNG sáng trong
   Loopback dù test pass — đây là bình thường, không phải dấu hiệu lỗi.
4. **Đọc đèn LED trên adapter ở chế độ Normal (không phải Loopback)**:
   - Đèn **PWR** sáng: adapter có điện, bình thường.
   - Đèn **TX** nháy khi gửi lệnh: adapter đang đẩy tín hiệu ra transceiver
     vật lý — xác nhận phía TX của adapter hoạt động.
   - Đèn **RX** không nháy dù đang lắng nghe: adapter không nhận được gì
     từ bus — nghi vấn dồn về dây nối hoặc phía phát của driver.
   - Nếu **cả TX lẫn RX đều không sáng** dù đã gửi lệnh liên tục ở chế độ
     Normal (khác Loopback): nghi ngờ cao adapter có vấn đề ở tầng
     transceiver vật lý, không liên quan gì tới driver/dây. Thử rút—cắm
     lại adapter (có thể do treo trạng thái, tự hết sau khi cấp nguồn
     lại) trước khi kết luận hỏng hẳn.
5. **Đo thông mạch (continuity) từng dây** giữa adapter và driver: GND,
   H, L — que đỏ 1 đầu, que đen đầu kia, chế độ đo continuity/diode của
   VOM, phải kêu bíp / ~0Ω. Đây là cách nhanh nhất phát hiện dây tưởng
   như đã cắm nhưng thực chất hở mạch (jumper wire lỏng, đứt ngầm bên
   trong vỏ nhựa dù ngoài nhìn còn nguyên).
6. Thử đảo H↔L (an toàn, xem mục 1.5) nếu nghi ngờ nhầm cực — dù đã đảo
   thử cả 2 chiều mà vẫn im lặng thì đây không phải nguyên nhân.
7. Quét toàn bộ baudrate CAN khả dĩ (1M/800k/500k/400k/250k/200k/125k/
   100k/50k/20k/10k/5k) phòng trường hợp giá trị mặc định 500kbps đã bị
   đổi trên 1 trong 2 bên.

Kinh nghiệm thực tế: dù toàn bộ config phần mềm đã xác nhận đúng 100%
(driver enable CAN, đúng node_id/baud/mux, termination bật, calibrated,
adapter loopback pass), bus vẫn có thể im lặng thuần túy do **tiếp xúc
vật lý** (dây jumper lỏng/oxy hóa) hoặc **adapter tạm thời bị treo** (tự
khắc phục sau khi rút-cắm lại nguồn) — luôn kiểm tra lại các bước 3-5
trước khi kết luận lỗi phần cứng vĩnh viễn.

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
