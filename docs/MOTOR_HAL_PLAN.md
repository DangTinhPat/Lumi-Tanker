# Kế hoạch lớp HAL động cơ (CAN GDS68 + UART bánh xe)

Bản kế hoạch, chưa có dòng code nào được viết. Rút ra từ `docs/actuatal_driver/*`
đối chiếu với code thực tế trong `src/tap_control/core` và
`src/tap_sim/native/native_runner.cpp`. Mọi con số đều kèm file:line.

Không cho phép vận hành phần cứng, nạp firmware, hay cấp lệnh động cơ thật.
Rule 4 (`docs/RULES.md`) giữ nguyên: sim là môi trường kiểm chứng, ESP32-S3
là đích triển khai.

---

## 0. Phạm vi

Lớp biên giữa `MotorCommand`/`RobotState` và 2 bus vật lý: 1 CAN classic cho
4 khớp chân, 2 UART cho 2 bánh. Quy ước đơn vị, dấu, frame, ngân sách thời
gian, chiến lược test.

Ngoài phạm vi: MPU6500 HAL (đi I2C/SPI riêng, không đụng 2 bus này),
nguồn/điện, cơ khí, và mọi việc chạy trên phần cứng thật ngoài bench một
động cơ.

---

## 1. Bản đồ phần cứng

Thứ tự khớp trong core cố định: `[LH, LK, LW, RH, RK, RW]` (`Types.hpp:13-20`).
Bảng dưới là bản dịch duy nhất từ index đó sang địa chỉ vật lý — không có
bản sao thứ hai ở bất cứ đâu khác. CAN chân đã được kiểm chứng trên robot
ngày 2026-09-25; source code chính thức là `robot_hal/MotorMapping.hpp`:

| idx | Khớp | Bus | Địa chỉ | Driver sign | Driver | Chế độ |
|---|---|---|---|---:|---|---|
| 0 | LeftHip | CAN | node_id 3 | +1 | GDS68 | MIT (0x008) |
| 1 | LeftKnee | CAN | node_id 4 | -1 | GDS68 | MIT (0x008) |
| 2 | LeftWheel | UART1 | motor id 0x01 | TBD | BL4820M12 | Moving Speed (0x20) |
| 3 | RightHip | CAN | node_id 1 | -1 | GDS68 | MIT (0x008) |
| 4 | RightKnee | CAN | node_id 2 | +1 | GDS68 | MIT (0x008) |
| 5 | RightWheel | UART2 | motor id 0x01 | TBD | BL4820M12 | Moving Speed (0x20) |

Khớp đúng ngân sách peripheral ESP32-S3: đúng 1 controller TWAI, và UART1/
UART2 (UART0 để lại cho console). Không còn dự phòng — không tách CAN thành
2 bus, không thêm UART thứ ba.

Bảng CAN chân ở trên đã được kiểm chứng bằng quan sát từng động cơ thật. Phần
bánh UART vẫn cần nghiệm thu riêng. Trước khi đóng bất kỳ vòng kín toàn thân
nào vẫn phải test từng khớp ở command rất nhỏ sau mỗi lần tháo/lắp lại dây.
Đảo trái/phải làm đảo dấu momen roll và đảo hướng yaw — robot lật ngay lần
enable đầu tiên.

---

## 2. Ngân sách thời gian

Sim hiện chạy vòng điều khiển 3000 Hz (`native_runner.cpp:63`). Phần cứng
không theo kịp con số đó — đây là ràng buộc lớn nhất của cả kế hoạch.

**CAN, 4 khớp.** Khung 11-bit ID, DLC=8 ≈ 120–135 bit kể cả bit-stuffing xấu
nhất. MIT là 2 chiều, mỗi tick cần 4 khung lệnh + 4 khung feedback = 8 khung:

| Baud | Thời gian 8 khung | Trần lý thuyết | Thực dụng (~60% tải) |
|---|---|---|---|
| 500 kbps (mặc định xuất xưởng) | ~2.16 ms | ~460 Hz | ~280 Hz |
| 1 Mbps (tối đa driver hỗ trợ) | ~1.08 ms | ~930 Hz | ~550 Hz |

Phải đổi CAN sang 1 Mbps — `GDS68_USB_CAN_A_BRINGUP.md` mới chỉ test thành
công ở 500 kbps, nên nâng baud là một mục bring-up riêng, có rủi ro dây/nhiễu/
termination, không phải đổi một biến số.

Encoder chân (`sensor_rate_hz = 500`, `native_runner.cpp:64`) về qua chính
khung feedback MIT trên cùng bus — không có đường riêng. Ở 500 kbps, 500 Hz
encoder vượt trần bus (460 Hz ở 100% tải); con số 500 Hz trong sim chỉ hợp lệ
nếu CAN thật sự chạy 1 Mbps.

**UART, 2 bánh.** 1 Mbps 8N1, mỗi bánh mỗi chu kỳ ghi+đọc ≈ 400 µs thời gian
dây (trần ~2.5 kHz), nhưng turnaround firmware chưa đo — và
`WHEEL_MOTOR_PROTOCOL.md` §7 ghi nhận 20 ms chỉ để nuốt xong một lệnh Torque
Enable. Giả định an toàn 500 Hz–1 kHz, cần đo thật ở Phase 5.

**Kết luận:** CAN là nút cổ chai. Tick chấp hành mục tiêu **500 Hz** ở CAN
1 Mbps, phương án lùi 250 Hz nếu 1 Mbps không ổn định trên harness thật.

Sim hiện validate `--rate` trong `[1000, 4000]` (`native_runner.cpp:116`) —
cận dưới phải nới xuống 500 để chạy kiểm chứng được.

Câu hỏi cốt lõi cần thí nghiệm trả lời, không phải suy luận: **balance/landing
hiện tại có sống sót ở 500 Hz không?** Cho tới khi có câu trả lời, mọi kết quả
sim ở 3 kHz chưa phải bằng chứng về phần cứng.

---

## 3. Ba chỗ rò rỉ

Ba chỗ dưới đây hiện chọc thủng biên actuator. Kiến trúc seam-mức-khung ở §4
ép đóng cả ba, vì khung truyền không có chỗ chứa chúng — liệt kê ở đây để
biết Phase 3 sẽ phải động vào cái gì.

**1. Kênh gia tốc MPC đi vòng qua `MotorCommand` (nghiêm trọng).**
`native_runner.cpp:1329` móc thẳng `controller_.mpcDebugState().optimal_command_m_s`,
rồi `:1093-1095` biến nó thành momen bánh xe:

```cpp
last_mpc_accel_ = controller_.mpcDebugState().optimal_command_m_s;   // :1329
const float longitudinal_torque =
    0.5F * total_mass_kg * wheel_radius_m * last_mpc_accel_;          // :1093
```

Kênh này không tồn tại trong `MotorCommand`, nên phần cứng không thể tái tạo
hành vi mà sim đang chứng minh — đây là đường truyền authority chính của vòng
balance, không phải chi tiết tuning. Và driver bánh xe vốn không nhận momen:
`WHEEL_MOTOR_PROTOCOL.md` §4 chỉ có thanh ghi Moving Speed.

Hướng xử lý: core phát lệnh **vận tốc bánh xe đã bao hàm ý đồ gia tốc của
MPC**, không để lớp mô phỏng rút gia tốc ra sau lưng. `MpcBalancer` vốn đã
tích phân gia tốc thành lệnh vận tốc (`MpcBalancer.hpp:19-22`, đúng upstream
upkie), và `MotorCommand.velocity_rad_s[2]/[5]` đã sẵn sàng nhận nó. Đường
vận tốc từng bị bỏ qua vì "vòng servo thứ hai phanh lại cú bắt của MPC"
(comment ở `:1087-1091`) — nghĩa là bug gốc từng được vá bằng cách đi vòng
qua interface thay vì vá đúng chỗ.

**2. Vòng PD chân chạy sai phía.** `native_runner.cpp:1131-1141` tính PD
chân ở phía host mỗi tick, dùng `state_.joints` — encoder đã trễ, đã nhiễu:

```cpp
const float kp = kLegPositionKpNmPerRad * motors.kp_scale[j];
const float kd = 2*sqrt(kLegPositionKpNmPerRad * joint_inertia) * motors.kd_scale[j];
const float tau = kp*(motors.position_rad[j] - q) - kd*qd + motors.feedforward_nm[j];
```

Trên phần cứng thật, GDS68 tự tính công thức này bên trong, ở tốc độ vòng
lặp nội bộ, bằng encoder cục bộ không trễ (`GDS68_DRIVER.md` §4). Sim đang mô
phỏng một bộ điều khiển khác — chậm hơn và nhiễu hơn — nên mọi tuning Kp/Kd
rút ra từ sim hiện tại đều lệch so với cái sẽ chạy thật.

Hướng xử lý: tách khối PD thành driver mô phỏng chạy ở tốc độ physics với
joint state không trễ, còn target chỉ làm mới ở tốc độ CAN.

**3. `torque_nm` không có chỗ trong khung MIT (nhẹ).** `GDS68_DRIVER.md`
phụ lục ghi rõ trường này không dùng cho khớp CAN, nhưng sim dùng nó làm
ngưỡng clamp momen mỗi tick (`:1141`). May mắn nó là hằng số —
`EmbeddedController.cpp:1786,1864,1868` luôn gán 3.0 N·m (chân) và
0.13 N·m (bánh), không bao giờ đổi theo tick.

Hướng xử lý: đặt một lần lúc bring-up qua `Set_Limits` (0x00F), HAL assert
giá trị bất biến giữa các tick.

---

## 4. Kiến trúc — seam ở mức khung truyền

Biên không nằm ở struct `MotorCommand`, mà nằm ở byte đi trên dây. Controller
luôn đóng gói khung CAN MIT thật và khung UART thật. Phía sim có driver giả
parse đúng những byte đó, mô phỏng hành vi GDS68/BL4820M12, trả lại khung
feedback thật. Sim và phần cứng dùng chung một codec; chỉ đường truyền khác.

```
        EmbeddedController              (core, không biết gì về bus)
               │ MotorCommand                  ▲ RobotState
               ▼                               │
        MotorFrameCodec        ◄── một hiện thực, dùng chung sim + thật
        pack:  4× MIT (11-bit ID, 8 byte)
               2× Dynamixel-like packet
        unpack: khung feedback → JointState
               │ bytes                         ▲ bytes
               ▼                               │
        ┌──────────────────────────────────────────┐
        │  Transport   ◄── chỗ duy nhất có 2 bản   │
        └──────────────────────────────────────────┘
           │                              │
   SimTransport                    HwTransport
     │                               ├── TWAI (CAN 1 Mbps)
     ├── SimGds68 ×4 ──┐             └── UART1 / UART2
     └── SimBl4820 ×2 ─┼──► mjData.ctrl
                       │
        (mô phỏng: PD nội bộ MIT, máy trạng thái
         IDLE/CLOSED_LOOP, heartbeat, settle 20 ms,
         quirk echo 3 byte, trễ bus, mất khung)
```

Lựa chọn này ép đóng cả 3 rò rỉ ở §3 mà không cần kỷ luật hay ghi nhớ:

| Rò rỉ | Vì sao bị ép đóng |
|---|---|
| 1. Gia tốc MPC đi vòng | Khung UART bánh xe chỉ có Moving Speed — không trường nào chở gia tốc, không còn đường vào byte cho `mpcDebugState()` |
| 2. PD chân phía host | Khung MIT chỉ chở `pos, vel, Kp, Kd, T_ff`; bên nhận khung — `SimGds68` — buộc phải tự chạy PD |
| 3. `torque_nm` mỗi tick | Khung MIT không có trường giới hạn momen, buộc đi qua `Set_Limits` lúc bring-up |

Nó cũng biến sim thành bench conformance: lỗi thứ tự enable, thiếu settle
20 ms, sai checksum, sai bit-packing, parse nhầm quirk echo — nổ trong sim
trên host, trước khi chạm phần cứng. Đây đúng loại lỗi mà
`GDS68_USB_CAN_A_BRINGUP.md` §6 ghi nhận đã tốn nhiều giờ debug trên bàn
thật. Và nó giữ đúng Rule 4: sim kiểm chứng đúng cái sẽ chạy, không phải một
phiên bản gần giống.

**`SimGds68` phải mô phỏng:** máy trạng thái trục IDLE↔CLOSED_LOOP (từ chối
MIT khi chưa CLOSED_LOOP, từ chối CLOSED_LOOP khi chưa "calibrated"); vòng PD
nội bộ ở tốc độ driver dùng joint state không trễ; hộp số 8:1 (khung MIT ở
trục ra, MuJoCo ở trục khớp — kiểm không nhân/chia nhầm 8 lần); feedback
momen ước lượng `Iq × Kt` với đúng hạn chế ở GDS68 §5; heartbeat 0x001 định
kỳ ~100 ms; lượng tử hoá mọi trường theo §5.3 cả hai chiều; trễ bus và mất
khung theo ngân sách §2.

**`SimBl4820` phải mô phỏng:** thanh ghi `0x18`/`0x20`/`0x24`; settle ≥20 ms
sau Torque Enable — lệnh đến sớm hơn bị bỏ qua, đúng hành vi đã verify ở
WHEEL §7 (đây là chỗ sim phải khó tính, không dễ dãi); quirk echo 3 byte đầu
data; checksum, từ chối gói sai; Present Position single-turn 0..1023 có wrap
thật, để buộc test code unwrap phía host; servo vận tốc nội bộ mô hình bậc 1
với tham số đánh dấu rõ là giả định chờ đo ở Phase 6.

Không phát minh interface thứ ba. `MotorFrameCodec` là hàm thuần;
`Transport` là 2 hiện thực thật. Không factory, không registry, không plugin.

---

## 5. Quy ước đơn vị, dấu, lượng tử hoá

### 5.1 Frame và dấu

Trục CAD native là `+Y` cả hai bên (`CLAUDE.md`, `architecture.md` §2) —
không mirror dấu phía phải. Nếu driver phải cần quay ngược để ra cùng chiều
vật lý, chỗ sửa là hiệu chuẩn phía driver hoặc một bảng dấu duy nhất trong
lớp khung, không bao giờ rải rác trong core.

MIT frame trả vị trí/tốc độ/momen ở trục ra (sau hộp số 8:1), khác odrivetool
qua USB (trục rotor, trước hộp số) — `GDS68_DRIVER.md` §4. Nhầm chỗ này là
sai số 8 lần.

Bánh xe: Moving Speed dương = CCW, và CCW của bánh trái/phải là hai hướng đi
tới ngược nhau về cơ khí. Core dùng cùng cực tính, cả hai bánh +forward
(`MpcBalancer.hpp`: `left_wheel_sign = 1.0F`) — bảng dấu bánh nằm trong lớp
khung, đúng một chỗ.

### 5.2 Quy đổi

| Đại lượng | Công thức | Nguồn |
|---|---|---|
| Bánh: rad/s → raw | `raw = rad/s × 9.5493`, clamp ±400 | WHEEL §8 |
| Bánh: position → rad | `raw × 2π/1024`, phải unwrap (single-turn 0..1023) | WHEEL §4 |
| Chân: mọi trường MIT | `raw = (x − x_min)(2^bits − 1)/(x_max − x_min)`, clamp x trước | GDS68 §4 |

Unwrap bánh xe là bắt buộc: odometry bánh là nguồn station-keeping duy nhất
tuân Rule 1. Encoder single-turn 1024 count, ở 400 RPM quay 6.8 vòng/giây —
ở tick 500 Hz là 0.68 vòng/lần đọc, gần Nyquist của wrap; ở 250 Hz đã vượt,
không phân biệt được +1.36 vòng với +0.36 vòng. Ràng buộc cứng: tick bánh
xe ≥ 500 Hz, hoặc dùng trường `Speed` (s16) làm nguồn vận tốc chính — miễn
nhiễm wrap, và chỉ dùng position để hiệu chỉnh chậm. Khuyến nghị dùng `Speed`.

### 5.3 Lượng tử hoá

| Trường | Bit | Dải | LSB | Đánh giá |
|---|---|---|---|---|
| Position | 16 | ±12.5 rad | 0.38 mrad | thừa sức |
| Velocity | 12 | ±65 rad/s | 31.7 mrad/s | đủ |
| Kp | 12 | 0..500 | 0.122 N·m/rad | thừa (dùng 80) |
| Kd | 12 | 0..5 | 1.2e-3 | sát trần, xem dưới |
| T_ff | 12 | ±50 N·m | 24.4 mN·m | thô, xem dưới |

Kd sát trần và đã có một chỗ tràn: sim dùng `kd = 2√(80 × I) × kd_scale` →
hip 2.08, knee 1.04 N·m/(rad/s) (`GeneratedGeometry.hpp:50-51`), trong khi
dải MIT chỉ tới 5.0. `LandingImpedance` đặt `impact_kd_scale = 3.0`
(`native_runner.cpp:686`) → 3.0 × 2.08 = 6.25, vượt dải, bão hoà im lặng.
Phát hiện trên giấy, chưa kiểm chứng bằng chạy — nhưng nằm đúng ở đường
landing đang hỏng.

T_ff thô hơn cần thiết: LSB 24.4 mN·m trên tín hiệu biên độ 2 N·m
(`landing_controller.max_feedforward_torque_nm = 2.0`, `:665`) chỉ cho ~82
mức, đủ dùng nhưng làm mọi feedforward mượt bị bậc thang — cần đưa lượng tử
hoá này vào sim để biết nó có làm hỏng landing không.

### 5.4 Trần vật lý

Chân: GDS68 danh định 5 N·m, stall 11 N·m; sim dùng 3.0 N·m (`:588`), còn
biên. Bánh: BL4820M12 130 mN·m, không hộp số; sim dùng đúng 0.13 N·m
(`:473`, `:1039`), trung thực. Đây là con số khắc nghiệt: 0.13/0.04 = 3.25 N
lực kéo mỗi bánh, 6.5 N cho robot 4.621 kg (45 N trọng lượng) — toàn bộ
authority bắt thăng bằng chỉ khoảng 0.14 g.

Bánh xe 400 RPM có tải → ±400 raw ↔ ±41.888 rad/s ↔ 1.6755 m/s. Sim clamp
đúng ±41.887902 (`:1080`) và mọi đường code khác cũng vậy. Con số 1.2566 m/s
(300 RPM) từng xuất hiện trong ba chỗ prose (`docs/NATIVE_RUNTIME.md`,
`native_landing_contract_test.py`, `native_runner.cpp:462`, đã sửa) — không
dòng code nào từng dùng nó, nhưng nó đang bị dùng để giải thích thất bại
landing: "1.5 m/s trong khi phần cứng chỉ chịu 1.2566". Với 1.6755, lời giải
thích đó không đứng vững: 1.5 m/s nằm trong envelope tốc độ bánh. Ràng buộc
thật là momen — 0.13 N·m × 2 bánh chặn gia tốc mặt đất ở 1.41 m/s²
(`native_runner.cpp:474-477`). Landing đang hỏng vì thiếu gia tốc, không vì
thừa tốc độ.

`WHEEL_MOTOR_PROTOCOL.md` §8 tự ghi "raw = RPM" là suy luận từ trùng số liệu,
chưa đo — cần đo thật (giữ raw cố định, đếm vòng/thời gian) trước khi tin.

### 5.5 Riêng cho seam mức khung

Codec phải là hàm thuần, không I/O, không state — điều kiện để test bit-exact
trên host và để cùng dòng code chạy trên ESP32. Không được có nhánh "chế độ
sim" bên trong codec; nếu xuất hiện `if (sim)` ở đó, toàn bộ giá trị
conformance mất sạch. Test hồi quy so byte, không so struct — struct có thể
đúng trong khi bit-packing sai.

---

## 6. Kế hoạch theo pha

Mỗi pha có cổng bằng chứng: không đạt thì không sang pha sau. Mức bằng
chứng theo `architecture.md` §8.

**Phase 0 — chốt hợp đồng, không code. XONG.** Tick mục tiêu 500 Hz, baud
CAN 1 Mbps, bánh 400 RPM có tải, bảng mapping §1 — tất cả đã chốt.

**Phase 1 — `MotorFrameCodec`, test trên host. XONG.**
`src/tap_control/hal/include/robot_hal/MotorFrameCodec.hpp` (pure,
allocation-free) + `src/tap_control/tests/motor_frame_codec_tests.cpp`.
37/37 checks pass, mọi vector đều bit-exact với ví dụ trong hai file doc
driver (MIT worked example, quickstart 3-khung, 5 khung UART có checksum).
Một phát hiện khi verify: công thức lượng tử hoá của doc **truncate**, không
round-to-nearest (`12.5*65535/25=32767.5` → doc lấy `32767`) - code khớp
đúng theo đó.

**Phase 2 — `SimGds68` + `SimBl4820`. XONG**, gộp chung với Phase 3 do quy
mô nhỏ hơn dự kiến (đủ đơn giản để triển khai và nối luôn trong một lượt).
`src/tap_sim/drivers/SimActuatorBus.hpp`: `SimGds68Joint` (máy trạng thái
IDLE/CLOSED_LOOP, PD nội bộ đúng công thức GDS68 §4, từ chối MIT trước
CLOSED_LOOP) và `SimBl4820Wheel` (Torque Enable/Disable, cửa sổ settle
20 ms có DROP thật, servo vận tốc nội bộ dạng bậc 1).

**Phase 3 — nối vào runner. XONG, có một phát hiện lớn.**
`native_runner.cpp` giờ đóng khung MIT/Dynamixel thật, đưa qua
`SimGds68Joint`/`SimBl4820Wheel`, không còn đường nào đọc `mpcDebugState()`
hay tính PD chân ở host bằng `state_.joints` (encoder trễ). Cả 3 rò rỉ ở §3
đã đóng, xác nhận bằng đọc lại code (không còn `last_mpc_accel_`,
`wheel_command_`, `wheel_state_`). **Nhưng khi nối `MotorCommand.velocity_rad_s`
làm tín hiệu chấp hành DUY NHẤT cho bánh (đúng như thật), balance bắt đầu
ngã** - `neutral_balance`, `stop_balance`, `posture_transition`, `yaw_station`
đỏ mà trước khi vá (cả bằng con đường rò rỉ 1 lẫn native trước đó) đều xanh.
Truy vết bằng telemetry (`mpc_accel` vs `mpc_velocity` vs `wheel_velocity`)
cho thấy đây **không phải bug trong khung/driver giả** - `wheel_velocity` bám
sát `mpc_velocity` rất tốt. Vấn đề nằm ở chỗ `mpc_velocity`
(`commanded_ground_velocity_m_s_`, MpcBalancer.cpp) tự nó bị trần
"station hold" (`station_posture_hold_max_v = 0.25`,
`native_runner.cpp:574`) ghìm lại trong ~150-200ms đầu của một cú pitch thật,
chỉ nhả trần khi `emergency_capture` kích hoạt - lúc đó thường đã trễ. Trần
này **chưa từng bị chạm tới trước đây** vì đường rò rỉ 1 (`last_mpc_accel_`)
lấy thẳng gia tốc QP thô (`optimal_command_m_s`, KHÔNG qua trần này) làm
momen, đi vòng qua toàn bộ logic station-hold. Nói cách khác: gỡ rò rỉ 1
đúng như kế hoạch đã **lộ ra một giới hạn authority có thật trong
`MpcBalancer`, chưa từng được bài test nào chạm tới vì đường thật chưa bao
giờ được dùng.** Đây là phát hiện giá trị nhất của cả đợt triển khai này -
đúng tinh thần "sim sát thật hơn" mà bạn yêu cầu. Sửa nó là việc trong
`MpcBalancer.cpp` (core control logic), ngoài phạm vi lớp HAL - xem §7 và
§9 mục 7 để quyết định hướng đi.
Cổng gốc "ctest không tệ hơn mốc trước khi vá" **chưa đạt** vì phát hiện
trên; xem §7 để biết bức tranh đầy đủ.

**Phase 4 — bật mô hình bus thật trong sim.** Tick 500 Hz, lượng tử hoá
§5.3, trễ bus, mất khung, wrap position bánh. Cổng: chạy lại toàn bộ kịch
bản native — đây là lúc biết balance/landing có sống sót ràng buộc phần
cứng không. Kết quả có thể là "không", và đó là thông tin giá trị nhất của
cả kế hoạch.

**Phase 5 — `HwTransport`, bench một động cơ.** TWAI + 2 UART, cùng codec,
cùng driver logic phía host, chỉ đổi transport. Disable = phát IDLE/Torque
Disable mỗi tick, không im lặng. Gate feedback: chặn enable tới khi cả 6
khớp có feedback thật. Cổng: một động cơ kê bàn, không tải, chưa lắp robot —
đo turnaround UART thật, đo tải bus CAN thật ở 1 Mbps.

**Phase 6 — nghiệm thu mapping vật lý + đối chiếu sim↔thật.** Mapping vật
lý và dấu encoder của 4 khớp CAN đã hoàn thành ngày 2026-09-25 và được đưa
vào `MotorMapping.hpp`. Phần còn lại: cùng chuỗi `MotorCommand` chạy qua
SimTransport và HwTransport, so feedback. Cổng: sai lệch sim↔thật được ghi
nhận và giải thích — đây mới là lúc `SimGds68`/`SimBl4820` trở thành mô hình
có bằng chứng thay vì giả định.

Lắp lên robot, vòng kín thật, nằm ngoài kế hoạch này và cần quyết định riêng.

---

## 7. Trạng thái cây build (đọc lại 2026-09-22, sau khi bạn sửa)

Build sạch từ scratch, ctest: **20/28 pass (71%)**, tăng từ 18/28 lần đọc
trước — `tap_mujoco_native_neutral_balance` và `tap_mujoco_native_dance_skill`
giờ đã xanh.

Vẫn đỏ, và vẫn đúng nguyên nhân đã ghi trước đó:

- `tap_mujoco_native_landing_cmf`, `tap_mujoco_native_multirate_cmf` —
  `assert any(seen_ballistic == "1")` vẫn fail. **CMF vẫn không phát hiện
  được flight** trong kịch bản ledge — đây là test chặn trực tiếp Rule 3.
- `landing_impedance_tests` — vẫn đúng 3 assertion cũ fail (Impact scale
  không engage khi touchdown thật, cả đối xứng lẫn một bên).
- `core_tests`, `deep_squat_tests`, `jump_tests`, `jump_quality_tests` —
  không đổi.

**Mới phát hiện — regression, không phải lỗi cũ:** `core_allocation_tests`
giờ **crash** (`Subprocess aborted`, không phải `Failed`):

```
terminate called after throwing an instance of 'std::invalid_argument'
  what():  Invalid optimized plan or conflicting trajectory owners
```

Ném ra từ `JumpController.cpp:53` — một test trong `core_allocation_tests.cpp`
đang dựng `JumpController::Config` với `plan` set cùng lúc một cờ xung đột
(`landing_compression`/`thrust_wheel_control`/`thrust_hip_offset_rad`...).
Đây là exception không bắt, làm sập cả file test thay vì báo 1 assertion
fail — khác mức nghiêm trọng so với 7 test còn lại chỉ fail bình thường.
Nằm trong stack `JumpController` (đã xác định là code chết, chưa nối vào
`native_runner`), nên không chặn Phase 1–2 của kế hoạch này, nhưng nó chặn
`ctest --test-dir build/native` chạy hết vì binary sập giữa chừng.

**`kd_scale = 3.0` lúc impact vẫn tràn dải MIT** (`native_runner.cpp:742`,
`impact_kd_scale = 3.0F` không đổi) — vẫn là phát hiện trên giấy, chưa kiểm
chứng bằng chạy, vẫn đáng nghi vì nằm đúng đường landing đang hỏng.

**Kết luận không đổi (tại thời điểm đó):** đề nghị xử lý cụm CMF/landing (và
crash mới) trước khi xây HAL lên nền này.

## 7b. Trạng thái sau khi triển khai HAL (Phase 1–3 xong, cùng ngày)

Bất chấp khuyến nghị ở §7, đã triển khai luôn theo yêu cầu trực tiếp của
bạn. ctest sau khi nối khung (trừ `dance_skill`, chạy riêng vì mất 120s):
**17/28 pass (61%)**, giảm so với 20/28 trước khi nối. Đã chạy 2 lần độc
lập (lần đầu bị nhiễu bởi 1 process ctest cũ treo lại, đã kill và chạy lại
sạch — kết quả giống nhau cả hai lần, không phải noise).

So với mốc trước khi nối:

| Test | Trước | Sau | Ghi chú |
|---|---|---|---|
| `native_landing_cmf` | Failed | **Passed** | bất ngờ tốt lên |
| `native_multirate_cmf` | Failed | **Passed** | bất ngờ tốt lên |
| `native_neutral_balance` | Passed | **Failed** | regression, xem dưới |
| `native_runner_lifecycle` | Passed | **Failed** | cùng nguyên nhân |
| `native_stop_balance` | Passed | **Failed** | cùng nguyên nhân |
| `native_yaw_station` | Passed | **Failed** | cùng nguyên nhân |
| `native_posture_transition` | Passed | **Failed** | cùng nguyên nhân |
| `core_allocation_tests`, `core_tests`, `deep_squat_tests`, `jump_tests`, `jump_quality_tests`, `landing_impedance_tests` | Failed | Failed | không đổi, không liên quan HAL |

5 test balance mới đỏ, tất cả cùng một gốc — không phải 5 bug khác nhau.
Truy vết bằng telemetry (`mpc_accel`/`mpc_velocity`/`wheel_velocity` trong
`STATE` line) trên `native_neutral_balance`:

```
t=0.66  pitch=-0.082  mpc_accel=-1.400 (bão hoà)  mpc_velocity=-0.250 (ghìm)  wheel_velocity=-0.308
t=0.80  pitch=-0.161  mpc_accel=-1.400 (vẫn bão hoà)  mpc_velocity=-0.250 (vẫn ghìm)
t=0.82  pitch=-0.195  mpc_velocity nhả trần, bắt đầu chạy theo accel
t=1.12  pitch=-1.120  mode=Fallen
```

`wheel_velocity` bám rất sát `mpc_velocity` (servo bánh xe hoạt động đúng
thiết kế). Vấn đề là `mpc_velocity` — tức `MotorCommand.velocity_rad_s`,
tín hiệu chấp hành DUY NHẤT cho bánh giờ đây — tự nó bị ghìm ở
`station_posture_hold_max_v = 0.25` (`native_runner.cpp:574`,
`MpcBalancer.cpp` khối `station_command_limit`) suốt ~150ms đầu của cú pitch,
dù QP đã bão hoà accel ở mức tối đa ngay từ đầu. Trần chỉ nhả qua nhánh
`emergency_capture` (pitch vượt `fall_pitch_rad/3` hoặc capture-distance
vượt ngưỡng) — và khi nhả thì thường đã trễ để phục hồi.

**Đây không phải bug trong `MotorFrameCodec`/`SimGds68`/`SimBl4820`** — cả
ba đã verify độc lập (37/37 bit-exact, máy trạng thái đúng đặc tả). Đây là
một giới hạn authority có thật, đã tồn tại sẵn trong `MpcBalancer.cpp`, và
**chưa từng bị test nào chạm tới** vì đường rò rỉ 1 cũ (`last_mpc_accel_ =
mpcDebugState().optimal_command_m_s`) lấy thẳng gia tốc QP thô — trường
`optimal_command_m_s` được gán TRƯỚC toàn bộ khối station-hold trong
`MpcBalancer.cpp` (dòng ~936 trở lên), tức đi vòng qua trần này hoàn toàn.
Gỡ rò rỉ 1 đúng như kế hoạch đã làm lộ ra: **cái đường thật (velocity-only)
mà phần cứng bắt buộc phải dùng, MpcBalancer chưa từng được kiểm chứng chạy
tốt trên đó.**

Đây chính là phát hiện "sim sát thật hơn" mà bạn yêu cầu — nhưng cái giá là
lộ ra một khoảng trống thật trong control logic. Sửa nó (nới trần cho
trường hợp pitch-catch khẩn cấp, hoặc để `pitch_capture_priority` — biến
đã tồn tại sẵn ở đó nhưng không gate khối này — gate luôn khối
`station_command_limit`) là việc trong `MpcBalancer.cpp`, core control,
**ngoài phạm vi lớp HAL tôi tự đặt ra và chưa làm** vì đây là logic đã được
tinh chỉnh kỹ lưỡng (nhiều comment ngày tháng, nhiều lớp capture-distance) —
sửa vội có rủi ro gây hỏng theo hướng khác. Xem §9 mục 7.

`kd_scale = 3.0` lúc impact tràn dải MIT (§5.3) — giờ **đã thật sự được thực
thi qua khung** (trước đây chỉ là dự đoán trên giấy). `landing_impedance_tests`
vẫn fail đúng 3 assertion cũ, chưa xác định được có phải do tràn dải này
hay không — cần đo riêng.

---

## 8. Chiến lược test

Thứ tự theo độ rẻ, không theo độ quan trọng:

| Mức | Cái gì | Cần gì |
|---|---|---|
| 1 | Codec bit-exact với vector trong doc | host |
| 2 | Bất biến HAL: disable luôn phát khung, gate feedback, assert `torque_nm` | host |
| 3 | Round-trip: `MotorCommand` → khung → parse → `RobotState` | host |
| 4 | Kịch bản native ở 500 Hz + lượng tử hoá | MuJoCo |
| 5 | Bench một động cơ | phần cứng |
| 6 | Đối chiếu sim ↔ bench | phần cứng |

Vector test đã có sẵn, không cần bịa: MIT — `GDS68_DRIVER.md` §4 "Ví dụ số
cụ thể", node 1, pos/vel/kp/kd/tff giữa dải → `ID=0x028
Data=7F FF 7F F7 FF 7F F7 FF` (chọn giữa dải để ra số tròn, verify được bằng
tay), cộng quickstart 3 khung `0x02B`/`0x027`/`0x028`. UART — 5 khung có
checksum đã verify ở `WHEEL_MOTOR_PROTOCOL.md` §5, gồm cả số âm (`38 FF` =
−200, test đúng little-endian signed).

---

## 9. Câu hỏi còn mở

1. **Rò rỉ gia tốc MPC (§3 mục 1):** core phát vận tốc bánh đã bao hàm ý đồ
   gia tốc, hay `MotorCommand` mang thêm trường gia tốc riêng? Khuyến nghị:
   phương án vận tốc — đúng upstream upkie, không nới interface.
2. **Sửa cây ctest xanh trước, hay làm HAL song song?**
3. Turnaround UART thật — đo ở Phase 5.
4. CAN 1 Mbps có ổn định trên harness thật — đo ở Phase 5.
5. Balance/landing có sống ở 500 Hz — đo ở Phase 4.
6. "raw = RPM" của bánh xe — đo thật, giữ raw cố định đếm vòng/thời gian.

---

## 10. Việc không làm

Không thêm lớp trừu tượng nào ngoài `MotorFrameCodec` + `Transport` (1
interface, 2 hiện thực thật). Không file config runtime cho hằng số không
bao giờ đổi (node_id, baud, bảng dấu) — đó là hằng số biên dịch, đổi phải
qua code review. Không nới trần momen/tốc độ để kịch bản pass — trần là
thông số hãng. Không dùng dữ liệu simulator (pose/contact/qpos) trong bất kỳ
nhánh nào của lớp này — Rule 1 áp dụng nguyên vẹn. Không vận hành phần cứng
ngoài bench Phase 5 nếu chưa có quyết định riêng.
