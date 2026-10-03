# Firmware v3 — con lắc ngược hai bánh, chân khóa tại tư thế cấp nguồn

Build/nạp: `pio run -e esp32-s3-v3 -t upload`. Điều khiển: `python3 tools/tap_serial_control_gui.py`
(tự kết nối, log ở `~/tap_logs/`). Không dùng BLE. v1 (`esp32-s3-poc`) và v2 giữ nguyên.

Profile khởi động của app v3 bật LQR phân tầng, giữ thân bằng hông, cân bằng
roll và hybrid treo gối: hông dùng POS, gối dùng MIT/VMC; nhánh nhún vị trí
giữ dải chậm. `START` trên gá chỉ khóa chân và vào `Ready` ở chiều cao tối đa.
Sau khi đưa robot xuống sàn, giữ yên rồi thả để tự vào cân bằng. Khi bắt đầu
cân bằng, bộ khởi động mới hạ chậm 12 mm để chừa hành trình roll. Các chế độ
WholeBody, VMC thích ứng và feed-forward vận tốc roll vẫn là lựa chọn thử riêng.

Gain VMC gửi tới GDS68 được giới hạn Kp ≤ 12 N·m/rad theo trần trong
`BoardConfig.hpp`, kể cả khung khóa chân đầu tiên. `vstiff` là gain core yêu
cầu; `vkpmax` là trần driver, còn `TR kkp` là gain hiệu dụng. Giới hạn này giữ
nguyên giảm chấn, mô-men bù tải và vận tốc feed-forward. Sim 2D dùng cùng
bộ giới hạn đầu ra khi chạy gối MIT.

Tầng hông nhanh dùng lại cách tách vòng của `b4b6d4a`: nghiêng con lắc ảo
trừ nghiêng theo kế hoạch, high-pass 3 s và lead theo tốc độ con lắc ảo.
Giữ ngang thân dùng vòng IMU chậm riêng (lọc 0.5 Hz, tích phân thích nghi
`max(0.75 × sqrt(l/l_hang), 0.5)` giây).
Bỏ phản hồi nhanh `5 × pitch thân + lead × gyro thân` vì chuyển động thân
ngược chân có thể bị khuếch đại vào lệnh CAN dù đường axle–CoM gần đứng yên.
Đây là khôi phục luật tầng nhanh, không phải checkout toàn bộ commit;
`b4b6d4a` chưa có VMC và dùng POS cho cả bốn khớp.

## v3.3: phối hợp đùi–gối và thích nghi theo chiều cao

### Bản nguồn mới: phối hợp bánh–chân khi bị đẩy (02/10)

LQR vẫn cân bằng theo đường trục bánh–CoM, còn chân giữ/giảm rung thân.
Vận tốc lăn theo trục CAD +Y là
`v_axle/R = wheel_relative + body_gyro + hip_rate + knee_rate`.
Nhánh đo đã có phép cộng này; nhánh lệnh mới bù ngược từng bên với gain
0.10, giới hạn tổng tốc độ carrier ±3 rad/s. Phần bù tối đa 0.3 rad/s,
tương đương 12 mm/s, được áp dụng trước deadband của bánh. Bù toàn phần
đã làm xấu mô phỏng có trễ nên không dùng mặc định.

Thêm giảm chấn hông theo gyro thân: `0.04*sqrt(r)*gyro/J_hip`, giới hạn
±0.04 rad, chỉ mở dần khi có nhiễu con lắc ảo. Thân và chân quay ngược
nhau mà đường trục–CoM đứng yên không kích hoạt vòng gyro này. Giữ luật
lead, HPF, level và giới hạn hành trình đã có. Đây là bù qua lệnh vị trí,
không phải bộ điều khiển mô-men toàn thân.

Khi đang nâng/hạ theo kế hoạch, hai phần bù mới giảm về 0 trong 100 ms;
sau khi kế hoạch yên 1 s, tăng lại trong 500 ms. LQR, IK cặp hông–gối,
roll và hybrid VMC vẫn hoạt động. Không thêm thao tác SET/GUI.
IK chiều cao dùng nghiệm giải tích theo CAD thay vòng lặp FK/COM; chưa
đo lại thời gian vòng điều khiển trên ESP32.

**Bản này đã build và nạp robot theo yêu cầu ngày 02/10.** Kết quả và
giới hạn mô phỏng ghi ở mục 5.31, artifact đã nạp và trạng thái sau
nạp ghi ở mục 5.32 trong `docs/V3.3_HANDOFF.md`.

Luồng LQR dùng IK Cartesian cho **cả hông và gối**. Đặt trục bánh trên
một mặt phẳng ngang theo trọng lực, rồi giải hai góc theo CAD hiện tại.
Tọa độ trước/sau vẫn được lấy từ FK của luật hông/bù gối cũ để giữ hiệu
chuẩn tầng điều khiển; nghiệm cuối ưu tiên chiều cao và giới hạn X theo
hành trình gối. Không để hông tiếp tục vẽ cung tròn khi gối đã chạm giới
hạn duỗi. Tích phân level bị chặn theo chiều hết hành trình, nhưng được
nhả khi sai số đảo chiều. Lệnh VMC gối dùng góc mới cộng bù roll.

Mặt phẳng mục tiêu có độ sâu `max(h_pose*cos(pitch_body), h_min)` so với
hông. Đây là bù hụt chiều cao do chuyển động tầng chân; không phải vòng
đo độ cao tuyệt đối của thân. Không có cảm biến độ cao mới. LQR bánh vẫn
cân bằng theo con lắc ảo. START/Ready giữ đúng tư thế cấp nguồn; sau khi
vào Balancing mới chừa 12 mm bằng profile khởi động cũ. Khoảng chừa này
cũng cần khi chỉ bật trunk và tắt roll. Chân duỗi hết không có hành trình
trước/sau tại cùng chiều cao; tầng bánh xử lý giai đoạn bắt cân bằng đó.

Schedule dùng chiều cao CoM tính từ encoder/FK, lọc 50 ms, với
`r=clamp(l_measured/l_hang, 0.1, 1)`. Lead mặc định `0.06*sqrt(r)` s,
giới hạn tốc độ yêu cầu trunk `3*sqrt(r)` rad/s, level tau như trên.
Biên độ dynamic không vượt `tmax` hoặc hành trình ngang khả dụng; nghiệm
IK cuối vẫn kiểm giới hạn cả lệnh dynamic và level. Không giảm mặc định
share giữ thân chỉ vì hạ chiều cao. LQR gain table, driver signs và trần
VMC Kp 12 được giữ. WholeBody là nhánh thử riêng, chưa dùng IK mới này.

| Hông–trục bánh đo được | r | Lead | Tốc độ yêu cầu trunk | Level tau |
| --- | --- | --- | --- | --- |
| 176.6 mm | 0.937 | 58.1 ms | 2.903 rad/s | 0.726 s |
| 110 mm | 0.584 | 45.9 ms | 2.293 rad/s | 0.573 s |
| 84 mm | 0.446 | 40.1 ms | 2.005 rad/s | 0.501 s |

Realtime: vòng tính giữ 500 Hz; bốn khớp gửi cùng batch khoảng 333 Hz
(deadline 3 ms trên tick 2 ms, khoảng cách 2/4 ms, không gửi bù dồn sau
stall). Lệnh bánh được gửi trước việc đóng khung bốn khớp. Tải CAN ước
tính 62–65% tại 500 kbit/s, tính cả hai reply MIT gối và bốn encoder
100 Hz; chưa phải đo trên bus với firmware mới. Encoder vẫn khoảng
96 Hz, không ngoại suy vị trí khớp để tạo phản hồi giả. Trace thêm
`swing` (yêu cầu), `dhip` (hông thực sự được lệnh), `axlim`, `adpr`,
`adpl`, `adpv`, `adpt`; GET có `axle_path=paired height_adapt=encoder`.

Build/test và giới hạn mô phỏng xem handoff mục 5.29–5.30. Bản phối hợp
đùi–gối/adaptive/realtime kèm sửa Ready đã nạp ngày 02/10 theo yêu cầu
người vận hành; SHA256 và kiểm tra sau nạp ở mục 5.30.

## Ý tưởng

- Người vận hành xếp chân (duỗi tự nhiên theo trọng lực) **trước khi cấp nguồn motor**.
  Encoder mỗi GDS68 bằng 0 tại lúc cấp nguồn, nên tư thế đó chính là mốc 0 — không offset,
  không mô hình, không homing.
- Hông được khóa tại encoder = 0 bằng **chế độ vị trí của driver**
  (`Set_Controller_Mode(position, passthrough)` + `Set_Input_Pos`). Profile app bật
  VMC cho hai gối: driver chạy MIT spring-damper khi khóa chân; khi gá/đang giữ,
  lệnh feed-forward trọng lực và nhún mềm được tắt.
- Robot còn lại là con lắc ngược hai bánh: `LqrBalancer` với chiều dài con lắc cố định cho
  tư thế chân duỗi, góc cân bằng tự động (seed = pitch lúc thả + tích phân chậm), chống
  tích lũy lệnh, trần tốc độ 0,8 m/s.

## Trình tự (một nút START, robot đặt trên giá)

1. Xếp chân, bật nguồn motor, đặt thân robot lên giá (bánh không chạm đất).
2. **START**: kiểm tra IMU/bánh/chân, mỗi khớp cách mốc 0 không quá 0,35 rad.
3. **Locking**: chân vào closed loop và đi êm về 0 trong 1 s, chờ tổng 1,2 s.
4. **Ready**: bánh bật mô-men nhưng giữ tốc độ 0, chưa chạy vòng cân bằng (không tự
   chạy vọt trên giá). `START` không engage ngay (`imm=0`). Người vận hành bưng robot xuống sàn, giữ yên ở điểm đứng tốt nhất
   đủ 0,7 s thời gian yên (tự "armed", góc giữ đó thành seed). Rung ngắn chỉ
   mất thời gian của đoạn rung, không xóa toàn bộ phần đã giữ yên; thông thường
   cần khoảng 0,7–0,8 s sau khi nhận biết thao tác bưng. Rung kéo dài, xoay thân
   hoặc gián đoạn dữ liệu vẫn bắt đầu lại bước chờ. Sau ARMED, buông: tốc độ
   nghiêng > 0,12 rad/s liên tục 20 ms hoặc lệch > 0,02 rad thì vào Balancing.
   Chuỗi này xét tốc độ thân từ IMU đã bù bias, tránh vận tốc encoder khớp
   bị nhiễu làm kẹt ở bước chờ yên khi chân duỗi thẳng. Góc seed vẫn lấy
   theo con lắc ảo. `GO` giữ luồng tự arm, không báo ARMED trước khi giữ yên đủ.
   `GO NOW` engage ngay; `SET auto 0` tắt tự động.
   Bước nhận biết bưng vẫn cần tốc độ nghiêng > 0,40 rad/s hoặc thay đổi
   góc con lắc ảo > 0,08 rad so với lúc vào Ready. Di chuyển hoàn toàn thẳng
   đứng, không có tín hiệu nghiêng đủ lớn, có thể vẫn ở bước này; firmware
   không có cảm biến tiếp xúc sàn để tự xác nhận đã đặt xuống.
5. **Balancing**: LQR dùng pitch con lắc ảo; hông giữ thân ngang/đưa thân theo gia tốc,
   gối dùng hybrid VMC + bù nhún dải chậm.
6. DISABLE / ESTOP / mất lệnh serial 0,5 s / nghiêng quá 0,6 rad / bánh > 15 rad/s trong
   0,5 s / mất cảm biến → dừng bánh (chân vẫn khóa tới DISABLE).

## PID dự phòng (`SET algo 1`)

- Vòng trong: `a = kp·(θ − ref) + kd·θ̇`, tích phân thành tốc độ bánh (±8 m/s², ±0,8 m/s,
  lệnh không vượt tốc độ đo quá 0,3 m/s).
- Vòng ngoài: `u = kv·(v − v_lệnh) + kx·(x − x_đích)`, `ref = θ_b − clamp(u, ±0,2)`.
- Góc cân bằng động: `θ_b −= kix·(x − x_đích)·dt` (dừng khi `u` bão hòa hoặc đang DRIVE),
  kẹp seed ± 0,2 rad. Hội tụ về góc cân bằng thật trong ~10–15 s, robot quay về chỗ thả.
- Bù trễ bánh: lệnh gửi = tốc độ tích phân + a·lead (lead 0,12 s), vì driver là servo tốc độ
  trễ 0,1–0,15 s.
- Bộ tham số PID dự phòng: kp 30, kd 4, lead 0,12, kv 0,1, kx 0,1, kix 0,02. Quét bền vững trên mô phỏng
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

Chiều cao chân là khoảng cách hip–axle: `SET height <mm>`, `SET hrate <mm/s>`,
`SET haccel <mm/s²>`, `SET hjerk <mm/s³>`. Mặc định chuyển cao dùng 25 mm/s,
70 mm/s² và 500 mm/s³; planner giới hạn giật và tự hãm trước đích.

Đo đạc (một bộ ghi RAM 250 Hz, ≤ 8 s, đổ ra thành dòng `RC` vào log sau khi xong;
phân tích bằng `tools/fit_leg_servo.py`):
`HIPTEST [amp] [resend_ms]` / `KNEETEST [amp]` (trên giá, bánh tắt: bước + sin 1/2/4/8 Hz
trên hông / gối), `KNEETEST FLOOR [amp]` (đang cân bằng trên sàn, ≥ 120 mm, cần vpitch:
gối ±amp rad, mặc định 0.03, ở 0.5/1/2/3 Hz qua đường tư thế chiều cao; tự hủy nếu θv lệch
trim > 0.08 rad), `REC [s]` (chỉ ghi, mặc định 6 s: đẩy, qua gờ, ôm cua).

v3.3 (điều khiển phân tầng, chỉ LQR, mặc định bật; xem `docs/V3.3_HANDOFF.md` mục 5):
`SET vpitch 0|1` (con lắc ảo, chỉ khi không Balancing), `SET trunk 0|1` (hông giữ thân),
`SET tshare|tlead|tmax|trate|texp|texpt v` (`texpt`: số mũ phần hông gánh ở độ cao lớn, `texp`: ở độ cao thấp; mặc định 0/0 = hông gánh toàn bộ), `SET tleadexp v` (lead tầng 2 = `tlead`·ratio^v, mặc định 0.5), `SET tplanfull 0|1` (nghiêng theo kế hoạch đủ 100 %, mặc định 1), `SET vmc 0|1` (VMC gối: gối chạy MIT lò xo–giảm chấn + bù trọng lực + sky-hook; chỉ khi DISABLE; app v3 hiện bật lúc boot), `SET vfreq|vzeta|vsky|vki|vgrav|vstiff|vtmax v` (tham số VMC, xem HANDOFF 5.17), `SET vadapt|vprog 0|1`, `SET vrlo|vrhi` (mm/s), `SET vplo|vphi` (mm), `SET vpratio` (thích ứng thử nghiệm, tắt mặc định, HANDOFF 5.18), `SET leash <m>` (mốc vị trí khi chạy cách robot tối đa, 0.10), `SET sleash <m>` (xích neo khi đứng: bánh vượt 0.7 trần thì neo LQR nhường theo robot trong khoảng này tới khi dừng, rồi đi về trạm; 0 = tắt, mặc định tắt, đề nghị thử 0.10, HANDOFF 5.33), `SET sret <m/s>` (tốc độ neo đi về trạm, 0.10), `SET tdb <rad>` (vùng chết phản hồi nhanh của hông, 0 = tắt, đề nghị thử 0.003), `SET tsatm v` (`Tier::sat_margin`, mặc định 0.6, đề nghị thử 0.8), `SET cdx <mm>` (lệch CoM thân), `SET timu 0|1`
(bù động học IMU), `SET tvg <0..1>` (trọng số vận tốc khớp trong θv_rate), `SET tprev 0|1`
(dự báo thao tác, cần vpitch), `SET tpacc|tpjerk|tpvmax|tpt v` (gia tốc/giật/trần tốc độ/
tầm dự báo của quỹ đạo), `SET tlvl 0|1` (giữ thân ngang, cần trunk), `SET tlevel <rad>`
(pitch thân mục tiêu, mặc định 0), `SET tltau|tlvmax v`, `SET hcap <m/s>` (trần tốc độ bánh
khi đang bị giữ/nhấc), `SET tsus 0|1` (gối nhún giữ độ cao thân, cần vpitch), `SET tsusg v`
(độ lợi), `SET tsusmax <mm>`; `SET algo 2` (LQR toàn thân, thử nghiệm; chỉ vào cân bằng qua chuỗi
ARMED: đặt xuống sàn, giữ yên 0.7 s, buông - `SET wbarm 0` bỏ ràng buộc này; lệnh bánh đập trần
qua lại 4 lần trong 2 s thì FAULT "whole-body loop ringing"), `SET wbgive <0.3..1>` (trên tỉ lệ
này của trần tốc độ bánh, mốc vị trí lùi theo robot; 1 = tắt), `SET wbenv 0|1` (giới hạn chuyển
động theo chiều cao khi WB: trần tốc độ, giật, gia tốc ≈ 1 m/s²), `SET tenv 0|1` (như vậy cho tầng
1/2, mặc định tắt), `SET tenvlean <rad>` (độ nghiêng chân nhận cho gia tốc kế hoạch), `SET wbkhold 0|1` / `SET khold 0|1`
(gối giữ chiều cao hông–trục khi hông vung, IK; mặc định bật cho cả WB và tầng 1/2), `SET wbsoft <0.2..1>`
(WB: từ tỉ lệ này của trần tốc độ bánh, gain chuyển dần sang bảng "mềm" để thân phụ đỡ; 1 = luôn
bảng cứng), `SET roll 0|1` (trục roll: thân ngang trái/phải, nghiêng vào cua; app v3 hiện bật lúc boot),
`SET rolltau|rollkp|rollff|rolltgt|rollyaw v`, `SET rollmax|rollres <mm>`, `SET rollsign 1|-1`, `SET rollvff 0|1` (tốc độ phần roll gửi kèm
`Set_Input_Pos` làm vel_ff cho GDS68 + gain roll nhanh); TR thêm `roll`
(rad, + = trái cao) và `rdh` (mm, chân trái +, phải −), `SET wbff|wbslack|wbbg v`,
`SET imm 0|1` (0: mọi thuật toán chờ ARMED như trên thay vì cân bằng ngay khi khóa chân; app v3 boot với 0). TR thêm
`tv` (θv), `dhip` (hông tầng 2), `ta` (quyền 0..1), `me` (sai lệch mô hình chân), `lvl`
(phần hông giữ thân ngang), `ho` (1 = đang bị giữ, bánh bị giới hạn), `lt` (trim đã học/hợp
lệ), `sus` (offset nhún, mm), `vz` (vận tốc dọc thân ước lượng); SENS thêm `hipv_rms` (nhiễu vận tốc hông), `prof=[update legs wheels trace]` (thời gian lớn
nhất từng phần của tick điều khiển trong cửa sổ báo cáo, µs) và `upd_avg` (trung bình
`V3Controller::update`, µs). Dòng TR/SENS dựng bằng `tap_fw/FastFormat.hpp` (cùng văn bản như
printf cũ, không tốn chi phí đổi số thực của printf).

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


## Prototype split CoM/gimbal — chỉ host, chưa qua cổng (03/10)

CMake host có `Config::Split`, mặc định tắt, qua
`TAP_SPLIT_HOST_PROTOTYPE`. PlatformIO không định nghĩa cờ này nên không
đưa prototype vào controller ESP32. App chưa có `SET tsplit`, TUNE split
hay TR hA/hB/aB; các trường này mới có ở CSV host. Không nạp firmware.

A đã được thử: giữ mục tiêu CoM/gimbal chậm, có IK và giới hạn tốc độ,
không thay LQR bánh. Cổng A thất bại ở plant hông mềm + trễ bánh, nên dừng
trước B và tích hợp firmware. Không bật mặc định hoặc dùng bản host làm
chứng nhận robot thật. Xem `docs/TSPLIT_IMPLEMENTATION_20261003.md`.
