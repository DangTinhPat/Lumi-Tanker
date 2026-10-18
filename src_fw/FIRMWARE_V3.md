# Firmware v3.5.2 — POS, con lắc ngược hai bánh với điều khiển phân tầng

`v3.5.2` tách từ `v3.5` tại `c757ff1` (`base 17:58`), đổi bốn khớp chân sang POS
để không cần Kt/Iq ở phía firmware. Chi tiết cấu hình và mức bằng chứng:
[`V3.5.2_POS.md`](../docs/V3.5.2_POS.md). Bản này giữ hình học GIM6010-8
từ `ROBOT (GIM6010-8).STEP`. Khối lượng, COM và mô-men quán tính là ước lượng
theo mô hình khối lượng đã gán; xem `docs/GIM_SIM_PHYSICS_SPEC.md`.

Mô hình khớp gối lấy 22° làm mốc 0 và giới hạn trên 135° (1.9722 rad); đùi dùng
trục CAD GIM. Gốc encoder của driver so với mốc CAD này chưa được xác nhận trên
robot thật, nên thông số hình học là đầu vào mô phỏng, chưa phải hiệu chuẩn phần cứng.

Tài liệu tham chiếu cho app `esp32-s3-v3` (`firmware_esp32/src/AppV3.cpp`, lõi `src_fw/src/v3/`). Trạng thái dự án, cấu hình
boot, số đo phần cứng và các hướng đã loại nằm ở `docs/V3.3_HANDOFF.md`; nhật ký cũ ở
`docs/archive/V3.3_HANDOFF_HISTORY_20261005.md`.

Build/nạp: `pio run -e esp32-s3-v3 [-t upload --upload-port /dev/ttyUSBx]`. Điều khiển: `python3 tools/tap_serial_control_gui.py`
(tự kết nối, log ở `~/tap_logs/`; mở cổng serial reset ESP32 và mất mọi `SET` chạy). Không dùng BLE khi điều khiển
bằng GUI (Dabble gamepad vẫn có). v1 (`esp32-s3-poc`) và v2 (`impulse-v2`) đã gỡ ngày 08/10/2026 (mã, test, tài liệu); `esp32-s3-v3` là môi trường mặc định và duy nhất, còn lịch sử trong git.

## Baseline UART bánh mới

Nhánh v3.5 (bao gồm v3.5.1) dùng mặc định hai bus half-duplex một dây:
trái GPIO15/UART2/ID2/sign +1; phải GPIO17/UART1/ID1/sign -1.
GPIO16/18 để input floating; TX nhả DATA ngay khi hardware idle.
Chi tiết cơ chế, log beta và giới hạn xác minh của port v3.5:
[`WHEEL_UART_BASELINE_20261013.md`](../docs/WHEEL_UART_BASELINE_20261013.md).

## Tổng quan

- **Cả bốn khớp chân chạy POS của GDS68** (`Set_Controller_Mode(position=3, passthrough=1)` +
  `Set_Input_Pos` / `0x00C`, khoảng 333 Hz). Góc gửi bằng vòng rotor, tỉ số truyền 8:1 và chiều driver giữ nguyên.
  Driver tự đóng vòng vị trí/tốc độ; torque feed-forward luôn bằng 0, không dùng Kt hoặc bù chênh lệch Iq.
  Mỗi lần engage gửi giới hạn `8 A` và `20 rev/s` trước closed loop. Không cần phát `Get_Iq` để START/cân bằng;
  vẫn bắt buộc encoder `0x009`, heartbeat, IMU và encoder bánh còn mới.
- Người vận hành xếp chân (duỗi tự nhiên theo trọng lực) **trước khi cấp nguồn motor**; encoder mỗi GDS68 bằng 0 tại lúc cấp
  nguồn nên tư thế đó là mốc 0. Chiều cao, tầng 2 và roll cộng offset lên mốc này (`LegLock::setOffsets/setSideOffsets`).
- Tầng 1 (bánh): `LqrBalancer` trên **góc con lắc ảo** θv = pitch + góc đường trục bánh–CoM đo từ hông/gối − góc ở tư thế treo.
  Tầng 2 (hông, `dhip`): phản xạ + preview + vòng level; IK cặp hông/gối giữ trục bánh trên mặt phẳng ngang. Gối: profile chiều
  cao, suspension vị trí, knee hold. Kiến trúc đầy đủ ở HANDOFF mục 2.
- Chiều cao là khoảng cách hông–trục bánh, 188.6 mm (treo) → 78 mm. `START` vào `Ready` ở chiều cao tối đa; khi cân bằng, bộ
  khởi động hạ chậm một đoạn để chừa hành trình knee cho tầng 2 (12 mm khi roll bật), rồi chiều cao yêu cầu được áp dụng.
- Bộ ước lượng khi Balancing (`balancingEstimatorConfig`): hiệu chỉnh gia tốc kế 0.2 Hz, tắt nhánh Kalman nhanh, tắt học bias gyro
  lúc yên. Khi bánh đẩy trên sàn gia tốc kế chỉ thấy ~17 % độ nghiêng thật (IMU gần tâm va chạm), cấu hình gốc (55 /s) làm vòng
  mù và ngã (`v3_physics_tests` T1). Khi cân bằng tự do, controller đưa vận tốc động học của điểm IMU vào bộ ước lượng
  (`Tier::imu_kinematic`) để tăng tốc kéo dài không làm lệch pitch ước lượng.

## Trình tự (một nút START, robot đặt trên giá)

1. Xếp chân, bật nguồn motor, đặt thân robot lên giá (bánh không chạm đất).
2. **START**: kiểm tra IMU/bánh/chân, mỗi khớp cách mốc 0 không quá 0.35 rad.
3. **Locking**: chân vào closed loop và đi êm về 0 trong 1 s, chờ tổng 1.2 s.
4. **Ready**: bánh bật mô-men nhưng giữ tốc độ 0, chưa chạy vòng cân bằng. `START` không engage ngay (firmware boot `imm=0`).
   Người vận hành đặt robot xuống sàn và giữ yên ở điểm đứng tốt nhất: yên 0.5 s (dải pitch 0.04 rad) thì **ARMED**, góc giữ đó
   thành seed. Rung ngắn chỉ mất thời gian của đoạn rung; rung kéo dài, xoay thân hoặc gián đoạn dữ liệu bắt đầu lại bước chờ.
   Sau ARMED, vào Balancing khi nghiêng nhanh (> 0.12 rad/s liên tục 20 ms, hoặc lệch > 0.02 rad) hoặc sau 0.5 s (`armgo`).
   Chuỗi này xét tốc độ thân từ IMU đã bù bias, không dùng vận tốc encoder khớp (nhiễu làm kẹt ở bước chờ yên khi chân duỗi
   thẳng). Bước nhận biết bưng cần tốc độ nghiêng > 0.40 rad/s hoặc góc con lắc ảo đổi > 0.08 rad so với lúc vào Ready;
   firmware không có cảm biến tiếp xúc sàn. `GO NOW` engage ngay; `SET auto 0` tắt tự động; `SET imm 1` cân bằng ngay sau khi khóa chân.
5. **Balancing**: LQR trên θv; hông giữ thân ngang và theo gia tốc; gối theo profile chiều cao và suspension.
6. DISABLE / ESTOP / mất lệnh serial 0.5 s / nghiêng quá 0.6 rad (θv hoặc thân) / bánh > 15 rad/s trong 0.5 s mà thân không
   ngã / mất cảm biến / θv và chân đo lệch nhau > 0.12 rad trong 0.3 s → dừng bánh (chân vẫn khóa tới DISABLE).
   Bánh quay nhanh mà thân không quay (bị giữ/nhấc) thì bộ phát hiện "bị giữ" re-zero thay vì FAULT.

## PID dự phòng (`SET algo 1`)

- Vòng trong: `a = kp·(θ − ref) + kd·θ̇`, tích phân thành tốc độ bánh (±8 m/s², ±0.8 m/s, lệnh không vượt tốc độ đo quá 0.3 m/s).
- Vòng ngoài: `u = kv·(v − v_lệnh) + kx·(x − x_đích)`, `ref = θ_b − clamp(u, ±0.2)`.
- Góc cân bằng động: `θ_b −= kix·(x − x_đích)·dt` (dừng khi `u` bão hòa hoặc đang DRIVE), kẹp seed ± 0.2 rad.
- Bù trễ bánh: lệnh gửi = tốc độ tích phân + a·lead (lead 0.12 s).
- Tham số: kp 30, kd 4, lead 0.12, kv 0.1, kx 0.1, kix 0.02. Quét bền vững trên sim vật lý (khối lượng ±30 %, quán tính −30/+40 %,
  CoM ±20 %, trễ bánh 0.05–0.2 s, trễ lệnh 4–40 ms): 318/324 đứng; không có lead thì chỉ 53/108.
- Tầng 1/2 (θv, hông) chỉ hoạt động với LQR; PID không dùng.

## Lệnh console

`START`, `GO`, `GO NOW`, `TILTTEST`, `BALANCE`, `BALANCE CLEAR`, `DISABLE`, `ESTOP`, `CLEAR_ESTOP`, `STOP`, `DRIVE f y`, `HMOVE ±1`,
`GET` (in các dòng `V3 TUNE …`), `SET <khóa> <giá trị>`.

`SET` theo nhóm (khóa chính xác nằm trong `handleSet` của `AppV3.cpp`; `GET` in giá trị đang dùng):
- **Bánh/LQR:** `algo` (0 LQR+tầng, 1 PID, 2 whole-body; chỉ khi Idle), `accel`, `amax`, `trim_gain|trim_max|trim|margin|pitch_scale`,
  `lqrpredict 0|1`, `lqrdelay|lqrlag <s>` (dự đoán trễ lệnh, mặc định tắt; GIM sim bật),
  `balance` (lưu góc cân bằng), `dbc|dbe` (bù vùng chết bánh), `lqrlead <s>` (ghim lead cố định, tắt thích ứng), `lqradapt 0|1`,
  `lqrleadtall|lqrleadlow`, `ttrimacc 0|1`, `ttrimtau`, `leash` (neo vị trí khi lái), `sleash|sret` (neo đứng, mặc định tắt),
  `hcap` (trần tốc độ bánh khi bị giữ), `wpred|wbpred|wpdelay|wptau|wpmeas` (bộ dự đoán bánh).
- **Tầng 2:** `vpitch` (chỉ khi không Balancing), `trunk`, `tshare|tlead|tleadexp|tmax|trate`, `texp|texpt` (số mũ phần hông gánh ở
  độ cao thấp/cao), `tvg|tvglow` (trọng số tốc độ khớp trong θv_rate), `tbdamp` (giảm chấn thân), `tbdlpf` (lọc gyro cho giảm chấn, Hz, boot 4,5; 0 = gyro thô), `tcarrier`, `tadapt`,
  `legacy 0|1` (đưa tầng 2 về như `b4b6d4a`), `tlsrc 0|1` (lead lấy gyro), `tgate|tglo|tghi|tgllo|tglhi|tgrel` (cổng sự kiện phản xạ),
  `tginertia 0|1` (khi tgate bật: mở theo gyro/gia tốc, không mở riêng vì lean tĩnh; mặc định tắt),
  `tdb` (vùng chết), `tsatm`, `tprev|tpacc|tpjerk|tpvmax|tpt` (preview), `tlvl|tlevel|tltau|tlvmax` (vòng level), `tplanfull`,
  `tgov 0|1` (mặc định 0: chừa mô-men cân bằng trong kế hoạch gia tốc), `tgreserve` (phần mô-men khả dụng chừa lại, mặc định 0.45),
  `tgtorque <Nm>` (mô-men liên tục mỗi bánh, mặc định 0.13; không dùng peak đứng dậy). Bản SIM bật `tgov`, lead hông 0 s,
  kế hoạch 0.9 m/s², jerk 10 m/s³; đây là tuning cho mô hình mô-men hiện tại, chưa phải tuning xác nhận trên robot. Xem `docs/DYNAMIC_BALANCE.md`.
  `tenv|tenvlean`, `khold|wbkhold`, `kfhz|kfrate` (gối bù độ cao của IK qua lọc Hz + giới hạn rad/s, boot 1.0/0.6, `kfhz 0` = tức thì), `cdx <mm>` (lệch CoM thân), `timu`.
- **Chiều cao/gối:** `height <mm>`, `hrate <mm/s>`, `haccel <mm/s²>`, `hjerk <mm/s³>`, `hff`, `tsus 0|1`, `tsusad 0|1`, `tsusk`,
  `tsusdelay`, `tsusg`, `tsusmax <mm>`.
- **Roll (trục ngang của gimbal):** `roll 0|1` (boot 0), `rolltau|rollkp|rollff|rollyaw|rollmax|rollres|rollsign|rollvff`,
  `rolltgt` (ghim mốc ngang, tắt học), `rollcal` (học lại mốc), `rolllearn|rollpre|rolldamp|rolllatch|rollgimbal`. TR: `rref`, `rm`.
- **Khởi động:** `imm`, `auto`, `armgo`, `stillms`, `stillspan`.
- **Whole-body (thử nghiệm):** `wbarm|wbgive|wbsoft|wbenv|wbff|wbslack|wbbg`.
- POS dùng gain vị trí/tốc độ bên trong GDS68 (`pos_gain`, `vel_gain`, `vel_integrator_gain`).
  `SET leg_kp|leg_kd|iqcomp` bị từ chối rõ ràng trong POS; chúng chỉ dành cho đường torque cũ được chọn qua cấu hình mã nguồn.
  Profile gain mô-men khi nhảy không áp dụng trong POS; nhảy vẫn tắt khi boot và chưa được xác nhận cho bản này.

Chiều cao boot dùng 15 mm/s, 50 mm/s², jerk 200 mm/s³ (mặc định của struct: 25 / 70 / 500); planner giới hạn giật và tự hãm
trước đích.

Đo đạc (một bộ ghi RAM 250 Hz, ≤ 8 s, đổ ra thành dòng `RC` vào log sau khi xong; phân tích bằng `tools/fit_leg_servo.py`):
`HIPTEST [amp] [resend_ms]` / `KNEETEST [amp]` (trên giá, bánh tắt: bước + sin 1/2/4/8 Hz trên hông / gối), `KNEETEST FLOOR [amp]`
(đang cân bằng trên sàn, ≥ 120 mm, cần vpitch: gối ±amp rad ở 0.5/1/2/3 Hz; tự hủy nếu θv lệch trim > 0.08 rad), `REC [s]`
(chỉ ghi, mặc định 6 s).

## Log

- `CONTROL mode=Idle|Locking|Ready|Balancing|Fault` khi đổi trạng thái; `RELEASE pitch=… seeded_trim=…`.
- `SENS` 5 Hz: `can_legs`, `uart_wheels`, `homed`, `joint_q`, pitch, gyro, gia tốc thân/chip, `vbus_min`, `hipv_rms` (nhiễu vận tốc hông),
  `prof=[update legs wheels trace]` (thời gian lớn nhất từng phần của tick, µs), `upd_avg`.
- `TR` 25 Hz khi không Idle: pitch, lệnh/đo bánh, vận tốc/vị trí trục, trim, góc 4 khớp, và `tv` (θv), `dhip` (hông tầng 2), `ta` (quyền
  0..1), `me` (sai lệch mô hình chân), `lvl` (phần hông giữ thân ngang), `ho` (1 = đang bị giữ), `lt` (trim đã học), `sus` (offset nhún, mm),
  `sdb` (vùng chết suspension, mm/s), `vz` (vận tốc dọc thân ước lượng), `rg` (cổng phản xạ), `ql` (lead bánh đang dùng), `roll`, `rdh`.
- `V3 …`: kết quả lệnh, lý do từ chối, lỗi. Dòng TR/SENS dựng bằng `tap_fw/FastFormat.hpp` (cùng văn bản như printf, không tốn chi phí đổi số thực).

Realtime: vòng tính 500 Hz trên task FreeRTOS lõi 1 (serial/BLE ở lõi 0; `-DTAP_V3_SINGLE_LOOP=1` giữ vòng đơn). Bốn khớp chân gửi cùng
batch ~333 Hz (deadline 3 ms trên tick 2 ms); lệnh bánh gửi trước việc đóng khung bốn khớp. Encoder khớp ~96 Hz, không ngoại suy.

## Kiểm chứng (test host)

Các kết quả mô tả dưới đây là bằng chứng lịch sử của các model tương ứng,
không phải lần chạy xác nhận bản POS v3.5.2. Mức bằng chứng của bản này nằm
trong [`V3.5.2_POS.md`](../docs/V3.5.2_POS.md).

Build: `cmake -S . -B <dir> -DBUILD_TESTING=ON -DTAP_BUILD_SIMULATOR=OFF`.
- `v3_physics_tests`: con lắc bánh-thân (Lagrange, khối lượng CAD), servo bánh trễ 80/120 ms, lực đặc trưng tại IMU qua đúng
  `ImuStateEstimator`: cấu hình gốc ngã; cấu hình Balancing đứng vững với góc cân bằng ±0.1 rad, seed lệch ±2°/±4.6°, bias trôi, đẩy 1 N,
  dốc 0.03 rad; θ_b về đúng góc thật (< 0.001 rad), robot về chỗ thả (< 3 mm).
- `v3_tests`: POS frame mặc định, torque/current frame khi chọn cấu hình cũ, giới hạn GDS68, ramp về 0, resend, IDLE; trình tự Idle→Locking→Balancing/Fault, cân bằng vòng
  kín trên mô hình con lắc bánh-trên-chân cứng (servo bánh 40 ms, nhiễu cảm biến, điểm cân bằng lệch, tay lệch ±2°).
- `v3_tier_tests`: sim hai vật thể (thân + chân nối bằng servo vị trí hông fit từ `HIPTEST`), chạy từng bài theo biến môi trường
  `TIER_*` (xem HANDOFF mục 7). Mặc định dừng ở assertion V14 đã có từ trước.
- Lỗi có từ trước ở ctest: `core_allocation_tests`, `jump_tests`, `jump_quality_tests`, `lqr_tests`, `v3_tier_tests`.
Mọi kết quả chỉ là bằng chứng cho model; chưa chứng nhận khả năng robot thật. Nếu chân vẫn mềm ở chế độ vị trí, kiểm gain driver
(`pos_gain`, `vel_gain`, `vel_integrator_gain`) bằng odrivetool.

## Prototype split CoM/gimbal — chỉ host

CMake host có `Config::Split` sau macro `TAP_SPLIT_HOST_PROTOTYPE`, mặc định tắt; PlatformIO không định nghĩa macro này nên prototype không có
trong firmware ESP32 (không có `SET tsplit`). Cổng A thất bại ở plant hông mềm + trễ bánh nên dừng trước B. Không dùng bản host làm chứng
nhận robot thật. Xem `docs/TSPLIT_IMPLEMENTATION_20261003.md`.
