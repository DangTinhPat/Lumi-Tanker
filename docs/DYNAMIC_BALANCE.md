# Cân bằng và điều khiển hai tầng sau khi dời pin — 17/10/2026

## Phạm vi

Nhánh `v3.5`, mốc đối chiếu `5ae6de9` (`rocking 16:38`). Mô hình GIM hiện tại,
khối lượng 3.02 kg, pin phía trước. Không sửa hình học, khối lượng, trọng lực,
ma sát hay mô hình truyền động để làm đẹp kết quả. `tap_mujoco_gim_v3` chạy
`V3Controller` và đường cảm biến MPU6500 + encoder; không dùng contact, lực bánh
hay CoM thật của MuJoCo làm đầu vào điều khiển.

Đây là bằng chứng SIM, chưa phải xác nhận trên robot. Firmware build được cả
hai môi trường; `tgov` mặc định **tắt** trong core và firmware. Không nạp robot.

## Nguyên nhân và thay đổi

Ghi 50 Hz không nhìn được rung 31 Hz. Ghi STATE 500 Hz cho thấy sau chạy 3 m/s
rồi STOP, lead gyro trực tiếp của hông kích vòng truyền động hông/mô-men và
phản hồi bánh. Tắt toàn bộ trunk loại rung nhưng mất phối hợp hông; vì vậy
giữ tầng 2 và chỉnh riêng đường lead. Các đối chiếu:

- Lead 0.06 s: rung gyro >5 Hz khoảng 1.73 rad/s, hông 0.0132 rad RMS.
- Chỉ bỏ giảm chấn thân: vẫn rung khoảng 41.6 Hz, gyro 1.04 rad/s.
- Chỉ lọc lead 6/15/30 Hz: ngã trong ca tăng tốc. Không dùng phương án này.
- Lead 0.02 s: êm ở một số profile, còn ngã với profile/trễ bánh khác.
- Lead 0 s, giữ preview + phản hồi lean + giảm chấn thân lọc 4.5 Hz: qua các
  ca tốc độ cao đã kiểm bên dưới.

Việc thử lại `tlead 0` có bằng chứng mới: plant chân chạy mô-men/trở kháng và
rung 31 Hz, khác plant vị trí và rung 6–8 Hz của HANDOFF 5.41/5.50. Đây là
tuning cho SIM hiện tại; không đổi lead khi boot firmware.

Tầng 1 tiếp tục LQR trên góc con lắc ảo; tầng 2 nhận cùng kế hoạch gia tốc,
đón trước góc `atan(a/g)` và giữ thân qua IK hông/gối. Không thêm một vòng
tích phân CoM cạnh tranh với trim của LQR.

`DriveGovernor.hpp` thêm ngân sách gia tốc trước kế hoạch chung của hai tầng:

1. Ước lượng phần mô-men motoring còn lại từ vận tốc encoder, vận tốc kế hoạch
   và yaw; mô hình mô-men giữ nguyên dưới tốc độ rated, giảm tuyến tính tới
   no-load. Chiều phanh giữ mô-men như mô hình bánh đã nhận.
2. Dùng bánh yếu hơn khi cua, chừa 45% mô-men khả dụng cho cân bằng.
3. Giới hạn gia tốc kế hoạch bằng `2*tau_available*(1-reserve)/(mass*radius)`.
   Kế hoạch vẫn giới hạn jerk: ngân sách mới không làm nhảy gia tốc tức thì.
4. Không tác động tới profile hop riêng. DISABLE/ESTOP/fault giữ luồng ưu tiên cũ.

Đây là một constraint governor đơn giản, không phải chứng minh miền bất biến
hay bảo đảm không ngã: Garone, Di Cairano, Kolmanovsky, *Reference and command
governors for systems with constraints*, Automatica 75 (2017), 306–328,
doi:10.1016/j.automatica.2016.08.013. Không sao chép mã nguồn ngoài.

## Cấu hình

| Thông số | SIM trước | SIM mới |
|---|---:|---:|
| Lead gyro hông | 0.06 s | 0 s |
| Gia tốc kế hoạch tối đa | 0.6 m/s² | 0.9 m/s² |
| Jerk kế hoạch | 10 m/s³ | 10 m/s³ |
| Governor | tắt | bật |
| Mô-men liên tục dùng cho ngân sách | — | 0.13 Nm/bánh |
| Phần mô-men chừa cân bằng | — | 45% |

Ngân sách ở tốc độ thấp ~1.184 m/s² (kế hoạch kẹp còn 0.9), ở 3 m/s ~0.560
m/s² trước khi trừ thêm ảnh hưởng cua. 0.2 Nm vẫn chỉ là peak ngắn của đứng dậy.

Firmware có `SET tgov 0|1`, `SET tgreserve <0..0.9>`, `SET tgtorque <0..0.13>`;
`GET` in cấu hình governor. Không coi mô-men khai báo là mô-men đo. Chỉ áp dụng
khi tham số actuator đã được kiểm trên robot. Hoàn tác governor: `SET tgov 0`.

SIM có `--control` để đối chiếu. Trở lại đúng điều khiển trước thay đổi:

```text
--control hip_lead=.06,governor=0,plan_accel=.6,plan_jerk=10
```

`--state-hz 500` chỉ tăng log STATE; VIEW vẫn 50 Hz, tần số điều khiển không đổi.

## Bằng chứng

Đo 5 giây cuối ca ENABLE 0 s, DRIVE 3 m/s 6 s, STOP 14 s, kết thúc 32 s,
roll bật; cùng mô hình và actuator:

| Đại lượng | Trước | Sau |
|---|---:|---:|
| Gyro >5 Hz RMS | 1.72859 rad/s | 0.01128 rad/s |
| Lệnh hông >5 Hz RMS | 0.013204 rad | 0.0000871 rad |
| Pitch đỉnh–đỉnh cuối ca | 0.09104 rad | 0.000837 rad |
| Pitch lớn nhất khi chạy/dừng | 0.09960 rad | 0.06241 rad |
| Tốc độ lớn nhất | 3.203 m/s | 3.052 m/s |

Gyro và rung lệnh hông đều giảm khoảng 99.3% trong cửa sổ này. Vận tốc dịch
chuyển thân thật, chấm offline từ VIEW, là 2.983 m/s ở đoạn cruise. Ca có lực
đẩy đạt đỉnh 3.36 m/s; pitch lớn nhất trong các ca chạy/đảo chiều là 0.09646 rad.
Thử jerk 3 m/s³ làm chậm đáp ứng hông, nên giữ 10 m/s³: hông vẫn chuyển động
ít nhất 1° trong 60 ms ở các sự kiện đổi tốc độ đã kiểm.

- Host: `v3_tests`, `v3_delay_tests`, `v3_standup_tests`, `gim_model_physics`
  đều qua. Contract governor kiểm tốc độ, cua, phanh, đảo chiều, mô-men bằng 0,
  reserve bằng 1 và mặc định tắt.
- Đường governor tắt và tuning cũ khớp **15,999 mẫu × 10 trường trạng thái/lệnh**
  với binary baseline; không chỉ khớp kết quả cuối.
- Planar: `TIER_ACCEL=1 ACC_H=.136 TR_GOVERNOR=1` qua 20 ca, không ngã hoặc
  held-reset; worst body drive 0.036 rad. Plant planar này khác MuJoCo và chưa
  xác nhận khả năng chạy 3 m/s của robot.
- `gim_high_speed_test.py`: trước 0/8, sau 8/8; full stick, giảm tốc, đảo chiều,
  cua, slalom, bánh delay/lag 70/110 ms và đổi chiều cao. Các ngưỡng cũ giữ nguyên.
- `gim_dynamic_balance_test.py`: countercase cũ tái hiện rung; 6 ca mới qua
  (chạy/dừng, nhiễu, lực đẩy, đảo chiều, đứng dậy rồi giữ yên, lái tư thế thấp),
  ghi 500 Hz. Ca tư thế thấp kiểm cả đáp ứng hông trong 60/80 ms.
- `gim_standup_test.py --extended --random-cases 12`: kiểm lại 34 ca recovery
  và hold/ESTOP. Kế hoạch B1–B4 giữ nguyên; profile dừng sau handoff dùng governor.
- Firmware: `pio run -e esp32-s3-v3 -e esp32-s3-v3-single-loop` đều build qua;
  RAM 158,172 byte; flash 1,121,925 / 1,120,893 byte. Chưa đo WCET trên ESP32.

Lệnh và log đầy đủ trong `build/native/gim_physics_evidence/`:
`dynamic_balance_regression/` (kèm hash binary/source/model),
`balance_standup_final/`, `balance_maneuver_jerk10/`.

Lệnh tái hiện (từ thư mục workspace, sau khi build runner):

```sh
ctest --test-dir build/native -R '^(v3_tests|v3_delay_tests|v3_standup_tests|gim_model_physics)$' --output-on-failure
TIER_ACCEL=1 ACC_H=.136 TR_GOVERNOR=1 build/native/bin/v3_tier_tests
ctest --test-dir build/native -R '^(gim_dynamic_balance|gim_high_speed)$' --output-on-failure
python3 src/tap_sim/tests/gim_standup_test.py --runner build/native/bin/tap_mujoco_gim_v3 --model src/tap_sim/model/gim_dynamic.xml --evidence build/native/gim_physics_evidence/balance_standup_final --extended --random-cases 12
```

## Giới hạn còn lại

Ở trạng thái nghỉ vẫn có thành phần rất nhỏ khoảng 230–250 Hz của plant bánh
và tích phân/contact: gyro ~0.01 rad/s, pitch vài phần mười mrad. Mô-men bánh
có thể đổi dấu nhanh do speed servo giả định cứng. Chưa nhận dạng servo bánh
thực để kết luận phần này giống robot. Không giảm gain vật lý hay sửa ma sát
để che nó. Các ngưỡng test kiểm rung ở thân và lệnh khớp; không chứng minh
độ êm mô-men thực hoặc xác suất thành công 100% ngoài tập ca đã chạy.

`gim_maneuver_test.py` còn fail ngưỡng tốc độ dư 0.020 m/s ở tư thế 0.08 m:
bản cũ 0.020860, bản mới ~0.020485 m/s, cùng không ngã và cổng reflex cuối ca
bằng 0. Không nới ngưỡng hoặc ghi bài này là pass. Chấm riêng 8 sự kiện từ log
xác nhận đáp ứng hông vẫn qua tiêu chí 60/80 ms. Các độ cao tiếp theo trong bài
này chưa chạy vì nó dừng tại assertion đầu; không tuyên bố toàn bộ suite qua.
