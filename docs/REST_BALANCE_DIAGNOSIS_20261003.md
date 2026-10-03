# Dao động khi đứng yên ở tư thế thấp — kiểm tra 03/10/2026

Phạm vi: đọc source hiện tại trên branch `fw`, phân tích log có sẵn, build
host và đối chứng plant 2D. Không thay controller, không nạp firmware và
không điều khiển phần cứng. Giữ nguyên các sửa đổi chưa commit của người dùng.

## Kết luận

Có hai vấn đề khác nhau:

1. Tầng chân hiện là bộ giữ thân và bù nghiêng động, chưa phải bộ giải tư
   thế tĩnh có mục tiêu CoM rõ ràng. Vòng chậm giữ pitch thân; vòng nhanh
   phản ứng với nghiêng/tốc độ con lắc ảo ngay cả khi không có lệnh DRIVE.
2. Dao động ở tư thế thấp có bằng chứng phù hợp với sai pha phản hồi do
   truyền động hông mềm. Chưa đủ bằng chứng để xác nhận đây là nguyên nhân
   duy nhất trên robot. Tên mục 5.37 trong handoff mạnh hơn mức xác nhận
   hiện có: sửa `tvglow=0.7` còn được ghi là chưa nạp thử.

Không có lệnh gia tốc không đồng nghĩa không có gia tốc thật: một vòng kín
không đủ giảm chấn có thể tự tạo chuyển động từ nhiễu nhỏ và stick-slip.
IMU đo tư thế, tốc độ quay và specific force; không trực tiếp đo vị trí CoM.
Ước lượng CoM cần encoder, FK và mô hình khối lượng đã hiệu chỉnh.

## Đường phản hồi gây nghi vấn

`src_fw/src/v3/V3Controller.cpp`:

- `updateVirtualPitch`: `theta_v = body_pitch + c_meas - c_hang`;
  `theta_v_rate = body_gyro + gain*(Jhip*hip_rate + Jknee*knee_rate)`.
- `update`: tốc độ odometry cũng cộng trực tiếp tốc độ hip/knee vào tốc độ
  bánh và gyro. Vì vậy `tvglow` chỉ xử lý một đường nhiễm, không xử lý hết.
- `updateTrunk`: `lean` là nghiêng con lắc ảo trừ trim/preview. Nhánh động
  dùng HPF 3 s, lead tốc độ, và bù giảm chấn; không có chế độ đứng yên
  riêng để loại bỏ việc đuổi theo dao động do chính hệ tạo ra.
- `updateWheelLegCoordination`: sau khi planner yên 1 s, quyền bù tăng lại.
  “Planner yên” không chứng minh robot yên hay có nhiễu bên ngoài.
- `updateTrunk`: `level_i` tích phân pitch thân, không tích phân sai số CoM.
- `coordinateTrunkAxle`: IK giữ chiều cao và X suy từ lệnh hông cũ; không
  giải đồng thời mục tiêu CoM, chiều cao và tư thế thân.

HPF 3 s không loại dao động 1.7 Hz: tần số góc của dao động nhân tau khoảng
32, nên biên độ qua HPF gần 1. Lead còn phản ứng với tốc độ của dao động.
Hông chuyển động -> đổi đường CoM -> bánh phản ứng -> thân/khớp chuyển động
-> tầng chân lại phản ứng. Nếu phản hồi khớp phía rotor sai pha với khâu,
các thành phần vốn được dùng để giảm chấn có thể làm giảm biên ổn định.

Ở tư thế thấp, thời gian đặc trưng `sqrt(z_com/g)` giảm, trong khi trễ cơ
khí/driver không giảm theo. Không nên coi `sqrt(g/z)` là tần số dao động
đo được của toàn vòng kín; nó chỉ là thang thời gian của con lắc tuyến tính.

## Bằng chứng đọc lại từ log

Chạy `tools/tr_rest_stats.py --calm` trên năm log 03/10:
171744, 164247, 163348, 162159, 155957 trong `/home/dvt/tap_logs`.
Có 6 cửa sổ 10 s, trong đó 5 ở bin 110 mm. Trung vị tại bin này:

- hông động peak-to-peak toàn cửa sổ 0.0638 rad;
- trung vị peak-to-peak từng giây 0.0285 rad;
- tốc độ IMU RMS 0.0838 rad/s;
- tốc độ con lắc ảo RMS 0.1454 rad/s;
- độ lệch chuẩn vận tốc axle 0.0415 m/s;
- đỉnh phổ tốc độ IMU 1.6991 Hz; phần công suất 1–4 Hz là 0.7297.

Bộ lọc dựa vào lệnh, chiều cao và burst; không chứng minh hoàn toàn không
có tay người. Không có cửa sổ đủ dài dưới 100 mm trong tập này để chứng
nhận trạng thái đứng yên ở độ cao đó. Dữ liệu không phải thí nghiệm nhân quả.

Handoff 5.35 đã đính chính giả thuyết bánh quá chậm: fit log 01–02/10 cho
lag 0.02–0.05 s. Do đó không chọn tăng lead bánh làm giải pháp mặc định.
Handoff 5.37 ghi tracking hông giảm và trễ tăng khi gập; đây là nghi vấn
mạnh hơn, cần phân biệt dữ liệu đo với giả định độ cứng truyền động trong sim.

## Đối chứng mới trên plant 2D

Build host riêng tại `/tmp/tap-rest-host-20261003/build`, chỉ đưa vào HAL,
core và `src_fw`. Root configure không chạy được do Python hiện tại thiếu
MuJoCo; các kết quả dưới đây không phải MuJoCo hay kiểm chứng 3D.

40 lần chạy: 2 plant × 4 chiều cao yêu cầu × 5 cấu hình, mỗi lần 40 s,
không lực đẩy ngoài và không DRIVE. Thống kê mẫu sau 20 s. Profile dùng
`TR_HEXP=1.5`, VMC và roll bật. Roll trong plant này chỉ là proxy.

Plant hông mềm giả định stiffness 20 N·m/rad/khớp, khe hở 0.001 rad,
gối stiffness 150 N·m/rad, trễ lệnh bánh 40 ms thay vì danh định 20 ms.
Đây là giả thuyết để thử độ nhạy, không phải thông số đã đo của robot.

Tại chiều cao yêu cầu 95 mm:

| Cấu hình | Chiều cao hip–axle đứng trung bình thực tế | std nghiêng plant | Hông động pp | Axle pp |
|---|---:|---:|---:|---:|
| Đủ tốc độ khớp, plant mềm | 100.8 mm | 0.0852 rad | 0.2189 rad | 423.7 mm |
| Trọng số tốc độ khớp thấp 0.7 | 93.8 mm | 0.0161 rad | 0.0600 rad | 128.4 mm |
| 0.7 và deadband hông 0.003 | 93.8 mm | 0.0159 rad | 0.0570 rad | 178.0 mm |
| 0.7, bỏ nhánh động/giảm chấn, giữ level | 168.1 mm | 0.1164 rad | 0.1498 rad | 470.3 mm |

Hai hàng đầu không có cùng chiều cao thực tế: không diễn giải mức giảm
thành hiệu quả tại một operating point khớp tuyệt đối. Hàng cuối không đạt
chiều cao yêu cầu nên phải bác bỏ, dù harness không báo ngã.
Trên plant cứng danh định ở 95 mm, giảm trọng số 0.7 chỉ thay std nghiêng
0.0034 -> 0.0031 rad; không phải cải thiện lớn ở mọi plant.

Không báo ngã không đồng nghĩa đạt ổn định: nhiều lần vẫn có dao động,
trôi hoặc lệch tư thế lớn. Deadband giảm rung chân đôi chút nhưng tăng axle
pp trên plant mềm; không chọn nó làm sửa chính. Tắt nhánh động thường trực
cũng không đạt. Số liệu nằm trong `log/rest_audit_20261003/results.json`;
script và SHA256 nguồn lưu cùng thư mục. CSV chi tiết ở
`/tmp/tap-rest-evidence-20261003`.

`v3_coordination_tests` chạy đạt: FK/IK, giữ chiều cao, startup, giới hạn
workspace, wheel-carrier và Ready. Chạy đầy đủ `v3_tier_tests` trên source
hiện tại dừng với exit 134 tại dòng 3541, assertion
`!rh.fell && rh.held_resets >= 1 && rh.held_max_dhip < 0.005`.
Đây là ca held có preview trong V15; chưa tách được thành phần nào của
assertion sai. Không thay source điều khiển trong lượt này và không sửa
assertion. Vì vậy không báo suite hồi quy đầy đủ đạt và chưa có cơ sở
đưa một thay đổi controller mới vào mặc định.

## Giải pháp đề xuất theo thứ tự

1. **Xác nhận phản hồi trước.** Bản app hiện đã đặt `joint_rate_gain_low=0.7`,
   nhưng source không chứng minh firmware đang chạy có giá trị đó. Kiểm
   provenance trước khi quy kết sửa đã thất bại. Dùng 0.7 như ứng viên giảm
   độ nhạy có bằng chứng sim; không gọi đó là sửa dứt điểm. Không bỏ tốc độ
   khớp hoàn toàn. Kiểm cơ khí/gain driver là phần cần xác nhận riêng.
2. **Tách mục tiêu đứng yên khỏi bù gia tốc.** Bánh giữ quyền bắt cân bằng
   nhanh. Chân có planner tư thế chậm, giải theo CoM/chiều cao/thân mong muốn,
   giới hạn tốc độ và gia tốc. Tách feed-forward từ gia tốc kế hoạch khỏi
   phản ứng với nhiễu. Phản ứng nhiễu vẫn phải có nhưng cần xác minh biên
   ổn định với trễ/độ mềm, không bật chỉ vì gyro hoặc encoder đang dao động.
3. **Một nơi sở hữu mục tiêu cân bằng.** Vòng bánh và vòng chân dùng chung
   mục tiêu con lắc ảo đã hiệu chỉnh; tránh thêm một tích phân CoM song song
   với trim bánh. Ở đứng yên: CoM gần trên axle, tốc độ CoM và bánh gần 0,
   chiều cao đúng và pitch thân đúng. Khi gia tốc: cho phép CoM lệch theo
   gia tốc yêu cầu. Chỉ ép CoM về axle mọi lúc sẽ chống lại việc tăng/giảm tốc.
4. **Xử lý độ tin cậy mô hình.** CoM thân mặc định đã được bù +21 mm từ đo
   trước; không dùng CAD chưa hiệu chỉnh để giải tư thế. Encoder phía rotor
   không quan sát được hoàn toàn độ võng phía khâu. Chưa có thêm cảm biến,
   nên giới hạn bandwidth, giữ sai số mô hình/biên độ hợp lý, và thể hiện
   bất định thay vì giả định FK chính xác tuyệt đối.

Lý thuyết nền: Capture Point (Pratt et al., Humanoids 2006) nhấn mạnh trạng
thái vị trí + vận tốc khi bắt cân bằng; wheel-legged whole-body control dùng
phối hợp bánh và khớp dưới ràng buộc. Tham khảo
https://doi.org/10.1109/iros51168.2021.9636371
(Whole-Body MPC and Online Gait Sequence Generation for Wheeled-Legged Robots).
Không sao chép giới hạn/gain robot khác và không cần chuyển ngay sang MPC lớn.

Tiêu chí cho prototype trước khi thay mặc định: đứng 60 s tại từng chiều cao
thực tế 180/110/95/85 mm, không drift/tăng biên độ; giảm rung chân mà không
tăng rung thân/bánh; đạt chiều cao; sau nhiễu phải trở lại đứng yên; drive,
brake, đổi chiều cao và stop không thoái lui. Chạy nhiều độ cứng/trễ/nhiễu,
so cùng operating point và không chỉ chọn một plant thuận lợi.
