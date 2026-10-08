# Khôi phục roll có cơ chế đáp ứng hoàn chỉnh — 05/10

Mục tiêu: roll không còn là một vòng tích phân thô giữ một góc cứng. **Roll là trục thứ hai (ngang) của cơ chế 2, gimbal**:
cùng một cơ chế giữ thân ngang và ổn định, pitch qua hông, roll qua chênh chiều cao hai chân, dùng chung quyền, thời gian chờ
và cơ chế đóng băng của gimbal. Chỉ là bằng chứng sim + log; không chứng nhận robot thật.

## 1. Bài học từ lần roll trước (log `~/tap_logs`, 01–05/10)

Thống kê từ dòng `TR` ở trạng thái Balancing (`roll` = ước lượng, `rdh` = chênh chiều cao hai bên):

| Điều kiện | `roll` trung bình | sd | `rdh` trung bình |
|---|---|---|---|
| Roll **bật**, 02–03/10 (đa số phiên) | −0.038 | 0.005–0.04 | **−5…−11 mm** (|rdh| ≥ 8 mm trong 5–90 % thời gian) |
| Roll **tắt**, 05/10 (5 phiên, 81–165 mm) | **−0.011…−0.018** | 0.002–0.018 | 0 |

1. **Mục tiêu sai.** `Roll::target_rad = −0.040` (đo một lần lúc chân xếp tay) làm vòng giữ thân ở −0.040 trong khi góc
   tự nhiên của robot là ≈ −0.013. Chênh 0.027 rad × 309 mm ≈ 8 mm: đúng bằng chênh chân thường trực. Nghĩa là roll bật
   đã **nghiêng thân có chủ ý** về một góc chọn sai. Góc tự nhiên đổi theo mỗi lần bật nguồn (chân xếp tay, lắp IMU).
2. **Sim tự lừa mình.** `Case::imu_roll_mount` mặc định cũng −0.040 nên sim luôn khớp hằng của controller; không ca nào
   có mount hay chân lệch khác. Sim cũng không đưa encoder từng bên có phần roll vào controller.
3. **Roll bị nghi nhầm là thủ phạm ngã khi hạ thấp**: đã loại (5.45, `lqr_lead_s` mới là yếu tố quyết định). Lỗi thật của roll
   là mục tiêu sai và vòng thô, không phải gây ngã.
4. **Rung chân khi bật `rollvff`** (gối rung ±2 mm / 40 ms): vòng nhanh trên ước lượng nhiễu, không có deadband/lọc theo nhiễu.
5. Một tầng duy nhất (PI) phải gánh cả dốc, chân lệch, vào cua và gờ; không có preview theo lệnh, không có giảm chấn theo
   gyro, không có cổng/latch khi vòng không hội tụ.

## 2. Thiết kế

Roll là phần **chênh lệch** trái/phải cộng vào lệnh chân (giữ nguyên: `Output::leg_roll_*`, IK từng bên chính xác tại tư thế
chung đã gồm `dhip`/suspension). Cơ chế 1 (phản xạ "ván trượt") vẫn chỉ ở trục pitch. Các phần của gimbal ngang:

- **Liên kết với gimbal pitch (`Roll::gimbal_link`).** Vòng roll chỉ mở khi gimbal pitch đã mở hoàn toàn (`Tier::level` bật,
  `level_beta_` = 1: sau chờ 2 s của tầng 2, không bị giữ/nhấc) và, như tích phân level của pitch, **ngừng tích phân khi bánh
  gần trần**. Tắt `SET tlvl 0` thì roll cũng không hoạt động. Hai trục dùng chung chân nên dùng chung cổng, tránh hai vòng
  chạy lệch pha nhau.

- **Tham chiếu ngang học tại chỗ (`learn_ref`).** Sau khi vào Balancing và êm (không giữ, không rẽ, tốc độ bánh nhỏ,
  gyro roll nhỏ) trung bình ước lượng roll trong 2 s với chênh bên = 0 → `ref`. Giả thiết sàn phẳng lúc bật; sai khác
  mount/chân xếp tay được hấp thụ, nên **trên sàn phẳng không có chênh chân thường trực**. Dốc xuất hiện sau đó thì được
  san bằng theo trọng lực. `SET rolltgt <rad>` ghim giá trị cố định, `SET rollcal` học lại ngay. Vòng roll chỉ mở sau khi
  học xong (ramp quyền 1.5 s).
- **Gimbal (lớp chậm).** PI chậm trên sai số roll đã lọc 2 Hz, τ 0.4 s, như cũ, nhưng quanh `ref` đã học; chặn windup theo
  hành trình khả dụng (`room`).
- **Preview vào cua (theo lệnh).** Độ nghiêng mục tiêu `atan(v·ω_lệnh/g)` dùng lệnh quay đã làm mượt (dẫn trước bánh) và
  tốc độ lọc, thay vì chỉ ước lượng từ đo đạc đã trễ; cộng vào mục tiêu như cũ nhưng dẫn trước.
- **Giảm chấn gyro (tùy chọn, `SET rolldamp`, boot 0).** `Δdh = −kd·(w/2)·ω_roll` có vùng chết mềm. Quét kd 0 → 0.15 trong sim cho
  đỉnh gờ 8 mm chỉ đổi 0.023 → 0.018 rad và kd ≥ 0.15 làm vào cua đảo chiều nhiều hơn, nên không bật mặc định.
- **An toàn.** (a) Chỉ chạy khi cân bằng tự do, không bị giữ/nhấc, sau `settle`. (b) **Latch lỗi:** chênh bên ≥ 0.9·`room`
  liên tục `fail_s` (2.5 s) mà sai số roll vẫn lớn thì roll tự rút quyền về 0 và khóa (báo `rlat=2`); không tự bật lại tới khi
  `SET roll 1`/`rollcal` hoặc vào lại Balancing. (c) Giới hạn tốc độ chênh bên như cũ. (d) `rollvff` vẫn tắt.
- **Phối hợp.** Roll chỉ đụng chênh lệch bên; suspension chạm trung bình hai bên; tầng 2 và IK cặp hông/gối đã nằm trong tư thế
  chung nên roll giải IK lại trên đó mỗi tick (đã có). Không đổi hành trình dự trữ: dự trữ 12 mm đã có vì trunk + knee hold.

## 3. Kiểm chứng (sim, thiết kế chống tự lừa)

Mở rộng mô hình ngang trong `v3_tier_tests.cpp` (V40, `TIER_ROLL2=1` in cả ba chế độ tắt / vòng cũ / phân lớp):
- mount IMU và chân xếp tay **khác** hằng của controller (mount ∈ {−0.040, −0.013, 0, +0.02}, chân lệch 8 mm);
- dốc xuất hiện **sau** lúc học (`Case::slope_t0`; dốc hằng từ t = 0 sẽ bị học thành tham chiếu);
- nhiễu gyro/gia tốc kế ×4 (`roll_noise_scale`), băng thông servo chân ×0.5 và ×2 (`roll_servo_scale`);
- chỉ số mới: |chênh bên| trung bình 4 s cuối, roll so với trọng lực, số lần đảo chiều chênh bên /s, chế độ cuối (0/1/2).
Mô hình ngang vẫn là động học (thân bám chân cứng, tải bánh theo lực hướng tâm); không có động lực học 3D thật.

Kết quả (sàn phẳng, mỗi mount): chênh bên cuối **0–0.4 mm** (vòng cũ với mount −0.013: 4.2 mm và thân lệch 0.027 rad, đúng như
log robot; mount +0.02: 9.3 mm). Dốc +0.06 và −0.05 xuất hiện lúc 12 s: thân về −0.001 rad, dưới 0.005 rad lúc 14.2–14.3 s.
Dốc +0.15 (quá hành trình): latch kích hoạt, chênh bên về 0 (vòng cũ kẹt 12 mm). Vào cua 0.55 m/s 1.5 rad/s: lệch so với phương
biểu kiến đỉnh 0.012 (roll tắt 0.090, vòng cũ 0.031), tải bánh nhỏ nhất 0.49 (0.44 / 0.48). Gờ 8 mm: 0.023 (tắt 0.026, vòng cũ 0.040).
Nhiễu ×4 và servo ×0.5/×2: không rung chân (≤ 0.2 đảo chiều/s). Gimbal pitch tắt: roll không hoạt động.

**Giới hạn đã biết.** Chân xếp tay lệch (8 mm) không phân biệt được với lệch mount IMU nên thân giữ nguyên độ nghiêng như khi roll tắt (0.026 rad)
nhưng không còn chênh chân thường trực; cần mốc ngang thật thì `SET rolltgt <rad>` (ghim, tắt học) sau khi đo bằng nivo/điện thoại trên sàn
phẳng, hoặc `SET rollcal` trên sàn đã biết phẳng. Lúc bật nguồn trên sàn nghiêng, mốc học bị coi là phẳng (không tệ hơn roll tắt).

## 4. Triển khai và rủi ro

Mọi thứ nằm sau `Config::Roll` (struct mặc định giữ hành vi cũ để test cũ không đổi); firmware bật các cờ mới khi `SET roll 1`.
Khóa mới: `rollcal` (học lại), `rolllearn`, `rollpre`, `rolldamp`, `rolllatch`, `rollgimbal`; `rolltgt` giờ ghim mốc và tắt học.
Log TR thêm `rref=` (mốc đã học) và `rm=` (0 chưa mở/đang học, 1 đang chạy, 2 latch). Boot ban đầu **giữ roll tắt** cho tới khi
người vận hành thấy log buổi thử đầu; sau đó có thể đổi mặc định boot.
Rủi ro còn lại, không kiểm được bằng sim: sàn nghiêng lúc bật bị coi là phẳng (hậu quả: không san bằng dốc ban đầu, không
tệ hơn roll tắt); đặc tính hai servo gối thật; lệch khối lượng ngang của pin. Cách hoàn tác: `SET roll 0`.
