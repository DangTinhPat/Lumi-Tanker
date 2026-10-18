# Kế hoạch cải thiện ổn định khi hạ thấp (crouch) — 05/10

> **Tài liệu quá trình (05/10).** Các mục 2–10 chứa giả thuyết và kế hoạch đã được thay thế; kết luận hiện hành nằm ở mục 11–12 của file này và `docs/V3.3_HANDOFF.md` (5.45–5.47). Không còn VMC trong mã.

## 1. Vấn đề và tiêu chí thành công
**Hiện tượng (robot 05/10, log `tap_20261005_113328.log`):** giữ ▼ hạ 176 → ~99 mm trong ~6 s,
thân chao đảo ~1.8 Hz từ ~133 mm, bùng lại ở ~100 mm và ngã (`fallen (pitch limit)` 75.1 s).
Không còn VMC, đã có `tvglow` 0.7, cổng tắt. Cùng kiểu ngã ghi ở 5.36/5.37.

**Thành công** khi hạ **liền một mạch** (cách người vận hành vẫn làm, không cần hạ từng nấc)
từ 176 mm xuống 90 mm ở tốc độ ≤ 11 mm/s:
- không ngã trong 5 lần liên tiếp;
- đỉnh |θv| < 0.10 rad, đỉnh |`axle_v`| < 0.30 m/s, không có dao động 1–3 Hz tăng dần;
- đứng yên ≥ 60 s ở 90 mm; không hồi quy ở bảng điểm sim hiện có (ma trận 84 ca, V35/V36,
  ACCEL/GENERAL) và `alt%` hông vẫn thấp (`tools/hip_chatter_stats.py`).

## 2. Bằng chứng: biết gì, chưa biết gì
**Biết (từ log robot):**
- Ngã từ phiên đầu tiên sau `041a99b` (02/10 10:35). Thời `b4b6d4a` (01/10) đứng 5000–34000
  mẫu ở 78 mm, `tv` sd 0.009–0.025; sau đó `tv` sd 0.04–0.16 ở 82–113 mm.
- Cú ngã là dao động 1.7–2 Hz (≈ tần số riêng √(g/l) ở 92–100 mm) tăng ×1.4–2 mỗi nhịp, hông
  ±0.06–0.08 rad và lệnh bánh ±5–12 rad/s cùng tham gia.
- Không phải VMC (đã xóa), không phải `tvglow` (đã bật), không phải độ trễ hông ở tư thế cao
  (46 → ~57 ms), không phải 5.29.

**Đính chính:** góc cân bằng thật **không** dịch −0.05 rad theo độ cao như tôi nói trước. Số đó là
do gộp phiên. Trong cùng một phiên, `ref` đổi từ cao xuống thấp chỉ −0.010 (136–144 mm),
−0.015…−0.026 (112–120 mm), và ở ≤ 96 mm lẫn lộn dấu (−0.032, +0.004, +0.027). Trong khi `ref` ở
tư thế cao khác nhau tới ±0.04 giữa các lần bật nguồn (chân xếp tay lệch, mục 5.12). Nghĩa là góc
cân bằng phụ thuộc mạnh vào mỗi lần bật nguồn, độ cao chỉ đóng góp nhỏ.

**Chưa biết:** vòng nào mất ổn định (bánh một mình, hay bánh–hông); sim không tái hiện được cú
ngã (5.36) nên chưa kiểm được cải tiến nào bằng sim.

## 2b. Dữ liệu mới của người vận hành và kết quả đối chiếu (05/10, sau khi lập kế hoạch)
**Dữ liệu:** ở `95f17a4` (v3.2, trước phân tầng) robot hạ được tới độ cao thấp nhất; sau khi thêm phân
tầng và cân bằng roll thì gặp lỗi.

**Đối chiếu commit và log (có thêm thông tin):**
- `6da60c5` (30/09) thêm phân tầng, `dc1a972` thêm vòng level, `b4b6d4a` (01/10 11:31) có phân tầng + level +
  suspension nhưng **chưa có roll**: robot ở bản đó **đứng được 78 mm** (5000–34000 mẫu, `tv` sd 0.009–0.025).
  Vậy phân tầng một mình không gây lỗi.
- `struct Roll` **chỉ xuất hiện từ `041a99b`** (01/10 22:59). Đó cũng là commit đầu tiên có lỗi (phiên 02/10
  10:35). Các thay đổi mới của `041a99b` trên đường điều khiển: **roll**, `knee_hold`, `lqr_lead_s` 0.08 → 0.03,
  mô hình bánh đo lại, FreeRTOS, lead_exp 0.5, mặc định `height_exp` (boot ghi đè lại 1.5).
- Mọi phiên từ 02/10 đều bật roll; chưa có log nào tắt roll, nên log không phân biệt được các thay đổi này.
- Sim phẳng không có tác động roll nào lên động lực pitch (lệnh từng bên không đi vào plant phẳng):
  đây có thể là lý do sim không tái hiện được cú ngã (5.36).

**Đã kiểm tra bằng tính toán (cơ chế roll):**
- Loại trừ: đường roll tính IK từng bên trên tư thế chung đã có `dhip`, nhưng hệ số hông hiệu dụng của tầng 2
  giữ ≈ 1.00 ở mọi độ cao (177–85 mm). Không phải cơ chế "roll đổi hệ số vòng".
- Thực tế: robot giữ chênh chiều cao hai bên `rdh` trung bình −9…−12 mm (03/10 tối, 05/10); 10 mm cho lệnh
  hông ±0.058 rad và gối ±0.105 rad mỗi bên ở 100 mm. Khả năng là bù độ lệch IMU roll hoặc sàn nghiêng
  (chưa phân biệt được). Ở 85 mm, ±10 mm đẩy chân ngắn vượt giới hạn IK (vùng không khả thi).
- Chưa kiểm: θv dùng góc **trung bình** hai bên (sai số bậc hai khi chân lệch), kênh roll trong gyro/gia tốc kế,
  chiều cao khởi động thấp hơn 12 mm khi bật roll.

**Hệ quả cho kế hoạch:** các giả thuyết thay đổi thứ hạng: **roll (H7)** và `knee_hold` đứng đầu cùng H1
(lead bánh), vì cả ba cùng đến từ `041a99b`. Pha B đổi thứ tự thử, mỗi lần chỉ một lệnh `SET`.

## 3. Các giả thuyết, xếp theo khả năng và chi phí kiểm
| # | Giả thuyết | Bằng chứng hiện có | Cách phân biệt | Chi phí |
|---|---|---|---|---|
| H1 | Lead phản hồi vòng bánh giảm `lqr_lead_s` 0.08 → 0.03 (`041a99b`), không còn đủ giảm chấn mode 1.9 Hz ở tư thế thấp | Khác biệt cấu hình có thật; `legacy 1` không hoàn tác; sim thích 0.03 | `SET lqrlead 0.08` trên robot | 1 lần thử |
| H2 | Tầng 2 (hông) tham gia mode con lắc ở tư thế thấp: nhóm "legacy" (texpt, lead_exp, level τ, knee_hold, carrier, body damping) | Hông ±0.08 rad trong dao động; 5.36 từng dự kiến thử | `SET legacy 1` | 1 lần thử |
| H3 | Sai số mô hình CoM / góc cân bằng khi hạ, trim học quá chậm | Chỉ dịch −0.01…−0.03 trong phiên; `axle_v` +0.13…0.23 lúc hạ | Đã thấy là nhỏ; đo thêm bằng `REC` | thấp |
| H4 | Hông mềm và trễ hơn khi chịu tải ở tư thế gập (82 ms ở 95–130 mm, 5.37) | Đo từ log 03/10; sim hông mềm tái hiện một phần | Nhận dạng từ `REC` 250 Hz | trung bình |
| H5 | Thay đổi kiến trúc của `041a99b`: FreeRTOS tách vòng, chu kỳ lệnh chân 3 ms, roll, suspension | Chỉ trùng thời điểm | `roll 0`, `tsus 0`; build `TAP_V3_SINGLE_LOOP` | thấp đến trung bình |
| H7 | Cân bằng roll (mới ở `041a99b`): chênh chân thường trực −9…−12 mm, vùng không khả thi gần 85–100 mm, sim không thấy | Chỉ trùng thời điểm và dữ liệu vận hành; cơ chế đổi hệ số hông đã loại | `SET roll 0` | 1 lần thử |
| H6 | Cách hạ: tốc độ, đổi độ cao kích thích mode | 11 mm/s ngã ở 92–94 mm, 7–9 mm/s ở 78–84 mm thì qua (01/10) | So tốc độ trong `REC` | thấp |

## 4. Kế hoạch theo pha
### Pha A — không cần robot (tôi làm, 1–2 buổi)
- **A1. Pháp y 6 vụ ngã** (02/10 → 05/10): tần số, tốc độ tăng, pha giữa θv, `dhip`, lệnh bánh, góc
  hông đo. Trả lời: bánh một mình mất ổn định, hay bánh–hông. Sản phẩm: bảng chẩn đoán.
- **A2. Hiệu chuẩn plant để sim tái hiện đúng cú ngã** ("bản sao số" của sự cố): tìm tham số
  (hông mềm 15–40 N·m/rad, trễ hông và bánh khi gập, lệch CoM, lệch zero chân) sao cho sim cho
  dao động ~1.8 Hz tăng ×1.4–2/nhịp ở ~100 mm với đúng cấu hình 05/10. Không có bước này thì mọi
  cải tiến điều khiển đều không kiểm chứng được. Tiêu chí: sim ngã trong cùng khoảng 1–2 s.
- **A3. Chuẩn hóa bảng điểm hạ thấp:** gộp các script đã dùng thành một lệnh (ma trận 84 ca, V35/V36,
  ACCEL/GENERAL, quét độ trễ hông, thêm ca "hạ liền mạch 176 → 90 mm").

### Pha B — một buổi robot (~30 phút, theo cách bạn quen)
Mỗi lần thử = START, `REC 8` ngay trước khi hạ, rồi giữ ▼ liền một mạch xuống ~90 mm, quan sát,
`DISABLE` nếu ngã hoặc rung tăng. Mỗi lần ~2–3 phút. `REC` ghi 250 Hz ~8 s (θv, pitch, gyro,
bánh lệnh/đo, hông lệnh/đo), không đổi hành vi; dữ liệu này dùng cho A1 và A2.
1. **B1:** 1 lần ở cấu hình hiện tại với `REC 8` (dữ liệu nền 250 Hz; đã thấy ngã nên 1 lần là đủ).
2. **B2:** `SET roll 0`, hạ liền một mạch. (Giả thuyết của người vận hành, rẻ nhất, đổi một thứ.)
3. **B3 nếu vẫn ngã:** thêm `SET khold 0` và `SET lqrlead 0.08` (cùng nguồn gốc `041a99b`); nếu đứng được thì tách
   từng cái.
4. **B4 nếu vẫn ngã:** `SET legacy 1`, `SET tvglow -1`, `SET tsus 0`.
5. **B5:** xác nhận cấu hình tốt nhất 3 lần liên tiếp.
Nếu cả B2–B4 đều ngã: các tham số `SET` bị loại; chuyển sang RTOS, chu kỳ lệnh chân (H5), nhận dạng hông (H4) và A2.

### Pha C — thiết kế theo kết quả
- **Nếu H1 (lead bánh):** lead theo độ cao `lqr_lead_s(h)` (0.03 cao → ~0.08 thấp), hoặc dựng lại gain
  LQR cho tư thế thấp bằng mô hình bánh đo ở tư thế thấp; kiểm bằng bản sao số.
- **Nếu H2 (tầng 2):** khôi phục tối thiểu tập con "legacy" gây ra; hoặc đóng cổng phản xạ khi h < ~110 mm;
  hoặc tách băng tần: tầng 2 chỉ hoạt động ở dải hông theo kịp.
- **Nếu H3:** thích nghi trim nhanh khi độ cao đang đổi (tăng hệ số tích phân khi |tốc độ độ cao| > 0);
  không cần bảng góc cân bằng theo độ cao (dữ liệu không ủng hộ).
- **Nếu H4:** mô hình hông mềm vào thiết kế gain (trễ và độ mềm theo độ cao); xem xét tăng gain vị trí
  driver hông.
- **Nếu H6:** bộ điều tiết tốc độ hạ theo biên độ dao động (đã có phần "giảm mềm theo độ yên của thân":
  siết chặt để người vận hành giữ ▼ vẫn an toàn mà không phải hạ từng nấc).

### Pha D — xác nhận và hồi quy
Bảng điểm sim đầy đủ trước khi nạp; 5 lần hạ liên tiếp trên robot; kiểm `alt%` hông; cập nhật handoff.

## 5. Rủi ro và nguyên tắc
- Mỗi thay đổi mã mới mặc định tắt, bật bằng `SET`; hoàn tác được ngay. Không nạp lại giữa các lần B nếu
  chỉ đổi `SET`.
- Robot ngã mỗi lần thử hỏng: người vận hành đỡ hoặc đặt đệm; dừng ngay khi `DISABLE`/ESTOP.
- Dữ liệu 25 Hz trong log TR quá thưa cho dao động 1.8 Hz: ưu tiên dữ liệu `REC` 250 Hz.
- Mọi kết quả sim là bằng chứng cho model, không chứng nhận khả năng robot thật.

## 6. Thứ tự và quyết định cần từ bạn
Thứ tự đề xuất: A1 → Pha B (B2 `SET roll 0` trước) trong cùng một buổi → A2 từ dữ liệu `REC` → Pha C theo kết quả.
Cần bạn chọn: (1) độ cao đích (90, 100 hay 78 mm) và tốc độ hạ mong muốn; (2) dành một buổi robot
~30 phút cho Pha B; (3) ưu tiên: ngã ít nhất hay đứng thấp nhất.

## 7. Trạng thái 05/10 (cập nhật)
- Thử B2: `SET roll 0` gửi lúc chạy, đã xác nhận bằng `GET` (`roll=0`) trong log. `SET` mất khi robot reset hoặc mất nguồn.
- Mặc định boot trong mã đã đổi sang `roll.enabled = false` (`AppV3.cpp`), build đạt, **chưa nạp**. Bật lại bằng `SET roll 1`.

## 8. Kết quả B2 (roll tắt) và bước tiếp — 05/10
- Log `tap_20261005_115810.log`, firmware `e6259a0f…dc07` (roll tắt từ boot, `rdh` = 0 suốt phiên): **vẫn ngã ở ~100 mm.**
  Hạ liền một mạch 176 → ~100 mm trong ~6 s (`tv` lệch tệ nhất −0.079), dừng hạ lúc 142.3 s, mất cân bằng và ngã
  lúc 145.0 s (~2.7 s sau khi dừng). Không có dữ liệu `REC` (không gõ). **Roll bị loại (H7).**
- Đối chiếu cấu hình `b4b6d4a` so với hiện tại: lõi LQR (`src/tap_control/core`) và `defaultV3Config()` **không đổi**;
  tham số duy nhất của vòng bánh khác là `lqr_lead_s` 0.08 → 0.03 (`95f17a4` cũng dùng 0.08).
- Bước tiếp: `SET lqrlead 0.08` (gửi lúc robot Idle; firmware không in lại giá trị, chỉ báo lỗi nếu khóa lạ nên không có lỗi = đã nhận).
  Nếu vẫn ngã: `SET khold 0`, rồi `SET legacy 1`, `SET tvglow -1`, `SET tsus 0`.

## 9. Kết quả với `SET lqrlead 0.08` (roll tắt) — 05/10
- Log `tap_20261005_120513.log`: START 84.9 s, hạ 170 → 79 mm trong 6.7 s (đỉnh 25 mm/s), **đạt 78.4 mm (thấp nhất) và đứng ~75 s
  không lỗi** (DISABLE bởi người vận hành lúc 180.9 s). Roll tắt một mình (lần trước) vẫn ngã; roll tắt + `lqrlead 0.08`
  đứng được: **`lqr_lead_s` là yếu tố quyết định** (một lần thử; cần lặp lại, và chưa biết roll bật + lead 0.08 có đứng không).
- Lúc hạ ở 70–100 mm: `tv` sd 0.062, lệnh bánh sd 10 rad/s, đỉnh |`axle_v`| 0.93 m/s; khi đứng yên ở cùng độ cao: `tv` sd 0.020,
  bánh sd 2.6. Kích động lớn nhất nằm ở **lúc dừng** (dừng ở 96.7 mm: `tv` +0.037; chạm 78.4 mm: `tv` +0.069, bánh ±8 rad/s).
- Người vận hành cảm thấy chuyển độ cao "thô". Đã có tầng giới hạn giật nhưng chỉ trên chiều cao (S-curve: gia tốc 70 mm/s², giật
  500 mm/s³) cùng hệ số giảm tốc theo sai lệch/tốc độ pitch (phản ứng sau khi thân đã lệch). Các giá trị này đã **tăng** so với
  `b4b6d4a`/`041a99b` (15 mm/s, 50 mm/s², không giới hạn giật) khi làm 5.23 ngày 02/10.
- Sim quét (rate, accel, jerk) × 7 biến thể: peak lệch thân gần như không đổi (0.019–0.021): sim không nhạy với cú giật khi
  hạ, và sim **ngược chiều robot** về `lqr_lead_s` (0.03 tốt hơn, 0.08 ngã 1/7). Mô hình bánh trong sim chưa đúng ở tư thế thấp.
- Việc cần làm: A/B trên robot `SET hrate`/`haccel`/`hjerk` (giá trị cũ 15/50 + giới hạn giật chặt hơn), thiết kế tầng kiểm soát
  giật theo nhu cầu bánh (không chỉ chiều cao), nhận dạng lại bánh (`WHEELID`) để sim đồng ý với robot.

## 10. Chuẩn bị buổi thử tiếp (build, chưa nạp)
Mặc định boot trong `AppV3.cpp`: roll tắt, `lqr_lead_s` 0.08, `height.rate` 15 mm/s, `accel` 50 mm/s², `jerk` 200 mm/s³. Hoàn tác: `SET lqrlead 0.03`, `SET hrate 25`, `SET haccel 70`, `SET hjerk 500`, `SET roll 1`. So cảm giác hạ liền một mạch với lần 120513 (25 mm/s, 70, 500).

## 11. Trôi sau khi hạ và góc cân bằng động (05/10, log `tap_20261005_121501`, đã sửa trong mã)
**Quan sát (robot, hạ 15 mm/s, 3 lần bấm):** không ngã, nhưng sau khi tới 78 mm robot trôi về sau **−0.21 m** (508–516 s) và
`ref` (trim cân bằng) mất ~25 s để hồi từ −0.031 về −0.004. `tv` thực tế đứng gần −0.003 trong khi `ref` còn −0.026: LQR tự
nghiêng về góc cân bằng thật còn trim thì chậm. Người vận hành phải nhả/bấm nhiều lần để robot kịp ổn định.

**Nguyên nhân (đọc mã):** trim chỉ được chỉnh bằng tích phân sai số **vị trí** với hệ số `lqr_trim_gain` = 0.01 rad/(m·s)
(`V3Controller.cpp` ~755): muốn dịch 0.03 rad phải trôi ~0.15 m trong ~20 s. `learned_trim` chỉ lưu và chỉ dùng khi thả sau
một lần giữ. Khi hạ, góc cân bằng thật đổi (+0.016 trong phiên này, thay đổi theo lần bật nguồn và sai số CoM), nên luôn có
một giai đoạn robot chạy theo trim sai.

**Sim tái hiện:** hạ tới 78 mm: trôi đỉnh 0.18 m, ổn định ~30–50 s; sai số CoM (biến thể `body dx`): 0.9–1.1 m.

**Sửa:** `Config::trim_acc` (`SET ttrimacc 0/1`, `SET ttrimtau <s>`; boot = bật, 3 s). Ước lượng góc cân bằng từ **gia tốc mặt
đất đo bằng bánh**: `θeq = θv − atan(a/g)` (dùng bánh, không dùng gia tốc kế vì khung nghiêng của nó chứa sai số pitch IMU),
trim đuổi theo với τ = 3 s; khóa khi |a| > 0.8 m/s² hoặc |tốc độ pitch| > 0.3 rad/s. Giữ nguyên tích phân vị trí.
- Sim hạ tới 78 mm (7 biến thể): trôi đỉnh nominal 0.18 → 0.09 m, thời gian ổn định 31–59 s → 6–25 s; trung bình 0.43 → 0.31 m
  (τ = 1.5 s: 0.28 m, nhưng thân xấu thêm tới +0.028 rad). Ca CoM sai nặng (`body dx .045`) vẫn 1.1 m: chạm trần trim 0.20 rad.
- Hồi quy: ma trận 84 ca, kịch bản khả thi 0 ngã, thân trung bình −0.0002 rad (tệ nhất +0.013); ACCEL/GENERAL 36 ep giữ nguyên;
  đẩy 8 N (ngoài khả năng bánh) 14 → 17 ngã (ranh giới). Test host đạt, `pio run` đạt.
- Chưa kiểm trên robot: ngưỡng gating, chất lượng gia tốc từ bánh trên phần cứng thật. Hoàn tác: `SET ttrimacc 0`.

## 12. Cơ chế 1 ở tư thế thấp gần như không hoạt động (05/10) — đã sửa trong mã, kiểm bằng log và sim
**Đo từ log robot (đứng yên, băng phản xạ ~1.5 s):** hệ số `dhip/lean` ở tư thế cao 0.9–0.97 (các phiên 05/10), ở tư thế thấp
**0.21–0.28** ở mọi phiên, kể cả `b4b6d4a`. Khớp với code: `share = ratio^height_exp`, `height_exp` boot 1.5 → 0.27 ở 78 mm
(0.39 ở 100 mm); lead hông cũng giảm theo `ratio^0.5`.

**Nhận định:** việc cắt phần hông ở tư thế thấp (`texp` 1.5, mục 5.13/5.36) được chọn trong các phiên mà `lqr_lead_s` cũng đang 0.03,
nên nguyên nhân thật có thể đã bị quy nhầm cho phần hông. Với `lead` 0.08 không có bằng chứng robot nào chống lại phần hông lớn hơn.

**Sim (7 biến thể plant, 100 và 84 mm):** với `lead` 0.08, `texp` 1.5 → 0.5 → 0: lệch thân khi đẩy 4 N 0.079 → 0.051 → 0.032, lái 0.030 →
0.025 → 0.022, **không thêm ca ngã** (hai ca ngã còn lại giống nhau ở mọi `texp`, là biến thể "bánh trễ +20 ms" của sim). Với `lead` 0.03:
`texp` 0 làm ngã khi đẩy (khớp lịch sử "hông full rung"), 0.5 thì không. Quét trễ hông 0/+20/+40 ms, `texp` 0.5: 0 ngã; đẩy 4 N tốt hơn
30–40 % đến +20 ms, bằng ở +40 ms; đảo chiều hông khi đứng 12 so với 10 lần/s (sim). Chọn `texp` 0.5 (an toàn cho cả hai giá trị `lead`).

**Trong mã:** `AppV3.cpp` boot `tier.height_exp = 0.5` (trước 1.5; `SET legacy 0` cũng 0.5). Hoàn tác: `SET texp 1.5`.
Cấu hình boot hiện tại: roll tắt, `lqr_lead_s` 0.08, hrate/haccel/hjerk 15/50/200, `trim_acc` bật (3 s), `texp` 0.5.
Firmware nạp lên robot 05/10 (SHA256 `8138d75c…8799`); chưa kiểm trên robot.

