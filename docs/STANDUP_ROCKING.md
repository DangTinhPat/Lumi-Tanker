# Đứng dậy từ tư thế nằm (rocking) — RB v2, GIM6010-8

Trạng thái: **chạy trong mô phỏng, tắt mặc định, chưa nối vào firmware, chưa thử trên robot.** Mọi số liệu là của mô hình
`src/tap_sim/model/gim_dynamic.xml` (bánh giới hạn **0,13 N·m**, đúng thông số bạn xác nhận; hình học chưa kiểm với STEP).

## 1. Việc làm được

Từ keyframe `home` (thân sát sàn, hông +74,8°, gối 0, nằm trên vỏ bụng và hai bánh, trọng tâm trước trục bánh 26 mm),
lệnh `STANDUP` đưa robot đến trạng thái cân bằng ở độ cao đầy đủ rồi lái được:

| Mốc | Thời điểm (từ lệnh) | Trạng thái |
| --- | --- | --- |
| Giữ | 0 – 0,3 s | chân và bánh đứng yên, IMU và encoder ổn định |
| Gập hông | 0,3 – 3,3 s | hông gập 0,8 rad/s tới 0,6 rad rồi 0,5 rad/s; bánh lăn tự do |
| Bàn giao | ~3,3 s | trọng tâm gần trục (|lean| < 0,12, tốc độ lean < 1,5 rad/s): V3 vào `Balancing` |
| Cân thân | 3,3 – 6 s | các tầng của V3 đưa thân từ chúi 0,6 rad về ngang |
| Đứng lên | 6 – 14 s | bộ chiều cao của V3 duỗi gối 0 → 1,69 rad (thân cao 0,204 m) |

## 2. So với quỹ đạo `RockerLower` của nhánh `tpa`

`tpa` là dòng robot cũ (`tap_robot.xml`), quỹ đạo gồm bảy pha (hạ hông, quăng, giữ, wheelie, bắt, cân thân, đứng) và cần **bánh
0,8 N·m** trong mô phỏng ("exploring a wheel/gearbox spec beyond the shipped 0.13 Nm"). Các con số của nó (hông 2,69; mức
1,1248 ở gối −1,2; lực bánh) là của hình học cũ và không dùng lại được. Ở RB v2:

- **Bỏ** hạ hông, quăng và wheelie: `home` đã nằm sẵn trên bụng, và bánh 0,13 N·m không đủ kéo thân.
- **Giữ ý chính**: trình tự là một bước chuẩn bị rồi bàn giao; gối ở chốt, hông gập; mọi quyết định dùng IMU và encoder chân.
- **Khác**: bánh được lệnh **bằng chính tốc độ đo được** (không bị hãm ở 0, cũng không đẩy; xem mục 4 về mô-men thực), bộ bắt là **bộ cân bằng V3** (LQR + các tầng),
  không phải bộ cân theo độ nghiêng riêng của `RockerLower`.

## 3. Vì sao thiết kế như vậy (những gì đo được, từng bước)

- **Quy ước**: `lean = pitch + com_line_angle(hông, gối)` (cùng công thức của `RockerLower` và của V3); `lean > 0` là trọng tâm
  trước trục. Ở `home` V3 báo `com_line_angle = 1,03 rad`, đúng bằng số tính độc lập từ MuJoCo.
- **Đích**: ở gối 0, thân ngang, trọng tâm nằm trên trục bánh khi hông ≈ 0,2 rad. V3 giữ được cân bằng ổn định ở đó (đo 30 s: hông
  0,2015, lean −0,009), nên đích cuối đã chạy được. Chỉ phần chuyển từ dáng nằm là mới.
- **Bánh phải lăn tự do.** Lần đầu bánh được lệnh tốc độ 0 (hãm) hoặc đẩy tiến: robot kẹt ở thân chúi 0,42 rad, bụng trước tì sàn,
  trọng tâm vẫn trước trục, vì bánh chỉ có 6,5 N lực kéo còn ma sát bụng ~15 N. Cho bánh lăn tự do thì hình học tự đưa trục bánh dần
  về phía trọng tâm khi hông gập.
- **Đích hông theo pitch hiện tại**, không theo thân ngang: khi thân đã chúi, muốn lean = 0 thì hông phải gập thêm (tới −0,42 rad ở
  pitch 0,6 rad). Hông đích được tính bằng chia đôi trên mô hình chân với pitch đo được.
- **Nới giới hạn ngã vài giây sau bàn giao.** Lúc bàn giao thân còn chúi 0,6 rad, vượt giới hạn ngã 0,60 rad của V3, nên trong
  `grace_s = 4 s` giới hạn là 1,0 rad để các tầng V3 tự đưa thân về ngang (đo: thân về ngang trong ~2,5 s).
- **Bánh không cần đẩy.** Quét 16 tổ hợp đẩy/không đẩy và gối khi bánh bị hãm đều kẹt; khi bánh lăn tự do thì đẩy không cần thiết
  (`wheel_push_rad_s = 0`).

## 4. Mô-men dùng thật (đo trong mô phỏng)

**Khớp chân** (giới hạn 3,76 N·m = 8 A × 0,47): hông đỉnh 0,62 N·m, gối đỉnh 1,32 N·m (2,8 A) lúc bắt đầu duỗi gối sau bàn giao; trong
pha đứng dậy gối đỉnh 0,93 và hông 0,62 N·m, nghĩa là chân còn dư rất nhiều.

**Bánh** (giới hạn 0,13 N·m mỗi bánh): trong pha gập hông bánh gần như đứng yên (tốc độ đỉnh 0,06 rad/s); servo tốc độ của bánh
giữ nó theo tốc độ đo (trễ), nên mô-men không bằng 0: trung bình 0,078 N·m, chạm trần 0,13 N·m trong 25% thời gian. Sau bàn giao
V3 dùng đỉnh 0,13 N·m ngay lúc bắt (3,5 s) rồi trung bình 0,06 N·m khi đưa thân về ngang và 0,05 N·m khi duỗi gối.

**Bánh cần tối thiểu bao nhiêu** (hạ trần mô-men bánh bằng `--wheel-torque`, đứng dậy rồi lái 0,3 m/s):

| Trần mô-men bánh | Bánh danh định (50/75 ms) | Bánh chậm 70/110 ms |
| --- | --- | --- |
| 0,13 N·m (thật) | đứng được | đứng được |
| 0,10 N·m | đứng được | đứng được |
| 0,08 N·m | đứng được | **ngã** |
| 0,075 N·m trở xuống | **không đứng** (ngã hoặc kẹt) | không đứng |

Dự trữ so với 0,13 N·m chỉ khoảng 1,3 lần với bánh chậm và 1,6 lần với bánh danh định. Cơ chế hỏng ở mô-men thấp: sau bàn giao
V3 không giữ nổi con lắc đảo thân chúi 0,6 rad để đưa về ngang.

## 5. Kết quả mô phỏng (bánh 0,13 N·m)

Bàn giao lúc 3,3 – 4,0 s và đứng thẳng (gối 1,69, thân ngang |pitch| < 0,002) trong cả 16 ca đã thử: lệnh lệch thời điểm 0,2 – 0,7 s;
trễ/lag bánh 20/30, 30/50, 35/70, 50/80, 70/110 ms; bật roll; địa hình gồ ghề 17,7° có và không có roll. Sau đó lái 0,3 m/s (đi
1,26 – 1,30 m trong 4 s) và rẽ 0,8 m/s đều ổn. ctest `gim_standup` (8 ca) và `v3_standup_tests` cố định các điều đó.

## 6. Cách dùng (mô phỏng)

```
TAP_SIM_START=home TAP_SIM_MODEL=src/tap_sim/model/gim_rough.xml python3 tools/tap_native_gui.py   # rồi nút "Đứng dậy (từ nằm)"
build/native/bin/tap_mujoco_gim_v3 --model src/tap_sim/model/gim_dynamic.xml --start-key home --at 0.2:STANDUP --at 22:"DRIVE 0.3 0"
```

`--standup key=val,...` đổi tham số (xem `applyStandUpSpec` trong `gim_v3_runner.cpp`: `rate1`, `mid`, `rate2`, `goal`, `knee`,
`wfree`, `push`, `catch_lean`, `grace`...). Lệnh `STANDUP` thay cho `ENABLE`. Trong cấu hình runner `standup.enabled = true`; trong
`V3Controller::Config` mặc định là tắt, firmware chưa có lệnh này.

## 7. Giới hạn đã biết, việc phải làm trước khi thử trên robot thật

1. **Gốc khớp.** Driver đếm góc từ 0 tại lúc cấp nguồn. Quy trình hiện tại coi tư thế duỗi chân lúc cấp nguồn là 0; để đứng dậy từ
   nằm, firmware cần biết hông đang ở 1,3055 rad (gối 0) tại `home`. Dáng nằm phải được xếp lặp lại được khi cấp nguồn (cùng vấn đề
   "không có thước vật lý" đã bàn); sai lệch gốc vài độ làm lệch điểm bàn giao.
2. **Bù trọng lực của runner** tính mô-men đỡ thân như khi chân gánh toàn bộ trọng lượng; ở dáng nằm bụng gánh phần lớn. Chưa kiểm
   firmware thật làm gì ở dáng này.
3. **Ma sát bụng–sàn** (hệ số 1,0 mặc định của MuJoCo) và hình dạng vỏ bụng chưa đo trên robot.
4. Chưa có: lệnh `STANDUP` trong firmware (`AppV3`), xử lý khi thất bại (nằm lại, thử lại), đứng dậy từ nằm ngửa hoặc nghiêng.
