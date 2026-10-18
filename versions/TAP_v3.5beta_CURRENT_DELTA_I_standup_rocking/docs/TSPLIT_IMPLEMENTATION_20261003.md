# Tách đặt dáng CoM/gimbal và phản xạ — 03/10/2026

> **Báo cáo lịch sử (03/10).** Prototype chỉ có trong CMake host (`TAP_SPLIT_HOST_PROTOTYPE`); cổng A không đạt. Các cấu hình chạy ở đây có VMC bật (đã gỡ ngày 04/10). Tóm tắt hiện hành: `docs/V3.3_HANDOFF.md` mục 5.

## Trạng thái bàn giao

**Dừng tại cổng lớp A. Không tích hợp vào ESP32, không nạp, không chạy robot,
không commit.** Có công cụ GĐ 0, đường cũ được giữ và kiểm chứng GĐ 1,
prototype đặt dáng A ở host. B chưa được triển khai sau khi cổng A thất bại.
GĐ 3–6 và V35/V36 chưa được tuyên bố hoàn thành.

Các sửa đổi chưa commit có sẵn được giữ nguyên. Không sửa `docs/RULES.md`,
`architecture.md`, CAD, mô hình vật lý, dấu driver, giới hạn ngã hay LQR bánh.
Không sửa `firmware_esp32/src/AppV3.cpp` trong lượt này.

## Mục tiêu đúng theo yêu cầu đã sửa

A chủ động đặt tư thế chân để CoM phù hợp với axle và thân giữ góc gimbal.
Không yêu cầu chân bị khóa: chân phải hội tụ về nghiệm khi đứng yên, và
dịch chuyển theo kế hoạch khi cần. B chỉ là phản xạ tạm thời. Tốc độ tiến/lùi
cần encoder bánh; IMU không trực tiếp đo vận tốc chạy đều hay vị trí CoM.

Khung vẫn là CAD +Y hai bên, góc hip/knee core so với zero cấp nguồn,
`hip_CAD=hangHipCad()+hip_offset`, knee core là knee CAD. Không dùng pose,
contact, COM hay lực thật của plant làm đầu vào controller. Dữ liệu đó
chỉ được dùng để chấm chiều cao, tư thế và dao động.

## GĐ 0 — bằng chứng

`tools/crouch_stop_check.py` đọc log robot hoặc CSV plant, tìm các ứng viên
dừng chiều cao và đo 3 s sau dừng: tvr pp/RMS, hông pp, tốc độ tăng biên độ
nửa chu kỳ, tần số. Time của robot lấy từ `TR t` (ms), không dùng thời gian
máy ghi. Tách đoạn khi lỗi, reset hoặc mất mẫu; cửa sổ thiếu vẫn được giữ
và ghi `complete=false`. Phân loại dựa vào chiều cao chỉ là ứng viên, không
chứng minh không có tay người hay đó là một thao tác có kế hoạch.

Năm log 03/10 có bốn ứng viên dừng ở 92/95/96.4/98.2 mm với tvr RMS
1.342/0.920/1.271/1.502 rad/s. Cả bốn thiếu cửa sổ 3 s vì lỗi/ngắt đoạn.
Không bỏ các ca này và không tính chúng là ca đứng yên đạt.

Pool ngưỡng tạm dùng cửa sổ dài, chiều cao ít đổi, lean nhỏ và không DRIVE.
Ba trong năm log không có pool đủ điều kiện. Hai log còn lại cho ngưỡng
`1.5*p99(|tvr|)` khoảng 0.699/0.815 rad/s. Không chứng minh loại hết tác
động tay; chưa dùng các ngưỡng này để bật B. Không có ngưỡng gia tốc CoM
đã kiểm chứng từ dữ liệu này.

`TIER_STOP` mới: hạ từ 150 mm sau 10 s tới 84/90/95/100/110 mm,
rate 20 mm/s; có tùy chọn hãm giữa chừng bằng `heightStopTarget()`.
Sáu biến thể: danh định, hông mềm, hông mềm + trễ bánh thêm 20 ms, chỉ
thêm trễ bánh, nhiễu tốc độ khớp, lệch zero hip/knee. Plant mềm có rotor
riêng, stiffness hông 20 N·m/rad/khớp và khe hở 0.001 rad; đây là giả định.

90 ca nền = 5 độ cao × 6 plant × 3 luật: hiện tại, chỉ tầng bánh dùng
con lắc ảo, và cấu hình legacy. Legacy là phục dựng các setting, không phải
toàn bộ commit b4. Chỉ tầng bánh vẫn bật virtual pitch, **không phải** replay
firmware v3.2 trên robot. Cả ba dùng cùng VMC/roll và tốc độ khớp thấp 0.7.
Roll là proxy bên cạnh plant dọc, không phải mô phỏng tiếp xúc 3D.

90 ca không báo ngã, nhưng có ca dao động hoặc không đạt chiều cao.
Biến thể chỉ thêm trễ bánh tái hiện luật hiện tại rung hơn chỉ tầng bánh:
ở 84 mm tvr RMS cuối bài 0.131 so với 0.089 rad/s, chiều cao thật đều gần
84 mm. Chưa tái hiện đầy đủ cú ngã sau dừng như robot thật. Không dùng các
ca không xuống được chiều cao để tuyên bố một luật tốt hơn ở tư thế thấp.

## GĐ 1 — đường cũ

Giữ nguyên thân hàm `updateTrunk`; wrapper `updateLegLayers` gọi đúng đường
cũ khi `Split::enabled=false`. `Config::Split` mặc định false.
`TAP_SPLIT_HOST_PROTOTYPE` chỉ được CMake host định nghĩa; không có trong
PlatformIO. Không có `SET tsplit` trên app ESP32 lúc này.

- Replay 30.000 tick cùng input qua thư viện nền đã lưu và thư viện mới:
  mọi trường số/bool của `Output`, chuỗi fault và các trạng thái quan sát
  đã ghi đều khớp; float xuất hex giữ đầy đủ bit, bao gồm signed zero.
- Bảy ca plant (đứng cao/thấp, drive/brake, đẩy, held, hông mềm, nhiễu):
  mọi trường CSV cũ và stdout khớp chính xác bản nền.
- CTest nền: v3_tests, v3_physics_tests, v3_coordination_tests,
  leg_height_tests đạt; v3_tier_tests dừng tại V15 held+preview.
- CTest mới có cùng lỗi V15; test split riêng đạt. Không nới assertion.
  V24/V32 nằm sau điểm dừng nên không tuyên bố chúng đã chạy/đạt/thất bại.

Đây là parity trên các input đã chạy, không phải chứng minh hình thức cho
mọi input hoặc mọi bit nội bộ không quan sát được.

## GĐ 2 — prototype A

- `splitModelPosture()` giải mục tiêu comLine từ trim bánh, góc gimbal
  và chiều cao, qua IK hiện tại; không chuyển toàn bộ trim sang hông.
- `updatePosture()` thêm hiệu chỉnh pitch thân chậm, tau ít nhất 1.5 s,
  giới hạn tốc độ 0.1 rad/s và giới hạn biên độ. Không lọc trễ tư thế thêm
  một lần nữa. A chạy khi balancing hợp lệ, không chờ enable delay 2 s
  của phản xạ cũ.
- Trim và learned trim vẫn do đường LQR hiện tại sở hữu. A không thêm
  tích phân sai số CoM song song. Tích phân body-level đóng băng khi held,
  bánh gần bão hòa, B có quyền hoặc workspace hết theo chiều sai số;
  vẫn cho phép tháo tích phân khi sai số đảo chiều.
- Preview kế hoạch riêng, B không hoạt động. Các trường dự trữ cho B trong
  Split chưa dùng; `updateShock` chỉ xóa B về zero, không phải detector.
- CSV host thêm hA/hB/aB/preview/modelA và trạng thái planner. Không đổi
  dòng TR của firmware, không đổi dấu joint và không thêm cảm biến.

Kết quả tích cực: TIER_ZERO đưa thân về vùng mục tiêu trong tối đa 3.5 s.
Với plant danh định, thống kê cuối 4 s của TIER_STOP:

| Chiều cao yêu cầu | Lệnh hông pp cũ | Lệnh hông pp A | Chiều cao thật A |
|---|---:|---:|---:|
| 84 mm | 0.0102 rad | 0.0017 rad | 83.8 mm |
| 90 mm | 0.0084 rad | 0.0017 rad | 89.8 mm |
| 95 mm | 0.0102 rad | 0.0011 rad | 94.8 mm |
| 100 mm | 0.0102 rad | 0.0011 rad | 99.8 mm |
| 110 mm | 0.0127 rad | 0.0026 rad | 109.7 mm |

Lệnh hông pp giảm khoảng 80–89%. Góc hông thật của plant giảm ít hơn:
84/90/95/100/110 mm lần lượt 0.0040/0.0043/0.0041/0.0057/0.0072 rad
thành 0.0031/0.0032/0.0026/0.0023/0.0036 rad, giảm khoảng 22–60%.
Vì thế cổng giảm rung hông thật ≥50% cũng chưa đạt ở mọi độ cao.
Tvr RMS không giảm ở mọi ca: tại 95 mm
danh định 0.044 -> 0.053 rad/s. Không đánh đồng chân ít vung với toàn hệ êm hơn.

**Cổng A KHÔNG ĐẠT:** với hông mềm + trễ bánh, cả năm chiều cao yêu cầu
đều không đạt; A còn ở khoảng 178 mm, tvr RMS khoảng 0.854 rad/s.
Tắt nhánh động đã làm plant này kém đi; cần giải quyết độ tin cậy/biên ổn
định trước khi lấy B để che vấn đề. 30 ca A và 6 ca hãm được lưu riêng.
Dung sai chấm chiều cao thực tế là 5 mm, không chỉ xét planner tới đích.

Không chứng nhận đủ 78–189 mm. Ở đỉnh, đường hiện tại còn chừa 12 mm khi
cần workspace gimbal/roll; không thể vừa dùng khớp duỗi hết vừa mặc định
có hành trình điều chỉnh trước/sau ở cùng chiều cao.

## Cổng dừng và bước còn lại

Dừng trước GĐ 3 theo nguyên tắc người dùng: không qua cổng thì không đưa
vào firmware. Chưa có detector B, lịch attack/hold/release, V35/V36,
ma trận tổng GĐ 4, SET/TUNE/TR ESP32, hai build PlatformIO của split, hoặc
buổi robot. Không cập nhật firmware hiện có trên thiết bị.

Bước quyết định tiếp theo là kiểm mô hình truyền động/pha phản hồi ở plant
thất bại và quan hệ bandwidth của A với LQR. Không tăng gain/lead chỉ để
đạt một plant. Khi A đạt cùng operating point và không làm xấu bánh/thân,
mới chuyển sang B có detector chống tự kích và tái kích.

Tài liệu này là checkpoint để phiên sau tiếp tục; không có file memory
chuyên biệt được tìm thấy trong repo. Tham khảo task allocation wheel-legged:
https://doi.org/10.1109/iros51168.2021.9636371 . Không sao chép gain robot khác.

## Chạy lại

```bash
cmake -S . -B build/tsplit-host -DTAP_BUILD_SIMULATOR=OFF \
  -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build/tsplit-host --target v3_tier_tests v3_split_tests \
  v3_split_replay v3_tests v3_physics_tests v3_coordination_tests leg_height_tests -j2
python3 -m unittest discover -s tools/tests -p test_crouch_stop_check.py
python3 tools/tsplit_stop_matrix.py --runner build/tsplit-host/bin/v3_tier_tests \
  --out log/tsplit_replay --split 1 --modes 0
```

`TAP_BUILD_SIMULATOR` mặc định ON. OFF chỉ để chạy core/firmware host khi
Python hiện tại không có MuJoCo; không thay runtime mặc định.
Ma trận và gate: `log/tsplit_20261003/gates.json`, `stage0_full/summary.json`,
`stageA_checked/summary.json`, `stageA_checked_brake/summary.json`.
Bản nền đọc-only: `/tmp/tap-tsplit-baseline-20261003`.
Replay hex, SHA256 nguồn, log CTest và log robot đã chấm nằm cùng thư mục log.
Các kết quả plant được tạo từ build host `/tmp/tap-rest-host-20261003/build`.
