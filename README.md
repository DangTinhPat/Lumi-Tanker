# Lumi-Tanker — TAP RB R1

Robot hai bánh chân (wheel-legged) RB R1. Repo gồm hai phần dùng chung một
lõi C++17:

1. **Firmware v3.3 cho ESP32-S3** (điều khiển robot thật): bánh cân bằng con
   lắc ngược bằng LQR, hông và gối giữ thân ngang và hấp thụ nhiễu loạn. Đây là
   thứ chạy trên robot.
2. **Mô phỏng MuJoCo native + GUI** để quan sát và thử nghiệm. Mô phỏng này chạy
   `EmbeddedController` cũ, **không** chạy `V3Controller` của firmware; firmware
   được mô phỏng bằng plant phẳng hai vật thể trong các test host.

Bản đồ thư mục, ranh giới giữa hai phần và hợp đồng cảm biến nằm ở
[`architecture.md`](architecture.md). Quy tắc bắt buộc nằm ở
[`docs/RULES.md`](docs/RULES.md). Trạng thái hiện tại của firmware (cấu hình
boot, số đo phần cứng, hướng đã loại, việc còn mở) nằm ở
[`docs/V3.3_HANDOFF.md`](docs/V3.3_HANDOFF.md).

## Phần cứng

- Board ESP32-S3-WROOM-1 N16R8 tuỳ biến, một MPU6500 (SPI).
- Bốn khớp chân: SteadyWin GDS68 (GIM6010-8, hộp số 8:1) trên CAN 500 kbit/s.
- Hai bánh: CJC BL4820M12 trên UART 1 Mbaud.
- Cảm biến triển khai được: một MPU6500 và sáu encoder động cơ. Dữ liệu mô
  phỏng (pose, tiếp xúc, lực ràng buộc) không bao giờ đi vào quyết định điều
  khiển.

## Quy ước robot

- Thứ tự khớp `[LH, LK, LW, RH, RK, RW]`; mọi trục native là `+Y` ở cả hai bên,
  gốc khớp là tư thế CAD (firmware lấy tư thế lúc cấp nguồn làm gốc).
- Bán kính bánh `0.040 m`, khoảng cách tâm trục `0.309 m` (nửa track
  `0.1545 m`).
- Khối lượng `4.621 kg`: thân `3.371`, mỗi đùi `0.296`, mỗi cẳng `0.109`, mỗi
  bánh cùng động cơ `0.220`. Khối lượng, trọng tâm và quán tính là ước lượng
  từ CAD/BOM, chưa phải phép đo động lực học (xem
  [`src/tap_sim/model/PHYSICS.md`](src/tap_sim/model/PHYSICS.md)).
- Chiều cao là khoảng cách hông–trục bánh: 188,6 mm (chân buông) xuống 78 mm.

## Build và kiểm tra

Test host (không cần MuJoCo):

```bash
cmake -S . -B build/native -DBUILD_TESTING=ON -DTAP_BUILD_SIMULATOR=OFF
cmake --build build/native -j4
ctest --test-dir build/native --output-on-failure
```

`ctest` hiện có 5 lỗi có từ trước (`core_allocation_tests`, `jump_tests`,
`jump_quality_tests`, `lqr_tests`, `v3_tier_tests` dừng ở assertion V14); chi
tiết ở HANDOFF. Các bài sim của firmware chọn bằng biến môi trường, ví dụ
`TIER_ACCEL=1 build/native/bin/v3_tier_tests`.

Firmware:

```bash
~/.platformio/penv/bin/pio run -e esp32-s3-v3          # build
~/.platformio/penv/bin/pio run -e esp32-s3-v3 -t upload --upload-port /dev/ttyUSB0
```

Chỉ nạp firmware hoặc gửi lệnh động cơ khi bạn chủ động yêu cầu. Đóng GUI trước
khi nạp; cổng hay đổi giữa `ttyUSB0` và `ttyUSB1`. Môi trường dự phòng
`esp32-s3-v3-single-loop` dùng vòng lặp đơn thay cho task FreeRTOS.

## Điều khiển robot và log

```bash
python3 tools/tap_serial_control_gui.py --port /dev/ttyUSB0
```

GUI gửi `START`, `GO`, `DRIVE`, `HMOVE`, `DISABLE`, `ESTOP`, `SET`, `GET`, `REC`
qua cổng nối tiếp (921600 baud) và ghi mọi dòng vào `~/tap_logs/`. Mở cổng
nối tiếp làm ESP32 reset và mất mọi `SET` đang chạy. Phân tích log:
`tools/tr_rest_stats.py`, `tools/hip_chatter_stats.py`,
`tools/hf_vibration_stats.py`. Tham chiếu lệnh, khoá `SET` và trường log:
[`src_fw/FIRMWARE_V3.md`](src_fw/FIRMWARE_V3.md).

## Mô phỏng MuJoCo native

Cần MuJoCo 3 (`pip install mujoco` hoặc đặt `MUJOCO_ROOT`):

```bash
cmake -S . -B build/native -DBUILD_TESTING=ON
cmake --build build/native -j4
python3 src/tap_sim/scripts/validate_model.py --model src/tap_sim/model/tap_robot.xml
build/native/bin/tap_mujoco_native --model src/tap_sim/model/tap_robot.xml --run-for 5
python3 tools/tap_native_gui.py --rate 3000
```

Model: [`tap_robot.xml`](src/tap_sim/model/tap_robot.xml), CAD manifest
[`rb_cad_manifest.json`](src/tap_sim/model/rb_cad_manifest.json); 7 link, root
tự do, 6 hinge, STL link-local chi tiết tách khỏi collision primitive ổn định,
một IMU kiểu MPU6500 và sáu encoder. Mesh thân chia thành bốn STL vì MuJoCo
giới hạn 200.000 tam giác mỗi mesh; toàn bộ 638.359 mặt CAD vẫn được giữ.
Hướng dẫn runtime: [`docs/NATIVE_RUNTIME.md`](docs/NATIVE_RUNTIME.md).

## Theo người (tuỳ chọn)

`blue_object_vision/` phát hiện người bằng YOLOv8n (ONNX) rồi theo dõi; nút
bật trong GUI nối tiếp gửi `DRIVE`/`STOP` qua cùng đường nối tiếp. Nó không
nằm trong vòng cân bằng. Xem [`blue_object_vision/README.md`](blue_object_vision/README.md).

## Mức bằng chứng

Phải báo cáo riêng từng mức: kiểm tra source/model, build, unit/contract test,
sim phẳng của firmware, sim MuJoCo/GUI, log robot thật. Không mức nào tự chứng
nhận khả năng của robot thật; hiển thị GUI đúng cũng không chứng minh ước
lượng hay cân bằng.
