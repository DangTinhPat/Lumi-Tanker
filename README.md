# Lumi-Tanker — TAP RB R1

Robot hai bánh chân (wheel-legged) RB R1. Repo gồm hai phần dùng chung một
lõi C++17:

1. **Firmware V3 cho ESP32-S3** (điều khiển robot thật, nhánh `v3.7`): bánh cân
   bằng con lắc ngược bằng LQR, hông và gối giữ thân ngang và hấp thụ nhiễu
   loạn; có đứng dậy từ tư thế nằm và bước nhảy `HopSequence`. Đây là thứ chạy
   trên robot.
2. **Mô phỏng MuJoCo native + GUI** để quan sát và thử nghiệm. Runner
   `tap_mujoco_gim_v3` chạy chính `V3Controller` của firmware trên model CAD
   `GIM6010-8` (`gim_dynamic.xml`). Runner cũ `tap_mujoco_native` chạy
   `EmbeddedController` và model RB R1 cũ; nó không còn được dùng (xem mục Mô
   phỏng).

Một kết quả mô phỏng chỉ là bằng chứng cho model và actuator giả lập mà nó dùng,
không phải bằng chứng về khả năng của robot thật.

Bản đồ thư mục, ranh giới giữa hai phần và hợp đồng cảm biến nằm ở
[`architecture.md`](architecture.md). Quy tắc bắt buộc nằm ở
[`docs/RULES.md`](docs/RULES.md). Trạng thái hiện tại của firmware (cấu hình
boot, số đo phần cứng, hướng đã loại, việc còn mở) nằm ở
[`docs/V3.3_HANDOFF.md`](docs/V3.3_HANDOFF.md).

## Bố cục thư mục

```text
controller/          Thuật toán điều khiển và test C++ trên host
  v3/                V3Controller, LegLock, HopSequence, StandUp, giao tiếp motor, test firmware
  core/              Lõi điều khiển dùng chung, bộ ước lượng
  hal/               Giao diện trừu tượng phần cứng (mapping motor, codec khung)
  examples/          Chương trình ví dụ
firmware/esp32/      Điểm vào ESP32, driver board (CAN, UART, MPU6500), cấu hình build
gui/                 GUI robot, GUI mô phỏng, thư viện Python và test
simulation/          MuJoCo runner, model CAD GIM, script và test mô phỏng
assets/              CAD (STEP) và datasheet động cơ
tools/               Công cụ build model, phân tích log, chẩn đoán
tests/               Test tích hợp cấp repository
docs/                Tài liệu thiết kế và hướng dẫn
```

Chi tiết và bảng đối chiếu đường dẫn cũ: [PROJECT_LAYOUT.md](docs/PROJECT_LAYOUT.md).
`architecture.md` và các tài liệu lịch sử giữ nguyên nội dung nên có thể còn
đường dẫn theo bố cục cũ. Vision đã được gỡ khỏi dự án.

## Phần cứng

- Board ESP32-S3-WROOM-1 N16R8 tuỳ biến, một MPU6500 (SPI).
- Bốn khớp chân: SteadyWin GDS68 (GIM6010-8, hộp số 8:1) trên CAN 500 kbit/s.
  Lệnh vị trí (chế độ POS) gửi theo vòng rotor, tức đã nhân tỉ số 8
  (`controller/v3/src/v3/LegLock.cpp`).
- Hai bánh: Kpower BL4012, mỗi bánh một bus UART một dây (half-duplex),
  1 Mbaud; bánh trái ID 2 (GPIO15), bánh phải ID 1 (GPIO17), xem
  `controller/hal/include/robot_hal/MotorMapping.hpp`. HANDOFF còn nhiều ghi chú
  theo BL4820 cũ; bánh hiện tại là BL4012.
- Cảm biến triển khai được: một MPU6500 và sáu encoder động cơ. Dữ liệu mô
  phỏng (pose, tiếp xúc, lực ràng buộc) không bao giờ đi vào quyết định điều
  khiển.

## Model CAD đang dùng

Nguồn hình học là [`assets/cad/ROBOT (GIM6010-8).STEP`](assets/cad) (59
occurrence, SHA256
`81c3dfe090b4341f626831b87c8fc556116255c740ac562b5a866b49be00343b`). Model cũ RB
R1 (`tap_robot.xml`, `rb_cad_manifest.json`, `mesh_inertials.yaml`) không còn
là model của robot.

| Tệp (trong `simulation/model/`) | Vai trò |
| --- | --- |
| `gim_dynamic.xml` | Model động lực học mà GUI và `tap_mujoco_gim_v3` nạp: root tự do và 6 hinge (`hip_*`, `knee_*`, `wheel_*`); hông/gối có ma sát khô 0,1 N·m, gối giới hạn 0–1,9722 rad |
| `gim_visual.xml`, `gim_visual/` | Model chỉ để xem |
| `gim_components.xml`, `gim_components.json`, `gim_components/` | 59 STL theo từng chi tiết CAD |
| `gim_mass_properties.yaml` | Khối lượng, ma sát, trục khớp và giới hạn gối do người vận hành cung cấp |
| `gim_dynamic_shell.xml` | Biến thể `gim6010_v3_shell_contact` |

Số liệu chính (từ `gim_mass_properties.yaml` và
[`docs/GIM_COM_20261018.md`](docs/GIM_COM_20261018.md)):

- Thứ tự khớp `[LH, LK, LW, RH, RK, RW]`; mọi trục native là `+Y` ở cả hai bên
  (trái `+Y`, phải `-Y` về vị trí). Gốc khớp là tư thế CAD; firmware lấy tư thế
  lúc cấp nguồn làm gốc và cộng mốc hông/gối của khung chân thẳng.
- Bán kính bánh `0.040 m`, khoảng cách tâm trục `0.2654 m`
  (`controller/core/.../GeneratedGeometry.hpp`).
- Khối lượng `3.232 kg`: cụm thân `2.612` (bốn motor khớp 1,6; pin 0,25 ở khoang
  trước; khung 0,35; PCB/dây/ốc 0,412), mỗi đùi `0.05`, mỗi ống cẳng `0.04`,
  mỗi motor bánh `0.20`, mỗi bánh `0.02`. Tất cả là khối lượng gán, chưa cân; COM
  thân ước lượng x = +5,3 mm, z = 15,1 mm.
- Gối: hông quay tự do; gối giới hạn góc trong của khớp 22°–135°. Trong
  controller, góc gối tính từ tư thế gối co (22°) nên dải điều khiển là
  `0 → 1,9722 rad` (0 → 113°). Chân thẳng hết cỡ cho chiều cao hông–trục bánh
  184,8 mm; ở gối 22° hình học cho 38 mm (dải làm việc của controller hẹp hơn,
  xem HANDOFF).
- Ma sát bánh–đất 0,8; ma sát khô mỗi khớp hông/gối 0,1 N·m.

## Build và kiểm tra

Test host (không cần MuJoCo):

```bash
cmake -S . -B build/host -DBUILD_TESTING=ON -DTAP_BUILD_SIMULATOR=OFF
cmake --build build/host -j4
ctest --test-dir build/host --output-on-failure
```

Một số test lỗi từ trước, không do thay đổi gần đây; danh sách và nguyên nhân ở
HANDOFF (cập nhật lần cuối ngày 08/10/2026, nên hãy chạy `ctest` để biết trạng
thái hiện tại). Các bài sim của firmware chọn bằng biến môi trường, ví dụ
`TIER_ACCEL=1 build/host/bin/v3_tier_tests`.

Firmware:

```bash
~/.platformio/penv/bin/pio run -e esp32-s3-v3-1840-roll  # build mặc định hiện tại
~/.platformio/penv/bin/pio run -e esp32-s3-v3-1840-roll -t upload --upload-port /dev/ttyUSB0
```

Chỉ nạp firmware hoặc gửi lệnh động cơ khi bạn chủ động yêu cầu. Đóng GUI trước
khi nạp; cổng hay đổi giữa `ttyUSB0` và `ttyUSB1`. Các môi trường khác trong
`platformio.ini`: `esp32-s3-v3` (gốc), `-single-loop` (vòng lặp đơn thay cho
task FreeRTOS), `-ap`, `-nowifi`, `-ota`, `-1840-addwifi`, `-1840-hip`.

## Điều khiển robot và log

```bash
python3 gui/robot_gui.py --port tcp://10.10.60.68:8023
# Hoặc USB: python3 gui/robot_gui.py --port /dev/ttyUSB0
```

GUI gửi `START`, `GO`, `DRIVE`, `HMOVE`, `MOVE`, `TURN`, `DISABLE`, `ESTOP`,
`SET`, `GET`, `REC` qua cổng nối tiếp (921600 baud) hoặc TCP và ghi mọi dòng vào
`~/tap_logs/`. Mở cổng nối tiếp làm ESP32 reset và mất mọi `SET` đang chạy.
Phân tích log: `tools/tr_rest_stats.py`, `tools/hip_chatter_stats.py`,
`tools/hf_vibration_stats.py`. Tham chiếu lệnh, khoá `SET` và trường log:
[`controller/v3/FIRMWARE_V3.md`](controller/v3/FIRMWARE_V3.md).

## Mô phỏng MuJoCo native

Cần MuJoCo 3 (`pip install mujoco` hoặc đặt `MUJOCO_ROOT`):

```bash
cmake -S . -B build/simulation -DBUILD_TESTING=ON -DTAP_BUILD_SIMULATOR=ON
cmake --build build/simulation -j4
build/simulation/bin/tap_mujoco_gim_v3 --model simulation/model/gim_dynamic.xml --run-for 5
python3 gui/simulator_gui.py --rate 3000
```

GUI tìm `tap_mujoco_gim_v3` trong `build/simulation/bin`, `build/native/bin` hoặc
`build/bin`, và nạp `simulation/model/gim_dynamic.xml`; đặt `TAP_SIM_MODEL=<xml>`
để dùng model khác (ví dụ địa hình gồ ghề từ `tools/build_rough_terrain.py`).
Runner có thêm tuỳ chọn `--wheel-delay`, `--wheel-lag`, `--sensor-noise`,
`--control`, `--realtime`. Hướng dẫn runtime: [`docs/NATIVE_RUNTIME.md`](docs/NATIVE_RUNTIME.md)
(tài liệu đó còn mô tả runner cũ).

Lưu ý còn tồn tại từ model cũ:

- `tap_mujoco_native` và các bài `simulation/tests/native_*_test.py` trong
  `simulation/CMakeLists.txt` vẫn trỏ tới `tap_robot.xml`. Model này cần các STL
  `body_detail_*.stl` mà repo không còn, nên nó không nạp được. Đừng dùng nó
  làm bằng chứng cho robot hiện tại.
- Mô-men bánh mô phỏng bị chặn ở ±0,13 N·m theo `ctrlrange` của `gim_dynamic.xml`
  (runner có chế độ what-if để mở rộng). Đây là giới hạn của model, chưa được đối
  chiếu với BL4012 trên robot.

## Mức bằng chứng

Phải báo cáo riêng từng mức: kiểm tra source/model, build, unit/contract test,
sim phẳng của firmware, sim MuJoCo/GUI, log robot thật. Không mức nào tự chứng
nhận khả năng của robot thật; hiển thị GUI đúng cũng không chứng minh ước
lượng hay cân bằng.
