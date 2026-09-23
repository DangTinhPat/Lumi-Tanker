# TAP RB R1 native simulator

Đây là runtime native MuJoCo cho robot RB R1. Controller core C++17, model
MJCF và GUI native là các thành phần chính.

Model chính:

- MJCF: [`src/tap_sim/model/tap_robot.xml`](src/tap_sim/model/tap_robot.xml)
- CAD manifest: [`src/tap_sim/model/rb_cad_manifest.json`](src/tap_sim/model/rb_cad_manifest.json)
- Physics/inertia: [`src/tap_sim/model/PHYSICS.md`](src/tap_sim/model/PHYSICS.md)
- STEP nguồn: [`LAP RAP ROBOT R1 ver 170926.stp`](LAP%20RAP%20ROBOT%20R1%20ver%20170926.stp)

Model dùng 7 link, free root, 6 hinge joint theo thứ tự `[LH, LK, LW, RH,
RK, RW]`, tất cả trục native `+Y`, STL link-local chi tiết, collision
primitive ổn định, một MPU6500-like IMU và sáu encoder.

Build và kiểm tra:

```bash
cmake -S . -B build/native -DBUILD_TESTING=ON
cmake --build build/native -j2
ctest --test-dir build/native --output-on-failure
python3 src/tap_sim/scripts/validate_model.py \
  --model src/tap_sim/model/tap_robot.xml
```

Chạy native runner:

```bash
build/native/bin/tap_mujoco_native \
  --model src/tap_sim/model/tap_robot.xml --run-for 5
```

GUI native:

```bash
python3 tools/tap_native_gui.py --rate 3000
```

BOM RB là `4.621 kg`: body `3.371 kg`, mỗi thigh `0.296 kg`, mỗi shin
`0.109 kg`, mỗi wheel/motor `0.220 kg`. Bánh trong STEP có envelope khoảng
`40.27 mm`, nên model native dùng bán kính nominal `0.040 m`. COM/inertia là
ước lượng uniform-density từ CAD, chưa phải phép đo động lực học phần cứng.

Mesh body được chia thành bốn STL asset vì MuJoCo giới hạn 200.000 triangle
trên một mesh; toàn bộ `638.359` mặt CAD vẫn được giữ lại.

Các mức bằng chứng cần phân biệt: kiểm tra source/model, build, unit/contract
test, hiển thị GUI và closed-loop simulation. Không mức nào trong số đó tự
động chứng nhận khả năng robot thật.
