# Đặc tả vật lý mô phỏng GIM6010-8

Ngày cập nhật: 2026-10-08. Đặc tả này mô tả **model đang thực thi**, không
khẳng định những thông số chưa đo đã được xác định chính xác trên robot.

Nguồn model: `ROBOT (GIM6010-8).STEP`, `gim_components.json`,
`gim_mass_properties.yaml`, `tools/build_gim_dynamic_model.py`.
Runtime: `src/tap_sim/native/gim_v3_runner.cpp`; controller: `V3Controller`.
Viewer chỉ nhận snapshot, không sở hữu vòng vật lý và không gửi pose về controller.

## 1. Quy ước và mức độ chắc chắn

Đơn vị: m, kg, s, rad, N, N·m, A. Thế giới: +X tiến, +Y trái, +Z lên.
Frame thân trùng frame CAD đã tịnh tiến về tâm bounding box; không đổi dấu
khớp bên phải. Gốc STEP trước tịnh tiến xấp xỉ
`[1370.7624, 648.5787, 964.1983] mm`.

| Đại lượng | Giá trị dùng | Nguồn |
| --- | --- | --- |
| Khối lượng thân | 2,50 kg | Người dùng, làm tròn |
| Khối lượng mỗi đùi | 0,050 kg | Người dùng |
| Khối lượng mỗi ống chân | 0,040 kg | Người dùng |
| Khối lượng mỗi motor bánh | 0,200 kg | Người dùng |
| Khối lượng mỗi bánh riêng | 0,020 kg | Suy ra từ tổng bánh + motor 0,220 kg trước đây |
| Tổng khối lượng | 3,120 kg | Tổng các nhóm trên |
| Bán kính bánh | 0,040 m | Hình học CAD |
| Khoảng cách tâm hai bánh | 0,2654 m; nửa khoảng cách 0,1327 m | CAD GIM |
| Hành trình gối | 22° đến 135° | Người dùng; CAD zero được ánh xạ tới 22° |
| Khớp đùi | Quay liên tục | Người dùng |
| Ma sát bánh–sàn | μ = 0,8 | Người dùng, cập nhật trong phiên này |
| Ma sát khô mỗi đùi/gối | 0,1 N·m | Người dùng, cập nhật trong phiên này |
| COM và tensor quán tính | Bảng mục 3 | Ước lượng hiện tại, chưa tích phân thể tích STEP |
| Ma sát thân/ống chân với sàn, độ mềm tiếp xúc | Mục 4 | Giả định mô phỏng |
| Armature, ma sát ổ bánh, băng thông vòng dòng/tốc độ | Mục 5 | Giả định mô phỏng |

Giá trị track 0,309 m từng có trong tài liệu trước đây không phải track
được dùng trong model GIM này. Encoder zero thực tế chưa hiệu chuẩn.

## 2. Cây khớp, FK và spawn

Thân có free joint 6 bậc tự do; mỗi bên gồm đùi → gối/ống chân → bánh.
Tất cả sáu trục quay là **+Y**. Thứ tự encoder/controller là
`[LH, LK, LW, RH, RK, RW]`; MJCF có `nq=13`, `nv=12`, `nu=6`.
Sáu actuator đặt mô-men vào đúng joint theo tên; thứ tự actuator trong MJCF
là `[LH, LK, RH, RK, LW, RW]`, không được đồng nhất với thứ tự encoder.

Gốc khớp trong frame thân ở CAD zero:

| Khớp | Trái, m | Phải, m |
| --- | --- | --- |
| Đùi | `[0, 0.1025, 0.00828]` | `[0, -0.1025, 0.00828]` |
| Gối | `[-0.09816, 0.1130, -0.01080]` | `[-0.09816, -0.1130, -0.01080]` |
| Bánh | `[0, 0.1327, -0.02988]` | `[0, -0.1327, -0.02988]` |

Trong mặt phẳng XZ, vector đùi `a = [-0.09816, -0.01908] m`, vector
ống chân `b = [0.09816, -0.01908] m`. Khoảng cách giữa các trục song song
đùi–gối và gối–bánh xấp xỉ 100 mm; các tọa độ Y chứa độ lệch ngang của lắp ráp.

`R(q)(x,z) = (cos(q)x + sin(q)z, -sin(q)x + cos(q)z)`.
Vị trí trục bánh tương đối với khớp đùi:
`r_axle = R(q_hip)a + R(q_hip + q_knee)b`.
Góc gối nội bộ `α = 22° + q_knee`; giới hạn MJCF
`q_knee ∈ [0, 1.9722222222] rad`. Đây là ràng buộc giới hạn mềm của solver,
không phải phép clamp tuyệt đối trạng thái q; có thể có sai số xuyên giới hạn nhỏ.

Mỗi spawn và Reset chọn keyframe **`spawn_extended`**:

- Thân thẳng, quaternion `[1,0,0,0]`, x=y=0.
- Hai đùi `−0.986111111 rad = −56,5°`.
- Hai gối `+1.972222222 rad = +113°` từ CAD zero, tương đương 135° nội bộ.
- Thân ở z = `0.216490386 m`; tâm bánh z = `0.040 m`; mọi vận tốc bằng 0.
- V3 có cùng mục tiêu khóa chân ban đầu. MuJoCo giữ fixture qua khởi động controller; xem mục 6.

`cad_neutral` chỉ giữ tư thế lắp ráp gốc, với thân z=0,06988 m và các góc
khớp bằng 0. Nó không phải mặc định spawn.

## 3. Khối lượng, COM và quán tính

Model dùng 7 vật rắn. Thân chứa cả bốn motor đùi/gối, pin, PCB, nhựa, vít và
dây. Mỗi shin chứa 40 g ống chân + 200 g motor bánh. Mỗi wheel chứa 20 g bánh.
Phân bố rotor/stator nội bộ motor chưa được xác định riêng.

COM dưới đây tương đối với frame vật rắn tương ứng. Tensor đang dùng là
đường chéo tại COM, với quaternion inertial bằng identity. Không dịch tensor
về gốc khớp một lần nữa khi khai báo MJCF.

| Vật rắn | m, kg | COM local, m | `(Ixx,Iyy,Izz)`, kg·m² |
| --- | --- | --- | --- |
| Thân | 2,50 | `(-0.0048,0,0.0140)` | `(0.020,0.007,0.020)` |
| Mỗi đùi | 0,050 | `(-0.04908,0,-0.00954)` | `(0.000420,0.0000417,0.000420)` |
| Mỗi shin + motor bánh | 0,240 | `(0.08998,0,-0.01749)` | `(0.001,0.000220,0.001)` |
| Mỗi bánh | 0,020 | `(0,0,0)` | `(0.000008,0.000016,0.000008)` |

Các tensor có trị riêng dương và thỏa bất đẳng thức tam giác quán tính.
Điều này xác nhận tính hợp lệ toán học, không xác nhận phân bố khối lượng thật.
Đặc biệt Ixx/Izz của thân và chân là ước lượng; chưa dùng kết quả roll/ngã 3D
để chứng minh độ chính xác của chúng.

Quán tính quanh trục bánh được tính đúng một lần theo trục song song:
`I_axle = Σ[Iyy_i + m_i((x_i−x_axle)²+(z_i−z_axle)²)]`.
COM là trung bình có trọng số khối lượng của cả 7 vật rắn.

| Tư thế | COM tương đối trục bánh `(x,z)`, m | COM cao trên sàn, m | I quanh trục bánh, kg·m² |
| --- | --- | --- | --- |
| CAD zero | `(-0.006677692,0.036322180)` | 0,076322180 | 0,012782763 |
| Spawn duỗi | `(-0.004950027,0.158262549)` | 0,198262549 | 0,100319790 |

Góc cân bằng tĩnh dự đoán ở spawn khoảng +1,7915° theo trục +Y;
đây là dự đoán từ COM ước lượng, không phải góc đã đo trên robot.
Đã loại phần quán tính mặc định `0.0128 kg·m²` từng bị cộng thêm vào tổng
trong `WholeBodyModel::compute()`; kiểm tra độc lập với MuJoCo bắt được lỗi này.

59 mesh CAD là visual; 28 chi tiết thân/chân cũng dùng làm convex collider.
Không mesh nào tự suy ra khối lượng; bảy inertial link vẫn khai báo riêng.
Việc gán mỗi chi tiết vào một nhóm vật rắn được kiểm tra không trùng/không thiếu
trong generator; STEP không chứa mates xác nhận cây động học này.

## 4. Va chạm và tiếp xúc

Mặt sàn là plane z=0, vô hạn về vật lý. Các collider robot:

| Nhóm | Collider | Kích thước / vị trí local |
| --- | --- | --- |
| Thân | Convex hull các mesh CAD | Vỏ 50/55 và gá hông lộ ngoài 7/8/31/32; giữ nguyên tọa độ CAD |
| Đùi trái/phải | Convex hull từng chi tiết CAD | Trái 9/10/11/12/16; phải 33/34/38/42/43; gồm vỏ quanh gối |
| Shin trái/phải | Convex hull từng chi tiết CAD | Trái 1/2/3/4/5/23; phải 26/27/28/29/30/41; gồm vỏ động cơ bánh |
| Mỗi bánh | Cylinder, trục song song Y | r=0,040 m; full width=0,017 m |

Collider robot có `contype=1`, `conaffinity=1`; visual còn lại có cả hai bằng 0.
MuJoCo áp dụng lọc self-contact mặc định giữa các body; không có constraint
ẩn, dây treo, mocap hoặc equality constraint giữ thân đứng.

Vỏ CAD rộng theo X khoảng 160 mm. Box cũ dài 300 mm đã gây tiếp xúc ảo và
giữ vỏ cách sàn 64 mm ở tư thế ngã. Đã bỏ box đó; convex hull dùng chính mesh
vỏ với các góc bo tròn, gồm cả nắp cao tới z=0,07927949 m trong frame thân.
Đã bỏ cả bốn capsule chân bán kính 10 mm: chúng quá nhỏ và để vỏ quanh gối
xuyên sàn 5,6 mm. Collider chân dùng chung mesh/frame với visual từng chi tiết.
Hull lấp các lỗ/lõm của từng chi tiết, chưa mô phỏng tiếp xúc trong các lỗ đó.

Hai cặp `ground–left_wheel_collision` và `ground–right_wheel_collision` được
khai báo riêng, `condim=3`, friction **`[0.8,0.8,0,0,0]`**.
Như vậy μ=0,8 được dùng đúng cho hai phương tiếp tuyến, không bị hệ số μ=1
mặc định của sàn/thân ghi đè trong quy tắc trộn geom. Xem
[MuJoCo: contact parameters](https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters).

Một hệ số Coulomb được dùng cho cả bám/trượt; chưa có hệ số tĩnh/động riêng
hoặc mô hình Stribeck. `condim=3` không thêm mô-men ma sát xoắn/lăn tại contact.
Ma sát các cặp thân/chân–sàn còn là μ=1,0 **giả định**, không phải μ bánh.

Contact mềm có `solref=[0.006,1]`,
`solimp=[0.95,0.99,0.001,0.5,2]` sau compile. Đây là tham số solver, không
phải độ cứng cao su, hệ số đàn hồi hay số đo vật liệu. Chưa có mô hình biến dạng
lốp, chassis, backlash hoặc độ đàn hồi khớp.

Mỗi khớp đùi/gối dùng `frictionloss=0.1 N·m`, viscous damping=0.
Mô-men ma sát khô chống chuyển động; khi đứng yên, solver có thể cân bằng
mô-men trong giới hạn đó. Nó không phải hệ số nhân với tốc độ. Xem
[MuJoCo: friction loss](https://mujoco.readthedocs.io/en/stable/computation/index.html#friction-loss).

## 5. Động cơ và đường bù dòng I

**Bốn khớp chân:** Kt đầu ra=0,47 N·m/A theo thông số GIM6010-8 trong
`docs/actuatal_driver/GDS68_DRIVER.md`; giới hạn dòng cấu hình=8 A theo
`BoardConfig.hpp`. Không nhân Kt đầu ra với tỉ số truyền 8 thêm lần nữa.
Giới hạn lệnh tại actuator MJCF và runner đều là ±3,76 N·m.
Đây là giới hạn lệnh từ dòng, không phải xác nhận mô-men liên tục ở mọi tốc độ.

V3 đưa mục tiêu vị trí. Vòng trở kháng tạo mô-men:
`τ_req = τ_gravity(q_hip,q_knee) + 12(q_ref−q) + 0.8(qd_ref−qd)`.
Feed-forward trọng lực dùng mô hình thân thẳng và chia tải thân đều hai chân;
đó là xấp xỉ của controller. MuJoCo vẫn tự tính trọng lực cho pose 3D thực tế.

`I_ref = clamp(τ_req/0.47, −8,8)`;
`ΔI = I_ref−I_measured`;
`τ_cmd = clamp(τ_req + 0.2×0.47×ΔI, −3.76,3.76)`.
Vòng dòng giả lập:
`I_next = I + dt/(0.020+dt) × (τ_cmd/0.47−I)`;
`τ_actuator = 0.47×I_next` khi khớp được cấp mô-men.
Khi Disable/Estop, mô-men và trạng thái dòng giả lập được đưa về 0.

20 ms là lag **giả định**, không phải số đo. I được phản hồi từ trạng thái
dòng actuator giả lập, không suy ra từ lực tiếp xúc hoặc toàn bộ joint effort.
Chưa có R/L winding, back-EMF, sụt áp pin, nhiệt, torque-speed envelope,
nhiễu Iq hoặc trễ CAN trong runner này. `SimGds68Joint` MIT cũ không phải đường
actuator chân của GIM V3 runner.

**Hai bánh:** V3 đưa lệnh tốc độ cho `SimBl4012Wheel`; chưa có bù ΔI cho bánh.
Thông số hiện hành: torque cap=0,13 N·m; speed-servo gain=3 N·m/(rad/s);
torque lag nội bộ=1 ms; torque-enable settle=20 ms; ma sát viscous nội bộ=0,001
N·m·s/rad; Coulomb nội bộ=0,006 N·m. Tham số đáp ứng/ma sát này còn ước lượng.
MuJoCo bổ sung wheel-bearing damping=0,002 N·m·s/rad.
Runner bổ sung **command dead time 50 ms + speed-reference lag 75 ms**;
lag tốc độ không được diễn giải thành lag mô-men. Các giá trị dựa vào các
ước lượng đáp ứng motor trước đây, còn khác nhau giữa các log; chưa xác định
riêng trên bộ lắp GIM. Có thể đổi bằng `--wheel-delay` / `--wheel-lag`.
Lượt đứng 25 s với các cặp delay/lag `(0,0.001)`, `(0.020,0.035)` và
`(0.050,0.075)` s đều giữ `Balancing`; đây chưa phải toàn bộ miền bất định.

Các hinge còn có `armature=0.0001 kg·m²`, là quán tính rotor quy đổi **ước
lượng**. Nó tham gia ma trận quán tính khớp, không cộng vào tổng khối lượng
hoặc bảng quán tính vật rắn/COM ở mục 3. Stiffness thụ động bằng 0.

Trọng lượng tổng ở g=9,81 là 30,6072 N. Nếu thân không chạm sàn và chia tải
đều: mỗi bánh có N≈15,3036 N, giới hạn bám μN≈12,2429 N. Torque cap bánh
0,13 N·m tương ứng lực 3,25 N/bánh; tổng lực danh nghĩa tối đa 6,50 N và
gia tốc tịnh tiến lý tưởng khoảng 2,0833 m/s², trước quán tính bánh và ma sát.
Profile GIM đã thay giới hạn LQR 8 m/s² kế thừa bằng **2,0833 m/s²**,
lấy min giữa lực bám và torque cap trên. Motor bánh là giới hạn chính trong
ước tính này. LQR dùng chiều dài hiệu dụng `I_axle/(m × COM_distance)`;
khoảng bảng gain là 0,035–0,240 m, không clamp tư thế duỗi ở 0,200 m nữa.
Torque cap bánh vẫn là giả định; chưa được xác nhận từ thông số BL4820 mới.

## 6. Tích phân, cảm biến và vòng đời

Runtime đang kiểm tra với MuJoCo **3.13.0**. Gravity `[0,0,-9.81] m/s²`.
Integrator `implicitfast`, solver Newton, friction cone elliptic;
iterations=100, tolerance=1e-8, line-search iterations=50 theo model đã compile.
Phương trình động lực học là `M(q)vdot + c(q,v) = τ + J(q)^T f`:
[MuJoCo computation](https://mujoco.readthedocs.io/en/stable/computation/index.html#general-framework).

MJCF có dt mặc định 0,5 ms; runner/GUI đặt lại dt=1/3000 s mặc định.
GUI có thể chọn 2/3/4 kHz; controller/sensor sampling mặc định 500 Hz.
STATE và VIEW phát khoảng 50 Hz. `VIEW.physics_t` là thời gian engine;
`t` là thời gian runner, vẫn tăng khi fixture đang tạm giữ.

Một IMU site ở `[0,0,0.0561] m` trong frame thân, axes cùng frame thân.
Sensor gồm accel 3 trục, gyro 3 trục và position/velocity của sáu encoder.
Sau release, IMU là specific force và angular velocity từ sensor MuJoCo.
Hiện sensor **lý tưởng**, không noise/bias/quantization/dropout/transport delay.
Vị trí chip và hướng lắp thật của MPU6500 vẫn chưa xác nhận riêng.
Trước release, fixture thân thẳng cung cấp 1g +Z và gyro 0 cho 50 mẫu seed.
Estimator dùng `balancingEstimatorConfig()` của firmware: accelerometer
correction 0,2 Hz, `r_measure=1e6`, `bias_learning_gain=0`. Vận tốc điểm IMU
để bù gia tốc được V3 suy ra từ encoder/gyro, không lấy root velocity MuJoCo.
Điểm cân bằng seed được đổi sang frame góc con lắc ảo của V3:
`theta_ref = equilibriumPitch(spawn) + com_line(spawn) − com_line(hang)`.
Với model hiện tại, theta_ref ≈0,181816 rad; seed bằng body pitch ≈0,031267 rad
là sai frame. Đây là sửa adapter mô phỏng, không phải đổi thuật toán LQR.

Profile mô phỏng giữ body damping 0,08 s/LPF 4,5 Hz, knee follow
1 Hz/0,6 rad/s, adaptive suspension, height rate 0,015 m/s, acceleration
0,05 m/s² và jerk 0,20 m/s³. Sau sửa dao động đứng thấp (mục 9), profile
dùng dự đoán trễ bánh + lead cố định bằng lag bánh; bật cổng phản xạ theo
gyro/gia tốc ngang, dùng đầy đủ Jacobian tốc độ COM cho encoder lý tưởng,
và tắt acceleration trim. Roll controller vẫn tắt trong profile này.
Profile hiện tại ở mục 13 dùng gia tốc kế hoạch 0,6 m/s², jerk 10 m/s³
và hip advance bằng preview 0,12 s; các giá trị này chỉ áp dụng cho GIM native.
Spawn duỗi 0,21649 m; khi cân bằng V3 dành 12 mm hành trình gối cho điều
chỉnh thân, nên HEIGHT tối đa hữu hiệu là **0,20449 m**. Runner trả mục tiêu
đã clamp và GUI dùng giới hạn này; không báo đã đạt 0,21649 m khi đang balance.

Reset khởi tạo mục tiêu/góc cân bằng từ tư thế CAD cấu hình, không đọc qpos
MuJoCo để khởi tạo controller. Khi nạp model, runner kiểm tra loại, số chiều
và joint/site đích của các cảm biến; không cho đổi tên một sensor world pose
thành IMU/encoder để đưa ground truth vào đường điều khiển.

Chỉ IMU/encoder và Iq giả lập chân vào nhánh điều khiển. Contact, pose, COM,
constraint force và VIEW chỉ phục vụ chẩn đoán; không làm sensor bổ sung.
Các trường support/flight/Fz chưa có estimator trong runner được đánh dấu
`unknown`, không được hiểu là lực chân bằng 0 hoặc chắc chắn có tiếp xúc.

ENABLE chuẩn bị IMU, khóa chân 1,2 s và bật bánh. Fixture chỉ nhả khi V3
đã ở `Balancing`, hết cửa sổ torque-enable và đủ thời gian làm ấm hàng đợi
lệnh bánh; mặc định khoảng 1,36 s sau ENABLE. Lệnh DRIVE khác 0 bị từ chối
khi chưa sẵn sàng. Fixture mô phỏng người giữ robot lúc bật, không giữ robot
trong các lượt kiểm tra cân bằng tự do.
Sau release, physics tiếp tục qua Fault, Disable và Estop. Disable/Estop
trong lúc chuẩn bị cũng nhả fixture để robot chịu trọng lực:

- Fault: bánh ngừng mô-men, chân giữ mục tiêu cuối; gravity/contact tiếp tục.
- Disable/Estop: actuator ngừng mô-men; gravity, friction và contact tiếp tục.
- Reset: trả về keyframe duỗi, xóa vận tốc/dòng chân và giữ fixture mới.

Các lượt Fault cũ (trước sửa startup adapter) xác nhận vỏ thân/gối thực sự
chạm sàn: thân z≈−0,010 mm, vỏ gối z≈−0,0015 mm ở trạng thái nghỉ. Sau sửa
startup, ENABLE bình thường đã cân bằng; test Fault dùng `PUSH 10 .3` tại
thời gian runner khoảng 2 s và chạy 10 s để kiểm tra tiếp xúc sau ngã. Lực
chỉ đi vào vật lý, không đi vào estimator/controller. Không đặt pose ngã bằng
tay hoặc ép thân nằm ngang; tư thế cuối tùy trạng thái chân khi Fault.

Trong các lượt Fault/Disable/Estop đã ghi trước sửa startup, xuyên sàn tức thời lớn nhất tại
các snapshot 50 Hz khoảng 2,0 mm khi va đập, do contact mềm; xuyên sàn ở tư thế
nghỉ chỉ vài micromet. Chưa đo cực trị ở tất cả bước physics 3000 Hz.

## 7. Kiểm chứng và giới hạn kết luận

Các kết quả cân bằng/ngã bên dưới được ghi với ma sát khớp **0,2 N·m**.
Theo yêu cầu mới, model đã đổi cả bốn đùi/gối xuống **0,1 N·m**; các kết quả
cũ chưa chứng minh hiệu năng với giá trị mới. Ma sát bánh–sàn vẫn μ=0,8.

`gim_model_physics_tests` đối chiếu COM, FK và quán tính controller với MuJoCo
ở CAD zero, tư thế duỗi và tư thế bất đối xứng; kiểm tra khối lượng, tensor
hợp lệ, trục +Y, μ cặp bánh–sàn, frictionloss và giới hạn mô-men.
Kiểm tra envelope CAD/collider cho thân và bốn link chân tại 22 tư thế
pitch/roll, sai khác tối đa 1 mm; model box/capsule cũ không đạt.
`gim_physics_lifecycle_test.py` kiểm tra physics clock không dừng khi Fault,
Disable, Estop, fixture Spawn/Reset và vỏ CAD chạm sàn sau ngã trong vòng chạy
controller. Kiểm tra tất cả visual CAD ở snapshot cuối: không chi tiết nào
xuyên sàn quá 0,1 mm. Bản capsule cũ thất bại vì gối xuyên 5,6 mm;
bản box thân cũ thất bại vì vỏ cách sàn 64 mm. Đây là kiểm tra nhất quán model
và vòng đời, không phải kiểm tra robot cân bằng.

```bash
cmake -S . -B build -DBUILD_TESTING=ON -DTAP_BUILD_SIMULATOR=ON
cmake --build build --target tap_mujoco_gim_v3 gim_model_physics_tests -j4
ctest --test-dir build -R '^gim_(model_physics|physics_lifecycle|balance)$' --output-on-failure
```

### Cân bằng tự do với controller V3.3

`gim_balance_test.py` chạy cùng V3Controller, actuator ΔI và profile bánh
50 ms/75 ms. Các giới hạn kiểm tra đặt trước: body pitch <0,30 rad, sai số góc
con lắc ảo <0,15 rad; cruise error và vận tốc cuối <0,05 m/s. Test so body
pitch với quaternion VIEW độc lập, kiểm tra fixture nhả, physics clock tăng,
`ground_truth_to_controller=0`, drive/yaw thực sự đổi pose và HEIGHT đạt mục tiêu.

| Kịch bản | Thời lượng | Kết quả |
| --- | --- | --- |
| Đứng tự do | 60 s | Đạt |
| Chạy 0,2 m/s từ giây 6, STOP giây 11 | 25 s | Đạt tốc độ, dịch chuyển và dừng |
| Đẩy +2 / −2 N, mỗi lần 0,2 s | 25 s | Đạt |
| HEIGHT 0,17 → 0,13 → 0,20 m | 35 s | Đạt các mục tiêu trong sai số 5 mm |
| Quay 0,3 rad/s rồi dừng | 25 s | Đạt góc quay và dừng |

Lượt đứng riêng **300 s**: không Fault, peak body pitch 0,04164 rad (~2,39°),
20 s cuối peak pitch 0,001856 rad và peak ground speed 0,03312 m/s;
root X cuối 0,00620 m, Z cuối 0,20390 m. Dòng khớp cực đại 1,314 A.
Lượt hạ HEIGHT=0,08 m trong **300 s**: không Fault, Z cuối 0,07861 m,
nhưng 20 s cuối peak speed **0,1383 m/s** và pitch 0,02015 rad. Đứng thấp
vẫn có dao động, chưa đạt tiêu chí dừng <0,05 m/s. Tăng low wheel lead
0,09 lên 0,12 s không giải quyết (120 s: 0,1347 m/s); đã bỏ thay đổi đó.
Lượt **đẩy 5 N trong 0,2 s gây Fault**; không nâng ngưỡng ngã để che kết quả.
Lượt max DRIVE 0,5 m/s và yaw 0,8 rad/s rồi dừng chưa Fault trong 40 s,
nhưng các giá trị này chưa được dùng làm tiêu chí robustness chính thức.

Đối chứng adapter: chỉ sửa fixture còn estimator cũ gây Fault ~1,84 s;
sửa estimator nhưng seed sai frame còn Fault ~2,54 s. Sửa seed con lắc ảo
mới đạt đứng tự do. Log/JSON được giữ ở `build/gim_physics_evidence/` (artifact
local, không phải log robot thật). Bốn unit test GUI kiểm tra nhận `Balancing`,
chặn DRIVE khi fixture đang giữ và finite-motion readiness đều đạt.
Ở lượt kiểm tra lịch sử, `v3_tests` còn assertion kế thừa coi knee=0 là
chiều cao lớn nhất. Mục 9 ghi kết quả sau sửa fixture theo CAD GIM.

Những phần chưa xác định từ dữ liệu hiện có: tensor/COM thật của từng cụm,
rotor inertia, độ mềm/lún/lăn của lốp, ma sát thân, backlash/đàn hồi truyền động,
torque-speed/thermal envelope và băng thông/trễ/noise của driver/cảm biến.
Tính được nghiệm mô phỏng với các giả định trên không xác nhận robot thật
đứng, tự phục hồi hoặc chịu được cùng nhiễu.

## 9. Sửa dao động đứng thấp — 08/10/2026, ma sát khớp 0,1 N·m

Nguyên nhân tái hiện được trong runtime: LQR giải gia tốc bánh tức thời,
nhưng lệnh tốc độ qua trễ chết 50 ms và lag 75 ms. Bù lead đơn thuần còn
để vòng bánh tự dao động khi hạ thấp; đường đùi không có cổng sự kiện tiếp
tục phản ứng với dao động đó. Tắt riêng phản xạ không chữa vòng bánh.
Đối chứng bỏ trễ/lag dập dao động; đối chứng giữ nguyên vật lý nhưng tắt
bộ dự đoán mới tái hiện dao động. Không đổi μ, ma sát khớp, khối lượng,
va chạm, giới hạn dòng/mômen hoặc ngưỡng ngã để đạt kết quả.

`LqrActuationPredictor` dự đoán `[x_axle, theta_virtual-error, v_axle,
theta_virtual_rate]` qua lịch sử lệnh đã gửi, có giới hạn gia tốc. Lead
0,075 s chỉ bù lag; dự đoán chịu phần trễ chết 0,050 s. Hàng đợi cố định
tối đa 400 mẫu; reset khi engage, held/re-zero, Fault, Disable hoặc đổi
thuật toán. Chỉ dùng IMU, encoder và lệnh đã gửi. Đây là mô hình con lắc
giảm bậc có bão hòa, chưa phải chứng minh ổn định của robot nhiều vật thể.
Cơ sở cho việc dự đoán theo trạng thái/lịch sử đầu vào:
[Kovács et al., 2022](https://onlinelibrary.wiley.com/doi/10.1002/rnc.5649).

Cổng phản xạ GIM mở theo gyro thân lọc 5 Hz (0,06..0,20 rad/s) hoặc
gia tốc ngang bỏ gravity (0,2..0,8 m/s²). Lean tĩnh không tự mở cổng;
preview và vòng level chậm vẫn hoạt động. Rate COM dùng Jacobian đầy đủ
thay hệ số 0,7 kế thừa từ encoder/rotor của robot cũ. Tắt acceleration
trim để không học dao động thành góc cân bằng mới.

Ngưỡng chấp nhận được đặt trước: 20 s cuối peak speed <0,02 m/s,
RMS speed <0,01 m/s, hành trình đùi/gối <0,01 rad, RMS gyro <0,05 rad/s,
cổng phản xạ <0,01. Fixture phải đã nhả và không có Fault.

| Ca | Thời gian | Peak speed cuối (m/s) | Hành trình đùi cuối (rad) | Cổng lúc yên |
| --- | ---: | ---: | ---: | ---: |
| Đối chứng tắt dự đoán, HEIGHT 0,08 | 100 s | 0,30228 | 0,29458 | 1 |
| HEIGHT 0,08 | 100 s | 0,00794 | 0,00147 | 0 |
| HEIGHT 0,08 + đẩy ±2 N × 0,2 s | 120 s | 0,00925 | 0,00120 | 0 |
| HEIGHT 0,08 + chạy 0,15 / dừng / quay 0,3 / dừng | 120 s | 0,01185 | 0,00187 | 0 |
| Chu kỳ 0,17 → 0,13 → 0,08 → 0,20 → 0,08 | 130 s | 0,01049 | 0,00229 | 0 |
| HEIGHT 0,06988 | 100 s | 0,00742 | 0,00014 | 0 |
| HEIGHT 0,08, dài | 300 s | 0,01322 | 0,00275 | 0 |
| HEIGHT 0,06988, dài | 300 s | 0,00163 | 0,000014 | 0 |

Các ca bật dự đoán đều kết thúc Balancing. Hai lần đẩy mở cổng >0,1
rồi cổng đóng sau phục hồi. Gyro RMS cuối ở ca 300 s/0,08 m là
0,01248 rad/s; thành phần >5 Hz đo từ telemetry 50 Hz là 0,01118 rad/s
(không đánh giá được phổ ≥25 Hz). Hông chủ yếu còn trôi chậm ~0,05 Hz,
không còn chu kỳ ván trượt nhiều Hz. Log/JSON ở
`build/gim_physics_evidence/crouch_fix/` và `crouch_long/`.

Kiểm chứng bổ sung: `v3_delay_tests` kiểm thứ tự hàng đợi, reset, instance
độc lập, giới hạn gia tốc, cờ tắt, cổng tĩnh/động, Fault/Disable; 12 ca plant
con lắc phẳng (length 0,08/0,12/0,20 m, delay 40/60 ms, lag 60/90 ms)
phục hồi lệch đặt ban đầu và xung vận tốc góc. `v3_tests` đạt sau sửa
hai assertion fixture: CAD knee=0 là height nhỏ nhất; khi không có encoder
chân, reference bánh bao gồm feed-forward độ cao. Không nới tolerance.
`gim_model_physics`, `gim_physics_lifecycle`, `gim_balance`, `gim_crouch`
và bốn unit GUI đạt. `pio run -e esp32-s3-v3` build thành công:
RAM 151836/327680 bytes, Flash 1087701/6553600 bytes; không flash.
Không tuyên bố toàn bộ host suite cũ hoặc độ bền với tham số chưa đo đã đạt.

## 10. Phản ứng ván trượt khi nhấn/thả tiến–lùi

Các kiểm tra mục 9 chỉ chạy thấp ở 0,15 m/s và chưa kiểm tra riêng phản ứng
đùi trong tăng tốc/phanh. Ở lượt kiểm tra bổ sung, chuyển tốc độ 0,3 m/s tại
HEIGHT=0,08 m đã kích dao động rồi Fault. Đường lead đùi lấy vận tốc góc con
lắc ảo, chứa cả vận tốc encoder của chính đùi, nên có thể khuếch đại chuyển
động nó vừa yêu cầu. Fault sau đó dừng phản ứng chủ động. Một lượt đối chứng
45 s khác không ngã: kết quả vật lý phụ thuộc pha actuator/contact, không phải
mọi thao tác nhấn đều gây Fault. Không có log trực tiếp của lần người dùng gặp lỗi.

Profile GIM chuyển đường lead đùi sang gyro thân bằng lựa chọn sẵn có
`tier.lead_gyro_only=true`. LQR bánh vẫn dùng vận tốc con lắc ảo đầy đủ.
Preview từ gia tốc kế hoạch và vòng level chậm vẫn hoạt động; chỉ phản xạ
nhiễu chịu cổng quán tính. Giữ nguyên ma sát, giới hạn actuator và ngưỡng ngã.
Đối chứng unit với gyro=0, gia tốc mở cổng và nhiễu encoder đùi 0,5 rad/s:
lead cũ tạo yêu cầu đùi 0,02378 rad; lead gyro tạo 0 rad. Đây là kiểm tra
đường phản hồi, không phải mô hình nhiễu cảm biến đã được đo trên robot.

`gim_maneuver_test.py` kiểm 3 độ cao 0,08/0,13/0,20 m, mỗi lượt 135 s.
Mỗi độ cao gồm 8 sự kiện tăng tốc/phanh có kiểm dấu preview, chuyển động cả
hai encoder đùi và cổng quán tính mở; 6 cửa sổ tốc độ tiến/lùi 0,1/0,3/0,5 m/s
phải đạt trong sai số 0,05 m/s. Đẩy ±2 N × 0,2 s rồi kiểm đứng yên bằng
cùng ngưỡng mục 9, không có fixture, Fault hay ground truth trong controller.
Lượt 100 s bổ sung nhấn/thả tiến và lùi 200 ms/800 ms kiểm phản ứng thực của
đùi; nhấn 200 ms có hành trình đùi khoảng 0,043 rad (2,5°), 800 ms khoảng
0,060 rad (3,4°). Nhấn ngắn không đạt ngay tốc độ đặt vì ramp và trễ driver.

Lặp lại đứng thấp 300 s với profile gyro: HEIGHT=0,08 m kết thúc Balancing,
20 s cuối peak speed 0,01322 m/s, hành trình đùi 0,00275 rad; HEIGHT=0,06988 m
peak speed 0,00139 m/s, hành trình đùi 0,000012 rad. Cổng lúc yên đều bằng 0.
Log/JSON ở `build/gim_physics_evidence/maneuver_fix/` và `maneuver_long/`.

GUI cũng sửa thanh tốc độ cập nhật lệnh đang chạy. Sáu unit GUI gồm kiểm
nhấn tiến/lùi và thả nút chỉ gửi STOP một lần. Đây là lỗi riêng; thao tác
người dùng xác nhận trong phiên này là nhấn/thả nút tiến hoặc lùi.
Tám nhóm CTest liên quan (`v3_delay_tests`, `v3_tests`, `v3_split_tests`,
`gim_model_physics`, `gim_physics_lifecycle`, `gim_balance`, `gim_crouch`,
`gim_maneuver`) đạt; firmware build thành công, không flash.
Các kết quả dùng cảm biến lý tưởng và driver giả định; chưa xác nhận trên phần cứng.

## 11. Độ nhạy nhấn/thả — profile GIM native

Profile trước dùng gia tốc kế hoạch 0,3 m/s², jerk 1 m/s³, hip advance 0,08 s
so với wheel preview 0,12 s: preview đùi còn chờ 40 ms lịch sử. Đo lệnh DRIVE
0,3 m/s tại 3 độ cao cho thấy yêu cầu động đùi >0,01 rad sau 140 ms và chuyển
động encoder đùi >0,008 rad sau 140–160 ms. Đạt 90% tốc độ đặt mất khoảng
2,58/1,72/1,44 s tại HEIGHT 0,08/0,13/0,20 m. Bản cũ đạt cân bằng nhưng
chưa có tiêu chí độ trễ phản ứng trong test.

Chỉ chỉnh 3 tham số profile native: `plan_accel_m_s2=0.6`,
`plan_jerk_m_s3=3`, `hip_advance_s=preview_s=0.12`. Core/firmware giữ mặc định
cũ. Preview đọc gia tốc kế hoạch hiện tại; body-gyro lead và cổng phản xạ
quán tính giữ theo mục 10. Gia tốc kế hoạch vẫn nằm dưới trần gia tốc bánh
2,0833 m/s² của model. Không đổi các giả định vật lý hay ngưỡng an toàn.

Tiêu chí bổ sung đặt trước lượt hồi quy: trong 120 ms sau từng lệnh tăng
tốc/phanh, preview động đúng dấu >0,01 rad và cả hai encoder đùi chuyển đúng
dấu >0,008 rad. Bản cũ không đạt tiêu chí này; bản mới đạt 24 sự kiện ở
3 độ cao và 4 lần nhấn ngắn. Lần tiến đầu tiên phản ứng sau 60 ms ở cả
3 độ cao. Nhấn 200 ms cho hành trình đùi khoảng 4,4° (cũ 2,5°), nhấn 800 ms
khoảng 5,8–6,0° (cũ 3,4°). Đây là thời gian mô phỏng từ command tới STATE
50 Hz, chưa bao gồm độ trễ chuột/desktop.

Tốc độ bánh cải thiện ít hơn phản ứng đùi: đạt 90% của 0,3 m/s sau khoảng
2,28/1,44/0,90 s tại 3 độ cao tương ứng. Không tuyên bố toàn bộ robot đạt
tốc độ đặt sau 60 ms. Tám nhóm CTest liên quan và sáu unit GUI đạt với
profile mới; đứng yên vẫn dùng các ngưỡng mục 9. Bằng chứng trước chỉnh nằm
ở `build/gim_physics_evidence/response_baseline/`; các biến thể thử ở
`response_variants/`, nhấn ngắn ở `response_latency/`, hồi quy profile mới ở
`maneuver_fix/` và `crouch_fix/`. Kết quả không chứng nhận phần cứng hay
driver/sensor thực tế; tăng ramp có thể tăng gia tốc và chuyển động thân.

## 12. Khớp phản ứng sớm và đường GUI không bị chặn

Người dùng xác nhận yêu cầu là khớp phản ứng sớm. Bài đo GUI trước sửa:
sự kiện nhấn → gửi DRIVE 8–18 ms; encoder đổi >0,008 rad sau 70–100 ms.
Trong bài này native clock bám wall clock (44,4803 s sim trong 44,4815 s wall),
nhưng GUI có khoảng trống giao telemetry tới 62 ms. Không có bằng chứng CPU
physics bị chậm kéo dài trong lượt đó. Kiểm tra mã thấy GUI gửi tuần tự từng
VIEW bằng pipe blocking, có thể nghẽn khi renderer chậm; viewer đọc và áp
từng frame cũ. Phép thử pipe đầy tái hiện điểm nghẽn độc lập với robot physics.

Sửa đường GUI: gửi DRIVE trong callback nhấn ngay sau các cổng readiness,
ENABLE/ESTOP; tick 20 ms tiếp tục refresh. STOP khi thả giữ ưu tiên như trước.
Hàng đợi telemetry giới hạn 256 dòng gần nhất, mỗi lượt drain chỉ gửi VIEW
mới nhất. Pipe viewer nonblocking, snapshot <=4096 bytes ghi nguyên tử hoặc
bỏ frame khi pipe đầy. Viewer chỉ áp VIEW hoàn chỉnh mới nhất trong mỗi lần
đọc, giữ phần dòng chưa hoàn chỉnh. Renderer không thay đổi physics hay sensor.

Chỉnh riêng jerk kế hoạch GIM 3 → 6 m/s³ để preview đùi dựng sớm hơn; giữ
gia tốc 0,6 m/s², hip advance 0,12 s. Không áp dụng các biến thể thử tăng
gain bám tốc độ bánh; gain bánh vẫn như trước. Các biến thể jerk 6/10/12
ở `hip_early_variants/` đều chưa Fault trong lượt 105 s nhưng jerk 10/12
tăng peak gyro; chọn mức 6. Firmware/core vẫn giữ mặc định cũ.

Tiêu chí controller siết từ 120 xuống **80 ms**: preview đúng dấu >0,01 rad
và cả hai encoder đùi chuyển đúng dấu >0,008 rad sau mỗi lệnh. Tám nhóm CTest
đạt, gồm 24 sự kiện tại 3 độ cao và 4 nhấn ngắn; đứng yên vẫn dùng mục 9.
Mười một unit GUI đạt, có pipe đầy không chặn, hàng đợi hữu hạn, chỉ gửi frame
mới nhất, giữ dòng chia nhỏ, DRIVE gửi ngay và vẫn chặn readiness/ESTOP.

Bài đo wall-clock trực tiếp qua Tk `ButtonPress/Release`, UDP, native runner
và viewer MuJoCo ở HEIGHT=0,15 m, nhấn tiến/lùi 200 ms:

| Đo từ sự kiện nhấn | Tiến | Lùi |
| --- | ---: | ---: |
| Gửi DRIVE | 0,25 ms | 0,21 ms |
| Encoder đùi chuyển >0,008 rad đúng dấu | 64,71 ms | 63,88 ms |
| Renderer áp snapshot có chuyển động đó | 64,94 ms | 64,08 ms |

Cả hai kết thúc Balancing. Log ở `build/gim_physics_evidence/gui_response_new/`
(`events.jsonl`, `rendered.jsonl`, `latency.json`); đối chứng GUI cũ ở
`gui_response/`. Mốc renderer là lúc cập nhật state cho MuJoCo, chưa đo
scan-out của màn hình. Đây là số đo trên desktop hiện tại, không phải bảo đảm
hard realtime dưới mọi tải, hoặc hiệu chuẩn trễ motor/cảm biến thật.

## 13. Tăng phản hồi khớp — Kd 0,8, jerk 10

Yêu cầu tiếp theo là tăng phản ứng khớp. Profile GIM native đổi đúng hai
tham số: gain tốc độ trong vòng trở kháng `Kd: 0.5 → 0.8 N·m·s/rad` và jerk
preview `6 → 10 m/s³`. Giữ Kp=12 N·m/rad, gia tốc kế hoạch 0,6 m/s²,
lọc gối 1 Hz, giới hạn dòng 8 A/mô-men 3,76 N·m, lag dòng 20 ms, ma sát
và model CAD. Đường bù ΔI vẫn giữ gain 0,2. Firmware giữ profile mặc định.

Các thử nghiệm tăng Kp 18/24 giảm sai số tracking nhưng không được áp dụng:
`BoardConfig.hpp` giữ trần Kp 12 từ kết quả động cơ trước đây; không bỏ trần
calibration để làm sim nhanh hơn. Tăng riêng cutoff gối 2 Hz không cải thiện
đáng kể độ trễ nên giữ 1 Hz. Bằng chứng biến thể ở `joint_gain_variants/`,
`joint_fast_variants/` và `joint_velocity_gain/` dưới `build/gim_physics_evidence/`.

Tám nhóm CTest đạt, cùng các ngưỡng cân bằng/dừng trước đó. Bổ sung tiêu chí
hai đùi phải chuyển **1° đúng dấu trong 60 ms**, chấp nhận 1 ms lượng tử thời
gian tại telemetry 50 Hz. Kiểm thêm gyro RMS thành phần >5 Hz <0,08 rad/s và
hip-target RMS >5 Hz <0,005 rad trong cửa sổ thao tác 35–85 s. Đây là phép
FFT trên telemetry 50 Hz, chỉ đánh giá phổ dưới 25 Hz, chưa phải đo tiếng
gearbox hay phổ rung toàn băng tần.

| HEIGHT | Gyro HF RMS (rad/s) | Hip target HF RMS (rad) | Dòng đỉnh (A) | Body pitch đỉnh (rad) |
| --- | ---: | ---: | ---: | ---: |
| 0,08 m | 0,04383 | 0,00209 | 2,633 | 0,05595 |
| 0,13 m | 0,03491 | 0,00131 | 2,420 | 0,04918 |
| 0,20 m | 0,03174 | 0,00127 | 1,820 | 0,04515 |

Ba lượt 135 s gồm 24 sự kiện tăng tốc/phanh, 18 cửa sổ tốc độ, đẩy và đứng
yên; thêm 4 nhấn ngắn trong lượt 100 s. Không Fault, fixture đã nhả,
ground-truth-to-controller=0 và cổng đóng khi yên. Mười một unit GUI đạt.

Đo wall-clock qua GUI ở HEIGHT 0,15 m, nhấn tiến/lùi 200 ms:

| Mốc từ sự kiện nhấn | Trước (Kd 0,5, jerk 6) | Sau (Kd 0,8, jerk 10) |
| --- | ---: | ---: |
| Encoder đùi đổi >0,008 rad | 64–65 ms | 39–42 ms |
| Encoder đùi chuyển 1° | 105–106 ms | 80–83 ms |
| Renderer áp snapshot đổi >0,008 rad | 64–65 ms | 39–42 ms |

Lệnh gửi sau khoảng 0,21–0,22 ms. Các lần nhấn đo được diễn ra trước các
thao tác người dùng bổ sung ở giây 41–45; không dùng snapshot cuối đang
phanh làm kết luận đứng yên. Log ở `gui_joint_fast/`, đối chứng `gui_response_new/`.
Số đo GUI bao gồm lịch desktop và lượng tử telemetry; số đo native ban đầu
20/40 ms chỉ là thời gian từ command đến encoder trong model. Renderer time
không bao gồm scan-out màn hình. Ramp mạnh hơn tăng xung gia tốc trong lúc
chuyển động; không bảo đảm cùng độ trễ/rung trên actuator thật.
