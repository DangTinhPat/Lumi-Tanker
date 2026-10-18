# GDS68 motor mapping thực tế

Ngày ghi nhận: 2026-09-25

Phạm vi: 4 driver GDS68/GIM6010-8 cho khớp hông/gối. Đây là mapping thực tế
do người dùng xác định trên phần cứng, dùng để đối chiếu khi viết lớp HAL,
calibration và bảng dấu. Không phải bằng chứng robot đã sẵn sàng chạy vòng kín.

## Mapping CAN ID và chiều encoder

Quy ước quan sát: "góc nhìn thứ nhất" theo mô tả của người dùng trong phiên
bring-up. Cần giữ nguyên mô tả này cho tới khi có ảnh/khung tọa độ cơ khí
chính thức hơn.

| Bên | Khớp | CAN node ID | Chuyển động kiểm tra | Dấu góc driver |
|---|---|---:|---|---|
| Phải | Hông/đùi | 1 | Từ chân duỗi, quay lên phía đầu robot 90 deg | Âm |
| Phải | Gối | 2 | Từ chân duỗi về trạng thái gập | Âm |
| Trái | Hông/đùi | 3 | Từ chân duỗi, quay lên phía đầu robot 90 deg | Dương |
| Trái | Gối | 4 | Từ chân duỗi về trạng thái gập | Dương |

## Mapping sang core

Core vẫn dùng joint order `[LH, LK, LW, RH, RK, RW]`. Bảng chuyển đổi chính
thức nằm trong `src/tap_control/hal/include/robot_hal/MotorMapping.hpp`:

| Core joint | CAN node ID | Driver sign |
|---|---:|---:|
| LH | 3 | +1 |
| LK | 4 | -1 |
| RH | 1 | -1 |
| RK | 2 | +1 |

`driver_value = core_value * driver_sign` cho position, velocity và torque/
feed-forward tại biên GDS68.

## Ghi chú tích hợp

- Mapping này là dữ liệu thực tế chính thức, đã được cập nhật vào code HAL và
  native sim frame boundary.
- Chỉ nên có một bảng chuyển đổi duy nhất từ joint index `[LH, LK, LW, RH, RK,
  RW]` sang CAN node ID và dấu encoder/command.
- Không rải dấu trái/phải trong controller core. Core vẫn giữ quy ước robot;
  bảng hiệu chuẩn driver chịu trách nhiệm đổi sang dấu driver thực tế.
- Trước khi enable bất kỳ vòng kín toàn thân nào, cần test từng khớp riêng ở
  mức command rất nhỏ, xác nhận đúng khớp quay và đúng chiều cơ khí.
