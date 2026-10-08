# Dữ liệu giữa V3.1 và driver động cơ bánh xe

Tài liệu này chỉ nói về dữ liệu đi qua driver của hai bánh xe và vai trò của chúng trong V3.1. Tóm tắt ngắn: firmware gửi **tốc độ bánh mong muốn**; driver trả về **tốc độ đang quay** và **góc/vòng quay đã đo được**. V3.1 cần cả lệnh lẫn feedback để biết motor có làm đúng yêu cầu không và robot đã dịch chuyển bao xa.

## Firmware gửi xuống driver

| Thông tin | Ý nghĩa | Vai trò trong V3.1 |
|---|---|---|
| Bật / tắt motor | Cho phép motor chạy hoặc ngừng giữ lực | Cho phép controller bắt đầu điều khiển bánh. Đây là trạng thái hoạt động, không phải thông tin về cân bằng. |
| Tốc độ mục tiêu của bánh trái và phải | Tốc độ góc mà mỗi bánh cần đạt; core dùng rad/s và quy ước dương là robot tiến | Đây là đầu ra của bộ cân bằng. Driver điều khiển motor chạy theo tốc độ được yêu cầu. V3.1 không gửi lệnh mô-men trực tiếp đến driver bánh. |

Lệnh tốc độ mục tiêu và tốc độ thực đo được là hai đại lượng khác nhau: lệnh là điều controller *muốn* bánh làm; feedback cho biết bánh *đã thực sự* làm gì.

## Driver trả feedback về firmware

| Thông tin | Ý nghĩa | Vai trò trong V3.1 |
|---|---|---|
| Tốc độ thực của từng bánh | Bánh đang quay nhanh/chậm và theo chiều nào | Cho biết robot đang chuyển động tức thời ra sao. V3.1 đổi nó thành vận tốc mặt đất; PID mặc định dùng vận tốc này trong vòng điều khiển tốc độ, còn tốc độ riêng từng bánh giúp phát hiện bánh quay quá nhanh. |
| Vị trí encoder của từng bánh | Góc bánh đã quay; có thể quy đổi thành số vòng | Giúp ước lượng robot đã đi được bao xa. PID dùng vị trí để hạn chế trôi/giữ chỗ; LQR cũng có thể dùng sai số vị trí khi chức năng giữ vị trí được bật. Cần tích lũy qua các vòng quay: giá trị encoder của driver chỉ quay trong một vòng rồi trở về đầu, không phải tổng số vòng từ lúc bật máy. |
| Mã trạng thái/lỗi của phản hồi | Cho biết phản hồi có lỗi hay không | Giúp firmware loại dữ liệu lỗi hoặc cũ; controller không nên cân bằng bằng feedback không hợp lệ. |

## Vì sao V3.1 cần cả tốc độ lẫn số vòng quay?

- **Tốc độ** trả lời “ngay lúc này bánh đang quay thế nào?”. Nó cần cho phản hồi nhanh của cân bằng và để kiểm tra bánh có theo kịp tốc độ lệnh hay không.
- **Góc quay tích lũy** trả lời “bánh đã quay tổng cộng bao nhiêu?”. Từ đó controller ước lượng vị trí/quãng đường; nếu chỉ biết tốc độ tức thời thì khó biết robot đã trôi khỏi chỗ đứng bao xa.

Trong code, V3.1 lấy trung bình hai bánh rồi dùng bán kính bánh `0.040 m` để đổi góc và tốc độ quay thành chuyển động dọc mặt đất. Ví dụ lý tưởng, mỗi bánh quay một vòng trong khi thân và chân không đổi góc thì tương ứng khoảng `0.251 m` lăn (chu vi bánh); đây là ước lượng và giả định bánh không trượt.

`V3Controller` hiện chọn PID làm thuật toán mặc định; LQR là lựa chọn khác. Hai cách điều khiển khác nhau, nhưng đều nhận feedback bánh đã đổi sang quy ước tốc độ/góc của core.

Do bánh gắn với cụm chân, khi chân đổi góc thì trục bánh cũng dịch chuyển tương đối với thân dù bánh chưa lăn tương ứng. Vì vậy V3.1 hiệu chỉnh ước lượng vị trí/vận tốc bằng **pitch và pitch-rate của thân** cùng **góc chuỗi hip+knee**. Dữ liệu hip/knee đến từ encoder chân, không phải driver bánh. Đây là lý do cần giữ riêng góc bánh tích lũy, trạng thái IMU và góc chân.

## Quy ước quan trọng khi đưa feedback vào core

- V3.1 nhận tốc độ bánh theo `rad/s` và góc quay theo **radian đã tích lũy qua nhiều vòng**; dấu dương trong core là tiến cho cả hai bánh. Lớp driver phải đổi dấu và đơn vị từ giao thức của motor sang quy ước này.
- Nếu chỉ đưa vị trí một-vòng của encoder vào core mà không nối tiếp các vòng qua điểm wrap, ước lượng quãng đường sẽ nhảy đột ngột mỗi vòng.
- Giá trị tốc độ thô của giao thức được repo quy đổi như RPM theo suy luận từ dải số của motor; phép quy đổi này vẫn cần xác nhận đo đạc. Sai hệ số sẽ làm controller hiểu sai tốc độ thực.

## Nguồn trong code và tài liệu

- [Dữ liệu đầu vào/đầu ra V3.1](../src/tap_control/core/include/tap_fw/v3/V3Controller.hpp)
- [Cách V3.1 tính vận tốc và vị trí mặt đất](../src/tap_control/core/src/v3/V3Controller.cpp)
- [Dữ liệu feedback của BL4820M12](actuatal_driver/WHEEL_MOTOR_PROTOCOL.md)
