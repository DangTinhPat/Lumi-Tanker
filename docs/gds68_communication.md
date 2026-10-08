# GDS68 giao tiếp với bộ điều khiển

GDS68 là driver tích hợp trong động cơ GIM6010-8. Trong robot, bộ điều khiển gửi yêu cầu chuyển động qua CAN; driver điều khiển động cơ và gửi lại trạng thái cùng một số giá trị đo được.

## Driver đo được những gì?

| Bộ phận/tín hiệu | Nói đơn giản | Dùng để làm gì |
|---|---|---|
| Encoder tích hợp | Đo góc quay; từ đó driver tính vị trí và tốc độ khớp | Driver tự giữ góc; bộ điều khiển robot đọc góc/tốc độ thực để biết khớp đang ở đâu và chạy nhanh thế nào |
| Cảm biến dòng điện bên trong driver | Đo dòng điện động cơ, gồm dòng trục Q (Iq) | Driver dùng cho điều khiển động cơ; từ dòng điện có thể ước lượng mô-men |
| Ước lượng mô-men | Giá trị driver tính từ dòng điện và hằng số mô-men của động cơ | Theo dõi phản ứng của động cơ; đây **không phải** cảm biến lực ở chân, cảm biến tải trọng hay phép đo chính xác lực tiếp xúc với sàn |
| Cảm biến nhiệt bên trong | Theo dõi nhiệt độ động cơ và driver | Bảo vệ quá nhiệt; đọc nhiệt độ qua giao tiếp tham số, không nằm trong gói phản hồi MIT thông thường |
| Điện áp và dòng nguồn | Theo dõi nguồn cấp cho driver | Chẩn đoán sụt áp hoặc tình trạng nguồn |

Encoder cung cấp thông tin vị trí, không đo lực. GDS68 cũng không có cảm biến “trở kháng” để đo độ cứng của chân. Chế độ MIT cho phép đặt độ cứng và giảm chấn bằng tham số điều khiển; đó là cách driver phản ứng với sai lệch, không phải cảm biến trở kháng.

## Các lệnh bộ điều khiển gửi xuống

| Nhóm lệnh | Thông tin gửi | Vai trò |
|---|---|---|
| Bật/tắt khớp | Cho trục vào chế độ nghỉ hoặc điều khiển kín | Cho phép hoặc ngừng driver chủ động điều khiển khớp |
| Chọn chế độ | Chọn điều khiển vị trí, tốc độ, mô-men hoặc MIT | Chọn cách driver diễn giải lệnh chuyển động |
| Đặt vị trí | Góc đích; có thể kèm tốc độ và mô-men bù | Yêu cầu khớp đi tới hoặc giữ một góc |
| Đặt tốc độ | Tốc độ quay đích; có thể kèm mô-men bù | Yêu cầu khớp quay theo tốc độ đặt |
| Đặt mô-men | Mô-men yêu cầu | Điều khiển theo mô-men thay vì đặt góc/tốc độ |
| Lệnh MIT | Góc đích, tốc độ đích, Kp, Kd và mô-men bù | Kết hợp bám góc với phản ứng đàn hồi/giảm chấn do driver tính toán bên trong |
| Cấu hình | Giới hạn tốc độ/dòng, gain, thông số quỹ đạo và địa chỉ khớp | Cấu hình cách driver vận hành; một số thay đổi cần lưu hoặc khởi động lại |
| Hiệu chuẩn và bảo trì | Hiệu chuẩn động cơ/encoder, đặt lại vị trí, xóa lỗi, lưu cấu hình, khởi động lại | Chuẩn bị hoặc chẩn đoán driver; không phải lệnh chuyển động thường xuyên |
| Dừng khẩn | Yêu cầu dừng khẩn cấp | Dừng điều khiển và báo trạng thái lỗi tương ứng |

## Driver gửi dữ liệu gì về?

| Dữ liệu phản hồi | Ý nghĩa và vai trò |
|---|---|
| Góc và tốc độ thực | Lấy từ encoder; xác nhận khớp đã tới mục tiêu chưa và cung cấp trạng thái chuyển động |
| Mô-men ước lượng | Tính từ dòng điện động cơ; hữu ích để theo dõi/đánh giá, nhưng không phải tải trọng đo trực tiếp |
| Trạng thái và lỗi | Cho biết driver còn hoạt động, đang ở trạng thái nào và có lỗi động cơ, encoder hay bộ điều khiển không |
| Dòng điện | Có thể đọc giá trị dòng đặt và dòng đo để chẩn đoán hoạt động của động cơ |
| Điện áp/dòng nguồn và công suất | Hỗ trợ kiểm tra nguồn cấp và mức tiêu thụ |
| Nhiệt độ | Có thể đọc qua truy cập tham số; không nên hiểu là luôn có trong gói phản hồi điều khiển thông thường |

Một số dữ liệu trạng thái được driver gửi định kỳ; những dữ liệu chẩn đoán khác cần bật chu kỳ gửi hoặc chủ động đọc. Vì vậy “driver hỗ trợ trả dữ liệu” không đồng nghĩa firmware robot hiện tại đang đọc và sử dụng hết các dữ liệu đó.

## Robot hiện đang dùng phần nào?

Trong cấu hình điều khiển chân hiện tại, bộ điều khiển chọn MIT, gửi góc mục tiêu cùng tốc độ/gain/mô-men bù, và nhận lại góc, tốc độ cùng mô-men ước lượng. Về cơ bản, góc là lệnh chính để đặt/giữ tư thế khớp; encoder xác nhận chuyển động thực tế. Các lệnh cấu hình, hiệu chuẩn và chẩn đoán là khả năng của giao thức, không phải lệnh cân bằng gửi mỗi chu kỳ.

## Nguồn

- [Tài liệu tải xuống chính thức của SteadyWin](https://www.steadywin.cn/col.jsp?id=124) — mục GDS68 có manual GIM6010-8.
- [Trang thông số chính thức GIM6010-8](https://steadywin.cn/en/pd.jsp?fromColId=0&id=116).
- [Ghi chú giao thức CAN và triển khai trong repo](actuatal_driver/GDS68_DRIVER.md).
- [Mã đóng gói/giải mã dữ liệu động cơ](../src/tap_control/hal/include/robot_hal/MotorFrameCodec.hpp).

*Lưu ý: manual và trang thông số của nhà sản xuất có chỗ ghi độ phân giải encoder khác nhau; tài liệu này chỉ ghi “encoder tích hợp”, không chốt số bit. Mô-men trả về là giá trị ước lượng từ dòng điện, không phải cảm biến lực/tải.*
