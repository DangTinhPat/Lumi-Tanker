# Kế thừa sửa miss của BL4012e vào HCR — 2026-10-31

Bản này cập nhật tiếp baseline `c53d6de`. Đã nạp bản miss-recovery chính và
kiểm tra phần cứng khi motor OFF. Bản bổ sung log sau đó chưa nạp được do lỗi
OTA cuối phiên. Phần LF, HCR payload, mapping, gains và giới hạn không đổi.

## Đã triển khai

1. Dùng snapshot **mới** của `joint_bus.c`/`.h` từ BL4012e. Chính bản upstream
   mới dùng FIFO TX + nhả pad tại TX idle, gap100us, drain late replies, timeout
   joint/echo3ticks và wheel9ticks, khôi phục thiếu đúng một syncFF. Không thêm
   OEN override hoặc RMT vào adapter.
2. Giữ deadline tuyệt đối từ batch/lease xuống module bus. Chờ drain/gap,
   lượt đọc và lần retry không gia hạn thời hạn. Kiểm quyền gửi trước TX và
   kiểm lại trước khi chấp nhận reply; hết hạn không ACK. OFF được gửi độc lập
   với lease motion, vẫn phải đọc lại thanh ghi enable để xác nhận.
3. Sửa trường hợp drain đủ10lượt vẫn có byte: giữ latch và báo Quiet, không
   phát tiếp. Gap100us cũng áp dụng sau khi thu bỏ byte đến muộn.
4. `sync_recovered` chỉ tăng khi frame sau phục hồi đã qua đủ validation.
   `BUS_TRACE.rx` giữ byte nguyên gốc; `sync_restored` nói rõ đã thử bù sync.
   Không bù payload hoặc bỏ kiểm checksum để nhận một gói lỗi.
5. Worker retry tối đa1lần cho ping/read/absolute MIT cùng giá trị. Giữ deadline,
   kiểm cancellation; không retry status fault, Packet/Unavailable/Quiet hoặc
   lệnh quá hạn. DISARM giữa giao dịch dừng retry rồi OFF; không biến DISARM
   bình thường thành driver fault cần home lại.
6. Wheel speed vẫn cần ACK hợp lệ hoặc READ0x20 trả đúng RPM sau missing reply.
   Không nhập cách bỏ qua lỗi speed write của bench. Lệnh RPM thay đổi được
   gửi ngay. RPM trùng đã xác nhận trong cùng phiên enable được tái sử dụng
   tối đa100ms trước keepalive, còn READ encoder vẫn chạy mỗi batch/poll.
   OFF/reset xóa cache; startup vẫn preload0 và verify0 sau enable.
7. Thêm `BUS_RECOVERY`: timeout/echo/frame/partial/late/sync_accepted/quiet_fail/
   aborted, retries/recovered, skipped_speed_writes. First failure vẫn được giữ
   kể cả khi retry thành công; đọc cùng local_fault để phân biệt phục hồi/fault.

Các counter của module bus cộng dồn từ init. Counter worker và first failure
được reset theo reset_generation. `retries` là số lượt thử lại ở worker;
`transactions` là số lần gọi module, không phải số TX chắc chắn đã phát.
`aborted` có thể bao gồm hủy hợp lệ khi DISARM.

## Lịch bánh và thời gian

Không nhập slot10ms + luân phiên ghi/đọc của bench: khi target liên tục đổi,
feedback theo lịch đó không vượt khoảng50Hz và dễ quá tuổi20ms của HCR.
Bản này giảm traffic ở RPM không đổi, không tự hạ tần số target thay đổi.
Batch đầy đủ vẫn97byte/bus; batch bỏ ghi tốc độ trùng còn79byte, chưa kể gap,
thời gian trả lời và lịch task. Tần số200Hz thực tế vẫn phải đo trên robot.

Ngân sách20ms là điều kiện phần mềm, không phải bảo đảm cắt lực vật lý đúng
20ms. UART read đang blocking có thể trì hoãn xử lý DISARM đến lúc trả về;
TX critical section có trần1ms (thường theo độ dài frame), có thể trì hoãn
service500us trên cùng core. Chưa đo jitter UART0/IMU hay độ trễ OFF trên bản này.
Driver không có sequence trong reply nên drain/gap giảm nguy cơ lẫn late reply,
không chứng minh phân biệt được mọi phản hồi trễ tùy ý có cùng ID/opcode/address.

## Kiểm chứng

Lệnh từ thư mục repository:

```sh
python3 tests/check_inherited_bus.py
python3 tests/check_bl_motor.py --runtime '/home/dvt/contoller hybrid/deploy/hybrid_runtime'
python3 tests/check_hybrid_gateway.py --runtime '/home/dvt/contoller hybrid/deploy/hybrid_runtime'
python3 -m unittest discover -s tests -p test_ota_upload.py
/home/dvt/.platformio/penv/bin/pio run -e esp32-s3-hybrid-mcu
```

Test transport biên dịch C module thực và C++ adapter với SDK mock, ASan/UBSan.
Bao gồm gap sau reply/drain, bus không yên, FIFO busy/stuck, thiếu/sai echo,
reply sai checksum/ID/opcode, thiếu sync hợp lệ/không hợp lệ, raw capture,
deadline và cancellation trong gap/drain/read, không sửa output khi thất bại.
Negative controls phải phát hiện bỏ attach TX và clear drain khi bus còn bận.

Test backend có transient MIT/wheel-read failure, giữ lỗi đầu sau recovery,
không retry sau DISARM hoặc hết lease, RPM thay đổi thực sự được gửi, stable
RPM giảm WRITE nhưng không giảm READ, keepalive và reset cache khi enable lại.
Các kiểm tra cũ về units/FF/range/status/gain, ACK đủ hai bus, stale encoder,
TTL/replay/epoch/E-stop tiếp tục được chạy. LF Core interop dùng fake logic và
policy, không chạy model thật hoặc driver vật lý.

Kết quả: transport SDK-mock + ASan/UBSan, backend, LF Core interop 2700 chu kỳ,
gateway contract và 8 test OTA helper đều PASS. Build public và provisioned đều
PASS (Arduino 3.3.0 / IDF 5.5, pioarduino 55.03.30-2). Cảnh báo công cụ size
`--ng` không được hỗ trợ; kiểm size trực tiếp thành công ở bản miss chính.
Build firmware và kiểm image checksum/hash không chứng minh OTA/hardware đạt.

Bổ sung `FEEDBACK_DIAG` trong source cuối phiên: reason 1=joint jump,
2=wheel range, 3=wheel gap, 4=wheel ambiguous; ghi ID, count trước/sau,
gap_us và active. Test tái hiện scheduling gap lúc OFF và reset diagnostics PASS.
Không thay ngưỡng feedback20ms. Callback OTA giữ Update error gốc thay vì ghi đè
bằng Aborted; trường `update_error` bổ sung vào STATUS. Hai phần log bổ sung này
đã build nhưng **chưa chạy trên ESP**.

## Phần cứng đã đo và giới hạn

- OTA đầu tiên từ bench sang HCR thành công, cùng MAC, đổi slot, đúng ELF;
  `BUS_DRIVER source=bl4012e_3in1_miss_v2`. Ảnh đang chạy:
  bin SHA256 `030289bb54577343b1260e63d06a195725fb892f08de66f68561bad3d850ad5c`,
  ELF `5b3ca7c9b5bbeaade6a22723fda3d9a92c0c7ddf4ebeb6871c939dc288254db8`.
- MOTOR_BUS_TEST hoàn thành 2000 vòng OFF mỗi bus (4000 tổng). Sau test:
  trái timeout1/retry1/recovered1; phải echo2/retry2/recovered2, chưa fault.
  Cả sáu enable readback=0. Đây là giao dịch OFF/đọc/ghi tốc độ0, không thử MIT
  bật motor hoặc cân bằng. Không suy ra chu kỳ chuyển động200Hz đã đạt.
- Sau đó xuất hiện local motor error6=Feedback ở bus trái, làm backend trip.
  First bus failure lưu từ trước đã phục hồi; không được coi trace đó là
  nguyên nhân trực tiếp của Feedback. Chưa biết chắc là khoảng cách mẫu quá20ms,
  count sai range, jump hay ambiguous. Vì vậy bổ sung log lý do ở source.
- CAL_IMU nhận500 mẫu, imu_valid=1, bias gần [0.0083,-0.2407,0.0097]rad/s;
  gyro sau trừ bias gần0. Calibration mất khi reboot/OTA maintenance.
- Test peer độc lập15s ở921600baud: LF nhận30 HELLO hợp CRC (1080bytes),
  CRC error0. LF ghi1620bytes, nhưng ESP rx_bytes=0, HELLO_ACK=0, telemetry=0.
  Đã chứng minh ESP→LF; chưa chứng minh LF TX phát điện hoặc ESP RX nhận.
  LF ttyS2 pinmux uart2m1-xfer đúng. CP2102/RX0 physical contention chưa xác minh.
- Đã khởi động lại LF runtime, ở WAIT_LINK, chưa ARM, chưa HOME, chưa chuyển động.
- OTA bản log bổ sung thất bại hai lần ở END (`error=4`, phản hồi Aborted).
  Đối chứng nạp lại đúng ảnh đang chạy cũng thất bại giống vậy. Firmware cũ
  vẫn nguyên ởapp0. Callback hiện hành che lỗi Update gốc, nên chưa có bằng
  chứng kết luận MD5, flash, activation hay mạng. Ảnh mới kiểm checksum/hash
  bằng esptool đều hợp lệ. Đã reboot về ảnh cũ, không để board kẹt maintenance.
  Sau reboot cuối: imu_valid=1, hiệu chuẩn500 mẫu, fault=0 tại thời điểm chụp,
  sáu motor OFF, HOME chưa đặt; không coi reset là sửa dứt điểm Feedback fault.
  Cần nạp bản chẩn đoán qua USB để lấy mã lỗi gốc nếu OTA vẫn không hoạt động.

## Ảnh source cuối phiên (chưa nạp)

- Public: 1002448 bytes; SHA256 `6503e21eac337db8e4e52c6bd6d5860353942ac6d58b0f918035630fd9727239`; ELF `3d8a9271a30444aafbda2cf4570ca3674020c918dab7b19d224f27f58159b095`.
- Provisioned: 1013296 bytes; SHA256 `029d693dedecf4952deca3aafc9212a308c749599c1f46188ebd16bd3fa7d461`; ELF `24fced30889d2b15d0f67b37ec0a2b4c6dd8384b17619be92b14ec3941d900dc`.

## Còn phải kiểm

Giải quyết OTA và đường LF→ESP; đo Feedback fault bằng log mới; sau đó mới kiểm
handshake/telemetry/TARGET disabled và ACK đủ hai bus. Chỉ chụp HOME/thử chuyển
động theo tư thế và phiên thử được xác nhận. Không nâng freshness/TTL để đạt test.
Log thô lưu riêng trong `.local/miss_integration/` (không đưa thông tin mạng
hoặc credentials vào repo).

[Provenance và delta nguồn](../drivers/bl4012e/PROVENANCE.md).
[Báo cáo baseline](validation-bl4012e-reuse.md).
