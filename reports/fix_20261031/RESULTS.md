# Kết quả sửa firmware và OTA — 2026-10-31

## Đã thực hiện trên phần cứng

- Nạp USB CP2102 /dev/ttyUSB0 thành công; esptool xác minh hash flash.
- Đúng ESP32-S3 rev0.1 MAC 70:04:1d:a8:ca:fc.
- Kết nối Wi-Fi LUMI, IP 10.10.61.85, console TCP8023.
- Thử OTA thực tế: xác thực OK, truyền 100%, kết quả OK, reboot từ app0 sang app1.
- Đối chiếu đầy đủ 64 ký tự SHA256 ELF sau reboot; đúng board và armed=enabled=0.
- Image 785952 bytes; SHA256 binary `c0231f7497af81df7236e61fbbbffabf4e5a849f933949779a5388dd4d47eacb`.
- ELF SHA256 `fb23d10043382f8c60a0ad0930a2ca2a56dfc2ae63e84b67a392b536b5d0c5a1`.
- Không gửi ARM, target bật motor, MOTOR_HOME hay CAL_IMU trong lượt này.

Log: upload.log, ota_live.log, post_ota_status.json, post_ota_console.txt.
Sau reboot OTA supported=1, maintenance=ready=0 là bình thường: endpoint chỉ
mở khi công cụ nạp yêu cầu maintenance và xác minh OFF.

## Phần mềm đã sửa

1. Boot/disarmed không ghi tốc độ bánh. Mất reply của speed write chỉ được
   khôi phục bằng đọc lại target 0x20 đúng RPM; lỗi CRC/echo/TX/status hoặc
   readback sai vẫn fault. ACK vẫn cần hai bus và deadline 20 ms.
2. OFF được xác minh bằng READ 0x18; E-stop buộc kiểm lại mọi motor. Log ghi
   side/ID/instruction/address/error stage, tách lỗi mỗi worker với lỗi chung.
3. HELLO/epoch mới không xóa E-stop hay lỗi driver cục bộ. CLEAR_FAULT phải
   được yêu cầu riêng và chỉ sau verified OFF.
4. CAL_IMU thủ công thu cửa sổ ổn định 1 s/500 mẫu, hỗ trợ bias Y đo được
   khoảng -.2201 rad/s. Boot vẫn dùng ngưỡng bảo thủ. Không đổi mapping IMU.
5. Provision riêng mạng LUMI và khóa OTA; các file riêng tư 0600/git-ignored.
   Công cụ ota_upload.py kiểm manifest, SHA256, MAC, OFF trước nạp, firmware
   và trạng thái DISARMED sau reboot. Mật khẩu không truyền qua argv/log.
6. Maintenance khóa cả core và backend tới reboot; kiểm OFF rồi ngừng bus I/O
   trong lúc ghi flash. Nếu chưa xác minh OFF thì tiếp tục OFF, chưa mở OTA.

## Kiểm thử phần mềm

- check_bl_motor.py --runtime: PASS, ASan/UBSan; LF Core thật + backend/codec
  motor thật + hai mock bus, 2.700 chu kỳ giả lập, ACK và các ca lỗi/mất phản hồi.
- check_hybrid_gateway.py --runtime: PASS, ASan/UBSan, attitude/bias/gap/NaN,
  TTL/E-stop, maintenance; PTY UART; 2.700 chu kỳ LF Core.
- unittest test_flow_contract.py ở LF runtime: 6 PASS.
- unittest test_ota_upload.py: 8 PASS; không nạp khi sai image/key/MAC,
  active/stale/not-OFF; kiểm kết quả reboot, lỗi upload và fragmented console.
- pio run -e esp32-s3-hybrid-mcu: PASS. Log build.log.
- Repro trước/sau: trước boot ghi speed=0, HELLO xóa E-stop, bias làm IMU
  invalid; sau boot không ghi speed, E-stop giữ nguyên, calibration thủ công
  xử lý được bias. regression_before_after.log, regression_repro.cpp.

Trong bộ test có sẵn, sửa ba lỗi fixture: biến frame ngoài scope; test sensor
fault chưa thực sự ARM; test wrap sequence dùng STATUS có timestamp rollback.
Không sửa thuật toán để chiều theo các precondition sai đó.

## Lần OTA lỗi đầu và điều chỉnh

Lần đầu authentication thành công nhưng upload dừng ở 5%, OTA error=4; MCU
vẫn ở maintenance. Việc OFF polling tiếp tục trong lúc ghi flash có thể làm
mất điều kiện ready. Đã sửa backend maintenance: xác minh OFF rồi giữ bus
im lặng, khóa apply/reset độc lập. Có test cho verified/unverified OFF. Lần
OTA sau thay đổi này truyền đủ và xác minh reboot thành công. Lỗi đầu không
được tính là PASS; giữ ota_attempt1_failed.log và ota_failed_status.json.
Một lần gọi ngay sau USB reset chưa đủ status mạng, đã gọi lại sau Wi-Fi sẵn.

## Chưa hoàn thành: lỗi motor vật lý và hiệu chuẩn

Không kết luận cả robot đã sửa xong. Log sau OTA vẫn báo local driver fault,
IMU invalid và chưa homing. Bản mới khoanh vùng timeout reply ở wheel ID1,
thanh ghi Torque Enable 0x18 trên hai bus (L ghi; R đọc trong first-failure).
Sau retry vẫn đọc được enable_readback=[0 0 0], đủ để thử OTA, nhưng lỗi latch
vẫn còn. Chưa chứng minh nguyên nhân cuối là timing, driver hay điện.

Mất speed-write ACK là sai khác đã sửa theo bench và kiểm thử bằng mock;
chưa phải lời giải đã được chứng minh cho mọi lỗi bus thực tế. Khả năng READ
0x20, tần số motor/applied ACK, cơ chế wheel startup và đáp ứng servo còn cần đo.
Không tăng timeout/TTL, không giả validity, không xóa homing gate để ép robot chạy.

CAL_IMU thủ công chưa chạy trên board ở lượt này: cần robot được giữ yên ở
cách lắp đã xác nhận. Raw accel z sau nạp khoảng -10 m/s², khác mẫu trước +9.83;
không tự đảo mapping dựa vào tư thế lúc nạp. Sáu trục không phân biệt yaw đều
với bias; hiệu chuẩn đòi hỏi robot thực sự đứng yên. Homing là bước độc lập.
UART LF–ESP sau nạp chưa kiểm lại (rx_bytes=0); bài 3.000 packet hai chiều
CRC=0 là kết quả bản trước, không phải kết quả bản mới này.

Hướng dẫn OTA: firmware/esp32/OTA.md. Bản sao mã trước sửa nằm ở before/.
