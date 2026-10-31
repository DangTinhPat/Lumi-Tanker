# Kết quả kiểm chứng LF–ESP32, 2026-10-31

## Trạng thái cuối

LF dùng UART2_M1 `/dev/ttyS2`, 921600/8N1; ESP UART0 TX43/RX44.
Lượt peer cuối nhận 2.998 TELEMETRY trong 15 s, 199,8603 Hz, CRC=0.
15 mẫu được in đều flags=0x07; IMU, encoder và HOME hợp lệ.
ACK TARGET DISABLED tăng tới applied_seq=2807 ở mẫu cuối. Không gửi TARGET enable.
HOME q=[2.357067108,2.357067108,0,0,0,0] rad sau xác nhận tư thế của người dùng.

Sau khi trả UART cho runtime thật: LF DISARMED/fault_code=0, link=true,
38.809 telemetry nhận và hợp lệ, rejected=0, parser_errors=0, inference=0.
ESP armed=0/enabled=0/local_fault=0; hai bus configured=1/off=1/homed=1,
bus_err=0, enable_readback=[0,0,0], startup_goal_rpm=0.
UART stale_tx và cả ba queue-full counter đều 0; pump_gap_max_us=4593.
IMU bias đo được [0.00876,-0.22907,0.01272] rad/s, gyro đã bù gần 0,
gravity xấp xỉ [0,0,-1]. Bias và HOME là RAM, mất sau reboot/OTA.

## Nguyên nhân và thay đổi

- LF service cũ dùng Unix socket. Đã sửa serial config và triển khai bản mới.
- Motor wheel phản hồi nhanh: TX pad nhả muộn làm mất/corrupt đầu gói.
  Trace cũ: write0x18 TX195us, reply6/9; read0x18 TX130us, reply7/10.
  Nay nạp FIFO, đợi TX idle trong critical section hữu hạn, nhả pad bằng
  gpio_ll_output_disable; bỏ thao tác đổi matrix dư trước output-disable.
  Không nới timeout/TTL, không bỏ checksum hoặc dùng retry che lỗi.
- UART HCR có task riêng priority17/core1; console/Wi-Fi/OTA không giữ parser.
  Service priority20 và motor workers18 giữ nguyên. Thêm chẩn đoán queue/trace.
- Bias gyro Y vượt ngưỡng boot .20 rad/s. CAL_IMU có xác nhận đứng yên đã
  hiệu chuẩn thành công, không tự coi gyro lớn là bias ở boot.
- Chụp MOTOR_HOME sau khi người dùng xác nhận đúng tư thế, không ra lệnh di chuyển.
- GUI/LF phân biệt telemetry_received và telemetry hợp lệ cho điều khiển.

## Bằng chứng và giới hạn

- Firmware build và OTA thành công; running ELF SHA256:
  f5bf991c10cec3a01a2eb5fc5db547b76d4ab1e5642db7050fff19c4ddf44931.
- Firmware binary SHA256: ba2f556f957a67180f7e449e7151386a7cb6050dff4f3010afa75f7114ffbdd7.
- 23 runtime Python tests PASS (`hybrid_runtime/reports/uart_fix_20261031/tests.log`).
- Motor/core/codec tests PASS, real LF Core + mock motor bus 2.700 cycles;
  xem motor_tests.log. Mock không chứng minh servo đáp ứng hoặc robot cân bằng.
- UART vật lý: uart_final.json/log; trạng thái thực: esp_final_status.txt,
  lf_final_status.txt; HOME: home_capture.txt; IMU: imu_calibration.txt.
- uart_final.json ghi ESP transport fault sau khi peer kết thúc và trước khi
  wrapper restart runtime: đó là khoảng ngắt link có chủ đích. Trạng thái sau
  restore trong esp_final_status.txt/lf_final_status.txt là link=1/fault=0.
- stable_30s là kết quả trung gian còn lỗi header; không phải bản PASS cuối.
  Bản cuối dùng release_build.log/release_ota.log/release_25s.txt và uart_final.
- Service missed wakeups vẫn có (1739 tích lũy, tick_max_us=643); không đồng
  nghĩa packet loss. Chưa chứng minh hard real-time hoặc tải khi motor bật.
- Chưa ARM, chạy stand-up, chạy model trên robot hoặc kiểm đáp ứng motor.
  TARGET vẫn 4 góc hip/crank +2 vận tốc bánh, không đổi sang torque.

## LF deployment bền vững

Rootfs EXT4 không remount RW được do unprocessed orphan inode list.
Không fsck root đang mounted. Bản chạy nằm `/userdata/rb-r1/hybrid_runtime`,
bind-mount vào `/root/rb-r1/contoller hybrid/deploy/hybrid_runtime`.
Hook `/oem/usr/etc/init.d/S95rbhybridmount` chạy trước root S96rbhybrid qua
RkLunch.sh. Mount/service restart đã xác minh; chưa thử reboot LF sau thay đổi.
Rootfs cần bảo trì offline riêng. Runtime đã được khôi phục làm chủ UART.
