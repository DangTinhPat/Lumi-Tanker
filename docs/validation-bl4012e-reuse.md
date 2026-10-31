# Kế thừa bus BL4012e/3in1 vào HCR — 2026-10-31

**Lịch sử baseline c53d6de.** Bản mới kế thừa sửa miss/turnaround của 3in1:
[xem báo cáo hiện tại](validation-bus-recovery.md).

Bản ứng viên thay phần truyền UART tự viết bằng **module C của 3in1**.
Chưa nạp ESP, chưa ARM, chưa thử motor trong đợt sửa này. Đây không phải kết
luận lỗi vật lý đã hết. Bản bench 3in1 của người dùng được giữ nguyên.

## Phạm vi thay đổi

```text
LF HCR1 TARGET
  → GatewayCore (epoch/source/TTL/enable)
  → BlMotorBackend (sign/zero, rad→counts, rad/s→RPM, batch hai bus)
  → EspMotorBus (adapter C++ mỏng)
  → bus_ping / bus_read / bus_write trong joint_bus.c của 3in1
  → ESP-IDF UART driver → pad một dây → motor ID2/ID3/ID1
```

- Snapshot nguồn gốc và SHA256 ở `drivers/bl4012e/upstream/`.
- Module chạy ở `drivers/bl4012e/src/joint_bus.c`. Delta so với nguồn gốc:
  phân loại lỗi, lưu byte/timing, kiểm số byte ghi; xem [provenance](../drivers/bl4012e/PROVENANCE.md).
- Bỏ custom TX FIFO, lựa chọn OEN, spinwait trong critical section và RMT
  can thiệp pad khỏi adapter. Drive/release/echo/reply do module 3in1 thực hiện.
- Pin, ID, sign, zero, gains, giới hạn, đơn vị và HCR payload không thay đổi.
- Giữ hai worker độc lập, ACK sau cả hai bus, timestamp encoder và fault latch.
  Echo/TX lỗi không được biến thành missing-reply để đi vào nhánh readback bánh.
- `BUS_DRIVER` báo nguồn/IDF; `BUS_BYTES` và `BUS_TRACE` giữ bằng chứng giao dịch.
  `MOTOR_WAVE` báo unavailable; không xuất số giả thay cho phép đo pad đã bỏ.
- Pin platform `pioarduino 55.03.30-2`, Arduino 3.3.0, IDF 5.5.0 để cùng thế hệ
  UART SDK với bench. Đây là thay đổi SDK toàn firmware: IMU/UART0/Wi-Fi/OTA
  cũng phải được kiểm lại khi nạp; không coi thư viện Arduino giống app IDF bench.
- Bỏ build hook controller cũ khỏi target hybrid của repository export;
  header cấu hình còn cần thiết. Logic đứng dậy/model tiếp tục ở LF.

## Những phần khác bench vẫn có chủ đích

Không nhập `leg_move()` vì đó là lệnh delta và bộ sinh quỹ đạo của GUI bench;
HCR nhận reference tuyệt đối từ LF. Cũng không nhập wheel ramp 600 RPM/s,
MISS_LIMIT25 hoặc cách bỏ qua lỗi speed write. Không kết luận lần chạy bench
thành công của người dùng là do các cơ chế này che lỗi; chưa có log để nói vậy.

Bench nominal 4 ms, wheel write/read luân phiên; HCR nhận TARGET mỗi 5 ms và
một batch đầy đủ gồm 2 MIT + wheel speed write + present read. Sau startup,
wire bytes/bus/batch là **97**: 2×(17+11)+(9+9)+(8+15).
READ `0x24` lấy **6** byte dữ liệu, response15. Echo là tự thu TX, không thêm
một lần truyền trên dây. Tải truyền thô HCR khoảng19,4%, bench khoảng19,125%
ở1Mbaud8N1; vì vậy không có căn cứ nói ba motor/bus tự nó gây nghẽn.
Khoảng chờ driver, lịch task, startup và timeout không nằm trong tỷ lệ này.

Timeout kế thừa: TX-done5 ms, echo2 ms, reply joint2 ms hoặc wheel6 ms.
Đây là từng lượt chờ; `uart_write_bytes` không có timeout argument. Không còn
cam kết tổng giao dịch4/8 ms của adapter cũ. TTL20 ms vẫn được kiểm giữa các
giao dịch và trước khi ACK; OFF có thể phải đợi giao dịch đang blocking trả về.
Không gọi TTL này là hard deadline cắt lực vật lý khi bus/driver không đáp ứng.

## Bằng chứng phần mềm

Chạy từ thư mục repository:

```sh
python3 tests/check_inherited_bus.py
python3 tests/check_bl_motor.py --runtime '/home/dvt/contoller hybrid/deploy/hybrid_runtime'
python3 tests/check_hybrid_gateway.py --runtime '/home/dvt/contoller hybrid/deploy/hybrid_runtime'
python3 -m unittest discover -s tests -p 'test_ota_upload.py'
/home/dvt/.platformio/penv/bin/pio run -e esp32-s3-hybrid-mcu
```

- Test module: so sánh transcript các lời gọi UART/GPIO của upstream và candidate
  (pin/config/thứ tự/timeout), golden MIT/position/wheel, lỗi echo/CRC/timeout.
- Test adapter thực cùng module C: short TX, TX wait timeout, thiếu/sai echo,
  thiếu reply, sai header/ID/length/checksum/opcode/address echo; reject broadcast,
  payload không hợp lệ; không thay output khi lỗi. Xác nhận batch97 byte.
- Negative control bỏ attach TX phải fail; ASan/UBSan bật cho host tests.
- Backend/core test: two-bus ACK, đơn vị, FF, freshness, TTL/cancel, wheel readback,
  lỗi status/gain; thêm EchoTimeout vào nhóm lỗi không được readback che mất.
- LF Core + MCU/motor backend giả lập: 2.700 telemetry exchanges đi qua
  HOME/STANDUP/WARMUP/MODEL; kiểm E-stop/reset và trường hợp thiếu motor.
- OTA helper: 8 bài kiểm tra trên mock; không thực hiện OTA vật lý.

Tất cả các lệnh kiểm tra trên đã PASS. Toolchain:
`framework-arduinoespressif32 3.3.0`, `framework-arduinoespressif32-libs
5.5.0+sha.b66b5448e0`, Xtensa GCC14.2.0+20241119, esptool5.0.2.

| Artifact | Build | RAM tĩnh (PIO) | Flash (PIO) | File .bin |
|---|---|---:|---:|---:|
| Baseline trước sửa, Arduino2.0.17/IDF4.4.7 | PASS | 62.292 B | 794.745 B | 795.104 B |
| Ứng viên public, không chứa credentials | PASS | 60.856 B | 998.467 B | 998.864 B |
| Ứng viên worktree gốc, có provisioning riêng | PASS | 62.320 B | 1.009.259 B | 1.009.664 B |

Kích thước tăng bao gồm thay đổi SDK, không quy toàn bộ cho module bus.
SHA256 binary public:
`5cb10a76da4c768178181d50fb307beff11d2fba7b393e1e0dc51fdd41b8e527`.
SHA256 binary có provisioning, giữ ngoài Git:
`5b29d15115ae1b19757b180d57a865ed2989a1e277620f1d754f8651b99327db`.
Bản public không có Wi-Fi/OTA credentials; không dùng nhầm nó thay cho bản
đã provision khi chuẩn bị nạp lên board hiện tại.

So sánh ELF bằng `xtensa-esp-elf-nm -C --defined-only`: hai bản ứng viên có
`joint_bus_init`, `bus_ping`, `bus_read`, `bus_write`; không còn
`EspMotorBus::readExact`, các hàm pad capture, `rmt_config` hay codec motor C++
cũ trong binary. Baseline có các hàm này. Không có controller cân bằng cũ hoặc
`EmbeddedController` trong hai binary ứng viên; baseline cũng đã loại bỏ các
controller không dùng khi link, dù hook cũ vẫn biên dịch source của chúng.

Bản worktree gốc còn qua `python3 tools/ota_upload.py --check`:
manifest, hash ảnh/ELF và provisioning hợp lệ; lệnh này không truy cập board
hoặc upload. Wi-Fi/OTA secrets và các target legacy trong worktree gốc được giữ.

Có cảnh báo từ thư viện Arduino touch API cũ (không dùng trong HCR). Helper
metrics của platform55.03.30-2 còn truyền `--ng` mà `esp-idf-size2.3.2` không
nhận; đây là lỗi báo cáo bổ sung, build/link/image và kiểm dung lượng PIO vẫn
PASS. Đã chạy trực tiếp `python -m esp_idf_size firmware.map` không có `--ng`
và nhận báo cáo bộ nhớ thành công. Không sửa thư viện global để giấu cảnh báo.

Log build/test, ELF baseline và bảng symbol được giữ cục bộ tại
`.local/inherit_bl4012e/` trong repository gatewayLF; binary/credentials không
đưa lên Git. Các kết quả ở đây là source/build/mock, không phải hardware PASS.

## Chưa được xác minh trên phần cứng

Log cũ khoanh vùng UART1/GPIO17 mất echo ở đầu chuyển động. Mức pad không có
xung thấp trong lần capture đó chưa phân biệt hoàn toàn driver UART/direction
với tác động điện trên đường dây. Protection HCR giải thích tại sao dừng sau
lỗi; nó không giải thích tại sao giao dịch bị mất. Không quy lỗi cho model/NPU,
băng thông bus hay pin mới chỉ từ các số liệu này.

Khi người dùng yêu cầu nạp/thử, kiểm lần lượt:

1. Đúng firmware/IDF, UART0 HCR hai chiều, IMU, console và OTA.
2. Cả hai bus configured/OFF, sáu encoder mới, error không tăng; OFF test nếu cần.
3. HOME/IMU đúng tư thế trước phiên chuyển động được xác nhận; thu first-failure,
   cycle_max_us, tuổi encoder và độ trễ applied ACK cho cả hai bus.
4. Nếu còn lỗi, đối chiếu bench và HCR trên cùng dây/nguồn/motor, thay từng yếu tố;
   dùng phép đo TX/line độc lập nếu cần. Không hạ bảo vệ để biến lỗi thành PASS.
