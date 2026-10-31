# Tích hợp motor HCR — kết quả 2026-10-30

Đã tích hợp backend BL4012e vào firmware ESP32-S3 HCR. Source và binary nằm
trên máy phát triển; **chưa flash hoặc vận hành motor**.

- Trái: UART2/GPIO15; phải: UART1/GPIO17. Hip ID2, crank ID3, wheel ID1.
- 4 góc tuyệt đối + 2 vận tốc bánh; MIT tau_ff=0. Crank FF=hip FF+knee FF.
- Homing thủ công, timestamp từng encoder, wheel unwrap, ACK sau cả hai bus,
  pending/disable bất đồng bộ, không ACK enqueue. Chặn ARM/capture khi OFF chưa xác nhận.
- Wheel startup: preload0 + enable + READBACK0; HOME zero ACK sớm,
  nonzero vẫn đợi25ms. Reference sau filter/quantization được trả về LF.
- Lỗi ghi/CRC/status/timeout/range/encoder stale gây hủy cả hai bus và OFF.

## Kết quả cuối

| Bài kiểm | Kết quả | Log |
|---|---|---|
| Build esp32-s3-hybrid-mcu | PASS; RAM53812B, flash769445B | build.log |
| Backend/codec motor, ASan/UBSan | PASS | motor_tests.log |
| LF Core → GatewayCore → backend thật → hai driver giả lập | PASS2700 chu kỳ, HOME/STANDUP/WARMUP/MODEL, E-stop/reset | motor_tests.log |
| Gateway core/codec, UART PTY, baseline interop | PASS | gateway_tests.log |
| Runtime LF unit/contract/ONNX reference | 22PASS | runtime_tests.log |
| Đồng bộ docs/codec và hashes | 5 cặp khớp,35 release hashes,80 source hashes | source_inventory.json |
| Python syntax và git diff --check | PASS | thực chạy trên host |

Motor regression gồm golden checksum69, mapping trái/phải và dấu, góc crank,
velocity FF, reference sau lượng tử/lọc, không ACK một bus, enable bits,
startup zero/nonzero, wheel wrap1020→5, stale read, mất write/enable reply,
stop giữa transaction, expiry khi service ngừng, gain max/status/CRC/config
fault và RESET không làm mất home tốt. Test gateway dùng backend mới ghi
first HOME ACK5600us trên **clock/bus giả lập**; không phải latency đo trên ESP.

Lệnh tái hiện từ worktree:

```sh
python3 firmware/esp32/tests/check_bl_motor.py --runtime '/home/dvt/contoller hybrid/deploy/hybrid_runtime'
python3 firmware/esp32/tests/check_hybrid_gateway.py --runtime '/home/dvt/contoller hybrid/deploy/hybrid_runtime'
/home/dvt/.platformio/penv/bin/pio run -e esp32-s3-hybrid-mcu
```

Runtime tests từ thư mục hybrid_runtime:

```sh
python3 -m unittest discover -s tests -p 'test_*.py' -v
python3 tests/check_flow_sources.py --mcu-root /home/dvt/luckfox_gateway_esp32 --motor-root /home/dvt/BL4012e --report /home/dvt/luckfox_gateway_esp32/reports/motor_integration_20261030/source_inventory.json
```

## Giới hạn độ fit

`fit.json` ghi nguồn GUI/settings và đính chính của người dùng. ID/chiều/bus và
giao diện dữ liệu đã đối chiếu; zero theo model chưa được đo trên robot.
Kp10/Kd.7 theo GUI được công khai cho ba profile, không tương đương mặc định
servo MuJoCo PD30/1 hoặc recovery PI/coupling. Không thêm trajectory/ramp của
bench, không tự clamp gain hoặc nâng wheel cap. Cần đo gain/max/giới hạn lực,
đáp ứng/timing thật và xác nhận READ0x20 trước chạy hybrid trên robot.

Các bài interop không chạy NPU hoặc plant cơ học. HIL33s trước đây dùng servo
fixture cũ, không chứng minh backend mới. Lỗi jitter NPU16.8ms của audit trước
vẫn là điểm còn mở; không sửa deadline/model/logic LF trong đợt tích hợp này.

Binaries đã lưu ở `artifacts/`, có SHA256SUMS. Hướng dẫn source/calibration:
`firmware/esp32/MOTOR_INTEGRATION.md` tại gốc worktree.
