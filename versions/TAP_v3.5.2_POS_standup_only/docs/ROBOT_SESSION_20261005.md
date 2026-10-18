# Buổi thử robot 05/10 (chiều) — kịch bản một buổi

Mục đích: thử firmware mới nhất (đã gỡ VMC, lead thích ứng theo độ cao, trim_acc, suspension thích ứng
nhiễu). Chỉ là bằng chứng cho model; không chứng nhận khả năng robot thật. Thứ tự từ ít rủi ro đến nhiều;
dừng ở bước lỗi. Mọi thứ đều đã đưa vào mã boot — **không cần gửi SET nào để chạy bản chuẩn**.

Firmware: `pio run -e esp32-s3-v3`, `firmware.bin` SHA256
`d521b033c3fd42e41ef66c4e80846e2b39f486307731e7f47bb24937149f154a` (mã chưa commit; kiểm lại sau build).

## 0. Nạp và kiểm tra boot
1. Đóng GUI serial (nó giữ cổng; mở cổng cũng reset ESP32 và mất mọi SET chạy). Cổng đổi giữa `ttyUSB0/1`.
   `~/.platformio/penv/bin/pio run -e esp32-s3-v3 -t upload --upload-port /dev/ttyUSBx`
2. Mở lại GUI: `python3 tools/tap_serial_control_gui.py --port /dev/ttyUSBx`; log ở `~/tap_logs`.
3. Dòng boot phải có: `V3 TUNE suspension: tsus=1 adaptive=1`, `V3 TUNE lqr lead: adaptive=1`; **không có** `knee vmc`.

## 1. Khởi động + đứng yên (mục tiêu: hết "nhúng lên xuống")
- START như mọi lần. Đứng yên **30 s** ở chiều cao mặc định (176.6 mm).
- Kỳ vọng: `h` trong ±1–2 mm quanh `htgt`, `sus` ≈ 0 trong 6 s đầu (đang chờ), `sdb=` hiện vùng chết > 5 mm/s.
- Nếu vẫn nhún: `SET tsus 0` (tắt hẳn suspension) để phân biệt có phải do suspension.

## 2. Hạ chiều cao (mục tiêu: không ngã, không phải bấm-nhả nhiều lần)
- Hạ từng nấc (giữ ▼): 150 → 130 → 110 → 100 → 90 mm, mỗi nấc đứng ~10 s. **Đừng xuống dưới ~85 mm.**
- Xem: `lqrlead` (`ql=`) tăng khi thấp, pitch không dao động 1–2 Hz, không trôi vị trí lớn sau khi hạ (trim_acc).

## 3. Đẩy / tăng-giảm tốc ở thấp và cao
- Đẩy nhẹ thân (tay) ở 176 mm rồi ở ~100 mm: kỳ vọng thân bớt nghiêng theo (hông chia tải lớn hơn ở thấp: texp 0.15).
- Chạy nhẹ 0.3 m/s tới/lui, dừng dứt khoát.

## Hoàn tác nhanh (không cần nạp lại)
`SET tsusad 0` (suspension cũ) · `SET tsus 0` · `SET lqradapt 0` rồi `SET lqrlead 0.08` · `SET ttrimacc 0` · `SET texp 1.5`.

## Dừng ngay khi
Hông/thân dao động tăng dần, tiếng kêu to lên, FAULT, `axis_error`, gối võng: `DISABLE` (hoặc ESTOP). Gửi log, không thử thêm.

## Sau buổi thử
```bash
python3 tools/hip_chatter_stats.py ~/tap_logs/tap_<log mới nhất>.log --window 10
```
`alt%` là dấu hiệu rung hông (0–2 % êm; 28–50 % kêu). Ghi giờ từng bước để khớp với log.
