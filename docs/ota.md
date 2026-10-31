# OTA tầng dưới qua LUMI

ESP32-S3 dùng Wi-Fi 2.4 GHz `LUMI`, DHCP, hostname `lf-mcu.local`.
IP đã đọc sau nạp USB ngày 2026-10-31: `10.10.61.85`.
MAC board đã nạp: `70:04:1d:a8:ca:fc`.

Wi-Fi và khóa OTA được cấp riêng trong các file git-ignored, quyền 0600:

- `include/gateway/HcrWifiSecrets.hpp`: mạng hybrid; ưu tiên trước mạng legacy.
- `include/gateway/OtaSecrets.hpp`: hash khóa OTA trong firmware.
- `.local/hybrid_ota.json` ở gốc repository: khóa OTA của laptop và MAC đích.

Không cần nhập lại mật khẩu khi dùng công cụ trên laptop này. Khi chuyển máy,
cần chuyển file credentials một cách riêng tư. Không dùng mật khẩu Wi-Fi làm
khóa OTA. Giao thức ArduinoOTA dùng xác thực digest trên LAN, không phải HTTPS.

## Nạp lần sau

Laptop cần truy cập được ESP trên LUMI. Tại thư mục `luckfox_gateway_esp32`:

```sh
python3 tools/ota_upload.py --build --host 10.10.61.85
```

Công cụ build đúng target hybrid, đối chiếu manifest/SHA256 của image và khóa
OTA; kiểm MAC board; từ chối nếu robot đang ARM. Sau đó gửi `OTA_BEGIN`, chờ
maintenance + verified OFF + trạng thái mới, nạp qua ArduinoOTA rồi xác minh
SHA256 ELF của firmware đang chạy sau reboot, đúng MAC và DISARMED.

ESP nhận lời mời UDP 3232 và kết nối ngược về laptop TCP 3233 để tải image.
Nếu mạng có client isolation/firewall chặn hai chiều thì phải xử lý kết nối
mạng. Không cần tháo dây LF khi nạp qua Wi-Fi; không dùng cổng UART để OTA.

```sh
# Đọc trạng thái, không nạp hay chuyển chế độ
python3 tools/ota_upload.py --status --host 10.10.61.85
# Kiểm image/manifest/credentials tại laptop, không kết nối ESP
python3 tools/ota_upload.py --check
```

Có thể dùng `--host lf-mcu.local` nếu laptop phân giải được mDNS. `NET` trong
console và UDP beacon port 8024 báo IP. `--mac` chọn board khác một cách tường
minh; không bỏ kiểm tra danh tính board.

## Chế độ bảo trì

OTA chỉ được mở sau khi driver cả sáu motor đã xác nhận OFF. `OTA_BEGIN` buộc
ghi/đọc lại OFF rồi dừng giao dịch bus trong thời gian bảo trì, tránh timeout
giả do ghi flash. Nếu chưa xác nhận OFF thì tiếp tục yêu cầu OFF và chưa mở
OTA. Backend tự khóa apply/reset; không tự xóa lỗi, homing hoặc ARM. Maintenance khóa
mọi TARGET/phiên LF mới đến reboot. Nếu nạp lỗi, tiếp tục giữ khóa; có thể thử
lại. `OTA_REBOOT` chỉ chấp nhận khi OFF được xác nhận và không đang nạp.
Sau reboot, firmware không ARM và phải hiệu chuẩn/home lại trước vận hành.

Nếu motor mất nguồn/bus không trả lời, công cụ không giả vờ đã OFF để nạp.
Khi đó sửa đường bus hoặc dùng USB. Nạp lần đầu để cài khóa OTA/partition cũng
cần USB: LF TX phải được tách khỏi ESP RX0/GPIO44 khi CP2102 dùng chân này.

Build/host tests và kết quả nạp thực tế được ghi riêng trong
[RESULTS.md](../reports/fix_20261031/RESULTS.md).
