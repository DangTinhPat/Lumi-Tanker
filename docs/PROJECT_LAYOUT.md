# Bố cục dự án

Tên thư mục viết thường; dùng `snake_case` cho file Python. Các include namespace
C++ (`tap_fw`, `robot_control`, `robot_hal`) và tên target build được giữ ổn định.

| Thành phần | Đường dẫn hiện tại | Đường dẫn cũ |
| --- | --- | --- |
| Controller robot | `controller/v3/` | `src_fw/` |
| Controller dùng chung | `controller/core/` | `src/tap_control/core/` |
| Test controller chung | `controller/core/tests/` | `src/tap_control/tests/` |
| HAL | `controller/hal/` | `src/tap_control/hal/` |
| Ví dụ C++ | `controller/examples/` | `src/tap_control/apps/` |
| ESP32 board và điểm vào | `firmware/esp32/` | `firmware_esp32/` |
| GUI robot | `gui/robot_gui.py` | `tools/tap_serial_control_gui.py` |
| GUI mô phỏng | `gui/simulator_gui.py` | `tools/tap_native_gui.py` |
| Thư viện GUI | `gui/tap_gui/` | `src/gui/gui/` |
| MuJoCo và model | `simulation/` | `src/tap_sim/` |
| CAD | `assets/cad/` | File STEP ở gốc repo |
| Datasheet | `assets/datasheets/` | `source_assets/datasheets/` |

## Điểm bắt đầu đọc code

- Robot thật: `firmware/esp32/src/`, sau đó
  `controller/v3/src/v3/V3Controller.cpp` và header tương ứng.
- Driver và mapping motor: `firmware/esp32/include/tap_hw/`,
  `controller/v3/include/tap_fw/`.
- GUI robot: `gui/robot_gui.py`; giao thức telemetry: `gui/tap_gui/telemetry.py`.
- Mô phỏng: `simulation/native/`, `simulation/model/`.
- Cấu hình build: `platformio.ini` và `CMakeLists.txt` tại gốc.

Các lệnh GUI cũ trong `tools/` vẫn được hỗ trợ qua wrapper. Build CMake mới
nên dùng thư mục build riêng; cache cũ lưu đường dẫn trước khi đổi bố cục.
MuJoCo chỉ được build khi bật `-DTAP_BUILD_SIMULATOR=ON`.

File Wi-Fi riêng nằm tại `firmware/esp32/include/tap_hw/WifiSecrets.hpp`
và được gitignore. Model và CAD được giữ lại; phần vision đã bị gỡ.
Tài liệu lịch sử và hai hợp đồng `architecture.md`, `docs/RULES.md` giữ nguyên;
dùng bảng trên để đối chiếu đường dẫn trong các tài liệu đó.
