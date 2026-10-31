// This translation unit supplies setup()/loop() only in the HCR1 build.
#ifdef TAP_HYBRID_GATEWAY
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <esp_ota_ops.h>
#include <driver/uart.h>
#include <esp_timer.h>
#include <lwip/sockets.h>
#include <algorithm>
#include <cerrno>
#include <atomic>
#include <cstdio>
#include <cstring>
#include "tap_fw/BoardConfig.hpp"
#include "tap_hw/Mpu6500Spi.hpp"
#include "tap_hw/HybridGatewayCore.hpp"
#include "tap_hw/EspBlMotor.hpp"
#if __has_include("tap_hw/OtaSecrets.hpp")
#include "tap_hw/OtaSecrets.hpp"
#endif
#ifndef HCR_OTA_PASSWORD_HASH
#define HCR_OTA_PASSWORD_HASH ""
#endif
#if __has_include("tap_hw/WifiSecrets.hpp")
#include "tap_hw/WifiSecrets.hpp"
#endif

// User-selected TX0/RX0 on the N16R8 PCB. UART0 is binary HCR1 only.
#ifndef HCR_UART_TX
#define HCR_UART_TX 43
#endif
#ifndef HCR_UART_RX
#define HCR_UART_RX 44
#endif
static_assert(HCR_UART_TX != HCR_UART_RX, "HCR1 needs separate TX and RX");
static_assert(ARDUINO_USB_CDC_ON_BOOT == 1, "Console must not use LF UART0");

namespace {
using namespace tap_hw::hybrid;
// Hybrid PCB installation reported 2026-10-30: chip +X forward, +Y toward
// robot-right, component face down (+Z down). Convert into the body frame
// (+X forward, +Y left, +Z up). Keep this mapping local to HCR; the older V3
// app has a separate, previously measured mounting configuration.
constexpr tap_fw::imu_mount::AxisMap kHybridImuAxisMap{
    {0U,1U,2U},{1.0F,-1.0F,-1.0F}};
static_assert(tap_fw::imu_mount::isRotation(kHybridImuAxisMap),
              "Hybrid IMU axis map must preserve handedness");
constexpr uart_port_t kLfUart = UART_NUM_0;
struct RxEvent {
  bool sent = false;
  std::uint64_t received_us = 0;
  HcrFrame frame{};
};
struct Command { std::uint32_t op = 0; float vx = 0, wz = 0; std::array<float,4> pose{}; };
constexpr std::uint32_t kEstop = 100, kCalibrate = 101, kMotorHome = 102, kClearFault = 103;
struct Report {
  std::uint64_t stamp_us = 0;
  Diagnostics state{};
  ImuObservation imu{};
  BlMotorDiagnostics motor{};
  std::array<float,3> raw_gyro{};
  std::uint32_t imu_samples = 0;
  std::uint8_t imu_id = 0;
  std::uint32_t service_max_us = 0, missed_ticks = 0, command_result = 0, command_op = 0;
};
QueueHandle_t rx_queue = nullptr, tx_queue = nullptr, command_queue = nullptr, report_queue = nullptr;
TaskHandle_t service_task = nullptr;
esp_timer_handle_t service_timer = nullptr;
std::atomic<bool> transport_fault{false};
std::atomic<bool> estop_requested{false};
std::atomic<bool> maintenance_requested{false}, maintenance_ready{false};
std::atomic<std::uint32_t> maintenance_tick_ms{0};
bool ota_started = false, ota_uploading = false;
int ota_error = -1;
unsigned ota_percent = 0;
bool uart_ready = false;
std::uint32_t parser_errors = 0, queue_errors = 0, uart_rx_bytes = 0, uart_tx_bytes = 0;
HcrParser parser{};
tap_hw::Mpu6500Spi imu_driver;
EspBlMotor motor_system;
BlMotorBackend& motors = motor_system.backend;
GatewayCore core(motors);
GravityEstimator attitude;
WiFiServer console_server(8023);
WiFiClient console_client;
WiFiUDP discovery;
Report latest_report{};

bool due(std::uint64_t now, std::uint64_t& next, std::uint64_t period) {
  if (now < next) return false;
  next = next ? next+((now-next)/period+1)*period : now+period;
  return true;
}

bool queuePacket(const WirePacket& p) {
  if (!p.size) return true;
  if (xQueueSend(tx_queue, &p, 0) == pdTRUE) return true;
  transport_fault.store(true); return false;
}

void timerCallback(void*) { xTaskNotifyGive(service_task); }

void service(void*) {
  const bool imu_present = imu_driver.begin();
  // The timer starts after initialization; no catch-up burst after SPI reset.
  if (esp_timer_start_periodic(service_timer, 500) != ESP_OK) {
    core.stop(LocalFault::Transport); vTaskDelete(nullptr); return;
  }
  std::uint64_t next_telemetry = 0, next_hello = 0, next_report = 0;
  ImuObservation imu{};
  Report report{}; report.imu_id = imu_driver.whoAmI();
  for (;;) {
    const auto ticks = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    const auto now = static_cast<std::uint64_t>(esp_timer_get_time());
    if (ticks > 1) report.missed_ticks += ticks-1;
    if (transport_fault.exchange(false)) core.stop(LocalFault::Transport);
    if (estop_requested.exchange(false)) core.estop();
    if (maintenance_requested.load() && !core.maintenance()) core.enterMaintenance();
    // Watchdog and driver service always run before draining UART events.
    core.tick(now);
    maintenance_ready.store(core.maintenanceReady());
    maintenance_tick_ms.store(static_cast<std::uint32_t>(now/1000));
    {
      tap_hw::Mpu6500Spi::Sample chip{};
      if (imu_present && imu_driver.readFresh(chip)) {
        const auto accel = tap_fw::imu_mount::toBody(kHybridImuAxisMap, chip.accel_m_s2);
        const auto gyro = tap_fw::imu_mount::toBody(kHybridImuAxisMap, chip.gyro_rad_s);
        report.raw_gyro = gyro; ++report.imu_samples;
        imu = attitude.update(accel, gyro, now);
        core.setImu(imu);
      }
    }
    RxEvent event{};
    // Bounded work per tick. No old queued packet can extend the 20 ms TTL.
    for (unsigned n = 0; n < 8 && xQueueReceive(rx_queue, &event, 0) == pdTRUE; ++n) {
      if (event.sent)
        core.telemetrySent(hcr_u32(event.frame.payload), event.frame.seq, event.frame.stamp_us);
      else
        queuePacket(core.receive(event.frame, now, event.received_us));
    }
    Command command{};
    for (unsigned n = 0; n < 4 && xQueueReceive(command_queue, &command, 0) == pdTRUE; ++n) {
      report.command_op = command.op; report.command_result = 1;
      if (core.maintenance()) {
        if (command.op == kEstop) core.estop();
        report.command_result = 2; continue;
      }
      if (command.op == kEstop) {
        core.estop();
      } else if (command.op == kClearFault) {
        report.command_result = core.clearLocalFault() ? 1 : 2;
      } else if (command.op == kMotorHome) {
        report.command_result = !core.diagnostics(now).armed &&
            motors.captureHome(command.pose,now) ? 1 : 2;
      } else if (command.op == kCalibrate) {
        if (core.diagnostics(now).armed) report.command_result = 2;
        else { attitude.reset(); imu = {}; core.setImu(imu); }
      } else report.command_result = 2;
    }
    if (due(now, next_hello, 500000)) {
      queuePacket(core.requestSession(now));
    }
    if (due(now, next_telemetry, 5000)) {
      queuePacket(core.telemetry(now));
    }
    const auto duration = static_cast<std::uint32_t>(esp_timer_get_time()-now);
    if (duration > report.service_max_us) report.service_max_us = duration;
    if (due(now, next_report, 100000)) {
      report.stamp_us = now;
      report.state = core.diagnostics(now); report.imu = imu;
      report.motor = motors.diagnostics();
      (void)xQueueOverwrite(report_queue, &report);
    }
  }
}

struct TextBuffer {
  char bytes[2048]{}; std::size_t used = 0;
  void append(const char* line) {
    const auto n = std::strlen(line);
    if (used+n <= sizeof bytes) { std::memcpy(bytes+used, line, n); used += n; }
  }
  void consume(std::size_t n) { used -= n; std::memmove(bytes, bytes+n, used); }
};
TextBuffer usb_out, tcp_out;

void printLine(const char* line) {
  // Slow consoles drop whole diagnostic lines; partial writes retain framing.
  if (Serial) usb_out.append(line);
  if (console_client.connected()) tcp_out.append(line);
}

void flushConsoles() {
  if (Serial && usb_out.used) {
    const auto n = std::min(usb_out.used, static_cast<std::size_t>(std::max(0, Serial.availableForWrite())));
    if (n) usb_out.consume(Serial.write(reinterpret_cast<const std::uint8_t*>(usb_out.bytes), n));
  } else if (!Serial) usb_out.used = 0;
  if (console_client.connected() && tcp_out.used) {
    const int n = ::send(console_client.fd(), tcp_out.bytes, tcp_out.used, MSG_DONTWAIT);
    if (n > 0) tcp_out.consume(n);
    else if (n < 0 && errno != EWOULDBLOCK && errno != EAGAIN) console_client.stop();
  } else if (!console_client.connected()) tcp_out.used = 0;
}

bool otaSafe() {
  const auto now = static_cast<std::uint32_t>(esp_timer_get_time()/1000);
  return maintenance_requested.load() && maintenance_ready.load() &&
      now-maintenance_tick_ms.load() <= 250;
}

void pumpOta() {
  const bool ready = otaSafe() && WiFi.status() == WL_CONNECTED &&
      std::strlen(HCR_OTA_PASSWORD_HASH) == 32;
  if (!ready) {
    if (ota_started) { ArduinoOTA.end(); ota_started = false; }
    return;
  }
  if (!ota_started) {
    ArduinoOTA.setHostname("lf-mcu");
    ArduinoOTA.setPort(3232);
    ArduinoOTA.setMdnsEnabled(false); // console owns mDNS
    ArduinoOTA.setPasswordHash(HCR_OTA_PASSWORD_HASH);
    ArduinoOTA.setTimeout(3000);
    ArduinoOTA.onStart([]() {
      if (!otaSafe() || ArduinoOTA.getCommand() != U_FLASH) {
        Update.abort(); ota_error = 100; return;
      }
      ota_uploading = true; ota_error = -1; ota_percent = 0;
      printLine("OTA updating; motor commands locked until reboot\n");
      flushConsoles();
    });
    ArduinoOTA.onProgress([](unsigned done, unsigned total) {
      if (!otaSafe()) { Update.abort(); ota_error = 101; }
      ota_percent = total ? static_cast<unsigned>(100ULL*done/total) : 0;
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Update.abort(); ota_uploading = false;
      ota_error = static_cast<int>(error);
      printLine("OTA failed; maintenance stays locked, retry or OTA_REBOOT\n");
      flushConsoles();
    });
    ArduinoOTA.onEnd([]() {
      ota_percent = 100;
      printLine("OTA complete; rebooting DISARMED, home calibration required\n");
      flushConsoles();
    });
    ArduinoOTA.begin(); ota_started = true;
  }
  // May block during transfer. The core's sticky maintenance gate, not a
  // cached console status, prevents any queued ARM/target from actuating.
  ArduinoOTA.handle();
}

void status() {
  char line[512];
  const auto& s = latest_report.state;
  const auto now = static_cast<std::uint64_t>(esp_timer_get_time());
  const auto age = latest_report.imu.stamp_us ?
      (latest_report.stamp_us-latest_report.imu.stamp_us)/1000 : 999999;
  std::snprintf(line, sizeof line,
      "HCR uart=%d tx=%d rx=%d baud=921600 link=%d epoch=%u armed=%d enabled=%d motors=%d flags=0x%02x "
      "lf_mode=%u lf_fault=%u local_fault=%u targets=%u applied=%u rejected=%u crc=%u queue_err=%u "
      "imu_age_ms=%llu report_age_ms=%llu gravity=[%.4f %.4f %.4f] tick_max_us=%u missed=%u cmd=%u result=%u\n",
      uart_ready, HCR_UART_TX, HCR_UART_RX, s.connected, s.epoch, s.armed, s.enabled,
      s.motors_ready, s.telemetry_flags, s.remote_mode, s.remote_fault,
      static_cast<unsigned>(s.fault), s.rx_targets, s.applied_seq, s.rejected,
      parser_errors, queue_errors, static_cast<unsigned long long>(age),
      static_cast<unsigned long long>((now-latest_report.stamp_us)/1000),
      latest_report.imu.gravity[0], latest_report.imu.gravity[1], latest_report.imu.gravity[2],
      latest_report.service_max_us, latest_report.missed_ticks,
      latest_report.command_op, latest_report.command_result);
  printLine(line);
  const auto* running = esp_ota_get_running_partition();
  const auto* next = esp_ota_get_next_update_partition(nullptr);
  std::snprintf(line,sizeof line,
      "OTA supported=%d maintenance=%d off_confirmed=%d ready=%d port=3232 uploading=%d progress=%u error=%d running=%s next=%s build=%s_%s\n",
      std::strlen(HCR_OTA_PASSWORD_HASH)==32,maintenance_requested.load(),
      maintenance_ready.load(),ota_started && otaSafe(),ota_uploading,ota_percent,ota_error,
      running?running->label:"none",next?next->label:"none",__DATE__,__TIME__);
  printLine(line);
  std::snprintf(line,sizeof line,
      "IO rx_bytes=%u tx_bytes=%u rx0_level=%d imu_id=0x%02x imu_samples=%u imu_valid=%d "
      "accel=[%.5f %.5f %.5f] raw_gyro=[%.5f %.5f %.5f]\n",
      uart_rx_bytes,uart_tx_bytes,digitalRead(HCR_UART_RX),latest_report.imu_id,
      latest_report.imu_samples,latest_report.imu.valid,latest_report.imu.accel[0],
      latest_report.imu.accel[1],latest_report.imu.accel[2],latest_report.raw_gyro[0],
      latest_report.raw_gyro[1],latest_report.raw_gyro[2]);
  printLine(line);
  const auto& motor = latest_report.motor;
  for (unsigned side=0;side<2;++side) {
    const auto& m=motor.leg[side];
    std::snprintf(line,sizeof line,
      "MOTOR side=%s uart=%u gpio=%u configured=%d off=%d homed=%d prepared=%d fault=%u "
      "raw_rad=[%.5f %.5f %.5f] raw_rad_s=[%.5f %.5f %.5f] "
      "age_us=[%llu %llu %llu] kp_max=[%u %u] kd_max=[%u %u] bus_err=%u cycle_max_us=%u\n",
      side?"R":"L",side?1U:2U,side?17U:15U,m.configured,m.off,motor.homed,m.prepared,
      static_cast<unsigned>(motor.error),m.raw_q[0],m.raw_q[1],m.raw_q[2],
      m.raw_qd[0],m.raw_qd[1],m.raw_qd[2],
      static_cast<unsigned long long>(m.stamp[0]?now-m.stamp[0]:999999),
      static_cast<unsigned long long>(m.stamp[1]?now-m.stamp[1]:999999),
      static_cast<unsigned long long>(m.stamp[2]?now-m.stamp[2]:999999),
      m.kp_max[0],m.kp_max[1],m.kd_max[0],m.kd_max[1],m.errors,m.cycle_max_us);
    printLine(line);
  }
}

void handleLine(const char* line) {
  Command c{};
  if (!std::strcmp(line, "STATUS") || !std::strcmp(line, "GET")) { status(); return; }
  if (!std::strcmp(line, "HELP")) {
    printLine("Commands: STATUS, ESTOP, CLEAR_FAULT, CAL_IMU, MOTOR_HOME, MOTOR_ZERO hipL hipR kneeL kneeR (rad). ARM/DISARM/RESET/VELOCITY use the LF local command socket.\n");
    printLine("OTA_BEGIN: lock motion until reboot, wait OFF then accept OTA. OTA_REBOOT: reboot only after confirmed OFF. STATUS: OTA readiness.\n");
    return;
  }
  if (!std::strcmp(line,"OTA_BEGIN")) {
    if (std::strlen(HCR_OTA_PASSWORD_HASH)!=32 || !esp_ota_get_next_update_partition(nullptr)) {
      printLine("ERR OTA unavailable: provision credentials and OTA partitions via USB\n"); return;
    }
    maintenance_requested.store(true);
    printLine("OTA maintenance requested; waiting for confirmed OFF; no motion until reboot\n");
    return;
  }
  if (!std::strcmp(line,"OTA_REBOOT")) {
    if (!otaSafe() || ota_uploading) { printLine("ERR reboot requires maintenance and confirmed OFF\n"); return; }
    printLine("OTA rebooting DISARMED\n"); flushConsoles(); ESP.restart(); return;
  }
  if (maintenance_requested.load() && std::strcmp(line,"ESTOP")) {
    printLine("ERR maintenance locked until reboot\n"); return;
  }
  if (!std::strcmp(line, "ESTOP")) { c.op = kEstop; estop_requested.store(true); }
  else if (!std::strcmp(line,"CLEAR_FAULT")) c.op=kClearFault;
  else if (!std::strcmp(line, "CAL_IMU")) c.op = kCalibrate;
  else if (!std::strcmp(line, "MOTOR_HOME")) {
    c.op = kMotorHome; c.pose = {2.35706715F,2.35706715F,0,0};
  } else if (!std::strncmp(line,"MOTOR_ZERO ",11)) {
    char extra;
    if (std::sscanf(line,"MOTOR_ZERO %f %f %f %f %c",&c.pose[0],&c.pose[1],
                    &c.pose[2],&c.pose[3],&extra)!=4) {
      printLine("ERR MOTOR_ZERO requires four measured model angles in rad\n"); return;
    }
    c.op=kMotorHome;
  }
  else { printLine("ERR unknown command; HELP\n"); return; }
  if (xQueueSend(command_queue, &c, 0) != pdTRUE) printLine("ERR command queue full\n");
  else printLine("QUEUED; HCR cmd/result reports accepted=1 or refused=2\n");
}

struct LineReader {
  char line[96]{}; std::size_t size = 0; bool overflow = false;
  void read(Stream& stream) {
    for (unsigned n = 0; n < 128 && stream.available(); ++n) {
      const int c = stream.read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (overflow) printLine("ERR line too long\n");
        else if (size) { line[size] = 0; handleLine(line); }
        size = 0; overflow = false;
      } else if (c >= 32 && c < 127) {
        if (size+1 < sizeof line) line[size++] = static_cast<char>(c);
        else overflow = true;
      } else overflow = true;
    }
  }
};
LineReader usb_lines, tcp_lines;

bool startUart() {
  uart_config_t cfg{};
  cfg.baud_rate = 921600; cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE; cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE; cfg.source_clk = UART_SCLK_APB;
  if (uart_param_config(kLfUart, &cfg) != ESP_OK ||
      uart_set_pin(kLfUart, HCR_UART_TX, HCR_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
    return false;
  // No TX ring: uart_tx_chars() only accepts bytes fitting the hardware FIFO.
  // Its nonblocking partial writes are completed by loop().
  return uart_driver_install(kLfUart, 4096, 0, 0, nullptr, 0) == ESP_OK;
}

void pumpUart() {
  if (!uart_ready) return;
  static WirePacket pending{};
  static std::size_t offset = 0;
  // Finish TX first, enqueue sent confirmation BEFORE reading its LF reply.
  for (unsigned n = 0; n < 4; ++n) {
    if (!pending.size && xQueueReceive(tx_queue, &pending, 0) != pdTRUE) break;
    const auto now = static_cast<std::uint64_t>(esp_timer_get_time());
    if (now-hcr_u64(pending.bytes.data()+12) > 10000 && offset == 0) {
      pending = {}; transport_fault.store(true); ++queue_errors; continue;
    }
    const int count = uart_tx_chars(kLfUart, reinterpret_cast<const char*>(pending.bytes.data()+offset),
                                     pending.size-offset);
    if (count <= 0) break;
    offset += count; uart_tx_bytes += count;
    if (offset < pending.size) break;
    if (pending.bytes[3] == HCR_TELEMETRY) {
      RxEvent event{}; event.sent = true;
      event.frame.seq = hcr_u32(pending.bytes.data()+8);
      event.frame.stamp_us = hcr_u64(pending.bytes.data()+12);
      std::memcpy(event.frame.payload, pending.bytes.data()+20, 4);
      if (xQueueSend(rx_queue, &event, 0) != pdTRUE) { ++queue_errors; transport_fault.store(true); }
    }
    pending = {}; offset = 0;
  }
  std::uint8_t bytes[512];
  const int count = uart_read_bytes(kLfUart, bytes, sizeof bytes, 0);
  if (count > 0) uart_rx_bytes += count;
  for (int i = 0; i < count; ++i) {
    RxEvent event{};
    if (hcr_feed(&parser, bytes[i], &event.frame)) {
      event.received_us = esp_timer_get_time();
      if (xQueueSend(rx_queue, &event, 0) != pdTRUE) { ++queue_errors; transport_fault.store(true); }
    }
  }
  parser_errors = parser.errors;
}
}  // namespace

void setup() {
  // Serial is USB native CDC in this environment. Never print on Serial0.
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  rx_queue = xQueueCreate(32, sizeof(RxEvent));
  tx_queue = xQueueCreate(16, sizeof(WirePacket));
  command_queue = xQueueCreate(8, sizeof(Command));
  report_queue = xQueueCreate(1, sizeof(Report));
  if (!rx_queue || !tx_queue || !command_queue || !report_queue) {
    printLine("FATAL queue allocation\n"); return;
  }
  motor_system.begin();
  uart_ready = startUart();
  if (!uart_ready) transport_fault.store(true);
  esp_timer_create_args_t timer_args{};
  timer_args.callback = timerCallback; timer_args.name = "hcr-service";
  if (esp_timer_create(&timer_args, &service_timer) != ESP_OK ||
      xTaskCreatePinnedToCore(service, "hcr-service", 8192, nullptr, 20, &service_task, 1) != pdPASS) {
    service_task = nullptr; printLine("FATAL service task/timer\n"); return;
  }
  WiFi.persistent(false); WiFi.mode(WIFI_STA); WiFi.setSleep(false);
  WiFi.setAutoReconnect(true); WiFi.setHostname("lf-mcu");
#if defined(TAP_WIFI_STA_SSID) && defined(TAP_WIFI_STA_PASS)
  const char* ssid = TAP_WIFI_STA_SSID;
  const char* password = TAP_WIFI_STA_PASS;
#if defined(TAP_WIFI_ALT_SSID) || defined(TAP_WIFI_ALT2_SSID)
  const int networks = WiFi.scanNetworks(false, true, false, 400);
#if defined(TAP_WIFI_ALT_SSID) && defined(TAP_WIFI_ALT_PASS)
  for (int i = 0; i < networks; ++i)
    if (WiFi.SSID(i) == TAP_WIFI_ALT_SSID) { ssid = TAP_WIFI_ALT_SSID; password = TAP_WIFI_ALT_PASS; break; }
#endif
#if defined(TAP_WIFI_ALT2_SSID) && defined(TAP_WIFI_ALT2_PASS)
  for (int i = 0; i < networks; ++i)
    if (WiFi.SSID(i) == TAP_WIFI_ALT2_SSID) { ssid = TAP_WIFI_ALT2_SSID; password = TAP_WIFI_ALT2_PASS; break; }
#endif
  WiFi.scanDelete();
#endif
  WiFi.begin(ssid, password);
#endif
  console_server.begin(); console_server.setNoDelay(true);
  printLine("HCR1 UART0 TX43/RX44 921600 8N1; USB native/TCP8023 console; BL4012e backend; manual MOTOR_HOME required\n");
}

void loop() {
  if (!service_task) { delay(10); return; }
  pumpUart();
  (void)xQueueReceive(report_queue, &latest_report, 0);
  usb_lines.read(Serial);
  if (console_server.hasClient()) {
    console_client.stop(); console_client = console_server.available();
    console_client.setNoDelay(true); console_client.setTimeout(0);
    tcp_lines = {}; tcp_out.used = 0; printLine("HCR1 lower layer; HELP for commands\n");
  }
  if (console_client.connected()) tcp_lines.read(console_client);
  static std::uint32_t last_status = 0, last_beacon = 0;
  static bool mdns_started = false;
  const auto now = millis();
  if (WiFi.status() == WL_CONNECTED && !mdns_started) {
    mdns_started = MDNS.begin("lf-mcu");
    if (mdns_started) MDNS.addService("hcr-console", "tcp", 8023);
  }
  if (now-last_status >= 1000) { status(); last_status = now; }
  if (WiFi.status() == WL_CONNECTED && now-last_beacon >= 2000) {
    char msg[128];
    std::snprintf(msg, sizeof msg, "HCR1 MCU ip=%s console=8023 uart=UART0 motors=BL4012e",
                  WiFi.localIP().toString().c_str());
    const IPAddress broadcast = ~WiFi.subnetMask() | WiFi.localIP();
    if (discovery.beginPacket(broadcast, 8024)) {
      discovery.write(reinterpret_cast<const std::uint8_t*>(msg), std::strlen(msg)); discovery.endPacket();
    }
    last_beacon = now;
  }
  flushConsoles();
  pumpOta();
  delay(1);
}
#endif
