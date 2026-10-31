#ifdef TAP_HYBRID_GATEWAY
#include "gateway/EspBlMotor.hpp"
#include <esp_timer.h>
#include <algorithm>
#include <cstring>

namespace gateway {
std::uint64_t EspMotorBus::now() const noexcept { return esp_timer_get_time(); }
bool EspMotorBus::begin(uart_port_t uart, gpio_num_t data, gpio_num_t unused_rx) noexcept {
  // The PCB's spare RX pad is tied to DATA. It must never drive that net.
  if (gpio_reset_pin(unused_rx)!=ESP_OK ||
      gpio_set_direction(unused_rx,GPIO_MODE_INPUT)!=ESP_OK ||
      gpio_set_pull_mode(unused_rx,GPIO_FLOATING)!=ESP_OK) return false;
  initialized_=joint_bus_init(&bus_,uart,data,1000000)==ESP_OK;
  return initialized_;
}
BusStats EspMotorBus::stats() const noexcept {
  return {bus_.err_timeout,bus_.err_echo,bus_.err_frame,bus_.err_partial,
          bus_.err_late,bus_.sync_recovered,bus_.err_quiet,bus_.aborted};
}
bool EspMotorBus::request(BusFormat fmt, std::uint8_t id, std::uint8_t inst,
                          const std::uint8_t* p, std::size_t n,
                          std::size_t reads, BusReply& reply) noexcept {
  error_=BusError::None; trace_={};
  if (!initialized_) { error_=BusError::Unavailable; return false; }
  // HCR requires an individual reply; broadcasts must never produce an ACK.
  const bool ping=inst==1 && n==0 && reads==0;
  const bool read=inst==2 && n==2 && p && reads>0 &&
                  reads<=reply.data.size() && p[1]==reads;
  const bool write=inst==3 && n>=1 && n<=42 && p && reads==0;
  if (id==0 || id>=0xfe || (fmt!=BusFormat::Joint && fmt!=BusFormat::Wheel) ||
      !(ping || read || write)) { error_=BusError::Packet; return false; }
  const auto format=fmt==BusFormat::Wheel?JB_FMT_WHEEL:JB_FMT_JOINT;
  bus_.deadline_us=now()+20000; // Bound even OFF/configuration transactions.
  if (permit_.deadline_us && permit_.deadline_us<static_cast<std::uint64_t>(bus_.deadline_us))
    bus_.deadline_us=permit_.deadline_us;
  bus_.permit=permit_.valid; bus_.permit_context=permit_.context;
  joint_reply_t result{};
  BusReply decoded{};
  esp_err_t rc;
  if (ping) rc=bus_ping(&bus_,format,id,&result);
  else if (read) rc=bus_read(&bus_,format,id,p[0],reads,decoded.data.data(),&result);
  else rc=bus_write(&bus_,format,id,p[0],p+1,n-1,&result);
  switch (bus_.last_error) {
    case JB_ERROR_NONE: error_=rc==ESP_OK?BusError::None:BusError::Packet; break;
    case JB_ERROR_PACKET: error_=BusError::Packet; break;
    case JB_ERROR_TX: error_=BusError::Tx; break;
    case JB_ERROR_ECHO_TIMEOUT: error_=BusError::EchoTimeout; break;
    case JB_ERROR_ECHO_MISMATCH: error_=BusError::EchoMismatch; break;
    case JB_ERROR_REPLY_TIMEOUT: error_=BusError::ReplyTimeout; break;
    case JB_ERROR_REPLY_INVALID: error_=BusError::ReplyInvalid; break;
    case JB_ERROR_DEADLINE: error_=BusError::Deadline; break;
    case JB_ERROR_CANCELLED: error_=BusError::Cancelled; break;
    case JB_ERROR_QUIET: error_=BusError::Quiet; break;
  }
  const auto& t=bus_.trace;
  trace_.tx_us=t.tx_us; trace_.echo_us=t.echo_us; trace_.total_us=t.total_us;
  trace_.tx_count=t.tx_count; trace_.echo_count=t.echo_count;
  trace_.sync_restored=t.sync_restored;
  trace_.rx_count=t.rx_count; trace_.expected=t.expected;
  std::copy_n(t.tx,trace_.tx.size(),trace_.tx.begin());
  std::copy_n(t.echo,trace_.echo.size(),trace_.echo.begin());
  std::copy_n(t.rx,trace_.rx.size(),trace_.rx.begin());
  if (rc!=ESP_OK) return false;
  decoded.status=result.status; decoded.position=result.pos;
  reply=decoded;
  return true;
}
void EspBlMotor::workerTask(void* arg) {
  auto& worker=*static_cast<BlMotorWorker*>(arg);
  for (;;) { worker.step(); vTaskDelay(1); }
}
bool EspBlMotor::begin() noexcept {
  static_assert(configTICK_RATE_HZ>=1000,"Motor timeout budget requires <=1 ms RTOS tick");
  // User correction: UART2 is LEFT; UART1 is RIGHT. Spare RX pads tied to
  // DATA are inputs only. UART0 TX43/RX44 remains exclusively the LF link.
  const bool left_ok=left_bus.begin(UART_NUM_2,GPIO_NUM_15,GPIO_NUM_16);
  const bool right_ok=right_bus.begin(UART_NUM_1,GPIO_NUM_17,GPIO_NUM_18);
  // A healthy bus must still get its OFF worker if the other bus cannot start.
  const bool left_started=left_ok &&
    xTaskCreatePinnedToCore(workerTask,"motor-left",4096,&left,18,&left_task,1)==pdPASS;
  const bool right_started=right_ok &&
    xTaskCreatePinnedToCore(workerTask,"motor-right",4096,&right,18,&right_task,1)==pdPASS;
  if (!left_started || !right_started) {
    backend.trip(MotorError::Startup); return false;
  }
  return true;
}
} // namespace gateway
#endif
