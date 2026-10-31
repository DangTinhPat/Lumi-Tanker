#ifdef TAP_HYBRID_GATEWAY
#include "tap_hw/EspBlMotor.hpp"
#include <esp_timer.h>
#include <esp_rom_gpio.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_sig_map.h>
#include <soc/uart_periph.h>
#include <cstring>

namespace tap_hw::hybrid {
std::uint64_t EspMotorBus::now() const noexcept { return esp_timer_get_time(); }
void EspMotorBus::release() noexcept {
  esp_rom_gpio_connect_out_signal(pin_,SIG_GPIO_OUT_IDX,false,false);
  gpio_ll_output_disable(&GPIO,pin_);
}
bool EspMotorBus::begin(uart_port_t uart, gpio_num_t data, gpio_num_t unused_rx) noexcept {
  uart_=uart; pin_=data; tx_signal_=UART_PERIPH_SIGNAL(uart,SOC_UART_TX_PIN_IDX);
  gpio_reset_pin(unused_rx);
  gpio_set_direction(unused_rx,GPIO_MODE_INPUT);
  gpio_set_pull_mode(unused_rx,GPIO_FLOATING);
  uart_config_t cfg{}; cfg.baud_rate=1000000; cfg.data_bits=UART_DATA_8_BITS;
  cfg.parity=UART_PARITY_DISABLE; cfg.stop_bits=UART_STOP_BITS_1;
  cfg.flow_ctrl=UART_HW_FLOWCTRL_DISABLE; cfg.source_clk=UART_SCLK_APB;
  if (uart_driver_install(uart_,512,0,0,nullptr,0)!=ESP_OK) return false;
  if (uart_param_config(uart_,&cfg)!=ESP_OK ||
      uart_set_pin(uart_,pin_,pin_,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE)!=ESP_OK ||
      gpio_set_direction(pin_,GPIO_MODE_INPUT_OUTPUT)!=ESP_OK ||
      gpio_set_level(pin_,1)!=ESP_OK || gpio_set_pull_mode(pin_,GPIO_PULLUP_ONLY)!=ESP_OK ||
      uart_set_rx_timeout(uart_,2)!=ESP_OK) {
    release(); uart_driver_delete(uart_); return false;
  }
  release(); initialized_=true; return true;
}
bool EspMotorBus::readExact(std::uint8_t* out, std::size_t n, std::uint64_t deadline) noexcept {
  std::size_t have=0;
  while (have<n && now()<deadline) {
    const int got=uart_read_bytes(uart_,out+have,n-have,1);
    if (got<0) return false;
    have+=got;
  }
  return have==n && now()<=deadline;
}
bool EspMotorBus::request(BusFormat fmt, std::uint8_t id, std::uint8_t inst,
                           const std::uint8_t* p, std::size_t n,
                           std::size_t reads, BusReply& reply) noexcept {
  error_=BusError::None;
  if (!initialized_ || reads>reply.data.size()) { error_=BusError::Unavailable; return false; }
  std::array<std::uint8_t,48> tx{}, echo{}, rx{};
  const auto length=motorPacket(id,inst,p,n,tx);
  if (!length) { error_=BusError::Packet; return false; }
  // A single overall budget prevents partial reads from repeatedly renewing
  // the timeout. At 1 Mbaud even the largest supported TX takes < 0.5 ms.
  const auto deadline=now()+(fmt==BusFormat::Wheel?8000:4000);
  uart_flush_input(uart_);
  esp_rom_gpio_connect_out_signal(pin_,tx_signal_,false,false);
  const int sent=uart_write_bytes(uart_,tx.data(),length);
  const auto done=uart_wait_tx_done(uart_,pdMS_TO_TICKS(2));
  release(); // always release the one-wire pad, including failure paths
  if (sent!=static_cast<int>(length) || done!=ESP_OK) { error_=BusError::Tx; return false; }
  if (!readExact(echo.data(),length,deadline)) { error_=BusError::EchoTimeout; return false; }
  if (std::memcmp(echo.data(),tx.data(),length)) { error_=BusError::EchoMismatch; return false; }
  const auto count=(fmt==BusFormat::Wheel?9U:11U)+reads;
  if (!readExact(rx.data(),count,deadline)) { error_=BusError::ReplyTimeout; return false; }
  if (!motorReply(fmt,id,inst,n?p[0]:0,tx[3],rx.data(),count,reads,reply)) {
    error_=BusError::ReplyInvalid; return false;
  }
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
} // namespace tap_hw::hybrid
#endif
