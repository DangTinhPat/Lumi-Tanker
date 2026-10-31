#ifdef TAP_HYBRID_GATEWAY
#include "tap_hw/EspBlMotor.hpp"
#include <esp_timer.h>
#include <esp_rom_gpio.h>
#include <hal/gpio_ll.h>
#include <hal/uart_ll.h>
#include <soc/gpio_sig_map.h>
#include <soc/uart_periph.h>
#include <cstring>

namespace tap_hw::hybrid {
std::uint64_t EspMotorBus::now() const noexcept { return esp_timer_get_time(); }
void EspMotorBus::release() noexcept {
  // On this ESP32-S3 SDK the LL call already disconnects the matrix AFTER
  // clearing output enable. Calling the ROM routing helper first briefly
  // drove idle-high again and could swallow the first start bit of a reply.
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
  if (uart_driver_install(uart_,512,0,32,&events_,0)!=ESP_OK) return false;
  if (uart_param_config(uart_,&cfg)!=ESP_OK ||
      uart_set_pin(uart_,pin_,pin_,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE)!=ESP_OK ||
      gpio_set_direction(pin_,GPIO_MODE_INPUT_OUTPUT)!=ESP_OK ||
      gpio_set_level(pin_,1)!=ESP_OK || gpio_set_pull_mode(pin_,GPIO_PULLUP_ONLY)!=ESP_OK ||
      uart_set_rx_timeout(uart_,2)!=ESP_OK) {
    release(); uart_driver_delete(uart_); return false;
  }
  release(); initialized_=true; return true;
}
bool EspMotorBus::readExact(std::uint8_t* out, std::size_t n, std::uint64_t deadline,
                            std::uint8_t& received) noexcept {
  std::size_t have=0;
  while (have<n && now()<deadline) {
    const int got=uart_read_bytes(uart_,out+have,n-have,1);
    if (got<0) return false;
    have+=got;
    received=static_cast<std::uint8_t>(have);
  }
  return have==n && now()<=deadline;
}
bool EspMotorBus::request(BusFormat fmt, std::uint8_t id, std::uint8_t inst,
                           const std::uint8_t* p, std::size_t n,
                           std::size_t reads, BusReply& reply) noexcept {
  error_=BusError::None; trace_={};
  if (!initialized_ || reads>reply.data.size()) { error_=BusError::Unavailable; return false; }
  std::array<std::uint8_t,48> tx{}, echo{}, rx{};
  const auto length=motorPacket(id,inst,p,n,tx);
  if (!length) { error_=BusError::Packet; return false; }
  // A single overall budget prevents partial reads from repeatedly renewing
  // the timeout. At 1 Mbaud even the largest supported TX takes < 0.5 ms.
  const auto started=now();
  const auto deadline=started+(fmt==BusFormat::Wheel?8000:4000);
  trace_.tx_count=static_cast<std::uint8_t>(length);
  std::memcpy(trace_.tx.data(),tx.data(),length);
  const auto finish=[&](BusError error) {
    auto* hw=UART_LL_GET_HW(uart_);
    trace_.irq_raw=hw->int_raw.val; trace_.irq_enable=hw->int_ena.val;
    trace_.uart_fsm=hw->fsm_status.val;
    trace_.hw_status=hw->status.val; trace_.pad_finish=gpio_get_level(pin_);
    std::size_t buffered=0; uart_get_buffered_data_len(uart_,&buffered);
    trace_.ring_bytes=buffered;
    uart_event_t event{};
    while (xQueueReceive(events_,&event,0)==pdTRUE)
      if (event.type<32) trace_.events|=1U<<event.type;
    std::memcpy(trace_.echo.data(),echo.data(),trace_.echo_count);
    error_=error; trace_.total_us=static_cast<std::uint32_t>(now()-started);
    std::memcpy(trace_.rx.data(),rx.data(),trace_.rx_count);
    return error==BusError::None;
  };
  uart_event_t prior{};
  while (xQueueReceive(events_,&prior,0)==pdTRUE) {}
  uart_flush_input(uart_);
  trace_.pad_before=gpio_get_level(pin_);
  // One owner, no TX ring, whole packet fits the hardware FIFO. The BL wheel
  // replies immediately: waking a task from uart_wait_tx_done() held the pad
  // for 130-195 us on an 80 us packet and corrupted the reply's first bytes.
  // Release at hardware idle, without a scheduler gap. No RTOS calls inside
  // this bounded critical section; RX bytes remain in the hardware FIFO.
  auto* hw=UART_LL_GET_HW(uart_);
  portENTER_CRITICAL(&tx_mux_);
  const auto tx_deadline=now()+length*10+50; // 1 Mbaud/8N1 + bounded overhead
  esp_rom_gpio_connect_out_signal(pin_,tx_signal_,false,false);
  // Manual one-wire direction must use the GPIO enable latch, not the
  // peripheral OEN signal selected by the ROM routing helper (oen_sel=0).
  // Keep RX input enabled; release() clears this latch before disconnecting TX.
  GPIO.func_out_sel_cfg[pin_].oen_sel=1;
  gpio_ll_output_enable(&GPIO,pin_);
  trace_.gpio_enable=GPIO.enable;
  trace_.gpio_out_route=GPIO.func_out_sel_cfg[pin_].val;
  trace_.gpio_in_route=GPIO.func_in_sel_cfg[UART_PERIPH_SIGNAL(uart_,SOC_UART_RX_PIN_IDX)].val;
  trace_.iomux=REG_READ(GPIO_PIN_MUX_REG[pin_]);
  uart_ll_write_txfifo(hw,tx.data(),length);
  while (!uart_ll_is_tx_idle(hw) && now()<tx_deadline) {
    if (gpio_get_level(pin_)) ++trace_.high_samples;
    else ++trace_.low_samples;
  }
  const bool done=uart_ll_is_tx_idle(hw);
  release(); // always release the one-wire pad, including failure paths
  trace_.fifo_release=hw->status.rxfifo_cnt;
  trace_.pad_release=gpio_get_level(pin_);
  portEXIT_CRITICAL(&tx_mux_);
  trace_.tx_us=static_cast<std::uint32_t>(now()-started);
  if (!done) return finish(BusError::Tx);
  const bool echo_ok=readExact(echo.data(),length,deadline,trace_.echo_count);
  trace_.echo_us=static_cast<std::uint32_t>(now()-started);
  if (!echo_ok) return finish(BusError::EchoTimeout);
  if (std::memcmp(echo.data(),tx.data(),length)) return finish(BusError::EchoMismatch);
  const auto count=(fmt==BusFormat::Wheel?9U:11U)+reads;
  trace_.expected=static_cast<std::uint8_t>(count);
  if (!readExact(rx.data(),count,deadline,trace_.rx_count)) return finish(BusError::ReplyTimeout);
  if (!motorReply(fmt,id,inst,n?p[0]:0,tx[3],rx.data(),count,reads,reply)) {
    return finish(BusError::ReplyInvalid);
  }
  return finish(BusError::None);
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
