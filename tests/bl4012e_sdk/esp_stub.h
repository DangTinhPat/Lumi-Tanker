
#ifndef ESP_TEST_STUB_H
#define ESP_TEST_STUB_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
typedef int esp_err_t; typedef int uart_port_t; typedef int gpio_num_t;
typedef unsigned TickType_t;
typedef struct {int baud_rate,data_bits,parity,stop_bits,flow_ctrl,source_clk;} uart_config_t;
#define ESP_OK 0
#define ESP_ERR_TIMEOUT 1
#define ESP_ERR_INVALID_RESPONSE 2
#define ESP_ERR_INVALID_SIZE 3
#define ESP_ERR_INVALID_ARG 4
#define UART_DATA_8_BITS 8
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1 1
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_SCLK_DEFAULT 0
#define UART_PIN_NO_CHANGE -1
#define GPIO_MODE_INPUT_OUTPUT 3
#define GPIO_PULLUP_ONLY 1
#define SIG_GPIO_OUT_IDX 0
#define SOC_UART_TX_PIN_IDX 0
#define UART_PERIPH_SIGNAL(u,s) ((unsigned)(u)+(unsigned)(s))
#define pdMS_TO_TICKS(x) (x)
#define portTICK_PERIOD_MS 1
#define ESP_RETURN_ON_ERROR(expr,tag,...) do {(void)(tag);int e=(expr);if(e)return e;}while(0)
#ifdef __cplusplus
extern "C" {
#endif
extern int GPIO;
extern int driven;
void event(const char*, int);
int64_t esp_timer_get_time(void);
#define ESP_IDF_VERSION_MAJOR 5
#define UART_NUM_1 1
#define UART_NUM_2 2
#define GPIO_NUM_15 15
#define GPIO_NUM_16 16
#define GPIO_NUM_17 17
#define GPIO_NUM_18 18
#define GPIO_MODE_INPUT 1
#define GPIO_FLOATING 0
#define configTICK_RATE_HZ 1000
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
#define pdPASS 1
typedef int portMUX_TYPE;
#define taskENTER_CRITICAL(p) portENTER_CRITICAL(p)
#define taskEXIT_CRITICAL(p) portEXIT_CRITICAL(p)
typedef struct {int unused;} uart_dev_t;
extern uart_dev_t UART_HW;
#define UART_LL_GET_HW(num) ((void)(num), &UART_HW)
bool uart_ll_is_tx_idle(uart_dev_t*);
uint32_t uart_ll_get_txfifo_len(uart_dev_t*);
void uart_ll_write_txfifo(uart_dev_t*,const uint8_t*,uint32_t);
void uart_ll_txfifo_rst(uart_dev_t*);
void esp_rom_delay_us(uint32_t);
typedef void* TaskHandle_t;
static inline int xTaskCreatePinnedToCore(void (*f)(void*),const char*n,int s,void*a,int p,TaskHandle_t*t,int c){(void)f;(void)n;(void)s;(void)a;(void)p;(void)t;(void)c;return pdPASS;}
static inline void vTaskDelay(int t){(void)t;}
static inline int gpio_reset_pin(int p){event("reset",p);return 0;}
static inline void esp_rom_gpio_connect_out_signal(int a,unsigned b,bool c,bool d){(void)c;(void)d;event("route_pin",a);event("route_signal",b);driven=b!=SIG_GPIO_OUT_IDX;}
static inline void gpio_ll_output_disable(int *a,int b){(void)a;event("release",b);driven=0;}
static inline int uart_driver_install(int a,int b,int c,int d,void *e,int f){(void)e;event("install_uart",a);event("rx_ring",b);event("tx_ring",c);event("queue",d);event("irq",f);return 0;}
static inline int uart_param_config(int a,const uart_config_t *b){event("config_uart",a);event("baud",b->baud_rate);event("bits",b->data_bits);event("parity",b->parity);event("stop",b->stop_bits);event("flow",b->flow_ctrl);event("clock",b->source_clk);return 0;}
static inline int uart_set_pin(int a,int b,int c,int d,int e){event("set_uart",a);event("tx_pin",b);event("rx_pin",c);event("rts",d);event("cts",e);return 0;}
static inline int gpio_set_direction(int a,int b){event("gpio_set_direction_pin",a);event("gpio_set_direction",b);return 0;}
static inline int gpio_set_level(int a,int b){event("gpio_set_level_pin",a);event("gpio_set_level",b);return 0;}
static inline int gpio_set_pull_mode(int a,int b){event("gpio_set_pull_mode_pin",a);event("gpio_set_pull_mode",b);return 0;}
static inline int uart_set_rx_timeout(int a,int b){event("uart_set_rx_timeout_pin",a);event("uart_set_rx_timeout",b);return 0;}
int uart_flush_input(uart_port_t);
int uart_write_bytes(uart_port_t,const void *,size_t);
int uart_wait_tx_done(uart_port_t,TickType_t);
int uart_read_bytes(uart_port_t,void *,size_t,TickType_t);
#ifdef __cplusplus
}
#endif
#endif
