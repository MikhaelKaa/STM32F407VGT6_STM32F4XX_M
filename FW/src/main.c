
#include "stm32f407xx.h"
#include "green_led.h"
#include "dwt_delay.h"
#include "uart1.h"

int main(void) {

  // init board led
  green_led_init();

  // init dwt time module
  dwt_delay_init();

  // uart1
  uart_open();
  
  uart_write("Its work!!!\r\n", sizeof("Its work!!!\r\n")-1);
  
  uint8_t   rx_buf[32]  = {0};
  uint8_t   rx_cnt      = 0;
  uint32_t  led_cnt     = 0;
  
  while (1) {

    // blink led demo
    green_led_set((led_cnt++&256)?(led_off):(led_on));

    rx_cnt = uart_read(rx_buf, 32);
    if(rx_cnt) {
      uart_write(rx_buf, rx_cnt);
      rx_cnt = 0;
    }
    
    dwt_delay_ms(1);
  }
}
