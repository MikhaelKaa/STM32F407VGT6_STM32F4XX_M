
#include <stdio.h>

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
  
  printf("Its work\r\n");
  
  uint32_t  led_cnt     = 0;
  char ch = 0;
  while (1) {

    // blink led demo
    green_led_set((led_cnt++&256)?(led_off):(led_on));
    
    if(scanf("%c", &ch) == 1) {
      printf("%c", ch);
      fflush(stdout);
    }

    dwt_delay_ms(1);
  }
}



