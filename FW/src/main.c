
#include "stm32f407xx.h"
#include "green_led.h"
#include "dwt_delay.h"

int main(void) {

  // init board led
  green_led_init();

  // init dwt time module
  dwt_delay_init();
  
  while (1) {

    // blink led demo
    green_led_set(led_off);
    dwt_delay_ms(500);
    
    green_led_set(led_on);
    dwt_delay_ms(500);
  }
}
