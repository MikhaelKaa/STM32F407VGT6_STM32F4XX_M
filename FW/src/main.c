
#include "stm32f407xx.h"
#include "green_led.h"

// Простая функция задержки
void delay(void) {
  for (volatile uint32_t i = 0; i < 500000; i++);
}

int main(void) {

  // init board led/
  green_led_init();

  while (1) {

    // blink led demo
    green_led_set(led_off);
    delay();
    
    green_led_set(led_on);
    delay();
  }
}

// void SystemInit (void)
// {
//   //SystemCoreClock = SYSTEM_CLOCK;
// }