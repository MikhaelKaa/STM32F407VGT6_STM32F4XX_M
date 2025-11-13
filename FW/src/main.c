
#include <stdio.h>

#include "stm32f407xx.h"
#include "green_led.h"
#include "dwt_delay.h"
#include "ucmd.h"

#include "dev_list.h"

int main(void)
{

    // init board led
    green_led_init();

    // init dwt time module
    dwt_delay_init();

    // uart1 - printf, console
    uart1_dev.open();
    setvbuf(stdin, NULL, _IONBF, 0);  // Отключаем буферизацию stdin
    setvbuf(stdout, NULL, _IONBF, 0); // Отключаем буферизацию stdout
    
    uart2_dev.open();

    printf("\r\n");

    ucmd_default_init();

    uint32_t led_cnt = 0;

    while (1)
    {
        // blink led demo
        green_led_set((led_cnt++ & 256) ? (led_off) : (led_on));

        ucmd_default_proc();

        dwt_delay_ms(1);
    }
}
