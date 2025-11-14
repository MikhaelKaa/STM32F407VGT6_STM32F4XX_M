
#include <stdio.h>

#include "stm32f407xx.h"
#include "green_led.h"
#include "dwt_delay.h"
#include "ucmd.h"

#include "dev_list.h"
#include "uart_ping.h"
#include "rng_gen.h"


int main(void)
{
    uint32_t led_cnt = 0;
    
    // init board led
    green_led_init();

    // init dwt time module
    dwt_delay_init();

    // uart1 - printf, console
    dev_uart1.open();
    setvbuf(stdin, NULL, _IONBF, 0);  // Отключаем буферизацию stdin
    setvbuf(stdout, NULL, _IONBF, 0); // Отключаем буферизацию stdout
    
    dev_uart2.open();

    printf("\r\n");

    
    dev_rng.open();
    uint32_t rng_buf[16] = {0};
    dev_rng.read(rng_buf, sizeof(rng_buf));
    for(unsigned int i = 0; i < sizeof(rng_buf)/4; i++) {
        printf("0x%08lx\r\n", rng_buf[i]);
    }

    dev_uart_ping = (interface_t*)&dev_uart2;
    dev_rng_gen   = (interface_t*)&dev_rng;

    ucmd_default_init();

    while (1)
    {
        // blink led demo
        green_led_set((led_cnt++ & 256) ? (led_off) : (led_on));

        ucmd_default_proc();

        dwt_delay_ms(1);
    }
}
