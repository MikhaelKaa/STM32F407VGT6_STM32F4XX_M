
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
    dev_uart1_get()->ioctl(UART_INIT, 0);
    setvbuf(stdin, NULL, _IONBF, 0);  // Отключаем буферизацию stdin
    setvbuf(stdout, NULL, _IONBF, 0); // Отключаем буферизацию stdout
    
    // dev_uart2.open();

    printf("\r\n");

    // Get RNG instance
    interface_t* dev_rng = (interface_t*)dev_rng_get();
    // Init RNG
    dev_rng->ioctl(RNG_INIT, NULL);
    // Set rng pointer to app
    app_dev_rng_set((interface_t*)dev_rng_get());

    dev_uart_ping = (interface_t*)dev_uart2_get();
    dev_uart_ping->ioctl(UART_INIT, 0);

    ucmd_default_init();

    while (1)
    {
        // blink led demo
        green_led_set((led_cnt++ & 256) ? (led_off) : (led_on));

        ucmd_default_proc();

        dwt_delay_ms(1);
    }
}
