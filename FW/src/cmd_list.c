
// #include <stdio.h>

#include "stm32f407xx.h"
// #include "green_led.h"
// #include "dwt_delay.h"
// #include "uart1.h"

#include "ucmd.h"

int ucmd_mcu_reset(int argc, char** argv)
{
    NVIC_SystemReset();
    return -1;
}

// define command list
command_t cmd_list[] = {
    {
        .cmd  = "help",
        .help = "print available commands with their help text",
        .fn   = print_help_cb,
    },

    {
        .cmd  = "reset",
        .help = "reset mcu",
        .fn   = ucmd_mcu_reset,
    },

    // {
    //   .cmd  = "mem",
    //   .help = "memory man, use mem help",
    //   .fn   = ucmd_mem,
    // },

    // {
    //   .cmd  = "time",
    //   .help = "rtc time. to set type time hh mm ss",
    //   .fn   = ucmd_time,
    // },

    // {
    //   .cmd  = "coremark",
    //   .help = "coremark",
    //   .fn   = coremark,
    // },

    // {
    //   .cmd  = "sd",
    //   .help = "sd card test utils",
    //   .fn   = ucmd_sd,
    // },

    // {
    //   .cmd  = "imu",
    //   .help = "imu test code",
    //   .fn   = ucmd_imu,
    // },

    // {
    //   .cmd  = "i2c",
    //   .help = "i2c tool",
    //   .fn   = ucmd_i2c,
    // },

    // {
    //   .cmd  = "bmp",
    //   .help = "bmp tool",
    //   .fn   = ucmd_bmp,
    // },

    // {
    //   .cmd  = "term",
    //   .help = "term test",
    //   .fn   = ucmd_term_test,
    // },


    {}, // null list terminator DON'T FORGET THIS!
};
