
// #include <stdio.h>

#include "stm32f407xx.h"
#include "memory_man.h"
#include "ucmd.h"
#include "uart_ping.h"
#include "rng_gen.h"
#include "svc_test.h"

int ucmd_mcu_reset(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    
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

    {
      .cmd  = "mem",
      .help = "memory man, use mem help",
      .fn   = ucmd_mem,
    },

    {
      .cmd  = "uping",
      .help = "uart test utility",
      .fn   = ucmd_uping,
    },

        {
      .cmd  = "rng",
      .help = "rng generate utility",
      .fn   = ucmd_rng,
    },
    {
      .cmd  = "svc",
      .help = "SVCall demo utility",
      .fn   = ucmd_tscv,
    },

    {0}, // null list terminator DON'T FORGET THIS!
};
