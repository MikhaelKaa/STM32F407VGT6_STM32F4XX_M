/* SPDX-License-Identifier: MIT */
/*
 * sdio_test.c - sdio test utility
 * 
 * Copyright (c) 2025 Michael Kaa
 * 
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include "dev_sdio.h"
#include "stm32f407xx.h"

// Объявляем функцию из dev_sdio.c
extern int sdio_send_command(uint32_t cmd, uint32_t arg);

const interface_t * sdio = NULL; 

#ifdef BAREMETAL
int ucmd_sdio(int argc, char* argv[])
#define ENDL "\r\n"
#else
int main(int argc, char* argv[])
#define ENDL "\n"
#endif // BAREMETAL

{
    sdio = dev_sdio_get();
    uint32_t cmd, arg;

    if(NULL == sdio) {
        printf("sdio instance error" ENDL);
        return -ENODEV;
    }

    if (argc < 2) {
        printf("Usage: sdio [cmd [cmd] [arg] | init | version]" ENDL);
        return -EINVAL;
    }
    
    // sdio version
    if ((2 == argc) && (strcmp(argv[1], "version") == 0))
    {
        const char * sdio_version = NULL;
        sdio->ioctl(SDIO_GET_VERSION, (void*)&sdio_version);
        printf("sdio version: %s" ENDL, sdio_version);
        return 0;
    }

    // sdio init
    if ((2 == argc) && (strcmp(argv[1], "init") == 0))
    {
        int ret = sdio->ioctl(SDIO_INIT, 0);
        printf("sdio init ret: %d" ENDL, ret);
        return 0;
    }
    
    // sdio cmd [cmd] [arg]
    if ((4 == argc) && (strcmp(argv[1], "cmd") == 0))
    {
        if (sscanf(argv[2], "%lu", &cmd) != 1)
        {
            printf("Invalid cmd format" ENDL);
            return -EINVAL;
        }
        if (sscanf(argv[3], "%lu", &arg) != 1)
        {
            printf("Invalid arg format" ENDL);
            return -EINVAL;
        }
        printf("sdio_send_command(%lu, 0x%08lx)" ENDL, cmd, arg);
        int ret = sdio_send_command(cmd, arg);
        printf("sdio cmd ret: %d" ENDL, ret);
        
        // 
        if (ret == 0 && cmd != 0 && cmd != 55) {
            printf("Response: 0x%08lx" ENDL, SDIO->RESP1);
        }
        return 0;
    } 

    return 0;
}

#undef ENDL