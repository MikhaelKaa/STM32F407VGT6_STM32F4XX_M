/* SPDX-License-Identifier: MIT */
/*
 * dev_sdio.h - POSIX-style SDIO interface implementation
 * 
 * Copyright (c) 2025 Michael Kaa
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef DEV_SDIO_H
#define DEV_SDIO_H

#include <stddef.h>
#include <stdint.h>
#include "dev_interface.h"

// SDIO-specific ioctl commands
#define SDIO_INIT               (INTERFACE_CMD_DEVICE + 0)
#define SDIO_DEINIT             (INTERFACE_CMD_DEVICE + 1)
#define SDIO_GET_CARD_STATUS    (INTERFACE_CMD_DEVICE + 2)
#define SDIO_GET_VERSION        (INTERFACE_CMD_DEVICE + 3)
#define SDIO_GET_CARD_INFO      (INTERFACE_CMD_DEVICE + 4)
#define SDIO_SET_CLOCK          (INTERFACE_CMD_DEVICE + 5)
#define SDIO_SET_BUS_WIDTH      (INTERFACE_CMD_DEVICE + 6)

// SD Card status
#define SD_CARD_READY           0x01
#define SD_CARD_WRITE_PROTECT   0x02
#define SD_CARD_LOCKED          0x04

// SDIO Bus width
#define SDIO_BUS_WIDTH_1BIT     0
#define SDIO_BUS_WIDTH_4BIT     1

const interface_t* dev_sdio_get(void);

// for test
int sdio_send_command(uint32_t cmd, uint32_t arg);

#endif /* DEV_SDIO_H */
