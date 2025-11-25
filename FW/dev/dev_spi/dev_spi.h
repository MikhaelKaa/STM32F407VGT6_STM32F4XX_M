/* SPDX-License-Identifier: MIT */
/*
 * dev_spi.h - POSIX-style SPI interface implementation
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

#ifndef DEV_SPI_H
#define DEV_SPI_H

#include <stddef.h>
#include <stdint.h>
#include "dev_interface.h"

// SPI-specific ioctl commands
#define SPI_INIT           (INTERFACE_CMD_DEVICE + 0)
#define SPI_DEINIT         (INTERFACE_CMD_DEVICE + 1)
#define SPI_GET_AVAILABLE  (INTERFACE_CMD_DEVICE + 2)
#define SPI_GET_VERSION    (INTERFACE_CMD_DEVICE + 3)
#define SPI_SET_CS         (INTERFACE_CMD_DEVICE + 4)  /* Set chip select state */
#define SPI_SET_MODE       (INTERFACE_CMD_DEVICE + 5)  /* Set SPI mode */
#define SPI_SET_SPEED      (INTERFACE_CMD_DEVICE + 6)  /* Set SPI speed */

const interface_t* dev_spi1_get(void);

#endif /* DEV_SPI_H */