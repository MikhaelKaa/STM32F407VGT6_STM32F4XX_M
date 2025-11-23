/* SPDX-License-Identifier: MIT */
/*
 * dev_rng.h - Hardware Random Number Generator interface for STM32F407
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

#ifndef DEV_RNG_H
#define DEV_RNG_H

#include "dev_interface.h"
#include <stdint.h>

// RNG-specific ioctl commands
#define RNG_INIT       (INTERFACE_CMD_DEVICE + 0)
#define RNG_DEINIT     (INTERFACE_CMD_DEVICE + 1)
#define RNG_GET_STATUS (INTERFACE_CMD_DEVICE + 2)
#define RNG_RESET      (INTERFACE_CMD_DEVICE + 3)
#define RNG_SELF_TEST  (INTERFACE_CMD_DEVICE + 4)

// RNG status flags
#define RNG_STATUS_READY       0x00000001
#define RNG_STATUS_ERROR       0x00000002
#define RNG_STATUS_SEED_ERROR  0x00000004
#define RNG_STATUS_CLOCK_ERROR 0x00000008

// RNG configuration
#define RNG_TIMEOUT            10000  // Timeout for RNG operations

// RNG device instance
const interface_t* dev_rng_get(void);

#endif /* DEV_RNG_H */