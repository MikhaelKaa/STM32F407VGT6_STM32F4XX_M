/* SPDX-License-Identifier: MIT */
/*
 * uart1.h - POSIX-style UART interface implementation
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

#ifndef UART1_H
#define UART1_H

#include "dev_interface.h"
#include <stddef.h>
#include <stdint.h>

// Buffer sizes
#define UART_TX_BUFFER_SIZE 256
#define UART_RX_BUFFER_SIZE 256

// UART-specific ioctrl commands
#define UART_GET_AVAILABLE  (INTERFACE_CMD_DEVICE + 0)
#define UART_FLUSH          (INTERFACE_CMD_DEVICE + 1)

// Global UART device instance
extern const interface_t uart1_dev;

#endif /* UART1_H */