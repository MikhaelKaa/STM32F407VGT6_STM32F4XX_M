/* SPDX-License-Identifier: MIT */
/*
 * dev_interface.h - Unified device interface abstraction
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

#ifndef _INTERFACE_H
#define _INTERFACE_H

#include <stddef.h>
#include <stdint.h>
#include <errno.h>


/**
 * struct interface - Unified device interface
 * @open: Initialize and open device
 * @close: Close and deinitialize device
 * @read: Read data from device
 * @write: Write data to device
 * @ioctrl: Device control and configuration
 *
 * Generic interface for all device types in the system.
 * Functions should return 0 on success or negative errno on error.
 */
typedef struct interface
{
    int (*open)(void);
    int (*close)(void);
    int (*read)(void* buf, size_t len);
    int (*write)(const void* buf, size_t len);
    int (*ioctrl)(int cmd, void* arg);
} interface_t;

/* Common ioctrl commands */
#define INTERFACE_GET_INFO   0x1000 /* Get device info */
#define INTERFACE_GET_STATUS 0x1001 /* Get device status */
#define INTERFACE_RESET      0x1002 /* Reset device */
#define INTERFACE_SET_CONFIG 0x1003 /* Set device configuration */
#define INTERFACE_GET_CONFIG 0x1004 /* Get device configuration */

/* Device-specific command space */
#define INTERFACE_CMD_DEVICE 0x8000 /* Base for device-specific commands */

#endif /* _INTERFACE_H */