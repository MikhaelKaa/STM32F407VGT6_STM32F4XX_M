/* SPDX-License-Identifier: MIT */
/*
 * uart_ping.c - UART test utility for STM32F407
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include "uart_ping.h"

interface_t* dev_uart_ping = NULL;

uint8_t tx_buf[1024] = {0};

#ifdef BAREMETAL
int ucmd_uping(int argc, char* argv[])
#define ENDL "\r\n"
#else
int main(int argc, char* argv[])
#define ENDL "\n"
#endif // BAREMETAL
{
    uint8_t pattern;
    uint32_t count;
    uint8_t rx_buffer[256];
    int rx_bytes;
    int available;
    size_t to_read;


    if(!dev_uart_ping){
        printf("dev_uart_ping is NULL"ENDL);
        return -EFAULT;
    }

    // Check arguments
    if (argc < 2) {
        printf("Usage: uart_ping <hex_byte> [count]" ENDL);
        printf("  <hex_byte> - byte value to send (hex, e.g., 55, AA, FF)" ENDL);
        printf("  [count]    - number of bytes to send (default: 1)" ENDL);
        return -EINVAL;
    }

    // Parse pattern byte
    if (sscanf(argv[1], "%hhx", &pattern) != 1) {
        printf("Invalid byte format: %s" ENDL, argv[1]);
        printf("Use hex values (00-FF)" ENDL);
        return -EINVAL;
    }

    // Parse count (default to 1)
    count = 1;
    if (argc >= 3) {
        if (sscanf(argv[2], "%lu", &count) != 1 || count == 0) {
            printf("Invalid count: %s" ENDL, argv[2]);
            return -EINVAL;
        }
    }

    // Limit maximum count to prevent buffer overflow
    if (count > 1024) {
        printf("Count too large, limiting to 1024" ENDL);
        count = 1024;
    }

    printf("UART Ping Test" ENDL);
    printf("Sending %lu bytes of 0x%02x" ENDL, count, pattern);

    memset(tx_buf, pattern, count);

    int result = dev_uart_ping->write(tx_buf, count);
    if (result != (int)count) {
        printf("Send error: %d" ENDL, result);
        return result;
    }

    printf("Send completed" ENDL);

    // Check for received data
    dev_uart_ping->ioctrl(INTERFACE_CMD_DEVICE, &available);
    
    if (available > 0) {
        printf("Received %d bytes:" ENDL, available);
        
        // Determine how many bytes to read (safe conversion)
        to_read = (size_t)available;
        if (to_read > sizeof(rx_buffer)) {
            to_read = sizeof(rx_buffer);
        }
        
        // Read available data
        rx_bytes = dev_uart_ping->read(rx_buffer, to_read);
        
        if (rx_bytes > 0) {
            // Print in hex format
            for (int i = 0; i < rx_bytes; i++) {
                printf("%02x ", rx_buffer[i]);
                if ((i + 1) % 16 == 0) printf(ENDL);
            }
            if (rx_bytes % 16 != 0) printf(ENDL);
            
            // Print in ASCII format (for printable characters)
            printf("ASCII: ");
            for (int i = 0; i < rx_bytes; i++) {
                if (rx_buffer[i] >= 32 && rx_buffer[i] <= 126) {
                    printf("%c", rx_buffer[i]);
                } else {
                    printf(".");
                }
            }
            printf(ENDL);
        }
    } else {
        printf("No data received" ENDL);
    }

    return 0;
}

#undef ENDL