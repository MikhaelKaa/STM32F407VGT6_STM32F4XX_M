/* SPDX-License-Identifier: MIT */
/*
 * ucmd_rng.c - RNG utility
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

#include "dev_interface.h"

interface_t* dev_rng_gen = NULL;
uint8_t rng_buffer[1024] = {0};

#ifdef BAREMETAL
int ucmd_rng(int argc, char* argv[])
#define ENDL "\r\n"
#else
int main(int argc, char* argv[])
#define ENDL "\n"
#endif
{
    uint32_t count;
    int bytes_read;

    if(!dev_rng_gen){
        printf("dev_rng_gen is NULL" ENDL);
        return -EFAULT;
    }

    // Check arguments
    if (argc < 2) {
        printf("Usage: rng <byte_count>" ENDL);
        printf("  <byte_count> - number of bytes to generate (1-1024)" ENDL);
        return -EINVAL;
    }

    // Parse byte count
    if (sscanf(argv[1], "%lu", &count) != 1 || count == 0) {
        printf("Invalid byte count: %s" ENDL, argv[1]);
        return -EINVAL;
    }

    // Limit maximum count to prevent buffer overflow
    if (count > 1024) {
        printf("Count too large, limiting to 1024" ENDL);
        count = 1024;
    }

    printf("Generating %lu random bytes..." ENDL, count);

    // Generate random bytes
    bytes_read = dev_rng_gen->read(rng_buffer, count);
    
    if (bytes_read != (int)count) {
        printf("RNG read error: %d" ENDL, bytes_read);
        return bytes_read;
    }

    printf("Generation completed" ENDL);

    // Print in hex format
    for (uint32_t i = 0; i < count; i++) {
        printf("%02x", rng_buffer[i]);
        if ((i + 1) % 16 == 0) printf(ENDL);
    }
    if (count % 16 != 0) printf(ENDL);

    return 0;
}

int app_dev_rng_set(interface_t* dev)
{
    if (dev != NULL)
    {
        dev_rng_gen = dev;
        return 0;
    }
    else
    {
        return -EFAULT;
    }
}

#undef ENDL