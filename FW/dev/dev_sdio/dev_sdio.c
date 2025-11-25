/* SPDX-License-Identifier: MIT */
/*
 * dev_sdio.c - POSIX-style SDIO interface implementation for stm32f407xx
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

#include <string.h>
#include <errno.h>

#include "dev_sdio.h"
#include "stm32f407xx.h"

#ifndef SDIO_TIMEOUT
#define SDIO_TIMEOUT (1000000U)
#endif // SDIO_TIMEOUT

// Driver version
const char *dev_sdio_version = "1.0.0";

// Card information structure
typedef struct {
    uint32_t card_type;
    uint32_t capacity;
    uint32_t block_size;
    uint32_t rca;
    uint8_t status;
} sd_card_info_t;

static sd_card_info_t card_info = {0};
static volatile uint8_t transfer_complete = 0;
static volatile uint8_t transfer_error = 0;

// Private function prototypes
static int sdio_wait_response(uint32_t mask, uint32_t value, uint32_t timeout);
static int sdio_send_command(uint32_t cmd, uint32_t arg);
static int sdio_initialize_card(void);
static int sdio_set_bus_width(uint32_t width);

// Open SDIO (interface implementation)
static int sdio_init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
    RCC->APB2ENR |= RCC_APB2ENR_SDIOEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    
    // Configure GPIO for SDIO
    // PC8 - DAT0, PC9 - DAT1, PC10 - DAT2, PC11 - DAT3, PC12 - CLK
    GPIOC->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9 | 
                      GPIO_MODER_MODER10 | GPIO_MODER_MODER11 | 
                      GPIO_MODER_MODER12);
    GPIOC->MODER |= (2 << GPIO_MODER_MODER8_Pos) | (2 << GPIO_MODER_MODER9_Pos) |
                    (2 << GPIO_MODER_MODER10_Pos) | (2 << GPIO_MODER_MODER11_Pos) |
                    (2 << GPIO_MODER_MODER12_Pos);
    
    // Alternate function AF12 for SDIO
    GPIOC->AFR[1] &= ~(GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9 | 
                       GPIO_AFRH_AFSEL10 | GPIO_AFRH_AFSEL11 | 
                       GPIO_AFRH_AFSEL12);
    GPIOC->AFR[1] |= (12 << (4 * 0)) | (12 << (4 * 1)) | 
                     (12 << (4 * 2)) | (12 << (4 * 3)) |
                     (12 << (4 * 4));
    
    // PD2 - CMD
    GPIOD->MODER &= ~GPIO_MODER_MODER2;
    GPIOD->MODER |= (2 << GPIO_MODER_MODER2_Pos);
    GPIOD->AFR[0] &= ~GPIO_AFRL_AFSEL2;
    GPIOD->AFR[0] |= (12 << (4 * 2));
    
    // High speed
    GPIOC->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED8_Pos) | (3 << GPIO_OSPEEDR_OSPEED9_Pos) |
                      (3 << GPIO_OSPEEDR_OSPEED10_Pos) | (3 << GPIO_OSPEEDR_OSPEED11_Pos) |
                      (3 << GPIO_OSPEEDR_OSPEED12_Pos);
    GPIOD->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED2_Pos);
    
    // Pull-ups
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9 | 
                      GPIO_PUPDR_PUPD10 | GPIO_PUPDR_PUPD11);
    GPIOC->PUPDR |= (1 << GPIO_PUPDR_PUPD8_Pos) | (1 << GPIO_PUPDR_PUPD9_Pos) |
                    (1 << GPIO_PUPDR_PUPD10_Pos) | (1 << GPIO_PUPDR_PUPD11_Pos);
    
    // Configure SDIO peripheral
    SDIO->POWER = 0;  // Power off
    for(volatile int i = 0; i < 1000; i++);
    
    SDIO->POWER = SDIO_POWER_PWRCTRL_0;  // Power on
    for(volatile int i = 0; i < 1000; i++);
    
    // Set initial clock (400kHz for initialization)
    SDIO->CLKCR = (0x76 << SDIO_CLKCR_CLKDIV_Pos) |  // 400kHz from 48MHz
                  SDIO_CLKCR_CLKEN | 
                  SDIO_CLKCR_PWRSAV;
    
    SDIO->DTIMER = 0xFFFFFFFF;  // Data timeout
    
    // Clear flags
    SDIO->ICR = 0xFFFFFFFF;
    
    // Enable interrupts
    SDIO->MASK = SDIO_MASK_DCRCFAILIE | SDIO_MASK_DTIMEOUTIE | 
                 SDIO_MASK_DATAENDIE | SDIO_MASK_CMDRENDIE | 
                 SDIO_MASK_CMDSENTIE | SDIO_MASK_RXOVERRIE | 
                 SDIO_MASK_TXUNDERRIE;
    
    NVIC_EnableIRQ(SDIO_IRQn);
    
    // Initialize card
    int ret = sdio_initialize_card();
    if (ret != 0) {
        return ret;
    }
    
    // Set higher clock speed after initialization (24MHz)
    SDIO->CLKCR = (1 << SDIO_CLKCR_CLKDIV_Pos) |  // 24MHz from 48MHz
                  SDIO_CLKCR_CLKEN | 
                  SDIO_CLKCR_PWRSAV | 
                  SDIO_CLKCR_WIDBUS_0;  // 4-bit bus
    
    return 0;
}

// Close SDIO (interface implementation)
static int sdio_deinit(void) {
    // Disable SDIO
    SDIO->POWER = 0;
    SDIO->CLKCR = 0;
    
    // Disable interrupts
    NVIC_DisableIRQ(SDIO_IRQn);
    SDIO->MASK = 0;
    
    // Disable clocks
    RCC->APB2ENR &= ~RCC_APB2ENR_SDIOEN;
    
    return 0;
}

// Wait for response
static int sdio_wait_response(uint32_t mask, uint32_t value, uint32_t timeout) {
    while (timeout--) {
        uint32_t status = SDIO->STA;
        
        if (status & (SDIO_STA_CMDREND | SDIO_STA_CMDSENT | SDIO_STA_CTIMEOUT | SDIO_STA_CCRCFAIL)) {
            if ((status & mask) == value) {
                SDIO->ICR = mask;  // Clear flags
                return 0;
            }
            return -EIO;
        }
    }
    return -ETIMEDOUT;
}

// Send command to SD card
static int sdio_send_command(uint32_t cmd, uint32_t arg) {
    SDIO->ARG = arg;
    SDIO->CMD = cmd;
    
    return sdio_wait_response(SDIO_STA_CMDREND | SDIO_STA_CMDSENT, 
                             SDIO_STA_CMDREND | SDIO_STA_CMDSENT, 
                             SDIO_TIMEOUT);
}

// Initialize SD card
static int sdio_initialize_card(void) {
    int ret;
    
    // CMD0 - GO_IDLE_STATE
    ret = sdio_send_command(SDIO_CMD_CPSMEN | 0, 0);
    if (ret != 0) return ret;
    
    // CMD8 - SEND_IF_COND
    ret = sdio_send_command(SDIO_CMD_CPSMEN | 8, 0x1AA);
    if (ret != 0) return ret;
    
    // CMD55 - APP_CMD
    ret = sdio_send_command(SDIO_CMD_CPSMEN | 55, 0);
    if (ret != 0) return ret;
    
    // ACMD41 - SD_APP_OP_COND  
    ret = sdio_send_command(SDIO_CMD_CPSMEN | 41, 0x40000000);
    if (ret != 0) return ret;
    
    // CMD2 - ALL_SEND_CID
    ret = sdio_send_command(SDIO_CMD_CPSMEN | 2, 0);
    if (ret != 0) return ret;
    
    // CMD3 - SEND_RELATIVE_ADDR
    ret = sdio_send_command(SDIO_CMD_CPSMEN | 3, 0);
    if (ret != 0) return ret;
    
    card_info.rca = SDIO->RESP1;
    
    // CMD9 - SEND_CSD
    ret = sdio_send_command(SDIO_CMD_CPSMEN | 9, card_info.rca);
    if (ret != 0) return ret;
    
    // CMD7 - SELECT_CARD
    ret = sdio_send_command(SDIO_CMD_CPSMEN | 7, card_info.rca);
    if (ret != 0) return ret;
    
    // Set bus width to 4-bit
    ret = sdio_set_bus_width(SDIO_BUS_WIDTH_4BIT);
    if (ret != 0) return ret;
    
    card_info.status = SD_CARD_READY;
    card_info.block_size = 512;  // Standard SD block size
    
    return 0;
}

// Set bus width
static int sdio_set_bus_width(uint32_t width) {
    // CMD55 - APP_CMD
    int ret = sdio_send_command(SDIO_CMD_CPSMEN | 55, card_info.rca);
    if (ret != 0) return ret;
    
    // ACMD6 - SET_BUS_WIDTH
    uint32_t acmd6_arg = (width == SDIO_BUS_WIDTH_4BIT) ? 2 : 0;
    ret = sdio_send_command(SDIO_CMD_CPSMEN | 6, acmd6_arg);
    
    if (ret == 0) {
        // Update SDIO bus width
        uint32_t clkcr = SDIO->CLKCR;
        clkcr &= ~SDIO_CLKCR_WIDBUS;
        if (width == SDIO_BUS_WIDTH_4BIT) {
            clkcr |= SDIO_CLKCR_WIDBUS_0;
        }
        SDIO->CLKCR = clkcr;
    }
    
    return ret;
}

// Read data from SD card (interface implementation)
static int sdio_read(void *buf, size_t count) {
    if (buf == NULL || count == 0) {
        return -EINVAL;
    }
    
    if ((count % 512) != 0) {
        return -EINVAL;  // SD cards require block-aligned reads
    }
    
    uint32_t block_count = count / 512;
    uint32_t start_block = 0;  // In real implementation, this would be a parameter
    
    transfer_complete = 0;
    transfer_error = 0;
    
    // Setup data transfer
    SDIO->DLEN = count;
    SDIO->DCTRL = (9 << SDIO_DCTRL_DBLOCKSIZE_Pos) |  // 512 bytes
                  SDIO_DCTRL_DTDIR |                  // Read transfer
                  SDIO_DCTRL_DTEN;                    // Enable data transfer
    
    // Send read command (CMD17 for single block, CMD18 for multiple)
    uint32_t cmd = (block_count > 1) ? 18 : 17;
    int ret = sdio_send_command(SDIO_CMD_CPSMEN | cmd | SDIO_CMD_WAITRESP_0, 
                               start_block);
    if (ret != 0) return ret;
    
    // Wait for transfer complete
    uint32_t timeout = SDIO_TIMEOUT;
    while (!transfer_complete && !transfer_error && timeout--) {
        __asm__("nop");
    }
    
    if (timeout == 0) return -ETIMEDOUT;
    if (transfer_error) return -EIO;
    
    // Read data from FIFO
    uint8_t *buffer = (uint8_t *)buf;
    for (size_t i = 0; i < count / 4U; i++) {
        uint32_t data = SDIO->FIFO;
        buffer[i * 4U + 0U] = (uint8_t)( data         & 0xFFU);
        buffer[i * 4U + 1U] = (uint8_t)((data >> 8U)  & 0xFFU);
        buffer[i * 4U + 2U] = (uint8_t)((data >> 16U) & 0xFFU);
        buffer[i * 4U + 3U] = (uint8_t)((data >> 24U) & 0xFFU);
    }
    
    return (int)count;
}

// Write data to SD card (interface implementation)
static int sdio_write(const void *buf, size_t count) {
    if (buf == NULL || count == 0) {
        return -EINVAL;
    }
    
    if ((count % 512) != 0) {
        return -EINVAL;  // SD cards require block-aligned writes
    }
    
    uint32_t block_count = count / 512;
    uint32_t start_block = 0;  // In real implementation, this would be a parameter
    
    transfer_complete = 0;
    transfer_error = 0;
    
    // Setup data transfer
    SDIO->DLEN = count;
    SDIO->DCTRL = (9 << SDIO_DCTRL_DBLOCKSIZE_Pos) |  // 512 bytes
                  SDIO_DCTRL_DTEN;                    // Enable data transfer (write)
    
    // Write data to FIFO
    const uint8_t *buffer = (const uint8_t *)buf;
    for (size_t i = 0; i < count / 4; i++) {
        uint32_t data = buffer[i * 4] | 
                       (buffer[i * 4 + 1] << 8) |
                       (buffer[i * 4 + 2] << 16) |
                       (buffer[i * 4 + 3] << 24);
        SDIO->FIFO = data;
    }
    
    // Send write command (CMD24 for single block, CMD25 for multiple)
    uint32_t cmd = (block_count > 1) ? 25 : 24;
    int ret = sdio_send_command(SDIO_CMD_CPSMEN | cmd | SDIO_CMD_WAITRESP_0, 
                               start_block);
    if (ret != 0) return ret;
    
    // Wait for transfer complete
    uint32_t timeout = SDIO_TIMEOUT;
    while (!transfer_complete && !transfer_error && timeout--) {
        __asm__("nop");
    }
    
    if (timeout == 0) return -ETIMEDOUT;
    if (transfer_error) return -EIO;
    
    return (int)count;
}

// IO Control for SDIO (interface implementation)
static int sdio_ioctl(int cmd, void *arg) {
    switch (cmd) {
        case SDIO_INIT:
            return sdio_init();

        case SDIO_DEINIT:
            return sdio_deinit();

        case SDIO_GET_CARD_STATUS:
            if (arg != NULL) {
                *(uint8_t *)arg = card_info.status;
            }
            return 0;

        case SDIO_GET_VERSION:
            if (arg != NULL) {
                *(const char **)arg = dev_sdio_version;
                return 0;
            }
            return -EINVAL;

        case SDIO_GET_CARD_INFO:
            if (arg != NULL) {
                *(sd_card_info_t *)arg = card_info;
                return 0;
            }
            return -EINVAL;

        case SDIO_SET_CLOCK:
            if (arg != NULL) {
                uint32_t clock = *(uint32_t *)arg;
                uint32_t clkdiv = (48000000 / clock) - 1;
                SDIO->CLKCR = (SDIO->CLKCR & ~SDIO_CLKCR_CLKDIV) | 
                              (clkdiv << SDIO_CLKCR_CLKDIV_Pos);
                return 0;
            }
            return -EINVAL;

        case SDIO_SET_BUS_WIDTH:
            if (arg != NULL) {
                uint32_t width = *(uint32_t *)arg;
                return sdio_set_bus_width(width);
            }
            return -EINVAL;

        default:
            return -ENOTSUP;  // Command not supported
    }
}

// SDIO device instance
static const interface_t dev_sdio = {
    .read = sdio_read, 
    .write = sdio_write, 
    .ioctl = sdio_ioctl
};

const interface_t* dev_sdio_get(void)
{
    return (const interface_t*) &dev_sdio;
}

// SDIO Interrupt Handler
void SDIO_IRQHandler(void) {
    uint32_t status = SDIO->STA;
    
    if (status & SDIO_STA_DATAEND) {
        transfer_complete = 1;
        SDIO->ICR = SDIO_ICR_DATAENDC;
    }
    
    if (status & (SDIO_STA_DCRCFAIL | SDIO_STA_DTIMEOUT | SDIO_STA_RXOVERR | SDIO_STA_TXUNDERR)) {
        transfer_error = 1;
        SDIO->ICR = SDIO_ICR_DCRCFAILC | SDIO_ICR_DTIMEOUTC | 
                    SDIO_ICR_RXOVERRC | SDIO_ICR_TXUNDERRC;
    }
    
    if (status & SDIO_STA_CMDREND) {
        SDIO->ICR = SDIO_ICR_CMDRENDC;
    }
    
    if (status & SDIO_STA_CMDSENT) {
        SDIO->ICR = SDIO_ICR_CMDSENTC;
    }
}
