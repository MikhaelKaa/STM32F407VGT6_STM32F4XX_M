/* SPDX-License-Identifier: MIT */
/*
 * spi1.c - POSIX-style SPI interface implementation for stm32f407xx
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

#include "dev_spi.h"
#include "stm32f407xx.h"

#ifndef SPI1_TX_TIMEOUT
#define SPI1_TX_TIMEOUT (10000000U)
#endif // SPI1_TX_TIMEOUT

// Buffer sizes
#ifndef SPI1_TX_BUFFER_SIZE
#define SPI1_TX_BUFFER_SIZE (256U)
#endif // SPI1_TX_BUFFER_SIZE

#ifndef SPI1_RX_BUFFER_SIZE
#define SPI1_RX_BUFFER_SIZE (256U)
#endif // SPI1_RX_BUFFER_SIZE

// Driver version
const char *dev_spi1_version = "1.0.0";

// Static buffers
static uint8_t tx_buffer[SPI1_TX_BUFFER_SIZE];
static volatile uint8_t rx_buffer[SPI1_RX_BUFFER_SIZE];

// Ring buffer pointers for RX
static volatile uint32_t rx_read_pos = 0;
static volatile uint32_t rx_write_pos = 0;

// Transfer state
static volatile uint8_t transfer_in_progress = 0;
static volatile uint32_t bytes_received = 0;

// Chip select state
static volatile uint8_t cs_state = 1; // High by default (inactive)

static int spi_available(void);
static void spi_set_cs(uint8_t state);

// Initialize SPI (interface implementation)
static int spi_init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    
    // Configure GPIO for SPI1
    // PA15 - CS (output), PB3 - SCK, PB4 - MISO, PB5 - MOSI
    
    // Configure CS (PA15) as output
    GPIOA->MODER &= ~GPIO_MODER_MODER15;
    GPIOA->MODER |= (1 << GPIO_MODER_MODER15_Pos);
    GPIOA->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED15_Pos);
    spi_set_cs(1); // Start with CS high
    
    // Configure SPI pins (PB3, PB4, PB5) in alternate function mode
    GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    GPIOB->MODER |= (2 << GPIO_MODER_MODER3_Pos) | 
                    (2 << GPIO_MODER_MODER4_Pos) | 
                    (2 << GPIO_MODER_MODER5_Pos);
    
    // Alternate function AF5 for SPI1
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL3 | GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5);
    GPIOB->AFR[0] |= (5 << (4 * 3)) | (5 << (4 * 4)) | (5 << (4 * 5));
    
    // High speed
    GPIOB->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED3_Pos) | 
                      (3 << GPIO_OSPEEDR_OSPEED4_Pos) | 
                      (3 << GPIO_OSPEEDR_OSPEED5_Pos);
    
    // Configure SPI1
    SPI1->CR1 = SPI_CR1_SSM |        // Software slave management
                SPI_CR1_SSI |        // Internal slave select
                SPI_CR1_MSTR |       // Master mode
                SPI_CR1_SPE |        // SPI enable
                (3 << SPI_CR1_BR_Pos) | // Baud rate: PCLK/16
                SPI_CR1_CPOL |       // Clock polarity high
                SPI_CR1_CPHA;        // Clock phase 2nd edge
    
    // Enable DMA for TX and RX
    SPI1->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
    
    // Configure DMA for transmission (SPI1_TX -> DMA2 Stream3)
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream3->CR & DMA_SxCR_EN);
    
    DMA2_Stream3->PAR = (uint32_t)&SPI1->DR;
    DMA2_Stream3->M0AR = (uint32_t)tx_buffer;
    DMA2_Stream3->NDTR = 0;
    
    DMA2_Stream3->CR = (3 << DMA_SxCR_CHSEL_Pos) |  // Channel 3
                       DMA_SxCR_MINC |              // Memory increment
                       DMA_SxCR_DIR_0 |             // Memory to peripheral
                       DMA_SxCR_TCIE;               // Transfer complete interrupt
    
    // Configure DMA for reception (SPI1_RX -> DMA2 Stream0) - Circular mode
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN);
    
    DMA2_Stream0->PAR = (uint32_t)&SPI1->DR;
    DMA2_Stream0->M0AR = (uint32_t)rx_buffer;
    DMA2_Stream0->NDTR = SPI1_RX_BUFFER_SIZE;
    
    DMA2_Stream0->CR = (3 << DMA_SxCR_CHSEL_Pos) |  // Channel 3
                       DMA_SxCR_MINC |              // Memory increment
                       DMA_SxCR_CIRC |              // Circular mode
                       DMA_SxCR_HTIE |              // Half transfer interrupt
                       DMA_SxCR_TCIE;               // Transfer complete interrupt
    
    // Enable DMA reception
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    
    // Enable interrupts
    NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    
    // Clear buffers
    memset(tx_buffer, 0, SPI1_TX_BUFFER_SIZE);
    memset((void *)rx_buffer, 0, SPI1_RX_BUFFER_SIZE);
    
    return 0;  // Success
}

// Deinitialize SPI (interface implementation)
static int spi_deinit(void) {
    // Disable SPI and DMA
    SPI1->CR1 &= ~SPI_CR1_SPE;
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    
    // Disable interrupts
    NVIC_DisableIRQ(DMA2_Stream3_IRQn);
    NVIC_DisableIRQ(DMA2_Stream0_IRQn);
    
    return 0;  // Success
}

// Set chip select state
static void spi_set_cs(uint8_t state) {
    cs_state = state;
    if (state) {
        GPIOA->BSRR = GPIO_BSRR_BS15;  // Set CS high
    } else {
        GPIOA->BSRR = GPIO_BSRR_BR15;  // Set CS low
    }
}

// Write data to SPI (interface implementation)
static int spi_write(const void *buf, size_t count) {
    if (buf == NULL || count == 0) {
        return -EINVAL;
    }
    
    if (count > SPI1_TX_BUFFER_SIZE) {
        count = SPI1_TX_BUFFER_SIZE;
    }

    // Wait for previous transfer to complete
    uint32_t timeout = SPI1_TX_TIMEOUT;
    while (transfer_in_progress && timeout--) {
        __asm__("nop");
    }
    
    if (timeout == 0) {
        return -ETIMEDOUT;
    }

    // Copy data to buffer
    memcpy(tx_buffer, buf, count);
    transfer_in_progress = 1;
    bytes_received = 0;
    
    // Activate chip select
    spi_set_cs(0);
    
    // Configure and start DMA transfers
    DMA2_Stream3->M0AR = (uint32_t)tx_buffer;
    DMA2_Stream3->NDTR = count;
    
    // Update RX buffer position tracking
    uint32_t current_ndtr = DMA2_Stream0->NDTR;
    rx_write_pos = (SPI1_RX_BUFFER_SIZE - current_ndtr) % SPI1_RX_BUFFER_SIZE;
    
    // Enable DMA streams
    DMA2_Stream3->CR |= DMA_SxCR_EN;
    
    return (int)count;
}

// Read data from SPI (interface implementation)
static int spi_read(void *buf, size_t count) {
    if (buf == NULL) {
        return -EINVAL;
    }
    
    uint8_t *buffer = (uint8_t *)buf;
    size_t bytes_read = 0;
    
    // Calculate available bytes in ring buffer
    int available = spi_available();
    if (available == 0) {
        return 0;  // No data available
    }
    
    if (count > (size_t)available) {
        count = (size_t)available;
    }
    
    // Read data from ring buffer
    for (size_t i = 0; i < count; i++) {
        buffer[i] = rx_buffer[rx_read_pos];
        rx_read_pos = (rx_read_pos + 1) % SPI1_RX_BUFFER_SIZE;
        bytes_read++;
    }
    
    return (int)bytes_read;
}

// Check how many bytes are available to read
static int spi_available(void) {
    uint32_t current_ndtr = DMA2_Stream0->NDTR;
    uint32_t dma_write_pos = (SPI1_RX_BUFFER_SIZE - current_ndtr) % SPI1_RX_BUFFER_SIZE;
    
    if (dma_write_pos >= rx_read_pos) {
        return (int)(dma_write_pos - rx_read_pos);
    } else {
        return (int)(SPI1_RX_BUFFER_SIZE - rx_read_pos + dma_write_pos);
    }
}

// IO Control for SPI (interface implementation)
static int spi_ioctl(int cmd, void *arg) {
    switch (cmd) {
        case SPI_INIT:
            spi_init();
            return 0;

        case SPI_DEINIT:
            spi_deinit();
            return 0;

        case SPI_GET_AVAILABLE:
            if (arg != NULL) {
                *(int *)arg = spi_available();
            }
            return 0;

        case SPI_GET_VERSION:
            if (arg != NULL) {
                *(const char **)arg = dev_spi1_version;
                return 0;
            }
            return -EINVAL;

        case SPI_SET_CS:
            if (arg != NULL) {
                spi_set_cs(*(uint8_t *)arg);
                return 0;
            }
            return -EINVAL;
            
        default:
            return -ENOTSUP;  // Command not supported
    }
}

// SPI device instance
static const interface_t dev_spi1 = {
    .read = spi_read, 
    .write = spi_write, 
    .ioctl = spi_ioctl
};

const interface_t* dev_spi1_get(void)
{
    return (const interface_t*) &dev_spi1;
}

// DMA2 Stream3 Interrupt Handler (SPI1 TX)
void DMA2_Stream3_IRQHandler(void) {
    if (DMA2->LISR & DMA_LISR_TCIF3) {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF3;  // Clear transfer complete flag
        transfer_in_progress = 0;  // Mark as ready for next transmission
        spi_set_cs(1);  // Deactivate chip select
    }
}

// DMA2 Stream0 Interrupt Handler (SPI1 RX)
void DMA2_Stream0_IRQHandler(void) {
    // Half transfer complete
    if (DMA2->LISR & DMA_LISR_HTIF0) {
        DMA2->LIFCR |= DMA_LIFCR_CHTIF0;
        // Update received bytes count
        bytes_received += SPI1_RX_BUFFER_SIZE / 2;
    }
    
    // Transfer complete
    if (DMA2->LISR & DMA_LISR_TCIF0) {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
        // Update received bytes count
        bytes_received += SPI1_RX_BUFFER_SIZE / 2;
    }
}


// DMA2 Stream0 Interrupt Handler (SPI1 RX)
// void DMA2_Stream0_IRQHandler(void) {
//     if (DMA2->LISR & DMA_LISR_TCIF0) {
//         DMA2->LIFCR |= DMA_LIFCR_CTCIF0;  // Clear transfer complete flag
        
//         // Wait for TX to complete
//         if (!(DMA2_Stream3->CR & DMA_SxCR_EN)) {
//             transfer_in_progress = 0;
//             spi_set_cs(1);  // Deactivate chip select
//         }
//     }
// }