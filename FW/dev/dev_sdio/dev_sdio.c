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
int sdio_send_command(uint32_t cmd, uint32_t arg);
static int sdio_initialize_card(void);
static int sdio_set_bus_width(uint32_t width);







static int sdio_init(void) {
    // ****************************************** GPIO ************************
    // Инициализация аппаратной части SDIO
        // 1. Инициализация тактирования
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
    RCC->APB2ENR |= RCC_APB2ENR_SDIOEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    
    // 2. Инициализация GPIO
    // PC8 - SDIO_D0
    // PC9 - SDIO_D1  
    // PC10 - SDIO_D2
    // PC11 - SDIO_D3
    // PC12 - SDIO_CK
    // PD2 - SDIO_CMD
    
    // Настройка порта C
    GPIOC->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9 | 
                      GPIO_MODER_MODER10 | GPIO_MODER_MODER11 | 
                      GPIO_MODER_MODER12);
    GPIOC->MODER |= (2U << GPIO_MODER_MODER8_Pos) |  // Alternate function
                    (2U << GPIO_MODER_MODER9_Pos) |
                    (2U << GPIO_MODER_MODER10_Pos) |
                    (2U << GPIO_MODER_MODER11_Pos) |
                    (2U << GPIO_MODER_MODER12_Pos);
    
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8 |  // Very high speed
                      GPIO_OSPEEDER_OSPEEDR9 |
                      GPIO_OSPEEDER_OSPEEDR10 |
                      GPIO_OSPEEDER_OSPEEDR11 |
                      GPIO_OSPEEDER_OSPEEDR12;
    
    GPIOC->AFR[1] |= (12U << GPIO_AFRH_AFSEL8_Pos) |  // AF12 для SDIO
                     (12U << GPIO_AFRH_AFSEL9_Pos) |
                     (12U << GPIO_AFRH_AFSEL10_Pos) |
                     (12U << GPIO_AFRH_AFSEL11_Pos) |
                     (12U << GPIO_AFRH_AFSEL12_Pos);

    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9 |
                      GPIO_PUPDR_PUPDR10 | GPIO_PUPDR_PUPDR11 |
                      GPIO_PUPDR_PUPDR12);
    GPIOC->PUPDR |= (1U << GPIO_PUPDR_PUPD8_Pos) |  // Pull-up
                    (1U << GPIO_PUPDR_PUPD9_Pos) |
                    (1U << GPIO_PUPDR_PUPD10_Pos) |
                    (1U << GPIO_PUPDR_PUPD11_Pos) |
                    (1U << GPIO_PUPDR_PUPD12_Pos);

    // Настройка порта D для CMD (PD2)
    GPIOD->MODER &= ~GPIO_MODER_MODER2;
    GPIOD->MODER |= (2U << GPIO_MODER_MODER2_Pos);  // Alternate function
    
    GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2;  // Very high speed
    
    GPIOD->AFR[0] |= (12U << GPIO_AFRL_AFSEL2_Pos);  // AF12 для SDIO
    GPIOD->PUPDR &= ~GPIO_PUPDR_PUPDR2;
    GPIOD->PUPDR |= (1U << GPIO_PUPDR_PUPD2_Pos);  // Pull-up


    // ****************************************** DMA *************************
    #ifdef SDIO_USE_DMA
    // Остановка DMA потоков
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    DMA2_Stream6->CR &= ~DMA_SxCR_EN;
    
    // Ожидание остановки потоков
    while(DMA2_Stream3->CR & DMA_SxCR_EN);
    while(DMA2_Stream6->CR & DMA_SxCR_EN);
    
    // Сброс флагов прерываний
    DMA2->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | 
                  DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | 
                  DMA_LIFCR_CFEIF3;
    DMA2->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | 
                  DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | 
                  DMA_HIFCR_CFEIF6;
    
    // Настройка DMA2 Stream3 (RX) - канал 4
    DMA2_Stream3->CR = 0;
    DMA2_Stream3->CR |= (4U << DMA_SxCR_CHSEL_Pos) |  // Channel 4
                       DMA_SxCR_PL |                  // High priority
                       DMA_SxCR_MSIZE_1 |             // 32-bit memory
                       DMA_SxCR_PSIZE_1 |             // 32-bit peripheral
                       DMA_SxCR_MINC |                // Memory increment
                       DMA_SxCR_PFCTRL |              // Peripheral flow control
                       DMA_SxCR_DBM |                 // Double buffer mode
                       DMA_SxCR_CIRC;                 // Circular mode
    
    DMA2_Stream3->FCR |= DMA_SxFCR_DMDIS |           // Direct mode disabled
                        DMA_SxFCR_FTH;               // Full FIFO threshold
    
    // Настройка DMA2 Stream6 (TX) - канал 4
    DMA2_Stream6->CR = 0;
    DMA2_Stream6->CR |= (4U << DMA_SxCR_CHSEL_Pos) |  // Channel 4
                       DMA_SxCR_PL |                  // High priority
                       DMA_SxCR_MSIZE_1 |             // 32-bit memory
                       DMA_SxCR_PSIZE_1 |             // 32-bit peripheral
                       DMA_SxCR_MINC |                // Memory increment
                       DMA_SxCR_DIR_0 |               // Memory to peripheral
                       DMA_SxCR_PFCTRL |              // Peripheral flow control
                       DMA_SxCR_DBM |                 // Double buffer mode
                       DMA_SxCR_CIRC;                 // Circular mode
    
    DMA2_Stream6->FCR |= DMA_SxFCR_DMDIS |           // Direct mode disabled
                        DMA_SxFCR_FTH;               // Full FIFO threshold
    
    // Включаем прерывания DMA
    DMA2_Stream3->CR |= DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    DMA2_Stream6->CR |= DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
    #endif // SDIO_USE_DMA        


    // ****************************************** IRQ *************************
    // Приоритеты прерываний
    NVIC_SetPriority(SDIO_IRQn, 7);      // Низкий приоритет для SDIO
    #ifdef SDIO_USE_DMA
    NVIC_SetPriority(DMA2_Stream3_IRQn, 0); // Высокий приоритет для DMA RX
    NVIC_SetPriority(DMA2_Stream6_IRQn, 0); // Высокий приоритет для DMA TX
    #endif // SDIO_USE_DMA 
    
    // Включаем прерывания
    NVIC_EnableIRQ(SDIO_IRQn);
    #ifdef SDIO_USE_DMA
    NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    NVIC_EnableIRQ(DMA2_Stream6_IRQn);
    #endif // SDIO_USE_DMA 

    // ****************************************** SDIO ************************
    // Сброс SDIO
    RCC->APB2RSTR |= RCC_APB2RSTR_SDIORST;
    for(volatile int i = 0; i < 1000; i++); // Задержка
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SDIORST;
    for(volatile int i = 0; i < 1000; i++); // Задержка после сброса
    
    // Настройка тактирования SDIO
    // Частота SDIO_CK = 48MHz / (CLKDIV + 2)
    // Для инициализации карты нужно < 400kHz, поэтому CLKDIV = 118 (48MHz/120 = 400kHz)
    SDIO->CLKCR = 0;
    SDIO->CLKCR |= (118U << SDIO_CLKCR_CLKDIV_Pos) |  // Divider для 400kHz
                  SDIO_CLKCR_CLKEN |                  // Включить тактирование
                  SDIO_CLKCR_PWRSAV;                  // Режим энергосбережения
    
    // Настройка управления питанием
    // ВНИМАНИЕ: Нужно установить оба бита PWRCTRL в '11' (0x3)
    SDIO->POWER = 0;
    for(volatile int i = 0; i < 1000; i++); // Задержка
    SDIO->POWER = SDIO_POWER_PWRCTRL;  // Устанавливаем ВСЕ биты PWRCTRL (0x3)
    
    // Ожидание включения питания
    // Ждем, пока биты PWRCTRL установятся в 11
    uint32_t timeout = 1000000;
    while(!(SDIO->POWER & SDIO_POWER_PWRCTRL) && timeout--) {
        // Пустой цикл ожидания
    }
    
    // Если таймаут - ошибка
    if(timeout == 0) {
        // Можно добавить обработку ошибки
        return -ETIMEDOUT;
    }
    
    // Очистка всех флагов
    SDIO->ICR = 0xFFFFFFFF;
    
    // Включаем прерывания SDIO (только основные для начала)
    SDIO->MASK = 0;
    SDIO->MASK |= SDIO_MASK_CCRCFAILIE |  // Ошибка CRC команды
                 SDIO_MASK_DCRCFAILIE |  // Ошибка CRC данных
                 SDIO_MASK_CTIMEOUTIE |  // Таймаут команды
                 SDIO_MASK_DTIMEOUTIE |  // Таймаут данных
                 SDIO_MASK_TXUNDERRIE |  // Underrun ошибка передачи
                 SDIO_MASK_RXOVERRIE |   // Overrun ошибка приема
                 SDIO_MASK_CMDRENDIE |   // Команда выполнена
                 SDIO_MASK_CMDSENTIE |   // Команда отправлена
                 SDIO_MASK_DATAENDIE;    // Конец передачи данных
    
    // Настройка таймаутов
    SDIO->DTIMER = 0xFFFFFFFF;  // Таймаут данных
    SDIO->DLEN = 0;            // Длина данных
    
    // Настройка DCTRL регистра (пока отключен)
    SDIO->DCTRL = 0;
    
    return 0;
}

// Close SDIO (interface implementation)
static int sdio_deinit(void) {
    // Disable SDIO
    SDIO->POWER = 0;
    SDIO->CLKCR = 0;
    
    // Disable interrupts
    NVIC_DisableIRQ(SDIO_IRQn);
    NVIC_DisableIRQ(DMA2_Stream3_IRQn);
    NVIC_DisableIRQ(DMA2_Stream6_IRQn);
    SDIO->MASK = 0;
    
    // Deinitialize DMA
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    DMA2_Stream6->CR &= ~DMA_SxCR_EN;
    
    // Disable clocks
    RCC->APB2ENR &= ~RCC_APB2ENR_SDIOEN;
    RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2EN;
    
    return 0;
}

// Wait for response
static int sdio_wait_response(uint32_t mask, uint32_t value, uint32_t timeout) {
    while (timeout--) {
        uint32_t status = SDIO->STA;
        
        // Если произошла ошибка
        if (status & (SDIO_STA_CTIMEOUT | SDIO_STA_CCRCFAIL)) {
            SDIO->ICR = status & (SDIO_STA_CTIMEOUT | SDIO_STA_CCRCFAIL);
            return -EIO;
        }
        
        // Если команда завершилась успешно
        if (status & mask) {
            // Проверяем, что установился ожидаемый флаг
            if ((status & mask) == value) {
                SDIO->ICR = status & mask;
                return 0;
            }
            // Если установился не тот флаг, который ожидали
            return -EIO;
        }
    }
    return -ETIMEDOUT;
}

// Send command to SD card
int sdio_send_command(uint32_t cmd, uint32_t arg) {
    // Очищаем флаги перед отправкой команды
    SDIO->ICR = SDIO_ICR_CMDRENDC | SDIO_ICR_CMDSENTC | 
                SDIO_ICR_CTIMEOUTC | SDIO_ICR_CCRCFAILC;
    
    SDIO->ARG = arg;
    
    // Формируем команду: CPSMEN + номер команды
    SDIO->CMD = SDIO_CMD_CPSMEN | cmd;
    
    // Для разных команд разные ожидаемые ответы:
    // - CMD0, CMD55: ожидаем CMDSENT (команда отправлена, нет ответа)
    // - Остальные: ожидаем CMDREND (получен ответ)
    
    uint32_t expected_response;
    if (cmd == 0U || cmd == 55U) {
        expected_response = SDIO_STA_CMDSENT;
    } else {
        expected_response = SDIO_STA_CMDREND;
    }
    
    return sdio_wait_response(expected_response, expected_response, SDIO_TIMEOUT);
}

// Initialize SD card
static int sdio_initialize_card(void) {
    int ret;
    
    // Даем карте время на инициализацию
    for(volatile int i = 0; i < 10000; i++);
    
    // CMD0 - GO_IDLE_STATE
    ret = sdio_send_command((SDIO_CMD_CPSMEN | 0), 0);
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
    for (size_t i = 0U; i < count / 4U; i++) {
        uint32_t data = buffer[i * 4U] | 
                       (buffer[i * 4U + 1U] << 8U) |
                       (buffer[i * 4U + 2U] << 16U) |
                       (buffer[i * 4U + 3U] << 24U);
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

void DMA2_Stream3_IRQHandler(void) {
    // Обработка прерываний DMA RX
    uint32_t status = DMA2->LISR;
    
    if(status & DMA_LISR_TCIF3) {
        // Transfer complete
        DMA2->LIFCR |= DMA_LIFCR_CTCIF3;
        // Ваш код обработки завершения приема
    }
    
    if(status & DMA_LISR_TEIF3) {
        // Transfer error
        DMA2->LIFCR |= DMA_LIFCR_CTEIF3;
        // Обработка ошибки
    }
}

void DMA2_Stream6_IRQHandler(void) {
    // Обработка прерываний DMA TX
    uint32_t status = DMA2->HISR;
    
    if(status & DMA_HISR_TCIF6) {
        // Transfer complete
        DMA2->HIFCR |= DMA_HIFCR_CTCIF6;
        // Ваш код обработки завершения передачи
    }
    
    if(status & DMA_HISR_TEIF6) {
        // Transfer error
        DMA2->HIFCR |= DMA_HIFCR_CTEIF6;
        // Обработка ошибки
    }
}