// File Name: uart1.c
// Implementation of POSIX-style UART interface

#include "uart1.h"

// Static buffers
static          uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
static volatile uint8_t rx_buffer[UART_RX_BUFFER_SIZE];

// Ring buffer pointers for RX
static volatile uint32_t rx_read_pos = 0;
static volatile uint32_t rx_write_pos = 0;

// DMA transfer state
static volatile uint8_t tx_in_progress = 0;
static volatile uint32_t tx_complete_flag = 0;

// Initialize UART peripheral
void uart_init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    
    // Configure GPIO for USART1 (PA9 - TX, PA10 - RX)
    GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10);
    GPIOA->MODER |= (2 << GPIO_MODER_MODER9_Pos) | (2 << GPIO_MODER_MODER10_Pos);
    
    // Alternate function AF7 for USART1
    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10);
    GPIOA->AFR[1] |= (7 << (4 * 1)) | (7 << (4 * 2));
    
    // High speed
    GPIOA->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED9_Pos) | (3 << GPIO_OSPEEDR_OSPEED10_Pos);
    
    // Configure USART1 - 115200 baud at 84MHz
    USART1->BRR = (84000000 + 115200 / 2) / 115200;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;
    USART1->CR1 |= USART_CR1_UE;
    
    // Configure DMA for transmission (USART1_TX -> DMA2 Stream7)
    DMA2_Stream7->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream7->CR & DMA_SxCR_EN);
    
    DMA2_Stream7->PAR = (uint32_t)&USART1->DR;
    DMA2_Stream7->M0AR = (uint32_t)tx_buffer;
    DMA2_Stream7->NDTR = 0;
    
    DMA2_Stream7->CR = (4 << DMA_SxCR_CHSEL_Pos) |  // Channel 4
                       DMA_SxCR_MINC |              // Memory increment
                       DMA_SxCR_DIR_0 |             // Memory to peripheral
                       DMA_SxCR_TCIE;               // Transfer complete interrupt
    
    // Configure DMA for reception (USART1_RX -> DMA2 Stream5) - Circular mode
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream5->CR & DMA_SxCR_EN);
    
    DMA2_Stream5->PAR = (uint32_t)&USART1->DR;
    DMA2_Stream5->M0AR = (uint32_t)rx_buffer;
    DMA2_Stream5->NDTR = UART_RX_BUFFER_SIZE;
    
    DMA2_Stream5->CR = (4 << DMA_SxCR_CHSEL_Pos) |  // Channel 4
                       DMA_SxCR_MINC |              // Memory increment
                       DMA_SxCR_CIRC |              // Circular mode
                       DMA_SxCR_HTIE |              // Half transfer interrupt
                       DMA_SxCR_TCIE;               // Transfer complete interrupt
    
    // Enable DMA reception
    DMA2_Stream5->CR |= DMA_SxCR_EN;
    
    // Enable interrupts
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    
    // Clear buffers
    memset((void *)tx_buffer, 0, UART_TX_BUFFER_SIZE);
    memset((void *)rx_buffer, 0, UART_RX_BUFFER_SIZE);
}

// Open UART (POSIX-like)
int uart_open(void) {
    uart_init();
    return UART_FD;  // Return file descriptor
}

// Close UART (POSIX-like)
int uart_close(void) {
    // Disable UART and DMA
    USART1->CR1 &= ~USART_CR1_UE;
    DMA2_Stream7->CR &= ~DMA_SxCR_EN;
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    
    // Disable interrupts
    NVIC_DisableIRQ(USART1_IRQn);
    NVIC_DisableIRQ(DMA2_Stream7_IRQn);
    NVIC_DisableIRQ(DMA2_Stream5_IRQn);
    
    return UART_SUCCESS;
}

// Write data to UART (POSIX-like)
ssize_t uart_write(const void *buf, size_t count) {
    if (buf == NULL || count == 0) {
        return UART_EINVAL;
    }
    
    if (count > UART_TX_BUFFER_SIZE) {
        count = UART_TX_BUFFER_SIZE;
    }
    
    // Wait for previous transmission to complete
    uint32_t timeout = 1000000;  // Timeout counter
    while (tx_in_progress && timeout--) {
        __asm__("nop");
    }
    
    if (timeout == 0) {
        return UART_ETIMEOUT;
    }
    
    // Copy data to buffer
    memcpy(tx_buffer, buf, count);
    tx_in_progress = 1;
    tx_complete_flag = 0;
    
    // Configure and start DMA transfer
    DMA2_Stream7->M0AR = (uint32_t)tx_buffer;
    DMA2_Stream7->NDTR = count;
    DMA2_Stream7->CR |= DMA_SxCR_EN;
    
    // Wait for transmission to complete
    timeout = 1000000;
    while (!tx_complete_flag && timeout--) {
        __asm__("nop");
    }
    
    if (timeout == 0) {
        return UART_ETIMEOUT;
    }
    
    return (ssize_t)count;
}

// Read data from UART (POSIX-like)
ssize_t uart_read(void *buf, size_t count) {
    if (buf == NULL) {
        return UART_EINVAL;
    }
    
    uint8_t *buffer = (uint8_t *)buf;
    size_t bytes_read = 0;
    
    // Calculate available bytes in ring buffer
    uint32_t available = uart_available();
    if (available == 0) {
        return 0;  // No data available
    }
    
    if (count > available) {
        count = available;
    }
    
    // Read data from ring buffer
    for (size_t i = 0; i < count; i++) {
        buffer[i] = rx_buffer[rx_read_pos];
        rx_read_pos = (rx_read_pos + 1) % UART_RX_BUFFER_SIZE;
        bytes_read++;
    }
    
    return (ssize_t)bytes_read;
}

// Check how many bytes are available to read
int uart_available(void) {
    uint32_t current_ndtr = DMA2_Stream5->NDTR;
    uint32_t current_write_pos = (UART_RX_BUFFER_SIZE - current_ndtr) % UART_RX_BUFFER_SIZE;
    
    if (current_write_pos >= rx_read_pos) {
        return current_write_pos - rx_read_pos;
    } else {
        return (UART_RX_BUFFER_SIZE - rx_read_pos) + current_write_pos;
    }
}

// Flush RX buffer
int uart_flush(void) {
    rx_read_pos = (UART_RX_BUFFER_SIZE - DMA2_Stream5->NDTR) % UART_RX_BUFFER_SIZE;
    return UART_SUCCESS;
}

// USART1 Interrupt Handler
void USART1_IRQHandler(void) {
    // RXNE interrupt - data received
    if (USART1->SR & USART_SR_RXNE) {
        volatile uint8_t data = USART1->DR;  // Read to clear flag
        (void)data;  // Suppress unused warning
        // Data is handled by DMA, this is just for flag clearing
    }
}

// DMA2 Stream7 Interrupt Handler (Transmission)
void DMA2_Stream7_IRQHandler(void) {
    if (DMA2->HISR & DMA_HISR_TCIF7) {
        DMA2->HIFCR |= DMA_HIFCR_CTCIF7;  // Clear transfer complete flag
        tx_in_progress = 0;
        tx_complete_flag = 1;
    }
}

// DMA2 Stream5 Interrupt Handler (Reception)
void DMA2_Stream5_IRQHandler(void) {
    // Half transfer complete
    if (DMA2->HISR & DMA_HISR_HTIF5) {
        DMA2->HIFCR |= DMA_HIFCR_CHTIF5;
        // Optional: handle half buffer event
    }
    
    // Transfer complete
    if (DMA2->HISR & DMA_HISR_TCIF5) {
        DMA2->HIFCR |= DMA_HIFCR_CTCIF5;
        // Optional: handle full buffer event
    }
}

// printf implementation using UART
int uart_printf(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len > 0) {
        uart_write(buffer, len);
    }
    
    return len;
}

// scanf implementation using UART (simplified)
int uart_scanf(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    
    // Wait for data with timeout
    uint32_t timeout = 1000000;
    while (uart_available() == 0 && timeout--) {
        __asm__("nop");
    }
    
    if (timeout == 0) {
        va_end(args);
        return UART_ETIMEOUT;
    }
    
    // Read available data
    ssize_t bytes_read = uart_read(buffer, sizeof(buffer) - 1);
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        int result = vsscanf(buffer, format, args);
        va_end(args);
        return result;
    }
    
    va_end(args);
    return 0;
}