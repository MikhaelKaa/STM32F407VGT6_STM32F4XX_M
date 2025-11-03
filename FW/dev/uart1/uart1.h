// Header: POSIX-style UART implementation for STM32F407 CMSIS
// File Name: uart1.h
// Author: Михаил Каа
// Date: 04.11.2025

#ifndef UART_POSIX_H
#define UART_POSIX_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <errno.h>

#if defined(STM32F407xx)
#include "stm32f407xx.h"
#include "core_cm4.h"
#endif

// Buffer sizes
#define UART_TX_BUFFER_SIZE (4096U)
#define UART_RX_BUFFER_SIZE (1024U)

// Error codes
#define UART_SUCCESS        0
#define UART_ERROR         -1
#define UART_EBUSY         -2
#define UART_EINVAL        -3
#define UART_ETIMEOUT      -4

// File descriptor for UART
#define UART_FD 1

// Function prototypes (POSIX-like interface)
int uart_open(void);
int uart_close(void);
ssize_t uart_write(const void *buf, size_t count);
ssize_t uart_read(void *buf, size_t count);
int uart_available(void);
int uart_flush(void);


#endif // UART_POSIX_H