// DWT (Data Watchpoint and Trace) delay utilities for STM32F407
// Michael Kaa
// 03.11.2025

#ifndef DWT_DELAY_H
#define DWT_DELAY_H

#include "core_cm4.h"

#ifndef SystemCoreClock
#define SystemCoreClock (168000000U)
#endif

// Инициализация DWT счётчика
static inline void dwt_delay_init(void)
{
    // Включаем тактирование DWT (требуется для доступа к счётчику циклов)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    
    // Сбрасываем счётчик и включаем его
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

// Задержка в микросекундах
static inline void dwt_delay_us(uint32_t us) 
{
    uint32_t start_tick = DWT->CYCCNT;
    // Конвертируем микросекунды в такты процессора
    uint32_t us_count_ticks = us * (SystemCoreClock / 1000000U);
    
    // Ожидание с учётом переполнения 32-битного счётчика
    while ((DWT->CYCCNT - start_tick) < us_count_ticks) {
        // Пустая инструкция
    }
}

// Задержка в миллисекундах
static inline void dwt_delay_ms(uint32_t ms) 
{
    for (uint32_t i = 0; i < ms; i++) {
        dwt_delay_us(1000);
    }
}

// Получить время в микросекундах с момента инициализации
static inline uint32_t dwt_micros(void) {
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

// Получить время в миллисекундах с момента инициализации
static inline uint32_t dwt_millis(void) {
    return DWT->CYCCNT / (SystemCoreClock / 1000U);
}

// Проверка инициализации DWT
static inline uint32_t dwt_is_ready(void) {
    return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) ? 1 : 0;
}

#endif // DWT_DELAY_H