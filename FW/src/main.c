
#include "stm32f407xx.h"

// Простая функция задержки
void delay(void) {
    for (volatile uint32_t i = 0; i < 500000; i++);
}

int main(void) {
    // Включаем тактирование порта A
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    
    // Настраиваем PA1 как выход
    GPIOA->MODER &= ~GPIO_MODER_MODER1;  // Сбрасываем биты
    GPIOA->MODER |= GPIO_MODER_MODER1_0; // Output mode (01)
    
    // Настраиваем тип выхода: push-pull (0 по умолчанию)
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT1;
    
    // Настраиваем скорость: medium speed (10)
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1;
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED1_0;
    
    // Настраиваем pull-up/pull-down: no pull (00)
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;
    
    while (1) {
        // Включаем светодиод (SET)
        GPIOA->BSRR = GPIO_BSRR_BS1;
        delay();
        
        // Выключаем светодиод (RESET)  
        GPIOA->BSRR = GPIO_BSRR_BR1;
        delay();
    }
}

int main(void)
{

  while (1)
  {
    for(volatile int i = 0; i < 1000;) {
      i++;
    }
  }

}

void SystemInit (void)
{
  //SystemCoreClock = SYSTEM_CLOCK;
}