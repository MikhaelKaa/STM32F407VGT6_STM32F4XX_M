// Green led on board stm32f407
// Michael Kaa
// 03.11.2025


#include <errno.h>
#include "stm32f407xx.h"
#include "green_led.h"


int green_led_init(void) {
  // Включаем тактирование порта A
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  // Настраиваем PA1 как выход
  GPIOA->MODER &= ~GPIO_MODER_MODER1;  // Сбрасываем биты
  GPIOA->MODER |= GPIO_MODER_MODER1_0; // Output mode (01)
  
  // Настраиваем тип выхода: OPEN-DRAIN
  GPIOA->OTYPER |= GPIO_OTYPER_OT1;
  
  // Настраиваем скорость: medium speed (10)
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1;
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED1_0;
  
  // Настраиваем pull-up/pull-down: используем pull-up для open-drain
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1; // Сначала сбрасываем
  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_0; // Pull-up (01)
  
  // Изначально выключаем светодиод (устанавливаем высокий уровень)
  GPIOA->BSRR = GPIO_BSRR_BS1;
  
  return 0;
}

int green_led_set(led_state_t state)
{
  int retval = 0;

  switch (state)
  {
    case led_off:
      // Выключаем светодиод - устанавливаем высокий уровень (open-drain: отключаем)
      GPIOA->BSRR = GPIO_BSRR_BS1;
      break;
    
    case led_on:
      // Включаем светодиод - устанавливаем низкий уровень (open-drain: включаем)
      GPIOA->BSRR = GPIO_BSRR_BR1;
      break;
  
    default:
      retval = EINVAL;
      break;
  }

  return retval;
}