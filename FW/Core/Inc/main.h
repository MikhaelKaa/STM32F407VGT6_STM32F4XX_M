/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

// #define ILI9341_Delay_us osDelay
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void dump_hex(uint8_t *buf, uint32_t len);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOA
#define ILI9341_DC_Pin GPIO_PIN_5
#define ILI9341_DC_GPIO_Port GPIOC
#define ILI9341_BLK_Pin GPIO_PIN_1
#define ILI9341_BLK_GPIO_Port GPIOB
#define ILI9341_CS_Pin GPIO_PIN_12
#define ILI9341_CS_GPIO_Port GPIOB
#define ILI9341_SCK_Pin GPIO_PIN_13
#define ILI9341_SCK_GPIO_Port GPIOB
#define ILI9341_RES_Pin GPIO_PIN_14
#define ILI9341_RES_GPIO_Port GPIOB
#define ILI9341_MOSI_Pin GPIO_PIN_15
#define ILI9341_MOSI_GPIO_Port GPIOB
#define W25Q16_CS_Pin GPIO_PIN_15
#define W25Q16_CS_GPIO_Port GPIOA
#define SDIO_detect_dummy_Pin GPIO_PIN_0
#define SDIO_detect_dummy_GPIO_Port GPIOD
#define SPI1_SCK_Pin GPIO_PIN_3
#define SPI1_SCK_GPIO_Port GPIOB
#define SPI1_MISO_Pin GPIO_PIN_4
#define SPI1_MISO_GPIO_Port GPIOB
#define SPI1_MOSI_Pin GPIO_PIN_5
#define SPI1_MOSI_GPIO_Port GPIOB
#define NRF24L01_NSS_Pin GPIO_PIN_6
#define NRF24L01_NSS_GPIO_Port GPIOB
#define NRF24L01_CE_Pin GPIO_PIN_7
#define NRF24L01_CE_GPIO_Port GPIOB
#define NRF24L01_IRQ_Pin GPIO_PIN_0
#define NRF24L01_IRQ_GPIO_Port GPIOE
#define NRF24L01_IRQ_EXTI_IRQn EXTI0_IRQn

/* USER CODE BEGIN Private defines */
// Директива для библиотеки экрана.
#define ILI9341_USE_SPI

#define DEBUG_W25
// #define USART_TX_DMA_BUF_SIZE (32768)

#define DEBUG
#ifdef DEBUG
#define DBG(...)    printf(__VA_ARGS__); printf_flush();HAL_Delay(800);
#else
#define DBG(...)
#endif

// #define W25Q_CS_GPIO_Port  (void*) 
// #define W25Q_CS_Pin    (void)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
