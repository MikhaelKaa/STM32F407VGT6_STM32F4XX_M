/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "rng.h"
#include "rtc.h"
#include "sdio.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "micros.h"
#include "retarget.h"

#include "ucmd.h"
#include "ucmd_time.h"
#include "memory_man.h"
#include "coremark.h"

#include "fatfs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// #ifndef VERSION
// #define VERSION "Dev build 0.00"
// const unsigned char build_version[] = VERSION " " __DATE__ " "__TIME__;
// #endif /* VERSION */ 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define PAGE_SIZE 4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int ucmd_mcu_reset(int argc, char ** argv) {
  NVIC_SystemReset();
  return -1;
}

#include "errno.h"
#include "fatfs.h"
#include "string.h"

#define ENDL "\r\n"
#define BUFFER_SIZE 128

// Глобальные объекты FatFS
static FATFS fs;
static DIR dir;
static FIL fil;
static char path[4] = "0:/";
static uint8_t is_mounted = 0;

int ucmd_sd(int argc, char **argv) {
    switch (argc) {
        case 1:
            printf("Usage: sd <command>" ENDL);
            printf("Commands: mount, unmount, ls, rm <file>, cat <file>, touch <file>, mkdir <dir>" ENDL);
            return -EINVAL;
        
        case 2:
            if (strcmp(argv[1], "mount") == 0) {
                // Монтирование SD-карты
                FRESULT res = f_mount(&fs, path, 1);
                if (res != FR_OK) {
                    printf("Mount error: %d" ENDL, res);
                    return -EIO;
                }
                is_mounted = 1;
                printf("SD mounted" ENDL);
                return 0;
            
            } else if (strcmp(argv[1], "unmount") == 0) {
                // Демонтирование SD-карты
                if (!is_mounted) {
                    printf("Not mounted!" ENDL);
                    return -EIO;
                }
                f_mount(NULL, path, 0);
                is_mounted = 0;
                printf("SD unmounted" ENDL);
                return 0;
            
            } else if (strcmp(argv[1], "ls") == 0) {
                // Вывод содержимого каталога
                if (!is_mounted) {
                    printf("Mount first!" ENDL);
                    return -EIO;
                }
                
                FRESULT res;
                FILINFO fno;
                
                res = f_opendir(&dir, path);
                if (res != FR_OK) {
                    printf("Open dir error: %d" ENDL, res);
                    return -EIO;
                }
                
                printf("Directory listing:" ENDL);
                for (;;) {
                    res = f_readdir(&dir, &fno);
                    if (res != FR_OK || fno.fname[0] == 0) break;
                    
                    if (fno.fattrib & AM_DIR)
                        printf("  [DIR]  %s" ENDL, fno.fname);
                    else
                        printf("  [FILE] %s (%lu bytes)" ENDL, fno.fname, fno.fsize);
                }
                f_closedir(&dir);
                return 0;
            }
            break;
        
        case 3:
            if (strcmp(argv[1], "rm") == 0) {
                // Удаление файла
                if (!is_mounted) {
                    printf("Mount first!" ENDL);
                    return -EIO;
                }
                
                FRESULT res = f_unlink(argv[2]);
                if (res != FR_OK) {
                    printf("Delete error: %d" ENDL, res);
                    return -EIO;
                }
                printf("File '%s' deleted" ENDL, argv[2]);
                return 0;
            
            } else if (strcmp(argv[1], "cat") == 0) {
                // Вывод содержимого файла
                if (!is_mounted) {
                    printf("Mount first!" ENDL);
                    return -EIO;
                }
                
                FRESULT res = f_open(&fil, argv[2], FA_READ);
                if (res != FR_OK) {
                    printf("Open error: %d" ENDL, res);
                    return -EIO;
                }
                
                UINT bytes_read;
                char buffer[BUFFER_SIZE];
                printf("File content:" ENDL);
                do {
                    res = f_read(&fil, buffer, BUFFER_SIZE - 1, &bytes_read);
                    if (res != FR_OK) {
                        f_close(&fil);
                        printf("Read error: %d" ENDL, res);
                        return -EIO;
                    }
                    buffer[bytes_read] = 0; // NULL-terminator
                    printf("%s", buffer);
                } while (bytes_read == BUFFER_SIZE - 1);
                
                f_close(&fil);
                printf(ENDL);
                return 0;
                
            } else if (strcmp(argv[1], "touch") == 0) {
                // Создание пустого файла
                if (!is_mounted) {
                    printf("Mount first!" ENDL);
                    return -EIO;
                }
                
                FRESULT res = f_open(&fil, argv[2], FA_CREATE_ALWAYS | FA_WRITE);
                if (res != FR_OK) {
                    printf("Create error: %d" ENDL, res);
                    return -EIO;
                }
                
                // Важно: закрываем файл сразу после создания
                f_close(&fil);
                printf("File created: %s" ENDL, argv[2]);
                return 0;
                
            } else if (strcmp(argv[1], "mkdir") == 0) {
                // Создание директории
                if (!is_mounted) {
                    printf("Mount first!" ENDL);
                    return -EIO;
                }
                
                FRESULT res = f_mkdir(argv[2]);
                if (res != FR_OK) {
                    printf("Mkdir error: %d" ENDL, res);
                    return -EIO;
                }
                
                printf("Directory created: %s" ENDL, argv[2]);
                return 0;
            }
            break;
    }

    printf("Invalid command or arguments!" ENDL);
    printf("Usage: sd <mount|unmount|ls|rm|cat|touch|mkdir>" ENDL);
    return -EINVAL;
}

#undef ENDL


// define command list
command_t cmd_list[] = {
  {
    .cmd  = "help",
    .help = "print available commands with their help text",
    .fn   = print_help_cb,
  },

  {
    .cmd  = "reset",
    .help = "reset mcu",
    .fn   = ucmd_mcu_reset,
  }, 

  {
    .cmd  = "mem",
    .help = "memory man, use mem help",
    .fn   = ucmd_mem,
  },

  {
    .cmd  = "time",
    .help = "rtc time. to set type time hh mm ss",
    .fn   = ucmd_time,
  },

  {
    .cmd  = "coremark",
    .help = "coremark",
    .fn   = coremark,
  },

  {
    .cmd  = "sd",
    .help = "sd card test utils",
    .fn   = ucmd_sd,
  },
  
  
  {}, // null list terminator DON'T FORGET THIS!
};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  // if(GPIO_Pin == GPIO_PIN_3) {
  //   osSemaphoreRelease(mpu6050_irqHandle);
  // }
  if(GPIO_Pin == NRF24L01_IRQ_Pin) {
    // NRF24L01_EXTI_IRQHandler(&nrf24L01Config);
    // HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_SDIO_SD_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_RNG_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_I2C3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  us_init();
  printf_init();
  ucmd_default_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static int led_cnt = 0;
    if(led_cnt++ % 512 == 0) HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    ucmd_default_proc();
    printf_flush();
    delay_us(800);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
