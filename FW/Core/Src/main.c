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

#include "mpu6050.h"

#include "i2c_tools.h"

#include "ili9341.h"

#include "bmp.h"

#include "ucmd_time_HAL.h"

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

#include <errno.h>
#include <string.h>
#include "fatfs.h"

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


// test code
MPU6050_t mpu;
int imu_show(void) {
  // MPU6050_Read_All(&hi2c1, &mpu);
  printf("mpu.Ax = %f\r\n", mpu.Ax);
  printf("mpu.Ay = %f\r\n", mpu.Ay);
  printf("mpu.Az = %f\r\n", mpu.Az);
  
  printf("mpu.Gx = %f\r\n", mpu.Gx);
  printf("mpu.Gy = %f\r\n", mpu.Gy);
  printf("mpu.Gz = %f\r\n", mpu.Gz);
  
  printf("mpu.KalmanAngleX = %f\r\n", mpu.KalmanAngleX);
  printf("mpu.KalmanAngleY = %f\r\n", mpu.KalmanAngleY);
  
  printf("mpu.Temperature = %f\r\n", mpu.Temperature);
  return 0;
}

int imu_show(void);

// ucmd handler for mem_dump.
int ucmd_imu(int argc, char *argv[])
{
  // static uint32_t argv2 = 0;
  // static uint32_t argv3 = 0;
  // static uint32_t argv4 = 0;
  // static uint16_t argv5 = 0;
  
  switch (argc) {
    case 1:
    printf("imu usage: imu show | irq_en | irq_dis\r\n");
    goto ok;
    break;
    case 2:
    if(strcmp(&argv[1][0], "show") == 0) {
      imu_show();
      goto ok;
    }
    if(strcmp(&argv[1][0], "init") == 0) {
      MPU6050_Init(&hi2c1);
      goto ok;
    }
    
    if(strcmp(&argv[1][0], "irq_en") == 0) {
      MPU6050_Enable_irq(&hi2c1, &mpu);
      goto ok;
    }
    if(strcmp(&argv[1][0], "irq_dis") == 0) {
      MPU6050_Diasble_irq(&hi2c1, &mpu);
      // HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      goto ok;
    }
    break;
    
    default:
    goto err;
    break;
  }
  
  goto err;
  ok: return 0;
  err: printf("imu say: ups...\r\n");
  return -1;
}

#include "terminal.h"
#include "term_gxf.h"
#define ENDL "\r\n"
int ucmd_term_test(int argc, char *argv[]) {
    if(argc < 2) {
        // Вывод справки, если нет аргументов
        printf("Usage: term <command> [args]" ENDL);
        printf("Commands:" ENDL);
        printf("  cls                  Clear screen" ENDL);
        printf("  text <string>        Print text" ENDL);
        printf("  color <fg> [bg]      Set colors" ENDL);
        printf("  cursor <row> <col>   Set cursor position" ENDL);
        printf("  ili                  Test direct char output" ENDL);
        return 0;
    }

    char *command = argv[1];
    
    if(strcmp(command, "cls") == 0) {
        // Очистка экрана
        terminal_input_data(ESC "[2J", sizeof(ESC "[2J") - 1);
    }
    else if(strcmp(command, "text") == 0 && argc >= 3) {
        // Вывод текста
        char *text = argv[2];
        terminal_input_data(text, strlen(text));
        
        // Добавляем перевод строки, если текст не заканчивается на \n
        if(text[strlen(text)-1] != '\n') {
            terminal_input_data("\n", 1);
        }
    }
    else if(strcmp(command, "color") == 0 && argc >= 3) {
        // Установка цвета
        char color_cmd[32];
        int bg = -1;
        int fg = atoi(argv[2]);
        
        if(argc >= 4) {
            bg = atoi(argv[3]);
        }
        
        if(bg >= 0) {
            snprintf(color_cmd, sizeof(color_cmd), ESC "[%d;%dm", fg, bg);
        } else {
            snprintf(color_cmd, sizeof(color_cmd), ESC "[%dm", fg);
        }
        
        terminal_input_data(color_cmd, strlen(color_cmd));
    }
    else if(strcmp(command, "cursor") == 0 && argc >= 4) {
        // Установка позиции курсора
        int row = atoi(argv[2]);
        int col = atoi(argv[3]);
        char cursor_cmd[32];
        
        snprintf(cursor_cmd, sizeof(cursor_cmd), ESC "[%d;%dH", row, col);
        terminal_input_data(cursor_cmd, strlen(cursor_cmd));
    }
    else if(strcmp(command, "ili") == 0) {
        // Прямой вывод символа
        ILI9341_Select();
        ILI9341_WriteChar(0, 0, 'U', Font_7x10, 0xffff, 0x0000);
        ILI9341_Unselect();
    }
    else {
        printf("Unknown command or missing arguments" ENDL);
        return -1;
    }

    return 0;
}

#ifdef ENDL
#undef ENDL 
#endif // ENDL


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
  
  {
    .cmd  = "imu",
    .help = "imu test code",
    .fn   = ucmd_imu,
  },
  
  {
    .cmd  = "i2c",
    .help = "i2c tool",
    .fn   = ucmd_i2c,
  },

  {
    .cmd  = "bmp",
    .help = "bmp tool",
    .fn   = ucmd_bmp,
  },

  {
    .cmd  = "term",
    .help = "term test",
    .fn   = ucmd_term_test,
  },
  
  
  {}, // null list terminator DON'T FORGET THIS!
};


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

  if(GPIO_Pin == GPIO_PIN_7) {
    MPU6050_Read_All(&hi2c1, &mpu);
  }

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


  ILI9341_Init();
  ILI9341_back_light(1);
  // ILI9341_WriteString(0, 0, "init done", Font_11x18, 0xffff, 0x0000);
  // ILI9341_WriteString(0, Font_11x18.height*1, "red", Font_11x18, ILI9341_COLOR565(0xff, 0x00, 0x00), 0x0000);
  // ILI9341_WriteString(0, Font_11x18.height*2, "green", Font_11x18, ILI9341_COLOR565(0x00, 0xff, 0x00), 0x0000);
  // ILI9341_WriteString(0, Font_11x18.height*3, "blue", Font_11x18, ILI9341_COLOR565(0x00, 0x00, 0xff), 0x0000);
  
  BMP_draw_pixel = ILI9341_DrawPixel;
  BMP_scr_width = ILI9341_WIDTH;
  BMP_scr_heigth = ILI9341_HEIGHT;
  
  if(!ucmd_sd(2, (char*[]){"sd ", "mount"})) {
    printf("SD card mount!\r\n");
    ucmd_bmp(3, (char*[]){"bmp", "load", "0:/eva.bmp"});
  }
  else{printf("SD card not mount!!!\r\n");}

  // draw_pixel = ILI9341_DrawPixel;
  ucmd_imu(2, (char*[]){"imu", "init"});
  ucmd_imu(2, (char*[]){"imu", "irq_en"});
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static int led_cnt = 0;
    if(led_cnt++ % 128 == 0) {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      time_update();
      char term_data[12];
      // set coordinats time
      snprintf(term_data, sizeof(term_data), ESC "[%d;%dH", 0, 38);
      terminal_input_data(term_data, strlen(term_data));
      // draw time
      snprintf(term_data, sizeof(term_data), "%02u:%02u:%02u", time.Hours, time.Minutes, time.Seconds);
      terminal_input_data(term_data, strlen(term_data));

      //
      // MPU6050_Read_All(&hi2c1, &mpu);
      snprintf(term_data, sizeof(term_data), ESC "[%d;%dH", 13, 20);
      terminal_input_data(term_data, strlen(term_data));
      snprintf(term_data, sizeof(term_data), "%0.2f:%0.2f", mpu.KalmanAngleX, mpu.KalmanAngleY);
      terminal_input_data(term_data, strlen(term_data));

    }
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
