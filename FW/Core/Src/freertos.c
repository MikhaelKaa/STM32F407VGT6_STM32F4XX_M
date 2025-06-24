/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "retarget.h"
#include "ucmd.h"

#include "ucmd_time.h"
#include "micros.h"

#include "ili9341.h"
#include "testimg.h"

#include "fatfs.h"

#include "memory_man.h"
#include "coremark.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ILI9341 */
osThreadId_t ILI9341Handle;
const osThreadAttr_t ILI9341_attributes = {
  .name = "ILI9341",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

int ucmd_mcu_reset(int argc, char ** argv) {
  NVIC_SystemReset();
  return -1;
}


#include "errno.h"
#include "string.h"

#define ENDL "\r\n"
#define BUFFER_SIZE 128

// Глобальные объекты FatFS
static FATFS fs;         // Объект файловой системы
static DIR dir;          // Объект каталога
static FIL fil;          // Объект файла
static char path[4] = "0:/";  // Путь к SD-карте
static uint8_t is_mounted = 0; // Флаг монтирования

int ucmd_sd(int argc, char **argv) {
    switch (argc) {
        case 1:
            printf("Usage: sd <command>" ENDL);
            printf("Commands: mount, unmount, ls, rm <file>, cat <file>" ENDL);
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
            }
            break;
    }

    printf("Invalid command or arguments!" ENDL);
    printf("Usage: sd <mount|unmount|ls|rm|cat>" ENDL);
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

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ILI9341_task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}


/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  printf_init();
  ucmd_default_init();
  
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ILI9341 */
  ILI9341Handle = osThreadNew(ILI9341_task, NULL, &ILI9341_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  
  /* Infinite loop */
  for(;;)
  {
    // CLI
    ucmd_default_proc();
    
    // LED
    static uint32_t led_toggle = 0;
    if(led_toggle++%32 == 0) HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

    // printf out
    // TODO: может так получиться, что osDelay(10); это мало, вывод принтф будет битый - надо переделывать логику работы.
    printf_flush();

    // 
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ILI9341_task */
/**
* @brief Function implementing the ILI9341 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ILI9341_task */
void ILI9341_task(void *argument)
{
  /* USER CODE BEGIN ILI9341_task */
  
  HAL_GPIO_WritePin(ILI9341_BLK_GPIO_Port,  ILI9341_BLK_Pin, GPIO_PIN_SET);
  ILI9341_Init();
  osDelay(10);
  // ILI9341_DrawImage(0, 0, 240, 320, (const uint16_t*)Simpsons_style_on_a_BMX);

  /* Infinite loop */
  for(;;)
  {
    extern RTC_TimeTypeDef time;
    ucmd_time(2, (char*[]){"time", "u"});
    static char time_buf[20];
    sprintf(time_buf, "%02u:%02u:%02u", time.Hours, time.Minutes, time.Seconds);
    ILI9341_WriteString(0, 0, time_buf, Font_11x18, 0xffff, 0x000c);

    osDelay(200);
  }
  /* USER CODE END ILI9341_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

