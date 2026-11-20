/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "iwdg.h"
#include "tim.h"
#include "dac.h"
#include "usart.h"
#include "lwip.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
} DioPin_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern volatile uint16_t plc_adc_raw[4];
extern struct netif gnetif;

/* All 17 non-PWM GPIO DIO outputs in logical order */
static const DioPin_t kDioOutputs[] = {
  {DIO2_GPIO_Port,  DIO2_Pin},
  {DIO3_GPIO_Port,  DIO3_Pin},
  {DIO13_GPIO_Port, DIO13_Pin},
  {DIO15_GPIO_Port, DIO15_Pin},
  {DIO16_GPIO_Port, DIO16_Pin},
  {DIO17_GPIO_Port, DIO17_Pin},
  {DIO18_GPIO_Port, DIO18_Pin},
  {DIO19_GPIO_Port, DIO19_Pin},
  {DIO20_GPIO_Port, DIO20_Pin},
  {DIO21_GPIO_Port, DIO21_Pin},
  {DIO22_GPIO_Port, DIO22_Pin},
  {DIO23_GPIO_Port, DIO23_Pin},
  {DIO24_GPIO_Port, DIO24_Pin},
  {DIO25_GPIO_Port, DIO25_Pin},
  {DIO26_GPIO_Port, DIO26_Pin},
  {DIO27_GPIO_Port, DIO27_Pin},
};
#define DIO_OUTPUT_COUNT (sizeof(kDioOutputs) / sizeof(kDioOutputs[0]))
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

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
  /* init code for USB_DEVICE (disabled while testing ETH at 150 MHz / 50 MHz MCO1) */
  /* MX_USB_DEVICE_Init(); */

  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
  uint32_t step = 0;
  uart3_printf("\r\n=== F407PLC boot | IP=192.168.1.100 ===\r\n");

  /* Infinite loop: board bring-up test pattern */
  for(;;)
  {
    HAL_IWDG_Refresh(&hiwdg);

    /* 1. Walking-1 chaser across all 17 GPIO DIO outputs + all-ON/all-OFF flash */
    uint32_t phase = step % (DIO_OUTPUT_COUNT + 2);
    for (uint32_t i = 0; i < DIO_OUTPUT_COUNT; i++)
    {
      GPIO_PinState state;
      if (phase < DIO_OUTPUT_COUNT) {
        state = (i == phase) ? GPIO_PIN_SET : GPIO_PIN_RESET;
      } else if (phase == DIO_OUTPUT_COUNT) {
        state = GPIO_PIN_SET;   /* brief all-HIGH flash */
      } else {
        state = GPIO_PIN_RESET; /* brief all-LOW gap */
      }
      HAL_GPIO_WritePin(kDioOutputs[i].port, kDioOutputs[i].pin, state);
    }

    /* 2. Breathing triangle-wave PWM on all 8 PWM pins (TIM3: DIO9-12, TIM4: DIO4-7) */
    uint32_t tri = (step * 50) % 2000;
    uint32_t base_duty = (tri < 1000) ? tri : (1999 - tri); /* 0..999 */

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (base_duty + 0)   % 1000); /* DIO4 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (base_duty + 125) % 1000); /* DIO5 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (base_duty + 250) % 1000); /* DIO6 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, (base_duty + 375) % 1000); /* DIO7 */

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (base_duty + 500) % 1000); /* DIO9 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (base_duty + 625) % 1000); /* DIO10 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (base_duty + 750) % 1000); /* DIO11 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (base_duty + 875) % 1000); /* DIO12 */

    /* 3. Complementary ramp on DAC1 (PA4) & DAC2 (PA5) */
    uint32_t dac1 = (base_duty * 4095) / 999;
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac1);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095 - dac1);

    /* 4. Once per second, print heartbeat + ADC + ETH link status over USART3 */
    if ((step % 10) == 0)
    {
      uart3_printf("tick=%lu | link=%s | ADC=[%u, %u, %u, %u]\r\n",
                   (unsigned long)osKernelGetTickCount(),
                   netif_is_link_up(&gnetif) ? "UP" : "DOWN",
                   plc_adc_raw[0], plc_adc_raw[1],
                   plc_adc_raw[2], plc_adc_raw[3]);
    }

    step++;
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

