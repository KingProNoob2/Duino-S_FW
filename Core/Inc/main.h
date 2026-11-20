/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIO25_Pin GPIO_PIN_13
#define DIO25_GPIO_Port GPIOC
#define DIO26_Pin GPIO_PIN_14
#define DIO26_GPIO_Port GPIOC
#define DIO27_Pin GPIO_PIN_15
#define DIO27_GPIO_Port GPIOC
#define ETH_RST_N_Pin GPIO_PIN_2
#define ETH_RST_N_GPIO_Port GPIOC
#define DIO14_Pin GPIO_PIN_2
#define DIO14_GPIO_Port GPIOB
#define DIO15_Pin GPIO_PIN_7
#define DIO15_GPIO_Port GPIOE
#define DIO24_Pin GPIO_PIN_13
#define DIO24_GPIO_Port GPIOE
#define DIO23_Pin GPIO_PIN_14
#define DIO23_GPIO_Port GPIOE
#define DIO22_Pin GPIO_PIN_15
#define DIO22_GPIO_Port GPIOE
#define DIO2_Pin GPIO_PIN_10
#define DIO2_GPIO_Port GPIOD
#define DIO3_Pin GPIO_PIN_11
#define DIO3_GPIO_Port GPIOD
#define DIO4_Pin GPIO_PIN_12
#define DIO4_GPIO_Port GPIOD
#define DIO5_Pin GPIO_PIN_13
#define DIO5_GPIO_Port GPIOD
#define DIO6_Pin GPIO_PIN_14
#define DIO6_GPIO_Port GPIOD
#define DIO7_Pin GPIO_PIN_15
#define DIO7_GPIO_Port GPIOD
#define DIO9_Pin GPIO_PIN_6
#define DIO9_GPIO_Port GPIOC
#define DIO10_Pin GPIO_PIN_7
#define DIO10_GPIO_Port GPIOC
#define DIO11_Pin GPIO_PIN_8
#define DIO11_GPIO_Port GPIOC
#define DIO12_Pin GPIO_PIN_9
#define DIO12_GPIO_Port GPIOC
#define DIO13_Pin GPIO_PIN_10
#define DIO13_GPIO_Port GPIOC
#define DIO16_Pin GPIO_PIN_11
#define DIO16_GPIO_Port GPIOC
#define DIO17_Pin GPIO_PIN_12
#define DIO17_GPIO_Port GPIOC
#define DIO18_Pin GPIO_PIN_0
#define DIO18_GPIO_Port GPIOD
#define DIO19_Pin GPIO_PIN_1
#define DIO19_GPIO_Port GPIOD
#define DIO20_Pin GPIO_PIN_2
#define DIO20_GPIO_Port GPIOD
#define DIO21_Pin GPIO_PIN_3
#define DIO21_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
