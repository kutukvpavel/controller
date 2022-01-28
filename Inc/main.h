/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_TIMER_MIN_VALUE 200-1
#define MOTOR_TIMER_PULSE_WIDTH MOTOR_TIMER_MIN_VALUE/2
#define MASTER_ENABLE_Pin LL_GPIO_PIN_13
#define MASTER_ENABLE_GPIO_Port GPIOC
#define BUTTON_Pin LL_GPIO_PIN_0
#define BUTTON_GPIO_Port GPIOA
#define GPIO_SHIFT_Pin LL_GPIO_PIN_1
#define GPIO_SHIFT_GPIO_Port GPIOA
#define BOARD_ADDR0_Pin LL_GPIO_PIN_3
#define BOARD_ADDR0_GPIO_Port GPIOA
#define BOARD_ADDR1_Pin LL_GPIO_PIN_4
#define BOARD_ADDR1_GPIO_Port GPIOA
#define BOARD_ADDR2_Pin LL_GPIO_PIN_5
#define BOARD_ADDR2_GPIO_Port GPIOA
#define IN_DATA_Pin LL_GPIO_PIN_12
#define IN_DATA_GPIO_Port GPIOB
#define IN_LOAD_Pin LL_GPIO_PIN_13
#define IN_LOAD_GPIO_Port GPIOB
#define OUT_STORE_Pin LL_GPIO_PIN_14
#define OUT_STORE_GPIO_Port GPIOB
#define OUT_DATA_Pin LL_GPIO_PIN_15
#define OUT_DATA_GPIO_Port GPIOB
#define nDRDY_Pin LL_GPIO_PIN_8
#define nDRDY_GPIO_Port GPIOA
#define nCS_Pin LL_GPIO_PIN_15
#define nCS_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
