/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *  Copyright (c) 2018-2019 Thomas Truong
  *
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_dma.h"

#include "stm32f4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32utilities.h"
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
#define SPI_SEL2_Pin GPIO_PIN_2
#define SPI_SEL2_GPIO_Port GPIOE
#define SPI_SEL3_Pin GPIO_PIN_3
#define SPI_SEL3_GPIO_Port GPIOE
#define I2C_OE_Pin GPIO_PIN_4
#define I2C_OE_GPIO_Port GPIOE
#define DIR_X_Pin GPIO_PIN_0
#define DIR_X_GPIO_Port GPIOA
#define DIR_Y_Pin GPIO_PIN_1
#define DIR_Y_GPIO_Port GPIOA
#define DIR_Z_Pin GPIO_PIN_2
#define DIR_Z_GPIO_Port GPIOA
#define DIR_A_Pin GPIO_PIN_3
#define DIR_A_GPIO_Port GPIOA
#define DIR_B_Pin GPIO_PIN_4
#define DIR_B_GPIO_Port GPIOA
#define DIR_C_Pin GPIO_PIN_5
#define DIR_C_GPIO_Port GPIOA
#define STEP_ENABLE_Pin GPIO_PIN_6
#define STEP_ENABLE_GPIO_Port GPIOA
#define STEP_X_Pin GPIO_PIN_7
#define STEP_X_GPIO_Port GPIOA
#define CON_FEED_HOLD_Pin GPIO_PIN_0
#define CON_FEED_HOLD_GPIO_Port GPIOB
#define CON_FEED_HOLD_EXTI_IRQn EXTI0_IRQn
#define CON_CYCLE_START_Pin GPIO_PIN_1
#define CON_CYCLE_START_GPIO_Port GPIOB
#define CON_CYCLE_START_EXTI_IRQn EXTI1_IRQn
#define CON_RESET_Pin GPIO_PIN_2
#define CON_RESET_GPIO_Port GPIOB
#define CON_RESET_EXTI_IRQn EXTI2_IRQn
#define PWM_1_Pin GPIO_PIN_9
#define PWM_1_GPIO_Port GPIOE
#define PWM_2_Pin GPIO_PIN_11
#define PWM_2_GPIO_Port GPIOE
#define PWM_3_Pin GPIO_PIN_13
#define PWM_3_GPIO_Port GPIOE
#define PWM_4_Pin GPIO_PIN_14
#define PWM_4_GPIO_Port GPIOE
#define INT_LIMITS_Pin GPIO_PIN_10
#define INT_LIMITS_GPIO_Port GPIOB
#define INT_LIMITS_EXTI_IRQn EXTI15_10_IRQn
#define AUX_5_Pin GPIO_PIN_8
#define AUX_5_GPIO_Port GPIOD
#define AUX_6_Pin GPIO_PIN_9
#define AUX_6_GPIO_Port GPIOD
#define AUX_7_Pin GPIO_PIN_10
#define AUX_7_GPIO_Port GPIOD
#define AUX_8_Pin GPIO_PIN_11
#define AUX_8_GPIO_Port GPIOD
#define PWM_5_Pin GPIO_PIN_6
#define PWM_5_GPIO_Port GPIOC
#define PWM_6_Pin GPIO_PIN_7
#define PWM_6_GPIO_Port GPIOC
#define PWM_7_Pin GPIO_PIN_8
#define PWM_7_GPIO_Port GPIOC
#define PWM_8_Pin GPIO_PIN_9
#define PWM_8_GPIO_Port GPIOC
#define STEP_Y_Pin GPIO_PIN_8
#define STEP_Y_GPIO_Port GPIOA
#define STEP_Z_Pin GPIO_PIN_9
#define STEP_Z_GPIO_Port GPIOA
#define STEP_A_Pin GPIO_PIN_10
#define STEP_A_GPIO_Port GPIOA
#define STEP_B_Pin GPIO_PIN_11
#define STEP_B_GPIO_Port GPIOA
#define STEP_C_Pin GPIO_PIN_12
#define STEP_C_GPIO_Port GPIOA
#define PWM_SPIN_Pin GPIO_PIN_15
#define PWM_SPIN_GPIO_Port GPIOA
#define SPI_SCK_Pin GPIO_PIN_10
#define SPI_SCK_GPIO_Port GPIOC
#define SPI_MISO_Pin GPIO_PIN_11
#define SPI_MISO_GPIO_Port GPIOC
#define AUX_1_Pin GPIO_PIN_0
#define AUX_1_GPIO_Port GPIOD
#define AUX_2_Pin GPIO_PIN_1
#define AUX_2_GPIO_Port GPIOD
#define AUX_3_Pin GPIO_PIN_2
#define AUX_3_GPIO_Port GPIOD
#define AUX_4_Pin GPIO_PIN_3
#define AUX_4_GPIO_Port GPIOD
#define SPIN_DIR_Pin GPIO_PIN_4
#define SPIN_DIR_GPIO_Port GPIOD
#define SPIN_EN_Pin GPIO_PIN_5
#define SPIN_EN_GPIO_Port GPIOD
#define COOL_MIST_Pin GPIO_PIN_6
#define COOL_MIST_GPIO_Port GPIOD
#define COOL_FLOOD_Pin GPIO_PIN_7
#define COOL_FLOOD_GPIO_Port GPIOD
#define CON_SAFETY_DOOR_Pin GPIO_PIN_3
#define CON_SAFETY_DOOR_GPIO_Port GPIOB
#define CON_SAFETY_DOOR_EXTI_IRQn EXTI3_IRQn
#define PROBE_Pin GPIO_PIN_4
#define PROBE_GPIO_Port GPIOB
#define SPI_MOSI_Pin GPIO_PIN_5
#define SPI_MOSI_GPIO_Port GPIOB
#define SPI_SEL0_Pin GPIO_PIN_0
#define SPI_SEL0_GPIO_Port GPIOE
#define SPI_SEL1_Pin GPIO_PIN_1
#define SPI_SEL1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#include "stm32_pin_out.h"


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
