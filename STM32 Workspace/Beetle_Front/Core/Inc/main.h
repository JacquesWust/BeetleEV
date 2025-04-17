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
#include "stm32f1xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define l_indicator_in_Pin GPIO_PIN_0
#define l_indicator_in_GPIO_Port GPIOC
#define r_indicator_in_Pin GPIO_PIN_1
#define r_indicator_in_GPIO_Port GPIOC
#define lowbeam_relay_Pin GPIO_PIN_1
#define lowbeam_relay_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define heaterLED_Pin GPIO_PIN_4
#define heaterLED_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define parking_in_Pin GPIO_PIN_6
#define parking_in_GPIO_Port GPIOA
#define cruise_in_Pin GPIO_PIN_4
#define cruise_in_GPIO_Port GPIOC
#define l_indicator_relay_Pin GPIO_PIN_5
#define l_indicator_relay_GPIO_Port GPIOC
#define reverseLED_Pin GPIO_PIN_0
#define reverseLED_GPIO_Port GPIOB
#define brake_relay_Pin GPIO_PIN_1
#define brake_relay_GPIO_Port GPIOB
#define reverse_relay_Pin GPIO_PIN_2
#define reverse_relay_GPIO_Port GPIOB
#define pump_relay_Pin GPIO_PIN_11
#define pump_relay_GPIO_Port GPIOB
#define contactor_relay_Pin GPIO_PIN_12
#define contactor_relay_GPIO_Port GPIOB
#define heater_relay_Pin GPIO_PIN_15
#define heater_relay_GPIO_Port GPIOB
#define r_indicator_relay_Pin GPIO_PIN_6
#define r_indicator_relay_GPIO_Port GPIOC
#define parking_relay_Pin GPIO_PIN_8
#define parking_relay_GPIO_Port GPIOC
#define highbeam_in_Pin GPIO_PIN_9
#define highbeam_in_GPIO_Port GPIOA
#define reverse_gearbox_in_Pin GPIO_PIN_10
#define reverse_gearbox_in_GPIO_Port GPIOA
#define highbeam_relay_Pin GPIO_PIN_15
#define highbeam_relay_GPIO_Port GPIOA
#define main_relay_Pin GPIO_PIN_10
#define main_relay_GPIO_Port GPIOC
#define starter_in_Pin GPIO_PIN_11
#define starter_in_GPIO_Port GPIOC
#define fan_relay_Pin GPIO_PIN_12
#define fan_relay_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define heater_in_Pin GPIO_PIN_4
#define heater_in_GPIO_Port GPIOB
#define reverse_panel_in_Pin GPIO_PIN_5
#define reverse_panel_in_GPIO_Port GPIOB
#define lowbeam_in_Pin GPIO_PIN_8
#define lowbeam_in_GPIO_Port GPIOB
#define hazard_in_Pin GPIO_PIN_9
#define hazard_in_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
