/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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
#define is_w_Pin GPIO_PIN_7
#define is_w_GPIO_Port GPIOF
#define ntc_phv_Pin GPIO_PIN_9
#define ntc_phv_GPIO_Port GPIOF
#define ntc_heatsink_Pin GPIO_PIN_10
#define ntc_heatsink_GPIO_Port GPIOF
#define ntc_ambient_Pin GPIO_PIN_0
#define ntc_ambient_GPIO_Port GPIOC
#define is_u_Pin GPIO_PIN_0
#define is_u_GPIO_Port GPIOA
#define dclink_voltage_Pin GPIO_PIN_3
#define dclink_voltage_GPIO_Port GPIOA
#define is_v_Pin GPIO_PIN_4
#define is_v_GPIO_Port GPIOA
#define spare_DAC_Pin GPIO_PIN_5
#define spare_DAC_GPIO_Port GPIOA
#define ntc_phw_Pin GPIO_PIN_6
#define ntc_phw_GPIO_Port GPIOA
#define pwm_phv_bottom_Pin GPIO_PIN_0
#define pwm_phv_bottom_GPIO_Port GPIOB
#define ntc_phu_Pin GPIO_PIN_1
#define ntc_phu_GPIO_Port GPIOB
#define pwm_phu_bottom_Pin GPIO_PIN_8
#define pwm_phu_bottom_GPIO_Port GPIOE
#define pwm_phu_top_Pin GPIO_PIN_9
#define pwm_phu_top_GPIO_Port GPIOE
#define pwm_phv_top_Pin GPIO_PIN_11
#define pwm_phv_top_GPIO_Port GPIOE
#define pwm_phw_bottom_Pin GPIO_PIN_12
#define pwm_phw_bottom_GPIO_Port GPIOE
#define pwm_phw_top_Pin GPIO_PIN_13
#define pwm_phw_top_GPIO_Port GPIOE
#define fan_pwm_Pin GPIO_PIN_12
#define fan_pwm_GPIO_Port GPIOD
#define enable_drive_phu_Pin GPIO_PIN_2
#define enable_drive_phu_GPIO_Port GPIOG
#define enbale_driver_phv_Pin GPIO_PIN_3
#define enbale_driver_phv_GPIO_Port GPIOG
#define enable_driver_phw_Pin GPIO_PIN_4
#define enable_driver_phw_GPIO_Port GPIOG
#define spare_DI_Pin GPIO_PIN_5
#define spare_DI_GPIO_Port GPIOG
#define spare_DIG6_Pin GPIO_PIN_6
#define spare_DIG6_GPIO_Port GPIOG
#define spare_DO_Pin GPIO_PIN_9
#define spare_DO_GPIO_Port GPIOC
#define spare_DOA9_Pin GPIO_PIN_9
#define spare_DOA9_GPIO_Port GPIOA
#define ocd_phu_Pin GPIO_PIN_2
#define ocd_phu_GPIO_Port GPIOD
#define ocd_phu_EXTI_IRQn EXTI2_IRQn
#define ocd_phv_Pin GPIO_PIN_3
#define ocd_phv_GPIO_Port GPIOD
#define ocd_phv_EXTI_IRQn EXTI3_IRQn
#define ocd_phw_Pin GPIO_PIN_4
#define ocd_phw_GPIO_Port GPIOD
#define ocd_phw_EXTI_IRQn EXTI4_IRQn
#define spare_DOG10_Pin GPIO_PIN_10
#define spare_DOG10_GPIO_Port GPIOG
#define spare_DOG11_Pin GPIO_PIN_11
#define spare_DOG11_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
