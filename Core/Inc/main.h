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
#include "stm32g4xx_hal.h"

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
#define TOF_PWR_Pin GPIO_PIN_13
#define TOF_PWR_GPIO_Port GPIOC
#define nRST_Pin GPIO_PIN_10
#define nRST_GPIO_Port GPIOG
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define TOF_LPn_Pin GPIO_PIN_5
#define TOF_LPn_GPIO_Port GPIOB
#define TOF_INT_Pin GPIO_PIN_6
#define TOF_INT_GPIO_Port GPIOB
#define TOF_INT_EXTI_IRQn EXTI9_5_IRQn
#define TOF_RST_Pin GPIO_PIN_7
#define TOF_RST_GPIO_Port GPIOB
#define BOOT0_Pin GPIO_PIN_8
#define BOOT0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
//#define VL53_INT_Pin GPIO_PIN_0 // Todo :provisoire, pour éviter de recoder les fonctions bas niveau
//#define VL53_INT_GPIO_Port GPIOA // Todo :provisoire, pour éviter de recoder les fonctions bas niveau
//#define VL53_RST_Pin GPIO_PIN_5 // Todo :provisoire, pour éviter de recoder les fonctions bas niveau
//#define VL53_RST_GPIO_Port GPIOB // Todo :provisoire, pour éviter de recoder les fonctions bas niveau
//#define VL53_LPn_Pin GPIO_PIN_7 // Todo :provisoire, pour éviter de recoder les fonctions bas niveau
//#define VL53_LPn_GPIO_Port GPIOB // Todo :provisoire, pour éviter de recoder les fonctions bas niveau


#define VL53_INT_Pin TOF_INT_Pin
#define VL53_INT_GPIO_Port TOF_INT_GPIO_Port

#define VL53_RST_Pin TOF_RST_Pin
#define VL53_RST_GPIO_Port TOF_RST_GPIO_Port

//#define VL53_LPn_Pin TOF_LPn_Pin
//#define VL53_LPn_GPIO_Port TOF_LPn_GPIO_Port

#define INT_C_Pin TOF_INT_Pin
#define LPn_C_GPIO_Port TOF_LPn_GPIO_Port
#define LPn_C_Pin TOF_LPn_Pin
#define PWR_EN_C_GPIO_Port TOF_PWR_GPIO_Port
#define PWR_EN_C_Pin TOF_PWR_Pin


#define SPD_KIT_ULD_VERSION "1.0.0"

#define TRACE_UART  1

/** Timing data
 */
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c1; // Present sur le connecteur J5
extern UART_HandleTypeDef huart2;

#define VL53_I2C_HANDLE &hi2c2

extern FDCAN_HandleTypeDef hfdcan1;




typedef struct {
    float ranging_frequency;    // [Hz]
    int ranging_period;         // [msec]
    int prev_tstp;              // [msec]
    int curr_tstp;              // [msec]
} APP_time_data;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
