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
#define Heat_Out_Pin GPIO_PIN_5
#define Heat_Out_GPIO_Port GPIOA
#define Humidification_Out_Pin GPIO_PIN_6
#define Humidification_Out_GPIO_Port GPIOA
#define HumAlm_In_Pin GPIO_PIN_12
#define HumAlm_In_GPIO_Port GPIOB
#define HumRun_In_Pin GPIO_PIN_13
#define HumRun_In_GPIO_Port GPIOB
#define HumServ_In_Pin GPIO_PIN_14
#define HumServ_In_GPIO_Port GPIOB
#define RST_ESP_Out_Pin GPIO_PIN_8
#define RST_ESP_Out_GPIO_Port GPIOC
#define EN_ESP_Out_Pin GPIO_PIN_9
#define EN_ESP_Out_GPIO_Port GPIOC
#define DE_RS_Out_Pin GPIO_PIN_11
#define DE_RS_Out_GPIO_Port GPIOA
#define RE_RS_Out_Pin GPIO_PIN_12
#define RE_RS_Out_GPIO_Port GPIOA
#define Led_Alarm_Out_Pin GPIO_PIN_5
#define Led_Alarm_Out_GPIO_Port GPIOB
#define Led_Warning_Out_Pin GPIO_PIN_6
#define Led_Warning_Out_GPIO_Port GPIOB
#define Led_WifI_Out_Pin GPIO_PIN_7
#define Led_WifI_Out_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
