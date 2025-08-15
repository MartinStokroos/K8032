/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD_GREEN_Pin GPIO_PIN_13
#define LD_GREEN_GPIO_Port GPIOC
#define D1_Pin GPIO_PIN_0
#define D1_GPIO_Port GPIOA
#define D2_Pin GPIO_PIN_1
#define D2_GPIO_Port GPIOA
#define D3_Pin GPIO_PIN_2
#define D3_GPIO_Port GPIOA
#define D4_Pin GPIO_PIN_3
#define D4_GPIO_Port GPIOA
#define D5_Pin GPIO_PIN_4
#define D5_GPIO_Port GPIOA
#define D6_Pin GPIO_PIN_5
#define D6_GPIO_Port GPIOA
#define D7_Pin GPIO_PIN_6
#define D7_GPIO_Port GPIOA
#define D8_Pin GPIO_PIN_7
#define D8_GPIO_Port GPIOA
#define A1_Pin GPIO_PIN_0
#define A1_GPIO_Port GPIOB
#define A2_Pin GPIO_PIN_1
#define A2_GPIO_Port GPIOB
#define I1_Pin GPIO_PIN_12
#define I1_GPIO_Port GPIOB
#define I2_Pin GPIO_PIN_13
#define I2_GPIO_Port GPIOB
#define I3_Pin GPIO_PIN_14
#define I3_GPIO_Port GPIOB
#define I4_Pin GPIO_PIN_15
#define I4_GPIO_Port GPIOB
#define CNT1_Pin GPIO_PIN_8
#define CNT1_GPIO_Port GPIOA
#define CNT2_Pin GPIO_PIN_9
#define CNT2_GPIO_Port GPIOA
#define I5_Pin GPIO_PIN_10
#define I5_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_6
#define PWM1_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_7
#define PWM2_GPIO_Port GPIOB
#define SK6_Pin GPIO_PIN_8
#define SK6_GPIO_Port GPIOB
#define SK5_Pin GPIO_PIN_9
#define SK5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
enum epoutBytes { // incoming package bytes
	CMD,
	DOUT,
	DAC1,
	DAC2,
	RESET1,
	RESET2,
	DEB1,
	DEB2
};

enum epinBytes { // outgoing package bytes
	DIN,
	BOARD_ID,
	AN1,
	AN2,
	CNT1_MSB,
	CNT1_LSB,
	CNT2_MSB,
	CNT2_LSB
};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
