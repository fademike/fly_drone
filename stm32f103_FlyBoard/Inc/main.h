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
#define BTN1_Pin GPIO_PIN_13
#define BTN1_GPIO_Port GPIOC
#define PIN_TEST2_Pin GPIO_PIN_15
#define PIN_TEST2_GPIO_Port GPIOC
#define PIN_TEST3_Pin GPIO_PIN_1
#define PIN_TEST3_GPIO_Port GPIOD
#define PIN_TEST_Pin GPIO_PIN_2
#define PIN_TEST_GPIO_Port GPIOA
#define SDCARD_SS_Pin GPIO_PIN_12
#define SDCARD_SS_GPIO_Port GPIOB
#define SPI1_NSS_Pin GPIO_PIN_6
#define SPI1_NSS_GPIO_Port GPIOB
#define SI_SDN_Pin GPIO_PIN_7
#define SI_SDN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

//void Main_Callback(void);

void Printf(const char *fmt, ...);
//void SendMSG(const char *fmt, ...);//(char * str);


void SYS_myTick(void);

void TimerIRQ_Handler (void);

#include "MPU9250.h"
#define MPU6050 0
#define MPU9250 1
#define MPU_CHIP MPU9250
#define hi2cN hi2c1	// num i2c	1 with conflict. 2 without conflict
#define CONFLICT 1 // conflict between i2c and spi

#define SI4463//nRF24	//SI4463

#include <stdint.h>


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
