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
#define PIN_TEST2_Pin GPIO_PIN_13
#define PIN_TEST2_GPIO_Port GPIOC
#define PIN_TEST_Pin GPIO_PIN_1
#define PIN_TEST_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_6
#define SPI1_NSS_GPIO_Port GPIOB
#define SI_SDN_Pin GPIO_PIN_7
#define SI_SDN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

//void Main_Callback(void);

void Printf(const char *fmt, ...);
//void SendMSG(const char *fmt, ...);//(char * str);

// #define Printf printf

void SYS_myTick(void);
uint32_t Get_tick(void);
void test_set(int set);

#include "MPU9250.h"
#define MPU6050 0
#define MPU9250 1
#define MPU_CHIP MPU6050	//FIX HERE!!!

#define msleep HAL_Delay
#include "system.h"
#define GetTime_ms Get_tick//system_getTime_ms

#define STM32   //LINUX // STM32
#define SI4463//nRF24	//SI4463

#include <stdint.h>


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
