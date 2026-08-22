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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_scheduler.h"
/* USER CODE END Includes */

#define U8_TO_U16(msb, lsb) \
  ((uint16_t)((((uint16_t)(uint8_t)(msb)) << 8) | ((uint16_t)(uint8_t)(lsb))))

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
#define SOL_OUT_Pin GPIO_PIN_0
#define SOL_OUT_GPIO_Port GPIOA
#define PUMP_OUT_Pin GPIO_PIN_1
#define PUMP_OUT_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_2
#define RS485_TX_GPIO_Port GPIOA
#define RS485_RX_Pin GPIO_PIN_3
#define RS485_RX_GPIO_Port GPIOA
#define RS485_EN_Pin GPIO_PIN_4
#define RS485_EN_GPIO_Port GPIOA
#define DAC_OUT_Pin GPIO_PIN_5
#define DAC_OUT_GPIO_Port GPIOA
#define DG408_EN_Pin GPIO_PIN_4
#define DG408_EN_GPIO_Port GPIOC
#define DG408_A0_Pin GPIO_PIN_5
#define DG408_A0_GPIO_Port GPIOC
#define DG408_A1_Pin GPIO_PIN_0
#define DG408_A1_GPIO_Port GPIOB
#define DG408_A2_Pin GPIO_PIN_1
#define DG408_A2_GPIO_Port GPIOB
#define C2H2_EN_Pin GPIO_PIN_14
#define C2H2_EN_GPIO_Port GPIOB
#define FAN_OUT_Pin GPIO_PIN_8
#define FAN_OUT_GPIO_Port GPIOA
#define HMI_EN_GPIO_Port GPIOB
#define HMI_EN_Pin GPIO_PIN_15
#define RS485_2_ENABLE_PORT GPIOB
#define RS485_2_ENABLE_PIN	GPIO_PIN_12
/* USER CODE BEGIN Private defines */
#define DEBUG_MODE 1
#define HW_VERIGY  

#define SYS_NAME "LGAS"
#define SYS_MAJOR 1
#define SYS_MINOR 0
#define SYS_PATCH 0

#define TRUE      1
#define FALSE     0  

typedef struct {
    uint8_t id;
    uint32_t _tick;
} task_tick;
/* USER CODE END Private defines */
#define STARTUP 0X01
#define WARNUP  0X02
#define RUNING  0X03
#define STOP    0X04

#define WARNMIN 1//60*6
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
