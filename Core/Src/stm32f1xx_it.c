/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "usb_task.h"
#include "C2H6.h"
#include "CH4.h"
#include "rs485.h"
#include "bus485_task.h"
#include "modbus_master.h"
#include "modbus_slave.h"
#include "loop_profiler.h"
#include "dataup.h"
#include "fault_monitor.h"
#include "dgusii.h"
#include "HMI_task.h"
#include "start_task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern volatile uint32_t g_fault_monitor_r4_r11[8];
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
__attribute__((naked)) void HardFault_Handler(void)
{
  __asm volatile
  (
    "ldr r3, =g_fault_monitor_r4_r11      \n"
    "str r4, [r3, #0]                     \n"
    "str r5, [r3, #4]                     \n"
    "str r6, [r3, #8]                     \n"
    "str r7, [r3, #12]                    \n"
    "str r8, [r3, #16]                    \n"
    "str r9, [r3, #20]                    \n"
    "str r10, [r3, #24]                   \n"
    "str r11, [r3, #28]                   \n"
    "tst lr, #4                           \n"
    "ite eq                               \n"
    "mrseq r1, msp                        \n"
    "mrsne r1, psp                        \n"
    "mov r2, lr                           \n"
    "movs r0, %0                          \n"
    "bl fault_monitor_capture             \n"
    "b .                                  \n"
    :
    : "I" (FAULT_MONITOR_EVT_HARDFAULT)
  );
}

/**
  * @brief This function handles Memory management fault.
  */
__attribute__((naked)) void MemManage_Handler(void)
{
  __asm volatile
  (
    "ldr r3, =g_fault_monitor_r4_r11      \n"
    "str r4, [r3, #0]                     \n"
    "str r5, [r3, #4]                     \n"
    "str r6, [r3, #8]                     \n"
    "str r7, [r3, #12]                    \n"
    "str r8, [r3, #16]                    \n"
    "str r9, [r3, #20]                    \n"
    "str r10, [r3, #24]                   \n"
    "str r11, [r3, #28]                   \n"
    "tst lr, #4                           \n"
    "ite eq                               \n"
    "mrseq r1, msp                        \n"
    "mrsne r1, psp                        \n"
    "mov r2, lr                           \n"
    "movs r0, %0                          \n"
    "bl fault_monitor_capture             \n"
    "b .                                  \n"
    :
    : "I" (FAULT_MONITOR_EVT_MEMMANAGE)
  );
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
__attribute__((naked)) void BusFault_Handler(void)
{
  __asm volatile
  (
    "ldr r3, =g_fault_monitor_r4_r11      \n"
    "str r4, [r3, #0]                     \n"
    "str r5, [r3, #4]                     \n"
    "str r6, [r3, #8]                     \n"
    "str r7, [r3, #12]                    \n"
    "str r8, [r3, #16]                    \n"
    "str r9, [r3, #20]                    \n"
    "str r10, [r3, #24]                   \n"
    "str r11, [r3, #28]                   \n"
    "tst lr, #4                           \n"
    "ite eq                               \n"
    "mrseq r1, msp                        \n"
    "mrsne r1, psp                        \n"
    "mov r2, lr                           \n"
    "movs r0, %0                          \n"
    "bl fault_monitor_capture             \n"
    "b .                                  \n"
    :
    : "I" (FAULT_MONITOR_EVT_BUSFAULT)
  );
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
__attribute__((naked)) void UsageFault_Handler(void)
{
  __asm volatile
  (
    "ldr r3, =g_fault_monitor_r4_r11      \n"
    "str r4, [r3, #0]                     \n"
    "str r5, [r3, #4]                     \n"
    "str r6, [r3, #8]                     \n"
    "str r7, [r3, #12]                    \n"
    "str r8, [r3, #16]                    \n"
    "str r9, [r3, #20]                    \n"
    "str r10, [r3, #24]                   \n"
    "str r11, [r3, #28]                   \n"
    "tst lr, #4                           \n"
    "ite eq                               \n"
    "mrseq r1, msp                        \n"
    "mrsne r1, psp                        \n"
    "mov r2, lr                           \n"
    "movs r0, %0                          \n"
    "bl fault_monitor_capture             \n"
    "b .                                  \n"
    :
    : "I" (FAULT_MONITOR_EVT_USAGEFAULT)
  );
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  app_scheduler_tick();
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
__weak void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
__weak void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
__weak void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
__weak void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
