/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"  // For access to motor driver
#include "../../Drivers/isd04-motor-driver/src/isd04_driver.h"  // Driver functions
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t stepperTaskHandle;
const osThreadAttr_t stepperTask_attributes = {
  .name = "stepperTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StepperTask(void *argument);
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief  Function implementing the stepper motor task.
 * @param  argument: Not used
 * @retval None
 */
void StepperTask(void *argument)
{
  Isd04Driver *motor_driver = isd04_driver_get_instance();

  /* Infinite loop */
  for(;;)
  {
    if (motor_driver != NULL) {
      int32_t speed = isd04_driver_get_speed(motor_driver);
      if (speed != 0) {
        uint32_t abs_speed = (speed > 0) ? (uint32_t)speed : (uint32_t)(-speed);
        uint32_t period_ms = 1000U / abs_speed;
        uint32_t min_period_ms = (ISD04_STEP_MIN_INTERVAL_US + 999U) / 1000U;
        if (period_ms < min_period_ms) {
          period_ms = min_period_ms;
        }
        if (period_ms == 0U) {
          period_ms = 1U;
        }
        isd04_driver_pulse(motor_driver);
        osDelay(period_ms);
        continue;
      }
    }

    osDelay(1);
  }
}

/* USER CODE END Application */

