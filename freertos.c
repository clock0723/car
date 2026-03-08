/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "usart.h"
#include "string.h"
#include "mpu6050.h"
#include "motor.h"
#include "i2c.h"
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

float speed_right;
float speed_left;
float v_target=0.0f;
float w_target=0.0f;
uint8_t tail[4]= {0x00,0x00,0x80,0x7F};
PID angle_pid = {
  .current = 0.0f,
  .target = 0.0f,
  .error = 0.0f,
  .last_error = 0.0f,
  .integral = 0.0f,
  .Kp = 10.0f,
  .Ki = 0.5f,
  .Kd = 1.0f,
  .output = 0.0f
};
PID distance_pid = {
  .current = 0.0f,
  .target = 0.0f,
  .error = 0.0f,
  .last_error = 0.0f,
  .integral = 0.0f,
  .Kp = 1.0f,
  .Ki = 0.05f,
  .Kd = 0.1f,
  .output = 0.0f
};
/* USER CODE END Variables */
/* Definitions for SPEEDCONTROL */
osThreadId_t SPEEDCONTROLHandle;
const osThreadAttr_t SPEEDCONTROL_attributes = {
  .name = "SPEEDCONTROL",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GETANGLE */
osThreadId_t GETANGLEHandle;
const osThreadAttr_t GETANGLE_attributes = {
  .name = "GETANGLE",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UARTCOMMUNICATE */
osThreadId_t UARTCOMMUNICATEHandle;
const osThreadAttr_t UARTCOMMUNICATE_attributes = {
  .name = "UARTCOMMUNICATE",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for target */
osMessageQueueId_t targetHandle;
const osMessageQueueAttr_t target_attributes = {
  .name = "target"
};
/* Definitions for angle */
osMessageQueueId_t angleHandle;
const osMessageQueueAttr_t angle_attributes = {
  .name = "angle"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void CONTROL(void *argument);
void ANGLE(void *argument);
void COMMUNICATE(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of target */
  targetHandle = osMessageQueueNew (16, sizeof(uint16_t), &target_attributes);

  /* creation of angle */
  angleHandle = osMessageQueueNew (16, sizeof(uint16_t), &angle_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SPEEDCONTROL */
  SPEEDCONTROLHandle = osThreadNew(CONTROL, NULL, &SPEEDCONTROL_attributes);

  /* creation of GETANGLE */
  GETANGLEHandle = osThreadNew(ANGLE, NULL, &GETANGLE_attributes);

  /* creation of UARTCOMMUNICATE */
  UARTCOMMUNICATEHandle = osThreadNew(COMMUNICATE, NULL, &UARTCOMMUNICATE_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_CONTROL */
/**
  * @brief  Function implementing the SPEEDCONTROL thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_CONTROL */
void CONTROL(void *argument)
{
  HAL_TIM_Encoder_Start(&ENCODERL, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&ENCODERR, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&PWM, PWMR);
  HAL_TIM_PWM_Start(&PWM, PWML);
  /* USER CODE BEGIN CONTROL */
  /* Infinite loop */
  for(;;)
  {
    osMessageQueueGet(angleHandle, &angle_pid.current, NULL, osWaitForever);
     
    v_target=PID_Cal(&distance_pid);
    w_target=PID_Cal(&angle_pid);
    speed_Get(&Motor_l);
    speed_Get(&Motor_r);
    PID_Cal(&Motor_l.pid);
    PID_Cal(&Motor_r.pid);
    motor_control(&Motor_l);
    motor_control(&Motor_r);
    send_vofa_data(&Motor_l);
    send_vofa_data(&Motor_r);
    osDelay(1);
  }
  /* USER CODE END CONTROL */
}

/* USER CODE BEGIN Header_ANGLE */
/**
* @brief Function implementing the ANGLECONTROL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ANGLE */
void ANGLE(void *argument)
{  mpu6050_init();
  float yaw;
  /* USER CODE BEGIN ANGLE */
  /* Infinite loop */
  for(;;)
  { mpu6050_read(&yaw);
    osMessageQueuePut(angleHandle, &yaw, 0, 0);
    osDelay(1);

  }
  /* USER CODE END ANGLE */
}

/* USER CODE BEGIN Header_COMMUNICATE */
/**
* @brief Function implementing the UARTCOMMUNICATE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_COMMUNICATE */
void COMMUNICATE(void *argument)
{
  /* USER CODE BEGIN COMMUNICATE */
  uint8_t data[100];
  float target_x, target_y;
  PID speed_left_pid = {0};
  PID speed_right_pid = {0};
  PID position_pid = {0};
  
  /* Infinite loop */
  for(;;)
  {
    if (osMessageQueueGet(targetHandle, data, NULL, 0) == osOK)
    {
      // 指令解析
      sscanf((char*)data, "X%f Y%f", &target_x, &target_y);
      sscanf((char*)data, "AKP%f", &angle_pid.Kp);
      sscanf((char*)data, "AKI%f", &angle_pid.Ki);
      sscanf((char*)data, "AKD%f", &angle_pid.Kd);
      sscanf((char*)data, "PKP%f", &position_pid.Kp);
      sscanf((char*)data, "SKP%f", &speed_left_pid.Kp);

      speed_right_pid.Kp = speed_left_pid.Kp;
    }
    send_vofa_data(&Motor_l);
    send_vofa_data(&Motor_r);
    osDelay(1);
  }
  /* USER CODE END COMMUNICATE */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

  
  
/* USER CODE END Application */

