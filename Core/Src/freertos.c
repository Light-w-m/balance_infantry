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
#include "ins_task.h"
#include "chassisR_task.h"
#include "chassisL_task.h"
#include "vofa_task.h"
#include "remote_task.h"
#include "observe_task.h"
#include "Robot.h"
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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId INS_TASKHandle;
osThreadId VOFADEBUGHandle;
osThreadId CHASSISR_TASKHandle;
osThreadId MOTORTASKHandle;
osThreadId REMOTE_TASKHandle;
osThreadId CHASSISL_TASKHandle;
osThreadId OBSERVE_TASKHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartInsTask(void const * argument);
void StartVofaDebug(void const * argument);
void StartChassisRTask(void const * argument);
void StartMotorTask(void const * argument);
void StartRemoteTask(void const * argument);
void StartChassisLTask(void const * argument);
void StartObserveTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of INS_TASK */
  osThreadDef(INS_TASK, StartInsTask, osPriorityHigh, 0, 1024);
  INS_TASKHandle = osThreadCreate(osThread(INS_TASK), NULL);

  /* definition and creation of VOFADEBUG */
  osThreadDef(VOFADEBUG, StartVofaDebug, osPriorityNormal, 0, 256);
  VOFADEBUGHandle = osThreadCreate(osThread(VOFADEBUG), NULL);

  /* definition and creation of CHASSISR_TASK */
  osThreadDef(CHASSISR_TASK, StartChassisRTask, osPriorityAboveNormal, 0, 512);
  CHASSISR_TASKHandle = osThreadCreate(osThread(CHASSISR_TASK), NULL);

  /* definition and creation of MOTORTASK */
  osThreadDef(MOTORTASK, StartMotorTask, osPriorityAboveNormal, 0, 512);
  MOTORTASKHandle = osThreadCreate(osThread(MOTORTASK), NULL);

  /* definition and creation of REMOTE_TASK */
  osThreadDef(REMOTE_TASK, StartRemoteTask, osPriorityAboveNormal, 0, 512);
  REMOTE_TASKHandle = osThreadCreate(osThread(REMOTE_TASK), NULL);

  /* definition and creation of CHASSISL_TASK */
  osThreadDef(CHASSISL_TASK, StartChassisLTask, osPriorityAboveNormal, 0, 512);
  CHASSISL_TASKHandle = osThreadCreate(osThread(CHASSISL_TASK), NULL);

  /* definition and creation of OBSERVE_TASK */
  osThreadDef(OBSERVE_TASK, StartObserveTask, osPriorityAboveNormal, 0, 512);
  OBSERVE_TASKHandle = osThreadCreate(osThread(OBSERVE_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartInsTask */
/**
* @brief Function implementing the INS_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInsTask */
void StartInsTask(void const * argument)
{
  /* USER CODE BEGIN StartInsTask */
  /* Infinite loop */
  for(;;)
  {
    INS_task();
    osDelay(1);
  }
  /* USER CODE END StartInsTask */
}

/* USER CODE BEGIN Header_StartVofaDebug */
/**
* @brief Function implementing the VOFADEBUG thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVofaDebug */
void StartVofaDebug(void const * argument)
{
  /* USER CODE BEGIN StartVofaDebug */
  /* Infinite loop */
  for(;;)
  {
    VofaDebug_Task();
    osDelay(1);
  }
  /* USER CODE END StartVofaDebug */
}

/* USER CODE BEGIN Header_StartChassisRTask */
/**
* @brief Function implementing the CHASSISR_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisRTask */
void StartChassisRTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisRTask */
  /* Infinite loop */
  for(;;)
  {
    ChassisR_Task();
    osDelay(1);
  }
  /* USER CODE END StartChassisRTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MOTORTASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorTask */
  /* Infinite loop */
  for(;;)
  {
    MotorTask();
    osDelay(1);
  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartRemoteTask */
/**
* @brief Function implementing the REMOTE_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRemoteTask */
void StartRemoteTask(void const * argument)
{
  /* USER CODE BEGIN StartRemoteTask */
  /* Infinite loop */
  for(;;)
  {
    Remote_Task();
    osDelay(1);
  }
  /* USER CODE END StartRemoteTask */
}

/* USER CODE BEGIN Header_StartChassisLTask */
/**
* @brief Function implementing the CHASSISL_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisLTask */
void StartChassisLTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisLTask */
  /* Infinite loop */
  for(;;)
  {
    ChassisL_Task();
    osDelay(1);
  }
  /* USER CODE END StartChassisLTask */
}

/* USER CODE BEGIN Header_StartObserveTask */
/**
* @brief Function implementing the OBSERVE_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartObserveTask */
void StartObserveTask(void const * argument)
{
  /* USER CODE BEGIN StartObserveTask */
  /* Infinite loop */
  for(;;)
  {
    Observe_Task();
    osDelay(1);
  }
  /* USER CODE END StartObserveTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
