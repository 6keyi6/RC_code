/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "Debug_DataScope.h"
#include "can_control.h"
#include "Task_Referee.h"
#include "RM_JudgeSystem.h"
#include "Task_Referee.h"
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
osThreadId DBGHandle;
osThreadId Task_DevicesMonitorHandle;
osThreadId Task_UserDefined_UIHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Task_RobotContrHandle;
osThreadId Task_InitHandle;
osThreadId CAN_SENDHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void DBG(void const * argument);

void Task_DevicesMonitor(void const * argument);
void Task_UserDefined_UI(void const  *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
extern void RobotControl(void const * argument);
void All_Init(void const * argument);
extern void Task_CAN_Send(void const * argument);

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

  /* definition and creation of Task_RobotContr */
  osThreadDef(Task_RobotContr, RobotControl, osPriorityRealtime, 0, 512);
  Task_RobotContrHandle = osThreadCreate(osThread(Task_RobotContr), NULL);

  /* definition and creation of Task_Init */
  osThreadDef(Task_Init, All_Init, osPriorityNormal, 0, 128);
  Task_InitHandle = osThreadCreate(osThread(Task_Init), NULL);

  /* definition and creation of CAN_SEND */
  osThreadDef(CAN_SEND, Task_CAN_Send, osPriorityNormal, 0, 128);
  CAN_SENDHandle = osThreadCreate(osThread(CAN_SEND), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(Task_DBG, DBG, osPriorityNormal, 0, 128);
  DBGHandle = osThreadCreate(osThread(Task_DBG), NULL);

	osThreadDef(DevicesMonitor, Task_DevicesMonitor, osPriorityNormal, 0, 128);
  Task_DevicesMonitorHandle = osThreadCreate(osThread(DevicesMonitor), NULL);

	osThreadDef(UserDefined_UI, Task_UserDefined_UI, osPriorityNormal, 0, 256);
  Task_UserDefined_UIHandle = osThreadCreate(osThread(UserDefined_UI), NULL);
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

/* USER CODE BEGIN Header_All_Init */
/**
* @brief Function implementing the Task_Init thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_All_Init */
void All_Init(void const * argument)
{
  /* USER CODE BEGIN All_Init */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END All_Init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
unsigned char x;          //计数变量
unsigned char Send_Count; //串口需要发送的数据个数
void DBG(void const * argument)
{
	for(;;)
  {

//			DataScope_Get_Channel_Data(chassismode,4);
//			DataScope_Get_Channel_Data(Frame_rate,5);
//			DataScope_Get_Channel_Data(RUN,6);
//			DataScope_Get_Channel_Data(cha,7);
//			Send_Count = DataScope_Data_Generate(7);
//
//			DataScope_Get_Channel_Data(Shoot_Freq,1);
//			DataScope_Get_Channel_Data(ext_shoot_data.data.bullet_speed,2);
//			DataScope_Get_Channel_Data(Cal_Speed[0],3);
//			DataScope_Get_Channel_Data(DRIVE_speed[0],4);	
//		DataScope_Get_Channel_Data(RUD_Param[0].Target_angle,1);
//			DataScope_Get_Channel_Data(RUN_totalAngle[0],2);	
//		DataScope_Get_Channel_Data(RUD_Param[1].Target_angle,3);
//		DataScope_Get_Channel_Data(RUN_totalAngle[1],4);		
//		
//		DataScope_Get_Channel_Data(RUD_Param[2].Target_angle,5);
//			DataScope_Get_Channel_Data(RUN_totalAngle[2],6);	
//		DataScope_Get_Channel_Data(RUD_Param[3].Target_angle,7);
//		DataScope_Get_Channel_Data(RUN_totalAngle[3],8);				
//			DataScope_Get_Channel_Data(-M3508moto_chassis[0].speed_rpm,1);
//			DataScope_Get_Channel_Data(M3508moto_chassis[1].speed_rpm,2);
//			DataScope_Get_Channel_Data(M3508moto_chassis[2].speed_rpm,3);
//			DataScope_Get_Channel_Data(-M3508moto_chassis[3].speed_rpm,4);
			DataScope_Get_Channel_Data(ext_power_heat_data.data.chassis_power,1);	
		  DataScope_Get_Channel_Data(ext_power_heat_data.data.chassis_power_buffer,2);
//		  DataScope_Get_Channel_Data(ext_power_heat_data.data.chassis_power_buffer,1);
//			DataScope_Get_Channel_Data(ext_power_heat_data.data.chassis_power,1);
			Send_Count = DataScope_Data_Generate(8);		
		for( x = 0 ; x < Send_Count; x++) 
		{
		  while((USART6->SR&0X40)==0);  
		  USART6->DR = DataScope_OutPut_Buffer[x]; 
		}
    osDelay(50);
  }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
