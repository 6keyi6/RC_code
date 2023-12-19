/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

#include "calibrate_task.h"

#include "INS_task.h"
#include "led_flow_task.h"
#include "Debug_DataScope.h"
#include "DJI_IMU.h"
#include "bsp_buzzer.h"
#include "DJI_IMU.h"
#include "Task_RobotContr.h"
#include "Task_CanMsg.h"

#include "Robot_control.h"
#include "Control_Vision.h"
#include "Devices_Monitor.h"

#include "ANO_DT.h"
#include "Shoot.h" 
#include "Robot_control.h"
#include "DR16_control.h"

#include "M6020_Motor.h"
#include "can_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

osThreadId calibrate_tast_handle;
osThreadId imuTaskHandle;
osThreadId led_RGB_flow_handle;
osThreadId Task_Can1MsgRecHandle;
osThreadId Task_DevicesMonitorHandle; 
osThreadId Task_DR16Handle;

osMessageQId CAN_SendHandle;

unsigned char x;          //计数变量
unsigned char Send_Count; //串口需要发送的数据个数
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void ALL_Send(void const * argument)
{
  /* USER CODE BEGIN ALL_Send */
  /* Infinite loop */
  for(;;)
  {
		//视觉数据发送
		Update_VisionTarget();
    osDelay(1);
  }
  /* USER CODE END ALL_Send */
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//CAN队列句柄osThreadId Task_DevicesMonitorHandle;
osMessageQId CAN2_Queue;
osThreadId Task_ConditionHandle;
osThreadId Task_RobotContrHandle;
/* USER CODE END Variables */
osThreadId testHandle;
osThreadId Debug_TaskHandle;
osThreadId IMU_Send_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
 void Task_DevicesMonitor(void const * argument);  
 void Task_DR16_Recv(void const * argument);
/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);
void Debug(void const * argument);
void IMU_Send(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
  
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}                   
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  /* definition and creation of test */
  osThreadDef(test, test_task, osPriorityNormal, 0, 128);
  testHandle = osThreadCreate(osThread(test), NULL);

  /* definition and creation of Debug_Task */
  osThreadDef(Debug_Task, Debug, osPriorityNormal, 0, 128);
  Debug_TaskHandle = osThreadCreate(osThread(Debug_Task), NULL);

  /* definition and creation of IMU_Send_Task */
  osThreadDef(IMU_Send_Task, IMU_Send, osPriorityHigh, 0, 128);
  IMU_Send_TaskHandle = osThreadCreate(osThread(IMU_Send_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
    osThreadDef(cali, calibrate_task, osPriorityAboveNormal, 0, 512);
    calibrate_tast_handle = osThreadCreate(osThread(cali), NULL);

    osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 1024);
    imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

    osThreadDef(led, led_RGB_flow_task, osPriorityNormal, 0, 256);
    led_RGB_flow_handle = osThreadCreate(osThread(led), NULL);
		
    osThreadDef(Task_Condition, ALL_Send, osPriorityBelowNormal, 0, 128);
    Task_ConditionHandle = osThreadCreate(osThread(Task_Condition), NULL);	
		
    osThreadDef(Task_RobotContr, RobotControl, osPriorityHigh, 0, 512);
    Task_RobotContrHandle = osThreadCreate(osThread(Task_RobotContr), NULL);		
		
	osThreadDef(DevicesMonitor, Task_DevicesMonitor, osPriorityNormal, 0, 128);
  Task_DevicesMonitorHandle = osThreadCreate(osThread(DevicesMonitor), NULL);		
	
	osThreadDef(Task_DR16, Task_DR16_Recv, osPriorityNormal, 0, 128);
  Task_DR16Handle = osThreadCreate(osThread(Task_DR16), NULL);			


  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the test thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_test_task */
__weak void test_task(void const * argument)
{
  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END test_task */
}

/* USER CODE BEGIN Header_Debug */
/**
* @brief Function implementing the Debug_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Debug */
void Debug(void const * argument)
{
  /* USER CODE BEGIN Debug */
  /* Infinite loop */
	for(;;)
  {
//			DataScope_Get_Channel_Data(M6020_motor.remote_control_YAW,1);
//			DataScope_Get_Channel_Data(M6020_motor.YAW_OUT_Current,2);
//			DataScope_Get_Channel_Data(M6020_motor.YAW_P_result,3);
//			DataScope_Get_Channel_Data(M6020_motor.YAW_IN_Current,4);
//		DataScope_Get_Channel_Data(Shoot_Freq,1);

////			DataScope_Get_Channel_Data(vision_yaw_angle,3);
////			DataScope_Get_Channel_Data(vision_pit_angle,4);
//			DataScope_Get_Channel_Data(angle_diff,1);
////	 		DataScope_Get_Channel_Data(M6020_motor.YAW_OUT_Current,2);	
////		
//		  DataScope_Get_Channel_Data(YAWVision.IMUVisionErr,1);
//			DataScope_Get_Channel_Data(M6020_motor.YAW_OUT_Current,2);
//  		DataScope_Get_Channel_Data(YAWVision.Vision_OUT,3);
//			DataScope_Get_Channel_Data(M6020_motor.YAW_IN_Current,4);		
//		DataScope_Get_Channel_Data(VisionData.RawData.yaw_angle,5);
		
//				  DataScope_Get_Channel_Data(Shoot.BulletSpeed,1);
//					DataScope_Get_Channel_Data(PITCH_Use.PICTH_OUT_Current,1);
// 		  DataScope_Get_Channel_Data(PITCH_Use.RC_PICTH,2);	
//			DataScope_Get_Channel_Data(M6020moto_chassis[3].angle ,1);
// 		  DataScope_Get_Channel_Data(PITCH_Use.PICTH_OUT_Current,2);	
//			
//			DataScope_Get_Channel_Data(YAWVision.IMUVisionErr,3);
// 		  DataScope_Get_Channel_Data(M6020_motor.YAW_OUT_Current,4);			
//			DataScope_Get_Channel_Data(PITVision.Vision_OUT,3);
//			DataScope_Get_Channel_Data(PITCH_Use.PICTH_IN_Current,4);
		
			DataScope_Get_Channel_Data(Frict_motor.RC_FrictL*0.1,1);
	 		DataScope_Get_Channel_Data(Frict_motor.Frict_CurL*0.1,2);	
			DataScope_Get_Channel_Data(Frict_motor.RC_FrictR*0.1,3);
			DataScope_Get_Channel_Data(-Frict_motor.Frict_CurR*0.1,4);
//			DataScope_Get_Channel_Data(M6020_motor.remote_control_YAW,1);
//	 		DataScope_Get_Channel_Data(yyyaw,2);	
		
////		  DataScope_Get_Channel_Data(Expt.Target_Vw,3);	
			Send_Count = DataScope_Data_Generate(4);

		for( x = 0 ; x < Send_Count; x++) 
		{
		  while((USART1->SR&0X40)==0);  
		  USART1->DR = DataScope_OutPut_Buffer[x]; 
		}
////		sent_data(500,200,'a');
//      Sent_DataFloat(USART1,500,500);
//    for (int i = 0; i < 1; i++)
//    {
//        HAL_UART_Transmit(&huart1, &data[i], 1, 0xffff);
//    }

    osDelay(48);
  }
  /* USER CODE END Debug */
}

/* USER CODE BEGIN Header_IMU_Send */
/**
* @brief Function implementing the IMU_Send_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_Send */
void IMU_Send(void const * argument)
{
  /* USER CODE BEGIN IMU_Send */
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(1);  //每1毫秒强制进入总控制
  /* Infinite loop */
  for(;;)
  {
//		#if send_way == 0
//		//欧拉角
//		if(cali_sensor[0].cali_done == CALIED_FLAG && cali_sensor[0].cali_cmd == 0)
//		{
//			Euler_Send.yaw = INS_angle[0];
//			Euler_Send.pitch = INS_angle[2];///从1改为2
//			Euler_Send_Fun(Euler_Send);
//			//角速度
//			Gyro_Send.Gyro_z = INS_gyro[2];
//			Gyro_Send.Gyro_y = INS_gyro[0]; //从1改为0
//			Gyro_Send_Fun(Gyro_Send);
//			
//			#endif
//			
//		}
		Updata_Hand_Euler_Gyro_Data();//更新陀螺仪角度

    vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
  /* USER CODE END IMU_Send */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Updata_Hand_Euler_Gyro_Data(void)
{
	//角度
	DJIC_IMU.yaw = (float)INS_angle[0]  * Angle_turn_Radian + 180.f;		//将弧度转为度
	DJIC_IMU.pitch = (float)INS_angle[2] * Angle_turn_Radian + 180.0f; //(-180° ~ 180°)
	//角速度
	DJIC_IMU.Gyro_z = INS_gyro[2] * Angle_turn_Radian  ;
	DJIC_IMU.Gyro_y = INS_gyro[0] * Angle_turn_Radian ;

	//yaw轴的过零处理
	if (DJIC_IMU.yaw - DJIC_IMU.last_yaw < -300.0f)
	{
		DJIC_IMU.yaw_turnCounts++;
	}
	if (DJIC_IMU.last_yaw - DJIC_IMU.yaw < -300.0f)
	{
		DJIC_IMU.yaw_turnCounts--;
	}
	DJIC_IMU.total_yaw = DJIC_IMU.yaw + DJIC_IMU.yaw_turnCounts * 360.0f;
	DJIC_IMU.last_yaw = DJIC_IMU.yaw;

	//Pitch
	if (DJIC_IMU.pitch - DJIC_IMU.last_pitch < -300.0f)
	{
		DJIC_IMU.pitch_turnCounts++;
	}
	if (DJIC_IMU.last_pitch - DJIC_IMU.pitch < -300.0f)
	{
		DJIC_IMU.pitch_turnCounts--;
	}
	DJIC_IMU.total_pitch = DJIC_IMU.pitch + DJIC_IMU.pitch_turnCounts * 360.0f;
	DJIC_IMU.last_pitch = DJIC_IMU.pitch;
}
     



/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
