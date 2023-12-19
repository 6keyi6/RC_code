#include "start_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "bsp_led.h"

#include "bsp_usart.h"
#include "bsp_ina226.h"
#include "ina226_task.h"




#define INFO_GET_TASK_PRIO 1
#define INFO_GET_TASK_STK_SIZE 128
TaskHandle_t Info_Get_Task_Handler;

#define START_TASK_PRIO 1
#define START_TASK_STK_SIZE 128
TaskHandle_t Start_Task_Handler;

#define LED0_TASK_PRIO 2
#define LED0_TASK_STK_SIZE 128
TaskHandle_t Led0_Task_Handler;

#define OLED_TASK_PRIO 4
#define OLED_TASK_STK_SIZE 128
TaskHandle_t Oled_Task_Handler;

//#define CHECK_TASK_PRIO 2
//#define CHECK_TASK_STK_SIZE 128
//TaskHandle_t CHECK_Task_Handler;

void create_start_task(void)
{
	xTaskCreate((TaskFunction_t)start_task,
				(char*         )"start_task",
				(uint16_t      )START_TASK_STK_SIZE,
				(void *        )NULL,
				(UBaseType_t   )START_TASK_PRIO,
				(TaskHandle_t*)&Start_Task_Handler);
}
void led0_task(void *pvParameters)
{
	while(1)
	{
		LED0 =~LED0;
//		if(can_flag < 1)
//		{
//			__set_FAULTMASK(1);
//			NVIC_SystemReset();
//		}
//		else
//		{
//			can_flag = 0;
//		}
		vTaskDelay(200);
	}
}

float can_flag;
void check_task(void *pvParameters)
{
	
//	if(can_flag < 1)
//	{
//		__set_FAULTMASK(1);
//		NVIC_SystemReset();
//	}
//	else
//	{
//		can_flag = 0;
//	}
//	vTaskDelay(500);
}

void start_task(void *pvParameters)
{
	taskENTER_CRITICAL();
	xTaskCreate((TaskFunction_t)led0_task,
				(char*         )"led0_task",
				(uint16_t      )LED0_TASK_STK_SIZE,
				(void *        )NULL,
				(UBaseType_t   )LED0_TASK_PRIO,
				(TaskHandle_t*)&Led0_Task_Handler);
				
//	xTaskCreate((TaskFunction_t)check_task,
//				(char*         )"check_task",
//				(uint16_t      )CHECK_TASK_STK_SIZE,
//				(void *        )NULL,
//				(UBaseType_t   )CHECK_TASK_PRIO,
//				(TaskHandle_t*)&CHECK_Task_Handler);
				
	info_task_creat();
//	uart_task_creat();			
	vTaskDelete(Start_Task_Handler);			
	taskEXIT_CRITICAL();
}

