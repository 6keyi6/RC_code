#include "Task_RobotContr.h"
#include "Robot_control.h"
#include "cmsis_os.h"
#include "Shoot.h"
#include "Control_Vision.h"

void RobotControl(void const * argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //每2毫秒强制进入总控制
	
	  Cloud_PID_Init();//初始化PID
	  Dial_PID_Init();
    Frict_PID_Init();
    for (;;)
    {
			Vision_processing();//视觉数据采样
      Robot_control();
      vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

