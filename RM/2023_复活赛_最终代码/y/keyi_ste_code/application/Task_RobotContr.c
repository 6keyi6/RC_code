#include "Task_RobotContr.h"
#include "Robot_control.h"
#include "cmsis_os.h"
#include "Shoot.h"
#include "Control_Vision.h"

void RobotControl(void const * argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //ÿ2����ǿ�ƽ����ܿ���
	
	  Cloud_PID_Init();//��ʼ��PID
	  Dial_PID_Init();
    Frict_PID_Init();
    for (;;)
    {
			Vision_processing();//�Ӿ����ݲ���
      Robot_control();
      vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

