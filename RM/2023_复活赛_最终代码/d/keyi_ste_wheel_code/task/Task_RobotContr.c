#include "Task_RobotContr.h"
#include "Robot_control.h"
#include "cmsis_os.h"
#include "RMClient_UI.h"

void RobotControl(void const * argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //ÿ2����ǿ�ƽ����ܿ���
	  Chassis_PID_Init();
    for (;;)
    {
      Robot_control();
      vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
