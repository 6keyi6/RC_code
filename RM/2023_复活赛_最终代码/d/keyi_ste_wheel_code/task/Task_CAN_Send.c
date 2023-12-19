#include "Task_CAN_Send.h"
#include "Robot_control.h"
#include "cmsis_os.h"
uint8_t RefereeData[8]; /*<! 板间通信存储数据 */
void Task_CAN_Send(void const * argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //每2毫秒强制进入总控制
    for (;;)
    {
			ShootFreq_Calc();//--- 计算发射射频	
      RefereeMsg_Send(RefereeData);	//裁判系统信息发送至云台主控
      vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }	
}
