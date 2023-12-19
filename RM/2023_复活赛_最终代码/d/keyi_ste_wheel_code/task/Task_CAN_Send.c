#include "Task_CAN_Send.h"
#include "Robot_control.h"
#include "cmsis_os.h"
uint8_t RefereeData[8]; /*<! ���ͨ�Ŵ洢���� */
void Task_CAN_Send(void const * argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //ÿ2����ǿ�ƽ����ܿ���
    for (;;)
    {
			ShootFreq_Calc();//--- ���㷢����Ƶ	
      RefereeMsg_Send(RefereeData);	//����ϵͳ��Ϣ��������̨����
      vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }	
}
