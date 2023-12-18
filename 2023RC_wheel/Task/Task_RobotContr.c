#include "Task_RobotContr.h"
#include "cmsis_os.h"
#include "Can_Control.h"
#include "Remote_Control.h"
#include "Robot_Control.h"
void RobotControl(void const * argument)
{
    portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //每2毫秒强制进入总控制
	Chassis_PID_Init();
    for (;;)
    {
    // 	CAN_ABS_SendData(&hcan1,0x04,0x01,0x02,0x05);//ID
			//CAN_ABS_SendData(&hcan1,0x04,0x04,0x04,0xAA);//自动
			//CAN_ABS_SendData(&hcan1,0x04,0x01,0x01,0x00);//读取值
			//CAN_ABS_SendData(&hcan1,0x04,0x01,0x03,0x01);
			//CAN_ABS_SendData(&hcan1,0x04,0x01,0x03,0x01);
//			CAN_ABS_Send(&hcan2,0x07,0x01,0x0D,0x00);
//			CAN_ABS_Send(&hcan2,0x07,0x02,0x0D,0x00);
//			CAN_ABS_Send(&hcan2,0x07,0x03,0x0D,0x00);
//			CAN_ABS_Send(&hcan2,0x07,0x04,0x0D,0x00);
//						CAN_ABS_SendReturnTime(&hcan2,0x05,0x01,0x05,0xE8,0x03);
//			CAN_ABS_SendReturnTime(&hcan2,0x05,0x02,0x05,0xE8,0x03);
		//	CAN_ABS_SendReturnTime(&hcan2,0x05,0x03,0x05,0xE8,0x03);
//			CAN_ABS_SendReturnTime(&hcan2,0x05,0x04,0x05,0xE8,0x03);
			DR16_LeverMode_Update();
			DR16RCCtrl_Update();
			Robot_control();
      vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
//			osDelay(200);
    }
}
