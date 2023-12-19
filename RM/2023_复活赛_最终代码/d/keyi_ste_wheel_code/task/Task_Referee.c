#include "Task_Referee.h"
#include "cmsis_os.h"
#include "RMClient_UI.h"
///**
// * @brief      裁判系统数据更新
// * @param[in]  None
// * @retval     None
// */
//void Task_RefereeRecv(void *arg)
//{
//    /* Pre-Load for task */
////	static USART_COB* referee_pack;
//     TickType_t xLastWakeTime_t = xTaskGetTickCount();
//    /* Infinite loop */
//    for(;;)
//		{
//			vTaskDelayUntil(&xLastWakeTime_t,1);
//	//        // Sent_Contorl(&huart1);
//	//		if(xTaskNotifyWait(0x00000000, 0xFFFFFFFF, (uint32_t *) &referee_pack, 0) == pdTRUE)
//	//		{
//	//			Referee.unPackDataFromRF((uint8_t*)referee_pack->address, referee_pack->len);		//更新裁判系统数据
//	//		}
//			/* Pass control to the next task */
//		}
//}

void Task_UserDefined_UI(void *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //每2毫秒强制进入总控制
    for (;;)
    {
  		UserDefined_UI();
      vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }	
}
