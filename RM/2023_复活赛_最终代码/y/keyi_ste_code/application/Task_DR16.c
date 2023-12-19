#include "cmsis_os.h"
#include "DR16_control.h"
/**
 * @brief      DR16数据处理任务
 * @param[in]  None
 * @retval     None
 */
void Task_DR16_Recv(void *argument)
{
//    portTickType xLastWakeTime;
//    xLastWakeTime = xTaskGetTickCount();
//    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);  // --- 1MS

//    /* Pre-Load for task */
//	/* static  */DR16_DataPack_Typedef* dr16_pack;
//    // static USART_COB* dr16_pack;

//    for(;;)
//    {
//        if(xTaskNotifyWait(0x00000000, 0xFFFFFFFF, (uint32_t *) &dr16_pack, 0) == pdTRUE)
//        {
//            /* Remote control data unpacking */
//            DR16.DataCapture((DR16_DataPack_Typedef *)dr16_pack);

//            /* Remote control data processing */
//            CTRL_DR16.LeverMode_Update();
//            CTRL_DR16.CTRLSource_Update();
//        }
//    }
	  portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //每2毫秒强制进入总控制
	

    for (;;)
    {
      DR16_LeverMode_Update();
			DR16_CTRLSource_Update();
      vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
