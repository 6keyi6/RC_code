#include "Task_Monitor.h"
#include "Devices_Monitor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Dev_Buzzer.h"
#include "Remote_Control.h"
#include "Dev_OLED.h"
/**
 * @brief      设备检测任务
 * @param[in]  None
 * @retval     None
 */
uint8_t Cloud_flag = true; 
void Task_DevicesMonitor(void *argument)
{
	  portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(50);  // --- 20MS	
    static uint16_t monitor_cnt = 0;
	  static uint16_t oled_cnt = 0;
//	  DEV_Buzzer_Init(&htim4, TIM_CHANNEL_3);//超级马里奥蜂鸣器初始化
	  for(;;)
    {
			  monitor_cnt++;
			  if(((monitor_cnt)%=4)==0) //--- 200ms
        {
					    //--- 设备检测
            Devices_Detec();
        }

				
       // OLED --------------------------------------------------------------
        oled_cnt++;
        if((Get_CH2()||Get_CH3())!=0) //LX & LY
        {
            if(Get_CH2() <= -330)
            {
                OLED_ShowMessage(3); //--- ← 视觉数据
            }
            else if(Get_CH2() >= 330)
            {
                OLED_ShowMessage(1); //--- → 云台数据
            }
            else if(Get_CH3() >= 330)
            {
                OLED_ShowMessage(2); //--- ↑ 发射数据
            }
            else if(Get_CH3() <= -330)
            {
                // OLED_ShowMessage(4);  //--- ↓
            }
        }
        else if(Gimbal_Mode == Gimbal_PCMode)
        {
            OLED_ShowMessage(3); //--- 自瞄模式在线时显示视觉数据
        }
        else
        {
//            // uint32_t temp_monitor = AllDevices_MONITOR;
//            if(DevicesMonitor.Get_State(0x3FF) == Off_line || 
//               (Infantry.Write_Msg[0][0] || Infantry.Write_Msg[0][1] || Infantry.Write_Msg[0][2] || 
//                Infantry.Write_Msg[0][3] || Infantry.Write_Msg[0][4] || Infantry.Write_Msg[0][5] ||
//                Infantry.Write_Msg[0][6] || Infantry.Write_Msg[0][7] || Infantry.Write_Msg[1][0] || 
//                Infantry.Write_Msg[1][1] || Infantry.Write_Msg[1][2] || Infantry.Write_Msg[1][3]) == Off_line) //--- 不检测can0x342
//            {
//                if(oled_cnt < 100)
//                {
//                    OLED_ShowMessage(0); //--- 显示异常信息
//                }
//                // else if(oled_cnt < 200)
//                // {
//                //     OLED_ShowMessage(4); //--- 显示异常信息
//                // }
//                else if(oled_cnt < 200)
//                {
//                    OLED_ShowMessage(5); //--- 显示Logo
//                }
//                else
//                {
//                    oled_cnt = 0;
//                }
//            }
//            else
//            {
                OLED_ShowMessage(5); //--- 全部在线只显示logo
//            }
        }
				
				vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
		}
}
