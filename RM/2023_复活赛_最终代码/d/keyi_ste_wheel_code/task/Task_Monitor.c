#include "Task_Monitor.h"
#include "Devices_Monitor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Devices_Monitor.h"
/**
 * @brief      设备检测任务
 * @param[in]  None
 * @retval     None
 */
void Task_DevicesMonitor(void *argument)
{
	  portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(20);  // --- 20MS	
    static uint16_t monitor_cnt = 0;
	  for(;;)
    {
			  monitor_cnt++;
			  if(((monitor_cnt)%=10)==0) //--- 200ms
        {
            Devices_Detec();
        }
				
				vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
		}
}
