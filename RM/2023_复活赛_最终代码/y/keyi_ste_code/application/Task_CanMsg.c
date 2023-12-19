#include "Task_CanMsg.h"
#include "typedef.h"

//#include "can_control.h"
#include "M6020_Motor.h"
//#include "M3508_Motor.h"
//#include "M2006_Motor.h"
//#include "DJI_CIMU.h"
int num ;
//void Can1Receives(void const *argument)
//{
//    Can_Export_Data_t Can_Export_Data;
//    uint32_t ID;
//	  for(;;)
//    {
////      xQueueReceive(CAN1_ReceiveHandle, &Can_Export_Data, portMAX_DELAY);///用于从一个队列中接收消息并把消息从队列中删除
////      ID = Can_Export_Data.CAN_RxHeader.StdId;	
////        if(ID == M6020_READID_START)
////        {
////					M6020_getInfo(Can_Export_Data);
////        }	
////				 else	if(ID >= M3508_READID_START || ID <= M3508_READID_END)
////        {
////           M3508_getInfo(Can_Export_Data);
////        }	
//				
//		}		
//}

//void Can2Receives(void const *argument)
//{

//	  Can_Export_Data_t Can_Export_Data;
//    uint32_t ID;
//     for(;;)
//    {
//			xQueueReceive(CAN2_ReceiveHandle, &Can_Export_Data, portMAX_DELAY);///用于从一个队列中接收消息并把消息从队列中删除
//      ID = Can_Export_Data.CAN_RxHeader.StdId;	
//			if(ID == M6020_PICTH)//picth轴   
//			{
//				 M6020_getInfo(Can_Export_Data);
//			}	
//			else if(ID == M2006_READID_START) 
//			{
//				 M2006_getInfo(Can_Export_Data);
//			}	
//			else if(ID == M3508_PowerL_1)
//			{
//				M3508_Friction_getInfo(Can_Export_Data);
//			}
//			else if(ID == M3508_PowerR_2)
//			{
//				M3508_Friction_getInfo(Can_Export_Data);
//			}

//		}
//	
//}
