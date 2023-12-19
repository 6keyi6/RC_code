/**
 * @file M2006_Motor.c
 * @author keyi (hzjqq66@163.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-8
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "M2006_Motor.h"

//M2006s_t M2006_Reload;//拨盘电机
//M2006s_t *M2006_Array[] = {&M2006_Reload};

/**
  * @brief  从CAN报文中获取M2006电机信息
  * @param[in]  RxMessage 	CAN报文接收结构体
  * @retval None
  */
//void M2006_getInfo(Can_Export_Data_t RxMessage)
//{
//    //报文id确认
//    // if ((RxMessage.CAN_RxHeader.StdId < M2006_READID_START) || (RxMessage.CAN_RxHeader.StdId > M2006_READID_END))
//    // 	return;
//    int32_t StdId;
//    StdId = (int32_t)RxMessage.CAN_RxHeader.StdId - M2006_READID_START;
//    // if (IndexOutofBounds(StdId, M2006_Amount))
//    // {
//    // 	Device_setAlertType(Alert_Times_SoftWare);
//    // 	return;
//    // }
//    M2006_Array[StdId]->lastAngle = M2006_Array[StdId]->realAngle;
//    //解包数据，数据格式详见C610电调说明书P9
//    M2006_Array[StdId]->realAngle = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
//    M2006_Array[StdId]->realSpeed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
//    M2006_Array[StdId]->realTorque = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);

//    if (M2006_Array[StdId]->realAngle - M2006_Array[StdId]->lastAngle < -6000)
//    {
//        M2006_Array[StdId]->turnCount++;
//    }

//    if (M2006_Array[StdId]->lastAngle - M2006_Array[StdId]->realAngle < -6000)
//    {
//        M2006_Array[StdId]->turnCount--;
//    }
//    M2006_Array[StdId]->totalAngle = M2006_Array[StdId]->realAngle + (8192 * M2006_Array[StdId]->turnCount);
//    M2006_Array[StdId]->lastAngle = M2006_Array[StdId]->realAngle;
//    M2006_Array[StdId]->InfoUpdateFrame++;
//    M2006_Array[StdId]->InfoUpdateFlag = 1;
//}
