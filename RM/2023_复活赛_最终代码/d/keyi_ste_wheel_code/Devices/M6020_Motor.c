/**
 * @file M6020_Motor.c
 * @author keyi (hzjqq66@163.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-8
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "M6020_Motor.h"
//直接声明对应的电机的结构体而不用数组，直观便于后期调试观察数据使用。
M6020s_t M6020s_Yaw;           //电调返回的数值          //ID为1
M6020s_t M6020s_Pitch;                                  //2
//M6020s_t * M6020_Array[] = {&M6020s_Yaw, &M6020s_Pitch}; //对应电机的ID必须为：索引+1
M6020_RUD_motor_t M6020_RUD_motor[4] = {0};
M6020s_t M6020s_RUD[4] = {0};
M6020s_t * M6020_Array[] = {&M6020s_RUD[0], &M6020s_RUD[1],&M6020s_RUD[2],&M6020s_RUD[3]}; //对应电机的ID必须为：索引+1
M6020_motor_t M6020_motor  = {0};//自定义结构体
M6020_PITCH_t PITCH_Use;//PICTH轴自定义结构体
YAWVision_t YAWVision;//YAW视觉自定义结构体



/**
  * @brief  从CAN报文中获取M6020电机信息
  * @param  RxMessage 	CAN报文接收结构体
  * @retval None
  */

void M6020_getInfo(Can_Export_Data_t RxMessage)
{

    int32_t StdId;
    StdId = (int32_t)RxMessage.CAN_RxHeader.StdId - M6020_READID_START; //由0开始
    // if (IndexOutofBounds(StdId, M6020_Amount))
    // {
    // 	Device_setAlertType(Alert_Times_SoftWare);
    // 	return;
    // }

    //解包数据，数据格式详见C620电调说明书P33
    M6020_Array[StdId]->lastAngle = M6020_Array[StdId]->realAngle;
    M6020_Array[StdId]->realAngle = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
    M6020_Array[StdId]->realSpeed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
    M6020_Array[StdId]->realCurrent = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
    M6020_Array[StdId]->temperture = RxMessage.CANx_Export_RxMessage[6];

    if (M6020_Array[StdId]->realAngle - M6020_Array[StdId]->lastAngle < -6500)
    {
        M6020_Array[StdId]->turnCount++;
    }

    if (M6020_Array[StdId]->lastAngle - M6020_Array[StdId]->realAngle < -6500)
    {
        M6020_Array[StdId]->turnCount--;
    }

    M6020_Array[StdId]->totalAngle = M6020_Array[StdId]->realAngle + (8192 * M6020_Array[StdId]->turnCount);

    //帧率统计，数据更新标志位
//    M6020_Array[StdId]->InfoUpdateFrame++;
//    M6020_Array[StdId]->InfoUpdateFlag = 1;
}
