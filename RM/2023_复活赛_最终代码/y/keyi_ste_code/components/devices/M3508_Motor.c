#include "M3508_Motor.h"
M3508s_t M3508s[4];//底盘电机
M3508s_t M3508_PowerL; //摩擦轮电机 201
M3508s_t M3508_PowerR; //摩擦轮电机; 202
M3508s_t *M3508_Power[] = {&M3508_PowerL, &M3508_PowerR};
/**
  * @brief  从CAN报文中获取M3508电机信息
  * @param  RxMessage 	CAN报文接收结构体
  * @retval None
  */
void M3508_getInfo(Can_Export_Data_t RxMessage)
{
    uint32_t StdId;
    StdId = (int32_t)(RxMessage.CAN_RxHeader.StdId - M3508_READID_START);
    //解包数据，数据格式详见C620电调说明书P33
    M3508s[StdId].realAngle = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
    M3508s[StdId].realSpeed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
    M3508s[StdId].realCurrent = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
    M3508s[StdId].temperture = RxMessage.CANx_Export_RxMessage[6];

    //帧率统计，数据更新标志位
    M3508s[StdId].InfoUpdateFrame++;
    M3508s[StdId].InfoUpdateFlag = 1;
}
/**
 * @brief 从CAN报文中获取M3508摩擦轮电机信息
 * 
 * @param RxMessage 
 * @return  
 */
void M3508_Friction_getInfo(Can_Export_Data_t RxMessage)
{
    M3508s_t *Motor_P = NULL;
    switch (RxMessage.CAN_RxHeader.StdId)
    {
    case 0x201:
        Motor_P = M3508_Power[0];
        break;
    case 0x202:
        Motor_P = M3508_Power[1];
        break;
    }
    if (Motor_P == NULL)
        return;
    //解包数据，数据格式详见C620电调说明书P33
    Motor_P->realAngle = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
    Motor_P->realSpeed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
    Motor_P->realCurrent = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
    Motor_P->temperture = RxMessage.CANx_Export_RxMessage[6];

    //帧率统计，数据更新标志位
    Motor_P->InfoUpdateFrame++;
    Motor_P->InfoUpdateFlag = 1;
}
