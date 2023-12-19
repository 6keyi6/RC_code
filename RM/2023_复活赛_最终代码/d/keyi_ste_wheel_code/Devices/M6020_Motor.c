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
//ֱ��������Ӧ�ĵ���Ľṹ����������飬ֱ�۱��ں��ڵ��Թ۲�����ʹ�á�
M6020s_t M6020s_Yaw;           //������ص���ֵ          //IDΪ1
M6020s_t M6020s_Pitch;                                  //2
//M6020s_t * M6020_Array[] = {&M6020s_Yaw, &M6020s_Pitch}; //��Ӧ�����ID����Ϊ������+1
M6020_RUD_motor_t M6020_RUD_motor[4] = {0};
M6020s_t M6020s_RUD[4] = {0};
M6020s_t * M6020_Array[] = {&M6020s_RUD[0], &M6020s_RUD[1],&M6020s_RUD[2],&M6020s_RUD[3]}; //��Ӧ�����ID����Ϊ������+1
M6020_motor_t M6020_motor  = {0};//�Զ���ṹ��
M6020_PITCH_t PITCH_Use;//PICTH���Զ���ṹ��
YAWVision_t YAWVision;//YAW�Ӿ��Զ���ṹ��



/**
  * @brief  ��CAN�����л�ȡM6020�����Ϣ
  * @param  RxMessage 	CAN���Ľ��սṹ��
  * @retval None
  */

void M6020_getInfo(Can_Export_Data_t RxMessage)
{

    int32_t StdId;
    StdId = (int32_t)RxMessage.CAN_RxHeader.StdId - M6020_READID_START; //��0��ʼ
    // if (IndexOutofBounds(StdId, M6020_Amount))
    // {
    // 	Device_setAlertType(Alert_Times_SoftWare);
    // 	return;
    // }

    //������ݣ����ݸ�ʽ���C620���˵����P33
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

    //֡��ͳ�ƣ����ݸ��±�־λ
//    M6020_Array[StdId]->InfoUpdateFrame++;
//    M6020_Array[StdId]->InfoUpdateFlag = 1;
}
