/**
 * @file M2006_Motor.h
 * @author keyi (hzjqq66@163.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-8
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __M2006_MOTOR_H
#define __M2006_MOTOR_H
#include "typedef.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define M2006_READID_START 0x203

typedef enum
{
    Reloader_allow = 0,
    Reloader_ban = 1
} Reloader_license_t;
//typedef struct
//{
//    uint16_t realAngle; //�������Ļ�е�Ƕ�
//    int16_t realSpeed;  //���������ٶ�
//    int16_t realTorque; //��������ʵ��ת��

//    int16_t targetSpeed; //Ŀ���ٶ�
//    int32_t targetAngle; //Ŀ��Ƕ�

//    uint16_t lastAngle; //�ϴεĽǶ�
//    int32_t totalAngle; //�ۻ��ܹ��Ƕ�
//    int16_t turnCount;  //ת����Ȧ��

//    int16_t outCurrent;      //�������
//    int16_t inneroutCurrent; //�������

//    uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
//    uint16_t InfoUpdateFrame; //֡��
//    uint8_t OffLineFlag;      //�豸���߱�־
//} M2006s_t;

typedef struct
{
  float RC_angle;//�⻷Ŀ��
	float pos_temp;//�⻷����
	float Dial_P_result;//�ڻ�Ŀ��
	float IN_CurSpeed;//�ڻ�����
	float Dial_I_result;//
  float ClogSpeed; //�����ٶ�
	float PillObstruct_Time;//��������ʱ��
  uint16_t NeedLaunchCount; //����Ҫ����ĵ��������� 
	int PillObstructLogo;//����������־λ
	int PillObstruct_targetAngle;//����Ŀ��ֵ
	Reloader_license_t Reloader_license;
	uint8_t PillObstruct_Direction;   //�����������ִ�з���1��ת��0����
}M2006_motor_t;

//extern M2006s_t M2006_Reload;//���̵��
extern M2006_motor_t M2006_motor;//�Զ���ṹ������

/**
  * @brief  ��CAN�����л�ȡM2006�����Ϣ
  * @param[in]  RxMessage 	CAN���Ľ��սṹ��
  * @retval None
  */
void M2006_getInfo(Can_Export_Data_t RxMessage);


#endif
