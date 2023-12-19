/**
 * @file M6020_Motor.h
 * @author keyi (hzjqq66@163.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-8
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __M6020_MOTOR_H
#define __M6020_MOTOR_H
#include "typedef.h"
#include "pid.h"
#define M6020_READID_START 0x205 //��IDΪ1ʱ�ı���ID
#define M6020_PICTH 0x206 
typedef struct
{
    uint16_t realAngle;  //�������Ļ�е�Ƕ�
    int16_t realSpeed;   //���������ٶ�
    int16_t realCurrent; //��������ʵ��ת�ص���
    uint8_t temperture;  //�������ĵ���¶�

    int16_t targetSpeed; //Ŀ���ٶ�
    int32_t targetAngle; //Ŀ��Ƕ�
    uint16_t lastAngle;  //�ϴεĽǶ�
    float totalAngle;  //�ۻ��ܹ��Ƕ�
    int16_t turnCount;   //ת����Ȧ��

    int16_t outCurrent; //�������

    uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־
} M6020s_t;

typedef struct{
  float remote_control_YAW;//ң����YAW�������ֵ
	float YAW_P_result;     //�ǶȻ�������
	float YAW_I_result;     //�ٶȻ�������
	float IMU_T[3];            //�����ǽǶȽ��
	
	float YAW_Centre_P_result; //YAW���м�����
  float YAW_Centre_I_result; //YAW���м�����	
	
	float YAW_OUT_Current;//�⻷����
	float YAW_IN_Current;//�ڻ�����ֵ
	
	float Vis_Init_YAWangle;//�Ӿ������ʼֵ
	
}M6020_motor_t;	

typedef struct{
	float RC_PICTH;//ң����PICTH�������ֵ
	float PICTH_OUT;    //�ǶȻ�������
	float PICTH_IN;     //�ٶȻ�������
	float IMU_P[3];     //�����ǽǶȽ��
	float PICTH_OUT_Current;//�⻷����
	float PICTH_IN_Current;//�ڻ�����ֵ	
	float Vis_Init_PITangle;//�Ӿ������ʼֵ
}M6020_PITCH_t;


typedef struct
{
  float IMUVisionErr;
	float Vision_OUT;
	float Vision_IN;

}YAWVision_t;//YAW���Ӿ��ṹ��

typedef struct
{
  float IMUVisionErr;
	float Vision_OUT;
	float Vision_IN;
}PITVision_t;//PIT���Ӿ��ṹ��

void M6020_getInfo(Can_Export_Data_t RxMessage);

extern M6020s_t M6020s_Yaw;   //IDΪ1
extern M6020s_t M6020s_Pitch; //2
extern M6020s_t *M6020_Array[];//��������


extern YAWVision_t YAWVision;
extern PITVision_t PITVision;
extern positionpid_t M6020_Yaw_Ppid;//YAW��PID����
extern positionpid_t M6020_Yaw_Ipid;//YAW��PID����
extern positionpid_t  YAW_Centre_Ppid;//PID ���в���
extern incrementalpid_t YAW_Centre_Ipid;//PID ���в���
extern positionpid_t PITCH_Ppid;//picth��PID����
extern positionpid_t PITCH_Ipid;//picth��PID����

extern M6020_motor_t M6020_motor;
extern M6020_PITCH_t PITCH_Use;

extern positionpid_t Yaw_Vision_Out_Pid;//YAW�������⻷PID����
extern positionpid_t Yaw_Vision_IN_Pid;//YAW�������ڻ�PID����

extern positionpid_t PIT_Vision_Out_Pid;//PIT�������⻷PID����
extern positionpid_t PIT_Vision_IN_Pid;//PIT�������ڻ�PID����
//δ��ֵ
extern positionpid_t Vision_AnglePid_Yaw;
extern positionpid_t Vision_SpeedPid_Yaw; 
extern positionpid_t Vision_AnglePid_Pitch; 
extern positionpid_t Vision_SpeedPid_Pitch;


extern positionpid_t M6020_Yaw_OUT;//YAW��PID����
extern positionpid_t M6020_Yaw_IN;//YAW��PID����

extern positionpid_t M6020_PIT_OUT;//YAW��PID����
extern positionpid_t M6020_PIT_IN;//YAW��PID����
#endif /* __M3508_MOTOR_H */
