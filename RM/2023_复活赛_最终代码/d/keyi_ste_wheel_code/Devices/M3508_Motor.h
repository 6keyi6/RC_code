#ifndef __M3508_MOTOR_H
#define __M3508_MOTOR_H
#include "typedef.h"
#define M3508_READID_START 0x201
#define M3508_READID_END 0x204

#define M3508_PowerL_1 0x201
#define M3508_PowerR_2 0x202
typedef struct
{
    uint16_t realAngle;  //�������Ļ�е�Ƕ�
    int16_t realSpeed;   //���������ٶ�
    int16_t realCurrent; //��������ʵ�ʵ���
    uint8_t temperture;  //�������ĵ���¶�

    int16_t targetSpeed;  //Ŀ���ٶ�
    uint16_t targetAngle; //Ŀ��Ƕ�
    uint16_t lastAngle;   //�ϴεĽǶ�
    int32_t totalAngle;   //�ۻ��ܹ��Ƕ�
    int16_t turnCount;    //ת����Ȧ��

    int16_t outCurrent; //�������

    uint8_t InfoUpdateFlag;   //��Ϣ��ȡ���±�־
    uint16_t InfoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־
} M3508s_t;

typedef struct
{
	float Follow_PTZ_P_result;
	float Follow_PTZ_I_result;
	float NotFollow_I_result[4];
	float Spin_I_result[4];
	int chassis_vx,chassis_vy,chassis_vw;//����XY�᷽��
	float Follow_PTZ_speed_result[4];
	float SPIN_result[4];
	
}M3508_motor_t;

typedef struct
{
	float Frict_CurL;
	float Frict_CurR;
	float Frict_OUTL;
	float Frict_OUTR;	
  float RC_FrictL;
  float RC_FrictR;	

	
}Frict_motor_t;
extern Frict_motor_t Frict_motor;
extern M3508_motor_t M3508_motor;//�����Զ���ṹ��

extern M3508s_t M3508s[4];//���̵��������ֵ
extern M3508s_t M3508_PowerL; //Ħ���ֵ�� 201
extern M3508s_t M3508_PowerR; //Ħ���ֵ��; 202

void M3508_getInfo(Can_Export_Data_t RxMessage);
/**
 * @brief ��CAN�����л�ȡM3508Ħ���ֵ����Ϣ
 * 
 * @param RxMessage 
 * @return  
 */
void M3508_Friction_getInfo(Can_Export_Data_t RxMessage);
#endif
