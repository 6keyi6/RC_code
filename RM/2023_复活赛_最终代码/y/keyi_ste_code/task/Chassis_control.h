/**
 * @file Chassis_control.c
 * @author keyi (hzjqq66@163.com)
 * @brief 
 * @version 1.1
 * @date 2022-09-9
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __CHIASSIS_CONTROL_H
#define __CHIASSIS_CONTROL_H
#include "pid.h"
#ifndef PI
#define PI               3.14159265358979f
#endif

#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define M6020_mAngleRatio 22.7527f //��е�Ƕ�����ʵ�Ƕȵı���

#define SpinSpeedRampInit \
    {                     \
        0,                \
            20,           \
            0,            \
            0,            \
    }
//#define GIMBAL_YAW_CENTRE	700    	// Yaw ���ĵ�
/* �����˶� */
typedef struct
{
    float targetXRaw;         //����x��ԭʼ����
    float targetYRaw;         //����y��ԭʼ����
    float targetZRaw;         //����z��ԭʼ����
    float LpfAttFactor;       //�����˲�ϵ��
    float targetXLPF;         //����x���˲�������
    float targetYLPF;         //����y���˲�������
    float targetZLPF;         //����z���˲�������
    float speedLimit;         //�����ٶ�����
    uint16_t OmegaLimit;      //�����ٶ�����
    float Trace_Distance;     //����װ�װ����
    float FollowtargetYawRaw; //����Ŀ��Yaw�������̨ԭʼ����
    float FollowtargetYawLPF; //����Yaw�������̨�˲�������

    float SpeedChange_Factor;  //�ٶȸı�����
    float SpeedLimit_Factor;   //��������
    uint8_t mode;              //���̿���ģʽ
    uint8_t swingFlag;         //Ť����־λ
    float spinSpeed;           //�����ٶ�
    float swingSpeed;          //Ť���ٶ�
    uint8_t PowerOverflowFlag; //�����ʱ�־λ
} Chassis_t;
typedef struct
{
	float Follow_PTZ_P_result;
	float Follow_PTZ_I_result;
	float NotFollow_I_result[4];
	float Spin_I_result[4];
	int chassis_vx,chassis_vy,chassis_vw;//����XY�᷽��
	float Follow_PTZ_speed_result[4];
}M3508_motor_t;
/**
  * @brief  �����ܿ���
  * @param  void
  * @retval None
  */
void Chassis_control(void);
/**
  * @brief  ����PID��ʼ��
  * @param  void
  * @retval None
  */
void Chassis_PID_Init(void);

/**
  * @brief  ���̸���PID����
  * @param  void
  * @retval None
  */
void Chassis_Follow_PTZ_cal(void);

/**
  * @brief  �����޸���PID����
  * @param  void
  * @retval None
  */
void Chassis_NotFollow_cal(void);
/**
  * @brief  ȫ��ʽ
  * @param  void
  * @retval None
  */
void Omnidirectional_Formula(float *Vx, float *Vy);
/**
  * @brief  ȫ��ʽ
  * @param  void
  * @retval None
  */
void Omnidirectional_Formula1(float *Vx, float *Vy);


void Spin_Ctrl(void);
void Follow_Ctrl(void);

/**
 * @brief      ���̵��ݷŵ翪��
 * @param[in]  state
 * @retval     None
 */
void ChassisCap_Ctrl(uint8_t state);
/**
 * @brief      �����ٶ�б��
 * @param[in]  rec, target, slow_Inc
 * @retval     None
 */
void Chassis_Drv_Slow(float *rec , float target , float slow_Inc);


void Variable_Spin(float *Omega);
void NotfollowMode(void);
/**
 * @brief      ��ȡ����ϵͳ��Ϣ
 * @param[in]  None
 * @retval     Power_Limit
 */
uint16_t CHAS_Max_Get_Power(void);

extern uint8_t Cap_switch;   /*<! ����������� */
extern float angle_diff;
#endif
