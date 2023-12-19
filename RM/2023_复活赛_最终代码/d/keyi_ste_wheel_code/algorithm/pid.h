#ifndef __PID_H
#define __PID_H
/* PID���� */
#include "typedef.h"
#include "stdint.h"

typedef struct {
	float Target; 			        //�趨Ŀ��ֵ
	float Measured; 				    //����ֵ
	float err; 						      //����ƫ��ֵ
	float err_last; 				    //��һ��ƫ��
	float err_beforeLast; 			//���ϴ�ƫ��
	float Kp, Ki, Kd;				    //Kp, Ki, Kd����ϵ��
	float p_out;
  float i_out;
  float d_out;            //���������ֵ
	float pwm; 						      //pwm���
	uint32_t MaxOutput;				  //����޷�
  uint32_t IntegralLimit;			//�����޷� 
//float (*Incremental_PID)(struct incrementalpid_t *pid_t, float target, float measured);	
}incrementalpid_t;

typedef struct {
	float Target; 					    //�趨Ŀ��ֵ
	float Measured; 				    //����ֵ
	float err; 						      //����ƫ��ֵ
	float err_last; 				    //��һ��ƫ��
	float integral_err; 			  //����ƫ��֮��
	float Kp, Ki, Kd;				    //Kp, Ki, Kd����ϵ��
	float p_out;
  float i_out;
  float d_out;            //���������ֵ
	float pwm; 						      //pwm���
	uint32_t MaxOutput;				  //����޷�
  uint32_t IntegralLimit;			//�����޷� 
	float I_SeparThresh;   /*!< ���ַ�����ֵ����Ϊ������fabs(error)���ڸ���ֵȡ���������á�*/
//  float (*Position_PID)(struct positionpid_t *pid_t, float target, float measured);
}positionpid_t;

void IncrementalPID_paraReset(incrementalpid_t *pid_t, float kp, float ki, float kd, uint32_t MaxOutput, uint32_t IntegralLimit);
float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);
void abs_limit(float *a, float ABS_MAX);

void PositionPID_paraReset(positionpid_t *pid_t, float kp, float ki, float kd, uint32_t MaxOutput, uint32_t IntegralLimit);
float Position_PID(positionpid_t *pid_t, float target, float measured);

//����ʽPID��������
void IncrementPID_Reset(incrementalpid_t *pid_t);
//����ʽPID��������
void PositionPID_Reset(positionpid_t *pid_t);
/*****************************����Ϊר��PID************************************************/
float Handle_Angle8191_PID_Over_Zero(float tar, float cur);//���㴦��
float Handle_Angle360_PID_Over_Zero(float tar, float cur);//���㴦��

float imu_Position_PID(imu_positionpid_t *pid_t, float target, float measured);//�������¶ȿ���


float YAW_Position_PID(positionpid_t *pid_t, float target, float measured);//YAW����PID
float YAW_Incremental_PID(incrementalpid_t *pid_t, float target, float measured);//YAW����PID


float Follow_PTZ_Incremental_PID(incrementalpid_t *pid_t, float target, float measured);
float Follow_PTZ_Position_PID(positionpid_t *pid_t, float target, float measured);
float Follow_PTZ_Incremental_PID_t(incrementalpid_t *pid_t, float target, float measured);


//YAW����ƻ��� PID λ��
float YAW_Centre_Position_PID(positionpid_t *pid_t, float target, float measured);
float YAW_Centre_Incremental_PID(incrementalpid_t *pid_t, float target, float measured);

//PICTH����� PID �⻷
float PICTH_Position_OUT_PID(positionpid_t *pid_t, float target, float measured);
//PICTH����� PID �ڻ�
float PICTH_Position_IN_PID(positionpid_t *pid_t, float target, float measured);

float Shoot_Position_PID(positionpid_t *pid_t, float target, float measured);


float YAW_Vision_OUT_PID(positionpid_t *pid_t, float target, float measured);
float YAW_Vision_IN_PID(positionpid_t *pid_t, float target, float measured);

float RUD_OUT_PID(positionpid_t *pid_t, float target, float measured);
float RUD_IN_PID(positionpid_t *pid_t, float target, float measured);

float Follow_OUT_PID(positionpid_t *pid_t, float target, float measured);
float Follow_IN_PID(incrementalpid_t *pid_t, float target, float measured);

float RUD_SPIN_OUT_PID(positionpid_t *pid_t, float target, float measured);
float RUD_SPIN_IN_PID(positionpid_t *pid_t, float target, float measured);

float SPIN_IN_PID(incrementalpid_t *pid_t, float target, float measured);

#endif





