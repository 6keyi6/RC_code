#ifndef __PID_H
#define __PID_H
#include "typedef.h"
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
}positionpid_t;



float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);
float Position_PID(positionpid_t *pid_t, float target, float measured);
void IncrementPID_Reset(incrementalpid_t *pid_t);
void PositionPID_Reset(positionpid_t *pid_t);
void IncrementalPID_paraReset(incrementalpid_t *pid_t, float kp, float ki, float kd, uint32_t MaxOutput, uint32_t IntegralLimit);
void PositionPID_paraReset(positionpid_t *pid_t, float kp, float ki, float kd, uint32_t MaxOutput, uint32_t IntegralLimit);
#endif
