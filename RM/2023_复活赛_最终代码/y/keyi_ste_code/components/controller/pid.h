/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�

} pid_type_def;

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
	float dt;
//  float (*Position_PID)(struct positionpid_t *pid_t, float target, float measured);
}positionpid_t;
float Handle_Angle8191_PID_Over_Zero(float tar, float cur);
//PID������ʼ������
void IncrementalPID_paraReset(incrementalpid_t *pid_t, float kp, float ki, float kd, uint32_t MaxOutput, uint32_t IntegralLimit);
void PositionPID_paraReset(positionpid_t *pid_t, float kp, float ki, float kd, uint32_t MaxOutput, uint32_t IntegralLimit);

//��ͨPID
float Position_PID(positionpid_t *pid_t, float target, float measured);
float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);

//����ʽPID��������
void IncrementPID_Reset(incrementalpid_t *pid_t);
//����ʽPID��������
void PositionPID_Reset(positionpid_t *pid_t);

float YAW_OUT_PID(positionpid_t *pid_t, float target, float measured);
float YAW_IN_PID(positionpid_t *pid_t, float target, float measured);

float PICTH_OUT_PID(positionpid_t *pid_t, float target, float measured);
float PICTH_IN_PID(positionpid_t *pid_t, float target, float measured);

float Follow_OUT_PID(positionpid_t *pid_t, float target, float measured);
//���̸��� ����
float Follow_IN_PID(incrementalpid_t *pid_t, float target, float measured) ;


float PositionPID_Cal(positionpid_t *pid_t, float target, float measured);

float Shoot_Position_PID(positionpid_t *pid_t, float target, float measured);

float YAW_Vision_OUT_PID(positionpid_t *pid_t, float target, float measured);
float YAW_Vision_IN_PID(positionpid_t *pid_t, float target, float measured);
float PICTH_Vis_OUT_PID(positionpid_t *pid_t, float target, float measured);
float PICTH_Vis_IN_PID(positionpid_t *pid_t, float target, float measured) ;
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION:��ͨPID
  *                 PID_DELTA: ���PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid������
  * @param[in]      max_iout: pid���������
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ��
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

#endif
