/**
 * @file Cloud_control.h
 * @author keyi (hzjqq66@163.com)
 * @brief 
 * @version 1.1
 * @date 2022-09-9
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __CLOUD_CONTROL_H
#define __CLOUD_CONTROL_H

#include "pid.h"
#include "M6020_Motor.h"
#include "user_lib.h"
#define ENCODER_ANGLE_RATIO 22.7527f 

//#define GIMBAL_PIT_MIN		8030-8192//5400//7700-8192	// Pit ������λ���ֵ 1400
//#define GIMBAL_PIT_MAX		1080//6500	// Pit ������λ��Сֵ


// ������ײ�����Ӧ��ֵ���ݵ���İ�װ��ʽ����-------------*/
//#define GIMBAL_PIT_MIN		4700	 // ������Pit ����������λ 
//#define GIMBAL_PIT_MAX		6000  	 // ������Pit ���븩����λ
//#define GIMBAL_PIT_CENTRE 	//������Pit5150

#define GIMBAL_PIT_MIN		5150	 // Pit ����������λ 
#define GIMBAL_PIT_MAX		6820  	 // Pit ���븩����λ
#define GIMBAL_PIT_CENTRE 	5895

#define GIMBAL_PIT_MIN1		5150	 // Pit ����������λ 
#define GIMBAL_PIT_MAX1		6820  	 // Pit ���븩����λ
/**
  * @brief  ��̨�ܿ��ƺ���
  * @param  void
  * @retval None
  */
void Cloud_control(void);
/***********************��ʼ��**********************************/
/**
  * @brief  ��̨��PID��ʼ��
  * @param  void
  * @retval None
  */
void Cloud_PID_Init(void);
/**
  * @brief  �����ǿ���YAW����PID��ʼ��
  * @param  void
  * @retval None
  */
void IMU_control_YAW_PID_Init(void);
/**
  * @brief  YAW�������PID��ʼ��
  * @param  void
  * @retval None
  */
void YAW_PID_Init(void);

/**
  * @brief  ��̨PICTH��Ƕȿ��Ƴ�ʼ��
  * @param  void
  * @retval None
  */
void PITCH_PID_Init(void);	
/***********************��ʼ��**********************************/

/**���ó�ʼĿ��Ƕ�
  * @brief  
  * @param  void
  * @retval None
  */
void Set_InitAngle(void);
/**Ŀ��Ƕȸ���
  * @brief  
  * @param  void
  * @retval None
  */
void TargetAngle_Update(float Yawparam, float Pitparam);
/***********************PID����**********************************/
/**
  * @brief  �����ǿ���YAW
  * @param  void
  * @retval None
  */
void IMU_control_YAW(void);
/**
  * @brief  YAW�����
  * @param  void
  * @retval None
  */
void YAW_Centre(void);
/**
  * @brief  �����ǿ���PICTH
  * @param  void
  * @retval None
  */
void IMU_control_PITCH(void); 
/**
  * @brief  PICTH��Ƕȿ���
  * @param  void
  * @retval None
  */
void Pit_AngleLimit(void);
/**
  * @brief  ƫ��Ƕ��޷���ֵ
  * @param  float
  * @retval None
  */
float Get_PitDeviateH(void);
/**
  * @brief  ��ȡPit���ƫ���޷�ֵ�ĽǶ�
  * @param  float
  * @retval ƫ��Ƕ��޷���ֵ
  */
float Get_PitDeviateL(void);
/**
  * @brief  �Ӿ�YAW������PID����
  * @param  void
  * @retval None
  */
void Cloud_YAWIMUVisionPID(void);

/**
  * @brief  �Ӿ�PIT������PID����
  * @param  void
  * @retval None
  */
void Cloud_PITIMUVisionPID(void);


float Cloud_VisionFusion(float Angle_raw);
/***********************PID����**********************************/

/**
  * @brief  ��ȡ��̨����ֵ
  * @param  void
  * @retval None
  */
void Get_CloudMeasured(void);

/**
 * @brief  	   ����Ƕȸ���
 * @param[in]  keyi
 * @retval 	   None
 */
void Vision_Angle_Update(void);


/**
 * @brief      ����ң��ģʽ�µ���̨����̵ķ���Ƕ�
 * @param[in]  Switch
 * @param[in]  yawparam
 * @retval     None
 */
void CHAS_SeparateLimit(bool Switch, float *yawparam);
/**
 * @brief      ���⿪��
 * @param[in]  None
 * @retval     None
 */
void Laser_Ctrl(bool state);

/**
 * @brief      Pit����Ƕ��޷�
 * @param[in]  None
 * @retval     None
 */
void Pit_AngleLimit(void);

/**
 * @brief      ��ȡPit���ƫ���޷�ֵ�ĽǶ�
 * @param[in]  None
 * @retval     deviate encoder
 */
float Get_PitDeviateH(void);
float Get_PitDeviateL(void);
float Get_PitDeviateH1(void);
float Get_PitDeviateL1(void);

extern uint8_t Self_aiming_logo;
extern uint8_t init_mode;
extern YAWVision_t YAWVision;


extern first_order_filter_type_t first_order_filter_BIG_YAW;
extern fp32 num_YAW[1];

extern first_order_filter_type_t first_order_filter_BIG_PIT;
extern fp32 num_PIT[1];

 float Cloud_getPitchAngleWithUp(void);
  float Cloud_getPitchAngleWithDown(void);
 void Pit_AngleLimit_1(float *PitchAngle);
 void Pit_AngleLimit2(void);
#endif


