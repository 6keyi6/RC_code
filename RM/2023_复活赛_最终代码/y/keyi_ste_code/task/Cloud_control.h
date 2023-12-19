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

//#define GIMBAL_PIT_MIN		8030-8192//5400//7700-8192	// Pit 编码限位最大值 1400
//#define GIMBAL_PIT_MAX		1080//6500	// Pit 编码限位最小值


// 顶部与底部的相应数值根据电机的安装方式而定-------------*/
//#define GIMBAL_PIT_MIN		4700	 // 分区赛Pit 编码仰角限位 
//#define GIMBAL_PIT_MAX		6000  	 // 分区赛Pit 编码俯角限位
//#define GIMBAL_PIT_CENTRE 	//分区赛Pit5150

#define GIMBAL_PIT_MIN		5150	 // Pit 编码仰角限位 
#define GIMBAL_PIT_MAX		6820  	 // Pit 编码俯角限位
#define GIMBAL_PIT_CENTRE 	5895

#define GIMBAL_PIT_MIN1		5150	 // Pit 编码仰角限位 
#define GIMBAL_PIT_MAX1		6820  	 // Pit 编码俯角限位
/**
  * @brief  云台总控制函数
  * @param  void
  * @retval None
  */
void Cloud_control(void);
/***********************初始化**********************************/
/**
  * @brief  云台总PID初始化
  * @param  void
  * @retval None
  */
void Cloud_PID_Init(void);
/**
  * @brief  陀螺仪控制YAW轴电机PID初始化
  * @param  void
  * @retval None
  */
void IMU_control_YAW_PID_Init(void);
/**
  * @brief  YAW轴回中心PID初始化
  * @param  void
  * @retval None
  */
void YAW_PID_Init(void);

/**
  * @brief  云台PICTH轴角度控制初始化
  * @param  void
  * @retval None
  */
void PITCH_PID_Init(void);	
/***********************初始化**********************************/

/**设置初始目标角度
  * @brief  
  * @param  void
  * @retval None
  */
void Set_InitAngle(void);
/**目标角度更新
  * @brief  
  * @param  void
  * @retval None
  */
void TargetAngle_Update(float Yawparam, float Pitparam);
/***********************PID计算**********************************/
/**
  * @brief  陀螺仪控制YAW
  * @param  void
  * @retval None
  */
void IMU_control_YAW(void);
/**
  * @brief  YAW轴回中
  * @param  void
  * @retval None
  */
void YAW_Centre(void);
/**
  * @brief  陀螺仪控制PICTH
  * @param  void
  * @retval None
  */
void IMU_control_PITCH(void); 
/**
  * @brief  PICTH轴角度控制
  * @param  void
  * @retval None
  */
void Pit_AngleLimit(void);
/**
  * @brief  偏离角度限幅的值
  * @param  float
  * @retval None
  */
float Get_PitDeviateH(void);
/**
  * @brief  获取Pit电机偏离限幅值的角度
  * @param  float
  * @retval 偏离角度限幅的值
  */
float Get_PitDeviateL(void);
/**
  * @brief  视觉YAW轴自瞄PID计算
  * @param  void
  * @retval None
  */
void Cloud_YAWIMUVisionPID(void);

/**
  * @brief  视觉PIT轴自瞄PID计算
  * @param  void
  * @retval None
  */
void Cloud_PITIMUVisionPID(void);


float Cloud_VisionFusion(float Angle_raw);
/***********************PID计算**********************************/

/**
  * @brief  获取云台测量值
  * @param  void
  * @retval None
  */
void Get_CloudMeasured(void);

/**
 * @brief  	   自瞄角度更新
 * @param[in]  keyi
 * @retval 	   None
 */
void Vision_Angle_Update(void);


/**
 * @brief      限制遥控模式下的云台与底盘的分离角度
 * @param[in]  Switch
 * @param[in]  yawparam
 * @retval     None
 */
void CHAS_SeparateLimit(bool Switch, float *yawparam);
/**
 * @brief      激光开启
 * @param[in]  None
 * @retval     None
 */
void Laser_Ctrl(bool state);

/**
 * @brief      Pit电机角度限幅
 * @param[in]  None
 * @retval     None
 */
void Pit_AngleLimit(void);

/**
 * @brief      获取Pit电机偏离限幅值的角度
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


