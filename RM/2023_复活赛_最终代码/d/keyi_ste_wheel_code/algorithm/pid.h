#ifndef __PID_H
#define __PID_H
/* PID参数 */
#include "typedef.h"
#include "stdint.h"

typedef struct {
	float Target; 			        //设定目标值
	float Measured; 				    //测量值
	float err; 						      //本次偏差值
	float err_last; 				    //上一次偏差
	float err_beforeLast; 			//上上次偏差
	float Kp, Ki, Kd;				    //Kp, Ki, Kd控制系数
	float p_out;
  float i_out;
  float d_out;            //各部分输出值
	float pwm; 						      //pwm输出
	uint32_t MaxOutput;				  //输出限幅
  uint32_t IntegralLimit;			//积分限幅 
//float (*Incremental_PID)(struct incrementalpid_t *pid_t, float target, float measured);	
}incrementalpid_t;

typedef struct {
	float Target; 					    //设定目标值
	float Measured; 				    //测量值
	float err; 						      //本次偏差值
	float err_last; 				    //上一次偏差
	float integral_err; 			  //所有偏差之和
	float Kp, Ki, Kd;				    //Kp, Ki, Kd控制系数
	float p_out;
  float i_out;
  float d_out;            //各部分输出值
	float pwm; 						      //pwm输出
	uint32_t MaxOutput;				  //输出限幅
  uint32_t IntegralLimit;			//积分限幅 
	float I_SeparThresh;   /*!< 积分分离阈值，需为正数。fabs(error)大于该阈值取消积分作用。*/
//  float (*Position_PID)(struct positionpid_t *pid_t, float target, float measured);
}positionpid_t;

void IncrementalPID_paraReset(incrementalpid_t *pid_t, float kp, float ki, float kd, uint32_t MaxOutput, uint32_t IntegralLimit);
float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);
void abs_limit(float *a, float ABS_MAX);

void PositionPID_paraReset(positionpid_t *pid_t, float kp, float ki, float kd, uint32_t MaxOutput, uint32_t IntegralLimit);
float Position_PID(positionpid_t *pid_t, float target, float measured);

//增量式PID参数清零
void IncrementPID_Reset(incrementalpid_t *pid_t);
//增量式PID参数清零
void PositionPID_Reset(positionpid_t *pid_t);
/*****************************以下为专属PID************************************************/
float Handle_Angle8191_PID_Over_Zero(float tar, float cur);//过零处理
float Handle_Angle360_PID_Over_Zero(float tar, float cur);//过零处理

float imu_Position_PID(imu_positionpid_t *pid_t, float target, float measured);//陀螺仪温度控制


float YAW_Position_PID(positionpid_t *pid_t, float target, float measured);//YAW轴电机PID
float YAW_Incremental_PID(incrementalpid_t *pid_t, float target, float measured);//YAW轴电机PID


float Follow_PTZ_Incremental_PID(incrementalpid_t *pid_t, float target, float measured);
float Follow_PTZ_Position_PID(positionpid_t *pid_t, float target, float measured);
float Follow_PTZ_Incremental_PID_t(incrementalpid_t *pid_t, float target, float measured);


//YAW轴控制回中 PID 位置
float YAW_Centre_Position_PID(positionpid_t *pid_t, float target, float measured);
float YAW_Centre_Incremental_PID(incrementalpid_t *pid_t, float target, float measured);

//PICTH轴控制 PID 外环
float PICTH_Position_OUT_PID(positionpid_t *pid_t, float target, float measured);
//PICTH轴控制 PID 内环
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





