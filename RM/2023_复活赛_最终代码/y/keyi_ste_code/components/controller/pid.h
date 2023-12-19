/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;

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
	float dt;
//  float (*Position_PID)(struct positionpid_t *pid_t, float target, float measured);
}positionpid_t;
float Handle_Angle8191_PID_Over_Zero(float tar, float cur);
//PID参数初始化函数
void IncrementalPID_paraReset(incrementalpid_t *pid_t, float kp, float ki, float kd, uint32_t MaxOutput, uint32_t IntegralLimit);
void PositionPID_paraReset(positionpid_t *pid_t, float kp, float ki, float kd, uint32_t MaxOutput, uint32_t IntegralLimit);

//普通PID
float Position_PID(positionpid_t *pid_t, float target, float measured);
float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);

//增量式PID参数清零
void IncrementPID_Reset(incrementalpid_t *pid_t);
//增量式PID参数清零
void PositionPID_Reset(positionpid_t *pid_t);

float YAW_OUT_PID(positionpid_t *pid_t, float target, float measured);
float YAW_IN_PID(positionpid_t *pid_t, float target, float measured);

float PICTH_OUT_PID(positionpid_t *pid_t, float target, float measured);
float PICTH_IN_PID(positionpid_t *pid_t, float target, float measured);

float Follow_OUT_PID(positionpid_t *pid_t, float target, float measured);
//底盘跟随 增量
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
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
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
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

#endif
