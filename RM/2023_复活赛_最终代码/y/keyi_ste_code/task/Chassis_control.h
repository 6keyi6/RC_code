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
#define M6020_mAngleRatio 22.7527f //机械角度与真实角度的比率

#define SpinSpeedRampInit \
    {                     \
        0,                \
            20,           \
            0,            \
            0,            \
    }
//#define GIMBAL_YAW_CENTRE	700    	// Yaw 中心点
/* 底盘运动 */
typedef struct
{
    float targetXRaw;         //底盘x轴原始数据
    float targetYRaw;         //底盘y轴原始数据
    float targetZRaw;         //底盘z轴原始数据
    float LpfAttFactor;       //底盘滤波系数
    float targetXLPF;         //底盘x轴滤波后数据
    float targetYLPF;         //底盘y轴滤波后数据
    float targetZLPF;         //底盘z轴滤波后数据
    float speedLimit;         //底盘速度限制
    uint16_t OmegaLimit;      //底盘速度限制
    float Trace_Distance;     //跟随装甲板距离
    float FollowtargetYawRaw; //底盘目标Yaw轴跟随云台原始数据
    float FollowtargetYawLPF; //底盘Yaw轴跟随云台滤波后数据

    float SpeedChange_Factor;  //速度改变因子
    float SpeedLimit_Factor;   //限速因子
    uint8_t mode;              //底盘控制模式
    uint8_t swingFlag;         //扭腰标志位
    float spinSpeed;           //自旋速度
    float swingSpeed;          //扭腰速度
    uint8_t PowerOverflowFlag; //超功率标志位
} Chassis_t;
typedef struct
{
	float Follow_PTZ_P_result;
	float Follow_PTZ_I_result;
	float NotFollow_I_result[4];
	float Spin_I_result[4];
	int chassis_vx,chassis_vy,chassis_vw;//底盘XY轴方向
	float Follow_PTZ_speed_result[4];
}M3508_motor_t;
/**
  * @brief  底盘总控制
  * @param  void
  * @retval None
  */
void Chassis_control(void);
/**
  * @brief  底盘PID初始化
  * @param  void
  * @retval None
  */
void Chassis_PID_Init(void);

/**
  * @brief  底盘跟随PID计算
  * @param  void
  * @retval None
  */
void Chassis_Follow_PTZ_cal(void);

/**
  * @brief  底盘无跟随PID计算
  * @param  void
  * @retval None
  */
void Chassis_NotFollow_cal(void);
/**
  * @brief  全向公式
  * @param  void
  * @retval None
  */
void Omnidirectional_Formula(float *Vx, float *Vy);
/**
  * @brief  全向公式
  * @param  void
  * @retval None
  */
void Omnidirectional_Formula1(float *Vx, float *Vy);


void Spin_Ctrl(void);
void Follow_Ctrl(void);

/**
 * @brief      底盘电容放电开关
 * @param[in]  state
 * @retval     None
 */
void ChassisCap_Ctrl(uint8_t state);
/**
 * @brief      底盘速度斜坡
 * @param[in]  rec, target, slow_Inc
 * @retval     None
 */
void Chassis_Drv_Slow(float *rec , float target , float slow_Inc);


void Variable_Spin(float *Omega);
void NotfollowMode(void);
/**
 * @brief      获取裁判系统信息
 * @param[in]  None
 * @retval     Power_Limit
 */
uint16_t CHAS_Max_Get_Power(void);

extern uint8_t Cap_switch;   /*<! 电容输出开关 */
extern float angle_diff;
#endif
