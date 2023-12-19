/**
 * @file shoot.h
 * @author keyi (hzjqq66@163.com)
 * @brief 
 * @version 1.1
 * @date 2022-09-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "M2006_Motor.h"
#include "M3508_Motor.h"
#include "Cloud_control.h"

//#include "Control_Vision.h"
#include "pid.h"
#ifndef __SHOOT_H
#define __SHOOT_H

/*拨盘参数*/
typedef struct
{
	uint8_t Began;		    // 开始供弹
	uint8_t Stuck_Flag;		// 卡弹标志位
	int8_t Motor_Direction;	// 电机转动方向  1 -> 正  | -1 -> 反
	int16_t Stuck_Duration;	// 卡弹持续的时长
    uint16_t Num;           // 弹丸装填数
    uint16_t CanShoot_Num;  // 可射击数/
	uint16_t CriticalVal;	// 拨盘的接线速度值->卡弹临界值
	int32_t Init_Angle;	// 初始位置
	int32_t Launched_Num;	// 已经发射的子弹数
    int32_t Target_Angle;   // 正常拨弹目标值
	int32_t Re_TargetAngle; // 解除卡弹反转目标值
} Reload_t;
typedef enum 
{
	Fric_Disable, // 关闭摩擦轮
	Fric_Unload,  // 退弹
	Fric_15m_s,	  // 15m/s
	Fric_18m_s,	  // 18m/s
	Fric_22m_s,   // 22m/s
	Fric_30m_s    // 30m/s
}Fric_CtrlMode_e;
/**
  * @brief  射击总控制
  * @param  void
  * @retval None
  */ 
void Shoot_control(void);
/**
  * @brief  拨盘PID初始化
  * @param  void
  * @retval None
  */
void Dial_PID_Init(void);
/**
  * @brief  摩擦轮PID初始化
  * @param  void
  * @retval None
  */
void Frict_PID_Init(void);
/**
  * @brief  摩擦轮处理
  * @param  void
  * @retval None
  */
void Fric_processing(void);
/**
 * @brief  	   摩擦轮转速调整 (Test ing)
 * @param[in]  枪管反馈速度 射速区间 递减值 递增值
 * @note       在转速调整的基础上加入温控处理，因为经过测试发现温度越高射速也越高
 * @note       优先测15 18 30，22几乎用不到
 * @retval 	   None
 */
void FricSpeed_Adapt(void);
/**
  * @brief  拨盘PID计算
  * @param  void
  * @retval None
  */
void Dial_PID_calc(void);
/**
 * @brief  	   拨盘重置
 * @param[in]  None
 * @retval 	   None
 */
void Reload_Reset(void);
/**
 * @brief  	   设置弹丸装填数
 * @param[in]  num
 * @retval 	   None 
 */
void Set_ReloadNum(uint16_t num);
/**
 * @brief  	   判断拨盘电机是否处于卡弹临界值
 * @param[in]  None
 * @retval 	   state
 */
bool Judge_ReloadStall(void);
/**
 * @brief  	   射击模式处理
 * @param[in]  None
 * @retval 	   None
 */
void AttackMode_Ctrl(void);
/**
 * @brief  	   舵机控制
 * @param[in]  开关
 * @retval 	   None
 */
void Mag_Ctrl(uint8_t state);
/**
 * @brief  	   获取裁判系统相关信息
 * @param[in]  None
 * @retval 	   shoot power state
 */
uint16_t ShootGet_SpeedLimit(void);
float Get_GunSpeed(void);
uint16_t Get_GunHeat(void);
uint16_t Get_CoolingRate(void);
uint16_t Get_CoolingLimit(void);

extern Reload_t Reload;          /*<! 拨盘参数 */
extern Fric_CtrlMode_e Fric_Mode;
extern bool whether_Fire;
extern incrementalpid_t FrictL_PID;
extern incrementalpid_t FrictR_PID;
extern uint8_t ContLaunch;       /*<! 连发标志位 */
extern WorldTime_RxTypedef shootUnit_WorldTime;
extern int needShoot ;                     //需要射击的颗数
extern bool Mag_Switch;          /*<! 弹仓开关标志位 */
extern uint16_t Shoot_Freq;        /*<! 射频 */
extern uint8_t Rage_Mode;        /*<! 狂暴模式,无视热量限制 */

#endif
