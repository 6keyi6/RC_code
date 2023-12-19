/**
 * @file Robot_control.h
 * @author keyi (hzjqq66@163.com)
 * @brief 
 * @version 1.1
 * @date 2022-09-9
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __Robot_CONTROL_H
#define __Robot_CONTROL_H
#include "Chassis_control.h"
#define USE_RM_Referee 1  // 启动裁判系统

/**********************新加********************************/
typedef enum
{
    TeamColor_Blue,
    TeamColor_Red
} TeamColor_e;

typedef enum
{
    Types_Hero,
    Types_Engineer,
    Types_Standard,
    Types_Aerial = 6,
    Types_Sentry = 7
} Types_e;
//攻击模式
typedef enum
{
    AttackMode_Disable,        //失能攻击
    AttackMode_AutoMode,       //自动瞄准
    AttackMode_ManualMode,     //手动瞄准
    AttackMode_SingleMode,     //单发模式
    AttackMode_ContinuousMode, //连发模式
    AttackMode_NoneMode,       //不击打模式
} AttackMode_e;
typedef enum
{
    ShootTarget_default = 0,     //默认模式
    ShootTarget_Self_aiming = 1, //自瞄模式
    ShootTarget_BIG_WHEEL,       //打符模式
    ShootTarget_Sentry,          //击打对方哨兵的模式
    ShootTarget_Spin,            //打小陀螺模式
    ShootTarget_Video,           //录像模式
    ShootTarget_UAV,             //我方无人机自瞄模式
    ShootTarget_OurSentry,       //我方哨兵自瞄模式
    ShootTarget_Radar,           //雷达模式

} ShootTarget_e;
typedef struct
{
    //攻击相关
    AttackMode_e Attack_AttackMode;	
    ShootTarget_e Attack_ShootTarget;
	  bool VisionEnabled; //自瞄识别开关
    //比赛相关
    TeamColor_e TeamColor; //我方的团队颜色。
    Types_e Types;         //我方兵种
} Robot_t;
/**********************新加********************************/
//机器人工作模式
typedef struct
{
  int ChassisMode;//底盘工作模式
	int Cloud_Mode;//云台工作模式
	int ShootMode;//射击工作模式	
	int FrictMode;//摩擦轮模式
} RobotWorkMode_e; 

typedef struct
{
	float Forward_Back_Value; //Vx
	float Left_Right_Value;   //Vy
	float spin;              //vw
  float YAW_Value;    //云台YAW轴自旋值
	float PICTH_Value;  //云台PICTH轴
	float Dial_Wheel;  //拨盘 往上小陀螺 往下拨弹
}Robot_TargetValue_t;
//
typedef struct
{
	CHAS_CtrlMode_e  Chassis_PreMode;
}Robot_classdef;

/* 板间通信数据索引 */
enum Commu_e
{
    Vision_State = 0,
    Fric_State = 1,
    Uphill_Mode = 2,
    Robot_Reset = 3,
    Cap_Ctrl = 4,
    Attack_Ctrl = 5,
    Mag_Ctrl = 6,
    UI_Reflash = 7
};

typedef enum 
{
	Vision_Disable = 0,		// 视觉失能	
	Vision_Default = 1,		// 默认自瞄(视觉方面不加预测)
	Vision_BIGWindmill = 2, // 大风车
	// Vision_Sentry = 3,	// 哨兵
	// Vision_BASE = 4,		// 基地
	Vision_Top = 5,		// 陀螺
	// Vision_Record = 6	// 录像
	Vision_Forcast = 6		// 视觉预测

}VisionMode_e;
extern VisionMode_e Vision_Mode;

extern Robot_TargetValue_t Robot_TargetValue;
extern RobotWorkMode_e RobotWorkMode;
extern Robot_classdef Infantry;

extern Robot_t Robot;
extern uint8_t Shoot_Freq;//子弹射频
/**
  * @brief  机器人总控制
  * @param  None
  * @retval None
  */
void Robot_control(void);

/**
  * @brief  裁判系统信息发送至云台主控
  * @param  can_rx_data
  * @retval None
  */
void RefereeMsg_Send(uint8_t *data);


/**
  * @brief  裁判系统信息发送至云台主控  2
  * @param  can_rx_data
  * @retval None
  */
void RefereeMsg_Send_2(uint8_t *data);

/**
 * @brief  	    根据枪管数据计算射频
 * @param[in]	None
 * @retval 	    ShootFreq
 */
uint8_t ShootFreq_Calc(void);

uint16_t Get_GunHeat(void);
uint16_t Get_CoolingRate(void);
uint16_t Get_CoolingLimit(void);
#endif
