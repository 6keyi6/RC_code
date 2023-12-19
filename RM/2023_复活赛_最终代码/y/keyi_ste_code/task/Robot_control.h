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
#include "Cloud_control.h"
#include "Remote_Control.h"

#define USE_RM_Referee    1 // 启动裁判系统



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
////机器人工作模式
//typedef struct
//{
//  int ChassisMode;//底盘工作模式
//	int Cloud_Mode;//云台工作模式
//	int ShootMode;//射击工作模式	
//	int FrictMode;//摩擦轮模式
//} RobotWorkMode_e; 
/* --- 新加 -----------------------------------------------------------*/
/* --- 云台控制模式 -----------------------------------------------------------*/
typedef enum 
{
	Gimbal_DisableMode,	// 失能
    Gimbal_NormalMode,	// 普通运转模式
	Gimbal_SupplyMode,  // 补给模式
    Gimbal_LockMode,    // 锁住模式
	Gimbal_PCMode       // PC控制(自瞄)
}Gimbal_CtrlMode_e;
/* --- 底盘控制模式 ------------------------------------------------------------*/
typedef enum 
{
	CHAS_DisableMode,	    // 失能模式
    CHAS_followMode ,	    // 跟随模式
	CHAS_ReFollowMode,      // 反向跟随
	CHAS_SpinMode ,	        // 小陀螺模式
    CHAS_sinSpinMode,       // 变速小陀螺
	CHAS_AutoTrackMode,     // 自动追踪模式
	CHAS_LockMode,		    // 锁住底盘
	CHAS_NotfollowMode,	    // 无跟随模式

	CHAS_SupplyMode,	    // 补给模式
	CHAS_SwingMode,		    // 扭腰模式
	CHAS_InitMode,          // 初始化阶段
}CHAS_CtrlMode_e;

typedef enum 
{
    Attack_Disable,		//失能攻击
	Attack_15,	        //手动瞄准 - 低速
	Attack_18,	        //手动瞄准 - 中速
	Attack_22,	        //手动瞄准 - 中高速
	Attack_30,	        //手动瞄准 - 高速
	Attack_BigWindmill, //打符模式
	Attack_Auto,		//自动射击
	Attack_Unload		//退弹
}Attack_CtrlMode_e;
typedef enum 
{
	Vision_Disable = 0,		// 视觉失能	
	Vision_Default = 1,		// 默认自瞄(视觉方面不加预测)
	Vision_BIGWindmill = 2, // 大风车
	// Vision_Sentry = 3,	// 哨兵
	// Vision_BASE = 4,		// 基地
	Vision_Top = 5,		// 陀螺
	// Vision_Record = 6	// 录像
	Vision_Forcast = 6,		// 视觉预测
  Vision_Outpost = 7  //前哨战
}VisionMode_e;

extern CHAS_CtrlMode_e Chassis_Mode;
extern Gimbal_CtrlMode_e Gimbal_Mode;
extern Attack_CtrlMode_e Attack_Mode;
extern VisionMode_e Vision_Mode;
/**
 * @brief  	  设置底盘运作模式
 * @param[in]	mode
 * @retval 	  None
 */
void Set_ChassisMode(CHAS_CtrlMode_e mode);
/**
 * @brief  	  设置云台运作模式
 * @param[in]	mode
 * @retval 	  None
 */
void Set_GimbalMode(Gimbal_CtrlMode_e mode);
/**
 * @brief  	    设置发射运作模式
 * @param[in]	mode
 * @retval 	    None
 */
void Set_AttackMode(Attack_CtrlMode_e mode);
/**
 * @brief  	    设置视觉运作模式
 * @param[in]	mode
 * @retval 	    None
 */
void Set_VisionMode(VisionMode_e mode);


/**
 * @brief  	  机器人失能,状态复位
 * @param[in]	None
 * @retval 	  None
 */
void Robot_Disable(void);
/**
 * @brief  	    获取机器人相关信息
 * @param[in]	None
 * @retval 	    robot data
 */
//Attack_CtrlMode_e Get_AttackMode()
//{
//	return Attack_Mode;
//}
//CHAS_CtrlMode_e Get_ChassisMode()
//{
//	return Chassis_Mode;
//} 
//Gimbal_CtrlMode_e Get_GimbalMode()
//{
//	return Gimbal_Mode;
//}
//Fric_CtrlMode_e Get_FricMode()
//{
//    return Fric_Mode;
//}




typedef struct
{
		float BulletSpeed;//--- 射击初速度
		float PowerState;//--- 发射电源状态
		float SpeedLimit; //--- 子弹初速度上限
		float Shoot_Freq; //--- 发射射频
}Shoot_t;

extern Shoot_t Shoot;

extern uint8_t SendData[8]; /*<! 板间通信存储数据 */
extern Robot_t Robot;

extern float cooling_limit;
extern float cooling_heat;
extern float cooling_rate;
extern uint8_t ID;//机器人ID
extern uint16_t Shoot_Freq;        /*<! 射频 */
extern uint8_t Chassis_start;
extern float Power_Limit;//--- 底盘功率限制
extern float cap_call;
/**
  * @brief  机器人总控制
  * @param  None
  * @retval None
  */
void Robot_control(void);

void SendMsgtoCHAS(void);
/**
 * @brief      写入从底盘主控接收的裁判系统数据
 * @param[in]  id, can_rx_data
 * @retval     None
 */
void RefereeMsg_Write(uint8_t can_rx_data[]);

/**
 * @brief      写入从底盘主控接收的裁判系统数据
 * @param[in]  id, can_rx_data
 * @retval     None
 */
void RefereeMsg_Write_1(uint8_t can_rx_data[]);

#endif
