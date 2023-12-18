#ifndef __Robot_CONTROL_H
#define __Robot_CONTROL_H
#include "Chassis_control.h"
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
extern CHAS_CtrlMode_e Chassis_Mode;

void Robot_control(void);
#endif




