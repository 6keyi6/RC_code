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
#include "Chassis_control.h"
#include "Cloud_control.h"
#include "Remote_Control.h"
#include "can_control.h"
#include "Robot_control.h"
#include "math.h"
#include "Math.h"
#include "pid.h"
#include "stdlib.h"
#include "DR16_control.h"
#include "arm_math.h"
//#define GIMBAL_YAW_CENTRE	700    	// Yaw 中心点 22年国赛舵轮
//#define GIMBAL_YAW_CENTRE	651    	// Yaw 中心点 23年分区赛舵轮
#define GIMBAL_YAW_CENTRE	7522    	// Yaw 中心点 23年国赛舵轮
float angle_diff;//相对角度差
uint8_t Cap_switch;   /*<! 电容输出开关 */
float VwLimt;
//根据底盘模式 选择不同的Vw做为输入量

uint8_t ReF_Flag = false;     /*<! 反向跟随转换标志位 */
int16_t ReF_Cnt;      /*<! 反向过度累计时间 */

void Chassis_control()
{
	 	if(Gimbal_Mode == Gimbal_DisableMode)//云台失能
	  { 
			Chassis_Mode = CHAS_DisableMode;
		  return;
	  }			

		if(Chassis_Mode == CHAS_DisableMode)//失能
		{ 
			angle_diff = 0;
			return;
		}

		switch(Chassis_Mode)
		{
			case CHAS_followMode:
			{
				Follow_Ctrl();
				break;
			}
			case CHAS_SpinMode:
			{
				Spin_Ctrl();
				break;
			}
			case CHAS_NotfollowMode:
			{
				NotfollowMode();
				break;
			}
		}
		
}

void Follow_Ctrl(void)
{
	static uint8_t done_cnt; // 过度完成计时
	
// 反向跟随过度----------------------------------------------------------
    if(ReF_Flag == true)
    {
			ReF_Cnt++;
			Omnidirectional_Formula1(&Expt.Target_Vx,&Expt.Target_Vy);//全向公式
			if (fabs(M6020moto_chassis[0].angle - GIMBAL_YAW_CENTRE) <= 100.0f)
			{
					if(++done_cnt > 50)
					{
							ReF_Flag = false;
							done_cnt = 0;
					}
			}
			if (ReF_Cnt > 1000 && ReF_Flag != false) // --- 超时未转换则直接转换
			{
					ReF_Flag = false;
					ReF_Cnt = 0;
					done_cnt = 0;
			}
    }

		else
		{		
			ReF_Cnt = 0;
			if(Diagonal_Mode == true)
			{
				 Omnidirectional_Formula1(&Expt.Target_Vx,&Expt.Target_Vy);//全向公式
			}		  
		}
				angle_diff = M6020moto_chassis[0].angle - GIMBAL_YAW_CENTRE;
	      angle_diff = (Diagonal_Mode == true ? angle_diff - 1023 : angle_diff);
      	angle_diff = Handle_Angle8191_PID_Over_Zero(0,angle_diff);
}

void Spin_Ctrl(void)
{
		if(CHAS_Max_Get_Power() < 60)
		{
			angle_diff = 4700.f;
		}
		else if(CHAS_Max_Get_Power() == 60)
		{
			angle_diff = 5500.f;
		}
		else if(CHAS_Max_Get_Power() == 70)
		{
			angle_diff = 6500.f;
		}	
		else if(CHAS_Max_Get_Power() == 80)
		{
			angle_diff = 8000.f;
		}		
		else if(CHAS_Max_Get_Power() > 80)
		{
			angle_diff = 8000.f;
		}		
		if(Cap_switch == ON)
		{
		  angle_diff = 9400.f;
		}

//		if(GetKeyMouseAction(KEY_W,KeyAction_PRESS) || GetKeyMouseAction(KEY_S,KeyAction_PRESS) || GetKeyMouseAction(KEY_A,KeyAction_PRESS) || GetKeyMouseAction(KEY_D,KeyAction_PRESS))
//		{
//			angle_diff = angle_diff;//移动时不进行变速
//		}
//		else
//		{
//			Variable_Spin(&angle_diff);//变速小陀螺
//		}
		Omnidirectional_Formula(&Expt.Target_Vx,&Expt.Target_Vy);//全向公式	  
		
		ReF_Flag = true;//激活反向跟随标志位
}

void NotfollowMode(void)
{
	angle_diff = -(float)Get_MouseX() * 250.f;
	Expt.Target_Vx *= 0.85;
	Expt.Target_Vy *= 0.85;
}

/**
  * @brief  全向公式
  * @param  void
  * @retval None
  */

void Omnidirectional_Formula(float *Vx, float *Vy)
{
		float RadRaw = 0.0f;
		float temp_Vx = 0.0f;
		float angle = (M6020moto_chassis[0].angle -(GIMBAL_YAW_CENTRE) -400) / M6020_mAngleRatio ;//机械角度偏差 单位为弧度
		RadRaw = angle * DEG_TO_RAD;                           //弧度偏差
		temp_Vx = *Vx;
		*Vx = *Vx * arm_cos_f32(RadRaw) - *Vy * arm_sin_f32(RadRaw);
		*Vy = *Vy * arm_cos_f32(RadRaw) + temp_Vx * arm_sin_f32(RadRaw);
}

/**
  * @brief  全向公式
  * @param  void
  * @retval None
  */

void Omnidirectional_Formula1(float *Vx, float *Vy)
{
		float RadRaw = 0.0f;
		float temp_Vx = 0.0f;
		float angle = (M6020moto_chassis[0].angle -(GIMBAL_YAW_CENTRE )) / M6020_mAngleRatio ;//机械角度偏差 单位为弧度
		RadRaw = angle * DEG_TO_RAD;                           //弧度偏差
		temp_Vx = *Vx;
		*Vx = *Vx * arm_cos_f32(RadRaw) - *Vy * arm_sin_f32(RadRaw);
		*Vy = *Vy * arm_cos_f32(RadRaw) + temp_Vx * arm_sin_f32(RadRaw);
}
/**
 * @brief      底盘电容放电开关
 * @param[in]  state
 * @retval     None
 */
void ChassisCap_Ctrl(uint8_t state)
{
    Cap_switch = state;
}
/**
 * @brief      底盘速度斜坡
 * @param[in]  rec, target, slow_Inc
 * @retval     None
 */
void Chassis_Drv_Slow(float *rec , float target , float slow_Inc)
{
    if(fabs(*rec) - fabs(target) < 0)//加速时
    {
        if(fabs(*rec) > 10)
        {
            slow_Inc = slow_Inc * 5;//速度提起来的时候增大到5倍
        }
    }
    
    if(fabs(*rec) - fabs(target) > 0)
    {
        slow_Inc = slow_Inc * 10.0f;//减速时放大10倍
    }
    if(fabs(*rec - target) < slow_Inc)
    {
        *rec = target;
    }
    else 
    {
        if((*rec) > target) (*rec) -= slow_Inc;
        if((*rec) < target) (*rec) += slow_Inc;
    }	
}

/**
 * @brief      获取裁判系统信息
 * @param[in]  None
 * @retval     Power_Limit
 */
uint16_t CHAS_Max_Get_Power(void)
{
	return Power_Limit;
}

void Limit_Omega(float *target,float Min,float Max);
/**
 * @brief      变速小陀螺处理
 * @param[in]  None
 * @retval     None
 */
void Variable_Spin(float *Omega)
{
 static float Variable_Angle;
 static float Variable_Radin;
 Variable_Angle += 0.1f;
 
 if(Variable_Angle >= 120.0f)
 {
  Variable_Angle = 60.0f;
 }
 Variable_Radin = Variable_Angle * PI / 180;
 float sin_result = arm_sin_f32(Variable_Radin); // 使用arm sin f32()函数计算sin值constrainlsin result, 0.7f. 1.0f) :
 Limit_Omega(&sin_result, 0.7f, 1.0f);
 *Omega *= sin_result; // 将计算得到的值存放到指针的指向位置
}

void Limit_Omega(float *target,float Min,float Max)
{
 if(*target <= Min)
 {
  *target = Min;
 }
 else if(*target >= Max)
 {
  *target = Max;
 }
 else
 {
  *target = *target;
 }
}
