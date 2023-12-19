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
//#define GIMBAL_YAW_CENTRE	700    	// Yaw ���ĵ� 22���������
//#define GIMBAL_YAW_CENTRE	651    	// Yaw ���ĵ� 23�����������
#define GIMBAL_YAW_CENTRE	7522    	// Yaw ���ĵ� 23���������
float angle_diff;//��ԽǶȲ�
uint8_t Cap_switch;   /*<! ����������� */
float VwLimt;
//���ݵ���ģʽ ѡ��ͬ��Vw��Ϊ������

uint8_t ReF_Flag = false;     /*<! �������ת����־λ */
int16_t ReF_Cnt;      /*<! ��������ۼ�ʱ�� */

void Chassis_control()
{
	 	if(Gimbal_Mode == Gimbal_DisableMode)//��̨ʧ��
	  { 
			Chassis_Mode = CHAS_DisableMode;
		  return;
	  }			

		if(Chassis_Mode == CHAS_DisableMode)//ʧ��
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
	static uint8_t done_cnt; // ������ɼ�ʱ
	
// ����������----------------------------------------------------------
    if(ReF_Flag == true)
    {
			ReF_Cnt++;
			Omnidirectional_Formula1(&Expt.Target_Vx,&Expt.Target_Vy);//ȫ��ʽ
			if (fabs(M6020moto_chassis[0].angle - GIMBAL_YAW_CENTRE) <= 100.0f)
			{
					if(++done_cnt > 50)
					{
							ReF_Flag = false;
							done_cnt = 0;
					}
			}
			if (ReF_Cnt > 1000 && ReF_Flag != false) // --- ��ʱδת����ֱ��ת��
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
				 Omnidirectional_Formula1(&Expt.Target_Vx,&Expt.Target_Vy);//ȫ��ʽ
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
//			angle_diff = angle_diff;//�ƶ�ʱ�����б���
//		}
//		else
//		{
//			Variable_Spin(&angle_diff);//����С����
//		}
		Omnidirectional_Formula(&Expt.Target_Vx,&Expt.Target_Vy);//ȫ��ʽ	  
		
		ReF_Flag = true;//���������־λ
}

void NotfollowMode(void)
{
	angle_diff = -(float)Get_MouseX() * 250.f;
	Expt.Target_Vx *= 0.85;
	Expt.Target_Vy *= 0.85;
}

/**
  * @brief  ȫ��ʽ
  * @param  void
  * @retval None
  */

void Omnidirectional_Formula(float *Vx, float *Vy)
{
		float RadRaw = 0.0f;
		float temp_Vx = 0.0f;
		float angle = (M6020moto_chassis[0].angle -(GIMBAL_YAW_CENTRE) -400) / M6020_mAngleRatio ;//��е�Ƕ�ƫ�� ��λΪ����
		RadRaw = angle * DEG_TO_RAD;                           //����ƫ��
		temp_Vx = *Vx;
		*Vx = *Vx * arm_cos_f32(RadRaw) - *Vy * arm_sin_f32(RadRaw);
		*Vy = *Vy * arm_cos_f32(RadRaw) + temp_Vx * arm_sin_f32(RadRaw);
}

/**
  * @brief  ȫ��ʽ
  * @param  void
  * @retval None
  */

void Omnidirectional_Formula1(float *Vx, float *Vy)
{
		float RadRaw = 0.0f;
		float temp_Vx = 0.0f;
		float angle = (M6020moto_chassis[0].angle -(GIMBAL_YAW_CENTRE )) / M6020_mAngleRatio ;//��е�Ƕ�ƫ�� ��λΪ����
		RadRaw = angle * DEG_TO_RAD;                           //����ƫ��
		temp_Vx = *Vx;
		*Vx = *Vx * arm_cos_f32(RadRaw) - *Vy * arm_sin_f32(RadRaw);
		*Vy = *Vy * arm_cos_f32(RadRaw) + temp_Vx * arm_sin_f32(RadRaw);
}
/**
 * @brief      ���̵��ݷŵ翪��
 * @param[in]  state
 * @retval     None
 */
void ChassisCap_Ctrl(uint8_t state)
{
    Cap_switch = state;
}
/**
 * @brief      �����ٶ�б��
 * @param[in]  rec, target, slow_Inc
 * @retval     None
 */
void Chassis_Drv_Slow(float *rec , float target , float slow_Inc)
{
    if(fabs(*rec) - fabs(target) < 0)//����ʱ
    {
        if(fabs(*rec) > 10)
        {
            slow_Inc = slow_Inc * 5;//�ٶ���������ʱ������5��
        }
    }
    
    if(fabs(*rec) - fabs(target) > 0)
    {
        slow_Inc = slow_Inc * 10.0f;//����ʱ�Ŵ�10��
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
 * @brief      ��ȡ����ϵͳ��Ϣ
 * @param[in]  None
 * @retval     Power_Limit
 */
uint16_t CHAS_Max_Get_Power(void)
{
	return Power_Limit;
}

void Limit_Omega(float *target,float Min,float Max);
/**
 * @brief      ����С���ݴ���
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
 float sin_result = arm_sin_f32(Variable_Radin); // ʹ��arm sin f32()��������sinֵconstrainlsin result, 0.7f. 1.0f) :
 Limit_Omega(&sin_result, 0.7f, 1.0f);
 *Omega *= sin_result; // ������õ���ֵ��ŵ�ָ���ָ��λ��
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
