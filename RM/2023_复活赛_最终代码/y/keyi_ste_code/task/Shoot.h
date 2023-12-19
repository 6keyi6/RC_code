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

/*���̲���*/
typedef struct
{
	uint8_t Began;		    // ��ʼ����
	uint8_t Stuck_Flag;		// ������־λ
	int8_t Motor_Direction;	// ���ת������  1 -> ��  | -1 -> ��
	int16_t Stuck_Duration;	// ����������ʱ��
    uint16_t Num;           // ����װ����
    uint16_t CanShoot_Num;  // �������/
	uint16_t CriticalVal;	// ���̵Ľ����ٶ�ֵ->�����ٽ�ֵ
	int32_t Init_Angle;	// ��ʼλ��
	int32_t Launched_Num;	// �Ѿ�������ӵ���
    int32_t Target_Angle;   // ��������Ŀ��ֵ
	int32_t Re_TargetAngle; // ���������תĿ��ֵ
} Reload_t;
typedef enum 
{
	Fric_Disable, // �ر�Ħ����
	Fric_Unload,  // �˵�
	Fric_15m_s,	  // 15m/s
	Fric_18m_s,	  // 18m/s
	Fric_22m_s,   // 22m/s
	Fric_30m_s    // 30m/s
}Fric_CtrlMode_e;
/**
  * @brief  ����ܿ���
  * @param  void
  * @retval None
  */ 
void Shoot_control(void);
/**
  * @brief  ����PID��ʼ��
  * @param  void
  * @retval None
  */
void Dial_PID_Init(void);
/**
  * @brief  Ħ����PID��ʼ��
  * @param  void
  * @retval None
  */
void Frict_PID_Init(void);
/**
  * @brief  Ħ���ִ���
  * @param  void
  * @retval None
  */
void Fric_processing(void);
/**
 * @brief  	   Ħ����ת�ٵ��� (Test ing)
 * @param[in]  ǹ�ܷ����ٶ� �������� �ݼ�ֵ ����ֵ
 * @note       ��ת�ٵ����Ļ����ϼ����¿ش�����Ϊ�������Է����¶�Խ������ҲԽ��
 * @note       ���Ȳ�15 18 30��22�����ò���
 * @retval 	   None
 */
void FricSpeed_Adapt(void);
/**
  * @brief  ����PID����
  * @param  void
  * @retval None
  */
void Dial_PID_calc(void);
/**
 * @brief  	   ��������
 * @param[in]  None
 * @retval 	   None
 */
void Reload_Reset(void);
/**
 * @brief  	   ���õ���װ����
 * @param[in]  num
 * @retval 	   None 
 */
void Set_ReloadNum(uint16_t num);
/**
 * @brief  	   �жϲ��̵���Ƿ��ڿ����ٽ�ֵ
 * @param[in]  None
 * @retval 	   state
 */
bool Judge_ReloadStall(void);
/**
 * @brief  	   ���ģʽ����
 * @param[in]  None
 * @retval 	   None
 */
void AttackMode_Ctrl(void);
/**
 * @brief  	   �������
 * @param[in]  ����
 * @retval 	   None
 */
void Mag_Ctrl(uint8_t state);
/**
 * @brief  	   ��ȡ����ϵͳ�����Ϣ
 * @param[in]  None
 * @retval 	   shoot power state
 */
uint16_t ShootGet_SpeedLimit(void);
float Get_GunSpeed(void);
uint16_t Get_GunHeat(void);
uint16_t Get_CoolingRate(void);
uint16_t Get_CoolingLimit(void);

extern Reload_t Reload;          /*<! ���̲��� */
extern Fric_CtrlMode_e Fric_Mode;
extern bool whether_Fire;
extern incrementalpid_t FrictL_PID;
extern incrementalpid_t FrictR_PID;
extern uint8_t ContLaunch;       /*<! ������־λ */
extern WorldTime_RxTypedef shootUnit_WorldTime;
extern int needShoot ;                     //��Ҫ����Ŀ���
extern bool Mag_Switch;          /*<! ���ֿ��ر�־λ */
extern uint16_t Shoot_Freq;        /*<! ��Ƶ */
extern uint8_t Rage_Mode;        /*<! ��ģʽ,������������ */

#endif
