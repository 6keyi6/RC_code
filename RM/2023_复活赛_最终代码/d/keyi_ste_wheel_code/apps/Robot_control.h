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
#define USE_RM_Referee 1  // ��������ϵͳ

/**********************�¼�********************************/
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
//����ģʽ
typedef enum
{
    AttackMode_Disable,        //ʧ�ܹ���
    AttackMode_AutoMode,       //�Զ���׼
    AttackMode_ManualMode,     //�ֶ���׼
    AttackMode_SingleMode,     //����ģʽ
    AttackMode_ContinuousMode, //����ģʽ
    AttackMode_NoneMode,       //������ģʽ
} AttackMode_e;
typedef enum
{
    ShootTarget_default = 0,     //Ĭ��ģʽ
    ShootTarget_Self_aiming = 1, //����ģʽ
    ShootTarget_BIG_WHEEL,       //���ģʽ
    ShootTarget_Sentry,          //����Է��ڱ���ģʽ
    ShootTarget_Spin,            //��С����ģʽ
    ShootTarget_Video,           //¼��ģʽ
    ShootTarget_UAV,             //�ҷ����˻�����ģʽ
    ShootTarget_OurSentry,       //�ҷ��ڱ�����ģʽ
    ShootTarget_Radar,           //�״�ģʽ

} ShootTarget_e;
typedef struct
{
    //�������
    AttackMode_e Attack_AttackMode;	
    ShootTarget_e Attack_ShootTarget;
	  bool VisionEnabled; //����ʶ�𿪹�
    //�������
    TeamColor_e TeamColor; //�ҷ����Ŷ���ɫ��
    Types_e Types;         //�ҷ�����
} Robot_t;
/**********************�¼�********************************/
//�����˹���ģʽ
typedef struct
{
  int ChassisMode;//���̹���ģʽ
	int Cloud_Mode;//��̨����ģʽ
	int ShootMode;//�������ģʽ	
	int FrictMode;//Ħ����ģʽ
} RobotWorkMode_e; 

typedef struct
{
	float Forward_Back_Value; //Vx
	float Left_Right_Value;   //Vy
	float spin;              //vw
  float YAW_Value;    //��̨YAW������ֵ
	float PICTH_Value;  //��̨PICTH��
	float Dial_Wheel;  //���� ����С���� ���²���
}Robot_TargetValue_t;
//
typedef struct
{
	CHAS_CtrlMode_e  Chassis_PreMode;
}Robot_classdef;

/* ���ͨ���������� */
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
	Vision_Disable = 0,		// �Ӿ�ʧ��	
	Vision_Default = 1,		// Ĭ������(�Ӿ����治��Ԥ��)
	Vision_BIGWindmill = 2, // ��糵
	// Vision_Sentry = 3,	// �ڱ�
	// Vision_BASE = 4,		// ����
	Vision_Top = 5,		// ����
	// Vision_Record = 6	// ¼��
	Vision_Forcast = 6		// �Ӿ�Ԥ��

}VisionMode_e;
extern VisionMode_e Vision_Mode;

extern Robot_TargetValue_t Robot_TargetValue;
extern RobotWorkMode_e RobotWorkMode;
extern Robot_classdef Infantry;

extern Robot_t Robot;
extern uint8_t Shoot_Freq;//�ӵ���Ƶ
/**
  * @brief  �������ܿ���
  * @param  None
  * @retval None
  */
void Robot_control(void);

/**
  * @brief  ����ϵͳ��Ϣ��������̨����
  * @param  can_rx_data
  * @retval None
  */
void RefereeMsg_Send(uint8_t *data);


/**
  * @brief  ����ϵͳ��Ϣ��������̨����  2
  * @param  can_rx_data
  * @retval None
  */
void RefereeMsg_Send_2(uint8_t *data);

/**
 * @brief  	    ����ǹ�����ݼ�����Ƶ
 * @param[in]	None
 * @retval 	    ShootFreq
 */
uint8_t ShootFreq_Calc(void);

uint16_t Get_GunHeat(void);
uint16_t Get_CoolingRate(void);
uint16_t Get_CoolingLimit(void);
#endif
