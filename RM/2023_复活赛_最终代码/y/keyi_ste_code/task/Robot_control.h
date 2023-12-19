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

#define USE_RM_Referee    1 // ��������ϵͳ



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
////�����˹���ģʽ
//typedef struct
//{
//  int ChassisMode;//���̹���ģʽ
//	int Cloud_Mode;//��̨����ģʽ
//	int ShootMode;//�������ģʽ	
//	int FrictMode;//Ħ����ģʽ
//} RobotWorkMode_e; 
/* --- �¼� -----------------------------------------------------------*/
/* --- ��̨����ģʽ -----------------------------------------------------------*/
typedef enum 
{
	Gimbal_DisableMode,	// ʧ��
    Gimbal_NormalMode,	// ��ͨ��תģʽ
	Gimbal_SupplyMode,  // ����ģʽ
    Gimbal_LockMode,    // ��סģʽ
	Gimbal_PCMode       // PC����(����)
}Gimbal_CtrlMode_e;
/* --- ���̿���ģʽ ------------------------------------------------------------*/
typedef enum 
{
	CHAS_DisableMode,	    // ʧ��ģʽ
    CHAS_followMode ,	    // ����ģʽ
	CHAS_ReFollowMode,      // �������
	CHAS_SpinMode ,	        // С����ģʽ
    CHAS_sinSpinMode,       // ����С����
	CHAS_AutoTrackMode,     // �Զ�׷��ģʽ
	CHAS_LockMode,		    // ��ס����
	CHAS_NotfollowMode,	    // �޸���ģʽ

	CHAS_SupplyMode,	    // ����ģʽ
	CHAS_SwingMode,		    // Ť��ģʽ
	CHAS_InitMode,          // ��ʼ���׶�
}CHAS_CtrlMode_e;

typedef enum 
{
    Attack_Disable,		//ʧ�ܹ���
	Attack_15,	        //�ֶ���׼ - ����
	Attack_18,	        //�ֶ���׼ - ����
	Attack_22,	        //�ֶ���׼ - �и���
	Attack_30,	        //�ֶ���׼ - ����
	Attack_BigWindmill, //���ģʽ
	Attack_Auto,		//�Զ����
	Attack_Unload		//�˵�
}Attack_CtrlMode_e;
typedef enum 
{
	Vision_Disable = 0,		// �Ӿ�ʧ��	
	Vision_Default = 1,		// Ĭ������(�Ӿ����治��Ԥ��)
	Vision_BIGWindmill = 2, // ��糵
	// Vision_Sentry = 3,	// �ڱ�
	// Vision_BASE = 4,		// ����
	Vision_Top = 5,		// ����
	// Vision_Record = 6	// ¼��
	Vision_Forcast = 6,		// �Ӿ�Ԥ��
  Vision_Outpost = 7  //ǰ��ս
}VisionMode_e;

extern CHAS_CtrlMode_e Chassis_Mode;
extern Gimbal_CtrlMode_e Gimbal_Mode;
extern Attack_CtrlMode_e Attack_Mode;
extern VisionMode_e Vision_Mode;
/**
 * @brief  	  ���õ�������ģʽ
 * @param[in]	mode
 * @retval 	  None
 */
void Set_ChassisMode(CHAS_CtrlMode_e mode);
/**
 * @brief  	  ������̨����ģʽ
 * @param[in]	mode
 * @retval 	  None
 */
void Set_GimbalMode(Gimbal_CtrlMode_e mode);
/**
 * @brief  	    ���÷�������ģʽ
 * @param[in]	mode
 * @retval 	    None
 */
void Set_AttackMode(Attack_CtrlMode_e mode);
/**
 * @brief  	    �����Ӿ�����ģʽ
 * @param[in]	mode
 * @retval 	    None
 */
void Set_VisionMode(VisionMode_e mode);


/**
 * @brief  	  ������ʧ��,״̬��λ
 * @param[in]	None
 * @retval 	  None
 */
void Robot_Disable(void);
/**
 * @brief  	    ��ȡ�����������Ϣ
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
		float BulletSpeed;//--- ������ٶ�
		float PowerState;//--- �����Դ״̬
		float SpeedLimit; //--- �ӵ����ٶ�����
		float Shoot_Freq; //--- ������Ƶ
}Shoot_t;

extern Shoot_t Shoot;

extern uint8_t SendData[8]; /*<! ���ͨ�Ŵ洢���� */
extern Robot_t Robot;

extern float cooling_limit;
extern float cooling_heat;
extern float cooling_rate;
extern uint8_t ID;//������ID
extern uint16_t Shoot_Freq;        /*<! ��Ƶ */
extern uint8_t Chassis_start;
extern float Power_Limit;//--- ���̹�������
extern float cap_call;
/**
  * @brief  �������ܿ���
  * @param  None
  * @retval None
  */
void Robot_control(void);

void SendMsgtoCHAS(void);
/**
 * @brief      д��ӵ������ؽ��յĲ���ϵͳ����
 * @param[in]  id, can_rx_data
 * @retval     None
 */
void RefereeMsg_Write(uint8_t can_rx_data[]);

/**
 * @brief      д��ӵ������ؽ��յĲ���ϵͳ����
 * @param[in]  id, can_rx_data
 * @retval     None
 */
void RefereeMsg_Write_1(uint8_t can_rx_data[]);

#endif
