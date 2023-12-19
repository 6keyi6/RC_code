#ifndef __Robot_CONTROL_H
#define __Robot_CONTROL_H
#include "Chassis_control.h"
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
extern CHAS_CtrlMode_e Chassis_Mode;

void Robot_control(void);
#endif




