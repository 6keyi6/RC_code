#ifndef _CONTROL_DR16_H_
#define _CONTROL_DR16_H_


#include <stdint.h>
#include "Remote_Control.h"
#define ON 1
#define OFF 0
//ң���������������
typedef struct 
{
	float Target_Yaw,Target_Pit;
	float Target_Vx,Target_Vy,Target_Vw;
}Expt_t;

extern Expt_t Expt;
 /**
 * @brief  	  �����˿���Դת��
 * @param[in]	mode
 * @retval 	  None
 */
 void CtrlSource_Switch(DR16Status_Typedef mode);
 
  /**
 * @brief      RC����ģʽ - ��������Ŀ��ֵ
 * @param[in]  None
 * @retval     None
 */
 void DR16RCCtrl_Update(void);
 
  /**
 * @brief      PC����ģʽ
 * @param[in]  None
 * @retval     None
 */
void DR16PCCtrl_Update(void);

/**
 * @brief      ����ģʽ����
 * @param[in]  None
 * @retval     None
 */
void DR16_LeverMode_Update(void);
 /**
 * @brief      ����Դ����
 * @param[in]  None
 * @retval     None
 */
 void DR16_CTRLSource_Update(void);
 
 
 void Robot_Reset(void);
 
 /**
 * @brief      ��ȡ�����������
 * @param[in]  None
 * @retval     export data
 */
float DR16_Get_ExptYaw(void);
float DR16_Get_ExptPit(void);
float DR16_Get_ExptVx(void);
float DR16_Get_ExptVy(void);
float DR16_Get_ExptVw(void);

extern float YawRCCtrl_Coe;//����
extern uint8_t ResetFlag;   /*<! ������־λ */
extern uint8_t Reflash_UI;  /*<! UIˢ�� */
extern uint8_t Diagonal_Mode;//�Խ�ģʽ��־λ 
extern uint8_t ReF_Flag;     /*<! �������ת����־λ */
extern uint8_t ChassisResetFlag;
#endif
