#ifndef _CONTROL_DR16_H_
#define _CONTROL_DR16_H_


#include <stdint.h>
#include "Remote_Control.h"
#define ON 1
#define OFF 0
//遥控器对外输出参数
typedef struct 
{
	float Target_Yaw,Target_Pit;
	float Target_Vx,Target_Vy,Target_Vw;
}Expt_t;

extern Expt_t Expt;
 /**
 * @brief  	  机器人控制源转换
 * @param[in]	mode
 * @retval 	  None
 */
 void CtrlSource_Switch(DR16Status_Typedef mode);
 
  /**
 * @brief      RC控制模式 - 对外设置目标值
 * @param[in]  None
 * @retval     None
 */
 void DR16RCCtrl_Update(void);
 
  /**
 * @brief      PC控制模式
 * @param[in]  None
 * @retval     None
 */
void DR16PCCtrl_Update(void);

/**
 * @brief      拨杆模式更新
 * @param[in]  None
 * @retval     None
 */
void DR16_LeverMode_Update(void);
 /**
 * @brief      控制源更新
 * @param[in]  None
 * @retval     None
 */
 void DR16_CTRLSource_Update(void);
 
 
 void Robot_Reset(void);
 
 /**
 * @brief      获取对外输出数据
 * @param[in]  None
 * @retval     export data
 */
float DR16_Get_ExptYaw(void);
float DR16_Get_ExptPit(void);
float DR16_Get_ExptVx(void);
float DR16_Get_ExptVy(void);
float DR16_Get_ExptVw(void);

extern float YawRCCtrl_Coe;//底盘
extern uint8_t ResetFlag;   /*<! 重启标志位 */
extern uint8_t Reflash_UI;  /*<! UI刷新 */
extern uint8_t Diagonal_Mode;//对角模式标志位 
extern uint8_t ReF_Flag;     /*<! 反向跟随转换标志位 */
extern uint8_t ChassisResetFlag;
#endif
