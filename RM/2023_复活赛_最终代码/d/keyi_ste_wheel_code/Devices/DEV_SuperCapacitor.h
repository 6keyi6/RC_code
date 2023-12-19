#ifndef __DEV_SUPERCAPACITOR_H
#define __DEV_SUPERCAPACITOR_H



#include <stdint.h>
#include <stdbool.h>
#include "can_control.h"

#define SCCM_RECEIVE_ID 0x600
#define SCCM_SEND_ID 0x601

#define SupCap_ON 1
#define SupCap_OFF 0
#define Charging_ON 1
#define Charging_OFF 0
#define Power_Supply 1
#define Power_NotSupply 0

#pragma anon_unions
typedef union
{
	uint8_t data[8];
	struct
	{
		float chassis_power;  /* 底盘功率，单位：W */
		uint8_t chassis_buff; /* 底盘功率缓冲 */
		uint8_t cap_usable;   /* 电容可以进行输出 */
		uint8_t cap_cell;     /* 电容剩余电量，会出现负数 */
	};
} SCCM_RecvData_u;

typedef union
{
	uint8_t data[8];
	struct
	{
		float charge_power;    /* 充电功率，单位：W ,范围 0-80W */
		uint8_t charge_enable; /* 充电使能 */
		uint8_t is_cap_output; /* 使用电容供电 */
	};
} SCCM_SendData_u;


extern	SCCM_RecvData_u RecvData;
extern	SCCM_SendData_u SendData;

extern uint8_t State;
extern float Charging_Power;      // 充电的功率


/**
  * @brief  底盘超级电容控制函数
  * @param	void
  * @retval None
  */
void SupCap_Control(void);
/**
  * @brief  电容总状态控制
  * @param	Ctrl Charge Supply
  * @retval None
  */
void SupCap_Main_Switch(bool Ctrl, bool Charge, bool Supply);

uint8_t Get_Cell(void);

/**
  * @brief  获取能否使用超电相关信息
  * @param	None
  * @retval cap data
  */
bool SupCap_Get_Usable(void);
/**
  * @brief  获取是否使用超电相关信息
  * @param	None
  * @retval cap data
  */
bool SupCap_Get_UseSup(void);
/**
  * @brief  超级电容放电开关
  * @param	Charging_Power
  * @retval None
  */
void SupCap_SupplySwitch(bool Switch);


/**
  * @brief  超级电容充电功率
  * @param	Charging_Power
  * @retval None
  */
void SupCap_ChargeControl(float Charging_Power);

/**
  * @brief  超级电容充电开关
  * @param	Charging_Power
  * @retval None
  */
void SupCap_ChargeSwitch(bool Switch);

/**
  * @brief  电容数据发送
  * @param	None
  * @retval None
  */
void SupCap_Send_Msg(CAN_HandleTypeDef *hcan);


/**
  * @brief  电容数据更新
  * @param	can_rx_data
  * @retval None
  */
void SupCap_Update(uint8_t can_rx_data[]);


#endif
