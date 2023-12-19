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
		float chassis_power;  /* ���̹��ʣ���λ��W */
		uint8_t chassis_buff; /* ���̹��ʻ��� */
		uint8_t cap_usable;   /* ���ݿ��Խ������ */
		uint8_t cap_cell;     /* ����ʣ�����������ָ��� */
	};
} SCCM_RecvData_u;

typedef union
{
	uint8_t data[8];
	struct
	{
		float charge_power;    /* ��繦�ʣ���λ��W ,��Χ 0-80W */
		uint8_t charge_enable; /* ���ʹ�� */
		uint8_t is_cap_output; /* ʹ�õ��ݹ��� */
	};
} SCCM_SendData_u;


extern	SCCM_RecvData_u RecvData;
extern	SCCM_SendData_u SendData;

extern uint8_t State;
extern float Charging_Power;      // ���Ĺ���


/**
  * @brief  ���̳������ݿ��ƺ���
  * @param	void
  * @retval None
  */
void SupCap_Control(void);
/**
  * @brief  ������״̬����
  * @param	Ctrl Charge Supply
  * @retval None
  */
void SupCap_Main_Switch(bool Ctrl, bool Charge, bool Supply);

uint8_t Get_Cell(void);

/**
  * @brief  ��ȡ�ܷ�ʹ�ó��������Ϣ
  * @param	None
  * @retval cap data
  */
bool SupCap_Get_Usable(void);
/**
  * @brief  ��ȡ�Ƿ�ʹ�ó��������Ϣ
  * @param	None
  * @retval cap data
  */
bool SupCap_Get_UseSup(void);
/**
  * @brief  �������ݷŵ翪��
  * @param	Charging_Power
  * @retval None
  */
void SupCap_SupplySwitch(bool Switch);


/**
  * @brief  �������ݳ�繦��
  * @param	Charging_Power
  * @retval None
  */
void SupCap_ChargeControl(float Charging_Power);

/**
  * @brief  �������ݳ�翪��
  * @param	Charging_Power
  * @retval None
  */
void SupCap_ChargeSwitch(bool Switch);

/**
  * @brief  �������ݷ���
  * @param	None
  * @retval None
  */
void SupCap_Send_Msg(CAN_HandleTypeDef *hcan);


/**
  * @brief  �������ݸ���
  * @param	can_rx_data
  * @retval None
  */
void SupCap_Update(uint8_t can_rx_data[]);


#endif
