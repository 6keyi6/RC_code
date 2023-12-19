#ifndef _POWER_METER_H_
#define _POWER_METER_H_

#include <stdint.h>
#include "can_control.h"

//typedef struct 
//{
//    float Voltage; //--- ��ѹ
//    float Current; //--- ����
//    float Power;   //--- ����
//}PowerMeter_t;

#define INA226_USART_SIZE 24
#define INA226_CAN_SIZE 8
#define INA226_Self 1

//typedef union 
//{
//	struct
//    {
//        int16_t Power_Val;//����mW
//        int16_t voltageVal;//mV
//        int16_t Shunt_Current;//mA
//        int16_t Shunt_voltage;//uV
//    }Pack;
//	uint8_t data[INA226_CAN_SIZE]; 
//}ina226RecvMsg_u;

typedef union 
{
	struct
    {
        float voltageVal;//mV
        float Shunt_Current;//mA
    }Pack;
	uint8_t data[INA226_CAN_SIZE]; 
}ina226RecvMsg_Self_u;

//extern PowerMeter_t Power_Meter;
//extern ina226RecvMsg_u RecvData;
extern ina226RecvMsg_Self_u Self_RecvData;


/**
  * @brief  ���ʼ����ݸ���
  * @param	can_rx_data
  * @retval None
  */
void Power_Update(uint8_t can_rx_data[]);

/**
  * @brief  ��ȡ���̵��� A
  * @param	None
  * @retval cap data
  */
float PowerMeter_Get_Shunt_Current(void);

/**
  * @brief  ��ȡ���̵�ѹ V
  * @param	None
  * @retval cap data
  */
float PowerMeter_Get_voltageVal(void);

/**
  * @brief  ��ȡ���̹��� W
  * @param	None
  * @retval cap data
  */
float PowerMeter_Get_Power_Val(void);
#endif

