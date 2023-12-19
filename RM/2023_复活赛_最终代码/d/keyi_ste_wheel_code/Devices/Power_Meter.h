#ifndef _POWER_METER_H_
#define _POWER_METER_H_

#include <stdint.h>
#include "can_control.h"

//typedef struct 
//{
//    float Voltage; //--- 电压
//    float Current; //--- 电流
//    float Power;   //--- 功率
//}PowerMeter_t;

#define INA226_USART_SIZE 24
#define INA226_CAN_SIZE 8
#define INA226_Self 1

//typedef union 
//{
//	struct
//    {
//        int16_t Power_Val;//功率mW
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
  * @brief  功率计数据更新
  * @param	can_rx_data
  * @retval None
  */
void Power_Update(uint8_t can_rx_data[]);

/**
  * @brief  获取底盘电流 A
  * @param	None
  * @retval cap data
  */
float PowerMeter_Get_Shunt_Current(void);

/**
  * @brief  获取底盘电压 V
  * @param	None
  * @retval cap data
  */
float PowerMeter_Get_voltageVal(void);

/**
  * @brief  获取底盘功率 W
  * @param	None
  * @retval cap data
  */
float PowerMeter_Get_Power_Val(void);
#endif

