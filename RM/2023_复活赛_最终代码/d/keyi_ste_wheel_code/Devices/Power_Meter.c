#include "Power_Meter.h"

//PowerMeter_t Power_Meter;
//ina226RecvMsg_u RecvData;
ina226RecvMsg_Self_u Self_RecvData;


/**
  * @brief  功率计数据更新
  * @param	can_rx_data
  * @retval None
  */
void Power_Update(uint8_t can_rx_data[])
{
	  memcpy(Self_RecvData.data, can_rx_data, 8);
}


/**
  * @brief  获取底盘电流 A
  * @param	None
  * @retval cap data
  */
float PowerMeter_Get_Shunt_Current()
{
  return (float)Self_RecvData.Pack.Shunt_Current/1000;
}
/**
  * @brief  获取底盘电压 V
  * @param	None
  * @retval cap data
  */
float PowerMeter_Get_voltageVal()
{
  return (float)Self_RecvData.Pack.voltageVal/1000;
}


/**
  * @brief  获取底盘功率 W
  * @param	None
  * @retval cap data
  */
float PowerMeter_Get_Power_Val()
{
  return PowerMeter_Get_Shunt_Current()*PowerMeter_Get_voltageVal();
}
