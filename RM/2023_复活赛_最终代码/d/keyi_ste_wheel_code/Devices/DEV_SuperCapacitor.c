#include "DEV_SuperCapacitor.h"
#include "Chassis_control.h"
#include "Devices_Monitor.h"
#include "Power_Meter.h"
#include "Robot_control.h"
SCCM_RecvData_u RecvData;
SCCM_SendData_u SendData;
uint8_t State;
float Charging_Power;      // 充电的功率

uint8_t charge_flag; //--- 超电用完后充电标志位
/**
  * @brief  底盘超级电容控制函数
  * @param	void
  * @retval None
  */
	uint16_t ChassisPower_Limit;	
void SupCap_Control()
{

	//--- 底盘失能或电容离线
	if(Robot_Receive.chassismode == CHAS_DisableMode || DevicesGet_State(SUPCAP_MONITOR) == Off_line)
	{
		State = SupCap_OFF;
		SupCap_Main_Switch(SupCap_OFF, Charging_OFF, Power_NotSupply);
		return;
	}
	else
	{
		State = SupCap_ON;
		SendData.charge_enable = SupCap_ON;
	}	


	  if(CHAS_Max_Get_Power() < 45)
    {
        ChassisPower_Limit = 45;
    }
    else if (CHAS_Max_Get_Power() >= 120)
    {
        ChassisPower_Limit = 120;
    }
    else
    {
        ChassisPower_Limit = CHAS_Max_Get_Power();
    }		
		
	 //---充能功率——剩余功率
	if(SupCap_Get_UseSup() == true && Get_Cell() > 40 /* 电容剩余电量，会出现负数 */)
	{
		//--- 边放边充，放电的时候满功率充电
		Charging_Power = ChassisPower_Limit;
	}
	else
	{
		if(DevicesGet_State(POWERMETER_MONITOR) == Off_line) //裁判系統
		{
				//--- 功率计掉线则使用裁判系统的数据
			Charging_Power = ChassisPower_Limit - CHAS_Get_Power();
		}
		else//功率計
		{
			Charging_Power = ChassisPower_Limit - PowerMeter_Get_Power_Val();
		}
	}	
	
	//充能功率限幅
	if (Charging_Power > ChassisPower_Limit + 5) // 底盘输出功率
	{
		Charging_Power = ChassisPower_Limit;
	}
	else if(Charging_Power < 0)
	{
		Charging_Power = 0.0f;
	}
	
	//缓存功率限制充能功率
	if(CHAS_Get_PowerBuffer() < 60 && CHAS_Get_PowerBuffer() > 55)
	{
		Charging_Power -= 5;
	}
	else if(CHAS_Get_PowerBuffer() <= 55)
	{
		Charging_Power = 0.1f;
	}	
	SupCap_ChargeControl(Charging_Power);

	//--- 用完后等充到50再允许再次使用
	if(charge_flag == true && Get_Cell() > 50)
	{
		charge_flag = false;
	}

	if(Robot_Receive.Write_Msg[Cap_Ctrl] && Get_Cell() > 40 && SupCap_Get_Usable() != false)
	{
		if(charge_flag == false)
		{
			SupCap_SupplySwitch(Power_Supply); //--- 放电
		}
	}
	else
	{
		if(Get_Cell() <= 40)
		{
			charge_flag = true;
		}
		SupCap_SupplySwitch(Power_NotSupply);
	}	

	   //超电发送
     SupCap_Send_Msg(&hcan1);	 
}

/**
  * @brief  电容总状态控制
  * @param	Ctrl Charge Supply
  * @retval None
  */
void SupCap_Main_Switch(bool Ctrl, bool Charge, bool Supply)
{
   	State = Ctrl;
	  SendData.charge_enable = Charge;/* 充电使能 ----关闭*/
	  SendData.is_cap_output = Supply;/* 使用电容供电 ----不使用*/
}
/**
  * @brief  获取电容相关信息
  * @param	None
  * @retval cap data
  */
uint8_t Get_Cell()
{
	return RecvData.cap_cell;
}

/**
  * @brief  获取能否使用超电相关信息
  * @param	None
  * @retval cap data
  */
bool SupCap_Get_Usable()
{
	return RecvData.cap_usable;
}
/**
  * @brief  获取是否使用超电相关信息
  * @param	None
  * @retval cap data
  */
bool SupCap_Get_UseSup()
{
	return SendData.is_cap_output;
}
/**
  * @brief  超级电容放电开关
  * @param	Charging_Power
  * @retval None
  */
void SupCap_SupplySwitch(bool Switch)
{
	SendData.is_cap_output = Switch;
}

/**
  * @brief  超级电容充电功率
  * @param	Charging_Power
  * @retval None
  */
void SupCap_ChargeControl(float Charging_Power)
{
	SendData.charge_power = Charging_Power;
}

/**
  * @brief  超级电容充电开关
  * @param	Charging_Power
  * @retval None
  */
void SupCap_ChargeSwitch(bool Switch)
{
	SendData.charge_enable = Switch;
}
/**
  * @brief  电容数据发送
  * @param	None
  * @retval None
  */
void SupCap_Send_Msg(CAN_HandleTypeDef *hcan)
{
	uint8_t Data[8];
	memcpy(Data, SendData.data, 8);
	CAN_Sup_senddata(hcan, SCCM_SEND_ID, Data, 8);
}

/**
  * @brief  电容数据更新
  * @param	can_rx_data
  * @retval None
  */
void SupCap_Update(uint8_t can_rx_data[])
{
	memcpy(RecvData.data, can_rx_data, 8);
}

