#include "DEV_SuperCapacitor.h"
#include "Chassis_control.h"
#include "Devices_Monitor.h"
#include "Power_Meter.h"
#include "Robot_control.h"
SCCM_RecvData_u RecvData;
SCCM_SendData_u SendData;
uint8_t State;
float Charging_Power;      // ���Ĺ���

uint8_t charge_flag; //--- ������������־λ
/**
  * @brief  ���̳������ݿ��ƺ���
  * @param	void
  * @retval None
  */
	uint16_t ChassisPower_Limit;	
void SupCap_Control()
{

	//--- ����ʧ�ܻ��������
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
		
	 //---���ܹ��ʡ���ʣ�๦��
	if(SupCap_Get_UseSup() == true && Get_Cell() > 40 /* ����ʣ�����������ָ��� */)
	{
		//--- �߷ű߳䣬�ŵ��ʱ�������ʳ��
		Charging_Power = ChassisPower_Limit;
	}
	else
	{
		if(DevicesGet_State(POWERMETER_MONITOR) == Off_line) //����ϵ�y
		{
				//--- ���ʼƵ�����ʹ�ò���ϵͳ������
			Charging_Power = ChassisPower_Limit - CHAS_Get_Power();
		}
		else//����Ӌ
		{
			Charging_Power = ChassisPower_Limit - PowerMeter_Get_Power_Val();
		}
	}	
	
	//���ܹ����޷�
	if (Charging_Power > ChassisPower_Limit + 5) // �����������
	{
		Charging_Power = ChassisPower_Limit;
	}
	else if(Charging_Power < 0)
	{
		Charging_Power = 0.0f;
	}
	
	//���湦�����Ƴ��ܹ���
	if(CHAS_Get_PowerBuffer() < 60 && CHAS_Get_PowerBuffer() > 55)
	{
		Charging_Power -= 5;
	}
	else if(CHAS_Get_PowerBuffer() <= 55)
	{
		Charging_Power = 0.1f;
	}	
	SupCap_ChargeControl(Charging_Power);

	//--- �����ȳ䵽50�������ٴ�ʹ��
	if(charge_flag == true && Get_Cell() > 50)
	{
		charge_flag = false;
	}

	if(Robot_Receive.Write_Msg[Cap_Ctrl] && Get_Cell() > 40 && SupCap_Get_Usable() != false)
	{
		if(charge_flag == false)
		{
			SupCap_SupplySwitch(Power_Supply); //--- �ŵ�
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

	   //���緢��
     SupCap_Send_Msg(&hcan1);	 
}

/**
  * @brief  ������״̬����
  * @param	Ctrl Charge Supply
  * @retval None
  */
void SupCap_Main_Switch(bool Ctrl, bool Charge, bool Supply)
{
   	State = Ctrl;
	  SendData.charge_enable = Charge;/* ���ʹ�� ----�ر�*/
	  SendData.is_cap_output = Supply;/* ʹ�õ��ݹ��� ----��ʹ��*/
}
/**
  * @brief  ��ȡ���������Ϣ
  * @param	None
  * @retval cap data
  */
uint8_t Get_Cell()
{
	return RecvData.cap_cell;
}

/**
  * @brief  ��ȡ�ܷ�ʹ�ó��������Ϣ
  * @param	None
  * @retval cap data
  */
bool SupCap_Get_Usable()
{
	return RecvData.cap_usable;
}
/**
  * @brief  ��ȡ�Ƿ�ʹ�ó��������Ϣ
  * @param	None
  * @retval cap data
  */
bool SupCap_Get_UseSup()
{
	return SendData.is_cap_output;
}
/**
  * @brief  �������ݷŵ翪��
  * @param	Charging_Power
  * @retval None
  */
void SupCap_SupplySwitch(bool Switch)
{
	SendData.is_cap_output = Switch;
}

/**
  * @brief  �������ݳ�繦��
  * @param	Charging_Power
  * @retval None
  */
void SupCap_ChargeControl(float Charging_Power)
{
	SendData.charge_power = Charging_Power;
}

/**
  * @brief  �������ݳ�翪��
  * @param	Charging_Power
  * @retval None
  */
void SupCap_ChargeSwitch(bool Switch)
{
	SendData.charge_enable = Switch;
}
/**
  * @brief  �������ݷ���
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
  * @brief  �������ݸ���
  * @param	can_rx_data
  * @retval None
  */
void SupCap_Update(uint8_t can_rx_data[])
{
	memcpy(RecvData.data, can_rx_data, 8);
}

