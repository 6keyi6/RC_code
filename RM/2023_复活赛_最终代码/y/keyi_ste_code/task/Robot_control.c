/**
 * @file Robot_control.c
 * @author keyi (hzjqq66@163.com)
 * @brief 
 * @version 1.1
 * @date 2022-09-9
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Robot_control.h"
#include "Remote_Control.h"
#include "can_control.h"
#include "Chassis_control.h"
#include "Devices_Monitor.h"
#include "shoot.h"
#include "DR16_control.h"
#include "WS2812.h"
Robot_t Robot;
CHAS_CtrlMode_e Chassis_Mode;
Gimbal_CtrlMode_e Gimbal_Mode;
Attack_CtrlMode_e Attack_Mode;
VisionMode_e Vision_Mode;
/**
  * @brief  �������ܿ���
  * @param  None
  * @retval None
  */
uint8_t SendData[8]; /*<! ���ͨ�Ŵ洢���� */
uint8_t Chassis_start = false;
void Robot_control(void)
{
	 ws2812_init(8);
	 ws2812_contril();

	  Shoot_control();//�������
		Cloud_control();//��̨����
		Chassis_control();//���̿���
			
		if(DevicesGet_State(DR16_MONITOR) != Off_line)//ң��������ʱ���������ݸ�����
		{
				SendMsgtoCHAS();//�������ݸ�����
		}
		else
		{
			
		}
}

/**
  * @brief  ������������������
  * @param  void
  * @retval None
  */
float tempSendData_t[8] = {0};
float PCtempSendData_t[8] = {0};
void SendMsgtoCHAS(void)
{

		  int16_t  tempSendData[8] = {0};

							//--- �ϲ����ؼ�����ĵ����ٶ�
			tempSendData[0] = (int16_t)Expt.Target_Vx >> 8;
			tempSendData[1] = (int16_t)Expt.Target_Vx;

			tempSendData[2] = (int16_t)Expt.Target_Vy >> 8;
			tempSendData[3] = (int16_t)Expt.Target_Vy;

			tempSendData[4] = (int16_t)Expt.Target_Vw >> 8;
			tempSendData[5] = (int16_t)Expt.Target_Vw;	
			
	    tempSendData[6] = 0;
			tempSendData[7] = 0;			
		

			//--- ����ģʽ(bit 0-3)
      tempSendData[6] |= Chassis_Mode&0x0F;
      //--- �Ӿ�ģʽ(bit 4-6)
      tempSendData[6] |= (Vision_Mode&0x07)<<4;
			
        //--- ģ����״̬	
      tempSendData[7] |= (DevicesGet_State(VISION_MONITOR) == Off_line?1<<0:0);	
      tempSendData[7]	|= ((DevicesGet_State(FRIC_L_MONITOR)||DevicesGet_State(FRIC_R_MONITOR)||DevicesGet_State(RELOAD_MONITOR)) == Off_line?1<<1:0);
//	    //--- ����ʧ��
      tempSendData[7]	|= ChassisResetFlag == true?1<<2:0;
        //--- �����˶Խ�ģʽ�Ƿ�����
      tempSendData[7] |= Diagonal_Mode == true?1<<3:0;
			//--- ���ݿ���
			tempSendData[7] |= Cap_switch==ON?1<<4:0;
			//--- Ħ����״̬
			tempSendData[7] |= Attack_Mode==Attack_Disable?0:1<<5;
			//--- ���ֿ���
			tempSendData[7] |= Mag_Switch==OFF?0:1<<6;		
      //--- UIˢ��
      tempSendData[7] |= Reflash_UI<<7;
		//--- �ϲ����ؼ�����ĵ����ٶ�
		for(int i = 0;i < 8;i++)
		{
			SendData[i] = tempSendData[i];
		}
		CAN_Senddata(&hcan1, 0x340, SendData, 8);
}

/**
 * @brief      д��ӵ������ؽ��յĲ���ϵͳ����
 * @param[in]  id, can_rx_data
 * @retval     None
 */
Shoot_t Shoot;
float Power_Limit;//--- ���̹�������
uint8_t ID;//������ID
float cooling_limit;//��������
float cooling_heat;//ǹ������
float cooling_rate;//��ȴ����
float cap_call;//�������
void RefereeMsg_Write(uint8_t can_rx_data[])
{
		uint8_t temp[4];
		float *speed;       //--- �ӵ����ٶ�	
	
		cooling_limit = can_rx_data[0];
		cooling_heat = can_rx_data[1];
		cap_call = can_rx_data[2];//�������
		temp[3] = can_rx_data[3];	
//	speed = (float*)(&temp);
		Shoot.BulletSpeed = *speed;  //������ٶ�

		ID = can_rx_data[4]&0x7F;//--- ������ID
		Shoot.PowerState = 1&can_rx_data[4]>>7; //---�����Դ״̬
		Shoot.SpeedLimit = can_rx_data[5];  //--- �ӵ����ٶ�����
		Shoot.Shoot_Freq = can_rx_data[6];  //--- ������Ƶ
		Power_Limit = can_rx_data[7];   //--- ���̹�������	
	


		if(Shoot.Shoot_Freq > 20)
		{
				Shoot.Shoot_Freq = 20;
		}
		Shoot_Freq = Shoot.Shoot_Freq;
}


///**
// * @brief      д��ӵ������ؽ��յĲ���ϵͳ����
// * @param[in]  id, can_rx_data
// * @retval     None
// */

//void RefereeMsg_Write_1(uint8_t can_rx_data[])
//{
//		cooling_limit = can_rx_data[0];////��������
//		cooling_heat = can_rx_data[1];////ǹ������
//	  cooling_rate =can_rx_data[2];////��ȴ����
//}


/**
 * @brief  	  ������̨����ģʽ
 * @param[in]	mode
 * @retval 	  None
 */
void Set_GimbalMode(Gimbal_CtrlMode_e mode)
{
	Gimbal_Mode = mode;
}
/**
 * @brief  	  ���õ�������ģʽ
 * @param[in]	mode
 * @retval 	  None
 */
void Set_ChassisMode(CHAS_CtrlMode_e mode)
{
	Chassis_Mode = mode;
}
/**
 * @brief  	    ���÷�������ģʽ
 * @param[in]	mode
 * @retval 	    None
 */
void Set_AttackMode(Attack_CtrlMode_e mode)
{

//    if(Attack_Mode == Attack_Auto && mode != Attack_Auto)//������
//    {
//        //--- ���Զ����ģʽ��ȡ��ʱ�������Զ����������
//        Set_ReloadNum(0);
//    }
//		
	Attack_Mode = mode;
	//����ģʽѡ������
	switch(Attack_Mode)
	{
    case Attack_30:
				 Fric_Mode = Fric_30m_s;
				 break;
	  case Attack_18:
				 Fric_Mode = Fric_18m_s;
				 break;
    case Attack_22:
         Fric_Mode = Fric_22m_s;
         break;
	  case Attack_15:
				 Fric_Mode = Fric_15m_s;
				 break;
 
	case Attack_Auto:
		switch (ShootGet_SpeedLimit())
		{
		case 15:
			Fric_Mode = Fric_15m_s; // --- 15m/s
			break;
		case 18:
			Fric_Mode = Fric_18m_s; // --- 18m/s
			break;
        case 22:
        case 25:
            Fric_Mode = Fric_22m_s; // --- 22m/s
            break;
		case 30:
        case 75:
			Fric_Mode = Fric_30m_s; // --- 30m/s
			break;

		default:
			Fric_Mode = Fric_15m_s; // --- 15m/s
			break;
		}
		break;

	case Attack_Unload:
		Fric_Mode = Fric_Unload;
		break;

	case Attack_Disable:
		Fric_Mode = Fric_Disable;
//		Shoot.Rage_Mode = false;
		break;

	default:
		Fric_Mode = Fric_Disable;
//        Shoot.Rage_Mode = false;
		break;
		 
	}
}
/**
 * @brief  	    �����Ӿ�����ģʽ
 * @param[in]	mode
 * @retval 	    None
 */
void Set_VisionMode(VisionMode_e mode)
{
	Vision_Mode = mode;
	//.............
}

/**
 * @brief  	  ������ʧ��,״̬��λ
 * @param[in]	None
 * @retval 	  None
 */
void Robot_Disable(void)
{
    Set_ChassisMode(CHAS_DisableMode);
    Set_GimbalMode(Gimbal_DisableMode);
    Set_AttackMode(Attack_Disable);
    Set_VisionMode(Vision_Forcast);	
	  
}
	
