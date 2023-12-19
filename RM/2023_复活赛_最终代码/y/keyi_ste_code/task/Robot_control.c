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
  * @brief  机器人总控制
  * @param  None
  * @retval None
  */
uint8_t SendData[8]; /*<! 板间通信存储数据 */
uint8_t Chassis_start = false;
void Robot_control(void)
{
	 ws2812_init(8);
	 ws2812_contril();

	  Shoot_control();//射击控制
		Cloud_control();//云台控制
		Chassis_control();//底盘控制
			
		if(DevicesGet_State(DR16_MONITOR) != Off_line)//遥控器离线时不发送数据给底盘
		{
				SendMsgtoCHAS();//发送数据给底盘
		}
		else
		{
			
		}
}

/**
  * @brief  发送数据至底盘主控
  * @param  void
  * @retval None
  */
float tempSendData_t[8] = {0};
float PCtempSendData_t[8] = {0};
void SendMsgtoCHAS(void)
{

		  int16_t  tempSendData[8] = {0};

							//--- 上层主控计算出的底盘速度
			tempSendData[0] = (int16_t)Expt.Target_Vx >> 8;
			tempSendData[1] = (int16_t)Expt.Target_Vx;

			tempSendData[2] = (int16_t)Expt.Target_Vy >> 8;
			tempSendData[3] = (int16_t)Expt.Target_Vy;

			tempSendData[4] = (int16_t)Expt.Target_Vw >> 8;
			tempSendData[5] = (int16_t)Expt.Target_Vw;	
			
	    tempSendData[6] = 0;
			tempSendData[7] = 0;			
		

			//--- 底盘模式(bit 0-3)
      tempSendData[6] |= Chassis_Mode&0x0F;
      //--- 视觉模式(bit 4-6)
      tempSendData[6] |= (Vision_Mode&0x07)<<4;
			
        //--- 模块检测状态	
      tempSendData[7] |= (DevicesGet_State(VISION_MONITOR) == Off_line?1<<0:0);	
      tempSendData[7]	|= ((DevicesGet_State(FRIC_L_MONITOR)||DevicesGet_State(FRIC_R_MONITOR)||DevicesGet_State(RELOAD_MONITOR)) == Off_line?1<<1:0);
//	    //--- 底盘失能
      tempSendData[7]	|= ChassisResetFlag == true?1<<2:0;
        //--- 机器人对角模式是否启用
      tempSendData[7] |= Diagonal_Mode == true?1<<3:0;
			//--- 电容开启
			tempSendData[7] |= Cap_switch==ON?1<<4:0;
			//--- 摩擦轮状态
			tempSendData[7] |= Attack_Mode==Attack_Disable?0:1<<5;
			//--- 弹仓开关
			tempSendData[7] |= Mag_Switch==OFF?0:1<<6;		
      //--- UI刷新
      tempSendData[7] |= Reflash_UI<<7;
		//--- 上层主控计算出的底盘速度
		for(int i = 0;i < 8;i++)
		{
			SendData[i] = tempSendData[i];
		}
		CAN_Senddata(&hcan1, 0x340, SendData, 8);
}

/**
 * @brief      写入从底盘主控接收的裁判系统数据
 * @param[in]  id, can_rx_data
 * @retval     None
 */
Shoot_t Shoot;
float Power_Limit;//--- 底盘功率限制
uint8_t ID;//机器人ID
float cooling_limit;//热量限制
float cooling_heat;//枪管热量
float cooling_rate;//冷却速率
float cap_call;//超电电量
void RefereeMsg_Write(uint8_t can_rx_data[])
{
		uint8_t temp[4];
		float *speed;       //--- 子弹初速度	
	
		cooling_limit = can_rx_data[0];
		cooling_heat = can_rx_data[1];
		cap_call = can_rx_data[2];//超电电量
		temp[3] = can_rx_data[3];	
//	speed = (float*)(&temp);
		Shoot.BulletSpeed = *speed;  //射击初速度

		ID = can_rx_data[4]&0x7F;//--- 机器人ID
		Shoot.PowerState = 1&can_rx_data[4]>>7; //---发射电源状态
		Shoot.SpeedLimit = can_rx_data[5];  //--- 子弹初速度上限
		Shoot.Shoot_Freq = can_rx_data[6];  //--- 发射射频
		Power_Limit = can_rx_data[7];   //--- 底盘功率限制	
	


		if(Shoot.Shoot_Freq > 20)
		{
				Shoot.Shoot_Freq = 20;
		}
		Shoot_Freq = Shoot.Shoot_Freq;
}


///**
// * @brief      写入从底盘主控接收的裁判系统数据
// * @param[in]  id, can_rx_data
// * @retval     None
// */

//void RefereeMsg_Write_1(uint8_t can_rx_data[])
//{
//		cooling_limit = can_rx_data[0];////热量限制
//		cooling_heat = can_rx_data[1];////枪管热量
//	  cooling_rate =can_rx_data[2];////冷却速率
//}


/**
 * @brief  	  设置云台运作模式
 * @param[in]	mode
 * @retval 	  None
 */
void Set_GimbalMode(Gimbal_CtrlMode_e mode)
{
	Gimbal_Mode = mode;
}
/**
 * @brief  	  设置底盘运作模式
 * @param[in]	mode
 * @retval 	  None
 */
void Set_ChassisMode(CHAS_CtrlMode_e mode)
{
	Chassis_Mode = mode;
}
/**
 * @brief  	    设置发射运作模式
 * @param[in]	mode
 * @retval 	    None
 */
void Set_AttackMode(Attack_CtrlMode_e mode)
{

//    if(Attack_Mode == Attack_Auto && mode != Attack_Auto)//有问题
//    {
//        //--- 当自动射击模式被取消时，清零自动射击的数量
//        Set_ReloadNum(0);
//    }
//		
	Attack_Mode = mode;
	//根据模式选择射速
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
 * @brief  	    设置视觉运作模式
 * @param[in]	mode
 * @retval 	    None
 */
void Set_VisionMode(VisionMode_e mode)
{
	Vision_Mode = mode;
	//.............
}

/**
 * @brief  	  机器人失能,状态复位
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
	
