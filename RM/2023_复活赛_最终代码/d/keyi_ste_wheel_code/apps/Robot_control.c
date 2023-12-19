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
#include "can_control.h"
#include "Devices_Monitor.h"
#include "RM_JudgeSystem.h"
#include "DEV_SuperCapacitor.h"
VisionMode_e Vision_Mode;
RobotWorkMode_e RobotWorkMode = {0};
Robot_TargetValue_t Robot_TargetValue = {0};
Robot_classdef Infantry = {0};
Robot_t Robot;
//uint8_t RefereeData[8]; /*<! 板间通信存储数据 */
uint8_t Shoot_Freq;//子弹射频
uint8_t Onebullet_heat = 10; /*<! 单个子弹的热量(小弹丸) */
/**
  * @brief  机器人总控制
  * @param  None
  * @retval None
  */
int reset_cnt = 0;
int mmll;
void Robot_control(void)
{
        tempV[0] = Robot_Receive.Target_Vx*20.f;
				tempV[1] = Robot_Receive.Target_Vy*20.f;
				tempV[2] = Robot_Receive.Target_Vw*2;
				//超級電容控制
				SupCap_Control();

				Chassis_control();//底盘控制  

//    ShootFreq_Calc();//--- 计算发射射频	

//    RefereeMsg_Send(RefereeData);	//裁判系统信息发送至云台主控
	
	
     if(Robot_Receive.Write_Msg[2] == true && DevicesGet_State(COMMU_0X340_MONITOR) != Off_line)
     {
         reset_cnt++;
         if(reset_cnt>=1000) //--- 收到重启信号持续2s重启
         {
             RobotReset();
         }
     }
     else
     {
         reset_cnt = 0;
     }	    
}


/**
  * @brief  裁判系统信息发送至云台主控
  * @param  can_rx_data
  * @retval None
  */
void RefereeMsg_Send(uint8_t *data)
{
	//--- 0x341
	  uint8_t ON = 1;
		uint8_t *temp;
		float bulletspeed = ext_shoot_data.data.bullet_speed;//--枪管射速
    temp = (uint8_t*)(&bulletspeed);  

    data[0] = ext_game_robot_state.data.shooter_id1_17mm_cooling_limit; //--- 热量限制
    data[1] = ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;
    data[2] = Cell;//超电电量
    data[3] = temp[3];	
	
    data[4] = 0;    //--- 机器人ID&发射电源接口状态
    data[4] |= ext_game_robot_state.data.robot_id;
    data[4] |= ext_game_robot_state.data.mains_power_shooter_output == ON?1<<7:0;	//没有验证
	
    data[5] = ext_game_robot_state.data.shooter_id1_17mm_speed_limit;  //--- 子弹初速度限制
    data[6] = Shoot_Freq;   //--- 子弹射频
	
    data[7] = CHAS_Max_Get_Power(); //--- 底盘功率限制	

	  CAN_Senddata(&hcan1, 0x341, data, 8);
}

/**
  * @brief  裁判系统信息发送至云台主控  2
  * @param  can_rx_data
  * @retval None
  */
void RefereeMsg_Send_2(uint8_t *data)
{
	data[0] = ext_game_robot_state.data.shooter_id1_17mm_cooling_limit;//热量限制
	data[1] = ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;//热量
	data[2] = ext_game_robot_state.data.shooter_id1_17mm_cooling_rate ;//冷却速率
	data[3] = 0;
	data[4] = 0;
	data[5] = 0;	
	data[6] = 0;
	data[7] = 0;
	CAN_Senddata(&hcan1, 0x066, data, 8); //字节 
}

/**
 * @brief  	    根据枪管数据计算射频
 * @param[in]	None
 * @retval 	    ShootFreq
 */
uint8_t ShootFreq_Calc(void)
{
   if((Get_CoolingLimit() - Get_GunHeat())/Onebullet_heat > 1/* CanShoot_Limit */) //--- 计算当前枪管剩余热量
    {
        if((Get_CoolingLimit() - Get_GunHeat())/Onebullet_heat == 2) //--- 刚好满足阈值
        {
            Shoot_Freq = Get_CoolingRate()/Onebullet_heat;
        }
        else if(Get_CoolingLimit()-Get_GunHeat() <= Onebullet_heat*4) //--- 剩余在某个阈值之内 4颗子弹
        {
            Shoot_Freq = ((Get_CoolingLimit()-Get_GunHeat()) / (Onebullet_heat*2)) * (20 - Get_CoolingRate()/Onebullet_heat) + Get_CoolingRate()/Onebullet_heat;
        }
        else
        {
            Shoot_Freq = 20; //---未达到热量警告区间，满频输出
        }
    }
    else //--- 不满足刚好发射阈值数子弹
    {
        Shoot_Freq = 0;
    }

    return Shoot_Freq;
}

uint16_t Get_CoolingLimit(void)
{
	return ext_game_robot_state.data.shooter_id1_17mm_cooling_limit;
}

uint16_t Get_GunHeat(void)
{
	return ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;
}

uint16_t Get_CoolingRate(void)
{
	return ext_game_robot_state.data.shooter_id1_17mm_cooling_rate;
}

