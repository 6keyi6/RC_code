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
//uint8_t RefereeData[8]; /*<! ���ͨ�Ŵ洢���� */
uint8_t Shoot_Freq;//�ӵ���Ƶ
uint8_t Onebullet_heat = 10; /*<! �����ӵ�������(С����) */
/**
  * @brief  �������ܿ���
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
				//������ݿ���
				SupCap_Control();

				Chassis_control();//���̿���  

//    ShootFreq_Calc();//--- ���㷢����Ƶ	

//    RefereeMsg_Send(RefereeData);	//����ϵͳ��Ϣ��������̨����
	
	
     if(Robot_Receive.Write_Msg[2] == true && DevicesGet_State(COMMU_0X340_MONITOR) != Off_line)
     {
         reset_cnt++;
         if(reset_cnt>=1000) //--- �յ������źų���2s����
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
  * @brief  ����ϵͳ��Ϣ��������̨����
  * @param  can_rx_data
  * @retval None
  */
void RefereeMsg_Send(uint8_t *data)
{
	//--- 0x341
	  uint8_t ON = 1;
		uint8_t *temp;
		float bulletspeed = ext_shoot_data.data.bullet_speed;//--ǹ������
    temp = (uint8_t*)(&bulletspeed);  

    data[0] = ext_game_robot_state.data.shooter_id1_17mm_cooling_limit; //--- ��������
    data[1] = ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;
    data[2] = Cell;//�������
    data[3] = temp[3];	
	
    data[4] = 0;    //--- ������ID&�����Դ�ӿ�״̬
    data[4] |= ext_game_robot_state.data.robot_id;
    data[4] |= ext_game_robot_state.data.mains_power_shooter_output == ON?1<<7:0;	//û����֤
	
    data[5] = ext_game_robot_state.data.shooter_id1_17mm_speed_limit;  //--- �ӵ����ٶ�����
    data[6] = Shoot_Freq;   //--- �ӵ���Ƶ
	
    data[7] = CHAS_Max_Get_Power(); //--- ���̹�������	

	  CAN_Senddata(&hcan1, 0x341, data, 8);
}

/**
  * @brief  ����ϵͳ��Ϣ��������̨����  2
  * @param  can_rx_data
  * @retval None
  */
void RefereeMsg_Send_2(uint8_t *data)
{
	data[0] = ext_game_robot_state.data.shooter_id1_17mm_cooling_limit;//��������
	data[1] = ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;//����
	data[2] = ext_game_robot_state.data.shooter_id1_17mm_cooling_rate ;//��ȴ����
	data[3] = 0;
	data[4] = 0;
	data[5] = 0;	
	data[6] = 0;
	data[7] = 0;
	CAN_Senddata(&hcan1, 0x066, data, 8); //�ֽ� 
}

/**
 * @brief  	    ����ǹ�����ݼ�����Ƶ
 * @param[in]	None
 * @retval 	    ShootFreq
 */
uint8_t ShootFreq_Calc(void)
{
   if((Get_CoolingLimit() - Get_GunHeat())/Onebullet_heat > 1/* CanShoot_Limit */) //--- ���㵱ǰǹ��ʣ������
    {
        if((Get_CoolingLimit() - Get_GunHeat())/Onebullet_heat == 2) //--- �պ�������ֵ
        {
            Shoot_Freq = Get_CoolingRate()/Onebullet_heat;
        }
        else if(Get_CoolingLimit()-Get_GunHeat() <= Onebullet_heat*4) //--- ʣ����ĳ����ֵ֮�� 4���ӵ�
        {
            Shoot_Freq = ((Get_CoolingLimit()-Get_GunHeat()) / (Onebullet_heat*2)) * (20 - Get_CoolingRate()/Onebullet_heat) + Get_CoolingRate()/Onebullet_heat;
        }
        else
        {
            Shoot_Freq = 20; //---δ�ﵽ�����������䣬��Ƶ���
        }
    }
    else //--- ������պ÷�����ֵ���ӵ�
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

