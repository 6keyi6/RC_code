#include "Chassis_Power.h"
#include "RM_JudgeSystem.h"
#include "can_control.h"
CHAS_Power_classdef CHAS_Power;

// 功率限制
void CHAS_Power_Limit(int16_t *wheelCurrent, int8_t amount)
{
	 float coe[4] = {0.0f};
   //--- 裁判系统离线 强制限制
//	 if()
//	 {
//	 }
	 //--- 不限功率
	 if(ext_game_robot_state.data.chassis_power_limit == 65535)
	 {
		 CHAS_Power.SumCurrent_Out = CHAS_Power.SumCurrent_In;//--- 无处理时为原来的值
		 return;
	 } 
	 CHAS_Power.SumCurrent_In = CHAS_Power.SumCurrent_Out = 0; 
	     /*----------------------- 上坡功率再分配 --------------------------*/
    float FWheel_coe[2][2] = {{0, 0}, {0.0}}; //--- 前轮电流占比
    int32_t Fwheel_IN[2] = {0}; //--- 前轮电流
    uint8_t i = 0;

//		if(1)
//		{
//							//--- 计算前轮电流
//				for (i = 0; i < amount / 2; i++)
//				{
//						Fwheel_IN[0] += abs(wheelCurrent[i]);
//				}
//        //--- 计算前轮百分比
//        for (i = 0; i < amount / 2; i++)
//        {
//            FWheel_coe[0][i] = ((float)(wheelCurrent[i])) / ((float)(Fwheel_IN[0]));
//        }		
//        //--- 需要分配后轮的功率 挖一般分给后轮(测试值70%)
//        int32_t temp_FWheel = (int32_t)Fwheel_IN[0] * 0.5f;
//        Fwheel_IN[0] -= temp_FWheel;
//        //--- 计算分配后，轮子该有的功率
//        for (i = 0; i < amount / 2; i++)
//        {
//            wheelCurrent[i] = (int16_t)(Fwheel_IN[0] * FWheel_coe[0][i]);
//        }		

//        //--- 计算后轮原本的输入电流
//        for (i = amount / 2; i < amount; i++)
//        {
//            Fwheel_IN[1] += abs(wheelCurrent[i]);
//        }
//        //--- 计算后轮原本的输入百分比
//        for (i = amount / 2; i < amount; i++)
//        {
//            FWheel_coe[1][i] = ((float)(wheelCurrent[i])) / ((float)(Fwheel_IN[1]));
//        }
//        //--- 给后轮加上从前轮挖过来的电流
//        Fwheel_IN[1] += temp_FWheel;
//        //--- 计算分配后，后轮最终该有的电流
//        for (i = amount / 2; i < amount; i++)
//        {
//            wheelCurrent[i] = (int16_t)(Fwheel_IN[1] * FWheel_coe[1][i]);
//        }				
//		}
		
		
		/*-----------------------------------------*/

    for(uint8_t i = 0 ; i < amount ; i++)
    {
        CHAS_Power.SumCurrent_In += abs(wheelCurrent[i]);
    }		
	  CHAS_Power.SumCurrent_Out = CHAS_Power.SumCurrent_In;	
		    // 计算每个电机的电流占比
    for(uint8_t i = 0 ; i < amount ; i++)
    {
        coe[i] = ((float)(wheelCurrent[i])) / ((float)(CHAS_Power.SumCurrent_In));
    }
     CHAS_Power_Limit_Calc();
		
    for(uint8_t i = 0 ; i < amount ; i++)
    {
        wheelCurrent[i] = ((CHAS_Power.SumCurrent_Out) * coe[i]);
    }		
}

int16_t powerBuffErr;  // 用掉的缓冲能量
float debug_powercoe = 80.0f;
void CHAS_Power_Limit_Calc(void)
{
   CHAS_Power.Power_Buffer = ext_power_heat_data.data.chassis_power_buffer;
	 powerBuffErr = 60 -  CHAS_Power.Power_Buffer;
	 CHAS_Power.DRV_CalcRatio = 0;
	 CHAS_Power.DRV_CalcRatio = (float)CHAS_Power.Power_Buffer / debug_powercoe;
	 CHAS_Power.DRV_CalcRatio *= CHAS_Power.DRV_CalcRatio;// 平方的关系
    if(powerBuffErr > 0 /*&& Write_Msg[Cap_Ctrl] != true */)  // 若用到缓冲功率则进行功率限制处理
    {
        CHAS_Power.SumCurrent_Out = CHAS_Power.SumCurrent_In * CHAS_Power.DRV_CalcRatio;
    }
	
}
