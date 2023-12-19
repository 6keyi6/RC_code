#include "Chassis_Power.h"
#include "RM_JudgeSystem.h"
#include "can_control.h"
CHAS_Power_classdef CHAS_Power;

// ��������
void CHAS_Power_Limit(int16_t *wheelCurrent, int8_t amount)
{
	 float coe[4] = {0.0f};
   //--- ����ϵͳ���� ǿ������
//	 if()
//	 {
//	 }
	 //--- ���޹���
	 if(ext_game_robot_state.data.chassis_power_limit == 65535)
	 {
		 CHAS_Power.SumCurrent_Out = CHAS_Power.SumCurrent_In;//--- �޴���ʱΪԭ����ֵ
		 return;
	 } 
	 CHAS_Power.SumCurrent_In = CHAS_Power.SumCurrent_Out = 0; 
	     /*----------------------- ���¹����ٷ��� --------------------------*/
    float FWheel_coe[2][2] = {{0, 0}, {0.0}}; //--- ǰ�ֵ���ռ��
    int32_t Fwheel_IN[2] = {0}; //--- ǰ�ֵ���
    uint8_t i = 0;

//		if(1)
//		{
//							//--- ����ǰ�ֵ���
//				for (i = 0; i < amount / 2; i++)
//				{
//						Fwheel_IN[0] += abs(wheelCurrent[i]);
//				}
//        //--- ����ǰ�ְٷֱ�
//        for (i = 0; i < amount / 2; i++)
//        {
//            FWheel_coe[0][i] = ((float)(wheelCurrent[i])) / ((float)(Fwheel_IN[0]));
//        }		
//        //--- ��Ҫ������ֵĹ��� ��һ��ָ�����(����ֵ70%)
//        int32_t temp_FWheel = (int32_t)Fwheel_IN[0] * 0.5f;
//        Fwheel_IN[0] -= temp_FWheel;
//        //--- �����������Ӹ��еĹ���
//        for (i = 0; i < amount / 2; i++)
//        {
//            wheelCurrent[i] = (int16_t)(Fwheel_IN[0] * FWheel_coe[0][i]);
//        }		

//        //--- �������ԭ�����������
//        for (i = amount / 2; i < amount; i++)
//        {
//            Fwheel_IN[1] += abs(wheelCurrent[i]);
//        }
//        //--- �������ԭ��������ٷֱ�
//        for (i = amount / 2; i < amount; i++)
//        {
//            FWheel_coe[1][i] = ((float)(wheelCurrent[i])) / ((float)(Fwheel_IN[1]));
//        }
//        //--- �����ּ��ϴ�ǰ���ڹ����ĵ���
//        Fwheel_IN[1] += temp_FWheel;
//        //--- �������󣬺������ո��еĵ���
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
		    // ����ÿ������ĵ���ռ��
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

int16_t powerBuffErr;  // �õ��Ļ�������
float debug_powercoe = 80.0f;
void CHAS_Power_Limit_Calc(void)
{
   CHAS_Power.Power_Buffer = ext_power_heat_data.data.chassis_power_buffer;
	 powerBuffErr = 60 -  CHAS_Power.Power_Buffer;
	 CHAS_Power.DRV_CalcRatio = 0;
	 CHAS_Power.DRV_CalcRatio = (float)CHAS_Power.Power_Buffer / debug_powercoe;
	 CHAS_Power.DRV_CalcRatio *= CHAS_Power.DRV_CalcRatio;// ƽ���Ĺ�ϵ
    if(powerBuffErr > 0 /*&& Write_Msg[Cap_Ctrl] != true */)  // ���õ����幦������й������ƴ���
    {
        CHAS_Power.SumCurrent_Out = CHAS_Power.SumCurrent_In * CHAS_Power.DRV_CalcRatio;
    }
	
}
