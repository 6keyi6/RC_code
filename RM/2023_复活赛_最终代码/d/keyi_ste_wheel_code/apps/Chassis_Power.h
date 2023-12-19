#ifndef _CHASSIS_POWER_H_
#define _CHASSIS_POWER_H_
#include "Chassis_control.h"
typedef struct
{
    int16_t Power_Buffer;   /*<! ���幦�� */
    int32_t SumCurrent_In;  /*<! ���������ܺ� */
    int32_t SumCurrent_Out; /*<! �����ĵ�������ܺ� */
	  float DRV_CalcRatio;       /*<! ���ڼ������ƹ��ʵ�ϵ�� */
    float RUD_CalcRatio;       /*<! ���ڼ������ƹ��ʵ�ϵ�� */
}CHAS_Power_classdef;

extern CHAS_Power_classdef CHAS_Power;

void CHAS_Power_Limit(int16_t *wheelCurrent, int8_t amount);
void CHAS_Power_Limit_Calc(void);
#endif
