#ifndef _CHASSIS_POWER_H_
#define _CHASSIS_POWER_H_
#include "Chassis_control.h"
typedef struct
{
    int16_t Power_Buffer;   /*<! 缓冲功率 */
    int32_t SumCurrent_In;  /*<! 电流输入总和 */
    int32_t SumCurrent_Out; /*<! 计算后的电流输出总和 */
	  float DRV_CalcRatio;       /*<! 用于计算限制功率的系数 */
    float RUD_CalcRatio;       /*<! 用于计算限制功率的系数 */
}CHAS_Power_classdef;

extern CHAS_Power_classdef CHAS_Power;

void CHAS_Power_Limit(int16_t *wheelCurrent, int8_t amount);
void CHAS_Power_Limit_Calc(void);
#endif
