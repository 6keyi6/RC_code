#ifndef __CHIASSIS_CONTROL_H
#define __CHIASSIS_CONTROL_H
#include "pid.h"
#include "Can_Control.h"
#include "Robot_Control.h"
#include "Devices_Monitor.h"

#ifndef PI
  #define PI               3.14159265358979f
#endif
#define RUD_OPSI       1
#define RUD_NOT_OPSI   0
#define RUD_RESET      1
#define RUD_NOT_RESET  0

/* --- 转向轮电机相关参数                                                                                               -------------------------------------------------------*/
typedef struct 
{
    float Init_angle;   // 初始化校准角度
    float Target_angle; // 目标角度
    float PreTar_angle; // 前一次目标角度
    float Total_angle;  // 当前总角度
    int32_t Turns_cnt;
    int32_t TarTurns_cnt;
    int32_t Turns_flag;
}RUD_Param_t;

typedef struct{
	float RudOutResult;     //角度环计算结果
	float RudInResult;     //速度环计算结果
}M6020_RUD_motor_t;

typedef struct
{
	float DriOutResult;
  float DriInResult;
	
}M3508_motor_t;

/* --- 底盘转向电机 ID --------------------------------------------------------*/
enum CHAS_RudMororID_e
{
    RF_205 = 0,
    LF_206 = 1,
    LB_207 = 2,
    RB_208 = 3
};
/* --- 底盘驱动电机 ID --------------------------------------------------------*/
enum CHAS_DrvMotorID_e
{
    RF_201 = 0,
    LF_202 = 1,
    LB_203 = 2,
    RB_204 = 3
};

float my_fabs(float num);
void Chassis_PID_Init(void);
void Process(float Vx, float Vy, float Vw);
void Chassis_Drv_Slow(float *rec , float target , float slow_Inc, float Accval, float DecVal);
void Rudder_Solve(int16_t Vx, int16_t Vy, int16_t Vw, int16_t *cal_speed);
void Constrain(int16_t *val, int16_t min, int16_t max);
void RudAngle_Calc(int16_t Vx, int16_t Vy, int16_t Vw);
void RUDTargetAngle_Calc(int8_t motor_num , int8_t reset , uint8_t opposite);
void AngleLimit(float *angle);
float Turn_InferiorArc(uint8_t motor, float target, float current);
void Chassis_control(void);
#endif
