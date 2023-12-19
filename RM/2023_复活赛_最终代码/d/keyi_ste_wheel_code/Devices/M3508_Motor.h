#ifndef __M3508_MOTOR_H
#define __M3508_MOTOR_H
#include "typedef.h"
#define M3508_READID_START 0x201
#define M3508_READID_END 0x204

#define M3508_PowerL_1 0x201
#define M3508_PowerR_2 0x202
typedef struct
{
    uint16_t realAngle;  //读回来的机械角度
    int16_t realSpeed;   //读回来的速度
    int16_t realCurrent; //读回来的实际电流
    uint8_t temperture;  //读回来的电机温度

    int16_t targetSpeed;  //目标速度
    uint16_t targetAngle; //目标角度
    uint16_t lastAngle;   //上次的角度
    int32_t totalAngle;   //累积总共角度
    int16_t turnCount;    //转过的圈数

    int16_t outCurrent; //输出电流

    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} M3508s_t;

typedef struct
{
	float Follow_PTZ_P_result;
	float Follow_PTZ_I_result;
	float NotFollow_I_result[4];
	float Spin_I_result[4];
	int chassis_vx,chassis_vy,chassis_vw;//底盘XY轴方向
	float Follow_PTZ_speed_result[4];
	float SPIN_result[4];
	
}M3508_motor_t;

typedef struct
{
	float Frict_CurL;
	float Frict_CurR;
	float Frict_OUTL;
	float Frict_OUTR;	
  float RC_FrictL;
  float RC_FrictR;	

	
}Frict_motor_t;
extern Frict_motor_t Frict_motor;
extern M3508_motor_t M3508_motor;//底盘自定义结构体

extern M3508s_t M3508s[4];//底盘电机返回数值
extern M3508s_t M3508_PowerL; //摩擦轮电机 201
extern M3508s_t M3508_PowerR; //摩擦轮电机; 202

void M3508_getInfo(Can_Export_Data_t RxMessage);
/**
 * @brief 从CAN报文中获取M3508摩擦轮电机信息
 * 
 * @param RxMessage 
 * @return  
 */
void M3508_Friction_getInfo(Can_Export_Data_t RxMessage);
#endif
