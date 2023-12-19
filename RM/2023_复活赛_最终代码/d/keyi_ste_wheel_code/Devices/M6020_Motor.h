/**
 * @file M6020_Motor.h
 * @author keyi (hzjqq66@163.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-8
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __M6020_MOTOR_H
#define __M6020_MOTOR_H
#include "typedef.h"
#include "pid.h"
#define M6020_READID_START 0x205 //当ID为1时的报文ID
#define M6020_PICTH 0x206 



typedef struct
{
    uint16_t realAngle;  //读回来的机械角度
    int16_t realSpeed;   //读回来的速度
    int16_t realCurrent; //读回来的实际转矩电流
    uint8_t temperture;  //读回来的电机温度

    int16_t targetSpeed; //目标速度
    int32_t targetAngle; //目标角度
    uint16_t lastAngle;  //上次的角度
    float totalAngle;  //累积总共角度
    int16_t turnCount;   //转过的圈数

    int16_t outCurrent; //输出电流

    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} M6020s_t;

typedef struct{
  float remote_control_YAW;//遥控器YAW轴接收数值
	float YAW_P_result;     //角度环计算结果
	float YAW_I_result;     //速度环计算结果
	float IMU_T[3];            //陀螺仪角度结果
	
	float YAW_Centre_P_result; //YAW回中计算结果
  float YAW_Centre_I_result; //YAW回中计算结果	
	
	float YAW_OUT_Current;//外环测量
	float YAW_IN_Current;//内环测量值
	
	float Vis_Init_YAWangle;//视觉自瞄初始值
	
}M6020_motor_t;
typedef struct{
	float RUD_OUT;     //角度环计算结果
	float RUD_IN;     //速度环计算结果
	
	float RUD_SPIN_OUT;
	float RUD_SPIN_IN;
}M6020_RUD_motor_t;

typedef struct{
	float RC_PICTH;//遥控器PICTH轴接收数值
	float PICTH_OUT;    //角度环计算结果
	float PICTH_IN;     //速度环计算结果
	float IMU_P[3];     //陀螺仪角度结果
	float PICTH_OUT_Current;//外环测量
	float PICTH_IN_Current;//内环测量值	
	float Vis_Init_PITangle;//视觉自瞄初始值
}M6020_PITCH_t;


typedef struct
{
  float IMUVisionErr;
	float Vision_YawAngle;
	float Vision_OUT;
	float Vision_IN;
	
	
}YAWVision_t;//YAW轴视觉结构体

typedef struct
{
  float IMUVisionErr;
	float Vision_PITAngle;
	float Vision_OUT;
	float Vision_IN;
}PITVision_t;//YAW轴视觉结构体

void M6020_getInfo(Can_Export_Data_t RxMessage);
extern M6020_RUD_motor_t M6020_RUD_motor[4];

extern M6020s_t M6020s_Yaw;   //ID为1
extern M6020s_t M6020s_Pitch; //2
//extern M6020_Fun_t M6020_Fun;
extern M6020s_t *M6020_Array[];//接收数组
extern YAWVision_t YAWVision;
extern PITVision_t PITVision;
extern positionpid_t M6020_Yaw_Ppid;//YAW轴PID参数
extern positionpid_t M6020_Yaw_Ipid;//YAW轴PID参数
extern positionpid_t  YAW_Centre_Ppid;//PID 回中参数
extern incrementalpid_t YAW_Centre_Ipid;//PID 回中参数
extern positionpid_t PITCH_Ppid;//picth轴PID参数
extern positionpid_t PITCH_Ipid;//picth轴PID参数

extern M6020_motor_t M6020_motor;
extern M6020_PITCH_t PITCH_Use;

extern positionpid_t Yaw_Vision_Out_Pid;//YAW轴自瞄外环PID参数
extern positionpid_t Yaw_Vision_IN_Pid;//YAW轴自瞄内环PID参数

extern positionpid_t PIT_Vision_Out_Pid;//PIT轴自瞄外环PID参数
extern positionpid_t PIT_Vision_IN_Pid;//PIT轴自瞄内环PID参数
//未赋值
extern positionpid_t Vision_AnglePid_Yaw;
extern positionpid_t Vision_SpeedPid_Yaw; 
extern positionpid_t Vision_AnglePid_Pitch; 
extern positionpid_t Vision_SpeedPid_Pitch;


extern M6020s_t M6020s_RUD[4];

#endif /* __M3508_MOTOR_H */
