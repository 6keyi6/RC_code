/**
 * @file M2006_Motor.h
 * @author keyi (hzjqq66@163.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-8
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __M2006_MOTOR_H
#define __M2006_MOTOR_H
#include "typedef.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define M2006_READID_START 0x203

typedef enum
{
    Reloader_allow = 0,
    Reloader_ban = 1
} Reloader_license_t;
//typedef struct
//{
//    uint16_t realAngle; //读回来的机械角度
//    int16_t realSpeed;  //读回来的速度
//    int16_t realTorque; //读回来的实际转矩

//    int16_t targetSpeed; //目标速度
//    int32_t targetAngle; //目标角度

//    uint16_t lastAngle; //上次的角度
//    int32_t totalAngle; //累积总共角度
//    int16_t turnCount;  //转过的圈数

//    int16_t outCurrent;      //输出电流
//    int16_t inneroutCurrent; //输出电流

//    uint8_t InfoUpdateFlag;   //信息读取更新标志
//    uint16_t InfoUpdateFrame; //帧率
//    uint8_t OffLineFlag;      //设备离线标志
//} M2006s_t;

typedef struct
{
  float RC_angle;//外环目标
	float pos_temp;//外环测量
	float Dial_P_result;//内环目标
	float IN_CurSpeed;//内环测量
	float Dial_I_result;//
  float ClogSpeed; //阻塞速度
	float PillObstruct_Time;//弹丸阻塞时间
  uint16_t NeedLaunchCount; //还需要发射的弹丸数量。 
	int PillObstructLogo;//弹丸阻塞标志位
	int PillObstruct_targetAngle;//卡弹目标值
	Reloader_license_t Reloader_license;
	uint8_t PillObstruct_Direction;   //卡弹后任务的执行方向。1反转，0正传
}M2006_motor_t;

//extern M2006s_t M2006_Reload;//拨盘电机
extern M2006_motor_t M2006_motor;//自定义结构体数据

/**
  * @brief  从CAN报文中获取M2006电机信息
  * @param[in]  RxMessage 	CAN报文接收结构体
  * @retval None
  */
void M2006_getInfo(Can_Export_Data_t RxMessage);


#endif
