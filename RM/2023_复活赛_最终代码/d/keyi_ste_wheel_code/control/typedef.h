#ifndef  __TYPEDEF_H
#define  __TYPEDEF_H
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h> 
#include "can.h"
/*************CAN对外数据接口************/
typedef struct
{
    CAN_RxHeaderTypeDef CAN_RxHeader;
    uint8_t CANx_Export_RxMessage[8];
} Can_Export_Data_t;

/*************CAN发送数据接口************/
typedef struct
{
    CAN_HandleTypeDef *Canx;
    CAN_TxHeaderTypeDef CAN_TxHeader;
    uint8_t CANx_Send_RxMessage[8];
} Can_Send_Data_t;
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

typedef int8_t  s8;
typedef int16_t s16;
typedef int32_t s32;

typedef volatile uint8_t  vu8;
typedef volatile uint16_t vu16;
typedef volatile uint32_t vu32;

typedef volatile int8_t  vs8;
typedef volatile int16_t vs16;
typedef volatile int32_t vs32;

///* PID参数 */
//typedef struct{
//	float Target; 			        //设定目标值
//	float Measured; 				    //测量值
//	float err; 						      //本次偏差值
//	float err_last; 				    //上一次偏差
//	float err_beforeLast; 			//上上次偏差
//	float Kp, Ki, Kd;				    //Kp, Ki, Kd控制系数
//	float pwm; 						      //pwm输出
//	uint32_t MaxOutput;				  //输出限幅
//  uint32_t IntegralLimit;			//积分限幅 
//}incrementalpid_t;

//typedef struct{
//	float Target; 					    //设定目标值
//	float Measured; 				    //测量值
//	float err; 						      //本次偏差值
//	float err_last; 				    //上一次偏差
//	float integral_err; 			  //所有偏差之和
//	float Kp, Ki, Kd;				    //Kp, Ki, Kd控制系数
//	float pwm; 						      //pwm输出
//	uint32_t MaxOutput;				  //输出限幅
//  uint32_t IntegralLimit;			//积分限幅 
//	float p_out, i_out, d_out;
//}positionpid_t;

typedef struct positionpid_t
{
    float Target;     //设定目标值
    float Measured;   //测量值
    float err;        //本次偏差值
    float err_last;   //上一次偏差
    float err_change; //误差变化率
    float Kp;
    float Ki;
    float Kd; //Kp, Ki, Kd控制系数
    float p_out;
    float i_out;
    float d_out;               //各部分输出值
    float pwm;                 //pwm输出
    float MaxOutput;           //输出限幅
    float Integral_Separation; //积分分离阈值
    float IntegralLimit;       //积分限幅
    float (*Position_PID)(struct positionpid_t *pid_t, float target, float measured);
} imu_positionpid_t;//专用陀螺仪控制温度PID结构体


/**********系统时间对外数据接口************/
typedef struct
{
    uint32_t WorldTime;      //世界时间
    uint32_t Last_WorldTime; //上一次世界时间
} WorldTime_RxTypedef;
void Get_FPS(WorldTime_RxTypedef *time, uint32_t *FPS);


extern int i,j;
#endif
