#ifndef DJI_IMU_H
#define DJI_IMU_H

#include "main.h"
#include "user_can.h"
#pragma anon_unions

#define send_way 0

//发送ID
#define DJI_C_Angle_SENDID 0x195
#define DJI_C_Gyro_SENDID 0x165
//陀螺仪校准 接收ID 
#define IMU_CAL_REIID 0x096

#define begin_calibration 1
#define stop_calibration 0

#define Angle_turn_Radian 57.295779513082320876798154814105f
//以联合体形式发送，可以将float 类型的数据 转成 以uint8_t类型去发送
//欧拉角
typedef union
{
	struct
	{
		float yaw;
		float pitch;
	};
	uint8_t Euler_Angle[8];
}Euler_Send_u;
extern Euler_Send_u Euler_Send;

//角速度
typedef union
{
	struct
	{
		float Gyro_z;
		float Gyro_y;
	};
	uint8_t Gyro_zy[8];
}Gyro_Send_u;
extern Gyro_Send_u Gyro_Send;

//接收变量
//开始校准
typedef struct
{
	uint8_t real_Status;
	uint8_t last_Status;
}IMU_CAL_t;
extern IMU_CAL_t IMU_CAL;


//DJI_C 陀螺仪角度接收联合体
typedef union
{
	struct
	{
		float yaw;
		float pitch;
	};
	uint8_t BraodData[8];
} DJI_C_Euler_u;
extern DJI_C_Euler_u DJI_C_Euler_Receive;
//DJI_C 陀螺仪角速度接收联合体
typedef union
{
	struct
	{
		float Gyro_z;
		float Gyro_y;
	};
	uint8_t BraodData[8];
} DJI_C_Gyro_u;
extern DJI_C_Gyro_u DJI_C_Gyro_Receive;


//处理过数据
typedef struct
{
	float yaw;
	float last_yaw;
	int32_t yaw_turnCounts;
	float total_yaw;
	float pitch;
	float last_pitch;
	int32_t pitch_turnCounts;
	float total_pitch;
	float Gyro_z;
	float Gyro_y;

//	P_PID_t yaw_pid;
//	P_PID_t gyro_z_pid;
//	P_PID_t pitch_pid;
//	P_PID_t gyro_y_pid;

} DJIC_IMU_t;
extern DJIC_IMU_t DJIC_IMU;

#if send_way == 0
void Euler_Send_Fun(Euler_Send_u Euler_Send);
void Gyro_Send_Fun(Gyro_Send_u Gyro_Send);
void IMU_Cal_Status_Reivece(CAN_Rx_TypeDef CAN_Rx_Structure);
#endif
#if send_way == 1
void Euler_Send_Fun(float Yaw,float Pitch);
void Gyro_Send_Fun(float Gyro_z,float Gyro_y);
#endif

//处理更新数据
void Updata_Hand_Euler_Gyro_Data(void);
#endif

