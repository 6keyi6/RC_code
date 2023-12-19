#include "DJI_IMU.h"

//由于can的数据帧最大的字节数为8
//欧拉角的一个轴的数据就是一个float类型变量，是4个字节
//那也就最多只能发两个轴
Euler_Send_u Euler_Send;
Gyro_Send_u Gyro_Send;
//接收变量
IMU_CAL_t IMU_CAL;

//接收数据联合体
DJI_C_Euler_u DJI_C_Euler_Receive;
DJI_C_Gyro_u DJI_C_Gyro_Receive;
//数据处理结构体
DJIC_IMU_t DJIC_IMU;
#if send_way == 0
/*第一种发送方式：联合体*/
//欧拉角
void Euler_Send_Fun(Euler_Send_u Euler_Send)
{
	for (int i = 0; i < 8; i++)//获取陀螺仪角度
	{
		DJI_C_Euler_Receive.BraodData[i] = Euler_Send.Euler_Angle[i];
	}
	//8个1字节的缓存局部变量
	uint8_t data[8];
	
	//Yaw轴angle
	data[0] = Euler_Send.Euler_Angle[0];
	data[1] = Euler_Send.Euler_Angle[1];
	data[2] = Euler_Send.Euler_Angle[2];
	data[3] = Euler_Send.Euler_Angle[3];
	
	//Pitch轴angle
	data[4] = Euler_Send.Euler_Angle[4];
	data[5] = Euler_Send.Euler_Angle[5];
	data[6] = Euler_Send.Euler_Angle[6];
	data[7] = Euler_Send.Euler_Angle[7];
	
	//以CAN通讯发送
//	CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Angle_SENDID,data);
}
//角速度
void Gyro_Send_Fun(Gyro_Send_u Gyro_Send)
{
	for (int i = 0; i < 8; i++)//获取角速度
	{
		DJI_C_Gyro_Receive.BraodData[i] = Gyro_Send.Gyro_zy[i];
	}
	//8个1字节的缓存局部变量
	uint8_t data[8];
	
	//Yaw轴angle
	data[0] = Gyro_Send.Gyro_zy[0];
	data[1] = Gyro_Send.Gyro_zy[1];
	data[2] = Gyro_Send.Gyro_zy[2];
	data[3] = Gyro_Send.Gyro_zy[3];
	
	//Pitch轴angle
	data[4] = Gyro_Send.Gyro_zy[4];
	data[5] = Gyro_Send.Gyro_zy[5];
	data[6] = Gyro_Send.Gyro_zy[6];
	data[7] = Gyro_Send.Gyro_zy[7];
	
	//以CAN通讯发送
//	CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Gyro_SENDID,data);
}

//接收解包
void IMU_Cal_Status_Reivece(CAN_Rx_TypeDef CAN_Rx_Structure)
{
	if(CAN_Rx_Structure.CAN_RxMessage.StdId != IMU_CAL_REIID)
	{
		return;
	}
	//获取陀螺仪当前的校准状态
	IMU_CAL.real_Status = CAN_Rx_Structure.CAN_RxMessageData[0];
}
#endif

//处理更新数据
//void Updata_Hand_Euler_Gyro_Data(void)
//{
//	//角度
//	DJIC_IMU.yaw = (float)Euler_Send.yaw  * Angle_turn_Radian + 180.f;		//将弧度转为度
//	DJIC_IMU.pitch = (float)Euler_Send.pitch * Angle_turn_Radian + 180.0f; //(-180° ~ 180°)
//	//角速度
//	DJIC_IMU.Gyro_z = Gyro_Send.Gyro_z * Angle_turn_Radian  ;
//	DJIC_IMU.Gyro_y = Gyro_Send.Gyro_y * Angle_turn_Radian ;

//	//yaw轴的过零处理
//	if (DJIC_IMU.yaw - DJIC_IMU.last_yaw < -300.0f)
//	{
//		DJIC_IMU.yaw_turnCounts++;
//	}
//	if (DJIC_IMU.last_yaw - DJIC_IMU.yaw < -300.0f)
//	{
//		DJIC_IMU.yaw_turnCounts--;
//	}
//	DJIC_IMU.total_yaw = DJIC_IMU.yaw + DJIC_IMU.yaw_turnCounts * 360.0f;
//	DJIC_IMU.last_yaw = DJIC_IMU.yaw;

//	//Pitch
//	if (DJIC_IMU.pitch - DJIC_IMU.last_pitch < -300.0f)
//	{
//		DJIC_IMU.pitch_turnCounts++;
//	}
//	if (DJIC_IMU.last_pitch - DJIC_IMU.pitch < -300.0f)
//	{
//		DJIC_IMU.pitch_turnCounts--;
//	}
//	DJIC_IMU.total_pitch = DJIC_IMU.pitch + DJIC_IMU.pitch_turnCounts * 360.0f;
//	DJIC_IMU.last_pitch = DJIC_IMU.pitch;
//}


#if send_way == 1
/*第二种发送形式：指针*/
//欧拉角
void Euler_Send_Fun(float Yaw,float Pitch)
{
	//所有指针都占4个字节
	unsigned char* p[2];
	uint8_t data[8];
	
	p[0] = (unsigned char*)&Yaw;
	p[1] = (unsigned char*)&Pitch;
	
	//Yaw轴angle
	data[0] = *p[0];
	data[1] = *(p[0] + 1);
	data[2] = *(p[0] + 2);
	data[3] = *(p[0] + 3);
	
	//Pitch轴angle
	data[4] = *p[1];
	data[5] = *(p[1] + 1);
	data[6] = *(p[1] + 2);
	data[7] = *(p[1] + 3);
	
	//以CAN通讯发送
	CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Angle_SENDID,data);
}
//角速度
void Gyro_Send_Fun(float Gyro_z,float Gyro_y)
{
	//所有指针都占4个字节
	unsigned char* p[2];
	uint8_t data[8];
	
	p[0] = (unsigned char*)&Gyro_z;
	p[1] = (unsigned char*)&Gyro_y;
	
	//Yaw轴angle
	data[0] = *p[0];
	data[1] = *(p[0] + 1);
	data[2] = *(p[0] + 2);
	data[3] = *(p[0] + 3);
	
	//Pitch轴angle
	data[4] = *p[1];
	data[5] = *(p[1] + 1);
	data[6] = *(p[1] + 2);
	data[7] = *(p[1] + 3);
	
	//以CAN通讯发送
	CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Gyro_SENDID,data);
}

#endif
