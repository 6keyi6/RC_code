#ifndef __CAN_CONTROL_
#define __CAN_CONTROL_

#include"can.h"
#include "typedef.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "queue.h"
#define Can_DataGroundInit \
    {                      \
        {0}, {0},          \
    }

//extern osMessageQId CAN1_ReceiveHandle;
//extern osMessageQId CAN2_ReceiveHandle;
//extern osMessageQId CAN_SendHandle;
typedef struct
{
    struct
    {
        CAN_FilterTypeDef CAN_Filter;
    } CAN_FilterTypedef;

    struct
    {
        CAN_RxHeaderTypeDef CANx_RxHeader;
        uint8_t CAN_RxMessage[8];
    } CAN_RxTypedef;

} Can_Data_t;

typedef enum
{
	CAN_2006Moto_ALL_ID =  0x200,
	CAN_2006Moto1_ID = 0x201,
	CAN_2006Moto2_ID = 0x202,
	CAN_2006Moto3_ID = 0x203,
	CAN_2006Moto4_ID = 0x204,
	CAN_6020Moto_ALL_ID =  0x205,
	CAN_6020Moto1_ID = 0x205,
	CAN_6020Moto2_ID = 0x206,
	CAN_6020Moto3_ID = 0x207,
	CAN_6020Moto4_ID = 0x208,
}CAN_Message_ID;

/*接收到的云台电机的参数结构体*/
typedef struct{
	int16_t	 	speed_rpm;
    float  	real_current;
    int16_t  	given_current;
    uint8_t  	hall;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	//abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
//	u8			buf_idx;
//	u16			angle_buf[FILTER_BUF_LEN];
//	u16			fited_angle;
//	u32			msg_cnt;
	int cyclinder_number;
}moto_measure_t;

extern moto_measure_t  moto_chassis[];//结构体初始化
extern moto_measure_t  M6020moto_chassis[];//结构体初始化

extern moto_measure_t M3508moto_chassis[4];
void CAN_IT_Init(CAN_HandleTypeDef *hcanx, uint8_t Can_type);


void CAN_senddata(CAN_HandleTypeDef *hcan,int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4);
//void get_moto_measure(moto_measure_t *ptr);

void CAN_senddata_6020(CAN_HandleTypeDef *hcan,int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4);
//void get_moto_measure_6020(moto_measure_t *ptr);


void HAL_CAN_RxFifo0MsgPendingCallback2(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback1(CAN_HandleTypeDef *hcan);
void WriteMsgFromGimbal(void );
void CAN_RxMessage_Export_Date(CAN_HandleTypeDef *hcanx, osMessageQId CANx_Handle, uint8_t Can_type);
void CAN_0x1FF_SendData(CAN_HandleTypeDef *CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void CAN_SendData(osMessageQId CANx_Handle, CAN_HandleTypeDef *CANx, uint8_t id_type, uint32_t id, uint8_t data[8]);
void CAN_0x200_SendData(CAN_HandleTypeDef *CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void get_moto_measure_6020(moto_measure_t *ptr);
void get_moto_measure_3508(moto_measure_t *ptr);

extern float Frame_rate;
extern float RUN,cha;
void CAN_Sup_senddata(CAN_HandleTypeDef *hcan,uint16_t ID, uint8_t* pData, uint16_t Len);
//板间通信
uint8_t CAN_Senddata(CAN_HandleTypeDef *hcan,uint16_t ID,uint8_t* pData,uint16_t Len);

/*接收到的云台参数结构体*/
typedef struct{
 int16_t Target_Vx,Target_Vy,Target_Vw,chassismode,Vision_Mode;
 uint8_t Write_Msg[8];
}RobotReceive_t;

extern RobotReceive_t Robot_Receive;
/**
 * @brief  	    机器人重启(软重启)
 * @param[in]	None
 * @retval 	    None
 */
void RobotReset(void);
#endif

