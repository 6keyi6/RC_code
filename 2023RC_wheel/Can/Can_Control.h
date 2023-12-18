#ifndef __CAN_CONTROL_
#define __CAN_CONTROL_
#include "can.h"
#include "cmsis_os.h"
#include "Devices_Monitor.h"
#define Can_DataGroundInit \
    {                      \
        {0}, {0},          \
    }
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


typedef struct
{
	uint32_t realAngle;
  uint32_t lastAngle;
  int32_t totolAngle;
	
	int32_t encode_count;//这次的圈数
}AbsEncoderData_t;
extern AbsEncoderData_t AbsEncoderData[4];

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
	int cyclinder_number;
}moto_measure_t;
extern moto_measure_t M2006moto[4];
extern moto_measure_t M3508moto[4];

typedef struct{
 int16_t Target_Vx,Target_Vy,Target_Vw,chassismode,Vision_Mode;
 uint8_t Write_Msg[8];
}RobotReceive_t;
extern RobotReceive_t Robot_Receive;
//过滤器初始化
void CAN_IT_Init(CAN_HandleTypeDef *hcanx, uint8_t Can_type);

//中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback1(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback2(CAN_HandleTypeDef *hcan);

//布瑞特绝对值编码器通用指令
void CAN_ABS_SendData(CAN_HandleTypeDef *hcan,uint8_t len,uint8_t ID,uint8_t FUNC,uint8_t DATA);
void CAN_ABS_SendReturnTime(CAN_HandleTypeDef *hcan,int16_t len,int16_t ID,int16_t FUNC,int16_t DATA1,int16_t DATA2);
void CAN_ABS_Send(CAN_HandleTypeDef *hcan,uint8_t len,uint8_t ID,uint8_t FUNC,uint8_t DATA);
void CAN_0x200Send(CAN_HandleTypeDef *hcan,int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4);
void CAN_0x1FFSend(CAN_HandleTypeDef *hcan,int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4);

void AngleUpData(AbsEncoderData_t *Data);
void get_moto_measure(moto_measure_t *ptr);
#endif

