#ifndef _BSP_CAN_H
#define _BSP_CAN_H
#include "sys.h"	
#include "ina226_task.h"
							    
#define CAN_Tx_Msg 0X0301			

typedef union
{
	uint8_t data[8];
	struct
	{
		float voltageVal;//mV
		float Shunt_Current;//mA
	};
} ina226_SendData_u;
extern ina226_SendData_u sdd;

void mx_can_init(void);
void can_send_msg(s16 mt1,s16 mt2,s16 mt3,s16 mt4);						//发送数据
void can_send_msg_32(u32 mt1,u32 mt2,u32 mt3,u32 mt4);						//发送数据
u8 Can_Receive_Msg(u8 *buf);							//接收数据
void can_send_msg_sd(void);
#endif
