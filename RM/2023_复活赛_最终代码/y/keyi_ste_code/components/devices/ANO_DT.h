//#ifndef __ANO_DT_H__
//#define __ANO_DT_H__

//void Test_Send_User(u16 data1, u8 data2, s16 data3);
//void ANO_DT_Send_Data(uint8_t *dataToSend , uint8_t length);
//void Usart_Send(uint8_t *data, uint8_t length);
//void sent_data(u16 A,u16 B,u8 C);


//#endif

#include "usart.h"


#ifndef __ANO_DT_H__
#define __ANO_DT_H__
#include <stdint.h>
#include "Robot_control.h"

int USART_GetFlagStatus(USART_TypeDef* USARTx);
void Test_Send_User(u16 data1, u8 data2, s16 data3);
void ANO_DT_Send_Data(USART_TypeDef* USARTx,uint8_t *dataToSend , uint8_t length);
void Usart_Send(uint8_t *data, uint8_t length);
void Sent_DataU16(USART_TypeDef* USARTx,s16 A,s16 B,s16 C,s16 D);
void Sent_DataFloat(USART_TypeDef* USARTx,float A,float B);
#endif

