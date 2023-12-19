#include "usart.h"
#include "ANO_DT.h"


////数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
//#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)) )
//#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
//#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
//#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
//	

//u8 BUFF[30];

//void sent_data(u16 A,u16 B,u8 C)
//{
//	int i;
//	u8 sumcheck = 0;
//	u8 addcheck = 0;
//	u8 _cnt=0;
//	BUFF[_cnt++]=0xAA;//帧头
//	BUFF[_cnt++]=0xFF;//目标地址
//	BUFF[_cnt++]=0XF1;//功能码
//	BUFF[_cnt++]=0x05;//数据长度
//	BUFF[_cnt++]=BYTE0(A);//数据内容,小段模式，低位在前
//	BUFF[_cnt++]=BYTE1(A);//需要将字节进行拆分，调用上面的宏定义即可。
//	BUFF[_cnt++]=BYTE0(B);
//	BUFF[_cnt++]=BYTE1(B);	
//	BUFF[_cnt++]=C;
//	//SC和AC的校验直接抄最上面上面简介的即可
//	for(i=0;i<BUFF[3]+4;i++) 
//	{
//		sumcheck+=BUFF[i];
//		addcheck+=sumcheck;
//	}
//	BUFF[_cnt++]=sumcheck;	
//	BUFF[_cnt++]=addcheck;	
//	ANO_DT_Send_Data(BUFF, _cnt);
//}
// 

///*Send_Data函数是协议中所有发送数据功能使用到的发送函数*/
////移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数，这里就采用有线连接，发送至串口2了

//void ANO_DT_Send_Data(uint8_t *dataToSend , uint8_t length)
//{
//	Usart_Send(BUFF, length);
//}

//void Usart_Send(uint8_t *data, uint8_t length)
//{
//	uint8_t  i;
//	for(i=0;i<length;i++) 
//	{
////	 USART_SendData(USART1, *(data+i));
//		HAL_UART_Transmit(&huart1, &data[i], 1, 0xffff);
////	 while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
////	 {}
//	}
//}
//#include "usart.h"
//#include "ANO_DT.h"


//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
	

uint8_t BUFF[30];

void Sent_DataU16(USART_TypeDef* USARTx,s16 A,s16 B,s16 C,s16 D)
{
	int i=0;
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint8_t _cnt=0;
	BUFF[_cnt++]=0xAA;//帧头
	BUFF[_cnt++]=0xFF;//目标地址
	BUFF[_cnt++]=0XF1;//功能码
	BUFF[_cnt++]=0x08;//数据长度
	BUFF[_cnt++]=BYTE0(A);//数据内容,小段模式，低位在前
	BUFF[_cnt++]=BYTE1(A);//需要将字节进行拆分，调用上面的宏定义即可。
	BUFF[_cnt++]=BYTE0(B);
	BUFF[_cnt++]=BYTE1(B);	
	BUFF[_cnt++]=BYTE0(C);
	BUFF[_cnt++]=BYTE1(C);
	BUFF[_cnt++]=BYTE0(D);
	BUFF[_cnt++]=BYTE1(D);
	//SC和AC的校验直接抄最上面上面简介的即可
	for(i=0;i<_cnt;i++) 
	{
		sumcheck+=BUFF[i];
		addcheck+=sumcheck;
	}
	BUFF[_cnt++]=sumcheck;	
	BUFF[_cnt++]=addcheck;	
	ANO_DT_Send_Data(USARTx,BUFF, _cnt);
}
 
void Sent_DataFloat(USART_TypeDef* USARTx,float A,float B)
{
	int i=0;
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint8_t _cnt=0;
	BUFF[_cnt++]=0xAA;//帧头
	BUFF[_cnt++]=0xFF;//目标地址
	BUFF[_cnt++]=0XF1;//功能码
	BUFF[_cnt++]=0x08;//数据长度
	BUFF[_cnt++]=BYTE0(A);//数据内容,小段模式，低位在前
	BUFF[_cnt++]=BYTE1(A);//需要将字节进行拆分，调用上面的宏定义即可。
	BUFF[_cnt++]=BYTE2(A);//数据内容,小段模式，低位在前
	BUFF[_cnt++]=BYTE3(A);//需要将字节进行拆分，调用上面的宏定义即可。
	BUFF[_cnt++]=BYTE0(B);
	BUFF[_cnt++]=BYTE1(B);	
	BUFF[_cnt++]=BYTE2(B);
	BUFF[_cnt++]=BYTE3(B);

	//SC和AC的校验直接抄最上面上面简介的即可
	for(i=0;i<_cnt;i++) 
	{
		sumcheck+=BUFF[i];
		addcheck+=sumcheck;
	}
	BUFF[_cnt++]=sumcheck;	
	BUFF[_cnt++]=addcheck;	
	ANO_DT_Send_Data(USARTx,BUFF, _cnt);
}


/*Send_Data函数是协议中所有发送数据功能使用到的发送函数*/
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数，这里就采用有线连接，发送至串口2了

void ANO_DT_Send_Data(USART_TypeDef* USARTx,uint8_t *dataToSend , uint8_t length)
{
	uint8_t  i;
	for(i=0;i<length;i++) 
	{
		USARTx->DR=dataToSend[i];
		while(USART_GetFlagStatus(USARTx) == RESET)
		{
		}
	}
}


int USART_GetFlagStatus(USART_TypeDef* USARTx)
{
	int bitstatus=0;
	if ((USARTx->SR & ((uint16_t)0x0040)) != (uint16_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
	return bitstatus;
}


