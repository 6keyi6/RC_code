#include "usart.h"
#include "ANO_DT.h"


////���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ������int16��float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з���
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
//	BUFF[_cnt++]=0xAA;//֡ͷ
//	BUFF[_cnt++]=0xFF;//Ŀ���ַ
//	BUFF[_cnt++]=0XF1;//������
//	BUFF[_cnt++]=0x05;//���ݳ���
//	BUFF[_cnt++]=BYTE0(A);//��������,С��ģʽ����λ��ǰ
//	BUFF[_cnt++]=BYTE1(A);//��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
//	BUFF[_cnt++]=BYTE0(B);
//	BUFF[_cnt++]=BYTE1(B);	
//	BUFF[_cnt++]=C;
//	//SC��AC��У��ֱ�ӳ�������������ļ���
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

///*Send_Data������Э�������з������ݹ���ʹ�õ��ķ��ͺ���*/
////��ֲʱ���û�Ӧ��������Ӧ�õ����������ʹ�õ�ͨ�ŷ�ʽ��ʵ�ִ˺���������Ͳ����������ӣ�����������2��

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


//���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ������int16��float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з���
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
	BUFF[_cnt++]=0xAA;//֡ͷ
	BUFF[_cnt++]=0xFF;//Ŀ���ַ
	BUFF[_cnt++]=0XF1;//������
	BUFF[_cnt++]=0x08;//���ݳ���
	BUFF[_cnt++]=BYTE0(A);//��������,С��ģʽ����λ��ǰ
	BUFF[_cnt++]=BYTE1(A);//��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	BUFF[_cnt++]=BYTE0(B);
	BUFF[_cnt++]=BYTE1(B);	
	BUFF[_cnt++]=BYTE0(C);
	BUFF[_cnt++]=BYTE1(C);
	BUFF[_cnt++]=BYTE0(D);
	BUFF[_cnt++]=BYTE1(D);
	//SC��AC��У��ֱ�ӳ�������������ļ���
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
	BUFF[_cnt++]=0xAA;//֡ͷ
	BUFF[_cnt++]=0xFF;//Ŀ���ַ
	BUFF[_cnt++]=0XF1;//������
	BUFF[_cnt++]=0x08;//���ݳ���
	BUFF[_cnt++]=BYTE0(A);//��������,С��ģʽ����λ��ǰ
	BUFF[_cnt++]=BYTE1(A);//��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	BUFF[_cnt++]=BYTE2(A);//��������,С��ģʽ����λ��ǰ
	BUFF[_cnt++]=BYTE3(A);//��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
	BUFF[_cnt++]=BYTE0(B);
	BUFF[_cnt++]=BYTE1(B);	
	BUFF[_cnt++]=BYTE2(B);
	BUFF[_cnt++]=BYTE3(B);

	//SC��AC��У��ֱ�ӳ�������������ļ���
	for(i=0;i<_cnt;i++) 
	{
		sumcheck+=BUFF[i];
		addcheck+=sumcheck;
	}
	BUFF[_cnt++]=sumcheck;	
	BUFF[_cnt++]=addcheck;	
	ANO_DT_Send_Data(USARTx,BUFF, _cnt);
}


/*Send_Data������Э�������з������ݹ���ʹ�õ��ķ��ͺ���*/
//��ֲʱ���û�Ӧ��������Ӧ�õ����������ʹ�õ�ͨ�ŷ�ʽ��ʵ�ִ˺���������Ͳ����������ӣ�����������2��

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


