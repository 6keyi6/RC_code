#include "bsp_can.h"
//#include "start_task.h"
#include "delay.h" 

ina226_SendData_u sdd;

//���ò�����1Mhz
void mx_can_init(void)
{ 
	GPIO_InitTypeDef 		GPIO_InitStructure; 
	CAN_InitTypeDef        	CAN_InitStructure;
	CAN_FilterInitTypeDef  	CAN_FilterInitStructure;

	NVIC_InitTypeDef  		NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO

	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=DISABLE;			//��ʱ�䴥��ͨ��ģʽ  
	CAN_InitStructure.CAN_ABOM=ENABLE;			//����Զ����߹���	 
	CAN_InitStructure.CAN_AWUM=ENABLE;			//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=ENABLE;			//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;		 	//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=ENABLE;			//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;//ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ; 
	//���ò�����
	//Fpclk1/((tbs1+1+tbs2+1+1)*brp)  36M/((1+6+2)*4) =1Mhz
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=CAN_BS1_6tq; 		//Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;		//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=4;          //��Ƶϵ��(Fdiv)Ϊbrp+1	
	CAN_Init(CAN1, &CAN_InitStructure);        	//��ʼ��CAN1 

	CAN_FilterInitStructure.CAN_FilterNumber=0;	//������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 	//����λģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 	//32λ�� 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0xffff;	//32λID
	CAN_FilterInitStructure.CAN_FilterIdLow=0xffff;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffff;//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//���������0

	CAN_FilterInit(&CAN_FilterInitStructure);			//�˲�����ʼ��
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);				//FIFO0��Ϣ�Һ��ж�����.		    

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}   

//�жϷ�����			    
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  	CanRxMsg RxMessage;
//	int i=0;
    CAN_Receive(CAN1, 0, &RxMessage);
//	for(i=0;i<8;i++)
//	printf("rxbuf[%d]:%d\r\n",i,RxMessage.Data[i]);
//	if(RxMessage.StdId == 0x200 || RxMessage.StdId == 0x1ff || RxMessage.StdId == 0x2ff)
//	{
//		can_flag++;
//	}
}

CanTxMsg TxMessage_text;
void can_send_msg(s16 mt1,s16 mt2,s16 mt3,s16 mt4)
{
	CanTxMsg TxMessage;
	TxMessage.StdId= CAN_Tx_Msg ;	// ��׼��ʶ�� 
	TxMessage.ExtId= 0;			    // ������չ��ʾ�� 
	TxMessage.IDE=CAN_Id_Standard; 	// ��׼֡
	TxMessage.RTR=CAN_RTR_Data;		// ����֡
	TxMessage.DLC=8;				// Ҫ���͵����ݳ���
    TxMessage.Data[0] = mt1 >> 8;
    TxMessage.Data[1] = mt1;
    TxMessage.Data[2] = mt2 >> 8;
    TxMessage.Data[3] = mt2;
    TxMessage.Data[4] = mt3 >> 8;
    TxMessage.Data[5] = mt3;
    TxMessage.Data[6] = mt4 >> 8;
    TxMessage.Data[7] = mt4;		          
    CAN_Transmit(CAN1, &TxMessage);   
	
//TxMessage_text.StdId= CAN_Tx_Msg ;	// ��׼��ʶ�� 
//TxMessage_text.ExtId= 0;			    // ������չ��ʾ�� 
//TxMessage_text.IDE=CAN_Id_Standard; 	// ��׼֡
//TxMessage_text.RTR=CAN_RTR_Data;		// ����֡
//TxMessage_text.DLC=8;				// Ҫ���͵����ݳ���
//	TxMessage_text.Data[0] = mt1 >> 8;
//	TxMessage_text.Data[1] = mt1;
//	TxMessage_text.Data[2] = mt2 >> 8;
//	TxMessage_text.Data[3] = mt2;
//	TxMessage_text.Data[4] = mt3 >> 8;
//	TxMessage_text.Data[5] = mt3;
//	TxMessage_text.Data[6] = mt4 >> 8;
//	TxMessage_text.Data[7] = mt4;		          
//	CAN_Transmit(CAN1, &TxMessage_text);   
	
}

void can_send_msg_sd(void)
{
	CanTxMsg TxMessage;
	TxMessage.StdId= CAN_Tx_Msg ;	// ��׼��ʶ�� 
	TxMessage.ExtId= 0;			    // ������չ��ʾ�� 
	TxMessage.IDE=CAN_Id_Standard; 	// ��׼֡
	TxMessage.RTR=CAN_RTR_Data;		// ����֡
	TxMessage.DLC=8;				// Ҫ���͵����ݳ���
    TxMessage.Data[0] = sdd.data[0];
    TxMessage.Data[1] = sdd.data[1];
    TxMessage.Data[2] = sdd.data[2];
    TxMessage.Data[3] = sdd.data[3];
    TxMessage.Data[4] = sdd.data[4];
    TxMessage.Data[5] = sdd.data[5];
    TxMessage.Data[6] = sdd.data[6];
    TxMessage.Data[7] = sdd.data[7];   
		
    CAN_Transmit(CAN1, &TxMessage);   
}

//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
u8 Can_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
    for(i=0;i<8;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}














