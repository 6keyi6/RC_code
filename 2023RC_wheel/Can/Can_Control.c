#include "Can_Control.h"
//接口赋值
Can_Data_t Can_Data[2] = Can_DataGroundInit;
#undef Can_DataGroundInit

uint32_t pTxMailbox = 0;
CAN_TxHeaderTypeDef TXHeader;
CAN_RxHeaderTypeDef RXHeader;

uint8_t TxData[8] = {0};//发送电流数据
uint8_t RXmessage[8]; //接收电调返回数据

//接收数据结构体
AbsEncoderData_t AbsEncoderData[4]={0};
moto_measure_t M2006moto[4]={0};
moto_measure_t M3508moto[4]={0};

/**
  * @Data   2021-03-24
  * @brief  CAN筛选器初始化
  * @param  CAN_FilterTypeDef *CAN_Filter, CAN_HandleTypeDef *hcanx
  * @retval void
  */
static void CAN_FILTER_Init(CAN_FilterTypeDef *CAN_Filter, CAN_HandleTypeDef *hcanx)
{
    CAN_Filter->FilterFIFOAssignment = CAN_FILTER_FIFO0; //筛选器被关联到FIFO0
    CAN_Filter->FilterBank = 0;                          //筛选器组0
    CAN_Filter->SlaveStartFilterBank = 0;
    CAN_Filter->FilterMode = CAN_FILTERMODE_IDMASK;   //工作在ID掩码模式
    CAN_Filter->FilterScale = CAN_FILTERSCALE_32BIT;  //筛选器位宽为单个32位。
    CAN_Filter->FilterActivation = CAN_FILTER_ENABLE; //使能筛选器
                                                      /* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
    CAN_Filter->FilterIdHigh = 0x0000;                //要筛选的ID高位
    CAN_Filter->FilterIdLow = 0x0000;                 //要筛选的ID低位
    CAN_Filter->FilterMaskIdHigh = 0x0000;            //筛选器高16位每位不须匹配
    CAN_Filter->FilterMaskIdLow = 0x0000;             //筛选器低16位每位不须匹配
    HAL_CAN_ConfigFilter(hcanx, CAN_Filter);
}
/**
  * @Data   2021-03-24
  * @brief   canx中断启动
  * @param   CAN_HandleTypeDef *hcanx, uint8_t Can_type
  * @retval  void
  */
void CAN_IT_Init(CAN_HandleTypeDef *hcanx, uint8_t Can_type)
{
    uint8_t Canx_type = Can_type - 1;
    /*使能滤波器*/

    CAN_FILTER_Init(&Can_Data[Canx_type].CAN_FilterTypedef.CAN_Filter, hcanx);
    /*启用CAN*/
    HAL_CAN_Start(hcanx);
    /*使能CAN的IT中断*/
    __HAL_CAN_ENABLE_IT(hcanx, CAN_IT_RX_FIFO0_MSG_PENDING); //  CAN_IT_FMP0
}



/***********************************************************************
		接收电调返回的消息
***********************************************************************/
int time1=0;
void HAL_CAN_RxFifo0MsgPendingCallback1(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO0,&RXHeader,RXmessage);//获取数据到RXmessage上

				switch (RXHeader.StdId)
    {
        case 0x201:
        {  time1++;
            static uint8_t i = 0;
            i = RXHeader.StdId - 0x201;
            get_moto_measure(&M3508moto[i]);
					  DevicesUpdate(Frame_CHAS_DRV0);
            break;
        }
					case 0x202:
					{
							static uint8_t i = 0;
							i = RXHeader.StdId - 0x201;
							get_moto_measure(&M3508moto[i]);
							DevicesUpdate(Frame_CHAS_DRV1);
							break;
					}
        case 0x203:
        {
            static uint8_t i = 0;
            i = RXHeader.StdId - 0x201;
            get_moto_measure(&M3508moto[i]);
					  DevicesUpdate(Frame_CHAS_DRV2);
            break;
        }
        case 0x204:
        {
            static uint8_t i = 0;
            i = RXHeader.StdId - 0x201;
            get_moto_measure(&M3508moto[i]);
					  DevicesUpdate(Frame_CHAS_DRV3);
            break;
        }		

        case 0x205:
        {
            static uint8_t i = 0;
            i = RXHeader.StdId - 0x201;
            get_moto_measure(&M2006moto[0]);
					  DevicesUpdate(Frame_CHAS_DRV0);
            break;
        }
				case 0x206:
				{
						static uint8_t i = 0;
						i = RXHeader.StdId - 0x201;
						get_moto_measure(&M2006moto[1]);
						DevicesUpdate(Frame_CHAS_DRV1);
						break;
				}
        case 0x207:
        {
            static uint8_t i = 0;
            i = RXHeader.StdId - 0x201;
            get_moto_measure(&M2006moto[2]);
					  DevicesUpdate(Frame_CHAS_DRV2);
            break;
        }
        case 0x208:
        {
            static uint8_t i = 0;
            i = RXHeader.StdId - 0x201;
            get_moto_measure(&M2006moto[3]);
					  DevicesUpdate(Frame_CHAS_DRV3);
            break;
        }	
    }	

}
int time=0;
void HAL_CAN_RxFifo0MsgPendingCallback2(CAN_HandleTypeDef *hcan)//接受邮箱0挂起中断回调函数
{
		HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO0,&RXHeader,RXmessage);//获取数据到RXmessage上
		switch (RXHeader.StdId)
    {
         case 0x01:
        {
            AngleUpData(&AbsEncoderData[0]);
					time++;
            break;
        }   
				 case 0x02:
        {
            AngleUpData(&AbsEncoderData[1]);
            break;
        }  
				 case 0x03:
        {
            AngleUpData(&AbsEncoderData[2]);
            break;
        }  
				 case 0x04:
        {
            AngleUpData(&AbsEncoderData[3]);
            break;
        }  
			
    }

}



//通用4个位指令
//长度len 设备ID 指令FUNC 数据DATA
void CAN_ABS_SendData(CAN_HandleTypeDef *hcan,uint8_t len,uint8_t ID,uint8_t FUNC,uint8_t DATA)
{
    uint8_t TxDaTa[4]={0};
		TxDaTa[0]=len;
    TxDaTa[1]=ID;
    TxDaTa[2]=FUNC;
    TxDaTa[3]=DATA;	
		TXHeader.StdId=ID;
		TXHeader.ExtId=ID;
		TXHeader.DLC=0x04;
		TXHeader.IDE=0;
		TXHeader.RTR=0;
		TXHeader.TransmitGlobalTime = DISABLE;
		HAL_CAN_AddTxMessage(hcan,&TXHeader,TxDaTa,&pTxMailbox);//发送信息
}
//设置编码器自动回传时间
//数值范围：50~65535
void CAN_ABS_SendReturnTime(CAN_HandleTypeDef *hcan,int16_t len,int16_t ID,int16_t FUNC,int16_t DATA1,int16_t DATA2)
{
    uint8_t TxDaTa[5]={0};
		TxDaTa[0]=len;
    TxDaTa[1]=ID;
    TxDaTa[2]=FUNC;
    TxDaTa[3]=DATA1;	
	  TxDaTa[4]=DATA2;
		TXHeader.StdId=ID;
		TXHeader.ExtId=ID;
		TXHeader.DLC=len;
		TXHeader.IDE=CAN_ID_STD;
		TXHeader.RTR=CAN_RTR_DATA;
		TXHeader.TransmitGlobalTime = DISABLE;
		HAL_CAN_AddTxMessage(hcan,&TXHeader,TxDaTa,&pTxMailbox);//发送信息
}

void CAN_ABS_Send(CAN_HandleTypeDef *hcan,uint8_t len,uint8_t ID,uint8_t FUNC,uint8_t DATA)
{
    uint8_t TxDaTa[7]={0};
		TxDaTa[0]=0x07;
    TxDaTa[1]=ID;
    TxDaTa[2]=0x0D;
    TxDaTa[3]=0x00;	
    TxDaTa[4]=0x01;
    TxDaTa[5]=0x90;	
    TxDaTa[6]=0x00;

		TXHeader.StdId=ID;
		TXHeader.ExtId=ID;
		TXHeader.DLC=0x07;
		TXHeader.IDE=0;
		TXHeader.RTR=0;
		TXHeader.TransmitGlobalTime = DISABLE;
		HAL_CAN_AddTxMessage(hcan,&TXHeader,TxDaTa,&pTxMailbox);//发送信息
}

//电流发送函数
void CAN_0x200Send(CAN_HandleTypeDef *hcan,int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4)
{
		TxData[0]=iq1>>8;
		TxData[1]=iq1;
		TxData[2]=iq2>>8;
		TxData[3]=iq2;
		TxData[4]=iq3>>8;
		TxData[5]=iq3;
		TxData[6]=iq4>>8;
	  TxData[7]=iq4;
		TXHeader.StdId=0x200;
		TXHeader.ExtId=0x200;
		TXHeader.DLC=0X08;
		TXHeader.IDE=CAN_ID_STD;
		TXHeader.RTR=CAN_RTR_DATA;
		TXHeader.TransmitGlobalTime = DISABLE;
		HAL_CAN_AddTxMessage(hcan,&TXHeader,TxData,&pTxMailbox);//发送信息
}
//电流发送函数
void CAN_0x1FFSend(CAN_HandleTypeDef *hcan,int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4)
{
		TxData[0]=iq1>>8;
		TxData[1]=iq1;
		TxData[2]=iq2>>8;
		TxData[3]=iq2;
		TxData[4]=iq3>>8;
		TxData[5]=iq3;
		TxData[6]=iq4>>8;
	  TxData[7]=iq4;
		TXHeader.StdId=0x1FF;
		TXHeader.ExtId=0x1FF;
		TXHeader.DLC=0X08;
		TXHeader.IDE=CAN_ID_STD;
		TXHeader.RTR=CAN_RTR_DATA;
		TXHeader.TransmitGlobalTime = DISABLE;
		HAL_CAN_AddTxMessage(hcan,&TXHeader,TxData,&pTxMailbox);//发送信息
}


void AngleUpData(AbsEncoderData_t *Data)
{
	 Data->lastAngle = Data->realAngle;
	 Data->realAngle =(uint32_t)(RXmessage[5]<<16 | RXmessage[4]<<8 |	RXmessage[3]);
//	 Data->realAngle = Data->realAngle-102400;
//	if((int32_t)(Data->realAngle - Data->lastAngle) < -10000)
//	{
//		Data->encode_count ++;
//	}
//	else if((int32_t)(Data->lastAngle - Data->realAngle) < -10000)
//	{
//		Data->encode_count --;
//	}
//		 
//	 Data->totolAngle = Data->realAngle + (Data->encode_count * 16384);//24为目前编码器可积累圈数
}


void get_moto_measure(moto_measure_t *ptr)
{
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(RXmessage[0]<<8 | RXmessage[1]) ;
	ptr->speed_rpm  = (int16_t)(RXmessage[2]<<8 | RXmessage[3]);
	ptr->real_current = (RXmessage[4]<<8 | RXmessage[5])*5.f/16384.f;
	ptr->hall = RXmessage[6];
	if(ptr->angle - ptr->last_angle < -6500)
		ptr->round_cnt ++;
	else if (ptr->last_angle - ptr->angle < -6500)
		ptr->round_cnt --;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle;
}