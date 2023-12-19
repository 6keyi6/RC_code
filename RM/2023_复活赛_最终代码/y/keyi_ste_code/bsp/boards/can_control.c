#include "can_control.h"

#include "M6020_Motor.h"
#include "Devices_Monitor.h"
#include "Robot_control.h"
#define CAN_LINE_BUSY  0
#define CAN_SUCCESS    1
/***********************************接口赋值************************************/
Can_Data_t Can_Data[2] = Can_DataGroundInit;
#undef Can_DataGroundInit

/*******************************************************************************/
#define M3508_PowerL_1 0x201
#define M3508_PowerR_2 0x202
#define M2006_READID_START 0x203

uint8_t TxData[8] = {0};//发送电流数据
uint8_t RXmessage[8]; //接收电调返回数据

uint32_t pTxMailbox = 0;
moto_measure_t moto_chassis[4] = {0};//4 chassis moto
moto_measure_t  M6020moto_chassis[4] = {0} ;
moto_measure_t M3508_Friction[2];
CAN_TxHeaderTypeDef TXHeader;
CAN_RxHeaderTypeDef RXHeader;

moto_measure_t M2006_Reload;

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
    CAN_Filter->SlaveStartFilterBank =0;
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
void HAL_CAN_RxFifo0MsgPendingCallback_1(CAN_HandleTypeDef *hcan)//接受邮箱0挂起中断回调函数
{
		HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO0,&RXHeader,RXmessage);//获取数据到RXmessage上
	
		switch (RXHeader.StdId)
    {
        case CAN_6020Moto1_ID:
        {
            static uint8_t i = 0;
            i = RXHeader.StdId - CAN_6020Moto1_ID;
            get_moto_measure_6020(&M6020moto_chassis[i]);
					  DevicesUpdate(Frame_GIMBAL_YAW);
            break;
        }
        case CAN_6020Moto4_ID:
        {
            static uint8_t i = 0;
            i = RXHeader.StdId - CAN_6020Moto1_ID;
            get_moto_measure_6020(&M6020moto_chassis[i]);
						DevicesUpdate(Frame_GIMBAL_PIT);
            break;
        }
	        case 0x341://接收裁判系统数据
				{
					  RefereeMsg_Write(RXmessage);
						DevicesUpdate(Frame_COMMU_0X341);
						break;
				}
    }
}
void HAL_CAN_RxFifo0MsgPendingCallback_2(CAN_HandleTypeDef *hcan)//接受邮箱0挂起中断回调函数
{
		HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO0,&RXHeader,RXmessage);//获取数据到RXmessage上
	
		switch (RXHeader.StdId)
    {

        case M3508_PowerL_1:
        {
            static uint8_t i = 0;
            i = RXHeader.StdId - M3508_PowerL_1;
            get_moto_measure(&M3508_Friction[i]);
            DevicesUpdate(Frame_FRIC_L);
            break;
        }
        case M3508_PowerR_2:
        {
            static uint8_t i = 0;
            i = RXHeader.StdId - M3508_PowerL_1;
            get_moto_measure(&M3508_Friction[i]);
					  DevicesUpdate(Frame_FRIC_R);
            break;
        }				
        case M2006_READID_START:
        {
            get_moto_measure_2006(&M2006_Reload);
					  DevicesUpdate(Frame_RELOAD);
            break;
        }			

//        case 0x066://接收裁判系统数据				
//				{
//					  RefereeMsg_Write_1(RXmessage);
//					  break;
//				}
    }
}


/********************************************************************
    解码电调返回的数据
********************************************************************/
void get_moto_measure(moto_measure_t *ptr)
{

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(RXmessage[0]<<8 | RXmessage[1]) ;
	ptr->speed_rpm  = (int16_t)(RXmessage[2]<<8 | RXmessage[3]);
	ptr->real_current = (RXmessage[4]<<8 | RXmessage[5])*5.f/16384.f;
	ptr->hall = RXmessage[6];

}


void get_moto_measure_2006(moto_measure_t *ptr)
{
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(RXmessage[0]<<8 | RXmessage[1]) ;
	ptr->speed_rpm  = (int16_t)(RXmessage[2]<<8 | RXmessage[3]);
	ptr->real_current = (RXmessage[4]<<8 | RXmessage[5])*5.f/16384.f;
	ptr->hall = RXmessage[6];
	if(ptr->angle - ptr->last_angle < -6000)
		ptr->round_cnt ++;
	else if (ptr->last_angle - ptr->angle < -6000)
		ptr->round_cnt --;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle;
}
void get_moto_measure_6020(moto_measure_t *ptr)
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

//电流发送函数
void CAN_senddata(CAN_HandleTypeDef *hcan,int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4)
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
void CAN_senddata_6020(CAN_HandleTypeDef *hcan,int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4)
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
//板间通信
uint8_t CAN_Senddata(CAN_HandleTypeDef *hcan,uint16_t ID,uint8_t* pData,uint16_t Len)
{
		static CAN_TxHeaderTypeDef Tx_Header;
	uint32_t used_mailbox;
	/* Check the parameters */
	assert_param(hcan != NULL);

	Tx_Header.StdId = ID;
	Tx_Header.ExtId = ID;
	Tx_Header.IDE = 0;
	Tx_Header.RTR = 0;
	Tx_Header.DLC = Len;

	if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, &used_mailbox) != HAL_OK)
	{
		return CAN_LINE_BUSY;
	}
	else {}

	return CAN_SUCCESS;
}
