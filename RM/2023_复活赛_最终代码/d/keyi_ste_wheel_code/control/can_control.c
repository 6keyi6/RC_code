#include "can_control.h"
#include "M6020_Motor.h"
#include "Devices_Monitor.h"
#include "DEV_SuperCapacitor.h"
#include "Power_Meter.h"
#include "Chassis_control.h"
#include "Robot_control.h"
/***********************************接口赋值************************************/
Can_Data_t Can_Data[2] = Can_DataGroundInit;
#undef Can_DataGroundInit
/*******************************************************************************/
#define CAN_LINE_BUSY  0
#define CAN_SUCCESS    1
RobotReceive_t Robot_Receive;

float Frame_rate;

uint8_t TxData[8] = {0};//发送电流数据
uint8_t RXmessage[8]; //接收电调返回数据

uint32_t pTxMailbox = 0;
moto_measure_t moto_chassis[4] = {0};//4 chassis moto
moto_measure_t  M6020moto_chassis[4] = {0} ;

moto_measure_t M3508moto_chassis[4] = {0};
CAN_TxHeaderTypeDef TXHeader;
CAN_RxHeaderTypeDef RXHeader;

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



/*************************************
* Method:    CAN_0x200_SendData
* Returns:   void
* Parameter: int16_t iq1，参数值。
* 说明：0x200报文的统一发送函数—— 控制3508的ID：1-4电机，2006的1-4
************************************/
void CAN_0x200_SendData(CAN_HandleTypeDef *CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{

//	uint8_t data[8];

//	//数据格式详见说明书P32
//	data[0] = iq1 >> 8;
//	data[1] = iq1;
//	data[2] = iq2 >> 8;
//	data[3] = iq2;
//	data[4] = iq3 >> 8;
//	data[5] = iq3;
//	data[6] = iq4 >> 8;
//	data[7] = iq4;

//  CAN_SendData(CAN_SendHandle, CAN_Num, CAN_ID_STD, 0x200, data);
}

//void Dial_SendData(CAN_HandleTypeDef *CAN_Num, int16_t iq3)
//{
//	uint8_t data[8];
//	data[4] = iq3 >> 8;
//	data[5] = iq3;
//	CAN_SendData(CAN_SendHandle, CAN_Num, CAN_ID_STD, 0x200, data);
//}
/*************************************
* Method:    CAN_0x1FF_SendData
* Returns:   void
* Parameter: int16_t iq1，参数值。
* 说明：0x1FF报文的统一发送函数—— 控制3508的ID：5-8电机，6020的1-4，2006的5-8
************************************/
void CAN_0x1FF_SendData(CAN_HandleTypeDef *CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{

//	uint8_t data[8];
//	data[0] = iq1 >> 8;
//	data[1] = iq1;
//	data[2] = iq2 >> 8;
//	data[3] = iq2;
//	data[4] = iq3 >> 8;
//	data[5] = iq3;
//	data[6] = iq4 >> 8;
//	data[7] = iq4;

//   CAN_SendData(CAN_SendHandle, CAN_Num, CAN_ID_STD, 0x1FF, data);
}






/***********************************************************************
		接收电调返回的消息
***********************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback1(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO0,&RXHeader,RXmessage);//获取数据到RXmessage上
		switch (RXHeader.StdId)
    {
			  case 0x340:
				{
					WriteMsgFromGimbal();
					DevicesUpdate(Frame_COMMU_0X340);
					Frame_rate++;
					break;
				}
        case 0x201:
        {
            static uint8_t i = 0;
            i = RXHeader.StdId - 0x201;
            get_moto_measure_3508(&M3508moto_chassis[i]);
					  DevicesUpdate(Frame_CHAS_DRV0);
            break;
        }
					case 0x202:
					{
							static uint8_t i = 0;
							i = RXHeader.StdId - 0x201;
							get_moto_measure_3508(&M3508moto_chassis[i]);
							DevicesUpdate(Frame_CHAS_DRV1);
							break;
					}
        case 0x203:
        {
            static uint8_t i = 0;
            i = RXHeader.StdId - 0x201;
            get_moto_measure_3508(&M3508moto_chassis[i]);
					  DevicesUpdate(Frame_CHAS_DRV2);
            break;
        }
        case 0x204:
        {
            static uint8_t i = 0;
            i = RXHeader.StdId - 0x201;
            get_moto_measure_3508(&M3508moto_chassis[i]);
					  DevicesUpdate(Frame_CHAS_DRV3);
            break;
        }		

        case 0x600:			
				{
					SupCap_Update(RXmessage);
					DevicesUpdate(Frame_SUPCAP);
					break;
				}					
        case 0x301:			
				{
					Power_Update(RXmessage);
					DevicesUpdate(Frame_PowerMeter);
					break;
				}	
    }	
}
void HAL_CAN_RxFifo0MsgPendingCallback2(CAN_HandleTypeDef *hcan)//接受邮箱0挂起中断回调函数
{
		HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO0,&RXHeader,RXmessage);//获取数据到RXmessage上
	
		switch (RXHeader.StdId)
    {

        case CAN_6020Moto1_ID:
        {
            static uint8_t i = 0;
            i = RXHeader.StdId - CAN_6020Moto1_ID;
            get_moto_measure_6020(&M6020moto_chassis[i]);
						DevicesUpdate(Frame_CHAS_RUD0);
            break;
        }
        case CAN_6020Moto2_ID:
        {
            static uint8_t i = 0;
            i = RXHeader.StdId - CAN_6020Moto1_ID;
            get_moto_measure_6020(&M6020moto_chassis[i]);
						DevicesUpdate(Frame_CHAS_RUD1); 
					  break;
        }
				 case CAN_6020Moto3_ID:
        {
            static uint8_t i = 0;
            i = RXHeader.StdId - CAN_6020Moto1_ID;
            get_moto_measure_6020(&M6020moto_chassis[i]);
						DevicesUpdate(Frame_CHAS_RUD2);
					  break;
        }
				 case CAN_6020Moto4_ID:
        {
            static uint8_t i = 0;
            i = RXHeader.StdId - CAN_6020Moto1_ID;
            get_moto_measure_6020(&M6020moto_chassis[i]);
						DevicesUpdate(Frame_CHAS_RUD3);
					  break;
        }
    }
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
void get_moto_measure_3508(moto_measure_t *ptr)
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

//获取从云台传输的数据
void WriteMsgFromGimbal(void)
{
			    //--- 底盘目标速度
    Robot_Receive.Target_Vx = ((int16_t)((RXmessage[0]<<8)|RXmessage[1]));
    Robot_Receive.Target_Vy = ((int16_t)((RXmessage[2]<<8)|RXmessage[3]));
    Robot_Receive.Target_Vw = ((int16_t)((RXmessage[4]<<8)|RXmessage[5]));

    //--- 底盘模式(0-3bit)	
	  Robot_Receive.chassismode = (CHAS_CtrlMode_e)(RXmessage[6]&0x0F);; //--- 底盘模式

	    //--- 视觉模式(4-6bit)
    Robot_Receive.Vision_Mode = (VisionMode_e)((RXmessage[6]&0x70)>>4);
	
		for(uint8_t i = 0 ; i < 8 ; i++)//--- UI与设备标志位
		{
				Robot_Receive.Write_Msg[i] = 1&(RXmessage[7]>>i);
		}	
//		if(Robot_Receive.Write_Msg[3] == 1)//对角模式
//		{
//			Robot_Receive.Target_Vw*=0.3;
//		}
		if(DevicesGet_State(COMMU_0X340_MONITOR) == Off_line)//--- 掉线
		{
			Robot_Receive.Target_Vx = Robot_Receive.Target_Vy = Robot_Receive.Target_Vw = 0;
			Robot_Receive.chassismode=0;
		}
}


void CAN_Sup_senddata(CAN_HandleTypeDef *hcan,uint16_t ID, uint8_t* pData, uint16_t Len)
{

		TXHeader.StdId=ID;
		TXHeader.ExtId=ID;
		TXHeader.DLC=Len;
		TXHeader.IDE=CAN_ID_STD;
		TXHeader.RTR=CAN_RTR_DATA;
		TXHeader.TransmitGlobalTime = DISABLE;
		HAL_CAN_AddTxMessage(hcan,&TXHeader,pData,&pTxMailbox);//发送信息
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


/**
 * @brief  	    机器人重启(软重启)
 * @param[in]	None
 * @retval 	    None
 */
void RobotReset(void)
{   
    //--- 芯片复位
    __set_FAULTMASK(1);     //--- 关闭所有中断
    HAL_NVIC_SystemReset(); //--- 复位
}