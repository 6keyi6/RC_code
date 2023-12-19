/**
 * @file Control_Vision.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Control_Vision.h"
#include "DJI_IMU.h"
#include "M6020_Motor.h"
#include "Cloud_control.h"
#include "M6020_Motor.h"
#include "cmsis_os.h"
#include "DJI_IMU.h"
#include "Devices_Monitor.h"
#include "can_control.h"

#include "user_lib.h"
#define PredictQueue_LEN	5
/****************************暂时将所需要的变量写在这里***************************************/
static uint32_t
FPS_Calculate(uint16_t deltaTime)
{                                                  //FPS计算函数。
    return (1.0f / (double)(deltaTime)) * 1000.0f; // 别忘了先转换为浮点数，否则会有精度丢失
}

void Get_FPS(WorldTime_RxTypedef *time, uint32_t *FPS) //获取当前系统时钟节拍并算出FPS
{
    time->WorldTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    *FPS = FPS_Calculate(time->WorldTime - time->Last_WorldTime);
    time->Last_WorldTime = time->WorldTime;
}


/****************************暂时将所需要的变量写在这里***************************************/

Send_Vision_data_t Send_Vision_data;

bool whether_Fire;//传给射击的一个是否打击标志

uint8_t Vision_DataBuff[Vision_BuffSIZE];
VisionData_t VisionData;
VisionSend_IMU_t VisionSend_IMU;
WorldTime_RxTypedef Vision_WorldTime;
WorldTime_RxTypedef VisionKF_TIME;



void Vision_Init(void);
void Update_VisionTarget(void);
void Vision_processing(void);
void Vision_ID_Init(void);
void Vision_Handler(UART_HandleTypeDef *huart);
void Vision_USART_Receive_DMA(UART_HandleTypeDef *huartx);
void Vision_SendBufFunction(float *angle, float *Gyro);
uint8_t GetVisionDiscMode(void);
uint8_t GetVisionHitMode(void);
void Check_Vision(void);
void Vision_CP(float CPData);
VisionExportData_t VisionExportData = VisionExportDataGroundInit;
#undef VisionExportDataGroundInit
Control_Vision_FUN_t Control_Vision_FUN = Control_Vision_FUNGroundInit;
#undef Control_Vision_FUNGroundInit


RoboInfUartBuff_t RoboInfUartBuff = {0};

WorldTime_t Vision_TIME;
float vision_yaw_angle,vision_pit_angle;//最终视觉角度
float last_vision_yaw_angle,last_vision_pit_angle;

float YAW_queue[PredictQueue_LEN];
float PIT_queue[PredictQueue_LEN];
/*
	sprintf(g_buf_temp, "%c %1d %1d %03d %03d %03d", 'S', mode, mode_select,x, y, depth);
	uint8_t CRC = Checksum_CRC8(g_buf_temp, sizeof(g_buf_temp));
	sprintf(g_buf, "%c %1d%1d%03d%03d%03d%03d%c", 'S',mode, mode_select, x, y, depth, CRC, 'E');
*/

//自己颜色：0 红蓝 1红	2 蓝
//模式：0默认，1自瞄，2大神符，3击打哨兵，4小陀螺，5录像，6飞机，7哨兵自瞄，8雷达
//机器人ID：1英雄，2工程，3步兵，6无人机，7哨兵
//通过一个二维数组嵌套保存不同目标的待发送数据，索引对应枚举ShootTarget_e
//uint8_t Vision_SendBuf[13] = {'S', 106, 0, '0', '0', '0', '0', '0', '0', '0', '0', 28,  'E'};//初始化//之前是15 下次改为30


uint8_t Vision_SendBuf[13] = {'S', 104, 0, '0', '0', '0', '0', '0', '0', '0', '0', 28 ,'E'};//初始化

/**
	* @brief  向视觉发送数据函数
 * @param	数据
 * @retval None
 */
int hh[13] = {3};
static void SendVisionData(uint8_t *data)
{
    if (data == NULL)
    {
        return;
    }
    for (int i = 0; i < 13; i++)
    {
        hh[i] = HAL_UART_Transmit(&huart6, &data[i], 1, 0xffff);
    }
}


/**
 * @brief 陀螺仪角度更新
 * 
 * @param angle 
 * @param Gyro 
 */
void Vision_SendBufFunction(float *angle, float *Gyro)
{
    VisionSend_IMU.VisionSend_t.YawAngle_Error = *angle;
    VisionSend_IMU.VisionSend_t.PitchAngle_Error = *Gyro; 
    Vision_SendBuf[3] = VisionSend_IMU.VisionSend_t.Angle_Error_Data[0];
		Vision_SendBuf[4] = VisionSend_IMU.VisionSend_t.Angle_Error_Data[1];
		Vision_SendBuf[5] = VisionSend_IMU.VisionSend_t.Angle_Error_Data[2];
		Vision_SendBuf[6] = VisionSend_IMU.VisionSend_t.Angle_Error_Data[3];
		Vision_SendBuf[7] = VisionSend_IMU.VisionSend_t.Angle_Error_Data[4];
		Vision_SendBuf[8] = VisionSend_IMU.VisionSend_t.Angle_Error_Data[5];
		Vision_SendBuf[9] = VisionSend_IMU.VisionSend_t.Angle_Error_Data[6];
		Vision_SendBuf[10] = VisionSend_IMU.VisionSend_t.Angle_Error_Data[7];
}
/**
 * @brief 更改机器人ID及视觉模式
 * 
 * @param  ID
 * @param  mode
 */
void Update_Vision(uint8_t *ID,uint8_t *mode)
{
    Vision_SendBuf[1] = *ID;
		Vision_SendBuf[2] = *mode;	
}
/**
 * @brief 发送敌方最低血量机器人 ID 血量
 * @param  
 * @param  
 */
void minimum_HP(void)
{
//    Vision_SendBuf[12] = 1;
//		Vision_SendBuf[13] = 66;	
}
/**
 * @brief 清除敌方最低血量机器人 ID 血量
 * @param  
 * @param  
 */
void minimum_HP_clean(void)
{
//    Vision_SendBuf[12] = 0;
//		Vision_SendBuf[13] = 0;	
}

/**
 * @brief 发送实时弹速
 * @param  
 * @param  
 */
void FiringRate(float *rate)
{
	Vision_SendBuf[11] = *rate;
}
/**
 * @brief 更新数据
 * @param  
 * @param  
 */
uint8_t mode;
void Update_VisionTarget(void)
{   
	  mode = Vision_Mode;
	  Vision_SendBufFunction(&DJIC_IMU.total_yaw , &DJIC_IMU.pitch);//陀螺仪数据发送
	  Update_Vision(&ID,&mode);//机器人ID及视觉模式
		SendVisionData(Vision_SendBuf);//发送信息给视觉
}

/**
 * @brief 视觉数据采样
 * 
 */
void Vision_processing(void)
{
	vision_yaw_angle = VisionData.RawData.yaw_angle;
	vision_pit_angle = VisionData.RawData.pitch_angle;


//	if(Vision_Mode == Vision_BIGWindmill)//打符模式下进行滑动平均滤波
//	{
//		vision_yaw_angle = SlopeFilter_Calc(VisionData.RawData.yaw_angle,YAW_queue,5);
//		vision_pit_angle = SlopeFilter_Calc(VisionData.RawData.pitch_angle,PIT_queue,5);
//	}	
	
}

/**
 * @brief 视觉接收
 * 
 * @param data 
 */

uint8_t yaw_nan , pit_nan;//硬件nan值计数
uint8_t vision_error = 0;//数据错误标志位
float sz[10];
void Vision_DataReceive(uint8_t *data)
{
	//--- 不校验帧头帧尾以及crc位
    uint8_t CRCBuffer = Checksum_CRC8(data+1, 13 - 3);
	
#if Vision_Version == Vision_Oldversion//旧视觉
   //已删掉
#elif Vision_Version == Vision_Newversion//新

    for(int i = 0; i < 13; i++)
    {
        VisionData.RawData.VisionRawData[i] = data[i];
    }

    if (CRCBuffer != VisionData.RawData.crc || VisionData.RawData.Start_Tag != 'S' || VisionData.RawData.End_Tag != 'E') //CRC校验失败。
    {
			vision_error = 1;
			VisionData.RawData.yaw_angle = VisionData.RawData.pitch_angle = 0;
      return;
    }

    VisionData.InfoUpdateFrame++; //帧率
    VisionData.InfoUpdateFlag = 1;
    Get_FPS(&Vision_WorldTime, &VisionData.FPS);
//     KalmanFilter(&pit_vision,VisionData.RawData.pitch_angle );
//     KalmanFilter(&yaw_vision, VisionData.RawData.yaw_angle);	 
		if(DevicesGet_State(VISION_MONITOR) == Off_line)
		{
        VisionData.RawData.pitch_angle = 0;
			  VisionData.RawData.yaw_angle = 0;			
		}
    if(VisionData.RawData.mode == 0 ) //视觉找不到目标
    {
			  VisionData.RawData.pitch_angle = 0;
			  VisionData.RawData.yaw_angle = 0;
        return;
    }
		
      //硬件NAN值错误！！
			VisionData.RawData.yaw_angle = isnan(VisionData.RawData.yaw_angle)==true ? 0 : VisionData.RawData.yaw_angle;
     	VisionData.RawData.pitch_angle = isnan(VisionData.RawData.pitch_angle)==true ? 0 : VisionData.RawData.pitch_angle;
		 	
	 		if(VisionData.Final_Offset.x > 50 || VisionData.Final_Offset.y > 50)//没有开小电脑前传输错误数据
			{
				vision_error = 1;
			}
			else
			{
				vision_error = 0;
			}
			if(VisionData.RawData.whether_Fire != 0)
			{
				vision_error = 1;
			}
			else
     {
			 vision_error = 0;
		 }

#endif
}



/*一个是读取SR寄存器，一个是读取对应的CR控制寄存器*/
/*正常情况下CR控制SR，我们入股要读取对应的标志位的话，既可以从CR读取也可以从SR读取*/
/*__HAL_UART_GET_FLAG是获取SR寄存器里面的情况，也就是读取被CR控制器控制之后的对应状态*/
/*__HAL_UART_GET_IT_SOURCE是直接读取控制寄存器里面的CRx标志位的情况*/
/*这里的DMA_GET_COUNTER是获取还没发出去的字符数量，和之前的不同*/
/*下面是两种情况的对比，请仔细阅读*/
/**
 * @Data    2019-03-23 20:07
 * @brief   视觉处理函数
 * @param   uint8_t *pData
 * @retval  void
 */
void Vision_Handler(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_DMA_DISABLE(huart->hdmarx);

    Vision_DataReceive(Vision_DataBuff);
    DevicesUpdate(Frame_VISION);
    __HAL_DMA_SET_COUNTER(huart->hdmarx, Vision_BuffSIZE);
    __HAL_DMA_ENABLE(huart->hdmarx);
}

/**
 * @brief USART_DMA接收开启和重定向
 * 
 * @param huart 
 * @param pData 
 * @param Size 
 * @return  
 */
static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
{

    /*检测当前huart状态*/
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        /*输入的地址或者数据有问题的话*/
        if ((pData == NULL) || (Size == 0))
        {
            return HAL_ERROR;
        }

        /*huart里面对应的Rx变量重定向*/
        huart->pRxBuffPtr = pData;
        huart->RxXferSize = Size;
        huart->ErrorCode = HAL_UART_ERROR_NONE;

        /*开启huart1上的RX_DMA*/
        HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

        /*只开启对应DMA上面的Rx功能（如果是开启Tx的话就是USART_CR3_DMAT）*/
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    }
    else
    {
        return HAL_BUSY;
    }

    return HAL_OK;
}

/**
 * @brief 视觉使能DMA-USART
 * 
 * @param huartx 
 */
void Vision_USART_Receive_DMA(UART_HandleTypeDef *huartx)
{
    /*清空标志位然后使能USART的中断*/
    __HAL_UART_CLEAR_IDLEFLAG(huartx);
    __HAL_UART_ENABLE(huartx);
    __HAL_UART_ENABLE_IT(huartx, UART_IT_IDLE);
    // assert(Vision_BuffSIZE == (13 + 2));
    USART_Receive_DMA_NO_IT(huartx, Vision_DataBuff, Vision_BuffSIZE);
}


/**************************************************以下函数没有用到****************************************************************************/
/**
 * @brief Get the Vision Disc Mode object
 * 
 * @return  
 */
uint8_t GetVisionDiscMode(void)
{
    return VisionData.RawData.mode;
}

/**
 * @brief Get the Vision Hit Mode object
 * 
 * @return  
 */
uint8_t GetVisionHitMode(void)
{
    uint8_t HitMode = 0;
    if (Robot.Attack_ShootTarget == ShootTarget_Self_aiming /*|| Robot.Attack_ShootTarget == ShootTarget_Sentry*/)
    {
        HitMode = 1;
    }
    else
    {
        HitMode = 0;
    }

    return HitMode;
}
/**
 * @brief 视觉检测
 * 
 */
void Check_Vision(void)
{
    //视觉自瞄 ---------------------------
    if (VisionData.InfoUpdateFrame < 1)
    {
        VisionData.OffLineFlag = 1;
    }
    else
    {
        VisionData.OffLineFlag = 0;
    }
    VisionData.InfoUpdateFrame = 0;
}

/**
 * @brief 视觉pitch补偿变化
 * 
 * @param CPData 
 */
void Vision_CP(float CPData)
{
    VisionData.Gravity_Offset.y += CPData;
}

/**
 * @brief 获取视觉pitch补偿变化
 * 
 * @return  
 */
float GetVision_CP(void)
{
    return VisionData.Gravity_Offset.y;
}


void Vision_Init(void)
{

}
void Vision_ID_Init(void)
{
	
}

