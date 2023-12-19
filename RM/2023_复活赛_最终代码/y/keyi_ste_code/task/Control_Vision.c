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
/****************************��ʱ������Ҫ�ı���д������***************************************/
static uint32_t
FPS_Calculate(uint16_t deltaTime)
{                                                  //FPS���㺯����
    return (1.0f / (double)(deltaTime)) * 1000.0f; // ��������ת��Ϊ��������������о��ȶ�ʧ
}

void Get_FPS(WorldTime_RxTypedef *time, uint32_t *FPS) //��ȡ��ǰϵͳʱ�ӽ��Ĳ����FPS
{
    time->WorldTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    *FPS = FPS_Calculate(time->WorldTime - time->Last_WorldTime);
    time->Last_WorldTime = time->WorldTime;
}


/****************************��ʱ������Ҫ�ı���д������***************************************/

Send_Vision_data_t Send_Vision_data;

bool whether_Fire;//���������һ���Ƿ�����־

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
float vision_yaw_angle,vision_pit_angle;//�����Ӿ��Ƕ�
float last_vision_yaw_angle,last_vision_pit_angle;

float YAW_queue[PredictQueue_LEN];
float PIT_queue[PredictQueue_LEN];
/*
	sprintf(g_buf_temp, "%c %1d %1d %03d %03d %03d", 'S', mode, mode_select,x, y, depth);
	uint8_t CRC = Checksum_CRC8(g_buf_temp, sizeof(g_buf_temp));
	sprintf(g_buf, "%c %1d%1d%03d%03d%03d%03d%c", 'S',mode, mode_select, x, y, depth, CRC, 'E');
*/

//�Լ���ɫ��0 ���� 1��	2 ��
//ģʽ��0Ĭ�ϣ�1���飬2�������3�����ڱ���4С���ݣ�5¼��6�ɻ���7�ڱ����飬8�״�
//������ID��1Ӣ�ۣ�2���̣�3������6���˻���7�ڱ�
//ͨ��һ����ά����Ƕ�ױ��治ͬĿ��Ĵ��������ݣ�������Ӧö��ShootTarget_e
//uint8_t Vision_SendBuf[13] = {'S', 106, 0, '0', '0', '0', '0', '0', '0', '0', '0', 28,  'E'};//��ʼ��//֮ǰ��15 �´θ�Ϊ30


uint8_t Vision_SendBuf[13] = {'S', 104, 0, '0', '0', '0', '0', '0', '0', '0', '0', 28 ,'E'};//��ʼ��

/**
	* @brief  ���Ӿ��������ݺ���
 * @param	����
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
 * @brief �����ǽǶȸ���
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
 * @brief ���Ļ�����ID���Ӿ�ģʽ
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
 * @brief ���͵з����Ѫ�������� ID Ѫ��
 * @param  
 * @param  
 */
void minimum_HP(void)
{
//    Vision_SendBuf[12] = 1;
//		Vision_SendBuf[13] = 66;	
}
/**
 * @brief ����з����Ѫ�������� ID Ѫ��
 * @param  
 * @param  
 */
void minimum_HP_clean(void)
{
//    Vision_SendBuf[12] = 0;
//		Vision_SendBuf[13] = 0;	
}

/**
 * @brief ����ʵʱ����
 * @param  
 * @param  
 */
void FiringRate(float *rate)
{
	Vision_SendBuf[11] = *rate;
}
/**
 * @brief ��������
 * @param  
 * @param  
 */
uint8_t mode;
void Update_VisionTarget(void)
{   
	  mode = Vision_Mode;
	  Vision_SendBufFunction(&DJIC_IMU.total_yaw , &DJIC_IMU.pitch);//���������ݷ���
	  Update_Vision(&ID,&mode);//������ID���Ӿ�ģʽ
		SendVisionData(Vision_SendBuf);//������Ϣ���Ӿ�
}

/**
 * @brief �Ӿ����ݲ���
 * 
 */
void Vision_processing(void)
{
	vision_yaw_angle = VisionData.RawData.yaw_angle;
	vision_pit_angle = VisionData.RawData.pitch_angle;


//	if(Vision_Mode == Vision_BIGWindmill)//���ģʽ�½��л���ƽ���˲�
//	{
//		vision_yaw_angle = SlopeFilter_Calc(VisionData.RawData.yaw_angle,YAW_queue,5);
//		vision_pit_angle = SlopeFilter_Calc(VisionData.RawData.pitch_angle,PIT_queue,5);
//	}	
	
}

/**
 * @brief �Ӿ�����
 * 
 * @param data 
 */

uint8_t yaw_nan , pit_nan;//Ӳ��nanֵ����
uint8_t vision_error = 0;//���ݴ����־λ
float sz[10];
void Vision_DataReceive(uint8_t *data)
{
	//--- ��У��֡ͷ֡β�Լ�crcλ
    uint8_t CRCBuffer = Checksum_CRC8(data+1, 13 - 3);
	
#if Vision_Version == Vision_Oldversion//���Ӿ�
   //��ɾ��
#elif Vision_Version == Vision_Newversion//��

    for(int i = 0; i < 13; i++)
    {
        VisionData.RawData.VisionRawData[i] = data[i];
    }

    if (CRCBuffer != VisionData.RawData.crc || VisionData.RawData.Start_Tag != 'S' || VisionData.RawData.End_Tag != 'E') //CRCУ��ʧ�ܡ�
    {
			vision_error = 1;
			VisionData.RawData.yaw_angle = VisionData.RawData.pitch_angle = 0;
      return;
    }

    VisionData.InfoUpdateFrame++; //֡��
    VisionData.InfoUpdateFlag = 1;
    Get_FPS(&Vision_WorldTime, &VisionData.FPS);
//     KalmanFilter(&pit_vision,VisionData.RawData.pitch_angle );
//     KalmanFilter(&yaw_vision, VisionData.RawData.yaw_angle);	 
		if(DevicesGet_State(VISION_MONITOR) == Off_line)
		{
        VisionData.RawData.pitch_angle = 0;
			  VisionData.RawData.yaw_angle = 0;			
		}
    if(VisionData.RawData.mode == 0 ) //�Ӿ��Ҳ���Ŀ��
    {
			  VisionData.RawData.pitch_angle = 0;
			  VisionData.RawData.yaw_angle = 0;
        return;
    }
		
      //Ӳ��NANֵ���󣡣�
			VisionData.RawData.yaw_angle = isnan(VisionData.RawData.yaw_angle)==true ? 0 : VisionData.RawData.yaw_angle;
     	VisionData.RawData.pitch_angle = isnan(VisionData.RawData.pitch_angle)==true ? 0 : VisionData.RawData.pitch_angle;
		 	
	 		if(VisionData.Final_Offset.x > 50 || VisionData.Final_Offset.y > 50)//û�п�С����ǰ�����������
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



/*һ���Ƕ�ȡSR�Ĵ�����һ���Ƕ�ȡ��Ӧ��CR���ƼĴ���*/
/*���������CR����SR���������Ҫ��ȡ��Ӧ�ı�־λ�Ļ����ȿ��Դ�CR��ȡҲ���Դ�SR��ȡ*/
/*__HAL_UART_GET_FLAG�ǻ�ȡSR�Ĵ�������������Ҳ���Ƕ�ȡ��CR����������֮��Ķ�Ӧ״̬*/
/*__HAL_UART_GET_IT_SOURCE��ֱ�Ӷ�ȡ���ƼĴ��������CRx��־λ�����*/
/*�����DMA_GET_COUNTER�ǻ�ȡ��û����ȥ���ַ���������֮ǰ�Ĳ�ͬ*/
/*��������������ĶԱȣ�����ϸ�Ķ�*/
/**
 * @Data    2019-03-23 20:07
 * @brief   �Ӿ�������
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
 * @brief USART_DMA���տ������ض���
 * 
 * @param huart 
 * @param pData 
 * @param Size 
 * @return  
 */
static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
{

    /*��⵱ǰhuart״̬*/
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        /*����ĵ�ַ��������������Ļ�*/
        if ((pData == NULL) || (Size == 0))
        {
            return HAL_ERROR;
        }

        /*huart�����Ӧ��Rx�����ض���*/
        huart->pRxBuffPtr = pData;
        huart->RxXferSize = Size;
        huart->ErrorCode = HAL_UART_ERROR_NONE;

        /*����huart1�ϵ�RX_DMA*/
        HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

        /*ֻ������ӦDMA�����Rx���ܣ�����ǿ���Tx�Ļ�����USART_CR3_DMAT��*/
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    }
    else
    {
        return HAL_BUSY;
    }

    return HAL_OK;
}

/**
 * @brief �Ӿ�ʹ��DMA-USART
 * 
 * @param huartx 
 */
void Vision_USART_Receive_DMA(UART_HandleTypeDef *huartx)
{
    /*��ձ�־λȻ��ʹ��USART���ж�*/
    __HAL_UART_CLEAR_IDLEFLAG(huartx);
    __HAL_UART_ENABLE(huartx);
    __HAL_UART_ENABLE_IT(huartx, UART_IT_IDLE);
    // assert(Vision_BuffSIZE == (13 + 2));
    USART_Receive_DMA_NO_IT(huartx, Vision_DataBuff, Vision_BuffSIZE);
}


/**************************************************���º���û���õ�****************************************************************************/
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
 * @brief �Ӿ����
 * 
 */
void Check_Vision(void)
{
    //�Ӿ����� ---------------------------
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
 * @brief �Ӿ�pitch�����仯
 * 
 * @param CPData 
 */
void Vision_CP(float CPData)
{
    VisionData.Gravity_Offset.y += CPData;
}

/**
 * @brief ��ȡ�Ӿ�pitch�����仯
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

