#include "WS2812.h"
#include "tim.h"
#include "Robot_control.h"		
#include "shoot.h"
#include "Cloud_control.h"
#include "Control_Vision.h"
#ifdef WS2812_new



// WS2812灯的颜色数据
uint8_t ws2812_data[WS2812_NUM * 3];

// DMA传输的缓存
uint16_t dma_buffer[DMA_BUFFER_SIZE];

// 更新WS2812灯的颜色
void WS2812_Update(void)
{
    uint32_t i, j;
    uint8_t data;

    // 将颜色数据转换为DMA传输的数据
    for (i = 0; i < WS2812_NUM * 3; i++)
    {
        data = ws2812_data[i];
        for (j = 0; j < 8; j++)
        {
            if (data & 0x80)
            {
                dma_buffer[i * 8 + j] = 28; // 1的高电平持续时间为28个时钟周期
            }
            else
            {
                dma_buffer[i * 8 + j] = 14; // 0的高电平持续时间为14个时钟周期
            }
            data <<= 1;
        }
    }

    // 启动DMA传输
    DMA_HandleTypeDef hdma_tim8_up;
    hdma_tim8_up.Instance = DMA1_Stream1;
    HAL_DMA_Start_IT(&hdma_tim8_up, (uint32_t)dma_buffer, (uint32_t)&TIM8->CCR2, DMA_BUFFER_SIZE);
}


#endif

#ifdef WS2812_old

#define ONE_PULSE        (149)                           //1 码
#define ZERO_PULSE       (74)                           //0 码
#define RESET_PULSE      (80)                           //80 ，复位信号
#define LED_NUMS         (11)                            //led 数量
#define LED_DATA_LEN     (24)                           //led 长度，单个需要24bits
#define WS2812_DATA_LEN  (LED_NUMS*LED_DATA_LEN)        //ws2812灯条需要的数组长度

uint16_t static RGB_buffur[RESET_PULSE + WS2812_DATA_LEN] = { 0 };

void ws2812_set_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t num)
{
    //
    uint16_t* p = (RGB_buffur + RESET_PULSE) + (num * LED_DATA_LEN);
    
    for (uint16_t i = 0;i < 8;i++)
    {
        //
        p[i]      = (G << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
        p[i + 8]  = (R << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
        p[i + 16] = (B << i) & (0x80)?ONE_PULSE:ZERO_PULSE;

    }
}
/*ws2812 初始化*/
void ws2812_init(uint8_t led_nums)
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;
	for(uint8_t i = 0; i < led_nums; i++)
	{
		ws2812_set_RGB(0x00, 0x00, 0x00, i);
	}
	 HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data)); 
}
/*全亮蓝灯*/
void ws2812_blue(uint8_t led_nums)
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;
	for(uint8_t i = 0; i < led_nums; i++)
	{
		ws2812_set_RGB(0x00, 0x00, 0x22, i);
	}
	 HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}
/*全亮红灯*/
void ws2812_red(uint8_t led_nums)
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;
	for(uint8_t i = 0; i < led_nums; i++)
	{
		ws2812_set_RGB(0x22, 0x00, 0x00, i);
	}
	 HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}
/*全亮绿灯*/
void ws2812_green(uint8_t led_nums)
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;
	for(uint8_t i = 0; i < led_nums; i++)
	{
		ws2812_set_RGB(0x00, 0x22, 0x00, i);
	}
	 HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}

void ws2812_example(void)
{		
	  ws2812_set_RGB(0x00, 0x00, 0x00, 0);
    ws2812_set_RGB(0x00, 0x00, 0x00, 1);
    ws2812_set_RGB(0x00, 0x00, 0x00, 2);
    ws2812_set_RGB(0x00, 0x00, 0x00, 3);
    ws2812_set_RGB(0x00, 0x00, 0x00, 4);
    ws2812_set_RGB(0x00, 0x00, 0x00, 5);
		ws2812_set_RGB(0x00, 0x00, 0x00, 6);
    ws2812_set_RGB(0x00, 0x00, 0x00, 7);
    HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(344)); //344 = 80 + 24 * LED_NUMS(11)
}


void ws2812_shoot(void)
{
//	if()
//	{
//		
//	}
}

//红绿蓝

//云台
void ws2812_Cloud(void)
{
	if(Gimbal_Mode == Gimbal_PCMode)
	{
		ws2812_set_RGB(0x00, 0x00, 0x22, 6);
	}
	else if(Gimbal_Mode == Gimbal_NormalMode)
	{
		ws2812_set_RGB(0x00, 0x22, 0x00, 6);
	}
	else if(Gimbal_Mode == Gimbal_DisableMode)
	{
		ws2812_set_RGB(0x22, 0x00, 0x00, 6);
	}
	HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(344)); //344 = 80 + 24 * LED_NUMS(11)	
}


//超电开关控制
void ws2812_Cap(void)
{
	if(Cap_switch==1)
	{
		ws2812_set_RGB(0x00, 0x22, 0x00, 4);
	}
	else
	{
		ws2812_set_RGB(0x22, 0x00, 0x00, 4);
	}
	HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(344)); //344 = 80 + 24 * LED_NUMS(11)
}

//超电电量
void ws2812_Cap_call(void)
{	
	if(cap_call > 5 && cap_call < 30)//红
	{
		ws2812_set_RGB(0x22, 0x00, 0x00, 5);
	}
	else if(cap_call > 30 && cap_call < 90)//蓝
	{
		ws2812_set_RGB(0x00, 0x00, 0x22, 5);
	}
	else if(cap_call > 90 && cap_call < 100)//绿
	{
		ws2812_set_RGB(0x00, 0x22, 0x00, 5);
	}
	else if(cap_call < 5)//不量
	{
		ws2812_set_RGB(0x00, 0x00, 0x00, 5);
	}
	HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(344)); //344 = 80 + 24 * LED_NUMS(11)
}



//弹仓
void ws2812_Mag(void)
{
    if(Mag_Switch==0)
		{
			ws2812_set_RGB(0x00, 0x22, 0x00, 2);
		}
		else
		{
		  ws2812_set_RGB(0x22, 0x00, 0x00, 2);
		}
		HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(344)); //344 = 80 + 24 * LED_NUMS(11)
}

void ws2812_chassis(void)
{
	if(Chassis_Mode == CHAS_followMode)//绿灯
	{
		ws2812_set_RGB(0x00, 0x22, 0x00, 1);
	}	
	else if(Chassis_Mode == CHAS_SpinMode)//蓝灯
	{
	  ws2812_set_RGB(0x00, 0x00, 0x22, 1);
	}
	else if(Chassis_Mode == CHAS_LockMode)//红灯
	{
	  ws2812_set_RGB(0x22, 0x00, 0x00, 1);
	}
	else if(Chassis_Mode == CHAS_DisableMode)//不亮
	{
		ws2812_set_RGB(0x00, 0x00, 0x00, 1);
	}
		HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(344)); //344 = 80 + 24 * LED_NUMS(11)
}
//摩擦轮xz
void ws2812_shot(void)
{
	
    if(Attack_Mode == Attack_Disable)//失能 红
		{
			ws2812_set_RGB(0x22, 0x00, 0x00, 2);
		}
		else
		{
			ws2812_set_RGB(0x00, 0x22, 0x00, 2);
		}
		HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(344)); //344 = 80 + 24 * LED_NUMS(11)
	
}

void ws2812_vision(void)
{
    if(Vision_Mode == Vision_Forcast)
		{
			ws2812_set_RGB(0x00, 0x22, 0x00, 3);
		}
		else if(Vision_Mode == Vision_BIGWindmill)
		{
			ws2812_set_RGB(0x00, 0x00, 0x22, 3);
		}
		else if(Vision_Mode == Vision_Top)
		{
			ws2812_set_RGB(0x22, 0x00, 0x00, 3);
		}
		else if(Vision_Mode == Vision_Outpost)
		{
			ws2812_set_RGB(0x00, 0x00, 0x00, 3);
		}
		HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(344)); //344 = 80 + 24 * LED_NUMS(11)
}
void ws2812_vision_01(void)
{
    if(VisionData.RawData.mode == 0)
		{
			ws2812_set_RGB(0x22, 0x00, 0x00, 4);
		}
		else if(VisionData.RawData.mode == 1)
		{
			ws2812_set_RGB(0x00, 0x22, 0x00, 4);
		}
		HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(344)); //344 = 80 + 24 * LED_NUMS(11)
}

void ws2812_contril(void)
{
//    ws2812_Cap();//电容
	  ws2812_Cap_call();
	  ws2812_shot();//摩擦轮
//  	ws2812_Mag();//弹仓
//	  ws2812_Cloud();//云台
	  ws2812_chassis();//底盘
	
	ws2812_vision();
	  ws2812_vision_01();
}

#endif

