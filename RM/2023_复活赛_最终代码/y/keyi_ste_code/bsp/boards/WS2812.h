#ifndef  WS2812_H
#define WS2812_H

#include "stdint.h"
#include "tim.h"

//#define WS2812_new

// WS2812灯的数量
#define WS2812_NUM 8

// PWM频率为800KHz
#define PWM_FREQUENCY 800000

// DMA传输的数据长度
#define DMA_BUFFER_SIZE (WS2812_NUM * 24)

// 颜色掩码
#define COLOR_MASK 0x7

extern uint8_t ws2812_data[];

void WS2812_Update(void);

#define WS2812_old

void ws2812_set_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t num);

void ws2812_example(void);
void ws2812_init(uint8_t led_nums);
extern void ws2812_blue(uint8_t led_nums);
extern void ws2812_red(uint8_t led_nums);
extern void ws2812_green(uint8_t led_nums);

void ws2812_Cap(void);

void ws2812_contril(void);

//摩擦轮
void ws2812_shot(void);

//弹仓
void ws2812_Mag(void);
void ws2812_Cloud(void);
void ws2812_Cap_call(void);
void ws2812_chassis(void);
void ws2812_vision(void);
void ws2812_vision_01(void);
#endif
