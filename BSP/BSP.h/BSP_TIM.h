/**
 * @file BSP_TIM.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <Math.h>
#include <string.h> //机器人的默认配置文件。
#include "tim.h"

#define TIM_SetComparexGroundInit \
	{                             \
		&TIM_SetCompare1,         \
			&TIM_SetCompare2,     \
			&TIM_SetCompare3,     \
	}

typedef struct
{
	void (*TIM_SetCompare1)(TIM_TypeDef *TIMx, uint32_t Compare1);
	void (*TIM_SetCompare2)(TIM_TypeDef *TIMx, uint32_t Compare2);
	void (*TIM_SetCompare3)(TIM_TypeDef *TIMx, uint32_t Compare3);
} TIM_SetComparex_t;

extern TIM_SetComparex_t TIM_SetComparex;

//typedef struct
//{
//	uint8_t STA;          //输入捕获状态		  
//	uint16_t Date1;  //输入捕获值
//	uint16_t Date2;  //输入捕获值

//	long long temp;
//}TIMxCHx_CAPTURE_t; //定时器捕获缓存值

//extern TIMxCHx_CAPTURE_t TIM4CH1_CAPTURE;
//extern TIMxCHx_CAPTURE_t TIM4CH2_CAPTURE;
//extern TIMxCHx_CAPTURE_t TIM4CH3_CAPTURE;
//extern TIMxCHx_CAPTURE_t TIM4CH4_CAPTURE;


#endif
