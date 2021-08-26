/**
 * @file ANO.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-04-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _ANO_H
#define _ANO_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "usart.h"

#define ANO_FUNGroundInit          \
	{                              \
		&ANO_Send_FloatData,       \
			&ANO_Send_Data_V4,     \
			&ANO_Send_FloatData_2, \
	}

typedef struct
{
	void (*ANO_Send_FloatData)(int32_t Target1, int32_t Target2, int32_t Target3, int32_t Target4);
	void (*ANO_Send_Data_V4)(float Temp_Target1, float Temp_Target2, float Temp_Target3, float Temp_Target4,
							 float Target1, float Target2, float Target3, float Target4);
	void (*ANO_Send_FloatData_2)(int32_t Target1, int32_t Target2);
} ANO_FUN_t;

extern ANO_FUN_t ANO_FUN;
#endif
