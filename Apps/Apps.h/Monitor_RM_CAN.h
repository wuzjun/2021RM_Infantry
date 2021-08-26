/**
 * @file Monitor_RM_CAN.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __Monitor_RM_CAN
#define __Monitor_RM_CAN

#include "can.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "BSP_CAN.h"
#include "Handle.h"

#define Monitor_CAN_FUNGroundInit \
	{                             \
		&CAN_0x1FF_SendData,      \
			&CAN_0x200_SendData,  \
			&CAN_0x2FF_SendData,  \
			&CAN_0x601_SendData,  \
	}

typedef struct
{
	void (*CAN_0x1FF_SendData)(CAN_HandleTypeDef *CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
	void (*CAN_0x200_SendData)(CAN_HandleTypeDef *CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
	void (*CAN_0x2FF_SendData)(CAN_HandleTypeDef *CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
	void (*CAN_0x601_SendData)(CAN_HandleTypeDef *CAN_Num, uint8_t data[8]);

} Monitor_CAN_FUN_t;

extern Monitor_CAN_FUN_t Monitor_CAN_FUN;

#endif
