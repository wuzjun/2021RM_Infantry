/**
 * @file Monitor_RM_CAN.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Monitor_RM_CAN.h"

void CAN_0x1FF_SendData(CAN_HandleTypeDef *CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void CAN_0x200_SendData(CAN_HandleTypeDef *CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void CAN_0x2FF_SendData(CAN_HandleTypeDef *CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void CAN_0x601_SendData(CAN_HandleTypeDef *CAN_Num, uint8_t data[8]);
Monitor_CAN_FUN_t Monitor_CAN_FUN = Monitor_CAN_FUNGroundInit;
#undef Monitor_CAN_FUNGroundInit

/*************************************
* Method:    CAN_0x1FF_SendData
* Returns:   void
* Parameter: int16_t iq1，参数值。
* 说明：0x1FF报文的统一发送函数—— 控制3508的ID：5-8电机，6020的1-4，2006的5-8
************************************/
void CAN_0x1FF_SendData(CAN_HandleTypeDef *CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{

	uint8_t data[8];

	data[0] = iq1 >> 8;
	data[1] = iq1;
	data[2] = iq2 >> 8;
	data[3] = iq2;
	data[4] = iq3 >> 8;
	data[5] = iq3;
	data[6] = iq4 >> 8;
	data[7] = iq4;

	Can_Fun.CAN_SendData(CAN_SendHandle, CAN_Num, CAN_ID_STD, 0x1FF, data);
}

/*************************************
* Method:    CAN_0x200_SendData
* Returns:   void
* Parameter: int16_t iq1，参数值。
* 说明：0x200报文的统一发送函数—— 控制3508的ID：1-4电机，2006的1-4
************************************/
void CAN_0x200_SendData(CAN_HandleTypeDef *CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{

	uint8_t data[8];

	//数据格式详见说明书P32
	data[0] = iq1 >> 8;
	data[1] = iq1;
	data[2] = iq2 >> 8;
	data[3] = iq2;
	data[4] = iq3 >> 8;
	data[5] = iq3;
	data[6] = iq4 >> 8;
	data[7] = iq4;

	Can_Fun.CAN_SendData(CAN_SendHandle, CAN_Num, CAN_ID_STD, 0x200, data);
}

/*************************************
* Method:    CAN_0x2FF_SendData
* Returns:   void
* Parameter: int16_t iq1，参数值。
* 说明：0x200报文的统一发送函数—— 控制6020的ID：5-7电机
************************************/
void CAN_0x2FF_SendData(CAN_HandleTypeDef *CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{

	uint8_t data[8];

	//数据格式详见说明书P32
	data[0] = iq1 >> 8;
	data[1] = iq1;
	data[2] = iq2 >> 8;
	data[3] = iq2;
	data[4] = iq3 >> 8;
	data[5] = iq3;
	data[6] = iq4 >> 8;
	data[7] = iq4;

	Can_Fun.CAN_SendData(CAN_SendHandle, CAN_Num, CAN_ID_STD, 0x2FF, data);
}

/**
 * @brief 超级电容发送函数
 * 
 * @param CAN_Num 
 * @param iq1 
 * @return  
 */
void CAN_0x601_SendData(CAN_HandleTypeDef *CAN_Num, uint8_t data[8])
{
	Can_Fun.CAN_SendData(CAN_SendHandle, CAN_Num, CAN_ID_STD, 0x601, data);
}
