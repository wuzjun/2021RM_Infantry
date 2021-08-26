/**
 * @file M6020_Motor.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "M6020_Motor.h"

//直接声明对应的电机的结构体而不用数组，直观便于后期调试观察数据使用。
M6020s_t M6020s_Yaw;                                    //ID为1
M6020s_t M6020s_Pitch;                                  //2
M6020s_t *M6020_Array[] = {&M6020s_Yaw, &M6020s_Pitch}; //对应电机的ID必须为：索引+1
#define M6020_Amount 2
void M6020_getInfo(Can_Export_Data_t RxMessage);
void M6020_setTargetAngle(M6020s_t *M6020, int32_t angle);
void M6020_Reset(M6020s_t *m6020);
void Check_M6020(void);

M6020_Fun_t M6020_Fun = M6020_FunGroundInit;
#undef M6020_FunGroundInit

/**
  * @brief  设置M6020电机电压（id号为1~4）
  * @param  iqx (x:1~4) 对应id号电机的电流值，范围 -30000~0~30000
  * @retval None
  */
// void M6020_setCurrent(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
// {

// 	uint8_t data[8];

// 	data[0] = iq1 >> 8;
// 	data[1] = iq1;
// 	data[2] = iq2 >> 8;
// 	data[3] = iq2;
// 	data[4] = iq3 >> 8;
// 	data[5] = iq3;
// 	data[6] = iq4 >> 8;
// 	data[7] = iq4;

// 	// CAN_SendData(&hcan1, CAN_ID_STD, M6020_SENDID, data);
// }

/**
  * @brief  从CAN报文中获取M6020电机信息
  * @param  RxMessage 	CAN报文接收结构体
  * @retval None
  */

void M6020_getInfo(Can_Export_Data_t RxMessage)
{

    int32_t StdId;
    StdId = (int32_t)RxMessage.CAN_RxHeader.StdId - M6020_READID_START; //由0开始
    // if (IndexOutofBounds(StdId, M6020_Amount))
    // {
    // 	Device_setAlertType(Alert_Times_SoftWare);
    // 	return;
    // }

    //解包数据，数据格式详见C620电调说明书P33
    M6020_Array[StdId]->lastAngle = M6020_Array[StdId]->realAngle;
    M6020_Array[StdId]->realAngle = (uint16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
    M6020_Array[StdId]->realSpeed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
    M6020_Array[StdId]->realCurrent = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
    M6020_Array[StdId]->temperture = RxMessage.CANx_Export_RxMessage[6];

    if (M6020_Array[StdId]->realAngle - M6020_Array[StdId]->lastAngle < -6500)
    {
        M6020_Array[StdId]->turnCount++;
    }

    if (M6020_Array[StdId]->lastAngle - M6020_Array[StdId]->realAngle < -6500)
    {
        M6020_Array[StdId]->turnCount--;
    }

    M6020_Array[StdId]->totalAngle = M6020_Array[StdId]->realAngle + (8192 * M6020_Array[StdId]->turnCount);

    //帧率统计，数据更新标志位
    M6020_Array[StdId]->InfoUpdateFrame++;
    M6020_Array[StdId]->InfoUpdateFlag = 1;
}

/*
*@brief  设定M6020电机的目标角度
* @param  motorName 	电机名字 @ref M6623Name_e
*			angle		电流值，范围 0~8191 由于设置0和8191会导致电机振荡，要做个限幅
* @retval None
* */
void M6020_setTargetAngle(M6020s_t *M6020, int32_t angle)
{
    M6020->targetAngle = angle;
}

/*************************************
* Method:    M6020_OverflowReset
* Returns:   void
* 说明：调运此函数以解决totalAngle 等溢出的问题。
************************************/
void M6020_Reset(M6020s_t *m6020)
{
    //解包数据，数据格式详见C620电调说明书P33
    m6020->lastAngle = m6020->realAngle;
    m6020->totalAngle = m6020->realAngle;
    m6020->turnCount = 0;
}

/**
 * @brief M6020检测
 * 
 */
void Check_M6020(void)
{
    for (uint8_t i = 0; i < 2; i++)
    {
        if (M6020_Array[i]->InfoUpdateFrame < 1)
        {
            M6020_Array[i]->OffLineFlag = 1;
        }
        else
        {
            M6020_Array[i]->OffLineFlag = 0;
        }
        M6020_Array[i]->InfoUpdateFrame = 0;
    }
}
