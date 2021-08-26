/**
 * @file BSP_CAN.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "BSP_CAN.h"

/*******************************用户数据定义************************************/
void CAN_IT_Init(CAN_HandleTypeDef *hcanx, uint8_t Can_type);
void CAN_RxMessage_Export_Date(CAN_HandleTypeDef *hcanx, osMessageQId CANx_Handle, uint8_t Can_type);
void CAN_SendData(osMessageQId CANx_Handle, CAN_HandleTypeDef *CANx, uint8_t id_type, uint32_t id, uint8_t data[8]);
void Check_CAN(void);
Can_Fun_t Can_Fun = Can_FunGroundInit;
#undef Can_FunGroundInit

/***********************************接口赋值************************************/
Can_Data_t Can_Data[2] = Can_DataGroundInit;
#undef Can_DataGroundInit
CAN_Devices_t *Monitor_CAN[] = {&Monitor_CAN1, &Monitor_CAN2};
CAN_Devices_t Monitor_CAN1, Monitor_CAN2;
/*******************************************************************************/

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

/**
  * @Data   2021-03-27
  * @brief  canx中断接收
  * @param  CAN_HandleTypeDef *hcanx, osMessageQId CANx_Handle, uint8_t Can_type
  * @retval void
  */
void CAN_RxMessage_Export_Date(CAN_HandleTypeDef *hcanx, osMessageQId CANx_Handle, uint8_t Can_type)
{
    Can_Export_Data_t Can_Export_Data[2];
    uint8_t Canx_type = Can_type - 1;
    HAL_CAN_GetRxMessage(hcanx, CAN_RX_FIFO0,
                         &Can_Data[Canx_type].CAN_RxTypedef.CANx_RxHeader,
                         Can_Data[Canx_type].CAN_RxTypedef.CAN_RxMessage);

    Can_Export_Data[Canx_type].CAN_RxHeader = Can_Data[Canx_type].CAN_RxTypedef.CANx_RxHeader;
    memcpy(&Can_Export_Data[Canx_type].CANx_Export_RxMessage,
           Can_Data[Canx_type].CAN_RxTypedef.CAN_RxMessage,
           sizeof(uint8_t[8]));

    xQueueSendToBackFromISR(CANx_Handle, &Can_Export_Data[Canx_type], 0); //把接收数据发给接收队列
}

/**
  * @brief  CAN发送数据
  * @param  CANx 		CAN编号
  * 		id_type ·	id类型 CAN_ID_STD， CAN_ID_EXT
  *			id			id号
  * 		data[8]		8个数据
  * @retval None
  */
void CAN_SendData(osMessageQId CANx_Handle, CAN_HandleTypeDef *CANx, uint8_t id_type, uint32_t id, uint8_t data[8])
{
    Can_Send_Data_t Can_Send_Data;

    Can_Send_Data.Canx = CANx;
    if (id_type == CAN_ID_STD)
    {
        Can_Send_Data.CAN_TxHeader.StdId = id;
    }
    else
    {
        Can_Send_Data.CAN_TxHeader.ExtId = id; //ID号
    }

    Can_Send_Data.CAN_TxHeader.IDE = id_type;      //ID类型
    Can_Send_Data.CAN_TxHeader.RTR = CAN_RTR_DATA; //发送的为数据
    Can_Send_Data.CAN_TxHeader.DLC = 0x08;         //数据长度为8字节
    Can_Send_Data.CAN_TxHeader.TransmitGlobalTime = DISABLE;

    memcpy(Can_Send_Data.CANx_Send_RxMessage,
           data,
           sizeof(uint8_t[8]));

    xQueueSend(CANx_Handle, &Can_Send_Data, 0);
}

void Check_CAN(void)
{
    for (uint8_t i = 0; i < 2; i++)
    {
        if (Monitor_CAN[i]->InfoUpdateFrame < 1)
        {
            Monitor_CAN[i]->OffLineFlag = 1;
        }
        else
        {
            Monitor_CAN[i]->OffLineFlag = 0;
        }
        Monitor_CAN[i]->InfoUpdateFrame = 0;
    }
}
