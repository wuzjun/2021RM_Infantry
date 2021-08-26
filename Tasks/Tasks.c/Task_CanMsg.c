/**
 * @file Task_CanMsg.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Task_CanMsg.h"
#include "BSP_CAN.h"
#include "DJI_C_IMU.h"
#include "M2006_Motor.h"
#include "M3508_Motor.h"
#include "M6020_Motor.h"
#include "SuperCapacitor.h"
#include "Wolf_GyIMU.h"

/**
  * @Data   2021-03-28	
  * @brief  can1接收任务
  * @param  void
  * @retval void
  */
void Can1Receives(void const *argument)
{
    Can_Export_Data_t Can_Export_Data;
    uint32_t ID;
    for (;;)
    {
        xQueueReceive(CAN1_ReceiveHandle, &Can_Export_Data, portMAX_DELAY);
        ID = Can_Export_Data.CAN_RxHeader.StdId;
        if (ID == M6020_READID_START)
        {
            M6020_Fun.M6020_getInfo(Can_Export_Data);
        }
        else if (ID == GY_IMU_READID_END) //彬哥版陀螺仪(底盘）
        {
            Wolf_GyIMU_Fun.GY6050_getInfo(Can_Export_Data);
        }
        else if (ID == SCCM_RECEIVE_ID) // 超级电容
        {
            SuperCapacitorFUN.SCCM_MsgProcess(Can_Export_Data);
        }
        else if (ID >= M3508_READID_START || ID <= M3508_READID_END)
        {
            M3508_FUN.M3508_getInfo(Can_Export_Data);
        }
        Monitor_CAN1.InfoUpdateFrame++;
    }
}

/**
  * @Data   2021-03-28
  * @brief  can2接收任务
  * @param  void
  * @retval void
  */
void Can2Receives(void const *argument)
{
    uint32_t ID;
    Can_Export_Data_t Can_Export_Data;
    for (;;)
    {
        xQueueReceive(CAN2_ReceiveHandle, &Can_Export_Data, portMAX_DELAY);
        ID = Can_Export_Data.CAN_RxHeader.StdId;
        if (ID == GY_IMU_READID_START) //彬哥云台陀螺仪
        {
            Wolf_GyIMU_Fun.GY6050_getInfo(Can_Export_Data);
        }
        else if (ID == M6020_READID_END) //云台pitch电机
        {
            M6020_Fun.M6020_getInfo(Can_Export_Data);
        }
        else if (ID == M2006_READID_START) //拨盘电机
        {
            M2006_FUN.M2006_getInfo(Can_Export_Data);
        }
        else if (ID == M30508_FricL_ID || ID == M30508_FricR_ID) // 二代步兵 - 3508摩擦轮
        {
            M3508_FUN.M3508_Friction_getInfo(Can_Export_Data);
        }
        else if (ID == DJI_C_Angle)
        {
            DJI_C_IMUFUN.DJI_C_Euler_getInfo(Can_Export_Data);
        }
        else if (ID == DJI_C_Gyro)
        {
            DJI_C_IMUFUN.DJI_C_Gyro_getInfo(Can_Export_Data);
        }
        Monitor_CAN2.InfoUpdateFrame++;
    }
}
