/**
 * @file Task_Sampling.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Task_Sampling.h"
#include "BSP_ADC.h"
#include "Cloud_control.h"
#include "Control_Vision.h"
#include "DJI_C_IMU.h"
#include "DJI_IMU.h"
#include "IMU_Compensate.h"
#include "Wolf_GyIMU.h"
#include "Status_Update.h"

WorldTime_RxTypedef Sampling_FPS;
uint32_t GetData_FPS;
uint8_t IMU_time;
void Fixed_Sampling(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    const TickType_t TimeIncrement = pdMS_TO_TICKS(1); //每一毫秒强制进入总控制

    for (;;)
    {
        if (imu_Export.IMU_Init_Condition == 1)
        {
            IMU_CompensateFUN.IMU_GetData_Compensate();
        }

        if (IMU_time >= 14)
        {
            Wolf_GyIMU_Fun.IMU_processing(&IMUExport_Cloud, &IMUExport_Chassis);
            IMU_time = 0;
        }

        IMU_time++;
        Control_Vision_FUN.Vision_processing();
        DJI_C_IMUFUN.Updata_Hand_Euler_Gyro_Data();

        Get_FPS(&Sampling_FPS, &GetData_FPS);
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
