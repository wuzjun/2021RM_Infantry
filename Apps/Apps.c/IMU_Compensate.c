/**
 * @file IMU_Compensate.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 陀螺仪温漂补偿
 * @version 0.1
 * @date 2021-04-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "IMU_Compensate.h"
#include "Control_Vision.h"
#include "BSP_ADC.h"
#include "DJI_IMU.h"
#include "mpu6500reg.h"
#include "USER_Filter.h"

int8_t Calibrate_Temperate = 0;
positionpid_t imuTempPid = imuTempPidInit;
#undef imuTempPidInit

void Preserve_temp(float Real_temp);
void IMU_GetData_Compensate(void);
IMU_CompensateFUN_t IMU_CompensateFUN = IMU_CompensateFUNInit;
#undef IMU_CompensateFUNInit

uint8_t first_temperate = 0;

static float get_control_temperate(void)
{
    return imu_Export.temp;
}

/**
 * @brief 温度补偿
 * 
 * @param temp 
 * @return  
 */
static void IMU_temp_Control(float temp)
{
    uint16_t tempPWM;
    if (first_temperate)
    {
        imuTempPid.Position_PID(&imuTempPid, temp, get_control_temperate());
        if (imuTempPid.pwm < 0.0f)
        {
            imuTempPid.pwm = 0.0f;
        }
        if (get_control_temperate() >= temp)
        {
            imu_Export.Temp_ReachFlag = 1;
        }
        tempPWM = (uint16_t)imuTempPid.pwm;
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        if (temp * 0.8f > get_control_temperate())
        {
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, MPU6500_TEMP_PWM_MAX - 1);
        }
        else
        {
            first_temperate = 1;
            imuTempPid.i_out = MPU6500_TEMP_PWM_MAX / 2.0f;
        }
    }
}

/**
 * @brief ADC温度获取
 * 
 * @param Real_temp 
 * @return  
 */
void Preserve_temp(float Real_temp)
{
    Calibrate_Temperate = Real_temp;
    if (Calibrate_Temperate > (int8_t)GYRO_CONST_MAX_TEMP)
    {
        Calibrate_Temperate = (int8_t)GYRO_CONST_MAX_TEMP;
    }
}

static void HAL_Update_IMU_Data(void)
{
    DJI_IMUFUN.HAL_mpu_get_data();
    DJI_IMUFUN.HAL_imu_ahrs_update();
    DJI_IMUFUN.HAL_imu_attitude_update();
    //视觉发送数据更新
    Control_Vision_FUN.Vision_SendBufFunction(imu_Export.Angle, imu_Export.SpeedLPF);
}

/**
 * @brief 温补加上获取数据
 * 
 * @return  
 */
void IMU_GetData_Compensate(void)
{
    IMU_temp_Control(Calibrate_Temperate);

    HAL_Update_IMU_Data();
}
