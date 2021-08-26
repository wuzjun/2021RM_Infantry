/**
 * @file IMU_Compensate.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 陀螺仪温漂补偿
 * @version 0.1
 * @date 2021-04-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _IMU_COMPENSATE_H_
#define _IMU_COMPENSATE_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "PID.h"
#include "tim.h"

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为?MPU6500_TEMP_PWM_MAX?-?1
#define GYRO_CONST_MAX_TEMP 40.0f

#define imuTempPidInit     \
    {                      \
        0,                 \
            0,             \
            0,             \
            0,             \
            0,             \
            1600.0f,       \
            0.2f,          \
            0.0f,          \
            0,             \
            0,             \
            0,             \
            0,             \
            4500.0f,       \
            0,             \
            4400.0f,       \
            &Position_PID, \
    }

#define IMU_CompensateFUNInit        \
    {                                \
        &Preserve_temp,              \
            &IMU_GetData_Compensate, \
    }

typedef struct
{
    void (*Preserve_temp)(float Real_temp);
    void (*IMU_GetData_Compensate)(void);
} IMU_CompensateFUN_t;

extern IMU_CompensateFUN_t IMU_CompensateFUN;

#endif /*_IMU_COMPENSATE_H_*/
