/**
 * @file DJI_IMU.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-04-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _DJI_IMU_H
#define _DJI_IMU_H

#include "spi.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define Calibrate 1 //是否开启陀螺仪校准
#define Calibrate_On 0
#define Calibrate_Off 1

#define IMUOffsetNum 6

#pragma anon_unions

#define MPU_DELAY(x) HAL_Delay(x)
#define BOARD_DOWN (1)

#define INS_DELTA_TICK 1 //任务调用的间隔

#define DMA_RX_NUM 23
#define MPU6500_RX_BUF_DATA_OFFSET 0
#define IST8310_RX_BUF_DATA_OFFSET 15

#define MPU_DATA_READY_BIT 0 //陀螺仪数据准备
#define MPU_MOT_BIT 1        //mpu6500 运动检测

#define DJI_IMUFUNGroundInit          \
    {                                 \
        &DJI_IMU_Init,                \
            &HAL_mpu_get_data,        \
            &HAL_imu_ahrs_update,     \
            &HAL_imu_attitude_update, \
            &IMU_Reset,               \
            &Check_DJIIMU,            \
    }

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t mx;
    int16_t my;
    int16_t mz;

    int16_t temp;

    union
    {
        struct
        {
            int16_t gz;
            int16_t gy;
            int16_t gx;
        };
        int16_t Original_gX[3];
    } Original_Gyro; //原始速度

    struct
    {
        int16_t ax_offset;
        int16_t ay_offset;
        int16_t az_offset;

        int16_t gx_offset;
        int16_t gy_offset;
        int16_t gz_offset;
    } Offset; //零漂值

    HAL_StatusTypeDef InfoUpdateFlag;
} mpu_data_t;

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t mx;
    int16_t my;
    int16_t mz;

    float temp;

    union
    {
        struct
        {
            float wz;
            float wy;
            float wx; /*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
        };
        float Real_wX[3];
    } Real_Gyro;

    union
    {
        struct
        {
            float gz;
            float gy;
            float gx;
        };
        float Original_gX[3];
    } Gyro;

    float vx;
    float vy;
    float vz;

    union
    {
        struct
        {
            float yaw;
            float pit;
            float rol;
        };
        float Real_Angle[3];
    } Axis;

    union
    {
        struct
        {
            float lastyaw;
            float lastpit;
            float lastrol;
        };
        float Last_Angle[3];
    } Last_Axis;

    int16_t turnCount[3];

} imu_t;

/************A板陀螺仪对外数据接口**********/
typedef struct
{
    float Angle[3];
    float SpeedLPF[3];
    float total[3];
    float temp;
    uint8_t Temp_ReachFlag;     //温度是否到达标志位
    uint8_t IMU_Init_Condition; //初始化是否完成
    int InfoUpdateFlag;         //是否正常更新
    uint8_t OffLineFlag;
} imu_Export_t;
/*****************************************/

typedef struct
{
    void (*DJI_IMU_Init)(void);
    void (*HAL_mpu_get_data)(void);
    void (*HAL_imu_ahrs_update)(void);
    void (*HAL_imu_attitude_update)(void);
    void (*IMU_Reset)(imu_Export_t *imu_Export);
    void (*Check_DJIIMU)(void);
} DJI_IMUFUN_t;

extern DJI_IMUFUN_t DJI_IMUFUN;
extern imu_Export_t imu_Export;
extern uint32_t IMU_Offset[6];
extern uint32_t IMUwriteFlashData[6];

#endif /*_DJI_IMU_H*/
