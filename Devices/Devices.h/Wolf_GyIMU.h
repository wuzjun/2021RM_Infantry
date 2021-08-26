/**
 * @file Wolf_GyIMU.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __GY_IMU_H
#define __GY_IMU_H

#include "typedef.h"
#include <AddMath.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define GY_IMU_PACKAGE_LENGTH 18
#define GY_IMU_READID_START 0x413 //can1的云台陀螺仪
#define GY_IMU_READID_END 0x414   //can2的底盘陀螺仪

#define Wolf_GyIMU_FunGroundInit \
    {                            \
        &GY6050_Init,            \
            &GY6050_getInfo,     \
            &IMU_processing,     \
            &Check_GyIMU,        \
            &GY6050_PitErr,      \
    }

typedef struct
{
    union
    {
        struct
        {
            uint16_t yaw;
            int16_t gyro_z;
            int16_t pitch;
            int16_t gyro_x;
        };
        uint8_t dataBuff[8];
    } bno055_data;

    struct
    {
        float x; //浮点数pitch轴的方向向量
        float y; //浮点数Y轴方向向量
        float z; //浮点数yaw轴的转动速度
    } Gyro;
    struct
    {
        float Roll;  //ROLL轴方向，当前的角度值
        float Pitch; //PITCH轴方向
        float Yaw;   //YAW轴方向
    } Eular;

    struct
    {
        float lastPitch; //上一次YAW轴数据
        float totalPitch;
        float errorPitch;   //目标角度的误差值
        float Target_pitch; //开机定义值
        int16_t turnCount;
    } Calculating_Data;

    uint8_t DevStatus;
    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;
    uint32_t IMU_FPS;
} GY_IMU_t;

typedef struct
{
    void (*GY6050_Init)(GY_IMUExport_t *IMUExport_Cloud, GY_IMUExport_t *IMUExport_Chassis);
    void (*GY6050_getInfo)(Can_Export_Data_t RxMessage);
    void (*IMU_processing)(GY_IMUExport_t *IMUExport_Cloud, GY_IMUExport_t *IMUExport_Chassis);
    void (*Check_GyIMU)(void);
    float (*GY6050_PitErr)(void);
} Wolf_GyIMU_Fun_t;

extern Wolf_GyIMU_Fun_t Wolf_GyIMU_Fun;

#endif
