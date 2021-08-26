/**
 * @file Cloud_control.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __CLOUD_CONTROL_H
#define __CLOUD_CONTROL_H

#include "PID.h"
#include "Robot_Config.h"
#include "SpeedRamp.h"
#include "kalman_filter.h"
#include "typedef.h"
#include <AddMath.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define Cloud_Mode_Normal 0
#define Cloud_Mode_WithIMU 1


//重新安装电机或移用代码时需要重新测量这些值（toalAngle）后再允许运动。

#if Infantry_Year == Infantry_2020
#define Cloud_Yaw_Center 4060
#define Cloud_Yaw_ReverseCenter 7809
#elif Infantry_Year == Infantry_2021
/***********************/
#if Who_Infantry == ZJ_Infantry
#define Cloud_Yaw_Center 3030
#define Cloud_Yaw_ReverseCenter 7126
#elif Who_Infantry == ZF_Infantry
#define Cloud_Yaw_Center 4800
#define Cloud_Yaw_ReverseCenter 705

#endif
/***********************/

#endif

#define Center(Min, Max) (((Max) + (Min)) / 2)

#define No_Compensate 0
#define Kalman 1
#define Fourth_Ring 2

#define VisionYaw 0
#define VisionPitch 1

/****************电机PID*******************/
#define M6020s_YawOPIDInit  \
    {                       \
        0,                  \
            0,              \
            0,              \
            0,              \
            0,              \
            0.3f,           \
            0.0f,           \
            18.0f,          \
            0,              \
            0,              \
            0,              \
            0,              \
            100,            \
            0,              \
            0,              \
            &Cloud_YAWOPID, \
    }

#define M6020s_YawIPIDInit   \
    {                        \
        0,                   \
            0,               \
            0,               \
            0,               \
            0,               \
            300.0f,          \
            0.07f,           \
            0.0f,            \
            0,               \
            0,               \
            0,               \
            0,               \
            M6020_MaxOutput, \
            30,              \
            5000,            \
            &Cloud_YAWIPID,  \
    }

#define M6020s_PitchOPIDInit  \
    {                         \
        0,                    \
            0,                \
            0,                \
            0,                \
            0,                \
            0.35f,            \
            0.0f,             \
            4.5f,             \
            0,                \
            0,                \
            0,                \
            0,                \
            100,              \
            0,                \
            3000,             \
            &Cloud_PITCHOPID, \
    }

#define M6020s_PitchIPIDInit  \
    {                         \
        0,                    \
            0,                \
            0,                \
            0,                \
            0,                \
            350.0f,           \
            2.5f,             \
            0.0f,             \
            0,                \
            0,                \
            0,                \
            0,                \
            M6020_MaxOutput,  \
            30,               \
            4000,             \
            &Cloud_PITCHIPID, \
    }
/****************电机PID END*******************/

/***************电机+视觉PID****************/
#define VisionM6020s_YawOPIDInit \
    {                            \
        0,                       \
            0,                   \
            0,                   \
            0,                   \
            0,                   \
            0.3f,                \
            0.0f,                \
            18.0f,               \
            0,                   \
            0,                   \
            0,                   \
            0,                   \
            100,                 \
            0,                   \
            0,                   \
            &Vision_YAWOPID,     \
    }

#define VisionM6020s_YawIPIDInit \
    {                            \
        0,                       \
            0,                   \
            0,                   \
            0,                   \
            0,                   \
            300.0f,              \
            0.07f,               \
            0.0f,                \
            0,                   \
            0,                   \
            0,                   \
            0,                   \
            M6020_MaxOutput,     \
            30,                  \
            5000,                \
            &Vision_YAWIPID,     \
    }

#define VisionM6020s_PitchOPIDInit \
    {                              \
        0,                         \
            0,                     \
            0,                     \
            0,                     \
            0,                     \
            0.35f,                 \
            0.0f,                  \
            4.5f,                  \
            0,                     \
            0,                     \
            0,                     \
            0,                     \
            100,                   \
            0,                     \
            3000,                  \
            &Vision_PITCHOPID,     \
    }

#define VisionM6020s_PitchIPIDInit \
    {                              \
        0,                         \
            0,                     \
            0,                     \
            0,                     \
            0,                     \
            350.0f,                \
            2.5f,                  \
            0.0f,                  \
            0,                     \
            0,                     \
            0,                     \
            0,                     \
            M6020_MaxOutput,       \
            30,                    \
            4000,                  \
            &Vision_PITCHIPID,     \
    }
/***************电机+视觉PID END****************/

/***************陀螺仪PID**************/
#define YawAttitude_PIDInit    \
    {                          \
        0,                     \
            0,                 \
            0,                 \
            0,                 \
            0,                 \
            -8.5f,             \
            0.0f,              \
            0.0f,              \
            0,                 \
            0,                 \
            0,                 \
            0,                 \
            18000,             \
            0,                 \
            0,                 \
            &Cloud_IMUYAWOPID, \
    }

#define YawSpeed_PIDInit       \
    {                          \
        0,                     \
            0,                 \
            0,                 \
            0,                 \
            0,                 \
            50.0f,             \
            0.3f,              \
            0.0f,              \
            0,                 \
            0,                 \
            0,                 \
            0,                 \
            M6020_MaxOutput,   \
            1000,              \
            15000,             \
            &Cloud_IMUYAWIPID, \
    }

#define PitchAttitude_PIDInit    \
    {                            \
        0,                       \
            0,                   \
            0,                   \
            0,                   \
            0,                   \
            18.0f,               \
            0.0f,                \
            0.0f,                \
            0,                   \
            0,                   \
            0,                   \
            0,                   \
            10000,               \
            0,                   \
            3000,                \
            &Cloud_IMUPITCHOPID, \
    }

#define PitchSpeed_PIDInit       \
    {                            \
        0,                       \
            0,                   \
            0,                   \
            0,                   \
            0,                   \
            9.1f,                \
            0.09f,               \
            0.0f,                \
            0,                   \
            0,                   \
            0,                   \
            0,                   \
            M6020_MaxOutput,     \
            4500,                \
            9000,                \
            &Cloud_IMUPITCHIPID, \
    }
/***************陀螺仪PID END**************/

/***************陀螺仪+视觉PID***************/
#define VisionYawAttitude_PIDInit    \
    {                                \
        0,                           \
            0,                       \
            0,                       \
            0,                       \
            0,                       \
            -8.5f,                   \
            0.0f,                    \
            0.0f,                    \
            0,                       \
            0,                       \
            0,                       \
            0,                       \
            18000,                   \
            0,                       \
            0,                       \
            &Cloud_VisionIMUYAWOPID, \
    }

#define VisionYawSpeed_PIDInit       \
    {                                \
        0,                           \
            0,                       \
            0,                       \
            0,                       \
            0,                       \
            60.0f,                   \
            0.5f,                    \
            0.0f,                    \
            0,                       \
            0,                       \
            0,                       \
            0,                       \
            M6020_MaxOutput,         \
            1000,                    \
            15000,                   \
            &Cloud_VisionIMUYAWIPID, \
    }

#define VisionPitchAttitude_PIDInit    \
    {                                  \
        0,                             \
            0,                         \
            0,                         \
            0,                         \
            0,                         \
            12.0f,                     \
            0.0f,                      \
            0.0f,                      \
            0,                         \
            0,                         \
            0,                         \
            0,                         \
            10000,                     \
            0,                         \
            3000,                      \
            &Cloud_VisionIMUPITCHOPID, \
    }

#define VisionPitchSpeed_PIDInit       \
    {                                  \
        0,                             \
            0,                         \
            0,                         \
            0,                         \
            0,                         \
            9.1f,                      \
            0.09f,                     \
            0.0f,                      \
            0,                         \
            0,                         \
            0,                         \
            0,                         \
            M6020_MaxOutput,           \
            4500,                      \
            9000,                      \
            &Cloud_VisionIMUPITCHIPID, \
    }
/***************陀螺仪+视觉PID END***************/

/******************视觉双环PID初始化*****************/
#define Vision_AnglePid_Yaw_PIDInit \
    {                               \
        0,                          \
            0,                      \
            0,                      \
            0,                      \
            0,                      \
            10,                     \
            12,                     \
            13,                     \
            0,                      \
            0,                      \
            0,                      \
            0,                      \
            28888,                  \
            0,                      \
            8000,                   \
            &Position_PID,          \
    }
#define Vision_SpeedPid_Yaw_PIDInit \
    {                               \
        0,                          \
            0,                      \
            0,                      \
            0,                      \
            0,                      \
            0,                      \
            0,                      \
            0,                      \
            0,                      \
            0,                      \
            0,                      \
            0,                      \
            28888,                  \
            0,                      \
            8000,                   \
            &Position_PID,          \
    }
#define Vision_AnglePid_Pitch_PIDInit \
    {                                 \
        0,                            \
            0,                        \
            0,                        \
            0,                        \
            0,                        \
            0,                        \
            0,                        \
            0,                        \
            0,                        \
            0,                        \
            0,                        \
            0,                        \
            28888,                    \
            0,                        \
            8000,                     \
            &Position_PID,            \
    }
#define Vision_SpeedPid_Pitch_PIDInit \
    {                                 \
        0,                            \
            0,                        \
            0,                        \
            0,                        \
            0,                        \
            0,                        \
            0,                        \
            0,                        \
            0,                        \
            0,                        \
            0,                        \
            0,                        \
            28888,                    \
            0,                        \
            8000,                     \
            &Position_PID,            \
    }
/******************视觉双环PID初始化 END*****************/

#define TestSpeed_PIDInit    \
    {                        \
        0,                   \
            0,               \
            0,               \
            0,               \
            0,               \
            50.0f,           \
            0.2f,            \
            0.0f,            \
            0,               \
            0,               \
            0,               \
            0,               \
            M6020_MaxOutput, \
            0,               \
            3000,            \
            &Position_PID,   \
    }

#define Cloud_FUNGroundInit               \
    {                                     \
        &Cloud_Init,                      \
            &Cloud_processing_WithIMU,    \
            &Cloud_processing_WithoutIMU, \
            &Cloud_getYawAngleWithCenter, \
            &IMUData_chance,              \
    }
/*********************卡尔曼数据初始化******************/
#define Kalman_YawDataInit      \
    {                           \
        0,                      \
            0,                  \
            0,                  \
            0,                  \
            {850, 0.1, 10, 50}, \
    }

#define Kalman_PitchDataInit   \
    {                          \
        0,                     \
            0,                 \
            0,                 \
            0,                 \
            {0, 0.03, 5, 250}, \
    }

#define KalmanRampInit           \
    {                            \
        0,                       \
            3,                   \
            -PredictAngle_limit, \
            PredictAngle_limit,  \
    }

/*********************卡尔曼数据初始化END******************/
typedef enum
{
    A_IMU,
    C_IMU
} IMUState_t;

typedef struct
{
    float *IMUAngle_Yaw;
    float *IMUAngle_Pitch;
    float *IMUSpeed_Yaw;
    float *IMUSpeed_Pitch;
    IMUState_t IMUState;
} RealData_t;

/* 云台运动 */
typedef struct
{
    float YawLpfAttFactor;   //yaw轴云台滤波系数
    float PitchLpfAttFactor; //pitch轴滤波系数。

    //电机位置angle运作数据
    float targetYawRaw; //云台目标yaw轴原始数据
    float last_targetYawRaw;
    float targetPitchRaw;   //云台目标pitch轴原始数据
    float targetYawLPF;     //云台yaw轴滤波后数据
    float targetPitchLPF;   //云台pitch轴滤波后数据
    int Reversefollow_Date; //底盘跟随中心角叠加值

    //陀螺仪运作数据
    float IMUtargetYawRaw;      //云台目标yaw轴原始数据
    float IMUtargetPitchRaw;    //云台目标pitch轴原始数据
    float IMUtargetYawLPF;      //云台yaw轴滤波后数据
    float last_IMUtargetYawLPF; //云台yaw轴滤波后数据
    float IMUtargetPitchLPF;    //云台pitch轴滤波后数据

    float IMUYawAngleMax;   //云台IMU最大角度(右)
    float IMUYawAngleMin;   //云台IMU最小角度(左)
    float IMUPitchAngleMax; //云台IMU最大角度(下)
    float IMUPitchAngleMin; //云台IMU最小角度(上)

    RealData_t RealData;

    uint8_t Mode; //云台控制模式
} Cloud_t;

typedef struct
{
    float Speed_Gain; //速度增益
    positionpid_t *Vision_AnglePID;
    positionpid_t *Vision_SpeedPID;
} VisionFourth_Ring_t;

typedef struct //视觉目标速度测量
{
    int delay_cnt; //计算相邻两帧目标不变持续时间,用来判断速度是否为0
    int freq;
    int last_time;         //上次受到目标角度的时间
    float last_position;   //上个目标角度
    float speed;           //速度
    float last_speed;      //上次速度
    float processed_speed; //速度计算结果
} speed_calc_data_t;

typedef struct
{
    void (*Cloud_Init)(void);
    void (*Cloud_processing_WithIMU)(float delta_yaw, float delta_pitch);
    void (*Cloud_processing_WithoutIMU)(float delta_yaw, float delta_pitch);
    float (*Cloud_getYawAngleWithCenter)(void);
    void (*IMUData_chance)(void);
} Cloud_FUN_t;

typedef struct
{
    float Test_Speed;
    float Test_angle;
} Cloud_ParameterDeBug_t;

typedef struct
{
    Kalman_Data_t *Kalman_Data;
    VisionFourth_Ring_t *VisionFourth_Ring;
} All_VisionCompensate_t;

extern Cloud_t Cloud;
extern GY_IMUExport_t IMUExport_Cloud;
extern float Cloud_Chassis_SpeedOffset_current_;
extern Cloud_FUN_t Cloud_FUN;
extern float Kalman_VisionAngle;

extern VisionFourth_Ring_t VisionFourth_Ring_Yaw;
extern VisionFourth_Ring_t VisionFourth_Ring_Pitch;
extern Cloud_ParameterDeBug_t Cloud_ParameterDeBug[2];
extern positionpid_t YawAttitude_PID;   //云台yaw轴姿态pid
extern positionpid_t YawSpeed_PID;      //云台yaw轴速度pid
extern positionpid_t PitchAttitude_PID; //云台roll轴姿态pid
extern positionpid_t PitchSpeed_PID;    //云台roll轴姿态pid
extern positionpid_t M6020s_YawOPID;    //YAW轴云台电机外环
extern positionpid_t M6020s_YawIPID;    //YAW轴云台电机内环
extern positionpid_t M6020s_PitchOPID;  //PITCH轴云台电机外环
extern positionpid_t M6020s_PitchIPID;  //PITCH轴云台电机内环
extern positionpid_t VisionYawAttitude_PID;
extern positionpid_t VisionYawSpeed_PID;
extern All_VisionCompensate_t All_VisionComYaw;
extern extKalman_t Cloud_PitchGyroAngle_Error_Kalman;
extern kalman_filter_t yaw_kalman_filter;
extern kalman_filter_t pitch_kalman_filter;

#endif /* __CLOUD_CONTROL_H */
