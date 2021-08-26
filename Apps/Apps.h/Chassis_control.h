/**
 * @file Chassis_control.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __CHIASSIS_CONTROL_H
#define __CHIASSIS_CONTROL_H

#include "M3508_Motor.h"
#include "PID.h"
#include "Robot_Config.h"
#include "SuperCapacitor.h"
#include "typedef.h"
#include <AddMath.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define Chassis_LpfAttFactor 0.05f //速度平滑处理的系数。(效果：可以让车更快的停下来。0.04)

#define WheelMaxSpeed 9000.0f //单轮最大速度

#define Chassis_MaxSpeed_Y 9000.0f //底盘前后最快速度
#define Chassis_MaxSpeed_X 9000.0f //底盘平移最快速度
#define Chassis_MaxSpeed_W 9000.0f //底盘自旋最快速度

#define Chassis_SpeedNormal 3400.0f
#define Chassis_SpeedHigh_Normal 3900.0f
#define Chassis_SpeedMid 4800.0f
#define Chassis_SpeedFast 5300.0f
#define Chassis_SuperFast 7000.0f

#if Infantry_Year == Infantry_2020
#define FollowYawAttitude_pidInit \
    {                             \
        0,                        \
            0,                    \
            0,                    \
            0,                    \
            0,                    \
            7500.0f,              \
            0.0f,                 \
            0.0f,                 \
            0,                    \
            0,                    \
            0,                    \
            0,                    \
            9500,                 \
            0,                    \
            0,                    \
            &Position_PID,        \
    }

#define FollowYawSpeed_pidInit \
    {                          \
        0,                     \
            0,                 \
            0,                 \
            0,                 \
            0,                 \
            0.6f,              \
            0.0f,              \
            0.0f,              \
            0,                 \
            0,                 \
            0,                 \
            0,                 \
            5000,              \
            0,                 \
            0,                 \
            &Position_PID,     \
    }
#elif Infantry_Year == Infantry_2021
#define FollowYawAttitude_pidInit \
    {                             \
        0,                        \
            0,                    \
            0,                    \
            0,                    \
            0,                    \
            6500.0f,              \
            0.0f,                 \
            0.0f,                 \
            0,                    \
            0,                    \
            0,                    \
            0,                    \
            5500,                 \
            0,                    \
            0,                    \
            &Position_PID,        \
    }

#define FollowYawSpeed_pidInit \
    {                          \
        0,                     \
            0,                 \
            0,                 \
            0,                 \
            0,                 \
            0.425f,            \
            0.0f,              \
            0.0f,              \
            0,                 \
            0,                 \
            0,                 \
            0,                 \
            10000,             \
            0,                 \
            0,                 \
            &Position_PID,     \
    }
#endif

#define FollowYaw_pidInit       \
    {                           \
        0,                      \
            0,                  \
            0,                  \
            0,                  \
            0,                  \
            2000.0f,            \
            0.0f,               \
            0.0f,               \
            0,                  \
            0,                  \
            0,                  \
            0,                  \
            10000,              \
            0,                  \
            0,                  \
            &ClassisFollow_PID, \
    }

#define TwisterYaw_pidInit       \
    {                            \
        0,                       \
            0,                   \
            0,                   \
            0,                   \
            0,                   \
            6000.0f,             \
            0.0f,                \
            0.0f,                \
            0,                   \
            0,                   \
            0,                   \
            0,                   \
            10000,               \
            0,                   \
            0,                   \
            &ClassisTwister_PID, \
    }

#define AutoTrace_pidInit        \
    {                            \
        0,                       \
            0,                   \
            0,                   \
            0,                   \
            0,                   \
            -10.0f,              \
            0.0f,                \
            0.0f,                \
            0,                   \
            0,                   \
            0,                   \
            0,                   \
            9900,                \
            0,                   \
            0,                   \
            &Vision_AutoTracPID, \
    }

#define LFWHEEL_PID_PARAM     \
    {                         \
        0,                    \
            0,                \
            0,                \
            0,                \
            0,                \
            13.0f,            \
            0.5f,             \
            0.0f,             \
            0,                \
            0,                \
            0,                \
            0,                \
            M3508_MaxOutput,  \
            3000,             \
            &Incremental_PID, \
    }
#define RFWHEEL_PID_PARAM     \
    {                         \
        0,                    \
            0,                \
            0,                \
            0,                \
            0,                \
            13.0f,            \
            0.5f,             \
            0.0f,             \
            0,                \
            0,                \
            0,                \
            0,                \
            M3508_MaxOutput,  \
            3000,             \
            &Incremental_PID, \
    }
#define LBWHEEL_PID_PARAM     \
    {                         \
        0,                    \
            0,                \
            0,                \
            0,                \
            0,                \
            13.0f,            \
            0.5f,             \
            0.0f,             \
            0,                \
            0,                \
            0,                \
            0,                \
            M3508_MaxOutput,  \
            3000,             \
            &Incremental_PID, \
    }
#define RBWHEEL_PID_PARAM     \
    {                         \
        0,                    \
            0,                \
            0,                \
            0,                \
            0,                \
            10.0f,            \
            0.3f,             \
            0.0f,             \
            0,                \
            0,                \
            0,                \
            0,                \
            M3508_MaxOutput,  \
            3000,             \
            &Incremental_PID, \
    }

#define Chassis_FUNGroundInit    \
    {                            \
        &Chassis_Init,           \
            &Chassis_processing, \
            &ChassisCapControl,  \
    }

#define Twister_ControlGroundInit \
    {                             \
        0,                        \
            0,                    \
            0,                    \
            0,                    \
            &Twister_Flag,        \
    }

#define SpinSpeedRampInit \
    {                     \
        0,                \
            20,           \
            0,            \
            0,            \
    }

typedef enum
{
    TwisterFlagStart = 0,
    TwisterFlagLeft = 1,
    TwisterFlagRight
} Twister_Flag_t;

typedef struct
{
    float MechAngle;
    float TwisterAngle;
    uint16_t TwisterFlag; //扭腰方向标志位
    bool IfCorgiChange;   //转向是否改变
    Twister_Flag_t *Twister_Flag;
} Twister_Control_t;

typedef struct
{
    int transition;
    uint16_t ReversefollowTime;
    uint8_t NewFollowFlag; //两种跟随标志位
} ReversefollowFlag_t;

typedef struct
{
    int BIG_WHEELEndFlag;      //打符后跑路
    uint16_t BIG_WHEELEndTime; //打符后跑路时间
    float Lsat_Angle;          //打符前记录角度
} BIGWHEEL_EndRunFlag_t;

/* 底盘运动 */
typedef struct
{
    float targetXRaw;         //底盘x轴原始数据
    float targetYRaw;         //底盘y轴原始数据
    float targetZRaw;         //底盘z轴原始数据
    float LpfAttFactor;       //底盘滤波系数
    float targetXLPF;         //底盘x轴滤波后数据
    float targetYLPF;         //底盘y轴滤波后数据
    float targetZLPF;         //底盘z轴滤波后数据
    float speedLimit;         //底盘速度限制
    uint16_t OmegaLimit;      //底盘速度限制
    float Trace_Distance;     //跟随装甲板距离
    float FollowtargetYawRaw; //底盘目标Yaw轴跟随云台原始数据
    float FollowtargetYawLPF; //底盘Yaw轴跟随云台滤波后数据

    float SpeedChange_Factor;  //速度改变因子
    float SpeedLimit_Factor;   //限速因子
    uint8_t mode;              //底盘控制模式
    uint8_t swingFlag;         //扭腰标志位
    float spinSpeed;           //自旋速度
    float swingSpeed;          //扭腰速度
    uint8_t PowerOverflowFlag; //超功率标志位
} Chassis_t;

typedef struct
{
    void (*Chassis_Init)(void);
    void (*Chassis_processing)(float Vx, float Vy, float VOmega);
    void (*ChassisCapControl)(void);
} Chassis_FUN_t;

extern Chassis_t Chassis;
extern Chassis_FUN_t Chassis_FUN;
extern GY_IMUExport_t IMUExport_Chassis;
extern ReversefollowFlag_t ReversefollowFlag;
extern BIGWHEEL_EndRunFlag_t BIGWHEEL_EndRunFlag;
extern Twister_Control_t Twister_Control;
extern float Charging_Power;
extern float Vy_Test;
extern positionpid_t FollowYaw_pid;

int ComputeMinOffset(int target, int value);

#endif /* __CHIASSIS_CONTROL_H */
