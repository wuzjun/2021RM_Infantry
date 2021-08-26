/**
 * @file M3508_Motor.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __M3508_MOTOR_H
#define __M3508_MOTOR_H

#include "PID.h"
#include "can.h"
#include "typedef.h"
#include <stdbool.h>
#include <stdint.h>

#define M3508_READID_START 0x201
#define M3508_READID_END 0x204
#define M30508_FricL_ID 0x205
#define M30508_FricR_ID 0x208
#define M3508_SENDID 0x200
#define M3508_MaxOutput 16384     //发送给电机的最大控制值
#define M3508_CurrentRatio 819.2f //16384/20A = 819.2->1A

#define M3508_FunGroundInit          \
    {                                \
        &M3508_getInfo,              \
            &M3508_Friction_getInfo, \
            &Check_WheelM3508,       \
            &Check_FrictionM3508,    \
    }

typedef struct
{
    uint16_t realAngle;  //读回来的机械角度
    int16_t realSpeed;   //读回来的速度
    int16_t realCurrent; //读回来的实际电流
    uint8_t temperture;  //读回来的电机温度

    int16_t targetSpeed;  //目标速度
    uint16_t targetAngle; //目标角度
    uint16_t lastAngle;   //上次的角度
    int32_t totalAngle;   //累积总共角度
    int16_t turnCount;    //转过的圈数

    int16_t outCurrent; //输出电流

    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} M3508s_t;

extern M3508s_t M3508s[4];
extern M3508s_t M3508_PowerL; //摩擦轮电机 201
extern M3508s_t M3508_PowerR; //摩擦轮电机; 202

typedef struct
{
    void (*M3508_getInfo)(Can_Export_Data_t RxMessage);
    void (*M3508_Friction_getInfo)(Can_Export_Data_t RxMessage);
    void (*Check_WheelM3508)(void);
    void (*Check_FrictionM3508)(void);
} M3508_FUN_t;

extern M3508_FUN_t M3508_FUN;

#endif /*__M3508_MOTOR_H*/
