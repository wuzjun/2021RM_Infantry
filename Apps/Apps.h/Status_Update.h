/**
 * @file Status_Update.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _STAUS_UPDATE_H
#define _STAUS_UPDATE_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h> //机器人的默认配置文件。
#include "SpeedRamp.h"

#define ForwardBackGroundInit \
    {                         \
        0,                    \
            0,                \
            -9900,            \
            9900,             \
    }

#define LeftRightGroundInit \
    {                       \
        0,                  \
            0,              \
            -9900,          \
            9900,           \
    }

#define RotateGroundInit \
    {                    \
        0,               \
            0,           \
            1,           \
            13,          \
    }

#define Status_FUNGroundInit   \
    {                          \
        &RemoteControl_Update, \
    }

typedef struct
{
    void (*RemoteControl_Update)(void);
} Status_FUN_t;

extern Status_FUN_t Status_FUN;
extern float Forward_Back_Value_Direction;
extern int SpinDirection_Flag;

#endif /*_STAUS_UPDATE_H*/
