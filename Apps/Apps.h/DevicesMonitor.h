/**
 * @file DevicesMonitor.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __DEVICESMONITOR_H
#define __DEVICESMONITOR_H

#include "tim.h"
#include "typedef.h"
#include <AddMath.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define DevicesMonitor_FUNGroundInit \
    {                                \
        &DevicesMonitor_update,      \
            &DevicesMonitor_Init,    \
            &Buzzer_On,              \
            &DevicesMonitor_Alert,   \
            &DevicesInit,            \
    }

typedef enum
{
    CHECK_M3508 = 0,
    CHECK_DR16 = 1,
    CHECK_Supercapacitor,
    CHECK_WolfIMU,
    CHECK_M6020,
    CHECK_DJI_CloudIMU,
    CHECK_DJI_C_CloudIMU,
    CHECK_M2006,
    CHECK_M3508F,
    CHECK_Vision,
    CHECK_CAN,
    CHECK_Judge,
    //数组下标
    CHECK_LIST_Device,
} CHECK_Devices_t;

typedef struct
{
    void (*GetDevices)(void);
} Check_Fun_t;

typedef struct
{
    void (*DevicesMonitor_update)(void);
    void (*DevicesMonitor_Init)(void);
    void (*Buzzer_On)(bool on, int volume);
    void (*DevicesMonitor_Alert)(void);
    void (*DevicesInit)(void);
} DevicesMonitor_FUN_t;

extern DevicesMonitor_FUN_t DevicesMonitor_FUN;

#endif /* __DEVICESMONITOR_H */
