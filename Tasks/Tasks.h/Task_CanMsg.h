/**
 * @file Task_CanMsg.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __TASK_CANMSG_H
#define __TASK_CANMSG_H

#include "Handle.h"

enum
{
    M6020 = 0,      /* 6020接收 */
    M3508,          /* 3508接收 */
    M3508_Friction, /* 3508（摩擦轮）接收 */
    M2006,          /* 2006接收 */
    Wolf_IMU,       /* 彬哥陀螺仪接收 */
    DJI_IMU,        /* C板陀螺仪接收 */
};

void Can1Receives(void const *argument);
void Can2Receives(void const *argument);

#endif /*__TASK_CANMSG_H*/
