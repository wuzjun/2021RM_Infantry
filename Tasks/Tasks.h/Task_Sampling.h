/**
 * @file Task_Sampling.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _TASK_SAMPLING_
#define _TASK_SAMPLING_

#include "Chassis_control.h"
#include "Cloud_control.h"
#include "Wolf_GyIMU.h"
#include "cmsis_os.h"

void Fixed_Sampling(void const *argument);
extern uint32_t STest_FPS;

#endif /*_TASK_SAMPLING_*/

