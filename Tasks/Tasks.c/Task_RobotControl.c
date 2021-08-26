/**
 * @file Task_RobotControl.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Task_RobotControl.h"
#include "Robot_control.h"
#include "Status_Update.h"

/**
 * @brief 
 * 
 * @param argument 
 * @return  
 */
void RobotControl(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //每十毫秒强制进入总控制

    for (;;)
    {

        Robot_control_FUN.Robot_control();
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
