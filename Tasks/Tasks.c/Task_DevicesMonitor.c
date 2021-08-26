/**
 * @file Task_DevicesMonitor.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Task_DevicesMonitor.h"
#include "DR16_Remote.h"
#include "OLED.h"
#include "Robot_Config.h"
#include "typedef.h"

/**
 * @brief 检测任务
 * 
 * @param argument 
 * @return  
 */
WorldTime_RxTypedef Detect_FPS;
uint32_t DeTest_FPS;
uint8_t RoboInit_Complete = 0; //机器人初始化完成标志位
void Detect(void const *argument)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(200); //每200毫秒强制进入总控制

    DevicesMonitor_FUN.DevicesInit();

    for (;;)
    {

        DevicesMonitor_FUN.DevicesMonitor_update();
#ifdef Enable_DeviceMonitor
        DevicesMonitor_FUN.DevicesMonitor_Alert();
#endif
        if (RoboInit_Complete == 0) //没初始化之前一直喂狗
        {
            HAL_IWDG_Refresh(&hiwdg); //喂狗
        }
        else
        {
            if (Robot_FPS == 500 && GetData_FPS == 1000 && DR16_Fun.DR16_DataCheck() == 0)
            {
                HAL_IWDG_Refresh(&hiwdg); //喂狗
            }
        }

        Get_FPS(&Detect_FPS, &DeTest_FPS);
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
