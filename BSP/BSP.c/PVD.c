/**
 * @file PVD.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-05-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "PVD.h"

PWR_PVDTypeDef pvd_config; //定义句柄

void PVD_Config(void) //初始化
{
    __HAL_RCC_PWR_CLK_ENABLE(); //初始化时钟
    pvd_config.PVDLevel = PWR_PVDLEVEL_7;
    pvd_config.Mode = PWR_PVD_MODE_IT_RISING_FALLING;
    HAL_PWR_ConfigPVD(&pvd_config);
    HAL_PWR_EnablePVD(); //使能

    HAL_NVIC_EnableIRQ(PVD_IRQn);         //使能pvd中断通道
    HAL_NVIC_SetPriority(PVD_IRQn, 0, 0); //抢占优先级0，子优先级0
}

void HAL_PWR_PVDCallback(void) //中断回调函数
{
    __set_FAULTMASK(1);
    HAL_NVIC_SystemReset();
}
