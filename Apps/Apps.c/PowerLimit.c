/**
 * @file PowerLimit.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "PowerLimit.h"
#include "RM_JudgeSystem.h"
#include "Wolf_GyIMU.h"

PowerLimit_t chassis_powerLimit;
int16_t power_buffer = 0;
void PowerLimit_Calculate(PowerLimit_t *powerlimit)
{

    power_buffer = ext_power_heat_data.data.chassis_power_buffer; //缓冲热量大小 数据来源。

    if (RM_Judge.OffLineFlag == 1)
    {
        power_buffer = 120;
    }

    int16_t powerBuffErr = 60 - power_buffer; //用掉的缓冲能量。

    powerlimit->powerBuffRatio = 0;
    powerlimit->powerBuffRatio = (float)power_buffer / 76.0f; //用掉的缓冲能量。

    powerlimit->powerBuffRatio *= powerlimit->powerBuffRatio; //立方的关系。
    //powerlimit->powerBuffRatio *= powerlimit->powerBuffRatio;//立方的关系。

    if (powerBuffErr > 0)
    {
        powerlimit->SumCurrent_Out = powerlimit->SumCurrent_IN * powerlimit->powerBuffRatio;
    }
}

// void

void PowerLimit(PowerLimit_t *powerlimit, int16_t *wheelCurrent, int16_t amount)
{
    //float *coe = calloc(4, sizeof(float));//系数	动态分配数组内存。

    float coe[4] = {0.0}; //系数	动态分配数组内存。
//    float Wheelcoe[2][2] = {{0, 0}, {0.0}};
    int i = 0;
//    int32_t Wheel_IN[2] = {0};
    powerlimit->SumCurrent_IN = powerlimit->SumCurrent_Out = 0;

    // if (Wolf_GyIMU_Fun.GY6050_PitErr() > -20 && Wolf_GyIMU_Fun.GY6050_PitErr() < -10)
    // {
    //     //计算前轮电流
    //     for (i = 0; i < amount / 2; i++)
    //     {
    //         Wheel_IN[0] += abs(wheelCurrent[i]);
    //     }
    //     //计算前轮百分比
    //     for (i = 0; i < amount / 2; i++)
    //     {
    //         Wheelcoe[0][i] = ((float)(wheelCurrent[i])) / ((float)(Wheel_IN[0]));
    //     }
    //     //需要分配后轮的功率
    //     int32_t temp_FWheel = (int32_t)Wheel_IN[0] * 0.3f;
    //     Wheel_IN[0] -= temp_FWheel;
    //     //计算分配后，轮子该有的功率
    //     for (i = 0; i < amount / 2; i++)
    //     {
    //         wheelCurrent[i] = (int16_t)(Wheel_IN[0] * Wheelcoe[0][i]);
    //     }

    //     //计算后轮电流
    //     for (i = amount / 2; i < amount; i++)
    //     {
    //         Wheel_IN[1] += abs(wheelCurrent[i]);
    //     }
    //     //计算后轮百分比
    //     for (i = amount / 2; i < amount; i++)
    //     {
    //         Wheelcoe[1][i] = ((float)(wheelCurrent[i])) / ((float)(Wheel_IN[1]));
    //     }
    //     //加上前轮的功率
    //     Wheel_IN[1] += temp_FWheel;
    //     //计算分配后，轮子该有的功率
    //     for (i = amount / 2; i < amount; i++)
    //     {
    //         wheelCurrent[i] = (int16_t)(Wheel_IN[1] * Wheelcoe[1][i]);
    //     }
    // }

    //计算总电流
    for (i = 0; i < amount; i++)
    {
        powerlimit->SumCurrent_IN += abs(wheelCurrent[i]);
    }
    powerlimit->SumCurrent_Out = powerlimit->SumCurrent_IN; //不处理时为原来的值。

    //计算每个电流占总电流的百分比
    for (i = 0; i < amount; i++)
    {
        coe[i] = ((float)(wheelCurrent[i])) / ((float)(powerlimit->SumCurrent_IN));
    }

    //计算此时限制功率后的最大电流
    PowerLimit_Calculate(powerlimit); //内部指针的方式修改 SumCurrent_Out

    //按照百分比分配最大电流
    for (i = 0; i < amount; i++)
    {
        wheelCurrent[i] = (int16_t)(powerlimit->SumCurrent_Out * coe[i]);
    }
}
