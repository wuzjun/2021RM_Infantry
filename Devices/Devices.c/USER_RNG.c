/**
 * @file USER_RNG.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-05-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "USER_RNG.h"

//RNG ：硬件随机数发生器 ：提供个32位的随机数
//两个连续随机数之间的间隔是40个PLL48CLK时钟信号周期
//若随机数发生器发生错误，则可以在SR寄存器中去读到它的状态
//若其发生错误，还可以产生中断
//数据需要就绪才可以去读取随机数，需要大量的种子产生
//取一定范围内的随机数，是通过除以range取余法
//range(min,max) X % (max-min+1)+min 即是range(min,max)内的随机数
//获取一定范围内的随机数
uint16_t Get_RandomNumbers_Range(int16_t min,int16_t max)
{
 uint32_t rng_number;
 //无需对其状态位进行判断，因为在获取随机数的函数中，本身就带有检测
 // if(__HAL_RNG_GET_FLAG(&hrng,RNG_FLAG_DRDY) == SET)
 // {}
 rng_number = HAL_RNG_GetRandomNumber(&hrng);
 
 return rng_number % (max - min + 1) + min;
}

