/**
 * @file USER_RNG.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-05-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _USER_RNG_H
#define _USER_RNG_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h> //机器人的默认配置文件。
#include <rng.h>

uint16_t Get_RandomNumbers_Range(int16_t min,int16_t max);

#endif /* _USER_RNG_H */

