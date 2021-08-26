/**
 * @file Parabola.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-04-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __Parabola_H
#define __Parabola_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "AddMath.h"
#include "stm32f4xx.h"
#include "arm_math.h"
#include "math.h"

bool Parabola_Matching(double angle_P, double d, double v0, double *result);



#endif
