/**
 * @file Laser.h
 * @author Miraggio (w1159904119@gmail)
 * @brief Dji激光应用函数接口
 * @version 0.1
 * @date 2021-04-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __LASER_H
#define __LASER_H

#include "gpio.h"


void LASER_Set(GPIO_PinState open);

#endif /* __LASER_H */
