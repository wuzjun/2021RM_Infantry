/**
 * @file Robot_Config.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __Robot_CONFIG_H
#define __Robot_CONFIG_H

/************************************************************************/
/*          这里是为了比赛应急情况，提前准备的方案。
			 目的：某硬件需要更换时，通过修改宏定义使机器人快速更换设备。
*/
/************************************************************************/



#define Device_BoardType 2 //0=彬哥的板子 ，1=2017旧官方板	， 2=2018新官方板
//以下当做枚举类型来使用。
#define Device_BoardType_Wolf 0
#define Device_BoardType_2017 1
#define Device_BoardType_2018 2

#define Infantry_Year 1
#define Infantry_2020 0
#define Infantry_2021 1

#define Friction__Wheel 0
#define M3508_Wheel 0 //M3508摩擦轮
#define Snail_Wheel 1 //Snail摩擦轮

#define Cloud_DeBug 0 //云台PID调参
#define Cloud_NoDeBug 0
#define Cloud_YesDeBug 1


#define Vision_Version 1
#define Vision_Oldversion 0 //旧视觉版本
#define Vision_Newversion 1 //新视觉版本

#define Cloud_IMU 0
#define Cloud_DJI_IMU 0
#define Cloud_Wolf_IMU 1

#define Who_Infantry 0
#define ZJ_Infantry 0
#define ZF_Infantry 1

//注释掉表示当前车不启用该模块
#define Enable_DeviceMonitor //启用离线、错误报警。
#define Enable_CloudControl
#define Enable_ShootSystem //启用射击系统
#define Enable_RMJudgeSystem
// #define Temporary_Function
//#define Enable_Buzzer //启用蜂鸣器
// #define Enable_Vision_Test
// #define Enable_shootUnit2  //是否开启双发射
#endif
