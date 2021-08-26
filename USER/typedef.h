/*
 * File:          typedef.h
 * Date:       2021/3/29
 * Description:   外部接口
 * Author:  Miraggio
 * Modifications:
 */
#ifndef __TYPEDEFS_H
/**
 * @file typedef.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#define __TYPEDEFS_H

#include "can.h"
#include <stdint.h>

#pragma anon_unions

/*************CAN对外数据接口************/
typedef struct
{
    CAN_RxHeaderTypeDef CAN_RxHeader;
    uint8_t CANx_Export_RxMessage[8];
} Can_Export_Data_t;

/*************CAN发送数据接口************/
typedef struct
{
    CAN_HandleTypeDef *Canx;
    CAN_TxHeaderTypeDef CAN_TxHeader;
    uint8_t CANx_Send_RxMessage[8];
} Can_Send_Data_t;

/**********陀螺仪对外数据接口************/
typedef struct
{
    struct
    {
        float x; //浮点数pitch轴的方向向量
        float y; //浮点数Y轴方向向量
        float z; //浮点数yaw轴的转动速度
    } Finally_Gyro;
    struct
    {
        float Roll;  //ROLL轴方向，当前的角度值
        float Pitch; //PITCH轴方向
        float Yaw;   //YAW轴方向
    } Finally_Eular;

    float Finally_totalPitch;
    float Target_pitch; //开机定义值
    uint8_t OffLineFlag;
    uint32_t IMUExport_FPS;
} GY_IMUExport_t;

/**********系统时间对外数据接口************/
typedef struct
{
    uint32_t WorldTime;      //世界时间
    uint32_t Last_WorldTime; //上一次世界时间
} WorldTime_RxTypedef;
void Get_FPS(WorldTime_RxTypedef *time, uint32_t *FPS);

/************临时全局访问变量***************/
extern uint32_t Robot_FPS;
extern uint32_t GetData_FPS;
extern uint8_t RoboInit_Complete;

#endif /* __TYPEDEFS_H */
