/**
 * @file Shoot.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __SHOOT_H
#define __SHOOT_H

#include "Flash_Store.h"
#include "M2006_Motor.h"
#include "PID.h"
#include "Parabola.h"
#include <SpeedRamp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/**********************************攻击系统框架结构****************************/
/*
	步兵4号：
			左发射装置 shootUnit_1
					攻击输出 FricOutput等
					拨弹装置Reloader

			右发射装置 shootUnit_2
					攻击输出 FricOutput等
					拨弹装置Reloader

	英雄：
			发射装置 shootUnit
				攻击输出 FricOutput等
				小拨弹装置Reloader

				大拨弹装置不考虑，本身就是不合理的设计，弄个pid让他一直转就好了。

	函数默认都以双发射机来写，单发射时只赋值第一个参数即可，
	拨弹装置默认使用2006
	默认上1下2， 左1右2，前1后2的序号来给各装置定义变量名。
*/
/************************************************************************/

#define Reloader1Direction 1;      //拨弹装置1的转动方向，如果与自己需要的方向反了就取反 -1
#define Angle_Pill (8191 * 36 / 9) //拨弹装置需要转动多少机械角度来发射一颗弹丸 = 8191*36/9
#define Fixed_BulletHeat 10        //固定热量值
#define FricOffNum 4               //摩擦轮偏移量长度
#define EraseWrite                 //是否擦除射速偏移量扇区并写入数据

#define M2006s_YawOPIDInit   \
    {                        \
        0,                   \
            0,               \
            0,               \
            0,               \
            0,               \
            0.34f,           \
            0.0f,            \
            0.0f,            \
            0,               \
            0,               \
            0,               \
            0,               \
            M2006_MaxOutput, \
            0,               \
            3000,            \
            &Position_PID,   \
    }

#define M2006s_YawIPIDInit   \
    {                        \
        0,                   \
            0,               \
            0,               \
            0,               \
            0,               \
            3.4f,            \
            0.0f,            \
            0.0f,            \
            0,               \
            0,               \
            0,               \
            0,               \
            M2006_MaxOutput, \
            0,               \
            3000,            \
            &Position_PID,   \
    }

#define M3508_PowerLPIDInit   \
    {                         \
        0,                    \
            0,                \
            0,                \
            0,                \
            0,                \
            20.0f,            \
            0.6f,             \
            0.0f,             \
            0,                \
            0,                \
            0,                \
            0,                \
            M2006_MaxOutput,  \
            3000,             \
            &Incremental_PID, \
    }

#define M3508_PowerRPIDInit   \
    {                         \
        0,                    \
            0,                \
            0,                \
            0,                \
            0,                \
            17.0f,            \
            1.2f,             \
            0.0f,             \
            0,                \
            0,                \
            0,                \
            0,                \
            M2006_MaxOutput,  \
            3000,             \
            &Incremental_PID, \
    }

#define Shoot_FunGroundInit            \
    {                                  \
        &Shoot_Init,                   \
            &Shoot_processing,         \
            &Shoot_openCartridge,      \
            &Shoot_setNeedLaunchCount, \
            &FricRead,                 \
            &FricWrite,                \
            false,                     \
    }

extern uint32_t auto_shoottime;
extern int need_shoottime;
extern int bullet1;
extern int need_shoottime1;
extern int needShoot;
extern int last_need_shoottime;

typedef enum
{
    Reloader_allow = 0,
    Reloader_ban = 1
} Reloader_license_t;

typedef struct
{
    int32_t totalAngle_start;         //记录任务开始时拨盘的位置
    uint8_t alreadyPush;              // 已经开始执行拨弹任务。
    uint8_t completePush;             //执行成功
    uint8_t PillObstruct;             //弹丸阻塞标志
    uint8_t PillObstruct_Direction;   //卡弹后任务的执行方向。1反转，0正传
    int16_t PillObstruct_Time;        //弹丸阻塞时间
    int32_t PillObstruct_targetAngle; //弹丸阻塞激活后，启用倒转目标值。
    int32_t amount_hadLaunch;         //当前任务已拨动的量。
    uint16_t ObstructSpeed;           //用来判断电机卡弹的速度值
    Reloader_license_t Reloader_license;
} Reloader_t; //一个拨弹装置。

typedef struct
{

    double shootSpeed_Judge; //当前摩擦轮的速度大小对应到裁判系统检测到的速度值。

    uint16_t NeedLaunchCount; //还需要发射的弹丸数量。

    //拨盘当前的拨弹任务结构体。
    Reloader_t Reloader;
    M2006s_t *Reloader_Motor; //拨弹装置对应的电机，需要初始化时指向对应的结构体

    uint16_t shootInterval; //发射间隔，单位毫秒。
    uint16_t bullet;        //发射子弹
    uint16_t FricSpeed;     //摩擦轮速度
    uint16_t FricOutput;    //攻击输出大小，最后赋给两个摩擦轮的输出大小
    bool FricUpdata;

} ShootUnit_t; //发射装置

typedef struct
{
    int shoot_LimitBullet;
    float shoot_OverallBullet;
    float shoot_JudgeOverallBullet;
    int shoot_RealBullet;
} Shoot_JudgeData_t; //直接调用裁判系统模块的数据变量太长，改为统一结构体。

typedef struct
{
    void (*Shoot_Init)(void);
    void (*Shoot_processing)(void);
    bool (*Shoot_openCartridge)(bool open);
    void (*Shoot_setNeedLaunchCount)(ShootUnit_t *shootUnit, uint16_t amount);
    void (*FricRead)(void);
    void (*FricWrite)(void);
    bool Cartridge_openFlag;
} Shoot_Fun_t;

extern Shoot_Fun_t Shoot_Fun;

extern ShootUnit_t shootUnit1;
extern Shoot_JudgeData_t Shoot_Judge;

extern int needShoot; //需要射击的颗数
extern uint32_t FricOffset[FricOffNum];

#ifdef Enable_shootUnit2
extern ShootUnit_t shootUnit2; //使用双发射
#endif

#endif /* __SHOOT_H */
