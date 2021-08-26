/**
 * @file Robot_control.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __Robot_CONTROL_H
#define __Robot_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/************************************************************************/
/*
特别说明：
以下枚举都带有前缀的原因：防止枚举名重复，枚举混乱导致不必要的问题。
*/
/************************************************************************/

//机器人控制的来源，非信号来源。
typedef enum
{
    ControlSource_RC = 0, //遥控器模式
    ControlSource_PC = 1, //键鼠模式
    ControlSource_Stop    //关闭机器
} ControlSource_e;

//底盘工作模式
typedef enum
{
    ChassisWorkMode_Follow = 0,        //跟随云台模式
    ChassisWorkMode_NewFollow = 1,     //跟随底盘模式
    ChassisWorkMode_Reversefollow = 2, //反向跟随云台模式
    ChassisWorkMode_Spin = 3,          //边走边旋模式
    ChassisWorkMode_Twister,           //扭腰模式
    ChassisWorkMode_AutoTrace,         //自动追踪模式
    ChassisWorkMode_Supply,            //补给模式
    ChassisWorkMode_Lock,
    ChassisWorkMode_Disable //失能模式
} ChassisWorkMode_e;

//底盘工作模式
typedef enum
{
    AttachWorkMode_Slow = 0, //展览慢速运动
    AttachWorkMode_Disable   //附加模式关闭
} AttachWorkMode_e;

//超级电容工作模式
typedef enum
{
    SuperCapacitor_Off = 0, //关闭超级电容
    SuperCapacitor_Open
} SuperCapacitorMode_e;

//云台工作模式
typedef enum
{
    CloudWorkMode_Normal = 0, //正常模式（360°云台）
    CloudWorkMode_LimitAngle, //角度限制模式（非360°云台）
    CloudWorkMode_SupplyWork, //补给模式
    CloudWorkMode_Disable     //失能
} CloudWorkMode_e;

typedef enum
{
    SpecialMode_Normal = 0,
    SpecialMode_BigSymbol, //大符模式
    SpecialMode_UPBridge,  //上桥模式
    SpecialMode_Supply,    //补给站模式
} SpecialMode_t;
//机器人工作状态
typedef enum
{
    WorkStatus_Prepare, //从失能进入到正常状态的过渡阶段，开机时候初始化的阶段，机器人设备离
    WorkStatus_Disable, //失能状态，应对紧急情况和开机启动。
    WorkStatus_Normal   //正常运行状态
} WorkStatus_e;

typedef enum
{
    Open,
    Close
} PlayRoomStatus_e;

//拨弹机状态
typedef enum
{
    RammerWorkMode_Disable,  //拨弹关闭状态
    RammerWorkMode_Normal,   //正常待命状态
    RammerWorkMode_MaxSpeed, //持续最大输出
} RammerWorkMode_e;

//摩擦轮模式
typedef enum
{
    FrictWorkMode_Disable,   //摩擦轮关闭模式
    FrictWorkMode_HighSpeed, //摩擦轮高速模式
    FrictWorkMode_LowSpeed,  //摩擦轮低速模式
    FrictWorkMode_AutoSpeed  //摩擦轮自动模式

} FrictWorkMode_e;

typedef enum
{
    Magazine_Close, //弹仓关闭
    Magazine_Open,  //弹仓开启
} Magazine_e;

//攻击模式
typedef enum
{
    AttackMode_Disable,        //失能攻击
    AttackMode_AutoMode,       //自动瞄准
    AttackMode_ManualMode,     //手动瞄准
    AttackMode_SingleMode,     //单发模式
    AttackMode_ContinuousMode, //连发模式
    AttackMode_NoneMode,       //不击打模式

} AttackMode_e;

typedef enum
{
    TeamColor_Blue,
    TeamColor_Red
} TeamColor_e;

typedef enum
{
    Types_Hero,
    Types_Engineer,
    Types_Standard,
    Types_Aerial = 6,
    Types_Sentry = 7
} Types_e;

typedef enum
{
    ShootTarget_default = 0,     //默认模式
    ShootTarget_Self_aiming = 1, //自瞄模式
    ShootTarget_BIG_WHEEL,       //打符模式
    ShootTarget_Sentry,          //击打对方哨兵的模式
    ShootTarget_Spin,            //打小陀螺模式
    ShootTarget_Video,           //录像模式
    ShootTarget_UAV,             //我方无人机自瞄模式
    ShootTarget_OurSentry,       //我方哨兵自瞄模式
    ShootTarget_Radar,           //雷达模式

} ShootTarget_e;

typedef enum
{
    OpenBurstOut_End = 0,
    OpenBurstOut_Start
} OpenBurstOut_e;

/**************************以上为枚举**************************/

typedef struct
{
    //控制相关
    ControlSource_e ControlSource; //机器人当前受什么设备控制
    WorkStatus_e WorkStatus;       //机器人是否处于准备状态。

    //运动相关
    ChassisWorkMode_e Sport_ChassisWorkMode;
    AttachWorkMode_e Sport_AttachWorkMode;
    CloudWorkMode_e Sport_CloudWorkMode;
    SuperCapacitorMode_e SuperCapacitorMode;

    //装置相关
    FrictWorkMode_e Device_FrictMode; //摩擦轮工作模式。
    Magazine_e Magazine;              //弹仓是否开启

    //攻击相关
    AttackMode_e Attack_AttackMode;
    ShootTarget_e Attack_ShootTarget;
    OpenBurstOut_e OpenBurstOut;
    bool VisionEnabled; //自瞄识别开关

    //比赛相关
    TeamColor_e TeamColor; //我方的团队颜色。
    Types_e Types;         //我方兵种

} Robot_t;

#define Robot_control_FUNGroundInit     \
    {                                   \
        &Robot_Enable,                  \
            &Robot_Disable,             \
            &Robot_Reset,               \
            &Robot_control,             \
            &Robot_init,                \
            &Robot_ChangeControlSource, \
            &Robot_setAttackMode,       \
            &Robot_setChassisWorkMode,  \
            &CAN_SendControl,           \
    }

typedef struct
{
    void (*Robot_Enable)(void);
    void (*Robot_Disable)(void);
    void (*Robot_Reset)(void);
    void (*Robot_control)(void);
    void (*Robot_init)(void);
    void (*Robot_ChangeControlSource)(ControlSource_e controlSource);
    void (*Robot_setAttackMode)(AttackMode_e mode);
    void (*Robot_setChassisWorkMode)(ChassisWorkMode_e mode);
    void (*CAN_SendControl)(void);

} Robot_control_FUN_t;

extern Robot_control_FUN_t Robot_control_FUN;
extern Robot_t Robot;

#endif
