/**
 * @file Robot_control.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Robot_control.h"
#include "BSP_CAN.h"
#include "Chassis_control.h"
#include "Cloud_control.h"
#include "Control_Vision.h"
#include "DJI_IMU.h"
#include "DR16_Remote.h"
#include "Laser.h"
#include "M6020_Motor.h"
#include "Monitor_RM_CAN.h"
#include "RMClient_UI.h"
#include "RM_JudgeSystem.h"
#include "Shoot.h"
#include "SuperCapacitor.h"
#include "typedef.h"

void Robot_init(void);
void CAN_SendControl(void);
void Robot_control(void);
void Robot_ChangeControlSource(ControlSource_e controlSource);
void Robot_Reset(void);
void Robot_Enable(void);
void Robot_Disable(void);
void Robot_setChassisWorkMode(ChassisWorkMode_e mode);
void Robot_setAttackMode(AttackMode_e mode);
Robot_control_FUN_t Robot_control_FUN = Robot_control_FUNGroundInit;
#undef Robot_control_FUNGroundInit

Robot_t Robot; //全局机器人对象。
uint8_t RM_UITime;

/**
 * @brief 机器人状态初始化
 * 
 */
void Robot_init(void)
{
    Control_Vision_FUN.Vision_ID_Init();
    Robot.Attack_ShootTarget = ShootTarget_Self_aiming;
    Robot.Magazine = Magazine_Close;
    SuperCapacitorFUN.SuperCapacitor_Switch(false);
    UI_DeleteAll();
    for (int i = 0; i < 9; i++)	
    {
        Dispaly_Init[i]();
    }
    if (Robot.ControlSource == ControlSource_PC)
    {
        Robot.Sport_ChassisWorkMode = ChassisWorkMode_Follow;
        Robot.Sport_CloudWorkMode = CloudWorkMode_Normal;
        Robot.Device_FrictMode = FrictWorkMode_Disable;
        Robot.Attack_AttackMode = AttackMode_ManualMode;
    }

    LASER_Set(GPIO_PIN_RESET);
#ifdef Enable_Vision_Test
    Vision_I_T_Set('1', '2');
#endif
}

static void Chassis_control()
{
    //超级电容通信计数器
    //	Cap_Tick_5MS++;
    //	if (Cap_Tick_5MS >= 2)
    //	{
    //		ChassisCapControl();
    //		Cap_Tick_5MS = 0;
    //	}

    Chassis_FUN.ChassisCapControl();
    Chassis_FUN.Chassis_processing(DR16_Export_Data.Robot_TargetValue.Left_Right_Value, DR16_Export_Data.Robot_TargetValue.Forward_Back_Value, DR16_Export_Data.Robot_TargetValue.Omega_Value);
}

/**
 * @brief 云台控制
 * 
 */
static void Cloud_control(void)
{
    Cloud_FUN.IMUData_chance();
    if (Robot.Sport_ChassisWorkMode == ChassisWorkMode_NewFollow || ReversefollowFlag.NewFollowFlag == 1)
    {
        Cloud_FUN.Cloud_processing_WithoutIMU(DR16_Export_Data.Robot_TargetValue.Yaw_Value, DR16_Export_Data.Robot_TargetValue.Pitch_Value);
    }
    else
    {
        Cloud_FUN.Cloud_processing_WithIMU(DR16_Export_Data.Robot_TargetValue.Yaw_Value, DR16_Export_Data.Robot_TargetValue.Pitch_Value);
    }
}

/**
 * @brief Can总发送
 * 
 */
void CAN_SendControl(void)
{
    //解决一个报文发给不同电机的问题。
    //CAN1 ：yaw = 1 ， 拨盘 = 7
    //CAN2 ： pitch = 2

    // -------  CAN1
    Monitor_CAN_FUN.CAN_0x200_SendData(&hcan1, M3508s[0].outCurrent, M3508s[1].outCurrent, M3508s[2].outCurrent, M3508s[3].outCurrent); //底盘电机(3508)

    Monitor_CAN_FUN.CAN_0x1FF_SendData(&hcan1, M6020s_Yaw.outCurrent, 0, 0, 0); //pitch轴电机(6020)，yaw轴电机(6020)。

    // Monitor_CAN_FUN.CAN_0x2FF_SendData(&hcan1, 5000, 0, 0, 0); //pitch轴电机(6020)，yaw轴电机(6020)。

    Monitor_CAN_FUN.CAN_0x601_SendData(&hcan1, supercapacitor.SendData.data);
    // -------  CAN2
    Monitor_CAN_FUN.CAN_0x1FF_SendData(&hcan2, M3508_PowerL.outCurrent, M6020s_Pitch.outCurrent, M2006_Reload.outCurrent, M3508_PowerR.outCurrent);
}

WorldTime_RxTypedef Control_WorldTime;
uint32_t Robot_FPS;
void Robot_control(void)
{

    Chassis_control();
    Cloud_control();
    Shoot_Fun.Shoot_processing();

    CAN_SendControl();
    Get_FPS(&Control_WorldTime, &Robot_FPS);
}

/**
	* @brief  更改机器人控制来源
 * @param	void
 * @retval None
 */
void Robot_ChangeControlSource(ControlSource_e controlSource)
{
    if (Robot.ControlSource != controlSource) //发生模式跳变，重置。
    {
        Robot_Reset();
    }
    Robot.ControlSource = controlSource;
}
/**
 * @brief  状态复位
 * @param	void
 * @retval None
 */
void Robot_Reset(void)
{
    //重新初始化机器人。
    Robot.Sport_ChassisWorkMode = ChassisWorkMode_Follow;
    Robot.Sport_CloudWorkMode = CloudWorkMode_Normal;
    Robot.Device_FrictMode = FrictWorkMode_Disable;
    Robot.Sport_AttachWorkMode = AttachWorkMode_Disable;
    Robot.Attack_AttackMode = AttackMode_ManualMode;
    Robot.Attack_ShootTarget = ShootTarget_Self_aiming;
    Robot.VisionEnabled = false;
    LASER_Set(GPIO_PIN_RESET);
}

/**
 * @brief 机器人全局使能
 * 
 */
void Robot_Enable(void)
{
    Robot.WorkStatus = WorkStatus_Normal;
}

/**
 * @brief 机器人全局失能
 * 
 */
void Robot_Disable(void)
{
    DR16_Export_Data.Robot_TargetValue.Forward_Back_Value = 0;
    DR16_Export_Data.Robot_TargetValue.Left_Right_Value = 0;
    DR16_Export_Data.Robot_TargetValue.Omega_Value = 0;
    DR16_Export_Data.Robot_TargetValue.Pitch_Value = 0;
    DR16_Export_Data.Robot_TargetValue.Yaw_Value = 0;
    Robot.Sport_ChassisWorkMode = ChassisWorkMode_Disable;
    Robot.Sport_AttachWorkMode = AttachWorkMode_Disable;
    Robot.Sport_CloudWorkMode = CloudWorkMode_Disable;
    Robot.Attack_AttackMode = AttackMode_Disable;
    Robot.Device_FrictMode = FrictWorkMode_Disable;
    Robot.Attack_ShootTarget = ShootTarget_Self_aiming;
    Robot.VisionEnabled = false;

    /***************云台跟随底盘结构体清零*************/
    ReversefollowFlag.NewFollowFlag = 0;
    ReversefollowFlag.ReversefollowTime = 0;
    ReversefollowFlag.transition = 0;

    /***************风车结构体清零*************/
    BIGWHEEL_EndRunFlag.BIG_WHEELEndFlag = 0;
    BIGWHEEL_EndRunFlag.BIG_WHEELEndTime = 0;
    BIGWHEEL_EndRunFlag.Lsat_Angle = 0;
    LASER_Set(GPIO_PIN_RESET);
}

void Robot_setAttackMode(AttackMode_e mode)
{
    // if (Robot.Attack_AttackMode == AttackMode_AutoMode && mode != AttackMode_AutoMode)
    // {
    //     shootUnit1.NeedLaunchCount = 0; //当自动射击模式被取消时，清零自动射击的数量
    // }
    // Robot.Attack_AttackMode = mode;

    switch (Robot.Attack_AttackMode)
    {
    case AttackMode_ManualMode:
        Robot.Device_FrictMode = FrictWorkMode_HighSpeed;
        break;
    case AttackMode_Disable:
        Robot.Device_FrictMode = FrictWorkMode_Disable;
        break;
    case AttackMode_AutoMode:
        Robot.Device_FrictMode = FrictWorkMode_HighSpeed;
        break;
    default:
        Robot.Device_FrictMode = FrictWorkMode_Disable;
        break;
    }
}

void Robot_setChassisWorkMode(ChassisWorkMode_e mode)
{

    if (Robot.Sport_ChassisWorkMode == ChassisWorkMode_Spin && mode != ChassisWorkMode_Spin)
    {
        //需要清零自旋后的totalAngle
        M6020_Fun.M6020_Reset(&M6020s_Yaw); //将自旋积累的total清零。
    }

    Robot.Sport_ChassisWorkMode = mode;
}
