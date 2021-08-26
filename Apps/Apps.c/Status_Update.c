/**
 * @file Status_Update.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Status_Update.h"
#include "Chassis_control.h"
#include "Cloud_control.h"
#include "DJI_IMU.h"
#include "DR16_Remote.h"
#include "Laser.h"
#include "RMClient_UI.h"
#include "RM_JudgeSystem.h"
#include "Robot_control.h"
#include "Shoot.h"
#include "SuperCapacitor.h"
#include "USER_Filter.h"
#include "Wolf_GyIMU.h"

//底盘遥控前后斜坡
SpeedRamp_t ChassisRamp_ForwardBack = ForwardBackGroundInit;
#undef ForwardBackGroundInit

//底盘遥控左右斜坡
SpeedRamp_t ChassisRamp_LeftRight = LeftRightGroundInit;
#undef LeftRightGroundInit

//底盘遥控左右转斜坡
SpeedRamp_t ChassisRamp_Rotate = RotateGroundInit;
#undef RotateGroundInit

void RemoteControl_Update(void);
Status_FUN_t Status_FUN = Status_FUNGroundInit;
#undef Status_FUNGroundInit

/**
 * @brief 
 * 
 * @return  
 */
int SpinDirection_Flag = 1; /* 小陀螺反向 */
int test_shoot = 0;
static void RemoteMode_Update(void)
{

    switch (DR16_Export_Data.ControlSwitch->Left)
    {
    case RemotePole_UP: //-----左拨杆 为上--------------PC控制、蓝牙。

        //如果s1和s2都打上则为键鼠控制模式
        if (DR16_Export_Data.ControlSwitch->Right == RemotePole_UP)
        {
            Robot_control_FUN.Robot_ChangeControlSource(ControlSource_PC);
        }
        else if (DR16_Export_Data.ControlSwitch->Right == RemotePole_MID) //摩擦轮高速
        {
            Robot.Sport_ChassisWorkMode = ChassisWorkMode_Follow;
            Robot.Sport_AttachWorkMode = AttachWorkMode_Disable;
            Robot.Sport_CloudWorkMode = CloudWorkMode_Normal;
            Robot.Device_FrictMode = FrictWorkMode_AutoSpeed;
            Robot.Attack_ShootTarget = ShootTarget_Self_aiming;
            Robot.Attack_AttackMode = AttackMode_NoneMode;
            Robot.VisionEnabled = true;
            if (DR16_Export_Data.Robot_TargetValue.Dial_Wheel > 50) //连发
            {
                Robot.Attack_AttackMode = AttackMode_ManualMode;
                test_shoot = 0;
            }
            else if (DR16_Export_Data.Robot_TargetValue.Dial_Wheel < -50 && test_shoot == 0) //单发
            {
                Robot.Attack_AttackMode = AttackMode_SingleMode;
                test_shoot = 1;
            }
            else if (DR16_Export_Data.Robot_TargetValue.Dial_Wheel >= -50)
            {
                test_shoot = 0;
            }
            LASER_Set(GPIO_PIN_SET);
            Robot_control_FUN.Robot_ChangeControlSource(ControlSource_RC);
        }
        else //退弹模式
        {
            Robot.Sport_ChassisWorkMode = ChassisWorkMode_Follow;
            Robot.Sport_AttachWorkMode = AttachWorkMode_Disable;
            Robot.Sport_CloudWorkMode = CloudWorkMode_Normal;
            Robot.Device_FrictMode = FrictWorkMode_LowSpeed;
            Robot.Attack_ShootTarget = ShootTarget_Self_aiming;
            if (DR16_Export_Data.Robot_TargetValue.Dial_Wheel > 100) //退弹
            {
                Robot.Attack_AttackMode = AttackMode_ManualMode;
            }
            else if (DR16_Export_Data.Robot_TargetValue.Dial_Wheel < -200) //弹仓盖
            {
                Shoot_Fun.Cartridge_openFlag = Shoot_Fun.Shoot_openCartridge(true);
            }
            else
            {
                Shoot_Fun.Cartridge_openFlag = Shoot_Fun.Shoot_openCartridge(false);
            }
            Robot.VisionEnabled = false;
            LASER_Set(GPIO_PIN_RESET);
            Robot_control_FUN.Robot_ChangeControlSource(ControlSource_RC);
        }

        break;
    case RemotePole_MID: //-------左拨杆 为中	------------遥控器控制模式

        switch (DR16_Export_Data.ControlSwitch->Right)
        {
        case RemotePole_UP:
            Robot.Sport_ChassisWorkMode = ChassisWorkMode_Lock;
            Robot.Sport_AttachWorkMode = AttachWorkMode_Disable;
            Robot.Sport_CloudWorkMode = CloudWorkMode_Normal;
            Robot.Attack_AttackMode = AttackMode_NoneMode;
            Robot.Attack_ShootTarget = ShootTarget_BIG_WHEEL;
            Robot.Device_FrictMode = FrictWorkMode_AutoSpeed;
            if (DR16_Export_Data.Robot_TargetValue.Dial_Wheel > 0 && test_shoot == 0) //补偿++
            {
                Robot.Attack_AttackMode = AttackMode_SingleMode;
                test_shoot = 1;
            }
            else if (DR16_Export_Data.Robot_TargetValue.Dial_Wheel <= 0 && test_shoot == 1) //补偿--
            {
                test_shoot = 0;
            }
            Robot.VisionEnabled = true;
            LASER_Set(GPIO_PIN_SET);

            break;
        case RemotePole_MID: //左中====右中
            // Robot.Sport_ChassisWorkMode = ChassisWorkMode_Follow;
            Robot.Sport_ChassisWorkMode = ChassisWorkMode_Lock;
            Robot.Sport_AttachWorkMode = AttachWorkMode_Disable;
            Robot.Sport_CloudWorkMode = CloudWorkMode_Normal;
            Robot.Attack_AttackMode = AttackMode_Disable;
            Robot.Attack_ShootTarget = ShootTarget_Self_aiming;
            Robot.Device_FrictMode = FrictWorkMode_Disable;
            LASER_Set(GPIO_PIN_SET);
            Robot.VisionEnabled = false;
            if (DR16_Export_Data.Robot_TargetValue.Dial_Wheel > 200) //跟随装甲板
            {
                Robot.VisionEnabled = true;
                Robot.Sport_ChassisWorkMode = ChassisWorkMode_AutoTrace;
            }
            else if (DR16_Export_Data.Robot_TargetValue.Dial_Wheel < -200) //瞄准
            {
                Robot.VisionEnabled = true;
            }

            break;

        case RemotePole_DOWM: //左中====右下

            Robot.Sport_ChassisWorkMode = ChassisWorkMode_Follow;
            Robot.Sport_AttachWorkMode = AttachWorkMode_Disable;
            // Robot.Sport_AttachWorkMode = AttachWorkMode_Slow;
            Robot.Sport_CloudWorkMode = CloudWorkMode_Normal;
            Robot.Attack_AttackMode = AttackMode_Disable;
            Robot.Device_FrictMode = FrictWorkMode_Disable;
            Robot.Attack_ShootTarget = ShootTarget_Self_aiming;
            SuperCapacitorFUN.SuperCapacitor_Switch(false);

            if (DR16_Export_Data.Robot_TargetValue.Dial_Wheel > 200) //小陀螺
            {
                Robot.Sport_ChassisWorkMode = ChassisWorkMode_Spin;
            }
            else if (DR16_Export_Data.Robot_TargetValue.Dial_Wheel < -200) //超级电容
            {
                Robot.Sport_AttachWorkMode = AttachWorkMode_Disable;
                SuperCapacitorFUN.SuperCapacitor_Switch(true);
            }
            Robot.VisionEnabled = false;
            LASER_Set(GPIO_PIN_RESET);
            break;
        }
        //更改控制来源。
        Robot_control_FUN.Robot_ChangeControlSource(ControlSource_RC);
        break;

    case RemotePole_DOWM: //----------左拨杆 为下

        Robot_control_FUN.Robot_Disable();
        Robot_control_FUN.Robot_ChangeControlSource(ControlSource_RC);
        break;
    }
}

/**
 * @brief 更新RC端状态
 * 
 * @return  
 */
void RemoteControl_RC_Update(void)
{
    DR16_Export_Data.Robot_TargetValue.Forward_Back_Value *= 15.15f;
    DR16_Export_Data.Robot_TargetValue.Left_Right_Value *= 15.15f;
    DR16_Export_Data.Robot_TargetValue.Pitch_Value *= -0.03f;
    DR16_Export_Data.Robot_TargetValue.Yaw_Value *= 0.002f;

    if (imu_Export.OffLineFlag == 1)
    {
        DR16_Export_Data.Robot_TargetValue.Omega_Value = 0.0f;
    }

    if (Robot.Sport_ChassisWorkMode == ChassisWorkMode_Disable || Robot.Sport_ChassisWorkMode == ChassisWorkMode_Lock)
    { //只动云台
        DR16_Export_Data.Robot_TargetValue.Forward_Back_Value *= 0.0f;
        DR16_Export_Data.Robot_TargetValue.Left_Right_Value *= 0.0f;
        DR16_Export_Data.Robot_TargetValue.Omega_Value = 0.0f;
        DR16_Export_Data.Robot_TargetValue.Pitch_Value *= -0.2f;
        DR16_Export_Data.Robot_TargetValue.Yaw_Value *= 0.8f;
    }
}

/**
 * @brief 更新PC端状态
 * 
 * @return  
 */
static int RampRate = 100;              //斜坡函数叠加值
static float RampRate_factor = 0;       //斜坡函数叠加值
static float Mouse_LpfAttFactor = 0.5;  //PC鼠标滤波系数
float Forward_Back_Value_Direction = 1; //改变前进方向
float Left_Right_Value_Direction = 1;   //改变左右方向
float Yaw_Value;
float Pitch_Value;
void RemoteControl_PC_Update(void)
{
    if (Robot.Sport_ChassisWorkMode == ChassisWorkMode_Spin || Robot.Sport_ChassisWorkMode == ChassisWorkMode_Twister)
    {
        RampRate_factor = 2.4f;
    }
    else
    {
        RampRate_factor = 1.0f;
    }
    /************** 运动控制begin ******************/
    /************************** W S **********************/

    if (DR16_Fun.GetKeyMouseAction(KEY_W, KeyAction_CLICK)) //w
    {
        if (Robot.Attack_ShootTarget == ShootTarget_BIG_WHEEL)
        {
            Control_Vision_FUN.Vision_CP(0.5f);
        }
    }
    else if (DR16_Fun.GetKeyMouseAction(KEY_S, KeyAction_CLICK))
    {
        if (Robot.Attack_ShootTarget == ShootTarget_BIG_WHEEL)
        {
            Control_Vision_FUN.Vision_CP(-0.5f);
        }
    }

    if (DR16_Fun.GetKeyMouseAction(KEY_W, KeyAction_PRESS)) //w
    {
        //DR16_Export_Data.Robot_TargetValue.Forward_Back_Value += 1000;
        ChassisRamp_ForwardBack.rate = RampRate * RampRate_factor;
    }
    else if (DR16_Fun.GetKeyMouseAction(KEY_S, KeyAction_PRESS)) //S
    {
        //DR16_Export_Data.Robot_TargetValue.Forward_Back_Value -= 1000;
        ChassisRamp_ForwardBack.rate = -RampRate * RampRate_factor;
    }
    else
    {
        //DR16_Export_Data.Robot_TargetValue.Forward_Back_Value = 0;
        CountReset(&ChassisRamp_ForwardBack);
        ChassisRamp_ForwardBack.rate = 0;
    }

    /***************************A D **********************/
    if (DR16_Fun.GetKeyMouseAction(KEY_D, KeyAction_PRESS)) //D
    {
        //DR16_Export_Data.Robot_TargetValue.Left_Right_Value += 1000;
        ChassisRamp_LeftRight.rate = -RampRate * 2.4;
    }
    else if (DR16_Fun.GetKeyMouseAction(KEY_A, KeyAction_PRESS)) //A
    {
        //DR16_Export_Data.Robot_TargetValue.Left_Right_Value -= 1000;
        ChassisRamp_LeftRight.rate = RampRate * 2.4;
    }
    else
    {
        //DR16_Export_Data.Robot_TargetValue.Left_Right_Value = 0;
        CountReset(&ChassisRamp_LeftRight);
        ChassisRamp_LeftRight.rate = 0;
    }

    DR16_Export_Data.Robot_TargetValue.Forward_Back_Value = 1 * Forward_Back_Value_Direction * SpeedRampCalc(&ChassisRamp_ForwardBack);
    DR16_Export_Data.Robot_TargetValue.Left_Right_Value = -1 * Left_Right_Value_Direction * SpeedRampCalc(&ChassisRamp_LeftRight);

    if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_W, KeyAction_CLICK))
    {
        DR16_Export_Data.Robot_TargetValue.Forward_Back_Value = Forward_Back_Value_Direction * 100;
    }
    if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_S, KeyAction_CLICK))
    {
        DR16_Export_Data.Robot_TargetValue.Forward_Back_Value = Forward_Back_Value_Direction * -100;
    }
    if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_A, KeyAction_CLICK))
    {
        DR16_Export_Data.Robot_TargetValue.Left_Right_Value = Left_Right_Value_Direction * 100;
    }
    if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_D, KeyAction_CLICK))
    {
        DR16_Export_Data.Robot_TargetValue.Left_Right_Value = Left_Right_Value_Direction * -100;
    }

    if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_F, KeyAction_CLICK)) //转头180度，撤退
    {
        if (Cloud.IMUtargetYawRaw > 0) //往0 的方向減去，减少溢出
        {
            Cloud.IMUtargetYawRaw -= 180;
        }
        else
        {
            Cloud.IMUtargetYawRaw += 180;
        }
    }

    if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_Z, KeyAction_CLICK)) //自旋模式
    {
        if (Robot.Sport_ChassisWorkMode != ChassisWorkMode_Spin)
        {
            Robot_control_FUN.Robot_setChassisWorkMode(ChassisWorkMode_Spin);
            if (SpinDirection_Flag != 1)
            {
                SpinDirection_Flag = 1;
            }
            else
            {
                SpinDirection_Flag = -1;
            }
        }
        else
        {
            if (ReversefollowFlag.NewFollowFlag == 1)
            {
                Robot.Sport_ChassisWorkMode = ChassisWorkMode_NewFollow;
            }
            else
            {
                Robot_control_FUN.Robot_setChassisWorkMode(ChassisWorkMode_Follow);
            }
        }
        if (Robot.Attack_ShootTarget == ShootTarget_BIG_WHEEL)
        {
            Robot.Attack_ShootTarget = ShootTarget_Self_aiming;
        }
    }

    if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_X, KeyAction_CLICK)) //扭腰模式
    {
        if (Robot.Sport_ChassisWorkMode != ChassisWorkMode_Twister)
        {
            Robot_control_FUN.Robot_setChassisWorkMode(ChassisWorkMode_Twister);
        }
        else
        {
            if (ReversefollowFlag.NewFollowFlag == 1)
            {
                Robot.Sport_ChassisWorkMode = ChassisWorkMode_NewFollow;
            }
            else
            {
                Robot_control_FUN.Robot_setChassisWorkMode(ChassisWorkMode_Follow);
            }
        }

        if (Robot.Attack_ShootTarget == ShootTarget_BIG_WHEEL)
        {
            Robot.Attack_ShootTarget = ShootTarget_Self_aiming;
        }
    }

    /************** 运动控制end ******************/

    /************** 特殊功能模式begin ****************/
    if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_C, KeyAction_CLICK)) //转头180度，撤退
    {
        if (Cloud.IMUtargetYawRaw > 0) //往0 的方向減去，减少溢出
        {
            Cloud.IMUtargetYawRaw -= 180;
        }
        else
        {
            Cloud.IMUtargetYawRaw += 180;
        }

        if (Robot.Sport_ChassisWorkMode == ChassisWorkMode_Reversefollow)
        {
            Robot.Sport_ChassisWorkMode = ChassisWorkMode_Follow;
            ReversefollowFlag.transition = 1;
        }
        else if (Robot.Sport_ChassisWorkMode == ChassisWorkMode_Follow)
        {
            ReversefollowFlag.transition = 1;
            Robot.Sport_ChassisWorkMode = ChassisWorkMode_Reversefollow;
        }
        else
        {
            if (Robot.Sport_ChassisWorkMode != ChassisWorkMode_Reversefollow)
            {
                Robot.Sport_ChassisWorkMode = ChassisWorkMode_Reversefollow;
            }
            else
            {
                if (ReversefollowFlag.NewFollowFlag == 1)
                {
                    Robot.Sport_ChassisWorkMode = ChassisWorkMode_NewFollow;
                }
                else
                {
                    Robot.Sport_ChassisWorkMode = ChassisWorkMode_Follow;
                }
            }
            ReversefollowFlag.transition = 0;
        }

        if (Robot.Attack_ShootTarget == ShootTarget_BIG_WHEEL)
        {
            Robot.Attack_ShootTarget = ShootTarget_Self_aiming;
        }
    }

    if (DR16_Fun.GetKeyMouseAction(KEY_R, KeyAction_CLICK)) //开关弹仓
    {
        if (Shoot_Fun.Cartridge_openFlag != true)
        {
            Shoot_Fun.Cartridge_openFlag = Shoot_Fun.Shoot_openCartridge(true);
        }
        else
        {
            Shoot_Fun.Cartridge_openFlag = Shoot_Fun.Shoot_openCartridge(false);
        }
    }

    if (DR16_Fun.GetKeyMouseAction(KEY_SHIFT, KeyAction_PRESS))
    {
        SuperCapacitorFUN.SuperCapacitor_Switch(true);
    }

    if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_Q, KeyAction_PRESS)) //爆发模式
    {
        Robot.OpenBurstOut = OpenBurstOut_Start; //无视热量限制
    }

    //两种跟随模式切换（提醒操作手不到万不得已时不要使用）
    if (DR16_Fun.GetKeyMouseAction(KEY_Z, KeyAction_CLICK) && DR16_Fun.GetKeyMouseAction(KEY_X, KeyAction_CLICK) && DR16_Fun.GetKeyMouseAction(KEY_C, KeyAction_CLICK))
    {
        if (Robot.Sport_ChassisWorkMode != ChassisWorkMode_NewFollow)
        {
            Robot.Sport_ChassisWorkMode = ChassisWorkMode_NewFollow;
            ReversefollowFlag.NewFollowFlag = 1;
        }
        else
        {
            Robot.Sport_ChassisWorkMode = ChassisWorkMode_Follow;
            ReversefollowFlag.NewFollowFlag = 0;
        }
    }
    /************** 特殊功能模式end ******************/

    /************************** 鼠标移动 **********************/
    if (Robot.Sport_ChassisWorkMode == ChassisWorkMode_NewFollow)
    {
        DR16_Export_Data.mouse.x *= 5000;
        Filter_IIRLPF(&DR16_Export_Data.mouse.x, &DR16_Export_Data.Robot_TargetValue.Omega_Value, Mouse_LpfAttFactor); //陀螺仪离线时云台跟随底盘
    }
    else
    {
        Filter_IIRLPF(&DR16_Export_Data.mouse.x, &Yaw_Value, Mouse_LpfAttFactor); //陀螺仪在线时底盘跟随云台
        DR16_Export_Data.Robot_TargetValue.Yaw_Value = Yaw_Value;
        DR16_Export_Data.Robot_TargetValue.Omega_Value = 9900.0f;
    }
    Filter_IIRLPF(&DR16_Export_Data.mouse.y, &Pitch_Value, Mouse_LpfAttFactor);
    DR16_Export_Data.Robot_TargetValue.Pitch_Value = Pitch_Value;

    if (Robot.Sport_ChassisWorkMode == ChassisWorkMode_Reversefollow)
    {
        Forward_Back_Value_Direction = -1;
        Left_Right_Value_Direction = -1;
    }
    else
    {
        Forward_Back_Value_Direction = 1;
        Left_Right_Value_Direction = 1;
    }

    /************************** 鼠标移动END **********************/

    /************** 射击控制begin ****************/

    if (DR16_Fun.GetKeyMouseAction(MOUSE_Left, KeyAction_CLICK))
    {
        Robot.Attack_AttackMode = AttackMode_SingleMode;
    }

    if (DR16_Fun.GetKeyMouseAction(MOUSE_Left, KeyAction_LONG_PRESS))
    {
        Robot.Attack_AttackMode = AttackMode_ContinuousMode;
    }

    // if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(MOUSE_Left, KeyAction_PRESS))
    // {
    //     Robot.Attack_AttackMode = AttackMode_AutoMode;
    // }

    if (DR16_Fun.GetKeyMouseAction(MOUSE_Right, KeyAction_LONG_PRESS))
    {
        Robot.VisionEnabled = true;
    }

    if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_G, KeyAction_CLICK))
    {
        if (Robot.Attack_ShootTarget != ShootTarget_Sentry)
        {
            Robot.Attack_ShootTarget = ShootTarget_Sentry;
        }
        else
        {
            Robot.Attack_ShootTarget = ShootTarget_Self_aiming;
        }
    }

    if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_B, KeyAction_CLICK))
    {
        if (Robot.Attack_ShootTarget != ShootTarget_Video)
        {
            Robot.Attack_ShootTarget = ShootTarget_Video;
        }
        else
        {
            Robot.Attack_ShootTarget = ShootTarget_Self_aiming;
        }
    }

    if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_V, KeyAction_CLICK))
    {
        if (Robot.Attack_ShootTarget != ShootTarget_BIG_WHEEL)
        {
            Robot.Attack_ShootTarget = ShootTarget_BIG_WHEEL;
            Robot.Sport_ChassisWorkMode = ChassisWorkMode_Lock;
            // BIGWHEEL_EndRunFlag.Lsat_Angle = Cloud.IMUtargetYawRaw;
        }
        else
        {
            Robot.Attack_ShootTarget = ShootTarget_Self_aiming;
            BIGWHEEL_EndRunFlag.BIG_WHEELEndFlag = 1;

            Cloud.IMUtargetYawRaw -= 180.0f;
        }
    }

    if (DR16_Fun.GetKeyMouseAction(KEY_E, KeyAction_CLICK))
    {
        Robot.Device_FrictMode = FrictWorkMode_AutoSpeed;
    }

    if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_PRESS) && DR16_Fun.GetKeyMouseAction(KEY_E, KeyAction_CLICK))
    {
        Robot.Device_FrictMode = FrictWorkMode_Disable;
    }
    /************** 射击控制end ******************/

    if (DR16_Fun.GetKeyMouseAction(KEY_CTRL, KeyAction_CLICK))
    {
        InitShow_Flag++; // --- 初始化UI界面用
    }

    //dr16_data.Remote_TargetValue.Forward_Back_Value = -forward_back_speed
    //	* SpeedRampCalc(&ChassisFB_DELL_Ramp);
    //dr16_data.Remote_TargetValue.Left_Right_Value = -left_right_speed
    //	* SpeedRampCalc(&ChassisLR_DELL_Ramp);

    //dr16_data.Remote_TargetValue.Turn_Left_Right_Value = RC_CtrlData.mouse.x * CLOUD_MOUSE_DB;

    //dr16_data.Remote_TargetValue.Yaw_Value = RC_CtrlData.mouse.x * CLOUD_MOUSE_DB;
    //dr16_data.Remote_TargetValue.Pitch_Value = -RC_CtrlData.mouse.y * CLOUD_MOUSE_DB;
}

/**
 * @brief 更新遥控器接收的控制数据
 * 
 * @return  
 */
void RemoteControl_Update(void)
{
    RemoteMode_Update();
    DR16_Fun.RemoteControl_Output();
    switch (Robot.ControlSource)
    {
    case ControlSource_RC: //遥控器控制模式开始。
        Robot_control_FUN.Robot_Enable();
        if (Robot.WorkStatus == WorkStatus_Normal) //机器人开机初始化完毕才进入处理。
        {
            RemoteControl_RC_Update();
        }

        break;
    case ControlSource_PC: //电脑控制模式开始。
        Robot_control_FUN.Robot_Enable();
        if (Robot.WorkStatus == WorkStatus_Normal)
        {

            Robot.Attack_AttackMode = AttackMode_ManualMode;
            Robot.OpenBurstOut = OpenBurstOut_End;          //攻击爆发模式关闭
            Robot.VisionEnabled = false;                    //关闭自瞄
            SuperCapacitorFUN.SuperCapacitor_Switch(false); //超电关闭
            RemoteControl_PC_Update();                      //键鼠控制模式
            if (ext_game_robot_state.data.mains_power_shooter_output == 0)
            {
                Robot.Attack_AttackMode = AttackMode_Disable;
                LASER_Set(GPIO_PIN_RESET); //关闭激光
            }
            if (Robot.Device_FrictMode != FrictWorkMode_Disable)
            {
                LASER_Set(GPIO_PIN_SET); //开启激光
            }
            else
            {
                Robot.Attack_AttackMode = AttackMode_Disable;
                LASER_Set(GPIO_PIN_RESET); //关闭激光
            }
        }
        break;

    case ControlSource_Stop:
        Robot_control_FUN.Robot_Disable();
        break;
    default:
        break;
    }
}
