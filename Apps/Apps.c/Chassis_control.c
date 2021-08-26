/**
 * @file Chassis_control.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Chassis_control.h"
#include "Cloud_control.h"
#include "Control_Vision.h"
#include "DJI_IMU.h"
#include "DR16_Remote.h"
#include "M6020_Motor.h"
#include "PowerLimit.h"
#include "RM_JudgeSystem.h"
#include "Robot_control.h"
#include "Status_Update.h"
#include "USER_Filter.h"
#include "USER_RNG.h"
#include "Wolf_GyIMU.h"

void Chassis_Init(void);
void Chassis_processing(float Vx, float Vy, float VOmega);
void ChassisCapControl(void);
Chassis_t Chassis;
GY_IMUExport_t IMUExport_Chassis;

/*****************扭腰标志位***************/
Twister_Flag_t Twister_Flag;
Twister_Control_t Twister_Control = Twister_ControlGroundInit;
#undef Twister_ControlGroundInit
/*****************************************/
ReversefollowFlag_t ReversefollowFlag;
BIGWHEEL_EndRunFlag_t BIGWHEEL_EndRunFlag;

positionpid_t FollowYawAttitude_pid = FollowYawAttitude_pidInit; //底盘Yaw轴跟随云台pid
#undef FollowYawAttitude_pidInit
positionpid_t FollowYawSpeed_pid = FollowYawSpeed_pidInit; //底盘Yaw轴跟随云台pid
#undef FollowYawSpeed_pidInit
positionpid_t FollowYaw_pid = FollowYaw_pidInit; //底盘Yaw轴跟随云台pid
#undef FollowYaw_pidInit
positionpid_t TwisterYaw_pid = TwisterYaw_pidInit; //自旋+扭腰PID
#undef TwisterYaw_pidInit
positionpid_t AutoTrace_pid = AutoTrace_pidInit; //底盘自动追踪pid
#undef AutoTrace_pidInit

incrementalpid_t LFWheel_PID = LFWHEEL_PID_PARAM;
#undef LFWHEEL_PID_PARAM
incrementalpid_t RFWheel_PID = RFWHEEL_PID_PARAM;
#undef RFWHEEL_PID_PARAM
incrementalpid_t LBWheel_PID = LBWHEEL_PID_PARAM;
#undef LBWHEEL_PID_PARAM
incrementalpid_t RBWheel_PID = RBWHEEL_PID_PARAM;
#undef RBWHEEL_PID_PARAM
incrementalpid_t *Wheel_PID[] = {&LFWheel_PID, &RFWheel_PID, &LBWheel_PID, &RBWheel_PID};
Chassis_FUN_t Chassis_FUN = Chassis_FUNGroundInit;
#undef Chassis_FUNGroundInit

SpeedRamp_t SpinSpeedRamp = SpinSpeedRampInit;
#undef SpinSpeedRampInit

/**
 * @brief  底盘初始化，配置参数
 * @param  None
 * @retval None
 */
void Chassis_Init(void)
{
    Chassis.LpfAttFactor = Chassis_LpfAttFactor;
}

/**
 * @brief 根据功率调整直走速度
 * 
 * @param Limit_factor 
 * @return  
 */
uint8_t USERCapacitor_Flag;
static void XYSpeedLimit(float *Limit_factor, uint16_t *OLimit_factor)
{
    if (ext_game_robot_state.data.chassis_power_limit <= 50)
    {
        *Limit_factor = Chassis_SpeedNormal;
        *OLimit_factor = Get_RandomNumbers_Range(Chassis_SpeedNormal - 50, Chassis_SpeedNormal + 200);
    }
    else if (ext_game_robot_state.data.chassis_power_limit > 50 && ext_game_robot_state.data.chassis_power_limit <= 60)
    {
        *Limit_factor = Chassis_SpeedHigh_Normal;
        *OLimit_factor = Get_RandomNumbers_Range(Chassis_SpeedHigh_Normal - 50, Chassis_SpeedHigh_Normal + 200);
    }
    else if (ext_game_robot_state.data.chassis_power_limit > 60 && ext_game_robot_state.data.chassis_power_limit <= 80)
    {
        *Limit_factor = Chassis_SpeedMid;
        *OLimit_factor = Get_RandomNumbers_Range(Chassis_SpeedMid - 200, Chassis_SpeedMid + 800);
    }
    else if (ext_game_robot_state.data.chassis_power_limit > 80 && ext_game_robot_state.data.chassis_power_limit <= 800)
    {
        *Limit_factor = Chassis_SpeedFast;
        *OLimit_factor = Get_RandomNumbers_Range(Chassis_SpeedFast - 200, Chassis_SpeedFast + 800);
    }

    if (supercapacitor.ReceiveData.cap_cell > 65 && Robot.SuperCapacitorMode == SuperCapacitor_Open)
    {
        *Limit_factor = Chassis_SuperFast;
        *OLimit_factor = Get_RandomNumbers_Range(Chassis_SuperFast - 200, Chassis_SuperFast + 500);
    }
    else
    {
        *Limit_factor = *Limit_factor;
        *OLimit_factor = *OLimit_factor;
        supercapacitor.EnableCapacitor = Charging_OFF;
    }

    if (USERCapacitor_Flag == 1)
    {
        *Limit_factor = Chassis_SuperFast;
        *OLimit_factor = *OLimit_factor;
    }

    // if (Wolf_GyIMU_Fun.GY6050_PitErr() > -20 && Wolf_GyIMU_Fun.GY6050_PitErr() < -12)
    // {
    //     *Limit_factor *= 0.65f;
    // }
}

/**
 * @brief  麦克纳姆轮速度解算
 * @param[in]  Vx		x轴速度
 *				Vy		y轴速度
 *				VOmega	自转速度
 * @param[out]	Speed	速度
 * @retval None
 */
static void MecanumCalculate(float Vx, float Vy, float VOmega, int16_t *Speed)
{
    float tempSpeed[4];
    float MaxSpeed = 0.0f;
    float Param = 1.0f;

    //四轮速度分解
    tempSpeed[0] = Vx - Vy + VOmega;
    tempSpeed[1] = Vx + Vy + VOmega;
    tempSpeed[2] = -Vx + Vy + VOmega;
    tempSpeed[3] = -Vx - Vy + VOmega;

    //寻找最大速度
    for (uint8_t i = 0; i < 4; i++)
    {
        if (abs(tempSpeed[i]) > MaxSpeed)
        {
            MaxSpeed = abs(tempSpeed[i]);
        }
    }

    //速度分配
    if (MaxSpeed > WheelMaxSpeed)
    {
        Param = (float)WheelMaxSpeed / MaxSpeed;
    }

    Speed[0] = tempSpeed[0] * Param;
    Speed[1] = tempSpeed[1] * Param;
    Speed[2] = tempSpeed[2] * Param;
    Speed[3] = tempSpeed[3] * Param;
}

/**
 * @brief 过零处理
 * 
 * @param target 
 * @param value 
 * @return int 
 */
static int ComputeMinOffset(int target, int value) //计算最小偏差，底盘跟随应该往哪个方向去完成跟随动作。
{
    int err = target - value;

    if (err > 4096)
    {
        err -= 8191;
    }
    else if (err < -4096)
    {
        err += 8191;
    }
    return err;
}

/**
 * @brief  底盘扭腰处理
 * @param None
 * @retval None
 */

#define TwisterFlagStart 0
#define TwisterFlagLeft 1
#define TwisterFlagRight 2

static float Twister_processing(void)
{
    float angle;
    Twister_Control.MechAngle = ComputeMinOffset(M6020s_Yaw.realAngle, Cloud_Yaw_Center);
    angle = Twister_Control.MechAngle / (float)M6020_mAngleRatio;
    angle = angle * (PI / 180);

    switch (Twister_Control.TwisterFlag)
    {

    case TwisterFlagStart:
        Twister_Control.TwisterAngle = 0.0f;
        Twister_Control.IfCorgiChange = false;
        Twister_Control.TwisterFlag = TwisterFlagLeft;
        break;

    case TwisterFlagLeft:
        Twister_Control.TwisterAngle = -1.1f;
        Twister_Control.IfCorgiChange = false;

        if (Twister_Control.MechAngle <= -1000)
        {
            Twister_Control.TwisterFlag = TwisterFlagRight;
            Twister_Control.IfCorgiChange = true;
        }
        break;

    case TwisterFlagRight:
        Twister_Control.TwisterAngle = 1.1f;
        Twister_Control.IfCorgiChange = false;

        if (Twister_Control.MechAngle >= 1000)
        {
            Twister_Control.TwisterFlag = TwisterFlagLeft;
            Twister_Control.IfCorgiChange = true;
        }
        break;
    }

    return angle;
}

/**
 * @brief  辅助模式处理
 * @param[in]  Vx		x轴速度
 *				Vy		y轴速度
 *				Omega	偏向角
 * @retval None 
 */
static void AttachWorkMode(float *Vx, float *Vy, float *VOmega)
{
    switch (Robot.Sport_AttachWorkMode)
    {
    case AttachWorkMode_Disable:
        if (Robot.Sport_ChassisWorkMode == ChassisWorkMode_Spin || Robot.Sport_ChassisWorkMode == ChassisWorkMode_Twister)
        {
            *Vx = *Vx / 5.0f;
            *Vy = *Vy / 5.0f;
        }
        break;
    case AttachWorkMode_Slow:
        *Vx = *Vx / 5.0f;
        *Vy = *Vy / 5.0f;
        *VOmega = *VOmega / 2.5f;
        break;
    }
}

/**
 * @brief 全向公式
 * 
 * @param Vx 
 * @param Vy 
 * @return  
 */
static void Omnidirectional_Formula(float *Vx, float *Vy)
{
    float RadRaw = 0.0f;
    float temp_Vx = 0.0f;

    float angle = Cloud_FUN.Cloud_getYawAngleWithCenter(); //机械角度偏差
    RadRaw = angle * DEG_TO_RAD;                           //弧度偏差
    /*Chassis_Vx = Chassis_Vx * cos(RadRaw) - Chassis_Vy * sin(RadRaw);
		Chassis_Vy = Chassis_Vy * cos(RadRaw) + temp_Vx * sin(RadRaw);*/
    //全向移动公式。
    temp_Vx = *Vx;
    *Vx = *Vx * cos(RadRaw) - *Vy * sin(RadRaw);
    *Vy = *Vy * cos(RadRaw) + temp_Vx * sin(RadRaw);
}

/**
 * @brief 跟随
 * 
 * @param Vx 
 * @param Vy 
 * @param VOmega 
 * @param Yaw_Center 
 * @return  
 */
static void Follow(float *Vx, float *Vy, float *VOmega, int Yaw_Center)
{

    float Angle = ComputeMinOffset(Yaw_Center, M6020s_Yaw.realAngle);
    float angle = Angle / (float)M6020_mAngleRatio;
    /*****************双环****************/
    // FollowYawAttitude_pid.Position_PID(&FollowYawAttitude_pid, angle, 0.0f);
    // FollowYawSpeed_pid.Position_PID(&FollowYawSpeed_pid, FollowYawAttitude_pid.pwm, imu_Export.SpeedLPF[0]);
    /*****************双环 END****************/

    /*****************分段PID****************/
    //这是一个特殊的PID，target是误差值，measured是直走速度值,Integral_Separation是功率上限值
    if (ext_game_robot_state.data.chassis_power_limit == 60)
    {
        FollowYaw_pid.Integral_Separation = 0.95f;
    }
    else if (ext_game_robot_state.data.chassis_power_limit < 60)
    {
        FollowYaw_pid.Integral_Separation = 0.9f;
    }
    else
    {
        FollowYaw_pid.Integral_Separation = 1.0f;
    }
    FollowYaw_pid.Position_PID(&FollowYaw_pid, angle, *Vy);
    /*****************分段PID END****************/

    if (Robot.Sport_ChassisWorkMode == ChassisWorkMode_NewFollow) //使在陀螺仪丢失时底盘不卡
    {
        *VOmega = *VOmega /* + Chassis.FollowYawSpeed_pid.pwm*/;
    }
    else
    {
        // *VOmega = FollowYawSpeed_pid.pwm;
        *VOmega = FollowYaw_pid.pwm;
    }

    //转弯时压低直走平移速度
    Chassis.SpeedLimit_Factor = (8191.0f - fabs(Angle)) / 8191.0f;
    Chassis.SpeedLimit_Factor = pow(Chassis.SpeedLimit_Factor, 4);
    *Vx *= Chassis.SpeedLimit_Factor;
    *Vy *= Chassis.SpeedLimit_Factor;
}

/**
 * @brief 反向跟随标志位计时清零
 * 
 * @param transition 
 * @return  
 */
void Clear_ReversefollowTransition(int *transition)
{
    ReversefollowFlag.ReversefollowTime++;
    if (ReversefollowFlag.ReversefollowTime > 500) //1秒后清零
    {
        *transition = 0;
    }
}

/**
 * @brief 反向跟随时间标志位清零
 * 
 * @param transition 
 * @return  
 */
void Clear_ReversefollowTime(void)
{
    ReversefollowFlag.ReversefollowTime = 0;
}

/**
 * @brief 神符结束标志位计时清零
 * 
 * @param transition 
 * @return  
 */
void Clear_BIG_WHEELEndTime(void)
{
    BIGWHEEL_EndRunFlag.BIG_WHEELEndTime = 0;
}

/**
 * @brief 神符结束标志位计时清零
 * 
 * @param transition 
 * @return  
 */
void Clear_BIG_WHEELEndFlag(int *transition)
{
    BIGWHEEL_EndRunFlag.BIG_WHEELEndTime++;
    if (BIGWHEEL_EndRunFlag.BIG_WHEELEndTime > 400) //1秒后清零
    {
        *transition = 0;
    }
    if (*transition != 1)
    {
        Clear_BIG_WHEELEndTime();
        if (ReversefollowFlag.NewFollowFlag == 1)
        {
            Robot.Sport_ChassisWorkMode = ChassisWorkMode_NewFollow;
        }
        else
        {
            Robot.Sport_ChassisWorkMode = ChassisWorkMode_Follow;
        }
    }
}

/**
 * @brief  底盘控制处理-跟随云台
 * @param[in]  Vx		x轴速度
 *				Vy		y轴速度
 *				Omega	偏向角
*				mode	模式 - 除Status_ControlOFF外，其他正常控制
 * @retval None
 */
float test_VOmega = 3000;
uint16_t OmegaLimitRaw;
uint16_t RNG_Time;
void Chassis_processing(float Vx, float Vy, float VOmega)
{
    if (Robot.Sport_ChassisWorkMode == ChassisWorkMode_Disable)
    {
        for (int i = 0; i < 4; i++)
        {
            M3508s[i].outCurrent = 0;
            Clear_IncrementalPIDData(Wheel_PID[i]);
        }
        return;
    }

    AttachWorkMode(&Vx, &Vy, &VOmega);

    XYSpeedLimit(&Chassis.speedLimit, &OmegaLimitRaw);
    if (RNG_Time > 100)
    {
        Chassis.OmegaLimit = OmegaLimitRaw;
        RNG_Time = 0;
    }
    RNG_Time++;

    //速度限制
    VAL_LIMIT(Vx, -Chassis.speedLimit * 0.7f, Chassis.speedLimit * 0.7f);
    VAL_LIMIT(Vy, -Chassis.speedLimit, Chassis.speedLimit);

    //云台跟随底盘时底盘运动模式删除
    if (ReversefollowFlag.NewFollowFlag == 1)
    {
        if (Robot.Sport_ChassisWorkMode == ChassisWorkMode_Follow)
        {
            Robot.Sport_ChassisWorkMode = ChassisWorkMode_Follow;
        }
        else if (Robot.Sport_ChassisWorkMode == ChassisWorkMode_Lock)
        {
            Robot.Sport_ChassisWorkMode = ChassisWorkMode_Lock;
        }
        else
        {
            Robot.Sport_ChassisWorkMode = ChassisWorkMode_NewFollow;
        }
    }

#ifdef Enable_CloudControl //为了紧急测试时不需要云台的情况。

    if (Robot.Sport_ChassisWorkMode != ChassisWorkMode_Spin)
    {
        Chassis.spinSpeed = 0;
    }

    switch (Robot.Sport_ChassisWorkMode)
    {
    case ChassisWorkMode_Spin:
    {
        Omnidirectional_Formula(&Vx, &Vy);

        //限制自旋速度
        VAL_LIMIT(VOmega, -Chassis.OmegaLimit, Chassis.OmegaLimit);
        VOmega *= SpinDirection_Flag;
        Chassis.spinSpeed = RAMP_float(VOmega, Chassis.spinSpeed, SpinSpeedRamp.rate);
        VOmega = Chassis.spinSpeed;
        if (Robot.ControlSource == ControlSource_RC)
        {
            VOmega = DR16_Export_Data.Robot_TargetValue.Dial_Wheel * 13.0f * SpinDirection_Flag;
            //觉得应该再对VOmega 进行处理使得小陀螺移动时，自旋速度不减。
        }
        break;
    }
    case ChassisWorkMode_Twister:
    {
        float angle = Twister_processing();
        VOmega = TwisterYaw_pid.Position_PID(&TwisterYaw_pid, Twister_Control.TwisterAngle, angle);

        //限制扭腰速度
        // VAL_LIMIT(VOmega, -Chassis.OmegaLimit, Chassis.OmegaLimit);
        VAL_LIMIT(VOmega, -Chassis.OmegaLimit * 0.75, Chassis.OmegaLimit * 0.75);
        break;
    }
    case ChassisWorkMode_Follow:
    {
        //跟随模式。
        /*Chassis.FollowYawRaw = 0.0f;*/
        /*****辅助模式：反向跟随过渡操作******/
        if (ReversefollowFlag.transition == 1)
        {
            Omnidirectional_Formula(&Vx, &Vy);
            VOmega = 0.0f;

            if (abs(M6020s_Yaw.realAngle - Cloud_Yaw_Center) <= 400)
            {
                ReversefollowFlag.transition = 0;
            }
            Clear_ReversefollowTransition(&ReversefollowFlag.transition);
        }
        /*****反向跟随过渡操作结束******/
        else
        {
            Follow(&Vx, &Vy, &VOmega, Cloud_Yaw_Center);
            Clear_ReversefollowTime();
        }
        break;
    }
    case ChassisWorkMode_Reversefollow:
    {
        //反向跟随模式。
        /*Chassis.FollowYawRaw = 0.0f;*/
        /*****辅助模式：反向跟随过渡操作******/
        if (ReversefollowFlag.transition == 1)
        {
            Omnidirectional_Formula(&Vx, &Vy);
            VOmega = 0.0f;

            if (abs(M6020s_Yaw.realAngle - Cloud_Yaw_ReverseCenter) <= 400)
            {
                ReversefollowFlag.transition = 0;
            }
            Clear_ReversefollowTransition(&ReversefollowFlag.transition);
        }
        /*****反向跟随过渡操作结束******/
        else
        {
            Follow(&Vx, &Vy, &VOmega, Cloud_Yaw_ReverseCenter);
            Clear_ReversefollowTime();
        }
        break;
    }
    case ChassisWorkMode_AutoTrace:
    {
        if (VisionExportData.FinalOffset_depth < 1000.0f)
        {
            VisionExportData.FinalOffset_depth = 1000.0f;
        }
        Chassis.Trace_Distance = AutoTrace_pid.Position_PID(&AutoTrace_pid, 1000.0f, VisionExportData.FinalOffset_depth);
        Vy += Chassis.Trace_Distance;

        Follow(&Vx, &Vy, &VOmega, Cloud_Yaw_Center);
        break;
    }
    case ChassisWorkMode_Lock:
    {
        Vx = Vy = VOmega = 0.0f;
        if (BIGWHEEL_EndRunFlag.BIG_WHEELEndFlag == 1)
        {
            Clear_BIG_WHEELEndFlag(&BIGWHEEL_EndRunFlag.BIG_WHEELEndFlag);
        }
        break;
    }
    }

#endif
    int16_t speed[4];

    //平滑处理,避免瞬间加速对地板产生较大的摩擦。
    Filter_IIRLPF(&Vx, &Chassis.targetXLPF, Chassis.LpfAttFactor);
    Filter_IIRLPF(&Vy, &Chassis.targetYLPF, Chassis.LpfAttFactor);
    Filter_IIRLPF(&VOmega, &Chassis.targetZLPF, Chassis.LpfAttFactor);

    //麦轮解算
    MecanumCalculate(Chassis.targetXLPF, Chassis.targetYLPF, Chassis.targetZLPF, speed);

    for (int i = 0; i < 4; i++)
    {
        if (M3508s[i].OffLineFlag == 0)
        { //避免在赋值多次而电机才读取了一次数据的情况。

            M3508s[i].targetSpeed = speed[i];
            speed[i] = Wheel_PID[i]->Incremental_PID(Wheel_PID[i], M3508s[i].targetSpeed, M3508s[i].realSpeed);
        }
    }

#ifdef Enable_RMJudgeSystem
    PowerLimit(&chassis_powerLimit, speed, 4);
#endif

    for (int i = 0; i < 4; i++)
    {
        M3508s[i].outCurrent = speed[i];
    }

    //发送电机电流值，改为在外部统一发送报文。
    //M3508_setCurrent(M3508s[0].outCurrent, M3508s[1].outCurrent, M3508s[2].outCurrent, M3508s[3].outCurrent);
}

/**
	* @brief  底盘超级电容控制函数
  * @param	void
  * @retval None
  */
float Charging_Power;
void ChassisCapControl(void)
{
    uint16_t chassis_power_limit = 0;
    if (supercapacitor.OffLineFlag == 1)
    {
        supercapacitor.EnableCapacitor = Charging_OFF;
        USERCapacitor_Flag = 1;
        return;
    }

    if (ext_game_robot_state.data.chassis_power_limit <= 50)
    {
        chassis_power_limit = 50;
    }
    else if (ext_game_robot_state.data.chassis_power_limit >= 120)
    {
        chassis_power_limit = 120;
    }
    else
    {
        chassis_power_limit = ext_game_robot_state.data.chassis_power_limit;
    }

    if (supercapacitor.EnableCapacitor == Charging_ON && USERCapacitor_Flag == 0)
    {
        if (supercapacitor.ReceiveData.cap_cell < 65)
        {
            // Charging_Power = chassis_power_limit - ext_power_heat_data.data.chassis_power;
            // SuperCapacitorFUN.SCCM_SendMsg(Charging_Power, Charging_ON, Power_NotSupply); //--- 充电功率  充电是否使能  电容是否供电
            USERCapacitor_Flag = 1;
        }
        Robot.SuperCapacitorMode = SuperCapacitor_Open;
        Charging_Power = chassis_power_limit;
        SuperCapacitorFUN.SCCM_SendMsg(Charging_Power, Charging_ON, Power_Supply); //--- 充电功率  充电是否使能  电容是否供电
    }
    else if (supercapacitor.EnableCapacitor == Charging_OFF || USERCapacitor_Flag == 1)
    {
        if (supercapacitor.ReceiveData.cap_cell > 80)
        {
            USERCapacitor_Flag = 0;
        }

        Robot.SuperCapacitorMode = SuperCapacitor_Off;
        Charging_Power = chassis_power_limit - ext_power_heat_data.data.chassis_power;
        SuperCapacitorFUN.SCCM_SendMsg(Charging_Power, Charging_ON, Power_NotSupply); //--- 充电功率  充电是否使能  电容是否供电
    }
}
