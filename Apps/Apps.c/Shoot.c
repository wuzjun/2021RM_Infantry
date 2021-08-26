/**
 * @file Shoot.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Shoot.h"
#include "BSP_TIM.h"
#include "Control_Vision.h"
#include "DR16_Remote.h"
#include "FreeRTOS.h"
#include "M3508_Motor.h"
#include "M6020_Motor.h"
#include "RM_JudgeSystem.h"
#include "Robot_Config.h"
#include "Robot_control.h"
#include "Snail_Motor.h"
#include "cmsis_os.h"

#define limitTime 50 //单次卡弹时间上限

int16_t ObstructTorque = 7000;
int16_t ObstructSpeed = 7000; //卡弹的速度，实际在800之间抖动。

ShootUnit_t shootUnit1 = {0};
WorldTime_RxTypedef shootUnit_WorldTime;

#ifdef Enable_shootUnit2
ShootUnit_t shootUnit2 = {0};
WorldTime_RxTypedef shootUnit2_WorldTime;
#endif

/*************************摩擦轮/拨盘PID初始化************************/
incrementalpid_t M3508_PowerLPID = M3508_PowerLPIDInit;
#undef M3508_PowerLPIDInit
incrementalpid_t M3508_PowerRPID = M3508_PowerRPIDInit;
#undef M3508_PowerRPIDInit
positionpid_t M2006s_YawOPID = M2006s_YawOPIDInit;
positionpid_t M2006s_YawIPID = M2006s_YawIPIDInit;
/*********************************************************************/
void Shoot_Init(void);
void Shoot_processing(void);
bool Shoot_openCartridge(bool open);
void Shoot_setNeedLaunchCount(ShootUnit_t *shootUnit, uint16_t amount);
void FricRead(void);
void FricWrite(void);
Shoot_Fun_t Shoot_Fun = Shoot_FunGroundInit;

//------- 不需要被外部调用的函数声明。
void Fric_processing(void);
void PillReload_processing(void);
void ResetPill(ShootUnit_t *shootUnit);
void Shoot_ReloaderCal(ShootUnit_t *shootUnit);
bool Shoot_MotorInObstruct(ShootUnit_t *shootUnit);
static void AutoAttack_process(ShootUnit_t *shootUnit);
static void Manual_Attack_process(ShootUnit_t *shootUnit, uint16_t speed, uint16_t *amount);
static void SingleAttack_process(ShootUnit_t *shootUnit);
static double Shoot_getPresentVo(ShootUnit_t *shootUnit);
static void Shoot_setFricOutput(int16_t ouput1, int16_t ouput2);
static int Shoot_getMaxShootAmount(void);
static void CountLaunchBullet(void);
static void ConAttack_process(ShootUnit_t *shootUnit, uint16_t time);
static void Detection_FricSpeed(uint32_t *FricOffset);
/**********************************************************/

Shoot_JudgeData_t Shoot_Judge;

int need_shoottime = 0;
uint32_t auto_shoottime = 0;
int needShoot = 0;                     //需要射击的颗数
uint32_t FricOffset[FricOffNum] = {0}; //摩擦轮速度偏移量

void Shoot_Init(void)
{

    shootUnit1.shootInterval = 0; //为0不启用
    shootUnit1.Reloader.alreadyPush = 0;
    shootUnit1.Reloader_Motor = &M2006_Reload; //绑定电机
    shootUnit1.Reloader.totalAngle_start = M2006_Reload.totalAngle;
    Shoot_setFricOutput(0, 0);                //摩擦轮初始值。
    shootUnit1.Reloader.ObstructSpeed = 1300; //设置卡弹速度的临界点。

#ifdef Enable_shootUnit2
    shootUnit2.shootInterval = 0; //为0不启用
    shootUnit2.Reloader.alreadyPush = 0;
    shootUnit2.Reloader_Motor = &M2006_Reload2; //绑定电机
    shootUnit2.Reloader.totalAngle_start = M2006_Reload2.totalAngle;
    positionpid_t M2006s_YawOPID2 = M2006s_YawOPID;
#undef M2006s_YawOPIDInit
    positionpid_t M2006s_YawIPID2 = M2006s_YawIPID;
#undef M2006s_YawIPIDInit
    shootUnit2.Reloader.ObstructSpeed = 1300; //设置卡弹速度的临界点，速度小于该值开始累积时间。
#endif
}

static void ResetPill(ShootUnit_t *shootUnit)
{
    //清零处理。
    shootUnit->Reloader_Motor->totalAngle = shootUnit->Reloader_Motor->realAngle;
    shootUnit->Reloader_Motor->targetAngle = shootUnit->Reloader_Motor->totalAngle;
    shootUnit->Reloader_Motor->lastAngle = 0;
    shootUnit->Reloader_Motor->turnCount = 0;
}

void Shoot_processing(void)
{

    PillReload_processing(); //拨弹处理
    Fric_processing();       //摩擦轮处理
    AutoAttack_process(&shootUnit1);
    Manual_Attack_process(&shootUnit1, DR16_Export_Data.Robot_TargetValue.Dial_Wheel, &shootUnit1.bullet);
    SingleAttack_process(&shootUnit1);
    ConAttack_process(&shootUnit1, 30);
}

static void PillReload_processing(void)
{
    Shoot_ReloaderCal(&shootUnit1);
    CountLaunchBullet(); //计算当前发射子弹
}

//摩擦轮速度斜坡
SpeedRamp_t FricRamp = {
    0,
    0,
    0,
    660,
};

//不用到双发射时，output2 给0即可
//output 的范围：[0,  +无穷]
static void Shoot_setFricOutput(int16_t ouput1, int16_t ouput2)
{

    FricRamp.count = shootUnit1.FricOutput;

    if (shootUnit1.FricOutput > ouput1)
    {
        FricRamp.rate = -1;
        Robot.Attack_AttackMode = AttackMode_Disable;
    }
    else if (shootUnit1.FricOutput < ouput1)
    {
        FricRamp.rate = 5;
        Robot.Attack_AttackMode = AttackMode_Disable;
    }
    else if (shootUnit1.FricOutput == ouput1)
    {
        FricRamp.rate = 0;
    }

    shootUnit1.FricOutput = 1 * SpeedRampCalc(&FricRamp);

#ifdef Enable_shootUnit2
    Fric2Ramp.count = shootUnit2.FricOutput;

    if (shootUnit2.FricOutput > ouput1)
    {
        Fric2Ramp.rate = -1;
        Robot.Attack_AttackMode = AttackMode_Disable;
    }
    else if (shootUnit2.FricOutput < ouput1)
    {
        Fric2Ramp.rate = 5;
        Robot.Attack_AttackMode = AttackMode_Disable;
    }
    else if (shootUnit2.FricOutput == ouput1)
    {
        Fric2Ramp.rate = 0;
    }
#endif
}

/**
 * @brief Get the Fric Speed object
 * 
 * @return  
 */
static uint16_t GetFricSpeed(void)
{
    uint16_t FricSpeed;
#if Friction__Wheel == Snail_Wheel
    switch (ext_game_robot_state.data.shooter_id1_17mm_speed_limit)
    {
    case 15:
        FricSpeed = 135;
        break;
    case 18:
        FricSpeed = 160;
        break;
    case 22:
        FricSpeed = 200;
        break;
    case 30:
        FricSpeed = 340;
        break;
    default:
        FricSpeed = 135;
    }
#elif Friction__Wheel == M3508_Wheel
    switch (ext_game_robot_state.data.shooter_id1_17mm_speed_limit)
    {
    case 15:
        FricSpeed = 4100 - FricOffset[0];
        break;
    case 18:
        FricSpeed = 4500 - FricOffset[1];
        break;
    case 22:
        FricSpeed = 5140 - FricOffset[2];
        break;
    case 30:
        FricSpeed = 6870 - FricOffset[3];
        break;
    default:
        FricSpeed = 4100 - FricOffset[0];
    }

#endif

    return FricSpeed;
}

static void Detection_FricSpeed(uint32_t *FricOffset)
{
    if (ext_robot_hurt.data.hurt_type == 0x2)
    {
        switch (ext_game_robot_state.data.shooter_id1_17mm_speed_limit)
        {
        case 15:
            FricOffset[0] += 100;
            break;
        case 18:
            FricOffset[1] += 100;
            break;
        case 22:
            FricOffset[2] += 50;
            break;
        case 30:
            FricOffset[3] += 50;
            break;
        }
        ext_robot_hurt.data.hurt_type = 0x0;
    }
}

/**
 * @brief 射速偏移量读取Flash
 * 
 */
void FricRead(void)
{
    Shoot_ReadOffset(FricOffset, FricOffNum);
    for (int i = 0; i < FricOffNum; i++)
    {
        if (FricOffset[i] > 1000)
        {
            FricOffset[i] = 0;
        }
    }
    
    if (Calibration_Shoot())
    {
        Shoot_EraseOffset(FricOffNum);
        shootUnit1.FricUpdata = true;
    }
}

/**
 * @brief 射速偏移量写入Flash
 * 
 */
void FricWrite(void)
{
    if (shootUnit1.FricUpdata == true)
    {
        Shoot_WriteOffset(FricOffset, FricOffNum);
        shootUnit1.FricUpdata = false;
    }
}

//测试使用速度
#if Friction__Wheel == Snail_Wheel
uint16_t Fric_Test = 400;
#elif Friction__Wheel == M3508_Wheel
uint16_t Fric_Test = 4000;
#endif
static void Fric_processing(void)
{
    Detection_FricSpeed(FricOffset);
#if Friction__Wheel == M3508_Wheel

    if (Robot.Device_FrictMode == FrictWorkMode_Disable)
    {
        shootUnit1.FricOutput = 0;
    }
    else if (Robot.Device_FrictMode == FrictWorkMode_HighSpeed)
    {
        shootUnit1.FricOutput = Fric_Test;
    }
    else if (Robot.Device_FrictMode == FrictWorkMode_AutoSpeed)
    {
        shootUnit1.FricOutput = GetFricSpeed();
    }
    else if (Robot.Device_FrictMode == FrictWorkMode_LowSpeed)
    {
        shootUnit1.FricOutput = 700;
    }

    M3508_PowerL.targetSpeed = -shootUnit1.FricOutput;
    M3508_PowerR.targetSpeed = shootUnit1.FricOutput;
    //发射装置1
    M3508_PowerL.outCurrent = M3508_PowerLPID.Incremental_PID(&M3508_PowerLPID, M3508_PowerL.targetSpeed, M3508_PowerL.realSpeed);
    M3508_PowerR.outCurrent = M3508_PowerRPID.Incremental_PID(&M3508_PowerRPID, M3508_PowerR.targetSpeed, M3508_PowerR.realSpeed);

#elif Friction__Wheel == Snail_Wheel
    Snail_Speed_Fun.C615_ObtainSpeed(&TIM4CH1_CAPTURE, &Snail_PowerL);
    Snail_Speed_Fun.C615_ObtainSpeed(&TIM4CH2_CAPTURE, &Snail_PowerR);

    if (Robot.Device_FrictMode == FrictWorkMode_Disable)
    {

        shootUnit1.FricSpeed = 0;
    }
    else if (Robot.Device_FrictMode == FrictWorkMode_HighSpeed)
    {
        shootUnit1.FricSpeed = Fric_Test;
    }
    else if (Robot.Device_FrictMode == FrictWorkMode_AutoSpeed)
    {
        shootUnit1.FricOutput = GetFricSpeed();
    }
    else if (Robot.Device_FrictMode == FrictWorkMode_LowSpeed)
    {
        shootUnit1.FricSpeed = 50;
    }

    ShootUnit_t *ShootUnit2;
#ifdef Enable_shootUnit2
    ShootUnit2 = &shootUnit2;
#endif
    Shoot_setFricOutput(shootUnit1.FricSpeed, ShootUnit2->FricSpeed);

    Snail_PowerL.output = shootUnit1.FricOutput + 1000;
    Snail_PowerR.output = shootUnit1.FricOutput + 1000;

    //发射装置1
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, Snail_PowerL.output); //摩擦轮控制
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, Snail_PowerR.output);
#endif
}

static void Shoot_ReloaderCal(ShootUnit_t *shootUnit)
{ //计算拨弹装置的输出值
    M2006s_t *Motor = shootUnit->Reloader_Motor;
    if (Robot.Attack_AttackMode == AttackMode_Disable)
    {
        //以当前位置为下次启动位置，防炸
        Motor->turnCount = 0;
        Motor->targetAngle = Motor->totalAngle = Motor->realAngle;
        shootUnit->NeedLaunchCount = 0;
        shootUnit->Reloader.alreadyPush = 0;
        shootUnit->Reloader.completePush = 0;
        ResetPill(shootUnit);
        Motor->outCurrent = 0;

        return;
    }

    if (Motor->OffLineFlag)
    {
        return;
    }

    //先执行上一次被激活的卡弹任务。
    if (shootUnit->Reloader.PillObstruct_Time > limitTime)
    {
        if (Shoot_MotorInObstruct(shootUnit)) //扭矩过大。避免执行卡弹反转任务时也卡弹。
        {
            shootUnit->Reloader.PillObstruct_Time++;                                                                                    //叠加已卡弹的时间。
            if (((int)(shootUnit->Reloader.PillObstruct_Time / limitTime)) % 2 == 0 && shootUnit->Reloader.PillObstruct_Direction == 1) //卡弹时间为：200~300 则正转。且需要跳变方向时。
            {
                shootUnit->Reloader.PillObstruct_targetAngle += Angle_Pill * 4;
                shootUnit->Reloader.PillObstruct_Direction = 0; //进行正转反堵；
            }
            else if (((int)(shootUnit->Reloader.PillObstruct_Time / limitTime)) % 2 == 1 && shootUnit->Reloader.PillObstruct_Direction == 0) //卡弹时间为：100~200 则反转 ，且需要跳变方向时。
            {
                shootUnit->Reloader.PillObstruct_targetAngle -= Angle_Pill * 4;
                shootUnit->Reloader.PillObstruct_Direction = 1; //进行反转；
            }
        }

        if (abs(Motor->totalAngle - shootUnit->Reloader.PillObstruct_targetAngle) <= Angle_Pill) //已经完成卡弹反转 任务。
        {
            shootUnit->Reloader.PillObstruct = 0;
            shootUnit->Reloader.PillObstruct_targetAngle = 0;
            shootUnit->Reloader.PillObstruct_Time = 0;
        }
        else
        {
            int M2006s_PIDOut = Position_PID(&M2006s_YawOPID, shootUnit->Reloader.PillObstruct_targetAngle, Motor->totalAngle);
            //速度PID计算(内环)
            Motor->outCurrent = Position_PID(&M2006s_YawIPID, M2006s_PIDOut, Motor->realSpeed);
            return;
        }
    }

    if (shootUnit->NeedLaunchCount > 0) //正在处于有弹丸输出需求状态中。
    {
        if (shootUnit->Reloader.alreadyPush == 0) //新任务还没开始执行
        {
            //重置准备进入状态。
            shootUnit->Reloader.alreadyPush = 1;
            shootUnit->Reloader.completePush = 0;
            ResetPill(shootUnit);
            shootUnit->Reloader.totalAngle_start = Motor->totalAngle;
            Motor->targetAngle += shootUnit->NeedLaunchCount * Angle_Pill * Reloader1Direction;
        }

        if (abs(Motor->totalAngle - shootUnit->Reloader.totalAngle_start) >= shootUnit->NeedLaunchCount * Angle_Pill - 8191)
        { //拨盘任务完成

            shootUnit->NeedLaunchCount = 0;
            shootUnit->Reloader.alreadyPush = 0;
            shootUnit->Reloader.completePush = 1;
            shootUnit->Reloader.PillObstruct = 0;
            shootUnit->Reloader.PillObstruct_targetAngle = 0;
            shootUnit->Reloader.amount_hadLaunch = shootUnit->NeedLaunchCount;
            shootUnit->Reloader.PillObstruct_Time = 0;

            //ResetPill();
        }
        else
        {
            shootUnit->Reloader.amount_hadLaunch = abs(Motor->totalAngle - shootUnit->Reloader.totalAngle_start) / Angle_Pill; //实时更新剩下待拨动弹丸量。
        }

        //-------  卡弹检测   -----
        if (Shoot_MotorInObstruct(shootUnit)) //扭矩超过卡弹扭矩。
        {
            shootUnit->Reloader.PillObstruct_Time++; //计算已卡弹的时间。
            if (shootUnit->Reloader.PillObstruct_Time > limitTime)
            {
                shootUnit->Reloader.PillObstruct = 1;           //激活卡弹标志位，下一次开始反转。
                shootUnit->Reloader.PillObstruct_Direction = 1; //开始执行首次反转任务
                shootUnit->Reloader.PillObstruct_targetAngle = Motor->totalAngle - Angle_Pill * 2;
            }
        }
        else
        {
            if (shootUnit->Reloader.PillObstruct_Time >= 0)
            {
                shootUnit->Reloader.PillObstruct_Time -= 10;
            }
            else
            {
                shootUnit->Reloader.PillObstruct_Time = 0;
            }
        }
    }
    else
    {
        shootUnit->Reloader.completePush = 0;
        shootUnit->NeedLaunchCount = 0;
    }

    Position_PID(&M2006s_YawOPID, Motor->targetAngle, Motor->totalAngle);
    //速度PID计算(内环)
    Motor->outCurrent = Position_PID(&M2006s_YawIPID, M2006s_YawOPID.pwm, Motor->realSpeed);

    //清标志位
    Motor->InfoUpdateFlag = 0;
    Motor->OffLineFlag = 0;
    Motor->InfoUpdateFrame++;
}

//************************************
// Method:    Shoot_setNeedLaunchCount
// Parameter: int32_t amount 覆盖当前的数量
//************************************
void Shoot_setNeedLaunchCount(ShootUnit_t *shootUnit, uint16_t amount)
{                    //重新设置目前的射击数量。
    if (amount <= 0) //紧急停止状态。避免卡弹造成扣血。
    {
        shootUnit->Reloader.alreadyPush = 0;

        shootUnit->NeedLaunchCount = 0;
        shootUnit->Reloader.PillObstruct = 0;
        shootUnit->Reloader.PillObstruct_targetAngle = 0;
        shootUnit->Reloader.PillObstruct_Time = 0;
        ResetPill(shootUnit);
        return;
    }

    if (shootUnit->Reloader.PillObstruct != 1)
    {
        shootUnit->Reloader.alreadyPush = 0;
        ResetPill(shootUnit);
        shootUnit->NeedLaunchCount = amount;
    }
}

/**
 * @brief //设置弹仓盖开关
 * 
 * @param open 
 * @return  
 */
bool Shoot_openCartridge(bool open)
{
#if Infantry_Year == Infantry_2020
    if (open == true)
    {
        Robot.Magazine = Magazine_Open;
        __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 750); //开弹仓。
    }
    else
    {
        Robot.Magazine = Magazine_Close;
        __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 1720); //关弹仓。
    }
#elif Infantry_Year == Infantry_2021
    if (open == true)
    {
        Robot.Magazine = Magazine_Open;
        __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 800); //开弹仓。
    }
    else
    {
        Robot.Magazine = Magazine_Close;
        __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 2050); //关弹仓。
    }
#endif
    return open;
}

static bool Shoot_MotorInObstruct(ShootUnit_t *shootUnit)
{

    return abs(shootUnit->Reloader_Motor->realSpeed) < shootUnit->Reloader.ObstructSpeed;
}

/**
 * @brief 热量限制
 * 
 * @return  
 */
static void Shoot_HeatLimitation(void)
{
    if (ext_game_robot_state.data.shooter_id1_17mm_cooling_limit > 0 && ext_game_robot_state.data.shooter_id1_17mm_cooling_limit <= 1000)
    {
        /*获取裁判系统热量值，待测试*/
        Shoot_Judge.shoot_LimitBullet = ext_game_robot_state.data.shooter_id1_17mm_cooling_limit / Fixed_BulletHeat; //减去 是为了避免非常接近临界值。
        /*******************************/
        needShoot = Shoot_getMaxShootAmount();
    }
    else
    {
        Shoot_Judge.shoot_LimitBullet = 50 / Fixed_BulletHeat; //减去 是为了避免非常接近临界值。
        /*******************************/
        needShoot = Shoot_getMaxShootAmount();
    }
}

/**
 * @brief 热量限制
 * 
 * @return  
 */
uint16_t Bullet_Time;
static void CountLaunchBullet(void)
{
    //计算发出弹量
    if (shootUnit1.Reloader.alreadyPush == 1)
    {
        Shoot_Judge.shoot_RealBullet = shootUnit1.NeedLaunchCount;
    }
    else if (shootUnit1.Reloader.completePush == 1)
    {
        Shoot_Judge.shoot_OverallBullet += Shoot_Judge.shoot_RealBullet;
        Shoot_Judge.shoot_RealBullet = 0;
    }
    //减去冷却值
    Shoot_Judge.shoot_OverallBullet -= ext_game_robot_state.data.shooter_id1_17mm_cooling_rate / 5000.0f;
    //防止热量小于零
    if (Shoot_Judge.shoot_OverallBullet < 0)
    {
        Shoot_Judge.shoot_OverallBullet = 0;
    }

    //获取裁判系统返回热量值
    Shoot_Judge.shoot_JudgeOverallBullet = ext_power_heat_data.data.shooter_id1_17mm_cooling_heat / Fixed_BulletHeat;

    //偏差超过一定时间使用裁判系统进行修正
    if (abs(Shoot_Judge.shoot_JudgeOverallBullet - Shoot_Judge.shoot_OverallBullet) >= (Shoot_Judge.shoot_LimitBullet * 0.2 - 1))
    {
        if (Bullet_Time > 50)
        {
            Shoot_Judge.shoot_OverallBullet = Shoot_Judge.shoot_JudgeOverallBullet;
            Bullet_Time = 0;
        }
        Bullet_Time++;
    }
    else
    {
        Bullet_Time = 0;
    }
}

/**
 * @brief 自动打击
 * 
 * @param shootUnit 
 * @return  
 */
static void AutoAttack_process(ShootUnit_t *shootUnit)
{ //自动射击处理函数。
    if (Robot.Attack_AttackMode != AttackMode_AutoMode || VisionExportData.AbleToFire == false)
    {
        return;
    }
    shootUnit_WorldTime.WorldTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (RM_Judge.OffLineFlag == 0)
    {
        Shoot_HeatLimitation();
    }
    else
    {
        needShoot = 1;
    }

    need_shoottime = 50; //计算打弹丸颗数的时间间距

    //根据计算间隔打一发弹丸9
    uint32_t spacing = shootUnit_WorldTime.WorldTime - shootUnit_WorldTime.Last_WorldTime;
    if ((int)(spacing / need_shoottime) % 2 == 0 && shootUnit->Reloader.Reloader_license == Reloader_allow)
    {
        shootUnit1.Reloader.Reloader_license = Reloader_ban;
    }
    else if ((int)(spacing / need_shoottime) % 2 == 1 && shootUnit->Reloader.Reloader_license == Reloader_ban)
    {
        shootUnit1.Reloader.Reloader_license = Reloader_allow;
        if (needShoot <= 0)
        {
            Shoot_setNeedLaunchCount(&shootUnit1, 0);
        }
        else
        {

            Shoot_setNeedLaunchCount(&shootUnit1, 1);
        }
    }

    //	Shoot_setNeedLaunchCount(shootUnit, bullet1);
}

/**
 * @brief 遥控打击
 * 
 * @param shootUnit 
 * @param speed 
 * @param amount 
 * @return  
 */
static void Manual_Attack_process(ShootUnit_t *shootUnit, uint16_t speed, uint16_t *amount)
{
    if (Robot.Attack_AttackMode != AttackMode_ManualMode || Robot.ControlSource == ControlSource_PC)
    {
        return;
    }
    if (RM_Judge.OffLineFlag == 0 && Robot.Device_FrictMode != FrictWorkMode_LowSpeed)
    {
        Shoot_HeatLimitation();
    }
    else
    {
        needShoot = 1;
    }

    shootUnit_WorldTime.WorldTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (speed == 0)
    {
        shootUnit_WorldTime.Last_WorldTime = shootUnit_WorldTime.WorldTime;
    }

    need_shoottime = 690 - speed;
    // need_shoottime = 500 / needShoot; //计算打弹丸颗数的时间间距

    //根据计算间隔打一发弹丸

    uint32_t spacing = shootUnit_WorldTime.WorldTime - shootUnit_WorldTime.Last_WorldTime;
    if ((int)(spacing / need_shoottime) % 2 == 0 && shootUnit->Reloader.Reloader_license == Reloader_allow)
    {
        shootUnit1.Reloader.Reloader_license = Reloader_ban;
    }
    else if ((int)(spacing / need_shoottime) % 2 == 1 && shootUnit->Reloader.Reloader_license == Reloader_ban)
    {
        shootUnit1.Reloader.Reloader_license = Reloader_allow;
        if (needShoot <= 0)
        {
            Shoot_setNeedLaunchCount(&shootUnit1, 0);
        }
        else
        {
            Shoot_setNeedLaunchCount(&shootUnit1, 1);
        }
    }
}

/**
 * @brief 单发模式
 * 
 * @param shootUnit 
 */
static void SingleAttack_process(ShootUnit_t *shootUnit)
{
    if (Robot.Attack_AttackMode != AttackMode_SingleMode && Robot.Attack_AttackMode != AttackMode_NoneMode)
    {
        return;
    }

    if (Robot.Attack_AttackMode == AttackMode_NoneMode)
    {
        needShoot = 0;
    }
    else if (RM_Judge.OffLineFlag == 0)
    {
        Shoot_HeatLimitation();
    }
    else
    {
        needShoot = 1;
    }

    if (needShoot > 0)
    {
        Shoot_setNeedLaunchCount(&shootUnit1, 1);
    }
    // else
    // {
    //     Shoot_setNeedLaunchCount(&shootUnit1, 1);
    // }
}

/**
 * @brief 连发模式
 * 
 * @param shootUnit 
 * @param time 
 */
static void ConAttack_process(ShootUnit_t *shootUnit, uint16_t time)
{
    if (Robot.Attack_AttackMode != AttackMode_ContinuousMode)
    {
        return;
    }
    if (RM_Judge.OffLineFlag == 0)
    {
        Shoot_HeatLimitation();
    }
    else
    {
        needShoot = 1;
    }

    shootUnit_WorldTime.WorldTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (time == 0)
    {
        shootUnit_WorldTime.Last_WorldTime = shootUnit_WorldTime.WorldTime;
    }

    need_shoottime = time;

    //根据计算间隔打一发弹丸

    uint32_t spacing = shootUnit_WorldTime.WorldTime - shootUnit_WorldTime.Last_WorldTime;
    if ((int)(spacing / need_shoottime) % 2 == 0 && shootUnit->Reloader.Reloader_license == Reloader_allow)
    {
        shootUnit1.Reloader.Reloader_license = Reloader_ban;
    }
    else if ((int)(spacing / need_shoottime) % 2 == 1 && shootUnit->Reloader.Reloader_license == Reloader_ban)
    {
        shootUnit1.Reloader.Reloader_license = Reloader_allow;
        if (needShoot <= 0)
        {
            Shoot_setNeedLaunchCount(&shootUnit1, 0);
        }
        else
        {
            Shoot_setNeedLaunchCount(&shootUnit1, 1);
        }
    }
}

//获取当前发射的预计初速度。
static double Shoot_getPresentVo(ShootUnit_t *shootUnit)
{
    //通过snail的pwm 拟合出对应的初速度曲线。
    double p1 = 2.941e-07;
    double p2 = -0.001246;
    double p3 = 1.766;
    double p4 = -810.9;

    return p1 * pow((double)shootUnit->FricOutput + 1000.0, 3) + p2 * pow((double)shootUnit->FricOutput + 1000.0, 2) + p3 * ((double)shootUnit->FricOutput + 1000.0) + p4;
}

/**
 * @brief 获取能击打弹丸值
 * 
 * @return  
 */
static int Shoot_getMaxShootAmount(void)
{
    int Bullet = 1;
    int LimitBullet = 0;
    if (Robot.OpenBurstOut == OpenBurstOut_End)
    {
        Bullet = Shoot_Judge.shoot_LimitBullet - Shoot_Judge.shoot_OverallBullet;
        if (ext_game_robot_state.data.shooter_id1_17mm_cooling_limit == 50)
        {
            LimitBullet = Shoot_Judge.shoot_LimitBullet * 0.6;
        }
        else
        {
            LimitBullet = Shoot_Judge.shoot_LimitBullet * 0.2;
        }
        if (Bullet < LimitBullet)
        {
            Bullet = 0;
        }
    }
    return Bullet;
}

//static double Shoot_ParabolaCompensate(bool isVision)
//{ //抛物线补偿
//    //返回补偿角度，单位为度

//    double d = 0;
//    if (isVision == true)
//    { //激光测距补偿
//        d = VisionExportData.FinalOffset_depth;
//    }
//    else
//    {
//        //视觉的深度转换为距离
//    }

//    double angle_A = 0; //计算得到的补偿角度，单位度数
//    double angle_P = 0;
//    angle_P = M6020_getRoundAngle(Cloud_Pitch_Center - M6020s_Pitch.totalAngle) + 90.0f; //需要根据电机增量方向修改正负。

//    double v0 = Shoot_getPresentVo(&shootUnit1);

//    bool run = Parabola_Matching(radians(angle_P), d, v0, &angle_A);
//    if (run == true)
//    {
//        //		printf("d=%.2f v0=%.1f P = %.2f~A= %.2f\n", VisionData.RawData.depth, v0, angle_P, angle_A);
//        //Cloud.targetPitchRaw -= angle_A * M6020_mAngleRatio;
//        return angle_A;
//    }
//    else
//    {
//        //		printf("错误d=%.3f v0=%.1f P = %.2f~A= %.2f\n", VisionData.RawData.depth, v0, angle_P, angle_A);
//        return 0;
//    }
//}
