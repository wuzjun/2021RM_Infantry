/**
 * @file Cloud_control.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Cloud_control.h"
#include "Chassis_control.h"
#include "Control_Vision.h"
#include "DJI_C_IMU.h"
#include "DJI_IMU.h"
#include "DR16_Remote.h"
#include "IMU_Compensate.h"
#include "M6020_Motor.h"
#include "Robot_control.h"
#include "USER_Filter.h"
#include "Wolf_GyIMU.h"
#include "cmsis_os.h"

Cloud_t Cloud;
GY_IMUExport_t IMUExport_Cloud;

/************电机PID***********/
positionpid_t M6020s_YawOPID = M6020s_YawOPIDInit; //YAW轴云台电机外环
#undef M6020s_YawOPIDInit
positionpid_t M6020s_YawIPID = M6020s_YawIPIDInit; //YAW轴云台电机内环
#undef M6020s_YawIPIDInit
positionpid_t M6020s_PitchOPID = M6020s_PitchOPIDInit; //PITCH轴云台电机外环
#undef M6020s_PitchOPIDInit
positionpid_t M6020s_PitchIPID = M6020s_PitchIPIDInit; //PITCH轴云台电机内环
#undef M6020s_PitchIPIDInit
/************电机PID END***********/

/************电机+视觉PID***********/
positionpid_t VisionM6020s_YawOPID = VisionM6020s_YawOPIDInit; //YAW轴云台电机+视觉外环
#undef VisionM6020s_YawOPIDInit
positionpid_t VisionM6020s_YawIPID = VisionM6020s_YawIPIDInit; //YAW轴云台电机+视觉内环
#undef VisionM6020s_YawIPIDInit
positionpid_t VisionM6020s_PitchOPID = VisionM6020s_PitchOPIDInit; //PITCH轴云台电机+视觉外环
#undef VisionM6020s_PitchOPIDInit
positionpid_t VisionM6020s_PitchIPID = VisionM6020s_PitchIPIDInit; //PITCH轴云台电机+视觉内环
#undef VisionM6020s_PitchIPIDInit
/************电机+视觉PID END***********/

/************云台--陀螺仪PID***********/
positionpid_t YawAttitude_PID = YawAttitude_PIDInit; //云台yaw轴姿态pid
#undef YawAttitude_PIDInit
positionpid_t YawSpeed_PID = YawSpeed_PIDInit; //云台yaw轴速度pid
#undef YawSpeed_PIDInit
positionpid_t PitchAttitude_PID = PitchAttitude_PIDInit; //云台roll轴姿态pid
#undef PitchAttitude_PIDInit
positionpid_t PitchSpeed_PID = PitchSpeed_PIDInit; //云台roll轴姿态pid
#undef PitchSpeed_PIDInit
/************云台--陀螺仪PID END***********/

/************云台--陀螺仪+视觉PID*************/
positionpid_t VisionYawAttitude_PID = VisionYawAttitude_PIDInit;
#undef VisionYawAttitude_PIDInit
positionpid_t VisionYawSpeed_PID = VisionYawSpeed_PIDInit;
#undef VisionYawSpeed_PIDInit
positionpid_t VisionPitchAttitude_PID = VisionPitchAttitude_PIDInit;
#undef VisionPitchAttitude_PIDInit
positionpid_t VisionPitchSpeed_PID = VisionPitchSpeed_PIDInit;
#undef VisionPitchSpeed_PIDInit
/************云台--陀螺仪+视觉PID END*************/

/***********视觉双环PID************/
positionpid_t Vision_AnglePid_Yaw = Vision_AnglePid_Yaw_PIDInit;
#undef Vision_AnglePid_Yaw_PIDInit
positionpid_t Vision_SpeedPid_Yaw = Vision_SpeedPid_Yaw_PIDInit;
#undef Vision_SpeedPid_Yaw_PIDInit

positionpid_t Vision_AnglePid_Pitch = Vision_AnglePid_Pitch_PIDInit;
#undef Vision_AnglePid_Pitch_PIDInit
positionpid_t Vision_SpeedPid_Pitch = Vision_SpeedPid_Pitch_PIDInit;
#undef Vision_SpeedPid_Pitch_PIDInit

VisionFourth_Ring_t VisionFourth_Ring_Yaw = {0, &Vision_AnglePid_Yaw, &Vision_SpeedPid_Yaw};
VisionFourth_Ring_t VisionFourth_Ring_Pitch = {0, &Vision_AnglePid_Pitch, &Vision_SpeedPid_Pitch};

/***********视觉双环PID END************/

/*************卡尔曼结构体**************/
Kalman_Data_t Kalman_YawData = Kalman_YawDataInit;
#undef Kalman_YawDataInit
Kalman_Data_t Kalman_PitchData = Kalman_PitchDataInit;
#undef Kalman_PitchDataInit
kalman_filter_init_t yaw_kalman_filter_para = {
    .P_data = {2, 0, 0, 2},
    .A_data = {1, 0.002 /*0.001*/, 0, 1}, //采样时间间隔
    .H_data = {1, 0, 0, 1},
    .Q_data = {1, 0, 0, 1},
    .R_data = {200, 0, 0, 400} //500 1000
};                             //初始化yaw的部分kalman参数

kalman_filter_init_t pitch_kalman_filter_para = {
    .P_data = {2, 0, 0, 2},
    .A_data = {1, 0.002 /*0.001*/, 0, 1}, //采样时间间隔
    .H_data = {1, 0, 0, 1},
    .Q_data = {1, 0, 0, 1},
    .R_data = {200, 0, 0, 400}}; //初始化pitch的部分kalman参数

kalman_filter_t yaw_kalman_filter;
kalman_filter_t pitch_kalman_filter;
float kf_angle_temp; //预测角度斜坡暂存量
SpeedRamp_t KalmanRamp = KalmanRampInit;
#undef KalmanRampInit

extKalman_t Cloud_YawGyroAngle_Error_Kalman;
extKalman_t Cloud_PitchGyroAngle_Error_Kalman;
extKalman_t Cloud_YawMotorAngle_Error_Kalman;
extKalman_t Cloud_PitchMotorAngle_Error_Kalman;
/*************卡尔曼结构体END**************/

speed_calc_data_t Vision_Yaw_speed_Struct;
speed_calc_data_t Vision_Pitch_speed_Struct;

positionpid_t testSpeed_PID = TestSpeed_PIDInit;
#undef TestSpeed_PIDInit
Cloud_ParameterDeBug_t Cloud_ParameterDeBug[2] = {{0, 180}, {0, 0}};

All_VisionCompensate_t All_VisionComYaw = {&Kalman_YawData, &VisionFourth_Ring_Yaw};
All_VisionCompensate_t All_VisionComPitch = {&Kalman_PitchData, &VisionFourth_Ring_Pitch};

void Cloud_Init(void);
void Cloud_processing_WithIMU(float delta_yaw, float delta_pitch);
void Cloud_processing_WithoutIMU(float delta_yaw, float delta_pitch);
float Cloud_getYawAngleWithCenter(void);
void IMUData_chance(void);
Cloud_FUN_t Cloud_FUN = Cloud_FUNGroundInit;
#undef Cloud_FUNGroundInit

/****************Pithch限位*****************/
#if Who_Infantry == ZJ_Infantry
const float Cloud_Pitch_Min = -1367;
const float Cloud_Pitch_Max = -52;
const float Cloud_Pitch_Center = Center(-1367, -52);
#elif Who_Infantry == ZF_Infantry
const float Cloud_Pitch_Min = 2170;
const float Cloud_Pitch_Max = 3280;
const float Cloud_Pitch_Center = Center(2170, 3280);
#endif
static void Cloud_setIMUPosForced(float posYaw, float posPitch);
static float Cloud_getPitchAngleWithDown(void);
static float Cloud_getPitchAngleWithUp(void);
static void IMU_LimUpdate(void);
/****************Pithch限位  End*****************/

/**
 * @brief 挂挡C板陀螺仪
 * 
 */
static void C_IMUEngage(void)
{
    Cloud.RealData.IMUAngle_Yaw = &/* DJI_C_IMU.total_yaw */ imu_Export.total[0];
    Cloud.RealData.IMUAngle_Pitch = &DJI_C_IMU.pitch;
    Cloud.RealData.IMUSpeed_Yaw = &/* DJI_C_IMU.Gyro_z */ imu_Export.SpeedLPF[0];
    Cloud.RealData.IMUSpeed_Pitch = &DJI_C_IMU.Gyro_y;

    /**********Yaw 正常+视觉***********/
    // Position_PIDInit(&YawAttitude_PID, -0.9f, 0, 0, 600, 0, 0);
    // Position_PIDInit(&YawSpeed_PID, 380, 30, 0, M6020_MaxOutput, 100, 10000);
    // Position_PIDInit(&VisionYawAttitude_PID, -0.9f, 0, 0, 600, 0, 0);
    // Position_PIDInit(&VisionYawSpeed_PID, 380, 30, 0, M6020_MaxOutput, 100, 10000);

    /**********Pitch 正常+视觉***********/
    Position_PIDInit(&PitchAttitude_PID, 0.9, 0, 0, 600, 0, 0);
    Position_PIDInit(&PitchSpeed_PID, 400, 15, 0, M6020_MaxOutput, 100, 9000);
    Position_PIDInit(&VisionPitchAttitude_PID, 0.7, 0, 0, 600, 0, 0);
    Position_PIDInit(&VisionPitchSpeed_PID, 400, 15, 0, M6020_MaxOutput, 100, 9000);
}

/**
 * @brief 挂挡A板陀螺仪
 * 
 */
static void A_IMUEngage(void)
{
    Cloud.RealData.IMUAngle_Yaw = &imu_Export.total[0];
    Cloud.RealData.IMUAngle_Pitch = &M6020s_Pitch.totalAngle;
    Cloud.RealData.IMUSpeed_Yaw = &imu_Export.SpeedLPF[0];
    Cloud.RealData.IMUSpeed_Pitch = &imu_Export.SpeedLPF[1];

    /**********Yaw 正常+视觉***********/
    // Position_PIDInit(&YawAttitude_PID, -8.5f, 0, 0, 18000, 0, 0);
    // Position_PIDInit(&YawSpeed_PID, 50.0f, 0.3f, 0, M6020_MaxOutput, 1000, 15000);
    // Position_PIDInit(&VisionYawAttitude_PID, -8.5f, 0, 0, 18000, 0, 0);
    // Position_PIDInit(&VisionYawSpeed_PID, 50.0f, 0.3f, 0, M6020_MaxOutput, 1000, 15000);

    /**********Pitch 正常+视觉***********/
    Position_PIDInit(&PitchAttitude_PID, 18.0f, 0, 0, 10000, 0, 3000);
    Position_PIDInit(&PitchSpeed_PID, 9.1f, 0.09f, 0, M6020_MaxOutput, 4500, 9000);
    Position_PIDInit(&VisionPitchAttitude_PID, 18.0f, 0, 0, 10000, 0, 3000);
    Position_PIDInit(&VisionPitchSpeed_PID, 9.1f, 0.09f, 0, M6020_MaxOutput, 4500, 9000);
}

/**
 * @brief  云台初始化，配置参数并归位云台
 * @param  None
 * @retval None
 */
void Cloud_Init(void)
{

    Cloud.YawLpfAttFactor = 1.0f;
    Cloud.PitchLpfAttFactor = 1.0f;

    if (DJI_C_IMU.OffLineFlag == 0) //C板陀螺仪在线
    {
        C_IMUEngage();
        IMU_LimUpdate();
        Cloud.RealData.IMUState = C_IMU;
    }
    else
    {
        A_IMUEngage();
        Cloud.RealData.IMUState = A_IMU;
    }

    //保存启动时刻的机械角度
    Cloud.targetYawRaw = Cloud.targetYawLPF = M6020s_Yaw.totalAngle; //开机让云台回中。
    Cloud.targetPitchRaw = Cloud.targetPitchLPF = Cloud_Pitch_Center;

    //保存启动时刻的陀螺仪姿态
    Cloud_setIMUPosForced(*Cloud.RealData.IMUAngle_Yaw, Center(Cloud.IMUPitchAngleMin, Cloud.IMUPitchAngleMax));

    M6020_Fun.M6020_setTargetAngle(&M6020s_Yaw, Cloud.targetYawRaw);
    M6020_Fun.M6020_setTargetAngle(&M6020s_Pitch, Cloud_Pitch_Center);

    kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
    kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);

    KalmanCreate(&Cloud_YawGyroAngle_Error_Kalman, 1, 40);
    KalmanCreate(&Cloud_PitchGyroAngle_Error_Kalman, 1, 40);
    KalmanCreate(&Cloud_YawMotorAngle_Error_Kalman, 1, 40);
    KalmanCreate(&Cloud_PitchMotorAngle_Error_Kalman, 1, 40);
    KalmanCreate(&Cloud_YAWODKalman, 1, 10);
    KalmanCreate(&Cloud_PITCHODKalman, 1, 10);
}

uint8_t Self_Lock = 0;
void IMUData_chance(void)
{
    if (DJI_C_IMU.OffLineFlag == 0 && Self_Lock == 1)
    {
        C_IMUEngage();
        Self_Lock = 0;
        Cloud_setIMUPosForced(*Cloud.RealData.IMUAngle_Yaw, *Cloud.RealData.IMUAngle_Pitch);
        Cloud.RealData.IMUState = C_IMU;
    }
    else if (imu_Export.OffLineFlag == 0 && DJI_C_IMU.OffLineFlag != 0 && Self_Lock == 0)
    {
        A_IMUEngage();
        Self_Lock = 1;
        Cloud_setIMUPosForced(*Cloud.RealData.IMUAngle_Yaw, *Cloud.RealData.IMUAngle_Pitch);
        Cloud.RealData.IMUState = A_IMU;
    }
}

static void IMU_LimUpdate(void)
{
    Cloud.IMUPitchAngleMax = *Cloud.RealData.IMUAngle_Pitch + fabs(Cloud_getPitchAngleWithUp());
    Cloud.IMUPitchAngleMin = *Cloud.RealData.IMUAngle_Pitch - fabs(Cloud_getPitchAngleWithDown());
}

/**
 * @brief 
 * 
 * @param PitchAngle 
 * @param Cloud_Pitch_Min 
 * @param Cloud_Pitch_Max 
 */
static void Cloud_AngleLimit(float *PitchAngle, const float Cloud_Pitch_Min, const float Cloud_Pitch_Max)
{
    //PITCH为机械角度写死的限位，第一次使用前务必修改或注释！需要根据电机方向适当调整符号。
    if (Robot.Sport_ChassisWorkMode == ChassisWorkMode_NewFollow || ReversefollowFlag.NewFollowFlag == 1)
    {
        if (*PitchAngle < Cloud_Pitch_Min)
        {
            *PitchAngle = Cloud_Pitch_Min;
        }
        else if (*PitchAngle > Cloud_Pitch_Max)
        {
            *PitchAngle = Cloud_Pitch_Max;
        }
    }
    else
    {
        switch (Cloud.RealData.IMUState)
        {
        case A_IMU:
        {
            if (*PitchAngle < Cloud_Pitch_Min)
            {
                *PitchAngle = Cloud_Pitch_Min;
            }
            else if (*PitchAngle > Cloud_Pitch_Max)
            {
                *PitchAngle = Cloud_Pitch_Max;
            }
            break;
        }
        case C_IMU:
        {
            IMU_LimUpdate();
            if (*PitchAngle < Cloud.IMUPitchAngleMin)
            {
                *PitchAngle = Cloud.IMUPitchAngleMin;
            }
            else if (*PitchAngle > Cloud.IMUPitchAngleMax)
            {
                *PitchAngle = Cloud.IMUPitchAngleMax;
            }
            break;
        }
        }
    }
}

/**
 * @brief  强制设置云台IMU坐标（绕过缓冲）
 * @param[in]  posYaw
 *				posPitch
 * @retval None
 */
static void Cloud_setIMUPosForced(float posYaw, float posPitch)
{
    Cloud.IMUtargetYawLPF = Cloud.IMUtargetYawRaw = posYaw;
    Cloud.IMUtargetPitchLPF = Cloud.IMUtargetPitchRaw = posPitch;
}

// /**
//  * @brief  获取当前云台Yaw的偏离角度（以右为0点）右负左正
//  * @param[in]  None
//  * @retval 角度
//  */
// float Cloud_getYawAngleWithRight(void)
// {
//     return (M6020s_Yaw.totalAngle - Cloud_Yaw_Max) / M6020_mAngleRatio;
// }

// /**
//  * @brief 获取当前云台Yaw的偏离角度（以右为0点）右负左正
//  * 
//  * @return 角度  
//  */
// float Cloud_getYawAngleWithLeft(void)
// {
//     return (M6020s_Yaw.totalAngle - Cloud_Yaw_Min) / M6020_mAngleRatio;
// }

/**
 * @brief  获取云台Pitch旋转角度（以下为0点）下负上正
 * @param[in]  None
 * @retval 角度
 */
static float Cloud_getPitchAngleWithDown(void)
{
    if (M6020s_Pitch.totalAngle < Cloud_Pitch_Min)
    {
        return 0;
    }
    else
    {
        return (M6020s_Pitch.totalAngle - Cloud_Pitch_Min);
    }
}

/**
 * @brief  获取云台Pitch旋转角度（以下为0点）下负上正
 * @param[in]  None
 * @retval 角度
 */
static float Cloud_getPitchAngleWithUp(void)
{
    if (M6020s_Pitch.totalAngle > Cloud_Pitch_Max)
    {
        return 0;
    }
    else
    {
        return (M6020s_Pitch.totalAngle - Cloud_Pitch_Max);
    }
}

/**
 * @brief  通过6020机械角度的方式获取云台Yaw旋转的角度（偏移车正前方的角度-中心点）
 * @param[in]  None
 * @retval 360度的角度值。
 */
float Cloud_getYawAngleWithCenter(void)
{
    return (M6020s_Yaw.totalAngle - Cloud_Yaw_Center) / M6020_mAngleRatio;
}

/**
 * @brief  强制设置云台机械坐标（绕过缓冲区）
 * @param[in]  posYaw
 *				posPitch
 * @retval None
 */
void Cloud_setAbsPosForced(float posYaw, float posPitch)
{
    Cloud.targetYawLPF = Cloud.targetYawRaw = posYaw;
    Cloud.targetPitchLPF = Cloud.targetPitchRaw = posPitch;
}

/**
 * @brief 计算视觉速度
 * 
 * @param S 
 * @param time 
 * @param position 
 * @return  
 */
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
    S->delay_cnt++;

    if (time != S->last_time)
    {
        S->speed = (position - S->last_position) / (time - S->last_time) * 2; //计算速度

        S->processed_speed = S->speed;

        S->last_time = time;
        S->last_position = position;
        S->last_speed = S->speed;
        S->delay_cnt = 0;
    }

    if (S->delay_cnt > 300 /*100*/) // delay 200ms speed = 0
    {
        S->processed_speed = 0; //时间过长则认为速度不变
    }

    return S->processed_speed; //计算出的速度
}

/**
 * @brief Get the Vision Angle object
 * 
 * @param XY 
 * @return  
 */
float GetVisionAngle(uint8_t XY)
{
    float VisionAngle = 0;
    if (XY == VisionYaw)
    {
        VisionAngle = VisionExportData.FinalOffset.x;
    }
    else
    {
        VisionAngle = VisionExportData.FinalOffset.y;
    }
    return VisionAngle;
}

/**
 * @brief 云台--视觉数据补偿
 * 
 * @param All_VisionCom 四环或卡尔曼
 * @param Vision_speed_Struct 计算视觉速度
 * @param kalman_filter 二阶卡尔曼
 * @param Angle_raw IMU角度+视觉角度
 * @param Type 开启卡尔曼/开启四环/不开启
 * @param XY 判断 YAW 或 PITCH
 * @return  
 */
static float Cloud_VisionFusion(All_VisionCompensate_t *All_VisionCom, speed_calc_data_t *Vision_speed_Struct, kalman_filter_t *kalman_filter, float Angle_raw, uint8_t Type, uint8_t XY)
{
    float Vision_Angle = Angle_raw;

    if (Type == Kalman)
    {
        float Vision_Speed = Target_Speed_Calc(Vision_speed_Struct, VisionKF_TIME.WorldTime, Angle_raw);

        All_VisionCom->Kalman_Data->Kf_result = kalman_filter_calc(kalman_filter, Angle_raw, Vision_Speed);

        All_VisionCom->Kalman_Data->Kf_Delay++;
        if (fabs(GetVisionAngle(XY)) < Angle_limit                                                                               /* 角度过大不开启预测 */
            && All_VisionCom->Kalman_Data->Kf_Delay > All_VisionCom->Kalman_Data->Parameter.kf_delay_open                        /* 延时开启 */
            && fabs(All_VisionCom->Kalman_Data->Kf_result[Kf_Speed]) >= All_VisionCom->Kalman_Data->Parameter.Predicted_SpeedMin /* 速度过小不开启预测 */
            && fabs(All_VisionCom->Kalman_Data->Kf_result[Kf_Speed]) < All_VisionCom->Kalman_Data->Parameter.Predicted_SpeedMax /* 速度过大不开启预测 */)
        {
            if (All_VisionCom->Kalman_Data->Kf_result[Kf_Speed] >= 0)
            {
                kf_angle_temp = All_VisionCom->Kalman_Data->Parameter.Predicted_Factor * (All_VisionCom->Kalman_Data->Kf_result[Kf_Speed] - All_VisionCom->Kalman_Data->Parameter.Predicted_SpeedMin);
            }
            else if (All_VisionCom->Kalman_Data->Kf_result[Kf_Speed] < 0)
            {
                kf_angle_temp = All_VisionCom->Kalman_Data->Parameter.Predicted_Factor * (All_VisionCom->Kalman_Data->Kf_result[Kf_Speed] + All_VisionCom->Kalman_Data->Parameter.Predicted_SpeedMin);
            }

            kf_angle_temp *= (VisionExportData.FinalOffset_depth / 1000.0f) / (ext_game_robot_state.data.shooter_id1_17mm_speed_limit - 4);

            /***********补偿速度斜坡***********/
            VAL_LIMIT(kf_angle_temp, KalmanRamp.mincount, KalmanRamp.maxcount);
            All_VisionCom->Kalman_Data->Vision_Speed = RAMP_float(kf_angle_temp, All_VisionCom->Kalman_Data->Vision_Speed, KalmanRamp.rate);

            /***********补偿速度斜坡 END***********/
            Vision_Angle = All_VisionCom->Kalman_Data->Vision_Angle = All_VisionCom->Kalman_Data->Kf_result[Kf_Angle] /* Angle_raw */ + All_VisionCom->Kalman_Data->Vision_Speed;
        }
    }
    else if (Type == Fourth_Ring)
    {
        float Vision_Speed = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), Angle_raw);
        All_VisionCom->VisionFourth_Ring->Vision_AnglePID->Position_PID(All_VisionCom->VisionFourth_Ring->Vision_AnglePID, 0, Angle_raw);
        All_VisionCom->VisionFourth_Ring->Vision_SpeedPID->Position_PID(All_VisionCom->VisionFourth_Ring->Vision_SpeedPID, All_VisionCom->VisionFourth_Ring->Vision_AnglePID->pwm, Vision_Speed);
        Vision_Angle = All_VisionCom->VisionFourth_Ring->Vision_SpeedPID->pwm + All_VisionCom->VisionFourth_Ring->Speed_Gain;
    }
    else
    {
        Vision_Angle = Angle_raw;
    }
    return Vision_Angle;
}

/**
 * @brief 清除视觉补偿缓存值
 * 
 * @param All_VisionCom 
 */
static void Clear_VisionFusion(All_VisionCompensate_t *All_VisionCom, kalman_filter_t *kalman_filter, float Angle_raw)
{
    //卡尔曼部分
    All_VisionCom->Kalman_Data->Kf_Delay = 0;
    All_VisionCom->Kalman_Data->Kf_result = kalman_filter_calc(kalman_filter, Angle_raw, 0);

    //四环部分
    Clear_PositionPIDData(All_VisionCom->VisionFourth_Ring->Vision_AnglePID);
    Clear_PositionPIDData(All_VisionCom->VisionFourth_Ring->Vision_SpeedPID);
}

/**
 * @brief 清除云台陀螺仪PID缓冲值
 * 
 * @return  
 */
static void Cloud_ClearIMUPIDData(void)
{
    /*************清除IMU--Yaw轴****************/
    Clear_PositionPIDData(&YawAttitude_PID);
    Clear_PositionPIDData(&YawSpeed_PID);
    Clear_PositionPIDData(&VisionYawAttitude_PID);
    Clear_PositionPIDData(&VisionYawSpeed_PID);

    /*************清除IMU--Pitch轴****************/
    Clear_PositionPIDData(&PitchAttitude_PID);
    Clear_PositionPIDData(&PitchSpeed_PID);
    Clear_PositionPIDData(&VisionPitchAttitude_PID);
    Clear_PositionPIDData(&VisionPitchSpeed_PID);
}

/**
 * @brief 清除云台电机PID缓冲值
 * 
 * @return  
 */
static void Cloud_ClearMoterPIDData(void)
{
    /*************清除Yaw轴****************/
    Clear_PositionPIDData(&M6020s_YawOPID);
    Clear_PositionPIDData(&M6020s_YawIPID);
    Clear_PositionPIDData(&VisionM6020s_YawOPID);
    Clear_PositionPIDData(&VisionM6020s_YawIPID);

    /*************清除Pitch轴****************/
    Clear_PositionPIDData(&M6020s_PitchOPID);
    Clear_PositionPIDData(&M6020s_PitchIPID);
    Clear_PositionPIDData(&VisionM6020s_PitchOPID);
    Clear_PositionPIDData(&VisionM6020s_PitchIPID);
}

/*****************云台陀螺仪函数*******************/
/**
 * @brief 云台陀螺仪YAW自瞄PID
 * 
 * @return  
 */
static void Cloud_YAWIMUVisionPID(float Angle, float Speed)
{
    float IMUVisionErr = Angle + VisionExportData.FinalOffset.x;
    float Vision_YawAngle = Cloud_VisionFusion(&All_VisionComYaw, &Vision_Yaw_speed_Struct, &yaw_kalman_filter, IMUVisionErr, Control_Vision_FUN.GetVisionHitMode(), VisionYaw);
    VisionYawAttitude_PID.Position_PID(&VisionYawAttitude_PID, Vision_YawAngle, Angle);
    M6020s_Yaw.outCurrent = VisionYawSpeed_PID.Position_PID(&VisionYawSpeed_PID, VisionYawAttitude_PID.pwm, Speed);
    // M6020s_Yaw.outCurrent = 0;
    Clear_PositionPIDData(&YawAttitude_PID);
    Clear_PositionPIDData(&YawSpeed_PID);
}

/**
 * @brief 云台陀螺仪YAW正常行走PID
 * 
 * @return  
 */
static void Cloud_YAWIMUNormalPID(float Angle, float Speed)
{
    float AngleErr = Angle - Cloud.IMUtargetYawLPF * M6020_mAngleRatio;
    YawAttitude_PID.Position_PID(&YawAttitude_PID, 0, AngleErr);
    M6020s_Yaw.outCurrent = YawSpeed_PID.Position_PID(&YawSpeed_PID, YawAttitude_PID.pwm, Speed);
    Clear_VisionFusion(&All_VisionComYaw, &yaw_kalman_filter, Angle);
    Clear_PositionPIDData(&VisionYawAttitude_PID);
    Clear_PositionPIDData(&VisionYawSpeed_PID);
}

/**
 * @brief 云台陀螺仪Pitch自瞄PID
 * 
 * @return  
 */
static void Cloud_PITCHIMUVisionPID(float Angle, float Speed)
{
    float IMUVisionErr = Angle + VisionExportData.FinalOffset.y;
    float Vision_PitchAngle = Cloud_VisionFusion(&All_VisionComPitch, &Vision_Pitch_speed_Struct, &pitch_kalman_filter, IMUVisionErr, No_Compensate, VisionPitch);
    VisionPitchAttitude_PID.Position_PID(&VisionPitchAttitude_PID, Vision_PitchAngle, Angle);
    M6020s_Pitch.outCurrent = VisionPitchSpeed_PID.Position_PID(&VisionPitchSpeed_PID, VisionPitchAttitude_PID.pwm, Speed);
    Clear_PositionPIDData(&PitchAttitude_PID);
    Clear_PositionPIDData(&PitchSpeed_PID);
}

/**
 * @brief 云台陀螺仪Pitch正常行走PID
 * 
 * @return  
 */
static void Cloud_PITCHIMUNormalPID(float Angle, float Speed)
{
    float AngleErr = Angle - Cloud.IMUtargetPitchLPF;
    PitchAttitude_PID.Position_PID(&PitchAttitude_PID, 0, AngleErr);
    M6020s_Pitch.outCurrent = PitchSpeed_PID.Position_PID(&PitchSpeed_PID, PitchAttitude_PID.pwm, Speed);
    Clear_VisionFusion(&All_VisionComPitch, &pitch_kalman_filter, Angle);
    Clear_PositionPIDData(&VisionPitchAttitude_PID);
    Clear_PositionPIDData(&VisionPitchSpeed_PID);
}
/*****************云台陀螺仪函数 END*******************/

/*****************云台电机函数******************/
/**
 * @brief 云台电机YAW自瞄PID
 * 
 */
static void Cloud_YAWMotorVisionPID(float Angle, float Speed)
{
    float IMUVisionErr = Angle + VisionExportData.FinalOffset.x;
    float Vision_YawAngle = Cloud_VisionFusion(&All_VisionComYaw, &Vision_Yaw_speed_Struct, &yaw_kalman_filter, IMUVisionErr, Control_Vision_FUN.GetVisionHitMode(), VisionYaw);
    VisionM6020s_YawOPID.Position_PID(&VisionM6020s_YawOPID, Vision_YawAngle, Angle);
    M6020s_Yaw.outCurrent = VisionM6020s_YawIPID.Position_PID(&VisionM6020s_YawIPID, VisionM6020s_YawOPID.pwm, Speed);
    Clear_PositionPIDData(&M6020s_YawOPID);
    Clear_PositionPIDData(&M6020s_YawIPID);
}

/**
 * @brief 云台电机YAW正常行走PID
 * 
 */
static void Cloud_YAWMotorNormalPID(float Angle, float Speed)
{
    float AngleErr = Angle - Cloud.targetYawLPF;
    AngleErr = KalmanFilter(&Cloud_YawMotorAngle_Error_Kalman, AngleErr);
    M6020s_YawOPID.Position_PID(&M6020s_YawOPID, 0, AngleErr);
    M6020s_Yaw.outCurrent = M6020s_YawIPID.Position_PID(&M6020s_YawIPID, M6020s_YawOPID.pwm, Speed);
    Clear_VisionFusion(&All_VisionComYaw, &yaw_kalman_filter, Angle);
    Clear_PositionPIDData(&VisionM6020s_YawOPID);
    Clear_PositionPIDData(&VisionM6020s_YawIPID);
}

/**
 * @brief 云台电机PITCH自瞄PID
 * 
 */
static void Cloud_PITCHMotorVisionPID(float Angle, float Speed)
{
    float IMUVisionErr = Angle + VisionExportData.FinalOffset.y;
    float Vision_PitchAngle = Cloud_VisionFusion(&All_VisionComPitch, &Vision_Pitch_speed_Struct, &pitch_kalman_filter, IMUVisionErr, No_Compensate, VisionPitch);
    VisionM6020s_PitchOPID.Position_PID(&VisionM6020s_PitchOPID, Vision_PitchAngle, Angle);
    M6020s_Pitch.outCurrent = VisionM6020s_PitchIPID.Position_PID(&VisionM6020s_PitchIPID, VisionM6020s_PitchOPID.pwm, Speed);
    Clear_PositionPIDData(&M6020s_PitchOPID);
    Clear_PositionPIDData(&M6020s_PitchIPID);
}

/**
 * @brief 云台电机PITCH正常行走PID
 * 
 */
static void Cloud_PITCHMotorNormalPID(float Angle, float Speed)
{
    float AngleErr = Angle - Cloud.targetPitchLPF;
    AngleErr = KalmanFilter(&Cloud_PitchMotorAngle_Error_Kalman, AngleErr);
    M6020s_PitchOPID.Position_PID(&M6020s_PitchOPID, 0, AngleErr);
    M6020s_Pitch.outCurrent = M6020s_PitchIPID.Position_PID(&M6020s_PitchIPID, M6020s_PitchOPID.pwm, Speed);
    Clear_VisionFusion(&All_VisionComPitch, &pitch_kalman_filter, Angle);
    Clear_PositionPIDData(&VisionM6020s_PitchOPID);
    Clear_PositionPIDData(&VisionM6020s_PitchIPID);
}
/*****************云台电机函数 END******************/

/**
 * @brief 获取陀螺仪误差值
 * 
 * @param delta_pitch 
 * @return  
 */
float IMUPitch_Error(float delta_pitch)
{
    float Error = 0;
    if (fabs(delta_pitch) != 0.0f)
    {
        IMUExport_Cloud.Target_pitch = IMUExport_Cloud.Finally_totalPitch * M6020_mAngleRatio;
        Error = 0.0f;
    }
    else
    {
        Error = IMUExport_Cloud.Target_pitch - IMUExport_Cloud.Finally_totalPitch * M6020_mAngleRatio;
    }
    return Error;
}

/**
	* @brief  云台控制处理-带IMU模式
	* @param[in]  delta_yaw		航向角变化量
	*				delta_pitch		俯仰角变化量
	*				shoot	射击使能
	*				mode	模式 - 除Status_ControlOFF外，其他正常控制
	* @retval None
	*/

void Cloud_processing_WithIMU(float delta_yaw, float delta_pitch)
{
    M6020_Fun.M6020_Reset(&M6020s_Yaw);
    Cloud.targetYawLPF = Cloud.targetYawRaw = M6020s_Yaw.totalAngle;
    Cloud.targetPitchLPF = Cloud.targetPitchRaw = M6020s_Pitch.totalAngle;
    Cloud_ClearMoterPIDData();

    if (Robot.Sport_CloudWorkMode == CloudWorkMode_Disable)
    {
        //以当前位置为下次启动位置，防炸
        Cloud_setIMUPosForced(*Cloud.RealData.IMUAngle_Yaw, *Cloud.RealData.IMUAngle_Pitch);
        M6020s_Yaw.outCurrent = M6020s_Pitch.outCurrent = 0;
        Cloud_ClearIMUPIDData();
        return;
    }

    if (DR16_Export_Data.OffLineFlag /*|| Robot.Sport_ChassisWorkMode == ChassisWorkMode_Spin*/)
    {
        delta_yaw = delta_pitch = 0.0f;
        //fric_turnOff();
    }

    if (Robot.Sport_AttachWorkMode == AttachWorkMode_Slow)
    {
        delta_yaw = delta_yaw / 2.5f;
    }

    /*************传入增加量****************/
    Cloud.IMUtargetYawRaw += delta_yaw;
    Cloud.IMUtargetPitchRaw += delta_pitch;

    ///*************上下限幅****************/
    Cloud_AngleLimit(&Cloud.IMUtargetPitchRaw, Cloud_Pitch_Min, Cloud_Pitch_Max);
    /*************云台限幅end****************/

    /*************平滑处理****************/
    //Cloud.IMUtargetYawLPF = Cloud.IMUtargetYawRaw;
    //Cloud.IMUtargetPitchLPF = Cloud.IMUtargetPitchRaw;
    Filter_IIRLPF(&Cloud.IMUtargetYawRaw, &Cloud.IMUtargetYawLPF, Cloud.YawLpfAttFactor); //0.4
    Filter_IIRLPF(&Cloud.IMUtargetPitchRaw, &Cloud.IMUtargetPitchLPF, Cloud.PitchLpfAttFactor);

    /*************自瞄控制****************/
    //视觉与控制的转换 Yaw
    if (Robot.VisionEnabled == true && VisionData.OffLineFlag == 0 && Control_Vision_FUN.GetVisionDiscMode() == 1)
    {
        Cloud.IMUtargetYawLPF = Cloud.IMUtargetYawRaw = *Cloud.RealData.IMUAngle_Yaw;
        Cloud_YAWIMUVisionPID(*Cloud.RealData.IMUAngle_Yaw * M6020_mAngleRatio, *Cloud.RealData.IMUSpeed_Yaw);
    }
    else
    {
#if Cloud_DeBug == Cloud_YesDeBug
        YawAttitude_PID.Position_PID(&YawAttitude_PID, Cloud_ParameterDeBug[0].Test_angle * M6020_mAngleRatio, *Cloud.RealData.IMUAngle_Yaw * M6020_mAngleRatio);
        M6020s_Yaw.outCurrent = YawSpeed_PID.Position_PID(&YawSpeed_PID, YawAttitude_PID.pwm, *Cloud.RealData.IMUSpeed_Yaw);
        // M6020s_Yaw.outCurrent = testSpeed_PID.Position_PID(&testSpeed_PID, Cloud_ParameterDeBug.Test_Speed, mpu_data.gz);
#elif Cloud_DeBug == Cloud_NoDeBug
        Cloud_YAWIMUNormalPID(*Cloud.RealData.IMUAngle_Yaw * M6020_mAngleRatio, *Cloud.RealData.IMUSpeed_Yaw);
#endif
    }

    if (Robot.VisionEnabled == true && VisionData.OffLineFlag == 0 && Control_Vision_FUN.GetVisionDiscMode() == 1)
    {
        Cloud.IMUtargetPitchLPF = Cloud.IMUtargetPitchRaw = *Cloud.RealData.IMUAngle_Pitch;
        Cloud_PITCHIMUVisionPID(*Cloud.RealData.IMUAngle_Pitch, *Cloud.RealData.IMUSpeed_Pitch);
    }
    else
    {
#if Cloud_DeBug == Cloud_YesDeBug
        PitchAttitude_PID.Position_PID(&PitchAttitude_PID, Cloud_ParameterDeBug[1].Test_angle * M6020_mAngleRatio, *Cloud.RealData.IMUAngle_Pitch);
        M6020s_Pitch.outCurrent = PitchSpeed_PID.Position_PID(&PitchSpeed_PID, PitchAttitude_PID.pwm, *Cloud.RealData.IMUSpeed_Pitch);
#elif Cloud_DeBug == Cloud_NoDeBug
        Cloud_PITCHIMUNormalPID(*Cloud.RealData.IMUAngle_Pitch, *Cloud.RealData.IMUSpeed_Pitch);
#endif
    }

    /*************PID计算****************/

    //清标志位
    M6020s_Yaw.InfoUpdateFlag = 0;

    //清标志位
    M6020s_Pitch.InfoUpdateFlag = 0;
}

/**
 * @brief  云台控制处理
 * @param[in]  delta_yaw		航向角变化量
 *				delta_pitch		俯仰角变化量
 *				shoot	射击使能
 *				mode	模式 - 除Status_ControlOFF外，其他正常控制
 * @retval None
 */
void Cloud_processing_WithoutIMU(float delta_yaw, float delta_pitch)
{
    DJI_IMUFUN.IMU_Reset(&imu_Export);
    DJI_C_IMUFUN.CIMU_Reset(&DJI_C_IMU);
    Cloud_setIMUPosForced(*Cloud.RealData.IMUAngle_Yaw, *Cloud.RealData.IMUAngle_Pitch);
    Cloud_ClearIMUPIDData();
    if (Robot.Sport_CloudWorkMode == CloudWorkMode_Disable)
    {
        //以当前位置为下次启动位置，防炸
        Cloud.targetYawLPF = Cloud.targetYawRaw = M6020s_Yaw.totalAngle;
        Cloud.targetPitchLPF = Cloud.targetPitchRaw = M6020s_Pitch.totalAngle;
        M6020s_Yaw.outCurrent = M6020s_Pitch.outCurrent = 0;
        Cloud_ClearMoterPIDData();
        return;
    }

    if (DR16_Export_Data.OffLineFlag /*|| Robot.Sport_ChassisWorkMode == ChassisWorkMode_Spin*/)
    {
        delta_yaw = delta_pitch = 0.0f;
        //fric_turnOff();
    }

    if (Robot.Sport_AttachWorkMode == AttachWorkMode_Slow)
    {
        delta_yaw = delta_yaw / 2.5f;
    }

    /*************传入增加量****************/
    Cloud.targetYawRaw += -delta_yaw * M6020_mAngleRatio;
    Cloud.targetPitchRaw += delta_pitch;

    if (Robot.Sport_ChassisWorkMode == ChassisWorkMode_NewFollow)
    {
        Cloud.targetYawRaw = Cloud_Yaw_Center;
    }

    ///*************上下限幅****************/

    //PITCH为机械角度写死的限位，第一次使用前务必修改或注释！需要根据电机方向适当调整符号。
    Cloud_AngleLimit(&Cloud.targetPitchRaw, Cloud_Pitch_Min, Cloud_Pitch_Max);

    /*************云台限幅end****************/

    /*************平滑处理****************/
    //Cloud.IMUtargetYawLPF = Cloud.IMUtargetYawRaw;
    //Cloud.targetPitchLPF = Cloud.targetPitchRaw;
    Filter_IIRLPF(&Cloud.targetYawRaw, &Cloud.targetYawLPF, Cloud.YawLpfAttFactor); //0.4
    Filter_IIRLPF(&Cloud.targetPitchRaw, &Cloud.targetPitchLPF, Cloud.PitchLpfAttFactor);

    M6020_Fun.M6020_setTargetAngle(&M6020s_Yaw, Cloud.targetYawLPF);
    M6020_Fun.M6020_setTargetAngle(&M6020s_Pitch, Cloud.targetPitchLPF);

    /*************自瞄控制****************/
    //视觉与控制的转换 Yaw
    if (Robot.VisionEnabled == true && VisionData.OffLineFlag == 0 && Control_Vision_FUN.GetVisionDiscMode() == 1)
    {
        Cloud.targetYawLPF = Cloud.targetYawRaw = M6020s_Yaw.totalAngle;
        Cloud_YAWMotorVisionPID(M6020s_Yaw.totalAngle, M6020s_Yaw.realSpeed);
    }
    else
    {
#if Cloud_DeBug == Cloud_YesDeBug
        M6020s_YawOPID.Position_PID(&M6020s_YawOPID, Cloud_ParameterDeBug[0].Test_angle * M6020_mAngleRatio, M6020s_Yaw.totalAngle);
        M6020s_Yaw.outCurrent = M6020s_YawIPID.Position_PID(&M6020s_YawIPID, M6020s_YawOPID.pwm, M6020s_Yaw.realSpeed);
        M6020s_Yaw.outCurrent = 0;
#elif Cloud_DeBug == Cloud_NoDeBug
        Cloud_YAWMotorNormalPID(M6020s_Yaw.totalAngle, M6020s_Yaw.realSpeed);
#endif
    }

    if (Robot.VisionEnabled == true && VisionData.OffLineFlag == 0 && Control_Vision_FUN.GetVisionDiscMode() == 1)
    {
        Cloud.targetPitchLPF = Cloud.targetPitchRaw = M6020s_Pitch.totalAngle;
        Cloud_PITCHMotorVisionPID(M6020s_Pitch.totalAngle, M6020s_Pitch.realSpeed);
    }
    else
    {
#if Cloud_DeBug == Cloud_YesDeBug
        M6020s_PitchOPID.Position_PID(&M6020s_PitchOPID, Cloud_ParameterDeBug[1].Test_angle * M6020_mAngleRatio, M6020s_Pitch.totalAngle);
        M6020s_Pitch.outCurrent = M6020s_PitchIPID.Position_PID(&M6020s_PitchIPID, M6020s_PitchOPID.pwm, M6020s_Pitch.realSpeed);
#elif Cloud_DeBug == Cloud_NoDeBug
        Cloud_PITCHMotorNormalPID(M6020s_Pitch.totalAngle, M6020s_Pitch.realSpeed);
#endif
    }

    //清标志位
    M6020s_Yaw.InfoUpdateFlag = 0;

    //清标志位
    M6020s_Pitch.InfoUpdateFlag = 0;
}
