/**
 * @file PID.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "PID.h"

extKalman_t Cloud_YAWODKalman;
extKalman_t Cloud_PITCHODKalman;

static void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}

float Incremental_PID(incrementalpid_t *pid_t, float target, float measured)
{

    pid_t->Target = target;
    pid_t->Measured = measured;
    pid_t->err = pid_t->Target - pid_t->Measured;

    //	if(abs(pid_t->err)<0.1f)
    //		pid_t->err = 0.0f;
    //return 0;

    pid_t->p_out = pid_t->Kp * (pid_t->err - pid_t->err_last);
    pid_t->i_out = pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - 2.0f * pid_t->err_last + pid_t->err_beforeLast);

    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit);

    pid_t->pwm += (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_beforeLast = pid_t->err_last;
    pid_t->err_last = pid_t->err;

    return pid_t->pwm;
}

void Incremental_PIDInit(incrementalpid_t *pid_t, float Kp, float Ki, float Kd, uint32_t MaxOutput, uint32_t IntegralLimit)
{
    pid_t->Kp = Kp;
    pid_t->Ki = Ki;
    pid_t->Kd = Kd;
    pid_t->MaxOutput = MaxOutput;
    pid_t->IntegralLimit = IntegralLimit;
    pid_t->p_out = 0;
    pid_t->d_out = 0;
    pid_t->i_out = 0;
    pid_t->err = 0;
    pid_t->err_last = 0;
    pid_t->err_beforeLast = 0;
    pid_t->pwm = 0;
    pid_t->Measured = 0;
    pid_t->Target = 0;
}

void Clear_IncrementalPIDData(incrementalpid_t *pid_t)
{
    pid_t->Target = 0;
    pid_t->Measured = 0;
    pid_t->err = 0;
    pid_t->err_last = 0;
    pid_t->err_beforeLast = 0;
    pid_t->p_out = 0;
    pid_t->i_out = 0;
    pid_t->d_out = 0;
    pid_t->pwm = 0;
}

//位置式PID算法，对偏差值进行累加积分。
float Position_PID(positionpid_t *pid_t, float target, float measured)
{

    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

void Position_PIDInit(positionpid_t *pid_t, float Kp, float Ki, float Kd, float MaxOutput, float Integral_Separation, float IntegralLimit)
{
    pid_t->Kp = Kp;
    pid_t->Ki = Ki;
    pid_t->Kd = Kd;
    pid_t->MaxOutput = MaxOutput;
    pid_t->Integral_Separation = Integral_Separation;
    pid_t->IntegralLimit = IntegralLimit;
    pid_t->p_out = 0;
    pid_t->d_out = 0;
    pid_t->i_out = 0;
    pid_t->err = 0;
    pid_t->err_last = 0;
    pid_t->err_change = 0;
    pid_t->pwm = 0;
    pid_t->Measured = 0;
    pid_t->Target = 0;
}

void Clear_PositionPIDData(positionpid_t *pid_t)
{
    pid_t->Target = 0;
    pid_t->Measured = 0;
    pid_t->err = 0;
    pid_t->err_change = 0;
    pid_t->err_last = 0;
    pid_t->p_out = 0;
    pid_t->i_out = 0;
    pid_t->d_out = 0;
    pid_t->pwm = 0;
}

/******************************以下是专属PID*********************************/

/**********************底盘跟随PID**********************/
int AngleTs[5] = {100, 90, 60, 30, 23};                   //角度分段阈值
float FollowKp[6] = {2500, 2700, 3500, 4800, 7000, 9000}; //跟随Kp变化值
float FollowFactor[4] = {0.75f, 1.0f, 0.7f, 0.65f};       //跟随Kp变化因子
//底盘跟随PID
float ClassisFollow_PID(positionpid_t *pid_t, float target, float measured)
{
    if (fabs(target) > AngleTs[0])
    {
        pid_t->Kp = FollowKp[0];
    }
    else if (fabs(target) > AngleTs[1])
    {
        pid_t->Kp = FollowKp[1];
    }
    else if (fabs(target) > AngleTs[2])
    {
        pid_t->Kp = FollowKp[2];
    }
    else if (fabs(target) > AngleTs[3])
    {
        pid_t->Kp = FollowKp[3];
    }
    else if (fabs(target) > AngleTs[4])
    {
        pid_t->Kp = FollowKp[4];
    }
    else
    {
        pid_t->Kp = FollowKp[5];
    }

    if (measured >= 5100)
    {
        pid_t->Kp *= FollowFactor[0];
    }
    else if (measured >= 4900)
    {
        pid_t->Kp *= FollowFactor[1];
    }
    else if (measured >= 3800)
    {
        pid_t->Kp *= FollowFactor[2];
    }
    else if (measured >= 3000)
    {
        pid_t->Kp *= FollowFactor[3];
    }
    else
    {
        pid_t->Kp *= pid_t->Integral_Separation;
    }

    target = target * (PI / 180);

    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

//自旋+扭腰
float ClassisTwister_PID(positionpid_t *pid_t, float target, float measured)
{
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}
/******************************************************/

/**********************视觉PID**********************/

float Vision_AutoTracPID(positionpid_t *pid_t, float target, float measured)
{
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 视觉YAW轴外环PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Vision_YAWOPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 视觉YAW轴内环PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Vision_YAWIPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    if (abs(pid_t->err) >= pid_t->Integral_Separation)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 视觉PITCH轴外环PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Vision_PITCHOPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 视觉PITCH轴内环PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Vision_PITCHIPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    if (abs(pid_t->err) >= pid_t->Integral_Separation)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}
/**************************************************/
/********************云台PID***********************/
/**
 * @brief 云台YAW轴外环PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Cloud_YAWOPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    pid_t->d_out = KalmanFilter(&Cloud_YAWODKalman, pid_t->d_out);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 云台YAW轴内环PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Cloud_YAWIPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    if (abs(pid_t->err) >= pid_t->Integral_Separation)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 云台PITCH轴外环PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Cloud_PITCHOPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    pid_t->d_out = KalmanFilter(&Cloud_PITCHODKalman, pid_t->d_out);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 云台PITCH轴内环PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Cloud_PITCHIPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    if (abs(pid_t->err) >= pid_t->Integral_Separation)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}
/***********************************************************************/

/********************云台PID***********************/
/**
 * @brief 云台YAW轴外环PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Cloud_IMUYAWOPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    pid_t->d_out = KalmanFilter(&Cloud_YAWODKalman, pid_t->d_out);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 云台YAW轴内环PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Cloud_IMUYAWIPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;
    // float Absolute_Measured = measured;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。
    if (abs(pid_t->err) >= pid_t->Integral_Separation)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 云台PITCH轴外环PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Cloud_IMUPITCHOPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    pid_t->d_out = KalmanFilter(&Cloud_PITCHODKalman, pid_t->d_out);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 云台PITCH轴内环PID
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Cloud_IMUPITCHIPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    if (abs(pid_t->err) >= pid_t->Integral_Separation)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/***********************************************************************/

/****************************IMU--Vision********************************/
/**
 * @brief 云台YAW轴外环PID(视觉--IMU)
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Cloud_VisionIMUYAWOPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);

    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 云台YAW轴内环PID(IMU--Vision)
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Cloud_VisionIMUYAWIPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;
    // float Absolute_Measured = measured;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。
    if (abs(pid_t->err) >= pid_t->Integral_Separation)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 云台PITCH轴外环PID(IMU--Vision)
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Cloud_VisionIMUPITCHOPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 云台PITCH轴内环PID(IMU--Vision)
 * 
 * @param pid_t 
 * @param target 
 * @param measured 
 */
float Cloud_VisionIMUPITCHIPID(positionpid_t *pid_t, float target, float measured)
{
    if (pid_t == NULL)
    {
        return 0;
    }
    pid_t->Target = (float)target;
    pid_t->Measured = (float)measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。
    if (abs(pid_t->err) >= pid_t->Integral_Separation)
    {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    }
    else
    {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}
/***********************************************************************/
