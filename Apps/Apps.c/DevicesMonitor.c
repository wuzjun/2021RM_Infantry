/**
 * @file DevicesMonitor.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "DevicesMonitor.h"
#include "BSP_CAN.h"
#include "Chassis_control.h"
#include "Control_Vision.h"
#include "DJI_C_IMU.h"
#include "DJI_IMU.h"
#include "DR16_Remote.h"
#include "M2006_Motor.h"
#include "M3508_Motor.h"
#include "M6020_Motor.h"
#include "Robot_control.h"
#include "SuperCapacitor.h"
#include "Wolf_GyIMU.h"
#include "cmsis_os.h"

const int Buzzer_DefaultPWM = 270; //蜂鸣器报警的音量，270最大。
/************************************************************************/
/* 模块离线检测思路：
	  特殊的：遥控器离线 == 机器人进入失能模式。
	  其他：部分设备离线 == 将该功能模块进行失能或切换。
	  如：自瞄识别、陀螺仪、裁判系统、
  */
/************************************************************************/
void DevicesMonitor_update(void);
void DevicesMonitor_Init(void);
void Buzzer_On(bool on, int volume);
void DevicesMonitor_Alert(void);
void DevicesInit(void);
DevicesMonitor_FUN_t DevicesMonitor_FUN = DevicesMonitor_FUNGroundInit;
#undef DevicesMonitor_FUNGroundInit
Check_Fun_t Check_Fun[CHECK_LIST_Device] = {NULL};

static uint32_t
FPS_Calculate(uint16_t deltaTime)
{                                                  //FPS计算函数。
    return (1.0f / (double)(deltaTime)) * 1000.0f; // 别忘了先转换为浮点数，否则会有精度丢失
}

void Get_FPS(WorldTime_RxTypedef *time, uint32_t *FPS) //获取当前系统时钟节拍并算出FPS
{
    time->WorldTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    *FPS = FPS_Calculate(time->WorldTime - time->Last_WorldTime);
    time->Last_WorldTime = time->WorldTime;
}

void DevicesInit(void)
{
    void *Check_Funnames[CHECK_LIST_Device] = {
        M3508_FUN.Check_WheelM3508,
        DR16_Fun.Check_DR16,
        SuperCapacitorFUN.Check_SuperCapacitor,
        Wolf_GyIMU_Fun.Check_GyIMU,
        M6020_Fun.Check_M6020,
        DJI_IMUFUN.Check_DJIIMU,
        DJI_C_IMUFUN.Check_DJI_C_IMU,
        M2006_FUN.Check_M2006,
        M3508_FUN.Check_FrictionM3508,
        Control_Vision_FUN.Check_Vision,
        Can_Fun.Check_CAN,
        JudgeSystem_FUN.Check_Judge,
    };
    for (uint8_t i = 0; i < CHECK_LIST_Device; i++)
    {
        Check_Fun[i].GetDevices = (void (*)(void))Check_Funnames[i];
    }
}

/**
  * @brief  外接设备帧率清零，每200ms清零一次
  * @param  None
  * @retval None
  */
void DevicesMonitor_update(void)
{
    /********************检测开始********************/
    for (uint8_t i = 0; i < CHECK_LIST_Device; i++)
    {
        Check_Fun[i].GetDevices();
    }

    /*****************特殊标志位检测*****************/
    if (imu_Export.OffLineFlag)
    {
        ReversefollowFlag.NewFollowFlag = 1;
    }

    if (DR16_Export_Data.OffLineFlag)
    {
        //全局失能
        Robot_control_FUN.Robot_Disable();
    }
    else
    {
        Robot_control_FUN.Robot_Enable();
    }
}

/**
 * @brief 蜂鸣器初始化
 * 
 * @return  
 */
void DevicesMonitor_Init(void)
{

    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
}

/**
 * @brief 控制蜂鸣器的开关接口。
 * 
 * @param on 
 * @return  
 */
void Buzzer_On(bool on, int volume)
{
    int pwm = volume;
    if (on == 0)
    {
        pwm = 0;
    }

    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, pwm); //开弹仓。
}

/**
 * @brief 报警函数
 * 
 * @return  
 */
int WorkStatus_AlertCount = 0;
void DevicesMonitor_Alert(void)
{
    WorkStatus_AlertCount++;
    Buzzer_On(false, Buzzer_DefaultPWM);

    if (Robot.WorkStatus != WorkStatus_Normal)
    {
        switch (WorkStatus_AlertCount)
        {
        case 1:
            Buzzer_On(true, Buzzer_DefaultPWM);
            break;
        case 2:
            Buzzer_On(false, Buzzer_DefaultPWM);
            break;
        default:
            WorkStatus_AlertCount = 0;
        }
    }
}
