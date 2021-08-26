/**
 * @file Task_DataScope.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Task_DataScope.h"
#include "BSP_CAN.h"
#include "Chassis_control.h"
#include "Cloud_control.h"
#include "Control_Vision.h"
#include "DJI_C_IMU.h"
#include "DJI_IMU.h"
#include "DR16_Remote.h"
#include "IMU_Compensate.h"
#include "M3508_Motor.h"
#include "M6020_Motor.h"
#include "RM_JudgeSystem.h"
#include "Shoot.h"
#include "Status_Update.h"
#include "Task_CanMsg.h"
#include "Task_Sampling.h"
#include "Wolf_GyIMU.h"

/**
  * @Data   2021-03-28
  * @brief  上位机调试
  * @param  void
  * @retval void
  */
void Test(void const *argument)
{
    for (;;)
    {
        //ANO_FUN.ANO_Send_FloatData_2(FollowYaw_pid.pwm, ext_power_heat_data.data.chassis_power_buffer);
        // ANO_FUN.ANO_Send_FloatData_2(FricOffset[0], shootUnit1.FricUpdata);
        ANO_FUN.ANO_Send_FloatData_2(VisionData.RawData.x, VisionData.FPS);

        osDelay(5);
    }
}
