/**
 * @file Wolf_GyIMU.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Wolf_GyIMU.h"

void GY6050_Init(GY_IMUExport_t *IMUExport_Cloud, GY_IMUExport_t *IMUExport_Chassis);
void GY6050_getInfo(Can_Export_Data_t RxMessage);
void IMU_processing(GY_IMUExport_t *IMUExport_Cloud, GY_IMUExport_t *IMUExport_Chassis);
void Check_GyIMU(void);
float GY6050_PitErr(void);

GY_IMU_t IMU_Cloud;
GY_IMU_t IMU_Chassis;
GY_IMU_t *IMU_Data[] = {&IMU_Cloud, &IMU_Chassis};

Wolf_GyIMU_Fun_t Wolf_GyIMU_Fun = Wolf_GyIMU_FunGroundInit;
#undef Wolf_GyIMU_FunGroundInit
WorldTime_RxTypedef IMU_Cloud_Time[2];

float GY6050_PitErr(void)
{
    return (IMU_Data[1]->Calculating_Data.Target_pitch - IMU_Data[1]->Eular.Pitch);
}

/**
  * @brief  陀螺仪初始化
  * @param  GY_IMUExport_t IMUExport_Cloud, GY_IMUExport_t IMUExport_Chassis
  * @retval None
  */
void GY6050_Init(GY_IMUExport_t *IMUExport_Cloud, GY_IMUExport_t *IMUExport_Chassis)
{
    IMUExport_Cloud->Target_pitch = IMU_Data[0]->Calculating_Data.Target_pitch = IMU_Data[0]->Eular.Pitch;
    IMUExport_Chassis->Target_pitch = IMU_Data[1]->Calculating_Data.Target_pitch = IMU_Data[1]->Eular.Pitch;
}

/**
  * @brief  陀螺仪数据接收
  * @param  Can_Export_Data_t RxMessage
  * @retval None
  */
void GY6050_getInfo(Can_Export_Data_t RxMessage)
{
    uint32_t StdId;
    StdId = (int32_t)(RxMessage.CAN_RxHeader.StdId - GY_IMU_READID_START);

    memcpy(IMU_Data[StdId]->bno055_data.dataBuff, RxMessage.CANx_Export_RxMessage, sizeof(uint8_t[8]));
    IMU_Data[StdId]->Eular.Yaw = (float)IMU_Data[StdId]->bno055_data.yaw / 100.0f;
    IMU_Data[StdId]->Gyro.z = IMU_Data[StdId]->bno055_data.gyro_z / 16;
    IMU_Data[StdId]->Eular.Pitch = (float)IMU_Data[StdId]->bno055_data.pitch / 100.0f;
    IMU_Data[StdId]->Gyro.x = IMU_Data[StdId]->bno055_data.gyro_x / 16;

    if (abs(IMU_Data[StdId]->Gyro.z) < 2)
    {
        IMU_Data[StdId]->Gyro.z = IMU_Data[StdId]->bno055_data.gyro_z = 0;
    }

    if (IMU_Data[StdId]->Eular.Pitch - IMU_Data[StdId]->Calculating_Data.lastPitch < -300)
    { //经过跳变边沿。
        IMU_Data[StdId]->Calculating_Data.turnCount++;
    }
    if (IMU_Data[StdId]->Calculating_Data.lastPitch - IMU_Data[StdId]->Eular.Pitch < -300)
    {
        IMU_Data[StdId]->Calculating_Data.turnCount--;
    }

    IMU_Data[StdId]->Calculating_Data.totalPitch = IMU_Data[StdId]->Eular.Pitch + (360 * IMU_Data[StdId]->Calculating_Data.turnCount);

    IMU_Data[StdId]->Calculating_Data.lastPitch = IMU_Data[StdId]->Eular.Pitch;

    IMU_Data[StdId]->InfoUpdateFrame++;
    IMU_Data[StdId]->InfoUpdateFlag = 1;

    Get_FPS(&IMU_Cloud_Time[StdId], &IMU_Data[StdId]->IMU_FPS);
}

/**
  * @brief  云台陀螺仪数据输出函数
  * @param  GY_IMUExport_t IMUExport_Cloud
  * @retval None
  */
static void IMUCloud_processing(GY_IMUExport_t *IMUExport_Cloud)
{
    IMUExport_Cloud->Finally_Eular.Pitch = IMU_Data[0]->Eular.Pitch;
    IMUExport_Cloud->Finally_Eular.Roll = IMU_Data[0]->Eular.Roll;
    IMUExport_Cloud->Finally_Eular.Yaw = IMU_Data[0]->Eular.Yaw;

    IMUExport_Cloud->Finally_Gyro.x = IMU_Data[0]->Gyro.x;
    IMUExport_Cloud->Finally_Gyro.y = IMU_Data[0]->Gyro.y;
    IMUExport_Cloud->Finally_Gyro.z = IMU_Data[0]->Gyro.z;

    IMUExport_Cloud->Finally_totalPitch = IMU_Data[0]->Calculating_Data.totalPitch;
    IMUExport_Cloud->OffLineFlag = IMU_Data[0]->OffLineFlag;
}

/**
  * @brief  底盘陀螺仪数据输出函数
  * @param  GY_IMUExport_t IMUExport_Cloud
  * @retval None
  */
static void IMUChassis_processing(GY_IMUExport_t *IMUExport_Chassis)
{
    IMUExport_Chassis->Finally_Eular.Pitch = IMU_Data[1]->Eular.Pitch;
    IMUExport_Chassis->Finally_Eular.Roll = IMU_Data[1]->Eular.Roll;
    IMUExport_Chassis->Finally_Eular.Yaw = IMU_Data[1]->Eular.Yaw;

    IMUExport_Chassis->Finally_Gyro.x = IMU_Data[1]->Gyro.x;
    IMUExport_Chassis->Finally_Gyro.y = IMU_Data[1]->Gyro.y;
    IMUExport_Chassis->Finally_Gyro.z = IMU_Data[1]->Gyro.z;

    IMUExport_Chassis->Finally_totalPitch = IMU_Data[1]->Calculating_Data.totalPitch;
    IMUExport_Chassis->OffLineFlag = IMU_Data[1]->OffLineFlag;
}

/**
  * @brief  陀螺仪数据输出函数
  * @param  GY_IMUExport_t IMUExport_Cloud,GY_IMUExport_t IMUExport_Chassis
  * @retval None
  */
void IMU_processing(GY_IMUExport_t *IMUExport_Cloud, GY_IMUExport_t *IMUExport_Chassis)
{
    IMUCloud_processing(IMUExport_Cloud);
    IMUChassis_processing(IMUExport_Chassis);
}

void Check_GyIMU(void)
{
    for (uint8_t i = 0; i < 2; i++)
    {
        //底盘陀螺仪检测 ---------------------------
        if (IMU_Data[i]->InfoUpdateFrame < 1)
        {
            IMU_Data[i]->OffLineFlag = 1;
        }
        else
        {
            IMU_Data[i]->OffLineFlag = 0;
        }
        IMU_Data[i]->InfoUpdateFrame = 0;
    }
}
