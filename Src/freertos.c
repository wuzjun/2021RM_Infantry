/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BSP_ADC.h"
#include "BSP_CAN.h"
#include "BSP_TIM.h"
#include "BSP_USART.h"
#include "Chassis_control.h"
#include "Cloud_control.h"
#include "Control_Vision.h"
#include "DJI_C_IMU.h"
#include "DJI_IMU.h"
#include "DR16_Remote.h"
#include "DevicesMonitor.h"
#include "Flash_Store.h"
#include "IMU_Compensate.h"
#include "OLED.h"
#include "PVD.h"
#include "RMClient_UI.h"
#include "RM_JudgeSystem.h"
#include "Robot_Config.h"
#include "Robot_control.h"
#include "Shoot.h"
#include "Wolf_GyIMU.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId Task_ControlHandle;

/* USER CODE END Variables */
osThreadId Task_ConditionHandle;
osThreadId Task_Can1MsgRecHandle;
osThreadId Task_Can2MsgRecHandle;
osThreadId Task_SamplingHandle;
osThreadId Task_USARTHandle;
osThreadId Task_InitHandle;
osThreadId Task_MonitorHandle;
osThreadId Task_OledHandle;
osThreadId Task_CanSendHandle;
osMessageQId CAN1_ReceiveHandle;
osMessageQId CAN2_ReceiveHandle;
osMessageQId CAN_SendHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void RobotControl(void const *argument);

/* USER CODE END FunctionPrototypes */

void ALL_Send(void const *argument);
extern void Can1Receives(void const *argument);
extern void Can2Receives(void const *argument);
extern void Fixed_Sampling(void const *argument);
extern void Test(void const *argument);
void All_Init(void const *argument);
extern void Detect(void const *argument);
extern void DJI_OLED(void const *argument);
extern void AllCanSend(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* definition and creation of CAN1_Receive */
    osMessageQDef(CAN1_Receive, 32, Can_Export_Data_t);
    CAN1_ReceiveHandle = osMessageCreate(osMessageQ(CAN1_Receive), NULL);

    /* definition and creation of CAN2_Receive */
    osMessageQDef(CAN2_Receive, 32, Can_Export_Data_t);
    CAN2_ReceiveHandle = osMessageCreate(osMessageQ(CAN2_Receive), NULL);

    /* definition and creation of CAN_Send */
    osMessageQDef(CAN_Send, 32, Can_Send_Data_t);
    CAN_SendHandle = osMessageCreate(osMessageQ(CAN_Send), NULL);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* definition and creation of Task_Condition */
    osThreadDef(Task_Condition, ALL_Send, osPriorityBelowNormal, 0, 128);
    Task_ConditionHandle = osThreadCreate(osThread(Task_Condition), NULL);

    /* definition and creation of Task_Can1MsgRec */
    osThreadDef(Task_Can1MsgRec, Can1Receives, osPriorityHigh, 0, 256);
    Task_Can1MsgRecHandle = osThreadCreate(osThread(Task_Can1MsgRec), NULL);

    /* definition and creation of Task_Can2MsgRec */
    osThreadDef(Task_Can2MsgRec, Can2Receives, osPriorityHigh, 0, 256);
    Task_Can2MsgRecHandle = osThreadCreate(osThread(Task_Can2MsgRec), NULL);

    /* definition and creation of Task_Sampling */
    osThreadDef(Task_Sampling, Fixed_Sampling, osPriorityHigh, 0, 384);
    Task_SamplingHandle = osThreadCreate(osThread(Task_Sampling), NULL);

    /* definition and creation of Task_USART */
    osThreadDef(Task_USART, Test, osPriorityNormal, 0, 128);
    Task_USARTHandle = osThreadCreate(osThread(Task_USART), NULL);

    /* definition and creation of Task_Init */
    osThreadDef(Task_Init, All_Init, osPriorityNormal, 0, 128);
    Task_InitHandle = osThreadCreate(osThread(Task_Init), NULL);

    /* definition and creation of Task_Monitor */
    osThreadDef(Task_Monitor, Detect, osPriorityAboveNormal, 0, 128);
    Task_MonitorHandle = osThreadCreate(osThread(Task_Monitor), NULL);

    /* definition and creation of Task_Oled */
    osThreadDef(Task_Oled, DJI_OLED, osPriorityBelowNormal, 0, 128);
    Task_OledHandle = osThreadCreate(osThread(Task_Oled), NULL);

    /* definition and creation of Task_CanSend */
    osThreadDef(Task_CanSend, AllCanSend, osPriorityHigh, 0, 256);
    Task_CanSendHandle = osThreadCreate(osThread(Task_CanSend), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_ALL_Send */
/**
  * @brief  Function implementing the Task_Condition thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ALL_Send */
void ALL_Send(void const *argument)
{
    /* USER CODE BEGIN ALL_Send */
    /* Infinite loop */
    for (;;)
    {
        //视觉数据发送
        Control_Vision_FUN.Update_VisionTarget();

        UserDefined_UI();

        osDelay(1);
    }
    /* USER CODE END ALL_Send */
}

/* USER CODE BEGIN Header_All_Init */
/**
* @brief Function implementing the Task_Init thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_All_Init */
void All_Init(void const *argument)
{
    /* USER CODE BEGIN All_Init */
    /* Infinite loop */
    Robot.WorkStatus = WorkStatus_Disable;
    DevicesMonitor_FUN.DevicesMonitor_Init();
    //PWR初始化
    PVD_Config();
    //A板陀螺仪温控初始化
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    IMU_CompensateFUN.Preserve_temp(/* ADC_FUN.get_temprate() */ 40); //温度校准
    DJI_IMUFUN.DJI_IMU_Init();
#if Calibrate == Calibrate_On
    DJI_IMUWriteOffset(IMUwriteFlashData, IMUOffsetNum);
#endif
    // 摩擦轮初始化
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
#if Infantry_Year == Infantry_2020
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 1720); //关弹仓。
#elif Infantry_Year == Infantry_2021
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 2050); //关弹仓。
#endif
    // 串口1初始化
    DR16_Fun.DR16_USART_Receive_DMA(&huart1);
    //CAN中断初始化
    Can_Fun.CAN_IT_Init(&hcan1, Can1_Type);
    Can_Fun.CAN_IT_Init(&hcan2, Can2_Type);
    //串口3初始化
    JudgeSystem_FUN.JudgeSystem_USART_Receive_DMA(&huart3);
    // 上位机初始化
    USART_Fun.DMA_USART_Send_IT_Init(&huart6);
    // 串口7初始化
    Control_Vision_FUN.Vision_USART_Receive_DMA(&huart7);

#ifdef Temporary_Function
    // OLED初始化
    oled_init();
#endif
    DevicesMonitor_FUN.Buzzer_On(true,270);
    HAL_Delay(1000);
    DevicesMonitor_FUN.Buzzer_On(false,270);
    HAL_Delay(1000);
    DevicesMonitor_FUN.Buzzer_On(true,270);
    HAL_Delay(1000);
    DevicesMonitor_FUN.Buzzer_On(false,270);
    Shoot_Fun.FricRead(); //读取Flash数据或扇区擦除功能

    taskENTER_CRITICAL(); //进入临界区
    Robot_control_FUN.Robot_init();
    Control_Vision_FUN.Vision_Init();
    Wolf_GyIMU_Fun.GY6050_Init(&IMUExport_Cloud, &IMUExport_Chassis);
    Chassis_FUN.Chassis_Init();
    Cloud_FUN.Cloud_Init();
    Shoot_Fun.Shoot_Init();
    Wolf_GyIMU_Fun.GY6050_Init(&IMUExport_Cloud, &IMUExport_Chassis);

    osThreadDef(Task_Control, RobotControl, osPriorityRealtime, 0, 600);
    Task_ControlHandle = osThreadCreate(osThread(Task_Control), NULL);

    Robot.WorkStatus = WorkStatus_Normal; //初始化完成，机器人从这里之后才能动。
    RoboInit_Complete = 1;                //初始化结束
    vTaskDelete(NULL);                    //删除当前任务。
    taskEXIT_CRITICAL();                  //退出临界区
                                          /* USER CODE END All_Init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
