#ifndef SUPERCAPACITOR_H
#define SUPERCAPACITOR_H

#include "can.h"
#include "typedef.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SCCM_RECEIVE_ID 0x600
#define SCCM_SEND_ID 0x601

#define Charging_ON 1
#define Charging_OFF 0

#define Power_Supply 1
#define Power_NotSupply 0

#pragma anon_unions

#define SuperCapacitorFUN_Init      \
    {                               \
        &SCCM_MsgProcess,           \
            &SCCM_SendMsg,          \
            &SuperCapacitor_Switch, \
            &Check_SuperCapacitor,  \
    }

typedef union
{
    uint8_t data[8];
    struct
    {
        float chassis_power;  /* 底盘功率，单位：W */
        uint8_t chassis_buff; /* 底盘功率缓冲 */
        uint8_t cap_usable;   /* 电容可以进行输出 */
        uint8_t cap_cell;     /* 电容剩余电量，会出现负数 */
    };
} SCCM_ReceiveData_t;

typedef union
{
    uint8_t data[8];
    struct
    {
        float charge_power;    /* 充电功率，单位：W ,范围 0-80W */
        uint8_t charge_enable; /* 充电使能 */
        uint8_t is_cap_output; /* 使用电容供电 */
    };
} SCCM_SendData_t;

typedef struct
{
    SCCM_ReceiveData_t ReceiveData;
    SCCM_SendData_t SendData;

    bool EnableCapacitor;
    uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志
} Supercapacitor_t;

typedef struct
{
    void (*SCCM_MsgProcess)(Can_Export_Data_t RxMessage);
    void (*SCCM_SendMsg)(float Charge_power, uint8_t Charge_enable, uint8_t Cap_enable);
    void (*SuperCapacitor_Switch)(bool state);
    void (*Check_SuperCapacitor)(void);
} SuperCapacitorFUN_t;

extern uint8_t Cap_Tick_5MS;
extern Supercapacitor_t supercapacitor;
extern SuperCapacitorFUN_t SuperCapacitorFUN;

#endif
