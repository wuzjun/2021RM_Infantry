/**
  ******************************************************************************
  * @file    Supercapacitor.c
  * @author  
  * @version V1.0
  * @brief    超级电容模块
  ******************************************************************************
  */

#include "SuperCapacitor.h"

Supercapacitor_t supercapacitor;

void SCCM_MsgProcess(Can_Export_Data_t RxMessage);
void SCCM_SendMsg(float Charge_power, uint8_t Charge_enable, uint8_t Cap_enable);
void SuperCapacitor_Switch(bool state);
void Check_SuperCapacitor(void);
SuperCapacitorFUN_t SuperCapacitorFUN = SuperCapacitorFUN_Init;
#undef SuperCapacitorFUN_Init

uint8_t Cap_Tick_5MS;

/**
	* @brief  超级电容接收函数
  * @param	函数buff
  * @retval None
  */
void SCCM_MsgProcess(Can_Export_Data_t RxMessage)
{

    memcpy(supercapacitor.ReceiveData.data, RxMessage.CANx_Export_RxMessage, 8);

    //帧率
    supercapacitor.InfoUpdateFrame++;
    supercapacitor.InfoUpdateFlag = 1;
}

/**
	* @brief  超级电容发送函数
  * @param	充电功率 充电使能 是否电容供电
  * @retval None
  */
void SCCM_SendMsg(float Charge_power, uint8_t Charge_enable, uint8_t Cap_enable)
{
    supercapacitor.SendData.charge_power = Charge_power;
    supercapacitor.SendData.charge_enable = Charge_enable;
    supercapacitor.SendData.is_cap_output = Cap_enable;
}

/**
	* @brief  超级电容开关
  * @param	void
  * @retval void
  */
void SuperCapacitor_Switch(bool state)
{
    if (state == true)
    {
        supercapacitor.EnableCapacitor = true;
    }
    else
    {
        supercapacitor.EnableCapacitor = false;
    }
}

/**
 * @brief 超级电容检测
 * 
 */
void Check_SuperCapacitor(void)
{
    if (supercapacitor.InfoUpdateFrame < 1)
    {
        supercapacitor.OffLineFlag = 1;
    }
    else
    {
        supercapacitor.OffLineFlag = 0;
    }
    supercapacitor.InfoUpdateFrame = 0;
}
