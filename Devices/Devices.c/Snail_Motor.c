/**
 * @file Snail_Motor.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "Snail_Motor.h"

Snail_t Snail_PowerL; //左边的摩擦轮。
Snail_t Snail_PowerR; //左边的摩擦轮。

//后面会根据测速器来获取速度。
uint8_t TIM4CH1_CAPTURE_STA;	//输入捕获状态
uint16_t TIM4CH1_CAPTURE_Date1; //输入捕获值
uint16_t TIM4CH1_CAPTURE_Date2; //输入捕获值

uint8_t TIM4CH2_CAPTURE_STA;	//输入捕获状态
uint16_t TIM4CH2_CAPTURE_Date1; //输入捕获值
uint16_t TIM4CH2_CAPTURE_Date2; //输入捕获值
long long temp1 = 0;
long long temp2 = 0;

TIMxCHx_CAPTURE_t TIM4CH1_CAPTURE;
TIMxCHx_CAPTURE_t TIM4CH2_CAPTURE;
TIMxCHx_CAPTURE_t TIM4CH3_CAPTURE;
TIMxCHx_CAPTURE_t TIM4CH4_CAPTURE;

void C615_CaptureSpeed(TIM_TypeDef *TIMx, uint8_t One_Two);
void C615_ObtainSpeed(TIMxCHx_CAPTURE_t *TIMxCHx_CAPTURE, Snail_t *Snail_Power);

Snail_Speed_Fun_t Snail_Speed_Fun = Snail_Speed_FunGroundInit;
#undef Snail_Speed_FunGroundInit

/**
  * @brief  获取C615速度，在需要获取速度的位置上使用
  * @param  TIMxCHx_CAPTURE_t *TIMxCHx_CAPTURE, Snail_t *Snail_Power
  * @retval None
  */
void C615_ObtainSpeed(TIMxCHx_CAPTURE_t *TIMxCHx_CAPTURE, Snail_t *Snail_Power)
{
	if (TIMxCHx_CAPTURE->STA & 0X80) //成功捕获到了一次高电平
	{
		TIMxCHx_CAPTURE->temp = TIMxCHx_CAPTURE->STA & 0X3F;
		TIMxCHx_CAPTURE->temp *= 65536; //溢出时间总和
		TIMxCHx_CAPTURE->temp += TIMxCHx_CAPTURE->Date2;
		TIMxCHx_CAPTURE->temp -= TIMxCHx_CAPTURE->Date1; //得到总的高电平时间
		if (abs(TIMxCHx_CAPTURE->temp) > 1800)
		{
			Snail_Power->realSpeed = Snail_Power->realSpeed;
		}
		else
		{
			Snail_Power->realSpeed = TIMxCHx_CAPTURE->temp;
		}
		TIMxCHx_CAPTURE->Date1 = 0;
		TIMxCHx_CAPTURE->STA = 0; //开启下一次捕获
	}
}

/**
* @brief 捕获第一组C615速度
* @param argument: Not used
* @retval None
*/
static void C615_CaptureSpeed_CH1CH2(TIM_TypeDef *TIM, TIMxCHx_CAPTURE_t *TIMxCH1_CAPTURE, TIMxCHx_CAPTURE_t *TIMxCH2_CAPTURE)
{
	uint16_t tsr;
	tsr = TIM->SR;
	//CH1中断处理
	if ((TIMxCH1_CAPTURE->STA & 0X80) == 0) //还未成功捕获
	{
		if (tsr & 0X01) //溢出
		{
			if (TIMxCH1_CAPTURE->STA & 0X40) //已经捕获到高电平了
			{
				if ((TIMxCH1_CAPTURE->STA & 0X3F) == 0X3F) //高电平太长了
				{
					TIMxCH1_CAPTURE->STA |= 0X80; //标记成功捕获了一次
					TIMxCH1_CAPTURE->Date2 = 0XFFFF;
				}
				else
					TIMxCH1_CAPTURE->STA++;
			}
		}
		if (tsr & 0x02) //捕获1发生捕获事件
		{
			if (TIMxCH1_CAPTURE->STA & 0X40) //捕获到一个下降沿
			{
				TIMxCH1_CAPTURE->STA |= 0X80;		//标记成功捕获到一次高电平脉宽
				TIMxCH1_CAPTURE->Date2 = TIM->CCR1; //获取当前的捕获值.
				TIM->CCER &= ~(1 << 1);				//CC1P=0 设置为上升沿捕获
			}
			else //还未开始,第一次捕获上升沿
			{
				TIMxCH1_CAPTURE->Date2 = 0;
				TIMxCH1_CAPTURE->STA = 0X40; //标记捕获到了上升沿
				TIMxCH1_CAPTURE->Date1 = TIM->CCR1;
				TIM->CCER |= 1 << 1; //CC1P=1 设置为下降沿捕获
			}
		}
	}
	//CH2中断处理
	if ((TIMxCH2_CAPTURE->STA & 0X80) == 0) //还未成功捕获
	{
		if (tsr & 0X01) //溢出
		{
			if (TIMxCH2_CAPTURE->STA & 0X40) //已经捕获到高电平了
			{
				if ((TIMxCH2_CAPTURE->STA & 0X3F) == 0X3F) //高电平太长了
				{
					TIMxCH2_CAPTURE->STA |= 0X80; //标记成功捕获了一次
					TIMxCH2_CAPTURE->Date2 = 0XFFFF;
				}
				else
					TIMxCH2_CAPTURE->STA++;
			}
		}
		if (tsr & 0x04) //捕获1发生捕获事件
		{
			if (TIMxCH2_CAPTURE->STA & 0X40) //捕获到一个下降沿
			{
				TIMxCH2_CAPTURE->STA |= 0X80;		//标记成功捕获到一次高电平脉宽
				TIMxCH2_CAPTURE->Date2 = TIM->CCR2; //获取当前的捕获值.
				TIM->CCER &= ~(1 << 5);				//CC1P=0 设置为上升沿捕获
			}
			else //还未开始,第一次捕获上升沿
			{
				TIMxCH2_CAPTURE->Date2 = 0;
				TIMxCH2_CAPTURE->STA = 0X40; //标记捕获到了上升沿
				TIMxCH2_CAPTURE->Date1 = TIM->CCR2;
				TIM->CCER |= 1 << 5; //CC1P=1 设置为下降沿捕获
			}
		}
	}
}

/**
* @brief 捕获第二组C615速度
* @param argument: Not used
* @retval None
*/
static void C615_CaptureSpeed_CH3CH4(TIM_TypeDef *TIM, TIMxCHx_CAPTURE_t *TIMxCH3_CAPTURE, TIMxCHx_CAPTURE_t *TIMxCH4_CAPTURE)
{
	uint16_t tsr;
	tsr = TIM->SR;
	//CH3中断处理
	if ((TIMxCH3_CAPTURE->STA & 0X80) == 0) //还未成功捕获
	{
		if (tsr & 0X01) //溢出
		{
			if (TIMxCH3_CAPTURE->STA & 0X40) //已经捕获到高电平了
			{
				if ((TIMxCH3_CAPTURE->STA & 0X3F) == 0X3F) //高电平太长了
				{
					TIMxCH3_CAPTURE->STA |= 0X80; //标记成功捕获了一次
					TIMxCH3_CAPTURE->Date2 = 0XFFFF;
				}
				else
					TIMxCH3_CAPTURE->STA++;
			}
		}
		if (tsr & 0x08) //捕获1发生捕获事件
		{
			if (TIMxCH3_CAPTURE->STA & 0X40) //捕获到一个下降沿
			{
				TIMxCH3_CAPTURE->STA |= 0X80;		//标记成功捕获到一次高电平脉宽
				TIMxCH3_CAPTURE->Date2 = TIM->CCR3; //获取当前的捕获值.
				TIM->CCER &= ~(1 << 9);				//CC1P=0 设置为上升沿捕获
			}
			else //还未开始,第一次捕获上升沿
			{
				TIMxCH3_CAPTURE->Date2 = 0;
				TIMxCH3_CAPTURE->STA = 0X40; //标记捕获到了上升沿
				TIMxCH3_CAPTURE->Date1 = TIM->CCR3;
				TIM->CCER |= 1 << 9; //CC1P=1 设置为下降沿捕获
			}
		}
	}
	//CH4中断处理
	if ((TIMxCH4_CAPTURE->STA & 0X80) == 0) //还未成功捕获
	{
		if (tsr & 0X01) //溢出
		{
			if (TIMxCH4_CAPTURE->STA & 0X40) //已经捕获到高电平了
			{
				if ((TIMxCH4_CAPTURE->STA & 0X3F) == 0X3F) //高电平太长了
				{
					TIMxCH4_CAPTURE->STA |= 0X80; //标记成功捕获了一次
					TIMxCH4_CAPTURE->Date2 = 0XFFFF;
				}
				else
					TIMxCH4_CAPTURE->STA++;
			}
		}
		if (tsr & 0x10) //捕获1发生捕获事件
		{
			if (TIMxCH4_CAPTURE->STA & 0X40) //捕获到一个下降沿
			{
				TIMxCH4_CAPTURE->STA |= 0X80;		//标记成功捕获到一次高电平脉宽
				TIMxCH4_CAPTURE->Date2 = TIM->CCR4; //获取当前的捕获值.
				TIM->CCER &= ~(1 << 13);			//CC1P=0 设置为上升沿捕获
			}
			else //还未开始,第一次捕获上升沿
			{
				TIMxCH4_CAPTURE->Date2 = 0;
				TIMxCH4_CAPTURE->STA = 0X40; //标记捕获到了上升沿
				TIMxCH4_CAPTURE->Date1 = TIM->CCR4;
				TIM->CCER |= 1 << 13; //CC1P=1 设置为下降沿捕获
			}
		}
	}
}

/**
  * @brief  捕获高（低）电平，在TIM中断处使用
  * @param  TIM_TypeDef *TIMx, uint8_t One_Two
  * @retval None
  */
void C615_CaptureSpeed(TIM_TypeDef *TIMx, uint8_t One_Two)
{
	C615_CaptureSpeed_CH1CH2(TIMx, &TIM4CH1_CAPTURE, &TIM4CH2_CAPTURE);
	if (One_Two == Two)
	{
		C615_CaptureSpeed_CH3CH4(TIMx, &TIM4CH3_CAPTURE, &TIM4CH4_CAPTURE);
	}

	TIMx->SR = 0; //清除中断标志位
}
