/**
 * @file BSP_USART.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "BSP_USART.h"
// #include "DR16_Remote.h"

void DMA_USART_Receive_IT_Init(UART_HandleTypeDef *huartx);
void DMA_USART_Send_IT_Init(UART_HandleTypeDef *huartx);
USART_Fun_t USART_Fun = USART_FunGroundInit;
#undef USART_FunGroundInit

/**
  * @Data    2021/3/26
  * @brief   串口接收中断初始化（关闭DMA中断）
  * @param   void
  * @retval  void
  */
void DMA_USART_Receive_IT_Init(UART_HandleTypeDef *huartx)
{
  /*清空标志位然后使能USART的中断*/
  __HAL_UART_CLEAR_IDLEFLAG(huartx);
  __HAL_UART_ENABLE(huartx);
  __HAL_UART_ENABLE_IT(huartx, UART_IT_IDLE);

}

/**
  * @Data    2021/3/28
  * @brief   串口发送中断初始化（关闭DMA中断）
  * @param   void
  * @retval  void
  */
void DMA_USART_Send_IT_Init(UART_HandleTypeDef *huartx)
{
  /*清空标志位然后使能USART的中断*/
  __HAL_UART_CLEAR_IDLEFLAG(huartx);
  __HAL_UART_ENABLE(huartx);
  __HAL_UART_ENABLE_IT(huartx, UART_IT_TXE);

}


