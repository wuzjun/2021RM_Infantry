/**
 * @file BSP_USART.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __BSP_USART
#define __BSP_USART

#include "usart.h"
#include "dma.h"

#define USART_FunGroundInit           \
    {                                 \
        &DMA_USART_Receive_IT_Init,   \
            &DMA_USART_Send_IT_Init,  \
    }

typedef struct
{
    void (*DMA_USART_Receive_IT_Init)(UART_HandleTypeDef *huartx);
    void (*DMA_USART_Send_IT_Init)(UART_HandleTypeDef *huartx);

} USART_Fun_t;

extern USART_Fun_t USART_Fun;

#endif /*__BSP_USART*/
