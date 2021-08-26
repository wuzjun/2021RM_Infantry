/**
 * @file Debug_DataScope.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __DEBUG_DATASCOPE
#define __DEBUG_DATASCOPE

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "usart.h"

#define Debug_DataScopeGroudInit \
    {                            \
        &Debug_addData,          \
            &Debug_show,         \
    }

typedef struct
{
    void (*Debug_addData)(float Data, unsigned char Channel);
    void (*Debug_show)(int ChannelAmount);
} Debug_DataScope_Fun_t;

extern Debug_DataScope_Fun_t Debug_DataScope_Fun;

void Debug_show(int ChannelAmount);
void Debug_addData(float Data, unsigned char Channel);

#endif /*__DEBUG_DATASCOPE*/
