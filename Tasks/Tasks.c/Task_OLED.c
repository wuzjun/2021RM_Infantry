/**
 * @file Task_OLED.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-04-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "Task_OLED.h"
#include "OLED.h"
#include "typedef.h"
#include "Robot_Config.h"

/**
 * @brief DJI_OLED¼ì²â
 * 
 * @param argument 
 * @return  
 */
uint8_t OledFrequency = 0;
WorldTime_RxTypedef DJI_OLED_FPS;
uint32_t DTest_FPS;
void DJI_OLED(void const *argument)
{
    for (;;)
    {
#ifdef Temporary_Function
        if (OledFrequency >= 10)
        {
            OLED_Button();
            OledFrequency = 0;
        }
        OledFrequency++;
        OLED_ShowMessage();
#endif

        Get_FPS(&DJI_OLED_FPS, &DTest_FPS);
        osDelay(5);
    }
}
