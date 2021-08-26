/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       oled.c
 * @brief      this file contains sd card basic operating function
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-28-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#include "OLED.h"
#include "Control_Vision.h"
#include "DJI_IMU.h"
#include "DR16_Remote.h"
#include "M2006_Motor.h"
#include "M3508_Motor.h"
#include "M6020_Motor.h"
#include "OLED_font.h"
#include "RM_JudgeSystem.h"
#include "Shoot.h"
#include "SuperCapacitor.h"
#include "adc.h"
#include "math.h"
#include <stdarg.h>
#include <stdio.h>

__IO uint16_t ADCDualConvertedValue[5];

uint32_t ADC_Value[50] = {0}; //5向按键AD值
uint32_t ad1 = 0;
uint16_t OLED_ADC_flag = 0; //5向按键标志位

/**
 * @brief   自写SPI_Transmit
 * @param   dat: the data ready to write
 * @param   cmd: 0x00,command 0x01,data
 * @retval  消掉HAL库的延时V1.0
 */
void Wolf_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
    assert_param(IS_SPI_DIRECTION_2LINES_OR_1LINE(hspi->Init.Direction));
    hspi->pTxBuffPtr = (uint8_t *)pData;
    hspi->TxXferSize = Size;
    hspi->TxXferCount = Size;
    uint16_t initial_TxXferCount = Size;
    if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_ENABLE(hspi);
    }
    if ((hspi->Init.Mode == SPI_MODE_SLAVE) || (initial_TxXferCount == 0x01U))
    {
        *((__IO uint8_t *)&hspi->Instance->DR) = (*hspi->pTxBuffPtr);
        hspi->pTxBuffPtr += sizeof(uint8_t);
        hspi->TxXferCount--;
    }
    while (hspi->TxXferCount > 0U)
    {
        /* Wait until TXE flag is set to send data */
        if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE) == RESET)
        {
            *((__IO uint8_t *)&hspi->Instance->DR) = (*hspi->pTxBuffPtr);
            hspi->pTxBuffPtr += sizeof(uint8_t);
            hspi->TxXferCount--;
        }
    }
}

/**
 * OLED flash Addr:
 * [0]0 1 2 3 ... 127
 * [1]0 1 2 3 ... 127
 * [2]0 1 2 3 ... 127
 * [3]0 1 2 3 ... 127
 * [4]0 1 2 3 ... 127
 * [5]0 1 2 3 ... 127
 * [6]0 1 2 3 ... 127
 * [7]0 1 2 3 ... 127
**/

static uint8_t OLED_GRAM[128][8];

//short delay uesd in spi transmmit
void oled_delay_ms(uint16_t delaytimes)
{
    uint16_t i;
    for (i = 0; i < delaytimes; i++)
    {
        int a = 10000;
        //delay based on mian clock, 168Mhz
        while (a--)
            ;
    }
}

/**
 * @brief   write data/command to OLED
 * @param   dat: the data ready to write
 * @param   cmd: 0x00,command 0x01,data
 * @retval  
 */
void oled_write_byte(uint8_t dat, uint8_t cmd)
{
    if (cmd != 0)
        OLED_CMD_Set();
    else
        OLED_CMD_Clr();

    //while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET);

    //HAL_SPI_Transmit(&hspi1, &dat , 1 , HAL_MAX_DELAY);

    Wolf_SPI_Transmit(&hspi1, &dat, 1);
}

/**
 * @brief   set OLED cursor position
 * @param   x: the X-axis of cursor
 * @param   y: the Y-axis of cursor
 * @retval  
 */
static void oled_set_pos(uint8_t x, uint8_t y)
{
    x += 2;
    oled_write_byte((0xb0 + y), OLED_CMD);               //set page address y
    oled_write_byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD); //set column high address
    oled_write_byte((x & 0xf0), OLED_CMD);               //set column low address
}

/**
 * @brief   turn on OLED display
 * @param   None
 * @param   None
 * @retval  
 */
void oled_display_on(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x14, OLED_CMD);
    oled_write_byte(0xaf, OLED_CMD);
}

/**
 * @brief   turn off OLED display
 * @param   None
 * @param   None
 * @retval  
 */
void oled_display_off(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x10, OLED_CMD);
    oled_write_byte(0xae, OLED_CMD);
}

/**
 * @brief   refresh the RAM of OLED
 * @param   None
 * @param   None
 * @retval  
 */
void oled_refresh_gram(void)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        oled_set_pos(0, i);

        for (n = 0; n < 128; n++)
        {
            oled_write_byte(OLED_GRAM[n][i], OLED_DATA);
        }
    }
}

/**
 * @brief   clear the screen
 * @param   None
 * @param   None
 * @retval  
 */
void oled_clear(Pen_Typedef pen)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        for (n = 0; n < 128; n++)
        {
            if (pen == Pen_Write)
                OLED_GRAM[n][i] = 0xff;
            else if (pen == Pen_Clear)
                OLED_GRAM[n][i] = 0x00;
            else
                OLED_GRAM[n][i] = 0xff - OLED_GRAM[n][i];
        }
    }
}

/**
 * @brief   draw a point at (x, y)
 * @param   x: the X-axis of cursor
 * @param   y: the Y-axis of cursor
 * @param   pen: Pen_Clear, Pen_Write, Pen_Inversion @Pen_Typedef
 * @retval  None
 */
void oled_drawpoint(int8_t x, int8_t y, Pen_Typedef pen)
{
    uint8_t page = 0, row = 0;

    /* check the corrdinate */
    if ((x < 0) || (x > (X_WIDTH - 1)) || (y < 0) || (y > (Y_WIDTH - 1)))
        return;

    page = y / 8;
    row = y % 8;

    if (pen == Pen_Write)
        OLED_GRAM[x][page] |= 1 << row;
    else if (pen == Pen_Inversion)
        OLED_GRAM[x][page] ^= 1 << row;
    else
        OLED_GRAM[x][page] &= ~(1 << row);
}

/**
 * @brief   draw a line from (x1, y1) to (x2, y2)
 * @param   x1, y1: the start point of line
 * @param   x2, y2: the end of line
 * @param   pen: Pen_Clear, Pen_Write, Pen_Inversion @Pen_Typedef
 * @retval  None
 */
void oled_drawline(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, Pen_Typedef pen)
{
    uint8_t col = 0, row = 0;
    uint8_t x_st = 0, x_ed = 0, y_st = 0, y_ed = 0;
    float k = 0.0f, b = 0.0f;

    if (y1 == y2)
    {
        (x1 <= x2) ? (x_st = x1) : (x_st = x2);
        (x1 <= x2) ? (x_ed = x2) : (x_ed = x1);

        for (col = x_st; col <= x_ed; col++)
        {
            oled_drawpoint(col, y1, pen);
        }
    }
    else if (x1 == x2)
    {
        (y1 <= y2) ? (y_st = y1) : (y_st = y2);
        (y1 <= y2) ? (y_ed = y2) : (y_ed = y1);

        for (row = y_st; row <= y_ed; row++)
        {
            oled_drawpoint(x1, row, pen);
        }
    }
    else
    {
        k = ((float)(y2 - y1)) / (x2 - x1);
        b = (float)y1 - k * x1;

        (x1 <= x2) ? (x_st = x1) : (x_st = x2);
        (x1 <= x2) ? (x_ed = x2) : (x_ed = x2);

        for (col = x_st; col <= x_ed; col++)
        {
            oled_drawpoint(col, (uint8_t)(col * k + b), pen);
        }
    }
}

//To add: rectangle, fillrectangle, circle, fillcircle,

/**
 * @brief   show a character
 * @param   row: row of character
 * @param   col: column of character
 * @param   chr: the character ready to show
 * @retval  None
 */
void oled_showchar(uint8_t row, uint8_t col, uint8_t chr)
{
    uint8_t x = col * 6;
    uint8_t y = row * 12;
    uint8_t temp, t, t1;
    uint8_t y0 = y;
    chr = chr - ' ';

    for (t = 0; t < 12; t++)
    {
        temp = asc2_1206[chr][t];

        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp & 0x80)
                oled_drawpoint(x, y, Pen_Write);
            else
                oled_drawpoint(x, y, Pen_Clear);

            temp <<= 1;
            y++;
            if ((y - y0) == 12)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

//m^n
static uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;

    while (n--)
        result *= m;

    return result;
}

/**
 * @brief   show a number
 * @param   row: row of number
 * @param   col: column of number
 * @param   num: the number ready to show
 * @param   mode: 0x01, fill number with '0'; 0x00, fill number with spaces
 * @param   len: the length of the number
 * @retval  None
 */
void oled_shownum(uint8_t row, uint8_t col, uint32_t num, uint8_t mode, uint8_t len)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;

        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                if (mode == 0)
                    oled_showchar(row, col + t, ' ');
                else
                    oled_showchar(row, col + t, '0');
                continue;
            }
            else
                enshow = 1;
        }

        oled_showchar(row, col + t, temp + '0');
    }
}

/**
 * @brief   show a float number string
 * @param   row: row of character string begin
 * @param   col: column of character string begin
 * @param   chr: the pointer to float number string
 * @retval  自写，非官方
 */
void oled_showfloatnum(uint8_t row, uint8_t col, float num)
{
    char Transfernum[12] = {0};
    sprintf((char *)Transfernum, "%.2f", num);
    oled_showstring(row, col, (uint8_t *)Transfernum);
}

/**
 * @brief   show a character string
 * @param   row: row of character string begin
 * @param   col: column of character string begin
 * @param   chr: the pointer to character string
 * @retval  None
 */
void oled_showstring(uint8_t row, uint8_t col, uint8_t *chr)
{
    uint8_t n = 0;

    while (chr[n] != '\0')
    {
        oled_showchar(row, col, chr[n]);
        col++;

        if (col > 20)
        {
            col = 0;
            row += 1;
        }
        n++;
    }
}

/**
 * @brief   formatted output in oled 128*64
 * @param   row: row of character string begin, 0 <= row <= 4;
 * @param   col: column of character string begin, 0 <= col <= 20;
 * @param   *fmt: the pointer to format character string
 * @retval  None
 * @note    if the character length is more than one row at a time, the extra characters will be truncated
 */
void oled_printf(uint8_t row, uint8_t col, const char *fmt, ...)
{
    uint8_t LCD_BUF[128] = {0};
    uint8_t remain_size = 0;
    va_list ap;

    if ((row > 4) || (row < 1) || (col > 20) || (col < 1))
        return;

    va_start(ap, fmt);

    vsprintf((char *)LCD_BUF, fmt, ap);

    va_end(ap);

    remain_size = 21 - col;

    LCD_BUF[remain_size] = '\0';

    oled_showstring(row, col, LCD_BUF);
}

void oled_LOGO(void)
{
    oled_clear(Pen_Clear);
    uint8_t temp_char = 0;
    uint8_t x = 0, y = 0;
    uint8_t i = 0;
    for (; y < 64; y += 8)
    {
        for (x = 0; x < 128; x++)
        {
            temp_char = LOGO_BMP[x / 1][y / 8];
            for (i = 0; i < 8; i++)
            {
                if (temp_char & 0x80)
                    oled_drawpoint(x, y + i, Pen_Write);
                else
                    oled_drawpoint(x, y + i, Pen_Clear);
                temp_char <<= 1;
            }
        }
    }
    oled_refresh_gram();
}

void OLED_ShowChinese(uint16_t x, uint16_t y, uint8_t *index, uint8_t size)
{
    uint8_t temp, t, t1;
    uint16_t y0 = y;
    uint8_t *dzk;
    uint8_t csize = (size / 8 + ((size % 8) ? 1 : 0)) * size; //汉字字节计算
    dzk = index;                                              //得到汉字编号对应的点阵库
    for (t = 0; t < csize; t++)
    {
        temp = dzk[t];             //得到点阵数据
        for (t1 = 0; t1 < 8; t1++) //按照从高位到低位的顺序画点
        {
            if (temp & 0x80)
                oled_drawpoint(x, y, Pen_Write);
            else
                oled_drawpoint(x, y, Pen_Clear);
            temp <<= 1;
            y++;
            if ((y - y0) == size) //对y坐标的处理，当y坐标距起始坐标差24个像素点，x坐标加1
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

/**
 * @brief   initialize the oled module
 * @param   None
 * @retval  None
 */
void oled_init(void)
{
    OLED_RST_Clr();
    HAL_Delay(500);
    OLED_RST_Set();

    oled_write_byte(0xae, OLED_CMD); //turn off oled panel
    oled_write_byte(0x00, OLED_CMD); //set low column address
    oled_write_byte(0x10, OLED_CMD); //set high column address
    oled_write_byte(0x40, OLED_CMD); //set start line address
    oled_write_byte(0x81, OLED_CMD); //set contrast control resigter
    oled_write_byte(0xcf, OLED_CMD); //set SEG output current brightness
    oled_write_byte(0xa1, OLED_CMD); //set SEG/column mapping
    oled_write_byte(0xc8, OLED_CMD); //set COM/row scan direction
    oled_write_byte(0xa6, OLED_CMD); //set nomarl display
    oled_write_byte(0xa8, OLED_CMD); //set multiplex display
    oled_write_byte(0x3f, OLED_CMD); //1/64 duty
    oled_write_byte(0xd3, OLED_CMD); //set display offset
    oled_write_byte(0x00, OLED_CMD); //not offest
    oled_write_byte(0xd5, OLED_CMD); //set display clock divide ratio/oscillator frequency
    oled_write_byte(0x80, OLED_CMD); //set divide ratio
    oled_write_byte(0xd9, OLED_CMD); //set pre-charge period
    oled_write_byte(0xf1, OLED_CMD); //pre-charge: 15 clocks, discharge: 1 clock
    oled_write_byte(0xda, OLED_CMD); //set com pins hardware configuration
    oled_write_byte(0x12, OLED_CMD); //
    oled_write_byte(0xdb, OLED_CMD); //set vcomh
    oled_write_byte(0x40, OLED_CMD); //set vcom deselect level
    oled_write_byte(0x20, OLED_CMD); //set page addressing mode
    oled_write_byte(0x02, OLED_CMD); //
    oled_write_byte(0x8d, OLED_CMD); //set charge pump enable/disable
    oled_write_byte(0x14, OLED_CMD); //charge pump disable
    oled_write_byte(0xa4, OLED_CMD); //disable entire dispaly on
    oled_write_byte(0xa6, OLED_CMD); //disable inverse display on
    oled_write_byte(0xaf, OLED_CMD); //turn on oled panel

    oled_write_byte(0xaf, OLED_CMD); //display on

    oled_clear(Pen_Clear);
    oled_set_pos(0, 0);
}

/**
  ******************************************************************************
  * @author  Wu Guoxi
  * @version didi
  * @date    December 5th
  * @brief   ADC平均值采集
  ******************************************************************************
  */
uint16_t ADC_Average(uint8_t size)
{
    uint8_t i;
    for (i = 0, ad1 = 0; i < size; i++)
    {
        HAL_ADC_Start(&hadc1);
        while (!__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC))
            ;
        ADC_Value[i] = HAL_ADC_GetValue(&hadc1);
        ad1 += ADC_Value[i];
    }
    return (ad1 /= size);
}

/**
  ******************************************************************************
  * @author  Wu Guoxi
  * @version didi
  * @date    December 5th
  * @brief   五维按键标志位
  ******************************************************************************
  */
void OLED_Button(void)
{
    //均值滤波
    ad1 = ADC_Average(50);
    if (ad1 > 0 && ad1 < 200) //中按键按下
    {
        OLED_ADC_flag = 0;
    }
    else if (ad1 > 0 && ad1 < 950) //左按键按下
    {
        OLED_ADC_flag = 1;
    }
    else if (ad1 > 950 && ad1 < 1735) //右按键按下
    {
        OLED_ADC_flag = 2;
    }
    else if (ad1 > 2450 && ad1 < 2470) //上按键按下
    {
        OLED_ADC_flag = 3;
    }
    else if (ad1 > 3270 && ad1 < 3279) //下按键按下
    {
        OLED_ADC_flag = 4;
    }
    else if (ad1 > 3900) //无按键按下
    {
        OLED_ADC_flag = 5;
    }
    //   HAL_Delay(50);
}

/**
 * @brief   OLED显示信息
 * @param   None
 * @retval  None
 */
void OLED_ShowMessage(void)
{
    oled_clear(Pen_Clear);

    switch (OLED_ADC_flag)
    {
    case 0:
        oled_showstring(0, 1, "Devices Message");
        oled_showstring(1, 1, "Remote:");
        if (DR16_Export_Data.OffLineFlag)
        {
            oled_showstring(1, 10, "Offline");
        }
        else
        {
            oled_showstring(1, 10, "Online");
        }

        oled_showstring(2, 1, "Chassis:");
        if ((M3508s[0].OffLineFlag || M3508s[1].OffLineFlag || M3508s[2].OffLineFlag || M3508s[3].OffLineFlag))
        {
            oled_showstring(2, 10, "Offline");
        }
        else
        {
            oled_showstring(2, 10, "Online");
        }

        oled_showstring(3, 1, "Cloud:");
        if ((M6020s_Yaw.OffLineFlag || M6020s_Pitch.OffLineFlag || imu_Export.OffLineFlag))
        {
            oled_showstring(3, 10, "Offline");
        }
        else
        {
            oled_showstring(3, 10, "Online");
        }
        oled_showstring(4, 1, "Vision:");
        if (VisionData.OffLineFlag)
        {
            oled_showstring(4, 10, "Offline");
        }
        else
        {
            oled_showstring(4, 10, "Online");
        }

        break;

    case 1:
        oled_showstring(0, 1, "M3508_Detection:");
        oled_showstring(1, 2, "RightFront:");
        oled_showfloatnum(1, 13, M3508s[0].realSpeed);
        oled_showstring(2, 2, "LeftFront:");
        oled_showfloatnum(2, 13, M3508s[1].realSpeed);
        oled_showstring(3, 2, "LeftRear:");
        oled_showfloatnum(3, 13, M3508s[2].realSpeed);
        oled_showstring(4, 2, "RightRear:");
        oled_showfloatnum(4, 13, M3508s[3].realSpeed);

        break;
    case 2:
        oled_showstring(0, 1, "Shoot_Detection");
        oled_showstring(1, 2, "PowerL:");
        oled_showfloatnum(1, 10, M3508_PowerL.realSpeed);

        oled_showstring(2, 2, "PowerR:");
        oled_showfloatnum(2, 10, M3508_PowerR.realSpeed);

        oled_showstring(3, 2, "Plate:");
        oled_showfloatnum(3, 10, M2006_Reload.realAngle);

        oled_showstring(4, 2, "Bullet:");
        oled_showfloatnum(4, 10, needShoot);

        break;

    case 3:
        oled_showstring(0, 1, "GM6020_Detection:");
        oled_showstring(1, 1, "YAW_message:");
        oled_showfloatnum(2, 1, M6020s_Yaw.realAngle);
        oled_showstring(3, 1, "PITCH_message:");
        oled_showfloatnum(4, 1, M6020s_Pitch.totalAngle);

        break;

    case 4:
        oled_showstring(0, 1, "Other_Message:");
        oled_showstring(1, 1, "SuperCap:");
        if (supercapacitor.OffLineFlag)
        {
            oled_showstring(1, 10, "Offline");
        }
        else
        {
            oled_showstring(1, 10, "Online");
        }
        oled_showstring(2, 1, "DJI_IMU:");
        if (imu_Export.OffLineFlag)
        {
            oled_showstring(2, 10, "Offline");
        }
        else
        {
            oled_showstring(2, 10, "Online");
        }
        oled_showstring(3, 2, "Power:");
        oled_showfloatnum(3, 10, ext_game_robot_state.data.chassis_power_limit);
        oled_showstring(4, 2, "Heat:");
        oled_showfloatnum(4, 10, ext_game_robot_state.data.shooter_id1_17mm_cooling_limit);
        break;

    case 5:
        OLED_ShowChinese(30, 25, infantry_1, 16);
        OLED_ShowChinese(47, 25, infantry_null, 16);
        OLED_ShowChinese(64, 25, infantry_2, 16);
        break;
    }
    oled_refresh_gram();
}
