/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       oled.h
 * @brief      this file contains sd card basic operating function
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-28-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __OLED__H
#define __OLED__H

#include "stm32f4xx.h"
#include "spi.h"
#include <stdint.h>

#define Max_Column      128
#define Max_Row         64

#define X_WIDTH         128
#define Y_WIDTH         64

#define OLED_CMD        0x00
#define OLED_DATA       0x01

#define CHAR_SIZE_WIDTH     6
#define VHAR_SIZE_HIGHT     12


#if   Board_type ==  DJI_Board_A
     

#define OLED_DC_Pin        GPIO_PIN_9
#define OLED_DC_GPIO_Port  GPIOB
#define OLED_RST_Pin       GPIO_PIN_10
#define OLED_RST_GPIO_Port GPIOB
#define OLED_SCLK_Pin      GPIO_PIN_3
#define OLED_SCLK_GPIO_Port GPIOB
#define OLED_MOSI_Pin      GPIO_PIN_7
#define OLED_MOSI_GPIO_Port GPIOA

#define OLED_CMD_Set()      HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET)
#define OLED_CMD_Clr()      HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET)

#define OLED_RST_Set()      HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET)
#define OLED_RST_Clr()      HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET) 

#define OLED_SCLK_Clr()     HAL_GPIO_WritePin(OLED_SCLK_GPIO_Port, OLED_SCLK_Pin, GPIO_PIN_SET)
#define OLED_SCLK_Set()     HAL_GPIO_WritePin(OLED_SCLK_GPIO_Port, OLED_SCLK_Pin, GPIO_PIN_RESET)

#define OLED_MOSI_Clr()     HAL_GPIO_WritePin(OLED_MOSI_GPIO_Port, OLED_MOSI_Pin, GPIO_PIN_SET)
#define OLED_MOSI_Set()     HAL_GPIO_WritePin(OLED_MOSI_GPIO_Port, OLED_MOSI_Pin, GPIO_PIN_RESET)

#elif   Board_type == WOLF_Board

#define OLED_DC_Pin        GPIO_Pin_9
#define OLED_DC_GPIO_Port  GPIOB
#define OLED_RST_Pin       GPIO_Pin_10
#define OLED_RST_GPIO_Port GPIOB
#define OLED_SCLK_Pin      GPIO_Pin_2
#define OLED_SCLK_GPIO_Port GPIOE
#define OLED_MOSI_Pin      GPIO_Pin_6
#define OLED_MOSI_GPIO_Port GPIOE

#define OLED_CMD_Set()      GPIO_SetBits(OLED_DC_GPIO_Port, OLED_DC_Pin)
#define OLED_CMD_Clr()      GPIO_ResetBits(OLED_DC_GPIO_Port, OLED_DC_Pin)

#define OLED_RST_Set()      GPIO_SetBits(OLED_RST_GPIO_Port, OLED_RST_Pin)
#define OLED_RST_Clr()      GPIO_ResetBits(OLED_RST_GPIO_Port, OLED_RST_Pin) 

#define OLED_SCLK_Clr()     GPIO_SetBits(OLED_SCLK_GPIO_Port, OLED_SCLK_Pin)
#define OLED_SCLK_Set()     GPIO_ResetBits(OLED_SCLK_GPIO_Port, OLED_SCLK_Pin)

#define OLED_MOSI_Clr()     GPIO_SetBits(OLED_MOSI_GPIO_Port, OLED_MOSI_Pin)
#define OLED_MOSI_Set()     GPIO_ResetBits(OLED_MOSI_GPIO_Port, OLED_MOSI_Pin)

#endif


typedef enum
{
    Pen_Clear = 0x00,
    Pen_Write = 0x01,
    Pen_Inversion = 0x02,
}Pen_Typedef;

/* function define */
void oled_init(void);
void oled_write_byte(uint8_t dat, uint8_t cmd);
void oled_display_on(void);
void oled_display_off(void);
void oled_refresh_gram(void);
void oled_clear(Pen_Typedef pen);
void oled_drawpoint(int8_t x, int8_t y, Pen_Typedef pen);
void oled_drawline(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, Pen_Typedef pen);
void oled_showchar(uint8_t row, uint8_t col, uint8_t chr);
void oled_shownum(uint8_t row, uint8_t col, uint32_t num, uint8_t mode, uint8_t len);
void oled_showstring(uint8_t row, uint8_t col, uint8_t *chr);
void oled_printf(uint8_t row, uint8_t col, const char *fmt,...);
void oled_LOGO(void);
void OLED_ShowChinese(uint16_t x, uint16_t y, uint8_t* index, uint8_t size);
uint16_t ADC_Average(uint8_t size);
void OLED_Button(void);
void Wolf_SPI_Transmit(SPI_HandleTypeDef *hspi,uint8_t *pData,uint16_t Size);
void OLED_ShowMessage(void);


extern uint16_t OLED_ADC_flag;

#endif








