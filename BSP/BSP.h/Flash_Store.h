/**
 * @file Flash_Store.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-05-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _FLASH_STORE_H_
#define _FLASH_STORE_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define STM32_FLASH_BASE 0x080C0000    //DJI_IMU---STM32 FLASH的起始地址
#define STM32_FLASH_Offset1 0x080A0000 //Shoot---STM32 FLASH的地址
#define FLASH_WAITETIME 50000          //FLASH等待超时时间

#define ADDR_FLASH_SECTOR_0 ((uint32_t)0x08000000)  /* Base @ of Sector 0, 16 Kbyte */
#define ADDR_FLASH_SECTOR_1 ((uint32_t)0x08004000)  /* Base @ of Sector 1, 16 Kbyte */
#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08008000)  /* Base @ of Sector 2, 16 Kbyte */
#define ADDR_FLASH_SECTOR_3 ((uint32_t)0x0800C000)  /* Base @ of Sector 3, 16 Kbyte */
#define ADDR_FLASH_SECTOR_4 ((uint32_t)0x08010000)  /* Base @ of Sector 4, 64 Kbyte */
#define ADDR_FLASH_SECTOR_5 ((uint32_t)0x08020000)  /* Base @ of Sector 5, 128 Kbyte */
#define ADDR_FLASH_SECTOR_6 ((uint32_t)0x08040000)  /* Base @ of Sector 6, 128 Kbyte */
#define ADDR_FLASH_SECTOR_7 ((uint32_t)0x08060000)  /* Base @ of Sector 7, 128 Kbyte */
#define ADDR_FLASH_SECTOR_8 ((uint32_t)0x08080000)  /* Base @ of Sector 8, 128 Kbyte */
#define ADDR_FLASH_SECTOR_9 ((uint32_t)0x080A0000)  /* Base @ of Sector 9, 128 Kbyte */
#define ADDR_FLASH_SECTOR_10 ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbyte */
#define ADDR_FLASH_SECTOR_11 ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbyte */

void MEM_If_Write_FS(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t Num);
void MEM_If_Read_FS(uint32_t ReadAddr, uint32_t *pBuffer, uint32_t size);
void DJI_IMUWriteOffset(uint32_t *pBuffer, uint32_t num);
void DJI_IMUReadOffset(uint32_t *pBuffer, uint32_t num);
void Shoot_WriteOffset(uint32_t *pBuffer, uint32_t num);
void Shoot_ReadOffset(uint32_t *pBuffer, uint32_t num);
void Shoot_EraseOffset(uint32_t num);

#endif /* _FLASH_STORE_H_ */
