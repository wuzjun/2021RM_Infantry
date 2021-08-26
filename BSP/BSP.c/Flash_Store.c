/**
 * @file Flash_Store.c
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-05-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "Flash_Store.h"

/**
 * @brief Flash解锁
 * 
 */
void MEM_If_Init_FS(void)
{
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}

/**
 * @brief Flash上锁
 * 
 */
void MEM_If_DeInit_FS(void)
{
    HAL_FLASH_Lock();
}

/**
  * @brief  Gets the sector of a given address
  * @param  Address: Flash address
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
    uint32_t sector = 0;

    if ((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_SECTOR_0;
    }
    else if ((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_SECTOR_1;
    }
    else if ((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_SECTOR_2;
    }
    else if ((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_SECTOR_3;
    }
    else if ((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_SECTOR_4;
    }
    else if ((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_SECTOR_5;
    }
    else if ((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
    {
        sector = FLASH_SECTOR_6;
    }
    else if ((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
    {
        sector = FLASH_SECTOR_7;
    }
    else if ((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
    {
        sector = FLASH_SECTOR_8;
    }
    else if ((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
    {
        sector = FLASH_SECTOR_9;
    }
    else if ((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
    {
        sector = FLASH_SECTOR_10;
    }
    else
    {
        sector = FLASH_SECTOR_11;
    }

    return sector;
}

/**
 * @brief 读取一个4字节的数据
 * 
 * @param faddr 
 * @return  
 */
uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
    return *(__IO uint32_t *)faddr;
}

/**
 * @brief Flash的擦除
 * 
 * @param WriteAddr 开头地址
 * @param Num 需要写入数组元素数量
 * @return  
 */
void MEM_If_Erase_FS(uint32_t WriteAddr, uint32_t Num)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    uint32_t SectorError = 0;
    uint32_t addrx = 0;
    uint32_t endaddr = 0;
    if (WriteAddr < WriteAddr || WriteAddr % 4)
        return; //非法地址

    MEM_If_Init_FS();
    addrx = WriteAddr;             //写入的起始地址
    endaddr = WriteAddr + Num * 4; //写入的结束地址

    if (addrx < 0x080E1000)
    {
        while (addrx < endaddr)
        {
            if (STMFLASH_ReadWord(addrx) != 0XFFFFFFFF)
            {
                FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;  //擦除类型，扇区擦除
                FlashEraseInit.Sector = GetSector(addrx);            //要擦除的扇区
                FlashEraseInit.NbSectors = 1;                        //一次只擦除一个扇区
                FlashEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3; //电压范围，VCC=2.7~3.6V之间!!
                if (HAL_FLASHEx_Erase(&FlashEraseInit, &SectorError) != HAL_OK)
                {
                    break; //发生错误了
                }
            }
            else
                addrx += 4;
            FLASH_WaitForLastOperation(FLASH_WAITETIME); //等待上次操作完成
        }
    }
    MEM_If_DeInit_FS(); //上锁
}

/**
 * @brief Flash的写入
 * 
 * @param WriteAddr 开头地址
 * @param pBuffer 需要写入的数据
 * @param Num 需要写入数组元素数量
 * @return  
 */
void MEM_If_Write_FS(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t Num)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    HAL_StatusTypeDef FlashStatus = HAL_OK;
    uint32_t SectorError = 0;
    uint32_t addrx = 0;
    uint32_t endaddr = 0;
    if (WriteAddr < WriteAddr || WriteAddr % 4)
        return; //非法地址

    MEM_If_Init_FS();
    addrx = WriteAddr;             //写入的起始地址
    endaddr = WriteAddr + Num * 4; //写入的结束地址

    if (addrx < 0x080E1000)
    {
        while (addrx < endaddr)
        {
            if (STMFLASH_ReadWord(addrx) != 0XFFFFFFFF)
            {
                FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;  //擦除类型，扇区擦除
                FlashEraseInit.Sector = GetSector(addrx);            //要擦除的扇区
                FlashEraseInit.NbSectors = 1;                        //一次只擦除一个扇区
                FlashEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3; //电压范围，VCC=2.7~3.6V之间!!
                if (HAL_FLASHEx_Erase(&FlashEraseInit, &SectorError) != HAL_OK)
                {
                    break; //发生错误了
                }
            }
            else
                addrx += 4;
            FLASH_WaitForLastOperation(FLASH_WAITETIME); //等待上次操作完成
        }
    }
    FlashStatus = FLASH_WaitForLastOperation(FLASH_WAITETIME); //等待上次操作完成
    if (FlashStatus == HAL_OK)
    {
        while (WriteAddr < endaddr) //写数据
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WriteAddr, *pBuffer) != HAL_OK) //写入数据
            {
                break; //写入异常
            }
            WriteAddr += 4;
            pBuffer++;
        }
    }
    MEM_If_DeInit_FS(); //上锁
}

/**
 * @brief Flash的读取
 * 
 * @param ReadAddr 开头地址
 * @param pBuffer 需要读取的数据
 * @param size 需要读取数据的大小
 * @return  
 */
void MEM_If_Read_FS(uint32_t ReadAddr, uint32_t *pBuffer, uint32_t size)
{
    MEM_If_Init_FS();
    for (uint32_t i = 0; i < size; i++)
    {
        pBuffer[i] = STMFLASH_ReadWord(ReadAddr); //读取4个字节.
        ReadAddr += 4;                            //偏移4个字节.
    }
    MEM_If_DeInit_FS(); //上锁
}

/****************DJI陀螺仪校准值*****************/
void DJI_IMUWriteOffset(uint32_t *pBuffer, uint32_t num)
{
    MEM_If_Write_FS(STM32_FLASH_BASE, pBuffer, num);
}

void DJI_IMUReadOffset(uint32_t *pBuffer, uint32_t num)
{
    MEM_If_Read_FS(STM32_FLASH_BASE, pBuffer, sizeof(uint32_t[num]));
}
/****************DJI陀螺仪校准值 END*****************/

/****************射速校准值****************/
void Shoot_WriteOffset(uint32_t *pBuffer, uint32_t num)
{
    MEM_If_Write_FS(STM32_FLASH_Offset1, pBuffer, num);
}

void Shoot_ReadOffset(uint32_t *pBuffer, uint32_t num)
{
    MEM_If_Read_FS(STM32_FLASH_Offset1, pBuffer, sizeof(uint32_t[num]));
}

void Shoot_EraseOffset(uint32_t num)
{
    MEM_If_Erase_FS(STM32_FLASH_Offset1, num);
}
/****************射速校准值 END****************/



