#ifndef M_FLASH_H
#define M_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdbool.h>
#include "stm32g0xx_hal.h"

#define STM32_FLASH_SIZE 	      128 	 	          //所选STM32的FLASH容量大小(单位为K)
#define STM_SECTOR_SIZE	        FLASH_PAGE_SIZE   //2K字节
#define STM32_FLASH_BASE        0x08000000 		    //STM32 FLASH的起始地址
#define FLASH_USER1_ADDR_S      0x0801F000
#define SECTOR_NUMBERS          2                 //扇区数量
#define FLASH_USER1_ADDR_E     (FLASH_USER1_ADDR_S + SECTOR_NUMBERS * STM_SECTOR_SIZE)

enum Data_Name
{
  CALIBRATION_DATA = 0x00FF0001,
  // CALIBRATION_DATA = 0x00000000,
};

bool Read_Specified_Flash(enum Data_Name name, unsigned int* buffer, unsigned int buffer_size);
bool Write_Specified_Flash(enum Data_Name name, unsigned int* buffer, unsigned int write_size);

#ifdef __cplusplus
}
#endif
#endif /*M_FLASH_H */