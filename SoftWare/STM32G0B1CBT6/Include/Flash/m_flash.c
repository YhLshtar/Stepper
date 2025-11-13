#include "m_flash.h"

#include <string.h>

#include "stm32g0xx_hal_flash.h"

#define   FLASH_FLAG_ALL_ERRORS   (FLASH_FLAG_OPERR   | FLASH_FLAG_PROGERR | FLASH_FLAG_WRPERR | \
                                   FLASH_FLAG_PGAERR  | FLASH_FLAG_SIZERR  | FLASH_FLAG_PGSERR | \
                                   FLASH_FLAG_MISERR  | FLASH_FLAG_FASTERR | \
                                   FLASH_FLAG_OPTVERR | FLASH_FLAG_ECCC    | FLASH_FLAG_ECCD)
#define   FLASH_ERROR_COUNT     	5

struct Flash_Data_Node
{
  unsigned int  user_data_id;
  unsigned int  user_data_date;
  unsigned int  user_data_length;
  unsigned int  data_flash_address;
  unsigned int  read_data_length;
  unsigned int  write_data_length;
  unsigned int  write_address;
  unsigned int* buffer_address;
};

struct Flash_Data_Node data_node1 =
{
  .user_data_id       = CALIBRATION_DATA,
  .user_data_date     = 0,
  .user_data_length   = 0,
  .data_flash_address = FLASH_USER1_ADDR_S,
};

int error_count = 0;
unsigned int	flash_data = 0;
long long unsigned int double_word = 0;
unsigned char node_buffer[4 * 4];

bool Erase_Flash(unsigned int s_addr, unsigned int e_addr);
bool Write_Flash(unsigned int s_addr, long long unsigned int* data);
void CheckFLashAddress(unsigned int* flash_addr);
static void Flash_Protect_Lock(void);
static void Flash_Protect_UnLock(void);

static unsigned int GetPage(unsigned int addr)
{
  return (addr - FLASH_BASE) / FLASH_PAGE_SIZE;
}

static uint32_t GetBank(uint32_t addr)
{
  return FLASH_BANK_1;
}

bool Flash_Init(void)
{
  HAL_Delay(10);
  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return false;
  }

  return true;
}

bool Flash_DeInit(void)
{
  HAL_Delay(10);
  if (HAL_FLASH_Lock() != HAL_OK)
  {
    return false;
  }

  return true;
}
/*包含s_addr往后的扇区和e_addr往后的扇区，
  s_addr和e_addr相等则擦除s_addr所在的那一个扇区*/
bool Erase_Flash(unsigned int s_addr, unsigned int e_addr)
{
  unsigned int  SectorError = 0;
  FLASH_EraseInitTypeDef E_FLASH;

  error_count = 0;

  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_CFGBSY) != 0x00U)
  {
    *(unsigned int*)(FLASH_USER1_ADDR_E) = 0x12345678;/* 制造错误 */
    FLASH->SR = FLASH_SR_CLEAR;/* 清除所有错误标志 */
  }
  Flash_Protect_UnLock();
  // __disable_irq();
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  /* Get the 1st page to erase */
  unsigned int FirstPage  = GetPage(s_addr);
  /* Get the number of pages to erase from 1st page */
  unsigned int NbOfPages  = GetPage(e_addr) - FirstPage + 1;
  unsigned int BankNumber = GetBank(s_addr);
  E_FLASH.TypeErase = FLASH_TYPEERASE_PAGES;
  E_FLASH.Banks     = BankNumber;
  E_FLASH.Page      = FirstPage;
  E_FLASH.NbPages   = NbOfPages;

  while(HAL_FLASHEx_Erase(&E_FLASH, &SectorError) != HAL_OK)
  {
		FLASH->SR = FLASH_SR_CLEAR;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
		HAL_Delay(10);
    error_count++;
    if (error_count > FLASH_ERROR_COUNT)
    {
      FLASH_WaitForLastOperation(500);
      break;
    }
  }
  HAL_Delay(10);
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_CFGBSY) != 0x00U)
  {
    *(unsigned int*)(FLASH_USER1_ADDR_E) = 0x12345678;/* 制造错误 */
    HAL_Delay(100);
    FLASH->SR = FLASH_SR_CLEAR;/* 清除所有错误标志 */
  }
  HAL_FLASH_Lock();
  // __enable_irq();
  Flash_Protect_Lock();

  if (error_count > FLASH_ERROR_COUNT)
  {
    return false;
  }

  return true;
}

bool Write_Flash(unsigned int s_addr, long long unsigned int* data)
{
  error_count = 0;

  if(s_addr < STM32_FLASH_BASE ||
     s_addr >= STM32_FLASH_BASE + STM_SECTOR_SIZE * STM32_FLASH_SIZE)
  {
    return false;
  }

  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_CFGBSY) != 0x00U)
  {
    *(unsigned int*)(FLASH_USER1_ADDR_E) = 0x12345678;/* 制造错误 */
    FLASH->SR = FLASH_SR_CLEAR;/* 清除所有错误标志 */
  }
  Flash_Protect_UnLock();
  // __disable_irq();
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

  while(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, s_addr, *data) != HAL_OK)
  {
    FLASH->SR = FLASH_SR_CLEAR;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    HAL_Delay(10);
    error_count++;
    if (error_count > FLASH_ERROR_COUNT)
    {
      FLASH_WaitForLastOperation(500);
      break;
    }
  }
	HAL_Delay(20);
	if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_CFGBSY) != 0x00U)
  {
    *(unsigned int*)(FLASH_USER1_ADDR_E) = 0x12345678;/* 制造错误 */
	  HAL_Delay(100);
    FLASH->SR = FLASH_SR_CLEAR;/* 清除所有错误标志 */
  }
  HAL_FLASH_Lock();
  // __enable_irq();
  Flash_Protect_Lock();

  if (error_count > FLASH_ERROR_COUNT)
  {
    return false;
  }

  return true;
}

unsigned int Read_Flash(unsigned int addr)
{
  return *(volatile unsigned int*)addr;
}

bool Write_Specified_Flash(enum Data_Name name, unsigned int* buffer, unsigned int write_size)
{
  switch(name)
  {
    case CALIBRATION_DATA:
    {
      data_node1.write_data_length = write_size;
      if (data_node1.user_data_length == 0)
      {
        if (!Erase_Flash(FLASH_USER1_ADDR_S, FLASH_USER1_ADDR_E))
        {
          return false;
        }
        data_node1.write_address = FLASH_USER1_ADDR_S;
      }
      else
      {
        data_node1.write_address = data_node1.data_flash_address +
                                   data_node1.user_data_length * 4;
      }
      unsigned int start_address = data_node1.data_flash_address + data_node1.user_data_length * 4;
      unsigned int end_address   = start_address + (4 + data_node1.write_data_length) * 4;
      unsigned int beyond        = start_address % STM_SECTOR_SIZE;
      unsigned int remaining     = STM_SECTOR_SIZE - beyond;
      if (remaining < (4 + data_node1.write_data_length) * 4)
      {
        start_address = (start_address / STM_SECTOR_SIZE + 1) * STM_SECTOR_SIZE;
        end_address   = start_address + 1 * STM_SECTOR_SIZE;
        if (start_address >= FLASH_USER1_ADDR_E)
        {
          unsigned int sector_nums = (4 + data_node1.write_data_length) * 4 / STM_SECTOR_SIZE;
          if (!Erase_Flash(FLASH_USER1_ADDR_S, FLASH_USER1_ADDR_S +
                           sector_nums * STM_SECTOR_SIZE))
          {
            return false;
          }
        }
        else
        {
          if (!Erase_Flash(start_address, start_address))
          {
            return false;
          }
          // if (!Erase_Flash(start_address, end_address))
          // {
          //   return false;
          // }
        }
      }
      data_node1.user_data_date++;
      data_node1.user_data_length = data_node1.write_data_length;
      double_word = ((long long unsigned int)data_node1.user_data_date << 32) |
                      data_node1.user_data_id;
      CheckFLashAddress(&data_node1.write_address);
      if (!Write_Flash(data_node1.write_address, &double_word))
      {
        return false;
      }
      data_node1.write_address += 8;
      CheckFLashAddress(&data_node1.write_address);
      double_word = data_node1.user_data_length;
      if (!Write_Flash(data_node1.write_address, &double_word))
      {
        return false;
      }
      data_node1.write_address += 8;
      CheckFLashAddress(&data_node1.write_address);
      data_node1.data_flash_address = data_node1.write_address;
      for (int i = 0; i < data_node1.write_data_length / 2; i++)
      {
        double_word = buffer[i * 2 + 1];
        double_word = double_word << 32 | buffer[i * 2];
        if (!Write_Flash(data_node1.write_address, &double_word))
        {
          return false;
        }
        data_node1.write_address += 8;
        CheckFLashAddress(&data_node1.write_address);
      }
      unsigned int flash_addr = data_node1.data_flash_address;
      for (int i = 0; i < data_node1.user_data_length; i++)
      {
        CheckFLashAddress(&flash_addr);
        flash_data = *(volatile unsigned int*)flash_addr;
        if (flash_data != buffer[i])
        {
          return false;
        }
        flash_addr += 4;
      }
    }break;
    default: break;
  }

  return true;
}

bool Read_Specified_Flash(enum Data_Name name, unsigned int* buffer, unsigned int buffer_size)
{
  bool isFind = false;

  switch(name)
  {
    case CALIBRATION_DATA:
    {
      data_node1.buffer_address   = buffer;
      data_node1.read_data_length = buffer_size;
      unsigned int start_address = FLASH_USER1_ADDR_S;
      for (unsigned int i = 0; i < (FLASH_USER1_ADDR_E - FLASH_USER1_ADDR_S) / 4; i++)
      {
        flash_data = *(volatile unsigned int*)start_address;
        if (flash_data == name)
        {
          isFind = true;
          if (FLASH_USER1_ADDR_E - start_address < 4 * 4)
          {
            unsigned int offset = FLASH_USER1_ADDR_E - start_address;
            memcpy(&node_buffer[0], (const void*)start_address, offset);
            memcpy(&node_buffer[offset], (const void*)FLASH_USER1_ADDR_S, 16 - offset);
          }
          else
          {
            memcpy(&node_buffer[0], (const void*)start_address, 4 * 4);
          }
          unsigned int write_date = 0;
          memcpy(&write_date, &node_buffer[4], sizeof(int));
          if (data_node1.user_data_date < write_date)
          {
            data_node1.user_data_date     = write_date;
            memcpy(&data_node1.user_data_length, &node_buffer[8], sizeof(int));
            data_node1.data_flash_address = start_address + 4 * 4;
            if (data_node1.data_flash_address > FLASH_USER1_ADDR_E)
            {
              data_node1.data_flash_address =
              data_node1.data_flash_address - FLASH_USER1_ADDR_E + FLASH_USER1_ADDR_S;
            }
          }
        }
        start_address += 4;
      }
      if (isFind)
      {
        unsigned int max = data_node1.read_data_length > data_node1.user_data_length ?
                           data_node1.user_data_length : data_node1.read_data_length;
        unsigned int flash_addr = data_node1.data_flash_address;
        for (unsigned int i = 0; i < max; i++)
        {
          CheckFLashAddress(&flash_addr);
          flash_data = *(volatile unsigned int*)flash_addr;
          data_node1.buffer_address[i] = flash_data;
          if (data_node1.buffer_address[i] == 0xFFFFFFFF)
          {
						isFind = false;
            data_node1.user_data_date     = 0;
            data_node1.user_data_length   = 0;
            data_node1.data_flash_address = FLASH_USER1_ADDR_S;
            return false;
          }
          flash_addr += 4;
        }
      }
    }break;
    default: break;
  }

  if (!isFind)
  {
    return false;
  }

  return true;
}

void Flash_Protect_UnLock(void)
{
  HAL_NVIC_DisableIRQ(TIM14_IRQn);
  HAL_NVIC_DisableIRQ(I2C1_IRQn);
  // HAL_NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
  // HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_DisableIRQ(TIM7_LPTIM2_IRQn);
  HAL_NVIC_DisableIRQ(TIM6_DAC_LPTIM1_IRQn);
  HAL_NVIC_DisableIRQ(TIM16_FDCAN_IT0_IRQn);
  // HAL_NVIC_DisableIRQ(USART1_IRQn);
  HAL_NVIC_DisableIRQ(TIM3_TIM4_IRQn);
}

void Flash_Protect_Lock(void)
{
  HAL_NVIC_EnableIRQ(TIM3_TIM4_IRQn);
  // HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_NVIC_EnableIRQ(TIM16_FDCAN_IT0_IRQn);
  HAL_NVIC_EnableIRQ(TIM6_DAC_LPTIM1_IRQn);
  HAL_NVIC_EnableIRQ(TIM7_LPTIM2_IRQn);
  // HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  // HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  HAL_NVIC_EnableIRQ(I2C1_IRQn);
  HAL_NVIC_EnableIRQ(TIM14_IRQn);
}

void CheckFLashAddress(unsigned int* flash_addr)
{
  if (*flash_addr >= FLASH_USER1_ADDR_E)
  {
    unsigned int offset = *flash_addr - FLASH_USER1_ADDR_E;
    *flash_addr = FLASH_USER1_ADDR_S + offset;
  }
}

void Flash_Write_Wait(void)
{
  for (unsigned int i = 0; i < 10; i++)
  {
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
  }
}