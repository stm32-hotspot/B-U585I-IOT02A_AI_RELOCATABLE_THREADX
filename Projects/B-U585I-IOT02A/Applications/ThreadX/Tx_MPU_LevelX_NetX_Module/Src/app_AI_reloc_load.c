/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_AI_reloc_load.c
  * @author  SRA Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "stm32u5xx_hal.h"
#include "fx_api.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32u5xx_hal_def.h"

extern const uint8_t            AI_reloc_flash_start_address;
extern const uint8_t            AI_reloc_flash_end_address;

/* Private function prototypes -----------------------------------------------*/
static uint32_t                 GetPage(uint32_t Address);
static uint32_t                 GetBank(uint32_t Address);

uint8_t * Flash_From_Memory (uint8_t * memory_addr, const uint8_t * flash_addr_arg, uint32_t size)
{
uint32_t         FirstPage = 0, NbOfPages = 0, BankNumber = 0;
uint32_t         Address = 0, PageError = 0;
uint8_t        * buffer_ptr = NULL;  
uint8_t        * flash_addr = (uint8_t *)flash_addr_arg;
FLASH_EraseInitTypeDef EraseInitStruct;
/*Variable used for check procedure*/
#if 0
__IO uint32_t    MemoryProgramStatus = 0;
uint32_t         Index= 0;
__IO uint32_t    data32 = 0;  
#endif

  /* check arguments */
  if (memory_addr == NULL || flash_addr_arg == NULL || size == 0) return NULL;

  /* Round flash_addr up to next 8K (0x2000) page to align to next page */
  if ((uint32_t)flash_addr % 0x2000 != 0 )
  {
    flash_addr += 0x2000 - (uint32_t)flash_addr % 0x2000;
  }
  
  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Erase the user Flash area, Get the 1st page to erase */
  FirstPage = GetPage((uint32_t)flash_addr);

  /* Get the number of pages to erase from 1st page */
  NbOfPages = GetPage((uint32_t)flash_addr + size) - FirstPage + 1;

  /* Get the bank */
  BankNumber = GetBank((uint32_t)flash_addr);

  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = BankNumber;
  EraseInitStruct.Page        = FirstPage;
  EraseInitStruct.NbPages     = NbOfPages;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
    return NULL;
  }

  /* Program the user Flash area word by word
    (area defined by  flash_addr and size) ***********/
  
  Address = (uint32_t)flash_addr;
  buffer_ptr = memory_addr;
  
  while (Address < (uint32_t)flash_addr + size)  
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, Address, ((uint32_t)buffer_ptr)) == HAL_OK)
    {
      Address = Address + 16; /* increment flash addr to next quad word*/
      buffer_ptr = buffer_ptr + 16; /* increment RAM addr to next quad word*/
    }
    else
    {
      return NULL;
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  /* Re-enable instruction cache */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    return NULL;
  }
  /* return the aligned flash mem address */
  return flash_addr;
}


uint8_t * Flash_From_File (FX_MEDIA * media_disk, unsigned char * fname, const uint8_t * flash_addr_arg, uint32_t * fsize)
{
uint32_t         attributes, year, month, day;
uint32_t         hour, minute, second;
uint32_t         ret=0;  
FX_FILE          AI_reloc_file;
uint32_t         FirstPage = 0, NbOfPages = 0, BankNumber = 0;
uint32_t         Address = 0, PageError = 0;
uint8_t        * flash_addr = (uint8_t *)flash_addr_arg;
FLASH_EraseInitTypeDef EraseInitStruct;
/*Variable used for check procedure*/
#if 0
__IO uint32_t    MemoryProgramStatus = 0;
uint32_t         Index= 0;
__IO uint32_t    data32 = 0;  
#endif
  
  /* check if file exists, the flash_addr is in range and file fits in flash available space */
  ret = fx_directory_information_get(media_disk, (CHAR *)fname, &attributes, (ULONG *)fsize,
                                      &year, &month, &day, &hour, &minute, &second);  
  
  if ((ret != FX_SUCCESS) || (flash_addr < &AI_reloc_flash_start_address || (flash_addr >= &AI_reloc_flash_end_address) ||
      (*fsize > (&AI_reloc_flash_end_address - flash_addr))))
  {
    return NULL;
  }
  
  ret = fx_file_open(media_disk, &AI_reloc_file, (CHAR *)fname, FX_OPEN_FOR_READ);
  /* open the FS media */
  if (ret != FX_SUCCESS)
  {
    printf ("Error file: %s not existing \n\r", fname);
    return NULL;
  }
                
  /* Seek to the beginning of the module file.  */
  ret =  fx_file_seek(&AI_reloc_file, 0);
  if (ret != FX_SUCCESS)
  {
    fx_file_close(&AI_reloc_file);
    return NULL;
  }   

  /* Round flash_addr up to next 8K (0x2000) page */
  if ((uint32_t)flash_addr % 0x2000 != 0 )
  {
    flash_addr += 0x2000 - (uint32_t)flash_addr % 0x2000;
  }
  printf ("Flashing from file: %s, size: %d, to Flash Addr: 0x%x \n\r", fname, *fsize, flash_addr);  
  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Erase the user Flash area Get the 1st page to erase */
  FirstPage = GetPage((uint32_t)flash_addr);

  /* Get the number of pages to erase from 1st page */
  NbOfPages = GetPage((uint32_t)flash_addr + *fsize) - FirstPage + 1;

  /* Get the bank */
  BankNumber = GetBank((uint32_t)flash_addr);

  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = BankNumber;
  EraseInitStruct.Page        = FirstPage;
  EraseInitStruct.NbPages     = NbOfPages;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
    fx_file_close(&AI_reloc_file);
    return NULL;
  }

  /* Program the user Flash area word by word
    (area defined by  flash_addr and fsize) ***********/

  uint8_t buffer_ptr[16];
  uint32_t act_size;
  
  Address = (uint32_t)flash_addr;  

  while (Address < (uint32_t)flash_addr + *fsize)  
  {
    if (fx_file_read(&AI_reloc_file, buffer_ptr, 16, (ULONG *)&act_size) != FX_SUCCESS) 
    {
      fx_file_close(&AI_reloc_file);
      HAL_FLASH_Lock();
      return NULL;
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, Address, ((uint32_t)buffer_ptr)) == HAL_OK)
    {
      Address = Address + 16; /* increment to next quad word*/
    }
    else
    {
      fx_file_close(&AI_reloc_file);
      return NULL;
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  /* Re-enable instruction cache */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    fx_file_close(&AI_reloc_file);
    return NULL;
  }
  fx_file_close(&AI_reloc_file);
#if 0 
  /* Check if the programmed data is OK
      MemoryProgramStatus = 0: data programmed correctly
      MemoryProgramStatus != 0: number of words not programmed correctly ******/
  Address = FLASH_USER_START_ADDR;
  MemoryProgramStatus = 0x0;

  while (Address < FLASH_USER_END_ADDR)
  {
    for(Index = 0; Index<4; Index++)
    {
      data32 = *(uint32_t*)Address;
      if(data32 != QuadWord[Index])
      {
        MemoryProgramStatus++;
      }
      Address +=4;
    }
  }

  /*Check if there is an issue to program data*/
  if (MemoryProgramStatus == 0)
  {
    /* No error detected. Switch on LED1*/
    BSP_LED_On(LED1);
  }
  else
  {
    /* Error detected. Switch on LED2*/
    BSP_LED_On(LED2);
  }
#endif
  return flash_addr;
}

/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

static uint32_t GetBank(uint32_t Addr)
{
  return FLASH_BANK_1;
}

