/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"
#include "app_filex.h"
#include "app_netxduo.h"
#include <ai_reloc_network.h>
#include <network_img_rel.h>  

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
PROCESSING_NOT_STARTED    = 99,
WRITING_TO_READWRITE      = 88,
WRITING_TO_READONLY       = 77,
READING_FROM_READWRITE    = 66,
READING_FROM_READONLY     = 55,
PROCESSING_FINISHED       = 44,
MEMORY_ACCESS_VIOLATION   = 45
} ProgressState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEFAULT_STACK_SIZE         (3*1024)
#define MODULE_DATA_SIZE           (32*1024)
#define OBJECT_MEM_SIZE            (16*1024)

#define READONLY_REGION            (0x20010000)
#define READWRITE_REGION           (0x20010100)
#define SHARED_MEM_SIZE            (0xFF)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Define the ThreadX object control blocks */
static TX_THREAD               ModuleManagerThreadHandler;
static TX_THREAD               AIThreadHandler;

static TXM_MODULE_INSTANCE     ModuleOne;
static TX_QUEUE                ResidentRxQueue;
static TX_QUEUE                ResidentTxQueue;

/* Define the module data pool area. */
UCHAR                   module_data_area[MODULE_DATA_SIZE];

/* Define the object pool area.  */
UCHAR                   object_memory[OBJECT_MEM_SIZE];

/* Define the NetX pool buffer */
#if defined ( __ICCARM__ )
#pragma data_alignment=4
#endif
static UCHAR            nx_byte_pool_buffer[NX_APP_MEM_POOL_SIZE];
#if defined ( __ICCARM__ )
#pragma data_alignment=4
#endif
static UCHAR            tx_byte_pool_buffer[TX_APP_MEM_POOL_SIZE];
#if defined ( __ICCARM__ )
#pragma data_alignment=4
#endif
static UCHAR            AI_byte_pool_buffer[AI_APP_MEM_POOL_SIZE];

static TX_BYTE_POOL     nx_app_byte_pool;

static TX_BYTE_POOL     ModuleManagerBytePool;

TX_BYTE_POOL            AIBytePool;

extern TX_SEMAPHORE     Semaphore_SNTP_ready;
//extern const uint32_t   AI_reloc_flash_start_address;
extern const uint8_t    AI_reloc_flash_start_address;
extern const uint8_t    AI_reloc_flash_end_address;
extern uint8_t        * Flash_From_File (FX_MEDIA * media_disk, unsigned char * fname, const uint8_t * flash_addr, uint32_t * fsize);

uint8_t * network_real_flash_addr = 0;
uint8_t * weights_real_flash_addr = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
VOID pretty_msg(char *p_msg, ULONG r_msg);
VOID ModuleManagerThreadEntry(ULONG thread_input);
VOID AIThreadEntry(ULONG thread_input);
VOID module_fault_handler(TX_THREAD *thread, TXM_MODULE_INSTANCE *module);
/* USER CODE END PFP */

/**
  * @brief  Define the initial system.
  * @param  first_unused_memory : Pointer to the first unused memory
  * @retval None
  */
VOID tx_application_define(VOID *first_unused_memory)
{
  CHAR *MM_mem_pointer;
  CHAR *AI_mem_pointer;  
  
  if (tx_byte_pool_create(&ModuleManagerBytePool, "Module Manager Byte Pool", tx_byte_pool_buffer, TX_APP_MEM_POOL_SIZE) != TX_SUCCESS)
  {
    /* USER CODE BEGIN TX_Byte_Pool_Error */
    Error_Handler();
    /* USER CODE END TX_Byte_Pool_Error */
  }

  if (tx_byte_pool_create(&AIBytePool, "AI Byte Pool", AI_byte_pool_buffer, AI_APP_MEM_POOL_SIZE) != TX_SUCCESS)
  {
    /* USER CODE BEGIN TX_Byte_Pool_Error */
    Error_Handler();
    /* USER CODE END TX_Byte_Pool_Error */
  }
  
    /* Allocate the stack for AI Thread.  */
    if (tx_byte_allocate(&AIBytePool, (VOID **) &AI_mem_pointer,
                         2*DEFAULT_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
      Error_Handler();
    }  

    /* Create the AI thread */
    if (tx_thread_create(&AIThreadHandler, "AI Thread", AIThreadEntry, 0,
                         AI_mem_pointer, 2*DEFAULT_STACK_SIZE,
                         AI_THREAD_PRIO, AI_THREAD_PREEMPTION_THRESHOLD,
                         TX_NO_TIME_SLICE, TX_DONT_START) != TX_SUCCESS)
    {
      Error_Handler();
    }
        
    /* Allocate the stack for Module Manager Thread.  */
    if (tx_byte_allocate(&ModuleManagerBytePool, (VOID **) &MM_mem_pointer,
                         2*DEFAULT_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
    {
      Error_Handler();
    }  
  
    /* Create Module Manager Thread.  */
    if (tx_thread_create(&ModuleManagerThreadHandler, "Module Manager Thread", ModuleManagerThreadEntry, 0,
                         MM_mem_pointer, 2*DEFAULT_STACK_SIZE,
                         MODULE_MANAGER_THREAD_PRIO, MODULE_MANAGER_THREAD_PREEMPTION_THRESHOLD,
                         TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
      Error_Handler();
    }
    
    /* Allocate the stack for ResidentRxQueue.  */
    if (tx_byte_allocate(&ModuleManagerBytePool, (VOID **) &MM_mem_pointer,
                         16 * sizeof(ULONG), TX_NO_WAIT) != TX_SUCCESS)
    {
      Error_Handler();
    }
    
    /* Create the ResidentRxQueue */
    if (tx_queue_create(&ResidentRxQueue, "Resident Rx Queue",TX_1_ULONG,
                        MM_mem_pointer, 16 * sizeof(ULONG)) != TX_SUCCESS)
    {
      Error_Handler();
    }
  
    /* Allocate the stack for ResidentTxQueue.  */
    if (tx_byte_allocate(&ModuleManagerBytePool, (VOID **) &MM_mem_pointer,
                         16 * sizeof(ULONG), TX_NO_WAIT) != TX_SUCCESS)
    {
      Error_Handler();
    }
    
    /* Create the ResidentTxQueue */
    if (tx_queue_create(&ResidentTxQueue, "Resident Tx Queue",TX_1_ULONG,
                        MM_mem_pointer, 16 * sizeof(ULONG)) != TX_SUCCESS)
    {
      Error_Handler();
    } 
}

/**
  * @brief  MX_ThreadX_Init
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN  1 */
/**
  * @brief  Module Manager main thread.
  * @param  thread_input: thread id
  * @retval none
  */
VOID ModuleManagerThreadEntry(ULONG thread_input)
{
  UINT   status;
  CHAR   p_msg[64];
  ULONG  r_msg = PROCESSING_NOT_STARTED;
  ULONG  module_properties;
  VOID *memory_ptr;

  printf ("\n\r==============================================================\n\r");
  printf (   "|                  Demo AI & Module Started                   |\n\r");  
  printf (    "==============================================================\n\r");
  
  if (tx_byte_pool_create(&nx_app_byte_pool, "Nx App memory pool", nx_byte_pool_buffer, NX_APP_MEM_POOL_SIZE) != TX_SUCCESS)
  {
    /* USER CODE BEGIN NX_Byte_Pool_Error */
    Error_Handler();
    /* USER CODE END NX_Byte_Pool_Error */
  }
  else
  {
    /* USER CODE BEGIN TX_Byte_Pool_Success */

    /* USER CODE END TX_Byte_Pool_Success */

    memory_ptr = (VOID *)&nx_app_byte_pool;
    status = MX_NetXDuo_Init(memory_ptr);
    if (status != NX_SUCCESS)
    {
      /* USER CODE BEGIN  MX_NetXDuo_Init_Error */
      Error_Handler();
      /* USER CODE END  MX_NetXDuo_Init_Error */
    }
    /* USER CODE BEGIN  MX_NetXDuo_Init_Success */
  /* wait until an IP address is obtained */
    if(tx_semaphore_get(&Semaphore_SNTP_ready, TX_WAIT_FOREVER) != TX_SUCCESS)
    {
      Error_Handler();
    }
    /* USER CODE END MX_NetXDuo_Init_Success */

  }  

#if defined (MODULE_LOAD_FROM_FILE) || (AI_NETWORK_LOAD_FROM_FILE)
  if (App_FileX_Init() != FX_SUCCESS) 
  {
    return;
  }    
#endif  
/* use this function only if FTP server is not available and and the file is not already on the file system, so Module.bin file is to be created from a previously Jtag flashed module 
instead of downloading it from FTP srv */  
//  App_FileX_Create_Module_File(MODULE_FILE_NAME);    
//#endif

#ifdef  MODULE_LOAD_FROM_FILE  
    status = Ftp_File_Get(MODULE_FILE_NAME, &media_disk);
    if (status != TX_SUCCESS) {
      printf ("Module %s dowload from FTP failed\n", MODULE_FILE_NAME);
      printf ("Checking if module file is already present on NOR file system ...\n");
        
      if (App_FileX_Check_Binary_File(MODULE_FILE_NAME, TXM_MODULE_ID)) {      
        printf ("Module file not present on file system: demo end\n");
        App_FileX_DeInit();   
        return;
      } else {
        printf ("Module file already present on file system: demo it\n");
      }
    }  
#endif 

#ifdef AI_NETWORK_LOAD_FROM_FILE             
  status = Ftp_File_Get(AI_NETWORK_FILE_NAME, &media_disk);
  if (status != TX_SUCCESS) {
    printf ("Network %s dowload from FTP failed\n", AI_NETWORK_FILE_NAME);
    printf ("Checking if AI Network file is already present on NOR file system ...\n"); 
    /* temporary redefined here as is not defined in any .h AI file */
    #define AI_RELOC_MAGIC       (0x4E49424E)     
    if (App_FileX_Check_Binary_File(AI_NETWORK_FILE_NAME, AI_RELOC_MAGIC)) {      
      printf ("AI Network file not present on file system: demo end\n");
      App_FileX_DeInit();   
      return;
    } else {
      printf ("AI Network file already present on file system: demo it\n");
    }
  }

  /* copy from File to Flash the AI network file  */
  uint32_t network_size = 0, weights_size = 0;
  network_real_flash_addr = Flash_From_File(&media_disk, AI_NETWORK_FILE_NAME, &AI_reloc_flash_start_address, &network_size);
  
  status = Ftp_File_Get(AI_WEIGHTS_FILE_NAME, &media_disk);
  if (status != TX_SUCCESS) {
    printf ("AI Weights %s download from FTP failed\n", AI_WEIGHTS_FILE_NAME);
    printf ("Checking if AI Weights file is already present on NOR file system ...\n");    
    if (App_FileX_Check_Binary_File(AI_WEIGHTS_FILE_NAME, NULL)) {      
        printf ("AI Weights file not present on file system: demo end\n");
        App_FileX_DeInit(); 
        return;
    } else {
        printf ("AI Weights file already present on file system: demo it\n");
    }    
  }
  
  /* copy from File to Flash the AI weights file  */    
  weights_real_flash_addr = Flash_From_File(&media_disk, AI_WEIGHTS_FILE_NAME, &AI_reloc_flash_start_address + network_size, &weights_size);
  
#endif
  
  /* start the AI processing thread */
  tx_thread_resume(&AIThreadHandler);    
  
  /* uncomment to suspend the current thread to leave AI thread running alone (without Module; just for debugging) */
  //tx_thread_suspend(tx_thread_identify());
  
  /* Initialize the module manager. */
  status = txm_module_manager_initialize((VOID *) module_data_area, MODULE_DATA_SIZE);

  if(status != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Create a pool for module objects. */
  status = txm_module_manager_object_pool_create(object_memory, OBJECT_MEM_SIZE);

  if(status != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Register a fault handler. */
  status = txm_module_manager_memory_fault_notify(module_fault_handler);

  if(status != TX_SUCCESS)
  {
    Error_Handler();
  } 

#ifdef MODULE_LOAD_FROM_FILE 
  
 /* Load the module from the specified FILE and execute it in SRAM */
  status = txm_module_manager_file_load(&ModuleOne, "Module One", &media_disk, MODULE_FILE_NAME);  // OK
  
#else
#ifdef MODULE_EXECUTE_IN_PLACE
  /* execute the module from the specified FLASH address, it needs to enable module flashing from IDE -> Option -> Debugger -> DownloadExtra Image */
  /* in this case the Module execution address and Module load/compile address are the same: "in place execution" */
  status = txm_module_manager_in_place_load(&ModuleOne, "Module One", (VOID *) MODULE_FLASH_ADDRESS);  // OK
//  status = txm_module_manager_absolute_load(&ModuleOne, "Module One", (VOID *) MODULE_FLASH_ADDRESS);    
#else  
 /* Load the module from the specified FLASH address and execute it in SRAM */ 
  status = txm_module_manager_memory_load(&ModuleOne, "Module One", (VOID *) MODULE_FLASH_ADDRESS);  // OK
#endif  
#endif
    
  if(status != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Enable shared memory region for module with read-only access permission. */
  status = txm_module_manager_external_memory_enable(&ModuleOne, (void*)READONLY_REGION, SHARED_MEM_SIZE, TXM_MODULE_ATTRIBUTE_READ_ONLY);

  if(status != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Enable shared memory region for module with read and write access permission. */
  status = txm_module_manager_external_memory_enable(&ModuleOne, (void*)READWRITE_REGION, SHARED_MEM_SIZE, TXM_MODULE_ATTRIBUTE_READ_WRITE);

  if(status != TX_SUCCESS)
  {
    Error_Handler();
  }
#ifdef CRC_EN
  /* Enable CRC memory mapped AHB1 bus region 0x40023000 for module with read and write access permission. */
  status = txm_module_manager_external_memory_enable(&ModuleOne, (void*)CRC_BASE_NS, 0x400233FF-CRC_BASE_NS, TXM_MODULE_ATTRIBUTE_READ_WRITE);

  if(status != TX_SUCCESS)
  {
    Error_Handler();
  } 
  /* Enable RCC memory mapped AHB3 bus region 0x46020C00 for module with read and write access permission. (To En/Dis CRC CK) */
  status = txm_module_manager_external_memory_enable(&ModuleOne, (void*)RCC_BASE_NS, 0x46020FFF-RCC_BASE_NS, TXM_MODULE_ATTRIBUTE_READ_WRITE);

  if(status != TX_SUCCESS)
  {
    Error_Handler();
  }    
#endif  
  /* Get module properties. */
  status = txm_module_manager_properties_get(&ModuleOne, &module_properties);

  if(status != TX_SUCCESS)
  {
    Error_Handler();
  }

  /* Print loaded module info */
#ifdef MODULE_LOAD_FROM_FILE 
  printf("Module <%s> is loaded from file %s executed from address: 0x%08X \n", ModuleOne.txm_module_instance_name, MODULE_FILE_NAME, ModuleOne.txm_module_instance_code_start);
#else  
  printf("Module <%s> is loaded from address 0x%08X executed from address: 0x%08X \n", ModuleOne.txm_module_instance_name, MODULE_FLASH_ADDRESS, ModuleOne.txm_module_instance_code_start);
#endif
  printf("Module code section size: %i bytes, data section size: %i\n", (int)ModuleOne.txm_module_instance_code_size, (int)ModuleOne.txm_module_instance_data_size);
  printf("Module Attributes:\n");
  printf("  - Compiled for %s compiler\n", ((module_properties >> 25) == 1)? "STM32CubeIDE (GNU)" : ((module_properties >> 24) == 1)? "ARM KEIL" : "IAR EW");
  printf("  - Shared/external memory access is %s\n", ((module_properties & 0x04) == 0)? "Disabled" : "Enabled");
  printf("  - MPU protection is %s\n", ((module_properties & 0x02) == 0)? "Disabled" : "Enabled");
  printf("  - %s mode execution is enabled for the module\n\n", ((module_properties & 0x01) == 0)? "Privileged" : "User");

  /* Start the modules. */
  status = txm_module_manager_start(&ModuleOne);
  if(status != TX_SUCCESS)
  {
    Error_Handler();
  }

#if defined (MODULE_LOAD_FROM_FILE) || defined (AI_NETWORK_LOAD_FROM_FILE)  
  App_FileX_DeInit();   // deinit the file system
#endif    
  
  printf("Module Manager execution is started\n");

  /* Send to Module the start msg */
  ULONG tx_msg = 0x55;
  tx_queue_send(&ResidentTxQueue, &tx_msg, TX_NO_WAIT);
  
  /* Get Module's progress messages */
  while(1)
  {
    if(tx_queue_receive(&ResidentRxQueue, &r_msg, TX_WAIT_FOREVER) == TX_SUCCESS)    
    {
      /* Convert the message to a user friendly string */
      pretty_msg(p_msg, r_msg);

      printf("Module is executing: %s\n", p_msg);

      /* Check if the last executed Module operation resulted in memory violation */
      if (r_msg == MEMORY_ACCESS_VIOLATION)
      {        
        break;
      }
    }
  }
  
  /* Stopping the modules triggers my_module_stop() function in module */
  printf ("Stopping the Module ...\n\r");
  status = txm_module_manager_stop(&ModuleOne);
  if(status != TX_SUCCESS)
  {
    Error_Handler();
  }
    /* the module send back the PROCESSING_FINISHED msg so is joined) */
    if(tx_queue_receive(&ResidentRxQueue, &r_msg, TX_WAIT_FOREVER) == TX_SUCCESS)    
    {
      /* Convert the message to a user friendly string */
      pretty_msg(p_msg, r_msg);

      printf("Module is executing: %s\n", p_msg);

      if (r_msg == PROCESSING_FINISHED)  
      {
       /* Unload the joined modules. */
       status = txm_module_manager_unload(&ModuleOne);
      }
      if(status != TX_SUCCESS)
      {
        Error_Handler();
      }
   }
  /* Toggle green LED to indicated success of operations */
  while(1) {
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    tx_thread_sleep(50);
  }
}

/**
  * @brief  AI main thread.
  * @param  thread_input: thread id
  * @retval none
  */
VOID AIThreadEntry(ULONG thread_input)
{ 
  uint32_t iter = 0;
  
  MX_X_CUBE_AI_Init();
  
  while (1)
  {
    MX_X_CUBE_AI_Process();
    printf ("===> Iter: %d\n\r", ++iter);
    tx_thread_sleep(100);
//    tx_thread_suspend(tx_thread_identify());   to stop the thread after first iter
  }  
}


VOID module_fault_handler(TX_THREAD *thread, TXM_MODULE_INSTANCE *module)
{
  ULONG s_msg;  
  /* send msg to MM thread to chain the fault event */ 
  s_msg = MEMORY_ACCESS_VIOLATION;
  tx_queue_send(&ResidentRxQueue, &s_msg, TX_NO_WAIT);
}

VOID pretty_msg(char *p_msg, ULONG r_msg)
{
  memset(p_msg, 0, 64);

  switch(r_msg)
  {
  case WRITING_TO_READWRITE:
    memcpy(p_msg, "Writing to ReadWrite Region", 27);
    break;
  case WRITING_TO_READONLY:
    memcpy(p_msg, "Writing to ReadOnly Region", 26);
    break;
  case READING_FROM_READWRITE:
    memcpy(p_msg, "Reading from ReadWrite Region", 29);
    break;
  case READING_FROM_READONLY:
    memcpy(p_msg, "Reading from ReadOnly Region", 28);
    break;
  case PROCESSING_FINISHED:
    memcpy(p_msg, "All operations were done", 24);
    break;
  case MEMORY_ACCESS_VIOLATION:
    memcpy(p_msg, "Memory access violation", 23);
    break;    
  default:
    memcpy(p_msg, "Invalid option", 14);
    break;
  }
}

/* USER CODE END  1 */

