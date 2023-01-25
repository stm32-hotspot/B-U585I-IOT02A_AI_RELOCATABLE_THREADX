/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_module.c
  * @author  MCD Application Team
  * @brief   ThreadX Module applicative file
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
/* Specify that this is a module! */
#define TXM_MODULE

/* Include the ThreadX module header. */
#include "txm_module.h"
#include "stm32u5xx_hal.h"
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
PROCESSING_FINISHED       = 44
} ProgressState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEFAULT_STACK_SIZE         (3*1024)
#define DEFAULT_BYTE_POOL_SIZE     (9120)
#define DEFAULT_BLOCK_POOL_SIZE    (1024)

#define READONLY_REGION            (0x20010000)
#define READWRITE_REGION           (0x20010100)

#define MAIN_THREAD_PRIO                         (2)
#define MAIN_THREAD_PREEMPTION_THRESHOLD         MAIN_THREAD_PRIO
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define UNUSED(x) (void)(x)

/* Define the pool space in the bss section of the module. ULONG is used to
   get word alignment. */
#if defined(__GNUC__) || defined(__CC_ARM)
ULONG  default_module_pool_space[DEFAULT_BYTE_POOL_SIZE / 4] __attribute__ ((aligned(32)));
#else /* __ICCARM__ */
_Pragma("data_alignment=32") ULONG  default_module_pool_space[DEFAULT_BYTE_POOL_SIZE / 4];
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static TX_THREAD             *	MainThread;
static TX_BYTE_POOL        * 	ModuleBytePool;
static TX_BLOCK_POOL      * 	ModuleBlockPool;
static TX_QUEUE                * 	Resident_Tx_Queue;
static TX_QUEUE                * 	Resident_Rx_Queue;

static CRC_HandleTypeDef 	hcrc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void MainThread_Entry(ULONG thread_input);
void Error_Handler(void);
static void MX_CRC_Init(void);
/* USER CODE END PFP */

/**
  * @brief  Module entry function.
  * @param  id : Module ID
  * @retval None
  */
void    default_module_start(ULONG id)
{
  CHAR    * pointer;
  CHAR    * byte_pointer;
#ifdef CRC_EN

#define BUFFER_SIZE    114
unsigned const int aDataBuffer[BUFFER_SIZE] =
{
  0x00001021, 0x20423063, 0x408450a5, 0x60c670e7, 0x9129a14a, 0xb16bc18c,
  0xd1ade1ce, 0xf1ef1231, 0x32732252, 0x52b54294, 0x72f762d6, 0x93398318,
  0xa35ad3bd, 0xc39cf3ff, 0xe3de2462, 0x34430420, 0x64e674c7, 0x44a45485,
  0xa56ab54b, 0x85289509, 0xf5cfc5ac, 0xd58d3653, 0x26721611, 0x063076d7,
  0x569546b4, 0xb75ba77a, 0x97198738, 0xf7dfe7fe, 0xc7bc48c4, 0x58e56886,
  0x78a70840, 0x18612802, 0xc9ccd9ed, 0xe98ef9af, 0x89489969, 0xa90ab92b,
  0x4ad47ab7, 0x6a961a71, 0x0a503a33, 0x2a12dbfd, 0xfbbfeb9e, 0x9b798b58,
  0xbb3bab1a, 0x6ca67c87, 0x5cc52c22, 0x3c030c60, 0x1c41edae, 0xfd8fcdec,
  0xad2abd0b, 0x8d689d49, 0x7e976eb6, 0x5ed54ef4, 0x2e321e51, 0x0e70ff9f,
  0xefbedfdd, 0xcffcbf1b, 0x9f598f78, 0x918881a9, 0xb1caa1eb, 0xd10cc12d,
  0xe16f1080, 0x00a130c2, 0x20e35004, 0x40257046, 0x83b99398, 0xa3fbb3da,
  0xc33dd31c, 0xe37ff35e, 0x129022f3, 0x32d24235, 0x52146277, 0x7256b5ea,
  0x95a88589, 0xf56ee54f, 0xd52cc50d, 0x34e224c3, 0x04817466, 0x64475424,
  0x4405a7db, 0xb7fa8799, 0xe75ff77e, 0xc71dd73c, 0x26d336f2, 0x069116b0,
  0x76764615, 0x5634d94c, 0xc96df90e, 0xe92f99c8, 0xb98aa9ab, 0x58444865,
  0x78066827, 0x18c008e1, 0x28a3cb7d, 0xdb5ceb3f, 0xfb1e8bf9, 0x9bd8abbb,
  0x4a755a54, 0x6a377a16, 0x0af11ad0, 0x2ab33a92, 0xed0fdd6c, 0xcd4dbdaa,
  0xad8b9de8, 0x8dc97c26, 0x5c644c45, 0x3ca22c83, 0x1ce00cc1, 0xef1fff3e,
  0xdf7caf9b, 0xbfba8fd9, 0x9ff86e17, 0x7e364e55, 0x2e933eb2, 0x0ed11ef0
};

/* Expected CRC Value */
unsigned int uwExpectedCRCValue = 0x379E9F06;  

#endif
  /* Allocate all the objects. In MPU mode, modules cannot allocate control blocks within
  their own memory area so they cannot corrupt the resident portion of ThreadX by overwriting
  the control block(s).  */
  txm_module_object_allocate((void*)&MainThread, sizeof(TX_THREAD));
  txm_module_object_allocate((void*)&ModuleBytePool, sizeof(TX_BYTE_POOL));
  txm_module_object_allocate((void*)&ModuleBlockPool, sizeof(TX_BLOCK_POOL));
  
  /* Create a byte memory pool from which to allocate the thread stacks.  */
  tx_byte_pool_create(ModuleBytePool, "Module Byte Pool", (UCHAR*)default_module_pool_space, DEFAULT_BYTE_POOL_SIZE);
  
  /* Allocate the stack for thread 0.  */
  tx_byte_allocate(ModuleBytePool, (VOID **) &pointer, DEFAULT_STACK_SIZE, TX_NO_WAIT);
/*=============================================================================*/

#ifdef CRC_EN
  MX_CRC_Init(); 
  /* Compute the CRC of "aDataBuffer" */
  volatile unsigned int uwCRCValue = HAL_CRC_Calculate(&hcrc, (unsigned int *)aDataBuffer, BUFFER_SIZE);
//0x4bb7ed3f
  /* Compare the CRC value to the Expected one */
  if (uwCRCValue != uwExpectedCRCValue)  
  {
    /* Wrong CRC value */
    return;
  }
  else
  {
    /* Right CRC value */
//    BSP_LED_On(LED1);
  }  
#endif  
/*=============================================================================*/   


//  MX_X_CUBE_AI_Init();

//**********************************************
  
  /* Create the main thread.  */
  tx_thread_create(MainThread, "Module Main Thread", MainThread_Entry, 0,
                   pointer, DEFAULT_STACK_SIZE,
                   MAIN_THREAD_PRIO, MAIN_THREAD_PREEMPTION_THRESHOLD, TX_NO_TIME_SLICE, TX_AUTO_START);
  
  /* Allocate the memory for a small block pool. */
  tx_byte_allocate(ModuleBytePool, (VOID **) &pointer, DEFAULT_BLOCK_POOL_SIZE, TX_NO_WAIT);
  byte_pointer = pointer;
  
  /* Create a block memory pool. */
  tx_block_pool_create(ModuleBlockPool, "Module Block Pool", sizeof(ULONG), pointer, DEFAULT_BLOCK_POOL_SIZE);
  
  /* Allocate a block. */
  tx_block_allocate(ModuleBlockPool, (VOID **) &pointer, TX_NO_WAIT);
  
  /* Release the block back to the pool. */
  tx_block_release(pointer);
 
  /* Delete a block memory pool. */
  tx_block_pool_delete(ModuleBlockPool);  
  
  /* Release the memory for a small block pool. */
  tx_byte_release((VOID *) byte_pointer);  
  
}

/**
  * @brief  Module main thread.
  * @param  thread_input: thread id
  * @retval none
  */
void MainThread_Entry(ULONG thread_input)
{
  UINT status;
  ULONG s_msg;
  ULONG rx_msg;  
  ULONG readbuffer;

  /* Request access to the queues from the module manager */
  status = txm_module_object_pointer_get(TXM_QUEUE_OBJECT, "Resident Rx Queue", (VOID **)&Resident_Rx_Queue);
  if(status)
  {
    Error_Handler();
  }

  status = txm_module_object_pointer_get(TXM_QUEUE_OBJECT, "Resident Tx Queue", (VOID **)&Resident_Tx_Queue);
  if(status)
  {
    Error_Handler();
  }  
 
  /* Waiting for start msg from Mod Manager app */
  tx_queue_receive(Resident_Tx_Queue, &rx_msg, TX_WAIT_FOREVER);
  if (rx_msg != 0x55) Error_Handler();
  
   /* Writing to write and read region */
  s_msg = WRITING_TO_READWRITE;
  tx_queue_send(Resident_Rx_Queue, &s_msg, TX_NO_WAIT);
  *(ULONG *)READWRITE_REGION = 0xABABABAB;
  tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10);

  /* WReading from write and read region */
  s_msg = READING_FROM_READWRITE;
  tx_queue_send(Resident_Rx_Queue, &s_msg, TX_NO_WAIT);
  readbuffer = *(ULONG*)READWRITE_REGION;
  tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10);

  /* Reading from read only region */
  s_msg = READING_FROM_READONLY;
  tx_queue_send(Resident_Rx_Queue, &s_msg, TX_NO_WAIT);
  readbuffer = *(ULONG*)READONLY_REGION;
  tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10);

  /* Writing to read only region should trigger the cb module_fault_handler() in resident app */
  *(ULONG *)READONLY_REGION = 0xABABABAB;
  tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10);

  /* Notify module manager about job finish */
  s_msg = PROCESSING_FINISHED;
  tx_queue_send(Resident_Rx_Queue, &s_msg, TX_NO_WAIT);

  /* Suppress unused variable warning */
  UNUSED(readbuffer);

  /* Stay here, waiting for the module manager to stop and unloading the module*/
  while(1)
  {
    tx_thread_sleep(100);
  }
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* Nothing to do, block here */
  tx_thread_sleep(TX_WAIT_FOREVER);
}
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */
    
  /* USER CODE END CRC_Init 2 */
}
void HAL_CRC_MspInit(CRC_HandleTypeDef* hcrc)
{
  if(hcrc->Instance==CRC)
  {
  /* USER CODE BEGIN CRC_MspInit 0 */

  /* USER CODE END CRC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CRC_CLK_ENABLE();
  /* USER CODE BEGIN CRC_MspInit 1 */

  /* USER CODE END CRC_MspInit 1 */
  }
}

/* stop function triggered by txm_module_manager_stop() in Mod Mngr */
void my_module_stop(ULONG thread_input)
{
  ULONG s_msg;

  /* release the allocated module objects */
  txm_module_object_deallocate((void*)&MainThread);
  txm_module_object_deallocate((void*)&ModuleBytePool);
  txm_module_object_deallocate((void*)&ModuleBlockPool);   
  
  /* signal the Module Mngr app to join */
  s_msg = PROCESSING_FINISHED;
  tx_queue_send(Resident_Rx_Queue, &s_msg, TX_NO_WAIT);     
}
