/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_netxduo.h
  * @author  MCD Application Team
  * @brief   NetXDuo applicative header file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_NETXDUO_H__
#define __APP_NETXDUO_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "nx_api.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "main.h"
#include "nxd_dhcp_client.h"
#include "nxd_sntp_client.h"
#include "nxd_dns.h"
#include "nx_driver_emw3080.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define PASSIVE_MODE                    // set FTP passive mode connection
#define SNTP_SERVER_NAME             "time1.google.com"

#define SNTP_UPDATE_EVENT            1
#define SNTP_KOD_EVENT               2

/* Define how often the demo checks for SNTP updates. */
#define PERIODIC_CHECK_INTERVAL      (60 * NX_IP_PERIODIC_RATE)

/* Define how often we check on SNTP server status. */
#define CHECK_SNTP_UPDATES_TIMEOUT   (180 * NX_IP_PERIODIC_RATE)

#define PAYLOAD_SIZE                 (1544)
#define NX_PACKET_POOL_SIZE          (( PAYLOAD_SIZE + sizeof(NX_PACKET)) * 10)
#define DEFAULT_MEMORY_SIZE          (1024)
#define ARP_MEMORY_SIZE              (DEFAULT_MEMORY_SIZE)
#define SNTP_CLIENT_THREAD_MEMORY    (3 * DEFAULT_MEMORY_SIZE)
#define NETX_THREAD_MEMORY           (3 * DEFAULT_MEMORY_SIZE)   

#define IP_MEMORY_SIZE               (3 * DEFAULT_MEMORY_SIZE)
#define NX_APP_MEM_POOL_SIZE         (NX_PACKET_POOL_SIZE + ARP_MEMORY_SIZE + IP_MEMORY_SIZE + NETX_THREAD_MEMORY + SNTP_CLIENT_THREAD_MEMORY + 20*DEFAULT_MEMORY_SIZE)   
   
    /*+ 10*DEFAULT_MEMORY_SIZE*/

#define DEFAULT_NETX_PRIORITY        (10)
#define DEFAULT_PRIORITY             (10)

#define NULL_ADDRESS                 (0)
#define USER_DNS_ADDRESS             IP_ADDRESS(1, 1, 1, 1)

#define DEFAULT_TIMEOUT              (10 * NX_IP_PERIODIC_RATE)

/* EPOCH_TIME_DIFF is equivalent to 70 years in sec
   calculated with www.epochconverter.com/date-difference
   This constant is used to delete difference between :
   Epoch converter (referenced to 1970) and SNTP (referenced to 1900) */
#define EPOCH_TIME_DIFF             (2208988800)
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define PRINT_IP_ADDRESS(addr)    do { \
                                       printf("STM32 %s: %lu.%lu.%lu.%lu \n", #addr, \
                                       (addr >> 24) & 0xff,                          \
                                       (addr >> 16) & 0xff,                          \
                                       (addr >> 8) & 0xff,                           \
                                       (addr & 0xff));                               \
                                     } while(0)

#define PRINT_SNTP_SERVER(addr)  do { \
                                       printf("Client connected to SNTP server: [%lu.%lu.%lu.%lu] \n", \
                                       (addr >> 24) & 0xff,                      \
                                       (addr >> 16) & 0xff,                    \
                                       (addr >> 8) & 0xff,                   \
                                       (addr & 0xff));                     \
                                    } while(0)

#define PRINT_CNX_SUCC()          do { \
                                        printf("SNTP client connected to NTP server : < %s > \n", SNTP_SERVER_NAME);\
                                     } while(0)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
extern UINT MX_NetXDuo_Init(VOID *memory_ptr);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /* __APP_NETXDUO_H__ */
