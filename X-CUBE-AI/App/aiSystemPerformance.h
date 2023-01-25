/**
 ******************************************************************************
 * @file    aiSystemPerformance.h
 * @author  MCD/AIS Team
 * @brief   Entry points for AI system performance application
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019,2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software is licensed under terms that can be found in the LICENSE file in
 * the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef __AI_SYSTEM_PERFORMANCE_H__
#define __AI_SYSTEM_PERFORMANCE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t _ai_network_data_weights_get_func(void);    
#define AI_NETWORK_DATA_WEIGHTS_GET_FUNC()   _ai_network_data_weights_get_func()  
  
int aiSystemPerformanceInit(void);
int aiSystemPerformanceProcess(void);
void aiSystemPerformanceDeInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __AI_SYSTEM_PERFORMANCE_H__ */
