/********************************************************************************
*
* Copyright (c) 2018 STMicroelectronics - All Rights Reserved
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*******************************************************************************/


#ifndef SPD_PLATFORM_H_
#define SPD_PLATFORM_H_

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "spd_types.h"

//#define SPD_PROFILE
#define PLATFORM_CALLBACK() (void)0
//#define MULTI_HUMAN_DETECTION 1

#if defined(TRACE)
extern int SPDLoggingLevel;
extern int uart_printf(const char *msg, ...);

/**
* @def SPD_DEBUG
* @brief Tracing function for debug purpose
*
* Defining TRACE in compiler command line will enable tracing feature (through #SPD_DEBUG macro) for debug purpose.
* This allows to print any formatted text in STM32 ST-Link utility (printf via SWO viewer).
*/
#define SPD_DEBUG(spd_logging_level,fmt, ... ) \
		if(spd_logging_level<=SPDLoggingLevel) \
			/*trace_printf("SPD %s at line %d \t" fmt "\n", __func__,__LINE__, ##__VA_ARGS__);*/ \
			uart_printf("SPD \t" fmt "\n", ##__VA_ARGS__); \
		else\
			(void)0;
#else
#define SPD_DEBUG(fmt, ... ) (void)0
#endif

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif


//#if defined(SPD_PROFILE)
//extern int profiling__exec_cycle_count;
//extern int profiling__exec_duration;
//extern uint32_t HAL_GetTick(void);
//#define GET_PROFILING_TIME_STAMP()  (int32_t)HAL_GetTick()
//
//// Defines to be added for cycle count
//#define DEMCR_TRCENA    0x01000000
//
///* Core Debug registers */
//#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
//#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
//#define CYCCNTENA       (1<<0)
//#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
//#define CPU_CYCLES      *DWT_CYCCNT
//
//// Functions to be added for cycle count
//static inline void stopwatch_reset(void)
//{
//    /* Enable DWT */
//    DEMCR |= DEMCR_TRCENA;
//    *DWT_CYCCNT = 0;
//    /* Enable CPU cycle counter */
//    DWT_CTRL |= CYCCNTENA;
//}
//
//static inline uint32_t stopwatch_getticks(void)
//{
//    return CPU_CYCLES;
//}
//
//#define reset_cycle_counts() stopwatch_reset()
//#define get_cycle_counts() profiling__exec_cycle_count = stopwatch_getticks()
//#define reset_profiling_duration() profiling__exec_duration = GET_PROFILING_TIME_STAMP()
//#define get_profiling_duration() profiling__exec_duration = GET_PROFILING_TIME_STAMP() - profiling__exec_duration
//
//#else
//#define GET_PROFILING_TIME_STAMP() 0
//#define stopwatch_reset() (void)0
//#define stopwatch_getticks() 0
//
//#define reset_cycle_counts() stopwatch_reset()
//#define get_cycle_counts() 0
//#define reset_profiling_duration() GET_PROFILING_TIME_STAMP()
//#define get_profiling_duration() 0
//#endif



#endif /* SPD_PLATFORM_H_ */
