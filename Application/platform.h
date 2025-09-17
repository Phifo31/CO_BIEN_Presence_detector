/*******************************************************************************
*
* Copyright (c) 2020 STMicroelectronics - All Rights Reserved
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*******************************************************************************/

#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>

// Code Profiling
//#define SPD_PROFILE
#if defined(SPD_PROFILE)

extern int profiling__exec_cycle_count;
extern int profiling__exec_duration;

extern uint32_t HAL_GetTick(void);
//#define GET_PROFILING_TIME_STAMP()  (int32_t)HAL_GetTick()
#define GET_TIME_STAMP()  (int32_t)HAL_GetTick()

// Defines to be added for cycle count
#define DEMCR_TRCENA    0x01000000

/* Core Debug registers */
#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT

static inline void stopwatch_reset(void)
{
    /* Enable DWT */
    DEMCR |= DEMCR_TRCENA;
    *DWT_CYCCNT = 0;
    /* Enable CPU cycle counter */
    DWT_CTRL |= CYCCNTENA;
}

static inline uint32_t stopwatch_getticks(void)
{
    return CPU_CYCLES;
}

#define reset_cycle_counts() stopwatch_reset()
#define get_cycle_counts() profiling__exec_cycle_count = stopwatch_getticks()
#define reset_profiling_duration() profiling__exec_duration = GET_TIME_STAMP()
#define get_profiling_duration() profiling__exec_duration = GET_TIME_STAMP() - profiling__exec_duration

#define start_profiling() {reset_cycle_counts(); reset_profiling_duration();}
#define stop_profiling(function_name) {get_cycle_counts(); get_profiling_duration(); uart_printf("%s : cycle_count:%d, exec_duration(ms):%d\n", function_name, profiling__exec_cycle_count, profiling__exec_duration);}
#else
#define start_profiling() {}
#define stop_profiling(function_name) {}
#endif


extern int uart_printf(const char *msg, ...);

/**
 * @brief Structure VL53LMZ_Platform needs to be filled by the customer,
 * depending on his platform. At least, it contains the VL53L5CX I2C address.
 * Some additional fields can be added, as descriptors, or platform
 * dependencies. Anything added into this structure is visible into the platform
 * layer.
 */

typedef struct
{
	/* To be filled with customer's platform. At least an I2C address/descriptor
	 * needs to be added */
	/* Example for most standard platform : I2C address of sensor */
    uint16_t  			address;

    uint8_t 			module_type;  // MZ-AI specific field used by sensor_command.c

} VL53LMZ_Platform;


/*
 * @brief The macro below is used to define the number of target per zone sent
 * through I2C. This value can be changed by user, in order to tune I2C
 * transaction, and also the total memory size (a lower number of target per
 * zone means a lower RAM). The value must be between 1 and 4.
 */

#define 	VL53LMZ_NB_TARGET_PER_ZONE		1U

/*
 * @brief The macro below can be used to avoid data conversion into the driver.
 * By default there is a conversion between firmware and user data. Using this macro
 * allows to use the firmware format instead of user format. The firmware format allows
 * an increased precision.
 */

#define 	VL53LMZ_USE_RAW_FORMAT

/*
 * @brief All macro below are used to configure the sensor output. User can
 * define some macros if he wants to disable selected output, in order to reduce
 * I2C access.
 */

//#define VL53LMZ_DISABLE_AMBIENT_PER_SPAD
//#define VL53LMZ_DISABLE_NB_SPADS_ENABLED
#define VL53LMZ_DISABLE_NB_TARGET_DETECTED
//#define VL53LMZ_DISABLE_SIGNAL_PER_SPAD
//#define VL53LMZ_DISABLE_RANGE_SIGMA_MM
//#define VL53LMZ_DISABLE_DISTANCE_MM
#define VL53LMZ_DISABLE_REFLECTANCE_PERCENT
//#define VL53LMZ_DISABLE_TARGET_STATUS
//#define VL53LMZ_DISABLE_MOTION_INDICATOR


#define DCI_BH__P_TYPE__GRP_PARAMS_START ((uint8_t) 13)
#define DCI_BH__P_TYPE__GRP_PARAMS_END 	((uint8_t) 14)

/**
 * @param (VL53LMZ_Platform*) p_platform : Pointer of VL53L5CX platform
 * structure.
 * @param (uint16_t) Address : I2C location of value to read.
 * @param (uint8_t) *p_values : Pointer of value to read.
 * @return (uint8_t) status : 0 if OK
 */

uint8_t RdByte(
		VL53LMZ_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value);

/**
 * @brief Mandatory function used to write one single byte.
 * @param (VL53LMZ_Platform*) p_platform : Pointer of VL53L5CX platform
 * structure.
 * @param (uint16_t) Address : I2C location of value to read.
 * @param (uint8_t) value : Pointer of value to write.
 * @return (uint8_t) status : 0 if OK
 */

uint8_t WrByte(
		VL53LMZ_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value);

/**
 * @brief Mandatory function used to read multiples bytes.
 * @param (VL53LMZ_Platform*) p_platform : Pointer of VL53L5CX platform
 * structure.
 * @param (uint16_t) Address : I2C location of values to read.
 * @param (uint8_t) *p_values : Buffer of bytes to read.
 * @param (uint32_t) size : Size of *p_values buffer.
 * @return (uint8_t) status : 0 if OK
 */

uint8_t RdMulti(
		VL53LMZ_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size);

/**
 * @brief Mandatory function used to write multiples bytes.
 * @param (VL53LMZ_Platform*) p_platform : Pointer of VL53L5CX platform
 * structure.
 * @param (uint16_t) Address : I2C location of values to write.
 * @param (uint8_t) *p_values : Buffer of bytes to write.
 * @param (uint32_t) size : Size of *p_values buffer.
 * @return (uint8_t) status : 0 if OK
 */

uint8_t WrMulti(
		VL53LMZ_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size);

/**
 * @brief This function reset the sensor, setting pins LPN, AVVD and VDDIO to 0,
 * then to 1. This implementation is optional.
 * @param (VL53LMZ_Platform) *p_platform : VL53LMZ platform structure.
 */

void Reset_Sensor(
		VL53LMZ_Platform *p_platform);

/**
 * @brief Mandatory function, used to swap a buffer. The buffer size is always a
 * multiple of 4 (4, 8, 12, 16, ...).
 * @param (uint8_t*) buffer : Buffer to swap, generally uint32_t
 * @param (uint16_t) size : Buffer size to swap
 */

void SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size);

/**
 * @brief This function wait during a selected timeout.
 * @param (VL53LMZ_Platform) *p_platform : VL53LMZ platform structure.
 * @param (uint32_t) TimeMs : Time to wait in ms.
 * @return (uint8_t) status : 0 if wait is finished.
 */

uint8_t WaitMs(
		VL53LMZ_Platform *p_platform,
		uint32_t TimeMs);

#endif	// _PLATFORM_H_
