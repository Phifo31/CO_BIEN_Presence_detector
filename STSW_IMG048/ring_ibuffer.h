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


#ifndef RING_IBUFFER_H_
#define RING_IBUFFER_H_

#include "spd_platform.h"

    
/** @defgroup misc_ring_buffer Ring Int Buffer
 *  @brief    Simple ring buffer implementation (uint8_t)
    @par Description
    Ring buffer is implemented as a static array of integers of size #RIB_MAX_SIZE. The functional size of the ring buffer is
    programmable. When the ring_buffer is full, new elements added are replacing the older elements.
 *  @ingroup misc
 *  @{  
 */

/** Ring Buffer maximum size */
#define RIB_MAX_SIZE 8

/**
 * @struct ring_buffer
 * @brief Simple ring buffer of int with a programmable size (max size is #RIB_MAX_SIZE)
 */    
typedef struct
{
    uint8_t  buffer[RIB_MAX_SIZE];
    uint8_t* buffer_end;
    uint8_t* data_start;
    uint8_t* data_end;
    int count;
    int size;
} ring_ibuffer;

/**
 * @brief Initialize Ring Buffer
 * @param rb  Ring Buffer pointer
 * @param size Number of int elements (max size is #RIB_MAX_SIZE)
 * @return 0 on success or -1 if size is greater than #RIB_MAX_SIZE
 */
int RIB_init(ring_ibuffer* rb, int size);

/**
 * @brief Push one element in Ring Buffer (after the last element)
 * @par Function Description
 * If ring buffer is full, added element is replacing the oldest element in the ring buffer
 * @param rb  Ring Buffer pointer
 * @param data Element to add
 * @return 0 on success
 */
int RIB_push(ring_ibuffer* rb, uint8_t data);

/**
 * @brief Fills buffer with data
 * @par Function Description
 * Fills from data start to data end, only changes the values, doesn't add elements
 * @param rb  Ring Buffer pointer
 * @param data  Element to fill buffer with
 * @return 0 if successful
 */
int RIB_fill(ring_ibuffer* rb, uint8_t data);

/**
 * @brief Check if ring buffer is full
 * @param rb  Ring Buffer pointer
 * @return true if full else false
 */
bool RIB_full(ring_ibuffer* rb);
 
/**
 * @brief print/trace all elements in the ring buffer
 * @param rb  Ring Buffer pointer
 * @note The TRACE key must be defined in the project
 */
void RIB_trace(ring_ibuffer*rb);

/**
 * @brief Return the sum of elements in the ring buffer
 * @param rb  Ring Buffer pointer
 * @return The sum
 */
uint32_t RIB_sum(ring_ibuffer*rb);

/**
 * @brief Return the mean of all elements in the ring buffer
 * @param rb  Ring Buffer pointer
 * @return The mean (rounded to integer)
 */
float RIB_mean(ring_ibuffer*rb);

/**
 * @brief Return the median of all elements in the ring buffer
 * @param rb  Ring Buffer pointer
 * @return The median
 */
float RIB_median(ring_ibuffer*rb);

/**
 * @brief Return the standard deviation of all elements in the ring buffer
 * @param rb  Ring Buffer pointer
 * @return The stdev (in float)
 */
float RIB_stdev(ring_ibuffer*rb);

/**
 * @brief Return the occurence count over the ring buffer
 * @param rb  Ring Buffer pointer
 * @param val  Value from which to count occurences in the ring buffer
 * @return The occurence count
 */
int RIB_occurence_count(ring_ibuffer*rb, uint8_t val);

/**
 * @brief Return the value of the element at index. Index goes from 0 to rb->count - 1
 * @param rb  Ring Buffer pointer
 * @param index  Index of the element to be read in the ring buffer
 * @return The element value
 */
uint8_t RIB_get_element_value(ring_ibuffer*rb, int index);

/**
 * @brief Sets the value of the element at index. Index goes from 0 to rb->count - 1
 * @param rb  Ring Buffer pointer
 * @param index  Index of the element to be changed in the ring buffer
 * @param value  Value to write
 * @return 0 if successful
 */
int RIB_set_element_value(ring_ibuffer* rb, int index, uint8_t value);

/**
 * @brief Return the pointer on the element at index. Index goes from 0 to rb->count - 1
 * @param rb  Ring Buffer pointer
 * @param index  Index of the element to be read in the ring buffer
 * @return The element pointer
 */
uint8_t* RIB_get_element_pointer(ring_ibuffer*rb, int index);


#endif /* RING_IBUFFER_H_ */
