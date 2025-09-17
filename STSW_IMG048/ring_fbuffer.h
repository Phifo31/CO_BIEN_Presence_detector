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


#ifndef RING_FBUFFER_H_
#define RING_FBUFFER_H_

#include "spd_platform.h"

/** @defgroup misc_ring_buffer Ring Float Buffer
 *  @brief    Simple ring buffer implementation (float)
    @par Description
    Ring buffer is implemented as a static array of integers of size #RFB_MAX_SIZE. The functional size of the ring buffer is
    programmable. When the ring_buffer is full, new elements added are replacing the older elements.
 *  @ingroup misc
 *  @{  
 */

/** Ring Buffer maximum size */
#define RFB_MAX_SIZE 8

/**
 * @struct ring_buffer
 * @brief Simple ring buffer of int with a programmable size (max size is #RFB_MAX_SIZE)
 */    
typedef struct
{
    float  buffer[RFB_MAX_SIZE];
    float* buffer_end;
    float* data_start;
    float* data_end;
    int count;
    int size;
} ring_fbuffer;

/**
 * @brief Initialize Ring Buffer
 * @param rb  Ring Buffer pointer
 * @param size Number of int elements (max size is #RFB_MAX_SIZE)
 * @return 0 on success or -1 if size is greater than #RFB_MAX_SIZE
 */
int RFB_init(ring_fbuffer* rb, int size);

/**
 * @brief Push one element in Ring Buffer (after the last element)
 * @par Function Description
 * If ring buffer is full, added element is replacing the oldest element in the ring buffer
 * @param rb  Ring Buffer pointer
 * @param data Element to add
 * @return 0 on success
 */
int RFB_push(ring_fbuffer* rb, float data);

/**
 * @brief Fills buffer with data
 * @par Function Description
 * Fills from data start to data end, only changes the values, doesn't add elements
 * @param rb  Ring Buffer pointer
 * @param data  Element to fill buffer with
 * @return 0 if successful
 */
int RFB_fill(ring_fbuffer* rb, float data);

/**
 * @brief Check if ring buffer is full
 * @param rb  Ring Buffer pointer
 * @return true if full else false
 */
bool RFB_full(ring_fbuffer* rb);
 
/**
 * @brief print/trace all elements in the ring buffer
 * @param rb  Ring Buffer pointer
 * @note The TRACE key must be defined in the project
 */
void RFB_trace(ring_fbuffer*rb);

/**
 * @brief return all elements in the ring buffer in the provided array
 * @param rb  Ring Buffer pointer
 * @note dest_array[] must be of size rb->count !
 */
int RFB_get_array_from_buffer(ring_fbuffer*rb, float *dest_array);

/**
 * @brief Return the sum of elements in the ring buffer
 * @param rb  Ring Buffer pointer
 * @return The sum
 */
float RFB_sum(ring_fbuffer*rb);

/**
 * @brief Return the sum of the absolute elements in the ring buffer
 * @param rb  Ring Buffer pointer
 * @return The sum of absolute elements
 */
float RFB_sum_abs(ring_fbuffer*rb);

/**
 * @brief Return the min value of the elements in the ring buffer
 * @param rb  Ring Buffer pointer
 * @return The minimum value
 */
float RFB_min(ring_fbuffer*rb);

/**
 * @brief Return the max value of the elements in the ring buffer
 * @param rb  Ring Buffer pointer
 * @return The maximum value
 */
float RFB_max(ring_fbuffer*rb);

/**
 * @brief Return the mean of all elements in the ring buffer
 * @param rb  Ring Buffer pointer
 * @return The mean
 */
float RFB_mean(ring_fbuffer*rb);

/**
 * @brief Return the median of all elements in the ring buffer
 * @param rb  Ring Buffer pointer
 * @return The median
 */
float RFB_median(ring_fbuffer*rb);

/**
 * @brief Return the variance of all elements in the ring buffer
 * @param rb  Ring Buffer pointer
 * @return The variance (as float)
 */
float RFB_var(ring_fbuffer*rb);

/**
 * @brief Return statistics over elements in the ring buffer
 * @param rb  Ring Buffer pointer
 */
void RFB_stats(ring_fbuffer*rb, float* min, float* mean, float* stddev, float* max);

/**
 * @brief Return the standard deviation of all elements in the ring buffer
 * @param rb  Ring Buffer pointer
 * @return The stdev (as float)
 */
float RFB_stdev(ring_fbuffer*rb);

/**
 * @brief Return the value of the element at index. Index goes from 0 to rb->count - 1
 * @param rb  Ring Buffer pointer
 * @param index  Index of the element to be read in the ring buffer
 * @return The element value
 */
float RFB_get_element_value(ring_fbuffer*rb, int index);

/**
 * @brief Sets the value of the element at index. Index goes from 0 to rb->count - 1
 * @param rb  Ring Buffer pointer
 * @param index  Index of the element to be changed in the ring buffer
 * @param value  Value to write
 * @return 0 if successful
 */
int RFB_set_element_value(ring_fbuffer* rb, int index, float value);

/**
 * @brief Return the pointer on the element at index. Index goes from 0 to rb->count - 1
 * @param rb  Ring Buffer pointer
 * @param index  Index of the element to be read in the ring buffer
 * @return The element pointer
 */
float* RFB_get_element_pointer(ring_fbuffer*rb, int index);


#endif /* RING_FBUFFER_H_ */
