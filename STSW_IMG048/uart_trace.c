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

//#include  "stm32xxx_hal.h"
#include "main.h"

#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#ifndef TRACE_UART
/**
 * Force TRACE_UART to 0 by default (if not defined in the project)
 */
#define TRACE_UART  0
#endif

#if TRACE_UART
extern UART_HandleTypeDef huart2;

static volatile int InUsed=0;
static char uart_buffer[256];
//static char uart_buffer[2048];
static uint32_t UartErrCnt=0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    // TODO check if any more to send and do it
	(void)huart;
    InUsed=0;
}

int uart_vprintf(const char *msg, va_list ap){
    int n;
    int status;
    while( InUsed ){
           //
        __WFI();
    }
    InUsed|=1;
    n=vsnprintf(uart_buffer, sizeof(uart_buffer),  msg, ap);
    status = HAL_UART_Transmit_DMA(&huart2, (uint8_t*)uart_buffer, n );
    if( status ){
        UartErrCnt++;
        InUsed=0;
    }
    return n;
}

int uart_printf_g(const char *msg, ...){
	va_list ap;
    int n;
    while( InUsed ){
        //
        __WFI();
    }
    va_start(ap, msg);
    n=uart_vprintf(msg, ap);
    va_end(ap);
    return n;
}

int uart_printf(const char *msg, ...){
	va_list ap;
	int n;
	#if 0
	// Compute and print message length (in number of characters)
	va_start(ap, msg);
	int l = vsnprintf(NULL, 0, msg, ap);
	uart_printf_g("%04d ", l+1);
	#endif
	// uart_printf code
	while( InUsed ){
		//
		__WFI();
	}
	#if 1
	va_start(ap, msg);
	#endif
	n=uart_vprintf(msg, ap);
	va_end(ap);
	return n;
}

int uart_printbin(uint8_t *buf, uint16_t n){
    int status;
    while( InUsed ){
           //
        __WFI();
    }
    InUsed|=1;
    memcpy(uart_buffer, buf, n); // HAL_UART_Transmit_DMA is not a blocking function. Need to ensure the buffer is not re-written by main loop while HAL_UART_Transmit_DMA is not ended.
    status = HAL_UART_Transmit_DMA(&huart2, (uint8_t*)uart_buffer, n);
    if( status ){
        UartErrCnt++;
        InUsed=0;
    }
    return n;
}

#else
#	define uart_vprintf(...) (void)0
#	define uart_printf(...)	(void)0
#endif
