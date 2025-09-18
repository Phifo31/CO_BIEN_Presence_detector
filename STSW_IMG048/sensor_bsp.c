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

#include <stdio.h>

#include "main.h"
//#include "stm32xxx_hal.h"


#include "sensor_bsp.h"

int uart_vprintf(const char *msg, va_list ap);

// Output to host via UART, SSH, ...

int SB_vprintf(const char *msg, va_list ap)
{
	return uart_vprintf(msg, ap);
}

int SB_printf(const char *msg, ...)
{
	va_list ap;
    int n;

    va_start(ap, msg);
    n = uart_vprintf(msg, ap);
    va_end(ap);

    return n;
}


#if defined(STM32F4011xE) || defined(STM32L476xx)

#define TRACE_ITM_STIMULUS_PORT 0
static int ITM_TraceWrite(const char* buf, int nbyte) {
	int i;

    // Check if ITM or the stimulus port are not enabled
    if (((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0) || ((ITM->TER & (1UL << TRACE_ITM_STIMULUS_PORT)) == 0)) {
        return (int) nbyte; // return the number of sent characters (may be 0)
    }

    for ( i = 0; i < nbyte; i++) {
        // Wait until STIMx is ready...
        while (ITM->PORT[TRACE_ITM_STIMULUS_PORT].u32 == 0)
        ;
        // then send data, one byte at a time
        ITM->PORT[TRACE_ITM_STIMULUS_PORT].u8 = (uint8_t) (*buf++);
    }

    return (int) nbyte; // all characters successfully sent
}

static char ITM_TraceBuffer[256];
int SB_VDebug(const char *fmt, va_list ap) {
    int n, len;
    len = vsnprintf(ITM_TraceBuffer, sizeof(ITM_TraceBuffer), fmt, ap);
    n = ITM_TraceWrite(ITM_TraceBuffer, len);
    return n;
}

int SB_Debug(const char *fmt, ...) {
    int n;
    va_list ap;
    va_start(ap, fmt);
    n = SB_VDebug(fmt, ap);
    return n;
}

#else
#   warning "ITM trace not supported for L053R8"
#   define SB_Debug(...)  (void)0
//#   define SB_VDebug(...) (void)0
#endif


int SB_VDebug(const char *msg, va_list ap) {
    UNUSED (msg);
    UNUSED (ap);
    return 0;
}


// Returns a temporary storage big enough to dump the parameters into (about 700 bytes so far)
static char SB_PrivateBuffer[1024];
char* SB_TmpBuffer(void)
{
	return SB_PrivateBuffer;
}


