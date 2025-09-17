
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSOR_BSP_H
#define __SENSOR_BSP_H

#ifdef __cplusplus
extern "C" {
#endif 

#include <stdio.h>
#include <stdarg.h>

// Output to host via UART, SSH, ...
int SB_vprintf(const char *msg, va_list ap);
int SB_printf(const char *msg, ...);

// Debug traces thru eg ITM port, if available
int SB_Debug(const char *msg, ...);
int SB_VDebug(const char *msg, va_list ap);

// Returns a temporary storage big enough to dump the parameters into
char* SB_TmpBuffer(void);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_BSP_H*/

