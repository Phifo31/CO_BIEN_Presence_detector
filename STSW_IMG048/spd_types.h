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

 
#ifndef SPD_TYPES_H_
#define SPD_TYPES_H_


#include <stdint.h>
#include <stddef.h> /* these is for NULL */
#include <stdbool.h>

#ifndef NULL
#error "review  NULL definition or add required include "
#endif

#if !defined(STDINT_H) &&  !defined(_GCC_STDINT_H) && !defined(__STDINT_DECLS) && !defined(_STDINT) && !defined(_STDINT_H)

#pragma message("Please review  type definition of STDINT define for your platform and add to list above ")

 /*
  *  target platform do not provide stdint or use a different #define than above
  *  to avoid seeing the message below addapt the #define list above or implement
  *  all type and delete these pragma
  */

typedef unsigned int uint32_t;
typedef int int32_t;

typedef unsigned short uint16_t;
typedef short int16_t;

typedef unsigned char uint8_t;
typedef signed char int8_t;

typedef float float_t;

#endif /* _STDINT_H */


typedef uint32_t FixPoint1616_t;

#endif /* SPD_TYPES_H_ */


