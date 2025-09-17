#ifndef _COMMS_ENCODE_H_
#define _COMMS_ENCODE_H_

#include <stdint.h>

#define PRINT_MAX_BUFFER_SIZE 256

uint8_t Encode_Uint32(uint8_t * buffer, uint32_t t);

uint8_t Encode_Uint16(uint8_t * buffer, uint16_t t);

uint8_t Encode_Uint8(uint8_t * buffer, uint8_t t);

uint8_t Encode_int32(uint8_t * buffer, int32_t t);

uint8_t Encode_int16(uint8_t * buffer, int16_t t);

uint8_t Encode_int8(uint8_t * buffer, int8_t t);



#endif /* _COMMS_ENCODE_H_ */
