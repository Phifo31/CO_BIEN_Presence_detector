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

#include "comms_encode.h"


//uint16_t append_to_print_buffer(uint8_t *buffer_start, uint16_t current_len, uint8_t encoding_val_whole, uint8_t encoding_val_fractional, )

uint8_t Encode_Uint32(uint8_t * buffer, uint32_t t)
{
	buffer[0]=t>>24;
	buffer[1]=(t>>16)&0x00FF;
	buffer[2]=(t>>8)&0x0000FF;
	buffer[3]=t&0x000000FF;
    return 4;
}

uint8_t Encode_Uint16(uint8_t * buffer, uint16_t t)
{
	buffer[0]=t>>8;
	buffer[1]=t&0x00FF;
    return 2;
}

uint8_t Encode_Uint8(uint8_t * buffer, uint8_t t)
{
	buffer[0]=t;
    return 1;
}


uint8_t Encode_int32(uint8_t * buffer, int32_t t)
{
	buffer[0]=t>>24;
	buffer[1]=(t>>16)&0x00FF;
	buffer[2]=(t>>8)&0x0000FF;
	buffer[3]=t&0x000000FF;
    return 4;
}

uint8_t Encode_int16(uint8_t * buffer, int16_t t)
{
	buffer[0]=t>>8;
	buffer[1]=t&0x00FF;
    return 2;
}

uint8_t Encode_int8(uint8_t * buffer, int8_t t)
{
	buffer[0]=t;
    return 1;
}

