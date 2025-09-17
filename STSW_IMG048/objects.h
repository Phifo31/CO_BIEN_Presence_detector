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
#ifndef OBJECTS_H_
#define OBJECTS_H_

#include "spd_platform.h"
#include "tracker.h"

#define MAX_NUMBER_OF_OBJECTS 4

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} OBJ_Vector_t;

typedef struct {
	OBJ_Vector_t 	CoM_pos;
	uint32_t		lifespan_msec;
	uint8_t			object_id;
	uint8_t			last_mtn_size_zones;
} OBJ_Prop_t;

typedef struct {
	uint64_t timestamp_ms;
	uint8_t NumberOfObjects;
	OBJ_Prop_t Properties[MAX_NUMBER_OF_OBJECTS];
} OBJ_Data_t;

int OBJ_init(OBJ_Data_t *OBJ_Data);
void OBJ_run(OBJ_Data_t *OBJ_Data, TRK_Data_t *TRK_Data, uint64_t timestamp_ms);

#endif /* OBJECTS_H_ */
