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

#ifndef _APP_COMM_H_
#define _APP_COMM_H_

#include "main.h"
#include "sensor_command.h"
#include "vl53lmz_api.h"
#include "comms_encode.h"
#include "vl53lmz_plugin_motion_indicator.h"
#include "spd.h"
//#include "stm32xxx_hal.h"

/* VL53L5 buffer print defines ------------------------------------------------------*/
#define PRINT_FORMAT_ARRAY 0
#define PRINT_FORMAT_TXT 1

union dci_union__block_header_u {
	uint32_t bytes;
	struct {

		uint32_t p_type : 4;

		uint32_t b_size__p_rep : 12;

		uint32_t b_idx__p_idx : 16;
	};
};


//void print_metadata(VL53LMZ_ResultsData *RangingData);
//void print_zone_results(VL53LMZ_ResultsData *RangingData, int zoneID);
//void print_sci_global_data(VL53LMZ_ResultsData *RangingData);
//void print_sci_agg_data(VL53LMZ_ResultsData *RangingData, int aggID);
void print_data();
void print_user();
#ifdef VL53LMZ_XTALK_CALIBRATION
static void _vl53lmz_swap_data(uint8_t *buffer, uint16_t size);
#endif




#endif	// _APP_COMM_H_
