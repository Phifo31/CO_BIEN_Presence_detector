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

#include "app_comm.h"

int uart_printbin(uint8_t *uart_buffer, uint16_t n);

extern CommandData_t CommandData;
extern struct Params_t Params;
extern VL53LMZ_Configuration 	LMZDev;
extern VL53LMZ_ResultsData 		RangingData;
extern VL53LMZ_Motion_Configuration 	sci_config;
extern SPD_Data_t 				SPD_Data;
extern SEN_Measurement_Data_t 	SEN_MeasData;
extern SEN_Info_t 				SEN_Info;
extern APP_time_data 			time_data;


uint8_t encode_CRC(uint8_t *buffer, uint8_t *msg, int l){
	int i, sum=0;
	uint8_t crc;

	for (i=0; i<l; i++){
		sum += msg[i];
	}
	crc = 0x100 - (sum % 0x100);
	buffer[0] = crc;

	return 1;
}

void print_encoded_frame(int log_level){
	//int target_indices[3];
	uint8_t msg[PRINT_MAX_BUFFER_SIZE];
	uint8_t *pbuffer;
	uint8_t prefix_len=4;
	uint8_t suffix_len=5;
	char prefix[] = "edv ";
	char suffix[] = " END\n";
	int i, ii;


	// Print per-frame data
	strcpy((char *)msg, prefix);
	pbuffer = msg + prefix_len;
	if (log_level >= 1){
		pbuffer += Encode_Uint8(pbuffer, 255); // Indicates per-frame data
		pbuffer += Encode_Uint8(pbuffer, LMZDev.streamcount);
	}
	if (log_level >= 2){
		pbuffer += Encode_Uint32(pbuffer, time_data.curr_tstp);
		pbuffer += Encode_Uint32(pbuffer, time_data.ranging_period);

		pbuffer += Encode_Uint32(pbuffer, RangingData.motion_indicator.global_indicator_1);
		pbuffer += Encode_Uint8(pbuffer, RangingData.motion_indicator.status);
		pbuffer += Encode_Uint8(pbuffer, RangingData.motion_indicator.nb_of_detected_aggregates);
		pbuffer += Encode_Uint8(pbuffer, RangingData.motion_indicator.nb_of_aggregates);
		pbuffer += Encode_Uint32(pbuffer, sci_config.detection_threshold); // Same thresholds for all aggregates
	}
	if (log_level >= 3){
		pbuffer += Encode_Uint8(pbuffer, (uint8_t)SPD_Data.presence);
		pbuffer += Encode_int16(pbuffer, (int16_t)(SPD_Data.TRK_Data.tkd_Obj[SPD_Data.userID].CoM_pos.x * (1 << 2)));	// signed 14.2
		pbuffer += Encode_int16(pbuffer, (int16_t)(SPD_Data.TRK_Data.tkd_Obj[SPD_Data.userID].CoM_pos.y * (1 << 2)));  	// signed 14.2
		pbuffer += Encode_int16(pbuffer, (int16_t)(SPD_Data.TRK_Data.tkd_Obj[SPD_Data.userID].CoM_pos.z * (1 << 2)));  	// signed 14.2
		pbuffer += Encode_int16(pbuffer, (uint16_t)(SPD_Data.TRK_Data.tkd_Obj[SPD_Data.userID].speed_norm / 278 * (1 << 4))); //convert mm/s to km/h -> *(3600/1000000) = /278  // signed 12.4
		pbuffer += Encode_Uint8(pbuffer, (uint8_t)(SPD_Data.TRK_Data.human_departure_detected));
		pbuffer += Encode_Uint8(pbuffer, (uint8_t)(SPD_Data.algo_out_obstructed_FoV * (1 << 7)));  		// unsigned 1.7
		pbuffer += Encode_Uint8(pbuffer, (uint8_t)(SPD_Data.algo_out_no_target * (1 << 7)));  			// unsigned 1.7
		pbuffer += Encode_Uint8(pbuffer, (uint8_t)(SPD_Data.algo_out_scene_change * (1 << 7)));  		// unsigned 1.7
		pbuffer += Encode_Uint8(pbuffer, (uint8_t)(SPD_Data.algo_out_static_close_object * (1 << 7)));  // unsigned 1.7
		pbuffer += Encode_Uint8(pbuffer, (uint8_t)(SPD_Data.algo_out_user_tracking * (1 << 7)));  		// unsigned 1.7
		pbuffer += Encode_Uint8(pbuffer, (uint8_t)(SPD_Data.algo_out_user_approaching * (1 << 7)));  	// unsigned 1.7
		pbuffer += Encode_Uint8(pbuffer, (uint8_t)(SPD_Data.state));
		// Users outputs (including intruders)
		pbuffer += Encode_Uint8(pbuffer, (uint8_t)(SPD_Data.USR_Data.nb_of_users));
		pbuffer += Encode_Uint8(pbuffer, (uint8_t)(SPD_Data.USR_Data.intruder_alert));
		for (i=0;i<MAX_NUMBER_OF_USERS;i++){
			pbuffer += Encode_int16(pbuffer, (int16_t)(SPD_Data.USR_Data.user_properties[i].obj_prop.CoM_pos.x));
			pbuffer += Encode_int16(pbuffer, (int16_t)(SPD_Data.USR_Data.user_properties[i].obj_prop.CoM_pos.y));
			pbuffer += Encode_int16(pbuffer, (int16_t)(SPD_Data.USR_Data.user_properties[i].obj_prop.CoM_pos.z));
			pbuffer += Encode_Uint8(pbuffer, (uint8_t)(SPD_Data.USR_Data.user_properties[i].intruder));
		}
	}
	pbuffer += encode_CRC(pbuffer, msg + prefix_len, pbuffer - (msg + prefix_len));
	strcpy((char *)pbuffer, suffix);
	uart_printbin(msg, (pbuffer - msg) + suffix_len);


	// Print per-zone data
	//for (i=0; i < RangingData.common_data.nb_zones; i++){
	for (i=0; i < Params.Resolution; i++){

		// Sensor data are not rotated, we need the correction here to display it correctly in Presence EVK
		ii = SEN_get_sensor_zoneID_from_rotation_corrected_zoneID(i, SEN_Info.orientation, SEN_Info.res_x);

		strcpy((char *)msg, prefix);
		pbuffer = msg + prefix_len;
		pbuffer += Encode_Uint8(pbuffer, (uint8_t) i); // Indicates zone number.
		if (log_level >= 1){
			pbuffer += Encode_Uint32(pbuffer, RangingData.ambient_per_spad[ii]);
			pbuffer += Encode_Uint8(pbuffer, RangingData.target_status[VL53LMZ_NB_TARGET_PER_ZONE*ii]);
			pbuffer += Encode_int16(pbuffer, RangingData.distance_mm[VL53LMZ_NB_TARGET_PER_ZONE*ii]);
			pbuffer += Encode_Uint32(pbuffer, RangingData.signal_per_spad[VL53LMZ_NB_TARGET_PER_ZONE*ii]);
		}
		if (log_level >= 2){
			pbuffer += Encode_Uint32(pbuffer, RangingData.motion_indicator.motion[sci_config.map_id[ii]]);

		}
		if (log_level >= 3){
			// SPD data are already rotated, no need to correct
			pbuffer += Encode_Uint8(pbuffer, (uint8_t)(SPD_Data.scene_change_detected[i]));
			pbuffer += Encode_Uint8(pbuffer, SPD_Data.TRK_Data.group_IDs[SPD_Data.TRK_Data.group[i]]);
		}

		pbuffer += encode_CRC(pbuffer, msg + prefix_len, pbuffer - (msg + prefix_len));
		strcpy((char *)pbuffer, suffix);
		uart_printbin(msg, (pbuffer - msg) + suffix_len);
	}
}



void print_user(){
	int i;
	uart_printf("\x1b[2J");
	uart_printf("\x1b[1;1H");
	uart_printf("Presence:%d\n",SPD_Data.presence);
	uart_printf("Main User position: X=%d mm, Y=%d mm, Z=%d mm\n",(int16_t)SPD_Data.TRK_Data.tkd_Obj[SPD_Data.userID].CoM_pos.x, (int16_t)SPD_Data.TRK_Data.tkd_Obj[SPD_Data.userID].CoM_pos.y, (int16_t)SPD_Data.TRK_Data.tkd_Obj[SPD_Data.userID].CoM_pos.z);

	if (SPD_Data.state == SPD_AUTONOMOUS)
		uart_printf("SPD state = SPD_AUTONOMOUS\n\n");
	else if (SPD_Data.state == SPD_PRESENCE_MONITORING)
		uart_printf("SPD state = SPD_PRESENCE_MONITORING\n\n");
	else if (SPD_Data.state == SPD_APPROACH)
		uart_printf("SPD state = SPD_APPROACH\n\n");
	else if (SPD_Data.state == SPD_PREPARING_AUTONOMOUS)
		uart_printf("SPD state = SPD_PREPARING_AUTONOMOUS\n\n");
	else
		uart_printf("Unknown SPD state\n\n");

	uart_printf("_____________________________________\n");
	if (SPD_Data.USR_Data.nb_of_users == 1){//need to check Intruder Alert ???.
		uart_printf("|\n|\n|\n|\n|\n|\n|\n|\n");
	}
	else if(SPD_Data.USR_Data.nb_of_users == 2){
		if (SPD_Data.USR_Data.user_properties[1].obj_prop.CoM_pos.x < -40){
			uart_printf("|\n\
|   /   \\ \n\
|  | @ @ | \n\
|  |  .  | \n\
|  |  =  | \n\
|  _\\   /_ \n\
| |       | \n|\n");
		}
		else if (SPD_Data.USR_Data.user_properties[1].obj_prop.CoM_pos.x <= 40) {
			uart_printf("|\n\
|              /   \\ \n\
|             | @ @ | \n\
|             |  .  | \n\
|             |  =  | \n\
|             _\\   /_ \n\
|            |       | \n|\n");
		}
		else{
			uart_printf("|\n\
|                         /   \\ \n\
|                        | @ @ | \n\
|                        |  .  | \n\
|                        |  =  | \n\
|                        _\\   /_ \n\
|                       |       | \n|\n");
		}
	}
	else if (SPD_Data.USR_Data.nb_of_users == 3){
		uart_printf("|\n\
|   /   \\                 /   \\ \n\
|  | @ @ |               | @ @ | \n\
|  |  .  |               |  .  | \n\
|  |  =  |               |  =  | \n\
|  _\\   /_               _\\   /_ \n\
| |       |             |       | \n|\n");
	}
	else if (SPD_Data.USR_Data.nb_of_users == 4){
		uart_printf("|\n\
|   /   \\      /   \\      /   \\ \n\
|  | @ @ |    | @ @ |    | @ @ | \n\
|  |  .  |    |  .  |    |  .  | \n\
|  |  =  |    |  =  |    |  =  | \n\
|  _\\   /_    _\\   /_    _\\   /_ \n\
| |       |  |       |  |       | \n|\n");
	}
	else{
		uart_printf("|\n|\n|\n|\n|\n|\n|\n|\n");
	}
	if (SPD_Data.presence){
		if (SPD_Data.USR_Data.nb_of_users == 0){
			uart_printf("|\n|\n|\n|\n|\n|\n|\n|\n|\n|\n|\n|\n|\n|\n|\n");
		}
		else{
			uart_printf("|\n\
|            ||||||||| \n\
|            ||||||||| \n\
|           |   _ _   | \n\
|           |  / | \\  | \n\
|           | | o|o | | \n\
|           |  \\_|_/  | \n\
|           |   ___   | \n");
		uart_printf("\
|           |  (___)  | \n\
|           | \\_____/ | \n\
|           |  \\___/  | \n\
|            \\       / \n\
|             \\__ __/ \n\
|          ___/     \\___ \n|\n");
		}
	}
	else{
		if (SPD_Data.state == SPD_AUTONOMOUS){
			uart_printf("|\n\
|  __________ \n\
| |_________ | \n\
|          / / \n\
|         / /      _______ \n");
		uart_printf("\
|        / /      |______ | \n\
|       / /             / / \n\
|      / /             / /     _____ \n");
		uart_printf("\
|     / /             / /     |____ |  \n\
|    / /             / /          / / \n\
|   / /             / /          / / \n");
		uart_printf("\
|  | /________     | /_____     / /__ \n\
|  |__________|    |_______|   |_____|\n|\n|\n");
		}
		else{
			uart_printf("|\n|\n|\n|\n|\n|\n|\n|\n|\n|\n|\n|\n|\n|\n|\n");
		}
	}
	uart_printf("_____________________________________\n");
	uart_printf("Nb users:%d, Intruder Alert:%d\n\n",SPD_Data.USR_Data.nb_of_users, SPD_Data.USR_Data.intruder_alert);
	for (i=1; i<SPD_Data.USR_Data.nb_of_users; i++){
		uart_printf("Other User #%d, X:%d, Y:%d, Z:%d\n\n",i,SPD_Data.USR_Data.user_properties[i].obj_prop.CoM_pos.x,SPD_Data.USR_Data.user_properties[i].obj_prop.CoM_pos.y,SPD_Data.USR_Data.user_properties[i].obj_prop.CoM_pos.z);

	}

}


void print_data(){
	// Encoded logging for PresenceEVK
	if (CommandData.EncodedDevLoggingLevel > 0){
		print_encoded_frame(CommandData.EncodedDevLoggingLevel);
	}
	else{
		// Presence logging
		if (CommandData.SPDLoggingLevel){
			print_user();
		}
	}
}

#ifdef VL53LMZ_XTALK_CALIBRATION
int print_buffer(uint8_t *buffer, uint16_t size, uint8_t format){
	int i = 0;
	uint8_t *s = buffer;

	if (size %4 != 0){
		uart_printf("print_buffer failed : data size not a multiple of 4 bytes !\n");
		return -1;
	}

	uart_printf("size : %d\n",size/4);
	for(i=0;i<size;i=i+4){
		if (format == PRINT_FORMAT_ARRAY)
			uart_printf("0x%02x, 0x%02x, 0x%02x, 0x%02x\n",*s,*(s+1),*(s+2),*(s+3));
		else if (format == PRINT_FORMAT_TXT)
			uart_printf("%02X%02X%02X%02X\n",*s,*(s+1),*(s+2),*(s+3));
		s=s+4;
	}

	return 0;
}

//TODO : Transfer in ULD xtalk plugin
#define DEV_PXTALK_GRID_META_IDX ((uint16_t)0x9fd8)
#define DEV_PXTALK_GRID_RATE_IDX ((uint16_t)0x9ffc)
#define DEV_PXTALK_GRID_RATE_CAL__GRID_DATA__RATE_KCPS_PER_SPAD_IDX ((uint16_t) DEV_PXTALK_GRID_RATE_IDX)
struct cal_grp__grid_meta_t {

	int16_t cal__grid_meta__distance_mm;

	uint16_t cal__grid_meta__reflectance_pc;

	int8_t cal__grid_meta__silicon_temp_degc;

	uint8_t cal__grid_meta__cols;

	uint8_t cal__grid_meta__rows;

	uint8_t cal__grid_meta__x_offset_spads;

	uint8_t cal__grid_meta__y_offset_spads;

	uint8_t cal__grid_meta__x_pitch_spads;

	uint8_t cal__grid_meta__y_pitch_spads;
	uint8_t cal__grid_meta__avg_count;
};
struct cal_grp__grid_data__rate_kcps_per_spad_t {

	uint32_t
	cal__grid_data__rate_kcps_per_spad[64];
};

int print_decoded_cal_data(uint8_t *buffer, uint16_t size){
	int32_t status = VL53LMZ_STATUS_OK;
	int i = 0;
	int j = 0;
	union dci_union__block_header_u *packed_data__bh_ptr;
	//uint32_t xtalk_signal_grid[64];
	//uint8_t xtalk_copy[VL53LMZ_XTALK_SIZE];
	//struct cal_grp__ref_spad_info_t   *ref_spad_info_ptr = NULL;
	//struct cal_grp__grid_meta_t *offset_grid_meta_ptr = NULL;
	//struct cal_grp__grid_data__rate_kcps_per_spad_t *offset_grid_rate_ptr = NULL;
	//struct cal_grp__grid_data__range_mm_t *offset_grid_offset_ptr = NULL;
	struct cal_grp__grid_meta_t *xtalk_grid_meta_ptr = NULL;
	struct cal_grp__grid_data__rate_kcps_per_spad_t *xtalk_grid_rate_ptr = NULL;
	float max_rate = 0;

	#if 0
	/* Create a copy of config */
	memcpy(xtalk_copy, buffer, size);

	/* Swap xtalk buffer (as BareDriver... ) */
	_vl53lmz_swap_data(xtalk_copy, size);
	#endif

	for (i=0;i<size;i+=4){
		packed_data__bh_ptr = (union dci_union__block_header_u *)&(buffer[i]);

		#if 1
		// Xtalk cal data : grid metadata
		if (packed_data__bh_ptr->b_idx__p_idx == DEV_PXTALK_GRID_META_IDX){
			xtalk_grid_meta_ptr =  ((struct cal_grp__grid_meta_t *)&(buffer[i + 4]));
			uart_printf_g("\"pxtalk_grid_meta\": {\n");
			uart_printf_g("\t\"cal__grid_meta__cols\":%.1f,\n",(float)(xtalk_grid_meta_ptr->cal__grid_meta__cols));
			uart_printf_g("\t\"cal__grid_meta__rows\":%.1f,\n",(float)(xtalk_grid_meta_ptr->cal__grid_meta__rows));
			uart_printf_g("\t\"cal__grid_meta__x_offset_spads\":%.1f,\n",(float)(xtalk_grid_meta_ptr->cal__grid_meta__x_offset_spads/2.0));
			uart_printf_g("\t\"cal__grid_meta__y_offset_spads\":%.1f,\n",(float)(xtalk_grid_meta_ptr->cal__grid_meta__y_offset_spads/2.0));
			uart_printf_g("\t\"cal__grid_meta__x_pitch_spads\":%.1f,\n",(float)(xtalk_grid_meta_ptr->cal__grid_meta__x_pitch_spads/2.0));
			uart_printf_g("\t\"cal__grid_meta__y_pitch_spads\":%.1f,\n",(float)(xtalk_grid_meta_ptr->cal__grid_meta__y_pitch_spads/2.0));
			uart_printf_g("}\n");
		}
		#endif
		//Xtalk cal data : signal grid
		if (packed_data__bh_ptr->b_idx__p_idx == DEV_PXTALK_GRID_RATE_CAL__GRID_DATA__RATE_KCPS_PER_SPAD_IDX){
			xtalk_grid_rate_ptr =  ((struct cal_grp__grid_data__rate_kcps_per_spad_t *)&(buffer[i + 4]));
			uart_printf_g("\"pxtalk_grid_rate\": {\n");
			uart_printf_g("\t\"cal__grid_data__rate_kcps_per_spad\": [");
			uart_printf_g("%.2f",(float)(xtalk_grid_rate_ptr->cal__grid_data__rate_kcps_per_spad[0]/2048.0));
			max_rate = xtalk_grid_rate_ptr->cal__grid_data__rate_kcps_per_spad[0]/2048.0;
			for (j=1;j<64;j++){
				uart_printf_g(",%.2f",(float)(xtalk_grid_rate_ptr->cal__grid_data__rate_kcps_per_spad[j]/2048.0));
				if ((xtalk_grid_rate_ptr->cal__grid_data__rate_kcps_per_spad[j]/2048.0) > max_rate) {
					max_rate = xtalk_grid_rate_ptr->cal__grid_data__rate_kcps_per_spad[j]/2048.0;
				}
			}
			uart_printf_g("]\n}\n");
			uart_printf_g("max xtalk rate : %.2f kcps/spad\n",max_rate);
		}
	}
	return status;
}

static void _vl53lmz_swap_data(uint8_t *buffer, uint16_t size){
	uint32_t i, tmp;
	for(i = 0; i < size ;i = i + 4) {
		tmp = (buffer[i]<<24)|(buffer[i+1]<<16)|(buffer[i+2]<<8)|(buffer[i+3]);
		memcpy(&(buffer[i]), &tmp, 4);
	}
}
#endif
