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
#ifndef USERS_H_
#define USERS_H_

#include "spd_platform.h"
#include "objects.h"

#define MAX_NUMBER_OF_USERS MAX_NUMBER_OF_OBJECTS
#define ITA_MAX_LEFT_DISTANCE 2000 // 1300 //1700
#define ITA_MAX_RIGHT_DISTANCE 2000 // 1300 //1700
#define ITA_TIME_TO_TRIGGER_ALERT 2000
#define ITA_TIME_TO_MAINTAIN_ALERT 2000

#pragma pack(push, 1)
typedef struct {
	OBJ_Prop_t		obj_prop;
	uint8_t			intruder;
	uint64_t 		last_update_timestamp;
} USR_Prop_t;
#pragma pack(pop)

/*
 * Users Data
 */
typedef struct {
	// INPUTS
	uint16_t ita_max_left_distance;
	uint16_t ita_max_right_distance;
	uint16_t ita_time_to_trigger_alert;
	uint16_t ita_time_to_maintain_alert;
	// INTERNALS
	uint64_t timestamp;
	// OUTPUTS
	uint8_t nb_of_users;
	uint8_t intruder_alert;
	USR_Prop_t user_properties[MAX_NUMBER_OF_USERS];
} USR_Data_t;

int USR_init(USR_Data_t *USR_Data, int init_usr_settings);
void USR_run(USR_Data_t *USR_Data, OBJ_Data_t *OBJ_Data, int presence);

#endif /* USERS_H_ */
