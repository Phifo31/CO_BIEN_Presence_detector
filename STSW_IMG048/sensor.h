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


#ifndef __SENSOR_H
#define __SENSOR_H

//#include <stdint.h>
#include "spd_platform.h"


#define SENSOR__MAX_NB_OF_ZONES 		64
#define SENSOR__EDGE_NB_OF_ZONES		8

#define SENSOR__NB_OF_TARGETS_PER_ZONE 	1
#define SENSOR__SCI_MAX_NB_OF_AGG 		16
#define SENSOR__SCI_MAX_FEAT_LEN 		48		// Same limit as PresenceEVK one

#define SENSOR__X_FOV 			43
#define SENSOR__Y_FOV 			43
#define SENSOR_PI 				3.14159265358979323846264338327950288
#define SENSOR__X_FOV_RAD		SENSOR__X_FOV * 2 * SENSOR_PI / 360
#define SENSOR__Y_FOV_RAD		SENSOR__Y_FOV * 2 * SENSOR_PI / 360


typedef enum {
	// 0 is ST sensor rotation (transmitter on the right of the receiver), then +1 means +90 degrees rotation clockwise
	SEN_ORIENTATION__TX_RIGHT	= 0,			/* VCSEL on the right when facing the sensor, black square at the top left corner */
	SEN_ORIENTATION__TX_BOTTOM	= 1,    		/* VCSEL at the bottom when facing the sensor, black square at the top right corner */
	SEN_ORIENTATION__TX_LEFT	= 2,    		/* VCSEL on the left when facing the sensor, black square at the bottom right corner */
	SEN_ORIENTATION__TX_TOP		= 3,			/* VCSEL at the top when facing the sensor, black square at the bottom left corner */
}SEN_Orientation_t;


typedef struct {
	// float sci__ref_bin_offset;
	float sci__detection_threshold;
	// float sci__extra_noise_sigma;
	// float sci__null_den_clip_value;
	// uint8_t sci__mem_update_mode;
	// uint8_t sci__mem_update_choice;
	// uint8_t sci__sum_span;
	// uint8_t sci__feature_length;
	// uint8_t sci__nb_of_aggregates;
	// uint8_t sci__nb_of_temporal_accumulations;
	// uint8_t sci__min_nb_for_global_detection;
	// uint8_t sci__global_indicator_format;
	int8_t sci__aggregate_id_map[SENSOR__MAX_NB_OF_ZONES];
} SEN_Sci_Config_Data_t;


/**
 * @struct Sensor Info
 * @brief Key parameters of the sensor
 */
typedef struct {
	int res_x;
	int res_y;
    int nb_zones;
    float freq;
    SEN_Orientation_t orientation;
    SEN_Sci_Config_Data_t sci__config_data;
    int sci__dmax;
} SEN_Info_t;


typedef struct {
	uint8_t sci__global_indicator;  // Global flag for scene change detection: 0 or 1
	uint8_t sci__status;
	uint8_t sci__nb_of_detected_aggregates;  // Number of aggregates where a scene change has been detected
	uint8_t sci__nb_of_aggregates;
	float sci__per_zone_indicator[SENSOR__MAX_NB_OF_ZONES];  // squared Mahalanobis distance for each aggregate. per zone and not per aggregate ! easier for replay/rotation etc...
} SEN_Sci_Output_Data_t;


typedef struct {
	float SFE[SENSOR__SCI_MAX_FEAT_LEN];
	float SFE_var[SENSOR__SCI_MAX_FEAT_LEN];
	float ambiant_estimate;
	float ambiant_estimate_var;
} SEN_Sci_Agg_Data_t;


typedef struct {
	uint64_t timestamp_ms;
//	uint8_t NumberOfTargets[SENSOR__MAX_NB_OF_ZONES];
	uint8_t RangeStatus[SENSOR__NB_OF_TARGETS_PER_ZONE][SENSOR__MAX_NB_OF_ZONES];
	float RangeMilliMeter[SENSOR__NB_OF_TARGETS_PER_ZONE][SENSOR__MAX_NB_OF_ZONES];
	float SignalRatePerSpad[SENSOR__NB_OF_TARGETS_PER_ZONE][SENSOR__MAX_NB_OF_ZONES];
	float TargetReflectanceEst[SENSOR__NB_OF_TARGETS_PER_ZONE][SENSOR__MAX_NB_OF_ZONES];
//	float SigmaEstimateRange[SENSOR__NB_OF_TARGETS_PER_ZONE][SENSOR__MAX_NB_OF_ZONES];
//	float SigmaEstimateSignal[SENSOR__NB_OF_TARGETS_PER_ZONE][SENSOR__MAX_NB_OF_ZONES];
	float AmbientRatePerSpad[SENSOR__MAX_NB_OF_ZONES];
	SEN_Sci_Output_Data_t sci__output_data;
	//SEN_Sci_Agg_Data_t sci__agg_data[SENSOR__SCI_MAX_NB_OF_AGG];
} SEN_Measurement_Data_t;


void SEN_get_row_col_coordinates_from_sensor_zoneID(int coords[2], int zoneNb, int orientation, int res_x);
int SEN_get_zoneID_from_row_col_coordinates(int row_col_coords[2], int res_x);
int SEN_get_rotation_corrected_zoneID_from_sensor_zoneID(int zoneNb, int orientation, int res_x);
int SEN_get_sensor_zoneID_from_rotation_corrected_zoneID(int corrected_zoneID, int orientation, int res_x);



#endif  // __SENSOR_H
