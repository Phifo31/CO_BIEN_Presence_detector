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


#ifndef TRACK_H_
#define TRACK_H_

#include "spd_platform.h"
#include "sensor.h"
#include "opti_math.h"
#include "ring_fbuffer.h"


#define TRK__MAX_Z_DEPTH_DIFF_WITHIN_OBJECT_MM 			160
#define TRK__MAX_NUMBER_OF_SEGMENTED_OBJECTS 			20
//#if MULTI_HUMAN_DETECTION
#define TRK__MAX_NUMBER_OF_OBJECTS_OF_INTEREST			10
//#endif
#define TRK__OBJECT_ID_WRAP_AROUND_NB 					TRK__MAX_NUMBER_OF_SEGMENTED_OBJECTS
#define TRK__MATCH_SCORE_THRESH 						0
#define TRK__PSI_MAX_NB_OF_NORMAL_VECTORS				10
#define TRK__PSI_MIN_OBJECT_SIZE_IN_ZONES				TRK__PSI_MAX_NB_OF_NORMAL_VECTORS  // Necessary to have a certain amount of zones. This criteria could be ligthened a bit, just have to make sure to have a sufficient number of zones to extract enough different normal vectors.
#define TRK__MIN_APPROACH_SPEED_MM_PER_SEC				200
#define TRK__MIN_LEAVE_Z_SPEED_MM_PER_SEC				400
#define TRK__MIN_LEAVE_Y_SPEED_MM_PER_SEC				150
#define TRK__MIN_LEAVE_X_SPEED_MM_PER_SEC				150
#define TRK__MIN_Z_DST_TRAVELLED_MM						500
#define TRK__MIN_Y_DST_TRAVELLED_MM						100
#define TRK__MIN_X_DST_TRAVELLED_MM						100
#define TRK__MIN_LIFETIME_BEFORE_LEAVE_DETECT_MSEC		3000
#define TRK__MIN_CONDITION_CNT_FOR_USR_DEPART_DETECT	5
#define TRK__MIN_X_SPEED_DEPARTURE						500

#define TRK__DEBUG_USE_RECURSIVITY				0
#define TRK__DEBUG_USE_ITERATION				!TRK__DEBUG_USE_RECURSIVITY

#define TRK__USE_3D_MATCHING					1


// HANDLE WITH CARE! not variable assignment when calling MIN or MAX.
// Example of wrong use: MIN(a++, b++);
#ifndef MIN
#   define MIN(a,b) (((a)<(b))?(a):(b))
#endif
#ifndef MAX
#   define MAX(a,b) (((a)>(b))?(a):(b))
#endif


typedef struct {
	float x;
	float y;
	float z;
} TRK_Vector_t;


typedef struct {
	uint8_t			curr_grp_nb;
	uint32_t		lifespan_msec;
	uint8_t			lifespan_cnt;
	uint8_t			obj_surface_zones;
	float			obj_surface_cm2;
	float			width_height_ratio;
	uint8_t			touching_ground;
	float			planar_surface;
	float			filtered_planar_surface;
	//uint8_t 		human_shape;
	TRK_Vector_t 	CoM_pos;
	TRK_Vector_t 	CoM_speed;
	TRK_Vector_t 	CoM_accel;
	ring_fbuffer 	xspeed_buffer;
	ring_fbuffer 	yspeed_buffer;
	ring_fbuffer 	zspeed_buffer;
	float 			speed_norm;
	float 			accel_norm;
	float			cum_distance_mm;
	float			max_accel;
	float 			movement_indicator;
	OnlineMean		omean_movement;
	uint64_t		last_substantial_mvt_t;
	uint8_t			approach_detected;
	uint8_t			is_flat;
	uint64_t		last_mtn_timestamp;
	uint8_t			last_mtn_size_zones;
	float			last_mtn_size_ratio;
	float			max_obj_surface_cm2;
	float			min_z_pos;
	//uint8_t 		max_mtn_zones_ratio;
	//uint8_t		object_size_decreased_rapidly;
	//uint8_t 		round_shaped;
	//uint8_t		square_shaped;
	float 			mean_reflectance;
	float 			std_dev_reflectance;
	float			std_dev_distance;
	//float			mean_signal;
	//uint8_t		object_moves_downward
} TRK_Tracked_Object_t;


typedef struct {
	uint8_t zNb_to_coord[SENSOR__MAX_NB_OF_ZONES][2];
	uint8_t	coord_to_zNb[SENSOR__EDGE_NB_OF_ZONES][SENSOR__EDGE_NB_OF_ZONES];

	uint8_t group[SENSOR__MAX_NB_OF_ZONES];
	uint8_t prev_group[SENSOR__MAX_NB_OF_ZONES];
	uint8_t label_count;

	float   prev_distances[SENSOR__MAX_NB_OF_ZONES];

	uint8_t group_IDs[TRK__MAX_NUMBER_OF_SEGMENTED_OBJECTS];
	uint8_t prev_group_IDs[TRK__MAX_NUMBER_OF_SEGMENTED_OBJECTS];
	uint8_t ID_count;
	uint8_t ID_usage[TRK__OBJECT_ID_WRAP_AROUND_NB];
	uint8_t obj_id[SENSOR__MAX_NB_OF_ZONES];

	uint64_t 	timestamp_ms_buff;
	TRK_Tracked_Object_t tkd_Obj[TRK__OBJECT_ID_WRAP_AROUND_NB];
//	#if MULTI_HUMAN_DETECTION
	uint8_t number_of_tkd_Obj_of_interest;
	uint8_t tkd_Obj_of_interest[TRK__MAX_NUMBER_OF_OBJECTS_OF_INTEREST];
//	#endif
	float 	leaky_distances[SENSOR__MAX_NB_OF_ZONES];
	//float	leaky_reflectance[SENSOR__MAX_NB_OF_ZONES];
	int 	leaky_count;
	float	planar_surface_indicators[TRK__PSI_MAX_NB_OF_NORMAL_VECTORS];

	uint64_t 	last_moving_obj_timestamp;
	uint8_t 	last_moving_obj_has_left;
	uint8_t 	no_object_where_scene_changes;

	int 		LockDistance;
	uint8_t		human_departure_detected;

	float match_matrix[TRK__MAX_NUMBER_OF_SEGMENTED_OBJECTS][TRK__MAX_NUMBER_OF_SEGMENTED_OBJECTS];

	float tan_alpha_diffs[SENSOR__EDGE_NB_OF_ZONES];

	int16_t min_x_speed_departure;

} TRK_Data_t;



int TRK_init(TRK_Data_t *TRK_Data, SEN_Info_t *SEN_Info, int init_trk_settings);
void TRK_label_and_match(TRK_Data_t 			*TRK_Data,
						SEN_Measurement_Data_t 	*SEN_MeasData,
						SEN_Info_t 				*SEN_Info);
//#if MULTI_HUMAN_DETECTION
void TRK_select_objects_of_interest(TRK_Data_t 				*TRK_Data,
									SEN_Measurement_Data_t 	*SEN_MeasData,
									SEN_Info_t 				*SEN_Info);
//#endif
int TRK_assign_group_ids(TRK_Data_t *TRK_Data,
						SEN_Info_t *SEN_Info);
uint8_t TRK_is_valid_zone(uint8_t zNb, SEN_Measurement_Data_t *SEN_MeasData);
void TRK_label_objects_in_scene(TRK_Data_t 				*TRK_Data,
								SEN_Measurement_Data_t 	*SEN_MeasData,
								SEN_Info_t 				*SEN_Info);
int TRK_flood_fill(TRK_Data_t 				*TRK_Data,
					SEN_Measurement_Data_t 	*SEN_MeasData,
					SEN_Info_t 				*SEN_Info,
					uint8_t 				zoneNb,
					uint8_t 				label);
int TRK_find_maximum(uint8_t *a, int n);
float TRK_find_maximum_2d(float *a, int n, int m, int coord[2]);
int TRK_compute_match_matrix(uint8_t *im0_grps, uint8_t *im1_grps, float *im0_dst, float *im1_dst, float match_matrix[][TRK__MAX_NUMBER_OF_SEGMENTED_OBJECTS], uint8_t resolution);
int TRK_compute_point_x_y_z_from_row_col_distance(float row, float col, float distance, float xyz_coord[3], int res_x, int res_y);
int TRK_compute_group_CoM_coordinates(TRK_Data_t 			*TRK_Data,
									SEN_Measurement_Data_t 	*SEN_MeasData,
									SEN_Info_t 				*SEN_Info);
int TRK_reset_tracked_object_info(TRK_Tracked_Object_t *tObj);
int TRK_update_tracked_objects_info(TRK_Data_t 				*TRK_Data,
									SEN_Measurement_Data_t 	*SEN_MeasData,
									SEN_Info_t 				*SEN_Info);
int TRK_extract_normal_vectors(TRK_Data_t 				*TRK_Data,
								SEN_Measurement_Data_t 	*SEN_MeasData,
								SEN_Info_t 				*SEN_Info,
								TRK_Vector_t 			normal_vectors[TRK__PSI_MAX_NB_OF_NORMAL_VECTORS],
								uint8_t					tkd_Obj_Id);
float TRK_compute_scalar_product(TRK_Vector_t u, TRK_Vector_t v);
TRK_Vector_t TRK_compute_cross_product(TRK_Vector_t u, TRK_Vector_t v);
float TRK_compute_vector_norm(TRK_Vector_t* u);
void TRK_normalize_vector(TRK_Vector_t* u);
int TRK_is_null_vector(TRK_Vector_t* u);


#endif /* TRACK_H_ */
