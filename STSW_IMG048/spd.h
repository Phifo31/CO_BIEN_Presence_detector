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


#ifndef SPD_H_
#define SPD_H_

#ifdef __cplusplus
extern "C" {
#endif

#define USE_COM_BREATHING_DETECT	0

//#include <stdint.h>
#include "spd_platform.h"
#include "ring_ibuffer.h"
#include "sensor.h"
#include "tracker.h"
#include "opti_math.h"
//#if MULTI_HUMAN_DETECTION
#include "objects.h"
#include "users.h"
//#endif

// HANDLE WITH CARE! not variable assignment when calling MIN or MAX.
// Example of wrong use: MIN(a++, b++);
#ifndef MIN
#   define MIN(a,b) (((a)<(b))?(a):(b))
#endif
#ifndef MAX
#   define MAX(a,b) (((a)>(b))?(a):(b))
#endif

#if 0 // This implementation is better since no risk from double evaluation. However, __typeof__() is not available for every compiler.
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
#endif


#define SPD_VERSION "1.0.0"


///////////////////////////////////
//                               //
//   SPD ALGO (ASSESS PRESENCE)  //
//                               //
///////////////////////////////////

// SPD DEFAULT SETTINGS
#define SPD__INVALID_DISTANCE_MM				4000
#define SPD__APPROACH_DISTANCE_MM 				1500
#define SPD__WAKEUP_DISTANCE_MM 				850
#define SPD__LOCK_DISTANCE_MM 					1500
//#define SPD__APPROACH_TIMEOUT_MS				15000
#define SPD__OBSTRUCTED_FOV_DIST_THRESHOLD		40
#define SPD__STATIC_CLOSE_OBJ_MAX_DISTANCE_MM   300

#define SPD__TIME_TO_AUTONOMOUS_MS				500
#define SPD__NO_MOTION_TIMEOUT 					10000
#define SPD__MININMUM_LOCK_TIME_MS				1000			//5000

// Default following timings will be overwritten by SPD__MININMUM_LOCK_TIME_MS to simplify the usage
#define SPD__LOCK_TIME_AFTER_TRK_LOST_MS 		8000
#define SPD__LOCK_TIME_AFTER_SCENE_CHANGE_MS 	120000
#define SPD__LOCK_TIME_OBSTRUCTED_FOV_MS		2000
#define SPD__LOCK_TIME_STATIC_CLOSE_OBJ_MS		10000
#define SPD__LOCK_TIME_NO_TARGET_MS				1000

#define SPD__WAKE_ON_STOP_MIN_SPEED				1 // km/h

#define SPD_SCI_MAX_SUPPORTED_RANGE 			2400 //Maximum range supported by the FW, can't be changed
#define SPD_SCI_DEFAULT_DCIMIN 					400 //
#define SPD_SCI_DEFAULT_DCIMAX 					2800

/*
 * Timing data
 */
typedef struct {
	float ranging_frequency;	// [Hz]
	int ranging_period; 		// [msec]
	uint64_t prev_tstp;  		// [msec]
	uint64_t curr_tstp;  		// [msec]
} SPD_time_data_t;


/*
 * SPD State machine
 */
typedef enum {
	SPD_AUTONOMOUS				= 0,
	SPD_PRESENCE_MONITORING		= 1,
	SPD_APPROACH                = 2,
	SPD_PREPARING_AUTONOMOUS	= 3
} SPD_State_t;


/**
 * @struct SPD_Data_t
 * @brief Data structure for SPD algo
 */
typedef struct {

	// SETTINGS
	int InvalidDistance;
	int ApproachDistance;
	int WakeUpDistance;
	int TimeToAutonomous;
	int ObstructedFoVDistThreshold;
	int StaticCloseObjectMaxDistance;
	int LockTimeAfterTrkLost;
	int NoMotionTimeOut;
	int LockTimeObstructedFoV;
	int LockTimeStaticCloseObject;
	int LockTimeNoTarget;
	int MinimumLockTime;
	float WakeOnStopMinSpeed; //in mm/s
	uint8_t ActiveZonesMap[SENSOR__MAX_NB_OF_ZONES];

	// INPUTS
	SPD_time_data_t SPD_time_data;

	// INTERNALS
	uint8_t		init;
	uint8_t		state;
	uint8_t		previous_state;
	uint64_t	timeout_start;
	int 		userID;
	uint8_t 	scene_change_detected[SENSOR__MAX_NB_OF_ZONES];
	uint64_t	last_scene_change_timestamp_ms;
	uint64_t	tracking_lost_timestamp_ms;
	uint64_t	obstructedFoV_start_timestamp_ms;
	uint64_t    static_closeObj_start_timestamp_ms;
	uint64_t	no_target_start_timestamp_ms;
	TRK_Data_t 	TRK_Data;
//	#if MULTI_HUMAN_DETECTION
	OBJ_Data_t	OBJ_Data;
	USR_Data_t  USR_Data;
//	#endif

	// ALGO OUTPUTS
	float algo_out_obstructed_FoV;
	float algo_out_no_target;
	float algo_out_scene_change;
	float algo_out_static_close_object;
	float algo_out_user_tracking;
	float algo_out_user_approaching;

	// OUTPUTS
	// int 		estimated_duration_since_user_left;
	int 		presence;
	int 		confidence;
	int 		distance;
} SPD_Data_t;



int SPD_init(SPD_Data_t *SPD_Data, SEN_Info_t *SEN_Info, int init_spd_settings);
int SPD_run(SPD_Data_t 				*SPD_Data,
			SEN_Measurement_Data_t 	*SEN_MeasData,
			SEN_Info_t 				*SEN_Info);

int SPD_update_time_data(SPD_Data_t *SPD_Data, uint64_t timestamp_ms);
int SPD_start_tracking_user_and_surrounding_objects(SPD_Data_t *SPD_Data, int objectID);
int SPD_process_sci_output(SPD_Data_t 				*SPD_Data,
							SEN_Measurement_Data_t 	*SEN_MeasData,
							SEN_Info_t 				*SEN_Info);


#ifdef __cplusplus
}
#endif

#endif /* SPD_H_ */
