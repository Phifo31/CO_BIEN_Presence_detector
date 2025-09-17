/*
 * application.c
 *
 * Recupération du SPD_KIT ULD API et de la démo
 *
 * Regroupement des fichiers nécessaires dans un seul répertoire local : SPD_STM32
 * Pour simplifier la compilation et la gestion de conf
 *
 *
 */

#include "main.h"

#define GET_TIME_STAMP()  (int32_t)HAL_GetTick()
#define UART_BUFFER_SIZE    2048

#include "sensor_command.h"
#include "app_comm.h"
#include "platform.h"

#include "vl53lmz_api.h"
#include "vl53lmz_plugin_infos.h"
#include "vl53lmz_plugin_detection_thresholds.h"
#include "vl53lmz_plugin_motion_indicator.h"
#include "vl53lmz_plugin_xtalk.h"

#include "spd_platform.h"
#include "spd.h"
#include "sensor.h"
#include "comms_encode.h" // Encoded channel for SPD EVK

#include "application.h"

int status = 0;
int off = 0;

volatile char Uart_RXBuffer[UART_BUFFER_SIZE];
volatile size_t Uart_RxRcvIndex = 0;
char UartComm_RXBuffer[UART_BUFFER_SIZE];
int UartComm_RXSize = 0;
volatile int UartActive;
volatile uint32_t Uart_nRxCb = 0;
volatile uint32_t Uart_nOverrun = 0;
volatile uint32_t Uart_nTxCb = 0;
volatile uint32_t Uart_nErrCb = 0;
volatile uint32_t nUartTxCb_User;
volatile uint32_t nUartRxCb_User;
volatile int UartComm_CmdReady = 0;
volatile int IntCount;

/* VL53LMZ variables -----------------------------------------------------------------*/
VL53LMZ_Configuration LMZDev;
VL53LMZ_ResultsData RangingData;
int ranging = 0;
int8_t xtalk64_calibration_stored = 0;
int8_t xtalk_calibrated = 0;    // 0:default, 16 or 64:calibrated
#ifdef VL53LMZ_XTALK_CALIBRATION
uint8_t                 xtalk_calibration_buffer[VL53LMZ_XTALK_BUFFER_SIZE];
#endif
VL53LMZ_Motion_Configuration sci_config;

/* Application (main.c) variables ---------------------------------------------------*/
extern struct Params_t Params;
APP_time_data time_data;
extern CommandData_t CommandData;
int profiling__exec_cycle_count;
int profiling__exec_duration;

/* Application function prototypes */
int print_buffer(uint8_t *buffer, uint16_t size, uint8_t format);
int print_decoded_cal_data(uint8_t *buffer, uint16_t size);

/* VL53LMZ related function prototypes -----------------------------------------------*/
int get_ranging_data(void);
int init_vl53lmz_sensor();
int perform_calibration(uint32_t calibration_type);
int sci_set_config(struct Params_t *p_Params);
int vl53lmz_sci_calculate_required_memory(VL53LMZ_Motion_Configuration *p_sci_config, uint32_t *p_required_memory);
int vl53lmz_Configure(void);
uint8_t vl53lmz_spd_set_output_pipe(VL53LMZ_Configuration *p_dev, uint8_t nb_target_per_zone,
        uint8_t disable_pipe_control, uint8_t enable_sharpener_post_filter, uint8_t auto_stop);

/* SPD algo variables ---------------------------------------------------------------*/
SPD_Data_t SPD_Data;
SEN_Measurement_Data_t SEN_MeasData;
SEN_Info_t SEN_Info;
int SPDLoggingLevel = 1; //used by the SPD library
uint32_t sci_motion_map[SENSOR__SCI_MAX_NB_OF_AGG]; // This buffer is needed to store the SCI motion map on the wake-up interrupt
uint8_t sci_motion_map_stored = 0; // This flag is used to know when the SCI map is stored

/* SPD related function prototypes --------------------------------------------------*/
int SEN_update_info();
int SEN_CopyMeasurementData(SEN_Measurement_Data_t *dest, VL53LMZ_ResultsData *src_ranging, uint64_t timestamp_ms);
int SEN_SetParamsForTracking();
int SEN_SetParamsForAutonomous();
int SPD_Init(int reset);
int SPD_ChangeSensorMode(SPD_Data_t *pSPDData);

volatile int IntCount;

/* VL53LMZ Interrupt handler -----------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == INT_C_Pin) {
        IntCount++;
    }
}

#if 1
/**
 * Enter critical section in interrupt handler aka disable all interrupts
 * @warning does not support multiple enter/exit
 */
#define _CriticalEnter() \
    uint32_t prim = __get_PRIMASK();\
    __disable_irq();

/**
 * Exit critical section in interrupt handler
 * @warning does not support multiple enter/exit in same scope (local var defined)
 */
#define _CriticalExit() \
    if( !prim  ){\
        __enable_irq();\
    }
#else
#   define _CriticalExit()
#   define _CriticalEnter()
#endif

/**
 * Init SPD with input parameters and optionally reset state machine (needed the first time)
 */
int SPD_Init(int reset) {
    int status_spd = 0;

    SEN_update_info();

    // SPD PARAMS
    SPD_Data.InvalidDistance = Params.spdInvalidDistance_mm;
    SPD_Data.ApproachDistance = Params.spdApproachDistance_mm;
    SPD_Data.WakeUpDistance = Params.spdWakeUpDistance_mm;
    SPD_Data.TimeToAutonomous = Params.spdTimeToAutonomous_ms;
    SPD_Data.ObstructedFoVDistThreshold = Params.spdObstructedFoVDistThreshold;
    SPD_Data.StaticCloseObjectMaxDistance = Params.spdStaticCloseObjectMaxDistance;
    SPD_Data.LockTimeAfterTrkLost = Params.spdLockTimeAfterTrkLost_ms;
    SPD_Data.NoMotionTimeOut = Params.spdNoMotionTimeOut_ms;
    SPD_Data.LockTimeObstructedFoV = Params.spdLockTimeObstructedFoV_ms;
    SPD_Data.LockTimeStaticCloseObject = Params.spdLockTimeStaticCloseObject_ms;
    SPD_Data.LockTimeNoTarget = Params.spdLockTimeNoTarget_ms;
    SPD_Data.MinimumLockTime = Params.spdMinimumLockTime_ms;
    SPD_Data.WakeOnStopMinSpeed = Params.spdWakeOnStopMinSpeed_kmh;
    SPD_Data.TRK_Data.min_x_speed_departure = Params.spdMinXSpeedDeparture;
    // ITA params
    SPD_Data.USR_Data.ita_max_left_distance = Params.itaMaxLeftDistance_mm;
    SPD_Data.USR_Data.ita_max_right_distance = Params.itaMaxRightDistance_mm;
    SPD_Data.USR_Data.ita_time_to_trigger_alert = Params.itaTimeToTriggerAlert_ms;
    SPD_Data.USR_Data.ita_time_to_maintain_alert = Params.itaTimeToMaintainAlert_ms;

    if (reset) {
        // Set init_spd_settings=0 to not overwrite params set in sensor_command.c when initiating SPD module.
        // Set init_spd_settings=1 to run spd with default params (set in spd.c).
        status_spd = SPD_init(&SPD_Data, &SEN_Info, 0);
    }
    return status_spd;
}

/**
 *
 */
int SEN_update_info() {
    int i, ii;
    SEN_Info.orientation = (SEN_Orientation_t) Params.spdSensorRotation;
    SEN_Info.nb_zones = Params.spdResolution;
    if (SEN_Info.nb_zones == 16) {
        SEN_Info.res_x = 4;
        SEN_Info.res_y = 4;
    } else if (SEN_Info.nb_zones == 64) {
        SEN_Info.res_x = 8;
        SEN_Info.res_y = 8;
    } else {
        return 1;
    }
    SEN_Info.freq = 1 / (float) (Params.spdRangingPeriod / 1000.0);

    // SCI configuration
    SEN_Info.sci__dmax = Params.sciDmax;
    SEN_Info.sci__config_data.sci__detection_threshold = sci_config.detection_threshold / 65536.0;
    for (i = 0; i < SENSOR__MAX_NB_OF_ZONES; i++) {
        ii = SEN_get_sensor_zoneID_from_rotation_corrected_zoneID(i, SEN_Info.orientation, SEN_Info.res_x);
        SEN_Info.sci__config_data.sci__aggregate_id_map[i] = sci_config.map_id[ii];
    }

    return 0;
}

/**
 *
 */
int init_vl53lmz_sensor() {
    int status = 0;
    VL53LMZ_ModuleInfo module_info;
    VL53LMZ_FWVersion fw_version;

    // Ideally do a HW reset of the sensor by toggling AVDD & IOVDD
    Reset_Sensor(&(LMZDev.platform)); /* Platform function, not in API */

    // Finally init sensor (SW reset sequence + sensor boot)
    status = vl53lmz_init(&LMZDev);
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("vl53lmz_init failed : %d\n", status);
        return status;
    }
    // Reset interrupt counter as interrupt line has toggled (twice) during sttof_init call
    IntCount = 0;

    // Read sensor module type : 0 is L5, 1 is L7, 2 is L8
    uart_printf("Module type = %s\n",
            LMZDev.module_type == 2 ?
                    "VL53L8CX" :
                    (LMZDev.module_type == 1 ? "VL53L7CX" : (LMZDev.module_type == 0 ? "VL53L5CX" : "Unknown")));
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("read_module_type failed : %d\n", status);
        return status;
    }
    if ((LMZDev.module_type != 0) && (LMZDev.module_type != 1) && (LMZDev.module_type != 2)) {
        uart_printf("Module type %u not supported !\n", LMZDev.module_type);
        return status;
    }

    // Read sensor HW unic ID
    status = vl53lmz_get_module_info(&LMZDev, &module_info);
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("vl53lmz_get_module_info failed : %d\n", status);
        return status;
    }
    uart_printf("Module ID : %08X%08X_%s\n", (unsigned int) (module_info.module_id_hi),
            (unsigned int) (module_info.module_id_lo), module_info.fgc);

    // Read sensor FW version
    status = vl53lmz_get_fw_version(&LMZDev, &fw_version);
    if (status) {
        uart_printf("vl53lmz_get_fw_version failed : status %d\n", status);
        return status;
    }
    uart_printf("FW Version : %d.%d.%d.%d\n", fw_version.fw_major, fw_version.fw_minor, fw_version.fw_build,
            fw_version.fw_revision);

    // Read ULD driver version
    uart_printf("ULD Driver %s\n", VL53LMZ_API_REVISION);

    // Go to VL53LMZ_POWER_MODE_SLEEP mode
    status = vl53lmz_set_power_mode(&LMZDev, VL53LMZ_POWER_MODE_SLEEP);
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("vl53lmz_set_power_mode WAKEUP failed : %d\n", status);
        return status;
    }

    // Update calibration status
    xtalk64_calibration_stored = 0;
    xtalk_calibrated = 0;

    return status;
}


int vl53lmz_sci_calculate_required_memory(
        VL53LMZ_Motion_Configuration    *p_sci_config,
        uint32_t                        *p_required_memory)
{
    uint16_t mem_feature;
    uint16_t mem_var;
    uint16_t mem_amb_per_bin;
    uint16_t mem_amb_per_bin_var;
    uint16_t mem_fp_description;
    uint16_t mem_overheard;

    mem_feature = VL53LMZ_SCI_FEATURE_ELT_BYTE
            * p_sci_config->nb_of_aggregates
            * p_sci_config->feature_length
            * (1+VL53LMZ_SCI_STORE_PREV_F);
    mem_var = VL53LMZ_SCI_FEATURE_ELT_BYTE
            * p_sci_config->nb_of_aggregates
            * (p_sci_config->feature_length*VL53LMZ_SCI_STORE_VAR)
            * (1+VL53LMZ_SCI_STORE_PREV_F);
    mem_amb_per_bin = VL53LMZ_SCI_FEATURE_ELT_BYTE
            * (1-VL53LMZ_SCI_STORE_VAR)
            * p_sci_config->nb_of_aggregates
            * (1+VL53LMZ_SCI_STORE_PREV_F);
    mem_amb_per_bin_var = p_sci_config->nb_of_aggregates
            * 4;
    mem_fp_description = p_sci_config->nb_of_aggregates
            * (p_sci_config->feature_length
                    + p_sci_config->feature_length
                    * VL53LMZ_SCI_STORE_VAR+1)
            * (1+VL53LMZ_SCI_STORE_PREV_F)
            + (1-VL53LMZ_SCI_STORE_VAR)
                * p_sci_config->nb_of_aggregates
                * (1+VL53LMZ_SCI_STORE_PREV_F);
    mem_overheard = 1+(1+VL53LMZ_SCI_STORE_PREV_F)
            +p_sci_config->nb_of_aggregates;

    *p_required_memory =
            mem_feature
            +mem_var
            +mem_amb_per_bin
            +mem_amb_per_bin_var
            +mem_fp_description
            +mem_overheard
            +VL53LMZ_SCI_PADDING_BYTES
            +VL53LMZ_SCI_PARTIALS_BYTES;

    return VL53LMZ_STATUS_OK;
}


/**
 *
 */
int sci_set_config(struct Params_t *p_Params) {
    int status = 0;
    int i = 0;
    uint32_t req_size;

    sci_config.ref_bin_offset = (int32_t) ((((p_Params->sciDmin / 10.0) / VL53LMZ_SCI_BIN_WIDTH_CM)
            - (VL53LMZ_SCI_PULSE_WIDTH_BIN / 2)) * 2048 + 0.5);
    sci_config.detection_threshold = (uint32_t) (p_Params->sciDetectionThreshold * 65536);
    sci_config.extra_noise_sigma = 0;
    sci_config.null_den_clip_value = 0;
    sci_config.mem_update_mode = p_Params->sciMemUpdateMode;
    sci_config.mem_update_choice = 2;
    sci_config.sum_span = (uint8_t) (p_Params->sciSumSpan);
    sci_config.feature_length = (uint8_t) (((((p_Params->sciDmax - p_Params->sciDmin) / 10.0)
            + VL53LMZ_SCI_PULSE_WIDTH_BIN * VL53LMZ_SCI_BIN_WIDTH_CM)
            / (p_Params->sciSumSpan * VL53LMZ_SCI_BIN_WIDTH_CM)) + 0.5);
    sci_config.nb_of_aggregates = (uint8_t) (p_Params->sciAggNb);
    sci_config.nb_of_temporal_accumulations = (uint8_t) (p_Params->sciTemporalAgg);
    sci_config.min_nb_for_global_detection = 1;
    sci_config.global_indicator_format_1 = 8;
    sci_config.global_indicator_format_2 = 0;
    sci_config.cnh_cfg = 0;
    sci_config.cnh_flex_shift = 0;
    sci_config.spare_3 = 0;

    if (p_Params->sciAggNb == 16) {
        if (p_Params->Resolution == 16) {
            /*
             Aggregation Map: (sensor view)
             12 13 14 15
             8  9 10 11
             4  5  6  7
             0  1  2  3
             */
            for (i = 0; i < 16; i++) {
                sci_config.map_id[i] = i;
            }
            memset(sci_config.map_id + 16, -1, 48);
        } else if (p_Params->Resolution == 64) {
            /*
             Aggregation Map: (sensor view)
             12 12 13 13 14 14 15 15
             12 12 13 13 14 14 15 15
             8  8  9  9 10 10 11 11
             8  8  9  9 10 10 11 11
             4  4  5  5  6  6  7  7
             4  4  5  5  6  6  7  7
             0  0  1  1  2  2  3  3
             0  0  1  1  2  2  3  3
             */
            for (i = 0; i < 64; i++) {
                sci_config.map_id[i] = (i % 8) / 2 + 4 * (i / 16);
            }
        } else {
            uart_printf("Resolution %d not supported !", p_Params->Resolution);
            return -1;
        }
    } else if (p_Params->sciAggNb == 1) {
        /*
         Aggregation Map: (sensor view)
         0  0  0  0  0  0  0  0
         0  0  0  0  0  0  0  0
         0  0  0  0  0  0  0  0
         0  0  0  0  0  0  0  0
         0  0  0  0  0  0  0  0
         0  0  0  0  0  0  0  0
         0  0  0  0  0  0  0  0
         0  0  0  0  0  0  0  0
         */
        memset(sci_config.map_id, 0, 64);
    } else {
        uart_printf("sciAggNb %d not supported\n", p_Params->sciAggNb);
        return -1;
    }

    // Calculate required memory and check it is less than 6152 Bytes
    status = vl53lmz_sci_calculate_required_memory(&sci_config, &req_size);
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("vl53lmz_sci_calculate_required_memory failed : %d\n", status);
        return status;
    }
    if (req_size > VL53LMZ_SCI_MAX_BUFFER_SIZE) {
        uart_printf("Required memory is too high !\n");
        return -1;
    }

    // Program SCI
    status = vl53lmz_dci_write_data(&LMZDev, (uint8_t*) &(sci_config), VL53LMZ_DCI_MOTION_DETECTOR_CFG,
            sizeof(sci_config));
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("vl53lmz_dci_write_data DCI failed : %d\n", status);
        return status;
    }

    return status;
}

uint8_t vl53lmz_spd_set_output_pipe(VL53LMZ_Configuration *p_dev, uint8_t nb_target_per_zone,
        uint8_t disable_pipe_control, uint8_t enable_sharpener_post_filter, uint8_t auto_stop) {
    uint8_t status = VL53LMZ_STATUS_OK;
    uint8_t output_pipe[] = { nb_target_per_zone, disable_pipe_control, /* disable_pipe_control (0 or 1) */
    enable_sharpener_post_filter, /* enable_sharpener_post_filter (0 or 1)*/
    auto_stop }; /* Enable autostop on interrupt */

    status = vl53lmz_dci_write_data(p_dev, (uint8_t*) &(output_pipe), VL53LMZ_DCI_PIPE_CONTROL, sizeof(output_pipe));

    p_dev->is_auto_stop_enabled = auto_stop;

    return status;
}

int vl53lmz_Configure(void) {
    int status = 0;
    int i = 0;
    VL53LMZ_DetectionThresholds checkers[VL53LMZ_NB_THRESHOLDS];
    //uart_printf("===============Program Algo Pipe enable/disable\n");
    status = vl53lmz_spd_set_output_pipe(&LMZDev, VL53LMZ_NB_TARGET_PER_ZONE, Params.disablePipe, 0x01,
            Params.auto_stop);
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("ERROR at %s(%d) : vl53lmz_spd_set_output_pipe failed : %d\n", __func__, __LINE__, status);
        return status;
    }
    //uart_printf("===============Program Resolution (offset and xtalk)\n");
    status = vl53lmz_set_resolution(&LMZDev, Params.Resolution == 16 ? VL53LMZ_RESOLUTION_4X4 : VL53LMZ_RESOLUTION_8X8);
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("ERROR at %s(%d) : vl53lmz_set_resolution failed : %d\n", __func__, __LINE__, status);
        return status;
    }
    status = vl53lmz_set_ranging_frequency_hz(&LMZDev, 1000.0 / Params.RangingPeriod);
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("ERROR at %s(%d) : vl53lmz_set_ranging_frequency_hz failed : %d\n", __func__, __LINE__, status);
        return status;
    }
    status = vl53lmz_set_integration_time_ms(&LMZDev, Params.IntegrationTime);
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("ERROR at %s(%d) : vl53lmz_set_integration_time_ms failed : %d\n", __func__, __LINE__, status);
        return status;
    }
    status = vl53lmz_set_sharpener_percent(&LMZDev, Params.SharpenerEnable ? 5 : 0);
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("ERROR at %s(%d) : vl53lmz_set_sharpener_percent failed : %d\n", __func__, __LINE__, status);
        return status;
    }
    status = vl53lmz_set_xtalk_margin(&LMZDev, Params.XtalkMargin);
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("ERROR at %s(%d) : vl53lmz_set_xtalk_margin failed : %d\n", __func__, __LINE__, status);
        return status;
    }
    status = sci_set_config(&Params);
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("ERROR at %s(%d) : sci_set_config failed : %d\n", __func__, __LINE__, status);
        return status;
    }

#ifdef VL53LMZ_XTALK_CALIBRATION
      if (xtalk64_calibration_stored){
          //uart_printf("===============Program Xtalk\n");
          status = vl53lmz_set_caldata_xtalk(&LMZDev, xtalk_calibration_buffer);
          if (status != VL53LMZ_STATUS_OK){
              uart_printf("ERROR at %s(%d) : vl53lmz_set_caldata_xtalk failed : %d\n",__func__, __LINE__,status);
              return status;
          }
          xtalk_calibrated = Params.Resolution;
      }
      #endif

    // Go to Autonomous - Enable Detection Thresholds on Motion Detection
    if (Params.InterruptMode == 1) {
        // Standard autonomous mode on SCI 4x4 full FoV
        memset(&checkers, 0, sizeof(checkers));
        for (i = 0; i < 16; i++) {
            checkers[i].measurement = VL53LMZ_MOTION_INDICATOR;
            checkers[i].type = VL53LMZ_GREATER_THAN_MAX_CHECKER;
            checkers[i].param_high_thresh = Params.sciDetectionThreshold;
            ;
            checkers[i].zone_num = i;
        }
        checkers[15].zone_num += VL53LMZ_LAST_THRESHOLD;

        status = vl53lmz_set_detection_thresholds(&LMZDev, checkers);
        if (status != VL53LMZ_STATUS_OK) {
            uart_printf("ERROR at %s(%d) : vl53lmz_set_detection_thresholds failed : %d\n", __func__, __LINE__, status);
            return status;
        }
        status = vl53lmz_set_detection_thresholds_enable(&LMZDev, 1);
        if (status != VL53LMZ_STATUS_OK) {
            uart_printf("ERROR at %s(%d) : vl53lmz_set_detection_thresholds_enable failed : %d\n", __func__, __LINE__,
                    status);
            return status;
        }
    } else {
        // Tracking mode : disable Detection Thresholds
        status = vl53lmz_set_detection_thresholds_enable(&LMZDev, 0);
        if (status != VL53LMZ_STATUS_OK) {
            uart_printf("ERROR at %s(%d) : vl53lmz_set_detection_thresholds_enable failed : %d\n", __func__, __LINE__,
                    status);
            return status;
        }
    }

    // Update the new sensor information
    status = SEN_update_info();

    return status;
}

/**
 * Prepare the params for Tracking mode
 */
int SEN_SetParamsForTracking() {
    Params.InterruptMode = 0;
    Params.Resolution = Params.spdResolution;
    Params.RangingPeriod = Params.spdRangingPeriod;
    Params.IntegrationTime = Params.spdIntegrationTime;
    Params.sciDmax = Params.spdApproachDistance_mm;
    Params.sciDmin = (Params.sciDmax < SPD_SCI_DEFAULT_DCIMAX) ?
    SPD_SCI_DEFAULT_DCIMIN :
                                                                 (Params.sciDmax - SPD_SCI_MAX_SUPPORTED_RANGE);
    Params.disablePipe = 0;
    Params.sciDetectionThreshold =
            LMZDev.platform.module_type == 0 ? 44 : (LMZDev.platform.module_type == 1 ? 44 : 150); //L5=44 / L7 = 44 / L8=150
    Params.sciMemUpdateMode = 6;
    Params.sciTemporalAgg = 16;
    Params.auto_stop = 0;

    return 0;
}

void UartComm_Start() {
    HAL_UART_StateTypeDef State;
    _CriticalEnter();
    Uart_RxRcvIndex = 0;
    UartActive = 1;
    State = HAL_UART_GetState(&huart2);
    if (State != HAL_UART_STATE_BUSY_TX_RX && State != HAL_UART_STATE_BUSY_RX) {
        // arm it
#if DMA_RX
        HAL_UART_Receive_DMA(&huart2, (uint8_t *) Uart_RXBuffer, sizeof(Uart_RXBuffer));
#else
        HAL_UART_Receive_IT(&huart2, (uint8_t*) Uart_RXBuffer, 1);
#endif
    }
    _CriticalExit();
}

void UartComm_Stop() {
    _CriticalEnter();
    UartActive = 0;
    //we may stop/deinit uart to ensure recetpion is aborted
    _CriticalExit();
}

/**
 * Returns a non null value if a new ranging data is available
 */
int get_ranging_data(void) {
    int new_data = 0;
    int status = 0;

    __WFI(); // Wait For Interrupt
    if (IntCount != 0) {
        IntCount = 0;
        HAL_Delay(1);
        status = vl53lmz_get_ranging_data(&LMZDev, &RangingData);
        if (status == 0) {
            new_data = 1;
        } else {
            // Ranging error
            uart_printf("Ranging Error : %d !\n", status);
            new_data = -1;
        }
    }
    return new_data;
}

/**
 * Copy VL53LMZ measurement data into the sensor data structure
 * The function corrects the rotation of the sensor
 */
int SEN_CopyMeasurementData(SEN_Measurement_Data_t *dest, VL53LMZ_ResultsData *src_ranging, uint64_t timestamp_ms) {
    int zone_index = 0, rot_corrected_zone_index = 0;
    int target_index;
    int t, z, a;

    if (src_ranging->motion_indicator.nb_of_aggregates > SENSOR__SCI_MAX_NB_OF_AGG) {
        uart_printf("Error: sci__nb_of_aggregates is greater than SENSOR__SCI_MAX_NB_OF_AGG\n");
        return 1;
    }

    dest->timestamp_ms = timestamp_ms;
    for (zone_index = 0; zone_index < Params.Resolution; zone_index++) {
        rot_corrected_zone_index = SEN_get_rotation_corrected_zoneID_from_sensor_zoneID(zone_index,
                SEN_Info.orientation, SEN_Info.res_x);
        dest->AmbientRatePerSpad[rot_corrected_zone_index] = (float) (src_ranging->ambient_per_spad[zone_index])
                / (float) 2048;                    // unsigned 19.11
        for (t = 0; t < SENSOR__NB_OF_TARGETS_PER_ZONE; t++) {
            target_index = zone_index * VL53LMZ_NB_TARGET_PER_ZONE + t;
            dest->RangeMilliMeter[t][rot_corrected_zone_index] = (float) (src_ranging->distance_mm[target_index])
                    / (float) 4;                  // signed 14.2
            dest->SignalRatePerSpad[t][rot_corrected_zone_index] = (float) (src_ranging->signal_per_spad[target_index])
                    / (float) 2048;         // unsigned 19.11
            if (dest->RangeMilliMeter[t][rot_corrected_zone_index] <= Params.spdApproachDistance_mm) {
                dest->RangeStatus[t][rot_corrected_zone_index] = src_ranging->target_status[target_index];
            } else {
                dest->RangeStatus[t][rot_corrected_zone_index] = 0;
            }
        }
    }

    dest->sci__output_data.sci__global_indicator = (uint8_t) (src_ranging->motion_indicator.global_indicator_1);
    dest->sci__output_data.sci__status = src_ranging->motion_indicator.status;
    dest->sci__output_data.sci__nb_of_detected_aggregates = src_ranging->motion_indicator.nb_of_detected_aggregates;
    dest->sci__output_data.sci__nb_of_aggregates = src_ranging->motion_indicator.nb_of_aggregates;
    if (sci_motion_map_stored) {
        // Replace the SCI motion map by the one previously stored
        memcpy(src_ranging->motion_indicator.motion, sci_motion_map,
                src_ranging->motion_indicator.nb_of_aggregates * sizeof(uint32_t));
        sci_motion_map_stored = 0;
    }
    for (z = 0; z < SENSOR__MAX_NB_OF_ZONES; z++) {
        a = SEN_Info.sci__config_data.sci__aggregate_id_map[z]; // SEN_Info->sci__config_data.sci__aggregate_id_map contains a rotated aggregate id map
        dest->sci__output_data.sci__per_zone_indicator[z] = (float) (src_ranging->motion_indicator.motion[a]
                / (float) 65536); // Unsigned 16.16
    }

    return 0;
}

/**
 * Prepare the params for Tracking mode
 */
int SEN_SetParamsForAutonomous() {
    Params.InterruptMode = Params.spdAutonomousInterruptMode;
    Params.Resolution = 16;
    Params.RangingPeriod = Params.spdAutonomousRangingPeriod;
    Params.IntegrationTime = Params.spdAutonomousIntegrationTime;
    Params.sciDmax = Params.spdApproachDistance_mm;
    Params.sciDmin =
            (Params.sciDmax < SPD_SCI_DEFAULT_DCIMAX) ?
                    SPD_SCI_DEFAULT_DCIMIN : (Params.sciDmax - SPD_SCI_MAX_SUPPORTED_RANGE);
    Params.disablePipe = 1; //Used to decrease the power consumption
    Params.sciDetectionThreshold =
            LMZDev.platform.module_type == 0 ? 100 : (LMZDev.platform.module_type == 1 ? 150 : 100); //L5=100 / L7 = 150 / L8=100
    Params.sciMemUpdateMode = 1;
    Params.sciTemporalAgg = 1;
    Params.auto_stop = 1; //Used to save time to reprogram the sensor before going back to tracking, it avoid sending the stop_ranging command...

    return 0;
}

/**
 * Change sensor mode according to SPD algo request
 */
int SPD_ChangeSensorMode(SPD_Data_t *pSPDData) {
    int status = 0; // 1 Fail, 0 Ok

    // Wake-on SCI interrupt - Back to tracking mode
    if (Params.InterruptMode != 0) {
        SPD_DEBUG(2, "Changing mode to: %d pix, %.2fHz, %d ms, DataReady", Params.spdResolution,
                1000.0/Params.spdRangingPeriod, Params.spdIntegrationTime);
        // Store the 4x4 SCI map containing the wake-up motion
        if (sci_config.nb_of_aggregates > SENSOR__SCI_MAX_NB_OF_AGG) {
            uart_printf("sci__nb_of_aggregates %d not supported (only 16 supported) !\n", sci_config.nb_of_aggregates);
            return status;
        }
        memcpy(sci_motion_map, RangingData.motion_indicator.motion, sci_config.nb_of_aggregates * sizeof(uint32_t));
        sci_motion_map_stored = 1;
        // Stop sensor ranging (if not already done by the AutoStop feature itself)
        status = vl53lmz_stop_ranging(&LMZDev);
        if (status != VL53LMZ_STATUS_OK) {
            uart_printf("vl53lmz_stop_ranging failed : %d\n", status);
            return status;
        }
        // Configure sensor in 8x8 DataReady mode
        //Tracking
        SEN_SetParamsForTracking();
        status = vl53lmz_Configure();
        if (status != VL53LMZ_STATUS_OK) {
            uart_printf("ERROR at %s(%d) : vl53lmz_Configure failed : %d\n", __func__, __LINE__, status);
            return status;
        }
        // Start ranging
        status = vl53lmz_start_ranging(&LMZDev);
        if (status != VL53LMZ_STATUS_OK) {
            uart_printf("vl53lmz_start_ranging failed : %d\n", status);
            return status;
        }
        ranging = 1;

    }
    // Go into autonomous mode
    else if (Params.spdAutonomousInterruptMode != 0 && pSPDData->state == SPD_AUTONOMOUS) {
        SPD_DEBUG(2, "Changing mode to: 16 pix, %.2f Hz, %d ms, Autonomous on SCI",
                1000.0/Params.spdAutonomousRangingPeriod, Params.spdAutonomousIntegrationTime);
        // Stop ranging
        status = vl53lmz_stop_ranging(&LMZDev);
        if (status != VL53LMZ_STATUS_OK) {
            uart_printf("vl53lmz_stop_ranging failed : %d\n", status);
            return status;
        }
        // Configure sensor in 4x4 Autonomous mode with SCI (algo pipe disabled) only and Interrupt checkers activated
        SEN_SetParamsForAutonomous();
        status = vl53lmz_Configure();
        if (status != VL53LMZ_STATUS_OK) {
            uart_printf("ERROR at %s(%d) : vl53lmz_Configure failed : %d\n", __func__, __LINE__, status);
            return status;
        }
        // Start ranging
        status = vl53lmz_start_ranging(&LMZDev);
        if (status != VL53LMZ_STATUS_OK) {
            uart_printf("vl53lmz_start_ranging failed : %d\n", status);
            return status;
        }

        // Reset Interrupt counter to force un-served interrupts to be dropped
        IntCount = 0;

    } else {
        // Nothing
    }
    return status;
}

void application_setup(void) {

    UartComm_Start();

#ifdef STM32F401xE
    /* patch the uart setup not to do filtering */
    huart2.Instance->CR3 |= USART_CR3_ONEBIT;
  #endif

    /* Clean the Serial Terminal */
    HAL_Delay(1000);
    uart_printf("\x1b[2J");
    uart_printf("\x1b[1;1H");
    uart_printf("SPD ULD SW Kit version %s\n", SPD_KIT_ULD_VERSION);

    /* Initialize VL53LMZ I2C address */
    LMZDev.platform.address = 0x52;

    /* Initialize sensor */
    status = init_vl53lmz_sensor();
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("init_vl53lmz_sensor failed : %d\n", status);
    }

    /* Sensor identification */
    Params.spdRangingPeriod = (LMZDev.module_type == 1) ? SPD_RANGING_PERIOD_L7 : SPD_RANGING_PERIOD_L8;
    Params.spdIntegrationTime = (LMZDev.module_type == 1) ? SPD_INTEGRATION_TIME_L7 : SPD_INTEGRATION_TIME_L8;
    Params.spdAutonomousIntegrationTime =
            (LMZDev.module_type == 1) ? SPD_AUTONOMOUS_INTEGRATION_TIME_L7 : SPD_AUTONOMOUS_INTEGRATION_TIME_L8;

    /* Initialize SPD algo */
    start_profiling();
    SPD_Init(1);
    stop_profiling("SPD_Init()");

    // Reset sensor settings in Tracking mode (after SPD init)
    SEN_SetParamsForTracking();
    sci_motion_map_stored = 0;

    /* Init time data */
    time_data.curr_tstp = GET_TIME_STAMP();
}

void application_loop(void) {

    while (1) {

        /*******************************/
        /*         RANGING Mode        */
        /*******************************/
        /* Check if a new VL53LMZ ranging data is available */
        start_profiling();
        if (ranging && get_ranging_data()) {
            /* time stats */
            time_data.prev_tstp = time_data.curr_tstp;
            time_data.curr_tstp = GET_TIME_STAMP();
            time_data.ranging_period = time_data.curr_tstp - time_data.prev_tstp;
            /* Fill-in SPD input RangeData structure from VL53LMZ ranging data */
            SEN_CopyMeasurementData(&SEN_MeasData, &RangingData, time_data.curr_tstp);
            stop_profiling("get_ranging_data() + SEN_CopyMeasurementData()");
            /* Do not call SPD_run if this is the first interrupt from Autonomous mode (wake-up interrupt) */
            if (Params.InterruptMode == 0) {
                /* Run SPD */
                start_profiling();
                SPD_run(&SPD_Data, &SEN_MeasData, &SEN_Info);
                stop_profiling("SPD_run()");
                /* Data logging */
                print_data();
            }

            /* Possibly change sensor mode */
            SPD_ChangeSensorMode(&SPD_Data);
        }
    }
}
