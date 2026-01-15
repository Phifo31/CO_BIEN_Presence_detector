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

#include "../../CO_BIEN_Konectik/Common/can_ids.h"

// Choix de l'id du capteur : 3 lignes à commenter 1 seule ligne à activer
//#define CAN_SENSOR_ID   SPD_ADRESS_SENSOR_SOUTH
//#define CAN_SENSOR_ID   SPD_ADRESS_SENSOR_NORTH
//#define CAN_SENSOR_ID   SPD_ADRESS_SENSOR_EAST
#define CAN_SENSOR_ID   SPD_ADRESS_SENSOR_WEST

#define GET_TIME_STAMP()  (int32_t)HAL_GetTick()
#define UART_BUFFER_SIZE    2048

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

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

#include "can_bus.h"

#include "application.h"

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
uint8_t xtalk_calibration_buffer[VL53LMZ_XTALK_BUFFER_SIZE];
#endif
VL53LMZ_Motion_Configuration sci_config;

uint8_t isAlive, isReady;
uint32_t maxMotion, motionPower;
int16_t dist, minDist, prevMinDist;

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
int sci_set_config(struct Params_t *p_Params);
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

CAN_BUS can_bus(CAN_SENSOR_ID);
volatile bool auto_led_debug = true;
uint32_t MOTION_THRESHOLD = 3000000; // sans unit
uint32_t DIST_THRESHOLD = 400; // en mm
uint32_t PERIODE_FILTRAGE = 10000; // en ms
uint32_t SPD_IMMOBILE_TIME_MS = 2000;

#if ENABLE_USER_LOG
/**
 * Affichage de tick système en secondes et milli-secondes
 */
uint16_t seconds;
uint16_t reste;
uint16_t msec2sec(uint32_t n, uint16_t *reste) {
    // Récupéré là mais je n'y comprend rien
    // https://stackoverflow.com/questions/1294885/convert-milliseconds-to-seconds-in-c
    //  uint32_t q, r, t;
    //n = n + 500; pourquoi ?
    //t = (n >> 7) + (n >> 8) + (n >> 12);
    //q = (n >> 1) + t + (n >> 15) + (t >> 11) + (t >> 14);
    //q = q >> 9;
    //r = n - q*1000;
    //return q + ((r + 24) >> 10);

    // ==> donc voici ma version :
    uint32_t q = n / 1000;
    *reste = n - 1000 * q;
    return (uint16_t) q;
}

// Redirection du printf sur UART2 (pour une nucleo 64)
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, 100);
    return (len);
}

/*
 int __io_putchar(int ch) {
 HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 100);
 //ITM_SendChar(ch);
 return (ch);
 } */

#endif

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

// UART functions

void Uart_DoCommand() {

    if (UartComm_CmdReady == 0) {
        memcpy(UartComm_RXBuffer, (char*) Uart_RXBuffer, Uart_RxRcvIndex + 1); //copy ending 0
        UartComm_RXSize = Uart_RxRcvIndex;
        UartComm_CmdReady = 1;
    } else {
        //TODO full command got lost
    }
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    int ContinueRX = 1;
    Uart_nRxCb++;
    if (!UartActive) {
        return;
    }

    nUartRxCb_User++;
    if (Uart_RXBuffer[Uart_RxRcvIndex] == '\r' || Uart_RXBuffer[Uart_RxRcvIndex] == '\n') {
        Uart_RXBuffer[Uart_RxRcvIndex] = 0;
        ContinueRX = 0;
        Uart_DoCommand();
    } else {
        if (Uart_RxRcvIndex >= sizeof(Uart_RXBuffer) - 1) {
            // overrun the buffer reset
            Uart_RxRcvIndex = 0;
            Uart_nOverrun++;
        } else {
            Uart_RxRcvIndex++;
        }
    }
    if (ContinueRX)
        HAL_UART_Receive_IT(huart, (uint8_t*) &Uart_RXBuffer[Uart_RxRcvIndex], 1);
}

/*
 * Sensor management functions
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
        USER_LOG("vl53lmz_init failed : %d", status);
        return status;
    }
    // Reset interrupt counter as interrupt line has toggled (twice) during sttof_init call
    IntCount = 0;

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
    status = vl53lmz_set_power_mode(&LMZDev, VL53LMZ_POWER_MODE_WAKEUP);
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("vl53lmz_set_power_mode WAKEUP failed : %d\n", status);
        return status;
    }

    // Update calibration status
    xtalk64_calibration_stored = 0;
    xtalk_calibrated = 0;

    return status;
}

int sci_set_config(struct Params_t *p_Params) {
    int status = 0;
    int i = 0;

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
    uart_printf("XTALK calibration start\n");
//        	  status = vl53lmz_calibrate_xtalk(
//        	  		VL53LMZ_Configuration		*p_dev,
//        	  		uint16_t			reflectance_percent,
//        	  		uint8_t				nb_samples,
//        	  		uint16_t			distance_mm)
    /* Start Xtalk calibration with a 3% reflective target at 600mm for the
     * sensor, using 4 samples.
     */
    status = vl53lmz_calibrate_xtalk(&LMZDev, 3, 4, 600);
    if (status) {
        uart_printf("XTALK calibration failed, status %u\n", status);
        return status;
    } else {
        uart_printf("Xtalk calibration done\n");

        /* Get Xtalk calibration data, in order to use them later */
        status = vl53lmz_get_caldata_xtalk(&LMZDev, xtalk_calibration_buffer);

        if (status) {
            uart_printf("XTALK get caldata failed, status %u\n", status);
            return status;
        }
        /* Set Xtalk calibration data */
        status = vl53lmz_set_caldata_xtalk(&LMZDev, xtalk_calibration_buffer);
        if (status) {
            uart_printf("XTALK set caldata  failed, status %u\n", status);
            return status;
        }
    }
//      xtalk_calibrated = Params.Resolution;
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

    return status;
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
/*
 * DEBUG FUNCTIONS
 */

/**
 * Gestion de la led de vie (tant qu'elle n'est pas utilisée par le BUS CAN
 */
uint32_t change_state_user_led(void) {

    static bool led_state = false;
    uint32_t time = 0;

    if (led_state) {
        led_state = false;
        if (auto_led_debug) {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }
        time = USER_LED_HIGH_TIME;
    } else {
        led_state = true;
        if (auto_led_debug) {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        }
        time = USER_LED_LOW_TIME;
    }
    return time;
}

/**
 *
 */
uint16_t can_bus_callback_debug_led(uint16_t sender, uint8_t data[6]) {

    switch (data[0]) {
    case 0:
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        auto_led_debug = false;
        break;

    case 1:
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        auto_led_debug = false;
        break;

    case 2:
    default:
        auto_led_debug = true;
    }
    return 0;
}

/**
 *
 */
uint16_t can_bus_callback_rxConfig(uint16_t sender, uint8_t data[6]) {
    uint32_t valeur = 0;
    uart_printf("Message recu : %02X %02X %02X %02X %02X %02X\r\n", data[0], data[1], data[2], data[3], data[4],
            data[5]);

    valeur = (data[5] << 24) + (data[4] << 16) + (data[3] << 8) + data[2];
    uart_printf("Valeur : %u\r\n", valeur);

    switch (data[0]) {
    case 0:
        DIST_THRESHOLD = valeur; // en mm
        break;
    case 1:
        MOTION_THRESHOLD = valeur; // sans unité
        break;
    case 2:
        PERIODE_FILTRAGE = valeur; // en ms
        break;
    case 4:
        SPD_IMMOBILE_TIME_MS = valeur; // RESET
        break;
    case 3:
        application_setup(); // RESET
        break;
//	case 4:
//		();
//		break;
    default:
        DIST_THRESHOLD = 400; // en mm
        MOTION_THRESHOLD = 3000000; // sans unité
        PERIODE_FILTRAGE = 10000; // en ms
    }

    return 0;
}

/**
 * APPLICATION
 */
void application_setup(void) {

    int status = 0;

    UartComm_Start();

#ifdef STM32F401xE
    /* patch the uart setup not to do filtering */
    huart2.Instance->CR3 |= USART_CR3_ONEBIT;
#endif

    /* Clean the Serial Terminal */
    uart_printf("\x1b[2J");         // ESC [ 2J        Erase entire screen
    uart_printf("\x1b[1;1H");
    uart_printf("SPD ULD SW Kit version %s\r\n", SPD_KIT_ULD_VERSION);
    //USER_LOG("SPD ULD SW Kit version %s", SPD_KIT_ULD_VERSION);

    /* Initialize VL53LMZ I2C address */
    LMZDev.platform.address = VL53LMZ_DEFAULT_I2C_ADDRESS;         //0x52;

    /* Initialize sensor */
    status = init_vl53lmz_sensor();
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("init_vl53lmz_sensor failed : %d\r\n", status);
    }

    uart_printf("Module type = %s\r\n",
            LMZDev.module_type == 2 ?
                    "VL53L8CX" :
                    (LMZDev.module_type == 1 ? "VL53L7CX" : (LMZDev.module_type == 0 ? "VL53L5CX" : "Unknown")));
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("read_module_type failed : %d\r\n", status);
    }
    Params.spdRangingPeriod = 500;
    Params.spdIntegrationTime = 450;
//    Params.spdAutonomousIntegrationTime = 100;
//    Params.InterruptMode = 0;
    Params.Resolution = Params.spdResolution;         //64;
//    Params.sciDmax = Params.spdApproachDistance_mm;
//    Params.sciDmin = (Params.sciDmax < SPD_SCI_DEFAULT_DCIMAX) ?
//    SPD_SCI_DEFAULT_DCIMIN :
//                                                                 (Params.sciDmax - SPD_SCI_MAX_SUPPORTED_RANGE);
//    Params.disablePipe = 0;
//    Params.sciDetectionThreshold = 44; // LMZDev.platform.module_type == 0 ? 44 : (LMZDev.platform.module_type == 1 ? 44 : 150); //L5=44 / L7 = 44 / L8=150
//    Params.sciMemUpdateMode = 6;
//    Params.sciTemporalAgg = 16;
//    Params.auto_stop = 0;

    /* Init time data */
    time_data.curr_tstp = GET_TIME_STAMP();

    // Initialisation du bus CAN
    can_bus.begin();
//    can_bus.register_callback_function(ARBITRATION_ID_LED_CONFIG, can_bus_callback_debug_led);
    can_bus.register_callback_function(arbitrationId_t(0x474), can_bus_callback_rxConfig);
    can_bus.register_callback_function(ARBITRATION_ID_LED_CONFIG, can_bus_callback_debug_led);

    // Configure VL53LMZ
    status = vl53lmz_Configure();
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("ERROR at %s(%d) : vl53lmz_Configure failed : %d\r\n", __func__, __LINE__, status);
    }

    // Start ranging
    status = vl53lmz_start_ranging(&LMZDev);
    if (status != VL53LMZ_STATUS_OK) {
        uart_printf("ERROR at %s(%d) : vl53lmz_start_ranging failed : %d\r\n", __func__, __LINE__, status);
        ranging = 0;
    }

    maxMotion = 0;
    motionPower = 0;
    minDist = 0;
    prevMinDist = DIST_THRESHOLD;
    dist = 0;
}

/**
 *
 */
int application_loop(void) {

    int status = 0;
    int i = 0;
    uint32_t time_for_can_bus_automatic_message = 15000;
    //uint32_t time_previous_movement = HAL_GetTick();
    uint32_t time_previous_near = HAL_GetTick();
    uint32_t time_for_change_led_state = 0;
    uint32_t current_time = HAL_GetTick();
    uint32_t last_motion_time;
    bool is_immobile = true;

    while (1) {
        //void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
        status = vl53lmz_check_data_ready(&LMZDev, &isReady);
        UNUSED(status);
        maxMotion = 0;
        minDist = 3500;
        last_motion_time = 0xFFFFFFFF;

        if (isReady) {
            status = vl53lmz_get_ranging_data(&LMZDev, &RangingData);
            for (i = 0; i < 64; i++) {
//				uart_printf("%3d,%3u,%5u,%12u\n",
//					i,
//					RangingData.target_status[i],
//					RangingData.distance_mm[i]  >> 2,
//					RangingData.motion_indicator.motion[sci_config.map_id[i]]);

                // getting minimum distance among all the ranging data
                dist = RangingData.distance_mm[i] >> 2;
                if (RangingData.target_status[i] != 0 && dist < minDist) {
                    minDist = dist;
                }

                // getting maximum motion power among all the ranging data
                motionPower = RangingData.motion_indicator.motion[sci_config.map_id[i]];
                if (maxMotion < motionPower) {
                    maxMotion = motionPower;
                }
            }

            current_time = HAL_GetTick();
            uart_printf("Proximite : %4d\r\n", minDist);

            // if the closest distance is lower than the threshold for the first time in PERIODE_CAN_BUS_AUTOMATIC_MESSAGE, CAN message sent CAN message sent
            if ((uint32_t) minDist <= DIST_THRESHOLD && (uint32_t) prevMinDist > DIST_THRESHOLD) {
                if (current_time - time_previous_near >= PERIODE_FILTRAGE) {
                    uint8_t toSend[4] = { 0xD1, 0x57, 0xA4, 0xCE };
                    can_bus.send(toSend, 4);
                    time_previous_near = current_time;
                }
            }
            prevMinDist = minDist;

            // if the greatest motion power is higher than the threshold for the first time in PERIODE_CAN_BUS_AUTOMATIC_MESSAGE, CAN message sent
            if (maxMotion >= MOTION_THRESHOLD) {  // if motion_detected
                last_motion_time = HAL_GetTick();
                if (is_immobile) {
                    is_immobile = false;
                    uint8_t toSend[4] = { 0x5E, 0xBA, 0x1A, 0xDE };
                    can_bus.send(toSend, 4);
                    //time_previous_movement = current_time;
                    uart_printf("Mouvement : %4d\r\n", maxMotion);
                    //DEBUG_LOG("Mouvement : ");
                }
            } else {
                if (!is_immobile && ((HAL_GetTick() - last_motion_time) > SPD_IMMOBILE_TIME_MS)) {
                    is_immobile = true;
                    uint8_t toSend[4] = { 0x00, 0xAB, 0xA1, 0xED };
                    can_bus.send(toSend, 4);
                    uart_printf("Plus rien ne bouge depuis : %4d\r\n", SPD_IMMOBILE_TIME_MS);
                    //DEBUG_LOG("Immobile depuis %u ms ", IMU_IMMOBILE_TIME_MS);
                }
            }

//            if (maxMotion >= MOTION_THRESHOLD) {
//                uart_printf("Mouvement : %4d\r\n", maxMotion);
//                if (current_time - time_previous_movement >= PERIODE_FILTRAGE) {
//                    //uart_printf("Curr time %8u Prev time %8u Diff %8u\n",current_time,time_previous_movement, current_time-time_previous_movement);
//                    uint8_t toSend[4] = { 0x5E, 0xBA, 0x1A, 0xDE, };
//                    can_bus.send(toSend, 4);
//                    time_previous_movement = current_time;
//                }
//            } else {
//                uart_printf("Rien ne bouge : %4d\r\n", maxMotion);
//            }
        }

        HAL_Delay(5);

        // Ecriture message CAN de vie (pour vérifier que le can fonctionne)
        if (current_time >= time_for_can_bus_automatic_message) {
            uint8_t toSend[8] = { 0x04, 0x74, 0xC0, 0xFF, 0xEE, 0xBA, 0xDB, 0xAD };
            can_bus.send(toSend, 8);
            time_for_can_bus_automatic_message += PERIODE_CAN_BUS_AUTOMATIC_MESSAGE;
        }

        // Modification état led debug (pour vérifier que l'application fonctionne)
        if (current_time >= time_for_change_led_state) {
            time_for_change_led_state += change_state_user_led();
        }
    }
}

// end of file
