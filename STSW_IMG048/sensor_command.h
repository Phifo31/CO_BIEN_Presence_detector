/**
  ******************************************************************************
  * @file    sensor_command.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSOR_CMD_H
#define __SENSOR_CMD_H


#ifdef __cplusplus
extern "C" {
#endif 

#include <stdint.h>
#define MAX_COMMAND_BUFFER_SIZE 776
typedef struct {
	int enable;
	int disable;
	int off;
	int set;
	int get_caldata;
	int set_caldata;
	uint8_t buffer[MAX_COMMAND_BUFFER_SIZE];
	uint16_t buffer_size;
	int calibrate;
	int get_calstatus;
	int RangeLoggingLevel;
	int DebugLoggingLevel;
	int SCILoggingLevel;
	int SPDLoggingLevel;
	int EncodedDevLoggingLevel;
} CommandData_t;

/**
 * Parameter parser description
 *
 * give scanner/formatter ptr etc ...
 */
typedef struct SetParam_t * pSetParam_t;

typedef int (*ScannerType)(const char* Buff, const char *fmt, ...);
typedef int (*CheckerType)(const pSetParam_t SetParam, const void *Param);
typedef char* (*PrinterType)(char* buffer, const pSetParam_t SetParam, const uint8_t *Param);

struct SetParam_t {
    const char *Name;     // Parameter name
    ScannerType Scanner;  //!< if NULL, default to sscanf
    char *ScanFmt;        //!< scanner argument ptr ie sscanf fmt string. If NULL, default to "%d"
    int ParamOffset;      //!<offset in @ref DevParams_t to param
    CheckerType Checker;  //!< checker : if null return nob 0 form scanner as validation
    PrinterType Printer;  //!< printer : print name=value
    int size;
};

#define SPD_INTEGRATION_TIME_L8 10
#define SPD_INTEGRATION_TIME_L7 40
#define SPD_AUTONOMOUS_INTEGRATION_TIME_L8 2
#define SPD_AUTONOMOUS_INTEGRATION_TIME_L7 20
#define SPD_RANGING_PERIOD_L8 100
#define SPD_RANGING_PERIOD_L7 200
struct Params_t {
	int Resolution;
	int RangingPeriod;
	int IntegrationTime;
	int InterruptMode;
	int SharpenerEnable;
	int InterruptTogglingTimeInUs;
	int Xtalk16CalibRangingRate;
	int Xtalk64CalibRangingRate;
	int XtalkMargin;
	uint8_t disableCP;
	uint8_t auto_stop;
	int ClosestFirst;
	int EnableVCSELBoost;
	int sciDmin;
	int sciDmax;
	int sciSumSpan;
	int sciAggNb;
	int sciTemporalAgg;
	int sciDetectionThreshold;
	int sciMemUpdateMode;
	int disablePipe;
	uint8_t AutonomousActiveZonesMap[64];
	uint8_t spdEnable;
	int spdSensorRotation;
	int spdResolution;
	int spdRangingPeriod;
	int spdIntegrationTime;
	int spdAutonomousRangingPeriod;
	int spdAutonomousIntegrationTime;
	int spdAutonomousInterruptMode;
	int spdInvalidDistance_mm;
	int spdApproachDistance_mm;
	int spdWakeUpDistance_mm;
	int spdLockDistance_mm;
	int spdApproachTimeout_ms;
	int spdTimeToAutonomous_ms;
	int spdObstructedFoVDistThreshold;
	int spdStaticCloseObjectMaxDistance;
	int spdLockTimeAfterTrkLost_ms;
	int spdNoMotionTimeOut_ms;
	int spdLockTimeObstructedFoV_ms;
	int spdLockTimeStaticCloseObject_ms;
	int spdLockTimeNoTarget_ms;
	int spdMinimumLockTime_ms;
	float spdWakeOnStopMinSpeed_kmh;
	int spdMinXSpeedDeparture;
	uint8_t spdActiveZonesMap[64];
	uint8_t spdAutonomousActiveZonesMap[64];
	int itaMaxRightDistance_mm;
	int itaMaxLeftDistance_mm;
	int itaTimeToTriggerAlert_ms;
	int itaTimeToMaintainAlert_ms;
};


int SC_HandleCmd(const char *Buffer);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_CMD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
