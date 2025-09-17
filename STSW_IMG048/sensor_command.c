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

#include <string.h>
#include <stddef.h>
#include <ctype.h>

#include "sensor_bsp.h"
//#include "stm32xxx_hal.h"
#include "sensor_command.h"

#include "main.h"
#include "spd.h"
#include "vl53lmz_api.h"

extern VL53LMZ_Configuration LMZDev;

CommandData_t CommandData = {	.enable=0,
								.disable=0,
								.off=0,
								.set=0,
								.get_caldata=0,
								.set_caldata=0,
								.get_calstatus=0,
								.buffer_size=0,
								.RangeLoggingLevel=0,
								.DebugLoggingLevel=0,
								.SCILoggingLevel=0,
								.SPDLoggingLevel=1,
								.EncodedDevLoggingLevel=0,
};

/**
 * ARRAY size of a static declared array
 */
#ifndef ARRAY_SIZE
#   define ARRAY_SIZE(a)  ((sizeof(a) / sizeof(a[0])))
#endif



static const char StrCmdOk[]="ok\n";


/**
 * Parameters exposed in the command protocol with their init values : "set param=value"
 * This structure is used in main.c
 * Params_t is defined in sensor_command.h
 * Add a new field in this structure and in the variable below to add a new parameter that can be controlled by the "set param=value" command
 */
struct Params_t Params = {
						.Resolution=64,
						.RangingPeriod=250,
						.IntegrationTime=10,  //To be increased for the L7 to get similar performances
						.InterruptMode=0,
						.SharpenerEnable=1,
						.InterruptTogglingTimeInUs=1000,
						.Xtalk16CalibRangingRate=150,
						.Xtalk64CalibRangingRate=20,
						.XtalkMargin=50,
						.sciDmin=SPD_SCI_DEFAULT_DCIMIN,
						.sciDmax=SPD_SCI_DEFAULT_DCIMAX,
						.sciSumSpan=4,
						.sciAggNb=16,
						.sciTemporalAgg=16,
						.sciDetectionThreshold=44,
						.sciMemUpdateMode=6,
						.disablePipe=0,
						.auto_stop=0,
						.spdSensorRotation = 0,
						.spdResolution = 64,
						.spdRangingPeriod = SPD_RANGING_PERIOD_L8,	// <=> 10Hz   -----  Low Power proposal -> 250 <=> 4 Hz // 7 value will be set in the main function if the sensor is detected
						.spdIntegrationTime = SPD_INTEGRATION_TIME_L8, //L7 value will be set in the main function if the sensor is detected
						.spdAutonomousRangingPeriod = 250, // <=> 4Hz   -----  Low Power proposal -> 1000 <=> 1 Hz
						.spdAutonomousIntegrationTime = SPD_AUTONOMOUS_INTEGRATION_TIME_L8, //L7 value will be set in the main function if the sensor is detected
						.spdAutonomousInterruptMode = 1,
						.spdInvalidDistance_mm = SPD__INVALID_DISTANCE_MM,
						.spdApproachDistance_mm = SPD__APPROACH_DISTANCE_MM,
						.spdWakeUpDistance_mm = SPD__WAKEUP_DISTANCE_MM,
						.spdObstructedFoVDistThreshold = SPD__OBSTRUCTED_FOV_DIST_THRESHOLD,
						.spdStaticCloseObjectMaxDistance = SPD__STATIC_CLOSE_OBJ_MAX_DISTANCE_MM,
						//To simplify the SPD usage, all timings to detect a departures are set to 1, so the algo will use spdMinimumLockTime_ms
						.spdLockTimeAfterTrkLost_ms = 1, //SPD__LOCK_TIME_AFTER_TRK_LOST_MS,
						.spdLockTimeObstructedFoV_ms = 1, //SPD__LOCK_TIME_OBSTRUCTED_FOV_MS,
						.spdLockTimeStaticCloseObject_ms = 1, //SPD__LOCK_TIME_STATIC_CLOSE_OBJ_MS,
						.spdLockTimeNoTarget_ms = 1, //SPD__LOCK_TIME_NO_TARGET_MS,
						.spdMinimumLockTime_ms = SPD__MININMUM_LOCK_TIME_MS,
						.spdNoMotionTimeOut_ms = SPD__NO_MOTION_TIMEOUT,
						.spdTimeToAutonomous_ms = SPD__TIME_TO_AUTONOMOUS_MS,
						.spdWakeOnStopMinSpeed_kmh = SPD__WAKE_ON_STOP_MIN_SPEED,
						.spdMinXSpeedDeparture = TRK__MIN_X_SPEED_DEPARTURE,
						.spdActiveZonesMap = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
						.itaMaxRightDistance_mm = ITA_MAX_RIGHT_DISTANCE,
						.itaMaxLeftDistance_mm = ITA_MAX_LEFT_DISTANCE,
						.itaTimeToTriggerAlert_ms = ITA_TIME_TO_TRIGGER_ALERT,
						.itaTimeToMaintainAlert_ms = ITA_TIME_TO_MAINTAIN_ALERT,

};

/**
 * List of parameters that can be changed by the "set param=value" command
 * This list is used by the Parse_SET() parser function (to be more generic)
 * Add a new entry in this list to expose a new parameter to the command parser
 */
struct SetParam_t SetableParams[]={
		{ .Name= "Xtalk16CalibRangingRate",					.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%d",   .ParamOffset = offsetof(struct Params_t, Xtalk16CalibRangingRate), .size=1 },
		{ .Name= "Xtalk64CalibRangingRate",					.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%d",   .ParamOffset = offsetof(struct Params_t, Xtalk64CalibRangingRate), .size=1 },
		{ .Name= "XtalkMargin",								.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%d",   .ParamOffset = offsetof(struct Params_t, XtalkMargin), .size=1 },
		{ .Name= "spdSensorRotation",						.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%d",   .ParamOffset = offsetof(struct Params_t, spdSensorRotation), .size=1 },
		{ .Name= "spdResolution",							.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%d",   .ParamOffset = offsetof(struct Params_t, spdResolution), .size=1 },
		{ .Name= "spdRangingPeriod",						.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%d",   .ParamOffset = offsetof(struct Params_t, spdRangingPeriod), .size=1 },
		{ .Name= "spdIntegrationTime",						.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%d",   .ParamOffset = offsetof(struct Params_t, spdIntegrationTime), .size=1 },
		{ .Name= "spdAutonomousRangingPeriod",				.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%d",   .ParamOffset = offsetof(struct Params_t, spdAutonomousRangingPeriod), .size=1 },
		{ .Name= "spdAutonomousIntegrationTime",			.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%d",   .ParamOffset = offsetof(struct Params_t, spdAutonomousIntegrationTime), .size=1 },
// Keep this param secret but can be set for the EVK logging
		{ .Name= "spdAutonomousInterruptMode",				.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%u",   .ParamOffset = offsetof(struct Params_t, spdAutonomousInterruptMode), .size=1 },
		{ .Name= "spdApproachDistance_mm",					.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%d",   .ParamOffset = offsetof(struct Params_t, spdApproachDistance_mm), .size=1 },
		{ .Name= "spdWakeUpDistance_mm",					.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%d",   .ParamOffset = offsetof(struct Params_t, spdWakeUpDistance_mm), .size=1 },
		{ .Name= "spdMinimumLockTime_ms",					.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%d",   .ParamOffset = offsetof(struct Params_t, spdMinimumLockTime_ms), .size=1 },
		{ .Name= "spdNoMotionTimeOut_ms",					.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%d",   .ParamOffset = offsetof(struct Params_t, spdNoMotionTimeOut_ms), .size=1 },
		{ .Name= "spdTimeToAutonomous_ms",					.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%d",   .ParamOffset = offsetof(struct Params_t, spdTimeToAutonomous_ms), .size=1 },
		{ .Name= "spdWakeOnStopMinSpeed_kmh",				.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%f",   .ParamOffset = offsetof(struct Params_t, spdWakeOnStopMinSpeed_kmh), .size=1 },
		{ .Name= "itaMaxRightDistance_mm",					.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%d",   .ParamOffset = offsetof(struct Params_t, itaMaxRightDistance_mm), .size=1 },
		{ .Name= "itaMaxLeftDistance_mm",					.Scanner= (int(*)(const char*, const char *,...))sscanf,    .ScanFmt="%d",   .ParamOffset = offsetof(struct Params_t, itaMaxLeftDistance_mm), .size=1 },
};

char UartComm_TmpBuffer[512];

#define ParseError(fmt, ...) _ParseError ("ERR %s "fmt "\n",  __func__, ##__VA_ARGS__)

static void _ParseError( const char *fmt, ...){
    va_list ap;
    va_start(ap,fmt);
    SB_vprintf(fmt, ap);
    SB_VDebug(fmt,ap);
}

struct BaseCommand_t{
    const char *Name;
    int (*Parse)(const struct BaseCommand_t *pCmd, const char *Buffer); /** !< parser is invoked with the command and string right after the command itself*/
    const char *Help;
    const char *Syntax;
    const char *Example;
    int NoAnswer; /**!< when set , successful command do not issue "ok"
                    use it for command that always echo back some answer bad or not */
};

static int Parse_Enable( const struct BaseCommand_t *pCmd, const char *Buffer){
	(void)pCmd;
	(void)Buffer;
	CommandData.enable = 1;
    return 0;
}

static int Parse_Disable( const struct BaseCommand_t *pCmd, const char *Buffer){
	(void)pCmd;
	(void)Buffer;
	CommandData.disable = 1;
	return 0;
}

static int Parse_Off( const struct BaseCommand_t *pCmd, const char *Buffer){
	(void)pCmd;
	(void)Buffer;
	CommandData.off = 1;
	return 0;
}

static int Parse_Name( const struct BaseCommand_t *pCmd, const char *Buffer){
	(void)pCmd;
	(void)Buffer;
	uart_printf("Name : SmartPresenceDetection_ULD\n");
    return 0;
}

static int Parse_Version( const struct BaseCommand_t *pCmd, const char *Buffer){
	(void)pCmd;
	(void)Buffer;
	uart_printf("version %s\n",SPD_KIT_ULD_VERSION);
    return 0;
}

static int Parse_Module( const struct BaseCommand_t *pCmd, const char *Buffer){
	(void)pCmd;
	(void)Buffer;
	uart_printf("%s\n", LMZDev.module_type==2 ? "VL53L8CX" : (LMZDev.module_type==1 ? "VL53L7CX" : (LMZDev.module_type==0 ? "VL53L5CX" :"Unknown")));
    return 0;
}

static int Parse_SPDVersion( const struct BaseCommand_t *pCmd, const char *Buffer){
	(void)pCmd;
	(void)Buffer;
	uart_printf("version %s\n",SPD_VERSION);
    return 0;
}

static int Parse_ApiVer( const struct BaseCommand_t *pCmd, const char *Unused){
	(void)pCmd;
	(void)Unused;
	uart_printf("Bare Driver %s\n",VL53LMZ_API_REVISION);
	return 0;
}

static int Parse_FwVer( const struct BaseCommand_t *pCmd, const char *Unused){
	(void)pCmd;
	(void)Unused;
	//DBG_YL
//	VL53LMZ_FWVersion fw_version;
//	vl53lmz_get_fw_version(&LMZDev, &fw_version);
//	uart_printf("FW Version : %d.%d.%d.%d\n", fw_version.fw_major, fw_version.fw_minor, fw_version.fw_build, fw_version.fw_revision);
	return 0;
}

static int Parse_Params( const struct BaseCommand_t *pCmd, const char *Buffer){
    int i,j;
    char *TmpStr;
    int n;
    uart_printf("size : %d\n",ARRAY_SIZE(SetableParams) - 1); // -1 -> Hide "spdAutonomousInterruptMode" because it's useless for the user, only used by the SPD EVK

    for( i=0; i<ARRAY_SIZE(SetableParams); i++){
    	if(strcmp(SetableParams[i].Name, "spdAutonomousInterruptMode") == 0){ //Hide "spdAutonomousInterruptMode" because it's useless for the user, only used by the SPD EVK
    			    		continue;
    			    	}
    	TmpStr=UartComm_TmpBuffer;
        uint8_t *ParamsPtr =(uint8_t*)&Params;
        ParamsPtr+=SetableParams[i].ParamOffset;

        if( SetableParams[i].Scanner==sscanf){
            sprintf(TmpStr, "%s=",SetableParams[i].Name);
            TmpStr=strstr(TmpStr,"=")+1; // move passed skip argname=
            // Go through each element (comma separated) if size is not 1
            for(j=0;j<SetableParams[i].size;j++){
				// Separate each element by a comma
            	if (j!=0) {
					sprintf(TmpStr,",");
					TmpStr++;
				}
            	// Manage data types : only %f (4 bytes), %d (4 bytes), %u (1 byte) supported
            	if( SetableParams[i].ScanFmt[1]=='f'){
					n = sprintf(TmpStr, SetableParams[i].ScanFmt, *((float *)ParamsPtr));
					ParamsPtr = ParamsPtr+4;
				}
            	else if( SetableParams[i].ScanFmt[1]=='u'){
					n = sprintf(TmpStr, SetableParams[i].ScanFmt, *((uint8_t *)ParamsPtr));
					ParamsPtr = ParamsPtr+1;
				}
				else{
					n = sprintf(TmpStr, SetableParams[i].ScanFmt, *((uint32_t *)ParamsPtr));
					ParamsPtr = ParamsPtr+4;
				}
				TmpStr = TmpStr+n;
            }
            sprintf(TmpStr, "\n");
            uart_printf(UartComm_TmpBuffer);
        }
    }
    return 0;
}

int Parse_SET(const struct BaseCommand_t *pCmd, const char *Buffer){
    int n;
    char *Find;
    char *ParamValue;
    int Status=-1;
    uint8_t *ParamsPtr;
    int i,j;

    Find=strchr(Buffer, '=');
    if( Find == NULL ){
    	ParseError("Missing = ");
        goto done;
    }
    ParamValue = Find +1 ; // what follow =
    // look for setable param name

    for( i=0; i< ARRAY_SIZE(SetableParams); i++){
        int ParamLen;
        ParamLen=strlen(SetableParams[i].Name);
        if( strncmp(Buffer+1,SetableParams[i].Name, ParamLen) == 0 ){
            // ok we have it
        	ParamsPtr =(uint8_t*)&Params;
        	ParamsPtr+=SetableParams[i].ParamOffset;
        	// Go through each element (comma separated) if size is not 1
        	for(j=0;j<SetableParams[i].size;j++){
        		if (j!=0) {
        			// Search for the next comma
        			Find=strchr(ParamValue, ',');
        			if( Find == NULL ){
						ParseError("Missing , ");
						goto done;
					}
        			// Jump to the string just after the comma
					ParamValue = Find +1 ;
				}
				n = SetableParams[i].Scanner( ParamValue, SetableParams[i].ScanFmt, ParamsPtr );
				if( SetableParams[i].Checker != NULL ){
					//dedicate checker
					Status=SetableParams[i].Checker( &SetableParams[i], ParamsPtr );
				}
				else{
					// simply use the n retrun
					if( n==1 ){
						CommandData.set = 1;
						Status = 0;
					}
					else{
						ParseError("fail: top decode argument value");
					}
				}
				// Manage data types : only %f (4 bytes), %d (4 bytes), %u (1 byte) supported
				if( SetableParams[i].ScanFmt[1]=='f'){
					ParamsPtr += 4;
				}
				else if( SetableParams[i].ScanFmt[1]=='u'){
					ParamsPtr += 1;
				}
				else{
					ParamsPtr += 4;
				}
        	}
            // done we have find params
            break;
        }
    }
    // parama name not found
    if( i >= ARRAY_SIZE(SetableParams) ){
    	ParseError("unknown param name");
    }
done:
    return Status;
}


static int Parse_Log(const struct BaseCommand_t *pCmd, const char *Buffer){
    int n, Value;
    int Cmd=-1; // 0:valid and -1 not known
    char *TmpBuffer;

    (void)pCmd;
    TmpBuffer = SB_TmpBuffer();
    n=sscanf(Buffer, "%s %d", TmpBuffer, &Value);

    if( n<2 ){
        ParseError("%s", "missing arg and/or value");
        return -1;
    }

    if( strcmp(TmpBuffer,"range")==0 )
    {
    	Cmd = 0;
    	CommandData.RangeLoggingLevel = Value;
    }
    else if (strcmp(TmpBuffer,"spd")==0) {
		Cmd=1;
		CommandData.SPDLoggingLevel = Value;
	}
    else if (strcmp(TmpBuffer,"edv")==0) {
    		Cmd=1;
    		CommandData.EncodedDevLoggingLevel = Value;
	}
    else if (strcmp(TmpBuffer,"debug")==0) {
    		Cmd=1;
    		CommandData.DebugLoggingLevel = Value;
	}
    else if (strcmp(TmpBuffer,"sci")==0) {
			Cmd=1;
			CommandData.SCILoggingLevel = Value;
	}

    else
    {  // add here new item to be parse
    }

    // no parsing of target
    if( Cmd == -1 ){
        ParseError("invalid/wrong log target %s", TmpBuffer);
        return -2;
    }

    return  0;
}

#ifdef VL53LMZ_XTALK_CALIBRATION
static int Parse_GetCalData(const struct BaseCommand_t *pCmd, const char *Buffer)
{
	int n;
	int Cmd=-1; // 0 = valid
	char *TmpBuffer;

	(void)pCmd;
	TmpBuffer = SB_TmpBuffer();
	n=sscanf(Buffer, "%s", TmpBuffer);

	if( n<1 ){
		uart_printf("%s", "missing arg\n");
		return -1;
	}

	if( strcmp(TmpBuffer,"xtalk")==0 )
	{
		Cmd = 0;
		CommandData.get_caldata = 1;
	}
	else
	{  // add here new item to be parse
	}

	// no parsing of target
	if( Cmd == -1 ){
		ParseError("invalid/wrong calibration data type (%s) : only xtalk supported", TmpBuffer);
		return -2;
	}

	return  0;
}

static int Parse_SetCalData(const struct BaseCommand_t *pCmd, const char *Buffer)
{
	int n;
	int Cmd=-1; // 0 = valid
	const char *stringPtr;
	uint8_t *dataPtr;
	int offset=0;
	unsigned int data1, data2, data3, data4;

	(void)pCmd;

	if( strncmp(Buffer+1,"xtalk64",strlen("xtalk64"))==0 )
	{
		Cmd = 0;
		offset = strlen("xtalk64")+1;
		CommandData.set_caldata = 1;
	}
	else
	{  	ParseError("invalid/wrong/missing calibration data type (%s) : only xtalk64 supported");
		return Cmd;
	}

	stringPtr = Buffer+offset+1;
	dataPtr = CommandData.buffer;
	CommandData.buffer_size = 0;
	while (stringPtr != NULL){
		n = sscanf(stringPtr, "%2x%2x%2x%2x", &data1, &data2, &data3, &data4);
		if (n != 4) {
			ParseError("invalid/wrong/missing calibration data format : format must be 12345678,deaddead");
			CommandData.set_caldata = 0;
			Cmd = -1;
			break;
		}
		if (CommandData.buffer_size+4 > MAX_COMMAND_BUFFER_SIZE){
			ParseError("calibration data size exceeds max buffer size : %d", MAX_COMMAND_BUFFER_SIZE);
			CommandData.set_caldata = 0;
			Cmd = -1;
			break;
		}
		*dataPtr = data1;
		*(dataPtr+1) = data2;
		*(dataPtr+2) = data3;
		*(dataPtr+3) = data4;
		dataPtr += 4;
		CommandData.buffer_size += 4;
		stringPtr = strchr(stringPtr,',');
		if (stringPtr != NULL)
			stringPtr++;
	}

	return  Cmd;
}

static int Parse_Calibrate(const struct BaseCommand_t *pCmd, const char *Buffer)
{
	int n;
	int Cmd=-1; // 0 = valid
	char *TmpBuffer;

	(void)pCmd;
	TmpBuffer = SB_TmpBuffer();
	n=sscanf(Buffer, "%s", TmpBuffer);

	if( n<1 ){
		uart_printf("%s", "missing arg\n");
		return -1;
	}

	if( strcmp(TmpBuffer,"xtalk")==0 )
	{
		Cmd = 0;
		CommandData.calibrate = 1;
	}
	else
	{  // add here new item to be parse
	}

	// no parsing of target
	if( Cmd == -1 ){
		ParseError("invalid/wrong calibration data type (%s) : only xtalk", TmpBuffer);
		return -2;
	}

	return  0;
}
#endif
static int Parse_GetCalStatus(const struct BaseCommand_t *pCmd, const char *Buffer)
{
	CommandData.get_calstatus = 1;
	return  0;
}

static int Parse_Help( const struct BaseCommand_t *pCmd, const char *Unused);

struct BaseCommand_t BaseCmd[]={
		{ .Name= "name", .Parse = Parse_Name,
						.Help="Give f/w name",
						.NoAnswer = 1,
		},
		{ .Name= "version", .Parse = Parse_Version,
						.Help="Give f/w version",
						.NoAnswer = 1,
		},
		{ .Name= "module", .Parse = Parse_Module,
								.Help="Give module type",
								.NoAnswer = 1,
		},
		{ .Name= "spdver", .Parse = Parse_SPDVersion,
						.Help="give SPD algo version",
						.NoAnswer = 1,
		},
		{ .Name= "apiver", .Parse = Parse_ApiVer,
				.Help="Give VL53LMZ API (Bare Driver) version.",
				.NoAnswer = 1,
		},
		{ .Name= "fwver", .Parse = Parse_FwVer,
						.Help="Give VL53LMZ FW version.",
						.NoAnswer = 1,
		},
		{ .Name= "enable", .Parse = Parse_Enable,
				.Help="Enable sensor ranging.",
				.Syntax="'enable'",
				.Example="'enable' => Enable sensor ranging"
		},
		{ .Name= "disable", .Parse = Parse_Disable,
				.Help="Disable sensor ranging.",
				.Syntax="'disable'",
				.Example="'disable' => Disable sensor ranging",
		},
		{ .Name= "off", .Parse = Parse_Off,
				.Help="Set sensor in ULP mode (Ultra Low Power).",
				.Syntax="'off'",
				.Example="'off' => Disable sensor",
		},
#ifdef VL53LMZ_XTALK_CALIBRATION
		{ .Name= "get_caldata", .Parse = Parse_GetCalData,
				.Help="Gets Calibration data from the device",
				.Syntax="'get_caldata xtalk'",
				.NoAnswer = 1
		},
		{ .Name= "set_caldata", .Parse = Parse_SetCalData,
				.Help="Sets Calibration data to the device",
				.Syntax="'set_caldata xtalk64 value0,value1,...,valueN'",
		},
		{ .Name= "calibrate", .Parse = Parse_Calibrate,
				.Help="Calibrate the device : only xtalk supported",
				.Syntax="'calibrate xtalk'",
				.NoAnswer = 1,
		},
#endif
		{ .Name= "get_calstatus", .Parse = Parse_GetCalStatus,
						.Help="Gets Calibration status",
						.Syntax="'get_calstatus'",
						.NoAnswer = 1,
		},
		{ .Name= "params", .Parse = Parse_Params,
				.Help="Show all input parameters",
				.Syntax="'params'",
				.Example="'params' => Show all input parameters",
				.NoAnswer = 1,
		},
		{ .Name= "set", .Parse = Parse_SET,
				  .Help="Set a parameter. See params command to know available parameters and current values.",
				  .Syntax="'set param_name=value'",
				  .Example="'set RangingPeriod=1'",
		},
		{ .Name= "log", .Parse = Parse_Log,
				.Help="Enable/Disable spd data logging and set logging level.",
				.Syntax="'log spd x'",
				.Example="'log spd 2' => Enable range logging on zone 1",
				.NoAnswer=0,
		},
        { .Name= "help", .Parse = Parse_Help,
        		.Help="Displays this help",
                .NoAnswer = 1,
        },



};



static int Parse_Help( const struct BaseCommand_t *pCmd, const char *Unused){
    size_t i;
    char *TmpBuffer;

    (void)pCmd;
    (void)Unused;
    TmpBuffer = SB_TmpBuffer();

    for(i=0; i<ARRAY_SIZE(BaseCmd);i++){
    	TmpBuffer[0]=0;    //star clean string
        strcat(TmpBuffer, BaseCmd[i].Name);
        if( BaseCmd[i].Help != NULL  ){
            strcat(TmpBuffer, "\t");
            strcat(TmpBuffer, BaseCmd[i].Help);
        }
        if( BaseCmd[i].Syntax != NULL  ){
            strcat(TmpBuffer, "\n\tSyntax:\t");
            strcat(TmpBuffer, BaseCmd[i].Syntax);
        }
        if( BaseCmd[i].Example != NULL  ){
			strcat(TmpBuffer, "\n\tExample:\t");
			strcat(TmpBuffer, BaseCmd[i].Example);
		}
        strcat(TmpBuffer, "\n");
        uart_printf(TmpBuffer);
    }
    return 0;
}


int SC_HandleCmd(const char *Buffer){
    int Status =-1;
    int CmdLen;
    int CmdDecoded=0;
    size_t i;

    for( i=0; i<ARRAY_SIZE(BaseCmd);i++){
        int CmdRet;
        CmdLen=strlen(BaseCmd[i].Name);
        if( strncmp( Buffer, BaseCmd[i].Name, CmdLen) == 0 &&
                (Buffer[CmdLen]==' ' || Buffer[CmdLen]=='\t' || Buffer[CmdLen]==0 )
          ){
            CmdRet=BaseCmd[i].Parse(&BaseCmd[i], Buffer+CmdLen);
            if( CmdRet == 0 ){
                Status =0;
                if( BaseCmd[i].NoAnswer == 0 )
                	uart_printf(StrCmdOk);
            }
            CmdDecoded=1;
            break;
        }
        // set command
    }
    if( CmdDecoded == 0 ){
        // TODO command not decoded ?
    	ParseError("unknown command %s", Buffer);
    }
    return Status;
}
