################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STSW_IMG048/app_comm.c \
../STSW_IMG048/comms_encode.c \
../STSW_IMG048/sensor_bsp.c \
../STSW_IMG048/sensor_command.c \
../STSW_IMG048/uart_trace.c \
../STSW_IMG048/vl53lmz_api.c \
../STSW_IMG048/vl53lmz_plugin_detection_thresholds.c \
../STSW_IMG048/vl53lmz_plugin_infos.c \
../STSW_IMG048/vl53lmz_plugin_xtalk.c 

C_DEPS += \
./STSW_IMG048/app_comm.d \
./STSW_IMG048/comms_encode.d \
./STSW_IMG048/sensor_bsp.d \
./STSW_IMG048/sensor_command.d \
./STSW_IMG048/uart_trace.d \
./STSW_IMG048/vl53lmz_api.d \
./STSW_IMG048/vl53lmz_plugin_detection_thresholds.d \
./STSW_IMG048/vl53lmz_plugin_infos.d \
./STSW_IMG048/vl53lmz_plugin_xtalk.d 

OBJS += \
./STSW_IMG048/app_comm.o \
./STSW_IMG048/comms_encode.o \
./STSW_IMG048/sensor_bsp.o \
./STSW_IMG048/sensor_command.o \
./STSW_IMG048/uart_trace.o \
./STSW_IMG048/vl53lmz_api.o \
./STSW_IMG048/vl53lmz_plugin_detection_thresholds.o \
./STSW_IMG048/vl53lmz_plugin_infos.o \
./STSW_IMG048/vl53lmz_plugin_xtalk.o 


# Each subdirectory must supply rules for building sources it contributes
STSW_IMG048/%.o STSW_IMG048/%.su STSW_IMG048/%.cyclo: ../STSW_IMG048/%.c STSW_IMG048/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Application -I../STSW_IMG048 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-STSW_IMG048

clean-STSW_IMG048:
	-$(RM) ./STSW_IMG048/app_comm.cyclo ./STSW_IMG048/app_comm.d ./STSW_IMG048/app_comm.o ./STSW_IMG048/app_comm.su ./STSW_IMG048/comms_encode.cyclo ./STSW_IMG048/comms_encode.d ./STSW_IMG048/comms_encode.o ./STSW_IMG048/comms_encode.su ./STSW_IMG048/sensor_bsp.cyclo ./STSW_IMG048/sensor_bsp.d ./STSW_IMG048/sensor_bsp.o ./STSW_IMG048/sensor_bsp.su ./STSW_IMG048/sensor_command.cyclo ./STSW_IMG048/sensor_command.d ./STSW_IMG048/sensor_command.o ./STSW_IMG048/sensor_command.su ./STSW_IMG048/uart_trace.cyclo ./STSW_IMG048/uart_trace.d ./STSW_IMG048/uart_trace.o ./STSW_IMG048/uart_trace.su ./STSW_IMG048/vl53lmz_api.cyclo ./STSW_IMG048/vl53lmz_api.d ./STSW_IMG048/vl53lmz_api.o ./STSW_IMG048/vl53lmz_api.su ./STSW_IMG048/vl53lmz_plugin_detection_thresholds.cyclo ./STSW_IMG048/vl53lmz_plugin_detection_thresholds.d ./STSW_IMG048/vl53lmz_plugin_detection_thresholds.o ./STSW_IMG048/vl53lmz_plugin_detection_thresholds.su ./STSW_IMG048/vl53lmz_plugin_infos.cyclo ./STSW_IMG048/vl53lmz_plugin_infos.d ./STSW_IMG048/vl53lmz_plugin_infos.o ./STSW_IMG048/vl53lmz_plugin_infos.su ./STSW_IMG048/vl53lmz_plugin_xtalk.cyclo ./STSW_IMG048/vl53lmz_plugin_xtalk.d ./STSW_IMG048/vl53lmz_plugin_xtalk.o ./STSW_IMG048/vl53lmz_plugin_xtalk.su

.PHONY: clean-STSW_IMG048

