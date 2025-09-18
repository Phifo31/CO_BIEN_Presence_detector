/*******************************************************************************
 *
 * Copyright (c) 2020 STMicroelectronics - All Rights Reserved
 *
 * License terms: STMicroelectronics Proprietary in accordance with licensing
 * terms at www.st.com/sla0081
 *
 * STMicroelectronics confidential
 * Reproduction and Communication of this document is strictly prohibited unless
 * specifically authorized in writing by STMicroelectronics.
 *
 *******************************************************************************/

#include "platform.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

uint8_t RdByte(VL53LMZ_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value) {
    uint8_t status = 0;
    uint8_t data_write[2];
    uint8_t data_read[1];

    data_write[0] = (RegisterAdress >> 8) & 0xFF;
    data_write[1] = RegisterAdress & 0xFF;
    status = HAL_I2C_Master_Transmit(&hi2c1, p_platform->address, data_write, 2, 100);
    status = HAL_I2C_Master_Receive(&hi2c1, p_platform->address, data_read, 1, 100);
    *p_value = data_read[0];
    //uart_printf("read 1 byte\n");
    return status;
}

uint8_t WrByte(VL53LMZ_Platform *p_platform, uint16_t RegisterAdress, uint8_t value) {
    uint8_t data_write[3];
    uint8_t status = 0;

    data_write[0] = (RegisterAdress >> 8) & 0xFF;
    data_write[1] = RegisterAdress & 0xFF;
    data_write[2] = value & 0xFF;
    status = HAL_I2C_Master_Transmit(&hi2c1, p_platform->address, data_write, 3, 100);
    //uart_printf("write 1 byte\n");
    return status;
}

uint8_t WrMulti(VL53LMZ_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
    uint8_t status = HAL_I2C_Mem_Write(&hi2c1, p_platform->address, RegisterAdress,
    I2C_MEMADD_SIZE_16BIT, p_values, size, 65535);
    //uart_printf("write %d bytes\n",size);
    return status;
}

uint8_t RdMulti(VL53LMZ_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
    uint8_t status;
    uint8_t data_write[2];
    data_write[0] = (RegisterAdress >> 8) & 0xFF;
    data_write[1] = RegisterAdress & 0xFF;

    status = HAL_I2C_Master_Transmit(&hi2c1, p_platform->address, data_write, 2, 10);
    status += HAL_I2C_Master_Receive(&hi2c1, p_platform->address, p_values, size, 400);
    //uart_printf("read %d bytes\n",size);

    return status;
}

void Reset_Sensor(VL53LMZ_Platform *p_platform) {
    /* Set 0 to pin LPN */
    HAL_GPIO_WritePin(LPn_C_GPIO_Port, LPn_C_Pin, GPIO_PIN_RESET);
    /* Set 0 to PWR_EN */
    HAL_GPIO_WritePin(PWR_EN_C_GPIO_Port, PWR_EN_C_Pin, GPIO_PIN_RESET);
    WaitMs(p_platform, 100);

    /* Set 1 to pin LPN */
    HAL_GPIO_WritePin(LPn_C_GPIO_Port, LPn_C_Pin, GPIO_PIN_SET);
    /* Set 1 to PWR_EN */
    HAL_GPIO_WritePin(PWR_EN_C_GPIO_Port, PWR_EN_C_Pin, GPIO_PIN_SET);
    WaitMs(p_platform, 100);

}

void SwapBuffer(uint8_t *buffer, uint16_t size) {
    uint32_t i, tmp;

    /* Example of possible implementation using <string.h> */
    for (i = 0; i < size; i = i + 4) {
        tmp = (buffer[i] << 24) | (buffer[i + 1] << 16) | (buffer[i + 2] << 8) | (buffer[i + 3]);

        memcpy(&(buffer[i]), &tmp, 4);
    }
}

uint8_t WaitMs(VL53LMZ_Platform *p_platform, uint32_t TimeMs) {
    HAL_Delay(TimeMs);
    return 0;
}
