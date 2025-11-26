/*
 * application.h
 *
 *  Created on: Sep 16, 2025
 *      Author: phifo
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_

#define ENABLE_USER_LOG   1
#define ENABLE_DEBUG_LOG  0

#define VL53LMZ_SCI_PULSE_WIDTH_BIN                 8
#define VL53LMZ_SCI_BIN_WIDTH_CM                    3.75348
#define VL53LMZ_SCI_STORE_PREV_F                    1
#define VL53LMZ_SCI_FEATURE_ELT_BYTE                4
#define VL53LMZ_SCI_STORE_VAR                       1
#define VL53LMZ_SCI_PADDING_BYTES                   8
#define VL53LMZ_SCI_PARTIALS_BYTES                  8
#define VL53LMZ_SCI_MAX_BUFFER_SIZE                 6152


void application_setup (void);
int application_loop (void);


// Sortie texte

extern uint16_t seconds;
extern uint16_t reste;
uint16_t msec2sec(uint32_t n, uint16_t *reste);

#if ENABLE_USER_LOG
#define USER_LOG(fmt, ...) ({ \
            seconds = msec2sec(HAL_GetTick(), &reste); \
            printf("%05u:%03u - ", seconds, reste); \
            printf("[USER] " fmt "\r\n", ##__VA_ARGS__); })
#else
  #define USER_LOG(fmt, ...)
#endif

#if ENABLE_DEBUG_LOG
#define DEBUG_LOG(fmt, ...) ({ \
            seconds = msec2sec(HAL_GetTick(), &reste); \
            printf("%05u:%03u - ", seconds, reste); \
            printf("[DEBUG] " fmt "\r\n", ##__VA_ARGS__); })
#else
  #define DEBUG_LOG(fmt, ...) UNUSED(fmt)
#endif



#endif /* APPLICATION_H_ */
