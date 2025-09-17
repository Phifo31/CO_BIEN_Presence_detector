/*
 * application.h
 *
 *  Created on: Sep 16, 2025
 *      Author: phifo
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_

#define VL53LMZ_SCI_PULSE_WIDTH_BIN                 8
#define VL53LMZ_SCI_BIN_WIDTH_CM                    3.75348
#define VL53LMZ_SCI_STORE_PREV_F                    1
#define VL53LMZ_SCI_FEATURE_ELT_BYTE                4
#define VL53LMZ_SCI_STORE_VAR                       1
#define VL53LMZ_SCI_PADDING_BYTES                   8
#define VL53LMZ_SCI_PARTIALS_BYTES                  8
#define VL53LMZ_SCI_MAX_BUFFER_SIZE                 6152


void application_setup (void);
void application_loop (void);


#endif /* APPLICATION_H_ */
